"""ROS adapter for mission waypoints. Owns goals only; never publishes thruster forces."""
import json
import time
from pathlib import Path
from collections import deque
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
from .waypoint_mission import Mission, transform
from .visual_approach import select_image_target, world_ray


def resolve_path(value, parent):
    if value.startswith('package://'):
        package, rest = value[len('package://'):].split('/', 1)
        return Path(get_package_share_directory(package))/rest
    path = Path(value)
    return path if path.is_absolute() else parent/path


def pose_transform(pose):
    p, q = pose.position, pose.orientation
    quat = np.array([q.x, q.y, q.z, q.w])
    if not np.isfinite(quat).all() or np.linalg.norm(quat) < 1e-6: raise ValueError('Invalid quaternion')
    T = np.eye(4); T[:3, :3] = Rotation.from_quat(quat).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return transform(T)


class Autonomy(Node):
    def __init__(self):
        super().__init__('auv_autonomy')
        default = Path(get_package_share_directory('auv_mission_control'))/'config/autonomy.yaml'
        self.declare_parameter('config', str(default))
        self.declare_parameter('autostart', False)
        path = Path(self.get_parameter('config').value)
        self.cfg = yaml.safe_load(path.read_text())
        task_path = resolve_path(self.cfg['task_file'], path.parent)
        task = yaml.safe_load(task_path.read_text())
        self.template_path = resolve_path(task['geometry_template'], task_path.parent)
        template = yaml.safe_load(self.template_path.read_text())
        if task['id'] != template['id']: raise ValueError('Task/template IDs differ')
        self.engine = Mission(self.cfg, task, template)
        self.state_history=deque(maxlen=240);self.last_2d_wall=0.
        self.camera_to_body=None
        if self.engine.visual:
            calibration=yaml.safe_load(resolve_path(self.cfg['perception_config'],path.parent).read_text())
            if not calibration['camera_to_body'].get('verified',False):raise ValueError('Unverified camera extrinsic')
            self.camera_to_body=transform(calibration['camera_to_body']['matrix'])
        self.state = None; self.estop = False; self.blocked = False; self.map_epoch = None
        self.reference = None; self.reference_time = None; self.last_status = 0.
        self.last_markers = 0.; self.started_at = time.monotonic(); self.last_stamp = None
        self.pending_hold = False
        self.want_start = self.get_parameter('autostart').value
        self.last_guard = -1e9; self.guard_error = 'discovering nodes'
        self.pub_goal = self.create_publisher(PoseStamped, self.cfg['topics']['goal'], 10)
        self.pub_status = self.create_publisher(String, '/auv/mission/status', 10)
        self.pub_path = self.create_publisher(MarkerArray, '/auv/mission/waypoints',
                                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, self.cfg['topics']['state'], self.on_state, qos_profile_sensor_data)
        self.create_subscription(String, self.cfg['topics']['map'], self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool, '/auv/control/estop', self.on_estop, 10)
        self.create_subscription(String,self.cfg['topics'].get('detections_2d','/auv/perception/detections_2d'),self.on_2d,10)
        for name, callback in [('start', self.start), ('stop', self.stop), ('resume', self.resume)]:
            self.create_service(Trigger, '~/'+name, callback)
        self.create_timer(1/self.cfg['goal_rate_hz'], self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.reset_pid = self.create_client(Trigger, '/pid_controller_node/reset')
        self.get_logger().info('Autonomy ready; task='+task['id']+'; goal ownership guard enabled. No actuator outputs.')

    def state_fresh(self):
        return self.state is not None and time.monotonic()-self.state[0] <= self.cfg['state_timeout_wall_s']

    def on_state(self, msg):
        if msg.header.frame_id != self.cfg['world_frame'] or msg.child_frame_id != 'auv_com': return
        try:
            T = pose_transform(msg.pose.pose)
            v = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
            w = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
            if not np.isfinite(np.r_[v, w]).all(): return
        except ValueError: return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.last_stamp is not None and stamp < self.last_stamp-.001:
            self.engine = Mission(self.cfg, self.engine.task, self.engine.template)
            self.engine.reason = 'simulation reset; call start again'
            self.want_start = False; self.reference = None
            self.pending_hold = True
            self.state_history.clear()
        self.last_stamp = stamp
        self.state = (time.monotonic(), stamp, T, v, w)
        self.state_history.append((stamp,T.copy()))

    def on_2d(self,msg):
        if not self.engine.visual or not self.state_fresh():return
        try:
            data=json.loads(msg.data)
            if data.get('schema_version')!=1 or not data.get('frame_id'):return
            now=self.state[1];stamp=float(data['stamp']);options=self.cfg['visual_approach']
            if not -options['max_future_s']<=now-stamp<=options['max_age_s']:
                self.get_logger().warning('2D timestamp rejected: state-image age %.3f s'%(now-stamp),throttle_duration_sec=5.)
                return
            candidate=select_image_target(data,self.engine.task,options)
            self.last_2d_wall=time.monotonic()
            if candidate is None:
                self.engine.visual.target=None;return
            capture=min(self.state_history,key=lambda x:abs(x[0]-stamp))
            if abs(capture[0]-stamp)>options['capture_pose_tolerance_s']:return
            candidate['stamp']=stamp
            candidate['ray_world']=world_ray(candidate['center'],data['projection'],self.camera_to_body,capture[1]).tolist()
            self.engine.observe_2d(candidate,now)
        except (ValueError,TypeError,KeyError,IndexError) as exc:
            self.get_logger().warning('Rejected 2D target: '+str(exc),throttle_duration_sec=5.)

    def on_map(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('frame_id') != self.cfg['world_frame'] or data.get('schema_version') != 1: return
            epoch = data.get('epoch', 0)
            if self.map_epoch is not None and epoch != self.map_epoch:
                if self.state_fresh() and self.engine.phase != 'IDLE':
                    self.engine.hold(self.state[2], 'perception map reset; call start again')
                self.engine.history.clear(); self.engine.points = []; self.engine.gate = None
                if self.engine.visual and self.state is not None:self.engine.visual.reset(self.state[1],self.state[2])
                self.want_start = False
            self.map_epoch = epoch
            if self.state_fresh():
                record = data.get('objects', {}).get(self.engine.task['id'])
                if record: self.engine.observe(record, self.state[1])
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().warning('Rejected map: '+str(exc), throttle_duration_sec=5.)

    def on_estop(self, msg):
        self.estop = msg.data
        if self.estop:
            self.want_start = False
            if self.state_fresh(): self.engine.hold(self.state[2], 'external estop; call resume/start after release')

    def ownership(self):
        others = [p.node_namespace.rstrip('/')+'/'+p.node_name
                  for p in self.get_publishers_info_by_topic(self.cfg['topics']['goal'])
                  if (p.node_name, p.node_namespace) != (self.get_name(), self.get_namespace())]
        if others: return 'goal already owned by '+', '.join(others)
        controls = self.get_publishers_info_by_topic('/auv/control/thruster_cmd')
        if len(controls) != 1 or controls[0].node_name != 'pid_controller_node':
            return 'requires exactly one PID controller; no other actuator publisher'
        return ''

    def start(self, req, resp):
        error = self.ownership()
        if self.estop: error = 'release external estop first'
        if not self.state_fresh(): error = 'waiting for fresh auv_world/auv_com state'
        if error:
            resp.success = False; resp.message = error; return resp
        self.blocked = False; self.want_start = False
        self.engine.begin(self.state[1], self.state[2]); self.reference = self.state[2].copy()
        self.reference_time = self.state[1]
        resp.success = True; resp.message = 'search started; frozen task path will follow stable gate acquisition'
        self.get_logger().info(resp.message)
        return resp

    def stop(self, req, resp):
        self.want_start = False
        if self.state_fresh():
            self.engine.hold(self.state[2], 'operator stop; holding current pose')
            self.reference = self.state[2].copy()
            resp.success = True; resp.message = 'holding current pose; resume or start explicitly'
        else:
            self.engine.phase = 'IDLE'; self.engine.target = None
            resp.success = False; resp.message = 'state stale; no new target published; PID state watchdog applies'
        return resp

    def resume(self, req, resp):
        if self.engine.phase not in ('HOLD', 'FAILED') or not self.engine.points:
            resp.success = False; resp.message = 'no suspended path; call start'; return resp
        error = self.ownership()
        if error or not self.state_fresh() or self.estop:
            resp.success = False; resp.message = error or 'state stale or estop'; return resp
        self.blocked = False
        self.engine.phase = 'EXECUTE'
        self.engine.index = min(self.engine.index, len(self.engine.points)-1)
        self.engine.set_waypoint(self.engine.points[self.engine.index], self.state[1])
        self.reference = self.state[2].copy(); self.reference_time = self.state[1]
        resp.success = True; resp.message = 'resuming locked waypoint'; return resp

    def limited_reference(self, target):
        # Feed a bounded-speed reference to unchanged PID; arrival uses FINAL waypoint.
        t = self.state[1]
        dt = max(0., min(.3, t-(self.reference_time if self.reference_time is not None else t)))
        self.reference_time = t
        if self.reference is None: self.reference = self.state[2].copy()
        p = self.reference[:3, 3]; goal = target[:3, 3]
        # Limit lookahead relative to actual pose, preventing runaway references.
        actual = self.state[2][:3, 3]
        delta = goal-actual; length = np.linalg.norm(delta)
        lead = self.cfg['reference']['max_position_lead_m']
        local_goal = actual + delta * min(1., lead/max(length, 1e-12))
        d = local_goal-p; size = np.linalg.norm(d)
        speed=self.cfg['reference']['linear_speed_mps']
        if self.engine.phase=='APPROACH_2D':
            speed=min(speed,self.cfg['visual_approach']['approach_speed_mps'])
            if self.engine.visual.target['source']=='parts':speed*=self.cfg['visual_approach']['part_speed_factor']
        p += d * min(1., speed*dt/max(size, 1e-12))
        R = self.reference[:3, :3]
        rv = Rotation.from_matrix(R.T @ target[:3, :3]).as_rotvec(); angle = np.linalg.norm(rv)
        step = np.deg2rad(self.cfg['reference']['angular_speed_deg_s'])*dt
        self.reference[:3, :3] = R @ Rotation.from_rotvec(rv*min(1., step/max(angle, 1e-12))).as_matrix()
        return self.reference

    def publish_goal(self, target):
        g = PoseStamped(); g.header.frame_id = self.cfg['world_frame']
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y, g.pose.position.z = map(float, target[:3, 3])
        q = Rotation.from_matrix(target[:3, :3]).as_quat()
        g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w = map(float, q)
        self.pub_goal.publish(g)

    def tick(self):
        wall = time.monotonic()
        if wall-self.last_guard > .2:
            self.guard_error = self.ownership(); self.last_guard = wall
        if self.guard_error and self.engine.phase not in ('IDLE', 'HOLD'):
            self.blocked = True; self.want_start = False
            if self.state_fresh(): self.engine.hold(self.state[2], self.guard_error)
        if self.want_start and not self.guard_error and self.state_fresh() and wall-self.started_at > 1.:
            self.start(None, Trigger.Response())
        if self.state_fresh() and not self.guard_error and not self.blocked:
            _, stamp, T, v, w = self.state
            if self.pending_hold:
                self.publish_goal(T); self.pending_hold = False
            if self.engine.phase != 'IDLE':
                if not self.estop:
                    if self.engine.visual and wall-self.last_2d_wall>self.cfg['visual_approach']['max_age_s']:
                        self.engine.visual.target=None
                    previous_phase=self.engine.phase
                    target = self.engine.step(stamp, T, v, w)
                    if target is not None:
                        # On failure/stop, stop advancing reference and capture actual state.
                        if self.engine.phase in ('HOLD', 'FAILED', 'DONE'):
                            self.reference = target.copy(); self.reference_time = stamp
                        elif self.engine.phase!=previous_phase and self.engine.phase in ('ALIGN_2D','WAIT_2D','WAIT_3D','CONFIRM','SEARCH'):
                            self.reference=T.copy();self.reference_time=stamp
                        if self.engine.phase in ('EXECUTE','APPROACH_2D'):
                            # Send the full waypoint or 10m visual target to PID.
                            # Search/alignment retain the bounded angular reference.
                            self.reference=target.copy();self.reference_time=stamp
                            self.publish_goal(target)
                        else:
                            self.publish_goal(self.limited_reference(target))
        elif self.engine.phase in ('SEARCH', 'CONFIRM', 'EXECUTE','ALIGN_2D','APPROACH_2D','WAIT_2D','WAIT_3D') and not self.state_fresh():
            # Freeze mission advancement; resumption requires explicit operator action.
            if self.state is not None: self.engine.hold(self.state[2], 'state stale; explicit resume/start required')
        if wall-self.last_status > .5:
            self.last_status = wall
            status = dict(self.engine.status(), state_fresh=self.state_fresh(), ownership_error=self.guard_error,
                          blocked=self.blocked, estop=self.estop, frame_id=self.cfg['world_frame'])
            self.pub_status.publish(String(data=json.dumps(status)))
        if wall-self.last_markers > 1.:
            self.last_markers = wall; self.publish_path()

    def publish_path(self):
        out = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; out.markers.append(clear)
        for i, wp in enumerate(self.engine.points):
            m = Marker(); m.header.frame_id = self.cfg['world_frame']; m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'task_waypoints'; m.id = i; m.type = Marker.ARROW; m.action = Marker.ADD
            a = wp.T[:3, 3]; b = a+.35*wp.T[:3, 2]
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in (a, b)]
            m.scale.x = .025; m.scale.y = .06; m.scale.z = .08
            m.color.b = 1.; m.color.g = .7; m.color.a = 1.
            out.markers.append(m)
        self.pub_path.publish(out)

    def shutdown_hold(self):
        # SIGINT must not leave the last distant external goal active indefinitely.
        if not self.ownership() and self.state_fresh() and self.engine.phase != 'IDLE':
            self.publish_goal(self.state[2])
            if self.reset_pid.service_is_ready():
                future = self.reset_pid.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=.5)


def main(args=None):
    # Keep ROS context alive long enough to clear PID's latched target on Ctrl+C.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Autonomy()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.shutdown_hold(); node.destroy_node(); rclpy.try_shutdown()
