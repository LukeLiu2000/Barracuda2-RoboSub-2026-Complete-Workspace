"""Isolated ROS graph; a bounded kinematic robot follows published goals.

Run only with AUV_AUTONOMY_ROS_TEST=1 in a separate ROS_DOMAIN_ID, never the user sim domain.
This tests message/service wiring, not Isaac physics or gate clearance.
"""
import os
import json
import time
from pathlib import Path
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

pytestmark = pytest.mark.skipif(os.environ.get('AUV_AUTONOMY_ROS_TEST') != '1', reason='requires isolated ROS domain')


def test_ros_goal_execution_and_ownership(tmp_path):
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 60
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String, Float64MultiArray, Bool
    from std_srvs.srv import Trigger
    from ament_index_python.packages import get_package_share_directory
    from auv_mission_control.autonomy_node import Autonomy, pose_transform
    from auv_mission_control.waypoint_mission import BODY_TO_TASK
    root = Path(__file__).resolve().parents[1]
    cfg = yaml.safe_load((root/'config/autonomy.yaml').read_text())
    task = yaml.safe_load((root/'config/tasks/task1_gate.yaml').read_text())
    task['max_laps'] = 1
    task_path = tmp_path/'task.yaml'; task_path.write_text(yaml.safe_dump(task))
    cfg['task_file'] = str(task_path)
    path = tmp_path/'autonomy.yaml'; path.write_text(yaml.safe_dump(cfg))
    rclpy.init(args=['--ros-args', '-p', 'config:='+str(path), '-p', 'use_sim_time:=true'])
    node = Autonomy(); robot = rclpy.create_node('synthetic_robot')
    fake_pid = rclpy.create_node('pid_controller_node')
    # Presence of this endpoint represents the PID consumer; actual PID core is regression-tested separately.
    actuator = fake_pid.create_publisher(Float64MultiArray, '/auv/control/thruster_cmd', 10)
    states = robot.create_publisher(Odometry, '/auv/state/control', qos_profile_sensor_data)
    clocks = robot.create_publisher(Clock, '/clock', 10)
    maps = robot.create_publisher(String, '/auv/perception/object_map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    detections = robot.create_publisher(String, '/auv/perception/detections_2d', 10)
    goals = []; statuses = []
    robot.create_subscription(PoseStamped, '/auv/control/goal', lambda m:goals.append(m), 10)
    robot.create_subscription(String, '/auv/mission/status', lambda m:statuses.append(json.loads(m.data)), 10)
    executor = SingleThreadedExecutor()
    for n in (node, robot, fake_pid): executor.add_node(n)

    def pump(duration=.01):
        end = time.monotonic()+duration
        while time.monotonic()<end: executor.spin_once(timeout_sec=.001)

    T = np.eye(4); T[:3, :3] = BODY_TO_TASK; T[:3, 3] = [-4, 0, -.63785]
    now = 10.

    def state(t, v=0., w=0.):
        c = Clock(); c.clock.sec = int(t); c.clock.nanosec = int((t-int(t))*1e9); clocks.publish(c)
        msg = Odometry(); msg.header.frame_id = 'auv_world'; msg.child_frame_id = 'auv_com'; msg.header.stamp = c.clock
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, T[:3, 3])
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = map(float, q)
        msg.twist.twist.linear.x = float(v); msg.twist.twist.angular.z = float(w)
        states.publish(msg); pump()

    def service(name):
        client = robot.create_client(Trigger, '/auv_autonomy/'+name)
        assert client.wait_for_service(timeout_sec=2)
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic()+2
        while not future.done() and time.monotonic()<deadline:pump()
        assert future.done()
        response = future.result(); robot.destroy_client(client); return response

    try:
        pump(.4); state(now)
        assert service('start').success
        # No cloud, map or depth exists: real ROS callback must transition to 2D approach.
        for _ in range(2):
            now+=.2;state(now)
            detections.publish(String(data=json.dumps(dict(schema_version=1,stamp=now,frame_id='test_optical',width=640,height=360,
                projection=[500,0,320,0,0,500,180,0,0,0,1,0],detections=[dict(label='FlagRepair',confidence=.9,bbox=[310,170,330,190])]))))
            pump();node.last_guard=-1e9;node.tick();pump()
        assert node.engine.phase=='APPROACH_2D'
        assert np.allclose(pose_transform(goals[-1].pose)[:3,3],T[:3,3]+[10,0,0],atol=1e-4)
        assert node.engine.visual.target['labels']==['FlagRepair']
        assert np.allclose(node.engine.target[:3,3],T[:3,3]+[10,0,0],atol=1e-4)
        now+=.2;state(now)
        detections.publish(String(data=json.dumps(dict(schema_version=1,stamp=now,frame_id='test_optical',width=640,height=360,
            projection=[500,0,320,0,0,500,180,0,0,0,1,0],detections=[]))))
        pump();node.tick();pump()
        assert node.engine.phase=='WAIT_2D'
        assert np.linalg.norm(pose_transform(goals[-1].pose)[:3,3]-T[:3,3])<.001
        G = np.eye(4); G[:3, :3] = [[0, -0., -1], [-1, 0, 0], [0, 1, 0]]
        # Proper upright gate model x world-Y, y world-Z, z world+X.
        G[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        # x cross y = +worldX, so this has determinant +1.
        for i in range(3):
            now += .6; state(now)
            maps.publish(String(data=json.dumps({'schema_version':1,'frame_id':'auv_world','epoch':0,'objects':{
                'task1_gate':{'T':G.tolist(),'last_accepted':now,'quality':{'accepted':True}}}})))
            pump(); node.last_guard = -1e9; node.tick(); pump()
        assert node.engine.phase == 'EXECUTE'
        # The PID must receive the COMPLETE waypoint, even beyond the 0.6m reference lead.
        assert np.allclose(pose_transform(goals[-1].pose),node.engine.target)
        assert np.linalg.norm(node.engine.target[:3,3]-T[:3,3])>.6
        visited = []
        for _ in range(2500):
            if node.engine.phase != 'EXECUTE': break
            name = node.engine.points[node.engine.index].name
            if not visited or visited[-1] != name: visited.append(name)
            if goals:
                target = pose_transform(goals[-1].pose)
                d = target[:3, 3]-T[:3, 3]; length = np.linalg.norm(d)
                move = d*min(1., .08/max(length,1e-12)); T[:3, 3] += move
                rv = Rotation.from_matrix(T[:3, :3].T@target[:3, :3]).as_rotvec()
                angle = np.linalg.norm(rv); turn = rv*min(1., .09/max(angle,1e-12))
                T[:3, :3] = T[:3, :3]@Rotation.from_rotvec(turn).as_matrix()
                v = np.linalg.norm(move)/.2; w = np.linalg.norm(turn)/.2
            else:v=w=0.
            now += .2; state(now, v, w); node.tick(); pump(.002)
            if node.engine.phase=='EXECUTE':
                assert np.allclose(pose_transform(goals[-1].pose),node.engine.target)
        assert node.engine.phase == 'DONE', node.engine.status()
        assert node.engine.laps == 1
        assert visited == [item['name'] for item in task['waypoints']]
        assert all(m.header.frame_id == 'auv_world' for m in goals)
        assert statuses
        # A legacy keyboard publisher appearing later must block new autonomous goals.
        assert service('start').success
        intruder = robot.create_publisher(PoseStamped, '/auv/control/goal', 10)
        pump(.2); node.last_guard = -1e9; node.tick(); pump()
        before = len(goals); node.tick(); pump()
        assert node.blocked and node.engine.phase == 'HOLD' and len(goals) == before
        assert not service('start').success
        robot.destroy_publisher(intruder); pump(.2)
        state(now)
        assert service('start').success
        # Stale input stops sequencing; clock reset requires explicit restart.
        node.state = (time.monotonic()-2, *node.state[1:]); node.last_guard = -1e9; node.tick()
        assert node.engine.phase == 'HOLD'
        now = 0.; state(now); node.last_guard = -1e9; node.tick(); pump()
        assert node.engine.phase == 'IDLE'
    finally:
        for n in (node, robot, fake_pid): executor.remove_node(n); n.destroy_node()
        executor.shutdown(); rclpy.try_shutdown()
