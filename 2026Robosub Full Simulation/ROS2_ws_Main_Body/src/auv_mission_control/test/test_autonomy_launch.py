"""Launch real unchanged PID in an isolated domain; no Isaac connection."""
import os
import signal
import subprocess
import time
from pathlib import Path
import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('AUV_AUTONOMY_ROS_TEST') != '1', reason='requires isolated ROS domain')


def test_launch_starts_pid_once_and_waits_for_explicit_start(tmp_path):
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 60
    import rclpy
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float64MultiArray
    from std_srvs.srv import Trigger
    from rclpy.qos import qos_profile_sensor_data
    from scipy.spatial.transform import Rotation
    from auv_mission_control.waypoint_mission import BODY_TO_TASK
    env = dict(os.environ, ROS_DOMAIN_ID='63')
    command = Path(__file__).resolve().parents[1]/'scripts/start_autonomy.sh'
    log_path = tmp_path/'launch.log'
    log = log_path.open('w')
    proc = subprocess.Popen(['bash', str(command), 'autostart:=false'], env=env,
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(domain_id=63)
    node = rclpy.create_node('isolated_launch_probe')
    states = node.create_publisher(Odometry, '/auv/state/control', qos_profile_sensor_data)
    clocks = node.create_publisher(Clock, '/clock', 10)
    cmds = []; goals = []
    node.create_subscription(Float64MultiArray, '/auv/control/thruster_cmd', lambda m:cmds.append(m.data), 10)
    node.create_subscription(PoseStamped, '/auv/control/goal', lambda m:goals.append(m), 10)
    client = node.create_client(Trigger, '/auv_autonomy/start')
    sim_time = 1.

    def pump(duration):
        nonlocal sim_time
        end = time.monotonic()+duration
        while time.monotonic()<end:
            sim_time += .02
            c = Clock(); c.clock.sec = int(sim_time); c.clock.nanosec = int((sim_time-int(sim_time))*1e9)
            clocks.publish(c)
            s = Odometry(); s.header.frame_id='auv_world'; s.child_frame_id='auv_com'; s.header.stamp=c.clock
            q = Rotation.from_matrix(BODY_TO_TASK).as_quat()
            s.pose.pose.orientation.x, s.pose.pose.orientation.y, s.pose.pose.orientation.z, s.pose.pose.orientation.w=map(float,q)
            states.publish(s)
            rclpy.spin_once(node, timeout_sec=.01)

    try:
        end = time.monotonic()+10
        while not client.service_is_ready() and time.monotonic()<end:
            assert proc.poll() is None, log_path.read_text()
            pump(.1)
        assert client.service_is_ready(), log_path.read_text()
        pump(.5)
        controllers = node.get_publishers_info_by_topic('/auv/control/thruster_cmd')
        assert len(controllers)==1 and controllers[0].node_name=='pid_controller_node'
        assert cmds and not goals  # PID hold runs; autonomy has not been started.
        future=client.call_async(Trigger.Request())
        end=time.monotonic()+3
        while not future.done() and time.monotonic()<end:pump(.05)
        assert future.done() and future.result().success
        pump(1.)
        assert goals
        assert any(np.linalg.norm(cmd)>1e-4 for cmd in cmds), 'search yaw must reach unchanged PID'
        assert all(g.header.frame_id=='auv_world' for g in goals)
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM);proc.wait(timeout=3)
        log.close();node.destroy_node();rclpy.try_shutdown()
