"""Explicit autonomous entry point. Reuses an existing PID; refuses goal conflicts."""
from pathlib import Path
import time
import rclpy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup(context):
    rclpy.init()
    probe = rclpy.create_node('autonomy_launch_check')
    try:
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline: rclpy.spin_once(probe, timeout_sec=.1)
        owners = probe.get_publishers_info_by_topic('/auv/control/goal')
        if owners:
            raise RuntimeError('Stop keyboard/hold/experiment/autonomy goal sources first: '+', '.join(p.node_name for p in owners))
        controllers = probe.get_publishers_info_by_topic('/auv/control/thruster_cmd')
        if controllers and (len(controllers) != 1 or controllers[0].node_name != 'pid_controller_node'):
            raise RuntimeError('Another controller is publishing thruster commands; cannot start PID autonomy')
        have_pid = bool(controllers)
    finally:
        probe.destroy_node(); rclpy.shutdown()
    nodes = []
    enable_pid = context.launch_configurations['with_pid'].lower() in ('true', '1', 'yes')
    if not have_pid and enable_pid:
        pid_config = context.launch_configurations['pid_config']
        nodes.append(Node(package='auv_controller_pid', executable='pid_controller_node',
                          name='pid_controller_node', output='screen',
                          parameters=[pid_config, {'use_sim_time': True}],
                          additional_env={'PYTHONNOUSERSITE': '1'}))
    elif not have_pid:
        raise RuntimeError('No PID controller. Use with_pid:=true or start the existing PID first.')
    nodes.append(Node(package='auv_mission_control', executable='autonomy_node', name='auv_autonomy',
                      output='screen', additional_env={'PYTHONNOUSERSITE': '1'},
                      parameters=[{'config': context.launch_configurations['config'], 'use_sim_time': True,
                                   'autostart': context.launch_configurations['autostart'].lower() in ('true', '1', 'yes')}]))
    return nodes


def generate_launch_description():
    mission = Path(get_package_share_directory('auv_mission_control'))
    pid = Path(get_package_share_directory('auv_controller_pid'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(mission/'config/autonomy.yaml')),
        DeclareLaunchArgument('with_pid', default_value='true'),
        DeclareLaunchArgument('pid_config', default_value=str(pid/'config/pid.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=setup),
    ])
