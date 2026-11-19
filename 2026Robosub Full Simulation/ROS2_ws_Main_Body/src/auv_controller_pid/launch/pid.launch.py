# -*- coding: utf-8 -*-
"""PID 控制器独立启动(联调用;统一入口见 auv_bringup/bringup.launch.py)。

  ros2 launch auv_controller_pid pid.launch.py
  ros2 launch auv_controller_pid pid.launch.py \
      params:=/abs/path/to/pid_tuned_YYYY-MM-DD.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('auv_controller_pid')
    default_params = os.path.join(pkg, 'config', 'pid.yaml')
    params = LaunchConfiguration('params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params', default_value=default_params,
            description='PID 参数 yaml(可用整定产出文件覆盖)'),
        Node(
            package='auv_controller_pid',
            executable='pid_controller_node',
            name='pid_controller_node',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
        ),
    ])
