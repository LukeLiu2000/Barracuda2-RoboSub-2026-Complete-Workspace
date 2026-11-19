# -*- coding: utf-8 -*-
"""Normal Mode 启动:MPPI 控制器节点。

用法:
  ros2 launch auv_controller_MPPI controller.launch.py
  # 用仿真真值当状态源调试(需要 auv_sensing 侧真值话题已接通):
  ros2 launch auv_controller_MPPI controller.launch.py \
      state_topic:=/auv/ground_truth/odometry
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('auv_controller_MPPI')
    cfg = lambda name: os.path.join(pkg_share, 'config', name)

    state_topic = LaunchConfiguration('state_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'state_topic', default_value='/auv/state/control',
            description='状态输入话题(nav_msgs/Odometry)'),
        Node(
            package='auv_controller_MPPI',
            executable='mppi_controller_node',
            name='mppi_controller_node',
            output='screen',
            parameters=[
                cfg('thrusters.yaml'),
                cfg('dynamics.yaml'),
                cfg('mppi.yaml'),
                {'topics.state': state_topic},
            ],
        ),
    ])
