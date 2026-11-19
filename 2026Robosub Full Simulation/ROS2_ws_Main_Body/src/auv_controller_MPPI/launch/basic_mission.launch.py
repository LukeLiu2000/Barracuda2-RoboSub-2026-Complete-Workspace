# -*- coding: utf-8 -*-
"""基础控制任务(第十二章)自包含闭环测试,不依赖任何外部话题。

  ros2 launch auv_controller_MPPI basic_mission.launch.py
  # 调参示例:
  ros2 launch auv_controller_MPPI basic_mission.launch.py K:=2000 sigma:=12.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('auv_controller_MPPI')
    cfg = lambda name: os.path.join(pkg_share, 'config', name)
    out_dir = os.path.join(pkg_share, 'data')
    os.makedirs(out_dir, exist_ok=True)

    return LaunchDescription([
        DeclareLaunchArgument('K', default_value='1000'),
        DeclareLaunchArgument('sigma', default_value='10.0'),
        Node(
            package='auv_controller_MPPI',
            executable='basic_mission_test',
            name='basic_mission_test',
            output='screen',
            parameters=[
                cfg('thrusters.yaml'), cfg('dynamics.yaml'),
                cfg('mppi.yaml'), cfg('training.yaml'),
                # 自包含仿真的世界系 = 初始体坐标系(USD Y-up)
                {'dynamics.world_up': [0.0, 1.0, 0.0],
                 'dynamics.gravity_world': [0.0, -9.81, 0.0],
                 'basic_mission.output_dir': out_dir,
                 'mppi.K': LaunchConfiguration('K'),
                 'mppi.sigma': LaunchConfiguration('sigma')},
            ],
        ),
    ])
