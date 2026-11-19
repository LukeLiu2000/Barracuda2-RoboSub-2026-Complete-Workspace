# -*- coding: utf-8 -*-
"""Training Mode 启动:任务生成 + 数据记录 + 控制器(含激励旁路)。

用法:
  ros2 launch auv_controller_MPPI training.launch.py mode:=random_goals
  ros2 launch auv_controller_MPPI training.launch.py mode:=excitation
数据落在包目录 data/ 下(授权目录内)。
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
    # 记录目录固定在包 share 目录内(隔离构建时整个都在授权目录里)
    data_dir = os.path.join(pkg_share, 'data')
    os.makedirs(data_dir, exist_ok=True)

    mode = LaunchConfiguration('mode')

    common = [cfg('thrusters.yaml'), cfg('dynamics.yaml'), cfg('mppi.yaml'),
              cfg('training.yaml')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='random_goals',
            description='random_goals | excitation'),
        Node(
            package='auv_controller_MPPI',
            executable='mppi_controller_node',
            name='mppi_controller_node',
            output='screen',
            parameters=common + [{'topics.state': '/auv/state/control'},
                                 {'use_sim_time': True}],
        ),
        Node(
            package='auv_controller_MPPI',
            executable='training_mission_node',
            name='training_mission_node',
            output='screen',
            parameters=common + [{'training.mode': mode},
                                 {'use_sim_time': True}],
        ),
        Node(
            package='auv_controller_MPPI',
            executable='training_data_collector_node',
            name='training_data_collector_node',
            output='screen',
            # use_sim_time: 命令时间加权积分与状态 header.stamp 同为仿真时间轴
            parameters=common + [{'collector.output_dir': data_dir},
                                 {'use_sim_time': True}],
        ),
    ])
