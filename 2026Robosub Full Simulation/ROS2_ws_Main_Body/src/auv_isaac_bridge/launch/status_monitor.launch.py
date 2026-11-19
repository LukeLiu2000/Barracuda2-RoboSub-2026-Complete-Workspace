# -*- coding: utf-8 -*-
"""桥状态监视(服务器系统 ROS 2 环境):1Hz 打印真值/命令/桥状态单行。"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="auv_isaac_bridge",
            executable="bridge_monitor.py",
            output="screen",
        ),
    ])
