# -*- coding: utf-8 -*-
"""启动 ZED 采集节点。

用法(新终端):
    source /opt/ros/jazzy/setup.bash
    source <ws>/install/setup.bash
    ros2 launch auv_sensing sensing.launch.py

可选参数:
    use_sim_time:=true   让节点内部时钟跟随 /clock(消息时间戳本来就来自
                          仿真时间,此开关只影响节点自身计时行为,默认 false)
    sensing_config:=<yaml 路径>   覆盖默认 config/sensing.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('auv_sensing')
    default_config = os.path.join(pkg_share, 'config', 'sensing.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sensing_config', default_value=default_config,
            description='节点参数 yaml'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否跟随 /clock(见文件头注释)'),
        Node(
            package='auv_sensing',
            executable='zed_sensor_node',
            name='zed_sensor_node',      # 必须与 sensing.yaml 顶层键一致
            namespace='auv',             # 话题最终为 /auv/zed/...
            output='screen',
            # LC_ALL=C:ZED SDK 解析标定时对 locale 敏感,非 C locale 会告警
            additional_env={'LC_ALL': 'C'},
            parameters=[
                LaunchConfiguration('sensing_config'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
