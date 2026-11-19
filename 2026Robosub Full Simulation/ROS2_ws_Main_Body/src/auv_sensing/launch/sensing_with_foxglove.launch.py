# -*- coding: utf-8 -*-
"""启动 ZED 采集节点 + foxglove_bridge(若已安装)。

foxglove_bridge 未安装时只启动采集节点,并在终端打印安装提示:
    sudo apt install ros-jazzy-foxglove-bridge

连接方式(推荐 SSH 隧道,不开公网端口):
    ssh -L 8765:localhost:8765 liuhao@172.30.30.1
本地 Foxglove 打开 ws://localhost:8765
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('auv_sensing')
    sensing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sensing.launch.py')))

    try:
        get_package_share_directory('foxglove_bridge')
    except Exception:
        return LaunchDescription([
            sensing,
            LogInfo(msg='[auv_sensing] foxglove_bridge 未安装,只启动了采集节点。'
                         '安装:sudo apt install ros-jazzy-foxglove-bridge'),
        ])

    foxglove = Node(
        package='foxglove_bridge', executable='foxglove_bridge',
        name='foxglove_bridge', output='screen',
        # 只监听本机回环,配合 SSH 隧道使用,不暴露到局域网
        parameters=[{'port': 8765, 'address': '127.0.0.1'}])
    return LaunchDescription([sensing, foxglove])
