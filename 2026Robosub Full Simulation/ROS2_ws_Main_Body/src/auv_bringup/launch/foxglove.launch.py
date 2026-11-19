# -*- coding: utf-8 -*-
"""foxglove.launch —— 只启动 foxglove_bridge 监测通道。

不启动 ZED 采集(用 auv_sensing 的 sensing_with_foxglove 才带相机),
不启动任何控制器;只是把 ROS 2 图以 Foxglove WebSocket 协议暴露出来。

  ros2 launch auv_bringup foxglove.launch.py
  ros2 launch auv_bringup foxglove.launch.py address:=127.0.0.1 port:=8765
  # 限制桥订阅/转发的话题(逗号分隔正则),降低相机/点云带宽:
  ros2 launch auv_bringup foxglove.launch.py \
      topic_whitelist:=/auv/state/.*,/auv/control/.*,/auv/pid/.*,/clock

统一入口等价用法(控制器 + 桥一条命令):
  ros2 launch auv_bringup bringup.launch.py controller:=pid with_foxglove:=true

安全/QoS 说明(2026-11-10 实测,foxglove_bridge 3.6.0 / Jazzy):
  - 默认只监听 127.0.0.1:8765,供笔记本 SSH 端口转发,不暴露公网;
  - /auv/state/control 是 SensorDataQoS(BEST_EFFORT)。bridge 默认
    best_effort_qos_topic_whitelist=["(?!)"](永不匹配),会用 RELIABLE 订阅
    ——话题列表里有名字但一条数据都收不到。本 launch 固定传 [".*"],
    已实测 20 Hz BEST_EFFORT 话题全量通过(499/25s)。这只改 bridge 自己
    的订阅方式,不改任何现有发布链路的可靠性语义;
  - 同端口只可能有一个 bridge:第二个实例 bind 失败报错退出
    ("Failed to bind port: Address already in use"),不会出现双桥;
    与 auv_sensing/sensing_with_foxglove.launch.py 互斥(都占 8765);
  - foxglove_bridge 未安装时本 launch 明确报错并给出安装命令,
    不会假装已连接:
      sudo apt install ros-jazzy-foxglove-bridge
  - 桥是只读监测通道,本 launch 不含任何向 /auv/control/* 发布的节点。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _setup(context):
    try:
        get_package_share_directory("foxglove_bridge")
    except Exception:
        raise RuntimeError(
            "[foxglove.launch] foxglove_bridge 未安装,无法启动监测通道。"
            "Jazzy 对应安装命令:\n"
            "    sudo apt install ros-jazzy-foxglove-bridge\n"
            "安装后重新 source /opt/ros/jazzy/setup.bash 再启动。")

    address = (context.launch_configurations.get("address")
               or "127.0.0.1").strip()
    port = int(context.launch_configurations.get("port") or "8765")
    wl_raw = (context.launch_configurations.get("topic_whitelist")
              or "").strip()

    params = {
        "port": port,
        "address": address,
        # 关键:让 bridge 以 BEST_EFFORT 订阅 BEST_EFFORT 发布者
        # (/auv/state/control 等 SensorDataQoS 话题),否则曲线出不来
        "best_effort_qos_topic_whitelist": [".*"],
    }
    if wl_raw:
        params["topic_whitelist"] = [
            s.strip() for s in wl_raw.split(",") if s.strip()]

    return [Node(
        package="foxglove_bridge", executable="foxglove_bridge",
        name="foxglove_bridge", output="screen", parameters=[params])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "address", default_value="127.0.0.1",
            description="WebSocket 监听地址(默认只回环,SSH 隧道用)"),
        DeclareLaunchArgument(
            "port", default_value="8765",
            description="WebSocket 端口"),
        DeclareLaunchArgument(
            "topic_whitelist", default_value="",
            description="逗号分隔的正则;空=不过滤(全部话题)。"
                        "例: /auv/state/.*,/auv/control/.*,/auv/pid/.*,/clock"),
        OpaqueFunction(function=_setup),
    ])
