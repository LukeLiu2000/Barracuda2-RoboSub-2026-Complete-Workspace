# -*- coding: utf-8 -*-
"""bringup.launch —— 控制器统一入口(ROS 侧;Isaac/桥另行启动,见下)。

  ros2 launch auv_bringup bringup.launch.py controller:=pid \
      goal_source:=hold params:=/abs/pid_tuned.yaml model_npz:=... ...

参数:
  controller    pid | mppi | gru(同一时刻仅启动一个控制器节点)
  goal_source   hold | keyboard | experiment | none(互斥;experiment/none
                表示目标由实验编排器或外部发布,本入口不启目标源)
  params        pid/gru 参数 yaml 覆盖(整定产物等)
  model_npz / model_meta   gru 模型覆盖(空=用包内默认)
  with_monitor  附带桥监视节点(默认 true)
  with_foxglove 附带 foxglove_bridge 监测通道(默认 false;只监听
                127.0.0.1:foxglove_port,SSH 端口转发后在本地 Foxglove
                连 ws://localhost:8765,详见 docs/foxglove_monitoring.md。
                需先 sudo apt install ros-jazzy-foxglove-bridge)
  foxglove_address / foxglove_port / foxglove_topic_whitelist
                bridge 监听地址/端口/话题白名单(逗号分隔正则,空=全部)

Isaac 侧启动(另开终端):
  GUI(带视窗可视化):
    /isaacsim/python.sh <ws>/src/auv_isaac_bridge/isaac/bootstrap.py
  无头(ROS 开, TCP 17790):
    /isaacsim/python.sh <ws>/src/auv_isaac_bridge/isaac/run_headless.py \
        --config ros
本 launch 落盘启动清单 JSON(时间/参数/包版本)便于复现实验。
"""

import json
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

CONTROLLERS = ("pid", "mppi", "gru")
GOAL_SOURCES = ("hold", "keyboard", "experiment", "none")


def _write_manifest(context):
    args = {k: context.launch_configurations.get(k, "")
            for k in ("controller", "goal_source", "params", "model_npz",
                      "model_meta", "with_monitor", "with_foxglove",
                      "foxglove_address", "foxglove_port",
                      "foxglove_topic_whitelist")}
    args["time"] = time.strftime("%Y-%m-%d %H:%M:%S")
    args["pid"] = os.getpid()
    out_dir = os.path.expanduser("~/auv_experiments/launch_manifests")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(
        out_dir, "launch_%s_%s.json" % (args["controller"],
                                        time.strftime("%Y%m%d_%H%M%S")))
    with open(path, "w", encoding="utf-8") as f:
        json.dump(args, f, ensure_ascii=False, indent=2)
    print("[bringup] 启动清单: %s" % path)
    return []


def _setup(context):
    controller = context.launch_configurations.get("controller", "pid")
    goal_source = context.launch_configurations.get("goal_source", "hold")
    params = (context.launch_configurations.get("params") or "").strip()
    model_npz = (context.launch_configurations.get("model_npz") or "").strip()
    model_meta = (context.launch_configurations.get("model_meta") or "").strip()
    with_monitor = (context.launch_configurations.get("with_monitor")
                    or "true").lower() in ("1", "true", "yes")
    with_foxglove = (context.launch_configurations.get("with_foxglove")
                     or "false").lower() in ("1", "true", "yes")

    if controller not in CONTROLLERS:
        raise RuntimeError("controller 必须是 %s 之一,得到 %r"
                           % (CONTROLLERS, controller))
    if goal_source not in GOAL_SOURCES:
        raise RuntimeError("goal_source 必须是 %s 之一,得到 %r"
                           % (GOAL_SOURCES, goal_source))

    nodes = []
    if controller == "pid":
        pkg = get_package_share_directory("auv_controller_pid")
        nodes.append(Node(
            package="auv_controller_pid", executable="pid_controller_node",
            name="pid_controller_node", output="screen",
            parameters=[params or os.path.join(pkg, "config", "pid.yaml"),
                        {"use_sim_time": True}]))
    elif controller == "gru":
        pkg = get_package_share_directory("auv_controller_gru")
        overrides = {"use_sim_time": True}
        if model_npz:
            overrides["model_npz"] = model_npz
        if model_meta:
            overrides["model_meta"] = model_meta
        nodes.append(Node(
            package="auv_controller_gru", executable="gru_controller_node",
            name="gru_controller_node", output="screen",
            parameters=[params or os.path.join(pkg, "config", "gru.yaml"),
                        overrides]))
    else:
        mppi = get_package_share_directory("auv_controller_MPPI")
        nodes.append(Node(
            package="auv_controller_MPPI", executable="mppi_controller_node",
            name="mppi_controller_node", output="screen",
            parameters=[os.path.join(mppi, "config", n) for n in
                        ("thrusters.yaml", "dynamics.yaml", "mppi.yaml")],
        ))

    if goal_source == "hold":
        nodes.append(Node(
            package="auv_mission_control", executable="hold_node",
            name="auv_hold_node", output="screen",
            # use_sim_time: /auv/control/goal 的时间戳与 /auv/state/control
            # 同处仿真时间轴(桥发 /clock),Foxglove 按头时间戳画曲线才对齐
            parameters=[{"use_sim_time": True}]))
    elif goal_source == "keyboard":
        print("[bringup] keyboard 模式:键盘节点另开终端:\n"
              "  ros2 run auv_mission_control keyboard_node\n"
              "  (web: ros2 run auv_mission_control web_keyboard + "
              "ssh -L 8791:127.0.0.1:8791)")
    # experiment/none: 目标由实验编排器或外部发布

    if with_monitor:
        nodes.append(Node(
            package="auv_isaac_bridge", executable="bridge_monitor.py",
            name="auv_bridge_monitor", output="screen"))

    if with_foxglove:
        # 只启动 foxglove_bridge(不含相机/第二控制器);未安装时
        # foxglove.launch 会明确报错并给出 apt 安装命令
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("auv_bringup"), "launch",
                "foxglove.launch.py")),
            launch_arguments={
                "address":
                    context.launch_configurations.get("foxglove_address",
                                                      "127.0.0.1"),
                "port": context.launch_configurations.get("foxglove_port",
                                                          "8765"),
                "topic_whitelist":
                    context.launch_configurations.get("foxglove_topic_whitelist",
                                                      ""),
            }.items()))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("controller", default_value="pid"),
        DeclareLaunchArgument("goal_source", default_value="hold"),
        DeclareLaunchArgument("params", default_value=""),
        DeclareLaunchArgument("model_npz", default_value=""),
        DeclareLaunchArgument("model_meta", default_value=""),
        DeclareLaunchArgument("with_monitor", default_value="true"),
        DeclareLaunchArgument("with_foxglove", default_value="false",
                              description="附带 foxglove_bridge 监测通道"),
        DeclareLaunchArgument("foxglove_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        DeclareLaunchArgument("foxglove_topic_whitelist", default_value="",
                              description="逗号分隔正则;空=全部话题"),
        OpaqueFunction(function=_write_manifest),
        OpaqueFunction(function=_setup),
    ])
