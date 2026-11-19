#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""run_headless —— 无头 Isaac + 桥的独立入口(Isaac python 运行)。

  /isaacsim/python.sh <ws>/src/auv_isaac_bridge/isaac/run_headless.py \
      [--config headless|ros] [--scene /abs/pool.usd] [--duration-s 600]
      [--hold]            # 不自动退出,常驻等待外部 TCP/ROS 控制

config:
  headless = config/isaac_bridge_headless.json(ros 关,tcp 17789:训练/整定)
  ros      = config/isaac_bridge_ros_headless.json(ros 开,tcp 17790:实验)

流程:SimulationApp(headless) -> 打开 pool.usd -> (无合法 scene 时)
运行期 author 零重力 PhysicsScene(内存,不保存) -> 桥 bootstrap ->
timeline play -> app.update() 循环(桥在物理步回调内施力/发话题) ->
SIGTERM/超时干净收尾(零输出 + 扰动停止 + 视窗清理 + 桥 shutdown)。

stdout 可能被 kit 吞掉:进度写 /tmp/auv_run_headless.log。
"""

import argparse
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

WS = ("/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/"
      "2026Robosub Full Simulation/ROS2_ws_Main_Body")
SCENE = ("/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/"
         "scenes/Full Process AUV Simulation 2026/Pool Enviroment/pool.usd")
LOG = "/tmp/auv_run_headless.log"

_g_stop = {"stop": False}


def _on_sig(signum, _frame):
    _g_stop["stop"] = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="ros",
                    choices=("headless", "ros"))
    ap.add_argument("--scene", default=SCENE)
    ap.add_argument("--duration-s", type=float, default=600.0)
    ap.add_argument("--hold", action="store_true")
    args = ap.parse_args()

    cfg_name = ("isaac_bridge_headless.json"
                if args.config == "headless"
                else "isaac_bridge_ros_headless.json")
    cfg_path = os.path.join(HERE, "..", "config", cfg_name)

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    logf = open(LOG, "a", encoding="utf-8")

    def log(m):
        logf.write("%s %s\n" % (time.strftime("%H:%M:%S"), m))
        logf.flush()

    # 源码目录可见(auv_common 等)
    for p in (os.path.join(WS, "src", "auv_common"),):
        if p not in sys.path:
            sys.path.insert(0, p)

    import omni.usd
    import omni.timeline
    from pxr import UsdPhysics, Sdf
    import numpy as np

    ctx = omni.usd.get_context()
    if not ctx.open_stage(args.scene):
        log("打开场景失败: %s" % args.scene)
        app.close()
        return 1
    stage = ctx.get_stage()
    valid = None
    for p in stage.TraverseAll():
        if p.IsA(UsdPhysics.Scene):
            d = p.GetAttribute("physics:gravityDirection").Get()
            m = p.GetAttribute("physics:gravityMagnitude").Get()
            if d is not None and m is not None and m >= 0.0 and \
                    float(np.linalg.norm(list(d))) > 1e-9:
                valid = p
                break
    if valid is None:
        sc = UsdPhysics.Scene.Define(stage, "/PhysicsScene_AUV_ZGD")
        sc.CreateGravityDirectionAttr((0.0, 0.0, -1.0))
        sc.CreateGravityMagnitudeAttr(0.0)
        # Isaac 5.1 的 UsdPhysics.Scene 无 CreateTimeStepsPerSecondAttr,
        # 用原生属性名(isaac-usd 与开源 pxr 通用)
        sc.GetPrim().CreateAttribute("physics:timeStepsPerSecond",
                                     Sdf.ValueTypeNames.Int, True).Set(120)
        log("已 author 零重力 PhysicsScene(内存,不保存)")
    else:
        log("场景已有合法 PhysicsScene: %s" % valid.GetPath())

    # ros 变体需先加载 ROS2 桥扩展(无头 kit 默认不装,import rclpy 会失败)
    ros_on = False
    try:
        with open(cfg_path, "r", encoding="utf-8") as f:
            import json
            ros_on = bool(json.load(f).get("ros", {}).get("enabled", False))
    except Exception as exc:                        # noqa: BLE001
        log("读配置失败(%s),按 ros=off 处理" % exc)
    if ros_on:
        from isaacsim.core.utils.extensions import enable_extension
        ok = enable_extension("isaacsim.ros2.bridge")
        log("enable isaacsim.ros2.bridge -> %s" % ok)

    import runpy
    log("启动桥: %s" % cfg_path)
    # 桥的 _main() 从模块全局读配置路径(init_globals),不是环境变量
    runpy.run_path(os.path.join(HERE, "auv_isaac_bridge.py"),
                   run_name="__main__",
                   init_globals={"AUV_BRIDGE_CONFIG_PATH":
                                 os.path.abspath(cfg_path)})
    import builtins
    bridge = getattr(builtins, "_AUV_ISAAC_BRIDGE_INSTANCE", None)
    if bridge is None:
        log("桥实例未创建")
        app.close()
        return 1

    tl = omni.timeline.get_timeline_interface()
    tl.set_play_every_frame(True)
    tl.play()
    log("play: scene=%s cfg=%s dur=%.0fs hold=%s"
        % (args.scene, cfg_name, args.duration_s, args.hold))

    signal.signal(signal.SIGTERM, _on_sig)
    signal.signal(signal.SIGINT, _on_sig)
    t0 = time.monotonic()
    n = 0
    try:
        while not _g_stop["stop"]:
            app.update()
            n += 1
            # 暂停/未播放时物理步不推进,由主循环 drain 排队的 USD 写操作
            try:
                bridge.drain_main_ops()
            except Exception:                           # noqa: BLE001
                pass
            if n % 600 == 0:
                st = bridge.get_state_snapshot() or {}
                log("t=%.1fs sim_t=%.2f pos=%s"
                    % (time.monotonic() - t0,
                       st.get("sim_time_s", -1.0),
                       [round(x, 3) for x in st.get("pos_m", [0, 0, 0])]))
            if not args.hold and time.monotonic() - t0 > args.duration_s:
                log("时长到,收尾")
                break
    finally:
        try:
            bridge.reset_output()
            bridge._disturb.stop()
            bridge._viz.clear()
            bridge.shutdown()
        except Exception as exc:                       # noqa: BLE001
            log("收尾异常: %s" % exc)
        log("停止(运行 %.1fs, %d 次 update)"
            % (time.monotonic() - t0, n))
        logf.close()
        app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
