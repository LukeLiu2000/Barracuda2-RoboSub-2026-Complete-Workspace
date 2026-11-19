# -*- coding: utf-8 -*-
"""sim_runtime —— 无头 Isaac Sim + auv_isaac_bridge 进程内驱动。

必须先创建 SimulationApp(调用方负责),再构造 HeadlessAuvSim:
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    sim = HeadlessAuvSim(app, bridge_config=..., physics_dt=1/120)

职责:
  - 打开 pool.usd(绝对路径可配);
  - 若场景无合法 PhysicsScene:运行期 author 一个显式零重力场景
    (gravityDirection=(0,0,-1) + magnitude=0,timeStepsPerSecond=1/physics_dt),
    只写内存 stage,不保存资产,启动日志如实记录(ZGD 第一阶段约定);
  - 以给定配置启动桥(复用全部真实代码路径:校验/限幅/阻尼/几何/力换算);
  - reset_robot(pause->set_pose->play,带位姿核验与一次重试);
  - step_for(seconds):app.update() 循环推进,直到桥观测的仿真时间前进
    所需秒数(桥订阅物理步事件,力在每物理步施加)。
"""

import importlib.util
import os
import time
import traceback

import numpy as np

WS = ("/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/"
      "2026Robosub Full Simulation/ROS2_ws_Main_Body")
BRIDGE_PATH = os.path.join(WS, "src", "auv_isaac_bridge",
                           "isaac", "auv_isaac_bridge.py")
DEFAULT_SCENE = ("/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/"
                 "scenes/Full Process AUV Simulation 2026/Pool Enviroment/"
                 "pool.usd")
DEFAULT_BRIDGE_CFG = os.path.join(WS, "src", "auv_isaac_bridge", "config",
                                  "isaac_bridge_headless.json")


class HeadlessAuvSim(object):

    def __init__(self, app, scene_path=None, bridge_config=None,
                 physics_dt=1.0 / 120.0, log=print):
        self.app = app
        self.log = log
        self.scene_path = scene_path or DEFAULT_SCENE
        self.bridge_config = bridge_config or DEFAULT_BRIDGE_CFG
        self.physics_dt = float(physics_dt)
        self._n_updates = 0
        self._t_wall0 = time.monotonic()

        import omni.usd
        import omni.timeline
        from pxr import UsdPhysics, Sdf
        self._usdphysics = UsdPhysics
        self._timeline = omni.timeline.get_timeline_interface()
        ctx = omni.usd.get_context()
        if not ctx.open_stage(self.scene_path):
            raise RuntimeError("打开场景失败: %s" % self.scene_path)
        stage = ctx.get_stage()
        self.stage = stage

        # ---- author 显式零重力 PhysicsScene(pool.usd 无 scene prim;
        # kit 自动 scene 是垃圾值,桥按"未确认"处理)。只在内存,不存盘。
        valid_scene = None
        for p in stage.TraverseAll():
            if p.IsA(UsdPhysics.Scene):
                d = p.GetAttribute("physics:gravityDirection").Get()
                m = p.GetAttribute("physics:gravityMagnitude").Get()
                if d is not None and m is not None and m >= 0.0 and \
                        float(np.linalg.norm(list(d))) > 1e-9:
                    valid_scene = p
                    break
        if valid_scene is None:
            scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene_AUV_ZGD")
            scene.CreateGravityDirectionAttr((0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr(0.0)
            tps = max(int(round(1.0 / self.physics_dt)), 1)
            # Isaac 5.1 的 UsdPhysics.Scene 无 CreateTimeStepsPerSecondAttr,
            # 用原生属性名(isaac-usd 与开源 pxr 通用)
            scene.GetPrim().CreateAttribute(
                "physics:timeStepsPerSecond", Sdf.ValueTypeNames.Int,
                True).Set(tps)
            self.log("[sim_runtime] 已运行期 author 零重力 PhysicsScene "
                     "(dir=(0,0,-1), mag=0, tps=%d;仅内存,不保存)" % tps)
        else:
            self.log("[sim_runtime] 场景已有合法 PhysicsScene: %s"
                     % valid_scene.GetPath())

        # ---- 启动桥(真实代码路径;配置可为 ros-off 训练变体)
        self.bridge = self._start_bridge()

        try:
            self._timeline.set_play_every_frame(True)
        except Exception as exc:                     # noqa: BLE001
            self.log("[sim_runtime] set_play_every_frame 失败: %s" % exc)
        self._timeline.play()
        self.step_for(0.1)
        st = self.state()
        self.log("[sim_runtime] ready: sim_t=%.3f pos=%s" % (
            st["sim_time_s"], np.round(st["pos_m"], 3).tolist()))
        # 记录资产原始位姿作为默认出生点(整定/训练都用它)
        self.spawn_p = np.asarray(st["pos_m"], dtype=np.float64)
        self.spawn_q = np.asarray(st["quat_xyzw"], dtype=np.float64)

    # ------------------------------------------------------------------
    def _start_bridge(self):
        spec = importlib.util.spec_from_file_location(
            "auv_isaac_bridge_runtime", BRIDGE_PATH)
        mod = importlib.util.module_from_spec(spec)
        mod.__dict__["AUV_BRIDGE_CONFIG_PATH"] = self.bridge_config
        spec.loader.exec_module(mod)
        inst = getattr(mod.builtins, "_AUV_ISAAC_BRIDGE_INSTANCE", None)
        if inst is None:
            raise RuntimeError("桥实例创建失败")
        return inst

    # ------------------------------------------------------------------
    def state(self):
        st = self.bridge.get_state_snapshot()
        if st is None:
            return {"sim_time_s": 0.0, "pos_m": [0, 0, 0],
                    "quat_xyzw": [0, 0, 0, 1], "v_body": [0, 0, 0],
                    "w_body": [0, 0, 0]}
        return st

    def set_command(self, f8_si):
        self.bridge.set_command(f8_si)

    def reset_output(self):
        self.bridge.reset_output()

    def step_for(self, seconds):
        """推进仿真时间 seconds(按桥观测的仿真时间计)。"""
        t0 = float(self.state()["sim_time_s"])
        target = t0 + float(seconds)
        n = 0
        n_max = int(seconds / max(self.physics_dt, 1e-4)) * 4 + 200
        while float(self.state()["sim_time_s"]) < target - 1e-9:
            self.app.update()
            n += 1
            self._n_updates += 1
            try:
                self.bridge.drain_main_ops()
            except Exception:                           # noqa: BLE001
                pass
            if n > n_max:
                self.log("[sim_runtime] step_for 超出更新上限(%d),提前返回"
                         % n_max)
                break
        return float(self.state()["sim_time_s"]) - t0

    def reset_robot(self, pos_m, quat_xyzw, settle_s=0.05):
        """复位:stop(时间归零,物理状态从 USD 重建)-> 写位姿 -> play ->
        推进 settle_s 建立速度差分基线;核验位姿生效,失败一次则改用
        pause->set->play 再试。"""
        pos_m = np.asarray(pos_m, dtype=np.float64)
        quat_xyzw = np.asarray(quat_xyzw, dtype=np.float64)
        self.bridge.reset_output()
        for attempt, mode in enumerate(("stop", "pause")):
            try:
                getattr(self._timeline, mode)()
                time.sleep(0.05)
                r = self.bridge.set_robot_pose(pos_m, quat_xyzw)
                if not r.get("ok"):
                    self.log("[sim_runtime] set_pose 失败: %s" % r)
                self._timeline.play()
                self.step_for(settle_s)
                st = self.state()
                err_p = float(np.linalg.norm(
                    np.asarray(st["pos_m"]) - pos_m))
                if err_p < 0.05:
                    return True
                self.log("[sim_runtime] 复位核验误差 %.3f m(%s 模式)"
                         % (err_p, mode))
            except Exception as exc:                # noqa: BLE001
                self.log("[sim_runtime] reset 异常(%s): %s\n%s"
                         % (mode, exc, traceback.format_exc()[-500:]))
        return False

    # ------------------------------------------------------------------
    def disturb_configure(self, patch):
        return self.bridge._disturb.configure(patch)

    def disturb_stop(self):
        self.bridge._disturb.stop()

    def viz_set(self, patch):
        return self.bridge._viz.cfg.update(patch)

    def close(self):
        try:
            self.bridge.shutdown()
        except Exception:                           # noqa: BLE001
            pass
