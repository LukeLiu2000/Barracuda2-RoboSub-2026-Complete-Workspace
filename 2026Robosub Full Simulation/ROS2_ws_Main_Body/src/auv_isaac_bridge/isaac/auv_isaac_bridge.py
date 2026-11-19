#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""auv_isaac_bridge —— 在 Isaac Sim(kit 进程内)运行的 AUV 力桥 + 真值源。

启动方式:用户在 Isaac Sim 的 Script Editor 里执行 isaac/bootstrap.py(一次),
它会 runpy 本文件。之后所有诊断/实验都通过 TCP(默认 127.0.0.1:17788,
JSON 按行收发,一问一答)远程驱动;改了本文件可用 TCP restart 命令热重启,
不需要用户再碰 Script Editor。

职责(完整约定见中文联调报告):
  1. 订阅 /auv/control/thruster_cmd(Float64MultiArray,8 路,SI 牛顿),
     校验(尺寸/NaN/限幅/超时/急停)后逐物理步把每路推力施加到该推进器的
     世界位置(PhysX 自动对质心取矩,桥不重复计算力矩);
  2. 从 base_link 世界变换读真值,发布质心系 nav_msgs/Odometry 到
     /auv/ground_truth/odometry 与 /auv/state/control(同内容;位置=质心
     世界位置,姿态=体姿态,速度=体坐标系质心速度,时间戳=仿真时间),
     同时发布 /clock。速度数值用相邻物理步差分(权威),无隐藏单位假设;
  3. 施加第一版水动力(补偿浮力于 CB + 体轴线性/二次阻尼),参数在
     config/isaac_bridge.json,支持 mtime 热重载与 TCP hydro_set;
  4. 1 Hz 发布 /auv/bridge/status(std_msgs/String,内容为与 TCP status 相同
     结构的 JSON;原 DiagnosticArray 因 isaacsim.ros2.bridge 4.12.4 的 C 层
     断言崩溃问题弃用,见 _ros_start 注释);
  5. TCP:ping/status/stage_info/robot_info/sensor_scan/graph_scan/
     test_thrust/zero/estop/hydro_get/hydro_set/config_reload/timeline/
     set_pose/evaluate/restart;
  6. [2026-11-08] 双作用点随机扰动力(disturbance_0/1,纯作用点 prim,
     无刚体/碰撞体,不改质量):每股独立随机过程,球面均匀方向+
     模长均匀幅值 ≤ force_max_n(默认 19.6133 N = 单推进器上限一半),
     周期采样目标向量 + 余弦平滑过渡(范数凸性保证插值模长不越界),
     默认机体系方向(可配世界系),支持种子/序列记录/回放/阶跃/脉冲;
     TCP disturb_config/disturb_get/disturb_stop;话题
     /auv/disturbance/forces(Float64MultiArray,12 = [F0xyz,F1xyz 世界 N,
     P0xyz,P1xyz 世界 m]);
  7. [2026-11-08] Isaac 视窗实时可视化(debug_draw 线段组,不参与碰撞):
     8 推进器力=蓝、2 扰动力=红、当前位姿=黄(+z 前)、目标位姿=绿(订阅
     /auv/control/goal 只读),力-长度统一比例(默认 1 m ↔ 40 N,可配),
     位姿箭头定长 + 三轴小姿态标,零力隐藏,超长裁剪如实标注(clipped);
     TCP viz_set/viz_get;数据与施力模块共用(不另算一遍);
  8. [2026-11-08] 多写者防护:1 Hz 检查 thruster_cmd 话题发布者数,
     >1 且 enforce_single_controller=true 时输出置零并在 status 标注
     (mode=fault);public set_command/get_state_snapshot 供无 ROS 的
     进程内训练/整定路径复用同一校验与限幅。

单位约定(重要):
  - stage 数值 <-> 真实米:world_numeric = meters / mpu(mpu=stage 每单位米数)。
    本文件里所有"局部米值表到世界数值"都是 value_m/mpu(祖先缩放已在推导中
    消去,见 _derive_stage 注释);里程计输出一律真实米。
  - 力:SI 牛顿 -> PhysX 数值 = SI × f_scale,默认 f_scale=1/mpu
    (mass 恒以 kg 计,meter = mpu 个数值单位,故 1 N = 1/mpu 个数值单位)。
    启动后用 TCP evaluate 实测校验,系统性偏差时在 JSON units.force_unit_scale
    写显式数值并 config_reload。
  - 仿真时间回跳(重置)清空命令与测试推力,恢复后不重放过期命令。
本文件不修改任何场景资产;运行期唯一的场景写操作是 TCP set_pose(试验复位)。
"""

import builtins
import json
import math
import os
import socket
import sys
import threading
import time
import traceback

import numpy as np

import carb

DEFAULT_CONFIG = ("/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/"
                  "2026Robosub Full Simulation/ROS2_ws_Main_Body/src/"
                  "auv_isaac_bridge/config/isaac_bridge.json")

# Isaac Sim 5.1.0 的 ros2.bridge 扩展内自带 jazzy 系 rclpy(py3.11 构建),
# 实际包在 jazzy/rclpy/ 下(4.12 时代直接在 jazzy/ 下,此处按 5.1 布局修正;
# 若目录不存在则回退旧布局)。原生 .so 依赖在 jazzy/lib,须在【进程启动前】
# 经 LD_LIBRARY_PATH 注入(dlopen 不读运行期改动的环境变量)。
KIT_RCLPY_ROOT = "/isaacsim/exts/isaacsim.ros2.bridge/jazzy/rclpy"
KIT_ROS_LIBDIR = "/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"


def _log(msg):
    carb.log_info("[auv_isaac_bridge] " + str(msg))


def _log_warn(msg):
    carb.log_warn("[auv_isaac_bridge] " + str(msg))


def _log_err(msg):
    carb.log_error("[auv_isaac_bridge] " + str(msg))


# ---------------------------------------------------------------------------
# 四元数/旋转小工具(q 一律 (x,y,z,w),右手系,体->世界)
# ---------------------------------------------------------------------------

def _quat_normalize(q):
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def _quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _matrix_to_quat(R):
    """Shepperd 方法,数值稳定。"""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return _quat_normalize(np.array([x, y, z, w]))


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def _quat_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def _angular_velocity_from_quats(q_prev, q_now, dt):
    if dt <= 1e-9:
        return np.zeros(3)
    dq = _quat_mul(q_now, _quat_conj(q_prev))
    if dq[3] < 0.0:
        dq = -dq
    dq = _quat_normalize(dq)
    s = math.sqrt(max(0.0, 1.0 - dq[3] * dq[3]))
    if s < 1e-9:
        return np.zeros(3)
    angle = 2.0 * math.acos(min(1.0, max(-1.0, dq[3])))
    return (dq[:3] / s) * (angle / dt)


def _gf_mat3_to_np(m, gf):
    """Gf.Matrix3d/4d -> np 3x3 线性部分,【列向量约定】(np 结果满足
    v' = result @ v)。Gf 是行向量约定(v' = v @ M),TransformDir(e_i)
    返回 M 的第 i 行,故收集后必须转置——不转置会得到映射的转置
    (祖先变换为单位阵时对称不可见,2026-11-03 live 引用链上实测暴露)。"""
    rows = []
    for e in (gf.Vec3d(1, 0, 0), gf.Vec3d(0, 1, 0), gf.Vec3d(0, 0, 1)):
        c = m.TransformDir(e)
        rows.append([c[0], c[1], c[2]])
    return np.asarray(rows, dtype=np.float64).T


def _proper_rot(m3):
    """线性 3x3 -> 正交旋转矩阵(极分解,SVD,含镜像修正)。
    为什么不用 Gf 的 ExtractRotationQuat/ExtractRotation:实测(2026-11-03)
    在带缩放的矩阵上(包裹 prim 均匀缩放 0.001)它们返回损坏结果——真实旋转
    四元数 (0.5,0.5,0.5,0.5) 被提取成 (0.0005,0.0005,0.5,0.0005)。引用
    进来的机器人必经带缩放的祖先链,故旋转提取一律走数值极分解。"""
    try:
        u, _, vt = np.linalg.svd(np.asarray(m3, dtype=np.float64))
        r = u.dot(vt)
        if np.linalg.det(r) < 0.0:
            u = u.copy()
            u[:, -1] *= -1.0
            r = u.dot(vt)
        return r
    except Exception:                       # noqa: BLE001
        return np.eye(3)


class _FakeCmdMsg(object):
    """set_command 进程内入口用的最小消息壳(仅 .data)。"""

    def __init__(self, data):
        self.data = list(data)


def _validate_scene_gravity(d, m):
    """校验 PhysicsScene 重力属性是否可用(纯函数,可离线单测)。

    返回 (valid, reason, gvec):
      - d 为 gravityDirection,m 为 gravityMagnitude(均为 USD 原始读取值);
      - 非法:属性未授权(None)、含 NaN/±Inf、方向为零向量、模长为负;
      - 【合法零重力】= 方向为有效非零向量 + magnitude 显式为 0(第一阶段
        Zero-Gravity Damped 设计,2026-11-03 与用户约定)——这与
        direction=(0,0,0)/magnitude=-Inf 的【非法垃圾值】严格区分:
        后者是 kit 自动 scene 的坏数据,会被 PhysX 忽略,不算显式零重力;
      - 合法则 gvec = d * m(m=0 时为零向量,valid=True)。
    任何非法值一律显式告警并按"未确认重力"处理(浮力禁用)。"""
    try:
        if d is None:
            return False, "gravityDirection 未授权(None)", None
        if m is None:
            return False, "gravityMagnitude 未授权(None)", None
        dv = np.asarray(d, dtype=np.float64)
        mv = float(m)
        if not np.all(np.isfinite(dv)):
            return False, "gravityDirection 含 NaN/Inf", None
        if not math.isfinite(mv):
            return False, "gravityMagnitude 非有限(%r)" % m, None
        if float(np.linalg.norm(dv)) < 1e-9:
            return False, ("gravityDirection 为零向量(非法,显式零重力应"
                           "用有效方向+magnitude=0)"), None
        if mv < 0.0:
            return False, "gravityMagnitude 为负(%r)" % m, None
        gv = dv * mv
        if not np.all(np.isfinite(gv)):
            return False, "合成重力向量含 NaN/Inf", None
        return True, "", gv
    except Exception as exc:                 # noqa: BLE001
        return False, "校验异常: %s" % exc, None


# ---------------------------------------------------------------------------
# 双作用点随机扰动力(2026-11-08)
# ---------------------------------------------------------------------------
class _DisturbChannel(object):
    """单股扰动的随机过程:周期采样新目标向量 + 余弦平滑过渡。

    目标向量:方向 = 标准正态 3 维归一化(球面均匀),模长 = 均匀
    分布 [0, force_max_n](上限作用于向量模长,不是分量)。
    平滑:s(t) = (1-cos(pi t))/2,插值 f = f_from + s (f_to - f_from);
    范数球凸性 ⇒ |f| ≤ max(|f_from|,|f_to|) ≤ fmax,插值期间不越界。
    阶跃(step)与脉冲(pulse)模式:目标固定/按时窗开关,同样过平滑器。
    """

    def __init__(self, index, cfg, rng):
        self.index = index
        self.cfg = dict(cfg)
        self.rng = rng
        self.fmax = float(cfg.get("force_max_n", 19.6133))
        self.update_interval_s = float(cfg.get("update_interval_s", 2.0))
        self.transition_s = max(float(cfg.get("transition_s", 1.5)), 1e-3)
        self.frame = str(cfg.get("frame", "body"))          # body | world
        self.mode = str(cfg.get("mode", "off"))             # off|cont|step|pulse
        self.fixed_vec = np.asarray(cfg.get("fixed_vec", [0.0, 0.0, 0.0]),
                                    dtype=np.float64)
        self.pulse_on_s = float(cfg.get("pulse_on_s", 2.0))
        self.pulse_off_s = float(cfg.get("pulse_off_s", 2.0))
        # 状态
        self.f_from = np.zeros(3)
        self.f_to = np.zeros(3)
        self.t_sample = -1e9
        self.t_trans = -1e9
        self.t_gate = 0.0                     # pulse 相位计时起点
        self.current = np.zeros(3)
        self.n_samples = 0

    def _sample_target(self):
        if self.mode == "step":
            v = np.asarray(self.fixed_vec, dtype=np.float64)
            return np.clip(np.linalg.norm(v), 0.0, self.fmax) * \
                (v / max(np.linalg.norm(v), 1e-12))
        if self.mode == "pulse":
            return np.asarray(self.fixed_vec, dtype=np.float64)
        d = self.rng.normal(size=3)
        n = np.linalg.norm(d)
        if n < 1e-12:
            return np.zeros(3)
        mag = self.rng.uniform(0.0, self.fmax)
        return (d / n) * mag

    def update(self, sim_t):
        self._just_sampled = False
        if self.mode == "off":
            self.current = np.zeros(3)
            return
        if self.t_sample < -1e8:              # 首拍
            self.t_sample = sim_t
            self.t_trans = sim_t
            self.t_gate = sim_t
            if self.mode == "cont":
                self.f_to = self._sample_target()
                self.n_samples += 1
                self._just_sampled = True
        if self.mode == "pulse":
            ph = (sim_t - self.t_gate) % max(self.pulse_on_s +
                                             self.pulse_off_s, 1e-3)
            want = np.asarray(self.fixed_vec, dtype=np.float64) \
                if ph < self.pulse_on_s else np.zeros(3)
            if not np.allclose(self.f_to, want, atol=1e-9):
                self.f_from = self.current.copy()
                self.f_to = want
                self.t_trans = sim_t
        elif sim_t - self.t_sample >= self.update_interval_s:
            self.f_from = self.current.copy()
            self.f_to = self._sample_target()
            self.t_sample = sim_t
            self.t_trans = sim_t
            self.n_samples += 1
            self._just_sampled = True
        s = (sim_t - self.t_trans) / self.transition_s
        s = 0.0 if s < 0.0 else (1.0 if s > 1.0 else s)
        s = 0.5 - 0.5 * math.cos(math.pi * s)
        self.current = self.f_from + s * (self.f_to - self.f_from)
        m = float(np.linalg.norm(self.current))       # 数值保险,不应触发
        if m > self.fmax * (1.0 + 1e-6):
            self.current *= self.fmax / m

    def just_sampled(self):
        return getattr(self, "_just_sampled", False)

    def state(self):
        return {"mode": self.mode, "frame": self.frame,
                "fmax_n": self.fmax,
                "current": [round(float(v), 4) for v in self.current],
                "norm_n": round(float(np.linalg.norm(self.current)), 4),
                "interval_s": self.update_interval_s,
                "transition_s": self.transition_s,
                "n_samples": self.n_samples}


class DisturbanceManager(object):
    """两个作用点 prim 上的随机外力编排(种子可复现/序列回放)。

    作用点 prim(disturbance_0/1)是 base_link 下的纯 Xform:只取其世界
    位置施力,力施加到所属刚体(base_link),由 PhysX 自然生成力矩,
    不重复施加 r×F,也不新建任何刚体/碰撞体。
    """

    def __init__(self, bridge, cfg):
        self.bridge = bridge
        self.cfg = dict(cfg or {})
        self.enabled = bool(self.cfg.get("enabled", False))
        seed = self.cfg.get("seed")
        self.rng = (np.random.default_rng(int(seed))
                    if seed is not None else np.random.default_rng())
        paths = self.cfg.get("prim_paths", [
            "/World/Barracuda_2/barracuda2/base_link/disturbance_0",
            "/World/Barracuda_2/barracuda2/base_link/disturbance_1"])
        self.channels = [_DisturbChannel(i, self.cfg, self.rng)
                         for i in range(len(paths))]
        self.prim_paths = [str(p) for p in paths]
        self.history = []          # [(t_sim, [目标向量 per channel], frame)]
        self.replay_queue = None   # [(interval, [vec]*n)] 回放序列
        self.replay_idx = 0
        self.last_applied = []     # [(pos_world_m(3), f_world_n(3))] 供 viz/记录
        self.n_applied = 0

    # ------------------------------------------------------------ 控制
    def configure(self, patch):
        self.cfg.update(patch or {})
        self.enabled = bool(self.cfg.get("enabled", self.enabled))
        seed = self.cfg.get("seed")
        if patch.get("reseed", False) or (seed is not None and
                                          "seed" in patch):
            self.rng = np.random.default_rng(
                int(seed) if seed is not None else None)
        for ch in self.channels:
            for k in ("mode", "force_max_n", "update_interval_s",
                      "transition_s", "frame", "fixed_vec",
                      "pulse_on_s", "pulse_off_s"):
                if k in self.cfg:
                    ch.cfg[k] = self.cfg[k]
            ch.fmax = float(ch.cfg.get("force_max_n", ch.fmax))
            ch.update_interval_s = float(
                ch.cfg.get("update_interval_s", ch.update_interval_s))
            ch.transition_s = max(float(
                ch.cfg.get("transition_s", ch.transition_s)), 1e-3)
            ch.frame = str(ch.cfg.get("frame", ch.frame))
            ch.mode = str(ch.cfg.get("mode", ch.mode))
            if "fixed_vec" in ch.cfg:
                ch.fixed_vec = np.asarray(ch.cfg["fixed_vec"],
                                          dtype=np.float64)
            ch.t_sample = -1e9        # 重配置后下一拍重新采样
        return self.state()

    def stop(self):
        self.enabled = False
        for ch in self.channels:
            ch.mode = "off"
            ch.current = np.zeros(3)

    def set_replay(self, seq):
        """seq: [(interval_s, [vec0(3), vec1(3), ...]), ...],frame 用当前配置。"""
        self.replay_queue = list(seq or [])
        self.replay_idx = 0

    # ------------------------------------------------------------ 物理步
    def apply(self, sim_if, stage, R, com_num, fs, sim_t):
        self.last_applied = []
        if not self.enabled:
            return
        # 回放模式覆盖随机采样
        if self.replay_queue is not None and self.replay_idx < len(
                self.replay_queue):
            interval, vecs = self.replay_queue[self.replay_idx]
            due = (not self.channels or
                   sim_t - self.channels[0].t_sample >= interval - 1e-9)
            if due and self.channels:
                for ch, v in zip(self.channels, vecs):
                    ch.f_from = ch.current.copy()
                    ch.f_to = np.asarray(v, dtype=np.float64)
                    ch.t_trans = sim_t
                    ch.t_sample = sim_t
                self.replay_idx += 1
        sampled = False
        for i, ch in enumerate(self.channels):
            ch.update(sim_t)
            if ch.just_sampled():
                sampled = True
            path = self.prim_paths[i] if i < len(self.prim_paths) else None
            prim = stage.GetPrimAtPath(path) if path else None
            if prim is None or not prim.IsValid():
                _log_warn("扰动作用点 prim 无效: %r" % path)
                continue
            tm = self.bridge._usdgeom.Xformable(
                prim).ComputeLocalToWorldTransform(0)
            pos_num = np.asarray(tm.ExtractTranslation(), dtype=np.float64)
            f = ch.current
            if ch.frame == "body":
                f_world = R.dot(f)
            else:
                f_world = f
            if float(np.linalg.norm(f_world)) > 1e-6:
                fv = f_world * fs
                sim_if.apply_force_at_pos(
                    self.bridge._stage_id, self.bridge._body_id,
                    carb.Float3(fv[0], fv[1], fv[2]),
                    carb.Float3(pos_num[0], pos_num[1], pos_num[2]), "Force")
            self.last_applied.append(
                (pos_num * self.bridge._mpu, f_world))     # 米 / N
            self.n_applied += 1
        if sampled:
            self.history.append((sim_t,
                                 [c.f_to.copy() for c in self.channels],
                                 self.channels[0].frame if self.channels
                                 else "body"))
        if len(self.history) > 4000:
            self.history = self.history[-2000:]

    def record_targets(self, sim_t):
        """记录本拍采样目标(供回放;由 apply 外层在采样事件时调用)。"""
        self.history.append((sim_t, [c.f_to.copy() for c in self.channels],
                             self.channels[0].frame if self.channels
                             else "body"))

    # ------------------------------------------------------------ 状态
    def state(self):
        return {
            "enabled": self.enabled,
            "channels": [c.state() for c in self.channels],
            "prim_paths": self.prim_paths,
            "seed": self.cfg.get("seed"),
            "replay_progress": (None if self.replay_queue is None else
                                "%d/%d" % (self.replay_idx,
                                           len(self.replay_queue))),
            "history_len": len(self.history),
            "last_applied_world": [
                {"pos_m": [round(float(x), 4) for x in p],
                 "force_n": [round(float(x), 4) for x in f]}
                for p, f in self.last_applied],
        }


# ---------------------------------------------------------------------------
# Isaac 视窗实时可视化(debug_draw;不参与碰撞,不改物理)
# ---------------------------------------------------------------------------
class BridgeViz(object):
    """力箭头(推进器=蓝,扰动=红,力-长度同比例)+位姿箭头(当前=黄,
    目标=绿,定长 + 三轴姿态标)。长度 = |F| / newtons_per_meter,
    超过 max_arrow_m 时裁剪并置 clipped 标志(status 如实报告)。
    更新已有线段(清空重画),支持清理/暂停/关闭;频率可配。"""

    COLORS = {
        "thruster": (0.0, 0.25, 1.0, 1.0),     # 蓝
        "disturb": (1.0, 0.0, 0.0, 1.0),       # 红
        "current": (1.0, 0.9, 0.0, 1.0),       # 黄
        "goal": (0.0, 1.0, 0.2, 1.0),          # 绿
    }

    def __init__(self, bridge, cfg):
        self.bridge = bridge
        self.cfg = dict(cfg or {})
        self.enabled = bool(self.cfg.get("enabled", True))
        self.rate_hz = float(self.cfg.get("rate_hz", 10.0))
        self.n_per_m = float(self.cfg.get("newtons_per_meter", 40.0))
        self.pose_arrow_len_m = float(self.cfg.get("pose_arrow_len_m", 0.5))
        self.axis_len_m = float(self.cfg.get("axis_len_m", 0.15))
        self.max_arrow_m = float(self.cfg.get("max_arrow_m", 2.0))
        self.head_frac = 0.18
        self._dd = None
        self._last_draw_t = -1e9
        self._clipped = False
        self._n_lines = 0
        self.goal_pos = None
        self.goal_quat = None
        self._acquire()

    def _acquire(self):
        if self._dd is not None:
            return True
        try:
            from isaacsim.util.debug_draw import _debug_draw
            self._dd = _debug_draw.acquire_debug_draw_interface()
            return True
        except Exception as exc:               # noqa: BLE001
            _log_warn("debug_draw 不可用(可视化禁用): %s" % exc)
            self._dd = None
            return False

    def set_goal(self, pos_m, quat_xyzw):
        self.goal_pos = None if pos_m is None else np.asarray(
            pos_m, dtype=np.float64)
        self.goal_quat = None if quat_xyzw is None else np.asarray(
            quat_xyzw, dtype=np.float64)

    def clear(self):
        if self._dd is not None:
            try:
                self._dd.clear_lines()
            except Exception:                   # noqa: BLE001
                pass
        self._n_lines = 0
        self._clipped = False

    def _arrow_segments(self, start_m, vec_m, color):
        """一段箭头 = 主干 + 两撇箭头头;vec 长度即显示长度。返回
        (starts, ends, colors, widths, clipped)。长度单位:米(内部
        /mpu 转世界数值)。"""
        mpu = self.bridge._mpu
        L = float(np.linalg.norm(vec_m))
        if L < 1e-4:
            return [], False
        clipped = False
        if L > self.max_arrow_m:
            vec_m = vec_m * (self.max_arrow_m / L)
            L = self.max_arrow_m
            clipped = True
        d = vec_m / L
        tip = start_m + vec_m
        head_len = max(min(L * self.head_frac, 0.12), 0.02)
        # 与主干成 ±25° 的两撇
        side = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else \
            np.array([1.0, 0.0, 0.0])
        u = np.cross(d, side)
        u = u / max(np.linalg.norm(u), 1e-12)
        ang = math.radians(25.0)
        d1 = d * math.cos(ang) + u * math.sin(ang)
        d2 = d * math.cos(ang) - u * math.sin(ang)
        segs = []
        for a, b, c in ((start_m, tip, color),
                        (tip, tip - d1 * head_len, color),
                        (tip, tip - d2 * head_len, color)):
            segs.append((
                carb.Float3(a[0] / mpu, a[1] / mpu, a[2] / mpu),
                carb.Float3(b[0] / mpu, b[1] / mpu, b[2] / mpu),
                carb.Float4(*c)))
        return segs, clipped

    def update(self, sim_t, thruster_arrows, disturb_arrows, cur_pos_m,
               cur_quat, force=False):
        """thruster_arrows/disturb_arrows: [(pos_m(3), f_world_n(3))]。"""
        if not self.enabled:
            return
        now_ok = (sim_t - self._last_draw_t) >= 1.0 / max(self.rate_hz, 0.1)
        if not now_ok and not force:
            return
        if not self._acquire():
            return
        self._last_draw_t = sim_t
        mpu = self.bridge._mpu
        starts, ends, colors, widths = [], [], [], []
        self._clipped = False

        def add(segs_clipped):
            if not segs_clipped:
                return
            segs, clipped = segs_clipped
            self._clipped = self._clipped or clipped
            for a, b, c in segs:
                starts.append(a)
                ends.append(b)
                colors.append(c)
                widths.append(3.0)

        scale = 1.0 / max(self.n_per_m, 1e-6)
        for pos, f in thruster_arrows or []:
            if float(np.linalg.norm(f)) < 1e-3:
                continue                      # 零推力隐藏
            add(self._arrow_segments(np.asarray(pos), f * scale,
                                     self.COLORS["thruster"]))
        for pos, f in disturb_arrows or []:
            if float(np.linalg.norm(f)) < 1e-3:
                continue
            add(self._arrow_segments(np.asarray(pos), f * scale,
                                     self.COLORS["disturb"]))
        # 位姿箭头:体 +z(前)方向,定长,不代表推力
        for pos, q, color, tag in (
                (cur_pos_m, cur_quat, self.COLORS["current"], "current"),
                (self.goal_pos, self.goal_quat, self.COLORS["goal"], "goal")):
            if pos is None or q is None:
                continue
            R = _quat_to_matrix(_quat_normalize(np.asarray(q)))
            fwd = R.dot(np.array([0.0, 0.0, 1.0]))
            add(self._arrow_segments(
                np.asarray(pos), fwd * self.pose_arrow_len_m, color))
            # 三轴小姿态标(补充滚转信息):x/y/z 三根短线
            for ax_i in range(3):
                axis = R[:, ax_i] * self.axis_len_m
                tip = np.asarray(pos) + axis
                starts.append(carb.Float3(pos[0] / mpu, pos[1] / mpu,
                                          pos[2] / mpu))
                ends.append(carb.Float3(tip[0] / mpu, tip[1] / mpu,
                                        tip[2] / mpu))
                colors.append(carb.Float4(
                    color[0] * 0.6 + 0.4 * (ax_i == 0),
                    color[1] * 0.6 + 0.4 * (ax_i == 1),
                    color[2] * 0.6 + 0.4 * (ax_i == 2), 0.9))
                widths.append(1.5)
        try:
            self._dd.clear_lines()
            if starts:
                self._dd.draw_lines(starts, ends, colors, widths)
            self._n_lines = len(starts)
        except Exception as exc:               # noqa: BLE001
            _log_warn("viz draw 失败(本拍跳过): %s" % exc)

    def state(self):
        return {
            "enabled": self.enabled,
            "rate_hz": self.rate_hz,
            "newtons_per_meter": self.n_per_m,
            "pose_arrow_len_m": self.pose_arrow_len_m,
            "max_arrow_m": self.max_arrow_m,
            "clipped_last_frame": self._clipped,
            "n_lines_last_frame": self._n_lines,
            "goal_set": self.goal_pos is not None,
        }


# ---------------------------------------------------------------------------
# 主类
# ---------------------------------------------------------------------------

class AuvIsaacBridge(object):

    def __init__(self, config_path=None):
        self._config_path = config_path or DEFAULT_CONFIG
        self._script_path = globals().get("__file__", "")
        self._cfg = {}
        self._cfg_mtime = None

        import omni.usd
        import omni.physx
        import omni.timeline
        from omni.physx import get_physx_simulation_interface
        from pxr import Gf, UsdGeom, UsdPhysics, PhysicsSchemaTools
        from omni.usd import StageEventType
        self._omni_usd = omni.usd
        self._physx = omni.physx
        self._timeline = omni.timeline.get_timeline_interface()
        self._sim_if = get_physx_simulation_interface()
        self._gf = Gf
        self._usdgeom = UsdGeom
        self._usdphysics = UsdPhysics
        self._schematools = PhysicsSchemaTools
        self._stage_event_type = StageEventType

        self._lock = threading.Lock()
        self._active = True
        self._started_at = time.time()
        self._need_rederive = True

        # 命令/急停/测试推力(跨线程,由 _lock 保护)
        self._cmd = np.zeros(8)             # 最近 thruster_cmd(SI N)
        self._cmd_stamp_s = -1e9            # 命令对应的仿真时间
        self._cmd_seq = 0
        self._cmd_hz_counter = [0, time.monotonic()]
        self._cmd_hz = 0.0
        self._estop = False
        self._estop_reason = ""
        self._thrust_enabled = True
        self._test_until_sim = -1e9
        self._test_vec = np.zeros(8)

        # 真值差分基线
        self._have_prev = False
        self._prev_com_m = None
        self._prev_q = None
        self._prev_sim_s = None

        # stage 派生量(_derive_stage 填充)
        self._mpu = 0.01
        self._f_scale = 100.0
        self._gravity_num = np.zeros(3)        # 未知重力:零向量,浮力禁用
        self._up_axis = "?"
        self._stage_id = 0
        self._body_id = 0
        self._body_path_str = ""
        self._r_body_num = np.zeros((8, 3))   # 相对质心,世界数值尺度(= 米/mpu)
        self._d_body = np.zeros((8, 3))       # 单位推力方向,体系
        self._cb_body_num = np.zeros(3)
        self._mass = 40.0
        self._geom_source = "未解析"
        self._geom_check = ""
        self._physics_scenes = []      # _derive_stage 填充(全部 scene prim,不只第一个)
        self._gravity_source = ""      # 重力数值来源描述(可追溯)
        self._gravity_provisional = False  # 无 scene prim 时的假设值,Play 后复扫
        self._gravity_valid = False        # scene prim 重力值合法且已确认
        self._gravity_warning = ""         # 非法/未确认重力的明确告警(进 status)
        self._buoy_state = None            # 浮力实际启停(状态变化时各记一条日志)
        self._gravity_rechecks = 0         # 复扫次数上限(防大 stage 反复遍历)
        self._stage_file = ""          # 当前 stage 根层文件
        self._body_from = ""           # 刚体解析来源(config-exact / 根内扫描)
        self._body_detail = {}         # 质量API明细(authored 标记)
        self._thruster_detail = []     # 每推进器局部/世界变换明细
        self._thruster_paths_used = [] # 每推进器实际使用的 prim 路径及来源
        self._last_dt = None           # 最近物理步长(实测)
        self._last_state = None        # 最近发布的真值快照(pos/quat/v/w,只读诊断)

        self._sim_s = 0.0
        self._last_pub_s = -1e9
        self._consec_errors = 0
        self._last_error = ""
        self._time_jumps = 0
        self._steps = 0
        self._last_mode = "idle(未 Play)"
        self._last_cmd_age = None

        # [2026-11-08] 每物理步状态快照(训练/整定进程内路径用)
        self._step_state = None
        # [2026-11-08] 主线程操作队列:TCP 线程在物理运行中写 USD 会与
        # app.update() 的物理步并发死锁,排队到主线程安全点执行
        self._main_q = []
        self._main_q_lock = threading.Lock()
        # 每物理步实际施加的推进器力 [(pos_world_m(3), f_world_n(3))]x8(viz/记录共用)
        self._last_thruster_applied = []
        # 多写者防护状态
        self._thruster_pub_count = None
        self._multi_pub_fault = False

        self._ros = None
        self._srv_socket = None

        self._maybe_reload_config(force=True)
        self._derive_stage()

        # 物理步订阅与 stage 事件
        self._phys_sub = self._physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step)
        stream = self._omni_usd.get_context().get_stage_event_stream()
        self._stage_sub = stream.create_subscription_to_pop(
            self._on_stage_event, name="auv_bridge_stage_event")

        self._ros_start()

        # [2026-11-08] 扰动与可视化子系统(配置缺省也能工作)
        self._disturb = DisturbanceManager(self, self._cfg.get(
            "disturbance", {}))
        self._viz = BridgeViz(self, self._cfg.get("viz", {}))

        self._tcp_thread = threading.Thread(
            target=self._tcp_server, name="auv_bridge_tcp", daemon=True)
        self._tcp_thread.start()
        self._status_thread = threading.Thread(
            target=self._status_loop, name="auv_bridge_status", daemon=True)
        self._status_thread.start()

        _log("启动完成 config=%s" % self._config_path)

    # ------------------------------------------------------------------ 配置
    def _maybe_reload_config(self, force=False):
        try:
            mt = os.path.getmtime(self._config_path)
        except OSError:
            _log_warn("配置文件不存在: %s" % self._config_path)
            return False
        if not force and mt == self._cfg_mtime:
            return False
        try:
            with open(self._config_path, "r", encoding="utf-8") as f:
                self._cfg = json.load(f)
            self._cfg_mtime = mt
            _log("配置已加载(v%s)" % self._cfg.get("version"))
            return True
        except Exception as exc:            # noqa: BLE001
            _log_err("配置解析失败(保留旧配置): %s" % exc)
            return False

    def _hydro(self):
        return self._cfg.setdefault("hydrodynamics", {})

    # ------------------------------------------------------------------ stage
    def _derive_stage(self):
        """从当前 stage 只读推导单位/重力/刚体/几何。局部【米】->世界数值 = 米/mpu:
        /World 根缩放 S 与 mpu 在推导里消去(世界偏移数值 = S*raw,
        raw = 米/(S*mpu),故 = 米/mpu)。

        刚体解析策略(2026-11-03 按任务书收紧,此前误绑
        /World/gate_01/gate/base_link 的教训):
          1) 只认配置 robot_prim_path 的精确路径;
          2) 精确路径失效时,只在配置的 robot_root_paths 各根【内部】扫
             RigidBodyAPI prim(取路径最浅者),绝不全局扫描、绝不按名字
             猜——比赛道具(gate 等)在根外,结构上不可能被选中;
          3) 候选无 RigidBodyAPI、或所有根都不存在 -> 记错误并保持惰性
             (不施力),等人工改配置。"""
        try:
            self._body_path_str = ""            # 解析中途失败则保持惰性(勿用旧值/handle 0 加力)
            self._thruster_detail = []
            self._thruster_paths_used = []
            self._body_detail = {}
            self._physics_scenes = []
            self._gravity_provisional = False
            self._gravity_valid = False
            self._gravity_warning = ""
            self._gravity_rechecks = 0
            stage = self._omni_usd.get_context().get_stage()
            if stage is None:
                self._last_error = "无打开的 stage"
                return False
            try:
                self._stage_file = str(stage.GetRootLayer().identifier or "")
            except Exception:                   # noqa: BLE001
                self._stage_file = ""
            self._mpu = float(self._usdgeom.GetStageMetersPerUnit(stage) or 0.01)
            self._up_axis = str(self._usdgeom.GetStageUpAxis(stage) or "?")

            fus = self._cfg.get("units", {}).get("force_unit_scale", "auto")
            self._f_scale = (1.0 / self._mpu) if isinstance(fus, str) else float(fus)

            # ---- Physics Scene:全部列出(不止第一个),重力值必须显式校验 ----
            # 规则(2026-11-03 水平漂移事故后):
            #   scene 值合法   -> 直接采用,gravity_valid=true,浮力允许;
            #   scene 值非法   -> 显式告警并忽略其值,按 stage up 轴假设
            #                    (provisional=true,浮力禁用,绝不静默回退 -Y);
            #   无 scene prim  -> 同上假设(Play 后复扫,见 _recheck_gravity)。
            gvec = np.zeros(3)
            gsrc = ""
            gwarn = ""
            gvalid = False
            prov = True
            try:
                for prim in stage.TraverseAll():
                    if not prim.IsA(self._usdphysics.Scene):
                        continue
                    d = prim.GetAttribute("physics:gravityDirection").Get()
                    m = prim.GetAttribute("physics:gravityMagnitude").Get()
                    valid, reason, gv = _validate_scene_gravity(d, m)
                    try:
                        layers = [str(sp.layer.identifier)
                                  for sp in prim.GetPrimStack()]
                    except Exception:               # noqa: BLE001
                        layers = []
                    self._physics_scenes.append({
                        "path": str(prim.GetPath()),
                        "gravity_direction_raw": (
                            None if d is None else [float(v) for v in d]),
                        "gravity_magnitude_raw": (
                            None if m is None else float(m)),
                        "gravity_vector_numeric": (
                            None if gv is None else
                            [round(float(v), 4) for v in gv]),
                        "direction_authored": d is not None,
                        "magnitude_authored": m is not None,
                        "valid": valid,
                        "invalid_reason": reason,
                        "defining_layers": layers,
                    })
            except Exception:               # noqa: BLE001
                pass
            good = next((s for s in self._physics_scenes if s["valid"]), None)
            if good is not None:
                gvec = np.asarray(good["gravity_vector_numeric"],
                                  dtype=np.float64)
                gsrc = "scene prim %s(值合法,已确认)" % good["path"]
                gvalid, prov = True, False
            else:
                ax = {"Y": np.array([0.0, 1.0, 0.0]),
                      "Z": np.array([0.0, 0.0, 1.0])}.get(self._up_axis)
                if ax is not None:
                    gvec = -9.81 * ax
                    axis_note = "按 stage up 轴(-%s)假设" % self._up_axis
                else:
                    gvec = np.array([0.0, 0.0, -9.81])
                    axis_note = "stage up 轴未知,暂按 -Z 假设"
                if self._physics_scenes:
                    bad = self._physics_scenes[0]
                    gwarn = ("scene prim %s 重力属性非法: %s;已忽略其值,%s"
                             "(未确认,浮力禁用,Play 后复扫)"
                             % (bad["path"], bad["invalid_reason"], axis_note))
                else:
                    gwarn = ("无 PhysicsScene prim;%s(未确认,浮力禁用,"
                             "Play 后复扫)" % axis_note)
                gsrc = gwarn
                _log_warn("[重力] " + gwarn)
            if not np.all(np.isfinite(gvec)):   # 双保险(不应触发)
                gvec = np.zeros(3)
                gvalid, prov = False, True
                gwarn = (gwarn + ";合成向量非有限,已清零").lstrip(";")
            self._gravity_num = gvec
            self._gravity_source = gsrc
            self._gravity_valid = gvalid
            self._gravity_provisional = prov
            self._gravity_warning = gwarn if not gvalid else ""

            # ---- 机器人刚体(策略见 docstring;绝不全局扫描/按名猜) ----
            body_path = str(self._cfg.get("robot_prim_path", "") or "")
            roots = [str(r) for r in self._cfg.get("robot_root_paths", []) if r]
            prim = stage.GetPrimAtPath(body_path) if body_path else None
            if prim is not None and prim.IsValid():
                body_from = "config-exact"
            else:
                prim = None
                for root in roots:
                    rprim = stage.GetPrimAtPath(root)
                    if rprim is None or not rprim.IsValid():
                        continue
                    prefix = root.rstrip("/") + "/"
                    cands = [p for p in stage.TraverseAll()
                             if str(p.GetPath()).startswith(prefix) and
                             p.HasAPI(self._usdphysics.RigidBodyAPI)]
                    if cands:
                        cands.sort(key=lambda p: len(str(p.GetPath())))
                        prim = cands[0]
                        body_path = str(prim.GetPath())
                        body_from = "scan(%s 内最浅,候选 %d 个)" % (
                            root, len(cands))
                        _log_warn("robot_prim_path 无效,在根 %s 内扫到 %d 个 "
                                  "RigidBodyAPI,取最浅: %s" % (
                                      root, len(cands), body_path))
                        break
                if prim is None:
                    self._last_error = (
                        "机器人刚体未解析:配置路径 %r 无效,且根 %s 内无 "
                        "RigidBodyAPI prim;控制初始化停止(不施力)"
                        % (body_path, roots))
                    _log_err(self._last_error)
                    return False
            if not prim.HasAPI(self._usdphysics.RigidBodyAPI):
                self._last_error = ("%s 无 RigidBodyAPI(非动力学刚体),"
                                    "控制初始化停止" % body_path)
                _log_err(self._last_error)
                return False
            self._body_path_str = body_path
            self._body_from = body_from
            # 注:Isaac Sim 5.1.0 中 apply_force_at_pos 的 stage 句柄应取
            # UsdUtils.StageCache 的 id(NVIDIA 官方 exts 的统一做法),
            # 不能用路径哈希;body 句柄仍用 sdfPathToInt(prim 路径)。
            from pxr import UsdUtils
            self._stage_id = int(
                UsdUtils.StageCache.Get().GetId(stage).ToLongInt())
            self._body_id = self._schematools.sdfPathToInt(body_path)

            # ---- 质量 API 明细(只读;缺省字段如实标 None/authored=false)----
            from pxr import Usd as _Usd
            massapi = self._usdphysics.MassAPI(prim)
            mass_v = massapi.GetMassAttr().Get()
            dens_v = massapi.GetDensityAttr().Get()
            com_v = massapi.GetCenterOfMassAttr().Get()
            inertia_v = massapi.GetDiagonalInertiaAttr().Get()
            paxes_v = massapi.GetPrincipalAxesAttr().Get()
            rbapi = self._usdphysics.RigidBodyAPI(prim)
            kin_v = rbapi.GetKinematicEnabledAttr().Get()
            rbe_v = rbapi.GetRigidBodyEnabledAttr().Get()
            coll_paths = []
            for cp in _Usd.PrimRange(prim):
                if cp.HasAPI(self._usdphysics.CollisionAPI):
                    coll_paths.append(str(cp.GetPath()))
            self._body_detail = {
                "from": body_from,
                "rigid_body_api": True,
                "rigid_body_enabled": None if rbe_v is None else bool(rbe_v),
                "kinematic": None if kin_v is None else bool(kin_v),
                "mass_kg": None if mass_v is None else float(mass_v),
                "mass_authored": mass_v is not None,
                "density": None if dens_v is None else float(dens_v),
                "center_of_mass_authored": com_v is not None,
                "center_of_mass_local_num": (
                    None if com_v is None or
                    not all(math.isfinite(float(v)) for v in com_v) else
                    [round(float(v), 5) for v in com_v]),
                "diag_inertia_authored": inertia_v is not None,
                "diag_inertia": (
                    None if inertia_v is None else
                    [round(float(v), 5) for v in inertia_v]),
                "principal_axes_authored": paxes_v is not None,
                "articulation_root_api": bool(
                    prim.HasAPI(self._usdphysics.ArticulationRootAPI)),
                "collision_count": len(coll_paths),
                "collision_paths_head": coll_paths[:6],
            }
            self._mass = float(mass_v) if mass_v else 40.0

            # ---- 几何:从 prim 世界变换推导,并与配置对账 ----
            cfg_t = self._cfg.get("thrusters", {})
            pos_cfg = np.asarray(cfg_t.get("positions", [[0.0] * 3] * 8),
                                 dtype=np.float64)
            dir_cfg = np.asarray(cfg_t.get("directions", [[0.0] * 3] * 8),
                                 dtype=np.float64)
            com_m = np.asarray(self._cfg.get("com_base_link_m", [0, 0, 0]),
                               dtype=np.float64)

            tm = self._usdgeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            trans_num = np.asarray(tm.ExtractTranslation(), dtype=np.float64)
            # 旋转一律极分解提取:含缩放祖先链上 Gf 的四元数提取会损坏
            R_bl = _proper_rot(_gf_mat3_to_np(tm, self._gf))
            q_bl = _matrix_to_quat(R_bl)

            # 质心世界数值:局部米值 -> 世界数值 = 米/mpu(缩放消去,见 docstring)
            com_num = trans_num + R_bl.dot(com_m / self._mpu)

            paths = list(self._cfg.get("thruster_prim_paths", []))
            r_derived, d_derived = [], []
            ok = len(paths) == 8
            valid_roots = [r for r in roots
                           if stage.GetPrimAtPath(r) is not None and
                           stage.GetPrimAtPath(r).IsValid()]
            root_basename_cache = {}

            def _find_named(root, name):
                """在指定根内部按 basename 精确找 prim(缓存,深度优先首个)。"""
                cache = root_basename_cache.get(root)
                if cache is None:
                    cache = {}
                    rp = stage.GetPrimAtPath(root)
                    for p in _Usd.PrimRange(rp):
                        bn = p.GetName()
                        if bn not in cache:
                            cache[bn] = p
                    root_basename_cache[root] = cache
                return cache.get(name)

            if ok:
                for i, tp in enumerate(paths):
                    t_prim = stage.GetPrimAtPath(tp)
                    src = "config-exact"
                    if t_prim is None or not t_prim.IsValid():
                        t_prim = None
                        for root in valid_roots:
                            hit = _find_named(root, "thruster_%d" % i)
                            if hit is not None:
                                t_prim, src = hit, "scan(%s 内按名)" % root
                                break
                    if t_prim is None or not t_prim.IsValid():
                        ok = False
                        _log_warn("推进器 %d 的 prim 不可用(配置 %r,根 %s 内也无 "
                                  "thruster_%d)" % (i, tp, valid_roots, i))
                        break
                    self._thruster_paths_used.append({
                        "index": i, "configured": tp,
                        "used": str(t_prim.GetPath()), "source": src})
                    t_tm = self._usdgeom.Xformable(
                        t_prim).ComputeLocalToWorldTransform(0)
                    p_num = np.asarray(t_tm.ExtractTranslation(), dtype=np.float64)
                    t_R = _proper_rot(_gf_mat3_to_np(t_tm, self._gf))
                    t_q = _matrix_to_quat(t_R)
                    # 体系方向 = R_bl^T * (推进器局部 +X 的世界方向)
                    d_i = R_bl.T.dot(t_R.dot(np.array([1.0, 0.0, 0.0])))
                    d_derived.append(d_i)
                    # 相对质心(世界数值尺度),存体系分量
                    r_derived.append(R_bl.T.dot(p_num - com_num))
                    entry = {
                        "index": i,
                        "path": str(t_prim.GetPath()),
                        "path_source": src,
                        "world_t_m": [round(v, 4) for v in (p_num * self._mpu)],
                        "world_q_xyzw": [round(v, 4) for v in t_q],
                        "r_from_base_link_origin_body_m": [
                            round(v, 4) for v in
                            (R_bl.T.dot(p_num - trans_num) * self._mpu)],
                        "r_from_com_body_m": [
                            round(v, 4) for v in
                            (R_bl.T.dot(p_num - com_num) * self._mpu)],
                        "dir_body_local_plus_x": [round(v, 4) for v in d_i],
                        "dir_cfg": [round(v, 4) for v in dir_cfg[i]],
                    }
                    try:
                        ltm = self._usdgeom.Xformable(
                            t_prim).GetLocalTransformation()
                        lt = ltm.ExtractTranslation()
                        lq = ltm.ExtractRotationQuat()
                        entry["local_t_num"] = [round(v, 4) for v in lt]
                        entry["local_q_xyzw"] = [
                            round(v, 4) for v in
                            (lq.imaginary[0], lq.imaginary[1],
                             lq.imaginary[2], lq.real)]
                    except Exception:           # noqa: BLE001
                        entry["local_t_num"] = None    # API 不可用时只报世界系
                        entry["local_q_xyzw"] = None
                    self._thruster_detail.append(entry)
            if ok:
                r_derived = np.asarray(r_derived)
                d_derived = np.asarray(d_derived)
                d_derived = np.asarray(
                    [v / (np.linalg.norm(v) or 1.0) for v in d_derived])
                r_cfg_num = pos_cfg / self._mpu
                dr = float(np.max(np.linalg.norm(r_derived - r_cfg_num, axis=1)))
                dd = float(np.max(np.abs(d_derived - dir_cfg)))
                self._geom_check = ("|Δr|max=%.4f stage数值(=%.2f mm), "
                                    "|Δdir|max=%.4f" % (dr, dr * self._mpu * 1000, dd))
                self._r_body_num = r_derived
                self._d_body = d_derived
                self._geom_source = "stage-derived(prim 变换)"
            else:
                self._r_body_num = pos_cfg / self._mpu
                self._d_body = dir_cfg
                self._geom_source = "config(prim 不可用)"
                self._geom_check = "推导失败,用配置几何"
                _log_warn("推进器 prim 推导失败,使用配置几何")

            cb_m = np.asarray(self._hydro().get("cb_offset_body_m", [0, 0, 0]),
                              dtype=np.float64)
            self._cb_body_num = cb_m / self._mpu

            self._have_prev = False
            self._last_error = ""       # 成功解析,清除历史错误(status 不留旧账)
            _log("stage 解析: up=%s mpu=%.4g f_scale=%.4g mass=%.1f g=%s "
                 "body=%s geom=%s %s" % (
                     self._up_axis, self._mpu, self._f_scale, self._mass,
                     np.round(self._gravity_num, 3).tolist(),
                     self._body_path_str, self._geom_source, self._geom_check))
            return True
        except Exception as exc:            # noqa: BLE001
            self._body_path_str = ""        # 失败时清空,物理步保持惰性
            self._last_error = "stage 解析异常: %s" % exc
            _log_err(self._last_error + "\n" + traceback.format_exc())
            return False

    def _recheck_gravity(self, stage):
        """无 PhysicsScene(或值非法)时,Play 后 kit 会自动创建 scene;
        只有值【合法】的 scene 才采用(只读,不写场景)。非法/缺失一律保持
        "未确认"状态(up 轴假设 + 浮力禁用 + 显式告警),下个周期再试。"""
        try:
            scenes = []
            adopted = None
            for prim in stage.TraverseAll():
                if not prim.IsA(self._usdphysics.Scene):
                    continue
                d = prim.GetAttribute("physics:gravityDirection").Get()
                m = prim.GetAttribute("physics:gravityMagnitude").Get()
                valid, reason, gv = _validate_scene_gravity(d, m)
                try:
                    layers = [str(sp.layer.identifier)
                              for sp in prim.GetPrimStack()]
                except Exception:                   # noqa: BLE001
                    layers = []
                entry = {
                    "path": str(prim.GetPath()),
                    "gravity_direction_raw": (
                        None if d is None else [float(v) for v in d]),
                    "gravity_magnitude_raw": (
                        None if m is None else float(m)),
                    "gravity_vector_numeric": (
                        None if gv is None else
                        [round(float(v), 4) for v in gv]),
                    "direction_authored": d is not None,
                    "magnitude_authored": m is not None,
                    "valid": valid,
                    "invalid_reason": reason,
                    "defining_layers": layers,
                }
                scenes.append(entry)
                if valid and adopted is None:
                    adopted = entry
            if not scenes:
                return
            self._physics_scenes = scenes
            if adopted is not None:
                self._gravity_num = np.asarray(
                    adopted["gravity_vector_numeric"], dtype=np.float64)
                self._gravity_source = ("scene prim %s(值合法,已确认)"
                                        % adopted["path"])
                self._gravity_valid = True
                self._gravity_provisional = False
                self._gravity_warning = ""
                _log("[重力] 已确认 scene %s g=%s" % (
                    adopted["path"], np.round(self._gravity_num, 3).tolist()))
            elif not self._gravity_valid:
                bad = scenes[0]
                warn = ("scene prim %s 重力属性非法: %s;保持 up 轴假设"
                        "(未确认,浮力禁用)" % (bad["path"],
                                               bad["invalid_reason"]))
                self._gravity_valid = False
                self._gravity_provisional = True
                if warn != self._gravity_warning:   # 告警去重(复扫最多 50 次)
                    self._gravity_warning = warn
                    self._gravity_source = warn
                    _log_warn("[重力] " + warn)
        except Exception:                   # noqa: BLE001
            pass

    def _on_stage_event(self, event):
        et = int(event.type)
        if et in (int(self._stage_event_type.OPENED),
                  int(self._stage_event_type.CLOSING)):
            _log("stage 事件 %d,下一物理步重新解析" % et)
            self._need_rederive = True

    # ------------------------------------------------------------------ ROS
    def _ros_start(self):
        try:
            if not bool(self._cfg.get("ros", {}).get("enabled", True)):
                _log("ROS 已按配置禁用(进程内训练/整定路径)")
                return False
            sys.path[:] = [p for p in sys.path if "/opt/ros" not in p]
            if KIT_RCLPY_ROOT not in sys.path:
                sys.path.insert(0, KIT_RCLPY_ROOT)
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.context import Context
            from rclpy.qos import qos_profile_sensor_data
            from std_msgs.msg import Float64MultiArray, Bool, String
            from nav_msgs.msg import Odometry
            from rosgraph_msgs.msg import Clock
            from geometry_msgs.msg import PoseStamped
            # 注:isaacsim.ros2.bridge 4.12.4(内部 jazzy 构建发布 DiagnosticArray 会在
            # C 层断言失败 (PyBytes_Check) 直接 abort 整个 kit 进程(2026-11-03
            # 实测复现)。故 /auv/bridge/status 改用 std_msgs/String 承载与 TCP
            # status 相同的 JSON,话题名不变。

            ctx = Context()
            rclpy.init(context=ctx)
            node = rclpy.create_node("auv_isaac_bridge", context=ctx)

            t = self._cfg.get("ros_topics", {})
            q = qos_profile_sensor_data
            ros = dict(
                rclpy=rclpy, ctx=ctx,
                Odometry=Odometry, Clock=Clock, String=String,
                pub_odom_gt=node.create_publisher(Odometry, t.get(
                    "ground_truth_odom", "/auv/ground_truth/odometry"), q),
                pub_state=node.create_publisher(Odometry, t.get(
                    "state_control", "/auv/state/control"), q),
                pub_clock=node.create_publisher(Clock, t.get("clock", "/clock"), 10),
                pub_status=node.create_publisher(String, t.get(
                    "bridge_status", "/auv/bridge/status"), 10),
                pub_applied=node.create_publisher(Float64MultiArray, t.get(
                    "applied_forces", "/auv/bridge/applied_forces"), q),
                pub_disturb=node.create_publisher(Float64MultiArray, t.get(
                    "disturbance_forces", "/auv/disturbance/forces"), q),
            )
            node.create_subscription(
                Float64MultiArray, t.get("thruster_cmd", "/auv/control/thruster_cmd"),
                self._on_thruster_cmd, q)
            node.create_subscription(
                Bool, t.get("estop", "/auv/control/estop"), self._on_estop_ros, 10)
            # 可视化只读目标(绿箭头);不与控制器争用任何命令话题
            node.create_subscription(
                PoseStamped, t.get("goal", "/auv/control/goal"),
                self._on_goal_ros, 10)

            ex = SingleThreadedExecutor(context=ctx)
            ex.add_node(node)
            ros["executor"] = ex
            ros["node"] = node
            threading.Thread(target=ex.spin, name="auv_bridge_ros",
                             daemon=True).start()
            self._ros = ros
            _log("ROS 桥已启动(kit 自带 rclpy)")
            return True
        except Exception as exc:            # noqa: BLE001
            self._ros = None
            self._last_error = "ROS 初始化失败: %s" % exc
            _log_err(self._last_error + "\n" + traceback.format_exc())
            return False

    def _on_thruster_cmd(self, msg):
        with self._lock:
            if len(msg.data) != 8:
                _log_warn("thruster_cmd 长度 %d != 8,丢弃" % len(msg.data))
                return
            arr = np.asarray(msg.data, dtype=np.float64)
            if not np.all(np.isfinite(arr)):
                _log_warn("thruster_cmd 含 NaN/Inf,丢弃")
                return
            t = self._cfg.get("thrusters", {})
            arr = np.clip(arr, float(t.get("force_min_n", -39.23)),
                          float(t.get("force_max_n", 39.23)))
            self._cmd = arr
            self._cmd_stamp_s = self._sim_s
            self._cmd_seq += 1
            self._cmd_hz_counter[0] += 1

    def _on_estop_ros(self, msg):
        with self._lock:
            if msg.data and not self._estop:
                self._estop = True
                self._estop_reason = "ROS /auv/control/estop"
                self._cmd = np.zeros(8)
                self._test_until_sim = -1e9
                _log_warn("急停(ROS)")
            elif not msg.data and self._estop and \
                    self._estop_reason.startswith("ROS"):
                self._estop = False
                self._estop_reason = ""
                _log("急停解除(ROS)")

    def _on_goal_ros(self, msg):
        """只读目标位姿(可视化绿箭头 + step_state 快照附带)。"""
        try:
            pos = np.array([msg.pose.position.x, msg.pose.position.y,
                            msg.pose.position.z], dtype=np.float64)
            q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                          msg.pose.orientation.z, msg.pose.orientation.w],
                         dtype=np.float64)
            self._viz.set_goal(pos, _quat_normalize(q))
        except Exception:                           # noqa: BLE001
            pass

    # ------------------------------------------------------------- 公共 API
    def set_command(self, vec8_si):
        """进程内命令入口(训练/整定):与 ROS 路径同一校验/限幅。"""
        fake = _FakeCmdMsg(vec8_si)
        self._on_thruster_cmd(fake)

    def reset_output(self):
        """清零命令与测试推力(复位流程调用)。"""
        with self._lock:
            self._cmd = np.zeros(8)
            self._cmd_stamp_s = -1e9
            self._test_until_sim = -1e9

    def get_state_snapshot(self):
        """最近物理步状态 dict(pos_m/quat_xyzw/v_body/w_body/sim_time_s)。
        None = 尚无物理步。"""
        return self._step_state

    def set_robot_pose(self, pos_m=None, quat_xyzw=None):
        """set_pose 的进程内版本(写 USD 变换,试验复位用)。"""
        req = {}
        if pos_m is not None:
            req["pos_m"] = list(pos_m)
        if quat_xyzw is not None:
            req["quat_xyzw"] = list(quat_xyzw)
        return self._cmd_set_pose(req)

    # ------------------------------------------------------------------ 物理步
    def _on_physics_step(self, dt):
        try:
            if not self._active:
                return
            self.drain_main_ops()
            if self._need_rederive:
                self._need_rederive = False
                self._derive_stage()

            self._last_dt = float(dt)
            if not self._body_path_str:
                return          # 刚体未解析:保持惰性,绝不拿空路径/handle 0 施力
            stage = self._omni_usd.get_context().get_stage()
            if stage is None:
                return
            prim = stage.GetPrimAtPath(self._body_path_str)
            if prim is None or not prim.IsValid():
                return
            if self._gravity_provisional and self._steps and \
                    self._steps % 30 == 0 and \
                    self._gravity_rechecks < 50:
                self._gravity_rechecks += 1
                self._recheck_gravity(stage)

            # 时间与回跳
            t_now = float(self._timeline.get_current_time())
            if t_now + 1e-6 < self._sim_s:
                self._time_jumps += 1
                with self._lock:
                    self._cmd = np.zeros(8)
                    self._cmd_stamp_s = -1e9
                    self._test_until_sim = -1e9
                self._have_prev = False
                self._last_pub_s = -1e9
                self._prev_sim_s = None
                _log_warn("仿真时间回跳 %.3f -> %.3f,清空命令" % (self._sim_s, t_now))
            self._sim_s = t_now
            self._steps += 1

            # 位姿
            tm = self._usdgeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            trans_num = np.asarray(tm.ExtractTranslation(), dtype=np.float64)
            R = _proper_rot(_gf_mat3_to_np(tm, self._gf))
            q_now = _matrix_to_quat(R)
            com_m_cfg = np.asarray(self._cfg.get("com_base_link_m", [0, 0, 0]),
                                   dtype=np.float64)
            com_num = trans_num + R.dot(com_m_cfg / self._mpu)
            com_m = com_num * self._mpu

            # 速度:相邻步差分(体坐标系,质心)
            v_body = np.zeros(3)
            w_body = np.zeros(3)
            if self._have_prev and self._prev_sim_s is not None:
                dts = t_now - self._prev_sim_s
                if dts > 1e-6:
                    v_world = (com_m - self._prev_com_m) / dts
                    v_body = R.T.dot(v_world)
                    w_body = R.T.dot(_angular_velocity_from_quats(
                        self._prev_q, q_now, dts))
            self._prev_com_m = com_m
            self._prev_q = q_now
            self._prev_sim_s = t_now
            self._have_prev = True

            # 发布(限频)
            hz = float(self._cfg.get("timing", {}).get("state_publish_hz", 60.0))
            publish_now = (self._ros is not None and
                           (t_now - self._last_pub_s) >=
                           1.0 / max(hz, 1.0) - 1e-9)
            if publish_now:
                self._last_pub_s = t_now
                self._publish_state(com_m, q_now, v_body, w_body, t_now)

            # 输出决策
            timeout = float(self._cfg.get("timing", {}).get("command_timeout_s", 0.5))
            with self._lock:
                estop = self._estop
                thrust_on = self._thrust_enabled
                test_active = t_now < self._test_until_sim
                test_vec = self._test_vec.copy()
                cmd = self._cmd.copy()
                cmd_age = t_now - self._cmd_stamp_s
                seq = self._cmd_seq
            if self._multi_pub_fault:
                u, mode = np.zeros(8), "fault(多写者 %s)" % self._thruster_pub_count
            elif estop or not thrust_on:
                u, mode = np.zeros(8), ("estop(%s)" % self._estop_reason
                                        if estop else "thrust_off")
            elif test_active:
                u, mode = test_vec, "test_thrust"
            elif seq == 0:
                u, mode = np.zeros(8), "zero(无命令)"
            elif cmd_age > timeout:
                u, mode = np.zeros(8), "zero(命令过期 %.2fs)" % cmd_age
            else:
                u, mode = cmd, "tracking"

            self._apply_forces(R, com_num, u, v_body, w_body)
            # [2026-11-08] 每步快照(训练/整定进程内读状态用)
            self._step_state = {
                "sim_time_s": float(t_now),
                "pos_m": com_m.tolist(),
                "quat_xyzw": q_now.tolist(),
                "v_body": v_body.tolist(),
                "w_body": w_body.tolist(),
            }
            # 扰动力(独立于 estop:环境外力,由 disturb_config 显式启停)
            try:
                self._disturb.apply(self._sim_if, stage, R, com_num,
                                    self._f_scale, t_now)
            except Exception as exc:               # noqa: BLE001
                self._consec_errors += 1
                self._last_error = "扰动施加异常: %s" % exc
                if self._consec_errors % 50 == 1:
                    _log_err(self._last_error)
            # 可视化(与施力模块共用 last_applied 数据,不另算)
            try:
                self._viz.update(
                    t_now,
                    thruster_arrows=self._last_thruster_applied,
                    disturb_arrows=self._disturb.last_applied,
                    cur_pos_m=com_m, cur_quat=q_now)
            except Exception:                       # noqa: BLE001
                pass
            if publish_now and self._ros is not None:
                self._publish_aux_forces(u, t_now)
            self._last_mode = mode
            self._last_cmd_age = None if seq == 0 else cmd_age
            self._consec_errors = 0
        except Exception as exc:            # noqa: BLE001
            self._consec_errors += 1
            self._last_error = "物理步异常: %s" % exc
            if self._consec_errors <= 3 or self._consec_errors % 100 == 0:
                _log_err(self._last_error + "\n" + traceback.format_exc())
            if self._consec_errors > 600:
                self._active = False
                _log_err("连续错误过多,桥停用;请重新 bootstrap 或 TCP restart")

    def _apply_forces(self, R, com_num, u_si, v_body, w_body):
        hydro = self._hydro()
        fs = self._f_scale
        sim = self._sim_if

        # 推力:作用点=质心+R*r(世界数值),方向=R*d,大小=SI*f_scale
        self._last_thruster_applied = []
        for i in range(8):
            fi = float(u_si[i])
            pos = com_num + R.dot(self._r_body_num[i])
            if abs(fi) >= 0.02:
                fvec = R.dot(self._d_body[i]) * (fi * fs)
                sim.apply_force_at_pos(
                    self._stage_id, self._body_id,
                    carb.Float3(fvec[0], fvec[1], fvec[2]),
                    carb.Float3(pos[0], pos[1], pos[2]), "Force")
                self._last_thruster_applied.append(
                    (pos * self._mpu, R.dot(self._d_body[i]) * fi))
            else:
                self._last_thruster_applied.append(
                    (pos * self._mpu, R.dot(self._d_body[i]) * 0.0))

        if not hydro.get("enabled", True):
            return

        # ---- 浮力:只在重力"已确认"(scene prim 值合法)时才施加 ----
        # 2026-11-03 事故:自动 scene 带垃圾值(dir=0/mag=-Inf),旧代码按
        # 假设重力把浮力指向水平 +Y,392N 把艇推漂 20+ m。此后假设重力
        # 一律禁浮力(阻尼照常),状态迁移各记一条日志。
        g_known = self._gravity_valid
        if g_known != self._buoy_state:
            self._buoy_state = g_known
            if g_known:
                _log("[浮力] 重力已确认,浮力补偿启用(方向=-g)")
            else:
                _log_warn("[浮力] 重力未确认(%s),浮力补偿禁用,仅施加阻尼"
                          % (self._gravity_warning or self._gravity_source
                             or "重力未知"))
        # 浮力(与场景重力数值精确抵消 -> 单位自洽)
        bcfg = hydro.get("buoyancy", {})
        mode = bcfg.get("mode", "neutral_compensated")
        g = self._gravity_num
        gmag = float(np.linalg.norm(g))
        bmag_num = 0.0
        if not g_known:
            bmag_num = 0.0            # 未确认:绝不按假设重力施浮力
        elif mode == "neutral_compensated" and gmag > 0.0:
            bmag_num = (self._mass * gmag *
                        float(bcfg.get("ratio", 1.0)) * fs)  # N -> 数值
        elif mode == "displaced_volume" and gmag > 0.0:
            # 物理浮力依赖重力(零重力下无浮力),故 gmag=0 时同样为 0
            rho = float(bcfg.get("water_density_kg_m3", 1000.0))
            vol = float(bcfg.get("displaced_volume_m3", 0.0134))
            bmag_num = rho * vol * 9.81 * fs
        if bmag_num > 0.0:
            cb = com_num + R.dot(self._cb_body_num)
            fb = (-g / gmag) * bmag_num if gmag > 0 else np.zeros(3)
            sim.apply_force_at_pos(
                self._stage_id, self._body_id,
                carb.Float3(fb[0], fb[1], fb[2]),
                carb.Float3(cb[0], cb[1], cb[2]), "Force")

        # 阻尼(体系;线力作用于质心,角量用力矩;SI -> 数值)
        dc = hydro.get("damping", {})
        dl = np.asarray(dc.get("linear", [0, 0, 0]), dtype=np.float64)
        dq = np.asarray(dc.get("quadratic", [0, 0, 0]), dtype=np.float64)
        al = np.asarray(dc.get("angular_linear", [0, 0, 0]), dtype=np.float64)
        aq = np.asarray(dc.get("angular_quadratic", [0, 0, 0]), dtype=np.float64)
        f_d_si = -(dl * v_body + dq * np.abs(v_body) * v_body)
        if np.linalg.norm(f_d_si) > 1e-6:
            f_d = R.dot(f_d_si) * fs
            sim.apply_force_at_pos(
                self._stage_id, self._body_id,
                carb.Float3(f_d[0], f_d[1], f_d[2]),
                carb.Float3(com_num[0], com_num[1], com_num[2]), "Force")
        t_d_si = -(al * w_body + aq * np.abs(w_body) * w_body)
        if np.linalg.norm(t_d_si) > 1e-6:
            t_d = t_d_si * fs
            sim.apply_torque(
                self._stage_id, self._body_id,
                carb.Float3(t_d[0], t_d[1], t_d[2]))

    # ------------------------------------------------------------------ 发布
    def _publish_state(self, com_m, q, v_body, w_body, t_now):
        ros = self._ros
        now_ns = int(t_now * 1e9)
        odom = ros["Odometry"]()
        odom.header.stamp.sec = now_ns // 1_000_000_000
        odom.header.stamp.nanosec = now_ns % 1_000_000_000
        odom.header.frame_id = "auv_world"       # 世界系(场景全局)
        odom.child_frame_id = "auv_com"          # 质心体系(MPPI 体坐标约定)
        odom.pose.pose.position.x = float(com_m[0])
        odom.pose.pose.position.y = float(com_m[1])
        odom.pose.pose.position.z = float(com_m[2])
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])
        cov = [0.0] * 36
        for i in range(6):
            cov[6 * i + i] = 0.01 if i < 3 else 0.02
        odom.pose.covariance = cov
        odom.twist.twist.linear.x = float(v_body[0])
        odom.twist.twist.linear.y = float(v_body[1])
        odom.twist.twist.linear.z = float(v_body[2])
        odom.twist.twist.angular.x = float(w_body[0])
        odom.twist.twist.angular.y = float(w_body[1])
        odom.twist.twist.angular.z = float(w_body[2])
        odom.twist.covariance = cov
        ros["pub_odom_gt"].publish(odom)
        ros["pub_state"].publish(odom)

        clk = ros["Clock"]()
        clk.clock.sec = now_ns // 1_000_000_000
        clk.clock.nanosec = now_ns % 1_000_000_000
        ros["pub_clock"].publish(clk)

        # 只读快照(TCP status 暴露,零命令漂移诊断用)
        self._last_state = {
            "sim_time_s": round(float(t_now), 3),
            "pos_m": [round(float(v), 4) for v in com_m],
            "quat_xyzw": [round(float(v), 5) for v in q],
            "v_body": [round(float(v), 4) for v in v_body],
            "w_body": [round(float(v), 4) for v in w_body],
        }

    def _publish_aux_forces(self, u_si, t_now):
        """实际施加的 8 路推力(N,含清零/test 模式)+ 扰动力(12 元组)。"""
        ros = self._ros
        try:
            m = ros["Float64MultiArray"]() \
                if "Float64MultiArray" in ros else None
        except Exception:                               # noqa: BLE001
            m = None
        if m is None:
            # ros dict 未存类型则动态构造(保持与 _ros_start 一致的导入)
            from std_msgs.msg import Float64MultiArray as _F
            ros["Float64MultiArray"] = _F
            m = _F()
        m.data = [float(x) for x in u_si]
        ros["pub_applied"].publish(m)

        d = ros["Float64MultiArray"]()
        data = []
        for pos, f in self._disturb.last_applied:
            data.extend([float(f[0]), float(f[1]), float(f[2])])
        for pos, f in self._disturb.last_applied:
            data.extend([float(pos[0]), float(pos[1]), float(pos[2])])
        # 定长 12 元:无扰动时也发布全零,订阅端记录/离线分析拿到的
        # 行结构恒定(空消息会让 CSV 行退化成 1 列,见 analyze 加载器)
        while len(data) < 12:
            data.append(0.0)
        d.data = data
        ros["pub_disturb"].publish(d)

    def _check_thruster_publishers(self):
        """多写者防护:thruster_cmd 发布者数 >1 且配置强制时输出置零。"""
        if self._ros is None:
            return
        try:
            topic = self._cfg.get("ros_topics", {}).get(
                "thruster_cmd", "/auv/control/thruster_cmd")
            n = int(self._ros["node"].count_publishers(topic))
            enforce = bool(self._cfg.get("safety", {}).get(
                "enforce_single_controller", True))
            self._thruster_pub_count = n
            if n > 1 and enforce:
                if not self._multi_pub_fault:
                    _log_err("thruster_cmd 有 %d 个发布者(应唯一),"
                             "输出置零直至恢复单一写者" % n)
                self._multi_pub_fault = True
            elif self._multi_pub_fault and n <= 1:
                _log("thruster_cmd 写者数恢复为 %d,解除 fault" % n)
                self._multi_pub_fault = False
        except Exception:                               # noqa: BLE001
            pass

    def _build_status(self):
        with self._lock:
            estop, reason, seq = self._estop, self._estop_reason, self._cmd_seq
        c, t0 = self._cmd_hz_counter
        now = time.monotonic()
        if now - t0 > 1.0:
            self._cmd_hz = c / (now - t0)
            self._cmd_hz_counter = [0, now]
        age = self._last_cmd_age
        return {
            "active": self._active,
            "uptime_s": round(time.time() - self._started_at, 1),
            "sim_time_s": round(self._sim_s, 3),
            "steps": self._steps,
            "mode": self._last_mode,
            "estop": estop,
            "estop_reason": reason,
            "cmd_seq": seq,
            "cmd_age_s": None if age is None else round(age, 3),
            "cmd_rate_hz": round(self._cmd_hz, 1),
            "time_jumps": self._time_jumps,
            "stage": {
                "up_axis": self._up_axis,
                "meters_per_unit": self._mpu,
                "force_scale": self._f_scale,
                "gravity_numeric": [round(v, 3) for v in self._gravity_num],
                "gravity_source": self._gravity_source,
                "gravity_valid": self._gravity_valid,
                "gravity_warning": self._gravity_warning,
                "zero_gravity": bool(self._gravity_valid and
                                     float(np.linalg.norm(
                                         self._gravity_num)) < 1e-6),
                "buoyancy_active": bool(
                    self._gravity_valid and
                    self._hydro().get("enabled", True) and
                    self._hydro().get("buoyancy", {}).get(
                        "mode", "neutral_compensated") not in (
                        "off", "none", "disabled")),
                "physics_scene": (self._physics_scenes[0]["path"]
                                  if self._physics_scenes else None),
                "stage_file": (self._stage_file.rsplit("/", 1)[-1]
                               if self._stage_file else ""),
                "body_path": self._body_path_str,
                "mass_kg": self._mass,
                "geom_source": self._geom_source,
                "geom_check": self._geom_check,
            },
            "ros_ok": self._ros is not None,
            "consec_errors": self._consec_errors,
            "last_error": self._last_error,
            "last_state": self._last_state,
            "disturbance": self._disturb.state(),
            "viz": self._viz.state(),
            "thruster_cmd_publishers": self._thruster_pub_count,
            "multi_pub_fault": self._multi_pub_fault,
        }

    def _status_loop(self):
        while self._active:
            time.sleep(1.0)
            try:
                self._maybe_reload_config()
                if self._ros is None:
                    self._ros_start()
                    continue
                self._check_thruster_publishers()
                st = self._build_status()
                ros = self._ros
                # DiagnosticArray 在此 ext 会触发 C 层断言崩溃(见 _ros_start 注释),
                # 改发与 TCP status 相同结构的 JSON 字符串(std_msgs/String)。
                msg = ros["String"]()
                msg.data = json.dumps(st, ensure_ascii=False, default=str)[:4000]
                ros["pub_status"].publish(msg)
            except Exception as exc:        # noqa: BLE001
                _log_err("status 线程异常: %s" % exc)

    # ------------------------------------------------------------------ TCP
    def _tcp_server(self):
        host = self._cfg.get("tcp", {}).get("host", "127.0.0.1")
        port = int(self._cfg.get("tcp", {}).get("port", 17788))
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.settimeout(0.5)   # 周期醒来检查 _active:阻塞在 accept 的线程无法被
        #                       close() 唤醒,不设超时会导致 shutdown 后端口不释放
        try:
            srv.bind((host, port))
            srv.listen(4)
            self._srv_socket = srv
            _log("TCP 服务就绪 %s:%d" % (host, port))
        except OSError as exc:
            _log_err("TCP 绑定失败 %s:%d: %s" % (host, port, exc))
            return
        while self._active:
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.settimeout(30.0)
            threading.Thread(target=self._tcp_handle, args=(conn,),
                             daemon=True).start()

    def _tcp_handle(self, conn):
        try:
            buf = b""
            while True:
                chunk = conn.recv(65536)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        req = json.loads(line.decode("utf-8"))
                        try:
                            resp = self._tcp_dispatch(req)
                        except Exception as exc:        # noqa: BLE001
                            resp = {"ok": False, "error": str(exc),
                                    "trace": traceback.format_exc()[-1200:]}
                    except Exception as exc:            # noqa: BLE001
                        resp = {"ok": False, "error": "JSON 解析失败: %s" % exc}
                    conn.sendall(
                        (json.dumps(resp, ensure_ascii=False) + "\n").encode("utf-8"))
        except Exception:                               # noqa: BLE001
            pass
        finally:
            try:
                conn.close()
            except Exception:                           # noqa: BLE001
                pass

    # 这些命令触及 timeline/USD 写入,从 TCP 线程直接调用会与主线程
    # app.update()/物理步跨线程争锁,实测(2026-11-08, ros 变体第 3 回合
    # reset 时 timeline.pause)可把整个 kit 进程挂死。一律排队到主线程。
    # 暂停态下 run_headless 主循环仍在旋转,drain_main_ops 每帧执行,
    # 不会超时。
    _MAIN_THREAD_CMDS = ("timeline", "set_pose", "config_reload", "restart",
                         "viz_set")

    def _tcp_dispatch(self, req):
        cmd = req.get("cmd", "")
        if cmd in self._MAIN_THREAD_CMDS and \
                threading.get_ident() != threading.main_thread().ident:
            return self._run_on_main(self._tcp_dispatch_impl, (req,))
        return self._tcp_dispatch_impl(req)

    def _tcp_dispatch_impl(self, req):
        cmd = req.get("cmd", "")
        if cmd == "ping":
            return {"ok": True, "pong": time.time(), "bridge": "auv_isaac_bridge"}
        if cmd == "status":
            return {"ok": True, **self._build_status()}
        if cmd == "stage_info":
            st = dict(self._build_status()["stage"])
            # kit 设置探测(只读):up 轴偏好/重力相关,查"旧场景设置"残留
            probes = {}
            try:
                import carb.settings                 # noqa: PLC0415
                sett = carb.settings.get_settings()
                for k in (
                    "/persistent/app/stage/upAxis",
                    "/app/stage/upAxis",
                    "/physics/gravityDirection",
                    "/physics/gravityMagnitude",
                    "/persistent/physics/gravityDirection",
                    "/persistent/physics/gravityMagnitude",
                ):
                    try:
                        probes[k] = sett.get(k)
                    except Exception:               # noqa: BLE001
                        probes[k] = None
            except Exception:                       # noqa: BLE001
                probes = {"error": "carb settings 不可用"}
            # PhysX 运行时重力(决定性证据:USD attr 是否真被 PhysX 使用)
            rtg = None
            for call in (lambda: self._sim_if.get_gravity(self._stage_id),
                         lambda: self._sim_if.get_gravity()):
                try:
                    g3 = call()
                    if g3 is not None:
                        rtg = [round(float(g3.x), 4), round(float(g3.y), 4),
                               round(float(g3.z), 4)]
                        break
                except Exception:                   # noqa: BLE001
                    continue
            st.update({
                "ok": True,
                "stage_file_full": self._stage_file,
                "physics_scenes": self._physics_scenes,
                "gravity_provisional": self._gravity_provisional,
                "last_physics_dt_s": (None if self._last_dt is None
                                      else round(self._last_dt, 6)),
                "kit_settings_probe": probes,
                "physx_runtime_gravity": rtg,
            })
            return st
        if cmd == "robot_info":
            if not self._body_path_str:
                return {"ok": False,
                        "error": self._last_error or "刚体未解析"}
            cfg_t = self._cfg.get("thrusters", {})
            dirs_cfg = cfg_t.get("directions")
            return {"ok": True,
                    "body_path": self._body_path_str,
                    "body_from": self._body_from,
                    "body_detail": self._body_detail,
                    "stage_id": self._stage_id, "body_id": self._body_id,
                    "mass_kg": self._mass,
                    "com_base_link_m_used": self._cfg.get(
                        "com_base_link_m", [0, 0, 0]),
                    "r_body_m": np.round(
                        self._r_body_num * self._mpu, 4).tolist(),
                    "directions_body": np.round(self._d_body, 4).tolist(),
                    "directions_cfg": (
                        [] if not dirs_cfg else
                        np.round(np.asarray(dirs_cfg, dtype=np.float64),
                                 4).tolist()),
                    "thrust_limit_n": [cfg_t.get("force_min_n"),
                                       cfg_t.get("force_max_n")],
                    "thruster_paths": self._thruster_paths_used,
                    "thruster_detail": self._thruster_detail,
                    "cb_body_m": np.round(
                        self._cb_body_num * self._mpu, 4).tolist(),
                    "geom_source": self._geom_source,
                    "geom_check": self._geom_check,
                    "note": ("directions_body 为推进器 prim 局部 +X 推导(假设正推"
                             "力=局部+X,待小推力实测确认);directions_cfg 为配置"
                             "对照值;两者不一致的推进器需在逐推进器测试中裁决")}
        if cmd == "config_reload":
            changed = self._maybe_reload_config(force=True)
            self._derive_stage()
            return {"ok": changed, "note": "配置重载并重解析 stage"}
        if cmd == "test_thrust":
            return self._cmd_test_thrust(req)
        if cmd == "zero":
            with self._lock:
                self._cmd = np.zeros(8)
                self._cmd_stamp_s = -1e9
                self._test_until_sim = -1e9
            return {"ok": True, "note": "命令与测试推力已清零"}
        if cmd == "estop":
            on = bool(req.get("enabled", True))
            with self._lock:
                self._estop = on
                self._estop_reason = "TCP estop" if on else ""
                if on:
                    self._cmd = np.zeros(8)
                    self._test_until_sim = -1e9
            return {"ok": True, "estop": on}
        if cmd == "hydro_get":
            return {"ok": True, "hydrodynamics": self._hydro()}
        if cmd == "hydro_set":
            self._hydro().update(req.get("patch", {}))
            return {"ok": True, "hydrodynamics": self._hydro(),
                    "note": "仅内存生效;长期值写 JSON 后 config_reload"}
        if cmd == "timeline":
            return self._cmd_timeline(req)
        if cmd == "set_pose":
            return self._cmd_set_pose(req)
        if cmd == "evaluate":
            return self._cmd_evaluate(req)
        if cmd == "sensor_scan":
            return self._cmd_sensor_scan(req)
        if cmd == "graph_scan":
            return self._cmd_graph_scan(req)
        if cmd == "xform_dump":
            return self._cmd_xform_dump(req)
        if cmd == "disturb_config":
            st = self._disturb.configure(req.get("patch", {}))
            return {"ok": True, "disturbance": st,
                    "note": "补丁仅内存生效;长期值写 JSON 后 config_reload"}
        if cmd == "disturb_get":
            return {"ok": True, "disturbance": self._disturb.state(),
                    "history_last_20": [
                        [round(t, 3), [[round(float(x), 4) for x in v]
                                       for v in vecs], fr]
                        for t, vecs, fr in self._disturb.history[-20:]]}
        if cmd == "disturb_stop":
            self._disturb.stop()
            return {"ok": True, "disturbance": self._disturb.state()}
        if cmd == "disturb_replay":
            seq = req.get("sequence") or []
            if not seq:
                # 默认回放本次会话记录到的序列
                hist = self._disturb.history
                seq = []
                for i in range(1, len(hist)):
                    seq.append([round(hist[i][0] - hist[i - 1][0], 3),
                                [[float(x) for x in v] for v in hist[i][1]]])
            self._disturb.set_replay(seq)
            return {"ok": True, "n_items": len(seq),
                    "note": "回放序列已装入,保持 enabled+mode=cont 生效"}
        if cmd == "viz_set":
            patch = req.get("patch", {})
            self._viz.cfg.update(patch)
            for k, attr in (("enabled", "enabled"),
                            ("rate_hz", "rate_hz"),
                            ("newtons_per_meter", "n_per_m"),
                            ("pose_arrow_len_m", "pose_arrow_len_m"),
                            ("max_arrow_m", "max_arrow_m")):
                if k in patch:
                    setattr(self._viz, attr, type(getattr(self._viz, attr))(
                        patch[k]))
            if patch.get("clear"):
                self._viz.clear()
            return {"ok": True, "viz": self._viz.state()}
        if cmd == "viz_get":
            return {"ok": True, "viz": self._viz.state()}
        if cmd == "step_state":
            return {"ok": True, "state": self._step_state}
        if cmd == "restart":
            return self._cmd_restart()
        if cmd == "stop":
            def _do():
                time.sleep(0.3)
                self.shutdown()
                builtins._AUV_ISAAC_BRIDGE_INSTANCE = None
            threading.Thread(target=_do, daemon=True).start()
            return {"ok": True, "note": "桥已停止(重启需再执行 bootstrap.py)"}
        return {"ok": False, "error": "未知命令: %r" % cmd}

    def _cmd_test_thrust(self, req):
        idx = req.get("index", "all")
        force = float(req.get("force_n", 0.0))
        dur = min(max(float(req.get("duration_s", 1.0)), 0.0), 5.0)
        t = self._cfg.get("thrusters", {})
        force = min(max(force, float(t.get("force_min_n", -39.23))),
                    float(t.get("force_max_n", 39.23)))
        vec = np.zeros(8)
        if idx == "all":
            vec[:] = force
        else:
            i = int(idx)
            if not 0 <= i < 8:
                return {"ok": False, "error": "index 需为 0..7 或 'all'"}
            vec[i] = force
        with self._lock:
            if self._estop:
                return {"ok": False, "error": "急停中,先发 estop:false"}
            self._test_vec = vec
            self._test_until_sim = self._sim_s + dur
        return {"ok": True, "force_n": vec.tolist(),
                "until_sim_s": round(self._sim_s + dur, 3),
                "note": "测试推力覆盖常规命令直到到期;期间 MPPI 输出被忽略"}

    def _cmd_timeline(self, req):
        action = req.get("action", "")
        if action not in ("play", "pause", "stop"):
            return {"ok": False, "error": "action 需为 play|pause|stop"}
        getattr(self._timeline, action)()
        note = ""
        if req.get("play_every_frame") is not None:
            try:
                self._timeline.set_play_every_frame(bool(req["play_every_frame"]))
                note = "; play_every_frame=%s" % bool(req["play_every_frame"])
            except Exception as exc:        # noqa: BLE001
                note = "; set_play_every_frame 失败: %s" % exc
        return {"ok": True, "action": action + note}

    # ------------------------------------------------------------- 主线程队列
    def drain_main_ops(self):
        """在主线程安全点(每物理步开头 / run_headless 主循环)执行排队操作。"""
        with self._main_q_lock:
            ops, self._main_q = self._main_q, []
        for fn, args, box, ev in ops:
            try:
                box["result"] = fn(*args)
            except Exception as exc:                    # noqa: BLE001
                box["result"] = {"ok": False, "error": str(exc),
                                 "trace": traceback.format_exc()[-800:]}
            ev.set()

    def _run_on_main(self, fn, args=(), timeout=15.0):
        """把 USD 写操作排到主线程执行并等待结果(TCP 线程用)。"""
        ev = threading.Event()
        box = {}
        with self._main_q_lock:
            self._main_q.append((fn, args, box, ev))
        if not ev.wait(timeout):
            with self._main_q_lock:
                self._main_q = [(f, a, b, e) for (f, a, b, e) in self._main_q
                                if e is not ev]
            return {"ok": False,
                    "error": "主线程执行超时(物理步未推进或已暂停?)"}
        return box.get("result", {"ok": False, "error": "无结果"})

    def _cmd_set_pose(self, req):
        """试验复位:把 base_link 摆到指定位姿。pos_m 世界【米】,quat_xyzw。
        做法:经父级局部-世界逆变换换算出局部平移/旋转,写 XformOp。
        建议流程 pause -> set_pose -> play。播放中从 TCP 线程调用时,
        排队到主线程物理步安全点执行(直接写会与物理步并发死锁);
        已暂停/主线程进程内调用维持直接写(与旧版行为一致)。"""
        try:
            from_tcp = threading.get_ident() != threading.main_thread().ident
            playing = False
            if from_tcp and self._timeline is not None:
                try:
                    playing = bool(self._timeline.is_playing())
                except Exception:                       # noqa: BLE001
                    playing = False
            if from_tcp and playing:
                return self._run_on_main(self._set_pose_impl, (req,),
                                         timeout=15.0)
            return self._set_pose_impl(req)
        except Exception as exc:                        # noqa: BLE001
            return {"ok": False, "error": str(exc),
                    "trace": traceback.format_exc()[-800:]}

    def _set_pose_impl(self, req):
        try:
            stage = self._omni_usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(self._body_path_str)
            if prim is None or not prim.IsValid():
                return {"ok": False, "error": "body prim 无效"}
            pos_m = req.get("pos_m")
            quat = req.get("quat_xyzw")
            if pos_m is None and quat is None:
                return {"ok": False, "error": "需要 pos_m 和/或 quat_xyzw"}

            # 语义:pos_m/quat_xyzw 是"报告位姿"(= prim 位姿 + R·com_base_link_m,
            # 与 state 话题/step_state 同一口径,控制器与目标都在该帧)。
            # 写 prim 前先把质心偏移扣除,使复位后的读回与指令一致。
            com_m_cfg = np.asarray(self._cfg.get("com_base_link_m", [0, 0, 0]),
                                   dtype=np.float64)
            q_w = None
            if quat is not None:
                q_w = _quat_normalize(np.asarray(quat, dtype=np.float64))
            pos_prim_m = None
            if pos_m is not None:
                if q_w is not None:
                    R_c = _quat_to_matrix(q_w)
                else:
                    # 只给位置:用当前姿态扣质心
                    tcur = self._usdgeom.Xformable(prim). \
                        ComputeLocalToWorldTransform(0)
                    R_c = _proper_rot(_gf_mat3_to_np(tcur, self._gf))
                pos_prim_m = (np.asarray(pos_m, dtype=np.float64)
                              - R_c.dot(com_m_cfg))

            parent = prim.GetParent()
            Mp = self._usdgeom.Xformable(
                parent).ComputeLocalToWorldTransform(0) if parent.IsValid() else None
            Mpi = Mp.GetInverse() if Mp is not None else None

            xf = self._usdgeom.Xformable(prim)
            ops = xf.GetOrderedXformOps()
            changed = []
            Xt = self._usdgeom.XformOp.TypeTranslate
            Xo = self._usdgeom.XformOp.TypeOrient
            gf = self._gf

            if pos_prim_m is not None:
                world_num = gf.Vec3d(pos_prim_m[0] / self._mpu,
                                     pos_prim_m[1] / self._mpu,
                                     pos_prim_m[2] / self._mpu)
                local = Mpi.Transform(world_num) if Mpi is not None else world_num
                for op in ops:
                    if op.GetOpType() == Xt:
                        op.Set(gf.Vec3d(local[0], local[1], local[2]))
                        changed.append("translate")
                        break
            if q_w is not None:
                R_w = _quat_to_matrix(q_w)
                if Mpi is not None:
                    # Mpi 的 3x3(列约定)∝ Rp^T:proper 后即父旋转的逆,
                    # 直接右乘世界旋转得局部旋转(旧版多转置一次,引入
                    # 恒定 Rp^2(120°)偏差)
                    Rp_inv = _proper_rot(_gf_mat3_to_np(Mpi, gf))
                    R_l = Rp_inv.dot(R_w)
                else:
                    R_l = R_w
                q_l = _matrix_to_quat(R_l)
                for op in ops:
                    if op.GetOpType() == Xo:
                        op.Set(gf.Quatd(float(q_l[3]), gf.Vec3d(
                            float(q_l[0]), float(q_l[1]), float(q_l[2]))))
                        changed.append("orient")
                        break
            self._have_prev = False       # 差分基线作废,防速度尖峰
            return {"ok": True, "changed": changed,
                    "note": "已写 USD 变换;若 PhysX 恢复后弹回旧位姿,先 pause 再试"}
        except Exception as exc:            # noqa: BLE001
            return {"ok": False, "error": str(exc),
                    "trace": traceback.format_exc()[-800:]}

    def _cmd_evaluate(self, req):
        """恒力激励->实测沿推力轴平均加速度,对账力单位换算与方向假设。
        用测试推力通道(覆盖常规命令),采样窗口内位置差分求加速度。"""
        i = int(req.get("index", 0))
        if not 0 <= i < 8:
            return {"ok": False, "error": "index 需为 0..7"}
        force = float(req.get("force_n", 5.0))
        dur = min(max(float(req.get("duration_s", 1.0)), 0.2), 3.0)
        settle = min(max(float(req.get("settle_s", 0.3)), 0.0), 1.0)

        with self._lock:
            if self._estop:
                return {"ok": False, "error": "急停中,先 estop:false"}
            vec = np.zeros(8)
            vec[i] = force
            self._test_vec = vec
            t0 = self._sim_s
            self._test_until_sim = t0 + settle + dur

        samples = []
        done = threading.Event()

        def sampler():
            while not done.is_set():
                if self._sim_s > t0 + settle + dur + 0.3:
                    done.set()
                    return
                if self._have_prev and self._prev_com_m is not None:
                    samples.append((self._sim_s, self._prev_com_m.copy(),
                                    self._prev_q.copy()))
                time.sleep(1.0 / 120.0)

        th = threading.Thread(target=sampler, daemon=True)
        th.start()
        th.join(timeout=settle + dur + 6.0)
        done.set()
        with self._lock:
            self._test_until_sim = -1e9

        ts, te = t0 + settle, t0 + settle + dur
        win = [s for s in samples if ts <= s[0] <= te]
        if len(win) < 4:
            return {"ok": False, "error":
                    "样本不足(%d)。确认仿真处于 Play 且未暂停。" % len(win)}
        p0, p1 = win[0][1], win[-1][1]
        dtw = win[-1][0] - win[0][0]
        if dtw <= 1e-6:
            return {"ok": False, "error": "时间窗口为零(暂停?)"}
        a_world = (p1 - p0) / dtw
        Rm = _quat_to_matrix(win[len(win) // 2][2])
        d_world = Rm.dot(self._d_body[i])
        a_along = float(a_world.dot(d_world))
        a_exp = force / self._mass
        return {
            "ok": True, "index": i, "force_n": force,
            "window_s": [round(win[0][0], 3), round(win[-1][0], 3)],
            "n_samples": len(win),
            "a_meas_world_m_s2": [round(v, 4) for v in a_world],
            "a_along_thrust_axis_m_s2": round(a_along, 4),
            "expected_a_m_s2": round(a_exp, 4),
            "ratio": round(a_along / a_exp, 4) if abs(a_exp) > 1e-9 else None,
            "note": ("ratio≈+1 力换算/方向都对;≈-1 该推进器方向取负;"
                     "量级系统性偏差 => 修 units.force_unit_scale 后 config_reload"),
        }

    # ------------------------------------------------------------------ 只读扫描
    def _sensor_entry(self, prim):
        """传感器 prim 的只读条目:类型 + 局部/世界变换(局部 API 不可用则置 None)。"""
        e = {"path": str(prim.GetPath()),
             "type": str(prim.GetTypeName() or "")}
        try:
            tm = self._usdgeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            t = tm.ExtractTranslation()
            q = _matrix_to_quat(_proper_rot(_gf_mat3_to_np(tm, self._gf)))
            e["world_t_m"] = [round(v * self._mpu, 4) for v in (t[0], t[1], t[2])]
            e["world_q_xyzw"] = [round(float(v), 4) for v in tuple(q)]
        except Exception:                       # noqa: BLE001
            e["world_t_m"] = None
            e["world_q_xyzw"] = None
        try:
            ltm = self._usdgeom.Xformable(prim).GetLocalTransformation()
            lt = ltm.ExtractTranslation()
            lq = _matrix_to_quat(_proper_rot(_gf_mat3_to_np(ltm, self._gf)))
            e["local_t_num"] = [round(v, 4) for v in (lt[0], lt[1], lt[2])]
            e["local_q_xyzw"] = [round(float(v), 4) for v in tuple(lq)]
        except Exception:                       # noqa: BLE001
            e["local_t_num"] = None
            e["local_q_xyzw"] = None
        return e

    def _cmd_sensor_scan(self, req):
        """只读遍历 ZED 根节点,自动找 Camera/IMU Sensor prim 并记录路径与
        类型;不创建任何 prim。传感器缺失时如实报告。"""
        try:
            stage = self._omni_usd.get_context().get_stage()
            if stage is None:
                return {"ok": False, "error": "无 stage"}
            roots = ([str(req["root"])] if req.get("root") else
                     [str(r) for r in self._cfg.get("zed_root_paths", []) if r])
            root_prim, root_used = None, None
            for r in roots:
                p = stage.GetPrimAtPath(r)
                if p is not None and p.IsValid():
                    root_prim, root_used = p, r
                    break
            if root_prim is None:
                return {"ok": False, "roots_tried": roots,
                        "error": "ZED 根节点不存在(引用可能未解析)",
                        "hint": "在配置 zed_root_paths 或请求参数 root 里给出正确路径"}
            from pxr import Usd
            tree, sensors = [], []
            n = 0
            for p in Usd.PrimRange(root_prim):
                n += 1
                tname = str(p.GetTypeName() or "")
                path = str(p.GetPath())
                if len(tree) < 400:
                    tree.append({"path": path, "type": tname})
                tl = tname.lower()
                is_cam = False
                try:
                    is_cam = bool(p.IsA(self._usdgeom.Camera))
                except Exception:               # noqa: BLE001
                    pass
                if is_cam or "camera" in tl or "imu" in tl or "sensor" in tl:
                    entry = self._sensor_entry(p)
                    entry["is_camera"] = is_cam or "camera" in tl
                    entry["is_imu"] = "imu" in tl
                    sensors.append(entry)
            cams = [s for s in sensors if s["is_camera"]]
            imus = [s for s in sensors if s["is_imu"]]
            others = [s for s in sensors if not s["is_camera"] and not s["is_imu"]]
            return {"ok": True, "root_used": root_used, "prim_count": n,
                    "cameras": cams, "imus": imus, "other_sensors": others,
                    "missing": ([] if cams else ["camera"]) +
                               ([] if imus else ["imu"]),
                    "note": "只读扫描;tree_first_400 为prim清单(截断)",
                    "tree_first_400": tree}
        except Exception as exc:                # noqa: BLE001
            return {"ok": False, "error": str(exc),
                    "trace": traceback.format_exc()[-800:]}

    def _cmd_graph_scan(self, req):
        """只读检查 stage 里的 ActionGraph/omni.graph:列出图、节点以及
        ROS2 相关节点(Clock/Camera/IMU/Odometry/订阅等),避免重复建发布器。"""
        try:
            stage = self._omni_usd.get_context().get_stage()
            if stage is None:
                return {"ok": False, "error": "无 stage"}
            from pxr import Usd
            keywords = ("ros2", "clock", "camera", "imu", "odometry",
                        "odom", "thrust", "publish", "subscribe", "context",
                        "diagnostic")
            graphs, loose_nodes = [], []
            for p in stage.TraverseAll():
                tname = str(p.GetTypeName() or "")
                pname = p.GetName()
                blob = (tname + " " + pname).lower()
                if "computegraph" in tname.lower() or \
                        pname.lower() == "actiongraph":
                    nodes = []
                    for c in Usd.PrimRange(p):
                        if c.GetPath() == p.GetPath():
                            continue
                        if len(nodes) < 400:
                            nodes.append({"path": str(c.GetPath()),
                                          "type": str(c.GetTypeName() or ""),
                                          "name": c.GetName()})
                    graphs.append({"path": str(p.GetPath()), "type": tname,
                                   "node_count": len(nodes), "nodes": nodes})
                elif any(k in blob for k in keywords) and \
                        "ros2" in blob:
                    loose_nodes.append({"path": str(p.GetPath()),
                                        "type": tname})
            return {"ok": True, "graph_count": len(graphs), "graphs": graphs,
                    "loose_ros2_prims": loose_nodes[:80],
                    "note": ("graphs[].nodes 里 type 含 ROS2* 的即 Isaac ROS2 "
                             "桥接节点;为空=场景无 ActionGraph 发布器,"
                             "全部 ROS2 收发由本 Python 桥承担")}
        except Exception as exc:                # noqa: BLE001
            return {"ok": False, "error": str(exc),
                    "trace": traceback.format_exc()[-800:]}

    def _cmd_xform_dump(self, req):
        """只读:若干 prim 的变换链诊断——世界 4x4 矩阵、3x3 行列式、
        局部平移/旋转与局部线性列向量。用于排查引用链缩放/镜像/非均匀
        缩放导致的坐标系异常(2026-11-03 几何对账 |Δr|=1.44m 用)。"""
        try:
            stage = self._omni_usd.get_context().get_stage()
            if stage is None:
                return {"ok": False, "error": "无 stage"}
            out = []
            for path in [str(p) for p in req.get("paths", [])]:
                prim = stage.GetPrimAtPath(path)
                if prim is None or not prim.IsValid():
                    out.append({"path": path, "valid": False})
                    continue
                xf = self._usdgeom.Xformable(prim)
                entry = {"path": path, "valid": True,
                         "type": str(prim.GetTypeName() or "")}
                wm = xf.ComputeLocalToWorldTransform(0)
                entry["world_rows"] = [
                    [round(float(wm[r][c]), 8) for c in range(4)]
                    for r in range(4)]
                m3 = np.array([[float(wm[r][c]) for c in range(3)]
                               for r in range(3)])
                entry["world_det3x3"] = round(float(np.linalg.det(m3)), 10)
                try:
                    lm = xf.GetLocalTransformation()
                    entry["local_t_num"] = [
                        round(float(v), 6) for v in lm.ExtractTranslation()]
                    lq = _matrix_to_quat(
                        _proper_rot(_gf_mat3_to_np(lm, self._gf)))
                    entry["local_q_xyzw"] = [
                        round(float(v), 6) for v in tuple(lq)]
                    gf = self._gf
                    cols = []
                    for e in (gf.Vec3d(1, 0, 0), gf.Vec3d(0, 1, 0),
                              gf.Vec3d(0, 0, 1)):
                        c = lm.TransformDir(e)
                        cols.append([round(float(c[0]), 10),
                                     round(float(c[1]), 10),
                                     round(float(c[2]), 10)])
                    l3 = np.asarray(cols, dtype=np.float64)
                    entry["local_col_norms"] = [
                        round(float(np.linalg.norm(c)), 8) for c in cols]
                    entry["local_det3x3"] = round(
                        float(np.linalg.det(l3)), 10)
                except Exception as exc:           # noqa: BLE001
                    entry["local_error"] = str(exc)
                out.append(entry)
            return {"ok": True, "prims": out}
        except Exception as exc:                # noqa: BLE001
            return {"ok": False, "error": str(exc),
                    "trace": traceback.format_exc()[-800:]}

    def _cmd_restart(self):
        """远程热重启(改了脚本文件后无需用户再粘贴)。"""
        if not self._script_path or not os.path.isfile(self._script_path):
            return {"ok": False, "error": "脚本路径不可用,请重新执行 bootstrap.py"}
        path, cfg = self._script_path, self._config_path

        def _do():
            time.sleep(0.5)
            try:
                self.shutdown()
            finally:
                builtins._AUV_ISAAC_BRIDGE_INSTANCE = None
                import runpy
                runpy.run_path(path, run_name="__main__",
                               init_globals={"AUV_BRIDGE_CONFIG_PATH": cfg})
        threading.Thread(target=_do, name="auv_bridge_restart", daemon=True).start()
        return {"ok": True, "note": "0.5s 后热重启 %s" % path}

    # ------------------------------------------------------------------ 关闭
    def shutdown(self):
        self._active = False
        self._phys_sub = None
        self._stage_sub = None
        try:
            self._disturb.stop()
        except Exception:                               # noqa: BLE001
            pass
        try:
            self._viz.clear()
        except Exception:                               # noqa: BLE001
            pass
        try:
            if self._srv_socket is not None:
                self._srv_socket.close()
        except Exception:                   # noqa: BLE001
            pass
        if self._ros is not None:
            try:
                self._ros["executor"].shutdown()
                self._ros["rclpy"].shutdown(context=self._ros["ctx"])
            except Exception:               # noqa: BLE001
                pass
            self._ros = None
        _log("已关闭")


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

def _main():
    existing = getattr(builtins, "_AUV_ISAAC_BRIDGE_INSTANCE", None)
    cfg_path = globals().get("AUV_BRIDGE_CONFIG_PATH", DEFAULT_CONFIG)
    if isinstance(existing, AuvIsaacBridge) and existing._active:
        print("[auv_isaac_bridge] 已在运行,忽略重复启动(如需重载: TCP restart)")
        return existing
    bridge = AuvIsaacBridge(config_path=cfg_path)
    builtins._AUV_ISAAC_BRIDGE_INSTANCE = bridge
    tcp = bridge._cfg.get("tcp", {})
    print("[auv_isaac_bridge] 已启动 TCP %s:%d。"
          "诊断: python3 scripts/bridge_client.py status。"
          "停止: TCP cmd=stop 或内置变量 _AUV_ISAAC_BRIDGE_INSTANCE.shutdown()"
          % (tcp.get("host", "127.0.0.1"), int(tcp.get("port", 17788))))
    return bridge


_main()
