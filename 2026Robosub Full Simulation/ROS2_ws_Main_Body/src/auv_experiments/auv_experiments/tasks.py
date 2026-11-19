# -*- coding: utf-8 -*-
"""tasks.py —— 实验任务库(目标序列 + 扰动配置,纯数据,无 ROS 依赖)。

坐标系:目标位移在世界系给出(出生位姿附近的安全空间),姿态增量在
出生姿态系(四元数右乘)。所有任务对三种控制器一视同仁。
扰动档位沿用桥 DisturbanceManager 配置 patch(见 isaac_bridge.json)。
"""

import numpy as np

from auv_common.frames import quat_about_axis, quat_mul

FMAX_HALF_N = 19.6133      # 扰动上限 = 半推力(与桥默认一致)

_DIST_CONT = {
    "enabled": True, "mode": "cont", "force_max_n": FMAX_HALF_N,
    "update_interval_s": 2.0, "transition_s": 1.5, "frame": "body",
}
_DIST_STEP = {
    "enabled": True, "mode": "step", "force_max_n": FMAX_HALF_N,
    "frame": "body",
}
_DIST_OFF = {"enabled": False, "mode": "off"}


def _goal(spawn_p, spawn_q, disp_world, rot_axis=None, rot_deg=0.0):
    q_d = spawn_q if rot_axis is None else quat_mul(
        spawn_q, quat_about_axis(np.asarray(rot_axis, dtype=float),
                                 np.deg2rad(rot_deg)))
    return {"p": np.asarray(spawn_p, dtype=float) +
            np.asarray(disp_world, dtype=float), "q": q_d}


def task_list(spawn_p, spawn_q):
    """返回 [(task_name, goal, disturb_cfg, duration_s, purpose), ...]。"""
    Rw = _quat_to_R(spawn_q)              # 位移取出生姿态系再转世界
    tasks = []

    def body(disp_body, name, rot_axis=None, rot_deg=0.0, dur=25.0,
             disturb=None, purpose=""):
        g = _goal(spawn_p, spawn_q, Rw @ np.asarray(disp_body, dtype=float),
                  rot_axis, rot_deg)
        tasks.append((name, g, disturb, dur, purpose))

    body([0, 0, 0], "hold", purpose="零输入保位:无指令漂移基线")
    body([0, 0, 0.4], "fwd_+0.4", purpose="单轴 +z 前 0.4 m(正向)")
    body([0, 0, -0.4], "fwd_-0.4", purpose="单轴 -z 后 0.4 m(反向)")
    body([0.4, 0, 0], "port_+0.4", purpose="单轴 +x 左舷 0.4 m")
    body([-0.4, 0, 0], "port_-0.4", purpose="单轴 -x 右舷 0.4 m")
    body([0, 0.4, 0], "up_+0.4", purpose="单轴 +y 上浮 0.4 m")
    body([0, -0.4, 0], "up_-0.4", purpose="单轴 -y 下潜 0.4 m")
    body([0, 0, 0], "yaw_+30", rot_axis=[0, 1, 0], rot_deg=30.0,
         dur=20.0, purpose="偏航 +30°(绕体 +y)")
    body([0, 0, 0], "yaw_-30", rot_axis=[0, 1, 0], rot_deg=-30.0,
         dur=20.0, purpose="偏航 -30°")
    body([0, 0, 0], "pitch_+20", rot_axis=[1, 0, 0], rot_deg=20.0,
         dur=20.0, purpose="俯仰 +20°(绕体 +x)")
    body([0, 0, 0], "roll_+20", rot_axis=[0, 0, 1], rot_deg=20.0,
         dur=20.0, purpose="横滚 +20°(绕体 +z)")
    body([0.3, 0.2, 0.3], "combo_xyz", dur=30.0,
         purpose="三轴组合位移")
    body([0, 0, 0.5], "diag_xyz_yaw", rot_axis=[0, 1, 0], rot_deg=25.0,
         dur=30.0, purpose="位移+姿态组合")
    body([0, 0, 0.4], "dist_cont_fwd", disturb=dict(_DIST_CONT, seed=1234),
         dur=30.0, purpose="定常随机扰动下的前进(鲁棒性)")
    body([0, 0, 0], "dist_step_hold", disturb=dict(_DIST_STEP, seed=4321,
         fixed_vec=[FMAX_HALF_N * 0.6, 0.0, 0.0], pulse_on_s=4.0,
         pulse_off_s=4.0, mode="pulse"), dur=30.0,
         purpose="保位 + 脉冲扰动(恢复指标)")
    return tasks


def eval_set(spawn_p, spawn_q):
    """独立验证集(与整定用任务不同位移量/方向)。"""
    Rw = _quat_to_R(spawn_q)
    out = []
    for name, db, ax, deg, dur in (
            ("val_hold", [0, 0, 0], None, 0.0, 20.0),
            ("val_diag", [0.25, -0.3, 0.35], None, 0.0, 25.0),
            ("val_yaw45", [0, 0, 0.15], [0, 1, 0], 45.0, 25.0),
            ("val_dist", [0, 0, 0.3], None, 0.0, 25.0)):
        g = _goal(spawn_p, spawn_q, Rw @ np.asarray(db, dtype=float),
                  ax, deg)
        disturb = dict(_DIST_CONT, seed=777) if name == "val_dist" else None
        out.append((name, g, disturb, dur, "independent_validation"))
    return out


def _quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
