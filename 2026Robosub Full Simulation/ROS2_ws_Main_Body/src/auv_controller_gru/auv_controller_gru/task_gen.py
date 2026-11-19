# -*- coding: utf-8 -*-
"""task_gen —— GRU 训练任务采样(无 gymnasium/ROS/Isaac 依赖,可随处单测)。

任务生成在安全空间:出生点=资产默认位姿,目标=出生姿态系下随机位移
(分级半径 0.4/0.8/1.2 m)+ 姿态增量(单位/±30°偏航/≤60°任意轴)。
"""

import numpy as np

from auv_common.frames import quat_about_axis, quat_mul, quat_to_matrix

FMAX_N = 39.2266          # 与 auv_common thrusters.yaml 一致(单测锁定)


def rand_ball(rng, r_max):
    """球内均匀采样(方向球面均匀 + 半径 u^(1/3))。"""
    d = rng.normal(size=3)
    d /= max(np.linalg.norm(d), 1e-9)
    r = r_max * (rng.uniform() ** (1.0 / 3.0))
    return d * r


class TaskSampler(object):
    """分级任务采样器。stage: 1/2/3(由 train.yaml 控制)。"""

    STAGES = {
        1: {"disp_max_m": 0.4, "rot_mode": "none", "rot_max_rad": 0.0},
        2: {"disp_max_m": 0.8, "rot_mode": "yaw",
            "rot_max_rad": float(np.deg2rad(30.0))},
        3: {"disp_max_m": 1.2, "rot_mode": "any",
            "rot_max_rad": float(np.deg2rad(60.0))},
    }

    def sample(self, rng, spawn_p, spawn_q, stage):
        cfg = self.STAGES[max(1, min(3, int(stage)))]
        disp_body = rand_ball(rng, cfg["disp_max_m"])
        if cfg["rot_mode"] == "none":
            ax, ang = np.array([0.0, 0.0, 1.0]), 0.0
        elif cfg["rot_mode"] == "yaw":
            ax, ang = np.array([0.0, 1.0, 0.0]), rng.uniform(
                -cfg["rot_max_rad"], cfg["rot_max_rad"])
        else:
            ax = rng.normal(size=3)
            ax /= max(np.linalg.norm(ax), 1e-9)
            ang = rng.uniform(0.0, cfg["rot_max_rad"])
        q_d = quat_mul(spawn_q, quat_about_axis(ax, ang))
        goal_p = spawn_p + quat_to_matrix(spawn_q) @ disp_body
        return {"goal_p": goal_p, "goal_q": q_d,
                "disp_m": float(np.linalg.norm(disp_body)),
                "rot_deg": float(np.rad2deg(ang)), "stage": int(stage)}
