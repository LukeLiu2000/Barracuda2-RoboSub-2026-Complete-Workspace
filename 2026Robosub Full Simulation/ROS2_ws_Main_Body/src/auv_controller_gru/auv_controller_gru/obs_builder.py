# -*- coding: utf-8 -*-
"""obs_builder —— GRU 策略的 23 维观测构建与归一化(训练/推理共用,纯 numpy)。

部署节点与训练环境必须走同一份代码,保证预处理一致。
姿态旋转向量在训练目标生成时限制在有限角度(≤90°)内,避免跨越 180°
表示歧义;运行期如实计算,报告记录该约定。
垂直参考方向:场景重力有效非零时取重力单位方向;零重力(ZGD 第一阶段)
场景取世界上轴(0,0,1)作为垂直参考——选择是显式的、写入模型元数据,
不泄漏任何真实外力或随机化参数。
"""

import json
import os

import numpy as np

from auv_common.frames import attitude_error_rotvec, quat_normalize, quat_to_matrix

OBS_DIM = 23
ACTION_DIM = 8

OBS_LAYOUT = [
    (0, 3, "e_pos_body = R^T (p_d - p) [m]"),
    (3, 6, "e_rot_rotvec = log(R^T R_d) [rad·axis components]"),
    (6, 9, "v_body [m/s]"),
    (9, 12, "w_body [rad/s]"),
    (12, 15, "vertical_ref_dir_body = R^T (g_dir_world 或 world_up) [-]"),
    (15, 23, "last_thruster_cmd_normalized = f_i / Fmax, 经公共限幅后 [-]"),
]


def build_obs(p, q, v_body, w_body, p_d, q_d,
              vertical_ref_world=(0.0, 0.0, 1.0),
              last_cmd_norm=None):
    """返回 23 维未归一化观测(顺序严格按 OBS_LAYOUT)。"""
    q = quat_normalize(q)
    q_d = quat_normalize(q_d)
    R = quat_to_matrix(q)
    e_pos_b = R.T @ (np.asarray(p_d, dtype=np.float64) -
                     np.asarray(p, dtype=np.float64))
    e_rot = attitude_error_rotvec(q, q_d)
    g_ref = np.asarray(vertical_ref_world, dtype=np.float64)
    n = float(np.linalg.norm(g_ref))
    if n < 1e-9:
        g_ref = np.array([0.0, 0.0, 1.0])
        n = 1.0
    g_b = R.T @ (g_ref / n)
    cmd = (np.zeros(ACTION_DIM) if last_cmd_norm is None
           else np.asarray(last_cmd_norm, dtype=np.float64).reshape(ACTION_DIM))
    obs = np.concatenate([e_pos_b, e_rot,
                          np.asarray(v_body, dtype=np.float64).reshape(3),
                          np.asarray(w_body, dtype=np.float64).reshape(3),
                          g_b,
                          np.clip(cmd, -1.0, 1.0)]).astype(np.float64)
    assert obs.shape == (OBS_DIM,)
    if not np.all(np.isfinite(obs)):
        obs = np.zeros(OBS_DIM)
    return obs


class ObsNormalizer(object):
    """固定的仿射归一化(obs'=(obs-mean)/std);参数训练前离线估计并冻结,
    随模型元数据保存/加载。std 下限 1e-6 防除零。"""

    def __init__(self, mean=None, std=None):
        self.mean = (np.zeros(OBS_DIM) if mean is None
                     else np.asarray(mean, dtype=np.float64).reshape(OBS_DIM))
        self.std = (np.ones(OBS_DIM) if std is None
                    else np.asarray(std, dtype=np.float64).reshape(OBS_DIM))

    @classmethod
    def from_dataset(cls, samples):
        arr = np.asarray(samples, dtype=np.float64)
        mean = arr.mean(axis=0)
        std = np.maximum(arr.std(axis=0), 1e-6)
        std = np.maximum(std, 0.05 * np.maximum(np.abs(mean), 0.1))
        return cls(mean, std)

    def normalize(self, obs):
        return (np.asarray(obs, dtype=np.float64) - self.mean) / self.std

    def to_dict(self):
        return {"mean": self.mean.tolist(), "std": self.std.tolist()}

    @classmethod
    def from_dict(cls, d):
        return cls(d["mean"], d["std"])


def save_metadata(path, normalizer, extra=None):
    meta = {
        "obs_dim": OBS_DIM,
        "action_dim": ACTION_DIM,
        "obs_layout": ["%d-%d %s" % (a, b, s) for a, b, s in OBS_LAYOUT],
        "normalizer": normalizer.to_dict(),
        "vertical_ref_note": ("零重力(ZGD)阶段 vertical_ref=世界 +Z;"
                              "重力恢复后应改用重力单位方向并重训"),
        "action_mapping": "f_i = a_i * Fmax, a_i ∈ [-1,1], Fmax=39.2266 N",
    }
    if extra:
        meta.update(extra)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)


def load_metadata(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)
