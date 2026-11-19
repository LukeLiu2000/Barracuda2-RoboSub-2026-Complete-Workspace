# -*- coding: utf-8 -*-
"""metrics 单测:时间跳变切分/目标插值/误差序列/容差指标/扰动恢复/合成回合。"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from auv_common.frames import quat_about_axis                    # noqa: E402
from auv_experiments import metrics as M                         # noqa: E402
from auv_experiments.tasks import task_list, eval_set            # noqa: E402


def _q():
    return np.array([0.0, 0.0, 0.0, 1.0])


# ------------------------------------------------------------ 时间切分
def test_split_time_jumps():
    t = np.array([0.0, 0.02, 0.04, 3.5, 3.52,   # 前跳>1s -> 切
                  2.0, 2.02])                    # 回跳 -> 切
    segs = M.split_at_time_jumps(t)
    assert segs == [(0, 3), (3, 5), (5, 7)]


def test_split_monotone_single_segment():
    t = np.linspace(0.0, 1.0, 51)
    assert M.split_at_time_jumps(t) == [(0, 51)]


# ------------------------------------------------------------ 目标插值
def test_interp_goal_step_hold():
    tg = np.array([0.0, 10.0])
    gp = np.array([[0.0, 0, 0], [1.0, 0, 0]])
    gq = np.array([_q(), _q()])
    t = np.array([-1.0, 0.0, 5.0, 9.9, 10.0, 15.0])
    GP, _GQ = M.interp_goal(t, tg, gp, gq)
    assert np.allclose(GP[0], gp[0])          # 起点前夹到首目标
    assert np.allclose(GP[1], gp[0])
    assert np.allclose(GP[2], gp[0])          # 10s 前保持首目标
    assert np.allclose(GP[4], gp[1])
    assert np.allclose(GP[5], gp[1])          # 末目标保持


# ------------------------------------------------------------ 误差序列
def test_error_series():
    n = 10
    pos = np.zeros((n, 3))
    quats = np.tile(_q(), (n, 1))
    gp = np.tile(np.array([0.3, 0.0, 0.4]), (n, 1))
    gq = np.tile(quat_about_axis(np.array([0, 1, 0.0]), 0.2), (n, 1))
    e_p, e_R = M.error_series(pos, quats, gp, gq)
    assert np.allclose(e_p, 0.5)              # 常值位置误差
    assert np.allclose(e_R, 0.2, atol=1e-9)   # 常值姿态误差角


# ------------------------------------------------------------ 容差指标
def test_first_tol_and_settle_and_overshoot():
    t = np.arange(0.0, 10.0, 0.1)
    err = np.where(t < 2.0, 0.5 - 0.2 * t, np.where(t < 3.0, 0.12, 0.02))
    # t∈[2,3) err=0.12(>0.08);t>=3 进入容差并保持
    ft = M.first_tol_time(t, err, 0.08)
    assert ft is not None and abs(ft - 3.0) < 1e-9
    # settle = 最后一次离开容差的时刻(t=2.9 采样)
    st = M.settle_time(t, err, 0.08)
    assert st is not None and abs(st - 2.9) < 1e-9
    # 首达容差后无再越出:overshoot = max(after)/err0 = 0.02/0.5
    ov = M.overshoot(t, err, 0.08)
    assert ov == pytest.approx(0.02 / 0.5)


def test_first_tol_none_when_never():
    t = np.arange(0.0, 5.0, 0.1)
    err = np.full_like(t, 0.5)
    assert M.first_tol_time(t, err, 0.08) is None
    assert M.settle_time(t, err, 0.08) is None


# ------------------------------------------------------------ 扰动指标
def test_disturbance_peak_and_recovery():
    t = np.arange(0.0, 12.0, 0.1)
    err = np.where((t >= 5.0) & (t < 8.0), 0.5, 0.02)
    mask = (t >= 5.0) & (t < 8.0)
    dm = M.disturbance_metrics(t, err, mask, 0.08)
    assert dm["peak_dev_m"] == pytest.approx(0.48, abs=0.01)
    assert dm["recovery_s"] is not None and 0.0 <= dm["recovery_s"] <= 0.2


# ------------------------------------------------------------ 合成回合
def test_episode_metrics_synthetic():
    n = 200
    t = np.arange(n) * 0.05
    # 位置误差 0.4 -> 0.03(一阶收敛);姿态误差恒 0.01
    e = 0.03 + 0.37 * np.exp(-t / 1.5)
    rec = {
        "t_sim": t,
        "err_p": e,
        "err_r": np.full(n, 0.01),
        "v": np.tile([0.05, 0.0, 0.0], (n, 1)),
        "w": np.zeros((n, 3)),
        "t_thr": t[1:],
        "u_req": np.tile(np.linspace(1, 0.1, 8), (n - 1, 1)),
        "u_applied": np.tile(np.linspace(1, 0.1, 8), (n - 1, 1)),
        "t_dis": [], "f_dis": np.zeros((0, 6)),
        "infer_ms": np.random.default_rng(0).uniform(0.4, 0.9, n).tolist(),
    }
    m = M.episode_metrics(rec)
    assert m["rmse_pos_m"] > 0
    assert m["final_pos_err_m"] == pytest.approx(e[-1])
    assert m["infer_ms_p95"] >= m["infer_ms_mean"]
    assert m["saturation_frac"] == 0.0
    assert m["success"] is True       # 末端位置/姿态误差均小于容差
    assert 0.0 <= m.get("thrust_variation_n", 0.0)


def test_episode_metrics_time_jump_split():
    # 200 步正常 + 时间回 0 再 50 步:应在跳变处切分,运动指标取最长段
    t1 = np.arange(200) * 0.05
    t2 = np.arange(50) * 0.05
    t = np.concatenate([t1, t2])
    rec = {
        "t_sim": t,
        "err_p": np.full(250, 0.05),
        "err_r": np.full(250, 0.01),
        "v": np.zeros((250, 3)), "w": np.zeros((250, 3)),
        "t_thr": [], "u_req": np.zeros((0, 8)),
        "u_applied": np.zeros((0, 8)), "t_dis": [], "f_dis": np.zeros((0, 6)),
        "infer_ms": [],
    }
    m = M.episode_metrics(rec)
    assert m["n_segments"] == 2
    # duration_s 是整段记录跨度(末-首);rmse 按最长段计算
    assert m["duration_s"] == pytest.approx(2.45)
    assert m["rmse_pos_m"] == pytest.approx(0.05)


# ------------------------------------------------------------ 任务库
def test_task_library_shapes():
    tasks = task_list(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))
    assert len(tasks) >= 14
    names = [t[0] for t in tasks]
    for must in ("hold", "fwd_+0.4", "fwd_-0.4", "yaw_+30", "roll_+20",
                 "combo_xyz", "dist_cont_fwd", "dist_step_hold"):
        assert must in names
    for _name, g, dist, dur, _p in tasks:
        assert np.linalg.norm(g["q"]) == pytest.approx(1.0)
        assert dur > 0
    # 扰动任务带 seed 且力上限=半推力
    d = [t[2] for t in tasks if t[0] == "dist_cont_fwd"][0]
    assert d["seed"] == 1234 and d["force_max_n"] == 19.6133


def test_eval_set_disjoint_from_tune():
    tune = {t[0] for t in task_list(np.zeros(3), np.array([0, 0, 0, 1.0]))}
    ev = eval_set(np.zeros(3), np.array([0, 0, 0, 1.0]))
    assert ev
    assert not (tune & {e[0] for e in ev})   # 验证集任务名不与整定重叠
