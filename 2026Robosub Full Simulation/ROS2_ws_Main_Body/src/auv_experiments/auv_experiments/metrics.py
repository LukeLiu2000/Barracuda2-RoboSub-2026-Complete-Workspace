# -*- coding: utf-8 -*-
"""metrics.py —— 离线指标计算(纯 numpy,无 ROS/Isaac 依赖,单测覆盖)。

输入是 experiment_runner 落盘的 CSV 数组(在线指标与离线同源:在线用
本模块逐回合调用,离线 analyze.py 重算可复现)。

时间约定:运动量用仿真时间 /clock(状态消息头);推理耗时用墙钟
time.monotonic()(诊断消息内);回合切分按仿真时间回跳/前跳>1s。
"""

import numpy as np

from auv_common.frames import attitude_error_rotvec


def split_at_time_jumps(t_sim, jump_s=1.0):
    """仿真时间回跳/异常前跳处切分;返回 [(start,end)) 索引段列表。"""
    t = np.asarray(t_sim, dtype=np.float64)
    n = len(t)
    if n == 0:
        return []
    cuts = [0]
    for i in range(1, n):
        if t[i] < t[i - 1] - 1e-6 or t[i] - t[i - 1] > jump_s:
            cuts.append(i)
    cuts.append(n)
    return [(cuts[k], cuts[k + 1]) for k in range(len(cuts) - 1)]


def interp_goal(t_sim, t_goal, goals_p, goals_q):
    """状态时刻的目标(分段保持;含起点外推)。"""
    t = np.asarray(t_sim, dtype=np.float64)
    idx = np.searchsorted(np.asarray(t_goal, dtype=np.float64), t,
                          side="right") - 1
    idx = np.clip(idx, 0, len(t_goal) - 1)
    return goals_p[idx], goals_q[idx]


def error_series(pos, quats, goals_p, goals_q):
    """位置误差[m](世界系)与姿态误差角[rad]序列。"""
    pos = np.asarray(pos, dtype=np.float64).reshape(-1, 3)
    e_p = np.linalg.norm(goals_p - pos, axis=1)
    e_R = np.array([
        np.linalg.norm(attitude_error_rotvec(quats[i], goals_q[i]))
        for i in range(len(quats))])
    return e_p, e_R


def first_tol_time(t, err, tol):
    """首次进入容差并保持 ≥0.5s 的时刻;未达到返回 None。"""
    inside = np.asarray(err) <= tol
    t = np.asarray(t, dtype=np.float64)
    for i in range(len(t)):
        if inside[i]:
            j = i
            while j < len(t) and inside[j]:
                j += 1
            if j < len(t) and t[min(j, len(t) - 1)] - t[i] >= 0.5:
                return float(t[i])
            if j >= len(t) and len(t) and t[-1] - t[i] >= 0.5:
                return float(t[i])
    return None


def settle_time(t, err, tol):
    """进入容差后最后一次离开容差的时刻(其后一直保持);None=未稳定。"""
    inside = np.asarray(err) <= tol
    t = np.asarray(t, dtype=np.float64)
    if not np.any(inside):
        return None
    last_out = -1
    for i in range(len(t)):
        if not inside[i]:
            last_out = i
    return 0.0 if last_out < 0 else float(t[last_out])


def overshoot(t, err, tol):
    """超调:首次达容差后的最大误差 / 初始误差(无首次达容差则 None)。"""
    t0 = first_tol_time(t, err, tol)
    if t0 is None:
        return None
    t = np.asarray(t)
    after = err[t >= t0]
    e0 = float(err[0]) if len(err) else 0.0
    if e0 <= 1e-9:
        return None
    return float(np.max(after) / e0)


def disturbance_metrics(t, err, disturb_mask, tol):
    """扰动窗内峰值偏差与扰动撤除后的恢复时间(回容差)。

    disturb_mask: 扰动生效样本布尔序列(来自扰动力记录 |F|>0)。"""
    t = np.asarray(t, dtype=np.float64)
    err = np.asarray(err, dtype=np.float64)
    mask = np.asarray(disturb_mask, dtype=bool)
    out = {"peak_dev_m": None, "recovery_s": None}
    if not np.any(mask):
        return out
    on_idx = np.where(mask)[0]
    base = float(np.median(err[:max(on_idx[0], 1)])) if on_idx[0] > 0 \
        else 0.0
    out["peak_dev_m"] = float(np.max(err[on_idx]) - base)
    off_idx = np.where(~mask & (np.arange(len(t)) > on_idx[-1]))[0]
    if len(off_idx):
        rec = first_tol_time(t[off_idx], err[off_idx], tol)
        out["recovery_s"] = (None if rec is None
                             else float(rec - t[off_idx[0]]))
    return out


def episode_metrics(rec, pos_tol_m=0.08, ang_tol_rad=0.087, fmax_n=39.2266):
    """一个回合记录 dict -> 指标 dict。rec 字段见 analyze.load_episode。"""
    t = np.asarray(rec["t_sim"], dtype=np.float64)
    segs = split_at_time_jumps(t)
    m = {"n_segments": len(segs), "duration_s": float(t[-1] - t[0])
         if len(t) else 0.0}
    # 只对最长段算运动指标(时间跳变多为复位残留)
    if not segs:
        return m
    s, e = max(segs, key=lambda ab: ab[1] - ab[0])
    ts = t[s:e]
    e_p, e_R = rec["err_p"], rec["err_r"]          # 已按段切好
    e_p, e_R = np.asarray(e_p)[s:e], np.asarray(e_R)[s:e]
    vel = np.asarray(rec["v"], dtype=np.float64)[s:e]
    om = np.asarray(rec["w"], dtype=np.float64)[s:e]
    m.update({
        "rmse_pos_m": float(np.sqrt(np.mean(e_p ** 2))),
        "max_pos_err_m": float(np.max(e_p)),
        "rmse_ang_rad": float(np.sqrt(np.mean(e_R ** 2))),
        "max_ang_err_rad": float(np.max(e_R)),
        "final_pos_err_m": float(e_p[-1]) if len(e_p) else None,
        "final_ang_err_rad": float(e_R[-1]) if len(e_R) else None,
        "vel_rms_mps": float(np.sqrt(np.mean(np.sum(vel ** 2, axis=1)))),
        "ang_vel_rms_radps": float(np.sqrt(np.mean(
            np.sum(om ** 2, axis=1)))),
    })
    ft = first_tol_time(ts, e_p, pos_tol_m)
    m["first_tol_pos_s"] = None if ft is None else float(ft - ts[0])
    m["settle_pos_s"] = (None if settle_time(ts, e_p, pos_tol_m) is None
                         else float(settle_time(ts, e_p, pos_tol_m) - ts[0]))
    m["overshoot_pos"] = overshoot(ts, e_p, pos_tol_m)
    tail = max(int(0.1 * len(e_p)), 1)
    m["steady_state_pos_m"] = float(np.mean(e_p[-tail:]))
    # 推力指标(限幅后 applied)
    if len(rec.get("t_thr", [])):
        ua = np.asarray(rec["u_applied"], dtype=np.float64)
        ur = np.asarray(rec["u_req"], dtype=np.float64)
        if ua.shape[0] and ua.shape == ur.shape:
            sat = np.any(np.abs(ua) >= 0.99 * fmax_n, axis=1)
            m["saturation_frac"] = float(np.mean(sat))
            m["thrust_variation_n"] = float(np.mean(
                np.linalg.norm(np.diff(ua, axis=0), axis=1))) \
                if len(ua) > 1 else 0.0
            m["thrust_sq_integral_n2s"] = float(np.sum(
                np.sum(ua ** 2, axis=1)) * np.mean(np.diff(
                    np.asarray(rec["t_thr"], dtype=np.float64)))
                if len(ua) > 1 else 0.0)
            m["req_vs_achieved_max_diff_n"] = float(np.max(
                np.abs(ur - ua)))
    # 扰动指标
    if len(rec.get("t_dis", [])):
        td = np.asarray(rec["t_dis"], dtype=np.float64)
        fmag = np.linalg.norm(np.asarray(rec["f_dis"], dtype=np.float64)
                              .reshape(-1, 2, 3), axis=2)
        mask_all = np.any(fmag > 0.05, axis=1)
        # 对齐到状态时间轴(最近邻)
        idx = np.searchsorted(ts, td, side="right") - 1
        ok = (idx >= 0) & (idx < len(ts))
        state_mask = np.zeros(len(ts), dtype=bool)
        state_mask[idx[ok]] = mask_all[ok]
        m["disturb_force_max_n"] = float(np.max(fmag))
        dm = disturbance_metrics(ts, e_p, state_mask, pos_tol_m)
        m["disturb_peak_dev_m"] = dm["peak_dev_m"]
        m["disturb_recovery_s"] = dm["recovery_s"]
    # 推理耗时(墙钟,诊断)
    infer = np.asarray([x for x in rec.get("infer_ms", []) if x is not None],
                       dtype=np.float64)
    if len(infer):
        m["infer_ms_p95"] = float(np.percentile(infer, 95))
        m["infer_ms_p99"] = float(np.percentile(infer, 99))
        m["infer_ms_mean"] = float(np.mean(infer))
    m["success"] = bool(
        m.get("final_pos_err_m") is not None and
        m["final_pos_err_m"] < pos_tol_m and
        m.get("final_ang_err_rad") is not None and
        m["final_ang_err_rad"] < ang_tol_rad)
    return m
