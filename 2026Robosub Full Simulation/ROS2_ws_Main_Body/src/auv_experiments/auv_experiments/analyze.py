#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""analyze.py —— 离线指标复算 + 绘图导出(Isaac python 运行:

  /isaacsim/python.sh analyze.py --run_dir <运行目录> [--pdf] [--compare]

对 episodes/*/ 的 CSV 重算指标(与在线同一 metrics 模块,可对账),导出:
  per-episode metrics.json + summary.csv(离线)
  plots/: 轨迹3D / 误差曲线 / 推力曲线 / 扰动曲线 / 恢复放大 / 多回合对比
  PNG 一律导出;--pdf 追加 PDF 矢量版。
系统 python 的 matplotlib 与 pyzed numpy 冲突,故指定 Isaac python。
"""

import argparse
import csv
import json
import os

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt            # noqa: E402
from mpl_toolkits.mplot3d import Axes3D    # noqa: F401,E402

from auv_common.frames import attitude_error_rotvec  # noqa: E402
from auv_experiments import metrics as M             # noqa: E402

FIG_DPI = 110


def load_episode(ep_dir):
    """CSV -> rec dict(metrics.episode_metrics 输入契约)。"""
    def _rows(fn):
        p = os.path.join(ep_dir, fn)
        if not os.path.isfile(p):
            return []
        with open(p, encoding="utf-8") as f:
            return list(csv.reader(f))[1:]

    states = _rows("states.csv")
    goals = _rows("goals.csv")
    if not states or not goals:
        return None
    t = np.array([float(r[0]) for r in states])
    p = np.array([[float(x) for x in r[2:5]] for r in states])
    quats = np.array([[float(x) for x in r[5:9]] for r in states])
    v = np.array([[float(x) for x in r[9:12]] for r in states])
    w = np.array([[float(x) for x in r[12:15]] for r in states])
    tg = np.array([float(r[0]) for r in goals])
    gp = np.array([[float(x) for x in r[2:5]] for r in goals])
    gq = np.array([[float(x) for x in r[5:9]] for r in goals])
    GP, GQ = M.interp_goal(t, tg, gp, gq)
    e_p, e_R = M.error_series(p, quats, GP, GQ)

    rec = {"t_sim": t, "p": p, "q": quats, "v": v, "w": w,
           "err_p": e_p, "err_r": e_R, "goal_p": GP, "goal_q": GQ,
           "t_thr": [], "u_req": np.zeros((0, 8)),
           "u_applied": np.zeros((0, 8)), "t_dis": [],
           "f_dis": np.zeros((0, 6)), "f_pos": np.zeros((0, 6)),
           "infer_ms": []}
    # 推力
    thr = _rows("thrust.csv")
    req, app, treq, tapp = [], [], [], []
    for r in thr:
        t_wall, kind = float(r[0]), r[1]
        u = [float(x) for x in r[2:10]]
        (req if kind == "req" else app).append(u)
        (treq if kind == "req" else tapp).append(t_wall)
    if req:
        rec["u_req"] = np.asarray(req)
        rec["t_thr"] = np.asarray(treq)
    if app:
        rec["u_applied"] = np.asarray(app)
        rec["t_thr_applied"] = np.asarray(tapp)
    # 扰动(只取完整行:桥端旧版本无扰动时会发空消息,CSV 行只有 1 列;
    # t_dis 与 f_dis 必须同长对齐,否则下游掩码索引错位崩溃)
    dis = [r for r in _rows("disturbance.csv") if len(r) >= 7]
    if dis:
        rec["t_dis"] = np.array([float(r[0]) for r in dis])
        rec["f_dis"] = np.array([[float(x) for x in r[1:7]] for r in dis])
        rec["f_pos"] = np.array([[float(x) for x in r[7:13]]
                                 for r in dis if len(r) >= 13]
                                ) if any(len(r) >= 13 for r in dis) \
            else np.zeros((0, 6))
    # 诊断
    jl = os.path.join(ep_dir, "diag.jsonl")
    if os.path.isfile(jl):
        with open(jl, encoding="utf-8") as f:
            for line in f:
                try:
                    d = json.loads(line)
                except Exception:                       # noqa: BLE001
                    continue
                if d.get("infer_ms") is not None:
                    rec["infer_ms"].append(d["infer_ms"])
    md = {}
    mdp = os.path.join(ep_dir, "metadata.json")
    if os.path.isfile(mdp):
        md = json.load(open(mdp, encoding="utf-8"))
    return {"rec": rec, "md": md, "dir": ep_dir}


def _save(fig, out_base, pdf):
    fig.savefig(out_base + ".png", dpi=FIG_DPI, bbox_inches="tight")
    if pdf:
        fig.savefig(out_base + ".pdf", bbox_inches="tight")
    plt.close(fig)


def plot_episode(item, out_dir, pdf=False, pos_tol=0.08):
    rec, md = item["rec"], item["md"]
    name = md.get("task", os.path.basename(item["dir"]))
    t = rec["t_sim"] - rec["t_sim"][0]
    # 1) 3D 轨迹
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(rec["p"][:, 0], rec["p"][:, 1], rec["p"][:, 2],
            label="state", color="#1f77b4", lw=1.4)
    ax.scatter(rec["goal_p"][-1, 0], rec["goal_p"][-1, 1],
               rec["goal_p"][-1, 2], color="#2ca02c", s=60,
               label="goal", marker="*")
    ax.scatter(rec["p"][0, 0], rec["p"][0, 1], rec["p"][0, 2],
               color="#888", s=30, label="start")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.set_title("%s trajectory (%s)" % (name, md.get("controller", "")))
    ax.legend()
    _save(fig, os.path.join(out_dir, "%s_traj3d" % name), pdf)
    # 2) 误差曲线
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(t, rec["err_p"], color="#1f77b4", label="pos err [m]")
    ax.axhline(pos_tol, color="#1f77b4", ls="--", lw=0.8)
    ax2 = ax.twinx()
    ax2.plot(t, rec["err_r"], color="#d62728", label="ang err [rad]")
    ax.set_xlabel("t_sim [s]")
    ax.set_ylabel("pos err [m]"); ax2.set_ylabel("ang err [rad]")
    ax.set_title("%s errors" % name)
    ax.legend(loc="upper left"); ax2.legend(loc="upper right")
    _save(fig, os.path.join(out_dir, "%s_errors" % name), pdf)
    # 3) 推力(applied 与 req 时间轴各自独立,行数可能不同)
    if len(rec["u_applied"]):
        fig, ax = plt.subplots(figsize=(7, 4))
        ta = rec.get("t_thr_applied", rec["t_thr"])
        tw = ta - ta[0]
        for i in range(8):
            ax.plot(tw, rec["u_applied"][:, i], lw=0.9, label="u%d" % i)
        ax.axhline(39.2266, color="k", ls=":", lw=0.8)
        ax.axhline(-39.2266, color="k", ls=":", lw=0.8)
        ax.set_xlabel("t_wall [s]"); ax.set_ylabel("applied thrust [N]")
        ax.set_title("%s thrusts (dashed=limits)" % name)
        ax.legend(ncol=4, fontsize=7)
        _save(fig, os.path.join(out_dir, "%s_thrust" % name), pdf)
    # 4) 扰动
    if len(rec["f_dis"]):
        fig, ax = plt.subplots(figsize=(7, 4))
        td = rec["t_dis"] - rec["t_dis"][0]
        mag = np.linalg.norm(rec["f_dis"].reshape(-1, 2, 3), axis=2)
        ax.plot(td, mag[:, 0], color="#d62728", label="|F0|")
        ax.plot(td, mag[:, 1], color="#ff7f0e", label="|F1|")
        ax.axhline(19.6133, color="k", ls=":", lw=0.8, label="bound")
        ax.set_xlabel("t_wall [s]"); ax.set_ylabel("force [N]")
        ax.set_title("%s disturbance" % name)
        ax.legend()
        _save(fig, os.path.join(out_dir, "%s_disturb" % name), pdf)
        # 5) 恢复(上:误差[仿真时间];下:扰动幅[墙钟],时间轴不同,分开画)
        fig, (ax, axb) = plt.subplots(2, 1, figsize=(7, 5.5),
                                      sharex=False)
        ax.plot(t, rec["err_p"], color="#1f77b4", label="pos err")
        ax.axhline(pos_tol, color="#1f77b4", ls="--", lw=0.8)
        ax.set_ylabel("pos err [m]")
        ax.set_title("%s recovery" % name)
        ax.legend(loc="upper left")
        axb.plot(td, mag[:, 0], color="#d62728", lw=0.8, label="|F0|")
        axb.plot(td, mag[:, 1], color="#ff7f0e", lw=0.8, label="|F1|")
        axb.set_xlabel("t_wall [s]")
        axb.set_ylabel("disturb |F| [N]")
        axb.legend(loc="upper right")
        _save(fig, os.path.join(out_dir, "%s_recovery" % name), pdf)


def plot_compare(items, out_dir, pdf=False):
    """多回合对比(同任务名按回合序号)。"""
    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    for it in items:
        rec, md = it["rec"], it["md"]
        t = rec["t_sim"] - rec["t_sim"][0]
        label = "%s/%s" % (md.get("controller", "?"),
                           md.get("task", "?"))
        ax.plot(t, rec["err_p"], lw=1.0, label=label)
    ax.set_xlabel("t_sim [s]"); ax.set_ylabel("pos err [m]")
    ax.set_title("comparison")
    ax.legend(fontsize=7)
    _save(fig, os.path.join(out_dir, "compare_errors"), pdf)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run_dir", required=True)
    ap.add_argument("--pdf", action="store_true")
    ap.add_argument("--compare", action="store_true")
    ap.add_argument("--pos_tol_m", type=float, default=0.08)
    args = ap.parse_args()

    eps_dir = os.path.join(args.run_dir, "episodes")
    if not os.path.isdir(eps_dir):
        raise SystemExit("无 episodes/ 目录: %s" % args.run_dir)
    plots = os.path.join(args.run_dir, "plots")
    os.makedirs(plots, exist_ok=True)
    rows = []
    items = []
    for name in sorted(os.listdir(eps_dir)):
        ep_dir = os.path.join(eps_dir, name)
        if not os.path.isdir(ep_dir):
            continue
        item = load_episode(ep_dir)
        if item is None:
            print("跳过(无数据): %s" % name)
            continue
        m = M.episode_metrics(item["rec"], pos_tol_m=args.pos_tol_m)
        m["task"] = item["md"].get("task", name)
        m["controller"] = item["md"].get("controller", "")
        m["end_reason"] = item["md"].get("end_reason", "")
        # 与在线指标对账(在线 summary.csv 若存在)
        rows.append(m)
        with open(os.path.join(ep_dir, "metrics.json"), "w",
                  encoding="utf-8") as f:
            json.dump(m, f, ensure_ascii=False, indent=2)
        plot_episode(item, plots, args.pdf, args.pos_tol_m)
        items.append(item)
        print("%s: succ=%s rmse=%.3f settle=%s" % (
            name, m.get("success"), m.get("rmse_pos_m", float("nan")),
            m.get("settle_pos_s")))
    if rows:
        with open(os.path.join(args.run_dir, "summary_offline.csv"), "w",
                  newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=sorted(
                {k for r in rows for k in r}))
            w.writeheader()
            for r in rows:
                w.writerow(r)
    if args.compare and items:
        plot_compare(items, plots, args.pdf)
    # 在线/离线对账
    onl = os.path.join(args.run_dir, "summary.csv")
    if os.path.isfile(onl) and rows:
        online = list(csv.DictReader(open(onl, encoding="utf-8")))
        checked = 0
        mismatch = []
        for on, off in zip(online, rows):
            a, b = on.get("rmse_pos_m"), off.get("rmse_pos_m")
            if a and b is not None:
                checked += 1
                if abs(float(a) - float(b)) > 1e-6:
                    mismatch.append((on.get("task"), a, b))
        acc = os.path.join(args.run_dir, "online_offline_check.json")
        with open(acc, "w", encoding="utf-8") as f:
            json.dump({"checked": checked, "mismatch": mismatch},
                      f, ensure_ascii=False, indent=2)
        print("在线/离线对账: %d 条, 不一致 %d" % (checked, len(mismatch)))
    print("绘图输出: %s" % plots)


if __name__ == "__main__":
    main()
