#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tune_pid —— PID 有界自动整定(进程内真实仿真,Isaac python 运行)。

  /isaacsim/python.sh tune_pid.py --wall-budget-s 1800 [--out dir]

流程(任务书 §6,如实逐步执行并记录):
  1) 零输入保位核验(小增益下不应发散 —— 方向/符号总检);
  2) 坐标下降搜索:先 P(平移/转动两组),再 D,最后 I(仅当存在
     稳态误差才引入);候选为基线增益的倍率;每组在固定任务集
     (单轴双向 + 偏航双向)上评测,目标函数加权 RMSE/稳定时间/
     饱和占比/发散;
  3) 组合任务 + 固定扰动(step 固定向量)评测最终候选;
  4) 独立验证集(eval_set,与整定任务不同)复评 —— 只评不再调;
  5) 产出 pid_tuned_YYYY-MM-DD.yaml(pid.launch.py params:= 直接加载)
     与 tuning_log.json(每一次评测逐条如实记录,含被拒候选)。

有界性:wall_budget_s 与 max_evals 双限,任一耗尽即停并如实记录。
中止条件:单次评测发散(OOB/NaN)记 inf 目标值并保留预算继续其它候选。
"""

import argparse
import datetime as _dt
import json
import os
import sys
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))


class PidEpisodeRunner(object):
    """进程内 PID 闭环评测(与 pid_controller_node 同一 PidCore/分配器)。"""

    def __init__(self, sim, geom, control_dt=0.05):
        from auv_controller_pid.pid_core import PidCore
        self.sim = sim
        self.geom = geom
        self.dt = float(control_dt)
        self._pid_cls = PidCore
        self.spawn_p = sim.spawn_p.copy()
        self.spawn_q = sim.spawn_q.copy()

    def run(self, gains, goal_p, goal_q, duration_s=12.0,
            disturb=None, log=print):
        """一次闭环回合;返回评测 dict。gains=dict(kp,ki,kd,...)。"""
        pid = self._pid_cls(
            kp=gains["kp"], ki=gains["ki"], kd=gains["kd"],
            integral_limit=gains.get(
                "integral_limit", (12.0, 12.0, 12.0, 1.5, 1.5, 1.5)),
            d_filter_hz=gains.get("d_filter_hz", 8.0),
            back_calc_gain=gains.get("back_calc_gain", 0.4))
        if not self.sim.reset_robot(self.spawn_p, self.spawn_q):
            log("[tune] reset_robot 核验失败")
        if disturb:
            self.sim.disturb_configure(disturb)
        else:
            self.sim.disturb_stop()
        self.sim.reset_output()
        pid.reset()
        self.sim.step_for(self.dt)
        st = self.sim.state()
        t0 = float(st["sim_time_s"])
        errs, angs, sats, thr = [], [], [], []
        alloc = None
        diverged = False
        from auv_common.frames import attitude_error_rotvec
        while float(st["sim_time_s"]) - t0 < duration_s:
            st = self.sim.state()
            p = np.asarray(st["pos_m"]); q = np.asarray(st["quat_xyzw"])
            v = np.asarray(st["v_body"]); w = np.asarray(st["w_body"])
            if not (np.all(np.isfinite(p)) and np.all(np.isfinite(v))):
                diverged = True
                break
            errs.append(float(np.linalg.norm(goal_p - p)))
            angs.append(float(np.linalg.norm(
                attitude_error_rotvec(q, goal_q))))
            tau = pid.update(p, q, v, w, goal_p, goal_q, self.dt,
                             alloc_result=alloc)
            alloc = self.geom.allocate_bounded(tau)
            self.sim.set_command(alloc["f"])
            sats.append(alloc["n_saturated"])
            thr.append(alloc["f"].copy())
            self.sim.step_for(self.dt)
            if float(st["sim_time_s"]) - t0 > 0.5 and \
                    np.linalg.norm(p - self.spawn_p) > 2.5:
                diverged = True
                break
        self.sim.set_command(np.zeros(8))
        errs = np.asarray(errs)
        angs = np.asarray(angs)
        out = {
            "n_steps": int(len(errs)),
            "rmse_pos_m": float(np.sqrt(np.mean(errs ** 2)))
            if len(errs) else None,
            "final_pos_m": float(errs[-1]) if len(errs) else None,
            "rmse_ang_rad": float(np.sqrt(np.mean(angs ** 2)))
            if len(angs) else None,
            "sat_frac": float(np.mean(np.asarray(sats) > 0))
            if len(sats) else None,
            "diverged": bool(diverged),
            "thrust_peak_n": float(np.max(np.abs(np.asarray(thr))))
            if len(thr) else None,
        }
        return out


def objective(ev):
    """加权目标(越小越好);发散=inf。"""
    if ev["diverged"] or ev["rmse_pos_m"] is None:
        return float("inf")
    j = ev["rmse_pos_m"] + 0.3 * (ev["rmse_ang_rad"] or 0.0)
    if ev["final_pos_m"] is not None:
        j += 0.5 * ev["final_pos_m"]
    j += 3.0 * (ev["sat_frac"] or 0.0)
    return float(j)


def tune_tasks(spawn_p, spawn_q):
    from auv_experiments.tasks import _quat_to_R
    Rw = _quat_to_R(spawn_q)
    out = []
    for name, db, ax, deg in (
            ("fwd+", [0, 0, 0.4], None, 0.0),
            ("fwd-", [0, 0, -0.4], None, 0.0),
            ("side+", [0.4, 0, 0], None, 0.0),
            ("side-", [-0.4, 0, 0], None, 0.0),
            ("yaw+", [0, 0, 0.0], [0, 1, 0], 30.0),
            ("yaw-", [0, 0, 0.0], [0, 1, 0], -30.0)):
        g_p = spawn_p + Rw @ np.asarray(db, dtype=float)
        g_q = spawn_q if ax is None else _quat_q(spawn_q, ax, deg)
        out.append((name, g_p, g_q))
    return out


def _quat_q(spawn_q, ax, deg):
    from auv_common.frames import quat_about_axis, quat_mul
    return quat_mul(spawn_q, quat_about_axis(np.asarray(ax, dtype=float),
                                             np.deg2rad(deg)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wall-budget-s", type=float, default=1800.0)
    ap.add_argument("--max-evals", type=int, default=48)
    ap.add_argument("--out", default="")
    ap.add_argument("--base", default=os.path.join(
        HERE, "..", "..", "auv_controller_pid", "config", "pid.yaml"))
    args = ap.parse_args()

    out_dir = os.path.abspath(args.out or os.path.join(
        HERE, "..", "..", "..", "runs", "pid_tune_%s" %
        time.strftime("%Y%m%d_%H%M%S")))
    os.makedirs(out_dir, exist_ok=True)
    logf = open(os.path.join(out_dir, "tuning.log"), "a", encoding="utf-8")

    def log(m):
        logf.write("%s %s\n" % (time.strftime("%H:%M:%S"), m))
        logf.flush()
        try:
            print(m)
        except Exception:                               # noqa: BLE001
            pass

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    ws = os.environ.get("AUV_WS", "/home/liuhao/LWKProjectWorkspaces/"
                         "AUVIssacSIMWorkspace/2026Robosub Full Simulation/"
                         "ROS2_ws_Main_Body")
    for p in (os.path.join(ws, "src", "auv_common"),
              os.path.join(ws, "src", "auv_controller_pid"),
              os.path.join(ws, "src", "auv_controller_gru"),
              os.path.join(ws, "src", "auv_experiments")):
        if p not in sys.path:
            sys.path.insert(0, p)

    from auv_common.alloc import ThrusterGeometry
    from auv_controller_gru.env.sim_runtime import HeadlessAuvSim
    from auv_experiments.tasks import eval_set

    base_yaml = yaml.safe_load(open(args.base, encoding="utf-8"))
    base = dict(base_yaml["/**"]["ros__parameters"]["pid"])
    gains = {
        "kp": list(map(float, base["kp"])),
        "ki": list(map(float, base["ki"])),
        "kd": list(map(float, base["kd"])),
        "integral_limit": list(map(float, base["integral_limit"])),
        "d_filter_hz": float(base.get("d_filter_hz", 8.0)),
        "back_calc_gain": float(base.get("back_calc_gain", 0.4)),
    }
    bridge_cfg = os.path.join(ws, "src", "auv_isaac_bridge", "config",
                              "isaac_bridge_headless.json")
    sim = HeadlessAuvSim(app, bridge_config=bridge_cfg,
                         physics_dt=1.0 / 120.0, log=log)
    geom = ThrusterGeometry.load_default()
    runner = PidEpisodeRunner(sim, geom)
    tasks = tune_tasks(sim.spawn_p, sim.spawn_q)

    t_start = time.monotonic()
    log_entries = []
    n_evals = [0]

    def budget_left():
        return args.wall_budget_s - (time.monotonic() - t_start)

    def evaluate(g, tag, duration_s=12.0, disturb=None):
        if n_evals[0] >= args.max_evals or budget_left() <= 0.0:
            return None
        n_evals[0] += 1
        evs = []
        for name, gp, gq in tasks:
            ev = runner.run(g, gp, gq, duration_s=duration_s,
                            disturb=disturb, log=log)
            evs.append(ev)
            if budget_left() <= 0.0:
                break
        j = float(np.mean([objective(e) for e in evs])) if evs else \
            float("inf")
        rec = {"tag": tag, "gains": {k: list(map(float, v)) if
                                     isinstance(v, (list, tuple)) else v
                                     for k, v in g.items()},
               "objective": j,
               "per_task": [dict(e) for e in evs],
               "budget_left_s": round(budget_left(), 1)}
        log_entries.append(rec)
        log("[eval %d] %-22s J=%.4f  final=%.3f sat=%.2f div=%s" % (
            n_evals[0], tag, j,
            float(np.mean([e["final_pos_m"] or 9.9 for e in evs])),
            float(np.mean([e["sat_frac"] or 0.0 for e in evs])),
            any(e["diverged"] for e in evs)))
        return j

    # ---- 1) 零输入保位核验
    log("[tune] 阶段0: 零输入保位(基线增益,目标=出生位姿)")
    zh = runner.run(gains, sim.spawn_p.copy(), sim.spawn_q.copy(),
                    duration_s=6.0, log=log)
    log("[tune] 保位漂移 final=%.4f m rmse=%.4f div=%s" % (
        zh["final_pos_m"] or -1.0, zh["rmse_pos_m"] or -1.0,
        zh["diverged"]))
    log_entries.append({"tag": "zero_hold_baseline", "result": zh})
    if zh["diverged"]:
        log("[tune] 保位即发散:中止整定(检查方向/几何)")
        _write_outputs(out_dir, gains, log_entries, "aborted_zero_hold")
        sim.close()
        app.close()
        return

    # ---- 2) 坐标下降:P(平移/转动) -> D(平移/转动) -> I
    cur = {k: (list(v) if isinstance(v, (list, tuple)) else v)
           for k, v in gains.items()}

    def _gains_key(g):
        return json.dumps(
            {k: (list(v) if isinstance(v, (list, tuple)) else v)
             for k, v in g.items()}, sort_keys=True)

    best = _gains_key(cur)
    j_cur = evaluate(cur, "baseline")
    if j_cur is None:
        log("[tune] 预算耗尽于基线评测")
    # 名义参考:基线值为 0 的组(如 ki)按名义值给倍率探针,否则永乘 0
    NOMINAL = {"kp": [30.0] * 3 + [2.5] * 3,
               "kd": [60.0] * 3 + [1.2] * 3,
               "ki": [2.0] * 3 + [0.2] * 3}
    groups = [
        ("P_trans", "kp", 0, (0.5, 1.0, 2.0)),
        ("P_rot", "kp", 3, (0.5, 1.0, 2.0)),
        ("D_trans", "kd", 0, (0.5, 1.0, 2.0)),
        ("D_rot", "kd", 3, (0.5, 1.0, 2.0)),
        ("I_trans", "ki", 0, (0.0, 0.3, 1.0)),
        ("I_rot", "ki", 3, (0.0, 0.3, 1.0)),
    ]
    for gname, key, off, mults in groups:
        for m in mults:
            if budget_left() <= 0.0 or n_evals[0] >= args.max_evals:
                log("[tune] 预算耗尽,停在 %s" % gname)
                break
            cand = {k: (list(v) if isinstance(v, (list, tuple)) else v)
                    for k, v in cur.items()}
            for i in range(3):
                ref = float(base[key][off + i])
                if ref <= 0.0:
                    ref = float(NOMINAL[key][off + i])
                cand[key][off + i] = ref * m
            if _gains_key(cand) == _gains_key(cur):
                continue                      # 与当前相同,不重复评测
            tag = "%s_x%.2f" % (gname, m)
            j = evaluate(cand, tag)
            if j is None:
                break
            if j < j_cur - 1e-4:
                cur, j_cur = cand, j
                best = _gains_key(cur)
                log("[tune] 接受 %s J=%.4f" % (tag, j))
        else:
            continue
        break
    final_gains = json.loads(best)

    # ---- 3) 组合 + 固定扰动(只评不调)
    combo_p = sim.spawn_p + np.array([0.3, 0.2, 0.3])
    combo_ev = runner.run(final_gains, combo_p, sim.spawn_q.copy(),
                          duration_s=15.0, log=log)
    fixed_dist = {"enabled": True, "mode": "step",
                  "force_max_n": 11.77, "frame": "body",
                  "fixed_vec": [11.77, 0.0, 0.0]}
    dist_ev = runner.run(final_gains, sim.spawn_p.copy(),
                         sim.spawn_q.copy(), duration_s=15.0,
                         disturb=fixed_dist, log=log)
    log("[tune] 组合: J=%.4f final=%.3f | 固定扰动: final=%.3f div=%s"
        % (objective(combo_ev), combo_ev["final_pos_m"] or -1,
           dist_ev["final_pos_m"] or -1, dist_ev["diverged"]))
    log_entries.append({"tag": "final_combo", "result": combo_ev})
    log_entries.append({"tag": "final_fixed_disturb", "result": dist_ev})

    # ---- 4) 独立验证集
    val = []
    for name, g, dist, dur, _p in eval_set(sim.spawn_p, sim.spawn_q):
        d = dict(dist) if dist else {}
        ev = runner.run(final_gains, g["p"], g["q"], duration_s=dur,
                        disturb=(d if d.get("enabled") else None), log=log)
        val.append({"task": name, **ev})
        log("[val] %-12s final=%.3f rmse=%.3f succ=%s" % (
            name, ev["final_pos_m"] or -1, ev["rmse_pos_m"] or -1,
            (ev["final_pos_m"] or 9) < 0.08 and not ev["diverged"]))
    log_entries.append({"tag": "independent_validation", "results": val})

    stop = "budget" if budget_left() <= 0.0 or n_evals[0] >= \
        args.max_evals else "done"
    _write_outputs(out_dir, final_gains, log_entries, stop,
                   base_yaml["/**"]["ros__parameters"])
    log("[tune] 完成(%s): %s" % (stop, out_dir))
    sim.close()
    app.close()


def _write_outputs(out_dir, gains, log_entries, stop, base_params=None):
    today = _dt.date.today().isoformat()
    kp = [float(x) for x in gains["kp"]]
    ki = [float(x) for x in gains["ki"]]
    kd = [float(x) for x in gains["kd"]]
    yaml_path = os.path.join(out_dir, "pid_tuned_%s.yaml" % today)
    topics = base_params.get("topics", {
        "state": "/auv/state/control", "goal": "/auv/control/goal"}) \
        if base_params else {}
    doc = {
        "/**": {
            "ros__parameters": {
                "topics": topics,
                "rates": {"control_hz": 50.0},
                "safety": {"state_timeout_wall_s": 0.6},
                "pid": {
                    "kp": kp, "ki": ki, "kd": kd,
                    "integral_limit": [float(x) for x in
                                       gains["integral_limit"]],
                    "output_limit": [1.0e9] * 6,
                    "d_filter_hz": float(gains.get("d_filter_hz", 8.0)),
                    "back_calc_gain": float(gains.get("back_calc_gain",
                                                      0.4)),
                    "enable_feedforward": False,
                    "feedforward_force_body": [0.0, 0.0, 0.0],
                    "hold_on_start": True,
                },
            },
        },
    }
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False)
    with open(os.path.join(out_dir, "tuning_log.json"), "w",
              encoding="utf-8") as f:
        json.dump({"stop": stop, "gains": {"kp": kp, "ki": ki, "kd": kd},
                   "evals": log_entries,
                   "note": "目标值为加权指标(越小越好);inf=发散候选;"
                           "全部候选如实记录"},
                  f, ensure_ascii=False, indent=2, default=str)
    return yaml_path


if __name__ == "__main__":
    main()
