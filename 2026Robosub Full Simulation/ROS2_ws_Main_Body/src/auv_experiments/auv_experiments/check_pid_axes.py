#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_pid_axes —— PID 六轴低幅闭环方向核验(Isaac python 运行)。

  /isaacsim/python.sh check_pid_axes.py [--out dir] [--amp 0.15]

任务书 §12 验收项:六轴(±x ±y ±z roll pitch yaw)逐轴给定小幅参考,
闭环运行后误差应收敛(方向/符号正确的总检)。每轴输出:
初始误差 -> 末段误差、容差内时长占比、饱和占比、是否发散。
判定通过:末段误差 < 30% 初始误差 且 未发散。
"""
import argparse
import json
import os
import sys

WS = ("/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/"
      "2026Robosub Full Simulation/ROS2_ws_Main_Body")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.expanduser(
        "~/auv_experiments/axis_check"))
    ap.add_argument("--amp", type=float, default=0.15,
                    help="平移幅值 [m];转动固定 15°")
    ap.add_argument("--dur", type=float, default=6.0, help="每轴闭环时长 s")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    logf = open(os.path.join(args.out, "axis_check.log"), "w",
                encoding="utf-8")

    def log(m):
        logf.write("%s\n" % m)
        logf.flush()

    sys.path.insert(0, WS + "/src/auv_common")
    sys.path.insert(0, WS + "/src/auv_controller_pid")
    sys.path.insert(0, WS + "/src/auv_controller_gru")
    sys.path.insert(0, WS + "/src/auv_experiments")

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import numpy as np
    import yaml
    from auv_controller_gru.env.sim_runtime import HeadlessAuvSim
    from auv_controller_pid.pid_core import PidCore
    from auv_common.alloc import ThrusterGeometry
    from auv_common.frames import (quat_about_axis, quat_mul, quat_normalize,
                                   quat_to_matrix, attitude_error_rotvec)

    sim = HeadlessAuvSim(app, bridge_config=WS + "/src/auv_isaac_bridge/"
                         "config/isaac_bridge_headless.json", log=log)
    geom = ThrusterGeometry.load_default()
    with open(os.path.join(WS, "src", "auv_controller_pid", "config",
                           "pid.yaml"), encoding="utf-8") as f:
        y = yaml.safe_load(f)
    pcfg = None
    if isinstance(y, dict):
        if isinstance(y.get("pid"), dict):
            pcfg = y["pid"]
        else:
            for v in y.values():
                rp = v.get("ros__parameters") if isinstance(v, dict) else None
                if isinstance(rp, dict) and "pid" in rp:
                    pcfg = rp["pid"]
                    break
    if pcfg is None:
        raise SystemExit("pid.yaml 结构无法解析(找不到 pid 段)")

    spawn_p = sim.spawn_p.copy()
    spawn_q = sim.spawn_q.copy()
    R0 = quat_to_matrix(spawn_q)   # 出生姿态的体->世界旋转

    ang = np.deg2rad(15.0)
    axes = [
        ("surge_+z", np.array([0, 0, args.amp]), None),
        ("sway_+x", np.array([args.amp, 0, 0]), None),
        ("heave_+y", np.array([0, args.amp, 0]), None),
        ("roll_+z", None, quat_about_axis(np.array([0.0, 0.0, 1.0]), ang)),
        ("pitch_+x", None, quat_about_axis(np.array([1.0, 0.0, 0.0]), ang)),
        ("yaw_+y", None, quat_about_axis(np.array([0.0, 1.0, 0.0]), ang)),
    ]

    dt = 0.05
    results = []
    for name, d_body, dq in axes:
        if d_body is not None:
            goal_p = spawn_p + R0.dot(d_body)
            goal_q = spawn_q.copy()
        else:
            goal_p = spawn_p.copy()
            goal_q = quat_normalize(quat_mul(spawn_q, dq))
        pid = PidCore(kp=pcfg["kp"], ki=pcfg["ki"], kd=pcfg["kd"],
                      integral_limit=pcfg["integral_limit"],
                      d_filter_hz=pcfg["d_filter_hz"],
                      back_calc_gain=pcfg["back_calc_gain"])
        if not sim.reset_robot(spawn_p, spawn_q):
            log("[%s] reset 失败(继续,结果仅供参考)" % name)
        sim.disturb_stop()
        sim.reset_output()
        pid.reset()
        sim.step_for(dt)
        st = sim.state()
        t0 = float(st["sim_time_s"])
        e0_p = e0_r = None
        ep_h, er_h, sat_h, thr_pk = [], [], [], 0.0
        alloc = None
        diverged = False
        while float(st["sim_time_s"]) - t0 < args.dur:
            st = sim.state()
            p = np.asarray(st["pos_m"]); q = np.asarray(st["quat_xyzw"])
            v = np.asarray(st["v_body"]); w = np.asarray(st["w_body"])
            if not (np.all(np.isfinite(p)) and np.all(np.isfinite(q))):
                diverged = True
                break
            e_p = float(np.linalg.norm(goal_p - p))
            e_r = float(np.linalg.norm(attitude_error_rotvec(q, goal_q)))
            if e0_p is None:
                e0_p, e0_r = e_p, e_r
            ep_h.append(e_p); er_h.append(e_r)
            tau = pid.update(p, q, v, w, goal_p, goal_q, dt,
                             alloc_result=alloc)
            alloc = geom.allocate_bounded(tau)
            sat_h.append(alloc["n_saturated"])
            thr_pk = max(thr_pk, float(np.max(np.abs(alloc["f"]))))
            sim.set_command(alloc["f"])
            sim.step_for(dt)
            if float(st["sim_time_s"]) - t0 > 0.5 and \
                    np.linalg.norm(p - spawn_p) > 2.5:
                diverged = True
                break
        sim.set_command(np.zeros(8))
        n_tail = max(len(ep_h) // 5, 1)
        e_end_p = float(np.mean(ep_h[-n_tail:])) if ep_h else None
        e_end_r = float(np.mean(er_h[-n_tail:])) if er_h else None
        ok = (not diverged and e_end_p is not None and
              e_end_p < 0.30 * max(e0_p, 1e-6) + 0.02 and
              e_end_r < 0.30 * max(e0_r, 1e-6) + 0.02)
        row = dict(axis=name, e0_p=e0_p, e_end_p=e_end_p,
                   e0_r=e0_r, e_end_r=e_end_r,
                   sat_frac=float(np.mean(np.asarray(sat_h) > 0))
                   if sat_h else None,
                   thrust_peak_n=thr_pk, diverged=diverged, passed=bool(ok))
        results.append(row)
        log("[%s] e_p %.3f->%.3f  e_r %.3f->%.3f  sat=%.2f pk=%.1f "
            "diverged=%s => %s"
            % (name, e0_p or -1, e_end_p if e_end_p is not None else -1,
               e0_r or -1, e_end_r if e_end_r is not None else -1,
               row["sat_frac"] or 0, thr_pk, diverged,
               "PASS" if ok else "FAIL"))

    summary = {
        "n_pass": sum(1 for r in results if r["passed"]),
        "n_total": len(results),
        "amp_m": args.amp, "dur_s": args.dur, "dt": dt,
        "gains": {k: pcfg[k] for k in ("kp", "ki", "kd")},
        "axes": results,
    }
    with open(os.path.join(args.out, "axis_check.json"), "w",
              encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    log("六轴方向核验: %d/%d 通过" % (summary["n_pass"], summary["n_total"]))

    try:
        sim.bridge.reset_output()
        sim.bridge.shutdown()
    except Exception:                                       # noqa: BLE001
        pass
    app.close()
    return 0 if summary["n_pass"] == summary["n_total"] else 2


if __name__ == "__main__":
    sys.exit(main())
