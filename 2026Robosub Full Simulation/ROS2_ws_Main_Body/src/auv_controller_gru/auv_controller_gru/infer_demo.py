#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""infer_demo —— GRU 策略重载闭环推理演示(Isaac python 运行)。

  /isaacsim/python.sh infer_demo.py --npz .../gru_policy.npz \
      --meta .../gru_policy_meta.json [--steps 240] [--hz 10]

验证链:npz 重载 -> HeadlessAuvSim 复位 -> 固定目标闭环 -> 误差轨迹
JSON(+逐步命令范数)。不宣称性能,只证推理闭环可用。
"""
import argparse
import json
import os
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", required=True)
    ap.add_argument("--meta", required=True)
    ap.add_argument("--out", default=os.path.expanduser(
        "~/auv_experiments/gru_infer_demo.json"))
    ap.add_argument("--steps", type=int, default=240)
    ap.add_argument("--hz", type=float, default=10.0)
    args = ap.parse_args()

    ws = ("/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/"
          "2026Robosub Full Simulation/ROS2_ws_Main_Body")
    for p in (ws + "/src/auv_common", ws + "/src/auv_controller_gru"):
        if p not in sys.path:
            sys.path.insert(0, p)

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import numpy as np
    from auv_controller_gru.gru_policy import GruPolicyNumpy
    from auv_controller_gru.obs_builder import (build_obs, ObsNormalizer,
                                                load_metadata)
    from auv_controller_gru.env.sim_runtime import HeadlessAuvSim
    from auv_common.frames import attitude_error_rotvec, quat_to_matrix

    logf = open(args.out + ".log", "w", encoding="utf-8")

    def log(m):
        logf.write(m + "\n")
        logf.flush()

    pol = GruPolicyNumpy(args.npz)
    meta = load_metadata(args.meta)
    norm = ObsNormalizer()
    # 元数据键为 "normalizer"(save_metadata 写入;gru_controller_node 同键)。
    # 首版误读 "obs_norm" 导致归一化被静默跳过、策略吃原始观测(2026-11-08
    # 首次演示 drift 至 1.77 m 时定位),修复后须与训练评测同源。
    if isinstance(meta.get("normalizer"), dict):
        norm = ObsNormalizer.from_dict(meta["normalizer"])
    log("policy loaded: npz=%s meta=%s norm=%s"
        % (args.npz, "ok" if meta else "?", bool(meta.get("normalizer"))))

    sim = HeadlessAuvSim(app, bridge_config=ws + "/src/auv_isaac_bridge/"
                         "config/isaac_bridge_headless.json", log=log)
    spawn_p, spawn_q = sim.spawn_p.copy(), sim.spawn_q.copy()
    R0 = quat_to_matrix(spawn_q)
    goal_p = spawn_p + R0.dot(np.array([0.0, 0.0, 0.6]))
    goal_q = spawn_q.copy()

    if not sim.reset_robot(spawn_p, spawn_q):
        log("reset 失败(结果仅供参考)")
    sim.disturb_stop()
    sim.reset_output()
    dt = 1.0 / args.hz
    h = pol.reset_hidden()
    last_cmd = np.zeros(8)
    st = sim.state()
    t0 = float(st["sim_time_s"])
    rows = []
    t_wall0 = time.monotonic()
    for k in range(args.steps):
        st = sim.state()
        p = np.asarray(st["pos_m"]); q = np.asarray(st["quat_xyzw"])
        v = np.asarray(st["v_body"]); w = np.asarray(st["w_body"])
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(q))):
            log("step %d: 状态非有限,停止" % k)
            break
        obs_raw = build_obs(p, q, v, w, goal_p, goal_q,
                            last_cmd_norm=last_cmd)
        obs = norm.normalize(obs_raw)
        t_a = time.monotonic()
        a, h = pol.step(obs, h)
        infer_ms = (time.monotonic() - t_a) * 1e3
        last_cmd = a
        sim.set_command(a * 39.2266)
        rows.append({
            "k": k, "t_sim": float(st["sim_time_s"] - t0),
            "e_p": float(np.linalg.norm(goal_p - p)),
            "e_r": float(np.linalg.norm(attitude_error_rotvec(q, goal_q))),
            "cmd_norm": float(np.linalg.norm(a)),
            "infer_ms": infer_ms,
        })
        sim.step_for(dt)
    sim.set_command(np.zeros(8))

    e = [r["e_p"] for r in rows]
    inf = [r["infer_ms"] for r in rows]
    summary = {
        "npz": args.npz, "n_steps": len(rows),
        "e_p_start": e[0] if e else None,
        "e_p_end": e[-1] if e else None,
        "e_p_min": min(e) if e else None,
        "infer_ms_p95": float(np.percentile(inf, 95)) if inf else None,
        "note": "短程推理闭环演示(非收敛性能声明)",
    }
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump({"summary": summary, "rows": rows}, f,
                  ensure_ascii=False, indent=1)
    log("summary: %s" % json.dumps(summary, ensure_ascii=False))

    try:
        sim.bridge.reset_output()
        sim.bridge.shutdown()
    except Exception:                                       # noqa: BLE001
        pass
    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
