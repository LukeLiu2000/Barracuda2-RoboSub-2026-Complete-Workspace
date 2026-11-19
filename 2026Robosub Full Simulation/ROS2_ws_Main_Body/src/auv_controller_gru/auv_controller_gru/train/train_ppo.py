#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""train_ppo —— GRU 循环策略训练入口(必须在 Isaac python 下运行)。

用法(服务器):
  /isaacsim/python.sh train/train_ppo.py --config config/train.yaml \
      [--out runs/xxx] [--resume ckpt.pt] [--timesteps 60000] \
      [--wall-budget-s 5400] [--stage 1]

流程(真实执行,不伪造):
  1. SimulationApp(headless) -> HeadlessAuvSim(真实桥/真实物理);
  2. 随机策略预热采集观测,估计并【冻结】归一化参数(存入元数据);
  3. RecurrentPPO 训练:Bethe 分布 rollout + chunk 更新;
     预算:total_timesteps 与 wall_budget_s 任一到达即停(如实记录);
  4. 周期性检查点 + 独立种子评测(训练/评测种子不同);
  5. 导出 numpy 推理权重(npz)+ 元数据(json)到 out/models/;
  6. train_summary.json 如实记录:步数/预算是否提前停/最终评测/
     各回合统计(不做性能美化)。

stdout 在 kit 进程中可能不可见:全部日志写入 out/train.log。
"""

import argparse
import datetime as _dt
import json
import os
import sys
import time

import numpy as np
import yaml


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--config", default=os.path.join(here, "..", "config",
                                                     "train.yaml"))
    ap.add_argument("--out", default="")
    ap.add_argument("--resume", default="")
    ap.add_argument("--timesteps", type=int, default=0)
    ap.add_argument("--wall-budget-s", type=float, default=0.0)
    ap.add_argument("--stage", type=int, default=0)
    args = ap.parse_args()

    with open(args.config, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    tr = dict(cfg.get("train", {}))
    rw_cfg = dict(cfg.get("reward", {}))
    env_cfg = dict(cfg.get("env", {}))
    if args.timesteps:
        tr["total_timesteps"] = int(args.timesteps)
    if args.wall_budget_s:
        tr["wall_budget_s"] = float(args.wall_budget_s)
    if args.stage:
        tr["stage"] = int(args.stage)

    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.abspath(args.out or os.path.join(
        here, "..", "..", "runs", "gru_%s" % stamp))
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(os.path.join(out_dir, "models"), exist_ok=True)
    log_path = os.path.join(out_dir, "train.log")
    logf = open(log_path, "a", encoding="utf-8")

    def log(msg):
        logf.write("%s %s\n" % (time.strftime("%H:%M:%S"), msg))
        logf.flush()
        try:
            print(msg)
        except Exception:                           # noqa: BLE001
            pass

    log("[train] out_dir=%s" % out_dir)
    log("[train] cfg=%s" % json.dumps(tr))

    # ---- 1. Isaac(必须最先创建)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    ws = os.environ.get("AUV_WS", "/home/liuhao/LWKProjectWorkspaces/"
                         "AUVIssacSIMWorkspace/2026Robosub Full Simulation/"
                         "ROS2_ws_Main_Body")
    for p in (os.path.join(ws, "src", "auv_common"),
              os.path.join(ws, "src", "auv_controller_gru")):
        if p not in sys.path:
            sys.path.insert(0, p)

    import torch
    from auv_controller_gru.env.sim_runtime import HeadlessAuvSim
    from auv_controller_gru.env.isaac_env import IsaacAuvEnv
    from auv_controller_gru.gru_policy import GruActorCriticTorch
    from auv_controller_gru.obs_builder import (ObsNormalizer,
                                                save_metadata)
    from auv_controller_gru.train.gru_ppo import RecurrentPPO
    from auv_controller_gru.env.isaac_env import FMAX_N

    device = "cuda" if torch.cuda.is_available() else "cpu"
    log("[train] device=%s torch=%s" % (device, torch.__version__))

    bridge_cfg = os.path.join(ws, "src", "auv_isaac_bridge", "config",
                              "isaac_bridge_headless.json")
    sim = HeadlessAuvSim(app, bridge_config=bridge_cfg,
                         physics_dt=1.0 / float(tr.get("physics_hz", 120.0)),
                         log=log)

    stage = int(tr.get("stage", 1))
    env = IsaacAuvEnv(sim, stage=stage,
                      control_dt=float(tr.get("control_dt", 0.05)),
                      time_limit_s=float(tr.get("time_limit_s", 30.0)),
                      reward_cfg=rw_cfg,
                      oob_radius_m=float(env_cfg.get("oob_radius_m", 2.0)),
                      z_bounds=env_cfg.get("z_bounds"),
                      seed=int(tr.get("seed_train", 1000)), log=log)
    eval_env = IsaacAuvEnv(sim, stage=stage,
                           control_dt=float(tr.get("control_dt", 0.05)),
                           time_limit_s=float(tr.get("time_limit_s", 30.0)),
                           reward_cfg=rw_cfg,
                           oob_radius_m=float(env_cfg.get("oob_radius_m", 2.0)),
                           z_bounds=env_cfg.get("z_bounds"),
                           seed=int(tr.get("seed_eval", 9000)), log=log)

    # ---- 2. 预热估计归一化参数(随机动作,随后冻结)
    policy = GruActorCriticTorch()
    ppo_cfg = {k: tr[k] for k in (
        "n_steps", "lr", "gamma", "gae_lambda", "clip_eps", "entropy_coef",
        "value_coef", "epochs", "chunk_len", "batch_chunks",
        "max_grad_norm", "target_kl") if k in tr}
    if "n_steps" not in ppo_cfg:
        ppo_cfg["n_steps"] = 600

    resume_ckpt = None
    if args.resume and os.path.isfile(args.resume):
        resume_ckpt = args.resume
    else:
        warmup_n = int(tr.get("warmup_steps", 800))
        log("[train] 预热采集 %d 步(随机动作)估计归一化参数" % warmup_n)
        obs_list = []
        o, _ = env.reset()
        rng = np.random.default_rng(int(tr.get("seed_train", 1000)) + 7)
        for _i in range(warmup_n):
            obs_list.append(o.copy())
            a = rng.uniform(-1.0, 1.0, 8)
            o, _r, _te, _tr, _in = env.step(a)
            if _te or _tr:
                o, _ = env.reset()
        normalizer = ObsNormalizer.from_dataset(np.asarray(obs_list))
        log("[train] 归一化参数已冻结: mean[:3]=%s std[:3]=%s" % (
            np.round(normalizer.mean[:3], 3).tolist(),
            np.round(normalizer.std[:3], 3).tolist()))

    agent = RecurrentPPO(env, policy, normalizer if not resume_ckpt
                         else ObsNormalizer(), ppo_cfg, device=device,
                         log=log, hidden=int(tr.get("hidden", 64)))
    if resume_ckpt:
        ck = agent.load(resume_ckpt)
        log("[train] 已恢复检查点 %s(已训练 %d 步)"
            % (resume_ckpt, agent.num_steps))

    # ---- 3. 训练循环(双预算)
    total = int(tr.get("total_timesteps", 60000))
    wall_budget = float(tr.get("wall_budget_s", 5400))
    t_start = time.monotonic()
    eval_every = int(tr.get("eval_interval_steps", 6000))
    ckpt_every = int(tr.get("ckpt_interval_steps", 6000))
    eval_eps = int(tr.get("eval_episodes", 5))
    stop_reason = "timesteps_done"
    eval_jsonl = open(os.path.join(out_dir, "eval.jsonl"), "a",
                      encoding="utf-8")
    next_eval = agent.num_steps + eval_every
    next_ckpt = agent.num_steps + ckpt_every

    while agent.num_steps < total:
        v_last = agent.collect_rollout()
        u = agent.update(v_last)
        el = agent.ep_stats[-30:]
        succ = float(np.mean([e["success"] for e in el])) if el else 0.0
        log("[train] steps=%d ep_r=%.2f succ=%.2f pi=%.4f v=%.4f "
            "ent=%.3f kl=%.4f" % (
                agent.num_steps,
                float(np.mean([e["reward"] for e in el])) if el else 0.0,
                succ, u["pi_loss"], u["v_loss"], u["ent"], u["kl"]))
        if agent.num_steps >= next_eval:
            next_eval = agent.num_steps + eval_every
            res = agent.evaluate(eval_env, n_episodes=eval_eps)
            rec = {"step": agent.num_steps, "results": res,
                   "succ_rate": float(np.mean([r["success"] for r in res])),
                   "mean_pos_err": float(np.nanmean(
                       [r["pos_err_m"] for r in res]))}
            eval_jsonl.write(json.dumps(rec, ensure_ascii=False) + "\n")
            eval_jsonl.flush()
            log("[eval] step=%d succ=%.2f pos_err=%.3f m"
                % (rec["step"], rec["succ_rate"], rec["mean_pos_err"]))
        if agent.num_steps >= next_ckpt:
            next_ckpt = agent.num_steps + ckpt_every
            ck_path = os.path.join(out_dir, "ckpt_latest.pt")
            agent.save(ck_path, extra={"stage": stage, "out_dir": out_dir})
        if time.monotonic() - t_start > wall_budget:
            stop_reason = "wall_budget"
            log("[train] 墙钟预算耗尽,提前停止(如实记录)")
            break

    ck_final = os.path.join(out_dir, "ckpt_final.pt")
    agent.save(ck_final, extra={"stage": stage, "stop_reason": stop_reason})

    # ---- 4. 导出 numpy 推理产物 + 元数据
    npz_path = os.path.join(out_dir, "models", "gru_policy.npz")
    policy.export_npz(npz_path)
    final_eval = agent.evaluate(eval_env, n_episodes=max(eval_eps, 8))
    meta_path = os.path.join(out_dir, "models", "gru_policy_meta.json")
    save_metadata(meta_path, agent.norm, extra={
        "trained_steps": int(agent.num_steps),
        "stage": stage,
        "stop_reason": stop_reason,
        "train_seed": int(tr.get("seed_train", 1000)),
        "eval_seed": int(tr.get("seed_eval", 9000)),
        "fmax_n": FMAX_N,
        "created": _dt.datetime.now().isoformat(timespec="seconds"),
        "final_eval": final_eval,
    })

    summary = {
        "trained_steps": int(agent.num_steps),
        "total_timesteps_budget": total,
        "wall_budget_s": wall_budget,
        "wall_used_s": round(time.monotonic() - t_start, 1),
        "stop_reason": stop_reason,
        "stage": stage,
        "episodes": len(agent.ep_stats),
        "success_rate_last30": float(np.mean(
            [e["success"] for e in agent.ep_stats[-30:]])
            if agent.ep_stats else None),
        "final_eval": final_eval,
        "models": {"npz": npz_path, "meta": meta_path,
                   "ckpt": ck_final},
        "device": device,
        "honest_note": ("短程预算训练:目标为训练管线验证,非收敛性能;"
                        "评测数字如实记录,不宣称达标"),
    }
    with open(os.path.join(out_dir, "train_summary.json"), "w",
              encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    log("[train] 完成: %s 步(%s),产物见 %s" % (
        agent.num_steps, stop_reason, out_dir))
    eval_jsonl.close()
    logf.close()
    sim.close()
    app.close()


if __name__ == "__main__":
    main()
