# -*- coding: utf-8 -*-
"""isaac_env —— 真实 Isaac Sim 的 gymnasium 环境(GRU 训练/评测)。

与部署共用同一物理路径:动作经桥 set_command(同一校验/限幅/阻尼/几何),
观测用部署同款 build_obs(23 维,顺序见 obs_builder)。

任务生成(安全空间):出生点=资产默认位姿,目标=出生姿态系下随机位移
(分级半径 0.4/0.8/1.2 m)+ 姿态增量(单位/±30°偏航/≤60°任意轴)。
奖励为多项和:位置/姿态/距离改善/目标附近速度/动作幅值/动作变化/
保持成功/出界失败 —— 不是逐步只做差减。

回合: terminated = 出界(OOB 球 2.0 m 或 z 越界)/数值发散;
        truncated = 超时。复位 = 桥 set_robot_pose(真实 USD 写入)。
单环境同步 stepping(先单环境跑通,不伪造并行)。

观测时序约定(与 gru_controller_node 一致):决策 t 时刻的观测含
"上一物理周期实际执行的命令" a_{t-1};本文件 step(a_t) 返回的观测在
执行 a_t 之后构建,故其中 last_cmd_norm = a_t(供下一决策用)。
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces

from auv_common.frames import attitude_error_rotvec, quat_to_matrix

from ..obs_builder import build_obs
from ..task_gen import FMAX_N, TaskSampler


class IsaacAuvEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, sim, stage=1, control_dt=0.05, time_limit_s=30.0,
                 reward_cfg=None, oob_radius_m=2.0, z_bounds=None,
                 seed=0, log=print):
        super().__init__()
        self.sim = sim
        self.log = log
        self.control_dt = float(control_dt)
        self.time_limit_s = float(time_limit_s)
        self.oob_radius_m = float(oob_radius_m)
        self.z_bounds = z_bounds                 # None=不额外限 z(OOB 球已含)
        self.rw = dict(
            pos_k=2.0,            # 位置误差负奖励(线性,每米)
            pos_tol_m=0.08,
            ang_k=0.6,            # 姿态误差(每弧度)
            ang_tol_rad=0.087,    # ≈5°
            progress_k=1.5,       # 距离改善(塑形,与 pos_k 同源不额外造势)
            vel_k=0.05,           # 容差内速度惩罚(到达后抑制晃动)
            action_k=0.02,        # 动作幅值 mean(a²) 代理
            action_change_k=0.05, # 动作变化 mean((a_t-a_{t-1})²)
            hold_bonus=0.5,       # 双容差内保持成功奖励/步
            oob_penalty=-40.0,
        )
        if reward_cfg:
            self.rw.update(reward_cfg)
        self.stage = int(stage)
        self.sampler = TaskSampler()
        self._rng = np.random.default_rng(seed)
        self._spawn_p = None
        self._spawn_q = None
        self._task = None
        self._t0 = 0.0
        self._prev_a = np.zeros(8)     # 上一实际执行动作(观测用)
        self._prev_dist = None
        self._last_info = {}
        self.action_space = spaces.Box(-1.0, 1.0, (8,), np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (23,), np.float64)
        self.disturb_cfg = None        # 训练中扰动力(可选;默认关)

    # ------------------------------------------------------------------
    def _obs(self, st, last_cmd):
        return build_obs(
            np.asarray(st["pos_m"]), np.asarray(st["quat_xyzw"]),
            np.asarray(st["v_body"]), np.asarray(st["w_body"]),
            self._task["goal_p"], self._task["goal_q"],
            vertical_ref_world=(0.0, 0.0, 1.0),
            last_cmd_norm=last_cmd)

    def _errs(self, st):
        p = np.asarray(st["pos_m"])
        q = np.asarray(st["quat_xyzw"])
        e_p = float(np.linalg.norm(
            quat_to_matrix(q).T @ (self._task["goal_p"] - p)))
        e_R = float(np.linalg.norm(
            attitude_error_rotvec(q, self._task["goal_q"])))
        return e_p, e_R

    # ------------------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        if self._spawn_p is None:
            self._spawn_p = self.sim.spawn_p.copy()
            self._spawn_q = self.sim.spawn_q.copy()
        ok = self.sim.reset_robot(self._spawn_p, self._spawn_q)
        if not ok:
            self.log("[env] reset_robot 核验未通过(仍继续,记录在 info)")
        self._task = self.sampler.sample(
            self._rng, self._spawn_p, self._spawn_q, self.stage)
        if self.disturb_cfg:
            self.sim.disturb_configure(self.disturb_cfg)
        else:
            self.sim.disturb_stop()
        self.sim.reset_output()
        self.sim.step_for(self.control_dt)       # 建立初始观测
        st = self.sim.state()
        self._t0 = float(st["sim_time_s"])
        self._prev_a = np.zeros(8)               # 复位后无已执行命令
        self._prev_dist = None
        obs = self._obs(st, last_cmd=np.zeros(8))
        self._last_info = {"task": dict(self._task), "reset_ok": bool(ok),
                           "sim_t0": self._t0}
        return obs, dict(self._last_info)

    def step(self, action):
        a_prev = self._prev_a.copy()
        a = np.clip(np.asarray(action, dtype=np.float64).ravel(), -1.0, 1.0)
        self.sim.set_command(a * FMAX_N)
        self.sim.step_for(self.control_dt)
        st = self.sim.state()
        obs = self._obs(st, last_cmd=a)          # 下一决策观测:含刚执行的 a
        self._prev_a = a.copy()

        e_p, e_R = self._errs(st)
        v = np.asarray(st["v_body"])
        w = np.asarray(st["w_body"])
        speed = float(np.linalg.norm(v)) + float(np.linalg.norm(w))
        t = float(st["sim_time_s"]) - self._t0

        r = 0.0
        r -= self.rw["pos_k"] * e_p
        r -= self.rw["ang_k"] * e_R
        if self._prev_dist is not None:
            r += self.rw["progress_k"] * (self._prev_dist - e_p)
        self._prev_dist = e_p
        in_pos = e_p < self.rw["pos_tol_m"]
        in_ang = e_R < self.rw["ang_tol_rad"]
        if in_pos and in_ang:
            r += self.rw["hold_bonus"]
            r -= self.rw["vel_k"] * speed
        r -= self.rw["action_k"] * float(np.mean(a * a))
        r -= self.rw["action_change_k"] * float(np.mean((a - a_prev) ** 2))

        p_world = np.asarray(st["pos_m"])
        oob = bool(np.linalg.norm(p_world - self._spawn_p) > self.oob_radius_m)
        if self.z_bounds is not None:
            oob = oob or not (self.z_bounds[0] <= p_world[2] <= self.z_bounds[1])
        bad_num = bool(np.any(~np.isfinite(obs)) or
                       np.any(np.abs(v) > 50.0) or np.any(np.abs(w) > 50.0))
        terminated = oob or bad_num
        truncated = bool(t >= self.time_limit_s) and not terminated
        if terminated:
            r += self.rw["oob_penalty"]

        info = {"pos_err_m": e_p, "ang_err_rad": e_R, "t_s": t,
                "in_pos": in_pos, "in_ang": in_ang,
                "success": bool(in_pos and in_ang and speed < 0.25),
                "oob": oob, "bad_num": bad_num,
                "sim_t": float(st["sim_time_s"])}
        self._last_info = info
        return obs, float(r), terminated, truncated, info

    def close(self):
        pass
