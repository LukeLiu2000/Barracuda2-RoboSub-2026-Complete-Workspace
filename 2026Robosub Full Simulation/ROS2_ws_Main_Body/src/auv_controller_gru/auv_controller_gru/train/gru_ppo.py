# -*- coding: utf-8 -*-
"""gru_ppo —— 自定义循环 PPO(recurrent PPO)核心。

为什么不用 sb3-contrib RecurrentPPO:其高斯+裁剪动作与任务书
"不要直接裁剪随机动作却使用不匹配的概率密度训练"冲突;这里动作
分布为 Beta(α,β) 于 [0,1],映射 a=2x-1∈[-1,1],log π(a) =
log π(x) - 8·log2,有界支撑与限幅完全一致。

实现要点(均有真实代码路径,非占位):
  - rollout:单环境顺序采集;存储每步【动作前】隐状态 h_pre;
    回合结束(done)时 h 复位为零 —— 隐状态生命周期与部署约定一致;
  - GAE(λ) 回报估计,bootstrap 末值;
  - 更新:定长 chunk(默认 64)滑窗采样,chunk 内 done 边界经 masks
    复位隐状态(h_init=chunk 起点真实 h_pre),跨回合 chunk 合法;
  - PPO clip 目标 + 价值 clip 损失 + Beta 熵正则 + 梯度范数裁剪;
  - approx_kl 监控,target_kl 早停(单 epoch 内);
  - 检查点保存/恢复(policy+optimizer+normalizer+进度)。
"""

import numpy as np

try:
    import torch
    from torch.distributions import Beta
    _HAS_TORCH = True
except Exception:                                        # noqa: BLE001
    _HAS_TORCH = False

_EPS_X = 1e-5        # Beta 支撑开区间保护(采样后夹到 (eps, 1-eps))


class RolloutBuffer(object):
    """单环境 rollout 存储(numpy,n_steps 固定)。"""

    def __init__(self, n_steps, obs_dim, act_dim, hidden):
        n = int(n_steps)
        self.obs = np.zeros((n, obs_dim), dtype=np.float64)
        self.x = np.zeros((n, act_dim), dtype=np.float64)      # Beta 空间动作
        self.logp = np.zeros(n, dtype=np.float64)
        self.value = np.zeros(n, dtype=np.float64)
        self.reward = np.zeros(n, dtype=np.float64)
        self.done = np.zeros(n, dtype=np.float64)              # term or trunc
        self.h_pre = np.zeros((n, 1, hidden), dtype=np.float64)
        self.ptr = 0
        self.n = n

    def add(self, obs, x, logp, value, reward, done, h_pre):
        i = self.ptr
        self.obs[i] = obs
        self.x[i] = x
        self.logp[i] = logp
        self.value[i] = value
        self.reward[i] = reward
        self.done[i] = float(done)
        self.h_pre[i] = h_pre
        self.ptr += 1


def compute_gae(rewards, values, dones, last_value, gamma, lam):
    """GAE(λ);dones[t]=1 表示 t 步后回合结束(下一状态为 reset 态)。"""
    n = len(rewards)
    adv = np.zeros(n, dtype=np.float64)
    lastgaelam = 0.0
    for t in reversed(range(n)):
        next_non_terminal = 1.0 - dones[t]
        next_v = last_value if t == n - 1 else values[t + 1]
        delta = rewards[t] + gamma * next_v * next_non_terminal - values[t]
        lastgaelam = delta + gamma * lam * next_non_terminal * lastgaelam
        adv[t] = lastgaelam
    return adv, adv + values


class RecurrentPPO(object):

    def __init__(self, env, policy, normalizer, cfg, device="cpu",
                 log=print, obs_dim=23, act_dim=8, hidden=64):
        if not _HAS_TORCH:
            raise RuntimeError("训练需要 torch(Isaac python 自带)")
        self.env = env
        self.policy = policy.to(device)
        self.norm = normalizer
        self.cfg = dict(cfg)
        self.device = device
        self.log = log
        self.obs_dim, self.act_dim, self.hidden = obs_dim, act_dim, hidden
        self.optimizer = torch.optim.Adam(
            policy.parameters(), lr=float(cfg.get("lr", 3e-4)))
        self.buf = RolloutBuffer(cfg.get("n_steps", 600),
                                 obs_dim, act_dim, hidden)
        self._obs = None
        self._h = None
        self._last_ep = None
        self.num_steps = 0
        self.ep_stats = []

    # ------------------------------------------------------------------
    def _act(self, obs_np, h, deterministic=False):
        """单步前向;返回 x(Beta 空间), logp(动作空间), value, h_next。"""
        o = torch.as_tensor(obs_np, dtype=torch.float32,
                            device=self.device).view(1, 1, self.obs_dim)
        h0 = torch.as_tensor(h, dtype=torch.float32, device=self.device)
        alpha, beta, v, h1 = self.policy(o, h0)
        alpha = alpha.view(-1)
        beta = beta.view(-1)
        dist = Beta(alpha, beta)
        if deterministic:
            x = dist.mean
        else:
            x = dist.sample()
        x = torch.clamp(x, _EPS_X, 1.0 - _EPS_X)
        logp_x = dist.log_prob(x).sum()
        logp_a = (logp_x - float(self.act_dim * np.log(2.0))).item()
        return (x.detach().cpu().numpy().reshape(-1),
                float(logp_a), float(v.view(-1)[0]),
                h1.detach().cpu().numpy())

    def set_env_state(self, obs, h):
        """从外部恢复交互状态(恢复检查点/评测切换)。"""
        self._obs = np.asarray(obs, dtype=np.float64).copy()
        self._h = np.asarray(h, dtype=np.float64).copy()

    # ------------------------------------------------------------------
    def collect_rollout(self):
        """采集 n_steps;返回末状态 bootstrap 值与回合统计。"""
        cfg = self.cfg
        n = self.buf.n
        self.buf.ptr = 0
        if self._obs is None:
            obs, info = self.env.reset()
            self._h = np.zeros((1, 1, self.hidden), dtype=np.float64)
            self._obs = np.asarray(obs, dtype=np.float64)
        ep_r, ep_len, ep_succ = 0.0, 0, False
        for _t in range(n):
            obs_n = self.norm.normalize(self._obs)
            h_pre = self._h.copy()
            x, logp, value, h_next = self._act(obs_n, self._h)
            action = 2.0 * x - 1.0
            obs2, r, term, trunc, info = self.env.step(action)
            done = bool(term or trunc)
            self.buf.add(self._obs.copy(), x, logp, value, r, done, h_pre)
            ep_r += float(r)
            ep_len += 1
            ep_succ = ep_succ or bool(info.get("success", False))
            if done:
                self.ep_stats.append({"reward": ep_r, "len": ep_len,
                                      "success": ep_succ,
                                      "pos_err": float(info.get("pos_err_m",
                                                                np.nan)),
                                      "step": self.num_steps + _t})
                ep_r, ep_len, ep_succ = 0.0, 0, False
                obs2, _ = self.env.reset()
                h_next = np.zeros((1, 1, self.hidden), dtype=np.float64)
            self._obs = np.asarray(obs2, dtype=np.float64)
            self._h = h_next
        self.num_steps += n
        # bootstrap
        with torch.no_grad():
            obs_n = self.norm.normalize(self._obs)
            _, _, v_last, _ = self._act(obs_n, self._h, deterministic=True)
        return float(v_last)

    # ------------------------------------------------------------------
    def update(self, last_value):
        cfg = self.cfg
        gamma = float(cfg.get("gamma", 0.99))
        lam = float(cfg.get("gae_lambda", 0.95))
        clip = float(cfg.get("clip_eps", 0.2))
        ent_c = float(cfg.get("entropy_coef", 0.01))
        vf_c = float(cfg.get("value_coef", 0.5))
        epochs = int(cfg.get("epochs", 6))
        chunk = int(cfg.get("chunk_len", 64))
        n_chunks = int(cfg.get("batch_chunks", 8))
        max_grad = float(cfg.get("max_grad_norm", 0.5))
        target_kl = float(cfg.get("target_kl", 0.03))

        adv, ret = compute_gae(self.buf.reward, self.buf.value, self.buf.done,
                               last_value, gamma, lam)
        adv = (adv - adv.mean()) / max(adv.std(), 1e-8)

        N = self.buf.n
        n_starts = max(N - chunk, 1)
        stats = {"pi_loss": [], "v_loss": [], "ent": [], "kl": [],
                 "clip_frac": []}
        for _ep in range(epochs):
            perm = np.random.randint(0, n_starts, size=n_chunks * 2)
            taken = 0
            for s in perm:
                if taken >= n_chunks:
                    break
                e = min(s + chunk, N)
                L = e - s
                if L < 2:
                    continue
                taken += 1
                obs_t = torch.as_tensor(
                    self.norm.normalize(self.buf.obs[s:e]),
                    dtype=torch.float32, device=self.device
                ).view(L, 1, self.obs_dim)
                # x_old 必须与 dist 的批形状 (L, act_dim) 完全一致:
                # 若留 (L,1,act_dim),log_prob 会把批维 (L,8) 与值维
                # (L,1,8) 右对齐广播成 (L,L,8),sum(-1) 后得 (L,L)=4096
                # 元素(2026-11-08 两次实测 crash: view [64,1] 失败)。
                x_old = torch.as_tensor(self.buf.x[s:e], dtype=torch.float32,
                                        device=self.device).view(L, -1)
                logp_old = torch.as_tensor(self.buf.logp[s:e],
                                           dtype=torch.float32,
                                           device=self.device).view(L, 1)
                adv_t = torch.as_tensor(adv[s:e], dtype=torch.float32,
                                        device=self.device).view(L, 1)
                ret_t = torch.as_tensor(ret[s:e], dtype=torch.float32,
                                        device=self.device).view(L, 1)
                v_old = torch.as_tensor(self.buf.value[s:e],
                                        dtype=torch.float32,
                                        device=self.device).view(L, 1)
                # masks: chunk 起点 h_init 即真实 h_pre,无需复位;
                # chunk 内 done 边界处复位
                masks = np.zeros((L, 1, 1), dtype=np.float32)
                masks[1:, 0, 0] = self.buf.done[s:e - 1]
                masks_t = torch.as_tensor(masks, device=self.device)
                h0 = torch.as_tensor(self.buf.h_pre[s], dtype=torch.float32,
                                     device=self.device)

                alpha, beta, v, _ = self.policy(obs_t, h0, masks_t)
                a = alpha.view(L, -1)
                b = beta.view(L, -1)
                dist = Beta(a, b)
                logp_x = dist.log_prob(torch.clamp(x_old, _EPS_X,
                                                   1.0 - _EPS_X)).sum(-1)
                logp_new = logp_x - float(self.act_dim * np.log(2.0))
                ratio = torch.exp(logp_new.view(L, 1) - logp_old)
                surr1 = ratio * adv_t
                surr2 = torch.clamp(ratio, 1.0 - clip, 1.0 + clip) * adv_t
                pi_loss = -torch.min(surr1, surr2).mean()
                v_clipped = v_old + torch.clamp(v.view(L, 1) - v_old, -clip,
                                                clip)
                v_loss = 0.5 * torch.max(
                    (v.view(L, 1) - ret_t) ** 2,
                    (v_clipped - ret_t) ** 2).mean()
                ent = dist.entropy().sum(-1).mean()
                loss = pi_loss + vf_c * v_loss - ent_c * ent
                self.optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(self.policy.parameters(),
                                               max_grad)
                self.optimizer.step()
                with torch.no_grad():
                    # logp_old (L,1) 与 logp_new (L,) 直接相减会广播成
                    # (L,L) 的两两差矩阵,均值就是错误 KL —— 必须先对齐
                    kl = (logp_old - logp_new.view(L, 1)).mean().item()
                    clip_frac = ((ratio - 1.0).abs() > clip).float() \
                        .mean().item()
                stats["pi_loss"].append(pi_loss.item())
                stats["v_loss"].append(v_loss.item())
                stats["ent"].append(ent.item())
                stats["kl"].append(kl)
                stats["clip_frac"].append(clip_frac)
            mean_kl = float(np.mean(stats["kl"][-n_chunks:])) \
                if stats["kl"] else 0.0
            if target_kl > 0 and mean_kl > 1.5 * target_kl:
                self.log("[ppo] epoch 早停: approx_kl=%.4f > %.4f"
                         % (mean_kl, 1.5 * target_kl))
                break
        return {k: (float(np.mean(v)) if v else 0.0)
                for k, v in stats.items()}

    # ------------------------------------------------------------------
    def evaluate(self, env, n_episodes=5):
        """确定性策略评测(独立环境实例/种子由调用方给)。"""
        results = []
        h = np.zeros((1, 1, self.hidden), dtype=np.float64)
        for _i in range(int(n_episodes)):
            obs, _ = env.reset()
            h = np.zeros((1, 1, self.hidden), dtype=np.float64)
            done, total_r, steps = False, 0.0, 0
            final = {}
            while not done:
                x, _, _, h = self._act(self.norm.normalize(obs), h,
                                       deterministic=True)
                obs, r, term, trunc, info = env.step(2.0 * x - 1.0)
                total_r += float(r)
                steps += 1
                final = info
                done = bool(term or trunc)
                if steps > int(1.5 * self.buf.n):
                    break
            results.append({"reward": total_r, "len": steps,
                            "success": bool(final.get("success", False)),
                            "pos_err_m": float(final.get("pos_err_m", np.nan)),
                            "ang_err_rad": float(final.get("ang_err_rad",
                                                           np.nan))})
        return results

    # ------------------------------------------------------------------
    def save(self, path, extra=None):
        ck = {
            "policy": self.policy.state_dict(),
            "optimizer": self.optimizer.state_dict(),
            "normalizer": self.norm.to_dict(),
            "num_steps": self.num_steps,
            "cfg": self.cfg,
        }
        if extra:
            ck.update(extra)
        torch.save(ck, path)
        return path

    def load(self, path):
        ck = torch.load(path, map_location=self.device)
        self.policy.load_state_dict(ck["policy"])
        self.optimizer.load_state_dict(ck["optimizer"])
        self.norm = type(self.norm).from_dict(ck["normalizer"])
        self.num_steps = int(ck.get("num_steps", 0))
        self._obs = None
        self._h = None
        return ck
