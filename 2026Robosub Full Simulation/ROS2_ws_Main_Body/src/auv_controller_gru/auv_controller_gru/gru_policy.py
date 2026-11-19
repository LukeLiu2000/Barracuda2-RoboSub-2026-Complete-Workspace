# -*- coding: utf-8 -*-
"""gru_policy —— GRU 策略网络(torch 训练版 + numpy 推理版,逐点一致)。

结构(任务书初始配置,如实实现):
  encoder: Linear(23, 64) + SiLU
  时序:   1 层 GRU(hidden 64)
  拼接:   [GRU 输出 64, 当前归一化输入 23] = 87
  动作头: Linear(87,64)+SiLU -> Linear(64,64)+SiLU -> Beta 头(α,β 各 8)
  价值头: Linear(87,64)+SiLU -> Linear(64,64)+SiLU -> Linear(64,1)
    (PPO 需要价值头;任务书未规定 critic,采用与动作头同宽的 87 输入
     MLP,此为本文档记录的实现选择)
动作分布:Beta(α,β) 于 [0,1],映射 a = 2x - 1 ∈ [-1,1],
  log π(a) = log π(x) - log 2 —— 有界支撑,概率密度与裁剪完全匹配,
  不存在"高斯采样后裁剪"的分布失配。
确定性推理(部署):x = α/(α+β) -> a = 2x-1。

numpy 推理(GruPolicyNumpy):从 torch 导出的 npz 权重做单步前向,
torch.GRU 权重布局(weight_ih_l0 = [r;z;n] 三段堆叠,输入先算 gi,
隐状态 gh,门控 r,z,n)精确复刻;单测对比 torch 前向误差 < 1e-5。
"""

import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    _HAS_TORCH = True
except Exception:                                        # noqa: BLE001
    _HAS_TORCH = False


# ==========================================================================
# torch 版(训练/导出)
# ==========================================================================
if _HAS_TORCH:

    class GruActorCriticTorch(torch.nn.Module):
        def __init__(self, obs_dim=23, act_dim=8, hidden=64):
            super().__init__()
            self.obs_dim, self.act_dim, self.hidden = obs_dim, act_dim, hidden
            self.encoder = nn.Sequential(nn.Linear(obs_dim, hidden), nn.SiLU())
            self.gru = nn.GRU(hidden, hidden, num_layers=1, batch_first=False)
            trunk_in = hidden + obs_dim
            self.actor = nn.Sequential(
                nn.Linear(trunk_in, 64), nn.SiLU(),
                nn.Linear(64, 64), nn.SiLU())
            self.alpha_head = nn.Linear(64, act_dim)
            self.beta_head = nn.Linear(64, act_dim)
            self.critic = nn.Sequential(
                nn.Linear(trunk_in, 64), nn.SiLU(),
                nn.Linear(64, 64), nn.SiLU(),
                nn.Linear(64, 1))
            # Beta 头初始化:输出集中在 1 附近(x≈0.5 => a≈0,零推力起步)
            self.alpha_head.weight.data.mul_(0.01)
            self.alpha_head.bias.data.fill_(1.0)
            self.beta_head.weight.data.mul_(0.01)
            self.beta_head.bias.data.fill_(1.0)

        def forward(self, obs_seq, h0=None, masks=None):
            """obs_seq: (T, batch, obs_dim) 已归一化;masks: (T, batch,1)
            done=1 处复位隐状态。返回 alpha,beta,value,(T,batch,act/1),
            h_next (1,batch,hidden)。"""
            T, B, _ = obs_seq.shape
            if h0 is None:
                h0 = obs_seq.new_zeros(1, B, self.hidden)
            h = h0
            alphas, betas, values = [], [], []
            for t in range(T):
                if masks is not None:
                    m = masks[t].view(B, 1)
                    h = (1.0 - m) * h + m * obs_seq.new_zeros(1, B, self.hidden)
                x = self.encoder(obs_seq[t])
                out, h = self.gru(x.unsqueeze(0), h)
                hh = out.squeeze(0)
                trunk = torch.cat([hh, obs_seq[t]], dim=-1)
                a = F.softplus(self.alpha_head(self.actor(trunk))) + 1e-3
                b = F.softplus(self.beta_head(self.actor(trunk))) + 1e-3
                alphas.append(a)
                betas.append(b)
                values.append(self.critic(trunk))
            return (torch.stack(alphas), torch.stack(betas),
                    torch.stack(values), h)

        @torch.no_grad()
        def deterministic_action(self, obs_norm, h):
            """单步确定性推理(部署路径)。obs_norm: (1,1,obs)。
            返回 a∈[-1,1] (1,1,act), h_next。"""
            alpha, beta, _, h = self.forward(obs_norm, h)
            x = alpha / (alpha + beta)
            return 2.0 * x - 1.0, h

        def export_npz(self, path):
            sd = {k: v.detach().cpu().numpy() for k, v in self.state_dict().items()}
            np.savez(path, **sd)
            return path


# ==========================================================================
# numpy 推理版(ROS 节点运行时,无 torch 依赖)
# ==========================================================================
def _silu(x):
    return x / (1.0 + np.exp(-x))


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


class GruPolicyNumpy(object):
    """单步 numpy 前向;权重来自 GruActorCriticTorch.export_npz。"""

    def __init__(self, npz_path):
        z = np.load(npz_path)
        w = {k: np.asarray(z[k], dtype=np.float64) for k in z.files}
        self.enc_w = w["encoder.0.weight"].T          # (23,64)
        self.enc_b = w["encoder.0.bias"]
        self.wih = w["gru.weight_ih_l0"]               # (192,64) [r;z;n]
        self.whh = w["gru.weight_hh_l0"]
        self.bih = w["gru.bias_ih_l0"]
        self.bhh = w["gru.bias_hh_l0"]
        self.hidden = self.whh.shape[1]
        self.a1_w = w["actor.0.weight"].T              # (87,64)
        self.a1_b = w["actor.0.bias"]
        self.a2_w = w["actor.2.weight"].T
        self.a2_b = w["actor.2.bias"]
        self.al_w = w["alpha_head.weight"].T           # (64,8)
        self.al_b = w["alpha_head.bias"]
        self.be_w = w["beta_head.weight"].T
        self.be_b = w["beta_head.bias"]

    def reset_hidden(self):
        return np.zeros((1, 1, self.hidden), dtype=np.float64)

    def step(self, obs_norm, h):
        """obs_norm: (obs_dim,) 已归一化观测;h: (1,1,hidden)。
        返回 (action (8,) ∈[-1,1], h_next)。torch GRU 精确复刻。"""
        x = _silu(self.enc_w.T @ obs_norm + self.enc_b)         # (64,)
        hh = h.reshape(self.hidden)
        gi = self.wih @ x + self.bih                              # (192,)
        gh = self.whh @ hh + self.bhh
        r = _sigmoid(gi[0:64] + gh[0:64])
        z = _sigmoid(gi[64:128] + gh[64:128])
        n = np.tanh(gi[128:] + r * gh[128:])
        h_new = (1.0 - z) * n + z * hh
        trunk = np.concatenate([h_new, obs_norm])                 # (87,)
        a = _silu(self.a1_w.T @ trunk + self.a1_b)
        a = _silu(self.a2_w.T @ a + self.a2_b)

        def _softplus(v):
            return np.log1p(np.exp(-np.abs(v))) + np.maximum(v, 0.0)

        alpha = _softplus(self.al_w.T @ a + self.al_b) + 1e-3
        beta = _softplus(self.be_w.T @ a + self.be_b) + 1e-3
        x_mean = alpha / (alpha + beta)
        action = 2.0 * x_mean - 1.0
        return action, h_new.reshape(1, 1, self.hidden)
