# -*- coding: utf-8 -*-
"""GRU 包单元测试(不依赖 Isaac/ROS,可系统 python 跑;torch 项自动跳过)。

覆盖:观测 23 维顺序与坐标变换、动作有界映射、numpy/torch 前向一致、
隐状态 masks 复位语义、GAE 手算对照、任务采样边界与可复现。
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from auv_common.frames import quat_about_axis, quat_mul, quat_normalize  # noqa: E402
from auv_controller_gru.gru_policy import _HAS_TORCH, GruPolicyNumpy       # noqa: E402
from auv_controller_gru.obs_builder import OBS_LAYOUT, OBS_DIM, build_obs  # noqa: E402
from auv_controller_gru.task_gen import FMAX_N, TaskSampler, rand_ball     # noqa: E402
from auv_controller_gru.train.gru_ppo import compute_gae                   # noqa: E402


def _ident():
    return np.array([0.0, 0.0, 0.0, 1.0])


# ---------------------------------------------------------------- 观测顺序
def test_obs_layout_order_identity():
    p = np.zeros(3)
    q = _ident()
    p_d = np.array([1.0, -2.0, 3.0])
    v = np.array([0.4, 0.5, 0.6])
    w = np.array([0.1, 0.2, 0.3])
    cmd = np.linspace(-1.0, 0.7, 8)
    obs = build_obs(p, q, v, w, p_d, _ident(),
                    vertical_ref_world=(0.0, 0.0, 1.0), last_cmd_norm=cmd)
    assert obs.shape == (OBS_DIM,)
    assert np.allclose(obs[0:3], p_d)                 # R=I: 体误差=世界误差
    assert np.allclose(obs[3:6], 0.0)                 # 姿态无误差
    assert np.allclose(obs[6:9], v)
    assert np.allclose(obs[9:12], w)
    assert np.allclose(obs[12:15], [0.0, 0.0, 1.0])   # 世界上轴->体系=不变
    assert np.allclose(obs[15:23], cmd)


def test_obs_body_frame_transform_yaw90():
    """艇体绕 +y(偏航)90°:体 +z(前)转到世界 +x,故世界 +x 目标
    在体系为 +z(正前方)。e_body = R^T (p_d - p) = (0,0,1)。"""
    q = quat_about_axis(np.array([0.0, 1.0, 0.0]), np.pi / 2)
    obs = build_obs(np.zeros(3), q, np.zeros(3), np.zeros(3),
                    np.array([1.0, 0.0, 0.0]), _ident(),
                    last_cmd_norm=np.zeros(8))
    assert np.allclose(obs[0:3], [0.0, 0.0, 1.0], atol=1e-9)


def test_obs_attitude_error_matches_axis():
    """q_d = 绕体 z(roll)转 30°:e_rot 应为 [0,0,+30°]。"""
    ang = np.deg2rad(30.0)
    q_d = quat_about_axis(np.array([0.0, 0.0, 1.0]), ang)
    obs = build_obs(np.zeros(3), _ident(), np.zeros(3), np.zeros(3),
                    np.zeros(3), q_d, last_cmd_norm=np.zeros(8))
    assert np.allclose(obs[3:6], [0.0, 0.0, ang], atol=1e-6)


def test_obs_last_cmd_clipped_and_nonfinite_guard():
    obs = build_obs(np.zeros(3), _ident(), np.zeros(3), np.zeros(3),
                    np.zeros(3), _ident(),
                    last_cmd_norm=np.full(8, 5.0))
    assert np.allclose(obs[15:23], 1.0)
    bad = build_obs(np.zeros(3), _ident(),
                    np.array([np.nan, 0.0, 0.0]), np.zeros(3),
                    np.zeros(3), _ident())
    assert np.all(np.isfinite(bad))


# ---------------------------------------------------------------- 任务采样
def test_task_sampler_bounds_and_repro():
    rng = np.random.default_rng(42)
    s = TaskSampler()
    for _ in range(60):
        t = s.sample(rng, np.zeros(3), _ident(), stage=1)
        assert t["disp_m"] <= 0.4 + 1e-9
        assert abs(t["rot_deg"]) < 1e-9
        t2 = s.sample(rng, np.zeros(3), _ident(), stage=2)
        assert t2["disp_m"] <= 0.8 + 1e-9
        assert abs(t2["rot_deg"]) <= 30.0 + 1e-9
        t3 = s.sample(rng, np.zeros(3), _ident(), stage=3)
        assert t3["disp_m"] <= 1.2 + 1e-9
        assert t3["rot_deg"] <= 60.0 + 1e-9
    a = s.sample(np.random.default_rng(7), np.zeros(3), _ident(), 3)
    b = s.sample(np.random.default_rng(7), np.zeros(3), _ident(), 3)
    assert np.allclose(a["goal_p"], b["goal_p"])
    assert np.allclose(a["goal_q"], b["goal_q"])


def test_rand_ball_uniform_bound():
    rng = np.random.default_rng(0)
    pts = np.array([rand_ball(rng, 0.5) for _ in range(300)])
    assert np.all(np.linalg.norm(pts, axis=1) <= 0.5 + 1e-9)
    # 方向覆盖:三轴均有正负样本
    assert np.all(pts.max(axis=0) > 0.05)
    assert np.all(pts.min(axis=0) < -0.05)
    assert FMAX_N == 39.2266


# ---------------------------------------------------------------- GAE
def test_gae_hand_computed():
    rewards = np.array([1.0, 0.0])
    values = np.array([0.5, 0.5])
    dones = np.array([0.0, 1.0])
    adv, ret = compute_gae(rewards, values, dones, last_value=0.0,
                           gamma=1.0, lam=1.0)
    assert np.allclose(adv, [0.5, -0.5], atol=1e-12)
    assert np.allclose(ret, [1.0, 0.0], atol=1e-12)


def test_gae_dones_cut_bootstrap():
    rewards = np.array([0.0, 1.0])
    values = np.array([0.0, 0.0])
    dones = np.array([1.0, 0.0])
    adv, _ = compute_gae(rewards, values, dones, last_value=99.0,
                         gamma=0.9, lam=0.8)
    # t=0 回合结束:其 GAE 与后续无关;delta0=0 -> adv0=0
    assert abs(adv[0]) < 1e-12
    assert adv[1] == pytest.approx(1.0 + 0.9 * 99.0 - 0.0)


# ---------------------------------------------------------------- torch 项
@pytest.mark.skipif(not _HAS_TORCH, reason="torch 不在当前解释器")
def test_torch_numpy_forward_match(tmp_path):
    """numpy 单步前向与 torch 逐点一致(< 1e-5),覆盖隐状态传播。"""
    import torch
    from auv_controller_gru.gru_policy import GruActorCriticTorch
    torch.manual_seed(3)
    pol = GruActorCriticTorch()
    npz = str(tmp_path / "w.npz")
    pol.export_npz(npz)
    npn = GruPolicyNumpy(npz)

    rng = np.random.default_rng(11)
    h_np = npn.reset_hidden()
    h_th = torch.zeros(1, 1, 64)
    pol.eval()
    for _ in range(12):                      # 连续 12 步隐状态传播
        obs = rng.normal(size=23)
        with torch.no_grad():
            a_th, h_th = pol.deterministic_action(
                torch.as_tensor(obs, dtype=torch.float32).view(1, 1, 23), h_th)
        a_np, h_np = npn.step(obs, h_np)
        assert np.max(np.abs(a_th.numpy().reshape(-1) - a_np)) < 1e-5
        assert np.max(np.abs(h_th.numpy() - h_np)) < 1e-5
        assert np.all(np.abs(a_np) <= 1.0)   # 动作天然有界


@pytest.mark.skipif(not _HAS_TORCH, reason="torch 不在当前解释器")
def test_hidden_reset_via_masks():
    """masks=1 处隐状态清零:前两步等价于单独跑第二步(h0=0)。"""
    import torch
    from auv_controller_gru.gru_policy import GruActorCriticTorch
    torch.manual_seed(5)
    pol = GruActorCriticTorch().eval()
    obs = torch.randn(3, 1, 23)
    masks = torch.zeros(3, 1, 1)
    masks[1] = 1.0                       # 第 1 步前复位
    with torch.no_grad():
        a1, b1, _, h1 = pol(obs, None, masks)
        a2, b2, _, h2 = pol(obs[1:].contiguous(),
                            torch.zeros(1, 1, 64))
    assert torch.max(torch.abs(a1[1:] - a2)) < 1e-6
    assert torch.max(torch.abs(b1[1:] - b2)) < 1e-6
    assert torch.max(torch.abs(h1 - h2)) < 1e-6


@pytest.mark.skipif(not _HAS_TORCH, reason="torch 不在当前解释器")
def test_beta_action_logp_mapping():
    """a=2x-1 映射的 log 概率含 -8·log2 雅可比,数值自洽。"""
    import torch
    from torch.distributions import Beta
    alpha = torch.full((8,), 2.0)
    beta = torch.full((8,), 3.0)
    dist = Beta(alpha, beta)
    x = torch.full((8,), 0.25)
    logp_x = dist.log_prob(x).sum().item()
    assert np.isfinite(logp_x)               # Beta log_prob 在支撑内有限
    # 校验 agent 侧公式与分布定义一致
    from auv_controller_gru.train.gru_ppo import RecurrentPPO
    from auv_controller_gru.gru_policy import GruActorCriticTorch
    from auv_controller_gru.obs_builder import ObsNormalizer
    torch.manual_seed(1)
    ag = RecurrentPPO(env=None, policy=GruActorCriticTorch().eval(),
                      normalizer=ObsNormalizer(),
                      cfg={"n_steps": 8}, device="cpu", log=lambda *_: None)
    obs = np.zeros(23)
    h = np.zeros((1, 1, 64))
    x, logp, _v, _h = ag._act(obs, h)
    assert np.all((x > 0.0) & (x < 1.0))
    with torch.no_grad():
        o = torch.as_tensor(obs, dtype=torch.float32).view(1, 1, 23)
        al, be, _, _ = ag.policy(o, torch.as_tensor(h, dtype=torch.float32))
        d = Beta(al.view(-1), be.view(-1))
        ref = (d.log_prob(torch.as_tensor(x)).sum()
               - 8.0 * torch.log(torch.tensor(2.0))).item()
    assert abs(logp - ref) < 1e-6


# ---------------------------------------------------------------- PPO 更新
@pytest.mark.skipif(not _HAS_TORCH, reason="torch 不在当前解释器")
def test_ppo_update_chunk_shapes():
    """update() 的 Beta log_prob/ratio/KL 形状回归(2026-11-08 两次实测
    crash:4096 元素 view 成 [64,1] 失败)。

    根因:批形状 (L,8) 的 dist 对 (L,1,8) 的值 log_prob 会右对齐广播成
    (L,L,8);x_old 必须 view(L,-1) 与批形状一致,logp_old 必须 (L,1),
    否则 ratio 直接崩、KL 退化成 (L,L) 两两差矩阵。
    """
    import torch
    from torch.distributions import Beta

    # 1) 广播数学根因自检
    d = Beta(torch.rand(64, 8) + 0.5, torch.rand(64, 8) + 0.5)
    assert tuple(d.log_prob(torch.rand(64, 1, 8)).sum(-1).shape) == (64, 64)
    assert tuple(d.log_prob(torch.rand(64, 8)).sum(-1).shape) == (64,)

    # 2) 真实 update() 代码路径(缓冲填充 -> GAE -> chunk 采样 -> 更新)
    from auv_controller_gru.gru_policy import GruActorCriticTorch
    from auv_controller_gru.train.gru_ppo import RecurrentPPO

    torch.manual_seed(0)
    np.random.seed(0)

    class _N(object):
        def normalize(self, x):
            return x

    agent = RecurrentPPO(
        None, GruActorCriticTorch(), _N(),
        {"n_steps": 128, "epochs": 1, "chunk_len": 64, "batch_chunks": 4,
         "target_kl": 1e9}, device="cpu", log=lambda *_: None)
    rng = np.random.default_rng(0)
    h = np.zeros((1, 1, 64))
    for i in range(agent.buf.n):
        if i % 60 == 0:                       # 跨回合:隐状态周期性复位
            h = np.zeros((1, 1, 64))
        agent.buf.add(rng.normal(size=23), rng.uniform(0.05, 0.95, 8),
                      -5.5, 0.0, rng.normal(),
                      1.0 if (i + 1) % 60 == 0 else 0.0, h.copy())
        h = h + rng.normal(scale=0.01, size=h.shape)
    stats = agent.update(0.0)
    assert all(np.isfinite(v) for v in stats.values()), stats


# ---------------------------------------------------------------- 布局文档
def test_obs_layout_documented():
    assert OBS_DIM == 23
    assert OBS_LAYOUT[0][0] == 0 and OBS_LAYOUT[-1][1] == 23
