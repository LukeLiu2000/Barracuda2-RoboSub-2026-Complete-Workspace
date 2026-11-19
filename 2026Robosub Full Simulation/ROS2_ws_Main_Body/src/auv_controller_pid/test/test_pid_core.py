# -*- coding: utf-8 -*-
"""PID 核心与分配单元测试(纯 numpy,无 ROS 依赖)。"""

import math
import os

import numpy as np
import pytest

from auv_common.alloc import ThrusterGeometry, cross_check_with_scipy
from auv_common.frames import (attitude_error_rotvec, quat_about_axis,
                               quat_normalize, rotvec_from_R)
from auv_controller_pid.pid_core import PidCore

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, "..", ".."))


@pytest.fixture(scope="module")
def geom():
    return ThrusterGeometry.load_default()


def test_geometry_rank_cond(geom):
    assert geom.rank == 6
    assert geom.cond < 50.0
    # 水平 45° 对角 + 竖直四推进器的结构自检:t4..t7 方向为 +y
    assert np.allclose(geom.directions[4:8, 1], 1.0)
    assert np.allclose(geom.directions[4:8, 0], 0.0, atol=1e-9)


def test_config_consistency_with_mppi(geom):
    """auv_common 与 auv_controller_MPPI 两份 thrusters.yaml 数值必须一致。"""
    mppi_yaml = os.path.join(WS, "auv_controller_MPPI", "config", "thrusters.yaml")
    if not os.path.isfile(mppi_yaml):
        pytest.skip("MPPI 配置不存在(独立运行)")
    gm = ThrusterGeometry.from_ros_yaml(mppi_yaml)
    assert np.allclose(geom.positions, gm.positions, atol=1e-9)
    assert np.allclose(geom.directions, gm.directions, atol=1e-9)
    assert geom.force_min == gm.force_min
    assert geom.force_max == gm.force_max


# ---------------------------------------------------------------- 六轴符号
def _core(kp=100.0, kd=0.0):
    return PidCore(kp=(kp,) * 6, ki=(0.0,) * 6, kd=(kd,) * 6,
                   integral_limit=(1e3,) * 6, d_filter_hz=0.0)


def test_translation_sign_xyz(geom):
    for ax in range(3):
        core = _core()
        p_d = np.zeros(3)
        p_d[ax] = 0.5
        tau = core.update(np.zeros(3), (0, 0, 0, 1), np.zeros(3), np.zeros(3),
                          p_d, (0, 0, 0, 1), dt=0.02)
        assert tau[ax] > 0.0, "体 +%s 位置误差应产生 +F%s" % ("xyz"[ax], "xyz"[ax])
        alloc = geom.allocate_bounded(tau)
        # 分配后该轴合力方向仍为正
        assert alloc["tau_achieved"][ax] > 0.0


def test_rotation_sign_xyz(geom):
    for ax, axis in enumerate(([1, 0, 0], [0, 1, 0], [0, 0, 1])):
        core = _core(kp=5.0)
        q_d = quat_about_axis(axis, math.radians(20))
        tau = core.update(np.zeros(3), (0, 0, 0, 1), np.zeros(3), np.zeros(3),
                          np.zeros(3), q_d, dt=0.02)
        assert tau[3 + ax] > 0.0, "绕体 %s 正向目标姿态误差应产生 +M%s" % (
            "xyz"[ax], "xyz"[ax])


def test_single_thruster_directions_table(geom):
    """逐推进器单推的合力应等于其配置方向(方向-力一致性)。"""
    for i, entry in enumerate(geom.check_single_thrust_response()):
        assert np.allclose(entry["force_body"], geom.directions[i], atol=1e-6)


# ------------------------------------------------------- 坐标旋转导数项
def test_position_error_derivative_rotation_term():
    """体系误差导数必须含 -w×e 项:纯旋转(w≠0, e≠0)时该项非零。"""
    core = _core(kp=0.0, kd=10.0)
    w = np.array([0.0, 0.0, 0.5])
    p_d = np.array([0.6, 0.0, 0.0])
    # 体系 +x 误差固定,机体绕 z 转:de_b 应包含 -w×e_b = (0, -0.3, 0)
    core.update(np.zeros(3), (0, 0, 0, 1), np.zeros(3), w, p_d,
                (0, 0, 0, 1), dt=0.02)
    de = np.array(core.last_diag["de_pos_body_exact"])
    expect = np.cross(-w, core._prev_e_b)
    assert np.allclose(de, expect, atol=1e-9)
    assert abs(de[1] + 0.3) < 1e-9 or abs(de[1]) < 1e-9  # -w×e 的 y 分量


def test_world_position_error_into_body():
    """世界系 +Z 误差,机体横滚 90°(体 +y 对世界 +Z)时,体误差应在 +y。"""
    core = _core(kp=1.0)
    q = quat_about_axis([1, 0, 0], math.radians(90))   # 绕体 x 转 90°
    # R: 体 y -> 世界 z
    tau = core.update(np.zeros(3), q, np.zeros(3), np.zeros(3),
                      np.array([0.0, 0.0, 0.5]), (0, 0, 0, 1), dt=0.02)
    e_b = np.array(core.last_diag["e_pos_body"])
    assert e_b[1] > 0.4 and abs(e_b[0]) < 1e-6 and abs(e_b[2]) < 1e-6


# ------------------------------------------------------------- 抗饱和
def test_integral_limit_and_conditional():
    core = PidCore(kp=(1e-6,) * 6, ki=(10.0,) * 6, kd=(0.0,) * 6,
                   integral_limit=(1.0,) * 6,
                   output_limit=(0.1,) * 6, d_filter_hz=0.0)
    p_d = np.array([1.0, 0, 0])
    for _ in range(300):
        core.update(np.zeros(3), (0, 0, 0, 1), np.zeros(3), np.zeros(3),
                    p_d, (0, 0, 0, 1), dt=0.02)
    assert np.max(np.abs(core.integral)) <= 1.0 + 1e-9


def test_alloc_saturation_backcalc():
    core = PidCore(kp=(1e6,) * 6, ki=(5.0,) * 6, kd=(0.0,) * 6,
                   integral_limit=(50.0,) * 6, d_filter_hz=0.0)
    p_d = np.array([5.0, 0, 0])
    sat = {"n_saturated": 8, "residual_weighted": 100.0}
    for _ in range(50):
        core.update(np.zeros(3), (0, 0, 0, 1), np.zeros(3), np.zeros(3),
                    p_d, (0, 0, 0, 1), dt=0.02, alloc_result=sat)
    assert core.n_alloc_sat_feedback > 0
    assert np.max(np.abs(core.integral)) <= 50.0 + 1e-9


# ------------------------------------------------------------- 分配
def test_alloc_exact_when_feasible(geom):
    tau = np.array([3.0, -2.0, 5.0, 0.15, -0.1, 0.08])
    r = geom.allocate_bounded(tau)
    assert np.allclose(r["tau_achieved"], tau, atol=2e-3)
    assert r["n_saturated"] == 0
    ref = cross_check_with_scipy(geom, tau)
    if ref is not None:
        assert np.allclose(r["f"], ref, atol=0.5)


def test_alloc_saturated_honest(geom):
    tau = np.array([500.0, 0, 0, 0, 0, 0])       # 不可达(>8×39N 分量上限)
    r = geom.allocate_bounded(tau)
    assert np.all(r["f"] <= geom.force_max + 1e-9)
    assert np.all(r["f"] >= geom.force_min - 1e-9)
    assert r["n_saturated"] > 0
    assert r["tau_achieved"][0] < 500.0


def test_achievable_limits(geom):
    lim = PidCore.achievable_output_limit(geom)
    assert all(v > 1.0 for v in lim[:3])          # N
    assert all(v > 0.05 for v in lim[3:])         # N·m


# ------------------------------------------------------------- frames
def test_rotvec_roundtrip():
    for axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 2, 3]):
        for ang in (0.01, 0.5, 1.5, 3.0):
            q = quat_about_axis(axis, ang)
            e = attitude_error_rotvec((0, 0, 0, 1), q)
            assert abs(np.linalg.norm(e) - ang) < 1e-6


def test_rotvec_symmetry():
    q = quat_about_axis([0, 0, 1], 0.3)
    e1 = attitude_error_rotvec((0, 0, 0, 1), q)
    e2 = attitude_error_rotvec(q, (0, 0, 0, 1))
    assert np.allclose(e1, -e2, atol=1e-9)
