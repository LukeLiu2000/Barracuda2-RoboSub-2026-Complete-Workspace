#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GoalManager 回归测试(纯逻辑,无 ROS 依赖,本机直接可跑):

    python3 test/test_goal_manager.py

覆盖:体轴方向数学(含偏航 90°/滚转 180° 复合)、比例×额定速率、
距离/姿态/工作空间三类限幅、状态过期冻结、松键保持、重捕获、首状态自动捕获。
"""
import math
import sys
import time

from auv_mission_control.goal_manager import (
    GoalManager, quat_about_axis, rotate_vec)

ok = True
def check(name, cond, detail=""):
    global ok
    print(("PASS" if cond else "FAIL"), name, detail)
    ok = ok and cond

BIG = dict(workspace_min=(-10, -10, -10), workspace_max=(10, 10, 10),
           max_goal_dist=5.0, max_goal_att_deg=45.0, state_timeout=100.0)
IDQ = (0.0, 0.0, 0.0, 1.0)


def t_surge():
    gm = GoalManager(surge_speed=0.30, sway_speed=0.25, heave_speed=0.25, **BIG)
    gm.set_state((0.0, 0.0, 0.0), IDQ, 0.0)
    gm.update(1.0, (0, 0, 1), 0.0)
    p, _ = gm.goal()
    check("W 沿 +z 前 0.30", abs(p[2] - 0.30) < 1e-6 and abs(p[0]) < 1e-6
          and abs(p[1]) < 1e-6, f"p={p}")


def t_sway():
    gm = GoalManager(sway_speed=0.25, **BIG)
    gm.set_state((0.0, 0.0, 0.0), IDQ, 0.0)
    gm.update(1.0, (1, 0, 0), 0.0)
    p, _ = gm.goal()
    check("A 沿 +x 左舷 0.25", abs(p[0] - 0.25) < 1e-6, f"p={p}")


def t_yaw90_surge():
    q = quat_about_axis((0.0, 1.0, 0.0), math.pi / 2)
    gm = GoalManager(surge_speed=0.30, **BIG)
    gm.set_state((0.0, 0.0, 0.0), q, 0.0)
    gm.update(1.0, (0, 0, 1), 0.0)
    p, _ = gm.goal()
    check("yaw90° 后 W 沿世界 +x", abs(p[0] - 0.30) < 1e-6 and abs(p[2]) < 1e-6,
          f"p={p}")


def t_body_up_yaw():
    """roll180 后体上轴=世界 -y;yaw+ 应绕体 +y(=世界 -y)转。"""
    q_roll180 = quat_about_axis((0.0, 0.0, 1.0), math.pi)
    gm = GoalManager(yaw_rate_deg_s=30.0, **BIG)
    gm.set_state((0.0, 0.0, 0.0), q_roll180, 0.0)
    gm.update(1.0, (0, 0, 0), +1.0)
    _, q4 = gm.goal()
    f = rotate_vec(q4, (0.0, 0.0, 1.0))
    check("体上轴偏航(roll180 后)",
          abs(f[0] + 0.5) < 1e-6 and abs(f[1]) < 1e-6
          and abs(f[2] - math.cos(math.radians(30))) < 1e-6, f"f={f}")


def t_dist_clamp():
    gm = GoalManager(surge_speed=0.30, **dict(BIG, max_goal_dist=0.5))
    gm.set_state((0.0, 0.0, 0.0), IDQ, 0.0)
    gm.update(10.0, (0, 0, 1), 0.0)
    p, _ = gm.goal()
    d = math.sqrt(sum(c * c for c in p))
    check("距离限幅 0.5", abs(d - 0.5) < 1e-6, f"d={d:.6f}")


def t_ws_clamp():
    gm = GoalManager(sway_speed=0.25, **BIG)
    gm.set_state((9.9, 0.0, 0.0), IDQ, 0.0)
    gm.update(1.0, (1, 0, 0), 0.0)
    p, _ = gm.goal()
    check("工作空间盒夹 x<=10", abs(p[0] - 10.0) < 1e-6, f"p={p}")


def t_att_clamp():
    gm = GoalManager(yaw_rate_deg_s=60.0, **BIG)
    gm.set_state((0.0, 0.0, 0.0), IDQ, 0.0)
    gm.update(1.0, (0, 0, 0), +1.0)
    _, q = gm.goal()
    ang = 2 * math.acos(max(-1.0, min(1.0, q[3])))
    check("姿态差限幅 45°", abs(math.degrees(ang) - 45.0) < 1e-3,
          f"deg={math.degrees(ang):.3f}")


def t_state_timeout():
    gm = GoalManager(**dict(BIG, state_timeout=0.3))
    gm.set_state((0.0, 0.0, 0.0), IDQ, 0.0)
    gm.update(0.1, (0, 0, 1), 0.0)
    for _ in range(20):            # 断流后持续 tick 1s,年龄超 0.3 冻结
        gm.update(0.05, (0, 0, 1), 0.0)
    p_mid, _ = gm.goal()
    for _ in range(20):            # 再 1s,应完全不动
        gm.update(0.05, (0, 0, 1), 0.0)
    p_after, _ = gm.goal()
    check("状态过期目标冻结", p_mid == p_after and not gm.state_valid,
          f"mid={p_mid} after={p_after}")


def t_release_freeze():
    gm = GoalManager(surge_speed=0.30, **BIG)
    gm.set_state((0.0, 0.0, 0.0), IDQ, 0.0)
    gm.update(1.0, (0, 0, 1), 0.0)
    pa, _ = gm.goal()
    gm.update(1.0, (0, 0, 0), 0.0)
    gm.update(1.0, (0, 0, 0), 0.0)
    pb, _ = gm.goal()
    check("松键目标保持", pa == pb, f"{pa} == {pb}")


def t_recapture():
    gm = GoalManager(**BIG)
    gm.set_state((1.0, 2.0, 3.0), IDQ, 0.0)
    gm.update(1.0, (0, 0, 1), 0.0)
    gm.recapture()
    p, q = gm.goal()
    check("recapture 回当前", p == (1.0, 2.0, 3.0) and q == IDQ, f"p={p}")


def t_auto_init():
    gm = GoalManager(**BIG)
    check("初始未初始化", not gm.goal_initialized)
    gm.set_state((1.0, 1.0, 1.0), IDQ, 0.0)
    p, _ = gm.goal()
    check("首状态自动捕获", gm.goal_initialized and p == (1.0, 1.0, 1.0))


if __name__ == "__main__":
    for fn in (t_surge, t_sway, t_yaw90_surge, t_body_up_yaw, t_dist_clamp,
               t_ws_clamp, t_att_clamp, t_state_timeout, t_release_freeze,
               t_recapture, t_auto_init):
        fn()
    print("\nALL PASS" if ok else "\nSOME FAILED")
    sys.exit(0 if ok else 1)
