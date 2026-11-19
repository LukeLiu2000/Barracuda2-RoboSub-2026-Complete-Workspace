#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""水平姿态键盘目标管理，不依赖 rclpy。

坐标约定：
    世界坐标：+Z 向上。
    艇体坐标：+X 左舷，+Y 向上，+Z 向前。
    四元数顺序：(x, y, z, w)。

控制规则：
    1. 目标姿态始终水平，只允许改变世界 Z 轴航向。
    2. 前后、左右沿实际航向的水平投影移动。
    3. 上下沿世界 Z 轴移动。
    4. 松键后目标位置、航向冻结。
    5. 重新捕获时保持当前位置，目标姿态改为水平。
    6. 保留位置限幅；姿态限幅改为航向差限幅。
    7. roll_cmd、pitch_cmd 保留接口，但不改变目标倾斜。
"""

import math
from typing import Optional, Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]


def quat_normalize(q: Quat) -> Quat:
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / n for v in q)


def quat_to_matrix(q: Quat):
    x, y, z, w = quat_normalize(q)
    return [
        [
            1 - 2 * (y*y + z*z),
            2 * (x*y - z*w),
            2 * (x*z + y*w),
        ],
        [
            2 * (x*y + z*w),
            1 - 2 * (x*x + z*z),
            2 * (y*z - x*w),
        ],
        [
            2 * (x*z - y*w),
            2 * (y*z + x*w),
            1 - 2 * (x*x + y*y),
        ],
    ]


def quat_mul(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


def quat_conj(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def quat_about_axis(axis: Vec3, angle_rad: float) -> Quat:
    n = math.sqrt(sum(v*v for v in axis))
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle_rad / 2.0) / n
    return quat_normalize((
        axis[0] * s,
        axis[1] * s,
        axis[2] * s,
        math.cos(angle_rad / 2.0),
    ))


def quat_angle_between(a: Quat, b: Quat) -> float:
    a = quat_normalize(a)
    b = quat_normalize(b)
    dot = abs(sum(x*y for x, y in zip(a, b)))
    return 2.0 * math.acos(min(1.0, max(0.0, dot)))


def rotate_vec(q: Quat, v: Vec3) -> Vec3:
    matrix = quat_to_matrix(q)
    return tuple(
        sum(matrix[i][j] * v[j] for j in range(3))
        for i in range(3)
    )


def slerp_toward(q_from: Quat, q_to: Quat,
                 max_step_rad: float) -> Quat:
    """保留原有工具函数接口。水平目标不使用它进行倾斜限幅。"""
    q_from = quat_normalize(q_from)
    q_to = quat_normalize(q_to)
    total = quat_angle_between(q_from, q_to)
    step_angle = max(0.0, max_step_rad)

    if total <= step_angle or total < 1e-9:
        return q_to

    dq = quat_normalize(quat_mul(quat_conj(q_from), q_to))
    if dq[3] < 0.0:
        dq = tuple(-v for v in dq)

    s = math.sqrt(max(0.0, 1.0 - dq[3]*dq[3]))
    if s < 1e-9:
        return q_from

    axis = (dq[0]/s, dq[1]/s, dq[2]/s)
    step = quat_about_axis(axis, step_angle)
    return quat_normalize(quat_mul(q_from, step))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def heading_from_quat(q: Quat, fallback: float = 0.0) -> float:
    """将艇体前方 +Z 投影到世界 XY 平面，得到航向。

    航向 0：朝世界 +X。
    航向 +pi/2：朝世界 +Y。
    """
    matrix = quat_to_matrix(q)
    fx, fy = matrix[0][2], matrix[1][2]

    if math.hypot(fx, fy) > 1e-6:
        return math.atan2(fy, fx)

    # 艇体近乎竖直时，从左舷轴的水平投影估计航向。
    lx, ly = matrix[0][0], matrix[1][0]
    if math.hypot(lx, ly) > 1e-6:
        return math.atan2(-lx, ly)

    return fallback


def level_quat(heading: float) -> Quat:
    """生成水平艇身姿态。

    航向为 0 时：
        艇体 +X -> 世界 +Y（左）
        艇体 +Y -> 世界 +Z（上）
        艇体 +Z -> 世界 +X（前）
    """
    base = (0.5, 0.5, 0.5, 0.5)
    yaw = quat_about_axis((0.0, 0.0, 1.0), heading)
    return quat_normalize(quat_mul(yaw, base))


class GoalManager:
    """保持原有节点调用接口的水平目标管理器。"""

    def __init__(
        self,
        surge_speed=0.30,
        sway_speed=0.25,
        heave_speed=0.25,
        yaw_rate_deg_s=30.0,
        roll_rate_deg_s=0.0,
        pitch_rate_deg_s=0.0,
        max_goal_dist=1.5,
        max_goal_att_deg=45.0,
        workspace_min=(-2.5, -1.0, -2.5),
        workspace_max=(2.5, 2.0, 2.5),
        state_timeout=0.3,
        yaw_axis="world_up",
        world_up=(0.0, 0.0, 1.0),
    ):
        self.surge = float(surge_speed)
        self.sway = float(sway_speed)
        self.heave = float(heave_speed)
        self.yaw_rate = math.radians(yaw_rate_deg_s)

        # 保留兼容字段；本模式不允许倾斜目标。
        self.roll_rate = math.radians(roll_rate_deg_s)
        self.pitch_rate = math.radians(pitch_rate_deg_s)

        self.max_goal_dist = max(0.0, float(max_goal_dist))
        self.max_goal_att = min(
            math.pi, max(0.0, math.radians(max_goal_att_deg))
        )
        self.ws_min = tuple(workspace_min)
        self.ws_max = tuple(workspace_max)
        self.state_timeout = float(state_timeout)

        # 本版固定使用 Z-up 水平模式。
        # 旧节点传入 body_up 或旧 world_up 时也不会改变这一规则。
        self.yaw_axis = "world_up"
        self.world_up = (0.0, 0.0, 1.0)

        self.state_p: Optional[Vec3] = None
        self.state_q: Quat = (0.0, 0.0, 0.0, 1.0)
        self.state_heading = 0.0
        self.state_age = 1e9
        self.have_state = False

        self.goal_p: Optional[Vec3] = None
        self.goal_q: Quat = level_quat(0.0)
        self.goal_initialized = False

        self.last_update_note = ""

    def set_state(self, p: Vec3, q: Quat, age_s: float):
        self.state_p = tuple(p)
        self.state_q = quat_normalize(tuple(q))
        self.state_heading = heading_from_quat(
            self.state_q, self.state_heading
        )
        self.state_age = float(age_s)
        self.have_state = True

        if not self.goal_initialized and self.state_valid:
            self.recapture()

    @property
    def state_valid(self) -> bool:
        return (
            self.have_state
            and math.isfinite(self.state_age)
            and self.state_age <= self.state_timeout
        )

    def recapture(self):
        """捕获当前位置和航向，但目标艇身始终水平。"""
        if self.state_p is None or not self.state_valid:
            return False

        self.goal_p = tuple(self.state_p)
        self.goal_q = level_quat(self.state_heading)
        self.goal_initialized = True
        self.last_update_note = "已捕获当前位置和水平目标姿态"
        return True

    def goal(self):
        return self.goal_p, self.goal_q

    def update(
        self,
        dt: float,
        vel_body: Vec3,
        yaw_cmd: float,
        roll_cmd: float = 0.0,
        pitch_cmd: float = 0.0,
    ):
        """vel_body 顺序仍为 (左移, 上移, 前进)。

        roll_cmd、pitch_cmd 在水平模式下忽略。
        """
        if not math.isfinite(dt) or dt <= 0.0:
            return False

        self.state_age += dt

        if not self.goal_initialized or not self.state_valid:
            self.last_update_note = "等待有效状态"
            return False

        clamp1 = lambda value: max(-1.0, min(1.0, value))
        sway = clamp1(vel_body[0])
        heave = clamp1(vel_body[1])
        surge = clamp1(vel_body[2])
        yaw = clamp1(yaw_cmd)

        moving = (
            abs(sway) + abs(heave) + abs(surge) + abs(yaw)
        ) > 1e-6

        if not moving:
            self.last_update_note = "保持位置、水平姿态和航向"
            return False

        # 平移采用实际航向，但去掉实际艇身的俯仰和侧倾。
        heading = heading_from_quat(self.goal_q)

        forward = (math.cos(heading), math.sin(heading), 0.0)
        left = (-math.sin(heading), math.cos(heading), 0.0)

        vx = surge*self.surge*forward[0] + sway*self.sway*left[0]
        vy = surge*self.surge*forward[1] + sway*self.sway*left[1]
        vz = heave*self.heave

        new_p = (
            self.goal_p[0] + vx*dt,
            self.goal_p[1] + vy*dt,
            self.goal_p[2] + vz*dt,
        )

        # 只更新目标航向，绕世界 +Z 转动。
        goal_heading = heading_from_quat(
            self.goal_q, self.state_heading
        )
        goal_heading = wrap_angle(
            goal_heading + yaw*self.yaw_rate*dt
        )
        new_q = level_quat(goal_heading)

        self.goal_p, self.goal_q = self._clamp(new_p, new_q)
        self.last_update_note = "水平运动目标更新"
        return True

    def _clamp(self, new_p: Vec3, new_q: Quat):
        assert self.state_p is not None

        # 目标与当前位置的距离限幅。
        delta = tuple(
            new_p[i] - self.state_p[i] for i in range(3)
        )
        distance = math.sqrt(sum(v*v for v in delta))
        if distance > self.max_goal_dist and distance > 1e-12:
            scale = self.max_goal_dist / distance
            new_p = tuple(
                self.state_p[i] + delta[i]*scale
                for i in range(3)
            )

        # 绝对工作空间限幅。
        new_p = tuple(
            max(self.ws_min[i], min(self.ws_max[i], new_p[i]))
            for i in range(3)
        )

        # 只限制航向差；不插值回实际的倾斜姿态。
        desired_heading = heading_from_quat(
            new_q, self.state_heading
        )
        heading_error = wrap_angle(
            desired_heading - self.state_heading
        )
        heading_error = max(
            -self.max_goal_att,
            min(self.max_goal_att, heading_error),
        )
        limited_heading = self.state_heading + heading_error

        return new_p, level_quat(limited_heading)