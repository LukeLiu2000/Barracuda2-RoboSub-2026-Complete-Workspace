# -*- coding: utf-8 -*-
"""frames —— 四元数/旋转工具(纯 numpy,无 ROS 依赖)。

约定(全工作区统一):
  - 四元数 q = (x, y, z, w),右手系,Hamilton 乘法,旋转方向 body->world。
  - R(q) 将体系向量映到世界系:v_world = R @ v_body。
  - 相对姿态误差(体系):e_R = log(R^T @ R_d) —— 旋转向量,模长为转角。
  - 体坐标系:+x 左舷, +y 上, +z 前(roll 绕 z / pitch 绕 x / yaw 绕 y)。
    注意:本文件一律以绕 x/y/z 的旋转向量分量表达,不使用航空序欧拉角,
    避免不同项目的 (roll,pitch,yaw) 排列歧义。
"""

import math

import numpy as np

Q_IDENTITY = np.array([0.0, 0.0, 0.0, 1.0])


def quat_normalize(q):
    q = np.asarray(q, dtype=np.float64)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return Q_IDENTITY.copy()
    return q / n


def quat_to_matrix(q):
    """(x,y,z,w) -> 3x3 旋转矩阵(body->world)。"""
    x, y, z, w = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R):
    """3x3 旋转矩阵 -> (x,y,z,w),Shepperd 方法,数值稳定。"""
    R = np.asarray(R, dtype=np.float64)
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return quat_normalize(np.array([x, y, z, w]))


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_about_axis(axis, angle_rad):
    """绕单位轴旋转 angle_rad 的四元数(右手系)。axis 会归一化。"""
    a = np.asarray(axis, dtype=np.float64)
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        return Q_IDENTITY.copy()
    a = a / n
    s = math.sin(angle_rad / 2.0)
    return quat_normalize(np.array([a[0] * s, a[1] * s, a[2] * s,
                                    math.cos(angle_rad / 2.0)]))


def quat_angle_between(a, b):
    """两姿态的相对转角 [rad](最短弧,∈[0,π])。"""
    a = quat_normalize(a)
    b = quat_normalize(b)
    dot = abs(float(np.dot(a, b)))
    return 2.0 * math.acos(min(1.0, max(0.0, dot)))


def rotvec_from_R(R):
    """旋转矩阵 -> 旋转向量(axis*angle)。小角度数值稳定处理。"""
    R = np.asarray(R, dtype=np.float64)
    c = (R[0, 0] + R[1, 1] + R[2, 2] - 1.0) / 2.0
    c = min(1.0, max(-1.0, c))
    angle = math.acos(c)
    if angle < 1e-9:
        # 一阶: v ≈ 0.5*[R32-R23, R13-R31, R21-R12]
        v = 0.5 * np.array([R[2, 1] - R[1, 2],
                            R[0, 2] - R[2, 0],
                            R[1, 0] - R[0, 1]])
        return v
    if angle > math.pi - 1e-6:
        # 接近 π:轴从对称部分恢复
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(A), 0.0, 1.0))
        k = int(np.argmax(axis))
        if axis[k] > 1e-9:
            axis = A[:, k] / axis[k]
            axis = axis / max(np.linalg.norm(axis), 1e-12)
        else:
            axis = np.array([1.0, 0.0, 0.0])
        return axis * angle
    s = math.sin(angle)
    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]]) / (2.0 * s)
    return axis * angle


def attitude_error_rotvec(q, q_d):
    """相对旋转旋转向量(体系):e = log(R^T @ R_d)。

    不是欧拉角相减;绕 x/y/z 的三个分量给出体系误差力矩方向。
    """
    R = quat_to_matrix(q)
    R_d = quat_to_matrix(q_d)
    return rotvec_from_R(R.T @ R_d)


def slerp(q0, q1, t):
    """球面插值,t∈[0,1]。"""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return quat_normalize(q0 + t * (q1 - q0))
    theta0 = math.acos(dot)
    st = math.sin(theta0)
    s0 = math.sin((1.0 - t) * theta0) / st
    s1 = math.sin(t * theta0) / st
    return quat_normalize(s0 * q0 + s1 * q1)


def rotate_body_to_world(q, v_body):
    return quat_to_matrix(q) @ np.asarray(v_body, dtype=np.float64)


def rotate_world_to_body(q, v_world):
    return quat_to_matrix(q).T @ np.asarray(v_world, dtype=np.float64)


def angular_velocity_from_quats(q_prev, q_now, dt):
    """相邻姿态差分角速度(体系,[rad/s])。"""
    if dt <= 1e-9:
        return np.zeros(3)
    dq = quat_mul(quat_normalize(q_now), quat_conj(quat_normalize(q_prev)))
    if dq[3] < 0.0:
        dq = -dq
    dq = quat_normalize(dq)
    s = math.sqrt(max(0.0, 1.0 - dq[3] * dq[3]))
    if s < 1e-9:
        return np.zeros(3)
    angle = 2.0 * math.acos(min(1.0, max(-1.0, dq[3])))
    return (dq[:3] / s) * (angle / dt)


def body_exyz_axes_from_quat(q):
    """返回体系 +x/+y/+z 单位轴在世界系中的方向(三列)。

    本项目体 +z 为艇体前方:位姿箭头方向 = R @ [0,0,1]。
    """
    return quat_to_matrix(q)
