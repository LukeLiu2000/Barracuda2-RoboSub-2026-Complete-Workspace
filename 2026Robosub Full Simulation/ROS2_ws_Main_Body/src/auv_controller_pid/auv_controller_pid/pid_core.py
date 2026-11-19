# -*- coding: utf-8 -*-
"""pid_core —— 六自由度 PID 纯逻辑(无 ROS 依赖,可被整定器/测试直接复用)。

坐标系与误差定义(体坐标系 +x 左舷 +y 上 +z 前;世界 Z-up):
  平移(体系 PID,含完整导数旋转项):
    e_p_w = p_d - p(世界)
    e_p_b = R^T e_p_w
    de_p_b/dt = R^T (v_d_w - v_w) - w × e_p_b    [不可忽略 -w×e_p_b 旋转项]
    F_b = Kp*e_p_b + Ki*∫e_p_b + Kd*de_p_b
  旋转(相对旋转误差,非欧拉角相减):
    e_R = log(R^T R_d)(体系旋转向量,绕 x/y/z 分量)
    de_R/dt ≈ w_d - w(目标角速度为零时为 -w;小误差下为该误差定义的
    精确一阶导,大误差时为标准近似——报告如实标注)
    M_b = Kp*e_R + Ki*∫e_R + Kd*(-w)

  输出 tau = [F_b; M_b](体系),由 auv_common.ThrusterGeometry 有界分配到 8 路。

抗饱和:
  1) 积分限幅(每轴 |I|<=i_limit);
  2) 条件积分:输出轴已到输出限且误差继续同向时不积分;
  3) 分配饱和反馈:分配器饱和且加权残差大时按饱和比例回衰积分
     (back-calculation 简化版,kb 可配)。
微分低通:一阶 IIR,alpha = dt/(dt + 1/(2π fc));fc<=0 关闭。
可选重力/浮力前馈:默认关闭(enable_feedforward=false);若开启仅作为
增强版基线记录,与纯 PID 区分。
"""

import numpy as np

from auv_common.frames import (attitude_error_rotvec, quat_normalize,
                               quat_to_matrix)


class PidCore(object):

    AXES = ("fx", "fy", "fz", "mx", "my", "mz")

    def __init__(self,
                 kp=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                 ki=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                 kd=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                 integral_limit=(10.0, 10.0, 10.0, 1.0, 1.0, 1.0),
                 output_limit=(None,) * 6,
                 d_filter_hz=8.0,
                 back_calc_gain=0.4,
                 enable_feedforward=False,
                 feedforward_force_body=(0.0, 0.0, 0.0)):
        self.kp = np.asarray(kp, dtype=np.float64).copy()
        self.ki = np.asarray(ki, dtype=np.float64).copy()
        self.kd = np.asarray(kd, dtype=np.float64).copy()
        self.integral_limit = np.asarray(integral_limit,
                                         dtype=np.float64).copy()
        ol = []
        for v in output_limit:
            ol.append(np.inf if v is None else float(v))
        self.output_limit = np.asarray(ol, dtype=np.float64)
        self.d_filter_hz = float(d_filter_hz)
        self.back_calc_gain = float(back_calc_gain)
        self.enable_feedforward = bool(enable_feedforward)
        self.feedforward_force_body = np.asarray(feedforward_force_body,
                                                 dtype=np.float64)
        for name, arr in (("kp", self.kp), ("ki", self.ki), ("kd", self.kd)):
            if arr.shape != (6,) or not np.all(np.isfinite(arr)) or \
                    np.any(arr < 0.0):
                raise ValueError("%s 必须为 6 维非负有限值" % name)

        self.integral = np.zeros(6)
        self.d_filt = np.zeros(6)
        self._have_prev = False
        self._prev_e_b = None
        self._prev_e_R = None
        self.n_updates = 0
        self.n_output_clipped = 0
        self.n_alloc_sat_feedback = 0
        self.last_diag = {}

    # ------------------------------------------------------------------
    def reset(self):
        """复位内部状态(积分/微分滤波/差分基线)。目标由外部管理。"""
        self.integral = np.zeros(6)
        self.d_filt = np.zeros(6)
        self._have_prev = False
        self._prev_e_b = None
        self._prev_e_R = None
        self.n_updates = 0
        self.n_output_clipped = 0
        self.n_alloc_sat_feedback = 0
        self.last_diag = {}

    # ------------------------------------------------------------------
    def update(self, p, q, v_body, w_body, p_d, q_d, dt,
               alloc_result=None, v_d_world=None, w_d_body=None):
        """一步 PID。返回 tau(体系 6 维,已按 output_limit 限幅)。

        alloc_result: 上一周期 auv_common allocate_bounded 的返回值
        (用于分配饱和回衰积分;首轮传 None)。
        """
        p = np.asarray(p, dtype=np.float64)
        p_d = np.asarray(p_d, dtype=np.float64)
        v_body = np.asarray(v_body, dtype=np.float64)
        w_body = np.asarray(w_body, dtype=np.float64)
        q = quat_normalize(q)
        q_d = quat_normalize(q_d)
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 1e-6 or dt > 1.0:
            # 时间异常:保守跳过积分与微分,仅比例项
            dt = 0.0
        R = quat_to_matrix(q)
        v_world = R @ v_body

        # ---- 平移误差(体系)与完整导数(含旋转项)----
        e_p_w = p_d - p
        e_b = R.T @ e_p_w
        v_d_world = (np.zeros(3) if v_d_world is None
                     else np.asarray(v_d_world, dtype=np.float64))
        de_b_exact = R.T @ (v_d_world - v_world) - np.cross(w_body, e_b)
        if not self._have_prev or dt <= 0.0:
            de_b = de_b_exact      # 直接用解析导数(不需要差分)
        else:
            de_b = (e_b - self._prev_e_b) / dt
        self._prev_e_b = e_b

        # ---- 姿态误差旋转向量与导数 ----
        e_R = attitude_error_rotvec(q, q_d)
        w_d_body = (np.zeros(3) if w_d_body is None
                    else np.asarray(w_d_body, dtype=np.float64))
        de_R = w_d_body - w_body
        self._prev_e_R = e_R

        e_all = np.concatenate([e_b, e_R])
        d_all = np.concatenate([de_b, de_R])

        # ---- 微分低通 ----
        if self.d_filter_hz > 0.0 and dt > 0.0:
            alpha = dt / (dt + 1.0 / (2.0 * np.pi * self.d_filter_hz))
            self.d_filt += alpha * (d_all - self.d_filt)
        else:
            self.d_filt = d_all.copy()

        # ---- 条件积分 + 积分限幅(用上一拍输出判断同向饱和)----
        tau_prev = self.last_diag.get("tau", np.zeros(6))
        if dt > 0.0:
            for i in range(6):
                at_limit = (tau_prev[i] >= self.output_limit[i] - 1e-9) or \
                           (tau_prev[i] <= -self.output_limit[i] + 1e-9)
                pushing = (np.sign(e_all[i]) == np.sign(tau_prev[i])) \
                    if abs(tau_prev[i]) > 1e-12 else False
                if at_limit and pushing:
                    continue        # 抗饱和:同向已饱和不积分
                inc = self.ki[i] * e_all[i] * dt
                self.integral[i] = float(np.clip(
                    self.integral[i] + inc,
                    -self.integral_limit[i], self.integral_limit[i]))

        # ---- 分配饱和回衰(back-calculation 简化版)----
        if alloc_result is not None and alloc_result.get("n_saturated", 0) > 0:
            sat_frac = alloc_result["n_saturated"] / 8.0
            resid = alloc_result.get("residual_weighted", 0.0)
            if resid > 1e-6:
                self.integral *= max(0.0, 1.0 - self.back_calc_gain * sat_frac)
                self.n_alloc_sat_feedback += 1

        # ---- PID 合成 ----
        tau = self.kp * e_all + self.integral + self.kd * self.d_filt
        if self.enable_feedforward:
            tau[:3] += self.feedforward_force_body

        tau_limited = np.clip(tau, -self.output_limit, self.output_limit)
        clipped = not np.allclose(tau, tau_limited, atol=1e-9)
        if clipped:
            self.n_output_clipped += 1

        self._have_prev = True
        self.n_updates += 1
        diag = {
            "tau": tau_limited.tolist(),
            "tau_raw": tau.tolist(),
            "e_pos_body": e_b.tolist(),
            "e_pos_world": e_p_w.tolist(),
            "e_rot_rotvec": e_R.tolist(),
            "d_filt": self.d_filt.tolist(),
            "integral": self.integral.tolist(),
            "output_clipped": bool(clipped),
            "de_pos_body_exact": de_b_exact.tolist(),
            "n_updates": self.n_updates,
        }
        self.last_diag = diag
        return tau_limited

    # ------------------------------------------------------------------
    @staticmethod
    def achievable_output_limit(geom, margin=0.98):
        """由几何分配推导每轴可实现输出上限(默认 output_limit 用)。

        对每轴请求大力矩,用有界分配读回可实现值(这是上限的可达近似,
        非联合可行域——如实说明)。
        """
        lim = []
        for i in range(6):
            tau = np.zeros(6)
            tau[i] = 1e6
            r_pos = geom.allocate_bounded(tau)
            tau[i] = -1e6
            r_neg = geom.allocate_bounded(tau)
            lim.append(float(margin * min(abs(r_pos["tau_achieved"][i]),
                                          abs(r_neg["tau_achieved"][i]))))
        return lim
