# -*- coding: utf-8 -*-
"""alloc —— 八推进器几何配置加载、分配矩阵 B 与有界分配。

模型(与任务书一致):
  第 i 个推进器:单位方向 d_i(体系),相对参考点位置 r_i(体系, m)。
  贡献:合力 d_i f_i,力矩 (r_i × d_i) f_i。
  分配矩阵 B 为 6×8,第 i 列 B_i = [d_i; r_i × d_i],tau = B f。

参考点 = 体积质心估计(thrusters.yaml com_offset_base_link,estimated;
真质心未实测——此事实在全链路保留)。桥发布的里程计即质心系,因此
r_i 与状态位置同源,不重复施加质心偏移(apply_com_offset=false)。

有界分配 allocate_bounded():
  投影最小二乘(迭代饱和-重解,逐个把最违反界的自由列钉到界上后对
  剩余自由列重解加权最小范数解),不是"伪逆后逐项裁剪"——被钉住的
  列取边界值后,其余列会重新优化以最好逼近目标力矩,并如实返回
  可实现 wrench 与饱和统计。这是有界最小二乘的近似迭代解(小规模
  6×8,最多 8 轮),不是精确 QP;测试用 scipy.optimize.lsq_linear
  (可用时)交叉验证可达性误差。
  力/力矩行加权:tau_rows / torque_scale 把 N·m 折算到 N 量级
  (默认 0.3 m ≈ 典型力臂),避免数值尺度失衡。
"""

import math
import os

import numpy as np

try:
    import yaml
except Exception:                                    # noqa: BLE001
    yaml = None


DEFAULT_CONFIG_CANDIDATES = (
    # 包内相对路径(源码树与 colcon share 安装均命中)
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "..", "config", "thrusters.yaml"),
    # 服务器工作区绝对路径(无 ROS 环境也能加载,训练/整定用)
    "/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/"
    "2026Robosub Full Simulation/ROS2_ws_Main_Body/src/auv_common/config/thrusters.yaml",
)


class ThrusterGeometry(object):
    """8 推进器几何 + 限幅 + 分配矩阵与有界分配。"""

    def __init__(self, positions, directions, force_min, force_max,
                 com_offset_base_link=(0.0, 0.0, 0.0),
                 torque_scale=0.3, source="inline"):
        self.positions = np.asarray(positions, dtype=np.float64).reshape(8, 3)
        self.directions = np.asarray(directions, dtype=np.float64).reshape(8, 3)
        if not (np.all(np.isfinite(self.positions)) and
                np.all(np.isfinite(self.directions))):
            raise ValueError("推进器几何含 NaN/Inf")
        norms = np.linalg.norm(self.directions, axis=1)
        if np.any(norms < 1e-9):
            raise ValueError("存在零方向推进器")
        self.directions = self.directions / norms[:, None]
        self.force_min = float(force_min)
        self.force_max = float(force_max)
        if not (self.force_min < 0.0 < self.force_max):
            raise ValueError("force_min 必须为负、force_max 必须为正(正反对称限幅)")
        self.com_offset_base_link = np.asarray(com_offset_base_link,
                                               dtype=np.float64)
        self.torque_scale = float(torque_scale)
        self.source = str(source)

        self.B = self.build_matrix()
        self.rank = int(np.linalg.matrix_rank(self.B))
        u, s, vt = np.linalg.svd(self.B)
        self.singular_values = s
        self.cond = float(s[0] / s[-1]) if s[-1] > 1e-12 else float("inf")
        # 行加权矩阵(力=1,力矩=1/torque_scale):W B 把力矩行折到力尺度
        self._W = np.diag(np.concatenate(
            [np.ones(3), np.full(3, 1.0 / self.torque_scale)]))

    # ---------------------------------------------------------------- 构建
    @classmethod
    def from_dict(cls, d, **kw):
        """从 {positions, directions, force_min, force_max, ...} 字典构造。"""
        return cls(
            positions=d["positions"],
            directions=d["directions"],
            force_min=d.get("force_min", d.get("force_min_n", -39.2266)),
            force_max=d.get("force_max", d.get("force_max_n", 39.2266)),
            com_offset_base_link=d.get("com_offset_base_link", (0, 0, 0)),
            **kw)

    @classmethod
    def from_ros_yaml(cls, path, **kw):
        """解析 ROS 参数 yaml(/**: ros__parameters: thrusters: ...)。"""
        if yaml is None:
            raise RuntimeError("PyYAML 不可用")
        with open(path, "r", encoding="utf-8") as f:
            root = yaml.safe_load(f)
        node = root.get("/**") or next(iter(root.values()))
        thr = node["ros__parameters"]["thrusters"]
        return cls.from_dict(thr, source=path, **kw)

    @classmethod
    def load_default(cls, **kw):
        """加载默认配置:优先服务器工作区 auv_common yaml。"""
        for p in DEFAULT_CONFIG_CANDIDATES:
            if os.path.isfile(p):
                return cls.from_ros_yaml(p, **kw)
        raise FileNotFoundError(
            "未找到推进器配置,尝试过: %s" % (DEFAULT_CONFIG_CANDIDATES,))

    def build_matrix(self):
        """6×8 分配矩阵 B,第 i 列 = [d_i; r_i × d_i]。"""
        B = np.zeros((6, 8))
        for i in range(8):
            d = self.directions[i]
            r = self.positions[i]
            B[:3, i] = d
            B[3:, i] = np.cross(r, d)
        return B

    # ---------------------------------------------------------------- 分配
    def allocate_bounded(self, tau_desired, max_iter=10):
        """有界分配:返回 dict(f, tau_achieved, saturated_mask, residual_w)。

        tau_desired: 体系 [Fx,Fy,Fz,Mx,My,Mz]。
        迭代饱和-重解:每次把加权残差下最违反界的自由列钉到最近界,
        对剩余自由列重解最小范数加权最小二乘;最多 max_iter 轮。
        """
        tau = np.asarray(tau_desired, dtype=np.float64).reshape(6)
        if not np.all(np.isfinite(tau)):
            tau = np.zeros(6)
        B = self.B
        W = self._W
        f = np.zeros(8)
        fixed = np.zeros(8, dtype=bool)      # 已钉在界上的列
        for _ in range(int(max_iter)):
            # 残差目标:总 tau 减去已固定列的贡献
            tau_eff = tau - B[:, fixed] @ f[fixed] if fixed.any() else tau
            free = ~fixed
            if not free.any():
                break
            Bf = W @ B[:, free]
            # 加权最小范数解:f_free = pinv(W B_free) W tau_eff
            sol, *_ = np.linalg.lstsq(Bf, W @ tau_eff, rcond=None)
            f_new = f.copy()
            f_new[free] = sol
            viol = np.where(f_new < self.force_min, self.force_min - f_new,
                            np.where(f_new > self.force_max,
                                     self.force_max - f_new, 0.0))
            n_viol = int(np.count_nonzero(np.abs(viol) > 1e-9))
            if n_viol == 0:
                f = f_new
                break
            # 钉住本轮违反量最大的一列(避免一轮全钉导致的过度保守)
            i = int(np.argmax(np.abs(viol)))
            f[i] = self.force_min if viol[i] > 0 else self.force_max
            fixed[i] = True
        f = np.clip(f, self.force_min, self.force_max)
        tau_ach = B @ f
        sat_mask = (np.abs(f - self.force_min) < 1e-6) | \
                   (np.abs(f - self.force_max) < 1e-6)
        return {
            "f": f,
            "tau_achieved": tau_ach,
            "tau_requested": tau,
            "saturated_mask": sat_mask,
            "n_saturated": int(np.count_nonzero(sat_mask)),
            "residual_weighted": float(
                np.linalg.norm(W @ (tau_ach - tau))),
        }

    # ---------------------------------------------------------------- 诊断
    def summary(self):
        return {
            "source": self.source,
            "rank": self.rank,
            "cond": round(self.cond, 4),
            "singular_values": [round(float(v), 5)
                                for v in self.singular_values],
            "force_limits_n": [self.force_min, self.force_max],
            "com_offset_base_link_m": self.com_offset_base_link.tolist(),
            "com_is_estimated": True,     # 体积质心估计,非实测——如实标注
            "positions_m": self.positions.tolist(),
            "directions_body": self.directions.tolist(),
            "B": self.B.tolist(),
        }

    def check_single_thrust_response(self):
        """逐推进器单推的体系合力/合力矩(方向符号核验用表)。"""
        out = []
        for i in range(8):
            f = np.zeros(8)
            f[i] = 1.0
            tau = self.B @ f
            out.append({
                "index": i,
                "force_body": np.round(tau[:3], 4).tolist(),
                "torque_body": np.round(tau[3:], 4).tolist(),
            })
        return out


def cross_check_with_scipy(geom, tau):
    """scipy 可用时用 lsq_linear 精确解交叉验证(测试用,运行不依赖)。"""
    try:
        from scipy.optimize import lsq_linear
    except Exception:                                  # noqa: BLE001
        return None
    W = geom._W
    res = lsq_linear(W @ geom.B, W @ np.asarray(tau, dtype=np.float64),
                     bounds=(geom.force_min, geom.force_max))
    return np.asarray(res.x, dtype=np.float64)
