#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gru_controller_node —— GRU 循环策略推理 ROS 2 节点(numpy 前向)。

话题/服务与 PID 节点一致(见 pid_controller_node 文档);差异:
  - 策略权重: parameters.model_npz + parameters.model_meta(json)
    (训练导出产物;numpy 前向与 torch 逐点一致,单测覆盖);
  - GRU 隐状态跨正常控制周期保留;以下情形复位:
      ~/reset_hidden 服务、/auv/control/reset、仿真时间回跳、
      状态位置跳变 > teleport_m(默认 0.5 m);
    普通目标更新【不】复位隐状态。
  - 动作映射:a_i ∈ [-1,1] -> f_i = a_i * Fmax(N),Fmax 来自公共配置;
    输出为物理推力百分比(非 PWM)。
  - 诊断: /auv/gru/diagnostics(String JSON,含推理耗时)。
"""

import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

from auv_common.alloc import ThrusterGeometry
from auv_common.frames import quat_normalize
from auv_controller_gru.gru_policy import GruPolicyNumpy
from auv_controller_gru.obs_builder import (ObsNormalizer, build_obs,
                                             load_metadata)


def _vec3(v):
    return np.array([v.x, v.y, v.z], dtype=np.float64)


class GruControllerNode(Node):

    def __init__(self):
        super().__init__("gru_controller_node")
        self.declare_parameter("model_npz", "")
        self.declare_parameter("model_meta", "")
        self.declare_parameter("topics.state", "/auv/state/control")
        self.declare_parameter("topics.goal", "/auv/control/goal")
        self.declare_parameter("rates.control_hz", 50.0)
        self.declare_parameter("safety.state_timeout_wall_s", 0.6)
        self.declare_parameter("gru.teleport_reset_m", 0.5)
        self.declare_parameter("gru.hold_on_start", True)
        self.declare_parameter("gru.action_scale_n", 0.0)   # 0=用公共配置

        npz = self.get_parameter("model_npz").value
        meta_p = self.get_parameter("model_meta").value
        if not npz or not os.path.isfile(npz):
            raise RuntimeError("model_npz 未配置或不存在: %r" % npz)
        if not meta_p or not os.path.isfile(meta_p):
            raise RuntimeError("model_meta 未配置或不存在: %r" % meta_p)
        self._policy = GruPolicyNumpy(npz)
        meta = load_metadata(meta_p)
        self._norm = ObsNormalizer.from_dict(meta["normalizer"])
        self._model_tag = os.path.basename(npz)

        self._geom = ThrusterGeometry.load_default()
        self._fmax = float(self.get_parameter("gru.action_scale_n").value) or \
            self._geom.force_max

        self._hidden = self._policy.reset_hidden()
        self._last_cmd_norm = np.zeros(8)
        self._state = None
        self._goal = None
        self._goal_held = False
        self._estop = False
        self._suppressed = False
        self._other_ctrl = None
        self._prev_sim_t = None
        self._prev_p = None
        self._infer_ms = []

        st = self.get_parameter("topics.state").value
        gl = self.get_parameter("topics.goal").value
        self.create_subscription(Odometry, st, self._on_state,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, gl, self._on_goal, 10)
        self.create_subscription(Bool, "/auv/control/estop", self._on_estop, 10)
        self.create_subscription(String, "/auv/control/reset",
                                 self._on_reset_topic, 10)
        self.pub_cmd = self.create_publisher(
            Float64MultiArray, "/auv/control/thruster_cmd", 10)
        self.pub_diag = self.create_publisher(String, "/auv/gru/diagnostics", 10)
        self.create_service(Trigger, "~/reset_hidden", self._on_reset_hidden)
        self.create_service(Trigger, "~/reset", self._on_reset_hidden)
        self._hb_pub = self.create_publisher(String, "/auv/control/heartbeat", 10)
        self._hb_sub = self.create_subscription(String, "/auv/control/heartbeat",
                                                self._on_heartbeat, 10)

        rate = float(self.get_parameter("rates.control_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._on_timer)
        self.create_timer(0.5, self._publish_heartbeat)
        self.get_logger().info(
            "GRU 节点启动 model=%s fmax=%.4g N" % (self._model_tag, self._fmax))

    # ------------------------------------------------------------------
    def _on_state(self, msg):
        h = msg.header.stamp
        sim_t = h.sec + h.nanosec * 1e-9
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,
                      msg.pose.pose.position.z], dtype=np.float64)
        # 状态不连续检测(位置跳变):复位 GRU 隐状态
        if self._prev_p is not None:
            jump = float(np.linalg.norm(p - self._prev_p))
            if jump > float(self.get_parameter("gru.teleport_reset_m").value):
                self._hidden = self._policy.reset_hidden()
                self._last_cmd_norm = np.zeros(8)
                self.get_logger().warning(
                    "状态位置跳变 %.3f m,GRU 隐状态复位" % jump)
        self._prev_p = p
        self._state = (sim_t, time.monotonic(), p, quat_normalize(q),
                       _vec3(msg.twist.twist.linear),
                       _vec3(msg.twist.twist.angular))

    def _on_goal(self, msg):
        h = msg.header.stamp
        sim_t = h.sec + h.nanosec * 1e-9
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w])
        self._goal = (sim_t, time.monotonic(),
                      np.array([msg.pose.position.x, msg.pose.position.y,
                                msg.pose.position.z], dtype=np.float64),
                      quat_normalize(q))
        self._goal_held = False

    def _on_estop(self, msg):
        self._estop = bool(msg.data)
        if self._estop:
            self.get_logger().warning("急停:输出置零")

    def _on_reset_topic(self, msg):
        self._hidden = self._policy.reset_hidden()
        self._last_cmd_norm = np.zeros(8)
        self._goal = None
        self._goal_held = False
        self.get_logger().info("reset 话题:GRU 隐状态与目标复位")

    def _on_heartbeat(self, msg):
        name = (msg.data or "").strip()
        if name and name != "gru":
            self._other_ctrl = (name, time.monotonic())
            if not self._suppressed:
                self._suppressed = True
                self.get_logger().error(
                    "检测到其它控制器心跳 %r,停止发布推进器命令" % name)

    def _publish_heartbeat(self):
        m = String()
        m.data = "gru"
        self._hb_pub.publish(m)

    def _on_reset_hidden(self, req, resp):
        self._hidden = self._policy.reset_hidden()
        self._last_cmd_norm = np.zeros(8)
        resp.success = True
        resp.message = "GRU 隐状态与上一命令已复位"
        return resp

    # ------------------------------------------------------------------
    def _on_timer(self):
        t0 = time.monotonic()
        if self._other_ctrl is not None and \
                t0 - self._other_ctrl[1] > 1.5:
            self._other_ctrl = None
            self._suppressed = False
            self.get_logger().warning("其它控制器心跳消失,恢复发布")
        if self._state is None or t0 - self._state[1] > float(
                self.get_parameter("safety.state_timeout_wall_s").value):
            self._publish_zero()
            self._publish_diag("state_stale", t0)
            return
        if self._estop or self._suppressed:
            self._publish_zero()
            self._publish_diag("estop" if self._estop else "suppressed", t0)
            return
        sim_t, _, p, q, v, w = self._state
        if self._goal is None or self._goal_held:
            if self.get_parameter("gru.hold_on_start").value:
                self._goal = (sim_t, t0, p.copy(), q.copy())
                self._goal_held = True
            else:
                self._publish_zero()
                self._publish_diag("no_goal", t0)
                return
        _, _, p_d, q_d = self._goal

        # 仿真时间回跳:隐状态复位(回合边界)
        if self._prev_sim_t is not None and sim_t < self._prev_sim_t - 1e-3:
            self._hidden = self._policy.reset_hidden()
            self._last_cmd_norm = np.zeros(8)
            self.get_logger().warning("仿真时间回跳,GRU 隐状态复位")
        self._prev_sim_t = sim_t

        obs = build_obs(p, q, v, w, p_d, q_d,
                        vertical_ref_world=(0.0, 0.0, 1.0),
                        last_cmd_norm=self._last_cmd_norm)
        obs_n = self._norm.normalize(obs)
        action, self._hidden = self._policy.step(obs_n, self._hidden)
        action = np.clip(action, -1.0, 1.0)
        f = action * self._fmax
        self._last_cmd_norm = action.copy()

        msg = Float64MultiArray()
        msg.data = [float(x) for x in f]
        self.pub_cmd.publish(msg)
        self._infer_ms.append((time.monotonic() - t0) * 1000.0)
        self._infer_ms = self._infer_ms[-500:]
        self._publish_diag("tracking", t0, obs=obs, action=action, f=f,
                           sim_t=sim_t)

    def _publish_zero(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * 8
        self.pub_cmd.publish(msg)

    def _publish_diag(self, mode, t0, **kw):
        diag = {
            "controller": "gru",
            "mode": mode,
            "model": self._model_tag,
            "infer_ms": round(float(np.mean(self._infer_ms)), 3)
                        if self._infer_ms else None,
            "infer_ms_max": round(float(np.max(self._infer_ms)), 3)
                            if self._infer_ms else None,
            "wall_time": time.time(),
            "estop": self._estop,
            "suppressed": self._suppressed,
        }
        if kw:
            diag.update({
                "sim_time_s": round(float(kw.get("sim_t", 0.0)), 4),
                "obs": np.round(kw["obs"], 4).tolist(),
                "action_norm": np.round(kw["action"], 4).tolist(),
                "thruster_req_n": np.round(kw["f"], 3).tolist(),
            })
        m = String()
        m.data = json.dumps(diag, ensure_ascii=False, default=str)
        self.pub_diag.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = GruControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
