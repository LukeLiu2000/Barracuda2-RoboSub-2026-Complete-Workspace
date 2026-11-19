#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pid_controller_node —— 六自由度 PID 控制器 ROS 2 节点。

话题:
  订阅 /auv/state/control        nav_msgs/Odometry(SensorDataQoS,质心体系)
  订阅 /auv/control/goal         geometry_msgs/PoseStamped(世界系目标位姿)
  订阅 /auv/control/estop        std_msgs/Bool(急停,true=输出置零)
  订阅 /auv/control/heartbeat    std_msgs/String(控制器互斥仲裁)
  发布 /auv/control/thruster_cmd std_msgs/Float64MultiArray(8 路,SI 牛顿)
  发布 /auv/pid/diagnostics      std_msgs/String(JSON 诊断/指标采集)
  发布 /auv/pid/effective_goal   geometry_msgs/PoseStamped(监测,见下)
  发布 /auv/pid/goal_source      std_msgs/String(监测:"external"|"hold"|"none")
服务:
  ~/reset  std_srvs/srv/Trigger(积分/滤波/目标复位;hold 重新捕获)

监测发布(2026-11-10 增,默认 10 Hz,rates.monitor_hz 可调,0=关闭):
  - effective_goal 只发布控制回路**本轮真正喂给 PID 计算**的目标位姿
    (外部目标或内部 hold 捕获值),frame_id 跟随最新状态消息(auv_world),
    时间戳=采用该目标那一拍的仿真时间;目标未建立/reset 后/状态过期
    (safety.state_timeout_wall_s)/急停/被其它控制器压制时不发布——
    曲线断流即"此刻没有有效目标",不伪造零坐标或过期数据。
  - goal_source 始终发布,标明 effective_goal 来源:
      external  PID 正在跟踪 /auv/control/goal 的外部目标
      hold      无外部目标,hold_on_start 捕获的内部目标
                (注意:hold 目标每拍重捕当前位姿,故 effective_goal≈当前位姿,
                输出表现为速度阻尼;这是既有控制语义,本监测只如实呈现)
      none      尚无任何目标被采用(刚启动/reset 后)
  监测在独立定时器执行,构建/发布一次消息的成本远低于 1 ms,不阻塞
  50 Hz 控制循环;发布失败只记节流日志,绝不抛出影响控制。
  MPPI/GRU 不向这些话题发布(名称专属 /auv/pid/*)。

时间约定:
  - 控制计算用状态消息的仿真时间戳(与 /clock 一致);
  - 状态过期与命令超时用单调墙钟 time.monotonic()(仿真暂停时墙钟仍在走,
    不会把暂停误判为新鲜;仿真时间回跳视为不连续,复位微分/积分)。
互斥:
  - 心跳 2 Hz 发布 "pid";收到其它控制器名的心跳且比本节点启动晚时,
    本节点停止发布推进器命令(安全置零一次)并在诊断中标记,直至对方消失。
"""

import json
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
from auv_controller_pid.pid_core import PidCore


def _vec3(v):
    return np.array([v.x, v.y, v.z], dtype=np.float64)


class PidControllerNode(Node):

    def __init__(self):
        super().__init__("pid_controller_node")
        # 参数全部由 _declare_common 声明默认值 + yaml/--params-file 覆盖,
        # 无需(也没有)单独的 _load_params 步骤。
        self._declare_common()
        self._geom = ThrusterGeometry.load_default()
        self._core = PidCore(
            kp=self._g("pid.kp"), ki=self._g("pid.ki"), kd=self._g("pid.kd"),
            integral_limit=self._g("pid.integral_limit"),
            output_limit=self._output_limits(),
            d_filter_hz=self._p("pid.d_filter_hz"),
            back_calc_gain=self._p("pid.back_calc_gain"),
            enable_feedforward=self._p("pid.enable_feedforward"),
            feedforward_force_body=self._g("pid.feedforward_force_body"))

        st = self.get_parameter("topics.state").value
        gl = self.get_parameter("topics.goal").value
        self.create_subscription(Odometry, st, self._on_state,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, gl, self._on_goal, 10)
        self.create_subscription(Bool, "/auv/control/estop",
                                 self._on_estop, 10)
        self.pub_cmd = self.create_publisher(
            Float64MultiArray, "/auv/control/thruster_cmd", 10)
        self.pub_diag = self.create_publisher(String, "/auv/pid/diagnostics", 10)
        self.pub_eff = self.create_publisher(
            PoseStamped, self._p("topics.effective_goal"), 10)
        self.pub_src = self.create_publisher(
            String, self._p("topics.goal_source"), 10)

        self.create_service(Trigger, "~/reset", self._on_reset)
        self._hb_pub = self.create_publisher(String, "/auv/control/heartbeat", 10)
        self._hb_sub = self.create_subscription(String, "/auv/control/heartbeat",
                                                self._on_heartbeat, 10)

        self._state = None            # (sim_t, wall_mono, p, q, v, w)
        self._goal = None             # (sim_t, wall_mono, p, q)
        self._goal_held = False
        self._estop = False
        self._prev_sim_t = None
        self._alloc_prev = None
        self._other_ctrl = None       # (name, wall_mono)
        self._suppressed = False
        self._t_start = time.monotonic()
        self._cycle_ms = []
        self._last_diag_pub = 0.0
        # ---- 监测状态 ----
        self._state_frame = "auv_world"   # 跟随状态消息 header.frame_id
        self._effective = None            # (sim_t, p, q, source) 最近一次
                                          # 真正喂给 core.update 的目标

        rate = float(self.get_parameter("rates.control_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._on_timer)
        self.create_timer(0.5, self._publish_heartbeat)
        mhz = float(self._p("rates.monitor_hz") or 0.0)
        if mhz > 0.0:
            self.create_timer(1.0 / min(mhz, max(rate, 1.0)),
                              self._publish_monitor)
        self.get_logger().info(
            "PID 节点启动 kp=%s ki=%s kd=%s" % (
                np.round(self._core.kp, 2).tolist(),
                np.round(self._core.ki, 2).tolist(),
                np.round(self._core.kd, 2).tolist()))

    # ------------------------------------------------------------------
    def _declare_common(self):
        defaults = {
            "topics.state": "/auv/state/control",
            "topics.goal": "/auv/control/goal",
            "topics.effective_goal": "/auv/pid/effective_goal",
            "topics.goal_source": "/auv/pid/goal_source",
            "rates.control_hz": 50.0,
            "rates.monitor_hz": 10.0,
            "safety.state_timeout_wall_s": 0.6,
            "safety.goal_timeout_sim_s": 1e9,
            "pid.kp": [0.0] * 6, "pid.ki": [0.0] * 6, "pid.kd": [0.0] * 6,
            "pid.integral_limit": [10.0, 10.0, 10.0, 1.0, 1.0, 1.0],
            "pid.output_limit": [1e9] * 6,
            "pid.d_filter_hz": 8.0,
            "pid.back_calc_gain": 0.4,
            "pid.enable_feedforward": False,
            "pid.feedforward_force_body": [0.0, 0.0, 0.0],
            "pid.hold_on_start": True,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)

    def _p(self, name):
        return self.get_parameter(name).value

    def _g(self, name):
        return np.asarray(self._p(name), dtype=np.float64)

    def _output_limits(self):
        ol = self._g("pid.output_limit")
        out = []
        for v in ol:
            out.append(None if (not np.isfinite(v)) or v > 1e8 else float(v))
        return out

    # ------------------------------------------------------------------
    def _on_state(self, msg):
        h = msg.header.stamp
        sim_t = h.sec + h.nanosec * 1e-9
        if msg.header.frame_id:
            self._state_frame = msg.header.frame_id
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self._state = (sim_t, time.monotonic(),
                       np.array([msg.pose.pose.position.x,
                                 msg.pose.pose.position.y,
                                 msg.pose.pose.position.z], dtype=np.float64),
                       quat_normalize(q),
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

    def _on_heartbeat(self, msg):
        name = (msg.data or "").strip()
        if name and name != "pid":
            self._other_ctrl = (name, time.monotonic())
            if not self._suppressed:
                self._suppressed = True
                self.get_logger().error(
                    "检测到其它控制器心跳 %r,本节点停止发布推进器命令" % name)

    def _publish_heartbeat(self):
        m = String()
        m.data = "pid"
        self._hb_pub.publish(m)

    def _on_reset(self, req, resp):
        self._core.reset()
        self._goal = None
        self._goal_held = False
        self._prev_sim_t = None
        self._alloc_prev = None
        self._effective = None      # 监测同步复位:回到 none,不发布旧目标
        resp.success = True
        resp.message = "PID 积分/微分/目标已复位"
        self.get_logger().info(resp.message)
        return resp

    # ------------------------------------------------------------------
    def _on_timer(self):
        t0 = time.monotonic()
        now = t0
        # 互斥:其它控制器活跃则不出力
        if self._other_ctrl is not None and \
                now - self._other_ctrl[1] > 1.5:
            self._other_ctrl = None
            self._suppressed = False
            self.get_logger().warning("其它控制器心跳消失,恢复发布")
        # 状态检查(墙钟新鲜度)
        if self._state is None or now - self._state[1] > \
                float(self._p("safety.state_timeout_wall_s")):
            self._publish_zero("state_stale")
            self._publish_diag("state_stale", t0)
            return
        sim_t, _, p, q, v, w = self._state

        if self._estop or self._suppressed:
            self._publish_zero("estop" if self._estop else "suppressed")
            self._publish_diag("estop" if self._estop else "suppressed", t0)
            return

        # 目标:无外部目标时捕获当前位姿(hold),目标速度为零
        if self._goal is None or self._goal_held:
            if self._p("pid.hold_on_start"):
                self._goal = (sim_t, now, p.copy(), q.copy())
                self._goal_held = True
            else:
                self._publish_zero("no_goal")
                self._publish_diag("no_goal", t0)
                return
        _, _, p_d, q_d = self._goal
        # 监测快照:记录本拍真正进入 PID 计算的目标(hold 模式每拍重捕,
        # 快照随之更新;这是对既有控制语义的如实记录,不是转发 /auv/control/goal)
        self._effective = (sim_t, p_d.copy(), q_d.copy(),
                           "hold" if self._goal_held else "external")

        # 仿真时间连续性
        if self._prev_sim_t is not None:
            dt = sim_t - self._prev_sim_t
            if dt < -1e-3:
                self.get_logger().warning(
                    "仿真时间回跳 %.3f -> %.3f,复位 PID 内部状态" % (
                        self._prev_sim_t, sim_t))
                self._core.reset()
            elif dt > 0.5:
                self.get_logger().warning("仿真时间大步进 dt=%.3f" % dt)
        else:
            dt = 0.02
        self._prev_sim_t = sim_t
        if dt <= 1e-6:
            dt = 0.02

        tau = self._core.update(p, q, v, w, p_d, q_d, dt,
                                alloc_result=self._alloc_prev)
        alloc = self._geom.allocate_bounded(tau)
        self._alloc_prev = alloc

        msg = Float64MultiArray()
        msg.data = [float(x) for x in alloc["f"]]
        self.pub_cmd.publish(msg)
        self._cycle_ms.append((time.monotonic() - t0) * 1000.0)
        self._cycle_ms = self._cycle_ms[-500:]
        self._publish_diag("tracking", t0, tau=tau, alloc=alloc, dt=dt,
                           sim_t=sim_t, p=p, q=q, p_d=p_d, q_d=q_d)

    def _publish_zero(self, reason):
        msg = Float64MultiArray()
        msg.data = [0.0] * 8
        self.pub_cmd.publish(msg)

    # ------------------------------------------------------------------
    def _publish_monitor(self):
        """监测发布(默认 10 Hz):effective_goal + goal_source。

        goal_source 始终发布(effective 快照存在时以其来源为准,保证两条
        监测话题一致);effective_goal 仅在控制回路当前确实在用该目标
        (状态新鲜、未急停、未被其它控制器压制)时发布。
        """
        try:
            eff = self._effective
            if eff is not None:
                src = eff[3]
            elif self._goal is not None:
                # 目标已到但尚未被任何一拍控制采用(≤1 个控制周期)
                src = "hold" if self._goal_held else "external"
            else:
                src = "none"
            m = String()
            m.data = src
            self.pub_src.publish(m)

            if eff is None:
                return
            now = time.monotonic()
            if self._state is None or now - self._state[1] > \
                    float(self._p("safety.state_timeout_wall_s")):
                return                     # 状态断流/暂停:不发过期目标
            if self._estop or self._suppressed:
                return                     # 不在跟踪:不发"有效目标"

            sim_t, p_d, q_d, _ = eff
            g = PoseStamped()
            g.header.frame_id = self._state_frame
            sec = int(sim_t)
            nsec = int(round((sim_t - sec) * 1e9))
            if nsec >= 1_000_000_000:
                sec, nsec = sec + 1, 0
            g.header.stamp.sec = sec
            g.header.stamp.nanosec = nsec
            g.pose.position.x = float(p_d[0])
            g.pose.position.y = float(p_d[1])
            g.pose.position.z = float(p_d[2])
            g.pose.orientation.x = float(q_d[0])
            g.pose.orientation.y = float(q_d[1])
            g.pose.orientation.z = float(q_d[2])
            g.pose.orientation.w = float(q_d[3])
            self.pub_eff.publish(g)
        except Exception as exc:            # noqa: BLE001
            self.get_logger().warning(
                "monitor 发布失败: %s" % exc, throttle_duration_sec=5.0)

    def _publish_diag(self, mode, t0, **kw):
        now = time.monotonic()
        if now - self._last_diag_pub < 0.2 and mode == "tracking":
            return
        self._last_diag_pub = now
        cms = self._cycle_ms
        diag = {
            "controller": "pid",
            "mode": mode,
            "goal_source": ("hold" if self._goal_held
                            else ("external" if self._goal is not None
                                  else None)),
            "cycle_ms": round(float(np.mean(cms)), 3) if cms else None,
            "cycle_ms_max": round(float(np.max(cms)), 3) if cms else None,
            "wall_time": time.time(),
            "estop": self._estop,
            "suppressed": self._suppressed,
        }
        if kw:
            tau = kw.get("tau")
            alloc = kw.get("alloc")
            diag.update({
                "sim_time_s": round(float(kw.get("sim_t", 0.0)), 4),
                "control_dt_s": round(float(kw.get("dt", 0.0)), 4),
                "tau_requested": np.round(tau, 3).tolist(),
                "tau_achieved": np.round(alloc["tau_achieved"], 3).tolist(),
                "thruster_req_n": np.round(alloc["f"], 3).tolist(),
                "n_saturated": alloc["n_saturated"],
                "saturated_mask": alloc["saturated_mask"].astype(int).tolist(),
                "pid": {k: (np.round(v, 4).tolist()
                            if isinstance(v, np.ndarray) else v)
                        for k, v in self._core.last_diag.items()},
            })
        m = String()
        m.data = json.dumps(diag, ensure_ascii=False, default=str)
        self.pub_diag.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PidControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
