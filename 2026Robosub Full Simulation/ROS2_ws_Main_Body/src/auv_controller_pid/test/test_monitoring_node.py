#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PID 监测发布节点级测试(无 Isaac,纯消息级)。

用法(在无其它 ROS 图的隔离域跑,例如):
  ROS_DOMAIN_ID=42 python3 test_monitoring_node.py

覆盖:
  T1 hold_on_start:无外部目标 → goal_source=hold,effective_goal 跟随当前位姿
  T2 外部目标到达 → goal_source=external,effective_goal=外部目标值
  T3 外部目标变更 → effective_goal 更新为新值
  T4 ~/reset → goal_source=none,无 effective_goal;随后 hold 重新捕获
  T5 状态断流(>state_timeout_wall_s)→ effective_goal 停发(不冒充有效)
  T6 急停 → effective_goal 停发;解除后恢复

全部通过打印 ALL MONITOR TESTS PASSED。
注意:Jazzy rclpy 解释器收尾可能 C++ 析构 abort(exit 250),结果打印后用
os._exit 规避(与既有实验脚本同策略)。
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.join(os.path.dirname(
    os.path.abspath(__file__)), ".."))
from auv_controller_pid.pid_controller_node import PidControllerNode  # noqa: E402

RESULTS = []


def check(name, cond, detail=""):
    RESULTS.append((name, bool(cond)))
    print("%s %s %s" % ("PASS" if cond else "FAIL", name, detail),
          flush=True)


class Harness(Node):
    def __init__(self):
        super().__init__("monitor_test_harness")
        self.pub_state = self.create_publisher(
            Odometry, "/auv/state/control", qos_profile_sensor_data)
        self.pub_goal = self.create_publisher(
            PoseStamped, "/auv/control/goal", 10)
        self.pub_estop = self.create_publisher(Bool, "/auv/control/estop", 10)
        self.sub_eff = self.create_subscription(
            PoseStamped, "/auv/pid/effective_goal", self._on_eff, 10)
        self.sub_src = self.create_subscription(
            String, "/auv/pid/goal_source", self._on_src, 10)
        self.sub_cmd = self.create_subscription(
            Float64MultiArray, "/auv/control/thruster_cmd", self._on_cmd, 10)
        self.cli_reset = self.create_client(Trigger,
                                            "/pid_controller_node/reset")
        self.eff = None
        self.src = None
        self.last_src_t = 0.0
        self.cmd_count = 0

    def _on_eff(self, m):
        self.eff = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                    m.header.frame_id,
                    m.pose.position.x, m.pose.position.y, m.pose.position.z)
        self.eff_msg = m

    def _on_src(self, m):
        self.src = m.data
        self.last_src_t = time.monotonic()

    def _on_cmd(self, m):
        self.cmd_count += 1

    def state(self, x, y=0.0, z=0.0, t=None):
        m = Odometry()
        m.header.stamp = (self.get_clock().now().to_msg() if t is None else t)
        m.header.frame_id = "auv_world"
        m.child_frame_id = "auv_com"
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.position.z = z
        m.pose.pose.orientation.w = 1.0
        self.pub_state.publish(m)

    def goal(self, x, y=0.0, z=0.0):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "auv_world"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        self.pub_goal.publish(m)


def spin_both(h, pid_node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(h, timeout_sec=0.01)
        rclpy.spin_once(pid_node, timeout_sec=0.01)


def main():
    rclpy.init()
    h = Harness()
    pid = PidControllerNode()

    # ---- T1: 无外部目标,hold 捕获并跟随当前位姿 ----
    spin_both(h, pid, 0.5)
    x = 1.0
    for _ in range(60):                     # 1.2 s 内 x 从 1.0 移到 2.2
        x += 0.02
        h.state(x)
        spin_both(h, pid, 0.02)
    spin_both(h, pid, 0.3)
    check("T1a goal_source=hold", h.src == "hold", "src=%r" % h.src)
    check("T1b effective_goal 已发布", h.eff is not None)
    if h.eff:
        check("T1c effective 跟随当前位姿(hold 语义)",
              abs(h.eff[2] - x) < 0.10, "eff.x=%.3f state.x=%.3f" % (h.eff[2], x))
        check("T1d frame=auv_world", h.eff[1] == "auv_world",
              "frame=%r" % h.eff[1])
        check("T1e 时间戳有效(跟随状态仿真时间)",
              h.eff[0] > 0.0, "stamp=%.3f" % h.eff[0])

    # ---- T2: 外部目标 → external,effective=外部值(非当前位姿转发) ----
    # (持续喂状态,保持新鲜;否则状态过期会正确地停发 effective)
    for i in range(40):
        h.state(2.2)
        if i == 0:
            h.goal(5.0, 1.0, 0.5)
        spin_both(h, pid, 0.02)
    spin_both(h, pid, 0.1)
    check("T2a goal_source=external", h.src == "external", "src=%r" % h.src)
    check("T2b effective=外部目标值",
          h.eff is not None and abs(h.eff[2] - 5.0) < 1e-6
          and abs(h.eff[3] - 1.0) < 1e-6 and abs(h.eff[4] - 0.5) < 1e-6,
          "eff=%r" % (h.eff,))

    # ---- T3: 外部目标变更 ----
    for i in range(40):
        h.state(2.2)
        if i == 0:
            h.goal(7.5, -2.0, 1.5)
        spin_both(h, pid, 0.02)
    spin_both(h, pid, 0.1)
    check("T3 effective 更新为新目标",
          h.eff is not None and abs(h.eff[2] - 7.5) < 1e-6
          and abs(h.eff[3] + 2.0) < 1e-6 and abs(h.eff[4] - 1.5) < 1e-6,
          "eff=%r" % (h.eff,))

    # ---- T4: reset → none(状态不新鲜时);恢复状态后 hold 重新捕获 ----
    # 注:状态持续新鲜时,reset 后下一拍(≤20ms)hold 立即重捕,"none"只在
    # 状态过期/hold_on_start=false 时可见——这是 reset 的文档语义。
    spin_both(h, pid, 0.7)                  # 先让状态过期
    h.cli_reset.call_async(Trigger.Request())
    spin_both(h, pid, 0.4)
    check("T4a 过期状态下 reset 后 goal_source=none", h.src == "none",
          "src=%r" % h.src)
    h.eff = None
    spin_both(h, pid, 0.3)
    check("T4b reset 后无 effective_goal(不冒充)", h.eff is None)
    for _ in range(30):
        h.state(3.0)
        spin_both(h, pid, 0.02)
    spin_both(h, pid, 0.3)
    check("T4c hold 重新捕获", h.src == "hold" and h.eff is not None
          and abs(h.eff[2] - 3.0) < 0.10, "src=%r eff=%r" % (h.src, h.eff))

    # ---- T5: 状态断流 → effective 停发,goal_source 持续 ----
    # 阶段A(0.8s):状态年龄超 0.6s 变过期,期间可能还有按旧鲜度的合法发布,不计
    spin_both(h, pid, 0.8)
    # 阶段B(0.8s):已确定过期,此窗口内不允许出现任何 effective_goal
    h.eff = None
    src_seen = False
    deadline = time.monotonic() + 0.8
    while time.monotonic() < deadline:
        rclpy.spin_once(h, timeout_sec=0.02)
        rclpy.spin_once(pid, timeout_sec=0.02)
        if h.src is not None:
            src_seen = True
    check("T5a 断流后无 effective_goal(过期不冒充)", h.eff is None)
    check("T5b goal_source 仍在发布", src_seen)

    # ---- T6: 急停 → 停发(状态保持新鲜,排除 T5 因素);解除恢复 ----
    m = Bool(); m.data = True
    h.pub_estop.publish(m)
    for _ in range(35):
        h.state(3.0)
        spin_both(h, pid, 0.02)
    h.eff = None
    for _ in range(35):
        h.state(3.0)
        spin_both(h, pid, 0.02)
    check("T6a 急停后无 effective_goal(状态新鲜仍停发)", h.eff is None)
    m = Bool(); m.data = False
    h.pub_estop.publish(m)
    for _ in range(40):
        h.state(3.0)
        spin_both(h, pid, 0.02)
    check("T6b 解除后恢复发布", h.eff is not None, "eff=%r" % (h.eff,))
    check("T6c 控制命令持续输出(监测未阻塞控制)", h.cmd_count > 50,
          "cmd=%d" % h.cmd_count)

    ok = all(c for _, c in RESULTS)
    print("\n%d/%d checks passed" % (sum(c for _, c in RESULTS), len(RESULTS)),
          flush=True)
    if ok:
        print("ALL MONITOR TESTS PASSED", flush=True)
    sys.stdout.flush()
    os._exit(0 if ok else 1)


if __name__ == "__main__":
    main()
