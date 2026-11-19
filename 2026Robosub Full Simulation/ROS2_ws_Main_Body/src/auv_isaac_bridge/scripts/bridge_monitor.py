#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bridge_monitor —— 服务器侧(系统 ROS 2)桥状态监视节点。

打印三类信息(1 Hz 单行,便于 SSH 终端观察闭环):
  - /auv/state/control  最新真值:位置/姿态(rpy 仅显示)/体速度 + 数据年龄
  - /auv/control/thruster_cmd 频率(如控制器在发)
  - /auv/bridge/status  桥自报状态字段(mode/estop/cmd_rate/force_scale...)

欧拉角仅用于人眼阅读,约定 roll 绕 z / pitch 绕 x / yaw 绕 y(体轴
+x 左舷 +y 上 +z 前);四元数才是权威。
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String
# 注:/auv/bridge/status 现为 std_msgs/String(JSON)。
# 原因:Isaac Sim 5.1.0 的 isaacsim.ros2.bridge 4.12.4 发布 DiagnosticArray
# 会在其 C 层断言失败(PyBytes_Check)直接 abort 整个 kit 进程,故桥端弃用。


def quat_to_rpy_body(q):
    """体约定下的显示欧拉角:roll 绕 z,pitch 绕 x,yaw 绕 y。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + x * x))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * x - y * z))))
    yaw = math.atan2(2 * (w * y + z * x), 1 - 2 * (y * y + x * x))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class BridgeMonitor(Node):

    def __init__(self):
        super().__init__("auv_bridge_monitor")
        q = qos_profile_sensor_data
        self._odom = None
        self._odom_t = 0.0
        self._cmd_n = 0
        self._cmd_t = time.monotonic()
        self._cmd_hz = 0.0
        self._last_cmd = None
        self._bridge = {}
        self.create_subscription(Odometry, "/auv/state/control",
                                 self._on_odom, q)
        self.create_subscription(Float64MultiArray, "/auv/control/thruster_cmd",
                                 self._on_cmd, q)
        self.create_subscription(String, "/auv/bridge/status",
                                 self._on_status, 10)
        self._timer = self.create_timer(1.0, self._tick)

    def _on_odom(self, msg):
        self._odom = msg
        self._odom_t = time.monotonic()

    def _on_cmd(self, msg):
        now = time.monotonic()
        self._cmd_n += 1
        if now - self._cmd_t >= 1.0:
            self._cmd_hz = self._cmd_n / (now - self._cmd_t)
            self._cmd_n = 0
            self._cmd_t = now
        self._last_cmd = list(msg.data)

    def _on_status(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        kv = {"mode": st.get("mode"), "estop": st.get("estop")}
        kv.update({k: v for k, v in st.get("stage", {}).items()})
        self._bridge = kv

    def _tick(self):
        parts = []
        if self._odom is not None:
            age = time.monotonic() - self._odom_t
            p = self._odom.pose.pose.position
            r, pi_, y = quat_to_rpy_body(self._odom.pose.pose.orientation)
            v = self._odom.twist.twist.linear
            w = self._odom.twist.twist.angular
            parts.append("age=%.2fs p=[%+.2f %+.2f %+.2f] rpy=[%+.1f %+.1f %+.1f] "
                         "deg v=[%+.2f %+.2f %+.2f] w=[%+.2f %+.2f %+.2f]" % (
                             age, p.x, p.y, p.z, r, pi_, y,
                             v.x, v.y, v.z, w.x, w.y, w.z))
        else:
            parts.append("无 /auv/state/control 数据")
        if self._last_cmd is not None:
            parts.append("cmd %.1fHz u=[%s]" % (
                self._cmd_hz, " ".join("%+.1f" % u for u in self._last_cmd)))
        kv = self._bridge
        if kv:
            parts.append("bridge[%s] estop=%s fscale=%s %s" % (
                kv.get("mode", "?"), kv.get("estop", "?"),
                kv.get("force_scale", "?"), kv.get("geom_source", "")))
        self.get_logger().info(" | ".join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = BridgeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
