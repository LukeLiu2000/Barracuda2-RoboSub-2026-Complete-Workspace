#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hold_node —— 基础保持模式(闭环验收的第一步)。

第一个有效状态到达时,目标 := 当前位姿,之后以 2 Hz 持续重发同一目标
(控制器 goal_timeout 内必须刷新)。目标冻结在捕获点,不跟随当前位置
漂移 —— 这正是"保持"的语义;艇被扰动后控制器会把它拉回捕获点。

服务 ~/recapture(std_srvs/SetBool):重新以当前位姿为保持目标。
话题:订阅 /auv/state/control,发布 /auv/control/goal(与键盘/训练任务
同源,互斥使用,统一 launch 保证不同时启动)。
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool


class HoldNode(Node):

    def __init__(self):
        super().__init__("auv_hold_node")
        self.declare_parameter("hold.state_topic", "/auv/state/control")
        self.declare_parameter("hold.goal_topic", "/auv/control/goal")
        self.declare_parameter("hold.republish_hz", 2.0)
        self.declare_parameter("hold.state_timeout_s", 0.5)
        topic = self.get_parameter("hold.state_topic").value
        hz = float(self.get_parameter("hold.republish_hz").value)
        self.timeout = float(self.get_parameter("hold.state_timeout_s").value)

        self.goal = None            # (p, q) 冻结目标
        self.frame = "auv_world"
        self.last_pub = 0.0
        self.pub = self.create_publisher(
            PoseStamped, self.get_parameter("hold.goal_topic").value, 10)
        self.create_subscription(Odometry, topic, self._on_state,
                                 qos_profile_sensor_data)
        self.create_service(SetBool, "~/recapture", self._on_recapture)
        self.create_timer(1.0 / max(0.5, hz), self.tick)
        self.get_logger().info("hold 节点就绪:首个有效状态到达即捕获为保持目标")

    def _on_state(self, msg):
        pp = msg.pose.pose.position
        qq = msg.pose.pose.orientation
        if msg.header.frame_id:
            self.frame = msg.header.frame_id
        if self.goal is None:
            self.goal = ((pp.x, pp.y, pp.z), (qq.x, qq.y, qq.z, qq.w))
            self.get_logger().info(
                "已捕获保持目标 pos=(%.2f,%.2f,%.2f)" % (pp.x, pp.y, pp.z))

    def _on_recapture(self, req, res):
        # 立刻用最新一帧状态重捕获(下次状态回调里如果 goal 为 None 才捕;
        # 这里直接把 goal 置空,让下一帧状态触发捕获)
        self.goal = None
        res.success = True
        res.message = "recapture: 将在下一帧状态捕获"
        return res

    def tick(self):
        if self.goal is None:
            return
        now = time.monotonic()
        if now - self.last_pub < 0.4:
            return
        self.last_pub = now
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = self.frame
        p, q = self.goal
        g.pose.position.x, g.pose.position.y, g.pose.position.z = p
        g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, \
            g.pose.orientation.w = q
        self.pub.publish(g)


def main(args=None):
    rclpy.init(args=args)
    node = HoldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
