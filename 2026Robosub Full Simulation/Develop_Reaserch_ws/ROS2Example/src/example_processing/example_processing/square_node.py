#!/usr/bin/env python3
"""square_node —— 订阅 count,计算平方后发布到 square(均为相对话题名)。

包:example_processing(ament_python)。平方计算与范围检查放在
number_math.py(纯算法模块)里,本文件只做 ROS 通信与日志。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Int64

from .number_math import checked_square


def make_demo_qos() -> QoSProfile:
    """教学示例统一 QoS:KEEP_LAST / depth=10 / RELIABLE / VOLATILE。"""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class SquareNode(Node):
    def __init__(self) -> None:
        super().__init__('square_node')
        qos = make_demo_qos()
        self._pub = self.create_publisher(Int64, 'square', qos)
        self._sub = self.create_subscription(Int64, 'count', self._on_count, qos)
        self.get_logger().info('square_node 已启动,等待 count 消息...')

    def _on_count(self, msg: Int64) -> None:
        n = msg.data
        result, ok = checked_square(n)  # 算法在 number_math,可单测
        if not ok:
            # 超出 Int64 范围:记录 Warning 并跳过发布,节点不崩溃
            self.get_logger().warning(f'平方超出 Int64 范围,跳过发布: n={n}')
            return
        out = Int64()
        out.data = result
        self._pub.publish(out)
        self.get_logger().info(f'收到 n={n},发布 n^2={result}')


def main(args=None):
    rclpy.init(args=args)          # 初始化 rclpy 上下文(进程级,一次)
    node = SquareNode()
    try:
        rclpy.spin(node)           # 阻塞分发回调,直到 Ctrl+C
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()        # 销毁节点,释放资源
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
