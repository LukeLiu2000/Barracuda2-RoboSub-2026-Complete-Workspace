#!/usr/bin/env python3
"""report_node —— 订阅 square,每凑满 batch_size 条生成一次批次统计,
以 String 发布到相对话题 feedback,并打印同样内容。

统计逻辑在 batch_statistics.py(纯算法模块),本文件只做 ROS 通信与日志。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Int64, String

from .batch_statistics import BatchStatistics, format_report
from .square_node import make_demo_qos


class ReportNode(Node):
    def __init__(self) -> None:
        super().__init__('report_node')

        # 参数声明与读取(可被 YAML / 命令行覆盖)
        self.declare_parameter('batch_size', 5)
        batch_size = self.get_parameter('batch_size').value

        try:
            # 非法值(非正整数)在构造期直接报错退出,不带病运行
            self._stats = BatchStatistics(batch_size)
        except ValueError as exc:
            self.get_logger().error(f'参数 batch_size 非法: {exc}')
            raise

        qos = make_demo_qos()
        self._sub = self.create_subscription(Int64, 'square', self._on_square, qos)
        self._pub = self.create_publisher(String, 'feedback', qos)
        self.get_logger().info(f'report_node 已启动: batch_size={batch_size}')

    def _on_square(self, msg: Int64) -> None:
        # 凑满一批返回统计 dict,不足一批返回 None(继续等待,不报错)
        report = self._stats.add(msg.data)
        if report is None:
            return

        text = format_report(report)
        out = String()
        out.data = text
        self._pub.publish(out)      # 发布汇总文本
        self.get_logger().info(text)  # 打印同样的内容


def main(args=None):
    rclpy.init(args=args)
    node = ReportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
