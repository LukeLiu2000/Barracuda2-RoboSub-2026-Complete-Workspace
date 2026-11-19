"""batch_statistics —— 纯算法模块:把顺序到达的数值按固定长度分批统计。

不导入 rclpy、不创建节点;节点类只负责 ROS 通信,统计逻辑全部在这里,
因此可以用普通单元测试覆盖(边界、清空、实例独立性)。
"""


class BatchStatistics:
    """每个实例独立维护自己的批次状态,两个实例互不影响。"""

    def __init__(self, batch_size: int):
        # 非法 batch_size 在构造时就报错,不带病运行
        if isinstance(batch_size, bool) or not isinstance(batch_size, int):
            raise ValueError(f'batch_size 必须是整数,收到: {batch_size!r}')
        if batch_size <= 0:
            raise ValueError(f'batch_size 必须为正整数,收到: {batch_size}')

        self.batch_size = batch_size
        self._buffer = []      # 当前未满批次的数值
        self._next_batch = 1   # 下一个将被完成的批次编号(从 1 开始)

    def add(self, value: int):
        """追加一条数值。

        凑满一批时返回统计 dict 并清空缓冲;否则返回 None。
        批次只由“实际收到的消息”决定:不足 batch_size 就一直等,
        不假定收到 0,也不假定消息永不丢失。
        """
        self._buffer.append(value)
        if len(self._buffer) < self.batch_size:
            return None

        report = {
            'batch': self._next_batch,
            'count': len(self._buffer),
            'sum': sum(self._buffer),
            'min': min(self._buffer),
            'max': max(self._buffer),
        }
        self._next_batch += 1
        self._buffer = []  # 清空:下一批不混入本批数据
        return report

    @property
    def pending(self) -> int:
        """当前批次已积攒、尚未出报告的条数。"""
        return len(self._buffer)


def format_report(report: dict) -> str:
    """把统计 dict 格式成稳定、易解析的文本(发布到 feedback 话题)。"""
    return (
        f"batch={report['batch']} count={report['count']} "
        f"sum={report['sum']} min={report['min']} max={report['max']}"
    )
