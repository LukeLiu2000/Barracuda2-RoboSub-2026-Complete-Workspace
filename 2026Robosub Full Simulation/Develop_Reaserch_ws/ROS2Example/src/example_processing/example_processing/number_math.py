"""number_math —— 纯算法模块:平方计算与 Int64 范围检查。

本模块不导入 rclpy、不创建节点,可以在没有 ROS 运行环境的地方
(单元测试、普通 Python)直接使用和测试。
"""

# std_msgs/msg/Int64 是有符号 64 位整数
INT64_MAX = 2 ** 63 - 1
INT64_MIN = -(2 ** 63)


def checked_square(n: int):
    """计算平方并检查范围。

    返回 (n*n, True) —— 平方在 Int64 范围内;
    返回 (None, False) —— 平方超出范围,调用方应跳过发布而不是崩溃。
    """
    s = n * n  # Python 的 int 无限精度,不会悄悄溢出,所以可以放心先算再比
    if s > INT64_MAX or s < INT64_MIN:
        return None, False
    return s, True
