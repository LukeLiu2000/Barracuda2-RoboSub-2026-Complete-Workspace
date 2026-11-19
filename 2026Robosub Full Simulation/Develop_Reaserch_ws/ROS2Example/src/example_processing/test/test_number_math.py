"""number_math 单元测试:平方与 Int64 范围检查(纯算法,无需 ROS)。"""

from example_processing.number_math import (INT64_MAX, INT64_MIN,
                                            checked_square)


class TestCheckedSquare:
    def test_zero(self):
        assert checked_square(0) == (0, True)

    def test_positive(self):
        assert checked_square(7) == (49, True)

    def test_negative(self):
        assert checked_square(-9) == (81, True)

    def test_one(self):
        assert checked_square(1) == (1, True)
        assert checked_square(-1) == (1, True)

    def test_boundary_largest_in_range(self):
        # floor(sqrt(2^63-1)) = 3037000499,其平方是 Int64 内最大的完全平方
        n = 3037000499
        s = n * n
        assert s <= INT64_MAX
        assert checked_square(n) == (s, True)

    def test_boundary_just_over(self):
        # 再大 1,平方就越界
        assert checked_square(3037000500) == (None, False)

    def test_int64_extremes_overflow(self):
        assert checked_square(INT64_MAX) == (None, False)
        assert checked_square(INT64_MIN) == (None, False)

    def test_negative_boundary(self):
        n = -3037000499
        assert checked_square(n) == (n * n, True)
        assert checked_square(-3037000500) == (None, False)
