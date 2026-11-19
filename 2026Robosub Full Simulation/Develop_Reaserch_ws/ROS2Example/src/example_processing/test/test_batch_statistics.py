"""BatchStatistics 单元测试:边界、批次清空与实例独立性(纯算法,无需 ROS)。

写成 unittest 风格,pytest 也能直接运行。
"""

import unittest

from example_processing.batch_statistics import BatchStatistics, format_report


class TestBatchStatisticsConstruction(unittest.TestCase):
    def test_invalid_batch_size(self):
        for bad in (0, -1, -5, 2.5, '5', None, True):
            with self.assertRaises(ValueError):
                BatchStatistics(bad)

    def test_valid_batch_size(self):
        self.assertEqual(BatchStatistics(1).batch_size, 1)
        self.assertEqual(BatchStatistics(5).batch_size, 5)


class TestBatchStatisticsBehavior(unittest.TestCase):
    def test_less_than_batch_size_no_report(self):
        s = BatchStatistics(5)
        for v in (1, 2, 3, 4):
            self.assertIsNone(s.add(v))
        self.assertEqual(s.pending, 4)

    def test_exactly_batch_size_reports_correct_stats(self):
        s = BatchStatistics(5)
        for v in (1, 2, 3, 4):
            self.assertIsNone(s.add(v))
        report = s.add(5)  # 第 5 条凑满
        self.assertIsNotNone(report)
        self.assertEqual(report['batch'], 1)
        self.assertEqual(report['count'], 5)
        self.assertEqual(report['sum'], 15)
        self.assertEqual(report['min'], 1)
        self.assertEqual(report['max'], 5)
        self.assertEqual(s.pending, 0)  # 缓冲已清空

    def test_negative_values_stats(self):
        s = BatchStatistics(3)
        self.assertIsNone(s.add(-5))
        self.assertIsNone(s.add(0))
        report = s.add(5)
        self.assertEqual((report['sum'], report['min'], report['max']),
                         (0, -5, 5))

    def test_next_batch_not_mixed_with_previous(self):
        s = BatchStatistics(2)
        self.assertIsNone(s.add(10))
        first = s.add(10)  # 第 2 条凑满第一批
        self.assertEqual(first, {'batch': 1, 'count': 2, 'sum': 20,
                                 'min': 10, 'max': 10})
        second = s.add(3)  # 新一批的第一条
        self.assertIsNone(second)
        third = s.add(4)   # 新一批凑满
        self.assertEqual(third['batch'], 2)
        self.assertEqual(third['sum'], 7)      # 不含上一批的 10
        self.assertEqual((third['min'], third['max']), (3, 4))

    def test_batch_number_increments(self):
        s = BatchStatistics(1)
        self.assertEqual(s.add(9)['batch'], 1)
        self.assertEqual(s.add(8)['batch'], 2)
        self.assertEqual(s.add(7)['batch'], 3)

    def test_two_instances_independent(self):
        a = BatchStatistics(2)
        b = BatchStatistics(2)
        self.assertIsNone(a.add(100))          # 只喂 a
        self.assertEqual(a.pending, 1)
        self.assertEqual(b.pending, 0)         # b 不受 a 影响
        r_b = None
        self.assertIsNone(b.add(1))
        r_b = b.add(2)
        self.assertEqual(r_b['sum'], 3)        # b 的统计不含 a 的 100
        r_a = a.add(200)
        self.assertEqual(r_a['sum'], 300)      # a 的统计不含 b 的 1、2

    def test_format_report(self):
        text = format_report({'batch': 2, 'count': 5, 'sum': 135,
                              'min': 9, 'max': 49})
        self.assertEqual(text, 'batch=2 count=5 sum=135 min=9 max=49')


if __name__ == '__main__':
    unittest.main()
