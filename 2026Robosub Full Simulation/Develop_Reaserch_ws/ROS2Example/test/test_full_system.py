#!/usr/bin/env python3
"""工作区级集成测试(独立脚本,不属于任何包,colcon test 不会发现它)。

执行方式(必须先加载工作区环境):
    cd <工作区根目录>
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 test/test_full_system.py

验证内容:
  阶段 A(全系统):四个业务节点均启动;三条话题类型正确;
      count 递增流、square 与 count 的对应关系、feedback 批次格式。
  阶段 B(受控输入):只用 processing 包,由测试发布指定的 count 序列,
      精确验证平方值序列与批次统计(不依赖默认计数器首发 0,
      且先发一个超 Int64 范围的值验证"跳过发布、不崩溃")。

证据来源是订阅到的消息流本身,不靠日志文本匹配。
所有日志/缓存(ROS_LOG_DIR、PYTHONPYCACHEPREFIX)都留在工作区内。
"""

import os
import random
import re
import signal
import subprocess
import sys
import time
import traceback

WS_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
RUNTIME_LOGS = os.path.join(WS_ROOT, 'runtime_logs')
VERIFICATION = os.path.join(WS_ROOT, '_verification')

# 让节点日志与 Python 缓存都落在本工作区,不污染 HOME
os.environ.setdefault('ROS_LOG_DIR', RUNTIME_LOGS)
os.environ.setdefault('PYTHONPYCACHEPREFIX',
                      os.path.join(VERIFICATION, 'pycache'))
os.makedirs(RUNTIME_LOGS, exist_ok=True)
os.makedirs(VERIFICATION, exist_ok=True)

import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy)
from std_msgs.msg import Int64, String  # noqa: E402

FEEDBACK_RE = re.compile(
    r'^batch=(\d+) count=(\d+) sum=(-?\d+) min=(-?\d+) max=(-?\d+)$')

# 用独立的 Domain ID 隔离测试,避免与机器上其他 ROS 会话互相串扰
DOMAIN = random.randint(33, 90)
os.environ['ROS_DOMAIN_ID'] = str(DOMAIN)


def make_qos() -> QoSProfile:
    """与业务节点一致的 QoS。"""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class Recorder(Node):
    """测试订阅端(独立于四个业务节点,不参与业务)。"""

    def __init__(self, ns: str):
        super().__init__('integration_test_checker')
        qos = make_qos()
        self.counts = []
        self.squares = []   # (到达时刻, 数值)
        self.feedbacks = []  # (到达时刻, 文本)
        self.create_subscription(
            Int64, f'{ns}/count',
            lambda m: self.counts.append(m.data), qos)
        self.create_subscription(
            Int64, f'{ns}/square',
            lambda m: self.squares.append((time.time(), m.data)), qos)
        self.create_subscription(
            String, f'{ns}/feedback',
            lambda m: self.feedbacks.append((time.time(), m.data)), qos)


class Launcher:
    """子进程启动与可靠清理(进程组方式)。

    `ros2 launch` 只被 SIGTERM 杀掉时会留下孤儿节点进程,
    因此用 start_new_session 让每次启动独占一个进程组,
    清理时先对整组发 SIGINT(等价 Ctrl+C 的优雅关闭),
    超时再对整组 SIGKILL,确保不留残留进程。
    """

    def __init__(self):
        self.env = os.environ.copy()
        self.procs = []   # (Popen, 日志文件对象)

    def launch(self, args, log_name):
        log = open(os.path.join(RUNTIME_LOGS, log_name), 'w',
                   encoding='utf-8')
        log.write(f'# launch: {args}\n')
        log.flush()
        proc = subprocess.Popen(args, cwd=WS_ROOT, env=self.env,
                                stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        self.procs.append((proc, log))
        return proc

    def _killpg(self, pid, sig):
        try:
            os.killpg(pid, sig)
        except ProcessLookupError:
            pass

    def cleanup(self):
        # 第一轮:整组 SIGINT(ros2 launch 会优雅结束所有子节点)
        for proc, _ in self.procs:
            if proc.poll() is None:
                self._killpg(proc.pid, signal.SIGINT)
        # 等待退出,超时的组补 SIGKILL
        deadline = time.time() + 8
        for proc, log in self.procs:
            try:
                proc.wait(timeout=max(0.2, deadline - time.time()))
            except subprocess.TimeoutExpired:
                self._killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=10)
            log.close()
        self.procs = []


def spin_until(node, done, timeout):
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if done():
            return True
    return False


def check(fails, ok, message):
    tag = 'PASS' if ok else 'FAIL'
    print(f'  [{tag}] {message}')
    if not ok:
        fails.append(message)


def ros2_output(args):
    return subprocess.run(args, cwd=WS_ROOT, env=os.environ.copy(),
                          capture_output=True, text=True, timeout=30).stdout


def phase_a(fails):
    print('== 阶段 A:全系统(all.launch.py,namespace=demo)==')
    launcher = Launcher()
    try:
        launcher.launch(['ros2', 'launch',
                         os.path.join(WS_ROOT, 'launch', 'all.launch.py')],
                        'itest_phase_a_launch.log')

        rclpy.init()
        rec = Recorder('/demo')
        try:
            # 先让订阅端完成发现,再等数据;总限时 20 秒
            ok = spin_until(rec,
                            lambda: len(rec.counts) >= 8
                            and len(rec.feedbacks) >= 2, 20.0)

            # 1) 四个业务节点都出现在图中(命名空间 /demo)
            names = {(n, ns) for n, ns in rec.get_node_names_and_namespaces()}
            for expected in (('counter_node', '/demo'),
                             ('square_node', '/demo'),
                             ('report_node', '/demo'),
                             ('feedback_node', '/demo')):
                check(fails, expected in names,
                      f'节点已启动: {expected[1]}/{expected[0]}')

            # 2) 三条话题类型正确(ros2 topic list -t 的输出为证)
            topic_text = ros2_output(['ros2', 'topic', 'list', '-t'])
            for line in ('/demo/count [std_msgs/msg/Int64]',
                         '/demo/square [std_msgs/msg/Int64]',
                         '/demo/feedback [std_msgs/msg/String]'):
                check(fails, line in topic_text, f'话题类型: {line}')

            # 3) count 流:至少 8 条、严格递增且步长为 1
            check(fails, len(rec.counts) >= 8,
                  f'count 消息数 >= 8(实际 {len(rec.counts)} 条)')
            if rec.counts:
                diffs = {b - a for a, b in zip(rec.counts, rec.counts[1:])}
                check(fails, rec.counts == sorted(rec.counts)
                      and diffs <= {1},
                      f'count 严格递增且步长=1'
                      f'(首条 {rec.counts[0]},末条 {rec.counts[-1]})')
            else:
                check(fails, False, 'count 为空')

            # 4) 每条 square 都等于某条 count 的平方(消息级证据)
            if rec.squares:
                known = {n * n for n in rec.counts}
                bad = [v for _, v in rec.squares if v not in known]
                check(fails, not bad,
                      f'square 均与 count 对应(收到 {len(rec.squares)} 条)')
            else:
                check(fails, False, 'square 为空')

            # 5) feedback 批次格式、batch_size=5、批次号递增
            parsed = []
            for _, text in rec.feedbacks:
                m = FEEDBACK_RE.match(text.strip())
                if m:
                    parsed.append(tuple(int(g) for g in m.groups()))
            check(fails, len(parsed) == len(rec.feedbacks) and len(parsed) >= 2,
                  f'feedback 可解析且 >= 2 批(实际 {len(parsed)} 批)')
            if parsed:
                batches = [p[0] for p in parsed]
                check(fails, batches == sorted(batches)
                      and all(p[1] == 5 for p in parsed),
                      'feedback 批次号递增且 count=5')
                # 若首批 5 条 square 在第一条 feedback 前都收到,可精确对账
                if len(rec.feedbacks) >= 1:
                    t0 = rec.feedbacks[0][0]
                    first5 = [v for t, v in rec.squares if t < t0][:5]
                    if len(first5) == 5:
                        b, c, s, lo, hi = parsed[0]
                        check(fails,
                              (c, s, lo, hi) == (5, sum(first5),
                                                 min(first5), max(first5)),
                              f'首批统计与实测 5 条 square 一致'
                              f'(sum={sum(first5)}, min={min(first5)}, '
                              f'max={max(first5)})')
            check(fails, ok, '20 秒内收集到足够数据(8 count / 2 feedback)')
        finally:
            rec.destroy_node()
            rclpy.shutdown()
    finally:
        launcher.cleanup()


def phase_b(fails):
    print('== 阶段 B:受控输入(processing 包,namespace=ctrl)==')
    launcher = Launcher()
    try:
        share = get_package_share_directory('example_processing')
        launcher.launch(['ros2', 'launch',
                         os.path.join(share, 'launch',
                                      'processing.launch.py'),
                         'namespace:=ctrl'],
                        'itest_phase_b_launch.log')

        rclpy.init()
        node = Node('integration_test_driver')
        squares = []
        feedbacks = []
        pub = node.create_publisher(Int64, '/ctrl/count', make_qos())
        node.create_subscription(Int64, '/ctrl/square',
                                 lambda m: squares.append(m.data), make_qos())
        node.create_subscription(String, '/ctrl/feedback',
                                 lambda m: feedbacks.append(m.data),
                                 make_qos())
        try:
            # 等两个业务节点出现在图中(限时 15 秒)
            ok = spin_until(
                node,
                lambda: {n for n, ns in
                         node.get_node_names_and_namespaces()
                         if ns == '/ctrl'} >= {'square_node', 'report_node'},
                15.0)
            check(fails, ok, 'square_node 与 report_node 已启动(/ctrl)')

            # 受控输入:先发一个平方必然超界的值,再发 3..7
            # —— 不从 0 开始(不依赖默认计数器首发 0)
            seq = [3037000500, 3, 4, 5, 6, 7]
            for value in seq:
                msg = Int64()
                msg.data = value
                pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.3)

            # 等收尾(第 5 条有效 square 凑满一批 -> 一条 feedback)
            spin_until(node, lambda: len(feedbacks) >= 1, 6.0)

            check(fails, squares == [9, 16, 25, 36, 49],
                  f'square 序列精确为 [9,16,25,36,49]'
                  f'(实际 {squares};超界值被跳过且节点未崩溃)')
            if feedbacks:
                m = FEEDBACK_RE.match(feedbacks[0].strip())
                check(fails, m is not None,
                      f'feedback 可解析(实际: {feedbacks[0]!r})')
                if m:
                    b, c, s, lo, hi = (int(g) for g in m.groups())
                    check(fails, (b, c, s, lo, hi) == (1, 5, 135, 9, 49),
                          '首批统计正确: batch=1 count=5 sum=135 '
                          'min=9 max=49')
            else:
                check(fails, False, '未收到 feedback')
        finally:
            node.destroy_node()
            rclpy.shutdown()
    finally:
        launcher.cleanup()


def main():
    # 前置:两个包必须已安装并被 source
    for pkg in ('example_source', 'example_processing'):
        try:
            get_package_share_directory(pkg)
        except Exception:
            print(f'找不到包 {pkg}:请先构建并 source install/setup.bash')
            return 2

    print(f'工作区: {WS_ROOT}')
    print(f'ROS_DOMAIN_ID={os.environ["ROS_DOMAIN_ID"]}(测试专用,自动隔离)')
    print(f'日志目录: {RUNTIME_LOGS}')

    fails = []
    for phase in (phase_a, phase_b):
        try:
            phase(fails)
        except Exception:
            traceback.print_exc()
            fails.append(f'{phase.__name__} 异常退出')

    print('\n===== 结果 =====')
    if fails:
        print(f'失败 {len(fails)} 项:')
        for f in fails:
            print(f'  - {f}')
        return 1
    print('全部通过。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
