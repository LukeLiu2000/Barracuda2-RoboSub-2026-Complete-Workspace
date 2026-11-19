#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ZED 采集节点:从 ZED SDK 读 Isaac Sim 里的虚拟 ZED 相机,发布 ROS 2 话题。

数据链路(实测 2026-10-31,见 docs/environment_report.md):

    Isaac Sim 虚拟双目相机(ZED_X prim,含物理 IMU)
      -> sl.sensor.camera 扩展(场景 Action Graph 里的 ZED Camera 节点)
      -> ZED SDK 流(本机 IPC,服务端 127.0.0.1:30000,默认 BOTH 模式)
      -> 本节点(pyzed 客户端,grab 拉流)
      -> ROS 2 话题 /auv/zed/...

关键事实:
    1. SDK 返回的图像时间戳就是 Isaac 仿真时间(纳秒),因为扩展内部把
       IsaacReadSimulationTime 接进了流。所以本节点直接用数据时间戳打
       header.stamp,与 /clock(若场景发布了)天然同源。
    2. 流的帧率由场景里 ZED 节点的 fps 参数决定(当前 60 Hz),仿真
       暂停时不会有新帧 —— 本节点在暂停期间不发任何消息,绝不重复发
       旧数据凑频率。
    3. 左右图像已经是立体校正后的图像,所以话题名用 image_rect。
    4. 深度/点云是 SDK 立体匹配的"视觉计算值",不是仿真真值;两者
       的区别见 docs/interfaces.md。
    5. 发布开关(publish_left_image/publish_right_image/publish_depth/
       publish_points/publish_imu):关闭 = 不创建 publisher、跳过
       retrieve/转换/构造;深度+点云都关时 SDK 以 DEPTH_MODE.NONE
       打开(跳过 SDK 端立体匹配)。SDK 内部仍存在的计算(无法从
       本节点关闭):Isaac 侧 sl.sensor.camera 扩展仍按场景 fps 渲染
       双目图像并经 IPC 推流,grab() 仍要接收解码这些帧(IMU 数据与
       时间戳也依赖 grab)。要彻底停渲染需改场景端 Action Graph
       (不在本节点职责内)。
"""

import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2

from auv_sensing.zed_converters import (
    bgra_to_image_msg,
    camera_info_msg,
    depth_to_image_msg,
    imu_msg,
    stamp_from_ns,
    xyzrgba_to_pointcloud2_msg,
)

# pyzed 只在服务器上存在;导入失败时给用户明确提示而不是崩溃栈。
try:
    import pyzed.sl as sl
    PYZED_AVAILABLE = True
except ImportError:
    PYZED_AVAILABLE = False


def _pyzed_matrix3_to_list(obj):
    """pyzed 5.4 的 Matrix3f(协方差对象)不能直接迭代——实测
    [float(v) for v in obj] 会抛 'int' object is not subscriptable。
    这里按优先级转换成 9 元列表(按行展开):
    1) get_data() 缓冲;2) 解析 repr 里的数字,过滤掉十六进制
    地址等非数字 token,以及明显不可能是协方差的超大数值。
    任何一步失败都返回 None,交由 imu_msg 按"未知(全 0)"处理。"""
    if obj is None:
        return None
    try:
        arr = np.asarray(obj.get_data(), dtype=np.float64).reshape(-1)
        if arr.size >= 9:
            return [float(v) for v in arr[:9]]
    except Exception:
        pass
    vals = []
    try:
        for tok in str(obj).replace(',', ' ').split():
            try:
                v = float(tok)
            except ValueError:
                continue          # 地址等非数字 token,直接丢弃
            if abs(v) < 1e5:      # 协方差不会是天文数字,滤掉误解析
                vals.append(v)
        if len(vals) >= 9:
            return vals[:9]
    except Exception:
        pass
    return None


class ZedSensorNode(Node):
    """ZED 虚拟相机 -> ROS 2。

    线程模型(初学者注解):
    - 主线程:rclpy.spin 驱动回调(目前只有 /clock 诊断订阅);
    - _grab_thread:独占 zed 相机对象,循环 grab -> 按限频发布;
      rclpy 的 publisher.publish 本身是线程安全的,可以跨线程调用;
    - _summary_thread:墙钟 1 Hz 打一行摘要(连接状态/频率/IMU 状态)。
      用墙钟而不是节点时钟,是为了在仿真暂停、use_sim_time 又开着、
      /clock 停走的情况下摘要也仍然活着。
    """

    def __init__(self):
        super().__init__('zed_sensor_node')

        # ---------------- 参数(带默认值,可被 config/sensing.yaml 覆盖) -------------
        self.declare_parameter('stream_ip', '127.0.0.1')      # ZED 流服务端地址(本机 IPC)
        self.declare_parameter('stream_port', 30000)          # 流服务端口(场景 ZED 节点配置)
        self.declare_parameter('depth_mode', 'PERFORMANCE')   # SDK 深度模式
        self.declare_parameter('open_retry_s', 5.0)           # 连接失败后的重试间隔
        self.declare_parameter('image_rate_hz', 15.0)         # 左目(和右目)图像发布频率
        # ---- 发布开关(闭环联调只需 IMU 时,把图像/深度/点云全部关掉) ----
        # 关闭 = 完全跳过该通道的 retrieve/转换/消息构造,且不创建 publisher
        # (话题直接不存在,ros2 topic list 可验证)。CameraInfo 跟随对应图像。
        self.declare_parameter('publish_left_image', True)    # 左目图像 + 左 CameraInfo
        self.declare_parameter('publish_right_image', True)   # 右目图像 + 右 CameraInfo
        self.declare_parameter('publish_depth', True)         # 深度图
        self.declare_parameter('publish_points', True)        # 点云
        self.declare_parameter('publish_imu', True)           # IMU(闭环/状态估计需要)
        self.declare_parameter('depth_rate_hz', 10.0)         # 深度图发布频率
        self.declare_parameter('points_rate_hz', 5.0)         # 点云发布频率
        self.declare_parameter('points_max_points', 200000)   # 点云抽稀上限
        self.declare_parameter('left_frame', 'zed_left_camera_frame')
        self.declare_parameter('right_frame', 'zed_right_camera_frame')
        self.declare_parameter('depth_frame', 'zed_left_camera_frame')   # 深度对齐左目
        self.declare_parameter('points_frame', 'zed_left_camera_frame')  # 点云在左目光学系
        self.declare_parameter('imu_frame', 'zed_imu_link')
        self.declare_parameter('summary_period_s', 1.0)       # 终端摘要周期

        self._p = {k: self.get_parameter(k).value for k in (
            'stream_ip', 'stream_port', 'depth_mode', 'open_retry_s',
            'image_rate_hz', 'publish_left_image', 'publish_right_image',
            'publish_depth', 'publish_points', 'publish_imu',
            'depth_rate_hz', 'points_rate_hz', 'points_max_points',
            'left_frame', 'right_frame', 'depth_frame', 'points_frame',
            'imu_frame', 'summary_period_s')}

        # ---------------- Publisher ------------------------------------------------
        # QoS:SensorDataQoS = BEST_EFFORT + KEEP_LAST(5)。
        # 传感器流宁可丢帧也不要积压(有界缓存),Foxglove/RViz 均兼容。
        # 话题是相对名,节点在 /auv 命名空间下,最终形如 /auv/zed/left/image_rect。
        # 关闭的通道连 publisher 都不创建 -> 话题不存在(而不是发空)。
        qos = qos_profile_sensor_data
        self._pub_left = (
            self.create_publisher(Image, 'zed/left/image_rect', qos)
            if self._p['publish_left_image'] else None)
        self._pub_left_info = (
            self.create_publisher(CameraInfo, 'zed/left/camera_info', qos)
            if self._p['publish_left_image'] else None)
        self._pub_right = (
            self.create_publisher(Image, 'zed/right/image_rect', qos)
            if self._p['publish_right_image'] else None)
        self._pub_right_info = (
            self.create_publisher(CameraInfo, 'zed/right/camera_info', qos)
            if self._p['publish_right_image'] else None)
        self._pub_depth = (
            self.create_publisher(Image, 'zed/depth', qos)
            if self._p['publish_depth'] else None)
        self._pub_points = (
            self.create_publisher(PointCloud2, 'zed/points', qos)
            if self._p['publish_points'] else None)
        self._pub_imu = (
            self.create_publisher(Imu, 'zed/imu', qos)
            if self._p['publish_imu'] else None)
        self.get_logger().info(
            '发布通道: 左目=%s 右目=%s 深度=%s 点云=%s IMU=%s' % (
                '开' if self._p['publish_left_image'] else '关',
                '开' if self._p['publish_right_image'] else '关',
                '开' if self._p['publish_depth'] else '关',
                '开' if self._p['publish_points'] else '关',
                '开' if self._p['publish_imu'] else '关'))

        # /clock 只用于诊断(摘要里打印图像时间戳与 /clock 的偏差),
        # 不参与消息打戳 —— 消息时间戳始终来自数据本身。
        self._last_clock_ns = None
        self.create_subscription(Clock, '/clock', self._on_clock, qos)

        # ---------------- 状态(跨线程读写,用锁保护) -------------------------------
        self._lock = threading.Lock()
        self._state = '未连接'             # 人类可读的连接状态
        self._counters = {k: 0 for k in (
            'left', 'right', 'depth', 'points', 'imu',
            'imu_zero', 'grab_err', 'time_back')}
        self._last_ts_ns = 0               # 最近一帧的(仿真)时间戳
        self._last_imu_note = ''

        self._stop = threading.Event()
        self._zed = None                   # 只被 _grab_thread 触碰

        self._grab_thread = threading.Thread(
            target=self._grab_loop, name='zed_grab', daemon=True)
        self._summary_thread = threading.Thread(
            target=self._summary_loop, name='zed_summary', daemon=True)
        self._grab_thread.start()
        self._summary_thread.start()

    # ------------------------------------------------------------------ /clock --
    def _on_clock(self, msg):
        self._last_clock_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec

    # ------------------------------------------------------------------ 摘要 ----
    def _summary_loop(self):
        """墙钟 1 Hz 摘要:连接状态、各话题实测频率、IMU 状态、时间戳。"""
        period = max(0.2, float(self._p['summary_period_s']))
        prev = dict(self._counters)
        prev_t = time.monotonic()
        while not self._stop.wait(period):
            now_t = time.monotonic()
            dt = now_t - prev_t
            with self._lock:
                cur = dict(self._counters)
                ts_ns = self._last_ts_ns
                imu_note = self._last_imu_note
            rates = {k: (cur[k] - prev[k]) / dt if dt > 0 else 0.0
                     for k in ('left', 'right', 'depth', 'points', 'imu')}

            def _ch(key, enabled, label):
                return (f"{label} {rates[key]:.1f}Hz" if enabled
                        else f"{label} 关")

            note = (f"{self._state} | "
                    + _ch('left', self._p['publish_left_image'], 'L') + ' '
                    + _ch('right', self._p['publish_right_image'], 'R') + ' '
                    + _ch('depth', self._p['publish_depth'], 'D') + ' '
                    + _ch('points', self._p['publish_points'], 'PC') + ' '
                    + _ch('imu', self._p['publish_imu'], 'IMU')
                    + (f"({imu_note})" if self._p['publish_imu'] else '')
                    + f" | 仿真时间 {ts_ns / 1e9:.2f}s")
            if self._last_clock_ns is not None and ts_ns:
                note += f" | /clock 偏差 {(ts_ns - self._last_clock_ns) / 1e9:+.2f}s"
            self.get_logger().info(note)
            prev, prev_t = cur, now_t

    # ------------------------------------------------------------------ 采集 ----
    def _grab_loop(self):
        """采集线程主体:连接 -> 循环拉流,断开后按间隔重试。"""
        if not PYZED_AVAILABLE:
            self._set_state('pyzed 未安装(缺 ZED SDK Python API),采集线程退出')
            return
        while not self._stop.is_set() and rclpy.ok():
            zed = self._open_camera()
            if zed is None:
                # 有节制的重试:期间摘要线程会持续显示"未连接"
                self._stop.wait(float(self._p['open_retry_s']))
                continue
            try:
                self._stream_loop(zed)
            except Exception as exc:  # noqa: BLE001 —— 采集线程必须自己兜底
                import traceback
                tb = traceback.format_exc()
                self.get_logger().error(f'流异常: {exc}\n{tb}')
                self._set_state(f'流异常: {exc}')
            finally:
                # 释放本会话的相机与缓冲(仿真暂停导致的 grab 阻塞靠
                # daemon 线程在进程退出时兜底)
                try:
                    zed.close()
                except Exception:
                    pass
                self._zed = None
                self._set_state('连接断开')
            if not self._stop.is_set():
                self._stop.wait(float(self._p['open_retry_s']))

    def _open_camera(self):
        """按参数打开流。成功返回 sl.Camera,失败返回 None。"""
        ip = str(self._p['stream_ip'])
        port = int(self._p['stream_port'])
        init = sl.InitParameters()
        init.set_from_stream(ip, port)
        # 深度与点云都关时,打开模式直接用 NONE —— SDK 端连立体匹配都不做
        # (省 GPU);只要任一开着,按 depth_mode 参数打开。
        if not (self._p['publish_depth'] or self._p['publish_points']):
            init.depth_mode = sl.DEPTH_MODE.NONE
            self.get_logger().info('深度+点云均已关闭:SDK 以 DEPTH_MODE.NONE 打开(跳过立体匹配)')
        else:
            init.depth_mode = getattr(sl.DEPTH_MODE, str(self._p['depth_mode']), sl.DEPTH_MODE.NONE)
        init.coordinate_units = sl.UNIT.METER
        init.sdk_verbose = 0
        zed = sl.Camera()
        try:
            status = zed.open(init)
        except Exception as exc:  # noqa: BLE001
            self._set_state(f'打开异常: {exc}')
            return None
        if status != sl.ERROR_CODE.SUCCESS:
            # 连不上最常见原因:Isaac Sim 没开 / 场景里 ZED 节点没运行 /
            # 仿真没按 Play。警告限频,避免刷屏。
            self.get_logger().warning(
                f'连接 ZED 流失败({ip}:{port}): {status} —— '
                f'请检查 Isaac Sim 是否运行、场景 ZED Camera 节点是否在推流',
                throttle_duration_sec=10.0)
            return None
        info = zed.get_camera_information()
        calib = info.camera_configuration.calibration_parameters
        # 虚拟相机的 get_camera_baseline() 实测返回 0,改从 stereo_transform
        # 的平移 X 取;返回 120 表示 120mm(ZED X 规格),>1 视为 mm 再换算成米。
        try:
            t = calib.stereo_transform.get_translation()
            try:
                tx = float(t.get()[0])
            except Exception:
                tx = float(str(t).split()[0])
        except Exception:
            tx = 0.0
        baseline = tx / 1000.0 if tx > 1.0 else tx
        self._calib = {
            'fx': float(calib.left_cam.fx), 'fy': float(calib.left_cam.fy),
            'cx': float(calib.left_cam.cx), 'cy': float(calib.left_cam.cy),
            'fx_r': float(calib.right_cam.fx), 'fy_r': float(calib.right_cam.fy),
            'cx_r': float(calib.right_cam.cx), 'cy_r': float(calib.right_cam.cy),
            'baseline': baseline,
        }
        self._zed = zed
        self._set_state(
            f"已连接({info.camera_model}, 基线 {baseline * 1000:.0f}mm, "
            f"SN {info.serial_number})")
        return zed

    def _stream_loop(self, zed):
        """已连接状态下的主循环:grab -> 限频发布。返回即视为断开。"""
        runtime = sl.RuntimeParameters()
        img_l, img_r = sl.Mat(), sl.Mat()
        depth_m, points = sl.Mat(), sl.Mat()
        sens = sl.SensorsData()
        try:
            last_pub_ns = {}        # 各通道上次发布用的(仿真)时间戳,用于限频
            fails = 0
            last_ts_ns = 0
            while not self._stop.is_set() and rclpy.ok():
                err = zed.grab(runtime)
                if err != sl.ERROR_CODE.SUCCESS:
                    fails += 1
                    self._bump('grab_err')
                    if fails > 60:      # 连续 1 秒(60fps)拿不到帧才判定断开
                        return
                    continue
                fails = 0

                ts_ns = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).data_ns
                if last_ts_ns and ts_ns < last_ts_ns:
                    # 仿真重置/回跳:时间戳变小。照常发布(这帧确实是新数据),
                    # 但计数并在摘要里可见,绝不把旧帧重发充数。
                    self._bump('time_back')
                    self.get_logger().warning(
                        f'仿真时间回跳:{ts_ns / 1e9:.2f}s < {last_ts_ns / 1e9:.2f}s',
                        throttle_duration_sec=5.0)
                last_ts_ns = ts_ns
                stamp = stamp_from_ns(ts_ns)
                with self._lock:
                    self._last_ts_ns = ts_ns

                # ---------------- 左/右图像 + CameraInfo(同时间戳成对发布) --------
                # 左右各自独立限频、独立开关;关闭的通道连 retrieve 都不调。
                if self._p['publish_left_image'] and self._due(
                        last_pub_ns, 'left', ts_ns, self._p['image_rate_hz']):
                    if zed.retrieve_image(img_l, sl.VIEW.LEFT) == sl.ERROR_CODE.SUCCESS:
                        arr = img_l.get_data()   # numpy 视图,指向 SDK 内存
                        msg = bgra_to_image_msg(arr, self._p['left_frame'], stamp)
                        # 转换内部 tobytes() 已把数据拷进消息,SDK 缓冲此后
                        # 被覆盖也不影响已发布的消息 —— 这是内存生命周期的
                        # 关键点(要求:发布后缓冲区不得被复用)。
                        self._pub_left.publish(msg)
                        self._pub_left_info.publish(camera_info_msg(
                            msg.width, msg.height,
                            self._calib['fx'], self._calib['fy'],
                            self._calib['cx'], self._calib['cy'],
                            stamp, self._p['left_frame'],
                            baseline_m=self._calib['baseline'], right=False))
                        self._bump('left')

                if self._p['publish_right_image'] and self._due(
                        last_pub_ns, 'right', ts_ns, self._p['image_rate_hz']):
                    if zed.retrieve_image(img_r, sl.VIEW.RIGHT) == sl.ERROR_CODE.SUCCESS:
                        msg_r = bgra_to_image_msg(
                            img_r.get_data(), self._p['right_frame'], stamp)
                        self._pub_right.publish(msg_r)
                        self._pub_right_info.publish(camera_info_msg(
                            msg_r.width, msg_r.height,
                            self._calib['fx_r'], self._calib['fy_r'],
                            self._calib['cx_r'], self._calib['cy_r'],
                            stamp, self._p['right_frame'],
                            baseline_m=self._calib['baseline'], right=True))
                        self._bump('right')

                # ---------------- 深度(SDK 视觉计算,非真值) --------------------------
                if self._p['publish_depth'] and self._due(
                        last_pub_ns, 'depth', ts_ns, self._p['depth_rate_hz']):
                    if zed.retrieve_measure(depth_m, sl.MEASURE.DEPTH) == sl.ERROR_CODE.SUCCESS:
                        self._pub_depth.publish(depth_to_image_msg(
                            depth_m.get_data(), self._p['depth_frame'], stamp))
                        self._bump('depth')

                # ---------------- 点云(左目光学系,抽稀后发布) ------------------------
                if self._p['publish_points'] and self._due(
                        last_pub_ns, 'points', ts_ns, self._p['points_rate_hz']):
                    if zed.retrieve_measure(points, sl.MEASURE.XYZRGBA) == sl.ERROR_CODE.SUCCESS:
                        self._pub_points.publish(xyzrgba_to_pointcloud2_msg(
                            points.get_data(), self._p['points_frame'], stamp,
                            max_points=int(self._p['points_max_points'])))
                        self._bump('points')

                # ---------------- IMU ---------------------------------------------------
                # 防假数据检查:acc/gyro 同时全零 = 场景端没给 IMU 数据
                # (2026-10-31 03:35 前就是这种状态),物理上不可能——
                # 静止时加速度也应有重力分量。全零 -> 不发布(发出去就是
                # 伪造测量值),只计数。03:35 起场景补上 Imu_Sensor 后,
                # 实测 acc=(0,-9.81,0)(重力),IMU 通道正常发布(~40Hz)。
                if self._p['publish_imu'] and zed.get_sensors_data(
                        sens, sl.TIME_REFERENCE.IMAGE) == sl.ERROR_CODE.SUCCESS:
                    imu = sens.get_imu_data()
                    acc = np.asarray(imu.get_linear_acceleration(), dtype=np.float64)[:3]
                    gyro = np.asarray(imu.get_angular_velocity(), dtype=np.float64)[:3]
                    if not np.any(acc) and not np.any(gyro):
                        self._bump('imu_zero')
                        self._note_imu('不可用:全零(场景缺 Imu_Sensor,见 README)')
                    else:
                        # Orientation 分量提取:pyzed 5.4 实测 get(i)/
                        # 下标取值都不可用,按 repr 解析,顺序 x y z w
                        # (单位阵时 0 0 0 1),再转成 imu_msg 要的 (w,x,y,z)。
                        ori_obj = imu.get_pose().get_orientation()
                        try:
                            ori = [float(ori_obj.get(i)) for i in range(4)]
                        except Exception:
                            ori = [float(v) for v in str(ori_obj).split()]
                        quat_wxyz = (ori[3], ori[0], ori[1], ori[2])
                        self._pub_imu.publish(imu_msg(
                            acc, gyro, quat_wxyz, stamp, self._p['imu_frame'],
                            acc_cov=_pyzed_matrix3_to_list(imu.get_linear_acceleration_covariance()),
                            gyro_cov=_pyzed_matrix3_to_list(imu.get_angular_velocity_covariance()),
                            ori_cov=_pyzed_matrix3_to_list(imu.get_pose_covariance())))
                        self._bump('imu')
                        self._note_imu(
                            f'acc[{acc[0]:+.2f},{acc[1]:+.2f},{acc[2]:+.2f}] '
                            f'gyro[{gyro[0]:+.3f},{gyro[1]:+.3f},{gyro[2]:+.3f}]')
        finally:
            for m in (img_l, img_r, depth_m, points):
                try:
                    m.free()
                except Exception:
                    pass

    # ------------------------------------------------------------------ 工具 ----
    def _due(self, last_pub_ns, key, ts_ns, rate_hz):
        """基于数据时间戳的限频:不到周期就不发。频率来自真实新帧,
        所以暂停/降速时发布频率自然跟着降,不会重复旧数据凑数。"""
        rate = float(rate_hz)
        if rate <= 0.0:
            return False
        last = last_pub_ns.get(key)
        if last is None or (ts_ns - last) >= 1e9 / rate:
            last_pub_ns[key] = ts_ns
            return True
        return False

    def _set_state(self, text):
        with self._lock:
            self._state = text
        self.get_logger().info(text)

    def _note_imu(self, text):
        with self._lock:
            self._last_imu_note = text

    def _bump(self, key):
        with self._lock:
            self._counters[key] += 1

    # ------------------------------------------------------------------ 释放 ----
    def destroy_node(self):
        """Ctrl+C / ros2 lifecycle 退出路径:先停线程,再释放。

        zed 相机对象只属于采集线程,这里只负责 join;相机在采集线程的
        finally 里 close。native grab 若被暂停的流阻塞,join 超时后由
        daemon 线程随进程退出兜底。
        """
        self._stop.set()
        for t in (getattr(self, '_grab_thread', None),
                  getattr(self, '_summary_thread', None)):
            if t is not None and t.is_alive():
                t.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    if not PYZED_AVAILABLE:
        print('[zed_sensor_node] 致命错误:当前 Python 环境导入不了 pyzed。'
              '请在装有 ZED SDK Python API 的环境里运行(见 README)。')
        raise SystemExit(1)
    rclpy.init(args=args)
    node = ZedSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
