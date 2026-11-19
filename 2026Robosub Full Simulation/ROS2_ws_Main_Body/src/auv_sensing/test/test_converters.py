# -*- coding: utf-8 -*-
"""转换函数单元测试。

只用明确标注的合成测试数据(不依赖运行中的 Isaac Sim / ZED SDK),
符合"独立单元测试可以使用明确标注的测试数据,但不进入正式运行链路"。
运行方式(在包目录下):
    python3 -m pytest test/test_converters.py -v
或整个 workspace:colcon test --packages-select auv_sensing
"""

import math

import numpy as np
import pytest

from auv_sensing.zed_converters import (
    bgra_to_image_msg,
    camera_info_msg,
    depth_to_image_msg,
    imu_msg,
    rotation_to_quaternion_wxyz,
    stamp_from_ns,
    xyzrgba_to_pointcloud2_msg,
)

STAMP = stamp_from_ns(1_234_567_890_123)     # 合成时间戳:1234.567890123 s


# ---------------------------------------------------------------- 时间戳 ----

def test_stamp_from_ns():
    assert STAMP.sec == 1234
    assert STAMP.nanosec == 567890123


# ---------------------------------------------------------------- 图像 ------

def test_bgra_image_msg():
    # 合成 4x3 BGRA 图(值 0..255)
    arr = np.arange(4 * 3 * 4, dtype=np.uint8).reshape(3, 4, 4)
    msg = bgra_to_image_msg(arr, 'zed_left_camera_frame', STAMP)
    assert msg.width == 4 and msg.height == 3
    assert msg.encoding == 'bgra8'
    assert msg.step == 4 * 4                 # 每像素 4 字节
    assert len(msg.data) == 3 * 4 * 4
    assert msg.header.frame_id == 'zed_left_camera_frame'
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (1234, 567890123)


def test_depth_nan_preserved():
    # 合成深度:中间像素无效(NaN),其余 1.0 m
    depth = np.ones((4, 4), dtype=np.float32)
    depth[1, 1] = np.nan
    msg = depth_to_image_msg(depth, 'zed_left_camera_frame', STAMP)
    assert msg.encoding == '32FC1'
    assert msg.step == 4 * 4
    back = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(4, 4)
    assert np.isnan(back[1, 1])              # NaN 语义必须原样保留
    assert back[0, 0] == pytest.approx(1.0)


# ---------------------------------------------------------------- 标定 ------

def test_camera_info_stereo_convention():
    # 虚拟 ZED X 实测标定(HD1200)+ 0.12 m 基线
    li = camera_info_msg(1920, 1200, 741.6, 741.6, 960.0, 600.0,
                         STAMP, 'zed_left_camera_frame',
                         baseline_m=0.12, right=False)
    ri = camera_info_msg(1920, 1200, 741.6, 741.6, 960.0, 600.0,
                         STAMP, 'zed_right_camera_frame',
                         baseline_m=0.12, right=True)
    # 左目 Tx = 0;右目 Tx = -fx*基线(REP 104)
    assert li.p[3] == pytest.approx(0.0)
    assert ri.p[3] == pytest.approx(-741.6 * 0.12)
    # 校正后:R 单位阵,D 全 0
    assert list(li.r) == [1, 0, 0, 0, 1, 0, 0, 0, 1]
    assert all(d == 0.0 for d in li.d)
    assert (li.width, li.height) == (1920, 1200)


# ---------------------------------------------------------------- IMU -------

def test_imu_msg_fields_and_quat_order():
    # 输入四元数 (w,x,y,z) = (1,0,0,0);消息字段应为 x=0,y=0,z=0,w=1
    msg = imu_msg((0.1, -0.2, 9.81), (0.01, 0.02, -0.03),
                  (1.0, 0.0, 0.0, 0.0), STAMP, 'zed_imu_link')
    assert (msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w) == (0.0, 0.0, 0.0, 1.0)
    assert msg.linear_acceleration.z == pytest.approx(9.81)
    assert msg.angular_velocity.y == pytest.approx(0.02)


def test_imu_msg_no_orientation():
    msg = imu_msg((0, 0, 0), (0, 0, 0), None, STAMP, 'zed_imu_link')
    assert msg.orientation_covariance[0] == -1.0   # REP 145:无姿态


# ---------------------------------------------------------------- 点云 ------

def _pack_rgba(r, g, b, a=255):
    """按 ZED 打包约定 (A<<24)|(R<<16)|(G<<8)|B 组装 float 位模式(测试数据)。"""
    u32 = (a << 24) | (r << 16) | (g << 8) | b
    return np.array([u32], dtype=np.uint32).view(np.float32)[0]


def test_pointcloud_filter_and_decimation():
    # 合成 100x100:点 (x, y, z, rgba);四角放 NaN,中心放一个红点
    h = w = 100
    xyz = np.zeros((h, w, 4), dtype=np.float32)
    xyz[..., 0] = np.linspace(0, 1, w)[None, :]
    xyz[..., 1] = np.linspace(0, 1, h)[:, None]
    xyz[..., 2] = 2.0
    xyz[0, 0, 2] = np.nan
    xyz[98, 98, 2] = np.inf
    xyz[50, 50, 3] = _pack_rgba(255, 0, 0)
    msg = xyzrgba_to_pointcloud2_msg(xyz, 'zed_left_camera_frame', STAMP,
                                     max_points=2500)
    # 100x100=10000 -> 抽稀到 <=2500:步长=2 -> 50x50=2500,再去掉 2 个无效点
    assert msg.width == 2500 - 2
    assert msg.height == 1
    assert msg.point_step == 16
    assert [f.name for f in msg.fields] == ['x', 'y', 'z', 'rgb']
    buf = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(-1, 4)
    assert np.isfinite(buf[:, :3]).all()          # 无效点已过滤
    red = buf[np.isclose(buf[:, 0], 0.5050, atol=0.02) & np.isclose(buf[:, 1], 0.5050, atol=0.02)]
    if len(red):
        rgb_u32 = np.array([red[0, 3]], dtype=np.float32).view(np.uint32)[0]
        r = (rgb_u32 >> 16) & 0xFF
        g = (rgb_u32 >> 8) & 0xFF
        b = rgb_u32 & 0xFF
        assert (r, g, b) == (255, 0, 0)           # 颜色被重排为 PCL 习惯


def test_pointcloud_no_decimation_when_small():
    xyz = np.ones((10, 10, 4), dtype=np.float32)
    msg = xyzrgba_to_pointcloud2_msg(xyz, 'f', STAMP, max_points=200000)
    assert msg.width == 100


# ---------------------------------------------------------------- 四元数 -----

def test_quaternion_identity():
    R = np.eye(3)
    w, x, y, z = rotation_to_quaternion_wxyz(R)
    assert (w, x, y, z) == pytest.approx((1, 0, 0, 0), abs=1e-9)


def test_quaternion_yaw_90deg():
    # 绕 z 轴转 90 度:期望 (w,x,y,z)=(sqrt2/2, 0, 0, sqrt2/2)
    th = math.pi / 2
    R = np.array([[math.cos(th), -math.sin(th), 0],
                  [math.sin(th),  math.cos(th), 0],
                  [0, 0, 1]])
    w, x, y, z = rotation_to_quaternion_wxyz(R)
    assert (w, x, y, z) == pytest.approx(
        (math.sqrt(2) / 2, 0.0, 0.0, math.sqrt(2) / 2), abs=1e-9)


def test_quaternion_roll_90deg():
    th = math.pi / 2
    R = np.array([[1, 0, 0],
                  [0, math.cos(th), -math.sin(th)],
                  [0, math.sin(th),  math.cos(th)]])
    w, x, y, z = rotation_to_quaternion_wxyz(R)
    assert (w, x, y, z) == pytest.approx(
        (math.sqrt(2) / 2, math.sqrt(2) / 2, 0.0, 0.0), abs=1e-9)
