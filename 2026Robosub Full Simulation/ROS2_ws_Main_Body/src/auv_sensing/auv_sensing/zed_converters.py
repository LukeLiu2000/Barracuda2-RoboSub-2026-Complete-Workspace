# -*- coding: utf-8 -*-
"""纯数据转换函数:numpy 数组 / 标定数值 -> ROS 2 消息。

本模块故意不依赖 pyzed(只依赖 numpy + sensor_msgs),这样转换逻辑
可以用合成的测试数据做单元测试(test/test_converters.py),
不需要运行中的 Isaac Sim 或 ZED SDK。
"""

import numpy as np
import struct

from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField

# ---------------------------------------------------------------------------
# 时间戳
# ---------------------------------------------------------------------------

def stamp_from_ns(ts_ns):
    """ZED SDK 的时间戳(整数纳秒)-> ROS Time 消息。

    实测:本工程里 Isaac Sim 的 ZED 扩展(sl.sensor.camera)把
    IsaacReadSimulationTime 节点的输出接进了流,因此 SDK 返回的图像
    时间戳就是 Isaac 仿真时间(单位 ns),不是墙钟时间。
    """
    sec, nsec = divmod(int(ts_ns), 1_000_000_000)
    t = TimeMsg()
    t.sec = int(sec)
    t.nanosec = int(nsec)
    return t


# ---------------------------------------------------------------------------
# 图像 / 深度
# ---------------------------------------------------------------------------

def bgra_to_image_msg(bgra, frame_id, stamp):
    """HxWx4 的 BGRA uint8 数组 -> sensor_msgs/Image(编码 bgra8)。

    SDK 的 retrieve_image(LEFT/RIGHT) 返回 BGRA(带 alpha 通道)。
    直接以 bgra8 发布,不为省带宽做像素级颜色转换(需要时由下游转)。
    注意:传入数组必须是已经拷贝过(np.copy)的、自己拥有内存的数据,
    因为 SDK 的 Mat.get_data() 返回的视图在下次 grab 后会被覆盖。
    """
    msg = Image()
    msg.height, msg.width = bgra.shape[0], bgra.shape[1]
    msg.encoding = 'bgra8'
    msg.is_bigendian = False          # x86 主机是小端
    msg.step = msg.width * 4          # 每行字节数
    msg.data = bgra.tobytes()         # tobytes() 会生成一份连续拷贝
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    return msg


def depth_to_image_msg(depth_m, frame_id, stamp):
    """HxW float32 深度图(单位:米)-> sensor_msgs/Image(编码 32FC1)。

    约定(见 docs/interfaces.md):
    - 数值 = 该像素的 Z 距离,单位米;
    - 无效/不可信像素为 NaN(NOT 0!),下游请用 np.isnan() 过滤。
    """
    msg = Image()
    msg.height, msg.width = depth_m.shape[0], depth_m.shape[1]
    msg.encoding = '32FC1'
    msg.is_bigendian = False
    msg.step = msg.width * 4
    msg.data = np.ascontiguousarray(depth_m, dtype=np.float32).tobytes()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    return msg


# ---------------------------------------------------------------------------
# 相机标定(CameraInfo)
# ---------------------------------------------------------------------------

def camera_info_msg(width, height, fx, fy, cx, cy, stamp, frame_id,
                    baseline_m=0.0, right=False):
    """矩形化图像的 CameraInfo(双目左/右共用本函数)。

    - SDK 输出的左右图已经过立体校正(rectified),所以 D=0(无畸变)、
      R=单位阵;
    - ROS REP 104 约定:右目 P 矩阵的 Tx = -fx * 基线长;
      左目 Tx = 0。本工程的虚拟 ZED X 基线实测 0.12 m。
    - 这些值来自 ZED SDK 的标定参数(虚拟相机的标定由
      sl.sensor.camera 扩展按型号规格注入,如 HD1200 fx=741.6)。
    """
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height, msg.width = height, width
    msg.distortion_model = 'plumb_bob'
    msg.d = [0.0] * 5                 # 校正后无畸变
    # R:从原始(畸变)图像坐标系到校正图像坐标系,校正后即单位阵
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    tx = -fx * baseline_m if right else 0.0
    # P:3x4 投影矩阵,行主序展开
    msg.p = [fx, 0.0, cx, tx,
             0.0, fy, cy, 0.0,
             0.0, 0.0, 1.0, 0.0]
    msg.binning_x = 0
    msg.binning_y = 0
    msg.roi.width = width
    msg.roi.height = height
    msg.roi.do_rectify = False
    return msg


# ---------------------------------------------------------------------------
# IMU
# ---------------------------------------------------------------------------

def imu_msg(acc, gyro, quat_wxyz, stamp, frame_id,
            acc_cov=None, gyro_cov=None, ori_cov=None):
    """组装 sensor_msgs/Imu。

    单位约定(ROS REP 103 / sensor_msgs 规范):
    - acc:线加速度,m/s^2,在 IMU 自身坐标系(IMU frame)表达;
    - gyro:角速度,rad/s,同上;
    - quat_wxyz:姿态四元数,输入顺序 (w, x, y, z),写入消息时转成
      geometry_msgs 的 (x, y, z, w)。

    协方差约定(REP 145):
    - 全 0 表示"未知/未提供",不是 0 方差;
    - orientation_covariance[0] = -1 表示本消息不携带姿态。
    本包在 IMU 数据不可用(SDK 全零)时不发布 IMU,见节点代码。
    """
    msg = Imu()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.linear_acceleration.x = float(acc[0])
    msg.linear_acceleration.y = float(acc[1])
    msg.linear_acceleration.z = float(acc[2])
    msg.angular_velocity.x = float(gyro[0])
    msg.angular_velocity.y = float(gyro[1])
    msg.angular_velocity.z = float(gyro[2])
    if quat_wxyz is None:
        msg.orientation_covariance[0] = -1.0     # 无姿态数据
    else:
        w, x, y, z = (float(v) for v in quat_wxyz)
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w
    if acc_cov is not None:
        msg.linear_acceleration_covariance = [float(v) for v in acc_cov]
    if gyro_cov is not None:
        msg.angular_velocity_covariance = [float(v) for v in gyro_cov]
    if ori_cov is not None and quat_wxyz is not None:
        msg.orientation_covariance = [float(v) for v in ori_cov]
    return msg


# ---------------------------------------------------------------------------
# 点云
# ---------------------------------------------------------------------------

def xyzrgba_to_pointcloud2_msg(xyzrgba, frame_id, stamp, max_points=200000):
    """SDK 的 HxWx4 float32 点云 -> sensor_msgs/PointCloud2。

    输入说明:
    - xyzrgba[..., 0:3] 是 X/Y/Z(米),坐标系为 ZED SDK 默认的
      IMAGE 坐标系(x 右,y 下,z 前),即左目光学坐标系;
    - xyzrgba[..., 3] 是把 RGBA 打包进 float32 的位模式。

    处理步骤:
    1. 抽稀:按等距行列步长把点数压到 max_points 以内
       (全分辨率 1920x1200 = 230 万点,直接发布网络带宽不可行);
    2. 过滤:丢掉 XYZ 含 NaN/Inf 的点(无效深度);
    3. 颜色:float 位模式重解释为 uint32,按 ZED 官方示例的
       打包约定 (R<<16)|(G<<8)|B 重排后放入 'rgb' 字段
       (uint32,PCL 约定;字节含义以 Foxglove 实测为准,见
       docs/verification.md 的验证记录)。
    """
    arr = np.ascontiguousarray(xyzrgba, dtype=np.float32)
    h, w = arr.shape[0], arr.shape[1]
    total = h * w
    if total > max_points:
        stride = int(np.ceil(np.sqrt(total / float(max_points))))
    else:
        stride = 1
    arr = arr[::stride, ::stride, :]
    xyz = arr[..., 0:3]
    rgba_f = arr[..., 3]
    valid = np.isfinite(xyz).all(axis=-1)       # 任一分量非有限即无效
    xyz = xyz[valid]
    rgba_f = rgba_f[valid]

    if rgba_f.size:
        raw = rgba_f.view(np.uint32)            # float 位模式 -> uint32
        r = (raw >> 16) & 0x000000FF
        g = (raw >> 8) & 0x000000FF
        b = raw & 0x000000FF
        rgb = (r << 16) | (g << 8) | b          # 重排成 PCL 习惯的 RGB
    else:
        rgb = np.empty(0, dtype=np.uint32)

    n = xyz.shape[0]
    # 内存布局:每点 16 字节 = x(4) y(4) z(4) rgb(4),紧密排列
    buf = np.empty((n, 4), dtype=np.float32)
    buf[:, 0:3] = xyz
    buf[:, 3] = rgb.view(np.float32) if rgb.size else 0.0

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * n
    msg.is_dense = True                          # NaN 点已在上面过滤
    msg.data = buf.tobytes()
    return msg


# ---------------------------------------------------------------------------
# 姿态四元数
# ---------------------------------------------------------------------------

def rotation_to_quaternion_wxyz(R):
    """3x3 旋转矩阵 -> 四元数 (w, x, y, z)。

    使用标准的 Shepperd 方法,保证数值稳定(避免除以接近 0 的数)。
    输入 R 必须是行主序的 numpy 3x3 数组。
    """
    R = np.asarray(R, dtype=np.float64)
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    return (float(w), float(x), float(y), float(z))
