# auv_sensing —— ZED 仿真传感器 + ROS 2 桥接包

把 Isaac Sim 虚拟 ZED 相机(经 ZED SDK 流)发布的图像/深度/点云/IMU
转换成标准 ROS 2 话题,并(配合场景脚本)发布机体真值里程计。

## 数据链路

```
Isaac Sim 场景(ZED Camera OGN 节点, 60Hz)
  └─ ZED SDK 流(IPC/SHM, 127.0.0.1:30000)
      └─ 本包 zed_sensor_node(pyzed 客户端)
          └─ /auv/zed/* ROS 2 话题(见下表)
真值: 场景 Action Graph(isaac/add_ground_truth_graph.py 添加)
  └─ /auv/ground_truth/odometry + /clock(不经本包转发)
```

关键事实:**所有图像/IMU 时间戳都是 Isaac 仿真时间(秒)**,由场景端
OGN 注入 SDK 流,与 /clock 同源;不依赖 ROS use_sim_time。

## 构建与运行(全新终端,逐条执行)

```bash
cd "/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/2026Robosub Full Simulation/ROS2_ws_Main_Body"
export PATH=$HOME/.local/bin:$PATH          # colcon 在 ~/.local/bin
source /opt/ros/jazzy/setup.bash
colcon build --packages-select auv_sensing
source install/setup.bash

# 前提: Isaac Sim 已打开场景且 ZED 流节点在推流(播放状态)
ros2 launch auv_sensing sensing.launch.py
```

停止:在节点终端按 Ctrl+C(干净退出,自动关 ZED、join 线程)。

## 话题一览(全部 SensorDataQoS: BEST_EFFORT + KEEP_LAST(5))

| 话题 | 类型 | 配置频率 | 实测频率* | 单位/说明 |
|---|---|---|---|---|
| /auv/zed/left/image_rect | sensor_msgs/Image | 15Hz | ~11Hz | bgra8 1920x1200 |
| /auv/zed/right/image_rect | sensor_msgs/Image | 15Hz | ~11Hz | 同上,与左目同帧同戳 |
| /auv/zed/left/camera_info | sensor_msgs/CameraInfo | 随图像 | ~11Hz | D=0, R=I, 内参来自 SDK 标定 |
| /auv/zed/right/camera_info | sensor_msgs/CameraInfo | 随图像 | ~11Hz | P[0][3]=-fx*基线(REP 104) |
| /auv/zed/depth | sensor_msgs/Image | 10Hz | ~7Hz | 32FC1, 米, NaN=无效点 |
| /auv/zed/points | sensor_msgs/PointCloud2 | 5Hz | ~3Hz | x/y/z(米)+rgb, ≤200k 点 |
| /auv/zed/imu | sensor_msgs/Imu | 随帧(~40Hz) | ~38Hz | m/s^2, rad/s; 协方差 0=未知 |
| /auv/ground_truth/odometry | nav_msgs/Odometry | 100Hz(场景端) | 待测 | 场景 Action Graph 直发 |

*实测:仿真推进慢于墙钟(约 0.6-0.7x),频率按数据时间戳限频,
所以墙钟频率 = 配置频率 x 仿真减速比。暂停仿真即停止发布(不发陈旧数据)。

坐标 frame: zed_left_camera_frame / zed_right_camera_frame / zed_imu_link;
真值: odom -> base_link。

## 验证命令

```bash
source /opt/ros/jazzy/setup.bash
source "/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/2026Robosub Full Simulation/ROS2_ws_Main_Body/install/setup.bash"
ros2 topic list | grep auv
ros2 topic echo --qos-reliability best_effort -n 1 /auv/zed/imu
ros2 topic echo --qos-reliability best_effort -n 1 /auv/zed/left/camera_info
# 注意: ros2 topic hz 不支持 QoS 参数且默认 RELIABLE,与本包
# BEST_EFFORT 发布不兼容,请勿用它测速;以节点 1Hz 状态行为准。
```

## 参数(config/sensing.yaml)

stream_ip/stream_port(ZED 流地址)、depth_mode、image_rate_hz、
depth_rate_hz、points_rate_hz、points_max_points、publish_right_image、
open_retry_s、各 frame_id、summary_period_s。全部带中文注释。

## 单元测试

```bash
cd "/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/2026Robosub Full Simulation/ROS2_ws_Main_Body/src/auv_sensing"
python3 -m pytest test/test_converters.py -v     # 11 项全过(无需 pyzed)
```

## 设计说明(为什么这样做)

1. **不写"空壳转发节点"**:真值由场景 Action Graph 直发
   (isaac/add_ground_truth_graph.py),本包不做无意义的二次转发。
2. **不造假数据**:IMU 全零帧(场景端传感器未就绪)不发布,只计数;
   深度无效点保留 NaN 而不是填 0。
3. **SDK 深度 ≠ 真值**:PERFORMANCE 模式深度是 SDK 估计值,用于算法
   输入;真值只在 /auv/ground_truth/odometry,两者严格分开。
4. **暂停/复位/回跳**:按数据时间戳限频+回跳检测,仿真暂停时自动
   停发,时间回跳只告警不崩溃。
5. pyzed 5.4 的 Orientation/Matrix3f 不能直接迭代/取下标,节点内有
   专门的解析函数(含中文注释说明实测行为)。

## 已知限制

- depth_mode=PERFORMANCE 会打印 SDK 弃用警告(建议 NEURAL),且
  实测约 55% 像素为 NaN(超量程/无效区);换 NEURAL 需首次 TensorRT
  编译,未默认启用。
- 图像编码 bgra8(SDK 内存原生格式);如需 rgb8 在消费端转换。
- /clock 目前无发布者(场景还没加 Clock 节点);本包时间戳本身就是
  仿真时间,use_sim_time=false 即可。真值脚本会同时发 /clock。
- IMU 协方差:SDK 给 acc/gyro 全 0(按 REP 145 语义为"未知"),
  orientation 协方差 0.004(场景提供)。
