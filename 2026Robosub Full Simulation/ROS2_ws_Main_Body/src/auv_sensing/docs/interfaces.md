# 接口文档

## 话题(发布方: zed_sensor_node,命名空间 /auv)

### /auv/zed/left/image_rect, /auv/zed/right/image_rect
- 类型 sensor_msgs/Image;encoding=bgra8;1920x1200
- frame: zed_left_camera_frame / zed_right_camera_frame
- 时间戳=SDK 图像时间=Isaac 仿真时间(秒);左右同帧同戳
- QoS: SensorDataQoS(BEST_EFFORT, KEEP_LAST depth=5)

### /auv/zed/left|right/camera_info
- D 全 0(无畸变仿真镜头), R=I
- 左目 P 标准形式 fx=741.6 cx=960 cy=600;右目 P[0][3]=-fx*0.12m=-89.0
  (REP 104 立体标注;基线从 SDK stereo_transform 实测 120mm)

### /auv/zed/depth
- encoding=32FC1,单位米,NaN=无效(不填 0)
- 来源=SDK PERFORMANCE 模式估计值(注意:不是仿真真值!)

### /auv/zed/points
- sensor_msgs/PointCloud2;fields: x/y/z(float32,米)+rgb(uint32)
- 每 16 字节一点;超 20 万点按步长抽稀;过滤 NaN/Inf;is_dense=True

### /auv/zed/imu
- 加速度 m/s^2(实测静止=(0,-9.81,0),含重力);角速度 rad/s
- orientation 四元数来自 SDK 姿态;协方差 0=未知(REP 145),
  orientation 协方差 0.004(场景提供)
- frame: zed_imu_link;全零帧不发布(防假数据),只计数告警

### /auv/ground_truth/odometry(场景 Action Graph 直发,非本包)
- nav_msgs/Odometry;odom -> base_link;目标 100Hz(OnPhysicsStep)
- 这是 Isaac Sim 仿真真值,与 SDK 估计严格区分

## 参数(节选,完整见 config/sensing.yaml)
| 参数 | 默认 | 说明 |
|---|---|---|
| stream_ip / stream_port | 127.0.0.1 / 30000 | ZED SDK 流地址 |
| depth_mode | PERFORMANCE | 可改 NEURAL(需 TensorRT 编译) |
| image_rate_hz | 15.0 | 按"数据时间戳"限频(非墙钟) |
| depth_rate_hz | 10.0 | 同上 |
| points_rate_hz | 5.0 | 同上 |
| points_max_points | 200000 | 点云抽稀上限 |
| open_retry_s | 5.0 | 断流重连退避基数 |

## 时间语义
- 所有数据时间戳 = Isaac 仿真时间(场景 OGN 注入 SDK)
- use_sim_time=false(节点自己的时间只用于日志/统计;/clock 就绪后
  可改 true,不影响数据戳)
- 仿真暂停: grab 停 → 不发布(无陈旧数据填充)
- 时间回跳(复位): 告警+计数,不崩溃
