# 验证记录(全部为实测,非计划值)

## 单元测试
- test/test_converters.py: 11/11 通过(2026-10-31, python3.12 + pytest)
- 覆盖: 时间戳换算/BGRA 转 Image/深度 NaN/CameraInfo 立体 Tx/
  IMU 字段与四元数顺序/点云抽稀+NaN 过滤+颜色/旋转矩阵转四元数

## 在线冒烟(2026-10-31,Isaac Sim 场景播放中)
工具: rclpy 探针(SensorDataQoS 直订,不依赖 ros2cli daemon)。
注意: Python 探针反序列化 9.2MB 图像跟不上,探针计数偏低;
发布端权威速率以节点 1Hz 状态行为准。

- 运行 60+ 秒,流异常 0 次,断线 0 次,持续稳定
- 发布端速率(墙钟): L/R 11Hz, D 7-8Hz, PC 3-4Hz, IMU 37-40Hz
  (配置 15/10/5Hz 按"数据时间戳"限频;仿真减速比 ~0.65,墙钟频率相应降低)
- IMU 实收: frame=zed_imu_link, ori=(0,0,0,1), acc=(0,-9.81,0) m/s^2
  (真实重力), gyro=(0,0,0)(机器人静止,合理), ori_cov 对角 0.004
- 深度实收: 32FC1, min=2.37m max=20.00m, NaN 占 55%(无效区,PERFORMANCE 模式)
- 点云实收: 61294 点, point_step=16, fields x/y/z/rgb, dense=True
- CameraInfo: fx=741.6 cx=960 cy=600 D=0;右目 Tx=-89.0(=-741.6*0.12)
- 图像: bgra8 1920x1200, 左右同戳(3079.417), 与 CameraInfo 配对(3079.217)

## 历史问题(已解决)
1. Orientation 下标崩溃 → repr 解析(x y z w)
2. 协方差 Matrix3f 迭代崩溃 → _pyzed_matrix3_to_list
3. 虚拟相机基线=0 → stereo_transform 平移 120mm
4. 早期 IMU 全零 → 用户在场景补 Imu_Sensor 后恢复(03:35 起)

## 待完成
- [ ] /auv/ground_truth/odometry: 等 diagnose_scene.py 提供 robot prim
      后由 add_ground_truth_graph.py 添加,实测 ~100Hz 与帧号
- [ ] Foxglove 目视验证(需用户本机连接,我无法替代目视)
