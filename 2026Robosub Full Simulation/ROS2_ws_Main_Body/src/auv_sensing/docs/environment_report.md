# 环境检查报告(liuhao-5080,只读检查,2026-10-31)

## 系统与软件版本
- OS: Ubuntu 24.04(GPU: RTX 5080)
- ROS 2: Jazzy(/opt/ros/jazzy,默认 RMW: rmw_fastrtps)
- Python: 3.12(ROS 环境)/ Isaac Sim 自带 3.11
- Isaac Sim: 5.1.0(/isaacsim)
- ZED SDK: 5.4.0;pyzed Python 绑定可在系统 python3 导入
- Isaac Sim 场景内: sl.sensor.camera 4.3.0 扩展(ZED Camera OGN 节点)
- colcon: ~/.local/bin/colcon(非交互 shell 需 export PATH)
- foxglove_bridge: 未安装(需 sudo apt ros-jazzy-foxglove-bridge)

## 数据链路实测(只读验证)
- ZED 流服务器: 127.0.0.1:30000(实际协商到 30002, SHM-boost 后端)
- set_from_stream 客户端可连接: 相机型号 ZED X, 基线 120mm, SN 45626933
- grab 频率约 35-40Hz(墙钟);SDK 数据时间戳=Isaac 仿真时间(ns)
- 左/右图: BGRA 1920x1200 uint8;深度: 32FC1 米制, NaN=无效
- 点云: HxWx4 float32(XYZRGBA);IMU: 2026-10-31 03:35 起有真实数据
  (此前全零;用户在场景中补了 Imu_Sensor 后恢复)
- /clock: 检查时无发布者(仅本包订阅使其出现在话题列表)
- 机器人 prim 路径/场景文件名: 未知,待 diagnose_scene.py 输出

## pyzed 5.4 API 坑(实测记录,节点代码已规避)
- Orientation.get(i) 不可用(get() 只接受 0 参数);用 repr 解析 "x y z w"
- Matrix3f(协方差)不可迭代,直接迭代抛 'int' object is not subscriptable;
  节点用 _pyzed_matrix3_to_list(get_data() 或 repr 解析)
- 虚拟相机 get_camera_baseline()=0;基线从 stereo_transform 平移取(120mm)
- zed.open(init) 只接受 1 个位置参数
- PERFORMANCE 深度模式可用但 SDK 提示弃用(建议 NEURAL)
- LC_ALL=C 可消除标定解析警告(launch 已设)
