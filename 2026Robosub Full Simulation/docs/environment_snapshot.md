# 环境快照 — auv_sensing 环境发现(2026-09-08)

只读发现结果。生成者:编码 Agent(Claude)。不含任何认证信息。

## 1. 系统与版本(已验证)
| 项 | 值 | 证据 |
|---|---|---|
| OS | Ubuntu 24.04.4 LTS(kernel 7.0.0-28-generic) | /etc/os-release, uname |
| ROS 2 | jazzy(/opt/ros/jazzy,setup.bash 存在) | ls /opt/ros |
| 系统 Python | 3.12.3(系统 rclpy 基于 3.12) | python3 --version |
| Isaac Sim | 5.1.0-rc.19+release.26219.9c81211b.gl,安装于 /isaacsim | cat /isaacsim/VERSION |
| Isaac 自带 Python | 3.11(kit/python/bin/python3.11),ros2 bridge 自带 jazzy rclpy | /isaacsim/kit/python |
| ROS 2 Bridge 扩展 | isaacsim.ros2.bridge(另有 sim_control/tf_viewer/urdf)已随 Isaac 安装 | ls /isaacsim/exts |
| ZED SDK | 5.4.0(Ubuntu24 cuda12.8 tensorrt10.9),/usr/local/zed,pyzed 5.4 | 安装包名 + pip list |
| ZED Isaac 扩展 | sl.sensor.camera 源码 + 构建于 ~/workspaces/zed-isaac-sim(已构建 _build,libsl_zed.so 已链接) | 目录检查 |
| RMW | rmw_fastrtps_cpp(mature 工程惯例:FASTDDS_BUILTIN_TRANSPORTS=UDPv4 + large-buffer profile) | run_isaac_barracuda.sh |
| ROS_DOMAIN_ID | 工程惯例 DEV_ID=1 → DOMAIN 0(可覆盖) | run_isaac_barracuda.sh |

## 2. 工程结构(已验证)
- 工程根:/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/2026Robosub Full Simulation/(含空格)
- ROS ws:ROS2_ws_Main_Body/src/{auv_sensing, auv_bringup, auv_controller_MPPI, auv_interfaces}(其余三个为空目录;auv_sensing 为本次实现目标,此前只有一个 0 字节 state_adapter_node.py,经用户明示已删)
- **整个工程不是 git 仓库**(无版本控制;建议后续 git init,本次未执行)
- 关联成熟工程:scenes/Robosub2026 Enviroment/(含 barracuda.auv_control Isaac 扩展、barracuda_ws(空)、docker、SENSORS.md/README.md/CLAUDE.md)

## 3. 场景与 Prim(离线 USD 解析,pxr 只读)
| 场景 | ZED | AUV 刚体 | 备注 |
|---|---|---|---|
| scenes/Full Process AUV Simulation 2026/mainscene.usd | 无 | 无 | **空场景(0 prims)**,占位 |
| scenes/Full Process AUV Simulation 2026/robot_barracuda/barracuda.usd | **ZED M**(barracuda_zedm_* 链接树:zedm_base_link→camera_link→zedm_camera_center→{imu_link, left/right camera (+optical) frames}) | **ArticulationRoot + RigidBody:/barracuda/barracuda_base_link**;另有 barracuda_barracuda_link(RigidBody)、8 推进器、ping1d/ping360、DVL | upAxis=Z,mpu=1.0;部分 visuals 引用断链 |
| scenes/Full Process AUV Simulation 2026/robot_barracuda2/Barracuda 2.usd | **ZED_X**(/World/barracuda2/base_link/zed_link/ZED_X,引用云端资产,实测 HTTP 200 可达) | /World/barracuda2/base_link(RigidBody,无 ArticulationRoot) | upAxis=**Y**,mpu=0.01 |
| scenes/robotsub_enviroment.usd | **ZED_X_Mini**(/World/ZED_Control_01/ZED_X_Mini,引用云端资产) | 无 AUV | 有 /World/ActionGraph(OmniGraph)、/World/Camera |
| scenes/Robosub2026 Enviroment/barracuda-world.usd | — | /World/barracuda(**引用断链**:指向另一台机器 /home/barracuda/... 路径) | 有 /World/ActionGraph |

## 4. 帧/话题约定(来自 barracuda.auv_control 扩展源码,强证据)
- ODOM_FRAME="odom";BASE_LINK_FRAME="barracuda/base_link";IMU_FRAME="barracuda/imu_link"(USD 侧 barracuda_imu_link)
- 已存在发布:IMU /barracuda/vectornav/imu(sensor_msgs/Imu,FLU 体系加速度/角速度,**姿态按 NED 发布**模拟 VN-100,+9.81 重力已补)、/barracuda/pose(PoseStamped)、/clock;订阅 /barracuda/wrench
- 注意:该扩展的 IMU 是 vectornav 仿真,**不是 ZED wrapper**;Isaac 用自带 rclpy(py3.11),外部节点用系统 jazzy(py3.12),两侧通过 DDS 通信(工程脚本先剥掉 /opt/ros 环境变量再启动 Isaac)

## 5. 六项不可假定项——当前状态
| 项 | 状态 | 说明 |
|---|---|---|
| ① ZED camera_model | **未冻结(三候选)** | zedm(barracuda.usd)/ zedx(Barracuda 2.usd)/ zedxm(robotsub_enviroment.usd);正确 Stage 未上机确认 |
| ② ZED namespace/node name | **缺失** | **ZED ROS 2 Wrapper 未安装**(全盘无 zed_wrapper;/opt/ros 无 zed 包) |
| ③ sim_address/sim_port | **未发现** | ZED Camera Helper 的 Action Graph 配置未能离线导出;IPC 同机惯例 127.0.0.1,未验证 |
| ④ AUV body Prim path | 候选已确认 | /World/barracuda/barracuda_barracuda_link(barracuda.auv_control PRIM_PATH);ArticulationRoot=/barracuda/barracuda_base_link(barracuda.usd) |
| ⑤ frame 名 | 强证据 | odom / barracuda/base_link / barracuda/imu_link(扩展源码);ZED 光学 frame 树见上表 |
| ⑥ 原始 GT Topic/类型 | **不存在** | 当前无任何 nav_msgs/Odometry Ground Truth 发布器;仅 /barracuda/pose(PoseStamped)。需新增 Isaac-side Publisher(见报告) |

## 6. live 状态
- 发现时 **无任何 Isaac/ROS/ZED 进程运行**;未做任何实时 Topic/QoS/TF/频率测量。
- 历史启动方式(bash_history,证据):
  - /isaacsim/isaac-sim.streaming.sh --/app/livestream/publicEndpointAddress=172.30.30.1 --/app/livestream/port=49100 --enable sl.sensor.camera
  - /isaacsim/isaac-sim.sh --ext-folder /home/liuhao/workspaces/zed-isaac-sim/exts --enable sl.sensor.camera
- 成熟工程启动:scripts/run_isaac_barracuda.sh(60Hz 主循环限速、--no-ros-env、AUTOPLAY=1 可无人值守按 Play)

## 7. 集成 Blocker(实现完成但集成受阻的原因)
1. ZED ROS 2 Wrapper 未安装(需 git clone zed-ros2-warehouse 的 zed-ros2-wrapper 并 colcon build,或由用户决定)
2. 正确 Stage 未确认(mainscene.usd 为空;需用户指认或上机组装)
3. 无 GT Odometry 发布器(建议:Action Graph 中对 /World/barracuda/barracuda_barracuda_link 加 isaacsim.ros2.bridge 的 Odometry publisher;或复用 /barracuda/pose 由 GT Node 转换——后者类型不符 PDF 推荐路径,列为备选)
