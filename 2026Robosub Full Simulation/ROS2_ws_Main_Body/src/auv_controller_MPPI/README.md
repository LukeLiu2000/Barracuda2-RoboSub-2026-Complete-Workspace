# auv_controller_MPPI

基于 MPPI(Model Predictive Path Integral)的 AUV 八推进器鲁棒控制器(ROS 2 Jazzy / C++17 / Eigen)。
几何参数取自 `robot_barracuda2` USD 模型实测读取,动力学为 Fossen 6-DOF 刚体+水动力近似模型,
支持常规控制与训练(数据采集 + 离线系统辨识)两种模式共用同一套核心。

## 包结构

```
auv_controller_MPPI/
├── CMakeLists.txt               # ament_cmake;核心库 mppi_core(纯 C++/Eigen,无 ROS 依赖)
├── package.xml
├── include/auv_controller_mppi/ # 核心算法头文件 + 控制器节点类
│   ├── types.hpp                # 13 维状态 / 目标 / 误差定义
│   ├── thruster_model.hpp       # 八推进器分布 + 6x8 分配矩阵 B
│   ├── dynamics_model.hpp       # Fossen 6-DOF 前向动力学
│   ├── cost_function.hpp        # MPPI 代价函数(YAML 全参数化)
│   ├── mppi_solver.hpp          # MPPI 求解器(重要性加权,OpenMP 并行 rollout)
│   ├── control_sequence.hpp     # H 步控制序列 + 时间戳采样
│   ├── sequence_executor.hpp    # 高频执行器(超时/激励旁路/急停看门狗)
│   └── mppi_controller_node.hpp # ROS 2 控制器节点类(头文件化以支持进程内测试)
├── src/                         # 上述实现 + 三个节点入口 + 基础任务闭环测试
├── scripts/system_identification.py  # 离线系统辨识(纯 numpy LM)
├── config/                      # thrusters/dynamics/mppi/training YAML
├── launch/                      # controller / training / basic_mission
├── test/                        # 6 个 gtest 套件(37 个用例)
└── docs/                        # 几何推导、算法说明、辨识指南、实测结果、对接指引
```

## 构建(隔离在包目录内)

按任务约束,构建产物全部落在包目录内(`.build/.install/.log`),不污染工作区:

```bash
cd ROS2_ws_Main_Body
source /opt/ros/jazzy/setup.bash
export PATH=$HOME/.local/bin:$PATH
colcon --log-base src/auv_controller_MPPI/.log build \
  --packages-select auv_controller_MPPI \
  --build-base  src/auv_controller_MPPI/.build \
  --install-base src/auv_controller_MPPI/.install \
  --cmake-args -DBUILD_TESTING=ON
```

测试:`cd src/auv_controller_MPPI/.build/auv_controller_MPPI && ctest --output-on-failure`

## 使用

```bash
source src/auv_controller_MPPI/.install/setup.bash
export ROS_LOG_DIR=$PWD/src/auv_controller_MPPI/.log/ros   # 日志也留在包内

# 常规模式:订阅 /auv/state/control(nav_msgs/Odometry)与 /auv/control/goal,
#            输出 /auv/control/thruster_cmd(Float64MultiArray, 8 维)
ros2 launch auv_controller_MPPI controller.launch.py

# 训练模式(随机目标 + 数据采集,数据落包内 data/)
ros2 launch auv_controller_MPPI training.launch.py mode:=random_goals
# 训练模式(推进器激励实验,提高辨识可辨识性)
ros2 launch auv_controller_MPPI training.launch.py mode:=excitation

# 基础任务闭环测试(无通信,前进 1 m + 偏航 30° + 停稳)
ros2 launch auv_controller_MPPI basic_mission.launch.py

# 系统辨识(训练数据 -> dynamics.yaml)
python3 scripts/system_identification.py --data <csv> --identify lin quad thrust
```

急停:`ros2 service call /mppi_controller_node/estop std_srvs/srv/SetBool '{data: true}'`

## 实测性能(服务器 RTX 5080 主机,单进程)

| 指标 | 实测 | 要求 |
|---|---|---|
| gtest | 6/6 套件通过(37 用例) | 全部通过 |
| 基础任务 | 到达:位置误差 0.059 m,角度误差 0.61°,线速度 0.018 m/s,角速度 0.023 rad/s | 前进 1 m + 偏航 30° 停稳 |
| MPPI 求解耗时 | 均值 36.8 ms / 峰值 44.2 ms(K=1000, H=15) | — |
| 可达求解频率 | 22.6 Hz | 10–20 Hz ✓ |
| 执行频率 | 100.008 Hz(实测 topic hz) | 50–100 Hz ✓ |

> 频率说明:以上为服务器 CPU 实测。Jetson Orin Nano 上需重新实测;
> rollouts 已 OpenMP 并行,核心库无 ROS 依赖、rollout 内核纯函数,可直接迁移 CUDA。

## 关键假设与待确认项(详见 docs/thruster_geometry.md)

1. 艇体长按 1 m 假设做**三轴统一缩放**(s = 1/0.6975 = 1.433692);
2. 质心 = 碰撞网格体积质心(USD 未 author COM,标记为计算值);
3. 水平推进器 t0–t3 推力轴 = 各自局部 +X(XZ 平面 ±45°)、垂直推进器 t4–t7 = 局部 +Y,
   **未经仿真驱动实测确认**——已参数化于 `config/thrusters.yaml`,待 Isaac 接通后标定;
4. 推力极限 4000 gf = 39.2266 N 正反对称(未标定);
5. 惯量/附加质量/阻尼均为箱体公式与经验量级估计,**必须**经系统辨识标定。

详细文档:[几何与 B 矩阵](docs/thruster_geometry.md) · [MPPI 算法](docs/mppi_algorithm.md) ·
[系统辨识指南](docs/system_identification.md) · [实测结果](docs/test_results.md) ·
[Isaac Sim 对接](docs/isaac_integration.md)
