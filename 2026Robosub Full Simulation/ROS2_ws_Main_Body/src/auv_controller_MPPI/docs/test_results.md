# 实测结果记录(服务器 liuhao-5080,ROS 2 Jazzy,2026-10-31)

所有数字均为真实运行输出,复现命令见各节。

## 1. 单元/集成测试(ctest,6 目标 40 用例)

```
$ cd src/auv_controller_MPPI/.build/auv_controller_MPPI && ctest --output-on-failure
100% tests passed, 0 tests failed out of 6      (总耗时 8.78 s)
```

| 套件 | 用例数 | 覆盖 |
|---|---|---|
| test_thruster_model | 8 | 方向单位性、B 列 = [d; r×d]、秩/条件数、限幅、最小范数分配回代 |
| test_dynamics_model | 7 | 中性悬浮不动点、重力下沉速度解析值、恢复力矩符号与量级、纯偏航角动量守恒、前进加速度解析值、四元数双覆盖、13 维往返 |
| test_mppi_solver | 7 | 权重归一非负、λ→大权重趋均匀(熵→lnK)、λ→小权重集中、闭环迭代代价下降(60 轮 < 50% 初值)、输出尊重推力极限、到达目标后近零推力、同 seed 逐位复现 |
| test_cost_function | 8 | 零点、位置/姿态(测地角)项、终端倍率、饱和 hinge、平滑度、工作空间违反、H 步积分 |
| test_control_sequence | 8 | 时间戳采样、时钟回退、ACTIVE/STALE 超时置零、新旧序列即时替换、激励旁路与超时、激励限幅、急停最高优先 |
| test_ros_interface | 2 | 进程内真实节点:3 s 收 ≥80 条 8 维有限 thruster_cmd 且推力非零;状态断流 2.5 s 后必须零推力 |

(用例计数含修复过程中修正过的 4 处测试自身漏项:饱和测试漏算 effort 项、
工作空间测试漏算位置项、采样测试 H 与断言不一致、收敛测试固定初态与
warm-start 左移语义冲突——均已改为与实现语义一致的断言,实现侧修复见下。)

## 2. 基础控制任务(任务书第十二章:静息 → 前进 1 m + 偏航 30° → 停稳)

`ros2 launch auv_controller_MPPI basic_mission.launch.py`(K=1000, H=15, dt=0.1)

```
RESULT:arrived=1
RESULT:final_pos_error_m=0.0586        (容差 0.10 m)
RESULT:final_ang_error_deg=0.61        (容差 5°)
RESULT:final_lin_vel=0.0180 m/s        (容差 0.10)
RESULT:final_ang_vel=0.0226 rad/s      (容差 0.15)
RESULT:solve_ms_mean=36.75 / max=44.15
RESULT:capable_solver_hz=22.6          (要求 10–20 Hz)
RESULT:virtual_mission_time_s=5.47     wall 2.04 s(快于实时)
```

优化前同任务 solve_ms 均值 445.63(2.2 Hz,不达标);优化内容:
对角质量阵免 LLT、Coriolis 叉积直算、四元数共轭代替旋转矩阵、常量缓存、
OpenMP 并行 rollout(细节 docs/mppi_algorithm.md §5)。

## 3. 常规模式 ROS 冒烟(controller.launch.py)

合成 20 Hz 里程计 + 单次目标(目标:z+1 m;合成数据仅用于接口验证):

- `/auv/control/thruster_cmd` 采样:8 维全部有限且非零
  `[6.62, 8.25, −2.55, −1.20, −3.63, −4.10, 6.57, −6.49]`;
- 诊断快照:`status=ACTIVE estop=0 solve_ms=36.995 solver_count=34 exec_count=1099
  state_age_s=0.013`;
- `ros2 topic hz /auv/control/thruster_cmd`:**平均 100.008 Hz**(要求 50–100 Hz);
- 启动日志(真实节点运行时校验):
  `allocation matrix B: rank=6 cond=5.50 sv=[2.000 1.422 1.421 1.031 0.687 0.364]`
  与离线计算一致(docs/thruster_geometry.md §5)。

## 4. 训练模式冒烟(training.launch.py)

**random_goals**(合成 20 Hz 状态,16 s):
- mission 节点生成目标 `新目标 #1: pos=(0.34,0.95,-0.01)`(工作空间夹取生效);
- collector 输出 `training_20261031_152250.csv` 共 **164 行数据**
  (表头 37 列:t, experiment_id, x13, u8, x_next13, dt;experiment_id=goal_0001)。

**excitation**(chirp,合成状态):
- `/auv/training/excitation_cmd` 输出 B 伪逆分配后的 8 维推力
  `[7.02, 6.91, −6.91, −7.02, 2.47, −2.47, −2.47, ...]`;
- 控制器诊断 `status=EXCITATION`(旁路独占总线,solve 挂起),exec 100 Hz 正常。

## 5. 构建与隔离

- 隔离构建(`--build-base/--install-base/--log-base` 均在包内)通过,
  无授权目录外产物;ROS 运行日志重定向 `ROS_LOG_DIR=包内/.log/ros`;
- 编译期仅 1 处 -Wunused 警告(已消除);-Wall -Wextra -Wpedantic 干净;
- 依赖服务器已有组件:rclcpp / 标准消息包 / Eigen3 / gtest / OpenMP,零新增系统依赖。

## 6. 未测/待实测项(如实声明)

- **Isaac Sim 联合仿真未接通**(任务书允许:推力执行端在授权目录外)——
  全部推力输出仅到 `/auv/control/thruster_cmd` 话题为止;
- Jetson Orin Nano 上的频率未实测(当前 22.6 Hz 为服务器 x86 数据);
- 系统辨识脚本未在真实仿真数据上跑过(需要先接通 Isaac 产生真实状态流;
  脚本自检依赖 numpy 语法检查,LM 内核与 C++ 单步一致由构造保证)。
