# ROS2Example —— ROS 2 教学示例工作区

独立、完整、可构建可运行的 ROS 2 教学工作区。**与正式的
`ROS2_ws_Main_Body` 完全无关**:不依赖它的代码/消息/构建结果,也不连接
Isaac Sim、ZED 或任何外部服务。只使用服务器自带的 ROS 2 Jazzy 标准环境。

## 环境要求(实测于 liuhao-5080,Ubuntu 24.04)

- ROS 2 **Jazzy**(`/opt/ros/jazzy`)
- Python 3.12 / pytest 8.3 / CMake 3.28 / g++ 13
- colcon 在 `~/.local/bin`(交互终端通常已包含;脚本中需 `export PATH`)

## 快速开始(新终端,逐条复制;路径含空格,注意引号)

```bash
cd '/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/2026Robosub Full Simulation/Develop_Reaserch_ws/ROS2Example'

source /opt/ros/jazzy/setup.bash
export PATH="$HOME/.local/bin:$PATH"

colcon build --symlink-install          # 产生 build/ install/ log/
source install/setup.bash               # 每个新终端都要重新 source

ros2 launch launch/all.launch.py        # 一次启动全部 4 个节点
# 观察完毕后 Ctrl+C 结束(会整组退出)
```

### 为什么总入口用"文件路径"启动?

`launch/all.launch.py` 是**工作区级**文件,不属于任何包——没有也不应该
为它造一个"第三方包"。`ros2 launch <包名> <文件>` 只能启动已安装进某个包
share 目录的文件,所以这里用路径 `ros2 launch launch/all.launch.py`。
它内部通过 ament 索引定位两个子包的 share 目录,与当前工作目录无关。

## 三种启动方式

| 方式 | 命令 | 说明 |
|---|---|---|
| A. 单节点 | `ros2 run example_source counter_node --ros-args -r __ns:=/demo -p publish_rate_hz:=5.0` | `ros2 run` **不会**自动带 launch 里的 namespace 和 YAML 参数,要显式传 |
| B. 单包 | `ros2 launch example_source source.launch.py`<br>`ros2 launch example_processing processing.launch.py` | 两个终端各启动一个包、用相同 `namespace:=demo`,即组成完整链路;单独启动 processing 时可用 `ros2 topic pub /demo/count ...` 手动喂数据 |
| C. 全系统 | `ros2 launch launch/all.launch.py` | 4 节点一次起齐 |

⚠️ **不要重复启动已经在运行的同一套节点**:同一话题出现多个发布者,
下游会重复处理、输出翻倍。先 Ctrl+C 掉旧的再起新的。

## 目录结构

```
ROS2Example/                 ← 工作区根(colcon 在这里执行)
  README.md                  ← 本文件
  docs/                      ← 教学文档(架构/文件/步骤/验证报告)
  launch/all.launch.py       ← 总启动入口(工作区级,不属于任何包)
  test/test_full_system.py   ← 工作区级集成测试(独立脚本,见下)
  src/
    example_source/          ← C++ 包(ament_cmake):counter_node, feedback_node
    example_processing/      ← Python 包(ament_python):square_node, report_node
  build/ install/ log/       ← 构建产生(colcon 自动生成)
  runtime_logs/              ← 运行与测试日志(ROS_LOG_DIR 指向这里)
  _verification/             ← 验证用临时产物(如 Python 缓存)
```

## 两种包的区别(本示例刻意各做一个)

| | example_source | example_processing |
|---|---|---|
| 构建类型 | ament_cmake | ament_python |
| 构建脚本 | CMakeLists.txt | setup.py + setup.cfg |
| 可执行入口 | `add_executable` + `install(TARGETS …)` | `entry_points` console_scripts |
| 语言/库 | C++ / rclcpp | Python / rclpy |

两者都用 `package.xml` 声明元数据与依赖;`build_type` 决定 colcon 用哪套流程。

## 测试

```bash
# 单元测试(两个包,colcon 会发现 src/ 下包内的 test/)
colcon test --packages-select example_processing
colcon test-result --verbose

# 集成测试(工作区根的独立脚本,colcon test 不会发现它,需手动跑)
source install/setup.bash
python3 test/test_full_system.py
```

## 推荐阅读顺序

1. 本 README → 快速跑起来
2. `docs/architecture.md`(结构与数据流)
3. `docs/learning_steps.md`(10 个小实验)
4. `docs/file_guide.md`(每个文件为什么存在)
5. `docs/verification_report.md`(实测记录)
