# architecture —— 结构与数据流

## 1. 静态文件组织

```
ROS2Example/(工作区根,colcon 的工作目录)
├── launch/all.launch.py          工作区级总入口(不属于任何包)
├── test/test_full_system.py      工作区级集成测试(独立脚本)
├── docs/                         教学文档
├── src/                          ← 只有这里的目录会被当作包
│   ├── example_source/           C++ 包(ament_cmake)
│   │   ├── CMakeLists.txt        编译 2 个可执行 + 安装 launch/config
│   │   ├── package.xml           包声明(<build_type>ament_cmake</build_type>)
│   │   ├── src/*.cpp             节点源码
│   │   ├── config/source.yaml    参数默认值
│   │   └── launch/source.launch.py
│   └── example_processing/       Python 包(ament_python)
│       ├── setup.py / setup.cfg  安装脚本与入口位置
│       ├── package.xml           包声明(<build_type>ament_python</build_type>)
│       ├── example_processing/   内层同名目录 = Python 包本体
│       ├── config/processing.yaml
│       ├── launch/processing.launch.py
│       └── test/                 包内单元测试(colcon test 可发现)
├── build/ install/ log/          colcon 生成
├── runtime_logs/                 运行日志(ROS_LOG_DIR)
└── _verification/                验证临时产物
```

## 2. 运行时数据流(默认 namespace=demo)

```
┌──────────────────┐ /demo/count ┌─────────────────┐ /demo/square ┌──────────────┐ /demo/feedback ┌──────────────────┐
│  counter_node    │ ──────────> │  square_node    │ ───────────> │  report_node │ ─────────────> │  feedback_node   │
│ C++ 进程 (源:    │  Int64      │ Python 进程     │   Int64      │ Python 进程  │    String       │ C++ 进程         │
│ example_source)  │             │ (example_       │              │ (example_    │                │ (example_source) │
│  Timer 2Hz 递增  │             │  processing)    │              │  processing) │                │  打印+计数        │
└──────────────────┘             └─────────────────┘              └──────────────┘                 └──────────────────┘
        跨包                                   包内通信                                  跨包反向
```

- `count` / `feedback`:**跨包**通信(C++ ↔ Python,证明语言无关)。
- `square`:**同一包内**两个节点之间的通信。
- feedback_node 只打印,不回传给 counter → 不形成难懂的闭环。

## 3. Workspace / Package / 可执行入口 / Node / 进程 / 线程

| 概念 | 本示例中 | 说明 |
|---|---|---|
| Workspace | ROS2Example/ | 一个目录:src 里放包,build/install/log 是产物 |
| Package | example_source、example_processing | colcon 的构建与安装单位,由 package.xml 界定 |
| 可执行入口(executable) | counter_node、feedback_node、square_node、report_node | `ros2 run <包> <入口名>` 用的名字;C++ 来自 `add_executable`,Python 来自 console_scripts |
| Node(节点) | 与入口同名 | 代码里 `Node("counter_node")` 等创建的通信实体;话题挂在节点上 |
| 进程 | **4 个独立进程** | launch 每个节点默认起一个进程,互不共享内存,靠话题通信 |
| 线程 | 默认每个进程 1 个 spin 线程 | 单线程 Executor 顺序执行回调,本示例不引入多线程 |

**默认运行形态 = 4 个进程,每个进程各持 1 个节点。**
节点名 ≠ 进程名 ≠ 源文件名,详见 file_guide.md 第 3 节。

## 4. Launch 是什么、不是什么

`ros2 launch` 只做一件事:按 launch 文件的描述**把进程启动起来**
(读参数、拼命令行、设环境),之后它只负责转发各进程的屏幕输出和
统一发信号(Ctrl+C 时逐个结束进程)。

**Launch 不是消息转发中心**:四个节点之间的数据走 DDS 直接点对点,
launch 进程即使退出,已启动的节点间通信依然正常(只是失去统一管理)。

## 5. QoS 说明(所有端点统一)

```
KEEP_LAST + depth=10 + RELIABLE + VOLATILE
```

- **RELIABLE**:发送方确认送达、失败重试。它是"尽力可靠",
  **不等于无限缓存,也不等于绝不丢消息**——缓存只有 depth 那么深,
  订阅方处理不过来时,最深的一层(10 条)之外仍会丢。
- **VOLATILE**:不保留历史。**晚加入的订阅者不会收到它加入之前
  发布的消息**。这就是本示例里"先起发布者、后起订阅者会错过开头几条"
  的原因,也是集成测试不以"必须收到首发 0"为前提的原因。
- **KEEP_LAST(depth=10)**:每端只保留最新 10 条待处理消息。
