# file_guide —— 逐文件说明:为什么存在、谁读取、何时生效

## 1. 构建与安装类文件

| 文件 | 谁读取 | 何时生效 | 作用 |
|---|---|---|---|
| `CMakeLists.txt` | CMake + colcon(仅 C++ 包) | `colcon build` 时 | 声明怎么编译 `src/*.cpp` 成可执行文件、安装哪些文件。**运行期不再读取** |
| `setup.py` | setuptools + colcon(仅 Python 包) | `colcon build` 时 | 声明 Python 包本体、data_files(launch/config/package.xml)、console_scripts 入口。**运行期不再读取** |
| `setup.cfg` | setuptools | `colcon build` 时 | 把入口脚本装到 `lib/<包名>/`,`ros2 run` 才找得到。仅 3 行,却不可缺 |
| `package.xml` | colcon、ament 索引、ros2 CLI | 构建时 + 运行时 | 包的身份卡:名字、依赖、`build_type` 决定用 CMake 还是 setuptools 流程;安装后进 ament 索引,`ros2 pkg`/launch 靠它找包 |
| `resource/example_processing` | ament_index_python | 安装后 | 空的资源索引标记,缺失则"包已安装"无法被识别。内容不重要,存在才重要 |
| 内层同名目录 `example_processing/` | Python import 机制 | 运行期 | Python 包本体。**外层目录只是"包的行政外壳",内层才是可 import 的模块** |
| `__init__.py` | Python import 机制 | 每次导入包时 | 让目录成为包。可为空;不放业务代码,更不放节点启动逻辑 |

## 2. 配置与启动类文件

| 文件 | 谁读取 | 何时生效 | 作用 |
|---|---|---|---|
| `config/*.yaml` | 各节点自身(rclcpp/rclpy 参数系统) | **进程启动时一次性** | 参数默认值;launch 通过 `parameters=[文件]` 传入。顶层 `/**` 通配,换 namespace 仍生效 |
| `launch/*.py` | `ros2 launch`(Python) | 启动那一刻 | 描述"起哪些进程、什么参数、什么 namespace"。启动完成后不再参与 |
| 工作区根 `launch/all.launch.py` | `ros2 launch`(按文件路径) | 启动那一刻 | Include 两个包的 launch 文件;不属于任何包 |
| 工作区根 `test/test_full_system.py` | 开发者手动执行 | 手动跑时 | 集成测试;**colcon test 不会发现它**(它不在任何包里) |

## 3. 三种"名字"的区别(易混淆)

以 counter 为例:

| 名字 | 值 | 定义处 | 用在哪 |
|---|---|---|---|
| 源文件名 | `src/counter_node.cpp` | 你建文件时 | 仅 CMakeLists 引用;编译后即无意义 |
| 可执行名(executable) | `counter_node` | C++:`add_executable(counter_node …)`;Python:console_scripts | `ros2 run <包> counter_node` |
| 节点名(Node name) | `counter_node` | 代码里 `Node("counter_node")` | `ros2 node list`、话题全名 `/demo/count` 的挂靠点、YAML 参数匹配 |

三者可以互不相同(比如源文件叫 `count_pub.cpp`、可执行叫 `counter_node`、
节点叫 `front_counter`)。本示例刻意保持同名,是为了降低初学负担;
但要记住:改名时三处要分别改,互不联动。

## 4. 算法模块 vs 节点文件

- `number_math.py`、`batch_statistics.py`:**纯 Python 模块**。
  不 import rclpy、不创建节点,任何 Python 环境都能用,因此单元测试
  不需要启动 ROS。
- `square_node.py`、`report_node.py`:**节点文件**。只做 ROS 通信
  (订阅/发布/参数/日志),计算全部委托给算法模块。
- C++ 侧因体量小,算法直接写在节点里(溢出检查内联于 `on_timer`)。

## 5. "文件不会每条消息跑一次"

构建类文件(CMakeLists/setup.py)只在 `colcon build` 时执行;
launch/yaml 只在进程启动瞬间读取;节点程序也只在启动时执行一次
`main()` —— 之后 `spin` 循环里反复运行的只有**回调函数**
(`on_timer`、`_on_count`、`_on_square`),它们是进程内普通的
函数调用,不重新读文件、不重新起进程。

## 6. 产物与忽略标记

| 目录 | 产生者 | 内容 |
|---|---|---|
| `build/` | colcon | 中间产物(C++ 对象文件、setuptools 前缀) |
| `install/` | colcon | **真正被运行的**:可执行、share 里的 launch/config、环境脚本 `setup.bash` |
| `log/` | colcon | 各次构建/测试日志 |
| `runtime_logs/` | 本工程约定 | 运行与集成测试日志(`ROS_LOG_DIR` 指向这里) |
| `_verification/` | 本工程约定 | 验证临时产物(`PYTHONPYCACHEPREFIX` 等) |

`docs/ launch/ test/ runtime_logs/ _verification/` 里放了 `COLCON_IGNORE`
空标记文件:让 colcon 的包发现跳过这些目录(它们不是包,也避免
在生成物里递归扫描)。删掉它们不影响运行,只是 colcon 会多扫几个目录。
