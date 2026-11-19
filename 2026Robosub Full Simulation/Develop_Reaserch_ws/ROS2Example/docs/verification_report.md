# verification_report —— 实测验证报告

日期:2026-10-30(服务器 liuhao-5080)

## 1. 实测环境版本

| 项 | 版本 |
|---|---|
| 发行版 | ROS 2 **Jazzy**(`/opt/ros/jazzy`,实测确认,非推测) |
| 系统 | Ubuntu 24.04(服务器既有环境记录) |
| Python | 3.12.3 |
| pytest | 8.3.5 |
| CMake / g++ | 3.28.3 / 13.3.0 |
| colcon | colcon-core 0.21.1 等(pip --user,`~/.local/bin`;非交互 shell 需 `export PATH="$HOME/.local/bin:$PATH"`) |

## 2. 构建命令与结果(已验证)

```bash
cd '<工作区根>' && source /opt/ros/jazzy/setup.bash
export PATH="$HOME/.local/bin:$PATH"
colcon build --symlink-install
```

结果:`Summary: 2 packages finished [3.97s]`,退出码 0。
`install/` 内 4 个可执行入口(`ros2 pkg executables` 实测):

```
example_source counter_node / example_source feedback_node
example_processing square_node / example_processing report_node
```

过程中发现并修复的问题(记录供学习):
- 首次构建失败:`<maintainer>` 缺少 `email` 属性(catkin_pkg 校验)。已修正。
- `colcon test` 首次报 NO TESTS RAN:colcon 对 ament_python 默认用
  `python -m unittest`,而 Python 3.12 的 unittest 发现不支持无
  `__init__.py` 的 test/ 目录。在 setup.py 声明 `tests_require=['pytest']`
  后 colcon 切换 pytest 运行器,正常发现。

## 3. 测试命令与结果(已验证)

单元测试:

```bash
colcon test --packages-select example_processing
colcon test-result --verbose
```

结果:**17 tests, 0 errors, 0 failures, 0 skipped**
(number_math 8 项:0/正/负/±1/最大界内 3037000499/越界 3037000500/
Int64 极值;batch_statistics 9 项:非法 batch_size、不足批、恰好一批、
负值统计、批次隔离、批号递增、实例独立、格式化)。

集成测试(`python3 test/test_full_system.py`,自动使用独立
ROS_DOMAIN_ID 隔离,进程组方式启动/清理):

- 阶段 A(全系统 all.launch.py,ns=demo):**14 项 PASS**
  - 4 节点全部出现在图:/demo/counter_node、/demo/square_node、
    /demo/report_node、/demo/feedback_node
  - 三话题类型:`/demo/count [Int64]`、`/demo/square [Int64]`、
    `/demo/feedback [String]`(ros2 topic list -t 输出)
  - count 流 10 条、严格递增步长 1(首 0 末 9);square 与 count 对应
  - feedback 2 批、批次号递增、每批 count=5;首批 sum=30 min=0 max=16,
    与实测 5 条 square(0,1,4,9,16)完全一致
- 阶段 B(受控输入,processing.launch.py,ns=ctrl):**5 项 PASS**
  - 先发超界值 3037000500(被跳过、节点不崩溃),再发 3..7
  - square 精确为 [9,16,25,36,49];feedback 精确为
    `batch=1 count=5 sum=135 min=9 max=49`
  - 全程不以"收到首发 0"为前提(输入从 3 开始)
- 证据来源为订阅到的消息流,非日志文本匹配;测试订阅者
  (integration_test_checker / driver)独立于 4 个业务节点。

退出码 0。清理实测:脚本结束后 `ps` 复查无任何残留节点进程
(采用进程组 SIGINT→SIGKILL;此前 SIGTERM 单杀 launch 会留孤儿,
已修复并复测)。

## 4. 三种启动方式实测(已验证)

| 方式 | 命令要点 | 实测证据 |
|---|---|---|
| A 单节点 | `ros2 run example_source counter_node --ros-args -r __ns:=/demo -p publish_rate_hz:=5.0` | `ros2 topic hz` 实测 **average rate: 4.999**;echo 取到 data:33/39;节点日志首行 `rate=5.00Hz`(命令行参数生效) |
| B 单包 launch | source.launch.py / processing.launch.py | `ros2 param get` 实测 publish_rate_hz=**2.0**、start_value=**0**(YAML 生效);双包同 ns 链路接通,feedback 采样 `batch=4 count=5 sum=1455 min=225 max=361` = 15²..19² |
| C 全系统 | `ros2 launch launch/all.launch.py` | 集成测试阶段 A,4 节点 3 话题全部验证 |

顺带的教学实证:方式 B 采样时订阅端晚加入,收到的是 batch=4 而非
batch=1——VOLATILE 不向晚加入者补发历史,亲眼可见。

## 5. 边界与日志检查(已验证)

- `ROS_LOG_DIR` 指向 `runtime_logs/`:所有节点 rcutils 日志
  (counter_node_*.log 等)、launch 输出、集成测试日志均在其中。
- `~/.ros/log` 未因本工程新增内容(其中仅存既往 Isaac Sim 的
  kit_*.log,与本工程无关)。
- Python 缓存:__pycache__ 位于 src 树内;`PYTHONPYCACHEPREFIX`
  统一收进 `_verification/pycache`(集成测试与验证脚本均设置)。
- colcon 产物 build/ install/ log/ 均在工作区根。
- 目录体积:build 14M / install 392K / log 400K / runtime_logs 296K /
  _verification 14M。

## 6. 已验证 vs 仅推测

**已验证**:构建与安装、4 入口、单元测试 17 项、集成测试 19 项、
三种启动方式、YAML 与命令行参数生效、namespace(demo/ctrl 实测,
launch 参数可改)、话题类型、QoS VOLATILE 行为、日志与缓存边界、
无残留进程。

**仅推测/未实测**:
- RELIABLE depth=10 的丢包行为(需人为构造订阅端背压,未做);
- counter_node 的 Int64 溢出停止路径(逻辑已实现:递增前检查上/下限,
  越界报错并 cancel timer;平方侧同类逻辑有单元测试,计数侧靠代码
  审查,未实跑 ~9.2e18 的计数);
- 多机/真实网络环境(全部验证在单机回环完成)。

## 7. 已知注意事项(非缺陷)

- 脚本化启动本示例时:非交互 shell 的后台任务 SIGINT 被忽略;
  单独 SIGTERM `ros2 launch` 会留下孤儿节点。脚本请参考
  `test/test_full_system.py` 的进程组方式(start_new_session + killpg),
  或 `setsid ... &` 后对整组 TERM。
- colcon 位于 `~/.local/bin`,非交互 shell(ssh 脚本、cron)需手动
  `export PATH="$HOME/.local/bin:$PATH"`。
