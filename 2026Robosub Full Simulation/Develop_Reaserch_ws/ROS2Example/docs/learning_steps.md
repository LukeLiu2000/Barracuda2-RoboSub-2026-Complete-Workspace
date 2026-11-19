# learning_steps —— 10 个小实验

前提(每个新终端):

```bash
WS='/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/2026Robosub Full Simulation/Develop_Reaserch_ws/ROS2Example'
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 1. 只启动 counter,查看 count

```bash
ros2 run example_source counter_node --ros-args -r __ns:=/demo
# 另开一个终端(同样 source 后):
ros2 topic echo /demo/count
ros2 topic hz /demo/count      # 约 2 Hz
```
看点:`ros2 run` 不经过 launch,namespace 要自己传(`-r __ns:=/demo`),
否则话题是 `/count`。

## 2. 启动 square,查看 square

保留实验 1 的 counter,再开终端:

```bash
ros2 run example_processing square_node --ros-args -r __ns:=/demo
ros2 topic echo /demo/square   # 0,1,4,9,16,...
```

## 3. 启动 report,查看 feedback

```bash
ros2 run example_processing report_node --ros-args -r __ns:=/demo
ros2 topic echo /demo/feedback # 每凑满 5 条输出: batch=1 count=5 sum=30 min=0 max=4
```

## 4. 启动 feedback_node,观察 C++ 收 Python 的反馈

```bash
ros2 run example_source feedback_node --ros-args -r __ns:=/demo
# 输出形如: [第 1 条反馈] batch=1 count=5 sum=30 min=0 max=4
```
看点:消息由 Python 节点产生、C++ 节点消费,语言无关。

## 5. 两个 Package Launch 启动(先关掉上面所有节点)

```bash
# 终端 1
ros2 launch example_source source.launch.py
# 终端 2
ros2 launch example_processing processing.launch.py
```
看点:两个终端、各自 launch,namespace 都是 demo,链路自动接通。
若只想验证 processing,可手动喂数据:

```bash
ros2 topic pub /demo/count std_msgs/msg/Int64 "{data: 7}" -r 1
```

## 6. 总 Launch 一次启动(先 Ctrl+C 掉上面两组)

```bash
ros2 launch launch/all.launch.py
```
看点:4 个节点一个命令;Ctrl+C 一次全停。
**不要**在已有节点运行时再次启动同一套(同话题双发布者 → 重复处理)。

## 7. 修改频率、起始值、batch_size,验证参数

```bash
# launch 传参:频率 5Hz、起点 100
ros2 launch launch/all.launch.py
# (Ctrl+C 后)用 YAML/命令行对比:
ros2 run example_source counter_node --ros-args -r __ns:=/demo \
  -p publish_rate_hz:=5.0 -p start_value:=100 -p increment:=-2
# batch_size 改为 3:
ros2 launch example_processing processing.launch.py
# 查询运行中的参数:
ros2 param get /demo/counter_node publish_rate_hz
ros2 param get /demo/report_node batch_size
```
改 `config/*.yaml` 里的默认值后重新 launch,同样生效。
非法值实验:`-p publish_rate_hz:=0` → 节点报错退出;
`-p batch_size:=-1` → report_node 报错退出。

## 8. 停掉一个节点,观察其他节点

全系统运行中,Ctrl+C 单独的 counter(或 `ros2 lifecycle` 不适用,直接
结束进程):square/report/feedback 不崩溃,只是停止产生新数据;
恢复 counter 后链路自动接上——DDC 发现机制会自动重连。
(注意:VOLATILE 语义下,断开期间的旧消息不会补发。)

## 9. 更改 namespace,观察 Topic 变化

```bash
ros2 launch launch/all.launch.py namespace:=lab2
ros2 topic list    # /lab2/count /lab2/square /lab2/feedback
ros2 node list     # /lab2/counter_node ...
```
看点:代码里全是相对话题名,namespace 只在 launch 层拼接;
YAML 用 `/**` 通配,换 namespace 后参数仍然生效。

## 10. 改了不同文件,要重建还是重启?

| 改动 | 需要 |
|---|---|
| `*.cpp`、CMakeLists.txt | **重新 colcon build**(C++ 要编译)+ 重启节点 |
| `*.py` 节点/算法源码 | 重启进程即可(本工作区 `--symlink-install` 构建,src 改动即时可见;新增文件/入口除外 → 重建) |
| `config/*.yaml` | 重启节点(参数启动时读一次);symlink 安装下改文件即生效,无需重建 |
| `launch/*.py` | 重新 ros2 launch(每次启动都会重读) |
| `package.xml` 依赖 | 重新 colcon build |

经验法则:不确定就 `colcon build` 一次(很快)+ 重新 launch。
