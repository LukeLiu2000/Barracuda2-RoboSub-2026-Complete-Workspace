# example_processing(Python / ament_python)

教学示例的"加工"包,包含两个 Python 节点和两个纯算法模块。

## 节点与模块

| 名称 | 种类 | 话题 | 说明 |
|---|---|---|---|
| square_node | ROS 节点 | 订阅 `count` → 发布 `square` | n → n²,超 Int64 范围告警并跳过 |
| report_node | ROS 节点 | 订阅 `square` → 发布 `feedback` | 每 batch_size 条输出批次统计文本 |
| number_math.py | 纯算法模块 | — | checked_square() 与 Int64 边界,不依赖 rclpy |
| batch_statistics.py | 纯算法模块 | — | BatchStatistics 批次统计类,不依赖 rclpy |

默认命名空间 demo 下的实际话题:`/demo/count` → `/demo/square` → `/demo/feedback`。
(square 是本包内部通信;count 来自 example_source,feedback 发回 example_source。)

## report_node 参数

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| batch_size | int | 5 | 每批条数,必须为正整数,否则启动时报错退出 |

默认值在 `config/processing.yaml`,由 `launch/processing.launch.py` 加载。

## 启动

```bash
# 整包(launch 自动带 namespace 与 YAML):
ros2 launch example_processing processing.launch.py

# 单节点(注意:ros2 run 不自动带 namespace/YAML,需显式传):
ros2 run example_processing square_node --ros-args -r __ns:=/demo
ros2 run example_processing report_node --ros-args -r __ns:=/demo -p batch_size:=3

# 没有上游时手动喂数据验证:
ros2 topic pub /demo/count std_msgs/msg/Int64 "{data: 7}" -r 1
```

## 单元测试

```bash
# 方式一:colcon(在工作区根目录执行)
colcon test --packages-select example_processing
colcon test-result --verbose

# 方式二:直接跑(在 src/example_processing/ 目录下)
python3 -m pytest test/ -v
```

覆盖:number_math 的 0/正/负/边界平方;BatchStatistics 的非法 batch_size、
不足一批、正好一批、批次隔离、实例独立性。
