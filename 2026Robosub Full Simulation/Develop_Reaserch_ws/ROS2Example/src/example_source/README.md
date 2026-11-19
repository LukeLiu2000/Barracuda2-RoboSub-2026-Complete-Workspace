# example_source(C++ / ament_cmake)

教学示例的"数据源"包,包含两个互相独立的 C++ 节点。

## 节点

| 节点 | 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|---|
| counter_node | 发布 | `count`(相对名) | std_msgs/msg/Int64 | 从 start_value 起按 increment 递增,默认 2 Hz |
| feedback_node | 订阅 | `feedback`(相对名) | std_msgs/msg/String | 打印反馈内容与累计条数 |

默认命名空间为 `demo` 时,实际话题是 `/demo/count` 与 `/demo/feedback`。

## counter_node 参数

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| publish_rate_hz | double | 2.0 | 发布频率,必须 > 0,否则报错退出 |
| start_value | int | 0 | 计数起点 |
| increment | int | 1 | 步长(可为负);递增即将溢出 Int64 时报错并停止发布 |

参数默认值在 `config/source.yaml`,由 `launch/source.launch.py` 加载。

## 单独启动(整个包)

```bash
ros2 launch example_source source.launch.py
ros2 launch example_source source.launch.py namespace:=demo2
```

只启动本包时:counter 正常发布,feedback_node 安静等待(没有上游就没有输出,不是卡死)。

## 单节点启动(注意:不会自动带 namespace 和 YAML)

`ros2 run` 不经过 launch,不会自动加命名空间、不会自动加载 YAML:

```bash
# 显式传入命名空间与参数,效果才等价于 launch:
ros2 run example_source counter_node --ros-args -r __ns:=/demo \
  -p publish_rate_hz:=5.0 -p start_value:=100
ros2 run example_source feedback_node --ros-args -r __ns:=/demo
```

## 文件结构

```
example_source/
  CMakeLists.txt   # 编译两个可执行文件并安装 launch/config
  package.xml      # ament_cmake 包声明与依赖
  src/counter_node.cpp    # 计数发布节点
  src/feedback_node.cpp   # 反馈接收节点
  config/source.yaml      # counter_node 默认参数
  launch/source.launch.py # 一次启动两个节点
```
