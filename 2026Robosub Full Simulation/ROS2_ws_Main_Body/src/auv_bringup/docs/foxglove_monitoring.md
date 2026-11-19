# Foxglove 实时监测:current / goal / effective_goal 位置曲线

2026-11-10 增。目标:在本地笔记本 Foxglove 里同一张图对比机器人当前位置与
目标位置(世界系 X/Y/Z),并区分"外部命令的目标"与"PID 此刻实际追踪的目标"。

## 1. 三条曲线用的话题

| 曲线 | 话题路径(Foxglove Plot 直接填) | 含义 |
|---|---|---|
| 当前位置 X | `/auv/state/control.pose.pose.position.x` | Isaac 桥发布的机器人**质心世界系**位置(`nav_msgs/Odometry`,frame=`auv_world`,child=`auv_com`) |
| 外部命令目标 X | `/auv/control/goal.pose.position.x` | 外部(hold_node/键盘/实验编排器)发给控制器的世界系目标(`geometry_msgs/PoseStamped`) |
| PID 实际追踪目标 X | `/auv/pid/effective_goal.pose.position.x` | **PID 计算时真正采用**的目标(含内部 hold 捕获值),`geometry_msgs/PoseStamped`,默认 10 Hz |

- 把 `.x` 换成 `.y` / `.z` 即另外两个轴;三条曲线可放同一 Plot 面板对比。
- 配套来源话题 `/auv/pid/goal_source`(`std_msgs/String`,10 Hz):
  - `external` —— PID 正在跟踪 `/auv/control/goal` 的外部目标
  - `hold` —— 无外部目标,hold 捕获的内部目标。**注意 hold 目标每拍重捕当前
    位姿,所以 hold 模式下 effective_goal≈当前位置(输出表现为速度阻尼),
    这是既有控制语义,监测只是如实呈现**
  - `none` —— 尚无任何目标被采用(刚启动 / ~/reset 之后)
- `/auv/pid/effective_goal` 只在控制回路真正在用目标时发布(状态新鲜、未急停、
  未被他控制器压制);**曲线断流 = 此刻没有有效目标**,不会伪造零坐标。
- MPPI/GRU 不发布这些话题(`/auv/pid/*` 为 PID 专属,无多写者冲突)。

### 坐标系注意

- 上面三个位置都是**世界系**(`auv_world`,Z-up 场景)数值。
- 机体坐标系约定是 +x 左舷、+y 上、+z 前——**不要**据此推断世界轴:艇转向后
  "左移"未必是世界 +X。解释位置曲线请始终用世界系;需要看姿态另画
  `/auv/state/control.pose.pose.orientation`(四元数)。

## 2. 启动方式(服务器)

依赖(已装):`ros-jazzy-foxglove-bridge` 3.6.0。未装时:
`sudo apt install ros-jazzy-foxglove-bridge`

方式 A——一条命令(推荐,PID + hold 目标源 + 桥监视 + Foxglove 桥):

```bash
source /opt/ros/jazzy/setup.bash
source <ws>/install/setup.bash      # ws 见下
ros2 launch auv_bringup bringup.launch.py controller:=pid with_foxglove:=true
```

方式 B——只开 Foxglove 桥(不启动相机、不启动第二个控制器;控制器另起):

```bash
ros2 launch auv_bringup foxglove.launch.py
# 可选: address:=127.0.0.1 port:=8765
# 可选白名单(降带宽,排除相机/点云):
#   topic_whitelist:=/auv/state/.*,/auv/control/.*,/auv/pid/.*,/clock
```

- 桥默认只监听 `127.0.0.1:8765`(SSH 隧道用,不暴露公网)。
- 不要与 `auv_sensing/sensing_with_foxglove.launch.py` 同时开:两者都占 8765,
  后启动者直接报 `Failed to bind port: Address already in use` 退出(不会双桥)。
- Isaac 侧另开终端(无头):`/isaacsim/python.sh <ws>/src/auv_isaac_bridge/isaac/run_headless.py --config ros`

## 3. QoS(BEST_EFFORT)——为什么这里容易踩空

`/auv/state/control` 是 SensorDataQoS(**BEST_EFFORT**)。foxglove_bridge 3.6.0
默认 `best_effort_qos_topic_whitelist=["(?!)"]`(永不匹配),对 BEST_EFFORT
发布者用 RELIABLE 订阅——**QoS 不兼容,一条都收不到**,但 `ros2 topic list`
照样显示话题名(2026-11-10 实测复现)。

处理:`foxglove.launch.py` 已固定传 `best_effort_qos_topic_whitelist: [".*"]`
(20 Hz BEST_EFFORT 实测全量通过)。这**只改桥自己的订阅方式**,不动原发布链路
的可靠性语义。

服务器端验证(不装本地 Foxglove 也能自检):

```bash
# 1) 话题存在且发布者 QoS 是 BEST_EFFORT(只证明图上有名字)
ros2 topic info /auv/state/control -v
# 2) 数据真的在发(BEST_EFFORT 话题,ros2 topic hz 默认 RELIABLE 收不到!)
python3 - <<'EOF'
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
rclpy.init(); n = Node("hz_be"); c = [0]
n.create_subscription(Odometry, "/auv/state/control",
                      lambda m: c.__setitem__(0, c[0]+1),
                      qos_profile_sensor_data)
t = time.monotonic() + 5
while time.monotonic() < t: rclpy.spin_once(n, timeout_sec=0.2)
print("got", c[0], "msgs in 5s"); n.destroy_node()
EOF
# 3) bridge 真的把数据送到 WebSocket 客户端(>0 才会有曲线)
python3 $(ros2 pkg prefix auv_bringup)/share/auv_bringup/scripts/fg_probe.py \
    8765 /auv/state/control,/auv/pid/effective_goal,/auv/control/goal 5
```

## 4. 笔记本连接(SSH 端口转发)

```bash
ssh -L 8765:127.0.0.1:8765 liuhao@172.30.30.1
```

Foxglove(桌面版)→ Open connection → **Foxglove WebSocket** →
`ws://localhost:8765`。

## 5. Plot 面板设置

1. 左上 Add panel → Plot。
2. 点击数据源框,逐条输入(回车确认,可粘贴同一路径再改末尾轴名):
   - `/auv/state/control.pose.pose.position.x`
   - `/auv/control/goal.pose.position.x`
   - `/auv/pid/effective_goal.pose.position.x`
3. `.x` → `.y` / `.z` 换轴;可再开两个 Plot 分别管 Y/Z。
4. 时间轴:X 轴取 "min"/最近的 30–60 s;底部播放条暂停时图面停在当前值。
   Plot 设置里 `Timestamp method` 建议保持默认(receive time)——桥侧所有消息
   都按到达时刻画,三条曲线必然同一时间轴,不受 header stamp 域影响。
5. `/auv/control/goal` 只在外部发目标时有点(effective_goal 则 10 Hz 连续),
   对比时先在服务器发一个目标试试:
   `ros2 topic pub -1 /auv/control/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: auv_world}, pose: {position: {x: -12.0, y: -1.0, z: 1.0}}}"`

## 6. 检查清单(曲线不出时的排查顺序)

1. 仿真在跑吗?暂停时 `/auv/state/control`、`/clock` 都没有新消息
   (看 `/auv/bridge/status` 里的 steps 是否增长)。
2. **/clock 在发吗?** bringup/launch 给控制器传了 `use_sim_time: true`;
   若 Isaac 桥没在跑(无 /clock),ROS 2 的 sim-time 定时器一个都不会触发,
   PID 节点"启动了但什么都不发"(控制/心跳/监测全静默)——这是 sim-time
   语义,不是节点挂了。`ros2 topic hz /clock` 确认(2026-11-10 实测踩过)。
3. `fg_probe.py` 第 3 步 >0?——0 说明 QoS/订阅问题(见上)。
4. effective_goal 没有?——正常现象若 goal_source=none/reset 后/状态过期
   (0.6 s 墙钟)/急停/被 MPPI/GRU 心跳压制。
5. 连接被拒?——先建 SSH 隧道,桥只听 127.0.0.1。
6. 8765 被占?——已有一个桥在跑(sensing_with_foxglove 或上一次没退干净:
   `ss -tlnp | grep 8765`)。

注:监测频率 `monitor_hz` 与控制频率一样按**仿真时间**计。无头仿真若跑得
比实时快(实测本机 ~1.5-2x),墙钟频率会等比升高——live 实测 effective_goal
墙钟 ~21 Hz = 10 Hz(sim) × ~2x,属正常。

## 7. 本次改动清单(2026-11-10)

- `auv_controller_pid`:节点新增 `/auv/pid/effective_goal` + `/auv/pid/goal_source`
  监测发布(参数 `topics.effective_goal`、`topics.goal_source`、
  `rates.monitor_hz`,默认 10 Hz,0=关);`pid.yaml` 同步;`~/reset` 同时清监测。
- `auv_bringup`:新增 `launch/foxglove.launch.py`(只启动桥);
  `bringup.launch.py` 新增 `with_foxglove`/`foxglove_address`/`foxglove_port`/
  `foxglove_topic_whitelist`;hold_node 补 `use_sim_time: true`(目标时间戳
  与状态同处仿真时间轴);setup.py 安装 docs/scripts。
- 未改:Isaac 桥、PID 控制公式、MPPI/GRU、sensing、thruster_cmd 链路。
