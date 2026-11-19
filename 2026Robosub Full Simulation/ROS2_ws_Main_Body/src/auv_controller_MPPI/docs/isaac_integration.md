# 对接 Isaac Sim 推力执行端(下一步工作)

本包输出止于 `/auv/control/thruster_cmd`(Float64MultiArray,8 维,牛顿,
顺序 = `config/thrusters.yaml` 的 thruster_0..7,与 USD prim 顺序一致——映射本身
待驱动确认,见 thruster_geometry.md §7)。把命令送进 Isaac 需要**修改授权目录外**
的场景/桥接代码,按任务约束本包不得代做,以下是对接清单。

## 1. 两条路线

**A. Isaac ROS 桥(推荐)**:场景侧已有 auv_sensing 的 ROS 桥(ZED IPC 流已通),
同一 IsaacRosConvert/omni.isaac.ros2_bridge 体系里给 8 个推进器 prim 加
`ros2_publisher`(订阅 Float64MultiArray 或每推进器 Float64),把推力写进
ArticulationDrive / apply_force("rigid_body_applied_forces" 不适合分布式推进器,
首选每个 thruster prim 施加本地力)。

**B. Python 扩展(快速验证)**:Isaac Script Editor 里写 30 行:
订阅 `/auv/control/thruster_cmd`,按 B 矩阵列方向对每个推进器
`rigid_prim.apply_local_force(direction_i * T_i, pos_is_local=True)`——
方向向量直接抄 `config/thrusters.yaml` 的 directions(与 USD 假设互为校验)。

## 2. 对接前必须确认的映射(对应 thruster_geometry.md 待确认清单)

1. **正推力方向**:逐个发 `thruster_cmd = [.., +20 N, ..]`,观察仿真转向是否
   与 yaml directions 一致;不一致改 directions(配置项,不用改代码);
2. **编号映射**:确认话题下标 i ↔ USD prim `thruster_i` 顺序;
3. **推力量纲**:确认 Isaac 端接受的是 N 还是 kgf(4000 gf 名义值是 kgf 口径);
4. **世界系竖直轴**:Isaac 直通里程计是 y-up,launch 里必须带
   `dynamics.world_up: [0,1,0]`、`gravity_world: [0,-9.81,0]`
   (basic_mission.launch.py 已示范)。

## 3. 推荐联调步骤

1. 先开环:手动 `ros2 topic pub /auv/control/thruster_cmd` 单推进器小推力,
   验证方向/量纲/编号(上一节 1–3);
2. 再闭环零点保持:`controller.launch.py` + 目标 = 当前位姿,看是否悬停稳;
3. 复现基础任务:goal = 前进 1 m + 偏航 30°,对比仿真实测误差
   (基础任务自闭环版本的指标做参照);
4. 采集训练数据:`training.launch.py mode:=excitation` → 跑系统辨识 →
   回写 dynamics.yaml → 重跑 3 验证改善;
5. 若推进器方向与假设不符导致 B 变化:改 `thrusters.yaml` 的 directions 即可,
   节点启动时重新校验秩/条件数(秩 < 6 直接 FATAL 拒跑,不会带病工作)。

## 4. 频率预算(供 Jetson 部署参考)

- exec 100 Hz × 8 float ≈ 8 kB/s,任何桥都无压力;
- solver 服务器实测 36.8 ms(K=1000);Orin Nano 按 1/5–1/8 保守估计 150–300 ms
  → 不够 10 Hz 时优先降 K(500 → 约 18 ms/90ms)或 H(15→12),
  参数全在 `config/mppi.yaml`,无需改码;必要时按 mppi_algorithm.md §5 的
  CUDA 路径迁移(内核已无 ROS 依赖)。
