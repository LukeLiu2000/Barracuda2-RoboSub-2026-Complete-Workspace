# MPPI 算法与实现说明

参考文献:Williams et al., "Aggressive driving with model predictive path integral
control", ICRA 2016;(信息论 MPC 框架)。

## 1. 问题设定

- 状态(13 维):`[p(3), q(4, xyzw), v(3), w(3)]`,p 世界系,q 世界←体系,
  v/w 体坐标系(与里程计 twist 约定一致);
- 控制(8 维):各推进器推力 T_i ∈ [−39.2266, +39.2266] N;
- 动力学:`x_{k+1} = f(x_k, u_k)`(docs 外见 dynamics_model,Fossen 6-DOF + 显式欧拉,
  可配子步 `mppi.substeps`);
- 目标:min E[Σ running cost + terminal cost] s.t. 推力盒约束。

## 2. 求解流程(每次 solve() 调用)

1. **预采样** K×H×8 高斯噪声 ε ~ N(0, σ²I)(单 RNG 一次性生成)——
   rollout 内核因此成为无状态纯计算,便于 OpenMP 并行与未来 GPU 迁移;
2. **K 条 rollout**:u = clamp(U_nominal + ε),逐步行进动力学并累计代价;
3. **重要性权重**(真 softmax,非 best-of-K):
   `w_k = exp(−(S_k − S_min)/λ) / Σ exp(...)`;
4. **加权更新(带动量 α)**:
   `U ← (1−α)·U + α·Σ_k w_k·clamp(U + ε_k)`
   (用**截断后**的采样控制参与平均,名义序列始终可行);
5. **左移热启动**:序列左移一步,末尾补 `shift_fill`;
6. 输出:最优序列 U(H×8)、名义轨迹 states(H+1×13)、代价、solve_ms、权重熵。

参数(K/H/dt/σ/λ/α/seed/substeps)全部 YAML 化(`config/mppi.yaml`),
相同 seed 结果逐位可复现(单测 DeterministicWithSameSeed 验证,OpenMP 并行不破坏确定性:
静态调度 + 各 rollout 写不相交内存 + 归约在串行段完成)。

## 3. 代价函数(config/mppi.yaml 全参数化)

| 项 | 公式 | 默认权重 |
|---|---|---|
| 位置 | w_p·‖p−p*‖² | 60 |
| 姿态 | w_q·θ(q,q*)²(θ=测地角) | 25 |
| 线速度 | w_v·‖v−v*‖² | 8 |
| 角速度 | w_w·‖w−w*‖² | 3 |
| 推力做功 | w_e·‖u‖²/u_max² | 0.02 |
| 平滑度 | w_s·‖u_k−u_{k−1}‖²/u_max² | 0.3 |
| 饱和惩罚 | w_sat·Σ hinge(‖u_raw‖−u_max)²/u_max² | 50 |
| 工作空间 | w_safe·viol(p)²/u_max²(越界距离 hinge) | 200 |
| 终端加权 | 上述(速度/姿态/位置/工作空间)× terminal_multiplier | ×8 |

双覆盖处理:姿态误差计算先做 q↔−q 就近选择(types.hpp orientationError)。

## 4. 节点架构与线程模型

```
/auv/state/control (nav_msgs/Odometry) ──┐
/auv/control/goal (PoseStamped) ─────────┤
                                          ▼
                              mppi_controller_node
   ├─ solver 线程:条件变量按 rates.solver_hz(默认 10 Hz)唤醒,
   │   状态/目标新鲜度门控(safety.*),solve() → SequenceExecutor.setSequence
   │   按需发布 /auv/control/predicted_path、/auv/control/optimal_sequence(debug)
   └─ exec 定时器:rates.exec_hz(默认 100 Hz)按时间戳采样序列,
       输出 /auv/control/thruster_cmd(Float64MultiArray 8 维)
       序列超时(safety.stall_s/序列尾部)→ STALE → 零推力(看门狗)
急停:~/estop(SetBool)→ ZEROED 零推力;1 Hz /auv/control/diagnostics
```

- 训练激励旁路:`/auv/training/excitation_cmd` 新鲜时优先执行(EXCITATION 状态),
  单写者原则——训练实验直接独占推力总线;
- 状态里程计若给 base_link 原点,节点按 `thrusters.com_offset_*` 换算到质心
  (`p += R·r_com, v += w×r_com`);
- goal 的 frame_id 与状态 frame 不一致直接拒收并 ERROR(本版不做 TF)。

## 5. 性能工程

实测(服务器 24 核 x86,K=1000/H=15/dt=0.1):

| 阶段 | 单次 solve 耗时 |
|---|---|
| 初版(6×6 LLT + 矩阵组装 Coriolis) | 445.6 ms(2.2 Hz,不达标) |
| 优化后(对角 M 逐元素除 + Coriolis 叉积直算 + 四元数共轭代替旋转矩阵 + 常量缓存) | **36.8 ms 均值 / 44.2 峰值(22.6 Hz)** |

再叠加 OpenMP 并行 rollout(噪声预采样保证无竞争)。核心库零 ROS 依赖,
rollout 内核纯函数,CUDA 迁移路径明确(K×H 批量并行是天然 grid)。

## 6. 已知边界

- 显式欧拉在 dt=0.1 s 下稳定(阻尼比大),更硬的动力学(辨识后)可能需要 substeps≥2;
- 温度 λ 需与代价量级匹配:λ 过小(如 5)时 softmax 退化为单样本精英,
  更新方差大;默认 λ=8 对应代价 ~1e2–1e3 量级,可辨识后重调;
- 执行频率与求解频率解耦(序列时间戳插值),solver 掉频时执行端自动进宽限→零推力。
