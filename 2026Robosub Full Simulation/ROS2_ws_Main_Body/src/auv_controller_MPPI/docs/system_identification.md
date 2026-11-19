# 离线系统辨识指南

辨识对象是**前向动力学模型的未知物理参数 θ**(附加质量/线性阻尼/二次阻尼/推力缩放),
不是 MPPI 本身——MPPI 无需训练,换参数即可用。

## 1. 采集数据

```bash
ros2 launch auv_controller_MPPI training.launch.py mode:=random_goals   # 随机目标
ros2 launch auv_controller_MPPI training.launch.py mode:=excitation     # 或激励实验
```

collector(`training_data_collector_node`)自动记录对齐样本,CSV 列:

```
t, experiment_id, x(13), u(8), x_next(13), dt
```

- 状态源:`collector.state_topic`(默认 `/auv/ground_truth/odometry`,仿真真值优先;
  无真值时退回 `/auv/state/control` 也可,改参数即可);
- 对齐规则:状态到达沿触发,间隔 ≥ sample_dt(默认 0.05 s)写一行,
  u 取区间内保持的命令;dt 超过 10×sample_dt 或 <1e-4 的行丢弃;
- 同一 experiment_id 内的目标切换/激励段边界自动成为样本边界。

激励实验建议(提高可辨识性):每轴独立做 step / chirp / rand 三种波形
(`training.excitation_*` 参数),chirp 0.1→1 Hz 覆盖姿态带宽。

## 2. 运行辨识(纯 numpy,无需 scipy/torch)

```bash
cd src/auv_controller_MPPI
python3 scripts/system_identification.py \
  --data .install/.../data/training_xxx.csv \
  --identify lin quad thrust \
  --out config/dynamics.yaml
```

方法:

- 残差:单步预测误差 `[Δp(3), Δangle(1), Δv(3), Δw(3)]`,各分量按尺度归一化
  (p 0.05 m / angle 0.05 rad / v 0.10 m/s / w 0.20 rad/s);
- 优化:阻尼高斯-牛顿(LM),有限差分 Jacobian,自适应阻尼;
- 数据划分:按 experiment_id 的 md5 哈希 70/15/15 划 Train/Val/Test
  (同一实验不跨集,防泄漏);
- 评估:报告初值/训练后各集单步 RMS + Test 10 步多步预测误差;
- 导出:正则替换写回 `config/dynamics.yaml`(原值留存在 dynamics_default.yaml,
  文件头自动加辨识标记与时间)。

## 3. 注意

- python 复刻与 C++ DynamicsModel **逐步一致**(包括四元数积分用步进前 w、
  位置积分用步进前 v 的细节),改动任何一边必须同步另一边;
- `--identify added` 可辨识附加质量,但与阻尼强相关,建议分批;
- 辨识后必须重跑基础任务测试(`basic_mission.launch.py`)验证控制性能不退化。

## 4. 与训练模式的关系

- 训练(random_goals)用**同一个** MPPI 控制器闭环跑:数据是当前动力学下
  控制器行为分布的样本——参数误差会让数据分布偏移,但 LM 对初值偏鲁棒;
- excitation 模式绕过 MPPI 直接注人力/力矩(B 伪逆分配),激励频谱可控,
  是推荐的辨识数据来源。
