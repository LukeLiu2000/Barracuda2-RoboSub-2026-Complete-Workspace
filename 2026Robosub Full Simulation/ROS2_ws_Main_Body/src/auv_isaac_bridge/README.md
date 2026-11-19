# auv_isaac_bridge —— Isaac Sim 力桥与真值源

在 Isaac Sim **进程内**运行的桥(不是独立 ROS 节点):把 `/auv/control/thruster_cmd`
的 8 路推力施加到物理刚体上,并把仿真真值发布成 `/auv/state/control` 里程计。
本包解决的是"控制输出 -> Isaac 施力 -> 真值反馈"的物理闭环。

## 为什么放在 kit 进程内(而不是 Action Graph / 独立节点)

- 当前 Isaac 实例由用户手动打开、场景手动加载,没有远程执行入口;
  Script Editor 一次性 bootstrap 是改动最小的接通方式(用户唯一手动操作)。
- 每物理步施力/读真值需要 `omni.physx` 物理步回调与 USD 变换,OGN 搭图
  无法表达"命令校验 + 超时清零 + 急停 + 时间回跳处理"这类逻辑。
- 桥自带 TCP(127.0.0.1:17788)命令服务:诊断/逐推进器试验/复位全部可以
  从 SSH 终端远程驱动;改脚本后 `restart` 热重启,不需要用户再粘贴。

## 目录

```
auv_isaac_bridge/
├── isaac/
│   ├── bootstrap.py          # 用户在 Script Editor 执行的唯一入口(一次)
│   └── auv_isaac_bridge.py   # 桥本体(kit 内运行:rclpy + 施力 + 水动力 + TCP)
├── config/isaac_bridge.json  # 几何/水动力/单位/超时参数(mtime 热重载)
├── scripts/
│   ├── bridge_client.py      # TCP 客户端(纯标准库;驱动所有实验)
│   └── bridge_monitor.py     # ROS 侧监视节点(1Hz 单行摘要)
└── launch/status_monitor.launch.py
```

## 启动(唯一手动步骤)

1. 用户打开 Isaac Sim 并加载场景(暂停态即可)。
2. `Window -> Script Editor`,粘贴 `isaac/bootstrap.py` 全文,Ctrl+Enter。
   成功标志:输出 `[auv_isaac_bridge] 已启动 TCP 127.0.0.1:17788`。

## 常用命令(服务器 SSH 终端)

```bash
B=src/auv_isaac_bridge/scripts/bridge_client.py
python3 $B ping                  # 桥活着吗
python3 $B status                # 全量状态(单位/几何/模式/错误)
python3 $B stage_info            # up 轴 / mpu / 重力 / 刚体路径(v2: 含物理场景清单)
python3 $B robot_info            # 推进器几何(实测推导 vs 配置)
python3 $B sensor_scan           # v2: ZED 相机/IMU prim 只读扫描
python3 $B graph_scan            # v2: ActionGraph 只读检查(防重复发布)
python3 $B xform_dump --paths "p1;p2"  # v2: 逐 prim 世界/局部矩阵诊断
python3 $B timeline --action play --pef false    # 1x 实时步进
python3 $B test_thrust --index 0 --force 2 --dur 1   # 单推进器小推力
python3 $B evaluate --index 0 --force 2 --dur 1      # 力单位/方向对账
python3 $B estop_on              # 急停(立即清零全部输出)
python3 $B estop_off             # 解除
python3 $B zero                  # 清命令与测试推力
python3 $B hydro_set '{"buoyancy": {"ratio": 1.02}}' # 运行时调水动力
python3 $B set_pose --pos 0,0,0 --quat 0,0,0,1       # 试验复位
python3 $B restart               # 改了桥脚本后热重启
python3 $B watch                 # 1Hz 状态摘要
```

ROS 侧监视(另一个终端,系统 ROS 2 环境):

```bash
python3 src/auv_isaac_bridge/scripts/bridge_monitor.py
```

## 接口约定(与控制器/任务节点共享,详见联调报告)

- `/auv/state/control`、`/auv/ground_truth/odometry`(nav_msgs/Odometry):
  同内容。`frame_id=auv_world`(场景世界系),`child_frame_id=auv_com`。
  **位置=质心世界位置(真实米),姿态=体姿态,速度=体坐标系质心线速度/
  角速度**,时间戳=仿真时间。因此控制器 `apply_com_offset` 必须为 false,
  所有下游使用同一质心原点,无二次换算。
- `/auv/control/thruster_cmd`(std_msgs/Float64MultiArray,8 路,SI 牛顿,
  下标=thruster_0..7):桥做尺寸/NaN/限幅(±39.2266N)/超时(0.5s 仿真时间)
  /急停校验,异常一律清零,绝不重放过期命令。
- `/auv/control/estop`(std_msgs/Bool):true=急停(键盘节点空格键发布)。
- `/clock`:仿真时间,60Hz 级。
- `/auv/bridge/status`(std_msgs/String,1Hz,内容为 JSON,字段与 TCP `status`
  相同):mode/estop/cmd_seq/cmd_age/cmd_rate/time_jumps/up_axis/mpu/force_scale/
  几何校验/错误。原 DiagnosticArray 因 Isaac Sim 5.1.0 isaacsim.ros2.bridge
  4.12.4 发布该类型会在 C 层断言失败并 abort 整个 kit 进程而弃用(2026-11-03
  实测定位,bridge_monitor.py 已同步改为解析 JSON)。

## 单位换算(桥内部自动处理)

- stage 数值 -> 真实米:`meters = numeric * metersPerUnit`(mpu 从当前
  stage 读取);局部体坐标(米)-> 世界数值:`numeric = meters / mpu`
  (祖先缩放已在推导中消去)。
- 力:SI 牛顿 -> PhysX 数值 = `SI * force_scale`,默认 `1/mpu`
  (auto)。**用 `evaluate` 实测校验**,系统性偏差时在 JSON
  `units.force_unit_scale` 写显式数值并 `config_reload`。

## 水动力(第一阶段 Zero-Gravity Damped,2026-11-03 起)

**第一阶段设计目标**(与用户约定):场景重力显式为零、桥不施加浮力,
只保留体轴线性/二次阻尼与 8 路推力——静止悬浮、推力动、停推减速。

- 场景侧:pool.usd 需含**合法** PhysicsScene prim(gravityDirection 为有效
  非零向量(如 (0,0,-1)) + gravityMagnitude **显式 0**)。注意
  `direction=(0,0,0)`/`magnitude=-Inf` 这类 kit 自动 scene 的垃圾值是
  **非法**的(被 PhysX 忽略),不算显式零重力——桥的
  `_validate_scene_gravity` 会区分二者:合法零重力 → `gravity_valid=true,
  zero_gravity=true`;非法值 → 告警 + 按未确认处理。
- 桥侧:`hydrodynamics.buoyancy.mode="off"`(浮力禁用,阻尼保留)。
  重力未确认时浮力分支本身也被闸死(2026-11-03 漂移事故的教训:
  绝不按假设重力自动施浮力)。
- 后续阶段恢复重力+浮力:场景 author 正常重力,config 把 mode 改回
  `neutral_compensated` 即可(沿 -g 方向,与 up 轴无关)。

参数在 `config/isaac_bridge.json`,区分 `measured`(USD 实测)与
`assumed`(暂定假设);JSON 修改即热重载。

## v2 变更(2026-11-03 下午,Barracuda_2 真实场景联调)

**刚体识别改为三步制,杜绝误绑 Gate**:

1. `robot_prim_path` 精确路径优先(当前
   `/World/Barracuda_2/barracuda2/base_link`);
2. 精确路径失效时,只在 `robot_root_paths` 指定的根(当前
   `/World/Barracuda_2` 与 `/World/barracuda2`)内部扫描带
   `UsdPhysics.RigidBodyAPI` 的 prim,取层级最浅者;
3. 两步都失败则**报错拒绑**(绝不全舞台按名字猜;曾误绑
   `/World/gate_01/gate/base_link`,已修)。

推进器同理:先精确路径 `.../thruster_0..7`,失效才在机器人根内按
basename 找。`robot_info` 的 `body_from`/`geom_source` 字段可见来源。

**旋转提取修复(几何对账 0.0000 的根因)**:

- `Gf.Matrix4d.ExtractRotationQuat` 在祖先带缩放(本场景 wrapper 均匀
  缩放 0.001)时返回损坏四元数——统一改用 SVD 极分解取正交旋转(含
  镜像修正);
- Gf 是行向量约定(`v' = v @ M`,`TransformDir(e_i)` 返回 M 的**第 i
  行**),收集基向量后必须转置才是列约定矩阵。两处都修后,
  stage 实测推进器几何与配置逐项一致(|Δr|max=0.00mm,|Δdir|max=0.0000)。

**stage 实测事实(live 场景 `pool.usd`)**:Z-up、mpu=1.0、**无
PhysicsScene prim**——Play 前 `stage_info` 的重力是"按 up 轴推定的
-9.81·Z"(`gravity_provisional=true`,`gravity_source` 注明);Play 后物理
场景自动创建,桥每 30 步复查一次并在状态里更新真实值。**桥不改场景
重力,只读。**

**只读检查结论**:ZED 相机/IMU prim 齐全(`sensor_scan`);ActionGraph
只有 `/World/ActionGraph`(on_playback_tick + zed_camera_helper,**无任何
ROS2 节点**)——桥是唯一 ROS2 出口,无重复发布;`auv_sensing` 里的
`add_ground_truth_graph.py` 会往场景加真值发布图,**不要**与桥同时用。

**已知跨包注意点(MPPI 只读,未改动)**:
`controller.launch.py` 不覆盖 world_up,用的 `dynamics.yaml`
`[0,0,1]`(Z-up)与场景一致 ✓;但 `basic_mission.launch.py:35` 硬编码
`[0,1,0]`(旧 Y-up 资产残留),只影响离线虚拟任务跑批,建议改回
`[0,0,1]`(待用户确认后由用户/授权修改)。

**传感器话题缺口**:桥只发布 odom/clock/status;`auv_sensing` 期望的
ZED 图像/深度/点云/IMU 话题(`zed/left/image_rect` 等)当前**无发布者**
(需 ActionGraph ROS2 相机桥或后续扩展,未在本次范围)。

## 已知限制

- 速度真值来自相邻物理步差分(1 步滞后,60Hz 下 ~16ms),对 10Hz 求解器
  足够;如需更准可换 RigidPrim 物理视图(未启用,见脚本内注释)。
- `set_pose` 直接写 USD 变换:若 PhysX 缓存把艇弹回旧位姿,用
  `timeline pause -> set_pose -> play` 流程。
- 质心用的是 USD 碰撞网格体积质心(measured);PhysX 运行时质心若有偏差,
  `evaluate` 会以系统性力矩漂移的形式暴露,届时在 JSON 里修正。
- 桥与 Isaac 实例同生共死:Isaac 关闭即桥关闭;重启 Isaac 后需重新粘贴
  bootstrap.py。
- 推力方向沿用"推进器局部 +X 为出力方向"的假设(thruster prim 局部轴与
  配置一致已验证,但推进器网格朝向与出力方向的对应未实测)——以第七阶段
  `evaluate` 逐推进器小推力试验为准。
