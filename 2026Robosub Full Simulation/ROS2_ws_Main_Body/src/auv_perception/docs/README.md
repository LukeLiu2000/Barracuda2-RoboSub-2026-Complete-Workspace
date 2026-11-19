# AUV semantic perception / 世界坐标目标记录

本包通过 ROS 话题消费 ZED SDK 原生 XYZ，不计算替代深度，不向推进器或目标话题发布。
单个共享 YOLO segmentation 模型；模板和算法模块分离。当前定位模板仅 Task 1。

## 模板与扩展

`config/templates/task1_gate.yaml` 定义各类别的有限几何表面、白色骨架线和图片标注。
来源是场景中组合后的 gate.usd + gate_visual.usd 材质子集边界；不是只按 USD 文件的 mpu 猜测。
视觉 USD 元数据为厘米/Y-up，而最终组合体为米/Z-up，必须以组合结果测量。
左右柱按红黑排列识别：左柱上黑下红，右柱上红下黑，不按图像左右。
当前场景左右袖套高约 1.2193m、中心红板 0.6096m、牌面约 0.3048m。
这些不等于 Handbook 门总外尺寸；模板中的截面是测量后的简化近似。
`Gate` 整体 mask 扣除部件已覆盖的像素，仅剩余点参与低权重整体表面约束，不重复计数。

新增任务：添加同 schema YAML，在 `perception.yaml/templates` 列表启用。
box/cylinder 刚性组合可复用 rigid_primitives；新的几何算法在 `core.SOLVERS` 注册。
模型类别可由模板 aliases 映射，不依赖类别 ID 顺序。陌生类别忽略并保留扩展空间。
目前同一模板维护一个地标；多同类物体关联尚未实现，部件仅取最高置信实例。

## 运行

推荐使用 `bash scripts/start_ros.sh`，它显式加载既有工作区及本包 overlay。
构建仅选择本包，已有控制包源码未改。ROS 侧 Python 禁用用户 site，
YOLO 在已经安装好的独立 CUDA 环境常驻子进程中运行，不修改全局 numpy/torch。

如果用户现有仿真、ZED 采集及 Foxglove 已运行，避免重复启动，
先核对输入话题、采集配置与相机外参，然后只启动本包节点。

## 独立仿真联调配置

`scripts/prepare_scene.py` 生成 `/tmp/auv_perception_vision_scene.usda` 临时副本。
原始场景不保存、不覆盖。临时副本替换不可解析的 CDN ZED 引用为本机 ZED 资产，
移除旧 Render 执行缓存，建立明确的左/右光学安装位置（0.12m 基线）。
`scripts/vision_scene.py` 开启真实 ZED SDK 推流，专用端口 30002，无控制器。
该入口的相机 profile 与 `perception.yaml/camera_to_body` 配套；
**不能把这个外参当作任意原始场景中已验证的外参**。更换场景应重新核对。
不会读取场景中 Gate 的世界位姿作为定位输入。

## 输出及 Foxglove

- `/auv/perception/scene`：MarkerArray，完整世界系场景快照。
- `/auv/perception/object_map`：schema_version=1 的 JSON String，权威地图状态，含最后接受时间与质量。
- `/auv/perception/observations`：每次拟合候选及是否提交；不是无条件可靠定位。
- `/auv/perception/status`：处理时延、提交情况和错误。
- `/auv/perception/overlay`：分割调试图。

本次笔记本已建立仅回环的 SSH 转发，本地使用 `ws://127.0.0.1:18765`。
服务器桥仍为 `127.0.0.1:8765`。协议为 foxglove.sdk.v1（现有 bridge 版本）。添加 3D 面板，
Fixed frame 选择 `auv_world`，开启 `/auv/perception/scene`。
黄色=current state，绿色=PID effective_goal（失效即移除），
白色线段=门框，白色叉及文字=SOS/Repair。
不自动启动 PID，不伪造 current/target 数据。没有有效目标时绿色箭头不显示。
单根箭头表示 body +Z 前向，短线表示 body +Y 上向。

可靠观测连续三帧后建图；质量不足/目标消失时冻结最后接受的世界位姿。
世界系位置不随当前机器人状态重算。map/markers 使用 transient local 完整快照，
晚连接可见；地图仅进程内保留，重启不恢复。仿真时间回跳清空地图。
手动清空：`ros2 service call /auv/perception/clear_map std_srvs/srv/Trigger '{}'`。

## 验证与边界

`PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q`
测试覆盖噪声+离群点几何拟合、局部可见、欠约束初始化拒绝、投影遮挡、
地图冻结/跳变拒绝、历史状态插值。合成测试不等于真实图像端到端验证。
质量采用内点率、RMS、数据雅可比条件比；不宣称严格统计置信区间。
未知遮挡、多目标、背面标志缺失与系统性 SDK 深度偏差仍需实景评估。
本次临时零重力配置 SDK 的 IMU 返回全零，既有采集逻辑按原规则拒发；
没有修改原 IMU 代码、原传感器配置或原 USD。该临时 profile 不能替代已验证的原场景传感器配置。
