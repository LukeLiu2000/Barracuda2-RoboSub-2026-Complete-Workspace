# auv_mission_control — AUV 逻辑控制层(键盘遥控 / 基础保持)

统一的目标管理接口 `GoalManager`(`auv_mission_control/goal_manager.py`,
纯逻辑、无 rclpy 依赖,可单测):键盘遥控、基础保持、后续自主任务
复用同一接口,保证目标语义一致。

## 节点

| 节点 | 可执行 | 作用 |
|---|---|---|
| 终端键盘 | `keyboard_node` | SSH 终端遥控;方向键/字母键八向+偏航,Space 急停,X 清急停,R 重捕获,Q 退出 |
| 浏览器键盘 | `web_keyboard` | 内嵌 HTTP 面板(127.0.0.1:8791),真 keydown/keyup+失焦+心跳;SSH 端口转发后用 |
| 基础保持 | `hold_node` | 捕获当前位姿并 2Hz 重发同一目标(位姿保持闭环最小验证) |

## 键位(终端与网页一致)

```
      ↑ (升)            W 前  S 后
  ← (左转) → (右转)     A 左移 D 右移
      ↓ (降)
```

- 方向键 ←/→ 默认绕**自身上轴(+y)** 偏航;若实测反了改
  `config/keyboard.yaml` 的 `keyboard.yaw_rate_sign`。
- 键可组合;按住以额定速度持续更新目标;松键目标保持不动
  (不重置为 current,控制器把机器人拉回目标)。
- 目标相对当前位置限幅(`max_goal_dist`/`max_goal_att_deg`)+
  工作空间盒(与 mppi.yaml 同步)。
- 状态过期(`state_timeout_s`)不更新目标;急停后需 X 清除并 R 重捕获。
- 终端键盘的按压保活 0.4s(SSH 终端无可靠 keyup,依赖自动重复);
  组合键不可靠时用浏览器面板。

## 启动

```bash
# 统一入口(控制器常驻;hold 模式再加 hold_node;monitor 可选)
ros2 launch auv_mission_control mission.launch.py mode:=hold
ros2 launch auv_mission_control mission.launch.py mode:=keyboard

# 键盘节点另开终端(独占 stdin,不进 launch)
ros2 run auv_mission_control keyboard_node
# 浏览器面板:本机先转发端口
#   ssh -L 8791:127.0.0.1:8791 liuhao@172.30.30.1
ros2 run auv_mission_control web_keyboard   # 然后开 http://127.0.0.1:8791

# 自动测试/辨识模式不要从本入口起:
ros2 launch auv_controller_MPPI training.launch.py mode:=random_goals
```

## 互斥

- 同一时间只允许一种**目标源**(hold / 键盘 / 自动测试);
  自动测试运行时键盘节点会冻结目标(experiment_id 新鲜度检测),
  反之手动模式起后请勿再起训练 launch。

## 急停与恢复

- Space(或网页 STOP):发布 `/auv/control/estop` 并调用
  `/mppi_controller_node/estop` 服务;桥端清零推力。
- 恢复:X 清急停,再 R 重捕获(目标跳到当前位姿,防跳变)。
