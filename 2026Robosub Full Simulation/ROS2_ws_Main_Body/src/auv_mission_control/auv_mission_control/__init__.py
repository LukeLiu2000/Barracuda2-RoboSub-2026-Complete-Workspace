# -*- coding: utf-8 -*-
"""auv_mission_control:AUV 逻辑控制层。

goal_manager.goal_manager.GoalManager 是可复用的目标位姿管理器
(键盘遥控、保持、后续自主任务共用同一接口):
  - set_state() 喂当前状态(nav_msgs/Odometry 提取)
  - update(dt, vel_body, yaw_cmd, ...) 按机体自身姿态推进目标
  - 输出限幅(相对当前位姿最大距离/姿态差 + 工作空间盒)
"""
