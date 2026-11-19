# -*- coding: utf-8 -*-
# ============================================================================
# 一键创建"机器人真值"ROS 2 发布图 —— 在 Isaac Sim 界面使用:
#   1. 先跑 diagnose_scene.py,把下面 ROBOT_PRIM 改成实际的机器人主体路径
#      (必须是带 RigidBody/ArticulationRoot 的那个 prim,不是摄像头);
#   2. Window -> Script Editor -> 粘贴本文件 -> Run;
#   3. 菜单 File -> Save 保存进场景,之后重启场景依然有效;
#   4. 验证(服务器终端):
#        source /opt/ros/jazzy/setup.bash
#        ros2 topic list        # 应出现 /auv/ground_truth/odometry 和 /clock
#        ros2 topic hz /auv/ground_truth/odometry
#
# 本脚本可重复运行:已存在同名图时先删除再重建(不会碰场景里的其他图)。
#
# 发布内容(全部来自 Isaac 物理引擎,是仿真真值,不是视觉估计):
#   /auv/ground_truth/odometry  nav_msgs/Odometry
#       frame: odom(参考系) -> base_link(机器人主体)
#       位置/姿态为世界系真值;速度由 IsaacComputeOdometry 按位姿差分计算
#       (publishRawVelocities=false 时,ROS2PublishOdometry 会把世界系
#        速度投影到机器人本体坐标系再发布,符合 Odometry 消息约定)。
#   /clock  rosgraph_msgs/Clock   仿真时钟(启用 ROS 端 use_sim_time 用)
# ============================================================================

GRAPH_PATH = '/World/auv_ground_truth'
ROBOT_PRIM = '/World/PASTE_ROBOT_PRIM_HERE'    # <-- 必改!见文件头说明

TOPIC_ODOM = '/auv/ground_truth/odometry'
TOPIC_CLOCK = '/clock'
ODOM_FRAME = 'odom'
CHASSIS_FRAME = 'base_link'

import omni.usd
import omni.graph.core as og

stage = omni.usd.get_context().get_stage()
assert stage, '没有打开的场景'

if ROBOT_PRIM == '/World/PASTE_ROBOT_PRIM_HERE':
    print('!! 请先把 ROBOT_PRIM 改成机器人主体路径(跑 diagnose_scene.py 找)')
elif not stage.GetPrimAtPath(ROBOT_PRIM):
    print(f'!! prim 不存在: {ROBOT_PRIM} —— 检查路径大小写和层级')
else:
    # 幂等:删旧图再建,避免重复发布者
    if stage.GetPrimAtPath(GRAPH_PATH):
        from omni.kit.commands import execute
        execute('DeletePrims', paths=[GRAPH_PATH])
        print('已删除旧的真值图(重建)')

    (keys, results) = og.Controller.edit(
        {'graph_path': GRAPH_PATH, 'evaluator_name': 'execution'},
        {
            og.Controller.Keys.CREATE_NODES: [
                ('on_physics', 'isaacsim.core.nodes.OnPhysicsStep'),
                ('odom', 'isaacsim.core.nodes.IsaacComputeOdometry'),
                ('pub_odom', 'isaacsim.ros2.bridge.ROS2PublishOdometry'),
                ('on_tick', 'omni.graph.action.OnPlaybackTick'),
                ('pub_clock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
            ],
            og.Controller.Keys.CONNECT: [
                ('on_physics.outputs:tick', 'odom.inputs:execIn'),
                ('odom.outputs:execOut', 'pub_odom.inputs:execIn'),
                ('odom.outputs:position', 'pub_odom.inputs:position'),
                ('odom.outputs:orientation', 'pub_odom.inputs:orientation'),
                ('odom.outputs:linearVelocity', 'pub_odom.inputs:linearVelocity'),
                ('odom.outputs:angularVelocity', 'pub_odom.inputs:angularVelocity'),
                ('on_tick.outputs:tick', 'pub_clock.inputs:execIn'),
            ],
            og.Controller.Keys.SET_VALUES: [
                ('odom.inputs:robotPrim', ROBOT_PRIM),
                ('odom.inputs:resetOdometry', False),
                ('pub_odom.inputs:topicName', TOPIC_ODOM),
                ('pub_odom.inputs:odomFrameId', ODOM_FRAME),
                ('pub_odom.inputs:chassisFrameId', CHASSIS_FRAME),
                ('pub_odom.inputs:publishRawVelocities', False),
                ('pub_clock.inputs:topicName', TOPIC_CLOCK),
            ],
        },
    )
    err = og.Controller.get_graph_evaluation_error(results)
    if err:
        print('!! 建图出错:', err)
    else:
        print(f'真值图创建成功:{GRAPH_PATH}')
        print(f'  ROBOT_PRIM = {ROBOT_PRIM}')
        print(f'  发布 {TOPIC_ODOM}(nav_msgs/Odometry,{ODOM_FRAME}->{CHASSIS_FRAME})')
        print(f'  发布 {TOPIC_CLOCK}(rosgraph_msgs/Clock)')
        print('记得 File -> Save 保存场景,并按 Play 验证 ros2 topic list。')
