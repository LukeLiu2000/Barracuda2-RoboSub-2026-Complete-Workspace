# -*- coding: utf-8 -*-
# ============================================================================
# 只读场景诊断脚本 —— 在 Isaac Sim 界面使用:
#   Window -> Script Editor -> 把本文件全部内容粘贴进编辑器 -> 点 Run
# 然后把 Output 里的全部输出发回(用于确定:场景文件、机器人主体 prim、
# ZED 相机 prim、IMU 是否存在、已有的 Action Graph 节点,
# 以便配置真值发布和修复 IMU)。
# 本脚本只做读取和打印,不修改任何内容。
# ============================================================================

import omni.usd
import omni.timeline as omni_timeline
from pxr import UsdGeom, UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()

print('=' * 60)
print('[1] 场景根层文件:', ctx.get_stage_url())
print('[1] upAxis =', UsdGeom.GetStageUpAxis(stage),
      '| metersPerUnit =', UsdGeom.GetStageMetersPerUnit(stage))

print('-' * 60)
print('[2] 相机 / IMU / 刚体 / 关节根 / 名字相关的 prim:')
for prim in stage.Traverse():
    tn = str(prim.GetTypeName())
    p = str(prim.GetPath())
    why = []
    if tn in ('Camera', 'IsaacImuSensor', 'PhysicsImuSensor'):
        why.append('type=' + tn)
    try:
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            why.append('ArticulationRoot')
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            why.append('RigidBody')
    except Exception:
        pass
    low = p.lower()
    if any(k in low for k in ('zed', 'imu', 'base_link', 'body', 'auv', 'robosub', 'robot')):
        why.append('name')
    if why:
        print(f'    {p}   <- {", ".join(why)}')

print('-' * 60)
print('[3] Action Graph 及其节点(含 ZED/ROS 节点的关键输入值):')
try:
    import omni.graph.core as og
    graphs = og.Controller.get_all_graphs()
    if not graphs:
        print('    (没有发现任何 OmniGraph)')
    for g in graphs:
        print(f'    graph: {g.get_path_to_graph()}')
        for node in g.get_nodes():
            print(f'      node: {node.get_type_name()}  @ {node.get_prim_path()}')
            # 打印 ZED / ROS 相关节点最关键的输入,用于确认相机 prim 与话题名
            for attr_name in ('inputs:cameraPrim', 'inputs:imuPrim',
                              'inputs:topicName', 'inputs:robotPrim',
                              'inputs:cameraModel', 'inputs:streamingPort'):
                try:
                    a = node.get_attribute(attr_name)
                    if a is not None and a.is_valid():
                        print(f'        {attr_name} = {a.get()}')
                except Exception:
                    pass
except Exception as e:
    print('    读取 OmniGraph 失败:', e)

print('-' * 60)
try:
    tl = omni_timeline.get_timeline_interface()
    print('[4] 仿真状态: is_playing =', tl.is_playing(),
          '| 仿真时间 =', tl.get_current_time(), 's')
except Exception as e:
    print('[4] 时间轴读取失败:', e)

print('=' * 60)
print('诊断完成 —— 请把以上全部输出发回。')
