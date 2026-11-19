# -*- coding: utf-8 -*-
"""统一联调入口(auv_mission_control)。三种模式互斥(同一时间只启动一种
目标源,避免 /auv/control/goal 多写者):

  mode:=hold          基础保持:控制器 + hold_node(捕获当前位姿并保持)
  mode:=keyboard      键盘遥控:启动控制器;键盘节点在独立终端运行
                      (终端键盘要独占 stdin,不能放进 launch)
  mode:=keyboard_web  同 keyboard,但提示用浏览器面板
  mode:=auto_test     自动测试/辨识:等价于
                      ros2 launch auv_controller_MPPI training.launch.py
                      (本入口拒绝启动,提示直接用上面那条,避免两套定义)

键盘节点启动命令(另开终端):
  ros2 run auv_mission_control keyboard_node
  # 浏览器面板(笔记本先做端口转发 ssh -L 8791:127.0.0.1:8791):
  ros2 run auv_mission_control web_keyboard
"""
import os

# 注意:本机 Jazzy 的 ament 模块是扁平名 ament_index_python(旧名
# ament_index 在这台机器上 import 不了,launch 会直接报错)。
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    with_monitor = LaunchConfiguration('with_monitor').perform(context).lower() \
        in ('1', 'true', 'yes')
    mppi = get_package_share_directory('auv_controller_MPPI')
    mc = get_package_share_directory('auv_mission_control')
    cfg = lambda share, name: os.path.join(share, 'config', name)
    common = [cfg(mppi, 'thrusters.yaml'), cfg(mppi, 'dynamics.yaml'),
              cfg(mppi, 'mppi.yaml'), cfg(mppi, 'training.yaml'),
              cfg(mc, 'keyboard.yaml')]

    nodes = []
    if mode == 'auto_test':
        print('[mission.launch] auto_test 模式请直接运行:\n'
              '  ros2 launch auv_controller_MPPI training.launch.py '
              'mode:=random_goals   # 或 mode:=excitation\n'
              '(本入口不重复启动该组合,避免定义漂移)')
        return nodes

    nodes.append(Node(
        package='auv_controller_MPPI',
        executable='mppi_controller_node',
        name='mppi_controller_node',
        output='screen',
        parameters=common + [{'topics.state': '/auv/state/control'},
                             {'use_sim_time': True}],
    ))
    if mode == 'hold':
        nodes.append(Node(
            package='auv_mission_control',
            executable='hold_node',
            name='auv_hold_node',
            output='screen',
            parameters=common,
        ))
    elif mode in ('keyboard', 'keyboard_web'):
        tip = ('ros2 run auv_mission_control keyboard_node' if mode == 'keyboard'
               else 'ssh -L 8791:127.0.0.1:8791 liuhao@172.30.30.1 后:\n'
                    '  ros2 run auv_mission_control web_keyboard\n'
                    '  浏览器打开 http://127.0.0.1:8791')
        print('[mission.launch] %s 模式:控制器已启动。键盘节点另开终端运行:\n  %s'
              % (mode, tip))
    else:
        raise RuntimeError('未知 mode: %s(应为 hold|keyboard|keyboard_web|'
                           'auto_test)' % mode)

    if with_monitor:
        nodes.append(Node(
            package='auv_isaac_bridge',
            executable='bridge_monitor.py',
            name='auv_bridge_monitor',
            output='screen',
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='hold',
            description='hold | keyboard | keyboard_web | auto_test'),
        DeclareLaunchArgument('with_monitor', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
