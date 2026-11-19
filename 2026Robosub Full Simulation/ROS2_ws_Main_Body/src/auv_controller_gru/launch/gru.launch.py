# -*- coding: utf-8 -*-
"""GRU 控制器独立启动(联调用;统一入口见 auv_bringup/bringup.launch.py)。

  ros2 launch auv_controller_gru gru.launch.py
  ros2 launch auv_controller_gru gru.launch.py \
      model_npz:=/abs/gru_policy.npz model_meta:=/abs/gru_policy_meta.json

model_npz/model_meta 为空时不覆盖 yaml 默认值(避免空串使节点加载失败)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _make_node(context):
    pkg = get_package_share_directory('auv_controller_gru')
    default_params = os.path.join(pkg, 'config', 'gru.yaml')
    params_file = context.launch_configurations.get('params') or \
        default_params
    overrides = {'use_sim_time': True}
    for key in ('model_npz', 'model_meta'):
        v = (context.launch_configurations.get(key) or '').strip()
        if v:
            overrides[key] = v
    return [Node(
        package='auv_controller_gru',
        executable='gru_controller_node',
        name='gru_controller_node',
        output='screen',
        parameters=[params_file, overrides],
    )]


def generate_launch_description():
    pkg = get_package_share_directory('auv_controller_gru')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params', default_value=os.path.join(pkg, 'config', 'gru.yaml'),
            description='GRU 节点参数 yaml'),
        DeclareLaunchArgument(
            'model_npz', default_value='',
            description='覆盖 numpy 权重路径(训练产物)'),
        DeclareLaunchArgument(
            'model_meta', default_value='',
            description='覆盖模型元数据路径'),
        OpaqueFunction(function=_make_node),
    ])
