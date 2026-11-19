# source.launch.py —— 一次启动 example_source 包的两个 C++ 节点。
#
# 用法(独立启动,不依赖工作区根目录的 all.launch.py):
#   ros2 launch example_source source.launch.py
#   ros2 launch example_source source.launch.py namespace:=demo2
#   ros2 launch example_source source.launch.py \
#     config_file:=/绝对路径/自定义.yaml
#
# 配置文件默认从“安装后的 share 目录”读取(get_package_share_directory),
# 因此在任意工作目录下执行都生效,不依赖当前目录。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('example_source')
    default_config = os.path.join(pkg_share, 'config', 'source.yaml')

    namespace = LaunchConfiguration('namespace')
    config_file = LaunchConfiguration('config_file')

    counter_node = Node(
        package='example_source',
        executable='counter_node',     # CMakeLists 里 add_executable 的名字
        name='counter_node',           # 节点名(ros2 node list 可见)
        namespace=namespace,           # 相对话题 count -> /<ns>/count
        parameters=[config_file],      # YAML 参数文件
        output='screen',
    )

    feedback_node = Node(
        package='example_source',
        executable='feedback_node',
        name='feedback_node',
        namespace=namespace,
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='demo',
            description='两个节点的命名空间(默认 demo)'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='counter_node 的 YAML 参数文件路径'),
        counter_node,
        feedback_node,
    ])
