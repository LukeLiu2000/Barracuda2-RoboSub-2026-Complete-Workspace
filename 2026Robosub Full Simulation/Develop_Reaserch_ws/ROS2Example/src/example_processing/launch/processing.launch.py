# processing.launch.py —— 一次启动 example_processing 包的两个 Python 节点。
#
# 用法(独立启动):
#   ros2 launch example_processing processing.launch.py
#   ros2 launch example_processing processing.launch.py namespace:=demo2
#
# 单独启动时没有上游 counter,square_node 不会收到任何消息:
# 节点安静等待是正常状态,不是崩溃。可用下面命令手动验证:
#   ros2 topic pub /demo/count std_msgs/msg/Int64 "{data: 7}" -r 1

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('example_processing')
    default_config = os.path.join(pkg_share, 'config', 'processing.yaml')

    namespace = LaunchConfiguration('namespace')
    config_file = LaunchConfiguration('config_file')

    square_node = Node(
        package='example_processing',
        executable='square_node',    # setup.py console_scripts 里注册的名字
        name='square_node',
        namespace=namespace,
        parameters=[config_file],
        output='screen',
    )

    report_node = Node(
        package='example_processing',
        executable='report_node',
        name='report_node',
        namespace=namespace,
        parameters=[config_file],    # batch_size 从这里读取
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='demo',
            description='两个节点的命名空间(默认 demo)'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='report_node 的 YAML 参数文件路径'),
        square_node,
        report_node,
    ])
