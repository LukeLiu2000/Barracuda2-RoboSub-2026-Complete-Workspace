# all.launch.py —— 工作区总启动入口:一次启动全部四个业务节点。
#
# 它是"工作区级"文件:不属于 example_source / example_processing 任何一个包,
# 也不是第三个包。因此用文件路径启动,而不是 ros2 launch <包名> <文件>:
#
#   cd <工作区根目录>            # 路径含空格时注意引号
#   source install/setup.bash
#   ros2 launch launch/all.launch.py
#
# 原理:两个子包的 launch 文件已随构建安装进各自的 share 目录,
# 本文件通过 ament 索引(get_package_share_directory)定位它们并 Include,
# 所以与当前工作目录无关。需要一个不存在的"第三方包名"来启动它反而
# 是错误做法——我们没有、也不需要第三个包。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    source_share = get_package_share_directory('example_source')
    processing_share = get_package_share_directory('example_processing')

    namespace = LaunchConfiguration('namespace')
    source_config = LaunchConfiguration('source_config_file')
    processing_config = LaunchConfiguration('processing_config_file')

    include_source = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(source_share, 'launch', 'source.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'config_file': source_config,
        }.items(),
    )

    include_processing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(processing_share, 'launch', 'processing.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'config_file': processing_config,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='demo',
            description='四个节点共用的命名空间(默认 demo)'),
        DeclareLaunchArgument(
            'source_config_file',
            default_value=os.path.join(source_share, 'config', 'source.yaml'),
            description='counter_node 参数文件'),
        DeclareLaunchArgument(
            'processing_config_file',
            default_value=os.path.join(processing_share,
                                       'config', 'processing.yaml'),
            description='report_node 参数文件'),
        include_source,
        include_processing,
    ])
