from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share=Path(get_package_share_directory('auv_perception'))
    config=LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(share/'config/perception.yaml')),
        DeclareLaunchArgument('sensing_config',default_value=str(share/'config/sensing_vision.yaml')),
        DeclareLaunchArgument('with_sensing',default_value='false',description='Enable only when no ZED acquisition node exists'),
        DeclareLaunchArgument('with_foxglove',default_value='false',description='Enable only when port 8765 is unused'),
        Node(package='auv_perception',executable='synced_zed_node',name='zed_sensor_node',namespace='auv',
            parameters=[LaunchConfiguration('sensing_config')],additional_env={'LC_ALL':'C'},output='screen',
            condition=IfCondition(LaunchConfiguration('with_sensing'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('auv_bringup'))/'launch/foxglove.launch.py')),
            condition=IfCondition(LaunchConfiguration('with_foxglove'))),
        Node(package='auv_perception',executable='perception_node',parameters=[{'config':config,'use_sim_time':True}],additional_env={'PYTHONNOUSERSITE':'1'},output='screen'),
        Node(package='auv_perception',executable='scene_node',parameters=[{'config':config,'use_sim_time':True}],additional_env={'PYTHONNOUSERSITE':'1'},output='screen')])
