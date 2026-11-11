"""좌표를 직접 입력하지 않고 저장 지도에서 위치를 추정한 뒤 검사 후 이어 맵핑을 시작한다."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory('limo_auto_mapping'))/'config/relocalization.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('posegraph', description='Absolute saved graph prefix, no extension'),
        DeclareLaunchArgument('map', default_value='', description='Defaults to <posegraph>.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization_timeout_sec', default_value='120.0'),
        Node(package='limo_auto_mapping', executable='resume_mapping', output='screen',
             parameters=[str(config), {
                 'posegraph': ParameterValue(LaunchConfiguration('posegraph'), value_type=str),
                 'map': ParameterValue(LaunchConfiguration('map'), value_type=str),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'localization_timeout_sec': ParameterValue(
                     LaunchConfiguration('localization_timeout_sec'), value_type=float)}])])
