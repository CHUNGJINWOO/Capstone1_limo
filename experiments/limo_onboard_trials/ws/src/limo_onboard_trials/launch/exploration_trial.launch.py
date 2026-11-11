"""맵핑 스택과 TF를 확인한 후 별도로 실행하는 centroid/middle 비교 진입점."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    target = LaunchConfiguration('target').perform(context)
    if target not in ('centroid', 'middle'):
        raise ValueError('target must be centroid or middle')
    # centroid는 USB에 설치된 기존 탐색기, middle은 별도 이름으로 복사한 기존 수정본이다.
    package = 'explore_lite' if target == 'centroid' else 'limo_mapping_explore'
    share = Path(get_package_share_directory('limo_onboard_trials'))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower()
    if use_sim_time not in ('true', 'false'):
        raise ValueError('use_sim_time must be true or false')
    return [Node(package=package, executable='explore', name='explore_node', output='screen',
                 parameters=[str(share/'config/explore.yaml'),
                             {'use_sim_time': use_sim_time == 'true'}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target', default_value='middle', description='centroid or middle'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=start)])
