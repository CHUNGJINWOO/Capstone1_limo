"""SLAM + Nav2 + idle action server. Robot driver/URDF/LiDAR run separately."""
from pathlib import Path
import math

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = Path(get_package_share_directory('limo_auto_mapping'))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    config = yaml.safe_load((share / 'config/slam_params.yaml').read_text())['slam_toolbox']['ros__parameters']
    config['use_sim_time'] = use_sim_time
    config['mode'] = 'mapping'
    graph = LaunchConfiguration('posegraph').perform(context)
    pose = LaunchConfiguration('start_pose').perform(context)
    if graph:
        path = Path(graph).expanduser()
        if not path.is_absolute() or not all(Path(str(path) + ext).is_file()
                                             for ext in ('.posegraph', '.data')):
            raise RuntimeError('posegraph must be an absolute prefix of existing .posegraph/.data files')
        values = yaml.safe_load(pose)
        if (not isinstance(values, list) or len(values) != 3
                or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in values)):
            raise RuntimeError('Resuming requires start_pose:="[x, y, yaw]" in map coordinates')
        config['map_file_name'] = str(path)
        config['map_start_pose'] = [float(x) for x in values]
        config['map_start_at_dock'] = False
    elif pose:
        raise RuntimeError('start_pose requires posegraph')
    nav = Path(get_package_share_directory('nav2_bringup')) / 'launch/navigation_launch.py'
    return [
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', parameters=[config], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav)), launch_arguments={
            'params_file': str(share / 'config/nav2_explore_params.yaml'),
            'use_sim_time': str(use_sim_time).lower(), 'autostart': 'true',
            'use_composition': 'false'}.items()),
        Node(package='limo_auto_mapping', executable='mapping_server',
             parameters=[{'use_sim_time': use_sim_time}], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('posegraph', default_value='', description='Serialized graph prefix; no extension'),
        DeclareLaunchArgument('start_pose', default_value='', description='Required map-frame [x,y,yaw] when resuming'),
        OpaqueFunction(function=start)])
