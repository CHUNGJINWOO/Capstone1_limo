import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_base = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107'

    default_map = os.path.join(pkg_base, 'maps', 'lounge_map.yaml')
    default_params = os.path.join(pkg_base, 'config', 'nav2_lounge_baseline.yaml')

    declare_map = DeclareLaunchArgument('map', default_value=default_map, description='Lounge map YAML')
    declare_params = DeclareLaunchArgument('params_file', default_value=default_params, description='Nav2 params')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time')
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true', description='Autostart Nav2')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
        }.items()
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_use_sim_time,
        declare_autostart,
        nav2_bringup_launch
    ])
