import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_limo_capstone = get_package_share_directory('limo_capstone')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 수정된 지도 경로 기본값 지정
    default_map_path = os.path.join(pkg_limo_capstone, 'maps', 'MC_3F_10_15.yaml')
    default_params_file = os.path.join(pkg_limo_capstone, 'config', 'nav2_explore_params.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # base_link -> base_footprint 정적 TF 브로드캐스터 (AMCL 필수 프레임 제공)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    # Nav2 표준 bringup 런치 포함 (AMCL, Costmap, Planner, Controller 등)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        map_arg,
        params_file_arg,
        use_sim_time_arg,
        static_tf,
        nav2_bringup_launch
    ])
