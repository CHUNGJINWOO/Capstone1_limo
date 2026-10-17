import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_limo_capstone = get_package_share_directory('limo_capstone')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Fortress 가상 환경(5m x 5m 방) 전용 지도 및 파라미터 기본값 지정
    default_map_path = '/home/cjinw/simulator-research/limo_sim_ws/experiments/fortress/maps/limo_fortress_map.yaml'
    default_params_file = os.path.join(pkg_limo_capstone, 'config', 'nav2_sim_params.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load (default: Fortress room map)'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    # Note: Fortress 시뮬레이션 환경은 자체 모델(URDF/SDF)에서 odom -> base_footprint -> base_link
    # 변환을 제공하므로, 실물용 base_link -> base_footprint static_tf_pub 노드는 제외합니다.

    # Nav2 표준 bringup 런치 포함 (AMCL, Costmap, Planner, Controller 등)
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
        map_arg,
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        nav2_bringup_launch
    ])
