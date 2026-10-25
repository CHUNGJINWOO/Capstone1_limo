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
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # 설정 파일 경로 선언
    slam_params_file = os.path.join(pkg_limo_capstone, 'config', 'slam_params.yaml')
    nav2_params_file = os.path.join(pkg_limo_capstone, 'config', 'nav2_explore_params.yaml')
    explore_params_file = os.path.join(pkg_limo_capstone, 'config', 'explore.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. SLAM Toolbox (온라인 비동기 매핑 모드 + 커스텀 파라미터 적용)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )

    # 2. Nav2 Navigation (AMCL/정적 지도 없이 컨트롤러/플래너 스택만 실행)
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )

    # 3. explore_lite 프론티어 탐색 노드 (수동 매핑 후 별도 실행을 위해 주석 유지)
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        parameters=[explore_params_file],
        output='screen'
    )

    # 4. base_footprint 정적 TF 브로드캐스터
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    return LaunchDescription([
        slam_launch,
        nav2_navigation_launch,
        static_tf,
        explore_node
    ])
