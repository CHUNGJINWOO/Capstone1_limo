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

    # 설정 파일 경로 선언 (기존 설정 파일 재사용)
    default_slam_params = os.path.join(pkg_limo_capstone, 'config', 'slam_params.yaml')
    default_nav2_params = os.path.join(pkg_limo_capstone, 'config', 'nav2_sim_params.yaml')
    default_explore_params = os.path.join(pkg_limo_capstone, 'config', 'explore.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='Full path to the slam_toolbox parameters file to use'
    )

    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=default_nav2_params,
        description='Full path to the Nav2 parameters file to use'
    )

    explore_params_arg = DeclareLaunchArgument(
        'explore_params_file',
        default_value=default_explore_params,
        description='Full path to the explore_lite parameters file to use'
    )

    # 1. SLAM Toolbox (온라인 비동기 매핑 모드)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
        }.items()
    )

    # 2. Nav2 Navigation (AMCL/정적 지도 없이 컨트롤러/플래너 스택만 실행)
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'autostart': 'true'
        }.items()
    )

    # 3. explore_lite 프론티어 탐색 노드
    # use_sim_time 파라미터를 명시적으로 전달하여 시뮬레이션 클럭과 동기화
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        parameters=[
            LaunchConfiguration('explore_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Note: Fortress 시뮬레이션 환경은 자체 모델(URDF/SDF)에서 odom -> base_footprint -> base_link
    # 변환을 제공하므로, 실물용 base_link -> base_footprint static_tf_pub 노드는 제외하여
    # TF 순환 루프(Cycle) 발생을 방지합니다.

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_arg,
        nav2_params_arg,
        explore_params_arg,
        slam_launch,
        nav2_navigation_launch,
        explore_node
    ])
