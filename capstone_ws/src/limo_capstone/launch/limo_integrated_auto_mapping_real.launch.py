#!/usr/bin/env python3
"""
limo_integrated_auto_mapping_real.launch.py
Master Real-Robot Launch File for LIMO Pro Integrated Auto-Mapping:
Excludes all simulation components (Gazebo, GT, normalizer) and interfaces with real hardware:
1. Real LIMO base serial driver (limo_base_node on /dev/ttylimo, pub_odom_tf=false)
2. Real YDLidar driver (ydlidar_ros2_driver on /dev/ydlidar)
3. Real Orbbec DaBai driver (astra_camera dabai.launch.py)
4. URDF & robot_state_publisher
5. DaBai 3D Depth Obstacle Filter (limo_depth_obstacles)
6. State Estimator: F4 (LiDAR ICP) or F5_improved (ICP + IMU EKF)
7. Single-Authority Odom TF Relay (odom -> base_footprint, publishing /odom_nav2)
8. SLAM Toolbox Online Asynchronous Mapping (map -> odom)
9. Nav2 Navigation Stack (controller_server gated strictly to /cmd_vel_raw)
10. explore_lite Autonomous Frontier Exploration Node
11. Stall Safety Gate (/cmd_vel_raw -> /cmd_vel, depth_mode=required)
12. FAIL-SAFE MOTOR LOCK:
    - allow_motor_drive defaults to 'false'.
    - If false, /cmd_vel output is redirected to /cmd_vel_locked to prevent unexpected
      physical movement until bench testing of hardware watchdog & E-stop is completed!
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    pkg_limo_capstone = get_package_share_directory('limo_capstone')
    pkg_limo_depth = get_package_share_directory('limo_depth_obstacles')
    pkg_limo_base = get_package_share_directory('limo_base')
    pkg_limo_desc = get_package_share_directory('limo_description')
    pkg_ydlidar = get_package_share_directory('ydlidar_ros2_driver')
    pkg_camera = get_package_share_directory('astra_camera')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')

    config_slam = os.path.join(pkg_limo_capstone, 'config', 'integrated_slam_params.yaml')
    config_nav2 = os.path.join(pkg_limo_capstone, 'config', 'integrated_nav2_params.yaml')
    config_explore = os.path.join(pkg_limo_capstone, 'config', 'integrated_explore_params.yaml')
    config_ekf = os.path.join(pkg_limo_capstone, 'config', 'fusion_f5_improved.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    estimator_type = LaunchConfiguration('estimator_type', default='f4')
    start_explore = LaunchConfiguration('start_explore', default='true')
    depth_mode = LaunchConfiguration('depth_mode', default='required')
    sensor_timeout_sec = LaunchConfiguration('sensor_timeout_sec', default='0.38')
    allow_motor_drive = LaunchConfiguration('allow_motor_drive', default='false')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_estimator_type = DeclareLaunchArgument(
        'estimator_type', default_value='f4',
        choices=['f4', 'f5_improved'],
        description='Selected state estimator: f4 (LiDAR ICP) or f5_improved (ICP+IMU EKF)'
    )
    declare_start_explore = DeclareLaunchArgument('start_explore', default_value='true')
    declare_depth_mode = DeclareLaunchArgument('depth_mode', default_value='required',
                                               description='Depth camera fail-safe mode: required, optional, disabled')
    declare_sensor_timeout = DeclareLaunchArgument('sensor_timeout_sec', default_value='0.38')
    declare_allow_motor = DeclareLaunchArgument(
        'allow_motor_drive', default_value='false',
        description='SAFETY LOCK: Set true only after MCU watchdog & physical E-Stop bench check!'
    )

    # 1. Real Robot Hardware Drivers
    # A. LIMO Base Serial Driver (pub_odom_tf=false to avoid TF clash)
    limo_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_limo_base, 'launch', 'limo_base.launch.py')),
        launch_arguments={'pub_odom_tf': 'false'}.items()
    )

    # B. YDLidar Driver
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ydlidar, 'launch', 'ydlidar.launch.py'))
    )

    # C. Orbbec DaBai Depth Camera Driver
    dabai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_camera, 'launch', 'dabai.launch.py'))
    )

    # D. Robot State Publisher (URDF)
    load_urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_limo_desc, 'launch', 'load_urdf.launch.py'))
    )

    # 2. Depth Obstacle Filter
    depth_filter_node = Node(
        package='limo_depth_obstacles',
        executable='depth_obstacle_filter_node',
        name='depth_obstacle_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'height_frame': 'base_footprint',
            'input_topic': '/camera/depth/points'
        }]
    )

    # 3. State Estimators: F4 vs F5_improved
    f4_node = Node(
        package='limo_capstone',
        executable='lidar_scan_matcher_odom_node',
        name='lidar_scan_matcher_odom_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    is_f5 = PythonExpression(["'", estimator_type, "' == 'f5_improved'"])
    is_f4 = PythonExpression(["'", estimator_type, "' == 'f4'"])

    f5_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_f5_improved_node',
        output='screen',
        parameters=[config_ekf, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/f5_improved')],
        condition=IfCondition(is_f5)
    )

    # 4. Single-Authority Odom TF Relay
    relay_f4 = Node(
        package='limo_capstone',
        executable='odom_tf_relay_node',
        name='odom_tf_relay_node',
        output='screen',
        parameters=[{
            'active_odom_topic': '/odometry/lidar_odom',
            'nav2_odom_topic': '/odom_nav2',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(is_f4)
    )

    relay_f5 = Node(
        package='limo_capstone',
        executable='odom_tf_relay_node',
        name='odom_tf_relay_node',
        output='screen',
        parameters=[{
            'active_odom_topic': '/odometry/f5_improved',
            'nav2_odom_topic': '/odom_nav2',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(is_f5)
    )

    # 5. SLAM Toolbox (Real Clock)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': config_slam
        }.items()
    )

    # 6. Nav2 Navigation Stack (gated strictly to /cmd_vel_raw)
    nav2_launch = GroupAction(
        actions=[
            SetRemap(src='cmd_vel', dst='/cmd_vel_raw'),
            SetRemap(src='/cmd_vel', dst='/cmd_vel_raw'),
            SetRemap(src='odom', dst='/odom_nav2'),
            SetRemap(src='/odom', dst='/odom_nav2'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': config_nav2,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # 7. explore_lite Frontier Exploration
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[config_explore, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_explore)
    )

    # 8. Stall Safety Gate with Fail-Closed Motor Lock
    # If allow_motor_drive is True, commands are routed to /cmd_vel
    stall_gate_live = Node(
        package='limo_capstone',
        executable='stall_detector_node',
        name='stall_detector_node',
        output='screen',
        parameters=[{
            'enabled': True,
            'enable_auto_reverse': False,
            'enable_auto_rotate_recovery': False,
            'require_operator_unlatch': True,
            'depth_mode': depth_mode,
            'sensor_timeout_sec': sensor_timeout_sec,
            'use_sim_time': use_sim_time
        }],
        remappings=[('/cmd_vel', '/cmd_vel')],
        condition=IfCondition(allow_motor_drive)
    )

    # If allow_motor_drive is False, output is redirected to /cmd_vel_locked
    stall_gate_locked = Node(
        package='limo_capstone',
        executable='stall_detector_node',
        name='stall_detector_node_locked',
        output='screen',
        parameters=[{
            'enabled': True,
            'enable_auto_reverse': False,
            'enable_auto_rotate_recovery': False,
            'require_operator_unlatch': True,
            'depth_mode': depth_mode,
            'sensor_timeout_sec': sensor_timeout_sec,
            'use_sim_time': use_sim_time
        }],
        remappings=[('/cmd_vel', '/cmd_vel_locked')],
        condition=UnlessCondition(allow_motor_drive)
    )

    log_lock_notice = LogInfo(
        condition=UnlessCondition(allow_motor_drive),
        msg="[SAFETY LOCK ACTIVE] Real robot motor drive is LOCKED (allow_motor_drive:=false). Commands redirected to /cmd_vel_locked."
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_estimator_type,
        declare_start_explore,
        declare_depth_mode,
        declare_sensor_timeout,
        declare_allow_motor,
        log_lock_notice,
        limo_base_launch,
        ydlidar_launch,
        dabai_launch,
        load_urdf_launch,
        depth_filter_node,
        f4_node,
        f5_ekf_node,
        relay_f4,
        relay_f5,
        slam_launch,
        nav2_launch,
        explore_node,
        stall_gate_live,
        stall_gate_locked
    ])
