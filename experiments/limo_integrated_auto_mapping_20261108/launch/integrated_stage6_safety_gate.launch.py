#!/usr/bin/env python3
"""
integrated_stage6_safety_gate.launch.py
Launches Stage 6 Integration:
1. State Estimators (F4 LiDAR Scan Matcher or F5_improved EKF) + Odom TF Relay
2. SLAM Toolbox Online Asynchronous Mapping
3. DaBai Depth Obstacle Filter (/depth_obstacles/points)
4. Nav2 Stack + optional explore_lite
   - Nav2 controller_server outputs to /cmd_vel_raw (strictly isolated from actuators)
5. Stall Detector & Safety Command Gate:
   - Intercepts /cmd_vel_raw
   - Verifies against LiDAR chassis velocity (/odometry/lidar_odom) & wheel velocity (/odom)
   - Dispatches safe commands strictly to /cmd_vel
   - Auto-reverse & auto-rotate recovery disabled (SAFETY LOCKED)
   - Operator unlatch strictly required
   - Fail-safe sensor timeout (0.45s) & ICP degradation protection
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    config_dir = os.path.join(pkg_dir, 'config')
    nodes_dir = os.path.join(pkg_dir, 'nodes')
    bin_dir = os.path.join(pkg_dir, 'bin')
    rl_lib_dir = os.path.abspath(os.path.join(pkg_dir, "../../capstone_ws/install/robot_localization/lib"))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    estimator_type = LaunchConfiguration('estimator_type', default='f4')
    enable_depth_filter = LaunchConfiguration('enable_depth_filter', default='true')
    start_explore = LaunchConfiguration('start_explore', default='false')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_estimator_type = DeclareLaunchArgument(
        'estimator_type', default_value='f4',
        choices=['f4', 'f5_improved'],
        description='Selected state estimator: f4 or f5_improved'
    )
    declare_enable_depth_filter = DeclareLaunchArgument(
        'enable_depth_filter', default_value='true',
        description='Enable Orbbec DaBai depth obstacle filtering into Nav2 costmap'
    )
    declare_start_explore = DeclareLaunchArgument(
        'start_explore', default_value='false',
        description='Start explore_lite frontier explorer node'
    )

    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld = f"{rl_lib_dir}:{existing_ld}" if existing_ld else rl_lib_dir
    set_no_usersite = SetEnvironmentVariable('PYTHONNOUSERSITE', '1')
    set_ld_path = SetEnvironmentVariable('LD_LIBRARY_PATH', new_ld)

    # 1. State Estimators
    lidar_odom_node = ExecuteProcess(
        cmd=['python3', '-s', os.path.join(nodes_dir, 'lidar_scan_matcher_odom_node.py')],
        output='screen'
    )

    is_f5 = PythonExpression(["'", estimator_type, "' == 'f5_improved'"])
    ekf_f5_node = ExecuteProcess(
        condition=IfCondition(is_f5),
        cmd=[
            os.path.join(bin_dir, 'ekf_node'),
            '--ros-args',
            '-r', '__node:=ekf_f5_improved_node',
            '--params-file', os.path.join(config_dir, 'fusion_f5_improved.yaml'),
            '-r', 'odometry/filtered:=/odometry/f5_improved'
        ],
        output='screen'
    )

    # 2. Odom TF Relay
    is_f4 = PythonExpression(["'", estimator_type, "' == 'f4'"])
    odom_tf_relay_f4 = ExecuteProcess(
        condition=IfCondition(is_f4),
        cmd=[
            'python3', '-s', os.path.join(nodes_dir, 'odom_tf_relay_node.py'),
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', 'active_odom_topic:=/odometry/lidar_odom',
            '-p', 'nav2_odom_topic:=/odom_nav2',
            '-p', 'odom_frame:=odom',
            '-p', 'base_frame:=base_footprint'
        ],
        output='screen'
    )

    odom_tf_relay_f5 = ExecuteProcess(
        condition=IfCondition(is_f5),
        cmd=[
            'python3', '-s', os.path.join(nodes_dir, 'odom_tf_relay_node.py'),
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', 'active_odom_topic:=/odometry/f5_improved',
            '-p', 'nav2_odom_topic:=/odom_nav2',
            '-p', 'odom_frame:=odom',
            '-p', 'base_frame:=base_footprint'
        ],
        output='screen'
    )

    # 3. SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'integrated_slam_toolbox_params.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    # 4. DaBai Depth Obstacle Filter
    depth_filter_node = Node(
        condition=IfCondition(enable_depth_filter),
        package='limo_integrated',
        executable='depth_obstacle_filter_node.py',
        name='depth_obstacle_filter_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'integrated_depth_params.yaml'),
                    {'use_sim_time': use_sim_time}]
    )

    # 5. Nav2 Stack: controller_server outputs to /cmd_vel_raw (routed to stall detector gate)
    nav2_params = os.path.join(config_dir, 'integrated_nav2_params.yaml')

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/cmd_vel_raw'),
            ('/cmd_vel', '/cmd_vel_raw'),
            ('odom', '/odom_nav2')
        ]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/cmd_vel_raw'),
            ('/cmd_vel', '/cmd_vel_raw')
        ]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[('odom', '/odom_nav2')]
    )

    lifecycle_manager_nav2 = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'recoveries_server',
                'bt_navigator'
            ]
        }]
    )

    # 6. Frontier Explorer (explore_lite)
    explore_node = Node(
        condition=IfCondition(start_explore),
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'integrated_explore_params.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    # 7. Stall Detector & Final Safety Command Gate
    stall_detector_node = Node(
        package='limo_integrated',
        executable='stall_detector_node.py',
        name='stall_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enabled': True,
            'enable_auto_reverse': False,  # SAFETY LOCKED
            'enable_auto_rotate_recovery': False,  # SAFETY LOCKED
            'require_operator_unlatch': True,  # LATCH ENFORCED
            'sensor_timeout_sec': 0.38,
            'depth_mode': 'required',
            'stall_confirm_duration_sec': 0.22,
            'cmd_vel_threshold': 0.075,
            'wheel_vel_threshold': 0.060,
            'lidar_vel_stall_threshold': 0.025,
            'publish_virtual_bumper': True,
            'bumper_distance_m': 0.24,
        }],
        remappings=[
            ('/cmd_vel_raw', '/cmd_vel_raw'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_estimator_type,
        declare_enable_depth_filter,
        declare_start_explore,
        set_no_usersite,
        set_ld_path,
        lidar_odom_node,
        ekf_f5_node,
        odom_tf_relay_f4,
        odom_tf_relay_f5,
        slam_toolbox_node,
        depth_filter_node,
        controller_server,
        planner_server,
        recoveries_server,
        bt_navigator,
        lifecycle_manager_nav2,
        explore_node,
        stall_detector_node,
    ])
