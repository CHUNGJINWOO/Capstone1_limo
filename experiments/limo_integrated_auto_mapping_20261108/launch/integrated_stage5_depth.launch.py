#!/usr/bin/env python3
"""
integrated_stage5_depth.launch.py
Launches Stage 5 Integration:
1. Stage 4 Nav2 + SLAM Toolbox + State Estimator (F4/F5)
2. Orbbec DaBai Depth Obstacle Filter:
   - Takes /camera/depth/points from depth_to_pointcloud_fixed_node
   - Filters points in vertical window [0.025m, 0.35m] relative to base_footprint
   - Publishes /depth_obstacles/points directly into Nav2 Costmap ObstacleLayer
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
    cmd_vel_out = LaunchConfiguration('cmd_vel_out', default='/cmd_vel_raw')

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
    declare_cmd_vel_out = DeclareLaunchArgument(
        'cmd_vel_out', default_value='/cmd_vel_raw',
        description='Velocity command destination (default /cmd_vel_raw for safety gate interception)'
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

    # 2. Odom TF Relay: Single authority for odom -> base_footprint and /odom_nav2
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

    # 3. SLAM Toolbox Online Asynchronous Node
    slam_params_file = os.path.join(config_dir, 'integrated_slam_params.yaml')
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )

    # 4. Orbbec DaBai Depth Obstacle Filter Node
    depth_params_file = os.path.join(config_dir, 'integrated_depth_params.yaml')
    depth_filter_node = ExecuteProcess(
        condition=IfCondition(enable_depth_filter),
        cmd=[
            'python3', '-s', os.path.join(nodes_dir, 'depth_obstacle_filter_node.py'),
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '--params-file', depth_params_file
        ],
        output='screen'
    )

    # 5. Nav2 Stack
    nav2_params_file = os.path.join(config_dir, 'integrated_nav2_params.yaml')

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', cmd_vel_out)]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', cmd_vel_out)]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': [
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
            ]
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_estimator_type,
        declare_enable_depth_filter,
        declare_cmd_vel_out,
        set_no_usersite,
        set_ld_path,
        lidar_odom_node,
        ekf_f5_node,
        odom_tf_relay_f4,
        odom_tf_relay_f5,
        slam_node,
        depth_filter_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_node,
    ])
