#!/usr/bin/env python3
"""
limo_integrated_auto_mapping_master.launch.py
Unified Master Launch File for LIMO Pro Auto-Mapping System:
Executes the entire end-to-end autonomous mapping stack in a single launch:
1. Gazebo Fortress simulation (Lounge World) + LIMO Pro entity + ROS-GZ Bridge
2. Normalizer & Ground Truth Publisher nodes
3. State Estimator (selectable: 'f4' LiDAR Scan Matcher or 'f5_improved' EKF)
4. Single-Authority Odom TF Relay (odom -> base_footprint, publishing /odom_nav2)
5. SLAM Toolbox Online Asynchronous Mapping (map -> odom)
6. Orbbec DaBai Depth Obstacle Filter (/depth_obstacles/points into Nav2 Costmap)
7. Nav2 Navigation Stack (RegulatedPurePursuitController, A* Planner, BT Navigator)
   - controller_server remapped to publish strictly to /cmd_vel_raw
8. explore_lite Autonomous Frontier Exploration Node
9. Stall Detector & Final Safety Command Gate:
   - Intercepts /cmd_vel_raw
   - Clamps safe output to /cmd_vel
   - Auto-reverse & auto-rotate recovery locked (False)
   - Operator unlatch enforced
   - Sensor timeout fail-safe active
10. Dual-Panel MP4 Video Recorder Node with synchronized Telemetry HUD
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    config_dir = os.path.join(pkg_dir, 'config')
    nodes_dir = os.path.join(pkg_dir, 'nodes')
    launch_dir = os.path.join(pkg_dir, 'launch')
    bin_dir = os.path.join(pkg_dir, 'bin')
    rl_lib_dir = os.path.abspath(os.path.join(pkg_dir, "../../capstone_ws/install/robot_localization/lib"))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    estimator_type = LaunchConfiguration('estimator_type', default='f4')
    start_explore = LaunchConfiguration('start_explore', default='true')
    record_video = LaunchConfiguration('record_video', default='true')
    output_mp4 = LaunchConfiguration('output_mp4', default=os.path.join(pkg_dir, 'videos', 'limo_integrated_auto_mapping.mp4'))
    spawn_x = LaunchConfiguration('spawn_x', default='-1.10')
    spawn_y = LaunchConfiguration('spawn_y', default='-0.60')
    spawn_z = LaunchConfiguration('spawn_z', default='0.05')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='1.5708')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_estimator_type = DeclareLaunchArgument(
        'estimator_type', default_value='f4',
        choices=['f4', 'f5_improved'],
        description='Selected state estimator: f4 or f5_improved'
    )
    declare_start_explore = DeclareLaunchArgument('start_explore', default_value='true')
    declare_record_video = DeclareLaunchArgument('record_video', default_value='true')
    declare_output_mp4 = DeclareLaunchArgument('output_mp4', default_value=os.path.join(pkg_dir, 'videos', 'limo_integrated_auto_mapping.mp4'))
    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='-1.10')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='-0.60')
    declare_spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.05')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='1.5708')

    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld = f"{rl_lib_dir}:/opt/ros/humble/lib:{existing_ld}" if existing_ld else f"{rl_lib_dir}:/opt/ros/humble/lib"
    set_no_usersite = SetEnvironmentVariable('PYTHONNOUSERSITE', '1')
    set_ld_path = SetEnvironmentVariable('LD_LIBRARY_PATH', new_ld)

    # 1. Base Gazebo + LIMO Base + ROS Bridge
    base_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_gazebo_for_integrated.launch.py')),
        launch_arguments={
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'spawn_yaw': spawn_yaw,
            'use_sim_time': use_sim_time,
            'headless': 'true'
        }.items()
    )

    # 2. Sensor Normalizer and Ground Truth Publisher
    normalizer_node = ExecuteProcess(
        cmd=['python3', '-s', os.path.join(nodes_dir, 'trial_sensor_normalizer_node.py')],
        output='screen'
    )

    gt_publisher_node = ExecuteProcess(
        cmd=['python3', '-s', os.path.join(nodes_dir, 'ground_truth_publisher_node.py')],
        output='screen'
    )

    # 3. State Estimators
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
        additional_env={'LD_LIBRARY_PATH': new_ld},
        output='screen'
    )

    # 4. Odom TF Relay: Single authority for odom -> base_footprint and /odom_nav2
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

    # 5. SLAM Toolbox Online Asynchronous Mapping
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'integrated_slam_params.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    # 6. Orbbec DaBai Depth Obstacle Filter
    depth_filter_node = ExecuteProcess(
        cmd=[
            'python3', '-s', os.path.join(nodes_dir, 'depth_obstacle_filter_node.py'),
            '--ros-args', '-p', 'use_sim_time:=true'
        ],
        output='screen'
    )

    # 7. Nav2 Stack
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

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
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
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    )

    # 8. explore_lite Autonomous Exploration
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

    # 9. Stall Detector & Final Safety Command Gate
    stall_detector_node = ExecuteProcess(
        cmd=[
            'python3', '-s', os.path.join(nodes_dir, 'stall_detector_node.py'),
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', 'enabled:=true',
            '-p', 'enable_auto_reverse:=false',
            '-p', 'enable_auto_rotate_recovery:=false',
            '-p', 'require_operator_unlatch:=true',
            '-p', 'sensor_timeout_sec:=0.38',
            '-p', 'depth_mode:=required',
            '-p', 'stall_confirm_duration_sec:=0.22',
            '-p', 'cmd_vel_threshold:=0.075',
            '-p', 'wheel_vel_threshold:=0.060',
            '-p', 'lidar_vel_stall_threshold:=0.025',
            '-p', 'publish_virtual_bumper:=true',
            '-p', 'bumper_distance_m:=0.24',
        ],
        output='screen'
    )

    # 10. Dual-Panel MP4 Video Recorder Node
    recorder_node = ExecuteProcess(
        condition=IfCondition(record_video),
        cmd=[
            'python3', '-s', os.path.join(nodes_dir, 'integrated_auto_mapping_recorder_node.py'),
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', ['output_mp4:=', output_mp4],
            '-p', ['active_estimator:=', estimator_type],
            '-p', 'fps:=15.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_estimator_type,
        declare_start_explore,
        declare_record_video,
        declare_output_mp4,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        set_no_usersite,
        set_ld_path,
        base_gazebo_launch,
        normalizer_node,
        gt_publisher_node,
        lidar_odom_node,
        ekf_f5_node,
        odom_tf_relay_f4,
        odom_tf_relay_f5,
        slam_toolbox_node,
        depth_filter_node,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_nav2,
        explore_node,
        stall_detector_node,
        recorder_node
    ])
