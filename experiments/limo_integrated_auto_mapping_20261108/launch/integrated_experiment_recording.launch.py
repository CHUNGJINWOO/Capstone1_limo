#!/usr/bin/env python3
"""
integrated_experiment_recording.launch.py
Dedicated Launch File for Auto-Mapping Experiment with Automated Multi-Modal Recording:
- Includes limo_integrated_auto_mapping_master.launch.py
- Launches Nav2 Action Lifecycle Monitor node
- Launches rosbag2 recording with verified QoS overrides and topic list
- Saves all artifacts into a unified experiment directory
"""
import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    config_dir = os.path.join(pkg_dir, 'config')
    nodes_dir = os.path.join(pkg_dir, 'nodes')
    launch_dir = os.path.join(pkg_dir, 'launch')
    experiments_dir = os.path.join(pkg_dir, 'evaluation', 'experiments')
    rl_lib_dir = os.path.abspath(os.path.join(pkg_dir, "../../capstone_ws/install/robot_localization/lib"))

    default_ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    default_exp_id = f"exp_{default_ts}_recording_launch"
    default_exp_dir = os.path.join(experiments_dir, default_exp_id)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    estimator_type = LaunchConfiguration('estimator_type', default='f4')
    start_explore = LaunchConfiguration('start_explore', default='true')
    record_bag = LaunchConfiguration('record_bag', default='true')
    record_video = LaunchConfiguration('record_video', default='true')
    exp_dir = LaunchConfiguration('exp_dir', default=default_exp_dir)
    spawn_x = LaunchConfiguration('spawn_x', default='-1.10')
    spawn_y = LaunchConfiguration('spawn_y', default='-0.60')
    spawn_z = LaunchConfiguration('spawn_z', default='0.05')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='1.5708')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_estimator_type = DeclareLaunchArgument('estimator_type', default_value='f4', choices=['f4', 'f5_improved'])
    declare_start_explore = DeclareLaunchArgument('start_explore', default_value='true')
    declare_record_bag = DeclareLaunchArgument('record_bag', default_value='true')
    declare_record_video = DeclareLaunchArgument('record_video', default_value='true')
    declare_exp_dir = DeclareLaunchArgument('exp_dir', default_value=default_exp_dir)
    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='-1.10')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='-0.60')
    declare_spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.05')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='1.5708')

    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld = f"{rl_lib_dir}:/opt/ros/humble/lib:{existing_ld}" if existing_ld else f"{rl_lib_dir}:/opt/ros/humble/lib"
    set_no_usersite = SetEnvironmentVariable('PYTHONNOUSERSITE', '1')
    set_ld_path = SetEnvironmentVariable('LD_LIBRARY_PATH', new_ld)

    # 1. Master Stack Include
    master_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'limo_integrated_auto_mapping_master.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'estimator_type': estimator_type,
            'start_explore': start_explore,
            'record_video': record_video,
            'output_mp4': [exp_dir, '/videos/dual_panel_recording.mp4'],
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'spawn_yaw': spawn_yaw
        }.items()
    )

    # 2. Nav2 Action Monitor Node
    action_monitor_node = ExecuteProcess(
        cmd=[
            'python3', '-s', os.path.join(nodes_dir, 'nav2_action_monitor_node.py'),
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', ['events_file:=', exp_dir, '/nav2_action_events.jsonl'],
            '-p', ['summary_file:=', exp_dir, '/nav2_action_summary.json'],
            '-p', ['active_estimator:=', estimator_type]
        ],
        output='screen'
    )

    # 3. rosbag2 Recording Process
    qos_overrides = os.path.join(config_dir, 'rosbag2_qos_overrides.yaml')
    is_f4 = PythonExpression(["'", estimator_type, "' == 'f4'"])

    # Topic list
    common_topics = [
        '/scan', '/imu', '/odom', '/odom_nav2', '/tf', '/tf_static',
        '/map', '/map_metadata', '/camera/depth/points', '/depth_obstacles/points',
        '/global_costmap/costmap', '/local_costmap/costmap',
        '/cmd_vel_raw', '/cmd_vel', '/stall_detector/status_json', '/stall_detector/virtual_bumper_cloud',
        '/navigate_to_pose/_action/feedback', '/navigate_to_pose/_action/status',
        '/rosout', '/ground_truth/world_odom'
    ]

    bag_cmd_f4 = [
        'ros2', 'bag', 'record',
        '-s', 'sqlite3',
        '-o', [exp_dir, '/rosbag2/rosbag2_data'],
        '--qos-profile-overrides-path', qos_overrides,
        '--use-sim-time',
        '--include-hidden-topics',
        '/odometry/lidar_odom'
    ] + common_topics

    bag_cmd_f5 = [
        'ros2', 'bag', 'record',
        '-s', 'sqlite3',
        '-o', [exp_dir, '/rosbag2/rosbag2_data'],
        '--qos-profile-overrides-path', qos_overrides,
        '--use-sim-time',
        '--include-hidden-topics',
        '/odometry/f5_improved'
    ] + common_topics

    rosbag_node_f4 = ExecuteProcess(
        condition=IfCondition(PythonExpression(["'", record_bag, "' == 'true' and '", estimator_type, "' == 'f4'"])),
        cmd=bag_cmd_f4,
        output='screen'
    )

    rosbag_node_f5 = ExecuteProcess(
        condition=IfCondition(PythonExpression(["'", record_bag, "' == 'true' and '", estimator_type, "' == 'f5_improved'"])),
        cmd=bag_cmd_f5,
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_estimator_type,
        declare_start_explore,
        declare_record_bag,
        declare_record_video,
        declare_exp_dir,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        set_no_usersite,
        set_ld_path,
        master_launch,
        action_monitor_node,
        rosbag_node_f4,
        rosbag_node_f5
    ])
