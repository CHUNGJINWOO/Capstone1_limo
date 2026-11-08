#!/usr/bin/env python3
"""
integrated_stage1_to_3.launch.py
Launches Stages 1~3 of the Integrated Auto-Mapping Architecture:
1. TF & Sensor Input Plumbing:
   - Enforces single authority for odom -> base_footprint TF.
   - Raw wheel odometry preserved on /odom.
   - Eliminates reverse/circular static TFs.
2. Selectable State Estimation (F4 vs F5_improved):
   - estimator_type:='f4'          -> F4 Trimmed-ICP Odometry (/odometry/lidar_odom)
   - estimator_type:='f5_improved' -> F5 LiDAR+IMU EKF (/odometry/f5_improved)
3. SLAM Toolbox Online Mapping:
   - async_slam_toolbox_node with integrated_slam_params.yaml
   - Broadcasts map -> odom TF at 50 Hz.
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

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_estimator_type = DeclareLaunchArgument(
        'estimator_type', default_value='f4',
        choices=['f4', 'f5_improved'],
        description='Selected state estimator: f4 (LiDAR ICP) or f5_improved (LiDAR+IMU EKF)'
    )

    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld = f"{rl_lib_dir}:{existing_ld}" if existing_ld else rl_lib_dir
    set_no_usersite = SetEnvironmentVariable('PYTHONNOUSERSITE', '1')
    set_ld_path = SetEnvironmentVariable('LD_LIBRARY_PATH', new_ld)

    # 1. F4 LiDAR Trimmed-ICP Odometry Node (Always launched; serves F4 odom or F5 odom0)
    lidar_odom_node = ExecuteProcess(
        cmd=['python3', '-s', os.path.join(nodes_dir, 'lidar_scan_matcher_odom_node.py')],
        output='screen'
    )

    # 2. F5_improved EKF Node (Launched when estimator_type == 'f5_improved')
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

    # 3. Odom TF Relay Node: The Single Authoritative Publisher for odom -> base_footprint TF
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

    # 4. SLAM Toolbox Online Asynchronous Node: Sole Owner of map -> odom TF
    slam_params_file = os.path.join(config_dir, 'integrated_slam_params.yaml')
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_estimator_type,
        set_no_usersite,
        set_ld_path,
        lidar_odom_node,
        ekf_f5_node,
        odom_tf_relay_f4,
        odom_tf_relay_f5,
        slam_node,
    ])
