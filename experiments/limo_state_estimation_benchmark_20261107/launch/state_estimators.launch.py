"""
state_estimators.launch.py
Launches all candidate state estimation algorithms concurrently on the same live or replayed
sensor stream (/odom, /imu, /scan) with publish_tf=False so they never collide on /tf:
- B0:  /odom                          (Raw Wheel Odometry from Gazebo AckermannSteering)
- F1a: /odometry/ekf_legacy           (robot_localization ekf_node with legacy limo_ekf.yaml)
- F1b: /odometry/ekf_tuned            (robot_localization ekf_node with decoupled vx,vy + IMU yaw,wz)
- F2:  /odometry/ukf_tuned            (robot_localization ukf_node with decoupled vx,vy + IMU yaw,wz)
- F3a: /odometry/comp_filter          (Complementary Filter standalone: Wheel + IMU)
- F3b: /odometry/comp_slip_gate       (Complementary Filter + Slip/Stall Gate: Wheel + IMU + Scan rate)
- F4:  /odometry/lidar_odom           (2D LiDAR Trimmed-ICP Scan Matching Odometry)
- F5:  /odometry/lidar_imu_wheel_ekf  (EKF fusing F4 LiDAR Odom + F3b Slip-Gated Velocity + IMU)
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    bench_dir = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107'
    config_dir = os.path.join(bench_dir, 'config')
    nodes_dir = os.path.join(bench_dir, 'nodes')
    bin_dir = os.path.join(bench_dir, 'bin')
    rl_lib_dir = '/home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/robot_localization/lib'

    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld = f"{rl_lib_dir}:{existing_ld}" if existing_ld else rl_lib_dir

    set_no_usersite = SetEnvironmentVariable('PYTHONNOUSERSITE', '1')
    set_ld_path = SetEnvironmentVariable('LD_LIBRARY_PATH', new_ld)

    # F1a: Legacy EKF (reproducing limo_ekf.yaml)
    ekf_legacy_node = ExecuteProcess(
        cmd=[
            os.path.join(bin_dir, 'ekf_node'),
            '--ros-args',
            '-r', '__node:=ekf_legacy_node',
            '--params-file', os.path.join(config_dir, 'ekf_legacy_limo.yaml'),
            '-r', 'odometry/filtered:=/odometry/ekf_legacy'
        ],
        output='screen'
    )

    # F1b: Tuned EKF
    ekf_tuned_node = ExecuteProcess(
        cmd=[
            os.path.join(bin_dir, 'ekf_node'),
            '--ros-args',
            '-r', '__node:=ekf_tuned_node',
            '--params-file', os.path.join(config_dir, 'ekf_tuned.yaml'),
            '-r', 'odometry/filtered:=/odometry/ekf_tuned'
        ],
        output='screen'
    )

    # F2: Tuned UKF
    ukf_tuned_node = ExecuteProcess(
        cmd=[
            os.path.join(bin_dir, 'ukf_node'),
            '--ros-args',
            '-r', '__node:=ukf_tuned_node',
            '--params-file', os.path.join(config_dir, 'ukf_tuned.yaml'),
            '-r', 'odometry/filtered:=/odometry/ukf_tuned'
        ],
        output='screen'
    )

    # F3a & F3b: Complementary Filter standalone + Complementary Filter with Slip/Stall Gate
    comp_filter_node = ExecuteProcess(
        cmd=['python3', '-s', os.path.join(nodes_dir, 'complementary_filter_node.py')],
        output='screen'
    )

    # F4: 2D LiDAR Scan Matching Odometry
    lidar_odom_node = ExecuteProcess(
        cmd=['python3', '-s', os.path.join(nodes_dir, 'lidar_scan_matcher_odom_node.py')],
        output='screen'
    )

    # F5: Multi-Sensor EKF (LiDAR Odom + Slip-Gated Wheel + IMU)
    ekf_lidar_imu_node = ExecuteProcess(
        cmd=[
            os.path.join(bin_dir, 'ekf_node'),
            '--ros-args',
            '-r', '__node:=ekf_lidar_imu_fusion_node',
            '--params-file', os.path.join(config_dir, 'fusion_lidar_imu_ekf.yaml'),
            '-r', 'odometry/filtered:=/odometry/lidar_imu_wheel_ekf'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_no_usersite,
        set_ld_path,
        ekf_legacy_node,
        ekf_tuned_node,
        ukf_tuned_node,
        comp_filter_node,
        lidar_odom_node,
        ekf_lidar_imu_node,
    ])
