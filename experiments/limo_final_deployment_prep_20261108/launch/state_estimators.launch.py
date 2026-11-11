"""
state_estimators.launch.py
Launches all state estimation algorithms concurrently on the same live sensor stream (/odom, /imu, /scan)
with publish_tf=False so they never collide on /tf:
- B0:           /odom                              (Raw Wheel Odometry)
- F1a:          /odometry/ekf_legacy               (Legacy limo_ekf.yaml reproduction)
- F1b:          /odometry/ekf_tuned                (Tuned EKF: Wheel vx,vy + IMU yaw,wz)
- F2:           /odometry/ukf_tuned                (Tuned UKF: Wheel vx,vy + IMU yaw,wz)
- F3a:          /odometry/comp_filter              (Complementary Filter standalone)
- F3b:          /odometry/comp_slip_gate           (Original F3b with 1.4s deque window, preserved)
- F3b_improved: /odometry/comp_slip_gate_improved  (Improved F3b with 0.2s LiDAR speed cross-check)
- F4:           /odometry/lidar_odom               (2D LiDAR Trimmed-ICP Scan Matching Odometry + Latency Diag)
- F5:           /odometry/lidar_imu_wheel_ekf      (Original F5: LiDAR x,y,vx,vy + IMU yaw,wz EKF, preserved)
- F5_improved:  /odometry/f5_improved              (Improved F5: LiDAR x,y,yaw,vx,vy + IMU wz EKF)
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    exp_dir = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_final_deployment_prep_20261108'
    config_dir = os.path.join(exp_dir, 'config')
    nodes_dir = os.path.join(exp_dir, 'nodes')
    bin_dir = os.path.join(exp_dir, 'bin')
    rl_lib_dir = '/home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/robot_localization/lib'

    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld = f"{rl_lib_dir}:{existing_ld}" if existing_ld else rl_lib_dir

    set_no_usersite = SetEnvironmentVariable('PYTHONNOUSERSITE', '1')
    set_ld_path = SetEnvironmentVariable('LD_LIBRARY_PATH', new_ld)

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

    comp_filter_node = ExecuteProcess(
        cmd=['python3', '-s', os.path.join(nodes_dir, 'complementary_filter_node.py')],
        output='screen'
    )

    lidar_odom_node = ExecuteProcess(
        cmd=['python3', '-s', os.path.join(nodes_dir, 'lidar_scan_matcher_odom_node.py')],
        output='screen'
    )

    # Original F5 (preserved on /odometry/lidar_imu_wheel_ekf for backward compatibility + /odometry/f5_original alias)
    ekf_f5_orig_node = ExecuteProcess(
        cmd=[
            os.path.join(bin_dir, 'ekf_node'),
            '--ros-args',
            '-r', '__node:=ekf_lidar_imu_fusion_node',
            '--params-file', os.path.join(config_dir, 'fusion_f5_original.yaml'),
            '-r', 'odometry/filtered:=/odometry/lidar_imu_wheel_ekf'
        ],
        output='screen'
    )

    # Improved F5 (/odometry/f5_improved)
    ekf_f5_improved_node = ExecuteProcess(
        cmd=[
            os.path.join(bin_dir, 'ekf_node'),
            '--ros-args',
            '-r', '__node:=ekf_f5_improved_node',
            '--params-file', os.path.join(config_dir, 'fusion_f5_improved.yaml'),
            '-r', 'odometry/filtered:=/odometry/f5_improved'
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
        ekf_f5_orig_node,
        ekf_f5_improved_node,
    ])
