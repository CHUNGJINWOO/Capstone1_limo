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

# Config split (real vs simulation): this launch reads ONLY real-robot YAML files
#   integrated_nav2_real_depth_params.yaml, integrated_nav2_lidar_only_params.yaml, integrated_nav2_lidar_only_slow_params.yaml,
#   integrated_explore_real_params.yaml. The simulation baseline files integrated_nav2_params.yaml and integrated_explore_params.yaml
#   are NOT used here (see docs/CONFIG_SIM_VS_REAL.md).
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


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
    config_nav2_depth = os.path.join(pkg_limo_capstone, 'config', 'integrated_nav2_real_depth_params.yaml')
    config_nav2_lidar = os.path.join(pkg_limo_capstone, 'config', 'integrated_nav2_lidar_only_params.yaml')
    config_nav2_lidar_slow = os.path.join(pkg_limo_capstone, 'config', 'integrated_nav2_lidar_only_slow_params.yaml')
    config_explore = os.path.join(pkg_limo_capstone, 'config', 'integrated_explore_real_params.yaml')
    config_ekf_f5 = os.path.join(pkg_limo_capstone, 'config', 'fusion_f5_improved.yaml')
    config_ekf_f1b = os.path.join(pkg_limo_capstone, 'config', 'ekf_tuned.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    estimator_type = LaunchConfiguration('estimator_type', default='f4')
    run_icp_observer = LaunchConfiguration('run_icp_observer', default='true')
    start_explore = LaunchConfiguration('start_explore', default='false')
    use_depth_camera = LaunchConfiguration('use_depth_camera', default='false')
    speed_profile = LaunchConfiguration('speed_profile', default='normal')
    depth_mode = LaunchConfiguration('depth_mode', default='required')
    sensor_timeout_sec = LaunchConfiguration('sensor_timeout_sec', default='0.38')
    allow_motor_drive = LaunchConfiguration('allow_motor_drive', default='false')
    drive_mode = LaunchConfiguration('drive_mode', default='diff')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_estimator_type = DeclareLaunchArgument(
        'estimator_type', default_value='f4',
        choices=['b0', 'f1b', 'f3b', 'f4', 'f5_improved'],
        description='Selected state estimator: b0 (Wheel Odom), f1b (Tuned EKF), f3b (Comp+SlipGate), f4 (LiDAR ICP), or f5_improved (ICP+IMU EKF)'
    )
    declare_run_icp_observer = DeclareLaunchArgument(
        'run_icp_observer', default_value='true',
        description='Run F4 LiDAR ICP as background observer for stall detection/diagnostic comparison in B0/F1b/F3b modes'
    )
    declare_start_explore = DeclareLaunchArgument(
        'start_explore', default_value='false',
        description='Start explore_lite frontier exploration (disabled by default for initial test)'
    )
    declare_use_depth_camera = DeclareLaunchArgument(
        'use_depth_camera', default_value='false',
        description='Whether to enable Orbbec DaBai camera driver, TF, depth filter, and costmap layer'
    )
    declare_depth_mode = DeclareLaunchArgument(
        'depth_mode', default_value='required',
        description='Depth camera fail-safe mode for stall detector: required, optional, disabled'
    )
    declare_sensor_timeout = DeclareLaunchArgument('sensor_timeout_sec', default_value='0.38')
    declare_allow_motor = DeclareLaunchArgument(
        'allow_motor_drive', default_value='false',
        description='SAFETY LOCK: Set true only after MCU watchdog & physical E-Stop bench check!'
    )
    declare_speed_profile = DeclareLaunchArgument(
        'speed_profile', default_value='normal',
        description="'normal' (current limits, desired_linear_vel 0.32 m/s) or 'slow' (first on-ground tests: "
                    "0.15 m/s, rotation <= 0.5 rad/s, hard cap in velocity_smoother). 'slow' exists for the "
                    "lidar-only configuration only: with use_depth_camera:=true the launch shuts down."
    )
    declare_drive_mode = DeclareLaunchArgument(
        'drive_mode', default_value='diff',
        description="Nav2 kinematics profile: 'diff' (4WD differential; stock BT with Spin/BackUp, "
                    "RPP rotate-to-heading) or 'ackermann' (no-reverse BT, no in-place rotation). "
                    "Must match the LIMO hardware motion_mode (/limo_status: 0 = four-wheel differential)."
    )
    declare_enable_stall_detector = DeclareLaunchArgument(
        'enable_stall_detector', default_value='true',
        description='DEPRECATED / IGNORED: the stall gate is always in the command path '
                    '(Nav2 -> /cmd_vel_raw -> stall_detector_node -> /cmd_vel or /cmd_vel_locked). '
                    'Kept only so existing command lines that pass it still work.'
    )

    # Static TF & Extrinsics Configuration Arguments
    declare_base_footprint_z = DeclareLaunchArgument(
        'base_footprint_z', default_value='0.0',
        description='Z offset between base_footprint and base_link (0.0 for physical 4WD chassis)'
    )
    declare_publish_camera_tf = DeclareLaunchArgument(
        'publish_camera_tf', default_value='true',
        description='Whether to publish static TF for DaBai depth camera'
    )
    declare_camera_parent = DeclareLaunchArgument('camera_parent_frame', default_value='base_link')
    declare_camera_optical = DeclareLaunchArgument('camera_optical_frame', default_value='camera_link',
                                                 description='Root camera frame to attach to base_link (camera_link connects to astra_camera TF tree)')
    declare_camera_x = DeclareLaunchArgument('camera_x', default_value='0.084',
                                           description='Camera X mounting offset [UNVERIFIED: CAD reference 0.084m]')
    declare_camera_y = DeclareLaunchArgument('camera_y', default_value='0.0',
                                           description='Camera Y mounting offset [UNVERIFIED: CAD reference 0.0m]')
    declare_camera_z = DeclareLaunchArgument('camera_z', default_value='0.03',
                                           description='Camera Z mounting offset [UNVERIFIED: CAD reference 0.03m]')
    declare_camera_roll = DeclareLaunchArgument('camera_roll', default_value='0.0',
                                              description='Camera Roll mounting angle [Robot FLU to camera_link FLU: 0.0 rad]')
    declare_camera_pitch = DeclareLaunchArgument('camera_pitch', default_value='0.0',
                                               description='Camera Pitch mounting angle [Robot FLU to camera_link FLU: 0.0 rad]')
    declare_camera_yaw = DeclareLaunchArgument('camera_yaw', default_value='0.0',
                                             description='Camera Yaw mounting angle [Robot FLU to camera_link FLU: 0.0 rad]')

    # Static TF: base_footprint (Parent) -> base_link (Child)
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_footprint_to_base_link',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', LaunchConfiguration('base_footprint_z'),
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link'
        ]
    )

    # Static TF: base_link -> DaBai camera optical frame (Configurable Extrinsics, published only when Depth is ON)
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--roll', LaunchConfiguration('camera_roll'),
            '--pitch', LaunchConfiguration('camera_pitch'),
            '--yaw', LaunchConfiguration('camera_yaw'),
            '--frame-id', LaunchConfiguration('camera_parent_frame'),
            '--child-frame-id', LaunchConfiguration('camera_optical_frame')
        ],
        condition=IfCondition(PythonExpression([
            "'", use_depth_camera, "' == 'true' and '", LaunchConfiguration('publish_camera_tf'), "' == 'true'"
        ]))
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

    # C. Orbbec DaBai Depth Camera Driver (Launched only when use_depth_camera is true)
    dabai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_camera, 'launch', 'dabai.launch.py')),
        condition=IfCondition(use_depth_camera)
    )

    # D. Robot State Publisher (URDF)
    load_urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_limo_desc, 'launch', 'load_urdf.launch.py'))
    )

    # 2. Depth Obstacle Filter (Active only when use_depth_camera is true)
    depth_filter_node = Node(
        package='limo_depth_obstacles',
        executable='depth_obstacle_filter_node',
        name='depth_obstacle_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'height_frame': 'base_footprint',
            'input_topic': '/camera/depth/points',
            'point_stride': 4,
            'processing_hz': 4.0,
            'data_timeout_sec': 0.75,
        }],
        condition=IfCondition(use_depth_camera)
    )

    # 3. State Estimators: B0, F1b, F3b, F4, F5_improved
    is_b0 = PythonExpression(["'", estimator_type, "' == 'b0'"])
    is_f1b = PythonExpression(["'", estimator_type, "' == 'f1b'"])
    is_f3b = PythonExpression(["'", estimator_type, "' == 'f3b'"])
    is_f4 = PythonExpression(["'", estimator_type, "' == 'f4'"])
    is_f5 = PythonExpression(["'", estimator_type, "' == 'f5_improved'"])
    run_f4 = PythonExpression([
        "'", estimator_type, "' in ['f4', 'f5_improved'] or '", run_icp_observer, "' == 'true'"
    ])

    # F1b: Tuned 2D EKF (Wheel linear velocity + IMU yaw/wz)
    f1b_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_f1b_node',
        output='screen',
        parameters=[config_ekf_f1b, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/ekf_tuned')],
        condition=IfCondition(is_f1b)
    )

    # F3b: Complementary Filter + Physics-Informed Slip/Stall Gate
    comp_filter_node = Node(
        package='limo_capstone',
        executable='complementary_filter_node',
        name='complementary_filter_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(is_f3b)
    )

    # F4: 2D LiDAR Scan-Matching Odometry (ICP)
    # Launched when selected as estimator OR as diagnostic observer for B0/F1b/F3b
    f4_node = Node(
        package='limo_capstone',
        executable='lidar_scan_matcher_odom_node',
        name='lidar_scan_matcher_odom_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(run_f4)
    )

    # F5_improved: Composite Sensor Fusion (F4 ICP Pose/Vel + IMU wz)
    f5_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_f5_improved_node',
        output='screen',
        parameters=[config_ekf_f5, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/f5_improved')],
        condition=IfCondition(is_f5)
    )

    # 4. Single-Authority Odom TF Relay
    # Subscribes to the active estimator topic, publishes odom -> base_footprint on /tf,
    # and republishes /odom_nav2 for Nav2 stack. Exactly ONE relay is active.
    relay_b0 = Node(
        package='limo_capstone',
        executable='odom_tf_relay_node',
        name='odom_tf_relay_node',
        output='screen',
        parameters=[{
            'active_odom_topic': '/odom',
            'nav2_odom_topic': '/odom_nav2',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(is_b0)
    )

    relay_f1b = Node(
        package='limo_capstone',
        executable='odom_tf_relay_node',
        name='odom_tf_relay_node',
        output='screen',
        parameters=[{
            'active_odom_topic': '/odometry/ekf_tuned',
            'nav2_odom_topic': '/odom_nav2',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(is_f1b)
    )

    relay_f3b = Node(
        package='limo_capstone',
        executable='odom_tf_relay_node',
        name='odom_tf_relay_node',
        output='screen',
        parameters=[{
            'active_odom_topic': '/odometry/comp_slip_gate',
            'nav2_odom_topic': '/odom_nav2',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(is_f3b)
    )

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

    # 6. Nav2 Navigation Stack
    enable_stall_detector = LaunchConfiguration('enable_stall_detector')
    selected_nav2_params = PythonExpression([
        "'", config_nav2_depth, "' if '", use_depth_camera, "' == 'true' else ('", config_nav2_lidar_slow,
        "' if '", speed_profile, "' == 'slow' else '", config_nav2_lidar, "')"
    ])
    # Fail closed: never run a "slow" request on a configuration that has no slow profile.
    bad_speed_profile = Shutdown(
        reason="speed_profile must be 'normal' or 'slow', and 'slow' needs use_depth_camera:=false",
        condition=IfCondition(PythonExpression([
            "('", speed_profile, "' not in ['normal', 'slow']) or ('", speed_profile, "' == 'slow' and '",
            use_depth_camera, "' == 'true')"
        ]))
    )

    # Command routing (fail-closed, no bypass):
    # Every velocity command from Nav2 goes to /cmd_vel_raw. The stall gate is the only node
    # that publishes the final command: /cmd_vel (motor driver) when allow_motor_drive == 'true',
    # otherwise /cmd_vel_locked. enable_stall_detector is deprecated and has no effect.
    #   controller_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed -> /cmd_vel_raw
    #   behavior_server (spin/backup/drive_on_heading)       cmd_vel          -> /cmd_vel_raw
    # Do NOT add a global SetRemap('cmd_vel'): it is applied before node-level remaps and would
    # make velocity_smoother consume its own output (self-input loop) and publish straight to /cmd_vel.
    ackermann_bt = os.path.join(pkg_limo_capstone, 'config', 'navigate_to_pose_ackermann_no_reverse.xml')
    stock_bt = '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'
    nav2_params_file = RewrittenYaml(
        source_file=selected_nav2_params,
        param_rewrites={
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': PythonExpression([
                "'", ackermann_bt, "' if '", drive_mode, "' == 'ackermann' else '", stock_bt, "'"]),
            'controller_server.ros__parameters.FollowPath.use_rotate_to_heading': PythonExpression([
                "'false' if '", drive_mode, "' == 'ackermann' else 'true'"]),
        },
        convert_types=True,
    )

    nav2_launch = GroupAction(
        actions=[
            SetRemap(src='cmd_vel_smoothed', dst='/cmd_vel_raw'),
            SetRemap(src='behavior_server:cmd_vel', dst='/cmd_vel_raw'),
            SetRemap(src='odom', dst='/odom_nav2'),
            SetRemap(src='/odom', dst='/odom_nav2'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file,
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
    effective_depth_mode = PythonExpression([
        "'disabled' if '", use_depth_camera, "' != 'true' else '", depth_mode, "'"
    ])
    effective_require_lidar = PythonExpression([
        "'true' if ('", run_icp_observer, "' == 'true' or '", estimator_type, "' in ['f4', 'f5_improved']) else 'false'"
    ])

    # Single stall gate: /cmd_vel_raw -> (/cmd_vel | /cmd_vel_locked). Exactly one instance always runs.
    # Output topic is fail-closed: only the exact string 'true' reaches the motor driver topic.
    gate_out_topic = PythonExpression([
        "'/cmd_vel' if '", allow_motor_drive, "' == 'true' else '/cmd_vel_locked'"
    ])
    stall_gate = Node(
        package='limo_capstone',
        executable='stall_detector_node',
        name='stall_detector_node',
        output='screen',
        parameters=[{
            'enabled': True,
            'enable_auto_reverse': False,
            'enable_auto_rotate_recovery': False,
            'require_operator_unlatch': False,
            'stall_confirm_duration_sec': 1.5,
            'depth_mode': effective_depth_mode,
            'sensor_timeout_sec': sensor_timeout_sec,
            'require_lidar_odom': effective_require_lidar,
            'estimator_odom_topic': '/odom_nav2',
            'use_sim_time': use_sim_time
        }],
        remappings=[('/cmd_vel', gate_out_topic)]
    )

    log_deprecated_stall_arg = LogInfo(
        condition=UnlessCondition(enable_stall_detector),
        msg="[DEPRECATED] enable_stall_detector:=false is ignored: the stall gate is always in the command path."
    )

    log_lock_notice = LogInfo(
        # Same test as gate_out_topic: only the exact string 'true' unlocks; anything else (false, TRUE, empty) is locked.
        condition=IfCondition(PythonExpression(["'", allow_motor_drive, "' != 'true'"])),
        msg="[SAFETY LOCK ACTIVE] Real robot motor drive is LOCKED (allow_motor_drive is not exactly 'true'). Commands redirected to /cmd_vel_locked."
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_estimator_type,
        declare_run_icp_observer,
        declare_start_explore,
        declare_use_depth_camera,
        declare_depth_mode,
        declare_sensor_timeout,
        declare_allow_motor,
        declare_speed_profile,
        declare_drive_mode,
        declare_enable_stall_detector,
        declare_base_footprint_z,
        declare_publish_camera_tf,
        declare_camera_parent,
        declare_camera_optical,
        declare_camera_x,
        declare_camera_y,
        declare_camera_z,
        declare_camera_roll,
        declare_camera_pitch,
        declare_camera_yaw,
        log_lock_notice,
        bad_speed_profile,
        log_deprecated_stall_arg,
        limo_base_launch,
        ydlidar_launch,
        dabai_launch,
        load_urdf_launch,
        base_footprint_tf,
        camera_tf_node,
        depth_filter_node,
        f1b_ekf_node,
        comp_filter_node,
        f4_node,
        f5_ekf_node,
        relay_b0,
        relay_f1b,
        relay_f3b,
        relay_f4,
        relay_f5,
        slam_launch,
        nav2_launch,
        explore_node,
        stall_gate
    ])
