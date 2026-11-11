#!/usr/bin/env python3
"""
limo_integrated_auto_mapping_sim.launch.py
Master Simulation Launch File for LIMO Pro Integrated Auto-Mapping:
Fully self-contained in capstone_ws:
1. Gazebo Fortress simulation (Lounge/MC3F World) + LIMO Pro Ackermann entity + ROS-GZ Bridge
2. Simulation Ground Truth & Sensor Normalizer
3. Simulation Depth Image to PointCloud2 converter
4. DaBai 3D Depth Obstacle Filter (limo_depth_obstacles package)
5. Selectable State Estimator: F4 (LiDAR ICP) or F5_improved (ICP + IMU EKF)
6. Single-Authority Odom TF Relay (odom -> base_footprint, publishing /odom_nav2)
7. Online Asynchronous SLAM Toolbox (map -> odom)
8. Nav2 Navigation Stack (controller_server gated strictly to /cmd_vel_raw)
9. explore_lite Autonomous Frontier Exploration Node
10. Stall Detector & Final Safety Command Gate (/cmd_vel_raw -> /cmd_vel, depth_mode=required)
11. Optional Telemetry Video Recorder (integrated_recorder_node)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_limo_capstone = get_package_share_directory('limo_capstone')
    pkg_limo_depth = get_package_share_directory('limo_depth_obstacles')
    pkg_limo_desc = get_package_share_directory('limo_description')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')

    # Simulation assets self-contained in limo_capstone and limo_description
    default_world = os.path.join(pkg_limo_capstone, 'worlds', 'lounge_evaluation_world.sdf')
    default_model = os.path.join(pkg_limo_capstone, 'models', 'limo_eval', 'model.sdf')
    default_urdf = os.path.join(pkg_limo_capstone, 'urdf', 'limo_ackermann_fortress.urdf')

    config_slam = os.path.join(pkg_limo_capstone, 'config', 'integrated_slam_params.yaml')
    config_nav2 = os.path.join(pkg_limo_capstone, 'config', 'integrated_nav2_params.yaml')
    config_explore = os.path.join(pkg_limo_capstone, 'config', 'integrated_explore_params.yaml')
    config_ekf = os.path.join(pkg_limo_capstone, 'config', 'fusion_f5_improved.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    estimator_type = LaunchConfiguration('estimator_type', default='f4')
    start_explore = LaunchConfiguration('start_explore', default='true')
    depth_mode = LaunchConfiguration('depth_mode', default='required')
    sensor_timeout_sec = LaunchConfiguration('sensor_timeout_sec', default='0.38')
    record_video = LaunchConfiguration('record_video', default='false')
    output_mp4 = LaunchConfiguration('output_mp4', default='/tmp/limo_integrated_auto_mapping_sim.mp4')
    spawn_x = LaunchConfiguration('spawn_x', default='-1.10')
    spawn_y = LaunchConfiguration('spawn_y', default='-0.60')
    spawn_z = LaunchConfiguration('spawn_z', default='0.05')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='1.5708')
    world_path = LaunchConfiguration('world', default=default_world)

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_estimator_type = DeclareLaunchArgument(
        'estimator_type', default_value='f4',
        choices=['f4', 'f5_improved'],
        description='Selected state estimator: f4 (LiDAR ICP) or f5_improved (ICP+IMU EKF)'
    )
    declare_start_explore = DeclareLaunchArgument('start_explore', default_value='true')
    declare_depth_mode = DeclareLaunchArgument('depth_mode', default_value='required',
                                               description='Depth camera fail-safe mode: required, optional, disabled')
    declare_sensor_timeout = DeclareLaunchArgument('sensor_timeout_sec', default_value='0.38')
    declare_record_video = DeclareLaunchArgument('record_video', default_value='false')
    declare_output_mp4 = DeclareLaunchArgument('output_mp4', default_value='/tmp/limo_integrated_auto_mapping_sim.mp4')
    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='-1.10')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='-0.60')
    declare_spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.05')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='1.5708')
    declare_world = DeclareLaunchArgument('world', default_value=default_world)

    # Resource paths for Gazebo Fortress
    models_dir = os.path.join(pkg_limo_capstone, 'models')
    mesh_dir = os.path.dirname(pkg_limo_desc)
    resource_path = f"{models_dir}:{mesh_dir}:{os.path.dirname(default_world)}"
    set_ign_res = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path)
    set_gz_res = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)

    # Robot State Publisher & Static TFs
    with open(default_urdf, 'r') as f:
        robot_desc = f.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )

    static_tf_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_depth_camera_link',
        arguments=['0.12', '0.0', '-0.02', '0', '0', '0', 'base_link', 'depth_camera_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    static_tf_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_depth_link_alias',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'depth_camera_link', 'depth_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 1. Gazebo Fortress Engine & Bridges
    ign_proc = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', world_path, '-v', '2'],
        output='screen'
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'lounge_world',
            '-file', default_model,
            '-name', 'limo',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw
        ]
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/lounge/overhead@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/world/lounge_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        remappings=[
            ('/odom', '/odom_gz_raw'),
            ('/imu', '/imu_gz_raw'),
            ('/tf', '/gz/wheel_tf_raw'),
            ('/world/lounge_world/dynamic_pose/info', '/gz/dynamic_pose_info'),
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2. Simulation Helpers (Normalizer, GT, Depth to PointCloud)
    normalizer_node = Node(
        package='limo_capstone',
        executable='sensor_normalizer_node',
        name='sensor_normalizer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    gt_node = Node(
        package='limo_capstone',
        executable='ground_truth_publisher_node',
        name='ground_truth_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    d2p_node = Node(
        package='limo_capstone',
        executable='depth_to_pointcloud_fixed_node',
        name='depth_to_pointcloud_fixed_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. Depth Obstacle Filter (from limo_depth_obstacles)
    depth_filter_node = Node(
        package='limo_depth_obstacles',
        executable='depth_obstacle_filter_node',
        name='depth_obstacle_filter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'height_frame': 'base_footprint', 'min_height': -0.02}]
    )

    # 4. State Estimators: F4 vs F5_improved
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

    # 5. Single-Authority Odom TF Relay
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

    # 6. SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': config_slam
        }.items()
    )

    # 7. Nav2 Navigation Stack (gated strictly to /cmd_vel_raw)
    # Ackermann chassis cannot rotate in place and reverse is blocked by the gate:
    # use a recovery tree without Spin/BackUp (key exists as a placeholder in the yaml).
    ackermann_bt = os.path.join(pkg_limo_capstone, 'config', 'navigate_to_pose_ackermann_no_reverse.xml')
    nav2_params_file = RewrittenYaml(
        source_file=config_nav2,
        param_rewrites={'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': ackermann_bt},
    )

    nav2_launch = GroupAction(
        actions=[
            # Every Nav2 velocity command must reach /cmd_vel only through the stall gate:
            # controller_server -> cmd_vel_nav (Nav2 default) -> velocity_smoother -> /cmd_vel_raw
            SetRemap(src='cmd_vel_smoothed', dst='/cmd_vel_raw'),
            # behavior_server (spin/backup/drive_on_heading) publishes 'cmd_vel' directly
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

    # 8. explore_lite Frontier Exploration Node
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[config_explore, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_explore)
    )

    # 9. Stall Safety Gate (/cmd_vel_raw -> /cmd_vel)
    stall_gate_node = Node(
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
            'estimator_odom_topic': '/odom_nav2',
            'use_sim_time': use_sim_time
        }]
    )

    # 10. Optional Video Recorder
    recorder_node = Node(
        package='limo_capstone',
        executable='integrated_recorder_node',
        name='integrated_recorder_node',
        output='screen',
        parameters=[{
            'output_mp4': output_mp4,
            'active_estimator': estimator_type,
            'fps': 15.0,
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(record_video)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_estimator_type,
        declare_start_explore,
        declare_depth_mode,
        declare_sensor_timeout,
        declare_record_video,
        declare_output_mp4,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        declare_world,
        set_ign_res,
        set_gz_res,
        rsp_node,
        static_tf_cam,
        static_tf_alias,
        ign_proc,
        spawn_node,
        bridge_node,
        normalizer_node,
        gt_node,
        d2p_node,
        depth_filter_node,
        f4_node,
        f5_ekf_node,
        relay_f4,
        relay_f5,
        slam_launch,
        nav2_launch,
        explore_node,
        stall_gate_node,
        recorder_node
    ])
