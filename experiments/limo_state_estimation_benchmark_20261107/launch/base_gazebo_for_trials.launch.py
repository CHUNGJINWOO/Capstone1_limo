"""
base_gazebo_for_trials.launch.py
Launches persistent Gazebo Fortress + LIMO entity + ROS-GZ Bridge for multi-trial benchmarks.
Remaps Gazebo's raw /odom -> /odom_gz_raw and /imu -> /imu_gz_raw so that per-trial instances
of `trial_sensor_normalizer_node.py` reset the trial odometry origin to (0, 0, 0) and apply
any seeded or synthetic fault parameters cleanly at the start of each trial.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lounge_exp_dir = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107'
    bench_dir = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107'
    sim_research_dir = '/home/cjinw/simulator-research/limo_sim_ws/experiments/fortress'
    mesh_dir = '/home/cjinw/simulator-research/limo_sim_ws/install/limo_description/share'

    default_world = os.path.join(lounge_exp_dir, 'worlds', 'lounge_evaluation_world.sdf')
    default_model = os.path.join(lounge_exp_dir, 'models', 'limo_eval', 'model.sdf')
    default_urdf = os.path.join(sim_research_dir, 'limo_ackermann_fortress.urdf')
    models_dir = os.path.join(lounge_exp_dir, 'models')
    nodes_dir = os.path.join(bench_dir, 'nodes')

    declare_world = DeclareLaunchArgument('world', default_value=default_world)
    declare_world_name = DeclareLaunchArgument('world_name', default_value='lounge_world')
    declare_model = DeclareLaunchArgument('model_file', default_value=default_model)
    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='-0.8')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='-0.3')
    declare_spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.05')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='0.0')
    declare_headless = DeclareLaunchArgument('headless', default_value='true')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    resource_path = f"{models_dir}:{mesh_dir}:{sim_research_dir}:{os.path.join(lounge_exp_dir, 'worlds')}"
    set_ign_resource = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path)
    set_gz_resource = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)
    set_sdf_path = SetEnvironmentVariable('SDF_PATH', models_dir)
    set_no_usersite = SetEnvironmentVariable('PYTHONNOUSERSITE', '1')

    with open(default_urdf, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    static_tf_depth_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_depth_camera_link',
        arguments=['0.12', '0.0', '-0.02', '0', '0', '0', 'base_link', 'depth_camera_link'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    static_tf_depth_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_depth_link_alias',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'depth_camera_link', 'depth_link'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    start_ign_headless = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('headless')),
        cmd=['ign', 'gazebo', '-r', '-s', LaunchConfiguration('world'), '-v', '2'],
        output='screen'
    )

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', LaunchConfiguration('world_name'),
            '-file', LaunchConfiguration('model_file'),
            '-name', 'limo',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw')
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
        ]
    )

    depth_pc_node = ExecuteProcess(
        cmd=['python3', '-s', os.path.join(nodes_dir, 'depth_to_pointcloud_fixed_node.py')],
        output='screen'
    )

    return LaunchDescription([
        declare_world,
        declare_world_name,
        declare_model,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        declare_headless,
        declare_use_sim_time,
        set_ign_resource,
        set_gz_resource,
        set_sdf_path,
        set_no_usersite,
        robot_state_publisher_node,
        static_tf_depth_cam,
        static_tf_depth_alias,
        start_ign_headless,
        spawn_entity_node,
        bridge_node,
        depth_pc_node,
    ])
