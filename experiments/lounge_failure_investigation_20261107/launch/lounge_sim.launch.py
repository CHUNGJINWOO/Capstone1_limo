import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_base = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107'
    sim_research_dir = '/home/cjinw/simulator-research/limo_sim_ws/experiments/fortress'
    mesh_dir = '/home/cjinw/simulator-research/limo_sim_ws/install/limo_description/share'

    default_world = os.path.join(pkg_base, 'worlds', 'lounge_evaluation_world.sdf')
    default_model = os.path.join(pkg_base, 'models', 'limo_eval', 'model.sdf')
    default_urdf = os.path.join(sim_research_dir, 'limo_ackermann_fortress.urdf')
    models_dir = os.path.join(pkg_base, 'models')

    declare_world = DeclareLaunchArgument('world', default_value=default_world, description='SDF world path')
    declare_world_name = DeclareLaunchArgument('world_name', default_value='lounge_world', description='World name inside SDF')
    declare_model = DeclareLaunchArgument('model_file', default_value=default_model, description='Robot model SDF path')
    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='0.0', description='Spawn X')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='0.0', description='Spawn Y')
    declare_spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.05', description='Spawn Z')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Spawn Yaw')
    declare_headless = DeclareLaunchArgument('headless', default_value='true', description='Run headless')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time')

    # Environment variables for Gazebo model lookup
    resource_path = f"{models_dir}:{mesh_dir}:{sim_research_dir}:{os.path.join(pkg_base, 'worlds')}"
    set_ign_resource = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path)
    set_gz_resource = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)
    set_sdf_path = SetEnvironmentVariable('SDF_PATH', models_dir)

    # Robot State Publisher
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

    # Ignition Gazebo Server (Headless)
    start_ign_headless = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('headless')),
        cmd=['ign', 'gazebo', '-r', '-s', LaunchConfiguration('world'), '-v', '3'],
        output='screen'
    )

    # Spawn entity using configurable world name
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


    # ROS-GZ Bridge (including clock, cmd_vel, scan, odom, tf, depth camera image)
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
            '/imu@sensor_msgs/msg/IMU[ignition.msgs.IMU',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image'
        ]
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
        robot_state_publisher_node,
        start_ign_headless,
        spawn_entity_node,
        bridge_node
    ])
