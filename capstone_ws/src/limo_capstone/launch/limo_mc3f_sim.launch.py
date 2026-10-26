import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_limo_capstone = get_package_share_directory('limo_capstone')
    sim_research_dir = '/home/cjinw/simulator-research/limo_sim_ws/experiments/fortress'

    default_world = os.path.join(pkg_limo_capstone, 'worlds', 'mc3f_10_15.sdf')
    default_sdf = os.path.join(sim_research_dir, 'limo_ackermann_fortress.sdf')
    default_urdf = os.path.join(sim_research_dir, 'limo_ackermann_fortress.urdf')
    default_rviz_config = os.path.join(sim_research_dir, 'limo_slam_fortress.rviz')
    mesh_dir = '/home/cjinw/simulator-research/limo_sim_ws/install/limo_description/share'

    # Launch Arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to SDF world file (default: mc3f_10_15.sdf)'
    )
    declare_model_cmd = DeclareLaunchArgument(
        'model_file',
        default_value=default_sdf,
        description='Full path to robot SDF file'
    )
    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x',
        default_value='-1.07',
        description='Initial robot spawn X coordinate (in map/world frame)'
    )
    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y',
        default_value='-0.68',
        description='Initial robot spawn Y coordinate (in map/world frame)'
    )
    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.05',
        description='Initial robot spawn Z coordinate'
    )
    declare_spawn_yaw = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='2.23',
        description='Initial robot spawn Yaw orientation (in radians, facing corridor)'
    )
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run Gazebo headless (-s)'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2'
    )

    # Set resource path for meshes
    set_ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f"{mesh_dir}:{sim_research_dir}:{os.path.join(pkg_limo_capstone, 'worlds')}"
    )
    set_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{mesh_dir}:{sim_research_dir}:{os.path.join(pkg_limo_capstone, 'worlds')}"
    )

    # Robot description from URDF for robot_state_publisher
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

    # Start Ignition Gazebo headless
    start_ign_headless = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('headless')),
        cmd=['ign', 'gazebo', '-r', '-s', LaunchConfiguration('world'), '-v', '3'],
        output='screen'
    )

    # Start Ignition Gazebo with GUI
    start_ign_gui = ExecuteProcess(
        condition=UnlessCondition(LaunchConfiguration('headless')),
        cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world'), '-v', '3'],
        output='screen'
    )

    # Spawn entity at specified corridor coordinates
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'mc3f_world',
            '-file', LaunchConfiguration('model_file'),
            '-name', 'limo',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw')
        ]
    )

    # ROS-GZ Bridge
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
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'
        ]
    )

    # RViz2 Node
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        declare_world_cmd,
        declare_model_cmd,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        declare_headless_cmd,
        declare_use_sim_time_cmd,
        declare_rviz_cmd,
        set_ign_resource,
        set_gz_resource,
        robot_state_publisher_node,
        start_ign_headless,
        start_ign_gui,
        spawn_entity_node,
        bridge_node,
        rviz_node
    ])
