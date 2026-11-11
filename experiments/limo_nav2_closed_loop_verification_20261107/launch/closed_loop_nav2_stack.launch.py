"""
closed_loop_nav2_stack.launch.py
Launches the real Nav2 closed-loop stack (map_server, planner_server, controller_server,
behavior_server, bt_navigator, lifecycle_manager) bound to:
- Static transform `map -> odom` at the trial's spawn pose (spawn_x, spawn_y, spawn_yaw)
- Active state estimator's `odom -> base_footprint` TF and `/odom_nav2` topic (via odom_tf_relay_node)
- Velocity command output remapped `/cmd_vel -> /cmd_vel_raw` (routed through stall_detector_node -> /cmd_vel)
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    exp_dir = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_nav2_closed_loop_verification_20261107'
    default_params = os.path.join(exp_dir, 'config', 'nav2_closed_loop_params.yaml')
    default_map = os.path.join(exp_dir, 'maps', 'lounge_map.yaml')

    declare_params = DeclareLaunchArgument('params_file', default_value=default_params)
    declare_map = DeclareLaunchArgument('map_yaml', default_value=default_map)
    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='1.0')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='0.3')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='-1.5708')

    set_no_usersite = SetEnvironmentVariable('PYTHONNOUSERSITE', '1')

    # Publish exact static transform map -> odom at trial start pose so map->odom->base_footprint
    # is driven 100% by the active state estimator under test (without AMCL overwriting odom drift)
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=[
            LaunchConfiguration('spawn_x'),
            LaunchConfiguration('spawn_y'),
            '0.0',
            LaunchConfiguration('spawn_yaw'),
            '0.0',
            '0.0',
            'map',
            'odom'
        ],
        parameters=[{'use_sim_time': True}]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'yaml_filename': LaunchConfiguration('map_yaml'), 'use_sim_time': True}
        ]
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': True}]
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': True}],
        remappings=[('cmd_vel', '/cmd_vel_raw')]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': True}],
        remappings=[('cmd_vel', '/cmd_vel_raw')]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': True}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': [
                'map_server',
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
            ]
        }]
    )

    return LaunchDescription([
        declare_params,
        declare_map,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_yaw,
        set_no_usersite,
        map_server_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_node,
    ])
