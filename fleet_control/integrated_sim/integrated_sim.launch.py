"""Integrated simulation launch (simulation only; never for the real LIMO).

Derived from fleet_control/gazebo/localized_sim.launch.py (saved map + AMCL instead of SLAM, original plant/sensors/Nav2/Safety Gate reused
from the read-only reference workspace). Differences, and only these:
  * the spawned robot is a COPY of limo_eval_diff with an extra RGB camera (integrated_sim/models/limo_eval_diff_rgb); the original model
    file is not modified. Gazebo topic /camera/color/image_raw is bridged to ROS (extra bridge node, original bridge untouched);
  * spawn pose and the AMCL initial pose come from environment variables (INT_SPAWN_*), so the scenario can start in a free, wide area;
  * the Safety Gate (stall_detector_node) is started unchanged, delayed by INT_GATE_DELAY_S (default 20 s) like the localized profile.
Safety Gate source/parameters, Nav2 parameters and the original launch are not modified.
"""
import importlib.util
import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)


def _f(name, default):
    return float(os.environ.get(name, default))


def generate_launch_description():
    capstone = get_package_share_directory('limo_capstone')
    parent = os.path.join(capstone, 'launch', 'limo_integrated_auto_mapping_sim_diff.launch.py')
    spec = importlib.util.spec_from_file_location('reference_diff_sim', parent)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    original = module.generate_launch_description()

    # Start pose in the saved Nav2 `map` frame and the matching Gazebo world pose.
    # world = (-(my + 1.1), mx - 0.6), world yaw = map yaw + pi/2 (estimate from the localized profile; AMCL corrects the residual).
    mx, my, myaw = _f('INT_START_MAP_X', -3.2), _f('INT_START_MAP_Y', -6.3), _f('INT_START_MAP_YAW', 0.84)
    wx, wy, wyaw = -(my + 1.1), mx - 0.6, myaw + math.pi / 2
    defaults = {'spawn_x': f'{wx:.4f}', 'spawn_y': f'{wy:.4f}', 'spawn_yaw': f'{wyaw:.6f}', 'start_explore': 'false'}
    model = os.environ.get('INT_ROBOT_MODEL', os.path.join(HERE, 'models', 'limo_eval_diff_rgb', 'model.sdf'))

    actions, removed, gate_action = [], 0, None
    for action in original.entities:
        if isinstance(action, Node) and action.node_executable == 'stall_detector_node':
            gate_action = action; continue
        if isinstance(action, Node) and action.node_executable in ('odom_tf_relay_node', 'ekf_node'):
            continue
        if isinstance(action, Node) and action.node_executable == 'create':   # the robot spawn -> use the RGB copy of the model
            actions.append(Node(package='ros_gz_sim', executable='create', output='screen',
                                arguments=['-world', 'lounge_world', '-file', model, '-name', 'limo',
                                           '-x', defaults['spawn_x'], '-y', defaults['spawn_y'], '-z', '0.05', '-Y', defaults['spawn_yaw']]))
            continue
        if isinstance(action, IncludeLaunchDescription):
            source = action.launch_description_source
            source.get_launch_description(LaunchContext())
            if os.path.basename(source.location) == 'online_async_launch.py':
                removed += 1; continue
        if isinstance(action, DeclareLaunchArgument) and action.name in defaults:
            action = DeclareLaunchArgument(action.name, default_value=defaults[action.name])
        actions.append(action)
    assert removed == 1, 'Reference launch changed: refusing to start two map->odom authorities'
    assert gate_action is not None, 'Reference Safety Gate missing'

    path = os.environ.get('INT_MAP_YAML', os.path.join(FLEET, 'maps', 'limo_sim_map.yaml'))
    with open(os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')) as f:
        amcl = yaml.safe_load(f)['amcl']['ros__parameters']
    amcl.update(use_sim_time=True, base_frame_id='base_footprint', global_frame_id='map', odom_frame_id='odom', scan_topic='/scan',
                set_initial_pose=True, initial_pose={'x': mx, 'y': my, 'z': 0.0, 'yaw': myaw}, update_min_a=.05, update_min_d=.05, laser_max_range=10.0)
    actions += [
        Node(package='limo_capstone', executable='odom_tf_relay_node', name='odom_tf_relay_node',
             parameters=[{'use_sim_time': True, 'active_odom_topic': '/odom', 'nav2_odom_topic': '/odom_nav2', 'odom_frame': 'odom', 'base_frame': 'base_footprint'}]),
        Node(package='nav2_map_server', executable='map_server', name='map_server', parameters=[{'use_sim_time': True, 'yaml_filename': path}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[amcl]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='localization_manager',
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server', 'amcl']}]),
        # RGB image: separate bridge so the reference bridge node stays untouched
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge_rgb', output='screen',
             arguments=['/camera/color/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image'],
             parameters=[{'use_sim_time': True}]),
        TimerAction(period=_f('INT_GATE_DELAY_S', 20.0), actions=[gate_action]),
    ]
    return LaunchDescription(actions)
