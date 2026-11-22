"""Simulation-only variation: retain the existing plant/sensors/Nav2/gate, replace SLAM with saved-map AMCL."""
import importlib.util
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    capstone = get_package_share_directory('limo_capstone')
    parent = os.path.join(capstone, 'launch', 'limo_integrated_auto_mapping_sim_diff.launch.py')
    spec = importlib.util.spec_from_file_location('reference_diff_sim', parent)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    original = module.generate_launch_description()
    defaults = {'spawn_x':'4.4','spawn_y':'-3.0','spawn_yaw':'1.570796326795','start_explore':'false'}
    actions = []
    removed = 0
    gate_action = None
    for action in original.entities:
        if isinstance(action, Node) and action.node_executable == 'stall_detector_node':
            gate_action = action; continue
        if isinstance(action, Node) and action.node_executable in ('odom_tf_relay_node','ekf_node'):
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
    path = os.path.join(os.path.dirname(__file__), '..', 'maps', 'limo_sim_map.yaml')
    with open(os.path.join(get_package_share_directory('nav2_bringup'),'params','nav2_params.yaml')) as f:
        amcl = yaml.safe_load(f)['amcl']['ros__parameters']
    amcl.update(use_sim_time=True, base_frame_id='base_footprint', global_frame_id='map', odom_frame_id='odom',
                scan_topic='/scan', set_initial_pose=True, initial_pose={'x':-2.4,'y':-5.5,'z':0.0,'yaw':0.0},
                update_min_a=.05, update_min_d=.05, laser_max_range=10.0)
    actions += [Node(package='limo_capstone',executable='odom_tf_relay_node',name='odom_tf_relay_node',
                     parameters=[{'use_sim_time':True,'active_odom_topic':'/odom','nav2_odom_topic':'/odom_nav2','odom_frame':'odom','base_frame':'base_footprint'}]),
                Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[{'use_sim_time':True,'yaml_filename':path}]),
                Node(package='nav2_amcl',executable='amcl',name='amcl',parameters=[amcl]),
                Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='localization_manager',
                     parameters=[{'use_sim_time':True,'autostart':True,'node_names':['map_server','amcl']}])]
    assert gate_action is not None, 'Reference Safety Gate missing'
    actions.append(TimerAction(period=20.0, actions=[gate_action]))
    return LaunchDescription(actions)
