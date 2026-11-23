"""Fleet development bringup for an isolated, driver-free ROS domain."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from limo_fleet_bringup import fleet_bridge, localization, mapping, navigation, perception, safety, teleoperation


def setup(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID', '').isdigit():
        raise RuntimeError('Set ROS_LOCALHOST_ONLY=1 and a dedicated ROS_DOMAIN_ID')
    if not Path(get('token_file')).is_file() or not get('state_dir'):
        raise RuntimeError('Provide the Agent token file and an isolated state_dir')
    flags = {key: get(key) == 'true' for key in (
        'use_sim_time', 'depth_filter', 'mapping', 'bag', 'manual', 'velocity', 'camera', 'fleet_goals')}
    if flags['fleet_goals'] and (not flags['mapping'] or any(flags[x] for x in ('bag', 'manual', 'velocity'))):
        raise RuntimeError('fleet_goals is the locked collision-test profile and requires mapping only')
    clock = {'use_sim_time': flags['use_sim_time']}
    parameters = {key: get(key) for key in ('robot_id', 'token_file', 'state_dir', 'server_url')}
    parameters.update(clock, camera_preview=flags['camera'], allow_missions=flags['fleet_goals'])
    parameters.update(mapping.worker_parameters(flags['mapping'], flags['bag']))
    parameters.update(teleoperation.input_parameters(flags['manual'], flags['velocity']))
    return [
        *([perception.depth_filter(clock)] if flags['depth_filter'] else []),
        *localization.odometry(clock),
        *navigation.navigation(clock, arbiter=flags['velocity']),
        safety.motion_watchdog(clock),          # G2: R2 watchdog between Nav2 / Arbiter and the Gate input
        safety.locked_gate(clock, flags['depth_filter']),
        fleet_bridge.agent(parameters),
    ]


def generate_launch_description():
    defaults = {
        'robot_id': 'locked_bench', 'server_url': 'http://127.0.0.1:8080',
        'use_sim_time': 'false', 'depth_filter': 'false', 'mapping': 'false',
        'bag': 'false', 'manual': 'false', 'velocity': 'false', 'camera': 'false', 'fleet_goals': 'false',
    }
    return LaunchDescription([
        DeclareLaunchArgument('token_file'), DeclareLaunchArgument('state_dir'),
        *(DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()),
        OpaqueFunction(function=setup),
    ])
