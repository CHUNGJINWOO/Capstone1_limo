from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def navigation(clock, arbiter=False):
    # G2 2026-11-23 (R2 chain): the smoother no longer writes the Gate input directly. Its output goes to the independent
    # motion watchdog (/fleet/nav_cmd_vel), or, with the Agent's VelocityArbiter, to the Arbiter input which feeds the watchdog.
    # origin/main wired velocity_smoother -> /cmd_vel_raw, which bypassed the watchdog.
    config = Path(get_package_share_directory('limo_capstone')) / 'config'
    profile = str(config / 'integrated_nav2_lidar_only_slow_params.yaml')
    nodes = [
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_planner', 'planner_server', 'planner_server'),
        ('nav2_smoother', 'smoother_server', 'smoother_server'),
        ('nav2_behaviors', 'behavior_server', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother'),
    ]
    result = []
    for package, executable, name in nodes:
        params = [profile, clock]
        remaps = []
        if name in ('controller_server', 'behavior_server'):
            remaps = [('cmd_vel', '/fleet/cmd_vel_nav')]
        if name == 'bt_navigator':
            params.append({'default_nav_to_pose_bt_xml': str(config / 'navigate_to_pose_ackermann_no_reverse.xml')})
        if name == 'velocity_smoother':
            remaps = [('cmd_vel', '/fleet/cmd_vel_nav'),
                      ('cmd_vel_smoothed', '/fleet/cmd_vel_nav_smoothed' if arbiter else '/fleet/nav_cmd_vel')]
            params.append({'max_velocity': [.10, 0., .30], 'min_velocity': [-.10, 0., -.30],
                           'velocity_timeout': .25})
        result.append(Node(package=package, executable=executable, name=name,
                           parameters=params, remappings=remaps, output='screen'))
    result.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                       name='lifecycle_manager_navigation', parameters=[clock, {
                           'autostart': True, 'node_names': [name for _, _, name in nodes],
                       }]))
    return result
