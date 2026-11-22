"""Isolated motor-locked stack: ICP odom + relay, Nav2 (slow LiDAR profile), locked Gate. No hardware, no /cmd_vel."""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
CFG = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', 'capstone_ws', 'src', 'limo_capstone', 'config') + os.sep
def generate_launch_description():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    # USE_SIM_TIME=1 only with the Gazebo backend (tests/gazebo_web, /clock from Ignition); the kinematic sim keeps wall time.
    clock = {'use_sim_time': os.environ.get('USE_SIM_TIME') == '1'}
    prof = CFG + 'integrated_nav2_lidar_only_slow_params.yaml'
    nodes = [('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'), ('nav2_smoother', 'smoother_server'),
             ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator'), ('nav2_velocity_smoother', 'velocity_smoother')]
    out = [Node(package='limo_capstone', executable='lidar_scan_matcher_odom_node', parameters=[clock]),
           Node(package='limo_capstone', executable='odom_tf_relay_node', parameters=[clock, {'active_odom_topic': os.environ.get('ODOM_ACTIVE_TOPIC') or '/odom', 'nav2_odom_topic': '/odom_nav2', 'odom_frame': 'odom', 'base_frame': 'base_footprint'}])]
    for pkg, name in nodes:
        remaps = [('cmd_vel', '/fleet/cmd_vel_nav')] if name in ('controller_server', 'behavior_server') else []
        params = [prof, clock]
        if os.environ.get('NAV2_PARAMS_OVERLAY'):    # 2026-11-22: candidate overrides (later file wins); the shared profile is unchanged
            params.insert(1, os.environ['NAV2_PARAMS_OVERLAY'])
        if name == 'velocity_smoother':
            # ARBITER=1: the Agent's VelocityArbiter is the single source for /fleet/nav_cmd_vel (MANUAL / MAPPING /
            # NAVIGATION by operating mode); Nav2 output goes to its input instead of straight to the Watchdog.
            out_topic = ('/fleet/cmd_vel_nav_smoothed' if os.environ.get('ARBITER') == '1'
                         else '/fleet/nav_cmd_vel' if os.environ.get('WATCHDOG', '1') == '1' else '/cmd_vel_raw')
            remaps = [('cmd_vel', '/fleet/cmd_vel_nav'), ('cmd_vel_smoothed', out_topic)]
        if name == 'bt_navigator':
            params.append({'default_nav_to_pose_bt_xml': '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'})
        out.append(Node(package=pkg, executable=name, name=name, parameters=params, remappings=remaps, output='screen'))
    manager = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
                   parameters=[clock, {'autostart': True, 'node_names': [n for _, n in nodes]}])
    # LIFECYCLE_DELAY_S: start the manager after the servers are discovered; an immediate start intermittently
    # lost a change_state response ("failed to send response ... timeout") and left Nav2 unconfigured.
    delay = float(os.environ.get('LIFECYCLE_DELAY_S', '0') or 0)
    out.append(TimerAction(period=delay, actions=[manager]) if delay > 0 else manager)
    gate_params = {'enabled': True, 'enable_auto_reverse': False, 'enable_auto_rotate_recovery': False, 'require_operator_unlatch': True,
                   'depth_mode': 'disabled', 'estimator_odom_topic': '/odom_nav2'}
    script = os.environ.get('GATE_SCRIPT')        # a proposed Gate revision (scratch copy); default = installed package
    if 'GATE_REQ_WD' in os.environ:                # needs a Gate that declares it (03b); older Gates ignore it
        gate_params['require_motion_watchdog'] = os.environ['GATE_REQ_WD'] == '1'
    if os.environ.get('GATE_INPUT_EXCL'):          # r11+: /cmd_vel_raw input-source validation (older Gates ignore unknown parameters)
        gate_params['input_exclusive_publishers'] = [x for x in os.environ['GATE_INPUT_EXCL'].split(',') if x]
        for k, e in (('input_max_abs_linear_x', 'GATE_INPUT_MAX_VX'), ('input_max_abs_angular_z', 'GATE_INPUT_MAX_WZ')):
            if os.environ.get(e): gate_params[k] = float(os.environ[e])
    if os.environ.get('GATE_RESET_PEER'):          # r12+: the input writer must share the DDS participant of this topic's writer
        gate_params['reset_input_peer_topic'] = os.environ['GATE_RESET_PEER']
    if os.environ.get('GATE_WRITER_STATUS'):       # r13 candidate: bind/keep the input writer only as its own status reports it
        gate_params['input_writer_status_topic'] = os.environ['GATE_WRITER_STATUS']
    if script:
        args = ['python3', script, '--ros-args', '-r', '__node:=stall_detector_node', '-r', '/cmd_vel:=/cmd_vel_locked']
        for k, v in {**clock, **gate_params}.items():
            args += ['-p', f'{k}:=' + ('[' + ','.join(v) + ']' if isinstance(v, list) else str(v).lower() if isinstance(v, bool) else str(v))]
        out.append(ExecuteProcess(cmd=args, output='screen'))
    else:
        out.append(Node(package='limo_capstone', executable='stall_detector_node', name='stall_detector_node', parameters=[clock, gate_params],
                        remappings=[('/cmd_vel', '/cmd_vel_locked')], output='screen'))
    return LaunchDescription(out)
