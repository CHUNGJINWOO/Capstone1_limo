from launch_ros.actions import Node


def locked_gate(clock, depth):
    return Node(package='limo_capstone', executable='stall_detector_node',
                name='stall_detector_node', parameters=[clock, {
                    'enabled': True, 'enable_auto_reverse': False, 'enable_auto_rotate_recovery': False,
                    'require_operator_unlatch': True, 'depth_mode': 'required' if depth else 'disabled',
                    'estimator_odom_topic': '/odom_nav2',
                    # G2: /cmd_vel_raw input-source validation. Only the motion watchdog may write the Gate input, bound to the GID
                    # the watchdog itself reports; anything else (second writer, other name, restart, non-finite or out-of-range
                    # command) is zero output + fault until the operator resets the Gate. Limits are above the Arbiter/Nav2 maxima.
                    'input_exclusive_publishers': ['fleet_motion_watchdog'],
                    'input_writer_status_topic': '/fleet/motion_watchdog/status',
                    'input_max_abs_linear_x': 0.20, 'input_max_abs_angular_z': 0.60,
                    'exclusive_cmd_vel_topic': '/cmd_vel_locked',     # isolated output (the Gate is remapped); on a robot this is /cmd_vel
                }], remappings=[('/cmd_vel', '/cmd_vel_locked')])


def motion_watchdog(clock, speed_clamp_state=None, mode_gate=False):
    """G2 2026-11-23 (R2): the independent motion watchdog is the only writer of the Gate input /cmd_vel_raw.

    It forwards /fleet/nav_cmd_vel only while the Agent heartbeat, the Gate status and the Nav2 goal evidence authorize motion,
    publishes zero otherwise and never republishes a non-zero command after shutdown (R2 tests). Defaults are the v5 values."""
    extra = {}
    if speed_clamp_state:
        # G2 2026-11-23: the same Watchdog speed clamp the real launch enables (the Agent has no Arbiter in this profile: velocity:=false).
        extra = {'speed_clamp': True, 'speed_limit_state_path': speed_clamp_state,
                 'input_publisher_nodes': ['fleet_robot_agent', 'velocity_smoother', 'manual_mapping_teleop'], 'mode_gate': bool(mode_gate)}
    return Node(package='limo_robot_agent', executable='motion_watchdog', name='fleet_motion_watchdog',
                parameters=[clock, extra], output='screen')
