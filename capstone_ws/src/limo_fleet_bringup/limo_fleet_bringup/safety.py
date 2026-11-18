from launch_ros.actions import Node


def locked_gate(clock, depth):
    return Node(package='limo_capstone', executable='stall_detector_node',
                name='stall_detector_node', parameters=[clock, {
                    'enabled': True, 'enable_auto_reverse': False, 'enable_auto_rotate_recovery': False,
                    'require_operator_unlatch': True, 'depth_mode': 'required' if depth else 'disabled',
                    'estimator_odom_topic': '/odom_nav2',
                }], remappings=[('/cmd_vel', '/cmd_vel_locked')])
