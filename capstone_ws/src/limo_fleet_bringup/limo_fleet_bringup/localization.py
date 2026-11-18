from launch_ros.actions import Node


def odometry(clock):
    return [
        Node(package='limo_capstone', executable='lidar_scan_matcher_odom_node', parameters=[clock]),
        Node(package='limo_capstone', executable='odom_tf_relay_node', parameters=[clock, {
            'active_odom_topic': '/odom', 'nav2_odom_topic': '/odom_nav2',
            'odom_frame': 'odom', 'base_frame': 'base_footprint',
        }]),
    ]
