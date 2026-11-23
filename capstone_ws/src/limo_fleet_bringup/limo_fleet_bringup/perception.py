from launch_ros.actions import Node


def depth_filter(clock):
    return Node(package='limo_depth_obstacles', executable='depth_obstacle_filter_node',
                name='depth_obstacle_filter', parameters=[clock, {
                    'height_frame': 'base_footprint', 'input_topic': '/camera/depth/points',
                    'point_stride': 4, 'processing_hz': 4., 'data_timeout_sec': .75,
                }])
