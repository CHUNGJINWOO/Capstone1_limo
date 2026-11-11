#!/usr/bin/env python3
"""
glass_sensor_filter.py:
ROS 2 node that filters raw /scan into /scan to emulate Condition 4:
Specular / non-returning laser beams hitting clean glass at oblique angles produce inf / invalid readings.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class GlassSensorFilter(Node):
    def __init__(self):
        super().__init__('glass_sensor_filter')
        self.declare_parameter('invalid_angle_min', -0.5)  # radians
        self.declare_parameter('invalid_angle_max', 0.5)   # radians
        self.declare_parameter('invalid_ratio', 0.6)       # 60% of rays in glass sector return inf

        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.get_logger().info('Glass Sensor Filter active (emulating specular no-return inf beams)')

    def on_scan(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.intensities = list(msg.intensities)

        new_ranges = list(msg.ranges)
        a_min = self.get_parameter('invalid_angle_min').value
        a_max = self.get_parameter('invalid_angle_max').value
        ratio = self.get_parameter('invalid_ratio').value

        for i, r in enumerate(new_ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if a_min <= angle <= a_max:
                # Oblique / specular rays return inf
                if (i % 3 != 0) and ratio > 0.5:
                    new_ranges[i] = float('inf')

        out.ranges = new_ranges
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GlassSensorFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
