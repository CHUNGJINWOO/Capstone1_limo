#!/usr/bin/env python3
"""Republish /scan as /scan_slam with a fixed number of beams (SLAM input only; Nav2, F4 and the gate keep using /scan)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from limo_scan_normalizer.core import normalize


class ScanNormalizerNode(Node):
    def __init__(self):
        super().__init__('scan_normalizer_node')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_slam')
        self.declare_parameter('n_out', 244)
        self.declare_parameter('angle_min_out', -1.9198621771937625)
        self.declare_parameter('angle_max_out', 1.9198621771937625)
        self.n_out = int(self.get_parameter('n_out').value)
        self.a_min = float(self.get_parameter('angle_min_out').value)
        self.a_max = float(self.get_parameter('angle_max_out').value)
        self.pub = self.create_publisher(LaserScan, self.get_parameter('output_topic').value, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.get_parameter('input_topic').value, self.cb, qos_profile_sensor_data)
        self.count = 0
        self.get_logger().info(f'normalizing to {self.n_out} beams over [{self.a_min:.4f}, {self.a_max:.4f}] rad')

    def cb(self, m: LaserScan):
        r = normalize(m.angle_min, m.angle_increment, list(m.ranges), list(m.intensities), m.scan_time,
                      self.n_out, self.a_min, self.a_max)
        o = LaserScan()
        o.header = m.header
        o.angle_min = r['angle_min']; o.angle_max = r['angle_max']; o.angle_increment = r['angle_increment']
        o.time_increment = r['time_increment']; o.scan_time = m.scan_time
        o.range_min = m.range_min; o.range_max = m.range_max
        o.ranges = r['ranges']; o.intensities = r['intensities']
        self.pub.publish(o)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    n = ScanNormalizerNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
