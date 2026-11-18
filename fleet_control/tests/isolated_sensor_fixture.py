"""Publish deterministic scan, odom and camera data for the driver-free integration stack."""
import math
import os

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import StaticTransformBroadcaster


class Sensors(Node):
    def __init__(self):
        super().__init__('fleet_isolated_sensors')
        self.scan = self.create_publisher(LaserScan, '/scan', 10)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.rgb = self.create_publisher(Image, '/camera/color/image_raw', 1)
        self.depth = self.create_publisher(Image, '/camera/depth/image_raw', 1)
        self.tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_footprint'
        transform.child_frame_id = 'laser'
        transform.transform.rotation.w = 1.
        self.tf.sendTransform(transform)
        self.timer = self.create_timer(.1, self.publish)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.orientation.w = 1.
        self.odom.publish(odom)

        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'laser'
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = 2 * math.pi / 359
        scan.range_min, scan.range_max, scan.scan_time = .12, 10., .1
        scan.ranges = [2. + .30 * math.cos(i * .021) + .17 * math.cos(i * .057) for i in range(360)]
        self.scan.publish(scan)

        rgb, depth = Image(), Image()
        for frame in (rgb, depth):
            frame.header.stamp, frame.header.frame_id = stamp, 'laser'
            frame.width, frame.height = 32, 24
        rgb.encoding, rgb.step, rgb.data = 'rgb8', 96, bytes([20, 100, 230] * 32 * 24)
        depth.encoding, depth.step = '16UC1', 64
        depth.data = b''.join((600 + x * 112).to_bytes(2, 'little') for x in range(32)) * 24
        self.rgb.publish(rgb)
        self.depth.publish(depth)


if __name__ == '__main__':
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID', '').isdigit():
        raise SystemExit('Local-only ROS and a dedicated domain are required')
    rclpy.init()
    node = Sensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
