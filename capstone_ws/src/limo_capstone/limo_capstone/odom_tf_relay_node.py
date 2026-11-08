#!/usr/bin/env python3
"""
odom_tf_relay_node.py
Closed-Loop Odometry & TF Relay Node:
1. Subscribes to the selected active estimator topic (active_odom_topic, e.g.:
   - F4:          /odometry/lidar_odom
   - F5_improved: /odometry/f5_improved
2. Broadcasts strictly ONE odom -> base_footprint transform on /tf.
3. Republishes the active Odometry message to /odom_nav2 with frame_id='odom' and
   child_frame_id='base_footprint' so Nav2 (bt_navigator, controller_server, velocity_smoother)
   uses the exact selected estimator for both TF lookup and odometry twist/pose feedback!
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfRelayNode(Node):
    def __init__(self):
        super().__init__('odom_tf_relay_node')
        self.declare_parameter('active_odom_topic', '/odom')
        self.declare_parameter('nav2_odom_topic', '/odom_nav2')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.active_topic = self.get_parameter('active_odom_topic').value
        self.nav2_odom_topic = self.get_parameter('nav2_odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_stamp_ns = -1

        self.pub_nav2_odom = self.create_publisher(Odometry, self.nav2_odom_topic, 20)
        self.sub = self.create_subscription(
            Odometry,
            self.active_topic,
            self.on_odom,
            20
        )
        self.get_logger().info(
            f"OdomTfRelayNode closed-loop binding: [{self.active_topic}] -> "
            f"TF ({self.odom_frame}->{self.base_frame}) AND [{self.nav2_odom_topic}]"
        )

    def on_odom(self, msg: Odometry):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns <= self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp_ns

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub_nav2_odom.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
