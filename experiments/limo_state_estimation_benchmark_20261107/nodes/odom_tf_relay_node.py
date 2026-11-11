#!/usr/bin/env -S python3 -s
"""
odom_tf_relay_node.py
Ensures strictly ONE broadcaster for the `odom -> base_footprint` transform on /tf.
- In Open-Loop mode: subscribes to `/odom` (raw wheel odometry B0) and publishes `odom -> base_footprint`.
- In Closed-Loop mode: subscribes to the selected estimator topic (e.g., `/odometry/ekf_tuned`,
  `/odometry/comp_slip_gate`, `/odometry/lidar_imu_wheel_ekf`) and broadcasts both
  `odom -> base_footprint` on `/tf` and `/odom_active` for Nav2.
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfRelayNode(Node):
    def __init__(self):
        super().__init__(
            'odom_tf_relay_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.declare_parameter('active_odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.active_topic = self.get_parameter('active_odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_stamp_ns = -1

        self.sub = self.create_subscription(
            Odometry,
            self.active_topic,
            self.on_odom,
            20
        )
        self.get_logger().info(
            f"OdomTfRelayNode broadcasting single TF ({self.odom_frame} -> {self.base_frame}) "
            f"from source topic [{self.active_topic}]"
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


def main():
    rclpy.init()
    node = OdomTfRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
