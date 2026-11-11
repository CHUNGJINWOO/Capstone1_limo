#!/usr/bin/env -S python3 -s
"""
ground_truth_publisher_node.py
Extracts the exact Gazebo Fortress ground-truth pose of model 'limo' from
/world/lounge_world/dynamic_pose/info (bridged to /gz/dynamic_pose_info as TFMessage).

STRICT ISOLATION RULE:
- Publishes ONLY to /ground_truth/odom (in 'odom' relative frame starting at 0,0,0)
  and /ground_truth/world_odom (in 'map' global Gazebo world frame).
- NEVER publishes to /tf or any topic consumed by state estimators or Nav2.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def quat_to_euler(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GroundTruthPublisherNode(Node):
    def __init__(self):
        super().__init__(
            'ground_truth_publisher_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.declare_parameter('robot_name', 'limo')
        self.robot_name = self.get_parameter('robot_name').value

        self.sub = self.create_subscription(
            TFMessage,
            '/gz/dynamic_pose_info',
            self.on_dynamic_pose,
            20
        )
        self.pub_odom = self.create_publisher(Odometry, '/ground_truth/odom', 20)
        self.pub_world = self.create_publisher(Odometry, '/ground_truth/world_odom', 20)

        self.init_pose = None  # (x0, y0, z0, yaw0)
        self.prev_state = None  # (t_sec, x_w, y_w, yaw_w)
        self.vx_filt = 0.0
        self.vy_filt = 0.0
        self.wz_filt = 0.0

        self.get_logger().info("GroundTruthPublisherNode started (strictly isolated from /tf and estimators).")

    def on_dynamic_pose(self, msg: TFMessage):
        target_tf = None
        for tf_stamped in msg.transforms:
            cid = tf_stamped.child_frame_id
            # Match root model name 'limo' (not wheel/hinge sub-links)
            if cid == self.robot_name or cid.endswith(f'/{self.robot_name}') or cid.endswith(f'::{self.robot_name}'):
                target_tf = tf_stamped
                break

        if target_tf is None:
            return

        t_sec = target_tf.header.stamp.sec + target_tf.header.stamp.nanosec * 1e-9
        if t_sec <= 0.0:
            now_msg = self.get_clock().now().to_msg()
            t_sec = now_msg.sec + now_msg.nanosec * 1e-9
            stamp = now_msg
        else:
            stamp = target_tf.header.stamp

        xw = target_tf.transform.translation.x
        yw = target_tf.transform.translation.y
        zw = target_tf.transform.translation.z
        _, _, yaww = quat_to_euler(target_tf.transform.rotation)

        if self.init_pose is None:
            self.init_pose = (xw, yw, zw, yaww)
            self.prev_state = (t_sec, xw, yw, yaww)
            self.get_logger().info(
                f"Locked initial GT world origin: x0={xw:.4f}, y0={yw:.4f}, yaw0={yaww:.4f} rad"
            )

        # Compute body-frame velocities via finite difference
        dt = t_sec - self.prev_state[0]
        if dt >= 0.005:
            dx_w = xw - self.prev_state[1]
            dy_w = yw - self.prev_state[2]
            dyaw = wrap_angle(yaww - self.prev_state[3])

            vx_raw = (math.cos(yaww) * dx_w + math.sin(yaww) * dy_w) / dt
            vy_raw = (-math.sin(yaww) * dx_w + math.cos(yaww) * dy_w) / dt
            wz_raw = dyaw / dt

            alpha_v = 0.4
            self.vx_filt = alpha_v * vx_raw + (1.0 - alpha_v) * self.vx_filt
            self.vy_filt = alpha_v * vy_raw + (1.0 - alpha_v) * self.vy_filt
            self.wz_filt = alpha_v * wz_raw + (1.0 - alpha_v) * self.wz_filt
            self.prev_state = (t_sec, xw, yw, yaww)

        # 1. Publish World-frame Ground Truth (/ground_truth/world_odom)
        world_msg = Odometry()
        world_msg.header.stamp = stamp
        world_msg.header.frame_id = 'map'
        world_msg.child_frame_id = 'base_footprint_gt'
        world_msg.pose.pose.position.x = xw
        world_msg.pose.pose.position.y = yw
        world_msg.pose.pose.position.z = zw
        world_msg.pose.pose.orientation = target_tf.transform.rotation
        world_msg.twist.twist.linear.x = self.vx_filt
        world_msg.twist.twist.linear.y = self.vy_filt
        world_msg.twist.twist.angular.z = self.wz_filt
        self.pub_world.publish(world_msg)

        # 2. Publish Odom-frame Ground Truth (/ground_truth/odom, starting at 0,0,0 at t0)
        x0, y0, z0, yaw0 = self.init_pose
        dx = xw - x0
        dy = yw - y0
        c0 = math.cos(yaw0)
        s0 = math.sin(yaw0)
        x_odom = c0 * dx + s0 * dy
        y_odom = -s0 * dx + c0 * dy
        yaw_odom = wrap_angle(yaww - yaw0)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint_gt'
        odom_msg.pose.pose.position.x = x_odom
        odom_msg.pose.pose.position.y = y_odom
        odom_msg.pose.pose.position.z = zw - z0
        odom_msg.pose.pose.orientation = yaw_to_quat(yaw_odom)
        odom_msg.twist.twist.linear.x = self.vx_filt
        odom_msg.twist.twist.linear.y = self.vy_filt
        odom_msg.twist.twist.angular.z = self.wz_filt
        self.pub_odom.publish(odom_msg)


def main():
    rclpy.init()
    node = GroundTruthPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
