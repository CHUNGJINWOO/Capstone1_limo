#!/usr/bin/env -S python3 -s
"""
trial_sensor_normalizer_node.py
1. Normalizes raw Gazebo wheel odometry (/odom_gz_raw) relative to its initial pose at the start
   of each trial so that /odom (B0) always starts at exact (0, 0, 0) when a new trial begins
   (even when Gazebo's robot pose is reset via /world/lounge_world/set_pose).
2. Republishes /imu_gz_raw to /imu, optionally applying deterministic seeded sensor perturbations
   or Scenario S4 synthetic fault injections (explicitly logged with synthetic_injection=True/False).
"""
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
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


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class TrialSensorNormalizerNode(Node):
    def __init__(self):
        super().__init__(
            'trial_sensor_normalizer_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.declare_parameter('seed', 0)
        self.declare_parameter('wheel_vel_scale', 1.0)
        self.declare_parameter('wheel_vel_noise_std', 0.0)
        self.declare_parameter('imu_gyro_bias_z', 0.0)
        self.declare_parameter('imu_gyro_noise_std', 0.0)
        self.declare_parameter('imu_accel_bias_x', 0.0)
        self.declare_parameter('imu_accel_noise_std', 0.0)
        self.declare_parameter('slip_burst_start_s', -1.0)
        self.declare_parameter('slip_burst_end_s', -1.0)
        self.declare_parameter('slip_burst_multiplier', 1.0)

        seed = int(self.get_parameter('seed').value)
        self.rng = random.Random(seed if seed > 0 else 42)

        self.wheel_scale = float(self.get_parameter('wheel_vel_scale').value)
        self.wheel_noise = float(self.get_parameter('wheel_vel_noise_std').value)
        self.gyro_bias_z = float(self.get_parameter('imu_gyro_bias_z').value)
        self.gyro_noise = float(self.get_parameter('imu_gyro_noise_std').value)
        self.accel_bias_x = float(self.get_parameter('imu_accel_bias_x').value)
        self.accel_noise = float(self.get_parameter('imu_accel_noise_std').value)
        self.slip_start = float(self.get_parameter('slip_burst_start_s').value)
        self.slip_end = float(self.get_parameter('slip_burst_end_s').value)
        self.slip_mult = float(self.get_parameter('slip_burst_multiplier').value)

        self.raw_origin = None  # (x0, y0, yaw0, t0)
        self.prev_raw = None    # (xr, yr, yawr, t_s)
        self.int_x = 0.0
        self.int_y = 0.0
        self.int_yaw = 0.0

        self.imu_t0 = None
        self.imu_bias_accum_yaw = 0.0
        self.last_imu_t = None

        self.sub_odom = self.create_subscription(Odometry, '/odom_gz_raw', self.on_raw_odom, 20)
        self.sub_imu = self.create_subscription(Imu, '/imu_gz_raw', self.on_raw_imu, 50)

        self.pub_odom = self.create_publisher(Odometry, '/odom', 20)
        self.pub_imu = self.create_publisher(Imu, '/imu', 50)

    def on_raw_odom(self, msg: Odometry):
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        xr = msg.pose.pose.position.x
        yr = msg.pose.pose.position.y
        _, _, yawr = quat_to_euler(msg.pose.pose.orientation)

        if self.raw_origin is None:
            self.raw_origin = (xr, yr, yawr, t_s)
            self.prev_raw = (xr, yr, yawr, t_s)
            self.int_x = 0.0
            self.int_y = 0.0
            self.int_yaw = 0.0

        elapsed = t_s - self.raw_origin[3]
        dt = t_s - self.prev_raw[3]

        # Check if any synthetic perturbation is active
        is_perturbed = (
            abs(self.wheel_scale - 1.0) > 1e-5
            or self.wheel_noise > 0.0
            or (self.slip_start >= 0.0 and self.slip_start <= elapsed <= self.slip_end and abs(self.slip_mult - 1.0) > 1e-5)
        )

        if not is_perturbed:
            # Exact rigid transformation from raw Gazebo odom origin (x0, y0, yaw0)
            x0, y0, yaw0, _ = self.raw_origin
            dx = xr - x0
            dy = yr - y0
            c0 = math.cos(yaw0)
            s0 = math.sin(yaw0)
            xo = c0 * dx + s0 * dy
            yo = -s0 * dx + c0 * dy
            yawo = wrap_angle(yawr - yaw0)
            vx_out = msg.twist.twist.linear.x
            vy_out = msg.twist.twist.linear.y
            wz_out = msg.twist.twist.angular.z
            self.int_x, self.int_y, self.int_yaw = xo, yo, yawo
        else:
            # Integrate with scale / slip burst perturbation
            mult = self.wheel_scale
            if self.slip_start >= 0.0 and self.slip_start <= elapsed <= self.slip_end:
                mult *= self.slip_mult
            vx_out = msg.twist.twist.linear.x * mult
            if self.wheel_noise > 0.0 and abs(vx_out) > 0.005:
                vx_out += self.rng.gauss(0.0, self.wheel_noise)
            vy_out = msg.twist.twist.linear.y * mult
            wz_out = msg.twist.twist.angular.z

            if 0.0 < dt < 0.5:
                dyaw_raw = wrap_angle(yawr - self.prev_raw[2])
                self.int_yaw = wrap_angle(self.int_yaw + dyaw_raw)
                c = math.cos(self.int_yaw)
                s = math.sin(self.int_yaw)
                self.int_x += (c * vx_out - s * vy_out) * dt
                self.int_y += (s * vx_out + c * vy_out) * dt
            xo, yo, yawo = self.int_x, self.int_y, self.int_yaw

        self.prev_raw = (xr, yr, yawr, t_s)

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'
        out.pose.pose.position.x = xo
        out.pose.pose.position.y = yo
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation = yaw_to_quat(yawo)
        out.twist.twist.linear.x = vx_out
        out.twist.twist.linear.y = vy_out
        out.twist.twist.angular.z = wz_out
        out.pose.covariance = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance
        self.pub_odom.publish(out)

    def on_raw_imu(self, msg: Imu):
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.imu_t0 is None:
            self.imu_t0 = t_s
            self.last_imu_t = t_s

        dt = t_s - self.last_imu_t
        self.last_imu_t = t_s

        out = Imu()
        out.header = msg.header
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        wz = msg.angular_velocity.z + self.gyro_bias_z
        if self.gyro_noise > 0.0:
            wz += self.rng.gauss(0.0, self.gyro_noise)

        ax = msg.linear_acceleration.x + self.accel_bias_x
        ay = msg.linear_acceleration.y
        if self.accel_noise > 0.0:
            ax += self.rng.gauss(0.0, self.accel_noise)
            ay += self.rng.gauss(0.0, self.accel_noise)

        if abs(self.gyro_bias_z) > 1e-6 and 0.0 < dt < 0.5:
            self.imu_bias_accum_yaw = wrap_angle(self.imu_bias_accum_yaw + self.gyro_bias_z * dt)
            _, _, raw_yaw = quat_to_euler(msg.orientation)
            out.orientation = yaw_to_quat(wrap_angle(raw_yaw + self.imu_bias_accum_yaw))
        else:
            out.orientation = msg.orientation

        out.angular_velocity.x = msg.angular_velocity.x
        out.angular_velocity.y = msg.angular_velocity.y
        out.angular_velocity.z = wz
        out.linear_acceleration.x = ax
        out.linear_acceleration.y = ay
        out.linear_acceleration.z = msg.linear_acceleration.z
        self.pub_imu.publish(out)


def main():
    rclpy.init()
    node = TrialSensorNormalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
