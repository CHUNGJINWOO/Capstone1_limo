"""Minimal 2D kinematic robot for isolated mode/mapping E2E (test-only, never on a robot).

Integrates the Safety Gate's locked output (/cmd_vel_locked) as a unicycle, publishes wheel /odom (pose + twist),
ray-casts a 360-beam /scan in a fixed world (6 x 5 m room, east door, 4 m corridor, one box), a static
base_footprint->laser TF and /limo_status. No physics, slip, noise or motor model: it shows that commands reaching
the locked Gate output move the simulated robot and that SLAM / ICP / Nav2 react; it is not a vehicle model.
Ground truth is published on /e2e/ground_truth (x, y, yaw) for the harness.
"""
import math, os, time
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from tf2_ros import StaticTransformBroadcaster
from limo_msgs.msg import LimoStatus

W = [((-3, -2.5), (3, -2.5)), ((3, -2.5), (3, -.55)), ((3, .55), (3, 2.5)), ((3, 2.5), (-3, 2.5)), ((-3, 2.5), (-3, -2.5)),
     ((3, -.55), (7, -.55)), ((3, .55), (7, .55)), ((7, -.55), (7, .55)),
     ((-1.6, .8), (-.9, .8)), ((-.9, .8), (-.9, 1.5)), ((-.9, 1.5), (-1.6, 1.5)), ((-1.6, 1.5), (-1.6, .8))]


def ray(px, py, a, rmax=8.):
    dx, dy = math.cos(a), math.sin(a); best = float('inf')
    for (x1, y1), (x2, y2) in W:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-12: continue
        t = ((x1 - px) * ey - (y1 - py) * ex) / den
        u = ((x1 - px) * dy - (y1 - py) * dx) / den
        if t > 0 and 0 <= u <= 1: best = min(best, t)
    return best if best <= rmax else float('inf')


import numpy as np
_SEG = np.array([[x1, y1, x2, y2] for (x1, y1), (x2, y2) in W], dtype=float)


def rays(px, py, angles, rmax=8.):
    """Vectorised ray cast (all beams x all segments) so scan publication never stalls the 10 Hz loop."""
    dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
    x1, y1, ex, ey = _SEG[:, 0], _SEG[:, 1], _SEG[:, 2] - _SEG[:, 0], _SEG[:, 3] - _SEG[:, 1]
    den = dx * ey - dy * ex
    with np.errstate(divide='ignore', invalid='ignore'):
        t = ((x1 - px) * ey - (y1 - py) * ex) / den
        u = ((x1 - px) * dy - (y1 - py) * dx) / den
    t = np.where((np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1), t, np.inf).min(axis=1)
    return np.where(t <= rmax, t, np.inf)


class Sim(Node):
    def __init__(self):
        super().__init__('e2e_kinematic_sim')
        self.x, self.y, self.th = 0., 0., 0.
        self.v = self.w = 0.; self.cmd_t = 0.; self.last = time.monotonic()
        self.create_subscription(Twist, '/cmd_vel_locked', self.on_cmd, 10)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', 10)
        self.status = self.create_publisher(LimoStatus, '/limo_status', 10)
        self.truth = self.create_publisher(Float64MultiArray, '/e2e/ground_truth', 10)
        self.tf = StaticTransformBroadcaster(self)
        t = TransformStamped(); t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'; t.child_frame_id = 'laser'; t.transform.rotation.w = 1.
        self.tf.sendTransform(t)
        self.create_timer(.05, self.step)
        self.create_timer(.1, self.publish_scan)

    def on_cmd(self, m):
        self.v, self.w, self.cmd_t = m.linear.x, m.angular.z, time.monotonic()

    def step(self):
        now = time.monotonic(); dt = min(.1, now - self.last); self.last = now
        if now - self.cmd_t > .3:                        # no Gate output -> the simulated robot stops
            self.v = self.w = 0.
        self.th += self.w * dt
        nx, ny = self.x + self.v * math.cos(self.th) * dt, self.y + self.v * math.sin(self.th) * dt
        if min(ray(self.x, self.y, math.atan2(ny - self.y, nx - self.x)), 9) > .2 or (nx, ny) == (self.x, self.y):
            self.x, self.y = nx, ny                     # crude collision stop 0.2 m before a wall (heading direction)
        stamp = self.get_clock().now().to_msg()
        o = Odometry(); o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.odom.publish(o)
        self.truth.publish(Float64MultiArray(data=[self.x, self.y, self.th, self.v, self.w]))

    def publish_scan(self):
        stamp = self.get_clock().now().to_msg()
        s = LaserScan(); s.header.stamp, s.header.frame_id = stamp, 'laser'
        s.angle_min, s.angle_max = -math.pi, math.pi; s.angle_increment = 2 * math.pi / 359
        s.range_min, s.range_max, s.scan_time = .12, 8., .1
        s.ranges = rays(self.x, self.y, self.th - math.pi + np.arange(360) * s.angle_increment).astype(float).tolist()
        self.scan.publish(s)
        st = LimoStatus(); st.header.stamp = stamp; st.battery_voltage = 11.6; self.status.publish(st)


if __name__ == '__main__':
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0', '77')
    rclpy.init(); n = Sim()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
