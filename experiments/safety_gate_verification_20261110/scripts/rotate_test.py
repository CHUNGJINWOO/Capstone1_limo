#!/usr/bin/env python3
"""Publish angular.z on a topic for DUR wall-seconds and report ground-truth yaw change.
usage: rotate_test.py <topic> <wz> <dur>"""
import sys, time, math, json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

topic, wz, dur = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class T(Node):
    def __init__(self):
        super().__init__('rotate_probe')
        self.gt = None
        self.gz_wz = None
        self.state = None
        self.cmd_out = []
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'gt', m), 10)
        self.create_subscription(Odometry, '/odom_gz_raw', lambda m: setattr(self, 'gz_wz', m.twist.twist.angular.z), 10)
        self.create_subscription(String, '/stall_detector/status_json', self.cb_s, 10)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.cmd_out.append(m.angular.z), 50)
        self.pub = self.create_publisher(Twist, topic, 10)

    def cb_s(self, m):
        try:
            self.state = json.loads(m.data).get('state')
        except Exception:
            pass


def spin_for(n, s):
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        rclpy.spin_once(n, timeout_sec=0.01)


rclpy.init()
n = T()
spin_for(n, 2.0)
if n.gt is None:
    print('no /ground_truth/odom'); sys.exit(1)
y0 = yaw(n.gt.pose.pose.orientation); st0 = n.state
n.cmd_out.clear()
tw = Twist(); tw.angular.z = wz
gz_samples = []
t0 = time.monotonic(); last = 0
while time.monotonic() - t0 < dur:
    if time.monotonic() - last > 0.05:
        n.pub.publish(tw); last = time.monotonic()
        if n.gz_wz is not None:
            gz_samples.append(n.gz_wz)
    rclpy.spin_once(n, timeout_sec=0.005)
st_during = n.state
cmd_seen = list(n.cmd_out)
n.pub.publish(Twist())
spin_for(n, 1.5)
y1 = yaw(n.gt.pose.pose.orientation)
d = math.degrees(math.atan2(math.sin(y1 - y0), math.cos(y1 - y0)))
nz = sum(abs(v) > 0.05 for v in cmd_seen)
print(f'topic={topic} wz={wz} dur={dur}s gate_before={st0} gate_during={st_during}')
print(f'GT yaw {math.degrees(y0):.2f} -> {math.degrees(y1):.2f} deg (delta {d:.2f} deg)')
print(f'/cmd_vel msgs during={len(cmd_seen)} nonzero={nz} max_wz={max(cmd_seen, default=0):.3f}')
if gz_samples:
    print(f'/odom_gz_raw angular.z mean={sum(gz_samples)/len(gz_samples):.3f} (wheel-derived, not proof)')
