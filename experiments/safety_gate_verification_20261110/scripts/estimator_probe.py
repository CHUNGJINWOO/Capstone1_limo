#!/usr/bin/env python3
"""Short straight-drive probe for the estimator comparison (sim only, through the safety gate: /cmd_vel_raw -> gate -> /cmd_vel; never resets the gate).
Samples at every /scan: GT, wheel /odom, F4 /odometry/lidar_odom, ACTIVE estimator /odom_nav2, TF map->base_footprint, gate status, F4 inliers.
Also saves the raw scans (npz) for offline ICP analysis.   usage: estimator_probe.py OUT_PREFIX [vx=0.2] [dist=1.8] [max_wall=16]"""
import sys, json, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32MultiArray
import tf2_ros

out = sys.argv[1]; vx = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2
dist = float(sys.argv[3]) if len(sys.argv) > 3 else 1.8; maxw = float(sys.argv[4]) if len(sys.argv) > 4 else 16.0
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
rclpy.init()
n = Node('estimator_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
st = {}; gate = {}; diag = {}; scans = []; rows = []; rec = {'on': False}
tfb = tf2_ros.Buffer(); tf2_ros.TransformListener(tfb, n)
def tf_pose():
    try:
        t = tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
        return (t.translation.x, t.translation.y, yaw(t.rotation))
    except Exception: return None
for k, tp in (('gt', '/ground_truth/world_odom'), ('lidar', '/odometry/lidar_odom'), ('wheel', '/odom'), ('active', '/odom_nav2')):
    n.create_subscription(Odometry, tp, lambda m, k=k: st.__setitem__(k, (m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))), 20)
def cb_g(m):
    try: gate.update(json.loads(m.data))
    except Exception: pass
n.create_subscription(String, '/stall_detector/status_json', cb_g, 10)
n.create_subscription(Float32MultiArray, '/diagnostics/f4_latency', lambda m: diag.update(inl=m.data[3], icp_ms=m.data[0], delay_ms=m.data[1], period_ms=m.data[2]), 20)
t0 = [None]
def cb_scan(m):
    if not rec['on'] or not all(k in st for k in ('gt', 'lidar', 'wheel', 'active')): return
    ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    scans.append((ts, np.asarray(m.ranges, dtype=np.float32), m.angle_min, m.angle_increment))
    rows.append(dict(t=round(time.monotonic() - t0[0], 3), scan_stamp=ts, gt=st['gt'], wheel=st['wheel'], lidar=st['lidar'], active=st['active'], map_base=tf_pose(),
                     gate=gate.get('state'), g_lidar_speed=gate.get('lidar_speed'), g_wheel_vx=gate.get('wheel_vx'), g_inl=gate.get('icp_inliers'), f4_inl=diag.get('inl'),
                     f4_icp_ms=diag.get('icp_ms'), f4_delay_ms=diag.get('delay_ms'), f4_period_ms=diag.get('period_ms')))
n.create_subscription(LaserScan, '/scan', cb_scan, qos_profile_sensor_data)
pub = n.create_publisher(Twist, '/cmd_vel_raw', 10)
def spin(s):
    t = time.monotonic()
    while time.monotonic() - t < s: rclpy.spin_once(n, timeout_sec=0.01)
t1 = time.monotonic()
while time.monotonic() - t1 < 60 and not (gate.get('state') == 'NORMAL' and all(k in st for k in ('gt', 'lidar', 'wheel', 'active')) and tf_pose()): spin(0.2)
print('ready gate=%s' % gate.get('state'))
if gate.get('state') != 'NORMAL': print('gate not NORMAL'); sys.exit(2)
spin(3.0)
g0 = st['gt']; tw = Twist(); tw.linear.x = vx; t0[0] = time.monotonic(); rec['on'] = True; last = 0; states = set()
while time.monotonic() - t0[0] < maxw:
    now = time.monotonic()
    if now - last > 0.05: pub.publish(tw); last = now
    states.add(gate.get('state'))
    g = st['gt']
    if math.hypot(g[0] - g0[0], g[1] - g0[1]) >= dist: break
    rclpy.spin_once(n, timeout_sec=0.005)
for _ in range(10): pub.publish(Twist()); spin(0.05)
spin(1.0); rec['on'] = False
json.dump(dict(rows=rows, vx=vx, gt0=g0), open(out + '.json', 'w'))
np.savez_compressed(out + '_scans.npz', stamps=np.array([s[0] for s in scans]), ranges=np.stack([s[1] for s in scans]), angle_min=scans[0][2], angle_inc=scans[0][3])
print(json.dumps(dict(n_rows=len(rows), gate_states=sorted(map(str, states)), gt_travel=round(math.hypot(st['gt'][0] - g0[0], st['gt'][1] - g0[1]), 3))))
