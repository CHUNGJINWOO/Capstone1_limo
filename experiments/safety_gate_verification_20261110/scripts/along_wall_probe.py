#!/usr/bin/env python3
"""Short repro probe (sim only). Drives straight ahead through the safety gate (/cmd_vel_raw -> gate -> /cmd_vel) at VX for up to
MAXS wall-seconds or DIST metres of ground-truth travel, recording at the callback rate: ground truth, /odometry/lidar_odom (F4 ICP),
wheel /odom, /odom_nav2 and TF map->base_footprint. Never resets the gate. Output: JSON of 10 Hz samples + summary of travel along the heading.
usage: along_wall_probe.py OUT.json [vx=0.2] [dist=1.8] [max_wall=14]"""
import sys, json, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros

out = sys.argv[1]; vx = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2
dist = float(sys.argv[3]) if len(sys.argv) > 3 else 1.8; maxw = float(sys.argv[4]) if len(sys.argv) > 4 else 14.0
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
rclpy.init()
n = Node('along_wall_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
st = {}; gate = {'s': None}
for k, tp in (('gt', '/ground_truth/world_odom'), ('lidar', '/odometry/lidar_odom'), ('wheel', '/odom'), ('nav2', '/odom_nav2')):
    n.create_subscription(Odometry, tp, lambda m, k=k: st.__setitem__(k, (m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))), 20)
def cb_g(m):
    try: gate['s'] = json.loads(m.data).get('state')
    except Exception: pass
n.create_subscription(String, '/stall_detector/status_json', cb_g, 10)
pub = n.create_publisher(Twist, '/cmd_vel_raw', 10)
tfb = tf2_ros.Buffer(); tf2_ros.TransformListener(tfb, n)
def spin(s):
    t = time.monotonic()
    while time.monotonic() - t < s: rclpy.spin_once(n, timeout_sec=0.01)
def tf_pose(a, b):
    try:
        t = tfb.lookup_transform(a, b, rclpy.time.Time()).transform
        return (t.translation.x, t.translation.y, yaw(t.rotation))
    except Exception: return None
t0 = time.monotonic()
while time.monotonic() - t0 < 60 and not (gate['s'] == 'NORMAL' and all(k in st for k in ('gt', 'lidar', 'wheel', 'nav2')) and tf_pose('map', 'base_footprint')):
    spin(0.2)
print('ready: gate=%s have=%s' % (gate['s'], sorted(st)))
if gate['s'] != 'NORMAL': print('gate not NORMAL, abort'); sys.exit(2)
spin(3.0)   # let SLAM take the first scans
samples = []; tw = Twist(); tw.linear.x = vx
base = dict(st); gate_log = set()
def snap(phase):
    samples.append(dict(t=round(time.monotonic() - tstart, 2), phase=phase, gate=gate['s'], gt=st.get('gt'), lidar=st.get('lidar'), wheel=st.get('wheel'), nav2=st.get('nav2'),
                        map_base=tf_pose('map', 'base_footprint'), map_odom=tf_pose('map', 'odom')))
tstart = time.monotonic(); last_p = 0; last_s = 0
g0 = st['gt']; l0 = st['lidar']; w0 = st['wheel']; m0 = tf_pose('map', 'base_footprint')
while time.monotonic() - tstart < maxw:
    now = time.monotonic()
    if now - last_p > 0.05: pub.publish(tw); last_p = now
    if now - last_s > 0.1: snap('drive'); last_s = now; gate_log.add(gate['s'])
    g = st['gt']
    if math.hypot(g[0] - g0[0], g[1] - g0[1]) >= dist: break
    rclpy.spin_once(n, timeout_sec=0.005)
for _ in range(10): pub.publish(Twist()); spin(0.05)
for i in range(30): spin(0.1); snap('settle') if i % 1 == 0 else None
def travel(a, b): return math.hypot(b[0] - a[0], b[1] - a[1])
g1 = st['gt']; l1 = st['lidar']; w1 = st['wheel']; m1 = tf_pose('map', 'base_footprint')
summ = dict(gate_states_during=sorted(map(str, gate_log)), gt_travel_m=round(travel(g0, g1), 3), lidar_odom_travel_m=round(travel(l0, l1), 3), wheel_odom_travel_m=round(travel(w0, w1), 3),
            slam_map_base_travel_m=round(travel(m0, m1), 3) if m0 and m1 else None, gt_yaw_change_deg=round(math.degrees(math.atan2(math.sin(g1[2] - g0[2]), math.cos(g1[2] - g0[2]))), 2),
            gt_start=g0, gt_end=g1, drive_samples=sum(s['phase'] == 'drive' for s in samples))
print(json.dumps(summ)); json.dump(dict(summary=summ, samples=samples), open(out, 'w'))
