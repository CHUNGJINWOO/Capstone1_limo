#!/usr/bin/env python3
"""Read-only DDS reception probe (runs on the robot; subscribes only, publishes nothing, no service calls).

usage: dds_probe.py LABEL [--duration S] [--topics /map,/scan,/odom,/tf,/tf_static] [--match-qos]
For every topic it records: seconds until a publisher is visible, seconds until the first message, message count, and the
publisher endpoints seen (node, reliability, durability). --match-qos subscribes with the publisher's own QoS (otherwise the
usual one: /map reliable+transient_local, /scan sensor_data, others reliable volatile). One JSON line per topic on stdout.
"""
import json, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

label = sys.argv[1]
dur = float(sys.argv[sys.argv.index('--duration') + 1]) if '--duration' in sys.argv else 12.0
topics = (sys.argv[sys.argv.index('--topics') + 1] if '--topics' in sys.argv else '/map,/scan,/odom,/tf,/tf_static').split(',')
match = '--match-qos' in sys.argv
TYPES = {'/map': 'nav_msgs/msg/OccupancyGrid', '/scan': 'sensor_msgs/msg/LaserScan', '/odom': 'nav_msgs/msg/Odometry',
         '/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage'}
rclpy.init()
t0 = time.monotonic()
n = Node('dds_probe_%d' % int(time.time() * 1000 % 100000))
st = {t: {'topic': t, 'label': label, 'count': 0, 't_pub_seen': None, 't_first_msg': None, 'pubs': []} for t in topics}


def qos_for(t):
    if match:
        infos = n.get_publishers_info_by_topic(t)
        if infos:
            q = infos[0].qos_profile
            return QoSProfile(depth=max(1, q.depth or 1), reliability=q.reliability, durability=q.durability, history=HistoryPolicy.KEEP_LAST)
    if t == '/map':
        return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    if t == '/scan':
        return qos_profile_sensor_data
    if t == '/tf_static':
        return QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


def cb(t):
    def f(_m):
        s = st[t]; s['count'] += 1
        if s['t_first_msg'] is None:
            s['t_first_msg'] = round(time.monotonic() - t0, 2)
    return f


subs = {}
end = t0 + dur
while time.monotonic() < end:
    for t in topics:
        s = st[t]
        if s['t_pub_seen'] is None:
            infos = n.get_publishers_info_by_topic(t)
            if infos:
                s['t_pub_seen'] = round(time.monotonic() - t0, 2)
                s['pubs'] = sorted({(i.node_name, i.qos_profile.reliability.name, i.qos_profile.durability.name) for i in infos})
        if t not in subs and (not match or s['t_pub_seen'] is not None):
            subs[t] = n.create_subscription(get_message(TYPES[t]), t, cb(t), qos_for(t))
    rclpy.spin_once(n, timeout_sec=0.1)
for t in topics:
    s = st[t]; s['pubs'] = [list(x) for x in s['pubs']]
    print(json.dumps(s), flush=True)
rclpy.shutdown()
