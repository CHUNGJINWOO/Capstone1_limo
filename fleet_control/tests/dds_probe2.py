#!/usr/bin/env python3
"""Read-only DDS layer probe (robot or lab). Separates discovery / endpoint matching / data, in both directions.

usage: dds_probe2.py LABEL [--duration S] [--real /scan,/tf,/odom,/map] [--ping-pub NAME] [--ping-sub NAME]
 - --real: subscribe to real topics with the publisher's own QoS. Reports per topic: publishers discovered, matched publisher count
   (rmw/rcl, i.e. endpoint matching done) and messages received. Subscribes only.
 - --ping-pub/--ping-sub: private topics /dds_diag/<NAME> between two probes started by the same test (outbound direction of a new participant).
JSON on stdout. Also prints participant GUID prefixes (first 12 bytes of endpoint GIDs) of discovered publishers and of this node's own endpoint.
"""
import json, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

A = sys.argv
label = A[1]
opt = lambda k, d=None: A[A.index(k) + 1] if k in A else d
dur = float(opt('--duration', 8))
real = [t for t in opt('--real', '').split(',') if t]
TYPES = {'/map': 'nav_msgs/msg/OccupancyGrid', '/scan': 'sensor_msgs/msg/LaserScan', '/odom': 'nav_msgs/msg/Odometry', '/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage'}
rclpy.init(); t0 = time.monotonic(); n = Node('dds_probe2_%d' % int(time.time() * 1000 % 100000))
gid = lambda i: bytes(i.endpoint_gid).hex()
out = {'label': label, 'topics': {}, 'ping': {}}
st = {t: {'count': 0, 'pubs_discovered': 0, 'matched_max': 0, 't_first_pub': None, 't_first_msg': None, 'pub_prefixes': []} for t in real}
subs = {}


def mk(t):
    def f(_):
        s = st[t]; s['count'] += 1
        if s['t_first_msg'] is None: s['t_first_msg'] = round(time.monotonic() - t0, 2)
    return f


ping_n = [0]; ping_pub = ping_sub = None
if opt('--ping-pub'):
    ping_pub = n.create_publisher(String, '/dds_diag/' + opt('--ping-pub'), QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
if opt('--ping-sub'):
    ping_sub = n.create_subscription(String, '/dds_diag/' + opt('--ping-sub'), lambda m: ping_n.__setitem__(0, ping_n[0] + 1), QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
ping_match = 0
own_prefix = None
end = t0 + dur
while time.monotonic() < end:
    for t in real:
        s = st[t]
        infos = n.get_publishers_info_by_topic(t)
        if infos and s['t_first_pub'] is None:
            s['t_first_pub'] = round(time.monotonic() - t0, 2)
        s['pubs_discovered'] = max(s['pubs_discovered'], len(infos))
        s['pub_prefixes'] = sorted({gid(i)[:24] for i in infos})
        if t not in subs and infos:
            q = infos[0].qos_profile
            subs[t] = n.create_subscription(get_message(TYPES[t]), t, mk(t), QoSProfile(depth=max(1, q.depth or 1), reliability=q.reliability, durability=q.durability, history=HistoryPolicy.KEEP_LAST))
        if t in subs:
            s['matched_max'] = max(s['matched_max'], n.count_publishers(t))   # humble rclpy has no sub-side matched count: this is graph-discovered publishers
            if own_prefix is None:
                mine = [i for i in n.get_subscriptions_info_by_topic(t) if i.node_name == n.get_name()]
                if mine: own_prefix = gid(mine[0])[:24]
    if ping_pub: ping_pub.publish(String(data='p'))
    if ping_sub: ping_match = max(ping_match, n.count_publishers('/dds_diag/' + opt('--ping-sub')))
    rclpy.spin_once(n, timeout_sec=0.1)
out['topics'] = st; out['own_participant_prefix'] = own_prefix
if ping_sub or ping_pub:
    out['ping'] = {'received': ping_n[0], 'publishers_discovered_by_sub': ping_match,
                   'my_pub_matched_subscribers(rmw matching)': ping_pub.get_subscription_count() if ping_pub else None}
print(json.dumps(out), flush=True)
rclpy.shutdown()
