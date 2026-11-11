#!/usr/bin/env python3
"""Replay a recorded LIMO bag into an external slam_toolbox, with the /scan variant chosen by --mode:
 raw   : scans exactly as recorded
 norm  : scans through limo_scan_normalizer.core.normalize (fixed 244 beams)
 n244  : only the scans that already have exactly 244 readings, untouched (reference without any resampling)
TF: only odom->base_footprint is replayed from /tf (the recorded map->odom came from the old live SLAM and is dropped); /tf_static as recorded."""
import os, sys, time, json, argparse, collections, numpy as np
sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'variants', 'scan_normalizer_ws', 'src', 'limo_scan_normalizer')))
import rclpy, rosbag2_py
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Time
from limo_scan_normalizer.core import normalize
ap = argparse.ArgumentParser(); ap.add_argument('bag'); ap.add_argument('--mode', default='raw'); ap.add_argument('--rate', type=float, default=1.0)
ap.add_argument('--out', required=True); ap.add_argument('--max_s', type=float, default=1e9); ap.add_argument('--startup', default=''); a = ap.parse_args()
startup = json.load(open(a.startup))[:3] if a.startup else []   # real first scans after a driver start (233/236/241 readings)
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
events = []; static = []
while r.has_next():
    topic, data, t = r.read_next()
    if topic in ('/scan', '/tf'): events.append((t, topic, data))
    elif topic == '/tf_static': static.append(data)
t0 = events[0][0]
rclpy.init(); n = Node('replay_node')
qs = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
pc = n.create_publisher(Clock, '/clock', 10); ps = n.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
pt = n.create_publisher(TFMessage, '/tf', 100); pst = n.create_publisher(TFMessage, '/tf_static', qs)
maps = []; marks = collections.defaultdict(int); latest = {}
def on_map(m):
    v = np.array(m.data, dtype=np.int8); latest['map'] = (m.info, v); maps.append((n.get_clock().now().nanoseconds * 1e-9, m.info.width, m.info.height, int((v != -1).sum()), int((v >= 65).sum())))
def on_marks(m):
    for k in m.markers: marks[(k.ns, k.type)] = max(marks[(k.ns, k.type)], len(k.points) or 1)
n.create_subscription(OccupancyGrid, '/map', on_map, qs); n.create_subscription(MarkerArray, '/slam_toolbox/graph_visualization', on_marks, 10)
for d in static: pst.publish(deserialize_message(d, TFMessage))
wall0 = time.time(); sent = collections.Counter(); skipped = 0; lens = collections.Counter()
def spin_short(): rclpy.spin_once(n, timeout_sec=0.0)
for (t, topic, data) in events:
    if (t - t0) / 1e9 > a.max_s: break
    target = wall0 + (t - t0) / 1e9 / a.rate
    while time.time() < target: spin_short(); time.sleep(0.002)
    c = Clock(); c.clock = Time(sec=int(t // 10**9), nanosec=int(t % 10**9)); pc.publish(c)
    if topic == '/tf':
        m = deserialize_message(data, TFMessage); keep = [x for x in m.transforms if x.header.frame_id == 'odom' and x.child_frame_id == 'base_footprint']
        if keep: pt.publish(TFMessage(transforms=keep)); sent['tf'] += 1
    else:
        m = deserialize_message(data, get_message(types['/scan']))
        if sent['scan'] + skipped < len(startup):          # replace the first recorded scans by REAL startup scans (header/stamp kept)
            sdump = startup[sent['scan'] + skipped]; nn = len(sdump['ranges'])
            m.angle_min = sdump['amin']; m.angle_increment = sdump['inc']; m.angle_max = sdump['amin'] + (nn - 1) * sdump['inc']
            m.ranges = sdump['ranges']; m.intensities = sdump['inten']; m.scan_time = 0.1
        lens[len(m.ranges)] += 1
        if a.mode == 'n244' and len(m.ranges) != 244: skipped += 1; continue
        if a.mode in ('norm', 'norm263'):
            o = normalize(m.angle_min, m.angle_increment, list(m.ranges), list(m.intensities), m.scan_time, n_out=(263 if a.mode == 'norm263' else 244))
            m.angle_min = o['angle_min']; m.angle_max = o['angle_max']; m.angle_increment = o['angle_increment']; m.time_increment = o['time_increment']
            m.ranges = o['ranges']; m.intensities = o['intensities']
        ps.publish(m); sent['scan'] += 1
end = time.time() + 6.0
while time.time() < end: spin_short(); time.sleep(0.01)
res = dict(mode=a.mode, scans_sent=sent['scan'], scans_skipped=skipped, tf_sent=sent['tf'], recorded_scan_lengths=dict(lens), map_msgs=len(maps),
           known_first=maps[0][3] if maps else None, known_last=maps[-1][3] if maps else None, known_max=max(x[3] for x in maps) if maps else None,
           occupied_last=maps[-1][4] if maps else None, size_last=(maps[-1][1], maps[-1][2]) if maps else None,
           known_trace=[(round(x[0] - maps[0][0], 1), x[3]) for x in maps[::max(1, len(maps) // 15)]] if maps else [], graph_markers={f'{k[0]}|type{k[1]}': v for k, v in marks.items()})
json.dump(res, open(a.out + '.json', 'w'), indent=1)
if 'map' in latest: info, v = latest['map']; np.save(a.out + '_map.npy', v.reshape(info.height, info.width)); json.dump(dict(res=info.resolution, ox=info.origin.position.x, oy=info.origin.position.y), open(a.out + '_map_meta.json', 'w'))
rclpy.shutdown()
