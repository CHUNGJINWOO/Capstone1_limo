import sys, math, collections, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag = sys.argv[1]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
cnt = collections.Counter(); inc = collections.Counter(); amin = collections.Counter(); amax = collections.Counter()
first = []; mapk = []; t0 = None; scan_t = []; odom = []; inv = collections.Counter(); n_int = 0; scan_seq = []
while r.has_next():
    topic, data, t = r.read_next()
    if t0 is None: t0 = t
    if topic == '/scan':
        m = deserialize_message(data, get_message(types[topic]))
        n = len(m.ranges); cnt[n] += 1; inc[round(m.angle_increment, 6)] += 1; amin[round(m.angle_min, 5)] += 1; amax[round(m.angle_max, 5)] += 1
        if len(first) < 12: first.append((round((t - t0) / 1e9, 2), n, round(m.angle_min, 5), round(m.angle_max, 5), round(m.angle_increment, 6), round(m.scan_time, 4), round(m.time_increment, 6), len(m.intensities)))
        scan_seq.append((round((t - t0) / 1e9, 2), n))
        a = np.array(m.ranges, dtype=float); inv['inf'] += int(np.isinf(a).sum()); inv['nan'] += int(np.isnan(a).sum()); inv['<=range_min'] += int(((a <= m.range_min) & np.isfinite(a)).sum()); inv['total'] += len(a)
        n_int += len(m.intensities) > 0
    elif topic == '/map':
        m = deserialize_message(data, get_message(types[topic]))
        v = np.array(m.data, dtype=np.int8); mapk.append((round((t - t0) / 1e9, 1), m.info.width, m.info.height, int((v != -1).sum()), int((v >= 65).sum())))
    elif topic == '/odom_nav2':
        m = deserialize_message(data, get_message(types[topic])); odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y))
print('scan messages:', sum(cnt.values()), '| len(ranges) distribution:', dict(sorted(cnt.items())))
print('angle_increment values:', dict(inc.most_common(4)), '| angle_min:', dict(amin.most_common(3)), '| angle_max:', dict(amax.most_common(3)))
print('messages with intensities:', n_int, '| invalid beam encodings (share of all beams):', {k: v for k, v in inv.items()}, )
print('first scans [t, N, angle_min, angle_max, inc, scan_time, time_inc, n_intens]:'); [print('  ', f) for f in first]
if scan_seq:
    ns = [n for _, n in scan_seq]; print('N over time (every ~25th scan):', [(t, n) for t, n in scan_seq[::25]][:14])
print('/map messages:', len(mapk)); [print('  map', x) for x in (mapk[:2] + mapk[len(mapk)//4:len(mapk)//4+1] + mapk[len(mapk)//2:len(mapk)//2+1] + mapk[-3:])] if mapk else None
if odom:
    P = np.array([(x, y) for _, x, y in odom]); d = np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1])).sum(); print('odom_nav2 path length %.2f m, extent x %.2f..%.2f y %.2f..%.2f' % (d, P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max()))
