import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag = sys.argv[1]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = {'/odom', '/odom_nav2', '/map'}
odom = {'/odom': [], '/odom_nav2': []}; mp = []; t0 = None
while r.has_next():
    topic, data, t = r.read_next()
    if topic not in want: continue
    if t0 is None: t0 = t
    m = deserialize_message(data, get_message(types[topic]))
    if topic == '/map':
        v = np.array(m.data, dtype=np.int8); mp.append(int((v != -1).sum()))
    else:
        odom[topic].append((m.pose.pose.position.x, m.pose.pose.position.y))
def plen(L):
    if len(L) < 2: return 0.0
    P = np.array(L); return float(np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1])).sum())
print('odom_path=%.2f/%.2f m (odom/odom_nav2) | map msgs=%d known first=%s last=%s max=%s' % (plen(odom['/odom']), plen(odom['/odom_nav2']), len(mp), mp[0] if mp else '-', mp[-1] if mp else '-', max(mp) if mp else '-'))
