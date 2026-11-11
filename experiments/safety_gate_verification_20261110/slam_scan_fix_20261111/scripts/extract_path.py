import sys, numpy as np, rosbag2_py, math
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}; rows = []; t0 = None
while r.has_next():
    topic, data, t = r.read_next()
    if topic != '/tf': continue
    m = deserialize_message(data, get_message(types[topic]))
    for tr in m.transforms:
        if tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_footprint':
            q = tr.transform.rotation; t0 = t0 or t
            rows.append(((t - t0) / 1e9, tr.transform.translation.x, tr.transform.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
a = np.array(rows); np.save(sys.argv[2], a)
d = np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2])).sum(); yaw = np.unwrap(a[:, 3]); print('poses %d, duration %.1f s, path length %.2f m, x %.2f..%.2f y %.2f..%.2f, total heading change %.0f deg' % (len(a), a[-1, 0], d, a[:, 1].min(), a[:, 1].max(), a[:, 2].min(), a[:, 2].max(), math.degrees(abs(yaw[-1] - yaw[0]))))
