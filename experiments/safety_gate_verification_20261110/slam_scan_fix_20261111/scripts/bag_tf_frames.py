import sys, collections, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}; pairs = collections.Counter(); static = []
while r.has_next():
    topic, data, t = r.read_next()
    if topic in ('/tf', '/tf_static'):
        m = deserialize_message(data, get_message(types[topic]))
        for tr in m.transforms:
            pairs[(topic, tr.header.frame_id, tr.child_frame_id)] += 1
            if topic == '/tf_static': static.append((tr.header.frame_id, tr.child_frame_id, round(tr.transform.translation.x, 3), round(tr.transform.translation.y, 3), round(tr.transform.translation.z, 3), round(tr.transform.rotation.z, 3), round(tr.transform.rotation.w, 3)))
for k, v in sorted(pairs.items()): print(k, v)
print('static transforms:'); [print('  ', s) for s in sorted(set(static))]
