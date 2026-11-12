#!/usr/bin/env python3
"""Robot path in the SLAM map frame from the recorded rosbag (/tf: map->odom and odom->base_footprint), plus travelled distance.
usage: path_from_bag.py RUN_DIR   -> writes RUN_DIR/path_map_frame.csv (t_s, x, y, yaw) and prints distance"""
import csv, math, os, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
run = sys.argv[1]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=os.path.join(run, 'bag'), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
def yaw(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
mo = None; rows = []; t0 = None; gt = []
def compose(a, b):  # a: (x,y,th) then b in a's frame
    c, s = math.cos(a[2]), math.sin(a[2]); return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])
while r.has_next():
    tp, data, t = r.read_next()
    if tp == '/ground_truth/world_odom':
        m = deserialize_message(data, get_message(types[tp])); gt.append((t * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y)); continue
    if tp != '/tf': continue
    msg = deserialize_message(data, get_message(types[tp]))
    for tr in msg.transforms:
        p = (tr.transform.translation.x, tr.transform.translation.y, yaw(tr.transform.rotation))
        if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom': mo = p
        elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_footprint' and mo is not None:
            t0 = t0 or t; x, y, th = compose(mo, p); rows.append(((t - t0) * 1e-9, x, y, th))
with open(os.path.join(run, 'path_map_frame.csv'), 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['t_s', 'x', 'y', 'yaw']); w.writerows([[round(a, 3), round(b, 4), round(c, 4), round(d, 4)] for a, b, c, d in rows])
d = sum(math.hypot(rows[i + 1][1] - rows[i][1], rows[i + 1][2] - rows[i][2]) for i in range(len(rows) - 1) if math.hypot(rows[i + 1][1] - rows[i][1], rows[i + 1][2] - rows[i][2]) < 0.5)
print(f'{len(rows)} poses, travelled distance in map frame = {d:.2f} m, duration {rows[-1][0]:.0f} s (sim clock)')
if gt:
    with open(os.path.join(run, 'gt_world_path.csv'), 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['t_s', 'x', 'y']); w.writerows([[round(a - gt[0][0], 3), round(b, 4), round(c, 4)] for a, b, c in gt])
    dg = sum(math.hypot(gt[i + 1][1] - gt[i][1], gt[i + 1][2] - gt[i][2]) for i in range(len(gt) - 1))
    print(f'ground-truth (Gazebo world) path: {len(gt)} poses, travelled distance = {dg:.2f} m')
