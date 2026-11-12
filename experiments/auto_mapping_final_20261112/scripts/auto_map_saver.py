#!/usr/bin/env python3
"""Automatic map saver: waits for explore_lite to report EXPLORATION_COMPLETE on /explore/status and then saves the
SLAM Toolbox map (/map) with nav2_map_server's map_saver_cli. Falls back to saving after --cap-wall seconds.
Never publishes velocity; it only reads topics and calls the map saver.
usage: auto_map_saver.py --prefix OUT/limo_sim_map --out-json OUT/saved.json [--cap-wall 1500] [--settle 3]"""
import argparse, json, subprocess, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from explore_lite_msgs.msg import ExploreStatus

ap = argparse.ArgumentParser()
ap.add_argument('--prefix', required=True); ap.add_argument('--out-json', required=True)
ap.add_argument('--cap-wall', type=float, default=1500.0); ap.add_argument('--settle', type=float, default=3.0)
a = ap.parse_args()
rclpy.init(); n = Node('auto_map_saver')
state = {'status': None, 'complete_at': None, 'history': []}
def on_status(m):
    state['history'].append((round(time.monotonic() - t0, 1), m.status))
    state['status'] = m.status
    if m.status == ExploreStatus.EXPLORATION_COMPLETE and state['complete_at'] is None:
        state['complete_at'] = time.monotonic()
q = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(ExploreStatus, '/explore/status', on_status, q)
t0 = time.monotonic(); reason = None
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.2)
    if state['complete_at'] and time.monotonic() - state['complete_at'] >= a.settle:
        reason = 'exploration_complete'; break
    if time.monotonic() - t0 > a.cap_wall:
        reason = 'wall_cap'; break
r = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', a.prefix, '--ros-args', '-p', 'save_map_timeout:=15.0',
                    '-p', 'free_thresh_default:=0.25', '-p', 'occupied_thresh_default:=0.65'], capture_output=True, text=True, timeout=60)
json.dump({'saved_because': reason, 'wall_s_since_start': round(time.monotonic() - t0, 1), 'status_history': state['history'],
           'map_saver_returncode': r.returncode, 'prefix': a.prefix, 'map_saver_tail': (r.stdout + r.stderr)[-400:]}, open(a.out_json, 'w'), indent=1)
print('saved', reason, r.returncode, flush=True)
