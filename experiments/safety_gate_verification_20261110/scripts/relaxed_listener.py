#!/usr/bin/env python3
"""Passive comparison listener for sim_diff_relaxed_passive.launch.py (publishes nothing, never moves the robot).

* /relaxed/relaxed_cap0/candidates   (explore_relaxed dry_run, cap 0)  -> must agree with the original explore_lite
* /relaxed/relaxed_cap60/candidates  (dry_run, cap 60 + reachability)  -> relaxed discovery
* /explore/frontiers markers         -> the ORIGINAL explore_lite frontier count (SPHERE markers = one per frontier)
For every new cap-60 candidate list the listener asks the Nav2 global planner (ComputePathToPose, read-only) whether each
candidate is drivable and stores it next to the costmap-based flag. Output: <out>/relaxed_passive.jsonl
usage: relaxed_listener.py --out DIR --seconds N"""
import argparse, json, math, os, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--seconds', type=float, default=300)
a = ap.parse_args()
rclpy.init()
n = Node('relaxed_listener', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
f = open(os.path.join(a.out, 'relaxed_passive.jsonl'), 'w')
state = {'orig_count': None, 'orig_t': None, 'last_query': 0.0}
cands = {}
ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')


def now():
    return n.get_clock().now().nanoseconds * 1e-9


def cb_markers(m):
    state['orig_count'] = sum(1 for k in m.markers if k.type == 2 and k.action == 0)    # SPHERE, ADD
    state['orig_t'] = now()


def cb_c(name):
    def cb(m):
        try:
            cands[name] = json.loads(m.data)
        except Exception:
            return
        d = dict(cands[name]); d['who'] = name; d['sim_t'] = round(now(), 2)
        if name == 'cap0':
            d['original_marker_count'] = state['orig_count']; d['original_marker_age'] = round(now() - state['orig_t'], 2) if state['orig_t'] else None
        if name == 'cap60' and d.get('list') and now() - state['last_query'] > 4.0 and ac.server_is_ready():
            state['last_query'] = now()
            state['pending'] = d          # Nav2 queries run in the main loop (no nested spinning inside a callback)
            return
        f.write(json.dumps(d) + '\n'); f.flush()
    return cb


def query_nav2(d):
    if True:
        if True:
            res = []
            for c in d['list']:
                g = ComputePathToPose.Goal(); g.use_start = False; g.planner_id = 'GridBased'
                ps = PoseStamped(); ps.header.frame_id = 'map'; ps.pose.position.x = c['m'][0]; ps.pose.position.y = c['m'][1]; ps.pose.orientation.w = 1.0
                g.goal = ps
                fut = ac.send_goal_async(g); t0 = time.monotonic()
                while not fut.done() and time.monotonic() - t0 < 3: rclpy.spin_once(n, timeout_sec=0.02)
                if not fut.done() or not fut.result().accepted:
                    res.append(dict(m=c['m'], nav2='no_goal')); continue
                rf = fut.result().get_result_async(); t0 = time.monotonic()
                while not rf.done() and time.monotonic() - t0 < 4: rclpy.spin_once(n, timeout_sec=0.02)
                if not rf.done():
                    res.append(dict(m=c['m'], nav2='timeout')); continue
                r = rf.result()
                pts = r.result.path.poses if r.status == 4 else []
                L = sum(math.hypot(pts[i + 1].pose.position.x - pts[i].pose.position.x, pts[i + 1].pose.position.y - pts[i].pose.position.y) for i in range(len(pts) - 1))
                res.append(dict(m=c['m'], nav2='ok' if r.status == 4 else f'fail{r.status}', path_len=round(L, 2), costmap_flag=c.get('r'), blacklisted=c.get('bl')))
            d['nav2_check'] = res
            f.write(json.dumps(d) + '\n'); f.flush()


be = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(MarkerArray, '/explore/frontiers', cb_markers, be)
n.create_subscription(String, '/relaxed/relaxed_cap0/candidates', cb_c('cap0'), be)
n.create_subscription(String, '/relaxed/relaxed_cap60/candidates', cb_c('cap60'), be)
t0 = time.monotonic()
while time.monotonic() - t0 < a.seconds:
    rclpy.spin_once(n, timeout_sec=0.05)
    if state.get('pending'):
        d = state.pop('pending'); query_nav2(d)
os._exit(0)
