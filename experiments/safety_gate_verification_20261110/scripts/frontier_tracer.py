#!/usr/bin/env python3
"""Read-only: re-implement explore_lite's frontier definition on the live global costmap to see why
a region (default: NE corridor) has no frontier. Publishes nothing.

explore_lite (m-explore-ros2, frontier_search.cpp):
  * BFS starts at the free (cost 0) cell nearest the robot and only moves to 4-neighbours whose cost <= the
    current cell's cost  ==> it only crosses cells of cost exactly 0 (inflated cells 1..252 stop it);
  * a frontier cell is an unknown (255) cell with a cost-0 4-neighbour; frontiers < min_frontier_size are dropped;
  * goal = the frontier cell closest to the robot ('middle'), cost = potential_scale*dist - gain_scale*size.
usage: frontier_probe.py [--min-size 0.35] [--box x0 y0 x1 y1 in world coords]"""
import argparse, collections, json, math, subprocess, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import tf2_ros

ap = argparse.ArgumentParser()
ap.add_argument('--min-size', type=float, default=0.35)
ap.add_argument('--box', type=float, nargs=4, default=[2.3, 1.2, 6.0, 4.5])
ap.add_argument('--out', default='')
a = ap.parse_args()

import csv, os, sys
LOG = a.out  # reuse --out as csv path


def analyse(m, robot_map_xy):
    info = m.info; res = info.resolution
    A = np.asarray(m.data, dtype=np.int16).reshape(info.height, info.width); A = np.where(A < 0, 255, A)
    H, W = A.shape
    ri = int((robot_map_xy[0] - info.origin.position.x) / res); rj = int((robot_map_xy[1] - info.origin.position.y) / res)
    res_ = dict(w=W, h=H, unknown_frac=round(float((A == 255).mean()), 3), free0=int((A == 0).sum()))
    if not (0 <= ri < W and 0 <= rj < H):
        res_.update(robot_in_bounds=False); return res_
    res_['robot_cell_cost'] = int(A[rj, ri])
    dq = collections.deque([(rj, ri)]); seen = {(rj, ri)}; start = None
    while dq:
        j, i = dq.popleft()
        if A[j, i] == 0:
            start = (j, i); break
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                jj, ii = j + dj, i + di
                if 0 <= jj < H and 0 <= ii < W and (jj, ii) not in seen:
                    seen.add((jj, ii)); dq.append((jj, ii))
    if start is None:
        res_.update(start_found=False, frontiers_kept=0, reachable0=0); return res_
    res_['start_dist_m'] = round(math.hypot(start[0] - rj, start[1] - ri) * res, 2)
    nb = lambda j, i: [(jj, ii) for jj, ii in ((j + 1, i), (j - 1, i), (j, i + 1), (j, i - 1)) if 0 <= jj < H and 0 <= ii < W]
    visited = np.zeros(A.shape, bool); ff = np.zeros(A.shape, bool); fr = []
    dq = collections.deque([start]); visited[start] = True
    while dq:
        j, i = dq.popleft()
        for jj, ii in nb(j, i):
            if A[jj, ii] <= A[j, i] and not visited[jj, ii]:
                visited[jj, ii] = True; dq.append((jj, ii))
            elif A[jj, ii] == 255 and not ff[jj, ii] and any(A[a_, b_] == 0 for a_, b_ in nb(jj, ii)):
                ff[jj, ii] = True; cells = [(jj, ii)]; stack = [(jj, ii)]
                while stack:
                    y, x = stack.pop()
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            y2, x2 = y + dy, x + dx
                            if 0 <= y2 < H and 0 <= x2 < W and A[y2, x2] == 255 and not ff[y2, x2] and any(A[a_, b_] == 0 for a_, b_ in nb(y2, x2)):
                                ff[y2, x2] = True; cells.append((y2, x2)); stack.append((y2, x2))
                fr.append(len(cells))
    res_.update(reachable0=int(visited.sum()), frontiers_total=len(fr), frontiers_kept=sum(1 for c in fr if c * res >= 0.35),
                largest_frontier_m=round(max(fr) * res, 2) if fr else 0.0)
    return res_


rclpy.init()
n = Node('frontier_tracer', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
vo = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
st = {}
for q in (tl, vo):
    n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: st.__setitem__('g', m), q)
for q in (tl, vo):
    n.create_subscription(OccupancyGrid, '/map', lambda m: st.__setitem__('slam', m), q)
tfb = tf2_ros.Buffer(); tf2_ros.TransformListener(tfb, n)
gate = {'state': None, 'bumper_msgs': 0, 'bumper_last_sim': None}


def cb_gate(m):
    try:
        gate['state'] = json.loads(m.data).get('state')
    except Exception:
        pass


def cb_bump(m):
    gate['bumper_msgs'] += 1; gate['bumper_last_sim'] = n.get_clock().now().nanoseconds * 1e-9


n.create_subscription(String, '/stall_detector/status_json', cb_gate, 20)
n.create_subscription(PointCloud2, '/stall_detector/virtual_bumper_cloud', cb_bump, 20)
snap = {'n': 0, 'prev_ratio': 1.0, 'since': None}
os.makedirs(os.path.dirname(LOG) + '/snapshots', exist_ok=True)
SPAWN = (-1.10, -0.60, 1.5708)     # default sim spawn = SLAM map frame origin/heading


def unknown_in_world_box(m, box, costmap):
    info = m.info; res = info.resolution
    A = np.asarray(m.data, dtype=np.int16)
    A = A.reshape(info.height, info.width)
    unk = (A < 0) | (A == 255) if costmap else (A < 0)
    tot = 0; u = 0; ob = 0
    x0, y0, x1, y1 = box
    for wx in np.arange(x0, x1, 0.05):
        for wy in np.arange(y0, y1, 0.05):
            dx, dy = wx - SPAWN[0], wy - SPAWN[1]
            mx = math.cos(-SPAWN[2]) * dx - math.sin(-SPAWN[2]) * dy; my = math.sin(-SPAWN[2]) * dx + math.cos(-SPAWN[2]) * dy
            i = int((mx - info.origin.position.x) / res); j = int((my - info.origin.position.y) / res)
            if 0 <= i < info.width and 0 <= j < info.height:
                tot += 1; u += bool(unk[j, i]); ob += int(A[j, i] >= 99)
    return tot, u, ob


NE_BOX = (3.0, 1.8, 5.8, 4.3)
f = open(LOG, 'w'); first = True; t_last = 0.0
while True:
    rclpy.spin_once(n, timeout_sec=0.05)
    now = n.get_clock().now().nanoseconds * 1e-9
    if now - t_last < 1.0 or 'g' not in st:
        continue
    t_last = now
    try:
        tr = tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform.translation
    except Exception:
        continue
    r = analyse(st['g'], (tr.x, tr.y))
    if 'slam' in st:
        t_, u_, o_ = unknown_in_world_box(st['slam'], NE_BOX, False); r['NE_slam_cells'] = t_; r['NE_slam_unknown'] = u_; r['NE_slam_occ'] = o_
    t_, u_, o_ = unknown_in_world_box(st['g'], NE_BOX, True); r['NE_costmap_cells'] = t_; r['NE_costmap_unknown'] = u_; r['NE_costmap_ge99'] = o_
    r['sim_t'] = round(now, 1); r['robot_map'] = (round(tr.x, 2), round(tr.y, 2))
    r['gate_state'] = gate['state']; r['bumper_msgs_total'] = gate['bumper_msgs']
    r['bumper_age_s'] = round(now - gate['bumper_last_sim'], 1) if gate['bumper_last_sim'] else None
    ratio = (r.get('reachable0', 0) / r['free0']) if r.get('free0') else 1.0
    take = False
    if snap['prev_ratio'] > 0.8 and ratio < 0.5 and snap['n'] < 4:      # robot's cost-0 component just collapsed
        snap['since'] = now; take = True
    elif snap['since'] is not None and now - snap['since'] < 7 and snap['n'] < 4:
        take = True
    if take:
        snap['n'] += 1
        np.save(os.path.dirname(LOG) + f"/snapshots/costmap_{snap['n']}_{int(now)}.npy", np.asarray(st['g'].data, dtype=np.int8).reshape(st['g'].info.height, st['g'].info.width))
        if 'slam' in st:
            np.save(os.path.dirname(LOG) + f"/snapshots/slam_{snap['n']}_{int(now)}.npy", np.asarray(st['slam'].data, dtype=np.int8).reshape(st['slam'].info.height, st['slam'].info.width))
        r['snapshot'] = snap['n']; r['costmap_origin'] = (st['g'].info.origin.position.x, st['g'].info.origin.position.y)
    snap['prev_ratio'] = ratio
    f.write(json.dumps(r) + '\n'); f.flush()
