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
import tf2_ros

ap = argparse.ArgumentParser()
ap.add_argument('--min-size', type=float, default=0.35)
ap.add_argument('--box', type=float, nargs=4, default=[2.3, 1.2, 6.0, 4.5])
ap.add_argument('--out', default='')
a = ap.parse_args()
rclpy.init()
n = Node('frontier_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
vo = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
st = {}
for q in (tl, vo):
    n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: st.__setitem__('g', m), q)
n.create_subscription(Odometry, '/ground_truth/world_odom', lambda m: st.__setitem__('gt', m), 10)
tfb = tf2_ros.Buffer(); tf2_ros.TransformListener(tfb, n)
t0 = time.monotonic()
while time.monotonic() - t0 < 40 and not ('g' in st and 'gt' in st):
    rclpy.spin_once(n, timeout_sec=0.1)
for _ in range(30):
    rclpy.spin_once(n, timeout_sec=0.1)
m = st['g']; info = m.info; res = info.resolution
A = np.asarray(m.data, dtype=np.int16).reshape(info.height, info.width)
A = np.where(A < 0, 255, A)        # explore_lite sees NO_INFORMATION=255
tr = None; t1 = time.monotonic()
while tr is None and time.monotonic() - t1 < 25:
    rclpy.spin_once(n, timeout_sec=0.2)
    try:
        tr = tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
    except Exception:
        pass
if tr is not None:
    sx, sy = tr.translation.x, tr.translation.y; syaw = 2 * math.atan2(tr.rotation.z, tr.rotation.w)
else:   # fallback: SLAM map frame starts at the spawn pose (-1.10,-0.60, yaw 1.5708) -> robot pose in map from GT
    p0 = st['gt'].pose.pose; q0 = p0.orientation
    gyaw0 = math.atan2(2 * (q0.w * q0.z + q0.x * q0.y), 1 - 2 * (q0.y * q0.y + q0.z * q0.z))
    d0x, d0y = p0.position.x + 1.10, p0.position.y + 0.60
    sx = math.cos(-1.5708) * d0x - math.sin(-1.5708) * d0y; sy = math.sin(-1.5708) * d0x + math.cos(-1.5708) * d0y; syaw = gyaw0 - 1.5708
    print('WARNING: map TF unavailable; using spawn-pose alignment')
p = st['gt'].pose.pose; gx, gy = p.position.x, p.position.y; q = p.orientation; gyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
th = gyaw - syaw; c, s = math.cos(th), math.sin(th); tx = gx - (c * sx - s * sy); ty = gy - (s * sx + c * sy)
to_world = lambda mx, my: (c * mx - s * my + tx, s * mx + c * my + ty)
to_map = lambda wx, wy: (c * (wx - tx) + s * (wy - ty), -s * (wx - tx) + c * (wy - ty))
cell_xy = lambda i, j: (info.origin.position.x + (i + 0.5) * res, info.origin.position.y + (j + 0.5) * res)
H, W = A.shape
out = dict(robot_map=(round(sx, 2), round(sy, 2)), robot_world=(round(gx, 2), round(gy, 2)), costmap=f'{W}x{H}@{res}')


def nb4(j, i):
    for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        jj, ii = j + dj, i + di
        if 0 <= jj < H and 0 <= ii < W:
            yield jj, ii


# robot cell -> nearest cost-0 cell (8-neighbourhood BFS over all cells, like nearestCell)
ri = int((sx - info.origin.position.x) / res); rj = int((sy - info.origin.position.y) / res)
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
out['robot_cell_cost'] = int(A[rj, ri]); out['bfs_start_cell_dist_m'] = round(math.hypot(start[0] - rj, start[1] - ri) * res, 2) if start else None
visited = np.zeros_like(A, dtype=bool); frontier_flag = np.zeros_like(A, dtype=bool)
frontiers = []
dq = collections.deque([start]); visited[start] = True
while dq:
    j, i = dq.popleft()
    for jj, ii in nb4(j, i):
        if A[jj, ii] <= A[j, i] and not visited[jj, ii]:
            visited[jj, ii] = True; dq.append((jj, ii))
        elif A[jj, ii] == 255 and not frontier_flag[jj, ii] and any(A[j2, i2] == 0 for j2, i2 in nb4(jj, ii)):
            # build frontier over the 8-neighbourhood of unknown cells that touch free cells
            frontier_flag[jj, ii] = True; cells = [(jj, ii)]; stack = [(jj, ii)]
            while stack:
                y, x = stack.pop()
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        y2, x2 = y + dy, x + dx
                        if 0 <= y2 < H and 0 <= x2 < W and A[y2, x2] == 255 and not frontier_flag[y2, x2] and any(A[a_, b_] == 0 for a_, b_ in nb4(y2, x2)):
                            frontier_flag[y2, x2] = True; cells.append((y2, x2)); stack.append((y2, x2))
            frontiers.append(cells)
out['cost0_cells_reachable_from_robot'] = int(visited.sum())
out['cost0_cells_total'] = int((A == 0).sum())
rows = []
for cells in frontiers:
    ws = [to_world(*cell_xy(i, j)) for j, i in cells]
    mid = min(ws, key=lambda w: math.hypot(w[0] - gx, w[1] - gy))
    cen = (float(np.mean([w[0] for w in ws])), float(np.mean([w[1] for w in ws])))
    rows.append(dict(size_m=round(len(cells) * res, 2), n_cells=len(cells), middle_world=[round(mid[0], 2), round(mid[1], 2)], centroid_world=[round(cen[0], 2), round(cen[1], 2)],
                     kept=len(cells) * res >= a.min_size))
rows.sort(key=lambda r: -r['size_m'])
out['frontiers_found_by_bfs'] = rows
out['frontiers_kept(min_size=%.2f)' % a.min_size] = sum(r['kept'] for r in rows)

# boundary between known and unknown inside the box: what costs do the known neighbours of unknown cells have?
x0, y0, x1, y1 = a.box
hist = collections.Counter(); samples = []
for j in range(H):
    for i in range(W):
        if A[j, i] != 255:
            continue
        wx, wy = to_world(*cell_xy(i, j))
        if not (x0 <= wx <= x1 and y0 <= wy <= y1):
            continue
        costs = [int(A[jj, ii]) for jj, ii in nb4(j, i) if A[jj, ii] != 255]
        if not costs:
            continue
        k = min(costs)
        hist['0 (free: would be a frontier cell)' if k == 0 else ('1-98 (inflated)' if k < 99 else ('99 (inscribed)' if k == 99 else '100 (lethal)'))] += 1
out['unknown_cells_in_box_touching_known: min neighbour cost histogram'] = dict(hist)
out['unknown_cells_in_box_total'] = int(sum(1 for j in range(H) for i in range(W) if A[j, i] == 255 and x0 <= to_world(*cell_xy(i, j))[0] <= x1 and y0 <= to_world(*cell_xy(i, j))[1] <= y1))

# ASCII picture of the box (world axes, 0.1 m per char; priority: lethal > inscribed > inflated > free > unknown)
lines = []
step = 2
for wy in np.arange(y1, y0 - 1e-6, -0.1):
    row = ''
    for wx in np.arange(x0, x1 + 1e-6, 0.1):
        mx, my = to_map(wx, wy)
        i = int((mx - info.origin.position.x) / res); j = int((my - info.origin.position.y) / res)
        if not (0 <= i < W and 0 <= j < H):
            row += ' '; continue
        win = A[max(0, j - 1):j + 2, max(0, i - 1):i + 2]
        mx_ = int(win.max()); known = win[win != 255]
        if (win == 100).any(): ch = '#'
        elif (win == 99).any(): ch = 'o'
        elif known.size and known.max() > 0 and (known > 0).any(): ch = ':'
        elif (win == 0).any(): ch = '.'
        else: ch = '?'
        row += ch
    lines.append(f'{wy:5.1f} {row}')
out['ascii_legend'] = "'#' lethal  'o' inscribed(99)  ':' inflated(1-98)  '.' free(0)  '?' unknown;  columns x=%.1f..%.1f step 0.1, rows y=%.1f..%.1f" % (x0, x1, y1, y0)
print(json.dumps(out, indent=1))
print('\n'.join(lines))
if a.out:
    json.dump(out, open(a.out, 'w'), indent=1); open(a.out + '.ascii.txt', 'w').write('\n'.join(lines))
import os; os._exit(0)
