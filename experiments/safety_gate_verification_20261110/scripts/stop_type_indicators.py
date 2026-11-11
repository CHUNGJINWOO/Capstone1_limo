#!/usr/bin/env python3
"""Offline, existing data only. For a run's final SLAM map: rebuild an approximate global costmap (static map + inflation with
the diff Nav2 params: robot_radius 0.18, inflation_radius 0.28, cost_scaling 7.5; NO obstacle-layer marks), write a dump for
equivalence_test, and print the minimal diagnostic indicators of the frontier-stop types.
usage: stop_type_indicators.py RUN_DIR OUT_DIR"""
import json, math, sys, collections
import numpy as np
run, out = sys.argv[1], sys.argv[2]
j = json.load(open(f'{run}/final_map.json')); m = np.load(f'{run}/final_map.npy').astype(np.int16)
res = j['resolution']; ox, oy = j['origin']; al = j['align_world_from_map']
s = json.load(open(f'{run}/summary.json'))
H, W = m.shape
RR, IR, CS = 0.18, 0.28, 7.5
lethal = m >= 65; unk = m < 0
rad = int(math.ceil(IR / res))
dist = np.full((H, W), 9.0)
for dy in range(-rad, rad + 1):
    for dx in range(-rad, rad + 1):
        d = math.hypot(dx, dy) * res
        if d > IR: continue
        sh = np.zeros_like(lethal)
        ys0, ys1 = max(0, dy), H + min(0, dy); xs0, xs1 = max(0, dx), W + min(0, dx)
        sh[ys0:ys1, xs0:xs1] = lethal[ys0 - dy:ys1 - dy, xs0 - dx:xs1 - dx]
        dist = np.where(sh, np.minimum(dist, d), dist)
cost = np.zeros((H, W), dtype=np.int16)
infl = dist <= IR
cost[infl] = np.where(dist[infl] <= RR, 253, (252 * np.exp(-CS * (dist[infl] - RR))).astype(int))
cost[lethal] = 254
# to OccupancyGrid units as nav2 publishes them
occ = np.where(cost == 0, 0, np.where(cost == 253, 99, np.where(cost == 254, 100, 1 + (97 * (cost - 1)) // 251)))
occ[unk] = -1
# robot (true pose = GT at end) in map frame
gx, gy = None, None
import csv
rows = list(csv.DictReader(open(f'{run}/timeseries.csv')))
gx, gy = float(rows[-1]['gt_x']), float(rows[-1]['gt_y'])
c, sn = math.cos(al['theta']), math.sin(al['theta'])
dxw, dyw = gx - al['tx'], gy - al['ty']
rx, ry = c * dxw + sn * dyw, -sn * dxw + c * dyw
with open(f'{out}/dump_{run.rstrip("/").split("/")[-1]}.txt', 'w') as f:
    f.write(f'{W} {H} {res} {ox} {oy} {rx} {ry}\n'); f.write(' '.join(map(str, occ.flatten())) + '\n')
# indicators
ci, cj = int((rx - ox) / res), int((ry - oy) / res)
robot_cost = int(cost[cj, ci])
free0 = (cost == 0) & ~unk
# robot's cost-0 component (4-conn) starting at nearest cost-0 cell
ys, xs = np.nonzero(free0)
k = np.argmin((xs - ci) ** 2 + (ys - cj) ** 2); st = (ys[k], xs[k]); start_dist = math.hypot(xs[k] - ci, ys[k] - cj) * res
seen = np.zeros((H, W), bool); q = collections.deque([st]); seen[st] = True
while q:
    y, x = q.popleft()
    for ny, nx in ((y+1, x), (y-1, x), (y, x+1), (y, x-1)):
        if 0 <= ny < H and 0 <= nx < W and free0[ny, nx] and not seen[ny, nx]:
            seen[ny, nx] = True; q.append((ny, nx))
def nb4_any(mask):
    r = np.zeros_like(mask)
    r[1:] |= mask[:-1]; r[:-1] |= mask[1:]; r[:, 1:] |= mask[:, :-1]; r[:, :-1] |= mask[:, 1:]; return r
unk_touch0 = unk & nb4_any(free0)                 # explore_lite frontier cells (before size filter and reachability)
unk_touch0_reach = unk & nb4_any(seen)            # ... that touch the robot's cost-0 component
infl_ok = (cost >= 1) & (cost <= 252) & ~unk
unk_touch_infl_only = unk & nb4_any(infl_ok) & ~nb4_any(free0)
# raw SLAM-map open boundary: free SLAM cell next to unknown
slam_free = (m >= 0) & (m < 25)
raw_open = slam_free & nb4_any(unk)
print(json.dumps(dict(run=run.rstrip('/').split('/')[-1], robot_map=(round(rx, 2), round(ry, 2)), robot_cell_cost=robot_cost, start_dist_m=round(start_dist, 2),
    free0_cells=int(free0.sum()), reach0_cells=int(seen.sum()), reach0_fraction=round(float(seen.sum()) / max(1, int(free0.sum())), 3),
    unknown_cells_touching_cost0=int(unk_touch0.sum()), unknown_touching_robot_cost0_component=int(unk_touch0_reach.sum()),
    unknown_touching_only_inflated=int(unk_touch_infl_only.sum()), slam_free_cells_next_to_unknown=int(raw_open.sum()))))
