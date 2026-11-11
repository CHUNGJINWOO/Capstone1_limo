#!/usr/bin/env python3
"""Passive per-stage latency probe for the simulated depth pipeline.

Stages (all ages are sim-clock now - message stamp, so they are what the depth filter and the gate see):
  image   : /camera/depth_image            (Gazebo render -> ros_gz_bridge)
  points  : /camera/depth/points           (depth_to_pointcloud_fixed_node)
  obst    : /depth_obstacles/points        (depth_obstacle_filter_node, throttled by processing_hz)
  status  : /auto_mapping/depth_status     (age of `source_stamp_sec` at publish time = what DEPTH_STALE tests)
Also records: sim-time and wall-time inter-arrival, which image stamps never reach /depth_obstacles/points
(dropped by the filter's wall-clock throttle), RTF, CPU per process group, system load.
Publishes nothing. usage: depth_latency_probe.py --out DIR --seconds N"""
import argparse, csv, json, math, os, threading, time
import numpy as np
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String

ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--seconds', type=float, default=300.0)
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)

GROUPS = {'gazebo': ['ign gazebo'], 'depth_filter': ['depth_obstacle_filter_node'], 'depth_to_points': ['depth_to_pointcloud'],
          'nav2': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'velocity_smoother', 'smoother_server'],
          'slam': ['async_slam_toolbox_node'], 'f4_icp': ['lidar_scan_matcher_odom_node'], 'gate': ['stall_detector_node'],
          'bridge': ['parameter_bridge']}


class P(Node):
    def __init__(self):
        super().__init__('depth_latency_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.t0 = time.monotonic(); self.sim = 0.0; self.clk = []
        self.age = {k: [] for k in ('image', 'points', 'obst', 'status')}      # (wall_t, sim_t, age)
        self.gap_sim = {k: [] for k in ('image', 'points', 'obst')}; self.gap_wall = {k: [] for k in ('image', 'points', 'obst')}
        self.last = {}
        self.stamps = {'image': set(), 'obst': set()}
        self.status_events = []; self.prev_code = None
        self.cpu = []; self.cpu_proc = {}
        be = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', self.cb_clock, 50)
        self.create_subscription(Image, '/camera/depth_image', lambda m: self.cb('image', m), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/camera/depth/points', lambda m: self.cb('points', m), be)
        self.create_subscription(PointCloud2, '/depth_obstacles/points', lambda m: self.cb('obst', m), qos_profile_sensor_data)
        self.create_subscription(String, '/auto_mapping/depth_status', self.cb_status, 20)
        threading.Thread(target=self.cpu_loop, daemon=True).start()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_clock(self, m):
        w = time.monotonic()
        if not self.clk or w - self.clk[-1][0] >= 0.25:
            self.clk.append((w, m.clock.sec + m.clock.nanosec * 1e-9))
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9

    def cb(self, k, m):
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9; t = self.now(); w = time.monotonic()
        self.age[k].append((w - self.t0, t, t - s))
        if k in self.stamps:
            self.stamps[k].add(round(s, 3))
        if k in self.last:
            self.gap_sim[k].append(t - self.last[k][0]); self.gap_wall[k].append(w - self.last[k][1])
        self.last[k] = (t, w)

    def cb_status(self, m):
        try:
            d = json.loads(m.data)
        except Exception:
            return
        t = self.now(); w = time.monotonic()
        src = d.get('source_stamp_sec')
        if src is not None:
            self.age['status'].append((w - self.t0, t, t - float(src)))
        c = d.get('code')
        if c != self.prev_code:
            self.status_events.append((round(t, 2), c, d.get('message', '')))
            self.prev_code = c

    def cpu_loop(self):
        while True:
            time.sleep(1.0)
            tot = {g: 0.0 for g in GROUPS}
            for p in psutil.process_iter(['pid', 'cmdline']):
                try:
                    cl = ' '.join(p.info['cmdline'] or [])
                except Exception:
                    continue
                for g, subs in GROUPS.items():
                    if any(x in cl for x in subs):
                        pr = self.cpu_proc.get(p.info['pid'])
                        if pr is None:
                            try:
                                pr = psutil.Process(p.info['pid']); pr.cpu_percent(None); self.cpu_proc[p.info['pid']] = pr
                            except Exception:
                                break
                            break
                        try:
                            tot[g] += pr.cpu_percent(None)
                        except Exception:
                            pass
                        break
            self.cpu.append((time.monotonic() - self.t0, psutil.cpu_percent(None), os.getloadavg()[0], tot))


rclpy.init(); n = P()
end = time.monotonic() + a.seconds
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.05)


def pct(v, q):
    return float(np.percentile(v, q)) if len(v) else None


out = dict(seconds=a.seconds, sim_seconds=None)
if len(n.clk) > 2:
    out['rtf_overall'] = (n.clk[-1][1] - n.clk[0][1]) / (n.clk[-1][0] - n.clk[0][0])
    out['sim_seconds'] = n.clk[-1][1] - n.clk[0][1]
    # 10 s windows
    w = []
    i = 0
    while i < len(n.clk):
        j = i
        while j < len(n.clk) and n.clk[j][0] - n.clk[i][0] < 10: j += 1
        if j >= len(n.clk): break
        w.append((n.clk[j][1] - n.clk[i][1]) / (n.clk[j][0] - n.clk[i][0])); i = j
    out['rtf_10s_windows'] = dict(min=min(w), mean=float(np.mean(w)), max=max(w)) if w else None
for k, v in n.age.items():
    ages = np.array([x[2] for x in v]) if v else np.array([])
    out[f'age_{k}'] = dict(n=len(v), p50=pct(ages, 50), p95=pct(ages, 95), p99=pct(ages, 99), max=float(ages.max()) if len(ages) else None,
                           over_0p25=int((ages > 0.25).sum()), over_0p30=int((ages > 0.30).sum()), over_0p38=int((ages > 0.38).sum()))
for k in ('image', 'points', 'obst'):
    gs = np.array(n.gap_sim[k]); gw = np.array(n.gap_wall[k])
    out[f'gap_{k}'] = dict(sim_p50=pct(gs, 50), sim_p99=pct(gs, 99), sim_max=float(gs.max()) if len(gs) else None,
                           wall_p50=pct(gw, 50), wall_p99=pct(gw, 99), wall_max=float(gw.max()) if len(gw) else None,
                           sim_over_0p19=int((gs > 0.19).sum()), n=len(gs))
img, ob = n.stamps['image'], n.stamps['obst']
out['frames'] = dict(images=len(img), obstacle_clouds=len(ob), dropped_before_publish=len(img - ob),
                     drop_fraction=round(len(img - ob) / max(1, len(img)), 3))
out['status_code_events'] = n.status_events
out['cpu_summary'] = {g: dict(mean=float(np.mean([c[3][g] for c in n.cpu])), max=float(np.max([c[3][g] for c in n.cpu]))) for g in GROUPS} if n.cpu else None
out['system_cpu_pct'] = dict(mean=float(np.mean([c[1] for c in n.cpu])), max=float(np.max([c[1] for c in n.cpu]))) if n.cpu else None
out['load1_mean'] = float(np.mean([c[2] for c in n.cpu])) if n.cpu else None
# worst status ages with the CPU/RTF context of that second
worst = sorted(n.age['status'], key=lambda x: -x[2])[:8]
ctx = []
for wt, st, ag in worst:
    c = min(n.cpu, key=lambda c: abs(c[0] - wt)) if n.cpu else None
    ctx.append(dict(wall_s=round(wt, 1), sim_s=round(st, 2), status_age=round(ag, 3), sys_cpu=c[1] if c else None, load1=round(c[2], 1) if c else None,
                    depth_filter_cpu=round(c[3]['depth_filter'], 0) if c else None, gazebo_cpu=round(c[3]['gazebo'], 0) if c else None))
out['worst_status_ages'] = ctx
json.dump(out, open(os.path.join(a.out, 'depth_latency.json'), 'w'), indent=1)
with open(os.path.join(a.out, 'status_ages.csv'), 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['wall_s', 'sim_s', 'status_age', 'image_age_last', 'obst_age_last'])
    for wt, st, ag in n.age['status']:
        w.writerow([round(wt, 3), round(st, 3), round(ag, 4), '', ''])
with open(os.path.join(a.out, 'stage_ages.csv'), 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['stage', 'wall_s', 'sim_s', 'age'])
    for k in ('image', 'points', 'obst'):
        for wt, st, ag in n.age[k]:
            w.writerow([k, round(wt, 3), round(st, 3), round(ag, 4)])
with open(os.path.join(a.out, 'cpu_1s.csv'), 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['wall_s', 'sys_cpu', 'load1'] + list(GROUPS))
    for c in n.cpu:
        w.writerow([round(c[0], 1), c[1], round(c[2], 2)] + [round(c[3][g], 1) for g in GROUPS])
print('PROBE_DONE'); os._exit(0)
