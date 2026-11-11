#!/usr/bin/env python3
"""Measure topic rates (wall & sim), max inter-arrival gap in sim time, and RTF."""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2, Image
from nav_msgs.msg import Odometry

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
TOPICS = [('/scan', LaserScan), ('/odometry/lidar_odom', Odometry),
          ('/depth_obstacles/points', PointCloud2), ('/camera/depth_image', Image)]


class P(Node):
    def __init__(self):
        super().__init__('rate_probe')
        self.sim = None
        self.clk = []
        self.create_subscription(Clock, '/clock', self.cb_clock, 50)
        self.data = {}
        for name, typ in TOPICS:
            self.data[name] = []
            self.create_subscription(typ, name, lambda m, n=name: self.cb(n), qos_profile_sensor_data)

    def cb_clock(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9
        self.clk.append((time.monotonic(), self.sim))

    def cb(self, n):
        self.data[n].append((time.monotonic(), self.sim))


rclpy.init()
p = P()
t0 = time.monotonic()
while time.monotonic() - t0 < DUR:
    rclpy.spin_once(p, timeout_sec=0.01)
if len(p.clk) >= 2:
    (w0, s0), (w1, s1) = p.clk[0], p.clk[-1]
    print(f'RTF={(s1 - s0) / (w1 - w0):.3f}  sim {s0:.2f}->{s1:.2f} over wall {w1 - w0:.2f}s')
else:
    print('no /clock')
for n, v in p.data.items():
    if len(v) < 2:
        print(f'{n}: msgs={len(v)}')
        continue
    wall = v[-1][0] - v[0][0]
    sims = [s for _, s in v if s is not None]
    simspan = sims[-1] - sims[0] if len(sims) >= 2 else float('nan')
    gaps_sim = [b - a for a, b in zip(sims, sims[1:])]
    gaps_wall = [b[0] - a[0] for a, b in zip(v, v[1:])]
    print(f'{n}: msgs={len(v)} wall_hz={(len(v) - 1) / wall:.2f} sim_hz={(len(v) - 1) / simspan:.2f} '
          f'max_gap_sim={max(gaps_sim):.3f}s max_gap_wall={max(gaps_wall):.3f}s '
          f'n_gap_sim>0.38={sum(g > 0.38 for g in gaps_sim)}')
