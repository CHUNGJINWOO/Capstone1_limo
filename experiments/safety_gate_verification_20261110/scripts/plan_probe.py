#!/usr/bin/env python3
"""Read-only: ask the global planner for paths from the robot's current pose to goals around it
(map frame, robot-relative: ahead / behind / left / right at 1 m). Publishes no velocity."""
import argparse, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
import tf2_ros

ap = argparse.ArgumentParser(); ap.add_argument('--spawn', type=float, nargs=3, default=None); ap.add_argument('--world-goals', default='')
a = ap.parse_args()
rclpy.init(); n = Node('plan_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
tfb = tf2_ros.Buffer(); tf2_ros.TransformListener(tfb, n)
ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
ac.wait_for_server(timeout_sec=30)
t0 = time.monotonic(); tr = None
while time.monotonic() - t0 < 30 and tr is None:
    rclpy.spin_once(n, timeout_sec=0.1)
    try:
        tr = tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time())
    except Exception:
        pass
x, y = tr.transform.translation.x, tr.transform.translation.y
q = tr.transform.rotation; yaw = 2 * math.atan2(q.z, q.w)
print(f'robot map pose=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.0f} deg')
targets = []
if a.spawn and a.world_goals:
    sx_, sy_, syaw_ = a.spawn      # SLAM map frame origin = spawn pose (map starts where the robot starts)
    for g in a.world_goals.split(';'):
        wx, wy = map(float, g.split(','))
        dx_, dy_ = wx - sx_, wy - sy_
        targets.append((f'world({wx:.2f},{wy:.2f})', math.cos(-syaw_) * dx_ - math.sin(-syaw_) * dy_, math.sin(-syaw_) * dx_ + math.cos(-syaw_) * dy_))
else:
    for name, (dx, dy) in {'ahead': (1.0, 0.0), 'behind': (-1.0, 0.0), 'left': (0.0, 1.0), 'right': (0.0, -1.0), 'ahead_left': (1.0, 1.0)}.items():
        targets.append((name, x + dx * math.cos(yaw) - dy * math.sin(yaw), y + dx * math.sin(yaw) + dy * math.cos(yaw)))
for name, gx, gy in targets:
    g = ComputePathToPose.Goal(); g.use_start = False; g.planner_id = 'GridBased'
    ps = PoseStamped(); ps.header.frame_id = 'map'; ps.pose.position.x = gx; ps.pose.position.y = gy; ps.pose.orientation.w = 1.0
    g.goal = ps
    f = ac.send_goal_async(g)
    while not f.done():
        rclpy.spin_once(n, timeout_sec=0.05)
    h = f.result()
    if not h.accepted:
        print(f'{name}: rejected'); continue
    r = h.get_result_async()
    while not r.done():
        rclpy.spin_once(n, timeout_sec=0.05)
    res = r.result()
    ok = res.status == 4
    L = 0.0
    if ok:
        pts = res.result.path.poses
        L = sum(math.hypot(pts[i + 1].pose.position.x - pts[i].pose.position.x, pts[i + 1].pose.position.y - pts[i].pose.position.y) for i in range(len(pts) - 1))
    print(f'{name}: goal=({gx:.2f},{gy:.2f}) status={res.status} (4=ok) path_len={L:.2f} m')
import os; os._exit(0)
