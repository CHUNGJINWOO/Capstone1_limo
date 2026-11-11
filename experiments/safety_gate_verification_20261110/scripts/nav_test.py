#!/usr/bin/env python3
"""Full-sim checks: Nav2 goal driving through the gate, optional fault injection mid-drive,
and behavior_server Spin routing.
usage: nav_test.py goal [--fault depth|lidar] [--dist D]
       nav_test.py spin
       nav_test.py reset_rearm"""
import json, math, subprocess, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose, Spin
import tf2_ros

FAULT_CMD = {
    'depth': ['pkill', '-9', '-f', '[d]epth_obstacle_filter_node'],
    'lidar': ['pkill', '-9', '-f', '[l]idar_scan_matcher_odom_node'],
    'est_odom': ['pkill', '-9', '-f', '[o]dom_tf_relay_node'],
    'wheel_odom': ['pkill', '-9', '-f', '[s]ensor_normalizer_node'],
}


class T(Node):
    def __init__(self):
        super().__init__('nav_test')
        self.map = None; self.gt = None; self.state = None; self.states = []
        self.out_nz = 0; self.out_n = 0
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m), q)
        self.create_subscription(Odometry, '/ground_truth/world_odom', lambda m: setattr(self, 'gt', m), 10)
        self.create_subscription(String, '/stall_detector/status_json', self.cb_s, 50)
        self.create_subscription(Twist, '/cmd_vel', self.cb_out, 100)
        self.tfb = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_ac = ActionClient(self, Spin, 'spin')
        self.reset_cli = self.create_client(Trigger, '/stall_detector/reset_latch')

    def cb_s(self, m):
        s = json.loads(m.data)['state']
        if s != self.state:
            self.states.append((round(self.now(), 2), s))
        self.state = s

    def cb_out(self, m):
        self.out_n += 1
        self.out_nz += (abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.01)

    def gt_xyyaw(self):
        p = self.gt.pose.pose; q = p.orientation
        return p.position.x, p.position.y, math.degrees(2 * math.atan2(q.z, q.w))

    def robot_in_map(self):
        t = None
        for _ in range(100):                      # TF buffer may still be filling right after start-up
            try:
                t = self.tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time()); break
            except Exception:
                self.spin_for(0.1)
        if t is None:
            t = self.tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        q = t.transform.rotation
        return t.transform.translation.x, t.transform.translation.y, 2 * math.atan2(q.z, q.w)

    def pick_goal(self, dist):
        m = self.map; info = m.info
        g = np.array(m.data, dtype=np.int16).reshape(info.height, info.width)
        rx, ry, ryaw = self.robot_in_map()
        res = info.resolution; ox, oy = info.origin.position.x, info.origin.position.y
        clear = int(math.ceil(0.30 / res))
        best = None
        for d in np.arange(dist, 0.39, -0.1):
            for da in np.radians([0, 20, -20, 40, -40, 60, -60, 90, -90, 120, -120, 180]):
                gx = rx + d * math.cos(ryaw + da); gy = ry + d * math.sin(ryaw + da)
                ci = int((gx - ox) / res); cj = int((gy - oy) / res)
                if clear <= ci < info.width - clear and clear <= cj < info.height - clear:
                    win = g[cj - clear:cj + clear + 1, ci - clear:ci + clear + 1]
                    if (win == 0).all():
                        best = (gx, gy, ryaw + da, d); break
            if best:
                break
        return best

    def send_goal(self, gx, gy, yaw):
        self.nav.wait_for_server(timeout_sec=10.0)
        goal = NavigateToPose.Goal(); ps = PoseStamped(); ps.header.frame_id = 'map'
        ps.pose.position.x = gx; ps.pose.position.y = gy
        ps.pose.orientation.z = math.sin(yaw / 2); ps.pose.orientation.w = math.cos(yaw / 2)
        goal.pose = ps
        f = self.nav.send_goal_async(goal)
        while not f.done():
            rclpy.spin_once(self, timeout_sec=0.01)
        return f.result()


def main():
    rclpy.init(); n = T(); mode = sys.argv[1]
    fault = sys.argv[sys.argv.index('--fault') + 1] if '--fault' in sys.argv else None
    dist = float(sys.argv[sys.argv.index('--dist') + 1]) if '--dist' in sys.argv else 1.2
    n.spin_for(3.0)
    print(f'gate state at start: {n.state}')
    if mode == 'reset_rearm':
        n.reset_cli.wait_for_service(timeout_sec=5)
        f = n.reset_cli.call_async(Trigger.Request())
        while not f.done():
            rclpy.spin_once(n, timeout_sec=0.01)
        print('reset:', f.result().message)
        n.spin_for(3.0); print('state transitions:', n.states); return
    x0, y0, yaw0 = n.gt_xyyaw()
    if mode == 'spin':
        n.spin_ac.wait_for_server(timeout_sec=10.0)
        g = Spin.Goal(); g.target_yaw = 1.57
        f = n.spin_ac.send_goal_async(g)
        while not f.done():
            rclpy.spin_once(n, timeout_sec=0.01)
        h = f.result(); print('spin accepted:', h.accepted)
        r = h.get_result_async()
        t0 = time.monotonic()
        while not r.done() and time.monotonic() - t0 < 25:
            rclpy.spin_once(n, timeout_sec=0.01)
        n.spin_for(1.0)
        x1, y1, yaw1 = n.gt_xyyaw()
        st = r.result().status if r.done() else 'timeout'
        print(f'spin status={st} GT yaw {yaw0:.1f} -> {yaw1:.1f} (delta {((yaw1-yaw0+180)%360)-180:.1f} deg); '
              f'/cmd_vel nonzero={n.out_nz}/{n.out_n}; states={n.states}')
        return
    best = n.pick_goal(dist)
    if not best:
        print('no free goal found'); return
    gx, gy, gyaw, d = best
    print(f'goal map=({gx:.2f},{gy:.2f}) yaw={math.degrees(gyaw):.0f} dist={d:.2f}')
    h = n.send_goal(gx, gy, gyaw); print('goal accepted:', h.accepted)
    r = h.get_result_async()
    t0 = time.monotonic(); faulted = False; path = 0.0; last = (x0, y0); t_fault = None; pos_fault = None
    while not r.done() and time.monotonic() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.01)
        x, y, _ = n.gt_xyyaw(); path += math.hypot(x - last[0], y - last[1]); last = (x, y)
        if fault and not faulted and path > 0.25:
            subprocess.run(FAULT_CMD[fault]); faulted = True; t_fault = time.monotonic()
            print(f'** fault injected ({fault}) after {path:.2f} m, state={n.state}')
        if faulted and pos_fault is None and time.monotonic() - t_fault > 2.0:
            pos_fault = (x, y)
            n.spin_for(3.0)
            x2, y2, _ = n.gt_xyyaw()
            print(f'   moved {math.hypot(x2-x, y2-y):.4f} m during 2..5 s after fault; state={n.state}')
            break
    n.spin_for(0.5)
    x1, y1, yaw1 = n.gt_xyyaw()
    st = r.result().status if r.done() else 'not finished'
    rx, ry, _ = n.robot_in_map() if fault != 'lidar' else (float('nan'),) * 3
    print(f'nav status={st} (4=SUCCEEDED); GT path={path:.2f} m, net={math.hypot(x1-x0, y1-y0):.2f} m; '
          f'map pose end=({rx:.2f},{ry:.2f}) goal err={math.hypot(rx-gx, ry-gy):.2f} m')
    print(f'/cmd_vel nonzero={n.out_nz}/{n.out_n}; state transitions={n.states}')
    if not r.done():
        h.cancel_goal_async(); n.spin_for(1.0)


main()
import os; sys.stdout.flush(); os._exit(0)
