#!/usr/bin/env python3
"""Passive monitor for an integrated auto-mapping simulation run (never publishes velocity,
never calls the gate's reset service).

Records at 1 Hz / 5 s: map known/free/occupied area, SLAM-vs-ground-truth pose error, GT path
length, Nav2 goal outcomes, safety gate state + counters, stream gaps (sim time), RTF, CPU.
Writes <out>/timeseries.csv, <out>/summary.json (refreshed every 30 s), map snapshots and the final
map (<out>/final_map.npy + .pgm + .yaml-like meta) so map quality can be evaluated offline.

usage: auto_mapping_monitor.py --out DIR --launch-log FILE [--max-wall SEC] [--tag NAME]
Stop reasons: wall_cap | exploration_stopped | gate_failsafe_held | no_goal_started |
              no_map_growth | robot_stuck | sigterm
Note: explore_lite re-sends a goal every ~2 s, and Nav2 reports the replaced goal as ABORTED, so the
action-status 'aborted' count includes preemptions; real failures are counted from the launch log
(see log_counts in summary.json).
"""
import argparse, csv, json, math, os, re, signal, sys, time
import numpy as np
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
import tf2_ros

STREAMS = {  # topic -> (type, qos)
    '/scan': (LaserScan, qos_profile_sensor_data),
    '/odometry/lidar_odom': (Odometry, 20),
    '/odom': (Odometry, 20),
    '/odom_nav2': (Odometry, 20),
    '/depth_obstacles/points': (PointCloud2, qos_profile_sensor_data),
}
GROUPS = [  # (name, substrings in cmdline)
    ('gazebo', ['ign gazebo', 'ruby']),
    ('slam_toolbox', ['async_slam_toolbox_node']),
    ('nav2', ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'smoother_server',
              'velocity_smoother', 'lifecycle_manager', 'waypoint_follower']),
    ('explore_lite', ['/explore/explore', 'explore_node']),
    ('safety_gate', ['stall_detector_node']),
    ('estimator_f4', ['lidar_scan_matcher_odom_node']),
    ('depth_filter', ['depth_obstacle_filter_node', 'depth_to_pointcloud']),
    ('ros_gz_bridge', ['parameter_bridge']),
]
FINAL = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled',
         GoalStatus.STATUS_ABORTED: 'aborted'}
BAD_STATES = ('FAILSAFE_SENSOR_TIMEOUT', 'FAILSAFE_ICP_DEGRADED', 'STALL_LATCHED')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Mon(Node):
    def __init__(self, a):
        super().__init__('auto_mapping_monitor', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.a = a
        os.makedirs(a.out, exist_ok=True)
        self.t_wall0 = time.monotonic()
        self.clock_pts = []          # (wall, sim)
        self.sim = 0.0
        self.map = None; self.map_n = 0
        self.gt = None; self.gt0 = None; self.gt_path = 0.0; self.gt_last = None
        self.gt_still_since = None
        self.tf0 = None
        self.gate = {}; self.gate_state = None; self.gate_trans = []
        self.gate_bad_since = None
        self.goals = {}              # uuid -> last status
        self.goal_first_seen = {}
        self.final_counts = {'succeeded': 0, 'canceled': 0, 'aborted': 0}
        self.goal_log = []           # (sim_t, uuid8, outcome)
        self.abort_streak = 0; self.max_abort_streak = 0
        self.first_goal_wall = None
        self.last = {k: None for k in STREAMS}
        self.gaps = {k: {'n': 0, 'max': 0.0, 'over_0p38': 0, 'over_1p0': 0} for k in STREAMS}
        self.min_scan = 10.0; self.min_scan_series_min = 10.0
        self.cmd_raw_n = 0; self.cmd_raw_nz = 0; self.cmd_n = 0; self.cmd_nz = 0
        self.cmd_vel_pubs = None
        self.map_hist = []           # (sim, known_cells)
        self.rows = []
        self.cpu_proc = {}           # pid -> Process
        self.cpu_acc = {g[0]: [] for g in GROUPS}; self.cpu_acc['other_ros'] = []; self.cpu_sys = []
        self.rtf_series = []
        self.stop_reason = None
        self.map_err = []            # (sim, err_xy_m, err_yaw_deg)
        self.snap_t = 0.0
        q_map = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Clock, '/clock', self.cb_clock, 50)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, q_map)
        self.create_subscription(Odometry, '/ground_truth/world_odom', self.cb_gt, 20)
        self.create_subscription(String, '/stall_detector/status_json', self.cb_gate, 50)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.cb_goal, 20)
        self.create_subscription(Twist, '/cmd_vel_raw', self.cb_raw, 50)
        self.create_subscription(Twist, '/cmd_vel', self.cb_out, 50)
        self.create_subscription(LaserScan, '/scan', self.cb_scan_min, qos_profile_sensor_data)
        for k, (typ, q) in STREAMS.items():
            self.create_subscription(typ, k, lambda m, k=k: self.cb_stream(k), q)
        self.tfb = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.create_timer(1.0, self.tick1)
        self.create_timer(5.0, self.tick5)
        self.create_timer(30.0, self.write_summary)
        self.csv = open(os.path.join(a.out, 'timeseries.csv'), 'w', newline='')
        self.w = csv.writer(self.csv)
        self.w.writerow(['wall_s', 'sim_s', 'rtf_5s', 'gate_state', 'known_m2', 'free_m2', 'occ_m2',
                         'gt_x', 'gt_y', 'gt_yaw_deg', 'gt_path_m', 'slam_err_xy_m', 'slam_err_yaw_deg',
                         'goals_ok', 'goals_cancel', 'goals_abort', 'goal_active', 'min_scan_m',
                         'cpu_sys_pct', 'load1'])

    # ---- callbacks -----------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_clock(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9
        self.clock_pts.append((time.monotonic(), self.sim))
        if len(self.clock_pts) > 4000:
            self.clock_pts = self.clock_pts[-2000:]

    def cb_map(self, m):
        self.map = m; self.map_n += 1

    def cb_gt(self, m):
        self.gt = m
        p = m.pose.pose.position
        if self.gt0 is None:
            self.gt0 = (p.x, p.y, yaw_of(m.pose.pose.orientation), self.now())
        if self.gt_last is not None:
            d = math.hypot(p.x - self.gt_last[0], p.y - self.gt_last[1])
            if d < 1.0:
                self.gt_path += d
            if d > 0.002:
                self.gt_still_since = None
        if self.gt_still_since is None:
            self.gt_still_since = self.now()
        self.gt_last = (p.x, p.y)

    def cb_gate(self, m):
        try:
            d = json.loads(m.data)
        except Exception:
            return
        self.gate = d
        s = d.get('state')
        if s != self.gate_state:
            self.gate_trans.append((round(d.get('sim_time', self.sim), 2), s))
            self.gate_state = s
        if s in BAD_STATES:
            if self.gate_bad_since is None:
                self.gate_bad_since = time.monotonic()
        else:
            self.gate_bad_since = None

    def cb_goal(self, m):
        for st in m.status_list:
            u = bytes(st.goal_info.goal_id.uuid).hex()[:8]
            if u not in self.goal_first_seen:
                self.goal_first_seen[u] = self.sim
                if self.first_goal_wall is None:
                    self.first_goal_wall = time.monotonic()
            prev = self.goals.get(u)
            self.goals[u] = st.status
            if st.status in FINAL and prev != st.status and prev not in FINAL:
                k = FINAL[st.status]
                self.final_counts[k] += 1
                self.goal_log.append((round(self.sim, 1), u, k, round(self.sim - self.goal_first_seen[u], 1)))
                if k == 'aborted':
                    self.abort_streak += 1; self.max_abort_streak = max(self.max_abort_streak, self.abort_streak)
                elif k == 'succeeded':
                    self.abort_streak = 0

    def cb_raw(self, m):
        self.cmd_raw_n += 1; self.cmd_raw_nz += (abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3)

    def cb_out(self, m):
        self.cmd_n += 1; self.cmd_nz += (abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3)

    def cb_scan_min(self, m):
        r = [x for x in m.ranges if m.range_min <= x <= m.range_max]
        if r:
            self.min_scan = min(r); self.min_scan_series_min = min(self.min_scan_series_min, self.min_scan)

    def cb_stream(self, k):
        t = self.now()
        last = self.last[k]
        if last is not None:
            g = t - last; s = self.gaps[k]
            s['n'] += 1; s['max'] = max(s['max'], g)
            s['over_0p38'] += g > 0.38; s['over_1p0'] += g > 1.0
        self.last[k] = t

    # ---- map helpers ---------------------------------------------------------------
    def map_stats(self):
        if self.map is None:
            return None
        info = self.map.info
        a = np.asarray(self.map.data, dtype=np.int16)
        cell = info.resolution ** 2
        occ = int((a >= 65).sum()); free = int(((a >= 0) & (a < 25)).sum()); unk = int((a < 0).sum())
        return dict(known=(occ + free) * cell, free=free * cell, occ=occ * cell, unknown=unk * cell,
                    w=info.width, h=info.height, res=info.resolution,
                    ox=info.origin.position.x, oy=info.origin.position.y)

    def slam_pose(self):
        try:
            t = self.tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        return t.transform.translation.x, t.transform.translation.y, yaw_of(q)

    def slam_err(self):
        """Pose error of SLAM (map->base_footprint) vs ground truth, after aligning both at t0."""
        sp = self.slam_pose()
        if sp is None or self.gt is None:
            return None
        p = self.gt.pose.pose; gx, gy, gyaw = p.position.x, p.position.y, yaw_of(p.orientation)
        if self.tf0 is None:
            self.tf0 = dict(slam=sp, gt=(gx, gy, gyaw))
            # world<-map rigid transform from the first simultaneous pair
            th = gyaw - sp[2]
            c, s = math.cos(th), math.sin(th)
            tx = gx - (c * sp[0] - s * sp[1]); ty = gy - (s * sp[0] + c * sp[1])
            self.tf0['align'] = dict(theta=th, tx=tx, ty=ty)
        al = self.tf0['align']
        c, s = math.cos(al['theta']), math.sin(al['theta'])
        wx = c * sp[0] - s * sp[1] + al['tx']; wy = s * sp[0] + c * sp[1] + al['ty']
        wyaw = sp[2] + al['theta']
        e = math.hypot(wx - gx, wy - gy)
        ey = math.degrees(math.atan2(math.sin(wyaw - gyaw), math.cos(wyaw - gyaw)))
        return e, ey

    # ---- timers --------------------------------------------------------------------
    def rtf_5s(self):
        if len(self.clock_pts) < 2:
            return None
        w1, s1 = self.clock_pts[-1]
        for w0, s0 in reversed(self.clock_pts):
            if w1 - w0 >= 5.0:
                return (s1 - s0) / (w1 - w0)
        return None

    def sample_cpu(self):
        self.cpu_sys.append(psutil.cpu_percent(interval=None))
        tot = {g[0]: 0.0 for g in GROUPS}; tot['other_ros'] = 0.0
        for p in psutil.process_iter(['pid', 'cmdline']):
            try:
                cl = ' '.join(p.info['cmdline'] or [])
            except Exception:
                continue
            if not cl or not ('/opt/ros' in cl or 'capstone_ws' in cl or 'ign' in cl or 'ruby' in cl):
                continue
            pid = p.info['pid']
            pr = self.cpu_proc.get(pid)
            if pr is None:
                pr = psutil.Process(pid); pr.cpu_percent(None); self.cpu_proc[pid] = pr; continue
            try:
                c = pr.cpu_percent(None)
            except Exception:
                continue
            for name, subs in GROUPS:
                if any(x in cl for x in subs):
                    tot[name] += c; break
            else:
                if 'ros' in cl:
                    tot['other_ros'] += c
        for k, v in tot.items():
            self.cpu_acc[k].append(v)

    def tick1(self):
        if self.cmd_vel_pubs is None and time.monotonic() - self.t_wall0 > 30:
            infos = self.get_publishers_info_by_topic('/cmd_vel')
            self.cmd_vel_pubs = [i.node_name for i in infos]
        # stop conditions
        wall = time.monotonic() - self.t_wall0
        if not self.stop_reason:
            if wall > self.a.max_wall:
                self.stop_reason = 'wall_cap'
            elif self.gate_bad_since is not None and time.monotonic() - self.gate_bad_since > 15:
                self.stop_reason = 'gate_failsafe_held'
            elif self.first_goal_wall is None and wall > 240:
                self.stop_reason = 'no_goal_started'
            elif self.no_map_growth():
                self.stop_reason = 'no_map_growth'
            elif (self.first_goal_wall is not None and self.gt_still_since is not None
                  and self.now() - self.gt_still_since > 240 and wall > 300):
                self.stop_reason = 'robot_stuck'
            elif self.log_says_exploration_done():
                self.stop_reason = 'exploration_stopped'
        if self.stop_reason:
            self.finish()

    _done_wall = None

    def no_map_growth(self):
        # < 0.5 m^2 of new known area during the last 360 s of sim time (after 600 s)
        if self.sim < 600 or not self.map_hist:
            return False
        ref = [k for t, k in self.map_hist if t <= self.sim - 360]
        return bool(ref) and (self.map_hist[-1][1] - ref[-1]) < 0.5

    def log_text(self):
        try:
            return open(self.a.launch_log, 'r', errors='ignore').read()
        except Exception:
            return ''

    def log_says_exploration_done(self):
        txt = self.log_text()
        if re.search(r'No frontiers found, stopping|All frontiers traversed/tried out, stopping', txt):
            if self._done_wall is None:
                self._done_wall = time.monotonic()
            # allow the (gated) return-to-initial-pose to finish, but not forever
            return ('Successfully returned to initial pose' in txt) or (time.monotonic() - self._done_wall > 150)
        return False

    def log_counts(self):
        txt = self.log_text()
        pats = {
            'goals_sent (Begin navigating)': r'Begin navigating from current location',
            'goal_preemptions': r'Received goal preemption request',
            'controller_failed_to_make_progress': r'Failed to make progress',
            'planner_no_valid_path': r'failed to generate a valid path|Failed to create a plan',
            'collision_ahead': r'Collision Ahead',
            'behavior_recovery_run': r'Running (spin|backup|wait|drive_on_heading)',
            'navigation_succeeded_log': r'Goal succeeded|Navigation succeeded',
            'fail_safe_log_lines': r'FAIL-SAFE',
            'stall_latched_log_lines': r'STALL LATCHED',
            'tf_wait_timeouts': r'Timed out waiting for transform',
            'message_filter_drops': r'Message Filter dropping',
            'tf_extrapolation_errors': r'extrapolation',
            'explore_no_frontiers_stop': r'No frontiers found, stopping|All frontiers traversed/tried out, stopping',
            'explore_returned_to_initial_pose': r'Successfully returned to initial pose',
            'depth_filter_errors': r'DEPTH FILTER/TF ERROR',
            'depth_sensor_stale': r'DEPTH SENSOR STALE',
            'costmap_errors': r'\[ERROR\].*costmap',
            'process_died': r'process has died',
        }
        return {k: len(re.findall(v, txt)) for k, v in pats.items()}

    def tick5(self):
        st = self.map_stats()
        self.sample_cpu()
        rtf = self.rtf_5s()
        if rtf is not None:
            self.rtf_series.append(rtf)
        er = self.slam_err()
        if er and st:
            self.map_err.append((round(self.sim, 1), er[0], er[1]))
        if st:
            self.map_hist.append((self.sim, st['known']))
            if self.sim - self.snap_t >= 120:
                self.snap_t = self.sim
                self.save_map(f'map_{int(self.sim):05d}')
        p = self.gt.pose.pose if self.gt else None
        active = sum(1 for v in self.goals.values() if v in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING))
        self.w.writerow([round(time.monotonic() - self.t_wall0, 1), round(self.sim, 1),
                         round(rtf, 3) if rtf else '', self.gate_state,
                         round(st['known'], 2) if st else '', round(st['free'], 2) if st else '', round(st['occ'], 2) if st else '',
                         round(p.position.x, 3) if p else '', round(p.position.y, 3) if p else '',
                         round(math.degrees(yaw_of(p.orientation)), 1) if p else '', round(self.gt_path, 3),
                         round(er[0], 3) if er else '', round(er[1], 2) if er else '',
                         self.final_counts['succeeded'], self.final_counts['canceled'], self.final_counts['aborted'], active,
                         round(self.min_scan, 3), self.cpu_sys[-1] if self.cpu_sys else '', os.getloadavg()[0]])
        self.csv.flush()

    def save_map(self, name):
        if self.map is None:
            return
        info = self.map.info
        a = np.asarray(self.map.data, dtype=np.int8).reshape(info.height, info.width)
        np.save(os.path.join(self.a.out, name + '.npy'), a)
        img = np.full(a.shape, 205, np.uint8); img[(a >= 0) & (a < 25)] = 254; img[a >= 65] = 0
        with open(os.path.join(self.a.out, name + '.pgm'), 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (info.width, info.height)); f.write(np.flipud(img).tobytes())
        meta = dict(resolution=info.resolution, origin=[info.origin.position.x, info.origin.position.y],
                    width=info.width, height=info.height, align_world_from_map=(self.tf0 or {}).get('align'))
        json.dump(meta, open(os.path.join(self.a.out, name + '.json'), 'w'))

    def summary(self):
        st = self.map_stats() or {}
        def stat(v):
            v = [x for x in v if x is not None]
            return dict(mean=round(float(np.mean(v)), 2), max=round(float(np.max(v)), 2)) if v else None
        el = [e[1] for e in self.map_err]
        gd = {k: {kk: (round(vv, 3) if isinstance(vv, float) else vv) for kk, vv in v.items()} for k, v in self.gaps.items()}
        g = self.gate
        rtf = [r for r in self.rtf_series]
        return dict(
            tag=self.a.tag, stop_reason=self.stop_reason, wall_s=round(time.monotonic() - self.t_wall0, 1),
            sim_s=round(self.sim, 1), gate_transitions=self.gate_trans,
            gate_counters={k: g.get(k) for k in ('failsafe_events', 'stall_events', 'icp_degraded_events', 'operator_unlatch_count')},
            gate_last_flags={k: g.get(k) for k in ('state', 'depth_stale', 'wheel_odom_stale', 'est_odom_stale', 'depth_code')},
            cmd_vel_publishers=self.cmd_vel_pubs,
            cmd_raw_nonzero=f'{self.cmd_raw_nz}/{self.cmd_raw_n}', cmd_out_nonzero=f'{self.cmd_nz}/{self.cmd_n}',
            map=st, map_updates=self.map_n,
            map_growth_sim=[(round(t, 0), round(k, 1)) for t, k in self.map_hist[::max(1, len(self.map_hist) // 40)]],
            gt_path_m=round(self.gt_path, 2),
            slam_err_xy_m=dict(mean=round(float(np.mean(el)), 3), max=round(float(np.max(el)), 3), last=round(el[-1], 3)) if el else None,
            goals=dict(self.final_counts, total_seen=len(self.goal_first_seen), max_abort_streak=self.max_abort_streak,
                       active_now=sum(1 for v in self.goals.values() if v in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING))),
            goal_log=self.goal_log,
            stream_gaps_sim=gd, min_scan_m=round(self.min_scan_series_min, 3),
            rtf=dict(mean=round(float(np.mean(rtf)), 3), min=round(float(np.min(rtf)), 3), p10=round(float(np.percentile(rtf, 10)), 3)) if rtf else None,
            cpu_pct_of_one_core={k: stat(v) for k, v in self.cpu_acc.items()},
            cpu_system_pct=stat(self.cpu_sys), n_cores=psutil.cpu_count(),
            align=(self.tf0 or {}).get('align'),
            log_counts=self.log_counts(),
        )

    def write_summary(self):
        json.dump(self.summary(), open(os.path.join(self.a.out, 'summary.json'), 'w'), indent=1, default=str)

    def finish(self):
        self.save_map('final_map'); self.write_summary(); self.csv.close()
        print('MONITOR_DONE', self.stop_reason, flush=True)
        os._exit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True); ap.add_argument('--launch-log', required=True)
    ap.add_argument('--max-wall', type=float, default=1200.0); ap.add_argument('--tag', default='run')
    a = ap.parse_args()
    rclpy.init(); n = Mon(a)
    signal.signal(signal.SIGTERM, lambda *_: (setattr(n, 'stop_reason', 'sigterm'), n.finish()))
    rclpy.spin(n)


main()
