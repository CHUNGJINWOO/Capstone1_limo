"""SIMULATION ONLY (2026-11-22): one-time-axis recorder for the isolated Gazebo web harness (run_gazebo_web.py, GZWEB_TIMELINE=1).

Writes JSON lines {"k": kind, "m": monotonic_s, "r": ros_time_s|None, ...} to argv[1]:
  scan    /scan arrival, header stamp, front min range (+-20 deg)          clock  /clock sim time
  locked  Gate output /cmd_vel_locked                                       drive  Gazebo bridge input /sim/drive_cmd
  truth   /e2e/ground_truth (spawn frame, 10 Hz)                            gate   Gate status (state, cmd in/out, wheel/lidar speed,
  wd      Motion Watchdog status (state, epoch)                                    front scan, cmd_input, fault reasons) on change or 2 Hz
  nav2    lifecycle state of each Nav2 server (1 Hz, get_state, 0.5 s timeout)
  http    Fleet robot state: mode, Arbiter source/blocker/selected, Watchdog, Gate, readiness level + reasons (2 Hz)
  load    /proc/loadavg and this process's 20 Hz timer lateness (callback latency proxy)
Never subscribes to /cmd_vel_raw or /fleet/nav_cmd_vel (the Arbiter's locked-graph contract allows exactly one reader there).
Read-only: no publisher, no service other than lifecycle get_state.
Usage: ROS_DOMAIN_ID=<d> python3 timeline_recorder.py <out.jsonl> <server_url> <operator_token_file> [robot_id]
"""
import json, math, os, sys, threading, time, urllib.request

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.srv import GetState

OUT, URL, TOKF = sys.argv[1], sys.argv[2], sys.argv[3]
RID = sys.argv[4] if len(sys.argv) > 4 else 'iso_bench'
NAV2 = ('controller_server', 'planner_server', 'smoother_server', 'behavior_server', 'bt_navigator', 'velocity_smoother',
        'map_server', 'amcl')
F = open(OUT, 'a', buffering=1); WL = threading.Lock(); CLOCK = [None]


def w(k, **d):
    d = {'k': k, 'm': round(time.monotonic(), 4), 'r': CLOCK[0], **d}
    with WL: F.write(json.dumps(d, separators=(',', ':')) + '\n')


rclpy.init(); n = rclpy.create_node('gz_timeline_observer'); cg = ReentrantCallbackGroup()   # 'observer': harness-owned reader in g00's graph check


def on_clock(m): CLOCK[0] = round(m.clock.sec + m.clock.nanosec * 1e-9, 4)


_clk = [0.0]
def on_clock_log(m):
    on_clock(m)
    if time.monotonic() - _clk[0] >= 1.0: _clk[0] = time.monotonic(); w('clock')


def on_scan(m):
    rs = [r for i, r in enumerate(m.ranges) if abs(m.angle_min + i * m.angle_increment) <= math.radians(20) and math.isfinite(r) and r > m.range_min]
    w('scan', stamp=round(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 4), front=round(min(rs), 3) if rs else None)


_tr = [0.0]
def on_truth(m):
    if time.monotonic() - _tr[0] < .1: return
    _tr[0] = time.monotonic(); w('truth', p=[round(v, 4) for v in m.data])


_gl = [None, 0.0]
def on_gate(m):
    try: d = json.loads(m.data)
    except ValueError: return
    row = {k: d.get(k) for k in ('state', 'cmd_blocked', 'cmd_vx_in', 'cmd_vx_out', 'wheel_vx', 'lidar_speed', 'min_front_scan_m',
                                 'fault_reasons', 'fault_latched', 'stall_events', 'sim_time')}
    ci = d.get('cmd_input') or {}
    row['cmd_input'] = {k: ci.get(k) for k in ('state', 'writers', 'age_s', 'stamp_age_s', 'last_rx_age_s') if k in ci} if isinstance(ci, dict) else ci
    key = (row['state'], row['cmd_blocked'], tuple(row['fault_reasons'] or []))
    if key != _gl[0] or time.monotonic() - _gl[1] >= .5:
        _gl[0], _gl[1] = key, time.monotonic(); w('gate', **row)


def on_wd(m):
    try: d = json.loads(m.data)
    except ValueError: return
    w('wd', state=d.get('state'), epoch=d.get('epoch'), reason=d.get('reason') or d.get('last_reason'))


n.create_subscription(Clock, '/clock', on_clock_log, 10, callback_group=cg)
n.create_subscription(LaserScan, '/scan', on_scan, 10, callback_group=cg)
n.create_subscription(Twist, '/cmd_vel_locked', lambda m: w('locked', vx=round(m.linear.x, 4), wz=round(m.angular.z, 4)), 50, callback_group=cg)
n.create_subscription(Twist, '/sim/drive_cmd', lambda m: w('drive', vx=round(m.linear.x, 4), wz=round(m.angular.z, 4)), 50, callback_group=cg)
n.create_subscription(Float64MultiArray, '/e2e/ground_truth', on_truth, 50, callback_group=cg)
n.create_subscription(String, '/stall_detector/status_json', on_gate, 20, callback_group=cg)
n.create_subscription(String, '/fleet/motion_watchdog/status', on_wd, 10, callback_group=cg)


# 2026-11-22: localization vs ground truth (g05 analysis). AMCL pose as published; ICP odometry decimated to ~5 Hz.
def _yaw(q): return round(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), 4)
def on_amcl(m):
    p = m.pose.pose; c = m.pose.covariance
    w('amcl', x=round(p.position.x, 4), y=round(p.position.y, 4), th=_yaw(p.orientation), cxx=round(c[0], 5), cyy=round(c[7], 5), cth=round(c[35], 5))
_icp_t = [0.0]
def on_icp(m):
    if time.monotonic() - _icp_t[0] < .2: return
    _icp_t[0] = time.monotonic(); p = m.pose.pose
    w('icp', x=round(p.position.x, 4), y=round(p.position.y, 4), th=_yaw(p.orientation))
n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', on_amcl, 10, callback_group=cg)
n.create_subscription(Odometry, '/odometry/lidar_odom', on_icp, 10, callback_group=cg)

CLI = {s: n.create_client(GetState, f'/{s}/get_state', callback_group=cg) for s in NAV2}
STOP = threading.Event()


def nav2_loop():
    while not STOP.is_set():
        t0 = time.monotonic(); st = {}
        for s, c in CLI.items():
            if not c.service_is_ready(): st[s] = None; continue
            f = c.call_async(GetState.Request()); end = time.monotonic() + .5
            while not f.done() and time.monotonic() < end: time.sleep(.01)
            st[s] = f.result().current_state.label if f.done() and f.result() else 'TIMEOUT'
        w('nav2', s=st); STOP.wait(max(0., 1. - (time.monotonic() - t0)))


def http_loop():
    tok = open(TOKF).read().strip()
    def get(p):
        rq = urllib.request.Request(URL + '/api/v1' + p, headers={'Authorization': 'Bearer ' + tok})
        with urllib.request.urlopen(rq, timeout=2) as r: return json.loads(r.read())
    while not STOP.is_set():
        t0 = time.monotonic()
        try:
            rb = get(f'/robots/{RID}'); s = rb.get('state') or {}
            arb = s.get('velocity_arbiter') or {}; g = s.get('safety_gate') or {}; wd = s.get('motion_watchdog') or {}
            row = {'online': rb.get('online'), 'mode': (s.get('operating_mode') or {}).get('mode') if isinstance(s.get('operating_mode'), dict) else s.get('operating_mode'),
                   'arb': {k: arb.get(k) for k in ('source', 'blocker', 'selected', 'linear', 'angular', 'last_nonzero_age_s') if k in arb},
                   'gate': g.get('status'), 'gate_reasons': g.get('movement_block_reasons'), 'wd': wd.get('state'),
                   'mw': {k: (s.get('mapping_worker') or {}).get(k) for k in ('status', 'stop_reason', 'attempted_goals')}}
            try:
                rd = get(f'/robots/{RID}/readiness'); row['ready'] = rd.get('level'); row['ready_reasons'] = (rd.get('reasons') or [])[:4]
                row['motion_permitted'] = rd.get('motion_permitted')
            except Exception as e: row['ready'] = 'ERR:' + type(e).__name__
            w('http', **row)
        except Exception as e: w('http', err=repr(e)[:120])
        STOP.wait(max(0., .5 - (time.monotonic() - t0)))


_lt = [time.monotonic(), 0.0, 0]
def tick():
    now = time.monotonic(); late = now - _lt[0] - .05; _lt[0] = now
    _lt[1] = max(_lt[1], late); _lt[2] += 1
    if _lt[2] >= 20:
        try: la = open('/proc/loadavg').read().split()[:3]
        except OSError: la = None
        w('load', loadavg=la, timer_late_max_s=round(_lt[1], 4)); _lt[1], _lt[2] = 0.0, 0


n.create_timer(.05, tick, callback_group=cg)
w('start', pid=os.getpid(), domain=os.environ.get('ROS_DOMAIN_ID'), wall=round(time.time(), 4))
for fn in (nav2_loop, http_loop): threading.Thread(target=fn, daemon=True).start()
ex = MultiThreadedExecutor(num_threads=4); ex.add_node(n)
try: ex.spin()
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
finally:
    STOP.set(); w('stop'); time.sleep(.2)
    ex.shutdown(); n.destroy_node(); rclpy.try_shutdown()
