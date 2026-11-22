"""SIMULATION ONLY (2026-11-21): web control chain on the Gazebo (Ignition Fortress) LIMO model, isolated.

Chain under test: Web/HTTP -> Fleet Server -> Robot Agent (VelocityArbiter) -> Motion Watchdog -> Safety Gate r12 -> /cmd_vel_locked
-> ros_gz_bridge -> Gazebo DiffDrive (limo_eval_diff) in the Office world. ROS /cmd_vel has no endpoint; the real-robot launch,
port 18190 and the LIMO are not involved. Stack: tests/mode_e2e/stack_up.sh with SIM_BACKEND=gazebo NAV_LOCALIZATION=1.
Every number below is a Gazebo result (synthetic), not a real-LIMO measurement.

Usage: OVERLAY=<explore_lite overlay> GZWEB_OUT=<new dir> python3 run_gazebo_web.py <port> <domain> [--browser] [--only a,b]
Per event the harness records: the ROS command (/fleet/nav_cmd_vel, /cmd_vel_raw), the Gate output (/cmd_vel_locked), the Gazebo
ground-truth motion (/e2e/ground_truth from gz_truth_adapter) and the times from the event to Gate-zero and to Gazebo standstill.
"""
import json, math, os, signal, subprocess, sys, threading, time, traceback, uuid, urllib.parse, urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent; FC = HERE.parents[1]; REPO = FC.parent; MODE = FC / 'tests' / 'mode_e2e'
sys.path[:0] = [str(MODE), str(FC / 'tests'), str(FC / 'agent' / 'limo_robot_agent')]
from owned_proc import owned_pid                       # never a machine-wide pgrep
from e2e_common import Results, PROTECTED_PORTS

PORT, DOMAIN = int(sys.argv[1]), sys.argv[2]
BROWSER = '--browser' in sys.argv
ONLY = sys.argv[sys.argv.index('--only') + 1].split(',') if '--only' in sys.argv else None
assert PORT not in PROTECTED_PORTS and DOMAIN not in ('0', '77', '231')
OUT = Path(os.environ['GZWEB_OUT']).resolve(); OUT.mkdir(parents=True, exist_ok=False)
RUN = OUT / 'stack'; RID = 'iso_bench'; R = Results(OUT / 'harness.log'); say = R.say
G12 = REPO / 'experiments/safety_chain_fix_20261113/candidate_gate_r12/nodes/stall_detector_node.py'
# GATE_SCRIPT overrides the Gate (e.g. a candidate copy); GATE_EXTRA_PARAMS='name:=value,...' passes extra Gate parameters.
STACK_ENV = dict(os.environ, SIM_BACKEND='gazebo', NAV_LOCALIZATION='1', IGN_PARTITION=f'fcgz_{DOMAIN}_{os.getpid()}',
                 GATE_SCRIPT=os.environ.get('GATE_SCRIPT', str(G12)), GATE_INPUT_EXCL='fleet_motion_watchdog',
                 GATE_INPUT_MAX_VX='0.20', GATE_INPUT_MAX_WZ='0.60',
                 # 2026-11-21 odom probe (Gazebo alone, rotate 20 s @0.22 + 0.8 m): truth 2.68 rad / 0.61 m, wheel /odom 4.40 rad / 0.80 m,
                 # ICP 2.43 rad / 0.58 m. The DiffDrive odom integrates commanded wheel speed of a skidding model, so the odom->base TF
                 # relay uses the stack's own ICP odometry (as the verified 2026-11-13 Office run did). Gate inputs are unchanged.
                 ODOM_ACTIVE_TOPIC=os.environ.get('ODOM_ACTIVE_TOPIC', '/odometry/lidar_odom'),
                 GZ_DRIVE_TIMEOUT_S=os.environ.get('GZ_DRIVE_TIMEOUT_S', '0.2'))
t_up = time.monotonic()
subprocess.run([str(MODE / 'stack_up.sh'), str(RUN), str(PORT), DOMAIN], check=True, env=STACK_ENV,
               stdout=(OUT / 'stack_up.log').open('x'), stderr=subprocess.STDOUT)
os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=DOMAIN); os.environ.pop('ROS_DISCOVERY_SERVER', None)

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from limo_robot_agent.core import Http, HttpError

URL = f'http://127.0.0.1:{PORT}'
tok = lambda n: (RUN / 'secrets' / f'{n}.token').read_text().strip()
OP, ADMIN = Http(URL, tok('operator')), Http(URL, tok('admin'))
for h in (OP, ADMIN): h.timeout = 30.0
rclpy.init(); obs = rclpy.create_node('gz_web_observer'); rogue = rclpy.create_node('gz_web_rogue')
S = {k: [] for k in ('locked', 'raw', 'arb', 'truth', 'gate', 'scan', 'drive')}
DRIVE_TIMEOUT = float(os.environ.get('GZ_DRIVE_TIMEOUT_S', '0.2') or 0)     # passed to stack_up.sh -> gazebo_world.launch.py
truth, watch, clock = [None], [None], [None]
LOCK = threading.Lock()


def _add(k, row):
    with LOCK: S[k].append(row)


def _truth(m):
    row = (time.monotonic(), *m.data); truth[0] = row; _add('truth', row)


GATE_LAST = [None]


def _gate(m):
    try: d = json.loads(m.data)
    except ValueError: return
    GATE_LAST[0] = d
    if isinstance(d.get('cmd_vx_in'), (int, float)): _add('raw', (time.monotonic(), float(d['cmd_vx_in']), 0.0))
    key = (d.get('state'), tuple((d.get('fault_reasons') or [])[:2]))
    if not S['gate'] or S['gate'][-1][1:] != key: _add('gate', (time.monotonic(),) + key)


obs.create_subscription(Twist, '/cmd_vel_locked', lambda m: _add('locked', (time.monotonic(), m.linear.x, m.angular.z)), 50)
# what the Gazebo bridge receives when the Gate-independent drive timeout is on (drive_timeout_relay.py)
obs.create_subscription(Twist, '/sim/drive_cmd', lambda m: _add('drive', (time.monotonic(), m.linear.x, m.angular.z)), 50)
# NO observer subscription on /cmd_vel_raw or /fleet/nav_cmd_vel: the Arbiter's locked-graph contract allows exactly one reader
# there (Gate / Watchdog) and correctly refuses motion otherwise (2026-11-21 smoke run 2). The Gate's own input as it saw it
# (status cmd_vx_in, 10 Hz) is recorded instead as the upstream command.
obs.create_subscription(Float64MultiArray, '/e2e/ground_truth', _truth, 50)
obs.create_subscription(String, '/stall_detector/status_json', _gate, 20)
obs.create_subscription(String, '/fleet/motion_watchdog/status', lambda m: watch.__setitem__(0, json.loads(m.data)), 10)
obs.create_subscription(Clock, '/clock', lambda m: clock.__setitem__(0, (time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
TF_PAIRS = {}
obs.create_subscription(TFMessage, '/tf', lambda m: [TF_PAIRS.__setitem__((t.header.frame_id, t.child_frame_id), TF_PAIRS.get((t.header.frame_id, t.child_frame_id), 0) + 1) for t in m.transforms], 50)
LAST_SCAN = [None]          # (angle_min, angle_increment, ranges) for home(face='open')
obs.create_subscription(LaserScan, '/scan', lambda m: (_add('scan', (time.monotonic(),)), LAST_SCAN.__setitem__(0, (m.angle_min, m.angle_increment, list(m.ranges)))), 10)
from tf2_ros import Buffer, TransformListener
TFB = Buffer(); TFL = TransformListener(TFB, obs)      # read-only: map -> base_footprint at map save (truth <-> map frame offset)
ex = SingleThreadedExecutor(); ex.add_node(obs); ex.add_node(rogue); SPIN = threading.Thread(target=ex.spin, daemon=True); SPIN.start()
# 2026-11-21 (Gazebo integration): optional one-time-axis recorder in its own process (scan, clock, Nav2 lifecycle, Fleet/readiness,
# Arbiter, Watchdog, Gate in/out, bridge input, truth, load). Read-only; never on /cmd_vel_raw or /fleet/nav_cmd_vel.
TIMELINE = None
if os.environ.get('GZWEB_TIMELINE') == '1':
    TIMELINE = subprocess.Popen([sys.executable, str(HERE / 'timeline_recorder.py'), str(OUT / 'timeline.jsonl'), URL,
                                 str(RUN / 'secrets' / 'operator.token'), RID], stdout=(OUT / 'timeline.log').open('x'), stderr=subprocess.STDOUT)


# ------------------------------------------------------------------ HTTP helpers (same calls as the web UI / nav_e2e)
def get(path, who='operator'):
    req = urllib.request.Request(URL + '/api/v1' + urllib.parse.quote(path, safe='/?=&'), headers={'Authorization': 'Bearer ' + tok(who)})
    with urllib.request.urlopen(req, timeout=10) as r: return json.loads(r.read())


robot = lambda: get(f'/robots/{RID}')
state = lambda: robot().get('state') or {}
loc = lambda: state().get('localization') or {}
pose = lambda: list(truth[0][1:4]) if truth[0] else None
# 2026-11-21: Gazebo truth is in the SPAWN frame; a saved SLAM map is in its own 'map' frame, which follows the drifting ICP odometry
# (odom probe: ICP rotation ~9 % short). after_full_r1 g04: truth pose given as INITIAL_POSE was 0.48 m / 0.34 rad from where AMCL
# settled (match 0.28, CONVERGING). The operator places the robot where it is ON THE MAP, so the harness converts truth into the
# map frame with the rigid offset measured at save time (TF map->base_footprint vs truth, robot stationary). Identity if not measured.
T2M = {'dx': 0.0, 'dy': 0.0, 'dth': 0.0, 'measured': False}

def capture_map_offset(label):
    time.sleep(1.0); p = pose(); tr = None
    try: tr = TFB.lookup_transform('map', 'base_footprint', rclpy.time.Time())
    except Exception as e: T2M.update(measured=False, error=f'{label}: {type(e).__name__}'); return dict(T2M)
    q = tr.transform.rotation; mth = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    mx, my = tr.transform.translation.x, tr.transform.translation.y
    dth = math.atan2(math.sin(mth - p[2]), math.cos(mth - p[2])); c, s_ = math.cos(dth), math.sin(dth)
    T2M.update(dx=mx - (c * p[0] - s_ * p[1]), dy=my - (s_ * p[0] + c * p[1]), dth=dth, measured=True, at=label,
               truth=[round(v, 3) for v in p], map=[round(mx, 3), round(my, 3), round(mth, 3)])
    return dict(T2M)

def t2m(x, y, th=0.0):
    c, s_ = math.cos(T2M['dth']), math.sin(T2M['dth'])
    return c * x - s_ * y + T2M['dx'], s_ * x + c * y + T2M['dy'], math.atan2(math.sin(th + T2M['dth']), math.cos(th + T2M['dth']))

def mpose():
    p = pose(); return list(t2m(*p)) if p else None

mtraj = lambda: [tuple(round(v, 2) for v in t2m(x, y)[:2]) for x, y in TRAJ]


def wait_until(pred, timeout, step=.25):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            v = pred()
            if v: return v
        except Exception: pass
        time.sleep(step)
    return None


def expect_http(fn, status, code=None):
    try: fn()
    except HttpError as e: return e.status == status and (code is None or code in str(e)), str(e)[:240]
    return False, 'accepted'


EPOCH = [None]


def lease():
    EPOCH[0] = OP.post(f'/robots/{RID}/lock', {'action': 'acquire', 'ttl_s': 120})['epoch']; return EPOCH[0]


def renewer():
    while True:
        time.sleep(15)
        try: OP.post(f'/robots/{RID}/lock', {'action': 'renew', 'ttl_s': 120})
        except Exception: pass


threading.Thread(target=renewer, daemon=True).start()


def switch(target, timeout=60):
    OP.post(f'/robots/{RID}/mode', {'target': target, 'epoch': EPOCH[0], 'idempotency_key': 'g-' + uuid.uuid4().hex[:10]})
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        rec = get(f'/robots/{RID}/mode')['operating_mode'] or {}; t = rec.get('transition') or {}
        if t.get('to') == target and t.get('state') in ('ACTIVE', 'FAILED'): return rec
        time.sleep(.25)
    return get(f'/robots/{RID}/mode')['operating_mode']


SID = [None]


def mapping(action, timeout=60, **extra):
    body = {'action': action, 'epoch': EPOCH[0], 'idempotency_key': 'g-' + uuid.uuid4().hex[:10], **extra}
    if action != 'START': body['session_id'] = SID[0]
    r = OP.post(f'/robots/{RID}/mapping', body)
    if action == 'START': SID[0] = r['session_id']
    return wait_until(lambda: (lambda s: s if (s.get('command') or {}).get('command_id') == r['command_id']
                               and s['command']['status'] in ('COMPLETED', 'FAILED', 'UNKNOWN') else None)(get(f'/robots/{RID}/mapping')), timeout)


navreq = lambda action, **kw: OP.post(f'/robots/{RID}/navigation', {'action': action, 'epoch': EPOCH[0], **kw})
TERMINAL = ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED', 'RECONCILIATION_REQUIRED')


def mission(x, y):
    return OP.post('/missions', {'robot_id': RID, 'type': 'navigate_to_pose', 'goal': {'frame_id': 'map', 'x': x, 'y': y},
                                 'lease_epoch': EPOCH[0], 'idempotency_key': 'g-' + uuid.uuid4().hex})


def mstatus(mid):
    return next((m['status'] for m in get(f'/missions?robot_id={RID}&limit=30')['missions'] if m['mission_id'] == mid), None)


def wait_mission(mid, timeout=150):
    return wait_until(lambda: mstatus(mid) if mstatus(mid) in TERMINAL else None, timeout, .5) or mstatus(mid)


class Manual:
    """The manual bench HTTP contract the web panel uses (start / sample at ~12 Hz / stop)."""
    def __init__(self, linear=1., turn=0.): self.lin, self.turn, self.on, self.deadman, self.errors, self.seq = linear, turn, False, True, [], 0

    def start(self):
        self.session = OP.post(f'/robots/{RID}/manual-bench', {'action': 'start', 'epoch': EPOCH[0]})['session_id']
        self.on = True; threading.Thread(target=self.loop, daemon=True).start(); return self

    def sample(self, deadman=None, session=None, seq=None):
        self.seq += 1
        return OP.post(f'/robots/{RID}/manual-bench', {'action': 'sample', 'epoch': EPOCH[0], 'session_id': session or self.session,
                                                       'seq': seq or self.seq, 'linear': self.lin, 'turn': self.turn,
                                                       'deadman': self.deadman if deadman is None else deadman})

    def loop(self):
        while self.on:
            try: self.sample()
            except Exception as e: self.errors.append(str(e)[:120]); self.on = False
            time.sleep(.08)

    def stop(self, send=True):
        self.on = False; time.sleep(.1)
        if send:
            try: OP.post(f'/robots/{RID}/manual-bench', {'action': 'stop', 'epoch': EPOCH[0], 'session_id': self.session})
            except Exception: pass


# ------------------------------------------------------------------ measurement
nz = lambda v, w: abs(v) > 1e-6 or abs(w) > 1e-6


def rows(k, t0, t1=None):
    with LOCK: return [r for r in S[k] if r[0] >= t0 and (t1 is None or r[0] <= t1)]


def stop_metrics(t_ev, settle=2.5):
    """Wait `settle` s after the event, then: last non-zero Gate output, last non-zero upstream command, Gazebo standstill time
    (|v| < 0.01 m/s and |w| < 0.02 rad/s from then on) and the displacement from the event to the end of the window."""
    time.sleep(settle); t1 = time.monotonic()
    lk = rows('locked', t_ev, t1); rw = rows('raw', t_ev, t1); tr = rows('truth', t_ev - .3, t1)
    last = lambda xs: max((r[0] for r in xs if nz(r[1], r[2])), default=None)
    moving = [r for r in tr if r[0] >= t_ev and (abs(r[4]) > .01 or abs(r[5]) > .02)]
    p0 = next((r for r in reversed(tr) if r[0] <= t_ev), tr[0] if tr else None); p1 = tr[-1] if tr else None
    rel = lambda t: None if t is None else round(t - t_ev, 3)
    return {'gate_last_nonzero_s': rel(last(lk)), 'gate_samples': len(lk), 'gate_input_last_nonzero_s': rel(last(rw)),
            'gz_last_moving_s': rel(moving[-1][0] if moving else None),
            'gz_moved_after_event_m': None if not (p0 and p1) else round(math.hypot(p1[1] - p0[1], p1[2] - p0[2]), 3),
            'gz_speed_at_event': None if not p0 else round(p0[4], 3), 'gate_states': [(rel(g[0]), g[1], list(g[2])) for g in rows('gate', t_ev - .5, t1)][:6]}


def gate_now(): return (state().get('safety_gate') or {}).get('status')


GATE_EVENTS, RECOVERY_EVENTS, MARKS = [], [], []
GATE_GUARD_PERSISTENT_MAX = int(os.environ.get('GZWEB_GUARD_PERSISTENT_MAX', '5'))
HEALTH = {'broken_by': None, 'pre': {}}   # 2026-11-22: first failure vs cascade (classification only)


def gate_guard(label, timeout=75):        # run 3: a killed writer's DDS ghost kept the Gate reset refused for 40 s
    """Environment guard (as in nav_e2e): a latched Gate is cleared ONLY through its own operator unlatch service; every use recorded."""
    from std_srvs.srv import Trigger
    end = time.monotonic() + timeout; persistent = 0
    while True:
        g = state().get('safety_gate') or {}
        if g.get('fault_latched') is not True and g.get('status') in ('NORMAL', 'STALL_PENDING'): return
        cli = rogue.create_client(Trigger, '/stall_detector/reset_latch'); res = None
        if cli.wait_for_service(timeout_sec=3):
            f = cli.call_async(Trigger.Request()); wait_until(lambda: f.done(), 5)
            res = (f.result().success, f.result().message[:80]) if f.done() and f.result() else None
        rogue.destroy_client(cli); time.sleep(1.5)
        GATE_EVENTS.append({'before': label, 'status': g.get('status'), 'last_fault_reasons': g.get('last_fault_reasons'), 'unlatch': res})
        # 2026-11-22 (gz_r13_a: 1376 refusals): a refusal because an ACTIVE sensor fault persists (e.g. 'REFUSED: SENSOR_FAULT:icp' while
        # the robot stays pressed against a wall) cannot clear by retrying; the Gate is right to refuse. Stop after GATE_GUARD_PERSISTENT_MAX
        # consecutive such refusals instead of retrying for the full timeout. Transient refusals (a killed writer's DDS ghost,
        # INPUT_WRITERS:2, ~15-40 s) keep the full timeout. The Gate's decision and the PASS/FAIL rules are unchanged.
        persistent = persistent + 1 if res and not res[0] and 'SENSOR_FAULT' in (res[1] or '') else 0
        if persistent >= GATE_GUARD_PERSISTENT_MAX:
            GATE_EVENTS.append({'before': label, 'gave_up': 'persistent sensor fault refusal', 'attempts': persistent}); return
        if time.monotonic() > end: return


def recovery_guard(label):
    r = robot()
    stuck = [m['mission_id'] for m in get(f'/missions?robot_id={RID}&limit=30')['missions'] if m['status'] == 'RECONCILIATION_REQUIRED']
    for mid in stuck:
        try: ADMIN.post(f'/missions/{mid}/recover', {'note': f'gz e2e {label}'}); res = 'OK'
        except HttpError as e: res = str(e)[:160]
        RECOVERY_EVENTS.append({'before': label, 'mission': mid, 'result': res})
    if r.get('reconciliation_required') or r.get('gate_boot_reconciliation'):
        wait_until(lambda: (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED' and gate_now() == 'NORMAL', 30)
        try: ADMIN.post(f'/robots/{RID}/recover', {'note': f'gz e2e {label}: isolated Gazebo stack'}); res = 'OK'
        except HttpError as e: res = str(e)[:200]
        RECOVERY_EVENTS.append({'before': label, 'robot_recover': res})


def mode_now(): return (get(f'/robots/{RID}/mode')['operating_mode'] or {}).get('mode')


def ensure_mode(target):
    if mode_now() == target: return True
    if mode_now() == 'FAULT':
        switch('IDLE', 45)
    return switch(target, 60).get('mode') == target


SCEN = []


def scenario(fn):
    SCEN.append(fn); return fn


# 2026-11-21 (Gazebo integration, first failure of the full run): g01+g02+g07 drive the robot ~1.1-2 m forward from patrol_A1 and
# g07 pushes it into the AdjTable (gz_moved 0.12 m/cycle, then 0.0 from cycle 6 while the Gate still passed 0.1 m/s). g03 then starts
# wedged: Gate STALL_LATCHED (cmd 0.1, wheel 0.1, lidar 0.0) is a TRUE positive, the map is one scan and g03b..f08 cascade. Scenarios
# that need free space therefore start from the spawn pose. The robot is driven back through the SAME web manual bench (forward-only,
# deadman held) with Gazebo truth as the harness's eyes; every homing is recorded (HOME_EVENTS), none is hidden. GZWEB_HOME=0 disables
# it (used for the same-code "before" reproduction).
HOME_BEFORE = {'g02_web_browser_keyboard_pointer_gamepad', 'g07_pointer_input_diagnosis', 'g03_mapping_start_explore_save_stop', 'g03b_fallback_map_manual_drive_with_harness_slam',
               'g04_load_saved_map_initial_pose_localized',   # 2026-11-22: g03 can end wedged on a below-scan obstacle; do not depend on g03b's homing
               'f01_web_disconnect_during_manual', 'f02_agent_kill_during_manual_and_stale_session_after_restart',
               'f03_watchdog_kill_during_manual', 'f04_gate_process_stall_0_35s_during_manual', 'f05_lidar_stop_during_manual',
               'd01_gate_sigstop_0_35s_forward', 'd02_gate_sigstop_0_5s_forward', 'd03_gate_sigstop_2s_forward',
               'd04_gate_and_watchdog_sigstop_2s_forward', 'd05_watchdog_sigkill_forward', 'd06_gate_sigkill_forward'}
HOME_EVENTS = []
# the forward-driving manual input tests (g02 ~0.2 m, g07 15 x ~0.12 m) start at spawn facing the freest line of sight (2026-11-21
# after_full_r1: from spawn yaw 0 the front range is 2.40 m, too little for ~1.9 m of travel plus the body)
HOME_FACE = {'g02_web_browser_keyboard_pointer_gamepad': 'open', 'g07_pointer_input_diagnosis': 'open'}
wrap = lambda a: math.atan2(math.sin(a), math.cos(a))


def scan_valid_count():
    """Returns the lidar_scan_matcher_odom_node counts as ICP points (finite, 0.15..9.5 m); the Gate blocks below 100 inliers."""
    sc = LAST_SCAN[0]
    return None if not sc else sum(1 for r in sc[2] if math.isfinite(r) and .15 <= r <= 9.5)


def open_heading():
    """Truth-frame yaw for forward-only drives: a +-15 deg sector with >= 2.5 m clear (20th-percentile range) AND a wall in range ahead.
    2026-11-22: the previous 'largest range' choice sent g07 into the open middle of the room, where fewer than 100 returns are within
    9.5 m -> Gate FAILSAFE_ICP_DEGRADED (correct) with motion blocked, so the robot could never leave (after_full_r2: 21 cascaded FAILs).
    Score: finite fraction of the sector first, then min(p20, 5 m)."""
    sc, p = LAST_SCAN[0], pose()
    if not (sc and p): return 0.0, None
    a0, inc, rs = sc; best = None
    for k in range(0, 360, 10):
        c = math.radians(k)
        raw = [r for i, r in enumerate(rs) if abs(wrap(a0 + i * inc - c)) <= math.radians(15)]
        if not raw: continue
        sec = sorted(r if math.isfinite(r) and r <= 9.5 else 9.5 for r in raw); p20 = sec[len(sec) // 5]
        fin = sum(1 for r in raw if math.isfinite(r) and .15 <= r <= 9.5) / len(raw)
        cand = (p20 >= 2.5, round(fin, 1), min(p20, 5.0), c, p20)
        if best is None or cand[:3] > best[:3]: best = cand
    if best is None: return 0.0, None
    return wrap(p[2] + best[3]), round(best[4], 2)


def home(label, tol=.30, timeout=120., face=None):
    p = pose()
    if os.environ.get('GZWEB_HOME') == '0' or not p: return
    ev = {'before': label, 'start': [round(v, 3) for v in p], 'dist_m': round(math.hypot(p[0], p[1]), 3)}
    if ev['dist_m'] < tol and abs(wrap(p[2])) < .35 and face is None: return
    lease()
    if not ensure_mode('MANUAL'): ev['result'] = 'MANUAL not reached'; HOME_EVENTS.append(ev); return
    m = Manual(0., 0.).start(); end = time.monotonic() + timeout; prog = (time.monotonic(), p[:2]); stuck = 0
    try:
        while time.monotonic() < end and m.on:
            p = pose(); d = math.hypot(p[0], p[1])
            if d < .12: break
            err = wrap(math.atan2(-p[1], -p[0]) - p[2])
            if abs(err) > .30: m.lin, m.turn = 0., (1. if err > 0 else -1.)
            else: m.lin, m.turn = 1., max(-1., min(1., 2.5 * err))
            if m.lin and time.monotonic() - prog[0] > 2.5:
                if math.hypot(p[0] - prog[1][0], p[1] - prog[1][1]) < .03:          # pushing without moving: back off by turning
                    stuck += 1; m.lin, m.turn = 0., 1.; time.sleep(1.5)
                prog = (time.monotonic(), p[:2])
            time.sleep(.1)
        yaw_t = 0.0
        if face == 'open':
            time.sleep(.3); yaw_t, ev['open_range_m'] = open_heading(); ev['face_yaw'] = round(yaw_t, 3); ev['scan_valid_at_start'] = scan_valid_count()
        while time.monotonic() < end and m.on and abs(wrap(yaw_t - pose()[2])) > .12:
            e = wrap(yaw_t - pose()[2]); m.lin, m.turn = 0., max(-1., min(1., 2. * e)) if abs(e) < .4 else (1. if e > 0 else -1.); time.sleep(.1)
    finally:
        m.lin = m.turn = 0.; time.sleep(.3); m.stop()
    p = pose(); time.sleep(.8)
    ev.update(end=[round(v, 3) for v in p], end_dist_m=round(math.hypot(p[0], p[1]), 3), stuck_backoffs=stuck, errors=m.errors[:3],
              gate=gate_now(), result='OK' if math.hypot(p[0], p[1]) < .2 else 'NOT_HOME')
    HOME_EVENTS.append(ev); say('home', json.dumps(ev))


def run_one(fn):
    if ONLY and fn.__name__ not in ONLY: return
    say('---', fn.__name__); MARKS.append((fn.__name__, 'start', round(time.time(), 3)))
    try:
        try: gate_guard(fn.__name__); recovery_guard(fn.__name__)
        except Exception as e: GATE_EVENTS.append({'before': fn.__name__, 'guard_error': repr(e)[:160]})
        if fn.__name__ in HOME_BEFORE:
            try: home(fn.__name__, face=HOME_FACE.get(fn.__name__)); gate_guard(fn.__name__ + ':after_home')
            except Exception as e: HOME_EVENTS.append({'before': fn.__name__, 'error': repr(e)[:200]})
        precondition(fn.__name__)
        fn()
    except Exception as e:
        try:
            st = state(); diag = {'gate': (st.get('safety_gate') or {}).get('status'), 'mode': mode_now(),
                                  'watchdog': (st.get('motion_watchdog') or {}).get('state'), 'arbiter': (st.get('velocity_arbiter') or {}).get('blocker'),
                                  'loc': {k: (st.get('localization') or {}).get(k) for k in ('status', 'detail')}}
        except Exception as d: diag = repr(d)[:100]
        R.record(fn.__name__, False, error=repr(e)[:300], diag=diag, trace=traceback.format_exc()[-700:])
    finally:
        MARKS.append((fn.__name__, 'end', round(time.time(), 3)))
        try:
            if not healthy_now()[0] and HEALTH['broken_by'] is None: HEALTH['broken_by'] = fn.__name__
        except Exception: pass


def healthy_now():
    """Environment health after the guards: Gate NORMAL/STALL_PENDING and not latched, operating mode not FAULT."""
    g = (state().get('safety_gate') or {}); m = mode_now()
    ok = g.get('fault_latched') is not True and g.get('status') in ('NORMAL', 'STALL_PENDING') and m != 'FAULT'
    return ok, {'gate': g.get('status'), 'fault_latched': g.get('fault_latched'), 'last_fault_reasons': g.get('last_fault_reasons'), 'mode': m}


def precondition(name):
    try: ok, snap = healthy_now()
    except Exception as e: ok, snap = None, {'error': repr(e)[:120]}
    if ok: HEALTH['broken_by'] = None
    HEALTH['pre'][name] = dict(snap, healthy=ok, inherited_from=None if ok else HEALTH['broken_by'])


def classify_failures():
    """FAIL with an unhealthy precondition inherited from an earlier scenario = cascade (cascade_of); otherwise first failure."""
    first, casc = [], {}
    for it in R.items:
        if it.get('result') != 'FAIL': continue
        pre = HEALTH['pre'].get(it.get('scenario'), {})
        it['precondition'] = pre
        if pre.get('healthy') is False and pre.get('inherited_from'):
            it['failure_class'] = 'cascade'; it['cascade_of'] = pre['inherited_from']; casc[it['scenario']] = pre['inherited_from']
        else:
            it['failure_class'] = 'first'; first.append(it.get('scenario'))
    return {'first_failures': first, 'cascaded': casc}


def graph():
    g = obs
    pubs = lambda t: sorted(f'{i.node_namespace.rstrip("/")}/{i.node_name}' for i in g.get_publishers_info_by_topic(t))
    subs = lambda t: sorted(f'{i.node_namespace.rstrip("/")}/{i.node_name}' for i in g.get_subscriptions_info_by_topic(t))
    names = sorted(n for n, _ in g.get_node_names_and_namespaces())
    return {'cmd_vel_pub': pubs('/cmd_vel'), 'cmd_vel_sub': subs('/cmd_vel'), 'locked_pub': pubs('/cmd_vel_locked'), 'locked_sub': subs('/cmd_vel_locked'), 'drive_pub': pubs('/sim/drive_cmd'), 'drive_sub': subs('/sim/drive_cmd'),
            'raw_pub': pubs('/cmd_vel_raw'), 'nav_cmd_pub': pubs('/fleet/nav_cmd_vel'), 'smoothed_pub': pubs('/fleet/cmd_vel_nav_smoothed'),
            'clock_pub': pubs('/clock'), 'scan_pub': pubs('/scan'), 'map_pub': pubs('/map'), 'odom_pub': pubs('/odom'), 'odom_nav2_pub': pubs('/odom_nav2'),
            'tf_pub': pubs('/tf'), 'amcl': [n for n in names if 'amcl' in n], 'map_server': [n for n in names if 'map_server' in n],
            'slam': [n for n in names if 'slam' in n]}


def owned(pattern, groups):
    return owned_pid(RUN, pattern, groups)


# ================================================================== normal operation
MAP, NOTE, TRAJ = {}, {}, []


NAV2_RESTARTS = []


@scenario
def g00_bringup_graph_and_clock():
    ready = lambda: (truth[0] and clock[0] and len(S['scan']) > 20 and gate_now() == 'NORMAL' and robot().get('online')
                     and (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED'
                     and ((state().get('execution') or {}).get('nav_idle') or {}).get('ok') is True)
    up = wait_until(ready, 120, 1.0)
    for attempt in range(2):
        if up: break
        # known intermittent Nav2 lifecycle bring-up failure (lost change_state response; same recovery as nav_e2e ensure_nav2):
        # restart the stack group (Nav2 + Gate) and the Agent; recorded, never silent
        NAV2_RESTARTS.append({'attempt': attempt, 'smoother_pub': graph()['smoothed_pub'], 'gate': gate_now()})
        for op, n in (('kill', 'stack'), ('kill', 'agent')): subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), op, n], check=False)
        time.sleep(3)
        for op, n in (('start', 'stack'), ('start', 'agent')): subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), op, n], check=False); time.sleep(6)
        up = wait_until(ready, 120, 1.0)
        recovery_guard('g00_after_nav2_restart')
    time.sleep(5.0); gr = graph(); t = time.monotonic(); sc = rows('scan', t - 5, t)
    gaps = [b[0] - a[0] for a, b in zip(sc, sc[1:])]
    c0 = clock[0]; time.sleep(2.0); c1 = clock[0]
    rtf = round((c1[1] - c0[1]) / (c1[0] - c0[0]), 3) if c0 and c1 and c1[0] > c0[0] else None
    # A clash = one of the LIMO localization/body frames published as a CHILD by anyone else than its expected parent
    # (static body TFs are on /tf_static and not counted here; the Office demo robots use their own world->tinyRobotN/* tree).
    expected = {'odom': 'map', 'base_footprint': 'odom', 'base_link': 'base_footprint', 'laser_link': 'base_link'}
    clash = sorted(f'{a}->{b}' for a, b in TF_PAIRS if b in expected and a != expected[b])
    ok = (bool(up) and not clash and gr['cmd_vel_pub'] == [] and gr['cmd_vel_sub'] == [] and gr['locked_pub'] == ['/stall_detector_node']
          and gr['locked_sub'] and all(('sim_drive_timeout' if DRIVE_TIMEOUT > 0 else 'gz_bridge') in s or 'observer' in s for s in gr['locked_sub'])
          and (DRIVE_TIMEOUT == 0 or (gr['drive_pub'] == ['/sim_drive_timeout'] and all('gz_bridge' in s or 'observer' in s for s in gr['drive_sub'])))
          and gr['raw_pub'] == ['/fleet_motion_watchdog'] and len(gr['nav_cmd_pub']) == 1 and gr['smoothed_pub'] == ['/velocity_smoother']
          and gr['clock_pub'] == ['/gz_bridge'] and gr['amcl'] == [] and gr['map_server'] == [] and gr['slam'] == [] and len(gr['odom_nav2_pub']) == 1)
    R.record('g00_bringup_graph_and_clock', ok, bringup_s=round(time.monotonic() - t_up, 1), nav2_restarts=NAV2_RESTARTS, graph=gr, scan_hz_5s=round(len(sc) / 5, 1), scan_gap_max_s=round(max(gaps), 3) if gaps else None,
             real_time_factor=rtf, pose_spawn_frame=pose(),
             tf_pairs=sorted(f'{a}->{b}:{n}' for (a, b), n in TF_PAIRS.items())[:40], tf_clash_with_limo_frames=clash)


def drive(lin, turn, secs):
    m = Manual(lin, turn).start(); t0 = time.monotonic(); p0 = pose(); time.sleep(secs); return m, t0, p0


@scenario
def g01_manual_drive_and_deadman_release():
    lease(); assert ensure_mode('MANUAL'), 'MANUAL not reached'
    m, t0, p0 = drive(1.0, 0.0, 2.5)
    p1 = pose(); lk = rows('locked', t0); tr = rows('truth', t0)
    m.deadman = False; t_rel = time.monotonic(); time.sleep(.6); m.stop()
    sm = stop_metrics(t_rel)
    m2, t2, q0 = drive(0.0, 1.0, 2.0); m2.stop(); turn = rows('truth', t2); th = [r[3] for r in turn]
    R.record('g01_manual_drive_and_deadman_release', bool(p0 and p1) and math.hypot(p1[0] - p0[0], p1[1] - p0[1]) > .08
             and sm['gate_last_nonzero_s'] is not None and sm['gate_last_nonzero_s'] < .5 and (sm['gz_last_moving_s'] or 0) < 1.0
             and any(abs(r[5]) > .1 for r in turn),
             moved_m=round(math.hypot(p1[0] - p0[0], p1[1] - p0[1]), 3), gate_max_vx=max((r[1] for r in lk), default=None),
             gz_max_v=round(max((r[4] for r in tr), default=0), 3), release=sm, errors=m.errors,
             turn_gz_max_w=round(max((abs(r[5]) for r in turn), default=0), 3), yaw_change=round(th[-1] - th[0], 3) if th else None)


@scenario
def g02_web_browser_keyboard_pointer_gamepad():
    if not BROWSER: R.skip('g02_web_browser_keyboard_pointer_gamepad', '--browser not given'); return
    lease(); assert ensure_mode('MANUAL')
    env = dict(os.environ, NODE_PATH=os.environ.get('MANUAL_E2E_NODE_PATH', ''))
    bdir = OUT / 'browser'; bdir.mkdir()
    p = subprocess.run(['node', str(FC / 'tests/manual_e2e/browser_manual_inputs.cjs'), URL, str(RUN / 'secrets' / 'operator.token'), str(bdir)],
                       capture_output=True, text=True, env=env, timeout=120)
    (bdir / 'browser.log').write_text(p.stdout + '\n' + p.stderr[-4000:])
    marks = {}
    for line in p.stdout.splitlines():
        try: d = json.loads(line); marks[d['mark']] = d
        except Exception: pass
    mono = time.monotonic() - time.time()                 # browser marks are wall clock
    out = {}
    for src, on, off, nxt in (('keyboard', 'kb_on', 'kb_off', 'ptr_on'), ('pointer', 'ptr_on', 'ptr_off', 'pad_on'), ('gamepad', 'pad_on', 'pad_off', 'padfreeze_on')):
        if on not in marks or off not in marks: out[src] = None; continue
        a, b = marks[on]['t'] + mono, marks[off]['t'] + mono; c = marks[nxt]['t'] + mono if nxt in marks else b + 1.5
        tr = rows('truth', a, b); after = rows('truth', b + .6, c)
        out[src] = {'gz_moved_m': round(math.hypot(tr[-1][1] - tr[0][1], tr[-1][2] - tr[0][2]), 3) if len(tr) > 1 else None,
                    'gate_max_vx': max((r[1] for r in rows('locked', a, b)), default=None),
                    'gz_max_speed_0_6s_after_release': round(max((abs(r[4]) for r in after), default=0), 3)}
    ok = p.returncode == 0 and all(o and (o['gz_moved_m'] or 0) > .02 and o['gz_max_speed_0_6s_after_release'] < .02 for o in out.values())
    R.record('g02_web_browser_keyboard_pointer_gamepad', ok, inputs=out, browser_rc=p.returncode,
             note='real dashboard in headless Chrome; gamepad is a synthetic navigator.getGamepads object')


def pointer_diag_run(name, cycles=15, hold=1500, gap=800, extra=True):
    """Browser event -> Fleet HTTP -> Agent/Arbiter -> Gate -> Gazebo, per pointer cycle; classifies where an input was lost."""
    lease(); assert ensure_mode('MANUAL'), 'MANUAL not reached'
    d = OUT / name; d.mkdir()
    arb, stop_arb = [], threading.Event()
    def arb_poll():
        while not stop_arb.is_set():
            try:
                a = state().get('velocity_arbiter') or {}
                arb.append((time.time(), a.get('source'), a.get('blocker'), (a.get('manual') or {}).get('status') if isinstance(a.get('manual'), dict) else None))
            except Exception as e: arb.append((time.time(), 'ERR', repr(e)[:60], None))
            time.sleep(.2)
    th = threading.Thread(target=arb_poll, daemon=True); th.start()
    env = dict(os.environ, NODE_PATH=os.environ.get('MANUAL_E2E_NODE_PATH', ''))
    p = subprocess.run(['node', str(FC / 'tests/manual_e2e/browser_pointer_diag.cjs'), URL, str(RUN / 'secrets' / 'operator.token'),
                        str(cycles), str(hold), str(gap), 'extra' if extra else 'none'], capture_output=True, text=True, env=env, timeout=600)
    stop_arb.set(); th.join(2)
    (d / 'browser_events.jsonl').write_text(p.stdout); (d / 'browser_stderr.txt').write_text(p.stderr[-4000:])
    (d / 'arbiter_poll.json').write_text(json.dumps(arb))
    ev = []
    for line in p.stdout.splitlines():
        try: ev.append(json.loads(line))
        except Exception: pass
    off = time.monotonic() - time.time()           # wall -> monotonic (same host)
    marks = [e for e in ev if e['kind'] == 'mark']
    cyc = []
    for i in range(cycles):
        dn = next((e['t'] for e in marks if e.get('mark') == 'down' and e.get('cycle') == i), None)
        up = next((e['t'] for e in marks if e.get('mark') == 'up' and e.get('cycle') == i), None)
        if dn is None or up is None:
            cyc.append({'cycle': i, 'loss': 'BROWSER_NO_PRESS', 'marks': [m.get('mark') for m in marks if m.get('cycle') == i]}); continue
        inwin = [e for e in ev if dn - .05 <= e['t'] <= up + .05]
        ptr = [e['kind'] for e in inwin if e['kind'].startswith('ptr_')]
        sent = [e for e in inwin if e['kind'] == 'bench_req' and e.get('action') == 'sample' and e.get('deadman') is True]
        resp = [e for e in inwin if e['kind'] == 'bench_resp' and e.get('action') == 'sample']
        bad = [e for e in resp if e.get('status') != 200]
        stat = [e['text'] for e in inwin if e['kind'] == 'status']
        gate = [r for r in rows('locked', dn + off, up + off) if nz(r[1], r[2])]
        trv = [r for r in rows('truth', dn + off, up + off + .3)]
        moved = round(math.hypot(trv[-1][1] - trv[0][1], trv[-1][2] - trv[0][2]), 3) if len(trv) > 1 else None
        after = [r for r in rows('locked', up + off + .45, up + off + .85) if nz(r[1], r[2])]
        arbw = sorted({(a[1], a[2]) for a in arb if dn <= a[0] <= up}, key=str)
        if 'ptr_pointerdown' not in ptr: loss = 'BROWSER_NO_POINTERDOWN'
        elif not sent: loss = 'PANEL_NO_DEADMAN_SAMPLE'
        elif bad: loss = 'HTTP_SAMPLE_REJECTED'
        elif not gate: loss = 'AGENT_OR_ARBITER_NO_OUTPUT'
        else: loss = None
        cyc.append({'cycle': i, 'loss': loss, 'ptr_events': ptr, 'deadman_samples': len(sent), 'sample_resp': [e.get('status') for e in resp][:12],
                    'sample_latency_ms_max': max((e.get('latency_ms') or 0 for e in resp), default=None), 'status': stat[:6],
                    'gate_nonzero': len(gate), 'gz_moved_m': moved, 'gate_nonzero_0_45_0_85s_after_up': len(after), 'arbiter': arbw[:4]})
    dash = [e for e in ev if e['kind'] == 'dash_resp']
    dgap = [round(b['t'] - a['t'], 3) for a, b in zip(dash, dash[1:])]
    extra_r = {}
    for k, a, b in (('leave', 'leave_down', 'leave_up'), ('stopbtn', 'stopbtn_click', 'stopbtn_up'), ('cancel', 'cancel_sent', 'cancel_up')):
        ta = next((e['t'] for e in marks if e.get('mark') == a), None); tb = next((e['t'] for e in marks if e.get('mark') == b), None)
        if ta and tb:
            if k == 'leave':
                extra_r[k] = {'gate_nonzero_while_outside': len([r for r in rows('locked', ta + off + .3, tb + off) if nz(r[1], r[2])]),
                              'gate_nonzero_after_up': len([r for r in rows('locked', tb + off + .45, tb + off + 1.0) if nz(r[1], r[2])]),
                              'status': [e['text'] for e in ev if e['kind'] == 'status' and ta <= e['t'] <= tb + 1][:4]}
            else:
                extra_r[k] = {'gate_nonzero_0_45s_after_event_until_up': len([r for r in rows('locked', ta + off + .45, tb + off) if nz(r[1], r[2])]),
                              'status': [e['text'] for e in ev if e['kind'] == 'status' and ta <= e['t'] <= tb + .5][:4]}
    return cyc, {'dashboard_poll_gap_s': {'p50': sorted(dgap)[len(dgap) // 2] if dgap else None, 'max': max(dgap) if dgap else None,
                                          'n_gt_1_0': sum(1 for x in dgap if x > 1.0)}, 'extra': extra_r, 'browser_rc': p.returncode}


@scenario
def g07_pointer_input_diagnosis():
    cyc, info = pointer_diag_run('pointer_diag')
    losses = {}
    for c in cyc: losses[c['loss'] or 'OK'] = losses.get(c['loss'] or 'OK', 0) + 1
    ok = losses.get('OK', 0) == len(cyc) and all(c.get('gate_nonzero_0_45_0_85s_after_up', 0) == 0 for c in cyc if not c['loss']) \
        and all(v.get('gate_nonzero_0_45s_after_event_until_up', v.get('gate_nonzero_after_up', 0)) == 0 for v in info['extra'].values())
    R.record('g07_pointer_input_diagnosis', ok, losses=losses, cycles=cyc, **info)


@scenario
def g03_mapping_start_explore_save_stop():
    lease(); assert ensure_mode('MAPPING'), 'MAPPING not reached'
    p0 = pose(); t0 = time.monotonic()
    st = mapping('START', mode='NEW_MAP', limits={'max_duration_s': 240, 'max_goals': 40})
    path, last = 0.0, pose()
    while time.monotonic() - t0 < float(os.environ.get('GZWEB_EXPLORE_S', '150')):
        w = state().get('mapping_worker') or {}
        if w.get('stop_reason'): break
        time.sleep(2); p = pose()
        if p: TRAJ.append(tuple(p[:2]))
        if p and last: path += math.hypot(p[0] - last[0], p[1] - last[1])
        last = p
    w = state().get('mapping_worker') or {}
    stalls = [g for g in rows('gate', t0) if 'STALL_LATCHED' in str(g[1])]
    end = ('BLOCKED_PHYSICAL (Gate STALL_LATCHED)' if stalls else ('WORKER_' + w['stop_reason']) if w.get('stop_reason') else 'TIME_LIMIT')
    MAP['t2m'] = capture_map_offset('g03')
    pz = mapping('PAUSE'); sv = mapping('SAVE', timeout=90, floor_id='paichai_c_1f', area_id='paichai_c_1f_c103_lounge'); sp = mapping('STOP')
    rec = switch('IDLE')
    MAP['id'] = (sv.get('registered') or {}).get('map_version_id') if sv else None
    MAP['usable'] = path > .5          # a map built without motion is one scan (2026-11-21 run 1: 733 free cells)
    R.record('g03_mapping_start_explore_save_stop', bool(MAP['id']) and rec.get('mode') == 'IDLE' and path > .5,
             start=(st or {}).get('command', {}).get('status'), explore_s=round(time.monotonic() - t0, 1), gz_path_m=round(path, 2),
             worker={k: w.get(k) for k in ('status', 'stop_reason', 'frontier_candidates', 'attempted_goals', 'goal_outcomes', 'repositions', 'last_decision', 'recent_decisions')},
             map_known_cells=(state().get('map') or {}).get('known_cells'), exploration_end=end,
             gate_stall_latches=[(round(g[0] - t0, 1), g[1]) for g in stalls][:4],
             pause=(pz or {}).get('command', {}).get('status'), save=(sv or {}).get('command', {}).get('status'),
             save_evidence=((sv or {}).get('result') or {}).get('evidence'), map_version_id=MAP['id'], stop=(sp or {}).get('state'),
             mode_after=rec.get('mode'), pose_start=p0, pose_end=pose())


@scenario
def g03b_fallback_map_manual_drive_with_harness_slam():
    """Only when g03 produced no usable map. NOT the web MAPPING path: web MANUAL driving (the same HTTP bench) while a
    harness-owned slam_toolbox builds the map; the bundle is saved, registered in THIS run's isolated registry, SLAM is stopped."""
    if MAP.get('id') and MAP.get('usable'):
        R.skip('g03b_fallback_map_manual_drive_with_harness_slam', 'g03 web mapping produced a usable map'); return
    lease(); assert ensure_mode('MANUAL'), 'MANUAL not reached'
    d = OUT / 'fallback_map'; d.mkdir()
    slam = subprocess.Popen(['/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node', '--ros-args', '-r', '__node:=slam_toolbox', '--params-file',
                             str(REPO / 'capstone_ws/src/limo_capstone/config/integrated_slam_params.yaml'), '-p', 'use_sim_time:=true'],
                            stdout=(d / 'slam.log').open('x'), stderr=subprocess.STDOUT, start_new_session=True)
    path = 0.0
    try:
        time.sleep(5)
        for lin, turn, secs in ((0.0, 1.0, 32), (1.0, 0.0, 10), (0.0, 1.0, 32), (1.0, 0.3, 12), (0.0, -1.0, 30)):
            m = Manual(lin, turn).start(); last = pose(); t0 = time.monotonic()
            while time.monotonic() - t0 < secs:
                time.sleep(1); p = pose(); path += math.hypot(p[0] - last[0], p[1] - last[1]); last = p; TRAJ.append(tuple(p[:2]))
            m.stop(); time.sleep(1.5)
        MAP['t2m'] = capture_map_offset('g03b')
        env = dict(os.environ)
        sv = subprocess.run(['ros2', 'service', 'call', '/slam_toolbox/save_map', 'slam_toolbox/srv/SaveMap', "{name: {data: '%s'}}" % (d / 'map')],
                            capture_output=True, text=True, timeout=40, env=env)
        sz = subprocess.run(['ros2', 'service', 'call', '/slam_toolbox/serialize_map', 'slam_toolbox/srv/SerializePoseGraph', "{filename: '%s'}" % (d / 'graph')],
                            capture_output=True, text=True, timeout=40, env=env)
    finally:
        os.killpg(slam.pid, signal.SIGINT)
        try: slam.wait(10)
        except subprocess.TimeoutExpired: os.killpg(slam.pid, signal.SIGKILL)
    code = ("import sys,json,os; sys.path.insert(0,sys.argv[1]); from fleet_server.map_registry import MapRegistry; d=sys.argv[3]; "
            "r=MapRegistry(sys.argv[2]+'/map_registry.db', sys.argv[2]+'/map_store'); rd=lambda n: open(os.path.join(d,n),'rb').read(); "
            "res=r.register_map_bundle(floor_id='paichai_c_1f', label='gazebo_office_fallback', yaml_bytes=rd('map.yaml'), pgm_bytes=rd('map.pgm'), "
            "posegraph_bytes=rd('graph.posegraph'), posegraph_data_bytes=rd('graph.data'), source_mode='SLAM_SAVED_BUNDLE', "
            "verification_status='UNVERIFIED', verification_evidence={'method':'isolated_gazebo_fallback_manual_slam'}, frame_id='map', actor='e2e-qa', actor_role='operator'); "
            "mv=res['map_version']['map_version_id']; r.link_map_version_area(mv, 'paichai_c_1f_c103_lounge', actor='e2e-qa'); "
            "print(json.dumps({'id': mv, 'integrity': r.verify_bundle_integrity(mv).get('ok')}))")
    out = subprocess.run([sys.executable, '-c', code, str(FC / 'server'), str(RUN / 'fleet'), str(d)], capture_output=True, text=True)
    res = json.loads(out.stdout.strip().splitlines()[-1]) if out.returncode == 0 and out.stdout.strip() else {'error': out.stderr[-300:]}
    MAP['id'] = res.get('id'); MAP['fallback'] = True
    R.record('g03b_fallback_map_manual_drive_with_harness_slam', bool(MAP['id']) and res.get('integrity') is True, register=res, gz_path_m=round(path, 2),
             save_map=sv.stdout[-160:], serialize=sz.stdout[-160:], files=sorted(x.name for x in d.iterdir()),
             note='fallback only: web MANUAL + harness slam_toolbox; the web MAPPING Frontier session is g03')


@scenario
def g04_load_saved_map_initial_pose_localized():
    assert MAP.get('id'), 'no map from g03'
    code = ("import sys,json; sys.path.insert(0,sys.argv[1]); from fleet_server.map_registry import MapRegistry; "
            "r=MapRegistry(sys.argv[2]+'/map_registry.db', sys.argv[2]+'/map_store'); "
            "mv=r.verify_map_version(sys.argv[3], status='VERIFIED', evidence={'method':'isolated_gazebo_e2e','world':'office_limo.world'}, "
            "actor='e2e-qa', actor_role='admin', reason='isolated Gazebo E2E map'); print(json.dumps({'s': mv.get('verification_status')}, default=str))")
    vr = subprocess.run([sys.executable, '-c', code, str(FC / 'server'), str(RUN / 'fleet'), MAP['id']], capture_output=True, text=True)
    lease(); assert ensure_mode('NAVIGATION'), 'NAVIGATION not reached'
    tries = []
    for attempt in range(4):
        navreq('LOAD', map_version_id=MAP['id'])
        st = wait_until(lambda: loc().get('status') in ('WAIT_INITIAL_POSE', 'FAILED') and loc().get('status'), 45); tries.append((st, loc().get('detail')))
        if st == 'WAIT_INITIAL_POSE': break
        time.sleep(10)
    p = mpose(); navreq('INITIAL_POSE', x=p[0], y=p[1], yaw=p[2])
    l = wait_until(lambda: loc().get('status') == 'CONFIRMED' and loc(), 45) or loc()
    gr = graph()
    R.record('g04_load_saved_map_initial_pose_localized', l.get('status') == 'CONFIRMED' and len(gr['amcl']) == 1 and len(gr['map_server']) == 1
             and gr['slam'] == [] and len(gr['map_pub']) == 1,
             verify=vr.stdout.strip()[-120:] or vr.stderr[-200:], load_tries=tries, truth_initial_pose=p,
             localization={k: l.get(k) for k in ('status', 'match_ratio', 'amcl_pose', 'planner_allow_unknown', 'map_version_id')},
             producers={k: gr[k] for k in ('amcl', 'map_server', 'slam', 'map_pub', 'tf_pub')})


def find_goal(dmin, dmax):
    p = mpose(); cands = []
    for x, y in mtraj()[::2]:          # cells the robot body physically passed in g03b (Gazebo furniture below the LiDAR plane exists)
        if dmin <= math.hypot(x - p[0], y - p[1]) <= dmax:
            c = get(f'/robots/{RID}/goal-check?x={round(x, 2)}&y={round(y, 2)}')
            if c.get('ok'): return (round(x, 2), round(y, 2))
    for i in range(-8, 9):
        for j in range(-8, 9):
            x, y = round(p[0] + .35 * i, 2), round(p[1] + .35 * j, 2)
            if dmin <= math.hypot(x - p[0], y - p[1]) <= dmax: cands.append((x, y))
    cands.sort(key=lambda c: abs(math.hypot(c[0] - p[0], c[1] - p[1]) - (dmin + dmax) / 2))
    for x, y in cands[:60]:
        c = get(f'/robots/{RID}/goal-check?x={x}&y={y}')
        if c.get('ok'): return (x, y)
    return None


def browser_click(name, targets):
    env = dict(os.environ, NODE_PATH=os.environ.get('MANUAL_E2E_NODE_PATH', ''))
    p = subprocess.run(['node', str(HERE / 'browser_map_click.cjs'), URL, str(RUN / 'secrets' / 'operator.token'), json.dumps(targets)],
                       capture_output=True, text=True, env=env, timeout=400)
    (OUT / f'{name}.jsonl').write_text(p.stdout); (OUT / f'{name}.stderr.txt').write_text(p.stderr[-3000:])
    ev = []
    for line in p.stdout.splitlines():
        try: ev.append(json.loads(line))
        except Exception: pass
    return ev, p.returncode


def find_reason(reason, rmin=0.5, rmax=4.0):
    p = mpose()
    for r in [rmin + 0.25 * k for k in range(int((rmax - rmin) / .25) + 1)]:
        for k in range(24):
            a = 2 * math.pi * k / 24; x, y = round(p[0] + r * math.cos(a), 2), round(p[1] + r * math.sin(a), 2)
            c = get(f'/robots/{RID}/goal-check?x={x}&y={y}')
            if not c.get('ok') and c.get('reason') == reason: return (x, y)
    return None


T_G08 = [0.0]


@scenario
def g08_browser_map_click_goal():
    """Real dashboard map click -> drawer -> pre-check -> /missions -> Agent -> Nav2 -> Gazebo -> UI status."""
    if not BROWSER: R.skip('g08_browser_map_click_goal', '--browser not given'); return
    assert MAP.get('id'), 'no map'
    lease(); assert ensure_mode('NAVIGATION'), 'NAVIGATION not reached'
    T_G08[0] = time.monotonic()
    nmiss = lambda: len(get(f'/missions?robot_id={RID}&limit=100')['missions'])
    # server goal-check accepted cells, nearest-first in the traversed area; the dashboard pre-check (footprint 0.32 m) may refuse
    # some of them - the browser keeps trying like an operator, and every refused spot is recorded ('try' events)
    cands = []
    for x, y in mtraj()[::2] + [(round(mpose()[0] + .35 * i, 2), round(mpose()[1] + .35 * j, 2)) for i in range(-6, 7) for j in range(-6, 7)]:
        d = math.hypot(x - mpose()[0], y - mpose()[1])
        if 1.0 <= d <= 2.6 and get(f'/robots/{RID}/goal-check?x={round(x, 2)}&y={round(y, 2)}').get('ok'): cands.append((round(x, 2), round(y, 2)))
        if len(cands) >= 12: break
    assert cands, 'no server-valid goal candidates'
    valid = cands[0]
    # (1) localization NOT confirmed: fresh LOAD, no initial pose yet -> the click may pass the map pre-check, the server must refuse
    for attempt in range(4):
        navreq('LOAD', map_version_id=MAP['id'])
        if wait_until(lambda: loc().get('status') in ('WAIT_INITIAL_POSE', 'FAILED') and loc().get('status'), 45) == 'WAIT_INITIAL_POSE': break
        time.sleep(10)
    # the click must pass the dashboard pre-check, so that only the missing localization can refuse it
    n0 = nmiss(); ev1, rc1 = browser_click('map_click_unconfirmed', [{'name': 'valid_but_unlocalized', 'candidates': cands, 'expect': 'submit', 'timeout_s': 5}])
    un = next((e for e in ev1 if e['kind'] == 'target'), {}); created_unconfirmed = nmiss() - n0
    p = mpose(); navreq('INITIAL_POSE', x=p[0], y=p[1], yaw=p[2])
    assert wait_until(lambda: loc().get('status') == 'CONFIRMED', 45), 'relocalization failed'
    # (2) negatives on a confirmed map + one valid goal
    mm = robot().get('map_meta') or {}
    outside = None
    if mm.get('width'):
        res, ox, oy = mm['resolution'], mm.get('origin_x', 0), mm.get('origin_y', 0)
        for off in (0.3, 0.6, 1.0):
            cand = (round(ox - off, 2), round(oy + mm['height'] * res / 2, 2))
            if get(f'/robots/{RID}/goal-check?x={cand[0]}&y={cand[1]}').get('reason') == 'OUTSIDE_MAP': outside = cand; break
    occ, unk = find_reason('OCCUPIED') or find_reason('FOOTPRINT_OCCUPIED'), find_reason('UNKNOWN_AREA')
    tg = [dict(name=n, x=c[0], y=c[1], expect='blocked') for n, c in (('outside_map', outside), ('obstacle', occ), ('unknown_area', unk)) if c]
    tg.append({'name': 'valid', 'candidates': cands, 'expect': 'submit', 'timeout_s': 150})
    n1 = nmiss(); ev2, rc2 = browser_click('map_click_confirmed', tg)
    tgt = {e['name']: e for e in ev2 if e['kind'] == 'target'}
    refused_by_ui = [{'want': e.get('want'), 'review': (e.get('review') or '')[-90:]} for e in ev2 if e['kind'] == 'try']
    valid = tuple(tgt['valid']['want']) if tgt.get('valid') else cands[0]
    ui = next((e for e in ev2 if e['kind'] == 'mission_ui'), {})
    posts = [e for e in ev2 if e['kind'] == 'mission_post']
    mid = next((e.get('mission_id') for e in posts if e.get('status') in (200, 201, 202) and e.get('mission_id')), None)
    if not mid and nmiss() > n1:      # fallback: the newest mission of this robot created after the click pass started
        mid = get(f'/missions?robot_id={RID}&limit=1')['missions'][0]['mission_id']
    final = wait_mission(mid, 60) if mid else None
    pz = mpose()          # map frame, as the clicked goal
    checks = {
        'unconfirmed_refused': created_unconfirmed == 0 and bool(un) and un.get('submitted') is True and bool(un.get('merr')),
        'negatives_blocked': all((not tgt.get(t['name'], {}).get('submitted') or bool(tgt.get(t['name'], {}).get('merr')))
                                 and 'error' not in tgt.get(t['name'], {'error': 1})
                                 for t in tg if t['expect'] == 'blocked'),
        'negatives_found': {'outside_map': bool(outside), 'obstacle': bool(occ), 'unknown_area': bool(unk)},
        'valid_clicked_coords_ok': bool(tgt.get('valid')) and abs(float(tgt['valid'].get('mx') or 1e9) - valid[0]) < .1 and abs(float(tgt['valid'].get('my') or 1e9) - valid[1]) < .1,
        'valid_succeeded': final == 'SUCCEEDED', 'ui_shows_terminal': 'SUCCEEDED' in (ui.get('text') or ''),
        'gz_at_goal': bool(pz) and math.hypot(pz[0] - valid[0], pz[1] - valid[1]) < .45,
        'only_one_mission_created': nmiss() - n1 == 1}
    ok = all(v for k, v in checks.items() if k != 'negatives_found') and all(checks['negatives_found'].values())
    R.record('g08_browser_map_click_goal', ok, checks=checks, unconfirmed=un, targets={k: {kk: v.get(kk) for kk in ('want', 'mx', 'my', 'cursor_at_click', 'submit_enabled', 'submitted', 'merr', 'review', 'error')} for k, v in tgt.items()},
             mission=mid, final=final, ui=ui.get('text'), gz_final=pz, browser_rc=[rc1, rc2],
             server_ok_but_ui_refused=refused_by_ui, n_server_candidates=len(cands),
             gate_latches_during=[g[1] for g in rows('gate', T_G08[0]) if 'FAILSAFE' in str(g[1]) or 'LATCHED' in str(g[1])][:4])


@scenario
def g05_web_goal_nav2_path_following():
    lease(); assert mode_now() == 'NAVIGATION' and loc().get('status') == 'CONFIRMED', 'not localized'
    g = find_goal(1.2, 2.4); assert g, 'no valid goal found near the robot'
    r = mission(*g); mid = r['mission_id']; t0 = time.monotonic(); srcs, paths = set(), []
    def watcher():
        while mstatus(mid) not in TERMINAL:
            srcs.add((state().get('velocity_arbiter') or {}).get('source')); pth = (loc().get('nav_path') or {}).get('points')
            if pth: paths.append(len(pth))
            time.sleep(.5)
    th = threading.Thread(target=watcher, daemon=True); th.start(); final = wait_mission(mid, 150); th.join(2)
    p = mpose(); arb = rows('raw', t0); lk = rows('locked', t0)
    R.record('g05_web_goal_nav2_path_following', final == 'SUCCEEDED' and p and math.hypot(p[0] - g[0], p[1] - g[1]) < .45 and 'NAV2' in srcs and bool(paths),
             goal=g, final=final, gz_final_pose_map_frame=p, truth_to_map=MAP.get("t2m"), gz_goal_error_m=round(math.hypot(p[0] - g[0], p[1] - g[1]), 3) if p else None,
             arbiter_sources=sorted(str(s) for s in srcs), path_points_max=max(paths) if paths else 0, duration_s=round(time.monotonic() - t0, 1),
             gate_input_nonzero=sum(1 for r_ in arb if nz(r_[1], r_[2])), gate_nonzero=sum(1 for r_ in lk if nz(r_[1], r_[2])),
             note='HTTP POST /missions = the request the web goal drawer sends; the map-canvas click itself is not driven')


@scenario
def g06_navigation_to_manual_mid_goal():
    lease(); assert mode_now() == 'NAVIGATION'
    g = find_goal(2.0, 3.2) or find_goal(1.2, 2.4); assert g
    mid = mission(*g)['mission_id']
    wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2' and truth[0][4] > .05, 40); time.sleep(1.0)
    t_ev = time.monotonic(); rec = switch('MANUAL', 40)
    sm = stop_metrics(t_ev, 1.5)
    R.record('g06_navigation_to_manual_mid_goal', rec.get('mode') == 'MANUAL' and mstatus(mid) == 'CANCELED' and (sm['gz_last_moving_s'] or 0) < 1.5,
             goal=g, mission=mstatus(mid), mode=rec.get('mode'), stop=sm)


# ================================================================== faults (MANUAL driving unless noted)
CHAIN_BLOCKERS = ('AGENT_EXECUTION_NOT_ALLOWED', 'FLEET_DISCONNECTED', 'RECONCILIATION_REQUIRED', 'LEASE_MISSING_OR_EXPIRED',
                  'WATCHDOG_NOT_AUTHORIZED', 'GATE_STATUS_STALE', 'GATE_NOT_NORMAL', 'ODOM_STALE', 'VELOCITY_GRAPH_NOT_LOCKED')


DIR = [1.0]


def in_manual_drive(secs=1.5):
    lease(); assert ensure_mode('MANUAL'), 'MANUAL not reached'
    # after a restarted Watchdog/Agent the Arbiter re-authorizes only once the chain is fresh again (2026-11-21 run 1: f04/f05
    # started 0 s after f03 and got mock_arbiter_not_ready WATCHDOG_NOT_AUTHORIZED); wait for it instead of failing the scenario
    wait_until(lambda: (state().get('velocity_arbiter') or {}).get('blocker') not in CHAIN_BLOCKERS
               and (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED', 40, .5)
    # alternate forward / reverse so repeated fault drives do not push the robot into furniture (2026-11-21 run 2: the robot
    # ended against an obstacle, Gate STALL_PENDING, v=0); a failed precondition always closes its own bench session
    # run 3: the web bench is forward-only (requested_linear = max(0, v)), so reverse never moved; repeated forward drives
    # ended in furniture (Gate STALL_PENDING). The fault drives therefore ROTATE IN PLACE, alternating direction: the
    # stop measurements (Gate output |v|,|w|, Gazebo |v|,|w|) are the same, the robot stays where the map is known.
    turn = DIR[0]; DIR[0] = -DIR[0]
    wait_until(lambda: gate_now() == 'NORMAL', 30, .5); time.sleep(1.5)   # a reset Gate passes INITIALIZING first (run 4 f04)
    for attempt in range(10):
        try: m, t0, p0 = drive(0.0, turn, secs); break
        except HttpError as e:
            if 'not_ready' not in str(e) or attempt == 9: raise
            time.sleep(1.0)
    if abs(truth[0][5]) <= .05 and truth[0][4] <= .03:
        m.stop(); raise AssertionError(f'robot not moving before the fault (v={truth[0][4]:.3f}, w={truth[0][5]:.3f}, gate={gate_now()})')
    return m


@scenario
def f01_web_disconnect_during_manual():
    m = in_manual_drive(); t_ev = time.monotonic(); m.stop(send=False)          # samples stop, no stop request (tab/network loss)
    sm = stop_metrics(t_ev, 2.5)
    R.record('f01_web_disconnect_during_manual', sm['gate_last_nonzero_s'] is not None and sm['gate_last_nonzero_s'] < 1.0
             and (sm['gz_last_moving_s'] or 0) < 1.5, stop=sm)


@scenario
def f02_agent_kill_during_manual_and_stale_session_after_restart():
    m = in_manual_drive(); sess, seq = m.session, m.seq
    t_ev = time.monotonic(); subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), 'kill', 'agent'], check=False); m.stop(send=False)
    sm = stop_metrics(t_ev, 2.5)
    subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), 'start', 'agent'], check=False)
    back = wait_until(lambda: robot().get('online') and mode_now() is not None, 60)
    time.sleep(3); t_re = time.monotonic()
    replay = expect_http(lambda: m.sample(deadman=True, session=sess, seq=seq + 1), 409)
    if replay[1] == 'accepted': replay = (False, 'accepted')
    time.sleep(1.5); moved = [r for r in rows('locked', t_re) if nz(r[1], r[2])]
    R.record('f02_agent_kill_during_manual_and_stale_session_after_restart',
             sm['gate_last_nonzero_s'] is not None and sm['gate_last_nonzero_s'] < 1.0 and (sm['gz_last_moving_s'] or 0) < 1.5
             and not moved and replay[1] != 'accepted',
             stop=sm, agent_back=bool(back), old_session_replay=replay, gate_nonzero_after_replay=len(moved), mode_after=mode_now())


@scenario
def f03_watchdog_kill_during_manual():
    m = in_manual_drive(); t_ev = time.monotonic()
    old_bound = ((GATE_LAST[0] or {}).get('cmd_input') or {}).get('bound_gid'); old_boot = (watch[0] or {}).get('boot_id')
    subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), 'kill', 'watchdog'], check=False)
    sm = stop_metrics(t_ev, 2.5); m.stop()
    subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), 'start', 'watchdog'], check=False)
    wait_until(lambda: (watch[0] or {}).get('state') in ('MOTION_AUTHORIZED', 'LOCKED') and (watch[0] or {}).get('boot_id') != old_boot, 30)
    # the killed Watchdog's DDS endpoint lingers (~15-30 s); the Gate latches cmd_input:NOT_EXCLUSIVE meanwhile (r11/r12 design)
    t_g = time.monotonic(); single = wait_until(lambda: len(obs.get_publishers_info_by_topic('/cmd_vel_raw')) == 1, 60, 1.0)
    ghost_s = round(time.monotonic() - t_g, 1)
    # 2026-11-22 (f04 root cause, run r4 Gate log): the Gate's own graph kept ONLY the dead writer's endpoint (= its bound GID) for
    # ~20 s, so it stayed NORMAL while the new Watchdog's input arrived, then discovered the new endpoint and latched NOT_EXCLUSIVE in
    # the middle of f04. Wait until the Gate is bound to the NEW writer (its reported output_gid), clearing a latch only through the
    # Gate's own reset service (gate_guard, refused while two writers are visible). Recorded: how long the Gate stayed bound to the
    # dead writer while the new one was already reporting its GID.
    stale = []
    def bound_to_new():
        g = GATE_LAST[0] or {}; ci = g.get('cmd_input') or {}; new = (watch[0] or {}).get('output_gid') or ''
        b = ci.get('bound_gid') or ''
        if b and b == old_bound and new and not new.startswith(b) and g.get('state') == 'NORMAL': stale.append(time.monotonic())
        if g.get('fault_latched') or ci.get('sticky'): gate_guard('f03:writer_swap', 4)
        return bool(b) and new.startswith(b) and len(ci.get('publishers') or []) == 1 and g.get('state') == 'NORMAL'
    t_b = time.monotonic(); rebound = wait_until(bound_to_new, 120, .5)
    R.record('f03_watchdog_kill_during_manual', sm['gate_last_nonzero_s'] is not None and sm['gate_last_nonzero_s'] < 1.0
             and (sm['gz_last_moving_s'] or 0) < 1.5 and rebound, stop=sm, gate_after=gate_now(),
             raw_writer_ghost_cleared_after_s=ghost_s if single else None,
             gate_bound_to_new_writer_after_s=round(time.monotonic() - t_b, 1) if rebound else None,
             gate_normal_bound_to_dead_writer_s=round(stale[-1] - stale[0] + .5, 1) if stale else 0.0,
             gate_input_after=((GATE_LAST[0] or {}).get('cmd_input') or {}).get('state'),
             gate_resets=[e for e in GATE_EVENTS if e.get('before') == 'f03:writer_swap'][:6])


@scenario
def f04_gate_process_stall_0_35s_during_manual():
    gpid = owned(r'stall_detector_node\.py', ['stack'])
    m = in_manual_drive(); t_ev = time.monotonic(); p0 = pose()
    os.kill(gpid, signal.SIGSTOP); time.sleep(.35); p_mid = pose(); os.kill(gpid, signal.SIGCONT); t_res = time.monotonic()
    during = rows('locked', t_ev, t_res); sm = stop_metrics(t_ev, 2.5); m.stop()
    after = rows('locked', t_res, t_res + 1.0)
    R.record('f04_gate_process_stall_0_35s_during_manual', True,
             verdict='MEASUREMENT (not pass/fail): Gazebo DiffDrive holds the last Gate output while the Gate is stopped',
             gate_outputs_during_stall=len(during), gz_moved_during_stall_m=round(math.hypot(p_mid[0] - p0[0], p_mid[1] - p0[1]), 3),
             gz_rotated_during_stall_rad=round(abs(math.atan2(math.sin(p_mid[2] - p0[2]), math.cos(p_mid[2] - p0[2]))), 3),
             last_gate_output_before=next(((round(r[1], 3), round(r[2], 3)) for r in reversed(rows('locked', t_ev - 1, t_ev))), None),
             first_gate_outputs_after_resume=[(round(r[0] - t_res, 3), round(r[1], 3)) for r in after[:5]], stop=sm, gate_after=gate_now())


# ================================================================== lower-drive protection (Gate-independent timeout)
def drive_metrics(t_ev, t_resume, horizon=3.0):
    """Event at t_ev (signal sent), resume at t_resume (None = never). Gate side, drive (bridge input) side, Gazebo truth."""
    time.sleep(max(0.0, t_ev + horizon - time.monotonic())); t1 = time.monotonic()
    lk, dv, tr = rows('locked', t_ev - 1.0, t1), rows('drive', t_ev, t1), rows('truth', t_ev - .3, t1)
    rel = lambda t: None if t is None else round(t - t_ev, 3)
    before = [r for r in lk if r[0] < t_ev]
    in_stall = [r for r in lk if t_ev <= r[0] < (t_resume or t1)]
    after = [r for r in lk if t_resume and r[0] >= t_resume]
    drive_zero = next((r[0] for r in dv if not nz(r[1], r[2])), None)
    still = lambda r: abs(r[4]) <= .01 and abs(r[5]) <= .02
    p0 = next((r for r in reversed(tr) if r[0] <= t_ev), None)
    # FIRST standstill after the event: from this sample on the robot stays still for >= 0.3 s (or until the Gate resumes)
    post = [r for r in tr if r[0] >= t_ev]
    t_stop = None
    for i, r in enumerate(post):
        win_end = r[0] + .3 if not t_resume else min(r[0] + .3, max(t_resume, r[0] + .05))
        win = [x for x in post[i:] if x[0] <= win_end]
        if still(r) and all(still(x) for x in win) and (win[-1][0] >= win_end - .06): t_stop = r[0]; break
    p_stop = next((r for r in tr if t_stop and r[0] >= t_stop), None)
    moving = [r for r in post if not still(r)]
    return {'last_gate_output_before': (round(before[-1][1], 3), round(before[-1][2], 3)) if before else None,
            'gate_outputs_during_stall': len(in_stall),
            'gate_last_msg_s': rel(max((r[0] for r in lk if r[0] >= t_ev - .05), default=None)),
            'gate_first_output_after_resume': (rel(after[0][0]), round(after[0][1], 3)) if after else None,
            'gate_nonzero_after_resume': sum(1 for r in after if nz(r[1], r[2])),
            'drive_first_zero_s': rel(drive_zero), 'drive_msgs': len(dv),
            'gz_speed_at_event': None if not p0 else round(p0[4], 3), 'gz_stop_s': rel(t_stop),
            'gz_last_moving_s': rel(moving[-1][0]) if moving else None,
            'gz_moved_until_stop_m': None if not (p0 and p_stop) else round(math.hypot(p_stop[1] - p0[1], p_stop[2] - p0[2]), 3),
            'gz_moved_until_horizon_m': None if not (p0 and tr) else round(math.hypot(tr[-1][1] - p0[1], tr[-1][2] - p0[2]), 3),
            'gz_still_moving_at_horizon': bool(tr) and (abs(tr[-1][4]) > .01 or abs(tr[-1][5]) > .02),
            'gate_states': [(rel(g[0]), g[1]) for g in rows('gate', t_ev - .2, t1)][:6]}


def forward_drive(secs=1.2):
    """Turn ~180 deg in place first (the bench is forward-only), then drive forward: repeated tests oscillate in place."""
    lease(); assert ensure_mode('MANUAL'), 'MANUAL not reached'
    wait_until(lambda: gate_now() == 'NORMAL', 30, .5); time.sleep(1.5)
    for attempt in range(10):
        try: m = Manual(0.0, 1.0).start(); break
        except HttpError as e:
            if 'not_ready' not in str(e) or attempt == 9: raise
            time.sleep(1.0)
    th0 = truth[0][3]; t0 = time.monotonic()
    while time.monotonic() - t0 < 20 and abs(math.atan2(math.sin(truth[0][3] - th0), math.cos(truth[0][3] - th0))) < 2.9: time.sleep(.1)
    m.stop(); time.sleep(1.0)
    m = Manual(1.0, 0.0).start(); time.sleep(secs)
    if truth[0][4] < .05:
        m.stop(); raise AssertionError(f'not moving forward before the event (v={truth[0][4]:.3f}, gate={gate_now()})')
    return m


def gate_pid(): return owned(r'stall_detector_node\.py', ['stack'])


def _stall(name, d, both=False):
    victims = [gate_pid()] + ([owned(r'motion_watchdog', ['watchdog'])] if both else [])
    m = forward_drive(); t_ev = time.monotonic()
    for v in victims: os.kill(v, signal.SIGSTOP)
    time.sleep(d)
    for v in victims: os.kill(v, signal.SIGCONT)
    t_res = time.monotonic(); dm = drive_metrics(t_ev, t_res, max(3.0, d + 2.5)); m.stop()
    ok = (DRIVE_TIMEOUT == 0) or (dm['drive_first_zero_s'] is not None and dm['drive_first_zero_s'] <= DRIVE_TIMEOUT + .1
                                  and dm['gz_stop_s'] is not None and dm['gz_stop_s'] <= DRIVE_TIMEOUT + .4)
    R.record(name, ok, drive_timeout_s=DRIVE_TIMEOUT, stall_s=d, stopped=('gate+watchdog' if both else 'gate'), **dm, gate_after=gate_now(),
             verdict=None if DRIVE_TIMEOUT > 0 else 'MEASUREMENT: no lower timeout (Ignition DiffDrive holds the last command)')


@scenario
def d01_gate_sigstop_0_35s_forward(): _stall('d01_gate_sigstop_0_35s_forward', .35)


@scenario
def d02_gate_sigstop_0_5s_forward(): _stall('d02_gate_sigstop_0_5s_forward', .5)


@scenario
def d03_gate_sigstop_2s_forward(): _stall('d03_gate_sigstop_2s_forward', 2.0)


@scenario
def d04_gate_and_watchdog_sigstop_2s_forward(): _stall('d04_gate_and_watchdog_sigstop_2s_forward', 2.0, both=True)


@scenario
def d05_watchdog_sigkill_forward():
    m = forward_drive(); t_ev = time.monotonic()
    subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), 'kill', 'watchdog'], check=False)
    dm = drive_metrics(t_ev, None, 3.0); m.stop()
    subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), 'start', 'watchdog'], check=False)
    # the killed Watchdog's DDS ghost makes the Gate latch NOT_EXCLUSIVE a few seconds AFTER the restart (run 1 of this series)
    latched = wait_until(lambda: gate_now() not in ('NORMAL', 'STALL_PENDING'), 25, .5); gate_guard('d05_after', 150)
    R.record('d05_watchdog_sigkill_forward', dm['gz_stop_s'] is not None and dm['gz_stop_s'] < 1.5, ghost_latch_seen=bool(latched), gate_after=gate_now(), drive_timeout_s=DRIVE_TIMEOUT, **dm,
             note='Gate alive: the Gate itself zeroes on stale input; the drive timeout is not expected to act')


@scenario
def d06_gate_sigkill_forward():
    """LAST scenario of a run: the Gate process is gone afterwards (the stack launch does not respawn it)."""
    m = forward_drive(); t_ev = time.monotonic(); os.kill(gate_pid(), signal.SIGKILL)
    dm = drive_metrics(t_ev, None, 3.0); m.stop()
    ok = (DRIVE_TIMEOUT == 0) or (dm['drive_first_zero_s'] is not None and dm['drive_first_zero_s'] <= DRIVE_TIMEOUT + .1
                                  and dm['gz_stop_s'] is not None and dm['gz_stop_s'] <= DRIVE_TIMEOUT + .4)
    R.record('d06_gate_sigkill_forward', ok, drive_timeout_s=DRIVE_TIMEOUT, **dm,
             verdict=None if DRIVE_TIMEOUT > 0 else 'MEASUREMENT: no lower timeout (Ignition DiffDrive holds the last command)')


@scenario
def f05_lidar_stop_during_manual():
    bpid = owned(r'gz_scan_bridge|__node:=gz_scan_bridge|/scan@sensor_msgs', ['sim'])
    m = in_manual_drive(); t_ev = time.monotonic(); os.kill(bpid, signal.SIGSTOP)
    sm = stop_metrics(t_ev, 2.5); m.stop(); os.kill(bpid, signal.SIGCONT)
    wait_until(lambda: len(rows('scan', time.monotonic() - 1)) > 3, 20)
    R.record('f05_lidar_stop_during_manual', sm['gate_last_nonzero_s'] is not None and sm['gate_last_nonzero_s'] < 1.0
             and (sm['gz_last_moving_s'] or 0) < 1.5 and any('SENSOR' in str(g[1]) or 'FAILSAFE' in str(g[1]) for g in sm['gate_states']),
             stop=sm, method='SIGSTOP of the /scan bridge process (LiDAR data stops; Gazebo sensor itself keeps running)')


def nav_moving():
    lease(); assert ensure_mode('NAVIGATION')
    if loc().get('status') != 'CONFIRMED':
        for attempt in range(4):        # a killed map_server/amcl stays a DDS ghost ('graph occupied') for a while (as nav_e2e)
            navreq('LOAD', map_version_id=MAP['id'])
            if wait_until(lambda: loc().get('status') in ('WAIT_INITIAL_POSE', 'FAILED') and loc().get('status'), 45) == 'WAIT_INITIAL_POSE': break
            time.sleep(10)
        p = mpose(); navreq('INITIAL_POSE', x=p[0], y=p[1], yaw=p[2]); assert wait_until(lambda: loc().get('status') == 'CONFIRMED', 45), 'relocalization failed'
    g = find_goal(2.0, 3.2) or find_goal(1.2, 2.4); assert g, 'no goal'
    mid = mission(*g)['mission_id']
    assert wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2' and truth[0][4] > .05, 40), 'not moving under Nav2'
    time.sleep(1.0); return mid, g


@scenario
def f06_localization_failure_during_goal():
    mid, g = nav_moving(); apid = owned(r'(^|/)amcl( |$)', ['agent'])
    t_ev = time.monotonic(); os.kill(apid, signal.SIGKILL)
    sm = stop_metrics(t_ev, 3.0); final = wait_mission(mid, 30)
    R.record('f06_localization_failure_during_goal', final in TERMINAL and final != 'SUCCEEDED' and (sm['gz_last_moving_s'] or 0) < 2.0,
             mission=final, localization={k: loc().get(k) for k in ('status', 'detail')}, stop=sm)


@scenario
def f07_cancel_unanswered_bt_navigator_frozen():
    mid, g = nav_moving(); bpid = owned(r'bt_navigator', ['stack'])
    os.kill(bpid, signal.SIGSTOP); t_ev = time.monotonic()
    try:
        OP.post(f'/missions/{mid}/cancel', {}); sm = stop_metrics(t_ev, 4.0); st = mstatus(mid)
    finally: os.kill(bpid, signal.SIGCONT)
    final = wait_mission(mid, 40)
    R.record('f07_cancel_unanswered_bt_navigator_frozen', (sm['gz_last_moving_s'] or 0) < 2.0 and (sm['gate_input_last_nonzero_s'] or 0) < 1.5,
             mission_while_frozen=st, mission_final=final, stop=sm)


@scenario
def f08_external_publisher_injection():
    lease(); assert ensure_mode('MANUAL'); time.sleep(1.0)
    out = {}
    for topic in ('/cmd_vel_raw', '/fleet/nav_cmd_vel', '/cmd_vel'):
        pub = rogue.create_publisher(Twist, topic, 10); tw = Twist(); tw.linear.x = .12
        p0 = pose(); t0 = time.monotonic()
        while time.monotonic() - t0 < 1.5: pub.publish(tw); time.sleep(.05)
        rogue.destroy_publisher(pub); time.sleep(1.0); p1 = pose()
        out[topic] = {'gate_nonzero': sum(1 for r in rows('locked', t0 + .3) if nz(r[1], r[2])), 'gz_moved_m': round(math.hypot(p1[0] - p0[0], p1[1] - p0[1]), 3),
                      'gate': gate_now(), 'watchdog': (watch[0] or {}).get('state'),
                      'blockers': (state().get('velocity_arbiter') or {}).get('blocker')}
        gate_guard('f08_' + topic, 40); wait_until(lambda: (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED', 30)
    R.record('f08_external_publisher_injection', all(o['gz_moved_m'] < .02 and o['gate_nonzero'] == 0 for o in out.values()), injected=out,
             note='/cmd_vel_locked (the Gazebo motor topic itself) is NOT injected: below the Gate there is no software guard to measure (see report)')


def main():
    try:
        # d06 kills the Gate and the stack launch does not respawn it: it must be the LAST scenario (its docstring). It was
        # registered before f05..f08, which then always ran without a Gate (fail-closed, 4 FAIL per full run). 2026-11-21.
        for fn in [f for f in SCEN if f.__name__ != 'd06_gate_sigkill_forward'] + [f for f in SCEN if f.__name__ == 'd06_gate_sigkill_forward']: run_one(fn)
    finally:
        try: (OUT / 'samples.json').write_text(json.dumps({k: v[-40000:] for k, v in S.items()}))
        except Exception as e: say('samples dump failed', e)
        try: fclass = classify_failures()
        except Exception as e: fclass = {'error': repr(e)[:200]}
        (OUT / 'result.json').write_text(json.dumps({'summary': R.counts(), 'failure_classes': fclass, 'results': R.items, 'gate_events': GATE_EVENTS,
                                                     'recovery_events': RECOVERY_EVENTS, 'home_events': HOME_EVENTS, 'marks': MARKS, 'backend': 'gazebo-fortress office_limo.world',
                                                     'stack_env': {k: STACK_ENV[k] for k in ('SIM_BACKEND', 'NAV_LOCALIZATION', 'GATE_SCRIPT', 'GATE_INPUT_EXCL', 'IGN_PARTITION', 'ODOM_ACTIVE_TOPIC', 'GZ_DRIVE_TIMEOUT_S')}},
                                                    indent=1, default=str))
        say('SUMMARY', R.counts()); say('FAILURE_CLASSES', json.dumps(fclass))
        subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), 'stopall'], check=False)
        shutdown_ros()


def shutdown_ros():
    """Stop the executor, JOIN its spin thread, then destroy nodes and shut rclpy down. Without the join the daemon spin thread is
    still inside rcl wait when the context is torn down and the interpreter exits -> C++ abort, rc=134 (separation experiment
    2026-11-21: shutdown-only 12/12 rc=134, +destroy 12/12 rc=134, +join 12/12 rc=0)."""
    if TIMELINE and TIMELINE.poll() is None:
        TIMELINE.send_signal(signal.SIGINT)
        try: TIMELINE.wait(10)
        except subprocess.TimeoutExpired: TIMELINE.kill()
    ex.shutdown(); SPIN.join(10)
    for n in (obs, rogue,):
        try: n.destroy_node()
        except Exception: pass
    rclpy.try_shutdown()
    if SPIN.is_alive(): raise SystemExit('executor spin thread did not stop')


if __name__ == '__main__':
    main()
    sys.exit(1 if R.counts().get('FAIL') else 0)       # exit code = verdict (0 only when nothing FAILED and teardown completed)
