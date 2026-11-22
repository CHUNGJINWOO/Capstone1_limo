"""Phase 1 isolated E2E: web panel -> Fleet HTTP -> Agent bench -> VelocityArbiter -> Motion Watchdog -> Safety Gate.

Real components: Fleet server (in-process, ephemeral DB/tokens, loopback OS-assigned port), Robot Agent process
(mode=mock, simulated, manual_web_bench + manual_locked_arbiter + require_motion_watchdog), Motion Watchdog process,
Safety Gate code from the field bundle (stall_detector_node, /cmd_vel remapped to /cmd_vel_locked, no driver),
real rclpy NavigateToPose/FollowPath ActionServers (status on change only, like Nav2), optional Chrome via Playwright.
Synthetic: LiDAR/odom/estimator/depth topics (constant), the 'foreign' Nav2 client, the synthetic Gamepad.

Run only on an isolated localhost ROS domain. Never touches /cmd_vel, ports 8080/18190-18192 or a robot.
Usage: MANUAL_E2E_OUT=<new dir> ROS_DOMAIN_ID=<id> ROS_LOCALHOST_ONLY=1 python3 run_manual_e2e.py [--browser]
"""
import json, math, os, signal, subprocess, sys, tempfile, threading, time, traceback, urllib.request
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
FLEET = HERE.parents[1]
REPO = FLEET.parent
sys.path[:0] = [str(FLEET / 'server'), str(FLEET / 'agent' / 'limo_robot_agent')]

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from nav2_msgs.action import FollowPath, NavigateToPose
from fleet_server.auth import Auth, init_tokens
from fleet_server.db import DB
from fleet_server.http_api import Servers
from fleet_server.service import Fleet
from limo_robot_agent.core import Http, HttpError

DOMAIN = os.environ.get('ROS_DOMAIN_ID')
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and DOMAIN not in (None, '', '0', '77'), 'isolated domain required'
assert not os.environ.get('ROS_DISCOVERY_SERVER'), 'discovery server must be unset'
OUT = Path(os.environ['MANUAL_E2E_OUT']).resolve(); OUT.mkdir(parents=True, exist_ok=False)
BROWSER = '--browser' in sys.argv
RID = 'e2e-limo'
GATE = REPO / 'experiments/real_auto_mapping_ready_20261113/src/limo_capstone/limo_capstone/stall_detector_node.py'
if os.environ.get('GATE_SCRIPT'):                 # 2026-11-20: Gate revision under test (default above unchanged)
    GATE = Path(os.environ['GATE_SCRIPT']).resolve()
GATE_INPUT = []
if os.environ.get('GATE_INPUT_EXCL'):             # r11+: /cmd_vel_raw input-source validation parameters
    GATE_INPUT = ['-p', f"input_exclusive_publishers:=[{os.environ['GATE_INPUT_EXCL']}]"]
    for k, e in (('input_max_abs_linear_x', 'GATE_INPUT_MAX_VX'), ('input_max_abs_angular_z', 'GATE_INPUT_MAX_WZ')):
        if os.environ.get(e): GATE_INPUT += ['-p', f'{k}:={float(os.environ[e])}']
if os.environ.get('GATE_WRITER_STATUS'):          # 2026-11-22: r13 candidate, bind the input writer only as its own status reports it
    GATE_INPUT += ['-p', f"input_writer_status_topic:={os.environ['GATE_WRITER_STATUS']}"]
PRIVATE = Path(tempfile.mkdtemp(prefix='manual-e2e-'))
init_tokens(str(PRIVATE), str(PRIVATE), [RID])
CFG = SimpleNamespace(offline_after_s=3, command_ttl_s=120, resend_after_s=1)
results = []
log = (OUT / 'harness.log').open('x')


def say(*a):
    line = time.strftime('%H:%M:%S ') + ' '.join(str(x) for x in a)
    print(line, flush=True); log.write(line + '\n'); log.flush()


# ------------------------------------------------------------------ Fleet server (restartable on the same port)
class Server:
    def __init__(self):
        self.port = 0; self.start()

    def start(self):
        self.fleet = Fleet(DB(str(PRIVATE / 'fleet.db')), CFG)
        self.srv = Servers(self.fleet, Auth(str(PRIVATE)), ['127.0.0.1'], self.port)
        self.port = self.srv.srvs[0].server_port
        assert self.port not in (8080, 18190, 18191, 18192)
        self.thread = threading.Thread(target=self.srv.serve_forever, daemon=True); self.thread.start()

    def stop(self):
        self.srv.stop_evt.set(); self.thread.join(timeout=5)


SERVER = Server()
URL = f'http://127.0.0.1:{SERVER.port}'
tok = lambda n: (PRIVATE / f'{n}.token').read_text().strip()
OP, ADMIN, VIEWER = Http(URL, tok('operator')), Http(URL, tok('admin')), Http(URL, tok('viewer'))
for h in (OP, ADMIN, VIEWER):
    h.timeout = 2.0

# ------------------------------------------------------------------ server responsiveness probe (diagnostics)
STALLS = []


def responsiveness_probe():
    import sys as _s, traceback as _t
    while True:
        t0 = time.monotonic(); box = {}
        def lock_probe():
            with SERVER.fleet.lock: box['ok'] = time.monotonic()
        th = threading.Thread(target=lock_probe, daemon=True); th.start(); th.join(1.0)
        n_threads = threading.active_count()
        if n_threads > 40 and (not STALLS or STALLS[-1].get('kind') != 'threads' or time.time() - STALLS[-1]['t'] > 2):
            STALLS.append({'t': time.time(), 'kind': 'threads', 'count': n_threads,
                           'names': sorted({t.name.split('-')[0] + ':' + str(getattr(t, '_target', None).__qualname__ if getattr(t, '_target', None) else '') for t in threading.enumerate()})})
        if 'ok' not in box:
            frames = {tid: ''.join(_t.format_stack(f)[-6:]) for tid, f in _s._current_frames().items()}
            names = {t.ident: t.name for t in threading.enumerate()}
            STALLS.append({'t': time.time(), 'kind': 'fleet_lock_wait_gt_1s',
                           'stacks': {names.get(k, str(k)): v for k, v in frames.items()}})
            th.join(10)
        time.sleep(.2)


threading.Thread(target=responsiveness_probe, daemon=True, name='probe').start()

# ------------------------------------------------------------------ ROS fixture
rclpy.init()
fx = rclpy.create_node('manual_e2e_fixture')
foreign = rclpy.create_node('foreign_nav_client')
nav_client = ActionClient(foreign, NavigateToPose, 'navigate_to_pose')
fp_client = ActionClient(foreign, FollowPath, 'follow_path')
messages = {'/scan': LaserScan(range_min=.1, range_max=8., ranges=[3.] * 20), '/odom': Odometry(), '/odom_nav2': Odometry(),
            '/odometry/lidar_odom': Odometry(), '/diagnostics/f4_latency': Float32MultiArray(data=[0., 0., 0., 220.]),
            '/auto_mapping/depth_status': String(data='{"state":"ready","code":"DEPTH_READY"}')}
pubs = {k: fx.create_publisher(type(m), k, 10) for k, m in messages.items()}
finish_pub = fx.create_publisher(String, '/fake_nav2/finish', 10)
locked, watch = [], []
fx.create_subscription(Twist, '/cmd_vel_locked', lambda m: locked.append((time.monotonic(), time.time(), m.linear.x, m.angular.z)), 50)
fx.create_subscription(String, '/fleet/motion_watchdog/status', lambda m: watch.append((time.monotonic(), json.loads(m.data))), 10)
sensors_on = [True]

children = {}
records = []
JOBS = {
    'gate': [sys.executable, str(GATE), '--ros-args', '-r', '/cmd_vel:=/cmd_vel_locked', '-p', 'estimator_odom_topic:=/odom_nav2', *GATE_INPUT],
    'watchdog': [sys.executable, '-m', 'limo_robot_agent.motion_watchdog'],
    'nav': [sys.executable, str(HERE / 'fake_nav2_servers.py')],
    'agent': [sys.executable, '-m', 'limo_robot_agent.agent_node', '--ros-args', '-p', 'mode:=mock', '-p', 'simulated:=true',
              '-p', 'manual_web_bench:=true', '-p', 'manual_locked_arbiter:=true', '-p', 'require_motion_watchdog:=true',
              '-p', 'allow_missions:=false', '-p', 'server_url:=' + URL, '-p', 'token_file:=' + str(PRIVATE / f'agent-{RID}.token'),
              '-p', 'robot_id:=' + RID, '-p', 'state_dir:=' + str(PRIVATE / 'agent_state'), '-p', 'period_s:=0.10'],
}
ENV = dict(os.environ, PYTHONPATH=os.pathsep.join([str(FLEET / 'agent' / 'limo_robot_agent'), os.environ.get('PYTHONPATH', '')]),
           ROS_LOG_DIR=str(PRIVATE / 'roslog'))


def spawn(name):
    n = sum(1 for r in records if r.name.startswith(str(OUT / name)))
    f = (OUT / (f'{name}.log' if not n else f'{name}_{n}.log')).open('x'); records.append(f)
    children[name] = subprocess.Popen(JOBS[name], stdout=f, stderr=subprocess.STDOUT, start_new_session=True, env=ENV, cwd=str(REPO))


def kill(name, sig=signal.SIGKILL):
    p = children.get(name)
    if p and p.poll() is None:
        os.killpg(p.pid, sig)
        try: p.wait(timeout=6)
        except subprocess.TimeoutExpired: os.killpg(p.pid, signal.SIGKILL); p.wait()


# ------------------------------------------------------------------ pumping and observation
class Driver:
    """Operator-side sample stream (what the panel sends every 80 ms)."""
    def __init__(self):
        self.session = None; self.epoch = None; self.seq = 0; self.v = self.w = 0.; self.dead = False; self.on = False
        self.errors = []; self.next_t = 0.

    def path(self): return f'/robots/{RID}/manual-bench'

    def start(self, epoch):
        self.epoch = epoch
        self.session = OP.post(self.path(), {'action': 'start', 'epoch': epoch})['session_id']; self.seq = 0; self.errors = []
        return self.session

    def set(self, v, w, dead=True): self.v, self.w, self.dead, self.on = v, w, dead, True

    def halt(self): self.on = False

    def stop(self): return OP.post(self.path(), {'action': 'stop', 'epoch': self.epoch, 'session_id': self.session})

    def send_once(self, **override):
        self.seq += 1
        body = {'action': 'sample', 'epoch': self.epoch, 'session_id': self.session, 'seq': self.seq,
                'linear': self.v, 'turn': self.w, 'deadman': self.dead}
        body.update(override)
        return OP.post(self.path(), body)

    def tick(self, now):
        if self.on and self.session and now >= self.next_t:
            self.next_t = now + .08
            try: self.send_once()
            except HttpError as e: self.errors.append((now, e.status, str(e)[:160]))
            except Exception as e: self.errors.append((now, None, repr(e)[:160]))


DRV = Driver()


TIMELINE = []          # (wall time, arbiter blocker, watchdog state) on every change
NEXT_SAMPLE = [0.]


def sample_timeline(now):
    if now < NEXT_SAMPLE[0]: return
    NEXT_SAMPLE[0] = now + .2
    try:
        a = ((SERVER.fleet.robots.get(RID) or {}).get('state') or {}).get('velocity_arbiter') or {}
        key = (a.get('blocker'), watch[-1][1].get('state') if watch else None)
    except Exception:
        return
    if not TIMELINE or TIMELINE[-1][1:] != key:
        TIMELINE.append((round(time.time(), 2),) + key)


def pump(seconds):
    end = time.monotonic() + seconds; nxt = 0.
    while time.monotonic() < end:
        now = time.monotonic()
        DRV.tick(now); sample_timeline(now)
        if now >= nxt and sensors_on[0]:
            for k, p in pubs.items(): p.publish(messages[k])
            nxt = now + .08
        rclpy.spin_once(fx, timeout_sec=.002); rclpy.spin_once(foreign, timeout_sec=.002)


def robot():
    with SERVER.fleet.lock:
        return json.loads(json.dumps(SERVER.fleet.robots.get(RID) or {}, default=str))


def arbiter():
    return ((robot().get('state') or {}).get('velocity_arbiter') or {})


def wd():
    return watch[-1][1] if watch else {}


def faults_since(wall):
    return sorted({x[1].get('last_fault') for x in watch if (x[1].get('fault_wall_time') or 0) > wall and x[1].get('last_fault')})


def wait_until(pred, timeout, step=.1):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            if pred(): return True
        except Exception: pass
        pump(step)
    return False


def fresh_robot():
    # The Fleet restores robots from its DB after a restart: persisted state is not live evidence.
    r = robot()
    return (r.get('online') is True and r.get('reconciled') is True and r.get('state_ts')
            and 0 <= time.time() - float(r['state_ts']) < 1.0)


def wait_ready(timeout=20.):
    ok = wait_until(lambda: fresh_robot() and arbiter().get('ready') is True and wd().get('state') == 'MOTION_AUTHORIZED', timeout)
    return ok, {'arbiter_blocker': arbiter().get('blocker'), 'watchdog': wd().get('state'), 'watchdog_reason': wd().get('reason')}


def nz(v, w): return abs(v) > 1e-6 or abs(w) > 1e-6


GATE_RESETS = []


def gate_unlatch(label):
    """Operator unlatch through the Gate's own /stall_detector/reset_latch service (recorded; never counted as a PASS by itself)."""
    from std_srvs.srv import Trigger
    cli = fx.create_client(Trigger, '/stall_detector/reset_latch')
    ok = wait_until(lambda: cli.service_is_ready(), 3.); res = None
    if ok:
        f = cli.call_async(Trigger.Request()); wait_until(lambda: f.done(), 5.)
        res = f.result().message if f.done() and f.result() else None
    fx.destroy_client(cli); GATE_RESETS.append({'label': label, 'service': ok, 'result': res}); return res


def gate_unlatch_until_ready(label, timeout=45.):
    """r11: a SIGKILLed writer lingers as a DDS ghost (~15-20 s); while the Gate still sees two /cmd_vel_raw writers its reset is
    refused (NOT_EXCLUSIVE stays). Repeat the operator reset until the chain is ready; every attempt is recorded."""
    end = time.monotonic() + timeout; n = 0
    while time.monotonic() < end:
        gate_unlatch(f'{label}#{n}'); n += 1
        if wait_ready(3.)[0]: return {'attempts': n, 'ready': True}
    return {'attempts': n, 'ready': False}


def window(t0, t1=None):
    t1 = time.monotonic() if t1 is None else t1
    return [(t, v, w) for t, _, v, w in locked if t0 <= t <= t1]


def zero_latency(t0, observe=1.2):
    """Seconds from t0 until the Gate output stays zero for the rest of the observation window."""
    pump(observe)
    win = window(t0)
    last_nz = max((t for t, v, w in win if nz(v, w)), default=None)
    samples_after = [s for s in win if last_nz is None or s[0] > last_nz]
    return (None if last_nz is None else round(last_nz - t0, 3)), len(win), len(samples_after)


def moving(t0, need=.10):
    win = window(t0)
    good = [(t, v, w) for t, v, w in win if v > .05]
    return bool(good), (max(v for _, v, _ in win) if win else None), (max(abs(w) for _, _, w in win) if win else None)


def lease(ttl=120):
    return OP.post(f'/robots/{RID}/lock', {'action': 'acquire', 'ttl_s': ttl})['epoch']


def expect_http(fn, status, code=None):
    try: fn()
    except HttpError as e:
        body = str(e)
        return e.status == status and (code is None or code in body), body[:200]
    return False, 'accepted'


def record(name, ok, **detail):
    results.append({'scenario': name, 'result': 'PASS' if ok else 'FAIL', **detail})
    say(('PASS ' if ok else 'FAIL ') + name, json.dumps(detail, default=str)[:600])


def scenario(fn):
    def run():
        DRV.halt()
        try: fn()
        except Exception as e:
            record(fn.__name__, False, error=repr(e)[:300], trace=traceback.format_exc()[-800:])
        finally:
            DRV.halt()
            if DRV.session:                 # the panel always sends STOP when it ends a session
                try: DRV.stop()
                except Exception: pass
            pump(.3)
    run.__name__ = fn.__name__
    return run


def drive_burst(epoch, v=1., w=.5, secs=.9):
    DRV.start(epoch); DRV.set(v, w); t0 = time.monotonic(); pump(secs)
    return t0, moving(t0)


# ------------------------------------------------------------------ scenarios
EPOCH = [None]


@scenario
def s01_startup_silent_nav2_authorizes():
    # Before any Lease the only remaining blocker must be the Lease itself.
    ok = wait_until(lambda: wd().get('state') == 'MOTION_AUTHORIZED' and arbiter().get('blocker') == 'LEASE_MISSING_OR_EXPIRED', 25.)
    d = {'arbiter_blocker': arbiter().get('blocker'), 'watchdog': wd().get('state')}
    w = wd()
    idle = ((robot().get('state') or {}).get('execution') or {}).get('nav_idle') or {}
    record('s01_startup_silent_nav2_authorizes', ok and w.get('nav_status_received') is False and idle.get('ok') is True,
           note='idle action servers never publish status; readiness must come from continuity evidence',
           watchdog_status_received=w.get('nav_status_received'), nav_idle_reason=idle.get('reason'),
           control_mode=SERVER.fleet.robot_view(SERVER.fleet.robots[RID])['control_mode'] if RID in SERVER.fleet.robots else None, **d)


@scenario
def s02_normal_drive_and_bounds():
    EPOCH[0] = lease(); pump(.6)
    ok, d = wait_ready(10.)
    t0, (mv, vmax, wmax) = drive_burst(EPOCH[0], 1., .5, 1.0)
    first = next((t for t, v, w in window(t0) if v > .05), None)
    mode = SERVER.fleet.robot_view(SERVER.fleet.robots[RID])['control_mode']
    record('s02_normal_drive_and_bounds', ok and mv and vmax <= .1000001 and wmax <= .3000001 and mode == 'MANUAL',
           first_motion_latency_s=None if first is None else round(first - t0, 3), max_linear=vmax, max_turn=wmax,
           control_mode_during=mode, driver_errors=DRV.errors[:3], **d)


@scenario
def s03_deadman_release_stops():
    DRV.set(1., .5, dead=False); t0 = time.monotonic()
    lat, n, n_after = zero_latency(t0, 1.0)
    record('s03_deadman_release_stops', (lat is None or lat < .5) and n_after > 10, zero_after_s=lat, gate_samples=n)


@scenario
def s04_input_stall_times_out():
    t0, (mv, *_ ) = drive_burst(EPOCH[0])
    DRV.halt(); t1 = time.monotonic()
    lat, n, n_after = zero_latency(t1, 1.2)
    later = expect_http(lambda: DRV.send_once(), 409, 'manual_bench_expired')
    record('s04_input_stall_times_out', mv and lat is not None and lat < .6 and n_after > 10 and later[0],
           moved=mv, zero_after_s=lat, stale_session_rejected=later)


@scenario
def s05_stop_button():
    t0, (mv, *_ ) = drive_burst(EPOCH[0])
    DRV.halt(); t1 = time.monotonic(); r = DRV.stop()
    lat, n, n_after = zero_latency(t1, 1.0)
    old = expect_http(lambda: DRV.send_once(), 409)
    record('s05_stop_button', mv and r.get('stopped') is True and (lat is None or lat < .45) and old[0],
           moved=mv, zero_after_s=lat, old_session=old)


@scenario
def s06_bad_inputs_rejected():
    DRV.start(EPOCH[0])
    nan = expect_http(lambda: OP.post(DRV.path(), json.loads(json.dumps({'action': 'sample', 'epoch': EPOCH[0], 'session_id': DRV.session,
                      'seq': 1, 'linear': float('nan'), 'turn': 0., 'deadman': True}))), 400)
    big = expect_http(lambda: DRV.send_once(linear=1.5), 400)
    DRV.send_once(linear=0., turn=0., deadman=False)
    replay = expect_http(lambda: DRV.send_once(seq=1), 400)
    viewer = expect_http(lambda: VIEWER.post(DRV.path(), {'action': 'start', 'epoch': EPOCH[0]}), 403)
    other = expect_http(lambda: ADMIN.post(DRV.path(), {'action': 'start', 'epoch': EPOCH[0]}), 409, 'lease_required')
    busy = expect_http(lambda: OP.post(DRV.path(), {'action': 'start', 'epoch': EPOCH[0]}), 409, 'manual_bench_busy')
    DRV.stop()
    record('s06_bad_inputs_rejected', all(x[0] for x in (nan, big, replay, viewer, other)),
           nan=nan, out_of_range=big, replayed_seq=replay, viewer=viewer, non_owner=other,
           second_browser_within_ttl=busy, note='second-browser check is informational: it races the 250 ms TTL')


@scenario
def s07_mode_exclusion_mission_and_mapping():
    t0, (mv, *_ ) = drive_burst(EPOCH[0], secs=.6)
    mission = expect_http(lambda: OP.post('/missions', {'robot_id': RID, 'type': 'navigate_to_pose', 'goal': {'frame_id': 'map', 'x': 1, 'y': 0},
                                                        'lease_epoch': EPOCH[0], 'idempotency_key': 'e2e-' + str(time.time())}), 409, 'manual_active')
    still, *_ = moving(time.monotonic() - .3)
    orig = SERVER.fleet.mapping_blocks_motion
    SERVER.fleet.mapping_blocks_motion = lambda rid: True       # synthetic: mapping session becomes non-final
    t1 = time.monotonic()
    try:
        lat, n, n_after = zero_latency(t1, 1.0)
        mode = SERVER.fleet.robot_view(SERVER.fleet.robots[RID])['control_mode']
        restart = expect_http(lambda: OP.post(DRV.path(), {'action': 'start', 'epoch': EPOCH[0]}), 409, 'mapping_active_or_unknown')
    finally:
        SERVER.fleet.mapping_blocks_motion = orig
    DRV.halt()
    record('s07_mode_exclusion_mission_and_mapping', mv and mission[0] and still and lat is not None and lat < .6 and restart[0] and mode == 'MAPPING',
           mission_during_manual=mission, manual_kept_moving_after_rejected_mission=still, mapping_zero_after_s=lat,
           control_mode=mode, manual_start_during_mapping=restart, note='mapping state injected via mapping_blocks_motion (synthetic)')


@scenario
def s08_auto_mission_blocks_manual():
    db = SERVER.fleet.db
    with SERVER.fleet.lock, db.transaction():
        db.x("INSERT INTO missions(mission_id,robot_id,type,goal_json,idempotency_key,status,detail,created_by,note,created_at,updated_at) "
             "VALUES('m_e2e_auto',?,'navigate_to_pose','{}',NULL,'EXECUTING','synthetic','operator','',?,?)", (RID, time.time(), time.time()))
    mode = SERVER.fleet.robot_view(SERVER.fleet.robots[RID])['control_mode']
    rej = expect_http(lambda: OP.post(DRV.path(), {'action': 'start', 'epoch': EPOCH[0]}), 409, 'robot_busy')
    with SERVER.fleet.lock, db.transaction():
        db.x("UPDATE missions SET status='CANCELED' WHERE mission_id='m_e2e_auto'")
    pump(.3)
    t0, (mv, *_ ) = drive_burst(EPOCH[0], secs=.6); DRV.halt(); DRV.stop()
    record('s08_auto_mission_blocks_manual', mode == 'AUTO' and rej[0] and mv, control_mode=mode, manual_during_auto=rej,
           manual_after_mission_terminal=mv, note='mission row inserted directly (mock Agent has missions disabled)')


@scenario
def s09_foreign_nav_goal_stops_then_recovers():
    t0, (mv, *_ ) = drive_burst(EPOCH[0], secs=.6)
    assert nav_client.wait_for_server(timeout_sec=3)
    t1 = time.monotonic(); fut = nav_client.send_goal_async(NavigateToPose.Goal())
    lat, n, n_after = zero_latency(t1, 1.2)
    blocker = arbiter().get('blocker'); w = wd()
    trips = faults_since(time.time() - (time.monotonic() - t1))
    handle = fut.result() if fut.done() else None
    canceled = wait_until(lambda: (wd().get('nav_active') in (0, None)), 6.)
    DRV.halt()
    ok_ready, d = wait_ready(15.)
    old = expect_http(lambda: DRV.send_once(), 409)
    t2, (mv2, *_ ) = drive_burst(EPOCH[0], secs=.8); DRV.halt(); DRV.stop()
    record('s09_foreign_nav_goal_stops_then_recovers', mv and lat is not None and lat < .7 and ok_ready and old[0] and mv2,
           zero_after_s=lat, arbiter_blocker=blocker, watchdog_fault='FOREIGN_NAV_GOAL' in trips, watchdog_state=w.get('state'),
           goal_accepted=getattr(handle, 'accepted', None), goal_canceled_by_watchdog=canceled, old_session=old,
           new_session_moves=mv2, recovery=d)


@scenario
def s10_foreign_followpath_stops():
    t0, (mv, *_ ) = drive_burst(EPOCH[0], secs=.6)
    assert fp_client.wait_for_server(timeout_sec=3)
    t1 = time.monotonic(); fut = fp_client.send_goal_async(FollowPath.Goal())
    lat, n, n_after = zero_latency(t1, 1.2)
    blocker = arbiter().get('blocker')
    DRV.halt()
    handle = fut.result() if fut.done() else None
    if handle and handle.accepted:
        cf = handle.cancel_goal_async(); wait_until(lambda: cf.done(), 3.)
    ok_ready, d = wait_ready(15.)
    t2, (mv2, *_ ) = drive_burst(EPOCH[0], secs=.6); DRV.halt(); DRV.stop()
    # 2026-11-21 diagnostics only (assertion unchanged): Gate output around the goal and the drive before it, for the open
    # 'zero_after_s=None' case seen with Gate r11 (no non-zero sample after t1).
    diag = {'drive_before': [(round(t - t1, 3), round(v, 3), round(w, 3)) for t, v, w in window(t0, t1)][-12:],
            'after_goal': [(round(t - t1, 3), round(v, 3), round(w, 3)) for t, v, w in window(t1, t1 + .4)][:20],
            'moved_before_goal': mv, 'goal_accepted': bool(handle and handle.accepted),
            'blocker_timeline': [x for x in TIMELINE if x[0] >= time.time() - (time.monotonic() - t0) - .5][:12]}
    # 2026-11-21: zero_latency() returns None when NO non-zero sample follows t1, i.e. the stop happened before the first sample after
    # the goal (diagnostics of closing/phase1_r11: non-zero 8 ms before t1, first sample +13 ms already zero). Count that as a stop
    # bounded by the first post-goal sample ONLY when the output was moving right before t1 and that sample is zero.
    after = window(t1, t1 + .4); before = window(t1 - .1, t1)
    stopped_before_first_sample = lat is None and bool(before) and nz(before[-1][1], before[-1][2]) and bool(after) and not nz(after[0][1], after[0][2])
    lat_eff = round(after[0][0] - t1, 3) if stopped_before_first_sample else lat
    record('s10_foreign_followpath_stops', mv and lat_eff is not None and lat_eff < .7 and blocker == 'FOLLOW_PATH_GOAL_ACTIVE' and ok_ready and mv2,
           zero_after_s=lat, zero_bound_s=lat_eff, stopped_before_first_sample=stopped_before_first_sample,
           arbiter_blocker=blocker, recovered=d, new_session_moves=mv2, diag=diag,
           note='watchdog does not observe FollowPath; the Agent arbiter blocks it')


@scenario
def s11_nav2_server_restart():
    w1 = time.time(); kill('nav', signal.SIGINT); t1 = time.monotonic()
    blocked = wait_until(lambda: arbiter().get('ready') is False, 5.)
    b = arbiter().get('blocker')
    rej = expect_http(lambda: OP.post(DRV.path(), {'action': 'start', 'epoch': EPOCH[0]}), 409)
    spawn('nav')
    ok_ready, d = wait_ready(25.)
    faults = faults_since(w1)
    t2, (mv, *_ ) = drive_burst(EPOCH[0], secs=.6); DRV.halt(); DRV.stop()
    record('s11_nav2_server_restart', blocked and rej[0] and ok_ready and mv, arbiter_blocker_while_down=b, start_while_down=rej,
           watchdog_faults=faults, recovered=d, new_session_moves=mv)


@scenario
def s12_lease_expiry_and_force_release():
    t0, (mv, *_ ) = drive_burst(EPOCH[0], secs=.6)
    with SERVER.fleet.lock, SERVER.fleet.db.transaction():
        SERVER.fleet.db.x('UPDATE locks SET expires_at=0 WHERE robot_id=?', (RID,))
    t1 = time.monotonic(); lat, n, n_after = zero_latency(t1, 1.0)
    after = expect_http(lambda: DRV.send_once(), 409)
    DRV.halt()
    EPOCH[0] = lease(); pump(.5); wait_ready(10.)
    t2, (mv2, *_ ) = drive_burst(EPOCH[0], secs=.6)
    ADMIN.post(f'/robots/{RID}/lock', {'action': 'force_release'}); t3 = time.monotonic()
    lat2, *_ = zero_latency(t3, 1.0)
    after2 = expect_http(lambda: DRV.send_once(), 409)
    DRV.halt()
    EPOCH[0] = lease(); pump(.5)
    record('s12_lease_expiry_and_force_release', mv and lat is not None and lat < .6 and after[0] and mv2 and lat2 is not None and lat2 < .6 and after2[0],
           expiry_zero_after_s=lat, input_after_expiry=after, force_release_zero_after_s=lat2, input_after_force_release=after2)


@scenario
def s13_agent_sigkill_and_restart():
    t0, (mv, *_ ) = drive_burst(EPOCH[0], secs=.6)
    w1 = time.time(); kill('agent'); t1 = time.monotonic()
    lat, n, n_after = zero_latency(t1, 1.5)
    faults = faults_since(w1)
    DRV.halt()
    spawn('agent')
    ok_ready, d = wait_ready(30.)
    old = expect_http(lambda: DRV.send_once(), 409)
    EPOCH[0] = lease(); pump(.3)
    t2, (mv2, *_ ) = drive_burst(EPOCH[0], secs=.6); DRV.halt(); DRV.stop()
    record('s13_agent_sigkill_and_restart', mv and lat is not None and lat < .7 and ok_ready and old[0] and mv2,
           zero_after_s=lat, watchdog_faults=faults, recovered=d, old_session=old, new_session_moves=mv2)


@scenario
def s14_watchdog_sigkill_and_restart():
    t0, (mv, *_ ) = drive_burst(EPOCH[0], secs=.6)
    kill('watchdog'); t1 = time.monotonic()
    lat, n, n_after = zero_latency(t1, 1.5)
    blocker = None
    wait_until(lambda: arbiter().get('ready') is False, 3.); blocker = arbiter().get('blocker')
    DRV.halt()
    spawn('watchdog')
    held = None
    if GATE_INPUT:
        # 2026-11-20 r11: the restarted watchdog is a NEW /cmd_vel_raw writer (new GID). The Gate must not transfer the old binding,
        # so no motion may be possible until the operator Gate reset. Original expectation (automatic recovery) kept for GATE_INPUT off;
        # original file: local_data/raw_results/gate_input_claude_20261120/pre_change_originals/run_manual_e2e.py
        held_ok, hd = wait_ready(6.); tb = time.monotonic()
        rej = expect_http(lambda: OP.post(DRV.path(), {'action': 'start', 'epoch': EPOCH[0]}), 409)   # web start must be refused
        mvb = bool([s for s in window(tb) if nz(s[1], s[2])])
        held = {'ready_before_reset': held_ok, 'start_rejected': rej, 'moved_before_reset': mvb, 'detail': hd,
                'gate_reset': gate_unlatch_until_ready('s14_after_watchdog_restart')}
    ok_ready, d = wait_ready(25.)
    t2, (mv2, *_ ) = drive_burst(EPOCH[0], secs=.6); DRV.halt(); DRV.stop()
    record('s14_watchdog_sigkill_and_restart', mv and lat is not None and lat < .7 and ok_ready and mv2
           and (held is None or (not held['moved_before_reset'] and not held['ready_before_reset'] and held['start_rejected'][0])),
           zero_after_s=lat, arbiter_blocker=blocker, recovered=d, new_session_moves=mv2, held_until_gate_reset=held)


@scenario
def s15_fleet_server_outage_and_restart():
    global OP, ADMIN, VIEWER
    t0, (mv, *_ ) = drive_burst(EPOCH[0], secs=.6)
    DRV.halt()
    SERVER.stop(); t1 = time.monotonic()
    lat, n, n_after = zero_latency(t1, 1.5)
    SERVER.start()                         # new Fleet boot, same DB and port
    ok_ready, d = wait_ready(30.)
    old = expect_http(lambda: DRV.send_once(), 409)
    EPOCH[0] = lease(); pump(.5); wait_ready(10.)
    t2, (mv2, *_ ) = drive_burst(EPOCH[0], secs=.6); DRV.halt(); DRV.stop()
    record('s15_fleet_server_outage_and_restart', mv and lat is not None and lat < .7 and ok_ready and old[0] and mv2,
           zero_after_s=lat, recovered=d, old_session=old, new_session_moves=mv2)


@scenario
def s16_browser_keyboard_pointer_gamepad():
    if not BROWSER:
        results.append({'scenario': 's16_browser_keyboard_pointer_gamepad', 'result': 'SKIP', 'reason': '--browser not given'}); return
    EPOCH[0] = lease(); pump(.3); wait_ready(10.)
    env = dict(os.environ, NODE_PATH=os.environ.get('MANUAL_E2E_NODE_PATH', ''))
    f = (OUT / 'browser.log').open('x'); records.append(f)
    p = subprocess.Popen(['node', str(HERE / 'browser_manual_inputs.cjs'), URL, str(PRIVATE / 'operator.token'), str(OUT)],
                         stdout=subprocess.PIPE, stderr=f, env=env, text=True)
    marks = {}
    def reader():
        for line in p.stdout:
            f.write(line); f.flush()
            try: m = json.loads(line); marks[m['mark']] = m
            except Exception: pass
    th = threading.Thread(target=reader, daemon=True); th.start()
    end = time.monotonic() + 90
    while p.poll() is None and time.monotonic() < end: pump(.1)
    if p.poll() is None: p.kill()
    th.join(timeout=2); pump(.3)
    def seg(a, b):
        if a not in marks or b not in marks: return None
        return [(v, w) for _, t, v, w in locked if marks[a]['t'] <= t <= marks[b]['t']]
    def zero_after(name, delay=.45, span=.4, until=None):
        # `until`: the next input phase's start mark. The window must not reach into the next source's own motion
        # (2026-11-21: ptr_off + 0.85 s overlapped pad_on by 75 ms and failed a correct release; Phase 3.5 regression).
        if name not in marks: return None
        t = marks[name]['t']; end = t + delay + span
        if until in marks: end = min(end, marks[until]['t'])
        win = [(v, w) for _, tw, v, w in locked if t + delay <= tw <= end]
        if end - (t + delay) < .15 or not win: return None      # too short / no samples: not evidence of a stop
        return all(not nz(v, w) for _, tw, v, w in locked if t + delay <= tw <= end)
    out = {}
    for src, on, off, nxt in (('keyboard', 'kb_on', 'kb_off', 'ptr_on'), ('pointer', 'ptr_on', 'ptr_off', 'pad_on'),
                              ('gamepad', 'pad_on', 'pad_off', 'padfreeze_on')):
        s = seg(on, off) or []
        out[src] = {'moved': any(v > .05 for v, w in s), 'max_linear': max((v for v, w in s), default=None),
                    'turn_sign': (lambda ws: None if not ws else (1 if max(ws) > .05 else (-1 if min(ws) < -.05 else 0)))([w for v, w in s]),
                    'zero_after_release': zero_after(off, until=nxt)}
    frozen = zero_after('padfreeze_frozen', .5, .3)
    mob = marks.get('mobile_layout') or {}
    ok = (p.returncode == 0 and all(o['moved'] and o['zero_after_release'] for o in out.values()) and frozen
          and out['keyboard']['turn_sign'] == 1 and out['pointer']['turn_sign'] == 1 and out['gamepad']['turn_sign'] == -1
          and mob.get('docScroll', 9999) <= mob.get('viewport', 0) + 1)
    record('s16_browser_keyboard_pointer_gamepad', ok, inputs=out, gamepad_freeze_zero=frozen,
           pad_freeze_status=(marks.get('padfreeze_status') or {}).get('status'), mobile=mob,
           errors=[m for k, m in marks.items() if k in ('error', 'pageerror')], browser_rc=p.returncode,
           note='gamepad is a synthetic navigator.getGamepads object')


@scenario
def s18_gate_input_rogue_raw_in_manual():
    """2026-11-20: a foreign /cmd_vel_raw writer while the web MANUAL path is driving. Asserted with every Gate:
    the rogue value must never appear on the locked output and nothing may move 0.3 s after it starts."""
    t0, (mv, *_ ) = drive_burst(EPOCH[0], secs=.6)
    raw = foreign.create_publisher(Twist, '/cmd_vel_raw', 10); m = Twist(); m.linear.x = .123
    t1 = time.monotonic(); end = t1 + 2.5
    while time.monotonic() < end:
        raw.publish(m); DRV.tick(time.monotonic()); pump(.05)
    foreign.destroy_publisher(raw); DRV.halt(); t2 = time.monotonic()
    try: DRV.stop()
    except HttpError: pass                 # the Gate fault may already have ended the bench session
    rogue_vals = [s for s in locked if s[0] >= t1 and abs(s[2] - .123) < 1e-4]
    late = [s for s in window(t1 + .3, t2 + .5) if nz(s[1], s[2])]
    pump(1.)
    res = gate_unlatch_until_ready('s18_after_rogue') if GATE_INPUT else None
    ok_ready, d = wait_ready(25.)
    t3, (mv2, *_ ) = drive_burst(EPOCH[0], secs=.6); DRV.halt(); DRV.stop()
    record('s18_gate_input_rogue_raw_in_manual', mv and not rogue_vals and not late and ok_ready and mv2,
           rogue_value_samples=len(rogue_vals), nonzero_after_0_3s=len(late), gate_reset=res, recovered=d, moves_after=mv2)


@scenario
def s17_gate_sigkill():
    t0, (mv, *_ ) = drive_burst(EPOCH[0], secs=.6)
    w1 = time.time(); kill('gate'); t1 = time.monotonic()
    pump(1.5)
    after = [s for s in window(t1 + .1)]
    blocked = wait_until(lambda: arbiter().get('ready') is False, 3.)
    b = arbiter().get('blocker')
    faults = faults_since(w1)
    DRV.halt()
    rej = expect_http(lambda: OP.post(DRV.path(), {'action': 'start', 'epoch': EPOCH[0]}), 409)
    record('s17_gate_sigkill', mv and not after and blocked and rej[0], gate_output_after_kill=len(after), arbiter_blocker=b,
           watchdog_faults=faults, start_rejected=rej,
           note='with the Gate gone nothing reaches the motor topic; a real MCU holding its last command is NOT covered (STM32 timeout unverified)')


ORDER = [s01_startup_silent_nav2_authorizes, s02_normal_drive_and_bounds, s03_deadman_release_stops, s04_input_stall_times_out,
         s05_stop_button, s06_bad_inputs_rejected, s07_mode_exclusion_mission_and_mapping, s08_auto_mission_blocks_manual,
         s16_browser_keyboard_pointer_gamepad, s09_foreign_nav_goal_stops_then_recovers, s10_foreign_followpath_stops,
         s11_nav2_server_restart, s12_lease_expiry_and_force_release, s13_agent_sigkill_and_restart,
         s14_watchdog_sigkill_and_restart, s15_fleet_server_outage_and_restart, s18_gate_input_rogue_raw_in_manual, s17_gate_sigkill]

try:
    for name in ('gate', 'watchdog', 'nav', 'agent'):
        spawn(name)
    pump(2.)
    dead = [n for n, p in children.items() if p.poll() is not None]
    if dead:
        record('startup_processes', False, exited=dead)
    else:
        only = os.environ.get('MANUAL_E2E_ONLY')
        for fn in ORDER:
            if only and fn.__name__ not in only.split(','): continue
            say('--- ' + fn.__name__)
            fn()
        motor = {'publishers': fx.count_publishers('/cmd_vel'), 'subscribers': fx.count_subscribers('/cmd_vel')}
        record('no_cmd_vel_endpoints', motor == {'publishers': 0, 'subscribers': 0}, **motor)
finally:
    DRV.halt()
    for name in ('agent', 'watchdog', 'nav', 'gate'):
        kill(name, signal.SIGINT)
    for f in records: f.close()
    try: SERVER.stop()
    except Exception: pass
    for n in (fx, foreign): n.destroy_node()
    rclpy.try_shutdown()
    (OUT / 'blocker_timeline.json').write_text(json.dumps(TIMELINE, indent=0))
    (OUT / 'server_stalls.json').write_text(json.dumps(STALLS, indent=1))
    summary = {'domain': DOMAIN, 'gate_source': str(GATE.relative_to(REPO)), 'gate_input_params': GATE_INPUT, 'gate_resets': GATE_RESETS, 'browser': BROWSER, 'results': results,
               'counts': {k: sum(r['result'] == k for r in results) for k in ('PASS', 'FAIL', 'SKIP')},
               'locked_samples': len(locked), 'finished': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    (OUT / 'result.json').write_text(json.dumps(summary, indent=1, default=str))
    say('SUMMARY', summary['counts'])
    log.close()
    sys.exit(0 if summary['counts']['FAIL'] == 0 else 1)
