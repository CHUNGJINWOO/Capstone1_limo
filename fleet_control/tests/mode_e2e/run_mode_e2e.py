"""Phase 2 isolated E2E: operating-mode transitions and web mapping through the real Fleet HTTP API.

Stack (tests/mode_e2e/stack_up.sh): Fleet server --require-operating-mode, 2D kinematic sim (moves only from the
Gate's /cmd_vel_locked), ICP odom + six Nav2 servers (slow LiDAR profile) + r10 candidate Gate (locked output),
Motion Watchdog, Agent (mock, VelocityArbiter + web bench + Mapping Worker + missions; map/localization FIXTURE
wrapper for NAVIGATION). The harness only uses HTTP (like the web UI) plus read-only ROS observation, and
stack_proc.sh for fault injection on processes it started.

Usage: OVERLAY=<explore_lite overlay> MODE_E2E_OUT=<new dir> python3 run_mode_e2e.py <port> <domain>
"""
import json, math, os, subprocess, sys, threading, time, traceback, uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent
FC = HERE.parents[1]
sys.path[:0] = [str(FC / 'agent' / 'limo_robot_agent')]
PORT, DOMAIN = int(sys.argv[1]), sys.argv[2]
assert PORT not in (8080, 18190, 18191, 18192) and DOMAIN not in ('0', '77')
OUT = Path(os.environ['MODE_E2E_OUT']).resolve()
RUN = OUT / 'stack'
OUT.mkdir(parents=True, exist_ok=False)
log = (OUT / 'harness.log').open('x')
results = []
RID = 'iso_bench'


def say(*a):
    line = time.strftime('%H:%M:%S ') + ' '.join(str(x) for x in a)
    print(line, flush=True); log.write(line + '\n'); log.flush()


subprocess.run([str(HERE / 'stack_up.sh'), str(RUN), str(PORT), DOMAIN], check=True,
               env=dict(os.environ, NAV_LOCALIZATION='1'),       # Phase 3 contract: real Localization Worker, no fixture
               stdout=(OUT / 'stack_up.log').open('x'), stderr=subprocess.STDOUT)
for line in (RUN / 'env.sh').read_text().splitlines():
    pass
os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=DOMAIN)
os.environ.pop('ROS_DISCOVERY_SERVER', None)

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
from limo_robot_agent.core import Http, HttpError

URL = f'http://127.0.0.1:{PORT}'
tok = lambda n: (RUN / 'secrets' / f'{n}.token').read_text().strip()
OP, ADMIN = Http(URL, tok('operator')), Http(URL, tok('admin'))
for h in (OP, ADMIN):
    h.timeout = 3.0

rclpy.init()
obs = rclpy.create_node('mode_e2e_observer')
locked, truth, watch = [], [None], [None]
obs.create_subscription(Twist, '/cmd_vel_locked', lambda m: locked.append((time.monotonic(), m.linear.x, m.angular.z)), 50)
obs.create_subscription(Float64MultiArray, '/e2e/ground_truth', lambda m: truth.__setitem__(0, (time.monotonic(), *m.data)), 10)
obs.create_subscription(String, '/fleet/motion_watchdog/status', lambda m: watch.__setitem__(0, json.loads(m.data)), 10)
GATE_TL = []
GATE_LAST = [{}]


def on_gate(m):
    try: d = json.loads(m.data)
    except ValueError: return
    GATE_LAST[0] = d
    key = (d.get('state'), d.get('cmd_blocked'), d.get('fault_reason') or d.get('reason') or d.get('last_fault'))
    if not GATE_TL or GATE_TL[-1][1:] != key:
        GATE_TL.append((round(time.time(), 2),) + key)


obs.create_subscription(String, '/stall_detector/status_json', on_gate, 10)
ex = SingleThreadedExecutor(); ex.add_node(obs)
threading.Thread(target=ex.spin, daemon=True).start()


def get(path):
    import urllib.request
    req = urllib.request.Request(URL + '/api/v1' + path, headers={'Authorization': 'Bearer ' + tok('operator')})
    with urllib.request.urlopen(req, timeout=3) as r:
        return json.loads(r.read())


def robot():
    return get(f'/robots/{RID}')


def mode():
    return get(f'/robots/{RID}/mode')


def state():
    return robot().get('state') or {}


def arbiter():
    return state().get('velocity_arbiter') or {}


def proc(op, name, sig=None):
    """Returns monotonic time when the command returned (the signal was sent at or before it)."""
    subprocess.run([str(HERE / 'stack_proc.sh'), str(RUN), op, name] + ([sig] if sig else []), check=False)
    return time.monotonic()


def wait_until(pred, timeout, step=.2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            v = pred()
            if v: return v
        except Exception:
            pass
        time.sleep(step)
    return None


def expect_http(fn, status, code=None):
    try: fn()
    except HttpError as e:
        return e.status == status and (code is None or code in str(e)), str(e)[:220]
    return False, 'accepted'


EPOCH = [None]
RENEW = threading.Event(); RENEW.set()


def lease(ttl=60):
    EPOCH[0] = OP.post(f'/robots/{RID}/lock', {'action': 'acquire', 'ttl_s': ttl})['epoch']
    return EPOCH[0]


def renewer():
    while True:
        time.sleep(10)
        if RENEW.is_set() and EPOCH[0] is not None:
            try: OP.post(f'/robots/{RID}/lock', {'action': 'renew', 'ttl_s': 60})
            except Exception: pass


threading.Thread(target=renewer, daemon=True).start()


def request_mode(target):
    return OP.post(f'/robots/{RID}/mode', {'target': target, 'epoch': EPOCH[0], 'idempotency_key': 'e2e-' + uuid.uuid4().hex[:12]})


def wait_mode(target, timeout=40):
    """Poll the transition; returns (final record, history of (state, blocker), seconds)."""
    t0 = time.monotonic(); hist = []
    while time.monotonic() - t0 < timeout:
        rec = mode()['operating_mode'] or {}
        t = rec.get('transition') or {}
        key = (rec.get('mode'), t.get('state'), t.get('blocker') or t.get('reason'))
        if not hist or hist[-1] != key: hist.append(key)
        if t.get('to') == target and t.get('state') in ('ACTIVE', 'FAILED'):
            return rec, hist, round(time.monotonic() - t0, 2)
        time.sleep(.2)
    return mode()['operating_mode'], hist, None


def switch(target, timeout=40):
    request_mode(target)
    return wait_mode(target, timeout)


MAP_SID = [None]


def mapping(action, **extra):
    body = {'action': action, 'epoch': EPOCH[0], 'idempotency_key': 'e2e-' + uuid.uuid4().hex[:12], **extra}
    if action != 'START': body['session_id'] = MAP_SID[0]
    r = OP.post(f'/robots/{RID}/mapping', body)
    if action == 'START': MAP_SID[0] = r['session_id']
    return r


def wait_mapping(cmd_id, timeout=40):
    t0 = time.monotonic(); hist = []
    while time.monotonic() - t0 < timeout:
        s = get(f'/robots/{RID}/mapping'); c = s.get('command') or {}
        key = (s.get('state'), c.get('status'))
        if not hist or hist[-1] != key: hist.append(key)
        if c.get('command_id') == cmd_id and c.get('status') in ('COMPLETED', 'FAILED', 'UNKNOWN'):
            return s, hist, round(time.monotonic() - t0, 2)
        time.sleep(.2)
    return get(f'/robots/{RID}/mapping'), hist, None


class Manual:
    def __init__(self): self.session = None; self.seq = 0; self.on = False; self.errors = []

    def start(self):
        self.session = OP.post(f'/robots/{RID}/manual-bench', {'action': 'start', 'epoch': EPOCH[0]})['session_id']; self.seq = 0
        self.on = True; threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        while self.on:
            self.seq += 1
            try: OP.post(f'/robots/{RID}/manual-bench', {'action': 'sample', 'input_format': 'normalized_axes_v1', 'epoch': EPOCH[0], 'session_id': self.session,
                                                          'seq': self.seq, 'linear': 1., 'turn': .3, 'deadman': True})
            except Exception as e: self.errors.append(str(e)[:120]); self.on = False
            time.sleep(.08)

    def stop(self):
        self.on = False; time.sleep(.1)
        try: OP.post(f'/robots/{RID}/manual-bench', {'action': 'stop', 'epoch': EPOCH[0], 'session_id': self.session})
        except Exception: pass


def nz_since(t0):
    return [s for s in locked if s[0] >= t0 and (abs(s[1]) > 1e-6 or abs(s[2]) > 1e-6)]


def still_after(t0, delay=.6, span=.8):
    time.sleep(delay + span)
    return not [s for s in locked if t0 + delay <= s[0] <= t0 + delay + span and (abs(s[1]) > 1e-6 or abs(s[2]) > 1e-6)]


def pose():
    return truth[0][1:4] if truth[0] else None


def moved(p0, p1):
    return None if not p0 or not p1 else round(math.hypot(p1[0] - p0[0], p1[1] - p0[1]), 3)


def record(name, ok, **d):
    results.append({'scenario': name, 'result': 'PASS' if ok else 'FAIL', **d})
    say(('PASS ' if ok else 'FAIL ') + name, json.dumps(d, default=str)[:700])


GATE_EVENTS = []


def _gate_guard_once(label):
    """Environment guard (same as nav_e2e): a latched Gate fault is cleared only via the Gate's own operator unlatch
    service and every use is recorded in result.json; it is never counted as a PASS."""
    g = state().get('safety_gate') or {}
    if g.get('fault_latched') is True or g.get('status') not in ('NORMAL', 'STALL_PENDING'):
        from std_srvs.srv import Trigger
        cli = obs.create_client(Trigger, '/stall_detector/reset_latch')
        ok = cli.wait_for_service(timeout_sec=3); res = None
        if ok:
            f = cli.call_async(Trigger.Request()); wait_until(lambda: f.done(), 5)
            res = f.result().success if f.done() and f.result() else None
        obs.destroy_client(cli); time.sleep(1.5)
        GATE_EVENTS.append({'before': label, 'status': g.get('status'), 'last_fault_reasons': g.get('last_fault_reasons'),
                            'unlatch_result': res, 'status_after': (state().get('safety_gate') or {}).get('status')})


def gate_guard(label, timeout=40):
    """2026-11-20: repeat the recorded operator unlatch while the Gate stays blocked (r11 refuses a reset while a lingering DDS
    endpoint of a killed /cmd_vel_raw writer is still visible, ~15-20 s). Each attempt is a GATE_EVENTS entry."""
    end = time.monotonic() + timeout
    while True:
        _gate_guard_once(label)
        g = state().get('safety_gate') or {}
        if (g.get('fault_latched') is not True and g.get('status') in ('NORMAL', 'STALL_PENDING')) or time.monotonic() > end: return
        time.sleep(1.5)


def scenario(fn):
    def run():
        try: gate_guard(fn.__name__)
        except Exception as e: GATE_EVENTS.append({'before': fn.__name__, 'error': repr(e)[:120]})
        try: fn()
        except Exception as e:
            record(fn.__name__, False, error=repr(e)[:300], trace=traceback.format_exc()[-900:])
    run.__name__ = fn.__name__
    return run


# ================================================================== normal transitions
@scenario
def n01_startup_idle_and_mode_gates():
    ok_up = wait_until(lambda: (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED' and robot().get('online'), 60)
    lease()
    time.sleep(.6)
    rec = mode()['operating_mode']
    g = (state().get('execution') or {}).get('nav2_goals') or {}
    man = expect_http(lambda: OP.post(f'/robots/{RID}/manual-bench', {'action': 'start', 'epoch': EPOCH[0]}), 409, 'mode_not_manual')
    mp = expect_http(lambda: mapping('START'), 409, 'mode_not_mapping')
    mi = expect_http(lambda: OP.post('/missions', {'robot_id': RID, 'type': 'navigate_to_pose', 'goal': {'frame_id': 'map', 'x': 1, 'y': 0},
                                                   'lease_epoch': EPOCH[0], 'idempotency_key': 'e2e-' + uuid.uuid4().hex}), 409, 'mode_not_navigation')
    record('n01_startup_idle_and_mode_gates', bool(ok_up) and rec['mode'] == 'IDLE' and man[0] and mp[0] and mi[0]
           and g.get('status_continuous') is True and g.get('received_status_array') is False,
           mode=rec['mode'], real_nav2_silent_idle={k: g.get(k) for k in ('publishers', 'active_count', 'received_status_array', 'status_continuous')},
           arbiter_blocker=arbiter().get('blocker'), manual=man, mapping_start=mp, mission=mi)


@scenario
def n02_idle_to_manual_and_drive():
    rec, hist, secs = switch('MANUAL')
    p0 = pose(); t0 = time.monotonic(); m = Manual(); m.start(); time.sleep(1.5); m.stop(); t1 = time.monotonic()
    p1 = pose(); stopped = still_after(t1)
    record('n02_idle_to_manual_and_drive', rec['mode'] == 'MANUAL' and bool(nz_since(t0)) and (moved(p0, p1) or 0) > .05 and stopped,
           transition_s=secs, history=hist, sim_moved_m=moved(p0, p1), zero_after_stop=stopped, errors=m.errors[:2])


@scenario
def n03_manual_to_idle_while_driving():
    m = Manual(); m.start(); time.sleep(.8); t0 = time.monotonic()
    request_mode('IDLE'); stopped = still_after(t0, .5, .8); m.on = False
    rec, hist, secs = wait_mode('IDLE')
    late = expect_http(lambda: OP.post(f'/robots/{RID}/manual-bench', {'action': 'start', 'epoch': EPOCH[0]}), 409, 'mode_not_manual')
    record('n03_manual_to_idle_while_driving', rec['mode'] == 'IDLE' and stopped and late[0],
           zero_within_0_5s=stopped, transition_s=secs, history=hist, manual_after=late)


@scenario
def n04_idle_to_mapping_start_and_explore():
    rec, hist, secs = switch('MAPPING')
    man = expect_http(lambda: OP.post(f'/robots/{RID}/manual-bench', {'action': 'start', 'epoch': EPOCH[0]}), 409, 'mode_not_manual')
    r = mapping('START', mode='NEW_MAP', limits={'max_duration_s': 600, 'max_goals': 40})
    s, mhist, msecs = wait_mapping(r['command_id'], 45)
    p0 = pose(); t0 = time.monotonic()
    drove = wait_until(lambda: arbiter().get('source') == 'NAV2' and (moved(p0, pose()) or 0) > .25, 60)
    mi = expect_http(lambda: OP.post('/missions', {'robot_id': RID, 'type': 'navigate_to_pose', 'goal': {'frame_id': 'map', 'x': 1, 'y': 0},
                                                   'lease_epoch': EPOCH[0], 'idempotency_key': 'e2e-' + uuid.uuid4().hex}), 409)
    worker = state().get('mapping_worker') or {}
    record('n04_idle_to_mapping_start_and_explore', rec['mode'] == 'MAPPING' and s['state'] == 'MAPPING' and bool(drove) and man[0] and mi[0],
           transition_s=secs, mapping_history=mhist, start_s=msecs, frontier_goal=worker.get('goal'),
           sim_moved_m=moved(p0, pose()), arbiter_source=arbiter().get('source'), manual_during_mapping=man, mission_during_mapping=mi,
           nonzero_gate_samples=len(nz_since(t0)))


SAVED = {}


@scenario
def n05_pause_save_resume():
    r = mapping('PAUSE'); t0 = time.monotonic(); s, h1, _ = wait_mapping(r['command_id'])
    stopped = still_after(t0, 1.5, 1.0) if s['state'] == 'PAUSED' else False
    r = mapping('SAVE', floor_id='paichai_c_1f', area_id='paichai_c_1f_c103_lounge')
    busy = expect_http(lambda: request_mode('MANUAL'), 409, 'mapping_busy')
    s2, h2, save_s = wait_mapping(r['command_id'], 60)
    SAVED.update(files=(s2.get('last_save') or {}).get('files'), registered=s2.get('registered'))
    r = mapping('RESUME'); s3, h3, _ = wait_mapping(r['command_id'])
    p0 = pose()
    again = wait_until(lambda: arbiter().get('source') == 'NAV2' and (moved(p0, pose()) or 0) > .1, 60)
    record('n05_pause_save_resume', s['state'] == 'PAUSED' and (s.get('result') or {}).get('evidence', {}).get('goal_terminal') is True
           and stopped and busy[0] and s2['command']['status'] == 'COMPLETED' and bool(s2.get('registered')) and s3['state'] == 'MAPPING' and bool(again),
           pause=h1, pause_evidence=(s.get('result') or {}).get('evidence'), zero_while_paused=stopped, mode_request_during_save=busy,
           save=h2, save_s=save_s, save_evidence=(s2.get('result') or {}).get('evidence'), registered=s2.get('registered'),
           resume=h3, moved_after_resume=bool(again))


@scenario
def n06_mapping_to_manual_direct():
    t0 = time.monotonic(); rec, hist, secs = switch('MANUAL', 60)
    s = get(f'/robots/{RID}/mapping')
    stopped = not [x for x in locked if x[0] > t0 + 2.5 and (abs(x[1]) > 1e-6 or abs(x[2]) > 1e-6) and x[0] < time.monotonic() - 0.0
                   and not (rec['mode'] == 'MANUAL')]
    p0 = pose(); m = Manual(); m.start(); time.sleep(1.2); m.stop(); p1 = pose()
    record('n06_mapping_to_manual_direct', rec['mode'] == 'MANUAL' and s['state'] == 'STOPPED'
           and (s.get('result') or {}).get('evidence', {}).get('goal_terminal') is True and (moved(p0, p1) or 0) > .05,
           transition_s=secs, history=hist, mapping_state=s['state'], stop_evidence=(s.get('result') or {}).get('evidence'),
           worker=(state().get('mapping_worker') or {}).get('status'), manual_moved_m=moved(p0, p1))


@scenario
def n07_manual_to_mapping_then_stop_then_idle():
    rec, hist, secs = switch('MAPPING')
    r = mapping('START', mode='NEW_MAP'); s, h1, _ = wait_mapping(r['command_id'], 45)
    time.sleep(4)
    r = mapping('STOP'); t0 = time.monotonic(); s2, h2, _ = wait_mapping(r['command_id'])
    stopped = still_after(t0, 1.0, 1.0)
    rec2, hist2, secs2 = switch('IDLE')
    nodes = (state().get('mapping_worker') or {})
    record('n07_manual_to_mapping_then_stop_then_idle', rec['mode'] == 'MAPPING' and s['state'] == 'MAPPING' and s2['state'] == 'STOPPED'
           and stopped and rec2['mode'] == 'IDLE' and nodes.get('processes_started') is False,
           to_mapping_s=secs, start=h1, stop=h2, stop_evidence=(s2.get('result') or {}).get('evidence'), zero_after_stop=stopped,
           to_idle_s=secs2, worker_processes=nodes.get('processes_started'))


# ================================================================== faults (manual / mapping)
@scenario
def f01_lease_loss_in_manual():
    switch('MANUAL'); m = Manual(); m.start(); time.sleep(.8); t0 = time.monotonic()
    ADMIN.post(f'/robots/{RID}/lock', {'action': 'force_release'})
    stopped = still_after(t0, .5, .8); m.on = False
    rec = wait_until(lambda: mode()['operating_mode'] if mode()['operating_mode']['mode'] == 'IDLE' else None, 5) or mode()['operating_mode']
    lease()
    after = expect_http(lambda: OP.post(f'/robots/{RID}/manual-bench', {'action': 'start', 'epoch': EPOCH[0]}), 409, 'mode_not_manual')
    record('f01_lease_loss_in_manual', stopped and rec['mode'] == 'IDLE' and after[0], zero_within_0_5s=stopped,
           mode=rec['mode'], reason=rec.get('reason'), manual_with_new_lease=after)


@scenario
def f02_server_restart_in_manual():
    switch('MANUAL'); m = Manual(); m.start(); time.sleep(.8); t0 = time.monotonic()
    t0 = proc('kill', 'server'); stopped = still_after(t0, .5, .8); m.on = False
    last_nz = max((x[0] - t0 for x in locked if x[0] >= t0 and (abs(x[1]) > 1e-6 or abs(x[2]) > 1e-6)), default=None)
    proc('start', 'server')
    up = wait_until(lambda: robot().get('online') and robot().get('reconciled'), 30)
    rec = mode()['operating_mode']
    lease()
    after = expect_http(lambda: OP.post(f'/robots/{RID}/manual-bench', {'action': 'start', 'epoch': EPOCH[0]}), 409, 'mode_not_manual')
    rec2, hist, secs = switch('MANUAL'); p0 = pose(); m = Manual(); m.start(); time.sleep(1.2); m.stop()
    record('f02_server_restart_in_manual', stopped and bool(up) and rec['mode'] == 'IDLE' and after[0] and rec2['mode'] == 'MANUAL'
           and (moved(p0, pose()) or 0) > .05, zero_within_0_5s=stopped, mode_after_restart=rec['mode'], reason=rec.get('reason'),
           manual_before_reselect=after, reselect_s=secs, last_nonzero_after_kill_s=last_nz)


def reconcile_mapping(attempts=8):
    """Operator RECONCILE, retried while the Worker's evidence is incomplete (e.g. DDS ghosts of killed nodes)."""
    tries = []
    for _ in range(attempts):
        s = get(f'/robots/{RID}/mapping')
        MAP_SID[0] = s.get('session_id')
        r = mapping('RECONCILE')
        s, hist, secs = wait_mapping(r['command_id'], 30)
        tries.append({'state': s['state'], 'evidence': (s.get('result') or {}).get('evidence')})
        if s['state'] == 'STOPPED':
            return s, tries, secs
        time.sleep(6)
    return s, tries, None


@scenario
def f03_lease_expiry_in_mapping_then_reconcile():
    switch('MAPPING'); r = mapping('START', mode='NEW_MAP'); wait_mapping(r['command_id'], 45)
    wait_until(lambda: arbiter().get('source') == 'NAV2', 40)
    RENEW.clear(); OP.post(f'/robots/{RID}/lock', {'action': 'renew', 'ttl_s': 5})
    t0 = time.monotonic()
    unknown = wait_until(lambda: get(f'/robots/{RID}/mapping')['state'] == 'UNKNOWN', 25)
    stopped = not [x for x in locked if t0 + 7 <= x[0] and (abs(x[1]) > 1e-6 or abs(x[2]) > 1e-6)]
    rec = mode()['operating_mode']
    lease(); RENEW.set()
    blocked = switch('MANUAL')
    rb = robot()
    diag = {k: rb.get(k) for k in ('online', 'reconciled', 'reconciliation_required', 'state_age_s')}
    s, hist, secs = reconcile_mapping()
    diag['events'] = [(e['kind'], e['message'][:90]) for e in get('/events?limit=14')['events']]
    diag['worker'] = (state().get('mapping_worker') or {}).get('status')
    rec2, hist2, _ = switch('IDLE')
    record('f03_lease_expiry_in_mapping_then_reconcile', bool(unknown) and stopped and rec['mode'] == 'FAULT'
           and blocked[0]['transition']['state'] == 'FAILED' and s['state'] == 'STOPPED' and rec2['mode'] == 'IDLE',
           mode_after_expiry=rec['mode'], reason=rec.get('reason'), no_motion_after_expiry=stopped,
           switch_while_unknown=blocked[0]['transition'].get('reason'), reconcile=hist, reconcile_evidence=(s.get('result') or {}).get('evidence'),
           then_idle=rec2['mode'], diag=diag)


@scenario
def f04_agent_restart_in_mapping_then_reconcile():
    switch('MAPPING'); r = mapping('START', mode='NEW_MAP'); wait_mapping(r['command_id'], 45)
    wait_until(lambda: arbiter().get('source') == 'NAV2', 40)
    old_boot = (robot().get('agent') or {}).get('boot_id')
    t0 = proc('kill', 'agent')
    stopped = still_after(t0, .6, 1.0)
    proc('start', 'agent')
    up = wait_until(lambda: (robot().get('agent') or {}).get('boot_id') != old_boot and robot().get('online') and robot().get('reconciled')
                    and get(f'/robots/{RID}/mapping')['state'] == 'UNKNOWN', 40)
    s0 = get(f'/robots/{RID}/mapping'); rec = mode()['operating_mode']
    s, hist, secs = reconcile_mapping()
    rec2, hist2, secs2 = switch('MANUAL')
    if rec2['mode'] != 'MANUAL':
        record('f04_agent_restart_in_mapping_then_reconcile', False, transition=rec2.get('transition'), history=hist2,
               mapping=s['state'], reconcile=hist, watchdog=(watch[0] or {}).get('state'), arbiter=arbiter())
        return
    p0 = pose(); m = Manual(); m.start(); time.sleep(1.2); m.stop()
    record('f04_agent_restart_in_mapping_then_reconcile', stopped and bool(up) and s0['state'] == 'UNKNOWN' and rec['mode'] == 'FAULT'
           and s['state'] == 'STOPPED' and rec2['mode'] == 'MANUAL' and (moved(p0, pose()) or 0) > .05,
           zero_within_0_6s=stopped, mapping_after_restart=s0['state'], mode_after_restart=rec['mode'], reason=rec.get('reason'),
           reconcile=hist, reconcile_evidence=(s.get('result') or {}).get('evidence'), to_manual=hist2)


# ================================================================== NAVIGATION (map_server + static map->odom FIXTURE)
FIXTURE_PROCS = []


def start_nav_fixture():
    """Phase 3 contract (2026-11-20): NAVIGATION needs a registered VERIFIED map loaded through the Registry, an operator
    initial pose and real AMCL CONFIRMED. The map saved by n05 is promoted in THIS run's isolated registry only (the
    verify route is not exposed over HTTP), then LOAD + INITIAL_POSE go through the web API. No map/loc fixture."""
    mvid = (SAVED.get('registered') or {}).get('map_version_id')
    code = ("import sys; sys.path.insert(0,sys.argv[1]); from fleet_server.map_registry import MapRegistry; "
            "r=MapRegistry(sys.argv[2]+'/map_registry.db', sys.argv[2]+'/map_store'); "
            "r.verify_map_version(sys.argv[3], status='VERIFIED', evidence={'method':'isolated_sim_e2e'}, actor='e2e-qa', actor_role='admin', reason='isolated sim map')")
    out = subprocess.run([sys.executable, '-c', code, str(FC / 'server'), str(RUN / 'fleet'), mvid], capture_output=True, text=True)
    NAVPREP.update(map_version_id=mvid, verify_rc=out.returncode, verify_err=out.stderr[-200:])
    return mvid


NAVPREP = {}


def localize():
    """LOAD the verified map and set the initial pose from ground truth; returns the localization report."""
    OP.post(f'/robots/{RID}/navigation', {'action': 'LOAD', 'epoch': EPOCH[0], 'map_version_id': NAVPREP['map_version_id']})
    wait_until(lambda: (state().get('localization') or {}).get('status') in ('WAIT_INITIAL_POSE', 'FAILED'), 40)
    p = pose()
    OP.post(f'/robots/{RID}/navigation', {'action': 'INITIAL_POSE', 'epoch': EPOCH[0], 'x': p[0], 'y': p[1], 'yaw': p[2]})
    wait_until(lambda: (state().get('localization') or {}).get('status') == 'CONFIRMED', 40)
    return state().get('localization') or {}


def valid_goal(far):
    """A goal the server accepts on the loaded map (Phase 3 goal validation), ranked by distance from the robot."""
    p = pose()
    cands = [(round(-2.6 + .4 * i, 2), round(-2.2 + .4 * j, 2)) for i in range(14) for j in range(12)]
    cands = [c for c in cands if math.hypot(c[0] - p[0], c[1] - p[1]) >= 1.0]
    cands.sort(key=lambda c: -math.hypot(c[0] - p[0], c[1] - p[1]) if far else math.hypot(c[0] - p[0], c[1] - p[1]))
    for c in cands:
        if get(f'/robots/{RID}/goal-check?x={c[0]}&y={c[1]}').get('ok'):
            return c
    raise AssertionError('no valid goal on the loaded map')


def mission(x, y):
    return OP.post('/missions', {'robot_id': RID, 'type': 'navigate_to_pose', 'goal': {'frame_id': 'map', 'x': x, 'y': y},
                                 'lease_epoch': EPOCH[0], 'idempotency_key': 'e2e-' + uuid.uuid4().hex})


def mission_status(mid):
    return get(f'/missions/{mid}')['status'] if False else next((m['status'] for m in get(f'/missions?robot_id={RID}&limit=20')['missions'] if m['mission_id'] == mid), None)


@scenario
def n08_idle_to_navigation_goal_succeeds():
    yaml = start_nav_fixture()
    rec, hist, secs = switch('NAVIGATION')
    l = localize(); NAVPREP['localization'] = {k: l.get(k) for k in ('status', 'match_ratio')}
    g = valid_goal(False); m = mission(*g); mid = m['mission_id']
    done = wait_until(lambda: mission_status(mid) in ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED', 'RECONCILIATION_REQUIRED')
                      and mission_status(mid), 90, .5)
    p = pose()
    record('n08_idle_to_navigation_goal_succeeds', rec['mode'] == 'NAVIGATION' and done == 'SUCCEEDED' and p and math.hypot(p[0] - g[0], p[1] - g[1]) < .4,
           map_version=str(yaml), localization=NAVPREP.get('localization'), transition_s=secs, history=hist, mission=done, final_pose=p,
           note='Phase 3 contract: registered map verified in the isolated registry, LOAD + INITIAL_POSE + real AMCL')


@scenario
def n09_navigation_to_manual_mid_goal():
    g = valid_goal(True); m = mission(*g); mid = m['mission_id']
    wait_until(lambda: arbiter().get('source') == 'NAV2' and mission_status(mid) == 'EXECUTING', 30)
    time.sleep(1.5)
    request_mode('MANUAL'); t0 = time.monotonic()
    second = expect_http(lambda: request_mode('IDLE'), 409, 'transition_in_progress')
    rec, hist, secs = wait_mode('MANUAL', 40)
    status = mission_status(mid)
    zero_from = None
    for x in locked:
        if x[0] > t0 and abs(x[1]) < 1e-6 and abs(x[2]) < 1e-6:
            zero_from = round(x[0] - t0, 3); break
    p0 = pose(); time.sleep(1.0); idle_drift = moved(p0, pose())
    mm = Manual(); p1 = pose(); mm.start(); time.sleep(1.2); mm.stop()
    record('n09_navigation_to_manual_mid_goal', rec['mode'] == 'MANUAL' and status == 'CANCELED' and second[0]
           and (idle_drift or 0) < .01 and (moved(p1, pose()) or 0) > .05,
           mission=status, first_zero_s=zero_from, transition_s=secs, history=hist, concurrent_request=second,
           drift_after_transition_m=idle_drift, manual_moved_m=moved(p1, pose()))


@scenario
def n10_manual_to_navigation():
    rec, hist, secs = switch('NAVIGATION'); localize()          # leaving NAVIGATION cleared the map request
    g = valid_goal(False); m = mission(*g); mid = m['mission_id']
    done = wait_until(lambda: mission_status(mid) in ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED', 'RECONCILIATION_REQUIRED')
                      and mission_status(mid), 90, .5)
    record('n10_manual_to_navigation', rec['mode'] == 'NAVIGATION' and done == 'SUCCEEDED', transition_s=secs, mission=done, final_pose=pose())


RECOVERED = [True]


@scenario
def f05_watchdog_kill_during_navigation():
    g = valid_goal(True); m = mission(*g); mid = m['mission_id']
    wait_until(lambda: arbiter().get('source') == 'NAV2', 30); time.sleep(1)
    t0 = proc('kill', 'watchdog')
    stopped = still_after(t0, .5, 1.0)
    status = wait_until(lambda: mission_status(mid) if mission_status(mid) not in ('EXECUTING', 'DISPATCHED', 'ACCEPTED') else None, 20)
    proc('start', 'watchdog')
    # 2026-11-20: with Gate r11 the restarted watchdog is a new /cmd_vel_raw writer; the Gate stays FAILSAFE until the operator
    # reset, and the server's locked-mock recovery requires gate_normal. Recorded operator reset (no-op when the Gate is NORMAL).
    # the killed watchdog's DDS ghost keeps the bound GID visible; the Gate only sees the new writer (NOT_EXCLUSIVE) once it appears
    wait_until(lambda: (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED', 30); time.sleep(2)
    gate_guard('f05_before_admin_recovery')
    rec, hist, secs = switch('IDLE', 60)
    recovery = []
    for path in (f'/missions/{mid}/recover', f'/robots/{RID}/recover'):    # existing admin recovery (evidence-checked)
        try: ADMIN.post(path, {'note': 'e2e: watchdog restarted, Nav2 idle'}); recovery.append((path, 'OK'))
        except HttpError as e: recovery.append((path, str(e)[:160]))
    time.sleep(2)
    RECOVERED[0] = not robot().get('reconciliation_required')
    record('f05_watchdog_kill_during_navigation', stopped and rec['mode'] in ('IDLE', 'FAULT') and rec['mode'] != 'NAVIGATION'
           and status in ('CANCELED', 'FAILED', 'RECONCILIATION_REQUIRED'),
           zero_within_0_5s=stopped, mission=status, to_idle=hist, to_idle_s=secs, admin_recovery=recovery, recovered=RECOVERED[0],
           note='watchdog trip invalidates the mission (no automatic re-departure); IDLE verification refuses while reconciliation is required')


RECOVERED = [True]


# ------------------------------------------------------------------ 2026-11-20 Gate input-source probes (asserted with every Gate)
def rogue_raw_probe(secs=2.5, vx=.123):
    """A foreign /cmd_vel_raw writer (this harness's observer node) while the mode's legitimate path is active."""
    pub = obs.create_publisher(Twist, '/cmd_vel_raw', 10); m = Twist(); m.linear.x = vx
    t0 = time.monotonic(); end = t0 + secs; g = {}
    while time.monotonic() < end:
        pub.publish(m); time.sleep(.05)
        if time.monotonic() > t0 + 1: g = g or dict(GATE_LAST[0])
    obs.destroy_publisher(pub); t1 = time.monotonic()
    rogue_vals = [s for s in locked if s[0] >= t0 and abs(s[1] - vx) < 1e-4]
    late = [s for s in locked if t0 + .3 <= s[0] <= t1 + .5 and (abs(s[1]) > 1e-6 or abs(s[2]) > 1e-6)]
    time.sleep(1.0)
    return {'rogue_value_samples': len(rogue_vals), 'nonzero_after_0_3s': len(late), 'gate_state_during': g.get('state'),
            'gate_cmd_input_during': (g.get('cmd_input') or {}).get('state'), 'gate_faults_during': (g.get('fault_reasons') or [])[:3],
            'blocked': not rogue_vals and not late}


def gate_rebound():
    gate_guard('after_rogue_probe')
    return wait_until(lambda: GATE_LAST[0].get('state') in ('NORMAL', 'STALL_PENDING') and (GATE_LAST[0].get('cmd_input') or {}).get('state', 'BOUND') == 'BOUND', 15)


@scenario
def g01_gate_input_rogue_in_manual():
    switch('MANUAL'); m = Manual(); m.start(); time.sleep(1.0)
    pr = rogue_raw_probe(); m.stop()
    rebound = gate_rebound(); p0 = pose(); t0 = time.monotonic(); m2 = Manual(); m2.start(); time.sleep(1.5); m2.stop()
    record('g01_gate_input_rogue_in_manual', pr['blocked'] and bool(rebound) and bool(nz_since(t0)), probe=pr, gate_rebound=rebound,
           legit_drive_after_reset_m=moved(p0, pose()))


@scenario
def g02_gate_input_rogue_in_mapping():
    gate_guard('g02_before'); switch('IDLE', 60)
    switch('MAPPING'); r = mapping('START', mode='NEW_MAP', limits={'max_duration_s': 600, 'max_goals': 40}); wait_mapping(r['command_id'], 45)
    p0 = pose(); drove = wait_until(lambda: arbiter().get('source') == 'NAV2' and (moved(p0, pose()) or 0) > .15, 60)
    pr = rogue_raw_probe(); sess = get(f'/robots/{RID}/mapping').get('state')
    rebound = gate_rebound()
    try:
        s = mapping('STOP'); wait_mapping(s['command_id'], 30)
    except Exception as e: s = {'error': str(e)[:120]}
    rec, hist, secs = switch('IDLE', 60)
    record('g02_gate_input_rogue_in_mapping', bool(drove) and pr['blocked'] and bool(rebound), probe=pr, mapping_state_after_probe=sess,
           gate_rebound=rebound, to_idle=rec['mode'])


@scenario
def g03_gate_input_rogue_in_navigation():
    # runs after f06, which ends in FAULT with a RECONCILIATION_REQUIRED mission: same evidence-checked admin recovery as f05
    pre = []
    gate_guard('g03_before_recovery')
    if robot().get('reconciliation_required') or mode().get('mode') == 'FAULT':
        time.sleep(3)
        for m in get(f'/missions?robot_id={RID}&limit=20')['missions']:
            if m['status'] == 'RECONCILIATION_REQUIRED':
                try: ADMIN.post(f"/missions/{m['mission_id']}/recover", {'note': 'e2e: after f06, Nav2 idle'}); pre.append((m['mission_id'], 'OK'))
                except HttpError as e: pre.append((m['mission_id'], str(e)[:120]))
        try: ADMIN.post(f'/robots/{RID}/recover', {'note': 'e2e: after f06'}); pre.append(('robot', 'OK'))
        except HttpError as e: pre.append(('robot', str(e)[:120]))
        time.sleep(2); switch('IDLE', 60)
    switch('NAVIGATION'); localize()
    g = valid_goal(True); mid = mission(*g)['mission_id']
    drove = wait_until(lambda: arbiter().get('source') == 'NAV2', 30); time.sleep(1)
    pr = rogue_raw_probe()
    status = wait_until(lambda: mission_status(mid) if mission_status(mid) not in ('EXECUTING', 'DISPATCHED', 'ACCEPTED') else None, 20)
    rebound = gate_rebound(); rec, hist, secs = switch('IDLE', 60); recovery = []
    if robot().get('reconciliation_required'):
        for path in (f'/missions/{mid}/recover', f'/robots/{RID}/recover'):
            try: ADMIN.post(path, {'note': 'e2e: gate input probe, Nav2 idle'}); recovery.append((path, 'OK'))
            except HttpError as e: recovery.append((path, str(e)[:160]))
        time.sleep(2)
    switch('NAVIGATION'); localize()
    record('g03_gate_input_rogue_in_navigation', bool(drove) and pr['blocked'] and bool(rebound), probe=pr, mission=status,
           gate_rebound=rebound, admin_recovery=recovery, reconciliation_after=robot().get('reconciliation_required'), recovery_before=pre)


@scenario
def f06_cancel_unanswered_bt_navigator_frozen():
    if not RECOVERED[0]:
        results.append({'scenario': 'f06_cancel_unanswered_bt_navigator_frozen', 'result': 'SKIP',
                        'reason': 'robot still RECONCILIATION_REQUIRED after f05 (admin recovery refused)'}); return
    switch('NAVIGATION'); localize()
    g = valid_goal(True); m = mission(*g); mid = m['mission_id']
    wait_until(lambda: arbiter().get('source') == 'NAV2', 30); time.sleep(1)
    # 2026-11-20 (Codex review): only the bt_navigator of THIS harness's stack process group (never a machine-wide
    # pgrep match, which could be another session's Nav2). Original preserved in local_data/.../codex_fix_claude_20261120/before_fix.
    sys.path.insert(0, str(HERE))
    from owned_proc import owned_pid
    pid = [owned_pid(RUN, r'__node:=bt_navigator|nav2_bt_navigator/bt_navigator', groups=('stack',))]
    os.kill(int(pid[0]), 19)          # SIGSTOP: cancel and result are never answered
    try:
        t0 = time.monotonic(); request_mode('MANUAL')
        stopped = still_after(t0, .5, 1.0)
        rec, hist, secs = wait_mode('MANUAL', 45)
        status = mission_status(mid)
    finally:
        os.kill(int(pid[0]), 18)      # SIGCONT the same owned process even if the scenario raises
    record('f06_cancel_unanswered_bt_navigator_frozen', stopped and rec['mode'] in ('FAULT', 'IDLE') and (rec.get('transition') or {}).get('state') == 'FAILED',
           zero_within_0_5s=stopped, mode=rec['mode'], reason=(rec.get('transition') or {}).get('reason'), mission=status, history=hist)


ORDER = [n01_startup_idle_and_mode_gates, n02_idle_to_manual_and_drive, g01_gate_input_rogue_in_manual, n03_manual_to_idle_while_driving,
         n04_idle_to_mapping_start_and_explore, n05_pause_save_resume, n06_mapping_to_manual_direct,
         n07_manual_to_mapping_then_stop_then_idle, f01_lease_loss_in_manual, f02_server_restart_in_manual,
         f03_lease_expiry_in_mapping_then_reconcile, f04_agent_restart_in_mapping_then_reconcile,
         n08_idle_to_navigation_goal_succeeds, n09_navigation_to_manual_mid_goal, n10_manual_to_navigation,
         f05_watchdog_kill_during_navigation, f06_cancel_unanswered_bt_navigator_frozen,
         g03_gate_input_rogue_in_navigation, g02_gate_input_rogue_in_mapping]   # g02 last: its exploration moves the robot off the n05 map used by n08   # 2026-11-20: after f06 (placed before f05 it left f05's admin recovery idle_not_proven -> f06 SKIP, both Gates)

NAV2_STARTS = []


def ensure_nav2():
    """2026-11-20 (same guard as nav_e2e): restart the stack group and the test Agent when Nav2 idle evidence and the watchdog do not
    come up (a new Gate boot latches the running Agent). A lifecycle 'all active' check is NOT valid here: in Phase 2 the planner /
    behavior servers only finish activating once SLAM provides the map frame (checked 2026-11-21). Recorded."""
    for attempt in range(3):
        ok = wait_until(lambda: ((state().get('execution') or {}).get('nav_idle') or {}).get('ok') is True
                        and (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED', 45)
        NAV2_STARTS.append({'attempt': attempt, 'nav2_up': bool(ok)})
        if ok: return True
        proc('kill', 'stack'); proc('kill', 'agent'); time.sleep(3); proc('start', 'stack'); time.sleep(6); proc('start', 'agent')
        NAV2_STARTS[-1]['restarted'] = ['stack', 'agent']
    return False


try:
    NAV2_STARTS.append({'ensure_nav2': ensure_nav2()})
    only = os.environ.get('MODE_E2E_ONLY')
    for fn in ORDER:
        if only and fn.__name__ not in only.split(','): continue
        say('--- ' + fn.__name__); fn()
    motor = {'publishers': obs.count_publishers('/cmd_vel'), 'subscribers': obs.count_subscribers('/cmd_vel')}
    record('no_cmd_vel_endpoints', motor == {'publishers': 0, 'subscribers': 0}, **motor)
finally:
    for p in FIXTURE_PROCS:
        try: os.killpg(p.pid, 2)
        except Exception: pass
    proc('stopall', 'x')
    summary = {'port': PORT, 'domain': DOMAIN, 'nav2_bringup_attempts': NAV2_STARTS, 'gate_unlatch_events': GATE_EVENTS, 'results': results,
               'counts': {k: sum(r['result'] == k for r in results) for k in ('PASS', 'FAIL', 'SKIP')},
               'locked_samples': len(locked), 'finished': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    (OUT / 'result.json').write_text(json.dumps(summary, indent=1, default=str))
    (OUT / 'gate_timeline.json').write_text(json.dumps(GATE_TL, indent=0, default=str))
    say('SUMMARY', summary['counts'])
    log.flush()
    os._exit(0 if summary['counts']['FAIL'] == 0 else 1)
