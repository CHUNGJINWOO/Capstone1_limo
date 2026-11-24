"""DRY-RUN Fleet Mission E2E on the REAL-AGENT SHAPE (no Arbiter; Motion Watchdog mode gate + speed clamp; Agent real_nav_dry_run), isolated ROS (G2, 2026-11-26).
Not a real-robot result; real_nav_dry_run never permits real motor commands. Derived from run_nav_e2e.py (Phase 3) with the missions sent to POST /dry-run-missions and the d01-d06 scenarios added.

Phase 3 isolated E2E: saved-map web navigation with real map_server + AMCL + Nav2 (no map/localization fixture).

Stack: tests/mode_e2e/stack_up.sh with NAV_LOCALIZATION=1 (unmodified agent_node + LocalizationWorker), kinematic sim.
Flow (HTTP only, like the web UI): MAPPING START -> explore -> PAUSE -> SAVE (Registry, UNVERIFIED) -> STOP -> IDLE ->
explicit isolated-registry verification of THIS sim map (C103/H107 untouched) -> NAVIGATION -> LOAD -> INITIAL_POSE ->
CONFIRMED -> goal-check -> mission -> Nav2 result, plus the failure scenarios.
Usage: OVERLAY=<explore_lite overlay> NAV_E2E_OUT=<new dir> python3 run_nav_e2e.py <port> <domain>
"""
import hashlib, json, math, os, shutil, subprocess, sys, threading, time, traceback, uuid, urllib.parse, urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
FC = HERE.parents[1]
MODE = FC / 'tests' / 'mode_e2e'
sys.path.insert(0, str(MODE))
from owned_proc import owned_pid      # never a machine-wide pgrep
sys.path[:0] = [str(FC / 'agent' / 'limo_robot_agent')]
PORT, DOMAIN = int(sys.argv[1]), sys.argv[2]
assert PORT not in (8080, 18190, 18191, 18192) and DOMAIN not in ('0', '77')
OUT = Path(os.environ['NAV_E2E_OUT']).resolve(); OUT.mkdir(parents=True, exist_ok=False)
RUN = OUT / 'stack'; RID = 'iso_bench'
log = (OUT / 'harness.log').open('x'); results = []


def say(*a):
    line = time.strftime('%H:%M:%S ') + ' '.join(str(x) for x in a)
    print(line, flush=True); log.write(line + '\n'); log.flush()


subprocess.run([str(MODE / 'stack_up.sh'), str(RUN), str(PORT), DOMAIN], check=True, env=dict(os.environ, NAV_LOCALIZATION='1', REAL_SHAPE='1', DRY_RUN='1'),
               stdout=(OUT / 'stack_up.log').open('x'), stderr=subprocess.STDOUT)
os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=DOMAIN); os.environ.pop('ROS_DISCOVERY_SERVER', None)
# 2026-11-21: test-only timeline recorder (owned child; stopped through its own handle in the finally block)
TIMELINE_PROC = subprocess.Popen([sys.executable, str(Path(__file__).resolve().parent / 'timeline_recorder.py'), f'http://127.0.0.1:{PORT}',
                                  str(RUN / 'secrets' / 'operator.token'), str(OUT / 'timeline.jsonl')],
                                 stdout=(OUT / 'timeline_recorder.log').open('x'), stderr=subprocess.STDOUT, start_new_session=True)
SCEN_MARKS = []

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
from nav2_msgs.action import NavigateToPose
from limo_robot_agent.core import Http, HttpError

URL = f'http://127.0.0.1:{PORT}'
tok = lambda n: (RUN / 'secrets' / f'{n}.token').read_text().strip()
OP, ADMIN = Http(URL, tok('operator')), Http(URL, tok('admin'))
for h in (OP, ADMIN): h.timeout = 30.0
rclpy.init(); obs = rclpy.create_node('nav_e2e_observer'); rogue = rclpy.create_node('nav_e2e_rogue')
locked, truth, watch = [], [None], [None]
obs.create_subscription(Twist, '/cmd_vel_locked', lambda m: locked.append((time.monotonic(), m.linear.x, m.angular.z)), 50)
obs.create_subscription(Float64MultiArray, '/e2e/ground_truth', lambda m: truth.__setitem__(0, (time.monotonic(), *m.data)), 10)
obs.create_subscription(String, '/fleet/motion_watchdog/status', lambda m: watch.__setitem__(0, json.loads(m.data)), 10)
ex = SingleThreadedExecutor(); ex.add_node(obs); ex.add_node(rogue); threading.Thread(target=ex.spin, daemon=True).start()


def get(path, token='operator'):
    path = urllib.parse.quote(path, safe='/?=&')          # map version ids contain '#'
    req = urllib.request.Request(URL + '/api/v1' + path, headers={'Authorization': 'Bearer ' + tok(token)})
    with urllib.request.urlopen(req, timeout=10) as r: return json.loads(r.read())


robot = lambda: get(f'/robots/{RID}')
state = lambda: robot().get('state') or {}
loc = lambda: state().get('localization') or {}
nav = lambda: get(f'/robots/{RID}/navigation')
pose = lambda: truth[0][1:4] if truth[0] else None


def proc(op, name):
    subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), op, name], check=False); return time.monotonic()


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
    EPOCH[0] = OP.post(f'/robots/{RID}/lock', {'action': 'acquire', 'ttl_s': 120})['epoch']


def renewer():
    while True:
        time.sleep(15)
        try: OP.post(f'/robots/{RID}/lock', {'action': 'renew', 'ttl_s': 120})
        except Exception: pass


threading.Thread(target=renewer, daemon=True).start()


def switch(target, timeout=60):
    rec = _switch(target, timeout)
    if target != 'NAVIGATION': return rec
    # 2026-11-26: on entry to NAVIGATION the localization worker brings Nav2 up and the Watchdog may trip once (a new action server). The server then ends the mode on purpose
    # (a trip is never carried over), so the operator-style harness re-requests NAVIGATION until it stays confirmed for a few seconds.
    for _ in range(4):
        time.sleep(4.0)
        cur = get(f'/robots/{RID}/mode')['operating_mode'] or {}
        if cur.get('mode') == 'NAVIGATION' and (cur.get('transition') or {}).get('state') == 'ACTIVE': return cur
        say('NAVIGATION ended by the server after entry (' + str(cur.get('reason'))[:120] + '); requesting it again')
        rec = _switch(target, timeout)
    return rec


def _switch(target, timeout=60):
    OP.post(f'/robots/{RID}/mode', {'target': target, 'epoch': EPOCH[0], 'idempotency_key': 'n-' + uuid.uuid4().hex[:10]})
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        rec = get(f'/robots/{RID}/mode')['operating_mode'] or {}; t = rec.get('transition') or {}
        if t.get('to') == target and t.get('state') in ('ACTIVE', 'FAILED'):
            return rec
        time.sleep(.25)
    return get(f'/robots/{RID}/mode')['operating_mode']


SID = [None]


def mapping(action, timeout=60, **extra):
    body = {'action': action, 'epoch': EPOCH[0], 'idempotency_key': 'n-' + uuid.uuid4().hex[:10], **extra}
    if action != 'START': body['session_id'] = SID[0]
    r = OP.post(f'/robots/{RID}/mapping', body)
    if action == 'START': SID[0] = r['session_id']
    return wait_until(lambda: (lambda s: s if (s.get('command') or {}).get('command_id') == r['command_id'] and s['command']['status'] in ('COMPLETED', 'FAILED', 'UNKNOWN') else None)(get(f'/robots/{RID}/mapping')), timeout)


def navreq(action, **kw):
    return OP.post(f'/robots/{RID}/navigation', {'action': action, 'epoch': EPOCH[0], **kw})


def mission(x, y):
    return OP.post('/dry-run-missions', {'robot_id': RID, 'goal': {'frame_id': 'map', 'x': x, 'y': y},
                                         'lease_epoch': EPOCH[0], 'idempotency_key': 'n-' + uuid.uuid4().hex})


def real_mission(x, y, key=None):
    return OP.post('/missions', {'robot_id': RID, 'type': 'navigate_to_pose', 'goal': {'frame_id': 'map', 'x': x, 'y': y},
                                 'lease_epoch': EPOCH[0], 'idempotency_key': key or 'n-' + uuid.uuid4().hex})


def mstatus(mid):
    return next((m['status'] for m in get(f'/missions?robot_id={RID}&limit=30')['missions'] if m['mission_id'] == mid), None)


TERMINAL = ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED', 'RECONCILIATION_REQUIRED')


def wait_mission(mid, timeout=120):
    return wait_until(lambda: mstatus(mid) if mstatus(mid) in TERMINAL else None, timeout, .5) or mstatus(mid)


def record(name, ok, **d):
    results.append({'scenario': name, 'result': 'PASS' if ok else 'FAIL', **d})
    say(('PASS ' if ok else 'FAIL ') + name, json.dumps(d, default=str)[:900])


GATE_EVENTS = []


def _gate_guard_once(label):
    """Environment guard, not an assertion: a latched Gate fault (e.g. one scan gap under machine load) is cleared only
    through the Gate's own operator unlatch service and every use is recorded in result.json."""
    g = state().get('safety_gate') or {}
    if g.get('fault_latched') is True or g.get('status') not in ('NORMAL', 'STALL_PENDING'):
        from std_srvs.srv import Trigger
        cli = rogue.create_client(Trigger, '/stall_detector/reset_latch')
        ok = cli.wait_for_service(timeout_sec=3)
        res = None
        if ok:
            f = cli.call_async(Trigger.Request()); wait_until(lambda: f.done(), 5)
            res = f.result().success if f.done() and f.result() else None
        rogue.destroy_client(cli)
        time.sleep(1.5)
        GATE_EVENTS.append({'before': label, 'status': g.get('status'), 'last_fault_reasons': g.get('last_fault_reasons'),
                            'unlatch_called': ok, 'unlatch_result': res, 'status_after': (state().get('safety_gate') or {}).get('status')})


def gate_guard(label, timeout=40):
    """2026-11-20: repeat the recorded operator unlatch while the Gate stays blocked (r11 refuses a reset while a lingering DDS
    endpoint of a killed /cmd_vel_raw writer is still visible, ~15-20 s). Each attempt is a GATE_EVENTS entry."""
    end = time.monotonic() + timeout
    while True:
        _gate_guard_once(label)
        g = state().get('safety_gate') or {}
        if (g.get('fault_latched') is not True and g.get('status') in ('NORMAL', 'STALL_PENDING')) or time.monotonic() > end: return
        time.sleep(1.5)


RECOVERY_EVENTS = []
SNAPSHOTS = []


def snap(label):
    """2026-11-27 P0-C: everything the recovery decision depends on, at one instant (evidence only; nothing is relaxed)."""
    r = robot(); st = r.get('state') or {}; ag = r.get('agent') or {}; g = st.get('safety_gate') or {}; wd = st.get('motion_watchdog') or {}; w = (watch[0] or {}); mp = w.get('mode_policy') or {}
    ex = st.get('execution') or {}; lc = st.get('localization') or {}
    d = {'label': label, 't': round(time.time() % 10000, 1), 'online': r.get('online'), 'reconciled': r.get('reconciled'), 'reconciliation_required': r.get('reconciliation_required'),
         'agent_boot': ag.get('boot_id'), 'lease': (r.get('lease') or {}).get('epoch'), 'mode': (r.get('operating_mode') or {}).get('mode'), 'mode_rev': (r.get('operating_mode') or {}).get('revision'),
         'mode_motion': r.get('mode_motion'), 'gate': {k: g.get(k) for k in ('status', 'boot_id', 'fault_epoch', 'fault_latched', 'cmd_blocked', 'contract_met', 'age_s', 'r7_invalid')},
         'motor_path': (g.get('motor_path') or {}).get('status'), 'watchdog': {'state': wd.get('state'), 'epoch': wd.get('epoch'), 'agent_boot_matches': wd.get('agent_boot_matches'), 'age_s': wd.get('age_s')},
         'watchdog_live': {'state': w.get('state'), 'fault': w.get('last_fault'), 'mode': mp.get('mode'), 'goal_owner': mp.get('goal_owner'), 'goal_ok': mp.get('goal_ok')},
         'localization': {'status': lc.get('status'), 'map_version_id': st.get('map_version_id'), 'verified': st.get('map_verified')}, 'nav_idle': (ex.get('nav_idle') or {}).get('ok'),
         'active_goal': bool(ex.get('active')), 'nav2_active_count': (ex.get('nav2_goals') or {}).get('active_count'), 'safety_context_reason': r.get('safety_context_reason'),
         'missions_unresolved': [m['mission_id'] + ':' + m['status'] for m in get(f'/missions?robot_id={RID}&limit=30')['missions'] if m['status'] in ('RECONCILIATION_REQUIRED', 'EXECUTING', 'DISPATCHED', 'CANCELING')]}
    SNAPSHOTS.append(d); return d


def recovery_guard(label):
    """If an earlier scenario left a reconciliation block, use the evidence-based admin recovery (recorded)."""
    r = robot()
    stuck = [m['mission_id'] for m in get(f'/missions?robot_id={RID}&limit=30')['missions'] if m['status'] == 'RECONCILIATION_REQUIRED']
    if r.get('reconciliation_required') or stuck:
        wait_until(lambda: (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED', 20)
        RECOVERY_EVENTS.append({'before': label, 'missions': stuck, 'result': recover_all(),
                                'blocked_after': robot().get('reconciliation_required')})
    if robot().get('gate_boot_reconciliation'):
        # 2026-11-21: a Gate boot change (stack restart / Gate kill) is reconciled only through the evidence-checked admin recovery
        wait_until(lambda: (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED' and (state().get('safety_gate') or {}).get('status') == 'NORMAL', 30)
        try: ADMIN.post(f'/robots/{RID}/recover', {'note': f'e2e {label}: Gate boot changed in the isolated stack'}); res = 'OK'
        except HttpError as e: res = str(e)[:200]
        RECOVERY_EVENTS.append({'before': label, 'gate_boot_reconciliation': res, 'flag_after': robot().get('gate_boot_reconciliation')})


def scenario(fn):
    def run():
        SCEN_MARKS.append((fn.__name__, 'start', round(time.time(), 3)))
        try: _run()
        finally: SCEN_MARKS.append((fn.__name__, 'end', round(time.time(), 3)))
    def _run():
        try: gate_guard(fn.__name__)            # first: a latched Gate also blocks the recovery evidence
        except Exception as e: GATE_EVENTS.append({'before': fn.__name__, 'error': repr(e)[:120]})
        try: recovery_guard(fn.__name__)
        except Exception as e: RECOVERY_EVENTS.append({'before': fn.__name__, 'error': repr(e)[:160]})
        try: fn()
        except Exception as e:
            try:
                st = state(); g = st.get('safety_gate') or {}
                diag = {'gate': g.get('status'), 'block': g.get('movement_block_reasons'), 'watchdog': (st.get('motion_watchdog') or {}).get('state'),
                        'loc': {k: (st.get('localization') or {}).get(k) for k in ('status', 'detail', 'match_ratio')},
                        'arbiter': (st.get('velocity_arbiter') or {}).get('blocker'), 'mode': get(f'/robots/{RID}/mode')['operating_mode']['mode'],
                        'nav2_goals': {k: ((st.get('execution') or {}).get('nav2_goals') or {}).get(k) for k in ('publishers', 'active_count', 'received_status_array', 'age_s', 'status_continuous')},
                        'nav_idle_nav': {k: (((st.get('execution') or {}).get('nav_idle') or {}).get('navigate_to_pose') or {}).get(k) for k in ('ok', 'reason', 'generation', 'stable_s', 'status_from_current_server')}}
            except Exception as d:
                diag = repr(d)[:100]
            record(fn.__name__, False, error=repr(e)[:300], diag=diag, trace=traceback.format_exc()[-600:])
    run.__name__ = fn.__name__; return run


def zero_after(t0, delay=.5, span=.8):
    time.sleep(delay + span)
    return not [s for s in locked if t0 + delay <= s[0] <= t0 + delay + span and (abs(s[1]) > 1e-6 or abs(s[2]) > 1e-6)]


MAP = {}


def find_goal(cands, want_ok=True, reason=None):
    for x, y in cands:
        c = get(f'/robots/{RID}/goal-check?x={x}&y={y}')
        if c['ok'] == want_ok and (reason is None or c['reason'] == reason):
            return (x, y), c
    return None, None


GRID = [(round(-2.6 + 0.4 * i, 2), round(-2.2 + 0.4 * j, 2)) for i in range(24) for j in range(12)]   # 0.4 m grid


# ================================================================== map preparation (web mapping -> Registry)
NAV2_STARTS = []


def ensure_nav2():
    """Known intermittent Nav2 lifecycle bring-up failure (lost change_state response): restart the stack group."""
    for attempt in range(3):
        ok = wait_until(lambda: ((state().get('execution') or {}).get('nav_idle') or {}).get('ok') is True
                        and (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED', 45)
        NAV2_STARTS.append({'attempt': attempt, 'nav2_up': bool(ok)})
        if ok: return True
        # 2026-11-20: a restarted stack is a new Gate boot. The running Agent then (by design) latches r7_invalid
        # (status_seq regression -> reconciliation required) and refuses every mission, so the test Agent is restarted too.
        proc('kill', 'stack'); proc('kill', 'agent'); time.sleep(3); proc('start', 'stack'); time.sleep(6); proc('start', 'agent')
        NAV2_STARTS[-1]['restarted'] = ['stack', 'agent']
    return False


@scenario
def p01_build_map_via_web_mapping():
    assert ensure_nav2(), 'Nav2 did not come up after 3 attempts'
    wait_until(lambda: (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED' and robot().get('online'), 60)
    lease()
    fixed = os.environ.get('NAV_E2E_FIXED_MAP')
    if fixed:
        # 2026-11-21: same map for every run of a comparison series (map quality otherwise differs per exploration). The bundle is
        # registered into THIS run's isolated registry; the files are read only (originals: phase3_stability_claude_20261121/fixed_map).
        code = ("import sys,json,os; sys.path.insert(0,sys.argv[1]); from fleet_server.map_registry import MapRegistry; d=sys.argv[3]; "
                "r=MapRegistry(sys.argv[2]+'/map_registry.db', sys.argv[2]+'/map_store'); rd=lambda n: open(os.path.join(d,n),'rb').read(); "
                "res=r.register_map_bundle(floor_id='paichai_c_1f', label='phase3_fixed_series', yaml_bytes=rd('map.yaml'), pgm_bytes=rd('map.pgm'), "
                "posegraph_bytes=rd('graph.posegraph'), posegraph_data_bytes=rd('graph.data'), source_mode='SLAM_SAVED_BUNDLE', "
                "verification_status='UNVERIFIED', verification_evidence={'method':'isolated_sim_fixed_series'}, frame_id='map', actor='e2e-qa', actor_role='operator'); "
                "mv=res['map_version']['map_version_id']; r.link_map_version_area(mv, 'paichai_c_1f_c103_lounge', actor='e2e-qa'); "
                "print(json.dumps({'id': mv, 'integrity': r.verify_bundle_integrity(mv).get('ok')}))")
        out = subprocess.run([sys.executable, '-c', code, str(FC / 'server'), str(RUN / 'fleet'), fixed], capture_output=True, text=True)
        res = json.loads(out.stdout.strip().splitlines()[-1]) if out.returncode == 0 else {'error': out.stderr[-300:]}
        MAP['id'] = res.get('id')
        record('p01_build_map_via_web_mapping', bool(MAP['id']) and res.get('integrity') is True, map_version_id=MAP['id'],
               fixed_map=fixed, register=res, pose=pose(), note='fixed-map series: web mapping not exercised in this run')
        return
    assert switch('MAPPING')['mode'] == 'MAPPING'
    # Geofence (existing web-mapping option): Frontier goals stay inside the 6 x 5 m room; the corridor stays partly UNKNOWN.
    s = mapping('START', mode='NEW_MAP', limits={'max_duration_s': 600, 'max_goals': 60},
                geofence=[[-2.7, -2.2], [2.7, -2.2], [2.7, 2.2], [-2.7, 2.2]])
    t0 = time.monotonic()
    while time.monotonic() - t0 < 70:
        w = state().get('mapping_worker') or {}
        if w.get('stop_reason') or (w.get('frontier_candidates') == 0 and time.monotonic() - t0 > 40): break
        time.sleep(2)
    mapping('PAUSE')
    s = mapping('SAVE', floor_id='paichai_c_1f', area_id='paichai_c_1f_c103_lounge')
    mapping('STOP'); rec = switch('IDLE')
    MAP['id'] = (s.get('registered') or {}).get('map_version_id')
    record('p01_build_map_via_web_mapping', bool(MAP['id']) and rec['mode'] == 'IDLE', map_version_id=MAP['id'],
           explore_s=round(time.monotonic() - t0, 1), save_evidence=(s.get('result') or {}).get('evidence'), pose=pose())


@scenario
def p02_verify_only_this_sim_map():
    mv = get(f"/map-registry/maps/{MAP['id']}")
    mv = mv.get('map_version', mv)
    MAP['content'] = (mv.get('sha256') or {}).get('content_sha256')
    # The verify route is deliberately NOT exposed over HTTP (permission policy undecided). This isolated QA step
    # promotes ONLY the map this run just built, through the Registry API on the run's own data dir.
    code = ("import sys,json; sys.path.insert(0,sys.argv[1]); from fleet_server.map_registry import MapRegistry; "
            "r=MapRegistry(sys.argv[2]+'/map_registry.db', sys.argv[2]+'/map_store'); "
            "mv=r.verify_map_version(sys.argv[3], status='VERIFIED', evidence={'method':'isolated_sim_e2e','world':'tests/mode_e2e/kinematic_sim.py'}, "
            "actor='e2e-qa', actor_role='admin', reason='isolated sim E2E map'); print(json.dumps({'map_version': mv}, default=str))")
    out = subprocess.run([sys.executable, '-c', code, str(FC / 'server'), str(RUN / 'fleet'), MAP['id']], capture_output=True, text=True)
    r = json.loads(out.stdout.strip().splitlines()[-1]) if out.returncode == 0 else {'error': out.stderr[-300:]}
    others = get('/map-registry/maps')
    others = others.get('map_versions', others.get('maps', []))
    c103 = [m for m in others if m['map_version_id'] != MAP['id']]
    record('p02_verify_only_this_sim_map', r.get('map_version', {}).get('verification_status') == 'VERIFIED'
           and all(m['verification_status'] != 'VERIFIED' for m in c103),
           verified=MAP['id'], other_versions=[(m['map_version_id'], m['verification_status']) for m in c103])


# ================================================================== saved-map navigation
LOAD_TRIES = []


def load_and_localize(mvid, initial, expect_confirm=True, timeout=40):
    # The Worker refuses while a killed map_server/amcl is still a DDS ghost ('graph occupied'); the operator retries.
    for attempt in range(4):
        navreq('LOAD', map_version_id=mvid)
        loaded = wait_until(lambda: loc().get('status') in ('WAIT_INITIAL_POSE', 'FAILED') and loc().get('status'), 40)
        LOAD_TRIES.append((attempt, loaded, loc().get('detail')))
        if loaded == 'WAIT_INITIAL_POSE': break
        time.sleep(10)
    navreq('INITIAL_POSE', x=initial[0], y=initial[1], yaw=initial[2])
    st = wait_until(lambda: loc().get('status') == 'CONFIRMED' and loc(), timeout) if expect_confirm else None
    return loaded, st or loc()


@scenario
def n01_load_map_initial_pose_confirmed():
    rec = switch('NAVIGATION')
    pre = expect_http(lambda: mission(0.5, 0.0), 409, 'no_map_loaded')
    p = pose()
    loaded, l = load_and_localize(MAP['id'], p)
    record('n01_load_map_initial_pose_confirmed', rec['mode'] == 'NAVIGATION' and pre[0] and loaded == 'WAIT_INITIAL_POSE'
           and l.get('status') == 'CONFIRMED' and l.get('planner_allow_unknown') is False and nav()['localized'] is True,
           mission_before_load=pre, loaded=loaded, localization={k: l.get(k) for k in ('status', 'match_ratio', 'amcl_pose', 'planner_allow_unknown', 'map_version_id', 'request_id')},
           truth=p)


@scenario
def n02_goal_validation():
    out = get(f'/robots/{RID}/goal-check?x=40&y=40')
    occ, oc = find_goal([(3.0, 2.0), (3.0, -2.0), (-3.0, 0.0), (0.0, 2.5), (0.0, -2.5)], False, 'OCCUPIED')
    if not occ: occ, oc = find_goal(GRID, False, 'OCCUPIED')
    unk, uc = find_goal([(6.6, 0.0), (6.0, 0.0), (5.5, 0.0)] + GRID, False, 'UNKNOWN_AREA')
    near, nc = find_goal(GRID, False, 'FOOTPRINT_OCCUPIED')
    ok_, okc = find_goal([(1.5, -1.0), (1.5, 1.0), (-1.5, -1.5), (0.5, 0.5)], True)
    m_out = expect_http(lambda: mission(40, 40), 409, 'OUTSIDE_MAP')
    m_occ = expect_http(lambda: mission(*occ), 409, 'goal_invalid') if occ else (False, 'no occupied cell found')
    m_unk = expect_http(lambda: mission(*unk), 409, 'goal_invalid') if unk else (None, 'no UNKNOWN cell inside the map (fully explored)')
    record('n02_goal_validation', out['reason'] == 'OUTSIDE_MAP' and bool(occ) and m_out[0] and m_occ[0] and bool(ok_) and bool(near)
           and (m_unk[0] is not False),
           outside=out, occupied=(occ, oc and oc['reason']), footprint=(near, nc and nc['reason']), unknown=(unk, uc and uc['reason']),
           valid=(ok_, okc and okc['reason']), mission_outside=m_out, mission_occupied=m_occ, mission_unknown=m_unk)
    MAP['valid'] = ok_


@scenario
def n03_valid_goal_succeeds_with_path():
    p0 = pose()
    cands = sorted([c for c in GRID if 1.2 <= math.hypot(c[0] - p0[0], c[1] - p0[1]) <= 2.2], key=lambda c: abs(c[0]) + abs(c[1]))[:30]
    g, _ = find_goal(cands, True)
    g = g or MAP['valid']; r = mission(*g); mid = r['mission_id']
    http_status = r.get('status')
    paths = []
    def watch_path():
        while mstatus(mid) not in TERMINAL:
            p = (loc().get('nav_path') or {})
            if p.get('points'): paths.append(len(p['points']))
            time.sleep(.5)
    th = threading.Thread(target=watch_path, daemon=True); th.start()
    final = wait_mission(mid, 120); th.join(2)
    p = pose()
    record('n03_valid_goal_succeeds_with_path', final == 'SUCCEEDED' and p and math.hypot(p[0] - g[0], p[1] - g[1]) < .4 and bool(paths),
           goal=g, http_accept_status=http_status, final=final, final_pose=p, path_points_seen=max(paths) if paths else 0)


@scenario
def n04_cancel_goal():
    (gx, gy), _ = find_goal([(-1.8, -1.6), (-2.0, 1.8), (2.2, 1.8), (2.2, -1.8)], True)
    r = mission(gx, gy); mid = r['mission_id']
    wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2', 30); time.sleep(1.5)
    t0 = time.monotonic(); OP.post(f'/missions/{mid}/cancel', {})
    final = wait_mission(mid, 30); stopped = zero_after(t0, 1.0, 1.0)
    record('n04_cancel_goal', final == 'CANCELED' and stopped, goal=(gx, gy), final=final, zero_after_cancel=stopped)


def mode_rec():
    return get(f'/robots/{RID}/mode')['operating_mode'] or {}


def in_navigation():
    wait_until(lambda: (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED', 30)
    # after an earlier fault injection Nav2 may still be re-activating (lifecycle bond reset) and the server may still show NAVIGATION that the Watchdog no longer confirms
    # (it ends such a mode itself after a short grace period): wait until the stack is idle again and, if the old mode is unverified, until it is gone, then enter it explicitly
    wait_until(lambda: ((state().get('execution') or {}).get('nav_idle') or {}).get('ok') is True and (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED', 60)
    if mode_rec().get('mode') == 'NAVIGATION' and (robot().get('mode_motion') or {}).get('verified') is False:
        wait_until(lambda: mode_rec().get('mode') != 'NAVIGATION', 12)
    if mode_rec().get('mode') == 'FAULT': switch('IDLE')
    if mode_rec().get('mode') != 'NAVIGATION': switch('NAVIGATION')
    if loc().get('status') != 'CONFIRMED': load_and_localize(MAP['id'], pose())


def active_goal():
    return (state().get('execution') or {}).get('active')


@scenario
def d01_dry_run_separation_and_flags():
    in_navigation()
    ag = robot().get('agent') or {}
    flags = ag.get('real_nav_dry_run') is True and ag.get('missions_enabled') is False and ag.get('velocity_selector') == 'WATCHDOG_SELECTOR' and not ag.get('locked_velocity_arbiter')
    (gx, gy), _ = find_goal([(-1.8, -1.6), (-2.0, 1.8), (2.2, 1.8), (2.2, -1.8)], True)
    refused, why = expect_http(lambda: real_mission(gx, gy), 409, 'dry_run_agent')
    r = mission(gx, gy)
    view_ok = r.get('dry_run') is True and r.get('real_motion_authorized') is False
    OP.post(f'/missions/{r["mission_id"]}/cancel', {}); final = wait_mission(r['mission_id'], 30)
    record('d01_dry_run_separation_and_flags', flags and refused and view_ok and final == 'CANCELED', agent_flags={k: ag.get(k) for k in ('real_nav_dry_run', 'missions_enabled', 'velocity_selector', 'locked_velocity_arbiter', 'mode', 'simulated')},
           real_endpoint_refused=[refused, why], dry_view={k: r.get(k) for k in ('dry_run', 'real_motion_authorized', 'status')}, final=final)


@scenario
def d02_goal_ownership_generation_and_motion_path():
    in_navigation()
    (gx, gy), _ = find_goal([(-1.8, -1.6), (-2.0, 1.8), (2.2, 1.8), (2.2, -1.8)], True)
    om = mode_rec(); t0 = time.monotonic()
    r = mission(gx, gy); mid = r['mission_id']
    act = wait_until(active_goal, 30); ok_move = wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2', 30)
    time.sleep(3.0)
    wd = watch[0] or {}; mp = wd.get('mode_policy') or {}
    m = next((x for x in get(f'/missions?robot_id={RID}&limit=30')['missions'] if x['mission_id'] == mid), {})
    cmd = (m.get('commands') or [{}])[-1]; boot = (robot().get('agent') or {}).get('boot_id')
    flow = [s for s in locked if s[0] >= t0 and (abs(s[1]) > 1e-6 or abs(s[2]) > 1e-6)]
    own = (isinstance(act, dict) and act.get('mission_id') == mid and cmd.get('owner_boot_id') == boot and cmd.get('type') == 'navigate_to_pose'
           and mp.get('mode') == 'NAVIGATION' and mp.get('goal_owner') == 'fleet' and mp.get('goal_ok') is True and mp.get('revision') == om.get('revision'))
    OP.post(f'/missions/{mid}/cancel', {}); final = wait_mission(mid, 30); t1 = time.monotonic()
    stopped = zero_after(t1, 1.0, 1.0)
    record('d02_goal_ownership_generation_and_motion_path', own and bool(flow) and max(abs(s[1]) for s in flow) <= 0.1501 and final == 'CANCELED' and stopped,
           active={k: (act or {}).get(k) for k in ('mission_id', 'command_id', 'goal_uuid')}, owner_boot_ok=cmd.get('owner_boot_id') == boot, watchdog={k: mp.get(k) for k in ('mode', 'revision', 'goal_owner', 'goal_ok', 'selected_channel')},
           mode_revision=om.get('revision'), gate_nonzero_samples=len(flow), peak_linear=max([abs(s[1]) for s in flow] or [0]), final=final, zero_after_cancel=stopped)


@scenario
def d03_cancel_result_then_idle():
    in_navigation()
    (gx, gy), _ = find_goal([(-1.8, -1.6), (-2.0, 1.8), (2.2, 1.8), (2.2, -1.8)], True)
    r = mission(gx, gy); mid = r['mission_id']; wait_until(active_goal, 30); time.sleep(2.0)
    OP.post(f'/missions/{mid}/cancel', {}); final = wait_mission(mid, 30)
    idle_goal = wait_until(lambda: active_goal() is None and ((state().get('execution') or {}).get('nav_idle') or {}).get('ok') is True, 30)
    t0 = time.monotonic(); om = switch('IDLE'); wd = (watch[0] or {}).get('mode_policy') or {}
    stopped = zero_after(t0, 1.0, 1.5)
    refused, why = expect_http(lambda: mission(gx, gy), 409, 'mode_not_navigation')
    record('d03_cancel_result_then_idle', final == 'CANCELED' and bool(idle_goal) and om.get('mode') == 'IDLE' and stopped and refused,
           final=final, nav_idle_after_cancel=bool(idle_goal), mode=om.get('mode'), watchdog_mode=wd.get('mode'), zero_after_idle=stopped, mission_in_idle_refused=[refused, why])
    in_navigation()                      # restore NAVIGATION + localization for the scenarios that follow


@scenario
def d04_lease_loss_old_goal_not_reused_no_restart():
    in_navigation()
    (gx, gy), _ = find_goal([(-1.8, -1.6), (-2.0, 1.8), (2.2, 1.8), (2.2, -1.8)], True)
    key = 'n-' + uuid.uuid4().hex
    r = OP.post('/dry-run-missions', {'robot_id': RID, 'goal': {'frame_id': 'map', 'x': gx, 'y': gy}, 'lease_epoch': EPOCH[0], 'idempotency_key': key}); mid = r['mission_id']
    wait_until(active_goal, 30); wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2', 30); time.sleep(1.5)
    t0 = time.monotonic(); OP.post(f'/robots/{RID}/lock', {'action': 'release'})
    final = wait_mission(mid, 40); stopped = zero_after(t0, 3.0, 2.0)
    ended = wait_until(lambda: mode_rec().get('mode') in ('IDLE', 'FAULT'), 40)
    lease()
    time.sleep(10.0); n0 = len(locked); time.sleep(4.0)
    nonzero = [s for s in locked[n0:] if abs(s[1]) > 1e-6 or abs(s[2]) > 1e-6]
    same_key = None
    try:
        again = OP.post('/dry-run-missions', {'robot_id': RID, 'goal': {'frame_id': 'map', 'x': gx, 'y': gy}, 'lease_epoch': EPOCH[0], 'idempotency_key': key}); same_key = ['accepted', again.get('mission_id') == mid, again.get('status')]
    except HttpError as e: same_key = ['refused', e.status, str(e)[:120]]
    cmds_after = len([c for c in next((x for x in get(f'/missions?robot_id={RID}&limit=30')['missions'] if x['mission_id'] == mid), {}).get('commands', []) if c['type'] == 'navigate_to_pose'])
    ok_reuse = (same_key[0] == 'refused') or (same_key[0] == 'accepted' and same_key[1] is True and same_key[2] in TERMINAL)
    record('d04_lease_loss_old_goal_not_reused_no_restart', final in TERMINAL and stopped and bool(ended) and mode_rec().get('mode') != 'NAVIGATION' and not nonzero and ok_reuse and cmds_after == 1,
           mission_final=final, zero_after_release=stopped, mode_after=mode_rec().get('mode'), nonzero_after_reacquire=len(nonzero), same_key_repost=same_key, navigate_commands_for_old_mission=cmds_after)


@scenario
def d05_watchdog_kill_during_dry_run_goal():
    in_navigation()
    (gx, gy), _ = find_goal([(-1.8, -1.6), (-2.0, 1.8), (2.2, 1.8), (2.2, -1.8)], True)
    mid = mission(gx, gy)['mission_id']; wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2', 30); time.sleep(1.5)
    n0 = len(locked); t0 = proc('kill', 'watchdog'); time.sleep(3.0)
    after = [s for s in locked[n0:] if s[0] > t0 + .4 and (abs(s[1]) > 1e-6 or abs(s[2]) > 1e-6)]
    proc('start', 'watchdog'); time.sleep(8.0)
    wait_until(lambda: mstatus(mid) in TERMINAL, 40)
    record('d05_watchdog_kill_during_dry_run_goal', not after, nonzero_after_kill=len(after), mission=mstatus(mid), note='Watchdog killed while a dry-run goal drove the Gate input; restarted afterwards (new boot: the Gate input source is not trusted until the operator resets it)')



@scenario
def d06_continuous_new_active_cancel_idle_restart_recover_new_goal():
    """NEW -> ACTIVE -> CANCEL -> IDLE -> (second goal ACTIVE) -> Agent restart -> admin recovery -> new goal approved and run. All motor-command observation is on the Gate output; nothing here proves a physical motor stop."""
    steps, cands = [], [(-1.8, -1.6), (-2.0, 1.8), (2.2, 1.8), (2.2, -1.8)]
    in_navigation()
    (g1x, g1y), _ = find_goal(cands, True)
    r1 = mission(g1x, g1y); m1 = r1['mission_id']; steps.append(['NEW', r1.get('status'), r1.get('dry_run'), r1.get('real_motion_authorized')])
    act = wait_until(active_goal, 30); steps.append(['ACTIVE', bool(act)]); time.sleep(1.5)
    OP.post(f'/missions/{m1}/cancel', {}); f1 = wait_mission(m1, 30); steps.append(['CANCEL', f1])
    idle = wait_until(lambda: active_goal() is None and ((state().get('execution') or {}).get('nav_idle') or {}).get('ok') is True, 30)
    om = switch('IDLE'); steps.append(['IDLE', om.get('mode'), bool(idle)])
    in_navigation()
    (g2x, g2y), _ = find_goal([(-1.8, 1.6), (2.2, -1.6)] + cands, True)
    r2 = mission(g2x, g2y); m2 = r2['mission_id']; steps.append(['NEW2', r2.get('status')])
    wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2', 30); time.sleep(1.0); steps.append(['ACTIVE2', bool(active_goal())])
    snap('before_restart'); old = (robot().get('agent') or {}).get('boot_id')
    n0 = len(locked); t0 = proc('kill', 'agent'); stopped = zero_after(t0, .5, 1.0)
    proc('start', 'agent')
    wait_until(lambda: (robot().get('agent') or {}).get('boot_id') != old and robot().get('online') and robot().get('reconciled'), 60); time.sleep(3)
    nz_after_restart = [x for x in locked[n0:] if x[0] > t0 + .8 and (abs(x[1]) > 1e-6 or abs(x[2]) > 1e-6)]
    s_after = snap('after_restart'); m2_after = mstatus(m2); mode_after = mode_rec().get('mode')
    steps.append(['RESTART', {'zero_within_0_5s': stopped, 'mission': m2_after, 'mode': mode_after, 'nonzero_after_restart': len(nz_after_restart)}])
    reuse = expect_http(lambda: mission(g2x, g2y), 409)
    recovery = []; recovered = False
    for k in range(25):
        gate_guard('d06_recovery'); rr = recover_all(); recovery.append(rr); snap(f'd06_recovery_attempt_{k} ' + json.dumps(rr)[:200])
        if not robot().get('reconciliation_required') and mstatus(m2) in TERMINAL and mstatus(m2) != 'RECONCILIATION_REQUIRED': recovered = True; break
        time.sleep(3)
    steps.append(['RECOVER', recovered, mstatus(m2), recovery[-1][:2] if recovery else None])
    nz_during = [x for x in locked[n0:] if x[0] > t0 + .8 and (abs(x[1]) > 1e-6 or abs(x[2]) > 1e-6)]
    rec2 = switch('IDLE'); rec2 = switch('NAVIGATION'); loaded, l = load_and_localize(MAP['id'], pose())
    r3 = mission(*MAP['valid']); m3 = r3['mission_id']; steps.append(['NEW3', r3.get('status'), r3.get('dry_run')])
    final = wait_mission(m3, 120); steps.append(['NEW_GOAL_RESULT', final])
    record('d06_continuous_new_active_cancel_idle_restart_recover_new_goal',
           f1 == 'CANCELED' and om.get('mode') == 'IDLE' and bool(idle) and stopped and not nz_after_restart and reuse[0] and recovered and not nz_during and final == 'SUCCEEDED' and r3.get('dry_run') is True,
           steps=steps, old_goal_reuse=reuse, recovery_attempts=len(recovery), boot_changed=(s_after.get('agent_boot') != old), nonzero_gate_samples_until_recovery=len(nz_during))


def reset_call():
    """The operator's Gate reset (service). Returns (success, message)."""
    from std_srvs.srv import Trigger
    cli = rogue.create_client(Trigger, '/stall_detector/reset_latch')
    try:
        if not cli.wait_for_service(timeout_sec=3): return False, 'service unavailable'
        f = cli.call_async(Trigger.Request()); wait_until(lambda: f.done(), 6)
        return (bool(f.result().success), f.result().message) if f.done() and f.result() else (False, 'no answer')
    finally:
        rogue.destroy_client(cli)


@scenario
def d07_gate_reset_policy_and_no_resume():
    """P0-B: a gated Gate reset needs ONE identified writer, a fresh explicit zero from it and quiet; a successful reset never resumes the earlier goal/mode."""
    in_navigation()
    (gx, gy), _ = find_goal([(-1.8, -1.6), (-2.0, 1.8), (2.2, 1.8), (2.2, -1.8)], True)
    mid = mission(gx, gy)['mission_id']; wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2', 30); time.sleep(1.0)
    m = Twist(); m.linear.x = .1; raw = rogue.create_publisher(Twist, '/cmd_vel_raw', 10); end = time.monotonic() + 2.5
    while time.monotonic() < end: raw.publish(m); time.sleep(.05)
    latched = wait_until(lambda: (state().get('safety_gate') or {}).get('status') not in ('NORMAL', 'STALL_PENDING'), 10)
    r_rogue_alive = reset_call(); rogue.destroy_publisher(raw); t_gone = time.monotonic()
    r_lingering = reset_call()
    # the Watchdog is frozen: the single writer exists but delivers no zero -> INPUT_SILENT; the reset is not allowed "because nothing is wrong any more"
    wd_pid = proc('stop', 'watchdog'); time.sleep(2.5)
    silent = []
    for _ in range(14):
        r = reset_call(); silent.append(r)
        if 'INPUT_WRITERS' not in r[1] and not r[0]: break
        time.sleep(2.0)
    r_silent = silent[-1]
    proc('cont', 'watchdog'); time.sleep(1.5)
    accepted = None
    for _ in range(25):
        r = reset_call()
        if r[0]: accepted = r; break
        time.sleep(1.5)
    n0 = len(locked); time.sleep(6.0)
    nz = [x for x in locked[n0:] if abs(x[1]) > 1e-6 or abs(x[2]) > 1e-6]
    st_m = mstatus(mid); md = mode_rec().get('mode'); gate_after = (state().get('safety_gate') or {}).get('status')
    refused_new = expect_http(lambda: mission(gx, gy), 409); mm_after = robot().get('mode_motion') or {}
    record('d07_gate_reset_policy_and_no_resume',
           bool(latched) and not r_rogue_alive[0] and 'INPUT_WRITERS' in r_rogue_alive[1] and not r_lingering[0] and (not r_silent[0] and 'INPUT_SILENT' in r_silent[1]) and accepted is not None
           and not nz and st_m in TERMINAL and (md != 'NAVIGATION' or mm_after.get('verified') is False) and refused_new[0],
           latched=bool(latched), reset_while_rogue_alive=r_rogue_alive, reset_right_after_rogue_gone=r_lingering, reset_with_no_zero_input=r_silent, silent_attempts=len(silent), reset_accepted=accepted,
           nonzero_gate_samples_after_reset=len(nz), old_mission=st_m, server_mode_after=md, gate_after=gate_after, mode_motion_after=mm_after, new_mission_without_new_approval_refused=refused_new,
           note='the Gate never accepted a reset on its own; motion needs a new MODE request, a new goal and (manual) a neutral command')


@scenario
def f01_version_mismatch_and_unknown_version():
    before = (nav()['request'] or {}).get('request_id')
    a = expect_http(lambda: navreq('LOAD', map_version_id=MAP['id'], expected_content_sha256='0' * 64), 409, 'map_version_mismatch')
    b = expect_http(lambda: navreq('LOAD', map_version_id='nope_000000#000000000000'), 404)
    still = (nav()['request'] or {}).get('request_id') == before and before is not None     # refused LOADs change nothing
    record('f01_version_mismatch_and_unknown_version', a[0] and b[0] and still is True, mismatch=a, unknown=b, localization_kept=still)


@scenario
def f02_corrupted_map_file_refused():
    prefix = MAP['id'].replace('#', '__') + '__'          # Registry storage key of THIS run's map only
    dirs = [d for d in (RUN / 'fleet' / 'map_store' / 'bundles').iterdir() if d.name.startswith(prefix)]
    assert len(dirs) == 1, dirs
    target = dirs[0] / 'map.pgm'
    expect = hashlib.sha256(target.read_bytes()).hexdigest()
    original = target.read_bytes(); mode = target.stat().st_mode
    os.chmod(target, 0o644); bad = bytearray(original); bad[-10] ^= 0xFF; target.write_bytes(bytes(bad))
    try:
        r = expect_http(lambda: navreq('LOAD', map_version_id=MAP['id']), 409, 'map_integrity_failed')
    finally:
        target.write_bytes(original); os.chmod(target, mode)
    ok_after = hashlib.sha256(target.read_bytes()).hexdigest() == expect
    record('f02_corrupted_map_file_refused', r[0] and ok_after, load=r, restored=ok_after,
           note='isolated test map in the run directory, restored byte-for-byte; C103/H107 originals untouched')


@scenario
def f03_wrong_initial_pose_not_confirmed():
    p = pose(); wrong = None
    for dx, dy in ((-1.2, 0), (1.2, 0), (-1.2, -p[1]), (1.2, -p[1]), (0, 1.5), (0, -1.5), (-1.5, -1.5)):
        cand = (p[0] + dx, p[1] + dy, p[2] + 1.0)     # a FREE cell >= 1.2 m away: accepted as a pose, wrong for AMCL
        try:
            navreq('INITIAL_POSE', x=cand[0], y=cand[1], yaw=cand[2]); wrong = cand; break
        except HttpError:
            continue
    assert wrong, 'no free cell for a wrong initial pose'
    time.sleep(8); l = loc()
    m = expect_http(lambda: mission(*MAP['valid']), 409, 'localization_not_confirmed')
    navreq('INITIAL_POSE', x=p[0], y=p[1], yaw=p[2])
    ok = wait_until(lambda: loc().get('status') == 'CONFIRMED', 40)
    record('f03_wrong_initial_pose_not_confirmed', l.get('status') != 'CONFIRMED' and m[0] and bool(ok),
           wrong_pose=wrong, status_with_wrong_pose=l.get('status'), match_ratio=l.get('match_ratio'), mission=m, reconfirmed=bool(ok))


@scenario
def n05_navigation_manual_navigation():
    (gx, gy), _ = find_goal([(-1.8, 1.6), (2.2, -1.6), (-1.8, -1.6)], True)
    mid = mission(gx, gy)['mission_id']
    wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2', 30); time.sleep(1)
    rec = switch('MANUAL'); st = mstatus(mid)
    cleared = wait_until(lambda: loc().get('status') == 'IDLE' and not loc().get('processes_started') and nav()['request'] is None, 20)
    nomap = expect_http(lambda: navreq('LOAD', map_version_id=MAP['id']), 409, 'mode_not_navigation')
    rec2 = switch('NAVIGATION')
    loaded, l = load_and_localize(MAP['id'], pose())
    final = wait_mission(mission(*MAP['valid'])['mission_id'], 120)
    record('n05_navigation_manual_navigation', rec['mode'] == 'MANUAL' and st == 'CANCELED' and bool(cleared) and nomap[0]
           and rec2['mode'] == 'NAVIGATION' and l.get('status') == 'CONFIRMED' and final == 'SUCCEEDED',
           mission_on_switch=st, localization_stopped=bool(cleared), load_in_manual=nomap, relocalized=l.get('status'), second_goal=final)


@scenario
def f04_nav2_goal_failure_planner_frozen():
    in_navigation()          # a Watchdog trip in an earlier scenario ends the server mode on purpose: re-enter it explicitly
    pid = [owned_pid(RUN, r'__node:=planner_server', groups=('stack',))]
    os.kill(int(pid[0]), 19)
    try:
        (gx, gy), _ = find_goal([(-1.8, -1.6), (2.2, 1.6)], True)
        mid = mission(gx, gy)['mission_id']; final = wait_mission(mid, 90)
    finally:
        os.kill(int(pid[0]), 18)
    time.sleep(6)                                      # let the stack settle before the next scenario's Gate guard
    record('f04_nav2_goal_failure_planner_frozen', final in ('FAILED', 'CANCELED', 'RECONCILIATION_REQUIRED') and final != 'SUCCEEDED',
           final=final, note='planner_server SIGSTOP: ComputePathToPose unanswered -> Nav2 aborts the goal')


@scenario
def f05_amcl_killed_during_goal():
    in_navigation()          # a Watchdog trip in an earlier scenario ends the server mode on purpose: re-enter it explicitly
    (gx, gy), _ = find_goal([(-1.8, 1.6), (2.2, -1.6), (-1.8, -1.6)], True)
    mid = mission(gx, gy)['mission_id']
    wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2', 30); time.sleep(1)
    pid = [owned_pid(RUN, r'nav2_amcl/amcl', groups=('agent',))]
    t0 = time.monotonic(); os.kill(int(pid[0]), 9)
    st = wait_until(lambda: loc().get('status') == 'FAILED' and loc(), 10) or loc()
    final = wait_mission(mid, 30); stopped = zero_after(t0, 3.0, 1.5)
    refuse = expect_http(lambda: mission(*MAP['valid']), 409)
    record('f05_amcl_killed_during_goal', st.get('status') == 'FAILED' and final != 'SUCCEEDED' and stopped and refuse[0],
           localization=st.get('status'), detail=st.get('detail'), mission=final, zero_within_3s=stopped, new_mission=refuse)
    NEED_RECOVERY.append(mid)


NEED_RECOVERY = []


def recover_all():
    recs = []
    for mid in [m['mission_id'] for m in get(f'/missions?robot_id={RID}&limit=30')['missions'] if m['status'] == 'RECONCILIATION_REQUIRED']:
        try: ADMIN.post(f'/missions/{mid}/recover', {'note': 'Nav2 terminal evidence reviewed in the isolated E2E'}); recs.append((mid, 'OK'))
        except HttpError as e: recs.append((mid, str(e)[:200]))
    if robot().get('reconciliation_required'):
        try: ADMIN.post(f'/robots/{RID}/recover', {'note': 'Nav2 terminal evidence reviewed in the isolated E2E'}); recs.append(('robot', 'OK'))
        except HttpError as e: recs.append(('robot', str(e)[:200]))
    return recs


@scenario
def f06_relocalize_after_localization_failure():
    rec = switch('NAVIGATION')
    loaded, l = load_and_localize(MAP['id'], pose())
    record('f06_relocalize_after_localization_failure', l.get('status') == 'CONFIRMED', load_attempts=LOAD_TRIES[-4:],
           relocalized=l.get('status'), note='first LOAD may be refused while the SIGKILLed amcl is still in the DDS graph')


@scenario
def f07_bypass_attempts():
    t0 = time.monotonic()
    rogue_out = rogue.create_publisher(Twist, '/fleet/nav_cmd_vel', 10)
    m = Twist(); m.linear.x = .1
    end = time.monotonic() + 3
    while time.monotonic() < end: rogue_out.publish(m); time.sleep(.05)
    wd = (watch[0] or {}).get('last_fault'); arb = (state().get('velocity_arbiter') or {}).get('blocker')
    moved = [s for s in locked if s[0] > t0 + .6 and s[1] > 1e-6]
    rogue.destroy_publisher(rogue_out)
    t1 = time.monotonic(); raw = rogue.create_publisher(Twist, '/cmd_vel_raw', 10)
    end = time.monotonic() + 3
    while time.monotonic() < end: raw.publish(m); time.sleep(.05)
    raw_moved = len([s for s in locked if s[0] > t1 + .6 and s[1] > 1e-6])
    raw_rogue_vals = len([s for s in locked if s[0] >= t1 and abs(s[1] - .1) < 1e-4])
    raw_late = len([s for s in locked if s[0] > t1 + .3 and (abs(s[1]) > 1e-6 or abs(s[2]) > 1e-6)])
    raw_blocker = (state().get('velocity_arbiter') or {}).get('blocker')
    gate_now = state().get('safety_gate') or {}
    rogue.destroy_publisher(raw)
    # 2026-11-20: Gate input-source validation is now asserted separately (f07 itself is unchanged).
    record('f07b_gate_input_rogue_raw', raw_rogue_vals == 0 and raw_late == 0,
           rogue_value_samples=raw_rogue_vals, nonzero_after_0_3s=raw_late, gate_status=gate_now.get('status'),
           gate_last_fault_reasons=(gate_now.get('last_fault_reasons') or [])[:3], arbiter=raw_blocker)
    record('f07_bypass_attempts', not moved and wd == 'VELOCITY_INPUT_NOT_EXCLUSIVE',
           watchdog_input_rogue={'gate_nonzero_samples_after_0_6s': len(moved), 'watchdog_fault': wd, 'arbiter': arb},
           gate_input_rogue={'gate_nonzero_samples_after_0_6s': raw_moved, 'arbiter': raw_blocker,
                             'note': 'reported here; asserted in f07b (r10 and the field Gate do not check their input, candidate r11 does)'})


@scenario
def f08_agent_kill_during_goal_then_recover():
    in_navigation()          # a Watchdog trip in an earlier scenario ends the server mode on purpose: re-enter it explicitly
    wait_until(lambda: (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED'
               and (state().get('velocity_arbiter') or {}).get('blocker') != 'VELOCITY_GRAPH_NOT_LOCKED', 40)   # after f07's rogue publishers
    rec = switch('NAVIGATION')
    if (loc().get('status') != 'CONFIRMED'): load_and_localize(MAP['id'], pose())
    (gx, gy), _ = find_goal([(-1.8, 1.6), (2.2, -1.6)], True)
    mid = mission(gx, gy)['mission_id']
    wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2', 30); time.sleep(1)
    old = (robot().get('agent') or {}).get('boot_id')
    t0 = proc('kill', 'agent'); stopped = zero_after(t0, .5, 1.0)
    proc('start', 'agent')
    wait_until(lambda: (robot().get('agent') or {}).get('boot_id') != old and robot().get('online') and robot().get('reconciled'), 40)
    time.sleep(3)
    m_after = mstatus(mid); mode_after = get(f'/robots/{RID}/mode')['operating_mode']['mode']
    loc_after = loc().get('status'); blocked = robot().get('reconciliation_required')
    reuse = expect_http(lambda: mission(*MAP['valid']), 409)
    # Evidence-based recovery (Nav2 terminal for the old goal + idle/Watchdog/Arbiter checks), admin + note, audited.
    recovery = []
    if blocked or m_after == 'RECONCILIATION_REQUIRED':
        # Evidence appears only once the new Agent, Watchdog and Nav2 status agree: retry, each attempt audited.
        for _ in range(20):                               # the Watchdog re-arms ~20 s after a SIGKILLed Agent (DDS lease)
            gate_guard('f08_recovery')
            rr = recover_all(); recovery += rr; snap('f08_recovery_attempt ' + json.dumps(rr)[:300])
            if not robot().get('reconciliation_required') and mstatus(mid) != 'RECONCILIATION_REQUIRED': break
            time.sleep(3)
    time.sleep(1)
    audit = [(e['kind'], e['message'][:100]) for e in get('/events?limit=30')['events'] if 'recover' in e['kind']]
    after = robot().get('reconciliation_required'); m_final = mstatus(mid)
    rec2 = switch('IDLE'); rec2 = switch('NAVIGATION'); loaded, l = load_and_localize(MAP['id'], pose())
    final = wait_mission(mission(*MAP['valid'])['mission_id'], 120)
    record('f08_agent_kill_during_goal_then_recover', stopped and mode_after in ('IDLE', 'FAULT') and loc_after in ('IDLE', 'UNAVAILABLE', None)
           and reuse[0] and not after and m_final in ('CANCELED', 'FAILED', 'SUCCEEDED') and final == 'SUCCEEDED',
           zero_within_0_5s=stopped, mission_after_restart=m_after, mode=mode_after, localization_after_restart=loc_after,
           old_session_reuse=reuse, blocked_before_recovery=blocked, recovery=recovery, audit=audit[:4],
           blocked_after=after, mission_final=m_final, new_goal_after_recovery=final,
           note='no automatic reuse: map/pose request cleared, operator recovers with evidence, reloads and re-localizes')


@scenario
def f09_gate_kill_stops():
    wait_until(lambda: (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED', 30)
    rec = switch('NAVIGATION')
    if loc().get('status') != 'CONFIRMED': load_and_localize(MAP['id'], pose())
    (gx, gy), _ = find_goal([(-1.8, -1.6), (2.2, 1.6)], True)
    mid = mission(gx, gy)['mission_id']
    wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2', 30); time.sleep(1)
    n0 = len(locked); t0 = proc('kill', 'stack')        # time when the kill command returned
    time.sleep(2)
    after = [s for s in locked[n0:] if s[0] > t0 + .3]
    record('f09_gate_kill_stops', not after, gate_output_after_kill=len(after), mission=mstatus(mid),
           note='whole stack process group (Gate, Nav2, ICP) killed; nothing reaches /cmd_vel_locked. MCU behaviour unverified')


@scenario
def r01_gate_boot_change_then_reconcile_and_mission():
    """2026-11-21 reproduction of n03 'safety_not_ready' after a Gate boot change (not in the default ORDER; NAV_E2E_ONLY)."""
    switch('NAVIGATION')
    if loc().get('status') != 'CONFIRMED': load_and_localize(MAP['id'], pose())
    boot0 = (state().get('safety_gate') or {}).get('boot_id')
    proc('kill', 'stack'); proc('kill', 'agent'); time.sleep(3); proc('start', 'stack'); time.sleep(6); proc('start', 'agent')
    up = wait_until(lambda: ((state().get('execution') or {}).get('nav_idle') or {}).get('ok') is True
                    and (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED' and (state().get('safety_gate') or {}).get('boot_id') not in (None, boot0), 90)
    flag = wait_until(lambda: robot().get('gate_boot_reconciliation'), 20)
    lease(); switch('NAVIGATION'); load_and_localize(MAP['id'], pose())
    p0 = pose(); cands = sorted([c for c in GRID if 1.2 <= math.hypot(c[0] - p0[0], c[1] - p0[1]) <= 2.2], key=lambda c: abs(c[0]) + abs(c[1]))[:30]
    g = find_goal(cands, True)[0]
    blocked = expect_http(lambda: mission(*g), 409, 'safety_not_ready')
    reason_blocked = (robot().get('safety_context_reason') or {}).get('reason')
    try: ADMIN.post(f'/robots/{RID}/recover', {'note': 'e2e r01: Gate boot changed by stack restart, Nav2 idle'}); rec = 'OK'
    except HttpError as e: rec = str(e)[:200]
    cleared = not robot().get('gate_boot_reconciliation')
    m = mission(*g); final = wait_until(lambda: mstatus(m['mission_id']) if mstatus(m['mission_id']) in ('SUCCEEDED', 'FAILED', 'CANCELED', 'RECONCILIATION_REQUIRED') else None, 90)
    record('r01_gate_boot_change_then_reconcile_and_mission', bool(up) and bool(flag) and blocked[0] and reason_blocked == 'GATE_BOOT_RECONCILIATION'
           and rec == 'OK' and cleared and final == 'SUCCEEDED', gate_boot_before=boot0, gate_boot_after=(state().get('safety_gate') or {}).get('boot_id'),
           flag_set=bool(flag), mission_before_recovery=blocked, first_failing_check=reason_blocked, admin_recovery=rec, flag_cleared=cleared, mission_after=final)


ORDER = [p01_build_map_via_web_mapping, p02_verify_only_this_sim_map, n01_load_map_initial_pose_confirmed, n02_goal_validation,
         n03_valid_goal_succeeds_with_path, n04_cancel_goal, d01_dry_run_separation_and_flags, d02_goal_ownership_generation_and_motion_path, d03_cancel_result_then_idle,
         f01_version_mismatch_and_unknown_version, f02_corrupted_map_file_refused, f03_wrong_initial_pose_not_confirmed,
         f04_nav2_goal_failure_planner_frozen, f05_amcl_killed_during_goal, f06_relocalize_after_localization_failure, f07_bypass_attempts,
         f08_agent_kill_during_goal_then_recover, d04_lease_loss_old_goal_not_reused_no_restart, d05_watchdog_kill_during_dry_run_goal, d06_continuous_new_active_cancel_idle_restart_recover_new_goal, d07_gate_reset_policy_and_no_resume, f09_gate_kill_stops]   # n05 (manual web bench + mission) does not apply: this shape has no web bench
try:
    only = os.environ.get('NAV_E2E_ONLY')
    EXTRA = [r01_gate_boot_change_then_reconcile_and_mission]   # reproduction scenarios: run only when named in NAV_E2E_ONLY
    for fn in ORDER + (EXTRA if only else []):
        if only and fn.__name__ not in only.split(','): continue
        say('--- ' + fn.__name__); fn()
    record('no_cmd_vel_endpoints', obs.count_publishers('/cmd_vel') == 0 and obs.count_subscribers('/cmd_vel') == 0)
finally:
    if os.environ.get('E2E_KEEP_STACK') != '1':          # 2026-11-28 (integrated copy only): E2E_KEEP_STACK=1 leaves the isolated stack running for a manual browser check (the caller stops it)
        subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), 'stopall', 'x'], check=False)
    try: os.killpg(TIMELINE_PROC.pid, 2); TIMELINE_PROC.wait(5)
    except Exception: pass
    summary = {'port': PORT, 'domain': DOMAIN, 'scenario_marks': SCEN_MARKS, 'map_version_id': MAP.get('id'), 'nav2_bringup_attempts': NAV2_STARTS, 'gate_unlatch_events': GATE_EVENTS, 'snapshots': SNAPSHOTS, 'recovery_events': RECOVERY_EVENTS, 'results': results,
               'counts': {k: sum(r['result'] == k for r in results) for k in ('PASS', 'FAIL', 'SKIP')}, 'finished': time.strftime('%FT%T')}
    (OUT / 'result.json').write_text(json.dumps(summary, indent=1, default=str))
    say('SUMMARY', summary['counts']); log.flush()
    os._exit(0 if summary['counts']['FAIL'] == 0 else 1)
