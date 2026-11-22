"""Isolated E2E of the admin web Gate reset (test-only; ROS_LOCALHOST_ONLY=1, private domain, no driver, Gate output /cmd_vel_locked).

Stack: tests/mode_e2e/stack_up.sh (Fleet --require-operating-mode, kinematic sim, Nav2 + Gate candidate r12 with input validation,
Motion Watchdog, Agent with web_gate_reset:=true). Faults are induced only in this isolated graph.
Usage: OVERLAY=<explore_lite overlay> GATE_RESET_E2E_OUT=<new dir> python3 run_gate_reset_e2e.py <port> <domain> [--browser]
"""
import json, math, os, signal, subprocess, sys, threading, time, traceback, uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent; FC = HERE.parents[1]; REPO = FC.parent; MODE = FC / 'tests' / 'mode_e2e'
PORT, DOMAIN = int(sys.argv[1]), sys.argv[2]
assert PORT not in (8080, 18190, 18191, 18192) and DOMAIN not in ('0', '77')
OUT = Path(os.environ['GATE_RESET_E2E_OUT']).resolve(); OUT.mkdir(parents=True, exist_ok=False)
RUN = OUT / 'stack'; RID = 'iso_bench'; BROWSER = '--browser' in sys.argv
R12 = REPO / 'experiments/safety_chain_fix_20261113/candidate_gate_r12/nodes/stall_detector_node.py'
log = (OUT / 'harness.log').open('x'); results = []


def say(*a):
    line = time.strftime('%H:%M:%S') + ' ' + ' '.join(str(x) for x in a)
    print(line, flush=True); log.write(line + '\n'); log.flush()


env = dict(os.environ, GATE_SCRIPT=os.environ.get('GATE_SCRIPT', str(R12)), GATE_INPUT_EXCL='fleet_motion_watchdog', GATE_INPUT_MAX_VX='0.20', GATE_INPUT_MAX_WZ='0.60',
           AGENT_EXTRA='-p web_gate_reset:=true', GATE_RESET_PEER='/fleet/motion_watchdog/status')
subprocess.run([str(MODE / 'stack_up.sh'), str(RUN), str(PORT), DOMAIN], check=True, env=env, stdout=(OUT / 'stack_up.log').open('x'), stderr=subprocess.STDOUT)
os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=DOMAIN); os.environ.pop('ROS_DISCOVERY_SERVER', None)
sys.path.insert(0, str(FC / 'agent' / 'limo_robot_agent'))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from limo_robot_agent.core import Http, HttpError

URL = f'http://127.0.0.1:{PORT}'
tok = lambda n: (RUN / 'secrets' / f'{n}.token').read_text().strip()
ADMIN, OPER, VIEWER = Http(URL, tok('admin')), Http(URL, tok('operator')), Http(URL, tok('viewer'))
NOAUTH = Http(URL, 'x' * 32)
for h in (ADMIN, OPER, VIEWER, NOAUTH): h.timeout = 4.0

rclpy.init(); obs = rclpy.create_node('gate_reset_e2e_observer')
locked, gate_last = [], [{}]
obs.create_subscription(Twist, '/cmd_vel_locked', lambda m: locked.append((time.monotonic(), m.linear.x, m.angular.z)), 50)
obs.create_subscription(String, '/stall_detector/status_json', lambda m: gate_last.__setitem__(0, json.loads(m.data)), 10)
ex = SingleThreadedExecutor(); ex.add_node(obs); threading.Thread(target=ex.spin, daemon=True).start()
EPOCH = [None]


def get(path, h=None):
    import urllib.request, urllib.error
    h = h or ADMIN
    req = urllib.request.Request(URL + '/api/v1' + path, headers={'Authorization': 'Bearer ' + h.token})
    try:
        with urllib.request.urlopen(req, timeout=4) as r: return json.loads(r.read())
    except urllib.error.HTTPError as e: raise HttpError(e.code, e.read().decode('utf-8', 'replace'))
def robot(): return next((r for r in get('/robots')['robots'] if r['robot_id'] == RID), {})
def mode(): return (get(f'/robots/{RID}/mode').get('operating_mode') or {})
def gview(h=None): return get(f'/robots/{RID}/gate-reset', h)
def proc(op, name): subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), op, name], check=False); return time.monotonic()


def wait_until(pred, timeout, step=.25):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            v = pred()
            if v: return v
        except Exception: pass
        time.sleep(step)
    return None


def nz_since(t0): return [s for s in locked if s[0] >= t0 and (abs(s[1]) > 1e-6 or abs(s[2]) > 1e-6)]


def record(name, ok, **d):
    assert 'result' not in d and 'scenario' not in d, 'reserved record keys'   # 2026-11-21: a 'result' kwarg once hid the verdict
    results.append({'scenario': name, 'result': 'PASS' if ok else 'FAIL', **d}); say(('PASS ' if ok else 'FAIL ') + name, json.dumps(d, default=str)[:900])


def scenario(fn):
    def run():
        try: fn()
        except Exception as e: record(fn.__name__, False, error=repr(e)[:300], trace=traceback.format_exc()[-900:])
    run.__name__ = fn.__name__; return run


def lease():
    EPOCH[0] = ADMIN.post(f'/robots/{RID}/lock', {'action': 'acquire', 'ttl_s': 300})['epoch']; return EPOCH[0]


def renewer():
    while True:
        try: ADMIN.post(f'/robots/{RID}/lock', {'action': 'acquire', 'ttl_s': 300})
        except Exception: pass
        time.sleep(20)


def switch(target, timeout=45):
    ADMIN.post(f'/robots/{RID}/mode', {'target': target, 'epoch': EPOCH[0], 'idempotency_key': 'k-' + uuid.uuid4().hex})
    return wait_until(lambda: (lambda m: m if m.get('mode') == target and (m.get('transition') or {}).get('state') in ('ACTIVE', None) else None)(mode()), timeout)


def gate(): return gate_last[0]


def gate_writers():
    return len(obs.get_publishers_info_by_topic('/cmd_vel_raw'))


def induce_input_fault(keep=False, secs=1.5):
    """A foreign /cmd_vel_raw writer (this observer node) -> r12 NOT_EXCLUSIVE (sticky) -> FAILSAFE."""
    pub = obs.create_publisher(Twist, '/cmd_vel_raw', 10); m = Twist()
    end = time.monotonic() + secs
    while time.monotonic() < end: pub.publish(m); time.sleep(.05)
    if keep: return pub
    obs.destroy_publisher(pub)
    wait_until(lambda: gate_writers() == 1 and len((gate().get('cmd_input') or {}).get('publishers') or []) == 1, 20)
    return None


def post_reset(h=None, note='e2e', **over):
    v = gview()['gate']['view'] or {}
    body = {'epoch': EPOCH[0], 'expected_gate_boot_id': v.get('boot_id'), 'expected_fault_epoch': v.get('fault_epoch'), 'note': note}
    body.update(over)
    return (h or ADMIN).post(f'/robots/{RID}/gate-reset', body)


def expect_http(fn, status, code=None):
    try: fn()
    except HttpError as e:
        return e.status == status and (code is None or code in str(e)), str(e)[:240]
    return False, 'accepted'


def wait_result(timeout=25):
    return wait_until(lambda: (lambda r: r if r and r['status'] not in ('QUEUED', 'DELIVERED') else None)(gview()['request']), timeout)


def wait_gate_ready(timeout=45):
    return wait_until(lambda: (gview()['gate']['view'] or {}).get('reset_readiness', {}).get('ready') is True and not gview()['server_blockers'], timeout)


def events(kind):
    return [e for e in get('/events?limit=500')['events'] if e.get('kind') == kind]


def clear_if_faulted(note):
    """Recorded admin reset of a fault left by the previous scenario (only when the Gate is latched)."""
    if gate().get('state') == 'NORMAL' and not (gate().get('cmd_input') or {}).get('sticky'): return None
    wait_until(lambda: gate_writers() == 1, 40); wait_gate_ready(40)
    post_reset(note=note); r = wait_result(); wait_until(lambda: gate().get('state') == 'NORMAL', 20); return r


def gate_counters():
    g = gate(); return {'refusals': g.get('reset_refusals'), 'unlatch': g.get('operator_unlatch_count'), 'last_reset': g.get('last_reset')}


NAV2_STARTS = []


def ensure_nav2():
    """Same recorded guard as nav_e2e/mode_e2e: the known lost change_state response at Nav2 bring-up leaves Nav2 inactive, so the
    watchdog can never prove idle (stays LOCKED). Restart the stack group and the test Agent; every attempt is recorded."""
    for attempt in range(3):
        ok = wait_until(lambda: (robot().get('state') or {}).get('motion_watchdog', {}).get('state') == 'MOTION_AUTHORIZED', 45)
        NAV2_STARTS.append({'attempt': attempt, 'watchdog_authorized': bool(ok)})
        if ok: return True
        proc('kill', 'stack'); proc('kill', 'agent'); time.sleep(3); proc('start', 'stack'); time.sleep(6); proc('start', 'agent')
        NAV2_STARTS[-1]['restarted'] = ['stack', 'agent']
    return False


# ------------------------------------------------------------------ scenarios
@scenario
def e00_startup():
    ensure_nav2()
    up = wait_until(lambda: robot().get('online') and robot().get('reconciled') and gate().get('state') == 'NORMAL'
                    and (gate().get('cmd_input') or {}).get('state') == 'BOUND'
                    and ((gview()['gate']['view'] or {}).get('reset_readiness') or {}).get('enforced') is True
                    and not any(b.startswith('WATCHDOG') for b in gview()['server_blockers']), 90)
    lease(); m = mode()
    v = gview()
    rr = (v['gate']['view'] or {}).get('reset_readiness') or {}
    record('e00_startup', bool(up) and m.get('mode') == 'IDLE' and rr.get('enforced') is True
           and 'GATE_NOT_FAULTED' in v['server_blockers'] and v['agent_enabled'],
           mode=m.get('mode'), server_blockers=v['server_blockers'], gate_ready=(v['gate']['view'] or {}).get('reset_readiness'))


@scenario
def e01_normal_recovery_stays_idle():
    induce_input_fault()
    before = gate_counters(); g0 = dict(gate())
    ready = wait_gate_ready(20)
    t0 = time.monotonic(); q = post_reset(note='e01 normal recovery'); res = wait_result()
    normal = wait_until(lambda: gate().get('state') == 'NORMAL' and (gate().get('cmd_input') or {}).get('state') == 'BOUND', 20)
    time.sleep(3)
    moved = nz_since(t0)
    kinds = {k: len(events(k)) for k in ('gate_reset_requested', 'gate_reset_delivered', 'gate_reset_result')}
    record('e01_normal_recovery_stays_idle', bool(ready) and q['status'] == 'QUEUED' and res and res['status'] == 'SUCCEEDED' and bool(normal)
           and mode().get('mode') == 'IDLE' and not moved and all(kinds.values()),
           gate_before=g0.get('state'), faults_before=g0.get('last_fault_reasons'), request=res, gate_after=gate().get('state'),
           mode_after=mode().get('mode'), nonzero_output_after=len(moved), audit=kinds, gate_counters_before=before, gate_counters_after=gate_counters())


@scenario
def e02_roles_and_audit():
    induce_input_fault(); wait_gate_ready(20)
    n0 = len(events('gate_reset_refused'))
    op = expect_http(lambda: post_reset(OPER), 403, 'forbidden')
    vw = expect_http(lambda: post_reset(VIEWER), 403, 'forbidden')
    na = expect_http(lambda: post_reset(NOAUTH), 401)
    n1 = len(events('gate_reset_refused'))
    ui_view_operator = gview(OPER)['server_blockers']
    record('e02_roles_and_audit', op[0] and vw[0] and na[0] and n1 - n0 == 2 and 'ROLE_NOT_ADMIN' in ui_view_operator and gate().get('state') != 'NORMAL',
           operator=op, viewer=vw, no_token=na, refused_events_added=n1 - n0, operator_view_blockers=ui_view_operator,
           note='an invalid token is rejected by authentication before the endpoint (401, not audited as a gate-reset refusal)')
    # leave the Gate faulted for e03


@scenario
def e03_fault_persists_refused_by_agent_and_gate():
    pub = induce_input_fault(keep=True, secs=1.0)          # foreign writer stays
    try:
        time.sleep(1.5); c0 = gate_counters()
        gv = (gview()['gate']['view'] or {}).get('reset_readiness')
        q = post_reset(note='e03 fault persists'); res = wait_result()
        # direct call of the Gate service (no web, no Agent): the Gate itself must refuse
        cli = obs.create_client(Trigger, '/stall_detector/reset_latch'); cli.wait_for_service(timeout_sec=3)
        f = cli.call_async(Trigger.Request()); wait_until(lambda: f.done(), 5); direct = f.result().message if f.done() else None
        obs.destroy_client(cli)
        st = gate().get('state')
    finally:
        obs.destroy_publisher(pub)
    record('e03_fault_persists_refused_by_agent_and_gate', bool(res) and res['status'] == 'REFUSED' and 'gate_ready' in (res.get('failed_checks') or [])
           and direct is not None and direct.startswith('REFUSED') and st != 'NORMAL' and gv and gv.get('ready') is False,
           gate_readiness=gv, request=res, direct_gate_call=direct, gate_state=st, gate_counters_before=c0, gate_counters_after=gate_counters())
    wait_until(lambda: gate_writers() == 1, 20)


@scenario
def e04_watchdog_restart_ghost_then_recovery():
    pre = clear_if_faulted('e04 clear leftover')
    t0 = proc('kill', 'watchdog'); proc('start', 'watchdog')
    # wait until the SERVER sees the Gate faulted by the new writer while the killed one's endpoint still lingers
    w2 = wait_until(lambda: gate_writers() >= 2 and ((gview()['gate']['view'] or {}).get('cmd_input') or {}).get('sticky'), 15)
    early = None
    try:
        early_q = post_reset(note='e04 during ghost'); early = wait_result()
    except HttpError as e:
        early = {'status': 'SERVER_REFUSED', 'reason': str(e)[:200]}
    ghost_s = wait_until(lambda: (gate_writers() == 1 and len((gate().get('cmd_input') or {}).get('publishers') or []) == 1) and round(time.monotonic() - t0, 1), 60)
    ready = wait_gate_ready(40)
    q = post_reset(note='e04 after ghost'); res = wait_result()
    normal = wait_until(lambda: gate().get('state') == 'NORMAL', 20)
    record('e04_watchdog_restart_ghost_then_recovery', bool(w2) and early and early['status'] in ('REFUSED', 'SERVER_REFUSED')
           and res and res['status'] == 'SUCCEEDED' and bool(normal) and mode().get('mode') == 'IDLE',
           during_ghost=early, ghost_gone_after_s=ghost_s, after=res, gate_after=gate().get('state'))


@scenario
def e05_publisher_swap_refused():
    # an impostor using the watchdog's node name while the real watchdog runs, then alone after the real one exits cleanly
    imp = subprocess.Popen([sys.executable, str(HERE / 'impostor_pub.py'), '--ros-args', '-r', '__node:=fleet_motion_watchdog'],
                           stdout=(OUT / 'impostor.log').open('x'), stderr=subprocess.STDOUT, start_new_session=True)
    try:
        wait_until(lambda: gate_writers() >= 2, 15); time.sleep(1.5)
        try: post_reset(note='e05 both'); both = wait_result()
        except HttpError as e: both = {'status': 'SERVER_REFUSED', 'reason': str(e)[:200]}
        proc('kill', 'watchdog')                 # SIGKILL leaves a ghost; wait for a single endpoint (the impostor)
        alone = wait_until(lambda: gate_writers() == 1, 60)
        time.sleep(2)
        gv = (gview()['gate']['view'] or {}).get('reset_readiness')
        try: post_reset(note='e05 impostor alone'); only = wait_result()
        except HttpError as e: only = {'status': 'SERVER_REFUSED', 'reason': str(e)[:200]}
        cli = obs.create_client(Trigger, '/stall_detector/reset_latch'); cli.wait_for_service(timeout_sec=3)   # direct Gate call
        f = cli.call_async(Trigger.Request()); wait_until(lambda: f.done(), 5); direct = f.result().message if f.done() else None
        obs.destroy_client(cli)
        st = gate().get('state')
    finally:
        os.killpg(imp.pid, signal.SIGINT); imp.wait(5)
        proc('start', 'watchdog')
    record('e05_publisher_swap_refused', both and both['status'] in ('REFUSED', 'SERVER_REFUSED') and only and only['status'] in ('REFUSED', 'SERVER_REFUSED')
           and st != 'NORMAL' and gv and gv.get('ready') is False and direct is not None and direct.startswith('REFUSED'),
           both_writers=both, impostor_alone=only, impostor_alone_gate_readiness=gv, direct_gate_call_impostor_alone=direct, gate_state=st,
           note='impostor = separate process using the watchdog node name: refused by the Agent (watchdog self-report) AND by the Gate (peer participant)')
    wait_until(lambda: gate_writers() == 1 and (robot().get('state') or {}).get('motion_watchdog', {}).get('state') == 'MOTION_AUTHORIZED', 60)


def watchdog_stable(secs=3.0, timeout=60):
    """The watchdog must have been MOTION_AUTHORIZED continuously for `secs` (the Agent observes 1.5 s before calling the Gate)."""
    since = [None]
    def ok():
        auth = (robot().get('state') or {}).get('motion_watchdog', {}).get('state') == 'MOTION_AUTHORIZED'
        since[0] = (since[0] or time.monotonic()) if auth else None
        return since[0] is not None and time.monotonic() - since[0] >= secs
    return wait_until(ok, timeout, .2)


@scenario
def e06_concurrent_requests():
    if gate().get('state') == 'NORMAL': induce_input_fault()
    watchdog_stable(); wait_gate_ready(60)
    out = []; barrier = threading.Barrier(6)
    def go():
        barrier.wait()
        try: out.append(post_reset(note='e06')['status'])
        except HttpError as e: out.append(str(e)[:60])
    ts = [threading.Thread(target=go) for _ in range(6)]; [t.start() for t in ts]; [t.join() for t in ts]
    res = wait_result(); normal = wait_until(lambda: gate().get('state') == 'NORMAL', 20)
    queued = out.count('QUEUED'); busy = sum('gate_reset_in_progress' in x or 'GATE_CHANGED' in x for x in out)
    record('e06_concurrent_requests', queued == 1 and busy == 5 and res and res['status'] == 'SUCCEEDED' and bool(normal), responses=out, request=res)


@scenario
def e07_server_restart_with_pending_request():
    induce_input_fault(); wait_gate_ready(20); c0 = gate_counters()
    proc('stop', 'agent')                                   # no delivery while the request is pending
    q = post_reset(note='e07 pending over server restart')
    proc('kill', 'server'); time.sleep(1); proc('start', 'server')
    wait_until(lambda: get('/robots') is not None, 20)
    proc('cont', 'agent')
    back = wait_until(lambda: robot().get('online') and robot().get('reconciled'), 60)
    time.sleep(3)
    r = gview()['request']; c1 = gate_counters()
    record('e07_server_restart_with_pending_request', q['status'] == 'QUEUED' and bool(back) and r and r['status'] == 'ABANDONED_SERVER_RESTART'
           and c1['unlatch'] == c0['unlatch'] and gate().get('state') != 'NORMAL', queued=q, after=r, gate_counters_before=c0, gate_counters_after=c1)


@scenario
def e08_agent_restart_with_pending_request():
    lease(); wait_until(lambda: mode().get('mode') == 'IDLE', 30)
    wait_gate_ready(30); c0 = gate_counters()
    proc('stop', 'agent'); q = post_reset(note='e08 pending over agent restart')
    proc('kill', 'agent'); time.sleep(1); proc('start', 'agent')
    back = wait_until(lambda: robot().get('online') and robot().get('reconciled'), 60)
    time.sleep(3)
    r = gview()['request']; c1 = gate_counters()
    record('e08_agent_restart_with_pending_request', q['status'] == 'QUEUED' and bool(back) and r and r['status'] == 'ABANDONED_AGENT_RESTART'
           and c1['unlatch'] == c0['unlatch'], queued=q, after=r, gate_counters_before=c0, gate_counters_after=c1)


@scenario
def e09_replay_of_old_request_values_refused():
    if gate().get('state') == 'NORMAL': induce_input_fault()
    wait_gate_ready(40)
    v = gview()['gate']['view']; old = {'expected_gate_boot_id': v['boot_id'], 'expected_fault_epoch': v['fault_epoch']}
    post_reset(note='e09 first'); first = wait_result(); wait_until(lambda: gate().get('state') == 'NORMAL', 20)
    again = expect_http(lambda: ADMIN.post(f'/robots/{RID}/gate-reset', {'epoch': EPOCH[0], 'note': 'e09 replay', **old}), 409)
    induce_input_fault()                                    # a new fault: the old body still names the old fault epoch
    stale = expect_http(lambda: ADMIN.post(f'/robots/{RID}/gate-reset', {'epoch': EPOCH[0], 'note': 'e09 stale', **old}), 409, 'GATE_CHANGED_SINCE_VIEWED')
    record('e09_replay_of_old_request_values_refused', first and first['status'] == 'SUCCEEDED' and again[0] and stale[0],
           first=first, replay_after_success=again, stale_after_new_fault=stale, fault_epoch_old=old['expected_fault_epoch'],
           fault_epoch_now=(gview()['gate']['view'] or {}).get('fault_epoch'))


@scenario
def e10_no_auto_resume_and_new_admission():
    clear_if_faulted('e10 clear leftover')
    switch('MANUAL')
    sess = ADMIN.post(f'/robots/{RID}/manual-bench', {'action': 'start', 'epoch': EPOCH[0]})['session_id']; seq = [0]; on = [True]
    def drive():
        while on[0]:
            seq[0] += 1
            try: ADMIN.post(f'/robots/{RID}/manual-bench', {'action': 'sample', 'epoch': EPOCH[0], 'session_id': sess, 'seq': seq[0], 'linear': 1., 'turn': .2, 'deadman': True})
            except Exception: pass
            time.sleep(.08)
    th = threading.Thread(target=drive, daemon=True); th.start(); time.sleep(1.2)
    t_drive = time.monotonic(); drove_before = bool(nz_since(t_drive - 1.2))
    induce_input_fault(secs=1.0)                            # fault while driving
    on[0] = False; th.join(2)
    switch('IDLE', 60)
    blocked_manual = gview()['server_blockers']
    wait_gate_ready(40); t0 = time.monotonic(); post_reset(note='e10 after manual'); res = wait_result()
    wait_until(lambda: gate().get('state') == 'NORMAL', 20); time.sleep(2)
    old = expect_http(lambda: ADMIN.post(f'/robots/{RID}/manual-bench', {'action': 'sample', 'epoch': EPOCH[0], 'session_id': sess, 'seq': seq[0] + 1,
                                                                         'linear': 1., 'turn': 0., 'deadman': True}), 409)
    still = not nz_since(t0); m_after = mode().get('mode')
    mission = expect_http(lambda: ADMIN.post('/missions', {'robot_id': RID, 'type': 'navigate_to_pose', 'goal': {'frame_id': 'map', 'x': 1, 'y': 0},
                                                           'lease_epoch': EPOCH[0], 'idempotency_key': 'e10-' + uuid.uuid4().hex}), 409)
    switch('MANUAL'); s2 = ADMIN.post(f'/robots/{RID}/manual-bench', {'action': 'start', 'epoch': EPOCH[0]})['session_id']
    t1 = time.monotonic()
    for i in range(1, 16):
        try: ADMIN.post(f'/robots/{RID}/manual-bench', {'action': 'sample', 'epoch': EPOCH[0], 'session_id': s2, 'seq': i, 'linear': 1., 'turn': 0., 'deadman': True})
        except Exception: pass
        time.sleep(.08)
    moved_new = bool(nz_since(t1))
    try: ADMIN.post(f'/robots/{RID}/manual-bench', {'action': 'stop', 'epoch': EPOCH[0], 'session_id': s2})
    except Exception: pass
    switch('IDLE')
    record('e10_no_auto_resume_and_new_admission', drove_before and res and res['status'] == 'SUCCEEDED' and old[0] and still and m_after == 'IDLE'
           and mission[0] and moved_new, drove_before_fault=drove_before, request=res, old_session_sample=old, zero_after_reset=still,
           mode_after_reset=m_after, mission_in_idle=mission, new_manual_session_moves=moved_new, blockers_while_manual=blocked_manual)


@scenario
def e11_browser_panel():
    if not BROWSER:
        results.append({'scenario': 'e11_browser_panel', 'result': 'SKIP', 'reason': '--browser not given'}); return
    induce_input_fault(); wait_gate_ready(20)
    envb = dict(os.environ, NODE_PATH=os.environ.get('GATE_RESET_E2E_NODE_PATH', ''))
    out = {}
    for role in ('operator', 'admin'):
        p = subprocess.run(['node', str(HERE / 'browser_gate_reset.cjs'), URL, str(RUN / 'secrets' / f'{role}.token'), str(OUT), role, '1' if role == 'admin' else '0'],
                           capture_output=True, text=True, env=envb, timeout=90)
        (OUT / f'browser_{role}.log').write_text(p.stdout + p.stderr)
        try: out[role] = json.loads([l for l in p.stdout.splitlines() if l.startswith('{')][-1])
        except Exception: out[role] = {'rc': p.returncode}
    res = wait_result(); normal = wait_until(lambda: gate().get('state') == 'NORMAL', 20)
    a, o = out.get('admin') or {}, out.get('operator') or {}
    record('e11_browser_panel', o.get('disabled') is True and 'ROLE_NOT_ADMIN' not in json.dumps(o) and '관리자 권한 필요' in (o.get('text') or '')
           and a.get('clicked') is True and res and res['status'] == 'SUCCEEDED' and bool(normal), operator=o, admin=a, request=res)


ORDER = [e00_startup, e01_normal_recovery_stays_idle, e02_roles_and_audit, e03_fault_persists_refused_by_agent_and_gate,
         e04_watchdog_restart_ghost_then_recovery, e05_publisher_swap_refused, e06_concurrent_requests, e07_server_restart_with_pending_request,
         e08_agent_restart_with_pending_request, e09_replay_of_old_request_values_refused, e10_no_auto_resume_and_new_admission, e11_browser_panel]

try:
    threading.Thread(target=renewer, daemon=True).start()
    only = os.environ.get('GATE_RESET_E2E_ONLY')
    for fn in ORDER:
        if only and fn.__name__ not in only.split(','): continue
        say('--- ' + fn.__name__); fn()
    motor = {'publishers': obs.count_publishers('/cmd_vel'), 'subscribers': obs.count_subscribers('/cmd_vel')}
    record('no_cmd_vel_endpoints', motor == {'publishers': 0, 'subscribers': 0}, **motor)
finally:
    try: events_all = get('/events?limit=500')['events']
    except Exception: events_all = []
    proc('stopall', 'x')
    summary = {'port': PORT, 'domain': DOMAIN, 'gate': str(R12.relative_to(REPO)), 'nav2_bringup_attempts': NAV2_STARTS, 'results': results,
               'counts': {k: sum(r['result'] == k for r in results) for k in ('PASS', 'FAIL', 'SKIP')},
               'gate_reset_events': [e for e in events_all if str(e.get('kind', '')).startswith('gate_reset')],
               'finished': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    (OUT / 'result.json').write_text(json.dumps(summary, indent=1, default=str))
    say('SUMMARY', summary['counts']); log.flush()
    os._exit(0 if summary['counts']['FAIL'] == 0 else 1)
