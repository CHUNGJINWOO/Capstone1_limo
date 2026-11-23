"""Phase 4 isolated E2E (2026-11-21): SSH-free start through the robot supervisor, readiness, failures, no stale reuse.

Stack: tests/mode_e2e/stack_up.sh NO_START=1 (kinematic sim + ICP + Nav2 + Gate r12 + Motion Watchdog + Agent, locked output
/cmd_vel_locked, no /cmd_vel, ROS_LOCALHOST_ONLY=1, private domain). The Fleet Server runs with --require-supervisor-ready and is
NOT supervised (central server); sim / stack / watchdog / agent are started ONLY by limo_robot_agent.supervisor from a local
config this harness writes (the same command lines stack_up.sh would use).
usage: OVERLAY=<explore_lite overlay> P4_OUT=<new dir> python3 run_phase4.py <port> <domain> [--order server_first|supervisor_first]
       [--sim-delay S] [--browser]
"""
import json, math, os, signal, subprocess, sys, threading, time, traceback, uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent; FC = HERE.parents[1]; REPO = FC.parent; MODE = FC / 'tests' / 'mode_e2e'
sys.path[:0] = [str(FC / 'tests'), str(MODE)]
from e2e_common import Fleet, HttpError, Results, wait_until, expect_http, PROTECTED_PORTS   # noqa: E402

PORT, DOMAIN = int(sys.argv[1]), sys.argv[2]
arg = lambda k, d=None: sys.argv[sys.argv.index(k) + 1] if k in sys.argv else d
ORDER, SIM_DELAY, BROWSER = arg('--order', 'server_first'), float(arg('--sim-delay', '0')), '--browser' in sys.argv
assert PORT not in PROTECTED_PORTS and DOMAIN not in ('0', '77', '231')
OUT = Path(os.environ['P4_OUT']).resolve(); OUT.mkdir(parents=True, exist_ok=False)
RUN = OUT / 'stack'; R = Results(OUT / 'harness.log'); say = R.say
R12 = REPO / 'experiments/safety_chain_fix_20261113/candidate_gate_r12/nodes/stall_detector_node.py'
env = dict(os.environ, NO_START='1', NAV_LOCALIZATION='1', GATE_SCRIPT=os.environ.get('GATE_SCRIPT', str(R12)), GATE_INPUT_EXCL='fleet_motion_watchdog',
           GATE_INPUT_MAX_VX='0.20', GATE_INPUT_MAX_WZ='0.60', SERVER_EXTRA='--require-supervisor-ready',
           AGENT_EXTRA='-p web_gate_reset:=true', GATE_RESET_PEER='/fleet/motion_watchdog/status')   # as tests/gate_reset_e2e
subprocess.run([str(MODE / 'stack_up.sh'), str(RUN), str(PORT), DOMAIN], check=True, env=env, stdout=(OUT / 'stack_up.log').open('x'), stderr=subprocess.STDOUT)
F = Fleet(PORT, RUN / 'secrets'); OPER = lambda: F; ADMIN_GET = lambda path: F.get(path, who='admin')

# ------------------------------------------------------------------ supervisor config: the robot's local root of trust
def svc(name, after=(), ready_topics=(), after_s=2.0, delay=0.0, actions=('start', 'stop', 'restart', 'retry'), restart_with=()):
    return {'name': name, 'cmd': f'source {RUN}/env.sh; source {RUN}/cmds.sh; eval "$CMD_{name}"', 'after': list(after), 'restart_with': list(restart_with),
            'ready': {'after_s': after_s, 'topics': list(ready_topics)}, 'restart': {'max': 3, 'window_s': 120, 'backoff_s': 2},
            'start_delay_s': delay, 'actions': list(actions)}


SUP_CFG = {'robot_id': 'iso_bench', 'server_url': f'http://127.0.0.1:{PORT}', 'token_file': str(RUN / 'secrets' / 'agent-iso_bench.token'),
           'report_period_s': 1.0,
           'services': [svc('sim', ready_topics=['/scan', '/odom'], delay=SIM_DELAY),
                        svc('stack', ['sim'], ['/stall_detector/status_json'], after_s=8.0),
                        svc('watchdog', ['stack'], ['/fleet/motion_watchdog/status'], restart_with=['stack']),
                        svc('agent', ['watchdog'], after_s=4.0, restart_with=['stack'])],
           'ros_probe': {'domain_id': int(DOMAIN), 'localhost_only': True, 'topics': {
               '/scan': {'type': 'sensor_msgs/msg/LaserScan', 'max_age_s': 1.0},
               '/odom': {'type': 'nav_msgs/msg/Odometry', 'max_age_s': 1.0},
               '/stall_detector/status_json': {'type': 'std_msgs/msg/String', 'max_age_s': 1.0, 'json_keys': ['state'], 'sensor_qos': False},
               '/fleet/motion_watchdog/status': {'type': 'std_msgs/msg/String', 'max_age_s': 1.5, 'json_keys': ['state'], 'sensor_qos': False}}}}
(RUN / 'supervisor.json').write_text(json.dumps(SUP_CFG, indent=1))
PROCS = {}


def start(name, argv):
    log = (OUT / f'{name}.{int(time.time())}.log').open('x')
    PROCS[name] = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, cwd=str(FC / 'server'))
    return PROCS[name]


def stop(name, sig=signal.SIGINT, wait=20):
    p = PROCS.get(name)
    if p and p.poll() is None:
        os.killpg(p.pid, sig)
        try: p.wait(wait)
        except subprocess.TimeoutExpired: os.killpg(p.pid, signal.SIGKILL); p.wait(5)


def start_server():
    return start('server', ['bash', '-c', f'source {RUN}/env.sh; source {RUN}/cmds.sh; eval "$CMD_server"'])


def start_supervisor():
    return start('supervisor', ['bash', '-c', f'source {RUN}/env.sh; exec python3 -m limo_robot_agent.supervisor --config {RUN}/supervisor.json --log-dir {RUN}/supervisor_logs'])


def ready():
    try: return F.get('/robots/iso_bench/readiness')
    except Exception as e: return {'level': 'NO_SERVER', 'reasons': [repr(e)[:80]]}


def level(): return ready().get('level')


def svc_pid(name):
    return next((s.get('pid') for s in ready().get('services', []) if s['name'] == name), None)


def svc_state(name):
    return next((s.get('state') for s in ready().get('services', []) if s['name'] == name), None)


def lease():
    return F.lease(ttl=300, who='admin')       # one principal: the Gate reset (admin-only) needs the lease holder


def mode_req(target, epoch, who='admin'):
    return F.post('/robots/iso_bench/mode', {'target': target, 'epoch': epoch, 'idempotency_key': 'p4-' + uuid.uuid4().hex[:10]}, who=who)


def gate_reset_if_latched(label):
    """Drive the robot back to READY with the operator recoveries only (recorded): the admin web Gate reset for a latched Gate (the
    known killed-writer DDS ghost after a Watchdog restart) and the admin recovery for a confirmed Gate / Agent boot change.
    Keeps checking for up to 150 s because a latch can appear only after the restarted process is back."""
    out = []
    end = time.monotonic() + 150
    while time.monotonic() < end and level() != 'READY':
        reasons = ready().get('reasons', [])
        try:
            if any(r.startswith('GATE_FAULT') for r in reasons):
                v = F.get('/robots/iso_bench/gate-reset', who='admin')['gate']['view']
                body = {'epoch': lease(), 'expected_gate_boot_id': v.get('boot_id'), 'expected_fault_epoch': v.get('fault_epoch'), 'note': f'phase4 {label}'}
                out.append(('gate_reset', F.post('/robots/iso_bench/gate-reset', body, who='admin').get('status')))
            elif 'GATE_BOOT_RECONCILIATION' in reasons or 'RECONCILIATION_REQUIRED' in reasons:
                out.append(('recover', admin_recover(label)))
        except HttpError as e: out.append(('refused', str(e)[60:150]))
        time.sleep(3)
    return out[-8:]


def admin_recover(label):
    r = F.get('/robots', who='admin')['robots'][0]
    if r.get('gate_boot_reconciliation') or r.get('reconciliation_required'):
        try: F.post('/robots/iso_bench/recover', {'note': f'phase4 {label}'}); return 'recovered'
        except HttpError as e: return str(e)[:120]
    return None


def scenario(fn):
    def run():
        say('---', fn.__name__)
        try: fn()
        except Exception as e:
            R.record(fn.__name__, False, error=repr(e)[:300], readiness=ready(), trace=traceback.format_exc()[-600:])
    run.__name__ = fn.__name__; return run


def graph_motor_lock():
    """ROS graph: /cmd_vel has no endpoint; the only /cmd_vel_locked writer is the Gate (checked with ros2 CLI in the run env)."""
    out = subprocess.run(['bash', '-c', f'source {RUN}/env.sh; ros2 topic info /cmd_vel 2>&1; ros2 topic info /cmd_vel_locked -v 2>&1 | sed -n "/Publisher count/,/Subscription count/p" | grep -E "Node name|Publisher count"'],
                         capture_output=True, text=True, timeout=60).stdout
    return out


# ================================================================== scenarios
@scenario
def s01_boot_to_ready_and_gate_before_ready():
    t0 = time.monotonic(); seen = []
    if ORDER == 'server_first':
        start_server(); assert wait_until(lambda: F.get('/robots') is not None, 30), 'server not up'
        start_supervisor()
    else:
        start_supervisor(); time.sleep(25)                              # robot side boots with no server reachable
        start_server()
    pre = None
    def poll():
        lv = level(); seen.append((round(time.monotonic() - t0, 1), lv, tuple(ready().get('reasons', [])[:3])))
        nonlocal pre
        # try MAPPING in every poll while the robot is registered but not READY (before registration there is no robot at all)
        if (pre is None or not pre[0]) and lv not in ('READY', 'NO_SERVER') and 'UNKNOWN_ROBOT' not in ready().get('reasons', []):
            try:
                ep = lease(); pre = expect_http(lambda: mode_req('MAPPING', ep), 409, 'robot_not_ready')
            except Exception as ex: pre = (False, repr(ex)[:120])
        return lv == 'READY'
    ok = wait_until(poll, 240, 1.0)
    trail = []
    for s in seen:
        if not trail or trail[-1][1:] != s[1:]: trail.append(s)
    rd = ready()
    R.record('s01_boot_to_ready_and_gate_before_ready', bool(ok) and bool(pre and pre[0]), order=ORDER, sim_delay_s=SIM_DELAY,
             ready_after_s=round(time.monotonic() - t0, 1) if ok else None, mapping_before_ready=pre, readiness_trail=trail[:25],
             services=[(s['name'], s['state'], s.get('restarts')) for s in rd.get('services', [])])


@scenario
def s02_ready_allows_mode_and_motor_lock_kept():
    ep = lease()
    m = F.switch('MANUAL', ep, 60); F.switch('IDLE', ep, 60)
    g = graph_motor_lock()
    no_cmd_vel = "Unknown topic '/cmd_vel'" in g or ('Publisher count: 0' in g and 'Subscription count: 0' in g)
    locked_writers = [l.split(':', 1)[1].strip() for l in g.splitlines() if l.startswith('Node name:')]
    R.record('s02_ready_allows_mode_and_motor_lock_kept', bool(m) and no_cmd_vel and 'stall_detector_node' in locked_writers,
             no_cmd_vel_endpoint=no_cmd_vel,
             manual=bool(m), cmd_vel_graph=g[-600:])


@scenario
def s03_watchdog_killed_restarted_by_supervisor():
    pid = svc_pid('watchdog'); assert pid
    t0 = time.monotonic(); os.killpg(pid, signal.SIGKILL)
    trail = []
    def poll():
        rd = ready(); trail.append((round(time.monotonic() - t0, 1), rd['level'], svc_state('watchdog'), tuple(rd['reasons'][:3])))
        return svc_state('watchdog') == 'READY' and svc_pid('watchdog') not in (None, pid)
    back = wait_until(poll, 90, 0.5)
    resets = gate_reset_if_latched('s03')
    final = wait_until(lambda: level() == 'READY', 120, 1.0)
    compact = []
    for t in trail:
        if not compact or compact[-1][1:] != t[1:]: compact.append(t)
    R.record('s03_watchdog_killed_restarted_by_supervisor', bool(back) and bool(final), restarted_after_s=round(time.monotonic() - t0, 1),
             trail=compact[:20], gate_resets=resets, final=ready()['level'], events=events_since(t0, ('service_state',)))


EV_T0 = time.time()


def events_since(t_mono, kinds=None):
    t_wall = time.time() - (time.monotonic() - t_mono)
    ev = F.get('/events?limit=300')['events']
    return [(round(e['ts'] - t_wall, 1), e['kind'], e['message'][:90]) for e in ev if e['ts'] >= t_wall and (kinds is None or e['kind'] in kinds)][:20]


@scenario
def s04_agent_killed_no_stale_session_reuse():
    ep = lease(); assert F.switch('MANUAL', ep, 60), 'MANUAL not reached'
    sess = F.post('/robots/iso_bench/manual-bench', {'action': 'start', 'epoch': ep})['session_id']
    pid = svc_pid('agent'); boot0 = (F.robot().get('agent') or {}).get('boot_id'); t0 = time.monotonic(); os.killpg(pid, signal.SIGKILL)
    # the supervisor may restart the Agent faster than the server's offline threshold (3 s): the new boot id is the evidence
    offline = wait_until(lambda: (F.robot().get('agent') or {}).get('boot_id') not in (None, boot0) or 'AGENT_OFFLINE' in ready().get('reasons', []), 60, 0.5)
    back = wait_until(lambda: svc_state('agent') == 'READY' and F.robot().get('online'), 90, 1.0)
    time.sleep(3)
    replay = expect_http(lambda: F.post('/robots/iso_bench/manual-bench', {'action': 'sample', 'input_format': 'normalized_axes_v1', 'epoch': ep, 'session_id': sess, 'seq': 99,
                                                                          'linear': 1.0, 'turn': 0.0, 'deadman': True}), 409)
    mode_after = F.mode().get('mode')
    rec = admin_recover('s04')
    R.record('s04_agent_killed_no_stale_session_reuse', bool(offline) and bool(back) and replay[0] and mode_after != 'MANUAL',
             new_agent_boot_seen=bool(offline), back_after_s=round(time.monotonic() - t0, 1), old_session_replay=replay, mode_after=mode_after,
             admin_recover=rec, events=events_since(t0, ('service_state', 'robot_offline', 'robot_online', 'mode')))


@scenario
def s05_server_killed_restarted():
    """Server dies while MANUAL with an open bench session: after restart the mode, the manual session and the motion authority
    are gone (the lease is persisted ownership by design and stays; a NEW explicit mode request is needed before any motion)."""
    rec = admin_recover('s05_pre'); gate_reset_if_latched('s05_pre')
    ep = lease(); assert F.switch('MANUAL', ep, 90), 'MANUAL not reached'
    sess = F.post('/robots/iso_bench/manual-bench', {'action': 'start', 'epoch': ep})['session_id']
    t0 = time.monotonic(); stop('server', signal.SIGKILL); time.sleep(8)
    sup_alive = PROCS['supervisor'].poll() is None
    start_server(); up = wait_until(lambda: ready().get('level') != 'NO_SERVER', 30, 0.5)
    rd = wait_until(lambda: level() == 'READY' or None, 120, 1.0)
    mode_after = F.mode().get('mode')
    stale = expect_http(lambda: F.post('/robots/iso_bench/manual-bench', {'action': 'sample', 'input_format': 'normalized_axes_v1', 'epoch': ep, 'session_id': sess, 'seq': 50,
                                                                          'linear': 1.0, 'turn': 0.0, 'deadman': True}), 409)
    arb = (F.robot().get('state') or {}).get('velocity_arbiter') or {}
    R.record('s05_server_killed_restarted', sup_alive and bool(up) and bool(rd) and mode_after != 'MANUAL' and stale[0] and arb.get('source') in (None, 'NONE'),
             supervisor_kept_running=sup_alive, ready_after_restart=bool(rd), mode_after=mode_after, old_session_sample=stale,
             arbiter_source=arb.get('source'), lease_after=F.get('/robots/iso_bench/lock').get('lease'))


@scenario
def s06_service_actions_roles_and_busy():
    adm = lambda b: F.post('/robots/iso_bench/services', b, who='admin')
    op = expect_http(lambda: F.post('/robots/iso_bench/services', {'service': 'watchdog', 'action': 'restart'}, who='operator'), 403)
    vw = expect_http(lambda: F.post('/robots/iso_bench/services', {'service': 'watchdog', 'action': 'restart'}, who='viewer'), 403)
    sh = expect_http(lambda: adm({'service': 'sim', 'action': 'exec', 'cmd': 'id'}), 400)
    ep = lease()
    nolease = expect_http(lambda: adm({'service': 'watchdog', 'action': 'restart'}), 400)                     # no epoch -> refused (Codex 2026-11-21)
    adm0 = adm; adm = lambda b: adm0(dict(b, epoch=ep))
    unk = expect_http(lambda: adm({'service': 'bash', 'action': 'start'}), 404)
    assert F.switch('MANUAL', ep, 90), 'MANUAL not reached'
    busy = expect_http(lambda: adm({'service': 'stack', 'action': 'restart'}), 409, 'robot_busy')
    assert F.switch('IDLE', ep, 90), 'IDLE not reached'
    t0 = time.monotonic(); a = adm({'service': 'watchdog', 'action': 'restart'})
    done = wait_until(lambda: (lambda x: x if x and x['status'] in ('DONE', 'FAILED') else None)(
        next((y for y in ready().get('actions', []) if y['action_id'] == a['action_id']), None)), 30, 0.5)
    back = wait_until(lambda: svc_state('watchdog') == 'READY', 90, 1.0)
    resets = gate_reset_if_latched('s06'); final = wait_until(lambda: level() == 'READY', 120, 1.0)
    R.record('s06_service_actions_roles_and_busy', nolease[0] and op[0] and vw[0] and sh[0] and unk[0] and busy[0] and bool(done) and done['status'] == 'DONE' and bool(back),
             operator=op, viewer=vw, no_epoch=nolease, shell_like=sh, unknown=unk, busy=busy, action=done, gate_resets=resets, final=ready()['level'],
             events=events_since(t0, ('service_action', 'service_action_result', 'service_state')))


@scenario
def s07_crash_loop_failed_then_admin_retry():
    """Stop the Gate stack by repeated SIGKILL beyond the restart budget -> FAILED -> FAULT; admin retry brings it back."""
    ep0 = lease(); F.switch('IDLE', ep0, 90)
    t0 = time.monotonic(); n = 0
    while svc_state('stack') != 'FAILED' and time.monotonic() - t0 < 200:
        pid = svc_pid('stack')
        if pid and svc_state('stack') in ('RUNNING', 'READY'):
            try: os.killpg(pid, signal.SIGKILL); n += 1
            except ProcessLookupError: pass
        time.sleep(3)
    failed = svc_state('stack') == 'FAILED'; rd = ready()
    ep = lease(); refused = expect_http(lambda: mode_req('NAVIGATION', ep), 409, 'robot_not_ready')
    a = F.post('/robots/iso_bench/services', {'service': 'stack', 'action': 'retry', 'epoch': ep}, who='admin')
    back = wait_until(lambda: all(svc_state(n) == 'READY' for n in ('stack', 'watchdog', 'agent')), 240, 1.0)   # cascade restarts Watchdog + Agent
    recov = None
    for _ in range(3):
        resets = gate_reset_if_latched('s07'); recov = admin_recover('s07')
        if wait_until(lambda: level() == 'READY', 60, 1.0): break
    R.record('s07_crash_loop_failed_then_admin_retry', failed and 'SERVICE_FAILED:stack' in rd['reasons'] and refused[0] and bool(back) and ready()['level'] == 'READY',
             kills=n, level_when_failed=rd['level'], reasons=rd['reasons'][:5], navigation_while_failed=refused, retry=a['status'],
             recovered=recov, final=ready()['level'], final_reasons=ready()['reasons'][:5])


@scenario
def s08_operator_web_readiness_card():
    if not BROWSER: R.skip('s08_operator_web_readiness_card', '--browser not given'); return
    p = subprocess.run(['node', str(HERE / 'browser_system_panel.cjs'), f'http://127.0.0.1:{PORT}/', str(RUN / 'secrets' / 'admin.token'), str(OUT)],
                       capture_output=True, text=True, env=dict(os.environ, NODE_PATH=os.environ.get('MANUAL_E2E_NODE_PATH', '')), timeout=120)
    (OUT / 'browser_system_panel.log').write_text(p.stdout + p.stderr[-3000:])
    res = json.loads(p.stdout.strip().splitlines()[-1]) if p.returncode == 0 and p.stdout.strip() else {'error': p.stderr[-300:]}
    R.record('s08_operator_web_readiness_card', res.get('level') == ready()['level'] and res.get('services', 0) >= 4 and res.get('reload_same') is True,
             browser=res, server_level=ready()['level'])


def main():
    try:
        for fn in (s01_boot_to_ready_and_gate_before_ready, s02_ready_allows_mode_and_motor_lock_kept, s08_operator_web_readiness_card,
                   s03_watchdog_killed_restarted_by_supervisor, s04_agent_killed_no_stale_session_reuse, s05_server_killed_restarted,
                   s06_service_actions_roles_and_busy, s07_crash_loop_failed_then_admin_retry):
            if level() == 'NO_SERVER' and fn.__name__ != 's01_boot_to_ready_and_gate_before_ready':
                R.skip(fn.__name__, 'server not running'); continue
            fn()
    finally:
        try: (OUT / 'readiness_final.json').write_text(json.dumps(ready(), indent=1, default=str))
        except Exception: pass
        stop('supervisor', signal.SIGINT, 60); stop('server')
        (OUT / 'result.json').write_text(json.dumps({'summary': R.counts(), 'results': R.items, 'order': ORDER, 'sim_delay_s': SIM_DELAY}, indent=1, default=str))
        say('SUMMARY', R.counts())


if __name__ == '__main__':
    main()
