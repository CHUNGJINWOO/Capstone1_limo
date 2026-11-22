"""Phase 5 isolated E2E (2026-11-21): visitor guide web -> Fleet missions -> Agent -> Nav2 -> Gate -> kinematic sim.

Stack: tests/mode_e2e/stack_up.sh (NAV_LOCALIZATION=1, Gate r12, locked output, ROS_LOCALHOST_ONLY=1, private domain).
Map: the fixed Phase 3 bundle (local_data/raw_results/phase3_stability_claude_20261121/fixed_map), registered and VERIFIED in THIS
run's isolated registry only; C103 is registered UNVERIFIED by stack_up (never verified here).
usage: OVERLAY=<overlay> MANUAL_E2E_NODE_PATH=<playwright node_modules> P5_OUT=<new dir> python3 run_phase5.py <port> <domain>
"""
import json, math, os, subprocess, sys, threading, time, traceback, uuid, urllib.request, urllib.error
from pathlib import Path

HERE = Path(__file__).resolve().parent; FC = HERE.parents[1]; REPO = FC.parent; MODE = FC / 'tests' / 'mode_e2e'
sys.path[:0] = [str(FC / 'tests'), str(MODE)]
from e2e_common import Fleet, HttpError, Results, wait_until, expect_http, PROTECTED_PORTS   # noqa: E402

PORT, DOMAIN = int(sys.argv[1]), sys.argv[2]
assert PORT not in PROTECTED_PORTS and DOMAIN not in ('0', '77', '231')
OUT = Path(os.environ['P5_OUT']).resolve(); OUT.mkdir(parents=True, exist_ok=False)
RUN = OUT / 'stack'; R = Results(OUT / 'harness.log'); say = R.say
R12 = REPO / 'experiments/safety_chain_fix_20261113/candidate_gate_r12/nodes/stall_detector_node.py'
FIXED = REPO / 'local_data/raw_results/phase3_stability_claude_20261121/fixed_map'
subprocess.run([str(MODE / 'stack_up.sh'), str(RUN), str(PORT), DOMAIN], check=True, stdout=(OUT / 'stack_up.log').open('x'), stderr=subprocess.STDOUT,
               env=dict(os.environ, NAV_LOCALIZATION='1', GATE_SCRIPT=os.environ.get('GATE_SCRIPT', str(R12)), GATE_INPUT_EXCL='fleet_motion_watchdog', GATE_INPUT_MAX_VX='0.20', GATE_INPUT_MAX_WZ='0.60'))
os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=DOMAIN); os.environ.pop('ROS_DISCOVERY_SERVER', None)
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
rclpy.init(); obs = rclpy.create_node('phase5_observer')
locked, truth, watch = [], [None], [None]
obs.create_subscription(Twist, '/cmd_vel_locked', lambda m: locked.append((time.monotonic(), m.linear.x, m.angular.z)), 50)
obs.create_subscription(Float64MultiArray, '/e2e/ground_truth', lambda m: truth.__setitem__(0, (time.monotonic(), *m.data)), 10)
obs.create_subscription(String, '/fleet/motion_watchdog/status', lambda m: watch.__setitem__(0, json.loads(m.data)), 10)
ex = SingleThreadedExecutor(); ex.add_node(obs); SPIN = threading.Thread(target=ex.spin, daemon=True); SPIN.start()

F = Fleet(PORT, RUN / 'secrets'); BASE = f'http://127.0.0.1:{PORT}'
pose = lambda: list(truth[0][1:4]) if truth[0] else None
EP = [None]


def lease(): EP[0] = F.lease(ttl=300, who='operator'); return EP[0]


def renew():
    while True:
        time.sleep(20)
        try: F.post('/robots/iso_bench/lock', {'action': 'renew', 'ttl_s': 300}, who='operator')
        except Exception: pass


threading.Thread(target=renew, daemon=True).start()


def switch(target): return F.switch(target, EP[0], 90, who='operator')


def loc(): return (F.state().get('localization') or {})


def load_localize(mvid):
    for attempt in range(4):
        F.post('/robots/iso_bench/navigation', {'action': 'LOAD', 'epoch': EP[0], 'map_version_id': mvid}, who='operator')
        if wait_until(lambda: loc().get('status') in ('WAIT_INITIAL_POSE', 'FAILED') and loc().get('status'), 45) == 'WAIT_INITIAL_POSE': break
        time.sleep(10)
    p = pose(); F.post('/robots/iso_bench/navigation', {'action': 'INITIAL_POSE', 'epoch': EP[0], 'x': p[0], 'y': p[1], 'yaw': p[2]}, who='operator')
    return wait_until(lambda: loc().get('status') == 'CONFIRMED', 60)


def visitor_call(method, path, token, body=None, session='p5probe' + 'x' * 16):
    req = urllib.request.Request(BASE + '/api/v1' + path, method=method, data=None if body is None else json.dumps(body).encode(),
                                 headers={'Authorization': token, 'X-Visitor-Session': session, 'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=10) as r: return r.status, json.loads(r.read() or b'{}')
    except urllib.error.HTTPError as e: return e.code, json.loads(e.read() or b'{}')


def browser(spec, wait=True):
    args = ['node', str(HERE / 'browser_visitor.cjs'), BASE, CTX['token'], json.dumps(dict(spec, shot=str(OUT)))]
    env = dict(os.environ, NODE_PATH=os.environ.get('MANUAL_E2E_NODE_PATH', ''))
    if not wait: return subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
    p = subprocess.run(args, capture_output=True, text=True, env=env, timeout=400)
    return parse(p.stdout, spec, p.stderr)


def parse(stdout, spec, stderr=''):
    (OUT / f"browser_{spec.get('session_tag', 'a')}_{int(time.time())}.jsonl").write_text(stdout + ('\n#stderr ' + stderr[-1500:] if stderr else ''))
    ev = []
    for l in stdout.splitlines():
        try: ev.append(json.loads(l))
        except Exception: pass
    return ev


def first(ev, kind): return next((e for e in ev if e['kind'] == kind), {})


def statuses(ev): return [e['status'] for e in ev if e['kind'] == 'status']


CTX = {}


def scenario(fn):
    def run():
        say('---', fn.__name__)
        try: fn()
        except Exception as e:
            R.record(fn.__name__, False, error=repr(e)[:300], trace=traceback.format_exc()[-700:])
    run.__name__ = fn.__name__; return run


# ================================================================== setup
@scenario
def v00_setup_navigation_destinations_link():
    ready = lambda: (((F.state().get('execution') or {}).get('nav_idle') or {}).get('ok') is True
                     and (watch[0] or {}).get('state') == 'MOTION_AUTHORIZED' and F.robot().get('online'))
    up = wait_until(ready, 120)
    for attempt in range(2):          # known intermittent Nav2 lifecycle bring-up failure: restart stack + Agent (as nav_e2e ensure_nav2)
        if up: break
        CTX.setdefault('nav2_restarts', []).append(attempt)
        for op, n in (('kill', 'stack'), ('kill', 'agent')): subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), op, n], check=False)
        time.sleep(3)
        for op, n in (('start', 'stack'), ('start', 'agent')): subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), op, n], check=False); time.sleep(6)
        up = wait_until(ready, 120)
        try: F.post('/robots/iso_bench/recover', {'note': 'phase5 setup: stack restarted after Nav2 bring-up failure'})
        except HttpError: pass
    assert up, 'stack not ready'
    code = ("import sys,json,os; sys.path.insert(0,sys.argv[1]); from fleet_server.map_registry import MapRegistry; d=sys.argv[3]; "
            "r=MapRegistry(sys.argv[2]+'/map_registry.db', sys.argv[2]+'/map_store'); rd=lambda n: open(os.path.join(d,n),'rb').read(); "
            "res=r.register_map_bundle(floor_id='paichai_c_1f', label='phase5_guide', yaml_bytes=rd('map.yaml'), pgm_bytes=rd('map.pgm'), "
            "posegraph_bytes=rd('graph.posegraph'), posegraph_data_bytes=rd('graph.data'), source_mode='SLAM_SAVED_BUNDLE', verification_status='UNVERIFIED', "
            "verification_evidence={'method':'isolated_sim_phase5'}, frame_id='map', actor='e2e-qa', actor_role='operator'); mv=res['map_version']['map_version_id']; "
            "r.link_map_version_area(mv, 'paichai_c_1f_c103_lounge', actor='e2e-qa'); "
            "r.verify_map_version(mv, status='VERIFIED', evidence={'method':'isolated_sim_phase5','world':'kinematic_sim'}, actor='e2e-qa', actor_role='admin', reason='isolated E2E map'); print(mv)")
    out = subprocess.run([sys.executable, '-c', code, str(FC / 'server'), str(RUN / 'fleet'), str(FIXED)], capture_output=True, text=True)
    CTX['mv'] = out.stdout.strip().splitlines()[-1]
    maps = F.get('/map-registry/maps'); maps = maps.get('map_versions', maps.get('maps', []))
    CTX['c103'] = next((m['map_version_id'] for m in maps if m['map_version_id'] != CTX['mv'] and m['verification_status'] != 'VERIFIED'), None)
    lease(); assert switch('NAVIGATION'), 'NAVIGATION not reached'
    assert load_localize(CTX['mv']), 'localization not confirmed'
    p = pose(); cands = []
    for i in range(-7, 8):
        for j in range(-6, 7):
            x, y = round(p[0] + .35 * i, 2), round(p[1] + .35 * j, 2); d = math.hypot(x - p[0], y - p[1])
            if 1.2 <= d <= 2.4 and F.get(f'/robots/iso_bench/goal-check?x={x}&y={y}').get('ok'): cands.append((x, y))
    a = cands[0]; b = next(c for c in cands if math.hypot(c[0] - a[0], c[1] - a[1]) > 1.5)
    CTX['A'], CTX['B'] = a, b
    post = lambda body: F.post('/guide/destinations', body, who='operator')
    da = post({'name': '안내데스크', 'description': '1층 로비 안내데스크', 'map_version_id': CTX['mv'], 'x': a[0], 'y': a[1], 'aliases': ['info']})
    db = post({'name': '세미나실', 'description': '회의·세미나', 'map_version_id': CTX['mv'], 'x': b[0], 'y': b[1], 'area_id': 'paichai_c_1f_c103_lounge'})
    occ = None
    for i in range(-12, 13):
        c = F.get(f'/robots/iso_bench/goal-check?x={round(p[0] + .25 * i, 2)}&y={round(p[1] + 2.4, 2)}')
        if c.get('reason') == 'OCCUPIED': occ = (round(p[0] + .25 * i, 2), round(p[1] + 2.4, 2)); break
    bad = expect_http(lambda: post({'name': '벽', 'map_version_id': CTX['mv'], 'x': occ[0], 'y': occ[1]}), 409, 'destination_invalid') if occ else (None, 'no occupied cell found')
    c103 = None
    if CTX['c103']:
        for x in range(-30, 31, 3):
            for y in range(-30, 31, 3):
                try: c103 = post({'name': 'C103 라운지', 'map_version_id': CTX['c103'], 'x': x / 10, 'y': y / 10}); break
                except HttpError: continue
            if c103: break
    link = F.post('/visitor-links', {'label': 'phase5', 'ttl_s': 7200}, who='admin'); CTX['token'] = link['token']; CTX['link_id'] = link['link_id']
    op_link = expect_http(lambda: F.post('/visitor-links', {'label': 'x'}, who='operator'), 403)
    R.record('v00_setup_navigation_destinations_link', bool(CTX['mv']) and bool(da) and bool(db) and bad[0] is not False and op_link[0],
             map=CTX['mv'], c103=CTX['c103'], A=a, B=b, obstacle_destination=bad, c103_destination=(c103 or {}).get('reason'), operator_issue_link=op_link,
             link_path_shape=link['path'][:20] + '…', nav2_restarts=CTX.get('nav2_restarts', []))


@scenario
def v01_service_closed_then_open():
    ev = browser({'request': None, 'session_tag': 'closed'}); lst = first(ev, 'list')
    closed_ok = all(not d['available'] for d in lst.get('destinations', [])) and '운영 중이 아닙니다' in (lst.get('service') or '')
    F.post('/guide/service', {'robot_id': 'iso_bench', 'enabled': True}, who='operator')
    ev2 = browser({'request': None, 'session_tag': 'open'}); lst2 = first(ev2, 'list')
    av = {d['name']: (d['available'], d['why']) for d in lst2.get('destinations', [])}
    R.record('v01_service_closed_then_open', closed_ok and av.get('안내데스크', (False,))[0] and av.get('세미나실', (False,))[0]
             and (('C103 라운지' not in av) or (av['C103 라운지'][0] is False and '검증되지 않은' in (av['C103 라운지'][1] or ''))),
             closed=lst, opened=av, token_removed_from_url=not first(ev2, 'url_after_load').get('token_in_url'),
             no_operator_controls=not (lst2.get('layout') or {}).get('operatorControls'), mobile_no_hscroll=(lst2.get('layout') or {}).get('docW', 9e9) <= (lst2.get('layout') or {}).get('vw', 0))


@scenario
def v02_request_completes_robot_arrives():
    ev = browser({'request': '안내데스크', 'session_tag': 'v02', 'timeout_s': 150}); st = statuses(ev)
    p = pose(); a = CTX['A']
    g = F.get('/guide/requests', who='operator')['requests'][0]
    m = F.get(f"/missions/{g['mission_id']}") if g.get('mission_id') else {}
    R.record('v02_request_completes_robot_arrives', st[-1:] == ['COMPLETED'] and 'IN_PROGRESS' in st and math.hypot(p[0] - a[0], p[1] - a[1]) < .45
             and m.get('created_by') == 'operator' and 'visitor guide' in (m.get('note') or ''),
             ui_statuses=st, after_reload=first(ev, 'after_reload'), final_pose=p, goal=a, mission_status=m.get('status'), created_by=m.get('created_by'))


@scenario
def v03_cancel_during_guidance_stops_robot():
    ev = browser({'request': '세미나실', 'session_tag': 'v03', 'cancel_on': 'IN_PROGRESS', 'timeout_s': 90}); st = statuses(ev)
    t_cancel = next((e['t'] for e in ev if e['kind'] == 'cancel_clicked'), None)
    off = time.monotonic() - time.time()
    still = None
    if t_cancel:
        time.sleep(2.5)
        still = not [s for s in locked if s[0] >= t_cancel + off + 1.5 and (abs(s[1]) > 1e-6 or abs(s[2]) > 1e-6)]
    R.record('v03_cancel_during_guidance_stops_robot', 'IN_PROGRESS' in st and st[-1:] == ['CANCELED'] and still is True, ui_statuses=st, gate_zero_1_5s_after_cancel=still)


@scenario
def v04_two_visitors_queue_fifo():
    p1 = browser({'request': '세미나실', 'session_tag': 'q1', 'timeout_s': 240}, wait=False); time.sleep(1.5)
    p2 = browser({'request': '안내데스크', 'session_tag': 'q2', 'timeout_s': 240}, wait=False)
    o1, e1 = p1.communicate(timeout=420); o2, e2 = p2.communicate(timeout=420)
    s1, s2 = statuses(parse(o1, {'session_tag': 'q1'}, e1)), statuses(parse(o2, {'session_tag': 'q2'}, e2))
    R.record('v04_two_visitors_queue_fifo', s1[-1:] == ['COMPLETED'] and s2[-1:] == ['COMPLETED'] and 'QUEUED' in s2 and 'QUEUED' not in s1,
             first_visitor=s1, second_visitor=s2)


@scenario
def v05_unavailable_reasons():
    tok = 'Visitor ' + CTX['token']; out = {}
    def reason(name):
        st, j = visitor_call('GET', '/visitor/destinations', tok)
        return next((d['reason'] for d in j.get('destinations', []) if d['name'] == name), 'missing')
    assert switch('MANUAL'); out['manual'] = reason('안내데스크')
    did = next(d['destination_id'] for d in F.get('/guide/destinations')['destinations'] if d['name'] == '안내데스크')
    st, j = visitor_call('POST', '/visitor/requests', tok, {'destination_id': did}, session='p5v05' + 'y' * 16)
    out['request_in_manual'] = (st, j.get('status'), j.get('reason'))
    assert switch('NAVIGATION'); out['navigation_no_map'] = reason('안내데스크')
    if CTX['c103']:
        F.post('/robots/iso_bench/navigation', {'action': 'LOAD', 'epoch': EP[0], 'map_version_id': CTX['c103']}, who='operator')
        wait_until(lambda: loc().get('status') in ('WAIT_INITIAL_POSE', 'FAILED'), 45); out['other_map'] = reason('안내데스크')
    load_localize(CTX['mv']); out['back_on_map'] = reason('안내데스크')
    F.post('/robots/iso_bench/lock', {'action': 'force_release'}, who='admin'); out['lease_changed'] = reason('안내데스크')
    lease(); out['relocked_needs_reopen'] = reason('안내데스크')
    F.post('/guide/service', {'robot_id': 'iso_bench', 'enabled': True}, who='operator')
    if F.mode().get('mode') != 'NAVIGATION': switch('NAVIGATION'); load_localize(CTX['mv'])
    out['reopened'] = reason('안내데스크')
    want = {'manual': 'ROBOT_NOT_IN_NAVIGATION', 'navigation_no_map': 'ROBOT_ON_OTHER_MAP', 'back_on_map': None, 'lease_changed': 'OPERATOR_LEASE_CHANGED',
            'relocked_needs_reopen': 'OPERATOR_LEASE_CHANGED'}
    if CTX['c103']: want['other_map'] = 'ROBOT_ON_OTHER_MAP'
    ok = all(out.get(k) == v for k, v in want.items()) and out['request_in_manual'][1:] == ('UNAVAILABLE', 'ROBOT_NOT_IN_NAVIGATION')
    R.record('v05_unavailable_reasons', ok, observed=out, expected=want)


@scenario
def v06_visitor_token_isolation_and_revoke():
    V = 'Visitor ' + CTX['token']; res = {}
    for m, p, b in (('GET', '/robots', None), ('GET', '/dashboard', None), ('GET', '/missions', None), ('POST', '/robots/iso_bench/manual-bench', {'action': 'start'}),
                    ('POST', '/robots/iso_bench/gate-reset', {}), ('POST', '/robots/iso_bench/mapping', {'action': 'START'}),
                    ('POST', '/robots/iso_bench/mode', {'target': 'MANUAL'}), ('POST', '/missions', {'robot_id': 'iso_bench'}),
                    ('POST', '/robots/iso_bench/navigation', {'action': 'LOAD'}), ('GET', '/guide/requests', None), ('GET', '/visitor-links', None)):
        res[f'{m} {p}'] = visitor_call(m, p, V, b)[0]
    F.post(f"/visitor-links/{CTX['link_id']}/revoke", {}, who='admin')
    revoked = visitor_call('GET', '/visitor/destinations', V)[0]
    ev = browser({'request': None, 'session_tag': 'revoked'})
    R.record('v06_visitor_token_isolation_and_revoke', all(v == 401 for v in res.values()) and revoked == 401 and first(ev, 'list').get('link_problem') is True,
             operator_endpoints=res, after_revoke=revoked, page_after_revoke_shows_link_problem=first(ev, 'list').get('link_problem'))


@scenario
def v07_operator_map_click_uses_server_goal_check():
    """Operator dashboard map click (tests/gazebo_web/browser_map_click.cjs on this kinematic stack): the server goal check now decides.
    A point the server accepts must be accepted at the FIRST click (before: the stricter 0.32 m browser check refused some)."""
    if F.mode().get('mode') != 'NAVIGATION': switch('NAVIGATION')
    if loc().get('status') != 'CONFIRMED': load_localize(CTX['mv'])
    p = pose(); cands = []; occ = unk = None
    for i in range(-8, 9):
        for j in range(-6, 7):
            x, y = round(p[0] + .3 * i, 2), round(p[1] + .3 * j, 2); c = F.get(f'/robots/iso_bench/goal-check?x={x}&y={y}')
            d = math.hypot(x - p[0], y - p[1])
            if c.get('ok') and 1.0 <= d <= 2.2: cands.append((x, y))
            if c.get('reason') == 'OCCUPIED' and not occ: occ = (x, y)
            if c.get('reason') == 'UNKNOWN_AREA' and not unk: unk = (x, y)
    mm = F.robot().get('map_meta') or {}
    out_pt = (round(mm.get('origin_x', 0) - 0.4, 2), round(mm.get('origin_y', 0) + mm.get('height', 0) * mm.get('resolution', 0.05) / 2, 2)) if mm else None
    # near-wall server-valid points first: those are the ones the old 0.32 m browser check refused
    cands.sort(key=lambda c: [F.get(f'/robots/iso_bench/goal-check?x={c[0] + dx}&y={c[1] + dy}').get('ok') for dx, dy in ((.3, 0), (-.3, 0), (0, .3), (0, -.3))].count(True))
    tg = [dict(name=n, x=c[0], y=c[1], expect='blocked') for n, c in (('outside_map', out_pt), ('obstacle', occ), ('unknown_area', unk)) if c]
    tg.append({'name': 'valid', 'candidates': cands[:6], 'expect': 'submit', 'timeout_s': 120})
    env = dict(os.environ, NODE_PATH=os.environ.get('MANUAL_E2E_NODE_PATH', ''))
    pr = subprocess.run(['node', str(FC / 'tests/gazebo_web/browser_map_click.cjs'), BASE, str(RUN / 'secrets' / 'operator.token'), json.dumps(tg)],
                        capture_output=True, text=True, env=env, timeout=400)
    ev = parse(pr.stdout, {'session_tag': 'mapclick'}, pr.stderr)
    tgt = {e['name']: e for e in ev if e['kind'] == 'target'}; tries = [e for e in ev if e['kind'] == 'try']
    ui = next((e for e in ev if e['kind'] == 'mission_ui'), {})
    neg_ok = all(not tgt.get(t['name'], {}).get('submitted') for t in tg if t['expect'] == 'blocked')
    newest = F.get('/missions?robot_id=iso_bench&limit=1')['missions'][0]
    R.record('v07_operator_map_click_uses_server_goal_check', neg_ok and not tries and bool(tgt.get('valid', {}).get('submitted'))
             and 'SUCCEEDED' in (ui.get('text') or '') and newest['status'] == 'SUCCEEDED', newest_mission=(newest['mission_id'], newest['status'], newest['goal']), ui_refusals_of_server_valid_points=len(tries),
             negatives={k: (v.get('submitted'), (v.get('review') or '')[-40:]) for k, v in tgt.items() if k != 'valid'},
             valid=(tgt.get('valid') or {}).get('want'), mission_ui=(ui.get('text') or '')[:200], found={'occupied': occ, 'unknown': unk, 'outside': out_pt})


def main():
    try:
        for fn in (v00_setup_navigation_destinations_link, v01_service_closed_then_open, v02_request_completes_robot_arrives,
                   v03_cancel_during_guidance_stops_robot, v04_two_visitors_queue_fifo, v05_unavailable_reasons, v07_operator_map_click_uses_server_goal_check,
                   v06_visitor_token_isolation_and_revoke):
            fn()
    finally:
        try: (OUT / 'guide_requests.json').write_text(json.dumps(F.get('/guide/requests?limit=50', who='operator'), indent=1, ensure_ascii=False))
        except Exception: pass
        (OUT / 'result.json').write_text(json.dumps({'summary': R.counts(), 'results': R.items}, indent=1, default=str, ensure_ascii=False))
        say('SUMMARY', R.counts())
        subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), 'stopall'], check=False)
        shutdown_ros()


def shutdown_ros():
    """Stop the executor, JOIN its spin thread, then destroy nodes and shut rclpy down. Without the join the daemon spin thread is
    still inside rcl wait when the context is torn down and the interpreter exits -> C++ abort, rc=134 (separation experiment
    2026-11-21: shutdown-only 12/12 rc=134, +destroy 12/12 rc=134, +join 12/12 rc=0)."""
    ex.shutdown(); SPIN.join(10)
    for n in (obs,):
        try: n.destroy_node()
        except Exception: pass
    rclpy.try_shutdown()
    if SPIN.is_alive(): raise SystemExit('executor spin thread did not stop')


if __name__ == '__main__':
    main()
    sys.exit(1 if R.counts().get('FAIL') else 0)       # exit code = verdict (0 only when nothing FAILED and teardown completed)
