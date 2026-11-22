#!/usr/bin/env python3
"""End-to-end checks: Open-RMF -> Fleet Adapter -> Mission Manager -> Robot Agent -> (mock) Nav2, in an isolated environment.
usage: source /opt/ros/humble/setup.bash && source ~/rmf_isolated/env.sh && python3 tests/run_rmf_checks.py
Ports/domains: Mission Manager 18090, ROS domain 151 (RMF) and 152 (mock robot). Production (8080) and the real LIMO are never touched."""
import json, math, os, signal, sys, threading, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from rmf_env import *
import rclpy
from limo_rmf_adapter.coords import Transform
from ws_sink import WsSink
RMF_DOMAIN=ENV_RMF['ROS_DOMAIN_ID']
CFG = json.load(open(os.path.join(RMF, 'config', 'fleet.json'))); TF = Transform.from_config(CFG['transform_rmf_from_nav2'])
WP = {w['name']: tuple(w['rmf']) for w in CFG['waypoints']}
NAV = {n: TF.rmf_to_nav2(*xy)[:2] for n, xy in WP.items()}
SAMPLER = {'max_inflight': 0, 'stop': False, 'overlap': []}


def sampler():
    while not SAMPLER['stop']:
        ms = (http('GET', '/api/v1/missions?robot_id=pcu&limit=200', tok('viewer'))[1] or {}).get('missions', [])
        live = [m for m in ms if not m['terminal'] and m['status'] != 'QUEUED']   # QUEUED = waiting, not in flight
        if len(live) > SAMPLER['max_inflight']: SAMPLER['max_inflight'] = len(live)
        if len(live) > 1: SAMPLER['overlap'].append([(m['mission_id'], m['status'], m['created_by']) for m in live])
        time.sleep(0.15)


def pose():
    p = ((robot() or {}).get('state') or {}).get('pose') or {}; return (p.get('x'), p.get('y')) if p.get('status') == 'OK' else None


def near(name, tol=0.35):
    p = pose(); return bool(p) and math.hypot(p[0] - NAV[name][0], p[1] - NAV[name][1]) < tol


def settle(timeout=60, quiet=4.0):
    """Wait until nothing is in flight and no new mission appeared for `quiet` seconds (RMF may issue a follow-up path after a cancel/finish)."""
    end = time.time() + timeout; last_n = -1; t_quiet = time.time()
    while time.time() < end:
        ms = missions(); n = len(ms)
        if n != last_n or any(not m['terminal'] for m in ms): last_n = n; t_quiet = time.time()
        elif time.time() - t_quiet >= quiet: return True
        time.sleep(0.4)
    return False


def rmf_missions(): return [m for m in missions() if (m.get('idempotency_key') or '').startswith('rmf:')]
def log_of(n): return open(os.path.join(T, n + '.log')).read() if os.path.exists(os.path.join(T, n + '.log')) else ''
def new_task(c, place):
    r = c.dispatch_patrol([place]); return ((r or {}).get('state') or {}).get('booking', {}).get('id'), r


def _watchdog():
    say('WATCHDOG: overall time limit reached, aborting'); cleanup(); os._exit(3)


threading.Timer(float(os.environ.get('RMF_E2E_TIMEOUT_S', '1200')), _watchdog).start()
try:
    section('0. static checks of the RMF map <-> Nav2 map connection')
    from PIL import Image
    import numpy as np
    my = open(MOCK_MAP).read(); ox, oy, _ = [float(v) for v in my.split('origin:')[1].split('[')[1].split(']')[0].split(',')]
    res = 0.05; im = np.array(Image.open(os.path.splitext(MOCK_MAP)[0] + '.pgm')); H = im.shape[0]

    def cell(x, y): return im[H - 1 - int((y - oy) / res), int((x - ox) / res)]
    check('transform round trip (RMF->Nav2->RMF) < 1e-9 m', all(math.hypot(*(np.array(TF.nav2_to_rmf(*TF.rmf_to_nav2(*xy, 0.3))[:2]) - np.array(xy))) < 1e-9 for xy in WP.values()))
    check('transform is not the identity (a real frame offset is exercised)', abs(TF.theta) > 0.1 and abs(TF.tx) > 1)
    check('every RMF waypoint maps to a FREE cell of the Nav2 map', all(cell(*NAV[n]) == 254 for n in WP), str({n: (round(NAV[n][0], 2), round(NAV[n][1], 2)) for n in WP}), req=1)
    from numpy.lib.stride_tricks import sliding_window_view as sw
    K = 4; okm = ~sw(np.pad(im != 254, K, constant_values=True), (2 * K + 1, 2 * K + 1)).max(axis=(2, 3))
    bad = 0
    for a, b in CFG['lanes']:
        for t in np.linspace(0, 1, 60):
            x = NAV[a][0] + t * (NAV[b][0] - NAV[a][0]); y = NAV[a][1] + t * (NAV[b][1] - NAV[a][1])
            if not okm[H - 1 - int((y - oy) / res), int((x - ox) / res)]: bad += 1
    check('every lane is a straight line with >=0.2 m clearance from occupied/unknown cells of the Nav2 map', bad == 0, f'{bad} blocked samples', req=1)

    section('1. RMF alone, then the isolated Mission Manager + simulated robot + adapter')
    SINK = WsSink(WS_PORT, os.path.join(T, 'rmf_api_stream.jsonl'))
    init_server(); check('Mission Manager (isolated 18090) up', start_server())
    start_rmf_base(); time.sleep(3)
    nodes = os.popen(f'bash -c "source /opt/ros/humble/setup.bash; source {RMF_PREFIX}; ROS_DOMAIN_ID={RMF_DOMAIN} ROS_LOCALHOST_ONLY=1 ros2 node list 2>/dev/null"').read()
    check('RMF traffic schedule and task dispatcher nodes are running', 'rmf_traffic_schedule_node' in nodes and 'rmf_dispatcher_node' in nodes)
    start_robot(speed=0.6)
    check('simulated robot pcu online at the Mission Manager', wait(lambda: (robot() or {}).get('online') and pose(), 25))
    threading.Thread(target=sampler, daemon=True).start()
    start_adapter()
    os.environ.update(ROS_DOMAIN_ID=RMF_DOMAIN, ROS_LOCALHOST_ONLY='1'); rclpy.init()
    from rmf_client import RmfClient
    c = RmfClient()
    r = wait(lambda: c.robot(), 30)
    check('fleet limo_fleet / robot pcu registered in RMF and reporting', r is not None, '' if r is None else f'({r.location.x:.2f},{r.location.y:.2f})', req=2)
    p = pose(); ex = TF.nav2_to_rmf(*p)
    check('RMF position equals the transformed Nav2 pose (<0.1 m)', r is not None and math.hypot(r.location.x - ex[0], r.location.y - ex[1]) < 0.1, f'rmf=({r.location.x:.2f},{r.location.y:.2f}) expected=({ex[0]:.2f},{ex[1]:.2f})')
    check('robot id is pcu and the map name is the RMF level', r.name == 'pcu' and r.location.level_name == CFG['map_name'])
    base = len(rmf_missions())

    section('2. RMF task -> Mission Manager mission -> mock Nav2')
    tid, resp = new_task(c, 'wp_b'); check('RMF accepted a patrol task to wp_b', bool(tid), str(resp)[:100], req=3)
    ds = wait(lambda: (lambda d: d if d and d.assignment.is_assigned else None)(c.dispatch_state(tid)), 20)
    check('RMF bidding finished: task assigned to fleet limo_fleet / robot pcu', bool(ds) and ds.assignment.expected_robot_name == 'pcu' and ds.assignment.fleet_name == 'limo_fleet', '' if not ds else f'{ds.assignment.fleet_name}/{ds.assignment.expected_robot_name}', req=4)
    ok = wait(lambda: SINK.status(tid) == 'completed', 90)
    check('RMF task reports status completed (RMF task_state_update)', bool(ok), f'state={SINK.status(tid)}', req=7)
    check('robot physically at wp_b in the Nav2 map frame', wait(lambda: near('wp_b'), 8), str(pose()), req=6)
    ms = [m for m in rmf_missions() if m['status'] == 'SUCCEEDED']
    check('missions were created through the Mission Manager by the RMF principal', len(ms) >= 1 and all(m['created_by'] == 'rmf-adapter' for m in rmf_missions()), f'{len(ms)} missions', req=5)
    check('idempotency keys are unique (no duplicate goals)', len({m['idempotency_key'] for m in rmf_missions()}) == len(rmf_missions()), req=13)
    check('never more than one non-terminal mission at a time', SAMPLER['max_inflight'] <= 1, f"max={SAMPLER['max_inflight']} {SAMPLER['overlap'][:1]}")

    section('3. cancel from RMF')
    tid, _ = new_task(c, 'wp_e')
    ge = NAV['wp_e']
    def long_leg():
        for m in rmf_missions():
            if m['status'] == 'EXECUTING' and math.hypot(m['goal']['x'] - ge[0], m['goal']['y'] - ge[1]) < 0.05 and (m.get('progress') or {}).get('distance_remaining_m', 0) > 1.2: return m
    target = wait(long_leg, 40)
    check('a long leg towards wp_e is executing (>1.2 m to go) before the cancel', bool(target), req=8)
    cr = c.cancel(tid); check('RMF accepted the cancel request', bool(cr) and cr.get('success') is True, str(cr)[:100])
    tm = lambda: http('GET', f"/api/v1/missions/{target['mission_id']}", tok('viewer'))[1]
    canceled = wait(lambda: tm()['status'] == 'CANCELED', 25)
    check('that Mission Manager mission ended CANCELED (confirmed by the robot side)', bool(canceled), tm()['status'], req=8)
    cmds = tm().get('commands') or []
    check('cancel went through a cancel_mission command that the robot side completed', any(x['type'] == 'cancel_mission' and x['status'] == 'SUCCEEDED' for x in cmds), str([x['type'] + ':' + x['status'] for x in cmds]), req=8)
    cm = tm(); cdone = cm['updated_at']
    later = [m for m in rmf_missions() if m['created_at'] > cm['created_at']]
    check('no later RMF mission was submitted before the cancel was confirmed', all(m['created_at'] >= cdone - 0.05 for m in later), str([(m['mission_id'], round(m['created_at'] - cdone, 2)) for m in later]))
    settle()
    again = [m for m in rmf_missions() if m['created_at'] > cm['created_at'] and math.hypot(m['goal']['x'] - ge[0], m['goal']['y'] - ge[1]) < 0.05]
    check('the canceled goal (wp_e) was not re-sent after the cancel', not again)
    p1 = pose(); time.sleep(2.5); p2 = pose()
    check('robot is stationary once RMF has no further path (and did not reach wp_e)', math.hypot(p2[0] - p1[0], p2[1] - p1[1]) < 0.05 and not near('wp_e'), f'{p1}->{p2}', req=8)
    check('RMF task status is canceled (RMF task_state_update)', bool(wait(lambda: SINK.status(tid) == 'canceled', 25)), f'status={SINK.status(tid)}', req=8)
    settle()
    section('4. ownership: server-side control lease (RMF holds it) vs web UI')
    lease = lambda: (http('GET', '/api/v1/robots/pcu/lock', tok('viewer'))[1] or {}).get('lease')
    check('adapter holds the control lease of pcu', bool(wait(lambda: (lease() or {}).get('owner') == 'rmf-adapter', 15)), str(lease()), req=9)
    gw = {'robot_id': 'pcu', 'type': 'navigate_to_pose', 'goal': {'frame_id': 'map', 'x': NAV['wp_c'][0], 'y': NAV['wp_c'][1], 'yaw_rad': 0}}
    n_all = len(missions())
    st, w = http('POST', '/api/v1/missions', tok('operator'), dict(gw, idempotency_key='web-conflict'))
    check('web mission while RMF holds the lease: HTTP 409 ownership_conflict (not queued)', st == 409 and w['error']['code'] == 'ownership_conflict', f'{st} {str(w)[:120]}', req=9)
    check('the rejected request created no mission', len(missions()) == n_all, req=9)
    tidq, _ = new_task(c, 'wp_e'); wait(lambda: [m for m in rmf_missions() if m['status'] == 'EXECUTING'], 40)
    st, w = http('POST', '/api/v1/missions', tok('operator'), dict(gw, idempotency_key='web-conflict-2'))
    check('web mission while an RMF mission is EXECUTING: HTTP 409 (never queued behind it)', st == 409, str(st), req=9)
    c.cancel(tidq); settle()
    # takeover by the operator: admin releases the RMF lease, web mission runs, RMF must refuse work meanwhile and re-acquire afterwards
    wid = None
    for _ in range(6):
        http('POST', '/api/v1/robots/pcu/lock', tok('admin'), {'action': 'force_release'})
        st, w = http('POST', '/api/v1/missions', tok('operator'), dict(gw, idempotency_key='web-takeover'))
        if st in (200, 201): wid = w['mission_id']; break
    check('admin force-release lets the web operator take control', wid is not None, str(st))
    n_rmf = len(rmf_missions()); tid2, r2 = new_task(c, 'wp_a')
    web_done = wait(lambda: http('GET', f'/api/v1/missions/{wid}', tok('viewer'))[1]['status'] == 'SUCCEEDED', 40)
    check('web mission executed', bool(web_done))
    check('no RMF mission started while the web mission was active (missions never overlapped)', SAMPLER['max_inflight'] <= 1 and not SAMPLER['overlap'], str(SAMPLER['overlap'][:1]), req=9)
    time.sleep(3); d2 = c.dispatch_state(tid2)
    check('RMF task offered during the web mission was explicitly REFUSED (FAILED_TO_ASSIGN)', d2 is not None and (not d2.assignment.is_assigned) and d2.status == 4, f"status={d2 and d2.status} errors={d2 and list(d2.errors)}"[:200], req=9)
    check('no RMF mission was created for the refused task', n_rmf == len(rmf_missions()), f'{n_rmf}->{len(rmf_missions())}', req=9)
    check('adapter re-acquires the lease after the web mission ended', bool(wait(lambda: (lease() or {}).get('owner') == 'rmf-adapter', 20)), str(lease()), req=9)
    settle()
    tid2, _ = new_task(c, 'wp_a')
    check('then a new RMF task is accepted and completes', bool(wait(lambda: SINK.status(tid2) == 'completed', 90)), f'state={SINK.status(tid2)}', req=9)
    settle()

    section('5. fault injection: Mission Manager (server) outage during an RMF task')
    tid, _ = new_task(c, 'wp_e'); wait(lambda: [m for m in rmf_missions() if m['status'] == 'EXECUTING'], 40); time.sleep(2)
    keys_before = {m['idempotency_key'] for m in rmf_missions()}
    n_sub = log_of('adapter').count('] submit:')
    kill('server', signal.SIGKILL); t_kill = time.time()
    off = wait(lambda: 'status override: offline' in log_of('adapter'), 20)
    check('adapter reports the robot OFFLINE to RMF when the Mission Manager is unreachable', bool(off), f'{time.time() - t_kill:.1f}s', req=10)
    time.sleep(3)
    check('adapter created no new mission while disconnected (no guessing)', log_of('adapter').count('] submit:') == n_sub, f"submits {n_sub}->{log_of('adapter').count('] submit:')}", req=10)
    start_server(); t_up = time.time()
    back = wait(lambda: 'status override: None' in log_of('adapter').split('status override: offline')[-1], 40)
    check('adapter noticed the reconnect and cleared the offline status', bool(back), f'{time.time() - t_up:.1f}s')
    done = wait(lambda: SINK.status(tid) == 'completed', 120)
    check('RMF task completed after reconnect (state re-synchronized)', bool(done), f'state={SINK.status(tid)}', req=10)
    check('robot is at wp_e', wait(lambda: near('wp_e', 0.45), 10), str(pose()))
    keys = [m['idempotency_key'] for m in rmf_missions()]
    check('no duplicated goal: every idempotency key exists exactly once', len(keys) == len(set(keys)), req=13)
    check('after the outage still never >1 non-terminal mission', SAMPLER['max_inflight'] <= 1, f"{SAMPLER['overlap'][:1]}")

    settle()
    section('6. adapter restart in the middle of a mission')
    tid, _ = new_task(c, 'wp_a'); wait(lambda: [m for m in rmf_missions() if m['status'] == 'EXECUTING'], 40); time.sleep(2)
    active_before = [m['mission_id'] for m in rmf_missions() if not m['terminal']]
    kill('adapter', signal.SIGKILL); time.sleep(1); start_adapter()
    orphan_ok = wait(lambda: all(m['terminal'] for m in rmf_missions()), 40)
    check('orphan mission of the dead adapter is canceled/terminal before anything new', bool(orphan_ok) and bool(active_before), f'{active_before}', req=11)
    check('adapter re-registers the robot in RMF', bool(wait(lambda: 'registered' in log_of('adapter').split('previous run left')[-1], 40)), req=11)

    section('7. agent restart in the middle of a mission -> RECONCILIATION_REQUIRED')
    c.cancel(tid); time.sleep(3)
    tid, _ = new_task(c, 'wp_e'); wait(lambda: [m for m in rmf_missions() if m['status'] == 'EXECUTING'], 60); time.sleep(2)
    kill('agent', signal.SIGKILL); time.sleep(1); start_agent()
    rec = wait(lambda: any(m['status'] == 'RECONCILIATION_REQUIRED' for m in missions()) or (robot() or {}).get('reconciliation_required'), 40)
    check('Mission Manager marks the unverifiable mission RECONCILIATION_REQUIRED (no resend)', bool(rec), req=12)
    n = len(rmf_missions()); tid3, r3 = new_task(c, 'wp_a'); time.sleep(2)
    check('no new RMF mission is created while reconciliation is required', len(rmf_missions()) == n, f'{n}->{len(rmf_missions())}', req=14)
    check('RMF task was not reported completed', SINK.status(tid) != 'completed', f'state={SINK.status(tid)}', req=14)

    section('7b. operator recovery of RECONCILIATION_REQUIRED (evidence-gated)')
    rm = [m for m in missions() if m['status'] == 'RECONCILIATION_REQUIRED']
    check('there is a RECONCILIATION_REQUIRED mission to recover', len(rm) >= 1)
    mid = rm[0]['mission_id']; adm = tok('admin'); note = 'operator verified on the simulated robot that no goal is running'
    st, d = http('POST', f'/api/v1/missions/{mid}/recover', tok('operator'), {'note': note}); check('operator role cannot recover (403)', st == 403)
    st, d = http('POST', f'/api/v1/missions/{mid}/recover', adm, {'note': note})
    print('     first recovery attempt:', st, str(d)[:140])
    if st == 409 and d['error']['code'] == 'goal_still_active':
        check('while the mock Nav2 still has the (orphaned) goal active, recovery is REFUSED (409 goal_still_active) and no new mission is allowed', True, req=14)
        nm = len(rmf_missions()); time.sleep(2); check('no new RMF mission meanwhile', len(rmf_missions()) == nm, req=14)
    else: check('recovery is refused while the goal is active (or the goal already ended)', st in (409, 200), f'{st}', req=14)
    gl = lambda: [json.loads(l) for l in open(os.path.join(T, 'mock_goals.jsonl'))] if os.path.exists(os.path.join(T, 'mock_goals.jsonl')) else []
    ended = wait(lambda: ((robot() or {}).get('state') or {}).get('execution', {}).get('nav2_goals', {}).get('active_count') == 0, 60)
    check('the orphaned mock goal ends by itself (nobody canceled or completed it on its behalf)', bool(ended))
    time.sleep(1.5)
    if st != 200:
        st, d = http('POST', f'/api/v1/missions/{mid}/recover', adm, {'note': note})
    check('admin recovery accepted once Nav2 reports no active goal', st == 200, f'{st} {str(d)[:100]}', req=14)
    mm = http('GET', f'/api/v1/missions/{mid}', tok('viewer'))[1]
    check('recovered mission is FAILED / RECOVERED_UNVERIFIED_OUTCOME (not SUCCEEDED, not CANCELED)', mm['status'] == 'FAILED' and 'RECOVERED_UNVERIFIED_OUTCOME' in (mm['detail'] or ''), f"{mm['status']}", req=14)
    settle(); tid4, _ = new_task(c, 'wp_a')
    check('after the recovery a new RMF task is accepted, moves the robot and RMF reports completed', bool(wait(lambda: SINK.status(tid4) == 'completed', 90)) , f'state={SINK.status(tid4)}', req=14)
    section('7c. goal accounting: every goal the mock Nav2 received corresponds to exactly one acknowledged Mission Manager command')
    time.sleep(2)
    got = [g for g in gl() if g['ev'] == 'goal_received']
    acked = [(m['mission_id'], x['command_id']) for m in missions() for x in (m.get('commands') or []) if x['type'] == 'navigate_to_pose' and x['acked_at'] and x['status'] not in ('RECONCILIATION_REQUIRED', 'REJECTED', 'EXPIRED')]   # refused by the agent = never reached Nav2
    check('mock Nav2 goal count == acknowledged navigate commands', len(got) == len(acked), f'mock={len(got)} acked_commands={len(acked)}', req=13)
    check('no command was acknowledged twice (command ids unique)', len({c_ for _, c_ in acked}) == len(acked), req=13)
    gt = [g['t'] for g in got]
    check('no two goals reached Nav2 within 0.3 s with the same target (no double send)', not any(abs(a['x'] - b['x']) < 1e-6 and abs(a['y'] - b['y']) < 1e-6 and abs(a['t'] - b['t']) < 0.3 for i_, a in enumerate(got) for b in got[i_ + 1:]), req=13)
except BaseException:
    import traceback; traceback.print_exc(); check('test run aborted by an exception', False)
finally:
    SAMPLER['stop'] = True
    try: rclpy.shutdown()
    except Exception: pass
    section('15. cleanup'); cleanup(); time.sleep(1)
    left = os.popen(f"ps -eo pid,args | grep -E '{T}|:{PORT}|--port {PORT}' | grep -v grep | grep -v run_rmf_checks").read().strip()
    check('no test process is left after cleanup', not left, left[:200], req=15)
    SECTIONS[-1][2] = time.time()
    names = {1: 'Nav2 map <-> RMF waypoints/lanes', 2: 'RMF robot registration', 3: 'RMF patrol task creation', 4: 'RMF bidding and assignment', 5: 'RMF path -> Mission Manager missions', 6: 'mock robot really moved', 7: 'RMF reports task COMPLETED', 8: 'RMF cancel confirmed', 9: 'web vs RMF ownership conflict', 10: 'server outage + reconnect', 11: 'Fleet Adapter restart', 12: 'Robot Agent restart', 13: 'no duplicate execution', 14: 'RECONCILIATION_REQUIRED blocks commands', 15: 'no leftover processes'}
    print('\n== required items');
    for k in range(1, 16): print(f"  {k:2d}. {names[k]:42s} {'NOT RUN' if k not in REQ else ('PASS' if all(REQ[k]) else 'FAIL')}")
    print('== section durations')
    for n, a, b in SECTIONS: print(f"  {(b or time.time()) - a:6.1f}s  {n}")
    p_ = sum(1 for _, o in RES if o); print(f'\n{p_}/{len(RES)} checks passed; logs in {T}', flush=True)
    json.dump({'results': RES, 'required': {str(k): (None if k not in REQ else all(REQ[k])) for k in range(1, 16)}, 'logs': T}, open(os.path.join(T, 'results.json'), 'w'))
    os._exit(0 if p_ == len(RES) else 1)
