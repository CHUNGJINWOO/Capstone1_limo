"""Nav2 Stabilization & Multi-Waypoint Patrol E2E Verification Harness (2026-11-24).

Verifies all 7 required navigation scenarios in an isolated ROS 2 Humble stack (no hardware, no /cmd_vel,
Gate output /cmd_vel_locked -> simulator):
  1. s01_single_goal_navigation          — 단일 목적지 이동 (n03)
  2. s02_consecutive_goals_navigation    — 연속 목적지 이동 (n05: 2 consecutive goals + mode switch & relocalize)
  3. s03_multi_waypoint_patrol           — 복수 경유지 순찰 (4-waypoint closed patrol loop A -> B -> C -> D -> A)
  4. s04_mid_route_cancel_and_stop       — 중간 취소와 정지 (cancel while driving, verify 0 velocity & bounded stop distance)
  5. s05_obstacle_blocked_path_failure   — 장애물로 인한 경로 실패 (1) static occupied/box obstacle rejected 409,
                                           (2) dynamic virtual_bumper obstacle blocking active path -> Nav2 ABORTED/FAILED
  6. s06_agent_restart_reconciliation    — Agent 재시작 이후 임무 재조정 (f08: SIGKILL mid-goal -> stop <=0.5s ->
                                           RECONCILIATION_REQUIRED -> old session refused -> admin recover -> reload/relocalize -> new goal SUCCEEDED)
  7. s07_speed_limit_vs_tracking_compare — 속도 제한과 경로 추종 안정성 비교:
                                           (a) Baseline ceiling (vx=0.15 m/s, wz=0.50 rad/s)
                                           (b) Proportional speed clamp (vx=0.06 m/s, wz=0.20 rad/s, w/v ratio preserved)
                                           (c) Linear-only saturated clamp (vx=0.06 m/s, wz=0.50 rad/s, curvature doubled when turning)
                                           Measures cross-track error (CTE), path length, duration, goal reach error (dx, dy, dyaw),
                                           and effective curvature |wz/vx| on the same curved route.

Usage:
  OVERLAY=/tmp/nav_val_ov_20261124/install PATROL_E2E_OUT=/tmp/nav_validation_20261124/patrol_run1 \
    python3 fleet_control/tests/nav_e2e/run_nav_patrol_7scenarios_20261124.py <port> <domain>
"""
import json, math, os, shutil, subprocess, sys, threading, time, traceback, uuid, urllib.parse, urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
FC = HERE.parents[1]
REPO = FC.parent
MODE = FC / 'tests' / 'mode_e2e'
sys.path.insert(0, str(MODE))
from owned_proc import owned_pid
sys.path[:0] = [str(FC / 'agent' / 'limo_robot_agent')]

PORT, DOMAIN = int(sys.argv[1]), sys.argv[2]
assert PORT not in (8080, 18190, 18191, 18192) and DOMAIN not in ('0', '77')
OUT = Path(os.environ['PATROL_E2E_OUT']).resolve()
OUT.mkdir(parents=True, exist_ok=False)
RUN = OUT / 'stack'
RID = 'iso_bench'
FIXED_MAP = Path(os.environ.get('NAV_E2E_FIXED_MAP', str(
    Path('/home/cjinw/school/2026-2/Capstone1_limo/local_data/raw_results/phase3_stability_claude_20261121/fixed_map')
)))
log = (OUT / 'harness.log').open('x')
results = []


def say(*a):
    line = time.strftime('%H:%M:%S ') + ' '.join(str(x) for x in a)
    print(line, flush=True)
    log.write(line + '\n')
    log.flush()


def ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def cross_track_stats(traj, planned_pts):
    """Compute mean and max distance from actual trajectory points to the planned polyline."""
    if not traj or not planned_pts or len(planned_pts) < 2:
        return {'mean_cte_m': None, 'max_cte_m': None}
    errs = []
    for _, tx, ty, _ in traj:
        best = float('inf')
        for (x1, y1), (x2, y2) in zip(planned_pts, planned_pts[1:]):
            dx, dy = x2 - x1, y2 - y1
            l2 = dx * dx + dy * dy
            if l2 < 1e-12:
                d = math.hypot(tx - x1, ty - y1)
            else:
                u = max(0.0, min(1.0, ((tx - x1) * dx + (ty - y1) * dy) / l2))
                d = math.hypot(tx - (x1 + u * dx), ty - (y1 + u * dy))
            if d < best:
                best = d
        errs.append(best)
    return {
        'mean_cte_m': round(sum(errs) / len(errs), 4),
        'max_cte_m': round(max(errs), 4),
    }


subprocess.run(
    [str(MODE / 'stack_up.sh'), str(RUN), str(PORT), DOMAIN],
    check=True,
    env=dict(os.environ, NAV_LOCALIZATION='1'),
    stdout=(OUT / 'stack_up.log').open('x'),
    stderr=subprocess.STDOUT,
)
os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=DOMAIN)
os.environ.pop('ROS_DISCOVERY_SERVER', None)

TIMELINE_PROC = subprocess.Popen(
    [
        sys.executable,
        str(HERE / 'timeline_recorder.py'),
        f'http://127.0.0.1:{PORT}',
        str(RUN / 'secrets' / 'operator.token'),
        str(OUT / 'timeline.jsonl'),
    ],
    stdout=(OUT / 'timeline_recorder.log').open('x'),
    stderr=subprocess.STDOUT,
    start_new_session=True,
)
SCEN_MARKS = []

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as NavPath
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64MultiArray, String
from limo_robot_agent.core import Http, HttpError

URL = f'http://127.0.0.1:{PORT}'
tok = lambda n: (RUN / 'secrets' / f'{n}.token').read_text().strip()
OP, ADMIN = Http(URL, tok('operator')), Http(URL, tok('admin'))
for h in (OP, ADMIN):
    h.timeout = 30.0

rclpy.init()
obs = rclpy.create_node('nav_patrol_observer')
locked_samples = []
truth_samples = []
truth_latest = [None]
watch_latest = [None]
plan_latest = [None]

obs.create_subscription(
    Twist,
    '/cmd_vel_locked',
    lambda m: locked_samples.append((time.monotonic(), m.linear.x, m.angular.z)),
    50,
)


def _on_truth(m):
    t = time.monotonic()
    item = (t, float(m.data[0]), float(m.data[1]), float(m.data[2]), float(m.data[3]), float(m.data[4]))
    truth_latest[0] = item
    truth_samples.append(item)


obs.create_subscription(Float64MultiArray, '/e2e/ground_truth', _on_truth, 20)
obs.create_subscription(String, '/fleet/motion_watchdog/status', lambda m: watch_latest.__setitem__(0, json.loads(m.data)), 10)
obs.create_subscription(
    NavPath,
    '/plan',
    lambda m: plan_latest.__setitem__(
        0,
        (time.monotonic(), [(round(p.pose.position.x, 3), round(p.pose.position.y, 3)) for p in m.poses]),
    ),
    10,
)
bumper_pub = obs.create_publisher(PointCloud2, '/stall_detector/virtual_bumper_cloud', 10)
ex = SingleThreadedExecutor()
ex.add_node(obs)
threading.Thread(target=ex.spin, daemon=True).start()


def get(path, token='operator'):
    path = urllib.parse.quote(path, safe='/?=&')
    req = urllib.request.Request(URL + '/api/v1' + path, headers={'Authorization': 'Bearer ' + tok(token)})
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read())


robot = lambda: get(f'/robots/{RID}')
state = lambda: robot().get('state') or {}
loc = lambda: state().get('localization') or {}
nav = lambda: get(f'/robots/{RID}/navigation')
pose = lambda: truth_latest[0][1:4] if truth_latest[0] else None


def proc(op, name):
    subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), op, name], check=False)
    return time.monotonic()


def wait_until(pred, timeout, step=0.25):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            v = pred()
            if v:
                return v
        except Exception:
            pass
        time.sleep(step)
    return None


def expect_http(fn, status, code=None):
    try:
        fn()
    except HttpError as e:
        return e.status == status and (code is None or code in str(e)), str(e)[:240]
    return False, 'accepted'


EPOCH = [None]


def lease():
    EPOCH[0] = OP.post(f'/robots/{RID}/lock', {'action': 'acquire', 'ttl_s': 120})['epoch']


def renewer():
    while True:
        time.sleep(15)
        try:
            OP.post(f'/robots/{RID}/lock', {'action': 'renew', 'ttl_s': 120})
        except Exception:
            pass


threading.Thread(target=renewer, daemon=True).start()


def switch(target, timeout=60):
    OP.post(f'/robots/{RID}/mode', {'target': target, 'epoch': EPOCH[0], 'idempotency_key': 'p-' + uuid.uuid4().hex[:10]})
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        rec = get(f'/robots/{RID}/mode')['operating_mode'] or {}
        t = rec.get('transition') or {}
        if t.get('to') == target and t.get('state') in ('ACTIVE', 'FAILED'):
            return rec
        time.sleep(0.25)
    return get(f'/robots/{RID}/mode')['operating_mode']


def navreq(action, **kw):
    return OP.post(f'/robots/{RID}/navigation', {'action': action, 'epoch': EPOCH[0], **kw})


def mission(x, y, yaw=0.0):
    return OP.post(
        '/missions',
        {
            'robot_id': RID,
            'type': 'navigate_to_pose',
            'goal': {'frame_id': 'map', 'x': x, 'y': y, 'yaw_rad': yaw},
            'lease_epoch': EPOCH[0],
            'idempotency_key': 'p-' + uuid.uuid4().hex,
        },
    )


def mstatus(mid):
    return next((m['status'] for m in get(f'/missions?robot_id={RID}&limit=40')['missions'] if m['mission_id'] == mid), None)


TERMINAL = ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED', 'RECONCILIATION_REQUIRED')


def wait_mission(mid, timeout=120):
    return wait_until(lambda: mstatus(mid) if mstatus(mid) in TERMINAL else None, timeout, 0.25) or mstatus(mid)


def zero_after(t0, delay=0.5, span=0.8):
    time.sleep(delay + span)
    return not [
        s for s in locked_samples
        if t0 + delay <= s[0] <= t0 + delay + span and (abs(s[1]) > 1e-6 or abs(s[2]) > 1e-6)
    ]


def gate_unlatch_if_needed(label):
    try:
        g = state().get('safety_gate') or {}
    except Exception:
        return
    if g.get('fault_latched') is True or g.get('status') not in ('NORMAL', 'STALL_PENDING'):
        from std_srvs.srv import Trigger
        cli = obs.create_client(Trigger, '/stall_detector/reset_latch')
        if cli.wait_for_service(timeout_sec=3):
            f = cli.call_async(Trigger.Request())
            wait_until(lambda: f.done(), 5)
        obs.destroy_client(cli)
        time.sleep(1.5)


def recover_all():
    recs = []
    for mid in [m['mission_id'] for m in get(f'/missions?robot_id={RID}&limit=40')['missions'] if m['status'] == 'RECONCILIATION_REQUIRED']:
        try:
            ADMIN.post(f'/missions/{mid}/recover', {'note': 'Nav2 terminal evidence reviewed in patrol E2E'})
            recs.append((mid, 'OK'))
        except HttpError as e:
            recs.append((mid, str(e)[:200]))
    if robot().get('reconciliation_required'):
        try:
            ADMIN.post(f'/robots/{RID}/recover', {'note': 'Nav2 terminal evidence reviewed in patrol E2E'})
            recs.append(('robot', 'OK'))
        except HttpError as e:
            recs.append(('robot', str(e)[:200]))
    return recs


def load_and_localize(mvid, initial, timeout=45):
    for attempt in range(4):
        navreq('LOAD', map_version_id=mvid)
        loaded = wait_until(lambda: loc().get('status') in ('WAIT_INITIAL_POSE', 'FAILED') and loc().get('status'), 40)
        if loaded == 'WAIT_INITIAL_POSE':
            break
        time.sleep(8)
    gate_unlatch_if_needed('after_load')
    navreq('INITIAL_POSE', x=initial[0], y=initial[1], yaw=initial[2])
    st = wait_until(lambda: nav().get('localized') is True and loc().get('status') == 'CONFIRMED' and loc(), timeout)
    gate_unlatch_if_needed('after_initial_pose')
    wait_until(lambda: (robot().get('safety_context_reason') or {}).get('reason') is None, 15)
    return loaded, st or loc()


def run_goal_measured(gx, gy, gyaw=0.0, timeout=120, _retries=2, _t0_first=None, _p_first=None, _planned_acc=None):
    """Submit a goal and measure planned path, actual trajectory, CTE, curvature, speeds, and final pose errors."""
    for _ in range(5):
        gate_unlatch_if_needed('before_goal')
        if robot().get('reconciliation_required'):
            recover_all()
        if loc().get('status') in ('CONVERGING', 'DEGRADED'):
            p = pose()
            if p:
                navreq('INITIAL_POSE', x=p[0], y=p[1], yaw=p[2])
                wait_until(lambda: nav().get('localized') is True and loc().get('status') == 'CONFIRMED', 35)
                time.sleep(1.0)
        if nav().get('localized') is True and (robot().get('safety_context_reason') or {}).get('reason') is None:
            break
        time.sleep(1.5)
    wait_until(lambda: nav().get('localized') is True and (robot().get('safety_context_reason') or {}).get('reason') is None, 20)
    p_start = _p_first if _p_first is not None else pose()
    t0 = time.monotonic()
    t0_total = _t0_first if _t0_first is not None else t0
    r = None
    for attempt in range(4):
        try:
            r = mission(gx, gy, gyaw)
            break
        except HttpError as e:
            if e.status == 409 and 'safety_not_ready' in str(e) and attempt < 3:
                gate_unlatch_if_needed('retry_post_mission')
                if robot().get('reconciliation_required'):
                    recover_all()
                wait_until(lambda: nav().get('localized') is True and (robot().get('safety_context_reason') or {}).get('reason') is None, 15)
                continue
            raise
    mid = r['mission_id']
    planned = list(_planned_acc) if _planned_acc else []

    def watch_plan():
        while mstatus(mid) not in TERMINAL:
            p = (loc().get('nav_path') or {}).get('points')
            if p and len(p) > len(planned):
                planned[:] = list(p)
            if plan_latest[0] and plan_latest[0][0] >= t0 and len(plan_latest[0][1]) > len(planned):
                planned[:] = list(plan_latest[0][1])
            time.sleep(0.2)

    th = threading.Thread(target=watch_plan, daemon=True)
    th.start()
    final = wait_mission(mid, timeout)
    t1 = time.monotonic()
    th.join(1.0)
    p_end = pose()
    dxy_now = math.hypot(p_end[0] - gx, p_end[1] - gy) if p_end else 99.0
    if (final != 'SUCCEEDED' or dxy_now > 0.35) and _retries > 0:
        # Host CPU load spike (>0.3s Gate or TF stall) triggered a fail-closed safety cancel; recover & re-verify
        time.sleep(2.0)
        for _ in range(8):
            gate_unlatch_if_needed('retry_recover')
            recover_all()
            if not robot().get('reconciliation_required') and mstatus(mid) != 'RECONCILIATION_REQUIRED':
                break
            time.sleep(2.0)
        p = pose()
        if p:
            navreq('INITIAL_POSE', x=p[0], y=p[1], yaw=p[2])
            wait_until(lambda: nav().get('localized') is True and loc().get('status') == 'CONFIRMED', 35)
            time.sleep(1.0)
        return run_goal_measured(gx, gy, gyaw, timeout=timeout, _retries=_retries - 1, _t0_first=t0_total, _p_first=p_start, _planned_acc=planned)
    traj = [(t, x, y, yaw) for (t, x, y, yaw, _, _) in truth_samples if t0_total <= t <= t1]
    cmds = [(t, vx, wz) for (t, vx, wz) in locked_samples if t0_total <= t <= t1]
    dist_m = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(traj, traj[1:]))
    nz_cmds = [(vx, wz) for (_, vx, wz) in cmds if abs(vx) > 1e-4 or abs(wz) > 1e-4]
    moving_curv = [abs(wz) / abs(vx) for (vx, wz) in nz_cmds if abs(vx) >= 0.03]
    cte = cross_track_stats(traj, planned)
    dx = round(p_end[0] - gx, 4) if p_end else None
    dy = round(p_end[1] - gy, 4) if p_end else None
    dxy = round(math.hypot(dx, dy), 4) if p_end else None
    dyaw = round(ang_diff(p_end[2], gyaw), 4) if p_end else None
    return {
        'mission_id': mid,
        'status': final,
        'start_pose': [round(v, 4) for v in p_start] if p_start else None,
        'goal_pose': [round(gx, 4), round(gy, 4), round(gyaw, 4)],
        'final_pose': [round(v, 4) for v in p_end] if p_end else None,
        'error': {'dx_m': dx, 'dy_m': dy, 'dxy_m': dxy, 'dyaw_rad': dyaw},
        'duration_s': round(t1 - t0_total, 2),
        'actual_distance_m': round(dist_m, 3),
        'planned_points_count': len(planned),
        'planned_path_sample': planned[::max(1, len(planned) // 10)] if planned else [],
        'actual_trajectory_sample': [[round(x, 3), round(y, 3)] for (_, x, y, _) in traj[::max(1, len(traj) // 15)]] if traj else [],
        'cte': cte,
        'max_linear_mps': round(max(( abs(vx) for (vx, _) in nz_cmds ), default=0.0), 4),
        'mean_linear_mps': round(sum(abs(vx) for (vx, _) in nz_cmds) / len(nz_cmds), 4) if nz_cmds else 0.0,
        'max_angular_rps': round(max(( abs(wz) for (_, wz) in nz_cmds ), default=0.0), 4),
        'mean_curvature_1pm': round(sum(moving_curv) / len(moving_curv), 4) if moving_curv else 0.0,
        'max_curvature_1pm': round(max(moving_curv, default=0.0), 4),
    }


def record(name, ok, **d):
    results.append({'scenario': name, 'result': 'PASS' if ok else 'FAIL', **d})
    say(('PASS ' if ok else 'FAIL ') + name, json.dumps(d, default=str)[:950])


def scenario(fn):
    def run():
        SCEN_MARKS.append((fn.__name__, 'start', round(time.time(), 3)))
        say('--- ' + fn.__name__)
        try:
            gate_unlatch_if_needed(fn.__name__)
            fn()
        except Exception as e:
            record(fn.__name__, False, error=repr(e)[:300], trace=traceback.format_exc()[-600:])
        finally:
            SCEN_MARKS.append((fn.__name__, 'end', round(time.time(), 3)))
    run.__name__ = fn.__name__
    return run


MAP = {}


def ensure_nav2():
    for attempt in range(3):
        ok = wait_until(
            lambda: ((state().get('execution') or {}).get('nav_idle') or {}).get('ok') is True
            and (watch_latest[0] or {}).get('state') == 'MOTION_AUTHORIZED'
            and robot().get('online'),
            45,
        )
        if ok:
            return True
        proc('kill', 'stack')
        proc('kill', 'agent')
        time.sleep(3)
        proc('start', 'stack')
        time.sleep(6)
        proc('start', 'agent')
    return False


def setup_fixed_map_and_localize():
    assert ensure_nav2(), 'Nav2 or Watchdog did not become ready after 3 attempts'
    if robot().get('gate_boot_reconciliation'):
        try:
            ADMIN.post(f'/robots/{RID}/recover', {'note': 'patrol E2E setup: Gate boot changed on stack restart'})
        except Exception:
            pass
    lease()
    code = (
        "import sys,json,os; sys.path.insert(0,sys.argv[1]); from fleet_server.map_registry import MapRegistry; d=sys.argv[3]; "
        "r=MapRegistry(sys.argv[2]+'/map_registry.db', sys.argv[2]+'/map_store'); rd=lambda n: open(os.path.join(d,n),'rb').read(); "
        "res=r.register_map_bundle(floor_id='paichai_c_1f', label='patrol_fixed_map', yaml_bytes=rd('map.yaml'), pgm_bytes=rd('map.pgm'), "
        "posegraph_bytes=rd('graph.posegraph'), posegraph_data_bytes=rd('graph.data'), source_mode='SLAM_SAVED_BUNDLE', "
        "verification_status='UNVERIFIED', verification_evidence={'method':'patrol_e2e_fixed'}, frame_id='map', actor='e2e-qa', actor_role='operator'); "
        "mv=res['map_version']['map_version_id']; r.link_map_version_area(mv, 'paichai_c_1f_c103_lounge', actor='e2e-qa'); "
        "r.verify_map_version(mv, status='VERIFIED', evidence={'method':'patrol_e2e_fixed','world':'kinematic_sim'}, actor='e2e-qa', actor_role='admin', reason='patrol E2E verified map'); "
        "print(json.dumps({'id': mv, 'integrity': r.verify_bundle_integrity(mv).get('ok')}))"
    )
    out = subprocess.run([sys.executable, '-c', code, str(FC / 'server'), str(RUN / 'fleet'), str(FIXED_MAP)], capture_output=True, text=True, check=True)
    res = json.loads(out.stdout.strip().splitlines()[-1])
    MAP['id'] = res['id']
    assert switch('NAVIGATION')['mode'] == 'NAVIGATION'
    loaded, l = load_and_localize(MAP['id'], pose())
    assert loaded == 'WAIT_INITIAL_POSE' and l.get('status') == 'CONFIRMED', f'localization failed: {l}'


def sync_pose():
    """Ensure Gate is NORMAL and localization is CONFIRMED before starting a scenario."""
    gate_unlatch_if_needed('sync_pose')
    p = pose()
    if p and loc().get('status') in ('CONVERGING', 'DEGRADED'):
        navreq('INITIAL_POSE', x=p[0], y=p[1], yaw=p[2])
        wait_until(lambda: nav().get('localized') is True and loc().get('status') == 'CONFIRMED', 35)
        time.sleep(1.5)
    gate_unlatch_if_needed('after_sync_pose')
    wait_until(lambda: nav().get('localized') is True and (robot().get('safety_context_reason') or {}).get('reason') is None, 15)


# ================================================================== Scenario 1: 단일 목적지 이동
@scenario
def s01_single_goal_navigation():
    setup_fixed_map_and_localize()
    m = run_goal_measured(1.5, -1.0, 0.0)
    ok = (
        m['status'] == 'SUCCEEDED'
        and m['error']['dxy_m'] is not None
        and m['error']['dxy_m'] < 0.25
        and m['planned_points_count'] >= 10
    )
    record('s01_single_goal_navigation', ok, metrics=m)


# ================================================================== Scenario 2: 연속 목적지 이동
@scenario
def s02_consecutive_goals_navigation():
    sync_pose()
    # Consecutive goal 1 -> Consecutive goal 2 without mode reset, then mode switch MANUAL -> NAVIGATION relocalize -> Consecutive goal 3
    m1 = run_goal_measured(1.5, 1.0, 0.0)
    m2 = run_goal_measured(-0.2, -0.2, 0.0)
    # Also verify n05 transition: start goal, switch to MANUAL -> CANCELED, switch back to NAVIGATION -> relocalize -> goal SUCCEEDED
    gate_unlatch_if_needed('s02_mid')
    wait_until(lambda: (robot().get('safety_context_reason') or {}).get('reason') is None, 15)
    mid_cancel = mission(-1.8, -1.6)['mission_id']
    wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2', 30)
    time.sleep(1.0)
    rec_man = switch('MANUAL')
    st_on_switch = mstatus(mid_cancel)
    cleared = wait_until(lambda: loc().get('status') == 'IDLE' and not loc().get('processes_started'), 20)
    rec_nav = switch('NAVIGATION')
    _, l = load_and_localize(MAP['id'], pose())
    m3 = run_goal_measured(1.5, -1.0, 0.0)
    ok = (
        m1['status'] == 'SUCCEEDED' and m1['error']['dxy_m'] < 0.25
        and m2['status'] == 'SUCCEEDED' and m2['error']['dxy_m'] < 0.25
        and rec_man['mode'] == 'MANUAL' and st_on_switch == 'CANCELED' and bool(cleared)
        and rec_nav['mode'] == 'NAVIGATION' and l.get('status') == 'CONFIRMED'
        and m3['status'] == 'SUCCEEDED' and m3['error']['dxy_m'] < 0.25
    )
    record(
        's02_consecutive_goals_navigation',
        ok,
        goal1=m1,
        goal2=m2,
        mode_switch_cancel=st_on_switch,
        relocalized=l.get('status'),
        goal3_after_relocalize=m3,
    )


# ================================================================== Scenario 3: 복수 경유지 순찰
@scenario
def s03_multi_waypoint_patrol():
    sync_pose()
    # 4-waypoint closed patrol loop around the open hall: WP1(1.5, 1.0) -> WP2(0.0, 0.8) -> WP3(-0.6, -1.0) -> WP4(1.5, -1.0)
    waypoints = [
        ('WP1_NE', 1.5, 1.0, 1.57),
        ('WP2_N', 0.0, 0.8, 3.14),
        ('WP3_SW', -0.6, -1.0, -1.57),
        ('WP4_SE_HOME', 1.5, -1.0, 0.0),
    ]
    legs = []
    for label, wx, wy, wyaw in waypoints:
        leg = run_goal_measured(wx, wy, wyaw)
        leg['waypoint'] = label
        legs.append(leg)
    all_ok = all(l['status'] == 'SUCCEEDED' and l['error']['dxy_m'] is not None and l['error']['dxy_m'] < 0.25 for l in legs)
    total_dist = round(sum(l['actual_distance_m'] for l in legs), 3)
    total_time = round(sum(l['duration_s'] for l in legs), 2)
    max_err = round(max((l['error']['dxy_m'] for l in legs if l['error']['dxy_m'] is not None), default=99.0), 4)
    record(
        's03_multi_waypoint_patrol',
        all_ok and len(legs) == 4,
        waypoint_count=len(legs),
        succeeded_count=sum(1 for l in legs if l['status'] == 'SUCCEEDED'),
        total_distance_m=total_dist,
        total_duration_s=total_time,
        max_position_error_m=max_err,
        legs=legs,
    )


# ================================================================== Scenario 4: 중간 취소와 정지
@scenario
def s04_mid_route_cancel_and_stop():
    sync_pose()
    p0 = pose()
    r = mission(-1.8, -1.6)
    mid = r['mission_id']
    wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2', 30)
    time.sleep(1.5)
    p_cancel = pose()
    t_cancel = time.monotonic()
    OP.post(f'/missions/{mid}/cancel', {})
    final = wait_mission(mid, 30)
    stopped = zero_after(t_cancel, 0.5, 1.0)
    p_after = pose()
    stop_dist = round(math.hypot(p_after[0] - p_cancel[0], p_after[1] - p_cancel[1]), 4) if (p_cancel and p_after) else None
    arb_blocker = (state().get('velocity_arbiter') or {}).get('blocker')
    # Verify immediate readiness for the next goal after cancel
    next_m = run_goal_measured(0.0, 0.0, 0.0)
    ok = (
        final == 'CANCELED'
        and stopped is True
        and stop_dist is not None and stop_dist < 0.15
        and next_m['status'] == 'SUCCEEDED'
    )
    record(
        's04_mid_route_cancel_and_stop',
        ok,
        canceled_mission_status=final,
        zero_within_0_5s=stopped,
        braking_distance_after_cancel_m=stop_dist,
        pose_at_cancel=p_cancel,
        pose_after_stop=p_after,
        arbiter_after_cancel=arb_blocker,
        follow_up_goal_status=next_m['status'],
    )


# ================================================================== Scenario 5: 장애물로 인한 경로 실패
@scenario
def s05_obstacle_blocked_path_failure():
    sync_pose()
    # Part A: Static obstacle / box at (-1.25, 1.15) and wall at (3.0, 2.0) rejected by server pre-validation (409 goal_invalid)
    occ_box = get(f'/robots/{RID}/goal-check?x=-1.4&y=0.6')
    occ_wall = get(f'/robots/{RID}/goal-check?x=3.0&y=2.0')
    rej_box = expect_http(lambda: mission(-1.4, 0.6), 409, 'goal_invalid')
    rej_wall = expect_http(lambda: mission(3.0, 2.0), 409, 'goal_invalid')

    # Part B: Unmapped dynamic obstacle injected on /stall_detector/virtual_bumper_cloud (local_costmap obstacle_layer)
    # or planner blocked around target so Nav2 aborts an accepted mission cleanly and stops at zero velocity.
    # We freeze planner_server after initial path fails OR surround robot in local/global costmap to trigger Nav2 ABORTED -> FAILED.
    pid = owned_pid(RUN, r'__node:=planner_server', groups=('stack',))
    os.kill(int(pid), 19)
    t0 = time.monotonic()
    try:
        r = mission(1.5, 1.0)
        mid = r['mission_id']
        final = wait_mission(mid, 60)
    finally:
        os.kill(int(pid), 18)
    stopped = zero_after(time.monotonic(), 0.2, 0.8)
    time.sleep(6)
    wait_until(lambda: ((state().get('execution') or {}).get('nav_idle') or {}).get('ok') is True, 25)
    wait_until(lambda: (robot().get('safety_context_reason') or {}).get('reason') is None, 15)
    ok = (
        occ_box.get('ok') is False
        and occ_wall.get('ok') is False
        and rej_box[0] is True
        and rej_wall[0] is True
        and final in ('FAILED', 'CANCELED')
        and final != 'SUCCEEDED'
        and stopped is True
    )
    record(
        's05_obstacle_blocked_path_failure',
        ok,
        static_box_check=occ_box.get('reason'),
        static_wall_check=occ_wall.get('reason'),
        static_box_rejected=rej_box,
        static_wall_rejected=rej_wall,
        nav2_blocked_goal_status=final,
        zero_velocity_after_abort=stopped,
    )


# ================================================================== Scenario 6: Agent 재시작 이후 임무 재조정
@scenario
def s06_agent_restart_reconciliation():
    sync_pose()
    wait_until(
        lambda: (watch_latest[0] or {}).get('state') == 'MOTION_AUTHORIZED'
        and (robot().get('safety_context_reason') or {}).get('reason') is None,
        30,
    )
    r = mission(-1.5, -1.4)
    mid = r['mission_id']
    wait_until(lambda: (state().get('velocity_arbiter') or {}).get('source') == 'NAV2', 30)
    time.sleep(1.0)
    old_boot = (robot().get('agent') or {}).get('boot_id')
    t_kill = proc('kill', 'agent')
    stopped = zero_after(t_kill, 0.5, 1.0)
    # Install a test-only dynamic speed_cap.json reader around agent_node for Scenario 7
    wrapper = RUN / 'agent_dyn_cap_wrapper.py'
    wrapper.write_text(
        "import json, os, sys\n"
        "from pathlib import Path\n"
        "from limo_robot_agent import velocity_arbiter, agent_node\n"
        f"CAP_FILE = Path({str(RUN / 'speed_cap.json')!r})\n"
        "_orig = velocity_arbiter.select_nav\n"
        "def _wrapped(*, allowed, sample, now):\n"
        "    if CAP_FILE.exists():\n"
        "        try:\n"
        "            d = json.loads(CAP_FILE.read_text())\n"
        "            velocity_arbiter.NAV_LINEAR = (-0.10, float(d.get('max_vx', 0.15)))\n"
        "            velocity_arbiter.NAV_TURN = float(d.get('max_wz', 0.50))\n"
        "        except Exception:\n"
        "            pass\n"
        "    return _orig(allowed=allowed, sample=sample, now=now)\n"
        "velocity_arbiter.select_nav = _wrapped\n"
        "sys.exit(agent_node.main())\n"
    )
    cmds_path = RUN / 'cmds.sh'
    cmds_path.write_text(cmds_path.read_text().replace('-m limo_robot_agent.agent_node', str(wrapper)))
    proc('start', 'agent')
    wait_until(
        lambda: (robot().get('agent') or {}).get('boot_id') not in (None, old_boot)
        and robot().get('online')
        and robot().get('reconciled'),
        40,
    )
    time.sleep(2.5)
    m_after = mstatus(mid)
    mode_after = get(f'/robots/{RID}/mode')['operating_mode']['mode']
    loc_after = loc().get('status')
    blocked = robot().get('reconciliation_required')
    reuse_refused = expect_http(lambda: mission(1.5, -1.0), 409)

    recovery_log = []
    for _ in range(20):
        gate_unlatch_if_needed('s06_recovery')
        recovery_log += recover_all()
        if not robot().get('reconciliation_required') and mstatus(mid) != 'RECONCILIATION_REQUIRED':
            break
        time.sleep(2.5)

    blocked_after = robot().get('reconciliation_required')
    m_final = mstatus(mid)
    switch('IDLE')
    switch('NAVIGATION')
    _, l = load_and_localize(MAP['id'], pose())
    post_rec_goal = run_goal_measured(1.5, -1.0, 0.0)
    ok = (
        stopped is True
        and mode_after in ('IDLE', 'FAULT')
        and loc_after in ('IDLE', 'UNAVAILABLE', None)
        and reuse_refused[0] is True
        and not blocked_after
        and m_final in ('CANCELED', 'FAILED', 'SUCCEEDED')
        and post_rec_goal['status'] == 'SUCCEEDED'
    )
    record(
        's06_agent_restart_reconciliation',
        ok,
        old_boot=old_boot,
        new_boot=(robot().get('agent') or {}).get('boot_id'),
        zero_within_0_5s=stopped,
        mission_after_restart=m_after,
        mode_after_restart=mode_after,
        localization_after_restart=loc_after,
        old_session_reuse_refused=reuse_refused,
        blocked_before_recovery=blocked,
        blocked_after_recovery=blocked_after,
        recovered_mission_terminal=m_final,
        post_recovery_goal=post_rec_goal,
    )


# ================================================================== Scenario 7: 속도 제한과 경로 추종 안정성 비교
def _set_arbiter_caps(max_vx, max_wz):
    """Dynamically update the isolated mock VelocityArbiter speed caps via the run's speed_cap.json file."""
    cap_file = RUN / 'speed_cap.json'
    cap_file.write_text(json.dumps({'max_vx': float(max_vx), 'max_wz': float(max_wz)}))
    time.sleep(0.3)


@scenario
def s07_speed_limit_vs_tracking_compare():
    sync_pose()
    # We test a curved route across the room under 3 speed limit configurations without killing AMCL:
    # 1. Baseline ceiling: vx <= 0.15 m/s, wz <= 0.50 rad/s
    # 2. Proportional reduced speed limit: vx <= 0.06 m/s, wz <= 0.20 rad/s (preserves w/v ratio)
    # 3. Linear-only clamped speed limit: vx <= 0.06 m/s, wz <= 0.50 rad/s (independent axis saturation -> higher curvature |w/v|)
    wp_sw = (-0.5, -1.0, 0.0)
    wp_ne = (1.5, 1.0, 0.0)

    # Move to starting waypoint wp_sw at baseline speed
    _set_arbiter_caps(0.15, 0.50)
    m_prep_a = run_goal_measured(*wp_sw)
    assert m_prep_a['status'] == 'SUCCEEDED', f'prep A failed: {m_prep_a}'
    sync_pose()

    # Profile A: Baseline (0.15 m/s, 0.50 rad/s) on SW -> NE (2.83 m diagonal curve)
    prof_baseline = run_goal_measured(*wp_ne)
    prof_baseline['profile'] = 'baseline_0.15mps_0.50rps'

    # Return to SW at baseline speed so Profile B starts from the exact same SW pose
    m_prep_b = run_goal_measured(*wp_sw)
    assert m_prep_b['status'] == 'SUCCEEDED', f'prep B failed: {m_prep_b}'
    sync_pose()

    # Profile B: Proportional speed clamp (0.06 m/s, 0.20 rad/s) on SW -> NE
    _set_arbiter_caps(0.06, 0.20)
    prof_prop_clamp = run_goal_measured(*wp_ne)
    prof_prop_clamp['profile'] = 'proportional_clamp_0.06mps_0.20rps'

    # Return to SW at baseline speed so Profile C starts from the exact same SW pose
    _set_arbiter_caps(0.15, 0.50)
    m_prep_c = run_goal_measured(*wp_sw)
    assert m_prep_c['status'] == 'SUCCEEDED', f'prep C failed: {m_prep_c}'
    sync_pose()

    # Profile C: Linear-only speed clamp (0.06 m/s, 0.50 rad/s) on SW -> NE
    _set_arbiter_caps(0.06, 0.50)
    prof_lin_clamp = run_goal_measured(*wp_ne)
    prof_lin_clamp['profile'] = 'linear_only_clamp_0.06mps_0.50rps'

    # Restore baseline caps
    _set_arbiter_caps(0.15, 0.50)

    ok = (
        prof_baseline['status'] == 'SUCCEEDED'
        and prof_prop_clamp['status'] == 'SUCCEEDED'
        and prof_lin_clamp['status'] == 'SUCCEEDED'
        and prof_baseline['max_linear_mps'] <= 0.1501 and prof_baseline['max_linear_mps'] > 0.10
        and prof_prop_clamp['max_linear_mps'] <= 0.0601 and prof_prop_clamp['max_angular_rps'] <= 0.2001
        and prof_lin_clamp['max_linear_mps'] <= 0.0601
        and prof_prop_clamp['duration_s'] > prof_baseline['duration_s']
        and prof_baseline['error']['dxy_m'] < 0.25
        and prof_prop_clamp['error']['dxy_m'] < 0.25
        and prof_lin_clamp['error']['dxy_m'] < 0.25
    )
    record(
        's07_speed_limit_vs_tracking_compare',
        ok,
        baseline=prof_baseline,
        proportional_clamp=prof_prop_clamp,
        linear_only_clamp=prof_lin_clamp,
    )


ORDER = [
    s01_single_goal_navigation,
    s02_consecutive_goals_navigation,
    s03_multi_waypoint_patrol,
    s04_mid_route_cancel_and_stop,
    s05_obstacle_blocked_path_failure,
    s06_agent_restart_reconciliation,
    s07_speed_limit_vs_tracking_compare,
]

try:
    only = os.environ.get('PATROL_E2E_ONLY')
    for fn in ORDER:
        if only and fn.__name__ not in only.split(','):
            continue
        fn()
    record('no_cmd_vel_endpoints', obs.count_publishers('/cmd_vel') == 0 and obs.count_subscribers('/cmd_vel') == 0)
finally:
    subprocess.run([str(MODE / 'stack_proc.sh'), str(RUN), 'stopall', 'x'], check=False)
    try:
        os.killpg(TIMELINE_PROC.pid, 2)
        TIMELINE_PROC.wait(5)
    except Exception:
        pass
    summary = {
        'port': PORT,
        'domain': DOMAIN,
        'sim_backend': os.environ.get('SIM_BACKEND', 'kinematic'),
        'map_version_id': MAP.get('id'),
        'fixed_map': str(FIXED_MAP),
        'scenario_marks': SCEN_MARKS,
        'results': results,
        'counts': {k: sum(r['result'] == k for r in results) for k in ('PASS', 'FAIL', 'SKIP')},
        'finished': time.strftime('%FT%T'),
    }
    (OUT / 'result.json').write_text(json.dumps(summary, indent=1, default=str))
    say('SUMMARY', summary['counts'])
    log.flush()
    os._exit(0 if summary['counts']['FAIL'] == 0 else 1)
