"""SIMULATION ONLY (2026-11-22, Claude): targeted Nav2 D3 scenarios in Gazebo, same conditions for each Nav2 parameter profile.

D3 = RPP drives a forward arc toward a goal behind it / next to a wall and stops inside robot_radius (p5_r13_a, gz_r13_a).
Each CASE is a fresh, isolated stack (ROS_LOCALHOST_ONLY=1, private domain, own IGN_PARTITION; no LIMO, no 18190, no /cmd_vel):
  gazebo_world.launch.py (Office world + a test corridor, derived copy; the original world file is read only) spawned at the case
  start pose -> web_mapping_isolated/stack.launch.py (Nav2 shared slow profile + NAV2_PARAMS_OVERLAY, Gate r13 candidate, ARBITER=0,
  WATCHDOG=0: velocity_smoother -> /cmd_vel_raw -> Gate -> /cmd_vel_locked -> drive timeout relay -> Gazebo DiffDrive).
Localization FIXTURE: odom = Gazebo truth (/ground_truth/odom via odom_tf_relay), map->odom identity, rolling global costmap without a
static map. This removes D1 (ICP rotation) and map quality, so only planner/controller/costmap behaviour is compared.
Recorded per case (case.json + streams.jsonl): Nav2 result, truth trajectory (world), minimum footprint clearance from the LiDAR
(beam end points -> footprint rectangle 0.29 x 0.23 m; LiDAR sees VISUAL geometry only, D2 caveat), commands on /fleet/cmd_vel_nav,
/cmd_vel_raw (smoothed) and /cmd_vel_locked, Gate state, "blocked" time (locked |vx| > 0.04 with truth speed < 0.01 for > 0.5 s =
contact indicator), in-place rotation time, Nav2 log events (planner failures, Collision Ahead, RPP collision). Collision Monitor:
not deployed in this stack (recorded as N/A).

usage: python3 nav2_d3_targeted.py --out <new dir> --profile <name> [--overlay <yaml>] [--cases a,b] [--repeat 3] [--domain 200]
"""
import argparse, json, math, os, re, signal, subprocess, sys, threading, time
from pathlib import Path

HERE = Path(__file__).resolve().parent; FC = HERE.parents[1]; REPO = FC.parent
MAIN_REPO = REPO.parent / 'Capstone1_limo'
def _repo_asset(rel):
    p = REPO / rel
    return p if p.exists() else (MAIN_REPO / rel)
WORLD_SRC = _repo_asset('experiments/office_limo_sim_20261113/office_limo.world')
GATE_R13 = _repo_asset('local_data/raw_results/gate_r13_integ_claude_20261122/candidate_r13/stall_detector_node.py')
SPAWN = (10.09, -6.98)                      # Office patrol_A1; every pose below is in this SPAWN frame (yaw 0), converted to world
FOOT_X, FOOT_Y = 0.145, 0.115               # limo_eval_diff body + wheels half extents (model.sdf)
LASER_X = 0.103
CORRIDOR = {'x0': -3.0, 'x1': -1.2, 'y_lo': -0.13, 'y_hi': 0.53}   # free area in the 2026-11-22 survey scan; 0.66 m wide

#           start (x, y, yaw)              goal (x, y)      notes
CASES = {
    'behind_goal':   ((-0.30, -0.45, -math.pi / 2), (-0.70, 0.90), 'parked facing the cubicle wall (y -0.85), goal behind-left (p5 v04 geometry)'),
    'corridor_uturn': ((-1.60, 0.20, 0.0), (-2.70, 0.20), '0.66 m test corridor, goal behind: needs an in-place U-turn'),
    'wall_oblique':  ((-0.90, 0.30, 0.0), (0.60, -0.42), 'goal 0.43 m from the cubicle wall face, approached on a curve (gz_r13_a geometry)'),
    'adjacent_start': ((0.20, -0.62, 0.0), (-0.70, 0.90), 'start with the wall 0.11 m from the body side, goal behind-left'),
    'cancel_mid':    ((-0.80, 1.60, math.pi), (-3.20, 1.60), 'straight goal, cancelled after 2.5 s of motion'),
}
LOG_EVENTS = {'planner_fail': r'failed to create plan', 'collision_ahead': r'Collision Ahead', 'rpp_collision': r'(?i)detected collision ahead',
              'progress_fail': r'(?i)failed to make progress', 'spin': r'Running spin', 'backup': r'Running backup',
              'controller_abort': r'(?i)controller.*(abort|fail)'}


def world_with_corridor(dst):
    s = WORLD_SRC.read_text(); i = s.rindex('</world>'); cx = (CORRIDOR['x0'] + CORRIDOR['x1']) / 2; L = CORRIDOR['x1'] - CORRIDOR['x0']
    walls = ''
    for k, y in (('s', CORRIDOR['y_lo']), ('n', CORRIDOR['y_hi'])):
        geo = f'<geometry><box><size>{L:.3f} 0.03 0.5</size></box></geometry>'
        walls += (f'<model name="d3_corridor_{k}"><static>true</static><pose>{SPAWN[0] + cx:.3f} {SPAWN[1] + y:.3f} 0.25 0 0 0</pose>'
                  f'<link name="l"><collision name="c">{geo}</collision><visual name="v">{geo}</visual></link></model>\n')
    dst.write_text(s[:i] + walls + s[i:])


def to_world(p): return (SPAWN[0] + p[0], SPAWN[1] + p[1], p[2] if len(p) > 2 else 0.0)


def ang(a): return math.atan2(math.sin(a), math.cos(a))


def foot_dist(px, py):
    dx = max(abs(px) - FOOT_X, 0.0); dy = max(abs(py) - FOOT_Y, 0.0); return math.hypot(dx, dy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True); ap.add_argument('--profile', required=True); ap.add_argument('--overlay', default='')
    ap.add_argument('--cases', default=','.join(CASES)); ap.add_argument('--repeat', type=int, default=3)
    ap.add_argument('--domain', default='200'); ap.add_argument('--timeout', type=float, default=90.0)
    a = ap.parse_args()
    assert 200 <= int(a.domain) <= 209, 'domains 200-209 are reserved for this harness (side-session note)'
    out = Path(a.out); out.mkdir()                     # never an existing directory
    world = out / 'office_limo_d3.world'; world_with_corridor(world)
    fixture = _repo_asset('local_data/raw_results/nav2_d3_claude_20261122/overlays/d3_targeted_truth_localization.yaml')
    import yaml
    merged = yaml.safe_load(fixture.read_text())
    if a.overlay:
        def deep(d, s):
            for k, v in s.items(): d[k] = deep(d.get(k, {}), v) if isinstance(v, dict) and isinstance(d.get(k), dict) else v
            return d
        merged = deep(merged, yaml.safe_load(Path(a.overlay).read_text()))
    ovl = out / 'nav2_overlay_merged.yaml'; ovl.write_text(yaml.safe_dump(merged, sort_keys=False))
    summary = []
    for r in range(a.repeat):
        for name in a.cases.split(','):
            res = run_case(out / f'{name}_r{r}', name, a, world, ovl)
            summary.append(res); print(json.dumps({k: res.get(k) for k in ('case', 'result', 'min_clear_m', 'blocked_s', 'final_err_m', 'duration_s')}), flush=True)
    (out / 'SUMMARY.json').write_text(json.dumps({'profile': a.profile, 'overlay': a.overlay, 'cases': summary}, indent=1))


def run_case(d, name, a, world, ovl):
    d.mkdir(); start, goal, note = CASES[name]; ws = to_world(start)
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=a.domain, IGN_PARTITION=f'd3_{a.domain}_{os.getpid()}_{d.name}', IGN_IP='127.0.0.1',
               GZ_SPAWN=f'{ws[0]},{ws[1]},{ws[2]}', GZ_WORLD=str(world), GZ_DRIVE_TIMEOUT_S='0.2', USE_SIM_TIME='1', ARBITER='0', WATCHDOG='0',
               ODOM_ACTIVE_TOPIC='/ground_truth/odom', LIFECYCLE_DELAY_S='5', GATE_SCRIPT=str(GATE_R13), NAV2_PARAMS_OVERLAY=str(ovl),
               ROS_LOG_DIR=str(d / 'roslog'), ROS2CLI_NO_DAEMON='1')
    for k in ('ROS_DISCOVERY_SERVER', 'GATE_INPUT_EXCL', 'GATE_WRITER_STATUS', 'GATE_RESET_PEER', 'GATE_REQ_WD'): env.pop(k, None)
    procs = []
    def start_p(tag, cmd):
        p = subprocess.Popen(cmd, env=env, stdout=(d / f'{tag}.log').open('x'), stderr=subprocess.STDOUT, start_new_session=True); procs.append(p)
    start_p('sim', ['ros2', 'launch', str(HERE / 'gazebo_world.launch.py')])
    time.sleep(4)
    start_p('stack', ['ros2', 'launch', str(FC / 'tests/web_mapping_isolated/stack.launch.py')])
    start_p('tf', ['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', 'map', '--child-frame-id', 'odom', '--ros-args', '-p', 'use_sim_time:=true'])
    os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=a.domain); os.environ.pop('ROS_DISCOVERY_SERVER', None)
    res = {'case': name, 'run': d.name, 'profile': a.profile, 'note': note, 'start_spawn_frame': start, 'goal_spawn_frame': goal,
           'collision_monitor': 'N/A (not deployed in this stack)', 'localization': 'FIXTURE: truth odom, map->odom identity'}
    try:
        res.update(drive(d, name, start, goal, a.timeout))
    except Exception as e:
        res.update(result='HARNESS_ERROR', error=repr(e)[:300])
    finally:
        for p in reversed(procs):
            try: os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError: pass
        time.sleep(4)
        for p in procs:
            try: os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError: pass
    log = (d / 'stack.log').read_text(errors='replace') if (d / 'stack.log').exists() else ''
    res['log_events'] = {k: len(re.findall(v, log)) for k, v in LOG_EVENTS.items()}
    (d / 'case.json').write_text(json.dumps(res, indent=1))
    return res


def drive(d, name, start, goal, timeout):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from nav2_msgs.action import NavigateToPose
    rclpy.init(); n = rclpy.create_node('d3_targeted_observer')
    S = {'t0': None, 'gate': None, 'truth': None, 'locked': (0.0, 0.0), 'clear': None}
    rows = (d / 'streams.jsonl').open('x'); lock = threading.Lock()
    def rec(k, **v):
        with lock: rows.write(json.dumps({'k': k, 't': round(time.monotonic(), 4), **v}) + '\n')
    def on_truth(m):
        p = m.pose.pose; q = p.orientation; y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        S['truth'] = (p.position.x, p.position.y, y, time.monotonic()); rec('truth', x=round(p.position.x, 4), y=round(p.position.y, 4), th=round(y, 4))
    def on_scan(m):
        best, front = 9.9, 9.9
        for i, r in enumerate(m.ranges):
            if not (m.range_min <= r <= m.range_max): continue
            a_ = m.angle_min + i * m.angle_increment; px, py = LASER_X + r * math.cos(a_), r * math.sin(a_)
            best = min(best, foot_dist(px, py))
            if abs(a_) < 0.35: front = min(front, r)
        S['clear'] = best; rec('scan', clear=round(best, 3), front=round(front, 3))
    def tw(k):
        def f(m):
            if k == 'locked': S['locked'] = (m.linear.x, m.angular.z)
            rec(k, vx=round(m.linear.x, 4), wz=round(m.angular.z, 4))
        return f
    def on_gate(m):
        try: g = json.loads(m.data)
        except Exception: return
        st = g.get('state') or g.get('status')
        if st != S['gate']: rec('gate', state=st, reasons=g.get('fault_reasons'))
        S['gate'] = st
    n.create_subscription(Odometry, '/ground_truth/world_odom', on_truth, 20)
    from rclpy.qos import qos_profile_sensor_data
    n.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    for k, t in (('nav_raw', '/fleet/cmd_vel_nav'), ('smoothed', '/cmd_vel_raw'), ('locked', '/cmd_vel_locked')):
        n.create_subscription(Twist, t, tw(k), 20)
    n.create_subscription(String, '/stall_detector/status_json', on_gate, 10)
    ex = SingleThreadedExecutor(); ex.add_node(n); th = threading.Thread(target=ex.spin, daemon=True); th.start()
    out = {}
    try:
        ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
        t_end = time.monotonic() + 150
        while time.monotonic() < t_end and not (S['gate'] == 'NORMAL' and S['truth'] and S['clear'] is not None and ac.server_is_ready()):
            time.sleep(.5)
        # 2026-11-22 (tg_base_a wall_oblique_r2 GOAL_REJECTED): the action server exists before the lifecycle manager has ACTIVATED
        # bt_navigator; wait for lifecycle_manager_navigation/is_active as well (harness readiness only).
        from std_srvs.srv import Trigger
        cli = n.create_client(Trigger, '/lifecycle_manager_navigation/is_active'); active = False
        while time.monotonic() < t_end and not active:
            if cli.wait_for_service(timeout_sec=1.0):
                f = cli.call_async(Trigger.Request()); w_end = time.monotonic() + 3
                while not f.done() and time.monotonic() < w_end: time.sleep(.05)
                active = bool(f.done() and f.result() and f.result().success)
            if not active: time.sleep(1.0)
        out['nav2_active'] = active
        out['ready_s'] = round(150 - (t_end - time.monotonic()), 1); out['gate_at_start'] = S['gate']
        if not ac.server_is_ready() or S['gate'] != 'NORMAL':
            out['result'] = 'NOT_READY'; return out
        time.sleep(4.0)                                  # costmaps receive a few scans
        sx, sy, sth = S['truth'][:3]; out['start_world'] = [round(sx, 3), round(sy, 3), round(sth, 3)]; out['start_clear_m'] = round(S['clear'], 3)
        g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'
        gx, gy = goal[0] - start[0], goal[1] - start[1]; c, s = math.cos(-start[2]), math.sin(-start[2])   # spawn frame -> case odom frame
        g.pose.pose.position.x, g.pose.pose.position.y = c * gx - s * gy, s * gx + c * gy; g.pose.pose.orientation.w = 1.0
        rec('goal_sent'); t0 = time.monotonic(); S['t0'] = t0
        fut = ac.send_goal_async(g)
        while not fut.done() and time.monotonic() - t0 < 10: time.sleep(.05)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted: out['result'] = 'GOAL_REJECTED'; return out
        rfut = gh.get_result_async(); cancel_t = None; moving_since = None
        while not rfut.done() and time.monotonic() - t0 < timeout:
            if name == 'cancel_mid' and cancel_t is None:
                if abs(S['locked'][0]) > 0.05: moving_since = moving_since or time.monotonic()
                if moving_since and time.monotonic() - moving_since > 2.5:
                    cancel_t = time.monotonic(); rec('cancel_sent'); gh.cancel_goal_async()
            time.sleep(.05)
        status = rfut.result().status if rfut.done() else None
        out['result'] = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(status, f'STATUS_{status}' if status is not None else 'TIMEOUT')
        if not rfut.done():
            gh.cancel_goal_async(); time.sleep(2)
        out['duration_s'] = round(time.monotonic() - t0, 1); out['cancel_t'] = cancel_t
        time.sleep(3.0)
        out['goal_world'] = [round(SPAWN[0] + goal[0], 3), round(SPAWN[1] + goal[1], 3)]
        if S['truth']: out['final_err_m'] = round(math.hypot(S['truth'][0] - out['goal_world'][0], S['truth'][1] - out['goal_world'][1]), 3)
    finally:
        ex.shutdown(); n.destroy_node(); rclpy.shutdown(); th.join(timeout=5); rows.close()
    out.update(analyse(d / 'streams.jsonl', out.get('cancel_t')))
    return out


def analyse(path, cancel_t):
    R = [json.loads(l) for l in path.open() if l.strip()]
    t0 = next((r['t'] for r in R if r['k'] == 'goal_sent'), None)
    if t0 is None: return {}
    run = [r for r in R if r['t'] >= t0]
    clears = [r['clear'] for r in run if r['k'] == 'scan']
    truth = [r for r in run if r['k'] == 'truth']; locked = [r for r in run if r['k'] == 'locked']
    path_len = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y']) for a, b in zip(truth, truth[1:]))
    spd = []                                           # truth speed over 0.2 s windows
    j = 0
    for i, b in enumerate(truth):
        while truth[j]['t'] < b['t'] - 0.2: j += 1
        a = truth[j]; dt = b['t'] - a['t']
        if dt > 0.1: spd.append((b['t'], math.hypot(b['x'] - a['x'], b['y'] - a['y']) / dt))
    def speed_at(t):
        best = None
        for tt, v in spd:
            if tt <= t: best = v
            else: break
        return best
    blocked = 0.0; run_b = None; inplace = 0.0; arc_max_w = 0.0; prev = None
    for r in locked:
        if prev is not None:
            dt = min(r['t'] - prev['t'], 0.2); v = speed_at(r['t'])
            if abs(prev['vx']) > 0.04 and v is not None and v < 0.01:
                run_b = (run_b or 0) + dt
                if run_b > 0.5: blocked += dt
            else: run_b = None
            if abs(prev['vx']) < 0.02 and abs(prev['wz']) > 0.1: inplace += dt
            if abs(prev['vx']) > 0.05: arc_max_w = max(arc_max_w, abs(prev['wz']))
        prev = r
    gates = [r['state'] for r in run if r['k'] == 'gate']
    out = {'min_clear_m': round(min(clears), 3) if clears else None, 'path_m': round(path_len, 2), 'blocked_s': round(blocked, 2),
           'inplace_rot_s': round(inplace, 1), 'max_wz_while_driving': round(arc_max_w, 2), 'gate_states': gates,
           'max_locked_vx': round(max((abs(r['vx']) for r in locked), default=0), 3)}
    moving = [r['clear'] for r in run if r['k'] == 'scan' and (speed_at(r['t']) or 0) > 0.02]
    out['min_clear_moving_m'] = round(min(moving), 3) if moving else None
    if cancel_t:
        after = [r for r in locked if r['t'] >= cancel_t]
        z = next((r['t'] for r in after if abs(r['vx']) < 1e-6 and abs(r['wz']) < 1e-6), None)
        out['cancel_zero_s'] = round(z - cancel_t, 3) if z else None
        out['nonzero_after_zero'] = sum(1 for r in after if z and r['t'] > z and (abs(r['vx']) > 1e-6 or abs(r['wz']) > 1e-6))
        # 2026-11-22: 'nonzero_after_zero' counts the controller's last in-flight command when the cancel happens to land on a momentary
        # zero (tg_o2_a cancel_mid_r2); the settled measure is the time of the LAST non-zero locked output after the cancel.
        nz = [r for r in after if abs(r['vx']) > 1e-6 or abs(r['wz']) > 1e-6]
        out['cancel_last_nonzero_s'] = round(nz[-1]['t'] - cancel_t, 3) if nz else 0.0; out['cancel_nonzero_samples'] = len(nz)
        st = next((tt for tt, v in spd if tt >= cancel_t and v < 0.01), None); out['cancel_truth_stop_s'] = round(st - cancel_t, 2) if st else None
    return out


if __name__ == '__main__':
    main()
