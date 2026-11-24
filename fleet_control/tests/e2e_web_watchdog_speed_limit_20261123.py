#!/usr/bin/env python3
"""Full-stack speed-limit E2E: console API -> Fleet Server -> Robot Agent -> Motion Watchdog -> Safety Gate (isolated ROS, G2/integration 2026-11-23).
Not a real-robot result.

Stack: scripts/run_isolated_fleet.sh with FLEET_FLAGS='speed_clamp:=true manual:=false velocity:=false' (the real launch's shape: no Arbiter, Watchdog
speed clamp on). Real processes: Fleet Server, Robot Agent (mock/simulated) with WatchdogSpeedLink, Motion Watchdog, Safety Gate (stall_detector_node,
output remapped to /cmd_vel_locked), Nav2 velocity_smoother, synthetic sensors. The measurement point is the Gate output; there is no motor driver in
this profile (the real launch's driver remap and Mock driver are covered by real_launch_chain_faults_20261123.py).

LIMITS OF THIS E2E (found while building it):
  * The non-zero command is a STAND-IN for the Nav2 controller: a test node publishes into the smoother input /fleet/cmd_vel_nav, so the real smoother,
    Watchdog and Gate carry it. Operating modes (MAPPING/NAVIGATION/MANUAL) cannot be entered in this profile: the server's mode confirmation requires an
    Agent velocity selector (Arbiter), which only mock Agents have ("agent has no velocity selector to confirm MAPPING"). So no web Mapping/Navigation goal
    exists here, a Lease release does not cancel any motion (there is no goal), and Nav2 path following under a limit is not measured (the synthetic robot
    does not move).
usage: e2e_web_watchdog_speed_limit_20261123.py RUN_DIR PORT      (overlay sourced, ROS_DOMAIN_ID as the stack; optional E2E_HOLD=name1,name2 pauses
       at those points until the file RUN_DIR/HOLD_name is removed, so a browser can be pointed at the live console)
Exit 0 = every check passed."""
import atexit, json, os, signal, subprocess, sys, threading, time, uuid
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

CHILDREN = []


@atexit.register
def _cleanup():
    for p in CHILDREN:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except Exception:
            pass


def main():
    run, port = Path(sys.argv[1]), int(sys.argv[2])
    token = (run / 'secrets/operator.token').read_text().strip()
    base = f'http://127.0.0.1:{port}/api/v1/robots/locked_bench'
    holds = set(filter(None, os.environ.get('E2E_HOLD', '').split(',')))
    res, log = [], []

    def check(name, ok, detail=''):
        res.append(bool(ok)); line = ('PASS ' if ok else 'FAIL ') + name + (f' | {detail}' if detail else ''); print(line, flush=True); log.append(line)

    def call(path, data=None):
        req = Request(base + path, None if data is None else json.dumps(data).encode(), headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'})
        try:
            with urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except HTTPError as e:
            return e.code, {**json.loads(e.read() or b'{}')}
        except Exception as e:
            return 0, {'error': {'code': 'unreachable', 'message': str(e)}}

    def wait(pred, secs=30, step=.25):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            v = pred()
            if v: return v
            time.sleep(step)
        return None

    def hold(name):
        if name in holds:
            f = run / f'HOLD_{name}'; f.write_text('remove this file to continue'); print(f'   HOLD {name}: waiting for {f} to be removed', flush=True)
            while f.exists(): time.sleep(1.0)

    import rclpy
    from geometry_msgs.msg import Twist
    rclpy.init(); node = rclpy.create_node('e2e_probe'); rows = []
    node.create_subscription(Twist, '/cmd_vel_locked', lambda m: rows.append((time.monotonic(), m.linear.x, m.angular.z)), 50)
    ctl = {'on': False, 'v': 0.30, 'w': 0.0}
    pub = node.create_publisher(Twist, '/fleet/cmd_vel_nav', 10)
    def tick():
        if ctl['on']:
            m = Twist(); m.linear.x, m.angular.z = ctl['v'], ctl['w']; pub.publish(m)
    node.create_timer(0.05, tick)
    from std_msgs.msg import String
    gate = {}
    node.create_subscription(String, '/stall_detector/status_json', lambda m: gate.update(json.loads(m.data)), 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def peak(t0, t1=None):
        nz = [r for r in rows if r[0] >= t0 and (t1 is None or r[0] <= t1) and (abs(r[1]) > 1e-6 or abs(r[2]) > 1e-6)]
        return (max([r[1] for r in nz] or [0.0]), min([r[1] for r in nz] or [0.0]), max([abs(r[2]) for r in nz] or [0.0]), len(nz))

    ep = {'e': None}
    def renew(): call('/lock', dict(action='renew', ttl_s=300))
    def robot(): return call('')[1]
    def state(): return robot().get('state') or {}

    def flow(secs=6.0, v=0.30, w=0.0):
        ctl.update(on=True, v=v, w=w); time.sleep(1.2); t0 = time.monotonic(); end = t0 + secs
        while time.monotonic() < end:
            renew(); time.sleep(1.0)
        return t0, peak(t0)

    def post_limit(scope, lin, ang, **kw):
        renew()
        b = dict(lease_epoch=ep['e'], agent_boot_id=robot()['agent']['boot_id'], protocol_version=2, safety_contract_version=1, scope=scope,
                 max_linear_mps=lin, max_angular_rps=ang, client_request_id=uuid.uuid4().hex[:16]); b.update(kw)
        return call('/speed-limits', b)

    def get_view(): return call('/speed-limits')[1]

    def approved(gen, secs=30):
        def f():
            renew(); v = get_view(); return v if v.get('generation') == gen and v.get('approved') is True else None
        return wait(f, secs, .5)

    def procs(key):
        out = subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True).stdout.splitlines()
        found = []
        for l in out:
            parts = l.split(None, 2)
            # the executable path must END with the name: 'limo_robot_agent' appears in every limo_robot_agent executable's path
            if len(parts) >= 2 and 'e2e_web_watchdog' not in l and 'ros2 run' not in l and (parts[1].endswith('/' + key) or (len(parts) == 3 and parts[2].split()[0].endswith('/' + key))):
                found.append((int(parts[0]), l.split(None, 1)[1]))
        return [p for p in found if 'robot_id:=second' not in p[1]]

    def save_params(key, tag):
        pid, args = procs(key)[0]; out = []
        for i, f in enumerate([a for a in args.split() if a.endswith('.yaml') or a.startswith('/tmp/launch_params_')]):
            d = run / f'{tag}_params_{i}.yaml'
            if not d.exists(): d.write_bytes(Path(f).read_bytes())
            out.append(str(d))
        return out

    def spawn(cmd, name):
        p = subprocess.Popen(cmd, stdout=open(run / f'{name}.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True); CHILDREN.append(p); return p

    # ============================================================ 1 connection, 2 Lease
    ok = wait(lambda: (lambda r: r if r.get('online') and r.get('reconciled') else None)(robot()), 60)
    check('E1 robot connected: online and reconciled with the server', ok)
    s = state(); sl = s.get('speed_limits') or {}; ev = sl.get('watchdog_evidence') or {}
    check('E1 Agent reports enforced_by WATCHDOG; its evidence carries the Watchdog boot, writer GID, Agent boot, age, and the SERVER verdict',
          sl.get('enforced_by') == 'WATCHDOG' and ev.get('server_verified') is True and ev.get('agent_verified') is True and ev.get('watchdog_boot_id') and ev.get('output_gid') and ev.get('agent_boot_id'),
          json.dumps({k: ev.get(k) for k in ('watchdog_boot_id', 'agent_boot_id', 'evidence_age_s', 'server_verified', 'agent_verified')}))
    check('E1 Watchdog authorized, Gate NORMAL, motor path LOCKED (no driver in this profile)', s['motion_watchdog']['state'] == 'MOTION_AUTHORIZED' and s['safety_gate']['status'] == 'NORMAL' and s['safety_gate']['motor_path']['status'] == 'LOCKED')
    lease = call('/lock', dict(action='acquire', ttl_s=300))[1]; ep['e'] = lease.get('epoch')
    check('E2 Lease acquired', isinstance(ep['e'], int), str(lease)[:90])
    wd_boot_1, agent_boot_1 = ev.get('watchdog_boot_id'), robot()['agent']['boot_id']
    wd_files, ag_files = save_params('motion_watchdog', 'wd'), save_params('robot_agent', 'agent')
    check('E2 launch parameter files captured for the restarts', bool(wd_files) and bool(ag_files), f'{len(wd_files)} / {len(ag_files)}')

    # Nav2's lifecycle manager only activates the whole navigation stack once a map exists; the smoother is what carries the stand-in controller command.
    sm = subprocess.run(['ros2', 'lifecycle', 'get', '/velocity_smoother'], capture_output=True, text=True).stdout.strip()
    if sm.startswith('inactive'):
        subprocess.run(['ros2', 'lifecycle', 'set', '/velocity_smoother', 'activate'], capture_output=True, text=True)
        sm = subprocess.run(['ros2', 'lifecycle', 'get', '/velocity_smoother'], capture_output=True, text=True).stdout.strip()
    check('E3 velocity_smoother is active (activated by this test: there is no map, so the lifecycle manager has not)', sm.startswith('active'), sm)
    # ============================================================ 3-4 positive control
    t0, p0 = flow(6.0)
    print(f'   baseline Gate output (no dynamic limit, 0.30 m/s offered): max forward {p0[0]:.3f}, max |angular| {p0[2]:.3f}, {p0[3]} non-zero samples')
    check('E4 positive control: a non-zero command flows through smoother, Watchdog and Gate (this profile caps the smoother at 0.10 / 0.30; the Watchdog static ceiling 0.15 / 0.50 is above it)', p0[3] >= 20 and 0.099 <= p0[0] <= 0.1501 and p0[2] <= 0.5001, str(p0))
    t0, pc = flow(5.0, v=0.30, w=0.30)
    print(f'   curvature (omega/v): offered {0.30/0.30:.2f} 1/m -> Gate output {(pc[2]/pc[0] if pc[0] else float("nan")):.2f} 1/m (max v {pc[0]:.3f}, max w {pc[2]:.3f})')
    ctl['w'] = 0.0

    # ============================================================ 5-9 request -> pending -> Agent -> Watchdog -> evidence -> approval -> measurement
    c, v = post_limit('auto', 0.06, 0.20); gen1, rid1, t_post = v.get('generation'), v.get('request_id'), time.monotonic()
    check('E5 POST /speed-limits: HTTP 202, PENDING, approved=false', c == 202 and v.get('status') == 'PENDING' and v.get('approved') is False, json.dumps({k: v.get(k) for k in ('status', 'approved', 'reason', 'generation')}))
    seen, final = [], None
    while time.monotonic() - t_post < 30:
        renew(); w = get_view(); key = (w.get('status'), w.get('approved'))
        if not seen or seen[-1] != key: seen.append(key)
        if w.get('approved') is True: final = w; break
        time.sleep(0.2)
    t_app = time.monotonic() - t_post
    check('E6 the view goes PENDING (approved=false) before APPLIED (approved=true)', seen and seen[0] == ('PENDING', False) and seen[-1] == ('APPLIED', True), f'{seen} after {t_app:.1f} s')
    check('E7 the approved view matches the request: robot_id, generation, request_id, scope, values, ceiling, agent_boot_id, watchdog_boot_id, enforced_by',
          final and final['robot_id'] == 'locked_bench' and final['generation'] == gen1 and final['request_id'] == rid1 and final['scope'] == 'auto' and final['applied']['linear_mps'] == 0.06 and final['applied']['angular_rps'] == 0.2
          and final['ceiling']['auto']['linear_mps'] == 0.15 and final['agent_boot_id'] == agent_boot_1 and final['watchdog_boot_id'] == wd_boot_1 and final['enforced_by'] == 'WATCHDOG',
          json.dumps({k: (final or {}).get(k) for k in ('generation', 'request_id', 'agent_boot_id', 'watchdog_boot_id', 'enforced_by')}))
    wev = (final or {}).get('watchdog_evidence') or {}; wa = (wev.get('applied') or {}).get('auto') or {}
    check('E7 the Watchdog evidence lists the same generation, request id and values (not stale) and the server derived server_verified from those facts',
          wev.get('server_verified') is True and (wa.get('generation'), wa.get('request_id'), wa.get('linear_mps'), wa.get('angular_rps'), wa.get('stale')) == (gen1, rid1, 0.06, 0.2, False) and wev.get('applied_match') is True,
          json.dumps(wev.get('server_checks')))
    hold('approved')
    time.sleep(0.5); t0, p1 = flow(6.0)
    print(f'   Gate output with the 0.06 / 0.20 limit: max forward {p1[0]:.3f}, max reverse {p1[1]:.3f}, max |angular| {p1[2]:.3f}, {p1[3]} samples')
    check('E8 measured Gate output while the controller keeps offering 0.30 m/s: never above 0.06 m/s / 0.20 rad/s, flow continues', p1[3] >= 20 and p1[0] <= 0.0601 and p1[1] >= -0.0601 and p1[2] <= 0.2001, str(p1))
    t0, pc2 = flow(5.0, v=0.30, w=0.30)
    print(f'   curvature under the limit: offered omega/v 1.00 1/m -> output {(pc2[2]/pc2[0] if pc2[0] else float("nan")):.2f} 1/m (v {pc2[0]:.3f}, w {pc2[2]:.3f}); path-following effect on Nav2 NOT measured (synthetic robot does not move)')
    ctl['w'] = 0.0

    # ============================================================ 10 refusals
    c, v = post_limit('auto', 0.20, 0.20); check('E9 above the ceiling (0.20 m/s): 422 speed_exceeds_ceiling', c == 422 and v['error']['code'] == 'speed_exceeds_ceiling', str(v)[:90])
    c, v = post_limit('auto', 0.06, 0.60); check('E9 above the angular ceiling: 422', c == 422)
    c, v = post_limit('manual', 0.12, 0.2); check('E9 manual above 0.10: 422', c == 422)
    c, v = post_limit('auto', 0.06, 0.20, lease_epoch=ep['e'] + 7); check('E9 wrong lease_epoch: 409', c == 409 and v['error']['code'] == 'lease_epoch_mismatch')
    c, v = post_limit('auto', 0.06, 0.20, agent_boot_id='not-the-boot'); check('E9 wrong agent_boot_id: 409', c == 409 and v['error']['code'] == 'agent_boot_mismatch')
    w = get_view(); check('E9 the refusals changed nothing (same generation, still approved)', w['generation'] == gen1 and w['approved'] is True)

    # ============================================================ 11 Lease loss -> STALE, no automatic raise
    call('/lock', dict(action='release')); time.sleep(3.0)
    lease = call('/lock', dict(action='acquire', ttl_s=300))[1]; ep['e'] = lease['epoch']; time.sleep(2.5); w = get_view()
    check('E10 after the Lease changed the old setting is STALE: approved=false, stale=true', w['approved'] is False and w.get('stale') is True, json.dumps({k: w.get(k) for k in ('approved', 'stale', 'stale_reason', 'status')}))
    sv = (state().get('speed_limits') or {}).get('watchdog_evidence') or {}
    check('E10 the Watchdog enforcement evidence stays server_verified after the Lease change (the operator can issue a NEW request) while applied_match is false for the old record', sv.get('server_verified') is True and sv.get('applied_match') is False, json.dumps({k: sv.get(k) for k in ('server_verified', 'applied_match')}))
    check('E10 and the effective limit is still the lower 0.06 (no automatic restore)', ((w.get('effective') or {}).get('auto') or {}).get('linear_mps') == 0.06, json.dumps((w.get('effective') or {}).get('auto')))
    check('E10 the previous lease epoch is refused', post_limit('auto', 0.06, 0.2, lease_epoch=ep['e'] - 1)[0] == 409)
    hold('stale')
    t0, p2 = flow(6.0)
    print(f'   after the Lease change: max forward {p2[0]:.3f}, max |angular| {p2[2]:.3f}, {p2[3]} samples')
    check('E11 measured output after Lease loss stays at or below 0.06 / 0.20 (no automatic raise)', p2[3] >= 20 and p2[0] <= 0.0601 and p2[2] <= 0.2001, str(p2))

    # ============================================================ 12 Watchdog restart (the Agent offers nothing: its setting is stale)
    os.kill(procs('motion_watchdog')[0][0], signal.SIGKILL); t_kill = time.monotonic(); time.sleep(1.5)
    check('E12 Watchdog killed while the controller keeps offering 0.30: non-zero Gate output stops', peak(t_kill + 1.0)[3] == 0, str(peak(t_kill + 1.0)))
    spawn(['ros2', 'run', 'limo_robot_agent', 'motion_watchdog', '--ros-args', '-r', '__node:=fleet_motion_watchdog'] + sum([['--params-file', f] for f in wd_files], []), 'watchdog_restart')
    ev2 = wait(lambda: (lambda e: e if e.get('watchdog_boot_id') not in (None, wd_boot_1) and e.get('server_verified') is True else None)((state().get('speed_limits') or {}).get('watchdog_evidence') or {}), 60)
    check('E12 the restarted Watchdog has a new watchdog_boot_id in the evidence the server verified', ev2 is not None, f'{wd_boot_1} -> {(ev2 or {}).get("watchdog_boot_id")}')
    wev = (state().get('speed_limits') or {}).get('watchdog_evidence') or {}; wa = ((wev.get('applied') or {}).get('auto')) or {}
    gs = state().get('safety_gate') or {}
    t_r = time.monotonic(); ctl.update(on=True, v=0.30, w=0.0); time.sleep(5.0); late = peak(t_r)
    print(f'   Gate input after the Watchdog restart: state {gs.get("status")}, non-zero samples over 5 s: {late[3]}')
    check('E12 a restarted Watchdog is a NEW writer: the Gate fails closed (WRITER_RESTARTED) and the output stays 0 although the controller offers 0.30 and the Watchdog is authorized', late[3] == 0, str(late))
    ctl['on'] = False; time.sleep(1.5)                      # the Gate does not re-arm while motion is commanded: stop first, as an operator would
    # The killed Watchdog's DDS endpoint lingers in the graph for a while (about 20 s observed), so the Gate sees TWO writers and keeps NOT_EXCLUSIVE:
    # a reset before the stale endpoint is gone is undone at once. The operator waits until exactly one writer is visible, then resets.
    gone = wait(lambda: len((gate.get('cmd_input') or {}).get('publishers') or []) == 1, 90)
    print(f'   Gate sees exactly one writer on /cmd_vel_raw again: {bool(gone)}; sticky input fault before reset: {(gate.get("cmd_input") or {}).get("sticky")}')
    check('E12 before the operator resets, the Gate still holds the sticky input fault (a restarted writer is not trusted on its own)', (gate.get('cmd_input') or {}).get('sticky') in ('NOT_EXCLUSIVE', 'WRITER_RESTARTED', 'PUBLISHER_CHANGED'), str((gate.get('cmd_input') or {}).get('sticky')))
    ok_reset = False
    for attempt in range(3):
        r = subprocess.run(['ros2', 'service', 'call', '/stall_detector/reset_latch', 'std_srvs/srv/Trigger'], capture_output=True, text=True, timeout=30)
        print('   operator reset of the Gate (reset service):', (r.stdout.strip().splitlines() or [''])[-1][:140])
        if wait(lambda: (gate.get('state') == 'NORMAL'), 25):
            ok_reset = True; break
    check('E12 the operator Gate reset is accepted and the Gate is NORMAL again (the web Gate-reset worker is not part of this profile)', ok_reset, f'gate={gate.get("state")} faults={gate.get("fault_reasons")}')
    check('E12 the restored Watchdog limit is the lower one and marked stale (not confirmed by the current Agent)', wa.get('linear_mps') == 0.06 and wa.get('stale') is True, json.dumps(wa))
    check('E12 the view is not approved after the Watchdog restart', get_view().get('approved') is False)
    t0, p3 = flow(8.0)
    print(f'   after the Watchdog restart: max forward {p3[0]:.3f}, max |angular| {p3[2]:.3f}, {p3[3]} samples')
    check('E12 measured output after the restart is at or below 0.06 / 0.20 (restored lower limit, never the 0.15 ceiling)', p3[3] >= 20 and p3[0] <= 0.0601 and p3[2] <= 0.2001, str(p3))

    # explicit raise under the new Lease: needs two messages and the evidence
    c, v = post_limit('auto', 0.12, 0.35); gen2 = v.get('generation'); w2 = approved(gen2, 30)
    check('E13 a new request under the new Lease is approved, with the NEW Watchdog boot in the evidence', w2 is not None and w2['watchdog_boot_id'] == (ev2 or {}).get('watchdog_boot_id') and w2['generation'] > gen1,
          json.dumps({k: (w2 or {}).get(k) for k in ('generation', 'watchdog_boot_id', 'approved')}))
    time.sleep(1.0); t0, p4 = flow(6.0)
    print(f'   after the explicit raise to 0.12: max forward {p4[0]:.3f}, max |angular| {p4[2]:.3f}, {p4[3]} samples')
    check('E13 measured output follows the explicit raise (above 0.06, not above 0.12 / 0.35)', p4[3] >= 20 and 0.06 < p4[0] <= 0.1201 and p4[2] <= 0.3501, str(p4))

    # ============================================================ Agent restart
    os.kill(procs('robot_agent')[0][0], signal.SIGKILL); time.sleep(2.0)
    check('E14 Agent killed (heartbeat lost): the Watchdog stops forwarding and the Gate output goes to 0', peak(time.monotonic() - 1.0)[3] == 0, str(peak(time.monotonic() - 1.0)))
    spawn(['ros2', 'run', 'limo_robot_agent', 'robot_agent', '--ros-args'] + sum([['--params-file', f] for f in ag_files], []), 'agent_restart')
    ok = wait(lambda: (lambda r: r if r.get('online') and r.get('reconciled') and (r.get('agent') or {}).get('boot_id') not in (None, agent_boot_1) else None)(robot()), 90)
    check('E14 the restarted Agent reconnects with a new boot id and reconciles', ok is not None, f'{agent_boot_1} -> {((ok or {}).get("agent") or {}).get("boot_id")}')
    time.sleep(3.0); w = get_view(); sl = state().get('speed_limits') or {}
    check('E14 the previous record is not approved for the new Agent boot; the Agent limiter is stale but still holds 0.12 (persisted, not the 0.15 ceiling)',
          w['approved'] is False and sl.get('stale') is True and ((sl.get('effective') or {}).get('auto') or {}).get('linear_mps') == 0.12, json.dumps({k: w.get(k) for k in ('approved', 'stale', 'status')}))
    c, v = post_limit('auto', 0.06, 0.2, agent_boot_id=agent_boot_1); check('E14 the old agent_boot_id is refused: 409', c == 409 and v['error']['code'] == 'agent_boot_mismatch')
    lease = call('/lock', dict(action='acquire', ttl_s=300))[1]; ep['e'] = lease.get('epoch')
    wait(lambda: state()['motion_watchdog'].get('state') == 'MOTION_AUTHORIZED', 40)
    t0, p5 = flow(8.0)
    print(f'   after the Agent restart: max forward {p5[0]:.3f}, max |angular| {p5[2]:.3f}, {p5[3]} samples')
    check('E14 measured output after the Agent restart stays at or below 0.12 / 0.35 (the restored lower limit; the ceiling would be 0.15)', p5[3] >= 20 and p5[0] <= 0.1201 and p5[2] <= 0.3501, str(p5))

    # ============================================================ 13 STOP / FAULT
    ctl['on'] = False; time.sleep(1.5)
    check('E15 STOP (the controller offers nothing): Gate output is 0', peak(time.monotonic() - 1.0)[3] == 0, str(peak(time.monotonic() - 1.0)))
    ctl.update(on=True, v=0.30, w=0.0); wait(lambda: peak(time.monotonic() - 1.0)[3] > 5, 20)
    agent_pid = procs('robot_agent')[0][0]; os.kill(agent_pid, signal.SIGSTOP); t_f = time.monotonic(); time.sleep(3.0)
    check('E15 FAULT (Agent frozen: heartbeat and control loop stop) while the controller keeps offering 0.30: Gate output is 0', peak(t_f + 1.0)[3] == 0, str(peak(t_f + 1.0)))
    os.kill(agent_pid, signal.SIGCONT); time.sleep(4.0)
    check('E15 after the fault the effective limit is unchanged (0.12), never raised', ((get_view().get('effective') or {}).get('auto') or {}).get('linear_mps', 1) <= 0.1201)

    # ============================================================ 14 UI-facing consistency
    s = state(); v = get_view()
    check('E16 console state velocity telemetry is marked /odom based and not independent', 'not independent' in ((s.get('velocity') or {}).get('source') or ''))
    check('E16 console state and GET /speed-limits agree on the effective auto limit', abs(s['speed_limits']['effective']['auto']['linear_mps'] - v['effective']['auto']['linear_mps']) < 1e-9)
    ctl['on'] = False; hold('final')
    n = sum(res); (run / 'e2e_result.txt').write_text('\n'.join(log) + f'\n{n}/{len(res)} PASS\n'); print(f'{n}/{len(res)} PASS', flush=True); _cleanup(); os._exit(0 if n == len(res) else 1)


if __name__ == '__main__':
    main()
