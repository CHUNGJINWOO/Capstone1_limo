#!/usr/bin/env python3
"""NAVIGATION mode entry for the REAL-AGENT shape (no Arbiter), isolated ROS (G2, 2026-11-24). Not a real-robot result.

Stack: FLEET_BAG=false FLEET_FLAGS='speed_clamp:=true mode_selector:=true fleet_goals:=true manual:=false velocity:=false' scripts/run_isolated_fleet.sh ...
Same processes as e2e_real_agent_modes_20261124.py, but the Agent has missions ENABLED (the server refuses NAVIGATION on an Agent with missions disabled, which is
what a real robot is). What this checks: the mode can be entered and left with Watchdog-confirmed evidence, that without an Agent-owned Fleet goal NOTHING is forwarded
on the nav channel, and the NAVIGATION <-> MANUAL switch. What it cannot check: a Fleet mission goal and its cancel on a real-shape Agent. Mission admission needs a
Safety Context whose proof for a real/unlocked Gate does not exist (and the isolated 'locked mock' proof needs the Arbiter), so the mission request is only recorded.
usage: e2e_real_agent_navigation_20261124.py RUN_DIR PORT"""
import atexit, json, os, signal, subprocess, sys, threading, time, uuid
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def main():
    run, port = Path(sys.argv[1]), int(sys.argv[2])
    token = (run / 'secrets/operator.token').read_text().strip()
    base = f'http://127.0.0.1:{port}/api/v1'
    rb = base + '/robots/locked_bench'
    res, log = [], []

    def check(name, ok, detail=''):
        res.append(bool(ok)); line = ('PASS ' if ok else 'FAIL ') + name + (f' | {detail}' if detail else ''); print(line, flush=True); log.append(line)

    def call(path, data=None, root=rb):
        req = Request(root + path, None if data is None else json.dumps(data).encode(), headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'})
        try:
            with urlopen(req, timeout=6) as r:
                return r.status, json.loads(r.read())
        except HTTPError as e:
            return e.code, json.loads(e.read() or b'{}')
        except Exception as e:
            return 0, {'error': {'code': 'unreachable', 'message': str(e)}}

    def wait(pred, secs=30, step=.25):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            v = pred()
            if v: return v
            time.sleep(step)
        return None

    import rclpy
    from geometry_msgs.msg import Twist
    rclpy.init(); node = rclpy.create_node('e2e_probe'); rows = []
    node.create_subscription(Twist, '/cmd_vel_locked', lambda m: rows.append((time.monotonic(), m.linear.x, m.angular.z)), 50)
    ctl_node = rclpy.create_node('e2e_controller'); man_node = rclpy.create_node('manual_mapping_teleop')
    ctl_pub = ctl_node.create_publisher(Twist, '/fleet/cmd_vel_nav', 10); man_pub = man_node.create_publisher(Twist, '/fleet/manual_cmd_vel', 10)
    src = {'ctl': False, 'man': False}

    def tick():
        for key, pub in (('ctl', ctl_pub), ('man', man_pub)):
            if src[key]:
                m = Twist(); m.linear.x = 0.30; pub.publish(m)
    node.create_timer(0.05, tick)
    ex = rclpy.executors.MultiThreadedExecutor(4)
    for n in (node, ctl_node, man_node): ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    def peak(t0, t1=None):
        nz = [r for r in rows if r[0] >= t0 and (t1 is None or r[0] <= t1) and (abs(r[1]) > 1e-6 or abs(r[2]) > 1e-6)]
        return (max([r[1] for r in nz] or [0.0]), min([r[1] for r in nz] or [0.0]), max([abs(r[2]) for r in nz] or [0.0]), len(nz))

    ep = {'e': None}
    def renew(): call('/lock', dict(action='renew', ttl_s=300))
    def mode_view(): return call('/mode')[1].get('operating_mode') or {}

    def mode(target, secs=60):
        renew(); r = call('/mode', dict(target=target, idempotency_key=uuid.uuid4().hex, epoch=ep['e']))
        if r[0] not in (200, 202): return {'mode': None, 'refused': r[1]}
        def done():
            om = mode_view(); t = om.get('transition') or {}
            if om.get('mode') == target and t.get('state') not in ('STOPPING_PREVIOUS', 'VERIFYING_IDLE', 'CONFIRMING'): return om
            if om.get('mode') == 'FAULT' or t.get('state') == 'FAILED': return om
            return None
        return wait(done, secs, .5) or mode_view()

    check('N0 robot online and reconciled', wait(lambda: (lambda r: r if r.get('online') and r.get('reconciled') else None)(call('')[1]), 60) is not None)
    a = call('')[1]['agent']
    check('N0 Agent: missions enabled, NO Arbiter, Watchdog selector', a.get('missions_enabled') is True and a.get('locked_velocity_arbiter') is False and a.get('velocity_selector') == 'WATCHDOG_SELECTOR', json.dumps({k: a.get(k) for k in ('missions_enabled', 'locked_velocity_arbiter', 'velocity_selector')}))
    lease = call('/lock', dict(action='acquire', ttl_s=300))[1]; ep['e'] = lease['epoch']
    if subprocess.run(['ros2', 'lifecycle', 'get', '/velocity_smoother'], capture_output=True, text=True).stdout.startswith('inactive'):
        subprocess.run(['ros2', 'lifecycle', 'set', '/velocity_smoother', 'activate'], capture_output=True, text=True)

    # Nav2 only becomes active once a map exists (SLAM). Bring it up the way the real workflow would: a short MAPPING session, then stop and go IDLE.
    om = mode('MAPPING'); renew(); sid = call('/mapping')[1].get('session_id')
    r = call('/mapping', dict(action='START', session_id=sid, idempotency_key=uuid.uuid4().hex, epoch=ep['e'])); sid = r[1].get('session_id') or sid
    up = wait(lambda: (renew(), subprocess.run(['ros2', 'lifecycle', 'get', '/controller_server'], capture_output=True, text=True).stdout.startswith('active'))[1], 120)
    check('N0 setup: Nav2 became active after a MAPPING session produced a map', bool(up))
    wait(lambda: (renew(), call('/mapping')[1].get('state') == 'MAPPING')[1], 60)
    call('/mapping', dict(action='STOP', session_id=sid, idempotency_key=uuid.uuid4().hex, epoch=ep['e'])); wait(lambda: (renew(), call('/mapping')[1].get('state') == 'STOPPED')[1], 60)
    om = mode('IDLE', 60); check('N0 setup: back to IDLE', om.get('mode') == 'IDLE', str(om.get('mode')))
    time.sleep(3.0)
    om = mode('NAVIGATION'); ev = (call('')[1]['state'].get('velocity_arbiter') or {}).get('evidence') or {}
    check('N1 IDLE -> NAVIGATION reaches ACTIVE', om.get('mode') == 'NAVIGATION' and (om.get('transition') or {}).get('state') == 'ACTIVE', json.dumps({'mode': om.get('mode'), 'blocker': (om.get('transition') or {}).get('blocker'), 'reason': (om.get('transition') or {}).get('reason'), 'refused': om.get('refused')})[:240])
    check('N1 the Watchdog confirms NAVIGATION on the nav channel with the server\'s revision', ev.get('watchdog_mode') == 'NAVIGATION' and ev.get('selected_channel') == 'nav' and ev.get('watchdog_revision') == om.get('revision'), json.dumps({k: ev.get(k) for k in ('watchdog_mode', 'watchdog_revision', 'selected_channel')}))
    src['ctl'] = True; time.sleep(1.0); t0 = time.monotonic(); time.sleep(4.0); p = peak(t0)
    check('N2 without an Agent-owned Fleet goal a controller offering 0.30 on the nav channel is NOT forwarded (owned-goal rule)', p[3] == 0, str(p))
    rm = call('/missions', dict(robot_id='locked_bench', type='navigate_to_pose', goal={'frame_id': 'map', 'x': 0.5, 'y': 0.5, 'yaw_rad': 0}, idempotency_key=uuid.uuid4().hex, lease_epoch=ep['e']), root=base)
    print('   mission request on the real-shape Agent:', rm[0], json.dumps(rm[1])[:200], flush=True); log.append(f'   mission request: {rm[0]} {json.dumps(rm[1])[:160]}')
    time.sleep(3.0)
    check('N3 a mission request does not move the robot on this Agent (admission refused or no goal started; recorded above)', peak(time.monotonic() - 3.0)[3] == 0, f'{rm[0]}')
    src['ctl'] = False; t_sw = time.monotonic(); om = mode('MANUAL', 90); t_done = time.monotonic()
    check('N4 NAVIGATION -> MANUAL: MANUAL ACTIVE and the Gate output is 0 until the joystick is used', om.get('mode') == 'MANUAL' and peak(t_sw + 1.0, t_done)[3] == 0, json.dumps({'mode': om.get('mode'), 'reason': (om.get('transition') or {}).get('reason'), 'refused': om.get('refused')})[:200])
    src['man'] = True; time.sleep(1.0); t0 = time.monotonic(); time.sleep(3.0); p = peak(t0)
    check('N4 the joystick flows in MANUAL, cut to the manual ceiling 0.10', p[3] >= 10 and 0.099 <= p[0] <= 0.1001, str(p))
    src['man'] = False; time.sleep(1.0); om = mode('NAVIGATION', 90)
    check('N5 MANUAL -> NAVIGATION reaches ACTIVE', om.get('mode') == 'NAVIGATION' and (om.get('transition') or {}).get('state') == 'ACTIVE', str(om.get('mode')))
    src['ctl'] = True; time.sleep(1.0); t_c = time.monotonic(); om = mode('IDLE', 60); time.sleep(2.0)
    check('N5 NAVIGATION -> IDLE: mode IDLE and the Gate output is 0 while the controller keeps offering 0.30', om.get('mode') == 'IDLE' and peak(t_c + 1.5)[3] == 0, str(peak(t_c + 1.5)))
    src['ctl'] = False
    n = sum(res); (run / 'e2e_result.txt').write_text('\n'.join(log) + f'\n{n}/{len(res)} PASS\n'); print(f'{n}/{len(res)} PASS', flush=True); os._exit(0 if n == len(res) else 1)


if __name__ == '__main__':
    main()
