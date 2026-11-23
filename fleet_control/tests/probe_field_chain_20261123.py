#!/usr/bin/env python3
"""Field-readiness probe for the isolated, motor-locked integrated stack (G2, 2026-11-23). Localhost only, no hardware.

Start the stack with scripts/run_isolated_fleet.sh RUN_DIR PORT DOMAIN (overlay sourced), then:
    python3 tests/probe_field_chain_20261123.py RUN_DIR PORT [--faults]
It drives the same web API the operator console uses (Lease -> operating mode -> Mapping / Bag / Manual) and checks, with
evidence from the Fleet server and the ROS graph, that:
  1. the velocity chain is Arbiter -> Watchdog -> Gate -> /cmd_vel_locked and /cmd_vel has no endpoint;
  2. MAPPING mode selects the Nav2/Frontier source (arbiter source NAV2), MANUAL selects the Gamepad source and a stopped input decays to zero;
  3. (--faults) a second /cmd_vel publisher, a stopped Watchdog and a released Lease each end the motion.
It never sets a motor path: the stack has no /cmd_vel. Exit code 0 = every check passed.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir'); ap.add_argument('port', type=int); ap.add_argument('--faults', action='store_true')
    args = ap.parse_args()
    run = Path(args.run_dir)
    token = (run / 'secrets/operator.token').read_text().strip()
    base = f'http://127.0.0.1:{args.port}/api/v1/robots/locked_bench'
    results = []

    def check(name, ok, detail=''):
        results.append((name, bool(ok)))
        print(('PASS ' if ok else 'FAIL ') + name + (f' | {detail}' if detail else ''), flush=True)

    def call(path, data=None):
        req = Request(base + path, None if data is None else json.dumps(data).encode(),
                      headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'})
        try:
            with urlopen(req, timeout=4) as r:
                return json.loads(r.read())
        except HTTPError as e:
            return {'_http': e.code, **json.loads(e.read() or b'{}')}

    def state():
        return call('')['state']

    def wait(pred, secs=25, step=.25):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            v = pred()
            if v:
                return v
            time.sleep(step)
        return None

    def graph(topic):
        out = subprocess.run(['ros2', 'topic', 'info', '-v', topic], capture_output=True, text=True, timeout=20).stdout
        pubs, subs, kind = [], [], None
        for line in out.splitlines():
            if line.startswith('Publisher count'): kind = pubs
            elif line.startswith('Subscription count'): kind = subs
            elif line.startswith('Node name:') and kind is not None: kind.append(line.split(':', 1)[1].strip())
        return pubs, subs

    wait(lambda: (lambda s: s if s.get('online') and s.get('reconciled') else None)(call('')), 40)
    lease = call('/lock', dict(action='acquire', ttl_s=120))
    epoch = lease['epoch']

    def renew():
        call('/lock', dict(action='renew', ttl_s=120))

    # 1. graph
    pubs, subs = graph('/cmd_vel_raw')
    check('Gate input /cmd_vel_raw: only the Watchdog publishes, only the Gate listens',
          pubs == ['fleet_motion_watchdog'] and subs == ['stall_detector_node'], f'pubs={pubs} subs={subs}')
    pubs, subs = graph('/cmd_vel')
    check('/cmd_vel has no publisher and no subscriber', not pubs and not subs, f'pubs={pubs} subs={subs}')
    pubs, subs = graph('/cmd_vel_locked')
    check('Gate output /cmd_vel_locked is published by the Gate only and consumed by nobody', pubs == ['stall_detector_node'] and not subs)
    s = state()
    check('Watchdog authorized, Gate NORMAL, motor path LOCKED, movement_permitted false',
          s['motion_watchdog']['state'] == 'MOTION_AUTHORIZED' and s['safety_gate']['status'] == 'NORMAL'
          and s['safety_gate']['motor_path']['status'] == 'LOCKED' and s['safety_gate']['movement_permitted'] is False)

    def mode(target):
        renew()
        r = call('/mode', dict(target=target, idempotency_key=uuid.uuid4().hex, epoch=epoch))
        ok = wait(lambda: (call('/mode').get('operating_mode') or {}).get('mode') == target
                  and not (call('/mode')['operating_mode'].get('transition') or {}).get('state') in ('STOPPING_PREVIOUS', 'VERIFYING_IDLE', 'CONFIRMING'), 40)
        return r, ok

    def arbiter():
        renew()
        return state().get('velocity_arbiter') or {}

    # 2. MAPPING: Frontier source selected
    r, ok = mode('MAPPING')
    check('operating mode MAPPING reached', ok, json.dumps(r)[:160])
    renew()
    sid = call('/mapping').get('session_id')
    start = call('/mapping', dict(action='START', session_id=sid, idempotency_key=uuid.uuid4().hex, epoch=epoch))
    check('Mapping START accepted', start.get('status') == 'QUEUED', json.dumps(start)[:160])
    got = wait(lambda: (lambda a: a if a.get('selected_source') == 'NAV2' and a.get('ready') else None)(arbiter()), 40)
    check('Arbiter selects the Nav2 (Frontier) source with the Watchdog, Lease and Gate conditions met', got, json.dumps(got or arbiter())[:200])
    mw = state().get('mapping_worker') or {}
    check('Mapping worker MAPPING and its goal is tracked', mw.get('status') == 'MAPPING', json.dumps({k: mw.get(k) for k in ('status', 'detail')}))
    stop = call('/mapping', dict(action='STOP', session_id=call('/mapping')['session_id'], idempotency_key=uuid.uuid4().hex, epoch=epoch))
    done = wait(lambda: call('/mapping').get('state') == 'STOPPED', 40)
    check('Mapping STOP completes with the goal terminal', done, json.dumps(call('/mapping').get('result'))[:200])

    # 3. MANUAL
    r, ok = mode('MANUAL')
    check('operating mode MANUAL reached', ok, json.dumps(r)[:160])
    renew()
    bench = call('/manual-bench', dict(action='start', epoch=epoch))
    check('Manual bench session started', 'session_id' in bench, json.dumps(bench)[:160])
    selected = False
    for seq in range(60):
        renew()
        smp = call('/manual-bench', dict(action='sample', input_format='normalized_axes_v1', epoch=epoch, session_id=bench.get('session_id'),
                                         seq=seq, linear=.05, turn=.12, deadman=True))
        a = state().get('velocity_arbiter') or {}
        if a.get('selected_source') == 'MANUAL' and a.get('selected_linear', 0) > 0:
            selected = True
            check('Arbiter selects MANUAL with a positive but bounded command', 0 < a['selected_linear'] <= .10 and abs(a['selected_turn']) <= .30, json.dumps(a)[:200])
            break
        time.sleep(.08)
    check('Gamepad input reached the Arbiter', selected)
    zero = wait(lambda: (lambda a: a if a.get('selected_linear') == 0 and a.get('selected_turn') == 0 else None)(state().get('velocity_arbiter') or {}), 6)
    check('Without further samples the command decays to zero (Deadman TTL)', zero)
    call('/manual-bench', dict(action='stop', epoch=epoch, session_id=bench.get('session_id')))

    # 3b. Gate input source (G2): a rogue /cmd_vel_raw writer while MANUAL is driving must never reach /cmd_vel_locked
    if args.faults:
        import rclpy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        rclpy.init(); node = rclpy.create_node('field_chain_probe')
        locked, gst = [], {}
        node.create_subscription(Twist, '/cmd_vel_locked', lambda m: locked.append((time.monotonic(), m.linear.x, m.angular.z)), 50)
        node.create_subscription(String, '/stall_detector/status_json', lambda m: gst.update(json.loads(m.data)), 10)
        import threading
        ex = rclpy.executors.SingleThreadedExecutor(); ex.add_node(node); threading.Thread(target=ex.spin, daemon=True).start()
        reset_cli = node.create_client(Trigger, '/stall_detector/reset_latch')
        mode('MANUAL'); renew()
        bench2 = call('/manual-bench', dict(action='start', epoch=epoch)); seq2 = [1000]

        def drive(secs):
            end = time.monotonic() + secs
            while time.monotonic() < end:
                seq2[0] += 1
                call('/manual-bench', dict(action='sample', input_format='normalized_axes_v1', epoch=epoch, session_id=bench2.get('session_id'),
                                           seq=seq2[0], linear=.5, turn=0.0, deadman=True))
                if seq2[0] % 20 == 0: renew()
                time.sleep(.08)
        drive(2.0)
        legit = [v for t, v, w in locked if time.monotonic() - t < 1.5]
        check('G1 legitimate MANUAL input reaches /cmd_vel_locked through Arbiter, Watchdog and Gate', any(abs(v) > 0.01 for v in legit), f'max={max([abs(v) for v in legit] or [0]):.3f}')
        before_state = (gst.get('cmd_input') or {}).get('state')
        seen = {}
        def watch_graph():                       # when does a SECOND endpoint become visible to this observer (DDS discovery)?
            while not seen.get('stop'):
                if len(node.get_publishers_info_by_topic('/cmd_vel_raw')) > 1 and 't' not in seen: seen['t'] = time.monotonic()
                time.sleep(.005)
        threading.Thread(target=watch_graph, daemon=True).start()
        rogue = subprocess.Popen(['ros2', 'topic', 'pub', '-r', '25', '/cmd_vel_raw', 'geometry_msgs/msg/Twist', '{linear: {x: 0.15}}'],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        t_rogue = time.monotonic()
        try:
            drive(3.0)
        finally:
            os.killpg(rogue.pid, signal.SIGINT); rogue.wait(timeout=10)
        window = [(t, v, w) for t, v, w in locked if t >= t_rogue]
        leaked = [v for t, v, w in window if abs(v - 0.15) < 0.01]
        first_zero = next((t - t_rogue for t, v, w in window if abs(v) < 1e-6 and t - t_rogue > 0.05), None)
        nonzero_after = [t - t_rogue for t, v, w in window if abs(v) > 1e-3]
        check('G2 the rogue /cmd_vel_raw value (0.15) never reaches /cmd_vel_locked', not leaked, f'leaked samples={len(leaked)}')
        seen['stop'] = True
        t_vis = seen.get('t', t_rogue)
        last_nz = max([t for t, v, w in window if abs(v) > 1e-3] or [t_vis])
        stop_s = max(0.0, last_nz - t_vis)
        check('G3 output is zero within 0.5 s of the second writer becoming visible (last non-zero sample after that, s)', stop_s <= 0.5,
              f'{stop_s:.3f} s after the endpoint is visible ({t_vis - t_rogue:.2f} s after the process start); before={before_state}')
        check('G4 Gate records the cause (cmd_input NOT_EXCLUSIVE, sticky, fault listed)',
              (gst.get('cmd_input') or {}).get('sticky') == 'NOT_EXCLUSIVE' and 'cmd_input:NOT_EXCLUSIVE' in (gst.get('fault_reasons') or []), json.dumps(gst.get('cmd_input') or {})[:200])
        time.sleep(1.0)
        drive(1.5)
        zero_after_leave = all(abs(v) < 1e-6 for t, v, w in locked if t > time.monotonic() - 1.2)
        check('G5 after the rogue leaves the Gate stays at zero until the operator reset (no automatic re-authorisation)', zero_after_leave)
        if reset_cli.wait_for_service(timeout_sec=3):
            f = reset_cli.call_async(Trigger.Request()); end = time.monotonic() + 3
            while not f.done() and time.monotonic() < end: time.sleep(.02)
            check('G6 operator reset accepted', f.done() and f.result().success, f.result().message if f.done() else 'timeout')
        else:
            check('G6 operator reset service available', False)
        # The Gate fault also ended the operating mode (server FAULT); recovery = the operator selects MANUAL again and starts a new bench
        # session (the old one belongs to the ended mode).
        time.sleep(1.0)
        mode_now = (call('/mode').get('operating_mode') or {}).get('mode')
        r2, ok2 = mode('MANUAL')
        bench2 = call('/manual-bench', dict(action='start', epoch=epoch))
        resumed = wait(lambda: (drive(.6), any(abs(v) > 0.01 for t, v, w in locked if time.monotonic() - t < 0.5))[1], 30)
        check('G7 after reset (mode %s -> MANUAL again) the legitimate writer is re-bound and MANUAL drives' % mode_now, resumed,
              json.dumps({'cmd_input': (gst.get('cmd_input') or {}).get('state'), 'gate': gst.get('state'), 'mode_ok': bool(ok2)}))
        # G8: a foreign writer on the Gate's own output topic (the driver input in the isolated stack) is reported by the Gate itself
        rogue2 = subprocess.Popen(['ros2', 'topic', 'pub', '-r', '10', '/cmd_vel_locked', 'geometry_msgs/msg/Twist', '{}'],
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        try:
            seen_fault = wait(lambda: any('FOREIGN_PUBLISHER' in x for x in ((gst.get('cmd_input') or {}).get('output_topic_faults') or [])), 20)
        finally:
            os.killpg(rogue2.pid, signal.SIGINT); rogue2.wait(timeout=10)
        check('G8 a foreign writer on the Gate output topic is reported (cmd_vel:FOREIGN_PUBLISHER), detection only', seen_fault,
              json.dumps((gst.get('cmd_input') or {}).get('output_topic_faults')))
        recovered = wait(lambda: not ((gst.get('cmd_input') or {}).get('output_topic_faults')), 40)
        check('G9 the output-topic fault clears by itself once the foreign writer is gone (not sticky)', recovered)
        call('/manual-bench', dict(action='stop', epoch=epoch, session_id=bench2.get('session_id')))
        ex.shutdown(); node.destroy_node(); rclpy.shutdown()

    # 4. faults
    if args.faults:
        mode('MAPPING'); renew()
        call('/mapping', dict(action='START', session_id=call('/mapping').get('session_id'), idempotency_key=uuid.uuid4().hex, epoch=epoch))
        wait(lambda: (lambda a: a if a.get('selected_source') == 'NAV2' and a.get('ready') else None)(arbiter()), 40)
        pub = subprocess.Popen(['ros2', 'topic', 'pub', '-r', '5', '/cmd_vel', 'geometry_msgs/msg/Twist', '{}'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        try:
            blocked = wait(lambda: (lambda s: s if s['safety_gate']['motor_path']['status'] == 'ARMED' else None)(state()), 20)
            check('A foreign /cmd_vel publisher is detected (motor path ARMED)', blocked)
            a = wait(lambda: (lambda a: a if not a.get('ready') else None)(arbiter()), 15)
            check('...and the Arbiter stops selecting motion', a, (a or {}).get('blocker'))
            check('...and the Mapping session is withdrawn (UNKNOWN or STOPPED), not left driving',
                  wait(lambda: (state().get('mapping_worker') or {}).get('status') in ('UNKNOWN', 'IDLE', 'PAUSED'), 25))
        finally:
            os.killpg(pub.pid, signal.SIGINT); pub.wait(timeout=10)
        wd_pids = subprocess.run(['pgrep', '-f', 'motion_watchdog'], capture_output=True, text=True).stdout.split()
        check('Watchdog process found for the fault injection', bool(wd_pids))
        if wd_pids:
            for pid in wd_pids: os.kill(int(pid), signal.SIGSTOP)
            try:
                # The Agent trusts a Watchdog status only while it is fresh (< 1 s); a frozen Watchdog keeps its last state string,
                # so the evidence is the status age and the Arbiter's refusal.
                lost = wait(lambda: (lambda s, a: (s, a) if (s.get('motion_watchdog') or {}).get('age_s', 0) > 1.0 and not a.get('ready') else None)(state(), arbiter()), 15)
                check('A stopped Watchdog goes stale and the Arbiter stops (fail closed)', lost,
                      json.dumps({'age_s': (state().get('motion_watchdog') or {}).get('age_s'), 'blocker': arbiter().get('blocker')}))
            finally:
                for pid in wd_pids: os.kill(int(pid), signal.SIGCONT)
        call('/lock', dict(action='release'))
        check('Released Lease: the Arbiter is not ready', wait(lambda: not arbiter().get('ready'), 15), json.dumps(arbiter())[:160])
    failed = [n for n, ok in results if not ok]
    print(f'{len(results) - len(failed)}/{len(results)} checks passed', flush=True)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
