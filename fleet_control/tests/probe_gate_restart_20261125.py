#!/usr/bin/env python3
"""Gate-restart trace probe (G2, 2026-11-25): records the actual timeline of a Gate kill/restart while the joystick stand-in is held."""
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
    res, log = [], []

    def check(name, ok, detail=''):
        res.append(bool(ok)); line = ('PASS ' if ok else 'FAIL ') + name + (f' | {detail}' if detail else ''); print(line, flush=True); log.append(line)

    def note(msg):
        print('   ' + msg, flush=True); log.append('   ' + msg)

    def call(path, data=None, timeout=5):
        req = Request(base + path, None if data is None else json.dumps(data).encode(), headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'})
        try:
            with urlopen(req, timeout=timeout) as r:
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
    from std_msgs.msg import String
    rclpy.init(); node = rclpy.create_node('e2e_probe'); rows = []; gate = {}; wdst = {}
    node.create_subscription(Twist, '/cmd_vel_locked', lambda m: rows.append((time.monotonic(), m.linear.x, m.angular.z)), 50)
    node.create_subscription(String, '/stall_detector/status_json', lambda m: gate.update(json.loads(m.data)), 10)
    node.create_subscription(String, '/fleet/motion_watchdog/status', lambda m: (wdst.clear(), wdst.update(json.loads(m.data))), 10)
    ctl_node = rclpy.create_node('e2e_controller'); man_node = rclpy.create_node('manual_mapping_teleop')
    ctl_pub = ctl_node.create_publisher(Twist, '/fleet/cmd_vel_nav', 10); man_pub = man_node.create_publisher(Twist, '/fleet/manual_cmd_vel', 10)
    src = {'ctl': False, 'man': False, 'ctl_v': 0.30, 'man_v': 0.30}

    def tick():
        for key, pub, node_ in (('ctl', ctl_pub, ctl_node), ('man', man_pub, man_node)):
            if src[key]:
                m = Twist(); m.linear.x = src[key + '_v']; pub.publish(m)
    node.create_timer(0.05, tick)
    ex = rclpy.executors.MultiThreadedExecutor(4)
    for n in (node, ctl_node, man_node): ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    def peak(t0, t1=None):
        nz = [r for r in rows if r[0] >= t0 and (t1 is None or r[0] <= t1) and (abs(r[1]) > 1e-6 or abs(r[2]) > 1e-6)]
        return (max([r[1] for r in nz] or [0.0]), min([r[1] for r in nz] or [0.0]), max([abs(r[2]) for r in nz] or [0.0]), len(nz))

    ep = {'e': None}
    def renew(): call('/lock', dict(action='renew', ttl_s=300))
    def robot(): return call('')[1]
    def state(): return robot().get('state') or {}

    def mode_view(): return call('/mode')[1].get('operating_mode') or {}

    def mode(target, secs=60):
        """Request a mode and wait for it to settle. Returns the final mode record (mode may be FAULT/another mode when refused)."""
        renew(); call('/mode', dict(target=target, idempotency_key=uuid.uuid4().hex, epoch=ep['e']))
        def done():
            om = mode_view(); t = om.get('transition') or {}
            if om.get('mode') == target and t.get('state') not in ('STOPPING_PREVIOUS', 'VERIFYING_IDLE', 'CONFIRMING'): return om
            if om.get('mode') in ('FAULT',) or t.get('state') == 'FAILED': return om
            return None
        return wait(done, secs, .5) or mode_view()

    def flowing(secs=4.0, min_n=10):
        t0 = time.monotonic(); time.sleep(secs); p = peak(t0); return p, p[3] >= min_n

    def sel():
        return (state().get('velocity_arbiter') or {})

    def procs(key):
        out = subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True).stdout.splitlines(); found = []
        for l in out:
            parts = l.split(None, 2)
            if len(parts) >= 2 and 'e2e_real_agent' not in l and 'ros2 run' not in l and (parts[1].endswith('/' + key) or (len(parts) == 3 and parts[2].split()[0].endswith('/' + key))):
                found.append((int(parts[0]), l.split(None, 1)[1]))
        mine = str(run.parent.parent)
        return [p for p in found if 'robot_id:=second' not in p[1] and mine in p[1]]

    def save_params(key, tag):
        pid, args = procs(key)[0]; out = []
        for i, f in enumerate([a for a in args.split() if a.endswith('.yaml') or a.startswith('/tmp/launch_params_')]):
            d = run / f'{tag}_params_{i}.yaml'
            if not d.exists(): d.write_bytes(Path(f).read_bytes())
            out.append(str(d))
        return out

    def spawn(cmd, name):
        p = subprocess.Popen(cmd, stdout=open(run / f'{name}.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True); CHILDREN.append(p); return p

    def gate_reset_when_writer_unique():
        wait(lambda: len((gate.get('cmd_input') or {}).get('publishers') or []) == 1, 90)
        for _ in range(3):
            r = subprocess.run(['ros2', 'service', 'call', '/stall_detector/reset_latch', 'std_srvs/srv/Trigger'], capture_output=True, text=True, timeout=30)
            if wait(lambda: gate.get('state') == 'NORMAL', 25): return True
        return False


    # ---------------------------------------------------------------- trace instrumentation (all times time.monotonic)
    T0 = time.monotonic(); ev = []
    def E(kind, **kw): ev.append((time.monotonic() - T0, kind, kw))
    from geometry_msgs.msg import Twist as _T
    raw = []
    # NOTE: no subscription on /cmd_vel_raw: a second subscriber there makes the Agent's graph lock fail (VELOCITY_GRAPH_NOT_LOCKED) -- correct fail-closed behaviour, but it would stop the test
    lastg = {}; lastw = {}
    def gcb(m):
        d = json.loads(m.data); k = (d.get('state'), (d.get('cmd_input') or {}).get('state'), d.get('sticky'), tuple(d.get('output_topic_faults') or []))
        if k != lastg.get('k'): lastg['k'] = k; E('gate', state=k[0], cmd_input=k[1], sticky=k[2], out_faults=k[3], want=d.get('want_motion'))
    def wcb(m):
        d = json.loads(m.data); mp = d.get('mode_policy') or {}; lo = d.get('last_output')
        k = (d.get('state'), d.get('last_fault'), mp.get('mode'), mp.get('revision'), mp.get('permits_manual'), mp.get('manual_input_ok'), mp.get('manual_rearmed'))
        lo = d.get('last_output') or {}; raw.append((time.monotonic(), lo.get('linear_x'), lo.get('age_s')))
        if k != lastw.get('k'): lastw['k'] = k; E('wd', state=k[0], fault=k[1], mode=k[2], rev=k[3], permits_manual=k[4], manual_ok=k[5], rearmed=k[6])
    node.create_subscription(String, '/stall_detector/status_json', gcb, 10)
    node.create_subscription(String, '/fleet/motion_watchdog/status', wcb, 10)
    pubs = {}
    def poll_graph():
        for topic in ('/cmd_vel_raw', '/cmd_vel_locked', '/fleet/manual_cmd_vel'):
            cur = set()
            for i in node.get_publishers_info_by_topic(topic):
                cur.add((i.node_name, bytes(i.endpoint_gid).hex()[:12]))
            old = pubs.get(topic, set())
            for x in cur - old: E('dds+', topic=topic, node=x[0], gid=x[1])
            for x in old - cur: E('dds-', topic=topic, node=x[0], gid=x[1])
            pubs[topic] = cur
    node.create_timer(0.2, poll_graph)
    first = None

    lease = call('/lock', dict(action='acquire', ttl_s=300))[1]; ep['e'] = lease.get('epoch')
    sess = {'sid': call('/mapping')[1].get('session_id')}
    def mapping_action(action):
        r = call('/mapping', dict(action=action, session_id=sess['sid'], idempotency_key=uuid.uuid4().hex, epoch=ep['e']))
        if action == 'START' and r[1].get('session_id'): sess['sid'] = r[1]['session_id']
        return r
    mapping_action('START'); wait(lambda: (renew(), peak(time.monotonic() - 3)[3] > 8)[1], 90)      # brings Nav2 (velocity_smoother) up, as in the E2E
    wait(lambda: (renew(), call('/mapping')[1].get('state') == 'MAPPING')[1], 60)
    mapping_action('STOP'); wait(lambda: (renew(), call('/mapping')[1].get('state') == 'STOPPED')[1], 60); time.sleep(2.0)
    for _try in range(4):
        mode('IDLE', 40); wait(lambda: not (sel().get('evidence') or {}).get('agent_graph', {}).get('nav_publishers') is None, 5); time.sleep(4.0)
        om = mode('MANUAL', 90)
        if om.get('mode') == 'MANUAL': break
        print('MANUAL entry attempt', _try, 'ended', om.get('mode'), (om.get('transition') or {}).get('blocker'), flush=True)
    E('mode', mode=om.get('mode')) if False else None
    E('mode', mode=om.get('mode'))
    print('MANUAL entered:', om.get('mode'), lease, flush=True)
    if om.get('mode') != 'MANUAL': sys.exit(2)
    scen = os.environ.get('SCEN', 'held')
    N = int(os.environ.get('N', '3'))
    for it in range(N):
        src['man'] = True; time.sleep(2.0)
        p, ok = flowing(2.0); E('baseline_flow', n=p[3], ok=ok)
        gp, gargs = procs('stall_detector_node')[0]
        gcmd = open(f'/proc/{gp}/cmdline').read().split('\0')[:-1]
        for c0 in list(gcmd):
            if c0.startswith('/tmp/launch_params_'):
                d = run / f'gate_{it}_{Path(c0).name}'; d.write_bytes(Path(c0).read_bytes()); gcmd[gcmd.index(c0)] = str(d)
        n_raw0 = len(raw); n_out0 = len(rows)
        E('gate_kill', pid=gp, scen=scen)
        os.kill(gp, signal.SIGKILL); t_k = time.monotonic()
        if scen == 'release_during':     # the joystick is released (silence) while the Gate is down, then pressed again later
            time.sleep(1.0); src['man'] = False; time.sleep(1.0); E('joystick', action='released')
        else:
            time.sleep(2.0)
        spawn(gcmd, f'gate_restart_{it}'); E('gate_spawn')
        if scen == 'release_during':
            time.sleep(8.0); src['man'] = True; E('joystick', action='pressed_again'); time.sleep(8.0)
        else:
            time.sleep(14.0)
        # first non-zero Gate output after the restart
        nz = [r for r in rows if r[0] > t_k and (abs(r[1]) > 1e-6 or abs(r[2]) > 1e-6)]
        fz = (nz[0][0] - T0) if nz else None
        E('first_nonzero_gate_out', at=fz, count=len(nz))
        # raw (Watchdog -> Gate) gaps > 0.25 s after the restart
        gaps = []
        for r in raw:
            if r[0] > t_k and r[2] is not None and r[2] > 0.25: gaps.append((round(r[0] - T0, 2), r[1], r[2]))
        gaps = gaps[:12]
        E('wd_output_age_gt_250ms_samples', gaps=gaps)
        E('wd_status_samples_after_kill', n=len(raw) - n_raw0, nonzero=len([r for r in raw if r[0] > t_k and r[1] and abs(r[1]) > 1e-6]))
        print(f'iter {it}: first non-zero Gate output t={fz}, nonzero samples after restart={len(nz)}, raw gaps>250ms={gaps}', flush=True)
        wst = (wdst.get('mode_policy') or {})
        E('wd_after_restart', state=wdst.get('state'), fault=wdst.get('last_fault'), need_release=wst.get('manual_need_release'), gate=gate.get('state'))
        src['man'] = False; time.sleep(1.2)                      # the operator releases the stick once
        if gate.get('state') != 'NORMAL': gate_reset_when_writer_unique()
        src['man'] = True; p2, ok2 = flowing(3.0); E('after_release_and_press', gate=gate.get('state'), nonzero=p2[3], flows=ok2)
        print(f'iter {it}: after release + new press: gate={gate.get("state")} nonzero={p2[3]} flows={ok2}', flush=True)
        src['man'] = False; time.sleep(2.0)
        # recover to a clean state for the next iteration (operator-style): reset the latch if the Gate asks for it
        if gate.get('state') != 'NORMAL': gate_reset_when_writer_unique()
        om = mode('IDLE', 40); om = mode('MANUAL', 60)
        if om.get('mode') != 'MANUAL': print('could not re-enter MANUAL', om.get('mode'), flush=True); break
    src['man'] = False
    out = run / f'gate_restart_trace_{scen}.jsonl'
    out.write_text('\n'.join(json.dumps(dict(ts=round(t, 3), k=k, **kw)) for t, k, kw in ev))
    for t, k, kw in ev: print(f'{t:8.3f} {k} {kw}')
    ex.shutdown()

main(); sys.stdout.flush(); os._exit(0)
