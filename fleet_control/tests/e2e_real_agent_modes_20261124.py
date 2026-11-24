#!/usr/bin/env python3
"""Operating-mode E2E for the REAL-AGENT shape (no Arbiter), isolated ROS (G2, 2026-11-24). Not a real-robot result.

Stack: scripts/run_isolated_fleet.sh with FLEET_FLAGS='speed_clamp:=true mode_selector:=true manual:=false velocity:=false':
real Fleet Server, Robot Agent (mock/simulated, watchdog_mode_selector, NO Arbiter), Motion Watchdog (speed clamp + mode gate), Safety Gate
(output remapped to /cmd_vel_locked), Nav2, synthetic sensors. Measurement point: the Gate output. There is no motor driver in this profile
(the driver remap and Mock driver are covered by real_launch_chain_faults_20261123.py).

Motion sources: MAPPING uses the real web Mapping worker (Frontier goals -> Nav2 -> smoother -> Watchdog nav channel). NAVIGATION and MANUAL use STAND-INS:
  * a test node publishing into the smoother input /fleet/cmd_vel_nav (stands for the Nav2 controller; no Fleet mission goal exists in this profile),
  * a test node named manual_mapping_teleop publishing /fleet/manual_cmd_vel (stands for the on-board joystick of the real launch).
NOT covered here: Fleet mission admission/cancel on a real Agent (blocked separately by the Safety Context, which has no real Gate attestation), the real-mode
policy locks on mapping, the physical motor.
usage: e2e_real_agent_modes_20261124.py RUN_DIR PORT   (overlay sourced, ROS_DOMAIN_ID as the stack)"""
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

    pubs_ = {'man': man_pub}; prev_ = {'ctl': False, 'man': False}
    src['silent'] = False                    # True: releasing the stand-in joystick is SILENCE (no neutral command), the case the Watchdog must not accept as a release

    def tick():
        pubs_['ctl'] = ctl_pub
        for key in ('ctl', 'man'):
            if src[key]:
                m = Twist(); m.linear.x = src[key + '_v']; pubs_[key].publish(m)
            elif key == 'man' and prev_[key] and not src['silent']:
                pubs_['man'].publish(Twist())         # a real teleop sends one neutral command when the deadman is released
            prev_[key] = bool(src[key])
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
            try:
                r = subprocess.run(['ros2', 'service', 'call', '/stall_detector/reset_latch', 'std_srvs/srv/Trigger'], capture_output=True, text=True, timeout=30)
            except subprocess.TimeoutExpired:
                note('reset_latch call timed out (CLI), retrying'); continue
            if wait(lambda: gate.get('state') == 'NORMAL', 25): return True
        return False

    def rogue_pub(topic, secs, name, v=0.0):
        code = (f"import rclpy,time\nfrom geometry_msgs.msg import Twist\nrclpy.init();n=rclpy.create_node('{name}');p=n.create_publisher(Twist,'{topic}',10)\n"
                f"e=time.time()+{secs}\nwhile time.time()<e:\n t=Twist();t.linear.x={v};p.publish(t);time.sleep(0.05)\n")
        return spawn([sys.executable, '-c', code], 'rogue_' + name)

    # ============================================================ 0 connection and selector evidence
    check('M0 robot online and reconciled', wait(lambda: (lambda r: r if r.get('online') and r.get('reconciled') else None)(robot()), 60) is not None)
    r0 = robot(); a0 = r0['agent']
    check('M0 Agent has NO Arbiter and declares the Watchdog selector', a0.get('locked_velocity_arbiter') is False and a0.get('velocity_selector') == 'WATCHDOG_SELECTOR', json.dumps({k: a0.get(k) for k in ('locked_velocity_arbiter', 'velocity_selector')}))
    lease = call('/lock', dict(action='acquire', ttl_s=300))[1]; ep['e'] = lease['epoch']
    wd_files, ag_files = save_params('motion_watchdog', 'wd'), save_params('robot_agent', 'agent')
    sm = subprocess.run(['ros2', 'lifecycle', 'get', '/velocity_smoother'], capture_output=True, text=True).stdout.strip()
    if sm.startswith('inactive'):
        subprocess.run(['ros2', 'lifecycle', 'set', '/velocity_smoother', 'activate'], capture_output=True, text=True)
    om = mode_view()
    check('M0 before any mode request the selector says there is no operating mode to confirm and the Watchdog forwards nothing',
          sel().get('kind') == 'WATCHDOG_SELECTOR' and sel().get('motor_output') is False and peak(time.monotonic() - 2)[3] == 0, str({k: sel().get(k) for k in ('operating_mode', 'ready', 'blocker')}))

    # ============================================================ 1 IDLE -> MAPPING
    om = mode('MAPPING'); a = sel(); ev = a.get('evidence') or {}
    check('M1 IDLE -> MAPPING reaches ACTIVE (the server accepted the selector evidence)', om.get('mode') == 'MAPPING' and (om.get('transition') or {}).get('state') == 'ACTIVE', json.dumps({'mode': om.get('mode'), 'state': (om.get('transition') or {}).get('state'), 'blocker': (om.get('transition') or {}).get('blocker'), 'reason': (om.get('transition') or {}).get('reason')})[:200])
    check('M1 the evidence names the SAME mode and revision the server holds, selects the nav channel, both inputs exclusive, same Watchdog boot as the speed-limit evidence',
          ev.get('watchdog_mode') == 'MAPPING' and ev.get('watchdog_revision') == om.get('revision') and ev.get('selected_channel') == 'nav' and ev.get('nav_input_ok') and ev.get('manual_input_ok')
          and ev.get('watchdog_boot_id') == (state().get('speed_limits') or {}).get('watchdog_evidence', {}).get('watchdog_boot_id'), json.dumps({k: ev.get(k) for k in ('watchdog_mode', 'watchdog_revision', 'selected_channel', 'nav_publishers')}))
    renew(); sess = {'sid': call('/mapping')[1].get('session_id')}

    def mapping_action(action):
        r = call('/mapping', dict(action=action, session_id=sess['sid'], idempotency_key=uuid.uuid4().hex, epoch=ep['e']))
        if action == 'START' and r[1].get('session_id'):
            sess['sid'] = r[1]['session_id']                      # START may open a new session: later actions must quote this one
        if r[0] not in (200, 202): note(f'{action} answered {r[0]} {json.dumps(r[1])[:140]}')
        return r
    r = mapping_action('START')
    check('M1 Mapping START accepted', r[1].get('status') == 'QUEUED', json.dumps(r[1])[:120])
    ok = wait(lambda: (renew(), peak(time.monotonic() - 3)[3] > 8)[1], 90)
    p = peak(time.monotonic() - 4); note(f'MAPPING positive control: max forward {p[0]:.3f}, max |angular| {p[2]:.3f}, {p[3]} non-zero samples in 4 s')
    check('M1 positive control: the Frontier/Nav2 command flows through smoother, Watchdog and Gate', ok and p[0] <= 0.1501 and p[2] <= 0.5001, str(p))
    check('M1 while moving, the selector reports the Watchdog output and the source NAV2', sel().get('source') in ('NAV2', 'NONE') and (sel().get('selected_source') == 'NAV2'), str({k: sel().get(k) for k in ('source', 'selected_source', 'selected_linear')}))
    # 2 PAUSE / RESUME (the worker accepts PAUSE only from MAPPING; the first Frontier goal can flow before the state flips)
    wait(lambda: (renew(), call('/mapping')[1].get('state') == 'MAPPING')[1], 60)
    t_p = time.monotonic(); r = mapping_action('PAUSE')
    paused = wait(lambda: (renew(), call('/mapping')[1].get('state') == 'PAUSED')[1], 40); time.sleep(3.0)
    check('M2 PAUSE: the session is PAUSED and the Gate output is 0 (the owned goal ended)', paused and peak(time.monotonic() - 2.5)[3] == 0, str(peak(time.monotonic() - 2.5)))
    r = mapping_action('RESUME')
    resumed = wait(lambda: (renew(), call('/mapping')[1].get('state') == 'MAPPING')[1], 40)
    ok = wait(lambda: (renew(), peak(time.monotonic() - 3)[3] > 8)[1], 60)
    note(f'after RESUME: state {call("/mapping")[1].get("state")}, flow seen: {bool(ok)} (the synthetic map may have no frontier left)')
    check('M2 RESUME: the session is MAPPING again and any command that flows stays within the ceilings', bool(resumed) and peak(time.monotonic() - 3)[0] <= 0.1501, str(peak(time.monotonic() - 3)))
    # 3 PAUSE -> SAVE -> STOP
    wait(lambda: (renew(), call('/mapping')[1].get('state') == 'MAPPING')[1], 60)
    mapping_action('PAUSE'); wait(lambda: (renew(), call('/mapping')[1].get('state') == 'PAUSED')[1], 40); time.sleep(2.0)
    rs = mapping_action('SAVE')
    saved = wait(lambda: (renew(), call('/mapping')[1].get('state') in ('PAUSED',) and (call('/mapping')[1].get('result') or call('/mapping')[1].get('last_result')))[1], 60)
    note(f'SAVE answer: {json.dumps(rs[1])[:160]}')
    check('M3 SAVE from PAUSED is accepted and the output stays 0 meanwhile', rs[0] in (200, 202) and peak(time.monotonic() - 2.0)[3] == 0, f'{rs[0]} {json.dumps(rs[1])[:100]}')
    mapping_action('STOP')
    stopped = wait(lambda: (renew(), call('/mapping')[1].get('state') == 'STOPPED')[1], 60); time.sleep(2.0)
    check('M3 STOP: STOPPED and the Gate output is 0', stopped and peak(time.monotonic() - 1.5)[3] == 0, str(peak(time.monotonic() - 1.5)))

    # ============================================================ 4-5 NAVIGATION (stand-in controller)
    om = mode('IDLE'); check('M4 MAPPING -> IDLE', om.get('mode') == 'IDLE', str(om.get('mode')))
    renew(); rn = call('/mode', dict(target='NAVIGATION', idempotency_key=uuid.uuid4().hex, epoch=ep['e']))
    check('M4 on this Agent (missions disabled, as on a real robot) NAVIGATION is refused up front: mode_unsupported (the Fleet-mission path stays closed; see the fleet_goals stack for the entry test)', rn[0] == 409 and rn[1].get('error', {}).get('code') == 'mode_unsupported', json.dumps(rn[1])[:160])
    check('M4 and the mode stays IDLE with zero output', mode_view().get('mode') == 'IDLE' and peak(time.monotonic() - 1.5)[3] == 0)
    src.update(ctl=True, ctl_v=0.30); time.sleep(1.0); t_n = time.monotonic(); time.sleep(3.0); p = peak(t_n)
    check('M5 in IDLE a controller offering 0.30 on the nav channel is not forwarded', p[3] == 0, str(p)); src['ctl'] = False

    # ============================================================ 6-7 MANUAL (stand-in joystick)
    om = mode('MANUAL'); check('M6 IDLE -> MANUAL reaches ACTIVE', om.get('mode') == 'MANUAL' and (om.get('transition') or {}).get('state') == 'ACTIVE', json.dumps({'mode': om.get('mode'), 'blocker': (om.get('transition') or {}).get('blocker'), 'reason': (om.get('transition') or {}).get('reason')})[:200])
    src.update(man=True, man_v=0.30); p, ok = flowing(4.0)
    note(f'MANUAL stand-in joystick: max forward {p[0]:.3f}, max |angular| {p[2]:.3f}, {p[3]} samples')
    check('M6 MANUAL positive control: joystick command flows, cut to the manual ceiling 0.10', ok and 0.099 <= p[0] <= 0.1001, str(p))
    t_c = time.monotonic(); om = mode('IDLE'); time.sleep(1.5)
    check('M7 MANUAL -> IDLE: the Gate output is 0 although the joystick keeps offering', om.get('mode') == 'IDLE' and peak(t_c + 2.0)[3] == 0, str(peak(t_c + 2.0)))
    src['man'] = False

    # ============================================================ 8 MAPPING <-> MANUAL
    om = mode('MAPPING'); renew(); sess['sid'] = call('/mapping')[1].get('session_id'); mapping_action('START')
    wait(lambda: (renew(), peak(time.monotonic() - 3)[3] > 8)[1], 90)
    t_sw = time.monotonic(); om = mode('MANUAL', 90); t_done = time.monotonic()
    ms = call('/mapping')[1].get('state')
    check('M8 MAPPING -> MANUAL: the mapping session was stopped first and MANUAL is ACTIVE', om.get('mode') == 'MANUAL' and ms in ('STOPPED', 'IDLE'), f'mode={om.get("mode")} mapping={ms}')
    check('M8 the Gate output was 0 from shortly after the request until MANUAL was active (no Nav2 motion under MANUAL)', peak(t_sw + 3.0, t_done)[3] == 0, str(peak(t_sw + 3.0, t_done)))
    src.update(man=True); p, ok = flowing(3.5); check('M8 the joystick flows in MANUAL', ok, str(p))
    src['man'] = False; time.sleep(1.0)
    om = mode('MAPPING'); check('M8 MANUAL -> MAPPING', om.get('mode') == 'MAPPING', str(om.get('mode')))
    # 9 NAVIGATION <-> MANUAL
    check('M9 (NAVIGATION <-> MANUAL is exercised in the fleet_goals stack: NAVIGATION is unsupported here)', True, 'see e2e_real_agent_navigation_20261124.py')
    t_sw = time.monotonic(); om = mode('MANUAL', 90); t_done = time.monotonic()
    check('M9 MAPPING -> MANUAL again: MANUAL ACTIVE and the Gate output is 0 until the joystick is used', om.get('mode') == 'MANUAL' and peak(t_sw + 1.0, t_done)[3] == 0, str(peak(t_sw + 1.0, t_done)))
    # ============================================================ 12 source conflict (still MANUAL, the controller stand-in still offers on the nav channel)
    src.update(man=True, ctl=True); t_cf = time.monotonic(); time.sleep(3.0)
    mp = (wdst.get('mode_policy') or {})
    note(f'conflict: watchdog state {wdst.get("state")} last_fault {wdst.get("last_fault")} conflict_age {mp.get("source_conflict_age_s")}')
    check('M12 motion on BOTH channels (joystick in MANUAL while the Nav2 stand-in offers): the Watchdog trips MOTION_SOURCE_CONFLICT and nothing is forwarded after it',
          wdst.get('last_fault') == 'MOTION_SOURCE_CONFLICT' and peak(t_cf + 1.0)[3] <= 5, f'fault={wdst.get("last_fault")} nz_after_1s={peak(t_cf + 1.0)[3]}')
    sl = sel(); check('M12 the selector no longer reports ready while the conflict is recent', sl.get('ready') is not True and sl.get('blocker') in ('MOTION_SOURCE_CONFLICT', 'WATCHDOG_NOT_AUTHORIZED', 'WATCHDOG_INPUT_NOT_EXCLUSIVE', 'WATCHDOG_MODE_NOT_CONFIRMED'), str(sl.get('blocker')))
    src.update(ctl=False); time.sleep(4.0)
    rogue = rogue_pub('/fleet/manual_cmd_vel', 120, 'rogue_manual', 0.0)
    seen2 = wait(lambda: (wdst.get('mode_policy') or {}).get('manual_input_ok') is False, 30); time.sleep(1.5)       # a new process needs a moment to appear in the DDS graph
    check('M12 a SECOND publisher on the manual channel (zero commands only) fails closed: NOT_EXCLUSIVE, no forwarding', bool(seen2) and wdst.get('last_fault') == 'VELOCITY_INPUT_NOT_EXCLUSIVE' and peak(time.monotonic() - 1.5)[3] == 0, f'fault={wdst.get("last_fault")} state={wdst.get("state")} manual_publishers={(wdst.get("mode_policy") or {}).get("manual_publishers")}')
    sl = sel(); check('M12 and the selector reports WATCHDOG_INPUT_NOT_EXCLUSIVE or the graph violation', sl.get('ready') is not True, str(sl.get('blocker')))
    src['man'] = False; om = mode('IDLE'); om2 = mode('MANUAL', 40)
    check('M12 while a foreign writer is on the manual channel MANUAL is NOT entered (refused as no manual input path, or not confirmed)', om2.get('mode') != 'MANUAL' or (om2.get('transition') or {}).get('state') != 'ACTIVE', json.dumps({'mode': om2.get('mode'), 'blocker': (om2.get('transition') or {}).get('blocker'), 'reason': (om2.get('transition') or {}).get('reason')})[:220])
    os.killpg(rogue.pid, signal.SIGTERM); time.sleep(25.0)
    if (mode_view().get('mode') != 'IDLE'):
        lease = call('/lock', dict(action='acquire', ttl_s=300))[1]; ep['e'] = lease.get('epoch'); mode('IDLE', 40)
    om = mode('MANUAL', 60); check('M12 after the foreign writer is gone MANUAL can be entered again', om.get('mode') == 'MANUAL' and (om.get('transition') or {}).get('state') == 'ACTIVE', json.dumps({'mode': om.get('mode'), 'blocker': (om.get('transition') or {}).get('blocker')})[:200])

    # ============================================================ 11 Lease loss
    src.update(man=True); flowing(3.0)
    t_l = time.monotonic(); call('/lock', dict(action='release')); time.sleep(4.0); omv = mode_view()
    check('M11 Lease released in MANUAL: the mode leaves MANUAL and the Gate output is 0 while the joystick keeps offering', omv.get('mode') != 'MANUAL' and peak(t_l + 2.5)[3] == 0, f'mode={omv.get("mode")} {peak(t_l + 2.5)}')
    lease = call('/lock', dict(action='acquire', ttl_s=300))[1]; ep['e'] = lease.get('epoch'); time.sleep(3.0)
    check('M11 re-acquiring the Lease does NOT restore the previous mode (output stays 0, mode is not MANUAL)', mode_view().get('mode') != 'MANUAL' and peak(time.monotonic() - 2.0)[3] == 0, f'mode={mode_view().get("mode")}')
    src['man'] = False
    om = mode('IDLE', 40); om = mode('MANUAL', 60); src.update(man=True); p, ok = flowing(3.0); check('M11 an explicit new MANUAL request works again', om.get('mode') == 'MANUAL' and ok, str(p))

    # ============================================================ 10 Watchdog restart in MANUAL
    wd_boot_1 = wdst.get('boot_id')
    os.kill(procs('motion_watchdog')[0][0], signal.SIGKILL); t_k = time.monotonic(); time.sleep(2.0)
    check('M10 Watchdog killed in MANUAL while the joystick offers: output stops', peak(t_k + 1.0)[3] == 0, str(peak(t_k + 1.0)))
    spawn(['ros2', 'run', 'limo_robot_agent', 'motion_watchdog', '--ros-args', '-r', '__node:=fleet_motion_watchdog'] + sum([['--params-file', f] for f in wd_files], []), 'watchdog_restart')
    wait(lambda: wdst.get('boot_id') not in (None, wd_boot_1), 60)
    mp0 = (wdst.get('mode_policy') or {}); note(f'first status of the restarted Watchdog: mode_policy.mode={mp0.get("mode")} permits_manual={mp0.get("permits_manual")} state={wdst.get("state")}')
    time.sleep(6.0)
    gs = gate.get('state')
    check('M10 the restarted Watchdog is a new writer: the Gate fails closed and the output stays 0 although the mode on the server is still MANUAL', peak(t_k + 3.0)[3] == 0 and gs != 'NORMAL', f'gate={gs} mode_server={mode_view().get("mode")}')
    src['man'] = False; time.sleep(1.5)
    check('M10 after the operator resets the Gate (single writer visible, no motion commanded) it is NORMAL again', gate_reset_when_writer_unique())
    src['man'] = True; p, ok = flowing(5.0); omw = mode_view()
    check('M14 server: the restarted Watchdog (new boot) does not inherit the old MANUAL confirmation: the mode ended (IDLE/FAULT, reason names the Watchdog) and the pressed stick moves nothing', omw.get('mode') in ('IDLE', 'FAULT') and p[3] == 0, json.dumps({'mode': omw.get('mode'), 'reason': omw.get('reason'), 'samples': p[3]})[:300])
    src['man'] = False; time.sleep(1.0); om = mode('IDLE', 40); om = mode('MANUAL', 60); src['man'] = True; p, ok = flowing(4.0)
    check('M10 an explicit new MANUAL request is confirmed by the restarted Watchdog (policy re-learned from the Agent) and the stick moves again', om.get('mode') == 'MANUAL' and ok, f'{om.get("mode")} {p}')
    src['man'] = False

    # ============================================================ 10 Agent restart in MANUAL
    agent_boot_1 = robot()['agent']['boot_id']; src['man'] = True
    os.kill(procs('robot_agent')[0][0], signal.SIGKILL); t_k = time.monotonic(); time.sleep(2.5)
    check('M10 Agent killed in MANUAL: heartbeat lost, the Watchdog stops forwarding', peak(t_k + 1.5)[3] == 0, str(peak(t_k + 1.5)))
    spawn(['ros2', 'run', 'limo_robot_agent', 'robot_agent', '--ros-args'] + sum([['--params-file', f] for f in ag_files], []), 'agent_restart')
    back = wait(lambda: (lambda r: r if r.get('online') and r.get('reconciled') and (r.get('agent') or {}).get('boot_id') not in (None, agent_boot_1) else None)(robot()), 90)
    check('M10 the restarted Agent reconnects with a new boot and reconciles', back is not None)
    time.sleep(8.0); omv = mode_view(); out_after = peak(time.monotonic() - 6.0)
    note(f'after the Agent restart the server mode is {omv.get("mode")} ({(omv.get("transition") or {}).get("state")}); non-zero samples in the last 6 s: {out_after[3]}; Watchdog state {wdst.get("state")}')
    check('M10 after the Agent restart the previous mode is NOT silently restored to motion (output stays 0 while the joystick keeps offering)', out_after[3] == 0, f'mode={omv.get("mode")} {out_after}')
    src['man'] = False

    # ============================================================ 13 Gate restart, Watchdog freeze, Agent freeze (state delay), command-source failure -- all in MANUAL
    lease = call('/lock', dict(action='acquire', ttl_s=300))[1]; ep['e'] = lease.get('epoch')
    om = mode('IDLE', 40); om = mode('MANUAL', 60)
    check('M13 MANUAL entered again for the fault steps', om.get('mode') == 'MANUAL', str(om.get('mode')))
    src['man'] = True; time.sleep(1.0); p, ok = flowing(3.0); check('M13 joystick flows (positive control)', ok, str(p))
    # command source failure: the joystick node stops publishing (silence), then comes back
    src['silent'] = True; t_s = time.monotonic(); src['man'] = False; time.sleep(2.0)
    check('M13 command-source failure: when the joystick stops publishing the Gate output returns to 0 within the Gate/Watchdog input timeouts', peak(t_s + 1.0)[3] == 0, str(peak(t_s + 1.0)))
    src['man'] = True; time.sleep(3.0); p = peak(time.monotonic() - 3.0)
    check('M15 A: the same deflected stream returning after > 0.6 s of SILENCE is NOT a release: nothing is forwarded (no neutral command was seen)', p[3] == 0, str(p))
    src['silent'] = False; src['man'] = False; time.sleep(1.0); src['man'] = True; p, ok = flowing(3.0)
    check('M15 A: after an explicit neutral command and a fresh deflection it flows again (positive control)', ok, str(p))
    # DDS endpoint removed and a same-named publisher created while the stick stays deflected
    old_pub = pubs_['man']; new_pub = man_node.create_publisher(Twist, '/fleet/manual_cmd_vel', 10); pubs_['man'] = new_pub; man_node.destroy_publisher(old_pub); t_e = time.monotonic()
    time.sleep(5.0); p = peak(t_e + 0.3)
    check('M15 B: the manual publisher was removed and recreated under the SAME node name while the stick stayed deflected: nothing is forwarded by itself (endpoint latch or NOT_EXCLUSIVE, then a neutral command is still required)', p[3] == 0, f'{p} wd={wdst.get("state")}/{wdst.get("last_fault")} latches={(wdst.get("mode_policy") or {}).get("manual_need_release")}')
    omx = mode_view()
    note(f'after the endpoint change: server mode {omx.get("mode")}, Watchdog {wdst.get("state")} ({wdst.get("last_fault")})')
    src['man'] = False; time.sleep(1.5)
    if mode_view().get('mode') != 'MANUAL': mode('IDLE', 40); mode('MANUAL', 60)
    src['man'] = True; p, ok = wait(lambda: peak(time.monotonic() - 3.0)[3] > 10, 40), None
    p = peak(time.monotonic() - 3.0)
    check('M15 B: after the neutral command and a fresh deflection the stream moves again (positive control)', p[3] > 10, str(p))
    # Watchdog frozen (stale status / scheduling stall)
    wdp = procs('motion_watchdog')[0][0]; os.kill(wdp, signal.SIGSTOP); t_f = time.monotonic(); time.sleep(3.0)
    check('M13 state delay: Watchdog frozen (SIGSTOP 3 s) while the joystick offers: Gate output is 0', peak(t_f + 1.0)[3] == 0, str(peak(t_f + 1.0)))
    os.kill(wdp, signal.SIGCONT); time.sleep(6.0); out_f = peak(time.monotonic() - 4.0)
    note(f'after the Watchdog resumed: watchdog {wdst.get("state")} fault {wdst.get("last_fault")}, non-zero samples in the last 4 s: {out_f[3]}')
    check('M13 after the freeze the Watchdog went through FAILSAFE (WATCHDOG_SELF_STALL or heartbeat/status loss), nothing queued was forwarded', wdst.get('last_fault') is not None, str(wdst.get('last_fault')))
    # Agent frozen: heartbeat and control loop stop, then it recovers
    agp = procs('robot_agent')[0][0]; os.kill(agp, signal.SIGSTOP); t_f = time.monotonic(); time.sleep(3.0)
    check('M13 state delay: Agent frozen (SIGSTOP 3 s): Gate output is 0', peak(t_f + 1.0)[3] == 0, str(peak(t_f + 1.0)))
    os.kill(agp, signal.SIGCONT); time.sleep(8.0); out_a = peak(time.monotonic() - 4.0); omv = mode_view()
    note(f'after the Agent resumed: server mode {omv.get("mode")}, non-zero samples in the last 4 s: {out_a[3]}, selector blocker {sel().get("blocker")}')
    check('M13 after the Agent freeze the mode revision is withdrawn: the held/offering joystick is NOT forwarded again by itself', out_a[3] == 0, str(out_a))
    mm = robot().get('mode_motion') or {}
    check('M14 server: while the Watchdog does not confirm the mode the robot is not reported as a normal ACTIVE MANUAL (mode_motion.verified is False, or the mode already ended)', omv.get('mode') != 'MANUAL' or mm.get('verified') is False, json.dumps({'mode': omv.get('mode'), 'mode_motion': mm})[:260])
    ended = wait(lambda: mode_view().get('mode') in ('IDLE', 'FAULT'), 15)
    check('M14 server: the contradiction that persists ends the mode (IDLE/FAULT) with the reason stated', bool(ended) and 'no longer confirms' in str(mode_view().get('reason')), json.dumps({'mode': mode_view().get('mode'), 'reason': mode_view().get('reason')})[:260])
    time.sleep(3.0); check('M14 and it does not come back by itself (the Watchdog recovered, the old MANUAL is not restored): mode stays IDLE, output 0', mode_view().get('mode') in ('IDLE', 'FAULT') and peak(time.monotonic() - 3.0)[3] == 0, f'{mode_view().get("mode")} {peak(time.monotonic() - 3.0)}')
    src['man'] = False; time.sleep(1.0)
    om = mode('IDLE', 40); om = mode('MANUAL', 60); src['man'] = True; p, ok = flowing(3.5)
    check('M13 an explicit new MANUAL request restores it after the freeze', om.get('mode') == 'MANUAL' and ok, str(p))
    # Gate restarted (a different writer of the output path than the Watchdog: its own boot, require_operator_unlatch)
    gp, gargs = procs('stall_detector_node')[0]
    gcmd = open(f'/proc/{gp}/cmdline').read().split('\0')[:-1]
    gcmd = [c if not c.startswith('/tmp/launch_params_') else str(run / ('gate_' + Path(c).name)) for c in gcmd]
    for c0 in open(f'/proc/{gp}/cmdline').read().split('\0')[:-1]:
        if c0.startswith('/tmp/launch_params_'): (run / ('gate_' + Path(c0).name)).write_bytes(Path(c0).read_bytes())
    os.kill(gp, signal.SIGKILL); t_k = time.monotonic(); time.sleep(2.5)
    check('M13 Gate killed in MANUAL: no further non-zero command is delivered on the Gate output', peak(t_k + 0.5)[3] == 0, str(peak(t_k + 0.5)))
    mm = robot().get('mode_motion') or {}
    note(f'1 s-3 s after the Gate was killed: mode {mode_view().get("mode")}, mode_motion {json.dumps(mm)[:200]}')
    spawn(gcmd, 'gate_restart'); time.sleep(10.0)
    note(f'restarted Gate: state {gate.get("state")}, require_operator_unlatch {gate.get("require_operator_unlatch")}; Watchdog {wdst.get("state")} ({wdst.get("last_fault")})')
    check('M13 the restarted Gate does not arm while a command is being offered: output stays 0', peak(time.monotonic() - 6.0)[3] == 0, f'gate={gate.get("state")} {peak(time.monotonic() - 6.0)}')
    src['man'] = False; time.sleep(1.5)
    reset_ok = wait(lambda: gate.get('state') == 'NORMAL', 25) or gate_reset_when_writer_unique()
    check('M13 with no command offered the Gate becomes NORMAL again (after the operator reset if it asks for one)', bool(reset_ok), f'gate={gate.get("state")}')
    time.sleep(3.0); src['man'] = True; p, ok = flowing(5.0)
    gate_ended = mode_view()
    check('M14 server: the Gate outage ended the MANUAL mode (the Watchdog left MOTION_AUTHORIZED); it is IDLE/FAULT, not an ACTIVE MANUAL that merely looks normal', gate_ended.get('mode') in ('IDLE', 'FAULT'), json.dumps({'mode': gate_ended.get('mode'), 'reason': gate_ended.get('reason')})[:260])
    check('M14 after the Gate restart and a fresh press the old MANUAL does not resume (mode ended): output stays 0', p[3] == 0, str(p))
    src['man'] = False; time.sleep(1.0); om = mode('IDLE', 40); om = mode('MANUAL', 60); src['man'] = True; p, ok = flowing(4.0)
    check('M14 an explicit new MANUAL request + release/press moves again (positive control after the Gate restart)', om.get('mode') == 'MANUAL' and ok, f'{om.get("mode")} {p}')
    src['man'] = False

    # ============================================================ 11 server outage in MANUAL
    lease = call('/lock', dict(action='acquire', ttl_s=300))[1]; ep['e'] = lease.get('epoch')
    om = mode('IDLE', 40); om = mode('MANUAL', 60)
    if om.get('mode') == 'MANUAL':
        src['man'] = True; flowing(3.0)
        spid = [int(l.split(None, 1)[0]) for l in subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True).stdout.splitlines() if 'fleet_server serve' in l and str(port) in l][0]
        os.kill(spid, signal.SIGSTOP); t_s = time.monotonic()
        try:
            time.sleep(8.0)
            check('M11 server unreachable (frozen): the Agent stops stating a mode to the Watchdog and the Gate output is 0 within a few seconds', peak(t_s + 4.0)[3] == 0, str(peak(t_s + 4.0)))
        finally:
            os.kill(spid, signal.SIGCONT)
        time.sleep(8.0); omv = mode_view(); out_after = peak(time.monotonic() - 5.0)
        note(f'after the server returned: mode {omv.get("mode")}, non-zero samples in the last 5 s: {out_after[3]}; selector blocker {sel().get("blocker")}')
        check('M11 after the connection came back the joystick is NOT forwarded again by itself (the mode revision was withdrawn)', out_after[3] == 0, f'mode={omv.get("mode")} {out_after}')
        om = mode('IDLE', 40); om = mode('MANUAL', 60); time.sleep(1.0); t_h = time.monotonic(); time.sleep(3.0)
        check('M11 an explicit new MANUAL request (new revision) is accepted although the joystick is still held, and the held joystick is NOT forwarded (deadman re-arm)', om.get('mode') == 'MANUAL' and (om.get('transition') or {}).get('state') == 'ACTIVE' and peak(t_h)[3] == 0, json.dumps({'mode': om.get('mode'), 'reason': (om.get('transition') or {}).get('reason')})[:200] + str(peak(t_h)))
        src['man'] = False; time.sleep(1.0); src['man'] = True; p, ok = flowing(3.5)
        check('M11 after releasing the joystick once and pressing it again it is forwarded', ok, str(p))
        src['man'] = False
    else:
        check('M11 MANUAL for the server-outage step', False, str(om.get('mode')))

    n = sum(res); (run / 'e2e_result.txt').write_text('\n'.join(log) + f'\n{n}/{len(res)} PASS\n'); print(f'{n}/{len(res)} PASS', flush=True); _cleanup(); os._exit(0 if n == len(res) else 1)


if __name__ == '__main__':
    main()
