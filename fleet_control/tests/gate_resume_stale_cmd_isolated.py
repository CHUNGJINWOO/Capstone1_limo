"""SIMULATION ONLY (2026-11-21): does the Safety Gate re-emit an OLD non-zero command right after it resumes from a stall?

Isolated rig (ROS_LOCALHOST_ONLY=1, a fresh private domain per case, no driver, no LIMO): the Gate under test (default: candidate r12,
UNMODIFIED, read in place), the fake robot of experiments/.../gate_r11_input (fake_sensors.py, driver HOLDS the last command), and a
/cmd_vel_raw writer named fleet_motion_watchdog (twist_pub.py, 0.08 m/s, 20 Hz). While the robot moves, the Gate process is
SIGSTOPped for D seconds and, during the stop, the upstream command changes:
  zero    the writer switches to 0 (SIGUSR1) at D/2: the operator released / the Arbiter zeroed. Correct Gate output after resume: 0.
  silent  the writer is SIGSTOPped with the Gate and stays stopped after the Gate resumes: no fresh input. Correct output: 0.
  live    the writer keeps commanding 0.08 through the stop (normal function check): the Gate must RESUME motion within 0.5 s
          after its own resume (or latch a sensor fault for long stops, which is also correct); reported: resume latency.
Clock: wall (Gate default) or sim (/clock published from wall time at 100 Hz, Gate use_sim_time:=true, as in Gazebo).
Verdict per case: STALE_REUSE if any Gate output after SIGCONT has |vx| > 1e-6 although the latest upstream command is zero / absent.
Recorded: writer change time, SIGSTOP/SIGCONT times, every Gate output (monotonic), Gate status after resume (state, cmd_vx_in,
fault reasons), robot displacement after resume. Model of the Gate boundary only: NOT a real-LIMO measurement.

  swap    (2026-11-22) DDS writer replacement: the writer (fake Watchdog that also publishes its status JSON with boot_id and its
          own output GID, like motion_watchdog.py) is SIGKILLed and a NEW writer with the same node name starts at once, commanding
          0.08. Correct: the Gate does not move the robot on the unverified new writer before an operator reset (verdict
          UNVERIFIED_WRITER_ACCEPTED otherwise); recovery: /stall_detector/reset_latch retried each 1 s, motion within 2 s after an
          accepted reset (RECOVERED, time recorded). The kill-old-endpoint linger time seen by the Gate's graph is recorded.
Extra Gate parameters: --gate-param name:=value (repeatable), e.g. input_writer_status_topic:=/fleet/motion_watchdog/status (r13).

usage: python3 gate_resume_stale_cmd_isolated.py --out <new dir> [--gate <path>] [--rounds 5] [--durations 0.2,0.5,2.0]
"""
import argparse, json, os, signal, subprocess, sys, threading, time
from pathlib import Path

HERE = Path(__file__).resolve().parent; REPO = HERE.parents[1]
IN = REPO / 'experiments/safety_chain_fix_20261113/harness/gate_r11_input'
R12 = REPO / 'experiments/safety_chain_fix_20261113/candidate_gate_r12/nodes/stall_detector_node.py'
BASE = {'enabled': True, 'enable_auto_reverse': False, 'enable_auto_rotate_recovery': False, 'require_operator_unlatch': True,
        'depth_mode': 'disabled', 'estimator_odom_topic': '/odom_nav2', 'input_exclusive_publishers': '[fleet_motion_watchdog]',
        'input_max_abs_linear_x': 0.20, 'input_max_abs_angular_z': 0.60}
CLOCK_SRC = '''
import os, time, rclpy
from rosgraph_msgs.msg import Clock
rclpy.init(); n = rclpy.create_node('resume_test_clock'); p = n.create_publisher(Clock, '/clock', 10)
def t():
    s = time.time(); m = Clock(); m.clock.sec = int(s); m.clock.nanosec = int((s % 1) * 1e9); p.publish(m)
n.create_timer(.01, t)
try: rclpy.spin(n)
except KeyboardInterrupt: pass
'''
WRITER_SRC = '''
import json, signal, sys, time, uuid, rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
v = [float(sys.argv[1])]
rclpy.init(); n = rclpy.create_node('fleet_motion_watchdog'); p = n.create_publisher(Twist, '/cmd_vel_raw', 10)
st = n.create_publisher(String, '/fleet/motion_watchdog/status', 10); boot = uuid.uuid4().hex
signal.signal(signal.SIGUSR1, lambda *a: v.__setitem__(0, 0.0)); signal.signal(signal.SIGUSR2, lambda *a: v.__setitem__(0, 0.08))
def tw():
    m = Twist(); m.linear.x = v[0]; p.publish(m)
def stat():
    mine = [x for x in n.get_publishers_info_by_topic('/cmd_vel_raw') if x.node_name == n.get_name()]
    g = bytes(mine[0].endpoint_gid).hex() if len(mine) == 1 else None
    st.publish(String(data=json.dumps({'boot_id': boot, 'output_gid': g, 'state': 'MOTION_AUTHORIZED'})))
n.create_timer(.05, tw); n.create_timer(.2, stat)
try: rclpy.spin(n)
except KeyboardInterrupt: pass
'''


def swap_case(R, procs, start, outs, st, truth, obs, n_pubs):
    from std_srvs.srv import Trigger
    time.sleep(1.0)
    x0 = truth[0][1]; t_kill = time.monotonic(); os.killpg(procs['wd'].pid, signal.SIGKILL); procs.pop('wd')
    start('wd2', [sys.executable, '-c', WRITER_SRC, '0.08'])
    t_new = time.monotonic(); t_reset = None; tries = []; cli = obs.create_client(Trigger, '/stall_detector/reset_latch')
    linger = None; t_pre_end = None; t_go = None; x_pre_box = [None]
    while time.monotonic() - t_kill < 45:
        if linger is None and n_pubs() <= 1 and time.monotonic() - t_kill > 1.5: linger = round(time.monotonic() - t_kill, 2)
        # before any reset: does the Gate move the robot on the new, unverified writer?
        if time.monotonic() - t_new >= 6 and t_pre_end is None:
            x_pre_box[0] = truth[0][1]; t_pre_end = time.monotonic(); os.killpg(procs['wd2'].pid, signal.SIGUSR1)    # phase 2: zero input (as a LOCKED Watchdog)
        if t_pre_end and time.monotonic() - t_pre_end >= 1.5 and t_reset is None and cli.wait_for_service(timeout_sec=1):
            f = cli.call_async(Trigger.Request()); end = time.monotonic() + 3
            while not f.done() and time.monotonic() < end: time.sleep(.02)
            res = f.result() if f.done() else None
            tries.append((round(time.monotonic() - t_kill, 2), bool(res and res.success), (res.message if res else 'NO_ANSWER')[:60]))
            if res and res.success: t_reset = time.monotonic(); time.sleep(1.0); os.killpg(procs['wd2'].pid, signal.SIGUSR2); t_go = time.monotonic()
        if t_go and time.monotonic() - t_go > 2.5: break
        time.sleep(.5 if t_reset is None else .05)
    x_pre = x_pre_box[0] if x_pre_box[0] is not None else truth[0][1]
    pre = [o for o in outs if t_kill + .3 <= o[0] < (t_reset or time.monotonic()) and abs(o[1]) > 1e-6]
    post = next((round(o[0] - t_go, 3) for o in outs if t_go and o[0] >= t_go and o[1] > .05), None)
    faults = sorted({tuple(s[4]) and s[4][0] for s in st if s[0] >= t_kill and s[4]})
    R.update(result='UNVERIFIED_WRITER_ACCEPTED' if pre else ('RECOVERED' if post is not None and post <= 2.0 else 'NO_RECOVERY'),
             nonzero_before_reset=len(pre), first_nonzero_before_reset_s=round(pre[0][0] - t_kill, 3) if pre else None,
             moved_in_unverified_phase_m=round(x_pre - x0, 4), reset_tries=tries[:12],
             reset_accepted_after_kill_s=round(t_reset - t_kill, 2) if t_reset else None, motion_after_reset_s=post,
             old_endpoint_gone_after_s=linger, gate_fault_reasons_after_kill=faults[:6],
             gate_states_after_kill=sorted({s[1] for s in st if s[0] >= t_kill}))


def case_main(name, out, dom, gate, gate_params=()):
    kind, clock, d = name.split('_')[0], name.split('_')[1], float(name.split('_')[2])
    os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=str(dom), ROS2CLI_NO_DAEMON='1', ROS_LOG_DIR=str(out / 'roslog'))
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String, Float64MultiArray
    procs, log = {}, open(out / f'{name}.proc.log', 'w')

    def start(key, cmd, env=None): procs[key] = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)

    def stop(key):
        p = procs.pop(key, None)
        if p and p.poll() is None:
            os.killpg(p.pid, signal.SIGCONT); os.killpg(p.pid, signal.SIGINT)
            try: p.wait(5)
            except subprocess.TimeoutExpired: os.killpg(p.pid, signal.SIGKILL); p.wait(5)
    gate_cmd = [sys.executable, str(gate), '--ros-args', '-r', '__node:=stall_detector_node', '-r', '/cmd_vel:=/cmd_vel_locked']
    for k, v in BASE.items(): gate_cmd += ['-p', f'{k}:={str(v).lower() if isinstance(v, bool) else v}']
    for gp_ in gate_params: gate_cmd += ['-p', gp_]
    if clock == 'sim': gate_cmd += ['-p', 'use_sim_time:=true']
    rclpy.init(args=[]); obs = rclpy.create_node('resume_test_observer')
    outs, st, truth = [], [], [None]
    obs.create_subscription(Twist, '/cmd_vel_locked', lambda m: outs.append((time.monotonic(), m.linear.x, m.angular.z)), 100)
    def on_st(m):
        try: d_ = json.loads(m.data)
        except ValueError: return
        st.append((time.monotonic(), d_.get('state'), d_.get('cmd_vx_in'), d_.get('cmd_vx_out'), (d_.get('fault_reasons') or [])[:3]))
    obs.create_subscription(String, '/stall_detector/status_json', on_st, 20)
    obs.create_subscription(Float64MultiArray, '/e2e/ground_truth', lambda m: truth.__setitem__(0, (time.monotonic(), m.data[0])), 50)
    ex = rclpy.executors.SingleThreadedExecutor(); ex.add_node(obs); th = threading.Thread(target=ex.spin, daemon=True); th.start()
    R = {'case': name, 'kind': kind, 'clock': clock, 'stall_s': d, 'gate': str(gate)}
    try:
        if clock == 'sim': start('clock', [sys.executable, '-c', CLOCK_SRC])
        start('sensors', [sys.executable, str(IN / 'fake_sensors.py')], env=dict(os.environ, SIM_DRIVER_TIMEOUT='0'))
        start('gate', gate_cmd)
        end = time.monotonic() + 25
        while not (st and st[-1][1] == 'NORMAL') and time.monotonic() < end: time.sleep(.05)
        if not (st and st[-1][1] == 'NORMAL'): R['result'] = 'NOT_ARMED'; return
        # a Gate with input_writer_status_topic binds only a writer that reports its own GID: use the status-publishing writer
        if kind == 'swap' or any(g.startswith('input_writer_status_topic') for g in gate_params): start('wd', [sys.executable, '-c', WRITER_SRC, '0.08'])
        else: start('wd', [sys.executable, str(IN / 'twist_pub.py'), '0.08', '0', '--ros-args', '-r', '__node:=fleet_motion_watchdog'])
        end = time.monotonic() + 10
        while not (outs and outs[-1][1] > .05) and time.monotonic() < end: time.sleep(.05)
        if not (outs and outs[-1][1] > .05): R['result'] = 'NOT_MOVING'; return
        if kind == 'swap':
            swap_case(R, procs, start, outs, st, truth, obs, lambda: len(obs.get_publishers_info_by_topic('/cmd_vel_raw'))); return
        time.sleep(1.0)
        gp, wp = procs['gate'].pid, procs['wd'].pid
        t_stop = time.monotonic(); os.killpg(gp, signal.SIGSTOP)
        if kind == 'silent': os.killpg(wp, signal.SIGSTOP)
        time.sleep(d / 2)
        t_change = time.monotonic()
        if kind == 'zero': os.killpg(wp, signal.SIGUSR1)          # writer publishes 0 from now on
        time.sleep(d / 2)
        x_res = truth[0][1]; t_res = time.monotonic(); os.killpg(gp, signal.SIGCONT)
        time.sleep(1.5)
        after = [o for o in outs if o[0] >= t_res]
        nz = [o for o in after if abs(o[1]) > 1e-6 or abs(o[2]) > 1e-6]
        if kind == 'live':
            moving = next((o[0] - t_res for o in after if o[1] > .05), None); fault = any(s[1] != 'NORMAL' for s in st if s[0] >= t_res)
            verdict = 'OK' if (moving is not None and moving <= .5) or (fault and moving is None) else 'NO_RESUME'
            R['live_resume_latency_s'] = None if moving is None else round(moving, 4)
        else: verdict = 'STALE_REUSE' if nz else 'OK'
        R.update(result=verdict, t_stop=round(t_stop, 4), t_change=round(t_change, 4), t_resume=round(t_res, 4),
                 last_output_before_stop=next(((round(o[1], 3), round(o[2], 3)) for o in reversed(outs) if o[0] < t_stop), None),
                 outputs_during_stop=sum(1 for o in outs if t_stop <= o[0] < t_res),
                 first_outputs_after_resume=[(round(o[0] - t_res, 4), round(o[1], 3)) for o in after[:5]],
                 nonzero_after_resume=len(nz), nonzero_after_resume_window_s=round(nz[-1][0] - t_res, 4) if nz else None,
                 gate_status_after=[(round(s[0] - t_res, 3), *s[1:]) for s in st if s[0] >= t_res][:4],
                 moved_after_resume_m=round(truth[0][1] - x_res, 4))
    finally:
        for k in list(procs): stop(k)
        (out / f'{name}.json').write_text(json.dumps(R, indent=1, default=str))
        ex.shutdown(); th.join(5); obs.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--case'); ap.add_argument('--dom', type=int)
    ap.add_argument('--gate', default=str(R12)); ap.add_argument('--rounds', type=int, default=5)
    ap.add_argument('--gate-param', action='append', default=[]); ap.add_argument('--durations', default='0.2,0.5,2.0'); ap.add_argument('--clocks', default='wall,sim'); ap.add_argument('--kinds', default='zero,silent')
    a = ap.parse_args(); out = Path(a.out); out.mkdir(parents=True, exist_ok=bool(a.case))
    if a.case: case_main(a.case, out, a.dom, Path(a.gate), a.gate_param); sys.exit(0)
    names = [f'{k}_{c}_{d}_r{r}' for r in range(a.rounds) for c in a.clocks.split(',') for k in a.kinds.split(',') for d in a.durations.split(',')]
    summ = {}
    for i, n in enumerate(names):
        subprocess.run([sys.executable, __file__, '--out', str(out), '--case', n, '--dom', str(190 + i % 30), '--gate', a.gate]
                       + [x for g in a.gate_param for x in ('--gate-param', g)], timeout=120)
        try: r = json.loads((out / f'{n}.json').read_text())
        except Exception: r = {'result': 'NO_RESULT'}
        key = n.rsplit('_r', 1)[0]; summ.setdefault(key, []).append(r.get('result'))
        print(n, r.get('result'), r.get('first_outputs_after_resume'), flush=True)
    (out / 'SUMMARY.json').write_text(json.dumps(summ, indent=1)); print(json.dumps(summ, indent=1))
    sys.exit(0)
