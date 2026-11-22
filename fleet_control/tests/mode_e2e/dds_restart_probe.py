"""SIMULATION ONLY (2026-11-21): do SIGKILL + restart cycles of the isolated stack lose /scan or Nav2 lifecycle replies?

Uses the unmodified isolated kinematic stack (stack_up.sh / stack_proc.sh: Fleet server, kinematic_sim, ICP odom + Nav2 + Gate r12,
Motion Watchdog, mock Agent) and tests/gazebo_web/timeline_recorder.py. Per cycle: SIGKILL the `stack` and `agent` process groups
(what stack_proc.sh kill does, and what the Phase 5 Nav2 fallback / a Supervisor restart does), start them again, then observe for
OBS_S seconds: Gate reaches NORMAL?, any later scan/lidar_odom NO_ARRIVAL or sensor-timeout fault, all Nav2 servers 'active'.
The transport is whatever the caller's environment selects (default Fast DDS = UDP + shared memory; FASTRTPS_DEFAULT_PROFILES_FILE
can select UDP only). Writes <out>/cycles.json. Never 18190, never the LIMO, ROS_LOCALHOST_ONLY=1, private domain.

usage: OVERLAY=<explore_lite overlay> python3 dds_restart_probe.py <new out dir> <port> <domain> [--cycles 6] [--obs 50]
"""
import json, os, signal, subprocess, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parent; FC = HERE.parents[1]
OUT = Path(sys.argv[1]).resolve(); PORT, DOM = sys.argv[2], sys.argv[3]
CYC = int(sys.argv[sys.argv.index('--cycles') + 1]) if '--cycles' in sys.argv else 6
OBS = float(sys.argv[sys.argv.index('--obs') + 1]) if '--obs' in sys.argv else 50.
assert PORT not in ('8080', '18190', '18191', '18192') and DOM not in ('0', '77', '231')
OUT.mkdir(parents=True, exist_ok=False); RUN = OUT / 'stack'; TL = OUT / 'timeline.jsonl'
subprocess.run([str(HERE / 'stack_up.sh'), str(RUN), PORT, DOM], check=True, stdout=(OUT / 'stack_up.log').open('x'), stderr=subprocess.STDOUT)
env = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=DOM); env.pop('ROS_DISCOVERY_SERVER', None)
rec = subprocess.Popen([sys.executable, str(FC / 'tests/gazebo_web/timeline_recorder.py'), str(TL), f'http://127.0.0.1:{PORT}',
                        str(RUN / 'secrets/operator.token'), 'iso_bench'], env=env, stdout=(OUT / 'timeline.log').open('x'), stderr=subprocess.STDOUT)
proc = lambda op, name: subprocess.run([str(HERE / 'stack_proc.sh'), str(RUN), op, name], check=False)


def rows(t0, t1):
    out = []
    for line in TL.read_text().splitlines():
        try: d = json.loads(line)
        except ValueError: continue
        if t0 <= d['m'] <= t1: out.append(d)
    return out


def judge(t0, t1, label):
    rs = rows(t0, t1); gate = [r for r in rs if r['k'] == 'gate']; nav = [r for r in rs if r['k'] == 'nav2']; scans = [r['m'] for r in rs if r['k'] == 'scan']
    t_norm = next((r['m'] for r in gate if r.get('state') == 'NORMAL'), None)
    faults = sorted({f for r in gate if t_norm and r['m'] > t_norm for f in (r.get('fault_reasons') or [])})
    after_norm = [r.get('state') for r in gate if t_norm and r['m'] > t_norm]
    core = ('controller_server', 'planner_server', 'bt_navigator', 'behavior_server', 'smoother_server', 'velocity_smoother')
    t_nav = next((r['m'] for r in nav if all((r['s'] or {}).get(s) == 'active' for s in core)), None)
    gaps = [round(b - a, 3) for a, b in zip(scans, scans[1:])]
    return {'label': label, 'gate_normal_after_s': None if t_norm is None else round(t_norm - t0, 1),
            'states_after_normal': sorted(set(after_norm)), 'fault_reasons_after_normal': faults,
            'nav2_all_active_after_s': None if t_nav is None else round(t_nav - t0, 1),
            'nav2_last': (nav[-1]['s'] if nav else None), 'scan_msgs_at_recorder': len(scans), 'scan_gap_max_s': max(gaps, default=None),
            'verdict': 'OK' if t_norm and not [s for s in after_norm if s != 'NORMAL'] and t_nav else 'DEGRADED'}


cycles = []
try:
    t0 = time.monotonic(); time.sleep(OBS); cycles.append(judge(t0, time.monotonic(), 'boot'))
    for i in range(CYC):
        t0 = time.monotonic()
        for n in ('agent', 'stack'): proc('kill', n)
        time.sleep(2.0)
        for n in ('stack', 'agent'): proc('start', n); time.sleep(3.0)
        time.sleep(OBS); c = judge(t0, time.monotonic(), f'restart{i + 1}'); cycles.append(c); print(json.dumps(c), flush=True)
finally:
    (OUT / 'cycles.json').write_text(json.dumps({'env': {k: os.environ.get(k) for k in ('RMW_IMPLEMENTATION', 'FASTRTPS_DEFAULT_PROFILES_FILE')},
                                                 'domain': DOM, 'cycles': cycles}, indent=1))
    rec.send_signal(signal.SIGINT)
    try: rec.wait(10)
    except subprocess.TimeoutExpired: rec.kill()
    proc('stopall', '-')
