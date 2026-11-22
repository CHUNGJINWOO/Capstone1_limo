#!/usr/bin/env python3
"""Agent graceful-stop matrix (2026-11-22). Laptop only: isolated ROS domain, loopback server, no robot, no motor.

usage: agent_shutdown_matrix_20261122.py --agent-src DIR --out OUTDIR [--domain 230] [--port 18293] [--only NAME,...]

For every combination of  scenario x signal x SIGINT-disposition  it starts a real-mode Agent (mode=real, allow_missions=false),
brings the server into the scenario state, sends ONE signal and records: exit within TIMEOUT_S or HUNG (then SIGKILL),
return code, time to exit, whether the final log shows a traceback, leftover processes, the Agent node still in the ROS graph,
and new /dev/shm fastrtps files. PASS = exits by itself within TIMEOUT_S, no traceback, node gone, no leftover.

scenarios:  up (connected) | down_start (server never reachable) | server_dies (connected, then server killed) |
            reconnect (connected, server killed and restarted, reconnected)
signals:    INT, TERM        SIGINT disposition: normal | ignored (what `nohup ... &` and `setsid ... &` from a non-interactive shell give)
"""
import argparse, json, os, shutil, signal, subprocess, sys, time
from pathlib import Path

TIMEOUT_S = 20.0
HERE = Path(__file__).resolve().parent
SERVER_SRC = HERE.parent / 'server'


def shm_names():
    try:
        return {p.name for p in Path('/dev/shm').iterdir() if 'fastrtps' in p.name or 'fastdds' in p.name}
    except OSError:
        return set()


def wait_for(path, needle, secs):
    t0 = time.time()
    while time.time() - t0 < secs:
        try:
            if needle in Path(path).read_text(errors='replace'):
                return True
        except OSError:
            pass
        time.sleep(0.25)
    return False


class Env:
    def __init__(self, a):
        self.a = a
        self.out = Path(a.out)
        self.out.mkdir(parents=True, exist_ok=False)
        self.data = self.out / 'server_data'
        self.sec = self.out / 'server_secrets'
        self.srv = None
        envbase = dict(os.environ, PYTHONPATH=str(SERVER_SRC) + ':' + os.environ.get('PYTHONPATH', ''), PYTHONDONTWRITEBYTECODE='1')
        subprocess.run([sys.executable, '-m', 'fleet_server', 'init', '--data-dir', str(self.data), '--secrets-dir', str(self.sec), '--robots', 'pcu'],
                       env=envbase, check=True, stdout=subprocess.DEVNULL)
        self.envbase = envbase

    def start_server(self, tag):
        log = open(self.out / f'server_{tag}.log', 'w')
        self.srv = subprocess.Popen([sys.executable, '-m', 'fleet_server', 'serve', '--data-dir', str(self.data), '--port', str(self.a.port), '--offline-after', '6'],
                                    env=self.envbase, stdout=log, stderr=subprocess.STDOUT, cwd=str(SERVER_SRC), start_new_session=True)
        for _ in range(40):
            if wait_for(self.out / f'server_{tag}.log', 'listening', 0.25):
                return
        raise RuntimeError('server did not start')

    def stop_server(self):
        if self.srv and self.srv.poll() is None:
            self.srv.send_signal(signal.SIGINT)
            try:
                self.srv.wait(8)
            except subprocess.TimeoutExpired:
                self.srv.kill(); self.srv.wait()
        self.srv = None

    def start_agent(self, tag, sigint_ignored):
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', ROS_DOMAIN_ID=str(self.a.domain), ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
        env['PYTHONPATH'] = f"{self.a.agent_src}:{env.get('PYTHONPATH', '')}"
        state = self.out / f'state_{tag}'
        state.mkdir()
        cmd = [sys.executable, '-c', 'from limo_robot_agent.agent_node import main; main()', '--ros-args', '-p', f'server_url:=http://127.0.0.1:{self.a.port}',
               '-p', f'token_file:={self.sec}/agent-pcu.token', '-p', 'robot_id:=pcu', '-p', 'mode:=real', '-p', 'simulated:=false',
               '-p', 'allow_missions:=false', '-p', f'state_dir:={state}', '-p', 'period_s:=0.5']
        pre = (lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)) if sigint_ignored else None
        log = self.out / f'agent_{tag}.log'
        return subprocess.Popen(cmd, env=env, stdout=open(log, 'w'), stderr=subprocess.STDOUT, start_new_session=True, preexec_fn=pre), log


def ros_nodes(domain):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1')
    try:
        r = subprocess.run(['ros2', 'node', 'list', '--no-daemon'], env=env, capture_output=True, text=True, timeout=25)
        return r.stdout.split()
    except Exception:
        return ['?']


def run_case(E, scenario, sig, ign, idx):
    tag = f'{idx:02d}_{scenario}_{sig}_{"ign" if ign else "norm"}'
    res = dict(case=tag, scenario=scenario, signal=sig, sigint_ignored=ign)
    shm0 = shm_names()
    if scenario != 'down_start':
        E.start_server(tag + '_a')
    proc, log = E.start_agent(tag, ign)
    ok_start = True
    if scenario == 'down_start':
        ok_start = wait_for(log, 'server unreachable', 25)
    else:
        ok_start = wait_for(log, 'connected and reconciled', 40)
        if ok_start and scenario in ('server_dies', 'reconnect'):
            E.stop_server()
            ok_start = wait_for(log, 'server unreachable', 25)
            if ok_start and scenario == 'reconnect':
                E.start_server(tag + '_b')
                ok_start = wait_for(log, 'reconnected', 40)
    res['reached_state'] = ok_start
    if not ok_start:
        res['result'] = 'SETUP_FAIL'
    else:
        s = signal.SIGINT if sig == 'INT' else signal.SIGTERM
        t0 = time.time()
        os.kill(proc.pid, s)
        try:
            rc = proc.wait(TIMEOUT_S)
            res.update(exited=True, rc=rc, exit_s=round(time.time() - t0, 2))
        except subprocess.TimeoutExpired:
            res.update(exited=False, rc=None, exit_s=None)
            proc.kill(); proc.wait()
        text = log.read_text(errors='replace')
        res['traceback'] = 'Traceback' in text
        res['tail'] = text.strip().splitlines()[-3:]
    E.stop_server()
    time.sleep(1.0)
    res['ghost_nodes'] = [n for n in ros_nodes(E.a.domain) if 'fleet' in n or 'agent' in n]
    res['new_shm'] = sorted(shm_names() - shm0)
    if res.get('result') != 'SETUP_FAIL':
        res['result'] = 'PASS' if (res['exited'] and not res['traceback'] and not res['ghost_nodes'] and not res['new_shm']) else 'FAIL'
    print(json.dumps({k: res[k] for k in ('case', 'result', 'rc', 'exit_s', 'traceback', 'ghost_nodes', 'new_shm') if k in res}), flush=True)
    return res


def _proc_env(a):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', ROS_DOMAIN_ID=str(a.domain), ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    for k in ('http_proxy', 'https_proxy', 'HTTP_PROXY', 'HTTPS_PROXY'):
        env.pop(k, None)
    env['PYTHONPATH'] = f"{a.agent_src}:{env.get('PYTHONPATH', '')}"
    return env


def run_watchdog_cases(E, only):
    env = _proc_env(E.a)
    cases = [
        ('wd_INT_norm', 'INT', False, 'normal'),
        ('wd_INT_ign', 'INT', True, 'normal'),
        ('wd_TERM_norm', 'TERM', False, 'normal'),
        ('wd_TERM_dup', 'TERM', False, 'duplicate'),
        ('wd_abnormal_ext_shutdown', None, False, 'ext_shutdown'),
        ('wd_abnormal_internal_exc', None, False, 'internal_exc'),
    ]
    results = []
    for name, sig, ign, mode in cases:
        if only and name not in only:
            continue
        res = dict(case=name, target='motion_watchdog', mode=mode, signal=sig, sigint_ignored=ign)
        shm0 = shm_names()
        log = E.out / f'{name}.log'
        if mode == 'ext_shutdown':
            code = (
                'import sys, rclpy\n'
                'from limo_robot_agent import motion_watchdog as mw\n'
                'orig = mw.MotionWatchdog.__init__\n'
                'def patched(self):\n'
                '    orig(self)\n'
                '    self.create_timer(0.4, lambda: rclpy.try_shutdown())\n'
                'mw.MotionWatchdog.__init__ = patched\n'
                'sys.exit(mw.main())\n'
            )
            cmd = [sys.executable, '-c', code]
        elif mode == 'internal_exc':
            code = (
                'import sys\n'
                'from limo_robot_agent import motion_watchdog as mw\n'
                'orig = mw.MotionWatchdog.tick\n'
                'n = {"c": 0}\n'
                'def patched(self):\n'
                '    n["c"] += 1\n'
                '    if n["c"] >= 3:\n'
                '        raise RuntimeError("injected_watchdog_fault")\n'
                '    return orig(self)\n'
                'mw.MotionWatchdog.tick = patched\n'
                'sys.exit(mw.main())\n'
            )
            cmd = [sys.executable, '-c', code]
        else:
            cmd = [sys.executable, '-m', 'limo_robot_agent.motion_watchdog']
        pre = (lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)) if ign else None
        t0 = time.time()
        proc = subprocess.Popen(cmd, env=env, stdout=open(log, 'w'), stderr=subprocess.STDOUT, start_new_session=True, preexec_fn=pre)
        if mode in ('normal', 'duplicate'):
            time.sleep(1.0)
            s = signal.SIGINT if sig == 'INT' else signal.SIGTERM
            t0 = time.time()
            os.kill(proc.pid, s)
            if mode == 'duplicate':
                time.sleep(0.02)
                try:
                    os.kill(proc.pid, s)
                except ProcessLookupError:
                    pass
        try:
            rc = proc.wait(TIMEOUT_S)
            res.update(exited=True, rc=rc, exit_s=round(time.time() - t0, 3))
        except subprocess.TimeoutExpired:
            res.update(exited=False, rc=None, exit_s=None)
            proc.kill(); proc.wait()
        text = log.read_text(errors='replace')
        res['traceback'] = 'Traceback' in text
        res['tail'] = text.strip().splitlines()[-4:]
        time.sleep(0.6)
        res['ghost_nodes'] = [n for n in ros_nodes(E.a.domain) if 'watchdog' in n or 'fleet' in n]
        res['new_shm'] = sorted(shm_names() - shm0)
        if mode in ('normal', 'duplicate'):
            ok = res['exited'] and res['rc'] == 0 and not res['traceback'] and not res['ghost_nodes'] and not res['new_shm'] and 'stopped cleanly' in text
        elif mode == 'ext_shutdown':
            ok = res['exited'] and res['rc'] != 0 and 'ABNORMAL_EXTERNAL_SHUTDOWN' in text and not res['ghost_nodes'] and not res['new_shm']
        else:
            ok = res['exited'] and res['rc'] != 0 and 'ABNORMAL_EXCEPTION' in text and 'injected_watchdog_fault' in text and not res['ghost_nodes'] and not res['new_shm']
        res['result'] = 'PASS' if ok else 'FAIL'
        print(json.dumps({k: res[k] for k in ('case', 'result', 'rc', 'exit_s', 'traceback', 'ghost_nodes', 'new_shm') if k in res}), flush=True)
        results.append(res)
    return results


def run_mock_robot_cases(E, only):
    env = _proc_env(E.a)
    cases = [('mock_INT_norm', 'INT', False), ('mock_INT_ign', 'INT', True), ('mock_TERM_norm', 'TERM', False)]
    results = []
    for name, sig, ign in cases:
        if only and name not in only:
            continue
        res = dict(case=name, target='mock_robot', signal=sig, sigint_ignored=ign)
        shm0 = shm_names()
        log = E.out / f'{name}.log'
        cmd = [sys.executable, '-c', 'from limo_robot_agent.mock_robot_node import main; main()']
        pre = (lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)) if ign else None
        proc = subprocess.Popen(cmd, env=env, stdout=open(log, 'w'), stderr=subprocess.STDOUT, start_new_session=True, preexec_fn=pre)
        ok_start = wait_for(log, 'mock robot up', 15)
        res['reached_state'] = ok_start
        if not ok_start:
            proc.kill(); proc.wait()
            res['result'] = 'SETUP_FAIL'
        else:
            s = signal.SIGINT if sig == 'INT' else signal.SIGTERM
            t0 = time.time()
            os.kill(proc.pid, s)
            try:
                rc = proc.wait(TIMEOUT_S)
                res.update(exited=True, rc=rc, exit_s=round(time.time() - t0, 3))
            except subprocess.TimeoutExpired:
                res.update(exited=False, rc=None, exit_s=None)
                proc.kill(); proc.wait()
            text = log.read_text(errors='replace')
            res['traceback'] = 'Traceback' in text
            res['tail'] = text.strip().splitlines()[-3:]
        time.sleep(0.6)
        res['ghost_nodes'] = [n for n in ros_nodes(E.a.domain) if 'mock_robot' in n]
        res['new_shm'] = sorted(shm_names() - shm0)
        if res.get('result') != 'SETUP_FAIL':
            res['result'] = 'PASS' if (res['exited'] and res['rc'] == 0 and not res['traceback'] and not res['ghost_nodes'] and not res['new_shm']) else 'FAIL'
        print(json.dumps({k: res[k] for k in ('case', 'result', 'rc', 'exit_s', 'traceback', 'ghost_nodes', 'new_shm') if k in res}), flush=True)
        results.append(res)
    return results


def run_supervisor_cases(E, only):
    env = _proc_env(E.a)
    cases = [('sup_INT_norm', 'INT', False), ('sup_INT_ign', 'INT', True), ('sup_TERM_norm', 'TERM', False)]
    results = []
    for name, sig, ign in cases:
        if only and name not in only:
            continue
        res = dict(case=name, target='supervisor', signal=sig, sigint_ignored=ign)
        log = E.out / f'{name}.log'
        sdir = E.out / f'{name}_logs'
        sdir.mkdir()
        cfg_path = E.out / f'{name}_cfg.json'
        marker = f'sup_dummy_child_{name}_{os.getpid()}'
        cfg = {
            'robot_id': 'pcu',
            'server_url': f'http://127.0.0.1:{E.a.port}',
            'token_file': str(E.sec / 'agent-pcu.token'),
            'report_period_s': 0.5,
            'services': [{
                'name': 'dummy',
                'cmd': [sys.executable, '-c', f'import signal, sys, time; signal.signal(signal.SIGTERM, lambda *_: sys.exit(0)); signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); time.sleep(60)  # {marker}'],
                'ready': {'after_s': 0.2},
                'restart': {'max': 1, 'window_s': 30, 'backoff_s': 1},
            }],
        }
        cfg_path.write_text(json.dumps(cfg))
        cmd = [sys.executable, '-m', 'limo_robot_agent.supervisor', '--config', str(cfg_path), '--log-dir', str(sdir)]
        pre = (lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)) if ign else None
        proc = subprocess.Popen(cmd, env=env, stdout=open(log, 'w'), stderr=subprocess.STDOUT, start_new_session=True, preexec_fn=pre)
        time.sleep(1.0)
        s = signal.SIGINT if sig == 'INT' else signal.SIGTERM
        t0 = time.time()
        os.kill(proc.pid, s)
        try:
            rc = proc.wait(TIMEOUT_S)
            res.update(exited=True, rc=rc, exit_s=round(time.time() - t0, 3))
        except subprocess.TimeoutExpired:
            res.update(exited=False, rc=None, exit_s=None)
            proc.kill(); proc.wait()
        text = log.read_text(errors='replace')
        res['traceback'] = 'Traceback' in text
        chk = subprocess.run(['pgrep', '-f', marker], capture_output=True, text=True)
        res['leftover_children'] = [x for x in chk.stdout.split() if x.strip()]
        res['tail'] = text.strip().splitlines()[-3:]
        res['result'] = 'PASS' if (res['exited'] and res['rc'] == 0 and not res['traceback'] and not res['leftover_children']) else 'FAIL'
        print(json.dumps({k: res[k] for k in ('case', 'result', 'rc', 'exit_s', 'traceback', 'leftover_children') if k in res}), flush=True)
        results.append(res)
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--agent-src', required=True); ap.add_argument('--out', required=True)
    ap.add_argument('--domain', type=int, default=230); ap.add_argument('--port', type=int, default=18293)
    ap.add_argument('--only', default='')
    ap.add_argument('--target', choices=('agent', 'motion_watchdog', 'mock_robot', 'supervisor'), default='agent')
    a = ap.parse_args()
    if a.port in (8080, 18190, 18191, 18192, 18290) or a.domain in (0, 77):
        sys.exit('refusing a production-like port or domain')
    E = Env(a)
    only = [x for x in a.only.split(',') if x]
    if a.target == 'motion_watchdog':
        results = run_watchdog_cases(E, only)
    elif a.target == 'mock_robot':
        results = run_mock_robot_cases(E, only)
    elif a.target == 'supervisor':
        results = run_supervisor_cases(E, only)
    else:
        cases = []
        for sc in ('up', 'down_start', 'server_dies', 'reconnect'):
            for sig, ign in (('INT', False), ('INT', True), ('TERM', False)):
                cases.append((sc, sig, ign))
        results = []
        for i, (sc, sig, ign) in enumerate(cases, 1):
            name = f'{sc}_{sig}_{"ign" if ign else "norm"}'
            if only and name not in only:
                continue
            results.append(run_case(E, sc, sig, ign, i))
    summ = {k: sum(1 for r in results if r['result'] == k) for k in ('PASS', 'FAIL', 'SETUP_FAIL')}
    json.dump({'target': a.target, 'agent_src': a.agent_src, 'results': results, 'summary': summ}, open(E.out / 'result.json', 'w'), indent=1)
    print('SUMMARY', summ)
    return 1 if summ['FAIL'] or summ['SETUP_FAIL'] else 0


if __name__ == '__main__':
    sys.exit(main())
