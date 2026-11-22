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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--agent-src', required=True); ap.add_argument('--out', required=True)
    ap.add_argument('--domain', type=int, default=230); ap.add_argument('--port', type=int, default=18293)
    ap.add_argument('--only', default='')
    a = ap.parse_args()
    if a.port in (8080, 18190, 18191, 18192, 18290) or a.domain in (0, 77):
        sys.exit('refusing a production-like port or domain')
    E = Env(a)
    cases = []
    for sc in ('up', 'down_start', 'server_dies', 'reconnect'):
        for sig, ign in (('INT', False), ('INT', True), ('TERM', False)):
            cases.append((sc, sig, ign))
    only = [x for x in a.only.split(',') if x]
    results = []
    for i, (sc, sig, ign) in enumerate(cases, 1):
        name = f'{sc}_{sig}_{"ign" if ign else "norm"}'
        if only and name not in only:
            continue
        results.append(run_case(E, sc, sig, ign, i))
    summ = {k: sum(1 for r in results if r['result'] == k) for k in ('PASS', 'FAIL', 'SETUP_FAIL')}
    json.dump({'agent_src': a.agent_src, 'results': results, 'summary': summ}, open(E.out / 'result.json', 'w'), indent=1)
    print('SUMMARY', summ)
    return 1 if summ['FAIL'] or summ['SETUP_FAIL'] else 0


if __name__ == '__main__':
    sys.exit(main())
