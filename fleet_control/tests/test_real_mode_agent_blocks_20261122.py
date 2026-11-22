"""Real-mode blocks in the Agent (2026-11-22). No Fleet Server, no robot, no motor command.

1. web_gate_reset is refused at start-up unless the Agent is isolated mock/sim (explicit guard in agent_node.py).
2. manual_web_bench never creates a /cmd_vel_raw publisher in mode=real (it would be a second writer next to Nav2 on a Gate
   without writer binding), but still does in an isolated mock (negative control: the guard is not a global disable).

run:  cd fleet_control/tests && PYTHONPATH=../agent/limo_robot_agent:$PYTHONPATH python3 -m unittest test_real_mode_agent_blocks_20261122
(needs the ROS Humble environment for the subprocess part)
"""
import os
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from limo_robot_agent.manual_web_bench import ManualWebBench

AGENT_SRC = Path(__file__).resolve().parents[1] / 'agent' / 'limo_robot_agent'


class Timer:
    def cancel(self):
        pass


class Robot:
    def __init__(self, mode):
        self.mode = mode
        self.lock = threading.Lock()
        self.active = None
        self.gate = {'state': 'NORMAL', 'wheel_odom_stale': False, 'cmd_blocked': False}
        self.created = 0
        self.published = []

    def create_timer(self, *_):
        return Timer()

    def _age(self, _):
        return 0.01

    def create_publisher(self, *_):
        self.created += 1
        outer = self

        class Pub:
            def publish(self, m):
                outer.published.append((m.linear.x, m.angular.z))
        return Pub()

    def destroy_publisher(self, _):
        pass


class Http:
    def post(self, *_, **__):
        return {'sample': {'linear': 1., 'turn': 0., 'deadman': True, 'session_id': 't', 'seq': 1, 'ttl_s': .25}}


def _bench(mode):
    robot = Robot(mode)
    agent = SimpleNamespace(is_execution_allowed=lambda: True, http=Http(), robot_id='t', boot_id='b')
    bench = ManualWebBench(robot, agent)
    bench._graph_blocker = lambda: None          # the graph snapshot is clean: only the real-mode rule can stop the publisher
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        with bench.lock:
            if isinstance(bench.sample, dict):
                break
        time.sleep(.01)
    return robot, bench


class TestBenchRawPublisher(unittest.TestCase):
    def test_real_mode_never_creates_raw_publisher(self):
        robot, bench = _bench('real')
        try:
            for _ in range(5):
                bench.tick()
                time.sleep(.02)
            self.assertEqual(robot.created, 0)
            self.assertIsNone(bench.publisher)
            self.assertTrue(bench.state['raw_publisher_blocked_real'])
            self.assertFalse(bench.state['motor_output'])
        finally:
            bench.close()

    def test_mock_mode_control_still_creates_it(self):
        robot, bench = _bench('mock')
        try:
            bench.tick()
            self.assertEqual(robot.created, 1)
            self.assertFalse(bench.state['raw_publisher_blocked_real'])
        finally:
            bench.close()


class TestGateResetGuard(unittest.TestCase):
    def _run(self, **env):
        tok = tempfile.NamedTemporaryFile('w', suffix='.token', delete=False)
        tok.write('x\n'); tok.close(); os.chmod(tok.name, 0o600)
        state = tempfile.mkdtemp()
        e = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', **env)
        e['PYTHONPATH'] = f"{AGENT_SRC}:{e.get('PYTHONPATH', '')}"
        mode = e.pop('T_MODE')
        sim = e.pop('T_SIM')
        cmd = [sys.executable, '-c', 'from limo_robot_agent.agent_node import main; main()', '--ros-args',
               '-p', 'server_url:=http://127.0.0.1:1', '-p', f'token_file:={tok.name}', '-p', f'mode:={mode}',
               '-p', f'simulated:={sim}', '-p', 'allow_missions:=false', '-p', f'state_dir:={state}', '-p', 'web_gate_reset:=true']
        try:
            p = subprocess.run(cmd, env=e, capture_output=True, text=True, timeout=8)
            return p.returncode, p.stdout + p.stderr
        except subprocess.TimeoutExpired as exc:
            return None, ((exc.stdout or b'').decode() if isinstance(exc.stdout, bytes) else (exc.stdout or '')) + \
                         ((exc.stderr or b'').decode() if isinstance(exc.stderr, bytes) else (exc.stderr or ''))
        finally:
            os.unlink(tok.name)

    def test_real_mode_refused(self):
        rc, out = self._run(T_MODE='real', T_SIM='false', ROS_DOMAIN_ID='217', ROS_LOCALHOST_ONLY='1')
        self.assertEqual(rc, 1, out)
        self.assertIn('web_gate_reset requires SIM/mock simulated mode', out)

    def test_real_mode_domain_77_refused(self):
        rc, out = self._run(T_MODE='real', T_SIM='false', ROS_DOMAIN_ID='77', ROS_LOCALHOST_ONLY='1')
        self.assertEqual(rc, 1, out)
        self.assertIn('web_gate_reset requires', out)

    def test_isolated_mock_not_refused_by_this_guard(self):
        rc, out = self._run(T_MODE='mock', T_SIM='true', ROS_DOMAIN_ID='217', ROS_LOCALHOST_ONLY='1')
        self.assertNotIn('web_gate_reset requires', out)


def run_agent(mode, sim, domain, params=(), env_extra=None, timeout=8, localhost='1'):
    """Start the Agent entry point once with the given (possibly wrong) configuration. Returns (rc or None on timeout, output)."""
    tok = tempfile.NamedTemporaryFile('w', suffix='.token', delete=False)
    tok.write('x\n'); tok.close(); os.chmod(tok.name, 0o600)
    state = tempfile.mkdtemp()
    e = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', ROS_LOCALHOST_ONLY=localhost)
    e.pop('ROS_DISCOVERY_SERVER', None)
    if domain is None:
        e.pop('ROS_DOMAIN_ID', None)
    else:
        e['ROS_DOMAIN_ID'] = domain
    e.update(env_extra or {})
    e['PYTHONPATH'] = f"{AGENT_SRC}:{e.get('PYTHONPATH', '')}"
    cmd = [sys.executable, '-c', 'from limo_robot_agent.agent_node import main; main()', '--ros-args',
           '-p', 'server_url:=http://127.0.0.1:1', '-p', f'token_file:={tok.name}', '-p', f'mode:={mode}',
           '-p', f'simulated:={sim}', '-p', f'state_dir:={state}'] + [x for kv in params for x in ('-p', kv)]
    try:
        p = subprocess.run(cmd, env=e, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout + p.stderr
    except subprocess.TimeoutExpired as exc:
        f = lambda b: b.decode() if isinstance(b, bytes) else (b or '')
        return None, f(exc.stdout) + f(exc.stderr)
    finally:
        os.unlink(tok.name)


class TestConfigurationErrors(unittest.TestCase):
    """Wrong mode / domain / flag combinations must not reach the robot graph or enable motion paths."""

    def refused(self, msg, *a, **k):
        rc, out = run_agent(*a, **k)
        self.assertEqual(rc, 1, out[-400:])
        self.assertIn(msg, out)

    def test_real_with_missions_refused_on_every_domain(self):
        for dom in ('77', '214', None):
            self.refused('allow_missions is blocked in real mode', 'real', 'false', dom, ['allow_missions:=true'])

    def test_misspelled_or_odd_mode_refused(self):
        for m in ('Real', 'REAL', 'foo'):
            self.refused('mode must be exactly', m, 'false', '214', ['allow_missions:=true'])
        rc, out = run_agent('', 'false', '214', ['allow_missions:=true'])      # an empty value does not even parse
        self.assertNotEqual(rc, 0, out[-300:])
        # 'real ' (trailing space) is normalised to 'real' by the ROS parameter parser, so it is refused by the real-mode missions rule instead
        rc, out = run_agent('real ', 'false', '214', ['allow_missions:=true'])
        self.assertEqual(rc, 1, out[-300:])
        self.assertTrue('mode must be exactly' in out or 'allow_missions is blocked' in out, out[-300:])

    def test_real_with_simulated_refused(self):
        self.refused("cannot be combined with simulated:=true", 'real', 'true', '214')

    def test_sim_or_mock_on_the_real_robot_domain_refused(self):
        for m in ('sim', 'mock'):
            self.refused('must not run on the real robot graph', m, 'true', '77',
                         ['allow_missions:=true', 'web_mapping_enabled:=true'])

    def test_sim_or_mock_with_discovery_server_refused(self):
        self.refused('must not run on the real robot graph', 'mock', 'true', '214', env_extra={'ROS_DISCOVERY_SERVER': '127.0.0.1:11811'})

    def test_real_mode_other_motion_flags_refused(self):
        for flag in ('web_mapping_enabled:=true', 'manual_locked_arbiter:=true', 'navigation_localization:=true', 'manual_mapping_enabled:=true',
                     'auto_mapping_enabled:=true', 'slam_sim_enabled:=true', 'web_gate_reset:=true'):
            rc, out = run_agent('real', 'false', '214', [flag])
            self.assertEqual(rc, 1, f'{flag}: {out[-300:]}')

    def test_real_mode_localhost_off_blocks_bench(self):
        rc, out = run_agent('real', 'false', '214', ['manual_web_bench:=true'], localhost='0')
        self.assertEqual(rc, 1, out[-300:])

    def test_isolated_mock_control_not_refused_by_these_guards(self):
        rc, out = run_agent('mock', 'true', '217', ['allow_missions:=false'], timeout=6)
        for needle in ('allow_missions is blocked', 'mode must be exactly', 'must not run on the real robot graph', 'cannot be combined'):
            self.assertNotIn(needle, out)


if __name__ == '__main__':
    unittest.main(verbosity=2)
