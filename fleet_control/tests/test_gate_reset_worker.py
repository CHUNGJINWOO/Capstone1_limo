"""Agent side of the admin web Gate reset (no ROS graph: fakes for the robot node, the Agent session and the Gate service)."""
import json
import sys
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agent' / 'limo_robot_agent'))
from limo_robot_agent import gate_reset_worker as grw  # noqa: E402

WD_PREFIX = '010f025b0000aaaa00000000'


def ep(node, gid_hex):
    return types.SimpleNamespace(node_name=node, node_namespace='/', endpoint_gid=list(bytes.fromhex(gid_hex)) if gid_hex else [0] * 24)


class FakeFuture:
    def __init__(self, res): self.res = res
    def done(self): return True
    def result(self): return self.res
    def cancel(self): pass


class FakeClient:
    def __init__(self, test): self.test, self.calls, self.available, self.answer = test, 0, True, (True, 'Latch cleared by operator')
    def wait_for_service(self, timeout_sec=None): return self.available
    def call_async(self, req):
        self.calls += 1
        self.test.journal_at_call = json.loads(self.test.worker.journal_path.read_text())   # the journal must already hold the id
        return FakeFuture(types.SimpleNamespace(success=self.answer[0], message=self.answer[1]))


class FakeRobot:
    def __init__(self):
        self.lock = threading.Lock(); self.agent_boot_id = 'agentboot1'; self.active = None
        self.gate = {'boot_id': 'gateboot', 'fault_epoch': 3, 'reset_readiness': {'enforced': True, 'ready': True, 'blockers': []}}
        self.gate_t = time.monotonic()
        self.motion_watchdog = (time.monotonic(), {'state': 'MOTION_AUTHORIZED', 'agent_boot_id': 'agentboot1', 'output_gid': WD_PREFIX + '00001203' + '0' * 16})
        self.graph = {'/cmd_vel_raw': [ep('fleet_motion_watchdog', WD_PREFIX + '00001203' + '0' * 16)],
                      '/fleet/motion_watchdog/status': [ep('fleet_motion_watchdog', WD_PREFIX + '00001403' + '0' * 16)]}
        self.idle = {'ok': True, 'reason': None}
    def _age(self, key): return time.monotonic() - self.gate_t
    def get_publishers_info_by_topic(self, t): return list(self.graph.get(t, []))
    def get_subscriptions_info_by_topic(self, t): return []
    def manual_idle_evidence(self): return dict(self.idle)


class FakeAgent:
    def __init__(self):
        self.boot_id = 'agentboot1'
        self.sess = {'connected': True, 'reconciliation_required': False, 'last_state_age_s': 0.1, 'server_boot_id': 'srvboot',
                     'lease': {'owner': 'admin', 'epoch': 7, 'expires_at': time.time() + 60}, 'operating_mode': {'mode': 'IDLE', 'transition': {'state': 'ACTIVE'}},
                     'gate_reset_request': None}
    def session_snapshot(self): return json.loads(json.dumps(self.sess))


class FakeArbiter:
    def __init__(self): self.locked = True; self.snap = {'selected_linear': 0., 'selected_turn': 0., 'source': 'NONE'}
    def graph_locked(self): return self.locked
    def snapshot(self): return dict(self.snap)


class Worker(unittest.TestCase):
    def setUp(self):
        grw.OBSERVE_S = 0.0                                   # single evaluation; the window has its own test
        self.dir = tempfile.mkdtemp(prefix='grw-')
        self.robot, self.agent, self.arb = FakeRobot(), FakeAgent(), FakeArbiter()
        self.bench = types.SimpleNamespace(lock=threading.Lock(), sample=None)
        self.client = FakeClient(self)
        self.worker = self.new_worker()
        self.n = 0

    def new_worker(self):
        w = grw.GateResetWorker(self.robot, self.agent, self.dir, arbiter=self.arb, bench=self.bench, trigger_client=self.client)
        w.stop.set(); w.thread.join(timeout=1)          # tests drive handle() directly
        return w

    def tearDown(self): self.worker.close()

    def req(self, **kw):
        self.n += 1
        r = {'request_id': f'req{self.n:06d}aa', 'requested_by': 'admin', 'lease_epoch': 7, 'server_boot_id': 'srvboot',
             'agent_boot_id': 'agentboot1', 'gate_boot_id': 'gateboot', 'fault_epoch': 3}
        r.update(kw); return r

    def refused(self, check, **kw):
        rec = self.worker.handle(self.req(**kw))
        self.assertEqual(rec['status'], 'REFUSED'); self.assertFalse(rec['gate_called']); self.assertFalse(rec['checks'][check]['ok'], check)
        self.assertIn(check, rec['reason']); return rec

    def test_success_calls_gate_once_journal_first_and_drops_bench_sample(self):
        self.bench.sample = None
        r = self.req(); rec = self.worker.handle(r)
        self.assertEqual(rec['status'], 'SUCCEEDED'); self.assertEqual(self.client.calls, 1)
        self.assertEqual(self.journal_at_call['executed'][-1], {**self.journal_at_call['executed'][-1], 'request_id': r['request_id'], 'outcome': 'CALLING'})
        self.assertTrue(all(v['ok'] for v in rec['checks'].values()))

    def test_replay_of_same_request_refused_also_after_worker_restart(self):
        r = self.req(); self.assertEqual(self.worker.handle(r)['status'], 'SUCCEEDED')
        rec = self.worker.handle(dict(r)); self.assertEqual(rec['status'], 'REFUSED'); self.assertIn('request_new', rec['reason'])
        self.worker.close(); self.worker = self.new_worker()                      # new process, same state_dir
        rec = self.worker.handle(dict(r)); self.assertFalse(rec['checks']['request_new']['ok']); self.assertEqual(self.client.calls, 1)

    def test_boots_session_lease_mode(self):
        self.refused('agent_boot', agent_boot_id='otherboot')
        self.refused('server_boot', server_boot_id='oldserver')
        self.agent.sess['reconciliation_required'] = True; self.refused('session'); self.agent.sess['reconciliation_required'] = False
        self.refused('lease', lease_epoch=6)
        self.agent.sess['operating_mode'] = {'mode': 'MANUAL'}; self.refused('mode_idle')
        self.agent.sess['operating_mode'] = {'mode': 'IDLE', 'transition': {'state': 'VERIFYING_IDLE'}}; self.refused('mode_idle')

    def test_gate_identity_readiness_freshness(self):
        self.refused('gate_identity', fault_epoch=2)
        self.robot.gate['reset_readiness'] = {'enforced': True, 'ready': False, 'blockers': ['INPUT_WRITERS:2']}
        rec = self.refused('gate_ready'); self.assertEqual(rec['checks']['gate_ready']['detail'], ['INPUT_WRITERS:2'])
        del self.robot.gate['reset_readiness']
        rec = self.refused('gate_ready'); self.assertEqual(rec['checks']['gate_ready']['detail'], 'GATE_RESET_READINESS_UNSUPPORTED')
        self.robot.gate['reset_readiness'] = {'enforced': True, 'ready': True, 'blockers': []}
        self.robot.gate_t = time.monotonic() - 2; self.refused('gate_fresh')

    def test_watchdog_state_and_freshness(self):
        t, d = self.robot.motion_watchdog
        self.robot.motion_watchdog = (t, dict(d, state='LOCKED')); self.refused('watchdog')
        self.robot.motion_watchdog = (time.monotonic() - 3, d); self.refused('watchdog')
        self.robot.motion_watchdog = (time.monotonic(), dict(d, agent_boot_id='old')); self.refused('watchdog')

    def test_ghost_endpoint_and_unclear_origin_refused(self):
        g = self.robot.graph
        g['/cmd_vel_raw'] = g['/cmd_vel_raw'] + [ep('fleet_motion_watchdog', '010f025b9999bbbb00000000000012030000000000000000')]
        self.refused('publisher_identity')                                         # lingering endpoint of a killed watchdog
        g['/cmd_vel_raw'] = [ep('fleet_motion_watchdog', '010f025b9999bbbb00000000000012030000000000000000')]
        self.refused('publisher_identity')                                         # GID differs from the watchdog's own report
        g['/cmd_vel_raw'] = [ep('fleet_motion_watchdog', None)]; self.refused('publisher_identity')     # unreadable GID
        g['/cmd_vel_raw'] = [ep('teleop', self.robot.motion_watchdog[1]['output_gid'])]; self.refused('publisher_identity')
        g['/cmd_vel_raw'] = [ep('fleet_motion_watchdog', self.robot.motion_watchdog[1]['output_gid'])]
        g['/fleet/motion_watchdog/status'] = [ep('fleet_motion_watchdog', '010f025b7777cccc00000000000014030000000000000000')]
        self.refused('publisher_identity')                                         # writer and status from different participants
        g['/fleet/motion_watchdog/status'] = []; self.refused('publisher_identity')

    def test_graph_and_zero_velocity(self):
        self.arb.locked = False; self.refused('graph'); self.arb.locked = True
        self.arb.snap = {'selected_linear': .05, 'selected_turn': 0., 'source': 'MANUAL'}; self.refused('zero_velocity')
        self.arb.snap = {'selected_linear': 0., 'selected_turn': 0., 'source': 'NONE'}
        self.bench.sample = {'linear': 1.}; self.refused('zero_velocity'); self.bench.sample = None
        self.robot.active = {'goal_uuid': 'x', 'terminal_status': None}; self.refused('zero_velocity'); self.robot.active = None
        self.robot.idle = {'ok': False, 'reason': 'FOLLOW_PATH_GOAL_ACTIVE'}; self.refused('zero_velocity')

    def test_gate_refusal_and_service_failures(self):
        self.client.answer = (False, 'REFUSED: MOTION_COMMANDED')
        rec = self.worker.handle(self.req()); self.assertEqual(rec['status'], 'REFUSED'); self.assertEqual(rec['reason'], 'GATE:REFUSED: MOTION_COMMANDED')
        self.assertEqual(json.loads(self.worker.journal_path.read_text())['executed'][-1]['outcome'], 'REFUSED')
        self.client.available = False
        rec = self.worker.handle(self.req()); self.assertEqual(rec['status'], 'FAILED'); self.assertEqual(rec['reason'], 'GATE_RESET_SERVICE_UNAVAILABLE')

    def test_observation_window_needs_a_live_watchdog(self):
        grw.OBSERVE_S = 0.4
        t, d = self.robot.motion_watchdog
        self.robot.motion_watchdog = (time.monotonic(), dict(d, t=1.0))        # status never advances (killed writer's last message)
        self.robot.gate_t = time.monotonic() + 10                               # Gate stays "fresh" for this test
        orig_age = self.robot._age; self.robot._age = lambda k: 0.0
        rec = self.worker.handle(self.req()); self.assertEqual(rec['status'], 'REFUSED'); self.assertIn('observation', rec['reason'])
        self.assertEqual(self.client.calls, 0)
        stop = threading.Event()
        def tick():
            i = 0
            while not stop.is_set():
                i += 1; self.robot.motion_watchdog = (time.monotonic(), dict(d, t=float(i))); time.sleep(.05)
        th = threading.Thread(target=tick, daemon=True); th.start()
        try:
            rec = self.worker.handle(self.req()); self.assertEqual(rec['status'], 'SUCCEEDED', rec.get('reason'))
            self.assertGreaterEqual(rec['checks']['observation']['detail']['watchdog_status_updates'], 2)
        finally:
            stop.set(); th.join(); self.robot._age = orig_age; grw.OBSERVE_S = 0.0

    def test_old_request_refused(self):
        r = self.req(); self.worker.first_seen[r['request_id']] = time.monotonic() - 6
        rec = self.worker.handle(r); self.assertFalse(rec['checks']['request_age']['ok']); self.assertEqual(self.client.calls, 0)

    def test_run_loop_handles_a_delivered_request_once(self):
        w = grw.GateResetWorker(self.robot, self.agent, tempfile.mkdtemp(prefix='grw2-'), arbiter=self.arb, bench=self.bench, trigger_client=self.client)
        try:
            self.worker = w
            self.agent.sess['gate_reset_request'] = self.req()
            time.sleep(.5)                                                         # the request stays in the session for several loops
            self.assertEqual(self.client.calls, 1); self.assertEqual(w.snapshot()['last']['status'], 'SUCCEEDED')
        finally:
            w.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
