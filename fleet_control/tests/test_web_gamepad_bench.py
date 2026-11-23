"""Manual bench fences and dry-run velocity selection, with controllable clocks."""
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

FLEET = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(FLEET / 'server'), str(FLEET / 'agent/limo_robot_agent')]
from fleet_server.service import ApiError, Fleet
from fleet_server.db import DB
from limo_robot_agent.velocity_arbiter import select_locked, select_nav
from limo_robot_agent.manual_web_bench import ManualWebBench
from limo_robot_agent.joystick_filter import JoystickConfig


class ManualBenchTest(unittest.TestCase):
    def setUp(self):
        db = DB(":memory:")
        self.addCleanup(db.c.close)
        self.fleet = Fleet(db, SimpleNamespace())
        self.agent = dict(mode='mock', simulated=True, missions_enabled=False, manual_web_bench=True, boot_id='boot', protocol_version=2,
                          safety_contract_version=1, locked_velocity_arbiter=True)
        self.nav = dict(known=True, publishers=1, active_count=0, received_status_array=True, age_s=0)
        self.robot = dict(agent=self.agent, state_ts=10, online=True, reconciled=True,
                          state=dict(execution=dict(nav2_goals=self.nav),
                                     velocity_arbiter=dict(mode="MOCK_LOCKED", ready=True)))
        self.fleet.robots['mock'] = self.robot
        self.lease = dict(owner='operator', epoch=1, expires_at=11)
        self.fleet.lease_view = lambda _: self.lease
        self.fleet.check_session = lambda *_: None
        self.addCleanup(patch.stopall)
        patch('fleet_server.service.time.time', lambda: 10.).start()
        self.clock = patch('fleet_server.service.time.monotonic', return_value=20.).start()

    def start(self):
        return self.fleet.manual_bench('mock', 'operator', dict(action='start', epoch=1))

    def sample(self, session, seq=0, **overrides):
        fields = dict(action='sample', input_format='normalized_axes_v1', epoch=1, seq=seq, linear=1., turn=-1., deadman=True, **session)
        fields.update(overrides)
        return self.fleet.manual_bench('mock', 'operator', fields)

    def pull(self):
        return self.fleet.pull_manual_bench('mock', self.agent['boot_id'])['sample']

    def test_axes_range_and_nonfinite_rejected(self):
        session = self.start()
        for fields in (dict(linear=-1.01), dict(linear=1.01), dict(turn=1.01), dict(turn=float('nan')), dict(turn=float('inf'))):
            with self.assertRaises(ApiError):
                self.sample(session, **fields)

    def test_release_fences_delayed_sample(self):
        session = self.start()
        self.assertEqual(self.sample(session), {"received": 0})
        self.fleet.manual_bench('mock', 'operator',
                                dict(action='stop', epoch=1, session_id=session['session_id']))
        self.assertIsNone(self.pull())
        with self.assertRaises(ApiError):
            self.sample(session, seq=1)

    def test_replay_and_transport_delay_rejected(self):
        session = self.start()
        self.assertEqual(self.sample(session), {"received": 0})
        with self.assertRaises(ApiError):
            self.sample(session, seq=0)
        self.sample(session, seq=2)
        with self.assertRaises(ApiError):
            self.sample(session, seq=1)
        self.clock.return_value = 20.26
        self.assertIsNone(self.pull())
        with self.assertRaises(ApiError):
            self.sample(session, seq=1)

    def test_lease_expiry_or_agent_restart_discards_old_preview(self):
        self.sample(self.start())
        self.lease = None
        self.assertIsNone(self.pull())
        self.lease = dict(owner='operator', epoch=2, expires_at=11)
        self.assertIsNone(self.pull())
        self.lease['epoch'] = 1
        self.agent['boot_id'] = 'new-boot'
        self.assertIsNone(self.pull())

    def test_nonterminal_and_unknown_nav_never_admit_manual_mode(self):
        for changes in (dict(active_count=1), dict(known=False), dict(publishers=2)):
            with self.subTest(changes=changes):
                original = self.nav.copy()
                self.nav.update(changes)
                with self.assertRaises(ApiError):
                    self.start()
                self.nav.update(original)

    def test_mock_requires_ready_locked_arbiter(self):
        self.agent['locked_velocity_arbiter'] = False
        with self.assertRaises(ApiError) as error:
            self.start()
        self.assertEqual(error.exception.code, 'manual_bench_unavailable')
        self.agent['locked_velocity_arbiter'] = True
        session = self.start()
        self.sample(session)
        self.robot['state']['velocity_arbiter']['ready'] = False
        with self.assertRaises(ApiError) as error:
            self.pull()
        self.assertEqual(error.exception.code, 'mock_arbiter_not_ready')
        self.assertNotIn('mock', self.fleet.manual_benches)

    def test_real_is_observation_only_and_missions_must_be_disabled(self):
        self.agent.update(mode='real', simulated=False, locked_velocity_arbiter=False)
        self.assertIn('session_id', self.start())
        self.agent['missions_enabled'] = True
        with self.assertRaises(ApiError):
            self.pull()

    def test_legacy_payload_rejected_without_advancing_sequence(self):
        session = self.start()
        for fields in ({'input_format': None}, {'ticket': 'old'}, {'input_format': 'm/s'}):
            with self.assertRaises(ApiError) as error:
                self.sample(session, **fields)
            self.assertEqual(error.exception.code, 'bad_joy_format')
        self.assertEqual(self.sample(session), {'received': 0})

    def test_mode_priority_and_deadman(self):
        manual = dict(requested_linear=.1, requested_turn=-.3, deadman=True, expires=21.)
        self.assertEqual(select_nav(allowed=True, sample=(20., .05, .2), now=20.), (.05, .2))
        self.assertEqual(select_locked(allowed=True, manual=manual, now=20.), (.1, -.3))
        for allowed, sample in ((False, manual), (True, None), (True, dict(manual, deadman=False))):
            self.assertEqual(select_locked(allowed=allowed, manual=sample, now=20.), (0., 0.))


class AgentBenchTest(unittest.TestCase):
    def test_generation_fences_cached_and_repolled_sample(self):
        from time import monotonic, time
        session = dict(connected=True, reconciliation_required=False, server_boot_id='server',
                       protocol_version=2, safety_contract_version=1, contract_generation=1,
                       lease=dict(owner='operator', epoch=1, expires_at=time() + 10))
        bench = ManualWebBench.__new__(ManualWebBench)
        bench.agent = SimpleNamespace(session_snapshot=lambda: session, boot_id='agent',
                                      is_execution_allowed=lambda: True)
        bench.robot = SimpleNamespace()
        bench.isolated_arbiter = True
        bench.config = JoystickConfig(1, 0, 4, .10, .30, .25)
        bench.lock = threading.Lock()
        bench.last_session, bench.session_authority, bench.last_sequence = None, None, -1
        sample = dict(session_id='one', seq=1, ttl_s=.20, linear=1., turn=-1., deadman=True)

        def consume(data):
            bench.sample = bench.accept_sample(data, bench.authority(), monotonic())
            bench.tick()
            return bench.snapshot()

        self.assertEqual(consume(sample)['requested_linear'], .1)
        old_authority = bench.authority()
        session['contract_generation'] += 1
        self.assertIsNone(bench.accept_sample(dict(sample, seq=2), old_authority, monotonic()))
        self.assertEqual(bench.snapshot()['requested_linear'], 0.)
        self.assertFalse(consume(dict(sample, seq=2))['deadman'])
        self.assertEqual(consume(dict(sample, session_id='two'))['requested_turn'], -.3)
        self.assertFalse(consume(dict(sample, session_id='two', seq=0))['deadman'])
        self.assertFalse(consume(dict(sample, session_id='two', seq=2, deadman=False))['deadman'])
        bench.sample = bench.accept_sample(dict(sample, session_id='two', seq=3),
                                           bench.authority(), monotonic() - 1.)
        bench.tick()
        self.assertEqual(bench.snapshot()['requested_linear'], 0.)


if __name__ == '__main__':
    unittest.main()
