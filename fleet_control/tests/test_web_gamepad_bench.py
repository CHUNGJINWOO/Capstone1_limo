"""Manual bench fences and dry-run velocity selection, with controllable clocks."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

FLEET = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(FLEET / 'server'), str(FLEET / 'agent/limo_robot_agent')]
from fleet_server.service import ApiError, Fleet
from limo_robot_agent.manual_web_bench import select_velocity


class ManualBenchTest(unittest.TestCase):
    def setUp(self):
        self.fleet = Fleet(SimpleNamespace(q=lambda *args: [], one=lambda *args: None, event=lambda *args: None), None)
        self.agent = dict(mode='mock', simulated=True, missions_enabled=False, manual_web_bench=True, boot_id='boot')
        self.nav = dict(known=True, publishers=1, active_count=0)
        self.robot = dict(agent=self.agent, state_ts=10, online=True, reconciled=True,
                          state=dict(execution=dict(nav2_goals=self.nav)))
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
        fields = dict(action='sample', epoch=1, seq=seq, linear=.1, turn=-.3, deadman=True, **session)
        fields.update(overrides)
        return self.fleet.manual_bench('mock', 'operator', fields)

    def pull(self):
        return self.fleet.pull_manual_bench('mock', self.agent['boot_id'])['sample']

    def test_reverse_and_over_limit_rejected(self):
        session = self.start()
        for fields in (dict(linear=-.01), dict(linear=.11), dict(turn=.31), dict(turn=float('nan'))):
            with self.assertRaises(ApiError):
                self.sample(session, **fields)

    def test_release_fences_delayed_sample(self):
        session = self.start()
        latest = self.sample(session)
        self.fleet.manual_bench('mock', 'operator',
                                dict(action='stop', epoch=1, session_id=session['session_id']))
        self.assertIsNone(self.pull())
        with self.assertRaises(ApiError):
            self.sample({**session, 'ticket': latest['ticket']}, seq=1)

    def test_replay_and_transport_delay_rejected(self):
        session = self.start()
        latest = self.sample(session)
        with self.assertRaises(ApiError):
            self.sample(session, seq=1)
        self.clock.return_value = 20.26
        self.assertIsNone(self.pull())
        with self.assertRaises(ApiError):
            self.sample({**session, 'ticket': latest['ticket']}, seq=1)

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

    def test_real_agent_cannot_start(self):
        self.agent.update(mode='real', simulated=False)
        with self.assertRaises(ApiError):
            self.start()

    def test_mode_priority_and_deadman(self):
        nav, manual = (.05, .2), dict(linear=.1, turn=-.3, deadman=True)
        self.assertEqual(select_velocity('NAVIGATION', nav_terminal=False, allowed=True,
                                         nav=nav, manual=manual), nav)
        self.assertEqual(select_velocity('WEB_MANUAL', nav_terminal=True, allowed=True,
                                         nav=nav, manual=manual), (.1, -.3))
        for options in (dict(nav_terminal=False, allowed=True, manual=manual),
                        dict(nav_terminal=True, allowed=False, manual=manual),
                        dict(nav_terminal=True, allowed=True, manual=None),
                        dict(nav_terminal=True, allowed=True, manual={**manual, 'deadman': False})):
            self.assertEqual(select_velocity('WEB_MANUAL', nav=nav, **options), (0., 0.))


if __name__ == '__main__':
    unittest.main()
