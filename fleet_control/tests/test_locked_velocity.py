"""Exercise the v5 locked selector and its distinct manual and Nav2 policies."""
import unittest
from test_arbiter_generation_20261122 import Agent, Bench, Robot, va
from unittest.mock import patch


class LockedVelocity(unittest.TestCase):
    def test_lock_deadman_and_terminal(self):
        robot, agent, bench = Robot(), Agent(), Bench()
        arb = va.VelocityArbiter(robot, agent, bench)
        with patch.object(va.time, 'monotonic', return_value=20.) as clock:
            with patch.object(arb, 'graph_locked', return_value=True) as graph:
                agent.mode = 'MANUAL'
                arb.update()
                bench.s = dict(requested_linear=.1, requested_turn=.2, deadman=True, expires=21.)
                arb.update()
                self.assertEqual(arb.output, (.1, .2))
                robot.active = {'goal_uuid': 'g', 'owner': 'fleet'}
                arb.update()
                self.assertEqual(arb.output, (0., 0.))
                robot.active = None
                bench.s['deadman'] = False
                arb.update()
                self.assertEqual(arb.output, (0., 0.))
                agent.mode = 'NAVIGATION'
                robot.active = {'goal_uuid': 'nav', 'owner': 'fleet'}
                graph.return_value = False
                arb.update()
                self.assertEqual(arb.output, (0., 0.))
                graph.return_value = True
                arb.update()
                self.assertEqual(arb.output, (0., 0.))
                clock.return_value = 20.5
                arb.nav_raw_t = 20.5
                arb.nav_sample = (20.5, 1., 1.)
                arb.update()
                self.assertEqual(arb.output, (.15, .5))
                robot.wd = False
                arb.update()
                self.assertEqual(arb.output, (0., 0.))
                robot.wd = True
                robot.gate['cmd_blocked'] = True
                arb.update()
                self.assertEqual(arb.output, (0., 0.))
