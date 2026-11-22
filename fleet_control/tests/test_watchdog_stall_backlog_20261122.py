"""UNIT (2026-11-22): after the Motion Watchdog itself was frozen, inputs queued during the freeze are not forwarded.

Real MotionWatchdog node, callbacks driven directly (same fakes as test_motion_watchdog.py). The executor may dispatch the queued
/fleet/nav_cmd_vel messages before the tick that detects WATCHDOG_SELF_STALL; before the 2026-11-22 change they were forwarded.
"""
import time
import unittest

from test_motion_watchdog import WatchdogTests, Twist, AUTHORIZED, FAILSAFE, LOCKED


class StallBacklog(WatchdogTests):
    def test_queued_input_after_self_stall_not_forwarded(self):
        self.authorize()
        self.w.self_stall = 0.25
        self.hb(); self.w.tick()
        t = Twist(); t.linear.x = 0.1
        self.out.clear(); self.w.on_cmd(t)
        self.assertEqual(self.out, [(0.1, 0.0)])            # normal forwarding while ticks are regular
        time.sleep(0.4)                                      # the process was frozen: no tick for 0.4 s
        self.out.clear()
        for _ in range(5): self.w.on_cmd(t)                  # backlog dispatched before the next tick
        self.assertTrue(all(o == (0.0, 0.0) for o in self.out), self.out)
        self.hb(); self.w.tick()
        self.assertIn(self.w.state, (FAILSAFE, LOCKED))
        self.assertEqual(self.w.last_fault, 'WATCHDOG_SELF_STALL')


del WatchdogTests
if __name__ == '__main__':
    unittest.main(verbosity=2)
