"""Missing Nav2 status array handling; isolated callbacks only.

2026-11-20 (Claude, Phase 1): contract changed. Real rclcpp/rclpy action servers never publish status while idle, so
"no status array" from a settled unique server is not a fault; authorization still needs the empty ERROR_NONE
cancel-all answer. Original expectation (LOCKED) preserved in
local_data/raw_results/manual_phase1_claude_20261120/superseded_originals/.
"""
import time
import unittest
import rclpy
import test_motion_watchdog as existing_watchdog

class MissingStatusFence(unittest.TestCase):
 @classmethod
 def setUpClass(cls):rclpy.init()
 @classmethod
 def tearDownClass(cls):rclpy.try_shutdown()
 def setUp(self):
  self.fixture=existing_watchdog.WatchdogTests(methodName='test_startup_locked_without_heartbeat_even_with_idle_proof')
  self.fixture.setUp()
 def tearDown(self):self.fixture.tearDown()
 def test_missing_status_with_empty_cancel_reply_authorizes(self):
  w=self.fixture.w
  self.assertIsNone(w.nav_active)
  self.fixture.hb();w.tick();time.sleep(.25);self.fixture.hb();w.tick()
  self.assertEqual(w.state,existing_watchdog.AUTHORIZED)
 def test_missing_status_without_cancel_reply_remains_locked(self):
  w=self.fixture.w; w.cancel.ready=False
  self.fixture.hb();w.tick();time.sleep(.25);self.fixture.hb();w.tick()
  self.assertEqual(w.state,existing_watchdog.LOCKED)
 def test_missing_status_with_nonempty_cancel_reply_remains_locked(self):
  w=self.fixture.w; w.cancel.n_canceling=1
  self.fixture.hb();w.tick();time.sleep(.25);self.fixture.hb();w.tick()
  self.assertEqual(w.state,existing_watchdog.LOCKED)

if __name__=='__main__':unittest.main(verbosity=2)
