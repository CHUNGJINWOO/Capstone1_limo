"""Watchdog must not infer an idle publisher when its GID lookup fails."""
import time, unittest
import rclpy
import test_motion_watchdog as fixture

class MissingGid(unittest.TestCase):
 @classmethod
 def setUpClass(cls):rclpy.init()
 @classmethod
 def tearDownClass(cls):rclpy.try_shutdown()
 def test_nonfinite_control_loop_age_never_authorizes(self):
  t=fixture.WatchdogTests('test_startup_locked_without_heartbeat_even_with_idle_proof');t.setUp()
  try:
   t.hb(); t.w.input_ok=True;t.w.status_pubs=1;t.w.status_pub_since=time.monotonic()-1
   t.w.loop_age=float('nan')
   reason=t.w.supervisor_fault(time.monotonic())
   print('OBSERVED control loop age NaN:',reason,flush=True)
   self.assertIsNotNone(reason,'NaN loop age must fail closed')
  finally:t.tearDown()
 def test_gid_lookup_unavailable_never_authorizes(self):
  t=fixture.WatchdogTests('test_startup_locked_without_heartbeat_even_with_idle_proof');t.setUp()
  try:
   t.w._status_gids=lambda:()  # graph count=1 but endpoint identity unavailable
   t.hb();t.w.tick();time.sleep(.25);t.hb();t.w.tick()
   print('OBSERVED count=1, GID lookup unavailable:',t.w.state,'status_gid=',t.w.status_gid,flush=True)
   self.assertEqual(t.w.state,fixture.LOCKED,'publisher identity unavailable must fail closed')
  finally:t.tearDown()
if __name__=='__main__':unittest.main(verbosity=2)
