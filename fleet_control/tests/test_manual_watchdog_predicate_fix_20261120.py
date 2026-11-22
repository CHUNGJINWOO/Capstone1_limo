"""Actual Watchdog predicate checks without starting ROS or touching the graph.

2026-11-20 (Claude, Phase 1): expectations changed with the continuity rule. Action servers publish status only on
change and an idle server never publishes, so message presence/age is no longer required; a settled unique server
plus an empty ERROR_NONE cancel-all sent after the settle is idle evidence. Previous expectations are preserved in
local_data/raw_results/manual_phase1_claude_20261120/superseded_originals/.
"""
import unittest
from types import SimpleNamespace
from limo_robot_agent.motion_watchdog import MotionWatchdog

class Predicate(unittest.TestCase):
 def setUp(self):
  self.w=SimpleNamespace(idle_proof_t=10.,proof_max_age=2.,status_pubs=1,status_pub_since=9.,status_settle=.5,nav_active=None,nav_status_t=None,status_timeout=1.,status_identity_ok=True,status_gid='ab'*16,input_ok=True)
 def test_silent_settled_server_and_active_goal(self):
  f=lambda:MotionWatchdog.idle_proven(self.w,10.5)
  self.assertTrue(f())                                   # never published: no goal was ever accepted
  self.w.nav_active=0;self.w.nav_status_t=10.2;self.assertTrue(f())
  self.w.nav_status_t=8.;self.assertTrue(f())            # age is irrelevant (published on change only)
  self.w.nav_active=1;self.assertFalse(f())              # a non-terminal goal is never idle
  self.w.nav_active=None;self.w.nav_status_t=10.2;self.assertFalse(f())   # inconsistent bookkeeping fails closed
  self.w.nav_active=0;self.w.idle_proof_t=9.2;self.assertFalse(f())       # cancel-all sent before the settle
  self.w.idle_proof_t=10.;self.w.status_identity_ok=False;self.assertFalse(f())  # unreadable publisher identity
  self.w.status_identity_ok=True;self.w.status_pubs=2;self.assertFalse(f())
 def test_supervisor_blocks_settling_and_duplicate(self):
  self.w.hb_unhealthy=False;self.w.hb_t=10.;self.w.hb_timeout=1.;self.w.loop_age=.01;self.w.loop_timeout=.5
  self.w.gate_t=10.;self.w.gate_timeout=1.
  self.assertIsNone(MotionWatchdog.supervisor_fault(self.w,10.2))
  self.w.status_pub_since=10.;self.assertEqual(MotionWatchdog.supervisor_fault(self.w,10.2),'NAV2_STATUS_SETTLING')
  self.w.status_pub_since=None;self.assertEqual(MotionWatchdog.supervisor_fault(self.w,10.2),'NAV2_STATUS_SETTLING')
  self.w.status_pubs=2;self.assertEqual(MotionWatchdog.supervisor_fault(self.w,10.2),'NAV2_ACTION_SERVER_NOT_UNIQUE')
if __name__=='__main__':unittest.main(verbosity=2)
