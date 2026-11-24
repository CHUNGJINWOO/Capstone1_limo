"""G2 Phase A: the Agent motor-path report must treat a writer on the driver input like a /cmd_vel writer (stub ROS, no graph)."""
import sys, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]


class AgentMotorPath(unittest.TestCase):
    """The Agent's motor-path report must treat a writer on the driver input like a /cmd_vel writer (G2 Phase A)."""
    def setUp(self):
        sys.path[:0] = [str(ROOT / 'fleet_control/server'), str(ROOT / 'fleet_control/agent/limo_robot_agent'), str(ROOT / 'fleet_control/tests')]
        import test_agent_safety_v1 as base
        self.r = base._make_ready_robot()

    def test_locked_when_driver_topic_is_silent(self):
        self.assertEqual(self.r._motor_path_status()[0], 'LOCKED')

    def test_armed_when_anything_writes_the_driver_input(self):
        self.r.topic_publishers['/limo_driver/cmd_vel'] = ['rogue']
        st, info = self.r._motor_path_status()
        self.assertEqual(st, 'ARMED'); self.assertIn('rogue', info['cmd_vel_publishers'])

    def test_unknown_when_the_driver_topic_lookup_fails(self):
        del self.r.topic_publishers['/limo_driver/cmd_vel']          # the stub raises like a failed graph query
        self.assertEqual(self.r._motor_path_status()[0], 'UNKNOWN')


class ForeignWriterLatch(AgentMotorPath):
    """A writer on the driver input is detected, reported as ARMED and the status does not return to LOCKED by itself (detection only)."""
    def test_foreign_writer_is_armed_then_latched_after_it_disappears(self):
        self.r.topic_publishers['/limo_driver/cmd_vel'] = ['rogue']
        st, info = self.r._motor_path_status(); self.assertEqual(st, 'ARMED'); self.assertTrue(info['foreign_writer_latched']); self.assertTrue(info['detection_only'])
        self.r.topic_publishers['/limo_driver/cmd_vel'] = []
        st, info = self.r._motor_path_status()
        self.assertEqual(st, 'ARMED', 'the writer is gone but a direct command may already have been executed: not LOCKED'); self.assertEqual(info['foreign_writers_seen'], ['rogue'])
        self.assertTrue(self.r.clear_motor_path_latch()); self.assertEqual(self.r._motor_path_status()[0], 'LOCKED')

    def test_cmd_vel_writer_is_latched_too(self):
        self.r.topic_publishers['/cmd_vel'] = ['vendor_demo']; self.assertEqual(self.r._motor_path_status()[0], 'ARMED')
        self.r.topic_publishers['/cmd_vel'] = []; self.assertEqual(self.r._motor_path_status()[0], 'ARMED')

    def test_latched_status_blocks_movement_preconditions(self):
        self.r.topic_publishers['/limo_driver/cmd_vel'] = ['rogue']; self.r._motor_path_status(); self.r.topic_publishers['/limo_driver/cmd_vel'] = []
        mp, _ = self.r._motor_path_status(); self.assertNotIn(mp, ('LOCKED', 'SIM_GATED'))       # the check in _verify_nav_preconditions


class VelocityTelemetry(AgentMotorPath):
    """Observed-velocity block: source, age and validity (G2 Phase A review item 1)."""
    def test_unavailable_before_any_odom(self):
        self.r.odom_twist = None
        self.assertEqual(self.r._velocity_telemetry()['status'], 'UNAVAILABLE')

    def test_fresh_stale_and_nonfinite(self):
        import time
        self.r.odom_twist = (0.08, -0.1); self.r.t['odom'] = time.monotonic()
        v = self.r._velocity_telemetry(); self.assertEqual((v['status'], v['linear_x_mps'], v['angular_z_rps']), ('OK', 0.08, -0.1))
        self.assertIn('not independent', v['source'])
        self.r.t['odom'] = time.monotonic() - 5; self.assertEqual(self.r._velocity_telemetry()['status'], 'STALE')
        self.r.t['odom'] = time.monotonic(); self.r.odom_twist = (float('nan'), 0.0)
        self.assertEqual(self.r._velocity_telemetry()['status'], 'UNAVAILABLE')


if __name__ == '__main__':
    unittest.main()
