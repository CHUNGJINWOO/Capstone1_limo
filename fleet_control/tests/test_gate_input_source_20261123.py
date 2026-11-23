"""Safety Gate /cmd_vel_raw input-source validation (G2, 2026-11-23): decision logic of capstone_ws limo_capstone stall_detector_node.

Runs the real StallDetectorNode with a mocked DDS graph view (get_publishers_info_by_topic) and a mocked writer status, so every
fault the graph can show is injected deterministically. Needs ROS 2 Humble (rclpy); run inside a private network namespace:
CAPSTONE_PRIVATE_ROS_TEST=1 ROS_DOMAIN_ID=<n> ... python3 -m unittest test_gate_input_source_20261123

Rule under test: the Gate uses a /cmd_vel_raw message only while exactly ONE endpoint exists, it carries an allowed node name, it is
bound to the GID the writer itself reports on its status topic, and nothing changed since. Everything else is zero output, a fault
and (for a second writer, a foreign name, a GID/boot change) a sticky state that only the operator's Gate reset clears.
Limit (stated in the Gate status as GRAPH_ONLY_UNAUTHENTICATED): DDS without security cannot authenticate a writer; a process that
publishes on /cmd_vel_raw ALONE under an allowed node name before the real writer exists, and also forges the status message, is accepted.
"""
import json
import math
import os
import sys
import time
import types
import unittest
from pathlib import Path

if os.environ.get('CAPSTONE_PRIVATE_ROS_TEST') != '1':
    raise unittest.SkipTest('needs a private ROS domain (CAPSTONE_PRIVATE_ROS_TEST=1, ROS_LOCALHOST_ONLY=1)')
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

GATE_DIR = Path(__file__).resolve().parents[2] / 'capstone_ws' / 'src' / 'limo_capstone'
sys.path.insert(0, str(GATE_DIR))
from limo_capstone.stall_detector_node import StallDetectorNode  # noqa: E402

WD = 'fleet_motion_watchdog'
GID_A, GID_B, GID_C = [1] * 16 + [0] * 8, [2] * 16 + [0] * 8, [3] * 16 + [0] * 8


def hexgid(g):
    return bytes(g).hex()


def pub(name, gid, ns='/'):
    return types.SimpleNamespace(node_name=name, node_namespace=ns, endpoint_gid=list(gid))


class GateInput(unittest.TestCase):
    ARGS = ['prog', '--ros-args', '-p', 'input_exclusive_publishers:=[fleet_motion_watchdog]', '-p', 'input_settle_sec:=0.05',
            '-p', 'input_writer_status_topic:=/fleet/motion_watchdog/status', '-p', 'input_max_abs_linear_x:=0.2',
            '-p', 'input_max_abs_angular_z:=0.6', '-p', 'enabled:=true', '-p', 'depth_mode:=disabled']

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=cls.ARGS)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.g = StallDetectorNode()
        self.addCleanup(self.g.destroy_node)
        self.graph = [pub(WD, GID_A)]
        self.g.get_publishers_info_by_topic = lambda topic: self._lookup(topic)
        self.fail_lookup = False

    def _lookup(self, topic):
        if self.fail_lookup:
            raise RuntimeError('graph lookup failed')
        return list(self.graph)

    def status(self, gid=GID_A, boot='boot-1', age=0.0):
        """What the Watchdog's status message says about itself (age shifts the receive time to simulate delay)."""
        self.g.wr = (boot, hexgid(gid), time.monotonic() - age)

    def bind(self, gid=GID_A, boot='boot-1'):
        self.graph = [pub(WD, gid)]
        self.status(gid, boot)
        self.g.input_graph()                          # candidate
        time.sleep(0.08)
        self.status(gid, boot)
        ok = self.g.input_graph()
        time.sleep(0.08)
        return ok or self.g.input_graph()

    def step(self):
        self.status_refresh()
        return self.g.input_graph()

    def status_refresh(self):
        if self.g.wr is not None:
            self.g.wr = (self.g.wr[0], self.g.wr[1], time.monotonic())

    # 1 -------------------------------------------------------------------------------------------------------------------
    def test_only_the_confirmed_watchdog_is_accepted(self):
        self.assertTrue(self.bind())
        self.assertIsNone(self.g.in_reason)
        self.assertEqual(self.g.in_bound, hexgid(GID_A))
        self.assertEqual(self.g.input_status()['validation'], 'GRAPH_ONLY_UNAUTHENTICATED')

    def test_writer_without_status_confirmation_is_not_bound(self):
        self.g.wr = None                                          # status message missing
        for _ in range(4):
            self.assertFalse(self.g.input_graph()); time.sleep(0.05)
        self.assertIsNone(self.g.in_bound)

    def test_delayed_status_does_not_bind(self):
        self.status(age=3.0)                                      # older than 1 s
        for _ in range(4):
            self.assertFalse(self.g.input_graph()); time.sleep(0.05)
        self.assertIsNone(self.g.in_bound)

    def test_status_with_another_gid_does_not_bind(self):
        self.status(gid=GID_B)                                    # the writer says its GID is B, the graph shows A
        for _ in range(4):
            self.assertFalse(self.step() if False else self.g.input_graph()); time.sleep(0.05)
        self.assertIsNone(self.g.in_bound)

    # 2 -------------------------------------------------------------------------------------------------------------------
    def test_watchdog_plus_external_writer_is_sticky_not_exclusive(self):
        self.assertTrue(self.bind())
        self.graph = [pub(WD, GID_A), pub('rogue_cmd_pub', GID_B)]
        self.assertFalse(self.step())
        self.assertEqual(self.g.in_sticky, 'NOT_EXCLUSIVE')
        self.graph = [pub(WD, GID_A)]                             # the rogue leaves: still refused until the operator resets
        time.sleep(0.1)
        self.assertFalse(self.step())
        self.assertEqual(self.g.in_reason, 'NOT_EXCLUSIVE')
        self.assertIn('cmd_input:NOT_EXCLUSIVE', self.g.input_faults(0))

    def test_rogue_with_the_allowed_name_is_still_a_second_writer(self):
        self.assertTrue(self.bind())
        self.graph = [pub(WD, GID_A), pub(WD, GID_B)]             # imitates the name: counted as a second endpoint
        self.assertFalse(self.step())
        self.assertEqual(self.g.in_sticky, 'NOT_EXCLUSIVE')

    # 3 -------------------------------------------------------------------------------------------------------------------
    def test_external_writer_alone_is_never_accepted(self):
        self.graph = [pub('rogue_cmd_pub', GID_B)]
        for _ in range(4):
            self.assertFalse(self.step()); time.sleep(0.05)
        self.assertEqual(self.g.in_sticky, 'UNEXPECTED_PUBLISHER')
        self.assertIsNone(self.g.in_bound)

    def test_no_publisher_is_zero_output_without_a_sticky_fault(self):
        self.graph = []
        self.assertFalse(self.step())
        self.assertEqual(self.g.in_reason, 'NO_PUBLISHER')
        self.assertIsNone(self.g.in_sticky)

    # 4 -------------------------------------------------------------------------------------------------------------------
    def test_watchdog_exit_then_restart_is_a_new_writer_until_reset(self):
        self.assertTrue(self.bind())
        self.graph = []
        self.assertFalse(self.step())
        self.assertEqual(self.g.in_reason, 'PUBLISHER_LOST')
        self.graph = [pub(WD, GID_C)]                             # restarted Watchdog: new GID
        self.status(GID_C, 'boot-2')
        time.sleep(0.08)
        self.assertFalse(self.step())
        self.assertEqual(self.g.in_sticky, 'PUBLISHER_CHANGED')

    def test_dead_endpoint_ghost_plus_new_writer_is_not_exclusive(self):
        self.assertTrue(self.bind())
        self.graph = [pub(WD, GID_A), pub(WD, GID_C)]             # SIGKILLed original lingers ~20 s while the new one runs
        self.assertFalse(self.step())
        self.assertEqual(self.g.in_sticky, 'NOT_EXCLUSIVE')

    def test_writer_boot_change_with_same_gid_is_a_restart(self):
        self.assertTrue(self.bind())
        self.status(GID_A, 'boot-2')                              # same endpoint reported by a different boot
        self.assertFalse(self.g.input_graph())
        self.assertEqual(self.g.in_sticky, 'WRITER_RESTARTED')

    def test_writer_reports_a_different_gid_after_binding(self):
        self.assertTrue(self.bind())
        self.status(GID_B)
        self.assertFalse(self.g.input_graph())
        self.assertEqual(self.g.in_sticky, 'PUBLISHER_CHANGED')

    def test_publisher_swap_at_the_same_name(self):
        self.assertTrue(self.bind())
        self.graph = [pub(WD, GID_B)]
        self.assertFalse(self.step())
        self.assertEqual(self.g.in_sticky, 'PUBLISHER_CHANGED')

    # 5 -------------------------------------------------------------------------------------------------------------------
    def test_missing_endpoint_gid_is_unverifiable_and_fails_closed(self):
        self.graph = [pub(WD, [0] * 24)]                          # all-zero GID: DDS gave no endpoint identity
        for _ in range(3):
            self.assertFalse(self.step())
        self.assertEqual(self.g.in_reason, 'GID_UNKNOWN')
        self.assertIsNone(self.g.in_bound)
        self.assertIn('cmd_input:GID_UNKNOWN', self.g.input_faults(0))

    def test_unreadable_gid_attribute_fails_closed(self):
        self.graph = [types.SimpleNamespace(node_name=WD, node_namespace='/')]      # no endpoint_gid attribute
        self.assertFalse(self.step())
        self.assertEqual(self.g.in_reason, 'GID_UNKNOWN')

    def test_graph_lookup_failure_fails_closed_and_recovers(self):
        self.assertTrue(self.bind())
        self.fail_lookup = True
        self.assertFalse(self.step())
        self.assertTrue(self.g.in_reason.startswith('LOOKUP_ERROR'))
        self.fail_lookup = False
        time.sleep(0.1)
        self.assertFalse(self.step())                            # settle window after a gap before messages are used again
        time.sleep(0.1)
        self.assertTrue(self.step())

    def test_unresolved_node_name_is_not_bound(self):
        self.graph = [pub('_NODE_NAME_UNKNOWN_', GID_A)]
        self.status()
        self.assertFalse(self.step())
        self.assertEqual(self.g.in_reason, 'NODE_UNRESOLVED')

    # 6 -------------------------------------------------------------------------------------------------------------------
    def test_non_finite_and_out_of_range_commands_are_rejected(self):
        for kw in (dict(linear_x=float('nan')), dict(linear_x=float('inf')), dict(linear_x=5.0), dict(angular_z=9.0), dict(angular_z=float('nan'))):
            m = Twist(); m.linear.x = kw.get('linear_x', 0.0); m.angular.z = kw.get('angular_z', 0.0)
            self.assertIsNotNone(self.g.input_valid(m), kw)
        m = Twist(); m.linear.x = 0.15; m.angular.z = -0.5
        self.assertIsNone(self.g.input_valid(m))

    def test_rejected_message_never_reaches_the_command_and_marks_motion(self):
        self.graph = [pub(WD, GID_A), pub('rogue_cmd_pub', GID_B)]
        m = Twist(); m.linear.x = 0.18
        self.g.cb_cmd_raw(m)
        self.assertEqual(self.g.latest_cmd.linear.x, 0.0)
        self.assertIsNotNone(self.g.in_drop_motion_t)             # blocks arming on the strength of the replacing zero
        self.assertEqual(self.g.in_n['dropped_graph'], 1)

    def test_accepted_message_is_used(self):
        self.assertTrue(self.bind())
        m = Twist(); m.linear.x = 0.08
        self.g.cb_cmd_raw(m)
        self.assertAlmostEqual(self.g.latest_cmd.linear.x, 0.08)
        self.assertEqual(self.g.in_n['accepted'], 1)

    # 7 -------------------------------------------------------------------------------------------------------------------
    def test_reset_clears_the_sticky_state_and_the_writer_must_rebind(self):
        self.assertTrue(self.bind())
        self.graph = [pub(WD, GID_A), pub('rogue_cmd_pub', GID_B)]
        self.step()
        self.assertEqual(self.g.in_sticky, 'NOT_EXCLUSIVE')
        self.graph = [pub(WD, GID_A)]
        self.g.input_on_reset()
        self.assertIsNone(self.g.in_sticky); self.assertIsNone(self.g.in_bound)
        self.assertFalse(self.step())                             # needs the settle time again
        self.assertTrue(self.bind())

    def test_reset_does_not_hide_a_persisting_second_writer(self):
        self.assertTrue(self.bind())
        self.graph = [pub(WD, GID_A), pub('rogue_cmd_pub', GID_B)]
        self.step()
        self.g.input_on_reset()                                   # operator resets while the rogue is still there
        self.step()
        self.assertEqual(self.g.in_sticky, 'NOT_EXCLUSIVE')

    # 7b ------------------------------------------------------------------------------------------------------------------
    def test_output_topic_exclusivity_detects_a_foreign_driver_writer(self):
        self.g.excl_topic = '/cmd_vel_locked'
        own = pub(self.g.get_name(), GID_A, self.g.get_namespace())
        self.graph = [own]
        self.g.excl_poll(); self.assertEqual(self.g.excl_faults, [])
        self.graph = [own, pub('teleop_twist_keyboard', GID_B)]
        self.g.excl_poll(); self.assertTrue(self.g.excl_faults[0].startswith('cmd_vel:FOREIGN_PUBLISHER:'))
        self.assertIn('teleop_twist_keyboard', self.g.excl_faults[0])
        self.assertTrue(self.g.input_faults(0))                                   # reaches the fail-safe decision even with validation off? (see below)
        self.graph = [own, pub(self.g.get_name(), GID_C, self.g.get_namespace())]
        self.g.excl_poll(); self.assertEqual(self.g.excl_faults, ['cmd_vel:DUPLICATE_GATE_ENDPOINT'])
        self.fail_lookup = True
        self.g.excl_poll(); self.assertTrue(self.g.excl_faults[0].startswith('cmd_vel:LOOKUP_ERROR'))
        self.fail_lookup = False
        self.graph = [own]
        self.g.excl_poll(); self.assertEqual(self.g.excl_faults, [])               # not sticky: the Gate itself recovers once the writer is gone

    # 8 -------------------------------------------------------------------------------------------------------------------
    def test_default_off_keeps_the_existing_gate_behaviour(self):
        rclpy.shutdown()
        rclpy.init(args=['prog', '--ros-args', '-p', 'enabled:=true', '-p', 'depth_mode:=disabled'])
        try:
            g = StallDetectorNode()
            self.addCleanup(g.destroy_node)
            m = Twist(); m.linear.x = 0.18
            g.cb_cmd_raw(m)
            self.assertAlmostEqual(g.latest_cmd.linear.x, 0.18)    # no validation configured: unchanged
            self.assertEqual(g.input_status()['validation'], 'DISABLED')
            self.assertEqual(g.input_faults(0), [])
        finally:
            rclpy.shutdown()
            rclpy.init(args=self.ARGS)


if __name__ == '__main__':
    unittest.main()
