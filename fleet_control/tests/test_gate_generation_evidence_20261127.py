"""Safety Gate generation evidence and gated operator reset (G2, 2026-11-27): selective port of the r13 candidate into capstone_ws limo_capstone stall_detector_node.

The Gate's boot_id / fault_epoch are SELF-REPORTS (status field evidence_kind GATE_SELF_REPORT_UNAUTHENTICATED); DDS without security cannot authenticate them. What is tested:
a Gate boot is a new generation, the epoch only grows and is published only after it is persisted, a fault episode is one epoch, the evidence of an old boot is not the new one,
and the operator reset is refused unless the single identified writer has just delivered an input (a zero counts) and nothing moved. Run in a private ROS domain:
CAPSTONE_PRIVATE_ROS_TEST=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=<n> python3 -m unittest test_gate_generation_evidence_20261127"""
import json, os, sys, tempfile, time, types, unittest
from pathlib import Path

if os.environ.get('CAPSTONE_PRIVATE_ROS_TEST') != '1':
    raise unittest.SkipTest('needs a private ROS domain (CAPSTONE_PRIVATE_ROS_TEST=1, ROS_LOCALHOST_ONLY=1)')
import rclpy
from geometry_msgs.msg import Twist
GATE_DIR = Path(__file__).resolve().parents[2] / 'capstone_ws' / 'src' / 'limo_capstone'
sys.path.insert(0, str(GATE_DIR))
import limo_capstone.stall_detector_node as sdn                      # noqa: E402
from limo_capstone.stall_detector_node import StallDetectorNode      # noqa: E402

TMP = tempfile.mkdtemp(prefix='gate_evidence_')
EPOCH = os.path.join(TMP, 'epoch.json')
WD = 'fleet_motion_watchdog'


def pub(name, gid=b'\x01' * 12 + b'\x00' * 12):
    return types.SimpleNamespace(node_name=name, node_namespace='/', endpoint_gid=list(gid))


class Evidence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['prog', '--ros-args', '-p', f'epoch_state_file:={EPOCH}', '-p', 'reset_gating:=true', '-p', 'reset_live_sec:=0.4', '-p', 'reset_quiet_sec:=0.4',
                         '-p', 'input_exclusive_publishers:=[fleet_motion_watchdog]', '-p', 'enabled:=true', '-p', 'depth_mode:=disabled'])

    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def node(self):
        g = StallDetectorNode(); self.addCleanup(g.destroy_node); g.graph = [pub(WD)]
        g.get_publishers_info_by_topic = lambda topic: list(g.graph); return g

    def test_each_boot_is_a_new_generation_and_the_epoch_is_persisted_before_it_is_published(self):
        if os.path.exists(EPOCH): os.remove(EPOCH)
        a = self.node(); self.assertTrue(a.flush_epoch()); self.assertEqual((a.fault_epoch, a.published_epoch(), a.epoch_state), (1, 1, 'CREATED'))
        with open(EPOCH) as fh: self.assertEqual(json.load(fh)['epoch'], 1)
        b = self.node(); self.assertTrue(b.flush_epoch())
        self.assertNotEqual(a.boot_id, b.boot_id, 'a restarted Gate is a different generation'); self.assertEqual((b.fault_epoch, b.epoch_state), (2, 'LOADED'))
        self.assertGreater(b.published_epoch(), a.published_epoch(), 'the epoch only grows across boots')

    def test_a_fault_episode_is_one_epoch_and_unpersisted_epochs_are_not_published(self):
        if os.path.exists(EPOCH): os.remove(EPOCH)
        g = self.node(); g.flush_epoch(); e0 = g.published_epoch()
        g.state = sdn.STATE_FAILSAFE_SENSOR_TIMEOUT; g.control_loop(); g.control_loop()
        self.assertEqual(g.fault_epoch, e0 + 1, 'entering a fault state once = one epoch, staying in it adds none')
        self.assertTrue(g.flush_epoch()); self.assertEqual(g.published_epoch(), e0 + 1)
        g.state = sdn.STATE_NORMAL; g.control_loop(); g.state = sdn.STATE_STALL_LATCHED; g.control_loop(); self.assertEqual(g.fault_epoch, e0 + 2, 'a new episode is a new epoch')

    def test_a_corrupt_or_unreadable_epoch_file_publishes_no_epoch_and_is_never_overwritten(self):
        open(EPOCH, 'w').write('{not json')
        g = self.node(); self.assertIsNone(g.published_epoch(), 'no epoch -> the Agent treats the evidence as invalid (fail-closed)'); self.assertEqual(g.epoch_state, 'CORRUPT')
        g.state = sdn.STATE_FAILSAFE_SENSOR_TIMEOUT; g.control_loop(); time.sleep(0.1); self.assertEqual(open(EPOCH).read(), '{not json')

    def test_status_carries_the_evidence_with_an_honest_label(self):
        if os.path.exists(EPOCH): os.remove(EPOCH)
        g = self.node(); g.flush_epoch(); msgs = []; g.pub_diag.publish = lambda m: msgs.append(json.loads(m.data))
        g._diag_tick = 4; g._prev_state = None; g.control_loop()
        d = msgs[-1]
        for k in ('boot_id', 'fault_epoch', 'fault_epoch_pending', 'fault_latched', 'cmd_blocked', 'reset_readiness'): self.assertIn(k, d)
        self.assertEqual((d['boot_id'], d['fault_epoch'], d['fault_epoch_pending'], d['fault_latched']), (g.boot_id, 1, 0, False))
        self.assertEqual(d['evidence_kind'], 'GATE_SELF_REPORT_UNAUTHENTICATED'); self.assertNotIn('contract_version', d, 'no gate-status/1.2-sim claim: the sim-only contract is not reused')

    def test_the_operator_reset_needs_the_identified_writer_a_fresh_zero_and_quiet(self):
        if os.path.exists(EPOCH): os.remove(EPOCH)
        g = self.node(); g.state = sdn.STATE_FAILSAFE_SENSOR_TIMEOUT
        self.assertIn('INPUT_SILENT', g.reset_readiness()['blockers'], 'no input at all: the zero the reset needs is missing')
        g.cb_cmd_raw(Twist()); self.assertTrue(g.reset_readiness()['ready'], g.reset_readiness())
        m = Twist(); m.linear.x = 0.1; g.cb_cmd_raw(m); self.assertIn('MOTION_COMMANDED', g.reset_readiness()['blockers'])
        time.sleep(0.5); g.cb_cmd_raw(Twist()); self.assertTrue(g.reset_readiness()['ready'])
        g.graph = [pub(WD), pub('rogue', b'\x02' * 12 + b'\x00' * 12)]; self.assertTrue(any(x.startswith('INPUT_WRITERS') for x in g.reset_readiness()['blockers']), 'a second writer')
        g.graph = [pub('rogue')]; self.assertIn('INPUT_WRITER_NOT_ALLOWED', g.reset_readiness()['blockers'])
        g.graph = []; self.assertIn('INPUT_NO_WRITER', g.reset_readiness()['blockers'])
        g.graph = [pub(WD)]; g.state = sdn.STATE_NORMAL; self.assertIn('NOTHING_TO_RESET', g.reset_readiness()['blockers'])

    def test_a_refused_reset_changes_nothing_and_an_accepted_one_forgets_the_earlier_command(self):
        from std_srvs.srv import Trigger
        if os.path.exists(EPOCH): os.remove(EPOCH)
        g = self.node(); g.state = sdn.STATE_FAILSAFE_SENSOR_TIMEOUT; r = g.cb_reset_latch(Trigger.Request(), Trigger.Response())
        self.assertFalse(r.success); self.assertIn('INPUT_SILENT', r.message); self.assertEqual(g.state, sdn.STATE_FAILSAFE_SENSOR_TIMEOUT)
        m = Twist(); m.linear.x = 0.0; g.cb_cmd_raw(m); g.latest_cmd.linear.x = 0.1; r = g.cb_reset_latch(Trigger.Request(), Trigger.Response())
        self.assertTrue(r.success, r.message); self.assertEqual(g.state, sdn.STATE_INITIALIZING); self.assertEqual(g.latest_cmd.linear.x, 0.0, 'the pre-reset command is not used after the reset')


class DefaultsUnchanged(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init(args=['prog', '--ros-args', '-p', 'enabled:=true', '-p', 'depth_mode:=disabled'])

    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def test_without_the_new_parameters_the_reset_is_not_gated_and_the_epoch_is_in_memory(self):
        from std_srvs.srv import Trigger
        g = StallDetectorNode(); self.addCleanup(g.destroy_node)
        self.assertEqual(g.reset_readiness(), {'enforced': False, 'ready': None, 'blockers': []}); self.assertEqual(g.published_epoch(), 0)
        g.state = sdn.STATE_FAILSAFE_SENSOR_TIMEOUT; self.assertTrue(g.cb_reset_latch(Trigger.Request(), Trigger.Response()).success)


if __name__ == '__main__':
    unittest.main()
