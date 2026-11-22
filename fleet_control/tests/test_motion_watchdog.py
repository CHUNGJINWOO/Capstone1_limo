"""State-machine checks for the Agent-independent motion watchdog (real rclpy node, callbacks driven directly).

Requires ROS 2 Humble (rclpy). No graph traffic is needed: the cancel service client is replaced by a fake.
"""
import json
import os
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agent' / 'limo_robot_agent'))
try:
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from action_msgs.srv import CancelGoal
    from limo_robot_agent.motion_watchdog import MotionWatchdog, AUTHORIZED, LOCKED, FAILSAFE
except Exception as exc:  # pragma: no cover
    raise unittest.SkipTest(f'ROS 2 python packages unavailable: {exc}')


class FakeFuture:
    def __init__(self, n_canceling, rc=0, hang=False):
        self.n, self.rc, self.hang = n_canceling, rc, hang

    def add_done_callback(self, cb):
        if not self.hang:
            cb(self)

    def result(self):
        return SimpleNamespace(goals_canceling=[object()] * self.n, return_code=self.rc)


class FakeCancel:
    def __init__(self):
        self.ready, self.n_canceling, self.calls, self.rc, self.hang = True, 0, 0, 0, False

    def service_is_ready(self):
        return self.ready

    def call_async(self, req):
        self.calls += 1
        return FakeFuture(self.n_canceling, self.rc, self.hang)


class WatchdogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.w = MotionWatchdog()
        self.w.cancel = FakeCancel()
        self.out = []
        self.w.out = SimpleNamespace(publish=lambda m: self.out.append((m.linear.x, m.angular.z)))
        self.w.rearm_hold = 0.2
        self.w.probe_period = 0.0
        self.w.status_settle = 0.0
        self.w.self_stall = 10.0              # unit tests sleep between ticks
        self.npubs = 1                            # one NavigateToPose status publisher (one action server)
        self.w.count_publishers = lambda topic: self.npubs
        # Real graph metadata: one endpoint per counted publisher, each with a readable GID (2026-11-20 identity rule).
        self.w.get_publishers_info_by_topic = (lambda topic: [SimpleNamespace(endpoint_gid=bytes([7 + k]) * 16, node_name='bt_navigator')
                                                              for k in range(self.npubs)] if topic.endswith('/status') else [])
        self.w.on_gate(String(data='{}'))        # Gate alive unless a test says otherwise

    def tearDown(self):
        self.w.destroy_node()

    def hb(self, boot='b1', loop=0.01, healthy=True, gate=True, goals=()):
        if gate:
            self.w.on_gate(String(data='{}'))   # a running Gate keeps publishing its status
        self.w.on_hb(String(data=json.dumps({'boot_id': boot, 'healthy': healthy, 'tasks_age_s': loop,
                                             'goal_uuids': list(goals)})))

    def status(self, *codes, uid=None):
        msg = GoalStatusArray()
        for i, c in enumerate(codes):
            s = GoalStatus(); s.status = c
            s.goal_info.goal_id.uuid = list((uid or bytes([i + 1]) * 16))
            msg.status_list.append(s)
        self.w.on_status(msg)

    def authorize(self):
        self.w.self_stall = 10.0              # tests sleep between ticks; the self-stall test sets it back
        self.status()  # A received empty array is required before authorization.
        self.hb(); self.w.tick(); time.sleep(0.25); self.hb(); self.w.tick()
        self.assertEqual(self.w.state, AUTHORIZED)

    def test_startup_locked_without_heartbeat_even_with_idle_proof(self):
        for _ in range(3):
            self.w.tick()
        self.assertEqual(self.w.state, LOCKED)
        self.assertTrue(all(o == (0.0, 0.0) for o in self.out))

    def test_unreceived_status_is_not_idle_proof(self):
        # 2026-11-20 (Claude): a silent server is no longer a fault by itself (status is published on change only);
        # without a cancel-all answer there is still no idle proof, so the watchdog stays LOCKED.
        self.w.cancel.ready = False          # no action server answer -> no proof
        self.hb(); self.w.tick(); time.sleep(0.25); self.hb(); self.w.tick()
        self.assertEqual(self.w.state, LOCKED)
        self.assertEqual(self.w.reason, 'AWAITING_IDLE_PROOF')

    def test_silent_idle_server_authorizes_with_cancel_all(self):
        # Real Nav2/rclcpp: an idle server that never had a goal publishes no status at all.
        self.hb(); self.w.tick(); time.sleep(0.25); self.hb(); self.w.tick()
        self.assertEqual(self.w.state, AUTHORIZED)
        self.assertIsNone(self.w.nav_status_t)

    def test_foreign_or_second_input_publisher_trips_and_drops(self):
        self.authorize()
        rogue = [SimpleNamespace(node_name='fleet_robot_agent'), SimpleNamespace(node_name='rogue_teleop')]
        status_info = [SimpleNamespace(endpoint_gid=b'\x07' * 16, node_name='bt_navigator')]
        self.w.get_publishers_info_by_topic = lambda topic: rogue if topic == self.w.input_topic else status_info
        self.hb(); self.w.tick()
        self.assertNotEqual(self.w.state, AUTHORIZED)            # FAILSAFE, then LOCKED in the same tick with the sync fake cancel
        self.assertEqual(self.w.last_fault, 'VELOCITY_INPUT_NOT_EXCLUSIVE')
        self.out.clear(); t = Twist(); t.linear.x = 0.1; self.w.on_cmd(t)
        self.assertEqual(self.out, [])
        self.w.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(node_name='fleet_robot_agent')] if topic == self.w.input_topic else status_info
        for _ in range(4):
            time.sleep(0.12); self.hb(); self.w.tick()
        self.assertEqual(self.w.state, AUTHORIZED)

    def test_unreadable_gid_never_authorizes_and_revokes(self):
        self.w.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(endpoint_gid=b'', node_name='bt_navigator')] if topic.endswith('/status') else []
        self.hb(); self.w.tick(); time.sleep(0.25); self.hb(); self.w.tick()
        self.assertEqual((self.w.state, self.w.reason), (LOCKED, 'NAV2_SERVER_IDENTITY_UNKNOWN'))
        self.setUp(); self.authorize()
        self.w.get_publishers_info_by_topic = lambda topic: (_ for _ in ()).throw(RuntimeError('graph')) if topic.endswith('/status') else []
        self.hb(); self.w.tick()
        self.assertEqual(self.w.last_fault, 'NAV2_SERVER_IDENTITY_UNKNOWN')
        self.assertNotEqual(self.w.state, AUTHORIZED)

    def test_partial_identity_with_two_publishers(self):
        self.npubs = 2
        self.w.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(endpoint_gid=b'\x01' * 16, node_name='bt_navigator')] if topic.endswith('/status') else []
        self.hb(); self.w.tick(); time.sleep(0.25); self.hb(); self.w.tick()
        self.assertEqual(self.w.state, LOCKED)

    def test_nonfinite_or_negative_loop_age_faults(self):
        for bad in (float('nan'), float('inf'), -0.1, True, None, '0.1'):
            with self.subTest(bad=bad):
                self.w.on_gate(String(data='{}')); self.w.on_hb(String(data=json.dumps({'boot_id': 'b', 'healthy': True, 'goal_uuids': []})))
                self.w.loop_age = bad
                self.w._update_graph(time.monotonic())
                self.assertEqual(self.w.supervisor_fault(time.monotonic()), 'AGENT_CONTROL_LOOP_STALLED')

    def test_settling_blocks_until_publisher_stable(self):
        self.w.status_settle = 5.0
        self.hb(); self.w.tick(); time.sleep(0.25); self.hb(); self.w.tick()
        self.assertEqual(self.w.state, LOCKED)
        self.assertEqual(self.w.reason, 'NAV2_STATUS_SETTLING')

    def test_status_publisher_gid_change_trips_and_resubscribes(self):
        gid = [b'\x01' * 16]
        self.w.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(endpoint_gid=gid[0], node_name='bt_navigator')] if topic.endswith('/status') else []
        self.authorize()
        self.w.status_settle = 0.3            # the new server must settle before a cancel-all counts
        before = self.w.counters_resub
        gid[0] = b'\x02' * 16               # Nav2 restarted between two graph polls
        self.hb(); self.w.tick()
        self.assertEqual(self.w.state, FAILSAFE)
        self.assertEqual(self.w.last_fault, 'NAV2_ACTION_SERVER_CHANGED')
        self.assertEqual(self.w.counters_resub, before + 1)
        self.assertIsNone(self.w.nav_active)
        self.hb(); self.w.tick()
        self.assertEqual(self.w.state, FAILSAFE)   # settling: no proof yet
        for _ in range(4):                    # keep the heartbeat continuous while the new server settles
            time.sleep(0.12); self.hb(); self.w.tick()
        self.assertEqual(self.w.state, AUTHORIZED)  # settled + empty cancel-all for the NEW server
        self.assertEqual(self.w.status_gid, (b'\x02' * 16).hex())

    def test_leftover_goal_blocks_authorization(self):
        self.w.cancel.n_canceling = 1        # the server is still canceling an old goal
        self.hb(); self.w.tick(); time.sleep(0.25); self.hb(); self.w.tick()
        self.assertEqual(self.w.state, LOCKED)
        self.w.cancel.n_canceling = 0
        self.status(GoalStatus.STATUS_CANCELING)   # still active per status
        self.hb(); self.w.tick()
        self.assertEqual(self.w.state, LOCKED)
        self.status(GoalStatus.STATUS_CANCELED)
        self.hb(); self.w.tick()
        self.assertEqual(self.w.state, AUTHORIZED)

    def test_authorized_forwards_zero_and_nonzero(self):
        self.authorize()
        self.out.clear()
        t = Twist(); self.w.on_cmd(t)
        t2 = Twist(); t2.linear.x = 0.1; self.w.on_cmd(t2)
        self.assertEqual(self.out, [(0.0, 0.0), (0.1, 0.0)])

    def test_heartbeat_loss_failsafe_then_locked_then_rearm(self):
        self.authorize()
        self.w.hb_t -= 1.0                    # heartbeat stopped
        self.w.cancel.n_canceling = 1        # an active goal gets canceled
        self.w.tick()
        self.assertEqual(self.w.state, FAILSAFE)
        self.out.clear(); self.w.on_cmd(Twist(linear=Twist().linear))
        self.assertEqual(self.out, [])       # inputs dropped
        self.w.tick(); self.assertEqual(self.w.state, FAILSAFE)
        self.w.cancel.n_canceling = 0; self.w.tick()
        self.assertEqual(self.w.state, LOCKED)   # goals cleared, still needs heartbeat hold
        self.hb(boot='b2'); self.w.tick(); self.assertEqual(self.w.state, LOCKED)
        time.sleep(0.25); self.hb(boot='b2'); self.w.tick()
        self.assertEqual(self.w.state, AUTHORIZED)

    def test_gate_status_required_and_loss_trips(self):
        self.w.gate_t = None
        self.hb(gate=False); self.w.tick(); time.sleep(0.25); self.hb(gate=False); self.w.tick()
        self.assertEqual(self.w.state, LOCKED)               # never authorized without a live Gate
        self.status(); self.w.on_gate(String(data='{}')); self.hb(); self.w.tick()
        self.assertEqual(self.w.state, AUTHORIZED)
        self.w.gate_t -= 1.0; self.hb(gate=False); self.w.tick()
        self.assertNotEqual(self.w.state, AUTHORIZED)
        self.assertEqual(self.w.last_fault, 'SAFETY_GATE_STATUS_LOST')

    def test_control_loop_stall_and_unhealthy(self):
        self.authorize()
        self.hb(loop=2.0); self.w.tick()
        # Left AUTHORIZED through FAILSAFE (no goal outstanding -> immediately LOCKED); output stays zero.
        self.assertNotEqual(self.w.state, AUTHORIZED)
        self.assertEqual((self.w.counters['trips'], self.w.last_fault), (1, 'AGENT_CONTROL_LOOP_STALLED'))
        self.setUp(); self.authorize()
        self.hb(healthy=False); self.w.tick()
        self.assertNotEqual(self.w.state, AUTHORIZED)
        self.assertEqual(self.w.last_fault, 'AGENT_REPORTED_UNHEALTHY')

    # ---- idle-proof hardening (2026-11-20b)
    def test_nonzero_return_code_is_not_idle_proof(self):
        self.w.cancel.rc = CancelGoal.Response.ERROR_REJECTED
        self.hb(); self.w.tick(); time.sleep(0.25); self.hb(); self.w.tick()
        self.assertEqual(self.w.state, LOCKED)
        self.assertEqual(self.w.last_cancel_rc, CancelGoal.Response.ERROR_REJECTED)

    def test_rejected_cancel_with_empty_list_and_active_status_is_not_idle(self):
        # rclpy server whose cancel callback rejects: return_code 0, empty list, goal still EXECUTING (observed)
        self.status(GoalStatus.STATUS_EXECUTING)
        self.hb(); self.w.tick(); time.sleep(0.25); self.hb(); self.w.tick()
        self.assertEqual(self.w.state, LOCKED)
        self.assertEqual(self.w.reason, 'AWAITING_IDLE_PROOF')

    def test_no_or_duplicate_action_server_blocks_proof_and_trips(self):
        self.npubs = 0
        self.hb(); self.w.tick(); time.sleep(0.25); self.hb(); self.w.tick()
        self.assertEqual(self.w.state, LOCKED)
        self.npubs = 2
        self.hb(); self.w.tick()
        self.assertEqual(self.w.state, LOCKED)
        self.npubs = 1; self.hb(); self.w.tick(); self.status(); self.hb(); self.w.tick(); time.sleep(0.25); self.hb(); self.w.tick()
        self.assertEqual(self.w.state, AUTHORIZED)
        self.npubs = 2; self.hb(); self.w.tick()
        self.assertEqual(self.w.last_fault, 'NAV2_ACTION_SERVER_NOT_UNIQUE')

    def test_status_publisher_must_settle_before_proof(self):
        self.w.status_settle = 10.0
        self.hb(); self.w.tick(); time.sleep(0.25); self.hb(); self.w.tick()
        self.assertEqual(self.w.state, LOCKED)

    def test_foreign_goal_trips_after_grace_declared_goal_does_not(self):
        self.w.foreign_grace = 0.15
        self.authorize()
        mine = bytes([7]) * 16
        self.hb(goals=[mine.hex()]); self.status(GoalStatus.STATUS_EXECUTING, uid=mine)
        self.w.tick(); time.sleep(0.2); self.hb(goals=[mine.hex()]); self.w.tick()
        self.assertEqual(self.w.state, AUTHORIZED)
        self.status(GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_EXECUTING)   # an undeclared second goal
        self.status(GoalStatus.STATUS_EXECUTING, uid=bytes([9]) * 16)
        self.hb(goals=[mine.hex()]); self.w.tick()
        self.assertEqual(self.w.state, AUTHORIZED)                             # within grace
        time.sleep(0.2); self.hb(goals=[mine.hex()]); self.w.tick()
        self.assertEqual(self.w.state, FAILSAFE)
        self.assertEqual(self.w.last_fault, 'FOREIGN_NAV_GOAL')

    def test_lost_cancel_answer_times_out_and_retries(self):
        self.w.cancel.hang = True
        self.w.probe_timeout = 0.05
        self.hb(); self.w.tick()
        self.assertIsNotNone(self.w.probe_inflight)
        time.sleep(0.1); self.hb(); self.w.tick()
        self.assertEqual(self.w.counters['cancel_timeouts'], 1)
        self.assertEqual(self.w.state, LOCKED)

    def test_nonfinite_command_is_zeroed_and_missing_healthy_is_unhealthy(self):
        self.authorize()
        self.out.clear()
        t = Twist(); t.angular.z = float('nan'); self.w.on_cmd(t)
        self.assertEqual(self.out, [(0.0, 0.0)])
        self.assertEqual(self.w.counters['nonfinite_inputs'], 1)
        self.w.on_hb(String(data=json.dumps({'boot_id': 'b1', 'tasks_age_s': 0.01})))
        self.w.tick()
        self.assertEqual(self.w.last_fault, 'AGENT_REPORTED_UNHEALTHY')

    def test_self_stall_trips_even_if_heartbeat_looks_fresh(self):
        self.authorize()
        self.w.self_stall = 0.25
        time.sleep(0.3)                       # this process was not scheduled (SIGSTOP / starvation)
        self.hb(); self.w.tick()              # a queued heartbeat is processed first after resume
        self.assertNotEqual(self.w.state, AUTHORIZED)
        self.assertEqual(self.w.last_fault, 'WATCHDOG_SELF_STALL')


if __name__ == '__main__':
    unittest.main()
