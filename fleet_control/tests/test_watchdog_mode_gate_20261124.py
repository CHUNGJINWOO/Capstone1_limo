"""Watchdog mode gate (G2, 2026-11-24): real rclpy MotionWatchdog, callbacks driven directly, reusing the R2/clamp test scaffolding. Not a real-robot result."""
import json, time, unittest
from types import SimpleNamespace
import test_watchdog_speed_clamp_20261123 as base
from test_watchdog_speed_clamp_20261123 import AUTHORIZED, LOCKED, FAILSAFE, Twist, String, time as _t
from limo_robot_agent.mode_policy import ModePolicy, GRANT_CONFIRM_S


OWNER = {'MAPPING': 'frontier', 'NAVIGATION': 'fleet'}


def mode_msg(name, rev=1, boot='b1', epoch=1, scopes=None, owner='default', cancel=False):
    owner = OWNER.get(name) if owner == 'default' else owner
    return String(data=json.dumps({'agent_boot_id': boot, 'binding': ['sb', 5, 3], 'seq': 1, 'scopes': scopes or {}, 'mode': {'name': name, 'revision': rev, 'epoch': epoch}, 'goal': {'owner': owner, 'cancel_requested': cancel}}))


def twist(x, z=0.0):
    t = Twist(); t.linear.x, t.angular.z = x, z; return t


class Gate(base.ClampTests):
    def setUp(self):
        super().setUp()
        self.w.mode_policy = ModePolicy(); self.w.manual_nodes = ('manual_mapping_teleop',); self.w.source_conflict_t = None
        self.man_pubs = ['manual_mapping_teleop']; self.man_gid = b'\x03' * 16
        prev = self.w.get_publishers_info_by_topic
        def pubs(topic):
            if topic == self.w.manual_topic:
                return [SimpleNamespace(node_name=n, endpoint_gid=self.man_gid) for n in self.man_pubs]
            return prev(topic)
        self.w.get_publishers_info_by_topic = pubs
        self.w._update_graph(time.monotonic())

    def set_mode(self, name, rev=1, grant=True):
        self.w.on_speed_limit(mode_msg(name, rev))
        if grant and name in ('MANUAL', 'MAPPING', 'NAVIGATION'):
            time.sleep(GRANT_CONFIRM_S + 0.02); self.hb(); self.w.on_speed_limit(mode_msg(name, rev))

    def authorize(self):
        self.w.self_stall = 10.0
        self.status(); self.hb(); self.set_mode('IDLE'); self.w.tick(); time.sleep(0.25); self.hb(); self.set_mode('IDLE'); self.w.tick()
        self.assertEqual(self.w.state, AUTHORIZED, self.w.reason)

    def nav(self, x, z=0.0):
        self.out.clear(); self.w.on_cmd(twist(x, z)); return self.out[-1] if self.out else None

    def man(self, x, z=0.0):
        self.out.clear(); self.w.on_manual_cmd(twist(x, z)); return self.out[-1] if self.out else None

    def test_no_mode_means_no_motion_from_either_channel(self):
        self.authorize(); self.assertEqual(self.nav(0.1), (0.0, 0.0)); self.assertEqual(self.man(0.1), (0.0, 0.0)); self.assertIsNone(self.w.source_conflict_t)

    def test_mapping_and_navigation_forward_the_nav_channel_only(self):
        for name in ('MAPPING', 'NAVIGATION'):
            with self.subTest(name):
                self.authorize(); self.set_mode(name, 2); self.assertEqual(self.nav(0.5, 0.9), (0.15, 0.5), 'nav channel, auto ceilings')
                self.assertEqual(self.man(0.0, 0.0), (0.0, 0.0)); self.w.source_conflict_t = None

    def test_manual_forwards_the_manual_channel_only_with_manual_ceilings(self):
        self.authorize(); self.set_mode('MANUAL', 2); self.man(0.0)          # a zero re-arms the joystick (deadman rule)
        self.assertEqual(self.man(0.5, 0.9), (0.10, 0.30)); self.assertEqual(self.man(-0.5, 0.0), (-0.10, 0.0))
        self.assertEqual(self.nav(0.1), (0.0, 0.0))

    def test_motion_on_the_other_channel_is_a_source_conflict_and_trips_the_watchdog(self):
        for mode, offender in (('MAPPING', 'man'), ('MANUAL', 'nav')):
            with self.subTest(mode):
                self.authorize(); self.set_mode(mode, 3); self.assertEqual(getattr(self, offender)(0.1), (0.0, 0.0))
                self.assertIsNotNone(self.w.source_conflict_t); self.hb(); self.set_mode(mode, 3, grant=False); self.w.tick()
                self.assertNotEqual(self.w.state, AUTHORIZED); self.assertEqual(self.w.last_fault, 'MOTION_SOURCE_CONFLICT'); self.w.source_conflict_t = None

    def test_the_nav_channel_stops_when_the_owned_goal_ends_or_a_cancel_is_requested_even_if_nav2_keeps_commanding(self):
        self.authorize(); self.set_mode('MAPPING', 2); self.assertEqual(self.nav(0.1), (0.1, 0.0))
        self.w.on_speed_limit(mode_msg('MAPPING', 2, cancel=True)); self.assertEqual(self.nav(0.1), (0.0, 0.0), 'cancel requested')
        self.w.on_speed_limit(mode_msg('MAPPING', 2, owner=None)); self.assertEqual(self.nav(0.1), (0.0, 0.0), 'no owned goal')
        self.w.on_speed_limit(mode_msg('MAPPING', 2)); self.assertEqual(self.nav(0.1), (0.0, 0.0), 'a new goal is granted only on the second message')
        time.sleep(GRANT_CONFIRM_S + 0.02); self.hb(); self.w.on_speed_limit(mode_msg('MAPPING', 2)); self.assertEqual(self.nav(0.1), (0.1, 0.0))
        self.assertIsNone(self.w.source_conflict_t, 'a blocked nav command is not a source conflict')

    def test_a_joystick_held_when_manual_is_granted_is_not_forwarded_until_it_was_released_once(self):
        self.authorize(); self.set_mode('MANUAL', 5)
        for _ in range(5):
            self.assertEqual(self.man(0.1), (0.0, 0.0), 'held through the grant window')
        self.assertEqual(self.man(0.0), (0.0, 0.0)); self.assertEqual(self.man(0.1), (0.1, 0.0), 'released once, then a new deflection is forwarded')
        self.assertGreaterEqual(self.w.mode_policy.counters['manual_rearm_blocked'], 5)

    def test_silence_after_a_held_deflection_is_not_a_release_only_a_neutral_command_is(self):
        self.authorize(); self.set_mode('MANUAL', 5)
        for _ in range(3):
            self.assertEqual(self.man(0.1), (0.0, 0.0))
        time.sleep(0.6); self.hb()
        self.assertEqual(self.man(0.1), (0.0, 0.0), 'silence >0.5 s then a non-zero command: still the held deflection')
        self.assertEqual(self.man(0.0), (0.0, 0.0)); self.assertEqual(self.man(0.1), (0.1, 0.0), 'neutral, then a fresh deflection')

    def test_silence_during_the_window_also_rearms_and_a_new_grant_starts_a_new_window(self):
        self.authorize(); self.set_mode('MANUAL', 5); time.sleep(0.55); self.hb(); self.assertEqual(self.man(0.1), (0.1, 0.0), 'no command during the window: a later press is a new deflection')
        self.set_mode('IDLE', 6, grant=False); self.hb(); self.set_mode('MANUAL', 7); self.assertEqual(self.man(0.1), (0.0, 0.0), 'a new MANUAL revision needs the re-arm again')

    # ---- release before resume (2026-11-25): the Gate-restart finding
    def gate_outage(self, silence=False, zero=False, held=8):
        """Gate status lost -> FAILSAFE/LOCKED while the stick stays deflected (messages keep arriving and are dropped), then the Gate is back and the Watchdog re-authorizes."""
        self.w.gate_t -= 1.0; self.hb(gate=False); self.w.tick(); self.assertNotEqual(self.w.state, AUTHORIZED)
        for _ in range(held):
            self.out.clear(); self.w.on_manual_cmd(twist(0.1)); self.assertEqual(self.out, [], 'nothing forwarded while not authorized')
        if zero:
            self.w.on_manual_cmd(twist(0.0))
        if silence:
            time.sleep(0.6)
        self.w.on_gate(String(data='{}')); self.status(); self.hb(); self.w.tick(); time.sleep(0.25); self.hb(); self.w.on_gate(String(data='{}')); self.w.tick()
        self.assertEqual(self.w.state, AUTHORIZED, self.w.reason); self.set_mode('MANUAL', 5)

    def enter_manual_flowing(self):
        self.authorize(); self.set_mode('MANUAL', 5); self.man(0.0); self.assertEqual(self.man(0.1), (0.1, 0.0), 'positive control')

    def test_a_stick_held_through_a_gate_outage_is_not_forwarded_when_the_gate_returns(self):
        self.enter_manual_flowing(); self.gate_outage()
        for _ in range(5):
            self.assertEqual(self.man(0.1), (0.0, 0.0), 'same deflection as before the outage: still blocked')
        self.assertEqual(self.man(0.0), (0.0, 0.0)); self.assertEqual(self.man(0.1), (0.1, 0.0), 'released once, then a new deflection moves')
        self.assertEqual(self.w.counters.get('release_rearms'), 1, 'one neutral-command release after the outage')

    def test_silence_during_the_outage_is_not_a_release_but_a_neutral_command_is(self):
        self.enter_manual_flowing(); self.gate_outage(silence=True); self.assertEqual(self.man(0.1), (0.0, 0.0), 'the stream was silent >0.5 s but never neutral: not a release')
        self.setUp(); self.enter_manual_flowing(); self.gate_outage(zero=True); self.assertEqual(self.man(0.1), (0.1, 0.0), 'a neutral command was seen during the outage')

    def test_each_new_episode_needs_its_own_release_and_the_previous_release_does_not_carry_over(self):
        self.enter_manual_flowing(); self.gate_outage(zero=True); self.assertEqual(self.man(0.1), (0.1, 0.0))
        self.gate_outage(); self.assertEqual(self.man(0.1), (0.0, 0.0), 'a release from the previous episode is not reused')
        self.assertEqual(self.man(0.0), (0.0, 0.0)); self.assertEqual(self.man(0.1), (0.1, 0.0))

    def test_a_stick_held_at_the_grant_needs_a_neutral_command_and_a_stick_never_seen_deflected_does_not(self):
        self.authorize(); self.set_mode('MANUAL', 5)
        for _ in range(4):
            self.assertEqual(self.man(0.1), (0.0, 0.0))
        self.setUp(); self.authorize(); self.set_mode('MANUAL', 5); time.sleep(0.55); self.hb()
        self.assertEqual(self.w.man_neutral, None); self.assertEqual(self.man(0.1), (0.1, 0.0), 'no deflection was ever observed before: a first press is a new deflection')

    def test_a_publisher_that_stops_for_more_than_600_ms_while_deflected_does_not_resume_by_itself(self):
        self.enter_manual_flowing()
        for _ in range(3):
            self.assertEqual(self.man(0.1), (0.1, 0.0))
        time.sleep(0.65); self.hb()                                              # communication loss with the stick still held
        self.assertEqual(self.man(0.1), (0.0, 0.0), 'comm restored, same non-zero command: blocked'); self.assertEqual(self.man(0.1), (0.0, 0.0))
        self.assertEqual(self.w.counters.get('silence_latches'), 1)
        self.assertEqual(self.man(0.0), (0.0, 0.0), 'explicit neutral'); self.assertEqual(self.man(0.1), (0.1, 0.0), 'fresh deflection after neutral')

    def test_silence_after_a_neutral_command_is_a_normal_new_press(self):
        self.enter_manual_flowing(); self.man(0.0); time.sleep(0.65); self.hb()
        self.assertEqual(self.man(0.1), (0.1, 0.0), 'the stick was known neutral before the silence')

    def test_a_recreated_manual_publisher_even_under_the_same_name_needs_a_neutral_command(self):
        self.enter_manual_flowing(); self.man(0.1)
        self.man_gid = b'\x07' * 16; self.w._update_graph(time.monotonic())      # endpoint removed and a same-named publisher created
        self.assertEqual(self.man(0.1), (0.0, 0.0)); self.assertEqual(self.w.counters.get('endpoint_latches'), 1)
        self.assertEqual(self.man(0.0), (0.0, 0.0)); self.assertEqual(self.man(0.1), (0.1, 0.0))
        self.man(0.0); self.man_gid = b'\x09' * 16; self.w._update_graph(time.monotonic()); self.assertEqual(self.man(0.1), (0.1, 0.0), 'known-neutral stick: an endpoint change latches nothing')
        self.assertEqual(self.w.counters.get('endpoint_latches'), 1)

    def test_the_nav_channel_is_not_affected_by_the_manual_release_rule(self):
        self.authorize(); self.set_mode('MAPPING', 6)
        self.assertEqual(self.nav(0.5, 0.9), (0.15, 0.5))

    def test_a_release_seen_while_another_mode_is_selected_does_not_mask_a_later_held_stick(self):
        self.enter_manual_flowing(); self.set_mode('IDLE', 7); self.man(0.0); self.set_mode('MANUAL', 8); self.assertEqual(self.man(0.0), (0.0, 0.0)); self.assertEqual(self.man(0.1), (0.1, 0.0))
        self.set_mode('IDLE', 9); self.set_mode('MANUAL', 10)
        for _ in range(4):
            self.assertEqual(self.man(0.1), (0.0, 0.0), 'held through a new MANUAL grant: the grant-window rule still applies')

    # ---- idle zero (2026-11-27): the explicit latest zero a gated Gate reset needs
    def test_idle_zero_is_off_by_default_and_never_starts_or_interleaves_motion(self):
        self.authorize(); self.set_mode('MAPPING', 6); self.out.clear(); self.w.tick(); self.assertEqual(self.out, [], 'default: nothing is published while idle')
        self.w.idle_zero_period = 0.05; time.sleep(0.06); self.hb(); self.w.tick(); self.assertEqual(self.out[-1], (0.0, 0.0), 'idle: an explicit zero')
        self.out.clear(); self.nav(0.1); self.w.tick(); self.assertEqual([o for o in self.out if o != (0.0, 0.0)][:1], [(0.1, 0.0)], 'a forwarded command is forwarded as before')
        n = len(self.out); self.w.tick(); self.assertEqual(len(self.out), n, 'no zero is inserted right after a forwarded command (output age < period)')

    def test_idle_zero_is_not_published_outside_authorized(self):
        self.authorize(); self.w.idle_zero_period = 0.01; self.w.gate_t -= 1.0; self.hb(gate=False); self.w.tick(); self.assertNotEqual(self.w.state, AUTHORIZED)
        self.out.clear(); time.sleep(0.02); self.w.tick(); self.assertTrue(all(o == (0.0, 0.0) for o in self.out), 'only zeros, as in every non-authorized state')

    def test_zero_on_the_other_channel_is_not_a_conflict(self):
        self.authorize(); self.set_mode('MAPPING', 2); self.man(0.0, 0.0); self.assertIsNone(self.w.source_conflict_t)

    def test_idle_and_transitioning_stop_forwarding_at_once(self):
        self.authorize(); self.set_mode('MAPPING', 2); self.assertEqual(self.nav(0.1), (0.1, 0.0))
        for name in ('TRANSITIONING', 'IDLE', 'FAULT'):
            self.w.on_speed_limit(mode_msg(name, 5)); self.assertEqual(self.nav(0.1), (0.0, 0.0), name); self.assertEqual(self.man(0.1), (0.0, 0.0), name)
            self.w.source_conflict_t = None

    def test_mapping_to_manual_switch_forwards_nothing_until_the_new_mode_is_granted(self):
        self.authorize(); self.set_mode('MAPPING', 2); self.assertEqual(self.nav(0.1), (0.1, 0.0))
        self.w.on_speed_limit(mode_msg('MANUAL', 3)); self.assertEqual(self.man(0.1), (0.0, 0.0), 'first MANUAL message grants nothing'); self.assertEqual(self.nav(0.1), (0.0, 0.0))
        self.w.source_conflict_t = None; time.sleep(GRANT_CONFIRM_S + 0.02); self.w.on_speed_limit(mode_msg('MANUAL', 3)); self.man(0.0); self.assertEqual(self.man(0.1), (0.1, 0.0))

    def test_a_second_or_foreign_publisher_on_either_channel_fails_closed(self):
        self.authorize(); self.set_mode('MAPPING', 2)
        for pubs in (['manual_mapping_teleop', 'rogue'], ['rogue']):
            self.man_pubs = pubs; self.w._update_graph(time.monotonic()); self.assertFalse(self.w.manual_input_ok); self.hb(); self.set_mode('MAPPING', 2, grant=False); self.w.tick()
            self.assertNotEqual(self.w.state, AUTHORIZED); self.assertEqual(self.w.last_fault, 'VELOCITY_INPUT_NOT_EXCLUSIVE')
            self.out.clear(); self.w.on_cmd(twist(0.1)); self.assertEqual([o for o in self.out if o != (0.0, 0.0)], [])
            self.man_pubs = ['manual_mapping_teleop']; self.w._update_graph(time.monotonic()); self.authorize(); self.set_mode('MAPPING', 2)

    def test_no_grant_without_agent_sync_and_a_foreign_policy_publisher_is_ignored(self):
        self.authorize(); self.sl_pubs = ['fleet_robot_agent', 'rogue']; self.w._update_graph(time.monotonic())
        self.w.on_speed_limit(mode_msg('MAPPING', 9)); time.sleep(GRANT_CONFIRM_S + 0.02); self.w.on_speed_limit(mode_msg('MAPPING', 9))
        self.assertEqual(self.nav(0.1), (0.0, 0.0)); self.assertFalse(self.w.mode_policy.permits('nav', 'b1')[0])

    def test_status_block_and_last_output(self):
        self.authorize(); self.set_mode('MANUAL', 4); self.man(0.0); self.man(0.05); w = self.w
        w.publish_status() if False else None
        st = w.mode_policy.status('b1'); self.assertEqual((st['mode'], st['revision'], st['selected_channel'], st['permits_manual'], st['permits_nav']), ('MANUAL', 4, 'manual', True, False))
        self.assertEqual(w.last_out[1:], (0.05, 0.0))

    def test_without_the_mode_gate_the_nav_input_behaves_exactly_as_before(self):
        self.w.mode_policy = None; self.authorize() if False else None
        self.status(); self.hb(); self.sync(); self.w.tick(); time.sleep(0.25); self.hb(); self.sync(); self.w.tick()
        self.assertEqual(self.w.state, AUTHORIZED); self.assertEqual(self.nav(0.5), (0.15, 0.0)); self.assertTrue(self.w.manual_input_ok)


if __name__ == '__main__':
    unittest.main()
