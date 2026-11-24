"""Real-Agent mode selection (G2, 2026-11-24): mode policy, Watchdog channel gate, RealModeSelector evidence and the server's re-check. No robot, stub ROS where needed."""
import sys, time, unittest
from pathlib import Path
from types import SimpleNamespace
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'agent' / 'limo_robot_agent'), str(ROOT / 'server'), str(ROOT / 'tests')]
from limo_robot_agent.mode_policy import ModePolicy, GRANT_CONFIRM_S, MODE_CHANNEL


OWNER = {'MAPPING': 'frontier', 'NAVIGATION': 'fleet'}


def msg(name, rev=1, boot='b1', epoch=1, owner='default', cancel=False):
    owner = OWNER.get(name) if owner == 'default' else owner
    return {'agent_boot_id': boot, 'mode': {'name': name, 'revision': rev, 'epoch': epoch}, 'goal': {'owner': owner, 'cancel_requested': cancel}}


class Policy(unittest.TestCase):
    def grant(self, p, name, rev=1, boot='b1'):
        p.accept(msg(name, rev, boot), boot, True); time.sleep(GRANT_CONFIRM_S + 0.02); return p.accept(msg(name, rev, boot), boot, True)

    def test_nothing_is_allowed_before_any_message_or_after_a_bad_one(self):
        p = ModePolicy(); self.assertEqual(p.permits('nav', 'b1'), (False, 'NO_MODE_POLICY'))
        self.grant(p, 'MAPPING'); self.assertTrue(p.permits('nav', 'b1')[0])
        for bad, why in ((msg('MAPPING', boot='other'), 'boot'), ({'agent_boot_id': 'b1'}, 'no mode'), ({'agent_boot_id': 'b1', 'mode': {'name': 'FLY', 'revision': 1}}, 'unknown mode'),
                         ({'agent_boot_id': 'b1', 'mode': {'name': 'MAPPING', 'revision': '1'}}, 'revision type')):
            with self.subTest(why):
                p = ModePolicy(); self.grant(p, 'MAPPING'); p.accept(bad, 'b1', True); self.assertFalse(p.permits('nav', 'b1')[0], 'a bad message must drop the grant')
        p = ModePolicy(); self.grant(p, 'MAPPING'); p.accept(msg('MAPPING'), 'b1', False); self.assertFalse(p.permits('nav', 'b1')[0], 'no allowed single publisher')

    def test_channel_per_mode(self):
        for name, ch, other in (('MAPPING', 'nav', 'manual'), ('NAVIGATION', 'nav', 'manual'), ('MANUAL', 'manual', 'nav')):
            p = ModePolicy(); self.grant(p, name)
            self.assertTrue(p.permits(ch, 'b1')[0], name); ok, why = p.permits(other, 'b1'); self.assertFalse(ok); self.assertTrue(why.startswith('CHANNEL_NOT_SELECTED'), why)
        for name in ('IDLE', 'TRANSITIONING', 'FAULT', 'UNKNOWN'):
            p = ModePolicy(); p.accept(msg(name), 'b1', True); self.assertFalse(p.permits('nav', 'b1')[0]); self.assertFalse(p.permits('manual', 'b1')[0])

    def test_granting_needs_two_messages_and_narrowing_is_immediate(self):
        p = ModePolicy(); self.assertFalse(p.accept(msg('MANUAL'), 'b1', True)); self.assertFalse(p.permits('manual', 'b1')[0])
        time.sleep(GRANT_CONFIRM_S + 0.02); self.assertTrue(p.accept(msg('MANUAL'), 'b1', True)); self.assertTrue(p.permits('manual', 'b1')[0])
        p.accept(msg('IDLE', 2), 'b1', True); self.assertFalse(p.permits('manual', 'b1')[0], 'IDLE takes effect at once')
        p.accept(msg('MAPPING', 3), 'b1', True); self.assertFalse(p.permits('nav', 'b1')[0], 'a first MAPPING message grants nothing')

    def test_mode_change_withdraws_the_old_grant_at_once(self):
        p = ModePolicy(); self.grant(p, 'MAPPING'); p.accept(msg('MANUAL', 2), 'b1', True)
        self.assertFalse(p.permits('nav', 'b1')[0]); self.assertFalse(p.permits('manual', 'b1')[0], 'the new grant is not valid yet')

    def test_stale_policy_allows_nothing_and_a_new_boot_resets_it(self):
        p = ModePolicy(); self.grant(p, 'MAPPING'); p.cur['seen_t'] -= 5; self.assertEqual(p.permits('nav', 'b1')[1], 'MODE_POLICY_STALE')
        p = ModePolicy(); self.grant(p, 'MAPPING'); self.assertFalse(p.permits('nav', 'b2')[0])

    def test_the_nav_channel_needs_the_owned_goal_and_stops_at_once_when_it_ends_or_a_cancel_is_requested(self):
        p = ModePolicy(); self.grant(p, 'MAPPING'); self.assertTrue(p.permits('nav', 'b1')[0])
        p.accept(msg('MAPPING', owner=None), 'b1', True); self.assertEqual(p.permits('nav', 'b1'), (False, 'NO_OWNED_GOAL'))            # goal ended: immediately
        self.assertFalse(p.accept(msg('MAPPING'), 'b1', True)); self.assertFalse(p.permits('nav', 'b1')[0], 'a new goal grants only after the second message')
        time.sleep(GRANT_CONFIRM_S + 0.02); self.assertTrue(p.accept(msg('MAPPING'), 'b1', True)); self.assertTrue(p.permits('nav', 'b1')[0])
        p.accept(msg('MAPPING', cancel=True), 'b1', True); self.assertFalse(p.permits('nav', 'b1')[0], 'cancel requested: stop at once')
        for wrong in ('fleet', 'someone'):
            q = ModePolicy(); q.accept(msg('MAPPING', owner=wrong), 'b1', True); time.sleep(GRANT_CONFIRM_S + 0.02); q.accept(msg('MAPPING', owner=wrong), 'b1', True)
            self.assertFalse(q.permits('nav', 'b1')[0], wrong)
        q = ModePolicy(); self.grant(q, 'NAVIGATION'); self.assertTrue(q.permits('nav', 'b1')[0])
        q = ModePolicy(); q.accept({'agent_boot_id': 'b1', 'mode': {'name': 'MAPPING', 'revision': 1}}, 'b1', True); time.sleep(GRANT_CONFIRM_S + 0.02)
        q.accept({'agent_boot_id': 'b1', 'mode': {'name': 'MAPPING', 'revision': 1}}, 'b1', True); self.assertFalse(q.permits('nav', 'b1')[0], 'a message without goal information grants nothing')

    def test_manual_is_not_granted_while_the_agent_owns_a_goal(self):
        p = ModePolicy(); p.accept(msg('MANUAL', owner='frontier'), 'b1', True); time.sleep(GRANT_CONFIRM_S + 0.02); p.accept(msg('MANUAL', owner='frontier'), 'b1', True)
        self.assertEqual(p.permits('manual', 'b1'), (False, 'GOAL_ACTIVE_IN_MANUAL'))

    def test_a_second_writer_between_messages_cannot_grant_motion(self):
        p = ModePolicy(); p.accept(msg('MAPPING'), 'b1', True); time.sleep(GRANT_CONFIRM_S + 0.02); self.assertFalse(p.accept(msg('MAPPING'), 'b1', False)); self.assertFalse(p.permits('nav', 'b1')[0])


class ServerConfirm(unittest.TestCase):
    def setUp(self):
        from fleet_server.operating_mode import OperatingModeMixin
        self.f = type('F', (OperatingModeMixin,), {})()

    def robot(self, target='MAPPING', **ev_over):
        ev = {'watchdog_state': 'MOTION_AUTHORIZED', 'watchdog_mode': target, 'watchdog_revision': 7, 'selected_channel': MODE_CHANNEL.get(target), 'nav_input_ok': True, 'manual_input_ok': True,
              'source_conflict_age_s': None, 'watchdog_boot_id': 'wd1', 'agent_graph': {'nav_publishers': ['velocity_smoother'], 'manual_publishers': []}}
        ev.update(ev_over)
        return {'agent': {'velocity_selector': 'WATCHDOG_SELECTOR'},
                'state': {'velocity_arbiter': {'kind': 'WATCHDOG_SELECTOR', 'operating_mode': target, 'blocker': {'MANUAL': None, 'MAPPING': 'NO_OWNED_GOAL', 'NAVIGATION': 'NO_OWNED_GOAL'}[target], 'evidence': ev},
                          'speed_limits': {'watchdog_evidence': {'server_verified': True, 'watchdog_boot_id': 'wd1'}}}}

    def test_accepts_only_with_complete_matching_evidence(self):
        for t in ('MANUAL', 'MAPPING', 'NAVIGATION'):
            self.assertIsNone(self.f._mode_confirm_blocker(self.robot(t), t, 7), t)

    def test_every_missing_or_wrong_piece_refuses(self):
        cases = {'wrong revision': dict(watchdog_revision=6), 'Watchdog still in another mode': dict(watchdog_mode='MANUAL'), 'wrong channel': dict(selected_channel='manual'),
                 'nav input not exclusive': dict(nav_input_ok=False), 'manual input not exclusive': dict(manual_input_ok=False), 'source conflict': dict(source_conflict_age_s=1.0),
                 'watchdog not authorized': dict(watchdog_state='FAILSAFE'), 'unexpected nav publisher': dict(agent_graph={'nav_publishers': ['rogue'], 'manual_publishers': []}),
                 'two nav publishers': dict(agent_graph={'nav_publishers': ['velocity_smoother', 'velocity_smoother'], 'manual_publishers': []}),
                 'manual publisher not the joystick': dict(agent_graph={'nav_publishers': [], 'manual_publishers': ['rogue']}), 'no graph': dict(agent_graph=None)}
        for name, over in cases.items():
            with self.subTest(name):
                self.assertIsNotNone(self.f._mode_confirm_blocker(self.robot('MAPPING', **over), 'MAPPING', 7), name)

    def test_evidence_of_another_watchdog_boot_or_unverified_is_refused(self):
        r = self.robot(); r['state']['speed_limits']['watchdog_evidence']['watchdog_boot_id'] = 'other'; self.assertIn('not verified', self.f._mode_confirm_blocker(r, 'MAPPING', 7))
        r = self.robot(); r['state']['speed_limits']['watchdog_evidence']['server_verified'] = False; self.assertIsNotNone(self.f._mode_confirm_blocker(r, 'MAPPING', 7))
        r = self.robot(); r['state'].pop('speed_limits'); self.assertIsNotNone(self.f._mode_confirm_blocker(r, 'MAPPING', 7))

    def test_not_ready_blocker_and_wrong_kind_and_missing_report(self):
        r = self.robot(); r['state']['velocity_arbiter']['blocker'] = 'GATE_NOT_NORMAL'; self.assertIn('not ready', self.f._mode_confirm_blocker(r, 'MAPPING', 7))
        r = self.robot(); r['state']['velocity_arbiter']['kind'] = 'ARBITER'; self.assertIn('not a Watchdog selector', self.f._mode_confirm_blocker(r, 'MAPPING', 7))
        r = self.robot(); r['state'].pop('velocity_arbiter'); self.assertIn('missing', self.f._mode_confirm_blocker(r, 'MAPPING', 7))
        r = self.robot(); r['agent'] = {}; self.assertIn('no velocity selector', self.f._mode_confirm_blocker(r, 'MAPPING', 7)); self.assertIsNone(self.f._mode_confirm_blocker(r, 'IDLE', 7))

    def test_idle_needs_no_watchdog_evidence_but_the_report_must_say_idle(self):
        r = self.robot('MAPPING'); r['state']['velocity_arbiter'].update(operating_mode='IDLE', blocker='MODE_IDLE'); self.assertIsNone(self.f._mode_confirm_blocker(r, 'IDLE', 8))
        r['state']['velocity_arbiter']['operating_mode'] = 'MAPPING'; self.assertIsNotNone(self.f._mode_confirm_blocker(r, 'IDLE', 8))

    def test_manual_input_path_for_a_real_agent_comes_from_the_selector_graph_evidence(self):
        r = self.robot('MANUAL'); r['state']['velocity_arbiter']['evidence']['agent_graph'] = {'nav_publishers': ['velocity_smoother'], 'manual_publishers': ['manual_mapping_teleop']}
        r['state']['velocity_arbiter']['evidence']['manual_input_ok'] = True; self.assertTrue(self.f._selector_manual_path(r))
        for mut in (lambda x: x['state']['velocity_arbiter']['evidence']['agent_graph'].update(manual_publishers=[]), lambda x: x['state']['velocity_arbiter']['evidence']['agent_graph'].update(manual_publishers=['manual_mapping_teleop', 'rogue']),
                    lambda x: x['state']['velocity_arbiter']['evidence']['agent_graph'].update(manual_publishers=['rogue']), lambda x: x['state']['velocity_arbiter']['evidence'].update(manual_input_ok=False),
                    lambda x: x['agent'].clear(), lambda x: x['state']['velocity_arbiter'].update(kind='ARBITER'), lambda x: x['state'].pop('velocity_arbiter')):
            q = self.robot('MANUAL'); q['state']['velocity_arbiter']['evidence']['agent_graph'] = {'nav_publishers': [], 'manual_publishers': ['manual_mapping_teleop']}
            q['state']['velocity_arbiter']['evidence']['manual_input_ok'] = True; mut(q); self.assertFalse(self.f._selector_manual_path(q))

    def test_the_mock_arbiter_path_is_unchanged(self):
        r = {'agent': {'locked_velocity_arbiter': True}, 'state': {'velocity_arbiter': {'operating_mode': 'MAPPING', 'blocker': 'NO_OWNED_GOAL'}}}
        self.assertIsNone(self.f._mode_confirm_blocker(r, 'MAPPING', 3))


class ActiveModeVerification(unittest.TestCase):
    """The server keeps checking an ACTIVE mode against the Watchdog's own report (2026-11-25): mismatch = no new motion / mission, then FAULT or IDLE, never an automatic return."""
    def setUp(self):
        import threading
        from fleet_server import operating_mode as om
        self.om = om
        events = self.events = []
        test = self

        class F(om.OperatingModeMixin):
            boot_id = 'sb'
            def __init__(self):
                self.lock = threading.RLock(); self.robots = {}; self.rec = None; self.manual_benches = {}; self.cleared = []; self.mapping_open = False
                self.db = SimpleNamespace(event=lambda *a, **k: events.append(a))
            def _mode_load(self, rid): return self.rec
            def _mode_save(self, rid, rec): self.rec = rec
            def lease_view(self, rid): return {'epoch': 1, 'owner': 'op'}
            def agent_contract_ok(self, rid): return True
            def _mapping_unresolved(self, rid): return self.mapping_open
            def navigation_clear(self, rid, why): self.cleared.append(why)
        self.f = F()
        self.f.cfg = SimpleNamespace(require_operating_mode=True)

    def robot(self, mode='MANUAL', rev=7, **ev_over):
        ev = {'watchdog_state': 'MOTION_AUTHORIZED', 'watchdog_mode': mode, 'watchdog_revision': rev, 'selected_channel': MODE_CHANNEL.get(mode), 'nav_input_ok': True, 'manual_input_ok': True,
              'source_conflict_age_s': None, 'watchdog_boot_id': 'wd1', 'watchdog_trips': 0, 'agent_graph': {'nav_publishers': ['velocity_smoother'], 'manual_publishers': ['manual_mapping_teleop']}}
        ev.update(ev_over)
        return {'online': True, 'reconciled': True, 'state_ts': time.time(), 'agent': {'velocity_selector': 'WATCHDOG_SELECTOR', 'boot_id': 'ab'},
                'state': {'velocity_arbiter': {'kind': 'WATCHDOG_SELECTOR', 'operating_mode': mode, 'blocker': None, 'evidence': ev},
                          'speed_limits': {'watchdog_evidence': {'server_verified': True, 'watchdog_boot_id': 'wd1'}}}}

    def install(self, mode='MANUAL', rev=7, **ev_over):
        self.f.robots['r'] = self.robot(mode, rev, **ev_over)
        self.f.rec = {'mode': mode, 'revision': rev, 'epoch': 1, 'owner': 'op', 'agent_boot': 'ab', 'server_boot': 'sb', 'since': time.time(), 'reason': 'x', 'transition': {'state': 'ACTIVE', 'to': mode}, 'watchdog_boot': 'wd1', 'watchdog_trips': 0}

    def test_a_confirmed_mode_is_verified_and_admits_commands(self):
        for m in ('MANUAL', 'MAPPING', 'NAVIGATION'):
            self.install(m); self.assertEqual(self.f.mode_motion_status('r')['verified'], True, m); self.f.require_mode('r', m, 'x')

    def test_every_mismatch_blocks_new_motion_at_once(self):
        cases = {'watchdog back in IDLE': dict(watchdog_mode='IDLE'), 'other revision': dict(watchdog_revision=3), 'wrong channel': dict(selected_channel='nav'), 'watchdog LOCKED': dict(watchdog_state='LOCKED'),
                 'input not exclusive': dict(manual_input_ok=False), 'source conflict': dict(source_conflict_age_s=0.5), 'another watchdog boot': dict(watchdog_boot_id='wd2')}
        from fleet_server.service import ApiError
        for name, over in cases.items():
            with self.subTest(name):
                self.install('MANUAL'); self.f.robots['r'] = self.robot('MANUAL', 7, **over)
                if name == 'another watchdog boot': self.f.robots['r']['state']['speed_limits']['watchdog_evidence']['watchdog_boot_id'] = 'wd2'
                st = self.f.mode_motion_status('r'); self.assertIs(st['verified'], False, st); self.assertTrue(st['hard'])
                with self.assertRaises(ApiError) as cm: self.f.require_mode('r', 'MANUAL', 'x')
                self.assertEqual(cm.exception.code if hasattr(cm.exception, 'code') else cm.exception.args[1], 'mode_unverified')

    def test_the_watchdog_restart_does_not_inherit_the_old_confirmation(self):
        self.install('MANUAL'); self.f.rec['watchdog_boot'] = 'wd0'                         # confirmed under another Watchdog boot, the new one now reports the same mode and revision
        st = self.f.mode_motion_status('r'); self.assertIs(st['verified'], False); self.assertIn('restarted', st['blocker'])

    def test_stale_or_offline_reports_are_unverified_but_not_a_contradiction(self):
        self.install('MANUAL'); self.f.robots['r']['state_ts'] -= 10; st = self.f.mode_motion_status('r'); self.assertIs(st['verified'], False); self.assertFalse(st['hard'])
        self.install('MANUAL'); self.f.robots['r']['online'] = False; st = self.f.mode_motion_status('r'); self.assertIs(st['verified'], False); self.assertFalse(st['hard'])
        self.install('MANUAL'); self.f.robots['r']['state_ts'] -= 10; self.f._mode_invalidate('r', self.f.rec); self.assertEqual(self.f.rec['mode'], 'MANUAL', 'a silent Agent alone does not change the mode record')

    def test_a_persisting_contradiction_ends_the_mode_and_it_does_not_come_back_by_itself(self):
        self.install('MANUAL', watchdog_state='LOCKED'); rec = self.f.rec
        self.assertFalse(self.f._mode_invalidate('r', rec), 'inside the grace period the mode record is kept (but motion is already refused)')
        self.f._mode_mismatch['r'] = (time.monotonic() - self.om.ACTIVE_MISMATCH_GRACE_S - 0.1, rec['revision'])
        self.assertTrue(self.f._mode_invalidate('r', rec)); self.assertEqual(rec['mode'], 'IDLE'); self.assertIn('no longer confirms', rec['reason'])
        self.f.robots['r'] = self.robot('MANUAL', 7)                                          # the Watchdog is fine again and even reports MANUAL rev 7 once more
        self.assertFalse(self.f._mode_invalidate('r', rec)); self.assertEqual(rec['mode'], 'IDLE', 'no automatic return')
        self.assertIsNone(self.f.mode_motion_status('r')['verified'])
        from fleet_server.service import ApiError
        with self.assertRaises(ApiError): self.f.require_mode('r', 'MANUAL', 'mode_not_manual')

    def test_a_watchdog_trip_ends_the_mode_at_once_even_when_the_watchdog_has_re_authorized(self):
        self.install('MANUAL', watchdog_trips=1); rec = self.f.rec                          # FAILSAFE happened (Gate restart, source conflict, ...) and the Watchdog is MOTION_AUTHORIZED again, same mode and revision
        st = self.f.mode_motion_status('r'); self.assertIs(st['verified'], False); self.assertEqual(st['hard'], 'LATCHED'); self.assertIn('tripped', st['blocker'])
        self.assertTrue(self.f._mode_invalidate('r', rec), 'no grace period for a latched trip'); self.assertEqual(rec['mode'], 'IDLE')
        self.f.robots['r'] = self.robot('MANUAL', 7, watchdog_trips=1); self.assertFalse(self.f._mode_invalidate('r', rec)); self.assertEqual(rec['mode'], 'IDLE')
        for bad in (None, 'x'):
            self.install('MANUAL', watchdog_trips=bad); self.assertIs(self.f.mode_motion_status('r')['verified'], False, 'a missing or malformed trip count fails closed')

    def test_an_unresolved_mapping_session_makes_the_ended_mode_a_fault(self):
        self.install('MAPPING', watchdog_mode='IDLE'); rec = self.f.rec; self.f.mapping_open = True
        self.f._mode_invalidate('r', rec); self.f._mode_mismatch['r'] = (time.monotonic() - 10, rec['revision']); self.f._mode_invalidate('r', rec); self.assertEqual(rec['mode'], 'FAULT')

    def test_a_short_blip_that_recovers_inside_the_grace_period_does_not_end_the_mode(self):
        self.install('MANUAL', watchdog_state='LOCKED'); rec = self.f.rec; self.f._mode_invalidate('r', rec)
        self.f.robots['r'] = self.robot('MANUAL', 7); self.f._mode_invalidate('r', rec); self.assertEqual(rec['mode'], 'MANUAL'); self.assertNotIn('r', self.f._mode_mismatch)

    def test_mode_change_is_reported_separately_from_motion_permission(self):
        self.install('MANUAL', watchdog_state='LOCKED'); self.assertEqual(self.f.mode_view('r', advance=False)['mode'], 'MANUAL'); self.assertIs(self.f.mode_motion_status('r')['verified'], False)

    def test_arbiter_agents_and_idle_are_not_affected(self):
        self.install('MANUAL'); self.f.robots['r']['agent'] = {'locked_velocity_arbiter': True, 'boot_id': 'ab'}; self.assertIsNone(self.f.mode_motion_status('r')['verified']); self.f.require_mode('r', 'MANUAL', 'x')
        self.install('MANUAL'); self.f.rec.update(mode='IDLE'); self.assertIsNone(self.f.mode_motion_status('r')['verified'])


if __name__ == '__main__':
    unittest.main()
