"""Codex Phase 4/5 review fixes (2026-11-21): the cases the independent boundary tests do not cover, both directions
(blocked AND still working). No ROS, no robot. Fixtures reused from test_phase4_readiness / test_phase5_guide."""
import sys, threading, time, unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'server'))
import test_phase4_readiness as p4  # noqa: E402
import test_phase5_guide as p5  # noqa: E402
from fleet_server.guide import Visitor  # noqa: E402
from fleet_server.service import ApiError, Fleet  # noqa: E402


class ReadyFix(unittest.TestCase):
    def setUp(self):
        self.t = p4.Readiness('test_ready_allows_mode_request'); self.t.setUp(); self.f = self.t.f; self.r = self.f.robots['R']

    def test_normal_ready_and_motion_permitted_are_separate(self):
        self.t.report(); rd = self.f.readiness('R')
        self.assertEqual(rd['level'], 'READY', rd['reasons']); self.assertTrue(rd['motion_permitted'])
        self.r['state']['safety_gate'] = dict(p4.GATE_OK, movement_permitted=False, movement_block_reasons=['GATE_SOURCE_STALE'])
        rd = self.f.readiness('R'); self.assertEqual(rd['level'], 'READY'); self.assertFalse(rd['motion_permitted'])
        self.assertEqual(rd['motion_block_reasons'], ['GATE_SOURCE_STALE'])

    def test_real_agent_needs_supervisor_even_without_server_flag(self):
        self.f.cfg.require_supervisor_ready = False
        self.r['agent'].update(mode='real', simulated=False)
        rd = self.f.readiness('R'); self.assertEqual(rd['level'], 'FAULT'); self.assertIn('SUPERVISOR_NOT_REPORTING', rd['reasons'])
        with self.assertRaises(ApiError) as e: self.t.mode('NAVIGATION')
        self.assertEqual(e.exception.code, 'robot_not_ready')
        self.r['agent'].update(mode='mock', simulated=True)      # explicit mock follows the server flag (legacy isolated E2E)
        self.f.require_ready('R', 'x')

    def test_supervisor_stale_duplicate_invalid_and_future_timestamps(self):
        self.t.report(); self.f.supervisors['R']['received_at'] = time.time() + 30
        self.assertIn('SUPERVISOR_STALE', self.f.readiness('R')['reasons'])
        self.f.supervisor_report('R', {'boot_id': 's1', 'services': p4.services() + p4.services(), 'ros': p4.ROS_OK})
        self.assertIn('SERVICE_DUPLICATE:stack', self.f.readiness('R')['reasons'])
        sv = p4.services(); sv[0]['state'] = 'BOGUS'
        self.f.supervisor_report('R', {'boot_id': 's1', 'services': sv, 'ros': p4.ROS_OK})
        self.assertEqual(self.f.readiness('R')['level'], 'FAULT')
        for bad in (float('inf'), -1.0, '0.1', True):
            self.t.report(ros={'topics': {'/scan': {'age_s': bad, 'max_age_s': 1.0}, '/odom': {'age_s': 0.1, 'max_age_s': 1.0}}})
            self.assertNotEqual(self.f.readiness('R')['level'], 'READY', bad)
        self.t.report(ros={'topics': {'/scan': {'age_s': 3.0, 'max_age_s': float('nan')}, '/odom': {'age_s': 0.1, 'max_age_s': 1.0}}})
        self.assertIn('STALE:/scan', self.f.readiness('R')['reasons'])          # untrusted limit falls back to 1 s
        self.t.report(); self.r['state_ts'] = float('nan')
        self.assertIn('AGENT_STATE_STALE', self.f.readiness('R')['reasons'])

    def test_gate_and_watchdog_evidence_required(self):
        self.t.report()
        for gate in (None, dict(p4.GATE_OK, cmd_blocked=None), dict(p4.GATE_OK, rx_fresh=False), dict(p4.GATE_OK, age_s=float('nan'))):
            self.r['state']['safety_gate'] = gate
            self.assertNotEqual(self.f.readiness('R')['level'], 'READY', gate)
        self.r['state']['safety_gate'] = dict(p4.GATE_OK, status='STALL_PENDING')          # pending but not blocking: still READY
        self.assertEqual(self.f.readiness('R')['level'], 'READY')
        for wd in (None, {'state': 'MOTION_AUTHORIZED'}, {'state': 'FAILSAFE', 'age_s': 0.1}, {'state': 'MOTION_AUTHORIZED', 'age_s': 9.0}):
            self.r['state']['motion_watchdog'] = wd
            self.assertNotEqual(self.f.readiness('R')['level'], 'READY', wd)


class ServiceActionFix(unittest.TestCase):
    def setUp(self):
        self.t = p4.Readiness('test_ready_allows_mode_request'); self.t.setUp(); self.f = self.t.f; self.r = self.f.robots['R']
        self.t.report(); self.ep = self.t.admin_lease()
        self.f._mode_save('R', dict(self.f._mode_new('IDLE', None, 't'), transition=None, epoch=self.ep, owner='admin', agent_boot='a1'))

    def act(self, **b): return self.f.request_service_action('R', p4.ADMIN, dict({'epoch': self.ep}, **b))

    def test_idle_restart_still_works(self):
        self.assertEqual(self.act(service='watchdog', action='restart')['status'], 'QUEUED')

    def test_lease_epoch_required_and_checked(self):
        for ep in (None, 'x', True, self.ep + 1):
            with self.assertRaises(ApiError, msg=ep): self.f.request_service_action('R', p4.ADMIN, {'service': 'watchdog', 'action': 'restart', 'epoch': ep})
        self.f.lease_action('R', p4.ADMIN, 'release')
        with self.assertRaises(ApiError) as e: self.act(service='watchdog', action='restart')
        self.assertEqual(e.exception.code, 'lease_required')

    def test_active_goal_mapping_manual_and_stale_state_refuse_stop(self):
        cases = [('execution', {'active': {'goal_uuid': 'a' * 32}}), ('execution', {'active': None, 'nav2_goals': {'active_count': 1}}),
                 ('mapping_worker', {'enabled': True, 'status': 'MAPPING'}), ('velocity_arbiter', {'source': 'MANUAL_WEB'})]
        for k, v in cases:
            self.r['state'][k] = v
            with self.assertRaises(ApiError, msg=k) as e: self.act(service='watchdog', action='stop')
            self.assertEqual(e.exception.code, 'robot_busy'); self.r['state'].pop(k)
        self.f.manual_benches['R'] = {'id': 'x'}
        with self.assertRaises(ApiError): self.act(service='agent', action='restart')
        self.f.manual_benches.pop('R')
        self.r['state_ts'] = time.time() - 10
        with self.assertRaises(ApiError) as e: self.act(service='agent', action='restart')
        self.assertEqual(e.exception.code, 'agent_state_stale')

    def test_fault_mode_is_not_idle(self):
        self.f._mode_save('R', dict(self.f._mode_new('FAULT', None, 'x'), transition=None, epoch=self.ep, owner='admin', agent_boot='a1'))
        with self.assertRaises(ApiError) as e: self.act(service='watchdog', action='restart')
        self.assertIn(e.exception.code, ('robot_not_idle', 'robot_busy'))

    def test_start_only_for_stopped_or_failed(self):
        with self.assertRaises(ApiError) as e: self.act(service='watchdog', action='start')
        self.assertEqual(e.exception.code, 'service_not_stopped')
        self.t.report(stack='FAILED'); self.assertEqual(self.act(service='stack', action='retry')['status'], 'QUEUED')   # recovery works


class GuideFix(unittest.TestCase):
    def setUp(self):
        self.t = p5.Guide('test_visitor_links'); self.t.setUp(); self.f = self.t.f
        self.d = self.t.dest(); self.f.set_guide_service(p5.OPER, {'robot_id': 'R', 'enabled': True})

    def tearDown(self): self.t.tearDown()

    def accepted(self): return self.t.db.q("SELECT request_id FROM guide_requests WHERE status IN ('ACCEPTED','IN_PROGRESS','CANCELING')")

    def test_same_visitor_view_cancel_and_cross_link_refused(self):
        a, _ = self.t.visitor('shared_session_123456789'); b, _ = self.t.visitor('shared_session_123456789')
        req = self.f.create_guide_request(a, {'destination_id': self.d['destination_id']}); rid = req['request_id']
        self.assertEqual(self.f.guide_request_view(rid, a)['status'], 'ACCEPTED')
        for fn in (lambda: self.f.guide_request_view(rid, b), lambda: self.f.cancel_guide_request(b, rid)):
            with self.assertRaises(ApiError) as e: fn()
            self.assertEqual(e.exception.status, 404)
        self.assertEqual(self.f.guide_request_view(rid, a)['status'], 'ACCEPTED')         # b's attempt changed nothing
        other_session = Visitor(a.link_id, 'other_session_1234567890')
        with self.assertRaises(ApiError): self.f.cancel_guide_request(other_session, rid)     # same link, other browser
        self.f.create_guide_request(b, {'destination_id': self.d['destination_id']})            # b has its own request (queued)
        self.assertEqual(self.f.cancel_guide_request(a, rid)['status'], 'CANCELING')         # owner cancel still works

    def test_strict_enabled_and_owner_close(self):
        for bad in ('false', 0, None, 'true', 1):
            with self.assertRaises(ApiError, msg=bad): self.f.set_guide_service(p5.OPER, {'robot_id': 'R', 'enabled': bad})
        self.assertTrue(self.f.guide_service_view('R')['enabled'])
        self.assertFalse(self.f.set_guide_service(p5.OPER, {'robot_id': 'R', 'enabled': False})['enabled'])   # owner close works

    def test_duplicate_request_same_visitor(self):
        v, _ = self.t.visitor('dup_session_12345678901')
        self.f.create_guide_request(v, {'destination_id': self.d['destination_id']})
        with self.assertRaises(ApiError) as e: self.f.create_guide_request(v, {'destination_id': self.d['destination_id']})
        self.assertEqual(e.exception.code, 'request_active')

    def test_many_concurrent_requests_one_owner(self):
        vs = [self.t.visitor(f'conc_session_{i:02d}_123456789')[0] for i in range(8)]
        errs, go = [], threading.Barrier(len(vs) + 1)
        def run(v):
            go.wait(5)
            try: self.f.create_guide_request(v, {'destination_id': self.d['destination_id']})
            except Exception as e: errs.append(repr(e))
        th = [threading.Thread(target=run, args=(v,)) for v in vs] + [threading.Thread(target=lambda: (go.wait(5), self.f.guide_tick()))]
        for x in th: x.start()
        for x in th: x.join(10)
        self.assertFalse(errs, errs)
        self.assertEqual(len(self.accepted()), 1)
        self.assertEqual(self.t.db.one("SELECT COUNT(*) n FROM missions WHERE status NOT IN ('SUCCEEDED','CANCELED','FAILED')")['n'], 1)
        self.assertEqual(self.t.db.one("SELECT COUNT(*) n FROM guide_requests WHERE status='QUEUED'")['n'], 7)

    def test_fifo_after_completion_and_restart(self):
        v1, _ = self.t.visitor('fifo_first_123456789012'); v2, _ = self.t.visitor('fifo_second_12345678901')
        r1 = self.f.create_guide_request(v1, {'destination_id': self.d['destination_id']})
        r2 = self.f.create_guide_request(v2, {'destination_id': self.d['destination_id']}); self.assertEqual(r2['status'], 'QUEUED')
        # server restart: a new Fleet on the same DB keeps following r1 and keeps r2 queued; no duplicate mission
        g = Fleet(self.t.db, p5.CFG); g.map_registry = self.t.reg; g.robots = self.f.robots
        g.safety_context = self.f.safety_context; g.dispatch_next = self.f.dispatch_next
        g.guide_tick(); self.assertEqual(g.guide_request_view(r2['request_id'])['status'], 'QUEUED')
        mid = g.guide_request_view(r1['request_id'])['mission_id']
        self.t.db.x("UPDATE missions SET status='SUCCEEDED' WHERE mission_id=?", (mid,))
        g.guide_tick()
        self.assertEqual(g.guide_request_view(r1['request_id'])['status'], 'COMPLETED')
        # the previous server boot's NAVIGATION / map-load records are not trusted after a restart (operating_mode / navigation
        # design): r2 waits, it is not lost or dispatched on stale evidence. The operator re-enters NAVIGATION and reloads the map.
        self.assertEqual(g.guide_request_view(r2['request_id'])['status'], 'QUEUED')
        self.t.f = g; self.t.navigate(self.t.mv)
        g.guide_tick(); self.assertEqual(g.guide_request_view(r2['request_id'])['status'], 'ACCEPTED')
        g.guide_tick(); self.assertEqual(len(self.accepted()), 1)

    def test_stale_received_after_crash_is_queued_then_dispatched(self):
        v, _ = self.t.visitor('crash_session_1234567890')
        now = time.time() - 30
        self.t.db.x("INSERT INTO guide_requests(request_id,destination_id,link_id,session,status,created_at,updated_at,history_json) VALUES(?,?,?,?,?,?,?,?)",
                    ('gr_crashed0001', self.d['destination_id'], v.link_id, v.session, 'RECEIVED', now, now, '[]'))
        self.f.guide_tick(); self.assertEqual(self.f.guide_request_view('gr_crashed0001', v)['status'], 'ACCEPTED')

    def test_mission_creation_failure_releases_robot(self):
        v1, _ = self.t.visitor('fail_first_123456789012'); v2, _ = self.t.visitor('fail_second_12345678901')
        with patch.object(self.f, 'create_mission', side_effect=ApiError(409, 'agent_unavailable', 'x')):
            r1 = self.f.create_guide_request(v1, {'destination_id': self.d['destination_id']})
        self.assertEqual((r1['status'], r1['reason']), ('FAILED', 'agent_unavailable'))
        with patch.object(self.f, 'create_mission', side_effect=RuntimeError('db')):
            r1b = self.f.create_guide_request(v2, {'destination_id': self.d['destination_id']})
        self.assertEqual((r1b['status'], r1b['reason']), ('FAILED', 'MISSION_CREATE_ERROR'))
        v3, _ = self.t.visitor('fail_third_123456789012')
        with patch.object(self.f, 'create_mission', side_effect=ApiError(409, 'safety_not_ready', 'x')):
            r3 = self.f.create_guide_request(v3, {'destination_id': self.d['destination_id']})
        self.assertEqual(r3['status'], 'QUEUED')                                      # transient: waits, robot not reserved
        self.f.guide_tick(); self.assertEqual(self.f.guide_request_view(r3['request_id'], v3)['status'], 'ACCEPTED')

    def test_map_change_invalidates_cache_and_restored_file_recovers(self):
        row = self.t.db.one('SELECT * FROM guide_destinations WHERE destination_id=?', (self.d['destination_id'],))
        self.assertIsNone(self.f.destination_block(row))
        key = self.t.reg.one('SELECT storage_key FROM map_versions WHERE map_version_id=?', (self.d['map_version_id'],))['storage_key']
        path = Path(self.t.reg.bundles_dir) / key / 'map.pgm'; path.chmod(0o600); orig = path.read_bytes()
        self.assertTrue(self.f._nav_raster_cache is not None)
        path.write_bytes(orig[:-1] + bytes([orig[-1] ^ 1]))
        self.assertEqual(self.f.destination_block(row)[0], 'MAP_INTEGRITY_FAILED')
        self.assertTrue(self.f._nav_raster_cache is None)                              # goal-check raster dropped
        path.write_bytes(orig); self.assertIsNone(self.f.destination_block(row))     # restored bytes -> available again
        path.unlink(); self.assertEqual(self.f.destination_block(row)[0], 'MAP_INTEGRITY_FAILED')

    def test_queued_request_not_dispatched_onto_tampered_map(self):
        v1, _ = self.t.visitor('tamper_first_12345678901'); v2, _ = self.t.visitor('tamper_second_1234567890')
        r1 = self.f.create_guide_request(v1, {'destination_id': self.d['destination_id']})
        r2 = self.f.create_guide_request(v2, {'destination_id': self.d['destination_id']}); self.assertEqual(r2['status'], 'QUEUED')
        key = self.t.reg.one('SELECT storage_key FROM map_versions WHERE map_version_id=?', (self.d['map_version_id'],))['storage_key']
        path = Path(self.t.reg.bundles_dir) / key / 'map.pgm'; path.chmod(0o600); b = path.read_bytes()
        path.write_bytes(b[:-1] + bytes([b[-1] ^ 1]))
        self.t.db.x("UPDATE missions SET status='SUCCEEDED' WHERE mission_id=?", (self.f.guide_request_view(r1['request_id'])['mission_id'],))
        self.f.guide_tick()
        v = self.f.guide_request_view(r2['request_id']); self.assertEqual((v['status'], v['reason']), ('QUEUED', 'MAP_INTEGRITY_FAILED'))
        self.assertEqual(self.t.db.one("SELECT COUNT(*) n FROM missions")['n'], 1)

    def test_agent_map_bytes_must_match_registry(self):
        rec = self.f._nav_load('R'); self.f._nav_save('R', dict(rec, pgm_sha256='0' * 64))
        row = self.t.db.one('SELECT * FROM guide_destinations WHERE destination_id=?', (self.d['destination_id'],))
        self.assertEqual(self.f.destination_block(row)[0], 'ROBOT_ON_OTHER_MAP')

    def test_unverified_c103_policy_kept(self):
        d = self.t.dest(mv=self.t.unv, destination_id='d_unverified')
        row = self.t.db.one('SELECT * FROM guide_destinations WHERE destination_id=?', (d['destination_id'],))
        self.assertEqual(self.f.destination_block(row)[0], 'MAP_NOT_VERIFIED')


if __name__ == '__main__':
    unittest.main(verbosity=2)
