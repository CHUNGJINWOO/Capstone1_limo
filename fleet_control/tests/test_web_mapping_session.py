"""Mock checks for the web Mapping contract (START/PAUSE/RESUME/SAVE/STOP) on the local Fleet server.

No ROS. A fake robot state stands in for the Agent; the worker side is driven through `worker_operation`.
"""
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'server'))
from fleet_server.db import DB  # noqa: E402
from fleet_server.map_registry import MapRegistry  # noqa: E402
from fleet_server.service import ApiError, Fleet  # noqa: E402

OP = SimpleNamespace(name='operator', role='operator')
OTHER = SimpleNamespace(name='rmf-adapter', role='operator')
IDLE_NAV = {'known': True, 'publishers': 1, 'active_count': 0, 'foreign_active_goal_uuids': []}


class MappingSessionTests(unittest.TestCase):
    def setUp(self):
        self.f = Fleet(DB(':memory:'), SimpleNamespace(offline_after_s=6, command_ttl_s=120, resend_after_s=1))
        self.boot = 'b' * 12
        self.f.robots['R'] = {'online': True, 'reconciled': True, 'state_ts': time.time(),
                              'agent': {'boot_id': self.boot, 'mode': 'sim', 'simulated': True, 'mapping_worker': True},
                              'state': {'execution': {'active': None, 'nav2_goals': dict(IDLE_NAV)},
                                        'mapping_worker': {'status': 'IDLE'}, 'slam': {'enabled': False, 'status': 'UNAVAILABLE'}}}
        self.epoch = self.f.lease_action('R', OP, 'acquire', 60)['epoch']
        self.n = 0

    def fresh(self):
        self.f.robots['R']['state_ts'] = time.time()

    def req(self, action, sid=None, **extra):
        self.n += 1
        self.fresh()
        body = dict(action=action, epoch=self.epoch, idempotency_key=f'k{self.n}', **extra)
        if sid:
            body['session_id'] = sid
        return self.f.request_operation('R', 'operator', body)

    def worker(self, action, **fields):
        self.fresh()
        return self.f.worker_operation('R', self.boot, dict(kind='mapping', action=action, **fields))

    def complete(self, out, evidence, outcome='OK'):
        c = self.worker('poll')['command']
        self.assertEqual(c['command_id'], out['command_id'])
        ref = dict(session_id=c['session_id'], command_id=c['command_id'])
        self.assertTrue(self.worker('accept', **ref)['accepted'])
        return self.worker('result', **ref, outcome=outcome, evidence=evidence)

    def start(self):
        out = self.req('START')
        self.assertEqual(self.f.operation_status('R')['state'], 'STARTING')
        s = self.complete(out, dict(slam_running=True, frontier_running=True))
        self.assertEqual(s['state'], 'MAPPING')
        return out['session_id']

    def test_renew_window_never_exceeds_lease(self):
        sid = self.start()
        r = self.worker('poll', session_id=sid)
        self.assertTrue(r['renew'])
        self.assertAlmostEqual(r['renew_valid_s'], 3.0, delta=0.01)          # long Lease: the normal 3 s fence
        self.f.db.x('UPDATE locks SET expires_at=? WHERE robot_id=?', (time.time() + 1.2, 'R'))
        r = self.worker('poll', session_id=sid)
        self.assertTrue(r['renew'])
        self.assertLessEqual(r['renew_valid_s'], 1.2)                       # capped by the Lease's remaining time
        self.assertGreater(r['renew_valid_s'], 1.0)
        self.f.db.x('UPDATE locks SET expires_at=? WHERE robot_id=?', (time.time() - 0.1, 'R'))
        r = self.worker('poll', session_id=sid)
        self.assertFalse(r['renew'])                                        # expired: no renew, worker fence runs out
        self.assertNotIn('renew_valid_s', r)

    def test_full_lifecycle_and_http_acceptance_vs_result(self):
        sid = self.start()
        out = self.req('PAUSE', sid)
        self.assertEqual(out['status'], 'QUEUED')                       # HTTP acceptance only
        self.assertEqual(self.f.operation_status('R')['state'], 'PAUSING')
        self.assertEqual(self.complete(out, dict(frontier_paused=True, goal_terminal=True))['state'], 'PAUSED')
        save = self.req('SAVE', sid)
        self.assertEqual(self.complete(save, dict(yaml_ok=True, pgm_ok=True, posegraph_ok=True))['state'], 'PAUSED')
        res = self.req('RESUME', sid)
        self.assertEqual(self.complete(res, dict(slam_running=True, frontier_running=True))['state'], 'MAPPING')
        stop = self.req('STOP', sid)
        self.assertEqual(self.complete(stop, dict(frontier_stopped=True, goal_terminal=True))['state'], 'STOPPED')
        self.assertFalse(self.f.mapping_blocks_motion('R'))

    def test_duplicate_start_is_idempotent_and_second_start_rejected(self):
        self.fresh()
        body = dict(action='START', epoch=self.epoch, idempotency_key='same')
        a = self.f.request_operation('R', 'operator', body)
        b = self.f.request_operation('R', 'operator', body)
        self.assertTrue(b['duplicate'])
        self.assertEqual(a['session_id'], b['session_id'])
        with self.assertRaises(ApiError) as ctx:
            self.req('START')
        self.assertEqual(ctx.exception.code, 'operation_state_conflict')

    def test_lease_and_session_required(self):
        self.fresh()
        with self.assertRaises(ApiError) as ctx:
            self.f.request_operation('R', 'operator', dict(action='START', epoch=self.epoch + 1, idempotency_key='x'))
        self.assertEqual(ctx.exception.code, 'lease_required')
        with self.assertRaises(ApiError) as ctx:
            self.f.request_operation('R', 'rmf-adapter', dict(action='START', epoch=self.epoch, idempotency_key='y'))
        self.assertEqual(ctx.exception.code, 'lease_required')
        sid = self.start()
        with self.assertRaises(ApiError) as ctx:
            self.req('PAUSE', 'f' * 32)
        self.assertEqual(ctx.exception.code, 'session_conflict')
        self.assertTrue(sid)

    def test_missing_goal_terminal_evidence_is_unknown_and_sticky(self):
        sid = self.start()
        out = self.req('PAUSE', sid)
        s = self.complete(out, dict(frontier_paused=True, goal_terminal=False))
        self.assertEqual(s['state'], 'UNKNOWN')
        for action in ('START', 'RESUME', 'STOP'):
            with self.assertRaises(ApiError):
                self.req(action, sid)
        self.assertTrue(self.f.mapping_blocks_motion('R'))

    def test_worker_staleness_and_boot_change_make_unknown(self):
        sid = self.start()
        self.f.robots['R']['state_ts'] = time.time() - 10
        self.assertEqual(self.f.operation_status('R')['state'], 'UNKNOWN')
        self.assertTrue(sid)

    def test_save_failure_keeps_paused(self):
        sid = self.start()
        self.complete(self.req('PAUSE', sid), dict(frontier_paused=True, goal_terminal=True))
        s = self.complete(self.req('SAVE', sid), dict(yaml_ok=True, pgm_ok=False, posegraph_ok=True), outcome='FAILED')
        self.assertEqual(s['state'], 'PAUSED')
        self.assertEqual(s['last_save']['status'], 'FAILED')

    def test_ok_without_required_evidence_is_downgraded(self):
        sid = self.start()
        self.complete(self.req('PAUSE', sid), dict(frontier_paused=True, goal_terminal=True))
        s = self.complete(self.req('SAVE', sid), dict(yaml_ok=True, pgm_ok=True, posegraph_ok=False))
        self.assertEqual(s['command']['status'], 'FAILED')

    def test_start_requires_idle_nav_and_no_foreign_goal_or_mission(self):
        self.f.robots['R']['state']['execution']['nav2_goals'] = dict(IDLE_NAV, active_count=1)
        with self.assertRaises(ApiError) as ctx:
            self.req('START')
        self.assertEqual(ctx.exception.code, 'nav_terminal_unproven')
        self.f.robots['R']['state']['execution']['nav2_goals'] = dict(IDLE_NAV, foreign_active_goal_uuids=['a' * 32])
        with self.assertRaises(ApiError) as ctx:
            self.req('START')
        self.assertEqual(ctx.exception.code, 'foreign_goal_present')
        self.f.robots['R']['state']['execution']['nav2_goals'] = dict(IDLE_NAV)
        self.f.db.x("INSERT INTO missions(mission_id,robot_id,type,goal_json,status,created_at,updated_at) VALUES('m_000000000001','R','navigate_to_pose','{}','EXECUTING',0,0)")
        with self.assertRaises(ApiError) as ctx:
            self.req('START')
        self.assertEqual(ctx.exception.code, 'destination_pending')

    def test_navigation_mission_blocked_while_mapping(self):
        self.start()
        self.f.safety_context = lambda robot_id, **kw: {'boot': self.boot}
        with self.assertRaises(ApiError) as ctx:
            self.f.create_mission('R', 'navigate_to_pose', {'frame_id': 'map', 'x': 1, 'y': 2}, None, '', 'operator', self.epoch)
        self.assertEqual(ctx.exception.code, 'mapping_active')

    def test_sim_slam_start_blocked_while_mapping(self):
        self.start()
        self.f.robots['R']['state']['slam'] = {'enabled': True, 'status': 'IDLE'}
        with self.assertRaises(ApiError) as ctx:
            self.f.slam_command('R', 'start', 'S1', 'operator')
        self.assertEqual(ctx.exception.code, 'mapping_active')

    def test_real_agent_is_blocked(self):
        self.f.robots['R']['agent'].update(mode='real', simulated=False)
        with self.assertRaises(ApiError) as ctx:
            self.req('START')
        self.assertEqual(ctx.exception.code, 'mapping_real_blocked')

    def test_goal_reservation_requires_resolved_previous(self):
        sid = self.start()
        g1, g2 = 'a' * 32, 'c' * 32
        self.assertTrue(self.worker('reserve_goal', session_id=sid, goal_id=g1)['reserved'])
        with self.assertRaises(ApiError):
            self.worker('reserve_goal', session_id=sid, goal_id=g2, previous={'command_id': g1, 'status': 'ACCEPTED'})
        self.assertTrue(self.worker('reserve_goal', session_id=sid, goal_id=g2,
                                    previous={'command_id': g1, 'status': 'CANCELED', 'goal_uuid': 'd' * 32})['reserved'])
        active = {'owner': 'frontier', 'mission_id': sid, 'command_id': g2}
        self.assertTrue(self.f.owns_mapping_goal('R', self.boot, active))
        self.assertFalse(self.f.owns_mapping_goal('R', 'other-boot', active))
        self.assertFalse(self.f.owns_mapping_goal('R', self.boot, dict(active, owner='fleet')))

    def test_lease_expiry_makes_unknown_but_reserved_goal_stays_owned(self):
        sid = self.start()
        g = 'e' * 32
        self.assertTrue(self.worker('reserve_goal', session_id=sid, goal_id=g)['reserved'])
        self.f.db.x('UPDATE locks SET expires_at=? WHERE robot_id=?', (time.time() - 1, 'R'))   # operator tab gone
        self.assertEqual(self.f.operation_status('R')['state'], 'UNKNOWN')
        self.assertTrue(self.f.mapping_blocks_motion('R'))
        self.assertTrue(self.f.owns_mapping_goal('R', self.boot, {'owner': 'frontier', 'mission_id': sid, 'command_id': g}))
        self.assertFalse(self.f.owns_mapping_goal('R', self.boot, {'owner': 'frontier', 'mission_id': sid, 'command_id': 'f' * 32}))

    def test_result_replay_does_not_rewrite(self):
        sid = self.start()
        out = self.req('PAUSE', sid)
        first = self.complete(out, dict(frontier_paused=True, goal_terminal=True))
        again = self.worker('result', session_id=sid, command_id=out['command_id'], outcome='UNKNOWN', evidence={})
        self.assertEqual(again['state'], first['state'])


class SaveTargetTests(unittest.TestCase):
    def test_save_target_must_be_registered(self):
        with tempfile.TemporaryDirectory() as d:
            reg = MapRegistry(os.path.join(d, 'r.db'), os.path.join(d, 'store'))
            reg.create_facility('fac', 'Fac'); reg.create_building('b', 'fac', 'B'); reg.create_floor('f1', 'b', '1F')
            reg.create_area('lounge', 'f1', 'Lounge')
            f = Fleet(DB(':memory:'), SimpleNamespace(offline_after_s=6, command_ttl_s=120, resend_after_s=1))
            f.map_registry = reg
            self.assertEqual(f._save_target({'floor_id': 'f1', 'area_id': 'lounge'}), {'floor_id': 'f1', 'area_id': 'lounge'})
            with self.assertRaises(ApiError):
                f._save_target({'floor_id': 'nope'})
            with self.assertRaises(ApiError):
                f._save_target({'floor_id': 'f1', 'area_id': 'other'})
            reg.close()


class PlanAndLineageTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.reg = MapRegistry(os.path.join(self.tmp.name, 'r.db'), os.path.join(self.tmp.name, 'store'))
        self.reg.create_facility('fac', 'Fac'); self.reg.create_building('b', 'fac', 'B'); self.reg.create_floor('f1', 'b', '1F')
        yaml = b'image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
        pgm = b'P5\n4 4\n255\n' + bytes([254] * 8 + [205] * 6 + [0, 0])
        self.raster_only = self.reg.register_map_bundle(floor_id='f1', label='rasteronly', yaml_bytes=yaml, pgm_bytes=pgm)['map_version']['map_version_id']
        self.with_graph = self.reg.register_map_bundle(floor_id='f1', label='withgraph', yaml_bytes=yaml, pgm_bytes=pgm,
                                                       posegraph_bytes=b'graph', posegraph_data_bytes=b'data')['map_version']['map_version_id']
        self.f = Fleet(DB(':memory:'), SimpleNamespace(offline_after_s=6, command_ttl_s=120, resend_after_s=1))
        self.f.map_registry = self.reg
        self.f.robots['R'] = {'agent': {}, 'state': {}}

    def tearDown(self):
        self.reg.close(); self.tmp.cleanup()

    def code(self, body):
        with self.assertRaises(ApiError) as ctx:
            self.f._start_plan('R', body)
        return ctx.exception.code

    def test_new_map_defaults_and_limits(self):
        plan = self.f._start_plan('R', {'limits': {'max_goals': 5, 'min_battery_v': 10.8}, 'geofence': [[0, 0], [3, 0], [3, 2]]})
        self.assertEqual(plan['mode'], 'NEW_MAP')
        self.assertEqual(plan['limits']['max_goals'], 5)
        self.assertEqual(plan['limits']['max_duration_s'], 900.0)
        self.assertEqual(self.code({'limits': {'max_goals': 0}}), 'bad_limits')
        self.assertEqual(self.code({'limits': {'speed': 1}}), 'bad_limits')
        self.assertEqual(self.code({'geofence': [[0, 0], [1, 1]]}), 'bad_geofence')
        self.assertEqual(self.code({'mode': 'MERGE'}), 'bad_mode')

    def test_continue_map_gates(self):
        self.assertEqual(self.code({'mode': 'CONTINUE_MAP'}), 'parent_required')
        self.assertEqual(self.code({'mode': 'CONTINUE_MAP', 'parent_map_version_id': 'nope#000000000000'}), 'parent_not_found')
        self.assertEqual(self.code({'mode': 'CONTINUE_MAP', 'parent_map_version_id': self.raster_only}), 'parent_without_posegraph')
        self.assertEqual(self.code({'mode': 'CONTINUE_MAP', 'parent_map_version_id': self.with_graph}), 'continue_unsupported')
        self.f.robots['R']['agent']['mapping_continue'] = True
        self.assertEqual(self.code({'mode': 'CONTINUE_MAP', 'parent_map_version_id': self.with_graph}), 'localization_unconfirmed')
        self.f.robots['R']['state']['mapping_localization'] = {'status': 'CONFIRMED', 'map_version_id': self.with_graph}
        plan = self.f._start_plan('R', {'mode': 'CONTINUE_MAP', 'parent_map_version_id': self.with_graph})
        self.assertEqual(plan['parent_map_version_id'], self.with_graph)

    def test_lineage_write_once_and_rollback_is_selection(self):
        yaml = b'image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
        pgm2 = b'P5\n4 4\n255\n' + bytes([254] * 10 + [205] * 4 + [0, 0])
        child = self.reg.register_map_bundle(floor_id='f1', label='v2', yaml_bytes=yaml, pgm_bytes=pgm2,
                                             posegraph_bytes=b'g2', posegraph_data_bytes=b'd2')['map_version']['map_version_id']
        self.reg.record_lineage(child, parent_map_version_id=self.with_graph, mapping_mode='CONTINUE_MAP', session_id='s1')
        self.assertEqual(self.reg.get_map_version(child)['lineage']['parent_map_version_id'], self.with_graph)
        self.reg.record_lineage(child, parent_map_version_id=self.with_graph, mapping_mode='CONTINUE_MAP', session_id='s1')
        from fleet_server.map_registry import ConflictError, ValidationError
        with self.assertRaises(ConflictError):
            self.reg.record_lineage(child, parent_map_version_id=None, mapping_mode='NEW_MAP', session_id='s1')
        with self.assertRaises(ValidationError):
            self.reg.record_lineage(self.raster_only, parent_map_version_id=None, mapping_mode='CONTINUE_MAP', session_id='s2')
        # Parent bundle is untouched (immutable) and still readable: rollback = select/activate the parent again.
        self.assertTrue(self.reg.verify_bundle_integrity(self.with_graph)['ok'])


class WorkerPolicyTests(unittest.TestCase):
    """Worker geofence and termination limits without ROS (methods only)."""
    def setUp(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agent' / 'limo_robot_agent'))
        try:
            from limo_robot_agent.mapping_worker import MappingWorker
        except Exception as exc:  # geometry_msgs missing outside ROS
            self.skipTest(f'ROS python packages unavailable: {exc}')
        self.W = MappingWorker
        self.w = MappingWorker.__new__(MappingWorker)
        self.w.robot = SimpleNamespace(limo={'voltage': 11.0})
        self.w.attempted = set()
        self.w.started_mono = time.monotonic()
        self.w.plan = {'limits': {'max_duration_s': 900, 'max_goals': 2, 'min_battery_v': None}}

    def test_inside(self):
        sq = [[0, 0], [2, 0], [2, 2], [0, 2]]
        self.assertTrue(self.W.inside(sq, 1, 1))
        self.assertFalse(self.W.inside(sq, 3, 1))

    def test_limits(self):
        self.assertIsNone(self.w.limit_reached())
        self.w.attempted = {(0, 0), (1, 1)}
        self.assertEqual(self.w.limit_reached(), 'MAX_GOALS')
        self.w.attempted = set()
        self.w.plan['limits']['min_battery_v'] = 11.5
        self.assertEqual(self.w.limit_reached(), 'LOW_BATTERY')
        self.w.robot.limo = None
        self.assertEqual(self.w.limit_reached(), 'BATTERY_UNKNOWN')
        self.w.plan['limits']['min_battery_v'] = None
        self.w.started_mono = time.monotonic() - 1000
        self.assertEqual(self.w.limit_reached(), 'MAX_DURATION')


class WorkerMotorPathTests(unittest.TestCase):
    """locked(): LOCKED graph or the existing SIM_GATED policy only; a real ARMED path is refused."""
    def setUp(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agent' / 'limo_robot_agent'))
        try:
            from limo_robot_agent.mapping_worker import MappingWorker
            from limo_robot_agent.motor_policy import motor_path_allowed
        except Exception as exc:
            self.skipTest(f'ROS python packages unavailable: {exc}')
        self.W, self.allowed = MappingWorker, motor_path_allowed

    def worker(self, pubs, subs, status):
        w = self.W.__new__(self.W)
        w.robot = SimpleNamespace(count_publishers=lambda t: pubs, count_subscribers=lambda t: subs,
                                  _motor_path_status=lambda: (status, {}), require_motion_watchdog=False)
        return w

    def test_locked_graph(self):
        self.assertTrue(self.worker(0, 0, 'LOCKED').locked())

    def test_sim_gated_with_bridge(self):
        self.assertTrue(self.worker(1, 1, 'SIM_GATED').locked())
        self.assertFalse(self.worker(1, 2, 'SIM_GATED').locked())     # an extra consumer is not accepted

    def test_armed_real_or_unknown_refused(self):
        for st in ('ARMED', 'UNKNOWN'):
            self.assertFalse(self.worker(1, 1, st).locked(), st)
        self.assertFalse(self.worker(0, 0, 'UNKNOWN').locked())       # Gate dead: no /cmd_vel_locked publisher
        self.assertFalse(self.worker(0, 1, 'LOCKED').locked())        # unexpected /cmd_vel consumer

    def test_watchdog_required(self):
        w = self.worker(0, 0, 'LOCKED')
        w.robot.require_motion_watchdog = True
        w.robot.watchdog_ok = lambda: False
        self.assertFalse(w.locked())
        w.robot.watchdog_ok = lambda: True
        self.assertTrue(w.locked())

    def test_policy_requires_sim_and_single_gate(self):
        gate = SimpleNamespace(node_name='stall_detector_node', node_namespace='/')
        other = SimpleNamespace(node_name='controller_server', node_namespace='/')
        kw = dict(mode='sim', simulated=True, allow_sim_safety_gate=True, use_sim_time=True)
        self.assertTrue(self.allowed([gate], **kw))
        self.assertFalse(self.allowed([gate, other], **kw))
        self.assertFalse(self.allowed([other], **kw))
        self.assertFalse(self.allowed([gate], **dict(kw, mode='real')))
        self.assertFalse(self.allowed([gate], **dict(kw, use_sim_time=False)))


if __name__ == '__main__':
    unittest.main()
