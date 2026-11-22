"""Agent-side map save worker against the real server logic (in-process loopback, no ROS, no HTTP, no robot).

The two SLAM calls are replaced by a fake saver that writes real files; everything else (stages, manifest, hashes, upload, retries,
journal, restart handling) is the production code. Failures are injected on the saver and on the transport.
"""
import json
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'server'), str(ROOT / 'agent' / 'limo_robot_agent'), str(ROOT / 'tests')]
from fleet_server.db import DB  # noqa: E402
from fleet_server.map_registry import MapRegistry  # noqa: E402
from fleet_server.service import ApiError, Fleet  # noqa: E402
from limo_robot_agent.core import HttpError  # noqa: E402
from limo_robot_agent.map_save_worker import MapSaveWorker  # noqa: E402
from test_map_save import OP, B1, pgm, yml, sha  # noqa: E402


class FakeSaver:
    free = 0.196

    def __init__(self):
        self.ready_result, self.motor, self.calls = (True, ''), (True, ''), []
        self.raster_ok = self.graph_ok = True
        self.yaml, self.pgm = yml(), pgm()
        self.empty = None
        self.positions = [(0.0, 0.0, 0.0)] * 3
        self.version = [4, 4, 4]
        self.i = 0

    def ready(self):
        return self.ready_result

    def motor_path_ok(self):
        return self.motor

    def sample(self):
        i = min(self.i, 2)
        self.i += 1
        x, y, yaw = self.positions[i]
        return {'t': time.time(), 'map_version': self.version[i], 'known_cells': 900, 'x': x, 'y': y, 'yaw': yaw}

    def save_raster(self, d):
        self.calls.append('raster')
        t0 = time.time()
        if self.raster_ok:
            (d / 'map.yaml').write_bytes(b'' if self.empty == 'yaml' else self.yaml)
            (d / 'map.pgm').write_bytes(self.pgm)
        return {'ok': self.raster_ok, 'started': t0, 'finished': time.time() + 0.05, 'error': '' if self.raster_ok else 'map_saver_cli exited with an error'}

    def save_posegraph(self, d):
        self.calls.append('graph')
        t0 = time.time()
        if self.graph_ok:
            (d / 'graph.posegraph').write_bytes(b'POSEGRAPH' * 40)
            (d / 'graph.data').write_bytes(b'' if self.empty == 'data' else b'DATA' * 30)
        return {'ok': self.graph_ok, 'started': t0 + 0.1, 'finished': time.time() + 0.2, 'result_code': 0 if self.graph_ok else 1,
                'error': '' if self.graph_ok else 'serialize_map returned result 1'}


class Loop:
    """Stands in for core.Http: calls the real Fleet. Hooks inject transport faults per action."""

    def __init__(self, fleet):
        self.fleet = fleet
        self.timeout_first = {}          # action -> number of calls that fail before reaching the server
        self.lose_response = {}          # action -> number of calls that reach the server but whose answer is lost
        self.corrupt_first_bundle = False
        self.calls = []

    def post(self, path, body, timeout=None):
        a = body['action']
        self.calls.append(a)
        if self.timeout_first.get(a, 0) > 0:
            self.timeout_first[a] -= 1
            raise TimeoutError('injected timeout before the server saw it')
        body = dict(body)
        if a == 'bundle' and self.corrupt_first_bundle:
            self.corrupt_first_bundle = False
            body['pgm_b64'] = body['pgm_b64'][:-8] + 'AAAAAAA='
        try:
            res = self.fleet.map_save_agent(body['robot_id'], body['agent_boot_id'], body)
        except ApiError as e:
            raise HttpError(e.status, json.dumps({'error': {'code': e.code, 'message': e.message}}))
        if self.lose_response.get(a, 0) > 0:
            self.lose_response[a] -= 1
            raise TimeoutError('injected: the server handled it but the answer was lost')
        return res


class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.f = Fleet(DB(':memory:'), SimpleNamespace(offline_after_s=6, command_ttl_s=120, resend_after_s=1))
        self.reg = MapRegistry(str(Path(self.tmp.name) / 'r.sqlite'), str(Path(self.tmp.name) / 'store'))
        self.f.map_registry = self.reg
        self.reg.create_facility('fac', '배재대학교', actor='t')
        self.reg.create_building('bld-h', 'fac', 'H관', actor='t')
        self.reg.create_floor('fl-h1', 'bld-h', '1F', actor='t')
        self.reg.create_area('h107', 'fl-h1', 'H107 강의실', actor='t')
        self.f.robots['R'] = {'online': True, 'reconciled': True, 'state_ts': time.time(), 'last_seen': time.time(),
                              'agent': {'boot_id': B1, 'mode': 'real', 'simulated': False, 'map_save': True},
                              'state': {'map_save': {'available': True, 'reason': ''}}}
        self.epoch = self.f.lease_action('R', OP, 'acquire', 300)['epoch']
        self.http = Loop(self.f)
        self.agent = SimpleNamespace(robot_id='R', boot_id=B1, connected=True, http=self.http, log=lambda m: None)
        self.saver = FakeSaver()
        self.state = Path(self.tmp.name) / 'agent'
        self.n = 0

    def tearDown(self):
        self.reg.close()
        self.tmp.cleanup()

    def worker(self, **kw):
        return MapSaveWorker(self.agent, self.state, self.saver, start_thread=False, backoff_s=0.0, **kw)

    def request(self, **over):
        self.n += 1
        self.f.robots['R']['state_ts'] = time.time()
        body = dict(epoch=self.epoch, idempotency_key=f'k{self.n}', floor_id='fl-h1', area_id='h107', label=f'save_{self.n}', note='')
        body.update(over)
        return self.f.map_save_request('R', 'operator', body)['save_id']

    def go(self, w, sid=None, **over):
        sid = sid or self.request(**over)
        self.f.robots['R']['state_ts'] = time.time()
        req = self.http.post('/agent/map-save', {'robot_id': 'R', 'agent_boot_id': B1, 'action': 'poll'})['request']
        self.assertEqual(req['save_id'], sid)
        w.handle(req)
        return sid

    def server(self, sid):
        self.f.robots['R']['state_ts'] = time.time()
        return next(s for s in self.f.map_save_status('R')['saves'] if s['save_id'] == sid)

    def local(self, sid):
        return json.loads((self.state / 'map_saves' / sid / 'status.json').read_text())

    def n_maps(self):
        return self.reg.one('SELECT COUNT(*) AS n FROM map_versions')['n']

    # ------------------------------------------------------------ success
    def test_success_end_to_end_keeps_files_and_matches_hashes(self):
        w = self.worker()
        sid = self.go(w)
        v = self.server(sid)
        self.assertEqual(v['status'], 'REGISTERED')
        loc = self.local(sid)
        self.assertEqual((loc['status'], loc['map_version_id']), ('REGISTERED', v['result']['map_version_id']))
        d = self.state / 'map_saves' / sid
        for name in ('map.yaml', 'map.pgm', 'graph.posegraph', 'graph.data', 'raster.log' if False else 'status.json'):
            self.assertTrue((d / name).is_file(), name)
        files = loc['manifest']['files']
        self.assertEqual(files['pgm']['sha256'], sha((d / 'map.pgm').read_bytes()))
        self.assertEqual(files['data']['size'], (d / 'graph.data').stat().st_size)
        self.assertEqual(self.saver.calls, ['raster', 'graph'])
        self.assertEqual(v['consistency']['same_snapshot'], 'LIKELY_NOT_PROVEN')
        self.assertEqual(self.http.calls.count('bundle'), 1)
        self.assertEqual(w.snapshot()['last']['status'], 'REGISTERED')
        self.assertIsNone(w.snapshot()['active'])

    def test_movement_or_map_change_during_save_is_recorded(self):
        self.saver.positions = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.12, 0.0, 0.0)]
        sid = self.go(self.worker())
        c = self.server(sid)['consistency']
        self.assertEqual(c['same_snapshot'], 'NOT_ESTABLISHED')
        self.assertFalse(c['robot_stationary_during_save'])
        self.assertAlmostEqual(c['robot_moved_m'], 0.12, places=3)
        self.saver.positions, self.saver.i = [(0.0, 0.0, 0.0)] * 3, 0
        self.saver.version = [4, 4, 5]
        self.saver.pgm = pgm(w=30)
        self.assertEqual(self.server(self.go(self.worker()))['consistency']['same_snapshot'], 'NOT_ESTABLISHED')

    # ------------------------------------------------------------ the robot side refuses or fails
    def test_raster_failure_is_reported_and_nothing_registered(self):
        self.saver.raster_ok = False
        sid = self.go(self.worker())
        v = self.server(sid)
        self.assertEqual((v['status'], v['error_code']), ('FAILED', 'raster_failed'))
        self.assertEqual(self.saver.calls, ['raster', 'graph'])              # both are attempted so each result is known
        self.assertIn('raster: map_saver_cli', v['detail'])
        files = v['manifest']['files']
        self.assertEqual(files['pgm'].get('problem'), 'file missing')        # the per-file state reaches the server and the web
        self.assertIsNotNone(files['posegraph']['sha256'])
        self.assertTrue(v['manifest']['results']['posegraph']['ok'])
        self.assertFalse(v['manifest']['results']['raster']['ok'])
        self.assertEqual(self.n_maps(), 0)
        self.assertEqual(self.local(sid)['status'], 'FAILED')

    def test_posegraph_failure_keeps_the_raster_file_locally(self):
        self.saver.graph_ok = False
        sid = self.go(self.worker())
        self.assertEqual(self.server(sid)['error_code'], 'posegraph_failed')
        self.assertTrue((self.state / 'map_saves' / sid / 'map.pgm').is_file())
        self.assertEqual(self.n_maps(), 0)

    def test_empty_saved_file_is_refused(self):
        self.saver.empty = 'data'
        sid = self.go(self.worker())
        self.assertEqual(self.server(sid)['error_code'], 'saved_file_invalid')
        self.assertEqual(self.n_maps(), 0)
        self.assertEqual(self.http.calls.count('bundle'), 0)                 # never uploaded

    def test_motor_path_not_locked_saves_nothing(self):
        self.saver.motor = (False, 'motor path is ARMED (must be LOCKED or SIM_GATED)')
        sid = self.go(self.worker())
        v = self.server(sid)
        self.assertEqual((v['status'], v['error_code']), ('FAILED', 'motor_path_not_locked'))
        self.assertEqual(self.saver.calls, [])

    def test_not_ready_saves_nothing(self):
        self.saver.ready_result = (False, 'no live /map received (SLAM is not running)')
        sid = self.go(self.worker())
        self.assertEqual(self.server(sid)['error_code'], 'agent_not_ready')
        self.assertEqual(self.saver.calls, [])

    def test_busy_worker_is_still_available_so_a_request_can_queue_behind_it(self):
        w = self.worker()
        w.active = 'x' * 32
        s = w.snapshot()
        self.assertTrue(s['available'])
        self.assertEqual(s['active'], 'x' * 32)

    def test_capability_reports_the_reason(self):
        w = self.worker()
        self.assertTrue(w.snapshot()['available'])
        self.saver.ready_result = (False, 'slam_toolbox serialize_map service is not available')
        s = w.snapshot()
        self.assertFalse(s['available'])
        self.assertIn('serialize_map', s['reason'])
        self.saver.ready_result, self.saver.motor = (True, ''), (False, 'motor path is ARMED')
        self.assertIn('ARMED', w.snapshot()['reason'])

    # ------------------------------------------------------------ transport failures and duplicates
    def test_transfer_corruption_is_retried_and_registers_once(self):
        self.http.corrupt_first_bundle = True
        sid = self.go(self.worker())
        self.assertEqual(self.server(sid)['status'], 'REGISTERED')
        self.assertEqual(self.http.calls.count('bundle'), 2)
        self.assertEqual(self.n_maps(), 1)

    def test_upload_timeouts_then_success(self):
        self.http.timeout_first['bundle'] = 2
        sid = self.go(self.worker())
        self.assertEqual(self.server(sid)['status'], 'REGISTERED')
        self.assertEqual(self.http.calls.count('bundle'), 3)
        self.assertEqual(self.n_maps(), 1)

    def test_lost_response_after_registration_does_not_duplicate(self):
        self.http.lose_response['bundle'] = 1
        sid = self.go(self.worker())
        self.assertEqual(self.server(sid)['status'], 'REGISTERED')
        self.assertEqual(self.local(sid)['status'], 'REGISTERED')
        self.assertEqual(self.http.calls.count('bundle'), 2)                 # second call got the idempotent answer
        self.assertEqual(self.n_maps(), 1)

    def test_all_uploads_fail_is_a_visible_failure_and_files_stay(self):
        self.http.timeout_first['bundle'] = 99
        sid = self.go(self.worker())
        v = self.server(sid)
        self.assertEqual((v['status'], v['error_code']), ('FAILED', 'upload_failed'))
        self.assertIn('files kept on the robot', v['detail'])
        self.assertEqual(self.http.calls.count('bundle'), 3)
        self.assertTrue((self.state / 'map_saves' / sid / 'graph.posegraph').is_file())
        self.assertEqual(self.n_maps(), 0)

    def test_registry_rejection_is_final_and_not_retried(self):
        self.saver.yaml = yml(free='0.25')                                   # the unsafe map_saver default
        sid = self.go(self.worker())
        v = self.server(sid)
        self.assertEqual((v['status'], v['error_code']), ('FAILED', 'unknown_pixels_misclassified'))
        self.assertEqual(self.http.calls.count('bundle'), 1)
        self.assertEqual(self.n_maps(), 0)
        self.assertEqual(self.local(sid)['error_code'], 'map_registry_rejected')

    def test_derive_policy_keeps_original_and_derived_yaml_apart(self):
        self.saver.yaml = yml(free='0.25')
        sid = self.go(self.worker(), threshold_policy='derive')
        res = self.server(sid)['result']
        self.assertEqual(res['yaml_adjustment']['free_thresh_to'], 0.196)
        d = self.state / 'map_saves' / sid
        self.assertEqual((d / 'map.yaml').read_bytes(), yml(free='0.25'))    # the file saved on the robot is never rewritten
        ev = self.reg.get_map_version(res['map_version_id'])['verification_evidence']
        self.assertEqual(ev['artifacts_sha256']['map/map.yaml'], sha(yml(free='0.25')))

    def test_progress_stages_are_visible_to_the_server_in_order(self):
        seen = []
        orig = self.f.map_save_agent

        def spy(rid, boot, body):
            if body['action'] == 'stage':
                seen.append(body['stage'])
            return orig(rid, boot, body)
        self.f.map_save_agent = spy
        self.go(self.worker())
        self.assertEqual(seen, ['SAVING_RASTER', 'SAVING_POSEGRAPH', 'UPLOADING'])

    # ------------------------------------------------------------ restart / duplicates
    def test_unfinished_local_save_is_interrupted_after_restart_and_never_replayed(self):
        d = self.state / 'map_saves' / ('a' * 32)
        d.mkdir(parents=True)
        (d / 'status.json').write_text(json.dumps({'save_id': 'a' * 32, 'status': 'SAVING_RASTER', 'stage': 'SAVING_RASTER'}))
        w = self.worker()
        st = json.loads((d / 'status.json').read_text())
        self.assertEqual((st['status'], st['error_code']), ('INTERRUPTED', 'agent_restarted'))
        self.assertEqual(self.saver.calls, [])
        self.assertIsNone(w.active)

    def test_same_save_id_is_never_executed_twice(self):
        w = self.worker()
        sid = self.request()
        (self.state / 'map_saves' / sid).mkdir(parents=True)                 # an earlier attempt of this id began on this robot
        self.f.robots['R']['state_ts'] = time.time()
        req = self.http.post('/agent/map-save', {'robot_id': 'R', 'agent_boot_id': B1, 'action': 'poll'})['request']
        w.handle(req)
        self.assertEqual(self.saver.calls, [])
        v = self.server(sid)
        self.assertEqual((v['status'], v['error_code']), ('FAILED', 'duplicate_attempt'))

    def test_two_requests_run_one_after_another_without_mixing(self):
        w = self.worker()
        a = self.go(w, label='first')
        self.saver.pgm, self.saver.i = pgm(w=32), 0
        b = self.go(w, label='second')
        ra, rb = self.server(a)['result'], self.server(b)['result']
        self.assertNotEqual(ra['map_version_id'], rb['map_version_id'])
        self.assertEqual(self.n_maps(), 2)


class AgentWiringOrder(unittest.TestCase):
    """Found on the real LIMO (2026-11-20): creating the serialize_map client AFTER the MultiThreadedExecutor started made the Agent
    stop receiving /scan, /map and TF (all NO_DATA). The mock/isolated Agent did not show it, so the order is pinned here."""

    def test_ros_saver_is_created_before_the_executor_spins(self):
        import pathlib
        src = (pathlib.Path(__file__).resolve().parents[1] / 'agent/limo_robot_agent/limo_robot_agent/agent_node.py').read_text()
        self.assertIn('ros_saver = RosSaver(robot)', src)
        self.assertLess(src.index('ros_saver = RosSaver(robot)'), src.index('ex = MultiThreadedExecutor'))
        self.assertNotIn('MapSaveWorker(agent, sdir, RosSaver(robot))', src)


if __name__ == '__main__':
    unittest.main()
