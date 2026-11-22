"""Mock checks for the web map save (save the live SLAM map + register UNVERIFIED) on the local Fleet server.

No ROS, no robot: a fake robot state stands in for the Agent and the Agent side is driven through `map_save_agent`.
Covers the request rules, the stage machine, per-file verification, the threshold safety check (original vs derived yaml),
idempotency, timeouts, Agent restarts, late uploads, registry failures and that existing registry entries are never changed.
"""
import base64
import hashlib
import json
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
B1 = 'b' * 12


def pgm(unknown=True, w=24, h=20):
    px = bytearray(254 for _ in range(w * h))
    for i in range(w):
        px[i] = 0
        px[(h - 1) * w + i] = 0
    for i in range(h):
        px[i * w] = 0
        px[i * w + w - 1] = 0
    if unknown:
        for r in range(6, 12):
            for c in range(8, 16):
                px[r * w + c] = 205
    return f'P5\n{w} {h}\n255\n'.encode() + bytes(px)


def yml(free='0.196', origin='[-1.0, -1.0, 0.0]'):
    return (f'image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: {origin}\nnegate: 0\n'
            f'occupied_thresh: 0.65\nfree_thresh: {free}\n').encode()


def sha(b):
    return hashlib.sha256(b).hexdigest()


def b64(b):
    return base64.b64encode(b).decode()


def bundle(yaml_b=None, pgm_b=None, t0=1000.0, before=None, after=None, moved=0.0, rot=0.0, corrupt=None):
    y, g = yaml_b or yml(), pgm_b or pgm()
    pg, data = b'POSEGRAPH' * 30, b'DATA' * 20
    blobs = {'yaml': y, 'pgm': g, 'posegraph': pg, 'data': data}
    files = {}
    for k, v in blobs.items():
        win = (t0, t0 + 0.8) if k in ('yaml', 'pgm') else (t0 + 1.0, t0 + 2.5)
        files[k] = {'name': k, 'size': len(v), 'sha256': sha(v), 'saved_start': win[0], 'saved_end': win[1]}
    mani = {'files': files, 'saver': {'free_thresh_requested': 0.196, 'serialize_result_code': 0},
            'evidence': {'map_before': before if before is not None else {'version': 4, 'known_cells': 900},
                         'map_after': after if after is not None else {'version': 4, 'known_cells': 900},
                         'moved_m': moved, 'rotated_rad': rot, 'pose_samples': 3}}
    if corrupt:
        blobs = dict(blobs, **{corrupt: blobs[corrupt] + b'X'})
    body = {'action': 'bundle', 'manifest': mani}
    for k, v in blobs.items():
        body[k + '_b64'] = b64(v)
    return body


class MapSaveTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.f = Fleet(DB(':memory:'), SimpleNamespace(offline_after_s=6, command_ttl_s=120, resend_after_s=1))
        self.reg = MapRegistry(str(Path(self.tmp.name) / 'r.sqlite'), str(Path(self.tmp.name) / 'store'))
        self.f.map_registry = self.reg
        self.reg.create_facility('fac', '배재대학교', actor='t')
        self.reg.create_building('bld-h', 'fac', 'H관', actor='t')
        self.reg.create_floor('fl-h1', 'bld-h', '1F', elevation_m=0.0, actor='t')
        self.reg.create_floor('fl-h2', 'bld-h', '2F', elevation_m=3.0, actor='t')
        self.reg.create_area('h107', 'fl-h1', 'H107 강의실', coverage='PARTIAL_FLOOR', actor='t')
        self.reg.create_area('h201', 'fl-h2', 'H201', coverage='PARTIAL_FLOOR', actor='t')
        self.f.robots['R'] = {'online': True, 'reconciled': True, 'state_ts': time.time(), 'last_seen': time.time(),
                              'agent': {'boot_id': B1, 'mode': 'real', 'simulated': False, 'map_save': True},
                              'state': {'map_save': {'available': True, 'reason': ''}}}
        self.epoch = self.f.lease_action('R', OP, 'acquire', 120)['epoch']
        self.n = 0

    def tearDown(self):
        self.reg.close()
        self.tmp.cleanup()

    # ------------------------------------------------------------ helpers
    def fresh(self):
        self.f.robots['R']['state_ts'] = time.time()
        self.f.robots['R']['last_seen'] = time.time()

    def req(self, actor=OP, **over):
        self.n += 1
        self.fresh()
        body = dict(epoch=self.epoch, idempotency_key=f'k{self.n}', floor_id='fl-h1', area_id='h107', label=f'h107_v{self.n}', note='test')
        body.update(over)
        return self.f.map_save_request('R', actor.name, body)

    def agent(self, **body):
        self.fresh()
        return self.f.map_save_agent('R', body.pop('boot', B1), body)

    def run_to_upload(self, save_id):
        self.assertEqual(self.agent(action='poll')['request']['save_id'], save_id)
        self.assertTrue(self.agent(action='accept', save_id=save_id)['accepted'])
        self.agent(action='stage', save_id=save_id, stage='SAVING_RASTER')
        self.agent(action='stage', save_id=save_id, stage='SAVING_POSEGRAPH')
        self.agent(action='stage', save_id=save_id, stage='UPLOADING')

    def status(self, sid=None):
        s = self.f.map_save_status('R')['saves']
        return s[0] if sid is None else next(x for x in s if x['save_id'] == sid)

    def n_maps(self):
        return self.reg.one('SELECT COUNT(*) AS n FROM map_versions')['n']

    def code(self, fn, *a, **k):
        with self.assertRaises(ApiError) as cm:
            fn(*a, **k)
        return cm.exception.status, cm.exception.code

    # ------------------------------------------------------------ request rules
    def test_request_validation_and_lease(self):
        self.assertEqual(self.code(self.req, actor=OTHER), (409, 'lease_required'))
        self.assertEqual(self.code(self.req, epoch=self.epoch + 5), (409, 'lease_required'))
        self.assertEqual(self.code(self.req, idempotency_key='bad key!'), (400, 'bad_request'))
        self.assertEqual(self.code(self.req, label='no spaces allowed'), (400, 'bad_label'))
        self.assertEqual(self.code(self.req, note='x' * 501), (400, 'bad_note'))
        self.assertEqual(self.code(self.req, threshold_policy='ignore'), (400, 'bad_policy'))
        self.assertEqual(self.code(self.req, floor_id='nope')[0], 400)
        self.assertEqual(self.code(self.req, area_id='h201')[0], 400)                   # area on another floor
        self.assertEqual(self.f.db.one('SELECT COUNT(*) AS n FROM map_save_requests')['n'], 0)

    def test_capability_and_exclusivity(self):
        r = self.f.robots['R']
        r['agent']['map_save'] = False
        self.assertEqual(self.code(self.req), (409, 'map_save_unavailable'))
        r['agent']['map_save'] = True
        r['state']['map_save'] = {'available': False, 'reason': 'no live /map received (SLAM is not running)'}
        st, code = self.code(self.req)
        self.assertEqual((st, code), (409, 'map_save_unavailable'))
        r['state']['map_save'] = {'available': True, 'reason': ''}
        r['online'] = False
        self.assertEqual(self.code(self.req), (409, 'map_save_unavailable'))
        r['online'] = True
        self.f.db.x('INSERT INTO operation_sessions(robot_id,kind,body) VALUES(?,?,?)', ('R', 'mapping', json.dumps(
            {'robot_id': 'R', 'kind': 'mapping', 'state': 'MAPPING', 'session_id': 's', 'boot': B1, 'server_boot': self.f.boot_id, 'owner': 'operator',
             'epoch': self.epoch, 'command': {'status': 'COMPLETED', 'expires_at': time.time() + 99, 'command_id': 'c', 'action': 'START'}, 'result': None, 'detail': ''})))
        self.assertEqual(self.code(self.req), (409, 'mapping_active'))
        self.f.db.x('DELETE FROM operation_sessions')
        first = self.req()
        self.assertEqual(first['status'], 'QUEUED')
        self.assertEqual(self.code(self.req), (409, 'save_in_progress'))

    def test_idempotency(self):
        a = self.req(idempotency_key='same', label='one')
        b = self.req(idempotency_key='same', label='one')
        self.assertTrue(b['duplicate'])
        self.assertEqual(a['save_id'], b['save_id'])
        self.assertEqual(self.code(self.req, idempotency_key='same', label='two'), (409, 'idempotency_conflict'))
        self.assertEqual(self.code(self.req, idempotency_key='same', label='one', floor_id='fl-h2', area_id='h201'), (409, 'idempotency_conflict'))

    # ------------------------------------------------------------ the happy path and what it records
    def test_full_save_registers_unverified_with_evidence(self):
        prior = self.reg.register_map_bundle(floor_id='fl-h1', label='older', yaml_bytes=yml(), pgm_bytes=pgm(w=30), actor='t', actor_role='operator')
        prior_id = prior['map_version']['map_version_id']
        prior_row = json.dumps(self.reg.get_map_version(prior_id), sort_keys=True, default=str)
        sid = self.req(label='h107_stationary')['save_id']
        self.assertEqual(self.status()['status'], 'QUEUED')
        self.run_to_upload(sid)
        self.assertEqual(self.status()['status'], 'UPLOADING')
        res = self.agent(**bundle(), save_id=sid)
        self.assertEqual(self.status()['status'], 'REGISTERED')
        mv = self.reg.get_map_version(res['map_version_id'])
        self.assertEqual(mv['verification_status'], 'UNVERIFIED')
        self.assertEqual(mv['area']['area_id'], 'h107')
        self.assertEqual(mv['lineage']['mapping_mode'], 'NEW_MAP')
        self.assertFalse(mv['is_floor_active'])
        ev = mv['verification_evidence']
        self.assertEqual((ev['method'], ev['save_id'], ev['robot_id'], ev['operator']), ('web_map_save', sid, 'R', 'operator'))
        self.assertEqual(set(ev['saved_files']), {'map/map.yaml', 'map/map.pgm', 'map/posegraph.posegraph', 'map/posegraph.data'})
        self.assertEqual(ev['saved_files']['map/posegraph.data']['sha256'], sha(b'DATA' * 20))
        self.assertEqual(ev['location']['area'], 'H107 강의실')
        self.assertFalse(ev['location']['auto_detected'])
        self.assertIn('known_area_m2', ev['grid_counts_pgm'])
        self.assertTrue(ev['agent_manifest_verified'])
        c = ev['snapshot_consistency']
        self.assertEqual(c['same_snapshot'], 'LIKELY_NOT_PROVEN')
        self.assertAlmostEqual(c['gap_between_raster_end_and_posegraph_start_s'], 0.2, places=3)
        self.assertIn('not one atomic snapshot', c['basis'])
        view = self.status()
        self.assertEqual(view['result']['map_version_id'], res['map_version_id'])
        self.assertEqual(view['consistency']['same_snapshot'], 'LIKELY_NOT_PROVEN')
        self.assertIsNone(self.agent(action='poll')['request'])
        # the existing entry was not touched, and nothing became the floor's active map
        self.assertEqual(json.dumps(self.reg.get_map_version(prior_id), sort_keys=True, default=str), prior_row)
        self.assertIsNone(self.reg.one('SELECT * FROM floor_active_maps WHERE floor_id=?', ('fl-h1',)))

    def test_consistency_never_overclaims(self):
        cases = [
            (dict(moved=0.2), 'NOT_ESTABLISHED'),
            (dict(rot=0.3), 'NOT_ESTABLISHED'),
            (dict(after={'version': 5, 'known_cells': 910}), 'NOT_ESTABLISHED'),
            (dict(moved=None, rot=None), 'NOT_ESTABLISHED'),                   # pose unavailable
            (dict(), 'LIKELY_NOT_PROVEN'),
        ]
        for i, (kw, expect) in enumerate(cases):
            sid = self.req(label=f'c{i}')['save_id']
            self.run_to_upload(sid)
            b = bundle(pgm_b=pgm(w=24 + i), **kw)
            self.agent(**b, save_id=sid)
            self.assertEqual(self.status(sid)['consistency']['same_snapshot'], expect, kw)

    # ------------------------------------------------------------ transfer, policy and registry failures
    def test_corrupt_transfer_is_rejected_then_retry_succeeds(self):
        sid = self.req()['save_id']
        self.run_to_upload(sid)
        before = self.n_maps()
        for bad in ('yaml', 'pgm', 'posegraph', 'data'):
            self.assertEqual(self.code(self.agent, **bundle(corrupt=bad), save_id=sid), (422, 'transfer_corrupt'))
        self.assertEqual(self.n_maps(), before)
        self.assertEqual(self.status()['status'], 'UPLOADING')                  # still retryable
        self.agent(**bundle(), save_id=sid)
        self.assertEqual(self.status()['status'], 'REGISTERED')
        self.assertEqual(self.n_maps(), before + 1)

    def test_manifest_must_describe_all_four_files(self):
        sid = self.req()['save_id']
        self.run_to_upload(sid)
        b = bundle()
        del b['manifest']['files']['data']
        self.assertEqual(self.code(self.agent, **b, save_id=sid), (400, 'bad_manifest'))
        b = bundle()
        del b['data_b64']
        self.assertEqual(self.code(self.agent, **b, save_id=sid), (400, 'invalid_map'))

    def test_unsafe_threshold_rejected_by_default_and_derived_on_request(self):
        sid = self.req(label='unsafe')['save_id']
        self.run_to_upload(sid)
        before = self.n_maps()
        y = yml(free='0.25')
        self.assertEqual(self.code(self.agent, **bundle(yaml_b=y), save_id=sid)[0], 422)
        v = self.status(sid)
        self.assertEqual((v['status'], v['error_code']), ('FAILED', 'unknown_pixels_misclassified'))
        self.assertEqual(self.n_maps(), before)
        # a derived copy only on request; the original hash is kept in the evidence
        sid2 = self.req(label='derived', threshold_policy='derive')['save_id']
        self.run_to_upload(sid2)
        res = self.agent(**bundle(yaml_b=y), save_id=sid2)
        self.assertEqual((res['yaml_adjustment']['free_thresh_from'], res['yaml_adjustment']['free_thresh_to']), (0.25, 0.196))
        ev = self.reg.get_map_version(res['map_version_id'])['verification_evidence']
        self.assertEqual(ev['artifacts_sha256']['map/map.yaml'], sha(y))
        self.assertEqual(ev['saved_files']['map/map.yaml']['sha256'], sha(y))
        self.assertNotEqual(ev['yaml_adjustment']['derived_yaml_sha256'], sha(y))

    def test_registry_conflict_is_a_visible_failure(self):
        sid = self.req(label='same_label')['save_id']
        self.run_to_upload(sid)
        self.agent(**bundle(), save_id=sid)
        sid2 = self.req(label='same_label')['save_id']
        self.run_to_upload(sid2)
        st, code = self.code(self.agent, **bundle(yaml_b=yml(origin='[-2.0, -2.0, 0.0]')), save_id=sid2)
        self.assertEqual(st, 422)
        v = self.status(sid2)
        self.assertEqual(v['status'], 'FAILED')
        self.assertTrue(v['error_code'])
        self.assertEqual(self.n_maps(), 1)

    # ------------------------------------------------------------ timeouts, restarts, late and repeated messages
    def test_queued_too_long_times_out_and_late_accept_is_refused(self):
        sid = self.req()['save_id']
        self.f.db.x('UPDATE map_save_requests SET created_at=created_at-100, expires_at=expires_at-100 WHERE save_id=?', (sid,))
        v = self.status(sid)
        self.assertEqual((v['status'], v['error_code']), ('TIMEOUT', 'timeout'))
        self.assertEqual(self.code(self.agent, action='accept', save_id=sid), (409, 'save_not_active'))
        self.assertIsNone(self.agent(action='poll')['request'])

    def test_deadline_after_accept_then_late_upload_refused(self):
        sid = self.req()['save_id']
        self.run_to_upload(sid)
        self.f.db.x('UPDATE map_save_requests SET expires_at=? WHERE save_id=?', (time.time() - 1, sid))
        self.assertEqual(self.status(sid)['status'], 'TIMEOUT')
        before = self.n_maps()
        self.assertEqual(self.code(self.agent, **bundle(), save_id=sid), (409, 'save_not_active'))
        self.assertEqual(self.n_maps(), before)
        sid2 = self.req()['save_id']                                          # a new request may start afterwards
        self.assertNotEqual(sid, sid2)

    def test_agent_restart_mid_save_is_unknown_and_not_resumed(self):
        sid = self.req()['save_id']
        self.run_to_upload(sid)
        self.f.robots['R']['agent'] = dict(self.f.robots['R']['agent'], boot_id='c' * 12)       # the Agent restarted
        v = self.status(sid)
        self.assertEqual((v['status'], v['error_code']), ('UNKNOWN', 'agent_restarted'))
        self.assertEqual(self.code(self.agent, **bundle(), save_id=sid, boot='c' * 12)[0], 409)  # the new session cannot finish an old save
        self.assertEqual(self.code(self.agent, **bundle(), save_id=sid)[0], 409)
        self.assertEqual(self.n_maps(), 0)

    def test_robot_offline_mid_save_is_unknown(self):
        sid = self.req()['save_id']
        self.run_to_upload(sid)
        self.f.robots['R']['online'] = False
        v = self.status(sid)
        self.assertEqual((v['status'], v['error_code']), ('UNKNOWN', 'robot_offline'))

    def test_agent_failure_report_and_replays(self):
        sid = self.req()['save_id']
        self.agent(action='poll')
        self.agent(action='accept', save_id=sid)
        self.assertTrue(self.agent(action='accept', save_id=sid)['duplicate'])
        self.agent(action='fail', save_id=sid, error_code='raster_failed', message='map_saver_cli exited with an error')
        v = self.status(sid)
        self.assertEqual((v['status'], v['error_code']), ('FAILED', 'raster_failed'))
        self.assertEqual(self.code(self.agent, **bundle(), save_id=sid), (409, 'save_not_active'))
        self.assertEqual(self.agent(action='stage', save_id=sid, stage='UPLOADING')['status'], 'FAILED')      # stages never revive a failed save

    def test_repeated_bundle_is_idempotent_and_different_files_conflict(self):
        sid = self.req()['save_id']
        self.run_to_upload(sid)
        a = self.agent(**bundle(), save_id=sid)
        b = self.agent(**bundle(), save_id=sid)
        self.assertTrue(b['duplicate'])
        self.assertEqual(a['map_version_id'], b['map_version_id'])
        self.assertEqual(self.n_maps(), 1)
        self.assertEqual(self.code(self.agent, **bundle(pgm_b=pgm(w=40)), save_id=sid), (409, 'save_conflict'))

    def test_stage_order_is_monotonic_and_owner_checked(self):
        sid = self.req()['save_id']
        self.agent(action='poll')
        self.agent(action='accept', save_id=sid)
        self.agent(action='stage', save_id=sid, stage='SAVING_POSEGRAPH')
        self.assertEqual(self.agent(action='stage', save_id=sid, stage='SAVING_RASTER')['status'], 'SAVING_POSEGRAPH')   # no going back
        self.assertEqual(self.code(self.agent, action='stage', save_id=sid, stage='NOPE'), (400, 'bad_stage'))
        self.assertEqual(self.code(self.agent, action='stage', save_id='f' * 32)[0], 404)

    def test_poll_only_returns_this_robots_queued_request(self):
        sid = self.req()['save_id']
        got = self.agent(action='poll')['request']
        self.assertEqual((got['save_id'], got['target']['floor_id'], got['target']['threshold_policy']), (sid, 'fl-h1', 'reject'))
        self.agent(action='accept', save_id=sid)
        self.assertIsNone(self.agent(action='poll')['request'])


if __name__ == '__main__':
    unittest.main()
