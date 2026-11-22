"""Phase 2-C: real loopback HTTP tests for the Map Registry registration routes exposed on the Fleet Server.

Exposed (POST): /facilities /buildings /floors /maps  -- registration only, always UNVERIFIED.
NOT exposed (must stay 404): verify / revoke / quarantine / quarantine-clear / floor active-map.
Also checks that robot map / epoch / missions / Safety Context fields are untouched by uploads.
"""
from __future__ import annotations

import base64
import hashlib
import json
import os
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'fleet_control' / 'server'))

from fleet_server.auth import Auth, init_tokens  # noqa: E402
from fleet_server.db import DB  # noqa: E402
from fleet_server.http_api import Servers  # noqa: E402
from fleet_server.map_registry import MAX_YAML_BYTES, MapRegistry, canonical_to_png_bytes  # noqa: E402
from fleet_server.service import Fleet  # noqa: E402


def pgm(width: int = 12, height: int = 10, seed: int = 0) -> bytes:
    header = f'P5\n{width} {height}\n255\n'.encode('ascii')
    px = bytearray()
    for r in range(height):
        for c in range(width):
            if r in (0, height - 1) or c in (0, width - 1):
                px.append(0)
            elif (r * 7 + c * 3 + seed) % 5 == 0:
                px.append(205)
            else:
                px.append(254)
    return header + bytes(px)


def yml(origin=(-0.5, -0.5, 0.0), res=0.05, frame: str | None = None) -> bytes:
    s = f'image: map.pgm\nresolution: {res}\norigin: [{origin[0]}, {origin[1]}, {origin[2]}]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
    if frame:
        s += f'frame_id: {frame}\n'
    return s.encode()


def b64(b: bytes) -> str:
    return base64.b64encode(b).decode('ascii')


def tree_digest(root: Path) -> dict[str, str]:
    out = {}
    for p in sorted(root.rglob('*')):
        if p.is_file():
            out[str(p.relative_to(root))] = hashlib.sha256(p.read_bytes()).hexdigest()
    return out


class RegistryUploadHttpTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        p = Path(self.tmp.name)
        self.p = p
        init_tokens(str(p), str(p / 'sec'), ['limo1'])
        rd = lambda n: (p / 'sec' / f'{n}.token').read_text().strip()
        self.agent, self.viewer, self.op, self.admin = rd('agent-limo1'), rd('viewer'), rd('operator'), rd('admin')
        self.db = DB(str(p / 'fleet.db'))
        self.f = Fleet(self.db, SimpleNamespace(offline_after_s=10, command_ttl_s=120, resend_after_s=0.1))
        self.reg = MapRegistry(str(p / 'map_registry.db'), str(p / 'map_store'))
        self.reg.create_facility('fac-main', 'Campus')
        self.reg.create_building('bld-a', 'fac-main', 'Hall A')
        self.reg.create_floor('fl-a1', 'bld-a', 'L1', elevation_m=0.0, rmf_map_name='L1')
        # pre-existing VERIFIED + active map that uploads must never disturb
        res = self.reg.register_map_bundle(
            floor_id='fl-a1', label='baseline', yaml_bytes=yml(), pgm_bytes=pgm(seed=9),
            verification_status='VERIFIED', verification_evidence={'method': 'manual_qa'}, actor='qa')
        self.base_id = res['map_version']['map_version_id']
        self.reg.set_floor_active_map('fl-a1', self.base_id, actor='qa', reason='baseline')
        self.s = Servers(self.f, Auth(str(p)), ['127.0.0.1'], 0, map_registry=self.reg)
        self.port = self.s.srvs[0].server_address[1]
        self.t = threading.Thread(target=self.s.serve_forever, daemon=True)
        self.t.start()

    def tearDown(self):
        self.s.stop_evt.set()
        self.t.join(3)
        self.reg.close()
        self.db.c.close()
        self.tmp.cleanup()

    def req(self, method, path, token=None, body=None, raw=None, headers=None):
        h = dict(headers or {})
        if token is not None:
            h['Authorization'] = f'Bearer {token}'
        data = raw
        if body is not None:
            data = json.dumps(body).encode()
        if data is not None:
            h['Content-Type'] = 'application/json'
        rq = urllib.request.Request(f'http://127.0.0.1:{self.port}{path}', method=method, data=data, headers=h)
        op = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            with op.open(rq, timeout=20) as r:
                return r.status, json.loads(r.read() or b'{}')
        except urllib.error.HTTPError as e:
            raw_b = e.read()
            try:
                return e.code, json.loads(raw_b or b'{}')
            except ValueError:
                return e.code, {}

    def upload(self, label='lab_v1', y=None, g=None, token=None, idem=None, extra=None, floor='fl-a1'):
        body = {'floor_id': floor, 'label': label, 'source_mode': 'IMPORTED_STATIC',
                'yaml_b64': b64(y if y is not None else yml()), 'pgm_b64': b64(g if g is not None else pgm())}
        body.update(extra or {})
        return self.req('POST', '/api/v1/map-registry/maps', token or self.op, body,
                        headers={'Idempotency-Key': idem} if idem else None)

    def read_robot(self):
        return (self.req('GET', '/api/v1/robots/limo1', self.viewer)[1], self.req('GET', '/api/v1/missions', self.viewer)[1])

    def robot_snapshot(self):
        self.req('POST', '/api/v1/agent/hello', self.agent, {'robot_id': 'limo1', 'agent': {
            'boot_id': 'boot-test', 'protocol_version': 2, 'safety_contract_version': 1, 'mode': 'sim', 'simulated': True, 'missions_enabled': True}})
        ex = {'active': None, 'nav2_goals': {'known': True, 'active_count': 0, 'received_status_array': True, 'age_s': 0.0}}
        self.req('POST', '/api/v1/agent/reconcile', self.agent, {'robot_id': 'limo1', 'agent_boot_id': 'boot-test', 'journal': {}, 'journal_ok': True, 'execution': ex})
        self.req('POST', '/api/v1/agent/state', self.agent, {'robot_id': 'limo1', 'seq': 1, 'agent_boot_id': 'boot-test', 'state': {
            'safety_gate': {'boot_id': 'g1', 'status': 'NORMAL', 'age_s': 0.0, 'fault_epoch': 1, 'fault_latched': False, 'cmd_blocked': False,
                            'movement_permitted': True, 'motor_path': {'status': 'SIM_GATED', 'cmd_vel_publishers': ['stall_detector_node']}},
            'operation_mode': 'NAVIGATION', 'operation_revision': 1, 'map_version_id': 'live-robot-map-v1', 'map_verified': True, 'map_epoch': 3,
            'localization_status': 'CONFIRMED', 'localization_map_version_id': 'live-robot-map-v1', 'localization_epoch': 1, 'execution': ex}})
        self.req('POST', '/api/v1/agent/map', self.agent, {'robot_id': 'limo1', 'agent_boot_id': 'boot-test', 'version': 7,
            'meta': {'width': 2, 'height': 2, 'resolution': 0.05, 'origin_x': 0.0, 'origin_y': 0.0, 'origin_yaw': 0.0, 'frame_id': 'map'},
            'png_base64': b64(canonical_to_png_bytes(2, 2, bytes([255, 0, 255, 128])))})
        return (self.req('GET', '/api/v1/robots/limo1', self.viewer)[1], self.req('GET', '/api/v1/missions', self.viewer)[1])

    # ------------------------------------------------------------------ RBAC / auth
    def test_rbac_401_403(self):
        body = {'floor_id': 'fl-a1', 'label': 'x', 'yaml_b64': b64(yml()), 'pgm_b64': b64(pgm())}
        self.assertEqual(self.req('POST', '/api/v1/map-registry/maps', None, body)[0], 401)
        self.assertEqual(self.req('POST', '/api/v1/map-registry/maps', 'bad-token', body)[0], 401)
        for who in (self.viewer, self.agent):
            for route in ('facilities', 'buildings', 'floors', 'maps'):
                st, b = self.req('POST', f'/api/v1/map-registry/{route}', who, body)
                self.assertEqual(st, 403, (route, who))
                self.assertEqual(b['error']['code'], 'forbidden')
        self.assertEqual(self.reg.list_map_versions(floor_id='fl-a1').__len__(), 1)   # nothing registered
        for method in ('PUT', 'DELETE'):
            self.assertEqual(self.req(method, '/api/v1/map-registry/maps', self.admin)[0], 405)
        # operator and admin may register
        self.assertEqual(self.upload('op_ok', token=self.op)[0], 201)
        self.assertEqual(self.upload('adm_ok', g=pgm(seed=3), token=self.admin)[0], 201)

    # ------------------------------------------------------------------ non-exposed routes
    def test_verify_revoke_quarantine_activemap_not_routed(self):
        st, b = self.upload('draft')
        self.assertEqual(st, 201)
        mvid = b['map_version']['map_version_id']
        enc = urllib.parse.quote(mvid, safe='')
        for tail in ('verify', 'revoke', 'quarantine', 'quarantine/clear'):
            st, _ = self.req('POST', f'/api/v1/map-registry/maps/{enc}/{tail}', self.admin,
                             {'status': 'VERIFIED', 'evidence': {'method': 'x'}, 'reason': 'try'})
            self.assertEqual(st, 404, tail)
        for path in ('/api/v1/map-registry/floors/fl-a1/active-map', '/api/v1/map-registry/robots/limo1/set-map',
                     '/api/v1/map-registry/slam/start', '/api/v1/map-registry/initialpose'):
            self.assertEqual(self.req('POST', path, self.admin, {})[0], 404, path)
        # still UNVERIFIED, baseline still active
        self.assertEqual(self.reg.get_map_version(mvid)['verification_status'], 'UNVERIFIED')
        self.assertEqual(self.reg.get_floor_active_map('fl-a1')['active']['active_map_version_id'], self.base_id)

    # ------------------------------------------------------------------ happy path + audit + preservation
    def test_upload_unverified_preserves_existing_and_audits(self):
        before_tree = tree_digest(self.p / 'map_store' / 'bundles')
        robot_before, missions_before = self.robot_snapshot()
        st, b = self.upload('lab_v1', idem='idem-lab-1', extra={'frame_id': 'map'})
        self.assertEqual(st, 201, b)
        mv = b['map_version']
        self.assertEqual(mv['verification_status'], 'UNVERIFIED')
        self.assertFalse(mv['is_floor_active'])
        self.assertTrue(b['created'])
        self.assertEqual(mv['frame_id'], 'map')
        self.assertEqual(len(mv['bundle_sha256']), 64)
        # existing registry maps byte-for-byte preserved; new bundle is additive
        after_tree = tree_digest(self.p / 'map_store' / 'bundles')
        for k, v in before_tree.items():
            self.assertEqual(after_tree.get(k), v, k)
        self.assertGreater(len(after_tree), len(before_tree))
        base = self.reg.get_map_version(self.base_id)
        self.assertEqual(base['verification_status'], 'VERIFIED')
        self.assertEqual(self.reg.get_floor_active_map('fl-a1')['active']['active_map_version_id'], self.base_id)
        self.assertEqual(self.reg.q('SELECT * FROM robot_map_bindings'), [])
        # robot map / epoch / missions / safety context untouched
        robot_after, missions_after = self.read_robot()
        self.assertEqual(robot_after['state']['map_version_id'], 'live-robot-map-v1')
        self.assertEqual(robot_after['state']['map_epoch'], 3)
        self.assertEqual(robot_after['map_version'], robot_before['map_version'])
        self.assertEqual(robot_after['state']['safety_gate'], robot_before['state']['safety_gate'])
        self.assertEqual(missions_after, missions_before)
        # audit trail: actor recorded, no raw payload leaked
        st, ev = self.req('GET', '/api/v1/map-registry/events?limit=50', self.viewer)
        regs = [e for e in ev['events'] if e['event_type'] == 'MAP_VERSION_REGISTERED' and e['entity_id'] == mv['map_version_id']]
        self.assertEqual(len(regs), 1)
        self.assertEqual(regs[0]['actor'], 'operator')
        self.assertNotIn('yaml_b64', json.dumps(regs[0]))
        enc = urllib.parse.quote(mv['map_version_id'], safe='')
        st, h = self.req('GET', f'/api/v1/map-registry/maps/{enc}/history', self.viewer)
        self.assertEqual(st, 200)
        self.assertEqual([(x['from_status'], x['to_status'], x['actor']) for x in h['history']], [('NONE', 'UNVERIFIED', 'operator')])

    def test_hierarchy_routes_idempotent(self):
        st, b = self.req('POST', '/api/v1/map-registry/facilities', self.op, {'facility_id': 'fac-2', 'name': 'Second'})
        self.assertEqual(st, 201)
        st, _ = self.req('POST', '/api/v1/map-registry/facilities', self.op, {'facility_id': 'fac-2', 'name': 'Second'})
        self.assertEqual(st, 201)                      # same attributes: safe re-create
        st, b = self.req('POST', '/api/v1/map-registry/facilities', self.op, {'facility_id': 'fac-2', 'name': 'Different'})
        self.assertEqual(st, 409)
        self.assertEqual(self.req('POST', '/api/v1/map-registry/buildings', self.op, {'building_id': 'bld-b', 'facility_id': 'fac-2', 'name': 'Hall B'})[0], 201)
        self.assertEqual(self.req('POST', '/api/v1/map-registry/floors', self.op, {'floor_id': 'fl-b1', 'building_id': 'bld-b', 'level_name': 'L1', 'elevation_m': 0})[0], 201)
        self.assertEqual(self.req('POST', '/api/v1/map-registry/buildings', self.op, {'building_id': 'bld-x', 'facility_id': 'nope', 'name': 'X'})[0], 404)

    # ------------------------------------------------------------------ duplicate / conflict / geometry
    def test_duplicate_idempotency_and_geometry_policy(self):
        st1, b1 = self.upload('dup_map', idem='k-1')
        self.assertEqual(st1, 201)
        # same files again, no key: registry recognises the identical bundle (200, idempotent, nothing new)
        n = len(self.reg.list_map_versions(floor_id='fl-a1'))
        st2, b2 = self.upload('dup_map')
        self.assertEqual(st2, 200)
        self.assertTrue(b2['idempotent'])
        self.assertEqual(len(self.reg.list_map_versions(floor_id='fl-a1')), n)
        # replay with same key -> stored response
        st3, b3 = self.upload('dup_map', idem='k-1')
        self.assertEqual(st3, 201)
        self.assertTrue(b3['idempotent_replay'])
        # same key, different payload -> 409
        self.assertEqual(self.upload('dup_map_other', idem='k-1')[0], 409)
        # same label + same pixels, different geometry -> 409 (no silent overwrite)
        st4, b4 = self.upload('dup_map', y=yml(res=0.10))
        self.assertEqual(st4, 409)
        # same pixels, different geometry, different label -> new immutable version, same content hash, different bundle hash
        st5, b5 = self.upload('dup_map_rot', y=yml(origin=(-1.0, 0.5, 0.3), res=0.025, frame='robot1/map'))
        self.assertEqual(st5, 201)
        a, c = b1['map_version'], b5['map_version']
        self.assertEqual(a['sha256']['content_sha256'], c['sha256']['content_sha256'])
        self.assertNotEqual(a['bundle_sha256'], c['bundle_sha256'])
        self.assertEqual(c['frame_id'], 'robot1/map')
        self.assertTrue(c['rotated_map_frame'])

    # ------------------------------------------------------------------ corrupt inputs / checksum / 413
    def test_corrupt_inputs_checksum_and_size_limits(self):
        files_before = tree_digest(self.p / 'map_store' / 'bundles')
        count_before = len(self.reg.list_map_versions())
        bad_cases = {
            'yaml_garbage': dict(y=b'\x00\x01 not yaml'),
            'yaml_wrong_image': dict(y=b'image: other.pgm\nresolution: 0.05\norigin: [0,0,0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.2\n'),
            'yaml_traversal': dict(y=b'image: ../../etc/passwd\nresolution: 0.05\norigin: [0,0,0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.2\n'),
            'pgm_not_pgm': dict(g=b'NOTAPGM' * 20),
            'pgm_truncated': dict(g=pgm()[:-30]),
        }
        for name, kw in bad_cases.items():
            st, b = self.upload(name, **kw)
            self.assertIn(st, (400, 403), (name, st, b))
        # checksum mismatch -> 422
        st, b = self.upload('ck_bad', extra={'expected_checksums': {'pgm_sha256': '0' * 64}})
        self.assertEqual(st, 422)
        self.assertEqual(b['error']['code'], 'checksum_mismatch')
        # correct client checksums are accepted
        y, g = yml(), pgm(seed=4)
        st, b = self.upload('ck_ok', y=y, g=g, extra={'expected_checksums': {
            'yaml_sha256': hashlib.sha256(y).hexdigest(), 'pgm_sha256': hashlib.sha256(g).hexdigest()}})
        self.assertEqual(st, 201)
        # bad base64 -> 400 ; verification_status bypass -> 422
        st, _ = self.req('POST', '/api/v1/map-registry/maps', self.op, {'floor_id': 'fl-a1', 'label': 'b64', 'yaml_b64': '%%%', 'pgm_b64': b64(pgm())})
        self.assertEqual(st, 400)
        self.assertEqual(self.upload('bypass', extra={'verification_status': 'VERIFIED'})[0], 422)
        # per-field decoded limit -> 413 (checked before decoding)
        st, b = self.upload('big_yaml', y=b'a' * (MAX_YAML_BYTES + 10))
        self.assertEqual(st, 413)
        self.assertEqual(b['error']['code'], 'too_large')
        # whole-body limit enforced from Content-Length before reading -> 413 and the server stays healthy
        self.reg.max_http_write_bytes = 4096
        st, b = self.req('POST', '/api/v1/map-registry/maps', self.op, raw=b'{"pad":"' + b'x' * 9000 + b'"}')
        self.assertEqual(st, 413)
        self.assertEqual(self.req('GET', '/api/v1/health')[0], 200)
        # failures left no new rows/files beyond the one valid upload ('ck_ok')
        self.assertEqual(len(self.reg.list_map_versions()), count_before + 1)
        after = tree_digest(self.p / 'map_store' / 'bundles')
        for k, v in files_before.items():
            self.assertEqual(after.get(k), v)
        self.assertEqual(os.listdir(self.p / 'map_store' / '.staging'), [])

    # ------------------------------------------------------------------ concurrency
    def test_concurrent_uploads(self):
        same_y, same_g = yml(origin=(-2.0, -2.0, 0.0)), pgm(seed=21)
        out: list[tuple[str, int, str]] = []
        lock = threading.Lock()
        gate = threading.Barrier(10)

        def same(i):
            gate.wait()
            st, b = self.upload('conc_same', y=same_y, g=same_g, idem='conc-key' if i % 2 else None)
            with lock:
                out.append(('same', st, (b.get('map_version') or {}).get('map_version_id', '')))

        def diff(i):
            gate.wait()
            st, b = self.upload(f'conc_diff_{i}', g=pgm(seed=100 + i))
            with lock:
                out.append(('diff', st, (b.get('map_version') or {}).get('map_version_id', '')))

        ts = [threading.Thread(target=same, args=(i,)) for i in range(6)] + [threading.Thread(target=diff, args=(i,)) for i in range(4)]
        for t in ts:
            t.start()
        for t in ts:
            t.join(60)
        self.assertEqual(len(out), 10)
        self.assertTrue(all(st in (200, 201) for _, st, _ in out), out)
        self.assertEqual(len({m for k, _, m in out if k == 'same'}), 1)
        self.assertEqual(len({m for k, _, m in out if k == 'diff'}), 4)
        labels = [m['label'] for m in self.reg.list_map_versions(floor_id='fl-a1')]
        self.assertEqual(sorted(labels).count('conc_same'), 1)
        self.assertEqual(os.listdir(self.p / 'map_store' / '.staging'), [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
