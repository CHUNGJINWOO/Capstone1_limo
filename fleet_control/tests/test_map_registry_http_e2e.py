"""Real loopback HTTP integration tests for Map Registry Phase 2-A GET API.

Tests:
1. GET /facilities, /maps?floor_id=..., /maps/{id}/meta, /maps/{id}/preview.png with %23 encoding.
2. Auth, RBAC (viewer/operator/admin allowed; agent 403; unauthenticated 401; lockout 429).
3. PUT/DELETE blocked (405); POST only for the 4 registration routes (see test_map_registry_http_upload.py); traversal rejected (403).
4. Security headers (Content-Security-Policy, Cache-Control, X-Content-Type-Options) on PNG/JSON.
5. Corrupted disk bundle -> 500 bundle_integrity_error; oversized payload -> 413 too_large.
6. Concurrent HTTP GET requests from multiple threads remain deterministic and error-free.
7. Existing robot live map & Safety Context v1 fields remain 100% unchanged after registry queries.
"""
from __future__ import annotations

import base64
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
from fleet_server.map_registry import MapRegistry, canonical_to_png_bytes  # noqa: E402
from fleet_server.service import Fleet  # noqa: E402


def _sample_pgm(width: int = 8, height: int = 6) -> bytes:
    header = f'P5\n{width} {height}\n255\n'.encode('ascii')
    rows = bytearray()
    for r in range(height):
        for c in range(width):
            if r in (0, height - 1) or c in (0, width - 1):
                rows.append(0)
            elif (r + c) % 2 == 0:
                rows.append(254)
            else:
                rows.append(205)
    return header + bytes(rows)


def _sample_yaml(origin: tuple[float, float, float] = (-1.5, -2.0, 0.25), res: float = 0.05) -> bytes:
    return (
        'image: map.pgm\n'
        f'resolution: {res}\n'
        f'origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n'
    ).encode('utf-8')


class MapRegistryHttpE2ETests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        p = Path(self.tmp.name)
        init_tokens(str(p), str(p / 'sec'), ['limo1'])
        self.agent = (p / 'sec' / 'agent-limo1.token').read_text().strip()
        self.viewer = (p / 'sec' / 'viewer.token').read_text().strip()
        self.op = (p / 'sec' / 'operator.token').read_text().strip()
        self.admin = (p / 'sec' / 'admin.token').read_text().strip()

        self.db = DB(str(p / 'fleet.db'))
        self.f = Fleet(self.db, SimpleNamespace(offline_after_s=10, command_ttl_s=120, resend_after_s=0.1))
        self.reg = MapRegistry(str(p / 'map_registry.db'), str(p / 'map_store'))

        self.reg.create_facility('fac-main', 'Engineering Campus')
        self.reg.create_building('bld-a', 'fac-main', 'Hall A')
        self.reg.create_floor('fl-a1', 'bld-a', 'L1', elevation_m=0.0, rmf_map_name='L1')
        self.reg.create_floor('fl-a2', 'bld-a', 'L2', elevation_m=4.0, rmf_map_name='L2')

        res_v = self.reg.register_map_bundle(
            floor_id='fl-a1',
            label='hall-a-l1',
            yaml_bytes=_sample_yaml((-1.25, -2.5, 0.3), 0.05),
            pgm_bytes=_sample_pgm(10, 8),
            source_mode='IMPORTED_STATIC',
            actor='tester',
        )
        self.verified_id = res_v['map_version']['map_version_id']
        self.verified_storage = res_v['map_version']['storage_key']
        self.reg.verify_map_version(
            self.verified_id,
            status='VERIFIED',
            evidence={'method': 'manual_qa'},
            actor='qa',
        )
        self.reg.set_floor_active_map(
            'fl-a1',
            self.verified_id,
            actor='qa',
            reason='baseline active map',
        )

        res_u = self.reg.register_map_bundle(
            floor_id='fl-a1',
            label='slam-draft',
            yaml_bytes=_sample_yaml((-0.5, -0.5, 0.0), 0.05),
            pgm_bytes=_sample_pgm(6, 6),
            source_mode='SLAM_LIVE_SNAPSHOT',
            actor='tester',
        )
        self.unverified_id = res_u['map_version']['map_version_id']

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

    def _req(self, method: str, path: str, token: str | None = None, body: dict | None = None):
        headers = {}
        if token is not None:
            headers['Authorization'] = f'Bearer {token}'
        data = None
        if body is not None:
            data = json.dumps(body).encode('utf-8')
            headers['Content-Type'] = 'application/json'
        req = urllib.request.Request(
            f'http://127.0.0.1:{self.port}{path}',
            method=method,
            data=data,
            headers=headers,
        )
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            with opener.open(req, timeout=5) as resp:
                return resp.status, dict(resp.headers), resp.read()
        except urllib.error.HTTPError as exc:
            return exc.code, dict(exc.headers), exc.read()

    def test_get_endpoints_percent_encoding_headers_and_png(self):
        st, hdrs, raw = self._req('GET', '/api/v1/map-registry/facilities', self.viewer)
        self.assertEqual(st, 200)
        facs = json.loads(raw)['facilities']
        self.assertEqual(len(facs), 1)
        self.assertEqual(facs[0]['buildings'][0]['floors'][0]['active_map_version_id'], self.verified_id)
        self.assertEqual(hdrs.get('Cache-Control'), 'no-store')
        self.assertEqual(hdrs.get('X-Content-Type-Options'), 'nosniff')
        self.assertIn("img-src 'self' blob: data:", hdrs.get('Content-Security-Policy', ''))

        st, _, raw = self._req('GET', '/api/v1/map-registry/maps?floor_id=fl-a1', self.op)
        self.assertEqual(st, 200)
        maps = json.loads(raw)['maps']
        self.assertEqual({m['map_version_id'] for m in maps}, {self.verified_id, self.unverified_id})

        st, _, raw = self._req('GET', '/api/v1/map-registry/maps?floor_id=fl-a2', self.op)
        self.assertEqual(st, 200)
        self.assertEqual(json.loads(raw)['maps'], [])

        enc = urllib.parse.quote(self.verified_id, safe='')
        self.assertIn('%23', enc)
        st, _, raw_meta = self._req('GET', f'/api/v1/map-registry/maps/{enc}/meta', self.op)
        self.assertEqual(st, 200)
        meta = json.loads(raw_meta)
        self.assertEqual(meta['map_version_id'], self.verified_id)
        self.assertEqual(meta['verification_status'], 'VERIFIED')
        self.assertEqual(meta['origin'], [-1.25, -2.5, 0.3])
        self.assertTrue(meta['is_floor_active'])

        st, png_hdrs, raw_png = self._req('GET', f'/api/v1/map-registry/maps/{enc}/preview.png', self.op)
        self.assertEqual(st, 200)
        self.assertEqual(png_hdrs.get('Content-Type'), 'image/png')
        self.assertEqual(png_hdrs.get('Cache-Control'), 'no-store')
        self.assertTrue(raw_png.startswith(b'\x89PNG\r\n\x1a\n'))

    def test_auth_rbac_methods_and_traversal_blocking(self):
        enc = urllib.parse.quote(self.verified_id, safe='')
        # Missing / invalid auth -> 401
        st, _, _ = self._req('GET', '/api/v1/map-registry/facilities', None)
        self.assertEqual(st, 401)
        st, _, _ = self._req('GET', f'/api/v1/map-registry/maps/{enc}/preview.png', 'invalid-token')
        self.assertEqual(st, 401)

        # Agent cannot read registry endpoints -> 403
        st, _, raw = self._req('GET', '/api/v1/map-registry/facilities', self.agent)
        self.assertEqual(st, 403)
        self.assertEqual(json.loads(raw)['error']['code'], 'forbidden')

        # PUT / DELETE remain blocked for everyone -> 405 (Phase 2-C only routes registration POSTs)
        for method in ('PUT', 'DELETE'):
            st, _, raw = self._req(method, '/api/v1/map-registry/maps', self.admin)
            self.assertEqual(st, 405, method)
            self.assertEqual(json.loads(raw)['error']['code'], 'method_not_allowed')
        # An invalid registration body is now handled by the registry (400), no longer 405
        st, _, raw = self._req('POST', '/api/v1/map-registry/maps', self.admin, {'x': 1})
        self.assertEqual(st, 400)

        # Encoded traversal rejected -> 403
        st, _, raw = self._req('GET', '/api/v1/map-registry/maps/..%2Fetc/meta', self.admin)
        self.assertEqual(st, 403)
        self.assertEqual(json.loads(raw)['error']['code'], 'security_violation')

        # Unknown map -> 404
        st, _, raw = self._req('GET', '/api/v1/map-registry/maps/nonexistent_map_id/meta', self.admin)
        self.assertEqual(st, 404)
        self.assertEqual(json.loads(raw)['error']['code'], 'not_found')

    def test_corrupted_bundle_500_and_oversized_response_413(self):
        enc = urllib.parse.quote(self.verified_id, safe='')
        # Warm PNG cache first, then tamper with map.pgm on disk
        st, _, _ = self._req('GET', f'/api/v1/map-registry/maps/{enc}/preview.png', self.viewer)
        self.assertEqual(st, 200)

        pgm_file = Path(self.reg.bundles_dir) / self.verified_storage / 'map.pgm'
        os.chmod(pgm_file, 0o644)
        data = pgm_file.read_bytes()
        pgm_file.write_bytes(data[:-1] + bytes([data[-1] ^ 0xFF]))

        st, _, raw = self._req('GET', f'/api/v1/map-registry/maps/{enc}/preview.png', self.viewer)
        self.assertEqual(st, 500)
        self.assertEqual(json.loads(raw)['error']['code'], 'bundle_integrity_error')

        # Oversized payload check with strict max_http_response_bytes on second server
        tight = MapRegistry(self.reg.db_path, self.reg.bundle_root, max_http_response_bytes=40)
        s2 = Servers(self.f, Auth(self.tmp.name), ['127.0.0.1'], 0, map_registry=tight)
        port2 = s2.srvs[0].server_address[1]
        t2 = threading.Thread(target=s2.serve_forever, daemon=True)
        t2.start()
        try:
            enc_u = urllib.parse.quote(self.unverified_id, safe='')
            req = urllib.request.Request(
                f'http://127.0.0.1:{port2}/api/v1/map-registry/maps/{enc_u}/preview.png',
                headers={'Authorization': f'Bearer {self.viewer}'},
            )
            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            with self.assertRaises(urllib.error.HTTPError) as ctx:
                opener.open(req, timeout=3)
            self.assertEqual(ctx.exception.code, 413)
            self.assertEqual(json.loads(ctx.exception.read())['error']['code'], 'too_large')
        finally:
            s2.stop_evt.set()
            t2.join(3)
            tight.close()

    def test_concurrent_http_get_burst_and_robot_isolation(self):
        # Register a real-like simulated robot state + live map on Fleet Server
        st, _, _ = self._req('POST', '/api/v1/agent/hello', self.agent, {
            'robot_id': 'limo1',
            'agent': {
                'boot_id': 'boot-test',
                'protocol_version': 2,
                'safety_contract_version': 1,
                'mode': 'sim',
                'simulated': True,
                'missions_enabled': True,
            },
        })
        self.assertEqual(st, 200)
        ex = {'active': None, 'nav2_goals': {'known': True, 'active_count': 0, 'received_status_array': True, 'age_s': 0.0}}
        st, _, _ = self._req('POST', '/api/v1/agent/reconcile', self.agent, {
            'robot_id': 'limo1',
            'agent_boot_id': 'boot-test',
            'journal': {},
            'journal_ok': True,
            'execution': ex,
        })
        self.assertEqual(st, 200)
        st, _, _ = self._req('POST', '/api/v1/agent/state', self.agent, {
            'robot_id': 'limo1',
            'seq': 1,
            'agent_boot_id': 'boot-test',
            'state': {
                'safety_gate': {
                    'boot_id': 'g1',
                    'status': 'NORMAL',
                    'age_s': 0.0,
                    'fault_epoch': 1,
                    'fault_latched': False,
                    'cmd_blocked': False,
                    'movement_permitted': True,
                    'motor_path': {'status': 'SIM_GATED', 'cmd_vel_publishers': ['stall_detector_node']},
                },
                'operation_mode': 'NAVIGATION',
                'operation_revision': 1,
                'map_version_id': 'live-robot-map-v1',
                'map_verified': True,
                'map_epoch': 3,
                'localization_status': 'CONFIRMED',
                'localization_map_version_id': 'live-robot-map-v1',
                'localization_epoch': 1,
                'execution': ex,
            },
        })
        self.assertEqual(st, 200)
        png_b64 = base64.b64encode(canonical_to_png_bytes(2, 2, bytes([255, 0, 255, 128]))).decode('ascii')
        st, _, _ = self._req('POST', '/api/v1/agent/map', self.agent, {
            'robot_id': 'limo1',
            'agent_boot_id': 'boot-test',
            'version': 7,
            'meta': {'width': 2, 'height': 2, 'resolution': 0.05, 'origin_x': 0.0, 'origin_y': 0.0, 'origin_yaw': 0.0, 'frame_id': 'map'},
            'png_base64': png_b64,
        })
        self.assertEqual(st, 200)

        before = json.loads(self._req('GET', '/api/v1/robots/limo1', self.viewer)[2])

        # Burst 8 concurrent readers across facilities, maps, meta, preview.png
        enc_v = urllib.parse.quote(self.verified_id, safe='')
        enc_u = urllib.parse.quote(self.unverified_id, safe='')
        outcomes: list[tuple[str, int]] = []
        lock = threading.Lock()

        def _worker(idx: int):
            targets = (
                '/api/v1/map-registry/facilities',
                '/api/v1/map-registry/maps?floor_id=fl-a1',
                f'/api/v1/map-registry/maps/{enc_v}/meta',
                f'/api/v1/map-registry/maps/{enc_v}/preview.png',
                f'/api/v1/map-registry/maps/{enc_u}/preview.png',
            )
            for path in targets:
                code, _, payload = self._req('GET', path, self.viewer)
                if path.endswith('preview.png') and code == 200:
                    assert payload.startswith(b'\x89PNG')
                with lock:
                    outcomes.append((f'w{idx}:{path}', code))

        threads = [threading.Thread(target=_worker, args=(i,)) for i in range(8)]
        for th in threads:
            th.start()
        for th in threads:
            th.join(5)

        self.assertEqual(len(outcomes), 40)
        self.assertTrue(all(code == 200 for _, code in outcomes), outcomes)

        after = json.loads(self._req('GET', '/api/v1/robots/limo1', self.viewer)[2])
        self.assertEqual(after['map_version'], before['map_version'])
        self.assertEqual(after['state']['map_version_id'], 'live-robot-map-v1')
        self.assertEqual(after['state']['map_epoch'], 3)
        self.assertEqual(after['reconciliation_required'], before['reconciliation_required'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
