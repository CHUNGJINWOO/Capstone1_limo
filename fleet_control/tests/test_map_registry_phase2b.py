"""Phase 2-B unit & fault-injection tests for Map Registry registration, verification, revocation, quarantine, and write boundary."""

from __future__ import annotations

import base64
import hashlib
import json
import os
import signal
import sqlite3
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SERVER_DIR = REPO_ROOT / 'fleet_control' / 'server'

sys.path.insert(0, str(SERVER_DIR))

from fleet_server.auth import Principal  # noqa: E402
from fleet_server.map_registry import (  # noqa: E402
    ConflictError,
    IntegrityFailureError,
    MAX_YAML_BYTES,
    MapRegistry,
    PolicyError,
    SecurityError,
    ValidationError,
    parse_pgm_p5,
)


def _make_pgm_bytes(width: int = 16, height: int = 16, seed: int = 0) -> bytes:
    header = f'P5\n{width} {height}\n255\n'.encode('ascii')
    pixels = bytearray()
    for r in range(height):
        for c in range(width):
            if r < 2 or r >= height - 2 or c < 2 or c >= width - 2:
                pixels.append(0)
            elif (r + c + seed) % 13 == 0 and r == 7:
                pixels.append(205)
            else:
                pixels.append(254)
    return header + bytes(pixels)


def _make_yaml_bytes(
    *,
    image: str = 'map.pgm',
    resolution: float = 0.05,
    origin: str = '[-0.4, -0.4, 0.0]',
    frame_id: str | None = None,
    extra: str = '',
) -> bytes:
    lines = [
        f'image: {image}',
        'mode: trinary',
        f'resolution: {resolution}',
        f'origin: {origin}',
        'negate: 0',
        'occupied_thresh: 0.65',
        'free_thresh: 0.196',
    ]
    if frame_id is not None:
        lines.append(f'frame_id: {frame_id}')
    if extra:
        lines.append(extra)
    return ('\n'.join(lines) + '\n').encode('utf-8')


def _b64(data: bytes) -> str:
    return base64.b64encode(data).decode('ascii')


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


class MapRegistryPhase2BTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.db_path = os.path.join(self.tmp.name, 'registry.sqlite')
        self.bundle_root = os.path.join(self.tmp.name, 'map_store')
        self.reg = MapRegistry(self.db_path, self.bundle_root)
        self.viewer = Principal('viewer1', 'viewer')
        self.operator = Principal('op1', 'operator')
        self.admin = Principal('admin1', 'admin')
        self.agent = Principal('agent-limo_1', 'agent', 'limo_1')

        self.reg.create_facility('fac-main', 'Capstone Campus', actor='op1')
        self.reg.create_building('bldg-eng', 'fac-main', 'Engineering Hall', actor='op1')
        self.reg.create_floor('fl-1', 'bldg-eng', 'L1', elevation_m=0.0, rmf_map_name='L1', actor='op1')

        self.sample_pgm = _make_pgm_bytes(16, 16, seed=1)
        self.sample_yaml = _make_yaml_bytes()

    def tearDown(self):
        self.reg.close()
        self.tmp.cleanup()

    def _register_via_api(
        self,
        *,
        label: str = 'lab_map',
        yaml_bytes: bytes | None = None,
        pgm_bytes: bytes | None = None,
        source_mode: str = 'IMPORTED_STATIC',
        frame_id: str | None = None,
        idempotency_key: str | None = None,
        extra_payload: dict | None = None,
        principal: Principal | None = None,
        headers: dict[str, str] | None = None,
    ):
        yb = yaml_bytes if yaml_bytes is not None else self.sample_yaml
        pb = pgm_bytes if pgm_bytes is not None else self.sample_pgm
        payload = {
            'floor_id': 'fl-1',
            'label': label,
            'source_mode': source_mode,
            'yaml_b64': _b64(yb),
            'pgm_b64': _b64(pb),
        }
        if frame_id is not None:
            payload['frame_id'] = frame_id
        if idempotency_key is not None:
            payload['idempotency_key'] = idempotency_key
        if extra_payload:
            payload.update(extra_payload)
        return self.reg.dispatch_write(
            'POST',
            '/api/v1/map-registry/maps',
            principal if principal is not None else self.operator,
            body=json.dumps(payload).encode('utf-8'),
            headers=headers,
        )

    # ------------------------------------------------------------------
    # 1. Identity separation (`content_sha256` vs `bundle_sha256`) & frame_id
    # ------------------------------------------------------------------

    def test_pixel_content_vs_bundle_identity_separation_and_frame_id(self):
        # Register base map with default frame_id ('map') and resolution 0.05
        st1, b1, _ = self._register_via_api(
            label='corridor_res05',
            yaml_bytes=_make_yaml_bytes(resolution=0.05, origin='[-1.0, -1.0, 0.0]', frame_id='map'),
            pgm_bytes=self.sample_pgm,
        )
        self.assertEqual(st1, 201)
        mv1 = b1['map_version']
        self.assertEqual(mv1['verification_status'], 'UNVERIFIED')
        self.assertEqual(mv1['frame_id'], 'map')

        # Register identical pixels with different resolution, origin_yaw, and frame_id under a distinct version label
        st2, b2, _ = self._register_via_api(
            label='corridor_res025_rot',
            yaml_bytes=_make_yaml_bytes(resolution=0.025, origin='[-0.5, 0.25, 0.35]', frame_id='robot1/map'),
            pgm_bytes=self.sample_pgm,
        )
        self.assertEqual(st2, 201)
        mv2 = b2['map_version']
        self.assertEqual(mv2['frame_id'], 'robot1/map')
        self.assertTrue(mv2['rotated_map_frame'])

        # Same canonical pixels -> identical content_sha256 (compatible with map_identity.py),
        # while bundle_sha256 distinguishes resolution/origin/frame_id/YAML changes.
        self.assertEqual(mv1['sha256']['content_sha256'], mv2['sha256']['content_sha256'])
        self.assertNotEqual(mv1['bundle_sha256'], mv2['bundle_sha256'])
        self.assertEqual(mv1['bundle_sha256'], mv1['sha256']['bundle_sha256'])

        # Lookup helpers find both by content_sha256 and each uniquely by bundle_sha256
        by_pixels = self.reg.find_maps_by_content_sha256(mv1['sha256']['content_sha256'])
        self.assertEqual({m['map_version_id'] for m in by_pixels}, {mv1['map_version_id'], mv2['map_version_id']})
        self.assertEqual(self.reg.get_map_by_bundle_sha256(mv2['bundle_sha256'])['map_version_id'], mv2['map_version_id'])

        # Same label + same pixels + differing geometry still raises 409 ConflictError
        st_conflict, b_conflict, _ = self._register_via_api(
            label='corridor_res05',
            yaml_bytes=_make_yaml_bytes(resolution=0.10, origin='[-1.0, -1.0, 0.0]'),
            pgm_bytes=self.sample_pgm,
        )
        self.assertEqual(st_conflict, 409)
        self.assertEqual(b_conflict['error']['code'], 'conflict')

        # Frame_id mismatch between argument and map.yaml rejects with 400
        st_fid_mismatch, _, _ = self._register_via_api(
            label='corridor_fid_mismatch',
            yaml_bytes=_make_yaml_bytes(frame_id='map'),
            frame_id='odom',
        )
        self.assertEqual(st_fid_mismatch, 400)

        # Path-traversal frame_id rejected with 403
        st_bad_fid, _, _ = self._register_via_api(
            label='corridor_bad_fid',
            yaml_bytes=_make_yaml_bytes(frame_id='../evil'),
        )
        self.assertEqual(st_bad_fid, 403)

    def test_additive_schema_migration_backfills_legacy_db_safely(self):
        legacy_db = os.path.join(self.tmp.name, 'legacy.sqlite')
        legacy_store = os.path.join(self.tmp.name, 'legacy_store')
        conn = sqlite3.connect(legacy_db)
        conn.executescript(
            """
            CREATE TABLE facilities (facility_id TEXT PRIMARY KEY, name TEXT NOT NULL, timezone TEXT NOT NULL, description TEXT NOT NULL, created_at REAL NOT NULL, updated_at REAL NOT NULL);
            CREATE TABLE buildings (building_id TEXT PRIMARY KEY, facility_id TEXT NOT NULL, name TEXT NOT NULL, description TEXT NOT NULL, created_at REAL NOT NULL, updated_at REAL NOT NULL);
            CREATE TABLE floors (floor_id TEXT PRIMARY KEY, building_id TEXT NOT NULL, level_name TEXT NOT NULL, elevation_m REAL NOT NULL, rmf_map_name TEXT NOT NULL, created_at REAL NOT NULL, updated_at REAL NOT NULL);
            CREATE TABLE map_versions (
                map_version_id TEXT PRIMARY KEY, floor_id TEXT NOT NULL, label TEXT NOT NULL, source_mode TEXT NOT NULL,
                verification_status TEXT NOT NULL, verification_evidence_json TEXT NOT NULL, storage_key TEXT NOT NULL UNIQUE,
                width INTEGER NOT NULL, height INTEGER NOT NULL, resolution REAL NOT NULL, origin_x REAL NOT NULL,
                origin_y REAL NOT NULL, origin_yaw REAL NOT NULL, negate INTEGER NOT NULL, occupied_thresh REAL NOT NULL,
                free_thresh REAL NOT NULL, content_sha256 TEXT NOT NULL, yaml_sha256 TEXT NOT NULL, pgm_sha256 TEXT NOT NULL,
                posegraph_sha256 TEXT, posegraph_data_sha256 TEXT, rotated_map_frame INTEGER NOT NULL,
                rmf_compatibility_json TEXT NOT NULL, created_by TEXT NOT NULL, created_at REAL NOT NULL,
                verified_by TEXT, verified_at REAL
            );
            INSERT INTO facilities VALUES('fac-1','Fac','UTC','',1.0,1.0);
            INSERT INTO buildings VALUES('b-1','fac-1','Bld','',1.0,1.0);
            INSERT INTO floors VALUES('f-1','b-1','L1',0.0,'L1',1.0,1.0);
            """
        )
        pm = parse_pgm_p5(self.sample_pgm)
        ysha = _sha(self.sample_yaml)
        mvid = f"legacy_v1#{pm['content_sha256'][:12]}"
        conn.execute(
            'INSERT INTO map_versions VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)',
            (
                mvid, 'f-1', 'legacy_v1', 'IMPORTED_STATIC', 'VERIFIED', '{"method":"legacy"}',
                f"legacy_v1__{pm['content_sha256'][:12]}__00000000", 16, 16, 0.05, -0.4, -0.4, 0.0,
                0, 0.65, 0.25, pm['content_sha256'], ysha, pm['pgm_sha256'], None, None, 0,
                '{"checked":false}', 'legacy_op', 1.0, 'legacy_op', 1.0,
            ),
        )
        conn.commit()
        conn.close()

        migrated = MapRegistry(legacy_db, legacy_store)
        try:
            mv = migrated.get_map_version(mvid)
            self.assertEqual(mv['frame_id'], 'map')
            self.assertEqual(len(mv['bundle_sha256']), 64)
            # Immutability trigger blocks editing backfilled bundle_sha256 or frame_id
            with self.assertRaises(sqlite3.DatabaseError):
                migrated.x('UPDATE map_versions SET bundle_sha256=? WHERE map_version_id=?', ('bad', mvid))
            with self.assertRaises(sqlite3.DatabaseError):
                migrated.x('UPDATE map_versions SET frame_id=? WHERE map_version_id=?', ('other', mvid))
        finally:
            migrated.close()

    # ------------------------------------------------------------------
    # 2. Bundle upload validation, checksum mismatch, size bounds, security
    # ------------------------------------------------------------------

    def test_upload_defaults_to_unverified_and_rejects_corrupt_or_oversized_inputs(self):
        # Attempting to bypass UNVERIFIED default in POST /maps -> 422
        st, body, _ = self._register_via_api(
            label='bypass_verify',
            extra_payload={'verification_status': 'VERIFIED'},
        )
        self.assertEqual(st, 422)
        self.assertEqual(body['error']['code'], 'policy_violation')

        # Checksum mismatch -> 422 checksum_mismatch
        st, body, _ = self._register_via_api(
            label='bad_ck',
            extra_payload={'expected_checksums': {'yaml_sha256': '0' * 64}},
        )
        self.assertEqual(st, 422)
        self.assertEqual(body['error']['code'], 'checksum_mismatch')

        # Malformed base64 -> 400
        st, _, _ = self.reg.dispatch_write(
            'POST',
            '/api/v1/map-registry/maps',
            self.operator,
            body={'floor_id': 'fl-1', 'label': 'bad_b64', 'yaml_b64': 'not!valid!b64', 'pgm_b64': _b64(self.sample_pgm)},
        )
        self.assertEqual(st, 400)

        # Raw body exceeding max_http_write_bytes rejected BEFORE JSON parse -> 413
        small_reg = MapRegistry(
            os.path.join(self.tmp.name, 'small.sqlite'),
            os.path.join(self.tmp.name, 'small_store'),
            max_http_write_bytes=256,
        )
        try:
            st, body, _ = small_reg.dispatch_write(
                'POST',
                '/api/v1/map-registry/maps',
                self.operator,
                body=b'{' + (b' ' * 300) + b'}',
            )
            self.assertEqual(st, 413)
            self.assertEqual(body['error']['code'], 'too_large')
        finally:
            small_reg.close()

        # Field base64 exceeding MAX_YAML_BYTES rejected BEFORE base64.b64decode -> 413
        huge_yaml_b64 = 'A' * (((MAX_YAML_BYTES + 2) // 3) * 4 + 32)
        st, body, _ = self.reg.dispatch_write(
            'POST',
            '/api/v1/map-registry/maps',
            self.operator,
            body={
                'floor_id': 'fl-1',
                'label': 'huge_yaml',
                'yaml_b64': huge_yaml_b64,
                'pgm_b64': _b64(self.sample_pgm),
            },
        )
        self.assertEqual(st, 413)
        self.assertEqual(body['error']['code'], 'too_large')

        # Unsafe path segment in URL -> 403
        st, _, _ = self.reg.dispatch_write(
            'POST',
            '/api/v1/map-registry/maps/..%2Fevil/verify',
            self.operator,
            body={'status': 'VERIFIED', 'evidence': {'method': 'check'}},
        )
        self.assertEqual(st, 403)

    # ------------------------------------------------------------------
    # 3. Idempotency-Key & Concurrent Uploads
    # ------------------------------------------------------------------

    def test_idempotency_key_replay_vs_conflict_and_concurrent_uploads(self):
        payload = {
            'floor_id': 'fl-1',
            'label': 'idem_map',
            'source_mode': 'IMPORTED_STATIC',
            'yaml_b64': _b64(self.sample_yaml),
            'pgm_b64': _b64(self.sample_pgm),
        }
        st1, b1, _ = self.reg.dispatch_write(
            'POST',
            '/api/v1/map-registry/maps',
            self.operator,
            body=json.dumps(payload).encode('utf-8'),
            headers={'Idempotency-Key': 'upload-key-001'},
        )
        self.assertEqual(st1, 201)
        self.assertFalse(b1['idempotent_replay'])
        mvid = b1['map_version']['map_version_id']

        # Exact replay using body idempotency_key
        payload_with_key = dict(payload, idempotency_key='upload-key-001')
        st2, b2, _ = self.reg.dispatch_write(
            'POST',
            '/api/v1/map-registry/maps',
            self.operator,
            body=json.dumps(payload_with_key).encode('utf-8'),
        )
        self.assertEqual(st2, 201)
        self.assertTrue(b2['idempotent_replay'])
        self.assertEqual(b2['map_version']['map_version_id'], mvid)

        # Reusing same key with different label/content -> 409 conflict
        different_payload = dict(payload, label='different_label')
        st_diff, b_diff, _ = self.reg.dispatch_write(
            'POST',
            '/api/v1/map-registry/maps',
            self.operator,
            body=json.dumps(different_payload).encode('utf-8'),
            headers={'Idempotency-Key': 'upload-key-001'},
        )
        self.assertEqual(st_diff, 409)
        self.assertEqual(b_diff['error']['code'], 'conflict')

        # Mismatched header vs body idempotency_key -> 400
        st_mismatch, _, _ = self.reg.dispatch_write(
            'POST',
            '/api/v1/map-registry/maps',
            self.operator,
            body=json.dumps(payload_with_key).encode('utf-8'),
            headers={'Idempotency-Key': 'other-key'},
        )
        self.assertEqual(st_mismatch, 400)

        # Concurrent 8-thread upload of the SAME new map (mix of shared idempotency_key & no key)
        pgm_c = _make_pgm_bytes(16, 16, seed=7)
        yaml_c = _make_yaml_bytes(origin='[-0.8, -0.8, 0.0]')
        results: list[tuple[int, dict]] = []
        rlock = threading.Lock()
        barrier = threading.Barrier(8)

        def worker(idx: int):
            barrier.wait()
            req = {
                'floor_id': 'fl-1',
                'label': 'concurrent_map',
                'source_mode': 'SLAM_SAVED_BUNDLE',
                'yaml_b64': _b64(yaml_c),
                'pgm_b64': _b64(pgm_c),
            }
            hdrs = {'Idempotency-Key': 'concurrent-key-1'} if idx < 4 else None
            st, rb, _ = self.reg.dispatch_write('POST', '/api/v1/map-registry/maps', self.operator, body=req, headers=hdrs)
            with rlock:
                results.append((st, rb))

        threads = [threading.Thread(target=worker, args=(i,)) for i in range(8)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self.assertEqual(len(results), 8)
        for st_i, rb_i in results:
            self.assertIn(st_i, (200, 201), msg=f'unexpected failure in concurrent upload: {rb_i}')
        mv_ids = {rb_i['map_version']['map_version_id'] for _, rb_i in results}
        self.assertEqual(len(mv_ids), 1)
        self.assertEqual(len(self.reg.list_map_versions(floor_id='fl-1')), 2)

    # ------------------------------------------------------------------
    # 4. Real subprocess SIGKILL crash recovery during staging
    # ------------------------------------------------------------------

    def test_subprocess_sigkill_during_staging_leaves_active_map_and_db_clean(self):
        # Register and activate a healthy baseline floor map first
        baseline_pgm = _make_pgm_bytes(16, 16, seed=2)
        base_res = self.reg.register_map_bundle(
            floor_id='fl-1',
            label='baseline_ok',
            yaml_bytes=self.sample_yaml,
            pgm_bytes=baseline_pgm,
            source_mode='IMPORTED_STATIC',
            verification_status='VERIFIED',
            verification_evidence={'method': 'survey'},
            actor='op1',
        )
        base_id = base_res['map_version']['map_version_id']
        self.reg.set_floor_active_map('fl-1', base_id, actor='op1', reason='baseline')

        victim_pgm = _make_pgm_bytes(16, 16, seed=9)
        script = f"""
import sys
sys.path.insert(0, {str(SERVER_DIR)!r})
from fleet_server.map_registry import MapRegistry
reg = MapRegistry({self.db_path!r}, {self.bundle_root!r})
def hang_after_stage(stage_dir):
    print('STAGED:' + stage_dir, flush=True)
    import time
    time.sleep(30)
reg.register_map_bundle(
    floor_id='fl-1',
    label='crashed_upload',
    yaml_bytes={self.sample_yaml!r},
    pgm_bytes={victim_pgm!r},
    _after_stage_hook=hang_after_stage,
)
"""
        proc = subprocess.Popen(
            [sys.executable, '-c', script],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            line = proc.stdout.readline().strip()
            self.assertTrue(line.startswith('STAGED:'), msg=f'expected STAGED marker, got {line!r}')
            stage_dir = line.split(':', 1)[1]
            self.assertTrue(os.path.isdir(stage_dir))
            os.kill(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait(timeout=5)
            if proc.stdout:
                proc.stdout.close()
            if proc.stderr:
                proc.stderr.close()

        # Reconcile cleans orphan staging directory without harming existing bundles or active floor pointer
        rec = self.reg.reconcile_storage(quarantine_corrupted=True)
        self.assertIn(os.path.basename(stage_dir), rec['cleaned_staging'])
        self.assertFalse(os.path.exists(stage_dir))
        self.assertEqual(rec['healthy'], [base_id])
        self.assertEqual(rec['corrupted'], [])
        self.assertEqual(self.reg.get_floor_active_map('fl-1')['active']['active_map_version_id'], base_id)
        self.assertEqual([m['label'] for m in self.reg.list_map_versions(floor_id='fl-1')], ['baseline_ok'])

    # ------------------------------------------------------------------
    # 5. Verification lifecycle, revocation, quarantine & unapproved block
    # ------------------------------------------------------------------

    def test_verification_history_revocation_and_quarantine_policies(self):
        # Register UNVERIFIED map via API
        st, body, _ = self._register_via_api(label='hallway_v1', idempotency_key='reg-hallway-1')
        self.assertEqual(st, 201)
        mvid = body['map_version']['map_version_id']

        # Unverified map CANNOT become floor active
        with self.assertRaises(PolicyError):
            self.reg.set_floor_active_map('fl-1', mvid, actor='op1', reason='try unverified')

        # Promote UNVERIFIED -> VERIFIED via API
        st_v, b_v, _ = self.reg.dispatch_write(
            'POST',
            f'/api/v1/map-registry/maps/{mvid.replace("#", "%23")}/verify',
            self.operator,
            body={'status': 'VERIFIED', 'evidence': {'method': 'field_walk', 'ticket': 'OPS-12'}, 'reason': 'checked'},
            headers={'Idempotency-Key': 'ver-hallway-1'},
        )
        self.assertEqual(st_v, 200)
        self.assertEqual(b_v['map_version']['verification_status'], 'VERIFIED')

        # Set as active floor map
        self.reg.set_floor_active_map('fl-1', mvid, actor='op1', reason='go live')
        self.assertEqual(self.reg.get_floor_active_map('fl-1')['active']['active_map_version_id'], mvid)

        # Revoke verification -> transitions to REVOKED, clears floor_active_maps, NEVER deletes files
        st_r, b_r, _ = self.reg.dispatch_write(
            'POST',
            f'/api/v1/map-registry/maps/{mvid.replace("#", "%23")}/revoke',
            self.operator,
            body={'reason': 'glass_wall_changed'},
        )
        self.assertEqual(st_r, 200)
        self.assertTrue(b_r['revoked'])
        self.assertEqual(b_r['cleared_floors'], ['fl-1'])
        self.assertEqual(b_r['map_version']['verification_status'], 'REVOKED')
        self.assertIsNone(self.reg.get_floor_active_map('fl-1')['active'])
        # Bundle files on disk preserved intact
        self.assertTrue(self.reg.verify_bundle_integrity(mvid)['ok'])

        # REVOKED map cannot become floor active or be promoted without allow_reverify=True
        with self.assertRaises(PolicyError):
            self.reg.set_floor_active_map('fl-1', mvid, actor='op1', reason='reactivate revoked')
        st_reverify_blocked, _, _ = self.reg.dispatch_write(
            'POST',
            f'/api/v1/map-registry/maps/{mvid.replace("#", "%23")}/verify',
            self.operator,
            body={'status': 'VERIFIED', 'evidence': {'method': 'recheck'}},
        )
        self.assertEqual(st_reverify_blocked, 422)

        # Re-verify WITH allow_reverify=True works cleanly after inspection
        st_reverify_ok, b_reverify, _ = self.reg.dispatch_write(
            'POST',
            f'/api/v1/map-registry/maps/{mvid.replace("#", "%23")}/verify',
            self.operator,
            body={'status': 'VERIFIED', 'allow_reverify': True, 'reason': 'resurvey_ok', 'evidence': {'method': 'resurvey'}},
        )
        self.assertEqual(st_reverify_ok, 200)
        self.assertEqual(b_reverify['map_version']['verification_status'], 'VERIFIED')

        # Activate again, then quarantine
        self.reg.set_floor_active_map('fl-1', mvid, actor='op1', reason='re-activated')
        st_q, b_q, _ = self.reg.dispatch_write(
            'POST',
            f'/api/v1/map-registry/maps/{mvid.replace("#", "%23")}/quarantine',
            self.operator,
            body={'reason': 'suspected_drift', 'detail': {'reporter': 'op1'}},
        )
        self.assertEqual(st_q, 200)
        self.assertTrue(b_q['quarantined'])
        self.assertEqual(b_q['cleared_floors'], ['fl-1'])
        self.assertIsNone(self.reg.get_floor_active_map('fl-1')['active'])
        self.assertTrue(self.reg.get_map_version(mvid)['quarantine']['is_quarantined'])

        # Quarantined map cannot become floor active even though verification_status == VERIFIED
        with self.assertRaises(PolicyError):
            self.reg.set_floor_active_map('fl-1', mvid, actor='op1', reason='bypass quarantine')

        # Tamper disk file -> operator cannot clear (403); admin fails integrity (500) until restored
        bundle_dir = os.path.join(self.bundle_root, 'bundles', b_reverify['map_version']['storage_key'])
        yaml_disk_path = os.path.join(bundle_dir, 'map.yaml')
        with open(yaml_disk_path, 'rb') as f:
            orig_yaml_disk = f.read()
        os.chmod(yaml_disk_path, 0o644)
        with open(yaml_disk_path, 'wb') as f:
            f.write(b'corrupted: true\n')

        st_op_clr, _, _ = self.reg.dispatch_write(
            'POST',
            f'/api/v1/map-registry/maps/{mvid.replace("#", "%23")}/quarantine/clear',
            self.operator,
            body={'reason': 'op_try_clear'},
        )
        self.assertEqual(st_op_clr, 403)

        st_adm_fail, b_adm_fail, _ = self.reg.dispatch_write(
            'POST',
            f'/api/v1/map-registry/maps/{mvid.replace("#", "%23")}/quarantine/clear',
            self.admin,
            body={'reason': 'try_clear_while_corrupt'},
        )
        self.assertEqual(st_adm_fail, 500)
        self.assertEqual(b_adm_fail['error']['code'], 'bundle_integrity_error')
        # Corrupted bundle files were NOT deleted automatically
        self.assertTrue(os.path.isfile(yaml_disk_path))

        # Restore file -> admin clears quarantine cleanly
        with open(yaml_disk_path, 'wb') as f:
            f.write(orig_yaml_disk)
        os.chmod(yaml_disk_path, 0o444)

        st_adm_ok, b_adm_ok, _ = self.reg.dispatch_write(
            'POST',
            f'/api/v1/map-registry/maps/{mvid.replace("#", "%23")}/quarantine/clear',
            self.admin,
            body={'reason': 'restored_from_backup'},
        )
        self.assertEqual(st_adm_ok, 200)
        self.assertTrue(b_adm_ok['cleared'])
        self.assertFalse(b_adm_ok['map_version']['quarantine']['is_quarantined'])

        # Check append-only verification history via GET dispatcher & trigger immutability
        st_h, h_body, _ = self.reg.dispatch_readonly(
            'GET',
            f'/api/v1/map-registry/maps/{mvid.replace("#", "%23")}/history',
            self.viewer,
        )
        self.assertEqual(st_h, 200)
        transitions = [(h['from_status'], h['to_status']) for h in h_body['history']]
        self.assertEqual(
            transitions,
            [
                ('NONE', 'UNVERIFIED'),
                ('UNVERIFIED', 'VERIFIED'),
                ('VERIFIED', 'REVOKED'),
                ('REVOKED', 'VERIFIED'),
                ('VERIFIED', 'VERIFIED'),  # quarantine
                ('VERIFIED', 'VERIFIED'),  # quarantine clear
            ],
        )
        with self.assertRaises(sqlite3.DatabaseError):
            self.reg.x("UPDATE map_verification_history SET reason='tampered'")
        with self.assertRaises(sqlite3.DatabaseError):
            self.reg.x('DELETE FROM map_verification_history')

    def test_reconcile_quarantine_corrupted_preserves_files_and_clears_active_pointer(self):
        res = self.reg.register_map_bundle(
            floor_id='fl-1',
            label='will_corrupt',
            yaml_bytes=self.sample_yaml,
            pgm_bytes=self.sample_pgm,
            source_mode='IMPORTED_STATIC',
            verification_status='VERIFIED',
            verification_evidence={'method': 'qa'},
            actor='op1',
        )
        mvid = res['map_version']['map_version_id']
        self.reg.set_floor_active_map('fl-1', mvid, actor='op1', reason='active')

        pgm_path = os.path.join(self.bundle_root, 'bundles', res['map_version']['storage_key'], 'map.pgm')
        os.chmod(pgm_path, 0o644)
        with open(pgm_path, 'wb') as f:
            f.write(b'corrupted-pgm')

        rec = self.reg.reconcile_storage(quarantine_corrupted=True)
        self.assertEqual(rec['quarantined'], [mvid])
        self.assertTrue(os.path.isfile(pgm_path))
        self.assertIsNone(self.reg.get_floor_active_map('fl-1')['active'])
        self.assertTrue(self.reg.get_map_version(mvid)['quarantine']['is_quarantined'])

        st_q, q_body, _ = self.reg.dispatch_readonly('GET', '/api/v1/map-registry/quarantines', self.viewer)
        self.assertEqual(st_q, 200)
        self.assertEqual(len(q_body['quarantines']), 1)
        self.assertTrue(q_body['quarantines'][0]['is_quarantined'])

    # ------------------------------------------------------------------
    # 6. RBAC, Auth & Out-of-Scope Operation Rejection
    # ------------------------------------------------------------------

    def test_rbac_auth_and_non_goals_blocked(self):
        # Missing auth -> 401
        st, _, _ = self.reg.dispatch_write('POST', '/api/v1/map-registry/facilities', None, body={'facility_id': 'f2', 'name': 'F2'})
        self.assertEqual(st, 401)

        # Viewer & agent -> 403 across write routes
        for p in (self.viewer, self.agent):
            for route in (
                '/api/v1/map-registry/facilities',
                '/api/v1/map-registry/maps',
                '/api/v1/map-registry/maps/foo%23000000000000/verify',
                '/api/v1/map-registry/maps/foo%23000000000000/revoke',
                '/api/v1/map-registry/maps/foo%23000000000000/quarantine',
            ):
                st_f, _, _ = self.reg.dispatch_write('POST', route, p, body={})
                self.assertEqual(st_f, 403, msg=f'{p.role} should be forbidden on {route}')

        # Wrong HTTP method -> 405
        st_get, _, _ = self.reg.dispatch_write('GET', '/api/v1/map-registry/maps', self.operator)
        self.assertEqual(st_get, 405)

        # Explicitly excluded live-robot / Nav2 / SLAM routes are NOT implemented -> 404
        for forbidden_future_route in (
            '/api/v1/map-registry/floors/fl-1/active-map',
            '/api/v1/map-registry/robots/limo_1/set-map',
            '/api/v1/map-registry/slam/start',
            '/api/v1/map-registry/initialpose',
        ):
            st_nf, _, _ = self.reg.dispatch_write('POST', forbidden_future_route, self.admin, body={})
            self.assertEqual(st_nf, 404, msg=f'{forbidden_future_route} must not be exposed in Phase 2-B')


if __name__ == '__main__':
    unittest.main()
