"""Unit tests for Map Registry Phase 1 (`fleet_server.map_registry`).

Covers:
1. Canonical SHA-256 identity & byte-for-byte parity with `map_identity.py` and real `limo_sim_map` fixtures.
2. Hierarchy CRUD (Facility -> Building -> Floor) & conflict/FK checks.
3. Path traversal, symlink, escaping `image:`, and malformed YAML/PGM rejection.
4. Checksum mismatches & atomic failure rollback (zero orphaned files or DB rows).
5. Immutable MapVersion storage & idempotent vs conflicting duplicate registration.
6. Policy enforcement: unverified / live SLAM snapshots cannot become VERIFIED or ACTIVE;
   registering a map never alters floor pointers or robot bindings; switching floor pointer
   preserves old MapVersion records untouched.
7. SLAM live `/map` progression vs `map_epoch` state change classification.
8. Open-RMF transform & NavGraph compatibility validation (`free_clearance_m`, lanes, level match, origin_yaw).
9. Storage reconciliation (`reconcile_storage`) & readonly HTTP dispatcher (`/api/v1/map-registry/*`).
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import sqlite3
import sys
import tempfile
import threading
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SERVER_DIR = REPO_ROOT / 'fleet_control' / 'server'
EXPERIMENTS_PKG_DIR = REPO_ROOT / 'experiments' / 'real_rmf_20261113'
SIM_MAPS_DIR = REPO_ROOT / 'fleet_control' / 'maps'

sys.path.insert(0, str(SERVER_DIR))

from fleet_server.auth import Principal  # noqa: E402
from fleet_server.map_registry import (  # noqa: E402
    ChecksumError,
    ConflictError,
    IntegrityFailureError,
    MapRegistry,
    NotFoundError,
    PayloadTooLargeError,
    PolicyError,
    SecurityError,
    ValidationError,
    classify_map_update,
    compute_content_sha256,
    parse_pgm_p5,
    parse_ros_map_yaml,
    validate_rmf_compatibility,
)


def _make_pgm_bytes(rows: list[list[int]], maxval: int = 255) -> bytes:
    height = len(rows)
    width = len(rows[0]) if height else 0
    header = f'P5\n{width} {height}\n{maxval}\n'.encode('ascii')
    body = bytes(px for row in rows for px in row)
    return header + body


def _make_yaml_bytes(
    *,
    image: str = 'map.pgm',
    resolution: float = 0.05,
    origin: str = '[-1.0, -1.0, 0.0]',
    negate: int = 0,
    occupied_thresh: float = 0.65,
    free_thresh: float = 0.25,
    extra: str = '',
) -> bytes:
    lines = [
        f'image: {image}',
        'mode: trinary',
        f'resolution: {resolution}',
        f'origin: {origin}',
        f'negate: {negate}',
        f'occupied_thresh: {occupied_thresh}',
        f'free_thresh: {free_thresh}',
    ]
    if extra:
        lines.append(extra)
    return ('\n'.join(lines) + '\n').encode('utf-8')


class MapRegistryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.db_path = os.path.join(self.tmp.name, 'registry.sqlite')
        self.bundle_root = os.path.join(self.tmp.name, 'map_store')
        self.reg = MapRegistry(self.db_path, self.bundle_root)
        # Base hierarchy for tests
        self.reg.create_facility('fac-main', 'Capstone Campus', actor='tester')
        self.reg.create_building('bldg-eng', 'fac-main', 'Engineering Hall', actor='tester')
        self.reg.create_floor('fl-1', 'bldg-eng', 'L1', elevation_m=0.0, rmf_map_name='L1', actor='tester')

        # 20x20 grid (1.0m x 1.0m at 0.05m/px, origin [-0.5, -0.5, 0]):
        # Border 2px occupied (0), interior free (254), center 2x2 unknown (205).
        rows: list[list[int]] = []
        for r in range(20):
            row: list[int] = []
            for c in range(20):
                if r < 2 or r >= 18 or c < 2 or c >= 18:
                    row.append(0)
                elif 9 <= r <= 10 and 9 <= c <= 10:
                    row.append(205)
                else:
                    row.append(254)
            rows.append(row)
        self.sample_pgm = _make_pgm_bytes(rows)
        self.sample_yaml = _make_yaml_bytes(origin='[-0.5, -0.5, 0.0]')

    def tearDown(self):
        self.reg.close()
        self.tmp.cleanup()

    # ------------------------------------------------------------------
    # 1. Canonical hash & parity with map_identity.py + sim fixture
    # ------------------------------------------------------------------

    def test_canonical_sha256_matches_experiments_map_identity(self):
        sys.path.insert(0, str(EXPERIMENTS_PKG_DIR))
        try:
            from limo_rmf_real import map_identity  # type: ignore
        finally:
            if str(EXPERIMENTS_PKG_DIR) in sys.path:
                sys.path.remove(str(EXPERIMENTS_PKG_DIR))

        sim_pgm_path = SIM_MAPS_DIR / 'limo_sim_map.pgm'
        self.assertTrue(sim_pgm_path.is_file())
        parsed = parse_pgm_p5(sim_pgm_path.read_bytes())
        arr = map_identity.read_pgm(sim_pgm_path)
        canon_ref = map_identity.canonical_from_pgm_array(arr)
        hash_ref = map_identity.content_sha256(canon_ref)
        self.assertEqual(parsed['content_sha256'], hash_ref)
        self.assertEqual(parsed['canonical_bytes'], canon_ref.tobytes())

    def test_register_sim_fixture_from_dir(self):
        res = self.reg.register_map_bundle_from_dir(
            floor_id='fl-1',
            label='limo_sim_map',
            bundle_dir=str(SIM_MAPS_DIR),
            yaml_filename='limo_sim_map.yaml',
            source_mode='SIM_FIXTURE',
            verification_status='VERIFIED',
            verification_evidence={'method': 'fixture_sha256_check'},
            actor='tester',
        )
        self.assertTrue(res['created'])
        mv = res['map_version']
        self.assertTrue(mv['map_version_id'].startswith('limo_sim_map#'))
        self.assertEqual(mv['verification_status'], 'VERIFIED')
        # Registering must not auto-activate the floor or bind any robot
        self.assertIsNone(self.reg.get_floor_active_map('fl-1')['active'])
        self.assertEqual(self.reg.q('SELECT * FROM robot_map_bindings'), [])

    # ------------------------------------------------------------------
    # 2. Hierarchy validation & conflicts
    # ------------------------------------------------------------------

    def test_hierarchy_conflicts_and_missing_parents(self):
        with self.assertRaises(NotFoundError):
            self.reg.create_building('b2', 'missing-fac', 'Ghost Building')
        with self.assertRaises(NotFoundError):
            self.reg.create_floor('fl-x', 'missing-bldg', 'L2')
        with self.assertRaises(ConflictError):
            self.reg.create_floor('fl-dup', 'bldg-eng', 'L1')  # duplicate level_name in building
        with self.assertRaises(ConflictError):
            self.reg.create_facility('fac-main', 'Different Name')

    # ------------------------------------------------------------------
    # 3. Path traversal, symlinks, invalid YAML/PGM
    # ------------------------------------------------------------------

    def test_path_traversal_and_unsafe_yaml_rejected(self):
        with self.assertRaises(SecurityError):
            parse_ros_map_yaml(_make_yaml_bytes(image='../../etc/passwd'))
        with self.assertRaises(SecurityError):
            parse_ros_map_yaml(_make_yaml_bytes(image='/tmp/map.pgm'))
        with self.assertRaises(SecurityError):
            parse_ros_map_yaml(_make_yaml_bytes(image='subdir/map.pgm'))
        with self.assertRaises(SecurityError):
            parse_ros_map_yaml(_make_yaml_bytes(extra='!!python/object/apply:os.system ["id"]'))
        with self.assertRaises(SecurityError):
            self.reg.register_map_bundle(
                floor_id='fl-1',
                label='../escape',
                yaml_bytes=self.sample_yaml,
                pgm_bytes=self.sample_pgm,
            )

    def test_symlink_bundle_dir_and_file_rejected(self):
        real_dir = os.path.join(self.tmp.name, 'src_bundle')
        os.makedirs(real_dir)
        secret_file = os.path.join(self.tmp.name, 'secret.pgm')
        with open(secret_file, 'wb') as f:
            f.write(self.sample_pgm)
        with open(os.path.join(real_dir, 'map.yaml'), 'wb') as f:
            f.write(self.sample_yaml)
        os.symlink(secret_file, os.path.join(real_dir, 'map.pgm'))

        with self.assertRaises(SecurityError):
            self.reg.register_map_bundle_from_dir(
                floor_id='fl-1',
                label='sym-test',
                bundle_dir=real_dir,
            )

        sym_dir = os.path.join(self.tmp.name, 'sym_dir')
        os.symlink(real_dir, sym_dir)
        with self.assertRaises(SecurityError):
            self.reg.register_map_bundle_from_dir(
                floor_id='fl-1',
                label='sym-dir',
                bundle_dir=sym_dir,
            )

    def test_invalid_pgm_and_yaml_values_rejected(self):
        # P2 ASCII PGM rejected
        with self.assertRaises(ValidationError):
            parse_pgm_p5(b'P2\n2 2\n255\n0 255 255 0\n')
        # Truncated pixel buffer
        with self.assertRaises(ValidationError):
            parse_pgm_p5(b'P5\n4 4\n255\n' + b'\xff' * 10)
        # Trailing garbage bytes
        with self.assertRaises(ValidationError):
            parse_pgm_p5(b'P5\n2 2\n255\n' + b'\xff' * 5)
        # Bad maxval
        with self.assertRaises(ValidationError):
            parse_pgm_p5(b'P5\n2 2\n65535\n' + b'\xff' * 8)
        # Non-finite YAML resolution/origin and inverted thresholds
        with self.assertRaises(ValidationError):
            parse_ros_map_yaml(_make_yaml_bytes(origin='[nan, 0.0, 0.0]'))
        with self.assertRaises(ValidationError):
            parse_ros_map_yaml(_make_yaml_bytes(resolution=-0.05))
        with self.assertRaises(ValidationError):
            parse_ros_map_yaml(_make_yaml_bytes(occupied_thresh=0.2, free_thresh=0.6))

    # ------------------------------------------------------------------
    # 4. Checksums & atomic rollback on failure
    # ------------------------------------------------------------------

    def test_checksum_mismatch_and_atomic_rollback_leave_zero_orphans(self):
        with self.assertRaises(ChecksumError):
            self.reg.register_map_bundle(
                floor_id='fl-1',
                label='bad-sum',
                yaml_bytes=self.sample_yaml,
                pgm_bytes=self.sample_pgm,
                expected_checksums={'pgm_sha256': '0' * 64},
            )
        self.assertEqual(os.listdir(self.reg.bundles_dir), [])
        self.assertEqual(os.listdir(self.reg.staging_dir), [])
        self.assertEqual(self.reg.list_map_versions(), [])

        # Simulate mid-registration failure after files are staged
        def boom(_stage_dir: str):
            raise RuntimeError('simulated disk/db crash after staging')

        with self.assertRaisesRegex(RuntimeError, 'simulated'):
            self.reg.register_map_bundle(
                floor_id='fl-1',
                label='crashed-reg',
                yaml_bytes=self.sample_yaml,
                pgm_bytes=self.sample_pgm,
                _after_stage_hook=boom,
            )
        self.assertEqual(os.listdir(self.reg.bundles_dir), [])
        self.assertEqual(os.listdir(self.reg.staging_dir), [])
        self.assertEqual(self.reg.list_map_versions(), [])

    def test_posegraph_checksums_and_bundle_tamper_detection(self):
        pg = b'POSEGRAPH_V1_HEADER'
        pgd = b'POSEGRAPH_BINARY_DATA'
        res = self.reg.register_map_bundle(
            floor_id='fl-1',
            label='with-pg',
            yaml_bytes=self.sample_yaml,
            pgm_bytes=self.sample_pgm,
            posegraph_bytes=pg,
            posegraph_data_bytes=pgd,
            expected_checksums={
                'posegraph_sha256': hashlib.sha256(pg).hexdigest(),
                'posegraph_data_sha256': hashlib.sha256(pgd).hexdigest(),
            },
        )
        mvid = res['map_version']['map_version_id']
        self.assertTrue(self.reg.verify_bundle_integrity(mvid)['ok'])
        self.assertEqual(self.reg.read_bundle_file(mvid, 'map.posegraph'), pg)

        # Tamper with stored bundle file on disk
        skey = res['map_version']['storage_key']
        target = os.path.join(self.reg.bundles_dir, skey, 'map.pgm')
        os.chmod(target, 0o644)
        with open(target, 'ab') as f:
            f.write(b'CORRUPT')
        integrity = self.reg.verify_bundle_integrity(mvid)
        self.assertFalse(integrity['ok'])
        with self.assertRaises(IntegrityFailureError):
            self.reg.read_bundle_file(mvid, 'map.pgm')

    # ------------------------------------------------------------------
    # 5. Immutability, idempotency, conflict, pointer separation
    # ------------------------------------------------------------------

    def test_duplicate_idempotent_vs_conflict_and_pointer_separation(self):
        ev = {'method': 'operator_review', 'note': 'checked'}
        r1 = self.reg.register_map_bundle(
            floor_id='fl-1',
            label='office',
            yaml_bytes=self.sample_yaml,
            pgm_bytes=self.sample_pgm,
            source_mode='SLAM_SAVED_BUNDLE',
            verification_status='VERIFIED',
            verification_evidence=ev,
            actor='op1',
        )
        self.assertTrue(r1['created'])
        mvid1 = r1['map_version']['map_version_id']

        # Exact identical re-register -> idempotent
        r1_dup = self.reg.register_map_bundle(
            floor_id='fl-1',
            label='office',
            yaml_bytes=self.sample_yaml,
            pgm_bytes=self.sample_pgm,
            source_mode='SLAM_SAVED_BUNDLE',
            actor='op1',
        )
        self.assertFalse(r1_dup['created'])
        self.assertTrue(r1_dup['idempotent'])

        # Same content_sha256 + label (same map_version_id) but modified YAML origin -> ConflictError 409
        yaml_shifted = _make_yaml_bytes(origin='[2.0, 2.0, 0.0]')
        with self.assertRaises(ConflictError):
            self.reg.register_map_bundle(
                floor_id='fl-1',
                label='office',
                yaml_bytes=yaml_shifted,
                pgm_bytes=self.sample_pgm,
                source_mode='SLAM_SAVED_BUNDLE',
            )

        # Register second distinct map version v2 (slightly modified free pixel -> distinct content hash)
        pgm_v2 = bytearray(self.sample_pgm)
        pgm_v2[-1] = 250  # border occupied pixel -> valid P5 different content
        r2 = self.reg.register_map_bundle(
            floor_id='fl-1',
            label='office',
            yaml_bytes=self.sample_yaml,
            pgm_bytes=bytes(pgm_v2),
            source_mode='SLAM_SAVED_BUNDLE',
            verification_status='VERIFIED',
            verification_evidence=ev,
            actor='op1',
        )
        mvid2 = r2['map_version']['map_version_id']
        self.assertNotEqual(mvid1, mvid2)

        # Activate v1 then switch floor pointer to v2: both immutable MapVersions keep exact hashes
        act1 = self.reg.set_floor_active_map('fl-1', mvid1, actor='op1', reason='initial deployment')
        self.assertEqual(act1['active']['pointer_revision'], 1)
        self.assertEqual(act1['active']['active_map_version_id'], mvid1)

        act2 = self.reg.set_floor_active_map('fl-1', mvid2, actor='op1', reason='updated perimeter')
        self.assertEqual(act2['active']['pointer_revision'], 2)
        self.assertEqual(act2['active']['active_map_version_id'], mvid2)

        mv1_after = self.reg.get_map_version(mvid1)
        mv2_after = self.reg.get_map_version(mvid2)
        self.assertFalse(mv1_after['is_floor_active'])
        self.assertTrue(mv2_after['is_floor_active'])
        self.assertEqual(mv1_after['sha256'], r1['map_version']['sha256'])
        self.assertEqual(mv2_after['sha256'], r2['map_version']['sha256'])

    # ------------------------------------------------------------------
    # 6. Policy 6: unverified & live SLAM snapshots blocked from VERIFIED / ACTIVE
    # ------------------------------------------------------------------

    def test_unverified_and_live_slam_cannot_be_verified_or_activated(self):
        # Direct registration of SLAM_LIVE_SNAPSHOT as VERIFIED -> blocked
        with self.assertRaises(PolicyError):
            self.reg.register_map_bundle(
                floor_id='fl-1',
                label='live-snap',
                yaml_bytes=self.sample_yaml,
                pgm_bytes=self.sample_pgm,
                source_mode='SLAM_LIVE_SNAPSHOT',
                verification_status='VERIFIED',
                verification_evidence={'method': 'auto'},
            )

        # Register live SLAM as UNVERIFIED -> allowed as unverified record, but cannot promote or activate
        live_res = self.reg.register_map_bundle(
            floor_id='fl-1',
            label='live-snap',
            yaml_bytes=self.sample_yaml,
            pgm_bytes=self.sample_pgm,
            source_mode='SLAM_LIVE_SNAPSHOT',
            verification_status='UNVERIFIED',
        )
        live_id = live_res['map_version']['map_version_id']
        with self.assertRaises(PolicyError):
            self.reg.verify_map_version(live_id, status='VERIFIED', evidence={'method': 'manual'})
        with self.assertRaises(PolicyError):
            self.reg.set_floor_active_map('fl-1', live_id, actor='op1', reason='should fail')

        # Static bundle registered UNVERIFIED cannot activate until verified
        draft_res = self.reg.register_map_bundle(
            floor_id='fl-1',
            label='draft-map',
            yaml_bytes=self.sample_yaml,
            pgm_bytes=self.sample_pgm,
            source_mode='IMPORTED_STATIC',
            verification_status='UNVERIFIED',
        )
        draft_id = draft_res['map_version']['map_version_id']
        with self.assertRaises(PolicyError):
            self.reg.set_floor_active_map('fl-1', draft_id, actor='op1', reason='unverified')

        verified = self.reg.verify_map_version(
            draft_id,
            status='VERIFIED',
            evidence={'method': 'checksum_and_graph_check'},
            actor='admin1',
        )
        self.assertEqual(verified['verification_status'], 'VERIFIED')
        act = self.reg.set_floor_active_map('fl-1', draft_id, actor='admin1', reason='now verified')
        self.assertEqual(act['active']['active_map_version_id'], draft_id)

    # ------------------------------------------------------------------
    # 7. Normal SLAM `/map` progression vs `map_epoch` change
    # ------------------------------------------------------------------

    def test_slam_progressive_growth_vs_map_epoch_changes(self):
        obs1 = {
            'operation_mode': 'MAPPING',
            'content_sha256': 'a' * 64,
            'resolution': 0.05,
            'origin': [-2.0, -2.0, 0.0],
        }
        c1 = classify_map_update(None, obs1)
        self.assertEqual(c1['classification'], 'SLAM_INITIAL_GRID')
        self.assertFalse(c1['increment_map_epoch'])
        self.assertFalse(c1['navigable'])

        # Normal SLAM /map update expands map content without origin/res jump
        obs2 = {
            'operation_mode': 'MAPPING',
            'content_sha256': 'b' * 64,
            'resolution': 0.05,
            'origin': [-2.0, -2.0, 0.0],
        }
        c2 = classify_map_update(obs1, obs2)
        self.assertEqual(c2['classification'], 'SLAM_PROGRESSIVE_GROWTH')
        self.assertFalse(c2['increment_map_epoch'])
        self.assertFalse(c2['invalidate_navigation'])

        # Frame discontinuity during MAPPING -> map_epoch advance required
        obs_jump = {
            'operation_mode': 'MAPPING',
            'content_sha256': 'c' * 64,
            'resolution': 0.05,
            'origin': [5.0, 1.0, 0.0],
        }
        c_jump = classify_map_update(obs2, obs_jump)
        self.assertEqual(c_jump['classification'], 'MAPPING_FRAME_DISCONTINUITY')
        self.assertTrue(c_jump['increment_map_epoch'])

        # NAVIGATION static match vs mismatch
        bound = {
            'map_version_id': 'office#' + ('f' * 12),
            'verification_status': 'VERIFIED',
            'source_mode': 'SLAM_SAVED_BUNDLE',
            'content_sha256': 'f' * 64,
            'resolution': 0.05,
            'origin_x': -0.5,
            'origin_y': -0.5,
            'origin_yaw': 0.0,
        }
        nav_ok = classify_map_update(
            None,
            {
                'operation_mode': 'NAVIGATION',
                'map_version_id': bound['map_version_id'],
                'content_sha256': 'f' * 64,
                'resolution': 0.05,
                'origin': [-0.5, -0.5, 0.0],
            },
            bound_version=bound,
        )
        self.assertEqual(nav_ok['classification'], 'VERIFIED_STATIC_MATCH')
        self.assertFalse(nav_ok['increment_map_epoch'])
        self.assertTrue(nav_ok['navigable'])

        nav_switch = classify_map_update(
            None,
            {
                'operation_mode': 'NAVIGATION',
                'map_version_id': 'other#123456789012',
                'content_sha256': 'e' * 64,
                'resolution': 0.05,
                'origin': [-0.5, -0.5, 0.0],
            },
            bound_version=bound,
        )
        self.assertEqual(nav_switch['classification'], 'MAP_VERSION_SWITCH_REQUIRED')
        self.assertTrue(nav_switch['increment_map_epoch'])
        self.assertTrue(nav_switch['invalidate_navigation'])

    # ------------------------------------------------------------------
    # 8. Open-RMF transform & NavGraph compatibility
    # ------------------------------------------------------------------

    def test_open_rmf_nav_graph_compatibility(self):
        parsed_pgm = parse_pgm_p5(self.sample_pgm)
        parsed_yaml = parse_ros_map_yaml(self.sample_yaml)
        meta = {
            'width': parsed_pgm['width'],
            'height': parsed_pgm['height'],
            'resolution': parsed_yaml['resolution'],
            'origin_x': parsed_yaml['origin_x'],
            'origin_y': parsed_yaml['origin_y'],
            'origin_yaw': parsed_yaml['origin_yaw'],
        }
        # Identity RMF->Nav2 transform with translation [10.0, 20.0]
        # Free area in Nav2 is roughly x,y in [-0.39, +0.39] skipping center [-0.05, +0.05] unknown block.
        tf = {'rotation_rad': 0.0, 'translation': [10.0, 20.0], 'rmf_level': 'L1'}
        good_graph = {
            'level_name': 'L1',
            'waypoints': [
                {'name': 'wp_a', 'x': 9.75, 'y': 20.25},   # nav2 (-0.25, +0.25) -> free
                {'name': 'wp_b', 'x': 10.25, 'y': 20.25},  # nav2 (+0.25, +0.25) -> free
            ],
            'lanes': [[0, 1]],
        }
        ok_report = validate_rmf_compatibility(
            meta,
            parsed_pgm['canonical_bytes'],
            tf,
            nav_graph=good_graph,
            floor_level_name='L1',
            free_clearance_m=0.05,
        )
        self.assertTrue(ok_report['compatible'], ok_report)
        self.assertEqual(ok_report['checked_waypoints'], 2)
        self.assertEqual(ok_report['checked_lanes'], 1)

        # Lane passing diagonally through center unknown 2x2 block must fail
        bad_lane_graph = {
            'level_name': 'L1',
            'waypoints': [
                {'name': 'wp_sw', 'x': 9.75, 'y': 19.75},  # nav2 (-0.25, -0.25)
                {'name': 'wp_ne', 'x': 10.25, 'y': 20.25}, # nav2 (+0.25, +0.25) -> crosses (0,0) UNKNOWN
            ],
            'lanes': [[0, 1]],
        }
        bad_lane_report = validate_rmf_compatibility(
            meta,
            parsed_pgm['canonical_bytes'],
            tf,
            nav_graph=bad_lane_graph,
            floor_level_name='L1',
        )
        self.assertFalse(bad_lane_report['compatible'])
        self.assertTrue(any('crosses non-free space' in e for e in bad_lane_report['errors']))

        # Wrong floor level must fail and block VERIFIED registration
        wrong_level_tf = {'rotation_rad': 0.0, 'translation': [10.0, 20.0], 'rmf_level': 'L2'}
        with self.assertRaises(PolicyError):
            self.reg.register_map_bundle(
                floor_id='fl-1',
                label='bad-rmf',
                yaml_bytes=self.sample_yaml,
                pgm_bytes=self.sample_pgm,
                source_mode='IMPORTED_STATIC',
                verification_status='VERIFIED',
                verification_evidence={'method': 'auto'},
                transform_rmf_from_nav2=wrong_level_tf,
                nav_graph=good_graph,
            )

    # ------------------------------------------------------------------
    # 9. Storage reconciliation & readonly REST dispatcher
    # ------------------------------------------------------------------

    def test_reconcile_storage_and_readonly_http_dispatch(self):
        orphan_stage = os.path.join(self.reg.staging_dir, 'stage_orphan_crash')
        os.makedirs(orphan_stage)
        with open(os.path.join(orphan_stage, 'partial.tmp'), 'wb') as f:
            f.write(b'partial')

        reg_res = self.reg.register_map_bundle(
            floor_id='fl-1',
            label='clean-map',
            yaml_bytes=self.sample_yaml,
            pgm_bytes=self.sample_pgm,
            source_mode='IMPORTED_STATIC',
            verification_status='VERIFIED',
            verification_evidence={'method': 'manual'},
            actor='admin1',
        )
        mvid = reg_res['map_version']['map_version_id']
        self.reg.set_floor_active_map('fl-1', mvid, actor='admin1', reason='go live')

        rec = self.reg.reconcile_storage()
        self.assertEqual(rec['cleaned_staging'], ['stage_orphan_crash'])
        self.assertEqual(rec['healthy'], [mvid])
        self.assertEqual(rec['corrupted'], [])
        self.assertEqual(os.listdir(self.reg.staging_dir), [])

        viewer = Principal('viewer', 'viewer')
        agent = Principal('agent-limo_01', 'agent', 'limo_01')

        # Role enforcement & method restriction
        st, _, _ = self.reg.dispatch_readonly('POST', '/api/v1/map-registry/maps', viewer)
        self.assertEqual(st, 405)
        st, _, _ = self.reg.dispatch_readonly('GET', '/api/v1/map-registry/facilities', None)
        self.assertEqual(st, 401)
        st, _, _ = self.reg.dispatch_readonly('GET', '/api/v1/map-registry/facilities', agent)
        self.assertEqual(st, 403)

        # Valid readonly routes
        st, body, _ = self.reg.dispatch_readonly('GET', '/api/v1/map-registry/facilities', viewer)
        self.assertEqual(st, 200)
        self.assertEqual(body['facilities'][0]['buildings'][0]['floors'][0]['active_map_version_id'], mvid)

        st, body, _ = self.reg.dispatch_readonly('GET', '/api/v1/map-registry/maps?floor_id=fl-1', viewer)
        self.assertEqual(st, 200)
        self.assertEqual(len(body['maps']), 1)

        enc_id = mvid.replace('#', '%23')
        st, body, _ = self.reg.dispatch_readonly('GET', f'/api/v1/map-registry/maps/{enc_id}/meta', viewer)
        self.assertEqual(st, 200)
        self.assertTrue(body['is_floor_active'])

        st, raw_yaml, ctype = self.reg.dispatch_readonly('GET', f'/api/v1/map-registry/maps/{enc_id}/bundle/map.yaml', viewer)
        self.assertEqual(st, 200)
        self.assertIn('yaml', ctype)
        self.assertEqual(raw_yaml, self.sample_yaml)

        st, png_bytes, ctype = self.reg.dispatch_readonly('GET', f'/api/v1/map-registry/maps/{enc_id}/preview.png', viewer)
        self.assertEqual(st, 200)
        self.assertEqual(ctype, 'image/png')
        self.assertEqual(png_bytes[:8], b'\x89PNG\r\n\x1a\n')

        # Accepts http_api.py flat query dict and strips trailing slashes
        st, body, _ = self.reg.dispatch_readonly('GET', '/api/v1/map-registry/maps/', viewer, query={'floor_id': 'fl-1'})
        self.assertEqual(st, 200)
        self.assertEqual(len(body['maps']), 1)

        # Traversal on bundle read URL blocked cleanly with http_api.py error envelope
        st, err_body, _ = self.reg.dispatch_readonly('GET', f'/api/v1/map-registry/maps/{enc_id}/bundle/..%2Fsecret', viewer)
        self.assertEqual(st, 403)
        self.assertEqual(err_body['error']['code'], 'security_violation')
        self.assertIn('message', err_body['error'])

    # ------------------------------------------------------------------
    # 10. Phase 1.5: Orphan unregistered bundle & missing/corrupt DB recovery
    # ------------------------------------------------------------------

    def test_unregistered_orphan_bundle_detected_and_never_deleted(self):
        orphan_key = 'orphan_map__abcdef012345__12345678'
        orphan_dir = os.path.join(self.reg.bundles_dir, orphan_key)
        os.makedirs(orphan_dir)
        marker = os.path.join(orphan_dir, 'manifest.json')
        with open(marker, 'w', encoding='utf-8') as f:
            json.dump({'map_version_id': 'orphan_map#abcdef012345'}, f)

        rec = self.reg.reconcile_storage()
        self.assertEqual(len(rec['unregistered_bundles']), 1)
        ub = rec['unregistered_bundles'][0]
        self.assertEqual(ub['storage_key'], orphan_key)
        self.assertTrue(ub['has_manifest'])
        self.assertEqual(ub['manifest_map_version_id'], 'orphan_map#abcdef012345')
        self.assertEqual(ub['status'], 'ORPHAN_UNREGISTERED_BUNDLE')
        # Reconcile MUST preserve the orphan directory untouched
        self.assertTrue(os.path.isdir(orphan_dir))
        self.assertTrue(os.path.isfile(marker))

    def test_missing_bundle_on_disk_fails_closed_without_mutating_floor_pointer(self):
        ev = {'method': 'operator_review'}
        r1 = self.reg.register_map_bundle(
            floor_id='fl-1',
            label='good-base',
            yaml_bytes=self.sample_yaml,
            pgm_bytes=self.sample_pgm,
            source_mode='IMPORTED_STATIC',
            verification_status='VERIFIED',
            verification_evidence=ev,
        )
        mvid1 = r1['map_version']['map_version_id']
        self.reg.set_floor_active_map('fl-1', mvid1, actor='op1', reason='baseline')

        pgm_v2 = bytearray(self.sample_pgm)
        pgm_v2[-1] = 250
        r2 = self.reg.register_map_bundle(
            floor_id='fl-1',
            label='missing-disk',
            yaml_bytes=self.sample_yaml,
            pgm_bytes=bytes(pgm_v2),
            source_mode='IMPORTED_STATIC',
            verification_status='UNVERIFIED',
        )
        mvid2 = r2['map_version']['map_version_id']
        # Remove r2 files from disk while DB row stays present
        shutil.rmtree(os.path.join(self.reg.bundles_dir, r2['map_version']['storage_key']))

        self.assertFalse(self.reg.verify_bundle_integrity(mvid2)['ok'])
        with self.assertRaises(IntegrityFailureError):
            self.reg.verify_map_version(mvid2, status='VERIFIED', evidence=ev)
        with self.assertRaises(IntegrityFailureError):
            self.reg.read_bundle_file(mvid2, 'map.pgm')
        with self.assertRaises(IntegrityFailureError):
            self.reg.render_preview_png(mvid2)

        # Active pointer must remain safely on mvid1
        self.assertEqual(self.reg.get_floor_active_map('fl-1')['active']['active_map_version_id'], mvid1)
        rec = self.reg.reconcile_storage()
        self.assertEqual(rec['healthy'], [mvid1])
        self.assertEqual([c['map_version_id'] for c in rec['corrupted']], [mvid2])

    # ------------------------------------------------------------------
    # 11. Phase 1.5: Concurrent registration race & all metadata conflict variants
    # ------------------------------------------------------------------

    def test_concurrent_identical_registration_no_spurious_conflict(self):
        n_threads = 6
        barrier = threading.Barrier(n_threads)
        results: list[dict] = []
        errors: list[Exception] = []
        lock = threading.Lock()

        def worker(idx: int):
            try:
                barrier.wait(timeout=5)
                res = self.reg.register_map_bundle(
                    floor_id='fl-1',
                    label='race-map',
                    yaml_bytes=self.sample_yaml,
                    pgm_bytes=self.sample_pgm,
                    source_mode='IMPORTED_STATIC',
                    actor=f'worker-{idx}',
                )
                with lock:
                    results.append(res)
            except Exception as exc:
                with lock:
                    errors.append(exc)

        threads = [threading.Thread(target=worker, args=(i,)) for i in range(n_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(10)

        self.assertEqual(errors, [])
        self.assertEqual(len(results), n_threads)
        self.assertEqual(sum(1 for r in results if r['created']), 1)
        self.assertEqual(sum(1 for r in results if r['idempotent']), n_threads - 1)
        self.assertEqual(len(self.reg.list_map_versions(floor_id='fl-1')), 1)

    def test_same_map_id_with_resolution_yaml_posegraph_or_floor_mismatch_conflicts(self):
        self.reg.create_floor('fl-2', 'bldg-eng', 'L2', elevation_m=3.5, actor='tester')
        self.reg.register_map_bundle(
            floor_id='fl-1',
            label='variant',
            yaml_bytes=self.sample_yaml,
            pgm_bytes=self.sample_pgm,
            source_mode='IMPORTED_STATIC',
            posegraph_bytes=b'PG_A',
        )

        # Same pixels + label, different resolution
        with self.assertRaises(ConflictError):
            self.reg.register_map_bundle(
                floor_id='fl-1',
                label='variant',
                yaml_bytes=_make_yaml_bytes(resolution=0.10, origin='[-0.5, -0.5, 0.0]'),
                pgm_bytes=self.sample_pgm,
                source_mode='IMPORTED_STATIC',
                posegraph_bytes=b'PG_A',
            )
        # Same pixels + label, different YAML threshold
        with self.assertRaises(ConflictError):
            self.reg.register_map_bundle(
                floor_id='fl-1',
                label='variant',
                yaml_bytes=_make_yaml_bytes(occupied_thresh=0.75, origin='[-0.5, -0.5, 0.0]'),
                pgm_bytes=self.sample_pgm,
                source_mode='IMPORTED_STATIC',
                posegraph_bytes=b'PG_A',
            )
        # Same pixels + label, different Posegraph bytes
        with self.assertRaises(ConflictError):
            self.reg.register_map_bundle(
                floor_id='fl-1',
                label='variant',
                yaml_bytes=self.sample_yaml,
                pgm_bytes=self.sample_pgm,
                source_mode='IMPORTED_STATIC',
                posegraph_bytes=b'PG_B',
            )
        # Same pixels + label, different floor_id
        with self.assertRaises(ConflictError):
            self.reg.register_map_bundle(
                floor_id='fl-2',
                label='variant',
                yaml_bytes=self.sample_yaml,
                pgm_bytes=self.sample_pgm,
                source_mode='IMPORTED_STATIC',
                posegraph_bytes=b'PG_A',
            )

    # ------------------------------------------------------------------
    # 12. Phase 1.5: SQLite immutability triggers & YAML/PGM bounds
    # ------------------------------------------------------------------

    def test_sqlite_immutability_triggers_and_input_limits(self):
        res = self.reg.register_map_bundle(
            floor_id='fl-1',
            label='locked-row',
            yaml_bytes=self.sample_yaml,
            pgm_bytes=self.sample_pgm,
        )
        mvid = res['map_version']['map_version_id']

        with self.assertRaises(sqlite3.IntegrityError):
            self.reg.x('UPDATE map_versions SET content_sha256=? WHERE map_version_id=?', ('0' * 64, mvid))
        with self.assertRaises(sqlite3.IntegrityError):
            self.reg.x('DELETE FROM map_versions WHERE map_version_id=?', (mvid,))

        # Duplicate YAML keys & control chars & oversized PGM dimensions rejected
        dup_yaml = self.sample_yaml + b'resolution: 0.10\n'
        with self.assertRaises(ValidationError):
            parse_ros_map_yaml(dup_yaml)
        with self.assertRaises(SecurityError):
            parse_ros_map_yaml(self.sample_yaml + b'\x07')
        with self.assertRaises(ValidationError):
            parse_pgm_p5(b'P5\n100000 100000\n255\n\xff')

    # ------------------------------------------------------------------
    # 13. Phase 2-A: Response size limits, bounded PNG cache & concurrency
    # ------------------------------------------------------------------

    def test_response_size_cap_png_cache_tamper_and_concurrency(self):
        res = self.reg.register_map_bundle(
            floor_id='fl-1',
            label='cache-test',
            yaml_bytes=self.sample_yaml,
            pgm_bytes=self.sample_pgm,
        )
        mvid = res['map_version']['map_version_id']
        viewer = Principal('viewer', 'viewer')

        # 1) Populate PNG cache and verify concurrent reads succeed cleanly
        results: list[tuple[int, bytes]] = []
        errors: list[Exception] = []
        lock = threading.Lock()

        def _worker():
            try:
                st, body, ct = self.reg.dispatch_readonly(
                    'GET',
                    f'/api/v1/map-registry/maps/{mvid.replace("#", "%23")}/preview.png',
                    viewer,
                )
                with lock:
                    results.append((st, body if ct == 'image/png' else b''))
            except Exception as exc:
                with lock:
                    errors.append(exc)

        threads = [threading.Thread(target=_worker) for _ in range(10)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(5)
        self.assertEqual(errors, [])
        self.assertEqual(len(results), 10)
        self.assertTrue(all(st == 200 and body.startswith(b'\x89PNG') for st, body in results))
        self.assertEqual(len({body for _, body in results}), 1)

        # 2) Tight HTTP response cap triggers 413 too_large on preview/bundle/facilities
        tight = MapRegistry(
            self.reg.db_path,
            self.reg.bundle_root,
            max_http_response_bytes=32,
        )
        try:
            st_p, body_p, _ = tight.dispatch_readonly(
                'GET',
                f'/api/v1/map-registry/maps/{mvid.replace("#", "%23")}/preview.png',
                viewer,
            )
            self.assertEqual(st_p, 413)
            self.assertEqual(body_p['error']['code'], 'too_large')

            st_f, body_f, _ = tight.dispatch_readonly(
                'GET',
                '/api/v1/map-registry/facilities',
                viewer,
            )
            self.assertEqual(st_f, 413)
            self.assertEqual(body_f['error']['code'], 'too_large')

            with self.assertRaises(PayloadTooLargeError):
                tight._enforce_response_limit(b'x' * 33, context='unit')
        finally:
            tight.close()

        # 3) Disk tampering MUST still return 500 bundle_integrity_error even after caching PNG
        bpath = Path(self.reg.bundles_dir) / res['map_version']['storage_key'] / 'map.pgm'
        os.chmod(bpath, 0o644)
        orig = bpath.read_bytes()
        bpath.write_bytes(orig[:-1] + bytes([orig[-1] ^ 0xFF]))
        st_bad, body_bad, _ = self.reg.dispatch_readonly(
            'GET',
            f'/api/v1/map-registry/maps/{mvid.replace("#", "%23")}/preview.png',
            viewer,
        )
        self.assertEqual(st_bad, 500)
        self.assertEqual(body_bad['error']['code'], 'bundle_integrity_error')

    def test_encoded_traversal_rejected_in_dispatch_readonly(self):
        viewer = Principal('viewer', 'viewer')
        for bad_uri in (
            '/api/v1/map-registry/maps/..%2Fsecret/meta',
            '/api/v1/map-registry/maps/..%5Csecret/meta',
            '/api/v1/map-registry/maps/id%2Fsub/preview.png',
        ):
            st, body, _ = self.reg.dispatch_readonly('GET', bad_uri, viewer)
            self.assertEqual(st, 403, bad_uri)
            self.assertEqual(body['error']['code'], 'security_violation')


if __name__ == '__main__':
    unittest.main(verbosity=2)
