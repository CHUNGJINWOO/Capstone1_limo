#!/usr/bin/env python3
"""Register the 2026-11-19 C103 lounge map (Depth + obstacle-memory navigation run) in an existing Map Registry.

Read-only on the source run directory. Idempotent: re-running with the same registry returns the same
map_version_id. The registry hierarchy (facility / building / floor / area) is operator-declared metadata;
the robot does not detect the building or floor.

Usage:
  python3 fleet_control/scripts/register_c103_map.py --data-dir <fleet data dir> [--run-dir <run dir>] [--dry-run]

`--data-dir` must contain (or will receive) `map_registry.db` and `map_store/`, i.e. the same layout used by
`python3 -m fleet_server --data-dir`. Point it at an isolated copy first; the field runtime DB is only changed
when the operator explicitly passes its path.
"""
import argparse
import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'fleet_control', 'server'))

from fleet_server.map_registry import MapRegistry, canonicalize_pgm_pixels, parse_ros_map_yaml  # noqa: E402

RUN_ID = '20261119_auto_map_depth'
DEFAULT_RUN_DIR = os.path.join(REPO, 'local_data', 'raw_results', 'limo_runs', RUN_ID)

FACILITY = ('paichai_univ', '배재대학교')
BUILDING = ('paichai_c', 'C관')
FLOOR = ('paichai_c_1f', '1F')
AREA = ('paichai_c_1f_c103_lounge', 'C103 인근 라운지')
LABEL = 'c103_lounge_20261119_depthmem'

# Hashes of the original run artifacts as recorded by the 2026-11-19 analysis (SHA-256 prefix check).
EXPECTED_PREFIX = {
    'map/map.pgm': '7db92ad2050364fe',
    'map/map.yaml': '1c6d98ca3cef70b1',
    'map/posegraph.posegraph': 'e03c404c1aeecce6',
    'map/posegraph.data': 'f6c342383119828c',
}


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def pgm_pixels(pgm_bytes):
    # Binary P5: magic, width, height, maxval separated by whitespace (comments not used by map_saver).
    tokens, pos = [], 0
    while len(tokens) < 4:
        while pgm_bytes[pos:pos + 1].isspace():
            pos += 1
        start = pos
        while not pgm_bytes[pos:pos + 1].isspace():
            pos += 1
        tokens.append(pgm_bytes[start:pos])
    w, h = int(tokens[1]), int(tokens[2])
    return w, h, pgm_bytes[pos + 1:pos + 1 + w * h]


def evidence(run_dir, hashes, counts, meta):
    return {
        'method': 'operator_declared_metadata_plus_run_artifacts',
        'run_id': RUN_ID,
        'run_dir_rel': os.path.relpath(run_dir, REPO),
        'captured_local_time': '2026-11-19T20:57:51+09:00/2026-11-19T21:06:17+09:00',
        'location': {'university': '배재대학교', 'building': 'C관', 'floor': '1층', 'area': 'C103 인근 라운지',
                     'coverage': 'PARTIAL_FLOOR', 'source': 'operator_declared', 'auto_detected': False},
        'sensors': {
            'slam_raster': {'lidar': True, 'depth': False, 'memory': False,
                            'basis': 'slam_toolbox scan_topic=/scan_slam (scan_normalizer); Depth is not a SLAM input'},
            'navigation_costmaps_during_mapping': {
                'lidar': True, 'depth': True, 'obstacle_memory': True,
                'basis': ['drive.launch.py use_depth_camera:=true depth_mode:=required',
                          'rosbag /depth_obstacles/points 1528 msgs, /collision_monitor/memory_points 10047 msgs',
                          'Gate depth_code DEPTH_READY 5139/5139'],
                'limitations': ['obstacle_memory_node.py source was deleted with /tmp after the run (not preserved)',
                                'depth height band referenced to base_footprint==base_link (field height unverified)'],
            },
        },
        'artifacts_sha256': hashes,
        'raster_posegraph_same_session': {
            'value': True,
            'basis': 'map_saver_cli then serialize_map in the same supervisor shutdown, robot stationary since +453 s',
        },
        'grid_counts_pgm': counts,
        'yaml': {k: meta[k] for k in ('resolution', 'origin', 'free_thresh', 'occupied_thresh', 'negate')},
        'quality': {
            'exploration': 'EXPLORATION_STOPPED (frontiers 114/29/16 cells remained; costmap enclosure at +451.8 s)',
            'geometry': 'lower half rotated ~5.7-6.6 deg vs upper half (RANSAC wall fit); no ground truth',
            'posegraph_nodes': 127,
        },
        'field_verification': 'UNVERIFIED (no localization or navigation trial on this map)',
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', required=True)
    ap.add_argument('--run-dir', default=DEFAULT_RUN_DIR)
    ap.add_argument('--actor', default='operator:c103-registration')
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()

    map_dir = os.path.join(a.run_dir, 'map')
    files = {k: os.path.join(a.run_dir, k) for k in EXPECTED_PREFIX}
    hashes = {}
    for rel, path in files.items():
        if not os.path.isfile(path) or os.path.islink(path):
            sys.exit(f'missing or symlinked artifact: {rel}')
        hashes[rel] = sha(path)
        if not hashes[rel].startswith(EXPECTED_PREFIX[rel]):
            sys.exit(f'hash mismatch for {rel}: {hashes[rel][:16]} != {EXPECTED_PREFIX[rel]}')

    yaml_bytes = open(files['map/map.yaml'], 'rb').read()
    pgm_bytes = open(files['map/map.pgm'], 'rb').read()
    meta = parse_ros_map_yaml(yaml_bytes, expected_image_name='map.pgm')
    w, h, pixels = pgm_pixels(pgm_bytes)
    raw = {v: pixels.count(bytes([v])) for v in (0, 205, 254)}
    if sum(raw.values()) != w * h:
        sys.exit(f'unexpected PGM values: only {sum(raw.values())}/{w*h} pixels are 0/205/254')
    canon = canonicalize_pgm_pixels(pixels)
    ccounts = {'occupied': canon.count(b'\x00'), 'unknown': canon.count(b'\x80'), 'free': canon.count(b'\xff')}
    if (ccounts['occupied'], ccounts['unknown'], ccounts['free']) != (raw[0], raw[205], raw[254]):
        sys.exit(f'canonical counts changed classes: raw={raw} canon={ccounts}')
    # ROS map_server trinary semantics: p=(255-v)/255; free if p < free_thresh, occupied if p > occupied_thresh.
    p_unknown = (255 - 205) / 255.0
    if not (meta['free_thresh'] <= p_unknown <= meta['occupied_thresh']):
        sys.exit(f'free_thresh {meta["free_thresh"]} would turn unknown pixels (p={p_unknown:.5f}) into free/occupied')
    counts = {'width': w, 'height': h, 'occupied': raw[0], 'unknown': raw[205], 'free': raw[254],
              'unknown_p': round(p_unknown, 5)}
    ev = evidence(a.run_dir, hashes, counts, meta)
    print(json.dumps({'counts': counts, 'yaml': ev['yaml']}, ensure_ascii=False))
    if a.dry_run:
        return 0

    os.makedirs(a.data_dir, exist_ok=True)
    reg = MapRegistry(os.path.join(a.data_dir, 'map_registry.db'), os.path.join(a.data_dir, 'map_store'))
    try:
        reg.create_facility(FACILITY[0], FACILITY[1], description='Operator-declared facility', actor=a.actor)
        reg.create_building(BUILDING[0], FACILITY[0], BUILDING[1], description='Operator-declared building', actor=a.actor)
        reg.create_floor(FLOOR[0], BUILDING[0], FLOOR[1], elevation_m=0.0, actor=a.actor)
        reg.create_area(AREA[0], FLOOR[0], AREA[1],
                        description='1층 전체가 아닌 C103 인근 라운지 부분 공간 (운영자 확인 메타데이터)',
                        coverage='PARTIAL_FLOOR', actor=a.actor)
        res = reg.register_map_bundle(
            floor_id=FLOOR[0], label=LABEL, yaml_bytes=yaml_bytes, pgm_bytes=pgm_bytes,
            posegraph_bytes=open(files['map/posegraph.posegraph'], 'rb').read(),
            posegraph_data_bytes=open(files['map/posegraph.data'], 'rb').read(),
            source_mode='SLAM_SAVED_BUNDLE', verification_status='UNVERIFIED', verification_evidence=ev,
            frame_id='map', actor=a.actor, actor_role='operator')
        mvid = res['map_version']['map_version_id']
        reg.link_map_version_area(mvid, AREA[0], actor=a.actor)
        reg.record_lineage(mvid, parent_map_version_id=None, mapping_mode='IMPORTED', session_id=RUN_ID, actor=a.actor)
        integrity = reg.verify_bundle_integrity(mvid)
        if not integrity.get('ok'):
            sys.exit(f'integrity check failed: {integrity}')
        mv = reg.get_map_version(mvid)
        print(json.dumps({'map_version_id': mvid, 'created': res['created'], 'integrity_ok': True,
                          'area': mv['area'], 'verification_status': mv['verification_status'],
                          'width': mv['width'], 'height': mv['height'], 'origin': mv['origin'],
                          'sha256': mv['sha256']}, ensure_ascii=False, indent=1))
    finally:
        reg.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
