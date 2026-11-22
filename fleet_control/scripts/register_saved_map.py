#!/usr/bin/env python3
"""Register a saved SLAM bundle (map.yaml/map.pgm/posegraph.data/posegraph.posegraph) as an UNVERIFIED map version.

Generic counterpart of register_c103_map.py (which stays unchanged). Location metadata is operator-declared; the robot
does not detect building/floor/area. Idempotent for the same registry and bundle. Read-only on the source map folder.

usage: register_saved_map.py --data-dir DATA --map-dir MAPDIR --sha256-file LIST \
         --facility-id ID --facility-name N --building-id ID --building-name N --floor-id ID --floor-name N \
         --area-id ID --area-name N [--coverage PARTIAL_FLOOR|FULL_FLOOR] --label L --session-id S \
         --captured 'start/end' --stationary --note TEXT [--dry-run]

--sha256-file: `sha256sum` output (file names relative to MAPDIR) taken from an independent copy (e.g. the robot);
every artifact must match. Refuses the operational field data dir unless --allow-field-data.
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

FIELD_DATA = os.path.expanduser('~/.local/share/robot-sim-projects/limo_field_20261113/data')
ARTIFACTS = ('map.pgm', 'map.yaml', 'posegraph.data', 'posegraph.posegraph')


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def pgm_pixels(b):
    tokens, pos = [], 0
    while len(tokens) < 4:
        while b[pos:pos + 1].isspace():
            pos += 1
        start = pos
        while not b[pos:pos + 1].isspace():
            pos += 1
        tokens.append(b[start:pos])
    w, h = int(tokens[1]), int(tokens[2])
    return w, h, b[pos + 1:pos + 1 + w * h]


def main():
    ap = argparse.ArgumentParser()
    for n in ('data-dir', 'map-dir', 'sha256-file', 'facility-id', 'facility-name', 'building-id', 'building-name', 'floor-id',
              'floor-name', 'area-id', 'area-name', 'label', 'session-id', 'captured', 'note'):
        ap.add_argument('--' + n, required=True)
    ap.add_argument('--coverage', default='PARTIAL_FLOOR', choices=['PARTIAL_FLOOR', 'FULL_FLOOR'])
    ap.add_argument('--mapping-mode', default='NEW_MAP')
    ap.add_argument('--fix-free-thresh', type=float, default=None,
                    help='register a DERIVED yaml with this free_thresh when the saved one would classify unknown pixels as free '
                         '(map_saver_cli defaults to 0.25; unknown trinary pixel p=0.196). Originals stay untouched; both hashes are recorded.')
    ap.add_argument('--stationary', action='store_true')
    ap.add_argument('--actor', default='operator:saved-map-registration')
    ap.add_argument('--allow-field-data', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()

    data = os.path.realpath(a.data_dir)
    if data == os.path.realpath(FIELD_DATA) and not a.allow_field_data:
        sys.exit('refusing the operational field data dir (needs --allow-field-data after approval and a fresh backup)')
    expected = {}
    for line in open(a.sha256_file):
        if '  ' in line:
            h, n = line.rstrip('\n').split('  ', 1)
            expected[n.lstrip('*')] = h
    hashes = {}
    for name in ARTIFACTS:
        p = os.path.join(a.map_dir, name)
        if not os.path.isfile(p) or os.path.islink(p):
            sys.exit(f'missing or symlinked artifact: {name}')
        hashes['map/' + name] = sha(p)
        if expected.get(name) != hashes['map/' + name]:
            sys.exit(f'hash mismatch against {a.sha256_file}: {name}')

    yaml_bytes = open(os.path.join(a.map_dir, 'map.yaml'), 'rb').read()
    pgm_bytes = open(os.path.join(a.map_dir, 'map.pgm'), 'rb').read()
    meta = parse_ros_map_yaml(yaml_bytes, expected_image_name='map.pgm')
    yaml_adjustment = None
    if a.fix_free_thresh is not None and meta['free_thresh'] > (255 - 205) / 255.0:
        old_line = f"free_thresh: {meta['free_thresh']}".encode()
        lines = yaml_bytes.split(b'\n')
        hit = [i for i, l in enumerate(lines) if l.strip().startswith(b'free_thresh:')]
        if len(hit) != 1:
            sys.exit('cannot locate a single free_thresh line in map.yaml')
        lines[hit[0]] = f'free_thresh: {a.fix_free_thresh}'.encode()
        derived = b'\n'.join(lines)
        yaml_adjustment = {'original_yaml_sha256': hashlib.sha256(yaml_bytes).hexdigest(),
                           'derived_yaml_sha256': hashlib.sha256(derived).hexdigest(),
                           'free_thresh_from': meta['free_thresh'], 'free_thresh_to': a.fix_free_thresh,
                           'reason': 'saved free_thresh would classify unknown pixels (p=0.196) as free in ROS map_server'}
        yaml_bytes = derived
        meta = parse_ros_map_yaml(yaml_bytes, expected_image_name='map.pgm')
    w, h, px = pgm_pixels(pgm_bytes)
    raw = {v: px.count(bytes([v])) for v in (0, 205, 254)}
    if sum(raw.values()) != w * h:
        sys.exit('unexpected PGM values (not trinary 0/205/254)')
    canon = canonicalize_pgm_pixels(px)
    if (canon.count(b'\x00'), canon.count(b'\x80'), canon.count(b'\xff')) != (raw[0], raw[205], raw[254]):
        sys.exit('canonical counts changed classes')
    p_unknown = (255 - 205) / 255.0
    if not (meta['free_thresh'] <= p_unknown <= meta['occupied_thresh']):
        sys.exit('thresholds would turn unknown pixels into free/occupied')
    known = raw[0] + raw[254]
    counts = {'width': w, 'height': h, 'occupied': raw[0], 'unknown': raw[205], 'free': raw[254],
              'known_cells': known, 'known_area_m2': round(known * meta['resolution'] ** 2, 2), 'unknown_p': round(p_unknown, 5)}
    ev = {
        'method': 'operator_declared_metadata_plus_run_artifacts',
        'session_id': a.session_id,
        'map_dir_rel': os.path.relpath(a.map_dir, REPO),
        'captured_local_time': a.captured,
        'location': {'university': a.facility_name, 'building': a.building_name, 'floor': a.floor_name, 'area': a.area_name,
                     'coverage': a.coverage, 'source': 'operator_declared', 'auto_detected': False},
        'sensors': {'slam_raster': {'lidar': True, 'depth': False, 'memory': False,
                                    'basis': 'slam_toolbox scan_topic=/scan_slam (scan_normalizer:=true); Depth is not a SLAM input'}},
        'artifacts_sha256': hashes,
        'yaml_adjustment': yaml_adjustment,
        'raster_posegraph_same_session': {'value': True, 'basis': 'map_saver_cli then serialize_map within seconds, same running slam_toolbox'},
        'grid_counts_pgm': counts,
        'yaml': {k: meta[k] for k in ('resolution', 'origin', 'free_thresh', 'occupied_thresh', 'negate')},
        'quality': {'robot_motion': 'stationary during the whole session (no motion command was sent)' if a.stationary else 'unknown',
                    'note': a.note},
        'field_verification': 'UNVERIFIED (no localization or navigation trial on this map)',
    }
    print(json.dumps({'counts': counts, 'yaml': ev['yaml']}, ensure_ascii=False))
    if a.dry_run:
        return 0

    os.makedirs(data, exist_ok=True)
    reg = MapRegistry(os.path.join(data, 'map_registry.db'), os.path.join(data, 'map_store'))
    try:
        reg.create_facility(a.facility_id, a.facility_name, description='Operator-declared facility', actor=a.actor)
        reg.create_building(a.building_id, a.facility_id, a.building_name, description='Operator-declared building', actor=a.actor)
        reg.create_floor(a.floor_id, a.building_id, a.floor_name, elevation_m=0.0, actor=a.actor)
        reg.create_area(a.area_id, a.floor_id, a.area_name, description=f'{a.area_name} (운영자 확인 메타데이터)',
                        coverage=a.coverage, actor=a.actor)
        res = reg.register_map_bundle(
            floor_id=a.floor_id, label=a.label, yaml_bytes=yaml_bytes, pgm_bytes=pgm_bytes,
            posegraph_bytes=open(os.path.join(a.map_dir, 'posegraph.posegraph'), 'rb').read(),
            posegraph_data_bytes=open(os.path.join(a.map_dir, 'posegraph.data'), 'rb').read(),
            source_mode='SLAM_SAVED_BUNDLE', verification_status='UNVERIFIED', verification_evidence=ev,
            frame_id='map', actor=a.actor, actor_role='operator')
        mvid = res['map_version']['map_version_id']
        reg.link_map_version_area(mvid, a.area_id, actor=a.actor)
        reg.record_lineage(mvid, parent_map_version_id=None, mapping_mode=a.mapping_mode, session_id=a.session_id, actor=a.actor)
        integrity = reg.verify_bundle_integrity(mvid)
        if not integrity.get('ok'):
            sys.exit(f'integrity check failed: {integrity}')
        mv = reg.get_map_version(mvid)
        print(json.dumps({'map_version_id': mvid, 'created': res['created'], 'integrity_ok': True, 'area': mv['area'],
                          'verification_status': mv['verification_status'], 'width': mv['width'], 'height': mv['height'],
                          'origin': mv['origin']}, ensure_ascii=False, indent=1))
    finally:
        reg.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
