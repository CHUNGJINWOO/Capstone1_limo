"""Map Registry Phase 1 — standalone immutable map bundle store & metadata registry.

Stdlib-only core designed for Capstone1_limo Fleet Server integration without
modifying shared Safety Context v1 / Fleet Server files during Phase 1.

Design policies enforced:
1. MapVersion content/files/checksums are strictly immutable once registered.
2. Floor activation (`floor_active_maps`) is a separate mutable pointer decoupled
   from immutable MapVersion records.
3. Normal live SLAM `/map` updates are classified distinctly from `map_epoch`
   discontinuities/switches.
4. Open-RMF `transform_rmf_from_nav2` and NavGraph waypoint/lane geometry can be
   validated against the canonical occupancy grid (`free=255, occupied=0, unknown=128`).
5. Registering a map version never switches any robot's active map and never grants
   movement/mission authorization.
6. Unverified maps (including live SLAM snapshots) cannot be marked `VERIFIED`
   or activated as a floor map.
"""

from __future__ import annotations

import base64
from collections import OrderedDict
import hashlib
import json
import math
import os
import re
import shutil
import sqlite3
import struct
import threading
import time
import urllib.parse
import uuid
import zlib
from typing import Any


CANON_OCCUPIED = 0
CANON_UNKNOWN = 128
CANON_FREE = 255

_PGM_TO_CANON_TABLE = bytes(
    CANON_OCCUPIED if v == 0 else (CANON_FREE if v >= 250 else CANON_UNKNOWN)
    for v in range(256)
)

SOURCE_MODES = frozenset({
    'SLAM_LIVE_SNAPSHOT',
    'SLAM_SAVED_BUNDLE',
    'IMPORTED_STATIC',
    'SIM_FIXTURE',
})
VERIFICATION_STATUSES = frozenset({'UNVERIFIED', 'VERIFIED', 'REJECTED', 'REVOKED'})
QUARANTINE_STATUSES = frozenset({'NONE', 'ACTIVE', 'CLEARED'})
OPERATION_MODES = frozenset({'IDLE', 'MAPPING', 'NAVIGATION'})

BUNDLE_FILES_ALLOWED = frozenset({
    'map.yaml',
    'map.pgm',
    'map.posegraph',
    'map.posegraph.data',
})

_ENTITY_ID_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$')
_MAP_VERSION_ID_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._:@#-]{0,127}$')
_FRAME_ID_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._/-]{0,63}$')
_IDEMPOTENCY_KEY_RE = re.compile(r'^[A-Za-z0-9._:-]{1,128}$')
_UNSAFE_YAML_TOKEN_RE = re.compile(r'(^|\s)(---|\.\.\.|![^\s]+|&[^\s]+|\*[^\s]+)')

MAX_YAML_BYTES = 256 * 1024
MAX_PGM_BYTES = 32 * 1024 * 1024
MAX_PGM_PIXELS = 16_777_216
MAX_POSEGRAPH_BYTES = 128 * 1024 * 1024
MAX_HTTP_WRITE_BODY_BYTES = 48 * 1024 * 1024
DEFAULT_MAX_HTTP_RESPONSE_BYTES = 16 * 1024 * 1024
DEFAULT_PNG_RENDER_CONCURRENCY = 4
DEFAULT_PNG_CACHE_MAX_ITEMS = 8
DEFAULT_PNG_CACHE_MAX_BYTES = 8 * 1024 * 1024
ORIGIN_TOL_M = 0.026
ORIGIN_YAW_STRICT_TOL_RAD = 1e-6


class MapRegistryError(Exception):
    """Base error with explicit HTTP status and machine-readable code matching http_api.py."""

    status = 400
    code = 'map_registry_error'

    def __init__(self, message: str, detail: dict[str, Any] | None = None):
        super().__init__(message)
        self.message = message
        self.detail = detail or {}

    def to_dict(self) -> dict[str, Any]:
        err: dict[str, Any] = {'code': self.code, 'message': self.message}
        if self.detail:
            err['detail'] = self.detail
        return {'error': err}


class ValidationError(MapRegistryError):
    status = 400
    code = 'validation_error'


class SecurityError(MapRegistryError):
    status = 403
    code = 'security_violation'


class NotFoundError(MapRegistryError):
    status = 404
    code = 'not_found'


class ConflictError(MapRegistryError):
    status = 409
    code = 'conflict'


class ChecksumError(MapRegistryError):
    status = 422
    code = 'checksum_mismatch'


class PolicyError(MapRegistryError):
    status = 422
    code = 'policy_violation'


class PayloadTooLargeError(MapRegistryError):
    status = 413
    code = 'too_large'


class IntegrityFailureError(MapRegistryError):
    status = 500
    code = 'bundle_integrity_error'


SCHEMA = """
PRAGMA foreign_keys = ON;
CREATE TABLE IF NOT EXISTS facilities (
    facility_id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    timezone TEXT NOT NULL,
    description TEXT NOT NULL,
    created_at REAL NOT NULL,
    updated_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS buildings (
    building_id TEXT PRIMARY KEY,
    facility_id TEXT NOT NULL REFERENCES facilities(facility_id) ON DELETE RESTRICT,
    name TEXT NOT NULL,
    description TEXT NOT NULL,
    created_at REAL NOT NULL,
    updated_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS floors (
    floor_id TEXT PRIMARY KEY,
    building_id TEXT NOT NULL REFERENCES buildings(building_id) ON DELETE RESTRICT,
    level_name TEXT NOT NULL,
    elevation_m REAL NOT NULL,
    rmf_map_name TEXT NOT NULL,
    created_at REAL NOT NULL,
    updated_at REAL NOT NULL,
    UNIQUE(building_id, level_name)
);
CREATE TABLE IF NOT EXISTS map_versions (
    map_version_id TEXT PRIMARY KEY,
    floor_id TEXT NOT NULL REFERENCES floors(floor_id) ON DELETE RESTRICT,
    label TEXT NOT NULL,
    source_mode TEXT NOT NULL,
    verification_status TEXT NOT NULL,
    verification_evidence_json TEXT NOT NULL,
    storage_key TEXT NOT NULL UNIQUE,
    width INTEGER NOT NULL,
    height INTEGER NOT NULL,
    resolution REAL NOT NULL,
    origin_x REAL NOT NULL,
    origin_y REAL NOT NULL,
    origin_yaw REAL NOT NULL,
    negate INTEGER NOT NULL,
    occupied_thresh REAL NOT NULL,
    free_thresh REAL NOT NULL,
    content_sha256 TEXT NOT NULL,
    yaml_sha256 TEXT NOT NULL,
    pgm_sha256 TEXT NOT NULL,
    posegraph_sha256 TEXT,
    posegraph_data_sha256 TEXT,
    frame_id TEXT NOT NULL DEFAULT 'map',
    bundle_sha256 TEXT NOT NULL DEFAULT '',
    rotated_map_frame INTEGER NOT NULL,
    rmf_compatibility_json TEXT NOT NULL,
    created_by TEXT NOT NULL,
    created_at REAL NOT NULL,
    verified_by TEXT,
    verified_at REAL
);
CREATE INDEX IF NOT EXISTS idx_map_versions_floor
    ON map_versions(floor_id, created_at DESC);
CREATE INDEX IF NOT EXISTS idx_map_versions_content_sha
    ON map_versions(content_sha256);
CREATE TABLE IF NOT EXISTS floor_active_maps (
    floor_id TEXT PRIMARY KEY REFERENCES floors(floor_id) ON DELETE RESTRICT,
    active_map_version_id TEXT NOT NULL REFERENCES map_versions(map_version_id) ON DELETE RESTRICT,
    pointer_revision INTEGER NOT NULL,
    activated_by TEXT NOT NULL,
    activated_at REAL NOT NULL,
    reason TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS robot_map_bindings (
    robot_id TEXT PRIMARY KEY,
    target_floor_id TEXT REFERENCES floors(floor_id) ON DELETE RESTRICT,
    target_map_version_id TEXT REFERENCES map_versions(map_version_id) ON DELETE RESTRICT,
    binding_revision INTEGER NOT NULL,
    assigned_by TEXT NOT NULL,
    assigned_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS map_registry_events (
    seq INTEGER PRIMARY KEY AUTOINCREMENT,
    ts REAL NOT NULL,
    actor TEXT NOT NULL,
    event_type TEXT NOT NULL,
    entity_id TEXT NOT NULL,
    detail_json TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS map_verification_history (
    seq INTEGER PRIMARY KEY AUTOINCREMENT,
    ts REAL NOT NULL,
    map_version_id TEXT NOT NULL REFERENCES map_versions(map_version_id) ON DELETE RESTRICT,
    from_status TEXT NOT NULL,
    to_status TEXT NOT NULL,
    actor TEXT NOT NULL,
    actor_role TEXT NOT NULL,
    reason TEXT NOT NULL,
    evidence_json TEXT NOT NULL,
    rmf_compatibility_json TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_map_ver_hist_mvid
    ON map_verification_history(map_version_id, seq DESC);
CREATE TRIGGER IF NOT EXISTS trg_map_verification_history_no_update
BEFORE UPDATE ON map_verification_history
BEGIN
    SELECT RAISE(ABORT, 'map_verification_history is append-only');
END;
CREATE TRIGGER IF NOT EXISTS trg_map_verification_history_no_delete
BEFORE DELETE ON map_verification_history
BEGIN
    SELECT RAISE(ABORT, 'map_verification_history is append-only');
END;
CREATE TABLE IF NOT EXISTS map_quarantines (
    map_version_id TEXT PRIMARY KEY REFERENCES map_versions(map_version_id) ON DELETE RESTRICT,
    status TEXT NOT NULL,
    reason TEXT NOT NULL,
    detail_json TEXT NOT NULL,
    quarantined_by TEXT NOT NULL,
    quarantined_at REAL NOT NULL,
    cleared_by TEXT,
    cleared_reason TEXT,
    cleared_at REAL
);
CREATE TABLE IF NOT EXISTS map_write_idempotency (
    scope_key TEXT PRIMARY KEY,
    method TEXT NOT NULL,
    route TEXT NOT NULL,
    actor TEXT NOT NULL,
    request_sha256 TEXT NOT NULL,
    status_code INTEGER NOT NULL,
    response_json TEXT NOT NULL,
    created_at REAL NOT NULL
);
CREATE TRIGGER IF NOT EXISTS trg_map_versions_no_delete
BEFORE DELETE ON map_versions
BEGIN
    SELECT RAISE(ABORT, 'map_versions rows are immutable and cannot be deleted');
END;
-- Optional operator-declared sub-floor area (e.g. a lounge mapped as a partial floor).
-- Additive only: map_versions rows are never altered; the area link is write-once.
CREATE TABLE IF NOT EXISTS map_areas (
    area_id TEXT PRIMARY KEY,
    floor_id TEXT NOT NULL REFERENCES floors(floor_id) ON DELETE RESTRICT,
    name TEXT NOT NULL,
    description TEXT NOT NULL,
    coverage TEXT NOT NULL,
    created_at REAL NOT NULL,
    updated_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS map_version_areas (
    map_version_id TEXT PRIMARY KEY REFERENCES map_versions(map_version_id) ON DELETE RESTRICT,
    area_id TEXT NOT NULL REFERENCES map_areas(area_id) ON DELETE RESTRICT,
    linked_by TEXT NOT NULL,
    linked_at REAL NOT NULL
);
CREATE TRIGGER IF NOT EXISTS trg_map_version_areas_no_update
BEFORE UPDATE ON map_version_areas
BEGIN
    SELECT RAISE(ABORT, 'map_version_areas links are write-once');
END;
CREATE TRIGGER IF NOT EXISTS trg_map_version_areas_no_delete
BEFORE DELETE ON map_version_areas
BEGIN
    SELECT RAISE(ABORT, 'map_version_areas links are write-once');
END;
-- Write-once provenance: which earlier version a map continued from and how it was produced.
CREATE TABLE IF NOT EXISTS map_version_lineage (
    map_version_id TEXT PRIMARY KEY REFERENCES map_versions(map_version_id) ON DELETE RESTRICT,
    parent_map_version_id TEXT REFERENCES map_versions(map_version_id) ON DELETE RESTRICT,
    mapping_mode TEXT NOT NULL,
    session_id TEXT NOT NULL,
    recorded_by TEXT NOT NULL,
    recorded_at REAL NOT NULL
);
CREATE TRIGGER IF NOT EXISTS trg_map_version_lineage_no_update
BEFORE UPDATE ON map_version_lineage
BEGIN
    SELECT RAISE(ABORT, 'map_version_lineage is write-once');
END;
CREATE TRIGGER IF NOT EXISTS trg_map_version_lineage_no_delete
BEFORE DELETE ON map_version_lineage
BEGIN
    SELECT RAISE(ABORT, 'map_version_lineage is write-once');
END;
"""

AREA_COVERAGES = frozenset({'PARTIAL_FLOOR', 'FULL_FLOOR'})
MAPPING_MODES = frozenset({'NEW_MAP', 'CONTINUE_MAP', 'IMPORTED'})


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def validate_entity_id(value: Any, field: str) -> str:
    """Validate hierarchy/label identifiers without path separators or traversal."""
    if not isinstance(value, str) or not value:
        raise ValidationError(f'{field} must be a non-empty string')
    if '..' in value or '/' in value or '\\' in value or '\x00' in value:
        raise SecurityError(f'{field} contains unsafe path characters', {'field': field})
    if not _ENTITY_ID_RE.match(value):
        raise ValidationError(
            f'{field} must match {_ENTITY_ID_RE.pattern}',
            {'field': field, 'value': value},
        )
    return value


def validate_map_version_id(value: Any) -> str:
    """Validate immutable map_version_id (e.g., `label#sha12` or `building:L1:label#sha12`)."""
    if not isinstance(value, str) or not value:
        raise ValidationError('map_version_id must be a non-empty string')
    if '..' in value or '/' in value or '\\' in value or '\x00' in value:
        raise SecurityError('map_version_id contains unsafe path characters')
    if not _MAP_VERSION_ID_RE.match(value):
        raise ValidationError(
            f'map_version_id must match {_MAP_VERSION_ID_RE.pattern}',
            {'value': value},
        )
    return value


def validate_safe_filename(filename: Any, *, allowset: frozenset[str] = BUNDLE_FILES_ALLOWED) -> str:
    """Reject traversal, separators, null bytes, or unknown bundle filenames."""
    if not isinstance(filename, str) or not filename:
        raise SecurityError('bundle filename must be a non-empty string')
    if '\x00' in filename or '/' in filename or '\\' in filename or '..' in filename:
        raise SecurityError('unsafe bundle filename rejected', {'filename': filename})
    if os.path.isabs(filename) or os.path.basename(filename) != filename:
        raise SecurityError('bundle filename must be a plain basename', {'filename': filename})
    if filename not in allowset:
        raise SecurityError('unsupported bundle filename', {'filename': filename})
    return filename


def _safe_contained_path(base_dir: str, rel_name: str) -> str:
    """Resolve `rel_name` inside `base_dir` and reject symlinks or escapes."""
    safe_name = validate_safe_filename(rel_name)
    base_real = os.path.realpath(base_dir)
    candidate_lex = os.path.join(base_dir, safe_name)
    if os.path.islink(candidate_lex):
        raise SecurityError('symlink files are forbidden inside bundle directory', {'filename': safe_name})
    candidate_real = os.path.realpath(candidate_lex)
    if os.path.dirname(candidate_real) != base_real or os.path.basename(candidate_real) != safe_name:
        raise SecurityError('resolved path escapes bundle root', {'filename': safe_name})
    return candidate_real


def canonicalize_pgm_pixels(pixel_bytes: bytes) -> bytes:
    """Convert raw PGM uint8 pixels to canonical bytes matching `map_identity.py`.

    Canonical encoding: occupied=0, free=255 (>=250 in PGM), unknown=128 (everything else).
    """
    return pixel_bytes.translate(_PGM_TO_CANON_TABLE)


def canonical_to_png_bytes(width: int, height: int, canon_bytes: bytes) -> bytes:
    """Encode 8-bit grayscale canonical occupancy grid into PNG using stdlib zlib/struct."""
    if width <= 0 or height <= 0 or len(canon_bytes) != width * height:
        raise ValidationError('canonical PNG dimensions do not match byte buffer')

    def _chunk(ctype: bytes, payload: bytes) -> bytes:
        body = ctype + payload
        return struct.pack('>I', len(payload)) + body + struct.pack('>I', zlib.crc32(body) & 0xFFFFFFFF)

    raw_scanlines = b''.join(
        b'\x00' + canon_bytes[r * width : (r + 1) * width]
        for r in range(height)
    )
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    return b'\x89PNG\r\n\x1a\n' + _chunk(b'IHDR', ihdr) + _chunk(b'IDAT', zlib.compress(raw_scanlines, 6)) + _chunk(b'IEND', b'')


def compute_content_sha256(width: int, height: int, canon_bytes: bytes) -> str:
    """Match `experiments/real_rmf_20261113/limo_rmf_real/map_identity.py:content_sha256`."""
    expected_len = width * height
    if len(canon_bytes) != expected_len:
        raise ValidationError(
            'canonical pixel byte count does not match width * height',
            {'expected': expected_len, 'actual': len(canon_bytes)},
        )
    header = f'{height}x{width}:'.encode('ascii')
    return hashlib.sha256(header + canon_bytes).hexdigest()


def make_map_version_id(label: str, content_hash: str) -> str:
    """Build deterministic map_version_id `<label>#<content_sha256[:12]>`."""
    clean_label = validate_entity_id(label, 'label')
    if not isinstance(content_hash, str) or len(content_hash) < 12:
        raise ValidationError('content_hash must be at least 12 hex characters')
    return f'{clean_label}#{content_hash[:12]}'


def validate_frame_id(frame_id: Any, *, field: str = 'frame_id') -> str:
    """Validate ROS coordinate frame identifier (e.g. `map`, `robot1/map`)."""
    if not isinstance(frame_id, str):
        raise ValidationError(f'{field} must be a string')
    val = frame_id.strip()
    if not val or '..' in val or '//' in val or val.startswith('/') or val.endswith('/') or '\\' in val:
        raise SecurityError(f'unsafe {field} rejected', {field: frame_id})
    if not _FRAME_ID_RE.fullmatch(val):
        raise ValidationError(f'invalid {field}', {field: frame_id})
    return val


def validate_idempotency_key(key: Any) -> str:
    """Validate client write idempotency_key token."""
    if not isinstance(key, str) or not _IDEMPOTENCY_KEY_RE.fullmatch(key):
        raise ValidationError('idempotency_key must match [A-Za-z0-9._:-]{1,128}')
    return key


def compute_bundle_sha256(
    *,
    content_sha256: str,
    yaml_sha256: str,
    pgm_sha256: str,
    posegraph_sha256: str | None,
    posegraph_data_sha256: str | None,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    negate: int,
    occupied_thresh: float,
    free_thresh: float,
    frame_id: str,
) -> str:
    """Compute canonical bundle identity SHA-256 separating pixels (`content_sha256`) from geometry/YAML/posegraph."""
    manifest = {
        'content_sha256': content_sha256,
        'yaml_sha256': yaml_sha256,
        'pgm_sha256': pgm_sha256,
        'posegraph_sha256': posegraph_sha256 or '',
        'posegraph_data_sha256': posegraph_data_sha256 or '',
        'width': int(width),
        'height': int(height),
        'resolution': float(resolution),
        'origin': [float(origin_x), float(origin_y), float(origin_yaw)],
        'negate': int(negate),
        'occupied_thresh': float(occupied_thresh),
        'free_thresh': float(free_thresh),
        'frame_id': str(frame_id),
    }
    encoded = json.dumps(manifest, sort_keys=True, separators=(',', ':')).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _decode_b64_limited(raw_b64: Any, *, field: str, max_bytes: int) -> bytes:
    """Reject oversized base64 strings BEFORE decoding, then decode strictly and re-check raw length."""
    if not isinstance(raw_b64, str) or not raw_b64:
        raise ValidationError(f'{field} must be a non-empty base64 string')
    max_b64_chars = ((max_bytes + 2) // 3) * 4 + 16
    if len(raw_b64) > max_b64_chars:
        raise PayloadTooLargeError(
            f'{field} exceeds size limit before decode',
            {'field': field, 'max_bytes': max_bytes},
        )
    try:
        decoded = base64.b64decode(raw_b64.encode('ascii'), validate=True)
    except Exception as exc:
        raise ValidationError(f'invalid base64 encoding in {field}') from exc
    if len(decoded) > max_bytes:
        raise PayloadTooLargeError(
            f'{field} decoded payload exceeds size limit',
            {'field': field, 'bytes': len(decoded), 'max_bytes': max_bytes},
        )
    return decoded


def parse_pgm_p5(pgm_bytes: bytes, *, max_pixels: int = MAX_PGM_PIXELS) -> dict[str, Any]:
    """Strictly parse binary PGM (P5) bytes without external dependencies.

    Enforces exact byte count `width * height`, maxval `1..255`, and rejects
    truncated payloads or trailing garbage bytes.
    """
    if not isinstance(pgm_bytes, (bytes, bytearray)) or len(pgm_bytes) == 0:
        raise ValidationError('map.pgm must be non-empty bytes')
    if len(pgm_bytes) > MAX_PGM_BYTES:
        raise ValidationError('map.pgm exceeds size limit', {'max_bytes': MAX_PGM_BYTES})

    data = bytes(pgm_bytes)
    n = len(data)
    pos = 0

    def _next_token() -> bytes:
        nonlocal pos
        while pos < n:
            b = data[pos]
            if b in (9, 10, 12, 13, 32):  # tab, lf, ff, cr, space
                pos += 1
            elif b == 35:  # '#'
                nl = data.find(b'\n', pos)
                pos = n if nl < 0 else nl + 1
            else:
                break
        if pos >= n:
            raise ValidationError('unexpected end of PGM header')
        start = pos
        while pos < n and data[pos] not in (9, 10, 12, 13, 32, 35):
            pos += 1
        return data[start:pos]

    magic = _next_token()
    if magic != b'P5':
        raise ValidationError('map.pgm must use binary P5 format', {'magic': magic.decode('latin1', 'replace')})
    try:
        width = int(_next_token())
        height = int(_next_token())
        maxval = int(_next_token())
    except ValueError as exc:
        raise ValidationError('invalid integer in PGM header') from exc

    if width <= 0 or height <= 0:
        raise ValidationError('PGM width and height must be positive', {'width': width, 'height': height})
    if width * height > max_pixels:
        raise ValidationError('PGM pixel count exceeds limit', {'pixels': width * height, 'max_pixels': max_pixels})
    if not (1 <= maxval <= 255):
        raise ValidationError('PGM maxval must be in 1..255 (8-bit gray)', {'maxval': maxval})

    if pos >= n or data[pos] not in (9, 10, 12, 13, 32):
        raise ValidationError('missing single whitespace delimiter after PGM maxval')
    pos += 1

    expected_pixels = width * height
    pixel_bytes = data[pos:]
    if len(pixel_bytes) != expected_pixels:
        raise ValidationError(
            'PGM pixel payload length mismatch',
            {'expected': expected_pixels, 'actual': len(pixel_bytes)},
        )

    canon_bytes = canonicalize_pgm_pixels(pixel_bytes)
    content_hash = compute_content_sha256(width, height, canon_bytes)
    return {
        'width': width,
        'height': height,
        'maxval': maxval,
        'pixel_bytes': pixel_bytes,
        'canonical_bytes': canon_bytes,
        'content_sha256': content_hash,
        'pgm_sha256': _sha256_bytes(data),
    }


def _strip_yaml_comment(line: str) -> str:
    in_single = False
    in_double = False
    for idx, ch in enumerate(line):
        if ch == "'" and not in_double:
            in_single = not in_single
        elif ch == '"' and not in_single:
            in_double = not in_double
        elif ch == '#' and not in_single and not in_double:
            return line[:idx]
    return line


def _parse_yaml_scalar(raw: str) -> Any:
    val = raw.strip()
    if not val:
        raise ValidationError('empty YAML value')
    if _UNSAFE_YAML_TOKEN_RE.search(val):
        raise SecurityError('unsupported or unsafe YAML construct')
    if (val.startswith('"') and val.endswith('"')) or (val.startswith("'") and val.endswith("'")):
        if len(val) < 2:
            raise ValidationError('unterminated quoted string in YAML')
        return val[1:-1]
    if val.startswith('[') and val.endswith(']'):
        inner = val[1:-1].strip()
        if not inner:
            return []
        return [_parse_yaml_scalar(item) for item in inner.split(',')]
    lower = val.lower()
    if lower in ('true', 'yes'):
        return True
    if lower in ('false', 'no'):
        return False
    if lower in ('null', '~', '.nan', 'nan', 'inf', '+inf', '-inf', '.inf', '+.inf', '-.inf'):
        raise ValidationError('non-finite or null scalar rejected in map.yaml', {'raw': val})
    try:
        if re.fullmatch(r'[+-]?\d+', val):
            return int(val)
        f = float(val)
        if not math.isfinite(f):
            raise ValidationError('non-finite float in map.yaml', {'raw': val})
        return f
    except ValueError:
        return val


THRESHOLD_POLICIES = frozenset({'reject', 'derive'})
_UNKNOWN_PIXEL = 205  # ROS map_saver_cli trinary value for "unknown"


def unknown_threshold_issue(yaml_meta: dict[str, Any], pgm_info: dict[str, Any]) -> dict[str, Any] | None:
    """Would ROS map_server read this trinary map's UNKNOWN pixels (205) as free or occupied?

    map_saver_cli writes `free_thresh: 0.25`, but an unknown pixel has occupancy probability (255-205)/255 = 0.196 (negate 0), so
    0.25 turns every unknown cell into free space. Only trinary PGMs (all pixels 0/205/254) are judged; others return None.
    """
    px = pgm_info['pixel_bytes']
    counts = {v: px.count(bytes([v])) for v in (0, _UNKNOWN_PIXEL, 254)}
    if sum(counts.values()) != len(px):
        return None
    negate = int(yaml_meta.get('negate', 0))
    p_unknown = (_UNKNOWN_PIXEL / 255.0) if negate else ((255 - _UNKNOWN_PIXEL) / 255.0)
    free_t, occ_t = float(yaml_meta['free_thresh']), float(yaml_meta['occupied_thresh'])
    if free_t <= p_unknown <= occ_t:
        return None
    return {'p_unknown': round(p_unknown, 5), 'free_thresh': free_t, 'occupied_thresh': occ_t, 'negate': negate,
            'unknown_cells': counts[_UNKNOWN_PIXEL], 'fixable_by_free_thresh': negate == 0 and free_t > p_unknown and p_unknown <= occ_t,
            'derived_free_thresh': math.floor(((255 - _UNKNOWN_PIXEL) / 255.0) * 1000) / 1000 if negate == 0 else None}


def derive_yaml_free_thresh(yaml_bytes: bytes, new_value: float) -> bytes:
    """Return a copy of map.yaml with only the free_thresh line replaced (original bytes are never modified)."""
    lines = bytes(yaml_bytes).split(b'\n')
    hit = [i for i, l in enumerate(lines) if l.strip().startswith(b'free_thresh:')]
    if len(hit) != 1:
        raise ValidationError('map.yaml needs exactly one free_thresh line to derive a corrected copy')
    lines[hit[0]] = f'free_thresh: {new_value}'.encode()
    return b'\n'.join(lines)



def parse_ros_map_yaml(yaml_bytes: bytes, *, expected_image_name: str = 'map.pgm') -> dict[str, Any]:
    """Parse and strictly validate a Nav2 `map.yaml` payload in pure stdlib.

    Accepts standard flat key-value `map.yaml` (plus inline `[x, y, yaw]` lists).
    Rejects path traversal in `image:`, non-finite numbers, invalid thresholds,
    and unsafe YAML directives/tags/anchors.
    """
    if not isinstance(yaml_bytes, (bytes, bytearray)) or len(yaml_bytes) == 0:
        raise ValidationError('map.yaml must be non-empty bytes')
    if len(yaml_bytes) > MAX_YAML_BYTES:
        raise ValidationError('map.yaml exceeds size limit', {'max_bytes': MAX_YAML_BYTES})
    try:
        text = bytes(yaml_bytes).decode('utf-8')
    except UnicodeDecodeError as exc:
        raise ValidationError('map.yaml must be valid UTF-8') from exc
    if '\x00' in text or any(ord(ch) < 32 and ch not in ('\t', '\r', '\n') for ch in text):
        raise SecurityError('map.yaml contains disallowed control characters')

    parsed: dict[str, Any] = {}
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        cleaned = _strip_yaml_comment(lines[i]).strip()
        i += 1
        if not cleaned:
            continue
        if _UNSAFE_YAML_TOKEN_RE.search(cleaned):
            raise SecurityError('unsafe YAML directive/tag/anchor rejected')
        if ':' not in cleaned:
            raise ValidationError('malformed line in map.yaml', {'line': cleaned})
        key, rest = cleaned.split(':', 1)
        key = key.strip()
        if not re.fullmatch(r'[A-Za-z0-9_]+', key):
            raise ValidationError('invalid key in map.yaml', {'key': key})
        if key in parsed:
            raise ValidationError('duplicate key rejected in map.yaml', {'key': key})
        rest_s = rest.strip()
        if rest_s == '':
            # Support 3-item YAML block sequence (`- x`) only for `origin`.
            items: list[Any] = []
            while i < len(lines):
                nxt = _strip_yaml_comment(lines[i]).strip()
                if not nxt:
                    i += 1
                    continue
                if nxt.startswith('- '):
                    items.append(_parse_yaml_scalar(nxt[2:]))
                    i += 1
                else:
                    break
            if not items:
                raise ValidationError(f'missing value for {key} in map.yaml')
            parsed[key] = items
        else:
            parsed[key] = _parse_yaml_scalar(rest_s)

    for req in ('image', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh'):
        if req not in parsed:
            raise ValidationError(f'map.yaml missing required field: {req}')

    image = parsed['image']
    if not isinstance(image, str) or not image:
        raise ValidationError('map.yaml `image` must be a non-empty string')
    if '\x00' in image or '/' in image or '\\' in image or '..' in image or os.path.isabs(image):
        raise SecurityError('map.yaml `image` must be a safe basename without path traversal', {'image': image})
    if expected_image_name and image != expected_image_name:
        raise ValidationError(
            'map.yaml `image` must match canonical bundle PGM filename',
            {'expected': expected_image_name, 'actual': image},
        )

    resolution = parsed['resolution']
    if not _is_finite_number(resolution) or float(resolution) <= 0.0 or float(resolution) > 10.0:
        raise ValidationError('map.yaml `resolution` must be finite in (0, 10]', {'resolution': resolution})
    resolution_f = float(resolution)

    origin = parsed['origin']
    if not isinstance(origin, list) or len(origin) != 3 or not all(_is_finite_number(v) for v in origin):
        raise ValidationError('map.yaml `origin` must be [x, y, yaw] finite numbers', {'origin': origin})
    ox, oy, oyaw = float(origin[0]), float(origin[1]), float(origin[2])

    negate_raw = parsed['negate']
    if isinstance(negate_raw, bool):
        negate = 1 if negate_raw else 0
    elif isinstance(negate_raw, int) and negate_raw in (0, 1):
        negate = negate_raw
    else:
        raise ValidationError('map.yaml `negate` must be 0 or 1', {'negate': negate_raw})

    occ_t = parsed['occupied_thresh']
    free_t = parsed['free_thresh']
    if not (_is_finite_number(occ_t) and _is_finite_number(free_t)):
        raise ValidationError('thresholds must be finite numbers')
    occ_tf, free_tf = float(occ_t), float(free_t)
    if not (0.0 <= free_tf < occ_tf <= 1.0):
        raise ValidationError(
            'thresholds must satisfy 0.0 <= free_thresh < occupied_thresh <= 1.0',
            {'free_thresh': free_tf, 'occupied_thresh': occ_tf},
        )

    mode = str(parsed.get('mode', 'trinary'))
    if mode not in ('trinary', 'scale', 'raw'):
        raise ValidationError('unsupported map.yaml `mode`', {'mode': mode})

    has_explicit_frame_id = 'frame_id' in parsed
    frame_id = validate_frame_id(parsed['frame_id'], field='frame_id') if has_explicit_frame_id else 'map'

    return {
        'image': image,
        'mode': mode,
        'frame_id': frame_id,
        'has_explicit_frame_id': has_explicit_frame_id,
        'resolution': resolution_f,
        'origin': [ox, oy, oyaw],
        'origin_x': ox,
        'origin_y': oy,
        'origin_yaw': oyaw,
        'negate': negate,
        'occupied_thresh': occ_tf,
        'free_thresh': free_tf,
        'yaml_sha256': _sha256_bytes(bytes(yaml_bytes)),
    }


def validate_rmf_compatibility(
    map_meta: dict[str, Any],
    canon_pixels: bytes,
    transform_rmf_from_nav2: dict[str, Any],
    *,
    nav_graph: dict[str, Any] | None = None,
    floor_level_name: str | None = None,
    free_clearance_m: float = 0.0,
    lane_sample_step_m: float = 0.05,
) -> dict[str, Any]:
    """Validate Open-RMF 2D coordinate transform & optional NavGraph against canonical map.

    Follows `fleet_control/rmf_adapter/limo_rmf_adapter/coords.py` rigid transform:
      nav2_to_rmf: (tx + cos*xn - sin*yn, ty + sin*xn + cos*yn)
      rmf_to_nav2: (cos*(xr-tx) + sin*(yr-ty), -sin*(xr-tx) + cos*(yr-ty))
    """
    errors: list[str] = []
    warnings: list[str] = []
    checks: list[dict[str, Any]] = []

    width = int(map_meta['width'])
    height = int(map_meta['height'])
    res = float(map_meta['resolution'])
    ox = float(map_meta['origin_x'])
    oy = float(map_meta['origin_y'])
    oyaw = float(map_meta.get('origin_yaw', 0.0))
    rotated_map_frame = abs(oyaw) > ORIGIN_YAW_STRICT_TOL_RAD

    if len(canon_pixels) != width * height:
        raise ValidationError('canon_pixels size does not match width * height')
    if free_clearance_m < 0.0 or not _is_finite_number(free_clearance_m):
        raise ValidationError('free_clearance_m must be finite >= 0')
    if lane_sample_step_m <= 0.0 or not _is_finite_number(lane_sample_step_m):
        raise ValidationError('lane_sample_step_m must be finite > 0')

    if rotated_map_frame:
        warnings.append(
            f'origin_yaw={oyaw:.6f} rad is non-zero; strict frame_guard (zero-yaw) rejects rotated map origins'
        )
    checks.append({
        'name': 'origin_yaw_check',
        'ok': True,
        'rotated_map_frame': rotated_map_frame,
        'origin_yaw': oyaw,
    })

    if not isinstance(transform_rmf_from_nav2, dict):
        errors.append('transform_rmf_from_nav2 must be an object')
        return {
            'compatible': False,
            'rotated_map_frame': rotated_map_frame,
            'checked_waypoints': 0,
            'checked_lanes': 0,
            'checks': checks,
            'errors': errors,
            'warnings': warnings,
        }

    rot = transform_rmf_from_nav2.get('rotation_rad')
    trans = transform_rmf_from_nav2.get('translation')
    tx_ok = (
        _is_finite_number(rot)
        and isinstance(trans, list)
        and len(trans) == 2
        and _is_finite_number(trans[0])
        and _is_finite_number(trans[1])
    )
    if not tx_ok:
        errors.append('transform_rmf_from_nav2 requires finite rotation_rad and [tx, ty] translation')
        checks.append({'name': 'transform_valid', 'ok': False})
        return {
            'compatible': False,
            'rotated_map_frame': rotated_map_frame,
            'checked_waypoints': 0,
            'checked_lanes': 0,
            'checks': checks,
            'errors': errors,
            'warnings': warnings,
        }
    checks.append({'name': 'transform_valid', 'ok': True})

    rmf_level = transform_rmf_from_nav2.get('rmf_level')
    graph_level = nav_graph.get('level_name') if isinstance(nav_graph, dict) else None
    if floor_level_name is not None:
        if rmf_level is not None and rmf_level != floor_level_name:
            errors.append(f'transform rmf_level={rmf_level!r} != floor level_name={floor_level_name!r}')
        if graph_level is not None and graph_level != floor_level_name:
            errors.append(f'nav_graph level_name={graph_level!r} != floor level_name={floor_level_name!r}')
        checks.append({
            'name': 'level_match',
            'ok': not any('level' in e for e in errors),
            'expected': floor_level_name,
            'rmf_level': rmf_level,
            'graph_level': graph_level,
        })

    c_rmf = math.cos(float(rot))
    s_rmf = math.sin(float(rot))
    tx, ty = float(trans[0]), float(trans[1])
    c_origin = math.cos(oyaw)
    s_origin = math.sin(oyaw)

    def rmf_to_nav2(xr: float, yr: float) -> tuple[float, float]:
        dx, dy = xr - tx, yr - ty
        return (c_rmf * dx + s_rmf * dy, -s_rmf * dx + c_rmf * dy)

    def is_free_at(xn: float, yn: float, clearance_m: float) -> tuple[bool, str | None]:
        dx, dy = xn - ox, yn - oy
        lx = c_origin * dx + s_origin * dy
        ly = -s_origin * dx + c_origin * dy
        col_f = lx / res
        row_b_f = ly / res
        if not (0.0 <= col_f < width and 0.0 <= row_b_f < height):
            return False, f'out of map bounds at nav2=({xn:.3f},{yn:.3f})'
        r_cells = int(math.floor(clearance_m / res))
        col_c = int(math.floor(col_f))
        row_c = height - 1 - int(math.floor(row_b_f))
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                if r_cells > 0 and (dr * dr + dc * dc) * (res * res) > clearance_m * clearance_m + 1e-12:
                    continue
                rr = row_c + dr
                cc = col_c + dc
                if not (0 <= rr < height and 0 <= cc < width):
                    return False, f'clearance crosses map border near nav2=({xn:.3f},{yn:.3f})'
                val = canon_pixels[rr * width + cc]
                if val != CANON_FREE:
                    state = 'OCCUPIED' if val == CANON_OCCUPIED else 'UNKNOWN'
                    return False, f'cell is {state} at nav2=({xn:.3f},{yn:.3f})'
        return True, None

    checked_waypoints = 0
    checked_lanes = 0

    if nav_graph is not None:
        if not isinstance(nav_graph, dict):
            errors.append('nav_graph must be an object')
        else:
            waypoints = nav_graph.get('waypoints')
            lanes = nav_graph.get('lanes', [])
            if not isinstance(waypoints, list) or len(waypoints) == 0:
                errors.append('nav_graph requires non-empty waypoints list')
            else:
                seen_names: set[str] = set()
                nav2_wps: list[tuple[float, float] | None] = []
                for idx, wp in enumerate(waypoints):
                    if not isinstance(wp, dict):
                        errors.append(f'waypoint[{idx}] must be an object')
                        nav2_wps.append(None)
                        continue
                    wname = wp.get('name', f'wp_{idx}')
                    if wname in seen_names:
                        errors.append(f'duplicate waypoint name: {wname!r}')
                    seen_names.add(wname)
                    wx, wy = wp.get('x'), wp.get('y')
                    if not (_is_finite_number(wx) and _is_finite_number(wy)):
                        errors.append(f'waypoint {wname!r} has non-finite coordinates')
                        nav2_wps.append(None)
                        continue
                    xn, yn = rmf_to_nav2(float(wx), float(wy))
                    nav2_wps.append((xn, yn))
                    checked_waypoints += 1
                    ok, reason = is_free_at(xn, yn, float(free_clearance_m))
                    if not ok:
                        errors.append(f'waypoint {wname!r} invalid: {reason}')
                checks.append({
                    'name': 'waypoints_free',
                    'ok': checked_waypoints > 0 and not any(e.startswith('waypoint') for e in errors),
                    'count': checked_waypoints,
                })

                if not isinstance(lanes, list):
                    errors.append('nav_graph.lanes must be a list')
                else:
                    for lidx, lane in enumerate(lanes):
                        if isinstance(lane, (list, tuple)) and len(lane) >= 2:
                            u, v = lane[0], lane[1]
                        elif isinstance(lane, dict):
                            u, v = lane.get('u'), lane.get('v')
                        else:
                            errors.append(f'lane[{lidx}] malformed')
                            continue
                        if not (isinstance(u, int) and isinstance(v, int)) or not (
                            0 <= u < len(nav2_wps) and 0 <= v < len(nav2_wps)
                        ):
                            errors.append(f'lane[{lidx}] endpoint index out of range: {u}->{v}')
                            continue
                        p0, p1 = nav2_wps[u], nav2_wps[v]
                        if p0 is None or p1 is None:
                            continue
                        checked_lanes += 1
                        dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                        steps = max(2, int(math.ceil(dist / float(lane_sample_step_m))) + 1)
                        for s in range(steps):
                            t = s / float(steps - 1)
                            xs = p0[0] + t * (p1[0] - p0[0])
                            ys = p0[1] + t * (p1[1] - p0[1])
                            ok, reason = is_free_at(xs, ys, 0.0)
                            if not ok:
                                errors.append(f'lane[{lidx}] ({u}->{v}) crosses non-free space: {reason}')
                                break
                    checks.append({
                        'name': 'lanes_free',
                        'ok': not any(e.startswith('lane[') for e in errors),
                        'count': checked_lanes,
                    })

    return {
        'compatible': len(errors) == 0,
        'rotated_map_frame': rotated_map_frame,
        'checked_waypoints': checked_waypoints,
        'checked_lanes': checked_lanes,
        'checks': checks,
        'errors': errors,
        'warnings': warnings,
    }


def classify_map_update(
    previous: dict[str, Any] | None,
    current: dict[str, Any],
    *,
    bound_version: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Distinguish normal SLAM `/map` progression from `map_epoch` state transitions.

    - In `MAPPING`, monotonic SLAM grid updates keep `increment_map_epoch=False`
      (`SLAM_PROGRESSIVE_GROWTH` / `SLAM_INITIAL_GRID`) unless resolution or
      map origin jumps discontinuously.
    - In `NAVIGATION`, any mismatch against the verified bound map version
      (or unverified map usage) forces `increment_map_epoch=True` and
      `invalidate_navigation=True`.
    """
    mode = str(current.get('operation_mode', 'IDLE')).upper()
    if mode not in OPERATION_MODES:
        raise ValidationError('invalid operation_mode', {'operation_mode': mode})

    cur_hash = current.get('content_sha256')
    cur_res = float(current.get('resolution', 0.0))
    cur_origin = list(current.get('origin', [0.0, 0.0, 0.0]))
    if len(cur_origin) != 3 or not all(_is_finite_number(v) for v in cur_origin):
        raise ValidationError('current.origin must be [x, y, yaw]')
    if not _is_finite_number(cur_res) or cur_res <= 0.0:
        raise ValidationError('current.resolution must be finite > 0')

    if mode == 'IDLE':
        return {
            'classification': 'IDLE_OBSERVATION',
            'increment_map_epoch': False,
            'invalidate_navigation': False,
            'requires_relocalization': False,
            'navigable': False,
            'reason': 'robot is in IDLE mode',
        }

    if mode == 'MAPPING':
        if previous is None:
            return {
                'classification': 'SLAM_INITIAL_GRID',
                'increment_map_epoch': False,
                'invalidate_navigation': False,
                'requires_relocalization': False,
                'navigable': False,
                'reason': 'initial live SLAM occupancy grid',
            }
        prev_res = float(previous.get('resolution', cur_res))
        prev_origin = list(previous.get('origin', cur_origin))
        res_jump = abs(cur_res - prev_res) > 1e-6
        origin_jump = (
            len(prev_origin) != 3
            or abs(float(cur_origin[0]) - float(prev_origin[0])) > ORIGIN_TOL_M
            or abs(float(cur_origin[1]) - float(prev_origin[1])) > ORIGIN_TOL_M
            or abs(float(cur_origin[2]) - float(prev_origin[2])) > ORIGIN_YAW_STRICT_TOL_RAD
        )
        if res_jump or origin_jump:
            return {
                'classification': 'MAPPING_FRAME_DISCONTINUITY',
                'increment_map_epoch': True,
                'invalidate_navigation': True,
                'requires_relocalization': True,
                'navigable': False,
                'reason': 'resolution or origin discontinuous during SLAM mapping',
            }
        if previous.get('content_sha256') == cur_hash:
            return {
                'classification': 'UNCHANGED',
                'increment_map_epoch': False,
                'invalidate_navigation': False,
                'requires_relocalization': False,
                'navigable': False,
                'reason': 'SLAM map content hash unchanged',
            }
        return {
            'classification': 'SLAM_PROGRESSIVE_GROWTH',
            'increment_map_epoch': False,
            'invalidate_navigation': False,
            'requires_relocalization': False,
            'navigable': False,
            'reason': 'normal live SLAM /map expansion keeps map_epoch unchanged and remains non-navigable',
        }

    # NAVIGATION mode
    if bound_version is None:
        return {
            'classification': 'MISSING_BOUND_MAP',
            'increment_map_epoch': True,
            'invalidate_navigation': True,
            'requires_relocalization': True,
            'navigable': False,
            'reason': 'NAVIGATION requires a bound VERIFIED static MapVersion',
        }
    if bound_version.get('verification_status') != 'VERIFIED':
        return {
            'classification': 'UNVERIFIED_MAP_IN_NAVIGATION',
            'increment_map_epoch': True,
            'invalidate_navigation': True,
            'requires_relocalization': True,
            'navigable': False,
            'reason': 'unverified map version cannot authorize navigation',
        }
    if bound_version.get('source_mode') == 'SLAM_LIVE_SNAPSHOT':
        return {
            'classification': 'LIVE_SLAM_DISALLOWED_IN_NAVIGATION',
            'increment_map_epoch': True,
            'invalidate_navigation': True,
            'requires_relocalization': True,
            'navigable': False,
            'reason': 'live SLAM snapshot is not valid for static navigation',
        }
    cur_mid = current.get('map_version_id')
    if cur_mid != bound_version.get('map_version_id'):
        return {
            'classification': 'MAP_VERSION_SWITCH_REQUIRED',
            'increment_map_epoch': True,
            'invalidate_navigation': True,
            'requires_relocalization': True,
            'navigable': False,
            'reason': 'observed map_version_id differs from bound MapVersion',
        }
    if cur_hash != bound_version.get('content_sha256'):
        return {
            'classification': 'MAP_CONTENT_MISMATCH',
            'increment_map_epoch': True,
            'invalidate_navigation': True,
            'requires_relocalization': True,
            'navigable': False,
            'reason': 'observed canonical content_sha256 differs from registered MapVersion',
        }
    exp_origin = [
        float(bound_version['origin_x']),
        float(bound_version['origin_y']),
        float(bound_version['origin_yaw']),
    ]
    if (
        abs(cur_res - float(bound_version['resolution'])) > 1e-6
        or abs(float(cur_origin[0]) - exp_origin[0]) > ORIGIN_TOL_M
        or abs(float(cur_origin[1]) - exp_origin[1]) > ORIGIN_TOL_M
        or abs(float(cur_origin[2]) - exp_origin[2]) > ORIGIN_YAW_STRICT_TOL_RAD
    ):
        return {
            'classification': 'MAP_GEOMETRY_MISMATCH',
            'increment_map_epoch': True,
            'invalidate_navigation': True,
            'requires_relocalization': True,
            'navigable': False,
            'reason': 'observed origin or resolution differs from registered MapVersion',
        }
    return {
        'classification': 'VERIFIED_STATIC_MATCH',
        'increment_map_epoch': False,
        'invalidate_navigation': False,
        'requires_relocalization': False,
        'navigable': True,
        'reason': 'observed static map matches verified registered MapVersion',
    }


def _fsync_file(path: str) -> None:
    fd = os.open(path, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _fsync_dir(path: str) -> None:
    fd = os.open(path, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _write_readonly_atomic_file(directory: str, filename: str, payload: bytes) -> str:
    safe_name = validate_safe_filename(filename)
    dest = os.path.join(directory, safe_name)
    fd = os.open(dest, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o444)
    try:
        view = memoryview(payload)
        written = 0
        while written < len(view):
            written += os.write(fd, view[written:])
        os.fsync(fd)
    finally:
        os.close(fd)
    return dest


class MapRegistry:
    """Standalone SQLite + atomic filesystem store for facilities, buildings, floors, and immutable map bundles."""

    def __init__(
        self,
        db_path: str,
        bundle_root: str,
        *,
        max_http_response_bytes: int = DEFAULT_MAX_HTTP_RESPONSE_BYTES,
        max_http_write_bytes: int = MAX_HTTP_WRITE_BODY_BYTES,
        png_render_concurrency: int = DEFAULT_PNG_RENDER_CONCURRENCY,
        png_cache_max_items: int = DEFAULT_PNG_CACHE_MAX_ITEMS,
        png_cache_max_bytes: int = DEFAULT_PNG_CACHE_MAX_BYTES,
    ):
        self.db_path = db_path
        self.bundle_root = os.path.abspath(bundle_root)
        self.bundles_dir = os.path.join(self.bundle_root, 'bundles')
        self.staging_dir = os.path.join(self.bundle_root, '.staging')
        os.makedirs(self.bundles_dir, exist_ok=True)
        os.makedirs(self.staging_dir, exist_ok=True)
        os.makedirs(os.path.dirname(os.path.abspath(db_path)) or '.', exist_ok=True)

        self.max_http_response_bytes = max(1, int(max_http_response_bytes))
        self.max_http_write_bytes = max(1, int(max_http_write_bytes))
        self.png_render_concurrency = max(1, int(png_render_concurrency))
        self.png_cache_max_items = max(0, int(png_cache_max_items))
        self.png_cache_max_bytes = max(0, int(png_cache_max_bytes))
        self._render_sem = threading.Semaphore(self.png_render_concurrency)
        self._cache_lock = threading.Lock()
        self._png_cache: OrderedDict[tuple[str, str], bytes] = OrderedDict()
        self._png_cache_bytes = 0

        self.c = sqlite3.connect(db_path, check_same_thread=False, isolation_level=None)
        self.c.row_factory = sqlite3.Row
        self.lock = threading.RLock()
        self._tx_depth = 0
        self.c.execute('PRAGMA journal_mode=WAL')
        self.c.execute('PRAGMA busy_timeout=5000')
        self.c.execute('PRAGMA foreign_keys=ON')
        self.c.executescript(SCHEMA)
        self._ensure_migrations()

    def _ensure_migrations(self) -> None:
        """Apply additive Phase 2-B columns, backfill legacy bundle_sha256, and refresh immutability trigger."""
        with self.transaction():
            cols = {str(r['name']) for r in self.q('PRAGMA table_info(map_versions)')}
            altered = False
            if 'frame_id' not in cols:
                self.x("ALTER TABLE map_versions ADD COLUMN frame_id TEXT NOT NULL DEFAULT 'map'")
                altered = True
            if 'bundle_sha256' not in cols:
                self.x("ALTER TABLE map_versions ADD COLUMN bundle_sha256 TEXT NOT NULL DEFAULT ''")
                altered = True

            legacy_rows = self.q("SELECT * FROM map_versions WHERE bundle_sha256 = '' OR bundle_sha256 IS NULL")
            if altered or legacy_rows:
                self.x('DROP TRIGGER IF EXISTS trg_map_versions_immutable')
                for r in legacy_rows:
                    fid = str(r['frame_id'] or 'map')
                    bsha = compute_bundle_sha256(
                        content_sha256=str(r['content_sha256']),
                        yaml_sha256=str(r['yaml_sha256']),
                        pgm_sha256=str(r['pgm_sha256']),
                        posegraph_sha256=r['posegraph_sha256'],
                        posegraph_data_sha256=r['posegraph_data_sha256'],
                        width=int(r['width']),
                        height=int(r['height']),
                        resolution=float(r['resolution']),
                        origin_x=float(r['origin_x']),
                        origin_y=float(r['origin_y']),
                        origin_yaw=float(r['origin_yaw']),
                        negate=int(r['negate']),
                        occupied_thresh=float(r['occupied_thresh']),
                        free_thresh=float(r['free_thresh']),
                        frame_id=fid,
                    )
                    self.x(
                        'UPDATE map_versions SET frame_id=?, bundle_sha256=? WHERE map_version_id=?',
                        (fid, bsha, r['map_version_id']),
                    )

            self.x('CREATE INDEX IF NOT EXISTS idx_map_versions_bundle_sha ON map_versions(bundle_sha256)')
            self.x('DROP TRIGGER IF EXISTS trg_map_versions_immutable')
            self.x(
                """
                CREATE TRIGGER trg_map_versions_immutable
                BEFORE UPDATE ON map_versions
                WHEN
                    NEW.map_version_id != OLD.map_version_id OR
                    NEW.floor_id != OLD.floor_id OR
                    NEW.label != OLD.label OR
                    NEW.source_mode != OLD.source_mode OR
                    NEW.storage_key != OLD.storage_key OR
                    NEW.width != OLD.width OR
                    NEW.height != OLD.height OR
                    NEW.resolution != OLD.resolution OR
                    NEW.origin_x != OLD.origin_x OR
                    NEW.origin_y != OLD.origin_y OR
                    NEW.origin_yaw != OLD.origin_yaw OR
                    NEW.negate != OLD.negate OR
                    NEW.occupied_thresh != OLD.occupied_thresh OR
                    NEW.free_thresh != OLD.free_thresh OR
                    NEW.content_sha256 != OLD.content_sha256 OR
                    NEW.yaml_sha256 != OLD.yaml_sha256 OR
                    NEW.pgm_sha256 != OLD.pgm_sha256 OR
                    COALESCE(NEW.posegraph_sha256, '') != COALESCE(OLD.posegraph_sha256, '') OR
                    COALESCE(NEW.posegraph_data_sha256, '') != COALESCE(OLD.posegraph_data_sha256, '') OR
                    NEW.frame_id != OLD.frame_id OR
                    NEW.bundle_sha256 != OLD.bundle_sha256 OR
                    NEW.rotated_map_frame != OLD.rotated_map_frame OR
                    NEW.created_by != OLD.created_by OR
                    NEW.created_at != OLD.created_at
                BEGIN
                    SELECT RAISE(ABORT, 'immutable map_versions columns cannot be modified');
                END
                """
            )

    def close(self) -> None:
        with self.lock:
            self.c.close()

    def transaction(self):
        return _Tx(self)

    def q(self, sql: str, params: tuple[Any, ...] = ()) -> list[dict[str, Any]]:
        with self.lock:
            return [dict(r) for r in self.c.execute(sql, params).fetchall()]

    def one(self, sql: str, params: tuple[Any, ...] = ()) -> dict[str, Any] | None:
        with self.lock:
            row = self.c.execute(sql, params).fetchone()
            return dict(row) if row is not None else None

    def x(self, sql: str, params: tuple[Any, ...] = ()) -> sqlite3.Cursor:
        with self.lock:
            return self.c.execute(sql, params)

    def _audit(self, actor: str, event_type: str, entity_id: str, detail: dict[str, Any]) -> None:
        self.x(
            'INSERT INTO map_registry_events(ts, actor, event_type, entity_id, detail_json) VALUES(?,?,?,?,?)',
            (time.time(), actor, event_type, entity_id, json.dumps(detail, sort_keys=True)),
        )

    # ------------------------------------------------------------------
    # Hierarchy: Facility -> Building -> Floor
    # ------------------------------------------------------------------

    def create_facility(
        self,
        facility_id: str,
        name: str,
        *,
        timezone: str = 'Asia/Seoul',
        description: str = '',
        actor: str = 'system',
    ) -> dict[str, Any]:
        fid = validate_entity_id(facility_id, 'facility_id')
        if not isinstance(name, str) or not name.strip():
            raise ValidationError('facility name must be non-empty')
        now = time.time()
        with self.transaction():
            existing = self.one('SELECT * FROM facilities WHERE facility_id=?', (fid,))
            if existing is not None:
                if existing['name'] == name and existing['timezone'] == timezone and existing['description'] == description:
                    return existing
                raise ConflictError('facility_id already exists with different attributes', {'facility_id': fid})
            self.x(
                'INSERT INTO facilities(facility_id, name, timezone, description, created_at, updated_at) '
                'VALUES(?,?,?,?,?,?)',
                (fid, name, timezone, description, now, now),
            )
            self._audit(actor, 'FACILITY_CREATED', fid, {'name': name, 'timezone': timezone})
        return self.one('SELECT * FROM facilities WHERE facility_id=?', (fid,))  # type: ignore[return-value]

    def create_building(
        self,
        building_id: str,
        facility_id: str,
        name: str,
        *,
        description: str = '',
        actor: str = 'system',
    ) -> dict[str, Any]:
        bid = validate_entity_id(building_id, 'building_id')
        fid = validate_entity_id(facility_id, 'facility_id')
        if not isinstance(name, str) or not name.strip():
            raise ValidationError('building name must be non-empty')
        now = time.time()
        with self.transaction():
            if self.one('SELECT facility_id FROM facilities WHERE facility_id=?', (fid,)) is None:
                raise NotFoundError('facility_id not found', {'facility_id': fid})
            existing = self.one('SELECT * FROM buildings WHERE building_id=?', (bid,))
            if existing is not None:
                if existing['facility_id'] == fid and existing['name'] == name and existing['description'] == description:
                    return existing
                raise ConflictError('building_id already exists with different attributes', {'building_id': bid})
            self.x(
                'INSERT INTO buildings(building_id, facility_id, name, description, created_at, updated_at) '
                'VALUES(?,?,?,?,?,?)',
                (bid, fid, name, description, now, now),
            )
            self._audit(actor, 'BUILDING_CREATED', bid, {'facility_id': fid, 'name': name})
        return self.one('SELECT * FROM buildings WHERE building_id=?', (bid,))  # type: ignore[return-value]

    def create_floor(
        self,
        floor_id: str,
        building_id: str,
        level_name: str,
        *,
        elevation_m: float = 0.0,
        rmf_map_name: str | None = None,
        actor: str = 'system',
    ) -> dict[str, Any]:
        flid = validate_entity_id(floor_id, 'floor_id')
        bid = validate_entity_id(building_id, 'building_id')
        lvl = validate_entity_id(level_name, 'level_name')
        if not _is_finite_number(elevation_m):
            raise ValidationError('elevation_m must be finite')
        rmf_name = validate_entity_id(rmf_map_name if rmf_map_name is not None else lvl, 'rmf_map_name')
        now = time.time()
        with self.transaction():
            if self.one('SELECT building_id FROM buildings WHERE building_id=?', (bid,)) is None:
                raise NotFoundError('building_id not found', {'building_id': bid})
            existing = self.one('SELECT * FROM floors WHERE floor_id=?', (flid,))
            if existing is not None:
                if (
                    existing['building_id'] == bid
                    and existing['level_name'] == lvl
                    and float(existing['elevation_m']) == float(elevation_m)
                    and existing['rmf_map_name'] == rmf_name
                ):
                    return existing
                raise ConflictError('floor_id already exists with different attributes', {'floor_id': flid})
            dup_lvl = self.one(
                'SELECT floor_id FROM floors WHERE building_id=? AND level_name=?',
                (bid, lvl),
            )
            if dup_lvl is not None:
                raise ConflictError(
                    'level_name already registered in building',
                    {'building_id': bid, 'level_name': lvl},
                )
            self.x(
                'INSERT INTO floors(floor_id, building_id, level_name, elevation_m, rmf_map_name, created_at, updated_at) '
                'VALUES(?,?,?,?,?,?,?)',
                (flid, bid, lvl, float(elevation_m), rmf_name, now, now),
            )
            self._audit(actor, 'FLOOR_CREATED', flid, {'building_id': bid, 'level_name': lvl})
        return self.one('SELECT * FROM floors WHERE floor_id=?', (flid,))  # type: ignore[return-value]

    def create_area(
        self,
        area_id: str,
        floor_id: str,
        name: str,
        *,
        description: str = '',
        coverage: str = 'PARTIAL_FLOOR',
        actor: str = 'system',
    ) -> dict[str, Any]:
        """Declare an operator-confirmed area inside a floor (idempotent, same attributes only)."""
        aid = validate_entity_id(area_id, 'area_id')
        flid = validate_entity_id(floor_id, 'floor_id')
        if not isinstance(name, str) or not name.strip():
            raise ValidationError('area name must be non-empty')
        if not isinstance(description, str):
            raise ValidationError('area description must be a string')
        if coverage not in AREA_COVERAGES:
            raise ValidationError('invalid area coverage', {'allowed': sorted(AREA_COVERAGES)})
        now = time.time()
        with self.transaction():
            if self.one('SELECT floor_id FROM floors WHERE floor_id=?', (flid,)) is None:
                raise NotFoundError('floor_id not found', {'floor_id': flid})
            existing = self.one('SELECT * FROM map_areas WHERE area_id=?', (aid,))
            if existing is not None:
                if (existing['floor_id'] == flid and existing['name'] == name
                        and existing['description'] == description and existing['coverage'] == coverage):
                    return existing
                raise ConflictError('area_id already exists with different attributes', {'area_id': aid})
            self.x(
                'INSERT INTO map_areas(area_id, floor_id, name, description, coverage, created_at, updated_at) '
                'VALUES(?,?,?,?,?,?,?)',
                (aid, flid, name, description, coverage, now, now),
            )
            self._audit(actor, 'AREA_CREATED', aid, {'floor_id': flid, 'name': name, 'coverage': coverage})
        return self.one('SELECT * FROM map_areas WHERE area_id=?', (aid,))  # type: ignore[return-value]

    def record_lineage(self, map_version_id: str, *, parent_map_version_id: str | None, mapping_mode: str,
                       session_id: str, actor: str = 'system') -> dict[str, Any]:
        """Write-once lineage. A child never replaces its parent; rollback = choosing the parent version again."""
        mvid = validate_map_version_id(map_version_id)
        if mapping_mode not in MAPPING_MODES:
            raise ValidationError('invalid mapping_mode', {'allowed': sorted(MAPPING_MODES)})
        sid = validate_entity_id(session_id, 'session_id')
        with self.transaction():
            row = self.one('SELECT floor_id FROM map_versions WHERE map_version_id=?', (mvid,))
            if row is None:
                raise NotFoundError('map_version_id not found', {'map_version_id': mvid})
            parent = None
            if parent_map_version_id is not None:
                parent = validate_map_version_id(parent_map_version_id)
                prow = self.one('SELECT floor_id FROM map_versions WHERE map_version_id=?', (parent,))
                if prow is None:
                    raise NotFoundError('parent map version not found', {'parent_map_version_id': parent})
                if parent == mvid or prow['floor_id'] != row['floor_id']:
                    raise ConflictError('parent must be a different version on the same floor')
            if (mapping_mode == 'CONTINUE_MAP') != (parent is not None):
                raise ValidationError('CONTINUE_MAP requires a parent; other modes must not have one')
            existing = self.one('SELECT * FROM map_version_lineage WHERE map_version_id=?', (mvid,))
            if existing is not None:
                if (existing['parent_map_version_id'], existing['mapping_mode'], existing['session_id']) == (parent, mapping_mode, sid):
                    return existing
                raise ConflictError('lineage already recorded differently', {'map_version_id': mvid})
            self.x('INSERT INTO map_version_lineage(map_version_id, parent_map_version_id, mapping_mode, session_id, recorded_by, recorded_at) '
                   'VALUES(?,?,?,?,?,?)', (mvid, parent, mapping_mode, sid, actor, time.time()))
            self._audit(actor, 'MAP_LINEAGE_RECORDED', mvid, {'parent': parent, 'mode': mapping_mode, 'session_id': sid})
        return self.one('SELECT * FROM map_version_lineage WHERE map_version_id=?', (mvid,))  # type: ignore[return-value]

    def _prepare_web_upload(self, payload: dict[str, Any], yaml_bytes: bytes, pgm_bytes: bytes,
                            pg_bytes: bytes | None, pgd_bytes: bytes | None, exp_ck: dict[str, Any] | None,
                            actor: str) -> dict[str, Any]:
        """Validate the optional authoring fields of POST /maps BEFORE anything is stored.

        area_id (same floor), lineage (write-once rules), threshold_policy and the unknown-pixel safety check shared with
        scripts/register_saved_map.py: a saved map whose free_thresh would make ROS read unknown cells as free is refused,
        unless threshold_policy='derive' registers a DERIVED yaml (only the free_thresh line changes). Original and derived
        hashes and the reason go into the evidence; the uploaded originals are checked against the client's checksums first.
        """
        floor_id = payload.get('floor_id', '')
        area_id = payload.get('area_id')
        area_row = None
        if area_id is not None:
            area_id = validate_entity_id(area_id, 'area_id')
            area_row = self.one('SELECT * FROM map_areas WHERE area_id=?', (area_id,))
            if area_row is None:
                raise NotFoundError('area_id not found', {'area_id': area_id})
            if area_row['floor_id'] != floor_id:
                raise ConflictError('area belongs to a different floor than floor_id',
                                    {'area_floor_id': area_row['floor_id'], 'floor_id': floor_id})
        lineage = None
        raw_lin = payload.get('lineage')
        if raw_lin is not None:
            if not isinstance(raw_lin, dict):
                raise ValidationError('lineage must be an object')
            mode = raw_lin.get('mapping_mode')
            if mode not in MAPPING_MODES:
                raise ValidationError('invalid mapping_mode', {'allowed': sorted(MAPPING_MODES)})
            parent = raw_lin.get('parent_map_version_id')
            if (mode == 'CONTINUE_MAP') != (parent is not None):
                raise ValidationError('CONTINUE_MAP requires a parent; other modes must not have one')
            if parent is not None:
                parent = validate_map_version_id(parent)
                prow = self.one('SELECT floor_id FROM map_versions WHERE map_version_id=?', (parent,))
                if prow is None:
                    raise NotFoundError('parent map version not found', {'parent_map_version_id': parent})
                if prow['floor_id'] != floor_id:
                    raise ConflictError('parent must be on the same floor')
            sid = raw_lin.get('session_id')
            if sid is None:
                sid = 'web_upload_' + hashlib.sha256(bytes(yaml_bytes) + bytes(pgm_bytes)).hexdigest()[:12]
            lineage = {'mapping_mode': mode, 'parent_map_version_id': parent, 'session_id': validate_entity_id(sid, 'session_id')}
        policy = payload.get('threshold_policy', 'reject')
        if policy not in THRESHOLD_POLICIES:
            raise ValidationError('invalid threshold_policy', {'allowed': sorted(THRESHOLD_POLICIES)})
        note = payload.get('note', '')
        if not isinstance(note, str) or len(note) > 500:
            raise ValidationError('note must be a string of at most 500 characters')

        pgm_info = parse_pgm_p5(pgm_bytes)
        ymeta = parse_ros_map_yaml(yaml_bytes)
        adjustment = None
        client_checksums_provided = bool(exp_ck)
        issue = unknown_threshold_issue(ymeta, pgm_info)
        if issue is not None:
            if policy != 'derive' or not issue['fixable_by_free_thresh']:
                raise PolicyError(
                    'saved thresholds would make ROS read unknown pixels as free or occupied; fix map.yaml or use threshold_policy=derive',
                    {'code': 'unknown_pixels_misclassified', **issue, 'policy': policy})
            orig_sha = _sha256_bytes(bytes(yaml_bytes))
            if exp_ck and 'yaml_sha256' in exp_ck and exp_ck['yaml_sha256'] != orig_sha:
                raise ChecksumError('checksum mismatch for yaml_sha256',
                                    {'field': 'yaml_sha256', 'expected': exp_ck['yaml_sha256'], 'actual': orig_sha})
            derived = derive_yaml_free_thresh(yaml_bytes, issue['derived_free_thresh'])
            adjustment = {'original_yaml_sha256': orig_sha, 'derived_yaml_sha256': _sha256_bytes(derived),
                          'free_thresh_from': issue['free_thresh'], 'free_thresh_to': issue['derived_free_thresh'],
                          'reason': 'saved free_thresh would classify unknown pixels as free in ROS map_server'}
            yaml_bytes = derived
            ymeta = parse_ros_map_yaml(yaml_bytes)
            exp_ck = {k: v for k, v in (exp_ck or {}).items() if k != 'yaml_sha256'} or None
        canon = pgm_info['canonical_bytes']
        occ, free, unk = canon.count(b'\x00'), canon.count(b'\xff'), canon.count(b'\x80')
        res = float(ymeta['resolution'])
        counts = {'width': pgm_info['width'], 'height': pgm_info['height'], 'occupied': occ, 'free': free, 'unknown': unk,
                  'known_cells': occ + free, 'known_area_m2': round((occ + free) * res * res, 2)}
        loc: dict[str, Any] = {'source': 'operator_selected', 'auto_detected': False, 'floor_id': floor_id}
        frow = self.one('SELECT f.level_name, b.building_id, b.name AS building_name, fa.name AS facility_name '
                        'FROM floors f JOIN buildings b ON b.building_id=f.building_id '
                        'JOIN facilities fa ON fa.facility_id=b.facility_id WHERE f.floor_id=?', (floor_id,))
        if frow is not None:
            loc.update({'facility': frow['facility_name'], 'building': frow['building_name'], 'floor': frow['level_name']})
        if area_row is not None:
            loc.update({'area': area_row['name'], 'coverage': area_row['coverage']})
        evidence = {
            'method': 'web_upload', 'uploader': actor, 'location': loc,
            'artifacts_sha256': {'map/map.yaml': (adjustment or {}).get('original_yaml_sha256') or _sha256_bytes(bytes(yaml_bytes)),
                                 'map/map.pgm': pgm_info['pgm_sha256'],
                                 'map/posegraph.posegraph': _sha256_bytes(bytes(pg_bytes)) if pg_bytes else None,
                                 'map/posegraph.data': _sha256_bytes(bytes(pgd_bytes)) if pgd_bytes else None},
            'yaml_adjustment': adjustment, 'grid_counts_pgm': counts,
            'client_checksums_provided': client_checksums_provided,
            'note': note,
            'field_verification': 'UNVERIFIED (uploaded for registration only; no localization or navigation trial)',
        }
        return {'area_id': area_id, 'lineage': lineage, 'yaml_bytes': yaml_bytes, 'expected_checksums': exp_ck,
                'evidence': evidence, 'yaml_adjustment': adjustment}

    def link_map_version_area(self, map_version_id: str, area_id: str, *, actor: str = 'system') -> dict[str, Any]:
        """Write-once link of an immutable MapVersion to an area on the same floor."""
        mvid = validate_map_version_id(map_version_id)
        aid = validate_entity_id(area_id, 'area_id')
        with self.transaction():
            mv = self.one('SELECT floor_id FROM map_versions WHERE map_version_id=?', (mvid,))
            if mv is None:
                raise NotFoundError('map_version_id not found', {'map_version_id': mvid})
            area = self.one('SELECT * FROM map_areas WHERE area_id=?', (aid,))
            if area is None:
                raise NotFoundError('area_id not found', {'area_id': aid})
            if area['floor_id'] != mv['floor_id']:
                raise ConflictError('area belongs to a different floor than the map version',
                                    {'area_floor_id': area['floor_id'], 'map_floor_id': mv['floor_id']})
            existing = self.one('SELECT * FROM map_version_areas WHERE map_version_id=?', (mvid,))
            if existing is not None:
                if existing['area_id'] == aid:
                    return existing
                raise ConflictError('map version is already linked to another area', {'area_id': existing['area_id']})
            self.x('INSERT INTO map_version_areas(map_version_id, area_id, linked_by, linked_at) VALUES(?,?,?,?)',
                   (mvid, aid, actor, time.time()))
            self._audit(actor, 'MAP_AREA_LINKED', mvid, {'area_id': aid})
        return self.one('SELECT * FROM map_version_areas WHERE map_version_id=?', (mvid,))  # type: ignore[return-value]

    # ------------------------------------------------------------------
    # Immutable MapVersion Registration & Atomic Bundle Storage
    # ------------------------------------------------------------------

    def _validate_bundle_payload(
        self,
        *,
        floor_id: str,
        label: str,
        yaml_bytes: bytes,
        pgm_bytes: bytes,
        source_mode: str,
        verification_status: str,
        verification_evidence: dict[str, Any] | None,
        posegraph_bytes: bytes | None,
        posegraph_data_bytes: bytes | None,
        expected_checksums: dict[str, str] | None,
        transform_rmf_from_nav2: dict[str, Any] | None,
        nav_graph: dict[str, Any] | None,
        floor_level_name: str,
        free_clearance_m: float,
        frame_id: str | None = None,
    ) -> dict[str, Any]:
        if source_mode not in SOURCE_MODES:
            raise ValidationError('unsupported source_mode', {'source_mode': source_mode})
        if verification_status not in VERIFICATION_STATUSES:
            raise ValidationError('unsupported verification_status', {'verification_status': verification_status})
        if verification_status == 'REVOKED':
            raise PolicyError('cannot register a new map version directly as REVOKED')

        # Policy 6: unverified / live SLAM snapshots cannot register as VERIFIED.
        if verification_status == 'VERIFIED':
            if source_mode == 'SLAM_LIVE_SNAPSHOT':
                raise PolicyError('SLAM_LIVE_SNAPSHOT cannot be registered as VERIFIED')
            if not isinstance(verification_evidence, dict) or not verification_evidence.get('method'):
                raise PolicyError('registering as VERIFIED requires explicit verification_evidence with `method`')

        yaml_meta = parse_ros_map_yaml(yaml_bytes, expected_image_name='map.pgm')
        pgm_meta = parse_pgm_p5(pgm_bytes)

        if frame_id is not None:
            resolved_frame_id = validate_frame_id(frame_id)
            if yaml_meta.get('has_explicit_frame_id') and yaml_meta['frame_id'] != resolved_frame_id:
                raise ValidationError(
                    'frame_id argument conflicts with map.yaml frame_id',
                    {'argument': resolved_frame_id, 'yaml_frame_id': yaml_meta['frame_id']},
                )
        else:
            resolved_frame_id = str(yaml_meta.get('frame_id') or 'map')

        pg_sha: str | None = None
        if posegraph_bytes is not None:
            if not isinstance(posegraph_bytes, (bytes, bytearray)) or len(posegraph_bytes) == 0:
                raise ValidationError('posegraph_bytes must be non-empty when provided')
            if len(posegraph_bytes) > MAX_POSEGRAPH_BYTES:
                raise ValidationError('posegraph exceeds size limit')
            pg_sha = _sha256_bytes(bytes(posegraph_bytes))

        pg_data_sha: str | None = None
        if posegraph_data_bytes is not None:
            if posegraph_bytes is None:
                raise ValidationError('posegraph_data_bytes requires posegraph_bytes')
            if not isinstance(posegraph_data_bytes, (bytes, bytearray)) or len(posegraph_data_bytes) == 0:
                raise ValidationError('posegraph_data_bytes must be non-empty when provided')
            if len(posegraph_data_bytes) > MAX_POSEGRAPH_BYTES:
                raise ValidationError('posegraph.data exceeds size limit')
            pg_data_sha = _sha256_bytes(bytes(posegraph_data_bytes))

        bundle_sha256 = compute_bundle_sha256(
            content_sha256=pgm_meta['content_sha256'],
            yaml_sha256=yaml_meta['yaml_sha256'],
            pgm_sha256=pgm_meta['pgm_sha256'],
            posegraph_sha256=pg_sha,
            posegraph_data_sha256=pg_data_sha,
            width=int(pgm_meta['width']),
            height=int(pgm_meta['height']),
            resolution=float(yaml_meta['resolution']),
            origin_x=float(yaml_meta['origin_x']),
            origin_y=float(yaml_meta['origin_y']),
            origin_yaw=float(yaml_meta['origin_yaw']),
            negate=int(yaml_meta['negate']),
            occupied_thresh=float(yaml_meta['occupied_thresh']),
            free_thresh=float(yaml_meta['free_thresh']),
            frame_id=resolved_frame_id,
        )

        checksum_pairs = {
            'yaml_sha256': yaml_meta['yaml_sha256'],
            'pgm_sha256': pgm_meta['pgm_sha256'],
            'content_sha256': pgm_meta['content_sha256'],
            'posegraph_sha256': pg_sha,
            'posegraph_data_sha256': pg_data_sha,
            'bundle_sha256': bundle_sha256,
        }
        if expected_checksums:
            for key, expected_val in expected_checksums.items():
                if key not in checksum_pairs:
                    raise ValidationError(f'unknown checksum expectation key: {key}')
                actual_val = checksum_pairs[key]
                if actual_val is None or actual_val != expected_val:
                    raise ChecksumError(
                        f'checksum mismatch for {key}',
                        {'field': key, 'expected': expected_val, 'actual': actual_val},
                    )

        compat_report: dict[str, Any] = {'checked': False}
        if transform_rmf_from_nav2 is not None:
            compat_report = validate_rmf_compatibility(
                {
                    'width': pgm_meta['width'],
                    'height': pgm_meta['height'],
                    'resolution': yaml_meta['resolution'],
                    'origin_x': yaml_meta['origin_x'],
                    'origin_y': yaml_meta['origin_y'],
                    'origin_yaw': yaml_meta['origin_yaw'],
                },
                pgm_meta['canonical_bytes'],
                transform_rmf_from_nav2,
                nav_graph=nav_graph,
                floor_level_name=floor_level_name,
                free_clearance_m=free_clearance_m,
            )
            compat_report['checked'] = True
            if verification_status == 'VERIFIED' and not compat_report['compatible']:
                raise PolicyError(
                    'cannot register as VERIFIED when Open-RMF compatibility check fails',
                    {'compatibility': compat_report},
                )

        map_version_id = validate_map_version_id(make_map_version_id(label, pgm_meta['content_sha256']))
        # Storage key strips '#' into '__' for filesystem neutrality while preserving bundle uniqueness.
        storage_key = f"{validate_entity_id(label, 'label')}__{pgm_meta['content_sha256'][:12]}__{bundle_sha256[:8]}"

        return {
            'floor_id': floor_id,
            'label': label,
            'map_version_id': map_version_id,
            'storage_key': storage_key,
            'source_mode': source_mode,
            'verification_status': verification_status,
            'verification_evidence': verification_evidence or {},
            'yaml_meta': yaml_meta,
            'pgm_meta': pgm_meta,
            'posegraph_sha256': pg_sha,
            'posegraph_data_sha256': pg_data_sha,
            'frame_id': resolved_frame_id,
            'bundle_sha256': bundle_sha256,
            'rotated_map_frame': 1 if abs(yaml_meta['origin_yaw']) > ORIGIN_YAW_STRICT_TOL_RAD else 0,
            'compat_report': compat_report,
        }

    def _matches_existing_version(self, existing: dict[str, Any], prepared: dict[str, Any]) -> bool:
        ym = prepared['yaml_meta']
        pm = prepared['pgm_meta']
        return (
            existing['floor_id'] == prepared['floor_id']
            and existing['label'] == prepared['label']
            and existing['source_mode'] == prepared['source_mode']
            and existing['content_sha256'] == pm['content_sha256']
            and existing['yaml_sha256'] == ym['yaml_sha256']
            and existing['pgm_sha256'] == pm['pgm_sha256']
            and existing['posegraph_sha256'] == prepared['posegraph_sha256']
            and existing['posegraph_data_sha256'] == prepared['posegraph_data_sha256']
            and str(existing.get('frame_id') or 'map') == prepared['frame_id']
            and str(existing.get('bundle_sha256') or '') == prepared['bundle_sha256']
            and int(existing['width']) == int(pm['width'])
            and int(existing['height']) == int(pm['height'])
            and float(existing['resolution']) == float(ym['resolution'])
            and float(existing['origin_x']) == float(ym['origin_x'])
            and float(existing['origin_y']) == float(ym['origin_y'])
            and float(existing['origin_yaw']) == float(ym['origin_yaw'])
        )

    def register_map_bundle(
        self,
        *,
        floor_id: str,
        label: str,
        yaml_bytes: bytes,
        pgm_bytes: bytes,
        source_mode: str = 'IMPORTED_STATIC',
        verification_status: str = 'UNVERIFIED',
        verification_evidence: dict[str, Any] | None = None,
        posegraph_bytes: bytes | None = None,
        posegraph_data_bytes: bytes | None = None,
        expected_checksums: dict[str, str] | None = None,
        transform_rmf_from_nav2: dict[str, Any] | None = None,
        nav_graph: dict[str, Any] | None = None,
        free_clearance_m: float = 0.0,
        frame_id: str | None = None,
        actor: str = 'system',
        actor_role: str = 'system',
        _after_stage_hook: Any = None,
    ) -> dict[str, Any]:
        """Atomically validate, stage, fsync, and register an immutable MapVersion bundle.

        Registering never modifies `floor_active_maps` or `robot_map_bindings`.
        """
        flid = validate_entity_id(floor_id, 'floor_id')
        clean_label = validate_entity_id(label, 'label')

        floor_row = self.one('SELECT * FROM floors WHERE floor_id=?', (flid,))
        if floor_row is None:
            raise NotFoundError('floor_id not found', {'floor_id': flid})

        prepared = self._validate_bundle_payload(
            floor_id=flid,
            label=clean_label,
            yaml_bytes=yaml_bytes,
            pgm_bytes=pgm_bytes,
            source_mode=source_mode,
            verification_status=verification_status,
            verification_evidence=verification_evidence,
            posegraph_bytes=posegraph_bytes,
            posegraph_data_bytes=posegraph_data_bytes,
            expected_checksums=expected_checksums,
            transform_rmf_from_nav2=transform_rmf_from_nav2,
            nav_graph=nav_graph,
            floor_level_name=str(floor_row['level_name']),
            free_clearance_m=free_clearance_m,
            frame_id=frame_id,
        )

        mvid = prepared['map_version_id']
        with self.lock:
            existing = self.one('SELECT * FROM map_versions WHERE map_version_id=?', (mvid,))
            if existing is not None:
                if not self._matches_existing_version(existing, prepared):
                    raise ConflictError(
                        'map_version_id already registered with differing bundle checksums or metadata',
                        {'map_version_id': mvid},
                    )
                integrity = self.verify_bundle_integrity(mvid)
                if not integrity['ok']:
                    raise IntegrityFailureError(
                        'existing bundle failed integrity check during idempotent registration',
                        {'map_version_id': mvid, 'integrity': integrity},
                    )
                return {
                    'created': False,
                    'idempotent': True,
                    'map_version': self.get_map_version(mvid),
                }

        op_dir = os.path.join(self.staging_dir, f'stage_{uuid.uuid4().hex}')
        final_dir = os.path.join(self.bundles_dir, prepared['storage_key'])

        ym = prepared['yaml_meta']
        pm = prepared['pgm_meta']
        manifest = {
            'schema': 'map_bundle_manifest/v1',
            'map_version_id': mvid,
            'floor_id': flid,
            'label': clean_label,
            'source_mode': source_mode,
            'frame_id': prepared['frame_id'],
            'width': pm['width'],
            'height': pm['height'],
            'resolution': ym['resolution'],
            'origin': ym['origin'],
            'sha256': {
                'content_sha256': pm['content_sha256'],
                'bundle_sha256': prepared['bundle_sha256'],
                'yaml_sha256': ym['yaml_sha256'],
                'pgm_sha256': pm['pgm_sha256'],
                'posegraph_sha256': prepared['posegraph_sha256'],
                'posegraph_data_sha256': prepared['posegraph_data_sha256'],
            },
        }
        manifest_bytes = json.dumps(manifest, indent=2, sort_keys=True).encode('utf-8')

        moved_to_final = False
        try:
            os.makedirs(op_dir, mode=0o755, exist_ok=False)
            _write_readonly_atomic_file(op_dir, 'map.yaml', bytes(yaml_bytes))
            _write_readonly_atomic_file(op_dir, 'map.pgm', bytes(pgm_bytes))
            if posegraph_bytes is not None:
                _write_readonly_atomic_file(op_dir, 'map.posegraph', bytes(posegraph_bytes))
            if posegraph_data_bytes is not None:
                _write_readonly_atomic_file(op_dir, 'map.posegraph.data', bytes(posegraph_data_bytes))
            manifest_path = os.path.join(op_dir, 'manifest.json')
            fd = os.open(manifest_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o444)
            try:
                os.write(fd, manifest_bytes)
                os.fsync(fd)
            finally:
                os.close(fd)
            _fsync_dir(op_dir)

            if _after_stage_hook is not None:
                _after_stage_hook(op_dir)

            with self.transaction():
                # Re-check duplicate inside write lock before touching bundles/.
                existing2 = self.one('SELECT * FROM map_versions WHERE map_version_id=?', (mvid,))
                if existing2 is not None:
                    if not self._matches_existing_version(existing2, prepared):
                        raise ConflictError(
                            'map_version_id registered concurrently with different content',
                            {'map_version_id': mvid},
                        )
                    integrity2 = self.verify_bundle_integrity(mvid)
                    if not integrity2['ok']:
                        raise IntegrityFailureError(
                            'existing bundle failed integrity check during concurrent idempotent registration',
                            {'map_version_id': mvid, 'integrity': integrity2},
                        )
                    return {
                        'created': False,
                        'idempotent': True,
                        'map_version': self.get_map_version(mvid),
                    }

                if os.path.exists(final_dir) or os.path.islink(final_dir):
                    raise ConflictError(
                        'unregistered orphan bundle directory already exists on disk; run reconcile_storage()',
                        {'storage_key': prepared['storage_key']},
                    )

                now = time.time()
                verified_by = actor if verification_status == 'VERIFIED' else None
                verified_at = now if verification_status == 'VERIFIED' else None
                evidence_json = json.dumps(prepared['verification_evidence'], sort_keys=True)
                compat_json = json.dumps(prepared['compat_report'], sort_keys=True)
                self.x(
                    'INSERT INTO map_versions('
                    'map_version_id, floor_id, label, source_mode, verification_status, '
                    'verification_evidence_json, storage_key, width, height, resolution, '
                    'origin_x, origin_y, origin_yaw, negate, occupied_thresh, free_thresh, '
                    'content_sha256, yaml_sha256, pgm_sha256, posegraph_sha256, posegraph_data_sha256, '
                    'frame_id, bundle_sha256, rotated_map_frame, rmf_compatibility_json, '
                    'created_by, created_at, verified_by, verified_at'
                    ') VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)',
                    (
                        mvid,
                        flid,
                        clean_label,
                        source_mode,
                        verification_status,
                        evidence_json,
                        prepared['storage_key'],
                        pm['width'],
                        pm['height'],
                        ym['resolution'],
                        ym['origin_x'],
                        ym['origin_y'],
                        ym['origin_yaw'],
                        ym['negate'],
                        ym['occupied_thresh'],
                        ym['free_thresh'],
                        pm['content_sha256'],
                        ym['yaml_sha256'],
                        pm['pgm_sha256'],
                        prepared['posegraph_sha256'],
                        prepared['posegraph_data_sha256'],
                        prepared['frame_id'],
                        prepared['bundle_sha256'],
                        prepared['rotated_map_frame'],
                        compat_json,
                        actor,
                        now,
                        verified_by,
                        verified_at,
                    ),
                )
                self.x(
                    'INSERT INTO map_verification_history('
                    'ts, map_version_id, from_status, to_status, actor, actor_role, reason, evidence_json, rmf_compatibility_json'
                    ') VALUES(?,?,?,?,?,?,?,?,?)',
                    (
                        now,
                        mvid,
                        'NONE',
                        verification_status,
                        actor,
                        str(actor_role or 'system'),
                        'map bundle registered',
                        evidence_json,
                        compat_json,
                    ),
                )
                self._audit(
                    actor,
                    'MAP_VERSION_REGISTERED',
                    mvid,
                    {
                        'floor_id': flid,
                        'source_mode': source_mode,
                        'verification_status': verification_status,
                        'frame_id': prepared['frame_id'],
                        'content_sha256': pm['content_sha256'],
                        'bundle_sha256': prepared['bundle_sha256'],
                    },
                )
                os.replace(op_dir, final_dir)
                moved_to_final = True
                _fsync_dir(self.bundles_dir)
        except Exception:
            if moved_to_final and os.path.exists(final_dir):
                shutil.rmtree(final_dir, ignore_errors=True)
            if os.path.exists(op_dir):
                shutil.rmtree(op_dir, ignore_errors=True)
            raise
        finally:
            if os.path.exists(op_dir):
                shutil.rmtree(op_dir, ignore_errors=True)

        return {
            'created': True,
            'idempotent': False,
            'map_version': self.get_map_version(mvid),
        }

    def register_map_bundle_from_dir(
        self,
        *,
        floor_id: str,
        label: str,
        bundle_dir: str,
        yaml_filename: str = 'map.yaml',
        pgm_filename: str | None = None,
        posegraph_filename: str | None = None,
        posegraph_data_filename: str | None = None,
        **kwargs: Any,
    ) -> dict[str, Any]:
        """Safely read map files from `bundle_dir` (rejecting symlinks & traversal) and register."""
        if not isinstance(bundle_dir, str) or not os.path.isdir(bundle_dir):
            raise ValidationError('bundle_dir does not exist', {'bundle_dir': bundle_dir})
        if os.path.islink(bundle_dir):
            raise SecurityError('bundle_dir symlink is forbidden')

        yaml_allow = frozenset({yaml_filename})
        validate_safe_filename(yaml_filename, allowset=yaml_allow)
        yaml_path = os.path.join(bundle_dir, yaml_filename)
        if os.path.islink(yaml_path):
            raise SecurityError('symlink map.yaml rejected')
        yaml_real = os.path.realpath(yaml_path)
        if os.path.dirname(yaml_real) != os.path.realpath(bundle_dir):
            raise SecurityError('yaml path escapes bundle_dir')
        if not os.path.isfile(yaml_real):
            raise NotFoundError('map.yaml missing in bundle_dir')
        with open(yaml_real, 'rb') as f:
            raw_yaml = f.read(MAX_YAML_BYTES + 1)

        # Peek `image:` in YAML safely before loading PGM; normalize to canonical `map.pgm` if needed.
        expected_img = pgm_filename
        if expected_img is None:
            peek_meta = parse_ros_map_yaml(raw_yaml, expected_image_name='')
            expected_img = peek_meta['image']
        else:
            validate_safe_filename(expected_img, allowset=frozenset({expected_img}))

        pgm_path = os.path.join(bundle_dir, expected_img)
        if os.path.islink(pgm_path):
            raise SecurityError('symlink PGM file rejected')
        pgm_real = os.path.realpath(pgm_path)
        if os.path.dirname(pgm_real) != os.path.realpath(bundle_dir):
            raise SecurityError('PGM path escapes bundle_dir')
        if not os.path.isfile(pgm_real):
            raise NotFoundError('PGM file missing in bundle_dir', {'image': expected_img})
        with open(pgm_real, 'rb') as f:
            pgm_bytes = f.read(MAX_PGM_BYTES + 1)

        # Normalize `image:` to `map.pgm` inside canonical stored bundle ONLY if YAML used a clean basename like `<label>.pgm`,
        # EXCEPT if caller explicitly supplied expected yaml_sha256 matching source bytes!
        yaml_to_store = raw_yaml
        if expected_img != 'map.pgm' and not (kwargs.get('expected_checksums') or {}).get('yaml_sha256'):
            yaml_text = raw_yaml.decode('utf-8')
            yaml_to_store = re.sub(
                r'(?m)^(\s*image\s*:\s*).*$',
                r'\1map.pgm',
                yaml_text,
                count=1,
            ).encode('utf-8')

        pg_bytes: bytes | None = None
        if posegraph_filename is not None:
            validate_safe_filename(posegraph_filename, allowset=frozenset({posegraph_filename}))
            pg_path = os.path.join(bundle_dir, posegraph_filename)
            if os.path.islink(pg_path):
                raise SecurityError('symlink posegraph rejected')
            pg_real = os.path.realpath(pg_path)
            if os.path.dirname(pg_real) != os.path.realpath(bundle_dir):
                raise SecurityError('posegraph escapes bundle_dir')
            with open(pg_real, 'rb') as f:
                pg_bytes = f.read(MAX_POSEGRAPH_BYTES + 1)

        pg_data_bytes: bytes | None = None
        if posegraph_data_filename is not None:
            validate_safe_filename(posegraph_data_filename, allowset=frozenset({posegraph_data_filename}))
            pgd_path = os.path.join(bundle_dir, posegraph_data_filename)
            if os.path.islink(pgd_path):
                raise SecurityError('symlink posegraph.data rejected')
            pgd_real = os.path.realpath(pgd_path)
            if os.path.dirname(pgd_real) != os.path.realpath(bundle_dir):
                raise SecurityError('posegraph.data escapes bundle_dir')
            with open(pgd_real, 'rb') as f:
                pg_data_bytes = f.read(MAX_POSEGRAPH_BYTES + 1)

        return self.register_map_bundle(
            floor_id=floor_id,
            label=label,
            yaml_bytes=yaml_to_store,
            pgm_bytes=pgm_bytes,
            posegraph_bytes=pg_bytes,
            posegraph_data_bytes=pg_data_bytes,
            **kwargs,
        )

    # ------------------------------------------------------------------
    # Integrity Verification & Verification State Transitions
    # ------------------------------------------------------------------

    def verify_bundle_integrity(self, map_version_id: str) -> dict[str, Any]:
        """Re-read bundle files on disk and check SHA-256 + canonical content SHA-256."""
        mvid = validate_map_version_id(map_version_id)
        row = self.one('SELECT * FROM map_versions WHERE map_version_id=?', (mvid,))
        if row is None:
            raise NotFoundError('map_version_id not found', {'map_version_id': mvid})

        bdir = os.path.join(self.bundles_dir, str(row['storage_key']))
        if not os.path.isdir(bdir) or os.path.islink(bdir):
            return {'ok': False, 'map_version_id': mvid, 'reason': 'bundle directory missing or symlink'}

        checks: dict[str, bool] = {}
        try:
            yaml_path = _safe_contained_path(bdir, 'map.yaml')
            pgm_path = _safe_contained_path(bdir, 'map.pgm')
            with open(yaml_path, 'rb') as f:
                yaml_bytes = f.read()
            with open(pgm_path, 'rb') as f:
                pgm_bytes = f.read()

            yaml_sha = _sha256_bytes(yaml_bytes)
            pgm_parsed = parse_pgm_p5(pgm_bytes)
            checks['yaml_sha256'] = yaml_sha == row['yaml_sha256']
            checks['pgm_sha256'] = pgm_parsed['pgm_sha256'] == row['pgm_sha256']
            checks['content_sha256'] = pgm_parsed['content_sha256'] == row['content_sha256']
            checks['dimensions'] = (
                pgm_parsed['width'] == int(row['width']) and pgm_parsed['height'] == int(row['height'])
            )

            if row['posegraph_sha256'] is not None:
                pg_path = _safe_contained_path(bdir, 'map.posegraph')
                with open(pg_path, 'rb') as f:
                    checks['posegraph_sha256'] = _sha256_bytes(f.read()) == row['posegraph_sha256']
            if row['posegraph_data_sha256'] is not None:
                pgd_path = _safe_contained_path(bdir, 'map.posegraph.data')
                with open(pgd_path, 'rb') as f:
                    checks['posegraph_data_sha256'] = _sha256_bytes(f.read()) == row['posegraph_data_sha256']
        except (OSError, MapRegistryError) as exc:
            return {'ok': False, 'map_version_id': mvid, 'reason': str(exc), 'checks': checks}

        ok = all(checks.values())
        return {
            'ok': ok,
            'map_version_id': mvid,
            'checks': checks,
            'reason': 'ok' if ok else 'checksum or geometry mismatch on disk',
        }

    def _quarantine_summary(self, qrow: dict[str, Any] | None) -> dict[str, Any]:
        if qrow is None:
            return {
                'is_quarantined': False,
                'status': 'NONE',
                'reason': None,
                'detail': None,
                'quarantined_by': None,
                'quarantined_at': None,
                'cleared_by': None,
                'cleared_reason': None,
                'cleared_at': None,
            }
        return {
            'is_quarantined': qrow['status'] == 'ACTIVE',
            'status': qrow['status'],
            'reason': qrow['reason'],
            'detail': json.loads(qrow['detail_json']),
            'quarantined_by': qrow['quarantined_by'],
            'quarantined_at': qrow['quarantined_at'],
            'cleared_by': qrow['cleared_by'],
            'cleared_reason': qrow['cleared_reason'],
            'cleared_at': qrow['cleared_at'],
        }

    def get_verification_history(self, map_version_id: str) -> list[dict[str, Any]]:
        """Return append-only verification/quarantine audit history for a map version."""
        mvid = validate_map_version_id(map_version_id)
        if self.one('SELECT map_version_id FROM map_versions WHERE map_version_id=?', (mvid,)) is None:
            raise NotFoundError('map_version_id not found', {'map_version_id': mvid})
        rows = self.q(
            'SELECT * FROM map_verification_history WHERE map_version_id=? ORDER BY seq ASC',
            (mvid,),
        )
        out: list[dict[str, Any]] = []
        for r in rows:
            out.append({
                'seq': int(r['seq']),
                'ts': float(r['ts']),
                'map_version_id': r['map_version_id'],
                'from_status': r['from_status'],
                'to_status': r['to_status'],
                'actor': r['actor'],
                'actor_role': r['actor_role'],
                'reason': r['reason'],
                'evidence': json.loads(r['evidence_json']),
                'rmf_compatibility': json.loads(r['rmf_compatibility_json']),
            })
        return out

    def find_maps_by_content_sha256(self, content_sha256: str) -> list[dict[str, Any]]:
        """Find all immutable MapVersions sharing the same canonical pixel SHA-256."""
        if not isinstance(content_sha256, str) or len(content_sha256) != 64:
            raise ValidationError('content_sha256 must be a 64-char hex digest')
        rows = self.q(
            'SELECT * FROM map_versions WHERE content_sha256=? ORDER BY created_at DESC, map_version_id ASC',
            (content_sha256.lower(),),
        )
        return self._hydrate_rows(rows)

    def get_map_by_bundle_sha256(self, bundle_sha256: str) -> dict[str, Any]:
        """Look up a specific immutable MapVersion by full `bundle_sha256`."""
        if not isinstance(bundle_sha256, str) or len(bundle_sha256) != 64:
            raise ValidationError('bundle_sha256 must be a 64-char hex digest')
        row = self.one('SELECT * FROM map_versions WHERE bundle_sha256=?', (bundle_sha256.lower(),))
        if row is None:
            raise NotFoundError('bundle_sha256 not found', {'bundle_sha256': bundle_sha256})
        return self._hydrate_rows([row])[0]

    def verify_map_version(
        self,
        map_version_id: str,
        *,
        status: str,
        evidence: dict[str, Any] | None = None,
        actor: str = 'operator',
        actor_role: str = 'operator',
        reason: str = '',
        allow_reverify: bool = False,
        transform_rmf_from_nav2: dict[str, Any] | None = None,
        nav_graph: dict[str, Any] | None = None,
        free_clearance_m: float = 0.0,
    ) -> dict[str, Any]:
        """Promote an UNVERIFIED map to VERIFIED (after integrity + policy checks) or REJECTED."""
        mvid = validate_map_version_id(map_version_id)
        if status not in ('VERIFIED', 'REJECTED'):
            raise ValidationError('verification transition target must be VERIFIED or REJECTED')

        row = self.one('SELECT * FROM map_versions WHERE map_version_id=?', (mvid,))
        if row is None:
            raise NotFoundError('map_version_id not found', {'map_version_id': mvid})

        qrow = self.one('SELECT * FROM map_quarantines WHERE map_version_id=?', (mvid,))
        if status == 'VERIFIED' and qrow is not None and qrow['status'] == 'ACTIVE':
            raise PolicyError(
                'quarantined map version cannot be promoted to VERIFIED; clear quarantine first',
                {'map_version_id': mvid, 'quarantine_reason': qrow['reason']},
            )

        if row['verification_status'] == status:
            return self.get_map_version(mvid)

        if row['verification_status'] == 'REVOKED' and status == 'VERIFIED' and not allow_reverify:
            raise PolicyError(
                'revoked map version cannot be re-verified without explicit allow_reverify=True',
                {'map_version_id': mvid},
            )

        if status == 'VERIFIED':
            if row['source_mode'] == 'SLAM_LIVE_SNAPSHOT':
                raise PolicyError('SLAM_LIVE_SNAPSHOT maps can never be marked VERIFIED')
            if not isinstance(evidence, dict) or not evidence.get('method'):
                raise PolicyError('VERIFIED transition requires evidence with `method`')
            integrity = self.verify_bundle_integrity(mvid)
            if not integrity['ok']:
                raise IntegrityFailureError('bundle failed disk integrity check', {'integrity': integrity})

        compat_report = json.loads(row['rmf_compatibility_json'])
        if transform_rmf_from_nav2 is not None:
            floor_row = self.one('SELECT * FROM floors WHERE floor_id=?', (row['floor_id'],))
            pgm_bytes = self.read_bundle_file(mvid, 'map.pgm')
            canon = parse_pgm_p5(pgm_bytes)['canonical_bytes']
            compat_report = validate_rmf_compatibility(
                row,
                canon,
                transform_rmf_from_nav2,
                nav_graph=nav_graph,
                floor_level_name=str(floor_row['level_name']) if floor_row else None,
                free_clearance_m=free_clearance_m,
            )
            compat_report['checked'] = True
            if status == 'VERIFIED' and not compat_report['compatible']:
                raise PolicyError(
                    'cannot verify map version because Open-RMF compatibility check failed',
                    {'compatibility': compat_report},
                )

        now = time.time()
        from_status = str(row['verification_status'])
        evidence_json = json.dumps(evidence or {}, sort_keys=True)
        compat_json = json.dumps(compat_report, sort_keys=True)
        hist_reason = reason.strip() if isinstance(reason, str) and reason.strip() else f'transitioned to {status}'
        with self.transaction():
            self.x(
                'UPDATE map_versions SET verification_status=?, verification_evidence_json=?, '
                'rmf_compatibility_json=?, verified_by=?, verified_at=? WHERE map_version_id=?',
                (
                    status,
                    evidence_json,
                    compat_json,
                    actor,
                    now,
                    mvid,
                ),
            )
            self.x(
                'INSERT INTO map_verification_history('
                'ts, map_version_id, from_status, to_status, actor, actor_role, reason, evidence_json, rmf_compatibility_json'
                ') VALUES(?,?,?,?,?,?,?,?,?)',
                (
                    now,
                    mvid,
                    from_status,
                    status,
                    actor,
                    str(actor_role or 'operator'),
                    hist_reason,
                    evidence_json,
                    compat_json,
                ),
            )
            self._audit(actor, f'MAP_VERSION_{status}', mvid, {'from_status': from_status, 'reason': hist_reason, 'evidence': evidence or {}})
        return self.get_map_version(mvid)

    def revoke_map_verification(
        self,
        map_version_id: str,
        *,
        reason: str,
        actor: str = 'operator',
        actor_role: str = 'operator',
        evidence: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Revoke a VERIFIED map version cleanly without deleting files or history.

        Any `floor_active_maps` pointer referencing this version is cleared immediately.
        """
        mvid = validate_map_version_id(map_version_id)
        if not isinstance(reason, str) or not reason.strip():
            raise ValidationError('revocation reason must be non-empty')
        clean_reason = reason.strip()

        row = self.one('SELECT * FROM map_versions WHERE map_version_id=?', (mvid,))
        if row is None:
            raise NotFoundError('map_version_id not found', {'map_version_id': mvid})
        from_status = str(row['verification_status'])
        if from_status == 'REVOKED':
            return {
                'revoked': False,
                'idempotent': True,
                'cleared_floors': [],
                'map_version': self.get_map_version(mvid),
            }
        if from_status != 'VERIFIED':
            raise PolicyError(
                'only VERIFIED map versions can be revoked',
                {'map_version_id': mvid, 'verification_status': from_status},
            )

        now = time.time()
        ev_payload = dict(evidence or {})
        ev_payload.setdefault('method', 'operator_revocation')
        ev_payload['revocation_reason'] = clean_reason
        with self.transaction():
            self.x(
                'UPDATE map_versions SET verification_status=? WHERE map_version_id=?',
                ('REVOKED', mvid),
            )
            self.x(
                'INSERT INTO map_verification_history('
                'ts, map_version_id, from_status, to_status, actor, actor_role, reason, evidence_json, rmf_compatibility_json'
                ') VALUES(?,?,?,?,?,?,?,?,?)',
                (
                    now,
                    mvid,
                    from_status,
                    'REVOKED',
                    actor,
                    str(actor_role or 'operator'),
                    clean_reason,
                    json.dumps(ev_payload, sort_keys=True),
                    row['rmf_compatibility_json'],
                ),
            )
            active_ptrs = self.q('SELECT floor_id FROM floor_active_maps WHERE active_map_version_id=?', (mvid,))
            cleared_floors = [str(p['floor_id']) for p in active_ptrs]
            for flid in cleared_floors:
                self.x('DELETE FROM floor_active_maps WHERE floor_id=?', (flid,))
                self._audit(
                    actor,
                    'FLOOR_ACTIVE_MAP_CLEARED_ON_REVOKE',
                    flid,
                    {'map_version_id': mvid, 'reason': clean_reason},
                )
            self._audit(actor, 'MAP_VERSION_REVOKED', mvid, {'reason': clean_reason, 'cleared_floors': cleared_floors})
        return {
            'revoked': True,
            'idempotent': False,
            'cleared_floors': cleared_floors,
            'map_version': self.get_map_version(mvid),
        }

    def quarantine_map_version(
        self,
        map_version_id: str,
        *,
        reason: str,
        detail: dict[str, Any] | None = None,
        actor: str = 'operator',
        actor_role: str = 'operator',
    ) -> dict[str, Any]:
        """Quarantine a map version without deleting bundle files or MapVersion metadata.

        Clears any `floor_active_maps` pointer referencing the quarantined version.
        """
        mvid = validate_map_version_id(map_version_id)
        if not isinstance(reason, str) or not reason.strip():
            raise ValidationError('quarantine reason must be non-empty')
        clean_reason = reason.strip()

        row = self.one('SELECT * FROM map_versions WHERE map_version_id=?', (mvid,))
        if row is None:
            raise NotFoundError('map_version_id not found', {'map_version_id': mvid})

        now = time.time()
        detail_json = json.dumps(detail or {}, sort_keys=True)
        cur_status = str(row['verification_status'])
        with self.transaction():
            self.x(
                'INSERT INTO map_quarantines('
                'map_version_id, status, reason, detail_json, quarantined_by, quarantined_at, cleared_by, cleared_reason, cleared_at'
                ') VALUES(?,?,?,?,?,?,?,?,?) '
                'ON CONFLICT(map_version_id) DO UPDATE SET '
                'status=excluded.status, reason=excluded.reason, detail_json=excluded.detail_json, '
                'quarantined_by=excluded.quarantined_by, quarantined_at=excluded.quarantined_at, '
                'cleared_by=NULL, cleared_reason=NULL, cleared_at=NULL',
                (mvid, 'ACTIVE', clean_reason, detail_json, actor, now, None, None, None),
            )
            self.x(
                'INSERT INTO map_verification_history('
                'ts, map_version_id, from_status, to_status, actor, actor_role, reason, evidence_json, rmf_compatibility_json'
                ') VALUES(?,?,?,?,?,?,?,?,?)',
                (
                    now,
                    mvid,
                    cur_status,
                    cur_status,
                    actor,
                    str(actor_role or 'operator'),
                    f'QUARANTINED: {clean_reason}',
                    json.dumps({'method': 'quarantine', 'detail': detail or {}}, sort_keys=True),
                    row['rmf_compatibility_json'],
                ),
            )
            active_ptrs = self.q('SELECT floor_id FROM floor_active_maps WHERE active_map_version_id=?', (mvid,))
            cleared_floors = [str(p['floor_id']) for p in active_ptrs]
            for flid in cleared_floors:
                self.x('DELETE FROM floor_active_maps WHERE floor_id=?', (flid,))
                self._audit(
                    actor,
                    'FLOOR_ACTIVE_MAP_CLEARED_ON_QUARANTINE',
                    flid,
                    {'map_version_id': mvid, 'reason': clean_reason},
                )
            self._audit(actor, 'MAP_VERSION_QUARANTINED', mvid, {'reason': clean_reason, 'detail': detail or {}, 'cleared_floors': cleared_floors})
        return {
            'quarantined': True,
            'cleared_floors': cleared_floors,
            'map_version': self.get_map_version(mvid),
        }

    def clear_map_quarantine(
        self,
        map_version_id: str,
        *,
        reason: str,
        actor: str = 'admin',
        actor_role: str = 'admin',
    ) -> dict[str, Any]:
        """Clear an active quarantine ONLY after re-verifying disk bundle integrity."""
        mvid = validate_map_version_id(map_version_id)
        if not isinstance(reason, str) or not reason.strip():
            raise ValidationError('quarantine clear reason must be non-empty')
        clean_reason = reason.strip()

        row = self.one('SELECT * FROM map_versions WHERE map_version_id=?', (mvid,))
        if row is None:
            raise NotFoundError('map_version_id not found', {'map_version_id': mvid})
        qrow = self.one('SELECT * FROM map_quarantines WHERE map_version_id=?', (mvid,))
        if qrow is None or qrow['status'] != 'ACTIVE':
            return {
                'cleared': False,
                'idempotent': True,
                'map_version': self.get_map_version(mvid),
            }

        integrity = self.verify_bundle_integrity(mvid)
        if not integrity['ok']:
            raise IntegrityFailureError(
                'cannot clear quarantine while bundle integrity check fails on disk',
                {'map_version_id': mvid, 'integrity': integrity},
            )

        now = time.time()
        cur_status = str(row['verification_status'])
        with self.transaction():
            self.x(
                'UPDATE map_quarantines SET status=?, cleared_by=?, cleared_reason=?, cleared_at=? '
                'WHERE map_version_id=?',
                ('CLEARED', actor, clean_reason, now, mvid),
            )
            self.x(
                'INSERT INTO map_verification_history('
                'ts, map_version_id, from_status, to_status, actor, actor_role, reason, evidence_json, rmf_compatibility_json'
                ') VALUES(?,?,?,?,?,?,?,?,?)',
                (
                    now,
                    mvid,
                    cur_status,
                    cur_status,
                    actor,
                    str(actor_role or 'admin'),
                    f'QUARANTINE_CLEARED: {clean_reason}',
                    json.dumps({'method': 'quarantine_clear'}, sort_keys=True),
                    row['rmf_compatibility_json'],
                ),
            )
            self._audit(actor, 'MAP_VERSION_QUARANTINE_CLEARED', mvid, {'reason': clean_reason})
        return {
            'cleared': True,
            'idempotent': False,
            'map_version': self.get_map_version(mvid),
        }

    # ------------------------------------------------------------------
    # Mutable Floor Pointer (`floor_active_maps`) — Decoupled from Content
    # ------------------------------------------------------------------

    def set_floor_active_map(
        self,
        floor_id: str,
        map_version_id: str,
        *,
        actor: str = 'operator',
        reason: str = '',
    ) -> dict[str, Any]:
        """Set or switch a floor's default active map pointer (VERIFIED, non-quarantined maps only).

        Never modifies `map_versions` content or existing versions, and never
        automatically switches robots or grants mission permissions.
        """
        flid = validate_entity_id(floor_id, 'floor_id')
        mvid = validate_map_version_id(map_version_id)
        if not isinstance(reason, str) or not reason.strip():
            raise ValidationError('activation reason must be non-empty')

        row = self.one('SELECT * FROM map_versions WHERE map_version_id=?', (mvid,))
        if row is None:
            raise NotFoundError('map_version_id not found', {'map_version_id': mvid})
        if row['floor_id'] != flid:
            raise ConflictError(
                'map_version_id belongs to a different floor',
                {'floor_id': flid, 'map_floor_id': row['floor_id']},
            )
        if row['verification_status'] != 'VERIFIED':
            raise PolicyError(
                'only VERIFIED map versions may be activated for a floor',
                {'map_version_id': mvid, 'verification_status': row['verification_status']},
            )
        integrity = self.verify_bundle_integrity(mvid)
        if not integrity['ok']:
            raise IntegrityFailureError('cannot activate corrupted map bundle', {'integrity': integrity})
        qrow = self.one('SELECT * FROM map_quarantines WHERE map_version_id=?', (mvid,))
        if qrow is not None and qrow['status'] == 'ACTIVE':
            raise PolicyError(
                'quarantined map version cannot be activated for a floor',
                {'map_version_id': mvid, 'quarantine_reason': qrow['reason']},
            )

        now = time.time()
        with self.transaction():
            cur = self.one('SELECT * FROM floor_active_maps WHERE floor_id=?', (flid,))
            if cur is None:
                rev = 1
                self.x(
                    'INSERT INTO floor_active_maps(floor_id, active_map_version_id, pointer_revision, activated_by, activated_at, reason) '
                    'VALUES(?,?,?,?,?,?)',
                    (flid, mvid, rev, actor, now, reason),
                )
            else:
                if cur['active_map_version_id'] == mvid:
                    return self.get_floor_active_map(flid)  # idempotent
                rev = int(cur['pointer_revision']) + 1
                self.x(
                    'UPDATE floor_active_maps SET active_map_version_id=?, pointer_revision=?, '
                    'activated_by=?, activated_at=?, reason=? WHERE floor_id=?',
                    (mvid, rev, actor, now, reason, flid),
                )
            self._audit(
                actor,
                'FLOOR_ACTIVE_MAP_SET',
                flid,
                {'map_version_id': mvid, 'pointer_revision': rev, 'reason': reason},
            )
        return self.get_floor_active_map(flid)

    def get_floor_active_map(self, floor_id: str) -> dict[str, Any]:
        flid = validate_entity_id(floor_id, 'floor_id')
        floor = self.one('SELECT * FROM floors WHERE floor_id=?', (flid,))
        if floor is None:
            raise NotFoundError('floor_id not found', {'floor_id': flid})
        ptr = self.one('SELECT * FROM floor_active_maps WHERE floor_id=?', (flid,))
        if ptr is None:
            return {'floor': floor, 'active': None}
        mv = self.get_map_version(ptr['active_map_version_id'])
        if mv['verification_status'] != 'VERIFIED' or mv['quarantine']['is_quarantined']:
            return {
                'floor': floor,
                'active': None,
                'blocked_active': {
                    'active_map_version_id': ptr['active_map_version_id'],
                    'verification_status': mv['verification_status'],
                    'quarantined': mv['quarantine']['is_quarantined'],
                },
            }
        return {
            'floor': floor,
            'active': {
                'pointer_revision': int(ptr['pointer_revision']),
                'active_map_version_id': ptr['active_map_version_id'],
                'activated_by': ptr['activated_by'],
                'activated_at': ptr['activated_at'],
                'reason': ptr['reason'],
                'map_version': mv,
            },
        }

    # ------------------------------------------------------------------
    # Read Helpers & Integrity-Checked File Retrieval
    # ------------------------------------------------------------------

    def _hydrate_map_version(
        self,
        row: dict[str, Any],
        active_id: str | None,
        qrow: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        frame_id = str(row['frame_id']) if 'frame_id' in row and row['frame_id'] else 'map'
        bundle_sha256 = str(row['bundle_sha256']) if 'bundle_sha256' in row and row['bundle_sha256'] else ''
        return {
            'map_version_id': row['map_version_id'],
            'floor_id': row['floor_id'],
            'label': row['label'],
            'source_mode': row['source_mode'],
            'verification_status': row['verification_status'],
            'verification_evidence': json.loads(row['verification_evidence_json']),
            'quarantine': self._quarantine_summary(qrow),
            'storage_key': row['storage_key'],
            'frame_id': frame_id,
            'width': int(row['width']),
            'height': int(row['height']),
            'resolution': float(row['resolution']),
            'origin': [float(row['origin_x']), float(row['origin_y']), float(row['origin_yaw'])],
            'origin_x': float(row['origin_x']),
            'origin_y': float(row['origin_y']),
            'origin_yaw': float(row['origin_yaw']),
            'negate': int(row['negate']),
            'occupied_thresh': float(row['occupied_thresh']),
            'free_thresh': float(row['free_thresh']),
            'rotated_map_frame': bool(row['rotated_map_frame']),
            'bundle_sha256': bundle_sha256,
            'sha256': {
                'content_sha256': row['content_sha256'],
                'bundle_sha256': bundle_sha256,
                'yaml_sha256': row['yaml_sha256'],
                'pgm_sha256': row['pgm_sha256'],
                'posegraph_sha256': row['posegraph_sha256'],
                'posegraph_data_sha256': row['posegraph_data_sha256'],
            },
            'rmf_compatibility': json.loads(row['rmf_compatibility_json']),
            'is_floor_active': row['map_version_id'] == active_id,
            'created_by': row['created_by'],
            'created_at': row['created_at'],
            'verified_by': row['verified_by'],
            'verified_at': row['verified_at'],
        }

    def _hydrate_rows(self, rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
        if not rows:
            return []
        active_by_floor = {
            r['floor_id']: r['active_map_version_id']
            for r in self.q('SELECT floor_id, active_map_version_id FROM floor_active_maps')
        }
        q_by_mvid = {
            r['map_version_id']: r
            for r in self.q('SELECT * FROM map_quarantines')
        }
        area_by_mvid = self._area_links()
        lineage = self._lineage()
        return [
            dict(self._hydrate_map_version(
                r,
                active_by_floor.get(r['floor_id']),
                q_by_mvid.get(r['map_version_id']),
            ), area=area_by_mvid.get(r['map_version_id']), lineage=lineage.get(r['map_version_id']))
            for r in rows
        ]

    def _lineage(self) -> dict[str, dict[str, Any]]:
        return {r['map_version_id']: {'parent_map_version_id': r['parent_map_version_id'], 'mapping_mode': r['mapping_mode'],
                                      'session_id': r['session_id']}
                for r in self.q('SELECT * FROM map_version_lineage')}

    def _area_links(self) -> dict[str, dict[str, Any]]:
        rows = self.q('SELECT l.map_version_id, a.area_id, a.name, a.coverage FROM map_version_areas l '
                      'JOIN map_areas a ON a.area_id = l.area_id')
        return {r['map_version_id']: {'area_id': r['area_id'], 'name': r['name'], 'coverage': r['coverage']} for r in rows}

    def get_map_version(self, map_version_id: str) -> dict[str, Any]:
        mvid = validate_map_version_id(map_version_id)
        row = self.one('SELECT * FROM map_versions WHERE map_version_id=?', (mvid,))
        if row is None:
            raise NotFoundError('map_version_id not found', {'map_version_id': mvid})
        ptr = self.one('SELECT active_map_version_id FROM floor_active_maps WHERE floor_id=?', (row['floor_id'],))
        active_id = ptr['active_map_version_id'] if ptr else None
        qrow = self.one('SELECT * FROM map_quarantines WHERE map_version_id=?', (mvid,))
        return dict(self._hydrate_map_version(row, active_id, qrow), area=self._area_links().get(mvid),
                    lineage=self._lineage().get(mvid))

    def list_map_versions(
        self,
        *,
        floor_id: str | None = None,
        verification_status: str | None = None,
    ) -> list[dict[str, Any]]:
        clauses: list[str] = []
        params: list[Any] = []
        if floor_id is not None:
            clauses.append('floor_id=?')
            params.append(validate_entity_id(floor_id, 'floor_id'))
        if verification_status is not None:
            if verification_status not in VERIFICATION_STATUSES:
                raise ValidationError('invalid verification_status filter')
            clauses.append('verification_status=?')
            params.append(verification_status)
        where = f"WHERE {' AND '.join(clauses)}" if clauses else ''
        rows = self.q(f'SELECT * FROM map_versions {where} ORDER BY created_at DESC, map_version_id ASC', tuple(params))
        return self._hydrate_rows(rows)

    def list_facilities_tree(self) -> list[dict[str, Any]]:
        facilities = self.q('SELECT * FROM facilities ORDER BY facility_id')
        buildings = self.q('SELECT * FROM buildings ORDER BY building_id')
        floors = self.q('SELECT * FROM floors ORDER BY elevation_m, floor_id')
        active_ptrs = {
            r['floor_id']: r for r in self.q('SELECT * FROM floor_active_maps')
        }
        counts = {
            r['floor_id']: int(r['cnt'])
            for r in self.q('SELECT floor_id, COUNT(*) AS cnt FROM map_versions GROUP BY floor_id')
        }
        area_versions: dict[str, list[str]] = {}
        for r in self.q('SELECT area_id, map_version_id FROM map_version_areas ORDER BY linked_at, map_version_id'):
            area_versions.setdefault(r['area_id'], []).append(r['map_version_id'])
        areas_by_floor: dict[str, list[dict[str, Any]]] = {}
        for a in self.q('SELECT * FROM map_areas ORDER BY area_id'):
            areas_by_floor.setdefault(a['floor_id'], []).append(
                dict(a, map_version_ids=area_versions.get(a['area_id'], [])))

        floors_by_b: dict[str, list[dict[str, Any]]] = {}
        for fl in floors:
            fl_copy = dict(fl)
            ptr = active_ptrs.get(fl['floor_id'])
            fl_copy['active_map_version_id'] = ptr['active_map_version_id'] if ptr else None
            fl_copy['pointer_revision'] = int(ptr['pointer_revision']) if ptr else 0
            fl_copy['map_version_count'] = counts.get(fl['floor_id'], 0)
            fl_copy['areas'] = areas_by_floor.get(fl['floor_id'], [])
            floors_by_b.setdefault(fl['building_id'], []).append(fl_copy)

        b_by_f: dict[str, list[dict[str, Any]]] = {}
        for b in buildings:
            b_copy = dict(b)
            b_copy['floors'] = floors_by_b.get(b['building_id'], [])
            b_by_f.setdefault(b['facility_id'], []).append(b_copy)

        result: list[dict[str, Any]] = []
        for fac in facilities:
            fac_copy = dict(fac)
            fac_copy['buildings'] = b_by_f.get(fac['facility_id'], [])
            result.append(fac_copy)
        return result

    def read_bundle_file(self, map_version_id: str, filename: str) -> bytes:
        """Read an immutable bundle file after validating path containment and SHA-256."""
        mvid = validate_map_version_id(map_version_id)
        safe_name = validate_safe_filename(filename)
        row = self.one('SELECT * FROM map_versions WHERE map_version_id=?', (mvid,))
        if row is None:
            raise NotFoundError('map_version_id not found', {'map_version_id': mvid})

        expected_sha_map = {
            'map.yaml': row['yaml_sha256'],
            'map.pgm': row['pgm_sha256'],
            'map.posegraph': row['posegraph_sha256'],
            'map.posegraph.data': row['posegraph_data_sha256'],
        }
        expected_sha = expected_sha_map.get(safe_name)
        if expected_sha is None:
            raise NotFoundError('file not present in this map bundle', {'filename': safe_name})

        bdir = os.path.join(self.bundles_dir, str(row['storage_key']))
        if not os.path.isdir(bdir) or os.path.islink(bdir):
            raise IntegrityFailureError('bundle directory missing or symlink', {'map_version_id': mvid})
        fpath = _safe_contained_path(bdir, safe_name)
        if not os.path.isfile(fpath):
            raise IntegrityFailureError('bundle file missing on disk', {'map_version_id': mvid, 'filename': safe_name})
        with open(fpath, 'rb') as f:
            payload = f.read()
        actual_sha = _sha256_bytes(payload)
        if actual_sha != expected_sha:
            raise IntegrityFailureError(
                'bundle file checksum mismatch on read (possible disk tampering)',
                {'map_version_id': mvid, 'filename': safe_name, 'expected': expected_sha, 'actual': actual_sha},
            )
        return payload

    def _enforce_response_limit(self, payload: bytes, *, context: str) -> None:
        if len(payload) > self.max_http_response_bytes:
            raise PayloadTooLargeError(
                f'{context} exceeds max HTTP response size limit',
                {'bytes': len(payload), 'limit': self.max_http_response_bytes},
            )

    def render_preview_png(self, map_version_id: str) -> bytes:
        """Render integrity-verified canonical occupancy PNG for dashboard preview."""
        # Always verify on-disk map.pgm integrity against DB SHA-256 before serving.
        pgm_bytes = self.read_bundle_file(map_version_id, 'map.pgm')
        pgm_sha256 = _sha256_bytes(pgm_bytes)
        cache_key = (validate_map_version_id(map_version_id), pgm_sha256)

        if self.png_cache_max_items > 0 and self.png_cache_max_bytes > 0:
            with self._cache_lock:
                cached = self._png_cache.get(cache_key)
                if cached is not None:
                    self._png_cache.move_to_end(cache_key)
                    self._enforce_response_limit(cached, context='preview.png')
                    return cached

        with self._render_sem:
            pgm_parsed = parse_pgm_p5(pgm_bytes)
            png_bytes = canonical_to_png_bytes(
                int(pgm_parsed['width']),
                int(pgm_parsed['height']),
                pgm_parsed['canonical_bytes'],
            )

        self._enforce_response_limit(png_bytes, context='preview.png')

        if (
            self.png_cache_max_items > 0
            and self.png_cache_max_bytes > 0
            and len(png_bytes) <= self.png_cache_max_bytes
        ):
            with self._cache_lock:
                if cache_key in self._png_cache:
                    prev = self._png_cache.pop(cache_key)
                    self._png_cache_bytes -= len(prev)
                while self._png_cache and (
                    len(self._png_cache) >= self.png_cache_max_items
                    or self._png_cache_bytes + len(png_bytes) > self.png_cache_max_bytes
                ):
                    _, evicted = self._png_cache.popitem(last=False)
                    self._png_cache_bytes -= len(evicted)
                self._png_cache[cache_key] = png_bytes
                self._png_cache_bytes += len(png_bytes)

        return png_bytes

    def reconcile_storage(self, *, quarantine_corrupted: bool = False) -> dict[str, Any]:
        """Clean orphan `.staging` dirs, verify DB bundles, and report DB-unregistered bundles.

        Never automatically deletes or overwrites directories inside `bundles/`.
        When `quarantine_corrupted=True`, automatically quarantines any corrupted/missing DB bundle.
        """
        cleaned_staging: list[str] = []
        for entry in sorted(os.listdir(self.staging_dir)):
            full = os.path.join(self.staging_dir, entry)
            shutil.rmtree(full, ignore_errors=True)
            cleaned_staging.append(entry)

        healthy: list[str] = []
        corrupted: list[dict[str, Any]] = []
        quarantined_ids: list[str] = []
        known_storage_keys: set[str] = set()
        for row in self.q('SELECT map_version_id, storage_key FROM map_versions ORDER BY map_version_id'):
            mvid = str(row['map_version_id'])
            known_storage_keys.add(str(row['storage_key']))
            res = self.verify_bundle_integrity(mvid)
            if res['ok']:
                healthy.append(mvid)
            else:
                corrupted.append(res)
                if quarantine_corrupted:
                    self.quarantine_map_version(
                        mvid,
                        reason='corrupted_or_missing_on_reconcile',
                        detail=res,
                        actor='reconciler',
                        actor_role='system',
                    )
                    quarantined_ids.append(mvid)

        unregistered_bundles: list[dict[str, Any]] = []
        for entry in sorted(os.listdir(self.bundles_dir)):
            if entry in known_storage_keys:
                continue
            full = os.path.join(self.bundles_dir, entry)
            info: dict[str, Any] = {
                'storage_key': entry,
                'is_dir': os.path.isdir(full) and not os.path.islink(full),
                'is_symlink': os.path.islink(full),
                'has_manifest': False,
                'manifest_map_version_id': None,
                'status': 'ORPHAN_UNREGISTERED_BUNDLE',
            }
            if info['is_dir']:
                mpath = os.path.join(full, 'manifest.json')
                if os.path.isfile(mpath) and not os.path.islink(mpath):
                    info['has_manifest'] = True
                    try:
                        with open(mpath, 'rb') as mf:
                            mdata = json.loads(mf.read(MAX_YAML_BYTES))
                        if isinstance(mdata, dict):
                            info['manifest_map_version_id'] = mdata.get('map_version_id')
                    except (OSError, ValueError):
                        info['status'] = 'ORPHAN_CORRUPT_MANIFEST'
            unregistered_bundles.append(info)

        return {
            'cleaned_staging': cleaned_staging,
            'healthy': healthy,
            'corrupted': corrupted,
            'quarantined': quarantined_ids,
            'unregistered_bundles': unregistered_bundles,
        }

    # ------------------------------------------------------------------
    # Non-Conflicting Readonly & Write REST Contract Dispatchers
    # ------------------------------------------------------------------

    def dispatch_readonly(
        self,
        method: str,
        raw_path: str,
        principal: Any,
        query: dict[str, Any] | None = None,
    ) -> tuple[int, Any, str]:
        """Pure read-only HTTP dispatcher under `/api/v1/map-registry/*`.

        Returns `(status_code, payload_dict_or_bytes, content_type)` using the same
        `{'error': {'code': ..., 'message': ...}}` envelope as `http_api.py`.
        Accepts either `parse_qs` (`dict[str, list[str]]`) or `http_api.py` flat
        query dicts (`dict[str, str]`).
        """
        if method != 'GET':
            return 405, {'error': {'code': 'method_not_allowed', 'message': 'only GET is allowed on readonly dispatcher'}}, 'application/json'
        if principal is None:
            return 401, {'error': {'code': 'unauthorized', 'message': 'missing or invalid bearer token'}}, 'application/json'
        can_read = getattr(principal, 'can_read', None)
        if not callable(can_read) or not can_read():
            return 403, {'error': {'code': 'forbidden', 'message': 'role may not read'}}, 'application/json'

        parsed = urllib.parse.urlsplit(raw_path)
        path = parsed.path.rstrip('/') or '/'
        qs_raw = query if query is not None else urllib.parse.parse_qs(parsed.query)

        def _qget(key: str, default: str | None = None) -> str | None:
            val = qs_raw.get(key)
            if isinstance(val, list):
                return val[0] if val else default
            if isinstance(val, str):
                return val
            return default

        prefix = '/api/v1/map-registry'
        if path != prefix and not path.startswith(prefix + '/'):
            return 404, {'error': {'code': 'not_found', 'message': 'outside map-registry namespace'}}, 'application/json'
        sub = path[len(prefix):]

        try:
            if sub == '/facilities':
                body = {'facilities': self.list_facilities_tree()}
                self._enforce_response_limit(json.dumps(body).encode('utf-8'), context='facilities')
                return 200, body, 'application/json'

            if sub == '/events':
                raw_limit = _qget('limit', '100') or '100'
                try:
                    limit = max(1, min(500, int(raw_limit)))
                except ValueError as exc:
                    raise ValidationError('invalid limit parameter') from exc
                rows = self.q('SELECT * FROM map_registry_events ORDER BY seq DESC LIMIT ?', (limit,))
                for r in rows:
                    r['detail'] = json.loads(r.pop('detail_json'))
                return 200, {'events': rows}, 'application/json'

            if sub == '/quarantines':
                rows = self.q('SELECT * FROM map_quarantines ORDER BY quarantined_at DESC, map_version_id ASC')
                items = [
                    {'map_version_id': r['map_version_id'], **self._quarantine_summary(r)}
                    for r in rows
                ]
                return 200, {'quarantines': items}, 'application/json'

            if sub == '/maps':
                floor_id = _qget('floor_id')
                vstatus = _qget('verification_status')
                body = {
                    'maps': self.list_map_versions(floor_id=floor_id, verification_status=vstatus)
                }
                self._enforce_response_limit(json.dumps(body).encode('utf-8'), context='maps')
                return 200, body, 'application/json'

            parts = [urllib.parse.unquote(p) for p in sub.strip('/').split('/') if p]
            for seg in parts:
                if seg in ('.', '..') or '/' in seg or '\\' in seg or '\x00' in seg:
                    raise SecurityError('invalid path segment in map-registry URI', {'segment': seg})
            if len(parts) == 3 and parts[0] == 'floors' and parts[2] == 'active-map':
                return 200, self.get_floor_active_map(parts[1]), 'application/json'

            if len(parts) >= 2 and parts[0] == 'maps':
                mvid = parts[1]
                if len(parts) == 2:
                    return 200, {'map': self.get_map_version(mvid)}, 'application/json'
                if len(parts) == 3 and parts[2] == 'meta':
                    mv = self.get_map_version(mvid)
                    return 200, {
                        'map_version_id': mv['map_version_id'],
                        'floor_id': mv['floor_id'],
                        'verification_status': mv['verification_status'],
                        'quarantine': mv['quarantine'],
                        'source_mode': mv['source_mode'],
                        'frame_id': mv['frame_id'],
                        'width': mv['width'],
                        'height': mv['height'],
                        'resolution': mv['resolution'],
                        'origin': mv['origin'],
                        'rotated_map_frame': mv['rotated_map_frame'],
                        'bundle_sha256': mv['bundle_sha256'],
                        'sha256': mv['sha256'],
                        'is_floor_active': mv['is_floor_active'],
                        'area': mv.get('area'),
                        'lineage': mv.get('lineage'),
                        'verification_evidence': mv['verification_evidence'],
                    }, 'application/json'
                if len(parts) == 3 and parts[2] == 'history':
                    return 200, {
                        'map_version_id': validate_map_version_id(mvid),
                        'history': self.get_verification_history(mvid),
                    }, 'application/json'
                if len(parts) == 3 and parts[2] == 'preview.png':
                    return 200, self.render_preview_png(mvid), 'image/png'
                if len(parts) == 4 and parts[2] == 'bundle':
                    fname = parts[3]
                    payload = self.read_bundle_file(mvid, fname)
                    self._enforce_response_limit(payload, context=f'bundle/{fname}')
                    ctype = 'text/yaml; charset=utf-8' if fname == 'map.yaml' else 'application/octet-stream'
                    return 200, payload, ctype

            return 404, {'error': {'code': 'not_found', 'message': 'unknown map-registry endpoint'}}, 'application/json'
        except MapRegistryError as exc:
            return exc.status, exc.to_dict(), 'application/json'

    def dispatch_write(self, method: str, raw_path: str, principal: Any, *, body: bytes | bytearray | str | dict[str, Any] | None = None, headers: dict[str, str] | None = None) -> tuple[int, Any, str]:
        """Serialize idempotency check, registration, audit, and receipt in one transaction."""
        class RollbackResponse(Exception):
            def __init__(self, response): self.response = response
        try:
            with self.transaction():
                response = self._dispatch_write_inner(method, raw_path, principal, body=body, headers=headers)
                if response[0] >= 400: raise RollbackResponse(response)
                return response
        except RollbackResponse as exc:
            return exc.response

    def _dispatch_write_inner(
        self,
        method: str,
        raw_path: str,
        principal: Any,
        *,
        body: bytes | bytearray | str | dict[str, Any] | None = None,
        headers: dict[str, str] | None = None,
    ) -> tuple[int, Any, str]:
        """Standalone Phase 2-B write HTTP boundary under `/api/v1/map-registry/*`.

        Supports hierarchy registration (`facilities`, `buildings`, `floors`),
        bounded bundle upload (`POST /maps` defaulting strictly to `UNVERIFIED`),
        verification promotion (`POST /maps/{id}/verify`), revocation
        (`POST /maps/{id}/revoke`), and quarantine management (`quarantine`, `clear`).

        Explicitly does NOT implement live robot map switching, `/initialpose`,
        or SLAM process control.
        """
        if method != 'POST':
            return 405, {'error': {'code': 'method_not_allowed', 'message': 'only POST is supported on write boundary'}}, 'application/json'
        if principal is None:
            return 401, {'error': {'code': 'unauthorized', 'message': 'missing or invalid bearer token'}}, 'application/json'
        can_control = getattr(principal, 'can_control', None)
        if not callable(can_control) or not can_control():
            return 403, {'error': {'code': 'forbidden', 'message': 'role may not write map registry'}}, 'application/json'

        actor_name = str(getattr(principal, 'name', '') or getattr(principal, 'role', '') or 'operator')
        actor_role = str(getattr(principal, 'role', '') or 'operator')

        parsed = urllib.parse.urlsplit(raw_path)
        path = parsed.path.rstrip('/') or '/'
        prefix = '/api/v1/map-registry'
        if path != prefix and not path.startswith(prefix + '/'):
            return 404, {'error': {'code': 'not_found', 'message': 'outside map-registry namespace'}}, 'application/json'
        sub = path[len(prefix):]

        try:
            parts = [urllib.parse.unquote(p) for p in sub.strip('/').split('/') if p]
            for seg in parts:
                if seg in ('.', '..') or '/' in seg or '\\' in seg or '\x00' in seg:
                    raise SecurityError('invalid path segment in map-registry URI', {'segment': seg})
            sub = '/' + '/'.join(parts)

            # Parse & bound request payload BEFORE heavy decoding.
            if body is None:
                payload: dict[str, Any] = {}
            elif isinstance(body, (bytes, bytearray)):
                if len(body) > self.max_http_write_bytes:
                    raise PayloadTooLargeError(
                        'request body exceeds max HTTP write size limit',
                        {'bytes': len(body), 'limit': self.max_http_write_bytes},
                    )
                try:
                    decoded_text = bytes(body).decode('utf-8')
                except UnicodeDecodeError as exc:
                    raise ValidationError('request body must be valid UTF-8 JSON') from exc
                try:
                    loaded = json.loads(decoded_text)
                except ValueError as exc:
                    raise ValidationError('invalid JSON request body') from exc
                if not isinstance(loaded, dict):
                    raise ValidationError('JSON request body must be an object')
                payload = loaded
            elif isinstance(body, str):
                raw_bytes = body.encode('utf-8')
                if len(raw_bytes) > self.max_http_write_bytes:
                    raise PayloadTooLargeError(
                        'request body exceeds max HTTP write size limit',
                        {'bytes': len(raw_bytes), 'limit': self.max_http_write_bytes},
                    )
                try:
                    loaded = json.loads(body)
                except ValueError as exc:
                    raise ValidationError('invalid JSON request body') from exc
                if not isinstance(loaded, dict):
                    raise ValidationError('JSON request body must be an object')
                payload = loaded
            elif isinstance(body, dict):
                payload = dict(body)
            else:
                raise ValidationError('unsupported request body type')

            # Extract & validate idempotency_key from header or body.
            hdr_idem: str | None = None
            if headers:
                for hk, hv in headers.items():
                    if str(hk).lower() == 'idempotency-key' and str(hv).strip():
                        hdr_idem = str(hv).strip()
                        break
            body_idem_raw = payload.get('idempotency_key')
            body_idem = str(body_idem_raw).strip() if isinstance(body_idem_raw, str) and body_idem_raw.strip() else None
            if hdr_idem and body_idem and hdr_idem != body_idem:
                raise ValidationError('Idempotency-Key header disagrees with body idempotency_key')
            idem_key = validate_idempotency_key(hdr_idem or body_idem) if (hdr_idem or body_idem) else None

            req_canonical = {k: v for k, v in payload.items() if k != 'idempotency_key'}
            request_sha256 = hashlib.sha256(
                json.dumps(req_canonical, sort_keys=True, separators=(',', ':')).encode('utf-8')
            ).hexdigest()
            scope_key = json.dumps(['v2', actor_name, 'POST', sub, idem_key], separators=(',', ':'), ensure_ascii=True) if idem_key is not None else None

            if scope_key is not None:
                prev_idem = self.one('SELECT * FROM map_write_idempotency WHERE scope_key=?', (scope_key,))
                if prev_idem is None:
                    legacy = self.one('SELECT * FROM map_write_idempotency WHERE scope_key=?', (f'POST:{sub}:{idem_key}',))
                    if legacy and legacy['actor'] == actor_name and legacy['method'] == 'POST' and legacy['route'] == sub:
                        prev_idem = legacy
                if prev_idem is not None:
                    if prev_idem['request_sha256'] != request_sha256:
                        raise ConflictError(
                            'idempotency_key already used with a different request payload',
                            {'idempotency_key': idem_key, 'route': sub},
                        )
                    replay_body = json.loads(prev_idem['response_json'])
                    replay_body['idempotent_replay'] = True
                    return int(prev_idem['status_code']), replay_body, 'application/json'

            status_code: int
            resp_body: dict[str, Any]

            if parts == ['facilities']:
                fid = payload.get('facility_id')
                name = payload.get('name')
                tz = payload.get('timezone', 'Asia/Seoul')
                desc = payload.get('description', '')
                fac = self.create_facility(
                    str(fid or ''),
                    str(name or ''),
                    timezone=str(tz),
                    description=str(desc),
                    actor=actor_name,
                )
                status_code = 201
                resp_body = {'ok': True, 'facility': fac, 'idempotent_replay': False}

            elif parts == ['buildings']:
                bid = payload.get('building_id')
                fid = payload.get('facility_id')
                name = payload.get('name')
                desc = payload.get('description', '')
                bldg = self.create_building(
                    str(bid or ''),
                    str(fid or ''),
                    str(name or ''),
                    description=str(desc),
                    actor=actor_name,
                )
                status_code = 201
                resp_body = {'ok': True, 'building': bldg, 'idempotent_replay': False}

            elif parts == ['floors']:
                flid = payload.get('floor_id')
                bid = payload.get('building_id')
                lvl = payload.get('level_name')
                elev = payload.get('elevation_m', 0.0)
                if not isinstance(elev, (int, float)) or isinstance(elev, bool):
                    raise ValidationError('elevation_m must be numeric')
                flr = self.create_floor(
                    str(flid or ''),
                    str(bid or ''),
                    str(lvl or ''),
                    elevation_m=float(elev),
                    rmf_map_name=payload.get('rmf_map_name'),
                    actor=actor_name,
                )
                status_code = 201
                resp_body = {'ok': True, 'floor': flr, 'idempotent_replay': False}

            elif parts == ['areas']:
                cov = payload.get('coverage', 'PARTIAL_FLOOR')
                desc = payload.get('description', '')
                area = self.create_area(
                    str(payload.get('area_id') or ''),
                    str(payload.get('floor_id') or ''),
                    str(payload.get('name') or ''),
                    description=str(desc),
                    coverage=str(cov),
                    actor=actor_name,
                )
                status_code = 201
                resp_body = {'ok': True, 'area': area, 'idempotent_replay': False}

            elif parts == ['maps']:
                req_status = payload.get('verification_status')
                if req_status is not None and req_status != 'UNVERIFIED':
                    raise PolicyError(
                        'API map registration always starts as UNVERIFIED; promote explicitly via /verify',
                        {'requested_verification_status': req_status},
                    )
                yaml_bytes = _decode_b64_limited(payload.get('yaml_b64'), field='yaml_b64', max_bytes=MAX_YAML_BYTES)
                pgm_bytes = _decode_b64_limited(payload.get('pgm_b64'), field='pgm_b64', max_bytes=MAX_PGM_BYTES)
                pg_bytes = (
                    _decode_b64_limited(payload['posegraph_b64'], field='posegraph_b64', max_bytes=MAX_POSEGRAPH_BYTES)
                    if payload.get('posegraph_b64') is not None
                    else None
                )
                pgd_bytes = (
                    _decode_b64_limited(payload['posegraph_data_b64'], field='posegraph_data_b64', max_bytes=MAX_POSEGRAPH_BYTES)
                    if payload.get('posegraph_data_b64') is not None
                    else None
                )
                exp_ck = payload.get('expected_checksums')
                if exp_ck is not None and not isinstance(exp_ck, dict):
                    raise ValidationError('expected_checksums must be an object')
                clearance = payload.get('free_clearance_m', 0.0)
                if not isinstance(clearance, (int, float)) or isinstance(clearance, bool):
                    raise ValidationError('free_clearance_m must be numeric')

                extras = self._prepare_web_upload(payload, yaml_bytes, pgm_bytes, pg_bytes, pgd_bytes, exp_ck, actor_name)
                yaml_bytes, exp_ck = extras['yaml_bytes'], extras['expected_checksums']

                reg_res = self.register_map_bundle(
                    floor_id=payload.get('floor_id', ''),
                    label=payload.get('label', ''),
                    yaml_bytes=yaml_bytes,
                    pgm_bytes=pgm_bytes,
                    source_mode=str(payload.get('source_mode') or 'IMPORTED_STATIC'),
                    verification_status='UNVERIFIED',
                    verification_evidence=extras['evidence'],
                    posegraph_bytes=pg_bytes,
                    posegraph_data_bytes=pgd_bytes,
                    expected_checksums=exp_ck,
                    transform_rmf_from_nav2=payload.get('transform_rmf_from_nav2'),
                    nav_graph=payload.get('nav_graph'),
                    free_clearance_m=float(clearance),
                    frame_id=payload.get('frame_id'),
                    actor=actor_name,
                    actor_role=actor_role,
                )
                status_code = 201 if reg_res['created'] else 200
                mvid_new = reg_res['map_version']['map_version_id']
                # Both calls are write-once and idempotent for identical arguments, so a retry after a failure here completes them.
                if extras['area_id'] is not None:
                    self.link_map_version_area(mvid_new, extras['area_id'], actor=actor_name)
                if extras['lineage'] is not None:
                    self.record_lineage(mvid_new, parent_map_version_id=extras['lineage']['parent_map_version_id'],
                                        mapping_mode=extras['lineage']['mapping_mode'], session_id=extras['lineage']['session_id'],
                                        actor=actor_name)
                resp_body = {'ok': True, **reg_res, 'idempotent_replay': False,
                             'area_id': extras['area_id'], 'lineage': extras['lineage'],
                             'yaml_adjustment': extras['yaml_adjustment']}

            elif len(parts) == 3 and parts[0] == 'maps' and parts[2] == 'verify':
                mvid = parts[1]
                target_status = str(payload.get('status') or 'VERIFIED')
                ev = payload.get('evidence')
                if ev is not None and not isinstance(ev, dict):
                    raise ValidationError('evidence must be an object')
                reason = payload.get('reason', '')
                if not isinstance(reason, str):
                    raise ValidationError('reason must be a string')
                clearance = payload.get('free_clearance_m', 0.0)
                if not isinstance(clearance, (int, float)) or isinstance(clearance, bool):
                    raise ValidationError('free_clearance_m must be numeric')
                mv = self.verify_map_version(
                    mvid,
                    status=target_status,
                    evidence=ev,
                    actor=actor_name,
                    actor_role=actor_role,
                    reason=reason,
                    allow_reverify=bool(payload.get('allow_reverify', False)),
                    transform_rmf_from_nav2=payload.get('transform_rmf_from_nav2'),
                    nav_graph=payload.get('nav_graph'),
                    free_clearance_m=float(clearance),
                )
                status_code = 200
                resp_body = {'ok': True, 'map_version': mv, 'idempotent_replay': False}

            elif len(parts) == 3 and parts[0] == 'maps' and parts[2] == 'revoke':
                mvid = parts[1]
                reason = payload.get('reason', '')
                ev = payload.get('evidence')
                if ev is not None and not isinstance(ev, dict):
                    raise ValidationError('evidence must be an object')
                res = self.revoke_map_verification(
                    mvid,
                    reason=reason,
                    actor=actor_name,
                    actor_role=actor_role,
                    evidence=ev,
                )
                status_code = 200
                resp_body = {'ok': True, **res, 'idempotent_replay': False}

            elif len(parts) == 3 and parts[0] == 'maps' and parts[2] == 'quarantine':
                mvid = parts[1]
                reason = payload.get('reason', '')
                detail = payload.get('detail')
                if detail is not None and not isinstance(detail, dict):
                    raise ValidationError('detail must be an object')
                res = self.quarantine_map_version(
                    mvid,
                    reason=reason,
                    detail=detail,
                    actor=actor_name,
                    actor_role=actor_role,
                )
                status_code = 200
                resp_body = {'ok': True, **res, 'idempotent_replay': False}

            elif len(parts) == 4 and parts[0] == 'maps' and parts[2] == 'quarantine' and parts[3] == 'clear':
                if actor_role != 'admin':
                    return 403, {'error': {'code': 'forbidden', 'message': 'only admin role may clear map quarantine'}}, 'application/json'
                mvid = parts[1]
                reason = payload.get('reason', '')
                res = self.clear_map_quarantine(
                    mvid,
                    reason=reason,
                    actor=actor_name,
                    actor_role=actor_role,
                )
                status_code = 200
                resp_body = {'ok': True, **res, 'idempotent_replay': False}

            else:
                return 404, {'error': {'code': 'not_found', 'message': 'unknown map-registry write endpoint'}}, 'application/json'

            if scope_key is not None:
                with self.transaction():
                    race_idem = self.one('SELECT * FROM map_write_idempotency WHERE scope_key=?', (scope_key,))
                    if race_idem is not None:
                        if race_idem['request_sha256'] != request_sha256:
                            raise ConflictError(
                                'idempotency_key already used concurrently with different request payload',
                                {'idempotency_key': idem_key, 'route': sub},
                            )
                        replay_body = json.loads(race_idem['response_json'])
                        replay_body['idempotent_replay'] = True
                        return int(race_idem['status_code']), replay_body, 'application/json'
                    self.x(
                        'INSERT INTO map_write_idempotency('
                        'scope_key, method, route, actor, request_sha256, status_code, response_json, created_at'
                        ') VALUES(?,?,?,?,?,?,?,?)',
                        (
                            scope_key,
                            'POST',
                            sub,
                            actor_name,
                            request_sha256,
                            status_code,
                            json.dumps(resp_body, sort_keys=True),
                            time.time(),
                        ),
                    )

            return status_code, resp_body, 'application/json'
        except MapRegistryError as exc:
            return exc.status, exc.to_dict(), 'application/json'


class _Tx:
    def __init__(self, reg: MapRegistry):
        self.reg = reg
        self.sp: str | None = None

    def __enter__(self) -> _Tx:
        self.reg.lock.acquire()
        try:
            if self.reg._tx_depth == 0:
                self.reg.c.execute('BEGIN IMMEDIATE')
            else:
                self.sp = f'sp_{self.reg._tx_depth}'
                self.reg.c.execute(f'SAVEPOINT {self.sp}')
            self.reg._tx_depth += 1
        except Exception:
            self.reg.lock.release()
            raise
        return self

    def __exit__(self, et: Any, ev: Any, tb: Any) -> bool:
        try:
            self.reg._tx_depth -= 1
            if self.sp is None:
                if et is None:
                    try: self.reg.c.execute('COMMIT')
                    except Exception:
                        if self.reg.c.in_transaction: self.reg.c.execute('ROLLBACK')
                        raise
                else: self.reg.c.execute('ROLLBACK')
            elif et is None:
                self.reg.c.execute(f'RELEASE SAVEPOINT {self.sp}')
            else:
                self.reg.c.execute(f'ROLLBACK TO SAVEPOINT {self.sp}')
                self.reg.c.execute(f'RELEASE SAVEPOINT {self.sp}')
        finally:
            self.reg.lock.release()
        return False
