"""Web map save: the operator asks the Agent to snapshot the live SLAM map (raster + pose graph) and register it UNVERIFIED.

Scope and rules
- Save only. It never starts mapping, never moves the robot and never touches the three real-robot Mapping blocks.
- One durable request row per save (`map_save_requests`); the operator needs the current Lease owner + epoch and an idempotency key.
- HTTP acceptance (QUEUED) is not a result. The Agent accepts, reports stages (SAVING_RASTER, SAVING_POSEGRAPH, UPLOADING) and the server
  reaches REGISTERED only after it has verified every uploaded file against the Agent's manifest (size + SHA-256), run the registry's
  upload validation (including the unknown-pixel threshold safety check) and registered the bundle.
- Terminal states: REGISTERED, FAILED (with a code and message), TIMEOUT (nothing terminal before the deadline), UNKNOWN (the Agent session changed
  or the robot went offline mid-save). TIMEOUT and UNKNOWN are sticky: a late upload is refused and the operator starts a new request.
- The raster and the pose graph are written by two separate calls. They are never assumed to be one snapshot: the registered evidence records
  each file's save window, the gap between them, whether the live map changed in between and how far the robot moved (`snapshot_consistency`).
- Existing registry entries are never modified: a save creates a new UNVERIFIED MapVersion (area link + NEW_MAP lineage), nothing else.
"""
import base64
import hashlib
import json
import re
import time
import uuid

ID_RE = re.compile(r'[A-Za-z0-9][A-Za-z0-9._-]{0,63}')
KEY_RE = re.compile(r'[A-Za-z0-9_.:-]{1,100}')
QUEUED_TTL_S = 30.0          # the Agent must accept within this time
TOTAL_TTL_S = 240.0          # everything (save + upload + registration) must finish within this time
ACTIVE = ('QUEUED', 'ACCEPTED', 'SAVING_RASTER', 'SAVING_POSEGRAPH', 'UPLOADING')
TERMINAL = ('REGISTERED', 'FAILED', 'TIMEOUT', 'UNKNOWN')
STAGE_RANK = {'QUEUED': 0, 'ACCEPTED': 1, 'SAVING_RASTER': 2, 'SAVING_POSEGRAPH': 3, 'UPLOADING': 4}
FILES = ('yaml', 'pgm', 'posegraph', 'data')
FILE_NAMES = {'yaml': 'map/map.yaml', 'pgm': 'map/map.pgm', 'posegraph': 'map/posegraph.posegraph', 'data': 'map/posegraph.data'}
THRESHOLD_POLICIES = ('reject', 'derive')
STATIONARY_MAX_M, STATIONARY_MAX_RAD = 0.02, 0.02

SCHEMA = """
CREATE TABLE IF NOT EXISTS map_save_requests(
  save_id TEXT PRIMARY KEY, robot_id TEXT NOT NULL, actor TEXT NOT NULL, epoch INTEGER NOT NULL, idem TEXT NOT NULL,
  target_json TEXT NOT NULL, status TEXT NOT NULL, detail TEXT NOT NULL DEFAULT '', error_code TEXT, agent_boot TEXT,
  created_at REAL NOT NULL, expires_at REAL NOT NULL, updated_at REAL NOT NULL, accepted_at REAL,
  manifest_json TEXT, result_json TEXT, UNIQUE(robot_id, actor, idem));
"""


class MapSaveMixin:
    def _ensure_map_save_schema(self):
        with self.db.lock:
            self.db.c.executescript(SCHEMA)

    # ------------------------------------------------------------------ views
    def _save_row_view(self, r):
        mani = json.loads(r['manifest_json']) if r['manifest_json'] else None
        try:
            consistency = self._consistency(mani) if mani and isinstance(mani.get('files'), dict) else None
        except (KeyError, TypeError, ValueError):
            consistency = None
        return {'consistency': consistency, 'save_id': r['save_id'], 'robot_id': r['robot_id'], 'status': r['status'], 'detail': r['detail'], 'error_code': r['error_code'],
                'owner': r['actor'], 'target': json.loads(r['target_json']), 'created_at': r['created_at'], 'expires_at': r['expires_at'],
                'updated_at': r['updated_at'], 'accepted_at': r['accepted_at'],
                'manifest': mani,
                'result': json.loads(r['result_json']) if r['result_json'] else None}

    def _save_refresh(self, row):
        """Persist the sticky terminal states the server can decide on its own (TIMEOUT, UNKNOWN)."""
        if row['status'] not in ACTIVE:
            return row
        now = time.time()
        robot = self.robots.get(row['robot_id']) or {}
        new = None
        if row['status'] == 'QUEUED' and now >= min(row['expires_at'], row['created_at'] + QUEUED_TTL_S):
            new = ('TIMEOUT', 'timeout', 'the Agent did not accept the save request in time')
        elif now >= row['expires_at']:
            new = ('TIMEOUT', 'timeout', 'no terminal result before the deadline; the files may exist on the robot')
        elif row['status'] != 'QUEUED' and row['agent_boot'] and (robot.get('agent') or {}).get('boot_id') != row['agent_boot']:
            new = ('UNKNOWN', 'agent_restarted', 'the Agent session changed while saving; files may remain on the robot, start a new save')
        elif row['status'] != 'QUEUED' and not robot.get('online'):
            new = ('UNKNOWN', 'robot_offline', 'the robot went offline while saving; the outcome is unknown')
        if new:
            self.db.x('UPDATE map_save_requests SET status=?, error_code=?, detail=?, updated_at=? WHERE save_id=? AND status IN (%s)'
                      % ','.join('?' * len(ACTIVE)), (new[0], new[1], new[2], now, row['save_id'], *ACTIVE))
            self.db.event('map_save_' + new[0].lower(), f"map save {row['save_id']}: {new[2]}", row['robot_id'], level='warning')
            row = self.db.one('SELECT * FROM map_save_requests WHERE save_id=?', (row['save_id'],))
        return row

    def map_save_status(self, robot_id, limit=10):
        from .service import ApiError
        with self.lock:
            if robot_id not in self.robots:
                raise ApiError(404, 'unknown_robot', 'robot not registered')
            rows = self.db.q('SELECT * FROM map_save_requests WHERE robot_id=? ORDER BY created_at DESC LIMIT ?', (robot_id, int(limit)))
            return {'robot_id': robot_id, 'saves': [self._save_row_view(self._save_refresh(r)) for r in rows],
                    'capability': self._map_save_capability(robot_id)}

    def _map_save_capability(self, robot_id):
        r = self.robots.get(robot_id) or {}
        cap = ((r.get('state') or {}).get('map_save')) if isinstance(r.get('state'), dict) else None
        if (r.get('agent') or {}).get('map_save') is not True:
            return {'available': False, 'reason': 'agent has no map-save capability (not started with map_save_enabled)'}
        if not r.get('online') or not r.get('reconciled') or r.get('reconciliation_required'):
            return {'available': False, 'reason': 'robot offline or not reconciled'}
        ts = r.get('state_ts')
        if not isinstance(ts, (int, float)) or time.time() - ts > self.cfg.offline_after_s:
            return {'available': False, 'reason': 'robot state is stale'}
        if not isinstance(cap, dict) or cap.get('available') is not True:
            return {'available': False, 'reason': (cap or {}).get('reason') or 'agent reports map save not ready'}
        return {'available': True, 'reason': ''}

    # ------------------------------------------------------------------ operator request
    def map_save_request(self, robot_id, actor, body):
        from .service import ApiError
        if not isinstance(body, dict):
            raise ApiError(400, 'bad_request', 'JSON object required')
        with self.lock:
            if robot_id not in self.robots:
                raise ApiError(404, 'unknown_robot', 'robot not registered')
            lease = self.lease_view(robot_id)
            if (not lease or lease['owner'] != actor or type(body.get('epoch')) is not int or lease['epoch'] != body['epoch']):
                raise ApiError(409, 'lease_required', 'current Lease owner and epoch required')
            key = body.get('idempotency_key')
            if not isinstance(key, str) or not KEY_RE.fullmatch(key):
                raise ApiError(400, 'bad_request', 'valid idempotency_key required')
            label, note, policy = body.get('label'), body.get('note', ''), body.get('threshold_policy', 'reject')
            if not isinstance(label, str) or not ID_RE.fullmatch(label):
                raise ApiError(400, 'bad_label', 'label must start with a letter or digit and use letters, digits, . _ - (max 64)')
            if not isinstance(note, str) or len(note) > 500:
                raise ApiError(400, 'bad_note', 'note must be a string of at most 500 characters')
            if policy not in THRESHOLD_POLICIES:
                raise ApiError(400, 'bad_policy', 'threshold_policy must be reject or derive')
            target = dict(self._save_target(body), label=label, note=note, threshold_policy=policy)    # validates floor + area against the registry
            old = self.db.one('SELECT * FROM map_save_requests WHERE robot_id=? AND actor=? AND idem=?', (robot_id, actor, key))
            if old:
                if json.loads(old['target_json']) != target or old['epoch'] != lease['epoch']:
                    raise ApiError(409, 'idempotency_conflict', 'key belongs to a different request')
                v = self._save_row_view(self._save_refresh(old))
                return dict(v, duplicate=True)
            cap = self._map_save_capability(robot_id)
            if not cap['available']:
                raise ApiError(409, 'map_save_unavailable', cap['reason'])
            if self.operation_status(robot_id)['state'] not in ('IDLE', 'STOPPED'):
                raise ApiError(409, 'mapping_active', 'a web mapping session is active; save from the mapping session instead')
            for r in self.db.q('SELECT * FROM map_save_requests WHERE robot_id=?', (robot_id,)):
                if self._save_refresh(r)['status'] in ACTIVE:
                    raise ApiError(409, 'save_in_progress', 'another map save is still running on this robot')
            now = time.time()
            sid = uuid.uuid4().hex
            self.db.x('INSERT INTO map_save_requests(save_id,robot_id,actor,epoch,idem,target_json,status,detail,created_at,expires_at,updated_at) '
                      'VALUES(?,?,?,?,?,?,?,?,?,?,?)', (sid, robot_id, actor, lease['epoch'], key, json.dumps(target, sort_keys=True), 'QUEUED',
                                                        'waiting for the Agent', now, now + TOTAL_TTL_S, now))
            self.db.event('map_save_requested', f'{actor}: save current map as {label} ({sid})', robot_id, data={'save_id': sid})
            return dict(self._save_row_view(self.db.one('SELECT * FROM map_save_requests WHERE save_id=?', (sid,))), duplicate=False)

    # ------------------------------------------------------------------ Agent side
    def map_save_agent(self, robot_id, boot, body):
        """poll | accept | stage | fail | bundle. Every action needs the live Agent session of this robot."""
        from .service import ApiError
        action = body.get('action')
        with self.lock:
            self.check_session(robot_id, boot)
            if action == 'poll':
                for r in self.db.q("SELECT * FROM map_save_requests WHERE robot_id=? AND status='QUEUED' ORDER BY created_at", (robot_id,)):
                    r = self._save_refresh(r)
                    if r['status'] == 'QUEUED':
                        return {'request': {'save_id': r['save_id'], 'target': json.loads(r['target_json']), 'expires_at': r['expires_at']}}
                return {'request': None}
            row = self.db.one('SELECT * FROM map_save_requests WHERE save_id=? AND robot_id=?', (body.get('save_id'), robot_id))
            if row is None:
                raise ApiError(404, 'unknown_save', 'no such save request for this robot')
            row = self._save_refresh(row)
            if action == 'accept':
                if row['status'] == 'QUEUED':
                    self.db.x("UPDATE map_save_requests SET status='ACCEPTED', agent_boot=?, accepted_at=?, updated_at=?, detail=? WHERE save_id=?",
                              (boot, time.time(), time.time(), 'accepted by the Agent', row['save_id']))
                    return {'accepted': True}
                if row['status'] in ACTIVE and row['agent_boot'] == boot:
                    return {'accepted': True, 'duplicate': True}
                raise ApiError(409, 'save_not_active', f"request is {row['status']}")
            if row['agent_boot'] != boot:
                raise ApiError(409, 'save_owner_mismatch', 'the request belongs to another Agent session')
            if action == 'stage':
                stage = body.get('stage')
                if stage not in STAGE_RANK or stage == 'QUEUED':
                    raise ApiError(400, 'bad_stage', 'unknown stage')
                if row['status'] in ACTIVE and STAGE_RANK[stage] >= STAGE_RANK[row['status']]:
                    self.db.x('UPDATE map_save_requests SET status=?, updated_at=?, detail=? WHERE save_id=?',
                              (stage, time.time(), str(body.get('detail') or '')[:300], row['save_id']))
                return {'status': self.db.one('SELECT status FROM map_save_requests WHERE save_id=?', (row['save_id'],))['status']}
            if action == 'fail':
                if row['status'] in ACTIVE:
                    mani = body.get('manifest') if isinstance(body.get('manifest'), dict) else None
                    self.db.x("UPDATE map_save_requests SET status='FAILED', error_code=?, detail=?, updated_at=?, manifest_json=? WHERE save_id=?",
                              (str(body.get('error_code') or 'agent_failed')[:60], str(body.get('message') or '')[:500], time.time(),
                               json.dumps(mani) if mani else None, row['save_id']))
                    self.db.event('map_save_failed', f"map save {row['save_id']} failed: {body.get('error_code')}", robot_id, level='warning')
                return {'status': self.db.one('SELECT status FROM map_save_requests WHERE save_id=?', (row['save_id'],))['status']}
            if action == 'bundle':
                return self._map_save_bundle(robot_id, boot, row, body)
            raise ApiError(400, 'bad_action', 'unknown action')

    def _fail_save(self, row, code, message, manifest=None):
        self.db.x("UPDATE map_save_requests SET status='FAILED', error_code=?, detail=?, updated_at=?, manifest_json=COALESCE(?, manifest_json) WHERE save_id=?",
                  (code, message[:500], time.time(), json.dumps(manifest) if manifest else None, row['save_id']))
        self.db.event('map_save_failed', f"map save {row['save_id']} failed: {code}", row['robot_id'], level='warning')

    def _map_save_bundle(self, robot_id, boot, row, body):
        from .map_registry import MapRegistryError
        from .service import ApiError
        if row['status'] == 'REGISTERED':                          # idempotent replay of a finished save
            res = json.loads(row['result_json'])
            if body.get('manifest', {}).get('files') != (json.loads(row['manifest_json']) or {}).get('files'):
                raise ApiError(409, 'save_conflict', 'this save was already registered with different files')
            return dict(res, duplicate=True)
        if row['status'] not in ACTIVE:
            raise ApiError(409, 'save_not_active', f"request is {row['status']}: a late upload is refused, start a new save")
        reg = self.map_registry
        if reg is None:
            raise ApiError(404, 'registry_unavailable', 'map registry unavailable')
        mani = body.get('manifest')
        if not isinstance(mani, dict) or not isinstance(mani.get('files'), dict) or set(mani['files']) != set(FILES):
            raise ApiError(400, 'bad_manifest', 'manifest.files must describe yaml, pgm, posegraph and data')
        blobs = {}
        for f in FILES:
            try:
                blobs[f] = base64.b64decode(body[f + '_b64'], validate=True)
            except (KeyError, TypeError, ValueError):
                raise ApiError(400, 'invalid_map', f'{f}: invalid or missing base64')
            m = mani['files'][f]
            if (not isinstance(m, dict) or len(blobs[f]) != m.get('size') or hashlib.sha256(blobs[f]).hexdigest() != m.get('sha256') or len(blobs[f]) == 0):
                # Retryable by the Agent (idempotent), so the request stays active.
                raise ApiError(422, 'transfer_corrupt', f'{f}: received size/SHA-256 differs from the Agent manifest')
        target = json.loads(row['target_json'])
        payload = {'floor_id': target['floor_id'], 'area_id': target.get('area_id'),
                   'lineage': {'mapping_mode': 'NEW_MAP', 'session_id': 'save_' + row['save_id'][:24]},
                   'threshold_policy': target['threshold_policy'], 'note': target.get('note', '')}
        try:
            prep = reg._prepare_web_upload(payload, blobs['yaml'], blobs['pgm'], blobs['posegraph'], blobs['data'], None, row['actor'])
            evidence = dict(prep['evidence'])
            evidence.update({
                'method': 'web_map_save', 'save_id': row['save_id'], 'robot_id': robot_id, 'operator': row['actor'], 'agent_boot_id': boot,
                'saved_files': {FILE_NAMES[f]: dict(mani['files'][f]) for f in FILES},
                'snapshot_consistency': self._consistency(mani),
                'saver': dict(mani.get('saver') or {}),
                'agent_manifest_verified': True,
                'field_verification': 'UNVERIFIED (saved from the live SLAM map; no localization or navigation trial on this map)'})
            artifacts = dict(evidence.get('artifacts_sha256') or {})
            artifacts.update({FILE_NAMES[f]: mani['files'][f]['sha256'] for f in FILES})
            evidence['artifacts_sha256'] = artifacts
            result = reg.register_map_bundle(
                floor_id=target['floor_id'], label=target['label'], yaml_bytes=prep['yaml_bytes'], pgm_bytes=blobs['pgm'],
                posegraph_bytes=blobs['posegraph'], posegraph_data_bytes=blobs['data'], source_mode='SLAM_SAVED_BUNDLE',
                verification_status='UNVERIFIED', verification_evidence=evidence, expected_checksums=prep['expected_checksums'],
                actor=robot_id, actor_role='agent')
            mvid = result['map_version']['map_version_id']
            if prep['area_id'] is not None:
                reg.link_map_version_area(mvid, prep['area_id'], actor=robot_id)
            reg.record_lineage(mvid, parent_map_version_id=None, mapping_mode='NEW_MAP', session_id=prep['lineage']['session_id'], actor=robot_id)
            integrity = reg.verify_bundle_integrity(mvid)
            if not integrity.get('ok'):
                raise ApiError(500, 'registry_integrity_failed', 'registered map bundle integrity check failed')
        except MapRegistryError as exc:
            code = getattr(exc, 'code', 'map_registry_rejected')
            detail = getattr(exc, 'detail', None) or {}
            self._fail_save(row, code if code != 'policy_violation' else str(detail.get('code') or code), str(exc), mani)
            raise ApiError(422 if exc.status in (409, 422) else exc.status, 'map_registry_rejected', f'{code}: {exc}'[:300])
        res = {'map_version_id': mvid, 'created': result['created'], 'integrity': integrity, 'area_id': prep['area_id'],
               'yaml_adjustment': prep['yaml_adjustment'], 'label': target['label']}
        self.db.x("UPDATE map_save_requests SET status='REGISTERED', error_code=NULL, detail=?, updated_at=?, manifest_json=?, result_json=? WHERE save_id=?",
                  ('registered as UNVERIFIED', time.time(), json.dumps(mani), json.dumps(res), row['save_id']))
        self.db.event('map_save_registered', f"map save {row['save_id']} registered {mvid}", robot_id, data={'map_version_id': mvid})
        return res

    @staticmethod
    def _consistency(mani):
        """What is known about how well the raster and the pose graph belong together. Never claims more than the evidence."""
        f = mani['files']
        ev = mani.get('evidence') or {}
        try:
            gap = round(float(f['posegraph']['saved_start']) - float(f['pgm']['saved_end']), 3)
        except (KeyError, TypeError, ValueError):
            gap = None
        before, after = ev.get('map_before') or {}, ev.get('map_after') or {}
        map_unchanged = (before.get('version') is not None and before.get('version') == after.get('version')
                         and before.get('known_cells') == after.get('known_cells'))
        moved, rot = ev.get('moved_m'), ev.get('rotated_rad')
        stationary = (isinstance(moved, (int, float)) and isinstance(rot, (int, float))
                      and moved <= STATIONARY_MAX_M and rot <= STATIONARY_MAX_RAD)
        return {'raster_window': [f['pgm'].get('saved_start'), f['pgm'].get('saved_end')],
                'posegraph_window': [f['posegraph'].get('saved_start'), f['posegraph'].get('saved_end')],
                'gap_between_raster_end_and_posegraph_start_s': gap,
                'live_map_before': before, 'live_map_after': after, 'live_map_unchanged_during_save': bool(map_unchanged),
                'robot_moved_m': moved, 'robot_rotated_rad': rot, 'robot_stationary_during_save': bool(stationary),
                'same_snapshot': 'LIKELY_NOT_PROVEN' if (map_unchanged and stationary) else 'NOT_ESTABLISHED',
                'basis': 'raster (map_saver_cli) and pose graph (slam_toolbox serialize_map) are two separate sequential calls; they are not '
                         'one atomic snapshot. The claim above comes only from the live-map version/cell count and the odometry displacement '
                         'observed around the calls.'}
