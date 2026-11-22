"""Phase 5 (2026-11-21): visitor guide service - destinations bound to map versions, visitor requests -> Fleet missions.

Roles
  * Visitors never get a Bearer principal. A visitor link token (admin-issued, expiring, revocable, stored as SHA-256) is sent as
    `Authorization: Visitor <token>` and only reaches the /api/v1/visitor/* routes below. Nothing else of the API accepts it:
    no robots/state/missions/events, no Manual, Gate Reset, Mapping, Navigation LOAD, raw velocity or admin routes.
  * A visitor browser identifies its own requests with a random `X-Visitor-Session` (not authentication: it only scopes
    "my request" / cancel; one active request per session, a creation rate limit per link).
  * Operators manage destinations and open / close the service for a robot. Opening requires holding the robot's lease; the
    visitor missions are created under that operator's name and lease epoch (accountable human), with note `visitor guide <id>`.
Destinations
  * bound to (map_version_id, content_sha256) and a map-frame pose; validated with the SAME server raster check as missions
    (NavigationMixin.check_point, GOAL_CLEARANCE_M) on that exact version.
  * offered for guidance only if: the version is VERIFIED, not quarantined, its content hash unchanged; the robot runs the
    service, is READY (with --require-supervisor-ready), in NAVIGATION, localized CONFIRMED on that SAME map version, and the
    point passes the server goal check on the loaded map. UNVERIFIED maps (C103 included) are never used for guidance.
Request states (visitor view): RECEIVED -> QUEUED (robot busy with another guide) -> ACCEPTED (mission created) -> IN_PROGRESS
  (EXECUTING) -> COMPLETED | CANCELED | FAILED ; UNAVAILABLE when refused at creation. Mission execution, the Agent, the Motion
  Watchdog and the Safety Gate are the existing chain; this module only creates / cancels Fleet missions.
"""
import hashlib, hmac, json, math, os, re, secrets, time, uuid

SESSION_RE = re.compile(r'[A-Za-z0-9_-]{16,64}')
LINK_MAX_TTL_S = 30 * 24 * 3600
CREATE_LIMIT_PER_MIN = 10
ACTIVE = ('RECEIVED', 'QUEUED', 'ACCEPTED', 'IN_PROGRESS', 'CANCELING')
RESERVING = ('ACCEPTED', 'IN_PROGRESS', 'CANCELING')     # a robot with one of these guide requests is reserved (one execution owner)
TRANSIENT_CREATE_ERRORS = ('safety_not_ready', 'manual_active', 'robot_not_ready')
RECEIVED_STALE_S = 5.0          # a RECEIVED request older than this (server crashed before dispatch) is queued by guide_tick
MISSION_INFLIGHT = ('QUEUED', 'DISPATCHED', 'ACCEPTED', 'EXECUTING', 'CANCELING')
INTEGRITY_TTL_S = 30.0          # an unchanged bundle (same stat signature) is re-hashed at least this often
RACY_S = 2.0
BUNDLE_FILES = ('map.yaml', 'map.pgm', 'map.posegraph', 'map.posegraph.data')
REASON_TEXT = {
    'SERVICE_CLOSED': '안내 서비스가 운영 중이 아닙니다', 'OPERATOR_LEASE_CHANGED': '운영자 확인이 필요합니다(서비스 일시 중지)',
    'ROBOT_OFFLINE': '로봇과 연결되지 않았습니다', 'ROBOT_NOT_READY': '로봇이 준비 중입니다', 'ROBOT_NOT_IN_NAVIGATION': '로봇이 안내 모드가 아닙니다',
    'ROBOT_NOT_LOCALIZED': '로봇이 위치를 확인하는 중입니다', 'ROBOT_ON_OTHER_MAP': '이 목적지는 현재 로봇 지도에서 안내할 수 없습니다',
    'MAP_NOT_VERIFIED': '검증되지 않은 지도의 목적지입니다', 'MAP_QUARANTINED': '지도 사용이 중지되었습니다', 'MAP_CHANGED': '지도가 변경되어 목적지 재확인이 필요합니다',
    'MAP_MISSING': '지도를 찾을 수 없습니다', 'GOAL_NOT_REACHABLE': '목적지 위치를 현재 지도에서 확인할 수 없습니다', 'DESTINATION_DISABLED': '안내가 중지된 목적지입니다',
    'ROBOT_BUSY': '다른 안내가 진행 중입니다(대기열)', 'MAP_STATUS_UNKNOWN': '지도 상태를 확인할 수 없습니다', 'MAP_INTEGRITY_FAILED': '지도 파일 확인에 실패했습니다', 'MISSION_FAILED': '안내 중 문제가 발생했습니다', 'MISSION_CANCELED': '안내가 취소되었습니다',
}
STATUS_TEXT = {'RECEIVED': '접수됨', 'QUEUED': '대기 중', 'ACCEPTED': '로봇 배정됨', 'IN_PROGRESS': '안내 중', 'CANCELING': '취소 중',
               'COMPLETED': '도착 완료', 'CANCELED': '취소됨', 'FAILED': '안내 실패', 'UNAVAILABLE': '이용 불가'}
MISSION_TO_REQUEST = {'QUEUED': 'ACCEPTED', 'DISPATCHED': 'ACCEPTED', 'ACCEPTED': 'ACCEPTED', 'EXECUTING': 'IN_PROGRESS', 'CANCELING': 'CANCELING',
                      'SUCCEEDED': 'COMPLETED', 'CANCELED': 'CANCELED', 'FAILED': 'FAILED', 'REJECTED': 'FAILED', 'EXPIRED': 'FAILED',
                      'RECONCILIATION_REQUIRED': 'FAILED'}

SCHEMA = """
CREATE TABLE IF NOT EXISTS guide_destinations (destination_id TEXT PRIMARY KEY, floor_id TEXT NOT NULL, area_id TEXT, name TEXT NOT NULL,
  aliases_json TEXT NOT NULL, description TEXT NOT NULL, map_version_id TEXT NOT NULL, map_content_sha256 TEXT NOT NULL, x REAL NOT NULL,
  y REAL NOT NULL, yaw REAL NOT NULL, enabled INTEGER NOT NULL, created_by TEXT NOT NULL, created_at REAL NOT NULL, updated_at REAL NOT NULL);
CREATE TABLE IF NOT EXISTS guide_requests (request_id TEXT PRIMARY KEY, destination_id TEXT NOT NULL, link_id TEXT NOT NULL, session TEXT NOT NULL,
  robot_id TEXT, status TEXT NOT NULL, reason TEXT, mission_id TEXT, created_at REAL NOT NULL, updated_at REAL NOT NULL, history_json TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS visitor_links (link_id TEXT PRIMARY KEY, token_sha256 TEXT NOT NULL UNIQUE, label TEXT NOT NULL, expires_at REAL NOT NULL,
  created_by TEXT NOT NULL, created_at REAL NOT NULL, revoked_at REAL);
CREATE TABLE IF NOT EXISTS guide_service (robot_id TEXT PRIMARY KEY, enabled INTEGER NOT NULL, operator TEXT, lease_epoch INTEGER, updated_by TEXT, updated_at REAL NOT NULL);
"""


def _h(t): return hashlib.sha256(t.encode()).hexdigest()


class Visitor:
    def __init__(self, link_id, session): self.link_id, self.session = link_id, session


class GuideMixin:
    def _ensure_guide_schema(self):
        if getattr(self, '_guide_schema', False): return
        with self.db.transaction():
            for stmt in SCHEMA.strip().split(';'):
                if stmt.strip(): self.db.x(stmt)
        self._guide_schema = True

    # ------------------------------------------------------------------ visitor links (admin)
    def create_visitor_link(self, principal, body):
        from .service import ApiError
        self._ensure_guide_schema()
        if principal.role != 'admin': raise ApiError(403, 'forbidden', 'only an admin may issue visitor links')
        ttl = body.get('ttl_s', 7 * 24 * 3600) if isinstance(body, dict) else None
        label = str((body or {}).get('label', 'visitor'))[:60]
        if not isinstance(ttl, (int, float)) or not 60 <= ttl <= LINK_MAX_TTL_S: raise ApiError(400, 'bad_ttl', f'ttl_s must be 60..{LINK_MAX_TTL_S}')
        tok = secrets.token_urlsafe(24); lid = 'vl_' + uuid.uuid4().hex[:10]; now = time.time()
        self.db.x('INSERT INTO visitor_links(link_id,token_sha256,label,expires_at,created_by,created_at) VALUES(?,?,?,?,?,?)', (lid, _h(tok), label, now + ttl, principal.name, now))
        self.db.event('visitor_link_created', f'{principal.name} issued visitor link {lid} ({label}) valid {ttl / 3600:.0f} h')
        return {'link_id': lid, 'token': tok, 'path': f'/visitor.html#t={tok}', 'expires_at': now + ttl, 'label': label}

    def visitor_links(self):
        self._ensure_guide_schema()
        return [dict(r, active=r['revoked_at'] is None and r['expires_at'] > time.time()) for r in
                self.db.q('SELECT link_id,label,expires_at,created_by,created_at,revoked_at FROM visitor_links ORDER BY created_at DESC LIMIT 50')]

    def revoke_visitor_link(self, principal, link_id):
        from .service import ApiError
        self._ensure_guide_schema()
        if principal.role != 'admin': raise ApiError(403, 'forbidden', 'only an admin may revoke visitor links')
        if not self.db.one('SELECT 1 FROM visitor_links WHERE link_id=?', (link_id,)): raise ApiError(404, 'unknown_link', 'no such link')
        self.db.x('UPDATE visitor_links SET revoked_at=? WHERE link_id=? AND revoked_at IS NULL', (time.time(), link_id))
        self.db.event('visitor_link_revoked', f'{principal.name} revoked visitor link {link_id}')
        return {'link_id': link_id, 'revoked': True}

    def visitor_auth(self, header, session):
        """Visitor principal or ApiError(401). Separate scheme: a visitor token is never a Bearer token and vice versa."""
        from .service import ApiError
        self._ensure_guide_schema()
        if not isinstance(header, str) or not header.startswith('Visitor '): raise ApiError(401, 'visitor_link_required', 'open the visitor link again (QR code)')
        h = _h(header[8:].strip()); row = None
        for r in self.db.q('SELECT * FROM visitor_links WHERE revoked_at IS NULL AND expires_at > ?', (time.time(),)):
            if hmac.compare_digest(r['token_sha256'], h): row = r
        if row is None: raise ApiError(401, 'visitor_link_invalid', 'this visitor link has expired or was revoked')
        if not isinstance(session, str) or not SESSION_RE.fullmatch(session): raise ApiError(400, 'bad_session', 'visitor session id required')
        return Visitor(row['link_id'], session)

    # ------------------------------------------------------------------ destinations (operator)
    def _map_row(self, mvid):
        reg = self.map_registry
        if reg is None: return None
        return reg.one('SELECT map_version_id, floor_id, verification_status, content_sha256, pgm_sha256 FROM map_versions WHERE map_version_id=?', (mvid,))

    def _map_quarantined(self, mvid):
        """True / False, or None when the lookup itself failed (UNKNOWN: never treated as 'not quarantined', Codex 2026-11-21)."""
        reg = self.map_registry
        try: q = reg.one('SELECT status FROM map_quarantines WHERE map_version_id=?', (mvid,))
        except Exception as e:
            self._guide_warn('map_status_unknown', f'quarantine lookup failed for {mvid}: {type(e).__name__}')
            return None
        return bool(q) and str(q.get('status', '')).upper() in ('ACTIVE', 'QUARANTINED')

    def _guide_warn(self, key, msg):
        """Rate-limited warning event (availability is polled by every visitor browser)."""
        last = getattr(self, '_guide_warned', None)
        if last is None: last = self._guide_warned = {}
        now = time.monotonic()
        if now - last.get(key + msg, -1e9) < 60: return
        last[key + msg] = now
        try: self.db.event('guide_map_check', msg, level='warning')
        except Exception: pass

    def _bundle_signature(self, mvid):
        reg = self.map_registry
        row = reg.one('SELECT storage_key FROM map_versions WHERE map_version_id=?', (mvid,))
        bdir = os.path.join(reg.bundles_dir, str(row['storage_key']))
        sig = []
        for n in BUNDLE_FILES:
            try: st = os.stat(os.path.join(bdir, n))
            except FileNotFoundError: sig.append((n, None)); continue
            sig.append((n, st.st_ino, st.st_size, st.st_mtime_ns, st.st_ctime_ns))
        return tuple(sig)

    def _drop_raster(self, mvid):
        rc = getattr(self, '_nav_raster_cache', None)
        if rc and rc[0][0] == mvid: self._nav_raster_cache = None

    def _map_integrity_ok(self, mvid):
        """The registered bundle on disk still matches its registered SHA-256s (MapRegistry.verify_bundle_integrity). Cached per
        bundle stat signature (inode, size, mtime, ctime) and re-hashed at least every INTEGRITY_TTL_S; any change also drops the
        server goal-check raster cache for that version. A failure or an exception is NOT ok."""
        cache = getattr(self, '_map_integrity_cache', None)
        if cache is None: cache = self._map_integrity_cache = {}
        try:
            sig = self._bundle_signature(mvid)
            hit = cache.get(mvid); now = time.monotonic()
            if hit and hit[0] == sig and now - hit[1] < INTEGRITY_TTL_S: return hit[2]
            seen = getattr(self, '_map_sig_seen', None)
            if seen is None: seen = self._map_sig_seen = {}
            if seen.get(mvid, sig) != sig: self._drop_raster(mvid)        # bundle files changed since last look
            seen[mvid] = sig
            ok = self.map_registry.verify_bundle_integrity(mvid).get('ok') is True
            if not ok: self._drop_raster(mvid)
        except Exception as e:
            self._guide_warn('map_integrity', f'integrity check failed for {mvid}: {type(e).__name__}')
            cache.pop(mvid, None); self._drop_raster(mvid); return False
        # Cache only a PASS of files not modified in the last RACY_S: file timestamps are coarse (ext4 ~ jiffies), so two writes inside
        # one tick can share a stat signature ("racy git" rule). A failure is never cached (re-hashed until fixed).
        newest = max((x[3] for x in sig if x[1] is not None), default=0) / 1e9
        if ok and time.time() - newest > RACY_S: cache[mvid] = (sig, now, ok)
        else: cache.pop(mvid, None)
        if not ok: self._guide_warn('map_integrity', f'map bundle {mvid} does not match its registered SHA-256 (guidance refused)')
        return ok

    def upsert_destination(self, principal, body):
        from .service import ApiError
        self._ensure_guide_schema()
        if not principal.can_control(): raise ApiError(403, 'forbidden', 'role may not manage destinations')
        if not isinstance(body, dict): raise ApiError(400, 'bad_destination', 'JSON object required')
        name = str(body.get('name', '')).strip()[:80]
        mvid = body.get('map_version_id')
        try: x, y, yaw = float(body['x']), float(body['y']), float(body.get('yaw', 0.0))
        except (KeyError, TypeError, ValueError): raise ApiError(400, 'bad_destination', 'numeric x, y (and yaw) required')
        if not name or not all(math.isfinite(v) for v in (x, y, yaw)): raise ApiError(400, 'bad_destination', 'name and finite pose required')
        mv = self._map_row(mvid) if isinstance(mvid, str) else None
        if not mv: raise ApiError(404, 'unknown_map_version', 'map_version_id must be a registered map version')
        ok, why, _ = self.check_point({'map_version_id': mvid, 'pgm_sha256': mv['pgm_sha256']}, x, y)
        if not ok: raise ApiError(409, 'destination_invalid', f'pose rejected on {mvid}: {why} (server goal check)')
        area = body.get('area_id')
        if area is not None and not self.map_registry.one('SELECT 1 FROM map_areas WHERE area_id=? AND floor_id=?', (area, mv['floor_id'])):
            raise ApiError(400, 'bad_area', 'area_id must belong to the map version floor')
        did = body.get('destination_id') or 'd_' + uuid.uuid4().hex[:10]
        if not re.fullmatch(r'[A-Za-z0-9_-]{3,40}', str(did)): raise ApiError(400, 'bad_destination_id', 'invalid destination_id')
        aliases = [str(a)[:40] for a in (body.get('aliases') or [])][:10]
        now = time.time(); prev = self.db.one('SELECT * FROM guide_destinations WHERE destination_id=?', (did,))
        self.db.x('INSERT OR REPLACE INTO guide_destinations VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)',
                  (did, mv['floor_id'], area, name, json.dumps(aliases, ensure_ascii=False), str(body.get('description', ''))[:200], mvid, mv['content_sha256'],
                   x, y, yaw, 0 if body.get('enabled') is False else 1, principal.name, prev['created_at'] if prev else now, now))
        self.db.event('guide_destination', f"{principal.name} {'updated' if prev else 'added'} destination {did} '{name}' on {mvid} ({mv['verification_status']})")
        return self.destination_view(self.db.one('SELECT * FROM guide_destinations WHERE destination_id=?', (did,)), internal=True)

    # ------------------------------------------------------------------ service open / close (operator with the lease)
    def set_guide_service(self, principal, body):
        from .service import ApiError
        self._ensure_guide_schema()
        if not principal.can_control(): raise ApiError(403, 'forbidden', 'role may not open or close the guide service')
        if not isinstance(body, dict): raise ApiError(400, 'bad_guide_service', 'JSON object required')
        rid = body.get('robot_id'); enabled = body.get('enabled')
        if type(enabled) is not bool: raise ApiError(400, 'bad_guide_service', 'enabled must be true or false (JSON boolean)')
        if not isinstance(rid, str) or rid not in self.robots: raise ApiError(404, 'unknown_robot', 'robot not registered')
        lease = self.lease_view(rid)
        if enabled is True and (not lease or lease['owner'] != principal.name):
            raise ApiError(409, 'lease_required', 'hold the robot lease to open the guide service (visitor missions run under it)')
        if enabled is False and principal.role != 'admin' and (not lease or lease['owner'] != principal.name):
            raise ApiError(409, 'lease_required', 'lease holder or admin required')
        now = time.time()
        self.db.x('INSERT OR REPLACE INTO guide_service(robot_id,enabled,operator,lease_epoch,updated_by,updated_at) VALUES(?,?,?,?,?,?)',
                  (rid, 1 if enabled is True else 0, principal.name if enabled is True else None, lease['epoch'] if enabled is True else None, principal.name, now))
        self.db.event('guide_service', f"{principal.name} {'opened' if enabled is True else 'closed'} the visitor guide service on {rid}", rid)
        if enabled is not True:
            for r in self.db.q("SELECT request_id FROM guide_requests WHERE robot_id=? AND status='QUEUED'", (rid,)):
                self._req_update(r['request_id'], 'FAILED', 'SERVICE_CLOSED')
        return self.guide_service_view(rid)

    def guide_service_view(self, rid):
        self._ensure_guide_schema()
        row = self.db.one('SELECT * FROM guide_service WHERE robot_id=?', (rid,)) or {'robot_id': rid, 'enabled': 0}
        return dict(row, enabled=bool(row['enabled']), effective=self._service_reason(rid) is None, reason=self._service_reason(rid))

    def _service_reason(self, rid):
        row = self.db.one('SELECT * FROM guide_service WHERE robot_id=?', (rid,))
        if not row or not row['enabled']: return 'SERVICE_CLOSED'
        lease = self.lease_view(rid)
        if not lease or lease['owner'] != row['operator'] or lease['epoch'] != row['lease_epoch']: return 'OPERATOR_LEASE_CHANGED'
        return None

    # ------------------------------------------------------------------ availability (single source for list, request and dispatch)
    def _map_block(self, d):
        """(reason, None) or (None, map row): destination enabled, map version registered, same content, VERIFIED, quarantine status
        KNOWN and clear, bundle on disk matching its SHA-256s. Used for availability AND again at dispatch."""
        if not d['enabled']: return 'DESTINATION_DISABLED', None
        try: mv = self._map_row(d['map_version_id'])
        except Exception: return 'MAP_STATUS_UNKNOWN', None
        if not mv: return 'MAP_MISSING', None
        if mv['content_sha256'] != d['map_content_sha256']: return 'MAP_CHANGED', None
        if mv['verification_status'] != 'VERIFIED': return 'MAP_NOT_VERIFIED', None
        q = self._map_quarantined(d['map_version_id'])
        if q is None: return 'MAP_STATUS_UNKNOWN', None
        if q: return 'MAP_QUARANTINED', None
        if not self._map_integrity_ok(d['map_version_id']): return 'MAP_INTEGRITY_FAILED', None
        return None, mv

    def destination_block(self, d, robot_id=None):
        """None if this destination can be guided NOW by robot_id (or any serving robot); otherwise (reason, robot_id)."""
        why, mv = self._map_block(d)
        if why: return why, None
        robots = [robot_id] if robot_id else [r['robot_id'] for r in self.db.q('SELECT robot_id FROM guide_service WHERE enabled=1')]
        best = ('SERVICE_CLOSED', None)
        for rid in robots:
            why = self._robot_block(rid, d, mv)
            if why is None: return None
            best = (why, rid)
        return best

    def _robot_block(self, rid, d, mv=None):
        r = self.robots.get(rid)
        why = self._service_reason(rid)
        if why: return why
        if not r or not r.get('online'): return 'ROBOT_OFFLINE'
        if self._supervisor_required(r) and self.readiness(rid)['level'] != 'READY': return 'ROBOT_NOT_READY'
        if (self.mode_view(rid, advance=False) or {}).get('mode') != 'NAVIGATION': return 'ROBOT_NOT_IN_NAVIGATION'
        nav = self._nav_load(rid)
        if not nav or nav.get('map_version_id') != d['map_version_id']: return 'ROBOT_ON_OTHER_MAP'
        mv = mv or self._map_row(d['map_version_id'])
        if not mv or nav.get('pgm_sha256') != mv['pgm_sha256']: return 'ROBOT_ON_OTHER_MAP'      # the Agent loaded other bytes
        if not self._nav_localized(rid, nav): return 'ROBOT_NOT_LOCALIZED'
        ok, _, _ = self.check_point(nav, d['x'], d['y'])
        if not ok: return 'GOAL_NOT_REACHABLE'
        return None

    def destination_view(self, d, internal=False):
        blk = self.destination_block(d)
        v = {'destination_id': d['destination_id'], 'name': d['name'], 'description': d['description'], 'floor_id': d['floor_id'],
             'area_id': d['area_id'], 'aliases': json.loads(d['aliases_json']), 'available': blk is None,
             'reason': None if blk is None else blk[0], 'reason_text': None if blk is None else REASON_TEXT.get(blk[0], blk[0]),
             'queue_length': self.db.one("SELECT COUNT(*) n FROM guide_requests WHERE status IN ('QUEUED','ACCEPTED','IN_PROGRESS')")['n']}
        if internal:
            v.update(map_version_id=d['map_version_id'], map_content_sha256=d['map_content_sha256'], x=d['x'], y=d['y'], yaw=d['yaw'], enabled=bool(d['enabled']))
        return v

    def destinations(self, floor_id=None, area_id=None, q=None, internal=False):
        self._ensure_guide_schema()
        rows = self.db.q('SELECT * FROM guide_destinations ORDER BY name')
        out = []
        for d in rows:
            if floor_id and d['floor_id'] != floor_id: continue
            if area_id and d['area_id'] != area_id: continue
            if q and q.lower() not in (d['name'] + ' ' + ' '.join(json.loads(d['aliases_json'])) + ' ' + d['description']).lower(): continue
            if not internal and not d['enabled']: continue
            out.append(self.destination_view(d, internal))
        return out

    def visitor_catalog(self):
        """Facility / building / floor / area tree restricted to places that have enabled destinations."""
        self._ensure_guide_schema()
        reg = self.map_registry; dests = self.db.q('SELECT floor_id, area_id FROM guide_destinations WHERE enabled=1')
        floors = sorted({d['floor_id'] for d in dests})
        tree = []
        for fid in floors:
            f = reg.one('SELECT f.floor_id, f.level_name, b.building_id, b.name AS building, c.facility_id, c.name AS facility FROM floors f '
                        'JOIN buildings b ON b.building_id=f.building_id JOIN facilities c ON c.facility_id=b.facility_id WHERE f.floor_id=?', (fid,)) if reg else None
            if not f: continue
            areas = [a for a in reg.q('SELECT area_id, name FROM map_areas WHERE floor_id=? ORDER BY name', (fid,)) if any(d['area_id'] == a['area_id'] for d in dests)]
            tree.append(dict(f, areas=areas))
        open_robots = [r['robot_id'] for r in self.db.q('SELECT robot_id FROM guide_service WHERE enabled=1') if self._service_reason(r['robot_id']) is None]
        return {'floors': tree, 'service_open': bool(open_robots)}

    # ------------------------------------------------------------------ visitor requests
    def _req_update(self, rid_, status, reason=None, mission_id=None, robot_id=None):
        row = self.db.one('SELECT * FROM guide_requests WHERE request_id=?', (rid_,))
        if not row or (row['status'] == status and row['reason'] == reason): return
        hist = json.loads(row['history_json']); now = time.time(); hist.append([round(now, 2), status, reason])
        self.db.x('UPDATE guide_requests SET status=?, reason=?, mission_id=COALESCE(?, mission_id), robot_id=COALESCE(?, robot_id), updated_at=?, history_json=? WHERE request_id=?',
                  (status, reason, mission_id, robot_id, now, json.dumps(hist[-30:]), rid_))
        self.db.event('guide_request', f'visitor request {rid_}: {status}' + (f' ({reason})' if reason else ''), robot_id or row['robot_id'], mission_id or row['mission_id'],
                      level='warning' if status in ('FAILED', 'UNAVAILABLE') else 'info')

    def create_guide_request(self, visitor, body):
        from .service import ApiError
        self._ensure_guide_schema()
        did = (body or {}).get('destination_id') if isinstance(body, dict) else None
        d = self.db.one('SELECT * FROM guide_destinations WHERE destination_id=? AND enabled=1', (did,)) if isinstance(did, str) else None
        if not d: raise ApiError(404, 'unknown_destination', 'no such destination')
        with self.lock:
            if self.db.one('SELECT 1 FROM guide_requests WHERE link_id=? AND session=? AND status IN (%s)' % ','.join('?' * len(ACTIVE)),
                           (visitor.link_id, visitor.session, *ACTIVE)):
                raise ApiError(409, 'request_active', 'you already have an active guide request')
            n = self.db.one('SELECT COUNT(*) n FROM guide_requests WHERE link_id=? AND created_at > ?', (visitor.link_id, time.time() - 60))['n']
            if n >= CREATE_LIMIT_PER_MIN: raise ApiError(429, 'rate_limited', 'too many requests; try again in a minute')
            now = time.time(); rq = 'gr_' + uuid.uuid4().hex[:12]
            self.db.x('INSERT INTO guide_requests(request_id,destination_id,link_id,session,status,created_at,updated_at,history_json) VALUES(?,?,?,?,?,?,?,?)',
                      (rq, did, visitor.link_id, visitor.session, 'RECEIVED', now, now, json.dumps([[round(now, 2), 'RECEIVED', None]])))
        blk = self.destination_block(d)
        if blk is not None:                            # refused now with the reason (busy is not a block: it queues in _dispatch_one)
            with self.lock:
                if self.db.one('SELECT status FROM guide_requests WHERE request_id=?', (rq,))['status'] == 'RECEIVED':
                    self._req_update(rq, 'UNAVAILABLE', blk[0], robot_id=blk[1])
        else:
            self._dispatch_one(rq)                     # the same FIFO / reservation path as housekeeping (never overtakes the queue)
        return self.guide_request_view(rq, visitor)

    def _robot_busy(self, rid):
        return bool(self.db.one("SELECT 1 FROM missions WHERE robot_id=? AND status IN ('QUEUED','DISPATCHED','ACCEPTED','EXECUTING','CANCELING')", (rid,)))

    def _robot_reserved(self, rid, rq):
        """Authoritative check under self.lock: an in-flight Fleet mission (any creator) or another guide request owns the robot."""
        if self.db.one('SELECT 1 FROM missions WHERE robot_id=? AND status IN (%s)' % ','.join('?' * len(MISSION_INFLIGHT)), (rid, *MISSION_INFLIGHT)):
            return True
        return bool(self.db.one('SELECT 1 FROM guide_requests WHERE robot_id=? AND request_id<>? AND status IN (%s)' % ','.join('?' * len(RESERVING)),
                                (rid, rq, *RESERVING)))

    def _older_waiting(self, row):
        return bool(self.db.one("SELECT 1 FROM guide_requests WHERE status='QUEUED' AND request_id<>? AND (created_at < ? OR (created_at = ? AND request_id < ?))",
                                (row['request_id'], row['created_at'], row['created_at'], row['request_id'])))

    def _dispatch_one(self, rq):
        """Pick a serving robot for this request: busy / older request waiting -> QUEUED; free -> one mission under the operator's lease.

        Atomicity (Codex 2026-11-21): availability (_robot_block: readiness, mode, localization, goal check) and a busy hint are read
        WITHOUT the server lock; the decision - request still waiting, FIFO head, robot not reserved by any in-flight mission or other
        guide request - and the mission creation + request update happen under ONE self.lock section (DB only: no ROS / HTTP call
        inside; the Agent pulls the mission later). Two dispatchers can therefore never both create a mission for the same robot, and
        the mission idempotency key 'guide-<request>' makes a repeated dispatch of the same request return the same mission."""
        from .service import ApiError
        row = self.db.one('SELECT * FROM guide_requests WHERE request_id=?', (rq,))
        if not row or row['status'] not in ('RECEIVED', 'QUEUED'): return
        d = self.db.one('SELECT * FROM guide_destinations WHERE destination_id=?', (row['destination_id'],))
        mblk, mv = self._map_block(d) if d is not None else ('DESTINATION_DISABLED', None)
        for s in ([] if mblk else self.db.q('SELECT * FROM guide_service WHERE enabled=1')):
            rid = s['robot_id']
            if self._robot_block(rid, d, mv) is not None: continue
            hint = self._robot_busy(rid)
            with self.lock:
                row = self.db.one('SELECT * FROM guide_requests WHERE request_id=?', (rq,))
                if not row or row['status'] not in ('RECEIVED', 'QUEUED'): return          # canceled / handled meanwhile
                cur = self.db.one('SELECT * FROM guide_service WHERE robot_id=?', (rid,))
                if not cur or not cur['enabled'] or cur['operator'] != s['operator'] or cur['lease_epoch'] != s['lease_epoch']: continue
                if hint or self._older_waiting(row) or self._robot_reserved(rid, rq):
                    self._req_update(rq, 'QUEUED', 'ROBOT_BUSY', robot_id=rid); return
                try:
                    view, _ = self.create_mission(rid, 'navigate_to_pose', {'frame_id': 'map', 'x': d['x'], 'y': d['y'], 'yaw_rad': d['yaw']},
                                                  idempotency_key='guide-' + rq, note=f"visitor guide {rq} -> {d['name']}"[:200],
                                                  created_by=cur['operator'], lease_epoch=cur['lease_epoch'])
                except ApiError as e:
                    self._req_update(rq, 'QUEUED' if e.code in TRANSIENT_CREATE_ERRORS else 'FAILED', e.code, robot_id=rid); return
                except Exception as e:                  # never leave the request RECEIVED or the robot half-reserved
                    self._req_update(rq, 'FAILED', 'MISSION_CREATE_ERROR', robot_id=rid)
                    self._guide_warn('dispatch', f'mission creation for {rq} failed: {type(e).__name__}'); return
                self._req_update(rq, 'ACCEPTED', None, mission_id=view['mission_id'], robot_id=rid); return
        blk = (mblk, None) if mblk else self.destination_block(d)
        with self.lock:
            row = self.db.one('SELECT * FROM guide_requests WHERE request_id=?', (rq,))
            if row and row['status'] in ('RECEIVED', 'QUEUED'):
                self._req_update(rq, 'QUEUED' if row['status'] == 'QUEUED' else 'UNAVAILABLE', blk[0] if blk else 'SERVICE_CLOSED')

    def guide_tick(self):
        """Housekeeping: follow missions, dispatch queued requests FIFO. Never raises into housekeeping."""
        self._ensure_guide_schema()             # also after a server restart: in-flight requests keep following their missions
        for r in self.db.q("SELECT * FROM guide_requests WHERE status IN ('ACCEPTED','IN_PROGRESS','CANCELING') AND mission_id IS NOT NULL"):
            m = self.db.one('SELECT status, detail FROM missions WHERE mission_id=?', (r['mission_id'],))
            if not m: continue
            st = MISSION_TO_REQUEST.get(m['status'], r['status'])
            reason = None if st in ('ACCEPTED', 'IN_PROGRESS', 'COMPLETED', 'CANCELING') else ('MISSION_CANCELED' if st == 'CANCELED' else 'MISSION_FAILED')
            with self.lock: self._req_update(r['request_id'], st, reason)
        with self.lock:                          # a request whose creating call died before dispatch (server crash) joins the queue
            for r in self.db.q("SELECT request_id FROM guide_requests WHERE status='RECEIVED' AND created_at < ?", (time.time() - RECEIVED_STALE_S,)):
                self._req_update(r['request_id'], 'QUEUED', 'ROBOT_BUSY')
        for r in self.db.q("SELECT request_id FROM guide_requests WHERE status='QUEUED' ORDER BY created_at, request_id"):
            self._dispatch_one(r['request_id'])
            if self.db.one('SELECT status FROM guide_requests WHERE request_id=?', (r['request_id'],))['status'] == 'QUEUED': break   # FIFO

    def _own_request(self, rq, visitor):
        """The request row if it belongs to this visitor: same link AND same browser session (Codex 2026-11-21: the session id alone,
        or a request id alone, is not ownership). Otherwise 404 (no existence oracle)."""
        from .service import ApiError
        row = self.db.one('SELECT * FROM guide_requests WHERE request_id=?', (rq,)) if isinstance(rq, str) else None
        if (not row or not hmac.compare_digest(str(row['link_id']), str(visitor.link_id))
                or not hmac.compare_digest(str(row['session']), str(visitor.session))):
            raise ApiError(404, 'unknown_request', 'no such request')
        return row

    def guide_request_view(self, rq, visitor=None):
        from .service import ApiError
        self._ensure_guide_schema()
        row = self._own_request(rq, visitor) if visitor is not None else self.db.one('SELECT * FROM guide_requests WHERE request_id=?', (rq,))
        if not row: raise ApiError(404, 'unknown_request', 'no such request')
        d = self.db.one('SELECT name, floor_id FROM guide_destinations WHERE destination_id=?', (row['destination_id'],)) or {}
        pos = None
        if row['status'] == 'QUEUED':
            pos = self.db.one("SELECT COUNT(*) n FROM guide_requests WHERE status='QUEUED' AND created_at <= ?", (row['created_at'],))['n']
        v = {'request_id': rq, 'destination_id': row['destination_id'], 'destination_name': d.get('name'), 'status': row['status'],
             'status_text': STATUS_TEXT.get(row['status'], row['status']), 'reason': row['reason'], 'reason_text': REASON_TEXT.get(row['reason']) if row['reason'] else None,
             'queue_position': pos, 'created_at': row['created_at'], 'updated_at': row['updated_at'], 'terminal': row['status'] not in ACTIVE,
             'history': json.loads(row['history_json'])}
        if visitor is None:
            v.update(robot_id=row['robot_id'], mission_id=row['mission_id'], link_id=row['link_id'])
        return v

    def cancel_guide_request(self, visitor, rq):
        from .service import ApiError
        self._ensure_guide_schema()
        with self.lock:
            row = self._own_request(rq, visitor)
            if row['status'] not in ACTIVE: return self.guide_request_view(rq, visitor)
            mid = row['mission_id']
            m = self.db.one('SELECT mission_id, idempotency_key FROM missions WHERE mission_id=?', (mid,)) if mid else None
            if m is not None and m['idempotency_key'] != 'guide-' + rq:
                self._guide_warn('cancel', f'request {rq} points at mission {mid} it did not create; cancel not forwarded')
                m = None
            if m is not None and row['status'] in ('ACCEPTED', 'IN_PROGRESS', 'CANCELING'):
                try: self.cancel_mission(m['mission_id'], f'visitor:{visitor.link_id}')
                except ApiError as e: self.db.event('guide_request', f'visitor cancel of {rq} not forwarded: {e.code}', row['robot_id'], level='warning')
                self._req_update(rq, 'CANCELING', None)
            else:
                self._req_update(rq, 'CANCELED', 'MISSION_CANCELED')
        return self.guide_request_view(rq, visitor)

    def guide_requests(self, limit=50):
        self._ensure_guide_schema()
        return [self.guide_request_view(r['request_id']) for r in self.db.q('SELECT request_id FROM guide_requests ORDER BY created_at DESC LIMIT ?', (int(limit),))]
