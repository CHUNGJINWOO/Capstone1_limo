"""HTTP layer (stdlib ThreadingHTTPServer). JSON API under /api/v1, static dashboard under /.
Every /api/v1 endpoint except /health needs `Authorization: Bearer <token>`; role checks per endpoint (see docs/API.md)."""
import base64, ipaddress, json, logging, mimetypes, os, re, socket, threading, time, traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from . import API_VERSION, __version__
from .service import ApiError

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
MAX_BODY, MAX_MAP = 1 * 1024 * 1024, 6 * 1024 * 1024
MAX_MAPPING_BUNDLE = 48 * 1024 * 1024   # base64 raster + SLAM posegraph (a 10 MiB graph was observed on 2026-11-19)
MAP_REGISTRY_POST_ROUTES = frozenset({'/api/v1/map-registry/facilities', '/api/v1/map-registry/buildings', '/api/v1/map-registry/floors', '/api/v1/map-registry/areas', '/api/v1/map-registry/maps'})
MAX_CONCURRENT_MAP_UPLOADS = 2
MAP_UPLOAD_WAIT_S = 1.0
MAP_UPLOAD_READ_S = 10.0


def check_bind_address(addr: str):
    """Refuse wildcard / public addresses: only loopback, private (RFC1918) or Tailscale CGNAT (100.64.0.0/10) addresses may be bound."""
    ip = ipaddress.ip_address(addr)
    if ip.is_unspecified: raise SystemExit(f'refusing to bind the wildcard address {addr} (would expose the service on every interface)')
    ok = ip.is_loopback or ip.is_private or (ip.version == 4 and ip in ipaddress.ip_network('100.64.0.0/10'))
    if not ok: raise SystemExit(f'refusing to bind {addr}: not loopback / private / Tailscale (100.64.0.0/10)')


def make_handler(fleet, auth, video_origins=(), map_registry=None):
    upload_slots = threading.BoundedSemaphore(MAX_CONCURRENT_MAP_UPLOADS)
    fleet.map_registry = map_registry
    sensor_lock = threading.Lock()
    sensor_store = {}
    # Optional extra origins the dashboard page may CONNECT to (the separate video/AI host). Default: none, CSP unchanged.
    csp = "default-src 'self'; img-src 'self' blob: data:; style-src 'self' 'unsafe-inline'" + ("; connect-src 'self' " + ' '.join(video_origins) if video_origins else '')
    class H(BaseHTTPRequestHandler):
        server_version = f'CapstoneFleet/{__version__}'
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):                                # quiet; failures are logged by _fail
            pass

        # ------------------------------------------------ helpers
        def _send(self, status, body=b'', ctype='application/json', extra=None):
            if isinstance(body, (dict, list)): body = json.dumps(body).encode()
            self.send_response(status)
            self.send_header('Content-Type', ctype); self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store'); self.send_header('X-Content-Type-Options', 'nosniff'); self.send_header('X-Frame-Options', 'DENY')
            self.send_header('Content-Security-Policy', csp)
            for k, v in (extra or {}).items(): self.send_header(k, v)
            self.end_headers()
            if self.command != 'HEAD': self.wfile.write(body)

        def _err(self, status, code, message): self._send(status, {'error': {'code': code, 'message': message}})

        def _body(self, limit=MAX_BODY):
            n = int(self.headers.get('Content-Length') or 0)
            if n > limit: raise ApiError(413, 'too_large', f'body larger than {limit} bytes')
            raw = self.rfile.read(n) if n else b''
            try: return json.loads(raw) if raw else {}
            except ValueError: raise ApiError(400, 'bad_json', 'body is not valid JSON')

        def _raw_body_limited(self, limit):
            """Read the raw body for Map Registry uploads. Size is checked BEFORE reading; on refusal the connection is closed so unread bytes cannot poison keep-alive."""
            if self.headers.get('Transfer-Encoding'):
                self.close_connection = True; raise ApiError(400, 'unsupported_transfer_encoding', 'Content-Length is required')
            try: n = int(self.headers.get('Content-Length') or 0)
            except ValueError: self.close_connection = True; raise ApiError(400, 'bad_content_length', 'invalid Content-Length')
            if n < 0: self.close_connection = True; raise ApiError(400, 'bad_content_length', 'invalid Content-Length')
            if n > limit: self.close_connection = True; raise ApiError(413, 'too_large', f'body larger than {limit} bytes')
            old_timeout = self.connection.gettimeout()
            deadline = time.monotonic() + MAP_UPLOAD_READ_S
            chunks = []; remaining = n
            try:
                while remaining:
                    left = deadline - time.monotonic()
                    if left <= 0: raise TimeoutError()
                    self.connection.settimeout(left)
                    chunk = self.rfile.read1(min(remaining, 65536))
                    if not chunk:
                        self.close_connection = True; raise ApiError(400, 'incomplete_body', 'request body ended early')
                    chunks.append(chunk); remaining -= len(chunk)
                return b''.join(chunks)
            except (TimeoutError, socket.timeout):
                self.close_connection = True; raise ApiError(408, 'request_timeout', 'request body read timed out')
            finally:
                self.connection.settimeout(old_timeout)

        def _principal(self):
            ip = self.client_address[0]
            if auth.locked_out(ip): raise ApiError(429, 'locked_out', 'too many failed authentications; try again later')
            p = auth.authenticate(self.headers.get('Authorization', ''), ip)
            if p is None: raise ApiError(401, 'unauthorized', 'missing or invalid bearer token')
            return p

        # ------------------------------------------------ routing
        def do_HEAD(self): self.do_GET()

        def do_GET(self): self._route('GET')
        def do_POST(self): self._route('POST')
        def do_PUT(self): self._err(405, 'method_not_allowed', 'method not allowed')
        def do_DELETE(self): self._err(405, 'method_not_allowed', 'method not allowed')

        def _route(self, method):
            try: self._route_inner(method)
            except (BrokenPipeError, ConnectionResetError):
                self.close_connection = True
                logging.getLogger(__name__).warning('client connection closed during request')

        def _route_inner(self, method):
            try:
                u = urlparse(self.path); path, qs = u.path, {k: v[0] for k, v in parse_qs(u.query).items()}
                if not path.startswith('/api/'): return self._static(path) if method == 'GET' else self._err(405, 'method_not_allowed', 'method not allowed')
                self._api(method, path, qs)
            except ApiError as e:
                if method == 'POST': self.close_connection = True
                self._err(e.status, e.code, e.message)
            except (BrokenPipeError, ConnectionResetError): raise
            except Exception:
                traceback.print_exc(); self._err(500, 'internal', 'internal server error')

        def _static(self, path):
            if path in ('', '/'): path = '/index.html'
            fp = os.path.normpath(os.path.join(STATIC, path.lstrip('/')))
            if not fp.startswith(STATIC + os.sep) or not os.path.isfile(fp): return self._err(404, 'not_found', 'not found')
            with open(fp, 'rb') as f: data = f.read()
            self._send(200, data, mimetypes.guess_type(fp)[0] or 'application/octet-stream')

        def _visitor(self, method, sub, qs):
            ip = self.client_address[0]
            if auth.locked_out(ip): raise ApiError(429, 'locked_out', 'too many failed attempts; try again later')
            try: v = fleet.visitor_auth(self.headers.get('Authorization', ''), self.headers.get('X-Visitor-Session'))
            except ApiError as e:
                if e.status == 401: auth.fail[ip].append(time.time())
                raise
            if method == 'GET' and sub == '/visitor/catalog': return self._send(200, fleet.visitor_catalog())
            if method == 'GET' and sub == '/visitor/destinations':
                return self._send(200, {'destinations': fleet.destinations(qs.get('floor_id'), qs.get('area_id'), (qs.get('q') or '')[:40] or None)})
            if method == 'POST' and sub == '/visitor/requests': return self._send(201, fleet.create_guide_request(v, self._body()))
            m = re.fullmatch(r'/visitor/requests/(gr_[a-f0-9]{12})', sub)
            if m and method == 'GET': return self._send(200, fleet.guide_request_view(m.group(1), v))
            m = re.fullmatch(r'/visitor/requests/(gr_[a-f0-9]{12})/cancel', sub)
            if m and method == 'POST': self._body(); return self._send(202, fleet.cancel_guide_request(v, m.group(1)))
            raise ApiError(404, 'not_found', 'unknown visitor endpoint')

        def _agent_slam_map(self, b, p):
            if not isinstance(b,dict): raise ApiError(400,'bad_json','object required')
            rid=b.get('robot_id')
            if not isinstance(rid,str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,64}',rid): raise ApiError(400,'bad_robot_id','invalid robot id')
            if not p.can_act_as_agent(rid): raise ApiError(403,'forbidden','agent identity mismatch')
            import base64
            fleet.check_session(rid, b.get('agent_boot_id'))
            if map_registry is None: raise ApiError(404,'registry_unavailable','map registry unavailable')
            command=fleet.db.one('SELECT * FROM commands WHERE command_id=? AND robot_id=?', (b.get('command_id'),rid))
            if not command or (command['type']!='slam_save' and not (command['type']=='slam_stop' and json.loads(command['payload_json']).get('manual_stop') is True)) or command['owner_boot_id']!=b.get('agent_boot_id') or command['status'] not in ('SENT','ACCEPTED','EXECUTING'):
                raise ApiError(403,'invalid_save_command','owned active save command required')
            payload=json.loads(command['payload_json'])
            if b.get('session_id')!=payload.get('session_id') or b.get('floor_id')!=payload.get('floor_id'):
                raise ApiError(409,'session_mismatch','session or floor does not match save command')
            try:
                yaml_bytes=base64.b64decode(b['yaml_b64'],validate=True);pgm_bytes=base64.b64decode(b['pgm_b64'],validate=True)
                graph_bytes=base64.b64decode(b['posegraph_b64'],validate=True) if b.get('posegraph_b64') is not None else None
                data_bytes=base64.b64decode(b['posegraph_data_b64'],validate=True) if b.get('posegraph_data_b64') is not None else None
            except (ValueError,KeyError,TypeError):raise ApiError(400,'invalid_map','invalid map encoding')
            from .map_registry import MapRegistryError
            try:
                result=map_registry.register_map_bundle(floor_id=payload['floor_id'],label=payload['session_id'],yaml_bytes=yaml_bytes,pgm_bytes=pgm_bytes,posegraph_bytes=graph_bytes,posegraph_data_bytes=data_bytes,source_mode='SLAM_LIVE_SNAPSHOT',actor=rid,actor_role='agent')
            except MapRegistryError as exc:
                raise ApiError(400,'map_registry_rejected',str(exc)[:200])
            integrity=map_registry.verify_bundle_integrity(result['map_version']['map_version_id'])
            if not integrity.get('ok'):raise ApiError(500,'registry_integrity_failed','registered map bundle integrity check failed')
            return self._send(200,{'map_version':result['map_version'],'created':result['created'],'integrity':integrity})

        def _agent_mapping_map(self, b, p):
            """Register the bundle saved by an accepted web-mapping SAVE (UNVERIFIED, optional area link)."""
            if not isinstance(b,dict): raise ApiError(400,'bad_json','object required')
            rid=b.get('robot_id')
            if not isinstance(rid,str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,64}',rid): raise ApiError(400,'bad_robot_id','invalid robot id')
            if not p.can_act_as_agent(rid): raise ApiError(403,'forbidden','agent identity mismatch')
            fleet.check_session(rid, b.get('agent_boot_id'))
            if map_registry is None: raise ApiError(404,'registry_unavailable','map registry unavailable')
            cmd=fleet.mapping_save_command(rid, b.get('agent_boot_id'), b.get('session_id'), b.get('command_id'))
            if not cmd: raise ApiError(403,'invalid_save_command','accepted SAVE with a registry target required')
            target=cmd['registry_target']
            try:
                yaml_bytes=base64.b64decode(b['yaml_b64'],validate=True);pgm_bytes=base64.b64decode(b['pgm_b64'],validate=True)
                graph_bytes=base64.b64decode(b['posegraph_b64'],validate=True) if b.get('posegraph_b64') is not None else None
                data_bytes=base64.b64decode(b['posegraph_data_b64'],validate=True) if b.get('posegraph_data_b64') is not None else None
            except (ValueError,KeyError,TypeError):raise ApiError(400,'invalid_map','invalid map encoding')
            evidence=b.get('evidence') if isinstance(b.get('evidence'),dict) else {}
            evidence=dict(evidence, method='web_mapping_save', session_id=cmd['session_id'], command_id=cmd['command_id'],
                          robot_id=rid, operator=cmd['owner'], location={'source':'operator_selected','auto_detected':False, **target})
            from .map_registry import MapRegistryError
            label='web_'+cmd['session_id'][:12]+'_'+cmd['command_id'][:6]
            try:
                result=map_registry.register_map_bundle(floor_id=target['floor_id'],label=label,yaml_bytes=yaml_bytes,pgm_bytes=pgm_bytes,
                    posegraph_bytes=graph_bytes,posegraph_data_bytes=data_bytes,source_mode='SLAM_SAVED_BUNDLE',
                    verification_evidence=evidence,actor=rid,actor_role='agent')
                mvid=result['map_version']['map_version_id']
                if target.get('area_id'): map_registry.link_map_version_area(mvid, target['area_id'], actor=rid)
                plan=cmd.get('plan') or {}
                map_registry.record_lineage(mvid, parent_map_version_id=plan.get('parent_map_version_id'),
                                            mapping_mode=plan.get('mode','NEW_MAP'), session_id=cmd['session_id'], actor=rid)
            except MapRegistryError as exc:
                raise ApiError(400,'map_registry_rejected',str(exc)[:200])
            integrity=map_registry.verify_bundle_integrity(mvid)
            if not integrity.get('ok'):raise ApiError(500,'registry_integrity_failed','registered map bundle integrity check failed')
            fleet.mapping_registered(rid, cmd['command_id'], mvid, target.get('area_id'))
            return self._send(200,{'map_version_id':mvid,'created':result['created'],'integrity':integrity})

        def _api(self, method, path, qs):
            pre = f'/api/{API_VERSION}'
            if not path.startswith(pre + '/') and path != pre: raise ApiError(404, 'not_found', f'use {pre}')
            sub = path[len(pre):].rstrip('/') or '/'
            if sub == '/health' and method == 'GET': return self._send(200, {'status': 'ok', 'api': API_VERSION})
            if sub == '/visitor' or sub.startswith('/visitor/'):
                return self._visitor(method, sub, qs)            # Phase 5: own token scheme; never a Bearer principal
            p = self._principal()
            # ---------------- Map Registry: GET (Phase 2-A) + registration-only POST (Phase 2-C)
            if sub == '/map-registry' or sub.startswith('/map-registry/'):
                if map_registry is None:
                    raise ApiError(404, 'not_found', 'map registry is not enabled on this server')
                if method == 'GET':
                    if not p.can_read():
                        raise ApiError(403, 'forbidden', 'role may not read')
                    st, body, ctype = map_registry.dispatch_readonly('GET', path, p, qs)
                    return self._send(st, body, ctype)
                # Only the four registration routes are routed in production. verify / revoke / quarantine / quarantine-clear and
                # any floor active-map change stay UNROUTED (404) until the permission + Safety Context policy is decided.
                if path.rstrip('/') not in MAP_REGISTRY_POST_ROUTES:
                    raise ApiError(404, 'not_found', 'unknown map-registry endpoint')
                if not p.can_control():
                    self.close_connection = True  # POST body was not consumed; do not reuse this connection.
                    raise ApiError(403, 'forbidden', 'role may not write map registry')
                if not upload_slots.acquire(timeout=MAP_UPLOAD_WAIT_S):
                    self.close_connection = True
                    raise ApiError(503, 'busy', 'too many concurrent map uploads; retry shortly')
                try:
                    raw = self._raw_body_limited(map_registry.max_http_write_bytes)
                    hdrs = {'Idempotency-Key': self.headers.get('Idempotency-Key')} if self.headers.get('Idempotency-Key') else None
                    st, body, ctype = map_registry.dispatch_write('POST', path, p, body=raw, headers=hdrs)
                finally:
                    upload_slots.release()
                return self._send(st, body, ctype)
            if sub == '/agent/map-save':
                if method != 'POST': raise ApiError(405,'method_not_allowed','POST only')
                if not upload_slots.acquire(timeout=MAP_UPLOAD_WAIT_S):
                    self.close_connection=True; raise ApiError(503,'busy','map upload busy')
                try:
                    raw=self._raw_body_limited(MAX_MAPPING_BUNDLE)
                    try: b=json.loads(raw)
                    except ValueError: raise ApiError(400,'bad_json','invalid JSON')
                    if not isinstance(b,dict): raise ApiError(400,'bad_json','object required')
                    rid=b.get('robot_id')
                    if not isinstance(rid,str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,64}',rid): raise ApiError(400,'bad_robot_id','invalid robot id')
                    if not p.can_act_as_agent(rid): raise ApiError(403,'forbidden','agent identity mismatch')
                    return self._send(200, fleet.map_save_agent(rid, b.get('agent_boot_id'), b))
                finally: upload_slots.release()
            if sub == '/agent/mapping-map':
                if method != 'POST': raise ApiError(405,'method_not_allowed','POST only')
                if not upload_slots.acquire(timeout=MAP_UPLOAD_WAIT_S):
                    self.close_connection=True; raise ApiError(503,'busy','map upload busy')
                try:
                    raw=self._raw_body_limited(MAX_MAPPING_BUNDLE)
                    try: b=json.loads(raw)
                    except ValueError: raise ApiError(400,'bad_json','invalid JSON')
                    return self._agent_mapping_map(b,p)
                finally: upload_slots.release()
            if sub == '/agent/slam-map':
                if method != 'POST': raise ApiError(405,'method_not_allowed','POST only')
                if not upload_slots.acquire(timeout=MAP_UPLOAD_WAIT_S):
                    self.close_connection=True; raise ApiError(503,'busy','map upload busy')
                try:
                    raw=self._raw_body_limited(MAX_MAP)
                    try: b=json.loads(raw)
                    except ValueError: raise ApiError(400,'bad_json','invalid JSON')
                    return self._agent_slam_map(b,p)
                finally: upload_slots.release()
            # ---------------- agent endpoints (token must belong to the robot named in the body)
            if sub.startswith('/agent/'):
                if method != 'POST': raise ApiError(405, 'method_not_allowed', 'POST only')
                limit = MAX_MAP if sub in ('/agent/map','/agent/slam-map','/agent/sensors') else MAX_BODY
                b = self._body(limit); rid = b.get('robot_id')
                if not isinstance(rid, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,64}', rid): raise ApiError(400, 'bad_robot_id', 'robot_id must match [A-Za-z0-9_.-]{1,64}')
                if not p.can_act_as_agent(rid): raise ApiError(403, 'forbidden', 'this token may not act as that robot')
                if sub == '/agent/operation':
                    if b.get('kind') == 'bag':          # G2: origin/main BagWorker journal (bag_operation.py)
                        return self._send(200, fleet.bag_worker_operation(rid, b.get('agent_boot_id'), b))
                    return self._send(200, fleet.worker_operation(rid, b.get('agent_boot_id'), b))
                if sub == '/agent/camera':
                    return self._send(200, fleet.ingest_camera(rid, b.get('agent_boot_id'), b))
                if sub == '/agent/manual-bench':
                    return self._send(200, fleet.pull_manual_bench(rid, b.get('agent_boot_id')))
                if sub == '/agent/hello': return self._send(200, fleet.hello(rid, b.get('agent') or {}))
                if sub == '/agent/reconcile': return self._send(200, fleet.reconcile(rid, b.get('agent_boot_id'), b.get('journal', {}), b.get('execution', {}), b.get('journal_ok') is True))
                if sub == '/agent/state': return self._send(200, fleet.ingest_state(rid, int(b.get('seq', 0)), b.get('state') or {}, b.get('agent_boot_id')))
                if sub == '/agent/operation':
                    if b.get('kind') == 'bag':          # G2: origin/main BagWorker journal (bag_operation.py)
                        return self._send(200, fleet.bag_worker_operation(rid, b.get('agent_boot_id'), b))
                    return self._send(200, fleet.worker_operation(rid, b.get('agent_boot_id'), b))
                if sub == '/agent/map-bundle':
                    return self._send(200, fleet.agent_map_bundle(rid, b.get('agent_boot_id'), b))
                if sub == '/agent/supervisor':                     # Phase 4: robot-side supervisor report (+ queued service actions)
                    return self._send(200, fleet.supervisor_report(rid, b))
                if sub == '/agent/manual-bench':
                    return self._send(200, fleet.pull_manual_bench(rid, b.get('agent_boot_id')))
                if sub == '/agent/map':
                    fleet.check_session(rid, b.get('agent_boot_id'))
                    return self._send(200, fleet.ingest_map(rid, int(b['version']), b.get('meta') or {}, b['png_base64']))
                if sub == '/agent/sensors':
                    now = time.time()
                    def _dec_b64(key):
                        val = b.get(key)
                        if not isinstance(val, str) or not val: return None
                        try:
                            raw = base64.b64decode(val.encode('ascii'), validate=True)
                            return raw if len(raw) <= 2 * 1024 * 1024 else None
                        except Exception:
                            return None
                    with sensor_lock:
                        prev = sensor_store.get(rid, {})
                        rgb_jpg = _dec_b64('rgb_jpeg_base64') or prev.get('rgb_jpg')
                        depth_jpg = _dec_b64('depth_jpeg_base64') or prev.get('depth_jpg')
                        costmap_png = _dec_b64('costmap_png_base64') or prev.get('costmap_png')
                        lidar_png = _dec_b64('lidar_png_base64') or prev.get('lidar_png')
                        seq = int(prev.get('seq', 0)) + 1
                        sensor_store[rid] = {
                            'seq': seq,
                            'received_at': now,
                            'stamp': float(b.get('stamp') or now),
                            'pose': b.get('pose') if isinstance(b.get('pose'), dict) else prev.get('pose'),
                            'lidar_points': (b.get('lidar_points') or [])[:600] if 'lidar_points' in b else prev.get('lidar_points', []),
                            'global_plan': (b.get('global_plan') or [])[:600] if 'global_plan' in b else prev.get('global_plan', []),
                            'frontiers': (b.get('frontiers') or [])[:600] if 'frontiers' in b else prev.get('frontiers', []),
                            'costmap_meta': b.get('costmap_meta') if isinstance(b.get('costmap_meta'), dict) else prev.get('costmap_meta'),
                            'depth_meta': b.get('depth_meta') if isinstance(b.get('depth_meta'), dict) else prev.get('depth_meta'),
                            'rgb_meta': b.get('rgb_meta') if isinstance(b.get('rgb_meta'), dict) else prev.get('rgb_meta'),
                            'summary': b.get('summary') if isinstance(b.get('summary'), dict) else prev.get('summary', {}),
                            'rgb_jpg': rgb_jpg,
                            'depth_jpg': depth_jpg,
                            'costmap_png': costmap_png,
                            'lidar_png': lidar_png,
                        }
                    return self._send(200, {'ok': True, 'robot_id': rid, 'seq': seq})
                m = re.fullmatch(r'/agent/commands/([A-Za-z0-9_-]{1,64})/ack', sub)
                if m:
                    fleet.check_session(rid, b.get('agent_boot_id'))
                    return self._send(200, fleet.ack_command(rid, m.group(1), str(b.get('status')), str(b.get('detail', ''))[:300], b.get('progress'), b.get('agent_boot_id')))
                raise ApiError(404, 'not_found', 'unknown agent endpoint')
            # ---------------- read endpoints
            if method == 'GET':
                if not p.can_read(): raise ApiError(403, 'forbidden', 'role may not read')
                if sub == '/me': return self._send(200, {'name': p.name, 'role': p.role, 'robot_id': p.robot_id})
                if sub == '/dashboard':
                    return self._send(200, {'server': {'time': time.time(), 'boot_id': fleet.boot_id, 'version': __version__, 'api': API_VERSION}, 'robots': fleet.robots_view(),
                                            'missions': fleet.missions(limit=40), 'events': fleet.events(limit=80)})
                if sub == '/robots': return self._send(200, {'robots': fleet.robots_view()})
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/navigation', sub)
                if m:
                    if m.group(1) not in fleet.robots: raise ApiError(404, 'unknown_robot', 'no such robot')
                    with fleet.lock: return self._send(200, fleet.navigation_view(m.group(1)))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/goal-check', sub)
                if m:
                    if m.group(1) not in fleet.robots: raise ApiError(404, 'unknown_robot', 'no such robot')
                    try: gx, gy = float(qs.get('x')), float(qs.get('y'))
                    except (TypeError, ValueError): raise ApiError(400, 'bad_goal', 'numeric x and y required')
                    # No Fleet lock: raster checks must never starve Agent state posts (record read is a single DB row).
                    return self._send(200, fleet.goal_check(m.group(1), gx, gy))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/gate-reset', sub)
                if m: return self._send(200, fleet.gate_reset_view(m.group(1), p))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/readiness', sub)
                if m: return self._send(200, fleet.readiness(m.group(1)))
                if sub == '/guide/destinations': return self._send(200, {'destinations': fleet.destinations(internal=True)})
                if sub == '/guide/requests': return self._send(200, {'requests': fleet.guide_requests(min(int(qs.get('limit', 50)), 200))})
                m = re.fullmatch(r'/guide/service/([A-Za-z0-9_.-]{1,64})', sub)
                if m: return self._send(200, fleet.guide_service_view(m.group(1)))
                if sub == '/visitor-links':
                    if p.role != 'admin': raise ApiError(403, 'forbidden', 'admin only')
                    return self._send(200, {'links': fleet.visitor_links()})
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/mode', sub)
                if m:
                    if m.group(1) not in fleet.robots: raise ApiError(404, 'unknown_robot', 'no such robot')
                    return self._send(200, {'robot_id': m.group(1), 'operating_mode': fleet.mode_view(m.group(1)),
                                            'control_mode': fleet.control_mode(m.group(1))})
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/mapping', sub)
                if m: return self._send(200, fleet.operation_status(m.group(1)))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/map-saves', sub)
                if m: return self._send(200, fleet.map_save_status(m.group(1), min(int(qs.get('limit', 10)), 50)))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/recording', sub)
                if m: return self._send(200, fleet.recording_status(m.group(1)))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/(sensors|rgb\.jpg|depth\.jpg|costmap\.png|lidar\.png)', sub)
                if m:
                    rid, kind = m.group(1), m.group(2)
                    with sensor_lock:
                        rec = sensor_store.get(rid)
                    if kind == 'sensors':
                        if not rec:
                            return self._send(200, {'available': False, 'robot_id': rid})
                        age = round(max(0.0, time.time() - rec['received_at']), 2)
                        return self._send(200, {
                            'available': True, 'robot_id': rid, 'seq': rec['seq'], 'stamp': rec['stamp'], 'age_s': age,
                            'pose': rec['pose'], 'lidar_points': rec['lidar_points'],
                            'global_plan': rec['global_plan'], 'frontiers': rec['frontiers'],
                            'costmap_meta': rec['costmap_meta'], 'depth_meta': rec['depth_meta'],
                            'rgb_meta': rec.get('rgb_meta'), 'summary': rec['summary'],
                            'has_rgb': rec.get('rgb_jpg') is not None,
                            'has_depth': rec['depth_jpg'] is not None,
                            'has_costmap': rec['costmap_png'] is not None,
                            'has_lidar_img': rec['lidar_png'] is not None,
                        })
                    if kind == 'rgb.jpg':
                        if not rec or not rec.get('rgb_jpg'): raise ApiError(404, 'no_rgb', 'no rgb image yet')
                        return self._send(200, rec['rgb_jpg'], 'image/jpeg', {'X-Sensor-Seq': str(rec['seq'])})
                    if kind == 'depth.jpg':
                        if not rec or not rec.get('depth_jpg'): raise ApiError(404, 'no_depth', 'no depth image yet')
                        return self._send(200, rec['depth_jpg'], 'image/jpeg', {'X-Sensor-Seq': str(rec['seq'])})
                    if kind == 'costmap.png':
                        if not rec or not rec.get('costmap_png'): raise ApiError(404, 'no_costmap', 'no costmap image yet')
                        return self._send(200, rec['costmap_png'], 'image/png', {'X-Sensor-Seq': str(rec['seq'])})
                    if kind == 'lidar.png':
                        if not rec or not rec.get('lidar_png'): raise ApiError(404, 'no_lidar', 'no lidar image yet')
                        return self._send(200, rec['lidar_png'], 'image/png', {'X-Sensor-Seq': str(rec['seq'])})
                camera = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/camera/(rgb|depth)\.jpg', sub)
                if camera:
                    frame, source = fleet.camera_frame(*camera.groups())
                    return self._send(200, frame, 'image/jpeg', {'X-Camera-Source': source})
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})(/map\.png|/map\.json)?', sub)
                if m:
                    rid = m.group(1)
                    if rid not in fleet.robots: raise ApiError(404, 'unknown_robot', 'no such robot')
                    if m.group(2) == '/map.png':
                        png, ver, meta = fleet.map_png(rid); return self._send(200, png, 'image/png', {'X-Map-Version': str(ver)})
                    if m.group(2) == '/map.json':
                        _, ver, meta = fleet.map_png(rid); return self._send(200, {'version': ver, 'meta': meta})
                    return self._send(200, fleet.robot_view(fleet.robots[rid]))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/lock', sub)
                if m:
                    if m.group(1) not in fleet.robots: raise ApiError(404, 'unknown_robot', 'no such robot')
                    return self._send(200, {'lease': fleet.lease_view(m.group(1))})
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/manual-bench', sub)
                if m:
                    return self._send(200, fleet.manual_preview_status(m.group(1)))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/bag', sub)
                if m:
                    return self._send(200, fleet.bag_status(m.group(1)))          # G2: origin/main Bag session view
                if sub == '/missions': return self._send(200, {'missions': fleet.missions(qs.get('robot_id'), qs.get('status'), min(int(qs.get('limit', 50)), 500))})
                m = re.fullmatch(r'/missions/(m_[a-f0-9]{12})', sub)
                if m: return self._send(200, fleet.mission(m.group(1)))
                if sub == '/events': return self._send(200, {'events': fleet.events(qs.get('since_id', 0), min(int(qs.get('limit', 100)), 500))})
                raise ApiError(404, 'not_found', 'unknown endpoint')
            # ---------------- control endpoints
            if method == 'POST':
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/gate-reset', sub)
                if m:
                    # 2026-11-21: role is checked (and every refusal audited) inside request_gate_reset, for every caller role.
                    return self._send(202, fleet.request_gate_reset(m.group(1), p, self._body()))
                if sub == '/visitor-links': return self._send(201, fleet.create_visitor_link(p, self._body()))
                m = re.fullmatch(r'/visitor-links/(vl_[a-f0-9]{10})/revoke', sub)
                if m: self._body(); return self._send(200, fleet.revoke_visitor_link(p, m.group(1)))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/services', sub)
                if m:
                    # Phase 4: admin-only predefined service actions; the role check and its audit event are inside.
                    return self._send(202, fleet.request_service_action(m.group(1), p, self._body()))
                if not p.can_control():
                    self.close_connection = True  # POST body was not consumed.
                    raise ApiError(403, 'forbidden', 'role may not create or cancel missions')
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/manual-bench', sub)
                if m:
                    return self._send(200, fleet.manual_bench(m.group(1), p.name, self._body()))
                if sub == '/guide/destinations': return self._send(201, fleet.upsert_destination(p, self._body()))
                if sub == '/guide/service': return self._send(200, fleet.set_guide_service(p, self._body()))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/navigation', sub)
                if m:
                    # 202: the Agent loads the map / applies the pose; results appear in GET .../navigation (localization).
                    return self._send(202, fleet.request_navigation(m.group(1), p.name, self._body()))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/mode', sub)
                if m:
                    # 202: the transition is queued; its result is in GET .../mode (transition.state ACTIVE | FAILED).
                    return self._send(202, fleet.request_mode(m.group(1), p.name, self._body()))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/slam/(start|save|stop)', sub)
                if m:
                    b=self._body()
                    if m.group(2)=='save' and (map_registry is None or not map_registry.one('SELECT 1 FROM floors WHERE floor_id=?',(b.get('floor_id'),))):
                        raise ApiError(400,'invalid_floor','registered floor is required')
                    return self._send(202, fleet.slam_command(m.group(1),m.group(2),b.get('session_id'),p.name,b.get('floor_id')))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/mapping', sub)
                if m:
                    return self._send(202, fleet.request_operation(m.group(1), p.name, self._body()))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/bag', sub)
                if m:
                    return self._send(202, fleet.request_bag(m.group(1), p.name, self._body()))          # G2: origin/main Bag START/STOP
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/map-save', sub)
                if m:
                    return self._send(202, fleet.map_save_request(m.group(1), p.name, self._body()))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/recording/(start|stop)', sub)
                if m:
                    b = self._body(); return self._send(202, fleet.recording_command(m.group(1), m.group(2), b.get('session_id'), p.name))
                if sub == '/missions':
                    b = self._body(); view, created = fleet.create_mission(str(b.get('robot_id', '')), str(b.get('type', 'navigate_to_pose')), b.get('goal'), b.get('idempotency_key'), str(b.get('note', ''))[:200], p.name, b.get('lease_epoch'))
                    return self._send(201 if created else 200, dict(view, created=created))
                m = re.fullmatch(r'/robots/([A-Za-z0-9_.-]{1,64})/lock', sub)
                if m:
                    b = self._body(); return self._send(200, fleet.lease_action(m.group(1), p, str(b.get('action', '')), b.get('ttl_s'), b.get('note', '')))
                m = re.fullmatch(r'/(missions/(m_[a-f0-9]{12})|robots/([A-Za-z0-9_.-]{1,64}))/recover', sub)
                if m:
                    if p.role != 'admin': raise ApiError(403, 'forbidden', 'only an admin may recover a RECONCILIATION_REQUIRED state')
                    b = self._body()
                    return self._send(200, fleet.recover_mission(m.group(2), p, b.get('note')) if m.group(2) else fleet.recover_robot_block(m.group(3), p, b.get('note')))
                m = re.fullmatch(r'/missions/(m_[a-f0-9]{12})/cancel', sub)
                if m: self._body(); return self._send(202, fleet.cancel_mission(m.group(1), p.name))
            raise ApiError(404, 'not_found', 'unknown endpoint')
    return H


class Servers:
    """One ThreadingHTTPServer per bind address, plus the housekeeping thread."""
    def __init__(self, fleet, auth, addrs, port, video_origins=(), map_registry=None):
        for a in addrs: check_bind_address(a)
        H = make_handler(fleet, auth, video_origins, map_registry=map_registry); self.fleet, self.srvs, self.stop_evt = fleet, [], threading.Event()
        for a in addrs:
            s = ThreadingHTTPServer((a, port), H); s.daemon_threads = True; self.srvs.append(s)

    def serve_forever(self):
        ts = [threading.Thread(target=s.serve_forever, daemon=True) for s in self.srvs]
        for t in ts: t.start()
        try:
            while not self.stop_evt.is_set():
                self.fleet.housekeeping(); self.stop_evt.wait(1.0)
        finally:
            for s in self.srvs: s.shutdown(); s.server_close()
