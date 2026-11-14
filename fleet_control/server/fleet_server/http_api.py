"""HTTP layer (stdlib ThreadingHTTPServer). JSON API under /api/v1, static dashboard under /.
Every /api/v1 endpoint except /health needs `Authorization: Bearer <token>`; role checks per endpoint (see docs/API.md)."""
import ipaddress, json, mimetypes, os, re, threading, time, traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from . import API_VERSION, __version__
from .service import ApiError

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
MAX_BODY, MAX_MAP = 1 * 1024 * 1024, 6 * 1024 * 1024


def check_bind_address(addr: str):
    """Refuse wildcard / public addresses: only loopback, private (RFC1918) or Tailscale CGNAT (100.64.0.0/10) addresses may be bound."""
    ip = ipaddress.ip_address(addr)
    if ip.is_unspecified: raise SystemExit(f'refusing to bind the wildcard address {addr} (would expose the service on every interface)')
    ok = ip.is_loopback or ip.is_private or (ip.version == 4 and ip in ipaddress.ip_network('100.64.0.0/10'))
    if not ok: raise SystemExit(f'refusing to bind {addr}: not loopback / private / Tailscale (100.64.0.0/10)')


def make_handler(fleet, auth, video_origins=()):
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
            try:
                u = urlparse(self.path); path, qs = u.path, {k: v[0] for k, v in parse_qs(u.query).items()}
                if not path.startswith('/api/'): return self._static(path) if method == 'GET' else self._err(405, 'method_not_allowed', 'method not allowed')
                self._api(method, path, qs)
            except ApiError as e: self._err(e.status, e.code, e.message)
            except Exception:
                traceback.print_exc(); self._err(500, 'internal', 'internal server error')

        def _static(self, path):
            if path in ('', '/'): path = '/index.html'
            fp = os.path.normpath(os.path.join(STATIC, path.lstrip('/')))
            if not fp.startswith(STATIC + os.sep) or not os.path.isfile(fp): return self._err(404, 'not_found', 'not found')
            with open(fp, 'rb') as f: data = f.read()
            self._send(200, data, mimetypes.guess_type(fp)[0] or 'application/octet-stream')

        def _api(self, method, path, qs):
            pre = f'/api/{API_VERSION}'
            if not path.startswith(pre + '/') and path != pre: raise ApiError(404, 'not_found', f'use {pre}')
            sub = path[len(pre):].rstrip('/') or '/'
            if sub == '/health' and method == 'GET': return self._send(200, {'status': 'ok', 'api': API_VERSION})
            p = self._principal()
            # ---------------- agent endpoints (token must belong to the robot named in the body)
            if sub.startswith('/agent/'):
                if method != 'POST': raise ApiError(405, 'method_not_allowed', 'POST only')
                limit = MAX_MAP if sub == '/agent/map' else MAX_BODY
                b = self._body(limit); rid = b.get('robot_id')
                if not isinstance(rid, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,64}', rid): raise ApiError(400, 'bad_robot_id', 'robot_id must match [A-Za-z0-9_.-]{1,64}')
                if not p.can_act_as_agent(rid): raise ApiError(403, 'forbidden', 'this token may not act as that robot')
                if sub == '/agent/hello': return self._send(200, fleet.hello(rid, b.get('agent') or {}))
                if sub == '/agent/reconcile': return self._send(200, fleet.reconcile(rid, b.get('agent_boot_id'), b.get('journal', {}), b.get('execution', {}), b.get('journal_ok') is True))
                if sub == '/agent/state': return self._send(200, fleet.ingest_state(rid, int(b.get('seq', 0)), b.get('state') or {}, b.get('agent_boot_id')))
                if sub == '/agent/map':
                    fleet.check_session(rid, b.get('agent_boot_id'))
                    return self._send(200, fleet.ingest_map(rid, int(b['version']), b.get('meta') or {}, b['png_base64']))
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
                if sub == '/missions': return self._send(200, {'missions': fleet.missions(qs.get('robot_id'), qs.get('status'), min(int(qs.get('limit', 50)), 500))})
                m = re.fullmatch(r'/missions/(m_[a-f0-9]{12})', sub)
                if m: return self._send(200, fleet.mission(m.group(1)))
                if sub == '/events': return self._send(200, {'events': fleet.events(qs.get('since_id', 0), min(int(qs.get('limit', 100)), 500))})
                raise ApiError(404, 'not_found', 'unknown endpoint')
            # ---------------- control endpoints
            if method == 'POST':
                if not p.can_control(): raise ApiError(403, 'forbidden', 'role may not create or cancel missions')
                if sub == '/missions':
                    b = self._body(); view, created = fleet.create_mission(str(b.get('robot_id', '')), str(b.get('type', 'navigate_to_pose')), b.get('goal'), b.get('idempotency_key'), str(b.get('note', ''))[:200], p.name)
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
    def __init__(self, fleet, auth, addrs, port, video_origins=()):
        for a in addrs: check_bind_address(a)
        H = make_handler(fleet, auth, video_origins); self.fleet, self.srvs, self.stop_evt = fleet, [], threading.Event()
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
