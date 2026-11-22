"""Phase 5 visitor guide service (no ROS): destinations bound to verified map versions, availability, requests -> missions,
visitor token isolation at the HTTP boundary."""
import json, shutil, sys, tempfile, threading, time, unittest, urllib.error, urllib.request
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'server'))
from fleet_server.db import DB  # noqa: E402
from fleet_server.map_registry import MapRegistry  # noqa: E402
from fleet_server.service import ApiError, Fleet  # noqa: E402
from fleet_server.auth import Principal  # noqa: E402


def pgm(w=80, h=80):
    px = bytearray(254 for _ in range(w * h))
    for i in range(w):
        px[i] = 0; px[(h - 1) * w + i] = 0
    for i in range(h):
        px[i * w] = 0; px[i * w + w - 1] = 0
    for r in range(10, 20):                    # occupied block (top rows = +y)
        for c in range(50, 60): px[r * w + c] = 0
    for r in range(60, 70):                    # unknown block
        for c in range(10, 20): px[r * w + c] = 205
    return f'P5\n{w} {h}\n255\n'.encode() + bytes(px)


YAML = b'image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: [-2.0, -2.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
OPER = Principal('operator', 'operator'); ADMIN = Principal('admin', 'admin'); VIEW = Principal('viewer', 'viewer')
CFG = SimpleNamespace(offline_after_s=6, command_ttl_s=120, resend_after_s=1, require_operating_mode=True, require_supervisor_ready=False)


class Guide(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.reg = MapRegistry(f'{self.tmp}/map_registry.db', f'{self.tmp}/map_store')
        self.reg.create_facility('fac', 'Campus', actor='t'); self.reg.create_building('bld', 'fac', 'H', actor='t')
        self.reg.create_floor('fl1', 'bld', '1F', actor='t'); self.reg.create_area('ar1', 'fl1', 'Lobby', actor='t')
        self.mv = self.reg.register_map_bundle(floor_id='fl1', label='verified', yaml_bytes=YAML, pgm_bytes=pgm(), actor='t', actor_role='operator')['map_version']
        self.reg.verify_map_version(self.mv['map_version_id'], status='VERIFIED', evidence={'method': 'unit'}, actor='t', actor_role='admin', reason='unit')
        self.unv = self.reg.register_map_bundle(floor_id='fl1', label='unverified', yaml_bytes=YAML, pgm_bytes=pgm(w=81), actor='t', actor_role='operator')['map_version']
        self.db = DB(':memory:'); self.f = Fleet(self.db, CFG); self.f.map_registry = self.reg
        self.f.robots['R'] = {'robot_id': 'R', 'online': True, 'reconciled': True, 'state_ts': time.time(), 'last_seen': time.time(),
                              'agent': {'boot_id': 'a1', 'protocol_version': 2},
                              'state': {'execution': {'active': None, 'nav2_goals': {'active_count': 0}}}}
        self.f.safety_context = lambda rid: {'ok': True}           # unit level: the safety context is covered by its own tests / E2E
        self.f.dispatch_next = lambda rid: None                    # delivery to the Agent is the existing path (isolated E2E)
        self.epoch = self.f.lease_action('R', OPER, 'acquire', 300)['epoch']
        self.navigate(self.mv)

    def tearDown(self):
        self.reg.close(); shutil.rmtree(self.tmp, ignore_errors=True)

    def navigate(self, mv, localized=True):
        self.f._mode_save('R', dict(self.f._mode_new('NAVIGATION', None, 't'), transition=None, epoch=self.epoch, owner='operator', agent_boot='a1'))
        rec = {'request_id': 'nq1', 'map_version_id': mv['map_version_id'], 'pgm_sha256': mv['sha256']['pgm_sha256'] if 'sha256' in mv else mv['pgm_sha256'],
               'initial_pose': {'x': 0, 'y': 0, 'yaw': 0, 'seq': 1}}
        self.f._nav_save('R', rec)
        self.f.robots['R']['state']['localization'] = {'status': 'CONFIRMED' if localized else 'CONVERGING', 'request_id': 'nq1',
                                                       'map_version_id': mv['map_version_id'], 'initial_pose_seq': 1}

    def dest(self, mv=None, x=0.0, y=0.0, **kw):
        return self.f.upsert_destination(OPER, dict({'name': 'Info desk', 'map_version_id': (mv or self.mv)['map_version_id'], 'x': x, 'y': y, 'area_id': 'ar1'}, **kw))

    def visitor(self, session='s' * 20):
        link = self.f.create_visitor_link(ADMIN, {'label': 'qr', 'ttl_s': 3600})
        return self.f.visitor_auth('Visitor ' + link['token'], session), link

    def test_destination_uses_server_goal_check_and_roles(self):
        d = self.dest(); self.assertEqual(d['map_version_id'], self.mv['map_version_id'])
        for x, y, why in ((0.75, 1.25, 'OCCUPIED'), (-1.25, -1.25, 'UNKNOWN_AREA'), (5.0, 0.0, 'OUTSIDE_MAP'), (-1.85, 0.0, 'FOOTPRINT')):
            with self.assertRaises(ApiError) as e: self.dest(x=x, y=y)
            self.assertIn(why, e.exception.message)
        with self.assertRaises(ApiError) as e: self.f.upsert_destination(VIEW, {'name': 'x', 'map_version_id': self.mv['map_version_id'], 'x': 0, 'y': 0})
        self.assertEqual(e.exception.status, 403)

    def test_availability_reasons(self):
        d = self.dest()
        self.assertEqual(self.f.destinations()[0]['reason'], 'SERVICE_CLOSED')
        with self.assertRaises(ApiError) as e: self.f.set_guide_service(ADMIN, {'robot_id': 'R', 'enabled': True})
        self.assertEqual(e.exception.code, 'lease_required')                # must hold the lease to open the service
        self.f.set_guide_service(OPER, {'robot_id': 'R', 'enabled': True})
        self.assertTrue(self.f.destinations()[0]['available'])
        self.navigate(self.mv, localized=False); self.assertEqual(self.f.destinations()[0]['reason'], 'ROBOT_NOT_LOCALIZED')
        self.navigate(self.unv); self.assertEqual(self.f.destinations()[0]['reason'], 'ROBOT_ON_OTHER_MAP')
        self.navigate(self.mv)
        self.db.x('UPDATE guide_destinations SET map_content_sha256=? WHERE destination_id=?', ('0' * 64, d['destination_id']))
        self.assertEqual(self.f.destinations()[0]['reason'], 'MAP_CHANGED')
        self.dest(destination_id=d['destination_id'])                       # operator re-confirms on the current version
        self.f.lease_action('R', OPER, 'release', None); self.f.lease_action('R', ADMIN, 'acquire', 300)
        self.assertEqual(self.f.destinations()[0]['reason'], 'OPERATOR_LEASE_CHANGED')

    def test_unverified_map_never_guided(self):
        d = self.dest(mv=self.unv, name='C103 style')
        self.f.set_guide_service(OPER, {'robot_id': 'R', 'enabled': True}); self.navigate(self.unv)
        v, _ = self.visitor()
        r = self.f.create_guide_request(v, {'destination_id': d['destination_id']})
        self.assertEqual((r['status'], r['reason']), ('UNAVAILABLE', 'MAP_NOT_VERIFIED'))
        self.assertEqual(self.db.q('SELECT * FROM missions'), [])

    def test_request_lifecycle_queue_and_cancel(self):
        d = self.dest(); self.f.set_guide_service(OPER, {'robot_id': 'R', 'enabled': True})
        v1, link = self.visitor('a' * 20); v2 = self.f.visitor_auth('Visitor ' + link['token'], 'b' * 20); v3 = self.f.visitor_auth('Visitor ' + link['token'], 'c' * 20)
        r1 = self.f.create_guide_request(v1, {'destination_id': d['destination_id']})
        self.assertEqual(r1['status'], 'ACCEPTED')
        m = self.db.one('SELECT * FROM missions'); self.assertEqual(m['created_by'], 'operator'); self.assertIn('visitor guide', m['note'])
        with self.assertRaises(ApiError) as e: self.f.create_guide_request(v1, {'destination_id': d['destination_id']})
        self.assertEqual(e.exception.code, 'request_active')
        r2 = self.f.create_guide_request(v2, {'destination_id': d['destination_id']}); self.assertEqual((r2['status'], r2['queue_position']), ('QUEUED', 1))
        r3 = self.f.create_guide_request(v3, {'destination_id': d['destination_id']})
        with self.assertRaises(ApiError): self.f.guide_request_view(r1['request_id'], v2)      # other sessions cannot see / cancel
        with self.assertRaises(ApiError): self.f.cancel_guide_request(v2, r1['request_id'])
        self.assertEqual(self.f.cancel_guide_request(v3, r3['request_id'])['status'], 'CANCELED')
        self.db.x("UPDATE missions SET status='EXECUTING' WHERE mission_id=?", (m['mission_id'],)); self.f.guide_tick()
        self.assertEqual(self.f.guide_request_view(r1['request_id'], v1)['status'], 'IN_PROGRESS')
        self.db.x("UPDATE missions SET status='SUCCEEDED' WHERE mission_id=?", (m['mission_id'],)); self.f.guide_tick()
        self.assertEqual(self.f.guide_request_view(r1['request_id'], v1)['status'], 'COMPLETED')
        self.assertEqual(self.f.guide_request_view(r2['request_id'], v2)['status'], 'ACCEPTED')  # queued one dispatched (FIFO)
        m2 = self.f.guide_request_view(r2['request_id'])['mission_id']
        self.db.x("UPDATE missions SET status='EXECUTING' WHERE mission_id=?", (m2,)); self.f.guide_tick()
        self.assertEqual(self.f.cancel_guide_request(v2, r2['request_id'])['status'], 'CANCELING')
        self.assertTrue(self.db.one('SELECT cancel_requested FROM missions WHERE mission_id=?', (m2,))['cancel_requested'])
        self.db.x("UPDATE missions SET status='CANCELED' WHERE mission_id=?", (m2,)); self.f.guide_tick()
        self.assertEqual(self.f.guide_request_view(r2['request_id'], v2)['status'], 'CANCELED')

    def test_mission_failure_and_service_close(self):
        d = self.dest(); self.f.set_guide_service(OPER, {'robot_id': 'R', 'enabled': True})
        v, link = self.visitor(); r = self.f.create_guide_request(v, {'destination_id': d['destination_id']})
        mid = self.f.guide_request_view(r['request_id'])['mission_id']
        self.db.x("UPDATE missions SET status='FAILED' WHERE mission_id=?", (mid,)); self.f.guide_tick()
        g = self.f.guide_request_view(r['request_id'], v); self.assertEqual((g['status'], g['reason']), ('FAILED', 'MISSION_FAILED'))
        self.f.set_guide_service(OPER, {'robot_id': 'R', 'enabled': False})
        r2 = self.f.create_guide_request(v, {'destination_id': d['destination_id']})
        self.assertEqual((r2['status'], r2['reason']), ('UNAVAILABLE', 'SERVICE_CLOSED'))

    def test_visitor_links(self):
        v, link = self.visitor()
        with self.assertRaises(ApiError) as e: self.f.visitor_auth('Bearer ' + link['token'], 's' * 20)
        self.assertEqual(e.exception.status, 401)
        with self.assertRaises(ApiError) as e: self.f.visitor_auth('Visitor ' + link['token'], 'x')
        self.assertEqual(e.exception.status, 400)
        self.f.revoke_visitor_link(ADMIN, link['link_id'])
        with self.assertRaises(ApiError) as e: self.f.visitor_auth('Visitor ' + link['token'], 's' * 20)
        self.assertEqual(e.exception.status, 401)
        with self.assertRaises(ApiError) as e: self.f.create_visitor_link(OPER, {})
        self.assertEqual(e.exception.status, 403)


class VisitorHttpIsolation(unittest.TestCase):
    """The visitor token reaches /visitor/* only; Bearer endpoints refuse it, /visitor/* refuses Bearer tokens."""
    def setUp(self):
        from fleet_server.auth import Auth, init_tokens
        from fleet_server.http_api import Servers
        self.tmp = tempfile.mkdtemp(); init_tokens(self.tmp, self.tmp + '/s', ['R'])
        self.fleet = Fleet(DB(self.tmp + '/fleet.db'), CFG)
        self.auth = Auth(self.tmp)
        self.srv = Servers(self.fleet, self.auth, ['127.0.0.1'], 0)
        self.port = self.srv.srvs[0].server_address[1]
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.vt = self.fleet.create_visitor_link(ADMIN, {'ttl_s': 600})['token']
        self.op = open(self.tmp + '/s/operator.token').read().strip()

    def tearDown(self):
        self.srv.stop_evt.set(); time.sleep(1.2); shutil.rmtree(self.tmp, ignore_errors=True)

    def call(self, method, path, auth, body=None):
        self.auth.fail.clear()                   # the per-IP lockout (10 failures/min) is real; probes here are deliberate
        req = urllib.request.Request(f'http://127.0.0.1:{self.port}/api/v1{path}', method=method, data=None if body is None else json.dumps(body).encode(),
                                     headers={'Authorization': auth, 'X-Visitor-Session': 'v' * 20, 'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=5) as r: return r.status
        except urllib.error.HTTPError as e: return e.code

    def test_isolation(self):
        V = 'Visitor ' + self.vt
        self.assertEqual(self.call('GET', '/visitor/catalog', V), 200)
        self.assertEqual(self.call('GET', '/visitor/destinations', V), 200)
        for m, p, b in (('GET', '/robots', None), ('GET', '/dashboard', None), ('GET', '/missions', None), ('GET', '/events', None),
                        ('POST', '/robots/R/manual-bench', {'action': 'start'}), ('POST', '/robots/R/gate-reset', {}),
                        ('POST', '/robots/R/mapping', {'action': 'START'}), ('POST', '/robots/R/mode', {'target': 'MANUAL'}),
                        ('POST', '/missions', {'robot_id': 'R'}), ('POST', '/robots/R/services', {'service': 'agent', 'action': 'stop'}),
                        ('GET', '/visitor-links', None), ('POST', '/guide/destinations', {})):
            self.assertEqual(self.call(m, p, V), 401, p)
            self.assertEqual(self.call(m, p, 'Bearer ' + self.vt), 401, p)            # the visitor token is not a Bearer token either
        self.assertEqual(self.call('GET', '/visitor/catalog', 'Bearer ' + self.op), 401)  # operator token is not a visitor link


if __name__ == '__main__':
    unittest.main()
