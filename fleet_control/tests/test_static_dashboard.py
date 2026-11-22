"""Loopback HTTP tests verifying static dashboard assets and UI-used API flows."""
import base64
import json
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'server'))
from fleet_server.auth import Auth, init_tokens
from fleet_server.db import DB
from fleet_server.http_api import Servers
from fleet_server.service import Fleet


class StaticDashboardTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        p = Path(self.tmp.name)
        init_tokens(str(p), str(p / 'sec'), ['pcu'])
        self.agent = (p / 'sec/agent-pcu.token').read_text().strip()
        self.viewer = (p / 'sec/viewer.token').read_text().strip()
        self.op = (p / 'sec/operator.token').read_text().strip()
        self.admin = (p / 'sec/admin.token').read_text().strip()
        self.db = DB(str(p / 'db'))
        self.f = Fleet(self.db, SimpleNamespace(offline_after_s=10, command_ttl_s=120, resend_after_s=0.1))
        self.s = Servers(self.f, Auth(str(p)), ['127.0.0.1'], 0)
        self.port = self.s.srvs[0].server_address[1]
        self.t = threading.Thread(target=self.s.serve_forever, daemon=True)
        self.t.start()

    def tearDown(self):
        self.s.stop_evt.set()
        self.t.join(3)
        self.db.c.close()
        self.tmp.cleanup()

    def raw_get(self, path):
        with urllib.request.urlopen(f'http://127.0.0.1:{self.port}{path}', timeout=3) as r:
            return r.status, dict(r.headers), r.read()

    def api(self, method, path, token, body=None):
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(
            f'http://127.0.0.1:{self.port}/api/v1{path}',
            method=method,
            data=data,
            headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'},
        )
        try:
            with urllib.request.urlopen(req, timeout=3) as r:
                return r.status, json.load(r)
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def test_static_assets_and_required_ids(self):
        status, headers, html = self.raw_get('/')
        self.assertEqual(status, 200)
        self.assertIn(b'Capstone1', html)
        self.assertIn("default-src 'self'", headers.get('Content-Security-Policy', ''))
        # Preserve video_panel.js DOM IDs
        for dom_id in (b'id="aiPanel"', b'id="aiForm"', b'id="aiBase"', b'id="aiTok"', b'id="aiImg"', b'id="aiStats"', b'id="aiDets"'):
            self.assertIn(dom_id, html)
        for asset in ('/style.css', '/state_helpers.js', '/map_view.js', '/app.js', '/video_panel.js'):
            st, _, content = self.raw_get(asset)
            self.assertEqual(st, 200, asset)
            self.assertGreater(len(content), 50, asset)

    def test_dashboard_api_lease_mission_map_and_recover_flow(self):
        self.assertEqual(
            self.api('POST', '/agent/hello', self.agent, {
                'robot_id': 'pcu',
                'agent': {'boot_id': 'b1', 'protocol_version': 2, 'safety_contract_version': 1, 'mode': 'sim', 'simulated': True, 'missions_enabled': True},
            })[0],
            200,
        )
        ex = {'active': None, 'nav2_goals': {'known': True, 'active_count': 0, 'received_status_array': True, 'age_s': 0.0}}
        self.assertEqual(
            self.api('POST', '/agent/reconcile', self.agent, {
                'robot_id': 'pcu', 'agent_boot_id': 'b1', 'journal': {}, 'journal_ok': True, 'execution': ex,
            })[0],
            200,
        )
        st = {
            'safety_gate': {'boot_id': 'g1', 'status': 'NORMAL', 'age_s': 0.0, 'fault_epoch': 1, 'fault_latched': False, 'cmd_blocked': False, 'movement_permitted': True, 'motor_path': {'status': 'SIM_GATED', 'cmd_vel_publishers': ['stall_detector_node']}},
            'operation_mode': 'NAVIGATION',
            'operation_revision': 1,
            'map_epoch': 1,
            'localization_epoch': 1,
            'map_version_id': 'map-v1',
            'localization_map_version_id': 'map-v1',
            'map_verified': True,
            'localization_status': 'CONFIRMED',
            'execution': ex,
        }
        self.assertEqual(self.api('POST', '/agent/state', self.agent, {'robot_id': 'pcu', 'agent_boot_id': 'b1', 'seq': 1, 'state': st})[0], 200)
        tiny_png = base64.b64encode(b'\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR').decode()
        self.assertEqual(
            self.api('POST', '/agent/map', self.agent, {
                'robot_id': 'pcu', 'agent_boot_id': 'b1', 'version': 1,
                'meta': {'width': 10, 'height': 10, 'resolution': 0.05, 'origin_x': 0.0, 'origin_y': 0.0, 'origin_yaw': 0.2, 'frame_id': 'map'},
                'png_base64': tiny_png,
            })[0],
            200,
        )

        # Viewer reads dashboard & map
        code, me_v = self.api('GET', '/me', self.viewer)
        self.assertEqual(code, 200)
        self.assertEqual(me_v['role'], 'viewer')
        code, dash = self.api('GET', '/dashboard', self.viewer)
        self.assertEqual(code, 200)
        self.assertEqual(dash['robots'][0]['robot_id'], 'pcu')
        self.assertEqual(dash['robots'][0]['agent']['missions_enabled'], True)
        self.assertEqual(self.api('POST', '/robots/pcu/lock', self.viewer, {'action': 'acquire'})[0], 403)

        # Operator acquires lease, creates mission with lease_epoch, and cancels
        code, lease = self.api('POST', '/robots/pcu/lock', self.op, {'action': 'acquire', 'ttl_s': 30, 'note': 'ui test'})
        self.assertEqual(code, 200)
        code, m = self.api('POST', '/missions', self.op, {
            'robot_id': 'pcu',
            'type': 'navigate_to_pose',
            'goal': {'frame_id': 'map', 'x': 1.2, 'y': 0.4, 'yaw_rad': 0.0},
            'lease_epoch': lease['epoch'],
            'idempotency_key': 'ui-test-1',
        })
        self.assertEqual(code, 201)
        self.assertEqual(self.api('POST', f"/missions/{m['mission_id']}/cancel", self.op, {})[0], 202)
        self.assertEqual(self.api('POST', '/robots/pcu/lock', self.op, {'action': 'release'})[0], 200)
        self.assertEqual(self.api('GET', '/missions?limit=200', self.viewer)[0], 200)


if __name__ == '__main__':
    unittest.main(verbosity=2)
