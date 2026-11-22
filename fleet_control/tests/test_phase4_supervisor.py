"""Phase 4 supervisor (no ROS): start order, bounded restarts -> FAILED, operator actions limited to the local config, server loss."""
import json, os, sys, tempfile, threading, time, unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agent' / 'limo_robot_agent'))
from limo_robot_agent.supervisor import Supervisor  # noqa: E402


class FakeServer:
    def __init__(self):
        self.reports, self.queue = [], []
        outer = self
        class H(BaseHTTPRequestHandler):
            def log_message(self, *a): pass
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                outer.reports.append(body); acts, outer.queue = outer.queue, []
                data = json.dumps({'server_boot_id': 'srv', 'actions': acts}).encode()
                self.send_response(200); self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)
        self.srv = ThreadingHTTPServer(('127.0.0.1', 0), H)
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.url = f'http://127.0.0.1:{self.srv.server_address[1]}'

    def last(self, name):
        return next(s for s in self.reports[-1]['services'] if s['name'] == name)


def wait(pred, t=15):
    end = time.monotonic() + t
    while time.monotonic() < end:
        if pred(): return True
        time.sleep(.1)
    return False


class SupervisorTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(); self.srv = FakeServer()
        tok = os.path.join(self.tmp, 'tok'); open(tok, 'w').write('t\n')
        self.cfg = {'robot_id': 'R', 'server_url': self.srv.url, 'token_file': tok, 'report_period_s': 0.3, 'services': [
            {'name': 'stack', 'cmd': ['sleep', '60'], 'ready': {'after_s': 0.3}},
            {'name': 'watchdog', 'cmd': ['sleep', '60'], 'after': ['stack'], 'restart_with': ['stack'], 'ready': {'after_s': 0.3}},
            {'name': 'flaky', 'cmd': 'exit 3', 'after': ['stack'], 'restart': {'max': 2, 'window_s': 60, 'backoff_s': 0.2}, 'required': False},
            {'name': 'agent', 'cmd': ['sleep', '60'], 'after': ['watchdog'], 'restart_with': ['stack'], 'ready': {'after_s': 0.3}, 'actions': ['restart']}]}

    def start(self, cfg=None):
        self.sup = Supervisor(cfg or self.cfg, self.tmp); self.th = threading.Thread(target=self.sup.run, daemon=True); self.th.start()

    def tearDown(self):
        if hasattr(self, 'sup'): self.sup.stop_evt.set(); self.th.join(20)
        self.srv.srv.shutdown()

    def test_order_ready_and_crash_loop_failed(self):
        self.start()
        self.assertTrue(wait(lambda: self.srv.reports and self.srv.last('agent')['state'] == 'READY'))
        st = {s.name: s for s in self.sup.services}
        self.assertLess(st['stack'].since, st['watchdog'].since)        # watchdog started only after stack READY
        self.assertTrue(wait(lambda: self.srv.last('flaky')['state'] == 'FAILED'))
        self.assertEqual(self.srv.last('flaky')['last_exit']['code'], 3)
        self.assertEqual(self.srv.last('flaky')['restarts'], 2)          # bounded, no endless restart loop

    def test_actions_limited_to_local_config(self):
        self.start(); wait(lambda: self.srv.reports and self.srv.last('agent')['state'] == 'READY')
        self.srv.queue = [{'action_id': 'a1', 'service': 'agent', 'action': 'stop'},                 # not in agent's allowlist
                          {'action_id': 'a2', 'service': 'nope', 'action': 'start'},
                          {'action_id': 'a3', 'service': 'stack', 'action': 'exec', 'cmd': ['rm', '-rf', '/']}]
        self.assertTrue(wait(lambda: any(r.get('action_results') for r in self.srv.reports)))
        res = next(r['action_results'] for r in self.srv.reports if r.get('action_results'))
        self.assertEqual([x['ok'] for x in res], [False, False, False])
        self.assertEqual(self.srv.last('agent')['state'], 'READY')

    def test_stop_retry_restart(self):
        self.start(); wait(lambda: self.srv.reports and self.srv.last('watchdog')['state'] == 'READY')
        pid0 = self.srv.last('watchdog')['pid']
        self.srv.queue = [{'action_id': 'b1', 'service': 'watchdog', 'action': 'stop'}]
        self.assertTrue(wait(lambda: self.srv.last('watchdog')['state'] == 'STOPPED'))
        self.srv.queue = [{'action_id': 'b2', 'service': 'watchdog', 'action': 'start'}]
        self.assertTrue(wait(lambda: self.srv.last('watchdog')['state'] == 'READY'))
        self.assertNotEqual(self.srv.last('watchdog')['pid'], pid0)
        wait(lambda: self.srv.last('flaky')['state'] == 'FAILED')
        self.srv.queue = [{'action_id': 'b3', 'service': 'flaky', 'action': 'retry'}]
        self.assertTrue(wait(lambda: self.srv.last('flaky')['state'] in ('RUNNING', 'BACKOFF', 'STARTING')))

    def test_server_down_keeps_services_and_reconnects(self):
        cfg = dict(self.cfg, server_url='http://127.0.0.1:9')            # nothing listens: agent-first boot
        self.start(cfg)
        self.assertTrue(wait(lambda: self.sup.by_name['agent'].state == 'READY'))
        self.assertFalse(self.sup.server_ok)
        self.sup.cfg['server_url'] = self.srv.url                          # server comes up later
        self.assertTrue(wait(lambda: self.sup.server_ok and self.srv.reports))

    def test_dependency_exit_restarts_dependents(self):
        self.start(); self.assertTrue(wait(lambda: self.srv.reports and self.srv.last('agent')['state'] == 'READY'))
        pids = {n: self.srv.last(n)['pid'] for n in ('stack', 'watchdog', 'agent')}
        os.killpg(pids['stack'], 9)                                     # the Gate stack dies
        self.assertTrue(wait(lambda: self.srv.last('watchdog')['state'] in ('WAITING', 'STARTING', 'RUNNING')))
        self.assertTrue(wait(lambda: all(self.srv.last(n)['state'] == 'READY' for n in ('stack', 'watchdog', 'agent')), 20))
        self.assertTrue(all(self.srv.last(n)['pid'] != pids[n] for n in ('stack', 'watchdog', 'agent')))   # all three are new processes
        self.assertLess(self.sup.by_name['stack'].since, self.sup.by_name['watchdog'].since)              # order kept on restart

    def test_watchdog_exit_does_not_restart_agent(self):
        self.start(); self.assertTrue(wait(lambda: self.srv.reports and self.srv.last('agent')['state'] == 'READY'))
        agent_pid, wd_pid = self.srv.last('agent')['pid'], self.srv.last('watchdog')['pid']
        os.killpg(wd_pid, 9)
        self.assertTrue(wait(lambda: self.srv.last('watchdog')['state'] == 'READY' and self.srv.last('watchdog')['pid'] != wd_pid))
        self.assertEqual(self.srv.last('agent')['pid'], agent_pid)                  # only restart_with cascades

    def test_bad_config_rejected(self):
        bad = dict(self.cfg, services=[{'name': 'a', 'cmd': ['true'], 'after': ['b']}, {'name': 'b', 'cmd': ['true']}])
        with self.assertRaises(ValueError): Supervisor(bad, self.tmp)
        with self.assertRaises(ValueError): Supervisor(dict(self.cfg, ros_probe={'topics': {'/cmd_vel': {'type': 'geometry_msgs/msg/Twist'}}}), self.tmp)


if __name__ == '__main__':
    unittest.main()
