"""dds_ready_check: READY only when every sample is healthy AND agent_seq advances; each fault must give NOT_READY (fake server, no ROS)."""
import contextlib, http.server, importlib.util, io, json, pathlib, tempfile, threading, unittest

spec = importlib.util.spec_from_file_location('rc', pathlib.Path(__file__).resolve().parents[1] / 'scripts/dds_ready_check.py')
rc = importlib.util.module_from_spec(spec); spec.loader.exec_module(rc)


def good(seq, **over):
    r = {'online': True, 'state_age_s': 0.4, 'state': {'lidar': {'status': 'OK', 'age_s': 0.05}, 'pose': {'status': 'OK'}, 'map': {'status': 'STATIC'}, 'agent_seq': seq}}
    for k, v in over.items():
        r['state'][k] = v
    return r


class Srv:
    def __init__(self, fn):
        self.n = 0
        outer = self

        class H(http.server.BaseHTTPRequestHandler):
            def do_GET(s):
                outer.n += 1
                body = json.dumps({'robot': fn(outer.n)}).encode()
                s.send_response(200); s.send_header('Content-Length', str(len(body))); s.end_headers(); s.wfile.write(body)

            def log_message(s, *a):
                pass
        self.httpd = http.server.HTTPServer(('127.0.0.1', 0), H)
        self.url = f'http://127.0.0.1:{self.httpd.server_address[1]}'
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self):
        self.httpd.shutdown()


class T(unittest.TestCase):
    def run_check(self, fn, samples=3):
        s = Srv(fn)
        tf = tempfile.NamedTemporaryFile('w', delete=False); tf.write('x' * 8); tf.close()
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                code = rc.main(['--url', s.url, '--robot', 'pcu', '--token-file', tf.name, '--samples', str(samples), '--interval', '0.01'])
        finally:
            s.close()
        return code, json.loads(buf.getvalue())

    def test_ready(self):
        code, r = self.run_check(lambda n: good(n))
        self.assertEqual((code, r['verdict']), (0, 'READY'))

    def test_lidar_no_data(self):
        self.assertEqual(self.run_check(lambda n: good(n, lidar={'status': 'NO_DATA', 'age_s': None}))[0], 5)

    def test_lidar_stale(self):
        self.assertEqual(self.run_check(lambda n: good(n, lidar={'status': 'OK', 'age_s': 4.0}))[0], 5)

    def test_pose_no_data(self):
        self.assertEqual(self.run_check(lambda n: good(n, pose={'status': 'NO_DATA'}))[0], 5)

    def test_map_no_map(self):
        self.assertEqual(self.run_check(lambda n: good(n, map={'status': 'NO_MAP'}))[0], 5)

    def test_agent_seq_frozen(self):
        self.assertEqual(self.run_check(lambda n: good(7))[0], 5)

    def test_offline_and_stale_state(self):
        self.assertEqual(self.run_check(lambda n: dict(good(n), online=False))[0], 5)
        self.assertEqual(self.run_check(lambda n: dict(good(n), state_age_s=9.0))[0], 5)

    def test_one_bad_sample_in_three_is_not_ready(self):
        self.assertEqual(self.run_check(lambda n: good(n, lidar={'status': 'NO_DATA', 'age_s': None}) if n == 2 else good(n))[0], 5)

    def test_unreachable_server(self):
        tf = tempfile.NamedTemporaryFile('w', delete=False); tf.write('x'); tf.close()
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = rc.main(['--url', 'http://127.0.0.1:9', '--robot', 'pcu', '--token-file', tf.name, '--samples', '2', '--interval', '0.01'])
        self.assertEqual(code, 5)

    def test_refuses_remote_url(self):
        self.assertEqual(rc.main(['--url', 'http://10.0.0.1:18190', '--robot', 'pcu', '--token-file', '/dev/null']), 2)


if __name__ == '__main__':
    unittest.main()
