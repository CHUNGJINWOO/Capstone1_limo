"""Unit tests of the shared E2E helpers (no network, no ROS)."""
import tempfile, time, unittest
from pathlib import Path
import e2e_common as ec


class Common(unittest.TestCase):
    def test_record_rejects_reserved_keys_and_counts(self):
        r = ec.Results(Path(tempfile.mkdtemp()) / 'log.txt')
        r.record('a', True, x=1); r.record('b', False); r.skip('c', 'no env')
        with self.assertRaises(ValueError): r.record('d', True, result={'status': 'x'})
        with self.assertRaises(ValueError): r.record('e', True, scenario='other')
        self.assertEqual(r.counts(), {'PASS': 1, 'FAIL': 1, 'SKIP': 1})

    def test_protected_ports_refused(self):
        for p in ec.PROTECTED_PORTS:
            with self.assertRaises(AssertionError): ec.Fleet(p, '/nonexistent')

    def test_wait_until_and_expect_http(self):
        t0 = time.monotonic(); self.assertIsNone(ec.wait_until(lambda: False, .2, .05)); self.assertGreaterEqual(time.monotonic() - t0, .2)
        self.assertEqual(ec.wait_until(lambda: 7, 1), 7)
        def boom(): raise ec.HttpError(409, '{"error": {"code": "mode_not_manual"}}')
        self.assertTrue(ec.expect_http(boom, 409, 'mode_not_manual')[0])
        self.assertEqual(ec.expect_http(lambda: None, 409), (False, 'accepted'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
