"""Agent graceful stop (2026-11-22): runs a 4-case subset of agent_shutdown_matrix_20261122.py against the current Agent source.
Laptop only (isolated domain 228, loopback port 18296, no robot, no motor). The full 12-case matrix is the script itself.
Regression for: a SIGINT inherited as SIG_IGN, or SIGTERM, left the real-mode Agent in an endless RCLError retry loop (needs SIGKILL);
a normal SIGINT left through a traceback in announce_shutdown().
"""
import json, subprocess, sys, tempfile, unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
CASES = 'up_INT_ign,up_TERM_norm,down_start_TERM_norm,reconnect_INT_norm'


class TestAgentGracefulStop(unittest.TestCase):
    def test_subset_of_matrix_passes(self):
        out = Path(tempfile.mkdtemp()) / 'm'
        r = subprocess.run([sys.executable, str(HERE / 'agent_shutdown_matrix_20261122.py'), '--agent-src', str(HERE.parent / 'agent' / 'limo_robot_agent'),
                            '--out', str(out), '--domain', '228', '--port', '18296', '--only', CASES], capture_output=True, text=True, timeout=600)
        res = json.loads((out / 'result.json').read_text())['results']
        self.assertEqual(len(res), 4, r.stdout[-500:])
        for c in res:
            self.assertEqual(c['result'], 'PASS', json.dumps(c)[:400])


if __name__ == '__main__':
    unittest.main(verbosity=2)
