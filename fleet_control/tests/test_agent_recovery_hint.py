"""Agent journal: a server recovery hint resolves a local RECONCILIATION_REQUIRED entry only with this Agent's own
Nav2 status-array terminal for the same goal UUID (no ROS; snapshot_fn is a stub)."""
import sys, tempfile, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agent' / 'limo_robot_agent'))
from limo_robot_agent.core import AgentCore

UID = 'a' * 32


class Hint(unittest.TestCase):
    def setUp(self):
        self.observed = {}
        self.core = AgentCore('r', 'http://127.0.0.1:9', 't', lambda: {'execution': {'nav_terminal_observed': dict(self.observed)}},
                              None, {'boot_id': 'b1'}, tempfile.mkdtemp(), log=lambda m: None)
        self.core.seen['c1'] = {'status': 'RECONCILIATION_REQUIRED', 'goal_uuid': UID, 'type': 'navigate_to_pose'}
        self.core.seen['c2'] = {'status': 'RECONCILIATION_REQUIRED', 'type': 'navigate_to_pose'}       # no UUID recorded

    def test_hint_without_evidence_keeps_blocked(self):
        self.core._apply_recovery(['c1', 'c2'])
        self.assertEqual(self.core.seen['c1']['status'], 'RECONCILIATION_REQUIRED')
        self.observed = {'b' * 32: 'CANCELED'}                    # a different goal ended
        self.core._apply_recovery(['c1'])
        self.assertEqual(self.core.seen['c1']['status'], 'RECONCILIATION_REQUIRED')

    def test_matching_terminal_resolves_only_that_entry(self):
        self.observed = {UID: 'CANCELED'}
        self.core._apply_recovery(['c1', 'c2'])
        self.assertEqual((self.core.seen['c1']['status'], self.core.seen['c1']['terminal_source']), ('CANCELED', 'nav2_status_array'))
        self.assertEqual(self.core.seen['c2']['status'], 'RECONCILIATION_REQUIRED')

    def test_not_listed_by_server_untouched(self):
        self.observed = {UID: 'SUCCEEDED'}
        self.core._apply_recovery(['c9'])
        self.assertEqual(self.core.seen['c1']['status'], 'RECONCILIATION_REQUIRED')


if __name__ == '__main__':
    unittest.main(verbosity=1)
