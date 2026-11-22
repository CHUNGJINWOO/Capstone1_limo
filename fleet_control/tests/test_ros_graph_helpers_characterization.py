"""Characterization (Phase 3.5): GID helper semantics of each caller, pinned before they share one implementation (R2)."""
import sys, types, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agent' / 'limo_robot_agent'))
from limo_robot_agent import action_status_evidence as ase, gate_reset_worker as grw  # noqa: E402

G = lambda b: types.SimpleNamespace(endpoint_gid=b)
NZ = [1] + [0] * 23


class GidHelpers(unittest.TestCase):
    def test_action_status_evidence_gid_hex_keeps_zero_gid(self):
        self.assertEqual(ase.gid_hex(G(NZ)), '01' + '00' * 23)
        self.assertEqual(ase.gid_hex(G([0] * 24)), '00' * 24)          # all-zero is returned; the caller rejects it
        self.assertIsNone(ase.gid_hex(types.SimpleNamespace()))        # unreadable -> None
        self.assertIsNone(ase.gid_hex(G(None)))

    def test_gate_reset_worker_gid_rejects_zero_gid(self):
        self.assertEqual(grw._gid(G(NZ)), '01' + '00' * 23)
        self.assertIsNone(grw._gid(G([0] * 24)))
        self.assertIsNone(grw._gid(types.SimpleNamespace()))
        self.assertIsNone(grw._gid(G([])))

    def test_participant_prefix(self):
        self.assertEqual(grw.participant_prefix('ab' * 16), 'ab' * 12)
        self.assertIsNone(grw.participant_prefix('ab'))
        self.assertIsNone(grw.participant_prefix(None))


if __name__ == '__main__':
    unittest.main(verbosity=2)
