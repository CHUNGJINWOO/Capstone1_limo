"""Manual motion remains fail-closed on missing safety or Nav2 evidence."""
import time
import unittest
from types import SimpleNamespace
from limo_robot_agent.manual_mapping_input import ManualMappingInput

ACTIONS = ('navigate_to_pose', 'navigate_through_poses', 'follow_path', 'spin', 'backup')
GID = bytes.fromhex('0102030405060708')

class GuardTests(unittest.TestCase):
    def setUp(self):
        self.gate = {'contract_version': 'gate-status/1.2-sim', 'trust': 'UNAUTHENTICATED_SIM_ONLY',
                     'contract_met': True, 'r7_invalid': False, 'safety_state': 'NORMAL',
                     'single_publisher_graph': True, 'rx_fresh': True, 'source_fresh': True,
                     'fault_latched': False, 'nav_evidence': {'scope_version': 1, 'observed': True,
                     'all_idle_proven': True, 'active_uuids': [], 'servers': {
                         a: {'verdict': 'IDLE_PROVEN_EL', 'writer_gid': GID.hex()} for a in ACTIONS}}}
        self.session = {'connected': True, 'reconciliation_required': False,
                        'lease': {'epoch': 1, 'expires_at': time.time() + 60}}
        robot = SimpleNamespace(mode='sim', simulated=True, allow_sim_safety_gate=True, active=None,
                                _use_sim_time=lambda: True, execution_allowed=lambda: True,
                                session_fn=lambda: self.session, _gate_evaluation=lambda: self.gate,
                                get_publishers_info_by_topic=lambda topic: [SimpleNamespace(endpoint_gid=GID)])
        self.adapter = ManualMappingInput.__new__(ManualMappingInput)
        self.adapter.robot = robot

    def test_valid_evidence_and_fail_closed_conditions(self):
        self.assertTrue(self.adapter._motion_permitted())
        self.gate['safety_state'] = 'LATCHED'
        self.assertFalse(self.adapter._motion_permitted())
        self.gate['safety_state'] = 'NORMAL'
        self.gate['nav_evidence']['servers']['spin']['verdict'] = 'ACTIVE_GOALS'
        self.assertFalse(self.adapter._motion_permitted())
        self.gate['nav_evidence']['servers']['spin']['verdict'] = 'IDLE_PROVEN_EL'
        self.gate['nav_evidence']['servers']['spin']['writer_gid'] = '0000000000000000'
        self.assertFalse(self.adapter._motion_permitted())
        self.gate['nav_evidence']['servers']['spin']['writer_gid'] = GID.hex()
        self.session['lease']['expires_at'] = time.time() - 1
        self.assertFalse(self.adapter._motion_permitted())

if __name__ == '__main__':
    unittest.main()
