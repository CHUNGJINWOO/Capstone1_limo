"""No ROS, no motors: verify the opt-in exception cannot change real/default behavior."""
import sys,unittest
from pathlib import Path
from types import SimpleNamespace as S
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'agent/limo_robot_agent'))
from limo_robot_agent.motor_policy import motor_path_allowed as allowed
G=S(node_name='stall_detector_node',node_namespace='/')
class Tests(unittest.TestCase):
 def test_real_default_rejects_gate(self):self.assertFalse(allowed([G]))
 def test_real_explicit_flags_still_reject(self):self.assertFalse(allowed([G],mode='real',simulated=True,allow_sim_safety_gate=True,use_sim_time=True))
 def test_real_locked_behavior_unchanged(self):self.assertTrue(allowed([]))
 def test_unknown_graph_rejected(self):self.assertFalse(allowed(None,**self.options()))
 def test_explicit_sim_allowed(self):self.assertTrue(allowed([G],**self.options()))
 def test_no_optin_rejected(self):self.assertFalse(allowed([G],mode='sim',simulated=True,use_sim_time=True))
 def test_wrong_mode_rejected(self):self.assertFalse(allowed([G],mode='mock',simulated=True,allow_sim_safety_gate=True,use_sim_time=True))
 def test_no_simulated_flag_rejected(self):self.assertFalse(allowed([G],mode='sim',allow_sim_safety_gate=True,use_sim_time=True))
 def test_wall_clock_rejected(self):self.assertFalse(allowed([G],mode='sim',simulated=True,allow_sim_safety_gate=True))
 def test_extra_publisher_rejected(self):self.assertFalse(allowed([G,S(node_name='other',node_namespace='/')],**self.options()))
 def test_duplicate_gate_rejected(self):self.assertFalse(allowed([G,G],**self.options()))
 def test_wrong_namespace_rejected(self):self.assertFalse(allowed([S(node_name=G.node_name,node_namespace='/other')],**self.options()))
 @staticmethod
 def options():return dict(mode='sim',simulated=True,allow_sim_safety_gate=True,use_sim_time=True)
if __name__=='__main__':unittest.main(verbosity=2)
