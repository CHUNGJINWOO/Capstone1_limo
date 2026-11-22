"""Current conservative Gate epoch behavior; r5 proposal is not implemented."""
import copy,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'agent/limo_robot_agent'))
from limo_robot_agent.safety_protocol import GateObservationTracker
import test_safety_context_v1 as server_fixture
from fleet_server.service import ApiError

class Tests(unittest.TestCase):
 def test_epoch_regression_within_boot_is_blocked(self):
  g=GateObservationTracker()
  g.on_message({'state':'NORMAL','boot_id':'a','fault_epoch':3,'fault_latched':False,'cmd_blocked':False},now_mono=1)
  g.on_message({'state':'NORMAL','boot_id':'a','fault_epoch':2,'fault_latched':False,'cmd_blocked':False},now_mono=2)
  self.assertTrue(g.evaluate(now_mono=2)['epoch_regressed'])
 def test_new_gate_boot_with_lower_epoch_currently_stays_blocked(self):
  g=GateObservationTracker()
  g.on_message({'state':'NORMAL','boot_id':'a','fault_epoch':3,'fault_latched':False,'cmd_blocked':False},now_mono=1)
  g.on_message({'state':'NORMAL','boot_id':'b','fault_epoch':0,'fault_latched':False,'cmd_blocked':False},now_mono=2)
  result=g.evaluate(now_mono=2)
  self.assertTrue(result['boot_changed']);self.assertTrue(result['epoch_regressed']);self.assertFalse(result['contract_met'])
 def test_server_new_gate_boot_lower_epoch_currently_stays_blocked(self):
  t=server_fixture.Tests('test_old_agent_blocked');t.setUp()
  try:
   old=copy.deepcopy(t.state);old['safety_gate']['fault_epoch']=3
   t.f.ingest_state('r',3,old,'boot1')
   new=copy.deepcopy(old);new['safety_gate'].update(boot_id='new-gate',fault_epoch=0)
   t.f.ingest_state('r',4,new,'boot1')
   self.assertTrue(t.f.db.one('SELECT 1 FROM meta WHERE k=?',('safety_epoch_invalid:r',)))
   with self.assertRaises(ApiError):t.create()
  finally:t.tearDown()

if __name__=='__main__':unittest.main(verbosity=2)
