"""Synthetic status sequence checks; no real Gate or authorization."""
import copy,json,unittest
import test_agent_safety_v1 as a
import test_safety_context_v1 as s
from limo_robot_agent.safety_protocol import GateObservationTracker

class Tests(unittest.TestCase):
 def test_float_invalid_on_agent_and_server(self):
  for v in (0.0,1.2,float('nan')):
   with self.subTest(value=v):
    robot=a._make_ready_robot();msg=a._make_gate_msg();raw=json.loads(msg.data);raw['fault_epoch_pending']=v;msg.data=json.dumps(raw);robot._set('gate',msg)
    self.assertFalse(robot.snapshot()['safety_gate']['movement_permitted'])
    t=s.Tests('test_old_agent_blocked');t.setUp()
    try:
     state=copy.deepcopy(t.state);state['safety_gate']['fault_epoch_pending']=v
     t.f.ingest_state('r',3,state,'boot1');self.assertIsNone(t.f.safety_context('r'))
    finally:t.tearDown()
 def test_pending_zero_cannot_bypass_other_conditions(self):
  for field,value in [('state','FAILSAFE_SENSOR_TIMEOUT'),('fault_latched',True),('cmd_blocked',True),('fault_epoch',None)]:
   with self.subTest(field=field):
    robot=a._make_ready_robot();msg=a._make_gate_msg();raw=json.loads(msg.data);raw.update(fault_epoch_pending=0);raw[field]=value;msg.data=json.dumps(raw);robot._set('gate',msg)
    self.assertFalse(robot.snapshot()['safety_gate']['movement_permitted'])
 def test_missing_pending_legacy_is_not_proof_and_real_mode_blocked(self):
  robot=a._make_ready_robot();self.assertNotIn('fault_epoch_pending',robot.snapshot()['safety_gate'])
  robot.mode='real';robot.simulated=False
  gate=robot.snapshot()['safety_gate'];self.assertFalse(gate['movement_permitted']);self.assertIn('GATE_ATTESTATION_UNAVAILABLE',gate['movement_block_reasons'])
  t=s.Tests('test_old_agent_blocked');t.setUp()
  try:
   self.assertNotIn('fault_epoch_pending',t.state['safety_gate'])
   self.assertIsNotNone(t.f.safety_context('r'))
   print('SAFETY_GAP: server synthetic admission alone does not establish missing pending state',flush=True)
  finally:t.tearDown()
 def test_normal_receive_age_expires(self):
  g=GateObservationTracker();g.on_message({'state':'NORMAL','boot_id':'g','fault_epoch':1,'fault_latched':False,'cmd_blocked':False,'fault_epoch_pending':0},now_mono=1)
  self.assertFalse(g.evaluate(now_mono=4)['rx_fresh'])
 def test_new_boot_invalidates_old_admission_even_if_old_boot_reappears(self):
  t=s.Tests('test_old_agent_blocked');t.setUp()
  try:
   mission,_=t.create();new=copy.deepcopy(t.state);new['safety_gate'].update(boot_id='new-gate',fault_epoch=2,fault_epoch_pending=0)
   self.assertEqual(t.f.ingest_state('r',3,new,'boot1')['commands'],[])
   self.assertEqual(t.f.mission(mission['mission_id'])['status'],'FAILED')
   old=copy.deepcopy(t.state);old['safety_gate'].update(boot_id='gate1',fault_epoch=1,fault_epoch_pending=0)
   self.assertEqual(t.f.ingest_state('r',4,old,'boot1')['commands'],[])
   self.assertEqual(t.f.mission(mission['mission_id'])['status'],'FAILED')
  finally:t.tearDown()
 def test_unstamped_delayed_normal_source_time_is_not_detectable(self):
  g=GateObservationTracker();g.on_message({'state':'NORMAL','boot_id':'g','fault_epoch':1,'fault_latched':False,'cmd_blocked':False,'fault_epoch_pending':0},now_mono=100)
  result=g.evaluate(now_mono=100,expect_source_clock=False)
  self.assertTrue(result['rx_fresh']);self.assertTrue(result['source_fresh'])
  print('SAFETY_GAP: receive freshness cannot establish unstamped source publication time',flush=True)
 def test_old_gate_boot_reappearing_stays_fenced(self):
  g=GateObservationTracker()
  base={'state':'NORMAL','fault_epoch':2,'fault_latched':False,'cmd_blocked':False,'fault_epoch_pending':0}
  for i,boot in enumerate(('old','new','old'),1):g.on_message(dict(base,boot_id=boot),now_mono=i)
  result=g.evaluate(now_mono=3)
  self.assertTrue(result['boot_changed']);self.assertEqual(result['boot_id'],'old');self.assertFalse(result['contract_met'])
if __name__=='__main__':unittest.main(verbosity=2)
