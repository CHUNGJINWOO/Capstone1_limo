"""Fail-closed Gate boot transition tests; synthetic data only."""
import copy,sys,unittest,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'agent/limo_robot_agent'))
import test_safety_context_v1 as server
from fleet_server.service import ApiError
from limo_robot_agent.safety_protocol import GateObservationTracker,compute_movement_permitted
import test_agent_safety_v1 as agent

class Tests(unittest.TestCase):
 def test_agent_boot_transition_and_replay_latch(self):
  g=GateObservationTracker();base={'state':'NORMAL','fault_epoch':1,'fault_latched':False,'cmd_blocked':False,'fault_epoch_pending':0}
  g.on_message(dict(base,boot_id='A'),now_mono=1)
  self.assertTrue(g.evaluate(now_mono=1)['contract_met'])
  for t,boot in [(2,'B'),(3,'A')]:
   g.on_message(dict(base,boot_id=boot),now_mono=t);state=g.evaluate(now_mono=t)
   self.assertFalse(state['contract_met'])
   ok,reasons=compute_movement_permitted(allow_missions=True,gate_eval=state,motor_path_ok=True,op_ctx={},map_ctx={},loc_ctx={},nav2_goals={},session_ctx={})
   self.assertFalse(ok);self.assertIn('GATE_BOOT_RECONCILIATION_REQUIRED',reasons)
 def test_server_transition_replay_and_restart_are_fenced(self):
  t=server.Tests('test_old_agent_blocked');t.setUp()
  try:
   mission,_=t.create();st=copy.deepcopy(t.state);st['safety_gate']['fault_epoch_pending']=0
   st['safety_gate']['boot_id']='B';self.assertEqual(t.f.ingest_state('r',3,st,'boot1')['commands'],[])
   self.assertEqual(t.f.mission(mission['mission_id'])['status'],'FAILED')
   st['safety_gate']['boot_id']='gate1';self.assertEqual(t.f.ingest_state('r',4,st,'boot1')['commands'],[])
   with self.assertRaises(ApiError):t.fixture.create('new-after-replay')
   self.assertTrue(t.f.db.one('SELECT 1 FROM meta WHERE k=?',('gate_boot_reconciliation:r',)))
   # Agent boot replacement must not erase the distinct Gate block.
   t.f.hello('r',{'boot_id':'boot2','protocol_version':2,'safety_contract_version':1})
   self.assertTrue(t.f.db.one('SELECT 1 FROM meta WHERE k=?',('gate_boot_reconciliation:r',)))
   self.assertIsNone(t.f.safety_context('r'))
   from fleet_server.db import DB
   from fleet_server.service import Fleet
   t.fixture.db.c.close();t.fixture.db=DB(t.fixture.path)
   t.f=Fleet(t.fixture.db,server.base.CFG);t.fixture.f=t.f
   self.assertTrue(t.f.db.one('SELECT 1 FROM meta WHERE k=?',('gate_boot_reconciliation:r',)))
   self.assertIsNone(t.f.safety_context('r'))
  finally:t.tearDown()
 def test_missing_gate_identity_and_pending_still_block(self):
  for change in ({'boot_id':None},{'fault_epoch':None},{'fault_epoch_pending':1}):
   with self.subTest(change=change):
    r=agent._make_ready_robot();m=agent._make_gate_msg();d=json.loads(m.data);d.update(change);m.data=json.dumps(d);r._set('gate',m)
    self.assertFalse(r.snapshot()['safety_gate']['movement_permitted'])
 def test_late_unstamped_and_old_timestamp_are_unverifiable(self):
  for extra in ({},{'published_at':1.0}):
   with self.subTest(extra=extra):
    r=agent._make_ready_robot();m=agent._make_gate_msg();d=json.loads(m.data);d.update(extra);m.data=json.dumps(d);r._set('gate',m)
    self.assertTrue(r.snapshot()['safety_gate']['movement_permitted'])
    print('SAFETY_GAP synthetic admission: original publication time unverified',extra,flush=True)
 def test_same_boot_out_of_order_unversioned_status_gap(self):
  g=GateObservationTracker();d={'boot_id':'A','fault_epoch':1,'fault_latched':False,'cmd_blocked':False,'fault_epoch_pending':0}
  g.on_message(dict(d,state='FAILSAFE_SENSOR_TIMEOUT',sequence=2),now_mono=1)
  g.on_message(dict(d,state='NORMAL',sequence=1),now_mono=2)
  self.assertEqual(g.evaluate(now_mono=2)['status'],'NORMAL')
  print('SAFETY_GAP synthetic tracker ignores unsupported sequence field',flush=True)
if __name__=='__main__':unittest.main(verbosity=2)
