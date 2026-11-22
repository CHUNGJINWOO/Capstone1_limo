"""Synthetic r6 pending epoch: no actual Gate or operating movement."""
import copy, unittest
import test_agent_safety_v1 as agent_fixture
import test_safety_context_v1 as server_fixture
from limo_robot_agent.safety_protocol import GateObservationTracker, check_active_goal_safety
from fleet_server.service import ApiError

class Tests(unittest.TestCase):
 def test_agent_reports_and_blocks_unpersisted_epoch(self):
  for value in (1, 3, -1, True, None, '0'):
   with self.subTest(value=value):
    robot=agent_fixture._make_ready_robot()
    msg=agent_fixture._make_gate_msg()
    import json
    data=json.loads(msg.data);data['fault_epoch_pending']=value;msg.data=json.dumps(data)
    robot._set('gate',msg)
    state=robot.snapshot()
    self.assertIn('fault_epoch_pending',state['safety_gate'])
    self.assertFalse(state['safety_gate']['movement_permitted'])
 def test_active_goal_pending_requires_cancel(self):
  g={'rx_fresh':True,'source_fresh':True,'status':'NORMAL','fault_latched':False,'cmd_blocked':False,'boot_id':'g','fault_epoch':1,'fault_epoch_pending':1}
  reason=check_active_goal_safety({'safety_context':{'gate_boot_id':'g','fault_epoch':1}},allow_missions=True,gate_eval=g,motor_path_ok=True,op_ctx={},map_ctx={},loc_ctx={},session_ctx={})
  self.assertEqual(reason,'GATE_EPOCH_PENDING_OR_INVALID')
 def test_server_refuses_present_pending_and_malformed(self):
  for value in (1,3,-1,True,None,'0'):
   with self.subTest(value=value):
    t=server_fixture.Tests('test_old_agent_blocked');t.setUp()
    try:
     state=copy.deepcopy(t.state);state['safety_gate']['fault_epoch_pending']=value
     t.f.ingest_state('r',t.f.robots['r']['seq']+1,state,'boot1')
     with self.assertRaises(ApiError):t.create()
    finally:t.tearDown()
 def test_agent_zero_pending_is_reported_and_does_not_block(self):
  import json
  robot=agent_fixture._make_ready_robot();msg=agent_fixture._make_gate_msg()
  data=json.loads(msg.data);data['fault_epoch_pending']=0;msg.data=json.dumps(data);robot._set('gate',msg)
  gate=robot.snapshot()['safety_gate'];self.assertEqual(gate['fault_epoch_pending'],0);self.assertTrue(gate['movement_permitted'])
 def test_server_zero_pending_does_not_block(self):
  t=server_fixture.Tests('test_old_agent_blocked');t.setUp()
  try:
   state=copy.deepcopy(t.state);state['safety_gate']['fault_epoch_pending']=0;t.f.ingest_state('r',t.f.robots['r']['seq']+1,state,'boot1');t.create()
  finally:t.tearDown()
 def test_server_pending_after_admission_invalidates_old_command(self):
  t=server_fixture.Tests('test_old_agent_blocked');t.setUp()
  try:
   mission,_=t.create();state=copy.deepcopy(t.state);state['safety_gate']['fault_epoch_pending']=1
   result=t.f.ingest_state('r',t.f.robots['r']['seq']+1,state,'boot1')
   self.assertEqual(result['commands'],[])
   self.assertEqual(t.f.mission(mission['mission_id'])['status'],'FAILED')
   state['safety_gate']['fault_epoch_pending']=0
   result=t.f.ingest_state('r',t.f.robots['r']['seq']+1,state,'boot1')
   self.assertEqual(result['commands'],[])
  finally:t.tearDown()
if __name__=='__main__':unittest.main(verbosity=2)
