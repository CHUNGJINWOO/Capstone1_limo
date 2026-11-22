"""Durable same-session Nav2 terminal evidence versus ownership state arrival."""
import copy, json, unittest
import test_r3_r4_server_safety as fixture

class TerminalRace(fixture.Tests):
 def running(self):
  m,_=self.create(); cmd=self.f._deliver('r')['commands'][0]; cid=cmd['command_id']; mid=m['mission_id']; uid='ab'*16
  self.f.ack_command('r',cid,'EXECUTING',agent_boot_id='boot1')
  self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)',('observed_goal_uuid:'+cid,uid))
  r={'command_id':cid,'mission_id':mid,'goal_uuid':uid,'status':'SUCCEEDED','terminal_status':'SUCCEEDED','terminal_source':'nav2_action_result','terminal_agent_boot_id':'boot1','safety_context':cmd['payload']['safety_context']}
  return mid,cid,uid,r
 def test_matching_durable_result_precedes_handle_loss(self):
  mid,cid,uid,r=self.running(); self.f.check_execution('r',{'active':None,'journal_terminals':[r]})
  self.assertEqual(self.f.mission(mid)['status'],'SUCCEEDED')
  self.assertFalse(self.f.robots['r']['reconciliation_required'])
 def test_missing_terminal_still_blocks(self):
  mid,cid,uid,r=self.running();self.f.check_execution('r',{'active':None,'journal_terminals':[]})
  self.assertEqual(self.f.mission(mid)['status'],'RECONCILIATION_REQUIRED')
 def test_invalid_terminal_identity_still_blocks(self):
  mid,cid,uid,r=self.running();r['goal_uuid']='cd'*16
  self.f.check_execution('r',{'active':None,'journal_terminals':[r]})
  self.assertEqual(self.f.mission(mid)['status'],'RECONCILIATION_REQUIRED')
 def test_state_before_ack_with_duplicate_and_stale_state(self):
  mid,cid,uid,r=self.running();state=copy.deepcopy(self.state)
  state['execution']['journal_terminals']=[r]
  self.f.ingest_state('r',2,state,'boot1')
  self.assertEqual(self.f.mission(mid)['status'],'SUCCEEDED')
  self.assertFalse(self.f.robots['r']['reconciliation_required'])
  self.f.ack_command('r',cid,'SUCCEEDED',agent_boot_id='boot1')
  with self.assertRaises(fixture.ApiError):self.f.ingest_state('r',2,state,'boot1')
  self.assertEqual(self.f.mission(mid)['status'],'SUCCEEDED')
 def test_missing_prior_observed_uuid_cannot_finish(self):
  mid,cid,uid,r=self.running();self.db.x('DELETE FROM meta WHERE k=?',('observed_goal_uuid:'+cid,))
  self.f.check_execution('r',{'active':None,'journal_terminals':[r]})
  self.assertEqual(self.f.mission(mid)['status'],'RECONCILIATION_REQUIRED')
 def test_old_boot_record_cannot_finish_after_agent_restart(self):
  mid,cid,uid,r=self.running();self.f.robots['r']['agent']['boot_id']='boot2'
  self.f.check_execution('r',{'active':None,'journal_terminals':[r]})
  self.assertEqual(self.f.mission(mid)['status'],'RECONCILIATION_REQUIRED')
 def test_duplicate_terminal_records_remain_uncertain(self):
  mid,cid,uid,r=self.running();self.f.check_execution('r',{'active':None,'journal_terminals':[r,dict(r)]})
  self.assertEqual(self.f.mission(mid)['status'],'RECONCILIATION_REQUIRED')
 def test_active_handle_loss_cannot_be_masked_by_terminal(self):
  mid,cid,uid,r=self.running();self.f.check_execution('r',{'active':{'mission_id':mid,'command_id':cid,'goal_uuid':uid,'goal_handle_available':False},'journal_terminals':[]})
  self.assertEqual(self.f.mission(mid)['status'],'RECONCILIATION_REQUIRED')

def make_reject(field, value):
 def check(self):
  mid,cid,uid,r=self.running();r[field]=value
  self.f.check_execution('r',{'active':None,'journal_terminals':[r]})
  self.assertEqual(self.f.mission(mid)['status'],'RECONCILIATION_REQUIRED')
 return check
for field,value in [('terminal_agent_boot_id','old-boot'),('terminal_source','cancel_ack'),('mission_id','other'),('status','EXECUTING'),('terminal_status','EXECUTING'),('safety_context',{})]:
 setattr(TerminalRace,'test_reject_'+field,make_reject(field,value))

if __name__=='__main__': unittest.main()
