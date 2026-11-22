"""Synthetic protocol v1 over isolated loopback HTTP."""
import unittest
import test_r3_r4_http as old

class Tests(old.Tests):
 def test_http_admission_and_fault(self):
  self.assertEqual(self.req('POST','/agent/hello',self.agent,{'robot_id':'r','agent':{'boot_id':'b','protocol_version':2,'safety_contract_version':1}})[0],200)
  ex={'active':None,'nav2_goals':{'known':True,'active_count':0,'received_status_array':True,'age_s':0.}}
  self.assertEqual(self.req('POST','/agent/reconcile',self.agent,{'robot_id':'r','agent_boot_id':'b','journal':{},'journal_ok':True,'execution':ex})[0],200)
  st={'safety_gate':{'boot_id':'g','status':'NORMAL','age_s':0.,'fault_epoch':1,'fault_latched':False,'cmd_blocked':False,'movement_permitted':True},'operation_mode':'NAVIGATION','operation_revision':1,'map_epoch':1,'localization_epoch':1,'map_version_id':'m','localization_map_version_id':'m','map_verified':True,'localization_status':'CONFIRMED','execution':ex}
  self.assertEqual(self.req('POST','/agent/state',self.agent,{'robot_id':'r','agent_boot_id':'b','seq':1,'state':st})[0],200)
  status,lease=self.req('POST','/robots/r/lock',self.op,{'action':'acquire'});self.assertEqual(status,200)
  body={'robot_id':'r','goal':{'frame_id':'map','x':1,'y':2},'idempotency_key':'http-a','lease_epoch':lease['epoch']}
  self.assertEqual(self.req('POST','/missions',self.op,body)[0],201)
  self.assertEqual(self.req('POST','/missions',self.op,body)[0],200)
  code,result=self.req('POST','/agent/state',self.agent,{'robot_id':'r','agent_boot_id':'b','seq':2,'state':st})
  self.assertEqual(code,200);self.assertEqual(result['lease']['epoch'],lease['epoch'])
  self.assertEqual(result['commands'][0]['payload']['safety_context']['gate_boot_id'],'g')
  st['safety_gate'].update(status='FAILSAFE_SENSOR_TIMEOUT',fault_latched=True,cmd_blocked=True,fault_epoch=2,movement_permitted=False)
  self.assertEqual(self.req('POST','/agent/state',self.agent,{'robot_id':'r','agent_boot_id':'b','seq':3,'state':st})[1]['commands'],[])

if __name__=='__main__':unittest.main(verbosity=2)
