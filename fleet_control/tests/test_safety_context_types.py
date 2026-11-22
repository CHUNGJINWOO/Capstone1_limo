"""Malformed synthetic input and persisted context checks; no robot/ROS."""
import copy, json, unittest
import test_safety_context_v1 as base
from fleet_server.service import ApiError
from fleet_server.auth import Principal

class Tests(unittest.TestCase):
 def setUp(self):
  self.t=base.Tests('test_old_agent_blocked');self.t.setUp();self.f=self.t.f
 def tearDown(self):self.t.tearDown()
 def test_bad_state_fields_rejected(self):
  for section,key,values in [('agent','safety_contract_version',[True,1.0,'1']),('agent','boot_id',[True,1,' ','']),('gate','boot_id',[True,1,' ']),('gate','fault_epoch',[True,-1,2**63,1.0]),('state','operation_revision',[True,-1,2**63]),('state','map_epoch',[True,-1,2**63]),('state','localization_epoch',[True,-1,2**63]),('state','map_version_id',[True,1,' ']),('gate','age_s',[True,-1,float('nan'),float('inf')])]:
   for value in values:
    with self.subTest(section=section,key=key,value=value):
     r=self.t.robot; target={'agent':r['agent'],'gate':r['state']['safety_gate'],'state':r['state']}[section];old=target[key];target[key]=value
     self.assertIsNone(self.f.safety_context('r'));target[key]=old
 def test_bad_ttl_rejected(self):
  for ttl in (True,'10',-1,float('nan'),float('inf'),10**400):
   with self.subTest(ttl=str(ttl)[:20]),self.assertRaises(ApiError):self.f.lease_action('r',Principal('op','operator'),'renew',ttl)
 def test_bad_command_fields_never_delivered(self):
  for key,value in [('version',True),('fault_epoch',True),('operation_revision',True),('lease_epoch',True),('issued_at',True),('expires_at',float('nan')),('gate_boot_id',' '),('agent_boot_id',1)]:
   with self.subTest(key=key):
    t=base.Tests('test_old_agent_blocked');t.setUp()
    try:
     m,_=t.create();db=t.fixture.db;row=db.one('SELECT * FROM commands WHERE mission_id=?',(m['mission_id'],));payload=json.loads(row['payload_json']);payload['safety_context'][key]=value
     db.x('UPDATE commands SET payload_json=? WHERE command_id=?',(json.dumps(payload),row['command_id']))
     self.assertEqual(t.f._deliver('r')['commands'],[])
    finally:t.tearDown()
 def test_bool_lease_epoch_rejected(self):
  with self.assertRaises(ApiError):self.f.create_mission('r','navigate_to_pose',base.base.GOAL,'bool',created_by='op',lease_epoch=True)

if __name__=='__main__':unittest.main(verbosity=2)
