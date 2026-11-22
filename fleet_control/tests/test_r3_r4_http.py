"""Loopback HTTP contract with synthetic Agent, no ROS or robot."""
import json, sys, tempfile, threading, unittest, urllib.request, urllib.error
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'server'))
from fleet_server.auth import Auth, init_tokens
from fleet_server.db import DB
from fleet_server.service import Fleet
from fleet_server.http_api import Servers

class Tests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();p=Path(self.tmp.name);init_tokens(str(p),str(p/'sec'),['r'])
  self.agent=(p/'sec/agent-r.token').read_text().strip();self.op=(p/'sec/operator.token').read_text().strip();self.viewer=(p/'sec/viewer.token').read_text().strip()
  self.db=DB(str(p/'db'));self.f=Fleet(self.db,SimpleNamespace(offline_after_s=10,command_ttl_s=120,resend_after_s=.1))
  self.s=Servers(self.f,Auth(str(p)),['127.0.0.1'],0);self.port=self.s.srvs[0].server_address[1]
  self.t=threading.Thread(target=self.s.serve_forever,daemon=True);self.t.start()
 def tearDown(self):self.s.stop_evt.set();self.t.join(3);self.db.c.close();self.tmp.cleanup()
 def req(self,method,path,token,body):
  req=urllib.request.Request(f'http://127.0.0.1:{self.port}/api/v1'+path,method=method,data=json.dumps(body).encode(),headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'})
  try:
   with urllib.request.urlopen(req,timeout=3) as r:return r.status,json.load(r)
  except urllib.error.HTTPError as e:return e.code,json.loads(e.read())
 def test_http_admission_and_fault(self):
  self.assertEqual(self.req('POST','/agent/hello',self.agent,{'robot_id':'r','agent':{'boot_id':'b','protocol_version':2,'safety_contract_version':1}})[0],200)
  ex={'active':None,'nav2_goals':{'known':True,'active_count':0,'received_status_array':True,'age_s':0.}}
  self.assertEqual(self.req('POST','/agent/reconcile',self.agent,{'robot_id':'r','agent_boot_id':'b','journal':{},'journal_ok':True,'execution':ex})[0],200)
  st={'safety_gate':{'boot_id':'synthetic-gate','status':'NORMAL','age_s':0.,'fault_epoch':1,'fault_latched':False,'cmd_blocked':False,'movement_permitted':True},'operation_mode':'NAVIGATION','operation_revision':1,'map_epoch':1,'localization_epoch':1,'map_version_id':'m','localization_map_version_id':'m','map_verified':True,'localization_status':'CONFIRMED','execution':ex}
  self.assertEqual(self.req('POST','/agent/state',self.agent,{'robot_id':'r','agent_boot_id':'b','seq':1,'state':st})[0],200)
  status,lease=self.req('POST','/robots/r/lock',self.op,{'action':'acquire'});self.assertEqual(status,200)
  body={'robot_id':'r','goal':{'frame_id':'map','x':1,'y':2},'idempotency_key':'http-a','lease_epoch':lease['epoch']}
  self.assertEqual(self.req('POST','/missions',self.viewer,body)[0],403)
  self.assertEqual(self.req('POST','/missions',self.op,{k:v for k,v in body.items() if k!='lease_epoch'})[0],409)
  code,mission=self.req('POST','/missions',self.op,body);self.assertEqual(code,201)
  code,repeat=self.req('POST','/missions',self.op,body);self.assertEqual(code,200);self.assertEqual(mission['mission_id'],repeat['mission_id'])
  st['safety_gate'].update(status='FAILSAFE_SENSOR_TIMEOUT',fault_latched=True,cmd_blocked=True,fault_epoch=2,movement_permitted=False)
  code,result=self.req('POST','/agent/state',self.agent,{'robot_id':'r','agent_boot_id':'b','seq':2,'state':st});self.assertEqual(code,200);self.assertEqual(result['commands'],[])
  self.assertEqual(self.req('POST','/missions',self.op,{**body,'idempotency_key':'fault'})[0],409)
if __name__=='__main__':unittest.main(verbosity=2)
