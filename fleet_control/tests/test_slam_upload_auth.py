import json,sys,tempfile,threading,unittest,urllib.request,urllib.error
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'server'))
from fleet_server.auth import Auth,init_tokens
from fleet_server.db import DB
from fleet_server.http_api import Servers
from fleet_server.map_registry import MapRegistry
from fleet_server.service import Fleet

class Tests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory(prefix='slam-upload-auth-');self.addCleanup(self.tmp.cleanup);self.d=Path(self.tmp.name)
  init_tokens(str(self.d),str(self.d/'tokens'),['r1','r2'])
  self.f=Fleet(DB(str(self.d/'fleet.db')),SimpleNamespace(offline_after_s=6,command_ttl_s=120,resend_after_s=1))
  self.reg=MapRegistry(str(self.d/'registry.db'),str(self.d/'maps'))
  self.s=Servers(self.f,Auth(str(self.d)),['127.0.0.1'],0,map_registry=self.reg)
  self.thread=threading.Thread(target=self.s.serve_forever,daemon=True);self.thread.start();self.url='http://127.0.0.1:'+str(self.s.srvs[0].server_address[1])+'/api/v1/agent/slam-map'
 def tearDown(self):
  self.s.stop_evt.set();self.thread.join(timeout=5);self.reg.close()
 def post(self,role,body):
  token=(self.d/'tokens'/f'{role}.token').read_text().strip()
  req=urllib.request.Request(self.url,data=json.dumps(body).encode(),headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'})
  try:
   with urllib.request.urlopen(req,timeout=3) as r:return r.status
  except urllib.error.HTTPError as e:return e.code
 def test_wrong_principal_and_session(self):
  self.assertEqual(403,self.post('viewer',{'robot_id':'r1'}))
  self.assertEqual(403,self.post('agent-r2',{'robot_id':'r1'}))
  self.assertEqual(409,self.post('agent-r1',{'robot_id':'r1','agent_boot_id':'forged'}))
if __name__=='__main__':unittest.main()
