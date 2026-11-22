"""Independent local-HTTP visitor-link isolation check. Only synthetic tokens."""
import json,os,sys,tempfile,threading,unittest,urllib.request,urllib.error
from pathlib import Path
root=Path(os.environ['CAPSTONE45_SNAPSHOT']);sys.path[:0]=[str(root/'fleet_control/tests'),str(root/'fleet_control/server')]
import test_phase5_guide as p5
from fleet_server.auth import Auth,init_tokens
from fleet_server.http_api import Servers

class CrossLinkHTTP(unittest.TestCase):
 def setUp(self):
  self.t=p5.Guide('test_visitor_links');self.t.setUp();self.f=self.t.f
  self.tmp=tempfile.TemporaryDirectory();init_tokens(self.tmp.name,self.tmp.name+'/secrets',['R'])
  self.server=Servers(self.f,Auth(self.tmp.name),['127.0.0.1'],0,map_registry=self.t.reg)
  self.thread=threading.Thread(target=self.server.serve_forever);self.thread.start()
  self.base='http://127.0.0.1:'+str(self.server.srvs[0].server_port)+'/api/v1'
 def tearDown(self):
  self.server.stop_evt.set();self.thread.join(4);self.t.tearDown();self.tmp.cleanup()
 def call(self,method,path,token,session):
  req=urllib.request.Request(self.base+path,method=method,data=b'{}' if method=='POST' else None,
    headers={'Authorization':'Visitor '+token,'X-Visitor-Session':session,'Content-Type':'application/json'})
  try:
   with urllib.request.urlopen(req,timeout=4) as res:return res.status
  except urllib.error.HTTPError as e:return e.code
 def test_cross_link_request_lookup_and_cancel(self):
  d=self.t.dest();self.f.set_guide_service(p5.OPER,{'robot_id':'R','enabled':True})
  a,link_a=self.t.visitor('same_browser_session_1234');b,link_b=self.t.visitor('same_browser_session_1234')
  request=self.f.create_guide_request(a,{'destination_id':d['destination_id']});path='/visitor/requests/'+request['request_id']
  self.assertEqual(self.call('GET',path,link_b['token'],'different_session_123456'),404)
  get=self.call('GET',path,link_b['token'],b.session);cancel=self.call('POST',path+'/cancel',link_b['token'],b.session)
  print('OBSERVED different Visitor link with same session: GET',get,'cancel',cancel,flush=True)
  self.assertEqual((get,cancel),(404,404))
if __name__=='__main__':unittest.main(verbosity=2)
