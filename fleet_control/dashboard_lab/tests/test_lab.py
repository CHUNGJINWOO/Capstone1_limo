import argparse,ast,asyncio,base64,hashlib,json,math,secrets,tempfile,time,unittest,zlib
from pathlib import Path
import cv2,numpy as np
from aiohttp import web,ClientSession
from dashboard_lab.media import Hub,Latest,PROFILES,FAULTS,key,envelope,decode_envelope,percentiles
from dashboard_lab.server import Lab
from dashboard_lab.state import ReadModel

def asset():return {'id':'fixture.jpg','sha256':'test','rgb':np.zeros((200,320,3),np.uint8)}
class Core(unittest.TestCase):
 def test_envelope_preserves_exact_nanoseconds_and_payload(self):
  h={'stamp_ns':'1794572440657407536','frame_id':'A','sequence':9};v,p=decode_envelope(envelope(h,b'jpeg'));self.assertEqual(v,h);self.assertEqual(p,b'jpeg')
 def test_result_key_distinguishes_boot_and_frame(self):
  h=dict(boot='a',stream='rgb',sequence=1,frame_id='A',stamp_ns='123');self.assertNotEqual(key(h),key(dict(h,boot='b')));self.assertNotEqual(key(h),key(dict(h,stamp_ns='124')))
 def test_invalid_envelope_rejected(self):
  with self.assertRaises(ValueError):decode_envelope(b'\x00\x01\x00\x01bad')
 def test_metrics_include_tail_percentiles(self):self.assertGreater(percentiles(range(100))['p99'],percentiles(range(100))['p95'])
 def test_profile_adapts_and_recovers(self):
  h=Hub([asset()]);h.set_fault('bandwidth150');self.assertEqual(h.profile,'emergency');h.set_fault('rtt400');self.assertEqual(h.profile,'low');h.set_fault('normal');self.assertEqual(h.profile,'balanced')
 def test_manual_profile_not_overridden(self):
  h=Hub([asset()]);h.adaptive=False;h.set_fault('stalequeue');self.assertEqual(h.profile,'balanced')
 def test_invalid_fault_rejected(self):
  with self.assertRaises(ValueError):Hub([asset()]).set_fault('arbitrary')
 def test_fixture_offline_and_video_independent(self):
  h=Hub([asset()]);r=ReadModel(h);s=r.update();self.assertFalse(s['mission']['robots'][1]['online']);self.assertEqual(len(s['mission']['robots']),2);self.assertTrue(s['mission']['missions'][2]['terminal'])
 def test_live_mode_does_not_invent_registration(self):
  s=ReadModel(Hub([asset()],mode='LIVE')).update();self.assertIsNone(s['rmf']['registered']);self.assertEqual(s['mission']['robots'],[])
 def test_no_robot_or_system_mutation_tools_in_lab(self):
  root=Path(__file__).parents[1]
  for name in ['server.py','media.py','state.py']:
   tree=ast.parse((root/name).read_text());calls={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
   self.assertFalse(calls&{'create_publisher','send_goal_async','create_client'})
 def test_mode_cycle_does_not_reuse_retired_sensor_boot(self):
  h=Hub([asset()]);r=ReadModel(h);r.update();h.mode='LIVE';r.update();h.mode='SIMULATION';self.assertEqual(r.update()['source_mode'],'SIMULATION')
class Async(unittest.IsolatedAsyncioTestCase):
 async def test_latest_buffer_has_one_value(self):
  b=Latest()
  for i in range(100):await b.put(i)
  ver,value=await b.get(0);self.assertEqual((ver,value),(100,99))
 async def test_all_codecs_run_without_silent_task_failure(self):
  h=Hub([asset()]);h.h264_enabled=True;await h.start()
  try:
   await asyncio.sleep(.6);self.assertGreater(h.metrics['h264_encoded'],2);self.assertGreater(h.metrics['depth_encoded'],2);self.assertEqual(h.snapshot()['task_errors'],[])
  finally:await h.close()
 async def test_lossless_depth_is_separate_from_jpeg(self):
  h=Hub([asset()]);await h.start()
  try:
   await asyncio.sleep(.2);p=h.depth.value;meta,raw=h.raw_depth[p.header['depth_key']];a=np.frombuffer(zlib.decompress(raw),dtype='<u2');self.assertEqual(a.size,320*200);self.assertEqual(meta['encoding'],'16UC1');self.assertGreater((a==0).sum(),0);self.assertTrue(p.payload.startswith(b'\xff\xd8'))
  finally:await h.close()
 async def test_replay_metric_values_are_bitwise_preserved(self):
  a=np.array([[np.nan,1.234],[2.5,0]],'<f4');recorded={'depth':{'width':2,'height':2,'stamp_ns':123,'values_f32le_zlib':base64.b64encode(zlib.compress(a.tobytes())).decode()}}
  h=Hub([asset()],recorded,mode='REPLAY');await h.start()
  try:
   await asyncio.sleep(.15);meta,raw=next(iter(h.raw_depth.values()));self.assertEqual(zlib.decompress(raw),a.tobytes());self.assertFalse(meta['unit_verified']);self.assertFalse(meta['rgb_registered'])
  finally:await h.close()
class Api(unittest.IsolatedAsyncioTestCase):
 async def asyncSetUp(self):
  self.temp=tempfile.TemporaryDirectory();d=Path(self.temp.name);cv2.imwrite(str(d/'one.jpg'),asset()['rgb']);self.lab=Lab(argparse.Namespace(state=str(d/'state'),assets=str(d),recorded=None,mode='SIMULATION',inference=None,baseline_url=None,mission_url=None,mission_token_file=None,adapter_snapshot=None))
  app=web.Application();app.router.add_route('*','/api/{tail:.*}',self.lab.api);app.router.add_get('/ws/{stream}',self.lab.ws);self.runner=web.AppRunner(app);await self.runner.setup();site=web.TCPSite(self.runner,'127.0.0.1',0);await site.start();self.url='http://127.0.0.1:'+str(site._server.sockets[0].getsockname()[1]);self.session=ClientSession()
 async def asyncTearDown(self):await self.session.close();await self.runner.cleanup();self.temp.cleanup()
 async def req(self,method,path,role='viewer',**kw):
  t=self.lab.viewer if role=='viewer' else self.lab.tester
  return await self.session.request(method,self.url+path,headers={'Authorization':'Bearer '+t},**kw)
 async def test_measured_slow_ack_reduces_profile(self):
  await self.lab.hub.start()
  try:
   async with self.session.ws_connect(self.url+'/ws/rgb',headers={'Authorization':'Bearer '+self.lab.viewer}) as ws:
    message=await ws.receive(timeout=2);h,_=decode_envelope(message.data);await asyncio.sleep(.2);await ws.send_json({'ack':h['sequence']});await asyncio.sleep(.05)
    self.assertGreater(self.lab.hub.delivery_ema_ms,100);self.assertEqual(self.lab.hub.profile,'low')
  finally:await self.lab.hub.close()
 async def test_adapter_snapshot_cannot_make_stale_registration_live(self):
  p=Path(self.temp.name)/'adapter.json';p.write_text(json.dumps({'observed_at':time.time()-20,'registered':True,'fleet':'fixture'}));self.lab.args.adapter_snapshot=str(p);self.lab.hub.mode='LIVE';self.lab.current=self.lab.model.update();await self.lab.external_state();self.assertEqual(self.lab.current['rmf']['status'],'STALE');self.assertIsNone(self.lab.current['rmf']['registered'])
 async def test_auth_required(self):
  async with self.session.get(self.url+'/api/state') as r:self.assertEqual(r.status,401)
 async def test_viewer_cannot_inject_faults(self):
  async with await self.req('POST','/api/lab/control',json={'fault':'rtt400'}) as r:self.assertEqual(r.status,403)
 async def test_lab_role_can_only_control_local_fixture(self):
  async with await self.req('POST','/api/lab/control',role='lab',json={'fault':'rtt400'}) as r:self.assertEqual(r.status,200)
  self.assertEqual(self.lab.hub.profile,'low')
 async def test_live_mode_refuses_fault_injection(self):
  self.lab.hub.mode='LIVE'
  async with await self.req('POST','/api/lab/control',role='lab',json={'fault':'rtt400'}) as r:self.assertEqual(r.status,403)
 async def test_robot_commands_are_not_routes(self):
  for path in ['/api/v1/missions','/api/v1/missions/test/cancel','/api/cmd_vel','/api/reset_latch']:
   async with await self.req('POST',path,json={}) as r:self.assertEqual(r.status,405)
 async def test_login_origin_rejected(self):
  async with self.session.post(self.url+'/api/login',json={'token':self.lab.viewer},headers={'Origin':'http://untrusted.invalid'}) as r:self.assertEqual(r.status,403)
 async def test_unknown_metric_key_explicitly_unavailable(self):
  async with await self.req('GET','/api/depth/raw?key=old') as r:self.assertEqual(r.status,410)
 async def test_session_survives_server_object_restart(self):
  async with self.session.post(self.url+'/api/login',json={'token':self.lab.viewer}) as r:self.assertEqual(r.status,200)
  self.assertTrue(self.lab.session_file.exists());j=json.loads(self.lab.session_file.read_text());self.assertTrue(any(v['role']=='viewer' for v in j.values()))
if __name__=='__main__':unittest.main(verbosity=2)
