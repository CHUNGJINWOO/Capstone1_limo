import ast,base64,copy,http.client,io,json,math,os,secrets,sys,tempfile,threading,time,unittest,zlib
from pathlib import Path
import numpy as np
from PIL import Image
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from dashboard.core import Grid,Store,metric_depth,compose,pack,unpack,grid_png,fresh_status
from dashboard.server import Gateway,Handler,ThreadingHTTPServer,key,wire_numbers

NS=1794545848381456197
def grid(**kwargs):
 v={'width':3,'height':2,'resolution':.05,'origin':{'x':1.,'y':2.,'yaw':.3},'frame_id':'map','stamp_ns':NS,'source_age_s':0.,'cells_zlib':pack(np.array([[0,1,100],[-1,50,99]],np.int8).tobytes())}
 v.update(kwargs);return v
def packet(kind='ROS_LIVE',**kwargs):
 p={'kind':kind,'session':'a','sequence':1,'grids':{'global_costmap':grid()},'paths':{}}
 p.update(kwargs);return p

class Core(unittest.TestCase):
 def test_png_ros_y_axis_is_not_mirrored(self):
  g=Grid(grid());im=np.array(Image.open(io.BytesIO(grid_png(g.cells,True))))
  self.assertEqual(list(im[0,0]),[112,128,146,65]);self.assertEqual(int(im[1,0,3]),0)
 def test_clearing_update_removes_obstacle(self):
  g=Grid(grid());old=g.meta('global_costmap')['image_id']
  g.update({'x':2,'y':0,'width':1,'height':1,'cells_zlib':pack(b'\x00')})
  self.assertEqual(g.cells[0,2],0);self.assertNotEqual(old,g.meta('global_costmap')['image_id'])
 def test_patch_bounds_and_frame(self):
  for patch in [dict(x=-1,y=0,width=1,height=1),dict(x=3,y=0,width=1,height=1),dict(x=0,y=0,width=1,height=1,frame_id='other')]:
   with self.assertRaises(ValueError):Grid(grid()).update(dict(patch,cells_zlib=pack(b'\0')))
 def test_bad_cell_values_rejected(self):
  with self.assertRaises(ValueError):Grid(grid(cells_zlib=pack(bytes([127])*6)))
 def test_bad_grid_shape_rejected(self):
  with self.assertRaises(ValueError):Grid(grid(width=0))
 def test_nonfinite_origin_rejected(self):
  with self.assertRaises(ValueError):Grid(grid(origin={'x':math.nan,'y':0,'yaw':0}))
 def test_malicious_compressed_size_rejected(self):
  with self.assertRaises(ValueError):unpack(pack(b'x'*10000),6)
 def test_yaw_composition(self):
  v=compose({'x':10,'y':20,'yaw':math.pi/2},{'x':2,'y':1,'yaw':.3})
  self.assertAlmostEqual(v['x'],9);self.assertAlmostEqual(v['y'],22)
 def test_tf_required_for_odom_grid(self):
  s=Store('live');s.ingest(packet(grids={'local_costmap':grid(frame_id='odom')}))
  self.assertFalse(s.snapshot()['layers']['local_costmap']['renderable'])
 def test_rolling_origin_and_yaw_replace_previous(self):
  s=Store('live');s.ingest(packet());s.ingest(packet(sequence=2,grids={'global_costmap':grid(origin={'x':8,'y':-5,'yaw':.7})}))
  self.assertEqual(s.snapshot()['layers']['global_costmap']['origin']['x'],8)
 def test_live_test_isolation_both_directions(self):
  for profile,kind in [('live','SYNTHETIC_ROS_TEST'),('test','ROS_LIVE')]:
   with self.assertRaises(ValueError):Store(profile).ingest(packet(kind))
 def test_late_packet_and_retired_boot_rejected(self):
  s=Store('live');s.ingest(packet())
  with self.assertRaises(ValueError):s.ingest(packet())
  s.ingest(packet(session='b',sequence=0))
  with self.assertRaises(ValueError):s.ingest(packet(session='a',sequence=9))
 def test_reupload_does_not_make_old_sensor_fresh(self):
  s=Store('live');s.ingest(packet(grids={'map':grid(source_age_s=20)}));self.assertEqual(s.snapshot()['layers']['map']['status'],'STALE')
 def test_no_timestamp_never_claimed_ok(self):
  self.assertEqual(fresh_status(0,None,True),'TIME_UNVERIFIED')
 def test_depth_16uc1_mm_invalid_and_padding(self):
  a=np.array([[0,1000,2500],[500,2000,4000]],'<u2');raw=b''.join(row.tobytes()+b'xx' for row in a)
  d=metric_depth(raw,width=3,height=2,step=8,encoding='16UC1')
  values=np.frombuffer(unpack(d['values_f32le_zlib'],24),'<f4').reshape(2,3)
  self.assertTrue(np.isnan(values[0,0]));self.assertAlmostEqual(values[0,2],2.5);self.assertFalse(d['rgb_registered'])
 def test_depth_32fc1_m_nonfinite_invalid(self):
  a=np.array([1.25,np.inf,np.nan,0],'<f4');d=metric_depth(a.tobytes(),width=4,height=1,step=16,encoding='32FC1')
  v=np.frombuffer(unpack(d['values_f32le_zlib'],16),'<f4');self.assertEqual(v[0],1.25);self.assertTrue(np.isnan(v[1:]).all())
 def test_depth_big_endian_and_sampling(self):
  a=np.full((2,640),1500,'>u2');d=metric_depth(a.tobytes(),width=640,height=2,step=1280,encoding='16UC1',bigendian=True)
  self.assertEqual(d['width'],320);self.assertAlmostEqual(d['valid_min_m'],1.5,5)
 def test_unknown_depth_encoding_rejected(self):
  with self.assertRaises(ValueError):metric_depth(b'1234',width=1,height=1,step=4,encoding='rgba8')
 def test_timestamp_keys_preserved_beyond_js_precision(self):
  p=wire_numbers({'stamp_ns':NS,'nested':{'ros_time_ns':NS},'seq':10})
  self.assertEqual(p['stamp_ns'],str(NS));self.assertEqual(p['nested']['ros_time_ns'],str(NS));self.assertIsInstance(p['seq'],int)
 def test_rgb_pair_requires_frame_stamp_and_boot(self):
  a={'seq':1,'frame_id':'rgb','stamp_ns':NS};b=dict(a,stamp_ns=NS+1)
  self.assertNotEqual(key(a,'boot'),key(b,'boot'));self.assertNotEqual(key(a,'boot'),key(a,'new_boot'))
 def test_bridge_has_no_ros_control_writes(self):
  tree=ast.parse((Path(__file__).parents[1]/'bridge_node.py').read_text())
  forbidden={'create_publisher','create_client','create_service','send_goal_async','reset_latch'}
  self.assertFalse({n.attr for n in ast.walk(tree) if isinstance(n,ast.Attribute)} & forbidden)

class Http(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();d=Path(self.temp.name);self.viewer=secrets.token_urlsafe(32);self.bridge=secrets.token_urlsafe(32)
  for name,t in [('viewer',self.viewer),('bridge',self.bridge)]:p=d/name;p.write_text(t);p.chmod(0o600)
  self.app=Gateway({'port':18570,'profile':'live','viewer_token_file':str(d/'viewer'),'bridge_token_file':str(d/'bridge')})
  self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler);self.server.app=self.app
  threading.Thread(target=self.server.serve_forever,daemon=True).start();self.port=self.server.server_address[1]
 def tearDown(self):self.server.shutdown();self.server.server_close();self.temp.cleanup()
 def request(self,method,path,body=None,token=None,headers=None):
  c=http.client.HTTPConnection('127.0.0.1',self.port,timeout=3);h={'Content-Type':'application/json',**(headers or {})}
  if token:h['Authorization']='Bearer '+token
  c.request(method,path,json.dumps(body).encode() if body is not None else None,h);r=c.getresponse();raw=r.read();status=r.status;rh=dict(r.headers);c.close();return status,raw,rh
 def test_telemetry_requires_auth(self):self.assertEqual(self.request('GET','/api/state')[0],401)
 def test_viewer_can_only_read(self):
  self.assertEqual(self.request('GET','/api/state',token=self.viewer)[0],200)
  self.assertEqual(self.request('POST','/api/bridge',packet(),self.viewer)[0],401)
 def test_bridge_cannot_read_viewer_data(self):self.assertEqual(self.request('GET','/api/state',token=self.bridge)[0],401)
 def test_all_robot_command_posts_blocked(self):
  for path in ['/api/v1/missions','/api/v1/missions/m_x/cancel','/cmd_vel','/reset_latch']:
   self.assertEqual(self.request('POST',path,{},self.viewer)[0],405)
 def test_live_rejects_synthetic(self):self.assertEqual(self.request('POST','/api/bridge',packet('SYNTHETIC_ROS_TEST'),self.bridge)[0],400)
 def test_real_grid_upload_and_png(self):
  self.assertEqual(self.request('POST','/api/bridge',packet(),self.bridge)[0],200)
  status,raw,h=self.request('GET','/api/grid/global_costmap.png',token=self.viewer);self.assertEqual(status,200);self.assertTrue(raw.startswith(b'\x89PNG'))
 def test_cookie_login_and_logout(self):
  status,_,h=self.request('POST','/api/login',{'token':self.viewer});self.assertEqual(status,200);cookie=h['Set-Cookie'].split(';')[0]
  self.assertIn('HttpOnly',h['Set-Cookie']);self.assertIn('SameSite=Strict',h['Set-Cookie'])
  self.assertEqual(self.request('GET','/api/state',headers={'Cookie':cookie})[0],200)
  self.request('POST','/api/logout',{},headers={'Cookie':cookie})
  self.assertEqual(self.request('GET','/api/state',headers={'Cookie':cookie})[0],401)
 def test_csrf_origin(self):self.assertEqual(self.request('POST','/api/login',{'token':self.viewer},headers={'Origin':'https://untrusted.invalid'})[0],403)
 def test_nonobject_json_rejected(self):self.assertEqual(self.request('POST','/api/login',[],self.viewer)[0],400)
 def test_content_security_policy_self_only(self):
  _,_,h=self.request('GET','/');self.assertIn("connect-src 'self'",h['Content-Security-Policy']);self.assertIn("frame-ancestors 'none'",h['Content-Security-Policy'])

if __name__=='__main__':unittest.main(verbosity=2)
