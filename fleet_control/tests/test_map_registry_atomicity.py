"""Independent regressions for scoped idempotency and transaction rollback."""
import base64, hashlib, json, os, signal, subprocess, sys, tempfile, threading, unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'server'))
from fleet_server.map_registry import MapRegistry,parse_pgm_p5,parse_ros_map_yaml,compute_bundle_sha256
from fleet_server.auth import Principal

class Tests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.r=MapRegistry(self.tmp.name+'/db',self.tmp.name+'/store');self.a=Principal('alice','operator');self.b=Principal('bob','operator')
 def tearDown(self):self.r.close();self.tmp.cleanup()
 def call(self,actor,key,body,path='/api/v1/map-registry/facilities'):
  return self.r.dispatch_write('POST',path,actor,body=body,headers={'Idempotency-Key':key})
 def test_scopes_replay_conflict_and_restart(self):
  body={'facility_id':'a','name':'A'}
  self.assertEqual(self.call(self.a,'k',body)[0],201)
  self.assertTrue(self.call(self.a,'k',body)[1]['idempotent_replay'])
  second=self.call(self.b,'k',body) # existing facility behavior remains independently idempotent
  self.assertEqual(second[0],201);self.assertFalse(second[1]['idempotent_replay'])
  self.assertEqual(self.call(self.b,'different',{'facility_id':'b','name':'B'})[0],201)
  self.assertEqual(self.call(self.a,'k',{'facility_id':'c','name':'C'})[0],409)
  self.r.close();self.r=MapRegistry(self.tmp.name+'/db',self.tmp.name+'/store')
  self.assertTrue(self.call(self.a,'k',body)[1]['idempotent_replay'])
  self.assertEqual(len(self.r.q('SELECT * FROM facilities')),2)
 def test_simultaneous_different_payload_only_one_commit(self):
  barrier=threading.Barrier(3);results=[];errors=[]
  def run(i):
   try:barrier.wait();results.append(self.call(self.a,'race',{'facility_id':'race'+str(i),'name':'X'})[0])
   except Exception as e:errors.append(e)
  ts=[threading.Thread(target=run,args=(i,)) for i in (1,2)]
  for t in ts:t.start()
  barrier.wait()
  for t in ts:t.join(5)
  self.assertFalse(errors);self.assertFalse(any(t.is_alive() for t in ts));self.assertEqual(sorted(results),[201,409])
  self.assertEqual(len(self.r.q('SELECT * FROM facilities')),1);self.assertEqual(len(self.r.q("SELECT * FROM map_registry_events WHERE event_type='FACILITY_CREATED'")),1)
 def test_insert_receipt_failure_rolls_back_registration_and_audit(self):
  original=self.r.x
  def fail(sql,params=()):
   if 'INSERT INTO map_write_idempotency' in sql:raise RuntimeError('injected receipt failure')
   return original(sql,params)
  with patch.object(self.r,'x',side_effect=fail):
   with self.assertRaises(RuntimeError):self.call(self.a,'k',{'facility_id':'a','name':'A'})
  self.assertEqual(self.r.q('SELECT * FROM facilities'),[]);self.assertEqual(self.r.q('SELECT * FROM map_registry_events'),[])
 def test_legacy_key_bound_to_original_actor(self):
  body={'facility_id':'a','name':'A'};result=self.call(self.a,'old',body)
  row=self.r.one('SELECT * FROM map_write_idempotency');self.r.x('UPDATE map_write_idempotency SET scope_key=? WHERE scope_key=?',('POST:/facilities:old',row['scope_key']))
  self.assertTrue(self.call(self.a,'old',body)[1]['idempotent_replay'])
  self.assertEqual(self.call(self.b,'old',{'facility_id':'b','name':'B'})[0],201)
 def test_process_exit_after_rename_preserves_orphan_without_db_commit(self):
  self.r.create_facility('f','F');self.r.create_building('b','f','B');self.r.create_floor('fl','b','L1')
  yaml=b'image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
  pgm=b'P5\n2 2\n255\n'+bytes([0,254,254,0])
  existing=self.r.register_map_bundle(floor_id='fl',label='existing',yaml_bytes=yaml,pgm_bytes=pgm,verification_status='VERIFIED',verification_evidence={'method':'fixture'})['map_version']['map_version_id']
  self.r.set_floor_active_map('fl',existing,actor='test',reason='existing test fixture')
  self.r.close()
  code='''import base64,os,signal,sys\nsys.path.insert(0,"fleet_control/server")\nfrom fleet_server.map_registry import MapRegistry\nfrom fleet_server.auth import Principal\nr=MapRegistry(sys.argv[1]+"/db",sys.argv[1]+"/store")\norig=os.replace\ndef crash(src,dst):\n orig(src,dst)\n if os.path.dirname(dst)==r.bundles_dir: os.kill(os.getpid(),signal.SIGKILL)\nos.replace=crash\ny=b"image: map.pgm\\nresolution: 0.05\\norigin: [0, 0, 0]\\nnegate: 0\\noccupied_thresh: 0.65\\nfree_thresh: 0.196\\n"\np=b"P5\\n2 2\\n255\\n"+bytes([0,254,254,0])\nbody={"floor_id":"fl","label":"m","yaml_b64":base64.b64encode(y).decode(),"pgm_b64":base64.b64encode(p).decode()}\nr.dispatch_write("POST","/api/v1/map-registry/maps",Principal("alice","operator"),body=body,headers={"Idempotency-Key":"crash"})\n'''
  proc=subprocess.run([sys.executable,'-c',code,self.tmp.name],cwd=str(Path(__file__).resolve().parents[2]),timeout=10)
  self.assertEqual(proc.returncode,-signal.SIGKILL)
  self.r=MapRegistry(self.tmp.name+'/db',self.tmp.name+'/store')
  self.assertEqual(len(self.r.q('SELECT * FROM map_versions')),1);self.assertTrue(self.r.verify_bundle_integrity(existing)['ok'])
  self.assertEqual(self.r.q('SELECT * FROM map_write_idempotency'),[])
  self.assertEqual(self.r.q('SELECT active_map_version_id FROM floor_active_maps')[0]['active_map_version_id'],existing)
  self.assertEqual(list(Path(self.r.staging_dir).iterdir()),[])
  diagnosis=self.r.reconcile_storage(quarantine_corrupted=False)
  self.assertEqual(len(diagnosis['unregistered_bundles']),1)
  info=diagnosis['unregistered_bundles'][0];self.assertEqual(info['status'],'ORPHAN_UNREGISTERED_BUNDLE')
  orphan=Path(self.r.bundles_dir)/info['storage_key'];manifest=json.loads((orphan/'manifest.json').read_text())
  for name,key in [('map.yaml','yaml_sha256'),('map.pgm','pgm_sha256')]:
   self.assertEqual(hashlib.sha256((orphan/name).read_bytes()).hexdigest(),manifest['sha256'][key])
  body={'floor_id':'fl','label':'m','yaml_b64':base64.b64encode((orphan/'map.yaml').read_bytes()).decode(),'pgm_b64':base64.b64encode((orphan/'map.pgm').read_bytes()).decode()}
  self.assertEqual(self.call(self.a,'crash',body,'/api/v1/map-registry/maps')[0],409)
  self.assertEqual(parse_pgm_p5((orphan/'map.pgm').read_bytes())['content_sha256'],manifest['sha256']['content_sha256'])
  pm=parse_pgm_p5((orphan/'map.pgm').read_bytes());ym=parse_ros_map_yaml((orphan/'map.yaml').read_bytes())
  digest=compute_bundle_sha256(content_sha256=pm['content_sha256'],yaml_sha256=ym['yaml_sha256'],pgm_sha256=pm['pgm_sha256'],posegraph_sha256=None,posegraph_data_sha256=None,width=pm['width'],height=pm['height'],resolution=ym['resolution'],origin_x=ym['origin_x'],origin_y=ym['origin_y'],origin_yaw=ym['origin_yaw'],negate=ym['negate'],occupied_thresh=ym['occupied_thresh'],free_thresh=ym['free_thresh'],frame_id=manifest['frame_id'])
  self.assertEqual(digest,manifest['sha256']['bundle_sha256'])
  self.assertTrue(orphan.exists());self.assertEqual(len(self.r.q('SELECT * FROM map_versions')),1)

if __name__=='__main__':unittest.main(verbosity=2)
