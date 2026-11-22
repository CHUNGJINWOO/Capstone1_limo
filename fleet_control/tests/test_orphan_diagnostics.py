"""Orphan inventory tests. All writes are confined to temporary fixtures."""
import json, os, shutil, signal, sqlite3, subprocess, sys, tempfile, unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'server'))
from fleet_server.map_registry import MapRegistry
from fleet_server.orphan_diagnostics import diagnose_orphans

Y=b'image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
P=b'P5\n2 2\n255\n'+bytes([0,254,254,0])
class Tests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();p=Path(self.tmp.name)
  self.r=MapRegistry(str(p/'db'),str(p/'store'));self.r.create_facility('f','F');self.r.create_building('b','f','B');self.r.create_floor('fl','b','L1')
  self.src=MapRegistry(str(p/'srcdb'),str(p/'srcstore'));self.src.create_facility('f','F');self.src.create_building('b','f','B');self.src.create_floor('fl','b','L1')
 def tearDown(self):self.src.close();self.r.close();self.tmp.cleanup()
 def orphan(self,label='m',yaml=Y):
  result=self.src.register_map_bundle(floor_id='fl',label=label,yaml_bytes=yaml,pgm_bytes=P)
  mid=result['map_version']['map_version_id'];row=self.src.one('SELECT storage_key FROM map_versions WHERE map_version_id=?',(mid,))
  target=Path(self.r.bundles_dir)/row['storage_key'];shutil.copytree(Path(self.src.bundles_dir)/row['storage_key'],target)
  return target
 def diag(self):
  snapshot=Path(self.tmp.name)/'offline-snapshot.db'
  dst=sqlite3.connect(snapshot)
  try:self.r.c.backup(dst)
  finally:dst.close()
  return diagnose_orphans(str(snapshot),self.r.bundle_root)
 def test_intact_staging_and_repeat_are_readonly(self):
  d=self.orphan();stage=Path(self.r.staging_dir)/'partial';stage.mkdir();(stage/'evidence').write_text('preserve')
  before=list(self.r.c.iterdump());first=self.diag();second=self.diag()
  self.assertEqual(first,second);self.assertEqual(before,list(self.r.c.iterdump()))
  self.assertEqual(first['bundles'][0]['classification'],'INTACT_UNVERIFIED_ORIGIN');self.assertFalse(first['bundles'][0]['recovery_allowed'])
  self.assertEqual(first['bundles'][0]['reupload_expected'],'409_STORAGE_CONFLICT');self.assertEqual((stage/'evidence').read_text(),'preserve');self.assertTrue(d.exists())
 def test_corrupted_and_symlink_are_refused(self):
  d=self.orphan();(d/'map.pgm').chmod(0o600);(d/'map.pgm').write_bytes(b'bad')
  self.assertEqual(self.diag()['bundles'][0]['classification'],'CORRUPT_OR_UNSAFE')
  (d/'map.pgm').unlink();(d/'map.pgm').symlink_to('/etc/passwd')
  self.assertEqual(self.diag()['bundles'][0]['classification'],'CORRUPT_OR_UNSAFE')
 def test_conflicting_existing_map_id(self):
  self.orphan(yaml=Y.replace(b'0.05',b'0.07'))
  self.r.register_map_bundle(floor_id='fl',label='m',yaml_bytes=Y,pgm_bytes=P)
  self.assertEqual(self.diag()['bundles'][0]['classification'],'ID_CONFLICT')
 def test_invalid_manifest_does_not_modify_evidence(self):
  d=self.orphan();p=d/'manifest.json';p.chmod(0o600);p.write_text('{broken')
  self.assertEqual(self.diag()['bundles'][0]['classification'],'CORRUPT_OR_UNSAFE');self.assertEqual(p.read_text(),'{broken')
 def test_diagnostic_exception_does_not_change_data(self):
  d=self.orphan();before=(d/'manifest.json').read_bytes();db_before=list(self.r.c.iterdump())
  with patch('fleet_server.orphan_diagnostics._read_file',side_effect=PermissionError('injected')):
   self.assertEqual(self.diag()['bundles'][0]['classification'],'CORRUPT_OR_UNSAFE')
  self.assertEqual((d/'manifest.json').read_bytes(),before);self.assertEqual(list(self.r.c.iterdump()),db_before)
 def test_live_nonempty_wal_is_rejected(self):
  self.assertTrue(Path(self.r.db_path+'-wal').stat().st_size)
  with self.assertRaisesRegex(ValueError,'snapshot required'):diagnose_orphans(self.r.db_path,self.r.bundle_root)
 def test_sigkill_after_rename_is_diagnosed(self):
  self.r.close()
  code='''import os,signal,sys\nsys.path.insert(0,"fleet_control/server")\nfrom fleet_server.map_registry import MapRegistry\nr=MapRegistry(sys.argv[1]+"/db",sys.argv[1]+"/store")\nold=os.replace\ndef crash(a,b):\n old(a,b)\n if os.path.dirname(b)==r.bundles_dir:os.kill(os.getpid(),signal.SIGKILL)\nos.replace=crash\ny=b"image: map.pgm\\nresolution: 0.05\\norigin: [0, 0, 0]\\nnegate: 0\\noccupied_thresh: 0.65\\nfree_thresh: 0.196\\n"\np=b"P5\\n2 2\\n255\\n"+bytes([0,254,254,0])\nr.register_map_bundle(floor_id="fl",label="m",yaml_bytes=y,pgm_bytes=p)\n'''
  child=subprocess.run([sys.executable,'-c',code,self.tmp.name],cwd=str(Path(__file__).resolve().parents[2]),timeout=10)
  self.assertEqual(child.returncode,-signal.SIGKILL)
  self.r=MapRegistry(self.tmp.name+'/db',self.tmp.name+'/store')
  item=self.diag()['bundles'][0];self.assertEqual(item['classification'],'INTACT_UNVERIFIED_ORIGIN')
  self.assertFalse(item['recovery_allowed']);self.assertEqual(self.r.q('SELECT * FROM map_versions'),[])

if __name__=='__main__':unittest.main(verbosity=2)
