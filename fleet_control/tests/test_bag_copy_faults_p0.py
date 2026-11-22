import errno
import importlib.util
from pathlib import Path
import sqlite3
import tempfile
import json
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/verify_bag_copy.py'
spec = importlib.util.spec_from_file_location('p0_copy', SCRIPT)
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)

class Tests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory(prefix='p0-copy-fault-');self.addCleanup(self.tmp.cleanup)
  self.root=Path(self.tmp.name);self.run=self.root/'run';self.bag=self.run/'bag';self.bag.mkdir(parents=True)
  self.dest=self.root/'laptop';self.dest.mkdir();(self.run/'exit.json').write_text(json.dumps({'reason':'operator_signal_SIGINT','recorder_returncode':0}))
  (self.bag/'metadata.yaml').write_text('test')
  with sqlite3.connect(self.bag/'bag_0.db3') as c:c.execute('create table t(x)')
  self.patch=patch.object(m.subprocess,'run',return_value=type('R',(),{'returncode':0})());self.patch.start();self.addCleanup(self.patch.stop)
 def test_disk_full_preserves_source_and_refuses_receipt(self):
  source=m.digest(self.bag/'bag_0.db3'); original=Path.open
  def simulated(path,mode='r',*a,**kw):
   if path.parent.parent==self.dest and path.name=='bag_0.db3' and mode=='xb':raise OSError(errno.ENOSPC,'injected disk full')
   return original(path,mode,*a,**kw)
  with patch.object(Path,'open',simulated):
   with self.assertRaises(OSError):m.copy_verified(self.run,self.dest,'run1')
  self.assertEqual(source,m.digest(self.bag/'bag_0.db3'))
  self.assertFalse((self.dest/'run1'/'verified-manifest.json').exists())
 def test_destination_hash_mismatch_refuses_receipt(self):
  original=m.digest
  def altered(path):
   return '0'*64 if path.parent.parent==self.dest else original(path)
  with patch.object(m,'digest',altered):
   with self.assertRaisesRegex(ValueError,'hash mismatch'):m.copy_verified(self.run,self.dest,'run1')
  self.assertFalse((self.dest/'run1'/'verified-manifest.json').exists())

if __name__=='__main__':unittest.main()
