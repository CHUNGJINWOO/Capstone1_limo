"""Transaction primitive checks using isolated SQLite; no ROS or network."""
import sys, tempfile, threading, time, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'server'))
from fleet_server.db import DB

class TransactionTests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory(); self.db=DB(str(Path(self.temp.name)/'db.sqlite'))
 def tearDown(self):
  self.db.c.close();self.temp.cleanup()
 def test_failure_rolls_back_writes_and_events(self):
  with self.assertRaisesRegex(RuntimeError,'injected'):
   with self.db.transaction():
    self.db.x('INSERT INTO meta(k,v) VALUES(?,?)',('mission','queued'))
    self.db.event('test','intermediate')
    raise RuntimeError('injected')
  self.assertIsNone(self.db.one('SELECT v FROM meta WHERE k=?',('mission',)))
  self.assertEqual(self.db.q('SELECT * FROM events'),[])
 def test_nested_rollback_keeps_outer_and_connection_usable(self):
  with self.db.transaction():
   self.db.x('INSERT INTO meta VALUES(?,?)',('outer','yes'))
   with self.assertRaises(ValueError):
    with self.db.transaction():
     self.db.x('INSERT INTO meta VALUES(?,?)',('inner','no'))
     raise ValueError('injected')
   self.db.x('INSERT INTO meta VALUES(?,?)',('outer2','yes'))
  self.assertEqual([r['k'] for r in self.db.q('SELECT k FROM meta ORDER BY k')],['outer','outer2'])
 def test_parallel_writers_serialize(self):
  self.db.x('INSERT INTO meta VALUES(?,?)',('n','0'))
  barrier=threading.Barrier(3);errors=[]
  def worker():
   try:
    barrier.wait()
    for _ in range(30):
     with self.db.transaction():
      n=int(self.db.one('SELECT v FROM meta WHERE k=?',('n',))['v'])
      time.sleep(.0001)
      self.db.x('UPDATE meta SET v=? WHERE k=?',(str(n+1),'n'))
   except Exception as e:errors.append(e)
  threads=[threading.Thread(target=worker) for _ in range(2)]
  for t in threads:t.start()
  barrier.wait()
  for t in threads:t.join(5)
  self.assertFalse(errors);self.assertFalse(any(t.is_alive() for t in threads))
  self.assertEqual(self.db.one('SELECT v FROM meta WHERE k=?',('n',))['v'],'60')
if __name__ == '__main__':unittest.main(verbosity=2)
