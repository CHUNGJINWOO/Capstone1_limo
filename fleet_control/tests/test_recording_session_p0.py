import importlib.util
import json
import os
from pathlib import Path
import signal
import tempfile
import time
import unittest

MODULE = Path(__file__).resolve().parents[1] / 'agent/limo_robot_agent/limo_robot_agent/recording_session.py'
spec = importlib.util.spec_from_file_location('p0_recording_session', MODULE)
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)

RECORDER = '''import argparse,json,pathlib,signal,time
p=argparse.ArgumentParser(); p.add_argument('--root');p.add_argument('--label');p.add_argument('--extra');a=p.parse_args()
r=pathlib.Path(a.root)/('run_'+a.label);(r/'bag').mkdir(parents=True)
(r/'bag'/'metadata.yaml').write_text('mock')
def stop(*args):
 (r/'exit.json').write_text(json.dumps({'reason':'operator_signal_SIGINT','recorder_returncode':0}));raise SystemExit(0)
signal.signal(signal.SIGINT,stop)
while True: time.sleep(.02)
'''
VERIFIER = '''def files_for(run):
 assert (run/'exit.json').exists()
 return [run/'bag'/'metadata.yaml']
'''

class Tests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory(prefix='p0-recorder-'); self.addCleanup(self.temp.cleanup)
  self.root=Path(self.temp.name); self.recorder=self.root/'recorder.py';self.recorder.write_text(RECORDER)
  self.verifier=self.root/'verifier.py';self.verifier.write_text(VERIFIER)
  self.store=self.root/'store';self.store.mkdir()
  self.s=m.RecordingSession(self.store,'session_1',self.recorder,self.verifier)
  self.addCleanup(self.cleanup_child)
 def cleanup_child(self):
  if self.s.process and self.s.process.poll() is None:
   self.s.process.terminate();self.s.process.wait(timeout=5)
  self.s.close()
 def wait_recording(self):
  for _ in range(100):
   time.sleep(.02);st=self.s.status()
   if st['status']=='RECORDING': time.sleep(.08);return
   if st['status']=='FAILED':self.fail(str(st))
  self.fail('recorder never started')
 def test_start_stop_idempotent_and_restart(self):
  self.s.start();self.wait_recording(); pid=self.s.process.pid
  self.s.start();self.assertEqual(pid,self.s.process.pid)
  self.assertEqual('VERIFIED',self.s.stop()['status'])
  self.assertEqual('VERIFIED',self.s.stop()['status'])
  self.s.close()
  other=m.RecordingSession(self.store,'session_1',self.recorder,self.verifier)
  self.addCleanup(other.close)
  self.assertEqual('VERIFIED',other.status()['status'])
  with self.assertRaises(RuntimeError):other.start()
 def test_lock_and_restart_with_uncertain_child(self):
  self.s.start();self.wait_recording()
  with self.assertRaises(BlockingIOError):m.RecordingSession(self.store,'session_1',self.recorder,self.verifier)
  self.s.close()
  other=m.RecordingSession(self.store,'session_1',self.recorder,self.verifier)
  self.addCleanup(other.close)
  self.assertEqual('UNKNOWN',other.status()['status'])
  with self.assertRaises(RuntimeError):other.start()
  with self.assertRaises(RuntimeError):other.stop()
 def test_invalid_id_and_foreign_directory(self):
  with self.assertRaises(ValueError):m.RecordingSession(self.store,'../x',self.recorder,self.verifier)
  (self.store/'foreign').mkdir()
  with self.assertRaises(FileNotFoundError):m.RecordingSession(self.store,'foreign',self.recorder,self.verifier)
 def test_unexpected_exit_is_failure(self):
  self.s.start();self.wait_recording();self.s.process.terminate();self.s.process.wait(timeout=5)
  self.assertEqual('FAILED',self.s.status()['status'])
  with self.assertRaises(RuntimeError):self.s.stop()
 def test_topic_type_ambiguity_is_failure(self):
  self.recorder.write_text(RECORDER.replace("while True: time.sleep(.02)","print(\"Topic '/map' has more than one type associated\",flush=True)\nwhile True: time.sleep(.02)"))
  self.s.start();self.wait_recording()
  result=self.s.stop()
  self.assertEqual('FAILED',result['status']);self.assertIn('topic type ambiguity',result['detail'])

if __name__=='__main__':unittest.main()

class TimeoutTests(unittest.TestCase):
 def test_stop_timeout_is_unknown(self):
  import subprocess
  from unittest.mock import patch
  with tempfile.TemporaryDirectory(prefix='rec-timeout-') as td:
   root=Path(td);rec=root/'rec.py';rec.write_text('pass');verify=root/'verify.py';verify.write_text('pass');store=root/'store';store.mkdir()
   session=m.RecordingSession(store,'timeout',rec,verify)
   class Child:
    def poll(self):return None
    def send_signal(self, sig):self.signal=sig
    def wait(self, timeout):raise subprocess.TimeoutExpired('synthetic',timeout)
   session.process=Child();session._set('RECORDING','synthetic')
   result=session.stop(timeout=.001)
   self.assertEqual('UNKNOWN',result['status']);self.assertEqual(signal.SIGINT,session.process.signal)
   session.close()
