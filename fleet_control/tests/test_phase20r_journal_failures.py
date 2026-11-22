"""Journal failure must never produce an unpersisted successful ACK."""
import json,sys,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'agent/limo_robot_agent'))
from limo_robot_agent.core import AgentCore

class Handler:
 def __init__(self):self.starts=0
 def start_navigate(self,*args):self.starts+=1
 def cancel_mission(self,*args):pass

class Tests(unittest.TestCase):
 def test_failed_fsync_preserves_disk_and_withholds_terminal_ack(self):
  with tempfile.TemporaryDirectory() as d:
   h=Handler();a=AgentCore('r','http://invalid','',lambda:{},h,{'boot_id':'b'},d,log=lambda *_:None)
   a.seen['c']={'status':'ACCEPTED','mission_id':'m','type':'navigate_to_pose','seq':1,'detail':'accepted'};a._save()
   before=Path(a.seen_path).read_bytes()
   with patch('limo_robot_agent.core.os.fsync',side_effect=OSError('injected failure')):
    a.ack('c','SUCCEEDED','terminal')
   self.assertFalse(a.journal_ok);self.assertEqual(Path(a.seen_path).read_bytes(),before)
   self.assertEqual(a.pending_acks,[])
   b=AgentCore('r','http://invalid','',lambda:{},Handler(),{'boot_id':'b2'},d,log=lambda *_:None)
   self.assertEqual(b.seen['c']['status'],'RECONCILIATION_REQUIRED')
 def test_failed_receipt_persistence_does_not_execute(self):
  with tempfile.TemporaryDirectory() as d:
   h=Handler();a=AgentCore('r','http://invalid','',lambda:{},h,{'boot_id':'b'},d,log=lambda *_:None)
   a._save();before=Path(a.seen_path).read_bytes();now=time.time()
   cmd={'command_id':'c','robot_id':'r','mission_id':'m','type':'navigate_to_pose','seq':1,'payload':{'goal':{'x':1}},'issued_at':now,'expires_at':now+60,'owner_boot_id':'b'}
   with patch('limo_robot_agent.core.os.fsync',side_effect=OSError('injected failure')):
    with self.assertRaises(OSError):a._on_command(cmd)
   self.assertEqual(h.starts,0);self.assertFalse(a.journal_ok)
   self.assertEqual(Path(a.seen_path).read_bytes(),before)
 def test_cancel_command_still_reaches_handler_with_healthy_journal(self):
  with tempfile.TemporaryDirectory() as d:
   h=Handler();calls=[];h.cancel_mission=lambda cmd,ack:(calls.append(cmd['command_id']),ack('ACCEPTED','forwarded'))
   a=AgentCore('r','http://invalid','',lambda:{},h,{'boot_id':'b'},d,log=lambda *_:None)
   now=time.time();a._on_command({'command_id':'cancel','robot_id':'r','mission_id':'m','type':'cancel_mission','seq':1,'payload':{},'issued_at':now,'expires_at':now+60,'owner_boot_id':'b'})
   self.assertEqual(calls,['cancel']);self.assertEqual(a.seen['cancel']['status'],'ACCEPTED')
   self.assertEqual(a.pending_acks[-1]['status'],'ACCEPTED')

if __name__=='__main__':unittest.main(verbosity=2)
