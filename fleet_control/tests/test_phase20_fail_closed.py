"""Phase 2.0: no synthetic robot or external service; fail-closed regressions."""
import os, sys, tempfile, threading, time, unittest
from types import SimpleNamespace
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'server'),str(ROOT/'agent/limo_robot_agent')]
from fleet_server.ownership import OwnershipMixin
from limo_robot_agent.core import AgentCore

class Handler:
 def __init__(self): self.started=0
 def start_navigate(self,*args):self.started+=1
 def cancel_mission(self,*args):pass

class T(unittest.TestCase):
 def test_empty_status_is_not_terminal_evidence(self):
  now=time.time();o=OwnershipMixin();o.cfg=SimpleNamespace(offline_after_s=10)
  o.robots={'r':{'online':True,'reconciled':True,'state_ts':now,'state':{'execution':{'active':None,'nav2_goals':{'known':True,'received_status_array':True,'active_count':0,'age_s':0}}}}}
  ok,reason,_=o.recovery_evidence('r')
  self.assertFalse(ok);self.assertEqual(reason,'terminal_evidence_unavailable')

 def test_recovery_id_does_not_clear_journal_from_empty_status(self):
  with tempfile.TemporaryDirectory() as d:
   a=AgentCore('r','http://invalid','',lambda:{'execution':{'active':None,'nav2_goals':{'known':True,'active_count':0}}},Handler(),{},d,log=lambda *_:None)
   a.seen['old']={'status':'RECONCILIATION_REQUIRED','mission_id':'m','type':'navigate_to_pose','seq':1};a._save()
   a._apply_recovery(['old']);self.assertEqual(a.seen['old']['status'],'RECONCILIATION_REQUIRED')

 def test_command_body_conflict_persists_across_restart(self):
  with tempfile.TemporaryDirectory() as d:
   h=Handler();a=AgentCore('r','http://invalid','',lambda:{},h,{'boot_id':'b'},d,log=lambda *_:None)
   now=time.time();cmd={'command_id':'c','robot_id':'r','mission_id':'m','type':'navigate_to_pose','seq':1,'attempt':1,'payload':{'goal':{'x':1}},'issued_at':now,'expires_at':now+60,'owner_boot_id':'b'}
   a._on_command(cmd);self.assertEqual(h.started,1)
   a._on_command(dict(cmd,attempt=2));self.assertEqual(h.started,1)
   changed=dict(cmd,payload={'goal':{'x':2}});a._on_command(changed)
   self.assertTrue(a.seen['c'].get('identity_conflict'))
   reloaded=AgentCore('r','http://invalid','',lambda:{},Handler(),{'boot_id':'b2'},d,log=lambda *_:None)
   self.assertFalse(reloaded.journal_ok)

 def test_concurrent_intent_reservations_are_serialized(self):
  with tempfile.TemporaryDirectory() as d:
   a=AgentCore('r','http://invalid','',lambda:{},Handler(),{'boot_id':'b'},d,log=lambda *_:None)
   a.seen={'one':{'status':'RECEIVED'},'two':{'status':'RECEIVED'}};a._save()
   barrier=threading.Barrier(3);results=[]
   def reserve(cid):
    barrier.wait()
    try:a.record_command_meta(cid,submission_intent=True);results.append('reserved')
    except RuntimeError:results.append('blocked')
   threads=[threading.Thread(target=reserve,args=(cid,)) for cid in ('one','two')]
   for thread in threads:thread.start()
   barrier.wait()
   for thread in threads:thread.join(3)
   self.assertFalse(any(thread.is_alive() for thread in threads))
   self.assertCountEqual(results,['reserved','blocked'])

 def test_duplicate_owner_and_operation_mismatch_block(self):
  for field in ('owner_type','owner_id','operation_id','mission_id'):
   with self.subTest(field=field), tempfile.TemporaryDirectory() as d:
    h=Handler();a=AgentCore('r','http://invalid','',lambda:{},h,{'boot_id':'b'},d,log=lambda *_:None)
    now=time.time();cmd={'command_id':'c','robot_id':'r','mission_id':'m','type':'navigate_to_pose','seq':1,'payload':{'goal':{'x':1}},'issued_at':now,'expires_at':now+60,'owner_boot_id':'b','owner_type':'WEB','owner_id':'u','operation_id':'op'}
    a._on_command(cmd);changed=dict(cmd);changed[field]='different';a._on_command(changed)
    self.assertEqual(h.started,1);self.assertTrue(a.seen['c'].get('identity_conflict'));self.assertFalse(a.journal_ok)

if __name__=='__main__':unittest.main(verbosity=2)
