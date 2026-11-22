"""Actual AgentCore journal with SIGKILL; handler is a non-ROS test double."""
import multiprocessing, os, signal, sys, tempfile, time, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'agent/limo_robot_agent'))
from limo_robot_agent.core import AgentCore

class HoldHandler:
 def __init__(self,conn):self.conn=conn;self.agent=None
 def start_navigate(self,cmd,ack):
  self.agent.record_command_meta(cmd['command_id'],submission_intent=True,submission_agent_boot_id='old')
  self.conn.send('intent_persisted');time.sleep(30)
 def cancel_mission(self,*args):pass

def child(root,conn):
 h=HoldHandler(conn);a=AgentCore('r','http://invalid','',lambda:{},h,{'boot_id':'old'},root,log=lambda *_:None);h.agent=a
 with a.lock:    # R1.4.3: motion commands need a valid server contract (the session a real hello/state reply would give)
  a.server_boot='srv';a.server_protocol_boot='srv';a.server_protocol_version=2;a.server_safety_contract_version=1;a.server_reconciliation_required=False;a.connected=True;a.lease={'owner':'op','epoch':1,'expires_at':time.time()+300};a._contract_note_locked()
 now=time.time();a._on_command({'command_id':'c','robot_id':'r','mission_id':'m','type':'navigate_to_pose','seq':1,'payload':{'goal':{'x':1}},'issued_at':now,'expires_at':now+60,'owner_boot_id':'old'})

class Tests(unittest.TestCase):
 def test_kill_after_durable_intent_reloads_unresolved(self):
  with tempfile.TemporaryDirectory() as root:
   parent,conn=multiprocessing.Pipe();p=multiprocessing.Process(target=child,args=(root,conn));p.start();conn.close()
   try:
    self.assertTrue(parent.poll(5));self.assertEqual(parent.recv(),'intent_persisted')
    os.kill(p.pid,signal.SIGKILL);p.join(5);self.assertEqual(p.exitcode,-signal.SIGKILL)
    a=AgentCore('r','http://invalid','',lambda:{},HoldHandler(parent),{'boot_id':'new'},root,log=lambda *_:None)
    self.assertTrue(a.seen['c']['submission_intent']);self.assertEqual(a.seen['c']['status'],'RECONCILIATION_REQUIRED')
   finally:
    if p.is_alive():p.kill();p.join()
    parent.close()
 def test_distinct_journals_do_not_fence_agents(self):
  with tempfile.TemporaryDirectory() as root:
   workers=[]
   try:
    for suffix in ('a','b'):
     parent,conn=multiprocessing.Pipe();p=multiprocessing.Process(target=child,args=(os.path.join(root,suffix),conn));p.start();conn.close();workers.append((p,parent))
    for _,parent in workers:
     self.assertTrue(parent.poll(5));self.assertEqual(parent.recv(),'intent_persisted')
    print('SECURITY_FAIL two_agentcores_reserved_with_different_journals',flush=True)
   finally:
    for p,parent in workers:
     if p.is_alive():p.kill()
     p.join(5);parent.close()

if __name__=='__main__':unittest.main(verbosity=2)
