import json,time,types,unittest
import test_agent_safety_v1 as fixture
from limo_robot_agent import ros_adapter

class Tests(unittest.TestCase):
 def setUp(self):
  self.r=fixture._make_ready_robot(True);r=self.r
  r.sim_r8_supervisor=True;r.mode='sim';r.simulated=True;r.allow_sim_safety_gate=True;r._use_sim_time=lambda:True
  r._r8_started=False;r._r8_seq=0;r._r8_challenge_seq=-1
  import collections
  r._r8_nonce_history=collections.deque(maxlen=256);self.out=[];r._r8_pub=types.SimpleNamespace(publish=lambda m:self.out.append(json.loads(m.data)))
  self.old=ros_adapter.String;ros_adapter.String=lambda data:types.SimpleNamespace(data=data)
  self.session={'connected':True,'reconciliation_required':False,'lease':{'epoch':1,'expires_at':time.time()+60}}
  r.session_fn=lambda:self.session;r.execution_allowed=lambda:True
  r.gate_tracker.evaluate=lambda **k:dict(boot_id='gate',boot_changed=False,r7_invalid=False)
 def tearDown(self):ros_adapter.String=self.old
 def send(self,seq=1,nonce='a'*32,**kw):
  d=dict(contract='gate-supervisor-challenge/0-sim',gate_boot_id='gate',nonce=nonce,challenge_seq=seq,issued_mono_ns=time.monotonic_ns());d.update(kw)
  self.r._on_r8_challenge(types.SimpleNamespace(data=json.dumps(d)))
 def test_good_replay_expiry(self):
  self.send();self.assertEqual(len(self.out),1);self.assertTrue(self.out[-1]['execution_allowed'])
  self.send();self.send(seq=2,nonce='b'*32,issued_mono_ns=time.monotonic_ns()-1_000_000_000);self.assertEqual(len(self.out),1)
 def test_reconciliation_revokes(self):
  self.send();self.session['reconciliation_required']=True;self.send(seq=2,nonce='b'*32)
  self.assertFalse(self.out[-1]['execution_allowed'])
 def test_wrong_boot_or_real_no_publish(self):
  self.send(gate_boot_id='other');self.r.mode='real';self.send();self.assertEqual(self.out,[])
 def test_no_lease_does_not_bind(self):
  self.session['lease']=None;self.send();self.assertEqual(self.out,[])

if __name__=='__main__':unittest.main(verbosity=2)
