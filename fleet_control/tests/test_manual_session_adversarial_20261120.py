"""Adversarial tests of actual Fleet methods with in-memory stubs, no network or ROS.

MANUAL_CONTRACT_VERSION selects staging/public package on PYTHONPATH. Known
safety failures use unittest expectedFailure, never count them as a safety PASS.
"""
import math, os, threading, time, unittest
from fleet_server.service import ApiError, Fleet

VERSION=os.environ['MANUAL_CONTRACT_VERSION']
assert VERSION in ('staging','public')
class DB:
 def q(self,*_):return []
 def one(self,*_):return None  # 2026-11-20 Claude: no operating-mode record = legacy robot (original in superseded_test_originals)
 def event(self,*_,**__):pass

class Adversarial(unittest.TestCase):
 def setUp(self):
  self.f=Fleet.__new__(Fleet);self.f.lock=threading.RLock();self.f.db=DB();self.f.manual_benches={};self.mapping='IDLE';self.lease={'owner':'op','epoch':3,'expires_at':time.time()+30}
  self.f.operation_status=lambda *_:{'state':self.mapping};self.f.lease_view=lambda _:self.lease
  pub=VERSION=='public'
  self.f.robots={'r':{'online':True,'reconciled':True,'state_ts':time.time(), 'agent':{'manual_web_bench':True,'mode':'mock' if pub else 'real','simulated':pub,'missions_enabled':False,'boot_id':'agent-a','locked_velocity_arbiter':pub},'state':{'execution':{'active':None,'follow_path_idle':True,'nav2_goals':{'known':True,'publishers':1,'active_count':0,'received_status_array':True,'age_s':0.0}}}}}
  def check(_rid,boot):
   if boot!=self.f.robots['r']['agent']['boot_id']:raise ApiError(409,'agent_session_mismatch','boot mismatch')
  self.f.check_session=check
 def start(self,actor='op'):return self.f.manual_bench('r',actor,{'action':'start','epoch':3})
 def sample(self,session,seq,*,ticket=None,**override):
  body={'action':'sample','epoch':3,'session_id':session,'seq':seq,'linear':.08,'turn':.18,'deadman':True}
  if ticket is not None:body['ticket']=ticket
  body.update(override);return self.f.manual_bench('r','op',body)
 def reject(self, fn):
  with self.assertRaises(ApiError):fn()
 def test_stale_session_wrong_epoch_and_seq(self):
  s=self.start();sid=s['session_id'];ticket=s.get('ticket');r=self.sample(sid,1,ticket=ticket);ticket=r.get('ticket')
  self.reject(lambda:self.sample('old-session',2,ticket=ticket))
  self.reject(lambda:self.sample(sid,2,ticket=ticket,epoch=2))
  for bad in (1,0,-1,True,2**53):self.reject(lambda bad=bad:self.sample(sid,bad,ticket=ticket))
  self.assertEqual(self.sample(sid,2,ticket=ticket)['received'],2)
 def test_ticket_is_single_use_public_only(self):
  if VERSION!='public':self.skipTest('staging has no ticket')
  s=self.start();self.sample(s['session_id'],1,ticket=s['ticket'])
  self.reject(lambda:self.sample(s['session_id'],2,ticket=s['ticket']))
 def test_two_browsers_concurrent_exactly_one(self):
  barrier=threading.Barrier(3);out=[]
  def run():
   barrier.wait()
   try:self.start();out.append('OK')
   except ApiError as e:out.append(e.code)
  threads=[threading.Thread(target=run) for _ in range(2)]
  for t in threads:t.start()
  barrier.wait()
  for t in threads:t.join(2)
  self.assertCountEqual(out,['OK','manual_bench_busy'])
 def test_nan_inf_and_reordered(self):
  s=self.start();sid=s['session_id'];ticket=s.get('ticket')
  for index,value in enumerate((math.nan,math.inf,-math.inf),1):self.reject(lambda v=value,i=index:self.sample(sid,i,ticket=ticket,linear=v))
  r=self.sample(sid,7,ticket=ticket);ticket=r.get('ticket')
  self.reject(lambda:self.sample(sid,6,ticket=ticket));self.reject(lambda:self.sample(sid,7,ticket=ticket))
 def test_deadman_release_and_expired_input(self):
  s=self.start();sid=s['session_id'];r=self.sample(sid,1,ticket=s.get('ticket'),deadman=False)
  pulled=self.f.pull_manual_bench('r','agent-a')['sample'];self.assertFalse(pulled['deadman'])
  if VERSION=='public':self.assertEqual((pulled['linear'],pulled['turn']),(0.,0.))
  else:self.assertEqual((pulled['linear'],pulled['turn']),(.08,.18)) # staging Agent must enforce release
  self.f.manual_benches['r']['until']=time.monotonic()-.001
  self.assertIsNone(self.f.pull_manual_bench('r','agent-a')['sample'])
  self.reject(lambda:self.sample(sid,2,ticket=r.get('ticket')))
 def test_server_reset_agent_boot_and_lease_epoch(self):
  s=self.start();sid=s['session_id'];ticket=s.get('ticket')
  self.f.manual_benches={} # new server process has no persisted manual stream
  self.reject(lambda:self.sample(sid,1,ticket=ticket))
  s=self.start();sid=s['session_id'];ticket=s.get('ticket')
  self.f.robots['r']['agent']['boot_id']='agent-b';self.reject(lambda:self.sample(sid,1,ticket=ticket))
  self.f.robots['r']['agent']['boot_id']='agent-a';self.lease={'owner':'other','epoch':4,'expires_at':time.time()+30}
  self.reject(lambda:self.sample(sid,1,ticket=ticket))
 def test_mapping_states_public_fence(self):
  for state in ('STARTING','MAPPING','PAUSING','PAUSED','SAVING','STOPPING','UNKNOWN'):
   self.mapping=state;self.reject(self.start)
 def test_active_goal_public_fence(self):
  self.f.robots['r']['state']['execution']['nav2_goals']['active_count']=1;self.reject(self.start)
 def test_known_gap_offline_rejection_invalidates_old_session(self):
  s=self.start();sid=s['session_id'];ticket=s.get('ticket')
  self.f.robots['r']['online']=False;self.reject(lambda:self.sample(sid,1,ticket=ticket))
  self.f.robots['r']['online']=True
  self.reject(lambda:self.sample(sid,1,ticket=ticket))
 @unittest.expectedFailure if VERSION=='public' else (lambda fn: fn)
 def test_known_gap_missing_nav_status_denied(self):
  self.f.robots['r']['state']['execution']['nav2_goals']['received_status_array']=False;self.reject(self.start)
 @unittest.expectedFailure if VERSION=='public' else (lambda fn: fn)
 def test_known_gap_stale_nav_status_denied(self):
  self.f.robots['r']['state']['execution']['nav2_goals']['age_s']=30.;self.reject(self.start)
 def test_duplicate_nav_publishers_denied(self):
  self.f.robots['r']['state']['execution']['nav2_goals']['publishers']=2;self.reject(self.start)
 def test_invalid_nav_status_denied(self):
  self.f.robots['r']['state']['execution']['nav2_goals']['active_count']='error';self.reject(self.start)

if __name__=='__main__':unittest.main(verbosity=2)
