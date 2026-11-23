"""Actual Fleet bench regression, in-memory data; no network or robot."""
import math
import threading
import time
import unittest
from fleet_server.service import ApiError, Fleet

class DB:
    def q(self,*args): return []
    def one(self,*args): return None   # 2026-11-20 Claude: no operating-mode record = legacy robot (original in superseded_test_originals)
    def event(self,*args,**kwargs): pass

class BenchSafety(unittest.TestCase):
    def setUp(self):
        f=self.f=Fleet.__new__(Fleet);f.lock=threading.RLock();f.db=DB();f.manual_benches={}
        self.mapping='IDLE';f.mapping_blocks_motion=lambda _:self.mapping not in ('IDLE','STOPPED')
        self.lease={'owner':'op','epoch':3,'expires_at':time.time()+30};f.lease_view=lambda _:self.lease
        f.robots={'r':{'online':True,'reconciled':True,'state_ts':time.time(),
          'agent':{'manual_web_bench':True,'mode':'real','simulated':False,'missions_enabled':False,'boot_id':'a','protocol_version':2,'safety_contract_version':1},
          'state':{'execution':{'active':None,'nav2_goals':{'known':True,'publishers':1,'active_count':0,'received_status_array':True,'age_s':0.}}}}}
        f.check_session=lambda *args:None
    def start(self): return self.f.manual_bench('r','op',{'action':'start','epoch':3})['session_id']
    def sample(self,sid,seq=1): return self.f.manual_bench('r','op',{'action':'sample', 'input_format': 'normalized_axes_v1','epoch':3,'session_id':sid,'seq':seq,'linear':.7,'turn':.2,'deadman':True})
    def test_normal_bench_and_deadman(self):
        sid=self.start();self.sample(sid);v=self.f.pull_manual_bench('r','a')['sample']
        self.assertEqual((v['linear'],v['turn']),(.7,.2))
        self.f.manual_bench('r','op',{'action':'sample', 'input_format': 'normalized_axes_v1','epoch':3,'session_id':sid,'seq':2,'linear':0.,'turn':0.,'deadman':False})
        self.assertFalse(self.f.pull_manual_bench('r','a')['sample']['deadman'])
    def test_offline_reconnect_never_revives(self):
        sid=self.start();self.f.robots['r']['online']=False
        with self.assertRaises(ApiError):self.sample(sid)
        self.f.robots['r']['online']=True
        with self.assertRaises(ApiError):self.sample(sid)
        self.assertNotEqual(self.start(),sid)
    def test_mapping_states_invalidate(self):
        for state in ('MAPPING','STARTING','PAUSING','PAUSED','SAVING','STOPPING','UNKNOWN'):
            self.setUp();sid=self.start();self.mapping=state
            with self.assertRaises(ApiError):self.sample(sid)
            self.assertFalse(self.f.manual_benches)
    def test_nav_evidence_invalidates(self):
        for key,value in (('known',False),('received_status_array',False),('publishers',2),('publishers',0),('active_count',1),('active_count','error'),('active_count',True),('age_s',30),('age_s',math.nan),('age_s',None)):
            self.setUp();sid=self.start();self.f.robots['r']['state']['execution']['nav2_goals'][key]=value
            with self.assertRaises(ApiError):self.sample(sid)
            self.assertFalse(self.f.manual_benches)
    def test_lease_epoch_boot_and_sequence(self):
        sid=self.start();self.sample(sid)
        with self.assertRaises(ApiError):self.sample(sid)
        self.f.robots['r']['agent']['boot_id']='b'
        with self.assertRaises(ApiError):self.sample(sid,2)
        self.f.robots['r']['agent']['boot_id']='a'
        with self.assertRaises(ApiError):self.sample(sid,2)
        sid=self.start();self.lease=None
        with self.assertRaises(ApiError):self.sample(sid)
        self.assertFalse(self.f.manual_benches)
    def test_other_actor_cannot_invalidate(self):
        sid=self.start()
        with self.assertRaises(ApiError):self.f.manual_bench('r','other',{'action':'start','epoch':3})
        self.sample(sid)

if __name__=='__main__': unittest.main(verbosity=2)
