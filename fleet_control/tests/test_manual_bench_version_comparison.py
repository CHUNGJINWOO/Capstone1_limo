"""Compare actual staging/public Fleet admission with a synthetic in-memory DB.

Set MANUAL_CONTRACT_VERSION=staging or public and select the matching package
on PYTHONPATH. This documents known gaps; accepted bench requests never drive a motor.
"""
import math
import os
import threading
import time
import unittest
from fleet_server.service import ApiError, Fleet

VERSION=os.environ['MANUAL_CONTRACT_VERSION']
assert VERSION in ('staging','public')

class DB:
    def q(self, _sql, _args): return []
    def event(self, *_args, **_kw): pass

class CompareAdmission(unittest.TestCase):
    def setUp(self):
        self.f=Fleet.__new__(Fleet);self.f.lock=threading.RLock();self.f.db=DB();self.f.manual_benches={}
        self.mapping='IDLE'
        self.f.operation_status=lambda *_args: {'state':self.mapping}
        self.f.lease_view=lambda _rid: {'owner':'operator','epoch':7,'expires_at':time.time()+30}
        mock=VERSION=='public'
        self.f.robots={'test':{'agent':{'manual_web_bench':True,'mode':'mock' if mock else 'real',
             'simulated':mock,'missions_enabled':False,'boot_id':'a','locked_velocity_arbiter':True},
             'online':True,'reconciled':True,'state_ts':time.time(),
             'state':{'execution':{'active':None,'follow_path_idle':True,
                      'nav2_goals':{'known':True,'publishers':1,'active_count':0,'received_status_array':True}}}}}
    def start(self):return self.f.manual_bench('test','operator',{'action':'start','epoch':7})
    def expect_block(self, code):
        with self.assertRaises(ApiError) as e:self.start()
        self.assertEqual(e.exception.code,code)
    def test_mapping_and_unknown(self):
        for state in ('MAPPING','UNKNOWN'):
            self.mapping=state;self.f.manual_benches={}
            if VERSION=='public':self.expect_block('mapping_active')
            else:self.assertIn('session_id',self.start());print('OBSERVED GAP staging: bench start accepted in',state,flush=True)
    def test_external_nav_active(self):
        self.f.robots['test']['state']['execution']['nav2_goals']['active_count']=1
        if VERSION=='public':self.expect_block('nav_ownership_unknown')
        else:self.assertIn('session_id',self.start());print('OBSERVED GAP staging: bench start accepted with synthetic active external Nav2 goal',flush=True)
    def test_missing_status_array(self):
        self.f.robots['test']['state']['execution']['nav2_goals']['received_status_array']=False
        self.assertIn('session_id',self.start())
        print('OBSERVED GAP',VERSION+': service admission accepts known/publisher/zero-count without received status array',flush=True)
    def test_follow_path_unconfirmed(self):
        self.f.robots['test']['state']['execution']['follow_path_idle']=False
        if VERSION=='public':self.expect_block('controller_terminal_unproven')
        else:self.assertIn('session_id',self.start());print('OBSERVED GAP staging: bench admission ignores FollowPath evidence',flush=True)
    def test_lease_and_invalid_sample(self):
        with self.assertRaises(ApiError):self.f.manual_bench('test','other',{'action':'start','epoch':7})
        response=self.start();sample={'action':'sample','epoch':7,'session_id':response['session_id'],'seq':1,'linear':math.nan,'turn':0.,'deadman':True}
        if VERSION=='public':sample['ticket']=response['ticket']
        with self.assertRaises(ApiError) as e:self.f.manual_bench('test','operator',sample)
        self.assertEqual(e.exception.code,'bad_joy_sample')
        print('PASS',VERSION+': foreign lease and NaN sample rejected',flush=True)
    def test_offline_reconnect_must_invalidate_prior_session(self):
        response=self.start()
        sample={'action':'sample','epoch':7,'session_id':response['session_id'],'seq':1,'linear':0.,'turn':0.,'deadman':False}
        if VERSION=='public': sample['ticket']=response['ticket']
        result=self.f.manual_bench('test','operator',sample)
        if VERSION=='public': sample['ticket']=result['ticket']
        sample['seq']=2
        self.f.robots['test']['online']=False
        with self.assertRaises(ApiError): self.f.manual_bench('test','operator',sample)
        self.f.robots['test']['online']=True
        if VERSION=='public':
            self.expect_sample_block(sample)
            print('PASS public: offline admission failure invalidates prior session',flush=True)
        else:
            self.assertEqual(self.f.manual_bench('test','operator',sample)['received'],2)
            print('OBSERVED GAP staging: after an offline rejection, same session can be accepted before TTL expiry',flush=True)
    def expect_sample_block(self, sample):
        with self.assertRaises(ApiError) as e:self.f.manual_bench('test','operator',sample)
        self.assertEqual(e.exception.code,'manual_bench_expired')

if __name__=='__main__':unittest.main(verbosity=2)
