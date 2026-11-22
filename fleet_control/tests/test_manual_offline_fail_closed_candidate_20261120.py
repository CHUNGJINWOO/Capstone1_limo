"""Isolated, fail-closed draft checks; imports ONLY an independently patched copy.

Set MANUAL_CANDIDATE_SERVER and MANUAL_CANDIDATE_AGENT to the copied package
parents. This does not start HTTP, ROS nodes, the Gate, or a robot.
"""
import os
import sys
import threading
import time
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.environ['MANUAL_CANDIDATE_SERVER'])
sys.path.insert(0, os.environ['MANUAL_CANDIDATE_AGENT'])
from fleet_server.service import ApiError, Fleet
from limo_robot_agent.motion_watchdog import MotionWatchdog

class DB:
    def q(self, *args): return []
    def event(self, *args, **kwargs): pass

class Candidate(unittest.TestCase):
    def setUp(self):
        self.f = Fleet.__new__(Fleet)
        self.f.lock = threading.RLock()
        self.f.db = DB()
        self.f.boot_id = 'server-a'
        self.f.manual_benches = {}
        self.f.lease_view = lambda robot: {'owner':'operator','epoch':7,'expires_at':time.time()+10}
        self.mapping = 'IDLE'
        self.f.operation_status = lambda robot: {'state':self.mapping}
        self.f.check_session = lambda robot, boot: None
        self.f.robots = {'r': {'online':True, 'reconciled':True, 'state_ts':time.time(),
            'agent':{'manual_web_bench':True, 'mode':'real', 'simulated':False,
                     'missions_enabled':False, 'boot_id':'agent-a'},
            'state':{'execution':{'nav2_goals':{}}}}}

    def seed(self):
        self.f.manual_benches['r'] = {'id':'old','epoch':7,'boot':'agent-a',
            'seq':3,'until':time.monotonic()+.25,'sample':{'linear':.2,'turn':0.,'deadman':True}}

    def request(self, **kw):
        body = {'epoch':7,'action':'sample','session_id':'old','seq':4,
                'linear':.2,'turn':0.,'deadman':True}
        body.update(kw)
        return self.f.manual_bench('r','operator',body)

    def denied(self, expected=None, **kw):
        with self.assertRaises(ApiError) as c: self.request(**kw)
        if expected: self.assertEqual(c.exception.code, expected)

    def test_offline_rejection_retires_session_even_after_same_boot_epoch(self):
        self.seed(); self.f.robots['r']['online'] = False
        self.denied('manual_bench_unavailable')
        self.assertNotIn('r', self.f.manual_benches)
        self.f.robots['r']['online'] = True
        self.denied('manual_terminal_proof_unavailable')
        self.assertNotIn('r', self.f.manual_benches)

    def test_all_mapping_states_retire_existing_session(self):
        for state in ('STARTING','MAPPING','PAUSING','PAUSED','SAVING','STOPPING','UNKNOWN'):
            with self.subTest(state=state):
                self.mapping=state; self.seed()
                self.denied('mapping_active_or_unknown')
                self.assertNotIn('r', self.f.manual_benches)

    def test_untrusted_external_nav_evidence_always_blocks(self):
        for nav in ({}, {'received_status_array':False}, {'received_status_array':True,'active_count':0},
                    {'received_status_array':True,'active_count':1}, {'active_count':'error'},
                    {'publishers':2}, {'age_s':30}, {'follow_path_idle':True}):
            with self.subTest(nav=nav):
                self.f.robots['r']['state']['execution']['nav2_goals']=nav
                self.seed(); self.denied('manual_terminal_proof_unavailable')
                self.assertNotIn('r',self.f.manual_benches)

    def test_invalid_principal_does_not_cancel_other_users_session(self):
        self.seed()
        with self.assertRaises(ApiError): self.f.manual_bench('r','intruder',{'epoch':7,'action':'start'})
        self.assertEqual(self.f.manual_benches['r']['id'],'old')

    def test_lease_missing_or_changed_retires_session(self):
        for lease in (None, {'owner':'other','epoch':8,'expires_at':time.time()+10}):
            self.seed(); self.f.lease_view=lambda robot, lease=lease:lease
            self.denied('lease_required'); self.assertNotIn('r',self.f.manual_benches)

    def test_concurrent_starts_both_blocked_without_terminal_proof(self):
        barrier=threading.Barrier(3); results=[]
        def try_start():
            barrier.wait()
            try: self.request(action='start'); results.append('accepted')
            except ApiError as e: results.append(e.code)
        ts=[threading.Thread(target=try_start) for _ in range(2)]
        for t in ts: t.start()
        barrier.wait()
        for t in ts: t.join(2)
        self.assertEqual(results.count('manual_terminal_proof_unavailable'),2)
        self.assertFalse(self.f.manual_benches)

    def test_watchdog_missing_status_cannot_rearm(self):
        f=SimpleNamespace(idle_proof_t=10.,proof_max_age=1.5,status_pubs=1,
                          status_pub_since=9.,status_settle=.5,nav_active=None)
        self.assertFalse(MotionWatchdog.idle_proven(f,10.1))
        f.nav_active=0
        self.assertTrue(MotionWatchdog.idle_proven(f,10.1)) # synthetic array, not trusted terminal evidence
        f.nav_active=1
        self.assertFalse(MotionWatchdog.idle_proven(f,10.1))
        f.nav_active=0
        self.assertFalse(MotionWatchdog.idle_proven(f,12.0))

if __name__ == '__main__': unittest.main(verbosity=2)
