"""Independent r7 SIM_ONLY consumer checks. No ROS or Gate producer emulation claims."""
import copy
import sys
import time
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'agent/limo_robot_agent'),str(ROOT/'server')]
from limo_robot_agent.safety_protocol import GateObservationTracker, sim_r7_nav_evidence
from fleet_server.safety import sim_r7_report_valid

class Tests(unittest.TestCase):
    def setUp(self):
        self.t=GateObservationTracker(); self.now=time.monotonic()
        self.m=dict(contract_version='gate-status/1.2-sim',status_seq=1,published_mono_ns=int(self.now*1e9),
                    host_boot_id=self.t.host_boot_id,boot_id='a',gate_boot_id='a',fault_epoch=0,
                    fault_epoch_pending=0,sim_time=1.,state='NORMAL',safety_state='NORMAL',fault_latched=False,
                    cmd_blocked=False,trust='UNAUTHENTICATED_SIM_ONLY',nav_evidence=dict(scope_version=1,observed=True,
                    all_idle_proven=True,active_uuids=[],servers={'navigate_to_pose':dict(verdict='IDLE_PROVEN_EL',writer_gid='010f7f0100000001')}))
        self.direct=dict(publishers=1,received_status_array=False,active_goal_uuids=[])
        self.agent=dict(mode='sim',simulated=True)
    def evaluate(self):
        self.t.on_message(self.m,self.now)
        g=self.t.evaluate(self.now,expect_source_clock=True);g['single_publisher_graph']=True
        return g
    def proof(self,g=None): return sim_r7_nav_evidence(g or self.evaluate(),('navigate_to_pose',),self.direct)
    def test_cold_start_no_fabricated_status(self):
        g=self.evaluate();p=self.proof(g)
        self.assertIsNotNone(p);self.assertFalse(p['received_status_array'])
        self.assertTrue(sim_r7_report_valid(self.agent,g,p,('navigate_to_pose',)))
    def test_server_default_and_real_fail_closed(self):
        g=self.evaluate();p=self.proof(g)
        self.assertFalse(sim_r7_report_valid(self.agent,g,p,()))
        self.assertFalse(sim_r7_report_valid(dict(mode='real',simulated=True),g,p,('navigate_to_pose',)))
    def test_missing_scope(self):
        self.assertIsNone(sim_r7_nav_evidence(self.evaluate(),('spin',),self.direct))
    def test_unobserved_or_multiple_writer(self):
        for verdict in ['UNOBSERVED:no_writer','UNOBSERVED:writers=2']:
            with self.subTest(verdict=verdict):
                self.t=GateObservationTracker();self.m['nav_evidence']['servers']['navigate_to_pose']['verdict']=verdict
                self.assertIsNone(self.proof())
    def test_duplicate_and_reverse_seq_latch(self):
        self.evaluate();self.assertIsNone(self.proof())
        self.m['status_seq']=3;self.assertIsNone(self.proof())
    def test_heartbeat_does_not_latch(self):
        self.evaluate();self.m['status_seq']=2;self.m['sim_time']=1.1
        self.assertIsNotNone(self.proof())
    def test_boot_replay(self):
        self.evaluate();self.m.update(boot_id='b',gate_boot_id='b',status_seq=2)
        self.assertIsNone(self.proof());self.m.update(boot_id='a',gate_boot_id='a',status_seq=3)
        self.assertIsNone(self.proof())
    def test_delayed_source(self):
        self.m['published_mono_ns']-=3_000_000_000;self.assertIsNone(self.proof())
    def test_missing_host(self):
        del self.m['host_boot_id'];self.assertIsNone(self.proof())
    def test_stale_rx(self):
        self.evaluate();g=self.t.evaluate(self.now+3,expect_source_clock=True);g['single_publisher_graph']=True
        self.assertIsNone(self.proof(g))
    def test_pending_and_safety_state(self):
        for k,v in [('fault_epoch_pending',1),('fault_epoch_pending',True),('safety_state','FAULT')]:
            with self.subTest(k=k,v=v):
                t=copy.deepcopy(self.m); self.m[k]=v;self.t=GateObservationTracker()
                self.assertIsNone(self.proof());self.m=t
    def test_bad_types(self):
        for value in [True,1.2,'1',None,-1,2**64]:
            with self.subTest(value=value):
                self.t=GateObservationTracker();self.m['status_seq']=value;self.assertIsNone(self.proof())
    def test_invalid_evidence(self):
        for value in [None,[],True,dict(scope_version=True)]:
            with self.subTest(value=value):
                self.t=GateObservationTracker();self.m['nav_evidence']=value;self.assertIsNone(self.proof())
    def test_gid_unverified(self):
        g=self.evaluate();g['single_publisher_graph']=False;self.assertIsNone(self.proof(g))
    def test_contradictory_active_status(self):
        self.direct.update(received_status_array=True,active_goal_uuids=['1'*32]);self.assertIsNone(self.proof())
    def test_server_tampering(self):
        g=self.evaluate();p=self.proof(g);p['gate_nav_evidence']['all_idle_proven']=False
        self.assertFalse(sim_r7_report_valid(self.agent,g,p,('navigate_to_pose',)))
    def test_unknown_active_uuid_remains_foreign(self):
        self.m['nav_evidence'].update(all_idle_proven=False,active_uuids=['1'*32])
        self.m['nav_evidence']['servers']['navigate_to_pose']['verdict']='ACTIVE_GOALS'
        g=self.evaluate();p=self.proof(g)
        self.assertEqual(p['foreign_active_goal_uuids'],['1'*32])
        self.assertFalse(sim_r7_report_valid(self.agent,g,p,('navigate_to_pose',)))

if __name__=='__main__':unittest.main(verbosity=2)
