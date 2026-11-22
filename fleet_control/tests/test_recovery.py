#!/usr/bin/env python3
"""No motors/ROS: real HTTP + SQLite + AgentCore, controllable mock goal handles and transport faults."""
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'server'), str(ROOT/'agent/limo_robot_agent')]
from fleet_server.auth import Auth, init_tokens
from fleet_server.db import DB
from fleet_server.http_api import Servers
from fleet_server.service import Fleet, ApiError
from limo_robot_agent.core import AgentCore, Http

CFG = SimpleNamespace(offline_after_s=.2, command_ttl_s=120., resend_after_s=.01)
GOAL = {'frame_id':'map','x':1.25,'y':-.125,'yaw_rad':.35}


def wait(fn, seconds=5):
    until = time.monotonic()+seconds
    while time.monotonic()<until:
        if fn(): return
        time.sleep(.01)
    raise AssertionError('condition did not become true')


class Robot:
    def __init__(self, backend=None):
        self.backend = backend if backend is not None else {'starts':0, 'running':False}
        self.active = None
        self.pending = []
        self.last_cancel = None

    def snapshot(self):
        # Safety Context v1 inputs (synthetic, same values as test_r3_r4_server_safety): fresh NORMAL Gate,
        # NAVIGATION mode, verified map, CONFIRMED localization, observed Nav2 status.
        a = self.active
        return {'safety_gate':{'boot_id':'gate1','status':'NORMAL','age_s':0.,'fault_epoch':1,'fault_latched':False,'cmd_blocked':False,'movement_permitted':True},
                'operation_mode':'NAVIGATION','operation_revision':1,'map_epoch':1,'localization_epoch':1,'map_version_id':'map1',
                'localization_map_version_id':'map1','map_verified':True,'localization_status':'CONFIRMED',
                'execution':{'active':({'mission_id':a[0]['mission_id'], 'command_id':a[0]['command_id'], 'goal_handle_available':True} if a else None),
                             'pending_cancel_ids':[cmd['command_id'] for cmd,ack in self.pending],
                             'nav2_goals':{'known':True,'active_count':1 if self.backend.get('running') else 0,'received_status_array':True,'age_s':0.}}}

    def start_navigate(self, cmd, ack):
        self.backend['starts'] += 1; self.backend['running'] = True
        self.active = (cmd, ack); ack('ACCEPTED','mock handle'); ack('EXECUTING','mock running')

    def cancel_mission(self, cmd, ack):
        self.last_cancel = cmd
        if not self.active or self.active[0]['mission_id'] != cmd['mission_id']:
            ack('RECONCILIATION_REQUIRED','mock has no owned handle'); return
        self.pending.append((cmd,ack)); ack('ACCEPTED','cancel requested, not finished')

    def finish(self, status='SUCCEEDED'):
        cmd,ack = self.active; ack(status,'mock terminal result'); self.active = None; self.backend['running'] = False
        for cmd,ack in self.pending: ack('SUCCEEDED' if status=='CANCELED' else 'REJECTED','mock terminal '+status)
        self.pending.clear()


class Transport:
    def __init__(self, inner): self.inner=inner; self.block=False; self.drop_ack=False
    def post(self,path,body,timeout=None):
        if self.block or (self.drop_ack and path.endswith('/ack')): raise TimeoutError('program-level injected timeout')
        return self.inner.post(path,body,timeout)


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.root=Path(self.tmp.name)
        init_tokens(str(self.root),str(self.root/'secrets'),['r'])
        self.token=(self.root/'secrets/agent-r.token').read_text().strip()
        self.agents=[]; self.db=DB(str(self.root/'fleet.db')); self.server=None
        self.start_server()

    def start_server(self):
        self.fleet=Fleet(self.db,CFG); self.server=Servers(self.fleet,Auth(str(self.root)),['127.0.0.1'],getattr(self,'port',0))
        self.port=self.server.srvs[0].server_address[1]
        self.server_thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.server_thread.start()

    def stop_server(self):
        self.server.stop_evt.set();self.server_thread.join(3);self.assertFalse(self.server_thread.is_alive())

    def agent(self, robot=None, boot='boot1', state='state'):
        robot=robot or Robot()
        a=AgentCore('r',f'http://127.0.0.1:{self.port}',self.token,robot.snapshot,robot,{'boot_id':boot},str(self.root/state),period_s=.03,log=lambda _:None)
        a.http.timeout=.1;a.http=Transport(a.http)
        th=threading.Thread(target=a.run,daemon=True);th.start();self.agents.append((a,th))
        wait(lambda:a.stats['state_posts']>0)
        return a,robot

    def stop_agent(self,a):
        a.stop();next(t for x,t in self.agents if x is a).join(2)

    def mission(self):
        # Current contract: only the live Lease owner with the current epoch may create a mission.
        from fleet_server.auth import Principal
        lease = self.fleet.lease_view('r') or self.fleet.lease_action('r', Principal('op','operator'), 'acquire', 300)
        wait(lambda: self.fleet.safety_context('r') is not None)
        return self.fleet.create_mission('r','navigate_to_pose',GOAL,'key'+str(time.time_ns()),created_by='op',lease_epoch=lease['epoch'])[0]['mission_id']

    def status(self,mid): return self.fleet.mission(mid)['status']

    def tearDown(self):
        for a,t in self.agents:a.stop()
        for a,t in self.agents:t.join(2)
        if self.server:self.stop_server()
        self.db.c.close();self.tmp.cleanup()

    def test_server_restart_retains_owned_handle_without_goal_replay(self):
        a,r=self.agent();mid=self.mission();wait(lambda:self.status(mid)=='EXECUTING')
        self.stop_server();wait(lambda:not a.connected);self.start_server()
        wait(lambda:a.stats['reconnects']>=2 and a.connected)
        # Contract v1 (SAFETY_CONTRACT.md: a delivered command's outcome is never inferred; server restart
        # invalidates prior movement authorization): the mission is fenced, never replayed.
        self.assertEqual(self.status(mid),'RECONCILIATION_REQUIRED');self.assertEqual(r.backend['starts'],1)
        self.assertEqual(self.fleet._deliver('r')['commands'],[])
        # A real terminal result from the retained same-session handle is still accepted (contract: keep actual terminals).
        r.finish();wait(lambda:self.status(mid)=='SUCCEEDED');self.assertEqual(r.backend['starts'],1)

    def test_agent_restart_lost_handle_blocks_reexecution_and_next_mission(self):
        a,r=self.agent();mid=self.mission();wait(lambda:self.status(mid)=='EXECUTING');self.stop_agent(a)
        a2,r2=self.agent(Robot(r.backend),'boot2');wait(lambda:self.status(mid)=='RECONCILIATION_REQUIRED')
        # Contract v1: while reconciliation is required there is no Safety Context, so a new movement mission is
        # refused at creation (stricter than the former queued-but-not-dispatched behaviour).
        with self.assertRaises(ApiError) as refused:
            self.fleet.create_mission('r','navigate_to_pose',GOAL,'next',created_by='op',lease_epoch=(self.fleet.lease_view('r') or {}).get('epoch'))
        self.assertEqual(refused.exception.status,409);self.assertEqual(r.backend['starts'],1)
        self.assertTrue(r.backend['running']);self.assertIsNone(r2.active)
        with self.assertRaises(ApiError):self.fleet.hello('r',{'boot_id':'boot1','protocol_version':2})

    def test_timeout_during_execution_offline_detection_and_recovery(self):
        a,r=self.agent();mid=self.mission();wait(lambda:self.status(mid)=='EXECUTING')
        a.http.block=True;wait(lambda:not a.connected);time.sleep(.25);self.fleet.housekeeping()
        self.assertFalse(self.fleet.robots['r']['online']);self.assertEqual(self.status(mid),'RECONCILIATION_REQUIRED')
        # Contract v1: reconnection does not promote RECONCILIATION_REQUIRED back to EXECUTING.
        a.http.block=False;wait(lambda:a.connected);time.sleep(.2)
        self.assertEqual(self.status(mid),'RECONCILIATION_REQUIRED');self.assertEqual(r.backend['starts'],1)
        self.assertEqual(self.fleet._deliver('r')['commands'],[])

    def test_terminal_ack_lost_then_agent_restart_recovers_durable_result(self):
        a,r=self.agent();mid=self.mission();wait(lambda:self.status(mid)=='EXECUTING')
        a.http.drop_ack=True;r.finish();wait(lambda:not a.connected);self.stop_agent(a)
        a2,r2=self.agent(Robot(r.backend),'boot2');wait(lambda:self.status(mid)=='SUCCEEDED')
        self.assertEqual(r.backend['starts'],1)

    def test_cancel_during_outage_not_success_until_actual_canceled_result(self):
        a,r=self.agent();mid=self.mission();wait(lambda:self.status(mid)=='EXECUTING')
        self.fleet.cancel_mission(mid);wait(lambda:bool(r.pending));self.assertEqual(self.status(mid),'CANCELING')
        a.http.block=True;wait(lambda:not a.connected);r.finish('CANCELED')
        self.assertNotEqual(self.status(mid),'CANCELED')
        a.http.block=False;wait(lambda:self.status(mid)=='CANCELED');self.assertEqual(r.backend['starts'],1)
        c=self.db.one("SELECT * FROM commands WHERE mission_id=? AND type='cancel_mission'",(mid,));self.assertEqual(c['status'],'SUCCEEDED')

    def test_restart_while_canceling_does_not_report_fake_cancel_success(self):
        a,r=self.agent();mid=self.mission();wait(lambda:self.status(mid)=='EXECUTING');self.fleet.cancel_mission(mid);wait(lambda:bool(r.pending));self.stop_agent(a)
        a2,r2=self.agent(Robot(r.backend),'boot2');wait(lambda:self.status(mid)=='RECONCILIATION_REQUIRED')
        self.assertTrue(r.backend['running']);self.assertEqual(r.backend['starts'],1)

    def test_same_command_repeated_executes_once_and_ownership_is_checked(self):
        a,r=self.agent();mid=self.mission();wait(lambda:self.status(mid)=='EXECUTING')
        c=self.db.one("SELECT * FROM commands WHERE mission_id=?",(mid,))
        cmd={'command_id':c['command_id'],'robot_id':'r','mission_id':mid,'type':'navigate_to_pose','owner_boot_id':a.boot_id,'seq':c['seq'],'issued_at':c['created_at'],'expires_at':c['expires_at'],'payload':{'goal':GOAL}}
        for _ in range(5):a._on_command(cmd)
        self.assertEqual(r.backend['starts'],1)
        with self.assertRaises(ApiError):self.fleet.ack_command('r',c['command_id'],'SUCCEEDED',agent_boot_id='wrong-boot')

    def test_unknown_delivered_command_is_fenced_after_reconnect(self):
        hello={'protocol_version':2,'safety_contract_version':1,'boot_id':'boot1'}
        self.fleet.hello('r',hello);self.fleet.reconcile('r','boot1',{}, {},True)
        self.fleet.ingest_state('r',1,Robot().snapshot(),'boot1')
        mid=self.mission();self.fleet.ingest_state('r',2,Robot().snapshot(),'boot1')   # delivered, no ACK
        self.fleet.hello('r',hello);self.fleet.reconcile('r','boot1',{}, {},True)
        self.assertEqual(self.status(mid),'RECONCILIATION_REQUIRED')
        self.assertEqual(self.fleet.ingest_state('r',3,{},'boot1')['commands'],[])

    def test_mission_mismatch_and_stale_state_are_detected(self):
        a,r=self.agent();mid=self.mission();wait(lambda:self.status(mid)=='EXECUTING');self.stop_agent(a)
        bad=r.snapshot();bad['execution']['active']['mission_id']='wrong'
        self.fleet.ingest_state('r',999,bad,a.boot_id);self.assertEqual(self.status(mid),'RECONCILIATION_REQUIRED')
        with self.assertRaises(ApiError):self.fleet.ingest_state('r',998,bad,a.boot_id)

    def test_expired_and_old_sequence_commands_never_reach_handler(self):
        a,r=self.agent();self.stop_agent(a)
        def command(cid,seq,age=0,expiry=120):return {'command_id':cid,'robot_id':'r','owner_boot_id':a.boot_id,'mission_id':'m_x','type':'navigate_to_pose','seq':seq,'issued_at':time.time()-age,'expires_at':time.time()+expiry,'payload':{'goal':GOAL}}
        a._on_command(command('expired',1,10,-1));a._on_command(command('old-issued',2,200,10))
        self.assertEqual(r.backend['starts'],0)
        a._on_command(command('new',10));r.finish();a._on_command(command('old-seq',9))
        self.assertEqual(r.backend['starts'],1)
        a2=AgentCore('r','http://x',self.token,r.snapshot,r,{},str(self.root/'state'),log=lambda _:None)
        c=command('old-after-restart',8);c['owner_boot_id']=a2.boot_id;a2._on_command(c)
        self.assertEqual(r.backend['starts'],1)

    def test_corrupt_journal_fails_closed_and_is_preserved(self):
        state=self.root/'broken';state.mkdir();f=state/'r_commands.json';f.write_text('{invalid')
        r=Robot();a=AgentCore('r','http://x',self.token,r.snapshot,r,{},str(state),log=lambda _:None)
        a._on_command({'command_id':'c','type':'navigate_to_pose'})
        self.assertFalse(a.journal_ok);self.assertEqual(r.backend['starts'],0);self.assertEqual(f.read_text(),'{invalid')

    def test_exponential_backoff_is_bounded_and_stop_interruptible(self):
        r=Robot();a=AgentCore('r','http://x',self.token,r.snapshot,r,{},str(self.root/'retry'),period_s=0,log=lambda _:None)
        class BadHttp:
            def post(self,*args,**kwargs):raise TimeoutError('injected')
        delays=[]
        class Event:
            def wait(self,delay):
                if delay:delays.append(delay)
                if len(delays)>=5:a.running=False
            def set(self):pass
        a.http=BadHttp();a.stop_evt=Event();a.run()
        self.assertEqual(delays,[1.,2.,4.,8.,10.]);self.assertFalse(a.connected)

    def test_crash_between_command_and_mission_commit_repairs_status(self):
        a,r=self.agent();mid=self.mission();wait(lambda:self.status(mid)=='EXECUTING');self.stop_agent(a)
        r.finish()  # The mock really completed; only the server's mission update is missing.
        self.db.x("UPDATE commands SET status='SUCCEEDED' WHERE mission_id=?",(mid,))
        self.assertEqual(self.status(mid),'EXECUTING')
        self.fleet.hello('r',{'protocol_version':2,'boot_id':'boot1'})
        self.fleet.reconcile('r','boot1',{}, {},True)
        self.assertEqual(self.status(mid),'SUCCEEDED');self.assertEqual(r.backend['starts'],1)

    def test_unowned_active_mission_blocks_dispatch_and_survives_restart(self):
        a,r=self.agent();self.stop_agent(a)
        unknown={'execution':{'active':{'command_id':'unknown','mission_id':'unknown','goal_handle_available':True}}}
        response=self.fleet.ingest_state('r',999,unknown,a.boot_id)
        self.assertTrue(response['reconciliation_required']);self.assertFalse(response['needs_reconcile'])
        # Contract v1: no Lease or Safety Context while the robot is blocked -> creation refused (was: queued).
        from fleet_server.auth import Principal
        with self.assertRaises(ApiError):
            self.fleet.lease_action('r',Principal('op','operator'),'acquire',60)
        self.assertEqual(self.fleet.missions('r'),[])
        self.stop_server();self.start_server();self.fleet.hello('r',{'protocol_version':2,'boot_id':a.boot_id})
        result=self.fleet.reconcile('r',a.boot_id,{}, {},True)
        self.assertTrue(result['reconciliation_required'])
        self.assertEqual(self.fleet.ingest_state('r',1000,{},a.boot_id)['commands'],[])


if __name__=='__main__':unittest.main(verbosity=2)
