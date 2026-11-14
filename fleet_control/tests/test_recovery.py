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
        a = self.active
        return {'execution':{'active':({'mission_id':a[0]['mission_id'], 'command_id':a[0]['command_id'], 'goal_handle_available':True} if a else None),
                             'pending_cancel_ids':[cmd['command_id'] for cmd,ack in self.pending]}}

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
        return self.fleet.create_mission('r','navigate_to_pose',GOAL,'key'+str(time.time_ns()))[0]['mission_id']

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
        self.assertEqual(self.status(mid),'EXECUTING');self.assertEqual(r.backend['starts'],1)
        r.finish();wait(lambda:self.status(mid)=='SUCCEEDED')

    def test_agent_restart_lost_handle_blocks_reexecution_and_next_mission(self):
        a,r=self.agent();mid=self.mission();wait(lambda:self.status(mid)=='EXECUTING');self.stop_agent(a)
        a2,r2=self.agent(Robot(r.backend),'boot2');wait(lambda:self.status(mid)=='RECONCILIATION_REQUIRED')
        next_mid=self.mission();time.sleep(.1)
        self.assertEqual(self.status(next_mid),'QUEUED');self.assertEqual(r.backend['starts'],1)
        self.assertTrue(r.backend['running']);self.assertIsNone(r2.active)
        with self.assertRaises(ApiError):self.fleet.hello('r',{'boot_id':'boot1','protocol_version':2})

    def test_timeout_during_execution_offline_detection_and_recovery(self):
        a,r=self.agent();mid=self.mission();wait(lambda:self.status(mid)=='EXECUTING')
        a.http.block=True;wait(lambda:not a.connected);time.sleep(.25);self.fleet.housekeeping()
        self.assertFalse(self.fleet.robots['r']['online']);self.assertEqual(self.status(mid),'RECONCILIATION_REQUIRED')
        a.http.block=False;wait(lambda:a.connected and self.status(mid)=='EXECUTING')
        self.assertEqual(r.backend['starts'],1)

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
        self.fleet.hello('r',{'protocol_version':2,'boot_id':'boot1'});self.fleet.reconcile('r','boot1',{}, {},True)
        mid=self.mission();self.fleet.ingest_state('r',1,{},'boot1')
        self.fleet.hello('r',{'protocol_version':2,'boot_id':'boot1'});self.fleet.reconcile('r','boot1',{}, {},True)
        self.assertEqual(self.status(mid),'RECONCILIATION_REQUIRED')
        self.assertEqual(self.fleet.ingest_state('r',2,{},'boot1')['commands'],[])

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
        mid=self.mission();self.assertEqual(self.status(mid),'QUEUED')
        self.stop_server();self.start_server();self.fleet.hello('r',{'protocol_version':2,'boot_id':a.boot_id})
        result=self.fleet.reconcile('r',a.boot_id,{}, {},True)
        self.assertTrue(result['reconciliation_required'])
        self.assertEqual(self.fleet.ingest_state('r',1000,{},a.boot_id)['commands'],[])


if __name__=='__main__':unittest.main(verbosity=2)
