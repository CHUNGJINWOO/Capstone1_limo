"""SIMULATION CONTRACT ONLY: no ROS node, no deployment, no motor output.

The mock fence and ordered status stream are assumed capabilities absent from
current staging. These tests do not prove that an actual ROS graph is fenced.
"""
import math
import secrets
import threading
import unittest

ACTIONS = ('navigate_to_pose', 'follow_path')
TERMINAL = frozenset(('SUCCEEDED', 'ABORTED', 'CANCELED'))

class Denied(Exception): pass

class MockActionServer:
    def __init__(self, broker, action):
        self.broker, self.action = broker, action
        self.seq, self.active = 0, set()
    def snapshot(self):
        self.seq += 1
        self.broker.status(self.action, self.seq, set(self.active), self.broker.now)
    def submit(self, uid, owner='AUTO'):
        self.broker.submit(owner, self.action, uid)
        self.active.add(uid)
        self.snapshot()
    def cancel_accepted(self, uid):
        if uid not in self.active: raise ValueError(uid)
        # No terminal result: cancel acceptance changes no ownership.
    def finish(self, uid, result='SUCCEEDED'):
        self.active.remove(uid)
        self.broker.result(self.action, uid, result)
        self.snapshot()

class Contract:
    """Single-process reference model of an existing Agent slot's intended behavior."""
    def __init__(self):
        self.lock = threading.RLock(); self.now=0.; self.max_age=1.
        self.server_boot='server-a'; self.agent_boot='agent-a'; self.lease_epoch=1
        self.lease_owner='op'; self.lease_until=100.; self.fence=False
        self.gate=False; self.watchdog=False; self.mode='UNKNOWN'; self.session=None
        self.mapping='IDLE'; self.goals={}; self.obs={a:None for a in ACTIONS}
        self.invalid=set(); self.highwater={a:0 for a in ACTIONS}; self.generation=0
    def _revoke(self):
        self.session=None; self.generation+=1
        if self.mode == 'MANUAL': self.mode='UNKNOWN'
    def configure_fence(self):
        # Stand-in for authenticated, exclusive control of EVERY goal submitter,
        # including Nav2 child actions. Current staging does not supply this.
        self.fence=True; self.gate=True; self.watchdog=True
    def status(self, action, seq, active, observed_at):
        with self.lock:
            previous=self.obs[action]
            if (type(seq) is not int or seq < 1 or not math.isfinite(observed_at)
                or not 0 <= observed_at <= self.now
                or seq <= self.highwater[action]
                or (previous and observed_at < previous[1])):
                self.invalid.add(action); self._revoke(); return False
            known={uid for (a,uid), result in self.goals.items() if a == action and result is None}
            if active != known:
                self.invalid.add(action); self._revoke(); return False
            self.highwater[action]=seq
            self.obs[action]=(seq, observed_at, frozenset(active))
            return True
    def result(self, action, uid, terminal):
        with self.lock:
            key=(action,uid)
            if key not in self.goals or self.goals[key] is not None or terminal not in TERMINAL:
                self.invalid.add(action); self._revoke(); return False
            self.goals[key]=terminal
            # Require a subsequent ordered snapshot after the verified result.
            self.obs[action]=None
            return True
    def submit(self, owner, action, uid):
        with self.lock:
            if (not self.fence or owner not in ('AUTO','MAPPING') or action not in ACTIONS
                or self.mode == 'MANUAL' or self.session is not None or (action,uid) in self.goals
                or (self.mode in ('AUTO','MAPPING') and self.mode != owner)):
                raise Denied('slot')
            self.mode=owner; self.goals[(action,uid)]=None
            self.obs[action]=None
    def idle(self):
        return (self.fence and self.mapping=='IDLE' and not self.invalid and
                all(v is not None for v in self.goals.values()) and
                all(self.obs[a] is not None and not self.obs[a][2] and
                    0 <= self.now-self.obs[a][1] <= self.max_age for a in ACTIONS))
    def start(self, owner, epoch):
        with self.lock:
            if (owner != self.lease_owner or epoch != self.lease_epoch or self.now >= self.lease_until
                or not self.gate or not self.watchdog or self.session is not None or not self.idle()):
                raise Denied('admission')
            self.mode='MANUAL'; self.generation+=1
            self.session={'id':secrets.token_hex(16),'ticket':secrets.token_hex(16),
                          'seq':-1,'owner':owner,'epoch':epoch,'server':self.server_boot,
                          'agent':self.agent_boot,'gen':self.generation}
            return dict(self.session)
    def sample(self, sid, ticket, seq, owner='op', epoch=1, linear=.1, turn=.2):
        with self.lock:
            s=self.session
            if (not s or self.now >= self.lease_until or not self.gate or not self.watchdog
                or not self.idle() or self.server_boot != s['server'] or self.agent_boot != s['agent']
                or self.lease_epoch != s['epoch'] or self.lease_owner != s['owner']):
                self._revoke(); raise Denied('revoked')
            if (sid != s['id'] or ticket != s['ticket'] or type(seq) is not int or seq <= s['seq']
                or owner != s['owner'] or epoch != s['epoch']
                or any(type(x) not in (int,float) or not math.isfinite(x) or abs(x)>1 for x in (linear,turn))): raise Denied('invalid')
            s['seq']=seq; s['ticket']=secrets.token_hex(16)
            return s['ticket']
    def restart(self, side):
        with self.lock:
            if side=='server': self.server_boot += 'x'
            else: self.agent_boot += 'x'
            self._revoke(); self.fence=False; self.mode='UNKNOWN'
            self.obs={a:None for a in ACTIONS}; self.highwater={a:0 for a in ACTIONS}; self.invalid=set()
            # Outstanding goals remain indeterminate, even if the new server is empty.
    def offline(self):
        with self.lock:
            self._revoke(); self.fence=False; self.obs={a:None for a in ACTIONS}
    def change_lease(self):
        with self.lock: self.lease_epoch+=1; self._revoke()

class ContractTests(unittest.TestCase):
    def setUp(self):
        self.c=Contract(); self.c.configure_fence()
        self.nav=MockActionServer(self.c,'navigate_to_pose'); self.fp=MockActionServer(self.c,'follow_path')
        self.nav.snapshot(); self.fp.snapshot()
    def start(self): return self.c.start('op',self.c.lease_epoch)
    def test_idle_and_nonzero_sample_success(self):
        s=self.start(); self.assertEqual(self.c.mode,'MANUAL')
        self.assertNotEqual(self.c.sample(s['id'],s['ticket'],1),s['ticket'])
    def test_both_actions_require_terminal_and_fresh_snapshot(self):
        self.nav.submit('nav1'); self.fp.submit('fp1')
        with self.assertRaises(Denied): self.start()
        self.nav.cancel_accepted('nav1')
        with self.assertRaises(Denied): self.start()
        self.nav.finish('nav1','CANCELED')
        with self.assertRaises(Denied): self.start()
        self.fp.finish('fp1','SUCCEEDED'); self.assertEqual(self.start()['epoch'],1)
    def test_active_and_mapping_block(self):
        self.c.mapping='MAPPING'
        with self.assertRaises(Denied): self.start()
        self.c.mapping='UNKNOWN'
        with self.assertRaises(Denied): self.start()
        self.c.mapping='IDLE'; self.nav.submit('nav1')
        with self.assertRaises(Denied): self.start()
    def test_concurrent_goal_and_manual_exclusive(self):
        for i in range(100):
            self.setUp(); barrier=threading.Barrier(3); out=[]
            def manual():
                barrier.wait()
                try: self.start(); out.append('manual')
                except Denied: pass
            def goal():
                barrier.wait()
                try: self.nav.submit('g'+str(i)); out.append('goal')
                except Denied: pass
            threads=[threading.Thread(target=manual),threading.Thread(target=goal)]
            for t in threads:t.start()
            barrier.wait()
            for t in threads:t.join(2)
            self.assertEqual(len(out),1,out)
    def test_missing_stale_duplicate_and_out_of_order(self):
        self.c.obs['follow_path']=None
        with self.assertRaises(Denied): self.start()
        self.fp.snapshot(); self.c.now=2.
        with self.assertRaises(Denied): self.start()
        self.nav.snapshot(); self.fp.snapshot()
        self.assertFalse(self.c.status('follow_path',self.fp.seq,set(),self.c.now))
        with self.assertRaises(Denied):self.start()
        self.setUp();self.nav.snapshot()
        self.assertFalse(self.c.status('navigate_to_pose',1,set(),self.c.now))
        with self.assertRaises(Denied):self.start()
    def test_old_snapshot_cannot_erase_active_goal(self):
        self.nav.submit('g')
        self.assertFalse(self.c.status('navigate_to_pose',self.nav.seq+1,set(),self.c.now))
        with self.assertRaises(Denied):self.start()
    def test_old_sequence_after_result_is_rejected(self):
        self.nav.submit('g'); old=self.nav.seq; self.nav.finish('g')
        self.assertFalse(self.c.status('navigate_to_pose',old,set(),self.c.now))
        with self.assertRaises(Denied): self.start()
    def test_modes_are_mutually_exclusive(self):
        self.nav.submit('g','AUTO')
        with self.assertRaises(Denied): self.fp.submit('f','MAPPING')
    def test_invalid_axes(self):
        s=self.start()
        for x in (float('nan'),float('inf'),-float('inf'),1.1,True):
            with self.assertRaises(Denied): self.c.sample(s['id'],s['ticket'],1,linear=x)
        self.c.sample(s['id'],s['ticket'],1,linear=.1)
    def test_restarts_and_lease_expiry_kill_old_credentials(self):
        for side in ('server','agent'):
            self.setUp();s=self.start();self.c.restart(side)
            with self.assertRaises(Denied):self.c.sample(s['id'],s['ticket'],0)
            with self.assertRaises(Denied):self.start()
        self.setUp();s=self.start();self.c.now=101
        with self.assertRaises(Denied):self.c.sample(s['id'],s['ticket'],0)
        self.setUp();s=self.start();self.c.change_lease()
        with self.assertRaises(Denied):self.c.sample(s['id'],s['ticket'],0)
    def test_same_boot_reconnect_requires_fresh_session(self):
        s=self.start();self.c.offline();self.c.configure_fence();self.nav.snapshot();self.fp.snapshot()
        new=self.start();self.assertNotEqual(s['id'],new['id'])
        with self.assertRaises(Denied):self.c.sample(s['id'],s['ticket'],0)
        self.c.sample(new['id'],new['ticket'],0)
    def test_old_result_cannot_resolve_new_goal(self):
        self.nav.submit('one');self.nav.finish('one');self.nav.submit('two')
        self.assertFalse(self.c.result('navigate_to_pose','one','CANCELED'))
        with self.assertRaises(Denied):self.start()
    def test_ticket_sequence_and_id_replay(self):
        s=self.start(); ticket=self.c.sample(s['id'],s['ticket'],1)
        for sid,tok,seq in ((s['id'],s['ticket'],2),(s['id'],ticket,1),('old',ticket,2),(s['id'],ticket,True)):
            with self.assertRaises(Denied):self.c.sample(sid,tok,seq)
        self.c.sample(s['id'],ticket,2)
    def test_gate_and_watchdog_fault_revoke(self):
        for name in ('gate','watchdog'):
            self.setUp();s=self.start();setattr(self.c,name,False)
            with self.assertRaises(Denied):self.c.sample(s['id'],s['ticket'],0)
            self.assertIsNone(self.c.session)
    def test_external_bypass_not_covered(self):
        self.c.fence=False
        with self.assertRaises(Denied): self.start()

if __name__=='__main__':unittest.main(verbosity=2)
