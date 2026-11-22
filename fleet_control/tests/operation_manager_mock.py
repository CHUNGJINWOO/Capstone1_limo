"""Pure in-memory design model for tests ONLY; no server, ROS or process control."""
from dataclasses import dataclass

STATES={'IDLE','STARTING_MAPPING','MAPPING','STOPPING_MAPPING','LOADING_MAP','NAVIGATION','PROFILE_SWITCHING','UNKNOWN','RECONCILIATION_REQUIRED'}
ROUTES={('IDLE','start_mapping'):'STARTING_MAPPING',('MAPPING','stop_mapping'):'STOPPING_MAPPING',
        ('IDLE','load_map'):'LOADING_MAP',('IDLE','switch_profile'):'PROFILE_SWITCHING',
        ('NAVIGATION','switch_profile'):'PROFILE_SWITCHING'}
COMPLETIONS={'STARTING_MAPPING':'MAPPING','STOPPING_MAPPING':'IDLE','LOADING_MAP':'NAVIGATION','PROFILE_SWITCHING':'IDLE'}

@dataclass(frozen=True)
class Evidence:
    boot: str
    fault_epoch: int
    map_epoch: int
    lease_epoch: int
    healthy: bool
    nav_idle_proven: bool
    external_goal: bool
    fresh: bool
    agent_boot: str = 'mock-agent'

class MockOperations:
    def __init__(self):
        self.state='UNKNOWN';self.revision=0;self.fence=None;self.pending=None;self.requests={};self.events=[]
    def _change(self,state,reason):
        assert state in STATES
        self.state=state;self.revision+=1;self.events.append((self.revision,state,reason))
    def reconcile(self,evidence,*,role):
        if role!='admin' or self.state not in ('UNKNOWN','RECONCILIATION_REQUIRED') or not self._safe(evidence):raise ValueError('reconciliation denied')
        self.fence=evidence;self.pending=None;self._change('IDLE','explicit operator reconciliation')
    @staticmethod
    def _safe(e):
        return isinstance(e,Evidence) and e.healthy is True and e.fresh is True and e.nav_idle_proven is True and e.external_goal is False and isinstance(e.boot,str) and bool(e.boot.strip()) and isinstance(e.agent_boot,str) and bool(e.agent_boot.strip()) and all(type(x) is int and 0<=x<2**63 for x in (e.fault_epoch,e.map_epoch,e.lease_epoch))
    def request(self,action,key,evidence,*,principal,role,expected_revision):
        if not isinstance(key,str) or not key or not isinstance(principal,str) or not principal:raise ValueError('identity')
        prior=self.requests.get((principal,key))
        if prior:
            if prior[0]!=(action,expected_revision):raise ValueError('idempotency conflict')
            return prior[1]
        if role not in ('operator','admin') or type(expected_revision) is not int or expected_revision!=self.revision:raise ValueError('role or revision')
        if not self._safe(evidence) or evidence!=self.fence:raise ValueError('safety or lease mismatch')
        target=ROUTES.get((self.state,action))
        if not target:raise ValueError('transition denied')
        self.pending=(principal,key,action,evidence);self._change(target,'requested '+action)
        self.requests[(principal,key)]=((action,expected_revision),self.revision)
        return self.revision
    def complete(self,evidence,*,proof):
        if not self.pending or not self._safe(evidence) or evidence!=self.pending[3] or proof is not True:
            self.pending=None;self._change('RECONCILIATION_REQUIRED','completion unverified');return
        target=COMPLETIONS[self.state];self.pending=None;self._change(target,'mock completion evidence')
    def fault_or_restart(self,reason):
        self.pending=None;self.fence=None;self._change('RECONCILIATION_REQUIRED',reason)
