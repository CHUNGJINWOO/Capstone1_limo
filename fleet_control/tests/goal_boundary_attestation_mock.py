"""Phase 1.2 in-memory contract model, NO ROS or production authentication."""
import threading
from dataclasses import dataclass

@dataclass(frozen=True)
class Attestation:
    gate_boot: str
    fault_epoch: int
    challenge: str
    role: str
    age_s: float
    writer_ids: tuple
    action_servers: tuple
    owned_terminal: tuple
    external_terminal: tuple
    unknown_goal_basis: bool
    mac_verified: bool  # synthetic test-only flag: NOT a cryptographic implementation

class MockGate:
    def __init__(self,boot,epoch,challenge,writers,required_action_servers=('navigate_to_pose',)):
        self.boot=boot;self.epoch=epoch;self.challenge=challenge;self.writers=writers;self.required_action_servers=set(required_action_servers);self.used=set()
    def verify(self,a,*,current_active,required_goals,status_seen,now_age_limit=5):
        if not isinstance(a,Attestation) or a.role not in ('AGENT','OPERATION_MANAGER') or a.mac_verified is not True:return False
        if (a.gate_boot,a.fault_epoch)!=(self.boot,self.epoch) or a.challenge!=self.challenge or a.challenge in self.used:return False
        if type(a.age_s) not in (int,float) or not 0<=a.age_s<=now_age_limit:return False
        if set(a.writer_ids)!=set(self.writers) or not self.required_action_servers.issubset(a.action_servers):return False
        if current_active or (a.role=='AGENT' and (a.external_terminal or not status_seen)):return False
        if not status_seen and a.unknown_goal_basis is not True:return False
        entries=a.owned_terminal+a.external_terminal
        all_terminal={}
        for item in entries:
            if not isinstance(item,tuple) or len(item)!=4:return False
            uuid,status,kind,age=item
            if not isinstance(uuid,str) or not uuid or uuid in all_terminal:return False
            if status not in ('SUCCEEDED','CANCELED','ABORTED') or kind not in ('RESULT_RESPONSE','STATUS_TERMINAL','CANCEL_RESPONSE_TERMINATED'):return False
            if type(age) not in (int,float) or not 0<=age<=now_age_limit:return False
            all_terminal[uuid]=status
        if any(u not in all_terminal for u in required_goals):return False
        self.used.add(a.challenge);return True

class MockBoundary:
    def __init__(self):
        self.lock=threading.Lock();self.active=None;self.seen=set();self.reconciliation=False;self.cancel=[];self.process_exited=False
    def submit(self,owner,uuid):
        with self.lock:
            if self.reconciliation or self.active is not None or uuid in self.seen:raise ValueError('occupied or stale')
            self.seen.add(uuid);self.active={'owner':owner,'uuid':uuid,'accepted':False,'terminal':None,'stop':False};return uuid
    def stop(self):
        with self.lock:
            if self.active is None:return
            self.active['stop']=True
            if self.active['accepted'] and self.active['uuid'] not in self.cancel:self.cancel.append(self.active['uuid'])
    def accept(self,uuid):
        with self.lock:
            if not self.active or self.active['uuid']!=uuid:return False
            self.active['accepted']=True
            if self.active['stop'] or self.reconciliation:
                if uuid not in self.cancel:self.cancel.append(uuid)
                self.reconciliation=True
            return True
    def terminal(self,uuid,status):
        with self.lock:
            if not self.active or self.active['uuid']!=uuid or status not in ('SUCCEEDED','CANCELED','ABORTED'):return False
            if self.active['terminal'] is not None:return self.active['terminal']==status
            self.active['terminal']=status;return True
    def process_exit(self):
        with self.lock:
            self.process_exited=True
            if self.active and not self.active['terminal']:self.reconciliation=True
    def restart(self):
        with self.lock:self.reconciliation=True
