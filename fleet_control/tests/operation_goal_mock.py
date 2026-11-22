"""Synthetic ownership and reconciliation model; NEVER used by the real robot."""
from dataclasses import dataclass

OWNERS={'WEB_NAVIGATION','RMF_NAVIGATION','AUTO_MAPPING_EXPLORE','MANUAL_MAPPING'}
TERMINALS={'SUCCEEDED','CANCELED','ABORTED'}

@dataclass(frozen=True)
class Generation:
    agent_boot_id: str
    gate_boot_id: str
    fault_epoch: int
    operation_revision: int
    map_epoch: int
    localization_epoch: int
    def __post_init__(self):
        if not all(isinstance(v,str) and v.strip() for v in (self.agent_boot_id,self.gate_boot_id)) or not all(type(v) is int and 0<=v<2**63 for v in (self.fault_epoch,self.operation_revision,self.map_epoch,self.localization_epoch)):
            raise ValueError('invalid synthetic generation')

@dataclass
class Goal:
    operation_id: str
    command_id: str
    owner_type: str
    owner_id: str
    goal_uuid: str
    generation: Generation
    submitted: bool = False
    accepted: bool = False
    terminal: str | None = None
    process_stopped: bool = False
    handle_owned: bool = True

class GoalModel:
    def __init__(self,generation):
        self.current=generation;self.goal=None;self.state='IDLE';self.blocked=False
        self.stop_keys={};self.commands=set();self.goal_uuids=set();self.cancel_requests=[];self.events=[]
    def submit(self,goal,*,lease_valid,proof_fresh,idle_proven,external_goals):
        if self.blocked or self.goal is not None or lease_valid is not True or proof_fresh is not True or idle_proven is not True or external_goals:
            raise ValueError('blocked, ownership or idle evidence missing')
        if goal.owner_type not in OWNERS or not all(isinstance(x,str) and x for x in (goal.operation_id,goal.command_id,goal.owner_id,goal.goal_uuid)):
            raise ValueError('invalid goal identity')
        if goal.command_id in self.commands or goal.goal_uuid in self.goal_uuids or goal.generation!=self.current:raise ValueError('old command, UUID or generation')
        goal.submitted=True;self.commands.add(goal.command_id);self.goal_uuids.add(goal.goal_uuid);self.goal=goal;self.state='SUBMITTED';self.events.append(('submitted',goal.goal_uuid))
    def acceptance(self,uuid):
        if not self.goal or self.goal.goal_uuid!=uuid:return False
        if self.goal.accepted:return True
        self.goal.accepted=True;self.events.append(('accepted',uuid))
        if self.blocked or self.state in ('STOP_REQUESTED','CANCEL_REQUESTED','RECONCILIATION_REQUIRED'):
            if self.goal.handle_owned:self._cancel()
            self.blocked=True;self.state='RECONCILIATION_REQUIRED'
        else:self.state='ACTIVE'
        return True
    def _cancel(self):
        if self.goal and self.goal.handle_owned and self.goal.goal_uuid not in self.cancel_requests:
            self.cancel_requests.append(self.goal.goal_uuid);self.events.append(('cancel_requested',self.goal.goal_uuid))
    def stop(self,operation_id,*,role,key,current_generation,lease_valid=False):
        # Authenticated safe-direction request does not require a lease.
        if role not in ('operator','admin') or not key or not self.goal or operation_id!=self.goal.operation_id or current_generation!=self.current:
            raise ValueError('invalid stop request')
        if key in self.stop_keys:
            if self.stop_keys[key]!=operation_id:raise ValueError('stop key conflict')
            return
        self.stop_keys[key]=operation_id;self.events.append(('stop_requested',operation_id));self.state='STOP_REQUESTED'
        if self.goal.accepted and self.goal.handle_owned:
            self._cancel();self.state='CANCEL_REQUESTED'
        elif self.goal.accepted:
            self.blocked=True;self.state='RECONCILIATION_REQUIRED'
        # Submission still awaiting acceptance: wait; no fabricated cancellation.
    def cancel_ack(self,uuid,accepted):
        if not self.goal or uuid!=self.goal.goal_uuid or uuid not in self.cancel_requests:return False
        if not accepted:self.blocked=True;self.state='RECONCILIATION_REQUIRED'
        elif not self.blocked:self.state='CANCEL_ACCEPTED'
        self.events.append(('cancel_ack',uuid));return True
    def terminal(self,uuid,status):
        if not self.goal or uuid!=self.goal.goal_uuid or status not in TERMINALS:return False
        if self.goal.terminal:return self.goal.terminal==status
        self.goal.terminal=status;self.events.append(('terminal',uuid,status))
        if not self.blocked:self.state='TERMINAL_CONFIRMED'
        return True
    def finish_stop(self,*,process_stopped,proof_fresh,idle_proven,external_goals):
        if not self.goal or not any(event[0]=='stop_requested' and event[1]==self.goal.operation_id for event in self.events) or not self.goal.terminal or proof_fresh is not True or idle_proven is not True or external_goals:
            self.blocked=True;self.state='RECONCILIATION_REQUIRED';return False
        if self.goal.owner_type in ('AUTO_MAPPING_EXPLORE','MANUAL_MAPPING') and process_stopped is not True:
            self.blocked=True;self.state='RECONCILIATION_REQUIRED';return False
        self.goal.process_stopped=bool(process_stopped)
        if self.blocked:self.state='RECONCILIATION_REQUIRED';return False
        self.state='STOPPED';return True
    def generation_change_or_restart(self,new_generation):
        self.current=new_generation;self.blocked=True;self.state='RECONCILIATION_REQUIRED'
        if self.goal and self.goal.accepted:self._cancel()
        self.events.append(('generation_or_restart',))
    def reconcile(self,*,role,proof_fresh,idle_proven,external_goals,process_stopped):
        if role!='admin' or not self.blocked or proof_fresh is not True or idle_proven is not True or external_goals:
            raise ValueError('reconciliation requires fresh evidence')
        if self.goal and (not self.goal.terminal or (self.goal.owner_type in ('AUTO_MAPPING_EXPLORE','MANUAL_MAPPING') and process_stopped is not True)):
            raise ValueError('terminal or process evidence missing')
        self.events.append(('reconciled',self.goal.goal_uuid if self.goal else None));self.goal=None;self.blocked=False;self.state='IDLE'
