"""ROS stub fixtures: real-mode block and journal-before-send invariants."""
import time, types, unittest
from unittest.mock import patch
import test_agent_safety_v1 as f

class Tests(unittest.TestCase):
 def test_real_without_attestation_is_not_permitted(self):
  robot=f._make_ready_robot();robot.mode='real';robot.simulated=False
  self.assertFalse(robot.snapshot()['safety_gate']['movement_permitted'])
  ok,status,reason,_=robot._verify_nav_preconditions(f._make_cmd(sc=f._valid_safety_context()))
  self.assertFalse(ok);self.assertEqual(status,'REJECTED');self.assertIn('GATE_ATTESTATION_UNAVAILABLE',reason)
 def test_sim_records_intent_before_send_and_terminal(self):
  robot=f._make_ready_robot();robot.mode='sim';robot.simulated=True;records=[];acks=[]
  def persist(cid,**data):
   if data.get('submission_intent'):self.assertEqual(len(robot.nav.sent_goals),0)
   records.append((cid,data))
  robot.on_command_meta=persist
  robot.start_navigate(f._make_cmd(sc=f._valid_safety_context()),lambda s,d='',pr=None:acks.append((s,d)))
  robot._run_tasks();self.assertEqual(len(robot.nav.sent_goals),1)
  fut=robot.nav.sent_goals[0][2]; reserved=fut.requested_goal_uuid.hex()
  # The agent reserves the goal UUID before sending; the action server answers with that same UUID
  # (StubFuture copies the requested UUID into the handle, as a real ActionServer does).
  handle=f.StubGoalHandle(uuid_hex='11'*16,accepted=True);fut.set_result(handle)
  self.assertEqual(bytes(handle.goal_id.uuid).hex(),reserved)
  handle.result_future.set_result(types.SimpleNamespace(status=f.GoalStatus.STATUS_SUCCEEDED))
  intents=[v for _,v in records if v.get('submission_intent')]
  self.assertTrue(intents);self.assertEqual(intents[0].get('goal_uuid'),reserved)       # intent journaled before send
  terminals=[v for _,v in records if v.get('terminal_source')=='nav2_action_result']
  self.assertEqual(len(terminals),1);self.assertEqual(terminals[0].get('goal_uuid'),reserved)
  self.assertEqual(terminals[0].get('terminal_status'),'SUCCEEDED')
  self.assertEqual(acks[-1][0],'SUCCEEDED')
 def test_journal_failure_blocks_send(self):
  robot=f._make_ready_robot();robot.mode='sim';robot.simulated=True;acks=[]
  def fail(*args,**kwargs):raise OSError('simulated failure')
  robot.on_command_meta=fail
  robot.start_navigate(f._make_cmd(sc=f._valid_safety_context()),lambda s,d='',pr=None:acks.append((s,d)))
  robot._run_tasks();self.assertEqual(len(robot.nav.sent_goals),0);self.assertEqual(acks[-1][0],'RECONCILIATION_REQUIRED')
 def test_stale_acceptance_does_not_replace_new_active(self):
  robot=f._make_ready_robot();robot.mode='sim';robot.simulated=True;acks=[]
  robot.on_command_meta=lambda *args,**kwargs:None
  robot.start_navigate(f._make_cmd(sc=f._valid_safety_context()),lambda s,d='',pr=None:acks.append((s,d)))
  robot._run_tasks();future=robot.nav.sent_goals[0][2]
  newer={'command_id':'newer'};robot.active=newer
  handle=f.StubGoalHandle(uuid_hex='22'*16,accepted=True);future.set_result(handle)
  self.assertIs(robot.active,newer);self.assertEqual(handle.cancel_calls,1)
  self.assertEqual(acks[-1][0],'RECONCILIATION_REQUIRED')
 def test_terminal_journal_failure_keeps_goal_blocked(self):
  robot=f._make_ready_robot();robot.mode='sim';robot.simulated=True;acks=[]
  def persist(cid,**data):
   if data.get('terminal_source'):raise OSError('simulated terminal journal failure')
  robot.on_command_meta=persist
  robot.start_navigate(f._make_cmd(sc=f._valid_safety_context()),lambda s,d='',pr=None:acks.append((s,d)))
  robot._run_tasks();handle=f.StubGoalHandle(uuid_hex='33'*16,accepted=True);robot.nav.sent_goals[0][2].set_result(handle)
  handle.result_future.set_result(types.SimpleNamespace(status=f.GoalStatus.STATUS_SUCCEEDED))
  self.assertIsNotNone(robot.active);self.assertTrue(robot.active['reconciliation_required'])
  self.assertEqual(acks[-1][0],'RECONCILIATION_REQUIRED')
 def test_stale_rejection_does_not_resolve_old_command(self):
  robot=f._make_ready_robot();robot.mode='sim';robot.simulated=True;acks=[]
  robot.on_command_meta=lambda *args,**kwargs:None
  robot.start_navigate(f._make_cmd(sc=f._valid_safety_context()),lambda s,d='',pr=None:acks.append((s,d)))
  robot._run_tasks();future=robot.nav.sent_goals[0][2];newer={'command_id':'newer'};robot.active=newer
  count=len(acks);future.set_result(f.StubGoalHandle(uuid_hex='44'*16,accepted=False))
  self.assertIs(robot.active,newer);self.assertEqual(len(acks),count)
 def test_expiry_boundary_is_closed(self):
  robot=f._make_ready_robot();robot.mode='sim';robot.simulated=True
  now=time.time();sc=f._valid_safety_context(issued_at=now-1,expires_at=now)
  cmd=f._make_cmd(sc=sc,issued_at=now-1,expires_at=now+5)
  with patch.object(f.safety_protocol.time,'time',return_value=now):
   ok,status,_,_=robot._verify_nav_preconditions(cmd)
  self.assertFalse(ok);self.assertEqual(status,'EXPIRED')

if __name__=='__main__':unittest.main(verbosity=2)
