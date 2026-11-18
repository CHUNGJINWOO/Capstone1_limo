#!/usr/bin/env python3
"""Actual ROS adapter callbacks, controlled futures; no ROS node or motor is started."""
import sys,threading,time,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from concurrent.futures import Future
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'agent/limo_robot_agent'))
from limo_robot_agent.ros_adapter import RosRobot
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
class Harness:
    _do_cancel=RosRobot._do_cancel
    _request_cancel=RosRobot._request_cancel
    _on_result=RosRobot._on_result
    _on_goal_response=RosRobot._on_goal_response
    _set_mission=RosRobot._set_mission
    _do_nav=RosRobot._do_nav
    def _nav2_goals(self): return dict(publishers=1, active_count=0)
    _on_fb=RosRobot._on_fb
    def __init__(self):self.active=None;self.last_mission={};self.lock=threading.RLock()
class Handle:
    accepted=True
    def __init__(self):
        self.goal_id=NS(uuid=list(range(16)));self.cancel=Future();self.result=Future();self.cancel_calls=0
    def cancel_goal_async(self):self.cancel_calls+=1;return self.cancel
    def get_result_async(self):return self.result
class Tests(unittest.TestCase):
    def setup_goal(self,handle=True):
        r=Harness();nav=[];cancel=[];h=Handle()
        rec={'mission_id':'m','command_id':'nav','handle':h if handle else None,'ack':lambda s,*a:nav.append(s),'cancel_requested':False,'cancel_acks':[],'cancel_sent':False}
        r.active=rec;return r,h,rec,nav,cancel
    def test_service_acceptance_is_not_cancel_completion(self):
        r,h,rec,nav,c=self.setup_goal();r._do_cancel({'mission_id':'m','command_id':'cancel'},lambda s,*a:c.append(s))
        h.cancel.set_result(NS(return_code=0,goals_canceling=[NS(goal_id=h.goal_id)]));self.assertEqual(c,['ACCEPTED'])
        f=Future();f.set_result(NS(status=GoalStatus.STATUS_CANCELED));r._on_result(rec,f)
        self.assertEqual(c,['ACCEPTED','SUCCEEDED']);self.assertEqual(nav,['CANCELED'])
    def test_cancel_of_a_goal_this_agent_saw_finish_is_terminal_not_uncertain(self):
        for st in ('SUCCEEDED','FAILED'):
            r=Harness();r.last_mission={'active_mission_id':'m','state':st};c=[]
            r._do_cancel({'mission_id':'m','command_id':'cancel'},lambda s,d='',*a:c.append((s,d)))
            self.assertEqual(c[0][0],'REJECTED');self.assertTrue(c[0][1].startswith('NOTHING_TO_CANCEL'))
    def test_cancel_of_an_unknown_mission_stays_uncertain(self):
        r=Harness();r.last_mission={'active_mission_id':'other','state':'SUCCEEDED'};c=[]
        r._do_cancel({'mission_id':'m','command_id':'cancel'},lambda s,*a:c.append(s));self.assertEqual(c,['RECONCILIATION_REQUIRED'])
    def test_rejected_cancel_is_uncertain_not_success(self):
        r,h,rec,nav,c=self.setup_goal();r._do_cancel({'mission_id':'m','command_id':'cancel'},lambda s,*a:c.append(s))
        h.cancel.set_result(NS(return_code=1,goals_canceling=[]));self.assertEqual(c,['ACCEPTED','RECONCILIATION_REQUIRED'])
    def test_cancel_targets_its_own_goal_uuid(self):
        r,h,rec,nav,c=self.setup_goal();r._do_cancel({'mission_id':'m','command_id':'cancel'},lambda s,*a:c.append(s))
        h.cancel.set_result(NS(return_code=0,goals_canceling=[NS(goal_id=NS(uuid=[99]*16))]));self.assertEqual(c[-1],'RECONCILIATION_REQUIRED')
    def test_missing_handle_never_claims_success(self):
        r=Harness();out=[];r._do_cancel({'mission_id':'m','command_id':'cancel'},lambda s,*a:out.append(s));self.assertEqual(out,['RECONCILIATION_REQUIRED'])
    def test_goal_completing_before_cancel_is_not_reported_canceled(self):
        r,h,rec,nav,c=self.setup_goal();r._do_cancel({'mission_id':'m','command_id':'cancel'},lambda s,*a:c.append(s))
        f=Future();f.set_result(NS(status=GoalStatus.STATUS_SUCCEEDED));r._on_result(rec,f);self.assertEqual(nav,['SUCCEEDED']);self.assertEqual(c[-1],'REJECTED')
    def test_cancel_before_goal_acceptance_waits_for_result(self):
        r,h,rec,nav,c=self.setup_goal(False);r._do_cancel({'mission_id':'m','command_id':'cancel'},lambda s,*a:c.append(s))
        self.assertEqual(c,['ACCEPTED']);f=Future();f.set_result(h);r._on_goal_response(rec,f);self.assertEqual(h.cancel_calls,1);self.assertEqual(c,['ACCEPTED'])
        h.result.set_result(NS(status=GoalStatus.STATUS_CANCELED));self.assertEqual(c[-1],'SUCCEEDED')
    def test_lost_result_marks_uncertain(self):
        r,h,rec,nav,c=self.setup_goal();r._do_cancel({'mission_id':'m','command_id':'cancel'},lambda s,*a:c.append(s))
        f=Future();f.set_exception(TimeoutError());r._on_result(rec,f);self.assertEqual(nav,['RECONCILIATION_REQUIRED']);self.assertEqual(c[-1],'RECONCILIATION_REQUIRED');self.assertIsNone(rec['handle'])
    def test_rejected_goal_resolves_pending_cancel_as_rejected(self):
        r,h,rec,nav,c=self.setup_goal(False);r._do_cancel({'mission_id':'m','command_id':'cancel'},lambda s,*a:c.append(s))
        h.accepted=False;f=Future();f.set_result(h);r._on_goal_response(rec,f)
        self.assertEqual(nav,['REJECTED']);self.assertEqual(c[-1],'REJECTED');self.assertIsNone(r.active)
    def test_lost_goal_response_marks_pending_cancel_uncertain(self):
        r,h,rec,nav,c=self.setup_goal(False);r._do_cancel({'mission_id':'m','command_id':'cancel'},lambda s,*a:c.append(s))
        f=Future();f.set_exception(TimeoutError());r._on_goal_response(rec,f)
        self.assertEqual(nav,['RECONCILIATION_REQUIRED']);self.assertEqual(c[-1],'RECONCILIATION_REQUIRED')
    def test_unknown_terminal_result_stays_uncertain(self):
        r,h,rec,nav,c=self.setup_goal();f=Future();f.set_result(NS(status=GoalStatus.STATUS_UNKNOWN));r._on_result(rec,f)
        self.assertEqual(nav,['RECONCILIATION_REQUIRED']);self.assertIsNone(rec['handle']);self.assertIs(r.active,rec)
    def test_live_or_ambiguous_foreign_goals_prevent_submission(self):
        for observed in ({'publishers': 1, 'active_count': 1},
                         {'publishers': 2, 'active_count': 0},
                         {'publishers': 0, 'active_count': None}):
            with self.subTest(observed=observed):
                robot = Harness()
                robot.allow_missions = True
                robot.execution_allowed = lambda: True
                robot.get_publishers_info_by_topic = lambda _: []
                robot.nav = NS(wait_for_server=lambda **kwargs: True)
                robot._nav2_goals = lambda: observed
                responses = []
                command = {'mission_id': 'm', 'command_id': 'c', 'expires_at': time.time()+60,
                           'payload': {'goal': {'frame_id': 'map', 'x': 1, 'y': 2}}}
                robot._do_nav(command, lambda status, detail: responses.append((status, detail)))
                self.assertEqual(responses[0][0], 'REJECTED')
                self.assertIn('NAV_GOAL_OWNERSHIP_UNAVAILABLE', responses[0][1])
                self.assertIsNone(robot.active)
    def test_submission_exception_does_not_free_robot_for_next_goal(self):
        r=Harness();out=[];r.allow_missions=True;r.execution_allowed=lambda:True;r.get_publishers_info_by_topic=lambda _:[]
        r.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time()))
        class Nav:
            def wait_for_server(self,**kw):return True
            def send_goal_async(self,*args,**kw):raise TimeoutError('injected submission ambiguity')
        r.nav=Nav();r._do_nav({'mission_id':'m','command_id':'c','expires_at':time.time()+60,'payload':{'goal':{'frame_id':'map','x':1,'y':2}}},lambda s,*a:out.append(s))
        self.assertEqual(out,['RECONCILIATION_REQUIRED']);self.assertIsNotNone(r.active)
    def test_fleet_and_frontier_share_the_first_goal_slot(self):
        robot = Harness()
        robot.allow_missions = True
        robot.execution_allowed = lambda: True
        robot.frontier_allowed = lambda _: True
        robot.get_publishers_info_by_topic = lambda _: []
        robot.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: Time()))
        barrier = threading.Barrier(2, timeout=2)
        sent, replies = [], []
        robot.nav = NS(wait_for_server=lambda **_: barrier.wait() >= 0,
                       send_goal_async=lambda *_, **__: sent.append(True) or Future())

        def submit(owner):
            robot._do_nav({'mission_id': owner, 'command_id': owner, 'goal_owner': owner,
                           'expires_at': time.time() + 5,
                           'payload': {'goal': {'frame_id': 'map', 'x': 1, 'y': 2}}},
                          lambda status, *_: replies.append(status))

        threads = [threading.Thread(target=submit, args=(owner,)) for owner in ('fleet', 'frontier')]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=3)
        self.assertEqual(len(sent), 1)
        self.assertEqual(replies, ['REJECTED'])
        self.assertIn(robot.active['owner'], ('fleet', 'frontier'))

    def test_follow_path_terminal_requires_active_observation_after_reset(self):
        r = Harness()
        r.follow_path_topic = 'follow_path/_action/status'
        r.follow_path_status = None
        r.follow_path_observed_active = False
        r.get_publishers_info_by_topic = lambda _: [NS(node_name='controller_server', endpoint_gid=[1])]
        callback = lambda status: RosRobot._on_follow_path_status(
            r, NS(status_list=[NS(status=status)]))
        callback(GoalStatus.STATUS_CANCELED)
        self.assertFalse(RosRobot.follow_path_idle(r))
        callback(GoalStatus.STATUS_EXECUTING)
        self.assertFalse(RosRobot.follow_path_idle(r))
        callback(GoalStatus.STATUS_CANCELED)
        self.assertTrue(RosRobot.follow_path_idle(r))
        r.follow_path_status = None
        r.follow_path_observed_active = False
        callback(GoalStatus.STATUS_CANCELED)
        self.assertFalse(RosRobot.follow_path_idle(r))

    def test_late_feedback_cannot_overwrite_terminal_display(self):
        r,h,rec,nav,c=self.setup_goal();r.active=None;r.last_mission={'state':'CANCELED'}
        r._on_fb(rec,NS());self.assertEqual(r.last_mission['state'],'CANCELED');self.assertEqual(nav,[])
if __name__=='__main__':unittest.main(verbosity=2)
