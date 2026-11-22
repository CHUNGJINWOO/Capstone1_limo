"""Independent fault-boundary tests against the actual VelocityArbiter, no ROS graph."""
import threading,time,unittest
from types import SimpleNamespace
from limo_robot_agent.velocity_arbiter import VelocityArbiter

class GateBoundary(unittest.TestCase):
 def setUp(self):
  self.gate={'state':'NORMAL','cmd_blocked':False,'wheel_odom_stale':False}
  self.r=SimpleNamespace(gate=self.gate,lock=threading.Lock(),_age=lambda _:0.01,watchdog_ok=lambda:True)
  self.a=VelocityArbiter.__new__(VelocityArbiter);self.a.robot=self.r;self.a.graph_locked=lambda:True
  self.a.agent=SimpleNamespace(is_execution_allowed=lambda:True,session_snapshot=lambda:{'connected':True,'reconciliation_required':False,'lease':{'epoch':1,'expires_at':time.time()+10}})
 def test_pending_allowed_only_when_unblocked(self):
  self.gate['state']='STALL_PENDING';self.assertIsNone(self.a.common_blocker())
  for value in (True,None):
   self.gate['cmd_blocked']=value;self.assertEqual(self.a.common_blocker(),'GATE_NOT_NORMAL')
 def test_confirmed_stall_and_stale_sensor_block(self):
  for state in ('STALL_LATCHED','FAILSAFE_SENSOR_TIMEOUT','FAILSAFE_ICP_DEGRADED'):
   self.gate['state']=state;self.assertEqual(self.a.common_blocker(),'GATE_NOT_NORMAL')
  self.gate['state']='NORMAL';self.gate['wheel_odom_stale']=True;self.assertEqual(self.a.common_blocker(),'ODOM_STALE')
 def test_watchdog_and_disconnection_block(self):
  self.r.watchdog_ok=lambda:False;self.assertEqual(self.a.common_blocker(),'WATCHDOG_NOT_AUTHORIZED')
  self.r.watchdog_ok=lambda:True;self.a.agent.is_execution_allowed=lambda:False
  self.assertEqual(self.a.common_blocker(),'AGENT_EXECUTION_NOT_ALLOWED')
if __name__=='__main__':unittest.main(verbosity=2)
