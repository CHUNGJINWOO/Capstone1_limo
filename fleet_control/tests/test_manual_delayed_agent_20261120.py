"""Current Agent bench: a response arriving after its TTL is never fresh.

No ROS nodes, sockets, Fleet Server, or physical robot are created.
"""
import threading
import time
import unittest
from types import SimpleNamespace
from limo_robot_agent.manual_web_bench import ManualWebBench

class Timer:
 def cancel(self): pass
class Robot:
 def __init__(self):
  self.lock=threading.Lock();self.active=None
  self.gate={'state':'NORMAL','wheel_odom_stale':False,'cmd_blocked':False}
  self.published=[]
 def create_timer(self,*_):return Timer()
 def _age(self,_):return 0.01
 def create_publisher(self,*_):
  outer=self
  class Pub:
   def publish(self,m):outer.published.append((m.linear.x,m.angular.z))
  return Pub()
 def destroy_publisher(self,_):pass
class DelayedHTTP:
 def post(self,*_,**__):
  time.sleep(.32)
  return {'sample':{'linear':1.,'turn':1.,'deadman':True,'session_id':'test','seq':1,'ttl_s':.25}}
class TestLateAgentResponse(unittest.TestCase):
 def test_late_response_cannot_become_fresh(self):
  robot=Robot();agent=SimpleNamespace(is_execution_allowed=lambda:True,http=DelayedHTTP(),robot_id='test',boot_id='boot')
  bench=ManualWebBench(robot,agent);bench._graph_blocker=lambda:None
  try:
   deadline=time.monotonic()+1
   while time.monotonic()<deadline:
    with bench.lock: arrived=isinstance(bench.sample,dict)
    if arrived:break
    time.sleep(.01)
   self.assertTrue(arrived,'test response did not arrive')
   bench.tick();self.assertEqual(bench.state['status'],'input_expired')
   self.assertFalse(bench.state['motor_output']);self.assertIsNone(bench.publisher)
   self.assertEqual((bench.state['requested_linear'],bench.state['requested_turn']),(0.,0.))
  finally:bench.close()

if __name__=='__main__':unittest.main(verbosity=2)
