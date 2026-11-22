"""Actual locked selector; no ROS graph or motor output."""
import math
import unittest
from limo_robot_agent.velocity_arbiter import select_locked

class Selector(unittest.TestCase):
 def setUp(self):self.m={'deadman':True,'expires':11.,'requested_linear':.1,'requested_turn':.15}
 def pick(self, allowed=True):return select_locked(allowed=allowed,manual=self.m,now=10.)
 def test_nonzero_and_angular_directions(self):
  self.assertEqual(self.pick(),(.1,.15));self.m['requested_turn']=-.15;self.assertEqual(self.pick(),(.1,-.15))
 def test_deadman_gate_and_expiry(self):
  self.assertEqual(self.pick(False),(0.,0.));self.m['deadman']=False;self.assertEqual(self.pick(),(0.,0.))
  self.m['deadman']=True;self.m['expires']=10.;self.assertEqual(self.pick(),(0.,0.))
 def test_invalid_values_fail_closed(self):
  for key,values in {'expires':(None,math.nan,math.inf),'requested_linear':(-.01,.11,True),'requested_turn':(-.31,.31,math.nan)}.items():
   for v in values:
    with self.subTest(key=key,value=v):
     self.setUp();self.m[key]=v;self.assertEqual(self.pick(),(0.,0.))
if __name__=='__main__':unittest.main(verbosity=2)
