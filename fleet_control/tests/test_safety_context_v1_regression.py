"""Legacy server regressions with a v1-capable synthetic state fixture."""
import unittest
import test_safety_regression_fixture as old

class RegressionV1(old.Regression):
 def setUp(self):
  super().setUp()
  self.f.robots['r']['agent']['safety_contract_version']=1
  self.state['safety_gate']['boot_id']='gate1'
  self.state['operation_revision']=1
  self.state['localization_status']='CONFIRMED'
  self.f.robots['r']['state']=self.state

if __name__=='__main__': unittest.main(verbosity=2)
