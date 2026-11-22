import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'agent/limo_robot_agent'))
from limo_robot_agent.real_locked_policy import require_available,BLOCK_REASON

class Tests(unittest.TestCase):
 def test_real_locked_modes_and_flag_always_blocked(self):
  for mode,flag in [('real_locked',False),('REAL-LOCKED',False),('real',True),('sim',True)]:
   with self.subTest(mode=mode,flag=flag):
    with self.assertRaisesRegex(RuntimeError,'no independently verified physical motor isolation'):
     require_available(mode=mode,requested=flag)
  with self.assertRaisesRegex(RuntimeError,'no independently verified physical motor isolation'):
   require_available(mode='real',requested=False,recording_enabled=True)
 def test_existing_modes_are_not_reclassified(self):
  for mode in ('sim','real','mock'):
   self.assertIsNone(require_available(mode=mode,requested=False))
if __name__=='__main__':unittest.main()
