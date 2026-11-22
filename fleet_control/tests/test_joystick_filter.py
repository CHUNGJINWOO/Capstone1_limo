import math
import unittest
from limo_robot_agent.joystick_filter import JoystickConfig, joystick_command

class JoystickFilterTests(unittest.TestCase):
    def setUp(self): self.c = JoystickConfig(1, 0, 0, .15, .4, .25)
    def cmd(self, axes=(.5, 1.), buttons=(1,), **kw):
        options = dict(received_at=1., now=1.1, manual_enabled=True); options.update(kw)
        return joystick_command(self.c, axes, buttons, **options)
    def test_forward_and_turn(self): self.assertEqual(self.cmd(), (.15, .2, 'active'))
    def test_deadman_and_mode(self):
        self.assertEqual(self.cmd(buttons=(0,))[:2], (0., 0.))
        self.assertEqual(self.cmd(manual_enabled=False)[:2], (0., 0.))
    def test_timeout_and_backward_clock(self):
        self.assertEqual(self.cmd(now=1.25)[2], 'stale')
        self.assertEqual(self.cmd(now=.9)[2], 'stale')
    def test_invalid_and_disconnect(self):
        for axes in ((), (math.nan, 0), (0, math.inf), (0, 1.1), (True, 0)):
            self.assertEqual(self.cmd(axes=axes)[:2], (0., 0.))
        self.assertEqual(self.cmd(buttons=())[:2], (0., 0.))
    def test_neutral(self): self.assertEqual(self.cmd(axes=(0., 0.))[:2], (0., 0.))
if __name__ == '__main__': unittest.main()
