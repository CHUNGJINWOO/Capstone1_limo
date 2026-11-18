"""Verify the arbiter's locked selection and the separate Nav2/manual inputs."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agent/limo_robot_agent'))
from limo_robot_agent.velocity_arbiter import select_locked


def test_lock_deadman_and_terminal():
    manual = dict(linear=.1, turn=.2, deadman=True)
    assert select_locked('MANUAL', locked=True, nav=(.1, .1), manual=manual, terminal=True) == (.1, .2)
    assert select_locked('MANUAL', locked=True, nav=(.1, .1), manual=manual, terminal=False) == (0., 0.)
    assert select_locked('MANUAL', locked=True, nav=None, manual=dict(manual, deadman=False), terminal=True) == (0., 0.)
    assert select_locked('AUTO', locked=False, nav=(1, 1), manual=manual, terminal=True) == (0., 0.)
    assert select_locked('AUTO', locked=True, nav=None, manual=manual, terminal=True) == (0., 0.)
    assert select_locked('AUTO', locked=True, nav=(1, 1), manual=manual, terminal=False) == (.1, .3)
