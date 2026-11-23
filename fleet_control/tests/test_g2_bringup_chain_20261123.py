"""G2 (2026-11-23): static velocity-chain check of capstone_ws/src/limo_fleet_bringup (origin/main bringup, R2 watchdog wired).

Needs ROS 2 Humble `launch` / `launch_ros` (skipped without them; a skip is NOT a pass). Nothing is launched: the Node
descriptions are only built and their remappings resolved. Expected chain:
  controller_server / behavior_server -> /fleet/cmd_vel_nav -> velocity_smoother -> /fleet/nav_cmd_vel (or, with the Agent's
  VelocityArbiter, /fleet/cmd_vel_nav_smoothed -> Arbiter -> /fleet/nav_cmd_vel) -> fleet_motion_watchdog -> /cmd_vel_raw ->
  stall_detector_node (Gate) -> /cmd_vel_locked.  No node may be remapped onto /cmd_vel or write /cmd_vel_raw except the watchdog.
"""
import importlib
import sys
import unittest
from pathlib import Path

BRINGUP = Path(__file__).resolve().parents[2] / 'capstone_ws' / 'src' / 'limo_fleet_bringup'
try:
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    import launch_ros  # noqa: F401
    HAVE_LAUNCH = True
except ImportError:
    HAVE_LAUNCH = False


def _text(value):
    return value if isinstance(value, str) else perform_substitutions(LaunchContext(), value)


def remaps(node):
    return {_text(a): _text(b) for a, b in node._Node__remappings}


def name(node):
    return _text(node._Node__node_name)


@unittest.skipUnless(HAVE_LAUNCH, 'ROS 2 launch/launch_ros not available (source Humble)')
class BringupChain(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(BRINGUP))
        cls.nav = importlib.import_module('limo_fleet_bringup.navigation')
        cls.safety = importlib.import_module('limo_fleet_bringup.safety')
        cls.tele = importlib.import_module('limo_fleet_bringup.teleoperation')
        cls.mapping = importlib.import_module('limo_fleet_bringup.mapping')

    def _nodes(self, arbiter):
        from ament_index_python.packages import PackageNotFoundError
        try:
            nav = self.nav.navigation({'use_sim_time': False}, arbiter=arbiter)
        except PackageNotFoundError as exc:    # limo_capstone share directory not installed in this environment
            self.skipTest(f'limo_capstone share not available: {exc}')
        return nav + [self.safety.motion_watchdog({'use_sim_time': False}), self.safety.locked_gate({'use_sim_time': False}, False)]

    def _gate_params(self):
        captured, orig = {}, self.safety.Node
        self.safety.Node = lambda **kw: captured.update(kw)
        try:
            self.safety.locked_gate({'use_sim_time': False}, False)
        finally:
            self.safety.Node = orig
        merged = {}
        for d in captured['parameters']:
            merged.update(d)
        return merged

    def test_chain_goes_through_the_watchdog(self):
        for arbiter in (False, True):
            with self.subTest(arbiter=arbiter):
                nodes = {name(n): remaps(n) for n in self._nodes(arbiter)}
                self.assertEqual(nodes['controller_server'].get('cmd_vel'), '/fleet/cmd_vel_nav')
                self.assertEqual(nodes['behavior_server'].get('cmd_vel'), '/fleet/cmd_vel_nav')
                smoother = nodes['velocity_smoother']
                self.assertEqual(smoother.get('cmd_vel'), '/fleet/cmd_vel_nav')
                self.assertEqual(smoother.get('cmd_vel_smoothed'), '/fleet/cmd_vel_nav_smoothed' if arbiter else '/fleet/nav_cmd_vel')
                self.assertIn('fleet_motion_watchdog', nodes)
                self.assertEqual(nodes['fleet_motion_watchdog'], {})          # defaults: /fleet/nav_cmd_vel -> /cmd_vel_raw
                self.assertEqual(nodes['stall_detector_node'], {'/cmd_vel': '/cmd_vel_locked'})
                gate_params = self._gate_params()
                self.assertEqual(gate_params['input_exclusive_publishers'], ['fleet_motion_watchdog'])      # the only approved writer
                self.assertEqual(gate_params['input_writer_status_topic'], '/fleet/motion_watchdog/status')
                self.assertGreater(gate_params['input_max_abs_linear_x'], .15); self.assertGreater(gate_params['input_max_abs_angular_z'], .5)  # above the Nav2 maxima
                for n, m in nodes.items():
                    self.assertNotIn('/cmd_vel_raw', m.values(), f'{n} must not write the Gate input')
                    self.assertNotIn('/cmd_vel', m.values(), f'{n} must not be remapped onto /cmd_vel')

    def test_agent_parameters_match_the_v5_agent(self):
        self.assertEqual(self.tele.input_parameters(True, True)['manual_locked_arbiter'], True)
        self.assertIs(self.tele.input_parameters(False, False)['require_motion_watchdog'], True)
        self.assertEqual(self.mapping.worker_parameters(True, False)['web_mapping_enabled'], True)
        self.assertEqual(self.mapping.worker_parameters(False, True)['isolated_ros_bag'], True)

    def test_watchdog_defaults_match_the_chain(self):
        src = (Path(__file__).resolve().parents[1] / 'agent' / 'limo_robot_agent' / 'limo_robot_agent' / 'topics.py').read_text()
        self.assertIn("NAV_CMD_VEL = '/fleet/nav_cmd_vel'", src)
        self.assertIn("CMD_VEL_RAW = '/cmd_vel_raw'", src)
        self.assertIn("NAV_SMOOTHED = '/fleet/cmd_vel_nav_smoothed'", src)


if __name__ == '__main__':
    unittest.main()
