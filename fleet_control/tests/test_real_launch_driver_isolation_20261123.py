"""G2 Phase A: the real launch must isolate the vendor driver behind the Gate (static, no ROS needed) and, in an isolated ROS graph,
a foreign /cmd_vel writer must not reach the driver input. Not a real-robot result."""
import os, re, subprocess, sys, time, unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / 'capstone_ws/src/limo_capstone/launch/limo_integrated_auto_mapping_real.launch.py'
GATE = ROOT / 'capstone_ws/src/limo_capstone/limo_capstone/stall_detector_node.py'


class StaticLaunch(unittest.TestCase):
    s = LAUNCH.read_text()

    def test_driver_input_remapped_away_from_cmd_vel(self):
        self.assertIn("DRIVER_CMD_TOPIC = '/limo_driver/cmd_vel'", self.s)
        block = self.s[self.s.index('limo_base_launch = GroupAction'):self.s.index('# B. YDLidar')]
        self.assertIn("SetRemap(src='cmd_vel', dst=DRIVER_CMD_TOPIC)", block)
        self.assertIn("SetRemap(src='/cmd_vel', dst=DRIVER_CMD_TOPIC)", block)

    def test_gate_is_only_driver_writer_and_locked_by_default(self):
        self.assertIn("'/limo_driver/cmd_vel' if '\", allow_motor_drive, \"' == 'true' else '/cmd_vel_locked'", self.s.replace('"', '"'))
        self.assertRegex(self.s, r"default_value='false'[^)]*\)?[\s\S]{0,400}SAFETY LOCK|allow_motor_drive")
        self.assertNotIn("'/cmd_vel' if '\", allow_motor_drive", self.s)

    def test_no_nav_or_joy_writer_to_gate_input_or_driver(self):
        for m in re.finditer(r"cmd_vel_raw'", self.s):
            ctx = self.s[max(0, m.start() - 80):m.end()]
            self.assertNotIn('SetRemap', ctx)
            self.assertNotIn("('cmd_vel', '/cmd_vel_raw')", ctx)
        self.assertIn("'/fleet/nav_cmd_vel'", self.s)

    def test_watchdog_and_gate_validation_in_launch(self):
        self.assertIn("executable='motion_watchdog'", self.s)
        self.assertIn("motion_watchdog,\n        stall_gate", self.s)
        self.assertIn("SetRemap(src='behavior_server:cmd_vel', dst=behavior_output)", self.s)
        self.assertIn("'/cmd_vel_nav'", self.s)
        self.assertIn("'input_publisher_nodes': ['fleet_robot_agent', 'velocity_smoother', 'manual_mapping_teleop']", self.s)
        self.assertIn("'speed_clamp': True", self.s)
        self.assertIn("'mode_gate': True", self.s)
        self.assertIn("('cmd_vel', '/fleet/manual_cmd_vel')", self.s)
        self.assertIn("condition=IfCondition(LaunchConfiguration('start_motion_watchdog'))", self.s)
        self.assertIn("'speed_limit_state_path': '~/.config/limo_fleet/watchdog_speed_limits.json'", self.s)
        for k in ("'input_exclusive_publishers': ['fleet_motion_watchdog']", "'input_writer_status_topic': '/fleet/motion_watchdog/status'",
                  "'exclusive_cmd_vel_topic': gate_out_topic"):
            self.assertIn(k, self.s)


@unittest.skipUnless(os.environ.get('CAPSTONE_PRIVATE_ROS_TEST') == '1', 'needs an isolated ROS network (unshare -Urn)')
class DriverTopicIsolation(unittest.TestCase):
    def test_foreign_cmd_vel_never_reaches_driver_input(self):
        import rclpy
        from geometry_msgs.msg import Twist
        rclpy.init()
        n = rclpy.create_node('fake_driver')
        got = []
        n.create_subscription(Twist, '/limo_driver/cmd_vel', lambda m: got.append((m.linear.x, time.time())), 10)
        gate = subprocess.Popen([sys.executable, str(GATE), '--ros-args', '-r', '/cmd_vel:=/limo_driver/cmd_vel'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        rogue = n.create_publisher(Twist, '/cmd_vel', 10)
        try:
            end = time.time() + 2.5
            while time.time() < end:
                t = Twist(); t.linear.x = 0.15
                rogue.publish(t)
                rclpy.spin_once(n, timeout_sec=0.05)
            self.assertEqual(n.count_subscribers('/cmd_vel'), 0, 'the driver must not subscribe to /cmd_vel')
            self.assertFalse([x for x, _ in got if abs(x) > 1e-9], 'rogue /cmd_vel reached the driver input')
        finally:
            gate.terminate(); gate.wait(5); n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
