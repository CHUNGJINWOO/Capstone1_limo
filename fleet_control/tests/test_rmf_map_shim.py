"""LD_PRELOAD shim rmf_map_shim: BuildingMap leaves /map, OccupancyGrid stays. Needs ROS 2 Humble + the isolated rmf_demos prefix; skipped otherwise.
Isolated DDS domain with ROS_LOCALHOST_ONLY. Uses rclpy (C typesupport path); the rclcpp path (Gazebo slotcar plugin) is covered by the Gazebo E2E
(experiments/rmf_map_bag_gazebo_20261117_R4m2)."""
import os
from pathlib import Path
import random
import subprocess
import tempfile
import textwrap
import unittest

ROOT = Path(__file__).resolve().parents[2]
SHIM_DIR = ROOT / 'fleet_control/rmf_adapter/rmf_map_shim'
RMF_ENV = Path.home() / 'rmf_demos_humble/env.sh'
HUMBLE = Path('/opt/ros/humble/setup.bash')
PROBE = textwrap.dedent('''
    import sys, time, rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from rmf_building_map_msgs.msg import BuildingMap
    rclpy.init(); n = rclpy.create_node('shim_probe_' + sys.argv[1]); q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    if sys.argv[1] == 'bm_sub': n.create_subscription(BuildingMap, '/map', lambda m: None, q)
    if sys.argv[1] == 'bm_pub': p = n.create_publisher(BuildingMap, 'map', q); p.publish(BuildingMap())
    if sys.argv[1] == 'og_pub': p = n.create_publisher(OccupancyGrid, '/map', q); p.publish(OccupancyGrid())
    t = time.time()
    while time.time() - t < float(sys.argv[2]): rclpy.spin_once(n, timeout_sec=0.1)
''')


@unittest.skipUnless(HUMBLE.exists() and RMF_ENV.exists(), 'ROS 2 Humble and ~/rmf_demos_humble required')
class ShimTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        r = subprocess.run(['bash', str(SHIM_DIR / 'build.sh')], capture_output=True, text=True)
        assert r.returncode == 0, r.stderr
        cls.so = SHIM_DIR / 'build/librmf_map_remap.so'
        cls.tmp = tempfile.TemporaryDirectory(prefix='shim-test-'); cls.probe = Path(cls.tmp.name) / 'probe.py'; cls.probe.write_text(PROBE)

    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()

    def graph(self, preload):
        dom = random.randint(60, 120); pre = f'LD_PRELOAD={self.so} ' if preload else ''
        script = (f'source {HUMBLE}; source {RMF_ENV}; export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID={dom}; '
                  f'({pre}python3 {self.probe} bm_sub 7 & {pre}python3 {self.probe} bm_pub 7 & python3 {self.probe} og_pub 7 & sleep 4; '
                  'ros2 topic list -t; echo ---; ros2 topic info /rmf/building_map -v 2>&1 | grep -E "Publisher count|Subscription count"; wait) 2>&1')
        r = subprocess.run(['bash', '-c', script], capture_output=True, text=True, timeout=60)
        return r.stdout

    def test_without_shim_map_has_two_types(self):
        out = self.graph(False)
        self.assertIn('nav_msgs/msg/OccupancyGrid, rmf_building_map_msgs/msg/BuildingMap', out)

    def test_with_shim_types_are_separated(self):
        out = self.graph(True)
        lines = [l for l in out.splitlines() if l.startswith('/map ') or l.startswith('/rmf/building_map ')]
        self.assertIn('/map [nav_msgs/msg/OccupancyGrid]', lines)
        self.assertIn('/rmf/building_map [rmf_building_map_msgs/msg/BuildingMap]', lines)
        self.assertIn('Publisher count: 1', out); self.assertIn('Subscription count: 1', out)


if __name__ == '__main__':
    unittest.main()
