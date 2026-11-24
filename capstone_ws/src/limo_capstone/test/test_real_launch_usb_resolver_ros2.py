#!/usr/bin/env python3
"""Real ROS 2 Humble LaunchService + Mock driver integration tests for USB Device Resolver.

Verifies that limo_integrated_auto_mapping_real.launch.py's OpaqueFunction and
SetParameter pass the resolved port parameters to mock limo_base and ydlidar
nodes while preserving /limo_driver/cmd_vel remapping and fail-closed behavior.
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path
from unittest import mock

try:
    import ament_index_python.packages  # noqa: F401
    import launch  # noqa: F401
    import rclpy  # noqa: F401
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

PKG_ROOT = Path(__file__).resolve().parents[1]
REAL_LAUNCH = PKG_ROOT / "launch" / "limo_integrated_auto_mapping_real.launch.py"
sys.path.insert(0, str(PKG_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from limo_capstone import usb_device_resolver as udr  # noqa: E402
from test_usb_device_resolver import SyntheticSysfsFixture  # noqa: E402


def _write_mock_driver_pkg(prefix_dir: Path) -> Dict[str, Path]:
    """Create minimal mock limo_base & ydlidar_ros2_driver packages in a temp AMENT_PREFIX_PATH."""
    share_root = prefix_dir / "share"
    idx_dir = share_root / "ament_index" / "resource_index" / "packages"
    idx_dir.mkdir(parents=True, exist_ok=True)
    limo_base_share = share_root / "limo_base"
    ydlidar_share = share_root / "ydlidar_ros2_driver"
    limo_base_lib = prefix_dir / "lib" / "limo_base"
    ydlidar_lib = prefix_dir / "lib" / "ydlidar_ros2_driver"
    for d in (limo_base_share / "launch", ydlidar_share / "launch", ydlidar_share / "params", limo_base_lib, ydlidar_lib):
        d.mkdir(parents=True, exist_ok=True)
    (idx_dir / "limo_base").write_text("", encoding="utf-8")
    (idx_dir / "ydlidar_ros2_driver").write_text("", encoding="utf-8")

    mock_base_py = limo_base_lib / "limo_base"
    mock_base_py.write_text(
        textwrap.dedent(
            """\
            #!/usr/bin/env python3
            import json, os
            from pathlib import Path
            import rclpy
            from rclpy.node import Node
            from geometry_msgs.msg import Twist

            def main():
                rclpy.init()
                node = Node('mock_limo_base')
                node.declare_parameter('port_name', 'ttylimo')
                node.declare_parameter('pub_odom_tf', False)
                sub = node.create_subscription(Twist, 'cmd_vel', lambda _m: None, 10)
                out = Path(os.environ['MOCK_OUT_DIR']) / 'limo_base.json'
                out.write_text(json.dumps({
                    'port_name': node.get_parameter('port_name').value,
                    'pub_odom_tf': node.get_parameter('pub_odom_tf').value,
                    'cmd_vel_topic': sub.topic_name,
                }), encoding='utf-8')
                node.destroy_node()
                rclpy.shutdown()

            if __name__ == '__main__':
                main()
            """
        ),
        encoding="utf-8",
    )
    mock_base_py.chmod(0o755)

    mock_lidar_py = ydlidar_lib / "ydlidar_ros2_driver_node"
    mock_lidar_py.write_text(
        textwrap.dedent(
            """\
            #!/usr/bin/env python3
            import json, os
            from pathlib import Path
            import rclpy
            from rclpy.node import Node

            def main():
                rclpy.init()
                node = Node('ydlidar_ros2_driver_node')
                node.declare_parameter('port', '/dev/ydlidar')
                out = Path(os.environ['MOCK_OUT_DIR']) / 'ydlidar.json'
                out.write_text(json.dumps({
                    'port': node.get_parameter('port').value,
                }), encoding='utf-8')
                node.destroy_node()
                rclpy.shutdown()

            if __name__ == '__main__':
                main()
            """
        ),
        encoding="utf-8",
    )
    mock_lidar_py.chmod(0o755)

    (limo_base_share / "launch" / "limo_base.launch.py").write_text(
        textwrap.dedent(
            """\
            from launch import LaunchDescription
            from launch.actions import DeclareLaunchArgument
            from launch.substitutions import LaunchConfiguration
            from launch_ros.actions import Node

            def generate_launch_description():
                return LaunchDescription([
                    DeclareLaunchArgument('port_name', default_value='ttylimo'),
                    DeclareLaunchArgument('pub_odom_tf', default_value='False'),
                    Node(
                        package='limo_base',
                        executable='limo_base',
                        name='limo_base',
                        namespace='limo',
                        parameters=[{
                            'port_name': LaunchConfiguration('port_name'),
                            'pub_odom_tf': LaunchConfiguration('pub_odom_tf'),
                        }],
                        output='screen',
                    ),
                ])
            """
        ),
        encoding="utf-8",
    )

    (ydlidar_share / "params" / "ydlidar.yaml").write_text(
        "ydlidar_ros2_driver_node:\n  ros__parameters:\n    port: /dev/ydlidar\n",
        encoding="utf-8",
    )
    (ydlidar_share / "launch" / "ydlidar.launch.py").write_text(
        textwrap.dedent(
            f"""\
            import os
            from launch import LaunchDescription
            from launch.actions import DeclareLaunchArgument
            from launch.substitutions import LaunchConfiguration
            from launch_ros.actions import Node

            def generate_launch_description():
                param_file = LaunchConfiguration('params_file')
                return LaunchDescription([
                    DeclareLaunchArgument(
                        'params_file',
                        default_value=os.path.join('{ydlidar_share}', 'params', 'ydlidar.yaml'),
                    ),
                    Node(
                        package='ydlidar_ros2_driver',
                        executable='ydlidar_ros2_driver_node',
                        name='ydlidar_ros2_driver_node',
                        namespace='/',
                        parameters=[param_file],
                        output='screen',
                    ),
                ])
            """
        ),
        encoding="utf-8",
    )
    return {"limo_base": limo_base_share, "ydlidar_ros2_driver": ydlidar_share}


@unittest.skipUnless(ROS2_AVAILABLE, "ROS 2 Humble launch/rclpy packages unavailable")
class TestRealLaunchUsbResolverRos2(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory(prefix="launch_usb_ros2_")
        self.root = Path(self.tmp.name)
        self.fx = SyntheticSysfsFixture(self.root / "sysfs_fx")
        self.prefix_dir = self.root / "prefix"
        self.shares = _write_mock_driver_pkg(self.prefix_dir)
        self.out_dir = self.root / "out"
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self._old_out = os.environ.get("MOCK_OUT_DIR")
        self._old_ament = os.environ.get("AMENT_PREFIX_PATH")
        os.environ["MOCK_OUT_DIR"] = str(self.out_dir)
        os.environ["AMENT_PREFIX_PATH"] = f"{self.prefix_dir}:{self._old_ament or ''}"

    def tearDown(self) -> None:
        if self._old_out is None:
            os.environ.pop("MOCK_OUT_DIR", None)
        else:
            os.environ["MOCK_OUT_DIR"] = self._old_out
        if self._old_ament is None:
            os.environ.pop("AMENT_PREFIX_PATH", None)
        else:
            os.environ["AMENT_PREFIX_PATH"] = self._old_ament
        self.tmp.cleanup()

    def _run_launch_serial_section(self, use_usb_resolver: str = "true") -> tuple[int, dict]:
        """Load real launch file's serial resolver OpaqueFunction in real ROS 2 LaunchService."""
        import importlib.util
        import launch
        from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration

        def fake_get_pkg_share(pkg_name: str) -> str:
            if pkg_name in self.shares:
                return str(self.shares[pkg_name])
            return str(PKG_ROOT)

        spec = importlib.util.spec_from_file_location("real_launch_under_test", str(REAL_LAUNCH))
        mod = importlib.util.module_from_spec(spec)
        assert spec and spec.loader

        with mock.patch("ament_index_python.packages.get_package_share_directory", side_effect=fake_get_pkg_share):
            spec.loader.exec_module(mod)
            full_ld = mod.generate_launch_description()

        resolver_opaque = None
        usb_decls = []
        for ent in full_ld.entities:
            if isinstance(ent, DeclareLaunchArgument) and ent.name.startswith(("use_usb_resolver", "usb_resolve_")):
                usb_decls.append(ent)
            if isinstance(ent, OpaqueFunction):
                fn = getattr(ent, "_OpaqueFunction__function", None)
                if getattr(fn, "__name__", "") == "resolve_serial_drivers":
                    resolver_opaque = ent
        self.assertIsNotNone(resolver_opaque, "resolve_serial_drivers OpaqueFunction not found in real launch")

        serial_ld = launch.LaunchDescription(
            [
                *usb_decls,
                SetLaunchConfiguration("use_usb_resolver", use_usb_resolver),
                SetLaunchConfiguration("usb_resolve_timeout_sec", "0.0"),
                SetLaunchConfiguration("usb_resolve_interval_sec", "0.05"),
                resolver_opaque,
            ]
        )

        for f in self.out_dir.glob("*.json"):
            f.unlink()

        def fx_cfg(*_args, **_kwargs):
            return self.fx.config

        with mock.patch.object(udr, "ResolverConfig", side_effect=fx_cfg):
            ls = launch.LaunchService()
            ls.include_launch_description(serial_ld)
            rc = ls.run()

        results = {}
        for name in ("limo_base", "ydlidar"):
            p = self.out_dir / f"{name}.json"
            if p.exists():
                results[name] = json.loads(p.read_text(encoding="utf-8"))
        return rc, results

    def test_01_normal_fixed_symlinks_launch(self) -> None:
        self.fx.add_serial("ttyUSB0", "3-3")
        hub = self.fx.add_hub("3-7", vid="1a40", pid="0101")
        self.fx.add_serial("ttyUSB1", "3-7.4", parent_dir=hub)
        self.fx.add_symlink("ydlidar", "ttyUSB0")
        self.fx.add_symlink("ttylimo", "ttyUSB1")

        rc, out = self._run_launch_serial_section("true")
        self.assertEqual(rc, 0)
        self.assertEqual(out["limo_base"]["port_name"], "ttylimo")
        self.assertEqual(out["limo_base"]["pub_odom_tf"], False)
        self.assertEqual(out["limo_base"]["cmd_vel_topic"], "/limo_driver/cmd_vel")
        self.assertEqual(out["ydlidar"]["port"], str(self.fx.dev_root / "ydlidar"))

    def test_02_changed_ports_without_symlinks_launch(self) -> None:
        self.fx.add_serial("ttyUSB2", "3-6")
        hub = self.fx.add_hub("3-5", vid="1a40", pid="0101")
        self.fx.add_serial("ttyUSB3", "3-5.2", parent_dir=hub)

        rc, out = self._run_launch_serial_section("true")
        self.assertEqual(rc, 0)
        self.assertEqual(out["limo_base"]["port_name"], "ttyUSB3")
        self.assertEqual(out["limo_base"]["cmd_vel_topic"], "/limo_driver/cmd_vel")
        self.assertEqual(out["ydlidar"]["port"], str(self.fx.dev_root / "ttyUSB2"))

    def test_03_missing_base_hub_refuses_to_launch_drivers(self) -> None:
        self.fx.add_serial("ttyUSB0", "3-3")
        rc, out = self._run_launch_serial_section("true")
        self.assertEqual(rc, 0)
        self.assertEqual(out, {}, "Neither limo_base nor ydlidar may launch when hub is missing")

    def test_04_ambiguous_cp2102_refuses_to_launch_drivers(self) -> None:
        self.fx.add_serial("ttyUSB0", "3-3")
        self.fx.add_serial("ttyUSB1", "3-4")
        rc, out = self._run_launch_serial_section("true")
        self.assertEqual(rc, 0)
        self.assertEqual(out, {}, "Neither limo_base nor ydlidar may launch when CP2102 is ambiguous")

    def test_05_use_usb_resolver_false_bypasses_check_with_defaults(self) -> None:
        # Even with missing hub, use_usb_resolver=false falls back to legacy defaults
        rc, out = self._run_launch_serial_section("false")
        self.assertEqual(rc, 0)
        self.assertEqual(out["limo_base"]["port_name"], "ttylimo")
        self.assertEqual(out["limo_base"]["cmd_vel_topic"], "/limo_driver/cmd_vel")
        self.assertEqual(out["ydlidar"]["port"], "/dev/ydlidar")


if __name__ == "__main__":
    unittest.main(verbosity=2)
