#!/usr/bin/env python3
"""Synthetic sysfs fixture unit tests for limo_capstone.usb_device_resolver."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from limo_capstone import usb_device_resolver as udr  # noqa: E402


class SyntheticSysfsFixture:
    """Builds a realistic minimal /dev + /sys USB topology inside a temp dir."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self.dev_root = root / "dev"
        self.sys_root = root / "sys"
        self.pci_usb3 = self.sys_root / "devices" / "pci0000:00" / "0000:00:14.0" / "usb3"
        self.usb_bus_dir = self.sys_root / "bus" / "usb" / "devices"
        self.tty_class_dir = self.sys_root / "class" / "tty"
        for d in (self.dev_root, self.pci_usb3, self.usb_bus_dir, self.tty_class_dir):
            d.mkdir(parents=True, exist_ok=True)
        self.config = udr.ResolverConfig(dev_root=self.dev_root, sys_root=self.sys_root)

    def _register_usb_dir(self, usb_dir: Path, vid: str, pid: str, serial: str = "", product: str = "", mfr: str = "") -> None:
        usb_dir.mkdir(parents=True, exist_ok=True)
        (usb_dir / "idVendor").write_text(vid + "\n", encoding="utf-8")
        (usb_dir / "idProduct").write_text(pid + "\n", encoding="utf-8")
        if serial:
            (usb_dir / "serial").write_text(serial + "\n", encoding="utf-8")
        if product:
            (usb_dir / "product").write_text(product + "\n", encoding="utf-8")
        if mfr:
            (usb_dir / "manufacturer").write_text(mfr + "\n", encoding="utf-8")
        bus_link = self.usb_bus_dir / usb_dir.name
        if bus_link.is_symlink() or bus_link.exists():
            bus_link.unlink()
        bus_link.symlink_to(usb_dir)

    def add_hub(self, usb_name: str, vid: str = "1a40", pid: str = "0101", product: str = "USB2.0 HUB") -> Path:
        hub_dir = self.pci_usb3 / usb_name
        self._register_usb_dir(hub_dir, vid=vid, pid=pid, product=product)
        return hub_dir

    def add_serial(
        self,
        tty_name: str,
        usb_name: str,
        parent_dir: Path | None = None,
        vid: str = "10c4",
        pid: str = "ea60",
        serial: str = "0001",
        product: str = "CP2102 USB to UART Bridge Controller",
        mfr: str = "Silicon Labs",
    ) -> Path:
        base_parent = self.pci_usb3 if parent_dir is None else parent_dir
        usb_dir = base_parent / usb_name
        self._register_usb_dir(usb_dir, vid=vid, pid=pid, serial=serial, product=product, mfr=mfr)
        iface_dir = usb_dir / f"{usb_name}:1.0"
        iface_dir.mkdir(parents=True, exist_ok=True)
        tty_entry = self.tty_class_dir / tty_name
        tty_entry.mkdir(parents=True, exist_ok=True)
        dev_link = tty_entry / "device"
        if dev_link.is_symlink() or dev_link.exists():
            dev_link.unlink()
        dev_link.symlink_to(iface_dir)
        dev_node = self.dev_root / tty_name
        dev_node.write_text("", encoding="utf-8")
        return dev_node

    def remove_serial(self, tty_name: str, usb_name: str) -> None:
        dev_node = self.dev_root / tty_name
        if dev_node.exists() or dev_node.is_symlink():
            dev_node.unlink()
        tty_entry = self.tty_class_dir / tty_name / "device"
        if tty_entry.exists() or tty_entry.is_symlink():
            tty_entry.unlink()
        bus_link = self.usb_bus_dir / usb_name
        if bus_link.exists() or bus_link.is_symlink():
            bus_link.unlink()

    def add_symlink(self, link_name: str, target_tty: str) -> None:
        lp = self.dev_root / link_name
        if lp.is_symlink() or lp.exists():
            lp.unlink()
        lp.symlink_to(target_tty)


class TestUsbDeviceResolver(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory(prefix="usb_resolver_test_")
        self.fx = SyntheticSysfsFixture(Path(self.tmp.name))

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def test_01_normal_fixed_symlinks(self) -> None:
        self.fx.add_serial("ttyUSB0", "3-3")
        hub = self.fx.add_hub("3-7", vid="1a40", pid="0101")
        self.fx.add_serial("ttyUSB1", "3-7.4", parent_dir=hub)
        self.fx.add_symlink("ydlidar", "ttyUSB0")
        self.fx.add_symlink("ttylimo", "ttyUSB1")

        res = udr.resolve_once(self.fx.config)
        self.assertTrue(res.ok, res.errors)
        self.assertEqual(res.base.port_name, "ttylimo")
        self.assertEqual(res.base.source, "symlink")
        self.assertEqual(Path(res.base.real_path).name, "ttyUSB1")
        self.assertEqual(res.lidar.preferred_path, str(self.fx.dev_root / "ydlidar"))
        self.assertEqual(res.lidar.source, "symlink")
        self.assertEqual(Path(res.lidar.real_path).name, "ttyUSB0")

    def test_02_swapped_ttyusb_numbers(self) -> None:
        # Base enumerated as ttyUSB0, LiDAR as ttyUSB1 (no symlinks)
        hub = self.fx.add_hub("3-7", vid="1a40", pid="0101")
        self.fx.add_serial("ttyUSB0", "3-7.4", parent_dir=hub)
        self.fx.add_serial("ttyUSB1", "3-3")

        res = udr.resolve_once(self.fx.config)
        self.assertTrue(res.ok, res.errors)
        self.assertEqual(res.base.port_name, "ttyUSB0")
        self.assertEqual(res.lidar.port_name, "ttyUSB1")
        self.assertEqual(res.base.source, "topology")
        self.assertEqual(res.lidar.source, "topology")

    def test_03_no_fixed_symlinks(self) -> None:
        self.fx.add_serial("ttyUSB0", "3-3")
        hub = self.fx.add_hub("3-7", vid="1a40", pid="0101")
        self.fx.add_serial("ttyUSB1", "3-7.4", parent_dir=hub)

        res = udr.resolve_once(self.fx.config)
        self.assertTrue(res.ok, res.errors)
        self.assertEqual(res.base.port_name, "ttyUSB1")
        self.assertEqual(res.lidar.preferred_path, str(self.fx.dev_root / "ttyUSB0"))

    def test_04_physical_usb_port_changed(self) -> None:
        # Both devices plugged into different physical USB ports (no udev symlinks)
        self.fx.add_serial("ttyUSB2", "3-6")
        hub = self.fx.add_hub("3-5", vid="1a40", pid="0101")
        self.fx.add_serial("ttyUSB3", "3-5.2", parent_dir=hub)

        res = udr.resolve_once(self.fx.config)
        self.assertTrue(res.ok, res.errors)
        self.assertEqual(res.base.port_name, "ttyUSB3")
        self.assertEqual(res.base.device.usb_name, "3-5.2")
        self.assertEqual(res.lidar.port_name, "ttyUSB2")
        self.assertEqual(res.lidar.device.usb_name, "3-6")

    def test_05_identical_vid_pid_serial_without_hub_disambiguation_fails_closed(self) -> None:
        # Two identical CP2102 devices both directly on root hub (no 1a40:0101 hub)
        self.fx.add_serial("ttyUSB0", "3-3")
        self.fx.add_serial("ttyUSB1", "3-4")

        res = udr.resolve_once(self.fx.config)
        self.assertFalse(res.ok)
        self.assertIn(udr.ERR_AMBIGUOUS_CP2102_CANDIDATES, res.errors)
        self.assertIsNone(res.base)
        self.assertIsNone(res.lidar)

    def test_06_extra_third_cp2102_device_fails_closed(self) -> None:
        self.fx.add_serial("ttyUSB0", "3-3")
        hub = self.fx.add_hub("3-7", vid="1a40", pid="0101")
        self.fx.add_serial("ttyUSB1", "3-7.4", parent_dir=hub)
        self.fx.add_serial("ttyUSB2", "3-8")  # extra unknown CP2102

        res = udr.resolve_once(self.fx.config)
        self.assertFalse(res.ok)
        self.assertIn(udr.ERR_AMBIGUOUS_CP2102_CANDIDATES, res.errors)

    def test_07_duplicate_parent_hub_model_fails_closed(self) -> None:
        self.fx.add_serial("ttyUSB0", "3-3")
        hub1 = self.fx.add_hub("3-7", vid="1a40", pid="0101")
        self.fx.add_hub("3-8", vid="1a40", pid="0101")  # second identical hub model
        self.fx.add_serial("ttyUSB1", "3-7.4", parent_dir=hub1)

        res = udr.resolve_once(self.fx.config)
        self.assertFalse(res.ok)
        self.assertIn(udr.ERR_AMBIGUOUS_CP2102_CANDIDATES, res.errors)

    def test_08_wrong_or_colliding_fixed_symlinks_fail_closed(self) -> None:
        self.fx.add_serial("ttyUSB0", "3-3")
        hub = self.fx.add_hub("3-7", vid="1a40", pid="0101")
        self.fx.add_serial("ttyUSB1", "3-7.4", parent_dir=hub)

        # Case A: swapped symlinks (ttylimo -> LiDAR, ydlidar -> Base)
        self.fx.add_symlink("ttylimo", "ttyUSB0")
        self.fx.add_symlink("ydlidar", "ttyUSB1")
        res_swap = udr.resolve_once(self.fx.config)
        self.assertFalse(res_swap.ok)
        self.assertIn(udr.ERR_AMBIGUOUS_CP2102_CANDIDATES, res_swap.errors)

        # Case B: both symlinks point to same device
        self.fx.add_symlink("ttylimo", "ttyUSB0")
        self.fx.add_symlink("ydlidar", "ttyUSB0")
        res_col = udr.resolve_once(self.fx.config)
        self.assertFalse(res_col.ok)
        self.assertIn(udr.ERR_PORT_COLLISION, res_col.errors)

    def test_09_base_hub_not_enumerated(self) -> None:
        # Exact current real LIMO state: only YDLIDAR present, 1a40:0101 hub absent
        self.fx.add_serial("ttyUSB0", "3-3")
        self.fx.add_symlink("ydlidar", "ttyUSB0")

        res = udr.resolve_once(self.fx.config)
        self.assertFalse(res.ok)
        self.assertIn(udr.ERR_BASE_HUB_NOT_ENUMERATED, res.errors)
        self.assertIsNone(res.base)

    def test_10_base_hub_present_but_base_port_missing(self) -> None:
        self.fx.add_serial("ttyUSB0", "3-3")
        self.fx.add_hub("3-7", vid="1a40", pid="0101")

        res = udr.resolve_once(self.fx.config)
        self.assertFalse(res.ok)
        self.assertIn(udr.ERR_BASE_PORT_NOT_FOUND, res.errors)

    def test_11_delayed_enumeration_bounded_retry(self) -> None:
        self.fx.add_serial("ttyUSB0", "3-3")
        clock = [0.0]

        def fake_mono() -> float:
            return clock[0]

        def fake_sleep(dt: float) -> None:
            clock[0] += dt
            if clock[0] >= 1.0 and not (self.fx.dev_root / "ttyUSB1").exists():
                hub = self.fx.add_hub("3-7", vid="1a40", pid="0101")
                self.fx.add_serial("ttyUSB1", "3-7.4", parent_dir=hub)

        res = udr.resolve_with_retry(
            self.fx.config,
            timeout_s=3.0,
            interval_s=0.5,
            sleep_fn=fake_sleep,
            monotonic_fn=fake_mono,
        )
        self.assertTrue(res.ok, res.errors)
        self.assertEqual(res.details["attempts"], 3)
        self.assertEqual(res.base.port_name, "ttyUSB1")

        # Also confirm bounded timeout does NOT loop forever when device never appears
        fx2 = SyntheticSysfsFixture(Path(self.tmp.name) / "timeout_case")
        fx2.add_serial("ttyUSB0", "3-3")
        c2 = [0.0]
        res_to = udr.resolve_with_retry(
            fx2.config,
            timeout_s=1.0,
            interval_s=0.5,
            sleep_fn=lambda dt: c2.__setitem__(0, c2[0] + dt),
            monotonic_fn=lambda: c2[0],
        )
        self.assertFalse(res_to.ok)
        self.assertIn(udr.ERR_BASE_HUB_NOT_ENUMERATED, res_to.errors)
        self.assertEqual(res_to.details["attempts"], 3)

    def test_12_permission_denied_fails_fast(self) -> None:
        self.fx.add_serial("ttyUSB0", "3-3")
        hub = self.fx.add_hub("3-7", vid="1a40", pid="0101")
        self.fx.add_serial("ttyUSB1", "3-7.4", parent_dir=hub)

        def deny_base(p: Path) -> bool:
            return p.name != "ttyUSB1"

        res = udr.resolve_with_retry(
            self.fx.config,
            timeout_s=5.0,
            interval_s=0.5,
            access_checker=deny_base,
        )
        self.assertFalse(res.ok)
        self.assertIn(udr.ERR_PORT_PERMISSION_DENIED, res.errors)
        # Non-transient error must fail fast on attempt 1
        self.assertEqual(res.details["attempts"], 1)

    def test_13_disconnect_reconnect_and_device_changed_before_start(self) -> None:
        self.fx.add_serial("ttyUSB0", "3-3")
        hub = self.fx.add_hub("3-7", vid="1a40", pid="0101")
        self.fx.add_serial("ttyUSB1", "3-7.4", parent_dir=hub)

        initial = udr.resolve_once(self.fx.config)
        self.assertTrue(initial.ok)
        # Re-verify succeeds when unchanged
        self.assertTrue(udr.verify_before_start(initial, self.fx.config).ok)

        # Simulate disconnect + re-enumeration on a different tty number before driver start
        self.fx.remove_serial("ttyUSB1", "3-7.4")
        re_disc = udr.verify_before_start(initial, self.fx.config)
        self.assertFalse(re_disc.ok)
        self.assertIn(udr.ERR_DEVICE_CHANGED_BEFORE_START, re_disc.errors)

        self.fx.add_serial("ttyUSB2", "3-7.4", parent_dir=hub)
        re_moved = udr.verify_before_start(initial, self.fx.config)
        self.assertFalse(re_moved.ok)
        self.assertIn(udr.ERR_DEVICE_CHANGED_BEFORE_START, re_moved.errors)

    def test_14_cli_json_output_and_exit_codes(self) -> None:
        script = PKG_ROOT / "limo_capstone" / "usb_device_resolver.py"
        # Missing hub -> exit 2
        self.fx.add_serial("ttyUSB0", "3-3")
        p_fail = subprocess.run(
            [sys.executable, str(script), "--dev-root", str(self.fx.dev_root), "--sys-root", str(self.fx.sys_root), "--json"],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(p_fail.returncode, 2)
        d_fail = json.loads(p_fail.stdout)
        self.assertFalse(d_fail["ok"])
        self.assertIn(udr.ERR_BASE_HUB_NOT_ENUMERATED, d_fail["errors"])

        # Add base hub + port -> exit 0
        hub = self.fx.add_hub("3-7", vid="1a40", pid="0101")
        self.fx.add_serial("ttyUSB1", "3-7.4", parent_dir=hub)
        p_ok = subprocess.run(
            [sys.executable, str(script), "--dev-root", str(self.fx.dev_root), "--sys-root", str(self.fx.sys_root), "--json"],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(p_ok.returncode, 0, msg=p_ok.stderr)
        d_ok = json.loads(p_ok.stdout)
        self.assertTrue(d_ok["ok"])
        self.assertEqual(d_ok["base"]["port_name"], "ttyUSB1")
        self.assertEqual(d_ok["lidar"]["port_name"], "ttyUSB0")


if __name__ == "__main__":
    unittest.main(verbosity=2)
