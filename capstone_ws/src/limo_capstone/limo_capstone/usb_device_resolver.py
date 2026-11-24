#!/usr/bin/env python3
"""Read-only, fail-closed USB serial device resolver for LIMO Base & YDLIDAR.

Operates purely inside user-space (Python standard library only) without
modifying /etc/udev/rules.d, OS USB power settings, EEPROM/firmware, or vendor
driver source code.

Design principles:
- Fixed symlinks (/dev/ttylimo, /dev/ydlidar) are checked FIRST, AND their
  underlying USB topology/role is verified (never blindly trusted).
- Never assigns roles by ttyUSB numbering order alone.
- Never disambiguates identical CP2102 devices by VID/PID/Serial alone.
- Never confirms a role from parent-hub VID/PID alone when multiple hubs or
  extra CP2102 devices make assignment ambiguous.
- Fail-closed on missing hub, missing port, ambiguity, collision, permission
  denial, or pre-start device change.
- Never performs USB reset, never auto-restarts drivers, never re-arms motion.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

ERR_BASE_HUB_NOT_ENUMERATED = "BASE_HUB_NOT_ENUMERATED"
ERR_BASE_PORT_NOT_FOUND = "BASE_PORT_NOT_FOUND"
ERR_LIDAR_PORT_NOT_FOUND = "LIDAR_PORT_NOT_FOUND"
ERR_AMBIGUOUS_CP2102_CANDIDATES = "AMBIGUOUS_CP2102_CANDIDATES"
ERR_PORT_COLLISION = "PORT_COLLISION"
ERR_PORT_PERMISSION_DENIED = "PORT_PERMISSION_DENIED"
ERR_DEVICE_CHANGED_BEFORE_START = "DEVICE_CHANGED_BEFORE_START"

TRANSIENT_ERRORS = frozenset(
    {
        ERR_BASE_HUB_NOT_ENUMERATED,
        ERR_BASE_PORT_NOT_FOUND,
        ERR_LIDAR_PORT_NOT_FOUND,
    }
)

USB_DEV_DIR_RE = re.compile(r"^\d+-\d+(?:\.\d+)*$")
ROOT_HUB_DIR_RE = re.compile(r"^usb\d+$")


@dataclass(frozen=True)
class HubInfo:
    usb_name: str
    vid: str
    pid: str
    product: str = ""


@dataclass(frozen=True)
class SerialDeviceInfo:
    tty_name: str
    dev_path: str
    real_path: str
    usb_name: str
    vid: str
    pid: str
    serial: str
    product: str
    manufacturer: str
    ancestor_hubs: Tuple[HubInfo, ...]
    readable_writable: bool

    def has_ancestor_hub(self, vid: str, pid: str) -> bool:
        v, p = vid.lower(), pid.lower()
        return any(h.vid == v and h.pid == p for h in self.ancestor_hubs)

    def fingerprint(self) -> Tuple[str, str, str, str, Tuple[Tuple[str, str, str], ...]]:
        return (
            self.real_path,
            self.usb_name,
            self.vid,
            self.pid,
            tuple((h.usb_name, h.vid, h.pid) for h in self.ancestor_hubs),
        )


@dataclass(frozen=True)
class ResolverConfig:
    dev_root: Path = Path("/dev")
    sys_root: Path = Path("/sys")
    base_symlink: str = "ttylimo"
    lidar_symlink: str = "ydlidar"
    cp2102_vid: str = "10c4"
    cp2102_pid: str = "ea60"
    base_hub_vid: str = "1a40"
    base_hub_pid: str = "0101"


@dataclass
class ResolvedPort:
    role: str
    preferred_path: str
    real_path: str
    port_name: str
    source: str  # "symlink" or "topology"
    device: SerialDeviceInfo


@dataclass
class ResolutionResult:
    ok: bool
    base: Optional[ResolvedPort] = None
    lidar: Optional[ResolvedPort] = None
    errors: List[str] = field(default_factory=list)
    details: Dict[str, object] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, object]:
        return {
            "ok": self.ok,
            "errors": list(self.errors),
            "base": _resolved_to_dict(self.base) if self.base else None,
            "lidar": _resolved_to_dict(self.lidar) if self.lidar else None,
            "details": self.details,
        }


def _resolved_to_dict(rp: ResolvedPort) -> Dict[str, object]:
    return {
        "role": rp.role,
        "preferred_path": rp.preferred_path,
        "real_path": rp.real_path,
        "port_name": rp.port_name,
        "source": rp.source,
        "usb_name": rp.device.usb_name,
        "vid": rp.device.vid,
        "pid": rp.device.pid,
        "serial": rp.device.serial,
        "ancestor_hubs": [asdict(h) for h in rp.device.ancestor_hubs],
        "readable_writable": rp.device.readable_writable,
    }


def _read_sys_attr(directory: Path, attr: str) -> str:
    p = directory / attr
    try:
        return p.read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return ""


def _default_access_checker(path: Path) -> bool:
    return os.access(str(path), os.R_OK | os.W_OK)


def _find_usb_device_and_ancestors(
    tty_device_link: Path,
) -> Tuple[Optional[Path], Tuple[HubInfo, ...]]:
    """Walk up from /sys/class/tty/<tty>/device to find the USB device and ancestor hubs."""
    try:
        cur = tty_device_link.resolve(strict=True)
    except OSError:
        return None, ()

    usb_dev_dir: Optional[Path] = None
    ancestors: List[HubInfo] = []

    for node in (cur, *cur.parents):
        name = node.name
        if ROOT_HUB_DIR_RE.match(name):
            break
        if USB_DEV_DIR_RE.match(name) and (node / "idVendor").is_file() and (node / "idProduct").is_file():
            if usb_dev_dir is None:
                usb_dev_dir = node
            else:
                ancestors.append(
                    HubInfo(
                        usb_name=name,
                        vid=_read_sys_attr(node, "idVendor").lower(),
                        pid=_read_sys_attr(node, "idProduct").lower(),
                        product=_read_sys_attr(node, "product"),
                    )
                )
    return usb_dev_dir, tuple(ancestors)


def enumerate_base_hubs(config: ResolverConfig) -> List[HubInfo]:
    """Read-only scan of /sys/bus/usb/devices for hubs matching base_hub_vid:base_hub_pid."""
    usb_devices_dir = config.sys_root / "bus" / "usb" / "devices"
    hubs: List[HubInfo] = []
    if not usb_devices_dir.is_dir():
        return hubs
    target_v = config.base_hub_vid.lower()
    target_p = config.base_hub_pid.lower()
    for entry in sorted(usb_devices_dir.iterdir(), key=lambda p: p.name):
        if not USB_DEV_DIR_RE.match(entry.name):
            continue
        vid = _read_sys_attr(entry, "idVendor").lower()
        pid = _read_sys_attr(entry, "idProduct").lower()
        if vid == target_v and pid == target_p:
            hubs.append(
                HubInfo(
                    usb_name=entry.name,
                    vid=vid,
                    pid=pid,
                    product=_read_sys_attr(entry, "product"),
                )
            )
    return hubs


def enumerate_serial_devices(
    config: ResolverConfig,
    access_checker: Callable[[Path], bool] = _default_access_checker,
) -> List[SerialDeviceInfo]:
    """Read-only scan of /dev/ttyUSB* and /dev/ttyACM* joined with /sys/class/tty/*."""
    devices: List[SerialDeviceInfo] = []
    if not config.dev_root.is_dir():
        return devices

    tty_names: List[str] = []
    for entry in sorted(config.dev_root.iterdir(), key=lambda p: p.name):
        if entry.name.startswith(("ttyUSB", "ttyACM")) and not entry.is_symlink():
            tty_names.append(entry.name)

    for tty_name in tty_names:
        dev_path = config.dev_root / tty_name
        try:
            real_path = str(dev_path.resolve(strict=True))
        except OSError:
            continue
        sys_tty_dev = config.sys_root / "class" / "tty" / tty_name / "device"
        usb_dir, ancestors = _find_usb_device_and_ancestors(sys_tty_dev)
        if usb_dir is None:
            continue
        devices.append(
            SerialDeviceInfo(
                tty_name=tty_name,
                dev_path=str(dev_path),
                real_path=real_path,
                usb_name=usb_dir.name,
                vid=_read_sys_attr(usb_dir, "idVendor").lower(),
                pid=_read_sys_attr(usb_dir, "idProduct").lower(),
                serial=_read_sys_attr(usb_dir, "serial"),
                product=_read_sys_attr(usb_dir, "product"),
                manufacturer=_read_sys_attr(usb_dir, "manufacturer"),
                ancestor_hubs=ancestors,
                readable_writable=bool(access_checker(dev_path)),
            )
        )
    return devices


def _resolve_symlink_target(dev_root: Path, link_name: str) -> Optional[str]:
    link_path = dev_root / link_name
    if not link_path.exists():
        return None
    try:
        return str(link_path.resolve(strict=True))
    except OSError:
        return None


def resolve_once(
    config: ResolverConfig = ResolverConfig(),
    access_checker: Callable[[Path], bool] = _default_access_checker,
) -> ResolutionResult:
    """Perform a single read-only, fail-closed resolution pass."""
    base_hubs = enumerate_base_hubs(config)
    all_serial = enumerate_serial_devices(config, access_checker=access_checker)
    by_real = {d.real_path: d for d in all_serial}

    cp_v, cp_p = config.cp2102_vid.lower(), config.cp2102_pid.lower()
    cp2102_devices = [d for d in all_serial if d.vid == cp_v and d.pid == cp_p]

    base_candidates = [
        d for d in cp2102_devices if d.has_ancestor_hub(config.base_hub_vid, config.base_hub_pid)
    ]
    lidar_candidates = [
        d for d in cp2102_devices if not d.has_ancestor_hub(config.base_hub_vid, config.base_hub_pid)
    ]

    details: Dict[str, object] = {
        "base_hub_count": len(base_hubs),
        "base_hubs": [asdict(h) for h in base_hubs],
        "cp2102_count": len(cp2102_devices),
        "base_candidate_ttys": [d.tty_name for d in base_candidates],
        "lidar_candidate_ttys": [d.tty_name for d in lidar_candidates],
    }
    errors: List[str] = []

    base_link_real = _resolve_symlink_target(config.dev_root, config.base_symlink)
    lidar_link_real = _resolve_symlink_target(config.dev_root, config.lidar_symlink)
    details["base_symlink_target"] = base_link_real
    details["lidar_symlink_target"] = lidar_link_real

    # 1. Symlink collision check first
    if base_link_real and lidar_link_real and base_link_real == lidar_link_real:
        errors.append(ERR_PORT_COLLISION)
        details["collision_reason"] = "base and lidar symlinks point to the same device"
        return ResolutionResult(ok=False, errors=errors, details=details)

    # 2. Ambiguity guards (strict fail-closed: never guess among ambiguous CP2102s or duplicate hubs)
    if len(base_hubs) > 1:
        errors.append(ERR_AMBIGUOUS_CP2102_CANDIDATES)
        details["ambiguity_reason"] = f"multiple base hubs ({config.base_hub_vid}:{config.base_hub_pid}) detected: {len(base_hubs)}"
    if len(cp2102_devices) > 2:
        if ERR_AMBIGUOUS_CP2102_CANDIDATES not in errors:
            errors.append(ERR_AMBIGUOUS_CP2102_CANDIDATES)
        details["ambiguity_reason"] = f"more than 2 CP2102 devices detected ({len(cp2102_devices)})"
    if len(base_candidates) > 1 or len(lidar_candidates) > 1:
        if ERR_AMBIGUOUS_CP2102_CANDIDATES not in errors:
            errors.append(ERR_AMBIGUOUS_CP2102_CANDIDATES)
        details["ambiguity_reason"] = (
            f"multiple candidates for a role (base={len(base_candidates)}, lidar={len(lidar_candidates)})"
        )

    if errors:
        return ResolutionResult(ok=False, errors=errors, details=details)

    # 3. Resolve Base (symlink first WITH topology/role verification, else unique topology candidate)
    resolved_base: Optional[ResolvedPort] = None
    if base_link_real is not None:
        dev = by_real.get(base_link_real)
        if dev is None or dev not in base_candidates:
            errors.append(ERR_AMBIGUOUS_CP2102_CANDIDATES)
            details["base_symlink_invalid"] = (
                f"/{config.base_symlink} -> {base_link_real} does not match a verified base CP2102 under hub "
                f"{config.base_hub_vid}:{config.base_hub_pid}"
            )
        else:
            pref = str(config.dev_root / config.base_symlink)
            resolved_base = ResolvedPort(
                role="base",
                preferred_path=pref,
                real_path=dev.real_path,
                port_name=config.base_symlink,
                source="symlink",
                device=dev,
            )
    else:
        if len(base_hubs) == 0:
            errors.append(ERR_BASE_HUB_NOT_ENUMERATED)
        elif len(base_candidates) == 0:
            errors.append(ERR_BASE_PORT_NOT_FOUND)
        elif len(base_candidates) == 1:
            dev = base_candidates[0]
            resolved_base = ResolvedPort(
                role="base",
                preferred_path=dev.dev_path,
                real_path=dev.real_path,
                port_name=dev.tty_name,
                source="topology",
                device=dev,
            )

    # 4. Resolve LiDAR (symlink first WITH topology/role verification, else unique topology candidate)
    resolved_lidar: Optional[ResolvedPort] = None
    if lidar_link_real is not None:
        dev = by_real.get(lidar_link_real)
        if dev is None or dev not in lidar_candidates:
            if ERR_AMBIGUOUS_CP2102_CANDIDATES not in errors:
                errors.append(ERR_AMBIGUOUS_CP2102_CANDIDATES)
            details["lidar_symlink_invalid"] = (
                f"/{config.lidar_symlink} -> {lidar_link_real} does not match a verified non-base-hub CP2102"
            )
        else:
            pref = str(config.dev_root / config.lidar_symlink)
            resolved_lidar = ResolvedPort(
                role="lidar",
                preferred_path=pref,
                real_path=dev.real_path,
                port_name=config.lidar_symlink,
                source="symlink",
                device=dev,
            )
    else:
        if len(lidar_candidates) == 0:
            errors.append(ERR_LIDAR_PORT_NOT_FOUND)
        elif len(lidar_candidates) == 1:
            dev = lidar_candidates[0]
            resolved_lidar = ResolvedPort(
                role="lidar",
                preferred_path=dev.dev_path,
                real_path=dev.real_path,
                port_name=dev.tty_name,
                source="topology",
                device=dev,
            )

    # 5. Final collision & permission checks
    if resolved_base and resolved_lidar and resolved_base.real_path == resolved_lidar.real_path:
        errors.append(ERR_PORT_COLLISION)
    if resolved_base and not resolved_base.device.readable_writable:
        errors.append(ERR_PORT_PERMISSION_DENIED)
        details["base_permission_denied"] = resolved_base.real_path
    if resolved_lidar and not resolved_lidar.device.readable_writable:
        if ERR_PORT_PERMISSION_DENIED not in errors:
            errors.append(ERR_PORT_PERMISSION_DENIED)
        details["lidar_permission_denied"] = resolved_lidar.real_path

    ok = (not errors) and (resolved_base is not None) and (resolved_lidar is not None)
    return ResolutionResult(
        ok=ok,
        base=resolved_base if ok else None,
        lidar=resolved_lidar if ok else None,
        errors=errors,
        details=details,
    )


def resolve_with_retry(
    config: ResolverConfig = ResolverConfig(),
    timeout_s: float = 5.0,
    interval_s: float = 0.5,
    access_checker: Callable[[Path], bool] = _default_access_checker,
    sleep_fn: Callable[[float], None] = time.sleep,
    monotonic_fn: Callable[[], float] = time.monotonic,
) -> ResolutionResult:
    """Bounded retry for delayed USB enumeration; fails fast on ambiguity/collision/permission."""
    timeout_s = max(0.0, float(timeout_s))
    interval_s = max(0.01, float(interval_s))
    deadline = monotonic_fn() + timeout_s
    attempts = 0

    while True:
        attempts += 1
        res = resolve_once(config=config, access_checker=access_checker)
        res.details["attempts"] = attempts
        if res.ok:
            return res
        # Fail fast if any non-transient error occurred
        if any(err not in TRANSIENT_ERRORS for err in res.errors):
            return res
        now = monotonic_fn()
        if now >= deadline:
            return res
        sleep_fn(min(interval_s, max(0.0, deadline - now)))


def verify_before_start(
    initial: ResolutionResult,
    config: ResolverConfig = ResolverConfig(),
    access_checker: Callable[[Path], bool] = _default_access_checker,
) -> ResolutionResult:
    """Re-verify device identity and permissions immediately prior to driver launch."""
    if not initial.ok or initial.base is None or initial.lidar is None:
        return initial

    current = resolve_once(config=config, access_checker=access_checker)
    if not current.ok or current.base is None or current.lidar is None:
        return ResolutionResult(
            ok=False,
            errors=[ERR_DEVICE_CHANGED_BEFORE_START, *current.errors],
            details={
                "reason": "device disappeared or became invalid before driver start",
                "recheck_details": current.details,
            },
        )

    if (
        current.base.device.fingerprint() != initial.base.device.fingerprint()
        or current.lidar.device.fingerprint() != initial.lidar.device.fingerprint()
    ):
        return ResolutionResult(
            ok=False,
            errors=[ERR_DEVICE_CHANGED_BEFORE_START],
            details={
                "reason": "device topology or path changed between resolution and driver start",
                "initial_base": initial.base.real_path,
                "current_base": current.base.real_path,
                "initial_lidar": initial.lidar.real_path,
                "current_lidar": current.lidar.real_path,
            },
        )
    return current


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Read-only fail-closed USB serial resolver for LIMO Base & YDLIDAR")
    ap.add_argument("--dev-root", default="/dev")
    ap.add_argument("--sys-root", default="/sys")
    ap.add_argument("--timeout", type=float, default=0.0, help="Bounded retry timeout in seconds (default: 0)")
    ap.add_argument("--interval", type=float, default=0.5, help="Retry interval in seconds (default: 0.5)")
    ap.add_argument("--json", dest="as_json", action="store_true")
    args = ap.parse_args(argv)

    cfg = ResolverConfig(dev_root=Path(args.dev_root), sys_root=Path(args.sys_root))
    res = resolve_with_retry(cfg, timeout_s=args.timeout, interval_s=args.interval)
    if res.ok:
        res = verify_before_start(res, cfg)

    if args.as_json:
        print(json.dumps(res.to_dict(), indent=2, ensure_ascii=False))
    else:
        if res.ok and res.base and res.lidar:
            print(
                f"OK base={res.base.preferred_path} (port_name={res.base.port_name}, real={res.base.real_path}, "
                f"usb={res.base.device.usb_name}, src={res.base.source}) | "
                f"lidar={res.lidar.preferred_path} (real={res.lidar.real_path}, "
                f"usb={res.lidar.device.usb_name}, src={res.lidar.source})"
            )
        else:
            print(f"FAIL errors={res.errors} details={json.dumps(res.details, ensure_ascii=False)}")
    return 0 if res.ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
