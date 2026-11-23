#!/usr/bin/env python3
"""Atomic package directory replacement for the LIMO workspace (R3 follow-up, 2026-11-23). Removes the "SRC missing" failure.

R3 (limo_package_deploy_20261122.sh) replaces a package with two `mv` calls (SRC -> OLD, STAGE -> SRC). Between them SRC does not
exist, and if the second `mv` and its compensation both fail the package is gone (manual recovery). This tool uses one
renameat2(RENAME_EXCHANGE) system call instead: SRC and the staged directory trade places atomically, so SRC exists at every instant,
holds either the old or the new tree, and can be traded back with the same call. If the call is unsupported or fails, nothing has changed.

  atomic_swap_20261123.py replace  --src WS/src/PKG --stage DIR/stage_STAMP/PKG --files-sha FILES.sha256 [--keep DIR/PKG_old_STAMP]
  atomic_swap_20261123.py rollback --src WS/src/PKG --stage DIR_with_extracted_backup/PKG --files-sha BACKUP.files.sha256 [--keep ...]

`stage` is a fully extracted, hash-verified tree (R3 `stage` / backup extraction produce one). The previous tree is left at the
stage path (never deleted); `--keep` renames it to a stable name. Exit codes: 0 done, 3 refused/failed with the state printed
(SRC is always present and is either the old tree [UNCHANGED/ROLLED_BACK] or the new tree [SWAPPED_UNVERIFIED]), 2 usage.
Standard library only; Linux (ext4/xfs/btrfs/tmpfs). Refuses to run while robot processes exist unless --force-running (tests only).
"""
import argparse
import ctypes
import ctypes.util
import hashlib
import os
import subprocess
import sys

RENAME_EXCHANGE = 2
AT_FDCWD = -100


class SwapError(Exception):
    pass


def exchange(a, b):
    """Atomically swap two paths. Raises OSError (nothing changed) if the kernel or file system cannot."""
    libc = ctypes.CDLL(ctypes.util.find_library('c') or 'libc.so.6', use_errno=True)
    fn = getattr(libc, 'renameat2', None)
    if fn is None:
        raise OSError(38, 'renameat2 not available')
    if fn(AT_FDCWD, os.fsencode(a), AT_FDCWD, os.fsencode(b), RENAME_EXCHANGE) != 0:
        e = ctypes.get_errno()
        raise OSError(e, os.strerror(e))


def tree_hashes(root):
    out = {}
    for d, _, files in os.walk(root):
        if '__pycache__' in d.split(os.sep):
            continue
        for f in files:
            if f.endswith('.pyc'):
                continue
            p = os.path.join(d, f)
            if os.path.islink(p):
                raise SwapError(f'symbolic link in package tree: {p}')
            with open(p, 'rb') as fh:
                out[os.path.relpath(p, root)] = hashlib.sha256(fh.read()).hexdigest()
    return out


def listed_hashes(files_sha, pkg):
    """FILES.sha256 lines are 'HASH  PKG/relative/path' (R3 artifact) or 'HASH  relative/path'."""
    out = {}
    for line in open(files_sha):
        if not line.strip():
            continue
        h, name = line.split(None, 1)
        name = name.strip()
        if name.startswith(pkg + os.sep):
            name = name[len(pkg) + 1:]
        out[name] = h
    return out


def verify(root, files_sha, pkg):
    want, got = listed_hashes(files_sha, pkg), tree_hashes(root)
    return want == got


def robot_processes():
    ps = subprocess.run(['ps', '-eo', 'args='], capture_output=True, text=True).stdout.splitlines()
    keys = ('robot_agent', 'limo_base', 'stall_detector_node', 'motion_watchdog', 'ros2 launch', 'ros2 bag')
    return [l for l in ps if any(k in l for k in keys) and 'atomic_swap' not in l and 'grep' not in l]


def swap_in(src, stage, files_sha, keep=None, force_running=False, exchange_fn=exchange, verify_fn=verify):
    """Returns (state, message). state in UNCHANGED, SWAPPED, ROLLED_BACK, SWAPPED_UNVERIFIED. SRC exists in every one."""
    pkg = os.path.basename(src.rstrip('/'))
    for p, label in ((src, 'src'), (stage, 'stage')):
        if not os.path.isdir(p) or os.path.islink(p):
            return 'UNCHANGED', f'{label} is not a real directory: {p}'
    if not force_running and robot_processes():
        return 'UNCHANGED', 'robot/ROS processes are running; stop them first'
    if os.stat(src).st_dev != os.stat(stage).st_dev:
        return 'UNCHANGED', 'src and stage are on different file systems; stage next to the workspace'
    try:
        if not verify_fn(stage, files_sha, pkg):
            return 'UNCHANGED', 'staged tree does not match FILES.sha256; nothing changed'
    except (SwapError, OSError) as e:
        return 'UNCHANGED', f'verification failed before the swap: {e}'
    try:
        exchange_fn(src, stage)
    except OSError as e:
        return 'UNCHANGED', f'atomic exchange failed ({e}); nothing changed'
    # now SRC = new tree, stage = old tree
    ok = False
    try:
        ok = verify_fn(src, files_sha, pkg)
    except (SwapError, OSError):
        ok = False
    if ok:
        if keep:
            try:
                os.rename(stage, keep)
            except OSError:
                pass
        return 'SWAPPED', 'SRC now holds the staged tree (verified); the previous tree is preserved at ' + (keep if keep and os.path.isdir(keep) else stage)
    try:
        exchange_fn(src, stage)
    except OSError as e:
        return 'SWAPPED_UNVERIFIED', f'post-swap verification failed AND the swap-back failed ({e}); SRC exists with the NEW unverified tree, previous tree preserved at {stage}'
    return 'ROLLED_BACK', f'post-swap verification failed; the previous tree is back in SRC and the new tree is at {stage}'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('action', choices=('replace', 'rollback'))
    ap.add_argument('--src', required=True); ap.add_argument('--stage', required=True); ap.add_argument('--files-sha', required=True)
    ap.add_argument('--keep'); ap.add_argument('--force-running', action='store_true')
    a = ap.parse_args(argv)
    state, msg = swap_in(a.src, a.stage, a.files_sha, a.keep, a.force_running)
    print(f'{state}: {msg}')
    return 0 if state == 'SWAPPED' else 3


if __name__ == '__main__':
    sys.exit(main())
