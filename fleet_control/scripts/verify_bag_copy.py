#!/usr/bin/env python3
"""Copy an already stopped run_recorder.py SQLite bag to private local storage.

No ROS nodes, network connections, process control, motion commands, or source
file deletion. Copy only bag data; metadata may itself contain sensitive data.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import sqlite3
import subprocess
import sys
import tempfile

CHUNK = 1024 * 1024
ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,79}$')


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        while True:
            block = f.read(CHUNK)
            if not block:
                return h.hexdigest()
            h.update(block)


def files_for(run):
    if run.is_symlink() or (run / 'bag').is_symlink() or (run / 'exit.json').is_symlink():
        raise ValueError('symlink source not allowed')
    result = json.loads((run / 'exit.json').read_text())
    if result.get('reason') not in ('operator_signal_SIGINT', 'operator_signal_SIGTERM', 'recorder_exited_normally'):
        raise ValueError('no normal recorder exit evidence')
    if result.get('recorder_returncode') not in (0, -2, 130):
        raise ValueError('recorder return code is not an accepted graceful exit')
    bag = run / 'bag'
    entries = list(bag.iterdir())
    if any(p.is_symlink() or not p.is_file() for p in entries):
        raise ValueError('bag must contain regular files only')
    if not (bag / 'metadata.yaml').is_file():
        raise ValueError('missing bag metadata')
    dbs = [p for p in entries if p.name.endswith(('.db3', '.db3.zstd'))]
    if not dbs or any(p.name != 'metadata.yaml' and p not in dbs for p in entries):
        raise ValueError('only SQLite rosbag2, optionally file-compressed with zstd, is supported')
    for db in dbs:
        if db.name.endswith('.zstd'):
            # SQLite must inspect decompressed bytes; never alter the original.
            with tempfile.TemporaryDirectory(prefix='limo-bag-verify-') as tmp:
                unpacked = Path(tmp) / 'verify.db3'
                with unpacked.open('xb') as out:
                    proc = subprocess.run(['zstd', '-d', '-q', '-c', str(db)], stdout=out, stderr=subprocess.PIPE, timeout=300)
                if proc.returncode:
                    raise ValueError('zstd decompression failed: ' + db.name)
                check_sqlite(unpacked)
        else:
            check_sqlite(db)
    proc = subprocess.run(['ros2', 'bag', 'info', str(bag)], capture_output=True, text=True, timeout=60)
    if proc.returncode:
        raise ValueError('ros2 bag info failed')
    return sorted(entries)


def check_sqlite(db):
    with sqlite3.connect(db.resolve().as_uri() + '?mode=ro', uri=True) as conn:
        if conn.execute('PRAGMA integrity_check').fetchone() != ('ok',):
            raise ValueError('SQLite integrity failure: ' + db.name)


def copy_verified(run, target_root, session):
    if not ID.fullmatch(session):
        raise ValueError('invalid session ID')
    run = Path(run).absolute()
    root = Path(target_root).absolute()
    if not root.is_dir() or root.is_symlink():
        raise ValueError('destination root must be an existing non-symlink directory')
    if root.resolve().is_relative_to(run.resolve()):
        raise ValueError('destination cannot be inside source')
    files = files_for(run)
    destination = root / session
    created = False
    try:
        destination.mkdir(mode=0o700)
        created = True
    except FileExistsError:
        if destination.is_symlink() or not destination.is_dir():
            raise ValueError('invalid session destination')
    marker = destination / '.copy-session.json'
    descriptor = {'session_id': session, 'source': str(run.resolve())}
    if created:
        fd = os.open(marker, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as out:
            json.dump(descriptor, out)
    elif marker.is_symlink() or not marker.is_file() or json.loads(marker.read_text()) != descriptor:
        raise ValueError('destination is not this copy session')
    with (destination / '.copy.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        manifest = []
        for src in files:
            before = src.stat()
            source_hash = digest(src)
            dest = destination / src.name
            if dest.is_symlink():
                raise ValueError('destination symlink refused')
            if dest.exists():
                # Resume only when the existing bytes match the corresponding source prefix.
                if not dest.is_file() or dest.stat().st_nlink != 1 or dest.stat().st_size > before.st_size:
                    raise ValueError('destination conflict: ' + src.name)
                with src.open('rb') as a, dest.open('rb') as b:
                    while True:
                        block = b.read(CHUNK)
                        if not block:
                            break
                        if a.read(len(block)) != block:
                            raise ValueError('destination prefix conflict: ' + src.name)
                offset = dest.stat().st_size
                mode = 'ab'
            else:
                offset = 0
                mode = 'xb'
            with src.open('rb') as inp, dest.open(mode) as out:
                os.chmod(dest, 0o600)
                inp.seek(offset)
                while True:
                    block = inp.read(CHUNK)
                    if not block:
                        break
                    out.write(block)
                out.flush()
                os.fsync(out.fileno())
            after = src.stat()
            if (before.st_size, before.st_mtime_ns, before.st_ino) != (after.st_size, after.st_mtime_ns, after.st_ino) or digest(src) != source_hash:
                raise ValueError('source changed during copy: ' + src.name)
            if digest(dest) != source_hash:
                raise ValueError('destination hash mismatch: ' + src.name)
            manifest.append({'name': src.name, 'size': before.st_size, 'sha256': source_hash})
        # Never overwrite a previous receipt. A repeated identical operation is idempotent.
        receipt = destination / 'verified-manifest.json'
        content = {'session_id': session, 'files': manifest, 'scope': 'local_copy_only'}
        if receipt.exists():
            if receipt.is_symlink() or json.loads(receipt.read_text()) != content:
                raise ValueError('existing receipt conflict')
        else:
            fd = os.open(receipt, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, 'w') as out:
                json.dump(content, out, indent=2)
                out.write('\n')
                out.flush()
                os.fsync(out.fileno())
        return content


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-dir', required=True, help='finished run_recorder.py run directory')
    ap.add_argument('--destination-root', required=True, help='existing private local destination directory')
    ap.add_argument('--session-id', required=True)
    args = ap.parse_args()
    try:
        result = copy_verified(args.run_dir, args.destination_root, args.session_id)
    except (OSError, ValueError, sqlite3.Error, subprocess.TimeoutExpired) as exc:
        print('verification failed: ' + str(exc), file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
