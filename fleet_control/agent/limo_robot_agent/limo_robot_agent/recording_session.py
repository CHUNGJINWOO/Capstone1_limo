"""Non-motion rosbag supervisor; integration with AgentCore is intentionally absent.

One supervisor owns one session directory. Child handles are never reconstructed
from a PID after restart; an unfinished session becomes UNKNOWN instead.
"""
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time

SESSION_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,79}$')
# Open-RMF BuildingMap lives on its own topic: /map carries nav_msgs/OccupancyGrid only (rosbag2 cannot record a topic with two types).
# See fleet_control/rmf_adapter/rmf_map_shim/README.md. Fixed profile entry, never operator-supplied.
RMF_BUILDING_MAP_TOPIC = '/rmf/building_map'


class RecordingSession:
    def __init__(self, root, session_id, recorder_script, verifier_script):
        if not SESSION_ID.fullmatch(session_id):
            raise ValueError('invalid session ID')
        self.root = Path(root).resolve()
        if not self.root.is_dir():
            raise ValueError('root must be an existing directory')
        self.path = self.root / session_id
        self.recorder = Path(recorder_script).resolve(strict=True)
        self.verifier = Path(verifier_script).resolve(strict=True)
        self.session_id = session_id
        self.lock = threading.RLock()
        self.process = None
        self.log = None
        try:
            self.path.mkdir(mode=0o700)
            created = True
        except FileExistsError:
            created = False
        if self.path.is_symlink() or not self.path.is_dir():
            raise ValueError('invalid session directory')
        self.lockfile = (self.path / '.recording.lock').open('a')
        try:
            fcntl.flock(self.lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            self.lockfile.close()
            raise
        self.state_path = self.path / 'recording-state.json'
        if created:
            self.state = {'session_id': session_id, 'status': 'PREPARED', 'detail': '', 'run_dir': None}
            self._save()
        else:
            try:
                self.state = json.loads(self.state_path.read_text())
                if self.state.get('session_id') != session_id:
                    raise ValueError('session identity mismatch')
                if self.state.get('status') in ('STARTING', 'RECORDING', 'STOP_REQUESTED', 'VERIFYING'):
                    self.state.update(status='UNKNOWN', detail='supervisor restarted; child ownership is not recoverable')
                    self._save()
            except BaseException:
                self.close()
                raise

    def _save(self):
        tmp = self.path / '.recording-state.tmp'
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, 'w') as f:
            json.dump(self.state, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.state_path)
        fd = os.open(self.path, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)

    def _set(self, status, detail='', **extra):
        self.state.update(status=status, detail=detail, **extra)
        self._save()

    def _runs(self):
        return sorted(p for p in self.path.iterdir() if p.is_dir() and not p.is_symlink())

    def start(self):
        """Start the fixed recorder profile, never accept executable/topic arguments."""
        with self.lock:
            if self.state['status'] in ('STARTING', 'RECORDING') and self.process is not None:
                return self.status()
            if self.state['status'] != 'PREPARED':
                raise RuntimeError('session is not prepared; refusing duplicate or uncertain start')
            self._set('STARTING', 'start intent persisted')
            try:
                fd = os.open(self.path / 'recorder.log', os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
                self.log = os.fdopen(fd, 'wb')
                self.process = subprocess.Popen([sys.executable, '-u', str(self.recorder), '--root', str(self.path), '--label', self.session_id, '--extra', '/scan_slam', '--extra', RMF_BUILDING_MAP_TOPIC],
                                                stdin=subprocess.DEVNULL, stdout=self.log, stderr=subprocess.STDOUT)
            except Exception as e:
                self._set('FAILED', 'recorder start failed: ' + type(e).__name__)
                if self.log:
                    self.log.close(); self.log = None
                raise
            return dict(self.state)

    def status(self):
        with self.lock:
            if self.process is not None and self.state['status'] in ('STARTING', 'RECORDING'):
                rc = self.process.poll()
                runs = self._runs()
                if rc is not None:
                    self._set('FAILED', 'recorder exited before stop request', returncode=rc)
                elif len(runs) == 1 and (runs[0] / 'bag').is_dir():
                    self._set('RECORDING', 'bag directory observed; topic receipt not proven', run_dir=str(runs[0]))
                elif len(runs) > 1:
                    self._set('UNKNOWN', 'multiple run directories; refusing to infer ownership')
            return dict(self.state)

    def stop(self, timeout=30):
        with self.lock:
            self.status()
            if self.state['status'] == 'VERIFIED':
                return dict(self.state)
            if self.process is None or self.state['status'] not in ('STARTING', 'RECORDING'):
                raise RuntimeError('no owned live recorder; stop outcome unknown')
            self._set('STOP_REQUESTED', 'SIGINT requested; not yet stopped')
            self.process.send_signal(signal.SIGINT)
            try:
                rc = self.process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self._set('UNKNOWN', 'recorder did not exit before timeout; no force kill attempted')
                return dict(self.state)
            finally:
                if self.process.poll() is not None and self.log:
                    self.log.close(); self.log = None
            if rc != 0:
                self._set('FAILED', 'recorder process failed', returncode=rc)
                return dict(self.state)
            self._set('VERIFYING', 'recorder exited; validating bag')
            runs = self._runs()
            try:
                if len(runs) != 1:
                    raise ValueError('expected exactly one recorder run')
                spec = importlib.util.spec_from_file_location('limo_bag_verifier', self.verifier)
                verifier = importlib.util.module_from_spec(spec); spec.loader.exec_module(verifier)
                files = verifier.files_for(runs[0])
                # rosbag2 may exit successfully despite refusing an ambiguous topic.
                # Structural integrity alone must not imply a complete recording.
                with (self.path / 'recorder.log').open('r', errors='replace') as log:
                    if any('has more than one type associated' in line for line in log):
                        raise ValueError('rosbag topic type ambiguity; recording incomplete')
                self._set('VERIFIED', 'recorder exited and bag structure verified; topic receipt not proven',
                          run_dir=str(runs[0]), bag_bytes=sum(p.stat().st_size for p in files))
            except Exception as e:
                detail = str(e) if isinstance(e, ValueError) else type(e).__name__
                self._set('FAILED', 'bag verification failed: ' + detail[:160])
            return dict(self.state)

    def close(self):
        # Closing a supervisor cannot prove or force its child to have stopped.
        if self.log:
            self.log.close(); self.log = None
        if self.lockfile and not self.lockfile.closed:
            self.lockfile.close()
