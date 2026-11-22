"""Opt-in robot-local recording command handler; never accepts executable or topic input."""
import fcntl
import json
import os
from pathlib import Path
import threading
import time
from .recording_session import RecordingSession, SESSION_ID


class RecordingControl:
    def __init__(self, root, recorder, verifier):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.recorder, self.verifier = recorder, verifier
        self.lockfile = (self.root / '.robot-recorder.lock').open('a')
        fcntl.flock(self.lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.lock = threading.RLock()
        self.session = None
        self.current = {'enabled': True, 'status': 'IDLE', 'session_id': None}
        self.pointer = self.root / 'current-session.json'
        if self.pointer.exists():
            try:
                data = json.loads(self.pointer.read_text())
                self.session = RecordingSession(self.root, data['session_id'], recorder, verifier)
                self.current = dict(enabled=True, **self.session.status())
            except Exception:
                self.current = {'enabled': True, 'status': 'UNKNOWN', 'detail': 'recording state cannot be recovered', 'session_id': None}

    def snapshot(self):
        with self.lock:
            if self.session and self.current['status'] in ('STARTING', 'RECORDING'):
                try: self.current = dict(enabled=True, **self.session.status())
                except Exception: self.current = {'enabled': True, 'status': 'UNKNOWN', 'detail': 'state read failed', 'session_id': self.current.get('session_id')}
            return dict(self.current)

    def _point(self, sid):
        tmp = self.root / '.current-session.tmp'
        fd = os.open(tmp, os.O_CREAT | os.O_TRUNC | os.O_WRONLY, 0o600)
        with os.fdopen(fd, 'w') as f:
            json.dump({'session_id': sid}, f); f.flush(); os.fsync(f.fileno())
        os.replace(tmp, self.pointer)
        fd = os.open(self.root, os.O_RDONLY | os.O_DIRECTORY)
        try: os.fsync(fd)
        finally: os.close(fd)

    def command(self, cmd, ack):
        sid = cmd.get('payload', {}).get('session_id')
        if not isinstance(sid, str) or not SESSION_ID.fullmatch(sid):
            ack('REJECTED', 'invalid session ID'); return
        typ = cmd.get('type')
        with self.lock:
            st = self.snapshot()
            if typ == 'record_start':
                if st['status'] not in ('IDLE', 'VERIFIED', 'FAILED') or sid == st.get('session_id'):
                    ack('REJECTED', 'recording active, uncertain, or session already used'); return
                try:
                    if self.session: self.session.close()
                    self._point(sid)
                    self.session = RecordingSession(self.root, sid, self.recorder, self.verifier)
                    self.current = dict(enabled=True, **self.session.start())
                except Exception as e:
                    self.current = {'enabled': True, 'status': 'UNKNOWN', 'session_id': sid, 'detail': type(e).__name__}
                    ack('RECONCILIATION_REQUIRED', 'record start outcome unknown'); return
                ack('ACCEPTED', 'record start requested; waiting for recorder')
                threading.Thread(target=self._wait_start, args=(ack,), daemon=True).start()
            elif typ == 'record_stop':
                if sid != st.get('session_id') or st['status'] not in ('STARTING', 'RECORDING'):
                    ack('REJECTED', 'session mismatch or no owned active recorder'); return
                self.current = dict(st, status='STOP_REQUESTED', detail='stop requested; awaiting process exit and validation')
                ack('ACCEPTED', 'stop requested; exit not yet confirmed')
                threading.Thread(target=self._wait_stop, args=(ack,), daemon=True).start()
            else:
                ack('REJECTED', 'unknown recording command')

    def _wait_start(self, ack):
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            with self.lock:
                if self.current.get('status') == 'STOP_REQUESTED':
                    ack('RECONCILIATION_REQUIRED', 'start observation interrupted by stop request'); return
                st = self.snapshot()
            if st['status'] == 'RECORDING': ack('SUCCEEDED', 'recorder process and bag directory observed'); return
            if st['status'] in ('FAILED', 'UNKNOWN'): ack('FAILED', st.get('detail', 'recording failed')); return
            time.sleep(.2)
        with self.lock: self.current = dict(self.current, status='UNKNOWN', detail='recorder start timed out')
        ack('RECONCILIATION_REQUIRED', 'record start timed out')

    def _wait_stop(self, ack):
        try:
            st = self.session.stop(timeout=30)
            with self.lock: self.current = dict(enabled=True, **st)
            if st['status'] == 'VERIFIED': ack('SUCCEEDED', 'recorder exited; bag structure verified')
            elif st['status'] == 'UNKNOWN': ack('RECONCILIATION_REQUIRED', st.get('detail', 'stop unknown'))
            else: ack('FAILED', st.get('detail', 'recording failed'))
        except Exception as e:
            with self.lock: self.current = dict(self.current, status='UNKNOWN', detail=type(e).__name__)
            ack('RECONCILIATION_REQUIRED', 'stop outcome unknown')

    def close(self):
        if self.session: self.session.close()
        self.lockfile.close()
