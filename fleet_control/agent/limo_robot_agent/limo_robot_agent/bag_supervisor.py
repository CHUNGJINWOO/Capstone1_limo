"""Per-session rosbag watchdog. Only a live Fleet fence renews its private pipe."""
from functools import partial
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import time
import shutil

from .local_process import spawn_owned, write_json
from .owned_bag import OwnedBag

MAX_DURATION = 300
MAX_BYTES = 256 * 1024 * 1024
MIN_FREE = 1024 * 1024 * 1024
LEASE_TIMEOUT = 5


def supervise(root, sid):
    session = root / 'sessions' / sid
    session.mkdir(parents=True, exist_ok=False)
    journal = session / 'journal.json'
    state = {'session_id': sid, 'status': 'STARTING', 'started_at': time.time()}
    write_json(journal, state)
    bag = OwnedBag(root / 'bags', spawn=partial(spawn_owned, file_limit=MAX_BYTES), node_name='fleet_bag_' + sid,
                   log_path=session / 'recorder.log')
    reason = 'recorder_failed'
    verified = False
    try:
        if shutil.disk_usage(root).free < MIN_FREE:
            reason = 'disk_low'
            return
        started = renewed = time.monotonic()
        target = bag.start(sid, ['/map'])
        state.update(status='SPAWNED', process=bag.identity)
        write_json(journal, state)
        while bag.process.poll() is None:
            now = time.monotonic()
            if now - started >= MAX_DURATION:
                reason = 'duration_limit'
            elif now - renewed >= LEASE_TIMEOUT:
                reason = 'fence_expired'
            elif shutil.disk_usage(root).free < MIN_FREE:
                reason = 'disk_low'
            elif sum(p.stat().st_size for p in target.glob('*') if p.is_file()) >= MAX_BYTES:
                reason = 'size_limit'
            else:
                if not select.select([0], [], [], .15)[0]:
                    continue
                requests = os.read(0, 4096)
                if requests and b'S' not in requests:
                    renewed = time.monotonic()
                    continue
                reason = 'requested_stop' if requests else 'agent_pipe_closed'
            break
    except Exception as exc:
        reason = type(exc).__name__
    finally:
        # Termination takes priority if the journal filesystem can no longer write.
        if bag.process is not None:
            try:
                verified = bag.stop(sid, timeout=2)
            except Exception:
                verified = False
        status = ('STOPPED' if verified and reason == 'requested_stop'
                  else 'FAILED' if reason == 'recorder_failed' else 'UNKNOWN')
        write_json(journal, dict(state, status=status, reason=reason, bag_verified=verified,
                                 finished_at=time.time()))


class SupervisedBag:
    def __init__(self, root):
        self.root = root
        self.process = None
        self.session_id = None

    def state(self):
        if not self.session_id:
            return {}
        path = self.root / 'sessions' / self.session_id / 'journal.json'
        try:
            return json.loads(path.read_text())
        except (OSError, ValueError):
            return {}

    @property
    def identity(self):
        return self.state().get('process')

    def start(self, sid, topics):
        if (self.process is not None or topics != ['/map'] or not sid.isascii()
                or not sid.isalnum() or len(sid) > 64):
            raise ValueError('invalid recorder session')
        self.process = subprocess.Popen(
            [sys.executable, '-m', __name__, str(self.root), sid],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        self.session_id = sid
        os.set_blocking(self.process.stdin.fileno(), False)

    def renew(self):
        os.write(self.process.stdin.fileno(), b'R')

    def stop(self, sid):
        if self.process is None or sid != self.session_id:
            return False
        if self.process.poll() is None:
            os.write(self.process.stdin.fileno(), b'S')
        self.process.wait(timeout=12)
        state = self.state()
        self.close()
        self.process = None
        self.session_id = None
        return state.get('status') == 'STOPPED' and state.get('bag_verified') is True

    def reap(self):
        """Retain UNKNOWN when a dead supervisor could not finish its journal."""
        if self.process is None or self.process.poll() is None:
            return {}
        state = self.state()
        if state.get('status') not in ('STOPPED', 'FAILED', 'UNKNOWN'):
            path = self.root / 'sessions' / self.session_id / 'journal.json'
            path.parent.mkdir(parents=True, exist_ok=True)
            state = dict(state, session_id=self.session_id, status='UNKNOWN',
                         reason='supervisor_lost', bag_verified=False, finished_at=time.time())
            write_json(path, state)
        self.close()
        self.process = None
        return state

    def close(self):
        if self.process is not None:
            self.process.stdin.close()


if __name__ == '__main__':
    supervise(Path(sys.argv[1]), sys.argv[2])
