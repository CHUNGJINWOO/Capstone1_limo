"""Own one rosbag2 recorder without discovering or signalling other processes."""
import os
from pathlib import Path
import signal
import time
import subprocess
import yaml


class OwnedBag:
    def __init__(self, root, *, spawn=None, node_name=None, log_path=None):
        self.root = Path(root)
        self.spawn, self.node_name, self.log_path = spawn, node_name, log_path
        self.process = None
        self.session_id = None
        self.identity = None

    def start(self, session_id, topics):
        if self.process is not None:
            raise RuntimeError('Recorder ownership already exists')
        if not session_id or not session_id.isalnum() or not topics:
            raise ValueError('Session identifier and topics are required')
        self.root.mkdir(parents=True, exist_ok=True)
        destination = self.root / session_id
        if destination.exists():
            raise FileExistsError(destination)
        command = ['ros2', 'bag', 'record', '-o', str(destination)]
        if self.node_name:
            command.extend(['--node-name', self.node_name])
        command.extend(topics)
        with open(self.log_path or os.devnull, 'a') as log:
            streams = dict(stdin=subprocess.DEVNULL, stdout=log, stderr=log)
            self.process = (self.spawn(command, **streams) if self.spawn else
                            subprocess.Popen(command, start_new_session=True, **streams))
        self.session_id = session_id
        stat = Path(f'/proc/{self.process.pid}/stat').read_text().rpartition(') ')[2].split()
        self.identity = {'pid': self.process.pid, 'start_ticks': int(stat[19]),
                         'pgid': os.getpgid(self.process.pid), 'session_id': session_id,
                         'destination': str(destination), 'started_at': time.time()}
        # Existence alone is not ROS readiness. Callers must observe subscriptions.
        return destination

    def stop(self, session_id, timeout=10.0):
        if self.process is None or session_id != self.session_id:
            raise RuntimeError('Recorder is not owned by this session')
        process = self.process
        if process.poll() is not None:
            return False
        if self.spawn and self.identity is None:
            from .local_process import stop_owned
            stop_owned(process)
            return False
        try:
            stat = Path(f'/proc/{process.pid}/stat').read_text().rpartition(') ')[2].split()
            if (self.identity is None or int(stat[19]) != self.identity['start_ticks']
                    or os.getpgid(process.pid) != self.identity['pgid']
                    or self.identity['pgid'] != process.pid):
                return False
        except (OSError, ValueError, IndexError):
            return False
        def send(sig):
            if self.spawn:
                process.send_signal(sig)
            else:
                os.killpg(process.pid, sig)

        send(signal.SIGINT)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            send(signal.SIGTERM)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                send(signal.SIGKILL)
                process.wait(timeout=2)
            # Escalation cannot establish a clean recorder shutdown.
            return False
        if process.returncode not in (0, -signal.SIGINT):
            return False
        destination = self.root / session_id
        metadata = destination / 'metadata.yaml'
        if not metadata.is_file():
            return False
        try:
            content = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
            if content.get('message_count', 0) <= 0 or not content.get('relative_file_paths'):
                return False
            for name in content.get('relative_file_paths', []):
                if Path(name).name != name or not (destination / name).is_file():
                    return False
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
            return False
        result = subprocess.run(['ros2', 'bag', 'info', str(destination)],
                                capture_output=True, text=True, timeout=5)
        if result.returncode:
            return False
        self.process = None
        self.session_id = None
        return True
