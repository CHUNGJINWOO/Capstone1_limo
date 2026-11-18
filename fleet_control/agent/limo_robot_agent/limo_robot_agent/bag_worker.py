"""Opt-in isolated ROS bag worker, fenced by Fleet session and durable journal."""
import json
import shutil
from pathlib import Path
import threading
import time

from .bag_supervisor import SupervisedBag
from .local_process import write_json


class BagWorker:
    def __init__(self, robot, agent, state_dir):
        self.robot, self.agent = robot, agent
        self.root = Path(state_dir) / 'bag_worker'
        self.root.mkdir(parents=True, exist_ok=True)
        self.path = self.root / 'journal.json'
        self.bag = SupervisedBag(self.root)
        self.status = 'IDLE'
        self.journal = {}
        if self.path.exists():
            try:
                self.journal = json.loads(self.path.read_text())
                if self.journal.get('active_session') or self.journal.get('pending'):
                    self.status = 'UNKNOWN'
            except (ValueError, OSError):
                self.status = 'UNKNOWN'
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def save(self):
        write_json(self.path, self.journal)

    def snapshot(self):
        return {'status': self.status, 'reason': self.journal.get('safety_stop_reason')}

    def request(self, action, **fields):
        return self.agent.http.post('/agent/operation', {
            'robot_id': self.agent.robot_id, 'agent_boot_id': self.agent.boot_id,
            'kind': 'bag', 'action': action, **fields}, timeout=1)

    def run(self):
        while not self.stop.wait(.15):
            if self.bag.process is not None and self.bag.process.poll() is not None:
                self.status = 'UNKNOWN'
                try:
                    state = self.bag.reap()
                    self.journal['safety_stop_reason'] = state.get('reason', 'supervisor_lost')
                    self.save()
                except OSError:
                    self.journal['safety_stop_reason'] = 'journal_write_failed'
            if not self.agent.connected:
                continue
            try:
                response = self.request('poll', session_id=self.bag.session_id)
                if response.get('renew') is True and self.status == 'RECORDING':
                    self.bag.renew()
                if response.get('command'):
                    self.execute(response['command'])
            except Exception:
                # A missing response cannot renew the supervisor or authorize a retry.
                continue

    def execute(self, command):
        cid, sid, epoch = command.get('command_id'), command.get('session_id'), command.get('epoch')
        if (command.get('robot_id') != self.agent.robot_id or command.get('kind') != 'bag'
                or not isinstance(cid, str) or not cid.isascii() or not cid.isalnum() or not isinstance(sid, str)
                or not sid.isascii() or not sid.isalnum() or type(epoch) is not int or command.get('expires_at', 0) <= time.time()):
            return
        if command.get('action') not in ('START', 'STOP'):
            return
        if self.status in ('UNKNOWN', 'LOST') or self.journal.get('pending'):
            return
        if command['action'] == 'STOP' and (self.journal.get('active_session') != sid
                                              or self.journal.get('epoch') != epoch):
            return
        if command['action'] == 'START' and self.journal.get('active_session'):
            return
        ref = {'session_id': sid, 'command_id': cid}
        accepted = self.request('accept', **ref)
        if accepted.get('accepted') is not True:
            return
        self.journal['pending'] = cid
        self.save()
        outcome, evidence, detail = 'UNKNOWN', {}, 'execution outcome unconfirmed'
        if command['action'] == 'START':
            try:
                if shutil.disk_usage(self.root).free < 1024 * 1024 * 1024:
                    raise OSError('insufficient free space')
                self.bag.start(sid, ['/map'])
                deadline = time.monotonic() + 7
                while time.monotonic() < deadline and self.agent.connected and not self.stop.is_set():
                    if self.bag.process.poll() is not None:
                        break
                    self.bag.renew()  # bounded by the START deadline until Fleet sees the result
                    if self.bag.identity and any(
                            info.node_name == 'fleet_bag_' + sid and info.node_namespace == '/'
                            for info in self.robot.get_subscriptions_info_by_topic('/map')):
                        self.journal['process'] = self.bag.identity
                        self.journal.update(active_session=sid, epoch=epoch)
                        self.status = 'RECORDING'
                        outcome, evidence, detail = 'OK', {'recorder_running': True}, 'recorder subscription observed'
                        break
                    time.sleep(.1)
            except Exception as exc:
                detail = type(exc).__name__
        elif command['action'] == 'STOP':
            try:
                if self.bag.stop(sid):
                    self.journal.pop('active_session', None)
                    self.status = 'IDLE'
                    outcome, evidence, detail = 'OK', {'owned_recorder_stopped': True, 'bag_verified': True}, 'metadata and bag info verified'
            except Exception as exc:
                detail = type(exc).__name__
        if outcome != 'OK':
            self.status = 'UNKNOWN'
            self.bag.close()
        self.journal.update(pending=None if outcome == 'OK' else cid, last_command=cid,
                            last_outcome=outcome)
        self.save()
        try:
            self.request('result', **ref, outcome=outcome, evidence=evidence, detail=detail)
        except Exception:
            self.status = 'UNKNOWN'
            self.bag.close()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=12)
        self.bag.close()
