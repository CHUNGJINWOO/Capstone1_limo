"""Candidate-only Fleet mission coordinator; no ROS action client.

This is a SIM integration building block, not an enabled AUTO mapping mode.
The caller supplies authenticated Fleet API operations and durable journal evidence.
Unknown submission or terminal ownership always halts progression.
"""
import threading
import uuid


class FrontierFleetCoordinator:
    TERMINAL = {'SUCCEEDED', 'FAILED', 'CANCELED'}

    def __init__(self, submit, cancel):
        self.submit = submit
        self.cancel = cancel
        self.lock = threading.RLock()
        self.stop_requested = threading.Event()
        self.state = 'IDLE'
        self.current = None
        self.last_result = None
        self.error = None

    def start(self):
        with self.lock:
            if self.current or self.state not in ('IDLE', 'STOPPED'):
                raise RuntimeError('previous goal unresolved')
            self.stop_requested.clear()
            self.state = 'SEARCHING'
            self.error = None

    def offer(self, goal):
        with self.lock:
            if self.stop_requested.is_set() or self.state != 'SEARCHING' or self.current:
                return False
            # Reserve before the network call. Ambiguous transport outcomes never
            # permit another request or an untargeted cancellation.
            key = 'frontier-' + uuid.uuid4().hex
            self.current = {'mission_id': None, 'goal_uuid': None, 'goal': dict(goal), 'idempotency_key': key}
            self.state = 'SUBMITTING'
            try:
                result = self.submit(dict(goal), key)
                mission_id = result.get('mission_id')
                if not isinstance(mission_id, str) or not mission_id:
                    raise RuntimeError('Fleet response missing mission ID')
                self.current['mission_id'] = mission_id
                self.state = 'ACTIVE'
                if self.stop_requested.is_set():
                    self.cancel(mission_id)
                    self.state = 'CANCELING'
                return True
            except Exception as exc:
                self.state = 'UNKNOWN'
                self.error = 'submission or targeted cancel uncertain: ' + str(exc)
                return False

    def observe(self, mission, execution, agent_boot_id):
        with self.lock:
            if not self.current or not self.current.get('mission_id'):
                return None
            if mission.get('mission_id') != self.current['mission_id']:
                return None
            active = execution.get('active') or {}
            if active.get('mission_id') == self.current['mission_id']:
                observed = active.get('goal_uuid')
                if observed:
                    previous = self.current['goal_uuid']
                    if previous and previous != observed:
                        self.state = 'UNKNOWN'; self.error = 'owned goal UUID changed'; return None
                    self.current['goal_uuid'] = observed
            status = mission.get('status')
            if status not in self.TERMINAL:
                return None
            # Fleet terminal alone is insufficient: bind it to the Agent's
            # durable journal result for this mission's navigate command.
            command_ids = {c.get('command_id') for c in mission.get('commands', []) if c.get('type') == 'navigate_to_pose'}
            goals = execution.get('nav2_goals') or {}
            if goals.get('known') is not True or goals.get('active_count') != 0 or execution.get('active') is not None:
                return None
            terminal = [r for r in execution.get('journal_terminals', [])
                        if r.get('command_id') in command_ids and r.get('mission_id') == self.current['mission_id']
                        and r.get('terminal_status') == status and r.get('status') == status
                        and r.get('terminal_source') == 'nav2_action_result'
                        and r.get('terminal_agent_boot_id') == agent_boot_id and isinstance(agent_boot_id, str)
                        and isinstance(r.get('goal_uuid'), str) and len(r['goal_uuid']) == 32
                        and all(c in '0123456789abcdef' for c in r['goal_uuid'])]
            if len(terminal) != 1 or (self.current['goal_uuid'] and terminal[0]['goal_uuid'] != self.current['goal_uuid']):
                self.state = 'UNKNOWN'; self.error = 'terminal journal evidence missing or inconsistent'; return None
            result = dict(self.current, goal_uuid=terminal[0]['goal_uuid'], status=status)
            self.last_result = result; self.current = None
            self.state = 'STOPPED' if self.stop_requested.is_set() else 'SEARCHING'
            return result

    def stop(self):
        # Set before taking the lock so a concurrent in-flight offer cannot
        # begin another mission after STOP is requested.
        self.stop_requested.set()
        with self.lock:
            if self.current is None:
                self.state = 'STOPPED'; return
            mission_id = self.current.get('mission_id')
            if not mission_id:
                self.state = 'UNKNOWN'; self.error = 'submission outcome unknown'; return
            if self.state == 'CANCELING':
                return
            try:
                self.cancel(mission_id)
                self.state = 'CANCELING'  # ACK is not a terminal Nav2 result.
            except Exception as exc:
                self.state = 'UNKNOWN'; self.error = 'targeted cancel uncertain: ' + str(exc)
