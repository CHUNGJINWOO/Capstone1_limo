"""Mock-only browser input. Reports the selected velocity without a ROS publisher."""
import math
import threading
import time
from .core import HttpError


def select_velocity(mode, *, nav_terminal, allowed, manual=None, nav=None):
    """Select one fresh, authorized input; every inactive case produces zero."""
    if not allowed:
        return 0., 0.
    if mode == 'NAVIGATION':
        return nav if nav is not None else (0., 0.)
    if mode != 'WEB_MANUAL' or not nav_terminal or not manual or manual.get('deadman') is not True:
        return 0., 0.
    v, w = manual.get('linear'), manual.get('turn')
    if type(v) not in (int, float) or type(w) not in (int, float):
        return 0., 0.
    if not (0 <= v <= .10 and -.30 <= w <= .30):
        return 0., 0.
    return v, w


class ManualWebBench:
    def __init__(self, robot, agent):
        self.robot, self.agent = robot, agent
        self.stop = threading.Event()
        self.state = {'linear': 0., 'turn': 0., 'motor_output': False}
        self.thread = threading.Thread(target=self.poll, daemon=True)
        self.thread.start()

    def poll(self):
        while not self.stop.is_set():
            started = time.monotonic()
            sample = None
            if self.agent.connected:
                try:
                    response = self.agent.http.post('/agent/manual-bench', {
                        'robot_id': self.agent.robot_id, 'agent_boot_id': self.agent.boot_id,
                    }, timeout=.15)
                    candidate = response.get('sample') or {}
                    ttl = candidate.get('ttl_s')
                    if type(ttl) in (int, float) and math.isfinite(ttl) and 0 < ttl <= .25:
                        if time.monotonic() < started + ttl:
                            sample = candidate
                except (OSError, ValueError, HttpError):
                    pass
            nav = self.robot._nav2_goals()
            terminal = nav['publishers'] == 1 and nav['active_count'] == 0 and self.robot.active is None
            v, w = select_velocity('WEB_MANUAL', nav_terminal=terminal,
                                   allowed=self.agent.connected, manual=sample)
            self.state = {'linear': v, 'turn': w, 'motor_output': False,
                          'deadline': started + sample['ttl_s'] if sample else 0.,
                          'sample_seq': sample['seq'] if sample else None,
                          'session_id': sample['session_id'] if sample else None,
                          'deadman': bool(sample and sample.get('deadman') is True and terminal)}
            self.stop.wait(max(0., started + .05 - time.monotonic()))

    def snapshot(self):
        state = self.state.copy()
        if not self.agent.connected or time.monotonic() >= state.pop('deadline', 0.):
            return {'linear': 0., 'turn': 0., 'motor_output': False}
        return state

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
