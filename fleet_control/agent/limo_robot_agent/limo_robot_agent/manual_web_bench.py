"""Observe real web Joy input under motor lock. This adapter can only publish zero."""
import math
import threading
import time

from geometry_msgs.msg import Twist

from . import topics
from .joystick_filter import JoystickConfig, joystick_command


class ManualWebBench:
    INPUT = topics.CMD_VEL_RAW
    GATE = topics.GATE_STATUS
    OUTPUT = topics.CMD_VEL_LOCKED
    RAW_ZERO_PUBLISHER_IN_REAL = False      # real mode: observe only, no /cmd_vel_raw publisher (changing this needs a Gate writer contract)

    def __init__(self, robot, agent, isolated_arbiter=False):
        self.isolated_arbiter = isolated_arbiter is True
        self.robot = robot
        self.agent = agent
        self.config = JoystickConfig(1, 0, 4, .10, .30, .25)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.sample = None
        self.publisher = None
        self.state = {'status': 'WAITING', 'motor_output': False}
        self.timer = robot.create_timer(.05, self.tick)
        self.thread = threading.Thread(target=self.poll, daemon=True)
        self.thread.start()

    def poll(self):
        while not self.stop.is_set():
            started = time.monotonic()
            sample = None
            try:
                if self.agent.is_execution_allowed():
                    response = self.agent.http.post('/agent/manual-bench', {
                        'robot_id': self.agent.robot_id, 'agent_boot_id': self.agent.boot_id,
                    }, timeout=.18)
                    sample = response.get('sample')
                    if isinstance(sample, dict):
                        ttl = sample.get('ttl_s')
                        if type(ttl) not in (float, int) or not math.isfinite(ttl) or not 0 < ttl <= .25:
                            sample = None
                        else:
                            sample = dict(sample, expires=started + ttl)
            except Exception:
                # HttpError and malformed data both mean there is no current authorization.
                pass
            with self.lock:
                self.sample = sample if isinstance(sample, dict) else None
            self.stop.wait(max(0., started + .05 - time.monotonic()))

    def _graph_blocker(self):
        r = self.robot
        if r.count_publishers(topics.CMD_VEL):
            return 'motor_topic_has_publisher'
        for topic in (self.OUTPUT, self.GATE):
            pubs = r.get_publishers_info_by_topic(topic)
            if len(pubs) != 1 or pubs[0].node_name != topics.GATE_NODE:
                return 'gate_graph_unconfirmed'
        sinks = r.get_subscriptions_info_by_topic(self.INPUT)
        if sum(s.node_name == topics.GATE_NODE for s in sinks) != 1:
            return 'gate_input_unconfirmed'
        pubs = r.get_publishers_info_by_topic(self.INPUT)
        if any((p.node_name, p.node_namespace) != (r.get_name(), r.get_namespace()) for p in pubs):
            return 'velocity_source_conflict'
        if len(pubs) > int(self.publisher is not None):
            return 'velocity_source_conflict'
        return None

    def tick(self):
        with self.lock:
            sample = self.sample
        robot = self.robot
        now = time.monotonic()
        if getattr(self, 'isolated_arbiter', False):
            # No raw publisher: the separate selector publishes through the Watchdog.
            fresh = sample is not None and now < sample['expires'] and self.agent.is_execution_allowed()
            axes = [sample.get('turn'), sample.get('linear')] if fresh else []
            buttons = [0] * 4 + [int(sample.get('deadman') is True)] if fresh else []
            v, w, reason = joystick_command(self.config, axes, buttons, received_at=now, now=now, manual_enabled=fresh)
            with self.lock:
                self.state = {'status': reason, 'motor_output': False, 'requested_linear': max(0., v),
                              'requested_turn': w, 'deadman': fresh and sample.get('deadman') is True,
                              'session_id': sample.get('session_id') if fresh else None,
                              'sample_seq': sample.get('seq') if fresh else None,
                              'expires': sample.get('expires') if fresh else None}
            return
        graph_blocker = self._graph_blocker()
        with robot.lock:
            gate = robot.gate if isinstance(robot.gate, dict) else {}
            odom_age, gate_age = robot._age('odom'), robot._age('gate')
            healthy = (gate.get('state') == 'NORMAL' and gate.get('wheel_odom_stale') is False
                       and gate.get('cmd_blocked') is False and robot.active is None
                       and odom_age is not None and 0 <= odom_age < .38
                       and gate_age is not None and 0 <= gate_age < .30)
        fresh = sample is not None and now < sample['expires'] and self.agent.is_execution_allowed()
        axes = [sample.get('turn'), sample.get('linear')] if fresh else []
        buttons = [0] * 4 + [int(sample.get('deadman') is True)] if fresh else []
        linear, turn, reason = joystick_command(self.config, axes, buttons, received_at=now,
                                                now=now, manual_enabled=fresh)
        # 2026-11-22: on the real robot the Gate input (/cmd_vel_raw) already has Nav2 writers and the Gate HEAD has no input-writer binding,
        # so this Agent must never become a second writer there, whatever the graph snapshot shows at that instant.
        raw_blocked = getattr(robot, 'mode', None) == 'real' and not self.RAW_ZERO_PUBLISHER_IN_REAL
        if (graph_blocker or raw_blocked) and self.publisher is not None:
            robot.destroy_publisher(self.publisher)
            self.publisher = None
        if not graph_blocker and not raw_blocked and fresh and healthy and self.publisher is None:
            self.publisher = robot.create_publisher(Twist, self.INPUT, 10)
        if self.publisher is not None:
            self.publisher.publish(Twist())
        with self.lock:
            self.state = {
                'status': graph_blocker or ('gate_or_odom_not_ready' if not healthy else
                                             (reason if fresh else 'input_expired')),
                'motor_output': False, 'gate_zero_published': self.publisher is not None, 'raw_publisher_blocked_real': raw_blocked,
                'requested_linear': max(0., linear), 'requested_turn': turn,
                'sample_seq': sample.get('seq') if fresh else None,
                'session_id': sample.get('session_id') if fresh else None,
            }

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        self.timer.cancel()
        if self.publisher is not None:
            self.publisher.publish(Twist())
            self.robot.destroy_publisher(self.publisher)
