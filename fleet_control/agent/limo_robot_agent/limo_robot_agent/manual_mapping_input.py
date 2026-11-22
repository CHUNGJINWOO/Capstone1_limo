"""Joystick input owned by the Agent SLAM session; output precedes collision monitor.

This component never starts a driver, unlocks motors or submits a Nav2 goal.
"""
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from .joystick_filter import JoystickConfig, joystick_command


class ManualMappingInput:
    def __init__(self, robot, slam, config, topic='/joy', output='/cmd_vel_pre_cm'):
        if output != '/cmd_vel_pre_cm':
            raise ValueError('manual input must enter the collision-monitor input')
        self.robot, self.slam, self.config = robot, slam, config
        self.sample = None
        self.last_reason = 'joystick_unobserved'
        self.publisher = robot.create_publisher(Twist, output, 10)
        self.subscription = robot.create_subscription(Joy, topic, self.receive, 10)
        self.timer = robot.create_timer(.05, self.tick)
        self.enabled = True

    def receive(self, msg):
        # Receive age is a local watchdog, not proof of device identity.
        self.sample = (list(msg.axes), list(msg.buttons), time.monotonic())

    def snapshot(self):
        age = None if self.sample is None else time.monotonic() - self.sample[2]
        return {'enabled': self.enabled, 'connected': age is not None and age < self.config.timeout_s,
                'age_s': age, 'detail': self.last_reason, 'input_kind': 'ROS_JOY',
                'output_topic': '/cmd_vel_pre_cm'}

    def _motion_permitted(self):
        """Require fresh r8 Gate idle evidence and a live Fleet lease, in SIM only."""
        try:
            if (self.robot.mode != 'sim' or self.robot.simulated is not True
                    or self.robot.allow_sim_safety_gate is not True or not self.robot._use_sim_time()
                    or self.robot.execution_allowed() is not True or self.robot.active):
                return False
            session = self.robot.session_fn(); lease = session.get('lease') or {}
            if (session.get('connected') is not True or session.get('reconciliation_required') is not False
                    or type(lease.get('epoch')) is not int or type(lease.get('expires_at')) not in (int, float)
                    or lease['expires_at'] <= time.time()):
                return False
            gate = self.robot._gate_evaluation(); evidence = gate.get('nav_evidence') or {}
            if (gate.get('contract_version') != 'gate-status/1.2-sim'
                    or gate.get('trust') != 'UNAUTHENTICATED_SIM_ONLY' or gate.get('contract_met') is not True
                    or gate.get('r7_invalid') is not False or gate.get('safety_state') != 'NORMAL'
                    or gate.get('single_publisher_graph') is not True or gate.get('rx_fresh') is not True
                    or gate.get('source_fresh') is not True or gate.get('fault_latched') is not False
                    or evidence.get('scope_version') != 1 or evidence.get('observed') is not True
                    or evidence.get('all_idle_proven') is not True or evidence.get('active_uuids')):
                return False
            servers = evidence.get('servers') or {}
            for action in ('navigate_to_pose', 'navigate_through_poses', 'follow_path', 'spin', 'backup'):
                record = servers.get(action) or {}; pubs = self.robot.get_publishers_info_by_topic('/' + action + '/_action/status')
                if (record.get('verdict') not in ('IDLE_PROVEN_EL', 'IDLE_PROVEN_ES') or len(pubs) != 1
                        or bytes(pubs[0].endpoint_gid).hex()[:16] != record.get('writer_gid')):
                    return False
            return True
        except Exception:
            return False

    def tick(self):
        state = self.slam.snapshot()
        # The same authority, safety and Nav2 observations must remain valid throughout motion.
        manual = self.enabled and state.get('status') == 'RUNNING' and self._motion_permitted()
        # A manual-only session may not share velocity ownership with navigation.
        if any(self.robot.count_publishers(t) for t in ('/cmd_vel_smoothed', '/cmd_vel_nav')):
            manual = False
        axes, buttons, received = self.sample or ([], [], float('-inf'))
        v, w, self.last_reason = joystick_command(self.config, axes, buttons,
                                                   received_at=received, now=time.monotonic(),
                                                   manual_enabled=manual)
        # Do not contend with another source, including a zero stream.
        if self.robot.count_publishers('/cmd_vel_pre_cm') != 1:
            self.last_reason = 'velocity_source_conflict'
            return
        cmd = Twist(); cmd.linear.x = v; cmd.angular.z = w
        self.publisher.publish(cmd)

    def close(self):
        self.enabled = False
        self.timer.cancel()
        self.publisher.publish(Twist())
        self.robot.destroy_subscription(self.subscription)
        self.robot.destroy_publisher(self.publisher)
