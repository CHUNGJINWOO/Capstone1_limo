"""Isolated bench velocity arbiter. Its only output goes through a locked Gate."""
import math
import time

from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType

from .manual_web_bench import select_velocity


def select_locked(mode, *, locked, nav, manual, terminal):
    if not locked:
        return 0., 0.
    if mode == 'AUTO' and nav is not None:
        v, w = nav
        if math.isfinite(v) and math.isfinite(w):
            return max(-.10, min(.10, v)), max(-.30, min(.30, w))
    if mode == 'MANUAL' and manual is not None and terminal:
        return select_velocity('WEB_MANUAL', allowed=True, nav_terminal=True, manual=manual)
    return 0., 0.


class VelocityArbiter:
    INPUT = '/fleet/cmd_vel_nav'
    SELECTED = '/fleet/cmd_vel_selected'
    RAW = '/cmd_vel_raw'
    LOCKED = '/cmd_vel_locked'

    def __init__(self, robot, agent, bench):
        self.robot, self.agent, self.bench = robot, agent, bench
        self.nav_sample = None
        self.mode = 'LOCKED'
        self.mode_since = time.monotonic()
        self.output = (0., 0.)
        self.pub = robot.create_publisher(Twist, self.SELECTED, 1)
        self.sub = robot.create_subscription(Twist, self.INPUT, self.receive_nav, 1)
        self.timer = robot.create_timer(.05, self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive_nav(self, msg):
        self.nav_sample = time.monotonic(), (msg.linear.x, msg.angular.z)

    def snapshot(self):
        return {'mode': self.mode, 'motor_output': False,
                'selected_linear': self.output[0], 'selected_turn': self.output[1]}

    def graph_locked(self):
        r = self.robot
        def nodes(query, topic):
            return sorted((i.node_namespace, i.node_name) for i in query(topic))

        pubs, subs = r.get_publishers_info_by_topic, r.get_subscriptions_info_by_topic
        nav_sources = set(nodes(pubs, self.INPUT))
        return (not nodes(pubs, '/cmd_vel') and not nodes(subs, '/cmd_vel')
                and nodes(pubs, self.LOCKED) == [('/', 'stall_detector_node')]
                and not nodes(subs, self.LOCKED)
                and nodes(subs, self.RAW) == [('/', 'stall_detector_node')]
                and nodes(pubs, self.RAW) == [('/', 'velocity_smoother')]
                and nodes(subs, self.SELECTED) == [('/', 'velocity_smoother')]
                and nodes(pubs, self.SELECTED) == [('/', 'fleet_robot_agent')]
                and ('/', 'controller_server') in nav_sources
                and nav_sources <= {('/', 'controller_server'), ('/', 'behavior_server')})

    def update(self):
        now = time.monotonic()
        active = self.robot.active
        nav = self.robot._nav2_goals()
        terminal = (active is None and nav['publishers'] == 1 and nav['active_count'] == 0
                    and nav['received_status_array'] is True and self.robot.follow_path_idle())
        recent_gate = self.robot._age('gate')
        locked = (self.agent.connected and self.graph_locked()
                  and recent_gate is not None and recent_gate < .5
                  and (self.robot.gate or {}).get('state') == 'NORMAL'
                  and (self.robot.gate or {}).get('cmd_blocked') is False)
        manual = self.bench.snapshot() if self.bench else {}
        manual = manual if manual.get('session_id') else None
        auto = (active and active.get('handle') is not None and not active.get('cancel_requested')
                and active.get('owner') == 'frontier' and self.robot.frontier_allowed(active['mission_id']))
        mode = ('AUTO' if auto else 'MANUAL' if terminal and manual and manual['deadman'] else 'IDLE') if locked else 'LOCKED'
        if mode != self.mode:
            self.mode, self.mode_since = mode, now
        sample = self.nav_sample
        fresh = sample is not None and self.mode_since <= sample[0] and now - sample[0] <= .25
        vx, wz = select_locked(mode, locked=locked, nav=sample[1] if fresh else None,
                               manual=manual, terminal=terminal)
        msg = Twist()
        msg.linear.x, msg.angular.z = vx, wz
        self.output = vx, wz
        self.pub.publish(msg)
