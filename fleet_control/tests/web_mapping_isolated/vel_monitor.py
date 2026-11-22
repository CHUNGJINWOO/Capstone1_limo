"""Record velocity commands and Nav2 goal activity (isolated domain only). usage: vel_monitor.py OUT.jsonl"""
import json, os, sys, time
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

TERMINAL = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)


class Monitor(Node):
    def __init__(self, path):
        super().__init__('iso_vel_monitor')
        self.f = open(path, 'a', buffering=1)
        # No /cmd_vel subscription: any /cmd_vel consumer makes the mapping worker refuse (motor path not locked).
        for topic in ('/cmd_vel_raw', '/cmd_vel_locked', '/fleet/cmd_vel_nav', '/fleet/nav_cmd_vel'):
            self.create_subscription(Twist, topic, lambda m, t=topic: self.rec(t, {'vx': m.linear.x, 'wz': m.angular.z}), 20)
        self.create_timer(1.0, lambda: self.rec('/cmd_vel#graph', {'publishers': self.count_publishers('/cmd_vel'), 'subscribers': self.count_subscribers('/cmd_vel')}))
        self.create_subscription(String, '/fleet/motion_watchdog/status', lambda m: self.rec('watchdog', json.loads(m.data)), 10)
        self.create_subscription(String, '/stall_detector/status_json', lambda m: self.gate(json.loads(m.data)), 10)
        self.last_gate = None

    def gate(self, d):
        keep = {k: d.get(k) for k in ('state', 'cmd_blocked', 'motion_watchdog_ok', 'motion_watchdog_state', 'cmd_vel_raw_publishers', 'require_motion_watchdog')}
        if keep != self.last_gate:               # Gate status is 10 Hz: record changes only
            self.last_gate = keep
            self.rec('gate', keep)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        for topic in ('/navigate_to_pose/_action/status', '/follow_path/_action/status'):
            self.create_subscription(GoalStatusArray, topic, lambda m, t=topic: self.rec(
                t, {'active': sum(1 for g in m.status_list if g.status not in TERMINAL),
                    'uuids': [bytes(g.goal_info.goal_id.uuid).hex() for g in m.status_list if g.status not in TERMINAL]}), qos)

    def rec(self, topic, data):
        self.f.write(json.dumps({'t': time.time(), 'topic': topic, **data}) + '\n')


if __name__ == '__main__':
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    n = Monitor(sys.argv[1])
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
