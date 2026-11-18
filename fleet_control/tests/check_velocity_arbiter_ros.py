"""Exercise the velocity selector with two isolated ROS inputs. No control outputs."""
import json
import sys
import time
from pathlib import Path

import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agent/limo_robot_agent'))
from limo_robot_agent.manual_web_bench import select_velocity


def main():
    rclpy.init()
    fixture = rclpy.create_node('velocity_fixture')
    receiver = rclpy.create_node('velocity_arbiter_probe')
    nav = fixture.create_publisher(Twist, '/fleet_bench/nav', 1)
    manual = fixture.create_publisher(String, '/fleet_bench/manual', 1)
    observed = {}
    receiver.create_subscription(Twist, '/fleet_bench/nav', lambda m: observed.update(
        nav=(time.monotonic(), (m.linear.x, m.angular.z))), 1)
    receiver.create_subscription(String, '/fleet_bench/manual', lambda m: observed.update(
        manual=(time.monotonic(), json.loads(m.data))), 1)

    def selected(mode, terminal):
        now = time.monotonic()
        fresh = {name: data for name, (received, data) in observed.items() if now - received < .25}
        return select_velocity(mode, nav_terminal=terminal, allowed=True, **fresh)

    try:
        deadline = time.monotonic() + 4
        while set(observed) != {'nav', 'manual'} and time.monotonic() < deadline:
            msg = Twist()
            msg.linear.x = .08
            nav.publish(msg)
            manual.publish(String(data='{"linear":0.0,"turn":0.3,"deadman":true}'))
            for _ in range(2):
                rclpy.spin_once(receiver, timeout_sec=.02)
        assert set(observed) == {'nav', 'manual'}
        assert selected('NAVIGATION', False) == (.08, 0.)
        assert selected('WEB_MANUAL', False) == (0., 0.)
        assert selected('WEB_MANUAL', True) == (0., .3)
        time.sleep(.26)
        assert selected('WEB_MANUAL', True) == (0., 0.)
        assert selected('NAVIGATION', True) == (0., 0.)
        assert receiver.count_publishers('/cmd_vel') == 0
        assert receiver.count_publishers('/cmd_vel_raw') == 0
        print('PASS isolated ROS: select one input; terminal fence and stale inputs choose zero')
    finally:
        receiver.destroy_node()
        fixture.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
