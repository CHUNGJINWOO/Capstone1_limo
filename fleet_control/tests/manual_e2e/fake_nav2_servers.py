"""Isolated stand-ins for Nav2's action servers with real rclpy ActionServer semantics.

NavigateToPose on node 'bt_navigator' and FollowPath on node 'controller_server'. Unlike a periodic status
fixture, an rclpy/rclcpp ActionServer publishes '<action>/_action/status' only on goal state changes, and an idle
server never publishes it (checked against the Humble C++ and Python tutorial servers). Goals run until canceled
(cancel is accepted, like Nav2) or until '/fake_nav2/finish' (std_msgs/String) names the action.
No velocity is published. Test-only; never launch on a robot.
"""
import os
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import FollowPath, NavigateToPose
from std_msgs.msg import String

assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0', '77')


def server(node, action_type, name, label, finish):
    def execute(handle):
        node.get_logger().info(f'{label} goal executing')
        while rclpy.ok():
            if handle.is_cancel_requested:
                handle.canceled()
                node.get_logger().info(f'{label} goal canceled')
                return action_type.Result()
            if finish.get(label):
                finish[label] = False
                handle.succeed()
                node.get_logger().info(f'{label} goal succeeded')
                return action_type.Result()
            time.sleep(0.02)
        return action_type.Result()

    return ActionServer(node, action_type, name, execute_callback=execute,
                        goal_callback=lambda _g: GoalResponse.ACCEPT,
                        cancel_callback=lambda _g: CancelResponse.ACCEPT,
                        callback_group=ReentrantCallbackGroup())


def main():
    which = set(sys.argv[1:] or ['nav', 'follow'])
    rclpy.init()
    finish = {}
    nodes = []
    if 'nav' in which:
        nav = rclpy.create_node('bt_navigator'); nodes.append(nav)
        server(nav, NavigateToPose, 'navigate_to_pose', 'nav', finish)
        nav.create_subscription(String, '/fake_nav2/finish', lambda m: finish.__setitem__(m.data, True), 10)
    if 'follow' in which:
        ctl = rclpy.create_node('controller_server'); nodes.append(ctl)
        server(ctl, FollowPath, 'follow_path', 'follow', finish)
        ctl.create_subscription(String, '/fake_nav2/finish', lambda m: finish.__setitem__(m.data, True), 10)
    ex = MultiThreadedExecutor(num_threads=4)
    for n in nodes:
        ex.add_node(n)
    print('fake nav2 servers ready:', sorted(which), flush=True)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
