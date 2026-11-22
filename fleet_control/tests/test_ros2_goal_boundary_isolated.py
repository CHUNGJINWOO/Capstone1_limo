"""Real ROS action isolation experiment; the mock server intentionally accepts two goals."""
import os
import socket
import threading
import time
import unittest

if os.environ.get('CAPSTONE_PRIVATE_ROS_TEST') != '1' or {n for _, n in socket.if_nameindex()} != {'lo'}:
    raise RuntimeError('Run only inside the private network and mount namespaces')

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def wait_for(predicate, seconds=8):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if predicate():
            return True
        time.sleep(.01)
    return False


class BoundaryTest(unittest.TestCase):
    def test_direct_client_bypasses_and_uuid_cancel_is_scoped(self):
        rclpy.init()
        server_node, a, b = (Node(n) for n in ('mock_nav_server', 'owner_client', 'uncontrolled_client'))
        group = ReentrantCallbackGroup()
        observed = []

        def execute(handle):
            observed.append(bytes(handle.goal_id.uuid).hex())
            until = time.monotonic() + 15
            while not handle.is_cancel_requested and time.monotonic() < until:
                time.sleep(.01)
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
            return NavigateToPose.Result()

        server = ActionServer(server_node, NavigateToPose, '/test_only_nav', execute,
                              cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        client_a = ActionClient(a, NavigateToPose, '/test_only_nav')
        client_b = ActionClient(b, NavigateToPose, '/test_only_nav')
        executor = MultiThreadedExecutor(num_threads=6)
        for node in (server_node, a, b):
            executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            self.assertTrue(client_a.wait_for_server(timeout_sec=5))
            first = client_a.send_goal_async(NavigateToPose.Goal())
            self.assertTrue(wait_for(first.done))
            owner = first.result()
            self.assertTrue(owner.accepted)
            # This second client never goes through Agent or a safety contract.
            direct = client_b.send_goal_async(NavigateToPose.Goal())
            self.assertTrue(wait_for(direct.done))
            other = direct.result()
            self.assertTrue(other.accepted)
            self.assertTrue(wait_for(lambda: len(observed) == 2))
            owner_id, other_id = bytes(owner.goal_id.uuid).hex(), bytes(other.goal_id.uuid).hex()
            self.assertNotEqual(owner_id, other_id)
            owner_result, other_result = owner.get_result_async(), other.get_result_async()
            cancel = owner.cancel_goal_async()
            self.assertTrue(wait_for(cancel.done))
            canceled = [bytes(item.goal_id.uuid).hex() for item in cancel.result().goals_canceling]
            self.assertEqual(canceled, [owner_id])
            self.assertTrue(wait_for(owner_result.done))
            self.assertEqual(owner_result.result().status, GoalStatus.STATUS_CANCELED)
            self.assertFalse(other_result.done(), 'other goal unexpectedly terminated')
            print('EVIDENCE direct_bypass=accepted scoped_cancel=owner_only owner_terminal=CANCELED other_active=true', flush=True)
            cleanup = other.cancel_goal_async()
            self.assertTrue(wait_for(cleanup.done))
            self.assertTrue(wait_for(other_result.done))
        finally:
            executor.shutdown(timeout_sec=5)
            thread.join(timeout=5)
            client_a.destroy(); client_b.destroy(); server.destroy()
            for node in (a, b, server_node):
                node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main(verbosity=2)
