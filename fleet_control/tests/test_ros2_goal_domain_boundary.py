"""Test-only ROS domain and cancel-all experiment; never run on a live ROS network."""
import os
import socket
import subprocess
import sys
import threading
import time
import unittest

if os.environ.get('CAPSTONE_PRIVATE_ROS_TEST') != '1' or {n for _, n in socket.if_nameindex()} != {'lo'}:
    raise RuntimeError('private network and mount isolation required')

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal


def wait(pred, timeout=7):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred(): return True
        time.sleep(.01)
    return False


def worker(role):
    rclpy.init()
    node = Node('test_domain_' + role)
    if role == 'server':
        def execute(handle):
            handle.succeed()
            return NavigateToPose.Result()
        server = ActionServer(node, NavigateToPose, '/phase14_test_nav', execute)
        print('SERVER_READY', flush=True)
        rclpy.spin(node)
        server.destroy()
    else:
        client = ActionClient(node, NavigateToPose, '/phase14_test_nav')
        if not client.wait_for_server(timeout_sec=3):
            print('NOT_DISCOVERED', flush=True)
            return 3
        future = client.send_goal_async(NavigateToPose.Goal())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or not future.result().accepted:
            print('NOT_ACCEPTED', flush=True)
            return 4
        print('DIRECT_ACCEPTED', flush=True)
        client.destroy()
    node.destroy_node()
    rclpy.shutdown()
    return 0


class Tests(unittest.TestCase):
    def test_domain_isolation_is_not_authorization(self):
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = '222'
        server = subprocess.Popen([sys.executable, __file__, 'server'], env=env,
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            self.assertEqual(server.stdout.readline().strip(), 'SERVER_READY')
            other = env.copy(); other['ROS_DOMAIN_ID'] = '223'
            blocked = subprocess.run([sys.executable, __file__, 'client'], env=other,
                                     capture_output=True, text=True, timeout=12)
            self.assertEqual(blocked.returncode, 3, blocked.stdout + blocked.stderr)
            self.assertIn('NOT_DISCOVERED', blocked.stdout)
            same = subprocess.run([sys.executable, __file__, 'client'], env=env,
                                  capture_output=True, text=True, timeout=12)
            self.assertEqual(same.returncode, 0, same.stdout + same.stderr)
            self.assertIn('DIRECT_ACCEPTED', same.stdout)
            print('EVIDENCE different_domain=not_discovered known_domain=direct_accepted', flush=True)
        finally:
            server.terminate()
            try: server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill(); server.wait()
            server.stdout.close()

    def test_cancel_all_hits_other_goals_and_mock_accepts_race(self):
        rclpy.init()
        server_node, one, two, outsider = (Node(n) for n in ('test_server', 'test_one', 'test_two', 'test_outsider'))
        group = ReentrantCallbackGroup()
        seen = []
        def execute(handle):
            seen.append(bytes(handle.goal_id.uuid).hex())
            if wait(lambda: handle.is_cancel_requested, 12): handle.canceled()
            else: handle.abort()
            return NavigateToPose.Result()
        server = ActionServer(server_node, NavigateToPose, '/phase14_cancel', execute,
                              cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        a = ActionClient(one, NavigateToPose, '/phase14_cancel')
        b = ActionClient(two, NavigateToPose, '/phase14_cancel')
        cancel_all = outsider.create_client(CancelGoal, '/phase14_cancel/_action/cancel_goal')
        executor = MultiThreadedExecutor(num_threads=6)
        for node in (server_node, one, two, outsider): executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
        try:
            self.assertTrue(a.wait_for_server(timeout_sec=5))
            self.assertTrue(b.wait_for_server(timeout_sec=5))
            self.assertTrue(cancel_all.wait_for_service(timeout_sec=5))
            fa = a.send_goal_async(NavigateToPose.Goal()); fb = b.send_goal_async(NavigateToPose.Goal())
            self.assertTrue(wait(lambda: fa.done() and fb.done()))
            ga, gb = fa.result(), fb.result()
            self.assertTrue(ga.accepted and gb.accepted)
            self.assertTrue(wait(lambda: len(seen) == 2))
            ra, rb = ga.get_result_async(), gb.get_result_async()
            future = cancel_all.call_async(CancelGoal.Request())
            self.assertTrue(wait(future.done))
            canceled = {bytes(info.goal_id.uuid).hex() for info in future.result().goals_canceling}
            self.assertEqual(canceled, {bytes(ga.goal_id.uuid).hex(), bytes(gb.goal_id.uuid).hex()})
            self.assertTrue(wait(lambda: ra.done() and rb.done()))
            self.assertEqual((ra.result().status, rb.result().status), (5, 5))
            print('EVIDENCE simultaneous_mock_goals=2 external_cancel_all=2 both_terminal=CANCELED', flush=True)
        finally:
            executor.shutdown(timeout_sec=5); thread.join(timeout=5)
            a.destroy(); b.destroy(); server.destroy()
            for node in (server_node, one, two, outsider): node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] in ('server', 'client'):
        sys.exit(worker(sys.argv[1]))
    unittest.main(verbosity=2)
