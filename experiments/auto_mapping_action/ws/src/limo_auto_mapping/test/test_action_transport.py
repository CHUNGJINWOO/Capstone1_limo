"""Real ROS action transport with a mock mapping backend, no robot/simulation.

Run explicitly with ROS_LOCALHOST_ONLY=1 and a separate ROS_DOMAIN_ID.
"""
import os
import threading
import time
from pathlib import Path

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from limo_auto_mapping.server import MappingServer
from limo_mapping_interfaces.action import AutoMapping
from test_policy import grid, world


@pytest.mark.skipif(os.environ.get('MAPPING_TRANSPORT_TEST') != '1',
                    reason='explicit isolated ROS domain required')
def test_action_success_duplicate_rejection_and_cancel(tmp_path):
    class MockMapping(MappingServer):
        def __init__(self):
            super().__init__()
            raw = grid([[100]*3, [100, 0, 100], [100]*3])
            self.samples['map'] = (raw, 0)
            self.position = world(raw, 4)
            self.entered = threading.Event()
            self.release = threading.Event()
            self.restore_calls = 0

        def ready(self):
            return True

        def explore(self, retries):
            self.entered.set()
            while not self.release.is_set():
                self.check()
                time.sleep(.02)
            return 'exploration_complete'

        def robot_pose(self, frame):
            return self.position

        def restore(self):
            self.restore_calls += 1

        def save(self, prefix):
            Path(prefix + '.mock_saved').write_text('mock backend only')

    def wait(future):
        end = time.monotonic() + 8
        while not future.done():
            assert time.monotonic() < end, 'ROS transport timeout'
            time.sleep(.02)
        return future.result()

    rclpy.init()
    server = MockMapping()
    client_node = Node('mapping_transport_test_client')
    client = ActionClient(client_node, AutoMapping, '/auto_mapping')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=8)
        request = AutoMapping.Goal()
        request.output_prefix = str(tmp_path / 'session1')
        request.max_duration_sec = 30.0
        handle = wait(client.send_goal_async(request))
        assert handle.accepted and server.entered.wait(5)
        duplicate = wait(client.send_goal_async(request))
        assert not duplicate.accepted
        cancel = wait(handle.cancel_goal_async())
        assert cancel.goals_canceling
        result = wait(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_CANCELED
        assert not result.result.success and server.restore_calls > 0
        assert not Path(request.output_prefix + '.mock_saved').exists()
        server.release.set()
        request.output_prefix = str(tmp_path / 'session2')
        handle = wait(client.send_goal_async(request))
        assert handle.accepted
        result = wait(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success
        assert Path(request.output_prefix + '.mock_saved').exists()
    finally:
        server.release.set()
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        client.destroy()
        server.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()
