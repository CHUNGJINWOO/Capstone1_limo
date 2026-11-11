"""Python 탐색 노드와 모의 Nav2 사이의 실제 ROS 통신을 확인한다. 주행 시험은 아니다."""
import os
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from limo_auto_mapping.explorer import FrontierExplorer


@pytest.mark.skipif(os.environ.get('MAPPING_TRANSPORT_TEST') != '1',
                    reason='explicit isolated ROS domain required')
def test_python_explorer_sends_free_goal_and_reports_end():
    rclpy.init()
    mock = Node('mock_nav2_for_python_explorer')
    explorer = FrontierExplorer()
    broadcaster = TransformBroadcaster(mock)
    costmap = mock.create_publisher(OccupancyGrid, '/global_costmap/costmap', 1)
    sent, statuses = [], []
    completed = threading.Event()

    def navigate(handle):
        # 로봇을 움직이지 않고 Nav2 성공 응답과 지도의 변화를 흉내 낸다.
        sent.append(handle.request.pose)
        completed.set()
        handle.succeed()
        return NavigateToPose.Result()

    action = ActionServer(mock, NavigateToPose, '/navigate_to_pose', execute_callback=navigate)
    sub = mock.create_subscription(String, 'explore/status', lambda m: statuses.append(m.data), 10)

    def publish():
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = mock.get_clock().now().to_msg()
        grid.info.resolution = .1
        grid.info.width = 5
        grid.info.height = 3
        grid.info.origin.orientation.w = 1.0
        # 1행은 free, 2행은 처음에 unknown이다. 응답 후 경계가 사라진 상태를 발행한다.
        grid.data = [100]*5 + [100, 0, 0, 0, 100] + (
            [100]*5 if completed.is_set() else [100, -1, -1, -1, 100])
        costmap.publish(grid)
        transform = TransformStamped()
        transform.header = grid.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = .15
        transform.transform.translation.y = .15
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)

    timer = mock.create_timer(.1, publish)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(mock)
    executor.add_node(explorer)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        end = time.monotonic() + 12
        while 'exploration_complete' not in statuses and 'exploration_error' not in statuses:
            assert time.monotonic() < end, f'Timeout; statuses={statuses}'
            time.sleep(.05)
        assert 'exploration_error' not in statuses
        assert len(sent) == 1
        assert sent[0].header.frame_id == 'map'
        assert sent[0].pose.position.x == pytest.approx(.25)
        assert sent[0].pose.position.y == pytest.approx(.15)
    finally:
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        timer.cancel()
        mock.destroy_subscription(sub)
        action.destroy()
        explorer.destroy_node()
        mock.destroy_node()
        rclpy.shutdown()
