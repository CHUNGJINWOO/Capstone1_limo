"""Python 탐색 노드: 목표 선택은 Python, 실제 이동은 기존 Nav2가 담당한다."""
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .frontier import blacklisted, choose_frontiers


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('explore_node')
        defaults = {'robot_base_frame': 'base_link',
                    'costmap_topic': '/global_costmap/costmap',
                    'planner_frequency': 0.33, 'progress_timeout': 25.0,
                    'potential_scale': 3.0, 'gain_scale': 1.0,
                    'min_frontier_size': 0.20, 'blacklist_radius': 0.20,
                    'min_progress_distance': 0.05, 'transform_tolerance': 0.6}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.grid = None
        self.received_at = 0.0
        self.started_at = time.monotonic()
        self.have_pose = False
        self.excluded = []        # 이미 시도한 위치는 이번 회차에서 제외해 같은 목표 반복을 막는다.
        self.token = 0            # 이전 취소 응답이 새 목표에 영향을 주지 않게 구분한다.
        self.target = None        # 전송 대기·주행·취소 중에는 다음 목표를 보내지 않는다.
        self.nav_goal = None
        self.canceling = False
        self.cancel_started = 0.0
        self.finished = False
        self.best_distance = float('inf')
        self.last_progress = 0.0
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        transient = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 회차마다 상태 토픽을 바꿔 이전 회차의 완료 알림과 섞이지 않게 한다.
        self.status = self.create_publisher(String, 'explore/status', transient)
        self.subscription = self.create_subscription(
            OccupancyGrid, self.param('costmap_topic'), self.on_grid, 1)
        frequency = self.param('planner_frequency')
        if not math.isfinite(frequency) or frequency <= 0:
            raise ValueError('planner_frequency must be positive')
        self.next_search = 0.0
        # TF·진행·취소는 0.2초마다, 경계 탐색은 planner_frequency 주기로 처리한다.
        self.timer = self.create_timer(0.2, self.tick)

    def param(self, name):
        return self.get_parameter(name).value

    def on_grid(self, message):
        self.grid = message
        self.received_at = time.monotonic()

    def finish(self, status):
        self.finished = True
        self.status.publish(String(data=status))
        self.timer.cancel()

    def pose(self):
        transform = self.tf.lookup_transform(
            self.grid.header.frame_id, self.param('robot_base_frame'), rclpy.time.Time())
        stamp = transform.header.stamp
        age = self.get_clock().now().nanoseconds/1e9 - stamp.sec - stamp.nanosec/1e9
        if age < -0.5 or age > self.param('transform_tolerance'):
            raise ValueError('Robot TF is stale')
        return transform.transform.translation.x, transform.transform.translation.y

    def tick(self):
        if self.finished or self.grid is None:
            return
        now = time.monotonic()
        if now - self.received_at > 5.0:
            self.get_logger().error('Global costmap stopped updating')
            self.finish('exploration_error')
            return
        try:
            try:
                robot = self.pose()
            except Exception:
                if not self.have_pose and now - self.started_at < 5.0:
                    return  # 새 프로세스의 TF 버퍼가 처음 채워지는 시간을 허용한다.
                raise
            self.have_pose = True
            if self.target is not None:
                # 실제 TF로 목표까지 거리를 측정하고 일정 이상 가까워져야 진행으로 인정한다.
                distance = math.hypot(robot[0]-self.target[0], robot[1]-self.target[1])
                if self.best_distance - distance >= self.param('min_progress_distance'):
                    self.best_distance = distance
                    self.last_progress = now
                if self.canceling:
                    if now - self.cancel_started > 10.0:
                        self.get_logger().error('Nav2 cancel/result timeout')
                        self.finish('exploration_error')
                elif now - self.last_progress > self.param('progress_timeout'):
                    if self.nav_goal is None:
                        self.get_logger().error('Nav2 did not acknowledge the goal')
                        self.finish('exploration_error')
                    else:
                        # 취소가 끝나기 전에는 다음 목표를 보내지 않아 목표 충돌을 방지한다.
                        self.canceling = True
                        self.cancel_started = now
                        self.nav_goal.cancel_goal_async().add_done_callback(
                            lambda future, token=self.token: self.on_cancel(future, token))
                return
            if now < self.next_search or not self.nav.server_is_ready():
                return
            self.next_search = now + 1.0/self.param('planner_frequency')
            candidates = choose_frontiers(
                self.grid, robot, self.param('min_frontier_size'),
                self.param('potential_scale'), self.param('gain_scale'))
            if not candidates:
                self.finish('exploration_complete')
                return
            available = [f for f in candidates if not blacklisted(
                f.point, self.excluded, self.param('blacklist_radius'))]
            if not available:
                self.finish('frontiers_exhausted')
                return
            self.send(available[0].point, robot)
        except Exception as exc:
            # TF 오류를 탐색 완료로 처리하지 않는다. 상위 Action이 정지·마진 복원을 맡는다.
            self.get_logger().error(str(exc))
            self.finish('exploration_error')

    def send(self, point, robot):
        self.token += 1
        self.target = point
        self.best_distance = math.hypot(point[0]-robot[0], point[1]-robot[1])
        self.last_progress = time.monotonic()
        self.canceling = False
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = self.grid.header.frame_id
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = point
        request.pose.pose.orientation.w = 1.0  # 위치 도달과 자세 판정은 기존 Nav2 설정을 사용한다.
        self.status.publish(String(data='exploration_in_progress'))
        self.nav.send_goal_async(request).add_done_callback(self.on_goal)

    def on_goal(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.excluded.append(self.target)
                self.target = None
                return
            self.nav_goal = handle
            handle.get_result_async().add_done_callback(self.on_result)
            if self.finished:
                handle.cancel_goal_async()  # 늦게 도착한 목표 수락도 취소해 남은 주행을 방지한다.
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.finish('exploration_error')

    def on_cancel(self, future, token):
        if token != self.token:
            return  # 이전 목표의 응답은 새 목표의 상태를 바꾸지 않는다.
        try:
            if not future.result().goals_canceling:
                # 결과와 취소 응답의 순서는 바뀔 수 있다. 결과를 받았다면 이미 종료된 것이다.
                if self.target is not None:
                    self.finish('exploration_error')
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.finish('exploration_error')

    def on_result(self, future):
        try:
            result = future.result()
            if self.finished:
                return
            if result.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                                     GoalStatus.STATUS_CANCELED):
                raise RuntimeError('Unexpected NavigateToPose terminal status')
            self.excluded.append(self.target)
            # 성공 지점도 회차 안에서 재방문하지 않는다. 남은 경계는 상위가 부분 완료로 보고한다.
            self.next_search = time.monotonic() + 2.0  # SLAM/costmap의 후속 갱신을 잠시 기다린다.
            self.target = None
            self.nav_goal = None
            self.canceling = False
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.finish('exploration_error')


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 상위 Action은 이 프로세스 종료 후 Nav2 취소를 확인한 다음 마진을 복원한다.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
