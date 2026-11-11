"""Python 탐색 노드: 목표 선택은 Python, 실제 이동은 기존 Nav2가 담당한다."""
import math
import json
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .frontier import RobotInInflationError, choose_frontiers
from .selection import rank_candidates, checked_path


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('explore_node')
        defaults = {'robot_base_frame': 'base_link',
                    'costmap_topic': '/global_costmap/costmap',
                    'planner_frequency': 0.33, 'progress_timeout': 25.0,
                    'potential_scale': 3.0, 'gain_scale': 1.0,
                    'min_frontier_size': 0.20, 'blacklist_radius': 0.20,
                    'min_progress_distance': 0.05, 'transform_tolerance': 0.6,
                    'inflation_timeout': 10.0, 'cooldown_sec': 10.0,
                    'max_goal_retries': 3, 'minimum_goal_distance': 0.30,
                    'heading_weight': 2.0, 'failure_weight': 3.0,
                    'targets_per_frontier': 3, 'target_separation': 0.4,
                    'path_probe_timeout': 10.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.grid = None
        self.received_at = 0.0
        self.started_at = time.monotonic()
        self.have_pose = False
        # 목표별 실패/성공 이력을 분리하여 추적한다.
        self.goal_history = {}    # {(rx, ry): {'fails': int, 'last_event': float, 'cooldown': float, 'retry_limit_reached': bool}}
        self.token = 0            # 이전 취소 응답이 새 목표에 영향을 주지 않게 구분한다.
        self.target = None        # 전송 대기·주행·취소 중에는 다음 목표를 보내지 않는다.
        self.goal_start_robot = None  # 주행 시작 시점의 로봇 위치 (변위 측정용)
        self.total_dispatched_goals = 0
        self.empty_candidate_count = 0
        self.inflation_started = 0.0
        self.nav_goal = None
        self.canceling = False
        self.cancel_started = 0.0
        self.finished = False
        self.best_distance = float('inf')
        self.last_progress = 0.0
        self.robot_yaw = 0.0
        self.probe_pending = False
        self.probe_handle = None
        self.probe_token = 0
        self.probe_started = 0.0
        self.servers_wait_started = time.monotonic()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        transient = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 회차마다 상태 토픽을 바꿔 이전 회차의 완료 알림과 섞이지 않게 한다.
        self.status = self.create_publisher(String, 'explore/status', transient)
        self.event_publisher = self.create_publisher(String, 'explore/events', 100)
        self.subscription = self.create_subscription(
            OccupancyGrid, self.param('costmap_topic'), self.on_grid, 1)
        frequency = self.param('planner_frequency')
        if not math.isfinite(frequency) or frequency <= 0:
            raise ValueError('planner_frequency must be positive')
        for name in ('minimum_goal_distance', 'path_probe_timeout', 'heading_weight', 'failure_weight'):
            value = self.param(name)
            if not math.isfinite(value) or value < 0 or (name in (
                    'minimum_goal_distance', 'path_probe_timeout') and value == 0):
                raise ValueError(name + ' must be finite and within its positive/nonnegative range')
        self.next_search = 0.0
        # TF·진행·취소는 0.2초마다, 경계 탐색은 planner_frequency 주기로 처리한다.
        self.timer = self.create_timer(0.2, self.tick)

    def param(self, name):
        return self.get_parameter(name).value

    def on_grid(self, message):
        self.grid = message
        self.received_at = time.monotonic()

    def event(self, kind, **fields):
        """목표 선택과 Nav2 결과를 연결하는 관측 기록. 성공 판정을 대신하지 않는다."""
        payload = json.dumps({'event': kind, 'goal_id': self.token,
                              'elapsed_sec': time.monotonic()-self.started_at, **fields},
                             ensure_ascii=False, allow_nan=False)
        self.get_logger().info('EXPLORE_EVENT ' + payload)
        if getattr(self, 'event_publisher', None) is not None:
            self.event_publisher.publish(String(data=payload))

    def finish(self, status, reason='', detail=''):
        if self.finished:
            return
        self.event('exploration_finished', status=status,
                   reason=reason or status, detail=detail,
                   dispatched_goals=self.total_dispatched_goals)
        self.finished = True
        self.status.publish(String(data=json.dumps(
            {'status': status, 'reason': reason or status, 'detail': detail}, ensure_ascii=False)))
        if self.probe_handle is not None:
            self.probe_handle.cancel_goal_async()
        self.timer.cancel()

    def pose(self):
        transform = self.tf.lookup_transform(
            self.grid.header.frame_id, self.param('robot_base_frame'), rclpy.time.Time())
        stamp = transform.header.stamp
        age = self.get_clock().now().nanoseconds/1e9 - stamp.sec - stamp.nanosec/1e9
        if age < -0.5 or age > self.param('transform_tolerance'):
            raise ValueError('Robot TF is stale')
        q = transform.transform.rotation
        self.robot_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return transform.transform.translation.x, transform.transform.translation.y

    def is_goal_excluded(self, point, radius):
        """목표가 이번 회차 재시도 한도에 도달했거나 쿨다운(cooldown) 기간 내에 있는지 판정한다."""
        now = time.monotonic()
        for p, info in self.goal_history.items():
            if math.hypot(point[0]-p[0], point[1]-p[1]) <= radius:
                if info.get('retry_limit_reached', False) or info.get('completed', False):
                    return True
                cooldown = info.get('cooldown', self.param('cooldown_sec'))
                if now - info.get('last_event', 0.0) < cooldown:
                    return True
        return False

    def exclude_goal(self, point, is_failure=True, cooldown=None):
        """목표를 임시 쿨다운 또는 재시도 한도 초과로 제외한다.
        
        - is_failure=True: Nav2 주행 실패(ABORTED) 시 호출. 실패 횟수를 증가시키며,
          max_goal_retries(기본 3회) 도달 시에만 이번 회차 제외(retry_limit_reached)로 전환.
        - is_failure=False: 명시적인 임시 제외. 제자리 성공은 실패 횟수에 포함한다.
        """
        # 지도 갱신으로 목표가 조금 이동해도 주변 실패 이력을 이어받는다.
        nearby = [p for p in self.goal_history if math.hypot(point[0]-p[0], point[1]-p[1])
                  <= self.param('blacklist_radius')]
        key = (min(nearby, key=lambda p: math.hypot(point[0]-p[0], point[1]-p[1])) if nearby
               else (round(point[0], 2), round(point[1], 2)))
        effective_cooldown = cooldown if cooldown is not None else self.param('cooldown_sec')
        info = self.goal_history.setdefault(
            key, {'fails': 0, 'last_event': 0.0, 'cooldown': effective_cooldown, 'retry_limit_reached': False}
        )
        info['last_event'] = time.monotonic()
        info['cooldown'] = effective_cooldown

        if is_failure:
            info['fails'] += 1
            if info['fails'] >= self.param('max_goal_retries'):
                info['retry_limit_reached'] = True
                self.get_logger().warn(
                    f'목표 {point} 주행 실패 한도({info["fails"]}회) 도달: 이번 탐색 회차에서 제외'
                )
            else:
                self.get_logger().info(
                    f'목표 {point} 주행 실패({info["fails"]}회): {effective_cooldown:.1f}초 쿨다운 후 재평가 예정'
                )
        else:
            self.get_logger().info(
                f'목표 {point} 제자리 성공: {effective_cooldown:.1f}초 쿨다운 제외 (실패 횟수 미증가)'
            )
        self.event('goal_excluded', target=list(point), is_failure=is_failure,
                   fails=info['fails'], cooldown_sec=effective_cooldown,
                   retry_limit_reached=info['retry_limit_reached'])

    def tick(self):
        if self.finished or self.grid is None:
            return
        now = time.monotonic()
        if now - self.received_at > 5.0:
            self.get_logger().error('Global costmap stopped updating')
            self.finish('exploration_error', 'COSTMAP_STALE', 'Global costmap stopped updating')
            return
        try:
            try:
                robot = self.pose()
            except Exception:
                if not self.have_pose and now - self.started_at < 5.0:
                    return  # 새 프로세스의 TF 버퍼가 처음 채워지는 시간을 허용한다.
                raise
            self.have_pose = True

            if self.probe_pending:
                if now-self.probe_started > self.param('path_probe_timeout'):
                    self.finish('exploration_error', 'PATH_PROBE_TIMEOUT',
                                'ComputePathToPose response/result timeout')
                return

            # 주행 중 또는 취소 진행 중인 경우 상태 감시
            if self.target is not None:
                # 실제 TF로 목표까지 거리를 측정하고 일정 이상 가까워져야 진행으로 인정한다.
                distance = math.hypot(robot[0]-self.target[0], robot[1]-self.target[1])
                if self.best_distance - distance >= self.param('min_progress_distance'):
                    self.best_distance = distance
                    self.last_progress = now
                if self.canceling:
                    if now - self.cancel_started > 10.0:
                        self.get_logger().error('Nav2 cancel/result timeout')
                        self.finish('exploration_error', 'NAV2_CANCEL_TIMEOUT', 'Nav2 cancel/result timeout')
                elif now - self.last_progress > self.param('progress_timeout'):
                    if self.nav_goal is None:
                        self.get_logger().error('Nav2 did not acknowledge the goal')
                        self.finish('exploration_error', 'NAV2_GOAL_TIMEOUT', 'Nav2 did not acknowledge the goal')
                    else:
                        # 취소가 끝나기 전에는 다음 목표를 보내지 않아 목표 충돌을 방지한다.
                        self.canceling = True
                        self.cancel_started = now
                        self.event('cancel_requested', reason='progress_timeout',
                                   elapsed_without_progress_sec=now-self.last_progress)
                        self.nav_goal.cancel_goal_async().add_done_callback(
                            lambda future, token=self.token: self.on_cancel(future, token))
                return

            if not self.nav.server_is_ready() or not self.planner.server_is_ready():
                if now-self.servers_wait_started > self.param('path_probe_timeout'):
                    self.finish('exploration_error', 'NAV2_UNAVAILABLE',
                                'NavigateToPose or ComputePathToPose server unavailable')
                return
            self.servers_wait_started = now
            if now < self.next_search:
                return
            self.next_search = now + 1.0/self.param('planner_frequency')

            try:
                candidates = choose_frontiers(
                    self.grid, robot, self.param('min_frontier_size'),
                    self.param('potential_scale'), self.param('gain_scale'),
                    allow_fallback=True, target_count=self.param('targets_per_frontier'),
                    target_separation=self.param('target_separation'))
                # 정상적으로 경계를 계산했으므로 인플레이션 대기 상태를 초기화한다.
                self.inflation_started = 0.0
            except RobotInInflationError as exc:
                # 로봇이 시작 위치 주변 인플레이션 영역에 있는 경우 설정된 시간 동안 대기 및 재평가
                if self.inflation_started == 0.0:
                    self.inflation_started = now
                elapsed = now - self.inflation_started
                timeout = self.param('inflation_timeout')
                if elapsed < timeout:
                    self.get_logger().warn(
                        f'로봇 주변 인플레이션 감지: {exc} | 재평가 대기 중 ({elapsed:.1f}s / {timeout:.1f}s)'
                    )
                    self.next_search = now + 1.0
                    return
                else:
                    self.get_logger().error(f'인플레이션 영역 대기 시간 초과 ({elapsed:.1f}s): {exc}')
                    self.finish('exploration_error', 'START_IN_INSCRIBED', str(exc))
                    return

            if not candidates:
                # 시작 직후 costmap 초기화 단계에서 경계 조각이 아직 형성되지 않은 경우 즉시 완료 방지
                if self.total_dispatched_goals == 0 and (now - self.started_at < 5.0):
                    self.get_logger().info('시작 초기 costmap 갱신 대기 중...')
                    return
                self.empty_candidate_count += 1
                self.event('no_frontiers', consecutive=self.empty_candidate_count,
                           grid_stamp_sec=self.grid.header.stamp.sec,
                           grid_stamp_nanosec=self.grid.header.stamp.nanosec)
                if self.empty_candidate_count < 3:
                    # 3회 연속 빈 후보가 확인될 때까지 조기 종료를 유보한다.
                    self.next_search = now + 1.0
                    return
                self.finish('exploration_complete')
                return

            self.empty_candidate_count = 0
            available = [f for f in candidates if not self.is_goal_excluded(
                f.point, self.param('blacklist_radius'))]
            self.event('candidate_summary', candidates=len(candidates), available=len(available),
                       excluded=len(candidates)-len(available))
            if not available:
                # 모든 후보가 재시도 한도에 도달했는지, 아니면 일시 쿨다운 대기 중인지 확인한다.
                has_cooldown_candidates = any(
                    any(math.hypot(f.point[0]-p[0], f.point[1]-p[1]) <= self.param('blacklist_radius')
                        and not info.get('retry_limit_reached', False)
                        and not info.get('completed', False)
                        for p, info in self.goal_history.items())
                    for f in candidates
                )
                if has_cooldown_candidates:
                    self.get_logger().info('일시 제외된 후보의 쿨다운 대기 중... 재평가 대기')
                    self.next_search = now + 2.0
                    return
                self.finish('frontiers_exhausted')
                return

            ranked = rank_candidates(
                available, robot, self.robot_yaw, self.goal_history, self.param('blacklist_radius'),
                self.param('minimum_goal_distance'), self.param('heading_weight'), self.param('failure_weight'))
            self.event('candidate_ranking', robot_yaw=self.robot_yaw,
                       candidates=[{'point': list(f.point), **info} for f, info in ranked],
                       too_close=len(available)-len(ranked))
            if not ranked:
                self.finish('frontiers_exhausted', 'GOALS_TOO_CLOSE',
                            'Remaining frontier targets are within minimum_goal_distance')
                return
            # 경로 검사가 실패하면 해당 목표를 제외하고 다음 tick에서 다른 후보를 평가한다.
            self.probe(ranked[0][0].point, robot)
        except Exception as exc:
            # TF 오류를 탐색 완료로 처리하지 않는다. 상위 Action이 정지·마진 복원을 맡는다.
            self.get_logger().error(str(exc))
            self.finish('exploration_error', 'EXPLORER_EXCEPTION', f'{type(exc).__name__}: {exc}')

    def probe(self, point, robot):
        self.probe_pending = True
        self.probe_started = time.monotonic()
        self.probe_token += 1
        token = self.probe_token
        request = ComputePathToPose.Goal()
        request.goal.header.frame_id = self.grid.header.frame_id
        request.goal.header.stamp = self.get_clock().now().to_msg()
        request.goal.pose.position.x, request.goal.pose.position.y = point
        request.goal.pose.orientation.w = 1.0
        request.planner_id = 'GridBased'
        request.use_start = False
        self.event('path_probe_started', target=list(point), probe_id=token)
        self.planner.send_goal_async(request).add_done_callback(
            lambda future: self.on_plan_goal(future, token, point, robot))

    def on_plan_goal(self, future, token, point, robot):
        try:
            handle = future.result()
            if self.finished or token != self.probe_token:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.plan_rejected(point, 'planner_rejected_goal')
                return
            self.probe_handle = handle
            handle.get_result_async().add_done_callback(
                lambda response: self.on_plan_result(response, token, point, robot))
        except Exception as exc:
            self.finish('exploration_error', 'PATH_PROBE_ERROR', str(exc))

    def plan_rejected(self, point, reason):
        self.event('path_probe_rejected', target=list(point), reason=reason)
        self.exclude_goal(point, is_failure=True)
        self.probe_pending = False
        self.probe_handle = None
        self.next_search = time.monotonic()  # 불가능한 후보 때문에 쿨다운 전체를 기다리지 않는다.

    def on_plan_result(self, future, token, point, robot):
        if self.finished or token != self.probe_token:
            return
        try:
            response = future.result()
            valid, yaw, reason = checked_path(response.result.path.poses, point, self.grid)
            if response.result.path.header.frame_id != self.grid.header.frame_id:
                valid, reason = False, 'path_frame_mismatch'
            if response.status != GoalStatus.STATUS_SUCCEEDED or not valid:
                self.plan_rejected(point, reason if not valid else 'planner_failed')
                return
            current = self.pose()  # 경로 계산을 기다리는 동안 위치가 바뀌었을 수 있다.
            self.event('path_probe_succeeded', target=list(point), arrival_yaw=yaw,
                       path_points=len(response.result.path.poses))
            self.probe_pending = False
            self.probe_handle = None
            self.send(point, current, yaw)
        except Exception as exc:
            self.finish('exploration_error', 'PATH_PROBE_ERROR', f'{type(exc).__name__}: {exc}')

    def send(self, point, robot, arrival_yaw=0.0):
        self.token += 1
        self.target = point
        self.goal_start_robot = robot
        self.total_dispatched_goals += 1
        self.best_distance = math.hypot(point[0]-robot[0], point[1]-robot[1])
        self.event('goal_sent', target=list(point), robot=list(robot),
                   initial_distance_m=self.best_distance, arrival_yaw=arrival_yaw)
        self.last_progress = time.monotonic()
        self.canceling = False
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = self.grid.header.frame_id
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = point
        # 모든 목표를 지도 동쪽으로 향하게 하지 않고 검사한 경로 끝 방향에 맞춘다.
        request.pose.pose.orientation.z = math.sin(arrival_yaw/2)
        request.pose.pose.orientation.w = math.cos(arrival_yaw/2)
        self.status.publish(String(data='exploration_in_progress'))
        self.nav.send_goal_async(request).add_done_callback(self.on_goal)

    def on_goal(self, future):
        try:
            handle = future.result()
            self.event('goal_response', accepted=handle.accepted)
            if not handle.accepted:
                self.exclude_goal(self.target, is_failure=True)
                self.target = None
                return
            self.nav_goal = handle
            handle.get_result_async().add_done_callback(self.on_result)
            if self.finished:
                handle.cancel_goal_async()  # 늦게 도착한 목표 수락도 취소해 남은 주행을 방지한다.
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.finish('exploration_error', 'NAV2_GOAL_ERROR', f'{type(exc).__name__}: {exc}')

    def on_cancel(self, future, token):
        if token != self.token:
            return  # 이전 목표의 응답은 새 목표의 상태를 바꾸지 않는다.
        try:
            # 취소 요청 수락 여부를 확인한다. 실제 종료 상태는 on_result가 처리한다.
            cancel_response = future.result()
            self.event('cancel_response', canceling_goals=len(cancel_response.goals_canceling))
            if not cancel_response.goals_canceling:
                self.get_logger().warn('Nav2 목표 취소 요청이 거부되었거나 이미 완료 단계임')
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.finish('exploration_error', 'NAV2_CANCEL_ERROR', f'{type(exc).__name__}: {exc}')

    def on_result(self, future):
        try:
            result = future.result()
            if self.finished:
                return
            if result.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                                     GoalStatus.STATUS_CANCELED):
                raise RuntimeError('Unexpected NavigateToPose terminal status')

            # 실제 로봇의 위치 변위를 측정하여 제자리 성공을 감지한다.
            current_robot = None
            pose_error = None
            try:
                current_robot = self.pose()
            except Exception as exc:
                pose_error = f'{type(exc).__name__}: {exc}'

            # TF 조회 실패는 변위 0이 아니라 미측정(None)이다.
            displacement = None
            if current_robot is not None and self.goal_start_robot is not None:
                displacement = math.hypot(current_robot[0]-self.goal_start_robot[0],
                                          current_robot[1]-self.goal_start_robot[1])

            self.event('goal_result', nav2_status=result.status, target=list(self.target),
                       displacement_m=displacement, pose_error=pose_error,
                       final_distance_m=(math.hypot(current_robot[0]-self.target[0],
                                                   current_robot[1]-self.target[1])
                                         if current_robot is not None else None))

            if result.status == GoalStatus.STATUS_SUCCEEDED:
                if displacement is None:
                    raise RuntimeError('Nav2 succeeded but progress is unmeasured: ' +
                                       (pose_error or 'goal start pose missing'))
                if displacement < self.param('min_progress_distance'):
                    self.get_logger().warn(
                        f'목표 도착 보고되었으나 변위({displacement:.3f}m)가 임계값 미만: 실패 횟수에 포함'
                    )
                    self.exclude_goal(self.target, is_failure=True, cooldown=15.0)
                else:
                    self.get_logger().info(f'목표 도달 성공 (시작점 대비 변위: {displacement:.3f}m)')
                    key = (round(self.target[0], 2), round(self.target[1], 2))
                    info = self.goal_history.setdefault(key, {'fails': 0, 'last_event': 0.0, 'cooldown': 10.0, 'retry_limit_reached': False})
                    # 도착 후 재방문 제외와 실패 한도 도달은 다른 사유로 기록한다.
                    info['completed'] = True
                    self.event('goal_completed', target=list(self.target),
                               displacement_m=displacement)
            elif result.status == GoalStatus.STATUS_ABORTED:
                # 일시적 장애물로 인한 abort일 수 있으므로 재시도 정책을 적용한다.
                self.get_logger().warn(f'Nav2 주행 중단(ABORTED): 목표 {self.target}')
                self.exclude_goal(self.target, is_failure=True)
            elif result.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info(f'Nav2 주행 취소 완료(CANCELED): 목표 {self.target}')
                self.exclude_goal(self.target, is_failure=True)

            self.next_search = time.monotonic() + 2.0  # SLAM/costmap의 후속 갱신을 잠시 기다린다.
            self.target = None
            self.nav_goal = None
            self.canceling = False
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.finish('exploration_error', 'NAV2_RESULT_ERROR', f'{type(exc).__name__}: {exc}')


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

