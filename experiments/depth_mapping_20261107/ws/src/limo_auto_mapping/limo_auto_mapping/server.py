"""자동 맵핑 Action: 탐색 → 조건부 마진 재시도 → 복원 → 지도 저장을 관리한다.

실제 경로 추종은 기존 Nav2가 맡고, 이 파일은 속도 명령을 직접 발행하지 않는다.
"""
import math
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
import traceback

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from limo_mapping_interfaces.action import AutoMapping
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from tf2_ros import Buffer, TransformListener

from .policy import cost, frontiers, suppressed_candidates, validate_radii
from .journal import SessionJournal
from .depth_gate import depth_issue


class SessionCancelled(Exception):
    pass


class MappingFailure(RuntimeError):
    def __init__(self, reason, detail):
        super().__init__(detail)
        self.reason = reason


class MappingServer(Node):
    def __init__(self):
        super().__init__('auto_mapping_server')
        defaults = {'reduced_inflation_radius': 0.24, 'minimum_clearance': 0.05,
                    'base_frame': 'base_link', 'data_timeout_sec': 5.0,
                    'service_timeout_sec': 10.0, 'max_path_probes': 12,
                    'require_localization_ready': False,
                    'require_depth_ready': False, 'depth_timeout_sec': 1.5}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.require_localization_ready = self.get_parameter('require_localization_ready').value
        self.require_depth_ready = self.get_parameter('require_depth_ready').value
        self.depth_status = None
        self.depth_received_at = float('-inf')
        # Action 실행을 기다리는 중에도 센서·서비스 응답을 처리하기 위한 콜백 그룹이다.
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.busy = False
        self.stop_requested = False
        self.healthy = True
        self.samples = {}
        self.last_health_issue = 'NOT_CHECKED: 상태 검사 전'
        self.health_log_code = None
        self.health_log_at = float('-inf')
        self.nav_status = []
        self.process = None
        self.journal = None
        self.recording_error = ''
        self.last_phase = None
        self.explorer_detail = {}
        self.goal = None
        self.deadline = float('inf')
        self.localization_ready_at = float('-inf')
        # 마진 변경 전 실제 값을 저장해 종료·취소·실패 시 같은 값으로 되돌린다.
        self.snapshots = {}
        self.changed = set()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/auto_mapping/localization_ready',
                                 self.on_localization_ready, transient, callback_group=self.group)
        self.create_subscription(OccupancyGrid, '/map',
                                 lambda m: self.sample('map', m), transient,
                                 callback_group=self.group)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: self.sample('costmap', m), 1,
                                 callback_group=self.group)
        self.create_subscription(LaserScan, '/scan',
                                 lambda m: self.sample('scan', m), qos_profile_sensor_data,
                                 callback_group=self.group)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self.on_status, transient, callback_group=self.group)
        self.cancel_nav = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal',
                                              callback_group=self.group)
        self.serialize = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map',
                                            callback_group=self.group)
        self.save_map = self.create_client(SaveMap, '/slam_toolbox/save_map',
                                           callback_group=self.group)
        self.params = {}
        for node in ['/global_costmap/global_costmap', '/local_costmap/local_costmap']:
            self.params[node] = (
                self.create_client(GetParameters, node + '/get_parameters',
                                   callback_group=self.group),
                self.create_client(SetParametersAtomically, node + '/set_parameters_atomically',
                                   callback_group=self.group))
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose',
                                    callback_group=self.group)
        self.server = ActionServer(self, AutoMapping, '/auto_mapping',
                                   execute_callback=self.execute,
                                   goal_callback=self.accept,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.group)
        self.create_subscription(Log, '/rosout', self.on_rosout, 100, callback_group=self.group)
        self.create_subscription(String, '/auto_mapping/depth_status', self.on_depth_status,
                                 10, callback_group=self.group)

    def on_depth_status(self, message):
        try:
            def invalid(value):
                raise ValueError('Non-finite JSON constant: '+value)
            status = json.loads(message.data, parse_constant=invalid)
            if not isinstance(status, dict):
                raise ValueError('Depth status must be an object')
        except (ValueError, TypeError) as exc:
            status = {'state': 'error', 'code': 'DEPTH_INVALID_STATUS', 'message': str(exc)}
        previous = getattr(self, 'depth_status', None)
        self.depth_status, self.depth_received_at = status, time.monotonic()
        if previous is None or (previous.get('state'), previous.get('code')) != (
                status.get('state'), status.get('code')):
            self.record('depth_status_changed', payload=status)

    def record(self, kind, **fields):
        journal = getattr(self, 'journal', None)
        if journal is not None:
            try:
                journal.event(kind, **fields)
            except Exception as exc:
                if not self.recording_error:
                    self.recording_error = f'{type(exc).__name__}: {exc}'
                    self.get_logger().error('SESSION_LOG_FAILED: ' + self.recording_error)

    def on_rosout(self, msg):
        # 전용 맵핑 stack의 경고·오류만 수집한다. rosout 콜백에서 다시 로그를 출력하지 않는다.
        names = ('auto_mapping_server', 'explore_node', 'controller_server', 'planner_server',
                 'bt_navigator', 'behavior_server', 'slam_toolbox', 'async_slam_toolbox_node',
                 'velocity_smoother', 'smoother_server', 'lifecycle_manager_navigation',
                 'local_costmap', 'global_costmap')
        names += ('depth_obstacle_filter', 'camera')
        # Humble의 byte 상수는 Python에서 bytes일 수 있지만 level 필드는 int다.
        warning_level = Log.WARN[0] if isinstance(Log.WARN, bytes) else Log.WARN
        if msg.level >= warning_level and any(msg.name == n or msg.name.startswith(n+'.') for n in names):
            self.record('ros_warning_or_error', node=msg.name, level=msg.level, message=msg.msg,
                        ros_stamp={'sec': msg.stamp.sec, 'nanosec': msg.stamp.nanosec})

    def param(self, name):
        return self.get_parameter(name).value

    def on_localization_ready(self, msg):
        # 자동 재위치 추정 프로세스가 살아 있고 전환 검사를 통과했을 때만 수락한다.
        self.localization_ready_at = time.monotonic() if msg.data else float('-inf')

    def localization_allowed(self):
        return (not getattr(self, 'require_localization_ready', False)
                or time.monotonic() - self.localization_ready_at < 5.0)

    def sample(self, key, msg):
        self.samples[key] = (msg, time.monotonic())

    def on_status(self, msg):
        self.nav_status = [s.status for s in msg.status_list]

    def nav_busy(self):
        return any(s in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                         GoalStatus.STATUS_CANCELING) for s in self.nav_status)

    # 다른 탐색과 충돌하거나 기존 지도 파일을 덮어쓰는 요청은 수락하지 않는다.
    def accept(self, request):
        if not MappingServer.localization_allowed(self):
            self.get_logger().warning('Goal rejected: automatic localization/handoff not ready')
            return GoalResponse.REJECT
        with self.lock:
            prefix = Path(request.output_prefix)
            valid = (request.output_prefix and prefix.is_absolute()
                     and prefix.name not in ('', '.', '..')
                     and math.isfinite(request.max_duration_sec)
                     and (request.max_duration_sec == 0
                          or 10 <= request.max_duration_sec <= 86400))
            if self.busy or self.stop_requested or not self.healthy or not valid or self.nav_busy():
                return GoalResponse.REJECT
            if any(Path(str(prefix) + ext).exists()
                   for ext in ('.posegraph', '.data', '.yaml', '.pgm', '.png',
                               '.events.jsonl', '.session.json', '.explorer.log')):
                return GoalResponse.REJECT
            self.busy = True
            return GoalResponse.ACCEPT

    def check(self):
        if self.stop_requested or not rclpy.ok():
            raise MappingFailure('NODE_SHUTDOWN', 'Mapping node shutdown requested')
        if self.goal and self.goal.is_cancel_requested:
            raise SessionCancelled('User requested Action cancellation')
        if self.recording_error:
            raise MappingFailure('LOG_WRITE_FAILED', self.recording_error)
        if time.monotonic() >= self.deadline:
            raise MappingFailure('SESSION_TIMEOUT', 'Session deadline reached')
        if not MappingServer.localization_allowed(self):
            raise MappingFailure('LOCALIZATION_LOST', 'Automatic localization supervisor heartbeat lost')

    def until(self, predicate, timeout, interrupt=True):
        if interrupt:
            self.check()
        end = time.monotonic() + timeout
        while not predicate():
            if interrupt:
                self.check()
            if time.monotonic() >= end:
                # 준비 대기 실패는 서비스 응답 지연과 구분해 Action 결과에도 남긴다.
                if (getattr(predicate, '__self__', None) is self
                        and getattr(predicate, '__name__', '') == 'ready'):
                    raise MappingFailure('INPUT_TIMEOUT', 'Input readiness timeout: ' + self.last_health_issue)
                raise TimeoutError('Timed out waiting for ROS operation')
            time.sleep(0.05)

    def future(self, future, interrupt=True):
        self.until(future.done, self.param('service_timeout_sec'), interrupt)
        if interrupt:
            self.check()
        return future.result()

    def call(self, client, request, interrupt=True):
        self.until(client.service_is_ready, self.param('service_timeout_sec'), interrupt)
        return self.future(client.call_async(request), interrupt)

    def feedback(self, phase, retries):
        if phase != self.last_phase:
            self.record('phase_changed', phase=phase, retries=retries)
            self.last_phase = phase
        msg = AutoMapping.Feedback()
        msg.phase = phase
        msg.elapsed_sec = time.monotonic() - self.started
        msg.retry_count = retries
        self.goal.publish_feedback(msg)

    def health_failure(self, code, detail):
        """검사 기준은 유지하고, 실패 항목과 수치를 보존한다."""
        self.last_health_issue = f'{code}: {detail}'
        now = time.monotonic()
        # 같은 원인의 대기 로그는 2초에 한 번만 출력한다. 최종 원인은 매번 갱신한다.
        if code != self.health_log_code or now - self.health_log_at >= 2.0:
            MappingServer.record(self, 'input_health_issue', code=code, detail=detail)
            self.get_logger().warning(self.last_health_issue)
            self.health_log_code = code
            self.health_log_at = now
        return False

    # 수신 여부뿐 아니라 센서 시각과 TF도 검사해 오래된 데이터로 움직이지 않게 한다.
    def ready(self):
        now = time.monotonic()
        if getattr(self, 'require_depth_ready', False):
            issue = depth_issue(self.depth_status, self.depth_received_at, now,
                                self.get_clock().now().nanoseconds/1e9,
                                self.param('depth_timeout_sec'))
            if issue is not None:
                return self.health_failure(*issue)
        limit = self.param('data_timeout_sec')
        topics = {'map': '/map', 'costmap': '/global_costmap/costmap', 'scan': '/scan'}
        for key, topic in topics.items():
            if key not in self.samples:
                return self.health_failure('INPUT_MISSING', f'topic={topic}')
            receipt_age = now - self.samples[key][1]
            if receipt_age > limit:
                return self.health_failure(
                    'INPUT_STALE', f'topic={topic} receipt_age={receipt_age:.3f}s limit={limit:.3f}s')
        # Receipt time alone would accept a publisher replaying stale sensor data.
        for k in ('map', 'costmap', 'scan'):
            stamp = self.samples[k][0].header.stamp
            age = (self.get_clock().now().nanoseconds / 1e9
                   - stamp.sec - stamp.nanosec / 1e9)
            if age < -0.5 or age > limit:
                return self.health_failure(
                    'STAMP_OUT_OF_RANGE', f'topic={topics[k]} age={age:.3f}s '
                    f'stamp={stamp.sec}.{stamp.nanosec:09d} allowed=[-0.5,{limit}]s')
        scan = self.samples['scan'][0]
        if not scan.ranges or not any(math.isfinite(x) and scan.range_min <= x <= scan.range_max
                                      for x in scan.ranges):
            return self.health_failure('SCAN_INVALID', 'topic=/scan 측정 범위 내 유한한 거리값 없음')
        frame = self.samples['map'][0].header.frame_id
        try:
            self.robot_pose(frame)
        except Exception as exc:
            return self.health_failure(
                'ROBOT_TF_UNAVAILABLE', f'target={frame} source={self.param("base_frame")} '
                f'request=latest error={type(exc).__name__}: {exc}')
        try:
            self.tf.lookup_transform(frame, scan.header.frame_id,
                                     rclpy.time.Time.from_msg(scan.header.stamp))
        except Exception as exc:
            stamp = scan.header.stamp
            return self.health_failure(
                'SCAN_TF_UNAVAILABLE', f'target={frame} source={scan.header.frame_id} '
                f'request={stamp.sec}.{stamp.nanosec:09d} error={type(exc).__name__}: {exc}')
        costmap_frame = self.samples['costmap'][0].header.frame_id
        if costmap_frame != frame:
            return self.health_failure('FRAME_MISMATCH', f'map={frame} costmap={costmap_frame}')
        if self.health_log_code is not None:
            self.get_logger().info('INPUTS_READY: 센서·지도·costmap·TF 검사 통과')
        self.last_health_issue = ''
        self.health_log_code = None
        return True

    def robot_pose(self, frame):
        transform = self.tf.lookup_transform(frame, self.param('base_frame'), rclpy.time.Time())
        stamp = transform.header.stamp
        age = self.get_clock().now().nanoseconds / 1e9 - stamp.sec - stamp.nanosec / 1e9
        if age < -0.5 or age > self.param('data_timeout_sec'):
            raise RuntimeError(f'Stale robot TF: target={frame} source={self.param("base_frame")} '
                               f'age={age:.3f}s')
        return (transform.transform.translation.x, transform.transform.translation.y)

    # 탐색 프로세스 종료 후 Nav2의 남은 목표도 취소한다. Nav2 독점 사용이 전제다.
    def stop_explorer(self):
        if self.process is None:
            return
        proc, self.process = self.process, None
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=2)
        # Dedicated Nav2 instance required: this cancels every NavigateToPose goal.
        try:
            response = self.call(self.cancel_nav, CancelGoal.Request(), interrupt=False)
            if response.return_code not in (0, 2):
                raise RuntimeError('Nav2 did not acknowledge cancellation')
            self.until(lambda: not self.nav_busy(), self.param('service_timeout_sec'), False)
        except Exception:
            self.healthy = False
            raise

    # 매 회차 Python 탐색 노드를 새로 실행하여 이전 회차의 제외 목록을 초기화한다.
    def explore(self, retries):
        self.until(self.ready, 30)
        token = uuid.uuid4().hex
        status = []
        topic = '/auto_mapping/session_' + token
        def receive_status(message):
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                payload = {'status': message.data}  # 이전 시험 노드의 상태 형식도 읽는다.
            if isinstance(payload, dict) and isinstance(payload.get('status'), str):
                self.explorer_detail = payload
                status.append(payload['status'])
                self.record('explorer_status', **payload)

        sub = self.create_subscription(String, topic, receive_status,
                                       QoSProfile(depth=10,
                                                  durability=DurabilityPolicy.TRANSIENT_LOCAL),
                                       callback_group=self.group)
        def receive_event(message):
            try:
                self.record('explorer_event', payload=json.loads(message.data))
            except (ValueError, TypeError) as exc:
                self.record('explorer_event_decode_error', detail=str(exc))
        event_sub = self.create_subscription(String, topic+'/events', receive_event, 100,
                                             callback_group=self.group)
        config = Path(get_package_share_directory('limo_auto_mapping')) / 'config/explore.yaml'
        command = ['ros2', 'run', 'limo_auto_mapping', 'frontier_explorer', '--ros-args',
                   '--params-file', str(config), '-p',
                   'use_sim_time:=' + str(self.get_parameter('use_sim_time').value).lower(),
                   '-r', 'explore/status:=' + topic, '-r', 'explore/events:=' + topic+'/events']
        try:
            self.record('explorer_started', retries=retries, command=command)
            with open(self.journal.explorer_path, 'a', encoding='utf-8') as output:
                self.process = subprocess.Popen(command, start_new_session=True,
                                                stdout=output, stderr=subprocess.STDOUT)
            while not any(s in ('exploration_complete', 'frontiers_exhausted', 'exploration_error') for s in status):
                self.check()
                if self.process.poll() is not None:
                    raise MappingFailure('EXPLORER_EXITED', 'Exploration process exited unexpectedly')
                if not self.ready():
                    raise MappingFailure('HEALTH_LOST', 'Sensor/map/costmap/TF health lost; '
                                         + self.last_health_issue)
                self.feedback('exploring_reduced' if retries else 'exploring', retries)
                time.sleep(0.25)
            if status[-1] == 'exploration_error':
                raise MappingFailure('EXPLORER_ERROR', 'Python explorer error: '
                                     + json.dumps(self.explorer_detail, ensure_ascii=False))
            return status[-1]
        finally:
            try:
                self.stop_explorer()
            finally:
                self.destroy_subscription(sub)
                self.destroy_subscription(event_sub)

    # 로봇 크기는 줄이지 않는다. inflation만 설정된 하한 안에서 변경할 수 있다.
    def snapshot(self):
        radii = []
        for node, (getter, _) in self.params.items():
            req = GetParameters.Request()
            req.names = ['inflation_layer.inflation_radius', 'robot_radius', 'footprint',
                         'inflation_layer.enabled']
            values = self.call(getter, req).values
            if len(values) != 4 or values[0].type != 3 or values[1].type != 3:
                raise RuntimeError('Expected double inflation/robot radius parameters')
            if values[2].string_value not in ('', '[]'):
                raise RuntimeError('Polygon footprint needs a separately reviewed clearance bound')
            if values[3].type != 1 or not values[3].bool_value:
                raise RuntimeError('Inflation layer is not enabled')
            radius = values[0].double_value
            self.snapshots[node] = radius
            radii.append(radius)
            validate_radii([radius], self.param('reduced_inflation_radius'),
                           values[1].double_value, self.param('minimum_clearance'))
        return radii

    def set_radius(self, node, radius, interrupt=True):
        req = SetParametersAtomically.Request()
        req.parameters = [Parameter('inflation_layer.inflation_radius', value=radius).to_parameter_msg()]
        response = self.call(self.params[node][1], req, interrupt)
        if not response.result.successful:
            raise RuntimeError('Costmap rejected inflation change: ' + response.result.reason)
        # Check effective value, including restoration.
        get = GetParameters.Request()
        get.names = ['inflation_layer.inflation_radius']
        actual = self.call(self.params[node][0], get, interrupt).values[0].double_value
        if abs(actual - radius) > 1e-6:
            raise RuntimeError('Inflation readback mismatch')

    # 두 costmap을 각각 복구한다. 하나라도 복구에 실패하면 다음 요청을 차단한다.
    def restore(self):
        errors = []
        for node in list(self.changed):
            try:
                self.set_radius(node, self.snapshots[node], interrupt=False)
                self.changed.remove(node)
            except Exception as exc:
                errors.append(f'{node}: {exc}')
        if errors:
            self.healthy = False
            raise RuntimeError('RESTORATION FAILED; stop robot and inspect: ' + '; '.join(errors))

    # 주행 전에 경로만 요청한다. 목표 공차 때문에 멀리서 끝난 경로는 통과시키지 않는다.
    def can_plan(self, point, frame):
        self.until(self.planner.server_is_ready, 10)
        req = ComputePathToPose.Goal()
        req.goal = PoseStamped()
        req.goal.header.frame_id = frame
        req.goal.header.stamp = self.get_clock().now().to_msg()
        req.goal.pose.position.x, req.goal.pose.position.y = point
        req.goal.pose.orientation.w = 1.0
        req.use_start = False
        req.planner_id = 'GridBased'
        handle = self.future(self.planner.send_goal_async(req))
        if not handle.accepted:
            return False
        try:
            result = self.future(handle.get_result_async())
            poses = result.result.path.poses
            if result.status != GoalStatus.STATUS_SUCCEEDED or not poses:
                return False
            end = poses[-1].pose.position
            raw, inflated = self.samples['map'][0], self.samples['costmap'][0]
            if math.hypot(end.x - point[0], end.y - point[1]) > raw.info.resolution * 1.5:
                return False  # Navfn tolerance may otherwise stop short of the frontier.
            for pose in poses:
                xy = (pose.pose.position.x, pose.pose.position.y)
                a, b = cost(raw, xy), cost(inflated, xy)
                if a is None or b is None or not (0 <= a < 50 and 0 <= b < 99):
                    return False  # Do not approve a probe through unknown/obstacles.
            return True
        finally:
            # Also cancel a probe if the session was cancelled or a result timed out.
            self.future(handle.cancel_goal_async(), interrupt=False)

    # 단순 주행 실패로 마진을 줄이지 않고, 비용 변화와 경로 생성 근거를 확인한다.
    def retry_margin(self):
        self.until(self.ready, 10)
        raw, old = self.samples['map'][0], self.samples['costmap'][0]
        points = suppressed_candidates(raw, old, self.robot_pose(raw.header.frame_id))
        if not points:
            return False
        self.snapshot()
        for node in self.params:
            # Mark BEFORE sending: timeout may leave a successful remote mutation.
            self.changed.add(node)
            self.set_radius(node, self.param('reduced_inflation_radius'))
        changed_at = time.monotonic()
        self.until(lambda: self.ready() and self.samples['costmap'][1] > changed_at + 2.0, 15)
        new = self.samples['costmap'][0]
        if ((old.info.width, old.info.height, old.info.resolution, old.info.origin)
                != (new.info.width, new.info.height, new.info.resolution, new.info.origin)):
            self.restore()
            return False  # Map geometry changed: comparison would be ambiguous.
        for point in [p for p in points if cost(new, p) == 0][:self.param('max_path_probes')]:
            if self.can_plan(point, raw.header.frame_id):
                return True
        self.restore()
        return False

    # 이어 맵핑용 pose graph 쌍과 일반 지도 이미지를 함께 저장한다.
    def save(self, prefix):
        prefix = Path(prefix)
        prefix.parent.mkdir(parents=True, exist_ok=True)
        if any(Path(str(prefix) + ext).exists() for ext in ('.posegraph', '.data', '.yaml', '.pgm', '.png')):
            raise RuntimeError('Output already exists; refusing overwrite')
        req = SerializePoseGraph.Request()
        req.filename = str(prefix)
        if self.call(self.serialize, req).result != 0:
            raise RuntimeError('Pose graph serialization failed')
        if not all(Path(str(prefix) + ext).is_file() and Path(str(prefix) + ext).stat().st_size > 0
                   for ext in ('.posegraph', '.data')):
            raise RuntimeError('Serialized pose graph pair missing on this host')
        req = SaveMap.Request()
        req.name.data = str(prefix)
        if self.call(self.save_map, req).result != 0:
            raise RuntimeError('Occupancy image save failed; pose graph may already exist')
        if not Path(str(prefix) + '.yaml').is_file():
            raise RuntimeError('Map YAML missing on this host')

    # 정상·취소·오류 모두 같은 정리 경로를 거쳐 종료 사유와 파일 경로를 반환한다.
    def execute(self, handle):
        self.goal = handle
        self.started = time.monotonic()
        self.deadline = (float('inf') if handle.request.max_duration_sec == 0
                         else self.started + handle.request.max_duration_sec)
        self.snapshots, self.changed = {}, set()
        self.journal = None
        self.recording_error = ''
        self.last_phase = None
        self.explorer_detail = {}
        result = AutoMapping.Result()
        result.termination_reason = 'INTERNAL_ERROR'
        cancelled = False
        cleanup_errors = []
        try:
            try:
                self.journal = SessionJournal(handle.request.output_prefix)
            except Exception as exc:
                raise MappingFailure('LOG_WRITE_FAILED', f'Cannot create session files: {exc}') from exc
            self.record('session_started', output_prefix=handle.request.output_prefix,
                        require_depth_ready=getattr(self, 'require_depth_ready', False),
                        depth_status=getattr(self, 'depth_status', None),
                        allow_margin_retry=handle.request.allow_margin_retry,
                        max_duration_sec=handle.request.max_duration_sec)
            self.feedback('waiting_for_inputs', 0)
            self.until(self.ready, 30)
            if self.nav_busy():
                raise MappingFailure('NAV2_BUSY', 'Nav2 already has a navigation goal')
            reason = self.explore(0)
            if handle.request.allow_margin_retry and self.retry_margin():
                result.margin_retried = True
                self.record('margin_retry_started', reduced_radius=self.param('reduced_inflation_radius'))
                reason = self.explore(1)
            self.restore()
            self.until(self.ready, 10)
            remaining = frontiers(self.samples['map'][0],
                                  self.robot_pose(self.samples['map'][0].header.frame_id))
            result.exploration_complete = reason == 'exploration_complete' and not remaining
            result.termination_reason = ('EXPLORATION_COMPLETE' if result.exploration_complete
                                         else 'FRONTIERS_EXHAUSTED')
            self.feedback('saving', int(result.margin_retried))
            self.record('map_save_started')
            try:
                self.save(handle.request.output_prefix)
            except SessionCancelled:
                raise
            except MappingFailure:
                raise
            except Exception as exc:
                raise MappingFailure('SAVE_FAILED', str(exc)) from exc
            result.posegraph_prefix = handle.request.output_prefix
            self.record('map_saved', posegraph_prefix=result.posegraph_prefix)
            result.success = result.exploration_complete
            result.message = ('No reachable raw-map frontier remains; saved image and pose graph'
                              if result.success else
                              'Partial map saved; remaining or exhausted frontiers; coverage not complete')
        except SessionCancelled as exc:
            cancelled = True
            result.termination_reason = 'USER_CANCELLED'
            result.message = str(exc) + '; automatic save skipped'
            self.record('cancellation_observed', message=str(exc))
        except Exception as exc:
            result.termination_reason = getattr(exc, 'reason', 'INTERNAL_ERROR')
            result.message = str(exc)
            self.record('session_error', reason=result.termination_reason,
                        message=str(exc), traceback=traceback.format_exc())
            self.get_logger().error('AUTO_MAPPING_FAILED: ' + result.message)
        finally:
            for operation in (self.stop_explorer, self.restore):
                try:
                    operation()
                except Exception as exc:
                    self.healthy = False
                    result.success = False
                    cleanup_errors.append(str(exc))
                    result.message += '; CLEANUP FAILED: ' + str(exc)
                    self.record('cleanup_error', operation=operation.__name__, message=str(exc))
            if handle.is_cancel_requested:
                cancelled = True
                result.success = False
                if not cleanup_errors:
                    result.termination_reason = 'USER_CANCELLED'
                self.record('cancellation_observed', during='finalization')
            if cleanup_errors:
                result.termination_reason = 'CLEANUP_FAILED'
            if not self.healthy:
                result.success = False
                result.termination_reason = 'CLEANUP_FAILED'
            if self.recording_error:
                result.termination_reason = 'LOG_WRITE_FAILED'
                result.success = False
                result.message += '; LOG FAILED: ' + self.recording_error
            state = ('CANCELED' if cancelled and handle.is_cancel_requested and self.healthy
                     else 'SUCCEEDED' if result.success and self.healthy else 'ABORTED')
            journal = self.journal
            if journal is not None:
                result.event_log_path = journal.event_path
                result.summary_path = journal.summary_path
                result.explorer_log_path = journal.explorer_path
                summary = {
                    'state': state, 'termination_reason': result.termination_reason,
                    'message': result.message, 'success': result.success,
                    'exploration_complete': result.exploration_complete,
                    'margin_retried': result.margin_retried,
                    'posegraph_prefix': result.posegraph_prefix,
                    'output_prefix': handle.request.output_prefix,
                    'elapsed_sec': time.monotonic()-self.started,
                    'explorer_detail': self.explorer_detail,
                    'last_health_issue': getattr(self, 'last_health_issue', ''),
                    'cleanup_errors': cleanup_errors,
                    'saved_files': {ext: Path(handle.request.output_prefix+ext).is_file()
                                    for ext in ('.yaml', '.pgm', '.png', '.posegraph', '.data')},
                    'event_log_path': result.event_log_path,
                    'explorer_log_path': result.explorer_log_path}
                self.record('session_finished', **summary)
                if self.recording_error:
                    result.success = False
                    result.termination_reason = 'LOG_WRITE_FAILED'
                    result.message += '; final event write failed: ' + self.recording_error
                    state = 'CANCELED' if state == 'CANCELED' else 'ABORTED'
                    summary.update(state=state, success=False, termination_reason=result.termination_reason,
                                   message=result.message)
                try:
                    journal.finish(summary)
                except Exception as exc:
                    result.success = False
                    result.termination_reason = 'LOG_WRITE_FAILED'
                    result.message += '; final log write failed: ' + str(exc)
                    state = 'CANCELED' if state == 'CANCELED' else 'ABORTED'
                    self.get_logger().error(result.message)
            if state == 'CANCELED':
                handle.canceled()
            elif state == 'SUCCEEDED':
                handle.succeed()
            else:
                handle.abort()
            self.journal = None
            self.goal = None
            with self.lock:
                self.busy = False
        return result


def main(args=None):
    # Keep ROS alive during SIGINT/SIGTERM cleanup so cancellation and parameter
    # restoration can still reach their remote services.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MappingServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    def request_stop(signum, frame):
        node.stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not (node.stop_requested and not node.busy):
            executor.spin_once(timeout_sec=0.2)
    finally:
        executor.shutdown(timeout_sec=30)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
