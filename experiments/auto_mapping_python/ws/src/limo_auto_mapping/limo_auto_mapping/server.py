"""자동 맵핑 Action: 탐색 → 조건부 마진 재시도 → 복원 → 지도 저장을 관리한다.

실제 경로 추종은 기존 Nav2가 맡고, 이 파일은 속도 명령을 직접 발행하지 않는다.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from limo_mapping_interfaces.action import AutoMapping
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
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


class SessionCancelled(Exception):
    pass


class MappingServer(Node):
    def __init__(self):
        super().__init__('auto_mapping_server')
        defaults = {'reduced_inflation_radius': 0.24, 'minimum_clearance': 0.05,
                    'base_frame': 'base_link', 'data_timeout_sec': 5.0,
                    'service_timeout_sec': 10.0, 'max_path_probes': 12}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        # Action 실행을 기다리는 중에도 센서·서비스 응답을 처리하기 위한 콜백 그룹이다.
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.busy = False
        self.stop_requested = False
        self.healthy = True
        self.samples = {}
        self.nav_status = []
        self.process = None
        self.goal = None
        self.deadline = float('inf')
        # 마진 변경 전 실제 값을 저장해 종료·취소·실패 시 같은 값으로 되돌린다.
        self.snapshots = {}
        self.changed = set()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
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

    def param(self, name):
        return self.get_parameter(name).value

    def sample(self, key, msg):
        self.samples[key] = (msg, time.monotonic())

    def on_status(self, msg):
        self.nav_status = [s.status for s in msg.status_list]

    def nav_busy(self):
        return any(s in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                         GoalStatus.STATUS_CANCELING) for s in self.nav_status)

    # 다른 탐색과 충돌하거나 기존 지도 파일을 덮어쓰는 요청은 수락하지 않는다.
    def accept(self, request):
        with self.lock:
            prefix = Path(request.output_prefix)
            valid = (request.output_prefix and prefix.is_absolute()
                     and prefix.name not in ('', '.', '..')
                     and math.isfinite(request.max_duration_sec)
                     and 10 <= request.max_duration_sec <= 86400)
            if self.busy or self.stop_requested or not self.healthy or not valid or self.nav_busy():
                return GoalResponse.REJECT
            if any(Path(str(prefix) + ext).exists()
                   for ext in ('.posegraph', '.data', '.yaml', '.pgm', '.png')):
                return GoalResponse.REJECT
            self.busy = True
            return GoalResponse.ACCEPT

    def check(self):
        if self.stop_requested or (self.goal and self.goal.is_cancel_requested):
            raise SessionCancelled('Cancellation or server shutdown requested')
        if not rclpy.ok() or time.monotonic() >= self.deadline:
            raise RuntimeError('Session deadline reached or ROS shutting down')

    def until(self, predicate, timeout, interrupt=True):
        if interrupt:
            self.check()
        end = time.monotonic() + timeout
        while not predicate():
            if interrupt:
                self.check()
            if time.monotonic() >= end:
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
        msg = AutoMapping.Feedback()
        msg.phase = phase
        msg.elapsed_sec = time.monotonic() - self.started
        msg.retry_count = retries
        self.goal.publish_feedback(msg)

    # 수신 여부뿐 아니라 센서 시각과 TF도 검사해 오래된 데이터로 움직이지 않게 한다.
    def ready(self):
        now = time.monotonic()
        limit = self.param('data_timeout_sec')
        if any(k not in self.samples or now - self.samples[k][1] > limit
               for k in ('map', 'costmap', 'scan')):
            return False
        # Receipt time alone would accept a publisher replaying stale sensor data.
        for k in ('map', 'costmap', 'scan'):
            stamp = self.samples[k][0].header.stamp
            age = (self.get_clock().now().nanoseconds / 1e9
                   - stamp.sec - stamp.nanosec / 1e9)
            if age < -0.5 or age > limit:
                return False
        scan = self.samples['scan'][0]
        if not scan.ranges or not any(math.isfinite(x) and scan.range_min <= x <= scan.range_max
                                      for x in scan.ranges):
            return False
        try:
            frame = self.samples['map'][0].header.frame_id
            self.robot_pose(frame)
            self.tf.lookup_transform(frame, scan.header.frame_id,
                                     rclpy.time.Time.from_msg(scan.header.stamp))
        except Exception:
            return False
        return self.samples['costmap'][0].header.frame_id == frame

    def robot_pose(self, frame):
        transform = self.tf.lookup_transform(frame, self.param('base_frame'), rclpy.time.Time())
        stamp = transform.header.stamp
        age = self.get_clock().now().nanoseconds / 1e9 - stamp.sec - stamp.nanosec / 1e9
        if age < -0.5 or age > self.param('data_timeout_sec'):
            raise RuntimeError('Stale robot TF')
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
        sub = self.create_subscription(String, topic, lambda m: status.append(m.data),
                                       QoSProfile(depth=10,
                                                  durability=DurabilityPolicy.TRANSIENT_LOCAL),
                                       callback_group=self.group)
        config = Path(get_package_share_directory('limo_auto_mapping')) / 'config/explore.yaml'
        command = ['ros2', 'run', 'limo_auto_mapping', 'frontier_explorer', '--ros-args',
                   '--params-file', str(config), '-p',
                   'use_sim_time:=' + str(self.get_parameter('use_sim_time').value).lower(),
                   '-r', 'explore/status:=' + topic]
        try:
            self.process = subprocess.Popen(command, start_new_session=True)
            while not any(s in ('exploration_complete', 'frontiers_exhausted', 'exploration_error') for s in status):
                self.check()
                if self.process.poll() is not None:
                    raise RuntimeError('Exploration process exited unexpectedly')
                if not self.ready():
                    raise RuntimeError('Sensor/map/costmap/TF health lost; stopping exploration')
                self.feedback('exploring_reduced' if retries else 'exploring', retries)
                time.sleep(0.25)
            if status[-1] == 'exploration_error':
                raise RuntimeError('Python explorer reported an error; inspect explorer log')
            return status[-1]
        finally:
            try:
                self.stop_explorer()
            finally:
                self.destroy_subscription(sub)

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

    # 전체 Action의 흐름이다. finally에서 정리하므로 오류가 나도 복원을 시도한다.
    def execute(self, handle):
        self.goal = handle
        self.started = time.monotonic()
        self.deadline = self.started + handle.request.max_duration_sec
        # 마진 변경 전 실제 값을 저장해 종료·취소·실패 시 같은 값으로 되돌린다.
        self.snapshots = {}
        self.changed = set()
        result = AutoMapping.Result()
        cancelled = False
        try:
            self.feedback('waiting_for_inputs', 0)
            self.until(self.ready, 30)
            if self.nav_busy():
                raise RuntimeError('Nav2 already has a navigation goal')
            reason = self.explore(0)
            if handle.request.allow_margin_retry and self.retry_margin():
                result.margin_retried = True
                reason = self.explore(1)
            self.restore()
            self.until(self.ready, 10)
            remaining = frontiers(self.samples['map'][0],
                                  self.robot_pose(self.samples['map'][0].header.frame_id))
            result.exploration_complete = reason == 'exploration_complete' and not remaining
            self.feedback('saving', int(result.margin_retried))
            self.save(handle.request.output_prefix)
            result.posegraph_prefix = handle.request.output_prefix
            result.success = result.exploration_complete
            result.message = ('No reachable raw-map frontier remains; saved image and pose graph'
                              if result.success else 'Partial map saved; remaining or exhausted frontiers; coverage not complete')
        except SessionCancelled as exc:
            cancelled = True
            result.message = str(exc) + '; automatic save skipped'
        except Exception as exc:
            result.message = str(exc)
        finally:
            for operation in (self.stop_explorer, self.restore):
                try:
                    operation()
                except Exception as exc:
                    self.healthy = False
                    result.success = False
                    result.message += '; CLEANUP FAILED: ' + str(exc)
            if handle.is_cancel_requested:
                cancelled = True
                result.success = False
                result.message += '; cancellation accepted during finalization'
            if not self.healthy:
                handle.abort()
            elif cancelled and handle.is_cancel_requested:
                handle.canceled()
            elif result.success:
                handle.succeed()
            else:
                handle.abort()
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
