"""정지 상태 전역 위치 추정 → 검증 → AMCL 종료 → 이어 맵핑. 모터 명령은 보내지 않는다."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from .localization import MapMatcher, StablePose, close_pose, covariance_ok, planar_pose


# 이 단계에서는 기존 SLAM·AMCL·Nav2와 동시에 실행하지 않는다.
CONFLICTS = {'slam_toolbox', 'amcl', 'map_server', 'auto_mapping_server',
             'controller_server', 'bt_navigator', 'explore_node'}


def validate_inputs(graph, map_file):
    graph = Path(graph).expanduser()
    if not graph.is_absolute():
        raise ValueError('posegraph must be an absolute prefix without extension')
    for ext in ('.posegraph', '.data'):
        p = Path(str(graph)+ext)
        if not p.is_file() or p.stat().st_size == 0:
            raise ValueError('Missing/empty serialized graph: '+str(p))
    path = Path(map_file).expanduser() if map_file else Path(str(graph)+'.yaml')
    if not path.is_absolute() or not path.is_file():
        raise ValueError('Missing absolute map YAML: '+str(path))
    doc = yaml.safe_load(path.read_text())
    if not isinstance(doc, dict) or not isinstance(doc.get('image'), str):
        raise ValueError('Map YAML must contain image')
    image = Path(doc['image']).expanduser()
    if not image.is_absolute():
        image = path.parent/image
    if not image.is_file():
        raise ValueError('Map image missing: '+str(image))
    return graph, path


def localization_config(source, use_sim_time, global_particles=20000):
    """기존 YAML은 읽기만 한다. 자동 전역 초기화 전에 임의 초기 위치를 주지 않는다."""
    config = yaml.safe_load(Path(source).read_text())
    amcl = config['amcl']['ros__parameters']
    if (amcl.get('global_frame_id') != 'map' or amcl.get('odom_frame_id') != 'odom'
            or amcl.get('base_frame_id') != 'base_link'):
        raise ValueError('Auto resume currently requires map/odom/base_link frames')
    amcl.update(use_sim_time=use_sim_time, set_initial_pose=False,
                always_reset_initial_pose=True, save_pose_rate=-1.0, tf_broadcast=True)
    # 기존 초기 위치 근방 추정보다 지도 전체 탐색에서는 후보를 많이 확보한다.
    amcl['max_particles'] = max(int(amcl.get('min_particles', 500)), global_particles)
    return config


def stop_child(process):
    """자신이 시작한 프로세스 그룹만 종료하고 종료 여부를 확인한다."""
    if process is None:
        return
    # launch 부모가 먼저 끝났더라도 남은 자식 노드를 같은 그룹에서 정리한다.
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)
        raise RuntimeError('Child did not shut down cleanly; no handoff')


class ResumeMapping(Node):
    def __init__(self):
        super().__init__('auto_mapping_resume')
        defaults = {'posegraph': '', 'map': '', 'localization_timeout_sec': 120.0,
                    'mapping_ready_timeout_sec': 45.0, 'service_timeout_sec': 10.0,
                    'freshness_sec': 2.0, 'stable_sec': 5.0, 'stable_samples': 5,
                    'localization_attempts': 2, 'max_xy_std': .20, 'max_yaw_std': .15,
                    'global_particles': 20000,
                    'min_scan_beams': 30, 'min_hit_ratio': .75, 'min_known_ratio': .80,
                    'max_blocked_ratio': .05, 'min_hit_sectors': 3,
                    'wall_tolerance': .15, 'minimum_clearance': .05,
                    'max_stationary_distance': .05, 'max_stationary_yaw': .05}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name, default in defaults.items():
            if isinstance(default, (int, float)):
                value = self.p(name)
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(name+' must be finite and positive')
        for name in ('min_hit_ratio', 'min_known_ratio', 'max_blocked_ratio'):
            if self.p(name) > 1:
                raise ValueError(name+' must be <= 1')
        if self.p('localization_attempts') < 2:
            raise ValueError('At least two independent global initializations are required')
        if self.p('global_particles') > 100000:
            raise ValueError('global_particles must be <= 100000')
        self.stopping = False
        self.owns_stack = False
        self.child = None
        self.samples = {}
        self.matcher = None
        self.anchor = None
        self.clearance = .24
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 lambda m: self.receive('pose', m), latched)
        self.create_subscription(LaserScan, '/scan', lambda m: self.receive('scan', m),
                                 qos_profile_sensor_data)
        self.readiness = self.create_publisher(Bool, '/auto_mapping/localization_ready', latched)
        self.status = self.create_publisher(String, '/auto_mapping/resume_status', latched)
        self.reset_global = self.create_client(Empty, '/reinitialize_global_localization')
        self.no_motion = self.create_client(Empty, '/request_nomotion_update')
        self.amcl_state = self.create_client(GetState, '/amcl/get_state')
        self.set_status('waiting', 'No motion commands are issued during localization')

    def p(self, name):
        return self.get_parameter(name).value

    def receive(self, key, msg):
        self.samples[key] = (msg, time.monotonic())

    def on_map(self, msg):
        try:
            if msg.header.frame_id != 'map':
                raise ValueError('Map frame must be map')
            self.matcher = MapMatcher(msg, self.p('wall_tolerance'), self.clearance)
            self.receive('map', msg)
        except Exception as exc:
            self.matcher = None
            self.get_logger().error(str(exc))

    def set_status(self, state, detail):
        self.status.publish(String(data=json.dumps({'state': state, 'detail': detail})))
        self.get_logger().info(state+': '+detail)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def check(self):
        if self.stopping or not rclpy.ok():
            raise InterruptedError('Resume stopped by user')
        if self.child is not None and self.child.poll() is not None:
            raise RuntimeError('Child launch exited; no automatic continuation')

    def spin(self):
        self.check()
        rclpy.spin_once(self, timeout_sec=.1)

    def wait(self, predicate, seconds):
        end = time.monotonic()+seconds
        while not predicate():
            self.spin()
            if time.monotonic() >= end:
                raise TimeoutError('Timed out waiting for localization/TF/service')

    def call(self, client, request):
        self.wait(client.service_is_ready, self.p('service_timeout_sec'))
        future = client.call_async(request)
        self.wait(future.done, self.p('service_timeout_sec'))
        return future.result()

    def transform(self, target, source, stamp=None, dynamic=False):
        t = self.tf.lookup_transform(target, source,
                                     rclpy.time.Time.from_msg(stamp) if stamp else rclpy.time.Time())
        if dynamic and not -.5 <= self.now()-self.stamp(t) <= self.p('freshness_sec'):
            raise ValueError('Stale '+target+' -> '+source+' TF')
        pose = Pose()
        pose.position.x, pose.position.y = t.transform.translation.x, t.transform.translation.y
        pose.orientation = t.transform.rotation
        return planar_pose(pose), self.stamp(t)

    def stationary(self):
        pose, _ = self.transform('odom', 'base_link', dynamic=True)
        if self.anchor is None:
            self.anchor = pose
        if not close_pose(pose, self.anchor, self.p('max_stationary_distance'),
                          self.p('max_stationary_yaw')):
            raise RuntimeError('Robot moved during localization/handoff; restart from rest')

    def stationary_ready(self):
        try:
            self.stationary()
            return True
        except (ValueError, TransformException):
            return False  # 자식 종료 대기 중 쌓인 최신 TF 콜백을 먼저 처리한다.

    def fresh(self, key, stamped=True):
        if key not in self.samples:
            raise ValueError('Waiting for '+key)
        msg, received = self.samples[key]
        if time.monotonic()-received > self.p('freshness_sec'):
            raise ValueError('Stale '+key)
        if stamped and not -.5 <= self.now()-self.stamp(msg) <= self.p('freshness_sec'):
            raise ValueError('Old/future '+key+' stamp')
        return msg

    def scan_matches(self, pose, scan):
        laser, _ = self.transform('base_link', scan.header.frame_id, scan.header.stamp)
        result = self.matcher.score(pose, scan, laser)
        passed = (result.beams >= self.p('min_scan_beams')
                  and result.hit_ratio >= self.p('min_hit_ratio')
                  and result.known_ratio >= self.p('min_known_ratio')
                  and result.blocked_ratio <= self.p('max_blocked_ratio')
                  and result.sectors >= self.p('min_hit_sectors'))
        if not passed:
            raise ValueError(f'Scan/map mismatch: {result}')

    def estimate(self):
        self.call(self.reset_global, Empty.Request())
        # 이전 시도의 latched pose가 다음 시도의 성공 판정을 만들지 못하게 한다.
        self.samples.pop('pose', None)
        started_stamp = self.now()
        stable = StablePose(self.p('stable_sec'), self.p('stable_samples'))
        last_request = last_report = 0.
        while time.monotonic() < self.localization_end:
            self.spin()
            self.stationary()
            if time.monotonic()-last_request >= 1.0:
                # 움직이지 않아도 새 scan으로 AMCL을 갱신한다. 속도 명령은 아니다.
                self.call(self.no_motion, Empty.Request())
                last_request = time.monotonic()
            try:
                scan, msg = self.fresh('scan'), self.fresh('pose')
                if self.matcher is None or msg.header.frame_id != 'map':
                    raise ValueError('Waiting for map-frame localization')
                if stable.last_stamp is not None and self.stamp(msg) <= stable.last_stamp:
                    continue  # 같은 pose를 여러 번 검사해 안정 관측 수를 부풀리지 않는다.
                if self.stamp(msg) <= started_stamp or abs(self.stamp(msg)-self.stamp(scan)) > .5:
                    raise ValueError('Waiting for synchronized new pose/scan')
                pose = planar_pose(msg.pose.pose)
                if not covariance_ok(msg.pose.covariance, self.p('max_xy_std'), self.p('max_yaw_std')):
                    raise ValueError('AMCL covariance has not converged')
                tf_pose, _ = self.transform('map', 'base_link')
                if not close_pose(pose, tf_pose):
                    raise ValueError('AMCL pose and map TF disagree')
                laser, _ = self.transform('base_link', scan.header.frame_id, scan.header.stamp)
                pose = self.matcher.refine_pose(pose, scan, laser)
                self.scan_matches(pose, scan)
                if stable.observe(time.monotonic(), self.stamp(msg), pose, True):
                    return pose
            except ValueError as exc:
                stable.observe(time.monotonic(), 0, (0, 0, 0), False)
                if time.monotonic()-last_report > 3:
                    self.set_status('localizing', str(exc))
                    last_report = time.monotonic()
            except Exception as exc:
                # TF가 아직 준비되지 않은 동안은 주행 없이 제한 시간 내에서 기다린다.
                stable.reset()
                if time.monotonic()-last_report > 3:
                    self.set_status('localizing', str(exc))
                    last_report = time.monotonic()
        raise TimeoutError('Localization not reliable before timeout; use manual start_pose or retry elsewhere')

    def launch(self, arguments):
        self.check()
        if self.child is not None:
            raise RuntimeError('Previous child still owned; refusing overlapping stacks')
        self.child = subprocess.Popen(arguments, start_new_session=True)

    def stop_stack(self):
        child, self.child = self.child, None
        stop_child(child)

    def run(self):
        graph, map_file = validate_inputs(self.p('posegraph'), self.p('map'))
        # DDS discovery에 잠깐 시간을 준 후 중복 스택이 없는지 확인한다.
        end = time.monotonic()+2
        while time.monotonic() < end:
            self.spin()
        names = self.get_node_names()
        conflicts = CONFLICTS.intersection(names)
        if conflicts or names.count('auto_mapping_resume') > 1:
            raise RuntimeError('Stop existing mapping/localization/Nav2 first: '+str(sorted(conflicts)))
        self.owns_stack = True
        self.readiness.publish(Bool(data=False))
        share = Path(get_package_share_directory('limo_capstone'))
        config = localization_config(share/'config/nav2_explore_params.yaml',
                                     self.get_parameter('use_sim_time').value,
                                     self.p('global_particles'))
        # 기존 로봇 반경을 기준으로만 검사한다. 실제 footprint 검증을 대신하지 않는다.
        global_params = config['global_costmap']['global_costmap']['ros__parameters']
        if global_params.get('footprint') not in (None, '', '[]'):
            raise ValueError('Polygon footprint needs a separately reviewed localization clearance check')
        self.clearance = float(global_params['robot_radius'])+self.p('minimum_clearance')
        if not math.isfinite(self.clearance) or self.clearance <= 0:
            raise ValueError('Invalid configured robot radius')
        sim = str(self.get_parameter('use_sim_time').value).lower()
        with tempfile.TemporaryDirectory(prefix='limo_relocalize_') as temp:
            params = Path(temp)/'amcl.yaml'
            params.write_text(yaml.safe_dump(config))
            self.launch(['ros2', 'launch', 'nav2_bringup', 'localization_launch.py',
                         'map:='+str(map_file), 'params_file:='+str(params),
                         'use_sim_time:='+sim, 'autostart:=true', 'use_composition:=False'])
            self.wait(lambda: self.matcher is not None and self.amcl_state.service_is_ready(), 30)
            active_end = time.monotonic()+20
            while self.call(self.amcl_state, GetState.Request()).current_state.id != 3:
                if time.monotonic() >= active_end:
                    raise TimeoutError('AMCL did not activate')
                self.spin()
            self.wait(self.stationary_ready, 10)
            self.localization_end = time.monotonic()+self.p('localization_timeout_sec')
            estimates = []
            for attempt in range(self.p('localization_attempts')):
                self.set_status('localizing', f'Independent global initialization {attempt+1}')
                pose = self.estimate()
                if estimates and not all(close_pose(pose, p, .25, .20) for p in estimates):
                    raise RuntimeError('Independent estimates disagree; no mapping started')
                estimates.append(pose)
            pose = estimates[-1]
            self.set_status('handoff', 'Stopping AMCL/map_server before SLAM; pose='+str(pose))
            self.stop_stack()
            self.wait(lambda: not {'amcl', 'map_server'}.intersection(self.get_node_names()), 10)
            self.wait(self.stationary_ready, 5)
            self.samples.pop('map', None)
            self.matcher = None
            mapping_started = self.now()
            self.launch(['ros2', 'launch', 'limo_auto_mapping', 'auto_mapping.launch.py',
                         'posegraph:='+str(graph), 'start_pose:='+json.dumps(pose),
                         'use_sim_time:='+sim, 'require_localization_ready:=true'])
            stable = StablePose(seconds=3., samples=3)
            end = time.monotonic()+self.p('mapping_ready_timeout_sec')
            last_report = 0.
            while time.monotonic() < end:
                self.spin()
                self.stationary()
                try:
                    scan = self.fresh('scan')
                    actual, stamp = self.transform('map', 'base_link', dynamic=True)
                    # AMCL이 미래로 발행한 이전 TF를 새 SLAM TF로 착각하지 않는다.
                    if stamp < mapping_started+1.5 or self.matcher is None:
                        raise ValueError('Waiting for new SLAM map/TF')
                    if not close_pose(actual, pose, .35, .30):
                        raise ValueError('SLAM start pose differs from accepted localization')
                    self.scan_matches(actual, scan)
                    if stable.observe(time.monotonic(), self.stamp(scan), actual, True):
                        break
                except Exception as exc:
                    stable.reset()
                    if time.monotonic()-last_report > 3:
                        self.set_status('handoff', str(exc))
                        last_report = time.monotonic()
            else:
                raise TimeoutError('SLAM handoff validation failed; no exploration allowed')
            self.set_status('ready', 'Localization checks passed. Inspect RViz, then send /auto_mapping goal.')
            # 이 프로세스가 사라지면 서버는 heartbeat 만료를 확인하고 세션을 중단한다.
            while True:
                self.readiness.publish(Bool(data=True))
                until = time.monotonic()+.5
                while time.monotonic() < until:
                    self.spin()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    code = 0
    try:
        node = ResumeMapping()
        def stop(*_):
            node.stopping = True
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        node.run()
    except InterruptedError:
        pass
    except Exception as exc:
        code = 1
        if node is not None:
            node.set_status('failed', str(exc))
        else:
            print(str(exc), flush=True)
    finally:
        if node is not None:
            if node.owns_stack:
                node.readiness.publish(Bool(data=False))
            try:
                node.stop_stack()
            except Exception as exc:
                node.get_logger().error(str(exc))
                code = 1
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if code:
        raise SystemExit(code)
