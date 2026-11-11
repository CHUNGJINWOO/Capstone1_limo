#!/usr/bin/env python3
"""고정된 실제 맵핑 코드 + 관측 전용 Gazebo 센서로 실패를 재시험한다."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid
import queue
import threading
from datetime import datetime
from zoneinfo import ZoneInfo

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from ros_gz_interfaces.msg import Contacts
from limo_mapping_interfaces.action import AutoMapping
from action_msgs.msg import GoalStatusArray
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
import yaml

from audit_metrics import (center_cost, contact_episodes, trajectory_metrics,
                           map_point_to_world, box_cost_summary)

EXP = Path(__file__).resolve().parents[1]
ROOT = EXP.parents[1]
OUT = ROOT / 'notebook/2026_11_05/06_실패시험_계측교정/runs'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stamp_ns(stamp):
    return stamp.sec*1_000_000_000 + stamp.nanosec


def check_provenance():
    import limo_auto_mapping.server as server
    import limo_auto_mapping.explorer as explorer
    source = ROOT / 'notebook/2026_11_05/04_저속_진단_목표기록/src/limo_auto_mapping'
    expected = EXP / 'ws/src/limo_auto_mapping'
    prefix = subprocess.check_output(['ros2', 'pkg', 'prefix', 'limo_auto_mapping'], text=True).strip()
    if Path(prefix).resolve() != (EXP / 'ws/install/limo_auto_mapping').resolve():
        raise RuntimeError('실행 패키지 prefix 불일치: '+prefix)
    hashes = {}
    for p in source.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:
            rel = p.relative_to(source)
            if digest(p) != digest(expected/rel):
                raise RuntimeError('고정 소스 변경: '+str(rel))
            hashes[str(rel)] = digest(p)
    for mod in (server, explorer):
        rel = Path('limo_auto_mapping') / Path(mod.__file__).name
        if digest(Path(mod.__file__)) != hashes[str(rel)]:
            raise RuntimeError('실제 import와 기준 해시 불일치')
    return {'prefix': prefix, 'server_import': server.__file__, 'explorer_import': explorer.__file__,
            'source_sha256': hashes}


class OwnedProcesses:
    """이 실행에서 만든 프로세스 그룹만 종료한다. killall/pkill을 사용하지 않는다."""
    def __init__(self, directory):
        self.directory = directory
        self.entries = []
        self.commands = []

    def start(self, name, args):
        fd = (self.directory / (name+'.log')).open('w')
        proc = subprocess.Popen(args, stdout=fd, stderr=subprocess.STDOUT, start_new_session=True)
        self.entries.append((name, proc, fd))
        self.commands.append({'name': name, 'argv': args, 'pid': proc.pid})
        return proc

    def stop(self):
        for _, proc, _ in reversed(self.entries):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic()+12
        for _, proc, _ in reversed(self.entries):
            try:
                proc.wait(timeout=max(.1, deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=5)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait(timeout=3)
                    except ProcessLookupError:
                        pass
        results = [{'name': name, 'exit_code': proc.poll()} for name, proc, _ in self.entries]
        for _, _, fd in self.entries:
            fd.close()
        return results

    def pose_stream(self, monitor):
        """Pose_V 바깥 header의 원본 stamp를 보존한다. TF 변환으로 유실하지 않는다."""
        args = ['ign', 'topic', '-e', '--json-output', '-t', '/world/mc3f_world/pose/info']
        fd = (self.directory/'world_pose_capture.stderr.log').open('w')
        proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=fd, text=True,
                                start_new_session=True, bufsize=1)
        self.entries.append(('world_pose_capture', proc, fd))
        self.commands.append({'name': 'world_pose_capture', 'argv': args, 'pid': proc.pid})

        def receive():
            decoder = json.JSONDecoder()
            buffer = ''
            try:
                for line in proc.stdout:
                    buffer += line
                    while buffer.strip():
                        buffer = buffer.lstrip()
                        try:
                            message, end = decoder.raw_decode(buffer)
                        except json.JSONDecodeError:
                            break
                        buffer = buffer[end:]
                        stamp = message.get('header', {}).get('stamp', {})
                        ns = int(stamp.get('sec', 0))*1_000_000_000+int(stamp.get('nsec', 0))
                        for pose in message.get('pose', []):
                            if pose.get('name') not in ('limo', 'audit_dynamic_box'):
                                continue
                            position = pose.get('position', {})
                            q = pose.get('orientation', {})
                            yaw = math.atan2(2*(q.get('w', 1)*q.get('z', 0)+q.get('x', 0)*q.get('y', 0)),
                                             1-2*(q.get('y', 0)**2+q.get('z', 0)**2))
                            monitor.world_queue.put({'stamp_ns': ns, 'x': position.get('x', 0),
                                                     'y': position.get('y', 0), 'z': position.get('z', 0),
                                                     'yaw': yaw, 'parent': 'Gazebo world pose/info',
                                                     'child': pose['name'], 'timestamp_source': 'Pose_V.header.stamp'})
            except Exception as exc:
                monitor.world_queue.put({'capture_error': f'{type(exc).__name__}: {exc}'})

        threading.Thread(target=receive, daemon=True).start()


class Monitor(Node):
    def __init__(self, directory):
        super().__init__('failure_audit_monitor', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.stream = (directory/'telemetry.jsonl').open('w')
        self.world = []
        self.dynamic_world = []
        self.world_queue = queue.Queue()
        self.contact_samples = []
        self.contact_messages = 0
        self.contact_stamps = set()
        self.positive_keys = set()
        self.contact_pairs = set()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.grid = None
        self.scan_count = 0
        self.scan_properties = None
        self.nav_plans = 0
        self.latest_plan = None
        self.cmd_count = 0
        self.latest_cmd = (0., 0.)
        self.max_cmd_x = self.max_cmd_yaw = 0.
        self.map_pose = None
        self.ground_pair_logged_at = {}
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Contacts, '/audit/contacts', self.on_contacts, 100)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_grid, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 50)
        self.create_subscription(NavPath, '/plan', self.on_plan, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 lambda m: self.record('nav_status', statuses=[s.status for s in m.status_list]),
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = ActionClient(self, AutoMapping, '/auto_mapping')

    def record(self, kind, **data):
        self.stream.write(json.dumps({'wall_monotonic': time.monotonic(), 'kind': kind, **data},
                                     ensure_ascii=False, allow_nan=False)+'\n')
        self.stream.flush()

    def on_contacts(self, msg):
        key = stamp_ns(msg.header.stamp)
        self.contact_messages += 1
        self.contact_stamps.add(key)
        for c in msg.contacts:
            names = [c.collision1.name, c.collision2.name]
            self.contact_pairs.add(tuple(names))
            # 접촉 센서에서 받은 원시 collision 이름·깊이는 삭제하지 않고 보존한다.
            pair = sorted(names)
            if any('ground' in name.lower() for name in names):
                # 바닥-바퀴 접촉은 파이프라인 확인용으로 쌍별 0.5초에 한 번 기록한다.
                if key-self.ground_pair_logged_at.get(tuple(pair), -1_000_000_000) >= 500_000_000:
                    self.record('ground_contact_sample', stamp_ns=key, pair=pair)
                    self.ground_pair_logged_at[tuple(pair)] = key
                continue
            if not all(names):
                continue  # 이름을 모르면 대상 접촉으로 확정하지 않는다.
            unique = (key, *pair)
            if unique not in self.positive_keys:
                self.positive_keys.add(unique)
                self.record('contact_raw', stamp_ns=key, pair=pair,
                            entity_ids=[c.collision1.id, c.collision2.id], depths=list(c.depths))
                self.contact_samples.append({'stamp_ns': key, 'pair': pair})

    def on_grid(self, msg):
        self.grid = msg
        # 나중에 다른 위치·표면도 검사할 수 있도록 원시 costmap을 함께 보존한다.
        q = msg.info.origin.orientation
        self.record('global_costmap', stamp_ns=stamp_ns(msg.header.stamp),
                    frame=msg.header.frame_id, width=msg.info.width, height=msg.info.height,
                    resolution=msg.info.resolution,
                    origin=[msg.info.origin.position.x, msg.info.origin.position.y,
                            math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))],
                    data=list(msg.data))
        try:
            tf = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
            p = tf.transform.translation
            q = tf.transform.rotation
            self.map_pose = (p.x, p.y)
            self.record('center_cost', stamp_ns=stamp_ns(msg.header.stamp),
                        robot_map_xy=list(self.map_pose), value=center_cost(msg, self.map_pose),
                        frame=msg.header.frame_id,
                        robot_map_yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                        tf_stamp_ns=stamp_ns(tf.header.stamp))
        except Exception as exc:
            self.record('map_tf_unavailable', detail=str(exc))

    def on_scan(self, msg):
        self.scan_count += 1
        self.scan_properties = {'frame': msg.header.frame_id, 'range_min': msg.range_min,
                                'range_max': msg.range_max, 'angle_min': msg.angle_min,
                                'angle_max': msg.angle_max, 'beams': len(msg.ranges)}
        if self.scan_count % 10 == 1:
            valid = [x for x in msg.ranges if math.isfinite(x) and msg.range_min <= x <= msg.range_max]
            self.record('scan', stamp_ns=stamp_ns(msg.header.stamp),
                        valid_beams=len(valid), minimum_range_m=min(valid) if valid else None,
                        ranges=[x if math.isfinite(x) else None for x in msg.ranges])

    def on_cmd(self, msg):
        self.cmd_count += 1
        self.latest_cmd = (msg.linear.x, msg.angular.z)
        self.max_cmd_x = max(self.max_cmd_x, abs(msg.linear.x))
        self.max_cmd_yaw = max(self.max_cmd_yaw, abs(msg.angular.z))
        self.record('cmd_vel', linear_x=msg.linear.x, angular_z=msg.angular.z)

    def on_plan(self, msg):
        self.nav_plans += 1
        self.latest_plan = msg
        self.record('nav_plan', stamp_ns=stamp_ns(msg.header.stamp), frame=msg.header.frame_id,
                    points=[[p.pose.position.x, p.pose.position.y] for p in msg.poses])

    def spin(self, seconds=.1):
        rclpy.spin_once(self, timeout_sec=seconds)
        while not self.world_queue.empty():
            item = self.world_queue.get_nowait()
            if 'capture_error' in item:
                raise RuntimeError(item['capture_error'])
            if item['child'] == 'audit_dynamic_box':
                if not self.dynamic_world or item['stamp_ns'] > self.dynamic_world[-1]['stamp_ns']:
                    self.dynamic_world.append(item)
                    self.record('gazebo_dynamic_box_pose', **item)
                continue
            if self.world and item['stamp_ns'] <= self.world[-1]['stamp_ns']:
                continue
            self.world.append(item)
            self.record('gazebo_world_pose', **item)

    def wait(self, predicate, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.spin()
            if predicate():
                return True
        return bool(predicate())

    def parameters(self, node_name, names):
        client = self.create_client(GetParameters, node_name+'/get_parameters')
        try:
            if not self.wait(client.service_is_ready, 10):
                return {'error': 'service unavailable'}
            request = GetParameters.Request()
            request.names = names
            future = client.call_async(request)
            if not self.wait(future.done, 10):
                return {'error': 'parameter response timeout'}
            result = {}
            # 숫자 상수를 추측하지 않고 메시지의 자료형 상수를 사용한다.
            fields = {ParameterType.PARAMETER_DOUBLE: 'double_value',
                      ParameterType.PARAMETER_INTEGER: 'integer_value',
                      ParameterType.PARAMETER_STRING: 'string_value',
                      ParameterType.PARAMETER_BOOL: 'bool_value',
                      ParameterType.PARAMETER_DOUBLE_ARRAY: 'double_array_value'}
            for name, value in zip(names, future.result().values):
                field = fields.get(value.type)
                item = getattr(value, field) if field else None
                if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    item = list(item)
                result[name] = item
            return result
        finally:
            self.destroy_client(client)

    def blocker_on_plan(self):
        """로봇과 겹치지 않는 전방 경로점을 월드로 바꿔 시험 장애물을 배치한다."""
        if self.latest_plan is None or self.latest_plan.header.frame_id != 'map' or not self.world:
            return None
        try:
            tf = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        q, t = tf.transform.rotation, tf.transform.translation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        truth = self.world[-1]
        if (self.get_clock().now().nanoseconds-stamp_ns(self.latest_plan.header.stamp))/1e9 > 2:
            return None
        # 지나온 경로점에 장애물을 놓지 않는다. 현재 위치에 가장 가까운 점 이후만 사용한다.
        poses = list(self.latest_plan.poses)
        if not poses:
            return None
        nearest = min(range(len(poses)), key=lambda i: math.hypot(
            poses[i].pose.position.x-t.x, poses[i].pose.position.y-t.y))
        for pose in poses[nearest+1:]:
            point = (pose.pose.position.x, pose.pose.position.y)
            distance = math.hypot(point[0]-t.x, point[1]-t.y)
            if .65 <= distance <= 1.2:
                world = map_point_to_world(point, (t.x, t.y, yaw),
                                           (truth['x'], truth['y'], truth['yaw']))
                baseline = box_cost_summary(self.grid, world, (.5, .6),
                                            (t.x, t.y, yaw), (truth['x'], truth['y'], truth['yaw']))
                if baseline['inscribed_or_obstacle_cells']:
                    continue  # 이미 장애물이 있는 영역은 새 장애물 감지 시험에 쓰지 않는다.
                return {'plan_point_map': list(point), 'box_center_world': list(world),
                        'robot_map': [t.x, t.y, yaw], 'robot_world': truth,
                        'robot_center_distance_m': distance,
                        'baseline_box_region': baseline,
                        'registration': 'current robot map TF aligned to Gazebo world pose'}
        return None


def parse_events(path):
    events = []
    for line in path.read_text(errors='replace').splitlines():
        if 'EXPLORE_EVENT ' in line:
            try:
                events.append(json.loads(line.split('EXPLORE_EVENT ', 1)[1]))
            except json.JSONDecodeError:
                pass
    return events


def validate_instrumentation():
    """센서·pose 관측만 추가됐는지 XML 구조로 원본과 비교한다."""
    import xml.etree.ElementTree as ET
    original = ROOT/'experiments/failure_limits_verification/models/limo_ackermann_fixed.sdf'
    actual = EXP/'models/limo_instrumented.sdf'
    a, b = ET.parse(original).getroot(), ET.parse(actual).getroot()
    model = b.find('model')
    for link in model.findall('link'):
        for sensor in list(link.findall('sensor')):
            if sensor.attrib['name'].startswith('audit_contact_'):
                link.remove(sensor)
    for plugin in list(model.findall('plugin')):
        if plugin.attrib['name'].endswith('::PosePublisher'):
            model.remove(plugin)
    def normalized(element):
        return (element.tag, sorted(element.attrib.items()), (element.text or '').strip(),
                [normalized(child) for child in element])
    if normalized(a) != normalized(b):
        raise RuntimeError('관측 이외 로봇 모델 차이 발견')
    for path in (EXP/'worlds').glob('world_scenario_*.sdf'):
        baseline = ET.parse(ROOT/'experiments/failure_limits_verification/worlds'/path.name).getroot()
        observed = ET.parse(path).getroot()
        world = observed.find('world')
        for plugin in list(world.findall('plugin')):
            if plugin.attrib['name'].endswith('::Contact'):
                world.remove(plugin)
        if normalized(baseline) != normalized(observed):
            raise RuntimeError('Contact system 이외 월드 변경 발견: '+path.name)


def run(args):
    validate_instrumentation()
    provenance = check_provenance()
    run_id = datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S')+'_'+args.scenario+'_'+uuid.uuid4().hex[:6]
    directory = OUT/run_id
    directory.mkdir(parents=True)
    shutil.copy2(__file__, directory/'runner_snapshot.py')
    shutil.copy2(EXP/'scripts/audit_metrics.py', directory/'metrics_snapshot.py')
    specs = {
        'calibration': ('calibration.sdf', (0., -.6, .05, math.pi/2)),
        'A': ('world_scenario_a_glass.sdf', (0., -1., .05, 1.5708)),
        'B1': ('world_scenario_b_desk.sdf', (0., -1.2, .05, 1.5708)),
        'B2': ('world_scenario_b_desk.sdf', (0., -.4, .05, 1.5708)),
        'C': ('world_scenario_c_dense.sdf', (0., 0., .05, 0.)),
        'D': ('world_scenario_d_dynamic.sdf', (-2.2, 0., .05, 0.)),
    }
    world_name, spawn = specs[args.scenario]
    world, model = EXP/'worlds'/world_name, EXP/'models/limo_instrumented.sdf'
    result = {'run_id': run_id, 'scenario': args.scenario, 'spawn_world_pose': spawn,
              'input_sha256': {'world': digest(world), 'model': digest(model),
                               'runner': digest(Path(__file__)), 'metrics': digest(EXP/'scripts/audit_metrics.py'),
                               'urdf': digest(EXP/'models/limo.urdf')},
              'provenance': provenance, 'external_limit_sec': args.seconds,
              'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
              'ign_partition': os.environ.get('IGN_PARTITION'),
              'action_accepted': None, 'action_result': None, 'tester_cancel': None,
              'dynamic_obstacle': {'created': False, 'removed': False},
              'dynamic_mode': args.dynamic_mode,
              'condition_created': 'not_established', 'failure_observations': [],
              'system_response': 'not_evaluated', 'error': None}
    print('RUN_DIRECTORY='+str(directory), flush=True)
    owned = OwnedProcesses(directory)
    monitor = None
    mapping = None
    goal = None
    final_future = None
    rclpy.init()
    try:
        monitor = Monitor(directory)
        owned.pose_stream(monitor)
        owned.start('gazebo', ['ign', 'gazebo', '-r', '-s', '-v', '3', str(world)])
        robot_params = directory/'robot_state_params.yaml'
        robot_params.write_text(yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {
            'robot_description': (EXP/'models/limo.urdf').read_text(), 'use_sim_time': True}}}))
        owned.start('robot_state', ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                                    '--ros-args', '--params-file', str(robot_params)])
        bridge_args = [
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/audit/contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/model/limo/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/world/mc3f_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V']
        owned.start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', *bridge_args])
        owned.start('spawn', ['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'mc3f_world',
                              '-file', str(model), '-name', 'limo', '-x', str(spawn[0]),
                              '-y', str(spawn[1]), '-z', str(spawn[2]), '-Y', str(spawn[3])])
        if not monitor.wait(lambda: bool(monitor.world), 45):
            raise RuntimeError('Gazebo world pose 미수신: 계측 준비 실패')
        initial = monitor.world[0]
        initial_error = math.hypot(initial['x']-spawn[0], initial['y']-spawn[1])
        result['initial_world_pose_error_m'] = initial_error
        if initial_error > .10:
            raise RuntimeError('월드 pose와 스폰 위치 불일치: 좌표계 확인 필요')
        monitor.record('world_pose_validated', position_error_m=initial_error)
        if args.scenario == 'calibration':
            result['condition_created'] = 'calibration_wall_and_world_pose'
            start = time.monotonic()
            last_command = float('-inf')
            while time.monotonic()-start < args.seconds:
                if time.monotonic()-last_command >= .1:
                    msg = Twist()
                    msg.linear.x = .08
                    monitor.cmd_pub.publish(msg)
                    last_command = time.monotonic()
                monitor.spin(.03)
            result['system_response'] = 'instrumentation_only_not_mapping'
        else:
            mapping = owned.start('mapping', ['ros2', 'launch', 'limo_auto_mapping',
                                               'auto_mapping.launch.py', 'nav2_profile:=slow', 'use_sim_time:=true'])
            if not monitor.wait(lambda: monitor.client.server_is_ready() and monitor.grid is not None
                                and monitor.scan_count > 0 and monitor.map_pose is not None, 60):
                raise RuntimeError('Action·costmap·scan·map TF 준비 시간 초과')
            result['initial_center_cost'] = center_cost(monitor.grid, monitor.map_pose)
            result['measured_parameters'] = {
                'controller': monitor.parameters('/controller_server', [
                    'FollowPath.max_vel_x', 'FollowPath.max_vel_theta', 'FollowPath.max_speed_xy',
                    'FollowPath.trans_stopped_velocity']),
                'smoother': monitor.parameters('/velocity_smoother', ['max_velocity', 'min_velocity']),
                'global_costmap': monitor.parameters('/global_costmap/global_costmap', [
                    'robot_radius', 'inflation_layer.inflation_radius'])}
            request = AutoMapping.Goal()
            request.output_prefix = str(directory/'map_output')
            request.allow_margin_retry = False
            request.max_duration_sec = 0.
            sent = monitor.client.send_goal_async(request, feedback_callback=lambda m: monitor.record(
                'mapping_feedback', phase=m.feedback.phase, retries=m.feedback.retry_count))
            if not monitor.wait(sent.done, 15):
                raise RuntimeError('AutoMapping 목표 수락 응답 미수신')
            goal = sent.result()
            result['action_accepted'] = goal.accepted
            if not goal.accepted:
                raise RuntimeError('AutoMapping 목표 거절')
            final_future = goal.get_result_async()
            result['condition_created'] = ('inscribed_start' if args.scenario == 'C'
                                            and result['initial_center_cost'] is not None
                                            and result['initial_center_cost'] >= 99 else
                                            'model_world_loaded' if args.scenario != 'D' else 'not_established')
            start = time.monotonic()
            trigger_start = monitor.world[-1]
            injected_at = None
            while time.monotonic()-start < args.seconds and not final_future.done():
                monitor.spin()
                # 실제 Gazebo 월드 이동을 확인한 뒤에만 동적 장애물을 생성한다.
                if args.scenario == 'D' and not result['dynamic_obstacle']['created']:
                    latest = monitor.world[-1]
                    travel = math.hypot(latest['x']-trigger_start['x'], latest['y']-trigger_start['y'])
                    placement = monitor.blocker_on_plan() if travel >= .35 else None
                    if placement is not None:
                        bx, by = placement['box_center_world']
                        sdf = f"<sdf version='1.6'><model name='audit_dynamic_box'><static>true</static><pose>{bx} {by} 0.45 0 0 0</pose><link name='link'><collision name='collision'><geometry><box><size>0.5 0.6 0.9</size></box></geometry></collision><visual name='visual'><geometry><box><size>0.5 0.6 0.9</size></box></geometry></visual></link></model></sdf>"
                        command = ['ign', 'service', '-s', '/world/mc3f_world/create', '--reqtype',
                                   'ignition.msgs.EntityFactory', '--reptype', 'ignition.msgs.Boolean',
                                   '--timeout', '3000', '--req', 'sdf: "'+sdf+'"']
                        response = subprocess.run(command, capture_output=True, text=True, timeout=8)
                        created = response.returncode == 0 and 'data: true' in response.stdout
                        result['dynamic_obstacle'].update(created=created, service_response=response.stdout,
                                                          created_after_world_travel_m=travel, placement=placement,
                                                          created_ros_stamp_ns=monitor.get_clock().now().nanoseconds)
                        monitor.record('dynamic_box_create_response', created=created,
                                       stdout=response.stdout, stderr=response.stderr)
                        # 경로가 실제로 막혔다는 판정은 별도 자료로 검토한다.
                        if created:
                            injected_at = time.monotonic()
                            result['condition_created'] = 'spawned_on_selected_plan_observation_pending'
                dynamic = result['dynamic_obstacle']
                if args.scenario == 'D' and dynamic['created']:
                    grid_stamp = stamp_ns(monitor.grid.header.stamp)
                    box_cost = center_cost(monitor.grid, dynamic['placement']['plan_point_map'])
                    if grid_stamp >= dynamic['created_ros_stamp_ns']:
                        dynamic['latest_box_cell_cost'] = box_cost
                        placement = dynamic['placement']
                        # 중심점뿐 아니라 표면이 관측되는 박스 영역 전체를 확인한다.
                        if grid_stamp != dynamic.get('region_checked_stamp_ns'):
                            # SLAM이 map→odom을 보정할 수 있어 생성 시 좌표 정합을 고정하지 않는다.
                            tf = monitor.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
                            t, q = tf.transform.translation, tf.transform.rotation
                            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                            truth = monitor.world[-1]
                            region = box_cost_summary(monitor.grid, placement['box_center_world'], (.5, .6),
                                                      (t.x, t.y, yaw),
                                                      (truth['x'], truth['y'], truth['yaw']))
                            dynamic['latest_region_registration'] = {
                                'robot_map': [t.x, t.y, yaw], 'robot_world': truth,
                                'tf_stamp_ns': stamp_ns(tf.header.stamp)}
                            dynamic['box_region'] = region
                            dynamic['region_checked_stamp_ns'] = grid_stamp
                            dynamic['maximum_observed_region_cost'] = max(
                                dynamic.get('maximum_observed_region_cost', -1),
                                region['maximum_cost'] if region['maximum_cost'] is not None else -1)
                        if (not dynamic['removed'] and dynamic.get('box_region', {}).get(
                                'inscribed_or_obstacle_cells', 0) > 0):
                            dynamic['observed_in_costmap'] = True
                            result['condition_created'] = 'blocker_on_selected_plan_observed_in_costmap'
                    if monitor.dynamic_world:
                        actual = monitor.dynamic_world[-1]
                        intended = dynamic['placement']['box_center_world']
                        dynamic['measured_world_pose'] = actual
                        dynamic['spawn_position_error_m'] = math.hypot(
                            actual['x']-intended[0], actual['y']-intended[1])
                    if (args.dynamic_mode == 'temporary' and not dynamic['removed']
                            and injected_at is not None and time.monotonic()-injected_at >= 10):
                        response = subprocess.run([
                            'ign', 'service', '-s', '/world/mc3f_world/remove', '--reqtype',
                            'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean',
                            '--timeout', '3000', '--req', 'name: "audit_dynamic_box", type: 2'],
                            capture_output=True, text=True, timeout=8)
                        dynamic['removed'] = response.returncode == 0 and 'data: true' in response.stdout
                        dynamic['remove_response'] = response.stdout
                        monitor.record('dynamic_box_remove_response', success=dynamic['removed'],
                                       stdout=response.stdout, stderr=response.stderr)
            result['monitor_elapsed_sec'] = time.monotonic()-start
            if not final_future.done():
                cancel = goal.cancel_goal_async()
                acknowledged = monitor.wait(cancel.done, 10)
                result['tester_cancel'] = {'request_completed': acknowledged,
                                           'accepted_goals': len(cancel.result().goals_canceling)
                                           if acknowledged else None}
                # 취소 수락과 최종 결과는 다른 응답이다. 결과까지 별도로 기다린다.
                result['terminal_after_cancel_received'] = monitor.wait(final_future.done, 25)
            if final_future.done():
                terminal = final_future.result()
                result['action_result'] = {'status': terminal.status, 'success': terminal.result.success,
                                           'exploration_complete': terminal.result.exploration_complete,
                                           'message': terminal.result.message,
                                           'margin_retried': terminal.result.margin_retried,
                                           'posegraph_prefix': terminal.result.posegraph_prefix}
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
        print('RUN_ERROR='+result['error'], flush=True)
    finally:
        if monitor is not None:
            if goal is not None and goal.accepted and final_future is not None and not final_future.done():
                cancel = goal.cancel_goal_async()
                monitor.wait(cancel.done, 5)
                monitor.wait(final_future.done, 15)
            # 교정 주행과 종료 시에만 정지 명령을 보낸다. 맵핑 의사결정에는 개입하지 않는다.
            for _ in range(3):
                monitor.cmd_pub.publish(Twist())
                monitor.spin(.05)
        result['process_exit'] = owned.stop()
        result['commands'] = owned.commands
        if monitor is not None:
            result['world_trajectory'] = trajectory_metrics(monitor.world)
            result['contact_messages_received'] = monitor.contact_messages
            result['contact_unique_timestamps'] = len(monitor.contact_stamps)
            result['contact_pairs_seen'] = [list(p) for p in sorted(monitor.contact_pairs)]
            result['obstacle_contact_samples'] = len(monitor.contact_samples)
            result['obstacle_contact_episodes'] = contact_episodes(monitor.contact_samples)
            result['contact_status'] = ('measured' if monitor.contact_messages else 'not_measured')
            result['wheel_slip'] = 'not_measured'
            result['scan_properties'] = monitor.scan_properties
            result['velocity_commands'] = {'samples': monitor.cmd_count,
                                           'maximum_abs_linear_x': monitor.max_cmd_x,
                                           'maximum_abs_angular_z': monitor.max_cmd_yaw}
            if monitor.contact_samples:
                result['failure_observations'].append('gazebo_reported_non_ground_contact')
            monitor.stream.close()
            monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        result['explorer_events'] = parse_events(directory/'mapping.log') if mapping else []
        result['saved_files'] = {ext: (directory/('map_output'+ext)).exists()
                                 for ext in ('.yaml', '.pgm', '.posegraph', '.data')}
        result['completed_at_kst'] = datetime.now(ZoneInfo('Asia/Seoul')).isoformat()
        (directory/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
        print('RUN_RESULT='+str(directory/'result.json'), flush=True)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=['calibration', 'A', 'B1', 'B2', 'C', 'D'], required=True)
    parser.add_argument('--seconds', type=float, default=120.)
    parser.add_argument('--dynamic-mode', choices=['persistent', 'temporary'], default='persistent')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('seconds must be finite and positive')
    # 전역 테스트 도메인과 Gazebo transport partition을 격리한다.
    if os.environ.get('ROS_DOMAIN_ID') != '97' or not os.environ.get('IGN_PARTITION', '').startswith('limo_audit_'):
        parser.error('Set ROS_DOMAIN_ID=97 and unique IGN_PARTITION=limo_audit_... before execution')
    run(args)
