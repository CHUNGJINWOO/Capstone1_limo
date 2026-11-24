"""Comprehensive Agent-side verification suite for Safety Context v1.

Runs cleanly with or without ROS 2 (`rclpy`) installed:
- If `rclpy` is unavailable in the test environment, lightweight ROS stubs are
  injected into `sys.modules` ONLY for the test process so real `RosRobot`,
  `AgentCore`, and `safety_protocol` code paths can be exercised end-to-end.
- No production code fabricates Gate/Operation/Map/Localization producers.
"""
import json
import os
import sys
import tempfile
import time
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'agent', 'limo_robot_agent'))


def _ensure_ros_stubs():
    # P2 (R1.4.1): this suite is written against the stub Node/ActionClient (robot.nav.sent_goals etc.), so the stubs are installed
    # even when a real rclpy exists. Whatever was in sys.modules is saved and restored in tearDownModule, together with any
    # limo_robot_agent modules that were imported against the real packages before this module loaded.
    rclpy = types.ModuleType('rclpy')
    rclpy_node = types.ModuleType('rclpy.node')
    rclpy_action = types.ModuleType('rclpy.action')
    rclpy_qos = types.ModuleType('rclpy.qos')

    class _GoalStatus:
        STATUS_ACCEPTED = 1
        STATUS_EXECUTING = 2
        STATUS_CANCELING = 3
        STATUS_SUCCEEDED = 4
        STATUS_CANCELED = 5
        STATUS_ABORTED = 6

    class _GoalStatusArray:
        def __init__(self, status_list=None):
            self.status_list = status_list or []

    class _QoSProfile:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    class _Durability:
        TRANSIENT_LOCAL = 1

    class _Reliability:
        RELIABLE = 1

    class _Logger:
        def info(self, msg): pass
        def warning(self, msg): pass
        def error(self, msg): pass

    class _Stamp:
        def __init__(self):
            self.sec = 0
            self.nanosec = 0

    class _Clock:
        def now(self):
            ns = time.time_ns()
            s = _Stamp()
            s.sec = ns // 1_000_000_000
            s.nanosec = ns % 1_000_000_000
            obj = types.SimpleNamespace(to_msg=lambda: s)
            return obj

    class _PubInfo:
        def __init__(self, node_name):
            self.node_name = node_name

    class _StubActionClient:
        def __init__(self, node, action_type, action_name):
            self.node = node
            self.action_type = action_type
            self.action_name = action_name
            self.wait_calls = 0
            self.sent_goals = []
            self.wait_ready = True
            self.on_wait = None
            self.send_exc = None
            self.next_send_future = None

        def server_is_ready(self):
            return self.wait_ready

        def wait_for_server(self, timeout_sec=2.0):
            self.wait_calls += 1
            if callable(self.on_wait):
                self.on_wait()
            return self.wait_ready

        def send_goal_async(self, goal, feedback_callback=None, goal_uuid=None):
            if self.send_exc is not None:
                raise self.send_exc
            fut = self.next_send_future if self.next_send_future is not None else StubFuture()
            fut.requested_goal_uuid = bytes(goal_uuid.uuid) if goal_uuid is not None else None
            self.sent_goals.append((goal, feedback_callback, fut))
            return fut

    class _StubNode:
        def __init__(self, name):
            self.node_name = name
            self.subs = {}
            self.timers = []
            self.params = {'use_sim_time': False}
            self.topic_publishers = {
                '/cmd_vel': [], '/limo_driver/cmd_vel': [],
                '/cmd_vel_locked': ['stall_detector'],
            }
            self.status_publishers = 1
            self.action_clients = []
            self._logger = _Logger()
            self._clock = _Clock()

        def create_subscription(self, msg_type, topic, cb, qos):
            self.subs[topic] = cb
            return cb

        def create_timer(self, period, cb, **kwargs):          # ros_adapter passes clock=...
            self.timers.append((period, cb))
            return cb

        def create_publisher(self, msg_type, topic, qos):
            # Models rclpy.Node.create_publisher; publications are recorded per topic for assertions.
            node = self
            node.__dict__.setdefault('published', {})

            class _Pub:
                def publish(self, msg):
                    node.published.setdefault(topic, []).append(msg)
            return _Pub()

        def count_publishers(self, topic):
            if topic == 'navigate_to_pose/_action/status':
                return self.status_publishers
            return len(self.topic_publishers.get(topic, []))

        def get_publishers_info_by_topic(self, topic):
            if topic not in self.topic_publishers:
                raise RuntimeError(f'graph error for {topic}')
            return [_PubInfo(n) for n in self.topic_publishers[topic]]

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self.params.get(name, False))

        def get_clock(self):
            return self._clock

        def get_logger(self):
            return self._logger

    def _action_client_factory(node, action_type, action_name):
        c = _StubActionClient(node, action_type, action_name)
        node.action_clients.append(c)
        return c

    class _NavigateToPose:
        class Goal:
            def __init__(self):
                self.pose = types.SimpleNamespace(
                    header=types.SimpleNamespace(frame_id='', stamp=None),
                    pose=types.SimpleNamespace(
                        position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                )

    rclpy_clock = types.ModuleType('rclpy.clock')          # ros_adapter imports Clock/ClockType (added after this suite was written)
    rclpy_clock.Clock = lambda **kwargs: types.SimpleNamespace(kwargs=kwargs)
    rclpy_clock.ClockType = types.SimpleNamespace(STEADY_TIME=1)
    rclpy_node.Node = _StubNode
    rclpy_action.ActionClient = _action_client_factory
    rclpy_qos.DurabilityPolicy = _Durability
    rclpy_qos.QoSProfile = _QoSProfile
    rclpy_qos.ReliabilityPolicy = _Reliability
    rclpy_qos.qos_profile_sensor_data = _QoSProfile()

    action_msgs = types.ModuleType('action_msgs')
    action_msgs_msg = types.ModuleType('action_msgs.msg')
    action_msgs_msg.GoalStatus = _GoalStatus
    action_msgs_msg.GoalStatusArray = _GoalStatusArray

    nav2_msgs = types.ModuleType('nav2_msgs')
    nav2_msgs_action = types.ModuleType('nav2_msgs.action')
    nav2_msgs_action.NavigateToPose = _NavigateToPose

    nav_msgs = types.ModuleType('nav_msgs')
    nav_msgs_msg = types.ModuleType('nav_msgs.msg')
    nav_msgs_msg.OccupancyGrid = object
    nav_msgs_msg.Odometry = object

    sensor_msgs = types.ModuleType('sensor_msgs')
    sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
    sensor_msgs_msg.LaserScan = object

    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    std_msgs_msg.String = object

    tf2_ros = types.ModuleType('tf2_ros')
    tf2_ros.Buffer = lambda: types.SimpleNamespace(lookup_transform=lambda *a, **kw: (_ for _ in ()).throw(RuntimeError('no tf')))
    tf2_ros.TransformListener = lambda buf, node: None
    unique_identifier_msgs = types.ModuleType('unique_identifier_msgs')
    unique_identifier_msgs_msg = types.ModuleType('unique_identifier_msgs.msg')
    unique_identifier_msgs_msg.UUID = lambda uuid: types.SimpleNamespace(uuid=uuid)

    stubs = {
        'rclpy': rclpy,
        'rclpy.node': rclpy_node,
        'rclpy.clock': rclpy_clock,
        'rclpy.action': rclpy_action,
        'rclpy.qos': rclpy_qos,
        'action_msgs': action_msgs,
        'action_msgs.msg': action_msgs_msg,
        'nav2_msgs': nav2_msgs,
        'nav2_msgs.action': nav2_msgs_action,
        'nav_msgs': nav_msgs,
        'nav_msgs.msg': nav_msgs_msg,
        'sensor_msgs': sensor_msgs,
        'sensor_msgs.msg': sensor_msgs_msg,
        'std_msgs': std_msgs,
        'std_msgs.msg': std_msgs_msg,
        'tf2_ros': tf2_ros,
        'unique_identifier_msgs': unique_identifier_msgs,
        'unique_identifier_msgs.msg': unique_identifier_msgs_msg,
    }
    global _REAL_ROS_PRESENT
    try:
        import importlib.util
        _REAL_ROS_PRESENT = importlib.util.find_spec('rclpy') is not None and 'rclpy' not in sys.modules or (
            'rclpy' in sys.modules and hasattr(sys.modules['rclpy'], 'init'))
    except Exception:
        _REAL_ROS_PRESENT = False
    saved = {k: sys.modules.get(k) for k in stubs}
    for name in [m for m in sys.modules if m == 'limo_robot_agent' or m.startswith('limo_robot_agent.')]:
        saved[name] = sys.modules.pop(name)                    # re-import the Agent against the stubs
    sys.modules.update(stubs)
    return saved


def _restore_ros_stubs(saved):
    if not saved:
        return
    for k, prev in saved.items():
        if prev is None:
            sys.modules.pop(k, None)
        else:
            sys.modules[k] = prev


class StubFuture:
    def __init__(self):
        self._callbacks = []
        self._done = False
        self._result = None
        self._exc = None

    def add_done_callback(self, cb):
        self._callbacks.append(cb)
        if self._done:
            cb(self)

    def set_result(self, val):
        # An ActionServer's goal response refers to the UUID sent in the request.
        if getattr(self, 'requested_goal_uuid', None) is not None and hasattr(val, 'goal_id'):
            val.goal_id.uuid = self.requested_goal_uuid
        self._result = val
        self._done = True
        for cb in list(self._callbacks):
            cb(self)

    def set_exception(self, exc):
        self._exc = exc
        self._done = True
        for cb in list(self._callbacks):
            cb(self)

    def result(self):
        if self._exc is not None:
            raise self._exc
        return self._result


class StubGoalHandle:
    def __init__(self, uuid_hex='aa' * 16, accepted=True):
        raw = bytes.fromhex(uuid_hex)
        self.goal_id = types.SimpleNamespace(uuid=raw)
        self.accepted = accepted
        self.cancel_calls = 0
        self.cancel_exc = None
        self.cancel_future = StubFuture()
        self.result_future = StubFuture()

    def cancel_goal_async(self):
        self.cancel_calls += 1
        if self.cancel_exc is not None:
            raise self.cancel_exc
        return self.cancel_future

    def get_result_async(self):
        return self.result_future


_REAL_ROS_PRESENT = False
_SAVED_ROS_MODULES = _ensure_ros_stubs()

try:
    from action_msgs.msg import GoalStatus  # noqa: E402
    from limo_robot_agent import core, ros_adapter, safety_protocol  # noqa: E402
    import importlib, pkgutil, limo_robot_agent as _pkg                  # import every Agent module NOW, while the stubs exist: RosRobot imports
    for _m in pkgutil.iter_modules(_pkg.__path__):                       # some of them lazily, and other suites that share this fixture build
        try: importlib.import_module('limo_robot_agent.' + _m.name)      # robots after tearDownModule has removed the stubs
        except Exception: pass
except BaseException:
    _restore_ros_stubs(_SAVED_ROS_MODULES)
    _SAVED_ROS_MODULES = None
    raise


def tearDownModule():
    global _SAVED_ROS_MODULES
    if _SAVED_ROS_MODULES:
        if _REAL_ROS_PRESENT:
            # Modules imported while the stubs were installed keep references to the stub classes: drop them, then put back what was
            # there before (real rclpy and any earlier limo_robot_agent import), so a real-ROS test module in this process binds real classes.
            # (Other suites that share this stub fixture cannot run in the same process then: run them without ROS sourced.)
            for name in [m for m in sys.modules if m == 'limo_robot_agent' or m.startswith('limo_robot_agent.')]:
                sys.modules.pop(name, None)
        _restore_ros_stubs(_SAVED_ROS_MODULES)
    _SAVED_ROS_MODULES = None


def _make_gate_msg(
    state='NORMAL',
    boot_id='gate-boot-1',
    fault_epoch=1,
    fault_latched=False,
    cmd_blocked=False,
    sim_time=None,
):
    d = {
        'state': state,
        'boot_id': boot_id,
        'fault_epoch': fault_epoch,
        'fault_latched': fault_latched,
        'cmd_blocked': cmd_blocked,
    }
    if sim_time is not None:
        d['sim_time'] = sim_time
    return types.SimpleNamespace(data=json.dumps(d))


def _make_nav_status_msg(entries=()):
    status_list = []
    for uuid_hex, st in entries:
        gid = types.SimpleNamespace(uuid=bytes.fromhex(uuid_hex))
        status_list.append(types.SimpleNamespace(status=st, goal_info=types.SimpleNamespace(goal_id=gid)))
    return types.SimpleNamespace(status_list=status_list)


def _valid_safety_context(**overrides):
    now = time.time()
    sc = {
        'version': 1,
        'server_boot_id': 'srv-boot-1',
        'agent_boot_id': 'agt-boot-1',
        'gate_boot_id': 'gate-boot-1',
        'fault_epoch': 1,
        'operation_mode': 'NAVIGATION',
        'operation_revision': 3,
        'map_version_id': 'map-v5',
        'map_epoch': 2,
        'localization_status': 'CONFIRMED',
        'localization_epoch': 4,
        'lease_owner': 'op-alice',
        'lease_epoch': 7,
        'issued_at': now - 0.1,
        'expires_at': now + 10.0,
    }
    sc.update(overrides)
    return sc


def _make_cmd(cmd_id='cmd-1', mission_id='mis-1', seq=1, sc=None, issued_at=None, expires_at=None, **top):
    now = time.time()
    payload = {'goal': {'x': 1.0, 'y': 2.0, 'yaw_rad': 0.0, 'frame_id': 'map'}}
    if sc is not None:
        payload['safety_context'] = sc
    cmd = {
        'command_id': cmd_id,
        'mission_id': mission_id,
        'robot_id': 'robot-1',
        'owner_boot_id': 'agt-boot-1',
        'type': 'navigate_to_pose',
        'seq': seq,
        'issued_at': now - 0.1 if issued_at is None else issued_at,
        'expires_at': now + 10.0 if expires_at is None else expires_at,
        'payload': payload,
    }
    cmd.update(top)
    return cmd


def _make_ready_robot(allow_missions=True):
    robot = ros_adapter.RosRobot(
        allow_missions=allow_missions,
        mode='sim',
        simulated=True,
        robot_id='robot-1',
        agent_boot_id='agt-boot-1',
        cancel_result_timeout_s=0.3,
        disconnect_cancel_after_s=2.0,
    )
    now_w = time.time()
    robot._session_state = {
        'connected': True,
        'server_boot_id': 'srv-boot-1',
        'agent_boot_id': 'agt-boot-1',
        'last_state_age_s': 0.1,
        'safety_contract_version': 1,
        'protocol_version': 2,
        'lease': {'owner': 'op-alice', 'epoch': 7, 'expires_at': now_w + 30.0},
    }
    robot._producer_state = {
        'operation': {'operation_mode': 'NAVIGATION', 'operation_revision': 3},
        'map': {'map_version_id': 'map-v5', 'map_epoch': 2, 'map_verified': True},
        'localization': {'localization_status': 'CONFIRMED', 'localization_map_version_id': 'map-v5', 'localization_epoch': 4},
    }
    robot.execution_allowed = lambda: bool(robot._session_state.get('connected'))
    robot.session_fn = lambda: dict(robot._session_state)
    robot.context_fn = lambda: dict(robot._producer_state)
    # ROS stub fixture only: tests that exercise AgentCore replace this with
    # its real durable journal callback; no real Gate attestation is implied.
    robot.on_command_meta = lambda *args, **kwargs: None
    robot._set('gate', _make_gate_msg())
    robot._on_nav_status(_make_nav_status_msg([]))
    return robot


class TestSafetyContextV1(unittest.TestCase):
    def test_1_valid_safety_context_executes_mocked_nav2(self):
        robot = _make_ready_robot(allow_missions=True)
        meta_calls = []
        robot.on_command_meta = lambda cid, **kw: meta_calls.append((cid, kw))
        snap = robot.snapshot()
        self.assertTrue(snap['safety_gate']['movement_permitted'])
        self.assertEqual(snap['operation_mode'], 'NAVIGATION')
        self.assertEqual(snap['localization_status'], 'CONFIRMED')
        self.assertTrue(snap['map_verified'])

        acks = []
        cmd = _make_cmd(sc=_valid_safety_context())
        robot.start_navigate(cmd, lambda s, d='', pr=None: acks.append((s, d)))
        robot._run_tasks()

        self.assertEqual(robot.nav.wait_calls, 1)
        self.assertEqual(len(robot.nav.sent_goals), 1)
        _, _, send_fut = robot.nav.sent_goals[0]
        handle = StubGoalHandle(uuid_hex='11' * 16, accepted=True)
        send_fut.set_result(handle)
        self.assertEqual(acks[-1][0], 'ACCEPTED')
        self.assertTrue(meta_calls[0][1]['submission_intent'])
        self.assertIn(('cmd-1', {'goal_uuid': robot.active['goal_uuid']}), meta_calls)

        handle.result_future.set_result(types.SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
        self.assertEqual(acks[-1][0], 'SUCCEEDED')
        self.assertTrue(any(m.get('terminal_source') == 'nav2_action_result' and m.get('goal_uuid') == bytes(handle.goal_id.uuid).hex() for _, m in meta_calls))
        self.assertIsNone(robot.active)

    def test_2_invalid_and_bool_safety_context_rejected_before_wait_for_server(self):
        robot = _make_ready_robot(allow_missions=True)

        # Missing each of the 15 fields
        base = _valid_safety_context()
        for field in safety_protocol.REQUIRED_SAFETY_CONTEXT_FIELDS:
            bad = dict(base)
            bad.pop(field)
            acks = []
            robot.start_navigate(_make_cmd(cmd_id=f'miss-{field}', sc=bad), lambda s, d='', pr=None: acks.append((s, d)))
            robot._run_tasks()
            self.assertEqual(acks[-1][0], 'REJECTED', f'missing {field} should reject')
            self.assertIn('SAFETY_CONTEXT_INVALID', acks[-1][1])

        # Booleans must NEVER pass non-negative integer or numeric checks (True == 1 trap)
        for int_field in ('version', 'fault_epoch', 'operation_revision', 'map_epoch', 'localization_epoch', 'lease_epoch'):
            bad = _valid_safety_context(**{int_field: True})
            acks = []
            robot.start_navigate(_make_cmd(cmd_id=f'bool-{int_field}', sc=bad), lambda s, d='', pr=None: acks.append((s, d)))
            robot._run_tasks()
            self.assertEqual(acks[-1][0], 'REJECTED', f'bool in {int_field} must fail')

        # Negative integers, empty strings, inverted timestamps, wrong boot_id, expired context
        for bad_kwargs, expected_status in [
            ({'fault_epoch': -1}, 'REJECTED'),
            ({'operation_mode': '   '}, 'REJECTED'),
            ({'server_boot_id': 'wrong-srv'}, 'REJECTED'),
            ({'agent_boot_id': 'wrong-agt'}, 'REJECTED'),
            ({'expires_at': time.time() - 0.5, 'issued_at': time.time() - 5.0}, 'EXPIRED'),
            ({'expires_at': time.time() - 10.0, 'issued_at': time.time() - 5.0}, 'REJECTED'),
        ]:
            acks = []
            sc = _valid_safety_context(**bad_kwargs)
            robot.start_navigate(_make_cmd(sc=sc), lambda s, d='', pr=None: acks.append((s, d)))
            robot._run_tasks()
            self.assertEqual(acks[-1][0], expected_status)

        # None of those invalid commands should ever touch wait_for_server or send_goal_async
        self.assertEqual(robot.nav.wait_calls, 0)
        self.assertEqual(len(robot.nav.sent_goals), 0)

    def test_3_gate_faults_fixed2_contract_boot_epoch_and_sim_time_freshness(self):
        # 3a: fixed2 message omitting boot_id and fault_epoch MUST NOT be fabricated
        robot = _make_ready_robot(allow_missions=True)
        fixed2_msg = types.SimpleNamespace(data=json.dumps({'state': 'NORMAL', 'cmd_blocked': False}))
        robot._set('gate', fixed2_msg)
        snap = robot.snapshot()
        sg = snap['safety_gate']
        self.assertIsNone(sg['boot_id'])
        self.assertIsNone(sg['fault_epoch'])
        self.assertFalse(sg['contract_met'])
        self.assertIn('boot_id', sg['missing_fields'])
        self.assertIn('fault_epoch', sg['missing_fields'])
        self.assertFalse(sg['movement_permitted'])

        # 3b: active goal canceled on Gate fault (STALL_LATCHED) and boot/epoch change
        for fault_name, trigger_fn in [
            ('STALL_LATCHED', lambda r: r._set('gate', _make_gate_msg(state='STALL_LATCHED', fault_epoch=2, fault_latched=True, cmd_blocked=True))),
            ('BOOT_RESTART', lambda r: r._set('gate', _make_gate_msg(state='NORMAL', boot_id='gate-boot-2', fault_epoch=0))),
            ('EPOCH_BUMP', lambda r: r._set('gate', _make_gate_msg(state='NORMAL', fault_epoch=2))),
        ]:
            robot = _make_ready_robot(allow_missions=True)
            acks = []
            robot.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks.append((s, d)))
            robot._run_tasks()
            _, _, send_fut = robot.nav.sent_goals[0]
            handle = StubGoalHandle(uuid_hex='22' * 16, accepted=True)
            send_fut.set_result(handle)
            self.assertEqual(acks[-1][0], 'ACCEPTED')

            trigger_fn(robot)
            robot._run_tasks()
            self.assertEqual(handle.cancel_calls, 1, f'{fault_name} must cancel owned goal')
            handle.cancel_future.set_result(types.SimpleNamespace(return_code=0, goals_canceling=[handle]))
            handle.result_future.set_result(types.SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
            self.assertEqual(acks[-1][0], 'CANCELED')
            self.assertIsNone(robot.active)

        # 3c: fault_epoch regression flagged and blocked
        robot = _make_ready_robot(allow_missions=True)
        robot._set('gate', _make_gate_msg(fault_epoch=5))
        robot._set('gate', _make_gate_msg(fault_epoch=4))
        self.assertTrue(robot.snapshot()['safety_gate']['epoch_regressed'])
        self.assertFalse(robot.snapshot()['safety_gate']['movement_permitted'])

        # 3d: RX fresh while sim_time stalls -> source_fresh=False, movement blocked
        tracker = safety_protocol.GateObservationTracker()
        tracker.on_message({'state': 'NORMAL', 'boot_id': 'g1', 'fault_epoch': 1, 'fault_latched': False, 'cmd_blocked': False, 'sim_time': 10.0}, now_mono=100.0)
        tracker.on_message({'state': 'NORMAL', 'boot_id': 'g1', 'fault_epoch': 1, 'fault_latched': False, 'cmd_blocked': False, 'sim_time': 10.0}, now_mono=101.2)
        ev = tracker.evaluate(now_mono=101.3, max_rx_age_s=2.0, max_source_stall_s=1.0, expect_source_clock=True)
        self.assertTrue(ev['rx_fresh'])
        self.assertFalse(ev['source_fresh'])
        self.assertEqual(ev['status'], 'STALE')

    def test_4_stage2_aborts_if_gate_faults_or_context_expires_during_wait_for_server(self):
        # 4a: Gate transitions to STALL_LATCHED inside wait_for_server
        robot = _make_ready_robot(allow_missions=True)
        robot.nav.on_wait = lambda: robot._set('gate', _make_gate_msg(state='STALL_LATCHED', fault_epoch=2, fault_latched=True, cmd_blocked=True))
        acks = []
        robot.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks.append((s, d)))
        robot._run_tasks()
        self.assertEqual(robot.nav.wait_calls, 1)
        self.assertEqual(len(robot.nav.sent_goals), 0)
        self.assertEqual(acks[-1][0], 'REJECTED')
        self.assertIn('ABORTED_PRE_SEND', acks[-1][1])

        # 4b: safety_context expires inside wait_for_server
        robot = _make_ready_robot(allow_missions=True)
        acks = []
        sc = _valid_safety_context(expires_at=time.time() + 0.08)
        cmd = _make_cmd(sc=sc, expires_at=time.time() + 10.0)
        robot.nav.on_wait = lambda: time.sleep(0.12)
        robot.start_navigate(cmd, lambda s, d='', pr=None: acks.append((s, d)))
        robot._run_tasks()
        self.assertEqual(robot.nav.wait_calls, 1)
        self.assertEqual(len(robot.nav.sent_goals), 0)
        self.assertEqual(acks[-1][0], 'EXPIRED')
        self.assertIn('ABORTED_PRE_SEND', acks[-1][1])

    def test_5_allow_missions_false_and_normal_gate_without_producers(self):
        # 5a: allow_missions=False always keeps movement_permitted=False even if all else looks valid
        robot = _make_ready_robot(allow_missions=False)
        self.assertFalse(robot.snapshot()['safety_gate']['movement_permitted'])
        self.assertIn('MISSIONS_DISABLED', robot.snapshot()['safety_gate']['movement_block_reasons'])
        acks = []
        robot.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks.append((s, d)))
        robot._run_tasks()
        self.assertEqual(acks[-1][0], 'REJECTED')
        self.assertIn('MISSIONS_DISABLED', acks[-1][1])

        # 5b: Default RosRobot (context_fn=None) with NORMAL gate & allow_missions=True still stays blocked
        robot = ros_adapter.RosRobot(allow_missions=True, robot_id='robot-1', agent_boot_id='agt-boot-1')
        robot._set('gate', _make_gate_msg(state='NORMAL'))
        robot._on_nav_status(_make_nav_status_msg([]))
        robot.execution_allowed = lambda: True
        robot.session_fn = lambda: {
            'connected': True,
            'server_boot_id': 'srv-boot-1',
            'agent_boot_id': 'agt-boot-1',
            'last_state_age_s': 0.1,
            'safety_contract_version': 1,
            'protocol_version': 2,
            'lease': {'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + 30.0},
        }
        snap = robot.snapshot()
        self.assertEqual(snap['operation_mode'], 'UNKNOWN')
        self.assertFalse(snap['map_verified'])
        self.assertEqual(snap['localization_status'], 'UNKNOWN')
        self.assertFalse(snap['safety_gate']['movement_permitted'])
        acks = []
        robot.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks.append((s, d)))
        robot._run_tasks()
        self.assertEqual(acks[-1][0], 'REJECTED')
        self.assertEqual(len(robot.nav.sent_goals), 0)

        # 5c: localization_status='READY' is NOT equivalent to 'CONFIRMED'
        robot = _make_ready_robot(allow_missions=True)
        robot._producer_state['localization']['localization_status'] = 'READY'
        self.assertFalse(robot.snapshot()['safety_gate']['movement_permitted'])
        self.assertIn('LOCALIZATION_UNCONFIRMED', robot.snapshot()['safety_gate']['movement_block_reasons'])

    def test_6_server_disconnect_and_lease_loss(self):
        # 6a: lease loss while goal active -> cancel
        robot = _make_ready_robot(allow_missions=True)
        acks = []
        robot.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks.append((s, d)))
        robot._run_tasks()
        _, _, send_fut = robot.nav.sent_goals[0]
        handle = StubGoalHandle(uuid_hex='33' * 16, accepted=True)
        send_fut.set_result(handle)

        robot._session_state['lease'] = None
        robot._run_tasks()
        self.assertEqual(handle.cancel_calls, 1)
        self.assertEqual(robot.active['fault_cancel_reason'], 'LEASE_REVOKED_OR_EXPIRED')

        # 6b: server disconnect while goal active -> cancel
        robot = _make_ready_robot(allow_missions=True)
        acks = []
        robot.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks.append((s, d)))
        robot._run_tasks()
        _, _, send_fut = robot.nav.sent_goals[0]
        handle2 = StubGoalHandle(uuid_hex='44' * 16, accepted=True)
        send_fut.set_result(handle2)

        robot._session_state['connected'] = False
        robot._run_tasks()
        self.assertEqual(handle2.cancel_calls, 1)
        self.assertEqual(robot.active['fault_cancel_reason'], 'SERVER_DISCONNECTED_OR_RESTARTED')

    def test_7_lost_cancel_result_marks_reconciliation_required_and_blocks(self):
        robot = _make_ready_robot(allow_missions=True)
        robot.cancel_result_timeout_s = 0.05
        acks = []
        robot.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks.append((s, d)))
        robot._run_tasks()
        _, _, send_fut = robot.nav.sent_goals[0]
        handle = StubGoalHandle(uuid_hex='55' * 16, accepted=True)
        send_fut.set_result(handle)

        # Trigger Gate fault; Nav2 never returns terminal result within timeout
        robot._set('gate', _make_gate_msg(state='STALL_LATCHED', fault_epoch=2, fault_latched=True, cmd_blocked=True))
        robot._run_tasks()
        self.assertEqual(handle.cancel_calls, 1)
        time.sleep(0.07)
        robot._run_tasks()
        self.assertEqual(acks[-1][0], 'RECONCILIATION_REQUIRED')
        self.assertIsNotNone(robot.active)  # Still blocks any new goal submission

        # Subsequent navigate must be refused while unresolved active remains
        robot._set('gate', _make_gate_msg(state='NORMAL', fault_epoch=2))
        acks2 = []
        robot.start_navigate(_make_cmd(cmd_id='cmd-2', mission_id='mis-2', seq=2, sc=_valid_safety_context(fault_epoch=2)), lambda s, d='', pr=None: acks2.append((s, d)))
        robot._run_tasks()
        self.assertEqual(acks2[-1][0], 'REJECTED')
        self.assertEqual(len(robot.nav.sent_goals), 1)

    def test_8_agent_restart_durable_reconciliation_and_agent_core_loop(self):
        with tempfile.TemporaryDirectory() as d:
            # Persist an in-progress command as from a previous boot
            jpath = os.path.join(d, 'robot-1_commands.json')
            with open(jpath, 'w') as f:
                json.dump({
                    'version': 2,
                    'highest_nav_seq': 5,
                    'commands': {
                        'old-cmd': {'status': 'ACCEPTED', 'detail': 'goal accepted', 'goal_uuid': '66' * 16, 'mission_id': 'm-old', 'type': 'navigate_to_pose', 'seq': 5},
                    },
                }, f)
            posted_hellos = []
            received_acks = []

            class DummyHandler:
                def __init__(self):
                    self.started = []
                def start_navigate(self, cmd, ack):
                    self.started.append(cmd)
                def cancel_mission(self, cmd, ack):
                    pass

            handler = DummyHandler()
            agent = core.AgentCore(
                'robot-1', 'http://127.0.0.1:1', 'tok',
                snapshot_fn=lambda: {'execution': {'active': None, 'nav2_goals': {'known': True, 'active_count': 0}}},
                handler=handler,
                agent_info={'agent_version': '0.1.0', 'boot_id': 'new-boot-2'},
                state_dir=d,
                require_safety_context=True,
            )
            # Restart converts unfinished command immediately on load and persists
            self.assertEqual(agent.seen['old-cmd']['status'], 'RECONCILIATION_REQUIRED')
            with open(jpath) as f:
                persisted = json.load(f)
            self.assertEqual(persisted['commands']['old-cmd']['status'], 'RECONCILIATION_REQUIRED')
            self.assertEqual(persisted['commands']['old-cmd']['goal_uuid'], '66' * 16)
            self.assertEqual(agent.agent_info['protocol_version'], 2)
            self.assertEqual(agent.agent_info['safety_contract_version'], 1)

            # Hook fake HTTP client to drive _hello and _on_command
            def fake_post(path, body, timeout=None):
                if path == '/agent/hello':
                    posted_hellos.append(body)
                    return {'server_boot_id': 'srv-boot-1', 'safety_contract_version': 1, 'protocol_version': 2}
                if path == '/agent/reconcile':
                    return {'commands': [], 'reconciliation_required': False}   # the real reconcile reply always carries this key (R1.4.4 reads it as authority)
                if '/ack' in path:
                    received_acks.append((path, body))
                    return {'ok': True}
                return {}

            agent.http = types.SimpleNamespace(post=fake_post)
            agent._hello()
            self.assertTrue(agent.connected)
            self.assertEqual(agent.server_safety_contract_version, 1)
            with agent.lock:   # R1.4.6: the lease a state reply would carry (motion commands need one at receipt)
                agent.lease = {'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + 60.0}

            # Command missing safety_context rejected by AgentCore when require_safety_context=True
            agent._on_command(_make_cmd(cmd_id='cmd-no-sc', seq=6, sc=None, owner_boot_id='new-boot-2'))
            self.assertEqual(len(handler.started), 0)
            self.assertEqual(agent.seen['cmd-no-sc']['status'], 'REJECTED')

            # Valid new command still blocked by unresolved old-cmd RECONCILIATION_REQUIRED
            sc_new = _valid_safety_context(agent_boot_id='new-boot-2')
            agent._on_command(_make_cmd(cmd_id='cmd-new', seq=7, sc=sc_new, owner_boot_id='new-boot-2'))
            self.assertEqual(len(handler.started), 0)
            self.assertEqual(agent.seen['cmd-new']['status'], 'REJECTED')
            self.assertIn('BLOCKED_BY_UNRESOLVED_COMMAND', agent.seen['cmd-new']['detail'])

    def test_9_explore_lite_foreign_goals_and_nav2_age_stagnation(self):
        # 9a: Foreign goal visible in GoalStatusArray while Agent owns no goal
        robot = _make_ready_robot(allow_missions=True)
        foreign_uuid = 'fe' * 16
        robot._on_nav_status(_make_nav_status_msg([(foreign_uuid, GoalStatus.STATUS_EXECUTING)]))
        snap = robot.snapshot()
        self.assertIsNone(snap['execution']['active'])
        self.assertEqual(snap['execution']['nav2_goals']['foreign_active_goal_uuids'], [foreign_uuid])
        self.assertFalse(snap['safety_gate']['movement_permitted'])
        self.assertIn('NAV2_FOREIGN_GOAL_ACTIVE', snap['safety_gate']['movement_block_reasons'])
        nav_acks = []
        robot.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: nav_acks.append((s, d)))
        robot._run_tasks()
        self.assertEqual(nav_acks[-1][0], 'REJECTED')
        self.assertIn('NAV2_NOT_IDLE', nav_acks[-1][1])

        # cancel_mission for another mission does NOT send cancel to Nav2
        cancel_acks = []
        robot.cancel_mission({'command_id': 'can-1', 'mission_id': 'explore-mis'}, lambda s, d='', pr=None: cancel_acks.append((s, d)))
        robot._run_tasks()
        self.assertEqual(cancel_acks[-1][0], 'RECONCILIATION_REQUIRED')
        self.assertIn('foreign Nav2 goal', cancel_acks[-1][1])

        # 9b: While Agent owns goal AA alongside foreign goal FE, fault cancels ONLY owned handle AA
        robot2 = _make_ready_robot(allow_missions=True)
        robot2._on_nav_status(_make_nav_status_msg([]))
        acks = []
        robot2.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks.append((s, d)))
        robot2._run_tasks()
        _, _, send_fut = robot2.nav.sent_goals[0]
        owned_handle = StubGoalHandle(uuid_hex='aa' * 16, accepted=True)
        send_fut.set_result(owned_handle)
        robot2._on_nav_status(_make_nav_status_msg([
            (robot2.active['goal_uuid'], GoalStatus.STATUS_EXECUTING),
            (foreign_uuid, GoalStatus.STATUS_EXECUTING),
        ]))
        self.assertEqual(robot2.snapshot()['execution']['nav2_goals']['foreign_active_goal_uuids'], [foreign_uuid])

        robot2._set('gate', _make_gate_msg(state='STALL_LATCHED', fault_epoch=2, fault_latched=True, cmd_blocked=True))
        robot2._run_tasks()
        self.assertEqual(owned_handle.cancel_calls, 1)

        # 9c: nav2_goals.age_s is NOT refreshed by publisher count when callbacks stop
        robot3 = _make_ready_robot(allow_missions=True)
        robot3.nav_status['t'] = time.monotonic() - 2.5
        robot3.status_publishers = 1
        snap3 = robot3.snapshot()
        self.assertGreaterEqual(snap3['execution']['nav2_goals']['age_s'], 2.4)
        self.assertFalse(snap3['safety_gate']['movement_permitted'])
        self.assertIn('NAV2_STATUS_STALE_OR_UNKNOWN', snap3['safety_gate']['movement_block_reasons'])

    def test_10_fault_between_stage2_and_send_goal_async_blocked(self):
        # 10a: Fault injected during DISPATCHED announcement (gap after Stage 2) MUST block send_goal_async
        for label, inject_fn in [
            ('gate_fault', lambda r: r._set('gate', _make_gate_msg(state='STALL_LATCHED', fault_epoch=2, fault_latched=True, cmd_blocked=True))),
            ('gate_boot_change', lambda r: r._set('gate', _make_gate_msg(boot_id='gate-boot-2', fault_epoch=0))),
            ('lease_revoked', lambda r: r._session_state.update({'lease': None})),
            ('server_disconnect', lambda r: r._session_state.update({'connected': False})),
        ]:
            with self.subTest(gap=label):
                robot = _make_ready_robot(allow_missions=True)
                orig_set_mission = robot._set_mission
                def hook(*args, **kwargs):
                    orig_set_mission(*args, **kwargs)
                    if len(args) > 2 and args[2] == 'DISPATCHED':
                        inject_fn(robot)
                robot._set_mission = hook
                acks = []
                robot.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks.append((s, d)))
                robot._run_tasks()
                self.assertEqual(len(robot.nav.sent_goals), 0, f'{label} must prevent send_goal_async')
                self.assertIsNone(robot.active)
                self.assertEqual(acks[-1][0], 'REJECTED')
                self.assertIn('ABORTED_PRE_SEND', acks[-1][1])

    def test_11_post_submit_and_delayed_goal_response_transitions(self):
        # 11a: Fault arrives immediately after send_goal_async while goal response is delayed -> cancels on acceptance
        robot = _make_ready_robot(allow_missions=True)
        acks = []
        robot.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks.append((s, d)))
        robot._run_tasks()
        self.assertEqual(len(robot.nav.sent_goals), 1)
        _, _, send_fut = robot.nav.sent_goals[0]
        self.assertIsNone(robot.active['handle'])

        robot._set('gate', _make_gate_msg(state='STALL_LATCHED', fault_epoch=2, fault_latched=True, cmd_blocked=True))
        robot._run_tasks()
        self.assertTrue(robot.active['cancel_requested'])
        self.assertEqual(robot.active['fault_cancel_reason'], 'GATE_FAULT:STALL_LATCHED')

        handle = StubGoalHandle(uuid_hex='77' * 16, accepted=True)
        send_fut.set_result(handle)
        self.assertEqual(handle.cancel_calls, 1)
        handle.result_future.set_result(types.SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
        self.assertEqual(acks[-1][0], 'CANCELED')
        self.assertIsNone(robot.active)

        # 11b: Delayed goal response rejected by Nav2 after post-submit fault cleans up cleanly
        robot_rej = _make_ready_robot(allow_missions=True)
        acks_rej = []
        robot_rej.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks_rej.append((s, d)))
        robot_rej._run_tasks()
        _, _, send_fut_rej = robot_rej.nav.sent_goals[0]
        robot_rej._set('gate', _make_gate_msg(state='STALL_LATCHED', fault_epoch=2, fault_latched=True, cmd_blocked=True))
        robot_rej._run_tasks()
        send_fut_rej.set_result(StubGoalHandle(uuid_hex='78' * 16, accepted=False))
        self.assertEqual(acks_rej[-1][0], 'REJECTED')
        self.assertIsNone(robot_rej.active)

        # 11c: Hung goal response exceeding goal_response_timeout_s marks RECONCILIATION_REQUIRED
        robot_hung = _make_ready_robot(allow_missions=True)
        robot_hung.goal_response_timeout_s = 0.04
        acks_hung = []
        robot_hung.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks_hung.append((s, d)))
        robot_hung._run_tasks()
        time.sleep(0.06)
        robot_hung._run_tasks()
        self.assertEqual(acks_hung[-1][0], 'RECONCILIATION_REQUIRED')
        self.assertIsNotNone(robot_hung.active)

    def test_12_fault_right_after_accept_and_cancel_accept_not_terminal(self):
        robot = _make_ready_robot(allow_missions=True)
        acks = []
        robot.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks.append((s, d)))
        robot._run_tasks()
        _, _, send_fut = robot.nav.sent_goals[0]
        handle = StubGoalHandle(uuid_hex='88' * 16, accepted=True)
        send_fut.set_result(handle)
        self.assertEqual(acks[-1][0], 'ACCEPTED')

        # Operator sends cancel_mission; Nav2 accepts cancel service request WITHOUT sending terminal result yet
        cancel_acks = []
        robot.cancel_mission({'command_id': 'can-op', 'mission_id': 'mis-1'}, lambda s, d='', pr=None: cancel_acks.append((s, d)))
        robot._run_tasks()
        self.assertEqual(handle.cancel_calls, 1)
        handle.cancel_future.set_result(types.SimpleNamespace(return_code=0, goals_canceling=[handle]))
        self.assertEqual(cancel_acks[-1][0], 'ACCEPTED')
        self.assertNotIn('CANCELED', [s for s, _ in acks])
        self.assertIsNotNone(robot.active)

        # Only terminal STATUS_CANCELED resolves both navigate and cancel commands
        handle.result_future.set_result(types.SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
        self.assertEqual(acks[-1][0], 'CANCELED')
        self.assertEqual(cancel_acks[-1][0], 'SUCCEEDED')
        self.assertIsNone(robot.active)

        # Gate fault -> lost cancel -> RECONCILIATION_REQUIRED; resetting Gate to NORMAL does NOT clear/replay
        robot2 = _make_ready_robot(allow_missions=True)
        robot2.cancel_result_timeout_s = 0.04
        acks2 = []
        robot2.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks2.append((s, d)))
        robot2._run_tasks()
        h2 = StubGoalHandle(uuid_hex='89' * 16, accepted=True)
        robot2.nav.sent_goals[0][2].set_result(h2)
        robot2._set('gate', _make_gate_msg(state='STALL_LATCHED', fault_epoch=2, fault_latched=True, cmd_blocked=True))
        robot2._run_tasks()
        time.sleep(0.06)
        robot2._run_tasks()
        self.assertEqual(acks2[-1][0], 'RECONCILIATION_REQUIRED')
        robot2._set('gate', _make_gate_msg(state='NORMAL', fault_epoch=2))
        robot2._run_tasks()
        self.assertIsNotNone(robot2.active)
        self.assertEqual(len(robot2.nav.sent_goals), 1)

    def test_13_multithreaded_gate_and_nav_status_callback_races(self):
        import threading
        robot = _make_ready_robot(allow_missions=True)
        stop = threading.Event()
        errors = []

        def gate_worker():
            i = 0
            while not stop.is_set():
                try:
                    robot._set('gate', _make_gate_msg(state='NORMAL', fault_epoch=1))
                    if i % 5 == 0:
                        robot._on_nav_status(_make_nav_status_msg([]))
                    i += 1
                except Exception as exc:
                    errors.append(exc)
                    return

        def snap_worker():
            while not stop.is_set():
                try:
                    robot.snapshot()
                except Exception as exc:
                    errors.append(exc)
                    return

        t1 = threading.Thread(target=gate_worker, daemon=True)
        t2 = threading.Thread(target=snap_worker, daemon=True)
        t1.start(); t2.start()
        try:
            for idx in range(10):
                acks = []
                robot.start_navigate(_make_cmd(cmd_id=f'race-{idx}', mission_id=f'm-{idx}', seq=idx + 1, sc=_valid_safety_context()), lambda s, d='', pr=None: acks.append((s, d)))
                robot._run_tasks()
                if robot.nav.sent_goals and robot.active is not None:
                    _, _, sf = robot.nav.sent_goals[-1]
                    if not sf._done:
                        h = StubGoalHandle(uuid_hex=f'{idx:02x}' * 16, accepted=True)
                        sf.set_result(h)
                        h.result_future.set_result(types.SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
                time.sleep(0.005)
        finally:
            stop.set()
            t1.join(timeout=2.0)
            t2.join(timeout=2.0)
        self.assertEqual(errors, [])

        # Concurrent cancel trigger from _monitor_active_goal + _on_goal_response issues cancel_goal_async exactly once
        robot_cancel = _make_ready_robot(allow_missions=True)
        acks_c = []
        robot_cancel.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda s, d='', pr=None: acks_c.append((s, d)))
        robot_cancel._run_tasks()
        h_c = StubGoalHandle(uuid_hex='99' * 16, accepted=True)
        robot_cancel._set('gate', _make_gate_msg(state='STALL_LATCHED', fault_epoch=2, fault_latched=True, cmd_blocked=True))
        robot_cancel._run_tasks()
        robot_cancel.nav.sent_goals[0][2].set_result(h_c)
        robot_cancel._run_tasks()
        self.assertEqual(h_c.cancel_calls, 1)

    def test_14_extreme_epochs_ids_and_timestamps_parity(self):
        # 14a: Boundary 2**63 - 1 and 256-char IDs allowed; >= 2**63, >256-char IDs, bools, negative/huge timestamps rejected safely
        max_ep = (1 << 63) - 1
        sc_ok, err_ok = safety_protocol.parse_safety_context(_valid_safety_context(
            fault_epoch=max_ep, operation_revision=max_ep, map_epoch=max_ep,
            localization_epoch=max_ep, lease_epoch=max_ep, gate_boot_id='g' * 256,
        ))
        self.assertIsNotNone(sc_ok, err_ok)

        for field, bad_val in [
            ('fault_epoch', 1 << 63),
            ('operation_revision', 1 << 63),
            ('map_epoch', 10 ** 400),
            ('localization_epoch', True),
            ('lease_epoch', -1),
            ('gate_boot_id', 'g' * 257),
            ('server_boot_id', '   '),
            ('issued_at', 0),
            ('issued_at', -5.0),
            ('issued_at', float('nan')),
            ('issued_at', float('inf')),
            ('issued_at', True),
            ('expires_at', 1e15),
            ('expires_at', 10 ** 400),
        ]:
            with self.subTest(field=field, bad_val=repr(bad_val)[:30]):
                sc, _ = safety_protocol.parse_safety_context(_valid_safety_context(**{field: bad_val}))
                self.assertIsNone(sc)

        # 14b: AgentCore._on_command survives corrupted command timestamps, seq >= 2**63, 10**400 OverflowError & corrupt journal
        with tempfile.TemporaryDirectory() as d:
            jpath = os.path.join(d, 'robot-1_commands.json')
            with open(jpath, 'w') as f:
                f.write('{corrupt-json')
            started = []
            handler = types.SimpleNamespace(start_navigate=lambda c, a: started.append(c), cancel_mission=lambda c, a: None)
            agent = core.AgentCore(
                'robot-1', 'http://127.0.0.1:1', 'tok',
                lambda: {'execution': {}}, handler, {'boot_id': 'agt-boot-1'}, d,
                require_safety_context=True, log=lambda *_: None,
            )
            self.assertFalse(agent.journal_ok)
            agent._on_command(_make_cmd(cmd_id='c-corrupt', seq=1, sc=_valid_safety_context()))
            self.assertEqual(len(started), 0)
            self.assertNotIn('c-corrupt', agent.seen)
            self.assertEqual(agent.pending_acks, [])
            with open(jpath) as saved:
                self.assertEqual(saved.read(), '{corrupt-json')

        with tempfile.TemporaryDirectory() as d2:
            started2 = []
            handler2 = types.SimpleNamespace(start_navigate=lambda c, a: started2.append(c), cancel_mission=lambda c, a: None)
            agent2 = core.AgentCore(
                'robot-1', 'http://127.0.0.1:1', 'tok',
                lambda: {'execution': {}}, handler2, {'boot_id': 'agt-boot-1'}, d2,
                require_safety_context=True, log=lambda *_: None,
            )
            agent2.server_boot = 'srv-boot-1'
            for bad_cmd in [
                _make_cmd(cmd_id='bad-seq-2e63', seq=1 << 63, sc=_valid_safety_context()),
                _make_cmd(cmd_id='bad-seq-bool', seq=True, sc=_valid_safety_context()),
                _make_cmd(cmd_id='bad-issued-ovf', seq=1, issued_at=10 ** 400, sc=_valid_safety_context()),
                _make_cmd(cmd_id='bad-exp-ovf', seq=2, expires_at=10 ** 400, sc=_valid_safety_context()),
                _make_cmd(cmd_id='bad-exp-nan', seq=3, expires_at=float('nan'), sc=_valid_safety_context()),
            ]:
                agent2._on_command(bad_cmd)
                self.assertEqual(agent2.seen[bad_cmd['command_id']]['status'], 'REJECTED')
            self.assertEqual(len(started2), 0)

        # 14c: Gate JSON with OverflowError sim_time, 2**63 fault_epoch, oversized boot_id does not crash agent
        robot = _make_ready_robot(allow_missions=True)
        robot._set('gate', types.SimpleNamespace(data=json.dumps({
            'state': 'NORMAL',
            'boot_id': 'b' * 257,
            'fault_epoch': 1 << 63,
            'fault_latched': False,
            'cmd_blocked': False,
            'sim_time': 10 ** 400,
        })))
        sg = robot.snapshot()['safety_gate']
        self.assertFalse(sg['contract_met'])
        self.assertFalse(sg['movement_permitted'])

    def test_15_late_goal_acceptance_after_timeout_preserves_reconciliation(self):
        # Reproduce Codex delayed_timeout defect with durable AgentCore journal + RosRobot:
        # goal_response_timeout -> RECONCILIATION_REQUIRED -> late GoalResponse.ACCEPT -> Feedback -> CancelResponse.ACCEPT
        robot = _make_ready_robot(allow_missions=True)
        robot.goal_response_timeout_s = 0.04
        robot.cancel_result_timeout_s = 5.0
        with tempfile.TemporaryDirectory() as d:
            agent = core.AgentCore(
                'robot-1', 'http://127.0.0.1:1', 'tok',
                robot.snapshot, robot, {'boot_id': 'agt-boot-1'}, d,
                require_safety_context=True, log=lambda *_: None,
            )
            agent.server_boot = 'srv-boot-1'
            agent.server_safety_contract_version = 1
            agent.server_protocol_version = 2; agent.server_protocol_boot = 'srv-boot-1'   # as a real hello of server boot srv-boot-1 would set
            agent.connected = True
            agent.server_reconciliation_required = False  # synthetic reconciled session
            agent.last_state_ok_mono = time.monotonic()
            agent.lease = {'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + 60.0}
            robot.session_fn = agent.session_snapshot
            robot.execution_allowed = agent.is_execution_allowed
            robot.on_command_meta = agent.record_command_meta

            cmd = _make_cmd(cmd_id='cmd-late-1', seq=1, sc=_valid_safety_context())
            agent._on_command(cmd)
            robot._run_tasks()
            self.assertEqual(len(robot.nav.sent_goals), 1)
            _, fb_cb, send_fut = robot.nav.sent_goals[0]

            # Step 1: goal_response_timeout triggers -> RECONCILIATION_REQUIRED in memory, disk, pending_acks, and last_mission
            time.sleep(0.06)
            robot._run_tasks()
            self.assertEqual(agent.seen['cmd-late-1']['status'], 'RECONCILIATION_REQUIRED')
            self.assertEqual(robot.last_mission['state'], 'RECONCILIATION_REQUIRED')
            self.assertEqual([a['status'] for a in agent.pending_acks if a['command_id'] == 'cmd-late-1'], ['RECONCILIATION_REQUIRED'])
            with open(agent.seen_path) as f:
                self.assertEqual(json.load(f)['commands']['cmd-late-1']['status'], 'RECONCILIATION_REQUIRED')

            # Step 2: Delayed GoalResponse.ACCEPT arrives -> MUST cancel runaway goal & store goal_uuid,
            # MUST NOT overwrite RECONCILIATION_REQUIRED with ACCEPTED anywhere
            late_handle = StubGoalHandle(uuid_hex='ab' * 16, accepted=True)
            send_fut.set_result(late_handle)
            self.assertEqual(late_handle.cancel_calls, 1)
            self.assertEqual(agent.seen['cmd-late-1']['status'], 'RECONCILIATION_REQUIRED')
            self.assertEqual(agent.seen['cmd-late-1'].get('goal_uuid'), bytes(late_handle.goal_id.uuid).hex())
            self.assertEqual(robot.last_mission['state'], 'RECONCILIATION_REQUIRED')
            self.assertEqual([a['status'] for a in agent.pending_acks if a['command_id'] == 'cmd-late-1'], ['RECONCILIATION_REQUIRED'])
            with open(agent.seen_path) as f:
                persisted = json.load(f)['commands']['cmd-late-1']
                self.assertEqual(persisted['status'], 'RECONCILIATION_REQUIRED')
                self.assertEqual(persisted.get('goal_uuid'), bytes(late_handle.goal_id.uuid).hex())

            # Step 3: Late Feedback arrives -> MUST NOT overwrite RECONCILIATION_REQUIRED with EXECUTING
            fb_msg = types.SimpleNamespace(
                goal_id=late_handle.goal_id,
                feedback=types.SimpleNamespace(
                    distance_remaining=1.5,
                    number_of_recoveries=0,
                    navigation_time=types.SimpleNamespace(sec=2, nanosec=0),
                ),
            )
            fb_cb(fb_msg)
            self.assertEqual(agent.seen['cmd-late-1']['status'], 'RECONCILIATION_REQUIRED')
            self.assertEqual(robot.last_mission['state'], 'RECONCILIATION_REQUIRED')

            # Step 4: Cancel service acceptance without terminal result MUST NOT clear RECONCILIATION_REQUIRED
            late_handle.cancel_future.set_result(types.SimpleNamespace(return_code=0, goals_canceling=[late_handle]))
            robot._run_tasks()
            self.assertEqual(agent.seen['cmd-late-1']['status'], 'RECONCILIATION_REQUIRED')
            self.assertIsNotNone(robot.active)
            self.assertIsNotNone(robot.snapshot()['execution']['active'])

            # Step 5: Gate reset to NORMAL does NOT clear block, and next movement command is REJECTED without send_goal_async
            robot._set('gate', _make_gate_msg(state='NORMAL', fault_epoch=1))
            cmd2 = _make_cmd(cmd_id='cmd-blocked-2', seq=2, sc=_valid_safety_context())
            agent._on_command(cmd2)
            robot._run_tasks()
            self.assertEqual(agent.seen['cmd-blocked-2']['status'], 'REJECTED')
            self.assertEqual(len(robot.nav.sent_goals), 1)

            # Step 6: Agent restart reloads durable RECONCILIATION_REQUIRED & goal_uuid
            restarted = core.AgentCore(
                'robot-1', 'http://127.0.0.1:1', 'tok',
                robot.snapshot, robot, {'boot_id': 'agt-boot-2'}, d,
                require_safety_context=True, log=lambda *_: None,
            )
            self.assertEqual(restarted.seen['cmd-late-1']['status'], 'RECONCILIATION_REQUIRED')
            self.assertEqual(restarted.seen['cmd-late-1'].get('goal_uuid'), bytes(late_handle.goal_id.uuid).hex())

    def test_16_reconciliation_races_out_of_order_and_duplicate_callbacks(self):
        # 16a: Direct AgentCore.ack invariant: RECONCILIATION_REQUIRED cannot be overwritten by non-terminal statuses
        with tempfile.TemporaryDirectory() as d:
            handler = types.SimpleNamespace(start_navigate=lambda c, a: None, cancel_mission=lambda c, a: None)
            agent = core.AgentCore(
                'robot-1', 'http://127.0.0.1:1', 'tok',
                lambda: {'execution': {}}, handler, {'boot_id': 'agt-boot-1'}, d,
                require_safety_context=True, log=lambda *_: None,
            )
            agent.ack('c1', 'RECONCILIATION_REQUIRED', 'initial timeout')
            for bad_st in ('RECEIVED', 'DISPATCHED', 'SENT', 'ACCEPTED', 'EXECUTING', 'CANCELING', 'RECONCILIATION_REQUIRED'):
                agent.ack('c1', bad_st, f'late {bad_st}')
                self.assertEqual(agent.seen['c1']['status'], 'RECONCILIATION_REQUIRED')
                self.assertEqual(agent.seen['c1']['detail'], 'initial timeout')
                self.assertEqual([a['status'] for a in agent.pending_acks if a['command_id'] == 'c1'], ['RECONCILIATION_REQUIRED'])

            # Terminal state cannot be overwritten by non-terminal or another terminal state
            agent.ack('c2', 'CANCELED', 'confirmed cancel')
            for st in ('ACCEPTED', 'EXECUTING', 'RECONCILIATION_REQUIRED', 'SUCCEEDED', 'FAILED'):
                agent.ack('c2', st, f'late {st}')
                self.assertEqual(agent.seen['c2']['status'], 'CANCELED')
                self.assertEqual(agent.seen['c2']['detail'], 'confirmed cancel')

        # 16b: Out-of-order _set_mission & timeout vs rejected goal_response + duplicate callbacks + UUID mismatch
        robot = _make_ready_robot(allow_missions=True)
        acks = []
        robot.start_navigate(_make_cmd(cmd_id='c-rej', mission_id='m-rej', sc=_valid_safety_context()), lambda s, d='', pr=None: acks.append((s, d)))
        robot._run_tasks()
        _, _, send_fut = robot.nav.sent_goals[0]
        send_fut.set_result(StubGoalHandle(uuid_hex='aa' * 16, accepted=False))
        # Simulate delayed _set_mission from a stale thread that had released lock earlier
        robot._set_mission('m-rej', 'c-rej', 'RECONCILIATION_REQUIRED', 'stale late timeout')
        self.assertEqual(robot.last_mission['state'], 'REJECTED')
        self.assertEqual(acks, [('REJECTED', 'Nav2 rejected the goal')])

        # Duplicate goal response & duplicate result & wrong goal_uuid ignored safely
        robot2 = _make_ready_robot(allow_missions=True)
        acks2 = []
        cancel_acks2 = []
        robot2.start_navigate(_make_cmd(cmd_id='c-dup', mission_id='m-dup', sc=_valid_safety_context()), lambda s, d='', pr=None: acks2.append((s, d)))
        robot2._run_tasks()
        _, _, sf2 = robot2.nav.sent_goals[0]
        rec2 = robot2.active
        h2 = StubGoalHandle(uuid_hex='bb' * 16, accepted=True)
        sf2.set_result(h2)
        robot2._on_goal_response(rec2, sf2)  # duplicate goal response callback
        self.assertEqual([s for s, _ in acks2], ['ACCEPTED'])

        # Wrong goal_uuid result ignored
        wrong_fut = StubFuture()
        wrong_fut.set_result(types.SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, goal_id=types.SimpleNamespace(uuid=bytes.fromhex('cc' * 16))))
        robot2._on_result(rec2, wrong_fut, expected_goal_uuid='cc' * 16)
        self.assertIs(robot2.active, rec2)
        self.assertEqual([s for s, _ in acks2], ['ACCEPTED'])

        # Cancel arrives, real terminal CANCELED arrives BEFORE late cancel rejection response
        robot2.cancel_mission({'command_id': 'can-ooo', 'mission_id': 'm-dup'}, lambda s, d='', pr=None: cancel_acks2.append((s, d)))
        robot2._run_tasks()
        h2.result_future.set_result(types.SimpleNamespace(status=GoalStatus.STATUS_CANCELED, goal_id=h2.goal_id))
        h2.cancel_future.set_result(types.SimpleNamespace(return_code=1, goals_canceling=[]))  # late cancel reject after terminal
        self.assertEqual([s for s, _ in acks2], ['ACCEPTED', 'CANCELED'])
        self.assertEqual([s for s, _ in cancel_acks2], ['ACCEPTED', 'SUCCEEDED'])
        self.assertIsNone(robot2.active)

        # 16c: Lease loss mid-cancel -> cancel timeout -> RECONCILIATION_REQUIRED; re-acquiring lease does NOT unblock
        robot3 = _make_ready_robot(allow_missions=True)
        robot3.cancel_result_timeout_s = 0.04
        acks3 = []
        lease_box = [{'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + 60.0}]
        robot3.session_fn = lambda: {
            'connected': True, 'server_boot_id': 'srv-boot-1', 'agent_boot_id': 'agt-boot-1',
            'last_state_age_s': 0.0, 'lease': lease_box[0], 'safety_contract_version': 1, 'protocol_version': 2,
        }
        robot3.start_navigate(_make_cmd(cmd_id='c-lease', mission_id='m-lease', sc=_valid_safety_context()), lambda s, d='', pr=None: acks3.append((s, d)))
        robot3._run_tasks()
        h3 = StubGoalHandle(uuid_hex='dd' * 16, accepted=True)
        robot3.nav.sent_goals[0][2].set_result(h3)
        lease_box[0] = {'owner': 'op-other', 'epoch': 8, 'expires_at': time.time() + 60.0}
        robot3._run_tasks()
        self.assertEqual(h3.cancel_calls, 1)
        time.sleep(0.06)
        robot3._run_tasks()
        self.assertEqual(acks3[-1][0], 'RECONCILIATION_REQUIRED')
        lease_box[0] = {'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + 60.0}
        robot3._run_tasks()
        self.assertIsNotNone(robot3.active)
        self.assertEqual(acks3[-1][0], 'RECONCILIATION_REQUIRED')

        # 16d: Late CONFIRMED terminal CANCELED after goal_response_timeout resolves RECONCILIATION_REQUIRED cleanly
        robot4 = _make_ready_robot(allow_missions=True)
        robot4.goal_response_timeout_s = 0.04
        with tempfile.TemporaryDirectory() as d4:
            agent4 = core.AgentCore(
                'robot-1', 'http://127.0.0.1:1', 'tok',
                robot4.snapshot, robot4, {'boot_id': 'agt-boot-1'}, d4,
                require_safety_context=True, log=lambda *_: None,
            )
            agent4.server_boot = 'srv-boot-1'
            agent4.server_safety_contract_version = 1
            agent4.server_protocol_version = 2; agent4.server_protocol_boot = 'srv-boot-1'   # as a real hello of server boot srv-boot-1 would set
            agent4.connected = True
            agent4.server_reconciliation_required = False  # synthetic reconciled session
            agent4.last_state_ok_mono = time.monotonic()
            agent4.lease = {'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + 60.0}
            robot4.session_fn = agent4.session_snapshot
            robot4.execution_allowed = agent4.is_execution_allowed
            robot4.on_command_meta = agent4.record_command_meta
            agent4._on_command(_make_cmd(cmd_id='c-fin', mission_id='m-fin', seq=1, sc=_valid_safety_context()))
            robot4._run_tasks()
            time.sleep(0.06)
            robot4._run_tasks()
            self.assertEqual(agent4.seen['c-fin']['status'], 'RECONCILIATION_REQUIRED')
            h4 = StubGoalHandle(uuid_hex='ee' * 16, accepted=True)
            robot4.nav.sent_goals[0][2].set_result(h4)
            self.assertEqual(agent4.seen['c-fin']['status'], 'RECONCILIATION_REQUIRED')
            h4.result_future.set_result(types.SimpleNamespace(status=GoalStatus.STATUS_CANCELED, goal_id=h4.goal_id))
            self.assertEqual(agent4.seen['c-fin']['status'], 'CANCELED')
            self.assertEqual(robot4.last_mission['state'], 'CANCELED')
            self.assertIsNone(robot4.active)

    def test_17_server_agent_reconciliation_alignment_under_late_accept(self):
        # Verify real Fleet server + AgentCore stay aligned on RECONCILIATION_REQUIRED under late Nav2 accept + duplicate delivery
        srv_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'server')
        if srv_root not in sys.path:
            sys.path.insert(0, srv_root)
        from fleet_server.db import DB
        from fleet_server.service import ApiError, Fleet

        class _LocalHttp:
            def __init__(self, fleet, robot_id):
                self.fleet = fleet
                self.robot_id = robot_id
                self.acked = []

            def post(self, path, body, timeout=None):
                if path == '/agent/hello':
                    return self.fleet.hello(body['robot_id'], body['agent'])
                if path == '/agent/reconcile':
                    return self.fleet.reconcile(body['robot_id'], body['agent_boot_id'], body['journal'], body['execution'], body.get('journal_ok', True))
                if path == '/agent/state':
                    return self.fleet.ingest_state(body['robot_id'], body['seq'], body['state'], agent_boot_id=body.get('agent_boot_id'))
                if path.startswith('/agent/commands/') and path.endswith('/ack'):
                    cid = path.split('/')[3]
                    self.acked.append((cid, body['status']))
                    return self.fleet.ack_command(body['robot_id'], cid, body['status'], body.get('detail', ''), body.get('progress'), agent_boot_id=body.get('agent_boot_id'))
                raise AssertionError(f'unexpected path {path}')

        with tempfile.TemporaryDirectory() as d:
            db = DB(os.path.join(d, 'fleet.db'))
            try:
                fleet = Fleet(db, types.SimpleNamespace(offline_after_s=10.0, command_ttl_s=120.0, resend_after_s=120.0))
                robot = _make_ready_robot(allow_missions=True)
                robot.goal_response_timeout_s = 0.04
                robot.cancel_result_timeout_s = 5.0
                agent = core.AgentCore(
                    'robot-1', 'http://local', 'tok',
                    robot.snapshot, robot, {'boot_id': 'agt-boot-1'}, d,
                    require_safety_context=True, log=lambda *_: None,
                )
                lhttp = _LocalHttp(fleet, 'robot-1')
                agent.http = lhttp
                robot.session_fn = agent.session_snapshot
                robot.execution_allowed = agent.is_execution_allowed
                robot.on_command_meta = agent.record_command_meta

                agent._hello()
                lease = fleet.lease_action('robot-1', types.SimpleNamespace(name='op-alice', role='operator'), 'acquire')
                resp1 = agent.http.post('/agent/state', {'robot_id': 'robot-1', 'agent_boot_id': 'agt-boot-1', 'seq': 1, 'state': robot.snapshot()})
                with agent.lock:
                    agent.last_state_ok_mono = time.monotonic()
                    agent.lease = resp1['lease'] or lease
                    agent.server_safety_contract_version = resp1['safety_contract_version']
                resp1b = agent.http.post('/agent/state', {'robot_id': 'robot-1', 'agent_boot_id': 'agt-boot-1', 'seq': 2, 'state': robot.snapshot()})
                with agent.lock:
                    agent.last_state_ok_mono = time.monotonic()
                    agent.lease = resp1b['lease'] or lease

                mission, created = fleet.create_mission('robot-1', 'navigate_to_pose', {'frame_id': 'map', 'x': 1.0, 'y': 2.0}, created_by='op-alice', lease_epoch=lease['epoch'])
                self.assertTrue(created)
                resp2 = agent.http.post('/agent/state', {'robot_id': 'robot-1', 'agent_boot_id': 'agt-boot-1', 'seq': 3, 'state': robot.snapshot()})
                for c in resp2['commands']:
                    agent._on_command(c)
                cid = resp2['commands'][0]['command_id']
                robot._run_tasks()

                time.sleep(0.06)
                robot._run_tasks()
                agent._flush_acks()
                self.assertEqual(fleet.mission(mission['mission_id'])['status'], 'RECONCILIATION_REQUIRED')

                # Late GoalResponse.ACCEPT + Feedback + duplicate command delivery MUST keep both Agent and Server in RECONCILIATION_REQUIRED
                h_late = StubGoalHandle(uuid_hex='ff' * 16, accepted=True)
                robot.nav.sent_goals[0][2].set_result(h_late)
                robot.nav.sent_goals[0][1](types.SimpleNamespace(
                    goal_id=h_late.goal_id,
                    feedback=types.SimpleNamespace(distance_remaining=2.0, number_of_recoveries=0, navigation_time=types.SimpleNamespace(sec=1, nanosec=0)),
                ))
                agent._on_command(resp2['commands'][0])
                agent._flush_acks()

                self.assertEqual(agent.seen[cid]['status'], 'RECONCILIATION_REQUIRED')
                self.assertEqual(fleet.mission(mission['mission_id'])['status'], 'RECONCILIATION_REQUIRED')
                self.assertTrue(all(st == 'RECONCILIATION_REQUIRED' for c, st in lhttp.acked if c == cid))
                with self.assertRaises(ApiError) as cm:
                    fleet.create_mission('robot-1', 'navigate_to_pose', {'frame_id': 'map', 'x': 3.0, 'y': 4.0}, created_by='op-alice', lease_epoch=lease['epoch'])
                self.assertEqual(cm.exception.status, 409)
            finally:
                db.c.close()


if __name__ == '__main__':
    unittest.main()
