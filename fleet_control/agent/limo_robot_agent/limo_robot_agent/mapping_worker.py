"""Motor-locked ROS mapping sessions using SLAM Toolbox and Frontier candidates."""
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseArray
import json
import math
import re
from pathlib import Path
import subprocess
import threading
import time
import uuid

from .local_process import spawn_owned, stop_owned, write_json


def executable(package, name):
    path = Path(get_package_prefix(package)) / 'lib' / package / name
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


class MappingWorker:
    def __init__(self, robot, agent, state_dir):
        from slam_toolbox.srv import SerializePoseGraph

        self.robot, self.agent = robot, agent
        self.root = Path(state_dir).resolve() / 'mapping_worker'
        self.root.mkdir(parents=True, exist_ok=True)
        self.path = self.root / 'journal.json'
        self.state = {}
        self.status = 'IDLE'
        if self.path.exists():
            try:
                self.state = json.loads(self.path.read_text())
                if self.state.get('session_id') or self.state.get('pending'):
                    self.status = 'UNKNOWN'
            except (OSError, ValueError):
                self.status = 'UNKNOWN'
        self.processes = []
        self.candidates = None
        self.goal = None
        self.attempted = set()
        self.fence = 0.
        self.finished = threading.Event()
        self.sub = robot.create_subscription(PoseArray, '/fleet/frontier_candidates', self.on_candidates, 10)
        self.serializer = robot.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.serialize_request = SerializePoseGraph.Request
        self.slam = executable('slam_toolbox', 'async_slam_toolbox_node')
        self.frontier = executable('explore_lite', 'fleet_frontier_candidates')
        self.map_saver = executable('nav2_map_server', 'map_saver_cli')
        self.config = Path(get_package_share_directory('limo_capstone')) / 'config/integrated_slam_params.yaml'
        self.robot.frontier_allowed = self.frontier_allowed
        self.robot.fleet_goal_allowed = lambda: self.status == 'IDLE' and self.locked()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def snapshot(self):
        return {'status': self.status, 'session_id': self.state.get('session_id'),
                'processes_started': bool(self.processes), 'goal': dict(self.goal) if self.goal else None}

    def request(self, action, **fields):
        return self.agent.http.post('/agent/operation', dict(
            robot_id=self.agent.robot_id, agent_boot_id=self.agent.boot_id,
            kind='mapping', action=action, **fields), timeout=1)

    def locked(self):
        return self.robot.count_publishers('/cmd_vel') == 0 and self.robot.count_subscribers('/cmd_vel') == 0

    def frontier_allowed(self, session_id):
        return (self.status == 'MAPPING' and self.state.get('session_id') == session_id
                and time.monotonic() < self.fence and self.locked())

    def on_candidates(self, message):
        if message.header.frame_id == 'map':
            self.candidates = time.monotonic(), message

    @staticmethod
    def single_node(endpoints, name):
        return len(endpoints) == 1 and endpoints[0].node_name == name and endpoints[0].node_namespace == '/'

    def ready(self):
        return (len(self.processes) == 2 and all(p.poll() is None for p in self.processes)
                and self.locked() and self.robot.nav.server_is_ready()
                and (self.robot._age('map') or float('inf')) < 3
                and self.single_node(self.robot.get_publishers_info_by_topic('/map'), 'slam_toolbox')
                and self.single_node(self.robot.get_publishers_info_by_topic('/fleet/frontier_candidates'),
                                     'fleet_frontier_candidates')
                and self.candidates is not None and time.monotonic() - self.candidates[0] < 4)

    def launch(self, session):
        if (not self.locked() or self.robot.count_publishers('/map')
                or self.robot.count_publishers('/fleet/frontier_candidates')
                or self.serializer.service_is_ready()
                or any(name in ('explore_node', 'explore_lite', 'fleet_frontier_candidates', 'slam_toolbox')
                       for name, _ in self.robot.get_node_names_and_namespaces())):
            raise RuntimeError('mapping graph is occupied or motor path is unlocked')
        self.candidates = None
        use_sim_time = str(self.robot.get_parameter('use_sim_time').value).lower()
        specs = (
            ('slam', [self.slam, '--ros-args', '--params-file', self.config,
                      '-p', 'use_sim_time:=' + use_sim_time]),
            ('frontier', [self.frontier, '--ros-args', '-p', 'use_sim_time:=' + use_sim_time,
                          '-p', 'costmap_topic:=/map', '-p', 'costmap_updates_topic:=/map_updates',
                          '-p', 'robot_base_frame:=base_footprint']),
        )
        for name, cmd in specs:
            with (session / (name + '.log')).open('x') as log:
                self.processes.append(spawn_owned(cmd, stdin=subprocess.DEVNULL, stdout=log, stderr=log))

    def wait(self, fn, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if fn():
                return True
            time.sleep(.1)
        return False

    def stop_processes(self):
        while self.processes:
            stop_owned(self.processes[-1])
            self.processes.pop()

    def offer(self):
        if not self.nav_idle():
            return
        if self.goal and self.goal['status'] not in ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'):
            return
        if not self.candidates or not self.ready():
            return
        for pose in self.candidates[1].poses:
            xy = (round(pose.position.x, 1), round(pose.position.y, 1))
            if not all(math.isfinite(x) for x in xy) or xy in self.attempted:
                continue
            self.attempted.add(xy)
            cid, sid = uuid.uuid4().hex, self.state['session_id']
            goal = {'command_id': cid, 'status': 'SUBMITTING', 'goal_uuid': None}
            previous = self.goal
            if previous:
                write_json(self.root / sid / ('goal-' + previous['command_id'] + '.json'), previous.copy())
            if self.request('reserve_goal', session_id=sid, goal_id=cid, previous=previous).get('reserved') is not True:
                return
            self.goal = goal
            write_json(self.root / sid / ('goal-' + cid + '.json'), goal)

            def ack(status, detail, progress=None):
                if status != 'EXECUTING':
                    goal.update(status=status, detail=detail, **(progress or {}))

            self.robot.start_frontier(dict(mission_id=sid, command_id=cid, expires_at=time.time()+3,
                                           payload={'goal': {'frame_id': 'map', 'x': xy[0], 'y': xy[1]}}), ack)
            return

    def halt_goal(self):
        goal = self.goal
        if goal is None:
            return self.nav_idle()
        if goal['status'] not in ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'):
            self.robot.cancel_frontier(dict(mission_id=self.state['session_id'],
                                            command_id=uuid.uuid4().hex), lambda *args: None)
        return self.wait(lambda: self.nav_idle() and (
            goal['status'] == 'REJECTED' and goal['goal_uuid'] is None
            or goal['status'] in ('SUCCEEDED', 'FAILED', 'CANCELED') and goal['goal_uuid']
            and self.robot.follow_path_idle()), 5)

    def nav_idle(self):
        nav = self.robot._nav2_goals()
        return nav['publishers'] == 1 and nav['active_count'] == 0 and self.robot.active is None

    def save_map(self, directory):
        directory.mkdir()
        prefix = directory / 'map'
        evidence = dict(yaml_ok=False, pgm_ok=False, posegraph_ok=False)
        try:
            with (directory / 'raster.log').open('x') as log:
                # Nav2 writes unknown pixels as 205/255. Keep the free threshold
                # below their occupancy (50/255) so Map Server reloads them as unknown.
                child = spawn_owned([self.map_saver, '-f', prefix, '--fmt', 'pgm', '--free', '0.196', '--ros-args',
                                     '-p', 'map_subscribe_transient_local:=true'], stdout=log, stderr=log)
                try:
                    saved = child.wait(timeout=7) == 0
                except subprocess.TimeoutExpired:
                    stop_owned(child, grace=1)
                    saved = False
            for ext in ('yaml', 'pgm'):
                file = prefix.with_suffix('.' + ext)
                evidence[ext + '_ok'] = saved and file.is_file() and file.stat().st_size > 0
        except OSError:
            # Raster errors leave its evidence false and still allow a graph save.
            pass
        try:
            if self.serializer.service_is_ready():
                req = self.serialize_request()
                req.filename = str(directory / 'graph')
                future = self.serializer.call_async(req)
                if not self.wait(future.done, 6):
                    future.cancel()
                    return 'UNKNOWN', evidence
                response = future.result()
                evidence['posegraph_ok'] = response.result == 0 and all(
                    (directory / ('graph.' + ext)).is_file() and (directory / ('graph.' + ext)).stat().st_size
                    for ext in ('posegraph', 'data'))
        except Exception:
            # A failed response can leave a partial posegraph after the raster succeeded.
            return 'UNKNOWN', evidence
        return ('OK' if all(evidence.values()) else 'FAILED'), evidence

    def operate(self, action, session, cid):
        if action == 'START':
            self.status = 'STARTING'
            self.goal = None
            self.attempted.clear()
            session.mkdir()
            nav = self.robot._nav2_goals()
            if self.robot.active is not None or nav['publishers'] not in (0, 1) or nav['active_count'] not in (None, 0):
                return 'UNKNOWN', {}, 'Nav2 slot unavailable'
            self.launch(session)
        if action in ('START', 'RESUME'):
            ready = self.wait(lambda: self.ready() and self.nav_idle() and self.agent.connected, 13)
            if ready:
                self.status = 'MAPPING'
            return ('OK' if ready else 'UNKNOWN'), dict(slam_running=ready, frontier_running=ready), (
                'SLAM map and Frontier candidates observed' if ready else 'ROS readiness timed out')
        if action == 'SAVE':
            outcome, evidence = self.save_map(session / cid)
            return outcome, evidence, 'raster and posegraph saved independently'
        self.status = 'PAUSING' if action == 'PAUSE' else 'STOPPING'
        terminal = self.halt_goal()
        if action == 'STOP':
            self.stop_processes()
        self.status = ('PAUSED' if action == 'PAUSE' else 'IDLE') if terminal else 'UNKNOWN'
        return ('OK' if terminal else 'UNKNOWN'), dict(
            goal_terminal=bool(terminal), frontier_paused=bool(terminal),
            frontier_stopped=action == 'STOP' and not self.processes), (
            'owned goal and controller terminal or never accepted' if terminal else 'goal or controller outcome unresolved')

    def execute(self, c):
        sid, cid, action = c.get('session_id'), c.get('command_id'), c.get('action')
        if (self.status == 'UNKNOWN' or self.state.get('pending') or c.get('kind') != 'mapping'
                or c.get('robot_id') != self.agent.robot_id or c.get('expires_at', 0) <= time.time()
                or type(c.get('epoch')) is not int
                or not all(isinstance(x, str) and re.fullmatch('[0-9a-f]{32}', x) for x in (sid, cid))):
            return
        expected = {'START': ('IDLE',), 'RESUME': ('PAUSED',), 'SAVE': ('PAUSED',),
                    'PAUSE': ('MAPPING',), 'STOP': ('MAPPING', 'PAUSED')}
        if self.status not in expected.get(action, ()):
            return
        if action != 'START' and (sid, c['epoch']) != (self.state.get('session_id'), self.state.get('epoch')):
            return
        ref = dict(session_id=sid, command_id=cid)
        if self.request('accept', **ref).get('accepted') is not True:
            return
        self.state.update(session_id=sid, epoch=c['epoch'], pending=cid)
        write_json(self.path, self.state)
        session = self.root / sid
        try:
            outcome, evidence, detail = self.operate(action, session, cid)
            if self.goal:
                write_json(session / ('goal-' + self.goal['command_id'] + '.json'), self.goal.copy())
        except Exception as exc:
            outcome, evidence, detail = 'UNKNOWN', {}, type(exc).__name__ + ': ' + str(exc)
        # Keep the local pending fence until Fleet confirms the durable result.
        self.state.update(last_command=cid, last_outcome=outcome, evidence=evidence)
        write_json(self.path, self.state)
        result = self.request('result', **ref, outcome=outcome, evidence=evidence, detail=detail)
        expected_result = 'COMPLETED' if outcome == 'OK' else 'FAILED'
        if outcome == 'UNKNOWN' or result.get('command', {}).get('status') != expected_result:
            raise RuntimeError('reconciliation required')
        self.state['pending'] = None
        if action == 'STOP':
            self.state['session_id'] = None
        write_json(self.path, self.state)
        self.fence = time.monotonic() + 3

    def lose_session(self):
        self.status = 'UNKNOWN'
        terminal = self.halt_goal()
        self.stop_processes()
        self.state.update(last_outcome='UNKNOWN', goal_terminal=bool(terminal))
        write_json(self.path, self.state)

    def run(self):
        try:
            while not self.finished.wait(.15):
                if self.status == 'UNKNOWN':
                    continue
                try:
                    reply = self.request('poll', session_id=self.state.get('session_id')) if self.agent.connected else {}
                    if reply.get('renew') is True:
                        self.fence = time.monotonic() + 3
                    if reply.get('command'):
                        self.execute(reply['command'])
                except Exception:
                    if self.state.get('pending'):
                        self.lose_session()
                if self.status in ('MAPPING', 'PAUSED'):
                    if (time.monotonic() >= self.fence or not self.locked()
                            or any(p.poll() is not None for p in self.processes)
                            or (self.goal and self.goal['status'] == 'RECONCILIATION_REQUIRED')):
                        self.lose_session()
                    elif self.status == 'MAPPING':
                        try:
                            self.offer()
                        except Exception:
                            self.lose_session()
        finally:
            if self.processes:
                self.lose_session()

    def close(self):
        self.finished.set()
        self.thread.join(timeout=19)
