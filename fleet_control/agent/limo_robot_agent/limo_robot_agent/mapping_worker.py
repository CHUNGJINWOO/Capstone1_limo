"""Motor-locked web mapping sessions: SLAM Toolbox + Frontier candidates + mapping-owned Nav2 goals.

Ported selectively from origin/main `mapping_worker.py` (ea2d963) onto the local RosRobot:
- the worker owns exactly the processes it spawned (`spawn_owned`, PDEATHSIG) and never stops others;
- START refuses an occupied graph (existing SLAM / explore / candidates node or `/map` publisher);
- each Frontier goal is reserved with Fleet before submission; PAUSE/STOP cancel only the owned goal and wait
  for its NavigateToPose terminal plus the FollowPath sub-action terminal;
- the local journal fences commands; a lost connection or lease (renew fence) makes the session UNKNOWN,
  halts the owned goal, stops owned processes and never restarts by itself;
- SAVE writes raster (`map_saver_cli --free 0.196`, keeps unknown 205 pixels) and posegraph independently,
  then, when the operator selected a registered floor, uploads the bundle for Map Registry registration.

Real-robot use is refused by agent_node (SIM/isolated only) until field safety conditions are cleared.
"""
import base64
import hashlib
import json
import math
import re
import subprocess
import threading
import time
import uuid
from pathlib import Path

from geometry_msgs.msg import PoseArray

from .local_process import spawn_owned, stop_owned, write_json
from . import frontier_policy as fp

GOAL_DONE = ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED')


def executable(package, name):
    from ament_index_python.packages import get_package_prefix
    path = Path(get_package_prefix(package)) / 'lib' / package / name
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


class MappingWorker:
    def __init__(self, robot, agent, state_dir, slam_params=None):
        from slam_toolbox.srv import SerializePoseGraph

        self.robot, self.agent = robot, agent
        self.root = Path(state_dir).resolve() / 'mapping_worker'
        self.root.mkdir(parents=True, exist_ok=True)
        self.path = self.root / 'journal.json'
        self.state = {}
        self.status = 'IDLE'
        self.detail = ''
        if self.path.exists():
            try:
                self.state = json.loads(self.path.read_text())
                if self.state.get('session_id') or self.state.get('pending'):
                    self.status = 'UNKNOWN'          # never resume or adopt an unfinished session after restart
                    self.detail = 'previous session unresolved at agent start'
            except (OSError, ValueError):
                self.status = 'UNKNOWN'
                self.detail = 'journal unreadable'
        self.processes = []
        self.candidates = None
        self.goal = None
        self.attempted = set()
        self.recorded = set()
        self.outcomes = []               # 2026-11-21: per-goal results (x, y, status, t, kind) for frontier_policy
        self.last_decision = None        # why the last offer() did / did not send a goal (operator-visible)
        self.decisions = []              # last 20 decisions (frontier / reposition / stop with skip reasons)
        self.repositions = 0
        self.fence = 0.
        self.plan = {}
        self.started_mono = None
        self.stop_reason = None          # why Frontier goals are no longer offered (limits), never a completion claim
        self.geofence_rejected = 0
        self.finished = threading.Event()
        self.sub = robot.create_subscription(PoseArray, '/fleet/frontier_candidates', self.on_candidates, 10)
        self.serializer = robot.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.serialize_request = SerializePoseGraph.Request
        self.tools_error = None
        try:
            self.slam = executable('slam_toolbox', 'async_slam_toolbox_node')
            self.frontier = executable('explore_lite', 'fleet_frontier_candidates')
            self.map_saver = executable('nav2_map_server', 'map_saver_cli')
        except Exception as exc:   # reported as START failure, never a silent fallback to `explore`
            self.tools_error = f'{type(exc).__name__}: {exc}'
        self.slam_params = slam_params
        if not self.slam_params:
            try:
                from ament_index_python.packages import get_package_share_directory
                self.slam_params = str(Path(get_package_share_directory('limo_capstone')) / 'config/integrated_slam_params.yaml')
            except Exception:
                self.slam_params = None
        robot.frontier_allowed = self.frontier_allowed
        robot.fleet_goal_allowed = lambda: self.status == 'IDLE'
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------ status
    def snapshot(self):
        return {'enabled': True, 'status': self.status, 'session_id': self.state.get('session_id'), 'detail': self.detail,
                'processes_started': bool(self.processes), 'goal': dict(self.goal) if self.goal else None,
                'frontier_candidates': len(self.candidates[1].poses) if self.candidates else None,
                'attempted_goals': len(self.attempted), 'tools_error': self.tools_error,
                'goal_outcomes': {k: sum(1 for o in self.outcomes if o['status'] == k) for k in ('SUCCEEDED',) + fp.DONE_FAIL},
                'repositions': self.repositions, 'last_decision': self.last_decision, 'recent_decisions': list(self.decisions[-20:]),
                'mode': self.plan.get('mode'), 'parent_map_version_id': self.plan.get('parent_map_version_id'),
                'geofence': bool(self.plan.get('geofence')), 'geofence_rejected': self.geofence_rejected,
                'limits': self.plan.get('limits'), 'stop_reason': self.stop_reason,
                'elapsed_s': round(time.monotonic() - self.started_mono, 1) if self.started_mono else None}

    def request(self, action, timeout=2.0, **fields):
        return self.agent.http.post('/agent/operation', dict(robot_id=self.agent.robot_id, agent_boot_id=self.agent.boot_id,
                                                             kind='mapping', action=action, **fields), timeout=timeout)

    def locked(self):
        """Motor path acceptable for mapping goals.

        LOCKED: no /cmd_vel publisher or subscriber (real robot / isolated graph, Gate output is /cmd_vel_locked).
        SIM_GATED: the existing simulation-only policy (`motor_policy.motor_path_allowed`): mode sim, simulated,
        allow_sim_safety_gate, use_sim_time and the stall gate as the ONLY /cmd_vel publisher; additionally at most one
        consumer (the simulator bridge). The Safety Gate is never bypassed or replaced."""
        if getattr(self.robot, 'require_motion_watchdog', False) and not self.robot.watchdog_ok():
            return False
        status, _ = self.robot._motor_path_status()
        if status == 'LOCKED':   # no /cmd_vel publisher AND the Gate is alive publishing /cmd_vel_locked
            return self.robot.count_subscribers('/cmd_vel') == 0
        return status == 'SIM_GATED' and self.robot.count_subscribers('/cmd_vel') <= 1

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
                and self.single_node(self.robot.get_publishers_info_by_topic('/fleet/frontier_candidates'), 'fleet_frontier_candidates')
                and self.candidates is not None and time.monotonic() - self.candidates[0] < 4)

    def nav_idle(self):
        nav = self.robot._nav2_goals()
        return (nav['publishers'] == 1 and nav['active_count'] == 0 and not nav.get('foreign_active_goal_uuids')
                and self.robot.active is None)

    # ------------------------------------------------------------ processes
    def launch(self, session):
        if self.tools_error:
            raise RuntimeError('mapping tools unavailable: ' + self.tools_error)
        if (not self.locked() or self.robot.count_publishers('/map') or self.robot.count_publishers('/fleet/frontier_candidates')
                or self.serializer.service_is_ready()
                or any(name in ('explore_node', 'explore', 'explore_lite', 'fleet_frontier_candidates', 'slam_toolbox')
                       for name, _ in self.robot.get_node_names_and_namespaces())):
            raise RuntimeError('mapping graph is occupied or motor path is unlocked')
        self.candidates = None
        sim = str(self.robot._use_sim_time()).lower()
        slam_cmd = [self.slam, '--ros-args', '-p', 'use_sim_time:=' + sim]
        if self.slam_params:
            slam_cmd[2:2] = ['--params-file', self.slam_params]
        specs = (
            ('slam', slam_cmd),
            ('frontier', [self.frontier, '--ros-args', '-p', 'use_sim_time:=' + sim, '-p', 'costmap_topic:=/map',
                          '-p', 'costmap_updates_topic:=/map_updates', '-p', 'robot_base_frame:=base_footprint']),
        )
        for name, cmd in specs:
            with (session / (name + '.log')).open('x') as log:
                self.processes.append(spawn_owned(cmd, stdin=subprocess.DEVNULL, stdout=log, stderr=log))
        write_json(session / 'processes.json', [{'pid': p.pid, 'cmd': [str(x) for x in c]} for p, (_, c) in zip(self.processes, specs)])

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

    # ------------------------------------------------------------ Frontier goals
    @staticmethod
    def inside(poly, x, y):
        inside = False
        for i in range(len(poly)):
            (xi, yi), (xj, yj) = poly[i], poly[i - 1]
            if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
                inside = not inside
        return inside

    def limit_reached(self):
        lim = self.plan.get('limits') or {}
        if self.started_mono and time.monotonic() - self.started_mono > lim.get('max_duration_s', 900):
            return 'MAX_DURATION'
        if len(self.attempted) >= lim.get('max_goals', 40):
            return 'MAX_GOALS'
        v = (getattr(self.robot, 'limo', None) or {}).get('voltage')
        if lim.get('min_battery_v') and (v is None or v < lim['min_battery_v']):
            return 'LOW_BATTERY' if v is not None else 'BATTERY_UNKNOWN'
        return None

    def offer(self):
        if not self.nav_idle():
            return
        if self.stop_reason is None:
            self.stop_reason = self.limit_reached()
        if self.stop_reason:
            return       # operator decides PAUSE / SAVE / STOP; the worker only stops offering goals
        if self.goal and self.goal['status'] not in GOAL_DONE:
            return
        if not self.candidates or not self.ready():
            return
        now = time.monotonic()
        if self.goal and self.goal['status'] in GOAL_DONE and self.goal['command_id'] not in self.recorded:
            self.recorded.add(self.goal['command_id'])
            self.outcomes.append({'x': self.goal['x'], 'y': self.goal['y'], 'status': self.goal['status'], 't': now, 'kind': self.goal.get('kind', 'frontier')})
        if fp.consecutive_failures(self.outcomes) >= fp.MAX_CONSECUTIVE_FAILS:
            self.stop_reason = 'GOALS_FAILING'        # probably blocked (Nav2 / Gate stall); not an exploration result
            self.last_decision = {'decision': 'STOP', 'reason': 'GOALS_FAILING', 'last': self.outcomes[-1]}
            self.decisions.append(self.last_decision); return
        robot_xy = self.robot_xy()
        fence = self.plan.get('geofence')
        cands = []
        for pose in self.candidates[1].poses:
            xy = (round(pose.position.x, 2), round(pose.position.y, 2))
            if fence and all(math.isfinite(v) for v in xy) and not self.inside(fence, *xy):
                self.geofence_rejected += 1
                continue
            cands.append(xy)
        xy, skipped = fp.pick_candidate(cands, robot_xy, self.outcomes, now)
        kind = 'frontier'
        if xy is None:
            grid, meta = self.grid()
            xy, score = (fp.pick_reposition(grid, meta, robot_xy, self.outcomes, now) if self.repositions < fp.MAX_REPOSITIONS else (None, 0))
            if xy is None:
                self.stop_reason = 'EXHAUSTED'        # no usable candidate and no informative reachable cell: stop offering
                self.last_decision = {'decision': 'STOP', 'reason': 'EXHAUSTED', 'candidates': len(cands), 'skipped': skipped,
                                      'repositions': self.repositions, 'robot': robot_xy}
                self.decisions.append(self.last_decision); return
            kind = 'reposition'; self.repositions += 1
        self.last_decision = {'decision': kind.upper(), 'goal': xy, 'candidates': len(cands), 'skipped': skipped,
                              'robot': None if robot_xy is None else (round(robot_xy[0], 2), round(robot_xy[1], 2))}
        self.decisions.append(self.last_decision); self.decisions = self.decisions[-20:]
        if True:                                 # (kept as a block: the previous for-loop body, now one decision per offer)
            self.attempted.add(xy)
            cid, sid = uuid.uuid4().hex, self.state['session_id']
            goal = {'command_id': cid, 'status': 'SUBMITTING', 'goal_uuid': None, 'x': xy[0], 'y': xy[1], 'kind': kind}
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

            self.robot.start_frontier(dict(mission_id=sid, command_id=cid, expires_at=time.time() + 3,
                                           payload={'goal': {'frame_id': 'map', 'x': xy[0], 'y': xy[1]}}), ack)
            return

    def robot_xy(self):
        """Robot position in the SLAM map frame (TF map->base_footprint), None if unavailable."""
        try:
            import rclpy
            tr = self.robot.tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return (tr.transform.translation.x, tr.transform.translation.y)
        except Exception:
            return None

    def grid(self):
        lock = getattr(self.robot, 'lock', None)
        try:
            if lock is not None:
                with lock: return getattr(self.robot, 'map_a', None), dict(getattr(self.robot, 'map_meta', None) or {})
            return getattr(self.robot, 'map_a', None), dict(getattr(self.robot, 'map_meta', None) or {})
        except Exception:
            return None, {}

    def halt_goal(self):
        goal = self.goal
        if goal is None:
            return self.nav_idle()
        if goal['status'] not in GOAL_DONE:
            self.robot.cancel_frontier(dict(mission_id=self.state['session_id'], command_id=uuid.uuid4().hex), lambda *a: None)
        # Terminal = our UUID ended (or was never accepted) and, for an accepted goal, the controller FollowPath ended too.
        return self.wait(lambda: self.nav_idle() and (
            goal['status'] == 'REJECTED' and self.robot.active is None
            or goal['status'] in ('SUCCEEDED', 'FAILED', 'CANCELED') and goal.get('goal_uuid') and self.follow_path_terminal()), 6)

    def follow_path_terminal(self):
        """FollowPath seen active then terminal, OR no FollowPath goal running now (unique controller_server, stable GID,
        no non-terminal status). The second form covers a goal canceled while still planning or in a recovery behaviour,
        when the controller never started; previously that left PAUSE/STOP without evidence (UNKNOWN)."""
        if self.robot.follow_path_idle():
            return True
        ev = self.robot.manual_idle_evidence() if hasattr(self.robot, 'manual_idle_evidence') else {}
        return bool((ev.get('follow_path') or {}).get('ok'))

    # ------------------------------------------------------------ SAVE
    def save_map(self, directory):
        directory.mkdir()
        prefix = directory / 'map'
        evidence = dict(yaml_ok=False, pgm_ok=False, posegraph_ok=False)
        try:
            with (directory / 'raster.log').open('x') as log:
                # Nav2 writes unknown as 205/255 (p=0.19608). free 0.196 keeps those pixels UNKNOWN on reload.
                child = spawn_owned([self.map_saver, '-f', prefix, '--fmt', 'pgm', '--free', '0.196', '--ros-args',
                                     '-p', 'map_subscribe_transient_local:=true'], stdout=log, stderr=log)
                try:
                    saved = child.wait(timeout=7) == 0
                except subprocess.TimeoutExpired:
                    stop_owned(child, grace=1)
                    saved = False
            for ext in ('yaml', 'pgm'):
                f = prefix.with_suffix('.' + ext)
                evidence[ext + '_ok'] = saved and f.is_file() and f.stat().st_size > 0
        except OSError:
            pass  # raster failure leaves its evidence false; the graph is still attempted
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
                    (directory / ('graph.' + e)).is_file() and (directory / ('graph.' + e)).stat().st_size for e in ('posegraph', 'data'))
        except Exception:
            return 'UNKNOWN', evidence     # partial posegraph files are kept for inspection
        return ('OK' if all(evidence.values()) else 'FAILED'), evidence

    def upload_saved(self, directory, ref):
        files = {'yaml': directory / 'map.yaml', 'pgm': directory / 'map.pgm',
                 'posegraph': directory / 'graph.posegraph', 'data': directory / 'graph.data'}
        payload = dict(robot_id=self.agent.robot_id, agent_boot_id=self.agent.boot_id, **ref,
                       yaml_b64=base64.b64encode(files['yaml'].read_bytes()).decode('ascii'),
                       pgm_b64=base64.b64encode(files['pgm'].read_bytes()).decode('ascii'),
                       posegraph_b64=base64.b64encode(files['posegraph'].read_bytes()).decode('ascii'),
                       posegraph_data_b64=base64.b64encode(files['data'].read_bytes()).decode('ascii'),
                       evidence={'sensors': {'slam_raster': {'lidar': True, 'depth': False, 'memory': False,
                                                             'basis': 'worker-owned async_slam_toolbox_node'}},
                                 'field_verification': 'UNVERIFIED', 'robot_mode': self.robot.mode,
                                 'simulated': self.robot.simulated})
        result = self.agent.http.post('/agent/mapping-map', payload, timeout=30)
        sums = {k: hashlib.sha256(v.read_bytes()).hexdigest() for k, v in files.items()}
        if not (result.get('integrity') or {}).get('ok'):
            raise RuntimeError('registry integrity check failed')
        return result.get('map_version_id'), sums

    # ------------------------------------------------------------ command execution
    def operate(self, c, session, cid):
        action = c['action']
        if action == 'START':
            self.plan = dict(c.get('plan') or {'mode': 'NEW_MAP'})
            if self.plan.get('mode') != 'NEW_MAP':
                return 'FAILED', {}, 'CONTINUE_MAP pose-graph loading is not implemented in this worker', None
            self.status = 'STARTING'
            self.goal = None
            self.attempted.clear()
            self.outcomes = []; self.recorded = set(); self.last_decision = None; self.repositions = 0; self.decisions = []
            self.stop_reason = None
            self.geofence_rejected = 0
            self.started_mono = time.monotonic()
            session.mkdir()
            nav = self.robot._nav2_goals()
            if self.robot.active is not None or nav['publishers'] not in (0, 1) or nav['active_count'] not in (None, 0):
                return 'UNKNOWN', {}, 'Nav2 slot unavailable', None
            self.launch(session)
        if action in ('START', 'RESUME'):
            if action == 'RESUME':
                self.status = 'STARTING'
            ready = self.wait(lambda: self.ready() and self.nav_idle() and self.agent.connected, 15)
            if ready:
                self.status = 'MAPPING'
            return ('OK' if ready else 'UNKNOWN'), dict(slam_running=ready, frontier_running=ready), (
                'SLAM map and Frontier candidates observed' if ready else 'ROS readiness timed out'), None
        if action == 'SAVE':
            directory = session / ('save-' + cid)
            outcome, evidence = self.save_map(directory)
            files = {'dir': str(directory)}
            detail = 'raster and posegraph saved independently'
            if outcome == 'OK' and c.get('registry_target'):
                try:
                    mvid, sums = self.upload_saved(directory, dict(session_id=c['session_id'], command_id=cid))
                    files.update(map_version_id=mvid, sha256=sums)
                    detail += f'; registered {mvid} (UNVERIFIED)'
                except Exception as exc:
                    outcome = 'FAILED'
                    detail = f'saved locally but registry upload failed: {type(exc).__name__}: {str(exc)[:120]}'
            return outcome, evidence, detail, files
        self.status = 'PAUSING' if action == 'PAUSE' else 'STOPPING'
        terminal = self.halt_goal()
        if action == 'STOP':
            self.stop_processes()
        self.status = ('PAUSED' if action == 'PAUSE' else 'IDLE') if terminal else 'UNKNOWN'
        return ('OK' if terminal else 'UNKNOWN'), dict(
            goal_terminal=bool(terminal), frontier_paused=bool(terminal),
            frontier_stopped=action == 'STOP' and not self.processes), (
            'owned goal and controller terminal or never accepted' if terminal else 'goal or controller outcome unresolved'), None

    def reconcile(self):
        """Evidence for clearing an UNKNOWN session (operator RECONCILE). Never resumes anything."""
        self.halt_goal()
        self.stop_processes()
        names = {n for n, _ in self.robot.get_node_names_and_namespaces()}
        graph_clear = (not names & {'slam_toolbox', 'fleet_frontier_candidates', 'explore_node', 'explore'}
                       and not self.robot.count_publishers('/fleet/frontier_candidates')
                       and all(i.node_name != 'slam_toolbox' for i in self.robot.get_publishers_info_by_topic('/map')))
        idle = self.robot.manual_idle_evidence() if hasattr(self.robot, 'manual_idle_evidence') else {}
        goal_terminal = (self.robot.active is None and self.nav_idle() and idle.get('ok') is True
                         and (self.goal is None or self.goal['status'] in GOAL_DONE))
        evidence = dict(processes_stopped=not self.processes, graph_clear=bool(graph_clear), goal_terminal=bool(goal_terminal))
        ok = all(evidence.values())
        return ('OK' if ok else 'FAILED'), evidence, (
            'owned processes stopped, graph clear, no active goal' if ok else 'reconciliation evidence incomplete: '
            + ', '.join(k for k, v in evidence.items() if not v))

    def execute(self, c):
        sid, cid, action = c.get('session_id'), c.get('command_id'), c.get('action')
        if action == 'RECONCILE':
            return self.execute_reconcile(c)
        if (self.status == 'UNKNOWN' or self.state.get('pending') or c.get('kind') != 'mapping'
                or c.get('robot_id') != self.agent.robot_id or c.get('expires_at', 0) <= time.time()
                or type(c.get('epoch')) is not int
                or not all(isinstance(x, str) and re.fullmatch('[0-9a-f]{32}', x) for x in (sid, cid))):
            return
        expected = {'START': ('IDLE',), 'RESUME': ('PAUSED',), 'SAVE': ('PAUSED',), 'PAUSE': ('MAPPING',), 'STOP': ('MAPPING', 'PAUSED')}
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
            outcome, evidence, detail, files = self.operate(c, session, cid)
            if self.goal:
                write_json(session / ('goal-' + self.goal['command_id'] + '.json'), self.goal.copy())
        except Exception as exc:
            outcome, evidence, detail, files = 'UNKNOWN', {}, type(exc).__name__ + ': ' + str(exc), None
            if action in ('START', 'RESUME'):
                self.status = 'UNKNOWN'
        self.detail = detail
        self.state.update(last_command=cid, last_outcome=outcome, evidence=evidence)
        write_json(self.path, self.state)
        if action == 'START' and outcome != 'OK':
            # A start that did not become ready leaves nothing running behind (owned processes only).
            self.halt_goal()
            self.stop_processes()
            self.status = 'UNKNOWN'
        result = self.request('result', **ref, outcome=outcome, evidence=evidence, detail=detail, files=files)
        expected_result = 'COMPLETED' if outcome == 'OK' else 'FAILED'
        if outcome == 'UNKNOWN' or result.get('command', {}).get('status') != expected_result:
            raise RuntimeError('reconciliation required')
        self.state['pending'] = None
        if action == 'STOP':
            self.state['session_id'] = None
        write_json(self.path, self.state)
        self.fence = time.monotonic() + 3

    def execute_reconcile(self, c):
        sid, cid = c.get('session_id'), c.get('command_id')
        if (self.status != 'UNKNOWN' or c.get('kind') != 'mapping' or c.get('robot_id') != self.agent.robot_id
                or c.get('expires_at', 0) <= time.time()
                or not all(isinstance(x, str) and re.fullmatch('[0-9a-f]{32}', x) for x in (sid, cid))):
            return
        ref = dict(session_id=sid, command_id=cid)
        if self.request('accept', **ref).get('accepted') is not True:
            return
        try:
            outcome, evidence, detail = self.reconcile()
        except Exception as exc:
            outcome, evidence, detail = 'FAILED', {}, 'reconcile error: ' + type(exc).__name__
        result = self.request('result', **ref, outcome=outcome, evidence=evidence, detail=detail, files=None)
        if outcome == 'OK' and (result.get('command') or {}).get('status') == 'COMPLETED':
            self.state.update(session_id=None, pending=None, last_command=cid, last_outcome='RECONCILED', evidence=evidence)
            write_json(self.path, self.state)
            self.goal = None
            self.status, self.detail = 'IDLE', 'reconciled by operator'

    def lose_session(self, why):
        self.status = 'UNKNOWN'
        self.detail = why
        terminal = self.halt_goal()
        self.stop_processes()
        if self.goal and self.state.get('session_id'):
            try:   # keep the per-goal journal consistent with what Nav2 finally reported
                write_json(self.root / self.state['session_id'] / ('goal-' + self.goal['command_id'] + '.json'), self.goal.copy())
            except OSError:
                pass
        self.state.update(last_outcome='UNKNOWN', goal_terminal=bool(terminal), lost_reason=why)
        write_json(self.path, self.state)

    def run(self):
        try:
            while not self.finished.wait(.15):
                if self.status == 'UNKNOWN':
                    if self.processes:
                        self.lose_session(self.detail or 'unknown state')
                    try:     # only an operator RECONCILE command is accepted while UNKNOWN
                        reply = self.request('poll', session_id=self.state.get('session_id')) if self.agent.connected else {}
                        command = reply.get('command')
                        if command and command.get('action') == 'RECONCILE':
                            self.execute(command)
                    except Exception:
                        pass
                    continue
                try:
                    sent = time.monotonic()
                    # While a session runs, a hung server must not hold this loop past the fence (the fence check
                    # below runs only after the poll returns): poll timeout <= time left on the fence.
                    t_out = max(0.2, min(2.0, self.fence - sent)) if self.status in ('MAPPING', 'PAUSED') else 2.0
                    reply = self.request('poll', timeout=t_out, session_id=self.state.get('session_id')) if self.agent.connected else {}
                    if reply.get('renew') is True:
                        # Fence from the request SEND time, capped by the Lease's remaining time on the server:
                        # motion stops at the Lease expiry instead of up to 3 s after it.
                        valid = reply.get('renew_valid_s')
                        valid = float(valid) if isinstance(valid, (int, float)) and valid == valid else 3.0
                        self.fence = sent + max(0.0, min(3.0, valid))
                    if reply.get('command'):
                        self.execute(reply['command'])
                except Exception as exc:
                    if self.state.get('pending'):
                        self.lose_session('command outcome unconfirmed: ' + type(exc).__name__)
                if self.status in ('MAPPING', 'PAUSED'):
                    if time.monotonic() >= self.fence:
                        self.lose_session('server/lease renewal lost')
                    elif not self.locked():
                        self.lose_session('motor path unlocked or motion watchdog not OK during mapping')
                    elif any(p.poll() is not None for p in self.processes):
                        self.lose_session('owned SLAM or Frontier process exited')
                    elif self.goal and self.goal['status'] == 'RECONCILIATION_REQUIRED':
                        self.lose_session('owned goal outcome unknown')
                    elif self.status == 'MAPPING':
                        try:
                            self.offer()
                        except Exception as exc:
                            self.lose_session('goal offer failed: ' + type(exc).__name__)
        finally:
            if self.processes:
                self.lose_session('agent shutting down')

    def close(self):
        self.finished.set()
        self.thread.join(timeout=19)
