"""Agent side of the web "save current SLAM map and register it" (save only: no mapping start, no motion, no Goal).

Flow (server: fleet_server/map_save.py): poll -> accept -> SAVING_RASTER -> SAVING_POSEGRAPH -> UPLOADING -> server registers UNVERIFIED.

Rules
- Runs only when the motor path is LOCKED (or the simulation Safety Gate policy) and a live SLAM map + slam_toolbox serialize service exist.
- Never deletes anything: every save keeps its files and a status.json under <state_dir>/map_saves/<save_id>/ even when the upload fails.
- The raster (map_saver_cli) and the pose graph (slam_toolbox serialize_map) are two separate calls. Each file records its own size, SHA-256 and
  save window; the live map version/cell count and the odometry displacement around the calls are sampled so the server can state, but never
  overstate, how well the two belong together.
- The Agent journals a started save locally; after a restart an unfinished save is marked INTERRUPTED and is never resumed or replayed.
- Upload retries are safe: the server registers by content and answers a repeat of a finished save idempotently.
"""
import base64
import hashlib
import json
import math
import subprocess
import sys
import threading
import time
from pathlib import Path

from .core import HttpError
from .local_process import spawn_owned, stop_owned, write_json

# Real LIMO finding (2026-11-20): on the running stack a DDS participant that starts alone never matched slam_toolbox's latched /map, even
# with a 10 s wait ('Failed to spin map subscription'; 0/3 trials). The same map_saver_cli succeeded in 0.4-1.3 s (6/6 trials) when ANOTHER
# throw-away participant had been started about 3 s earlier. The cause is not established. Until it is, every raster save starts such a
# helper first (no topics, no publishing, stopped right after) and the real result is still judged only by map_saver_cli's exit code and files.
SAVE_MAP_TIMEOUT_S = 10.0
WARMUP_S = 3.0
WARMUP_CODE = ('import rclpy,time\nrclpy.init()\nn=rclpy.create_node("fleet_map_save_warmup")\nt=time.time()\n'
               'while time.time()-t<40: rclpy.spin_once(n,timeout_sec=0.5)\n')
FILES = {'yaml': 'map.yaml', 'pgm': 'map.pgm', 'posegraph': 'graph.posegraph', 'data': 'graph.data'}
TERMINAL = ('REGISTERED', 'FAILED', 'INTERRUPTED')


class MapSaveError(Exception):
    def __init__(self, code, message, manifest=None):
        super().__init__(message)
        self.code, self.message, self.manifest = code, message, manifest


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


class RosSaver:
    """The two real saving calls. Raster: map_saver_cli with free_thresh 0.196 (keeps unknown pixels unknown). Pose graph: serialize_map."""

    def __init__(self, robot, free_thresh=0.196):
        from slam_toolbox.srv import SerializePoseGraph
        from .mapping_worker import executable
        self.robot, self.free = robot, free_thresh
        self.serializer = robot.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.request_type = SerializePoseGraph.Request
        self.error = None
        try:
            self.map_saver = executable('nav2_map_server', 'map_saver_cli')
        except Exception as exc:  # noqa: BLE001
            self.map_saver, self.error = None, f'{type(exc).__name__}: {exc}'

    def ready(self):
        if self.map_saver is None:
            return False, f'map_saver_cli not found ({self.error})'
        if self.robot.map_meta is None:
            return False, 'no live /map received (SLAM is not running)'
        if not self.serializer.service_is_ready():
            return False, 'slam_toolbox serialize_map service is not available'
        return True, ''

    def sample(self):
        with self.robot.lock:
            meta = dict(self.robot.map_meta) if self.robot.map_meta else {}
            version = self.robot.map_version
        out = {'t': time.time(), 'map_version': version, 'known_cells': meta.get('known_cells')}
        try:
            pose = self.robot.snapshot()['pose']
            if pose.get('status') == 'OK':
                out.update(x=pose['x'], y=pose['y'], yaw=pose['yaw_rad'])
        except Exception:  # noqa: BLE001
            pass
        return out

    def motor_path_ok(self):
        try:
            status = self.robot._motor_path_status()[0]
        except Exception as exc:  # noqa: BLE001
            return False, f'motor path unknown ({type(exc).__name__})'
        return (status in ('LOCKED', 'SIM_GATED')), f'motor path is {status} (must be LOCKED or SIM_GATED)'

    def save_raster(self, directory):
        t0 = time.time()
        prefix = directory / 'map'
        helper = None
        try:
            if WARMUP_S > 0:
                helper = spawn_owned([sys.executable, '-c', WARMUP_CODE], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(WARMUP_S)
            with (directory / 'raster.log').open('x') as log:
                child = spawn_owned([self.map_saver, '-f', prefix, '--fmt', 'pgm', '--free', str(self.free), '--ros-args',
                                     '-p', 'map_subscribe_transient_local:=true', '-p', f'save_map_timeout:={SAVE_MAP_TIMEOUT_S}'], stdout=log, stderr=log)
                try:
                    ok = child.wait(timeout=SAVE_MAP_TIMEOUT_S + 12) == 0
                except subprocess.TimeoutExpired:
                    stop_owned(child, grace=1)
                    return {'ok': False, 'started': t0, 'finished': time.time(), 'error': 'map_saver_cli timed out'}
        except OSError as exc:
            return {'ok': False, 'started': t0, 'finished': time.time(), 'error': f'cannot run map_saver_cli: {exc}'}
        finally:
            if helper is not None:
                stop_owned(helper, grace=1)
        return {'ok': ok, 'started': t0, 'finished': time.time(), 'error': '' if ok else 'map_saver_cli exited with an error (see raster.log)'}

    def save_posegraph(self, directory):
        t0 = time.time()
        req = self.request_type()
        req.filename = str(directory / 'graph')
        try:
            future = self.serializer.call_async(req)
            deadline = time.monotonic() + 12
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not future.done():
                future.cancel()
                return {'ok': False, 'started': t0, 'finished': time.time(), 'error': 'serialize_map did not answer in time', 'result_code': None}
            code = int(future.result().result)
        except Exception as exc:  # noqa: BLE001
            return {'ok': False, 'started': t0, 'finished': time.time(), 'error': f'serialize_map call failed: {type(exc).__name__}', 'result_code': None}
        return {'ok': code == 0, 'started': t0, 'finished': time.time(), 'result_code': code,
                'error': '' if code == 0 else f'serialize_map returned result {code}'}


class MapSaveWorker:
    def __init__(self, agent, state_dir, saver, poll_s=1.0, upload_attempts=3, backoff_s=2.0, start_thread=True):
        self.agent, self.saver = agent, saver
        self.root = Path(state_dir).resolve() / 'map_saves'
        self.root.mkdir(parents=True, exist_ok=True)
        self.poll_s, self.upload_attempts, self.backoff_s = poll_s, upload_attempts, backoff_s
        self.lock = threading.Lock()
        self.active = None
        self.last = None
        self.stop_evt = threading.Event()
        self._recover()
        self.thread = threading.Thread(target=self.run, daemon=True)
        if start_thread:
            self.thread.start()

    # ------------------------------------------------------------ status
    def capability(self):
        # Busy is NOT "unavailable": the server already allows one save at a time, and the Agent's last posted state can still say
        # "busy" for up to a second after the server has registered the previous save. A queued request simply waits for the loop.
        ok, why = self.saver.ready()
        if not ok:
            return False, why
        ok, why = self.saver.motor_path_ok()
        if not ok:
            return False, why
        return True, ''

    def snapshot(self):
        ok, why = self.capability()
        with self.lock:
            return {'enabled': True, 'available': ok, 'reason': why, 'active': self.active, 'last': dict(self.last) if self.last else None}

    def stop(self):
        self.stop_evt.set()

    # ------------------------------------------------------------ journal
    def _recover(self):
        for d in sorted(self.root.iterdir()) if self.root.exists() else []:
            f = d / 'status.json'
            try:
                st = json.loads(f.read_text())
            except (OSError, ValueError):
                continue
            if st.get('status') not in TERMINAL:
                st.update(status='INTERRUPTED', error_code='agent_restarted', detail='the Agent restarted during this save; never resumed',
                          updated_at=time.time())
                write_json(f, st)

    def _journal(self, d, **fields):
        f = d / 'status.json'
        try:
            st = json.loads(f.read_text())
        except (OSError, ValueError):
            st = {}
        st.update(fields, updated_at=time.time())
        write_json(f, st)
        with self.lock:
            self.last = {k: st.get(k) for k in ('save_id', 'status', 'stage', 'error_code', 'detail', 'map_version_id', 'dir')}

    # ------------------------------------------------------------ server calls
    def request(self, action, timeout=4.0, **fields):
        return self.agent.http.post('/agent/map-save', dict(robot_id=self.agent.robot_id, agent_boot_id=self.agent.boot_id, action=action, **fields),
                                    timeout=timeout)

    def _stage(self, sid, d, stage, detail=''):
        self._journal(d, stage=stage, status=stage, detail=detail)
        try:
            self.request('stage', save_id=sid, stage=stage, detail=detail)
        except Exception:  # noqa: BLE001
            pass                   # progress is informational; the terminal result is what counts

    def _fail(self, sid, d, code, message, manifest=None):
        self._journal(d, status='FAILED', stage='FAILED', error_code=code, detail=message)
        for _ in range(2):
            try:
                self.request('fail', save_id=sid, error_code=code, message=message, manifest=manifest)
                return
            except Exception:  # noqa: BLE001
                time.sleep(0.5)

    # ------------------------------------------------------------ loop
    def run(self):
        while not self.stop_evt.wait(self.poll_s):
            if not getattr(self.agent, 'connected', False):
                continue
            try:
                reply = self.request('poll')
                if reply.get('request'):
                    self.handle(reply['request'])
            except Exception as exc:  # noqa: BLE001
                self.agent.log(f'map save poll failed: {type(exc).__name__}')

    def handle(self, req):
        sid = req['save_id']
        d = self.root / sid
        try:
            d.mkdir()
        except FileExistsError:                 # a previous attempt of this id began here: never run it twice
            try:
                self.request('accept', save_id=sid)
            except Exception:  # noqa: BLE001
                pass
            self._fail(sid, d, 'duplicate_attempt', 'a local attempt for this save id already exists; start a new save')
            return
        with self.lock:
            self.active = sid
        try:
            self._journal(d, save_id=sid, status='ACCEPTED', stage='ACCEPTED', dir=str(d), target=req.get('target'), started_at=time.time())
            if self.request('accept', save_id=sid).get('accepted') is not True:
                self._journal(d, status='FAILED', error_code='not_accepted', detail='the server did not accept the Agent')
                return
            self._execute(sid, d, req.get('target') or {})
        except MapSaveError as exc:
            self._fail(sid, d, exc.code, exc.message, exc.manifest)
        except Exception as exc:  # noqa: BLE001
            self._fail(sid, d, 'agent_error', f'{type(exc).__name__}: {exc}'[:300])
        finally:
            with self.lock:
                self.active = None

    # ------------------------------------------------------------ the save itself
    def _execute(self, sid, d, target):
        ok, why = self.saver.ready()
        if not ok:
            raise MapSaveError('agent_not_ready', why)
        ok, why = self.saver.motor_path_ok()
        if not ok:
            raise MapSaveError('motor_path_not_locked', why)
        samples = [self.saver.sample()]
        self._stage(sid, d, 'SAVING_RASTER', 'map_saver_cli')
        raster = self.saver.save_raster(d)
        samples.append(self.saver.sample())
        self._stage(sid, d, 'SAVING_POSEGRAPH', 'slam_toolbox serialize_map')
        graph = self.saver.save_posegraph(d)
        samples.append(self.saver.sample())
        manifest = self._manifest(d, raster, graph, samples)
        manifest['results'] = {'raster': {k: raster.get(k) for k in ('ok', 'error', 'started', 'finished')},
                               'posegraph': {k: graph.get(k) for k in ('ok', 'error', 'started', 'finished', 'result_code')}}
        problems = [f"{k}: {v['problem']}" for k, v in manifest['files'].items() if v.get('problem')]
        if not raster['ok'] or not graph['ok']:        # both are attempted so the operator sees what each one did
            self._journal(d, manifest=manifest)
            both = '; '.join(f"{n}: {r['error']}" for n, r in (('raster', raster), ('posegraph', graph)) if not r['ok'])
            raise MapSaveError('raster_failed' if not raster['ok'] else 'posegraph_failed', both, manifest)
        if problems:
            self._journal(d, manifest=manifest)
            raise MapSaveError('saved_file_invalid', '; '.join(problems), manifest)
        self._journal(d, manifest=manifest, stage='SAVED_LOCALLY')
        self._stage(sid, d, 'UPLOADING', f"{sum(v['size'] for v in manifest['files'].values())} bytes")
        result = self._upload(sid, d, manifest)
        self._journal(d, status='REGISTERED', stage='REGISTERED', map_version_id=result.get('map_version_id'), detail='registered as UNVERIFIED')

    def _manifest(self, d, raster, graph, samples):
        files = {}
        for key, name in FILES.items():
            p = d / name
            win = raster if key in ('yaml', 'pgm') else graph
            entry = {'name': name, 'saved_start': win['started'], 'saved_end': win['finished']}
            try:
                size = p.stat().st_size
                entry.update(size=size, sha256=sha256_file(p))
                if size == 0:
                    entry['problem'] = 'empty file'
            except OSError:
                entry.update(size=0, sha256=None, problem='file missing')
            files[key] = entry
        pts = [(s.get('x'), s.get('y'), s.get('yaw')) for s in samples if s.get('x') is not None]
        moved = rot = None
        if len(pts) == len(samples):
            moved = max(math.hypot(a[0] - b[0], a[1] - b[1]) for a in pts for b in pts)
            rot = max(abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))) for a in pts for b in pts)
        return {'files': files,
                'saver': {'raster': 'nav2_map_server map_saver_cli --fmt pgm', 'free_thresh_requested': getattr(self.saver, 'free', None),
                          'posegraph': 'slam_toolbox serialize_map', 'serialize_result_code': graph.get('result_code')},
                'evidence': {'map_before': {k: samples[0].get(k) for k in ('map_version', 'known_cells')} | {'version': samples[0].get('map_version')},
                             'map_after': {k: samples[-1].get(k) for k in ('map_version', 'known_cells')} | {'version': samples[-1].get('map_version')},
                             'moved_m': None if moved is None else round(moved, 4), 'rotated_rad': None if rot is None else round(rot, 4),
                             'pose_samples': len(samples)}}

    def _upload(self, sid, d, manifest):
        payload = {'save_id': sid, 'manifest': manifest}
        for key, name in FILES.items():
            payload[key + '_b64'] = base64.b64encode((d / name).read_bytes()).decode('ascii')
        last = 'no attempt'
        for attempt in range(1, self.upload_attempts + 1):
            try:
                return self.request('bundle', timeout=60, **payload)
            except HttpError as exc:
                code, msg = f'http_{exc.status}', exc.body[:200]
                try:
                    err = json.loads(exc.body).get('error', {})
                    code, msg = err.get('code') or code, err.get('message') or msg
                except (ValueError, AttributeError):
                    pass
                if code == 'transfer_corrupt' and attempt < self.upload_attempts:
                    last = f'{code}: {msg}'
                    time.sleep(self.backoff_s * attempt)
                    continue
                raise MapSaveError(code, msg)
            except Exception as exc:  # noqa: BLE001      # outcome unknown (timeout, connection): retrying is idempotent on the server
                last = f'{type(exc).__name__}: {exc}'[:200]
                if attempt < self.upload_attempts:
                    time.sleep(self.backoff_s * attempt)
        raise MapSaveError('upload_failed', f'upload failed after {self.upload_attempts} attempts ({last}); files kept on the robot at {d}')
