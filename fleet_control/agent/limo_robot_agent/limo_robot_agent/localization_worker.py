"""Saved-map localization for NAVIGATION mode: Registry bundle -> map_server + AMCL (owned processes) -> evidence.

Desired state comes from the Fleet server (state response `navigation_request`, present only in NAVIGATION mode):
  {request_id, map_version_id, yaml_sha256, pgm_sha256, map_verified, initial_pose: {x, y, yaw, seq} | None}
The worker
  * downloads map.yaml / map.pgm of exactly that version over the Agent API and checks both SHA-256 values,
  * starts nav2_map_server + nav2_amcl from the downloaded files (owned, PDEATHSIG; never adopts other nodes) and
    refuses when another /map publisher (e.g. SLAM) exists,
  * sets the global planner `GridBased.allow_unknown=false` for saved-map patrol (restored to true afterwards; mapping
    needs unknown space for Frontier goals),
  * publishes the operator's initial pose on /initialpose,
  * reports CONFIRMED only when AMCL published a pose after that initial pose, its xy/yaw covariance is small, the
    map->laser TF exists and >= MATCH_OK of the LiDAR endpoints land on (dilated) occupied map cells for
    CONFIRM_SAMPLES consecutive evaluations. Covariance alone is not used: AMCL does not update while stationary.
A localization that drops below MATCH_LOST for LOST_S becomes DEGRADED (not CONFIRMED). Failures never retry the
same request automatically. Only the isolated mock/sim Agent runs this worker (see agent_node).
"""
import base64, hashlib, math, re, subprocess, threading, time
from pathlib import Path

import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path as NavPath
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters
from std_srvs.srv import Empty
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan

from .local_process import spawn_owned, stop_owned, write_json

MATCH_OK, MATCH_LOST, LOST_S = .60, .40, 3.0
CONFIRM_SAMPLES = 5
INITIAL_POSE_SUB_WAIT_S = 5.0          # wait for AMCL's /initialpose subscription before publishing
INITIAL_POSE_RESEND_S, INITIAL_POSE_RESENDS = 3.0, 5   # re-send the same pose while AMCL publishes nothing
COV_XY_MAX, COV_YAW_MAX = .25, .30          # variance (m^2, rad^2)
AMCL_KEYS = dict(alpha1=.2, alpha2=.2, alpha3=.2, alpha4=.2, alpha5=.2, base_frame_id='base_footprint', beam_skip_distance=.5,
                 beam_skip_error_threshold=.9, beam_skip_threshold=.3, do_beamskip=False, global_frame_id='map', lambda_short=.1,
                 laser_likelihood_max_dist=2.0, laser_max_range=100.0, laser_min_range=-1.0, laser_model_type='likelihood_field',
                 max_beams=60, max_particles=2000, min_particles=500, odom_frame_id='odom', pf_err=.05, pf_z=.99,
                 recovery_alpha_fast=0.0, recovery_alpha_slow=0.0, resample_interval=1,
                 robot_model_type='nav2_amcl::DifferentialMotionModel', save_pose_rate=.5, sigma_hit=.2, tf_broadcast=True,
                 transform_tolerance=1.0, update_min_a=.2, update_min_d=.25, z_hit=.5, z_max=.05, z_rand=.5, z_short=.05,
                 scan_topic='scan', set_initial_pose=False)   # values of capstone_ws/.../config/nav2_sim_params.yaml amcl


def _exe(package, name):
    from ament_index_python.packages import get_package_prefix
    p = Path(get_package_prefix(package)) / 'lib' / package / name
    if not p.is_file():
        raise FileNotFoundError(p)
    return p


def parse_pgm(data):
    m = re.match(rb'P5\s+(?:#[^\n]*\s+)*(\d+)\s+(?:#[^\n]*\s+)*(\d+)\s+(?:#[^\n]*\s+)*(\d+)\s', data)
    if not m:
        raise ValueError('not a binary PGM')
    w, h, mx = (int(x) for x in m.groups())
    px = np.frombuffer(data[m.end():m.end() + w * h], dtype=np.uint8)
    if px.size != w * h or mx != 255:
        raise ValueError('PGM size/maxval unsupported')
    return w, h, px.reshape(h, w)


def parse_yaml(text):
    out = {}
    for line in text.splitlines():
        line = line.split('#', 1)[0]
        if ':' in line:
            k, v = line.split(':', 1); out[k.strip()] = v.strip()
    origin = [float(x) for x in out['origin'].strip('[]').split(',')]
    return {'image': out['image'], 'resolution': float(out['resolution']), 'origin': origin[:3],
            'negate': int(out.get('negate', 0)), 'occupied_thresh': float(out.get('occupied_thresh', .65)),
            'free_thresh': float(out.get('free_thresh', .196))}


class LocalizationWorker:
    def __init__(self, robot, agent, state_dir):
        self.robot, self.agent = robot, agent
        self.root = Path(state_dir).resolve() / 'localization_worker'
        self.root.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self.status, self.detail = 'IDLE', ''
        self.request = None; self.processes = []; self.grid = None; self.meta = None
        self.applied_seq = None; self.pose_sent_t = None; self.amcl = None
        self.pose_resends = 0; self.last_pose_send = 0.0; self.pose_subscribers = None
        self.scan = None; self.matches = []; self.lost_since = None; self.ratio = None
        self.allow_unknown = None; self.path = None
        self.tools_error = None
        try:
            self.map_server, self.amcl_exe, self.bringup = (_exe('nav2_map_server', 'map_server'), _exe('nav2_amcl', 'amcl'),
                                                            _exe('nav2_util', 'lifecycle_bringup'))
        except Exception as exc:
            self.tools_error = f'{type(exc).__name__}: {exc}'
        self.init_pub = robot.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        robot.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        robot.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        robot.create_subscription(NavPath, '/plan', self.on_plan, 10)
        self.set_params = robot.create_client(SetParameters, '/planner_server/set_parameters')
        self.get_params = robot.create_client(GetParameters, '/planner_server/get_parameters')
        # AMCL does not update (or publish a pose) while the robot is stationary; this service forces one filter update.
        self.nomotion = robot.create_client(Empty, '/request_nomotion_update')
        self.last_nomotion = 0.0
        robot.localization_fn = self.contexts
        self.finished = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True); self.thread.start()

    # ------------------------------------------------------------ ROS inputs
    def on_amcl(self, m):
        c = m.pose.covariance
        q = m.pose.pose.orientation
        self.amcl = (time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), c[0], c[7], c[35])

    def on_scan(self, m):
        self.scan = m

    def on_plan(self, m):
        pts = [(round(p.pose.position.x, 2), round(p.pose.position.y, 2)) for p in m.poses]
        step = max(1, len(pts) // 60)
        self.path = (time.monotonic(), pts[::step] + (pts[-1:] if pts and len(pts) % step else []), m.header.frame_id)

    # ------------------------------------------------------------ outputs
    def contexts(self):
        """(map_ctx, loc_ctx) for the Safety Context; fail closed unless CONFIRMED on the requested version."""
        r = self.request
        if not r or self.status in ('IDLE', 'FAILED', 'LOADING'):
            return None
        mapc = {'map_version_id': r['map_version_id'], 'map_epoch': int(r['request_id']), 'map_verified': r.get('map_verified') is True}
        locc = {'localization_status': 'CONFIRMED' if self.status == 'CONFIRMED' else self.status,
                'localization_map_version_id': r['map_version_id'] if self.status == 'CONFIRMED' else None,
                'localization_epoch': int(self.applied_seq) if self.applied_seq is not None else None}
        return mapc, locc

    def snapshot(self):
        a = self.amcl
        r = self.request or {}
        return {'enabled': True, 'status': self.status, 'detail': self.detail, 'request_id': r.get('request_id'),
                'map_version_id': r.get('map_version_id'), 'map_verified': r.get('map_verified'),
                'initial_pose_seq': self.applied_seq, 'match_ratio': None if self.ratio is None else round(self.ratio, 3),
                'amcl_pose': None if not a else {'x': round(a[1], 3), 'y': round(a[2], 3), 'yaw': round(a[3], 3),
                                                 'cov_xx': round(a[4], 4), 'cov_yy': round(a[5], 4), 'cov_yaw': round(a[6], 4),
                                                 'age_s': round(time.monotonic() - a[0], 2)},
                'processes_started': bool(self.processes), 'planner_allow_unknown': self.allow_unknown,
                'nav_path': None if not self.path else {'points': self.path[1], 'frame': self.path[2],
                                                         'age_s': round(time.monotonic() - self.path[0], 2)},
                'tools_error': self.tools_error}

    # ------------------------------------------------------------ helpers
    def planner_allow_unknown(self, value):
        if not self.set_params.service_is_ready():
            return None
        req = SetParameters.Request()
        req.parameters = [Parameter(name='GridBased.allow_unknown',
                                    value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(value)))]
        fut = self.set_params.call_async(req)
        end = time.monotonic() + 3
        while not fut.done() and time.monotonic() < end:
            time.sleep(.05)
        ok = fut.done() and fut.result() is not None and all(r.successful for r in fut.result().results)
        g = GetParameters.Request(); g.names = ['GridBased.allow_unknown']
        fut = self.get_params.call_async(g)
        end = time.monotonic() + 3
        while not fut.done() and time.monotonic() < end:
            time.sleep(.05)
        self.allow_unknown = fut.result().values[0].bool_value if fut.done() and fut.result() and fut.result().values else None
        return ok and self.allow_unknown == bool(value)

    def stop(self, why):
        while self.processes:
            stop_owned(self.processes.pop())
        if self.allow_unknown is False:
            self.planner_allow_unknown(True)
        self.grid = self.meta = None; self.applied_seq = None; self.amcl = None; self.matches = []; self.ratio = None
        self.detail = why

    def download(self, req, session):
        files = {}
        for name, key in (('map.yaml', 'yaml_sha256'), ('map.pgm', 'pgm_sha256')):
            r = self.agent.http.post('/agent/map-bundle', {'robot_id': self.agent.robot_id, 'agent_boot_id': self.agent.boot_id,
                                                           'map_version_id': req['map_version_id'], 'file': name}, timeout=20)
            data = base64.b64decode(r['data_b64'])
            if hashlib.sha256(data).hexdigest() != req[key]:
                raise ValueError(f'{name} SHA-256 mismatch')
            files[name] = data
        meta = parse_yaml(files['map.yaml'].decode())
        w, h, px = parse_pgm(files['map.pgm'])
        (session / 'map.pgm').write_bytes(files['map.pgm'])
        yaml_text = re.sub(r'(?m)^image:.*$', 'image: map.pgm', files['map.yaml'].decode())
        (session / 'map.yaml').write_text(yaml_text)
        p = (255 - px.astype(np.float32)) / 255.0 if not meta['negate'] else px.astype(np.float32) / 255.0
        occ = p > meta['occupied_thresh']
        dil = occ.copy()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                dil |= np.roll(np.roll(occ, dr, 0), dc, 1)
        self.grid, self.meta = dil, dict(meta, width=w, height=h)

    def launch(self, session):
        if self.tools_error:
            raise RuntimeError('localization tools unavailable: ' + self.tools_error)
        if self.robot.count_publishers('/map') or any(n in ('map_server', 'amcl', 'slam_toolbox')
                                                      for n, _ in self.robot.get_node_names_and_namespaces()):
            raise RuntimeError('graph occupied: /map publisher or map_server/amcl/slam_toolbox already running')
        sim = str(self.robot._use_sim_time()).lower()
        params = session / 'amcl.yaml'
        lines = ['amcl:', '  ros__parameters:', f'    use_sim_time: {sim}']
        lines += [f'    {k}: {str(v).lower() if isinstance(v, bool) else (repr(v) if isinstance(v, str) else v)}' for k, v in AMCL_KEYS.items()]
        params.write_text('\n'.join(lines).replace("'", '"') + '\n')
        for name, cmd in (('map_server', [self.map_server, '--ros-args', '-p', f'yaml_filename:={session / "map.yaml"}', '-p', 'use_sim_time:=' + sim]),
                          ('amcl', [self.amcl_exe, '--ros-args', '--params-file', str(params)])):
            with (session / (name + '.log')).open('x') as log:
                self.processes.append(spawn_owned([str(c) for c in cmd], stdin=subprocess.DEVNULL, stdout=log, stderr=log))
        time.sleep(1.5)
        with (session / 'bringup.log').open('x') as log:
            rc = subprocess.run([str(self.bringup), 'map_server', 'amcl'], stdout=log, stderr=log, timeout=40).returncode
        if rc != 0 or any(p.poll() is not None for p in self.processes):
            raise RuntimeError('map_server/amcl lifecycle bring-up failed')
        if not self.wait(lambda: self.robot.count_publishers('/map') == 1, 10):
            raise RuntimeError('map_server did not publish /map')

    def wait(self, fn, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            if fn(): return True
            time.sleep(.1)
        return False

    def send_initial_pose(self, pose):
        # 2026-11-21 (Phase 3 timeline, baseline/01_r11 n05): after a re-LOAD the new AMCL's /initialpose subscription was not matched
        # yet; the pose published 3x within 0.6 s was lost and AMCL never published a pose (CONVERGING for > 50 s). Wait until AMCL
        # subscribes (bounded), then publish. step() re-sends the SAME operator-given pose if AMCL still produces nothing.
        self.wait(lambda: self.init_pub.get_subscription_count() >= 1, INITIAL_POSE_SUB_WAIT_S)
        self.pose_subscribers = self.init_pub.get_subscription_count()
        m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'
        m.header.stamp = self.robot.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(pose['x']), float(pose['y'])
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(pose['yaw'] / 2), math.cos(pose['yaw'] / 2)
        cov = [0.0] * 36; cov[0] = cov[7] = .06; cov[35] = .06
        m.pose.covariance = cov
        for _ in range(3):
            self.init_pub.publish(m); time.sleep(.2)

    def match_ratio(self):
        s, g, meta = self.scan, self.grid, self.meta
        if s is None or g is None:
            return None
        # map->odom (AMCL, slowly varying, latest) composed with odom->laser AT THE SCAN STAMP (fast motion); the latest
        # odom->laser would misplace the endpoints while turning, and map->laser at the scan stamp is often not
        # available yet because AMCL's map->odom lags the newest scan.
        try:
            mo = self.robot.tfb.lookup_transform('map', 'odom', Time())
            ol = self.robot.tfb.lookup_transform('odom', s.header.frame_id, Time.from_msg(s.header.stamp),
                                                 timeout=Duration(seconds=0.15))   # the scan may be a few ms newer than the last odom TF
        except Exception:
            return None
        def xyyaw(tf):
            t, q = tf.transform.translation, tf.transform.rotation
            return t.x, t.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        (ax, ay, ath), (bx, by, bth) = xyyaw(mo), xyyaw(ol)
        t = type('T', (), {})(); t.x = ax + math.cos(ath) * bx - math.sin(ath) * by; t.y = ay + math.sin(ath) * bx + math.cos(ath) * by
        yaw = ath + bth
        r = np.asarray(s.ranges, dtype=np.float32)
        a = s.angle_min + np.arange(r.size) * s.angle_increment
        ok = np.isfinite(r) & (r > s.range_min) & (r < s.range_max * .98)
        if ok.sum() < 30:
            return None
        x = t.x + r[ok] * np.cos(a[ok] + yaw); y = t.y + r[ok] * np.sin(a[ok] + yaw)
        ox, oy, oyaw = meta['origin']; res = meta['resolution']
        dx, dy = x - ox, y - oy
        lx, ly = math.cos(oyaw) * dx + math.sin(oyaw) * dy, -math.sin(oyaw) * dx + math.cos(oyaw) * dy
        col = np.floor(lx / res).astype(int); row = meta['height'] - 1 - np.floor(ly / res).astype(int)
        inside = (col >= 0) & (col < meta['width']) & (row >= 0) & (row < meta['height'])
        hits = np.zeros(col.size, dtype=bool)
        hits[inside] = g[row[inside], col[inside]]
        return float(hits.mean())

    def evaluate(self):
        a = self.amcl
        ratio = self.match_ratio(); self.ratio = ratio
        fresh_pose = a is not None and self.pose_sent_t is not None and a[0] >= self.pose_sent_t
        cov_ok = fresh_pose and a[4] <= COV_XY_MAX and a[5] <= COV_XY_MAX and a[6] <= COV_YAW_MAX
        good = cov_ok and ratio is not None and ratio >= MATCH_OK
        self.matches = (self.matches + [good])[-CONFIRM_SAMPLES:]
        if self.status in ('CONVERGING', 'DEGRADED'):
            if len(self.matches) == CONFIRM_SAMPLES and all(self.matches) and self.allow_unknown is not False:
                if time.monotonic() - getattr(self, 'last_allow_unknown_try', 0) >= 1.0:
                    self.last_allow_unknown_try = time.monotonic(); self.planner_allow_unknown(False)
                if self.allow_unknown is not False:
                    self.detail = 'pose and match OK; waiting for planner GridBased.allow_unknown=false before CONFIRMED'
                    return
            if len(self.matches) == CONFIRM_SAMPLES and all(self.matches) and self.allow_unknown is False:
                self.status, self.detail, self.lost_since = 'CONFIRMED', 'AMCL pose + scan/map match confirmed', None
            elif not fresh_pose:
                self.detail = 'waiting for an AMCL pose after the initial pose'
            else:
                self.detail = f'converging: match {ratio if ratio is None else round(ratio, 2)}, cov ok {cov_ok}'
        elif self.status == 'CONFIRMED':
            if ratio is None or ratio < MATCH_LOST or not cov_ok:
                self.lost_since = self.lost_since or time.monotonic()
                if time.monotonic() - self.lost_since >= LOST_S:
                    self.status, self.detail = 'DEGRADED', f'scan/map match lost ({ratio})'
            else:
                self.lost_since = None

    # ------------------------------------------------------------ loop
    def step(self):
        desired = (self.agent.session_snapshot() or {}).get('navigation_request')
        if not isinstance(desired, dict) or not desired.get('map_version_id'):
            if self.processes or self.status != 'IDLE':
                self.stop('navigation request cleared'); self.request = None; self.status = 'IDLE'
            return
        if self.request is None or desired['request_id'] != self.request['request_id']:
            self.stop('new map request'); self.request = dict(desired); self.status = 'LOADING'; self.detail = 'downloading bundle'
            session = self.root / f"req-{desired['request_id']}-{int(time.time())}"
            try:
                session.mkdir(parents=True)
                self.download(desired, session)
                self.launch(session)
                # 2026-11-21 (Phase 3 fixed-map series, after_fix/01_r11): after a boot WITHOUT a SLAM session the planner's global
                # costmap waits for the map->base_footprint TF, which only AMCL provides after the initial pose; its parameter service
                # does not answer meanwhile. Failing LOAD here deadlocked (no LOAD -> no AMCL -> no TF -> planner never active).
                # The rule is unchanged: CONFIRMED (and therefore any mission) still requires allow_unknown == false (evaluate()).
                self.planner_allow_unknown(False)
                write_json(session / 'request.json', desired)
                self.status, self.detail = 'WAIT_INITIAL_POSE', 'map loaded; set the initial pose'
            except Exception as exc:
                self.stop('load failed'); self.status = 'FAILED'; self.detail = f'{type(exc).__name__}: {str(exc)[:160]}'
            return
        if self.status == 'FAILED':
            return
        if any(p.poll() is not None for p in self.processes):
            self.stop('map_server or amcl exited'); self.status = 'FAILED'; self.detail = 'map_server or amcl exited'
            return
        pose = desired.get('initial_pose')
        if isinstance(pose, dict) and pose.get('seq') != self.applied_seq:
            self.send_initial_pose(pose)
            self.applied_seq, self.pose_sent_t, self.matches = pose['seq'], time.monotonic(), []
            self.pose_resends, self.last_pose_send = 0, time.monotonic()
            self.status, self.detail = 'CONVERGING', 'initial pose sent'
            self.request = dict(desired)
            return
        a = self.amcl
        if (self.status == 'CONVERGING' and isinstance(pose, dict) and pose.get('seq') == self.applied_seq
                and not (a is not None and self.pose_sent_t is not None and a[0] >= self.pose_sent_t)
                and time.monotonic() - getattr(self, 'last_pose_send', 0) >= INITIAL_POSE_RESEND_S and getattr(self, 'pose_resends', 0) < INITIAL_POSE_RESENDS):
            self.send_initial_pose(pose)                  # same seq, same pose: no AMCL pose yet after the previous send
            self.pose_resends += 1; self.last_pose_send = time.monotonic()
            self.detail = f'initial pose re-sent ({self.pose_resends}/{INITIAL_POSE_RESENDS}, AMCL subscribers {self.pose_subscribers})'
            return
        if self.status in ('CONVERGING', 'DEGRADED') and time.monotonic() - self.last_nomotion > 1.0 and self.nomotion.service_is_ready():
            self.last_nomotion = time.monotonic(); self.nomotion.call_async(Empty.Request())
        if self.status in ('CONVERGING', 'CONFIRMED', 'DEGRADED'):
            self.evaluate()

    def run(self):
        while not self.finished.wait(.2):
            try:
                self.step()
            except Exception as exc:
                self.detail = 'worker error: ' + type(exc).__name__

    def close(self):
        self.finished.set(); self.thread.join(timeout=5)
        try:
            self.stop('agent shutting down')
        except Exception:
            pass
