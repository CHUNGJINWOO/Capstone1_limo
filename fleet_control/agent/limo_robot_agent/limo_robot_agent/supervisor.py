"""Phase 4 (2026-11-21): robot-side service supervisor - SSH-free start, readiness, bounded restarts, predefined web actions.

    python3 -m limo_robot_agent.supervisor --config <supervisor.json>

The ONLY source of command lines is the local config file on the robot (root of trust). The Fleet Server can name a service and
one of ACTIONS; it can never send a command, argument, path or environment. systemd (or any init) starts just this process
(see fleet_control/deploy/systemd/limo-supervisor.service, not installed anywhere by this repo).

Config (JSON):
  robot_id, server_url, token_file          identity: the robot's existing agent token (robot-scoped, not an operator token)
  report_period_s (1.0)
  services: [ {name, cmd: [argv...] | "shell string run with bash -lc" , env: {}, cwd, after: [names], start_delay_s,
               ready: {after_s, topics: [..]}, restart: {max, window_s, backoff_s}, required: true,
               actions: ["start","stop","restart","retry"]} ]   (listed in start order; `after` must be READY first)
  ros_probe: {domain_id, localhost_only, topics: {"/scan": {"type": "sensor_msgs/msg/LaserScan", "max_age_s": 1.0}, ...}}

`restart_with: [names]` restarts this service whenever one of those services restarts, exits or is stopped (Gate stack ->
Watchdog + Agent). Service states: WAITING (dependency not READY) -> STARTING -> RUNNING (process up) -> READY (probe ok) ; exit -> BACKOFF ->
restart (at most `max` in `window_s`) else FAILED (stays down until an operator `retry`); STOPPED by an operator `stop`.
Safety: this process starts processes, nothing else. It never publishes a ROS message, never touches /cmd_vel*, never
starts a mapping / navigation session and never clears a Gate latch. A restarted Gate or Agent goes through the existing
boot-change reconciliation on the server (admin recovery), exactly as after a manual restart.
"""
import argparse, json, os, signal, socket, subprocess, threading, time, urllib.error, urllib.request, uuid

ACTIONS = ('start', 'stop', 'restart', 'retry')


class Service:
    def __init__(self, spec, log_dir):
        self.spec, self.name = spec, spec['name']
        self.after = list(spec.get('after', []))
        self.restart_with = list(spec.get('restart_with', []))   # services whose restart / stop must also restart this one
        r = spec.get('restart') or {}
        self.max_restarts, self.window_s, self.backoff_s = int(r.get('max', 3)), float(r.get('window_s', 120)), float(r.get('backoff_s', 3))
        rd = spec.get('ready') or {}
        self.ready_after_s, self.ready_topics = float(rd.get('after_s', 2.0)), list(rd.get('topics', []))
        self.required = spec.get('required', True) is not False
        self.allowed = [a for a in spec.get('actions', ACTIONS) if a in ACTIONS]
        self.proc = None; self.state = 'WAITING'; self.since = time.time(); self.detail = ''; self.restarts = []; self.last_exit = None
        self.backoff_until = 0.0; self.started_mono = None; self.operator_stopped = False; self.log_dir = log_dir; self.n_start = 0

    def set(self, state, detail=''):
        if state != self.state or detail != self.detail:
            self.state, self.detail, self.since = state, detail, time.time()

    def start(self):
        cmd = self.spec['cmd']
        argv = ['bash', '-lc', cmd] if isinstance(cmd, str) else [str(x) for x in cmd]
        env = dict(os.environ, **{k: str(v) for k, v in (self.spec.get('env') or {}).items()})
        log = open(os.path.join(self.log_dir, f'{self.name}.{self.n_start}.log'), 'ab'); self.n_start += 1
        self.proc = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, cwd=self.spec.get('cwd'),
                                     env=env, start_new_session=True)
        log.close()                                                   # the child keeps its own descriptor
        self.started_mono = time.monotonic(); self.set('RUNNING', f'pid {self.proc.pid}')

    def stop(self, grace=8.0):
        p = self.proc
        if p is None or p.poll() is not None:
            self.proc = None; return
        self.set('STOPPING')
        for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
            try: os.killpg(p.pid, sig)
            except ProcessLookupError: break
            try: p.wait(wait); break
            except subprocess.TimeoutExpired: continue
        self.proc = None

    def view(self):
        return {'name': self.name, 'state': self.state, 'pid': self.proc.pid if self.proc and self.proc.poll() is None else None,
                'restarts': len(self.restarts), 'last_exit': self.last_exit, 'since': round(self.since, 1), 'detail': self.detail[:160],
                'required': self.required, 'actions': self.allowed}


class RosProbe:
    """Read-only freshness of configured topics (never a velocity topic). Runs in its own thread; ages in seconds."""
    def __init__(self, cfg):
        self.cfg = cfg or {}; self.seen = {}; self.payload = {}; self.ok = False; self.error = None
        topics = self.cfg.get('topics') or {}
        if any('cmd_vel' in t for t in topics):
            raise ValueError('the probe must not subscribe a velocity topic')
        if topics: threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        try:
            if self.cfg.get('domain_id') is not None: os.environ['ROS_DOMAIN_ID'] = str(self.cfg['domain_id'])
            if self.cfg.get('localhost_only'): os.environ['ROS_LOCALHOST_ONLY'] = '1'
            import importlib, rclpy
            from rclpy.qos import qos_profile_sensor_data
            rclpy.init(); node = rclpy.create_node('fleet_supervisor_probe')
            for topic, spec in self.cfg['topics'].items():
                pkg, _, typ = spec['type'].split('/')
                cls = getattr(importlib.import_module(f'{pkg}.msg'), typ)
                def cb(m, t=topic, keep=spec.get('json_keys')):
                    self.seen[t] = time.monotonic()
                    if keep:
                        try:
                            d = json.loads(m.data); self.payload[t] = {k: d.get(k) for k in keep}
                        except Exception: pass
                node.create_subscription(cls, topic, cb, qos_profile_sensor_data if spec.get('sensor_qos', True) else 10)
            self.ok = True
            rclpy.spin(node)
        except Exception as e:
            self.error = f'{type(e).__name__}: {e}'[:200]

    def age(self, topic):
        t = self.seen.get(topic)
        return None if t is None else time.monotonic() - t

    def view(self):
        out = {}
        for topic, spec in (self.cfg.get('topics') or {}).items():
            a = self.age(topic)
            out[topic] = {'age_s': None if a is None else round(a, 2), 'max_age_s': spec.get('max_age_s', 1.0), **({'last': self.payload[topic]} if topic in self.payload else {})}
        return {'probe_ok': self.ok, 'error': self.error, 'topics': out}


class Supervisor:
    def __init__(self, cfg, log_dir):
        self.cfg = cfg; self.boot_id = uuid.uuid4().hex[:12]; self.log_dir = log_dir
        names = [s['name'] for s in cfg['services']]
        if len(names) != len(set(names)): raise ValueError('duplicate service names')
        for s in cfg['services']:
            for d in s.get('after', []):
                if d not in names[:names.index(s['name'])]: raise ValueError(f"{s['name']}: dependency {d} must be listed before it")
        self.services = [Service(s, log_dir) for s in cfg['services']]
        self.by_name = {s.name: s for s in self.services}
        self.probe = RosProbe(cfg.get('ros_probe'))
        self.results, self.lock, self.stop_evt = [], threading.Lock(), threading.Event()
        self.token = open(os.path.expanduser(cfg['token_file'])).read().strip()
        self.events = open(os.path.join(log_dir, 'supervisor_events.jsonl'), 'a', buffering=1)
        self.server_ok = False; self.last_report_error = None; self.t0 = time.monotonic()
        self.event('supervisor_start', boot_id=self.boot_id, services=names)

    def event(self, kind, **d):
        self.events.write(json.dumps({'t': round(time.time(), 3), 'kind': kind, **d}) + '\n')

    # ------------------------------------------------------------------ service loop
    def ready_check(self, s):
        if s.proc is None or s.proc.poll() is not None: return False
        if time.monotonic() - s.started_mono < s.ready_after_s: return False
        for t in s.ready_topics:
            a = self.probe.age(t)
            limit = ((self.cfg.get('ros_probe') or {}).get('topics') or {}).get(t, {}).get('max_age_s', 1.0)
            if a is None or a > limit: return False
        return True

    def cascade(self, name, why):
        """Services that declare `restart_with: [name]` are restarted (after `name` is READY again). Used for the Gate stack: a new
        Gate boot needs a fresh Watchdog and Agent (the running Agent latches a Gate boot change by design, tests/nav_e2e
        ensure_nav2). Start order (`after`) alone never restarts anything: a Watchdog restart does not restart the Agent."""
        for d in self.services:
            if name in d.restart_with and d.state not in ('STOPPED', 'FAILED', 'WAITING'):
                d.stop(); d.restarts = []; d.set('WAITING', f'{why}: {name} restarted')
                self.event('service_cascade', service=d.name, because=name)
                self.cascade(d.name, why)

    def step(self):
        now = time.monotonic()
        for s in self.services:
            if s.state in ('STOPPED', 'FAILED', 'STOPPING'):
                continue
            if s.proc is not None and s.proc.poll() is not None:            # unexpected exit
                code = s.proc.returncode; s.proc = None; s.last_exit = {'code': code, 't': round(time.time(), 1)}
                s.restarts = [t for t in s.restarts if now - t < s.window_s]
                if len(s.restarts) >= s.max_restarts:
                    s.set('FAILED', f'exited ({code}) {len(s.restarts) + 1} times in {s.window_s:.0f} s; operator retry required')
                    self.event('service_failed', service=s.name, code=code, restarts=len(s.restarts))
                    self.cascade(s.name, 'dependency failed')
                    continue
                s.restarts.append(now); s.backoff_until = now + s.backoff_s
                s.set('BACKOFF', f'exited ({code}); restart in {s.backoff_s:.0f} s'); self.event('service_exit', service=s.name, code=code)
                self.cascade(s.name, 'dependency exited')
                continue
            if s.state == 'BACKOFF' and now < s.backoff_until:
                continue
            if s.proc is None:
                delay = float(s.spec.get('start_delay_s', 0))       # first start only (test hook for a slow sensor / DDS)
                if s.n_start == 0 and now - self.t0 < delay:
                    s.set('WAITING', f'start delay {delay:.0f} s'); continue
                blockers = [d for d in s.after if self.by_name[d].state != 'READY']
                if blockers:
                    s.set('WAITING', 'waiting for ' + ', '.join(blockers)); continue
                s.set('STARTING'); self.event('service_start', service=s.name, attempt=len(s.restarts))
                try: s.start()
                except Exception as e:
                    s.set('FAILED', f'start failed: {type(e).__name__}: {e}'[:160]); self.event('service_failed', service=s.name, error=str(e)[:160])
                continue
            if s.state == 'RUNNING' and self.ready_check(s):
                s.set('READY', ''); self.event('service_ready', service=s.name)
            elif s.state == 'READY' and not self.ready_check(s):
                s.set('RUNNING', 'readiness probe lost'); self.event('service_unready', service=s.name)

    def apply(self, a):
        s = self.by_name.get(a.get('service'))
        if s is None or a.get('action') not in ACTIONS or a['action'] not in s.allowed:
            return False, 'not allowed by the local configuration'
        act = a['action']
        if act == 'stop':
            s.stop(); s.set('STOPPED', 'stopped by operator'); self.cascade(s.name, 'dependency stopped'); return True, 'stopped'
        if act == 'restart':
            s.stop(); s.restarts = []; s.set('WAITING', 'restart requested'); self.cascade(s.name, 'dependency restart'); return True, 'restarting'
        if act in ('start', 'retry'):
            if s.state not in ('STOPPED', 'FAILED'): return False, f'service is {s.state}'
            s.restarts = []; s.set('WAITING', f'{act} requested'); return True, 'starting'
        return False, 'unknown action'

    # ------------------------------------------------------------------ server link
    def report(self):
        body = {'robot_id': self.cfg['robot_id'], 'boot_id': self.boot_id, 'services': [s.view() for s in self.services],
                'ros': self.probe.view(), 'host': {'name': socket.gethostname(), 'uptime_s': round(time.monotonic(), 1)},
                'action_results': self.results[:16]}
        req = urllib.request.Request(self.cfg['server_url'].rstrip('/') + '/api/v1/agent/supervisor', data=json.dumps(body).encode(), method='POST',
                                     headers={'Authorization': 'Bearer ' + self.token, 'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=3) as r: resp = json.loads(r.read())
            self.results = []
            if not self.server_ok: self.event('server_connected', server_boot=resp.get('server_boot_id'))
            self.server_ok = True; self.last_report_error = None
            for a in resp.get('actions') or []:
                ok, detail = self.apply(a); self.results.append({'action_id': a.get('action_id'), 'ok': ok, 'detail': detail})
                self.event('service_action', service=a.get('service'), action=a.get('action'), ok=ok, detail=detail)
        except (urllib.error.URLError, OSError, ValueError) as e:
            if self.server_ok: self.event('server_lost', error=str(e)[:120])
            self.server_ok = False; self.last_report_error = str(e)[:120]      # services keep running; nothing moves the robot

    def run(self):
        period = float(self.cfg.get('report_period_s', 1.0)); next_report = 0.0
        while not self.stop_evt.is_set():
            self.step()
            if time.monotonic() >= next_report:
                self.report(); next_report = time.monotonic() + period
            self.stop_evt.wait(0.2)
        for s in reversed(self.services): s.stop()                     # reverse start order
        self.event('supervisor_stop')


def check_config(cfg):
    """Static validation only (nothing is started): names unique, dependencies listed earlier, actions known, probe safe."""
    errs, names = [], []
    for k in ('robot_id', 'server_url', 'token_file', 'services'):
        if k not in cfg: errs.append(f'missing {k}')
    for s in cfg.get('services', []):
        n = s.get('name')
        if not n or n in names: errs.append(f'bad or duplicate service name {n!r}')
        if not s.get('cmd'): errs.append(f'{n}: cmd missing')
        for d in s.get('after', []):
            if d not in names: errs.append(f'{n}: dependency {d} must be listed before it')
        for d in s.get('restart_with', []):
            if d not in names: errs.append(f'{n}: restart_with {d} must be listed before it')
        bad = [a for a in s.get('actions', ACTIONS) if a not in ACTIONS]
        if bad: errs.append(f'{n}: unknown actions {bad}')
        names.append(n)
    if any('cmd_vel' in t for t in ((cfg.get('ros_probe') or {}).get('topics') or {})): errs.append('ros_probe must not subscribe a velocity topic')
    return errs


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--config', required=True); ap.add_argument('--log-dir'); ap.add_argument('--check', action='store_true')
    a = ap.parse_args()
    cfg = json.load(open(a.config))
    if a.check:
        errs = check_config(cfg); print(json.dumps({'ok': not errs, 'errors': errs, 'services': [s.get('name') for s in cfg.get('services', [])]}))
        raise SystemExit(1 if errs else 0)
    log_dir = a.log_dir or cfg.get('log_dir') or os.path.join(os.path.dirname(os.path.abspath(a.config)), 'supervisor_logs')
    os.makedirs(log_dir, exist_ok=True)
    sup = Supervisor(cfg, log_dir)
    signal.signal(signal.SIGTERM, lambda *_: sup.stop_evt.set()); signal.signal(signal.SIGINT, lambda *_: sup.stop_evt.set())
    sup.run()


if __name__ == '__main__':
    main()
