#!/usr/bin/env python3
"""Integrated simulation: Open-RMF x Gazebo (real physics) x Nav2/AMCL x unchanged Safety Gate x Robot Agent x Mission Manager x video/YOLO (CUDA).

  RMF -> Fleet Adapter -> Mission Manager -> Robot Agent -> Nav2 -> Safety Gate -> Gazebo LIMO          (command plane)
  Gazebo RGB camera (on the moving robot) -> sender -> [chaos proxy] -> AI host (CUDA YOLO11n) -> result -> consumer on the robot side   (video/AI plane, separate TCP link)
  Mission Manager dashboard (map, pose, missions, video panel with YOLO) on http://127.0.0.1:<INT_SERVER_PORT>

usage (run through run_integrated.sh, which sources ROS 2 + the isolated Open-RMF environment):
  run_integrated.sh up        start every process and leave it running (dashboard address printed)
  run_integrated.sh down      stop ONLY the processes registered by `up`, then check for leftovers
  run_integrated.sh all       up -> scenarios -> down, results in <run_dir>/results (one click)
  options: --run-dir DIR  --scenarios baseline,obstacle,person,video_off,faults  --repeats N

SIMULATION ONLY: no real LIMO, no motor, no Tailscale/Wi-Fi change, port 8080 untouched, Safety Gate neither modified nor bypassed.
"""
import argparse, json, math, os, secrets, socket, subprocess, sys, threading, time, hashlib, signal
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from integrated_env import *   # noqa
HERE = os.path.dirname(os.path.abspath(__file__)); TESTS = os.path.join(FLEET, 'rmf_adapter', 'tests')
sys.path.insert(0, TESTS); sys.path.insert(0, os.path.join(FLEET, 'rmf_adapter'))
import urllib.request, urllib.error

RUN = None; SINK = None; RMFC = None; RES = {}
FLEET_JSON = os.path.join(HERE, 'fleet_integrated.json')
MAP_TO_WORLD = lambda mx, my: (-(my + 1.1), mx - 0.6)           # estimate (see integrated_sim.launch.py); AMCL corrects the residual
BOX_MAP = (-2.0, -4.95); BOX_HALF = 0.25                           # centre of the lane wp_b -> wp_c in the saved-map frame


# ------------------------------------------------------------------ REST helpers
def tok(n): return open(RUN.sec / (n + '.token')).read().strip()


def http(method, path, token=None, body=None, timeout=6, base=None):
    req = urllib.request.Request((base or BASE) + path, method=method, data=json.dumps(body).encode() if body is not None else None, headers={'Content-Type': 'application/json'})
    if token: req.add_header('Authorization', 'Bearer ' + token)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r: d = r.read(); return r.status, (json.loads(d) if d else None)
    except urllib.error.HTTPError as e:
        d = e.read(); return e.code, (json.loads(d) if d else None)
    except (urllib.error.URLError, ConnectionError, TimeoutError, OSError): return 0, None


def wait(fn, timeout, step=0.3):
    end = time.time() + timeout
    while time.time() < end:
        v = fn()
        if v: return v
        time.sleep(step)
    return None


def robot(): return (http('GET', '/api/v1/robots/pcu', tok('viewer')) or (0, None))[1]
def missions(): return (http('GET', '/api/v1/missions?robot_id=pcu&limit=300', tok('viewer'))[1] or {}).get('missions', [])
def ai_status(): return (http('GET', '/api/v1/status', open(RUN.sec / 'video_viewer.token').read().strip(), base=f"http://127.0.0.1:{CFG['AI_HTTP_PORT']}", timeout=3) or (0, None))[1]


def evidence():
    try:
        with open(RUN.dir / 'gazebo_evidence.jsonl', 'rb') as f:
            f.seek(0, 2); f.seek(max(0, f.tell() - 65536)); lines = f.read().splitlines()
        for line in reversed(lines):
            try: return json.loads(line)
            except ValueError: continue
    except OSError: pass
    return {}


# ------------------------------------------------------------------ startup of the components
def init_server():
    sd = os.path.join(FLEET, 'server')
    subprocess.run([sys.executable, '-m', 'fleet_server', 'init', '--data-dir', str(RUN.data), '--secrets-dir', str(RUN.sec), '--robots', 'pcu'], cwd=sd, env=dict(os.environ, PYTHONPATH=sd), check=True, capture_output=True)
    ap = RUN.data / 'auth.json'; a = json.load(open(ap)); t = 'rmf-' + hashlib.sha256(os.urandom(16)).hexdigest()
    a['tokens'].append({'name': 'rmf-adapter', 'role': 'operator', 'robot_id': None, 'sha256': hashlib.sha256(t.encode()).hexdigest()})
    for fn, val in ((str(ap) + '.new', json.dumps(a)),):
        fd = os.open(fn, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600); os.write(fd, val.encode()); os.close(fd)
    os.replace(str(ap) + '.new', ap)
    for name, val in (('rmf-adapter', t), ('ai_sender', secrets.token_hex(24)), ('video_viewer', secrets.token_hex(24))):
        fd = os.open(RUN.sec / (name + '.token'), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600); os.write(fd, (val + '\n').encode()); os.close(fd)


def start_server():
    RUN.spawn('server', f"cd {FLEET}/server && exec python3 -m fleet_server serve --data-dir {RUN.data} --port {CFG['SERVER_PORT']} --offline-after 4 --video-origin http://127.0.0.1:{CFG['AI_HTTP_PORT']}", 'server')
    return wait(lambda: http('GET', '/api/v1/health')[0] == 200, 15)


def start_gazebo():
    RUN.spawn('gazebo', f"exec ros2 launch {HERE}/integrated_sim.launch.py", 'robot')
    RUN.spawn('observer', f"exec python3 {FLEET}/gazebo/observe.py {RUN.dir}/gazebo_evidence.jsonl", 'robot')
    def ready():
        if not RUN.alive('gazebo'): raise RuntimeError('Gazebo launch exited; see logs/gazebo.log')
        e = evidence(); return e.get('gt') and e.get('pose')
    assert wait(ready, 120), 'Gazebo/AMCL not ready'
    gl = lambda: (RUN.logs / 'gazebo.log').read_text(errors='ignore')
    assert wait(lambda: 'Managed nodes are active' in gl() and 'lifecycle_manager_navigation' in gl(), 120), 'Nav2 lifecycle not active'
    assert wait(lambda: evidence().get('gate', {}).get('state') == 'NORMAL', 60), 'Safety Gate not NORMAL (no reset is ever called; no goal is sent)'


def start_agent():
    RUN.spawn('agent', f"exec python3 -m limo_robot_agent.agent_node --ros-args -p server_url:={BASE} -p token_file:={RUN.sec}/agent-pcu.token -p robot_id:=pcu -p mode:=sim -p simulated:=true "
              f"-p allow_missions:=true -p allow_sim_safety_gate:=true -p use_sim_time:=true -p state_dir:={RUN.dir}/agent_state", 'robot')


def start_rmf():
    RUN.spawn('schedule', 'exec ros2 run rmf_traffic_ros2 rmf_traffic_schedule', 'rmf')
    RUN.spawn('dispatcher', 'exec ros2 run rmf_task_ros2 rmf_task_dispatcher --ros-args -p use_unique_hex_string_with_task_id:=false', 'rmf')


def start_adapter():
    RUN.spawn('adapter', f"exec python3 -u -m limo_rmf_adapter.adapter_node --config {FLEET_JSON} --server-url {BASE} --token-file {RUN.sec}/rmf-adapter.token --state-dir {RUN.dir}/adapter_state "
              f"--rmf-api-uri ws://127.0.0.1:{CFG['WS_PORT']}", 'rmf')


def start_ai_host():
    RUN.spawn('ai_host', f"exec {CFG['YOLO_PY']} {CFG['VIDEO_HOME']}/server/ai_server.py --host 127.0.0.1 --port {CFG['AI_LINK_PORT']} --http-port {CFG['AI_HTTP_PORT']} --sender-token-file {RUN.sec}/ai_sender.token "
              f"--viewer-token-file {RUN.sec}/video_viewer.token --weights {CFG['WEIGHTS']} --cors-origin {BASE} --log-jsonl {RUN.dir}/ai_host_frames.jsonl --trust-sender-clock 1", 'ai')
    return wait(lambda: 'link tcp' in (RUN.logs / 'ai_host.log').read_text(errors='ignore') if (RUN.logs / 'ai_host.log').exists() else False, 90)


IMG = '/camera/color/image_raw'


def start_video_robot_side():
    """proxy (fault injection point) + JPEG republisher + sender + consumer. The sender talks to the proxy; with an all-zero control file the proxy is transparent."""
    (RUN.dir / 'proxy_control.json').write_text('{}')
    RUN.spawn('proxy', f"exec python3 {CFG['VIDEO_HOME']}/tools/chaos_proxy.py --listen {CFG['PROXY_PORT']} --target 127.0.0.1:{CFG['AI_LINK_PORT']} --control {RUN.dir}/proxy_control.json", 'plain')
    RUN.spawn('republish', f"exec ros2 run image_transport republish raw compressed --ros-args --remap in:={IMG} --remap out/compressed:={IMG}/compressed", 'robot')
    start_sender(); start_consumer()


def start_sender(fps=10.0):
    RUN.spawn('sender', f"exec python3 {CFG['VIDEO_HOME']}/nodes/sender_node.py --ros-args -p robot_id:=pcu -p image_topic:={IMG}/compressed -p compressed:=true -p server_host:=127.0.0.1 "
              f"-p server_port:={CFG['PROXY_PORT']} -p token_file:={RUN.sec}/ai_sender.token -p fps:={fps:.3f} -p max_frame_age_s:=0.5 -p max_result_age_s:=1.5", 'robot')


def start_consumer():
    RUN.spawn('consumer', f"exec python3 {CFG['VIDEO_HOME']}/nodes/ai_consumer_node.py --image-topic {IMG}/compressed --compressed --max-detection-age-s 1.5 --out {RUN.dir}/consumer.jsonl", 'robot')


def proxy_set(**kw): (RUN.dir / 'proxy_control.json').write_text(json.dumps(kw))


# ------------------------------------------------------------------ background samplers
class Samplers:
    def __init__(self):
        self.stop = False; self.rows = []; self.pcache = {}
        threading.Thread(target=self.sysmon, daemon=True).start(); threading.Thread(target=self.aipoll, daemon=True).start()

    def gpu(self):
        try:
            u, m = subprocess.check_output(['nvidia-smi', '--query-gpu=utilization.gpu,memory.used', '--format=csv,noheader,nounits'], timeout=3).decode().split(','); return float(u), float(m)
        except Exception: return None, None

    def sysmon(self):
        f = open(RUN.dir / 'system.jsonl', 'a'); psutil.cpu_percent(None)
        while not self.stop:
            t = time.time(); g = self.gpu(); per = {}
            for n, o in list(RUN.owned.items()):
                tot = 0.0; rss = 0
                for pid in RUN._members(n):
                    try:
                        p = self.pcache.setdefault(pid, psutil.Process(pid)); tot += p.cpu_percent(None); rss += p.memory_info().rss
                    except psutil.Error: self.pcache.pop(pid, None)
                per[n] = [round(tot, 1), round(rss / 1e6)]
            vm = psutil.virtual_memory(); ev = evidence()
            f.write(json.dumps(dict(t=t, cpu=psutil.cpu_percent(None), ram_used_mb=round((vm.total - vm.available) / 1e6), gpu_util=g[0], gpu_mem_mb=g[1], load1=os.getloadavg()[0], sim=ev.get('sim'), wall=ev.get('wall'),
                                    gate=(ev.get('gate') or {}).get('state'), per_proc=per)) + '\n'); f.flush()
            time.sleep(max(0, 1 - (time.time() - t)))

    def aipoll(self):
        f = open(RUN.dir / 'ai_status.jsonl', 'a'); last_seq = None
        while not self.stop:
            t = time.time(); s = ai_status()
            if s:
                f.write(json.dumps({'t': t, 'status': s}) + '\n'); f.flush()
            time.sleep(max(0, 0.25 - (time.time() - t)))


def window(path, t0, t1, key='t'):
    out = []
    try:
        for line in open(path):
            try: r = json.loads(line)
            except ValueError: continue
            if t0 <= r.get(key, 0) <= t1: out.append(r)
    except OSError: pass
    return out


def pct(v, q):
    v = sorted(x for x in v if x is not None)
    return None if not v else round(v[min(len(v) - 1, int(q * len(v)))], 2)


def avg(v):
    v = [x for x in v if x is not None]; return round(sum(v) / len(v), 2) if v else None


def collect_stats(t0, t1):
    """system + video/AI metrics of the wall window [t0, t1]"""
    sysr = window(RUN.dir / 'system.jsonl', t0, t1); ai = window(RUN.dir / 'ai_status.jsonl', t0, t1)
    out = dict(window_s=round(t1 - t0, 1), samples=len(sysr))
    if sysr:
        sw = [(r['sim'], r['wall']) for r in sysr if r.get('sim') and r.get('wall')]
        out['rtf'] = round((sw[-1][0] - sw[0][0]) / (sw[-1][1] - sw[0][1]), 3) if len(sw) > 1 and sw[-1][1] > sw[0][1] else None
        out['cpu_pct_avg'] = avg([r['cpu'] for r in sysr]); out['cpu_pct_max'] = max(r['cpu'] for r in sysr); out['ram_mb_max'] = max(r['ram_used_mb'] for r in sysr)
        out['gpu_util_avg'] = avg([r['gpu_util'] for r in sysr]); out['gpu_util_max'] = max([r['gpu_util'] for r in sysr if r['gpu_util'] is not None] or [None]) if any(r['gpu_util'] is not None for r in sysr) else None
        out['vram_mb_max'] = max([r['gpu_mem_mb'] for r in sysr if r['gpu_mem_mb'] is not None] or [None]) if any(r['gpu_mem_mb'] is not None for r in sysr) else None
        out['gate_states'] = {s: sum(1 for r in sysr if r.get('gate') == s) for s in {r.get('gate') for r in sysr}}
        procs = {}
        for r in sysr:
            for n, (c, _m) in r['per_proc'].items(): procs.setdefault(n, []).append(c)
        out['proc_cpu_pct_of_1core_avg'] = {n: avg(v) for n, v in procs.items()}
    if ai:
        s0, s1 = ai[0]['status'], ai[-1]['status']; ss = [a['status'] for a in ai]
        out['ai'] = dict(samples=len(ai), states={s: sum(1 for x in ss if x.get('ai_state') == s) for s in {x.get('ai_state') for x in ss}},
                         rx_fps_avg=avg([x.get('rx_fps') for x in ss if x.get('ai_state') == 'RECEIVING']), infer_ms_avg=avg([x.get('infer_ms_avg') for x in ss]), infer_ms_p95_max=max([x.get('infer_ms_p95') or 0 for x in ss]),
                         frames_delta=(s1.get('frames_total') or 0) - (s0.get('frames_total') or 0), results_delta=(s1.get('results_ok') or 0) - (s0.get('results_ok') or 0),
                         dropped_stale_delta=(s1.get('dropped_stale') or 0) - (s0.get('dropped_stale') or 0), dropped_overwritten_delta=(s1.get('dropped_overwritten') or 0) - (s0.get('dropped_overwritten') or 0),
                         detection_fps=round(((s1.get('results_ok') or 0) - (s0.get('results_ok') or 0)) / max(1e-6, ai[-1]['t'] - ai[0]['t']), 2), reconnects_delta=(s1.get('reconnects') or 0) - (s0.get('reconnects') or 0))
        tel = [x.get('robot_telemetry') for x in ss if x.get('robot_telemetry')]
        if tel: out['robot_side'] = dict(rtt_ms_avg=avg([t.get('rtt_ms_avg') for t in tel]), e2e_ms_avg=avg([t.get('e2e_ms_avg') for t in tel]), src_fps=avg([t.get('src_fps') for t in tel]), result_fps=avg([t.get('result_fps') for t in tel]),
                                         stale_blocked=(tel[-1].get('counters') or {}).get('stale_results') if tel else None, link=tel[-1].get('robot_ai_link'))
        # detections: every distinct result seen on the status API (class / confidence / bbox / original camera stamp)
        seen = {}; cls = {}
        for a in ai:
            lr = a['status'].get('last_result')
            if lr and lr.get('seq') not in seen:
                seen[lr['seq']] = lr
                for d in lr.get('detections', []): cls.setdefault(d['cls'], []).append(d['conf'])
        out['detections'] = {c: dict(count=len(v), conf_mean=avg(v), conf_max=max(v)) for c, v in cls.items()}; out['distinct_results_sampled'] = len(seen)
        out['detection_examples'] = [dict(seq=s, stamp_ns=r.get('stamp_ns'), frame_id=r.get('frame_id'), detections=r.get('detections')[:4]) for s, r in list(seen.items())[:: max(1, len(seen) // 4)][:4]] if seen else []
    fr = window(RUN.dir / 'ai_host_frames.jsonl', t0, t1)
    if fr: out['ai_host_frames'] = dict(n=len(fr), infer_ms_avg=avg([r.get('infer_ms') for r in fr]), infer_ms_p95=pct([r.get('infer_ms') for r in fr], .95), server_total_ms_avg=avg([r.get('server_total_ms') for r in fr]), server_total_ms_p95=pct([r.get('server_total_ms') for r in fr], .95),
                                          transport_ms_avg=avg([r.get('transport_ms') for r in fr]))
    cons = window(RUN.dir / 'consumer.jsonl', t0, t1)
    if cons:
        rt = [r.get('roundtrip_ms') for r in cons if 'roundtrip_ms' in r]; sm = [r['summary'] for r in cons if 'summary' in r]
        out['consumer'] = dict(detection_msgs=len(rt), send_to_result_ms_avg=avg(rt), send_to_result_ms_p95=pct(rt, .95), states={s: sum(1 for x in sm if x.get('detection_state') == s) for s in {x.get('detection_state') for x in sm}},
                               links={s: sum(1 for x in sm if x.get('ai_link') == s) for s in {x.get('ai_link') for x in sm}})
    return out


# ------------------------------------------------------------------ RMF tasks and scenario machinery
def rmf_task(place, timeout=240, tick=None, quiet=False):
    """dispatch a patrol task through the standard RMF task API, follow it to a terminal state (RMF's own task_state_update)"""
    t0 = time.time(); sim0 = evidence().get('sim'); base_m = {m['mission_id'] for m in missions()}
    r = RMFC.dispatch_patrol([place]); tid = ((r or {}).get('state') or {}).get('booking', {}).get('id')
    if not tid: return dict(task_id=None, status='NOT_ACCEPTED', response=str(r)[:200])
    wait(lambda: (lambda d: d if d and d.assignment.is_assigned else None)(RMFC.dispatch_state(tid)), 25)
    end = time.time() + timeout; st = None
    while time.time() < end:
        st = SINK.status(tid)
        if st in ('completed', 'failed', 'canceled', 'killed'): break
        if tick: tick()
        time.sleep(0.25)
    sim1 = evidence().get('sim'); ms = [m for m in missions() if m['mission_id'] not in base_m]
    return dict(task_id=tid, place=place, status=st, wall_s=round(time.time() - t0, 1), sim_s=round(sim1 - sim0, 1) if sim0 and sim1 else None, wall_t0=t0, wall_t1=time.time(),
                missions=[dict(id=m['mission_id'], status=m['status'], created_by=m['created_by'], goal=[round(m['goal']['x'], 2), round(m['goal']['y'], 2)]) for m in ms])


def at_wp(name, tol=0.5):
    nav = NAV[name]; p = (((robot() or {}).get('state') or {}).get('pose') or {})
    return p.get('status') == 'OK' and math.hypot(p['x'] - nav[0], p['y'] - nav[1]) < tol


def ensure_at(name):
    if at_wp(name): return True
    r = rmf_task(name, 200); return r['status'] == 'completed' and bool(wait(lambda: at_wp(name), 10))


def metrics_start(tag, boxes=()):
    args = ' '.join(f'--box {x} {y} {h}' for x, y, h in boxes)
    RUN.spawn('metrics', f"exec python3 {HERE}/tools/sim_metrics.py {RUN.results}/{tag} {args}", 'robot')
    time.sleep(2.5)


def metrics_stop(tag):
    RUN.kill('metrics', grace=6)
    try: return json.load(open(RUN.results / f'{tag}_summary.json'))
    except OSError: return None


def trajectory(tag):
    rows = []
    try:
        for l in open(RUN.results / f'{tag}.jsonl'):
            try: rows.append(json.loads(l))
            except ValueError: pass
    except OSError: pass
    return rows


def world_line(a, b):
    return MAP_TO_WORLD(*NAV[a]), MAP_TO_WORLD(*NAV[b])


def lateral_dev(rows, a, b):
    (ax, ay), (bx, by) = world_line(a, b); L = math.hypot(bx - ax, by - ay); out = []
    for r in rows:
        g = r.get('gt')
        if g: out.append(((g[0] - ax) * (by - ay) - (g[1] - ay) * (bx - ax)) / L)
    return (round(max(out, key=abs), 3) if out else None)


def nav2_goal_counts():
    gs = evidence().get('goals', {}); return {'total': len(gs), 'succeeded': sum(1 for g in gs.values() if g['status'] == 4), 'canceled': sum(1 for g in gs.values() if g['status'] == 5), 'aborted': sum(1 for g in gs.values() if g['status'] == 6)}


def scenario_common(tag, place, boxes=(), tick_factory=None, timeout=240, setup=None, teardown=None):
    ensure_at('wp_a'); time.sleep(2)
    g0 = nav2_goal_counts(); w0 = time.time(); metrics_start(tag, boxes)
    ctx = {}
    if setup: setup(ctx)
    r = rmf_task(place, timeout, tick=tick_factory(ctx) if tick_factory else None)
    time.sleep(1.5); summ = metrics_stop(tag); w1 = time.time()
    if teardown: teardown(ctx)
    g1 = nav2_goal_counts(); rows = trajectory(tag)
    res = dict(tag=tag, rmf=r, metrics=summ, nav2_goals_delta={k: g1[k] - g0[k] for k in g0}, stats=collect_stats(w0, w1), ctx={k: v for k, v in ctx.items() if k != 'obj'})
    json.dump(res, open(RUN.results / f'{tag}.json', 'w'), indent=1, default=str); RES[tag] = res
    return res, rows


def verdict(name, ok, detail=''):
    RES.setdefault('_checks', []).append((name, bool(ok), detail)); RUN.say(f"  [{'PASS' if ok else 'FAIL'}] {name} {detail}")


# ------------------------------------------------------------------ scenarios
def sc_baseline(video, tag):
    res, rows = scenario_common(tag, 'wp_c')
    m = res['metrics'] or {}
    res['summary'] = dict(rmf_status=res['rmf']['status'], wall_s=res['rmf'].get('wall_s'), sim_s=res['rmf'].get('sim_s'), distance_m=m.get('distance_m'), collisions=m.get('collisions'), depth_obstacle_points_max=m.get('depth_obstacle_points_max'), nav2=res['nav2_goals_delta'], video=video,
                          gate_states=m.get('gate_states'), rtf=res['stats'].get('rtf'), cpu_avg=res['stats'].get('cpu_pct_avg'), cpu_max=res['stats'].get('cpu_pct_max'))
    json.dump(res, open(RUN.results / f'{tag}.json', 'w'), indent=1, default=str); return res


def sc_obstacle():
    wx, wy = MAP_TO_WORLD(*BOX_MAP); tag = 'obstacle'
    py = f"{HERE}/tools/scenario_ctl.py"; envp = RUN.env('robot')

    def sc(*a): return subprocess.run([sys.executable, py, *map(str, a)], env=envp, capture_output=True, text=True, timeout=30)

    def setup(ctx): ctx['box_world'] = [round(wx, 3), round(wy, 3)]; ctx['spawned'] = None; sc('remove', 'obstacle_box')

    def tick_factory(ctx):
        def tick():
            if ctx['spawned'] is not None: return
            g = evidence().get('gt'); ms = [m for m in missions() if m['status'] == 'EXECUTING']
            if g and ms and math.hypot(g[0] - wx, g[1] - wy) < 1.5:
                r = sc('spawn-box', 'obstacle_box', wx, wy); ctx['spawned'] = dict(wall=time.time(), sim=evidence().get('sim'), robot_gt=[round(v, 2) for v in g], dist_to_box=round(math.hypot(g[0] - wx, g[1] - wy), 2), ok='true' in r.stdout.lower())
        return tick

    def teardown(ctx): ctx['removed'] = 'true' in sc('remove', 'obstacle_box').stdout.lower()
    res, rows = scenario_common(tag, 'wp_c', boxes=[(wx, wy, BOX_HALF)], tick_factory=tick_factory, setup=setup, teardown=teardown, timeout=300)
    m = res['metrics'] or {}; ctx = res['ctx']; sp = ctx.get('spawned') or {}
    after = [r for r in rows if sp and r['wall'] >= sp['wall']]
    loc = max([(r['costmap'] or {}).get('local_box0', -1) for r in after] or [-1]); glo = max([(r['costmap'] or {}).get('global_box0', -1) for r in after] or [-1])
    plans = json.load(open(RUN.results / f'{tag}_plans.json')) if os.path.exists(RUN.results / f'{tag}_plans.json') else []
    before = [p['len'] for p in plans if sp and p['t'] < sp['wall']]; aft = [p['len'] for p in plans if sp and p['t'] >= sp['wall']]
    dev = lateral_dev(rows, 'wp_b', 'wp_c')
    res['summary'] = dict(rmf_status=res['rmf']['status'], box_spawned_after_motion_start=bool(sp and sp.get('ok')), spawn=sp, local_costmap_max_near_box_after_spawn=loc, global_costmap_max_near_box_after_spawn=glo,
                          lidar_scan_min_m=m.get('scan_min_all_m'), depth_obstacle_points_max=m.get('depth_obstacle_points_max'), plan_len_before_m=before[-1] if before else None, plan_len_after_max_m=max(aft) if aft else None, plan_changes_after_spawn=len(aft), lateral_deviation_from_straight_b_to_c_m=dev,
                          min_clearance_robot_centre_to_box_surface_m=(m.get('min_clearance_to_box_m') or {}).get('0'), collisions=m.get('collisions'), distance_m=m.get('distance_m'), wall_s=res['rmf'].get('wall_s'), sim_s=res['rmf'].get('sim_s'),
                          nav2=res['nav2_goals_delta'], gate_states=m.get('gate_states'), box_removed=ctx.get('removed'))
    json.dump(res, open(RUN.results / f'{tag}.json', 'w'), indent=1, default=str)
    s = res['summary']
    verdict('obstacle appeared only after the mission was EXECUTING (motion started)', s['box_spawned_after_motion_start'], str(sp))
    verdict('obstacle visible in the costmaps (global/local max cost >= 99 near the box)', max(loc, glo) >= 99, f'local={loc} global={glo}')
    verdict('path changed (plan re-computed after spawn and/or lateral deviation > 0.15 m)', bool(aft) and (dev is not None and abs(dev) > 0.15 or (before and max(aft) > before[-1] + 0.1)), f'dev={dev} before={before[-1:] } after_max={max(aft) if aft else None}')
    verdict('no collision (robot centre never within chassis radius of the box)', (s['collisions'] or 0) == 0, f"min clearance {s['min_clearance_robot_centre_to_box_surface_m']}")
    verdict('RMF task completed after the avoidance', s['rmf_status'] == 'completed')
    return res


def sc_person():
    tag = 'person'; py = f"{HERE}/tools/scenario_ctl.py"; envp = RUN.env('robot')
    def sc(*a): return subprocess.run([sys.executable, py, *map(str, a)], env=envp, capture_output=True, text=True, timeout=30)
    (bx, by), (cx, cy) = world_line('wp_b', 'wp_c'); mx_, my_ = (bx + cx) / 2, (by + cy) / 2; L = math.hypot(cx - bx, cy - by); ux, uy = (cx - bx) / L, (cy - by) / L; px, py_ = -uy, ux   # perpendicular
    sx, sy = mx_ + 1.6 * px, my_ + 1.6 * py_; yaw = math.atan2(-py_, -px)

    def setup(ctx):
        ctx['person_start_world'] = [round(sx, 2), round(sy, 2)]; ctx['spawned'] = None; sc('remove', 'person_walker'); sc('remove', 'scenario_chair')
        cx_, cy_ = mx_ - 1.1 * px + 0.5 * ux, my_ - 1.1 * py_ + 0.5 * uy; ctx['chair_world'] = [round(cx_, 2), round(cy_, 2)]; ctx['chair_spawned'] = 'true' in sc('spawn-chair', cx_, cy_, 0.0).stdout.lower()

    def tick_factory(ctx):
        def tick():
            g = evidence().get('gt')
            if ctx['spawned'] is None and g and any(m['status'] == 'EXECUTING' for m in missions()) and math.dist(g[:2], (bx, by)) > 0.5:
                r = sc('spawn-person', sx, sy, yaw); time.sleep(0.5); sc('move-person', 0.5); ctx['spawned'] = dict(wall=time.time(), robot_gt=[round(v, 2) for v in g], ok='true' in r.stdout.lower())
            elif ctx['spawned'] and time.time() - ctx['spawned']['wall'] > 6.0 and not ctx['spawned'].get('stopped'):
                sc('stop-person'); ctx['spawned']['stopped'] = True
        return tick

    def teardown(ctx): sc('stop-person'); ctx['removed'] = 'true' in sc('remove', 'person_walker').stdout.lower(); sc('remove', 'scenario_chair')
    res, rows = scenario_common(tag, 'wp_c', tick_factory=tick_factory, setup=setup, teardown=teardown, timeout=300)
    m = res['metrics'] or {}; sp = res['ctx'].get('spawned') or {}
    res['summary'] = dict(rmf_status=res['rmf']['status'], person_spawned_after_motion_start=bool(sp.get('ok')), min_distance_to_person_m=m.get('min_distance_to_person_m'), collisions=m.get('collisions'), distance_m=m.get('distance_m'),
                          wall_s=res['rmf'].get('wall_s'), nav2=res['nav2_goals_delta'], gate_states=m.get('gate_states'), yolo=res['stats'].get('detections'), person_removed=res['ctx'].get('removed'))
    json.dump(res, open(RUN.results / f'{tag}.json', 'w'), indent=1, default=str)
    verdict('moving person spawned after motion start and RMF task completed', res['summary']['person_spawned_after_motion_start'] and res['summary']['rmf_status'] == 'completed', str(sp))
    verdict('YOLO detected class chair (static scene object)', 'chair' in (res['stats'].get('detections') or {}), str(res['stats'].get('detections')))
    verdict('YOLO detected class person while the robot was moving', 'person' in (res['stats'].get('detections') or {}), str(res['stats'].get('detections')))
    return res


def fault_log(events, name, **kw): events.append(dict(t=time.time(), sim=evidence().get('sim'), event=name, **kw)); RUN.say(f'  FAULT {name} {kw}')


def sc_faults():
    """one long RMF patrol (several legs) while faults are injected one after the other"""
    tag = 'faults'; ensure_at('wp_a'); time.sleep(2); ev = []; w0 = time.time(); metrics_start(tag); g0 = nav2_goal_counts()
    gate_bad = []; motion = []

    def sample():
        e = evidence(); gate_bad.append((e.get('gate') or {}).get('state')); g = e.get('gt'); motion.append(g[:2] if g else None)

    def hold(seconds):
        end = time.time() + seconds
        while time.time() < end: sample(); time.sleep(0.5)
    # a long task: patrol over several places is produced by repeating dispatches until the injection program is over
    results = []; done = threading.Event()

    def run_tasks():
        for place in ['wp_c', 'wp_d', 'wp_a', 'wp_c', 'wp_a']:
            if done.is_set(): break
            results.append(rmf_task(place, 200))
    th = threading.Thread(target=run_tasks, daemon=True); th.start(); time.sleep(8)
    pre_mission = len(missions())
    # 1 sender stop
    fault_log(ev, 'video_sender_stopped'); RUN.kill('sender', grace=4); hold(7); a = ai_status(); fault_log(ev, 'video_sender_state_during_stop', ai_state=(a or {}).get('ai_state'))
    start_sender(); hold(8); a = ai_status(); fault_log(ev, 'video_sender_restarted', ai_state=(a or {}).get('ai_state'))
    # 2 AI host restart
    fault_log(ev, 'ai_host_killed'); RUN.kill('ai_host', grace=4); hold(6); fault_log(ev, 'ai_host_restarting'); start_ai_host(); hold(12); a = ai_status(); fault_log(ev, 'ai_host_back', ai_state=(a or {}).get('ai_state'))
    # 3 delay, 4 loss, 5 silent stall (all inside the proxy; no OS network setting is touched)
    proxy_set(delay_ms=1500); fault_log(ev, 'video_delay_1500ms'); hold(8); proxy_set(); hold(5)
    proxy_set(frame_drop_prob=0.7); fault_log(ev, 'frame_loss_70pct'); hold(8); proxy_set(); hold(5)
    proxy_set(blackhole=True); fault_log(ev, 'link_blackhole'); hold(7); proxy_set(); fault_log(ev, 'link_blackhole_released'); hold(8)
    # 6 mission manager connection fault
    fault_log(ev, 'mission_manager_killed'); RUN.kill('server', grace=4); hold(8); r = robot(); fault_log(ev, 'mission_manager_down_probe', http=http('GET', '/api/v1/health')[0]); ok = start_server(); fault_log(ev, 'mission_manager_restarted', up=bool(ok)); hold(10)
    th.join(timeout=240); done.set(); summ = metrics_stop(tag); w1 = time.time()
    st = collect_stats(w0, w1); gc = nav2_goal_counts()
    # robot kept moving during the video faults? use GT positions sampled during the video-fault part
    gates = {s: gate_bad.count(s) for s in set(gate_bad)}
    res = dict(tag=tag, events=ev, tasks=results, metrics=summ, stats=st, gate_during_run=gates, nav2_goals_delta={k: gc[k] - g0[k] for k in g0}, missions_before_faults=pre_mission)
    json.dump(res, open(RUN.results / f'{tag}.json', 'w'), indent=1, default=str); RES[tag] = res
    # per-window gate states from the 1 Hz system samples (video faults: from the first injection until the last video fault was released; server outage: its own window)
    def gate_in(t0_, t1_): return sorted({r.get('gate') for r in window(RUN.dir / 'system.jsonl', t0_, t1_)} - {None})
    te = {e['event']: e['t'] for e in ev}
    vwin = gate_in(te['video_sender_stopped'], te['link_blackhole_released']); swin = gate_in(te['mission_manager_killed'], te['mission_manager_restarted'] + 10)
    res['gate_video_fault_window'] = vwin; res['gate_server_outage_window'] = swin; json.dump(res, open(RUN.results / f'{tag}.json', 'w'), indent=1, default=str)
    verdict('Safety Gate stayed NORMAL during every injected VIDEO fault (sender stop, AI host restart, delay, loss, stall)', vwin == ['NORMAL'], str(vwin))
    verdict('Safety Gate never entered a failsafe/blocked state during the whole fault run incl. the server outage (STALL_PENDING = its normal stall pre-warning, also seen without any fault)', set(gates) <= {'NORMAL', 'STALL_PENDING', None} and set(swin) <= {'NORMAL', 'STALL_PENDING'}, f'whole run {gates}; outage window {swin}')
    verdict('all RMF tasks of the fault run ended completed', bool(results) and all(r['status'] == 'completed' for r in results), str([r['status'] for r in results]))
    verdict('no Nav2 goal aborted during the fault run', res['nav2_goals_delta']['aborted'] == 0, str(res['nav2_goals_delta']))
    return res


# ------------------------------------------------------------------ main
def up(args):
    global RUN
    RUN.say(f'run dir {RUN.dir}')
    bad = preflight()
    if bad:
        for b in bad: RUN.say('PREFLIGHT FAIL: ' + b)
        sys.exit(2)
    RUN.say('preflight ok (ports free, ROS domains empty, tools present)')
    init_server(); assert start_server(), 'Mission Manager did not start'; RUN.say(f'Mission Manager up on {BASE}')
    start_rmf(); time.sleep(3)
    start_ai_host() or sys.exit('AI host did not start'); RUN.say('AI host (CUDA YOLO11n) up')
    start_gazebo(); RUN.say('Gazebo + Nav2 + AMCL + Safety Gate ready (gate NORMAL)')
    start_agent(); assert wait(lambda: (lambda r: bool(r and r.get('online') and ((r.get('state') or {}).get('nav2') or {}).get('status') == 'READY'))(robot()), 60), 'agent/robot not online with Nav2 READY'
    RUN.say('Robot Agent online (mode=sim, SIM_GATED)')
    start_video_robot_side(); assert wait(lambda: (ai_status() or {}).get('ai_state') == 'RECEIVING', 60), 'video not RECEIVING at the AI host'
    RUN.say('Video: Gazebo RGB -> sender -> AI host (CUDA) RECEIVING')
    start_adapter()
    Path(RUN.dir / 'DASHBOARD.txt').write_text(f"dashboard: {BASE}/?video=http://127.0.0.1:{CFG['AI_HTTP_PORT']}\nviewer token: {RUN.sec}/viewer.token\noperator token: {RUN.sec}/operator.token\nvideo viewer token: {RUN.sec}/video_viewer.token\n")
    RUN.say(f"dashboard {BASE}/?video=http://127.0.0.1:{CFG['AI_HTTP_PORT']}  (tokens: {RUN.sec}, never printed)")


def connect_rmf():
    global SINK, RMFC, NAV, TF, CFGJ
    import rclpy
    from limo_rmf_adapter.coords import Transform
    from ws_sink import WsSink
    from rmf_client import RmfClient
    CFGJ = json.load(open(FLEET_JSON)); TF = Transform.from_config(CFGJ['transform_rmf_from_nav2']); NAV = {w['name']: TF.rmf_to_nav2(*w['rmf'])[:2] for w in CFGJ['waypoints']}
    SINK = WsSink(CFG['WS_PORT'], str(RUN.dir / 'rmf_api_stream.jsonl'))
    os.environ.update(ROS_DOMAIN_ID=CFG['RMF_DOMAIN'], ROS_LOCALHOST_ONLY='1'); rclpy.init(); RMFC = RmfClient()


def main():
    global RUN
    ap = argparse.ArgumentParser(); ap.add_argument('cmd', choices=['up', 'down', 'status', 'all']); ap.add_argument('--run-dir'); ap.add_argument('--scenarios', default='baseline,obstacle,person,video_off,faults'); ap.add_argument('--repeats', type=int, default=2)
    a = ap.parse_args(); RUN = Run(a.run_dir)
    if a.cmd == 'down':
        RUN.kill_all(); left = RUN.leftovers(); RUN.say('down: leftovers ' + (str(left) if left else 'none')); sys.exit(1 if left else 0)
    if a.cmd == 'status':
        for n, o in RUN.owned.items(): print(n, 'alive' if RUN._members(n) else 'dead'); return
    try:
        up(a)
        if a.cmd == 'up': return
        connect_rmf(); smp = Samplers()
        r = wait(lambda: RMFC.robot(), 40); verdict('fleet limo_fleet / robot pcu registered in RMF and reporting', r is not None)
        names = a.scenarios.split(',')
        if 'baseline' in names:
            RUN.say('== baseline (no obstacle, video ON): RMF -> Gazebo LIMO; NOT counted as avoidance'); [sc_baseline(True, f'baseline_video_on_{i}') for i in range(a.repeats)]
        if 'obstacle' in names: RUN.say('== obstacle avoidance'); sc_obstacle(); ensure_at('wp_a')
        if 'person' in names: RUN.say('== moving person + YOLO'); sc_person(); ensure_at('wp_a')
        if 'video_off' in names:
            RUN.say('== video OFF comparison (sender stopped, AI host idle)'); RUN.kill('sender', grace=4); RUN.kill('consumer', grace=4); [sc_baseline(False, f'baseline_video_off_{i}') for i in range(a.repeats)]
            start_sender(); start_consumer(); wait(lambda: (ai_status() or {}).get('ai_state') == 'RECEIVING', 30)
        if 'faults' in names: RUN.say('== fault injection during a long RMF patrol'); sc_faults()
        smp.stop = True
        json.dump({k: v for k, v in RES.items()}, open(RUN.results / 'all_results.json', 'w'), indent=1, default=str)
    finally:
        if a.cmd == 'all':

            try: SINK.srv.shutdown(socket.SHUT_RDWR)   # the accept thread blocks the listening socket otherwise
            except Exception: pass
            try: SINK.close()
            except Exception: pass
            RUN.kill_all(); left = RUN.leftovers(); RUN.say('shutdown: leftovers ' + (str(left) if left else 'none'))
            checks = RES.get('_checks', []); RUN.say(f"checks: {sum(1 for c in checks if c[1])}/{len(checks)} PASS")
            for n, ok, d in checks:
                if not ok: RUN.say(f'  FAIL {n} {d}')
    os._exit(0)


if __name__ == '__main__':
    main()
