"""Process / environment helpers of the integrated simulation (Open-RMF x Gazebo x Nav2 x video/YOLO). Simulation only.

Everything this module starts is registered (pid + creation time + process group) in <run_dir>/procs.json; only registered processes are ever
signalled. Nothing is matched by name. Ports, ROS domains and the Ignition partition are isolated and checked for conflicts before the start.
No real LIMO, no Tailscale, no production server (8080) is touched.
"""
import json, os, shlex, signal, socket, subprocess, sys, tempfile, time
from pathlib import Path
import psutil

HERE = os.path.dirname(os.path.abspath(__file__)); FLEET = os.path.dirname(HERE); REPO = os.path.dirname(FLEET)
HOME = os.path.expanduser('~')
CFG = dict(
    REFERENCE_WS=os.environ.get('CAPSTONE_REFERENCE_WS', ''),
    RMF_ENV=os.environ.get('RMF_ISOLATED_ENV', f'{HOME}/rmf_isolated/env.sh'),
    VIDEO_HOME=os.environ.get('VIDEO_AI_HOME', f'{HOME}/.local/share/robot-sim-projects/limo_video_ai_preverify'),
    YOLO_HOME=os.environ.get('YOLO_HOME', f'{HOME}/.local/share/robot-sim-projects/yolo_gazebo_limo'),
    SERVER_PORT=int(os.environ.get('INT_SERVER_PORT', '18200')), WS_PORT=int(os.environ.get('INT_WS_PORT', '18201')),
    AI_LINK_PORT=int(os.environ.get('INT_AI_LINK_PORT', '18210')), AI_HTTP_PORT=int(os.environ.get('INT_AI_HTTP_PORT', '18211')),
    PROXY_PORT=int(os.environ.get('INT_PROXY_PORT', '18212')),
    ROBOT_DOMAIN=os.environ.get('INT_ROBOT_DOMAIN', '181'), RMF_DOMAIN=os.environ.get('INT_RMF_DOMAIN', '182'),
    GPU_RENDER=os.environ.get('INT_GPU_RENDER', '1'),
    FORBIDDEN_PORTS={8080},
)
CFG['YOLO_PY'] = f"{CFG['YOLO_HOME']}/venv/bin/python"; CFG['WEIGHTS'] = f"{CFG['YOLO_HOME']}/weights/yolo11n.pt"
CFG['CHAIR_MESH'] = os.environ.get('INT_CHAIR_MESH', f"{CFG['YOLO_HOME']}/models/Chair/meshes/Chair.obj")
CFG['PERSON_MESH'] = os.environ.get('INT_PERSON_MESH', f"{CFG['YOLO_HOME']}/models/person_standing/meshes/standing.dae")
BASE = f"http://127.0.0.1:{CFG['SERVER_PORT']}"


class Run:
    """One run directory with logs, tokens, a process registry."""

    def __init__(self, run_dir=None):
        base = os.environ.get('INT_RUN_ROOT', f'{HOME}/.local/share/robot-sim-projects/limo_integrated_sim/runs')
        self.dir = Path(run_dir or tempfile.mkdtemp(prefix=time.strftime('%Y%m%d_%H%M%S_'), dir=self._mk(base)))
        self.dir.mkdir(parents=True, exist_ok=True)
        self.logs = self.dir / 'logs'; self.logs.mkdir(exist_ok=True)
        self.sec = self.dir / 'secrets'; self.sec.mkdir(mode=0o700, exist_ok=True)
        self.data = self.dir / 'data'; self.results = self.dir / 'results'; self.results.mkdir(exist_ok=True)
        self.reg_path = self.dir / 'procs.json'
        self.partition = f'limo_integ_{os.getpid()}' if not (self.dir / 'partition').exists() else (self.dir / 'partition').read_text().strip()
        (self.dir / 'partition').write_text(self.partition)
        self.procs = {}; self.owned = json.loads(self.reg_path.read_text()) if self.reg_path.exists() else {}
        self.T0 = time.time()

    @staticmethod
    def _mk(p): os.makedirs(p, exist_ok=True); return p

    def say(self, msg):
        line = f'{time.time() - self.T0:7.1f}s {msg}'; print(line, flush=True)
        with open(self.dir / 'progress.log', 'a') as f: f.write(line + '\n')

    # ---- environments
    def env(self, kind):
        e = dict(os.environ, ROS_LOCALHOST_ONLY='1', PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1')
        e.pop('ROS_DOMAIN_ID', None)
        if kind == 'robot':
            e.update(ROS_DOMAIN_ID=CFG['ROBOT_DOMAIN'], IGN_PARTITION=self.partition, PYTHONPATH=os.path.join(FLEET, 'agent', 'limo_robot_agent'),
                     INT_PERSON_MESH=CFG['PERSON_MESH'], INT_CHAIR_MESH=CFG['CHAIR_MESH'], INT_START_MAP_X='-3.2', INT_START_MAP_Y='-6.3', INT_START_MAP_YAW='0.84')
            if CFG['GPU_RENDER'] == '1':
                e.update(__NV_PRIME_RENDER_OFFLOAD='1', __GLX_VENDOR_LIBRARY_NAME='nvidia', __EGL_VENDOR_LIBRARY_FILENAMES='/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
        elif kind == 'rmf':
            e.update(ROS_DOMAIN_ID=CFG['RMF_DOMAIN'], PYTHONPATH=os.path.join(FLEET, 'rmf_adapter'))
        elif kind == 'server':
            e.update(PYTHONPATH=os.path.join(FLEET, 'server'))
        elif kind == 'ai':
            e.update(YOLO_CONFIG_DIR=f"{CFG['YOLO_HOME']}/ultralytics_config")
            e.pop('PYTHONNOUSERSITE', None)   # the YOLO venv has its own numpy/opencv; it must not see the ROS-side restriction
        return e

    def prefix(self, kind):
        if kind == 'rmf': return f"set +u; source /opt/ros/humble/setup.bash; source {shlex.quote(CFG['RMF_ENV'])}; "
        if kind == 'robot': return f"set +u; source /opt/ros/humble/setup.bash; source {shlex.quote(CFG['REFERENCE_WS'])}/install/setup.bash; "
        return 'set +u; '

    # ---- process registry
    def _save(self): self.reg_path.write_text(json.dumps(self.owned, indent=1))

    def spawn(self, name, cmd, kind, cwd=None):
        p = subprocess.Popen(['bash', '-c', self.prefix(kind) + cmd], env=self.env(kind), stdout=open(self.logs / f'{name}.log', 'a'), stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL, start_new_session=True, cwd=cwd or str(self.dir))
        self.procs[name] = p
        self.owned[name] = {'pid': p.pid, 'pgid': os.getpgid(p.pid), 'created': psutil.Process(p.pid).create_time(), 'cmd': cmd[:160]}
        self._save(); return p

    def _members(self, name):
        """live processes of the registered group (pid, create_time verified when it is the group leader)"""
        o = self.owned.get(name)
        if not o: return []
        try:
            leader = psutil.Process(o['pid'])
            if abs(leader.create_time() - o['created']) > 1e-3: return []
        except psutil.NoSuchProcess:
            leader = None
        out = []
        for p in psutil.process_iter(['pid']):
            try:
                if os.getpgid(p.pid) == o['pgid'] and p.status() != psutil.STATUS_ZOMBIE: out.append(p.pid)
            except (ProcessLookupError, psutil.Error): pass
        return out

    def alive(self, name): return bool(self._members(name))

    def kill(self, name, grace=10.0):
        o = self.owned.get(name)
        if not o: return
        if not self._members(name): self.owned.pop(name, None); self.procs.pop(name, None); self._save(); return
        for sig, wait_s in ((signal.SIGINT, grace), (signal.SIGTERM, 4), (signal.SIGKILL, 3)):
            try: os.killpg(o['pgid'], sig)
            except ProcessLookupError: break
            end = time.time() + wait_s
            while time.time() < end and self._members(name): time.sleep(0.2)
            if not self._members(name): break
        p = self.procs.pop(name, None)
        if p:
            try: p.wait(1)
            except Exception: pass
        self.owned.pop(name, None); self._save()

    def kill_all(self):
        for n in list(self.owned)[::-1]: self.kill(n)

    def leftovers(self):
        left = [n for n in self.owned if self._members(n)]
        ports = [CFG[k] for k in ('SERVER_PORT', 'WS_PORT', 'AI_LINK_PORT', 'AI_HTTP_PORT', 'PROXY_PORT')]
        for pt in ports:
            s = socket.socket(); s.settimeout(0.2)
            if s.connect_ex(('127.0.0.1', pt)) == 0: left.append(f'port:{pt}')
            s.close()
        return left


def port_free(port):
    s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)   # a closing connection (TIME_WAIT) of a previous run is not a conflict
    try: s.bind(('127.0.0.1', port)); return True
    except OSError: return False
    finally: s.close()


def domain_busy(domain, spin=2.5):
    """True when any ROS 2 node is already visible in this domain (localhost-only discovery)."""
    code = ("import rclpy,time;rclpy.init();n=rclpy.create_node('int_probe');t=time.time()\n"
            f"while time.time()-t<{spin}: rclpy.spin_once(n,timeout_sec=.1)\n"
            "print(len([x for x,_ in n.get_node_names_and_namespaces() if x!='int_probe' and not x.startswith('_ros2cli')]))")
    r = subprocess.run(['bash', '-c', f"source /opt/ros/humble/setup.bash; ROS_DOMAIN_ID={domain} ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1 python3 -c \"{code}\""], capture_output=True, text=True, timeout=30)
    try: return int(r.stdout.strip().splitlines()[-1]) > 0
    except (ValueError, IndexError): return True


def preflight():
    """list of problems (empty = ok)"""
    bad = []
    if not CFG['REFERENCE_WS'] or not Path(CFG['REFERENCE_WS'], 'install/setup.bash').is_file(): bad.append('CAPSTONE_REFERENCE_WS must name the built (read-only) reference workspace')
    if not Path(CFG['RMF_ENV']).is_file(): bad.append(f"isolated Open-RMF env not found: {CFG['RMF_ENV']} (fleet_control/rmf_adapter/scripts/install_rmf_isolated.sh)")
    for what, p in (('YOLO python', CFG['YOLO_PY']), ('YOLO weights', CFG['WEIGHTS']), ('video/AI host code', CFG['VIDEO_HOME'] + '/server/ai_server.py'), ('person mesh', CFG['PERSON_MESH'])):
        if not Path(p).exists(): bad.append(f'{what} not found: {p}')
    for k in ('SERVER_PORT', 'WS_PORT', 'AI_LINK_PORT', 'AI_HTTP_PORT', 'PROXY_PORT'):
        if CFG[k] in CFG['FORBIDDEN_PORTS']: bad.append(f'{k}={CFG[k]} is the production port')
        elif not port_free(CFG[k]): bad.append(f'port {CFG[k]} ({k}) is already in use')
    if CFG['ROBOT_DOMAIN'] == CFG['RMF_DOMAIN']: bad.append('robot and RMF ROS domains must differ')
    for d in (CFG['ROBOT_DOMAIN'], CFG['RMF_DOMAIN']):
        if domain_busy(d): bad.append(f'ROS domain {d} already has nodes (another run or experiment): choose INT_ROBOT_DOMAIN/INT_RMF_DOMAIN')
    return bad
