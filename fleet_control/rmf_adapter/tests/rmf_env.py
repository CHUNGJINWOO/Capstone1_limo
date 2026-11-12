"""Isolated test environment for the RMF adapter: its own Mission Manager (port 18090, temp data dir, test tokens), a simulated robot "pcu"
(mock Nav2 + Robot Agent, ROS domain 152) and an RMF stack (traffic schedule + task dispatcher + our adapter, ROS domain 151). Production (8080) is never touched."""
import json, os, signal, subprocess, sys, tempfile, time, urllib.error, urllib.request, hashlib
HERE = os.path.dirname(os.path.abspath(__file__)); RMF = os.path.dirname(HERE); FLEET = os.path.dirname(RMF); REPO = os.path.dirname(FLEET)
PORT = int(os.environ.get('RMF_TEST_PORT', '18090')); BASE = f'http://127.0.0.1:{PORT}'
RMF_PREFIX = os.environ.get('RMF_ISOLATED_ENV', os.path.expanduser('~/rmf_isolated/env.sh'))
MOCK_MAP = os.environ.get('FLEET_MOCK_MAP_YAML', os.path.join(FLEET, 'maps', 'limo_sim_map.yaml'))
T = tempfile.mkdtemp(prefix='rmf_test_'); DATA, SEC = os.path.join(T, 'data'), os.path.join(T, 'secrets')
PROCS = {}; WS_PORT = int(os.environ.get('RMF_TEST_WS_PORT', '18091'))
ENV_RMF = dict(os.environ, ROS_DOMAIN_ID='151', ROS_LOCALHOST_ONLY='1', PYTHONPATH=RMF)
ENV_ROBOT = dict(os.environ, ROS_DOMAIN_ID='152', ROS_LOCALHOST_ONLY='1', PYTHONPATH=os.path.join(FLEET, 'agent', 'limo_robot_agent'))
RES = []


T0 = time.time(); SECTIONS = []; REQ = {}


def say(msg):
    line = f'{time.time() - T0:7.1f}s {msg}'; print(line, flush=True)
    try: open(os.path.join(T, 'progress.log'), 'a').write(line + '\n')
    except Exception: pass


def section(name):
    SECTIONS.append([name, time.time(), None])
    if len(SECTIONS) > 1: SECTIONS[-2][2] = time.time()
    say('== ' + name)


def check(name, ok, detail='', req=None):
    RES.append((name, bool(ok)))
    if req is not None: REQ.setdefault(req, []).append(bool(ok))
    say(f"  [{'PASS' if ok else 'FAIL'}] {name} {detail}")


def tok(n): return open(os.path.join(SEC, n + '.token')).read().strip()


def http(method, path, token=None, body=None, timeout=6):
    req = urllib.request.Request(BASE + path, method=method, data=json.dumps(body).encode() if body is not None else None, headers={'Content-Type': 'application/json'})
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


def spawn(name, cmd, env, ros_rmf=False):
    pre = 'set +u; source /opt/ros/humble/setup.bash; ' + (f'source {RMF_PREFIX}; ' if ros_rmf else f'source {REPO}/capstone_ws/install/setup.bash 2>/dev/null; ')
    p = subprocess.Popen(['bash', '-c', pre + cmd], env=env, stdout=open(os.path.join(T, name + '.log'), 'a'), stderr=subprocess.STDOUT, start_new_session=True, cwd=T)
    PROCS[name] = p; return p


def kill(name, sig=signal.SIGINT):
    p = PROCS.pop(name, None)
    if not p: return
    try: os.killpg(p.pid, sig)
    except Exception: pass
    try: p.wait(8)
    except Exception:
        try: os.killpg(p.pid, signal.SIGKILL)
        except Exception: pass


def init_server():
    env = dict(os.environ, PYTHONPATH=os.path.join(FLEET, 'server'))
    subprocess.run([sys.executable, '-m', 'fleet_server', 'init', '--data-dir', DATA, '--secrets-dir', SEC, '--robots', 'pcu'], cwd=os.path.join(FLEET, 'server'), env=env, check=True, capture_output=True)
    # extra principal for the RMF adapter (operator role, its own name => created_by identifies RMF-owned missions)
    ap = os.path.join(DATA, 'auth.json'); a = json.load(open(ap)); t = 'rmf-' + hashlib.sha256(os.urandom(16)).hexdigest()
    a['tokens'].append({'name': 'rmf-adapter', 'role': 'operator', 'robot_id': None, 'sha256': hashlib.sha256(t.encode()).hexdigest()})
    fd = os.open(ap + '.new', os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600); os.write(fd, json.dumps(a).encode()); os.close(fd); os.replace(ap + '.new', ap)
    fd = os.open(os.path.join(SEC, 'rmf-adapter.token'), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600); os.write(fd, (t + '\n').encode()); os.close(fd)


def start_server(offline_after=4):
    spawn('server', f'cd {FLEET}/server && exec python3 -m fleet_server serve --data-dir {DATA} --port {PORT} --offline-after {offline_after}', dict(os.environ, PYTHONPATH=os.path.join(FLEET, 'server')))
    return wait(lambda: http('GET', '/api/v1/health')[0] == 200, 10)


def start_robot(speed=0.6):
    spawn('mock', f'exec python3 -m limo_robot_agent.mock_robot_node --ros-args -p map_yaml:={MOCK_MAP} -p speed_mps:={speed} -p start_x:=-2.4 -p start_y:=-5.5 -p goal_log:={T}/mock_goals.jsonl', ENV_ROBOT)
    time.sleep(2); start_agent()


def start_agent():
    spawn('agent', f'exec python3 -m limo_robot_agent.agent_node --ros-args -p server_url:={BASE} -p token_file:={SEC}/agent-pcu.token -p robot_id:=pcu -p mode:=mock -p simulated:=true -p allow_missions:=true -p state_dir:={T}/agent_state', ENV_ROBOT)


def start_rmf_base():
    spawn('schedule', 'exec ros2 run rmf_traffic_ros2 rmf_traffic_schedule', ENV_RMF, True)
    spawn('dispatcher', 'exec ros2 run rmf_task_ros2 rmf_task_dispatcher --ros-args -p use_unique_hex_string_with_task_id:=false', ENV_RMF, True)


def start_adapter():
    spawn('adapter', f'exec python3 -u -m limo_rmf_adapter.adapter_node --config {RMF}/config/fleet.json --server-url {BASE} --token-file {SEC}/rmf-adapter.token --state-dir {T}/adapter_state --rmf-api-uri ws://127.0.0.1:{WS_PORT}', ENV_RMF, True)


def robot(): return (http('GET', '/api/v1/robots/pcu', tok('viewer')) or (0, None))[1]
def missions(): return http('GET', '/api/v1/missions?robot_id=pcu&limit=200', tok('viewer'))[1]['missions']


def cleanup():
    for n in list(PROCS): kill(n)
