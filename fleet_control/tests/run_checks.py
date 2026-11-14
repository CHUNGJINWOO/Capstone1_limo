#!/usr/bin/env python3
"""End-to-end checks of the stage-1 control server with a simulated robot (mock Nav2) on the laptop. Uses a temporary data dir and port 18080 (never the
real tokens). usage: python3 tests/run_checks.py   (needs ROS 2 Humble installed; no real robot is involved)"""
import json, os, shutil, signal, subprocess, sys, tempfile, time, urllib.error, urllib.request
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); REPO = os.path.dirname(ROOT)
MOCK_MAP = os.environ.get('FLEET_MOCK_MAP_YAML', os.path.join(ROOT, 'maps', 'limo_sim_map.yaml'))
PORT = int(os.environ.get('FLEET_TEST_PORT','18080')); BASE = f'http://127.0.0.1:{PORT}'
T = tempfile.mkdtemp(prefix='fleet_test_'); DATA, SEC = os.path.join(T, 'data'), os.path.join(T, 'secrets')
RES = []; PROCS = []
ENV = dict(os.environ, ROS_DOMAIN_ID=os.environ.get('FLEET_TEST_DOMAIN','141'), ROS_LOCALHOST_ONLY='1', PYTHONPATH=os.path.join(ROOT, 'agent', 'limo_robot_agent'))

def check(name, ok, detail=''):
    RES.append((name, bool(ok))); print(f"  [{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)

def tok(n): return open(os.path.join(SEC, n + '.token')).read().strip()

def http(method, path, token=None, body=None, raw=False):
    req = urllib.request.Request(BASE + path, method=method, data=json.dumps(body).encode() if body is not None else None, headers={'Content-Type': 'application/json'})
    if token: req.add_header('Authorization', 'Bearer ' + token)
    try:
        with urllib.request.urlopen(req, timeout=8) as r: d = r.read(); return r.status, (d if raw else (json.loads(d) if d else None))
    except urllib.error.HTTPError as e:
        d = e.read(); return e.code, (d if raw else (json.loads(d) if d else None))
    except (urllib.error.URLError, ConnectionError, TimeoutError):
        return 0, None

def wait(fn, timeout, step=0.3):
    t = time.time() + timeout
    while time.time() < t:
        v = fn()
        if v: return v
        time.sleep(step)
    return None

def start_server():
    p = subprocess.Popen([sys.executable, '-m', 'fleet_server', 'serve', '--data-dir', DATA, '--port', str(PORT), '--offline-after', '4'], cwd=os.path.join(ROOT, 'server'), env=dict(os.environ, PYTHONPATH=os.path.join(ROOT, 'server')), stdout=open(os.path.join(T, 'server.log'), 'a'), stderr=subprocess.STDOUT)
    PROCS.append(p); wait(lambda: http('GET', '/api/v1/health')[0] == 200 if p.poll() is None else False, 10) if True else None; return p

def ros(cmd, log):
    full = 'set +u; source /opt/ros/humble/setup.bash; source %s/capstone_ws/install/setup.bash 2>/dev/null; %s' % (REPO, cmd)
    p = subprocess.Popen(['bash', '-c', full], env=ENV, stdout=open(os.path.join(T, log), 'a'), stderr=subprocess.STDOUT, start_new_session=True); PROCS.append(p); return p

def kill(p):
    try: os.killpg(p.pid, signal.SIGINT)
    except Exception:
        try: p.send_signal(signal.SIGINT)
        except Exception: pass
    try: p.wait(8)
    except Exception:
        try: os.killpg(p.pid, signal.SIGKILL)
        except Exception: p.kill()

def agent_cmd(rid, mode, sim, allow, port=PORT):
    return (f'python3 -m limo_robot_agent.agent_node --ros-args -p server_url:=http://127.0.0.1:{port} -p token_file:={SEC}/agent-{rid}.token -p robot_id:={rid} -p mode:={mode} '
            f'-p simulated:={sim} -p allow_missions:={allow} -p state_dir:={T}/agent_state_{rid}')

def robot(rid):
    c, d = http('GET', f'/api/v1/robots/{rid}', tok('viewer')); return d if c == 200 else None

def mission(mid): return http('GET', f'/api/v1/missions/{mid}', tok('viewer'))[1]

def goal(x, y, yaw=0.0): return {'frame_id': 'map', 'x': x, 'y': y, 'yaw_rad': yaw}

try:
    subprocess.run([sys.executable, '-m', 'fleet_server', 'init', '--data-dir', DATA, '--secrets-dir', SEC, '--robots', 'pcu', 'mock1'], cwd=os.path.join(ROOT, 'server'), env=dict(os.environ, PYTHONPATH=os.path.join(ROOT, 'server')), check=True, capture_output=True)
    print('== 1. server, dashboard, authentication')
    srv = start_server()
    c, b = http('GET', '/', raw=True); check('dashboard HTML is served', c == 200 and b'Capstone1' in b, f'status={c}')
    check('/api/v1/health works without a token', http('GET', '/api/v1/health')[0] == 200)
    check('API without token -> 401', http('GET', '/api/v1/dashboard')[0] == 401)
    check('API with a wrong token -> 401', http('GET', '/api/v1/dashboard', 'x' * 40)[0] == 401)
    check('viewer can read', http('GET', '/api/v1/dashboard', tok('viewer'))[0] == 200)
    check('viewer cannot create a mission -> 403', http('POST', '/api/v1/missions', tok('viewer'), {'robot_id': 'mock1', 'goal': goal(0, 0)})[0] == 403)
    check('agent token cannot read operator endpoints -> 403', http('GET', '/api/v1/dashboard', tok('agent-mock1'))[0] == 403)
    check('agent token cannot act as another robot -> 403', http('POST', '/api/v1/agent/hello', tok('agent-mock1'), {'robot_id': 'pcu', 'agent': {}})[0] == 403)
    check('path traversal is refused', http('GET', '/../../etc/passwd')[0] in (400, 404))
    r = subprocess.run([sys.executable, '-m', 'fleet_server', 'serve', '--data-dir', DATA, '--port', '18081', '--bind', '0.0.0.0'], cwd=os.path.join(ROOT, 'server'), env=dict(os.environ, PYTHONPATH=os.path.join(ROOT, 'server')), capture_output=True, text=True, timeout=15)
    check('binding 0.0.0.0 is refused', r.returncode != 0 and 'refusing' in (r.stdout + r.stderr), (r.stdout + r.stderr).strip()[:80])
    r = subprocess.run([sys.executable, '-m', 'fleet_server', 'serve', '--data-dir', DATA, '--port', '18081', '--bind', '8.8.8.8'], cwd=os.path.join(ROOT, 'server'), env=dict(os.environ, PYTHONPATH=os.path.join(ROOT, 'server')), capture_output=True, text=True, timeout=15)
    check('binding a public address is refused', r.returncode != 0 and 'refusing' in (r.stdout + r.stderr))
    check('server listens on 127.0.0.1 only', subprocess.run(f"ss -ltn | awk '$4 ~ /:{PORT}$/ {{print $4}}'", shell=True, capture_output=True, text=True).stdout.split() == [f'127.0.0.1:{PORT}'])

    print('== 2. simulated robot (mock Nav2) + agent')
    mock = ros('python3 -m limo_robot_agent.mock_robot_node --ros-args -p map_yaml:=%s -p speed_mps:=1.0 -p start_x:=0.0 -p start_y:=0.0' % MOCK_MAP, 'mock.log')
    time.sleep(2); ag = ros(agent_cmd('mock1', 'mock', 'true', 'true'), 'agent_mock1.log')
    r = wait(lambda: (lambda d: d if d and d['online'] and d['state'] and d['state']['map']['status'] != 'NO_MAP' and d['state']['lidar']['status'] != 'NO_DATA' and d['state']['safety_gate']['status'] != 'NO_DATA' else None)(robot('mock1')), 25)
    check('mock1 online with state and map', r is not None)
    s = (r or {}).get('state') or {}
    check('mock1 flagged simulated', ((r or {}).get('agent') or {}).get('simulated') is True)
    check('pose in the map frame', (s.get('pose') or {}).get('frame_id') == 'map' and (s.get('pose') or {}).get('status') == 'OK', str(s.get('pose')))
    # The map image is uploaded separately from the state (at most once per 3 s), so it can lag behind the first state that reports a map: poll instead of a single GET.
    c, png = 0, b''
    for _ in range(40):
        c, png = http('GET', '/api/v1/robots/mock1/map.png', tok('viewer'), raw=True)
        if c == 200: break
        time.sleep(0.5)
    check('map PNG available', c == 200 and png[:8] == b'\x89PNG\r\n\x1a\n', f'{len(png)} bytes')
    # The first state that reports a map can precede the first scan / gate message: wait (bounded) for the sensor fields to be reported, then check their VALUES as before.
    settled = wait(lambda: (lambda st: st if st and (st.get('lidar') or {}).get('status') == 'OK' and (st.get('safety_gate') or {}).get('status') == 'NORMAL' and (st.get('nav2') or {}).get('status') == 'READY' else None)((robot('mock1') or {}).get('state')), 20)
    s = settled or (robot('mock1') or {}).get('state') or s
    check('nav2 READY', (s.get('nav2') or {}).get('status') == 'READY')
    check('lidar OK (mock scan)', (s.get('lidar') or {}).get('status') == 'OK', str(s.get('lidar'))[:80])
    # With limo_msgs installed the mock publishes /limo_status and the battery is VOLTAGE_ONLY; without it the agent must stay UNKNOWN. In both cases the percentage is never invented.
    bat = s.get('battery') or {}; have_limo = subprocess.run(['bash', '-c', 'set +u; source /opt/ros/humble/setup.bash; source %s/capstone_ws/install/setup.bash 2>/dev/null; python3 -c "import limo_msgs.msg"' % REPO], capture_output=True).returncode == 0
    check('battery: VOLTAGE_ONLY (limo_msgs present) or UNKNOWN (absent), percent never invented', bat.get('percent') is None and bat.get('status') == ('VOLTAGE_ONLY' if have_limo else 'UNKNOWN'), f"limo_msgs={'yes' if have_limo else 'no'} {str(bat)[:70]}")
    check('depth camera NO_DATA (mock publishes none)', (s.get('depth_camera') or {}).get('status') == 'NO_DATA')
    check('safety gate NORMAL', (s.get('safety_gate') or {}).get('status') == 'NORMAL')
    check('motor path not ARMED', ((s.get('safety_gate') or {}).get('motor_path') or {}).get('status') != 'ARMED')

    print('== 3. missions')
    key = 'test-key-1'; c1, m1 = http('POST', '/api/v1/missions', tok('operator'), {'robot_id': 'mock1', 'goal': goal(1.0, 0.0), 'idempotency_key': key})
    c2, m2 = http('POST', '/api/v1/missions', tok('operator'), {'robot_id': 'mock1', 'goal': goal(1.0, 0.0), 'idempotency_key': key})
    check('mission created (201)', c1 == 201 and m1['mission_id'].startswith('m_'))
    check('same idempotency_key returns the same mission (200, created=false)', c2 == 200 and m2['mission_id'] == m1['mission_id'] and m2['created'] is False)
    done = wait(lambda: (lambda m: m if m['terminal'] else None)(mission(m1['mission_id'])), 40)
    check('mission 1 SUCCEEDED through the mock NavigateToPose action', done and done['status'] == 'SUCCEEDED', str(done and (done['status'], done['detail'])))
    check('command ids recorded for the mission', done and len(done['commands']) == 1 and done['commands'][0]['command_id'].startswith('c_'))
    pose = (robot('mock1')['state'].get('pose') or {}); check('robot pose moved near the goal', abs(pose.get('x', 99) - 1.0) < 0.3, str(pose.get('x')))
    c, mA = http('POST', '/api/v1/missions', tok('operator'), {'robot_id': 'mock1', 'goal': goal(6.0, 2.0), 'idempotency_key': 'k-far'})
    c, mB = http('POST', '/api/v1/missions', tok('operator'), {'robot_id': 'mock1', 'goal': goal(0.0, 0.0), 'idempotency_key': 'k-queued'})
    check('second mission waits in QUEUED while the first is active', wait(lambda: mission(mA['mission_id'])['status'] in ('DISPATCHED', 'ACCEPTED', 'EXECUTING'), 10) and mission(mB['mission_id'])['status'] == 'QUEUED')
    c, cb = http('POST', f"/api/v1/missions/{mB['mission_id']}/cancel", tok('operator'), {}); check('queued mission canceled without dispatch', cb['status'] == 'CANCELED' and not cb['commands'], str(cb['status']))
    wait(lambda: mission(mA['mission_id'])['status'] == 'EXECUTING', 15)
    c, ca = http('POST', f"/api/v1/missions/{mA['mission_id']}/cancel", tok('operator'), {}); check('cancel of an executing mission accepted (202)', c == 202)
    fin = wait(lambda: (lambda m: m if m['terminal'] else None)(mission(mA['mission_id'])), 20); check('executing mission ends CANCELED', fin and fin['status'] == 'CANCELED', str(fin and fin['status']))
    check('cancel is idempotent', http('POST', f"/api/v1/missions/{mA['mission_id']}/cancel", tok('operator'), {})[1]['status'] == 'CANCELED')
    ev = http('GET', '/api/v1/events?limit=200', tok('viewer'))[1]['events']; kinds = {e['kind'] for e in ev}
    check('event log has registration, map, mission events', {'robot_registered', 'map_updated', 'mission_created', 'mission_status'} <= kinds, str(sorted(kinds))[:120])
    check('bad goal frame rejected (400)', http('POST', '/api/v1/missions', tok('operator'), {'robot_id': 'mock1', 'goal': {'frame_id': 'odom', 'x': 0, 'y': 0}})[0] == 400)
    check('unknown robot -> 404', http('POST', '/api/v1/missions', tok('operator'), {'robot_id': 'nobody', 'goal': goal(0, 0)})[0] == 404)

    print('== 4. interlocks (agent side)')
    ag_real = ros(agent_cmd('pcu', 'real', 'false', 'false'), 'agent_pcu.log')
    wait(lambda: (robot('pcu') or {}).get('online'), 20)
    c, mr = http('POST', '/api/v1/missions', tok('operator'), {'robot_id': 'pcu', 'goal': goal(1, 0), 'idempotency_key': 'k-real'})
    rr = wait(lambda: (lambda m: m if m['terminal'] else None)(mission(mr['mission_id'])), 20)
    check("real-mode agent (allow_missions=false) REJECTS missions", rr and rr['status'] == 'REJECTED' and 'MISSIONS_DISABLED' in rr['detail'], str(rr and (rr['status'], rr['detail'])))
    armed = ros("python3 -c \"import rclpy,time;from geometry_msgs.msg import Twist;rclpy.init();n=rclpy.create_node('fake_cmd_vel_publisher');p=n.create_publisher(Twist,'/cmd_vel',10);time.sleep(60)\"", 'armed.log')
    wait(lambda: ((robot('mock1')['state'].get('safety_gate') or {}).get('motor_path') or {}).get('status') == 'ARMED', 15)
    check('agent reports motor path ARMED when /cmd_vel has a publisher', ((robot('mock1')['state'].get('safety_gate') or {}).get('motor_path') or {}).get('status') == 'ARMED')
    c, mk = http('POST', '/api/v1/missions', tok('operator'), {'robot_id': 'mock1', 'goal': goal(1, 1), 'idempotency_key': 'k-armed'})
    rk = wait(lambda: (lambda m: m if m['terminal'] else None)(mission(mk['mission_id'])), 20)
    check('mission REFUSED while the motor path is ARMED', rk and rk['status'] == 'REJECTED' and 'MOTOR_PATH_ARMED' in rk['detail'], str(rk and rk['detail'])[:90])
    kill(armed)
    # The killed participant can linger in the ROS graph for a while; wait until the agent no longer sees a /cmd_vel publisher, otherwise a later mission is (correctly) refused as MOTOR_PATH_ARMED.
    wait(lambda: ((robot('mock1')['state'].get('safety_gate') or {}).get('motor_path') or {}).get('status') != 'ARMED', 40)

    print('== 5. server restart')
    kill(srv); time.sleep(1)
    check('agent keeps running while the server is down', ag.poll() is None)
    srv = start_server()
    back = wait(lambda: (lambda d: d if d and d['online'] and d['last_seen_age_s'] is not None and d['last_seen_age_s'] < 3 else None)(robot('mock1')), 25)
    check('mock1 reconnects automatically after the server restart', back is not None)
    c, ms = http('GET', '/api/v1/missions?limit=50', tok('viewer')); ids = {m['mission_id']: m['status'] for m in ms['missions']}
    check('missions persisted across the restart', ids.get(m1['mission_id']) == 'SUCCEEDED' and ids.get(mA['mission_id']) == 'CANCELED', str({k[:6]: v for k, v in ids.items()}))
    ev = http('GET', '/api/v1/events?limit=300', tok('viewer'))[1]['events']; check('server_started event recorded twice', sum(1 for e in ev if e['kind'] == 'server_started') == 2)
    c, mn = http('POST', '/api/v1/missions', tok('operator'), {'robot_id': 'mock1', 'goal': goal(0.5, 0.5), 'idempotency_key': 'k-after-restart'})
    fn = wait(lambda: (lambda m: m if m['terminal'] else None)(mission(mn['mission_id'])), 40); check('new mission works after the restart', fn and fn['status'] == 'SUCCEEDED', str(fn and fn['status']))
    check('map still served after the restart', http('GET', '/api/v1/robots/mock1/map.png', tok('viewer'), raw=True)[0] == 200)

    print('== 6. offline detection')
    kill(ag); off = wait(lambda: (lambda d: d if d and not d['online'] else None)(robot('mock1')), 15); check('robot becomes OFFLINE after the agent stops', off is not None)
    ev = http('GET', '/api/v1/events?limit=300', tok('viewer'))[1]['events']; check('robot_offline event recorded', any(e['kind'] == 'robot_offline' and e['robot_id'] == 'mock1' for e in ev))
finally:
    for p in reversed(PROCS):
        if p.poll() is None: kill(p)
    ok = sum(1 for _, p in RES if p); print(f'\nTOTAL {ok}/{len(RES)} passed   (logs in {T})')
    if ok == len(RES): shutil.rmtree(T, ignore_errors=True)
    sys.exit(0 if ok == len(RES) else 1)
