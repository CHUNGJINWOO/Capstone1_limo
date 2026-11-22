"""Loss-of-supervision scenarios with the independent motion watchdog, judged on velocity topics and Nav2 goals.

usage: motion_faults.py RUN_DIR PORT SCENARIO   (fresh run per scenario)
SCENARIO: agent_kill | agent_freeze | server_partition | lease_loss | watchdog_kill | gate_kill | agent_restart
          | watchdog_freeze | foreign_goal | bypass_publisher (needs GATE_REQ_WD=1: proposed Gate revision 03b)
Measured after the fault instant (t_f):
  raw_zero  = last non-zero /cmd_vel_raw (Gate input) - t_f
  gate_zero = last non-zero /cmd_vel_locked (Gate output; /cmd_vel on the real robot) - t_f
  goal_end  = first NavigateToPose status with 0 active goals - t_f
"""
import json, os, signal, subprocess, sys, time, urllib.error, urllib.request

RUN, PORT, SCEN = sys.argv[1], int(sys.argv[2]), sys.argv[3]
HERE = os.path.dirname(os.path.abspath(__file__))
BASE = f'http://127.0.0.1:{PORT}/api/v1'
tok = lambda n: open(f'{RUN}/secrets/{n}.token').read().strip()
OPT, VIEW, RID = tok('operator'), tok('viewer'), 'iso_bench'
PG = dict(line.split() for line in open(f'{RUN}/pgnames'))
RES, TAG = [], str(int(time.time()))
OBSERVE = 25


def http(method, path, token=VIEW, body=None, timeout=4):
    req = urllib.request.Request(BASE + path, method=method, data=json.dumps(body).encode() if body is not None else None,
                                 headers={'Content-Type': 'application/json', 'Authorization': 'Bearer ' + token})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read() or b'null')
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b'null')
    except Exception as e:
        return 0, {'error': str(e)}


def check(name, ok, detail=''):
    RES.append({'name': name, 'ok': bool(ok), 'detail': str(detail)[:300]})
    print(f"  [{'PASS' if ok else 'FAIL'}] {name} {str(detail)[:200]}", flush=True)


def rows(since=0):
    return [r for r in map(json.loads, open(f'{RUN}/vel.jsonl')) if r['t'] >= since]


def nz(r):
    return abs(r.get('vx', 0)) > 1e-4 or abs(r.get('wz', 0)) > 1e-4


def last_nz(rs, topic, t_f):
    ts = [r['t'] for r in rs if r['topic'] == topic and nz(r)]
    return round(max(ts) - t_f, 2) if ts else None


def goal_end(rs, t_f):
    ts = [r['t'] for r in rs if r['topic'] == '/navigate_to_pose/_action/status' and r['active'] == 0 and r['t'] >= t_f]
    return round(min(ts) - t_f, 2) if ts else None


def wait(fn, t, step=.4):
    end = time.time() + t
    while time.time() < end:
        v = fn()
        if v:
            return v
        time.sleep(step)
    return None


def sig(name, s):
    os.killpg(int(PG[name]), s)


ttl = 10 if SCEN == 'lease_loss' else 120
c, lease = http('POST', f'/robots/{RID}/lock', OPT, {'action': 'acquire', 'ttl_s': ttl})
c, b = http('POST', f'/robots/{RID}/mapping', OPT, dict(action='START', epoch=lease['epoch'], idempotency_key=f'm-{TAG}', mode='NEW_MAP'))
t0 = time.time()


def driving():
    if SCEN == 'lease_loss':
        http('POST', f'/robots/{RID}/lock', OPT, {'action': 'renew', 'ttl_s': ttl})
    w = (http('GET', f'/robots/{RID}')[1].get('state') or {}).get('mapping_worker') or {}
    rs = rows(time.time() - 1.0)
    return w if (w.get('goal') or {}).get('status') == 'ACCEPTED' and any(r['topic'] == '/cmd_vel_locked' and nz(r) for r in rs) else None


w = wait(driving, 60)
# ---- preconditions (any failure => INVALID run, no safety verdict)
robot = http('GET', f'/robots/{RID}')[1]
ex = (robot.get('state') or {}).get('execution') or {}
recent = rows(time.time() - 2.0)
wd_now = [r for r in rows(t0) if r['topic'] == 'watchdog'][-1:]
pre = {
    'agent online+reconciled': bool(robot.get('online') and robot.get('reconciled')),
    'watchdog MOTION_AUTHORIZED for this agent boot': bool(wd_now and wd_now[0]['state'] == 'MOTION_AUTHORIZED'
                                                           and wd_now[0]['agent_boot_id'] == (robot.get('agent') or {}).get('boot_id')),
    'idle proof obtained from Nav2 before first goal (empty cancel-all answer)': bool(wd_now and wd_now[0]['cancel_empty'] > 0),
    'mapping goal ACCEPTED with UUID': bool(w and (w.get('goal') or {}).get('goal_uuid')),
    'active goal owned by the mapping session': bool((ex.get('active') or {}).get('owner') == 'frontier'
                                                     and (ex.get('active') or {}).get('mission_id') == (w or {}).get('session_id')),
    'Nav2 non-zero output (/fleet/nav_cmd_vel)': any(r['topic'] == '/fleet/nav_cmd_vel' and nz(r) for r in recent),
    'watchdog forwarded non-zero (/cmd_vel_raw)': any(r['topic'] == '/cmd_vel_raw' and nz(r) for r in recent),
    'Gate output non-zero (/cmd_vel_locked)': any(r['topic'] == '/cmd_vel_locked' and nz(r) for r in recent),
}
GATE_REQ = os.environ.get('GATE_REQ_WD') == '1'
gate_rows = [r for r in rows(0) if r['topic'] == 'gate']   # recorded on change only: the latest row is the current state
if GATE_REQ:
    pre['Gate requires the watchdog and reports it OK with exactly one /cmd_vel_raw publisher'] = bool(
        gate_rows and gate_rows[-1].get('require_motion_watchdog') is True and gate_rows[-1].get('motion_watchdog_ok') is True
        and gate_rows[-1].get('cmd_vel_raw_publishers') == 1)
if SCEN == 'bypass_publisher' and not GATE_REQ and os.environ.get('BYPASS_CONTROL') != '1':
    pre['scenario needs the Gate watchdog requirement (GATE_REQ_WD=1)'] = False
for k, v in pre.items():
    check('PRECONDITION ' + k, v)
if not all(pre.values()):
    json.dump({'scenario': SCEN, 'valid': False, 'preconditions': pre, 'checks': RES}, open(f'{RUN}/motion_faults_{SCEN}.json', 'w'), indent=1)
    print('INVALID RUN: preconditions not met; no safety verdict'); sys.exit(2)

t_f = time.time()
if SCEN == 'agent_kill':
    sig('agent', signal.SIGKILL)
elif SCEN == 'agent_freeze':
    sig('agent', signal.SIGSTOP)
elif SCEN == 'server_partition':
    sig('server', signal.SIGSTOP)
elif SCEN == 'lease_loss':
    exp = http('GET', f'/robots/{RID}/lock')[1]['lease']['expires_at']
    time.sleep(max(0, exp - time.time()))
    t_f = exp                                  # reference = lease expiry
elif SCEN == 'watchdog_kill':
    sig('watchdog', signal.SIGKILL)
elif SCEN == 'gate_kill':
    # Only the Gate inside THIS run's stack process group (never another session's Gate).
    out = subprocess.run(['pgrep', '-g', PG['stack'], '-f', 'stall_detector_node'], capture_output=True, text=True).stdout.split()
    assert out, 'gate of this stack not found'
    for pid in out:
        os.kill(int(pid), signal.SIGKILL)
elif SCEN == 'agent_restart':
    sig('agent', signal.SIGKILL)
elif SCEN == 'watchdog_freeze':
    sig('watchdog', signal.SIGSTOP)
elif SCEN in ('foreign_goal', 'bypass_publisher'):
    pose = (robot.get('state') or {}).get('pose') or {}
    args = ['foreign_goal', str((pose.get('x') or 0.0) + 0.3), str(pose.get('y') or 0.0)] if SCEN == 'foreign_goal' else ['bypass_publisher']
    helper = subprocess.Popen(['python3', os.path.join(HERE, 'helper.py'), *args], stdout=open(f'{RUN}/helper_{SCEN}.log', 'w'),
                              stderr=subprocess.STDOUT, start_new_session=True)
    open(f'{RUN}/pgids', 'a').write(f'{helper.pid}\n')
time.sleep(OBSERVE)
if SCEN == 'bypass_publisher':
    os.killpg(helper.pid, signal.SIGINT)        # graceful: leaves the ROS graph
if SCEN == 'watchdog_freeze':
    t_cont = time.time()
    sig('watchdog', signal.SIGCONT)
    time.sleep(10)        # a 25 s freeze outlives the DDS participant lease: rediscovery before status is observed
rs = rows(t_f - 1)
raw_z, gate_z, g_end = last_nz(rs, '/cmd_vel_raw', t_f), last_nz(rs, '/cmd_vel_locked', t_f), goal_end(rs, t_f)
fs = [r for r in rs if r['topic'] == 'watchdog' and r['state'] == 'FAILSAFE' and r['t'] >= t_f]
detect = round(fs[0]['fault_wall_time'] - t_f, 2) if fs and fs[0].get('fault_wall_time') else None
extra = {}
if SCEN == 'foreign_goal':
    own = set((((robot.get('state') or {}).get('execution') or {}).get('active') or {}).get('goal_uuid') or '')
    own = {(((robot.get('state') or {}).get('execution') or {}).get('active') or {}).get('goal_uuid')}
    seen = [r['t'] for r in rs if r['topic'] == '/navigate_to_pose/_action/status' and r['t'] >= t_f and set(r.get('uuids', [])) - own]
    extra['foreign_goal_seen'] = round(seen[0] - t_f, 2) if seen else None
if SCEN == 'bypass_publisher':
    seen = [r['t'] for r in rs if r['topic'] == 'gate' and r.get('cmd_vel_raw_publishers') == 2 and r['t'] >= t_f]
    extra['gate_sees_second_publisher'] = round(seen[0] - t_f, 2) if seen else None
print(f'  TIMING detect={detect} watchdog_out_zero={raw_z} gate_out_zero={gate_z} goal_end={g_end} {extra} (s after fault)')
graph = [r for r in rows(t0) if r['topic'] == '/cmd_vel#graph']
check('/cmd_vel never had a publisher (locked motor path)', graph and all(r['publishers'] == 0 for r in graph))
LIMIT_ZERO, LIMIT_GOAL = 0.5, 5.0
if SCEN == 'foreign_goal':
    fg = extra['foreign_goal_seen']
    check('foreign goal reached Nav2 (scenario valid)', fg is not None, fg)
    check('watchdog FAILSAFE FOREIGN_NAV_GOAL', any(r['topic'] == 'watchdog' and r['state'] == 'FAILSAFE' and r.get('last_fault') == 'FOREIGN_NAV_GOAL' for r in rs))
    check('Gate output zero within 1.0 s of the foreign goal appearing', fg is not None and (gate_z is None or gate_z - fg <= 1.0), (gate_z, fg))
    check(f'all goals ended within {LIMIT_GOAL} s', g_end is not None and g_end <= LIMIT_GOAL, g_end)
elif SCEN == 'bypass_publisher':
    gs = extra['gate_sees_second_publisher']
    check('Gate saw a second /cmd_vel_raw publisher', gs is not None, gs)
    late = [r for r in rs if r['topic'] == '/cmd_vel_locked' and nz(r) and gs is not None and r['t'] > t_f + gs + 0.1]
    check('Gate output stays zero while the bypass publisher exists', gs is not None and not late, len(late))
    check('bypass commands reached the Gate input (scenario valid)', any(r['topic'] == '/cmd_vel_raw' and abs(r['vx'] - 0.05) < 1e-6 for r in rs if r['t'] >= t_f))
elif SCEN == 'watchdog_freeze':
    check(f'Gate output zero within {LIMIT_ZERO} s (stale input + stale watchdog status)', gate_z is None or gate_z <= LIMIT_ZERO, gate_z)
    check(f'goal ended within {LIMIT_GOAL} s (Agent sees watchdog stale)', g_end is not None and g_end <= LIMIT_GOAL, g_end)
    wr = [r for r in rows(t_cont) if r['topic'] == 'watchdog']
    check('after SIGCONT the watchdog left AUTHORIZED via WATCHDOG_SELF_STALL (no resumed forwarding)',
          any(r.get('last_fault') == 'WATCHDOG_SELF_STALL' for r in wr), wr and {k: wr[-1].get(k) for k in ('state', 'last_fault', 'trips')})
    post = [r for r in rows(t_cont) if r['topic'] in ('/cmd_vel_raw', '/cmd_vel_locked') and nz(r)]
    check('after SIGCONT: no non-zero velocity at Gate input or output', not post, len(post))
elif SCEN == 'gate_kill':
    gate_msgs = [r for r in rs if r['topic'] == '/cmd_vel_locked' and r['t'] > t_f + 0.5]
    check('Gate output stops entirely when the Gate dies (motor driver timeout decides; MCU watchdog unverified)', not gate_msgs, len(gate_msgs))
    check(f'goal ended within {LIMIT_GOAL} s (Agent sees motor path not locked)', g_end is not None and g_end <= LIMIT_GOAL, g_end)
elif SCEN == 'server_partition':
    check('Gate output zero within the Agent fence budget (<= 5 s)', gate_z is None or gate_z <= 5.0, gate_z)
    check(f'goal ended within {LIMIT_GOAL + 1} s', g_end is not None and g_end <= LIMIT_GOAL + 1, g_end)
elif SCEN == 'lease_loss':
    check('Gate output zero within 5 s of lease expiry', gate_z is None or gate_z <= 5.0, gate_z)
    check('goal ended within 6 s of lease expiry', g_end is not None and g_end <= 6.0, g_end)
else:
    check(f'Gate output zero within {LIMIT_ZERO} s', gate_z is None or gate_z <= LIMIT_ZERO, gate_z)
    if SCEN == 'watchdog_kill':
        check(f'goal ended within {LIMIT_GOAL} s (Agent sees watchdog missing)', g_end is not None and g_end <= LIMIT_GOAL, g_end)
    else:
        check(f'goal ended within {LIMIT_GOAL} s (watchdog cancel-all)', g_end is not None and g_end <= LIMIT_GOAL, g_end)
wd = [r for r in rs if r['topic'] == 'watchdog']
if SCEN in ('agent_kill', 'agent_freeze', 'agent_restart'):
    check('watchdog FAILSAFE (heartbeat lost) and cancel-all accepted by Nav2', any(r['state'] == 'FAILSAFE' and r['reason'] == 'AGENT_HEARTBEAT_LOST' for r in wd)
          and wd and wd[-1]['cancel_nonempty'] > 0, wd and {k: wd[-1][k] for k in ('state', 'reason', 'cancel_requests', 'cancel_nonempty', 'cancel_empty', 'dropped_inputs')})

if SCEN == 'agent_freeze':
    sig('agent', signal.SIGCONT)
if SCEN == 'server_partition':
    sig('server', signal.SIGCONT)
if SCEN == 'agent_restart':
    fc = os.path.abspath(os.path.join(HERE, '..', '..'))
    repo = os.path.dirname(fc)
    cmd = (f'source /opt/ros/humble/setup.bash; source {repo}/capstone_ws/install/setup.bash; export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID={os.environ["ROS_DOMAIN_ID"]}; '
           f'PYTHONPATH={fc}/agent/limo_robot_agent:$PYTHONPATH exec python3 -m limo_robot_agent.agent_node --ros-args -p server_url:=http://127.0.0.1:{PORT} '
           f'-p token_file:={RUN}/secrets/agent-iso_bench.token -p robot_id:=iso_bench -p mode:=mock -p simulated:=true -p allow_missions:=false '
           f'-p state_dir:={RUN}/agent -p web_mapping_enabled:=true -p require_motion_watchdog:=true')
    p = subprocess.Popen(['bash', '-c', cmd], stdout=open(f'{RUN}/agent_restart.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    open(f'{RUN}/pgids', 'a').write(f'{p.pid}\n')
if SCEN in ('agent_freeze', 'server_partition', 'agent_restart'):
    t_r = time.time()
    time.sleep(20)
    rs2 = rows(t_r)
    check('after recovery: no non-zero velocity at Gate input or output (old goal not resumed)',
          not any(r['topic'] in ('/cmd_vel_raw', '/cmd_vel_locked') and nz(r) for r in rs2))
    st = [r for r in rs2 if r['topic'] == '/navigate_to_pose/_action/status']
    check('after recovery: no active NavigateToPose goal', not st or st[-1]['active'] == 0, st and st[-1]['active'])
    wd2 = [r for r in rs2 if r['topic'] == 'watchdog']
    if SCEN in ('agent_freeze', 'agent_restart'):
        check('watchdog re-armed only with heartbeat + idle proof (no active goal)', wd2 and wd2[-1]['state'] == 'MOTION_AUTHORIZED' and wd2[-1]['nav_active'] in (0, None), wd2 and wd2[-1])
s = wait(lambda: (lambda x: x if x.get('state') else None)(http('GET', f'/robots/{RID}/mapping')[1]), 15)
if SCEN in ('bypass_publisher', 'foreign_goal', 'watchdog_freeze'):
    print('  session state (informational for the new scenarios):', s and s.get('state'))
    check('mapping session not MAPPING (no silent continuation)', s and s.get('state') != 'MAPPING', s and s.get('state')) if SCEN != 'bypass_publisher' else None
else:
    check('server mapping session UNKNOWN (no silent recovery)', s and s['state'] == 'UNKNOWN', s and s.get('state'))
if SCEN != 'bypass_publisher':
    c, lease = http('POST', f'/robots/{RID}/lock', OPT, {'action': 'acquire', 'ttl_s': 60})
    c, b = http('POST', f'/robots/{RID}/mapping', OPT, dict(action='START', epoch=lease.get('epoch'), idempotency_key=f'n-{TAG}', mode='NEW_MAP'))
    check('new START refused afterwards (operator review required)', c == 409, b)
json.dump({'scenario': SCEN, 'valid': True, 'gate_requires_watchdog': GATE_REQ, 'timing': {'detect': detect, 'watchdog_out_zero': raw_z, 'gate_out_zero': gate_z, 'goal_end': g_end, **extra}, 'checks': RES},
          open(f'{RUN}/motion_faults_{SCEN}.json', 'w'), indent=1)
print(f"TOTAL {sum(r['ok'] for r in RES)}/{len(RES)} passed")
