"""Fault injection: kill the Agent during MAPPING. usage: fault.py RUN_DIR PORT AGENT_PGID"""
import json, os, signal, subprocess, sys, time
sys.argv, (RUN, PORT, PG) = sys.argv[:1], sys.argv[1:4]
sys.argv += [RUN, PORT]
exec(open(os.path.join(os.path.dirname(__file__), 'probe.py')).read().split("t0 = time.time()")[0])
c, lease = http('POST', f'/robots/{RID}/lock', OPT, {'action': 'acquire', 'ttl_s': 300}); epoch = lease['epoch']
c, b = op('START', mode='NEW_MAP')
sid = b.get('session_id')
s = wait(lambda: (lambda x: x if x['state'] in ('MAPPING', 'UNKNOWN') else None)(status()), 40)
check('fault run: MAPPING reached', s and s['state'] == 'MAPPING', s and s['state'])
time.sleep(3)
pids = subprocess.run(['pgrep', '-f', 'async_slam_toolbox_node|fleet_frontier_candidates'], capture_output=True, text=True).stdout.split()
check('owned SLAM/Frontier processes running', len(pids) >= 2, pids)
os.killpg(int(PG), signal.SIGKILL)                      # Agent crash (no graceful cleanup)
t = time.time()
s = wait(lambda: (lambda x: x if x['state'] == 'UNKNOWN' else None)(status()), 15)
check('server marks session UNKNOWN after agent loss', s is not None, round(time.time() - t, 1))
gone = wait(lambda: not subprocess.run(['pgrep', '-f', 'async_slam_toolbox_node|fleet_frontier_candidates'], capture_output=True, text=True).stdout.split(), 10)
check('owned processes terminated by parent-death signal', gone)
c, b = op('START', mode='NEW_MAP')
check('START blocked while UNKNOWN', c == 409, b)
c, b = http('POST', '/missions', OPT, {'robot_id': RID, 'type': 'navigate_to_pose', 'goal': {'frame_id': 'map', 'x': 0.5, 'y': 0.2}, 'lease_epoch': epoch})
check('navigation blocked while UNKNOWN', c == 409, b)
json.dump(RES, open(f'{RUN}/fault_result.json', 'w'), indent=1)
print(f"TOTAL {sum(r['ok'] for r in RES)}/{len(RES)} passed")
