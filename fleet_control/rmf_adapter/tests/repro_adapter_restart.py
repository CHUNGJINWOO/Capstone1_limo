#!/usr/bin/env python3
"""Reproduces and measures what happens when the Fleet Adapter dies (SIGKILL) in the middle of an RMF task and is started again.
One trial per process (own temp dirs). usage: python3 tests/repro_adapter_restart.py [observe_seconds]  -> prints a JSON summary."""
import json, os, signal, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rmf_env import *
from ws_sink import WsSink
import rclpy
OBS = float(sys.argv[1]) if len(sys.argv) > 1 else 45


def adapter_lines(): return open(os.path.join(T, 'adapter.log')).read().splitlines()


try:
    SINK = WsSink(WS_PORT, os.path.join(T, 'rmf_api_stream.jsonl'))
    init_server(); start_server(); start_rmf_base(); time.sleep(3); start_robot(speed=0.6)
    wait(lambda: (robot() or {}).get('online'), 25); start_adapter()
    os.environ.update(ROS_DOMAIN_ID='151', ROS_LOCALHOST_ONLY='1'); rclpy.init()
    from rmf_client import RmfClient
    c = RmfClient(); wait(lambda: c.robot(), 30)
    r = c.dispatch_patrol(['wp_e']); tid = r['state']['booking']['id']
    wait(lambda: [m for m in missions() if m['status'] == 'EXECUTING'], 40); time.sleep(3)
    pre = {'task': tid, 'sink_status': SINK.status(tid), 'dispatch': (lambda d: d and (d.status, d.assignment.expected_robot_name))(c.dispatch_state(tid)),
           'missions': [(m['mission_id'], m['status'], m['idempotency_key']) for m in missions()]}
    n_lines = len(adapter_lines()); t_kill = time.time()
    kill('adapter', signal.SIGKILL); start_adapter(); t_restart = time.time()
    timeline = []
    last = None
    while time.time() - t_restart < OBS:
        snap = {'t': round(time.time() - t_restart, 1), 'task_sink': SINK.status(tid), 'dispatch': (lambda d: d and (d.status, d.assignment.expected_robot_name))(c.dispatch_state(tid)),
                'missions': [(m['mission_id'][-4:], m['status'], (m['idempotency_key'] or '')[-8:]) for m in missions() if not m['terminal']]}
        key = json.dumps({k: v for k, v in snap.items() if k != 't'})
        if key != last: timeline.append(snap); last = key
        time.sleep(0.5)
    # follow-up: what does the operator see, can the stale task be canceled, does the restarted adapter accept new work?
    cr = c.cancel(tid); canceled = bool(wait(lambda: SINK.status(tid) in ('canceled', 'killed', 'failed'), 20)); st_after_cancel = SINK.status(tid)
    ds = c.dispatch_state(tid); dispatch_after_cancel = ds and (ds.status, list(ds.errors))
    r2 = c.dispatch_patrol(['wp_c']); tid2 = r2['state']['booking']['id']; done2 = bool(wait(lambda: SINK.status(tid2) == 'completed', 60))
    new_lines = adapter_lines()[n_lines:]
    interesting = [l for l in new_lines if any(k in l for k in ('Bidding notice', 'awarded', 'Beginning new task', 'before receiving BidNotice', 'RMF path', 'submit:', 'adopt', 'orphan', 'registered', 'Added a robot', 'cancel', 'stop:', 'FAILED', 'lease'))]
    gl = [json.loads(l) for l in open(os.path.join(T, 'mock_goals.jsonl'))] if os.path.exists(os.path.join(T, 'mock_goals.jsonl')) else []
    post_missions = [(m['mission_id'], m['status'], m['idempotency_key'], m['goal']) for m in missions() if m['created_at'] > t_restart]
    gl = [json.loads(l) for l in open(os.path.join(T, 'mock_goals.jsonl'))] if os.path.exists(os.path.join(T, 'mock_goals.jsonl')) else []
    out = {'pre': pre, 'timeline': timeline, 'adapter_log_after_restart': interesting[:60], 'missions_created_after_restart': post_missions, 'mock_goals': [(round(g['t'] - t_kill, 1), g['ev'], g.get('x'), g.get('y')) for g in gl],
           'cancel_response': cr, 'stale_task_status_after_cancel': st_after_cancel, 'dispatch_after_cancel': dispatch_after_cancel, 'new_task_completed': done2, 'new_task_goals_at_mock': [g for g in gl if g['ev'] == 'goal_received'][1:], 'final_task_status': SINK.status(tid), 'dispatch_final': (lambda d: d and (d.status, list(d.errors)))(c.dispatch_state(tid)), 'logs': T}
    print(json.dumps(out, indent=1, default=str))
finally:
    try: rclpy.shutdown()
    except Exception: pass
    cleanup(); os._exit(0)
