"""Run a subset of the unchanged snapshot harness with independent timing/cleanup instrumentation."""
import os
from pathlib import Path
root=Path(os.environ['CAPSTONE45_SNAPSHOT'])
original=root/'fleet_control/tests/phase5/run_phase5.py'
s=original.read_text()
a="threading.Thread(target=ex.spin, daemon=True).start()"
b="spin_thread = threading.Thread(target=ex.spin, daemon=True); spin_thread.start()"
assert s.count(a)==1;s=s.replace(a,b)
a="    args = ['node', str(HERE / 'browser_visitor.cjs'), BASE, CTX['token'], json.dumps(dict(spec, shot=str(OUT)))]"
b="    args = ['node', os.environ['CAPSTONE45_TIMING_BROWSER'], BASE, CTX['token'], json.dumps(dict(spec, shot=str(OUT)))]"
assert s.count(a)==1;s=s.replace(a,b)
a="    R.record('v03_cancel_during_guidance_stops_robot', 'IN_PROGRESS' in st and st[-1:] == ['CANCELED'] and still is True, ui_statuses=st, gate_zero_1_5s_after_cancel=still)"
b="""    sent = next((e['t'] for e in ev if e['kind']=='cancel_http_sent'), None)
    reply = next((e['t'] for e in ev if e['kind']=='cancel_http_reply'), None)
    cut = None if sent is None else sent + off
    around = [] if cut is None else [(round(t-cut,4),v,w) for t,v,w in locked if cut-.5 <= t <= cut+2.5]
    post = [(d,v,w) for d,v,w in around if d >= 0]
    nonzero = [(d,v,w) for d,v,w in post if abs(v)>1e-6 or abs(w)>1e-6]
    last_nonzero = max((d for d,v,w in nonzero),default=None)
    zeros = [d for d,v,w in post if abs(v)<=1e-6 and abs(w)<=1e-6 and (last_nonzero is None or d>last_nonzero)]
    until_zero = min(zeros) if zeros else None
    (OUT/'cancel_velocity_timeline.json').write_text(json.dumps({'samples':around,'http_duration_s':None if sent is None or reply is None else round(reply-sent,4),'last_nonzero_s':last_nonzero,'first_zero_after_last_nonzero_s':until_zero},indent=1))
    R.record('v03_cancel_during_guidance_stops_robot', 'IN_PROGRESS' in st and st[-1:] == ['CANCELED'] and still is True and bool(post) and until_zero is not None,
             ui_statuses=st, gate_zero_1_5s_after_cancel=still, sample_count=len(post), http_duration_s=None if sent is None or reply is None else round(reply-sent,4),
             first_zero_after_last_nonzero_s=until_zero, last_nonzero_s=last_nonzero)
"""
assert s.count(a)==1;s=s.replace(a,b)
a="""        for fn in (v00_setup_navigation_destinations_link, v01_service_closed_then_open, v02_request_completes_robot_arrives,
                   v03_cancel_during_guidance_stops_robot, v04_two_visitors_queue_fifo, v05_unavailable_reasons, v07_operator_map_click_uses_server_goal_check,
                   v06_visitor_token_isolation_and_revoke):"""
b="""        for fn in (v00_setup_navigation_destinations_link, v01_service_closed_then_open, v02_request_completes_robot_arrives,
                   v03_cancel_during_guidance_stops_robot):"""
assert s.count(a)==1;s=s.replace(a,b)
a="        ex.shutdown(); rclpy.try_shutdown()"
b="        ex.shutdown(); spin_thread.join(timeout=5); obs.destroy_node(); rclpy.try_shutdown()"
assert s.count(a)==1;s=s.replace(a,b)
exec(compile(s,str(original),'exec'),{'__name__':'__main__','__file__':str(original)})
