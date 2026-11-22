"""SIMULATION ONLY (2026-11-22): how long the installed Nav2 velocity_smoother keeps publishing a NON-ZERO command after its
input (the controller / behavior server, /fleet/cmd_vel_nav in the stacks) stops or goes to zero.

Why: the Agent VelocityArbiter forwards velocity_smoother output (geometry_msgs/Twist: no header, no stamp, no goal id). After a
goal ends, any non-zero smoother output that is not backed by a live controller command is a replay of an older command.
Runs the real smoother node (installed package, the shared slow profile read in place, unmodified) in a private domain with
ROS_LOCALHOST_ONLY=1; no Gate, no motor. Cases (input 0.10 m/s at 15 Hz for 2 s, then):
  stop_silent   input stops without a zero (controller killed / frozen / message lost)
  stop_zero     one zero, then silence (normal controller termination: publishZeroVelocity)
  zero_stream   zeros at 15 Hz
usage: python3 nav2_smoother_tail_probe_20261122.py --out <new file.json> [--domain 231] [--repeat 3]
"""
import argparse, json, os, signal, subprocess, sys, time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PROFILE = REPO / 'capstone_ws/src/limo_capstone/config/integrated_nav2_lidar_only_slow_params.yaml'


def probe(case):
    import rclpy
    from geometry_msgs.msg import Twist
    node = rclpy.create_node('smoother_tail_probe')
    pub = node.create_publisher(Twist, '/probe/in', 10)
    out = []
    node.create_subscription(Twist, '/probe/out', lambda m: out.append((time.monotonic(), m.linear.x, m.angular.z)), 50)
    def spin_for(s, v=None, hz=15.):
        end, nxt = time.monotonic() + s, 0.
        while time.monotonic() < end:
            if v is not None and time.monotonic() >= nxt:
                m = Twist(); m.linear.x = v; pub.publish(m); nxt = time.monotonic() + 1 / hz
            rclpy.spin_once(node, timeout_sec=.005)
    spin_for(1.0)                                    # discovery
    spin_for(2.0, .10)
    t_end = time.monotonic()
    if case == 'stop_zero':
        pub.publish(Twist()); spin_for(3.0)
    elif case == 'zero_stream':
        spin_for(3.0, 0.)
    else:
        spin_for(3.0)
    after = [(round(t - t_end, 3), v) for t, v, _ in out if t >= t_end]
    nz = [t for t, v in after if abs(v) > 1e-6]
    node.destroy_node()
    return {'case': case, 'msgs_before': sum(1 for t, *_ in out if t < t_end), 'msgs_after': len(after),
            'last_nonzero_after_input_end_s': max(nz) if nz else None, 'nonzero_after': len(nz),
            'first_after': after[:3], 'value_at_last_nonzero': next((v for t, v in after if nz and t == max(nz)), None)}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--domain', default='231')
    ap.add_argument('--repeat', type=int, default=3)
    a = ap.parse_args(); out = Path(a.out)
    if out.exists(): sys.exit(f'refusing to overwrite {out}')
    os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=a.domain)
    proc = subprocess.Popen(['ros2', 'run', 'nav2_velocity_smoother', 'velocity_smoother', '--ros-args', '--params-file', str(PROFILE),
                             '-p', 'use_sim_time:=false', '-r', 'cmd_vel:=/probe/in', '-r', 'cmd_vel_smoothed:=/probe/out'],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        time.sleep(2.0)
        for t in ('configure', 'activate'):
            subprocess.run(['ros2', 'lifecycle', 'set', '/velocity_smoother', t], check=True, timeout=20, capture_output=True)
        params = {}
        for p in ('velocity_timeout', 'smoothing_frequency', 'max_decel', 'feedback', 'deadband_velocity'):
            r = subprocess.run(['ros2', 'param', 'get', '/velocity_smoother', p], capture_output=True, text=True, timeout=20)
            params[p] = r.stdout.strip()
        import rclpy
        rclpy.init(args=[])
        res = [probe(c) for _ in range(a.repeat) for c in ('stop_silent', 'stop_zero', 'zero_stream')]
        rclpy.shutdown()
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try: proc.wait(10)
        except subprocess.TimeoutExpired: os.killpg(proc.pid, signal.SIGKILL)
    doc = {'profile': str(PROFILE.relative_to(REPO)), 'params': params, 'results': res}
    out.write_text(json.dumps(doc, indent=1))
    for r in res: print(json.dumps(r))
    print('PARAMS', json.dumps(params))


if __name__ == '__main__':
    main()
