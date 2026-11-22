"""R2 Motion Watchdog graceful/abnormal shutdown & lifecycle matrix tests (2026-11-23).

Runs in isolated ROS 2 Humble domain (231, ROS_LOCALHOST_ONLY=1) on the laptop only; no robot, no motor, no 18190.
Skips cleanly when ROS 2 Python packages are not sourced in the shell.
"""
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
AGENT_SRC = HERE.parent / 'agent' / 'limo_robot_agent'
if str(AGENT_SRC) not in sys.path:
    sys.path.insert(0, str(AGENT_SRC))

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from limo_robot_agent import motion_watchdog as mw
    ROS_AVAILABLE = True
except ModuleNotFoundError as exc:
    ROS_AVAILABLE = False
    ROS_ERR = str(exc)


@unittest.skipUnless(ROS_AVAILABLE, f'ROS 2 python packages unavailable: {ROS_ERR if not ROS_AVAILABLE else ""}')
class TestWatchdogShutdownSafety(unittest.TestCase):
    """Deterministic verification of R2 shutdown velocity serialization and exit-code invariants (A-G)."""

    def test_a_and_d_normal_forwarding_and_post_shutdown_drop(self):
        """Cases A & D: normal velocity forwarding before shutdown, and dropping velocity inputs during/after shutdown."""
        os.environ['ROS_DOMAIN_ID'] = '231'
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        try:
            node = mw.MotionWatchdog()
            published = []
            orig_pub = node.out.publish
            node.out.publish = lambda m: (published.append((m.linear.x, m.angular.z, node.shutting_down)), orig_pub(m))
            node.state = mw.AUTHORIZED
            node.input_ok = True
            cmd = Twist()
            cmd.linear.x = 0.25
            cmd.angular.z = 0.3

            # Case A: normal velocity publish before shutdown
            node.on_cmd(cmd)
            self.assertEqual(node.counters['forwarded'], 1)
            self.assertEqual(published[-1], (0.25, 0.3, False))

            # Request shutdown: must switch to LOCKED, emit zero twist, and drop any further cmd
            node.request_shutdown('SIGNAL_SIGTERM')
            self.assertTrue(node.shutting_down)
            self.assertEqual(node.state, mw.LOCKED)
            self.assertEqual(published[-1], (0.0, 0.0, True))

            # Case D: velocity input arriving during/after shutdown processing
            dropped_before = node.counters['dropped_inputs']
            node.on_cmd(cmd)
            self.assertEqual(node.counters['forwarded'], 1)
            self.assertEqual(node.counters['dropped_inputs'], dropped_before + 1)

            # tick() during shutdown must never re-arm and must emit zero twist
            node.tick()
            self.assertEqual(node.state, mw.LOCKED)
            self.assertEqual(published[-1], (0.0, 0.0, True))
            # Invariant: zero non-zero velocity published while shutting_down is True
            post_shutdown_nonzero = [p for p in published if p[2] and (p[0] != 0.0 or p[1] != 0.0)]
            self.assertEqual(post_shutdown_nonzero, [])
            node.destroy_node()
        finally:
            rclpy.try_shutdown()

    def _run_hooked_watchdog_subprocess(self, hook_code: str) -> tuple[int, str, list[tuple[float, float, bool]]]:
        """Run mw.main() in a fresh subprocess with a deterministic hook and capture all out.publish events."""
        env = dict(
            os.environ,
            PYTHONDONTWRITEBYTECODE='1',
            ROS_DOMAIN_ID='231',
            ROS_LOCALHOST_ONLY='1',
            RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
            PYTHONPATH=f"{AGENT_SRC}:{os.environ.get('PYTHONPATH', '')}",
        )
        r = subprocess.run([sys.executable, '-c', hook_code], env=env, capture_output=True, text=True, timeout=15)
        output = r.stdout + ('\n' + r.stderr if r.stderr else '')
        events = []
        for line in output.splitlines():
            if line.startswith('PUB_EVENT '):
                parts = line.split()
                events.append((float(parts[1]), float(parts[2]), parts[3] == 'True'))
        return r.returncode, output, events

    def test_b_sigterm_right_after_on_cmd_initial_check(self):
        """Case B (Codex counterexample): SIGTERM delivered inside on_cmd right after initial safety check (during _finite_twist)."""
        code = """
import os, signal, sys
from geometry_msgs.msg import Twist
from limo_robot_agent import motion_watchdog as mw

orig_init = mw.MotionWatchdog.__init__
orig_finite = mw._finite_twist

def hooked_init(self):
    orig_init(self)
    orig_pub = self.out.publish
    def trace_pub(msg):
        print(f'PUB_EVENT {msg.linear.x} {msg.angular.z} {self.shutting_down}', flush=True)
        return orig_pub(msg)
    self.out.publish = trace_pub
    def fire_once():
        self._one_shot.cancel()
        self.state = mw.AUTHORIZED
        self.input_ok = True
        m = Twist()
        m.linear.x = 0.1
        m.angular.z = 0.2
        self.on_cmd(m)
    self._one_shot = self.create_timer(0.05, fire_once)

sent = {'done': False}
def hooked_finite(msg):
    ok = orig_finite(msg)
    if not sent['done']:
        sent['done'] = True
        os.kill(os.getpid(), signal.SIGTERM)
    return ok

mw.MotionWatchdog.__init__ = hooked_init
mw._finite_twist = hooked_finite
sys.exit(mw.main())
"""
        rc, out, events = self._run_hooked_watchdog_subprocess(code)
        self.assertEqual(rc, 0, out)
        self.assertTrue(len(events) >= 1, out)
        # Critical invariant: no non-zero velocity may ever be published once shutting_down=True
        nonzero_after_shutdown = [e for e in events if e[2] and (e[0] != 0.0 or e[1] != 0.0)]
        self.assertEqual(nonzero_after_shutdown, [], f'Non-zero velocity emitted after shutdown! events={events}\n{out}')
        # All published velocities must be zero because SIGTERM interrupted on_cmd before _publish_authorized
        self.assertTrue(all(e[0] == 0.0 and e[1] == 0.0 for e in events), f'Unexpected non-zero event: {events}\n{out}')

    def test_c_sigterm_immediately_before_publisher_call(self):
        """Case C: SIGTERM delivered immediately before _publish_authorized / self.out.publish call."""
        code = """
import os, signal, sys
from geometry_msgs.msg import Twist
from limo_robot_agent import motion_watchdog as mw

orig_init = mw.MotionWatchdog.__init__
orig_pub_auth = mw.MotionWatchdog._publish_authorized

def hooked_init(self):
    orig_init(self)
    orig_pub = self.out.publish
    def trace_pub(msg):
        print(f'PUB_EVENT {msg.linear.x} {msg.angular.z} {self.shutting_down}', flush=True)
        return orig_pub(msg)
    self.out.publish = trace_pub
    def fire_once():
        self._one_shot.cancel()
        self.state = mw.AUTHORIZED
        self.input_ok = True
        m = Twist()
        m.linear.x = 0.15
        m.angular.z = -0.25
        self.on_cmd(m)
    self._one_shot = self.create_timer(0.05, fire_once)

def hooked_pub_auth(self, msg):
    # Deliver SIGTERM immediately before entering the masked publish critical section
    os.kill(os.getpid(), signal.SIGTERM)
    return orig_pub_auth(self, msg)

mw.MotionWatchdog.__init__ = hooked_init
mw.MotionWatchdog._publish_authorized = hooked_pub_auth
sys.exit(mw.main())
"""
        rc, out, events = self._run_hooked_watchdog_subprocess(code)
        self.assertEqual(rc, 0, out)
        self.assertTrue(len(events) >= 1, out)
        nonzero_after_shutdown = [e for e in events if e[2] and (e[0] != 0.0 or e[1] != 0.0)]
        self.assertEqual(nonzero_after_shutdown, [], f'Non-zero velocity emitted after shutdown! events={events}\n{out}')
        self.assertEqual(events[-1], (0.0, 0.0, True), f'Final publish was not zero Twist: {events}\n{out}')

    def test_c2_sigterm_queued_while_inside_masked_publish_section(self):
        """Case C2: SIGTERM queued while inside _publish_authorized masked critical section is deferred until after publish, then emits final zero Twist."""
        code = """
import os, signal, sys
from geometry_msgs.msg import Twist
from limo_robot_agent import motion_watchdog as mw

orig_init = mw.MotionWatchdog.__init__

def hooked_init(self):
    orig_init(self)
    orig_pub = self.out.publish
    sent = {'done': False}
    def trace_pub(msg):
        if not sent['done'] and msg.linear.x != 0.0:
            sent['done'] = True
            # Signal arrives while _mask_stop_signals(True) is active during self.out.publish(msg)
            os.kill(os.getpid(), signal.SIGTERM)
        print(f'PUB_EVENT {msg.linear.x} {msg.angular.z} {self.shutting_down}', flush=True)
        return orig_pub(msg)
    self.out.publish = trace_pub
    def fire_once():
        self._one_shot.cancel()
        self.state = mw.AUTHORIZED
        self.input_ok = True
        m = Twist()
        m.linear.x = 0.2
        self.on_cmd(m)
    self._one_shot = self.create_timer(0.05, fire_once)

mw.MotionWatchdog.__init__ = hooked_init
sys.exit(mw.main())
"""
        rc, out, events = self._run_hooked_watchdog_subprocess(code)
        self.assertEqual(rc, 0, out)
        # Find the authorized publish event and confirm it happened with shutting_down=False
        auth_events = [e for e in events if e[0] == 0.2]
        self.assertEqual(auth_events, [(0.2, 0.0, False)], f'Expected authorized publish before signal delivery: {events}\n{out}')
        nonzero_after_shutdown = [e for e in events if e[2] and (e[0] != 0.0 or e[1] != 0.0)]
        self.assertEqual(nonzero_after_shutdown, [], f'Non-zero velocity emitted after shutdown! events={events}\n{out}')
        self.assertEqual(events[-1], (0.0, 0.0, True), f'Final publish was not zero Twist: {events}\n{out}')

    def test_e_consecutive_duplicate_sigterm_never_exits_minus_15(self):
        """Case E: Consecutive duplicate SIGTERM bursts across shutdown window always exit with rc=0 (never rc=-15)."""
        code = """
import os, signal, sys, threading, time
from limo_robot_agent import motion_watchdog as mw

orig_init = mw.MotionWatchdog.__init__
def hooked_init(self):
    orig_init(self)
    def fire_dup():
        self._dup_timer.cancel()
        pid = os.getpid()
        os.kill(pid, signal.SIGTERM)
        os.kill(pid, signal.SIGTERM)
        def spam():
            for _ in range(8):
                try:
                    os.kill(pid, signal.SIGTERM)
                except OSError:
                    break
                time.sleep(0.001)
        threading.Thread(target=spam, daemon=True).start()
    self._dup_timer = self.create_timer(0.05, fire_dup)

mw.MotionWatchdog.__init__ = hooked_init
sys.exit(mw.main())
"""
        for i in range(3):
            rc, out, _ = self._run_hooked_watchdog_subprocess(code)
            self.assertEqual(rc, 0, f'Iteration {i} failed with rc={rc}:\n{out}')
            self.assertIn('duplicate signal 15 (SIGTERM) ignored during shutdown', out)

    def test_f_sigint_clean_vs_abnormal_context_shutdown_and_exception(self):
        """Case F: SIGINT exits with rc=0; abnormal ROS context shutdown without signal and internal callback exception exit with rc=1."""
        # 1) SIGINT -> rc=0
        sigint_code = """
import os, signal, sys
from limo_robot_agent import motion_watchdog as mw
orig_init = mw.MotionWatchdog.__init__
def hooked_init(self):
    orig_init(self)
    self.create_timer(0.05, lambda: os.kill(os.getpid(), signal.SIGINT))
mw.MotionWatchdog.__init__ = hooked_init
sys.exit(mw.main())
"""
        rc_int, out_int, _ = self._run_hooked_watchdog_subprocess(sigint_code)
        self.assertEqual(rc_int, 0, out_int)
        self.assertIn('stopped cleanly (exit_code=0', out_int)

        # 2) Abnormal ROS context shutdown without stop signal -> rc=1
        ctx_code = """
import sys, rclpy
from limo_robot_agent import motion_watchdog as mw
orig_init = mw.MotionWatchdog.__init__
def hooked_init(self):
    orig_init(self)
    self.create_timer(0.05, lambda: rclpy.shutdown())
mw.MotionWatchdog.__init__ = hooked_init
sys.exit(mw.main())
"""
        rc_ctx, out_ctx, _ = self._run_hooked_watchdog_subprocess(ctx_code)
        self.assertEqual(rc_ctx, 1, out_ctx)
        self.assertIn('ABNORMAL_EXTERNAL_SHUTDOWN', out_ctx)

        # 3) Unhandled internal exception -> rc=1 (non-zero)
        exc_code = """
import sys
from limo_robot_agent import motion_watchdog as mw
orig_init = mw.MotionWatchdog.__init__
def hooked_init(self):
    orig_init(self)
    def boom():
        raise RuntimeError('SYNTHETIC_INTERNAL_WATCHDOG_FAULT')
    self.create_timer(0.05, boom)
mw.MotionWatchdog.__init__ = hooked_init
sys.exit(mw.main())
"""
        rc_exc, out_exc, _ = self._run_hooked_watchdog_subprocess(exc_code)
        self.assertNotEqual(rc_exc, 0, out_exc)
        self.assertIn('ABNORMAL_EXCEPTION: RuntimeError: SYNTHETIC_INTERNAL_WATCHDOG_FAULT', out_exc)

    def test_g_restart_produces_new_boot_id_and_initial_locked_state(self):
        """Case G: Restart after shutdown produces new boot_id, initial LOCKED state, and final zero Twist."""
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', ROS_DOMAIN_ID='231', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
        env['PYTHONPATH'] = f"{AGENT_SRC}:{env.get('PYTHONPATH', '')}"

        def run_once_and_observe():
            out_dir = Path(tempfile.mkdtemp(prefix='wd_restart_'))
            log = out_dir / 'wd.log'
            with open(log, 'w') as log_fp:
                proc = subprocess.Popen(
                    [sys.executable, '-m', 'limo_robot_agent.motion_watchdog'],
                    env=env, stdout=log_fp, stderr=subprocess.STDOUT, start_new_session=True,
                )
            statuses = []
            twists = []
            rclpy.init()
            try:
                obs = rclpy.create_node(f'wd_obs_{os.getpid()}_{time.monotonic_ns() % 100000}')
                obs.create_subscription(String, '/fleet/motion_watchdog/status', lambda m: statuses.append(json.loads(m.data)), 10)
                obs.create_subscription(Twist, '/cmd_vel_raw', lambda m: twists.append((m.linear.x, m.angular.z)), 10)
                t0 = time.monotonic()
                while time.monotonic() - t0 < 5.0 and not statuses:
                    rclpy.spin_once(obs, timeout_sec=0.05)
                os.kill(proc.pid, signal.SIGTERM)
                t1 = time.monotonic()
                while time.monotonic() - t1 < 2.0 and proc.poll() is None:
                    rclpy.spin_once(obs, timeout_sec=0.05)
                for _ in range(5):
                    rclpy.spin_once(obs, timeout_sec=0.02)
                rc = proc.wait(timeout=5.0)
                obs.destroy_node()
            finally:
                rclpy.try_shutdown()
                if proc.poll() is None:
                    proc.kill(); proc.wait()
            return rc, statuses, twists, log.read_text(errors='replace')

        rc1, st1, tw1, log1 = run_once_and_observe()
        rc2, st2, tw2, log2 = run_once_and_observe()
        self.assertEqual(rc1, 0, log1)
        self.assertEqual(rc2, 0, log2)
        self.assertTrue(st1 and st2)
        self.assertEqual(st1[0]['state'], 'LOCKED')
        self.assertEqual(st2[0]['state'], 'LOCKED')
        self.assertNotEqual(st1[0]['boot_id'], st2[0]['boot_id'])
        self.assertTrue(tw1 and tw1[-1] == (0.0, 0.0))
        self.assertTrue(tw2 and tw2[-1] == (0.0, 0.0))

    def test_h_codex_worker_thread_sigterm_before_native_publish(self):
        """Codex worker-thread SIGTERM counterexample: unmasked auxiliary OS thread receives SIGTERM
        right before native ROS Publisher.publish; Python signal handler runs on main thread without
        finalizing shutdown mid-publish, so native publish order is strictly 0.1(False) -> 0.0(True) -> 0.0(True)."""
        code = """
import os, signal, sys, threading
import rclpy
from geometry_msgs.msg import Twist
from limo_robot_agent import motion_watchdog as mw

real_cls = mw.MotionWatchdog
main_tid = threading.get_ident()
main_os_tid = threading.get_native_id()
info = {'worker_os_tid': None, 'handler_os_tid': None}

orig_signal = signal.signal
def wrapped_signal(sig, handler):
    if sig == signal.SIGTERM and callable(handler):
        def traced(signum, frame):
            info['handler_os_tid'] = threading.get_native_id()
            return handler(signum, frame)
        return orig_signal(sig, traced)
    return orig_signal(sig, handler)
signal.signal = wrapped_signal

class WorkerProbeWatchdog(real_cls):
    def __init__(self):
        super().__init__()
        self.state = mw.AUTHORIZED
        self.input_ok = True
        self.last_tick = None
        native_pub = self.out
        node = self
        class Boundary:
            topic_name = native_pub.topic_name
            def publish(self, msg):
                if msg.linear.x != 0.0:
                    node.kick.set()
                    assert node.done.wait(2.0)
                print(f'PUB_EVENT {msg.linear.x} {msg.angular.z} {node.shutting_down}', flush=True)
                return native_pub.publish(msg)
        self.out = Boundary()
        self.kick = threading.Event()
        self.done = threading.Event()
        def worker():
            info['worker_os_tid'] = threading.get_native_id()
            assert signal.SIGTERM not in signal.pthread_sigmask(signal.SIG_BLOCK, [])
            assert self.kick.wait(3.0)
            signal.pthread_kill(threading.get_ident(), signal.SIGTERM)
            self.done.set()
        self.worker = threading.Thread(target=worker)
        self.worker.start()
        self.probe_timer = self.create_timer(0.01, self.drive)

    def drive(self):
        self.probe_timer.cancel()
        self.state = mw.AUTHORIZED
        self.input_ok = True
        m = Twist()
        m.linear.x = 0.1
        self.on_cmd(m)

mw.MotionWatchdog = WorkerProbeWatchdog
rc = mw.main()
print(f'TIDS main={main_os_tid} worker={info["worker_os_tid"]} handler={info["handler_os_tid"]}', flush=True)
assert info['worker_os_tid'] != main_os_tid
assert info['handler_os_tid'] == main_os_tid
assert signal.getsignal(signal.SIGTERM) == signal.SIG_IGN
assert signal.getsignal(signal.SIGINT) == signal.SIG_IGN
os.kill(os.getpid(), signal.SIGTERM)
os.kill(os.getpid(), signal.SIGINT)
sys.exit(rc)
"""
        rc, out, events = self._run_hooked_watchdog_subprocess(code)
        self.assertEqual(rc, 0, out)
        # Verify exact native Publisher ordering: no non-zero velocity after first shutting_down=True event
        first_sd_idx = next((i for i, e in enumerate(events) if e[2]), None)
        self.assertIsNotNone(first_sd_idx, f'No shutting_down=True publish observed: {events}\n{out}')
        post_sd = events[first_sd_idx:]
        post_sd_nonzero = [e for e in post_sd if e[0] != 0.0 or e[1] != 0.0]
        self.assertEqual(post_sd_nonzero, [], f'Non-zero velocity emitted after shutdown! events={events}\n{out}')
        self.assertEqual(events[-1], (0.0, 0.0, True), out)
        self.assertIn('TIDS main=', out)

    def test_i_repeated_concurrent_multithread_publish_vs_shutdown(self):
        """Repeated multi-threaded race: concurrent on_cmd() and request_shutdown() across 20 rounds
        must never emit any non-zero velocity after shutting_down=True at the native ROS Publisher."""
        import threading
        os.environ['ROS_DOMAIN_ID'] = '231'
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        try:
            for round_idx in range(20):
                node = mw.MotionWatchdog()
                native_pub = node.out
                observed = []
                obs_lock = threading.Lock()

                class ObservedPublisher:
                    topic_name = native_pub.topic_name
                    def publish(self, msg):
                        with obs_lock:
                            observed.append((msg.linear.x, msg.angular.z, node.shutting_down))
                        return native_pub.publish(msg)

                node.out = ObservedPublisher()
                node.state = mw.AUTHORIZED
                node.input_ok = True
                start_barrier = threading.Barrier(2)

                def publisher_worker():
                    start_barrier.wait()
                    cmd = Twist()
                    cmd.linear.x = 0.1
                    for _ in range(50):
                        node.on_cmd(cmd)

                def shutdown_worker():
                    start_barrier.wait()
                    if round_idx % 2 == 1:
                        time.sleep(0.0002)
                    node.request_shutdown('CONCURRENT_SHUTDOWN')

                t1 = threading.Thread(target=publisher_worker)
                t2 = threading.Thread(target=shutdown_worker)
                t1.start(); t2.start()
                t1.join(timeout=5.0); t2.join(timeout=5.0)
                node.destroy_node()

                first_sd = next((i for i, e in enumerate(observed) if e[2]), None)
                self.assertIsNotNone(first_sd, f'Round {round_idx}: missing shutdown zero-publish: {observed}')
                post_sd_nonzero = [e for e in observed[first_sd:] if e[0] != 0.0 or e[1] != 0.0]
                self.assertEqual(post_sd_nonzero, [], f'Round {round_idx}: non-zero publish after shutdown! {observed}')
                self.assertEqual(observed[-1], (0.0, 0.0, True), f'Round {round_idx}: final publish not zero: {observed}')
        finally:
            rclpy.try_shutdown()


@unittest.skipUnless(ROS_AVAILABLE, f'ROS 2 python packages unavailable: {ROS_ERR if not ROS_AVAILABLE else ""}')
class TestR2ShutdownMatrices(unittest.TestCase):
    def _run_matrix(self, target, domain, port):
        out = Path(tempfile.mkdtemp(prefix=f'r2_{target}_')) / 'm'
        cmd = [
            sys.executable, str(HERE / 'agent_shutdown_matrix_20261122.py'),
            '--target', target,
            '--agent-src', str(AGENT_SRC),
            '--out', str(out),
            '--domain', str(domain),
            '--port', str(port),
        ]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
        data = json.loads((out / 'result.json').read_text())
        self.assertEqual(r.returncode, 0, r.stdout + '\n' + r.stderr)
        self.assertEqual(data['summary']['FAIL'], 0, json.dumps(data['results']))
        self.assertEqual(data['summary']['SETUP_FAIL'], 0, json.dumps(data['results']))
        return data

    def test_motion_watchdog_matrix(self):
        data = self._run_matrix('motion_watchdog', 225, 18297)
        self.assertEqual(data['summary']['PASS'], 6)

    def test_mock_robot_matrix(self):
        data = self._run_matrix('mock_robot', 226, 18298)
        self.assertEqual(data['summary']['PASS'], 3)

    def test_supervisor_matrix(self):
        data = self._run_matrix('supervisor', 227, 18299)
        self.assertEqual(data['summary']['PASS'], 3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
