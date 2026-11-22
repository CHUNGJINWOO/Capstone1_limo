"""Robot Agent entry point. ROS parameters:
  server_url   http://<server tailscale ip>:8080     token_file  file with this robot's agent token (0600)
  robot_id     pcu | mock1 ...                        mode        real | mock | sim      simulated  true for mock robots (shown as SIMULATED)
  allow_missions  false (default) -> every mission is REJECTED (MISSIONS_DISABLED); true -> missions go to Nav2 if the interlocks pass
  allow_sim_safety_gate false; explicit sim-only single Safety Gate publisher opt-in (also requires use_sim_time)
  state_dir    where the de-duplication file is kept (default ~/.local/state/limo_robot_agent)"""
import os, signal, socket, sys, threading, time, uuid

import rclpy
from rclpy.executors import MultiThreadedExecutor

from . import core
from .ros_adapter import RosRobot


def _install_stop_handlers(agent):
    """SIGINT / SIGTERM -> orderly stop of the Agent loop (2026-11-22).

    rclpy.init() installs C-level handlers that only shut the ROS context down. A SIGINT that was inherited as SIG_IGN (what `nohup cmd &`
    from a non-interactive shell gives) also never raises KeyboardInterrupt. The Agent's main loop then kept retrying on RCLError forever
    (real LIMO, 2026-11-22: only SIGKILL stopped it), or, with a normal SIGINT, left through a traceback in announce_shutdown().
    These handlers replace both: the first signal stops the loop, a second one restores the default action. A watcher also stops the Agent
    when the ROS context is shut down by anything else. The loop can still be inside one HTTP call (timeout <= 20 s) when the signal arrives.
    """
    hits = []

    def handler(signum, _frame):
        hits.append(signum)
        if len(hits) == 1:
            agent.log(f'signal {signum} received: stopping')
            agent.stop()
        else:
            signal.signal(signum, signal.SIG_DFL)
            os.kill(os.getpid(), signum)

    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, handler)

    def watch():
        while not agent.stop_evt.wait(0.5):
            if not rclpy.ok():
                agent.log('ROS context was shut down: stopping')
                agent.stop()

    threading.Thread(target=watch, name='agent-ros-watch', daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    tmp = rclpy.create_node('fleet_agent_params')
    for n, d in (('server_url', 'http://127.0.0.1:8080'), ('token_file', ''), ('robot_id', 'pcu'), ('mode', 'real'), ('simulated', False), ('allow_missions', False), ('state_dir', os.path.expanduser('~/.local/state/limo_robot_agent')), ('period_s', 1.0), ('allow_sim_safety_gate', False), ('cancel_result_timeout_s', 5.0), ('disconnect_cancel_after_s', 3.0), ('recording_enabled', False), ('recording_root', ''), ('recorder_script', ''), ('verifier_script', '')):
        tmp.declare_parameter(n, d)
    tmp.declare_parameter('slam_sim_enabled', False)
    tmp.declare_parameter('slam_root', '')
    tmp.declare_parameter('manual_mapping_enabled', False)
    tmp.declare_parameter('manual_web_bench', False)
    tmp.declare_parameter('manual_locked_arbiter', False)
    tmp.declare_parameter('navigation_localization', False)   # saved-map map_server + AMCL worker (isolated mock/sim only)
    tmp.declare_parameter('auto_mapping_enabled', False)
    tmp.declare_parameter('sim_r8_supervisor', False)
    tmp.declare_parameter('web_mapping_enabled', False)     # web START/PAUSE/RESUME/SAVE/STOP worker (SIM/mock, isolated ROS only)
    tmp.declare_parameter('web_mapping_slam_params', '')
    tmp.declare_parameter('map_save_enabled', False)        # web "save current SLAM map + register" (save only; needs missions disabled)
    tmp.declare_parameter('web_gate_reset', False)           # admin web Gate reset worker (needs Gate r12 reset_readiness; isolated sim only)
    tmp.declare_parameter('require_motion_watchdog', False)  # mapping goals need a fresh OK watchdog (separate process)
    for name, default in (('joy_linear_axis', 1), ('joy_angular_axis', 0), ('joy_deadman_button', 4), ('joy_linear_limit', 0.15), ('joy_angular_limit', 0.4), ('joy_timeout_s', 0.25)):
        tmp.declare_parameter(name, default)
    tmp.declare_parameter('real_locked_requested', False)
    g = lambda n: tmp.get_parameter(n).value
    server_url, token_file, rid, mode, sim, allow, sdir, period = g('server_url'), g('token_file'), g('robot_id'), g('mode'), bool(g('simulated')), bool(g('allow_missions')), g('state_dir'), float(g('period_s'))
    # 2026-11-22: configuration-error guards (the later per-feature guards compare `mode` exactly, so a mistyped mode must not pass).
    if mode not in ('real', 'sim', 'mock', 'real_locked'):
        sys.exit(f"mode must be exactly 'real', 'sim' or 'mock' (got {mode!r})")
    if mode == 'real' and sim:
        sys.exit("mode 'real' cannot be combined with simulated:=true")
    if mode == 'real' and allow:
        # Missions on the real robot stay blocked until independent physical motor-isolation evidence exists (see real_locked_policy.py).
        # Today's refusals are the server (robot_not_ready), the Gate HEAD fields and GATE_ATTESTATION_UNAVAILABLE; this is the Agent's own.
        sys.exit('allow_missions is blocked in real mode (no approved physical motor-isolation evidence)')
    if mode in ('sim', 'mock') and (os.environ.get('ROS_DOMAIN_ID') == '77' or os.environ.get('ROS_DISCOVERY_SERVER')):
        sys.exit('a sim/mock Agent must not run on the real robot graph (ROS_DOMAIN_ID 77 or a Discovery Server)')
    g_sim_gate = g('allow_sim_safety_gate')
    cancel_timeout = float(g('cancel_result_timeout_s'))
    disconnect_timeout = float(g('disconnect_cancel_after_s'))
    recording_enabled = bool(g('recording_enabled'))
    recording_root, recorder_script, verifier_script = g('recording_root'), g('recorder_script'), g('verifier_script')
    slam_sim_enabled, slam_root = bool(g('slam_sim_enabled')), g('slam_root')
    manual_enabled = bool(g('manual_mapping_enabled'))
    web_bench_enabled = bool(g('manual_web_bench'))
    locked_arbiter = bool(g('manual_locked_arbiter'))
    nav_loc_enabled = bool(g('navigation_localization'))
    if nav_loc_enabled and (mode not in ('sim', 'mock') or not sim or os.environ.get('ROS_LOCALHOST_ONLY') != '1'
                            or os.environ.get('ROS_DOMAIN_ID') in (None, '', '0', '77')):
        sys.exit('navigation_localization requires SIM/mock simulated mode on an isolated localhost ROS domain')
    auto_enabled = bool(g('auto_mapping_enabled'))
    if auto_enabled and (manual_enabled or not slam_sim_enabled or mode != 'sim' or not sim or allow or os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('CAPSTONE_GAZEBO_SIM') != '1' or os.environ.get('ROS_DOMAIN_ID') in (None, '', '0')):
        sys.exit('auto_mapping_enabled requires isolated SIM_ONLY SLAM, disabled missions and no manual input')
    sim_r8_supervisor = bool(g('sim_r8_supervisor'))
    if sim_r8_supervisor and not ((manual_enabled or auto_enabled) and mode == 'sim' and sim and not allow and bool(g_sim_gate)):
        sys.exit('sim_r8_supervisor requires mapping SIM_ONLY, disabled missions and allow_sim_safety_gate')
    joy_values = [g(n) for n in ('joy_linear_axis', 'joy_angular_axis', 'joy_deadman_button', 'joy_linear_limit', 'joy_angular_limit', 'joy_timeout_s')]
    if manual_enabled and not slam_sim_enabled:
        sys.exit('manual_mapping_enabled currently requires isolated SIM_ONLY SLAM control')
    real_locked_requested = bool(g('real_locked_requested'))
    g_require_watchdog = bool(g('require_motion_watchdog'))
    g_gate_reset = bool(g('web_gate_reset'))
    if g_gate_reset and (mode not in ('sim', 'mock') or not sim or os.environ.get('ROS_LOCALHOST_ONLY') != '1'
                         or os.environ.get('ROS_DOMAIN_ID') in (None, '', '0', '77')):
        # 2026-11-22: explicit real-mode block (was only indirectly refused by the Gate reset_readiness / locked-graph checks).
        sys.exit('web_gate_reset requires SIM/mock simulated mode on an isolated localhost ROS domain (blocked on the real robot)')
    web_mapping_enabled, web_mapping_slam_params = bool(g('web_mapping_enabled')), g('web_mapping_slam_params')
    map_save_enabled = bool(g('map_save_enabled'))
    if map_save_enabled and allow:
        sys.exit('map_save_enabled requires allow_missions:=false (save only, no motion paths)')
    if web_mapping_enabled and (mode not in ('sim', 'mock') or not sim or slam_sim_enabled or manual_enabled or auto_enabled
                                or os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') in (None, '', '0')
                                or sys.platform != 'linux'):
        sys.exit('web_mapping_enabled requires SIM/mock simulated mode, isolated localhost ROS and no other SLAM control')
    if locked_arbiter:
        from urllib.parse import urlsplit
        endpoint = urlsplit(server_url)
        # Isolated mock only: the Arbiter is the single velocity source for MANUAL / MAPPING / NAVIGATION, so the
        # web Mapping Worker and (operating-mode gated) missions may share this Agent. Gate output is /cmd_vel_locked.
        if (not web_bench_enabled or mode != 'mock' or not sim or manual_enabled or auto_enabled
                or slam_sim_enabled or recording_enabled or real_locked_requested
                or not g_require_watchdog or os.environ.get('ROS_LOCALHOST_ONLY') != '1'
                or os.environ.get('ROS_DOMAIN_ID') in (None, '', '0', '77')
                or os.environ.get('ROS_DISCOVERY_SERVER')
                or endpoint.scheme != 'http' or endpoint.hostname not in ('127.0.0.1', 'localhost')
                or endpoint.port in (None, 8080, 18190, 18191, 18192)):
            sys.exit('manual_locked_arbiter requires isolated mock, localhost HTTP and independent Watchdog')
    elif web_bench_enabled and (mode != 'real' or sim or allow or manual_enabled or auto_enabled
                                or real_locked_requested or recording_enabled or slam_sim_enabled
                                or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        sys.exit('manual_web_bench requires a local real Agent with motion and recording disabled')
    tmp.destroy_node()
    from .real_locked_policy import require_available
    try:
        require_available(mode=mode, requested=real_locked_requested, recording_enabled=recording_enabled)
    except RuntimeError as exc:
        if rclpy.ok(): rclpy.shutdown()
        sys.exit(str(exc))
    if not token_file or not os.path.isfile(token_file): sys.exit('token_file is required')
    if os.stat(token_file).st_mode & 0o077: print('WARNING: token file is readable by others; chmod 600 it', flush=True)
    token = open(token_file).read().strip()
    boot_id = uuid.uuid4().hex[:12]
    robot = RosRobot(
        allow_missions=allow,
        mode=mode,
        simulated=sim,
        allow_sim_safety_gate=bool(g_sim_gate),
        sim_r8_supervisor=sim_r8_supervisor,
        robot_id=rid,
        agent_boot_id=boot_id,
        cancel_result_timeout_s=cancel_timeout,
        disconnect_cancel_after_s=disconnect_timeout,
    )
    mapping = None
    arbiter = None
    savew = None
    recording = None
    if recording_enabled:
        if not recording_root or not recorder_script or not verifier_script:
            sys.exit('recording requires recording_root, recorder_script and verifier_script')
        from .recording_control import RecordingControl
        recording = RecordingControl(recording_root, recorder_script, verifier_script)
        robot.recording_command = recording.command
    slam = None
    if slam_sim_enabled:
        if mode != 'sim' or not sim or allow or not slam_root or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            sys.exit('slam_sim_enabled requires SIM_ONLY, disabled missions, isolated ROS and slam_root')
        from .slam_control import SlamControl
        slam = SlamControl(slam_root, robot)
        robot.slam_command = slam.command
    manual = None
    if manual_enabled:
        from .joystick_filter import JoystickConfig
        from .manual_mapping_input import ManualMappingInput
        manual = ManualMappingInput(robot, slam, JoystickConfig(*joy_values))
        slam.enable_manual_stop()
    ros_saver = None
    if map_save_enabled:        # the serialize_map client must exist BEFORE the executor spins: adding it afterwards stopped every subscription on the real robot
        from .map_save_worker import RosSaver
        ros_saver = RosSaver(robot)
    ex = MultiThreadedExecutor(num_threads=4); ex.add_node(robot)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()
    info = {'agent_version': '0.1.0', 'boot_id': boot_id, 'host': socket.gethostname(), 'mode': mode, 'simulated': sim, 'missions_enabled': allow, 'recording_enabled': recording_enabled,
            'interlocks': ['missions_enabled', 'safety_context_v1', 'safety_gate_contract', 'server_lease', 'nav2_action_server_ready', 'no_publisher_on_/cmd_vel'], 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID')}
    agent = core.AgentCore(
        rid,
        server_url,
        token,
        lambda: dict(robot.snapshot(), map_save=savew.snapshot() if savew else None, mapping_worker=mapping.snapshot() if mapping else {'enabled': False, 'status': 'UNAVAILABLE'}, manual_web_bench=bench.snapshot() if bench else None, gate_reset=gresetw.snapshot() if gresetw else {'enabled': False}, localization=locw.snapshot() if locw else {'enabled': False, 'status': 'UNAVAILABLE'}, velocity_arbiter=arbiter.snapshot() if arbiter else None, joystick=manual.snapshot() if manual else {'enabled': False, 'connected': False}, mapping_mode=(('MANUAL_MAPPING' if slam.snapshot().get('status') == 'RUNNING' else slam.snapshot().get('status')) if manual else 'IDLE'), recording=recording.snapshot() if recording else {'enabled': False, 'status': 'UNAVAILABLE'}, slam=slam.snapshot() if slam else {'enabled': False, 'status': 'UNAVAILABLE'}),
        robot,
        dict(info, gate_reset=g_gate_reset, navigation_localization=nav_loc_enabled, manual_web_bench=web_bench_enabled, locked_velocity_arbiter=locked_arbiter, mapping_worker=web_mapping_enabled, map_save=map_save_enabled),
        sdir,
        map_fn=robot.map_for_upload,
        period_s=period,
        log=lambda m: print(time.strftime('%H:%M:%S'), 'agent:', m, flush=True),
        require_safety_context=True,
    )
    robot.execution_allowed = agent.is_execution_allowed
    robot.require_motion_watchdog = bool(g_require_watchdog)
    robot.locked_arbiter = bool(locked_arbiter)
    if web_mapping_enabled:
        from .mapping_worker import MappingWorker
        mapping = MappingWorker(robot, agent, sdir, slam_params=web_mapping_slam_params or None)
    if map_save_enabled:
        from .map_save_worker import MapSaveWorker
        savew = MapSaveWorker(agent, sdir, ros_saver)
    robot.session_fn = agent.session_snapshot
    robot.on_command_meta = agent.record_command_meta
    locw = None
    if nav_loc_enabled:
        from .localization_worker import LocalizationWorker
        locw = LocalizationWorker(robot, agent, sdir)
    bench = None
    if web_bench_enabled:
        from .manual_web_bench import ManualWebBench
        bench = ManualWebBench(robot, agent, isolated_arbiter=locked_arbiter)
    arbiter = None
    if locked_arbiter:
        from .velocity_arbiter import VelocityArbiter
        arbiter = VelocityArbiter(robot, agent, bench)
    gresetw = None
    if g_gate_reset:                                   # 2026-11-21: admin web Gate reset (server-delivered, Agent- and Gate-checked)
        from .gate_reset_worker import GateResetWorker
        gresetw = GateResetWorker(robot, agent, sdir, arbiter=arbiter, bench=bench)
    if slam:
        def upload_slam_map(sid, floor, command_id, yaml_path, pgm_path, graph_prefix):
            import base64
            if yaml_path.stat().st_size + pgm_path.stat().st_size >= 3_800_000:
                raise ValueError('map exceeds upload limit')
            payload={'robot_id': rid, 'agent_boot_id': boot_id, 'command_id': command_id,
                'session_id': sid, 'floor_id': floor, 'yaml_b64': base64.b64encode(yaml_path.read_bytes()).decode('ascii'),
                'pgm_b64': base64.b64encode(pgm_path.read_bytes()).decode('ascii')}
            if graph_prefix:
                graph=graph_prefix.with_suffix('.posegraph'); data=graph_prefix.with_suffix('.data')
                if graph.stat().st_size+data.stat().st_size+pgm_path.stat().st_size+yaml_path.stat().st_size < 3_800_000:
                    payload.update(posegraph_b64=base64.b64encode(graph.read_bytes()).decode('ascii'),posegraph_data_b64=base64.b64encode(data.read_bytes()).decode('ascii'))
            result=agent.http.post('/agent/slam-map', payload, timeout=20)
            import hashlib
            checks=(result.get('map_version') or {}).get('sha256') or {}
            if (not (result.get('integrity') or {}).get('ok')
                    or checks.get('yaml_sha256') != hashlib.sha256(yaml_path.read_bytes()).hexdigest()
                    or checks.get('pgm_sha256') != hashlib.sha256(pgm_path.read_bytes()).hexdigest()):
                raise RuntimeError('registry bundle checksum mismatch')
            return result
        slam.upload_map = upload_slam_map
    print(f'robot agent {rid} mode={mode} simulated={sim} missions_enabled={allow} -> {server_url}', flush=True)
    _install_stop_handlers(agent)
    try: agent.run()
    except KeyboardInterrupt: pass
    finally:
        def step(what, fn):
            # every step runs even if an earlier one fails (e.g. the ROS context already shut down)
            try: fn()
            except Exception as exc: print(f'shutdown: {what} failed ({type(exc).__name__}: {exc})', flush=True)
        agent.stop()
        if rclpy.ok(): step('announce_shutdown', robot.announce_shutdown)
        if mapping: step('mapping.close', mapping.close)
        if manual: step('manual.close', manual.close)
        step('executor.shutdown', ex.shutdown)
        spin_thread.join(timeout=5.0)
        if spin_thread.is_alive(): print('shutdown: executor thread still running after 5 s', flush=True)
        if arbiter: step('arbiter.close', arbiter.close)
        if locw: step('localization.close', locw.close)
        if gresetw: step('gate_reset.close', gresetw.close)
        if bench: step('bench.close', bench.close)
        step('destroy_node', robot.destroy_node)
        if recording: step('recording.close', recording.close)
        if slam: step('slam.close', slam.close)
        if rclpy.ok(): step('rclpy.shutdown', rclpy.shutdown)
        print('robot agent stopped', flush=True)


if __name__ == '__main__':
    main()
