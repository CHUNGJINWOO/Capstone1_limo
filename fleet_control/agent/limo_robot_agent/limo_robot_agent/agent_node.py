"""Robot Agent entry point. ROS parameters:
  server_url   http://<server tailscale ip>:8080     token_file  file with this robot's agent token (0600)
  robot_id     pcu | mock1 ...                        mode        real | mock | sim      simulated  true for mock robots (shown as SIMULATED)
  allow_missions  false (default) -> every mission is REJECTED (MISSIONS_DISABLED); true -> missions go to Nav2 if the interlocks pass
  allow_sim_safety_gate false; explicit sim-only single Safety Gate publisher opt-in (also requires use_sim_time)
  state_dir    where the de-duplication file is kept (default ~/.local/state/limo_robot_agent)"""
import os, signal, socket, sys, threading, time, uuid

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from . import core
from .ros_adapter import RosRobot
from .manual_web_bench import ManualWebBench
from .bag_worker import BagWorker


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    tmp = rclpy.create_node('fleet_agent_params')
    for n, d in (('server_url', 'http://127.0.0.1:8080'), ('token_file', ''), ('robot_id', 'pcu'), ('mode', 'real'), ('simulated', False), ('allow_missions', False), ('state_dir', os.path.expanduser('~/.local/state/limo_robot_agent')), ('period_s', 1.0), ('allow_sim_safety_gate', False), ('manual_web_bench', False), ('isolated_ros_bag', False), ('isolated_ros_mapping', False), ('locked_velocity_arbiter', False), ('camera_preview', False),
                 ('rgb_topic', '/camera/color/image_raw'), ('depth_topic', '/camera/depth/image_raw')):
        tmp.declare_parameter(n, d)
    g = lambda n: tmp.get_parameter(n).value
    server_url, token_file, rid, mode, sim, allow, sdir, period = g('server_url'), g('token_file'), g('robot_id'), g('mode'), bool(g('simulated')), bool(g('allow_missions')), g('state_dir'), float(g('period_s'))
    g_sim_gate = g('allow_sim_safety_gate')
    bench_enabled = g('manual_web_bench')
    bag_enabled = g('isolated_ros_bag')
    mapping_enabled = g('isolated_ros_mapping')
    velocity_enabled = g('locked_velocity_arbiter')
    camera_enabled = g('camera_preview')
    rgb_topic, depth_topic = g('rgb_topic'), g('depth_topic')
    tmp.destroy_node()
    local = sys.platform.startswith('linux') and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    if (mapping_enabled or velocity_enabled) and (not local or not sim or mode != 'mock' or g_sim_gate):
        sys.exit('isolated mapping/velocity requires Linux, localhost ROS, mock and simulated mode')
    if allow and velocity_enabled:
        sys.exit('locked_velocity_arbiter requires missions disabled')
    if bench_enabled and (mode != 'mock' or not sim or allow):
        sys.exit('manual_web_bench requires mock mode, simulated=true and allow_missions=false')
    if bag_enabled and (mode != 'mock' or not sim or allow or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        sys.exit('isolated_ros_bag requires mock, simulated, missions disabled and ROS_LOCALHOST_ONLY=1')
    if bag_enabled and not sys.platform.startswith('linux'):
        sys.exit('isolated_ros_bag requires Linux process supervision')
    if not token_file or not os.path.isfile(token_file): sys.exit('token_file is required')
    if os.stat(token_file).st_mode & 0o077: print('WARNING: token file is readable by others; chmod 600 it', flush=True)
    token = open(token_file).read().strip()
    robot = RosRobot(allow_missions=allow, mode=mode, simulated=sim, allow_sim_safety_gate=bool(g_sim_gate))
    ex = MultiThreadedExecutor(num_threads=4); ex.add_node(robot)
    info = {'agent_version': '0.1.0', 'boot_id': uuid.uuid4().hex[:12], 'host': socket.gethostname(), 'mode': mode, 'simulated': sim, 'missions_enabled': allow,
            'interlocks': ['missions_enabled', 'nav2_action_server_ready', 'no_publisher_on_/cmd_vel'], 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID')}
    info['manual_web_bench'] = bench_enabled
    info['isolated_ros_bag'] = bag_enabled
    info['isolated_ros_mapping'] = mapping_enabled
    info['locked_velocity_arbiter'] = velocity_enabled
    info['camera_preview'] = camera_enabled
    bench = None
    bag_worker = None
    mapping_worker = None
    velocity = None
    camera = None
    def snapshot():
        state = robot.snapshot()
        if bench:
            state['manual_bench'] = bench.snapshot()
        if bag_worker:
            state['bag_worker'] = bag_worker.snapshot()
        if mapping_worker:
            state['mapping_worker'] = mapping_worker.snapshot()
        if velocity:
            state['velocity_arbiter'] = velocity.snapshot()
        return state
    agent = core.AgentCore(rid, server_url, token, snapshot, robot, info, sdir, map_fn=robot.map_for_upload, period_s=period, log=lambda m: print(time.strftime('%H:%M:%S'), 'agent:', m, flush=True))
    robot.execution_allowed = lambda: agent.connected and not agent.stop_evt.is_set()
    if mapping_enabled:
        from .mapping_worker import MappingWorker
        mapping_worker = MappingWorker(robot, agent, sdir)
    if bag_enabled:
        bag_worker = BagWorker(robot, agent, sdir)
    if bench_enabled:
        bench = ManualWebBench(robot, agent)
    if velocity_enabled:
        from .velocity_arbiter import VelocityArbiter
        velocity = VelocityArbiter(robot, agent, bench)
    if camera_enabled:
        from .camera_preview import CameraPreview
        camera = CameraPreview(robot, agent, rgb_topic, depth_topic)
    def stop_agent(*_):
        agent.stop()
        agent.connected = False

    previous_handlers = {sig: signal.signal(sig, stop_agent) for sig in (signal.SIGINT, signal.SIGTERM)}
    threading.Thread(target=ex.spin, daemon=True).start()
    print(f'robot agent {rid} mode={mode} simulated={sim} missions_enabled={allow} -> {server_url}', flush=True)
    try: agent.run()
    except KeyboardInterrupt: pass
    finally:
        stop_agent()
        if camera:
            camera.close()
        if mapping_worker:
            mapping_worker.close()
        if bag_worker:
            bag_worker.close()
        if bench:
            bench.close()
        ex.shutdown(); robot.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
