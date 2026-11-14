"""Robot Agent entry point. ROS parameters:
  server_url   http://<server tailscale ip>:8080     token_file  file with this robot's agent token (0600)
  robot_id     pcu | mock1 ...                        mode        real | mock | sim      simulated  true for mock robots (shown as SIMULATED)
  allow_missions  false (default) -> every mission is REJECTED (MISSIONS_DISABLED); true -> missions go to Nav2 if the interlocks pass
  allow_sim_safety_gate false; explicit sim-only single Safety Gate publisher opt-in (also requires use_sim_time)
  state_dir    where the de-duplication file is kept (default ~/.local/state/limo_robot_agent)"""
import os, socket, sys, threading, time, uuid

import rclpy
from rclpy.executors import MultiThreadedExecutor

from . import core
from .ros_adapter import RosRobot


def main(args=None):
    rclpy.init(args=args)
    tmp = rclpy.create_node('fleet_agent_params')
    for n, d in (('server_url', 'http://127.0.0.1:8080'), ('token_file', ''), ('robot_id', 'pcu'), ('mode', 'real'), ('simulated', False), ('allow_missions', False), ('state_dir', os.path.expanduser('~/.local/state/limo_robot_agent')), ('period_s', 1.0), ('allow_sim_safety_gate', False)):
        tmp.declare_parameter(n, d)
    g = lambda n: tmp.get_parameter(n).value
    server_url, token_file, rid, mode, sim, allow, sdir, period = g('server_url'), g('token_file'), g('robot_id'), g('mode'), bool(g('simulated')), bool(g('allow_missions')), g('state_dir'), float(g('period_s'))
    g_sim_gate = g('allow_sim_safety_gate')
    tmp.destroy_node()
    if not token_file or not os.path.isfile(token_file): sys.exit('token_file is required')
    if os.stat(token_file).st_mode & 0o077: print('WARNING: token file is readable by others; chmod 600 it', flush=True)
    token = open(token_file).read().strip()
    robot = RosRobot(allow_missions=allow, mode=mode, simulated=sim, allow_sim_safety_gate=bool(g_sim_gate))
    ex = MultiThreadedExecutor(num_threads=4); ex.add_node(robot)
    threading.Thread(target=ex.spin, daemon=True).start()
    info = {'agent_version': '0.1.0', 'boot_id': uuid.uuid4().hex[:12], 'host': socket.gethostname(), 'mode': mode, 'simulated': sim, 'missions_enabled': allow,
            'interlocks': ['missions_enabled', 'nav2_action_server_ready', 'no_publisher_on_/cmd_vel'], 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID')}
    agent = core.AgentCore(rid, server_url, token, robot.snapshot, robot, info, sdir, map_fn=robot.map_for_upload, period_s=period, log=lambda m: print(time.strftime('%H:%M:%S'), 'agent:', m, flush=True))
    robot.execution_allowed = lambda: agent.connected
    print(f'robot agent {rid} mode={mode} simulated={sim} missions_enabled={allow} -> {server_url}', flush=True)
    try: agent.run()
    except KeyboardInterrupt: pass
    finally:
        agent.stop(); ex.shutdown(); robot.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
