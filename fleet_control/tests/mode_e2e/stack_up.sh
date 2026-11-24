#!/bin/bash
# Isolated operating-mode E2E stack (no hardware, Gate output /cmd_vel_locked, no /cmd_vel).
# usage: OVERLAY=<explore_lite overlay install> stack_up.sh NEW_RUN_DIR PORT ROS_DOMAIN_ID
#   Fleet server (--require-operating-mode) | kinematic sim | ICP odom + Nav2 (slow LiDAR profile) + locked Gate (ARBITER=1)
#   | Motion Watchdog | Agent (mock, locked arbiter + web bench + web mapping worker + missions, map/loc FIXTURE wrapper)
# Individual processes can be restarted with: stack_proc.sh RUN_DIR start|kill NAME
set -e
[ "${REAL_SHAPE:-0}" = 1 ] && exec "$(dirname "$0")/stack_up_real_shape.sh" "$@"     # G2 2026-11-26: the real-Agent-shape variant (no Arbiter)
RUN=$1; PORT=$2; DOM=$3; HERE=$(cd "$(dirname "$0")" && pwd); FC=$(cd "$HERE/../.." && pwd); REPO=$(cd "$FC/.." && pwd)
test -n "$OVERLAY" -a -f "$OVERLAY/setup.bash" || { echo "set OVERLAY to the explore_lite overlay install dir" >&2; exit 2; }
case " 8080 18190 18191 18192 " in *" $PORT "*) echo "protected port" >&2; exit 2;; esac
mkdir "$RUN"; : > $RUN/pgids
CAP_SETUP="$REPO/capstone_ws/install/setup.bash"
[ ! -f "$CAP_SETUP" ] && [ -f "/home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/limo_msgs/share/limo_msgs/local_setup.bash" ] && CAP_SETUP="/home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/limo_msgs/share/limo_msgs/local_setup.bash"
DEFAULT_GATE="$REPO/experiments/safety_chain_fix_20261113/candidate_gate_r12/nodes/stall_detector_node.py"
[ -f "/home/cjinw/school/2026-2/Capstone1_limo/local_data/raw_results/gate_r13_integ_claude_20261122/candidate_r13/stall_detector_node.py" ] && DEFAULT_GATE="/home/cjinw/school/2026-2/Capstone1_limo/local_data/raw_results/gate_r13_integ_claude_20261122/candidate_r13/stall_detector_node.py"
cat > $RUN/env.sh <<EOF
source /opt/ros/humble/setup.bash; [ -f "$CAP_SETUP" ] && source "$CAP_SETUP"; source $OVERLAY/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=$DOM ROS2CLI_NO_DAEMON=1 ROS_LOG_DIR=$RUN/roslog ARBITER=1 WATCHDOG=1 LIFECYCLE_DELAY_S=5 GATE_SCRIPT=${GATE_SCRIPT:-$DEFAULT_GATE}
export GATE_RESET_PEER=${GATE_RESET_PEER:-} GATE_INPUT_EXCL=${GATE_INPUT_EXCL:-fleet_motion_watchdog} GATE_INPUT_MAX_VX=${GATE_INPUT_MAX_VX:-0.20} GATE_INPUT_MAX_WZ=${GATE_INPUT_MAX_WZ:-0.60} GATE_WRITER_STATUS=${GATE_WRITER_STATUS:-/fleet/motion_watchdog/status}
unset ROS_DISCOVERY_SERVER
export PYTHONPATH=$FC/agent/limo_robot_agent:$FC/server:\$PYTHONPATH
EOF
# SIM_BACKEND=gazebo (2026-11-21, SIMULATION ONLY): Ignition Fortress Office world + LIMO model instead of kinematic_sim.py.
# Ignition transport is confined to IGN_IP=127.0.0.1 and this run's IGN_PARTITION; the stack nodes and the Agent use /clock.
GZ=0; [ "${SIM_BACKEND:-kinematic}" = "gazebo" ] && GZ=1
if [ $GZ = 1 ]; then
  test -n "$IGN_PARTITION" || { echo "SIM_BACKEND=gazebo needs IGN_PARTITION" >&2; exit 2; }
  echo "export USE_SIM_TIME=1 IGN_PARTITION=$IGN_PARTITION IGN_IP=127.0.0.1 GZ_SPAWN=${GZ_SPAWN:-} RMF_DEMOS_HOME=${RMF_DEMOS_HOME:-} ODOM_ACTIVE_TOPIC=${ODOM_ACTIVE_TOPIC:-} GZ_DRIVE_TIMEOUT_S=${GZ_DRIVE_TIMEOUT_S:-0.2}" >> $RUN/env.sh
fi
source $RUN/env.sh
(cd $FC/server && python3 -m fleet_server init --data-dir $RUN/fleet --secrets-dir $RUN/secrets --robots iso_bench > $RUN/init.log)
python3 $FC/scripts/register_c103_map.py --data-dir $RUN/fleet > $RUN/register.log 2>&1 || {
  echo "C103 artifacts unavailable; seeding synthetic facility/floor/area only" >> $RUN/register.log
  python3 -c "import sys; sys.path.insert(0, '$FC/server'); from fleet_server.map_registry import MapRegistry; r=MapRegistry('$RUN/fleet/map_registry.db', '$RUN/fleet/map_store'); r.create_facility('paichai_univ', 'Test University'); r.create_building('paichai_c', 'paichai_univ', 'Hall C'); r.create_floor('paichai_c_1f', 'paichai_c', '1F'); r.create_area('paichai_c_1f_c103_lounge', 'paichai_c_1f', 'Test lounge')" >> $RUN/register.log 2>&1 || true
}
# NAV_LOCALIZATION=1 (Phase 3): unmodified agent_node with the saved-map Localization Worker (no map/loc fixture).
if [ "${NAV_LOCALIZATION:-0}" = "1" ]; then AGENT_ENTRY="-m limo_robot_agent.agent_node"; NAV_ARGS="-p navigation_localization:=true";
else AGENT_ENTRY="$HERE/agent_nav_fixture.py"; NAV_ARGS=""; fi
cat > $RUN/cmds.sh <<EOF
CMD_server="cd $FC/server && exec python3 -m fleet_server serve --data-dir $RUN/fleet --bind 127.0.0.1 --port $PORT --offline-after 3 --require-operating-mode ${SERVER_EXTRA:-}"
CMD_sim="$( [ $GZ = 1 ] && echo "exec ros2 launch $FC/tests/gazebo_web/gazebo_world.launch.py" || echo "exec python3 $HERE/kinematic_sim.py" )"
CMD_stack="exec ros2 launch $FC/tests/web_mapping_isolated/stack.launch.py"
CMD_watchdog="exec python3 -m limo_robot_agent.motion_watchdog"
CMD_agent="exec python3 $AGENT_ENTRY --ros-args -p server_url:=http://127.0.0.1:$PORT -p token_file:=$RUN/secrets/agent-iso_bench.token -p robot_id:=iso_bench -p mode:=mock -p simulated:=true -p allow_missions:=true -p state_dir:=$RUN/agent -p web_mapping_enabled:=true -p manual_web_bench:=true -p manual_locked_arbiter:=true -p require_motion_watchdog:=true -p period_s:=0.10 $NAV_ARGS $( [ $GZ = 1 ] && echo "-p use_sim_time:=true" ) ${AGENT_EXTRA:-} -p web_mapping_slam_params:=$REPO/capstone_ws/src/limo_capstone/config/integrated_slam_params.yaml"
EOF
# NO_START=1 (Phase 4): prepare the run only; a supervisor (limo_robot_agent.supervisor) starts the robot-side processes
if [ "${NO_START:-0}" = "1" ]; then echo "prepared $RUN (no process started)"; exit 0; fi
for p in server sim stack watchdog; do $HERE/stack_proc.sh $RUN start $p; done
sleep 6
$HERE/stack_proc.sh $RUN start agent
echo "started $RUN (port $PORT, domain $DOM)"
