#!/bin/bash
# Isolated, motor-locked web-mapping stack (no hardware, no /cmd_vel). usage: run.sh NEW_RUN_DIR PORT ROS_DOMAIN_ID
# Prereq: an overlay that provides explore_lite with fleet_frontier_candidates, e.g.
#   colcon --log-base <tmp>/log build --base-paths capstone_ws/src/m-explore-ros2 --packages-select explore_lite --build-base <tmp>/build --install-base <tmp>/install
# then: OVERLAY=<tmp>/install run.sh ...
set -e
RUN=$1; PORT=$2; DOM=$3; HERE=$(cd "$(dirname "$0")" && pwd); FC=$(cd "$HERE/../.." && pwd); REPO=$(cd "$FC/.." && pwd)
test -n "$OVERLAY" -a -f "$OVERLAY/setup.bash" || { echo "set OVERLAY to the explore_lite overlay install dir" >&2; exit 2; }
mkdir "$RUN"; : > $RUN/pgids
source /opt/ros/humble/setup.bash; source $REPO/capstone_ws/install/setup.bash; source $OVERLAY/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=$DOM ROS2CLI_NO_DAEMON=1 ROS_LOG_DIR=$RUN/roslog; unset ROS_DISCOVERY_SERVER
(cd $FC/server && PYTHONPATH=$FC/server python3 -m fleet_server init --data-dir $RUN/fleet --secrets-dir $RUN/secrets --robots iso_bench > $RUN/init.log)
python3 $FC/scripts/register_c103_map.py --data-dir $RUN/fleet > $RUN/register.log || echo "C103 artifacts unavailable; registry left empty"
start() { setsid bash -c "$1" > $RUN/$2.log 2>&1 & echo $! >> $RUN/pgids; echo "$2 $!" >> $RUN/pgnames; }
start "cd $FC/server && PYTHONPATH=$FC/server exec python3 -m fleet_server serve --data-dir $RUN/fleet --bind 127.0.0.1 --port $PORT" server
start "exec python3 $HERE/fixture.py" fixture
start "exec ros2 launch $HERE/stack.launch.py" stack
start "exec python3 $HERE/vel_monitor.py $RUN/vel.jsonl" vel
REQ_WD=false; if [ "${WATCHDOG:-1}" = "1" ]; then REQ_WD=true; start "PYTHONPATH=$FC/agent/limo_robot_agent:\$PYTHONPATH exec python3 -m limo_robot_agent.motion_watchdog" watchdog; fi
sleep 6
start "PYTHONPATH=$FC/agent/limo_robot_agent:\$PYTHONPATH exec python3 -m limo_robot_agent.agent_node --ros-args -p server_url:=http://127.0.0.1:$PORT -p token_file:=$RUN/secrets/agent-iso_bench.token -p robot_id:=iso_bench -p mode:=mock -p simulated:=true -p allow_missions:=false -p state_dir:=$RUN/agent -p web_mapping_enabled:=true -p require_motion_watchdog:=$REQ_WD -p web_mapping_slam_params:=$REPO/capstone_ws/src/limo_capstone/config/integrated_slam_params.yaml" agent
echo "started; stop with $HERE/stop.sh $RUN"
