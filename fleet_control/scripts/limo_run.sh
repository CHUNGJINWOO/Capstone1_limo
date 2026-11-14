#!/bin/bash
# Runs ON THE LIMO (ssh <ROBOT_USER>@<ROBOT_HOST> 'VAR=.. bash -s' < limo_run.sh). Overlay workspace only; the team workspace is read-only.
# env: BUILD=1 builds limo_robot_agent; STACK=1 starts the existing real launch with its DEFAULT motor lock (allow_motor_drive stays false, no exploration);
#      DUR=seconds the agent runs; SERVER=http://<TAILSCALE_IP>:8080 (REQUIRED, no default) ; ALLOW_MISSIONS=false (default)
#      TEAM_WS (default $HOME/capstone_ws, only sourced) ; OVERLAY_WS (default $HOME/capstone_experiment_ws, where limo_robot_agent is built) ; LOG_DIR (default $HOME/limo_fleet_logs)
# Never starts anything with allow_motor_drive:=true and never publishes a velocity.
set -o pipefail
DUR=${DUR:-60}; ALLOW=${ALLOW_MISSIONS:-false}
if [ -z "${SERVER:-}" ]; then echo "ABORT: set SERVER=http://<TAILSCALE_IP>:8080 (address of the central server)"; exit 2; fi
if ps -eo args | grep -E "[r]os2 launch|[l]imo_base|[s]tall_detector|[y]dlidar|[n]av2_|[s]lam_toolbox|[r]obot_agent" | grep -v "bash -s" | grep -q .; then echo "ABORT: ROS/motor/agent process already running"; exit 2; fi
set +u; source /opt/ros/humble/setup.bash; TEAM_WS=${TEAM_WS:-$HOME/capstone_ws}; OV=${OVERLAY_WS:-$HOME/capstone_experiment_ws}
source "$TEAM_WS/install/setup.bash"; source "$OV/install/setup.bash"
if [ "${BUILD:-0}" = 1 ]; then
  (cd $OV && colcon build --packages-select limo_robot_agent 2>&1 | grep -E "Starting|Finished|Summary|rror|Failed"); source $OV/install/setup.bash
fi
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
echo "== server reachable from the LIMO? $(curl -sS -m 4 $SERVER/api/v1/health 2>&1 | head -c 80)"
LOGD=${LOG_DIR:-$HOME/limo_fleet_logs}; mkdir -p "$LOGD"
PIDS=()
if [ "${STACK:-0}" = 1 ]; then
  setsid ros2 launch limo_capstone limo_integrated_auto_mapping_real.launch.py scan_normalizer:=true > $LOGD/real_launch.log 2>&1 & PIDS+=($!); echo "real launch started (motor lock default); waiting for it to come up"; sleep 40
  echo "== /cmd_vel publishers (expect none): $(ros2 topic info /cmd_vel 2>&1 | grep -E 'Publisher count')"
fi
setsid ros2 run limo_robot_agent robot_agent --ros-args -p server_url:=$SERVER -p token_file:=$HOME/.config/limo_fleet/agent-pcu.token -p robot_id:=pcu -p mode:=real -p simulated:=false -p allow_missions:=$ALLOW > $LOGD/agent.log 2>&1 & PIDS+=($!)
echo "agent started; running for $DUR s"
sleep "$DUR"
echo "== agent log tail:"; tail -5 $LOGD/agent.log | cut -c1-160
echo "== /cmd_vel publishers at the end: $(ros2 topic info /cmd_vel 2>&1 | grep -E 'Publisher count')"
for ((i=${#PIDS[@]}-1; i>=0; i--)); do kill -INT -- -${PIDS[$i]} 2>/dev/null; sleep 6; kill -TERM -- -${PIDS[$i]} 2>/dev/null; sleep 2; kill -KILL -- -${PIDS[$i]} 2>/dev/null; done
echo "== leftover processes (expect none):"; ps -eo args | grep -E "[r]os2 launch|[l]imo_base|[s]tall_detector|[n]av2_|[s]lam_toolbox|[r]obot_agent" | grep -v "bash -s" | cut -c1-80 || true; echo "(end)"
