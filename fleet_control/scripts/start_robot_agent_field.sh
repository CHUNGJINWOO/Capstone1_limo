#!/bin/bash
# Run ON the LIMO: (re)start ONLY the Robot Agent in the field's motor-locked configuration. Not the ROS stack, not the Gate, not 18190.
#   usage: start_robot_agent_field.sh SERVER_URL TOKEN_FILE [extra ros2 -p args...]   (both required: no server address or token path is stored here)
#          e.g.  start_robot_agent_field.sh http://127.0.0.1:18290 ~/.config/<dir>/agent-pcu.token -p map_save_enabled:=true
#   env STATE_DIR (default ~/.local/state/limo_robot_agent_field_20261120), WS (default ~/Capstone1_limo/robot_ws), LOG (default $STATE_DIR/agent.out)
# Always allow_missions:=false. Keeps the Agent's ROS settings: domain 77, localhost only, rmw_fastrtps_cpp. No tokens are printed.
#
# After starting, judge readiness from the server (laptop): scripts/dds_ready_check.py. A NOT_READY Agent is not ready, whatever the process says.
# Why the warm-up participant (TEMPORARY mitigation, NOT a solution): on the real robot a DDS participant that starts ALONE did not match slam_toolbox's latched /map (and the Agent
# came up with lidar/pose/map NO_DATA); the same start succeeded when another throw-away participant had been started ~3 s earlier
# (6/6 vs 0/3, 2026-11-20). Root cause not established; this keeps the Agent from coming up deaf. A deaf Agent stays fail-closed (NO_DATA).
# (no set -u: ROS setup.bash reads unset variables)
if [ $# -lt 2 ]; then echo "usage: $0 SERVER_URL TOKEN_FILE [extra ros2 -p args...]" >&2; exit 2; fi
case "$1" in http://*|https://*) ;; *) echo "ABORT: SERVER_URL must start with http:// or https://" >&2; exit 2;; esac
SERVER_URL=$1; TOKEN_FILE=$2; shift 2
STATE_DIR=${STATE_DIR:-$HOME/.local/state/limo_robot_agent_field_20261120}; WS=${WS:-$HOME/Capstone1_limo/robot_ws}; LOG=${LOG:-$STATE_DIR/agent.out}
source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if pgrep -f 'install/limo_robot_agent/lib/limo_robot_agent/robot_agent' >/dev/null; then echo "ABORT: a robot_agent is already running (stop it first)"; exit 2; fi
# Pre-start slot check (read-only canary; see dds_slot_precheck.py). Not a fix: it only tells whether the slot this node would get delivers data.
PRE="$(dirname "$0")/dds_slot_precheck.py"
if [ -f "$PRE" ]; then
  python3 "$PRE" --json > "$STATE_DIR/dds_precheck.json" 2>"$STATE_DIR/dds_precheck.err"; rc=$?
  echo "dds slot precheck: exit $rc (SLOT_OK=0 SLOT_DEAD=3 NO_HEALTHY_SLOT_FOUND=4) -> $STATE_DIR/dds_precheck.json"
  if [ $rc -eq 4 ] && [ "${FORCE_START:-0}" != 1 ]; then echo "ABORT: no free DDS slot receives data (stack down or all slots bad). FORCE_START=1 to start anyway (Agent will stay NO_DATA / fail-closed)."; exit 3; fi
fi
# TEMPORARY mitigation only (root cause unproven): occupy the lowest free slot with a throw-away participant so the Agent gets the next one.
python3 -c 'import rclpy,time
rclpy.init(); n=rclpy.create_node("fleet_agent_warmup"); t=time.time()
while time.time()-t<30: rclpy.spin_once(n,timeout_sec=0.5)' >/dev/null 2>&1 &
sleep 4
exec ros2 run limo_robot_agent robot_agent --ros-args -p server_url:="$SERVER_URL" -p token_file:="$TOKEN_FILE" -p robot_id:=pcu -p mode:=real \
  -p simulated:=false -p allow_missions:=false -p state_dir:="$STATE_DIR" "$@" >> "$LOG" 2>&1 < /dev/null
