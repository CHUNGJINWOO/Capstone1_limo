#!/bin/bash
# RMF stack on the laptop: traffic schedule + task dispatcher + LIMO fleet adapter. Talks ONLY to the Mission Manager URL given in SERVER_URL.
# Usage: SERVER_URL=http://127.0.0.1:18090 TOKEN_FILE=/path/rmf-adapter.token bash scripts/run_rmf_stack.sh
# Never point this at the production server (8080) before the missions-allowed policy of the real LIMO agent has been reviewed (see docs/RMF.md).
set -u; HERE=$(cd "$(dirname "$0")/.." && pwd)
: "${SERVER_URL:?set SERVER_URL}"; : "${TOKEN_FILE:?set TOKEN_FILE}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-151} ROS_LOCALHOST_ONLY=1 PYTHONPATH=$HERE
set +u; source /opt/ros/humble/setup.bash; source "${RMF_ISOLATED_ENV:-$HOME/rmf_isolated/env.sh}"; set -u
W=$(mktemp -d); cd "$W"; trap 'kill 0' EXIT
ros2 run rmf_traffic_ros2 rmf_traffic_schedule & sleep 2
ros2 run rmf_task_ros2 rmf_task_dispatcher --ros-args -p use_unique_hex_string_with_task_id:=false & sleep 1
python3 -u -m limo_rmf_adapter.adapter_node --config "$HERE/config/fleet.json" --server-url "$SERVER_URL" --token-file "$TOKEN_FILE" --state-dir "$W/adapter_state"
