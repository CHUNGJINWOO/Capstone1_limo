#!/bin/bash
# Simulated robot "mock1" on the laptop: mock navigation stack + agent (isolated ROS domain, localhost only). Uses fleet_control/maps/limo_sim_map.yaml (a saved Gazebo auto-mapping result) as the map; override with FLEET_MOCK_MAP_YAML.
HERE=$(cd "$(dirname "$0")/.." && pwd); REPO=$(dirname "$HERE")
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-140} ROS_LOCALHOST_ONLY=1 PYTHONPATH=$HERE/agent/limo_robot_agent
set +u; source /opt/ros/humble/setup.bash; [ -f "$REPO/capstone_ws/install/setup.bash" ] && source "$REPO/capstone_ws/install/setup.bash"
python3 -m limo_robot_agent.mock_robot_node --ros-args -p map_yaml:="${FLEET_MOCK_MAP_YAML:-$HERE/maps/limo_sim_map.yaml}" -p speed_mps:=${MOCK_SPEED:-0.5} &
sleep 2
exec python3 -m limo_robot_agent.agent_node --ros-args -p server_url:=${SERVER_URL:-http://127.0.0.1:8080} -p token_file:="${TOKEN_FILE:-$HERE/secrets/agent-mock1.token}" -p robot_id:=mock1 -p mode:=mock -p simulated:=true -p allow_missions:=true -p state_dir:="${STATE_DIR:-$HERE/data/agent_state}"
