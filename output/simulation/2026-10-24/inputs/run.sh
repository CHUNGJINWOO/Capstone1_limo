#!/bin/bash
# Legacy room demo. Provide the separate Fortress asset workspace explicitly.
set -e
limo_demo_inputs=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
limo_project_root=$(cd -- "$limo_demo_inputs/../../../.." && pwd)
: "${LIMO_SIM_RESEARCH_WS:?Set LIMO_SIM_RESEARCH_WS to the external limo_sim_ws directory}"
source /opt/ros/humble/setup.bash
source "$LIMO_SIM_RESEARCH_WS/install/setup.bash"
source "$limo_project_root/capstone_ws/install/setup.bash"
export ROS_DOMAIN_ID=87
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=limo_video_obstacle_contact_20261024
export IGN_IP=127.0.0.1
export FASTRTPS_DEFAULT_PROFILES_FILE="$LIMO_SIM_RESEARCH_WS/experiments/fortress/fastdds_udp_only.xml"
export ROS_LOG_DIR=/tmp/limo-sim-demo/ros-logs
export PYTHONNOUSERSITE=1
exec /usr/bin/python3 "$limo_demo_inputs/run_demo.py" "$@"
