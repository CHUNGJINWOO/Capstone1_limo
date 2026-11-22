#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "$0")/../.." && pwd)
task_dir=$(mktemp -d /tmp/capstone-manual-offline-XXXXXXXX)
mkdir -p "$task_dir/fleet_control/server" "$task_dir/fleet_control/agent/limo_robot_agent"
cp -a "$repo/fleet_control/server/fleet_server" "$task_dir/fleet_control/server/"
cp -a "$repo/fleet_control/agent/limo_robot_agent/limo_robot_agent" "$task_dir/fleet_control/agent/limo_robot_agent/"
patch -d "$task_dir" -p1 --batch < "$repo/fleet_control/docs/manual_offline_fail_closed_candidate_20261120.patch"
set +u
source /opt/ros/humble/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export MANUAL_CANDIDATE_SERVER="$task_dir/fleet_control/server"
export MANUAL_CANDIDATE_AGENT="$task_dir/fleet_control/agent/limo_robot_agent"
printf 'Candidate copied to %s; no network or ROS node is started.\n' "$task_dir"
/usr/bin/python3 "$repo/fleet_control/tests/test_manual_offline_fail_closed_candidate_20261120.py"
