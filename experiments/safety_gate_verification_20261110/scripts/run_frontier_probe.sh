#!/bin/bash
# usage: run_frontier_probe.sh <launch_file> <tag> [wait_for_stop_sec]
# Runs the real launch with start_explore:=true, waits for explore_lite to stop ("stopping."), then runs the
# read-only frontier probe on the global costmap. Never publishes velocity / never resets the gate.
HERE=$(cd "$(dirname "$0")" && pwd); EXP=$(dirname "$HERE")
L=$1; TAG=$2; WAIT=${3:-420}
OUT=$EXP/frontier_analysis/$TAG; [ -e "$OUT" ] && { echo "refusing to overwrite $OUT"; exit 1; }; mkdir -p "$OUT"
source /opt/ros/humble/setup.bash; source ~/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-93}
cd ~/school/2026-2/Capstone1_limo/capstone_ws
setsid ros2 launch limo_capstone "$L" start_explore:=true > "$OUT/launch.log" 2>&1 &
t=0; while [ $t -lt $WAIT ]; do
  grep -qE "stopping\." "$OUT/launch.log" && break
  grep -qE "FAIL-SAFE|STALL LATCHED" "$OUT/launch.log" && { echo "gate event before explore stopped"; break; }
  sleep 3; t=$((t+3)); done
sleep 12
echo "waited ${t}s; explore msg: $(grep -oE 'No frontiers found, stopping|All frontiers traversed/tried out, stopping' $OUT/launch.log | head -1)"
python3 -u "$HERE/frontier_probe.py" --out "$OUT/frontier_probe.json" > "$OUT/frontier_probe.txt" 2>&1
pkill -INT -f "[r]os2 launch limo_capstone"; sleep 8
pkill -9 -f "[i]gn gazebo"
pkill -9 -f "[l]imo_capstone/lib|[n]av2_|[s]lam_toolbox|[p]arameter_bridge|[r]obot_state_pub|[s]tatic_transform|[d]epth_obstacle|[e]xplore|[c]ontroller_server|[p]lanner_server|[b]ehavior_server|[b]t_navigator|[v]elocity_smoother|[s]moother_server|[l]ifecycle_manager|[w]aypoint_follower"
sleep 2; echo "leftover=$(pgrep -f '[i]gn gazebo|[p]arameter_bridge|[s]tall_detector|[c]ontroller_server' | wc -l)"
