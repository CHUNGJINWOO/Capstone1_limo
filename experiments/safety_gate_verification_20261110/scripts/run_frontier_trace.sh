#!/bin/bash
# usage: run_frontier_trace.sh <launch_file> <tag> [max_wall]
HERE=$(cd "$(dirname "$0")" && pwd); EXP=$(dirname "$HERE")
L=$1; TAG=$2; MAXW=${3:-420}
OUT=$EXP/frontier_analysis/$TAG; [ -e "$OUT" ] && { echo "refusing to overwrite $OUT"; exit 1; }; mkdir -p "$OUT"
source /opt/ros/humble/setup.bash; source ~/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-96}
cd ~/school/2026-2/Capstone1_limo/capstone_ws
setsid ros2 launch limo_capstone "$L" start_explore:=true > "$OUT/launch.log" 2>&1 &
sleep 20
python3 -u "$HERE/frontier_tracer.py" --out "$OUT/frontier_trace.jsonl" > "$OUT/tracer.log" 2>&1 &
TR=$!
t=0; while [ $t -lt $MAXW ]; do
  grep -qE "stopping\." "$OUT/launch.log" && break
  grep -qE "FAIL-SAFE|STALL LATCHED" "$OUT/launch.log" && { echo "gate event"; break; }
  sleep 2; t=$((t+2)); done
sleep 6; kill $TR 2>/dev/null
echo "waited ${t}s; explore msg: $(grep -oE 'No frontiers found, stopping|All frontiers traversed/tried out, stopping' $OUT/launch.log | head -1)"
pkill -INT -f "[r]os2 launch limo_capstone"; sleep 8
pkill -9 -f "[i]gn gazebo"
pkill -9 -f "[l]imo_capstone/lib|[n]av2_|[s]lam_toolbox|[p]arameter_bridge|[r]obot_state_pub|[s]tatic_transform|[d]epth_obstacle|[e]xplore|[c]ontroller_server|[p]lanner_server|[b]ehavior_server|[b]t_navigator|[v]elocity_smoother|[s]moother_server|[l]ifecycle_manager|[w]aypoint_follower"
sleep 2; echo "leftover=$(pgrep -f '[i]gn gazebo|[p]arameter_bridge|[s]tall_detector|[c]ontroller_server' | wc -l)"
