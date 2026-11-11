#!/bin/bash
# usage: run_auto_mapping_eval.sh <launch_file> <tag> [max_wall_sec]
# Runs the REAL repo launch with start_explore:=true plus the passive monitor, then cleans up.
# Never resets the safety gate and never publishes velocity.
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(dirname "$HERE")
L=$1; TAG=$2; MAXW=${3:-1200}
OUT=$EXP/auto_mapping/$TAG
[ -e "$OUT" ] && { echo "refusing to overwrite $OUT"; exit 1; }
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source ~/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-90}
cd ~/school/2026-2/Capstone1_limo/capstone_ws
setsid ros2 launch limo_capstone "$L" start_explore:=true > "$OUT/launch.log" 2>&1 &
sleep 1
[ -n "$PROBE" ] && { python3 -u "$HERE/costmap_probe.py" --still-sec 25 --timeout "$MAXW" > "$OUT/costmap_probe.txt" 2>&1 & }
python3 -u "$HERE/auto_mapping_monitor.py" --out "$OUT" --launch-log "$OUT/launch.log" --max-wall "$MAXW" --tag "$TAG" > "$OUT/monitor.log" 2>&1
echo "monitor exited: $(tail -1 "$OUT/monitor.log")"
pkill -INT -f "[r]os2 launch limo_capstone"; sleep 8
pkill -9 -f "[i]gn gazebo"
pkill -9 -f "[l]imo_capstone/lib|[n]av2_|[s]lam_toolbox|[p]arameter_bridge|[r]obot_state_pub|[s]tatic_transform|[d]epth_obstacle|[e]xplore|[c]ontroller_server|[p]lanner_server|[b]ehavior_server|[b]t_navigator|[v]elocity_smoother|[s]moother_server|[l]ifecycle_manager|[w]aypoint_follower"
sleep 2
echo "leftover=$(pgrep -f '[i]gn gazebo|[p]arameter_bridge|[s]tall_detector|[c]ontroller_server' | wc -l)"
