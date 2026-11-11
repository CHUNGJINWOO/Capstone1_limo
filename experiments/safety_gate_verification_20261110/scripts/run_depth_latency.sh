#!/bin/bash
# usage: run_depth_latency.sh <launch_file_or_name> <tag> <seconds> [extra launch args...]
# Starts the launch (default start_explore:=true) and the passive latency probe for <seconds> wall seconds.
# The probe never publishes and the gate is never reset; a FAILSAFE just stays latched while measuring continues.
HERE=$(cd "$(dirname "$0")" && pwd); EXP=$(dirname "$HERE")
L=$1; TAG=$2; SECS=$3; shift 3
OUT=$EXP/depth_analysis/$TAG
[ -e "$OUT" ] && { echo "refusing to overwrite $OUT"; exit 1; }
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash; source ~/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-92}
cd ~/school/2026-2/Capstone1_limo/capstone_ws
case "$L" in /*) LAUNCH="$L";; *) LAUNCH="limo_capstone $L";; esac
if [ "${L:0:1}" = "/" ]; then
  setsid ros2 launch "$L" start_explore:=${EXPLORE:-true} "$@" > "$OUT/launch.log" 2>&1 &
else
  setsid ros2 launch limo_capstone "$L" start_explore:=${EXPLORE:-true} "$@" > "$OUT/launch.log" 2>&1 &
fi
sleep 25     # let the stack come up before measuring
python3 -u "$HERE/depth_latency_probe.py" --out "$OUT" --seconds "$SECS" > "$OUT/probe.log" 2>&1
echo "probe: $(tail -1 $OUT/probe.log)"
pkill -INT -f "[r]os2 launch"; sleep 8
pkill -9 -f "[i]gn gazebo"
pkill -9 -f "[l]imo_capstone/lib|[n]av2_|[s]lam_toolbox|[p]arameter_bridge|[r]obot_state_pub|[s]tatic_transform|[d]epth_obstacle|[e]xplore|[c]ontroller_server|[p]lanner_server|[b]ehavior_server|[b]t_navigator|[v]elocity_smoother|[s]moother_server|[l]ifecycle_manager|[w]aypoint_follower"
sleep 2
echo "leftover=$(pgrep -f '[i]gn gazebo|[p]arameter_bridge|[s]tall_detector|[c]ontroller_server' | wc -l)"
