#!/bin/bash
# usage: run_pocket_bench.sh <launch_file_or_name> <tag>
# Spawns the robot at the ack_run1 stuck pose (world -0.26,1.36, yaw 1.71) with start_explore:=false and asks the global planner
# (read-only ComputePathToPose) for paths to the world goals that all failed in ack_run1, plus the costmap probe. No velocity.
HERE=$(cd "$(dirname "$0")" && pwd); EXP=$(dirname "$HERE")
L=$1; TAG=$2; OUT=$EXP/stuck_analysis/$TAG; [ -e "$OUT" ] && { echo "refusing to overwrite $OUT"; exit 1; }; mkdir -p "$OUT"
source /opt/ros/humble/setup.bash; source ~/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-95}
cd ~/school/2026-2/Capstone1_limo/capstone_ws
if [ "${L:0:1}" = "/" ]; then F="$L"; else F="limo_capstone $L"; fi
setsid ros2 launch $F start_explore:=false spawn_x:=-0.26 spawn_y:=1.36 spawn_yaw:=1.71 > "$OUT/launch.log" 2>&1 &
sleep 60
python3 -u "$HERE/costmap_probe.py" --still-sec 3 --timeout 40 > "$OUT/costmap_probe.txt" 2>&1
timeout 120 python3 -u "$HERE/plan_probe.py" --spawn -0.26 1.36 1.71 --world-goals="-1.76,-1.8;2.88,1.73;-4.95,0.44;1.29,3.6;-3.66,3.91;4.62,1.96;-1.0,0.0;-0.3,0.4" > "$OUT/plan_probe.txt" 2>&1
pkill -INT -f "[r]os2 launch"; sleep 8
pkill -9 -f "[i]gn gazebo"
pkill -9 -f "[l]imo_capstone/lib|[n]av2_|[s]lam_toolbox|[p]arameter_bridge|[r]obot_state_pub|[s]tatic_transform|[d]epth_obstacle|[c]ontroller_server|[p]lanner_server|[b]ehavior_server|[b]t_navigator|[v]elocity_smoother|[s]moother_server|[l]ifecycle_manager|[w]aypoint_follower"
sleep 2; echo "leftover=$(pgrep -f '[i]gn gazebo|[p]arameter_bridge|[s]tall_detector|[c]ontroller_server' | wc -l)"
