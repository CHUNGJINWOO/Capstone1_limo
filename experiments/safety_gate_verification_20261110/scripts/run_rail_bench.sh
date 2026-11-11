#!/bin/bash
# usage: run_rail_bench.sh <launch_file_or_name> <tag>
# Spawns the robot 0.73 m in front of the chair-3 north-2 sled rail (the ack_run3 stall location), start_explore:=false,
# no velocity is ever published; reads the depth chain and costmaps with low_obstacle_probe.py.
HERE=$(cd "$(dirname "$0")" && pwd); EXP=$(dirname "$HERE")
L=$1; TAG=$2; OUT=$EXP/low_obstacle_analysis/$TAG; [ -e "$OUT" ] && { echo "refusing to overwrite $OUT"; exit 1; }; mkdir -p "$OUT"
source /opt/ros/humble/setup.bash; source ~/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-94}
cd ~/school/2026-2/Capstone1_limo/capstone_ws
if [ "${L:0:1}" = "/" ]; then F="$L"; else F="limo_capstone $L"; fi
setsid ros2 launch $F start_explore:=false spawn_x:=-1.2 spawn_y:=0.5 spawn_yaw:=3.14159 > "$OUT/launch.log" 2>&1 &
sleep 50
python3 -u "$HERE/low_obstacle_probe.py" --rail-x 0.73 --rail-y0 -0.22 --rail-y1 0.12 --wait 15 --out "$OUT/low_obstacle_probe.json" > "$OUT/low_obstacle_probe.txt" 2>&1
for kv in "/depth_obstacle_filter min_height" "/depth_obstacle_filter max_height" "/depth_obstacle_filter processing_hz" "/local_costmap/local_costmap obstacle_layer.depth_obstacles.min_obstacle_height" "/global_costmap/global_costmap obstacle_layer.depth_obstacles.min_obstacle_height"; do set -- $kv; echo "$1 $2: $(timeout 8 ros2 param get $1 $2 2>&1 | tr '\n' ' ')"; done > "$OUT/params.txt"
pkill -INT -f "[r]os2 launch"; sleep 8
pkill -9 -f "[i]gn gazebo"
pkill -9 -f "[l]imo_capstone/lib|[n]av2_|[s]lam_toolbox|[p]arameter_bridge|[r]obot_state_pub|[s]tatic_transform|[d]epth_obstacle|[c]ontroller_server|[p]lanner_server|[b]ehavior_server|[b]t_navigator|[v]elocity_smoother|[s]moother_server|[l]ifecycle_manager|[w]aypoint_follower"
sleep 2; echo "leftover=$(pgrep -f '[i]gn gazebo|[p]arameter_bridge|[s]tall_detector|[c]ontroller_server' | wc -l)"
