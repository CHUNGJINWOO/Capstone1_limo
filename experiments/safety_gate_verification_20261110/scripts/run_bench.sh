#!/bin/bash
# usage: run_bench.sh <launch_file_or_name> <tag> <spawn_x> <spawn_y> <spawn_yaw> <probe args...>
# Static bench (start_explore:=false, no velocity ever published): spawn the robot, wait, read the depth chain and costmaps.
# Output: sim_settings_20261111/bench/<tag>/ (never overwrites).
HERE=$(cd "$(dirname "$0")" && pwd); EXP=$(dirname "$HERE")
L=$1; TAG=$2; SX=$3; SY=$4; SYAW=$5; shift 5
OUT=$EXP/sim_settings_20261111/bench/$TAG; [ -e "$OUT" ] && { echo "refusing to overwrite $OUT"; exit 1; }; mkdir -p "$OUT"
source /opt/ros/humble/setup.bash; source ~/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-97}
cd ~/school/2026-2/Capstone1_limo/capstone_ws
if [ "${L:0:1}" = "/" ]; then F="$L"; else F="limo_capstone $L"; fi
setsid ros2 launch $F start_explore:=false spawn_x:=$SX spawn_y:=$SY spawn_yaw:=$SYAW > "$OUT/launch.log" 2>&1 &
sleep 50
python3 -u "$HERE/low_obstacle_probe.py" --wait 15 --out "$OUT/probe.json" "$@" > "$OUT/probe.txt" 2>&1
for kv in "/depth_obstacle_filter processing_hz" "/depth_obstacle_filter min_height" "/depth_obstacle_filter data_timeout_sec" "/local_costmap/local_costmap obstacle_layer.depth_obstacles.min_obstacle_height" "/global_costmap/global_costmap obstacle_layer.depth_obstacles.min_obstacle_height" "/local_costmap/local_costmap obstacle_layer.virtual_bumper.min_obstacle_height" "/global_costmap/global_costmap obstacle_layer.virtual_bumper.min_obstacle_height" "/bt_navigator default_nav_to_pose_bt_xml" "/stall_detector_node sensor_timeout_sec"; do set -- $kv; echo "$1 $2: $(timeout 8 ros2 param get $1 $2 2>&1 | tr '\n' ' ' | sed 's#/home/.*/##')"; done > "$OUT/params.txt"
echo "/cmd_vel publishers: $(ros2 topic info -v /cmd_vel 2>/dev/null | awk '/Node name:/{n=$3} /Endpoint type: PUBLISHER/{print n}' | sort | uniq -c | tr '\n' ' ')" >> "$OUT/params.txt"
bash "$HERE/cleanup_sim.sh"
