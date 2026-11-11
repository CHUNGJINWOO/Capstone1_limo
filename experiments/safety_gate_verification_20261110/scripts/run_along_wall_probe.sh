#!/bin/bash
# usage: run_along_wall_probe.sh TAG SPAWN_X SPAWN_Y SPAWN_YAW   (sim only; isolated ROS domain; start_explore=false)
HERE=$(cd "$(dirname "$0")" && pwd); EXP=$(dirname "$HERE"); TAG=$1
OUT=$EXP/post_analysis_20261111/slam_error/repro_$TAG; [ -e "$OUT" ] && { echo "refusing to overwrite $OUT"; exit 1; }; mkdir -p "$OUT"
source /opt/ros/humble/setup.bash; source ~/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash
bash "$HERE/kill_stray_launches.sh" >/dev/null
export ROS_DOMAIN_ID=$((100 + $(date +%s) % 120))
cd ~/school/2026-2/Capstone1_limo/capstone_ws
setsid ros2 launch limo_capstone limo_integrated_auto_mapping_sim_diff.launch.py start_explore:=false spawn_x:=$2 spawn_y:=$3 spawn_yaw:=$4 > "$OUT/launch.log" 2>&1 &
sleep 45
python3 -u "$HERE/along_wall_probe.py" "$OUT/probe.json" 0.2 1.8 16 > "$OUT/probe.out" 2>&1
cat "$OUT/probe.out"
bash "$HERE/cleanup_sim.sh"
