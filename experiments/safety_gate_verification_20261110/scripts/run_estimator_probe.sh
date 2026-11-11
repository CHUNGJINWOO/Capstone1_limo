#!/bin/bash
# usage: run_estimator_probe.sh EST SCEN SPAWN_X SPAWN_Y SPAWN_YAW  (sim only; isolated ROS domain; no explorer)
HERE=$(cd "$(dirname "$0")" && pwd); EXP=$(dirname "$HERE"); EST=$1; SC=$2
OUT=$EXP/estimator_study_20261111/runs/${EST}__${SC}; [ -e "$OUT" ] && { echo "refusing to overwrite $OUT"; exit 1; }; mkdir -p "$OUT"
source /opt/ros/humble/setup.bash; source ~/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash
bash "$HERE/kill_stray_launches.sh" >/dev/null
export ROS_DOMAIN_ID=$((100 + $(date +%s) % 120))
cd ~/school/2026-2/Capstone1_limo/capstone_ws
setsid ros2 launch $EXP/variants/sim_diff_estimator_compare.launch.py estimator_type:=$EST start_explore:=false spawn_x:=$3 spawn_y:=$4 spawn_yaw:=$5 > "$OUT/launch.log" 2>&1 &
sleep 45
python3 -u "$HERE/estimator_probe.py" "$OUT/probe" 0.2 1.8 16 > "$OUT/probe.out" 2>&1
tail -2 "$OUT/probe.out"
bash "$HERE/cleanup_sim.sh" >/dev/null
