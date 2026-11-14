#!/bin/bash
# usage: run_final.sh <tag> [max_wall_sec]
# Runs the REAL repo differential-drive simulation launch (unchanged) with start_explore:=true, the passive monitor, an automatic map saver and a short rosbag.
# Never resets the safety gate, never publishes velocity, never bypasses the gate. Cleans up only the process group it started.
HERE=$(cd "$(dirname "$0")" && pwd); EXP=$(dirname "$HERE"); SG=$(dirname "$EXP")/safety_gate_verification_20261110/scripts
TAG=$1; MAXW=${2:-1200}; LAUNCH_FILE=${LAUNCH_FILE:-}; HOLD=${FAILSAFE_HOLD:-15}
OUT=$EXP/runs/$TAG; [ -e "$OUT" ] && { echo "refusing to overwrite $OUT"; exit 1; }; mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source "$(dirname "$EXP")/../capstone_ws/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-121} ROS_LOCALHOST_ONLY=1
echo "tag=$TAG domain=$ROS_DOMAIN_ID start=$(date -Is)" | tee "$OUT/run_info.txt"
cd "$(dirname "$EXP")/../capstone_ws"
setsid ros2 launch ${LAUNCH_FILE:-limo_capstone limo_integrated_auto_mapping_sim_diff.launch.py} start_explore:=true > "$OUT/launch.log" 2>&1 &
LP=$!
python3 -u "$HERE/auto_map_saver.py" --prefix "$OUT/limo_sim_map" --out-json "$OUT/saved.json" --cap-wall $((MAXW + 120)) > "$OUT/saver.log" 2>&1 &
SP=$!
sleep 20   # let the simulation, SLAM and Nav2 come up before recording
setsid ros2 bag record -o "$OUT/bag" /clock /tf /tf_static /scan /odom_nav2 /map /cmd_vel_raw /cmd_vel /explore/status /stall_detector/status_json /ground_truth/world_odom > "$OUT/bag.log" 2>&1 &
BP=$!
python3 -u "$HERE/auto_mapping_monitor_hold.py" --failsafe-hold $HOLD --out "$OUT" --launch-log "$OUT/launch.log" --max-wall $MAXW --tag "$TAG" > "$OUT/monitor.log" 2>&1
echo "monitor: $(tail -1 "$OUT/monitor.log")"
for i in $(seq 1 30); do [ -s "$OUT/saved.json" ] && break; sleep 1; done
[ -s "$OUT/saved.json" ] || { echo "saver did not finish by itself -> stopping it (no map saved by the saver)"; kill $SP 2>/dev/null; }
kill -INT -- -$BP 2>/dev/null; sleep 4
kill -INT -- -$LP 2>/dev/null; for i in $(seq 1 15); do kill -0 -$LP 2>/dev/null || break; sleep 1; done
kill -TERM -- -$LP 2>/dev/null; sleep 3; kill -KILL -- -$LP 2>/dev/null
echo "end=$(date -Is)" | tee -a "$OUT/run_info.txt"
echo "leftover gazebo/nav2 processes started by this run (domain $ROS_DOMAIN_ID): $(ps -eo pid,args | grep -E "[i]gn gazebo" | wc -l) ign gazebo total"
