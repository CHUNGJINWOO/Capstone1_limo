#!/bin/bash
# usage: run_nav_check.sh <launch_file_or_name> <tag> [fault]    (fault = depth | none)
# Short Nav2 goal through the safety gate. Reports /cmd_vel publishers, gate state, goal result, GT path, and with
# fault=depth kills the depth filter mid-goal to confirm FAILSAFE latches and the robot stops (no reset is ever sent).
HERE=$(cd "$(dirname "$0")" && pwd); EXP=$(dirname "$HERE")
L=$1; TAG=$2; FAULT=${3:-none}
OUT=$EXP/sim_settings_20261111/nav/$TAG; [ -e "$OUT" ] && { echo "refusing to overwrite $OUT"; exit 1; }; mkdir -p "$OUT"
source /opt/ros/humble/setup.bash; source ~/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-98}
cd ~/school/2026-2/Capstone1_limo/capstone_ws
if [ "${L:0:1}" = "/" ]; then F="$L"; else F="limo_capstone $L"; fi
setsid ros2 launch $F start_explore:=false > "$OUT/launch.log" 2>&1 &
sleep 60
{
echo "gate: $(timeout 5 ros2 topic echo /stall_detector/status_json --once --field data | cut -c1-70)"
for t in /cmd_vel /cmd_vel_raw; do echo "$t PUB: $(ros2 topic info -v $t 2>/dev/null | awk '/Node name:/{n=$3} /Endpoint type: PUBLISHER/{print n}' | sort | uniq -c | tr '\n' ' ')"; done
echo "BT: $(ros2 param get /bt_navigator default_nav_to_pose_bt_xml 2>&1 | tail -1 | sed 's#.*/##')"
if [ "$FAULT" = "none" ]; then timeout 150 python3 -u "$HERE/nav_test.py" goal --dist 1.2 2>&1 | grep -v "^\[WARN\]"
else timeout 150 python3 -u "$HERE/nav_test.py" goal --dist 1.5 --fault "$FAULT" 2>&1 | grep -v "^\[WARN\]"; fi
echo "gate after: $(timeout 5 ros2 topic echo /stall_detector/status_json --once --field data | cut -c1-70)"
echo "gate log: $(grep -E 'FAIL-SAFE|STALL LATCHED|OPERATOR RESET' "$OUT/launch.log" | cut -c1-200 | tr '\n' ' ')"
} > "$OUT/result.txt" 2>&1
bash "$HERE/cleanup_sim.sh"
