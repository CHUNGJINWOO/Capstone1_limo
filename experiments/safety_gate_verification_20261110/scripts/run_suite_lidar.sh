#!/bin/bash
# usage: run_suite.sh <launch_file_name> <tag>
S=$(cd "$(dirname "$0")" && pwd); L=$1; TAG=$2; O=$S/suite_$TAG; mkdir -p $O
source /opt/ros/humble/setup.bash; source ~/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash
unset ROS_DOMAIN_ID
setsid python3 $S/exp2/state_watch.py > $O/state_watch.log 2>&1 &
cd ~/school/2026-2/Capstone1_limo/capstone_ws
setsid ros2 launch limo_capstone $L start_explore:=false > $O/launch.log 2>&1 &
sleep 60
echo "### startup"; grep -E "ARMED|FAIL-SAFE|STARTUP TIMEOUT" $O/launch.log | cut -c1-200; cat $O/state_watch.log
echo "### wiring"
for t in /cmd_vel /cmd_vel_raw /cmd_vel_nav /cmd_vel_smoothed; do
  echo "$t PUB: $(ros2 topic info -v $t 2>/dev/null | awk '/Node name:/{n=$3} /Endpoint type: PUBLISHER/{print n}' | sort | uniq -c | tr '\n' ' ')"
done
echo "### nav goal + lidar odom dropout"; timeout 120 python3 -u $S/nav_test.py goal --dist 1.2 --fault lidar 2>&1 | grep -v "^\[WARN\]"
echo "### gate log"; grep -E "stall_detector" $O/launch.log | grep -E "ARMED|FAIL-SAFE|RESET|STALL" | cut -c1-220
# cleanup
pkill -INT -f "[r]os2 launch limo_capstone"; sleep 8
pkill -9 -f "[i]gn gazebo"; pkill -9 -f "[l]imo_capstone/lib|[n]av2_|[s]lam_toolbox|[p]arameter_bridge|[r]obot_state_pub|[s]tatic_transform|[d]epth_obstacle|[s]tate_watch|[e]xplore"
sleep 2; echo "### leftover: $(pgrep -f '[i]gn gazebo|[p]arameter_bridge|[s]tall_detector' | wc -l)"
