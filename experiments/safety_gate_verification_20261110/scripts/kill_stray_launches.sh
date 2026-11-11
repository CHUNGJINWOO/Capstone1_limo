#!/bin/bash
# Kill leftover `ros2 launch` parents, explore_relaxed nodes and helpers from earlier runs (the sim cleanup used to miss
# them), then remove stale FastDDS shared-memory files of this user when no ROS process is left. The ros2 CLI daemon is kept.
pkill -9 -f "[r]os2 launch"
pkill -9 -f "[l]ib/explore_relaxed/explore_relaxed"
pkill -9 -f "[j]oint_state_publisher"
pkill -9 -f "[r]elaxed_listener.py"
pkill -9 -f "[f]rontier_tracer.py"
pkill -9 -f "[a]uto_mapping_monitor.py"
pkill -9 -f "[i]gn gazebo"
pkill -9 -f "[l]imo_capstone/lib|[n]av2_|[s]lam_toolbox|[p]arameter_bridge|[r]obot_state_pub|[s]tatic_transform|[d]epth_obstacle|[c]ontroller_server|[p]lanner_server|[b]ehavior_server|[b]t_navigator|[v]elocity_smoother|[s]moother_server|[l]ifecycle_manager|[w]aypoint_follower|[e]xplore_node"
sleep 2
left=$(pgrep -f '[r]os2 launch|[l]aunch_ros|/opt/ros/humble/lib/|[l]ib/explore_relaxed/explore_relaxed' | wc -l)
echo "remaining ROS launch-related processes: $left"
if [ "$left" -eq 0 ]; then
  before=$(find /dev/shm -maxdepth 1 -user "$USER" -name 'fastrtps_*' | wc -l)
  find /dev/shm -maxdepth 1 -user "$USER" -name 'fastrtps_*' -delete
  echo "removed stale fastrtps shm files: $before"
fi
