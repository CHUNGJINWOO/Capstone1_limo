#!/bin/bash
# Stop any simulation started from this repo (launch, gz, nav2, slam, helper nodes). Safe to run repeatedly.
pkill -INT -f "[r]os2 launch limo_capstone"; pkill -INT -f "[r]os2 launch /home"; sleep 6
pkill -9 -f "[r]os2 launch"
pkill -9 -f "[i]gn gazebo"
pkill -9 -f "[l]imo_capstone/lib|[n]av2_|[s]lam_toolbox|[p]arameter_bridge|[r]obot_state_pub|[s]tatic_transform|[d]epth_obstacle|[c]ontroller_server|[p]lanner_server|[b]ehavior_server|[b]t_navigator|[v]elocity_smoother|[s]moother_server|[l]ifecycle_manager|[w]aypoint_follower|[e]xplore_node|[l]ib/explore_relaxed/explore_relaxed|[f]rontier_tracer|[r]elaxed_listener"
sleep 2
echo "leftover=$(pgrep -f '[i]gn gazebo|[p]arameter_bridge|[s]tall_detector|[c]ontroller_server' | wc -l)"
