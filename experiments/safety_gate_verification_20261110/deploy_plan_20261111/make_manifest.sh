#!/bin/bash
# Read-only on the repo: prints tier / git-state / sha256 / path for the planned deploy set (relative to capstone_ws/src).
cd "$(git rev-parse --show-toplevel)" || exit 1
SRC=capstone_ws/src
state() { if git ls-files --error-unmatch "$1" >/dev/null 2>&1; then if git diff --quiet -- "$1"; then echo tracked; else echo MODIFIED-uncommitted; fi; else echo UNTRACKED; fi; }
emit() { tier=$1; shift; for f in "$@"; do [ -f "$f" ] && printf '%s\t%s\t%s\t%s\n' "$tier" "$(state "$f")" "$(sha256sum "$f" | cut -c1-16)" "${f#$SRC/}"; done; }
P=$SRC/limo_capstone
emit T1-limo_capstone $P/package.xml $P/setup.py $P/setup.cfg $P/resource/limo_capstone $P/limo_capstone/__init__.py \
  $P/limo_capstone/stall_detector_node.py $P/limo_capstone/lidar_scan_matcher_odom_node.py $P/limo_capstone/odom_tf_relay_node.py $P/limo_capstone/complementary_filter_node.py \
  $P/launch/limo_integrated_auto_mapping_real.launch.py \
  $P/config/integrated_slam_params.yaml $P/config/integrated_nav2_params.yaml $P/config/integrated_nav2_lidar_only_params.yaml $P/config/integrated_explore_params.yaml \
  $P/config/fusion_f5_improved.yaml $P/config/ekf_tuned.yaml $P/config/navigate_to_pose_ackermann_no_reverse.xml
emit T2-optional $P/rviz/limo_real_auto_mapping.rviz $P/scripts/auto_mapping_experiment_manager.py $P/scripts/manage_auto_mapping.sh $P/scripts/audit_estimator_dryrun.py $P/scripts/run_unpowered_trial.py $P/scripts/run_recorder.py
emit T3a-limo_depth_obstacles $(git ls-files $SRC/limo_depth_obstacles | grep -v pycache)
emit T3b-explore_lite $(git ls-files $SRC/m-explore-ros2/explore $SRC/m-explore-ros2/explore_lite_msgs)
emit T3c-ydlidar_ros2_driver $(git ls-files $SRC/ydlidar_ros2_driver)
emit T3d-robot_localization $(git ls-files $SRC/robot_localization | grep -v "\.bag$")
