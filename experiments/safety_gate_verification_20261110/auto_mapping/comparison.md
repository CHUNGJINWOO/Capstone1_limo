| run | stop | sim_s | rtf_overall | last_motion_sim_s | gt_path_m | known_m2 | interior_known_frac | wall_cov | wall_abs_offset_m | slam_err_mean_m | goals_sent | ctrl_no_progress | planner_fail | recoveries | gate_failsafe | gate_stall | stall_pending_events | cpu_sys_mean |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| diff_run1 | exploration_stopped | 224.0 | 0.862 | 95.0 | 11.54 | 96.4 | 0.91 | 0.883 | 0.022 | 0.09 | 37 | 3 | 6 | 2 | 0 | 0 | 6 | 38.67 |
| diff_run2 | gate_failsafe_held | 136.0 | 0.87 | 125.0 | 12.07 | 96.7 | 0.91 | 0.882 | 0.032 | 0.111 | 33 | 8 | 15 | 6 | 1 | 0 | 4 | 44.41 |
| diff_run3 | exploration_stopped | 225.0 | 0.86 | 100.0 | 13.25 | 97.7 | 0.922 | 0.905 | 0.051 | 0.099 | 38 | 3 | 11 | 3 | 0 | 0 | 5 | 38.79 |
| ack_run1 | robot_stuck | 254.0 | 0.827 | 15.0 | 2.41 | 60.7 | 0.577 | 0.491 | 0.031 | 0.055 | 17 | 0 | 166 | 35 | 0 | 0 | 1 | 38.61 |
| ack_run2_probe | gate_failsafe_held | 60.0 | 0.865 | 50.0 | 10.6 | 68.0 | 0.648 | 0.385 | 0.044 | 0.069 | 14 | 2 | 2 | 0 | 1 | 0 | 2 | 40.09 |
| ack_run3_probe | gate_failsafe_held | 26.0 | 0.82 | 10.0 | 1.5 | 49.7 | 0.47 | 0.488 | 0.057 | 0.024 | 7 | 2 | 4 | 1 | 0 | 1 | 6 | 43.07 |
| ack_run4_probe | exploration_stopped | 217.0 | 0.854 | 95.0 | 22.88 | 104.1 | 0.985 | 0.997 | 0.046 | 0.065 | 25 | 0 | 4 | 0 | 0 | 0 | 5 | 40.28 |

- diff_run1: failsafe_cause=[] stall_cause=[] min_scan=0.322 gaps>0.38s={'/odometry/lidar_odom': 1, '/odom': 1, '/odom_nav2': 1, '/depth_obstacles/points': 1} cpu(gazebo/nav2/slam/gate)=155.91/79.99/14.05/67.16
- diff_run2: failsafe_cause=['scan_stale=False, lidar_stale=False, wheel_odom_stale=False, est_odom_stale=False, depth_fault=True (depth_mode=required, code=DEPTH_STALE)'] stall_cause=[] min_scan=0.356 gaps>0.38s={} cpu(gazebo/nav2/slam/gate)=144.11/82.34/14.38/63.08
- diff_run3: failsafe_cause=[] stall_cause=[] min_scan=0.334 gaps>0.38s={} cpu(gazebo/nav2/slam/gate)=157.96/81.57/14.65/67.59
- ack_run1: failsafe_cause=[] stall_cause=[] min_scan=0.1 gaps>0.38s={} cpu(gazebo/nav2/slam/gate)=158.11/80.61/13.35/69.23
- ack_run2_probe: failsafe_cause=['scan_stale=False, lidar_stale=False, wheel_odom_stale=False, est_odom_stale=False, depth_fault=True (depth_mode=required, code=DEPTH_STALE)'] stall_cause=[] min_scan=0.1 gaps>0.38s={} cpu(gazebo/nav2/slam/gate)=145.6/76.26/14.1/61.88
- ack_run3_probe: failsafe_cause=[] stall_cause=['Physical stall confirmed after 1'] min_scan=0.101 gaps>0.38s={} cpu(gazebo/nav2/slam/gate)=133.3/70.68/11.63/57.57
- ack_run4_probe: failsafe_cause=[] stall_cause=[] min_scan=0.152 gaps>0.38s={} cpu(gazebo/nav2/slam/gate)=156.42/81.12/15.09/66.32
