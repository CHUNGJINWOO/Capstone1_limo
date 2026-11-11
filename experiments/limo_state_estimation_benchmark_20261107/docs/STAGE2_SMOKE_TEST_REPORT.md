# Stage 2: Foundation Pipeline Implementation & 1-Run Smoke Test Report

- **Execution Timestamp:** `2026-11-07 20:24:16 KST`
- **Experiment Directory:** `/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107`
- **Original Source Integrity:** 100% preserved (`capstone_ws` and `experiments/lounge_failure_investigation_20261107` untouched).

---

## 1. Fixes & Architectural Protections Implemented

1. **IMU ROS-GZ Bridge Fix (`launch/benchmark_sim.launch.py`):**
   - Fixed the message type typo in the previous lounge launch file (`/imu@sensor_msgs/msg/IMU[ignition.msgs.IMU` $\to$ `/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU`).
   - Verified live reception at **95.47 Hz** (`2,387` messages in 25s, `frame_id: imu_link`, sim-time offset `0.008 s`).
2. **Depth Camera TF Disconnection Fix (`launch/benchmark_sim.launch.py` + `nodes/depth_to_pointcloud_fixed_node.py`):**
   - Added static TF publishers `base_link -> depth_camera_link` (`0.12, 0.0, -0.02, 0, 0, 0`) and `depth_camera_link -> depth_link` (`0, 0, 0, 0, 0, 0`).
   - Updated `depth_to_pointcloud_fixed_node.py` to publish `/camera/depth/points` with `frame_id: depth_camera_link`.
   - Verified `tf_buffer.can_transform('base_link', 'depth_camera_link') == True` and `can_transform('base_link', 'depth_link') == True`.
3. **Single `odom -> base_footprint` TF Broadcaster (`nodes/odom_tf_relay_node.py`):**
   - Remapped Gazebo's internal `/tf` output to `/gz/wheel_tf_raw` and set `publish_tf: false` on all parallel estimator nodes (`F1a`, `F1b`, `F2`, `F3a`, `F3b`, `F4`, `F5`).
   - `odom_tf_relay_node.py` broadcasts the single `odom -> base_footprint` transform on `/tf` from `active_odom_topic` (default `/odom` in open-loop evaluation; switchable to any estimator topic in closed-loop evaluation).
   - Verified `1,193` `/tf` messages with **`0` backward timestamp jumps (`TF_REPEATED_DATA` = 0)**.
4. **Strict Ground Truth Isolation (`nodes/ground_truth_publisher_node.py`):**
   - Subscribes to `/world/lounge_world/dynamic_pose/info` (remapped to `/gz/dynamic_pose_info`) and publishes `/ground_truth/world_odom` (`frame_id: map`) and `/ground_truth/odom` (`frame_id: odom`, relative to initial spawn pose).
   - Verified that zero estimator nodes subscribe to `/ground_truth/*` or `/gz/dynamic_pose_info`.
5. **`robot_localization` Binary Permission Staging (`bin/ekf_node`, `bin/ukf_node`):**
   - Discovered that `capstone_ws/install/robot_localization/lib/robot_localization/ekf_node` and `ukf_node` lacked `+x` executable permissions (`-rw-r--r--`) in the workspace copy.
   - Without touching `capstone_ws`, staged executable copies into `experiments/limo_state_estimation_benchmark_20261107/bin/` and configured `LD_LIBRARY_PATH` in `launch/state_estimators.launch.py`.

---

## 2. Candidate Estimator Matrix (Refined per Instructions)

| Code | Estimator Name | Sensor Inputs | Category | Topic |
| :---: | :--- | :--- | :--- | :--- |
| **B0** | Raw Wheel Odometry (`AckermannSteering`) | Wheel Encoders | Baseline | `/odom` |
| **F1a** | EKF Legacy (`limo_ekf.yaml` reproduction) | Wheel ($x,y,\psi,v_x,v_y,\omega_z$) + IMU ($\psi,\omega,a$) | Same-Sensor Filter (Wheel+IMU) | `/odometry/ekf_legacy` |
| **F1b** | EKF Tuned (`robot_localization`) | Wheel ($v_x,v_y$) + IMU ($\psi,\omega_z$) | Same-Sensor Filter (Wheel+IMU) | `/odometry/ekf_tuned` |
| **F2** | UKF Tuned (`robot_localization`) | Wheel ($v_x,v_y$) + IMU ($\psi,\omega_z$) | Same-Sensor Filter (Wheel+IMU) | `/odometry/ukf_tuned` |
| **F3a** | Complementary Filter Standalone | Wheel ($v_x,v_y$) + IMU ($\psi,\omega_z$) | Same-Sensor Filter (Wheel+IMU) | `/odometry/comp_filter` |
| **F3b** | Complementary Filter + Slip/Stall Gate | Wheel ($v_x,v_y$) + IMU ($\psi,\omega_z$) + LiDAR Range-Rate Gate | Gated Multi-Sensor Filter | `/odometry/comp_slip_gate` |
| **F4** | 2D LiDAR Scan Matching Odometry (Trimmed ICP) | 2D LiDAR (`/scan`) | LiDAR-Only Odometry | `/odometry/lidar_odom` |
| **F5** | LiDAR Odometry + Slip-Gated Wheel + IMU EKF | LiDAR Odom ($x,y,\psi,v_x,v_y$) + F3b ($v_x,v_y$) + IMU ($\psi,\omega_z$) | Multi-Modal Sensor Fusion | `/odometry/lidar_imu_wheel_ekf` |

---

## 3. 1-Run Smoke Test Results (`S1_Normal_MultiTurn_SmokeTest`)

### 3.1 Topic Reception & Sim-Time Synchronization (`results/smoke_test/s1_smoke_test_report.json`)

| Topic | Messages | Rate (Hz) | Frame ID | `|stamp - sim_time|` (s) |
| :--- | :---: | :---: | :--- | :---: |
| `/clock` | 23,785 | 951.31 | - | 0.000 |
| `/imu` | 2,387 | 95.47 | `imu_link` | 0.008 |
| `/scan` | 238 | 9.58 | `laser_link` | 0.020 |
| `/camera/depth_image` | 239 | 9.58 | `depth_camera_link` | 0.002 |
| `/camera/depth/points` | 238 | 9.71 | `depth_camera_link` | 0.024 |
| `/lounge/overhead` | 361 | 14.51 | `overhead_camera_model/...` | 0.032 |
| `/ground_truth/odom` | 1,436 | 57.51 | `odom` | 0.001 |
| `/odom` (**B0**) | 1,193 | 47.77 | `odom` | 0.000 |
| `/odometry/ekf_legacy` (**F1a**) | 716 | 28.68 | `odom` | 0.009 |
| `/odometry/ekf_tuned` (**F1b**) | 1,193 | 47.73 | `odom` | 0.002 |
| `/odometry/ukf_tuned` (**F2**) | 1,185 | 48.16 | `odom` | 0.002 |
| `/odometry/comp_filter` (**F3a**) | 1,193 | 47.77 | `odom` | 0.000 |
| `/odometry/comp_slip_gate` (**F3b**) | 1,193 | 47.77 | `odom` | 0.000 |
| `/odometry/lidar_odom` (**F4**) | 238 | 9.58 | `odom` | 0.026 |
| `/odometry/lidar_imu_wheel_ekf` (**F5**) | 1,192 | 47.73 | `odom` | 0.010 |

### 3.2 Quantitative Accuracy Metrics (215 Synchronized Samples, 21.5s S-Curve Trajectory)

| Code | Method | ATE RMSE (m) | ATE Mean (m) | ATE Median (m) | ATE Max (m) | Final Pos Err (m) | RPE Trans RMSE (m) | RPE Rot RMSE (°) | Yaw RMSE (°) | Yaw Max (°) |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **B0** | Wheel Odometry (Raw) | `0.2084` | `0.1571` | `0.1047` | `0.3466` | `0.2990` | `0.0187` | `3.5245` | `8.0134` | `20.6432` |
| **F1a** | EKF Legacy (`limo_ekf.yaml`) | `0.0754` | `0.0606` | `0.0644` | `0.1403` | `0.0941` | `0.0209` | `0.2333` | `0.1837` | `0.6759` |
| **F1b** | EKF Tuned (`robot_localization`) | `0.0769` | `0.0623` | `0.0654` | `0.1428` | `0.0951` | `0.0211` | `0.1483` | **`0.1117`** | **`0.4210`** |
| **F2** | UKF Tuned (`robot_localization`) | `0.0771` | `0.0627` | `0.0659` | `0.1429` | `0.0948` | `0.0211` | **`0.1472`** | `0.1158` | `0.4544` |
| **F3a** | Complementary Filter Standalone | `0.0777` | `0.0635` | `0.0660` | `0.1431` | `0.0955` | `0.0207` | `0.2072` | `0.1616` | `0.6553` |
| **F3b** | Complementary Filter + Slip Gate | `0.0776` | `0.0635` | `0.0660` | `0.1431` | `0.0955` | `0.0207` | `0.2075` | `0.1634` | `0.6553` |
| **F4** | 2D LiDAR Scan Matching (ICP) | `0.0697` | `0.0626` | `0.0593` | `0.1178` | **`0.0926`** | `0.0204` | `1.5766` | `2.3992` | `6.1994` |
| **F5** | **LiDAR Odom + SlipGate + IMU EKF** | **`0.0633`** | **`0.0553`** | **`0.0535`** | **`0.1017`** | **`0.0926`** | **`0.0160`** | `0.2066` | `0.1675` | `0.6841` |

### 3.3 Video & Rosbag2 Verification
- **MP4 Video File:** `videos/smoke_test/s1_normal_multiturn_smoke_test.mp4` (`7,613,273` bytes, H.264 baseline profile)
- **Written / Decoded / Non-Black Frames:** `350 / 350 / 350` (`1920x960` @ `15.0 FPS`, duration `23.32 s`, `playable: true`)
- **Rosbag2 Archive:** `results/smoke_test/rosbag2_s1_smoke_test/rosbag2_s1_smoke_test_0.db3` (`86,849` total messages recorded across 14 topics over `24.62 s`)
