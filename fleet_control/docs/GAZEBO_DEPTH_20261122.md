# Gazebo LiDAR + Depth obstacle integration: criteria correction (2026-11-22, Claude, SIM_ONLY)

The LIMO's target configuration is **LiDAR + Depth**. LiDAR-only results are a comparison baseline and a Depth-fault case, never the final obstacle-avoidance verification.
Results: `local_data/raw_results/gazebo_depth_claude_20261122/`.
Harness:
- `fleet_control/tests/gazebo_depth/run_depth_targets.py`;
- `depth_sensors.launch.py`;
- `gate_depth_mode.py`.

## 1. Why Depth was off in the Gazebo web/test stack (source facts)

| Stage | State in `tests/gazebo_web/gazebo_world.launch.py` + `tests/web_mapping_isolated/stack.launch.py` | Exists elsewhere (reused, not rewritten) |
|---|---|---|
| Sensor model | **Present**: `limo_eval_diff/model.sdf` `depth_camera`, 320×240 R_FLOAT32, hfov 1.15 rad, clip 0.2–3.0 m, `/camera/depth_image` | – |
| Bridge | **Missing**: only clock, cmd_vel, odom, tf, imu, joint_states, pose and `/scan` are bridged | 11-13 office launch bridged `/camera/depth_image` |
| Image → points | **Missing** | `limo_capstone/depth_to_pointcloud_fixed_node` (`/camera/depth/points`, frame `depth_camera_link`) |
| Camera TF | **Missing**: `limo_diff_fortress.urdf` has no camera link | 11-13 static TF `base_link→depth_camera_link (0.12, 0, −0.02)` plus `depth_link` alias |
| Depth filter | **Not launched** | `limo_depth_obstacles/depth_obstacle_filter_node` (`/depth_obstacles/points`, `/auto_mapping/depth_status`). The installed package links to the working tree, which includes another owner's uncommitted change (sha `ba01c612…`). Run unmodified |
| Costmap | **Not registered**: the stack uses `integrated_nav2_lidar_only_slow_params.yaml` (scan + virtual_bumper) | `integrated_nav2_depth_slow_params.yaml` (untracked, other owner) adds a `depth_obstacles` source to both costmaps; reused here as an overlay with the same values |
| Gate | `stack.launch.py` hard-codes `depth_mode: disabled` | Gate default and real launch: `required` |
| Collision Monitor | Not in this stack and **not in the real launch** | Experimental C (`experiments/office_limo_sim_20261113/ws/src/limo_collision_guard`, 11-13), not built in this repository. Not used here: N/A |

The cause is configuration. The 11-21 web Gazebo harness was assembled without the depth path, and the Gate was set to `disabled` so it would arm without depth. The sensor itself exists.

## 2. What the earlier Gazebo numbers were measured with

Every run below logged Gate `depth_mode=disabled` and contains **0** depth-topic lines in the stack log. They are **LiDAR-only** results.

| Result | Configuration | How to read it |
|---|---|---|
| Gazebo full 21/2/1 (`gazebo_p0_claude_20261122/full_r12_a`), 20/3/1 (`full_r12_b`) | LiDAR only, Gate r12, `depth_mode=disabled` | Web/control/safety-chain regression under LiDAR only. **Not** an obstacle-avoidance verification of the target configuration |
| Gazebo r13 campaign 5/18/1, 20/3/1, r12 17/6/1 (`gate_r13_integ_claude_20261122/gz_*`) | Same | Same |
| Phase 1–5, Gate Reset (kinematic sim) | No depth sensor exists in that backend | Control-chain regressions only |
| 11-13 office run (`experiments/office_limo_sim_20261113`) | LiDAR + Depth + Collision Monitor C, Gate `depth_mode=required` | 190 s exploration, minimum LiDAR distance 0.299 m. No targeted low-obstacle test |

## 3. Gazebo furniture models: collision hulls the sensors cannot see

Gazebo LiDAR (`gpu_lidar`) and the depth camera render **visual** meshes only. The rmf_demos Office furniture uses collision meshes much larger than the visuals:

| Model | Collision mesh | Visual mesh |
|---|---|---|
| OfficeChairGrey | One hull from the floor to 0.49 m, x ±0.36, y −0.25 to 0.44 | At the LiDAR plane (0.161 m): a 4 cm column. Star base at ≤0.10–0.125 m |
| AdjTable | Block from the floor to 0.81 m, y −0.30 to 0.40 | Thin legs (y 0.16–0.24) plus foot rails at ≤0.03 m |

The other session (Nav2 D3/D1/D2) reached the same result independently for all models: `local_data/raw_results/nav2_d3_claude_20261122/d2_collision_vs_visual.json`.
Consequence: part of the Gazebo "D2 wedge" is a contact with geometry that no sensor, LiDAR or Depth, can detect. It is a **model artifact**, not a sensor or costmap failure. The targeted tests below therefore use **fixtures whose visual and collision geometry are identical**: chair star base, chair crossbars, desk stretcher, low box and a glass pane.

## 4. Pass criteria for the integrated (LiDAR + Depth) configuration

A run may PASS only if **all** of these hold before the goal is sent:
- depth_status `DEPTH_READY`, fresh (< 0.5 s);
- `/depth_obstacles/points` fresh (< 0.5 s);
- TF `base_footprint ← depth_camera_link`;
- both costmaps subscribed to `/depth_obstacles/points`;
- Gate NORMAL with `depth_mode=required`.

Otherwise the run is `SKIP_DEPTH_NOT_READY`.

Then:
- Nav2 SUCCEEDED;
- no contact (footprint-to-fixture clearance > 0 and no blocked motion > 0.5 s);
- minimum clearance ≥ 0.05 m.

Detection is reported in four stages per target: raw Depth points, filter points, costmap LETHAL cells, and LiDAR points.
A point cloud alone never counts as a PASS.
LiDAR-only runs are labelled BASELINE. The depth-fault run (depth bridge SIGSTOP during the approach) PASSES only if the Gate stops the robot.

## 5. Results (campaign `c_*`, 2026-11-22 16:52–18:10, domain 210, Gate r13 copy, SIM_ONLY)

### 5.1 Setup
- **Test room, not the Office world.** The smoke run in the Office world (`smoke3_lowbox_depth_sep`) stalled against AdjTable_2's invisible collision hull (§3). The harness builds a room from the Office world's physics, plugins and light, with 4 walls (interior x −1..6, y −1.5..1.5) and one fixture at (2.0, 0). Every part has identical visual and collision geometry. The lane runs from (0, 0) to the goal at (4.0, 0).
- **Localization:** truth odometry with `map→odom` identity (the D3 session's fixture). This isolates detection and avoidance from SLAM/AMCL.
- Profile: `integrated_nav2_lidar_only_slow_params.yaml` plus an overlay with the same `depth_obstacles` values as the other owner's `integrated_nav2_depth_slow_params.yaml`, in one of two layouts:
  - `shared`: the depth source is added to `obstacle_layer`, as in the existing depth profile;
  - `separate`: a second `nav2_costmap_2d::ObstacleLayer` named `depth_layer` holds only the depth source.
- Real-time factor during the stationary phase: 0.986–1.005. No shared files were modified.

### 5.2 Per-target result (4 detection stages, then avoidance)

Stationary 6 s at 2.0 m from the fixture (identical in every run of a target):

| Target | Raw Depth pts | Filter pts | LiDAR pts | Local / global LETHAL (depth configs) | LiDAR-only local / global LETHAL |
|---|---|---|---|---|---|
| chair_base (spokes 0.04–0.075 m) | 321 | 179 | 2 (column) | 88 / 138 | 38 / 52 |
| chair_bars (bars 0.08–0.11 m) | 345 | 265 | 4 (legs) | 84 / 144 | 58 / 116 |
| desk_frame (stretcher 0.06–0.10 m) | 972 | 500 | 4 (legs) | 110 / 158 | 48 / 96 |
| low_box (0.08 m high) | 101 | 142* | **0** | 50 / 70 | **0 / 0** |
| glass (transparency 0.9) | 2025 | 1932 | 40 | 60 / 60 | 60 / 60 |
| none (control) | 0 | 0 | 0 | 0 / 0 | – |

\* The filter accumulates over a window, so it can exceed one raw frame. In every depth run the first filter hit and the first LETHAL cell appeared at 2.0 m, i.e. as soon as the measurement started.

Avoidance (Nav2 goal past the fixture; `min clear` = footprint-to-fixture clearance from ground truth):

| Target | LiDAR+Depth, **separate** layer | LiDAR+Depth, shared layer | LiDAR only (baseline) |
|---|---|---|---|
| chair_base | PASS_AVOIDED 2/2, min clear 0.206 | PASS_AVOIDED 2/2, min clear **0.079** | **FAIL** 1/1 (contact with the spokes at x 1.58), 1 NOT_READY |
| chair_bars | PASS_AVOIDED 2/2, 0.207 | PASS_AVOIDED 2/2, 0.215 | PASS_AVOIDED 2/2, 0.197 |
| desk_frame | PASS_AVOIDED 2/2, 0.217 | **FAIL 2/2** (contact) | INCONCLUSIVE 2/2 (see 5.4) |
| low_box | PASS_AVOIDED 2/2, 0.190 | 1 PASS_AVOIDED (0.124), **1 FAIL** (contact) | INCONCLUSIVE 1/1 (see 5.4), 1 NOT_READY |
| glass | PASS_AVOIDED 2/2, 0.205 | NOT_READY 2/2 (infra) | PASS_AVOIDED 2/2, 0.175 (not valid for the real robot, see 5.5) |
| none | PASS_AVOIDED 1/1 | – | – |

Depth fault (separate layer; depth processes SIGSTOPed at 1.0 m range):
- chair_base: Gate `FAILSAFE_SENSOR_TIMEOUT`, robot stopped 0.53 s after the fault, min clear 0.459 → **PASS**.
- low_box: stopped after 0.549 s, min clear 0.609 → **PASS**.

### 5.3 Shared vs separate depth layer (main finding)
In the shared layer, the fixture's LETHAL cells decreased as the robot approached:
- desk_frame r0: 154 → 111;
- low_box r1: 80 → 59.

The robot then touched the fixture. In the separate layer, the fixture's cells stayed constant (desk_frame: 158 throughout) and every run passed.

Mechanism (matches the data; not instrumented per ray):
- The LiDAR (plane at 0.161 m) raytraces over a low obstacle and clears its depth marks in the shared `obstacle_layer`.
- This happens once the obstacle leaves the camera's FOV (hfov 1.15 rad) or enters its minimum range (0.2 m).
- With its own layer, only the depth source can clear depth marks.

**Change proposal for the depth-profile owner** (not applied; `integrated_nav2_depth_slow_params.yaml` is untracked and owned by someone else):
- Move `depth_obstacles` from `obstacle_layer.observation_sources` into a separate `depth_layer` (`nav2_costmap_2d::ObstacleLayer`) in both costmaps, placed before `inflation_layer`.
- The overlay used here is `local_data/raw_results/gazebo_depth_claude_20261122/c_depth_sep/run/nav2_overlay.yaml`.
- Trade-off: depth marks that leave the FOV persist until the depth camera sees the area free again. This can leave stale obstacles behind moving objects. Clearing and persistence settings need a real-robot check.

### 5.4 Runs that are not results
- **NOT_READY (4 runs: glass shared ×2, chair_base LiDAR r1, low_box LiDAR r1).** The Nav2 lifecycle `change_state` service response timed out at bring-up, so the costmaps never subscribed. This is a test-infrastructure failure: no goal was sent, and the runs are excluded.
- **LiDAR-only desk_frame ×2 and low_box r0 (harness label PASS_STOPPED, reclassified INCONCLUSIVE).**
  - The Gate latched `STALL_LATCHED` 1.5 s after the goal: `cmd_vx=wheel_vx=0.15`, `lidar_speed=0.001–0.002`. Ground truth shows the robot had moved 0.236 m.
  - So the obstacle response was never exercised. This is a Gate false stall, because the LiDAR-odometry speed reads near zero along the featureless lane.
  - Cause assumed, not yet confirmed: the same ICP-degeneracy family as D1.
  - The harness now labels this case `INCONCLUSIVE_GATE_STOP`. Existing `run.json` files were not rewritten.
  - Every run had a short `STALL_PENDING` at 0.13–0.28 s; it was not determined why only these three never cleared.
- **Correct stall latches.** In the contact FAILs (chair_base LiDAR, desk_frame shared, low_box shared r1), the Gate stall latch after contact was the right reaction. The cause of those failures is detection or costmap, not the Gate.

### 5.5 Not detected and not verifiable in Gazebo
- **LiDAR alone does not detect:**
  - the low box (0 points, 0 LETHAL);
  - the chair spokes (only the column: 2 points, then contact).
  
  This confirms that Depth is required for low obstacles.
- **Glass is not verifiable in Gazebo.** Both the `gpu_lidar` and the depth camera return points from the pane: transparency affects rendering colour only. Real LiDAR and depth cameras can miss glass, or see through it. The glass PASS results say nothing about the real robot.
- **Not tested:** dynamic obstacles; obstacles below the filter's `min_height` (−0.02 m, shifted for the sim floor); black or specular surfaces; obstacles inside the 0.2 m camera minimum range at start-up; the Collision Monitor (not deployed in this stack or the real launch); the shared/separate difference with SLAM/AMCL localization.

### 5.6 Verification level
| Item | Gazebo (this campaign) | Real LIMO |
|---|---|---|
| Depth sensor output | Verified (simulated camera) | Not verified today (robot offline). 11-20 stationary measurements exist separately |
| Filter output | Verified (unmodified filter, sim height band) | Not verified (real floor and band differ) |
| Costmap marking | Verified (LETHAL cells in the fixture, both costmaps) | Not verified |
| Avoidance | Verified for 4 low fixtures with the separate layer, 2 runs each | Not verified (no driving allowed) |
| Depth fault stop | Verified (Gate stop in 0.53–0.55 s) | Not verified |
| Glass | Not verifiable | Not verified |
