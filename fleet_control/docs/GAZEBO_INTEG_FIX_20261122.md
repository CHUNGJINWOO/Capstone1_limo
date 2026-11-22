# Gazebo 통합 실패 수정 · Phase 4·5 안정화 (2026-11-21~22, Claude, SIM_ONLY)

All results are from the isolated Gazebo and kinematic sims, with `ROS_LOCALHOST_ONLY=1` and private domains 224, 228 and 190–219. No LIMO, no 18190 server, no motor unlock and no real-robot Mapping START were used. No commit or push.
Results root: `local_data/raw_results/gazebo_integ_claude_20261121/`. Timeline: `fleet_control/tests/gazebo_web/timeline_recorder.py`, and the analysis tool is `timeline_analyse.py`.

## 1. First failure: g03 not moving

| | |
|---|---|
| Problem | g03 (Mapping explore) failed 6/7 in the full suite and passed 9/9 when run alone. |
| Investigation | The timeline was healthy at every stage: Arbiter, Watchdog, Gate, `/cmd_vel_locked`, `/sim/drive_cmd`. But truth speed was 0 and the front scan read 0.525 m. |
| Cause | Test order: the manual drives in earlier scenarios (g02, g07) left the robot physically wedged against a table. |
| Fix (harness) | `run_gazebo_web.py` `home()` drives back to spawn through the web Manual bench before each scenario, using truth feedback and a stuck back-off. `open_heading()` picks the facing direction. |
| Verification | g03 moved 2.23 m (`after_g03seq_r1`). g03 PASS in full runs r3 and r4. |

## 2. Second failure: Mapping session lost, then FAULT

- **Cause:** about 0.3 s gaps in `/clock` delivery stopped the Agent's sim-time timers (heartbeat at 0.1 s, task loop at 0.05 s). The Watchdog judges in steady time and raised `AGENT_HEARTBEAT_LOST`. The mapping worker then called `lose_session`, giving STOP UNKNOWN and FAULT.
- **Fix (product, Agent):** in `ros_adapter.py`, both timers moved to a `ClockType.STEADY_TIME` clock. Data stamps and sim-time judgements are unchanged.
- **Verification:** across r3 and r4, the only `AGENT_HEARTBEAT_LOST` is the deliberate Agent kill inside f02. Unit-test failures match the baseline set of 21.

## 3. Harness side effect: 21 cascaded failures in r2

- The first `open_heading()` chose "the farthest open direction". g07 then drove into the middle of the room, where fewer than 100 returns were within 9.5 m. The Gate entered `FAILSAFE_ICP_DEGRADED` (inliers=94<100), which is correct. Because motion was then blocked, the robot could not leave and every later scenario failed.
- **Fix (harness):** pick a direction with at least 2.5 m clear **and** a wall within range, and record the scan valid-point count (`scan_valid_at_start` 188). g04 was also added to `HOME_BEFORE`, so the run doesn't depend on g03b's homing.

## 4. Product defect: robot kept driving after a cancel (f07)

- **Reproduction:** freeze `bt_navigator` with SIGSTOP, then cancel the mission from the web. The Arbiter kept forwarding the Nav2 controller's output until the cancel was confirmed or `cancel_result_timeout_s` (5 s) ran out. Measured in r3: **driving continued for 4.0 s or more and 0.50 m**, stopping only when `bt_navigator` was resumed.
- The earlier gz_target6 PASS did not measure this: the robot was not moving at the event, and the Gate's stall latch stopped it.
- **Fix (product, Agent):** `velocity_arbiter.py` `nav_blocker` returns `CANCEL_REQUESTED` as soon as `active.cancel_requested` is set, so Nav2 output is zero. Terminal confirmation and the reconciliation flow are unchanged.
- **Verification:** `after_cancelfix_nav_r2` f07 PASS. The robot was moving at 0.102 m/s and stopped 0.17 s later, after 0.016 m. g05, g06 and f06 also passed (7/7). Phase 1/2/3 and Gate Reset passed 19/0, 17/0, 18/0 and 13/0.

## 5. Before and after, same conditions

Same domain 224, port 28961, spawn A1, harness and r12 Gate:

| Run | Result | Notes |
|---|---|---|
| Earlier full runs (codex_fix, before the changes) | 13 PASS / 11 FAIL ×2 | g03 not moving, followed by a cascade |
| after_full_r1 (homing + steady timers) | 13 / 11 | stuck before g03, g04 frame mismatch, d06 order |
| after_full_r2 | 3 / 21 | g07 ICP_DEGRADED cascade (harness side effect in §3) |
| **after_full_r3** | **18 / 5 / SKIP 1** | |
| **after_full_r4** (+ cancel fix) | **18 / 5 / SKIP 1** | g08 also PASS |

## 6. Remaining failures and how they are classified (not fixed)

| Symptom | Classification | Evidence | Level |
|---|---|---|---|
| f04 latches `cmd_input:NOT_EXCLUSIVE` at start | cascade from f03: SIGKILL of the Watchdog leaves a DDS ghost writer | f04 failed in every full run and passed alone in gz_target6 | observed, cause not proven |
| r4 g05, then g06, f06, f07 | separate defect: map/localization quality | large truth→map rotation (−0.85 to −1.0 rad) means AMCL tracking is lost (match 0.32) and the initial pose lands in `UNKNOWN_AREA`; a small rotation (0.16) passes. Consistent with the known ICP rotation under-read (~9%) | hypothesis |
| r3 g08 drove into an obstacle the LiDAR could see (0.88 m down to 0.15 m), wedging g05/g06 | separate defect: Nav2 settings | RPP `use_collision_detection: false` (`capstone_ws/.../integrated_nav2_lidar_only_slow_params.yaml`, other owner, not modified) | hypothesis, costmap not captured |
| robot stuck at the g03 exploration end point (bridge input present, truth 0, front 1.45 m clear) | separate: perception limit | low obstacles below the LiDAR plane, e.g. chair bases. **Also applies to the real robot.** | observed |
| Phase 2 n08–f06 `UNKNOWN_AREA` (2/11 runs, including before the change) | separate, intermittent | the saved map has single-cell unknown speckles, and the initial-pose check judges only the centre cell; exact pose not recorded | hypothesis |
| global costmap scan drops while the map frame switches (SLAM ends → AMCL starts) | observed, impact not confirmed | `Message Filter dropping` at sim 471–538 and 695–1142; none during driving | observed |

## 7. Phase 4·5

- 6 runs with the recorder attached. Phase 4 server_first 7/1 and 8/0, supervisor_first 8/0 and 8/0. Phase 5 8/0 and 8/0, including normal requests, two-user FIFO, link isolation and revoke.
- **Scan dropout did not reproduce:** largest gap 0.13 s, 0 sensor faults. It also did not reproduce in the DDS restart probe (14 cycles across default and UDP-only transports), so the SHM hypothesis is rejected. Cause still unknown.
- server_first_r1 s02 FAIL: the `ros2 topic info` CLI hit discovery mid-way and saw 0 publishers. The recorder received 200 `/cmd_vel_locked` messages in the same window. Harness check-method flake.
- `--require-supervisor-ready`: no deadlock in either start order. Restart counts were 0 in every run's final readiness.
- Mission state after restart: Phase 3 f08 gave `RECONCILIATION_REQUIRED` with mode IDLE, and reusing the old session was rejected.
- Nav2 startup failure (change_state response timeout): did not reproduce in these 6 runs; not resolved.

## 8. Gate resume (isolated rig, `fleet_control/tests/gate_resume_stale_cmd_isolated.py`)

- **r12 (unchanged):** for a 0.2 s stall (wall and sim clocks) and 0.5–2 s stalls (sim clock), the old 0.08 command is output again for 1–2 ticks (about 2–10 ms) right after resume (STALE_REUSE). Movement after resume was 0.004 m or less. It was also seen in Gazebo d01/d03 (`gate_nonzero_after_resume` 2).
- **Candidate** (`gate_candidate_r12_resume/`, `resume_hold.patch`, 36 lines): after a control-tick gap over 0.1 s, output stays at 0 until 0.1 s has passed and a new input arrives 0.05 s after the hold starts. Result: zero and silent cases OK 5/5 in every condition (excluding NOT_ARMED), and live resume within 0.10–0.12 s.
- **Not applied** to the operating Gate, `capstone_ws`, or r12. The decision is the Gate owner's.

## Changed files

- Product (Agent): `limo_robot_agent/ros_adapter.py` (steady timers) and `velocity_arbiter.py` (`CANCEL_REQUESTED`). Originals: `pre_change_originals/`.
- Harness: `tests/gazebo_web/run_gazebo_web.py`.
- New: `tests/gazebo_web/{timeline_recorder,timeline_analyse}.py`, `tests/gate_resume_stale_cmd_isolated.py`, `tests/mode_e2e/dds_restart_probe.py`.
- Hashes: `source_after.sha256`. The Gate r12 and `capstone_ws` Gate hashes are unchanged (`gate_protected_before.sha256` OK).

## Reproduction (from the results root)

```
./gz_seq.sh <new_name> 1 ALL                      # full Gazebo suite (domain 224, port 28961, timeline)
./gz_seq.sh <new_name> 1 g00_bringup_graph_and_clock,g03_mapping_start_explore_save_stop,g04_load_saved_map_initial_pose_localized,f07_cancel_unanswered_bt_navigator_frozen
python3 fleet_control/tests/gazebo_web/timeline_analyse.py <run dir> <scenario> --bin 0.5
python3 fleet_control/tests/gate_resume_stale_cmd_isolated.py --out <new dir> [--gate <candidate>] --kinds zero,silent,live
../refactor35_claude_20261121/regress.sh ../gazebo_integ_claude_20261121/<new dir>   # Phase 1/2/3 + Gate Reset
```

---

# Part 2 (2026-11-22, Claude, SIM_ONLY): velocity re-output safety and remaining Gazebo failures

Results root: `local_data/raw_results/gazebo_p0_claude_20261122/` (`env.txt`: Humble, Gazebo Sim 8.15.0, Python 3.10.12). Isolated only: no LIMO SSH, no 18190 change, no motor unlock, no real-robot Mapping START. No commit, push or worktree. The Gate r11/r12 originals, patch 06, `capstone_ws`, `rmf_adapter/` and `fleet_frontier_candidates.cpp` are unchanged (`source_before.sha256`, `source_after.sha256`).

## P1. Contract: no command from before a reset or resume is reused

| Layer | Rule | Where |
|---|---|---|
| Agent VelocityArbiter | **Control generation** = (operating mode, server boot, lease epoch, owned goal UUID, cancel_requested, common conditions OK: Gate NORMAL, Watchdog authorized, session). A generation change drops the stored Nav2 sample. A Nav2 sample is forwarded only if received ≥ `NAV_GEN_SETTLE_S` (0.3 s) after the generation started (`NAV_INPUT_PREDATES_GENERATION`). In MANUAL, a deadman already held when the generation started must be released and pressed again (`MANUAL_DEADMAN_NOT_REARMED`). | `velocity_arbiter.py` (product) |
| Motion Watchdog | If its own tick stalled (> `self_stall`), queued inputs delivered at wake-up are dropped and 0 is published (`dropped_inputs`) | `motion_watchdog.py` (product) |
| Gate (candidate r13, not applied) | r12 + `resume_hold` + **writer identity cross-check**: bind `/cmd_vel_raw` only to the GID that the Watchdog's own status reports (`output_gid`, `boot_id`, < 1 s old). A boot change gives sticky `WRITER_RESTARTED`, a GID change gives `PUBLISHER_CHANGED`, and reset sets `latest_cmd = 0` and starts the resume hold | `gate_candidate_r13/stall_detector_node.py`, `r13_vs_r12.patch` (100 lines) |

Events covered: Gate resume, Watchdog restart, Agent restart (new server/lease generation), MANUAL↔NAVIGATION, Nav2 cancel (immediately 0 from `cancel_requested`, kept until a new goal UUID is admitted, also if the cancel result never arrives), and DDS publisher replacement (r13).

## P2. Tests: velocity re-output and cancel

| Test | Before change | After change |
|---|---|---|
| `tests/test_arbiter_generation_20261122.py`, 7 cases: settle-then-drive, cancel zero with late samples, terminal → new goal tail, NAV → MANUAL with deadman held, Gate resume, Watchdog re-authorization, lease epoch, mapping owner | 7 FAIL (stale samples reused) `unit/arbiter_generation_pre_change.txt` | 7/7 OK |
| `tests/test_watchdog_stall_backlog_20261122.py` (queued input after self-stall) | FAIL | OK, with 23 Watchdog tests |
| Unit suite (`unit_suite_after/SUMMARY.txt`) | baseline: 22+ non-zero | 22 non-zero, a subset of the baseline; no new failure. `test_public_manual_arbiter_locked_ros` fails on a stale pinned `ManualWebBench` hash (Codex test, same at baseline, not modified) |
| Isolated Gate resume rig (`gate_resume_stale_cmd_isolated.py`, zero/silent/live × wall/sim × 0.2/0.5/2 s × 3) | r12: STALE_REUSE | r13 + status writer: zero/silent OK in all cases, live resumes in 0.105–0.118 s; 2 NOT_ARMED (rig start-up, not a reuse) `resume_r13/` |
| Writer-swap graph state from Gazebo r4 (`gate_writer_swap_graph_20261122.py`) | r12: `ACCEPTS_UNVERIFIED_NEW_WRITER` | r13: `ZERO_UNTIL_RESET` (`WRITER_RESTARTED`) |
| **Gazebo d01/d03/d04 `gate_nonzero_after_resume`** (Gate SIGSTOP, then resume) | r12: 1/0/1 (full_r12_a), 1/1/1 (full_r12_b) | r13: 0/0/0 (full_r13_a) |
| Gazebo f07 (cancel, bt_navigator frozen, result never arrives) | — | PASS in tgt_r12_r1 and full_r12_a: stopped 0.18 s / 0.016 m, mission `FAILED` after the timeout without resuming |

A normal new goal still drives after admission: tgt_r12_r1 g05 and g06 PASS, full_r12_a/b g08 PASS, and Phase 1/2/3 normal navigation PASS.

## P3. The 5 remaining Gazebo failures (from r3/r4: 18 PASS / 5 FAIL / 1 SKIP)

| Failure | Cause (evidence) | Resolution | Level |
|---|---|---|---|
| **f04** `NOT_EXCLUSIVE` at start | f03 SIGKILLs the Watchdog. The dead writer's endpoint lingers 16–20 s. The r4 Gate log shows only the dead endpoint for ~20 s (r12 accepted the new writer under the old binding), then the unresolved new endpoint appears and it latches 0.37 s into f04. | Harness: f03 waits until the Gate is NORMAL and bound to the **new** Watchdog's `output_gid` with one writer (`gate_bound_to_new_writer_after_s` 2.3–4.0 s, `gate_normal_bound_to_dead_writer_s` 0.0). FAILSAFE is kept while two writers exist: reset refused with `INPUT_WRITERS:2`. Product candidate: Gate r13 closes the window where it accepts an unverified writer. | **f04 PASS in 3/3 full runs + tgt** (fixed) |
| **g05** cancel, then g06 refused (`localization_not_confirmed`) | **ICP odometry under-reads rotation.** Against Gazebo truth, the yaw error during the g05 turn was −0.205 rad over 2.99 rad of rotation (≈7 %, full_r12_a). AMCL yaw error went from −0.02 to −0.20 rad, cov_yaw 0.04 → 0.66, and match fell below 0.40. The Localization worker reported DEGRADED, the server invalidated the mission because the safety epoch changed, and the cancel followed. AMCL position error stayed ≤ 0.13 m. The map→world rotation (−0.75 to −1.04 rad) is the arbitrary SLAM frame and not an error. | **Not fixed:** this is the ICP node in `capstone_ws` (`lidar_scan_matcher_odom_node`) and the AMCL/odometry settings, owned by others. The safety response itself (cancel, stop, refuse a new goal) is correct. | cause measured (timeline `amcl`/`icp` vs `truth`), fix is a proposal |
| **g06 / f06 / f07** prerequisite failures | Follow-on of g05 (localization DEGRADED) or of the robot wedge below. full_r12_b f07: the server refused the mission (`safety_not_ready`, Nav2 not idle after f06's AMCL restart); in the same window scan, ICP and est_odom stopped together (`NO_ARRIVAL`, load 4–5). | No product change; the Gate faulted correctly. | observed, scan-stop root cause unknown |
| **Collision** (r3 g08; r13_a after g03) | full_r13_a: Mapping exploration drove into the robot's wedge point, world ≈ (14.63, −8.03), **0.65 m from `OfficeChairGrey_8`**. The command was 0.10 m/s while truth speed went to 0, and the front LiDAR showed 0.48–1.67 m clear, so contact was with structure outside the 2D scan plane (chair base or seat). Every later r13_a failure (g08, g05, g06, f07, home `NOT_HOME` ×30 back-offs) follows from it. The Gate's stall latch held. | **Not fixed:** needs depth in the costmap (the Gazebo stack runs `depth_mode: disabled`) or keep-out areas, which is other owners' configuration. The RPP collision overlay (`nav2_overlay_rpp_collision.yaml`) was **not tested** and would not help for obstacles the LiDAR cannot see. **The same limit applies to the real robot.** | cause strongly indicated, no contact sensor |
| (r3) g05 wedge after g08 | same as the collision row | — | — |

Gazebo `world`: `experiments/office_limo_sim_20261113/office_limo.world`, spawn (10.09, −6.98).

## P4. Before and after under the same conditions (domain 224, port 28961, spawn A1)

| Run | Gate | PASS / FAIL / SKIP | First failure, cause |
|---|---|---|---|
| r3, r4 (2026-11-22, before) | r12 | 18 / 5 / 1 (×2) | f04 ghost, g05 localization, g08 collision (r3) |
| tgt_r12_r1 (targeted 9) | r12 | 9 / 0 / 0 | — |
| full_r12_a | r12 | **21 / 2 / 1** | g05 (ICP yaw, AMCL DEGRADED), then g06 |
| full_r12_b | r12 | **20 / 3 / 1** | g05 (same), g06, f07 (`safety_not_ready` + scan stopped) |
| full_r13_a | r13 candidate | 19 / 4 / 1 | robot wedged at the chair after g03, then g08, g05, g06, f07 |
| Phase 1 / 2 / 3 / Gate Reset (`regress_after`) | r12 | 19/0, 17/0, 18/0, 13/0, snapshot unchanged | — |
| Phase 4 server_first / supervisor_first | r12 | 8/0, 8/0 | — |
| Phase 5 p5_r1 / p5_r2 / p5_r3 | r12 | 6/2, **8/0, 8/0** | r1 only: v04 second visitor Nav2 `failed to create plan` (tolerance 0.30), v07 Nav2 aborted (v07 also failed 1 of 3 runs on 2026-11-21, before this change). Intermittent; not reproduced in r2/r3 |

r12 and r13 cannot be compared one to one: the r13 run had the physical wedge, which comes before the Gate in the causal chain. The r13 safety result that holds is d01/d03/d04 0/0/0 and f03/f04/d05 PASS.

## P5. Remaining issues and real-robot block conditions

- Gate r13 is a **proposal for the Gate owner** (`gate_candidate_r13/r13_vs_r12.patch`). It is not applied to the operating Gate, `capstone_ws` or r12. Until it is, r12 re-outputs 1 tick after resume.
- ICP rotation under-read → AMCL DEGRADED (g05): proposal for the localization/ICP owner.
- Obstacles outside the LiDAR plane (chairs, desks): proposal for the Nav2/costmap owner (depth layer, keep-out areas).
- Scan/ICP stopping together under load (full_r12_b f07): cause not identified.
- **Real robot:** none of this was verified on the LIMO. Real-robot movement stays blocked until the Gate r13 (or equivalent) contract, the ICP/AMCL fix and below-LiDAR obstacle handling are verified, along with the existing items (Gate E, STM32 timeout, E-Stop, braking distance).

## P6. Changed files (Part 2)

- Product (Agent): `limo_robot_agent/velocity_arbiter.py` (control generation, settle, manual re-arm) and `limo_robot_agent/motion_watchdog.py` (self-stall backlog drop). Originals: `gazebo_p0_claude_20261122/pre_change/`.
- Harness: `tests/gazebo_web/run_gazebo_web.py` (f03 waits for rebind to the new writer, `GATE_LAST`), `tests/gazebo_web/timeline_recorder.py` (`amcl`, `icp` streams), `tests/web_mapping_isolated/stack.launch.py` (env-gated `GATE_WRITER_STATUS`, `NAV2_PARAMS_OVERLAY`), `tests/gate_resume_stale_cmd_isolated.py` (`swap` kind, status writer, `--gate-param`).
- New: `tests/test_arbiter_generation_20261122.py`, `tests/test_watchdog_stall_backlog_20261122.py`, `tests/gate_writer_swap_graph_20261122.py`.
- Candidates (results dir only): `gate_candidate_r13/`, `nav2_overlay_rpp_collision.yaml` (not tested). Analysis: `amcl_vs_truth.py`.
- Records note: the first `p5_rerun.sh` version mistakenly re-ran `regress.sh` into `regress_after/`. The harnesses refused to start (directory exists), so the result directories are original, but the 4 console `.out` files and the snapshot files were overwritten. See `regress_after/INCIDENT_20261122.txt`.

## P7. Reproduction (Codex)

```
cd local_data/raw_results/gazebo_p0_claude_20261122
python3 ../../../fleet_control/tests/test_arbiter_generation_20261122.py
python3 ../../../fleet_control/tests/test_watchdog_stall_backlog_20261122.py
python3 ../../../fleet_control/tests/gate_writer_swap_graph_20261122.py --gate gate_candidate_r13/stall_detector_node.py
python3 ../../../fleet_control/tests/gate_resume_stale_cmd_isolated.py --out <new dir> --gate gate_candidate_r13/stall_detector_node.py --gate-param input_writer_status_topic:=/fleet/motion_watchdog/status --kinds zero,silent,live,swap
./gz_seq.sh <new name> 1 ALL                                    # Gate r12
GATE_SCRIPT=$PWD/gate_candidate_r13/stall_detector_node.py GATE_WRITER_STATUS=/fleet/motion_watchdog/status ./gz_seq.sh <new name> 1 ALL
python3 amcl_vs_truth.py <run dir>                              # AMCL vs Gazebo truth in the map frame (P3)
```

---

# Part 3 (2026-11-22 afternoon, Claude, SIM_ONLY): Gate r13 integration readiness and the velocity generation contract

Results: `local_data/raw_results/gate_r13_integ_claude_20261122/`. Each run has its own directory with `MANIFEST.json`, holding the run_id, git HEAD, the status digest, sha256 of the 280 fleet_control sources, the Gate script and Nav2 profile hashes, and the environment. Each also has `FINISH.json`. Every run reported `sources_changed_during_run: 0`. The formal Gate (`capstone_ws/.../stall_detector_node.py`, the owner's 8-line uncommitted change, mtime 11-19) was **not modified**. r12 (`7fdd3aa1…`) and patch 06 were not modified either.

## Q1. Was the 0.3 s generation settle enough? No, measured; now fixed with producer liveness

- Measurement (`fleet_control/tests/nav2_smoother_tail_probe_20261122.py`, real Humble `nav2_velocity_smoother` with the shared slow profile, result `smoother_tail.json`): when the controller stops **without** sending a zero, the smoother keeps publishing the old command (0.10 m/s) for **0.91–0.95 s** (3 of 3). With a final zero or a zero stream the tail is 0 s. So a settle of 0.3 s alone could forward a previous-generation command.
- Nav2 `Twist` has no stamp and no goal ID. The fix assumes no such fields. The Arbiter (mock/sim only, `velocity_arbiter.py`) now also subscribes to the smoother's **input** `/fleet/cmd_vel_nav` (controller_server and behavior_server output, `topics.NAV_RAW`). A smoothed sample is forwarded only if a raw controller message arrived **after the generation start** and **within 0.25 s** (`NAV_RAW_MAX_AGE_S`). Otherwise the reason is `NAV_PRODUCER_NOT_LIVE`. The generation key is unchanged: mode, server boot, lease epoch, goal UUID, cancel and common_ok. `nav_raw_t` is reset at every generation change.
- Remaining bound (assumption, not measured on the LIMO): within one goal, a smoother replay can last at most 0.25 s after the controller dies. A DDS-delayed old sample arriving after the 0.3 s settle is not excluded by design. On localhost the delay is far below 0.3 s, but this is not measured on real Wi-Fi or Ethernet.
- Unit tests: `fleet_control/tests/test_arbiter_generation_20261122.py` (14 tests). They cover:
  - cancel, new goal, MANUAL↔NAVIGATION, Mapping end then a new mapping goal, lease epoch, server boot change, and Agent restart (new Arbiter);
  - Gate FAILSAFE → INITIALIZING → NORMAL and a late old sample inside the settle window;
  - smoother replay without a live controller, and a new goal with only the smoother tail;
  - a normal new goal driving once its controller publishes.

  Result: **14/14 PASS** (`unit/arbiter_generation_after.txt`). Against the previous Arbiter, exactly the 2 smoother-replay tests fail (`unit/arbiter_generation_vs_previous_arbiter.txt`). Full unit suite: 22/123 non-zero, the same set as before the change (`unit_suite/`).
- In Gazebo and Phase 5, `NAV_PRODUCER_NOT_LIVE` appeared only **after** the controller had stopped: at the end of a goal or in recovery loops. It always appeared as `NAV2 → NAV_PRODUCER_NOT_LIVE (≤0.5 s) → NAV_INPUT_STALE`. It never interrupted a moving goal. Normal goals were admitted and drove: Phase 3 18/0, Phase 5 v02/v03 (3 runs), and in gz_r13_b g04 plus the goal-driven f06/f07, which passed.
- Watchdog: unchanged (`ccdc2fe3…`). It forwards per message, keeps no previous command, and drops input while not AUTHORIZED and after a self-stall.

## Q2. Gate r13 against r12, isolated rig (`gate_resume_stale_cmd_isolated.py`, 3 rounds, wall and sim clock, D = 0.2/0.5/2.0 s)

| Case | r12 (`rig_r12_resume_swap`) | r13 (`rig_r13_resume_swap`) |
|---|---|---|
| zero (upstream zero during the stop) | wall 0.2 s: 3/3 STALE_REUSE; sim: 8/9 STALE_REUSE | **18/18 OK** |
| silent (no fresh input) | wall 0.2 s: 3/3 STALE_REUSE; sim: 6 STALE_REUSE, 1 NOT_ARMED | **18/18 OK** |
| live (normal motion resumes) | 18/18 OK | 18/18 OK |
| swap (writer killed, new writer) | 18/18 RECOVERED | 17 RECOVERED, 1 NOT_ARMED |

NOT_ARMED means the fake sensors did not become ready within 30 s, so the case never reached NORMAL. It is a setup failure, counted as **SKIP**, and happened once with each Gate. In 17 valid swap cases, r13 never moved on the unverified new writer and recovered after the operator reset.

## Q3. Gazebo full suites, plus Phase 1–5 with r13

| Run | Gate | PASS/FAIL/SKIP | First failure and cause |
|---|---|---|---|
| gz_r13_a | r13 | 5/18/1 | End of g03 exploration: RPP drove a forward arc at 0.10 m/s into a cubicle wall that the scan could see (front 0.70 → 0.18 m; truth near `SmallCubicle` 0.87 m). The robot stayed pressed against it at front 0.10–0.13 m, and ICP inliers fell to 92 (< 100). The Gate went to `FAILSAFE_ICP_DEGRADED` and **refused every reset while the sensor fault persisted** (1376× `REFUSED: SENSOR_FAULT:icp`), so all 18 later cases cascaded. This is the correct Gate response. The cause is Nav2/RPP: the same mechanism as D3 in `GAZEBO_OPEN_DEFECTS_20261122.md`. |
| gz_r13_b | r13 | 20/3/1 | g08: stalled at 0.15 m/s with LiDAR speed 0 and front 1.27 m clear, next to `OfficeChairGrey_11` (0.53 m) and `AdjTable_10` (0.63 m), which are outside the scan plane (D2). `STALL_LATCHED` followed, the mission was CANCELED and set to RECONCILIATION_REQUIRED. g05/g06 were then refused with 409 `safety_not_ready` (correct refusal). |
| gz_r12_c | r12 | 17/6/1 | g04/g08/g05/g06/f06/f07: `initial pose rejected: UNKNOWN_AREA`. The robot's pose after g03 lies in unknown cells of the web map (map coverage). Not velocity-related. |
| phase1_r13 | r13 | 19/0/0 | – (r12 baseline 19/0) |
| phase2_r13 | r13 | 16/1/0 | f04 (Agent restart in Mapping): a kinematic-sim `/scan` dropout (`scan:NO_ARRIVAL`, the known §7 dropout) happened **inside** f04, about 4 s after the Agent kill. The harness clears the Gate only **between** scenarios (`gate_guard`), so FAULT→MANUAL was refused with `GATE_NOT_NORMAL`. In the r12 baseline the same dropout fell between scenarios and was cleared, giving 17/0. This latch is unchanged r12 logic. **The cause is the environment and harness, not r13.** |
| phase3_r13 | r13 | 18/0/0 | – (r12 18/0) |
| gate_reset_r13 | r13 | 13/0/0 | – (r12 13/0) |
| p4_server_first_r13 / p4_supervisor_first_r13 | r13 | 8/0/0, 8/0/0 | – |
| p5_r13_a / b / c | r13 | 5/3/0, 8/0/0, 8/0/0 | v04: the second visitor's goal A lies behind the robot at B. RPP (`use_rotate_to_heading: false`) drove a forward arc at 0.10 m/s, wz up to −0.5, toward the obstacle at map (−1.58, 0.82). The robot stopped **0.16–0.20 m** from the occupied cell, which is inside robot_radius 0.18 m (fixed map). Then: planner "failed to create plan" 14×, Spin and BackUp "Collision Ahead", `LOCALIZATION_CONTEXT_CHANGED` safety cancel and FAULT. v05/v07 cascaded. This is D3, and **its cause is now identified**. |

**Velocity re-output after resume, Gazebo d01–d04 `gate_nonzero_after_resume`, same harness:**

| | d01 0.35 s | d02 0.5 s | d03 2 s | d04 Gate+Watchdog 2 s |
|---|---|---|---|---|
| r12 (gz_r12_c) | 2 (first output vx 0.1) | 1 (0.1) | 1 (0.1) | 1 (0.1) |
| r13 (gz_r13_b) | **0** (0.0) | **0** | **0** | **0** |

This reproduces Part 2: r12 1/0/1 and 1/1/1, r13 0/0/0.
Other results:
- d05/d06 were 0 with both Gates.
- f03 (Watchdog kill): r13 rebound to the new writer after 2.3 s, once the old DDS endpoint cleared at 16 s.
- f04 (Gate 0.35 s stall): r13 first outputs ramp up from 0.008, with no step to the old 0.1.
- f07 (cancel, bt_navigator frozen): PASS with r13 (gz_r13_b) and FAIL with r12 (gz_r12_c), but the r12 run had already failed at UNKNOWN_AREA, so this is not a Gate comparison.

The r13 diff (`candidate_r13/r13_vs_r12.patch`, 7 hunks) does not touch the ICP, sensor-timeout or stall logic.

## Q4. Readiness verdict

- **r13 as an integration candidate: ready for owner review (SIM_ONLY).** Evidence:
  - the isolated rig: 0 STALE_REUSE in 36 valid zero/silent cases (r12: 20 of 35 valid; corrected 2026-11-22 15:10, an earlier version said 23);
  - Gazebo d01–d04: 0 re-outputs (r12: 5);
  - Phase 1/3/Gate Reset/Phase 4: equal to r12;
  - every FAIL in this campaign traced to a non-Gate cause (D2, D3, map UNKNOWN_AREA, sim scan dropout), with the Gate responding correctly.
- **Not ready for the formal Gate or the real LIMO.** The formal Gate is a different lineage (30 KB vs 133 KB) with an owner's uncommitted change, and it was not handed over. Real-robot timing (DDS latency, hardware writer restart) has not been measured. Applying r13 to the formal Gate is a separate step that needs the owner's approval.
- **Open blocking conditions for full Gazebo regression PASS:** D2 (out-of-plane furniture), D3 (RPP arc into obstacles), D1 (ICP rotation), and a map coverage issue (UNKNOWN_AREA initial pose). Details and fix candidates are in `GAZEBO_OPEN_DEFECTS_20261122.md`.

## Q5. Test record protection

`fleet_control/tests/run_guard.py`:
- `claim` creates exactly one new directory (atomic mkdir, refused if it exists) and writes MANIFEST.json in `x` mode;
- `finish` writes FINISH.json once;
- `check` re-verifies hashes.

`run.sh` uses `set -o noclobber` and puts the harness OUT and the console inside the claimed directory.
Harness edits: `GATE_SCRIPT` is now selectable in gate_reset_e2e, phase4 and phase5, and `GATE_WRITER_STATUS` in manual_e2e. These are environment variables only, and the r12 default is unchanged.
The console logs lost earlier in the day (`regress_after/INCIDENT_20261122.txt`) are **not** recovered.

## Q6. Reproduction (Codex)

```
source /opt/ros/humble/setup.bash; source capstone_ws/install/setup.bash
python3 fleet_control/tests/test_arbiter_generation_20261122.py
python3 fleet_control/tests/nav2_smoother_tail_probe_20261122.py --out <new file>.json
cd local_data/raw_results/gate_r13_integ_claude_20261122
./run.sh r12 rig <new name> zero,silent,live,swap      # and: ./run.sh r13 rig <new name> zero,silent,live,swap
./run.sh r13 gz <new name> ALL                          # r12: ./run.sh r12 gz <new name> ALL
./run.sh r13 phase1|phase2|phase3|gate_reset|p5 <new name>
./run.sh r13 p4 <new name> --order server_first
python3 ../../../fleet_control/tests/run_guard.py check <run dir>
```
`run.sh` sources a local Nav2 overlay (`OVERLAY=`, machine-specific): adjust it per machine. A repeated `<new name>` is refused.

---

# Part 4 (2026-11-22 evening, Claude, SIM_ONLY): Nav2 D3 improvement and D1/D2 separation

Results: `local_data/raw_results/nav2_d3_claude_20261122/`. Each run is run_guard-protected, with a MANIFEST and FINISH, run on domains 200–204 and ports 28981–28984. The shared Nav2 profile `capstone_ws/src/limo_capstone/config/integrated_nav2_lidar_only_slow_params.yaml` is committed by the project owner and used by the real launch. **It is not modified.** Candidates are applied as `NAV2_PARAMS_OVERLAY` (a later parameter file) in the isolated stacks only. The formal Gate, capstone_ws, Gate r12/r13 and the server/agent sources are not modified. The Gate used throughout is the r13 candidate `gate_r13_integ_claude_20261122/candidate_r13/stall_detector_node.py`, as in Part 3. **The Arbiter is mock/sim only; nothing here applies the velocity-generation contract to the real drive path.**

## R1. D3 targeted scenarios in Gazebo (`fleet_control/tests/gazebo_web/nav2_d3_targeted.py`)

Same conditions for every profile:
- each case is a fresh isolated stack, spawned at the case start pose, in a derived copy of the Office world with a 0.66 m test corridor added (the original world file is read only);
- velocity path: Nav2 → velocity_smoother → `/cmd_vel_raw` → Gate r13 → `/cmd_vel_locked` → drive relay → Gazebo;
- localization FIXTURE: odom = Gazebo truth, map→odom identity, rolling global costmap without a static map (`overlays/d3_targeted_truth_localization.yaml`), which keeps D1 (ICP) and map quality out of the comparison;
- recorded per case: Nav2 result, truth trajectory, minimum footprint clearance from the LiDAR (LiDAR sees visual geometry only), commands at `/fleet/cmd_vel_nav`, `/cmd_vel_raw` and `/cmd_vel_locked`, Gate state, blocked time (commanded with truth speed 0), in-place rotation time and Nav2 log events;
- Collision Monitor: **not deployed** in this stack (N/A).

| Case | Start → goal (spawn frame) | What it tests |
|---|---|---|
| behind_goal | (−0.30, −0.45) facing the cubicle wall → (−0.70, 0.90) | p5_r13_a v04 geometry: goal behind, wall ahead |
| corridor_uturn | (−1.60, 0.20) in the 0.66 m corridor → (−2.70, 0.20) behind | narrow in-place U-turn |
| wall_oblique | (−0.90, 0.30) → (0.60, −0.42), 0.43 m from the wall face | gz_r13_a geometry: approach a wall on a curve |
| adjacent_start | (0.20, −0.62), wall 0.11 m from the body side → (−0.70, 0.90) | start next to an obstacle |
| cancel_mid | straight 2.4 m goal, cancelled after 2.5 s of motion | goal cancel |

Profiles (overlay files in `nav2_d3_claude_20261122/overlays/`):
- **base**: shared profile.
- **O1**: `use_rotate_to_heading: true` (min angle 0.785).
- **O2**: O1 + `use_collision_detection: true`.
- **O6**: O1 + `progress_checker: nav2_controller::PoseProgressChecker` (`required_movement_angle` 0.5).
- **O7**: O6 + cost-regulated slow-down (`inflation_cost_scaling_factor` 7.5).
- **O8**: O6 with min angle 0.35.

Full table: `targeted_summary.md`. Key rows (successes / valid runs; min clearance while moving, median):

| Case | base | O1 | O2 | **O6** | O7 | O8 |
|---|---|---|---|---|---|---|
| behind_goal | **0/5**, 0.0 m: wall contact, Gate STALL_LATCHED / ICP DEGRADED | 5/5, 0.33 m | 1/2, 0.33 m | **3/3, 0.34 m** | 2/2 | 3/3, 0.53 m |
| corridor_uturn | 0/5, contact | 0/5, contact | 0/3 (aborts) | 0/3, contact | 0/3, contact | 0/3, no contact at the LiDAR plane (0.09 m), stall latch |
| wall_oblique | 4/4, 0.34 m | 4/4 | **0/3** (aborts) | 3/3, 0.34 m | 3/3 | 3/3 |
| adjacent_start | 5/5, 0.11 m (drives along the wall) | 5/5, 0.43 m | 1/3 | 3/3, 0.43 m | 3/3 | 2/2, 0.14 m |
| cancel_mid: last non-zero locked output after cancel | 0.13–0.17 s | 0.06–0.15 s | 0.06–0.07 s | 0.10–0.13 s | 0.10–0.14 s | – |
| progress failures (sum) | 77 | 56 | 30 | **12** (corridor only) | 9 | 6 |

Excluded as SKIP (harness or environment, not a profile result):
- 6 NOT_READY: the known Nav2 lifecycle bring-up flake "failed to send response to /controller_server/change_state (timeout)";
- 1 GOAL_REJECTED: the goal was sent before the lifecycle was active, fixed in the harness after `tg_base_a` by waiting for `lifecycle_manager_navigation/is_active`.

Findings:
- **D3 is reproduced in Gazebo with the base profile.** `behind_goal` drives straight into the wall (0/5).
- **`use_rotate_to_heading` fixes it.** O1/O6: 5/5 and 3/3, with clearance while moving 0.33–0.34 m. LIMO's differential/skid drive can rotate in place.
- **The shared `SimpleProgressChecker` (0.12 m in 5 s, translation only) aborts in-place turns.** A 180° turn at 0.3 rad/s takes 10.5 s, which causes "Failed to make progress", replanning and Spin. Humble's `PoseProgressChecker` (it also counts 0.5 rad) removes this everywhere except the corridor: O6 12 vs O1 56 progress failures.
- **RPP `use_collision_detection` (O2) is rejected.** Humble's projected-arc check aborted goals 0.43 m from a wall although the measured clearance stayed 0.34 m (wall_oblique 0/3, 243 events). It only sees the local costmap, so it cannot help D2.
- **Cost-regulated slow-down (O7) changed nothing measurable**, so it is not proposed.
- **`corridor_uturn` (0.66 m) is still unsolved.** During the 180° turn the 1 Hz replanning flips the path between going through the corridor and going around it, so the rotation reverses. Once within 45°, RPP drives a ≥ 0.1 m/s arc (radius about 0.29 m) into the wall. O8 (min angle 0.35) avoids contact at the LiDAR plane but still stalls. This is a corridor-width limit, robot_radius 0.18 m plus inflation, for this planner/controller pair; it is not solved by RPP parameters.
- **Cancel:** with every profile the locked output reached zero within 0.17 s and stayed zero. An earlier count of "non-zero after zero" in O2 was a measurement artefact (the cancel landed on a momentary zero, and the controller's last in-flight command followed). The settled metric is used now.

**Proposal for the Nav2 owner: O6** (`overlays/nav2_overlay_d3_o6_rotate_poseprogress.yaml`):
- `FollowPath.use_rotate_to_heading: true`, `rotate_to_heading_min_angle: 0.785`, `allow_reversing: false`;
- `progress_checker.plugin: nav2_controller::PoseProgressChecker`, `required_movement_radius: 0.12`, `required_movement_angle: 0.5`, `movement_time_allowance: 5.0`.

Not proposed: collision detection (O2) and cost slow-down (O7).

Real-robot notes, not verified:
- The real LIMO footprint (about 0.322 × 0.22 m, circumscribed radius ≈ 0.195 m) is larger than `robot_radius` 0.18 m, which matches the Gazebo model (about 0.29 × 0.23 m).
- `rotate_to_heading_angular_vel` 0.3 needs checking against the real skid-steer's minimum turning torque.

## R2. Gate reset refusals (gz_r13_a: 1376 refusals)

**Cause:** the test harness, not the product. `run_gazebo_web.py` `gate_guard` called `/stall_detector/reset_latch` every 1.5 s for up to 75 s, before each scenario and after homing. The Gate correctly refused every call (`REFUSED: SENSOR_FAULT:icp`) while the robot stayed pressed against a wall. The product path `agent/.../gate_reset_worker.py` calls the Gate at most once per admin request (durable journal, its own readiness checks first). There is no automatic repeat there.

**Change** (harness only, Claude-owned; original saved as `nav2_d3_claude_20261122/run_gazebo_web.py.pre_change`):
- `gate_guard` stops after `GZWEB_GUARD_PERSISTENT_MAX` (default 5) consecutive `SENSOR_FAULT` refusals and records `gave_up`;
- transient refusals (a killed writer's DDS ghost, `INPUT_WRITERS:2`) keep the full timeout;
- every scenario records a precondition snapshot (Gate state, latch, mode);
- `result.json` gets `failure_classes` (`first_failures`, `cascaded: {scenario: origin}`), and each FAIL is labelled `failure_class` first or cascade with `cascade_of`.

PASS/FAIL rules and Gate behaviour are unchanged; this is classification only.

## R3. D1: ICP odometry under pure in-place rotation (`fleet_control/tests/gazebo_web/d1_rotation_probe.py`)

Record:
- Gazebo, Gate in the loop, 2 turns each at ±0.3 and ±0.5 rad/s, at spawn A1 (cluttered office) and at an open spot 1.6 m north-west;
- `/scan`, truth, `/odometry/lidar_odom` and `/diagnostics/f4_latency`.

Replay: the node's own `scan_to_points` and `icp_2d` (imported read-only) re-run offline on the recorded scans with variants.

| | spawn A1 (±0.3 / ±0.5 rad/s) | open spot |
|---|---|---|
| live ICP vs truth | −4.3, −5.0 / −6.9, −7.0 % | −12.4, −12.9 / −12.8, −12.5 % |
| replay, node defaults | identical to live | identical to live |
| laser offset −0.103 or 0 | unchanged | unchanged |
| 50 iterations | unchanged | unchanged |
| keyframe every scan | −57, −59 / −40, −39 % | −54, −53 / −56, −56 % |
| correspondence 0.15 / 0.60 m | worse / slightly better | −25…−33 % / −11 % |
| keyframe 0.30 rad/m | −3.7 / −2.6, −3.2 % | −7.0, −6.5 / −6.9 % |
| no 85 % trimming | −4.2, −4.0 / −2.2, −2.3 % | −5.7, −6.5 / −3.3, −4.4 % |
| **keyframe 0.30 + no trimming** | **−2.4 / −1.3, −1.1 %** | **−2.9, −2.4 / −2.4, −2.6 %** |
| constant-velocity initial guess | −3.5, −4.0 / **+7.7, +8.7 %** (overshoot) | −4.0, −4.2 / +7.1 % |
| CV + keyframe 0.30 + no trimming | −2.3 / +1.4, +1.6 % | −2.1, −1.7 / 0.0, +0.1 % |
| wheel odom (`/odom`) | +64 % (Gazebo skid-steer model) | +64 % |

Cause classification:
- **Algorithm/parameters: yes.** Point-to-point nearest-neighbour ICP on 1° beams under-converges each match toward zero motion. The 85th-percentile trimming discards the correspondences that carry the rotation. The bias accumulates once per keyframe (every 0.12 rad/m), and it is larger with fewer or more distant features (open spot).
- **TF: no.** Offset sign and zero make no difference.
- **Time sync: no.** The replay uses scan stamps; the live and replay results match. f4 publish delay p50 10 ms, max 90 ms. Gazebo's gpu_lidar has no sweep skew, so de-skew is not the cause in simulation; on the real robot it would add.
- **ICP inliers** stayed 167–190, above the Gate's 100 threshold, during pure rotation.

Proposal for the ICP owner (`capstone_ws/.../lidar_scan_matcher_odom_node.py`, not modified):
- make the keyframe threshold (0.12 hard-coded) and the trim percentile (85 hard-coded) parameters;
- candidates: keyframe 0.30, no trimming;
- a constant-velocity prior only together with them.

Validate on translation and on the real robot first; only rotation was measured here.

## R4. D2: collision hull vs visual (read-only mesh analysis; depth detection is the parent session's work)

See `GAZEBO_OPEN_DEFECTS_20261122.md` D2 and `nav2_d3_claude_20261122/d2_collision_vs_visual.{py,json}`:
- In Gazebo the robot hits the **collision** mesh while the LiDAR and depth camera render the **visual** mesh.
- Office chairs (30) are a solid hull from the floor to 0.49 m over about 0.7 × 0.7 m; at the LiDAR plane the visual is a 4–8 cm column, and the legs lie under the scan plane.
- AdjTables (10) are a solid block to 0.81 m, while the LiDAR plane sees one rail.
- Depth detection, the depth costmap layer and the scan-clearing interaction are measured by the parent session (`fleet_control/docs/GAZEBO_DEPTH_20261122.md`).
- Structures no sensor sees need keep-out (Nav2 `KeepoutFilter`). This is a proposal and is not tested.

## R5. Full regression with O6 (stopped early at the parent session's request, 18:20; priority moved to the real LIMO)

Planned: Gazebo full suite 3× base + 3× O6, alternating; Phase 1–5 and Gate Reset with O6; Phase 5 5× O6 + 3× base.
Run: 2× base + 1× O6 Gazebo; Phase 1/2/3/Gate Reset; Phase 4 server_first ×2 (the first one invalid, the rerun 8/0). Three targeted records (`tg_base_b`, `tg_o1_a`, `tg_o7_a`) got their FINISH.json at wrap-up because their wrappers exited when queues were restarted. Their harnesses completed (SUMMARY.json present); rc is recorded as 0 or −1 (not observed) with a note in `CAMPAIGN.log`.
**Not run:** gz O6 ×2, gz base ×1, Phase 4 supervisor_first, Phase 5 (so the Phase 5 v04 effect of O6 is **not verified**), and the Phase 1 reruns.
Load: the parent session's Gazebo depth runs (domain 210) and my kinematic queue ran in parallel from about 17:45, at loadavg about 7–10. This is recorded in each MANIFEST and in `CAMPAIGN.log`.

| Run | Profile | PASS/FAIL/SKIP | First failures (cause); cascades |
|---|---|---|---|
| gz_base_a | base | 20/3/1 | g05 CANCELED (D1: ICP rotation under-read, AMCL DEGRADED, safety cancel); g06 refused `localization_not_confirmed` (cascade of g05: localization health is not in the harness's health snapshot, so the classifier labelled it 'first'); f03: Gate output 1.012 s after the Watchdog kill against a 1.0 s limit, but the timeline shows zero output from the kill to the end of the scenario (only the earlier homing turn was non-zero) and 0.019 m of motion, so this is a harness window measurement, not a re-output |
| gz_base_b | base | 21/2/1 | g05 CANCELED (D1); g06 (cascade of g05, localization) |
| gz_o6_a | O6 | 18/5/1 | g07 manual pointer `gate_nonzero_after_up` 2 (manual path; the Nav2 overlay is not involved; loadavg about 9; PASS in both base runs and in Part 3; load-timing item for Codex); g05 SAFETY_CANCEL `LOCALIZATION_CONTEXT_CHANGED` after an arc plus a 3 s in-place turn, AMCL heading error −0.09 → −0.30 rad over about 1.4 rad, ICP about −12 % (D1); g06 (cascade, localization); f06/f07 `initial pose rejected: UNKNOWN_AREA` (map coverage, as in gz_r12_c) |
| phase1_o6 | O6 | 18/1 | s09 `zero_after_s` null (no non-zero output in the window: the manual burst had already ended at the foreign goal; fake Nav2 servers are used, so the overlay is inert; Part 3 r13 19/0; reruns not done) |
| phase2_o6 / phase3_o6 / gate_reset_o6 | O6 | 17/0, 18/0, 13/0 | – |
| p4_server_first_o6 | O6 | 2/6 (**invalid**) | Nav2 never became active: lifecycle bring-up flake "failed to send response to /controller_server/change_state (timeout)" at loadavg about 10; PoseProgressChecker loaded correctly |
| p4_server_first_o6_b | O6 | **8/0** | rerun at lower load (loadavg about 5): equal to Part 3 r13 (8/0) |

The harness change (R2) worked as intended: no 75 s reset loops, and every FAIL has a precondition snapshot. The classifier only counts Gate/mode health, so localization-driven cascades (g06 after g05) must be read from `diag.loc`. Adding localization to the snapshot is left as a follow-up.

## R6. Verdict and open items

- **D3:** O6 fixes the measured D3 mechanism in Gazebo (behind_goal 0/5 → 3/3, clearance while moving 0.34 m) with no regression in the targeted cases.
  - In the full suite (1 O6 run against 2 base runs) the failures are dominated by D1 and by map coverage, so the full-suite and Phase 5 effect of O6 is **not verified**.
  - **Status: proposal for the Nav2 owner, SIM_ONLY.** The shared profile is unchanged.
- **D3 interaction with D1:** O6 increases pure in-place rotation, which is where D1 is worst (4–13 % under-read). Applying O6 without the D1 fix can shift failures from wall contact (base) to localization cancels (O6 g05). The safety response is correct in both cases. D1 should be fixed first or together with O6.
- **Open, Nav2:** corridor U-turn ≤ 0.66 m (planner path flip plus RPP arc); `robot_radius` 0.18 against the real footprint ≈ 0.195; the lifecycle change_state flake under load (affects every harness).
- **Open, ICP owner (D1):** make the keyframe threshold and trim percentile parameters; replay candidate keyframe 0.30 + no trim gives −1.1 to −2.9 %; then re-validate translation and real scans.
- **Open, D2:** depth detection (parent session's `GAZEBO_DEPTH_20261122.md`); keep-out for structures no sensor sees (proposal); Gazebo collision hulls ≠ visuals for chairs and AdjTables.
- **Gate r13 / safety contract:** unchanged in this part; the r13 isolated results and the formal-Gate question go to **Codex independent verification** (Part 3 Q4). The Arbiter remains mock/sim only; the real drive path does not have this contract.
- **Real robot:** nothing here is verified on the LIMO. No SSH, no 18190, no motor unlock, Mapping START still blocked.

## R7. Changed files and reproduction (Part 4)

- New: `fleet_control/tests/gazebo_web/nav2_d3_targeted.py`, `fleet_control/tests/gazebo_web/d1_rotation_probe.py`.
- Changed (Claude-owned harness): `fleet_control/tests/gazebo_web/run_gazebo_web.py` (`gate_guard` bound plus failure classes; original `nav2_d3_claude_20261122/run_gazebo_web.py.pre_change`).
- Results only (not repository code): `nav2_d3_claude_20261122/{overlays/, d3_run.sh, campaign_*.sh, aggregate_targeted.py, d2_collision_vs_visual.py, targeted_summary.md}`.

```
cd local_data/raw_results/nav2_d3_claude_20261122
./d3_run.sh base targeted <new> --repeat 3 --domain 200          # O6: ./d3_run.sh O6 targeted <new> --repeat 3 --domain 200
python3 aggregate_targeted.py <run dirs...>
./d3_run.sh base d1 <new> --domain 200 --w 0.3,0.5 --turns 2 --spawn 10.09,-6.98,0   # record + replay (replay.json)
python3 ../../../fleet_control/tests/gazebo_web/d1_rotation_probe.py replay --run <dir>/run --out <new>.json --variants node_default,kf_0.30_no_trim
./d3_run.sh O6 gz <new> ALL                                      # full Gazebo suite with the overlay
python3 d2_collision_vs_visual.py                                # D2 mesh table (needs ~/rmf_demos_humble models)
```
`d3_run.sh` sources a machine-specific explore overlay (`OVERLAY=`); adjust it per machine. Domains 200–209 are used.
