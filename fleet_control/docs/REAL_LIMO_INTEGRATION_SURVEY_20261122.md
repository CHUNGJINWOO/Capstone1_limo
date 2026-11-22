# Real LIMO Pro integration: survey and motor-locked readiness (2026-11-22, Claude)

Scope: connect the latest Fleet Control to the real LIMO **with the motors locked** and verify the web control path. Real driving, motor unlock and real Mapping START are out of scope.
The Gazebo D3/D1/D2 work is paused. Its results, uncommitted changes and logs are kept as they were:
- `local_data/raw_results/gate_r13_integ_claude_20261122/`
- `fleet_control/docs/GAZEBO_OPEN_DEFECTS_20261122.md`

Results directory: `local_data/raw_results/real_limo_integ_claude_20261122/`

Legend:
- **Verified**: measured today.
- **Record**: from an earlier dated record or a robot backup, not re-checked today.
- **Not verified**: not checked.

## 1. Access and site safety (2026-11-22 14:55 KST)

| Item | State | Evidence |
|---|---|---|
| LIMO `pcu` on Tailscale | **OFFLINE**, last seen about 1 day ago | Verified: `tailscale status` on the laptop |
| SSH / on-robot survey | **SKIP**: robot unreachable | – |
| Site safety: area clear, wheels off the floor or space secured, physical stop (power switch / E-Stop) within reach, person present | **Not verified**: only the user can confirm this | – |
| 18190 (field server) | Not running (no listener). Not started, not changed | Verified: `ss -ltn` |
| 8080 | No listener. Not touched | Verified |
| Other agents | The Codex desktop app is running. No working-tree file changed after 14:38 except this session's documents. The formal Gate, the real launch and others' uncommitted diffs are untouched | Verified: `find -newermt`, `git status` |

**Consequence:** nothing was sent to the robot. Every robot-side statement below comes from the 11-18 to 11-20 records and from the robot backups taken on 11-20 (`local_data/robot_backups_20261120/`, which contain tokens and must not be published). The robot may have changed since then.

## 2. Software on the real LIMO (Record: 11-20 backup and FIELD_LOCKED_READINESS_20261120.md)

| Layer | On the robot | Notes |
|---|---|---|
| ROS workspace | `/home/pcu/Capstone1_limo/robot_ws` (built on `/opt/ros/humble` only): limo_base, limo_capstone, limo_depth_obstacles, limo_scan_normalizer, ydlidar, astra_camera, explore, plus `limo_robot_agent` added on 11-20 | The older `/home/pcu/capstone_ws` and `~/real_auto_mapping_ready_20261113` still exist and are not used by robot_ws |
| Launch | `limo_integrated_auto_mapping_real.launch.py`, sha `b87e16d3…` = git HEAD launch + 11-12 scan normalizer. The laptop working tree (`1671d282…`, other owner's uncommitted change) is **not** on the robot | Record |
| Safety Gate | `stall_detector_node.py` sha `d103f445…` = **git HEAD** (29 KB lineage). **Not r11/r12/r13.** No input-writer binding, no writer-status check, no resume hold | Verified: hash of the backup file = `git show HEAD:` |
| Nav2 profile | `integrated_nav2_lidar_only_slow_params.yaml` `208d7f44…` = HEAD = working tree | Verified (hash) |
| Robot Agent | `limo_robot_agent` from the 11-20 13:41 deploy: the 19-file version plus `agent_node.py` and `map_save_worker.py`. Run with `mode=real allow_missions=false map_save_enabled=true` | Record |
| Motion Watchdog / Arbiter on the robot | The watchdog module exists in the old Agent package, but **the real launch does not start it** (patch 02b not applied). `velocity_arbiter.py` does not exist on the robot and is **mock/sim only by design** (`ValueError` unless mode=mock and simulated) | Verified: code. Record: launch |
| Base driver | `limo_base/limo_driver.cpp`: subscribes `/cmd_vel` and forwards each message to the STM32. **No host-side command timeout** | Verified: backup source |
| STM32 command timeout | Bench, no load: 0.10 m/s, command stops, wheels stop after 0.37–0.38 s (3 runs). Ground, turning and braking distance **not measured** | Record (CURRENT_STATUS §1) |
| DDS | Two environments exist. `env.sh`: `ROS_LOCALHOST_ONLY=0`, Discovery Server on the robot's own tailnet address `:11811`. `env_local77.sh` (11-20): domain 77, `ROS_LOCALHOST_ONLY=1`. The 11-20 locked runs used `env_local77.sh` | Verified (backup env.sh). Record |
| Motor lock | Launch argument `allow_motor_drive` (default false): Gate output `/cmd_vel_locked`; only the exact string `true` gives `/cmd_vel`. 11-20 runtime graph: `/cmd_vel` publishers 0, subscriber 1 (base), `[SAFETY LOCK ACTIVE]` | Record (runtime 11-20) |
| Autostart | No systemd or autostart unit was recorded on the robot. The Phase 4 supervisor/systemd work is SIM_ONLY | Not verified on the robot |

Real-only packages (not in Gazebo): limo_base (STM32 serial), ydlidar driver, astra_camera, limo_scan_normalizer (only in robot_ws, not in `capstone_ws/src`).
Sim-only packages and paths: Gazebo bridge and drive_timeout_relay, gz_truth_adapter, kinematic sim backend, the Agent VelocityArbiter (mock), Gate r12/r13 candidates, `web_mapping_isolated/stack.launch.py`.

## 3. Velocity path: simulation vs real

```
SIM (verified in Gazebo/isolated):
 Web -> Fleet Server -> Agent [VelocityArbiter, mock only] -> /fleet/nav_cmd_vel -> Motion Watchdog -> /cmd_vel_raw
     -> Gate r12/r13 -> /cmd_vel_locked -> drive_timeout_relay -> Gazebo
 Nav2 controller/behavior -> /fleet/cmd_vel_nav -> velocity_smoother -> Arbiter

REAL (robot_ws, as of 11-20):
 Nav2 controller -> velocity_smoother -> /cmd_vel_raw ; behavior_server -> /cmd_vel_raw      (no Arbiter, no Watchdog)
 /cmd_vel_raw -> Gate HEAD -> /cmd_vel_locked (allow_motor_drive:=false)  or  /cmd_vel (true) -> limo_base -> STM32
 Agent (real): status / lease / refusals only; manual bench output is 0; missions disabled
```

| Question | Answer |
|---|---|
| Does the real Agent use the latest Arbiter? | **No.** The Arbiter is mock/sim only and is not on the robot. The generation contract (cancel, new goal, mode change, lease, boot) and producer liveness are **not applied** on the real path. |
| Does real Nav2 output bypass the Arbiter? | Yes, by construction. Nav2 writes `/cmd_vel_raw` directly to the Gate, so no Arbiter or Watchdog stage exists on the robot. |
| Is the Gate r13 contract in the formal Gate or on the robot? | **No.** The robot runs Gate HEAD. r13 is a candidate in `local_data/…/candidate_r13/`, not reviewed by the owner. |
| Is there a separate `/cmd_vel` bypass publisher? | No publisher was seen in the locked runtime graph on 11-20. Possible bypasses (not prevented by any authentication): (1) any process on the robot that publishes `/cmd_vel` (vendor demos, teleop, `ros2 topic pub`); (2) with `env.sh` (`ROS_LOCALHOST_ONLY=0` + Discovery Server on the tailnet), **any tailnet host on domain 77 can publish `/cmd_vel`**; (3) launching with `allow_motor_drive:=true`. The base driver accepts the latest `/cmd_vel` from any writer. |
| What happens if the Gate stops or communication is lost? | The Gate stops publishing, and the driver has no timeout of its own. The STM32 stopped the wheels after 0.37–0.38 s on the bench (no load). A **frozen** Gate (SIGSTOP) that resumes could re-publish its last stored command: see §4. A laptop/server link loss does not affect the on-robot path (the Agent cannot command motion). |

## 4. Isolated check of the Gate the robot actually runs (HEAD `d103f445`)

`fleet_control/tests/gate_resume_stale_cmd_isolated.py` against the robot's Gate HEAD (isolated rig, private domains, no LIMO), kinds zero/silent/live/swap, 3 rounds, wall and sim clock.
Run: `rig_robot_gate_HEAD/`, guarded by run_guard (`FINISH.json` rc 0). During the run, another session changed `nav2_d3_targeted.py` and `run_gazebo_web.py`; the rig does not use them.

### 4.1 Result (Verified, SIM_ONLY rig model of the Gate boundary, not a real-LIMO measurement)

| Case | Robot Gate HEAD | For reference: r12 / r13 (same rig, 2026-11-22) |
|---|---|---|
| zero (upstream zero during the freeze) | **STALE_REUSE**: wall 0.2 s 3/3; sim 0.2 s 3/3, 0.5 s 2/3, 2.0 s 1/3 | r12 STALE_REUSE 11/18, r13 0/18 |
| silent (no fresh input) | **STALE_REUSE**: wall 0.2 s 3/3; sim 0.2 s 3/3; 1 NOT_ARMED (SKIP) | r12 9/17, r13 0/18 |
| live (normal resume) | OK 18/18 | OK 18/18 both |
| swap (writer killed, new writer, same node name) | **UNVERIFIED_WRITER_ACCEPTED 17/17** (1 NOT_ARMED SKIP). The fake robot moved about 0.47–0.48 m on the new writer before any reset | r12 RECOVERED 18/18, r13 RECOVERED 17 + 1 SKIP |

Total stale re-output: 15 of 35 valid (zero 9/18, silent 6/17; r12 20/35, r13 0/36) zero/silent cases. The re-output is short (in `zero_wall_0.2_r0`, one output of 0.08 m/s about 1 ms after resume; displacement 0.0 m), but it is the defect class r13 removes.
Writer acceptance: the robot's Gate has no input-writer binding, so **any** `/cmd_vel_raw` writer on the robot graph is forwarded.
Meaning for the real robot: the Gate that runs on the LIMO today does **not** satisfy the previous-command or writer contract. Until the owner reviews r13 (or an equivalent) and it is deployed and verified on the robot, the motor lock (`allow_motor_drive:=false`) is the only protection. That supports keeping all motion blocked.

## 5. Minimum changes needed for a motor-locked web integration (proposal, nothing applied)

| # | Change | Where | Owner / approval |
|---|---|---|---|
| 1 | Redeploy the current `limo_robot_agent` (real mode, `allow_missions=false`), after a tar backup of the robot's package and state | robot `robot_ws/src/limo_robot_agent` only | User approval. The Agent is ours. 7 changed + 8 new modules since 11-20 13:41 |
| 2 | Start with `env_local77.sh` (`ROS_LOCALHOST_ONLY=1`), never `env.sh`, so no tailnet host can join the robot graph | robot shell | No file change |
| 3 | Use an **isolated test server** on a new port (not 18190) with a copy of the B1/C103 Registry and a fresh token set. Bind it to loopback plus the laptop's tailnet address | laptop `fleet_control/scripts/run_field_server.sh <copy> <port>` | User approval for the port and tokens. 18190 is untouched |
| 4 | Robot stack exactly as on 11-20: `ros2 launch limo_capstone limo_integrated_auto_mapping_real.launch.py allow_motor_drive:=false …` (robot's own launch) | robot | Site safety confirmation first. This opens `/dev/ttylimo` |
| 5 | Not needed for locked web checks: Watchdog 02b/03b/04, Gate r13, Arbiter on the real path. These are **required before any real motion** (§7) | – | Gate/launch owners |

## 6. Motor-locked test plan (to run only after §1 is satisfied)

Each step needs a pass before the next. The lock must be **independently** shown before any web command is sent:
- `/cmd_vel` publishers = 0 and subscriber = 1 (the base), measured on the robot;
- Gate output = `/cmd_vel_locked`;
- wheels physically off the floor or observed by the user;
- `/limo_status` and `/odom` show zero velocity during every command.

1. Read-only robot survey (SSH, no process started or killed):
   - `ps`, `ros2 node list` (if a graph exists);
   - `/cmd_vel` publisher/subscriber counts with `ros2 topic info -v /cmd_vel`;
   - deployed file hashes versus §2;
   - `systemctl --user list-units` and `systemctl list-units | grep -i limo`;
   - `crontab -l`, `~/.config/autostart`;
   - USB (`ls -l /dev/ttylimo /dev/ydlidar`, `dmesg | tail`), disk, and `env_local77.sh` present.
2. Approval point: isolated server port and data copy. Approval point: Agent redeploy (backup first).
3. Approval point: site safety. Then start the locked stack (§5 #4) and the Agent.
4. Probe: `python3 fleet_control/tests/real_locked_probe.py --url http://127.0.0.1:<port> --robot pcu --token-file <viewer> --operator-token-file <operator> --refusal --out <run>/probe.json`. This covers:
   - online and state freshness;
   - LiDAR, Depth, TF/pose, Gate and Nav2 status;
   - lease acquire and release;
   - Mapping START refused (409);
   - map-click goal refused (409 `safety_not_ready`);
   - motor path LOCKED.
5. Added checks (each recorded with the robot-side `/cmd_vel` count and `/odom` speed):
   - unsafe command refused;
   - STOP accepted;
   - lease expiry;
   - server restart, then the old session and command are not reused;
   - Agent restart, then the old command is not reused;
   - manual bench output stays 0.
6. Stop in the documented order (FIELD_LOCKED_READINESS §6.4). Copy logs and bags to `local_data/raw_results/limo_runs/<run_id>/` with SHA checks, keeping the robot originals.

Navigation and Mapping: request refusal and readiness only. No goal and no Mapping START is executed.

## 7. Blocking conditions before any real low-speed driving (all still open)

1. The previous-command re-output contract on the **real** path:
   - the Gate actually on the robot is HEAD (§4);
   - r13 is not reviewed;
   - the Arbiter does not exist on the real path;
   - the Watchdog is not in the real launch.
2. The real Arbiter/Watchdog/Gate chain is not deployed or verified.
3. STM32 timeout under load, ground braking distance at 0.15 m/s, and Gate SIGKILL/SIGSTOP stop time are not measured.
4. The physical E-Stop is not tested.
5. Communication loss: the stop is measured only on the bench.
6. Sensor and localization:
   - no producer for localization CONFIRMED;
   - wheel odometry dropouts and the USB hub re-enumeration (11-18 to 11-20) are unresolved;
   - the LiDAR driver timeout seen on 11-20 is unresolved;
   - the Depth camera TF correction is per launch only.
7. DDS exposure: `env.sh` allows tailnet writers on `/cmd_vel`.
8. Site safety and a separate user approval.

## 8. Items for Codex independent verification

1. Gate HEAD on the robot: reproduce §4 (stale re-output after resume, writer swap) with the same rig, and review whether r13 or an equivalent fix should go into the formal Gate (owner review).
2. Check that no real-mode code path instantiates the Arbiter or publishes velocity: `agent_node.py` real branch, manual bench real output 0.
3. Review the `/cmd_vel` bypass list (§3) against the robot's runtime graph once it is online, including the DDS environment used.
4. Review `real_locked_probe.py` freshness rules: an offline robot's stale values must not count as PASS.
5. STM32 timeout and E-Stop test procedure (bench and ground), written before any motion test.

## 9. Real stationary Depth test of chair base structures (prepared, **SKIP**: robot offline)

Reuses the 2026-11-20 read-only scripts unchanged (`local_data/raw_results/limo_runs/20261120_stationary_measure_01/`):
- `stationary_measure.py` (floor plane, camera height and tilt);
- `low_obstacle_check.py` (raw Depth cloud in `base_footprint`, `/depth_obstacles/points`, local costmap, LiDAR).

Robot stationary and motor-locked stack as in §6. No velocity command.

Conditions:
- the §6 lock evidence (`/cmd_vel` publishers 0);
- the camera TF correction re-applied at launch (`camera_pitch`, `camera_roll`, `base_footprint_z` from §18 of FIELD_LOCKED_READINESS, re-measured first, because it is lost on every restart);
- the user places each object and measures its distance with a tape.

| # | Object, placed by the user in front of the camera | Distances | Expected sensor visibility (to be measured) |
|---|---|---|---|
| R1 | Office chair, star base facing the robot (legs and casters about 2–10 cm high, column in the LiDAR plane) | 0.5, 1.0, 1.5 m | LiDAR: column only. Depth: legs (band 0.025–0.35 m above the floor after the correction) |
| R2 | Chair or stool with low crossbars between the legs | 0.5, 1.0 m | LiDAR: legs. Depth: bars |
| R3 | Desk with a low stretcher (or a horizontal bar about 6–10 cm high) | 0.5, 1.0 m | LiDAR: legs. Depth: stretcher |
| R4 | Low box, height about 8–10 cm (below the LiDAR plane of about 0.134 m) | 0.5, 1.0, 1.5 m | LiDAR: none. Depth: yes |
| R5 | Glass door or wall, if one is available on site | 0.5, 1.0 m | Unknown. Real glass may pass or reflect both LiDAR and Depth, so this is recorded as measured, never assumed |

Per object and distance, record:
- raw Depth points inside the object's area;
- filter points;
- local costmap LETHAL cells at the object;
- LiDAR points;
- depth_status.

PASS means **filter + costmap** both show the object. A raw cloud alone is not a PASS. Avoidance cannot be tested on the real robot (motion is blocked), so the real result is detection only.
Run command (on the robot, stack running): `python3 low_obstacle_check.py <new run dir>/R<k>_<d>m.json`. Copy the results to `local_data/raw_results/limo_runs/<new run_id>/` with SHA checks.

## 10. Re-check and integration plan v2 (2026-11-22 18:25 KST, after the Gazebo Depth campaign)

### 10.1 Access (Verified 18:25)
| Item | State |
|---|---|
| `pcu` on Tailscale | **OFFLINE**, last seen 2 days ago; `tailscale ping` gets no reply. The LAN was not scanned |
| 18190 / 8080 | No listener; not started |
| Real-robot steps (§6, §9) | **SKIP**; nothing was sent to the robot |
| Gazebo | The only Gazebo still running (`office_limo.world`, port 28981) belongs to the Nav2 D3 session. It was asked to finish its queued run and stop. Its processes were not touched |

### 10.2 Real chain, confirmed from code (laptop Agent tree = deploy candidate; robot = 11-20 Record)
```
Web -> Fleet Server -> Agent (mode=real)
   real: status, lease, refusals; missions off (allow_missions=false)
   VelocityArbiter: cannot start in real mode (velocity_arbiter.py:71 raises ValueError unless mode=mock and simulated)
   ManualWebBench (only if manual_web_bench:=true): publishes zero Twist only, on /cmd_vel_raw
   -> [no Arbiter] -> [no Motion Watchdog in the real launch]
Nav2 velocity_smoother / behavior_server -> /cmd_vel_raw -> Gate HEAD d103f445 (no writer binding)
   -> /cmd_vel_locked (allow_motor_drive:=false) | /cmd_vel -> limo_base -> STM32
```

New observation: with `manual_web_bench:=true`, the real Agent becomes a **second `/cmd_vel_raw` writer** next to Nav2. It publishes zero only, so under the lock it is harmless. But Gate HEAD has no writer binding, so a zero from the Agent and a Nav2 command can interleave. This is another reason no motion is allowed before a reviewed writer contract (r13 or equivalent) is on the robot.

Depth on the real launch (HEAD = robot):
- `use_depth_camera` defaults to `false`.
- With `true`, Nav2 uses `integrated_nav2_params.yaml`: normal speed 0.32 m/s, **shared** `obstacle_layer` with the `depth_obstacles` source. This is the layout that let LiDAR raytrace clear low obstacles in Gazebo (GAZEBO_DEPTH §5.3).
- The HEAD launch shuts down on Depth + `slow`.
- The working-tree launch (another owner, uncommitted) adds `locked_depth_slow_params` and `manual_mapping_probe`. It is **not** on the robot.

### 10.3 Candidate changes: owner, verification state, decision for the locked phase
| Candidate | Owner | Verified | Decision now |
|---|---|---|---|
| Gate r13 | Gate owner (formal Gate in `capstone_ws`, 8 uncommitted lines) | Rig and Gazebo only (SIM_ONLY) | **Not applied.** Not needed for locked checks; required before motion (§7.1) |
| Depth separate `depth_layer` | Depth profile owner (untracked `integrated_nav2_depth_slow_params.yaml`) | Gazebo only (10/10 vs shared 7/10) | **Not applied.** A stationary locked test is not affected by motion clearing. Required before motion |
| ICP / D1 improvements | Nav2 / localization owners (D3 session) | Pending that session's final report | **Not applied** |
| Real Arbiter / Watchdog on the real path | Fleet Agent (ours), launch owner | Arbiter is mock-only by design; Watchdog 02b not in the real launch | Needs a reviewed real-mode design. Not in the locked phase |
| Agent redeploy (current tree) | Ours | Real-mode policy unit tests 2/2 OK, Phase 4 supervisor 7/7 OK (laptop, 18:2x). `test_agent_safety_v1` 19 errors: `rclpy.init()` not called in its fixture when run alone (test setup, not judged) | Candidate prepared, **not deployed** (approval point) |

Deploy candidate: `local_data/raw_results/real_limo_integ_claude_20261122/deploy_candidate_agent/` (32 files, `FILES.sha256`, tar sha `48dc1894…`). No literal secrets in the files; tokens stay in token files on the robot.

### 10.4 What counts as an independent motor lock
`allow_motor_drive:=false` is a software lock inside the same ROS graph that a bypass writer would use, so on its own it is **not** independent. Command injection (web STOP, lease, mode and bench tests) is allowed only when **all** of these hold:

| # | Layer | Evidence required before and during the test |
|---|---|---|
| L1 | Graph isolation | Robot shell sourced `env_local77.sh` (`ROS_LOCALHOST_ONLY=1`, domain 77): printed env recorded. `env.sh` never sourced in that shell |
| L2 | No motor writer | `ros2 topic info -v /cmd_vel`: publishers 0, subscriber 1 (`limo_base`), sampled before each command and at the end. The Agent's `motor_path` reports LOCKED |
| L3 | Gate output | Gate log `[SAFETY LOCK ACTIVE]`, Gate publisher on `/cmd_vel_locked` only |
| L4 | Physical | Wheels off the floor on a stand, or the user watching with the power switch within reach. **User confirmation only** |
| L5 | Motion observed | `/odom` and `/limo_status` linear and angular speed zero throughout. Any non-zero value aborts the test |

If any layer cannot be shown, the result is SKIP and no command is injected.

### 10.5 Locked integration test matrix (each item: robot `/cmd_vel` count, `/odom` speed, server and Agent log excerpt)
| Task item | Procedure (§6 step) | PASS criterion |
|---|---|---|
| Bypass check | L1–L3 before the first command | Exactly as §10.4 |
| Wheels stay still | L5 throughout | Zero speed in every sample |
| Robot ONLINE and sensors | `real_locked_probe.py` (§6.4) | Online and fresh; LiDAR, Depth, TF and Gate fresh; stale values from an offline robot never count |
| Lease, mode, command ACK | Probe lease acquire/release; mode change; STOP | ACK with the matching command_id; refusals 409 for Mapping START and goals |
| STOP, session expiry, link loss | §6.5: STOP, lease expiry, server restart, Agent restart | Old session and command not reused; Agent reports the link loss; motor path stays LOCKED |
| Depth detection and costmap | §9 R1–R5, stationary | Filter points **and** local LETHAL cells at the object; raw cloud alone is not a PASS |
| Gate FAILSAFE and recovery | Approval point: stop the Depth driver process on the robot while locked (owner's process; only with approval) | Gate `FAILSAFE_SENSOR_TIMEOUT`; recovery only under its documented conditions; no velocity on `/cmd_vel` |

Approval points, in order:
1. The robot is online, and the user confirms site safety (L4).
2. Isolated server port and data copy (not 18190).
3. Agent redeploy after a tar backup of the robot's package.
4. The Depth-driver stop test.

### 10.6 Still blocking real driving (unchanged from §7, plus)
- The Agent bench is a second `/cmd_vel_raw` writer on a Gate without writer binding (§10.2).
- The real Depth profile uses the shared layer that Gazebo showed clearing low obstacles.
- The Depth-slow profile exists only in another owner's uncommitted launch.

### 10.7 Read-only robot survey (Verified 2026-11-22 18:31 KST, after the robot joined Wi-Fi)
SSH used keys only. No process was started or stopped. One temporary hash list in the robot's `/tmp` was removed after copying. Evidence: `local_data/raw_results/real_limo_integ_claude_20261122/robot_survey_1831/`.

| Item | Result |
|---|---|
| Access | `pcu` online (Tailscale ping 95 ms); uptime 3 h 39 m; Ubuntu 22.04.5; `/opt/ros/humble`; disk 76 GB free; 5.7 GB RAM available |
| Running ROS / LIMO processes | **None**: no ros2, limo_base, LiDAR, camera, Gate, Nav2 or Agent process |
| Autostart | None for LIMO, ROS, fleet or the Agent (systemd user and system, crontab, `~/.config/autostart`) |
| `/cmd_vel` path now | No ROS graph exists, so there is no writer. The base driver is not running, so nothing holds `/dev/ttylimo` open |
| USB | `/dev/ydlidar → ttyUSB0`, `/dev/ttylimo → ttyUSB1` (2× CP210x); Orbbec depth sensor `2bc5:060e` and Orbbec USB camera `2bc5:050e` present |
| DDS env files | `env.sh`: domain 77, `ROS_LOCALHOST_ONLY=0`, Discovery Server `<ROBOT_TAILNET_IP>:11811` (bypass risk, §3). `env_local77.sh`: domain 77, `ROS_LOCALHOST_ONLY=1`, Discovery Server unset. The SSH login shell sets no ROS variables |
| Launch / Gate / Nav2 hashes | `b87e16d3…` / `d103f445…` (Gate HEAD) / `208d7f44…` (LiDAR slow). Depth profile `integrated_nav2_params.yaml` `cad6681c…`. All match §2. The installed `stall_detector_node` is the 979-byte entry wrapper from 11-19 |
| Agent on the robot vs deploy candidate | 13 modules identical; 7 changed (`agent_node`, `core`, `manual_web_bench`, `mapping_worker`, `motion_watchdog`, `ros_adapter`, `safety_protocol`); 9 only in the candidate (`action_status_evidence`, `frontier_policy`, `gate_reset_worker`, `localization_worker`, `ros_graph`, `supervisor`, `topics`, `velocity_arbiter`, `setup.py`). Same as the 11-20 13:41 Record |

Not yet measured, because each needs the stack running (approval points in §10.5):
- the sensor topics;
- TF;
- the runtime `/cmd_vel` publisher and subscriber counts;
- the Gate lock message;
- the web ONLINE state.
