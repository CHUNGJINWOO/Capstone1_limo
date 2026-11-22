# Real LIMO Pro: reuse of prior validation and Fleet integration plan (2026-11-22, Claude)

Priority change: the sensors, TF, odometry and SLAM Toolbox were already used on the real robot during the earlier auto-mapping work. Sensor-only tests are **not repeated**. This document lists what is reused, what is new, which files change, and what blocks motion.
Nothing was started, stopped or changed on the robot for this document. The robot survey of 18:31 (read-only SSH) is in `REAL_LIMO_INTEGRATION_SURVEY_20261122.md` §10.7.

Legend: **Reuse** = earlier real-robot evidence stays valid because the configuration is unchanged (checked below). **New** = no evidence for the current configuration. **Out of scope** = needs motion or a separate approval.

## 1. Is the earlier evidence still valid for today's robot? (Verified 18:31)

| Component on the robot today | Hash | Same as the record the earlier runs used? |
|---|---|---|
| `limo_integrated_auto_mapping_real.launch.py` | `b87e16d3…` | Yes (11-20 record) |
| Gate `stall_detector_node.py` | `d103f445…` (git HEAD) | Yes |
| Nav2 LiDAR slow profile | `208d7f44…` | Yes |
| Nav2 Depth profile (`integrated_nav2_params.yaml`) | `cad6681c…` | Not compared with an earlier hash today |
| Depth filter (installed) | `ba01c612…` (11-18 deploy) | Yes |
| Agent package | 11-20 13:41 version (20 modules) | Yes |
| udev `/dev/ttylimo`, `/dev/ydlidar`, Orbbec 2bc5 devices | present | Yes (11-19 rules) |

So the robot has not changed since the 11-20 field work. Reuse is valid for that configuration. It does **not** cover anything run only on the laptop (§3).
Caveat: the robot may have been rebooted or changed in ways the hashes do not show. The base USB bridge (CP2102) had unresolved re-enumeration events on 11-18 and 11-20.

## 2. Reuse / new / out of scope

### 2.1 Reuse (do not repeat)
| Area | Earlier evidence (real robot unless stated) | Where |
|---|---|---|
| LiDAR | `/scan` 10 Hz, `/scan_slam` 244 beams, 11-19 and 11-20 locked runs | VALIDATION 11-19 `robot_ws`, `20261120_locked_agent_01` |
| Odometry / IMU / status | `/odom` 49.9–49.95 Hz, `/imu` 99.8–99.9 Hz, `/limo_status` 2 Hz, 12.7–13.0 V, 60 s windows without gaps (11-18 run_004) | VALIDATION 11-18 CP2102 section |
| TF, Depth driver and filter | Depth TF fix and 300 s observation (error state 0.64 % vs 79 % before); Depth 11 Hz, RGB 13.6 Hz; stationary whiteboard frame marked in the local costmap (run_012) | VALIDATION 11-18 Depth deploy, 11-13 run_012 |
| SLAM Toolbox mapping | run_010: 110 s at 0.15 m/s, map 1,080→30,430 cells, stopped by `FAILSAFE_ICP_DEGRADED`, maps saved. Locked runs with rosbag: `/map` 306×251 (11-19), field save 48×120 and posegraph rebuild with zero cell differences (11-18) | VALIDATION 11-13 run_010, 11-18, 11-19 |
| Map save, transfer, reload | `/tmp` save, `scp`, SHA-256 match, map_server reload (11-19); CDR-to-PGM check (11-18) | same |
| C103 maps | `c103_lounge_20261119_depthmem` (UNVERIFIED), registered **only** in the isolated Registry, not the field Registry; `20261120_h107_map` | VALIDATION 11-19 Web Mapping, `limo_runs/` |
| Web ONLINE, sensors, map, Gate, LOCKED display; Lease; Mapping START and goal refusals | Probe 17/0 (Depth off) and 13/0 (Depth on) against the robot's 11-20 Agent | VALIDATION 11-20 |
| Gate behaviour under sensor loss | `wheel_odom_stale` → `FAILSAFE_SENSOR_TIMEOUT` observed repeatedly; locked output all zero; `/cmd_vel` publishers 0 | VALIDATION 11-18, 11-19 |
| Base stream stops | 11-18 CP2102 failures, reboot recovery, hub re-enumeration: root cause **not** established | VALIDATION 11-18 |

### 2.2 New (no evidence for the current configuration)
| # | Item | Why it is new |
|---|---|---|
| N1 | The Agent deploy candidate (7 changed, 9 new modules) in **real mode**, all new features off | Robot runs the 11-20 version |
| N2 | Command ACK for web STOP, session expiry (Lease), server restart and Agent restart on the real Agent | 11-20 covered Lease, START/goal refusals; STOP/expiry/restart ACKs were only Gazebo or isolated |
| N3 | Link loss: server and Tailscale cut while the robot runs locked | Measured only isolated (server loss 4.4 s, Lease expiry 3.3 s) |
| N4 | Depth detection of **low structures** in front of the stationary robot (chair star base, crossbars, desk stretcher, low box), filter **and** local costmap | Earlier real Depth test was one whiteboard frame; low objects were never placed |
| N5 | Real Depth failure → Gate `FAILSAFE_SENSOR_TIMEOUT` and recovery rules | 300 s observation saw errors, no deliberate Depth stop test while the Gate is watched |
| N6 | Dedicated isolated Fleet Server (not 18190) with the real Agent | Earlier runs used 18190 |
| N7 | Independent proof of the motor lock (§5) | The earlier lock evidence was one layer: `/cmd_vel` publisher count 0 |

### 2.3 Out of scope until separate approval
Any wheel motion, `allow_motor_drive:=true`, real Mapping START, goals, STM32 timeout under load, braking distance, E-Stop test, real Arbiter/Watchdog on the motion path.

## 3. Differences: robot (11-20) vs the latest Fleet code

### 3.1 Launch, Gate, Nav2, Depth (shared files with other owners: **not modified**)
| File | Robot | Latest laptop working tree | Owner / state |
|---|---|---|---|
| Real launch | = HEAD + normalizer | + `locked_depth_slow_params`, `manual_mapping_probe`, validation, 80 lines added | Launch owner, uncommitted |
| Gate | HEAD `d103f445` | HEAD + 8 lines: `max_abs_linear_x` cap | Gate owner, uncommitted |
| Gate r12 / r13 | not present | candidates in `local_data/raw_results/…/candidate_r13/` | Candidate, not reviewed |
| Depth Nav2 profile | `integrated_nav2_params.yaml` (shared `obstacle_layer`, 0.32 m/s) | adds untracked `integrated_nav2_depth_slow_params.yaml` | Depth profile owner |
| Depth filter | `ba01c612…` | same | Depth owner |

### 3.2 Agent: candidate vs robot (all new features default **off** in real mode)
| Module | Change size (diff lines) | Effect when the real launch is unchanged |
|---|---|---|
| `agent_node.py` | 63 | New parameters `manual_locked_arbiter`, `navigation_localization`, `web_gate_reset`, `map_save_enabled` (all default false). `manual_locked_arbiter` and `navigation_localization` `sys.exit` unless the Agent is mock/sim, isolated, localhost HTTP |
| `core.py` | 51 | Server/Agent contract changes. Needs a matching server (same tree), see N6 |
| `ros_adapter.py` | 131 | Graph/safety evidence |
| `mapping_worker.py` | 121 | Worker-side checks (web Mapping), real Mapping START stays refused |
| `motion_watchdog.py` | 146 | Process not started by the real launch |
| `manual_web_bench.py` | 29 | Only if `manual_web_bench:=true` |
| `safety_protocol.py` | 8 | Small |
| New: `velocity_arbiter.py`, `gate_reset_worker.py`, `localization_worker.py`, `supervisor.py`, `topics.py`, `ros_graph.py`, `action_status_evidence.py`, `frontier_policy.py`, `setup.py` | – | Imported lazily only when the matching parameter is on |
| Unchanged (13) | – | incl. `map_save_worker.py`, `real_locked_policy.py`, `motor_policy.py` |

Compatibility with existing operation:
- Map save: `map_save_worker.py` is byte-identical on the robot; the candidate keeps the real-mode rule that `map_save_enabled` requires `allow_missions:=false`.
- Locked run command: `start_robot_agent_field.sh` still starts the Agent with `allow_missions:=false`, domain 77, localhost only.
- Not yet checked: the server and the Agent must come from the same tree. The 11-20 robot Agent talks to the 11-20 server contract, so the redeploy must be tested against an isolated server of the latest tree first (N6).
- Laptop unit evidence: `test_real_locked_policy` 2/2, `test_phase4_supervisor` 7/7. `test_agent_safety_v1` errors only because its fixture does not call `rclpy.init()` when run alone.

## 4. Minimum integration design (proposal; nothing applied)

### 4.1 Stage A: locked web integration (now possible once site safety is confirmed)
- No change to the robot's launch, Gate, Nav2, drivers or ports.
- Replace only the Agent package with the candidate, features off. Backup first (tar + hash, the 11-20 procedure).
- Run through an **isolated** server (§6), not 18190.

### 4.2 Stage B: bringing the simulation safety chain to the real path (before any motion, needs owners)
Simulation chain: Web → Server → Agent Arbiter → `/fleet/nav_cmd_vel` → Watchdog → `/cmd_vel_raw` → Gate → `/cmd_vel_locked`.
Real chain today: Nav2 smoother / behavior_server → `/cmd_vel_raw` → Gate HEAD → base.

Minimum steps, in this order (the order from `proposals_20261120/README.md`, extended):
1. **Gate**: owner decides on r13 (previous-command and writer contract) plus 03b (watchdog requirement) and 04 (NaN) and the 8-line cap. Rig result with the robot's Gate HEAD: stale re-output 15 of 35 and an unverified writer accepted 17/17 (survey §4).
2. **Launch 02b**: Nav2 output moves to `/fleet/cmd_vel_nav`, the Watchdog is started, and `allow_motor_drive:=true` is refused without it. It needs the Agent package in the overlay. `start_explore` and `manual_mapping_probe` stay refused together with the Watchdog.
3. **Arbiter in real mode**: not a change in the Arbiter itself but a new, reviewed real-locked → real-armed mode. Today it is blocked by `velocity_arbiter.py:71` and the `agent_node.py` start guard. It needs: its own conditions for real hardware, the generation contract, producer liveness (already in the sim code), and the Watchdog as an independent process. Design review by the Agent owner (ours) and Codex before any code change.
4. **Depth separate layer**: the Depth profile owner moves `depth_obstacles` to a separate `depth_layer` (Gazebo: 10/10 vs 7/10 shared). Real clearing and persistence settings need a stationary and a later low-speed check.
5. **ICP (D1) with O6**: in-place rotation under-read of 4–13 % in simulation; real run_010 stopped with `FAILSAFE_ICP_DEGRADED`. Owner decision.

Owners and protection: none of these files is touched here. They are managed as integration candidates; every candidate needs the owner's sha256 baseline check and an isolated re-test, as the README requires.

## 5. Independent motor-lock verification setup (prepared; the lock is software unless L4 is met)

`allow_motor_drive:=false` only changes a topic name inside the robot's ROS graph. It is not independent. A command injection test starts only when all five layers are evidenced:

| Layer | Evidence | Who can provide it |
|---|---|---|
| L1 graph isolation | Robot shell uses `env_local77.sh` (domain 77, `ROS_LOCALHOST_ONLY=1`, no Discovery Server). `env | grep ROS_` printed and saved. `env.sh` is never sourced | Claude (read-only) |
| L2 no motor writer | `ros2 topic info -v /cmd_vel`: publishers 0, subscriber 1 (`limo_base`). Sampled before the first command, after every command, at the end | Claude |
| L3 Gate output | `/cmd_vel_locked` only, `[SAFETY LOCK ACTIVE]` in the Gate log | Claude |
| L4 physical | Wheels off the floor on a stand, or the user watching with the power switch within reach | **User only** |
| L5 observed stillness | `/odom` and `/limo_status` speed zero in every sample. Any non-zero value stops the test | Claude |

Extra safeguard to decide with the user: a **hardware or OS level** block on the base serial port for any process other than `limo_base` is not present. Not adding one in this phase.
If one layer cannot be shown, the result is SKIP and nothing is injected.

## 6. Isolated Fleet Server integration test plan (to be fixed here, executed after approvals)

| Step | Action | Needs |
|---|---|---|
| 0 | Site safety (L4) confirmed by the user | User |
| 1 | New data dir from a **copy** of the field Registry backup (`local_data/field_registry_backups_20261120`), new tokens via `fleet_server init`. New port outside 18190–18192 and 8080, e.g. 18290 (to be approved). Start: `fleet_control/scripts/run_field_server.sh <copy> 18290`. It binds 127.0.0.1 and the laptop's Tailscale address, `--require-supervisor-ready` always on | Approval for port and token set |
| 2 | Tar backup and hash of the robot's `limo_robot_agent`, install the candidate (`FILES.sha256`, tar sha `48dc1894…`). No other robot file changes | Approval |
| 3 | Robot: `. ~/Capstone1_limo/robot_ws/env_local77.sh`; `ros2 launch limo_capstone limo_integrated_auto_mapping_real.launch.py allow_motor_drive:=false use_depth_camera:=true scan_normalizer:=true` (the 11-20 command; opens `/dev/ttylimo`). Then L1–L3, L5 | User confirms L4 |
| 4 | Robot: `scripts/start_robot_agent_field.sh http://<laptop tailnet IP>:18290 <token file for the isolated server>` (`allow_missions:=false`). Readiness is judged by `scripts/dds_ready_check.py`, not by the process | – |
| 5 | `python3 fleet_control/tests/real_locked_probe.py --url … --robot pcu --token-file … --operator-token-file … --refusal --out <run>/probe.json` | – |
| 6 | Added checks, each with the robot-side `/cmd_vel` count and `/odom` speed: STOP ACK, Lease expiry, server restart (old session and command not reused), Agent restart, link cut (server stopped, Agent reports it), manual bench output 0 | – |
| 7 | Stationary low-structure Depth test R1–R5 (survey §9), reusing the 11-20 scripts; PASS = filter and local costmap both show the object | User places objects |
| 8 | Depth failure: stop only the Depth driver process (owner's process, approved); Gate must go `FAILSAFE_SENSOR_TIMEOUT` with zero velocity | Approval |
| 9 | Stop in the documented order, copy logs and bags to `local_data/raw_results/limo_runs/<new run_id>/` with SHA checks (real-robot data must also exist on the laptop); remove nothing on the robot except what this run created | – |

The Agent token for the isolated server is never printed or committed. The robot's existing 18190 token file is not reused.

## 7. Safety blocking conditions (unchanged; motion stays prohibited)
1. The robot's Gate (HEAD) fails the previous-command and writer contracts (rig: 15/35, 17/17). r13 or an equivalent is not reviewed, deployed or measured on the robot.
2. No Arbiter and no Watchdog on the real path; the real Arbiter mode does not exist.
3. STM32 timeout under load, braking distance at 0.15 m/s, Gate stop-time on SIGKILL/SIGSTOP: not measured. Physical E-Stop: not tested.
4. Base USB/UART and `wheel_odom_stale` interruptions: unresolved root cause (11-18, 11-19 runs 02, 11-20).
5. Localization on the real robot: no CONFIRMED producer; D1 ICP bias in simulation; `FAILSAFE_ICP_DEGRADED` in run_010.
6. DDS: `env.sh` lets any tailnet host write `/cmd_vel`. Only `env_local77.sh` is allowed in the test.
7. With `manual_web_bench` on, the Agent is a second `/cmd_vel_raw` writer next to Nav2 on a Gate without writer binding.
8. Real Depth profile uses the shared obstacle layer that Gazebo showed clearing low-obstacle marks.
9. Site safety and a separate user approval for every robot-side action.

## 8. Items for Codex independent verification
1. Real-mode start guards in the candidate: all four new features refuse or stay off in `mode=real`; no `/cmd_vel*` publisher is created by the Agent (grep and a graph test).
2. The Agent/server contract between the candidate Agent and the latest server (`core.py` changes) on an isolated graph, with the robot's 11-20 Agent as the negative control.
3. L1–L5 review: is the software lock plus L4 enough for a command injection test, or is a hardware block needed?
4. Gate HEAD rig reproduction and the r13 decision.
5. `real_locked_probe.py` freshness rules: stale values from an offline robot must not count as PASS.

## 9. Files that would change (all proposals, none applied)
| File | Change | Owner |
|---|---|---|
| Robot `robot_ws/src/limo_robot_agent/**` | Replace with the candidate (Stage A, after backup) | Agent (ours) |
| New data dir and tokens for the isolated server | New | Ours |
| `stall_detector_node.py` | r13 / 03b / 04 / cap | Gate owner |
| `limo_integrated_auto_mapping_real.launch.py` | 02b | Launch owner |
| `integrated_nav2_params.yaml` or the slow Depth profile | separate `depth_layer` | Depth profile owner |
| `velocity_arbiter.py`, `agent_node.py` | real-armed mode (design first) | Agent (ours) with Codex review |
| ICP node | keyframe/trim parameters | ICP owner |

## 10. Stage-A pre-deploy preparation, partial (2026-11-22 late, Claude; interrupted by the usage limit)
Done (code reading and port check only; **no test was run, no server was started, no data dir or token was created, nothing on the robot was touched**):
- Port 18290 is free on the laptop (also 18190, 18191, 18192 and 8080: no listener at the time).
- Server entry (`python3 -m fleet_server init|serve`): `init` writes new tokens (admin, operator, viewer, `agent-<robot>`) in a data dir and a secrets dir, mode 0600. `serve` has `--require-supervisor-ready`; `scripts/run_field_server.sh <data> <port>` always turns it on and refuses a port already in use and the operational field data dir without `ALLOW_FIELD_DATA=1`.
- Candidate Agent real-mode restrictions read from the code (not yet executed):
  - `manual_locked_arbiter`: `sys.exit` unless mock, simulated, localhost HTTP, isolated domain (not 77), no Discovery Server, port not in 8080/18190–18192, Watchdog required.
  - `navigation_localization`: `sys.exit` unless sim/mock simulated, localhost-only, domain not 77.
  - `map_save_enabled`: `sys.exit` unless `allow_missions:=false`.
  - `web_gate_reset`: **no real-mode guard found in `agent_node.py`** (only a comment "isolated sim only"). The worker's own checks were not read. Open item for Codex; keep it off in the real launch.
  - `manual_web_bench` (real): `_graph_blocker()` returns `velocity_source_conflict` if any other node writes `/cmd_vel_raw`, and requires exactly one Gate publisher/subscriber. The Agent publisher is created only when Gate is NORMAL, wheel odom fresh and `cmd_blocked` false. It publishes zero Twist only. Because the check is a graph snapshot, the second-writer condition stays a **separate blocking condition** (§7.7), not closed.

Not done (next session):
1. `run_guard claim` for a new run dir, then `fleet_server init` into a new data/secrets dir under it (loopback, `NO_TAILSCALE=1`, port 18290, `--require-supervisor-ready`).
2. Server/Agent contract run: candidate Agent in `mode=real`, `allow_missions=false`, isolated localhost domain (not 77), against the 18290 server; `real_locked_probe.py`. Negative control: the robot's 11-20 Agent copy (from `local_data/robot_backups_20261120/`).
3. Real-mode guard tests: each parameter above, expected refusals recorded.
4. Regression: map save worker and status snapshot with the 7 changed and 9 new modules.
5. Backup / SHA256 / rollback procedure for the robot package (tar of `robot_ws/src/limo_robot_agent` and its `install/`, hash list, restore commands, post-restore hash compare) written as a checklist; not executed.
6. L1–L5 are observation criteria, **not a sufficient independent block** of motor drive: state this explicitly in the approval request.

## 11. Isolated server and Agent pre-deploy verification (2026-11-22 19:3x, Claude, laptop only)
Run: `local_data/raw_results/real_limo_integ_claude_20261122/isolated_server_prep_01/` (MANIFEST and FINISH rc 0, 0 sources changed). Tokens are in its `secrets/` (0600, git-ignored `local_data`), never printed. 18190, 8080 and the robot were not touched. The robot's Agent files were not replaced.

| Check | Result | Evidence |
|---|---|---|
| Port 18290 | Free; isolated server started on 127.0.0.1 only (new data dir, new tokens, `--require-supervisor-ready`). Stopped afterwards; no listener left | `run/server.log` |
| Candidate Agent `mode=real`, `allow_missions=false`, `map_save_enabled`, isolated domain 214, no robot graph, against the latest server | Connected and reconciled; session protocol 2, contract 1, real mode. Probe (viewer): PASS 5 / FAIL 6 / SKIP 2. The FAILs are LiDAR, pose, map, Gate, motor path, Nav2 = `NO_DATA` because **no robot graph exists** (expected, fail-closed); movement not permitted | `run/agent_real.log`, `run/probe_viewer.json` |
| Operator probe `--refusal` | Lease acquired and released; web START refused 409 `mapping_real_blocked`; map-click goal refused 409 `robot_not_ready`. PASS 9 / FAIL 6 (same NO_DATA) / SKIP 2 | `run/probe_operator_refusal.json` |
| Agent velocity topics | Graph of the real-mode Agent: no `/cmd_vel`, no `/cmd_vel_raw` topic at all | `ros2 topic info` (domain 214) |
| `manual_locked_arbiter` in real | Refused, rc 1: "requires isolated mock, localhost HTTP and independent Watchdog" | `run/guards/arbiter.log` |
| `navigation_localization` in real | Refused, rc 1: "requires SIM/mock simulated mode…" | `run/guards/navloc.log` |
| `map_save_enabled` with `allow_missions:=true` | Refused, rc 1: "requires allow_missions:=false" | `run/guards/mapsave_missions.log` |
| **`web_gate_reset` in real** | **Not refused.** The Agent starts and connects. No explicit real-mode guard exists in `agent_node.py`. Protection is indirect: the worker needs the Gate's `reset_readiness` (r12 field); the robot's Gate HEAD has `/stall_detector/reset_latch` but no `reset_readiness`, so the worker would refuse. Not run with a Gate, so that refusal is **code-read, not measured** | `run/guards/gatereset.log` |
| `manual_web_bench` in real | Allowed to start (design: real, localhost-only, missions off). Publisher creation needs a Gate graph, so no publisher was created here (no Gate). The second-writer condition on a robot with Nav2 is **not tested** | `run/guards/bench.log` |
| Shutdown noise | SIGINT of the Agent prints a heartbeat `RCLError: publisher's context is invalid` traceback in both the gate-reset and bench runs. A shutdown race, not seen as a functional error | same logs |
| Regression (laptop, domain 215) | `test_map_save` 17/17, `test_map_save_worker` 20/20, `test_phase4_readiness` 10/10, `test_action_status_evidence` 7/7, `test_locked_graph_contract_characterization` 1/1, `test_manual_control_bench_contract` 5/5, `test_phase20r_lease_contract` 2/2, `test_r3_r4_contract` 2/2, `test_real_locked_policy` 2/2 (+ earlier `test_phase4_supervisor` 7/7) | `run/regress/` (three first attempts failed on import paths and are kept as `*_attempt1_importpath.log`) |
| Backup / SHA / rollback | Checklist written, not executed | `ROLLBACK_CHECKLIST.md` |

Not covered: STOP ACK, Lease expiry, server restart and link loss with a live robot graph (they need the sensors/Gate); status reports with real data (no graph here); Agent behaviour against the robot's real DDS slot problem (the warm-up participant workaround in `start_robot_agent_field.sh`).

### 11.1 Blocking conditions kept
- Second `/cmd_vel_raw` writer: `manual_web_bench` stays **off** in the real launch (blocking condition §7.7, not closed).
- `web_gate_reset` stays **off** in the real launch until Codex reviews the missing real-mode guard (and decides whether to add an explicit `sys.exit` in real mode: a change to our Agent code, proposed, not made).
- L1–L5 are **observation criteria**. They are not a sufficient independent block of motor drive: L2 and L3 read a graph that the same software controls, L5 detects motion only after it happens, and only L4 is physical and it is the user's. Every approval request says so.

### 11.2 Change scope and approvals needed for the real deploy
| Scope | Detail |
|---|---|
| Changed on the robot | `robot_ws/src/limo_robot_agent` (7 modules changed, 9 new) and its build. Nothing else |
| Not changed | launch, Gate, Nav2/Depth profiles, drivers, ports, udev, `env*.sh` |
| New on the laptop | isolated server data/tokens (done, local) |
| Approvals needed | (1) deploy window and site safety (L4) by the user; (2) robot backup file creation under `~/backups`; (3) Agent package replacement per `ROLLBACK_CHECKLIST.md`; (4) the isolated server's Tailscale bind (so the robot can reach 18290), with a token file copied to the robot; (5) launching the locked stack (separate approval, plan §6) |

## 12. Independent safety review before the real Agent deploy (2026-11-22, Claude as Agent owner)
Scope: Agent files only (`fleet_control/agent/limo_robot_agent/**`, `fleet_control/tests/`). Not touched: 18190, robot files, Gate, launch, Nav2, Depth, ICP, other agents' work. Records: `local_data/raw_results/real_limo_integ_claude_20261122/agent_fix_regress_01/` (run_guard MANIFEST; logs in `run/`).

### 12.1 `web_gate_reset` in real mode
Review result before the change: **not explicitly blocked** (§11). Real-mode behaviour if the flag were on, from code:
- the worker refuses unless the Gate reports `reset_readiness` (Gate HEAD does not), a fresh `MOTION_AUTHORIZED` Watchdog with the Agent's boot id (none in the real launch), a single Watchdog writer on `/cmd_vel_raw`, and `_graph_locked()` (always False without an Arbiter, which cannot exist in real mode). Four independent refusals, all indirect.

Change (agent_node.py, 4 lines): `web_gate_reset` now `sys.exit`s unless `mode` is `sim`/`mock`, `simulated` is true, `ROS_LOCALHOST_ONLY=1` and the domain is not unset/0/77. The Gazebo/isolated harnesses run it with the locked Arbiter, which already needs mock + isolated domain, so they are unaffected.

### 12.2 `manual_web_bench` second `/cmd_vel_raw` publisher
Before: in real mode the bench created a zero-Twist publisher on `/cmd_vel_raw` whenever one graph snapshot showed a clean path. In the real launch Nav2's smoother and behavior_server are writers of that topic, so the check normally blocked it (`velocity_source_conflict`, seen on 11-18). But the check is a snapshot at 20 Hz: before Nav2 is up, or after a Nav2 crash, the Agent could become the writer, and Gate HEAD has no writer binding.

Change (manual_web_bench.py): class constant `RAW_ZERO_PUBLISHER_IN_REAL = False`. In `mode=real` the bench never creates (and destroys any existing) `/cmd_vel_raw` publisher; its state shows `raw_publisher_blocked_real: true`. In mock/isolated modes behaviour is unchanged. Consequence: a real bench is observe-only; "Gate input stays zero" can no longer be asserted by this Agent. Re-enabling needs the Gate writer contract (r13 or equivalent) and a new review.
The §7.7 blocking condition changes from "open" to "closed in the Agent, still open for any other process that writes `/cmd_vel_raw`", because Gate HEAD still accepts any writer.

### 12.3 Inventory of velocity, Gate reset and Nav2 call paths in the Agent (code, all `agent/limo_robot_agent/**`)
| Path | Where | Real mode (`allow_missions=false`, launched by `start_robot_agent_field.sh`) |
|---|---|---|
| `/cmd_vel` publisher | none in any module | none |
| `/cmd_vel` subscriber | `slam_control.py` | needs `slam_sim_enabled` = sim only; not created |
| `/cmd_vel_raw` publisher | `manual_web_bench.py` (now blocked in real), `manual_mapping_input.py` (needs `manual_mapping_enabled` → `slam_sim_enabled` → sim only), `motion_watchdog.py` (separate process, not started by the real launch) | none after §12.2 |
| `/fleet/nav_cmd_vel` publisher | `velocity_arbiter.py` | refused in real (`velocity_arbiter.py:71`, `agent_node.py` guard; measured §11) |
| Gate reset call (`/stall_detector/reset_latch`) | `gate_reset_worker.py` | refused at start-up since §12.1 (measured below) |
| Nav2 goal | `ros_adapter.py` `send_goal_async` (missions and mapping), `ActionClient` is created in every mode | no goal without `allow_missions=true`; server also refuses (409 `robot_not_ready`, measured §11); `mapping_worker.py` mock/sim only |
| Nav2 cancel | `ros_adapter.py` (own goals); Watchdog `CancelGoal` client | Agent only cancels goals it owns; Watchdog not running |
| `/initialpose`, planner `set_parameters`, `/request_nomotion_update` | `localization_worker.py` | `navigation_localization` refused in real (measured §11) |
| slam_toolbox `serialize_map` | `map_save_worker.py`, `mapping_worker.py` | `map_save_enabled`: save only, requires `allow_missions=false` (measured §11) |
| Heartbeats | `/fleet/agent_heartbeat` (always), `/stall_detector/supervisor_heartbeat` (only `sim_r8_supervisor`, sim) | status topic only |

Residual finding: `agent_node.py` itself accepts `allow_missions:=true` in real mode. Only `start_robot_agent_field.sh` forces false. With Gate HEAD the movement permission is still always false (below), so a mission could not be started, but the node would not stop an operator who bypasses the script. Proposal, not implemented because it changes the future driving gate: refuse `allow_missions=true` in real mode unless an explicit approval environment variable is set.

### 12.4 Compatibility with the Gate HEAD that runs on the robot
Status JSON fields the candidate Agent evaluates:
| Field | Gate HEAD `d103f445` | Effect in the candidate |
|---|---|---|
| `state`, `cmd_blocked`, `wheel_odom_stale` | present | shown; `healthy` evaluated |
| `boot_id`, `fault_epoch`, `fault_latched`, `safety_state`, `reset_readiness` | **absent** (0 matches in the source) | movement permission always false (`GATE_BOOT_ID_MISSING`, `GATE_FAULT_EPOCH_INVALID`, `GATE_FAULT_LATCHED`); Gate-reset readiness unavailable |
| `require_motion_watchdog` / Watchdog status | absent / no Watchdog in launch | `watchdog.required=false`, state null |

So with Gate HEAD the candidate is **fail-closed by design**: it reports the robot online and the Gate state, and never permits movement. This is the wanted locked phase behaviour. It also means a real mission can never become permitted until a Gate that reports the epoch fields (r12/r13 or equivalent) is deployed. Server and Agent protocol are unchanged between the robot's 11-20 Agent and the candidate (`protocol_version=2`, `safety_contract_version=1`); the candidate adds server-sent `operating_mode`, `navigation_request`, `gate_reset_request` fields that are stored only; with no worker started in real mode nothing consumes them. The map-save path and its worker are byte-identical.

### 12.5 Tests (laptop; ROS domain 216-218; logs in `agent_fix_regress_01/run/`)
| Test | Result |
|---|---|
| New `fleet_control/tests/test_real_mode_agent_blocks_20261122.py` (5): real-mode bench creates 0 publishers, mock control still creates 1, `web_gate_reset` refused in real, refused on domain 77, not refused in isolated mock | 5/5 OK |
| Same file against the pre-change Agent (v1 tar) | 3 FAIL + 1 ERROR (the mock control errors on the missing new state key), as expected: the tests detect the old behaviour |
| Regression: manual bench tests (`locked_ros` 1, `safety_fix` 6, `delayed_agent` 1, `bench_contract` 5), map save 17 and 20, readiness 10, action status 7, locked-graph characterization 1, Lease contract 2, r3r4 2, real policy 2, Phase 4 supervisor 7, Arbiter generation 14, locked Arbiter 3 | all OK |
Two earlier attempts failed only on my own path mistakes (a relative log path, a `PYTHONPATH` that dropped ROS) and are kept as `*_attempt1_*` logs.

## 13. Deploy preparation (nothing executed on the robot; laptop files only)

### 13.1 Deploy candidate v2 (replaces v1 `deploy_candidate_agent/`)
`local_data/raw_results/real_limo_integ_claude_20261122/deploy_candidate_agent_v2/`: `FILES.txt`, `FILES.sha256` (32 files), `limo_robot_agent_candidate_v2.tar.gz`, `TAR.sha256` (`2fc8305e…`). Differences from v1: exactly two files, `agent_node.py` and `manual_web_bench.py`. The robot's Agent is the 11-20 version (13 modules identical, 7 changed, 9 new; §3.2). v1 is kept.
Robot Agent backup and SHA256 (to create at the deploy step, not now): `ROLLBACK_CHECKLIST.md` in `isolated_server_prep_01/` (src and install tars, per-file hash list, copy to `local_data/robot_backups_<date>/`, hash compare on both sides). The 11-20 backups (`local_data/robot_backups_20261120/predeploy_*`) are older and are not a substitute: they predate the 11-20 13:41 map-save deploy only partly, so take a new one.

### 13.2 Server and robot configuration comparison
| Item | Robot (11-20 record + 18:31 survey) | Isolated server 18290 (prepared) |
|---|---|---|
| Protocol / contract | 2 / 1 | 2 / 1 |
| Robot id | `pcu` | `pcu` (registered by `init --robots pcu`) |
| Agent flags | `mode=real allow_missions=false map_save_enabled=true`, domain 77, localhost-only (`env_local77.sh`) | same flags, tested on domain 214 |
| Server flags | field server 18190 (not touched) | `run_field_server.sh <data> 18290`: `--require-supervisor-ready` always on, 127.0.0.1 (+ Tailscale address only with `NO_TAILSCALE` unset) |
| Data / tokens | operational field data | new data dir and 4 new tokens inside the run directory (0600) |
| Registry / maps | field Registry with C103 | empty (no maps); map-registry checks are out of scope for the first connection |

### 13.3 18290 access scope and token protection (proposal; needs approval before anything is bound)
The server can bind only 127.0.0.1 or 127.0.0.1 + the laptop's Tailscale IPv4 (never 0.0.0.0). A robot on the tailnet can reach it only through the Tailscale bind, so the scope has to be limited in two places:
1. **Packet filter on the laptop** (needs root; commands prepared, not run): accept TCP 18290 on `tailscale0` only from `<ROBOT_TAILNET_IP>` (pcu), drop other sources. Verify afterwards from the laptop itself (loopback still works) and, if available, from another tailnet device that the port is closed. If a tailnet ACL is managed in the Tailscale admin console, the same restriction can be set there by the tailnet owner.
2. **Server side**: tokens are role-bound. The robot gets only `agent-pcu.token` (role `agent`, `robot_id=pcu`). Admin, operator and viewer tokens stay on the laptop. Operator/viewer probes run on the laptop over loopback.
3. **Token transfer**: one file, `scp` over the tailnet to `~/.config/limo_isolated_18290/agent-pcu.token` on the robot, mode 0600, directory 0700; never in a command line, log or document; compare the sha256 on both ends without printing the token. The 18190 token file on the robot is not reused or modified.
4. **Lifetime**: the isolated server runs only during the test window, is stopped afterwards, the packet-filter rule is removed, and the robot's token file is deleted at the end. Tokens can be rotated with `fleet_server init --force`.
5. Transport is HTTP inside the Tailscale tunnel (WireGuard); no TLS on top. That is the same as the 18190 setup.

### 13.4 What the real Agent is allowed to do in this step
Allowed: initial state report, Lease acquire/release, command ACKs (STOP, expiry, restart), map save status.
Blocked, and how:
| Function | Block | Evidence |
|---|---|---|
| Mapping START | server 409 `mapping_real_blocked`; Agent `web_mapping_enabled` refused in real | measured (§11) |
| Navigation goal | `allow_missions=false`; server 409 `robot_not_ready`; Gate HEAD lacks epoch fields so permission is always false | measured (§11), code (§12.4) |
| Manual | `manual_web_bench` not enabled; even if enabled, no `/cmd_vel_raw` publisher in real | measured (§12.5) |
| Gate reset | `web_gate_reset` refused at start-up in real; server also needs a Gate that reports `reset_readiness` | measured (§12.5) |

### 13.5 Approvals needed before any robot-side step (in order)
1. Site safety (L4) and a deploy window, from the user. L1–L5 remain observation criteria; they are not a sufficient independent block of motor drive (L2/L3 read a graph the same software controls, L5 detects motion afterwards).
2. Root on the laptop for the packet-filter rule (13.3-1), or a tailnet ACL change by the tailnet owner.
3. Creating backup files on the robot (`~/backups`) and copying them to the laptop.
4. Replacing `robot_ws/src/limo_robot_agent` with v2 and rebuilding only that package (checklist).
5. Copying `agent-pcu.token` to the robot.
6. Starting the isolated server with the Tailscale bind.
7. Launching the locked stack (separate approval, plan §6), then the Agent via `start_robot_agent_field.sh http://<laptop tailnet IP>:18290 <token> `.
Rollback: `ROLLBACK_CHECKLIST.md` (restore tars, per-file hash compare, rebuild only the Agent, 11-20 start command). At any hash mismatch or unexpected behaviour: stop the Agent, restore, stop the server, remove the packet-filter rule, delete the robot's token file.

## 14. Agent v3: configuration-error guards, loopback refusal checks, deploy candidate v3 (2026-11-22 late, Claude)
Records: `local_data/raw_results/real_limo_integ_claude_20261122/{agent_v3_guards_01,agent_v3_loopback_01,deploy_candidate_agent_v3,isolated_server_prep_01}/`. Laptop only. Robot, 18190, Gate, launch, Nav2, Depth, ICP, other agents' work: untouched. `run_guard` FINISH shows `sources_changed_during_run` 1 and 2: those are my own edits of the two test files while the runs were open; the Agent sources did not change after each run's claim.

### 14.1 Independent review of the two earlier guards (§12) and the bypass paths found
Reviewed by tracing every `mode`, `simulated`, `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` comparison in `agent_node.py`. Findings:
| # | Bypass path | Before | Now |
|---|---|---|---|
| B1 | `allow_missions:=true` accepted in `mode=real` (only the launch script forced false) | open (movement was still never permitted: server `robot_not_ready`, Gate HEAD fields, `GATE_ATTESTATION_UNAVAILABLE`) | `sys.exit` in real mode, any domain |
| B2 | A mistyped mode (`Real`, `REAL`, `foo`) skips every `mode == 'real'` guard because they compare exactly | open | `mode` must be exactly `real`, `sim`, `mock` (or `real_locked`, which `real_locked_policy` always refuses) |
| B3 | `mode=real` together with `simulated:=true` | accepted | refused |
| B4 | `mode=sim` or `mock` started on the robot's own graph (domain 77, or with the Discovery Server of `env.sh`): `web_mapping_enabled` only excluded domains unset/0, so a mapping worker could start against the real graph; a `sim` Agent can also treat a Gate publishing `/cmd_vel` as `SIM_GATED` | open | sim/mock refused when `ROS_DOMAIN_ID=77` or `ROS_DISCOVERY_SERVER` is set (the isolated fixtures already assert "not 77") |
| B5 | `ROS_DOMAIN_ID` unset or wrong for a real Agent | real Agent sees no robot graph: `NO_DATA`, fail-closed (measured §11) | unchanged |
| B6 | `ROS_LOCALHOST_ONLY=0` for a real Agent with `manual_web_bench` | refused by the existing guard | measured (test) |
Residual, not closed by code: an operator who starts a real Agent on a wrong but existing domain simply gets no data (safe); a wrong-domain Agent that is `mock` + `simulated` on a domain other than 77 can only see that domain's graph.
Second-writer guard (§12.2) reviewed: `raw_blocked` is evaluated every 50 ms tick before the publisher is created, so a publisher cannot appear between graph snapshots; an existing one is destroyed. Open point: a destroyed publisher can still have sent zeros while it existed (only in mock modes, by design).

### 14.2 Changes (Agent-owned files only)
| File | Change |
|---|---|
| `agent/limo_robot_agent/limo_robot_agent/agent_node.py` | 12 lines: exact mode check, real+simulated refused, real+`allow_missions` refused, sim/mock on domain 77 or with a Discovery Server refused (in addition to §12.1) |
| `agent/limo_robot_agent/limo_robot_agent/manual_web_bench.py` | unchanged since §12.2 |
| `tests/test_real_mode_agent_blocks_20261122.py` | 13 tests (8 new configuration-error tests) |
| `tests/real_agent_refusals_probe.py` | new: loopback Lease / MANUAL / manual bench / Mapping START / Goal / Gate reset / STOP / release probe |

### 14.3 Test results
| Test | Result |
|---|---|
| `test_real_mode_agent_blocks_20261122` (13: bench ×2, Gate reset guard ×3, configuration errors ×8) | 13/13 OK (`new_tests.log`) |
| Same file against the v2 Agent (before B1–B4) | 5 FAIL (odd mode, real+missions on every domain, real+simulated, sim/mock on 77, sim/mock with Discovery Server): the tests detect the old behaviour (`negative_control_v2_agent.log`) |
| Regression (same 15 modules as §12.5) | all OK |
| `test_terminal_agent_order` (2), `test_r8_reconcile_cancel` (1), `test_agent_safety_v1` (17) | errors in these fixture-based tests even with `rclpy.init()` (`'ActionClient' has no attribute 'sent_goals'`). The same counts (3 failures, 15 errors) appear with the v1 Agent from before today's edits (`test_agent_safety_v1_control_v1_agent.log`), so they are not caused by these changes; they are **not judged** and still need their harness (Codex item) |
Earlier failed attempts are kept as `*_attempt*` logs (odd-mode assertion, empty mode value, missing `rclpy.init`).

### 14.4 Real-mode Agent against the isolated server 18290 (loopback, domain 214, no robot graph)
`agent_v3_loopback_01/run/`:
| Check | Result |
|---|---|
| Online, `mode=real`, `simulated=false`, reconciled, `missions_enabled=false`, `gate_reset=false`, `manual_web_bench=false`, `movement_permitted=false` | PASS |
| Lease acquire / release ACK | PASS (200, epoch returned) |
| Lease expiry (10 s lease) | PASS: gone after 10.1 s (a 3 s request is rejected as invalid, kept as `lease_expiry_attempt1_ttl3_rejected.json`) |
| Mapping START | refused 409 `mapping_real_blocked` |
| Navigation goal (mission) | refused 409 `robot_not_ready` (supervisor not reporting, Gate NO_DATA) |
| Manual bench start / STOP | refused 409 `manual_bench_unavailable` ("fresh reconciled real bench Agent required") |
| Gate reset by admin | refused 409 `gate_reset_refused` (`AGENT_GATE_RESET_UNSUPPORTED`, lease not held by caller, operating mode unavailable, Watchdog not authorized, …) |
| Gate reset by operator | refused 403 (admin only) |
| `mode:=MANUAL` request | 400 `bad_mode` (my request used a mode name the server does not accept; it therefore does not prove the server refuses MANUAL for a real robot. The bench refusal above is the test that counts) |
| Agent stopped (SIGINT) | offline after about 12 s; restarted: online, reconciled, **new boot id**, `movement_permitted=false` |
| Server stopped | Agent logs `server unreachable` with 1, 2, 4, 8 s retries; server restarted: Agent logs `server restarted; reconnected`, new server boot reconciled |
Not tested: a lease held across the Agent or server restart (the lease was released before), and anything that needs real sensors.

### 14.5 Deploy candidate v3 and backup facts
`deploy_candidate_agent_v3/`: 32 files, `FILES.sha256`, `limo_robot_agent_candidate_v3.tar.gz`, `TAR.sha256` (`3bf1e88d…`). v3 differs from v2 in exactly one file, `agent_node.py`. The live source tree equals `FILES.sha256` (checked with `sha256sum -c`). v1 and v2 stay unchanged and intact. **Use v3, not v1/v2.** Rollback target and procedure: `ROLLBACK_CHECKLIST.md` (restore tars taken from the robot at deploy time; compare the per-file hash list).
Token non-exposure (checked): no token value of the four isolated tokens appears in any file under the run directories, `fleet_control/docs`, the common docs, or the scratchpad; token files and `auth.json` are mode 0600. No token was passed on a command line.

### 14.6 18290 scope
Prepared, not applied: `isolated_server_prep_01/NETWORK_SCOPE_18290_PROPOSED.md`. `ufw` is active on the laptop and root needs a password, so nothing was changed. The server binds only 127.0.0.1 and `<LAPTOP_TAILNET_IP>`. The ufw rule would allow only `<ROBOT_TAILNET_IP>` to port 18290 on `tailscale0`; "other devices are blocked" can be verified only if a second tailnet device is online (otherwise SKIP).

### 14.7 Real-robot communication result
| Item | Result |
|---|---|
| Robot ↔ 18290 ONLINE / state / Lease / ACK | **SKIP** (not deployed, not connected; waits for the approvals below) |
| Applied file hashes on the robot | none (no file was replaced) |
| Recoverability | the robot is unchanged; the deploy and rollback procedure and the v3 hashes are prepared |
| Mapping START / Goal / Manual / Gate Reset refusals with the real robot Agent | PASS on the laptop (§14.4); real-robot repeat is SKIP |

### 14.8 Approvals needed (in order; none was assumed)
1. Site safety (L4) and the deploy window.
2. Laptop network rule: the ufw rule in `NETWORK_SCOPE_18290_PROPOSED.md` (root; or the tailnet ACL change).
3. Robot backup files under `~/backups` and the copy to the laptop with SHA256 comparison.
4. Replace `robot_ws/src/limo_robot_agent` with v3 and rebuild only that package.
5. Copy `agent-pcu.token` (one file, 0600) to the robot.
6. Start the isolated server with the Tailscale bind; start the Agent via `start_robot_agent_field.sh http://<LAPTOP_TAILNET_IP>:18290 <token file>` (the existing 18190 start path is not used). The full ROS stack is **not** started for this first connection: the Agent alone reports `NO_DATA` sensors, which still verifies ONLINE, Lease and ACK; the locked stack stays a later, separate approval.
Remaining safety blockers (unchanged): Gate HEAD fails the stale-command and writer contracts, no Arbiter/Watchdog on the real path, STM32 timeout under load / braking distance / E-Stop unmeasured, base USB interruptions, localization, DDS exposure with `env.sh`, shared Depth layer. L1–L5 are observation criteria and not a sufficient independent block of motor drive.

## 15. Stage A execution, part 1: Agent v3 deployed on the robot; connection to 18290 stopped at the network step (2026-11-22 20:00–20:10, Claude)
Record: `local_data/raw_results/real_limo_integ_claude_20261122/stage_a_deploy_01/` (run_guard MANIFEST/FINISH; logs in `run/`). Laptop backups: `local_data/robot_backups_20261122/`.

| Step | Result | Evidence |
|---|---|---|
| 1 Robot recheck | PASS: Tailscale 21 ms, SSH keys only, up 5 h, no ROS/limo_base/robot_agent/launch process, nothing holds `/dev/ttylimo`; 18190 and 18290 have no listener | `run/` console, `robot_final_state.log` |
| 2 Backup | PASS: on the robot `~/backups/limo_robot_agent_{src,install}_pre_20261122_200059.tar.gz` (+ per-file list of the 24 source files and a tar hash file); copied to the laptop and compared: identical. src tar `3e1f2811…`, install tar `09c19bbb…` | `robot_backup.log`, `run/limo_robot_agent_pre_20261122_200059.tars.sha256` |
| 3 Deploy v3 | PASS: laptop v3 tar and live tree verified; staged in `~/stage_a_v3` (700), tar hash and all 32 extracted files compared on the robot; old source dir moved to `~/backups/limo_robot_agent_src_dir_replaced_20261122_200059` (outside `src`, so colcon sees one package); `colcon build --packages-select limo_robot_agent --symlink-install` finished in 0.7 s, rc 0. Robot `src` == `FILES.sha256` (32 files, `sha256sum -c`). `agent_node.py` `94ade245…`, `manual_web_bench.py` `0625ab3c…`. The installed code imports and shows `RAW_ZERO_PUBLISHER_IN_REAL=False` (v3 content). No other package changed (the build regenerated only the workspace `install/setup.*` helper scripts, as colcon always does) | `robot_stage_v3.log`, `robot_build.log`, `robot_hash_verify.log`, `robot_post_build_check.log` |
| 4 Limit 18290 to pcu (ufw) | **FAIL / stopped**: `ufw` is active but `sudo` needs a password (`sudo -n` refused). No firewall change was made. Per the instruction, the network-dependent steps were not attempted | this section |
| 5 Token install | **SKIP** (not installed; no token left the laptop) | – |
| 6 Isolated server on the tailnet, Agent alone on the robot | **SKIP** | – |
| 7 ONLINE, heartbeat, Lease, ACK, restart reconnects, refusals, `limo_base` not running / motor port unused | **SKIP on the robot** (nothing connected). `limo_base` not running and `/dev/ttylimo` unused were verified before and after the deploy | `robot_final_state.log` |
| Earlier failed attempt | The first deploy command aborted at its own safety check because `pgrep -f` matched its own command line; nothing was moved. Kept as `robot_build_attempt1_pgrep_selfmatch.log` | – |

State left on the robot (all inside the approval): `~/backups/` (4 backup files + the replaced source dir), `~/stage_a_v3/` (tar, hash files, empty `x/`), the new `robot_ws/src/limo_robot_agent` and its build. No process was started. The robot's Agent is v3 but not running; the previous Agent is recoverable from the backup (rollback checklist steps 7–8).

### 15.1 Options for the network step (needs the user's choice)
| Option | What changes | Notes |
|---|---|---|
| A. User runs the ufw rule on the laptop | `sudo ufw allow in on tailscale0 from <ROBOT_TAILNET_IP> to <LAPTOP_TAILNET_IP> port 18290 proto tcp` (details and removal in `NETWORK_SCOPE_18290_PROPOSED.md`) | the approved plan; the server then listens on the Tailscale address. "Other devices blocked" stays SKIP unless a second tailnet device is online |
| B. SSH reverse tunnel, no root, no tailnet listener | The server binds **127.0.0.1 only** on the laptop. From the laptop: `ssh -N -R 127.0.0.1:18290:127.0.0.1:18290 pcu@<ROBOT_TAILNET_IP>`. sshd on the robot has no `GatewayPorts` line (default no), so the port exists only on the robot's loopback while the tunnel runs. The Agent connects to `http://127.0.0.1:18290` | different from the approved ufw mechanism, so not started without approval. It exposes 18290 to nothing but the robot's own processes, and ends when the tunnel is closed |
| C. Tailnet ACL | owner of the tailnet allows only pcu → pulsar:18290 | no root needed; outside this session |

## 16. Stage A execution, part 2: real Agent v3 ↔ isolated server 18290 over an SSH reverse tunnel (2026-11-22 20:07–20:15, Claude)
Approved by the user (option B, two tool scripts copied, token 0600, Agent alone, teardown, backups kept). Record: `local_data/raw_results/real_limo_integ_claude_20261122/stage_a_connect_01/` (MANIFEST/FINISH; `run/` and `run/robot_logs/`). Not started or changed: full ROS launch, Gate, Nav2, `limo_base`, motor serial port, Arbiter, Watchdog, ICP, Depth, 18190, other agents' files. No v3 reinstall.

### 16.1 Setup (all verified)
| Item | Result |
|---|---|
| Server | `run_field_server.sh` with `NO_TAILSCALE=1`: listener **127.0.0.1:18290 only** (no other address, none on the Tailscale IP; checked again after every restart) |
| Tunnel | `ssh -N -o BatchMode=yes -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 -o ServerAliveCountMax=3 -R 127.0.0.1:18290:127.0.0.1:18290 pcu@<ROBOT_TAILNET_IP>` (a separate connection; the existing SSH sessions were not used or changed). Robot-side listener: **127.0.0.1:18290 only**, 0 non-loopback listeners; robot → server without a token answers 401 |
| Token | one file, created with `umask 077` from stdin (never on a command line), `~/.config/limo_isolated_18290/agent-pcu.token`, mode 600 in a 700 directory; sha256 equal to the laptop file (compared, not printed) |
| Tools | `~/limo_isolated_18290_tools/{start_robot_agent_field.sh,dds_slot_precheck.py}` (700), hashes equal to the repository files (`9a4e30ef…`, `f86d871a…`) |
| Script review before use | sets domain 77, `ROS_LOCALHOST_ONLY=1`, fastrtps (like `env_local77.sh`; never sources `env.sh`); starts only `ros2 run limo_robot_agent robot_agent` with `mode=real simulated=false allow_missions=false`, no extra flags; refuses to start if a `robot_agent` already runs (checked, 0); `STATE_DIR` set to a new directory, separate from the 11-20/18190 one; `dds_slot_precheck.py` ran as `NO_HEALTHY_SLOT_FOUND` (exit 4) because no stack runs, so `FORCE_START=1` was used as the script documents (Agent stays `NO_DATA`, fail-closed); a 30 s throw-away warm-up participant is started on the robot's domain 77 |
| Processes on the robot while connected | only `ros2 run` + `robot_agent`; 0 `limo_base`/launch/Gate/Nav2/LiDAR; nobody holds `/dev/ttylimo` or `/dev/ttyUSB1`; no `/cmd_vel` or `/cmd_vel_raw` topic exists in domain 77 (topic list shows only the Agent's subscriptions and `/fleet/agent_heartbeat`) |

### 16.2 Results
| Check | Result | Evidence |
|---|---|---|
| ONLINE, `robot_id=pcu`, `boot_id`, `host=pcu`, `ros_domain_id=77`, `mode=real`, `simulated=false`, `missions_enabled=false`, reconciled | PASS (first boot `0fe25a382d9f`) | `state_1_online.json` |
| Heartbeat / state freshness | PASS: `last_seen_age_s` and `state_age_s` 0.3–0.4 s in 3 samples | `heartbeat_samples.txt` |
| `movement_permitted` | false throughout | all state files |
| Lease acquire / release ACK | PASS (epochs 4, 5, 6) | `refusals_real_robot.log` |
| Lease expiry | PASS: 10 s lease gone after 10.1 s, Agent online throughout | `lease_expiry.json` |
| Command ACKs | PASS: Lease, mode and STOP requests answered with the right codes | `refusals_real_robot.log` |
| Mapping START | refused 409 `mapping_real_blocked` | PASS |
| Navigation goal | refused 409 (`robot_not_ready` in the first run: supervisor not reporting, Gate NO_DATA) | PASS |
| Manual | operating mode MANUAL refused 409 `mode_unsupported` ("agent has no manual input path"); bench start 409 `manual_bench_unavailable`; bench STOP 409 | PASS (the first probe attempt used the wrong key `mode` instead of `target` and got 400 `bad_mode`; kept as `refusals_real_robot_attempt1_modekey.*`, it proved nothing) |
| Gate reset | admin: refused 409 `gate_reset_refused` (`AGENT_GATE_RESET_UNSUPPORTED`, …); operator: 403 | PASS |
| Observed only | mode MAPPING 409 `mapping_real_blocked`, NAVIGATION 409 `robot_not_ready`; after these the server showed the operating mode as `TRANSITIONING` during the second probe run, so the bench/goal refusals in that run carry the reason `mode_not_manual`/`mode_not_navigation`; the first run's reasons are the direct ones | `refusals_real_robot.log` |
| Agent restart | PASS: offline about 12 s after the stop, restarted Agent online and reconciled with a **new boot id** (`24901029be46`), `movement_permitted=false`; the previous lease was already released, so "old lease not reused" is not tested | `state_2/3` |
| Server restart | PASS: Agent logs `server unreachable` with 2, 4, 8 s back-off, then `server restarted; reconnected`; new server boot reconciled | `server_restart_agent_log.txt`, `state_4` |
| Link loss (tunnel dropped) | PASS: server shows offline, Agent retries; tunnel restored: online and reconciled again | `state_5/6` |
| `limo_base` not running, motor serial port unused, no velocity topic | PASS (before, during, after) | `robot_motor_checks.log`, `teardown_robot.log` |
| **Graceful stop of the real Agent** | **FAIL (finding)**: SIGINT to `robot_agent` made rclpy shut down but the Agent kept running in its retry loop (`server unreachable (RCLError)`, every 1–10 s); SIGTERM was ignored as well; only SIGKILL ended it (state offline correctly). The 11-20 stop procedure (`pkill -INT -f "robot_agent --ros-args"`) would not stop this Agent. Not yet known whether the 11-20 Agent behaves the same (not tested: it cannot be run against the same robot id) | `robot_logs/agent_run1.out`, `agent_stop.log` |
The second Agent run was ended with SIGKILL directly (the first run had already shown that SIGINT/SIGTERM do not stop it).

### 16.3 Teardown and state left behind
Done: Agent killed (0 processes), tunnel closed, server stopped (no listener on 18290, 18190 had none and was not touched), robot token file deleted (the empty config directory remains), robot-side listener gone. Checked afterwards: the robot's `src/limo_robot_agent` is still v3 (`sha256sum -c`, 32 files); no file under the 18190-era state/config directories or any other `robot_ws` package changed.
Kept on the robot, not deleted or moved (as instructed):
| Path | Purpose |
|---|---|
| `~/backups/limo_robot_agent_{src,install}_pre_20261122_200059.tar.gz`, `.files.sha256`, `.tars.sha256` | Backup of the previous Agent (rollback source; copies on the laptop in `local_data/robot_backups_20261122/`) |
| `~/backups/limo_robot_agent_src_dir_replaced_20261122_200059/` | The previous source directory, moved out of `src` |
| `~/stage_a_v3/` (tar, `FILES.sha256`, `TAR.sha256`, empty `x/`) | Staging copy of the deployed v3 |
| `~/limo_isolated_18290_tools/` (2 scripts) | Temporary tools for the isolated 18290 test; to be integrated into the standard LIMO folder layout later (user decision) |
| `~/.local/state/limo_robot_agent_isolated_18290/` | State and logs of the two Stage A Agent runs |
| `~/.config/limo_isolated_18290/` | Now empty (token deleted) |
On the laptop: the isolated server data and tokens stay in `isolated_server_prep_01/` (local, 0600); the 18190 files were not used.

### 16.4 Remaining safety blockers and next steps
Blockers unchanged: Gate HEAD fails the stale-command and writer contracts; no Arbiter/Watchdog on the real path; STM32 timeout under load, braking distance, E-Stop unmeasured; base USB interruptions; localization; DDS exposure with `env.sh`; shared Depth layer; L1–L5 are observation criteria, not a sufficient independent block of motor drive.
Proposed next steps (each needs its own approval): (1) fix the Agent shutdown loop (our code) and add a test; (2) decide the standard place for the Stage A tools; (3) the locked ROS stack with the Agent through the same tunnel (Depth status, TF, Gate fields), per §6; (4) stationary Depth chair-base test (survey §9).

## 17. Agent graceful-stop fix (Work A) and LIMO directory audit (Work B) (2026-11-22 20:25–20:45, Claude)
Records: `local_data/raw_results/real_limo_integ_claude_20261122/{agent_shutdown_fix_01,limo_dir_audit_01,deploy_candidate_agent_v4}/`. `run_guard` FINISH shows `sources_changed_during_run` 2 for each: my own edits of `agent_node.py` and the test files while the runs were open. The robot's Agent v3 was **not** replaced; the robot was only read over SSH (audit). No ROS stack, motor control, 18190, Gate/Nav2/Depth/ICP, or other agents' files were touched.

### 17.1 Cause (Stage A finding F1)
Flow in `agent_node.py` before the fix: `rclpy.init()` installs C-level SIGINT/SIGTERM handlers that **only shut the ROS context down**. `agent.run()` (in the main thread) is a `while self.running` loop whose `except Exception` branch turns every error into "server unreachable (…); retry in N s". Observed (before-run, 12 cases, all FAIL):
| Condition | Behaviour |
|---|---|
| SIGINT with a normal disposition | exits in about 0.2 s with return code 1 **and a traceback**: the context is already shut down, so `robot.announce_shutdown()` (a heartbeat publish) raises `RCLError` at the top of the `finally` block and **skips the rest of the cleanup** (executor, node, workers) |
| SIGINT inherited as `SIG_IGN` (what `nohup … &`, `setsid … &` from a non-interactive shell give: this is how the robot Agent was started) | no `KeyboardInterrupt`; only the ROS context shuts down; the loop retries on `RCLError` forever → **hung until SIGKILL**; leftover fastrtps shared-memory files |
| SIGTERM | same hang |
Same result with the server up, never reachable, killed while connected, and during reconnect (4 scenarios × 3 signal variants, 0 PASS / 12 FAIL; `run_before/`, `before_console.log`). Threads involved: main (loop), `ex.spin` (daemon, 4-thread executor), heartbeat/timers inside the executor, worker threads. The retry loop never looked at the ROS context, and the shutdown order assumed a live context. (Mechanism inferred from the code and the 12 observations; the traceback and the hang are measured.)

### 17.2 Change (one file, `agent/limo_robot_agent/limo_robot_agent/agent_node.py`, +66/−12 lines vs v3)
- `_install_stop_handlers(agent)`: replaces the SIGINT/SIGTERM handlers after `rclpy.init()`; the first signal logs and calls `agent.stop()` (sets the loop flag and event), a second signal restores the default action; a watcher thread stops the Agent if the ROS context is shut down by anything else.
- `finally`: every cleanup step runs inside its own try/except (so one failure cannot skip the others), `announce_shutdown` only while the context is valid, executor join logged if it exceeds 5 s, final line `robot agent stopped`.
- Known bound: if a signal arrives while the loop is inside one HTTP call, the exit waits for that call (timeout ≤ 20 s for map uploads, shorter for state posts). Not seen in the matrix.
- Not changed: server protocol, Lease, ACK, heartbeat content, all real-mode guards (§12, §14), `core.py`.

### 17.3 Verification (laptop, isolated domains 228–232, loopback ports 18293–18296, no robot)
| Check | Result |
|---|---|
| Matrix after the fix, live tree (`run_after/`): up / down at start / server killed while connected / reconnect × (SIGINT normal, SIGINT ignored, SIGTERM) | **12/12 PASS**: exits by itself in about 0.2 s, rc 0, no traceback, no `shutdown: … failed` line, Agent node gone from `ros2 node list --no-daemon`, no new `/dev/shm` fastrtps files. An earlier run on a scratch copy (10/2) reported shm files in two cases while the before-run was executing on the same machine; the clean rerun on the live tree, run alone, has none |
| Lifecycle (`run_lifecycle/`): connect, heartbeat age 0.2–0.7 s, Lease probe 12/12 PASS, SIGTERM → log `signal 15 received: stopping` / `robot agent stopped`, server shows offline after about 6 s, restart → **new boot id**, Lease probe 12/12 PASS, SIGINT → clean exit | PASS (my first SIGTERM attempt used a wrong pid from `setsid` and hit nothing; the correct pid was used afterwards; the log of the mistake is in `run_lifecycle/`) |
| New `test_agent_shutdown_20261122.py` (4-case subset) | PASS; first attempt failed because DDS domain 233 is invalid (max 232), kept as `*_attempt1_domain233_invalid.log` |
| Regression after the change: real-mode blocks 13/13, manual bench ×4 modules 13, map save 17+20, readiness 10, action status 7, locked-graph 1, Lease 2, r3r4 2, real policy 2, Phase 4 supervisor 7, Arbiter generation 14, locked Arbiter 3 | all OK (`run_regress/`) |
Still not judged: fixture-based `test_agent_safety_v1`, `test_terminal_agent_order`, `test_r8_reconcile_cancel` (same errors as before the change).

### 17.4 Deploy candidate v4 (not deployed)
`deploy_candidate_agent_v4/`: 32 files, `FILES.sha256`, tar `97071d50…`. v4 differs from v3 (deployed on the robot, `agent_node.py` `94ade245…`) in one file: `agent_node.py` `6a8252da…`. Deploying it needs the same approvals as v3 (backup first: the v3 directory on the robot becomes the rollback target, `ROLLBACK_CHECKLIST.md`). Until then the robot's Agent must be stopped with SIGKILL after SIGINT/SIGTERM fail (Stage A procedure).

### 17.5 LIMO directory audit (Work B)
Full report: `LIMO_DIRECTORY_AUDIT_20261122.md` (structure, roles, path references, duplicates, must-keep list, laptop vs robot, proposed layout, draft rules for an agent working on the LIMO). Key results:
- one canonical workspace, `~/Capstone1_limo/robot_ws` (12 packages, not a git repo); `~/capstone_ws` and others are older and contain **divergent copies** of the launch, Gate, slow profile and depth filter (different hashes);
- `~/.bashrc` aliases point to the old workspace and include `stop` (publishes `/cmd_vel`), `base`, `nav2`, `auto_map`;
- `~/limo_field_agent_ws` and `~/capstone_experiment_ws` named in documents/scripts **do not exist**;
- backups are in three places; the 8.2 GB `~/.local/state/limo-real-auto-mapping` holds original rosbags, and **no laptop copy of the 11-13 runs (about 4.1 GB) was found by name**;
- no autostart of any kind; all Stage A paths are listed with purpose;
- proposal (nothing created or moved): register the existing Stage A directories in the common documents first; optional later consolidation under `~/Capstone1_limo/{backups,tools,staging}` after approval.
