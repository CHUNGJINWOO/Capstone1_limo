# LIMO structure standardisation and refactoring design (2026-11-22, Claude; investigation and design only)

Goal: keep every working function, record and real-robot evidence, and fix the structure so that the web control system, Open-RMF, the visitor guidance web and a future 3D digital twin can share one space model.
Nothing was changed on the robot or in the working tree for this document (read-only SSH, local reads, one throw-away signal test). No commit, no push, no new workspace, no move or delete.
Reused, not repeated: `LIMO_DIRECTORY_AUDIT_20261122.md` (robot directories), `ARCHITECTURE_AUDIT_20261120.md` (packages, nodes, duplicates), `REAL_LIMO_FLEET_INTEGRATION_PLAN_20261122.md` (§7 blockers, §14–§17).
Evidence: `local_data/raw_results/real_limo_integ_claude_20261122/refactor_design_01/run/` (robot_ws hash list, 11-13 run inventory, v4 review).
Legend: **Verified** = measured today. **Record** = from a dated document. **Inference** = my reading. **Proposal** = nothing applied.

## 1. Current structure (Verified unless marked)

### 1.1 Laptop repository (working tree `staging/fleet-control-on-progress-c804241`, HEAD `c804241a`)
| Item | Fact |
|---|---|
| Worktrees | one (`Capstone1_limo`); the other worktrees were released (Record: WORKTREES) |
| Tracked changes | 9 modified files, all other agents' or the owner's: `AGENTS.md`, `README.md`, real launch, `stall_detector_node.py` (+8 lines), `package.xml`, `run_recorder.py`, depth filter, `m-explore-ros2/explore/CMakeLists.txt`, `AUTONOMOUS_DRIVING_DESIGN.md` |
| Untracked | 413 entries: `fleet_control/`, the common documents (`START_HERE`, `WORKTREES`, `VALIDATION`, `HANDOFF`, `CODE_INDEX`, …), `experiments/*`, `docs/troubleshooting`, `fleet_frontier_candidates.cpp`, `integrated_nav2_depth_slow_params.yaml`, … |
| **`fleet_control/` is not under version control on this branch** | 646 files in the working tree, 0 tracked on this branch and on `main`. Branch `feat/fleet-control-unified` (`ed53d81b`) tracks 203 of them, an older snapshot: `agent` 11 tracked files vs 32 now (6 identical, 5 changed, 21 new), `server` 12 vs 42 (2/10/30), `tests` 8 vs 229 (3/5/221), `docs` 4 vs 145, `scripts` 4 vs 14; `dashboard`, `integrated_sim`, `gazebo` identical |
| Size | `experiments/` 34 GB (79 directories, one of them 29 GB: `safety_chain_fix_20261113`), `local_data/` 14 GB (11 GB `raw_results`), `capstone_ws` 1.5 GB (849 MB `build`) |
Consequence (**risk R-1, highest**): the Fleet Server, the Agent v3/v4, 221 tests and all of today's work exist only as untracked files and in `local_data`. A disk fault or a stray `git clean` would lose the Fleet control code. The backups in `local_data/robot_backups_*` and the `deploy_candidate_agent_v*` tars are the only other copies of the Agent.

### 1.2 Robot `~/Capstone1_limo/robot_ws/src` against the laptop (hash comparison of 342 files)
| Package | Robot files | Identical | Differs | Laptop counterpart |
|---|---|---|---|---|
| `limo_base`, `limo_description`, `limo_msgs`, `ydlidar_ros2_driver`, `explore`, `explore_lite_msgs`, `astra_camera_msgs`, `limo_depth_obstacles` | 22+35+6+20+18+3+11+7 | all | 0 | `capstone_ws/src/...` |
| `astra_camera` | 146 | 145 | 0 (1 robot-only file) | `capstone_ws/src/ros2_astra_camera/astra_camera` |
| `limo_capstone` | 34 | 31 | **3**: `launch/limo_integrated_auto_mapping_real.launch.py`, `limo_capstone/stall_detector_node.py`, `package.xml` | `capstone_ws/src/limo_capstone` (these three carry the owners' **uncommitted** changes; the robot has git HEAD + the scan-normalizer patch) |
| `limo_robot_agent` | 32 | 31 | **1**: `agent_node.py` (robot = v3, laptop = v4) | `fleet_control/agent/limo_robot_agent` |
| `limo_scan_normalizer` | 8 | 8 identical to **four** copies | – | no counterpart in `capstone_ws/src`; only inside `experiments/real_auto_mapping_ready_20261113`, `real_depth_slow*_20261113` (4 identical copies) |
So the robot is a faithful selection of the laptop sources, except the three owner-held files, the one Agent file, and the normalizer that lives only in experiment folders. `robot_ws` itself is not a git repository, so the laptop is the only source.

### 1.3 Running entry points (what is actually started, by evidence)
| Subsystem | Entry point | Used where (evidence) | State |
|---|---|---|---|
| Fleet Server | `python3 -m fleet_server serve` via `fleet_control/scripts/run_field_server.sh` (always `--require-supervisor-ready`); 51 REST routes in `http_api.py`; static UI `server/fleet_server/static/*` (operator `index.html`, visitor `visitor.html`) | 18190 field server (Record), 18290/loopback tests (today), 74 test files | in use |
| Map Registry | `map_registry.py` (3,532 lines, SQLite): facility → building → floor → map version (resolution, origin, yaw, `frame_id`, `rmf_compatibility_json`) → areas, active map per floor, robot map bindings | 18190 Registry; C103 map registered only in the isolated Registry (Record) | in use |
| Visitor web | `/visitor*`, `/guide/*`, `guide.py`, `visitor.html/js` | Gazebo/isolated Phase 5 (Record) | sim/isolated only; real blocked |
| Robot Agent | `robot_agent = limo_robot_agent.agent_node:main` (also `mock_robot`, `motion_watchdog` console scripts) | robot (v3) and tests | in use |
| Agent modules reachable in `mode=real` | `agent_node`, `core`, `ros_adapter`, `safety_protocol`, `map_save_worker`, `manual_web_bench` (observe-only), `real_locked_policy`, `motor_policy`, `topics`, `ros_graph`, `action_status_evidence`, `joystick_filter`, `pngmap` | refusals measured (§11, §14, §16) | in use |
| Agent modules refused in real | `velocity_arbiter`, `localization_worker`, `gate_reset_worker`, `mapping_worker` (+`frontier_policy`), `slam_control`, `manual_mapping_input`, `recording_*` (real recording refused) | start-up guards, measured | sim/mock only |
| Supervisor | `supervisor.py` + `scripts/autostart/limo-supervisor.service` | Phase 4 isolated/Gazebo; **not installed on the robot** (no unit, Verified) | sim only |
| Motion Watchdog | `motion_watchdog` process | isolated/Gazebo chain; **not in the real launch** (patch 02b not applied) | sim only |
| Arbiter | `velocity_arbiter.py` | mock/sim only by design (`ValueError` in real) | sim only |
| Safety Gate | `limo_capstone/stall_detector_node.py` (formal) | real launch (robot: HEAD `d103f445`), all sim stacks; candidates r11/r12/r13/patches 03b/04/05/06 are copies under `local_data` and `proposals_20261120/` | in use; r13 candidate not reviewed |
| Nav2 | profiles in `capstone_ws/src/limo_capstone/config`: real launch uses `integrated_nav2_params.yaml` (Depth) or `…_lidar_only[_slow]_params.yaml`; `…_depth_slow_params.yaml` is untracked and referenced by tests only | robot profile hashes match HEAD | in use / candidate |
| Mapping (real) | real launch with SLAM Toolbox + `explore_lite` (`start_explore`, direct goal submit) | 11-13, 11-18, 11-19 runs (Record) | in use (field, locked) |
| Mapping (web) | `server/mapping.py` + Agent `mapping_worker` + `fleet_frontier_candidates` (untracked C++) | isolated/Gazebo (Record); real START refused 409 | sim/isolated only |
| Open-RMF | `rmf_adapter/limo_rmf_adapter` (`coords.py` rigid 2D transform), `config/fleet*.json`, `rmf_map_shim` | Gazebo RMF checks (Record), `real_rmf_20261113` | sim only |
| Gazebo web harness | `tests/gazebo_web/*`, `tests/web_mapping_isolated/stack.launch.py` | all 11-21/22 Gazebo campaigns | test infrastructure |

### 1.4 Velocity and DDS facts (Verified in source and on the robot)
- The only source file that publishes `/cmd_vel` is the Gate (`stall_detector_node.py:174`). `/cmd_vel_raw` writers in source: `motion_watchdog` (sim chain), `manual_web_bench` (now blocked in real), `manual_mapping_input` (sim). In the real launch Nav2's `velocity_smoother` and `behavior_server` write `/cmd_vel_raw`.
- Operational writers outside the source: `~/.bashrc` alias `stop` (`ros2 topic pub --once /cmd_vel`), `base`, `nav2`, `auto_map`, `nav_test`; anything on a tailnet host while the robot uses `env.sh` (`ROS_LOCALHOST_ONLY=0`, Discovery Server); `limo_base` accepts the latest `/cmd_vel` from any writer.
- Gate HEAD (robot) has no input-writer binding and no stale-command hold (rig: 15/35 and 17/17, survey §4).

## 2. Findings from the special reviews
### 2.1 Agent v4 signal handling and cleanup (independent re-check)
Re-read the diff and ran extra adversarial cases beyond the 12-case matrix (`v4_review/`):
| Case | Result |
|---|---|
| server accepts the connection and never answers; SIGTERM | exit in 2.5 s, rc 0, `robot agent stopped` (bounded by the 4 s state timeout) |
| same with SIGINT inherited as `SIG_IGN` | exit in 2.5 s, rc 0 |
| two SIGTERM in 1 s while hung | second signal restores the default: exit in 1.0 s, rc −15 (killed), no stop line, no traceback |
Other checks: `supervisor.py` stops children with SIGINT → (grace) → SIGTERM (3 s) → SIGKILL (2 s) through the process group, so v3 (ignores both) was always killed by escalation, v4 exits on the first; `motion_watchdog` uses `rclpy.spin`, which returns when the context shuts down, so it does not have the Agent's retry-loop problem (not tested under `SIG_IGN`; add it to the matrix); every test starts the Agent as a subprocess or from the main thread (the handler requires the main thread); worst-case exit is the longest in-flight HTTP call (≤ 20 s for map upload) plus up to 5 s executor join.
Verdict: **no defect found in v4**; open items: extend the matrix to `motion_watchdog`, `mock_robot_node`, `supervisor`; keep the real-robot stop procedure at "SIGINT, then SIGKILL after 10 s" until v4 is deployed and measured on the robot.

### 2.2 Real chain: Gate HEAD, Arbiter and Watchdog are not integrated
Unchanged from survey §3/§7 and plan §12.4: robot path is Nav2 → `/cmd_vel_raw` → Gate HEAD → `/cmd_vel_locked` or `/cmd_vel`; no Arbiter, no Watchdog, no writer binding; the candidate Agent is fail-closed against Gate HEAD (no `boot_id`, `fault_epoch`, `fault_latched`, `reset_readiness`). Refactoring must not "fix" this implicitly: it is stage R4 below and belongs to the Gate/launch owners.

### 2.3 Data preservation
- Robot originals: `~/.local/state/limo-real-auto-mapping` 8.2 GB, `limo_locked_stack`, field Agent state, `~/maps_h107_20261120`, backups (audit §4).
- **11-13 runs**: 21 run directories, **5.3 GB**, 19 rosbags, on the robot; **no copy by name on the laptop** (`limo_runs/` starts with 11-10 smoke and 11-19; `local_data/rosbags` is empty; `experiments/real_limo_field_20261113` holds reports only). Laptop free space 666 GB. Copying is non-destructive and needs only read access to the robot, but it is not done here (design only).
- Proposed copy rule (R0): per run `rsync -a --checksum` into `local_data/raw_results/limo_runs/20261113_run_NNN/` (existing convention), manifest `sha256` created on the robot and compared on the laptop, rosbags checked with `ros2 bag info` and SQLite integrity, robot originals untouched; the same for `20261118_*` where no laptop copy exists.

### 2.4 Classification of duplicates and old versions (by use, not by existence)
Keep = used on the robot, by 18190, or by the current test/harness chain. Merge = same function in several places. Retire candidate = no reference from code, tests, scripts or launch (still needs the owner's approval; nothing deleted).
| Item | Evidence | Class |
|---|---|---|
| `fleet_server` (all modules), `static/` operator and visitor UI | field server + 74 tests | **Keep**; split later (R6) |
| Agent modules listed as "reachable in real" | measured | **Keep** |
| Agent sim/mock modules (`velocity_arbiter`, `localization_worker`, `gate_reset_worker`, `mapping_worker`, `frontier_policy`, `slam_control`, `manual_mapping_input`, `mock_robot_node`, `supervisor`) | referenced by 1–22 tests each; refused in real | **Keep, labelled SIM/MOCK** (they are the verified safety chain); package-level split in R5 |
| `frontier_fleet_coordinator.py` | imported by nothing, 1 test | Retire candidate (owner) |
| `recording_control`/`recording_session`, `integrated_recorder_node`, `scripts/run_recorder.py` | three recording paths (Record 11-20) | **Merge** into one |
| `slam_control` (sim) vs `mapping_worker` (web) vs real launch `explore` direct submit | three Mapping paths | **Merge** into the Worker as the single owner for web mapping; keep the real launch path until R4 |
| `limo_scan_normalizer` ×4 copies in experiments | identical hashes, none under `capstone_ws/src` | **Merge**: one package in `capstone_ws/src` (it is deployed on the robot) |
| Nav2 profiles (`integrated_nav2_params`, `…_lidar_only`, `…_lidar_only_slow`, `…_depth_slow`) + overlays (D3 O6, Depth `depth_layer`) | each differs from the others by a few keys | **Merge** into one base + named overlays (owner: Nav2) |
| Gate lineage: formal, r11, r12, r13 candidate, patches 02b/03b/04/05/06 | one formal file on the robot | **Merge** after the owner decides; keep the lineage in `proposals_20261120/` |
| `fleet_control/dashboard` | video-AI bridge (`bridge_node`, `readonly_rgb`), 33 references | **Keep** (rename to avoid confusion with the UI) |
| `dashboard_lab`, `dashboard_v2_design`, `rmf_web_research`, `integrated_sim` | 0 references from code/tests/scripts/rmf_adapter | **Retire candidates** → archive branch/folder (owner) |
| `capstone_ws_10_21` (symlink to `local_data/old_workspaces/capstone_ws_10_21`, a 10-21 workspace snapshot with its own `.git`), `archive/legacy_agilex_src` | no references from code, tests or launch | Keep as archive (not used); the symlink is a retire candidate, the snapshot is not to be deleted |
| Vendor demo packages (`limo_ros2_application`, `wego*`, `magic_enum`) | not in `robot_ws`, not in launches | Retire candidates (repository only) |
| Robot legacy workspaces (`~/capstone_ws`, `limo_capstone`, `agilex_ws`, `limo_ws`, `jin_ws`) | divergent copies; not in `env.sh` | **Freeze read-only**, owner review (audit §4) |
| `experiments/real_depth_slow*_20261113` (4 variants) | variants of one design with full `src` copies | Keep as records; do not source or build |

## 3. Target structure (Proposal)
Principle: laptop = development source of truth; LIMO = approved deployment environment that receives only verified, hash-listed artifacts. Reuse existing names and directories.

### 3.1 Laptop repository
```
Capstone1_limo/                                (one git repository, branch policy in 3.4)
├── capstone_ws/src/                           ROS 2 robot packages (existing): limo_capstone, limo_depth_obstacles, limo_ros2/*,
│                                              + limo_scan_normalizer (moved in from experiments, R5)
├── fleet_control/                             Fleet system (existing; must become tracked, R0)
│   ├── common/        (new)  contracts shared by server, agent, adapter: topic/node names (today agent/.../topics.py),
│   │                         SAFETY_CONTRACT_VERSION, API schema (JSON), site model schema (3.6)
│   ├── server/fleet_server (existing)  → split map_registry.py (3.5k lines) by concern in R6
│   ├── agent/limo_robot_agent (existing)  real-reachable core vs sim/mock sub-package in R5
│   ├── rmf_adapter, gazebo, scripts, tests, docs   (existing)
├── deploy/            (new, small)  robot_ws.manifest (package → source path → sha256), deploy/rollback scripts (R3)
├── experiments/       (existing)  records only: never sourced, never built, never deleted
├── local_data/        (existing, git-ignored)  raw_results/{limo_runs, <task>_<agent>_<date>}, robot_backups_<date>/, releases/<release_id>/
└── docs + 8 common documents (existing)
```
### 3.2 LIMO
```
~/Capstone1_limo/robot_ws/{src,build,install,log,env.sh,env_local77.sh}   existing: the only workspace we own
~/Capstone1_limo/{backups,tools,staging}                                  Proposal (new subfolders under the existing root, needs approval):
                                                                          one backup root, the Stage A tools, deploy staging
~/.config/limo_<purpose>/ (0700/0600)     secrets;   ~/.local/state/limo_<purpose>/   runtime state and run data (never auto-cleaned)
legacy workspaces                         read-only, never sourced or built
```
Cheaper first step: no new folders, register the existing Stage A paths in the documents (audit §6).

### 3.3 Operating rules (new folder, backup, log, record, deploy, rollback)
1. **New folder / worktree**: reuse an existing directory first; ask before creating any top-level directory or worktree; register purpose, owner, branch in `WORKTREES.md` and `CODE_INDEX.md` (AGENTS §18.2). On the LIMO only the three proposed subfolders above.
2. **Records**: one new directory per run (`run_guard.py claim`, `FINISH`); never overwrite; failed attempts are kept as `*_attempt*`.
3. **Backups**: before any deployment, tar of source and install on the robot + SHA256, copy to the laptop and compare on both ends (Stage A procedure). Real-robot data is also copied to the laptop with hashes.
4. **Deploy unit**: one package, from a laptop `deploy/robot_ws.manifest` entry; staged in a new directory; every file hash compared; `colcon build --packages-select <pkg>`; the previous directory moved outside `src`; post-deploy hash check; rollback = restore tars + hash compare (`ROLLBACK_CHECKLIST.md`).
5. **Release record**: `local_data/releases/<id>/` with manifest, hashes, test logs, approver, rollback tars location.
6. **Logs**: robot logs stay under `~/.local/state/limo_<purpose>/`; copied, not moved, to the laptop; secrets never in logs.
7. **Deletion/move/cleanup**: only after explicit per-item approval and a verified copy.

### 3.4 Git policy (needs the user's decision; nothing committed)
- Put `fleet_control/` (agent, server, tests, scripts, docs, rmf_adapter) and the common documents under version control on a dedicated branch cut from `feat/fleet-control-unified` (`ed53d81b`, an ancestor snapshot): one reviewed commit sequence per subsystem, no mixing with the owners' uncommitted files in `capstone_ws`.
- `experiments/` and `local_data/` stay out of git (PUBLICATION_POLICY): commit only summaries.
- Owner files with uncommitted changes (Gate +8 lines, real launch, depth filter, `package.xml`, `run_recorder.py`) are not touched by Fleet commits.

### 3.5 Guide for AI agents working on the LIMO (draft, to replace nothing yet)
Required reading before touching the robot: `START_HERE.md`, this design §3.2–3.3, `LIMO_DIRECTORY_AUDIT_20261122.md` §7 (ten rules), `REAL_LIMO_FLEET_INTEGRATION_PLAN_20261122.md` §7 and §16.
Additional rules proposed in this document:
- **Never use the `~/.bashrc` aliases** `stop`, `base`, `nav2`, `auto_map`, `nav_test`, `explore`, `slam`, `light`: `stop` publishes `/cmd_vel`; `base` opens the motor port; `nav2`/`auto_map`/`explore` start navigation or exploration from the legacy workspace. Do not source `~/.bashrc` in non-interactive sessions; use `env_local77.sh`. (Proposal for the owner: rename or comment these aliases; not done.)
- Never run `ros2 topic pub` on a `/cmd_vel*` topic; never launch `limo_base` or a full launch without a written approval and a stated site-safety condition.
- Anchored process patterns (`^…`) when listing or killing processes.
- Report format: Verified / Not verified / Planned / Assumed; hashes; rollback path.

### 3.6 Shared space model for building, floor, map, Open-RMF and 3D model (Proposal)
What exists (Verified): Registry tables `facilities → buildings → floors(level_name, rmf_map_name) → map_versions(resolution, origin_x/y/yaw, frame_id='map', rmf_compatibility_json) → areas`, `floor_active_maps`, `robot_map_bindings`; `rmf_adapter` rigid 2D transform `T_rmf_from_nav2` per fleet file; the 11-17 multi-floor experiment `floor_bundle.template.yaml` (per floor: `rmf_level_name`, `elevation_m`, `map_version_id`, `rmf_from_nav2`, `keepout_mask`, `initial_pose`, lift connections). Gaps: no common site frame, no 3D model reference, `rmf_from_nav2` stored per fleet file and per map version separately, Gazebo world coordinates (Office world) unrelated to the Registry.
Proposed model (one record per building, versioned by SHA256 and stored in the Registry; the 2D map stays the Nav2 source of truth):
```
site (facility) ── building ── building_frame: local ENU, metres, z up, origin and heading fixed once by survey
   └─ floor f:  elevation_m (z of the floor in building_frame),  rmf_level_name,
                T_building_from_map[f] = (x, y, yaw)   ← the one stored transform per floor (map frame of the active map version)
                map_version_id[f]  (immutable bundle, existing),   keepout_mask,   lift_connections
   └─ rmf:      T_rmf_from_building (one per building; usually identity)  → T_rmf_from_nav2[f] is DERIVED = T_rmf_from_building · T_building_from_map[f], not stored separately
   └─ model3d:  model_id, uri, sha256, format (glTF/USD/SDF), T_building_from_model (x, y, z, yaw)  → twin and Gazebo load the same model
```
Rules: every consumer (Nav2 map, RMF nav graph, visitor guide destinations, 3D twin, Gazebo world) uses `T_building_from_map[f]` and `elevation_m` from the same record; a map version binds to exactly one floor and one transform; changing a transform creates a new site-model version; `rmf_compatibility_json` is validated against the derived transform (existing `validate_rmf_compatibility`).
Acceptance tests (design): round trip map → building → RMF → 3D → map within 0.05 m and 0.5°; at least three surveyed landmarks per floor agree between the 2D map, the RMF graph and the 3D model within 0.10 m; a lift connection links floor A and B waypoints with consistent elevation; a moved transform fails the registry validation until re-verified. Real surveying is not available yet: values stay `UNVERIFIED` (as in the template).

## 4. Refactoring stages (ordered by safety and dependency)
Verification levels: **M** = Mock (pure Python/unit), **I** = isolated ROS 2 (private domain, no simulator), **G** = Gazebo (web harness), **R0** = real robot motor-locked, no motion, **R1** = real low-speed (not allowed today).
| Stage | Content | Changed files | Depends on | Risk | Regression and acceptance |
|---|---|---|---|---|---|
| **R0** Preservation | version-control `fleet_control/` + common docs (decision 3.4); copy 11-13 and 11-18 robot runs with hashes; register Stage A paths in docs; freeze robot legacy workspaces | no code; git branch; `local_data/raw_results/limo_runs/*`; WORKTREES/CODE_INDEX | user approval | low (only if no owner file is staged) | `git status` shows owner files untouched; `sha256` manifests equal on both ends; `ros2 bag info` + SQLite check |
| **R1** Contracts | `fleet_control/common`: topic/node names (`topics.py`), `SAFETY_CONTRACT_VERSION`, API/JSON schemas extracted from `http_api.py` and `core.py`; contract tests Server↔Agent | `agent/.../topics.py`, `safety_protocol.py` (import only), new `common/`, tests | R0 | low | M: all current unit/contract tests identical results; 51 routes listed in a schema test; protocol 2/contract 1 unchanged |
| **R2** Process lifecycle | deploy v4; extend the stop matrix to `motion_watchdog`, `mock_robot_node`, `supervisor`; same pattern where needed | `agent_node.py` (done), maybe `mock_robot_node.py`; tests | R0 | low | M+I: matrix 12/12 on each process; supervisor Phase 4 tests; R0: Stage A procedure repeated with SIGINT stop |
| **R3** Deployment tooling | `deploy/robot_ws.manifest` + scripts that implement §3.3-4 (backup, stage, verify, build one package, rollback) | new `deploy/`, `fleet_control/scripts` | R0 | low | dry-run on the laptop; R0: Agent v4 deploy through the script reproduces v3's checks; rollback executed once on a copy |
| **R4** Real safety chain | owners decide Gate r13 (+03b, 04, cap); launch 02b (Watchdog, `/fleet/cmd_vel_nav`); real-armed Arbiter design (new, reviewed); depth separate layer; ICP D1 with O6 | Gate, real launch, Nav2/Depth profiles, `velocity_arbiter.py` (owners) | R1, R2; owner approvals | **high** | M+I: rig 0 stale/0 unverified writer; G: full 24-test campaign with LiDAR+Depth (target config) and the 12 low-obstacle tests; R0: graph check (one `/cmd_vel_raw` writer, `/cmd_vel` 0, Gate reports epoch fields), then separate approval for R1 |
| **R5** Package hygiene | `limo_scan_normalizer` into `capstone_ws/src`; Nav2 base+overlays; one recorder; Agent real core vs `sim/` sub-package; retire candidates archived (owner) | `capstone_ws/src`, launch files, Agent package layout | R1; owners | medium (launch + build paths) | M+I+G regression; **robot_ws hash comparison** shows only intended differences; real launch hash unchanged until the owner re-approves |
| **R6** Server modularity | split `map_registry.py` (schema/validation/RMF/areas/audit), `service.py`, `http_api.py` into modules; no behaviour change; API versioning | `server/fleet_server/*` | R1 | medium | M: full server tests + API contract tests byte-identical responses on recorded requests; I: Phase 1–5 isolated suites; browser E2E |
| **R7** Space model | implement §3.6 in the Registry + `rmf_adapter` + guide destinations; multi-floor sim (reuse 11-17 clinic world); 3D model binding | `map_registry`, `rmf_adapter/coords.py`, `fleet.json`, new site schema | R1, R6 | medium | M: transform algebra + validation tests; G: two-floor sim with lift; round-trip and landmark tests (3.6); R0: map/pose consistency only (no motion) |
| **R8** Visitor and 3D twin integration | visitor web on the shared model; twin reads Registry API read-only | `guide.py`, `visitor.*`, new twin client | R7 | medium | I/G: Phase 5 suites; destination → goal check uses the shared transform; no write path from the twin |
Real-robot gate for any stage touching a motion path: separate written approval, site-safety statement, the §7 blockers cleared (Gate contract, Arbiter/Watchdog on the path, STM32 timeout, braking distance, E-Stop, DDS exposure, base USB, localization).

## 5. Dependencies and risks
| Risk | Where | Mitigation |
|---|---|---|
| R-1 untracked Fleet code | §1.1 | R0 first; interim: the `deploy_candidate_agent_v*` tars and `local_data` copies |
| R-2 owners' uncommitted changes (Gate, launch, depth filter, `package.xml`) are on the robot only partly | §1.2 | never stage them; compare by hash before every deploy; owners decide |
| R-3 robot `robot_ws` has no VCS | §1.2 | manifest with hashes (R3); the laptop repository is the only source |
| R-4 11-13 originals only on the robot | §2.3 | R0 copy before any robot cleanup |
| R-5 alias `stop` and `env.sh` | §1.4 | rules 3.5; owner to rename aliases; `env_local77.sh` only |
| R-6 launch/package renames break the robot build | R5 | one package at a time, hash check, rollback |
| R-7 splitting `map_registry.py` changes behaviour | R6 | recorded-request replay, no logic edits in the same commit |
| R-8 transforms stored in two places drift | §3.6 | derive, validate, version |

## 6. Recommended order and approvals
1. **R0** (needs: decision on the git branch policy; permission to copy the 11-13/11-18 robot runs). 2. **R2** (needs: v4 deploy approval; backup first). 3. **R3** and **R1** in parallel (laptop only). 4. **R5** items that do not touch launch files, then owner decisions for **R4** (Gate, launch, Depth layer, ICP). 5. **R6**, then **R7** and **R8**.
Nothing here authorises motion. Pending owner/user decisions: git branch for `fleet_control`; copy of 11-13 data; register the Stage A paths and correct `~/limo_field_agent_ws` in CODE_INDEX/WORKTREES; whether to rename the `.bashrc` aliases; Gate/launch/Nav2/Depth/ICP owners' decisions for R4; whether the three Stage A directories move under `~/Capstone1_limo/`.

## 7. Proposed changes to the common documents (not applied: AGENTS.md is modified by another party; the others are shared)
| Document | Proposed addition |
|---|---|
| `START_HERE.md` | one line under §1: "Working on the LIMO: read `fleet_control/docs/LIMO_DIRECTORY_AUDIT_20261122.md` §7 and `LIMO_REFACTORING_DESIGN_20261122.md` §3.3/§3.5 first" |
| `AGENTS.md` | §15/§18: robot paths are documented only in `WORKTREES.md`; add "aliases in `~/.bashrc` on the LIMO are forbidden for agents" (**do not edit while the working-tree change of AGENTS.md is pending**) |
| `CODE_INDEX.md` / `WORKTREES.md` | correct `~/limo_field_agent_ws` → `robot_ws/src/limo_robot_agent`; register `~/backups`, `~/stage_a_v3`, `~/limo_isolated_18290_tools`, `~/.local/state/limo_robot_agent_isolated_18290`, `~/.config/limo_isolated_18290`; add "fleet_control is untracked on `staging/…` (646 files)" |
| `ARCHITECTURE.md` | link to this design under the 2026-11-20 audit section |
| `DECISIONS.md` | none: nothing here is an approved decision |
