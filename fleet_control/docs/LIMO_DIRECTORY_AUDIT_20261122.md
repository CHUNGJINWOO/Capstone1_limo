# LIMO (`pcu`) directory and working-rule audit (2026-11-22, Claude; read-only)

Scope: what is on the robot's home directory, how it relates to the repository documents, and a standard layout that **reuses the existing directories**. Nothing on the robot was created, moved, deleted or changed by this audit (read-only SSH only). Evidence: `local_data/raw_results/real_limo_integ_claude_20261122/limo_dir_audit_01/run/`.
Legend: **Verified** = observed on 2026-11-22 20:26–20:35. **Record** = from a dated repository document. **Inference** = my reading, not confirmed by the owner.
Secret files were listed by name and mode only; no token or `secrets/` content was read.

## 1. Actual structure (Verified)
Host `pcu@pcu`, Ubuntu 22.04.5, ROS 2 Humble, 117 GB disk (76 GB free, 32 % used), single user `pcu` (no files owned by anyone else in the first two levels).

### 1.1 Home directory
| Path | Size | Last change | What it is | Evidence of origin |
|---|---|---|---|---|
| `~/Capstone1_limo/` | 388 MB | 11-20 | **Project root of this repository's deployment.** Contains only `robot_ws/` and one pre-deploy tar (`predeploy_agent_20261120_085514.tar.gz` + 2 hash files). **Not a git repository**; its contents are copied from the laptop | our 11-19/11-20 records |
| `~/Capstone1_limo/robot_ws/` | 351 MB | 11-22 | **The one ROS workspace used on the LIMO**: `src` 125 MB, `build` 219 MB, `install` 5.7 MB, `log` 1.5 MB, `env.sh`, `env_local77.sh` | WORKTREES 11-19 |
| `~/backups/` | 548 KB | 11-22 | Stage A backups (created today) | this session |
| `~/stage_a_v3/` | 116 KB | 11-22 | Stage A staging copy of Agent v3 (tar, hash files, empty `x/`) | this session |
| `~/limo_isolated_18290_tools/` | 16 KB | 11-22 | Stage A temporary tools (2 scripts) | this session, per the user's instruction |
| `~/predeploy_mapsave_agent_20261120_134125.tar.gz` (+ `.sha256`) | 184 KB | 11-20 | Backup of the Agent before the map-save deploy | our 11-20 record |
| `~/maps_h107_20261120/` | 4.8 MB | 11-20 | H107 saved map (`map.pgm/yaml`, `posegraph.*`) | our 11-20 record |
| `~/limo_video_kit/` | 112 KB | 11-20 | video/AI sender-consumer kit (`real_limo_kit/*.sh`, `nodes`, `vlink`; has a `secrets/` directory, not read) | video-AI work |
| `~/load_experiment_env.sh`, `~/run_unpowered_trial.py`, `~/test_rosbag_record.sh` | small | 11-09 | 11-09 experiment helpers; the first sources `~/capstone_ws/install` and `~/capstone_experiment_ws/install` (**the second does not exist**) and sets `ROS_DOMAIN_ID=0` | earlier experiments |
| `~/capstone_ws/` | 1.4 GB | 11-20 | **Older team workspace**; `src` is a git repo (`mavic89k/limo_workspace`, branch `dev`, clean, last commit 11-20) containing 23 entries incl. `limo_capstone_ros`, `limo_depth_obstacles`, `m-explore-ros2`, `ros2_astra_camera`, and also stray `build/ install/ log/` folders inside `src` | team member's repository |
| `~/limo_capstone/` | 404 KB | 10-15 | Early package repo (`mavic89k/limo_capstone`, `main`, clean); maps `demo_map`, `domi_map` | team member |
| `~/agilex_ws/` (500 MB), `~/limo_ws/` (503 MB, vendor `limo_ros2` 2023 + 2 dirty files), `~/jin_ws/` (331 MB), `~/YDLidar-SDK/` (118 MB, 39 dirty files) | – | 10-2 … 2025-12 | vendor and earlier-semester workspaces | vendor / earlier students |
| `~/snap/` (1.3 GB), `~/.ros/` (102 MB), `~/.nx/`, `~/.gemini/`, `~/다운로드/` (112 MB) | – | – | OS/application data | – |
| Loose files: `aruco_*.py`, `arduco_detect.py`, `4x4_1000-*.jpg`, `ydlidar-test.yaml`, `slam_toolbox_mapping.launch.py`, `limo_mapping.log`, `인수인계`, `how` | small | 2025-12 … 2026-10 | earlier experiments and a hand-over note (contents not read) | earlier students |
| `~/build/`, `~/install/`, `~/log/`, `~/__pycache__/` | small | 10-16 | colcon output created by a build run from `~` (has `COLCON_IGNORE`) | accidental |

### 1.2 Configuration and state (hidden)
| Path | Size | Role | Verified detail |
|---|---|---|---|
| `~/.config/limo_field_20261113/` (700) | 12 KB | 18190 field Agent token + `field.env` (both 600) | **do not touch**: used by the 18190 start path |
| `~/.config/limo_fleet/` (700) | 8 KB | older Agent token (600) | 11-12 |
| `~/.config/limo_isolated_18290/` (700) | 4 KB | Stage A isolated-server token directory | empty now (token deleted) |
| `~/.local/state/limo-real-auto-mapping/` | **8.2 GB** | Real auto-mapping runs with rosbags: `20261113_run_012` (2.5 GB), `20261118_run_004` (853 MB) and `…_run_004_bag_recovered_copy` (837 MB), `20261113_run_021/015/016`, `20261118_run_001/003`, … | **original field data; never delete** |
| `~/.local/state/limo_locked_stack/` | 1.4 MB | 11-20 locked-stack runs (6 timestamped dirs) | field records |
| `~/.local/state/limo_robot_agent_field_20261120/` | 9.6 MB | 11-20 field Agent state, `agent.out`, `map_saves/` | **do not touch**: 18190-era Agent state |
| `~/.local/state/limo_robot_agent_field/`, `…/limo_robot_agent/`, `…/limo-depth-observe/`, `…/limo_field_cam/` | small | earlier Agent and observation runs | records |
| `~/.local/state/limo_robot_agent_isolated_18290/` | 24 KB | Stage A Agent state/logs | this session |

### 1.3 ROS workspaces and what they contain
| Workspace | Packages relevant to the LIMO | Used by the real launch? |
|---|---|---|
| `~/Capstone1_limo/robot_ws/src` | `astra_camera`, `astra_camera_msgs`, `explore`, `explore_lite_msgs`, `limo_base`, `limo_capstone`, `limo_depth_obstacles`, `limo_description`, `limo_msgs`, `limo_robot_agent`, `limo_scan_normalizer`, `ydlidar_ros2_driver` (12 entries) | **Yes** (`env.sh`/`env_local77.sh` source its `install`) |
| `~/capstone_ws/src` | package `limo_capstone_ros` (the same code under another name) and others | No. **Divergent copies**: launch `c83fe324…` vs robot_ws `b87e16d3…`, Gate `3581e203…` vs `d103f445…`, slow profile `d27112c6…` vs `208d7f44…`, depth filter `657bbeba…` vs `ba01c612…` |
| `~/limo_capstone`, `~/agilex_ws`, `~/limo_ws`, `~/jin_ws` | older/vendor | No |
Nothing is auto-started: no systemd unit, no cron, no autostart entry (Verified).

## 2. Path references in launch files, scripts and environment (Verified)
| Referrer | What it references |
|---|---|
| `robot_ws/env.sh` | `source /opt/ros/humble/setup.bash`, `source /home/pcu/Capstone1_limo/robot_ws/install/setup.bash`, domain 77, `ROS_LOCALHOST_ONLY=0`, `ROS_DISCOVERY_SERVER=<ROBOT_TAILNET_IP>:11811` (**tailnet-wide DDS exposure**) |
| `robot_ws/env_local77.sh` | same sources, domain 77, `ROS_LOCALHOST_ONLY=1`, discovery unset (**the only environment allowed for locked tests**) |
| `~/.bashrc` | aliases `slam`, `save_map`, `base`, `lidar`, `urdf`, `light`, `nav2`, `auto_map`, `explore`, `nav_test` pointing to **`~/capstone_ws/...`** and old launches; **alias `stop` publishes `/cmd_vel` once** (`ros2 topic pub --once /cmd_vel …`); `base` starts `limo_base` (opens the motor serial port); `nav_test`/`nav2` start Nav2 |
| `robot_ws/src/limo_robot_agent` | only `expanduser('~/.local/state/limo_robot_agent')` as the default state path (no `/home/pcu` literal elsewhere) |
| `~/limo_isolated_18290_tools/start_robot_agent_field.sh` | defaults `WS=$HOME/Capstone1_limo/robot_ws`, `STATE_DIR=$HOME/.local/state/limo_robot_agent_field_20261120` (**overridden to the isolated directory in Stage A**), token default `~/.config/limo_field_20261113/agent-pcu.token`, sets domain 77 + localhost only |
| `~/load_experiment_env.sh` | `~/capstone_ws/install`, **non-existent** `~/capstone_experiment_ws/install`, `ROS_DOMAIN_ID=0` |
| `~/limo_video_kit/real_limo_kit/*.sh` | own `common.sh`; not audited in depth |

## 3. Differences from the repository documents (Verified vs Record)
| Document statement | Reality |
|---|---|
| CODE_INDEX / WORKTREES (11-18): Agent at `~/limo_field_agent_ws` | **That directory does not exist** (the Agent now lives only in `robot_ws`, since 11-19/20). The index should point to `robot_ws/src/limo_robot_agent` |
| WORKTREES 11-19: LIMO path `/home/pcu/Capstone1_limo/robot_ws`, `capstone_ws` untouched | Matches. `capstone_ws` is still there with divergent copies of the same files |
| VALIDATION 11-20: pre-deploy backups next to the workspace | Two 11-20 tars exist in two different places (`~/` and `~/Capstone1_limo/`) plus the Stage A backups in `~/backups/` (three locations) |
| AGENTS §15: do not assume a fixed workspace path | The Agent source has no hard-coded path; the start script and env files do (`/home/pcu/...`, `$HOME` defaults) |
| AGENTS §18.2: reuse existing directories; register new ones | Three Stage A directories were created today at the user's instruction (`~/backups` was new, `~/stage_a_v3`, `~/limo_isolated_18290_tools`); they are not yet in `WORKTREES.md`/`CODE_INDEX.md` (proposal in §6) |

## 4. Duplicates, unneeded-looking folders and what must be kept
Candidates for the owners to review later (**nothing deleted or moved**):
| Item | Why it looks redundant | Risk if removed |
|---|---|---|
| `~/capstone_ws`, `~/limo_capstone`, `~/agilex_ws`, `~/limo_ws`, `~/jin_ws`, `~/YDLidar-SDK` | not used by the real launch; two have uncommitted changes (`limo_ws/limo_ros2`: 2, `YDLidar-SDK`: 39) | `capstone_ws` is a teammate's git repo (clean, on branch `dev`); the others may hold hand-made calibration or SDK builds. Ask owners first |
| `~/build`, `~/install`, `~/log`, `~/__pycache__` | stray colcon output in `~` | low, but not mine |
| `~/predeploy_agent_20261120_085514.tar.gz*` (in `~/Capstone1_limo/`) and `~/predeploy_mapsave_agent_20261120_134125.tar.gz*` (in `~/`) | backups in three places | **these are the only copies on the robot of the pre-11-20 Agent; the laptop has copies in `local_data/robot_backups_20261120/`** |
| `~/.local/state/limo-real-auto-mapping/20261118_run_004_bag_recovered_copy` | copy of `…_run_004` (837 MB vs 853 MB) | evidence of the recovery; keep |
| Loose `aruco_*`, `arduco_detect.py` (typo duplicate of `aruco_detect.py`), `4x4_*.jpg`, `how`, `인수인계` | earlier experiments | the hand-over note may matter; owner decides |

**Must not be deleted or changed by anyone without explicit approval**
- `~/.local/state/limo-real-auto-mapping/` (8.2 GB original rosbags and maps), `~/.local/state/limo_locked_stack/`, `~/.local/state/limo_robot_agent_field_20261120/`, `~/maps_h107_20261120/`.
- `~/.config/limo_field_20261113/`, `~/.config/limo_fleet/` (18190-era tokens and `field.env`).
- `~/backups/` (rollback source of the deployed Agent), `~/stage_a_v3/` (until the user releases it), the two 11-20 pre-deploy tars.
- `~/Capstone1_limo/robot_ws/src/*` (the deployed code; not a git repo, so there is no other copy than the laptop repository).
- Any uncommitted change in `~/limo_ws/limo_ros2`, `~/YDLidar-SDK`.
Memory rule of this project: real-robot rosbags, logs and maps must also exist on the laptop. The 8.2 GB mapping state dir was **not** compared by hash with the laptop in this audit (Not verified). By name only: the laptop has `local_data/raw_results/real_limo_*_20261118_*` (11-18 results and recovered bags) and `limo_runs/20261119_*`, `20261120_*`; **no directory named after the 11-13 runs (`run_012`, `run_015`, `run_016`, `run_021`, about 4.1 GB on the robot) was found on the laptop** (`experiments/real_limo_field_20261113/` holds only reports). Those robot copies may be the only originals: treat them as irreplaceable until a hash comparison shows otherwise.

## 5. Laptop development environment vs the LIMO deployment
| Aspect | Laptop (`Capstone1_limo` repository) | LIMO |
|---|---|---|
| Source of truth | git repository, many branches/worktrees, `capstone_ws/src`, `fleet_control/` | no git for `robot_ws`; a **selected copy** of 12 packages; `fleet_control/agent/limo_robot_agent` is the only package developed here and deployed there |
| ROS workspace | `capstone_ws` (full, Gazebo, RMF, Nav2 experiments) | `robot_ws` (minimal, built on `/opt/ros/humble` only, `PYTHONNOUSERSITE=1`) |
| DDS | isolated domains 200–230 for tests, `env_laptop.sh` as a Discovery client | domain 77; `env_local77.sh` (localhost) for locked tests; `env.sh` (Discovery Server, tailnet) |
| Network | Tailscale `<LAPTOP_TAILNET_IP>` | Tailscale `<ROBOT_TAILNET_IP>`; reaches the laptop only through a tunnel in Stage A |
| Tests | unittest/pytest, Gazebo, isolated rigs | none; only the Agent and read-only probes |
| Launch | many (Gazebo, isolated stack) | one: `limo_integrated_auto_mapping_real.launch.py` (hash `b87e16d3…`, = git HEAD + scan normalizer) |
| Secrets | `local_data/…/secrets` (0600, git-ignored) | `~/.config/limo_*` (0700/0600) |
| Data | `local_data/raw_results`, `local_data/robot_backups_*` | `~/.local/state/limo-*`, `~/backups` |

## 6. Proposed standard structure (reuses existing directories; **nothing created**)
Principle: one root, one workspace, one backup root, state under `~/.local/state`, secrets under `~/.config`. Existing directories are kept as they are; only **new subfolders under the existing root** are suggested, each needing approval.
```
~/Capstone1_limo/                      (existing; root for everything we own on the LIMO)
├── robot_ws/                          (existing; the only ROS workspace we build; env.sh, env_local77.sh)
│   └── src/{limo_robot_agent, limo_capstone, ...}   (selected copies; source of truth = laptop repository)
├── backups/    (proposed: ONE backup root. Today ~/backups, ~/ and ~/Capstone1_limo hold backups)   
├── tools/      (proposed: the two Stage A scripts; today ~/limo_isolated_18290_tools)
└── staging/    (proposed: deploy staging; today ~/stage_a_v3)
~/.config/limo_<purpose>/              (existing pattern; tokens 0600 in 0700 directories)
~/.local/state/limo_<purpose>/         (existing pattern; Agent state, runs, rosbags; never cleaned automatically)
legacy, read-only, not sourced, not built: ~/capstone_ws, ~/limo_capstone, ~/agilex_ws, ~/limo_ws, ~/jin_ws, ~/YDLidar-SDK
```
Cheaper alternative with **no new folders and no moves**: keep `~/backups`, `~/stage_a_v3`, `~/limo_isolated_18290_tools` where they are and only register them (purpose, owner, retention) in `WORKTREES.md` and `CODE_INDEX.md`. Recommended first step: this one.
Moving steps (when approved, each with SHA256 compare before and after and a rollback note): consolidate the two 11-20 pre-deploy tars into the single backup root; move the Stage A tools and staging copy; do not move anything under `~/.local/state/`.
Document updates proposed (not made): `CODE_INDEX.md` and `WORKTREES.md` — correct `~/limo_field_agent_ws` to `robot_ws/src/limo_robot_agent`, add the three Stage A paths with purpose and "keep until released", add a "LIMO deployment" section with the table in §1.1; `START_HERE.md` — one line pointing to the LIMO rules below.

## 7. Proposed working rules for an AI agent operating on the LIMO (draft; not installed anywhere)
1. **Read-only first.** Start with `ps`, `ss -ltn`, `ls`, `sha256sum`; list processes with a pattern that cannot match your own command line (e.g. anchor with `^`).
2. **One workspace.** Use only `~/Capstone1_limo/robot_ws`. Never source or build `~/capstone_ws`, `~/limo_capstone` or the other legacy workspaces; never use the `~/.bashrc` aliases (`stop` publishes `/cmd_vel`, `base`/`nav2`/`auto_map` start motors and navigation).
3. **Environment.** `. ~/Capstone1_limo/robot_ws/env_local77.sh` (domain 77, localhost only). Never `env.sh` for tests; never change either file.
4. **No motion.** Do not start `limo_base`, the full launch, Gate, Nav2, or publish `/cmd_vel*` unless the user gave a separate, written approval for that step and confirmed site safety (wheels off the floor or a person at the physical stop). Software locks are observation, not an independent block.
5. **Deploy discipline.** Before replacing any package: tar backup (source and install) + SHA256 on the robot, copy to the laptop, compare on both ends; stage in a new directory; compare every file hash with the laptop `FILES.sha256`; build only `--packages-select <package>`; keep the previous directory outside `src`.
6. **Secrets.** Tokens only through stdin into a 0600 file in a 0700 directory; never in a command line, log, document or chat; delete only the tokens you created, at the end of the test.
7. **Ports.** 18190 is the operational server: never start, stop or change it. Tests use 18290 through an SSH reverse tunnel (`-R 127.0.0.1:18290:127.0.0.1:18290`, `ExitOnForwardFailure=yes`); check that the listener is loopback-only on both ends.
8. **Data.** Never delete or overwrite rosbags, runs, maps, field state or backups. Copy real-robot data to the laptop (`local_data/raw_results/…`) and verify hashes. New runs get new directories.
9. **Cleanup.** Stop only processes you started; record leftovers; list (do not delete) what you created. Deleting or moving anything needs explicit approval per item.
10. **Report.** State what is Verified, Not verified, Planned; give file hashes, logs, PASS/FAIL/SKIP and the rollback path. No commit/push/PR from the robot.
