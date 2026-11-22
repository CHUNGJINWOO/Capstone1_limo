# Agent v4 real-robot deployment: preparation (2026-11-22, Claude; NOTHING deployed, started or changed on the robot)

Scope of this step: checks, laptop-only tests and a rehearsal. The robot was read from over SSH (state listing, one read-only `preflight`); no file was created, moved or replaced there. The G1 baseline commit (`0ca09dc4`, branch `r0/fleet-control-snapshot-20261122`), HEAD, the working tree of others, and the robot originals are untouched; no commit, push or PR.
Records: `local_data/raw_results/real_limo_integ_claude_20261122/v4_deploy_prep_01/` (`run/` has the robot state, diffs, matrix, lifecycle, regression and rehearsal logs).

## 1. Robot state (Verified 21:50)
| Item | State |
|---|---|
| Reachability | Tailscale up (89 ms), SSH by key, uptime 6 h 50 min |
| Running | 0 robot/ROS processes (`robot_agent`, `limo_base`, launch, bag), no listener on the operational or the isolated server ports, nobody holds `/dev/ttylimo` or `/dev/ttyUSB1` |
| Agent source | `robot_ws/src/limo_robot_agent` equals the v3 `FILES.sha256` (32 files; `agent_node.py` `94ade245…`) |
| Build | `build/limo_robot_agent/limo_robot_agent` resolves to the source directory (symlink install); entry scripts `robot_agent`, `motion_watchdog`, `mock_robot` present |
| Existing backups | `~/backups/`: `limo_robot_agent_{src,install}_pre_20261122_200059.tar.gz`, `.tars.sha256`, `.files.sha256`, and the replaced pre-v3 source directory. These are the **pre-v3** Agent (11-20): **they are the rollback target for v3, not for v4**. A new backup of the installed v3 is part of the deploy |
| Stage A leftovers (kept, not touched) | `~/stage_a_v3/`, `~/limo_isolated_18290_tools/` (2 scripts, the old interface: both arguments optional with the operational server as default), `~/.config/limo_isolated_18290/` (empty, token deleted), `~/.local/state/limo_robot_agent_isolated_18290/` (Stage A logs) |
| Disk | 76 GB free |
| Other Stage A facts reused | rsync/python/colcon present; sshd forwards only to the robot's loopback (no `GatewayPorts`) |

## 2. v3 -> v4 (Verified)
Same 32-file set; **one file differs**: `limo_robot_agent/limo_robot_agent/agent_node.py` (v3 `94ade245…` -> v4 `6a8252da…`, 59 added or removed lines, diff in `run/agent_node_v3_to_v4.diff`). Content: SIGINT/SIGTERM handlers that stop the loop, a watcher for a shut-down ROS context, per-step shutdown cleanup, plus the real-mode configuration guards already in v3 (mode names, `allow_missions`, sim/mock on domain 77, `web_gate_reset`).
The deploy artifact `deploy_candidate_agent_v4/` (tar `97071d50…`, `FILES.sha256`) = the working tree = the G1 commit `0ca09dc4` (all 32 files compared byte for byte). No other robot package, launch file, Gate, Nav2/Depth/ICP profile, Arbiter or Watchdog is part of the change.

## 3. Test results on the exact deploy artifact (laptop, isolated domains, loopback; no robot)
| Check | Result |
|---|---|
| Stop matrix on the **extracted v4 tar** (server up / unreachable / dies / reconnect × SIGINT, SIGINT ignored, SIGTERM) | **12/12 PASS**, exit ≈ 0.2 s, rc 0, no traceback, no ghost node, no new shared-memory file |
| Lifecycle on the artifact, real mode, isolated server: online, reconciled, `movement_permitted=false`; Lease acquire/release ACK; Mapping START, Goal, Manual, Gate Reset refused | 12/12 PASS (twice) |
| Stop and restart: SIGTERM -> exits (`signal 15 received: stopping`, `robot agent stopped`) -> server shows offline after ~6 s -> restart gives a **new boot id** -> SIGINT exits | PASS |
| Regression (17 modules incl. the stop test, real-mode blocks 13/13, map save 17+20, readiness 10, status 7, Lease 2, Phase 4 supervisor 7, Arbiter generation 14, locked Arbiter 3) | all OK |
| Still not judged (same as before v4): fixture-based `test_agent_safety_v1`, `test_terminal_agent_order`, `test_r8_reconcile_cancel` | open |

## 4. Deploy and rollback procedure (rehearsed on the laptop)
New tool: `fleet_control/scripts/limo_package_deploy_20261122.sh` (sha256 prefix `b6b5f959…`): actions `preflight`, `backup`, `stage`, `replace`, `build`, `verify`, `rollback`. It is streamed to the robot with `ssh … 'bash -s -- ACTION …' < script`, so no script file is left there. It **never deletes** (replaced and failed directories are moved into the backup directory), starts no ROS node, builds only `--packages-select limo_robot_agent`, refuses to run while any robot/ROS process exists, refuses to overwrite a backup or stage directory, and verifies every hash.
Rehearsal in a scratch colcon workspace seeded with v3 (`run/rehearsal_*.log`): preflight, backup, stage, replace, build, verify = v4 (installed code has the stop handler), rollback restores v3 (32 files equal the pre-deploy list, installed code is v3 again). Abort tests: a running process named `robot_agent` -> ABORT; a tampered artifact -> "tar hash mismatch"; replace without a staged package/backup -> ABORT. The first rehearsal exposed two bugs of my own (a silent exit caused by `pipefail`, and a process check that matched its own command line), both fixed and re-tested; the failed logs are kept as `*_attempt1_*`. The read-only `preflight` was also run **on the robot**: "no processes, package present, 77 GB free", nothing created.
**Steps on the robot (each needs the approval named in §6)**, with `STAMP=v4_20261122` and placeholders for addresses:
1. `preflight` (done once read-only).
2. Copy the artifact: create `~/stage_a_v4` (0700) and copy `limo_robot_agent_candidate_v4.tar.gz`, `TAR.sha256`, `FILES.sha256` into it (`scp`, hashes compared on both ends). `~/stage_a_v3` is not touched.
3. `backup` into `~/backups` (new files `limo_robot_agent_{src,install}_pre_v4_20261122.tar.gz`, hash lists); copy the three backup files to the laptop `local_data/robot_backups_<date>/` and compare SHA256 on both ends **before** step 4.
4. `stage` (new directory `~/backups/stage_v4_20261122`), `replace` (v3 source directory moved to `~/backups/limo_robot_agent_src_dir_replaced_v4_20261122`), `build`, `verify` against the v4 `FILES.sha256`.
5. Isolated first run (Stage A procedure, §5), then the post-deploy checks.
**Rollback** (any failed step, or on request): `rollback` with the same stamp: the failed v4 directory is moved to `~/backups/limo_robot_agent_src_dir_failed_v4_20261122`, the v3 backup is restored, rebuilt and compared with the pre-deploy hash list. The previous (pre-v3) backups stay as an older rollback point. Time: seconds (a package build takes about one second).

## 5. Isolated run after the deploy (Stage A procedure reused)
Same as Stage A (§15–§16 of the integration plan), with these differences:
- Server: `run_field_server.sh` on the laptop, loopback only, data/tokens in `isolated_server_prep_01` (still present; tokens 0600); SSH reverse tunnel `-R 127.0.0.1:18290:127.0.0.1:18290` with `ExitOnForwardFailure=yes`; the listener must be loopback-only on both ends (checked before the Agent starts).
- Token: one file, via stdin with `umask 077`, into the (empty) `~/.config/limo_isolated_18290/`; deleted at the end. Nothing else is installed.
- Start: the robot's existing tool copy with **explicit** arguments (server `http://127.0.0.1:18290`, the isolated token file) and `STATE_DIR` set to the isolated state directory, `FORCE_START=1` (the DDS slot precheck reports "no healthy slot" because no stack runs, as in Stage A). The laptop copy of the script now requires both arguments (G1 patch P3); the robot copy is unchanged (replacing it is a separate robot change).
- **New measured step: graceful stop on the real robot.** Stop the Agent with `SIGINT`, require exit within 10 s and `robot agent stopped` in the log; only if it fails, SIGKILL and roll back to v3 (the v3 behaviour was the reason for this release). Then restart (new boot id), server restart, tunnel drop and restore, as in Stage A.
- Checks as in Stage A: ONLINE/robot_id/boot_id, heartbeat age, Lease acquire/release/expiry, ACKs, Mapping START / Goal / Manual / Gate Reset refused, no `limo_base`, no motor-port user, no `/cmd_vel*` topic.
- Teardown: Agent stopped (SIGINT), tunnel and server stopped, token deleted, and the result recorded. v3 and v4 backups, `~/stage_a_v3`, `~/stage_a_v4`, the tools and state directories are listed and **not** deleted or moved.

## 6. Changes on the robot, abort conditions, approvals
**What would change (only after approval):** new directory `~/stage_a_v4` (3 files); new files in `~/backups` (v3 backup, hash lists, stage directory, the moved v3 source directory); `robot_ws/src/limo_robot_agent` replaced by v4 and its `build/`/`install/` entries and the workspace `install/setup.*` helper scripts regenerated by colcon (as in Stage A); during the isolated run: one token file, the isolated state directory, one SSH tunnel, one Agent process. **Unchanged:** every other package, the real launch, Gate, Nav2/Depth/ICP profiles, Arbiter, Watchdog, drivers, udev, `env*.sh`, the operational server and its state/config, all robot originals, the motor port.
**Abort (stop, restore v3 if the source was already replaced):** any robot/ROS process present; `preflight` fails or < 1 GB free; any hash mismatch (artifact, stage, backup copy, deployed source); backup copy to the laptop not verified; build error or import error; Agent not online/reconciled within 30 s; a listener for the isolated port on anything but loopback; a `/cmd_vel` or `/cmd_vel_raw` topic or a user of the motor port appears; the operational port 18190 involved in any way; SIGINT does not stop the Agent in 10 s (then SIGKILL, roll back to v3); `movement_permitted` ever true; any unexpected refusal result changing (Mapping START, Goal, Manual, Gate Reset must stay refused).
**Approvals needed (each separately, as in Stage A):** (1) the deploy window and site safety (wheels off the floor or a person at the physical stop; L1–L5 are observation criteria, not an independent block); (2) creating `~/stage_a_v4` and the new backup files on the robot and copying them to the laptop; (3) replacing `limo_robot_agent` with v4 and rebuilding that package; (4) the isolated run: loopback server, SSH tunnel, token file, Agent start and stop; (5) optional, separate: replacing the robot's tool copy of `start_robot_agent_field.sh`.
Remaining safety blockers for motion are unchanged (Gate HEAD contracts, no Arbiter/Watchdog on the real path, STM32 timeout/braking/E-Stop unmeasured, base USB interruptions, localization, `env.sh` exposure, shared Depth layer). v4 is a lifecycle fix; it does not change what the Agent may command.

## 7. Relation to R1–R3 of the refactoring design
This preparation already contains the first R3 artifact (`limo_package_deploy_20261122.sh`, backup/stage/replace/build/verify/rollback with hash lists) and the R2 evidence (stop matrix on the artifact). After v4 is verified on the robot: R1 (shared contracts) and R3 (manifest-driven deploy for all packages) can start on the laptop; R2 still needs the same matrix for `motion_watchdog`, `mock_robot_node`, `supervisor`.

## 8. Real-robot isolated run of Agent v4 (2026-11-22 21:59–22:05 KST, Claude; user approval and site-safety confirmation given)
Record: `local_data/raw_results/real_limo_integ_claude_20261122/v4_isolated_01/` (`run/` incl. `robot_logs/`). Robot: installed Agent == v4 (32 files), no ROS/motor processes before; laptop and robot clocks equal.
**Environment:** isolated server on the laptop loopback 127.0.0.1:18290 (data/tokens `isolated_server_prep_01`), SSH reverse tunnel `-R 127.0.0.1:18290:127.0.0.1:18290` with `ExitOnForwardFailure=yes` (listener loopback-only on both ends, robot -> server without token 401), token via stdin `umask 077` (mode 600 in a 700 directory, hash equal), state directory `~/.local/state/limo_robot_agent_isolated_18290` (set explicitly), Agent started with the robot's tools script and **explicit** URL and token path, no extra arguments: `mode=real simulated=false allow_missions=false`, domain 77 localhost. The operational port 18190 was never used (no listener at any time).
**FORCE_START=1, actual scope (read from the script and precheck source, then observed):** it only skips the abort when the DDS slot precheck exits 4 (`NO_HEALTHY_SLOT_FOUND`: with no stack there is nothing publishing `/scan`/`/odom`). The precheck (exit 4 observed) is a subscribe-only canary with a temporary FastDDS profile in `/tmp`; the 30-s warm-up participant `fleet_agent_warmup` publishes nothing. No mode or safety parameter changes.
| Item | Result |
|---|---|
| Start/stop times (KST) | tunnel+token 21:59:27; run 1 started 21:59:36 (up 22:00:24, 46 s); server/tunnel tests until 22:02:16; SIGINT 22:02:27; run 2 started 22:02:45; SIGTERM 22:03:43; teardown done 22:05:00 |
| ONLINE, robot_id `pcu`, host `pcu`, domain 77, `mode=real`, `simulated=false`, `missions_enabled=false`, reconciled | **PASS**; `movement_permitted=false` in every sample |
| Heartbeat age | PASS: 0.2–0.8 s |
| Lease acquire/release ACK, Lease expiry (10 s lease) | **PASS** (gone after 10.1 s, Agent online throughout) |
| Mapping START / Goal / Manual / Gate Reset | **PASS (refused)**: 409 `mapping_real_blocked`; Goal 409; Manual 409 (`mode_unsupported`, bench 409); Gate Reset admin 409 `gate_reset_refused` (`AGENT_GATE_RESET_UNSUPPORTED`…), operator 403. Probe 12/12 in run 1 and run 2 |
| Server restart | **PASS**: 1, 2, 4, 8 s back-off, "server restarted; reconnected", online/reconciled |
| Tunnel dropped and restored | **PASS**: offline while down, online/reconciled after |
| Restart | **PASS**: new boot id (`66205bdc48cf` -> `173fd65aeeea`) |
| **SIGINT stop on the robot** | **PASS**: agent process gone in **0.164 s**, whole start chain finished in 0.167 s, **exit code 0** (`ros2 run` wrapper), log `signal 2 received: stopping` / `robot agent stopped`; server marks offline after about 6 s |
| **SIGTERM stop on the robot** | **PASS**: **0.162 s**, chain 0.167 s, **exit code 0**, `signal 15 received: stopping` / `robot agent stopped` |
| ROS graph while running | only node `/fleet_robot_agent`; no `/cmd_vel`, `/cmd_vel_raw`, `/cmd_vel_locked`, `/fleet/nav_cmd_vel` topic; Agent is the only publisher of `/fleet/agent_heartbeat`; no `limo_base`/launch/Gate/Nav2/Watchdog process; **no user of `/dev/ttylimo`, `/dev/ttyUSB0/1`** (checked in run 1 and run 2) |
| Residuals after the stops | 0 Agent / `ros2 run` processes, 0 ROS nodes; the 30-s warm-up participant ended by itself (it was still alive when the first check ran, 1 process) |
| DDS resources | shared-memory files: baseline 8 before; after the run only 2 remain, `fastrtps_port26665` and its semaphore from 20:10:46 (**erratum 2026-11-22 later: first attributed to the SIGKILL of the v3 Agent in Stage A; the timestamp equals the start of the `ros2` CLI daemon that my own diagnostic `ros2 topic list` created at 20:10:46, so the more likely origin is that daemon's participant; not v4**); the 6 others disappeared when I stopped that daemon. No new file is left by the v4 runs |
| Teardown | tunnel and server stopped (no 18290 listener on either side), token file deleted (config directory empty), installed Agent still == v4, logs preserved on the laptop |
**Overall: PASS, 0 FAIL, 0 SKIP.** No abort condition occurred. **Rollback: not needed**, v4 stays installed (v3 backup `~/backups/limo_robot_agent_*_pre_v4_20261122*`, copies on the laptop). Notes: the operating-mode view of the isolated server shows `FAULT`/`TRANSITIONING` after my mode requests (a server-side state of the probe, not an Agent fault); the first `ros2 topic list` use created a CLI daemon that I stopped at the end.
**Remaining defects/limits:** no robot sensors/Gate were running, so Gate/Depth/TF status fields were `NO_DATA` by design (fail-closed); a real Gate or stack was not tested with v4; the robot's tools copy of `start_robot_agent_field.sh` still has the old defaults (explicit arguments were used); the 2 orphaned shared-memory files from Stage A remain (not touched). Motion blockers unchanged.
