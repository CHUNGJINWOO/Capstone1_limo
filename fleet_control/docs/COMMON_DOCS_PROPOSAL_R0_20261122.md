# Proposed changes to the common documents (R0, 2026-11-22; NOT applied)
Reason for a separate proposal: `AGENTS.md` has an uncommitted change by another party (+21 lines); `WORKTREES.md`, `CODE_INDEX.md`, `START_HERE.md`, `ARCHITECTURE.md` are untracked on this branch and shared by several sessions.

## WORKTREES.md (append under the 2026-11-19 `robot_ws` section)
- Robot environment (Verified 2026-11-22): host `pcu`, Ubuntu 22.04.5, ROS 2 Humble. The only workspace we own is `/home/pcu/Capstone1_limo/robot_ws` (12 packages, not a git repository; equals the laptop sources except `limo_capstone` launch/Gate/`package.xml` (owner changes) and `limo_robot_agent/agent_node.py` (robot v3, laptop v4)). `~/limo_field_agent_ws` named in earlier entries **does not exist**.
- Created 2026-11-22 (Stage A, user-approved): `~/backups/` (Agent backups, rollback source), `~/stage_a_v3/` (staging copy of v3), `~/limo_isolated_18290_tools/` (temporary tools), `~/.local/state/limo_robot_agent_isolated_18290/` (Stage A Agent state), `~/.config/limo_isolated_18290/` (empty). Keep until released by the user.
- Robot originals to be copied to the laptop (R0): `~/.local/state/limo-real-auto-mapping/` (8.2 GB); until verified copies exist, treat them as the only originals.

## CODE_INDEX.md
- Agent: `fleet_control/agent/limo_robot_agent` (robot: `robot_ws/src/limo_robot_agent`, v3 `agent_node.py` `94ade245…`; laptop v4 `6a8252da…`, not deployed). Replace the `~/limo_field_agent_ws` references.
- `fleet_control/` is **untracked** on `staging/fleet-control-on-progress-c804241`; the tracked snapshots are `feat/fleet-control-unified` (203 files) and `origin/main` (197 files, parallel line).
- New this week: `fleet_control/docs/{R0_BASELINE,LIMO_REFACTORING_DESIGN,LIMO_DIRECTORY_AUDIT,REAL_LIMO_*,GAZEBO_*}_20261122.md`, `fleet_control/tests/{run_depth_targets…,agent_shutdown_matrix_20261122.py,test_agent_shutdown_20261122.py,test_real_mode_agent_blocks_20261122.py,real_agent_refusals_probe.py}`.

## AGENTS.md (do not edit while its working-tree change is pending; text for the owner)
- §15: "robot paths are documented only in WORKTREES.md; code must not hard-code them".
- §17 (identity): "commits must use a GitHub noreply author and committer e-mail; the local git config may hold a personal address, so pass the identity explicitly per commit and verify with `git log --format='%ae %ce'`".
- New §19 "Working on the LIMO": read `LIMO_DIRECTORY_AUDIT_20261122.md` §7 first; never use the `~/.bashrc` aliases (`stop` publishes `/cmd_vel`; `base`, `nav2`, `auto_map`, `explore`, `nav_test` start motors or navigation); use `env_local77.sh` only; no deletion or move without per-item approval.

## START_HERE.md
- One line in §1: "LIMO work: `fleet_control/docs/LIMO_DIRECTORY_AUDIT_20261122.md` §7 and `R0_BASELINE_20261122.md` first."

## DECISIONS.md, ARCHITECTURE.md
- No change (no approved decision; the design document stays a proposal).

## Addendum after the R0 execution (still NOT applied)
- WORKTREES.md / CODE_INDEX.md: register `local_data/robot_originals_20261122/` (robot originals of 11-13, 11-18, 11-19; 36 verified copies + 9 verified reuses; **never clean or move**), `local_data/fleet_control_snapshot_20261122{,_02}/` (file snapshots; **never clean**), the tools `fleet_control/scripts/{robot_originals_copy_20261122.py,fleet_snapshot_20261122.py}`, and the record directories `r0_baseline_01`, `r0_copy_01`.
- VALIDATION.md: add the R0 execution table (36 items 8,233,753,902 B re-verified, 22/23 bags PASS, one damaged original bag `20261119_run_002`).
- Policy text for WORKTREES: "a backup that lives under `local_data/` inside the repository directory is not a second copy".
