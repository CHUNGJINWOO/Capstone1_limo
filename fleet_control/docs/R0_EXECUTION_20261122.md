# R0 execution report: originals preserved, Fleet snapshot taken, Git candidates fixed (2026-11-22, Claude)

Follows `R0_BASELINE_20261122.md` and the user's R0 execution approval. **Not done, as instructed:** no commit (G1 not executed), no new worktree, no branch change, no `git clean`/`reset`/`checkout`/`stash`, no push/PR, no change to tracked files or to any other agent's files, no robot ROS stack, Gate, Nav2 or motor, no robot file moved, deleted or modified.
Records: `local_data/raw_results/real_limo_integ_claude_20261122/{r0_copy_01,r0_baseline_01}/` (`run_guard` MANIFEST/FINISH, manifests, per-item results, logs). Tool: `fleet_control/scripts/robot_originals_copy_20261122.py`.

## 1. Robot originals (read-only on the robot) -> laptop
Destination: `local_data/robot_originals_20261122/` (new, git-ignored, 7.7 GB, 36 completed items, each with a `<item>.COMPLETE.json` marker). Layout mirrors the robot: `limo-real-auto-mapping__<run>`, `limo_locked_stack__<run>`, `state__<dir>`, `home__maps_h107_20261120`.
Mechanism: manifest of every file (size, SHA256) streamed from the robot with a read-only python helper (nothing written on the robot); `rsync -a --partial` (+ zstd compression after measuring 2 MB/s and a 7.6x compression ratio) into `.partial/`; verification; only then a rename to the final name and the COMPLETE marker. An interrupted run resumes from `.partial/` (it was interrupted once on purpose to add compression and resumed without losing data); finished items are skipped; a final directory that exists without a marker is never touched.
| Result | Items | Detail |
|---|---|---|
| **PASS (copied and verified)** | 36 | 871 files, 8,233,753,902 bytes; per file: same relative path, same size, same SHA256 as the robot manifest; no missing, no extra files; file count and total bytes equal. Independently **re-verified from the laptop copy against the stored manifests after the run: 36/36 PASS** (`reverify_complete_items.json`) |
| **REUSED_VERIFIED (no copy needed)** | 9 | every file of the item already exists on the laptop with an identical SHA256 (mapping robot path -> laptop path in `r0_copy_01/run/reuse/`): `20261118_run_001`, `20261118_run_002`, `20261118_base_recovery_diag`, `20261118_joystick_recovery_diag`, `limo_locked_stack/{20261120_085938, 090230, 090640, 110852}`, `maps_h107_20261120`. 64 files |
| **Single file** | 1 | `limo_locked_stack/latest` is a 63-byte pointer file, not a directory: the tool reported FAIL for it by design; copied separately to `limo_locked_stack__latest.file`, SHA256 equal on both sides |
| **FAIL** | 0 remaining | (the `latest` case above was a tool classification, not a data problem) |
Robot side after the copy: 935 files in the copied state directories, **0 missing, 0 size differences** against the manifests: the originals are unchanged and still on the robot. Nothing was moved or deleted there.
Scope copied: all of `limo-real-auto-mapping/` (11-13 runs 001-021, 11-18 runs and recovery/diagnostic directories, 11-19 runs), `limo_locked_stack/`, the field Agent state directories (`limo_robot_agent_field_20261120`, `limo_robot_agent_field`, `limo_robot_agent`), `limo-depth-observe`, `limo_field_cam`, `maps_h107_20261120`.
**Not copied (classified separately):** `~/.config/limo_*` (field and isolated tokens, `field.env`), `~/limo_video_kit/real_limo_kit/secrets`, shell history, other home data. A scan of every copied text file for the words token/secret/password/bearer/authorization found one file: `limo_robot_agent_field_20261120/agent.out`, whose only hit is an `rcl` argument-parser message ("Expecting token or wildcard"), not a credential; no secret-like file names. `ros_logs` and `events.jsonl` content was not searched for credential values beyond that keyword scan.

### 1.1 rosbag integrity (23 bags)
| Check | Result |
|---|---|
| SQLite `PRAGMA integrity_check` and `SELECT count(*) FROM messages` on every `.db3` (read-only; a scratch copy is used when a `-wal` exists) | **22 PASS**, **1 BAG_INTEGRITY_FAIL** |
| Messages counted by SQLite over all 23 bags | 4,032,544 (the malformed bag counts 0) |
| `ros2 bag info` | rc 0 for all 15 bags that have `metadata.yaml`; message count equals `metadata.yaml` and the SQLite count |
| Bags without `metadata.yaml` (the recorder was killed): `20261113_run_003/013/014/015/021`, `20261118_run_004`, `20261119_run_001`, `20261119_run_002` | SQLite checks only; 7 PASS |
| **`20261119_run_002/bag/bag_0.db3`: malformed in the ORIGINAL** | The robot's file is byte-identical to the copy (same SHA256). The SQLite header declares 83,047 pages (340,160,512 bytes) but the file has 81,455 pages (333,639,680 bytes): **6,520,832 bytes are missing at the end** (the recorder was killed during the 11-19 safety stop; there is no `metadata.yaml`). Not a copy error. Salvage (`.recover`) was not attempted: `sqlite3` is not installed on the laptop, and the copy must stay untouched; a salvage on a scratch copy is a possible follow-up |
| `20261118_run_004` (recorder died on a SQLite lock, 11-18) | original and `_bag_recovered_copy` both pass the SQLite check |

## 2. Fleet source snapshot
`local_data/fleet_control_snapshot_20261122/` (new, 11 MB): taken from the working tree at 21:05; archives + SHA256 per file; **restore test passed**.
| Part | Content | Verification |
|---|---|---|
| `fleet_control_source.tar.gz` | 610 files = every file under `fleet_control/` that git would add (untracked, not ignored) | tar hash OK; extracted to a scratch directory: 610/610 files equal the hash list; live tree still equals the list; 29 smoke tests (`test_real_locked_policy`, `test_map_save`, `test_phase4_readiness`) pass from the restored copy |
| `fleet_control_ignored_evidence.tar.gz` | 33 git-ignored but useful files (dashboard evidence JSON/screenshots) | 33/33 |
| `common_docs_and_owner_files.tar.gz` + `owner_uncommitted_tracked_changes.patch` | the 8 untracked common documents, the 9 tracked files other parties modified (copies + a `git diff --binary` patch that applies in reverse cleanly), 3 untracked owner source files (`integrated_nav2_depth_slow_params.yaml`, `start_locked_standby.py`, `fleet_frontier_candidates.cpp`) | 20/20; copies only, the originals are untouched |
| `SENSITIVE_NOT_COPIED.tsv` | 7 entries (5 token files, `data/auth.json`, `data/fleet.db`) with size and mode only: **no content, no hashes** | – |
| `PYCACHE_EXCLUDED.txt` | 148 generated `__pycache__` files | – |
**Count explanation (the 647 / 607 / 188 of the R0 baseline):**
- 647 = files under `fleet_control/` excluding generated `__pycache__`/`.pyc`: 607 git-addable + 40 ignored non-generated (7 sensitive + 33 evidence/other). Three new files were created since (this report, the common-docs proposal, the copy tool), so the same count today is **650 = 610 + 7 + 33**.
- 607 (now 610) = untracked files not covered by any ignore rule = what `git add fleet_control` would stage.
- 188 = files `git status --ignored` hides in `fleet_control/`: 148 `__pycache__` + 7 sensitive + 33 evidence/other. They are never untracked-visible, which is why 647 and 607 differ by 40 and not by 188.
Documents written after 21:05 (this report, status entries, the copy tool's compression edit) are not in the first snapshot, so a **second snapshot** was made with `fleet_control/scripts/fleet_snapshot_20261122.py`: `local_data/fleet_control_snapshot_20261122_02/` (612 source files, 33 evidence, same restore test: tar hashes OK, 612/612 restored files equal the list, live tree equals the list). The first snapshot is kept unchanged and still verifies.

### 2.1 Where to keep a second copy (proposal; nothing done)
`local_data/` is git-ignored and sits in the same repository directory: a cleanup of the repository, a `git clean -x`, or the 14 GB `local_data` housekeeping would remove both the originals and the snapshots. Proposed (needs the user): (a) a copy on a different physical medium (external disk) of `robot_originals_20261122/` (7.7 GB), the snapshot and the manifests; (b) the encrypted private Google Drive procedure already fixed in `VALIDATION.md` §2.5 (GPG AES-256 with an interactive passphrase, SHA256 list, remote hash check); the assistant cannot type the passphrase; (c) mark `local_data/robot_originals_20261122` and `local_data/fleet_control_snapshot_*` as "never clean" in `WORKTREES.md`.

## 3. Git readiness (G1 not executed)
Files: `r0_baseline_01/run/git_final_candidates.tsv` (610 rows: path, bytes, relation to `feat/fleet-control-unified`, owner inference, decision, note).
| Decision | Files | Meaning |
|---|---|---|
| INCLUDE | 588 | code, tests, scripts, docs, 76 UI screenshots |
| INCLUDE_RECORD | 15 | dated reports with an absolute `/home/...` path inside the text (28 occurrences in 16 files before the later edits): 15 of the occurrences are the path of another tool's runtime in the dashboard documents, 6 the laptop workspace path, 7 robot paths. PUBLICATION_POLICY allows past environment paths in records; optional normalisation to `<WORKSPACE_PARENT>` at commit time |
| REDACT_BEFORE_COMMIT | 5 | `LIMO_DIRECTORY_AUDIT`, `R0_BASELINE`, `REAL_LIMO_FLEET_INTEGRATION_PLAN`, `REAL_LIMO_INTEGRATION_SURVEY` (tailnet IPs of the laptop and the robot, `env.sh` Discovery Server address) and the copy tool (default host): replace by `<LAPTOP_TAILNET_IP>`/`<ROBOT_TAILNET_IP>` or keep out of the first commit |
| FIX_BEFORE_COMMIT | 1 | `scripts/start_robot_agent_field.sh`: its default `SERVER_URL` is the laptop tailnet IP with the operational port; make it a required argument |
| HOLD | 1 | `dashboard/docs/screenshots/live-api.jpg`: a dimmed real camera frame of a room with a faint detected-"person" label; exclude or blur (owner/user decision) |
Image review (all 77 viewed in four contact sheets; all are UI screenshots): 70 PNG show the synthetic fixture floor plan; the 7 JPEG are dashboard captures, of which `actual-rgb-yolo.jpg` shows a real camera frame of a wall (no person), `actual-map-depth.jpg`/`gateway-offline.jpg` show a real LiDAR map (room outline only), `ros-stale.jpg`/`test-*.jpg` show a Gazebo scene with a simulated person model. No EXIF in the 14 JPEGs.
Secrets: literal values of all 9 known token files in the 610 candidates: 0; e-mail strings: only `example.com`; no student-number pattern. Token files, `auth.json`, `fleet.db`, rosbags and robot originals stay out of git (`fleet_control/.gitignore`, root ignore file, `*.db3`).
Owners (inference from HANDOFF and dates): Agent/Server/tests/scripts/docs = Claude sessions with Codex defect fixes of 11-20/21 (the Gazebo harness `tests/gazebo_web/*` by the Nav2 session); `rmf_adapter` = the Codex RMF line; `dashboard*`, `integrated_sim`, `gazebo`, `maps`, `rmf_web_research` = earlier sessions, unchanged since 11-14. No candidate touches a tracked file; the owners' uncommitted files (Gate, real launch, depth filter, `package.xml`, `run_recorder.py`, `explore/CMakeLists.txt`, `AGENTS.md`, `README.md`, design doc) are not candidates.
Relation to `feat/fleet-control-unified`: 153 of the candidates equal it, 22 differ (Agent `agent_node`, `core`, `ros_adapter`, `package.xml`, `setup.py`; Server `__main__`, `db`, `http_api`, `ownership`, `recovery`, `service`, 4 static UI files; 2 docs; 5 tests), the rest are new; unified is a 16-commit-older ancestor line of the working tree; `origin/main` carries a parallel line (17 files absent here, 35 differing) for G2.
Identity: the last three commits use a GitHub noreply address (read it at execution time with `git log -1 --format=%ae HEAD`); the local git config holds a personal address, so G1 must pass the noreply identity explicitly and be checked with `git log --format='%ae %ce'` before any publication.
**Final candidate set for G1: 609 files** = 610 candidates minus the 1 HOLD file: 603 as they are (INCLUDE 588 + INCLUDE_RECORD 15) and 6 only after the edits above (5 REDACT + 1 FIX). G1 is not executed.

## 4. Remaining items and next approvals
| Item | State |
|---|---|
| Robot originals | all copied or reused; one damaged original bag (`20261119_run_002`, 6.5 MB truncated) kept byte-identical |
| Uncopied | `~/.config/limo_*` (secrets) and other non-run data: not in scope; `limo_video_kit` and legacy workspaces unchanged |
| Second storage location | proposal only (§2.1) |
| G1 plumbing snapshot | waits for: the decisions on the 1 HOLD, 5 REDACT and 1 FIX files; approval of the noreply identity use |
| G2 new worktree for `origin/main` | not created |
| Salvage of `20261119_run_002` | optional, on a scratch copy |
| Common documents | existing proposal kept; this step adds status entries only |
