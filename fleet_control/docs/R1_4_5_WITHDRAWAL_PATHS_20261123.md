# R1.4.5: one withdrawal rule for every authority change; strict reconciliation flag (2026-11-23, Claude; isolated/mock only)

Not done: no real LIMO/ROS/deploy, no 18190 change, no motor unlock, no commit/push/PR. R1.4.1-R1.4.4 and the Mapping STOP session check kept. **Not an approval of Agent v5 deployment, R4 or unlocking the motors. R1 is not closed before Codex re-verifies.**
Baseline: HEAD `c804241a`, G1 `0ca09dc4`, tracked-modified 9; `core.py`, `safety_protocol.py`, `ros_adapter.py`, `service.py`, `recovery.py` byte-identical to the Codex R1.4.4 snapshot (`codex_r144_audit_20261123_6z7wix`) before editing. Records: `local_data/raw_results/real_limo_integ_claude_20261122/r1_4_5_01/`.

## 1. Root causes
**P1-A.** The two failure branches of `AgentCore.run` (`except HttpError`, `except Exception`) set `connected=False` and dropped lease/mode/requests under `AgentCore.lock` only. They neither took `contract_fence` nor counted as a generation change (`connected` was not part of the authority), so a 503 arriving after the Stage 3 check was applied without waiting and the old goal was still submitted; only the later disconnect cancel caught it.
**P1-B.** The Agent read `resp.get('reconciliation_required') is True`, so any value other than the boolean True (missing, null, 0, 1, strings, lists, objects) became False and restored the authority. The server's normal state reply (`_deliver_atomic`) omitted the key altogether, so "missing" was the normal wire format and could not be refused without a server change.

## 2. Changes
| File | Function | Change |
|---|---|---|
| `agent/.../core.py` | `_contract_ok_locked` | authority additionally requires `connected is True` |
| | `run`, both failure branches | `with contract_fence, AgentCore.lock`; `_contract_note_locked()` after dropping the session (generation +1) |
| | `run`, state reply | `server_reconciliation_required = resp.get('reconciliation_required') is not False` (only the exact boolean False is "none pending") |
| | `_hello`, final block | `connected = True` before `_contract_note_locked()` (order fix so the note sees the final state) |
| `server/fleet_server/service.py` | `_deliver_atomic` normal return | adds `'reconciliation_required': False` (the blocked return already had an explicit boolean) |
| tests | `test_r1_4_1_agent_protocol` (fake state reply), `test_dedupe` (fake state reply), `test_phase20_fail_closed`, `test_phase20_agent_sigkill`, `test_r1_4_3` unit (direct sessions) | fixtures follow the new wire (explicit False) and model a connected session; no assertion changed |
| new | `tests/test_r1_4_5_withdrawal_paths_20261123.py` | 13 tests |
`ros_adapter.py` and `safety_protocol.py` unchanged. No new queue or state machine.

## 3. Survey of authority fields (all writers; readers in brackets)
| Field | Writer (thread) | Locks held | Fence | Generation | Queued commands | Live goal |
|---|---|---|---|---|---|---|
| `connected` | `_hello` reset/finish, mismatch branch, failure branches (run loop) | fence + AgentCore.lock (failure branches: new) | yes (all) | +1 on loss (new) | refused (generation) | cancel (generation) |
| `server_boot` | `_hello` reply (run loop) | fence + lock | yes | via the reset before it | refused (generation, command server boot) | cancel |
| `lease` | state reply; dropped in mismatch/failure/hello reset (run loop) | fence + lock | yes | no own bump (dropping it coincides with a connected/contract bump) | per command `LEASE_MISMATCH` / `LEASE_MISSING_OR_EXPIRED` | `LEASE_REVOKED_OR_EXPIRED` |
| `server_reconciliation_required` | reconcile result in `_hello`, state reply (run loop) | fence + lock | yes | +1 on False->not-False | refused | cancel |
| `server_protocol_version` / `_boot` | `_hello` reset/reply, state reply withdrawal, mismatch branch | fence + lock | yes | +1 when invalid | refused | cancel |
| `server_safety_contract_version` | `_hello`, every state reply, mismatch branch | fence + lock | yes | +1 when invalid | refused | cancel |
| `contract_generation` | `_contract_note_locked` only (called in every block above) | fence + lock | yes | - | [read at receipt in `_on_command` and in Stage 1-3] | [read by the monitor] |
| `RosRobot.active` and its cancel fields | `_do_nav` Stage 3, goal response/result/feedback callbacks, `_monitor_active_goal`, `cancel_mission` (ROS executor / tasks) | RosRobot.lock (Stage 3 also the fence) | Stage 3 only | - | - | the cancel duty itself; these are consequences, not authority changes |
Other readers (`velocity_arbiter`, `manual_web_bench`, `gate_reset_worker`, `localization_worker`) only read `session_snapshot()` / `is_execution_allowed()`; they do not write authority fields and are not bound to the generation (unchanged; Codex observation, a separate proposal). After this change no authority writer remains outside the fence; no path takes the fence while holding `RosRobot.lock` or `AgentCore.lock`.

## 4. Fence and lock order (unchanged rule)
`contract_fence -> RosRobot.lock -> AgentCore.lock`. Held only for assignments (run loop) and for Stage 3 check + start of `send_goal_async`; no network I/O, Future wait or ROS callback wait inside. The failure branches release the fence before the retry back-off wait.
Behaviour change to note: because a lost session now bumps the generation, a live goal is cancelled on the first monitor run after an HTTP failure instead of after the old `disconnect_cancel_after_s` grace (3 s). This is the more conservative choice required by "every withdrawal keeps the cancel duty"; a transient network error therefore ends the current goal.

## 5. Wire contract for reconciliation_required
| Reply | Before | After |
|---|---|---|
| reconcile (`recovery.py`) | explicit boolean | unchanged |
| state, blocked (`needs_reconcile` or server-side reconciliation) | explicit boolean | unchanged |
| state, normal | key absent | explicit `False` |
| hello | not carried (the Agent never read it there) | unchanged |
| error replies | not carried (handled by the failure branch) | unchanged |
Agent rule: only the exact boolean `False` grants; `True`, absent, null, numbers (including 0 and 0.0), strings, lists, objects all withdraw. No contract-version bump: Agent v4 reads `is True`, so the extra explicit `False` changes nothing for v4 (backward compatible); a v5 Agent against a server without this change refuses motion (fail-closed, not unsafe). Deployment order: Server first, then Agent v5. Not applied to 18190.

## 6. Deterministic tests (`test_r1_4_5_withdrawal_paths`, 13 tests, 3 consecutive runs PASS; on the R1.4.4 source 23 failures incl. subtests)
| | Result |
|---|---|
| A HTTP 503 / exception before Stage 3 | 0 submissions, REJECTED, generation 0->1 |
| B released while Stage 3 holds the fence | the failure branch is observed WAITING at the fence; at the send instant `connected` and `lease` are still set and the generation is 0 (Codex: already False/None); afterwards generation 1, 1 submission, 1 cancel request |
| C error after the send; C2 across the reconnect | 1 submission, 1 cancel, reason `SERVER_CONTRACT_INVALID`, kept after the re-hello |
| D error -> reconnect, queued command | 0 submissions, REJECTED; D2 still offline: cancel kept |
| E True -> exact False | a new goal runs |
| F/G True -> missing/null/0/1/'false'/''/[]/{}/0.0, and the same values after a valid session | 0 submissions, REJECTED, local flag True |
| H malformed -> valid with redelivery | old command REJECTED, a new command runs |
| I lease epoch change | refused |
| server wire | real `Fleet.ingest_state` replies: False, True (server-side block), False; the Agent follows them and regains authority only on the exact False |
| J 500 replies with HTTP errors, connection resets, contract/reconciliation flaps, a ROS thread submitting/monitoring/resolving/cancelling, shutdown | no deadlock (join with timeout as detector), no exception, and no command received under a withdrawn authority was ever submitted |
Codex probes: `disconnect.py` and the reconciliation part of `edge144.py` relied on the old behaviour (`disconnect.py` waits for the error to finish INSIDE Stage 3, impossible now, ends with its own `IndexError`; `edge144.py` asserts the omitted server key). With its assertions removed (`edge144_noassert.py`), all 7 malformed/missing flags give 0 submissions (Codex: 1 each). `interleave.py` (R1.4.2) unchanged.

## 7. Regression
All PASS: R1.4 9, R1.4.1 11 + Mapping STOP 10, R1.4.2 40 (includes Mapping STOP poll/accept/result again), R1.4.3 15, R1.4.4 12, contract 11, `test_agent_safety_v1` 17 (also ROS sourced), loopback E2E 16 (real server replies through the Agent), phase20 stub 7, SIGKILL 2, fail-closed 5, r8 1 + 4, recovery 14 + 14, watchdog admission 4, mapping server 26 (7 test-skips), slam sessions 4, gate reset server 14, gate pending/sequences 6 + 7, terminal order 2, safety-context regression 24, r3/r4 2, lease contract 2, operating mode 11, dedupe 14/14. Isolated ROS (real rclpy, domain 187, localhost only, `unshare -Urn`): `test_ros2_isolated_action`, `test_ros2_late_goal_reconciliation`, `test_agent_shutdown_20261122` (SIGINT/SIGTERM/disconnect-reconnect matrix) OK.
Full `unittest discover` (no ROS): 949 run, 2 FAIL, 33 ERROR, 17 SKIP; failing set unchanged and not converted: 28 import failures (ROS packages absent / private-namespace guard), 5 `test_gate_reset_worker` (`std_srvs`; one FAIL), `test_agent_shutdown_20261122` SETUP_FAIL in the plain shell (it passes in the ROS private namespace above: environment), `test_phase45_qr_libzbar` (environment variable). No new regression: four names flagged by the log parser pass individually (interleaved warning lines).

## 8. R1 closure assessment
Conditions met in isolation: HTTP failure paths no longer bypass the fence; malformed reconciliation values never restore authority; the authority-field survey and lock/generation rule are documented (section 3); all earlier P1 counterexamples stay blocked; normal Agent and STOP/Cancel work; deterministic concurrency tests and the core regression pass. **R1 can be proposed for closure only after Codex re-verifies.** Open but outside R1.4.5: the other execution actors (Arbiter, manual bench, Gate-reset worker, localization worker) are not bound to the generation; stale same-boot response ordering has no sequence check; P2 stub cache; Watchdog admission timing flake.
R2/R3 (offline): may proceed. R4 (motor-locked real integration) stays blocked by: Codex re-verification, Agent v5 package + deployment approval, the server change deployed first, real locked-stack verification of STOP/Arbiter/Watchdog/Gate and Mapping STOP; motor unlock additionally needs Gate r13, `/cmd_vel` bypass removal, Depth, power/USB-UART/STM32 timeouts, physical E-Stop and braking distance.

## 9. Re-audit
```bash
cd <repo>/fleet_control/tests          # ROS not sourced
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=../server:../agent/limo_robot_agent:. python3 -W ignore -m unittest -v test_r1_4_5_withdrawal_paths_20261123 test_r1_4_4_submission_fence_20261123 test_r1_4_3_contract_generation_20261123 test_r1_4_2_contract_followup_20261123 test_r1_4_1_agent_protocol_20261123 test_r1_4_1_mapping_stop_20261123 test_r1_4_safety_contract_hardening_20261123 test_agent_server_contract_20261123 test_phase20_agent_sigkill test_phase20_fail_closed test_server_agent_safety_e2e
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=../server:../agent/limo_robot_agent:. python3 -W ignore ../../local_data/raw_results/real_limo_integ_claude_20261122/r1_4_5_01/edge144_noassert.py
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=../server:../agent/limo_robot_agent:. python3 test_dedupe.py
# isolated ROS: source Humble, ROS_LOCALHOST_ONLY=1, unused ROS_DOMAIN_ID, unset ROS_DISCOVERY_SERVER
CAPSTONE_PRIVATE_ROS_TEST=1 unshare -Urn bash -c 'ip link set lo up; python3 -m unittest test_ros2_isolated_action test_ros2_late_goal_reconciliation test_agent_shutdown_20261122'
```
Pre-fix comparison: unpack the Codex R1.4.4 `snapshot.tar.gz`, copy in `test_r1_4_5_withdrawal_paths_20261123.py`, `test_r1_4_4_submission_fence_20261123.py`, `test_agent_safety_v1.py`, run that file: 23 failures (including subtests).
