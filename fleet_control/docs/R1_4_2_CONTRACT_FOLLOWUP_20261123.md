# R1.4.2: two new contract defects from the Codex R1.4.1 audit (2026-11-23, Claude; isolated/mock verification only)

Not done: no real LIMO access/deploy/ROS, no 18190 change, no motor unlock, no commit/push/PR. R1.4.1 fixes are kept. **Not an approval of R4 or of unlocking the motors.**
Baseline checked: HEAD `c804241a`, G1 `0ca09dc4`, tracked-modified 9 (unchanged), `core.py` and `mapping.py` byte-identical to the Codex audit snapshot (`codex_r141_audit_20261123_b4bnJg`) before my edit; Codex edits (R1.4 server hardening, API.md R1.3) were kept. Records: `local_data/raw_results/real_limo_integ_claude_20261122/r1_4_2_01/`.

## 1. Causes
**A.** In `AgentCore.run`, `safety_contract_version` was copied from a state reply only `if 'safety_contract_version' in resp`, so a reply without the key kept the previous 1 and the old approval lived on (new goal reached `send_goal_async`; a live goal was not cancelled). Explicit null/False already blocked. Also, the R1.4.1 rule "a state `protocol_version` replaces the stored value" let a valid 2 in a state reply approve after an invalid hello.
**B.** In `mapping.worker_operation` the STOP-only branch required the boot but its `poll` ignored `session_id`, so a same-boot poll naming another session received the STOP body (accept and renew were already refused; no motion bypass seen, but it contradicted the session-bound contract).

## 2. Changes
| File | Function | Change |
|---|---|---|
| `agent/.../core.py` | `AgentCore.run` state-reply handling | `server_safety_contract_version = resp.get('safety_contract_version')` on every reply (missing -> `None` = invalid); `protocol_version` in a state reply is adopted only when it is NOT the exact int 2 (it can withdraw, never grant); import `SERVER_PROTOCOL_VERSION` |
| `server/fleet_server/mapping.py` | `worker_operation` (stop-only `poll`) | the STOP is returned only if boot AND `session_id` (a string equal to the session's id) match; otherwise `command: null`, `renew: false`. accept/result unchanged (still refused by the existing command/session matching) |
| new `tests/test_r1_4_2_contract_followup_20261123.py` | 40 test runs = 15 on state replies + 5 new on the STOP poll + 20 runs of the R1.4.1 `MappingStop` tests (imported class and the subclass), so 20 are new test bodies | see section 4 |
| `server/SAFETY_CONTRACT.md` | R1.4.2 section | documents the two rules |
`safety_protocol.py`, `ros_adapter.py`, Gate/Watchdog/Arbiter, the server wire format and Admission are unchanged. Protocol enforcement is not moved into state replies: its absence there stays normal.

## 3. State transitions
A before: reply1 `scv=1` -> reply2 (key absent) -> `scv` stays 1 -> `_on_command` -> `compute_movement_permitted` passes -> goal submitted; a live goal: `check_active_goal_safety` passes -> no cancel.
A after: reply2 (key absent) -> under the Agent lock `scv=None` before the commands of that reply are handled (`_on_command` runs after the lock block) -> a command of the same reply is refused (`SERVER_DISCONNECTED_OR_STALE`, terminal REJECTED, not replayed later) -> the active-goal monitor (`_monitor_active_goal`, called by the 0.1 s timers and right after submission) evaluates `check_active_goal_safety` and requests the cancel with `SERVER_CONTRACT_INVALID` on its next run. So invalidation and the cancel request happen in the same reply cycle only in the sense that invalidation precedes the next monitor tick (up to about 0.1 s later); they are not one atomic step. A later reply with a valid value makes the contract valid again for NEW commands; a goal already cancelled stays cancelled and the refused command is not replayed.
B before: stop-only `poll(session_id='wrong-session')` -> STOP body. B after: -> `command: null`, `renew: false`; the correct session still polls, accepts and reports normally.

## 4. Results (isolated/mock; plain Python without ROS unless marked)
| Check | Before (Codex R1.4.1 source) | After |
|---|---|---|
| Codex `edge.py` partial-state-empty | 1 goal | 0 goals |
| active-contract-missing | 1 goal, 0 cancels | 1 goal, 1 cancel, reason `SERVER_CONTRACT_INVALID` |
| invalid-hello-corrected-by-state | 1 goal | 0 goals |
| wrong-session STOP poll | STOP returned | no command |
| other edge cases (null, invalid protocol, explicit cancel, needs-reconcile, STOP round trip, single result) | as expected | unchanged |
| Codex `original-inputs.py` | - | all 7 version cases and contract/lease/boot cases 0 (valid case 1) |
| New `test_r1_4_2_contract_followup` | 6 FAIL | 40/40 PASS |
Test coverage: missing/null/True/False/1.0/'1'/0/2/-1 on a later reply (new goal 0), live goal cancelled for each invalid value (1 cancel request on the stub goal handle), valid goal not cancelled, withdrawal then valid reply, cancel then stale redelivery (no second submission), state reply cannot repair an invalid hello, state reply can withdraw a valid hello, absent/valid-2 protocol in state replies harmless, reconnect with/without a valid hello, lease expiry and epoch change; STOP: wrong/absent/non-string session, other boot, full round trip with one recorded result, forged ids, reserve_goal/renew, compatible worker unchanged. **A cancel request is observed on a stub; the real robot stopping is not shown.**
Regression: R1.4 hardening 9, R1.4.1 agent 11 and mapping 10, Agent-Server contract 11, `test_agent_safety_v1` 17 (also with ROS sourced), Fleet loopback E2E `test_server_agent_safety_e2e` 16, `test_phase20_agent_stub` 7, r8 reconcile/supervisor 1+4, `test_recovery` 14, `test_phase20r_recovery_v1` 14, `test_watchdog_admission` 4, Mapping server `test_web_mapping_session` 26 (7 skipped by the test itself), `test_slam_sessions` 4, gate-reset server 14, safety-context suites, `test_dedupe` 14/14: all PASS. Isolated ROS (real rclpy, domain 187, localhost only, `unshare -Urn`): `test_ros2_isolated_action` and `test_ros2_late_goal_reconciliation` OK (a valid Agent still submits to the mock Nav2 action server). Not run: `tests/web_mapping_isolated/*` (needs the full SLAM/frontier stack) and the Agent-side Mapping Worker against a real ROS graph.
Full `unittest discover` (no ROS): 909 run, 2 FAIL, 33 ERROR, 17 SKIP (earlier R1.4.1: 2 FAIL, 34 ERROR); the failing set is unchanged and not converted to PASS: 28 import failures (ROS packages missing or the private-namespace guard), 5 `test_gate_reset_worker` (`std_srvs`, one of them a FAIL), `test_agent_shutdown_20261122` (SETUP_FAIL), `test_phase45_qr_libzbar` (env variable). Per-test comparison with the R1.4.1 run: no test changed from ok to failing (two names that looked like regressions are multi-line name parsing artefacts; the modules pass). `test_watchdog_admission` did not flake in this run; the P2 stub-cache problem and the Watchdog timing flake remain open and are unrelated to these two edits (the first lives in the test fixture module lifecycle, the second in the HTTP/AgentCore timing of that suite).

## 5. Agent v5 impact
`core.py` changes (run loop): Agent-only, needs the v5 package and the isolated robot verification procedure; not done. Compatibility: the Server's hello/state format is unchanged and it always sends `safety_contract_version`, so a v5 Agent against the current Server behaves as before in normal operation; against a Server that omits that key it now refuses movement (intended, fail-closed). The `mapping.py` change is Server-only; v4 Agents (compatible contract) are unaffected.

## 6. Still blocking R4 (unchanged by this work)
Real ROS/DDS STOP delivery and physical stopping, Watchdog/Gate/Arbiter behaviour under lost worker/contract, Arbiter/bench/Gate-reset/localization execution actors applying the contract directly (Codex lists the added coverage; confirm in its next audit), Gate r13 and real Arbiter/Watchdog application, `/cmd_vel` bypass, Depth, power/UART/STM32 timeouts, physical E-Stop and braking distance, Agent v5 deployment.

## 7. Re-audit commands for Codex
```bash
cd <repo>/fleet_control/tests          # ROS not sourced
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=../server:../agent/limo_robot_agent:. python3 -m unittest -v test_r1_4_2_contract_followup_20261123 test_r1_4_1_agent_protocol_20261123 test_r1_4_1_mapping_stop_20261123
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=../server:../agent/limo_robot_agent:. python3 ../../local_data/raw_results/codex_r141_audit_20261123_b4bnJg/edge.py        # compare with edge_before_codex.txt
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=../server:../agent/limo_robot_agent:. python3 ../../local_data/raw_results/codex_r141_audit_20261123_b4bnJg/original-inputs.py
# isolated ROS: source Humble, ROS_LOCALHOST_ONLY=1, an unused ROS_DOMAIN_ID, unset ROS_DISCOVERY_SERVER
CAPSTONE_PRIVATE_ROS_TEST=1 unshare -Urn bash -c 'ip link set lo up; python3 -m unittest test_ros2_isolated_action test_ros2_late_goal_reconciliation'
```
To see the tests detect the defects: unpack the Codex R1.4.1 `snapshot.tar.gz`, copy only `test_r1_4_2_contract_followup_20261123.py` in and run the first command: 6 failures.
