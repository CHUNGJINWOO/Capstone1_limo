# R1.4.4: atomic final check + submission start, reconciliation as authority withdrawal (2026-11-23, Claude; isolated/mock only)

Not done: no real LIMO/ROS/deploy, no 18190 change, no motor unlock, no commit/push/PR. R1.4.1-R1.4.3 fixes kept. **Not an approval of R4 or of unlocking the motors.**
Baseline: HEAD `c804241a`, G1 `0ca09dc4`, tracked-modified 9; `core.py`, `safety_protocol.py`, `ros_adapter.py` byte-identical to the Codex R1.4.3 snapshot (`codex_r143_audit_20261123_BqbR1o`) before editing. Records: `local_data/raw_results/real_limo_integ_claude_20261122/r1_4_4_01/`.

## 1. Root causes
**P1-A.** `_do_nav` Stage 3 held only `RosRobot.lock`; the Agent run loop changed the contract under the separate `AgentCore.lock`. The Stage 3 check reads the session through `session_fn` (taking `AgentCore.lock` briefly), so the run loop could invalidate after that read returned and before `send_goal_async` was called: the old-generation goal was submitted (only the later cancel duty caught it). Existing lock order: `RosRobot.lock -> AgentCore.lock` (Stage 3 and the monitor call `session_fn` while holding the ROS lock); the run loop never takes the ROS lock, which is why simply taking `RosRobot.lock` in the run loop would invert the order.
**P1-B.** `_contract_ok_locked` looked only at the two versions. `reconciliation_required` blocked execution only indirectly (`is_execution_allowed` -> `connected=False` in `_session_context`), and that block disappeared again on the next `False` reply. A `False -> True -> False` sequence therefore left the generation unchanged: a queued command executed and a live goal was cancelled only if the monitor happened to run during the `True` window.

## 2. Changes
| File | Function | Change |
|---|---|---|
| `agent/.../core.py` | `AgentCore.__init__` | `contract_fence = threading.RLock()` (the shared serialization boundary) |
| | `_contract_ok_locked` | execution authority = valid contract **and** `server_reconciliation_required is False` |
| | `_hello`, state-reply handling in `run` (normal and boot-mismatch/`needs_reconcile` branches) | every block that changes contract or reconciliation fields runs under `contract_fence` then `AgentCore.lock`; hello now also assigns `server_boot`/`safety_contract_version` inside that block; the final hello block (reconcile result) calls `_contract_note_locked` |
| `agent/.../ros_adapter.py` | `_submission_fence` (new), `_do_nav` Stage 3 | Stage 3 = `with fence, RosRobot.lock: check; send_goal_async(...)`; the fence is the AgentCore's when `session_fn` is bound to an AgentCore (production wiring and the AgentCore tests), otherwise a private lock (stand-alone robot tests) |
| tests | `test_agent_safety_v1.py` (test 8 fake reconcile reply), `test_phase20_fail_closed.py`, `test_phase20_agent_sigkill.py`, `test_r1_4_3...` unit test | fixtures now state `reconciliation_required: False`, as the real reconcile/state replies always do (`recovery.py:78`, `service.py:360`); no assertion changed |
| new | `tests/test_r1_4_4_submission_fence_20261123.py` | 12 tests (A-J) |
`safety_protocol.py` unchanged in this step (the generation checks of R1.4.3 already cover execution and the active goal). No new queue, no new state machine.

## 3. Serialization boundary and lock order
Order everywhere: `contract_fence -> RosRobot.lock -> AgentCore.lock`. Holders of the fence: the run loop while it changes authority fields (a few assignments, no I/O, no wait), and `_do_nav` Stage 3 from the final check to the return of `send_goal_async` (a local call that returns a Future; the Future, goal response, feedback, result and cancel are handled outside the fence). ROS callbacks (`_on_goal_response`, result, feedback), `_monitor_active_goal`, `cancel_mission`, heartbeat and `stop()` never take the fence. Nothing holding `AgentCore.lock` or `RosRobot.lock` acquires the fence, so no cycle exists.
**Linearization:** whichever side enters the fence first is ordered first. Invalidation first: Stage 3 sees the new generation/withdrawn authority and refuses (`CONTRACT_GENERATION_CHANGED` or the existing session reasons), 0 submissions. Stage 3 first: the check and the start of `send_goal_async` complete, then the invalidation proceeds and the goal record (stamped with the old generation) carries a latched cancel duty; a late goal handle is cancelled when it arrives. There is no third order.

## 4. Where reconciliation withdraws authority
`run`: `self.server_reconciliation_required = resp.get('reconciliation_required') is True` followed by `_contract_note_locked()` in the same fenced block, before the reply's commands are handled. A `False -> True` reply is a valid->invalid transition, so `contract_generation` increases; commands of that reply are refused at receipt; queued commands and live goals of the older generation are refused/cancelled; a later `False` makes the authority valid again only for commands received from then on. A server "reconciled" report never revives an old goal: the goal record keeps its old generation until it ends. Hello: the reconcile result sets the flag (`is not False`) inside the fence and notes the change.

## 5. Withdrawal conditions (existing logic reused)
| Event | generation +1 | queued commands | live goal cancelled | STOP/Cancel | condition for new missions |
|---|---|---|---|---|---|
| contract version missing/invalid in a state reply | yes (R1.4.2/3) | refused (receipt / generation) | yes, latched | allowed | a later valid reply; only new commands |
| `reconciliation_required` True (R1.4.4) | yes | refused | yes, latched | allowed | False again + valid contract; old goals never resumed |
| server boot change / `needs_reconcile` | yes (fenced reset + re-hello) | refused (generation, and the command's server boot) | yes (generation, and `SERVER_DISCONNECTED_OR_RESTARTED`) | allowed | new hello + reconcile + valid state reply |
| Agent reconnect (hello) | yes (fenced reset) | refused | yes | allowed | as above |
| Agent process restart | new process starts at generation 0, but the old commands carry the old `owner_boot_id` and the durable journal marks unresolved ones `RECONCILIATION_REQUIRED` (existing) | refused (owner/journal) | old goal handled by the existing restart reconciliation (no handle in the new process) | allowed | server reconciliation with Nav2 terminal evidence (existing) |
| Lease expiry / epoch or owner change | no (not needed) | refused per command (`LEASE_MISMATCH`, existing) | yes (`LEASE_REVOKED_OR_EXPIRED`, existing; a re-acquired lease has a new epoch so the old goal never matches again) | allowed | new mission admitted under the new lease |
| HTTP disconnect within one boot | no | refused while `connected` is false (existing) | yes after `disconnect_cancel_after_s` (existing) | allowed | the reconnect re-hello bumps the generation, so pre-disconnect commands cannot run afterwards |

## 6. Results (isolated/mock; plain Python without ROS unless marked)
New `test_r1_4_4_submission_fence`: 12/12 PASS on the fixed tree (3 consecutive runs); on the Codex R1.4.3 source the same file gives 4 FAIL + 8 ERROR (`new_tests_on_prefix.txt`).
| Scenario | Result after |
|---|---|
| A invalidation (missing contract or reconciliation) before Stage 3 | 0 submissions, REJECTED `CONTRACT_GENERATION_CHANGED` |
| B invalidation released while Stage 3 holds the fence | the run loop is observed WAITING at the fence (instrumented lock), generation still 0 and the invalid reply not yet applied when `send_goal_async` is called; afterwards generation 1, 1 submission, 1 cancel request, reason `SERVER_CONTRACT_INVALID` (also after recovery) |
| C invalidation after the send | 1 submission, 1 cancel request |
| D queued command, reconciliation True -> False | 0 submissions, REJECTED (Codex: 1) |
| E live goal, reconciliation True -> False, monitor only after recovery | 1 cancel request, not repeated (Codex: 0) |
| F late handle + reconciliation flap | cancel flag set, cancel issued when the handle arrives |
| G mixed repeated withdrawals with redelivery | 0 old submissions, both REJECTED, generation >= 3 |
| H new command after recovery | 1 submission; H2 a command received while reconciliation is pending stays REJECTED |
| I explicit cancel while reconciliation is pending | honoured; the new motion command refused |
| J concurrency: run loop (400 replies flapping contract and reconciliation), ROS thread draining tasks, monitoring, resolving goals and cancels | both threads finish (join with timeout as deadlock detector), no exception |
In B the test can tell the two orders apart: it records the generation and whether the invalid reply was processed at the instant `send_goal_async` is about to be called, so B is used as evidence. The Codex `fence.py` probe cannot reach its boundaries any more (its hook waits for the invalidation INSIDE Stage 3, which the fence now forbids; it ends with an `IndexError` after its own wait), see `fence_after.txt`; B reproduces the same attempt deterministically.
Codex probes: `interleave.py` (R1.4.2) 8 orders unchanged from R1.4.3 (inactive 0 submissions, active 1 cancel); `edge.py`, `original-inputs.py` unchanged results.
Regression (all PASS): R1.4 9, R1.4.1 11 + 10, R1.4.2 40, R1.4.3 15, contract 11, `test_agent_safety_v1` 17 (also ROS sourced), loopback E2E 16, phase20 agent stub 7, SIGKILL 2, fail-closed 5, r8 1 + 4, recovery 14 + 14, watchdog admission 4, mapping server 26 (7 test-skips), gate reset server 14, gate pending/state sequences 6 + 7, terminal order 2, safety-context regression 24, `test_dedupe` 14/14. Isolated ROS (real rclpy, MultiThreadedExecutor, domain 187, localhost only, `unshare -Urn`): `test_ros2_isolated_action`, `test_ros2_late_goal_reconciliation` OK (they use the AgentCore-bound fence with real executor callbacks).
Full `unittest discover` (no ROS): 936 run, 2 FAIL, 33 ERROR, 17 SKIP; failing set identical to R1.4.3 (28 import failures for missing ROS packages / private-namespace guard, 5 `test_gate_reset_worker` for `std_srvs`, `test_agent_shutdown_20261122` SETUP_FAIL, `test_phase45_qr_libzbar` environment variable); no new regression (two names flagged by the log parser are interleaved ResourceWarning lines; both tests pass individually). P2 stub cache and the Watchdog timing flake remain open, unrelated.

## 7. Agent v5 impact
Agent-only change. A v5 Agent now also refuses motion while the server reports `reconciliation_required` (it already refused execution then; now the refusal is final for commands received in that window and old approvals end). Submission adds a short wait on the fence when it collides with a reply being applied (microseconds of assignments). Needs the v5 package and isolated robot verification: not done.

## 8. Still unverified / blocking R4
Real Nav2 cancel completion and physical stopping; real DDS timing of `send_goal_async` under the fence; the residual that the Nav2 server may already execute a goal whose request left before an invalidation (bounded only by the cancel duty and the monitor period, which is not a stopping-time guarantee); real Fleet reconciliation sequences (simulated replies here); Localization/Gate-reset worker, Arbiter and manual bench still do not bind their own actions to the generation (Codex observation, out of scope); Gate r13 and real Arbiter/Watchdog, `/cmd_vel` bypass, Depth, power/UART/STM32 timeouts, physical E-Stop and braking distance, v5 deployment.

## 9. Re-audit
```bash
cd <repo>/fleet_control/tests          # ROS not sourced
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=../server:../agent/limo_robot_agent:. python3 -W ignore -m unittest -v test_r1_4_4_submission_fence_20261123 test_r1_4_3_contract_generation_20261123 test_r1_4_2_contract_followup_20261123 test_r1_4_1_agent_protocol_20261123 test_r1_4_1_mapping_stop_20261123 test_r1_4_safety_contract_hardening_20261123 test_agent_server_contract_20261123 test_phase20_agent_sigkill test_phase20_fail_closed
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=../server:../agent/limo_robot_agent:. python3 ../../local_data/raw_results/codex_r142_audit_20261123_YYkMGp/interleave.py
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=../server:../agent/limo_robot_agent:. python3 test_dedupe.py
# isolated ROS: source Humble, ROS_LOCALHOST_ONLY=1, unused ROS_DOMAIN_ID, unset ROS_DISCOVERY_SERVER
CAPSTONE_PRIVATE_ROS_TEST=1 unshare -Urn bash -c 'ip link set lo up; python3 -m unittest test_ros2_isolated_action test_ros2_late_goal_reconciliation'
```
Pre-fix comparison: unpack the Codex R1.4.3 `snapshot.tar.gz`, copy in `test_r1_4_4_submission_fence_20261123.py` and `test_agent_safety_v1.py`, run the first command for that file: 4 FAIL + 8 ERROR.
