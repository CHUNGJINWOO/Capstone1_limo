# R1.4.6: the lease is re-read from every state reply; required-field matrix for motion approval (2026-11-23, Claude; isolated/mock only)

Not done: no real LIMO/ROS/deploy, no 18190 change, no motor unlock, no commit/push/PR; Neon's R2/R3 files (`motion_watchdog.py`, `mock_robot_node.py`, `limo_package_deploy_20261122.sh`, `agent_shutdown_matrix_20261122.py`, `test_r2_watchdog_shutdown_20261123.py`, `test_r3_deploy_tool_20261123.py`) untouched. Fence and state machine unchanged. **Not an approval of deployment, R4 or motor unlock; R1 closes only after Codex re-verifies this invariant.**
Baseline: HEAD `c804241a`, G1 `0ca09dc4`, tracked-modified 9; `core.py`, `safety_protocol.py`, `ros_adapter.py`, `service.py`, `ownership.py` byte-identical to the Codex R1.4.5 snapshot (`codex_r145_audit_20261123_xa2Xnm`). Records: `local_data/raw_results/real_limo_integ_claude_20261122/r1_4_6_01/`.

## 1. Cause
`AgentCore.run` replaced the lease only `if 'lease' in resp`. `Fleet.ingest_state` sends `lease` (a lease view or null) in every state reply, but a reply without the key kept the previous lease; the lease was also not part of the authority, so no generation change happened. A queued command was submitted (Codex: 1) and a live goal was not cancelled (Codex: 0). An explicit null already blocked, because the existing movement checks require a lease dict.

## 2. Changes (`agent/limo_robot_agent/limo_robot_agent/`)
| File | Function | Change |
|---|---|---|
| `safety_protocol.py` | new `lease_wellformed` | structure of `ownership.lease_view`: non-empty owner string, exact non-negative int epoch, valid expiry timestamp (reuses the existing helpers) |
| `core.py` | `run`, state reply | `lease` read from every reply: a well-formed lease is copied, anything else (missing, null, malformed) becomes `None` |
| | `_contract_ok_locked`, new `_lease_live_locked` | authority also needs a well-formed, unexpired lease from the current reply |
| | `_contract_note_locked` | a change of lease identity (owner, epoch) between two valid states also starts a new generation |
| | `_on_command` (receipt) | a motion command whose safety context names another lease owner/epoch than the current reply is refused at receipt (`LEASE_MISMATCH ... at receipt`), the same rule the execution stages apply |
Fixtures that had no lease although they model a session holding one (direct `_on_command` callers in `test_phase20_fail_closed`, `test_phase20_agent_sigkill`, the R1.4.3 unit test, `test_agent_safety_v1` test 8, the `test_dedupe` fake server, `test_recovery`'s receipt test which now acquires a real lease on the real server, the R1.4.5 wire test) were given one; no assertion changed. New `tests/test_r1_4_6_required_fields_20261123.py` (14 tests, matrices as subtests).

## 3. Required fields for motion approval, by message
| Message | Field | Rule (Agent) | Source |
|---|---|---|---|
| hello reply | `server_boot_id` | required; the protocol approval is bound to it | `service.hello` |
| | `protocol_version` | required, exact int 2 (only approval point) | `service.hello` |
| reconcile reply | `reconciliation_required` | read with `is not False`; alone it never grants, the next state reply decides | `recovery.reconcile` |
| state reply (normal and blocked) | `server_boot_id` | must equal the hello boot, else the whole reply is discarded (reset + re-hello) | `_deliver_atomic` |
| | `safety_contract_version` | required, exact int 1 | `ingest_state` |
| | `reconciliation_required` | required, exact boolean False grants | `_deliver_atomic` (explicit since R1.4.5) |
| | `lease` | required key; a well-formed unexpired lease view grants; null = no lease = no authority; owner/epoch must match the command's safety context | `ingest_state` (`lease_view`) |
| | `protocol_version` | optional; present and not exact 2 withdraws, never grants | - |
| | `needs_reconcile` | true -> reset + re-hello | `_deliver_atomic` |
| error reply / exception | - | the whole session is withdrawn (R1.4.5) | - |
| local | `connected` | true only between a completed hello/reconcile and the next failure | Agent |
| command | `owner_boot_id`, safety context boots, lease owner/epoch, timestamps | existing receipt/execution checks; now lease owner/epoch also at receipt | Agent |
Other state-reply fields (`operating_mode`, `navigation_request`, `gate_reset_request`) are re-read every reply already and default to `None`; their consumers check them themselves.

## 4. Results (isolated/mock; plain Python without ROS unless marked)
`test_r1_4_6_required_fields`: 14/14 PASS (3 runs). On the R1.4.5 source: 57 failures, **all of them lease cases** (18 lease values x 3 matrix tests + A, B, C/D); every other field in the matrix already passed there, which confirms the earlier fixes.
Matrix (each bad value after a valid session; 5 fields, 39 values: lease missing/null/list/string/empty/no owner/empty owner/int owner/string, bool, float, negative epoch/no expiry/string expiry/NaN expiry/expired/older epoch/other owner; safety version missing/null/bool/float/string/2; reconciliation missing/null/0/'false'/[]/true; protocol null/bool/float/string/1; server boot missing/null/int/old): queued command never submitted; live goal cancel request issued even when the monitor first runs after recovery; a command delivered in the bad reply is REJECTED for good (for a wrong server boot the whole reply is discarded and its commands are never received; a later delivery by a valid reply of the current boot is a new, valid delivery).
Scenarios: A missing lease, queued goal: 0 submissions (Codex 1); B live goal: 1 cancel request, reason `SERVER_CONTRACT_INVALID` (Codex 0); C/D after recovery the old command stays REJECTED and a new one runs; E null lease unchanged (0); F malformed structures (matrix); G epoch change refused; H null/malformed lease before Stage 3: 0 submissions, inside Stage 3: the run loop waits at the fence, generation 0 at the send, then 1 submission + 1 cancel request, after the send: cancel kept; I explicit cancel honoured without a lease, new motion refused; J real HTTP Server -> Agent (`wire.py current`): normal reply carries the lease, one start, mission CANCELED as before; a normal renewal (same epoch, later expiry) still executes.
Codex `independent.py` lease part: absent -> 0 submissions (was 1), active absent -> 1 cancel request (was 0), explicit null unchanged.
Regression (all PASS): R1.4 9, R1.4.1 11 + 10, R1.4.2 40, R1.4.3 15, R1.4.4 12, R1.4.5 13, contract 11, `test_agent_safety_v1` 17 (also ROS sourced), loopback E2E 16, phase20 stub 7, SIGKILL 2, fail-closed 5, r8 1 + 4, recovery 14 + 14, watchdog admission 4, mapping server 26 (7 test-skips), slam sessions 4, gate reset server 14, gate pending/sequences 6 + 7, terminal order 2, safety-context regression 24, r3/r4 2, lease contract 2, operating mode 11, manual bench contract 5, dedupe 14/14. Isolated ROS (real rclpy, domain 187, localhost only, `unshare -Urn`): `test_ros2_isolated_action`, `test_ros2_late_goal_reconciliation`, `test_agent_shutdown_20261122` OK.
Full `unittest discover` (no ROS): 974 run, 2 FAIL, 33 ERROR, 22 SKIP (17 + 5 new skips of Neon's `test_r2_watchdog_shutdown_20261123`, which need ROS); failing set identical to R1.4.5 and not converted (28 import failures for ROS packages / private-namespace guard, 5 `test_gate_reset_worker` for `std_srvs`, `test_agent_shutdown_20261122` SETUP_FAIL in the plain shell (passes in the ROS namespace), `test_phase45_qr_libzbar` env variable). No new regression: seven names flagged by the log parser pass individually (interleaved warning lines).
Behaviour change: releasing or losing the lease now ends the generation, so a live goal is cancelled and queued commands die at once (they could never execute without that lease anyway).

## 5. R1 closure
All R1 counterexamples found by Codex (R1.4 to R1.4.5) stay blocked, the required-field matrix covers every field the Agent uses to approve motion, the normal Server -> Agent path works. **Proposed for closure once Codex independently re-verifies the R1.4.6 invariant** (missing lease never reuses the previous lease; queued goal 0; cancel duty kept; recovery revives nothing; matrix). Outside R1: other execution actors not bound to the generation, stale same-boot reply ordering, P2 stub cache, Watchdog admission timing flake. R4 remains blocked by v5 packaging/approval (server change deployed first), real locked-stack STOP/Arbiter/Watchdog/Gate/Mapping STOP and DDS checks; motor unlock additionally by Gate r13, `/cmd_vel` path, Depth, power/UART/STM32 timeouts, physical E-Stop and braking distance.

## 6. Minimal re-audit
```bash
cd <repo>/fleet_control/tests          # ROS not sourced
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=../server:../agent/limo_robot_agent:. python3 -W ignore -m unittest -v test_r1_4_6_required_fields_20261123 test_r1_4_5_withdrawal_paths_20261123 test_r1_4_4_submission_fence_20261123 test_r1_4_3_contract_generation_20261123 test_r1_4_2_contract_followup_20261123 test_r1_4_1_agent_protocol_20261123 test_r1_4_1_mapping_stop_20261123 test_r1_4_safety_contract_hardening_20261123 test_recovery test_server_agent_safety_e2e
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=../server:../agent/limo_robot_agent:. python3 -W ignore ../../local_data/raw_results/codex_r145_audit_20261123_xa2Xnm/independent.py
```
Pre-fix comparison: unpack the Codex R1.4.5 `snapshot.tar.gz`, copy in `test_r1_4_6_required_fields_20261123.py`, `test_r1_4_5_withdrawal_paths_20261123.py`, `test_r1_4_4_submission_fence_20261123.py`, `test_agent_safety_v1.py` and run the first test: 57 failures, all lease cases.
