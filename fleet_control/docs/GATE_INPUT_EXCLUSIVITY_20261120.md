# Safety Gate input exclusivity and web control integration check (2026-11-20)

Scope: Web/Nav2 → Arbiter → Motion Watchdog → Safety Gate → locked output (`/cmd_vel_locked`). Isolated ROS only (`ROS_LOCALHOST_ONLY=1`, private domains, no driver, `/cmd_vel` with 0 endpoints).

Not done in this work:
- no LIMO connection or SSH;
- no change to the 18190 server;
- no real-robot motor unlock, and real-robot Mapping START stays blocked;
- no commit, push, PR or worktree;
- the field bundle Gate and the `capstone_ws/src` Gate are **not modified**.

## 1. Ownership check

| Gate copy | State | Decision |
|---|---|---|
| `capstone_ws/src/limo_capstone/limo_capstone/stall_detector_node.py` | 597 lines, uncommitted 8-line `max_abs_linear_x` change by another agent (2026-11-19) | not touched; proposal patch 05 only |
| `experiments/real_auto_mapping_ready_20261113/.../stall_detector_node.py` (field bundle, Phase 1 default) | 589 lines | not touched |
| `experiments/safety_chain_fix_20261113/candidate_gate_r10/` (Claude candidate line r7–r10, used by the Phase 2/3 stacks) | unchanged (SHA-256 `0a6afe75…`) | kept as is |
| **new** `candidate_gate_r11/` | r10 + input-source validation (SHA-256 `26fb680d…`) | implemented here. It is the same candidate line, not a second Gate: one process, one `/cmd_vel` output |

## 2. Cause of the bypass

- The Gate used every `/cmd_vel_raw` message, whoever sent it. The Watchdog and Arbiter check their own inputs, but no stage checked the input of the last stage before the motor.
- Humble rclpy has no per-message sender GID. `take_message` MessageInfo holds only timestamps (probed 2026-11-20), so a check can only be graph-level.
- NaN commands passed: `abs(NaN) > threshold` is False, so `want_motion` was False and the NaN was copied to the output.
- Reproduced before the fix, on the same harness:
  - standalone r10: rogue 0.18 / 0.12 / 0.15, NaN, Inf, 5.0 and 9.0 all reached the output;
  - Phase 1 s18 (field Gate): 57 rogue-value samples;
  - Phase 2 g01/g02/g03 (r10): 114 / 116 / 64;
  - Phase 3 f07b (r10): 91.

## 3. Code changes

- **Gate r11** (`experiments/safety_chain_fix_20261113/candidate_gate_r11/nodes/stall_detector_node.py`).
  - Rule and limits: [GATE_R11_INPUT_SOURCE_20261120.md](../../experiments/safety_chain_fix_20261113/docs/GATE_R11_INPUT_SOURCE_20261120.md).
  - Summary: the input is bound to one allowed writer GID. A second endpoint, a foreign name, a GID change, a lookup error or an unreadable GID → FAILSAFE and zero output.
  - NOT_EXCLUSIVE, UNEXPECTED_PUBLISHER and PUBLISHER_CHANGED stay set until `/stall_detector/reset_latch`; no automatic re-authorisation.
  - After any non-OK graph period there is a settle window before messages are used again.
  - NaN/Inf and out-of-range Twist → zero plus a fault. A non-finite output is forced to zero.
  - A rejected message that carries motion blocks arming (keeps the r10 rule "arm only with no motion commanded").
  - Every r10 arming / sensor / stall check is unchanged.
- **Test harnesses** (originals in `local_data/raw_results/gate_input_claude_20261120/pre_change_originals/`, SHA256SUMS):
  - `tests/web_mapping_isolated/stack.launch.py`, `tests/mode_e2e/stack_up.sh`: env-guarded `GATE_INPUT_EXCL`, `GATE_INPUT_MAX_VX`, `GATE_INPUT_MAX_WZ`, `GATE_SCRIPT` override. Defaults are unchanged (r10, no input parameters).
  - `tests/manual_e2e/run_manual_e2e.py`: `GATE_SCRIPT`/`GATE_INPUT_EXCL` override (default stays the field Gate); new **s18** (rogue `/cmd_vel_raw` while web MANUAL drives).
    - **s14 expectation change, with r11 only:** a restarted Watchdog is a new writer, so the web start must be refused until the operator Gate reset; then it must recover. The original automatic-recovery expectation is kept when the input parameters are off.
  - `tests/mode_e2e/run_mode_e2e.py`: new **g01/g02/g03** (rogue `/cmd_vel_raw` during MANUAL / MAPPING / NAVIGATION). They are asserted with every Gate, so r10 fails them. g03 runs after f06.
  - `tests/nav_e2e/run_nav_e2e.py`: new **f07b** asserts the Gate-input part of f07 (f07 itself is unchanged).
    - `ensure_nav2`: a stack restart also restarts the test Agent. A new Gate boot makes a running Agent latch `r7_invalid` by design, which blocked every later mission in one run.
  - All three runners: `gate_guard` / `gate_unlatch_until_ready` repeat the recorded operator unlatch while the Gate is still blocked. A SIGKILLed writer's DDS ghost lasts about 15–20 s, and r11 correctly refuses the reset while two writers are visible.
- **Proposal** `docs/proposals_20261120/05_gate_input_source_validation.patch` for the `capstone_ws/src` Gate (not applied; `git apply --check` passes on the current working tree).

No server, Agent, Watchdog, Arbiter or UI code was changed in this task.

## 4. Data flow and the protection boundary

```
Web panel ─HTTP→ Fleet ─→ Agent ─ VelocityArbiter ─/fleet/nav_cmd_vel→ fleet_motion_watchdog ─/cmd_vel_raw→ Gate r11 ─/cmd_vel_locked (isolated) / /cmd_vel (robot)
Nav2 velocity_smoother ─/fleet/cmd_vel_nav_smoothed→ Arbiter                     ▲ graph check: exactly 1 endpoint, name fleet_motion_watchdog, bound GID
```

| Protection available with the current DDS (no security) | Needs access control (SROS2 DDS-Security) |
|---|---|
| a second `/cmd_vel_raw` writer while the Watchdog lives; a foreign node name; a writer swap or restart (GID change); a failed or unreadable GID lookup; non-finite or out-of-range values; no automatic re-authorisation | a process that uses the name `fleet_motion_watchdog` and is the **only** writer from the Gate's start, or after an operator reset, is accepted (measured: `impostor_only` 150/150). GIDs are identifiers, not credentials |

## 5. Results

### 5.1 Standalone Gate (r10 vs r11, details in the Gate report)

| Case | r10 | r11 |
|---|---|---|
| normal Watchdog only | PASS 150/150 | PASS 150/150 |
| normal + external writer | **bypass** (0.18 output) | blocked; rogue value never output; sticky until reset |
| external writer only | **bypass** 187/201 | 0/200 |
| GID lookup failure / unreadable GID | — (no check) | unit: FAULT + drop |
| publisher swap | accepted automatically | 0/200 until reset |
| original exits, other remains | **bypass** 200/200 | 0/201 (sticky) |
| invalid Twist (NaN, Inf, 5.0, 9.0) | **bypass** | 0 in every segment; reset → normal |
| input expiry / Watchdog zero / SIGSTOP | zero within 0.25 s | same |
| Watchdog SIGKILL + restart | auto-accepted | blocked during the 16 s ghost; reset → normal |
| Gate SIGKILL + restart | no arming while motion commanded | same (after the arming fix) |
| forged writer alone | accepted | **accepted (known limitation)** |

Unit tests `test_gate_r11_input_unit.py`: 13/13 PASS.

### 5.2 Phase 1–3 E2E (isolated, domain 228, port 28957; raw results `local_data/raw_results/gate_input_claude_20261120/`)

**Gate-input probes: the result is the same in every run** (counted from every `result.json` under that directory; runs where a probe errored before probing are not counted).

| Probe (rogue `/cmd_vel_raw` writer while the legitimate path is active) | field Gate / r10 | r11 |
|---|---|---|
| Phase 1 s18 — MANUAL (web bench) | rogue value reached the output in 3/3 runs where it ran (57, 58, 69 samples) | 0 rogue samples in 7/7 runs |
| Phase 2 g01 — MANUAL | reached it in 6/6 runs (36–114 samples) | 0 in 7/7 runs |
| Phase 2 g02 — MAPPING | reached it in 6/6 runs (20–116) | 0 in 6/6 runs where g02 ran |
| Phase 2 g03 — NAVIGATION (active mission) | reached it in 2/2 runs where g03 ran (37, 64) | 0 in 3/3 runs where g03 ran |
| Phase 3 f07b — NAVIGATION stack | reached it in 1/1 run (91; Codex's earlier run: 28) | 0 in 2/2 runs |

**Full-suite runs.** "Before" = field Gate (Phase 1) or r10 (Phase 2/3); "after" = r11 with the input parameters.

| Run | After (r11) | Before | What decided the non-Gate failures |
|---|---|---|---|
| Phase 1 run03 (no browser) | **18 PASS / 0 FAIL** / 1 SKIP (s16 not run) | 17 / 1 (s18) / 1 | — |
| Phase 1 run16 ×2 (with browser, full order) | 18 / **1 (s10)** — both runs | 18 / 1 (s18) — both runs | **open:** s10 `zero_after_s=None` = no non-zero sample after the foreign FollowPath goal. r11 passed s10 in 5 other full runs (0.010–0.049 s) and 4/4 subset runs; the field Gate passed it everywhere. Cause not identified, so r11 is not ruled out |
| Phase 1 run12/run14 (browser) | 18 / 1 (s16 pointer) | — | the pointer phase showed "no fresh safety evidence" with the Gate NORMAL in its log; s16 then passed 4/4 subset runs and 2/2 full runs with each Gate |
| Phase 2, original 17 scenarios (run10) | **17 / 0 / 0** (incl. f05 Watchdog kill → recorded Gate reset → admin recovery, and f06) | earlier baseline 17 / 0 / 0 (`codex_fix_claude_20261120/mode_e2e_run04`) | — |
| Phase 2 with g-probes, run04 / run08 | **19 / 0 / 1** (f06 SKIP) | run03 16 / 3 (g01–g03) / 1; run05 17 / 3 / 0 | f06 SKIP: Watchdog DDS ghost timing before f05's reset (fixed afterwards) |
| Phase 2 with g-probes, other runs | run03 6 / 14; run05 18 / 1; run06 11 / 9; run07 14 / 6; run09 15 / 5 | run06 12 / 8; run07 12 / 8; run08 15 / 5; run09 12 / 8 | heartbeat gap during SAVE (run03); g03 after f06 in FAULT (run05); Nav2 bt_navigator lifecycle response lost (run06 r11); **my faulty lifecycle guard restarting a healthy stack 3× (run07, both Gates)**; localization / mission variance after the g-probes moved the robot (run06/08/09, both Gates) |
| Phase 3 run04 (r11) / run03 (r10) | **18 / 0 / 0** | 17 / 1 (f07b) / 0 | — |

Reading the table:
- Every assertion about the Gate input passed with r11 and failed with field/r10.
- The original Phase 2 and Phase 3 suites pass with r11 exactly as before.
- Inserting the three g-probes made the later NAVIGATION scenarios order-sensitive, with **both** Gates. The g-probe harness is therefore not yet a stable regression suite; the probes themselves are reliable.
- One Phase 1 item (s10 in full browser order) is unexplained.

Runs that are not results:
- `e2e_run01`: aborted at start (`set -u`).
- `e2e_run02`: stopped after a harness defect in the new s14.

The first r11 Phase 2 run (`e2e_run03/phase2_after_r11`) ended in a cascade because the Agent heartbeat (0.1 s executor timer) stopped for more than the Watchdog's 0.3 s timeout during the map SAVE. The Watchdog went FAILSAFE (`AGENT_HEARTBEAT_LOST`), the mapping worker declared the session lost, and the Gate stayed NORMAL. This is an existing margin problem, not resolved.

Harness changes during the work (each original kept in `pre_change_originals/`):
- the s14 409 handling;
- `ensure_nav2` restarting the Agent with the stack (Phase 3, then Phase 2);
- g03 and g02 moved to the end, with the f05-style recovery first;
- f05 waiting for the new Watchdog before the recorded Gate reset;
- a lifecycle 'all active' check added and **removed**: Phase 2 Nav2 only fully activates after SLAM provides the map frame.

Snapshot hashes are in `snapshot_start/end.sha256` of each sequence directory.

## 6. Gazebo

**SKIP for this task.** The existing Ignition Fortress office world + LIMO + real Nav2 + Gate launch (`experiments/safety_chain_fix_20261113/office/office_limo_r10.launch.py`, run network-isolated by `experiments/fleet_followpath_trust_20261118_N3p5/run_isolated.sh`) is reusable, but it is not on the web/Arbiter chain:

1. `ros_gz_bridge` drives the robot from `/cmd_vel`. The Arbiter's locked-graph check requires `/cmd_vel` to have 0 endpoints and `/cmd_vel_locked` to be published only by the Gate. → The bridge would have to subscribe `/cmd_vel_locked`, as a sim-only launch variant.
2. Nav2 publishes `cmd_vel_smoothed` / `behavior_server:cmd_vel` → `/cmd_vel_pre_cm` → Collision Monitor → `/cmd_vel_raw`. → It would have to go to `/fleet/cmd_vel_nav_smoothed` (Arbiter input) → Watchdog → `/cmd_vel_raw`. The Collision Monitor would then sit before the Arbiter or be dropped, which is a design decision.
3. Gazebo runs with `use_sim_time=true`. The Agent, Watchdog and Arbiter use monotonic time and must be audited for sim time; the Gate already runs its loop on a steady clock.
4. Agent options: the Localization Worker starts its own map_server/AMCL, so the office launch's SLAM/AMCL must be off for NAVIGATION.

Not done here because it is a separate integration (a new launch variant plus a time audit), and running Gazebo next to the E2E regression would have loaded the CPU and contaminated the timing results. No real-robot launch or Gate was weakened.

## 7. Remaining real-robot blockers

1. r11 is not in the field / `capstone_ws` Gate. Proposal 05 needs the Gate owner's review and an isolated re-test of the patched file.
2. No authentication: a forged single writer is accepted. SROS2 permissions for `/cmd_vel_raw` and `/cmd_vel` have not been designed or tested.
3. **No web path for the Gate operator reset.** With r11 every Watchdog restart, and any input fault, needs `/stall_detector/reset_latch`, which today requires ROS access. This conflicts with SSH-free operation. An admin-only, evidence-checked web Gate re-arm is needed.
4. The Agent heartbeat margin during map SAVE (see 5.2) can end a mapping session.
5. Open: Phase 1 s10 latency `None` with r11 in 2 full browser-order runs (see 5.2). Codex should check it with the locked-output samples recorded.
6. Earlier open items remain:
   - external FollowPath/Spin/BackUp goals are not watched by the Watchdog;
   - there is no web policy for VERIFIED map promotion;
   - Gazebo is not connected (section 6);
   - there is no hardware attestation;
   - the STM32 command timeout, E-Stop and braking distance are not measured.

## 8. Codex reproduction

```bash
cd <repo>; source /opt/ros/humble/setup.bash; source capstone_ws/install/setup.bash
# Gate unit + standalone (r10 vs r11)
cd experiments/safety_chain_fix_20261113/harness/gate_r11_input
python3 test_gate_r11_input_unit.py
python3 run_gate_input.py --gate r10 --out ../../runs/gate_r11_input_20261120/<new_r10_dir>
python3 run_gate_input.py --gate r11 --out ../../runs/gate_r11_input_20261120/<new_r11_dir>
# Phase 1-3, before and after (OVERLAY = explore_lite overlay install; writes NEW directories only)
OVERLAY=<overlay> local_data/raw_results/gate_input_claude_20261120/run_seq.sh <new_out_dir>
```

Single runs: set `GATE_SCRIPT=experiments/safety_chain_fix_20261113/candidate_gate_r11/nodes/stall_detector_node.py GATE_INPUT_EXCL=fleet_motion_watchdog GATE_INPUT_MAX_VX=0.20 GATE_INPUT_MAX_WZ=0.60` before `run_manual_e2e.py`, `run_mode_e2e.py <port> <domain>` or `run_nav_e2e.py <port> <domain>`. Omit them for the before Gates.

## 9. Verification status

- **Verified (isolated ROS, kinematic sim):**
  - the bypass with the field Gate / r10;
  - r11 blocking in every standalone case and in every MANUAL / MAPPING / NAVIGATION probe run;
  - Phase 1 (no browser), the original Phase 2 17 and Phase 3 18 with r11;
  - the proposal-05 patched copy of the `capstone_ws` Gate in all 10 standalone cases (`runs/gate_r11_input_20261120/p05_capstone_ws_patched_copy`);
  - unit tests unchanged from the previous summary: two files error without their stubs, as before.
- **Not stable:** the Phase 2 suite with the g-probes inserted (order-dependent localization / Nav2 variance, both Gates); Phase 1 s10 in full browser order with r11 (2 runs).
- **Not verified:** Gazebo; the real robot; proposal 05 on a real-robot launch; SROS2.
- **Assumed:** discovery precedes data delivery for a new writer (no rogue value was ever observed before detection, but it is not proven).
