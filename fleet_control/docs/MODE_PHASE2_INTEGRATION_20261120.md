# Phase 2 — 웹 운용 모드 전환과 자동 맵핑 통합 (격리 ROS, 2026-11-20, Claude)

설계 기준: [INTEGRATED_CONTROL_PLAN_20261120.md](INTEGRATED_CONTROL_PLAN_20261120.md). Phase 1: [MANUAL_PHASE1_INTEGRATION_20261120.md](MANUAL_PHASE1_INTEGRATION_20261120.md).
증거: `local_data/raw_results/mode_phase2_claude_20261120/` (Git 제외).

## 1. 결론

웹 HTTP API만으로 IDLE / MANUAL / MAPPING / NAVIGATION 사이를 전환하고, 각 모드에서 실제 ROS 2 작업(수동 속도, SLAM Toolbox + Frontier 자동 맵핑, Nav2 NavigateToPose)을 실행·정지했다. 결과는 아래 §5 표와 같다. 실행 환경은 다음과 같다.
- **격리 localhost ROS**(domain 228)
- 실제 Fleet 서버 프로세스(`--require-operating-mode`)
- 실제 Agent·Watchdog·Nav2 6개 서버·SLAM Toolbox·`fleet_frontier_candidates`
- r10 후보 Gate(출력 `/cmd_vel_locked`)
- **2D 운동학 시뮬레이터**

시뮬레이터는 Gate 출력만으로 움직이며, LiDAR ray-cast와 바퀴 odom을 생성한다.

- **실제로 움직인 것**: 시뮬레이터 로봇. 모든 이동은 Gate의 `/cmd_vel_locked`를 적분한 결과다. Frontier 탐색 중 이동, NAVIGATION goal 도달, 수동 조작을 확인했다.
- **Gazebo는 SKIP**: Arbiter가 mock + 잠금 그래프 전용이고, Gazebo 경로(SIM_GATED, Gate → `/cmd_vel` → gz bridge)에는 Arbiter가 없다. 이를 연결하는 것은 별도 변경이다.
- **실차 미실행**: 모터 잠금과 실차 Mapping START 차단(서버 `mapping_real_blocked`, UI, Agent)은 그대로 유지했다.
- **NAVIGATION의 지도·localization은 FIXTURE**다. 저장한 SLAM 지도를 `map_server`로 올리고 정적 `map→odom`(identity)을 사용했으며, 시뮬레이터에 drift가 없어 성립한다. 실제 localization 생산자는 여전히 없다(Phase 3). 운용 모드(`operation_mode`)는 FIXTURE가 아니라 서버의 실제 모드다.

## 2. 구현한 모드 전환 경로

`POST /api/v1/robots/{id}/mode {target, epoch, idempotency_key}` → 202(접수만). 결과는 `GET .../mode` 또는 대시보드 `operating_mode`로 확인한다.

```text
요청 → STOPPING_PREVIOUS → VERIFYING_IDLE (1 s 연속) → CONFIRMING → ACTIVE
                 └ 실패/시간초과(30 s) → FAILED: 이전 작업 종료가 증명되면 IDLE, 아니면 FAULT. 자동 재시도 없음
```

- **STOPPING_PREVIOUS**: 기존 메커니즘만 사용한다.
  - 수동: 세션을 삭제한다. Agent 샘플은 250 ms 안에 만료된다.
  - 맵핑: 기존 STOP 명령을 쓴다. Worker의 `goal_terminal` 증거가 필요하다.
  - 임무: 기존 `cancel_mission`을 쓴다. Nav2 결과로 terminal이 되어야 한다.
  - 맵핑 명령이 진행 중(STARTING / PAUSING / SAVING / STOPPING / RECONCILING)이면 전환 요청 자체를 `mapping_busy`로 거부한다.
- **VERIFYING_IDLE**: 아래 조건이 모두 1 s 이상 연속으로 성립해야 한다. GID 연속성과 빈 cancel-all만으로는 종료 증명으로 보지 않는다.
  - 신선한 Agent 보고(≤1.5 s)
  - 중단한 임무의 terminal 결과
  - Agent 소유 goal 없음, 외부 NavigateToPose goal 없음
  - NavigateToPose·FollowPath idle 증거
  - Watchdog `MOTION_AUTHORIZED`(Watchdog 자체의 cancel-all 증명)
  - Arbiter 출력 0
  - Mapping Worker가 IDLE이고 프로세스 없음
- **CONFIRMING**: 목표 모드를 Agent에 전달하고, Arbiter가 해당 모드로 동작한다고 보고할 때만 ACTIVE가 된다. 이 단계는 시험 중 실제 경합을 발견해 추가했다. 서버가 ACTIVE를 먼저 선언해 첫 수동 시작이 거부되던 문제였다.
- **admission 게이트**:
  - 수동 start/sample/pull은 MANUAL에서만 허용한다.
  - 맵핑 START/RESUME은 MAPPING에서만 허용한다.
  - 임무 생성은 NAVIGATION에서만 허용한다.
  - 모두 ACTIVE 이후에만 허용한다. PAUSE / SAVE / STOP / RECONCILE(정지 방향)은 언제나 허용한다.
- **무효화**:
  - Lease 만료·변경, Agent boot 변경, 서버 재시작 시 IDLE로 돌아간다.
  - 맵핑 세션이 미해결이면 FAULT가 된다. 수동 세션은 즉시 삭제한다.
- **Agent 측**:
  - 서버 응답의 `operating_mode`에 따라 Arbiter가 단일 속도 출처를 선택한다(MANUAL → bench, MAPPING / NAVIGATION → Nav2 smoother).
  - Nav2 출처는 해당 모드의 Agent 소유 goal(`frontier` / `fleet`)이 실행 중일 때만 쓴다. 속도는 slow profile 상한으로 clamp한다.
  - 그 외 모드는 0을 출력한다. 서버 모드는 Safety Context의 실제 `operation_mode` 생산자가 된다.
- **기존(legacy) 로봇**: 모드 요청을 한 번도 받지 않은 로봇은 `--require-operating-mode`가 없으면 이전 동작을 유지한다. Phase 1 계약과 기존 시험 호환을 위해서다.

시험에서 확인한 전환은 IDLE→MANUAL, MANUAL→IDLE, IDLE→MAPPING, MAPPING→MANUAL(자동 STOP), MANUAL→MAPPING, MAPPING→IDLE, IDLE→NAVIGATION, NAVIGATION→MANUAL(진행 중 goal 취소), MANUAL→NAVIGATION, FAULT→IDLE/MANUAL(상태 조정 후)이다.

## 3. 웹에서 실행 가능한 Mapping 기능

기존 `mapping_panel.js` 계약을 그대로 쓴다. START, PAUSE, RESUME, SAVE(층·영역 선택 시 Registry에 UNVERIFIED 새 버전 등록), STOP을 지원한다. 상태는 `QUEUED → ACCEPTED → COMPLETED | FAILED | UNKNOWN`이다. HTTP 202는 접수만 의미하며, 결과는 Worker 증거로 판정한다.

이번에 추가한 것은 다음과 같다.
- **RECONCILE(상태 조정)**:
  - UNKNOWN 세션에만 허용한다. 현재 Agent boot의 Worker가 증거 세 가지(자신이 띄운 SLAM·Frontier 종료, 그래프에 SLAM·Frontier 노드 없음, NavigateToPose·FollowPath 활성 goal 없음)를 증명할 때만 STOPPED가 된다.
  - 실패하면 UNKNOWN을 유지하고 자동 재개는 없다.
  - 기존에는 UNKNOWN 상태를 해제하는 API가 없어, SSH로 Worker 저널을 지우지 않는 한 MANUAL과 MAPPING이 영구 차단됐다.
- START/RESUME 버튼은 운용 모드가 MAPPING이 아니면 비활성화하고 사유를 표시한다.
- 운용 모드 카드(`static/operating_mode_panel.js`)를 추가했다.
  - 표시: 현재 모드, 전환 단계와 대기·실패 사유, Lease, 활성 Nav2 goal, Nav2 idle 증거, Mapping Worker, 속도 출처와 값, Deadman, 차단 사유, Watchdog, Gate
  - 보고되지 않은 값은 "미보고"로 표시한다.

## 4. 함께 고친 결함 (실제 시험에서 발견)

| 결함 | 영향 | 수정 |
|---|---|---|
| 서버 Safety Context·Agent `compute_movement_permitted`가 NavigateToPose status 메시지 나이 ≤2 s를 요구 | 실제 Nav2는 상태 변화 때만 발행하므로, idle 시 임무 admission이 불가하고 실행 중 임무도 2 s 뒤 무효화된다 | Agent가 `status_continuous`(유일 발행자, GID 연속)를 보고하고, 서버와 Agent가 나이 대신 이를 허용한다 |
| Gate `STALL_PENDING`(가속 시작 ~20 ms) | Arbiter가 수동 세션을 끊고, 서버가 진행 중 임무를 RECONCILIATION_REQUIRED로 보낸다 | Arbiter는 `cmd_blocked=false`인 STALL_PENDING을 허용하고, 확정 stall은 계속 차단한다. 서버의 기존 continuation 규칙은 잠금 mock 그래프(모터 경로 LOCKED)에서 같은 goal 소유 검사로 허용한다 |
| 전환 중 `operation_mode`가 바뀌어 취소 중인 임무가 무효화됨 | 깨끗한 CANCELED 대신 reconciliation이 발생한다 | STOPPING_PREVIOUS 동안 NAVIGATION 해제 시에는 NAVIGATION과 이전 revision을 유지한다. 속도는 TRANSITIONING이라 0이다 |
| RECONCILING 세션이 Worker UNKNOWN 보고로 즉시 다시 UNKNOWN이 되고, 예약 goal 소유가 인정되지 않음 | 상태 조정 불가 | RECONCILING에서는 Worker UNKNOWN을 예외로 두고, 소유 상태 목록에 RECONCILING을 추가했다 |
| Worker `fleet_goal_allowed` 훅이 정의만 있고 미사용 | 맵핑 세션 중 Fleet goal을 Agent가 독립적으로 막지 못한다 | `_verify_nav_preconditions`에서 강제한다 |
| 격리 Nav2 lifecycle 기동 간헐 실패(`change_state` 응답 유실) | Nav2 미기동 | `stack.launch.py`에 env 보호된 `LIFECYCLE_DELAY_S`를 추가했다(기본 0, 기존 동작 유지) |

## 5. 시험 결과

`tests/mode_e2e/run_mode_e2e.py`: **run01 16 PASS / 0 FAIL / 1 SKIP, run02 16 / 0 / 1, run03(f06 subset) 8 / 0 / 0**. SKIP은 f05 뒤 로봇 단위 reconciliation이 남아 f06을 건너뛴 것이며, f06은 run03에서 단독으로 PASS했다. 개발 중 실행(mode_dev01–14)은 scratch에만 있다.

| 시나리오 | 확인 내용 |
|---|---|
| n01 | 기동 시 IDLE. 실제 Nav2 idle 무발행(`received_status_array:false`, `status_continuous:true`)에서 Watchdog 승인. 모드 밖 수동·START·임무 409 |
| n02 / n03 | IDLE→MANUAL(1.2 s) 후 수동 주행으로 시뮬레이터 이동. 주행 중 MANUAL→IDLE 시 0.5 s 안에 0, 이후 수동 409 |
| n04 | IDLE→MAPPING, START(QUEUED→ACCEPTED→COMPLETED). Frontier goal → Nav2 → Arbiter(source NAV2) → Gate로 로봇 이동. MAPPING 중 수동·임무 409 |
| n05 | PAUSE(goal_terminal), PAUSED 동안 출력 0. SAVE 중 모드 요청 409 `mapping_busy`. SAVE yaml / pgm / posegraph와 Registry UNVERIFIED 등록. RESUME 후 다시 이동 |
| n06 | MAPPING→MANUAL 직접 전환: 서버가 STOP 발행 → goal_terminal → idle 검증 → Agent 확인 → MANUAL 주행 |
| n07 | MANUAL→MAPPING, START, STOP, MAPPING→IDLE. Worker 프로세스 없음 |
| n08 / n09 / n10 | IDLE→NAVIGATION 임무 SUCCEEDED(목표 오차 <0.4 m). 이동 중 NAVIGATION→MANUAL: 74 ms 안에 0, 임무 CANCELED(Nav2 결과), 전환 중 두 번째 요청 409 `transition_in_progress`, 이후 수동 주행. MANUAL→NAVIGATION 임무 SUCCEEDED |
| f01 | MANUAL 주행 중 Lease 강제 해제 → 0.5 s 안에 0, 모드 IDLE |
| f02 | MANUAL 주행 중 서버 SIGKILL → 0. 재시작 후 IDLE(`server restarted`), 재선택 전 수동 409 |
| f03 | MAPPING 중 Lease 만료 → 출력 없음, 세션 UNKNOWN, 모드 FAULT, MANUAL 전환 거부 → RECONCILE(증거 3종) → STOPPED → IDLE |
| f04 | MAPPING 중 Agent SIGKILL → 0. 재시작 후 UNKNOWN / FAULT. RECONCILE 1차 시도는 `goal_terminal:false`로 거부되고 재시도에서 STOPPED. 이후 MANUAL 주행 |
| f05 | NAVIGATION 주행 중 Watchdog SIGKILL → 0.5 s 안에 0, 임무 무효화(자동 재출발 없음). **로봇 단위 reconciliation은 admin recover가 `no_evidence`로 거부해 해제하지 못했다**(기존 한계, §8) |
| f06 | bt_navigator SIGSTOP(취소 무응답) 상태에서 NAVIGATION→MANUAL 요청 → 0.5 s 안에 0, 30 s 뒤 FAILED → FAULT, 자동 재시도 없음 |
| 공통 | `/cmd_vel` 발행·구독 0 |

**회귀 시험**: §6 참조.

## 6. 회귀

| 항목 | 결과 |
|---|---|
| Phase 1 수동 E2E(legacy 경로, 브라우저 포함) | 18/18 PASS (`phase1_manual_e2e_regression/`) |
| 서버·Agent 단위 및 계약 시험 | PASS: 웹 맵핑 26, 소유권, Safety Context v1 계열, 수동 벤치·Lease, Watchdog admission, 지도 저장, 복구 14, 서버–Agent 안전 E2E 16, Agent safety 17, r3/r4, r8 continuation, FollowPath 하위 goal, Watchdog 18, action 증거 4, 운용 모드 11, 패널 Node |
| Codex chain(domain 225) 기본·agent_restart, adversarial(staging) | PASS, 12 PASS / 1 SKIP |
| 실브라우저 운용 모드 카드(`browser_run01/`) | Lease 획득 → MANUAL·MAPPING·IDLE 전환(각 약 2 s) 단계와 상태 행 표시, 375 px에서 가로 스크롤 없음, page error 0 |
| Phase 2 이전에도 실패하던 항목(변화 없음) | `run_checks.py` mission created 201(2건), `test_manual_bench_version_comparison`, `test_manual_arbiter_freeze_20261120` |
| 환경상 SKIP | 사설 network namespace 필요 5개, 후보 패치·공개 트리 경로 필요 2개, preview URL 인자가 필요한 대시보드 브라우저 `.cjs` 다수 |

Codex 시험의 가짜 DB에 `one()`(기록 없음 = legacy)을 추가했다. 대상은 `test_manual_bench_safety_fix_20261120.py`와 `test_manual_session_adversarial_20261120.py`이며, 기대값은 바꾸지 않았다. 원본은 `superseded_test_originals/`에 있다.

## 7. Codex 독립 검증용 재현

```bash
cd <repo>
source /opt/ros/humble/setup.bash; source capstone_ws/install/setup.bash
W=$(mktemp -d)   # explore_lite overlay (fleet_frontier_candidates); capstone_ws/install은 수정하지 않음
colcon --log-base $W/log build --base-paths capstone_ws/src/m-explore-ros2 --packages-select explore_lite --build-base $W/build --install-base $W/install
cd fleet_control
export OVERLAY=$W/install PYTHONPATH=$PWD/agent/limo_robot_agent:$PWD/server:$PYTHONPATH
MODE_E2E_OUT=$PWD/../local_data/raw_results/<새 run_id> python3 tests/mode_e2e/run_mode_e2e.py <빈 포트> <격리 domain ≤232>
python3 tests/test_operating_mode.py; node tests/test_operating_mode_panel.cjs
```

`MODE_E2E_ONLY=...`로 일부 시나리오만 실행할 수 있다. f06은 f05 이후 로봇 단위 reconciliation이 남으므로 단독 subset으로 실행한다(run03 참조).

권장 독립 검증:
- `VERIFYING_IDLE` 조건의 충분성: NavigateThroughPoses·Spin·BackUp 외부 goal은 Watchdog과 Arbiter 모두 감시하지 않는다.
- CONFIRMING 경합: Arbiter 보고 지연과 Agent 재시작이 겹치는 경우.
- STOPPING 중 NAVIGATION 유지 규칙의 오용 가능성.
- RECONCILE `graph_clear`에서 DDS ghost 처리.
- 잠금 mock continuation 예외가 실차·SIM 경로에 새지 않는지.

## 8. 미해결 안전 차단 조건

1. 로봇 단위 RECONCILIATION_REQUIRED(Watchdog trip, 취소 무응답)를 해제할 실모드·mock 복구 증거 경로가 없다. admin recover는 `no_evidence`를 반환한다(기존 P1). 안전하게 거부되지만, SSH 없는 복구는 아직 불가능하다.
2. 실제 localization·지도 검증 생산자가 없다. NAVIGATION은 FIXTURE로만 검증했다(Phase 3).
3. Gazebo와 Arbiter 통합(SIM_GATED 경로)은 미구현이다.
4. Watchdog은 NavigateToPose만 감시하고, FollowPath는 Agent Arbiter만 감시한다. NavigateThroughPoses·Spin·BackUp 외부 goal은 미감시다.
5. SIGKILL된 노드의 DDS ghost(~20 s) 동안 RECONCILE과 그래프 검사가 실패-폐쇄한다(재시도 필요).
6. 실차: 모터 잠금, 실차 Mapping START 차단, STM32 timeout·E-Stop·제동 거리 미측정, Watchdog 실차 launch 미편입 상태가 유지된다.
