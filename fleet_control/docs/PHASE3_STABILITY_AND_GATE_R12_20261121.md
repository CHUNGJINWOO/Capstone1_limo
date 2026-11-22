# Phase 3 불안정성 분석과 Gate r12 통합 준비 (2026-11-21, 격리 ROS만)

이 작업에서 하지 않은 것:
- LIMO 연결·SSH 없음
- 18190 운영 서버 변경 없음
- 실차 모터 잠금·Mapping START 차단 유지
- 정식 Gate(field·`capstone_ws`) 미수정. 다른 에이전트의 미커밋 8줄 diff는 그대로 보존(`other_agent_gate_diff.patch`)
- 프로세스 종료는 내가 띄운 프로세스만 했다(경로·PID 확인 후). 전역 pgrep 종료 없음
- 원본 결과 보존(`local_data/raw_results/phase3_stability_claude_20261121/`, 이전 결과 디렉터리 미변경)
- commit·push·PR 없음

## 1. 보존·해시

작업 전 코드·설정·결과 해시를 `snapshot_before.sha256`(411개 파일)에 기록했다.
- 기존 Gate Reset 결과 1,608개 파일은 변경하지 않았다.
- 수정한 모든 파일의 원본은 같은 디렉터리의 `*.before_*`에 있다.

## 2. 계측

- **`tests/nav_e2e/timeline_recorder.py`** (시험 전용, 하네스 소유 자식 프로세스): 0.2 s마다 다음을 한 줄씩 기록한다.
  - Fleet: 모드, 전환, operation_revision, Agent·Gate boot, Gate 상태·fault epoch·차단 사유·Reset 판단, Watchdog, Arbiter, localization(상태·match·AMCL pose·공분산), 소유 Goal
  - 서버: `safety_context`의 **최초 실패 검사**, Gate boot 재조정 플래그
  - ROS 직접: `/amcl_pose` 나이·공분산, `/scan` 수신·stamp 나이, TF `map→base`·`odom→base` 나이, `/cmd_vel_locked` 마지막 수신·마지막 비영 시각, NavigateToPose 상태 배열, 주요 토픽 Publisher 수와 **그래프 조회 지연**, 기록기 자신의 **타이머 지터**
- **서버 진단**(판정 불변):
  - `safety_context`의 23개 실패 지점마다 사유를 기록해 `robot_view.safety_context_reason`으로 보고한다.
  - `gate_boot_reconciliation` 플래그를 노출한다.
- **분석:** `tests/nav_e2e/analyze_timeline.py`가 실패 시나리오마다 구간 시작 시점의 차단과 **최초 차단 지점**을 추출하고, 실행별 지연·나이 최댓값을 계산한다.

## 3. 원인 (재현 조건 · 최초 차단 지점)

기준 시리즈(`baseline`, r11·r12 교대 6회, 매 실행 p01에서 지도를 새로 만듦)는 r11 15/3·18/0·13/5, r12 18/0·7/11·15/3이었다. 실패를 모두 아래 원인으로 분류했다. **r12에만 나타난 원인은 없었다.**

### A. 매 실행 다른 지도
- **재현 조건:** 매 실행 p01이 새 지도를 만든다.
- **최초 차단 지점:** 지도 생성 단계에서 시작한다. baseline 04_r12에서 SLAM 기동 순간 Gate가 `scan_stale` FAILSAFE를 래치했다(E). p01 동안 로봇이 움직이지 못해 **지도가 거의 비었고**(점유 9셀, 다른 실행은 약 450셀), 이후 모든 localization이 실패했다.
- **Gate 관련성:** 지도 품질 자체는 Gate와 무관하다.
- **조치:** 하네스 고정 지도 모드(`NAV_E2E_FIXED_MAP`). 비교 시리즈는 같은 지도·같은 시작 자세로 실행한다. 웹 맵핑은 Phase 2가 따로 검증한다.

### B. 탐색 종료 위치에 따른 시작 자세 차이
- **재현 조건:** 탐색이 끝난 위치가 실행마다 다르다.
- **최초 차단 지점:** n03 주행 중 scan/map match가 0.48에서 0.32로 떨어졌다(05_r11). `LOCALIZATION DEGRADED`로 임무가 취소되었다. 기존 규칙(0.40 미만 3 s)이 정상 동작한 결과다.
- **Gate 관련성:** 없음. r11·r12 모두에서 발생했다.
- **조치:** 고정 지도 모드에서는 모든 실행이 원점에서 시작한다. localization 기준은 바꾸지 않았다.

### C. 재LOAD 후 AMCL이 초기 위치를 못 받음
- **재현 조건:** 재LOAD 직후 초기 위치를 보낸다.
- **최초 차단 지점:** 01_r11 n05에서 새 AMCL이 `/initialpose`를 구독하기 전에 Agent가 0.6 s 동안 3회 발행했다. AMCL이 50 s 이상 pose를 내지 않았고 `CONVERGING`이 지속되었다.
- **Gate 관련성:** 없음(Agent 문제).
- **조치:** Localization Worker를 수정했다.
  - AMCL 구독을 확인한 뒤 발행한다(최대 5 s).
  - AMCL 응답이 없으면 **같은** 운영자 자세를 3 s마다 최대 5회 다시 보낸다.

### D. Gate boot 변경 후 서버가 영구 차단
- **재현 조건:** Gate가 재시작한다(스택 재시작·Gate kill).
- **최초 차단 지점:** 서버 `safety_context`의 **`GATE_BOOT_RECONCILIATION`**이다.
  - 새 Gate boot가 보이면 서버가 영구 플래그를 남긴다.
  - 그 플래그를 지우는 코드가 **어디에도 없었다**. 관리자 `/recover`도 `not_recoverable`로 응답했다.
  - 그래서 하네스의 Nav2 기동 가드가 스택을 재시작한 실행은 이후 모든 임무가 `safety_not_ready`였다(이전 보고의 n03).
- **Gate 관련성:** 없음(서버 문제).
- **조치:** 기존 관리자 복구(`POST /robots/{id}/recover`, 관리자·사유 필수)에 재조정 경로를 추가했다.
  - 조건: 증거 기반. Agent 온라인·재동기화, 보고 2 s 이내, 새 Gate NORMAL·래치 없음, 소유·활성 Goal 없음, Watchdog 승인.
  - 처리: 새 boot를 관찰 boot로 채택하고, 이전 Admission을 무효화하고, 감사 이벤트를 남긴다.
  - 하네스는 이 기록된 복구를 사용한다.

### E. Gate 프로세스 일시 정지 → 센서 FAILSAFE 래치
- **재현 조건:** SLAM 기동, Agent 재시작, map_server·AMCL 기동처럼 새 DDS participant가 들어오는 순간이다.
- **최초 차단 지점:** Gate의 `scan_stale`(또는 `lidar_odom`·`est_odom`) FAILSAFE이며, `require_operator_unlatch`로 래치된다. 예시: final_series 01_r11 n05, LOAD 시작 시점.
  - 기록기는 `/scan`을 0.08 s 간격으로 계속 받았다.
  - 같은 순간 **Gate 자신의 10 Hz 상태 발행이 약 0.35~0.4 s 멈췄다**.
  - 즉 센서가 멈춘 것이 아니라 **Gate 프로세스(단일 스레드 executor)가 멈췄다**. 재개 직후 제어 루프가 0.38 s 기준을 넘긴 scan을 stale로 판정한다.
- **Gate 관련성:** Gate 내부 문제지만 r12와는 무관하다.
  - 평균 CPU는 r11·r12 모두 14%로 같았다.
  - Gate가 재시작하지 않은 채 상태 발행이 0.3 s 이상 멈춘 자발적 정지: r11 8회 중 2건(0.33 s ×2), r12 8회 중 1건(0.37 s). 드물고 두 Gate 모두에서 나타난다. baseline 04_r12의 SLAM 기동 시 정지는 약 0.28 s로 이 기준 바로 아래였다. 처음에 "실행당 약 3회"로 셌던 값은 Gate kill·재시작에 따른 정지 기간을 포함한 오집계였다.
  - 이 작업 전에 저장된 격리 실행의 센서 FAILSAFE 로그: r10 21회 중 10건, r11 15회 중 0건, r12 19회 중 5건. r11 0건은 표본 차이일 수 있고, final_series에서 r11 01회에 5건이 나왔다.
- **원인 상세:** 콜백 시간 진단(§6.2)으로 확인한다.
- **조치:** 아직 수정하지 않았다(§9).
## 4. r11 대 r12 판정

**판정: 이 격리 환경에서 r12가 r11 대비 회귀를 일으킨다는 증거는 없다.** 정상 동작과 장애 대응이 같은 입력에서 같았다. 아래는 같은 스냅샷 결과다.

| 비교 항목 | 시험 | r11 | r12 |
|---|---|---|---|
| Reset 없는 일반 Navigation | Phase 3 n01~n04 (`final_series`, 고정 지도, 교대 8회) | 4회 모두 PASS | 4회 모두 PASS |
| Reset 이후 Navigation | Phase 3 f07→f08 (Gate 래치 후 기록된 Reset → 임무) | 4회 중 4회 | 4회 중 4회 |
| Watchdog 재시작 후 복구 | Phase 2 f05, Phase 1 s14 | PASS | PASS |
| MANUAL → NAVIGATION / NAVIGATION → MANUAL | Phase 2 n10 / n09 (closing), Phase 3 n05 (final_series) | Phase 2 PASS. Phase 3 n05 3/4 (1회 E: Gate 정지 → 센서 FAILSAFE) | Phase 2 PASS. Phase 3 n05 4/4 |
| Gate boot 변경 이후 서버 재조정 | r01 재현(스택 재시작 → 거부 `GATE_BOOT_RECONCILIATION` → 관리자 복구 → 임무 SUCCEEDED) | PASS | PASS |
| 외부 `/cmd_vel_raw` 차단 | Phase 3 f07b, Phase 1 s18 | rogue 값 0 | rogue 값 0 |
| STOP·Deadman 해제 | Phase 1 s03·s05 | PASS | PASS |

**같은 입력에서 생긴 실패와 Localization 변동 실패를 분리한 결과:**
- `final_series` 8회 중 실패는 r11 01회의 n05 1건뿐이다. 원인은 Gate 프로세스 정지에 따른 센서 FAILSAFE(E)로, r11에서 발생했다.
- 지도·시작 자세를 고정하자 기존 A·B 유형의 localization 실패는 사라졌다.
- C(재LOAD 초기 위치)와 D(Gate boot 재조정)는 수정 후 재발하지 않았다.

**r12가 바꾸는 것(의도된 동작):**
- Reset 직전 1 s 동안 운동 명령이 있으면 Reset을 거부한다(`MOTION_COMMANDED`). Phase 3 f08 직전 f07의 rogue 운동 명령 때문에 첫 Reset이 거부되었고, 하네스 재시도에서 수락되었다.
- Reset 조건 판단을 상태 JSON에 매 주기(10 Hz) 넣는다. 그래프 조회가 추가되지만 같은 스택에서 Gate CPU 차이는 없었다(14% 대 14%).

**미판정으로 남는 것:**
- Gate 프로세스 정지(E)의 정확한 원인 콜백. 진단 실행 1회에서는 자발적 정지가 일어나지 않아 원인을 특정하지 못했다.

## 5. 수정한 코드와 하네스

원본은 모두 `local_data/raw_results/phase3_stability_claude_20261121/*.before_*`에 있다.

| 파일 | 변경 | 원인 |
|---|---|---|
| `server/fleet_server/safety.py` | `safety_context` 실패 지점별 사유 기록(판정 불변) | 진단 |
| `server/fleet_server/service.py` | `robot_view.safety_context_reason`, `gate_boot_reconciliation` 노출 | 진단·D |
| `server/fleet_server/ownership.py` | `/recover`에 증거 기반 Gate boot 재조정(`gate_boot_evidence`, `_reconcile_gate_boot`) 추가, 감사 이벤트 `gate_boot_reconciled`·`gate_boot_reconcile_refused` | D |
| `agent/.../localization_worker.py` | (1) AMCL 구독 확인 후 초기 위치 발행, 같은 자세 재전송 최대 5회 (2) LOAD 때 planner 매개변수를 못 바꿔도 실패하지 않음. **CONFIRMED는 `allow_unknown=false` 확인 후에만** | C, 고정 지도 교착 |
| `server/fleet_server/static/gate_reset_panel.js` | 재시도 가능 여부(지금 가능·조건 해소 대기·조치 필요·불필요), Gate 최근 Reset 판단·거부 횟수, Gate 조건 안내(1 s 0 명령) | §4 Reset 절차 |
| `tests/nav_e2e/run_nav_e2e.py` | 시간 기록기 연결, 시나리오 시각 표시, `NAV_E2E_FIXED_MAP`, recovery_guard의 Gate boot 재조정, r01 재현 시나리오(기본 순서 밖) | 계측·A·B·D |
| `tests/nav_e2e/{timeline_recorder.py, analyze_timeline.py}`(새 파일) | 계측·분석 | §2 |
| `tests/manual_e2e/run_manual_e2e.py` | s10: 진단 기록. 목표 직전 비영·직후 첫 샘플 0이면 그 샘플 시각을 정지 상한으로 판정(0.7 s 한도 불변) | s10 |
| `tests/{test_gate_boot_reconcile.py, test_localization_initial_pose.py}`(새 파일), `test_gate_reset_panel.cjs` | 단위 시험 | — |
| `experiments/.../harness/gate_r12_reset/{gate_callback_timing.py(새 파일), test_gate_r12_reset_unit.py(GATE_SRC)}` | 진단·제안 06 사본 시험 | E·§8 |
| `docs/proposals_20261120/06_gate_input_and_reset_preconditions.patch`(새 파일), README | 정식 Gate 통합 제안 | §8 |

**Gate r11·r12 코드는 이번 작업에서 수정하지 않았다**(해시 동일).

## 6. 반복시험 결과

### 6.1 Phase 3

| 시리즈 | 조건 | r11 | r12 |
|---|---|---|---|
| `baseline` (수정 전, 매 실행 새 지도) | 교대 6회 | 15/3, 18/0, 13/5 | 18/0, 7/11, 15/3 |
| `final_series` (모든 수정, 고정 지도, 스냅샷 동일) | 교대 8회 | **17/1, 18/0, 18/0, 18/0** | **18/0 ×4** |

`after_fix/01_r11`(7/11)은 고정 지도 교착을 발견한 실행이라 결과로 세지 않고 보존만 했다.

### 6.2 Gate 콜백 시간 진단

`closing/diag_timing_r12`: r12를 수정 없이 감싸 30 ms 이상 콜백과 100 ms 이상 executor 공백을 기록했다(18/0).
- 이 실행에서는 자발적 Gate 정지가 없었다. 기록기 기준 0.3 s 이상 공백은 f09의 의도된 Gate kill뿐이었다.
- 따라서 기록이 0건이고 원인 콜백은 미확인이다.

### 6.3 s10 FollowPath 중단 (별도 회귀 항목)

- `closing/phase1_r11`: 진단으로 원인을 확인했다. 목표 8 ms 전까지 출력이 (0.1, 0.15)였고 목표 후 첫 샘플(+13 ms)이 이미 0이었다. 첫 샘플보다 빨리 멈춘 경우를 하네스가 `None`으로 처리해 FAIL이 된 측정 결함이다.
- 수정 후 `closing_s10`: r11·r12 각 2회, 전체 순서·브라우저 포함 **19/19 ×4**. 그중 1회(r11)가 바로 이 경우였다(정지 상한 0.017 s).

## 7. 기존 Phase 1·2·Gate Reset 회귀 (`closing`, 스냅샷 동일)

| 시험 | r11 | r12 |
|---|---|---|
| Phase 1 + 브라우저 | 18/1(s10 측정 결함, §6.3) → 수정 후 19/19 ×2 | 19/19, 수정 후 19/19 ×2 |
| Phase 2 원래 17 | 17/17 | 17/17 |
| Gate Reset 격리 E2E | 해당 없음(r11은 Reset 판단이 없음) | 13/13 |
| r01 Gate boot 재조정 | 5/5 | 5/5 |
| 단위·회귀 전체 | 기존과 동일 | 새 시험 전부 PASS |

기존 스텁 필요 2개 파일의 오류는 변경 전과 같다.

## 8. 정식 Gate 통합 가능 여부

**아직 정식 적용은 하지 않는다. 담당자 검토용 제안은 준비되었다.**

| 구분 | 상태 |
|---|---|
| 격리 ROS 검증 | 완료 |
| Gazebo | 미검증 |
| 실차 | 미검증 |
| DDS 출처 검사 | 인증이 아니다(그래프·GID·participant 일치). 위조 단독 발행자 차단에는 SROS2가 필요하다 |
| 물리 안전 | 미완료(STM32 명령 timeout, E-Stop, 제동거리) |

**제안 06**(`docs/proposals_20261120/06_gate_input_and_reset_preconditions.patch`, 미적용):
- 05를 포함하고 r12 Reset 판단을 이식했다.
- 새 매개변수는 모두 기본 꺼짐이다.
- 다른 에이전트의 미커밋 `max_abs_linear_x` 수정 위에 `git apply --check`가 통과한다(충돌 없음).
- 시험: 패치 사본으로 r12 Reset 판단 단위 14/14, 입력 출처 단독 10종에서 r11·r12와 같은 결과.

**검증 계획:**
1. 담당자 검토
2. 06 적용
3. 격리 재시험 묶음(단위, 단독 10종, Phase 1·2·3, Gate Reset E2E)
4. 실차 launch 매개변수는 별도 제안으로 한다(`input_exclusive_publishers`, `reset_requires_preconditions`, `reset_input_peer_topic`)
5. Gazebo
6. 실차 비구동 bringup
7. 물리 시험

**적용 전제:**
- 아래 §9-1(Gate 프로세스 정지)의 원인 확인 또는 완화 설계가 필요하다.
- 정지 중에는 Gate가 0 명령도 내지 않으므로 실차에서는 하부 timeout에 의존하게 된다.

## 9. Gazebo 통합 전 남은 차단 조건

1. **Gate 프로세스 일시 정지(E).**
   - 원인 콜백이 미확인이다.
   - 영향: 정지 동안 출력이 멈추고, 재개 직후 센서 FAILSAFE가 래치된다.
   - 다음 조치: 진단 래퍼로 반복 실행해 정지를 포착한다. 후보는 그래프 조회와 DDS participant 발견 처리다.
   - 그 뒤 판정 완화 없는 설계를 검토한다(예: 자기 정지 감지 시 0 출력 유지 + 정지 사실을 별도 사유로 기록).
2. Agent heartbeat 여유 부족(SAVE 중, 이전 보고).
3. Gazebo 연결 지점(이전 보고 §6):
   - bridge의 `/cmd_vel_locked`
   - Nav2 출력 → Arbiter 입력
   - sim time 감사
   - Localization Worker와 office launch의 AMCL 중복 제거
4. 웹 맵핑 없이 저장 지도로 바로 주행하는 흐름(고정 지도)은 이번에 처음 격리 검증했다. 실차에서도 기동 직후 같은 planner 대기가 있을 수 있다.
5. 인증(SROS2)과 물리 안전 측정은 남아 있다.

## 10. Codex 독립 검증 명령

```bash
cd <repo>; source /opt/ros/humble/setup.bash; source capstone_ws/install/setup.bash
export PYTHONPATH=$PWD/fleet_control/agent/limo_robot_agent:$PWD/fleet_control/server
python3 fleet_control/tests/test_gate_boot_reconcile.py
python3 fleet_control/tests/test_localization_initial_pose.py
node fleet_control/tests/test_gate_reset_panel.cjs
GATE_SRC=local_data/raw_results/phase3_stability_claude_20261121/proposal06/stall_detector_node.capstone_ws_plus_p06.py \
  ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=229 python3 experiments/safety_chain_fix_20261113/harness/gate_r12_reset/test_gate_r12_reset_unit.py
git apply --check fleet_control/docs/proposals_20261120/06_gate_input_and_reset_preconditions.patch
# Phase 3 교대 시리즈(고정 지도). 새 디렉터리에만 기록
NAV_E2E_FIXED_MAP=local_data/raw_results/phase3_stability_claude_20261121/fixed_map \
  local_data/raw_results/phase3_stability_claude_20261121/series.sh <new_out_dir> "r11 r12 r11 r12"
python3 fleet_control/tests/nav_e2e/analyze_timeline.py <new_out_dir>/01_r11 <new_out_dir>/02_r12
# 회귀 묶음: local_data/raw_results/phase3_stability_claude_20261121/closing_chain.sh
```

## 11. 공통 문서

HANDOFF·VALIDATION·CODE_INDEX·CURRENT_STATUS에 짧은 절을 추가했다(원본 보존). DECISIONS는 변경하지 않았다(설계 결정 승인 없음).
