# Gazebo 웹 관제 안정화와 Gate 독립 정지 검증 (2026-11-21, Claude)

이 문서는 [GATE_STALL_GAZEBO_20261121.md](GATE_STALL_GAZEBO_20261121.md)의 후속이다.

범위:
- 모든 결과는 격리 Gazebo(Ignition Fortress 6.18, Office 월드, `limo_eval_diff`, ROS 도메인 224, 포트 28961, 실행별 `IGN_PARTITION`)이다.
- 실차·LIMO SSH·운영 18190은 사용하지 않았다. 실차 모터 잠금과 실차 Mapping START 차단은 그대로다. 실차 펌웨어·구동 코드는 바꾸지 않았다.
- Gate는 candidate r12 원본이다. 다음은 수정하지 않았다:
  - r11/r12/패치 06 원본
  - 다른 담당자의 `capstone_ws` Gate 변경
  - Frontier C++ 노드 `fleet_frontier_candidates.cpp`(담당자 검토 대기 목록)
  - `rmf_adapter/`(다른 작업자가 오늘 수정 중)
- `fleet_control/`은 이 브랜치에서 미추적이다. 변경 전 원본은 `local_data/raw_results/refactor35_claude_20261121/pre_change_originals/*.before_*`에 있다.
- 원자료(`RAW/` = `local_data/raw_results/`). 실행마다 `stack/env.sh`·`cmds.sh`(명령·환경·도메인), Gate/Agent/Gazebo 로그, `result.json`, `samples.json`(명령·Gate 출력·Gazebo truth), 저장 지도를 보존했다.

## 1. 실제 수정 파일

| 파일 | 내용 | 성격 |
|---|---|---|
| `fleet_control/tests/gazebo_web/drive_timeout_relay.py`(신규) | Gate와 독립인 하위 명령 timeout 프로세스. 아래 2절 | 시뮬 전용 |
| `fleet_control/tests/gazebo_web/gazebo_world.launch.py` | `GZ_DRIVE_TIMEOUT_S`(기본 0.2). 0이면 기존 직결(hold) | 시뮬 전용 |
| `fleet_control/tests/mode_e2e/stack_up.sh` | `GZ_DRIVE_TIMEOUT_S`를 env.sh로 전달(Gazebo 백엔드만) | 시험 |
| `fleet_control/server/fleet_server/static/manual_control_panel.js` | 시작 시 증거가 0.5 s보다 오래되면 1회 갱신 후 판단. `active()` 노출. 증거 한계 1000 ms는 그대로 | **제품(웹)** |
| `fleet_control/server/fleet_server/static/app.js` | 수동 세션 중 `/dashboard` 300 ms 갱신. 패널에 `refreshEvidence` 제공 | **제품(웹)** |
| `fleet_control/agent/limo_robot_agent/limo_robot_agent/frontier_policy.py`(신규) | Frontier 목표 선택 규칙(순수 함수) | **제품(Agent)** |
| `fleet_control/agent/limo_robot_agent/limo_robot_agent/mapping_worker.py` | `offer()`가 위 규칙을 사용. 결과 기록·결정 이력(`recent_decisions`)·`EXHAUSTED`/`GOALS_FAILING` 보고 | **제품(Agent)** |
| `fleet_control/tests/test_frontier_policy.py`(신규) | 정체 사례 포함 6건 | 시험 |
| `fleet_control/tests/gazebo_web/run_gazebo_web.py` | d01~d06, g07, g08 추가. g00 Nav2 기동 재시도. g03 종료 원인 분류 | 시험 |
| `fleet_control/tests/gazebo_web/browser_map_click.cjs`(신규), `fleet_control/tests/manual_e2e/browser_pointer_diag.cjs`(신규) | 실제 대시보드 지도 클릭 / 포인터 진단 | 시험 |
| `experiments/…/gate_r12_reset/gate_proc_watch.py`(신규) | Gate 프로세스를 외부에서 `/proc`으로 관찰(Gate 안에 코드 없음) | 진단 |

## 2. Gate 정지 시 하위 구동 보호(최우선)

### 2.1 현재 브리지 동작 확인

시험 방법(`RAW/gazebo_drive_timeout_claude_20261121/bridge_hold_probe`): Gazebo만 띄우고 "Gate" 역할로 0.1 m/s를 1.5 s 보낸 뒤 침묵했다.

| 브리지 | 침묵 후 3 s 이동 | 끝에서 속도 | 첫 0 명령 | 정지 |
|---|---|---|---|---|
| 직결(기존) | 0.302 m | 0.1 m/s(계속 주행) | 없음 | 정지 안 함 |
| 하위 timeout 0.2 s | 0.027 m | 0 | 0.263 s(점검 20 Hz일 때. 이후 50 Hz로 변경) | 0.279 s |

Ignition DiffDrive는 마지막 명령을 무기한 유지한다.

### 2.2 설계

```text
Gate r12 --/cmd_vel_locked--> drive_timeout_relay(별도 프로세스) --/sim/drive_cmd--> ros_gz_bridge --> DiffDrive
```

- Gate 출력을 받는 즉시 그대로 전달한다. 제한이나 정책은 없다. 안전 판단은 Gate만 한다.
- **자기 프로세스의 steady clock 타이머(50 Hz)**로 마지막 Gate 출력의 나이를 본다. 0.2 s를 넘으면 0을 계속 보낸다. Gate 타이머·`/clock`에 의존하지 않는다.
- 상태: `/sim/drive_timeout/status`.
- 한계: 이 프로세스가 멈추면 Gazebo는 다시 마지막 명령을 유지한다. 실차 MCU timeout과 같은 단일 지점이다.
- 실차 대응 부품이 아니다. 실차 STM32 timeout 값은 **미측정**이다.
- 잠금 그래프 계약은 그대로다. Arbiter는 `/cmd_vel_locked` writer(Gate 1개)만 본다. g00에서 reader = `sim_drive_timeout`, `/sim/drive_cmd` writer = `sim_drive_timeout` 1개, reader = `gz_bridge`를 확인했다.

### 2.3 결과

조건: 웹 MANUAL로 0.1 m/s 전진 중 사건을 일으켰다. 측정 정의는 다음과 같다.
- "하위 0" = `/sim/drive_cmd` 첫 0
- "정지" = Gazebo truth가 처음으로 |v|<0.01 m/s, |w|<0.02 rad/s가 되어 0.3 s 유지된 시각
- "이동" = 사건부터 그 정지 시각까지의 거리
- `RAW/gazebo_drive_timeout_claude_20261121/timeout_0.2_run{2,3}`, `timeout_0_hold_run1`

| 사건 | timeout 0.2 s(2회): 하위 0 / 정지 / 이동 | timeout 없음(hold, 1회): 정지 / 이동 |
|---|---|---|
| Gate SIGSTOP 0.35 s | 0.213–0.217 s / 0.241–0.246 s / 0.024 m | 0.394 s / 0.038 m |
| Gate SIGSTOP 0.5 s | 0.191–0.199 s / 0.218–0.240 s / 0.020–0.023 m | 0.544 s / 0.054 m |
| Gate SIGSTOP 2 s | 0.191–0.198 s / 0.223–0.235 s / 0.021–0.022 m | 2.047 s / 0.204 m |
| Gate + Watchdog SIGSTOP 2 s | 0.189–0.207 s / 0.223–0.232 s / 0.022 m | 2.045 s / 0.204 m |
| Gate SIGKILL | 0.213 s / 0.238–0.255 s / 0.022–0.023 m | **정지 안 함**(3 s 동안 0.30 m, 끝에서도 0.1 m/s) |
| Watchdog SIGKILL(Gate 살아 있음) | Gate가 스스로 0 출력. 하위 timeout은 동작하지 않음: 0.791–0.803 s / 0.828–0.837 s / 0.081 m | 0.839 s / 0.084 m |

해석:
- 하위 timeout이 있으면 Gate가 멈추든 죽든 이동은 정지 길이와 무관하게 약 0.02 m이다(0.1 m/s × 약 0.2 s).
- 없으면 정지 시간 내내 움직인다. Gate가 죽으면 끝없이 움직인다.
- **Gate 정상 경로 유지**: 정상 주행 g01(Deadman 해제 → Gate 0 0.12 s, 정지 0.16 s), 포인터·키보드·게임패드(4절), Nav2 목표(5절)는 모두 timeout 프로세스를 거쳐 동작했다.
- 정지 사이에 Gate 출력은 0개였다.
- **Gate 발견(r12, 수정 안 함)**:
  - 2 s 정지에서 재개된 직후 Gate의 **첫 출력이 2 s 전 명령(0.1 m/s)**이었다(6/6). 다음 tick(약 13 ms 뒤)에 `FAILSAFE_SENSOR_TIMEOUT`로 래치된다.
  - 하위 timeout은 이 값을 그대로 전달한다(정책은 Gate 몫). 그 결과 재개 후 추가 이동은 0.001~0.002 m였다.
  - 제안(담당자 검토): 제어 루프 간격이 센서 timeout을 넘었으면, 그 tick은 센서 판정 전에 0을 출력한다.
- 1회차(`timeout_0.2_run1`)의 d03/d04 FAIL은 측정 정의 오류였다(재개 후 움직임까지 "마지막 움직임"으로 셈). 정의를 고친 뒤 2·3회차는 모두 PASS다. d06 1회차 FAIL은 Watchdog 재시작 고스트 래치가 늦게 생긴 탓이며, 대기 보강 후 해소됐다.

## 3. Gate E(정지 원인) — 범위 제한

- 이번에는 Gate **밖**에서 보는 `gate_proc_watch.py`를 추가했다. 20 ms마다 Gate 스레드 상태(R/S/D/T), `run_delay`, 메인 스레드 CPU를 읽는다.
- 회귀 묶음(Phase 2·3·Gate Reset)과 함께 실행했다. **E는 재현되지 않았다**(8절). 원인은 미해결로 남긴다.
- 하위 timeout(2절)은 E와 무관하게 Gate 정지 중 이동을 약 0.02 m로 묶는다(시뮬).

## 4. 웹 포인터 입력

### 4.1 손실 지점(관측)

`browser_pointer_diag.cjs`(15주기 × 1.5 s 누름 + 밖으로 끌기·STOP·touch cancel)를 실행하고, 각 단계의 시각을 하네스(`g07`)가 Agent Arbiter 상태·Gate 출력·Gazebo와 맞췄다.

| 단계 | 수정 전(`before_fix_run2`) |
|---|---|
| 브라우저 `pointerdown` / `gotpointercapture` | 15/15 수신 |
| 패널 → HTTP sample(Deadman) | 12/15. 3주기는 누르기 전에 세션이 이미 끝나 0개 |
| HTTP 응답 | 전부 200(지연 41~54 ms) |
| Agent/Arbiter → Gate 출력 | sample이 간 주기는 모두 출력 |
| 끝까지 유지 | 3/15만 1.5 s 유지. 나머지 9주기는 누르는 중에 `최신 안전 증거 없음`으로 정지 |

**손실 지점은 브라우저 패널의 `tick()`**이다. HTTP·Agent·Arbiter에서는 손실이 없었다.
- 패널은 증거 나이가 1000 ms를 넘으면 세션을 멈춘다.
- 증거 나이 = 서버의 상태 나이 + 마지막 `/dashboard` 이후 시간인데, 대시보드는 1 s마다 갱신된다(측정 간격 중앙값 1.0 s, 최대 1.074 s, 1 s 초과 20회).
- 그래서 갱신 직전마다 한계를 넘었다. 키보드·게임패드도 같은 위험이 있다(시험 누름이 0.9 s로 짧았을 뿐).

### 4.2 수정과 결과

수정 내용:
- 세션 중에는 300 ms마다 증거를 갱신한다.
- 시작 시 증거가 0.5 s보다 오래되면 1회 갱신한 뒤 같은 1000 ms 기준으로 판단한다.
- 한계 자체는 완화하지 않았다.

| 판 | 결과 |
|---|---|
| 수정 v1(300 ms 갱신, 1 s tick 중에는 건너뜀) | 10/15·12/15. 시작 순간 stale |
| 수정 v2(+시작 시 stale이면 갱신) | 15/15, 13/15. 남은 2건은 시작 37 ms 뒤 한계 초과(사건 시각열로 확인) |
| **수정 v3(+시작 시 0.5 s 넘으면 갱신)** | **15/15 × 3회(45/45)**. 매 주기 Gate 출력 68~76개, 해제 0.45~0.85 s 뒤 0 출력, 대시보드 갱신 간격 중앙값 0.26~0.30 s(최대 0.996 s, 세션 사이) |

- 밖으로 끌기: 캡처가 유지되어 계속 동작했다. 놓으면 0.
- STOP 버튼: 이후 0 출력이 0개였다.
- touch cancel(`pointercancel`): 이후 0 출력이 0개였다.
- 키보드·포인터·게임패드 g02: v3 유효 실행 3/3 PASS.
- 패널 브라우저 시험 `test_manual_control_panel.cjs`(Playwright 경로 지정 시): PASS. 마우스·터치·게임패드·탭 전환·서버 재시작에서 자동 재개하지 않음을 확인했다.
- 수정 전 1회차 결과(15/15 `BROWSER_NO_POINTERDOWN`)는 진단 스크립트 결함이었다. 시작 전에 계산한 조이스틱 위치(y≈2953, 화면 밖)를 썼다.
- `after_fix3_run3`은 Nav2 기동 실패(알려진 lifecycle 간헐)로 무효다. g00에 재시도를 추가한 뒤 run4로 대체했다.

## 5. Frontier 자동 맵핑

### 5.1 원인과 수정

원인(이전 보고 6.1):
- 넓은 공간의 첫 scan에서 후보가 로봇 0.25 m 앞 1개뿐이었다.
- 그 목표로는 SLAM의 0.2 m / 0.2 rad 갱신 기준을 넘지 못했다.
- Worker는 시도한 셀을 다시 쓰지 않아 정체했다.

`frontier_policy.py`의 규칙:
1. 로봇 0.6 m 이내 후보는 건너뛴다(TOO_CLOSE).
2. SUCCEEDED 목표 0.5 m 이내 후보는 건너뛴다(VISITED).
3. 실패(FAILED/CANCELED/REJECTED) 0.5 m 이내 후보: 1회 실패 후 20 s 뒤 1회 재시도를 허용하고, 2회 실패하면 제외한다.
4. 쓸 후보가 없으면 SLAM 지도에서 0.6~1.5 m 거리의 여유 공간 셀(점유·미관측에서 0.3 m)을 고른다. 주변 미관측이 가장 많은 셀로 REPOSITION 목표를 낸다(최대 12회).
5. 그것도 없으면 `EXHAUSTED`. 연속 3회 실패면 `GOALS_FAILING`(막힘)이다. 둘 다 목표 제안만 멈춘다. 완료를 주장하지 않으며 SAVE/STOP은 운영자가 한다.

Nav2 heading 설정(`yaw_goal_tolerance` 3.14, `use_rotate_to_heading: false`, 공용 설정이라 바꾸지 않음) 때문에 제자리 회전 목표는 쓸 수 없다.

### 5.2 반복 결과(같은 월드)

| 묶음 | 스폰 | 탐색 거리(실행별) | 종료 원인 |
|---|---|---|---|
| 수정 전(이전 보고) | patrol_A1 | 0.0, 0.0, 0.1, 0.0 + 1.81, 13.85 | 4/6 정체 |
| 수정 후 v1 | patrol_A1 | 0.85, 2.06, 2.05 m | Gate STALL 래치(물리 막힘) |
| 수정 후 v2/v3 | patrol_D2 | 6.69, 2.83, 14.94, 4.63, 17.08 m | TIME_LIMIT 3, 물리 막힘 2 |

- 수정 후 8/8 실행이 탐색을 이어갔다.
- v3 두 실행 모두 **첫 결정에서 `TOO_CLOSE: 1`로 근접 후보를 건너뛰고** 다음 후보로 갔다. 이전 정체 조건에서 규칙이 작동한 것을 기록으로 확인했다.
- REPOSITION·EXHAUSTED·GOALS_FAILING은 이 Gazebo 실행에서 발생하지 않았다. 단위 시험으로만 확인했다.
- **막힘 vs 정체 구분**:
  - g03은 탐색 중 Gate `STALL_LATCHED`가 있으면 `BLOCKED_PHYSICAL`로 기록한다.
  - patrol_A1 막힘 위치는 의자 `OfficeChairGrey_11`에서 0.49 m, 테이블 `AdjTable`에서 0.63 m였다. 의자 다리·테이블 발판이 LiDAR 평면(약 0.17 m) 아래라 보이지 않는 것으로 **추정**한다.
  - 그때 Gazebo truth 속도는 약 0이었다. Gate STALL은 참 양성이다.
- patrol_D2에서는 다른 문제가 나왔다.
  - 복도에서 ICP가 진행 방향 이동을 못 봐서 Gate `STALL_LATCHED`가 **거짓 양성**으로 났다. truth 0.1 m/s인데 LiDAR 속도는 0.012였다.
  - `FAILSAFE_ICP_DEGRADED`(inlier 98~99 < 100)도 났다. 안전 방향이지만 복도 운용성 문제다. Gate 담당자 검토 대상이다.
- 스폰 변경(A1 → D2)은 월드·지도 동일, 시작 위치만 다르다. 각 묶음 안에서는 고정했다.

## 6. 실제 브라우저 지도 클릭 E2E(g08)

흐름:
1. 실제 대시보드 지도 위에서 UI 커서 표시값 3점으로 화면 좌표→세계 좌표를 계산한다.
2. 그 지점을 클릭하고, Goal Drawer에서 사전 검사를 거친 뒤 확인 → "임무 생성"을 누른다.
3. 요청은 `/missions` → Agent → Nav2 → Gazebo 이동으로 이어지고, UI 표시를 확인한다.

결과(실행 8회 중 7회 판정. `v3_D2_run2`는 Localization CONVERGING으로 g08 전에 중단):

| 확인 | 결과 |
|---|---|
| 클릭 좌표 = 의도 좌표 | 7/7. 커서 표시값 = 목표, 오차 ≤0.01 m |
| 지도 밖 / 장애물 / 미관측 | 7/7. UI 사전 검사가 차단했다("지도 경계 밖", "점유 셀", "미관측(UNKNOWN) 셀"). 임무 0건 |
| Localization 미확인 | 4/4(v2·v3). UI 사전 검사를 통과한 셀을 제출하면 서버가 거부했고, UI에 "localization on the loaded map is not CONFIRMED"가 표시됐다. 임무 0건. v1 3회는 그 셀이 UI 사전 검사에서 먼저 막혀 이 경로를 시험하지 못했다(하네스 v1 결함) |
| 정상 목표 → 완료 | **1회 완료**(`v2_D2_run3`): 목표 (0.48, 0.01), Gazebo 최종 (0.478, −0.044), 오차 0.055 m, SUCCEEDED |
| 미완료 원인 | v1 3회: 하네스가 서버 기준 "유효" 지점 1개만 시도했고 UI 사전 검사에 막혔다(v2에서 후보 탐색으로 수정). 나머지 3회는 모두 **Agent 안전 취소**다: `GATE_R7_STATE_INVALID`(같은 순간 Gate ICP_DEGRADED 래치, v2 run1), `SERVER_RECONCILIATION_REQUIRED`(17 s 주행 후, 원인 미분석, v2 run2), `LOCALIZATION_CONTEXT_CHANGED`(v3 run1). 셋 다 정지 방향의 판단이다 |
| UI 완료 표시 | 활성 임무 패널은 종료 후 "활성 임무 없음"으로 바뀐다. 임무 표 행 확인으로 바꿨으나 **완료된 실행에서 그 판을 쓰지 않았다**(미검증) |

발견:
- 서버 `goal-check`가 `ok`로 본 셀을 대시보드 사전 검사가 거부하는 경우가 있었다. 대시보드는 반경 0.32 m(차체 0.22 + 여유 0.10)의 점유·미관측 셀을 본다. 검증 규칙이 두 곳에서 다르다.
- 그래서 하네스는 운영자처럼 UI가 받아들이는 지점을 찾아 클릭한다. 거부된 지점은 기록한다.

## 7. 정상 동작 / 장애 주입 / 반복 결과 요약

| 구분 | 항목 | 결과 |
|---|---|---|
| 정상 | 웹 MANUAL·Deadman 해제 | A1 실행 매회 PASS(Gate 0 0.10~0.15 s) |
| 정상 | 포인터·키보드·게임패드 | 수정 v3 45/45 주기, g02 3/3(유효 실행) |
| 정상 | 웹 MAPPING → 저장 → LOAD → CONFIRMED | 수정 후 탐색 8/8, CONFIRMED 7/8(1회 AMCL 미수렴) |
| 정상 | 지도 클릭 → Nav2 완료 | 1회 완료(원인 분석은 6절) |
| 장애 | Gate SIGSTOP/SIGKILL, Gate+Watchdog SIGSTOP | timeout 0.2 s: 약 0.02 m 이동 후 정지(2절) |
| 장애 | Watchdog SIGKILL | Gate 자체 0, 0.08 m |
| 장애 | 포인터 밖으로 끌기·STOP·touch cancel | 0 출력 확인 |
| 장애 | 미확인 Localization·지도 밖·장애물·미관측 목표 | 모두 거부 |

## 8. 회귀

조건: 이번 변경을 모두 적용한 뒤 실행했다. 스냅샷은 시작과 끝이 동일하다(`SNAPSHOT_SAME`). 위치는 `RAW/refactor35_claude_20261121/{e2e_after_stabilization, unit_after_stabilization}`이다.

| 묶음 | 결과 | 비고 |
|---|---|---|
| 단위 | 91 / 21 | 21 FAIL은 변경 전부터 같은 환경 의존 목록이다. 새 `test_frontier_policy` PASS |
| Phase 1 manual(브라우저, s10 FollowPath 포함) | **19 / 0** | |
| Phase 2 mode(17) | **17 / 0** | 이 묶음의 kinematic 맵핑 시나리오가 새 Frontier 규칙으로 동작 |
| Phase 3 nav(고정 지도) | **18 / 0** | |
| Gate Reset E2E | **13 / 0** | |

Gate E 관찰(`gate_proc_watch.jsonl`, Phase 2·3·Gate Reset 동안 20 ms 주기):
- E(scan_stale FAILSAFE)는 발생하지 않았다.
- 이상 기록은 3건뿐이었다. Phase 3 기동 직후 한 스레드의 run_delay 34 ms가 1건이다. 나머지 2건은 sampler 자신이 63~76 ms 늦은 경우로, 호스트 전체가 지연된 것이지 Gate만의 정지가 아니다.
- 로그의 다른 FAILSAFE는 모두 `scan_stale=False`이며 의도된 장애 시나리오(외부 입력 주입 등)에서 나온 것이다.
- **E는 이번에도 미재현이며 원인 미해결**이다.

## 9. Codex 재현 명령

```bash
cd fleet_control/tests && python3 test_frontier_policy.py && NODE_PATH=<playwright node_modules> node test_manual_control_panel.cjs
```

```bash
local_data/raw_results/gazebo_drive_timeout_claude_20261121/chain_d.sh 0.2:codex_timeout_run1 0:codex_hold_run1
```

```bash
cd fleet_control/tests/gazebo_web && OVERLAY=<overlay> MANUAL_E2E_NODE_PATH=<playwright node_modules> GZWEB_OUT=<new dir> python3 run_gazebo_web.py 28961 224 --browser --only g00_bringup_graph_and_clock,g02_web_browser_keyboard_pointer_gamepad,g07_pointer_input_diagnosis
```

```bash
GZ_SPAWN=10.25,-3.09,0 local_data/raw_results/gazebo_mapping_click_claude_20261121/chain_v3.sh codex_run1
```

```bash
local_data/raw_results/refactor35_claude_20261121/regress_watch.sh e2e_codex_1
```

- `chain_*.sh`에는 이 기계의 explore_lite overlay 경로가 들어 있다. 다른 기계에서는 `OVERLAY`를 바꾼다.
- 도메인 224/228, 포트 28957/28961은 시험 전용이다. 231은 피한다.

## 10. 남은 문제

1. Gate E 원인 미해결(8절). 하위 timeout으로 정지 중 이동은 시뮬에서 약 0.02 m로 묶였다.
2. **실차 STM32 명령 timeout 미측정.** 2절은 시뮬 모델이다. 실차에서는 하위 timeout이 실제 MCU에 있는지, 값이 얼마인지부터 확인해야 한다.
3. r12 재개 직후 오래된 명령 1회 출력(2.3절). Gate 담당자 검토가 필요하다.
4. 복도 ICP 축퇴에 따른 STALL 거짓 양성과 ICP_DEGRADED(5.2절). LiDAR에 안 보이는 낮은 장애물(5.2절). 실차에서는 depth 입력으로 다뤄야 한다.
5. 서버 goal-check와 대시보드 사전 검사의 기준 불일치(6절).
6. Gazebo 지도 클릭 → 완료는 1회만 확인했다. UI 완료 표시는 미검증이다.
7. 큰 지도에서 AMCL 미수렴 1건, 주행 중 `SERVER_RECONCILIATION_REQUIRED`·`LOCALIZATION_CONTEXT_CHANGED` 안전 취소 각 1건(원인 미분석).
8. Nav2 lifecycle 기동 간헐 실패(하네스 재시도로 우회. 원인 미해결).

## 11. 다음 단계 인수인계(SSH 없는 서비스 자동 기동, 방문객 웹)

자동 기동 전에 정할 것:
- 기동 단위와 순서. 현재 격리 기동 `stack_up.sh`의 순서 server → sim → stack(Nav2+Gate) → watchdog → (6 s) agent를 서비스 단위로 옮긴다. Nav2 lifecycle 간헐 실패를 서비스 재시작 정책으로 다룰지 정한다(현재는 시험 하네스만 재시도).
- 하위 정지: 실차 하위(STM32 또는 limo_base)의 명령 timeout을 측정한다. 없거나 길면 2절의 독립 timeout 같은 층을 실차 쪽 어디에 둘지 결정한다(담당자 결정 사항).
- Watchdog 재시작 후 Gate `NOT_EXCLUSIVE` 래치와 약 40 s reset 거부. 서비스 재시작 시 운영자가 보게 될 상태라서 안내 문구 또는 대기 절차가 필요하다.
- 증거 신선도: 웹 패널은 이제 세션 중 300 ms로 갱신한다. 방문객 웹은 조작 권한이 없으므로 1 s 갱신 그대로 둔다.

방문객 웹 전에 정할 것:
- 권한: 현재 역할은 admin/operator/viewer다. 방문객은 viewer보다 좁게(목적지 선택만, 지도·상태 일부만) 하는 새 역할이 필요하다. 서버 권한 정책 변경이므로 승인이 필요하다.
- 목적지 선택: 지도 클릭 대신 등록된 목적지 목록(rmf nav graph 이름 등)을 권장한다. 지도 클릭은 서버 goal-check와 UI 사전 검사 기준 불일치(6절)를 먼저 하나로 정한다.
- 공용 계약: Agent→Server 상태 스키마 계약 시험(REFACTOR_PHASE35 R6)을 방문객 화면 구현 전에 만든다.
