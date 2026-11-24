# Gate 생성 증거(boot_id·fault_epoch) 선별 이식, Gate reset 정책, f08 복구, C4 원인 (G2, 2026-11-27, 격리 ROS·mock만, 미커밋)

**실제 LIMO 접속·배포·모터 구동 없음. 이 문서의 어떤 결과도 실차 READY가 아니며, "모터 명령 차단"은 소프트웨어 관측(Gate 출력 `/cmd_vel_locked`의 0)일 뿐 물리 모터 정지의 검증이 아니다.** 사용자 결정: Gate r13급 증거의 격리 통합·검증 승인 / `INPUT_SILENT` 단순 제거 불승인 / AUTO 최종 Watchdog clamp는 축별 유지 / 물리 모터 차단은 소프트웨어 LOCKED와 별도 검증.

## 1. P0-A: 설치된(G2) Gate와 r13 후보의 실제 비교, 선별 이식
| 항목 | 리포·G2 Gate(745줄, 변경 전) | r13 후보(1880줄, `local_data/.../candidate_r13`) | 이번 이식 |
|---|---|---|---|
| `boot_id` | 없음 | `uuid4().hex` | **이식**(시작마다 새로 생성) |
| `fault_epoch` | 없음 | 부팅마다 +1, 장애 episode마다 +1, 파일에 영속, **영속 후에만 게시**(`fault_epoch_pending`) | **이식**(`epoch_state_file`; 파일 없으면 메모리 값, 파일 손상·읽기 실패면 epoch를 게시하지 않아 Agent가 fail-closed) |
| `fault_latched`, `cmd_blocked` | `cmd_blocked`만 | 둘 다 | `fault_latched` = FAILSAFE_*/STALL_LATCHED 상태 |
| 상태 계약 라벨 | 없음 | `gate-status/1.2-sim`, `trust: UNAUTHENTICATED_SIM_ONLY`(sim 전용, 인증·서명 **미구현**) | **이식하지 않음**. 대신 `evidence_kind: GATE_SELF_REPORT_UNAUTHENTICATED`. sim 전용 계약을 재사용하면 `sim_r7_report_valid`류 sim 경로가 실차형 보고를 받아들일 위험이 생기므로 쓰지 않음 |
| 운영자 reset 게이팅 | 무조건(INITIALIZING만 거부) | `reset_readiness`(단일 writer, `INPUT_SILENT`, 정지 대기, 건강 상태 …) | **부분 이식**(아래 §2). 기본값 off(이전 동작 유지) |
| 감독 하트비트·nav 증거·cliff·memory costmap 등 | 없음 | 있음(sim 전용 프로토콜) | **이식하지 않음**(범위 밖, sim 계약) |
`stall_detector_node.py`는 리포의 597줄 기본본에서 G2가 입력 검증을 얹은 745줄이었고, r7~r13은 `experiments/safety_chain_fix_20261113`·`local_data`의 후보였을 뿐 어느 것도 패키지에 통합된 적이 없다. 실차에 설치되는 Gate에는 attestation이 없다(`ros_adapter`의 "Gate attestation exists only for SIM").
### 확인 항목별 결과
- **boot_id·fault_epoch 생성·갱신**: 단위 시험 `test_gate_generation_evidence_20261127.py` 7개(부팅마다 다른 boot_id, epoch는 부팅 간 증가, 장애 episode당 1 증가, 영속 전 미게시, 손상 파일 덮어쓰기 금지, 기본값 불변).
- **증거의 원본과 최신성**: Gate의 자기 보고이며 **인증이 아니다**(DDS 보안 없음). Agent `GateObservationTracker`가 신선도·boot 변경·epoch 역행을 계속 판정한다.
- **Gate 재시작 후 기존 증거 무효화**: 새 `boot_id`는 Agent의 `boot_changed`를 켜고(기존), 새 DDS endpoint는 Agent의 상태 publisher 연속성 검사(`real_nav_dry_run`에서는 이번에 sim 전용에서 일반화)를 래치시킨다. 실차형 E2E f09(Gate 종료)에서 임무는 `RECONCILIATION_REQUIRED`로 끝나고 이전 증거로는 admission이 되지 않는다.
- **Watchdog writer GID·배타성**: G2 Gate의 입력 검증(단일 endpoint, 허용 이름, writer 자기 보고 GID 일치, sticky)은 그대로이며 reset 게이팅이 같은 규칙을 재사용한다.
- **LOCKED와 NORMAL**: Gate 상태 `NORMAL`과 Agent가 보고하는 모터 경로 `LOCKED`는 별개의 필드다. 건식 admission은 둘 다를 요구한다.
- **Mock 증거가 실차 증거로 오인될 가능성**: ① sim 계약을 쓰지 않음, ② `locked_motor_proof`는 `real_nav_dry_run` Agent에만 유효, ③ 건식 Agent는 `missions_enabled:false`라 실제 임무 경로가 열리지 않음, ④ 상태 topic에 두 번째 publisher가 있거나 endpoint가 바뀌면 건식 Agent의 증거가 무효(§1 표). 남은 한계: 증거는 여전히 비인증이므로 같은 이름으로 먼저 게시하는 악성 프로세스는 막지 못한다.
- **건식 연속 확인(STALL_PENDING)**: 서버 Safety Context는 sim 전용 `safety_state`가 없는 Gate의 평문 필드(`fault_latched False`, `cmd_blocked False`)로 대체하되 `locked_motor_proof`가 참일 때만 허용한다(그 외 admission 조건은 변경 없음).
- 결과: **r13 파일 없이 G2 Gate만으로** 건식 admission·취소·소유권 시험이 통과한다(§5).

## 2. P0-B: `INPUT_SILENT`의 원인과 Gate reset 정책
- **원인(코드·로그로 확인)**: r12/r13의 `reset_readiness`는 단일 허용 writer가 `reset_live` 안에 메시지(0 포함)를 배달했는지 본다. 실차형(Arbiter 없음) 형태에서 Watchdog은 **AUTHORIZED이고 전달할 입력이 없으면 아무것도 발행하지 않는다**(비인가 상태에서만 0을 발행). Arbiter가 있던 형태는 0 스트림을 보냈으므로 문제가 없었다. 그래서 Watchdog 재시작·외부 게시자 시험 뒤 Gate 래치가 `REFUSED: INPUT_SILENT`로 풀리지 않았다(`p58_*`의 `stack.0.log`). 검사 자체는 옳다: 입력이 조용하면 "최신 0 입력이 없다"는 뜻이다.
- **검사를 제거하지 않고 조건을 만족시킬 수 있게 했다**: ① Watchdog 파라미터 `idle_zero_period_s`(기본 0 = 이전 동작): AUTHORIZED이고 `idle_zero_period_s` 이상 아무것도 전달하지 않았을 때만 **명시적 0**을 발행한다(전달된 명령 사이에는 끼어들지 않으며 모션을 시작하지 않는다). ② G2 Gate에 `reset_gating`(기본 false)과 `reset_readiness()` 부분 이식.
- **reset 조건(`reset_gating:=true`)**: 지울 장애(FAILSAFE/STALL 또는 sticky 입력 장애)가 있고 INITIALIZING이 아니며 / `/cmd_vel_raw` writer가 **정확히 하나**이고 GID가 알려졌고 허용 이름 / 그 writer가 `reset_live_sec`(1 s) 안에 메시지를 줬음(**최신 0 입력**) / `reset_quiet_sec`(1 s) 동안 수신된 모션 없음(거부된 모션 포함). 만족하지 못하면 `REFUSED: <blockers>`이고 상태는 변하지 않는다. 운영자의 명시적 서비스 호출이 여전히 필요하다. (r13의 센서 건강·출력 비영 검사는 이식하지 않았다: reset 뒤에는 기존대로 INITIALIZING에서 센서가 건강하고 명령이 없을 때만 NORMAL로 무장한다.)
- **0 입력을 줄 수 없는 상태는 BLOCKED**: Watchdog이 얼어 있거나(`SIGSTOP`) writer가 없으면 reset은 거부된다(E2E d07에서 `INPUT_SILENT` 거부 확인). 임의 허용 경로는 없다.
- **reset 성공이 운동 재개가 아니다**: ① reset은 reset 이전에 받은 명령을 폐기한다(`latest_cmd` 비움, 단위 시험), ② Watchdog trip 횟수가 오르므로 서버가 ACTIVE 모드를 즉시 끝낸다(이전 문서 §7.2), 새 MODE 요청·새 goal 승인이 필요, ③ MANUAL은 중립 입력 뒤 새로 눌러야 한다(`REAL_NAV_DRY_RUN_20261126.md` §1), ④ 이전 goal은 소유권·generation이 달라 재사용되지 않는다(E2E d07·d04).
- **기본값**: 실차 launch의 Gate에는 아직 `reset_gating`·`epoch_state_file`·`idle_zero_period_s`를 켜지 않았다(켜는 것은 별도 결정). 격리 스택(`stack_up_real_shape.sh`)만 켠다.

## 3. P0-C: f08 `robot_not_ready` 추적
이전 보고의 "60 s 동안 `robot_not_ready`"는 r13 Gate(`GATE_SCRIPT`) + 실차형 형태에서 본 것이다. G2 Gate로 바꾸고 매 복구 시도마다 상태를 기록하자(`snap()`: `result.json`의 `snapshots`) 서버가 반환한 정확한 blocker는 시간 순서로 다음이었다.
| 단계 | 서버 blocker | 그때의 증거 |
|---|---|---|
| 재시작 직후~Watchdog 재무장 전 | `no_evidence`(실차 경로) | Agent online·reconciled, Agent boot 새로움, lease epoch 1, Gate NORMAL(boot_id·fault_epoch 있음), 모터 경로 LOCKED, Watchdog `LOCKED`(`AGENT_HEARTBEAT_LOST`, 약 20 s 뒤 재무장), Localization IDLE(재시작으로 지도 비움), `nav_idle ok`, Nav2 active 0, 임무 `RECONCILIATION_REQUIRED` |
| 같은 상태 | `terminal_evidence_unavailable`가 코드상의 최종 거부 | `ownership.recovery_evidence`: "인증된 terminal 증거 생산자가 없으므로 실차 경로 복구는 막혀 있다". 격리 mock만 `_locked_mock_recovery`(Arbiter + LOCKED)로 Nav2 terminal 증거를 받아 복구한다 |
| 규칙 확장 뒤 | `idle_not_proven` | Watchdog이 `MOTION_AUTHORIZED`가 되기 전(약 15~20 s): 증거가 정당하게 부족 |
| Watchdog 재무장 뒤 | 성공 | Nav2 action result가 journal에 있는 terminal(`FAILED`)로 임무 확정 |
- **무엇을 바꿨나(완화가 아니라 같은 규칙의 적용 범위)**: `_locked_mock_recovery`가 `real_nav_dry_run` + Watchdog selector Agent에도 적용된다(`locked_motor_proof`). 규칙 자체(명령마다 Nav2 terminal 증거, 소유 goal 없음, Nav2 idle, Gate NORMAL·cmd_blocked False, Watchdog 승인, 출력 0, 모터 경로 LOCKED, 매핑 worker 정지)는 그대로이며 단위 시험이 각 조건이 없으면 `idle_not_proven`임을 확인한다. 실차형이지만 `real_nav_dry_run`이 아닌 Agent는 여전히 `terminal_evidence_unavailable`이다.
- **이전 `robot_not_ready`의 원인**: r13 후보를 쓴 실행에서 서버의 `reconciled`가 False였다. 이번에는 같은 증상을 G2 Gate에서 다시 만들지 못했다(재시작 뒤 `reconciled` True). r13의 sim 계약(`status_seq`·`host_boot_id` 검증)과 Agent 재시작이 겹쳤을 가능성이 있으나 r13으로는 재현·확인하지 않았다 — 해당 부분은 **미확정**.
- 연속 흐름·독립 흐름 결과는 §5.

## 4. P0-D: C4 지연 — 원인 구간 확정, DDS 내부 메커니즘 미확정(BLOCKED 유지)
Watchdog에 진단 전용 필드 `sl_diag`(speed-limit 메시지가 보인 시점의 graph publisher 수, 메시지 최초 도착 시각, tick 간격)를 넣고, 하네스에 독립 관측자(같은 프로세스의 graph 질의 50 Hz)를 추가했다. CLOCK_MONOTONIC이라 프로세스 간 시각을 직접 비교한다.
| 시각(rogue 프로세스 시작 +s) | 정상 8회 | 느린 2회(run 1, 8) |
|---|---|---|
| 독립 관측자의 graph가 두 번째 writer를 봄 | 0.25 | 0.25 / **1.74** |
| Watchdog이 처음으로 2 writer를 보고, 동시에 첫 rogue 메시지가 도착 | 0.21~0.23 | **2.73 / 2.72** |
| Watchdog `LOCKED`(SPEED_LIMIT_NOT_SYNCED) | +0.3 | +2.79 / +2.8 |
| Watchdog tick 간격 최대 | 0.05~0.07 s(>150 ms는 0회) | 0.05 s(0회) |
| 시스템 load1 | 0.4~1.1 | 0.6~0.85 |
- **분리된 사실**: (1) CPU·스케줄링·Watchdog timer 지연이 아니다(tick 최대 0.07 s, 부하 낮음). (2) Watchdog의 검사 로직이 늦은 것도 아니다: **Watchdog의 graph 보기와 rogue의 데이터가 같은 순간(+2.7 s)에 함께 나타났고**, 그로부터 0.06 s 안에 정지했다. (3) 느린 실행에서는 Watchdog **프로세스**가 새 participant를 늦게 발견했다(다른 프로세스인 관측자는 0.25 s, 한 번은 1.74 s). 즉 새 DDS participant가 각 기존 participant에 발견되는 시간이 0.2 s ~ 2.7 s로 흔들린다.
- **안전 성질과의 구분**: 발견 전에는 rogue의 **데이터가 Watchdog에 도착하지 않았다**(첫 메시지 도착 = 발견 시각). 따라서 감지가 늦어진 구간에서 상향 제한(0.15)이 수용될 수 없었고 실제로도 `applied=0.06` 그대로였다. 그 구간의 출력은 정상 동작(이미 승인된 0.06 한도)이다. 시험의 "1.5 s 안에 정지" 기대는 *발견되지 않은* writer에 대한 기대였다.
- **확정하지 못한 것**: 발견 지연의 DDS 내부 원인(참가자 발견 announcement 주기·multicast 누락 등)은 측정하지 않았다. 따라서 "새 writer 감지 시간의 상한"을 보장한다고 말할 수 없고 그 항목은 BLOCKED다(그래프 기반 비인증 감지의 설계 한계; 해결에는 DDS 보안 또는 허용 목록을 쓰는 discovery 설정이 필요). 시험 기대값은 바꾸지 않았다.

## 5. 건식 Fleet Mission E2E (실차형 스택: Arbiter 없음, Watchdog mode gate+clamp+idle zero, Agent selector+`real_nav_dry_run`, **G2 Gate**(`epoch_state_file`·`reset_gating` 켬), O6 Nav2 overlay, 운동학 시뮬레이터)
드라이버 `tests/nav_e2e/run_dry_run_missions_e2e_20261126.py`(d01~d07 포함), 스택 `tests/mode_e2e/stack_up_real_shape.sh`. 로그 `p64`~`p72`. 모든 "모터 차단" 판정은 **Gate 출력 `/cmd_vel_locked`의 0**이며 물리 모터 정지의 검증이 아니다.
### 5.1 연속 흐름: NEW → ACTIVE → CANCEL → IDLE → (2번째 goal ACTIVE) → Agent 재시작 → 관리자 복구 → 새 goal 승인·완료 (`d06`)
단계별 결과: NEW(`QUEUED`, `dry_run:true`, `real_motion_authorized:false`) → ACTIVE(Nav2 goal 소유) → CANCEL(`CANCELED`) → IDLE(모드 IDLE, `nav_idle ok`) → 2번째 goal NEW·ACTIVE → **재시작**(종료 0.5 s 안에 Gate 출력 0, 임무 `RECONCILIATION_REQUIRED`, 모드 IDLE, 재시작 뒤 비영 샘플 0, 옛 goal 재사용 `mode_not_navigation` 거부) → **관리자 복구**(몇 번의 `idle_not_proven` 거부 뒤, Watchdog 재무장 후 Nav2 journal terminal `FAILED`로 확정) → 새 모드 요청·LOAD·localization → NEW3 `SUCCEEDED`. **독립 새 스택 2회 PASS**(`p65_d06`, `p68_indep_d06`), 같은 스택 누적 실행 안에서 2/3 PASS(첫 실행은 하네스가 Watchdog이 확인하지 않는 모드를 기다리지 않아 실패 → 수정 후 2회 중 2회 PASS).
### 5.2 각 고장 주입 — 독립 새 스택 (한 스택에 하나)
| 시나리오 | 결과 |
|---|---|
| 건식 admission·소유권(generation)·취소→IDLE·Lease 상실·f01~f03 (묶음 A, 14항목) | 14/14 PASS (`p64_g2gate_groupA_run2`; 첫 실행은 STALL_PENDING 연속 확인 증거 문제로 실패 → §1의 평문 필드 대체 도입) |
| f05 AMCL 종료 + f06 재위치 | PASS |
| f07 Watchdog·Gate 입력의 외부 게시자 | PASS (비영 샘플 0) |
| d05 건식 goal 중 Watchdog 종료 | PASS (종료 뒤 출력 0, 임무 `CANCELED`) |
| f09 Gate(스택) 종료 | PASS (출력 0, 임무 `RECONCILIATION_REQUIRED`) |
| d06 연속 흐름 | PASS ×2 |
| **d07 Gate reset 정책과 운동 비재개** | **PASS**(`p70_indep_d07_rerun`): rogue가 있는 동안 reset `REFUSED: INPUT_WRITERS:2,MOTION_COMMANDED,REJECTED_MOTION_RECENT`; rogue가 사라진 직후에도 거부(남은 endpoint); Watchdog 동결(0 입력 없음) `REFUSED: INPUT_SILENT`; 재개 뒤 reset 수락; **수락 뒤 6 s 동안 Gate 비영 0, 옛 임무 `CANCELED`, 새 임무는 `mode_unverified`로 거부** (첫 실행은 판정식이 서버의 3 s 유예 안의 "미확인 모드"를 허용하지 않아 실패 → 판정식 수정) |
| f08 Agent 종료 → 증거 복구 → 새 goal | 4회 중 2회 PASS, 2회 실패 — **둘 다 시나리오 시작 시 AMCL이 수렴하지 못해(`CONVERGING`, match 0.08) 목표 검사가 막힌 것**이지 복구 로직의 실패가 아니다(Agent 종료 이후 단계에 도달하지 못함). 복구 규칙 자체는 f08 PASS 4회(독립 2 + 누적 2)·d06 PASS 4회(독립 2 + 누적 2)에서 검증 |
| f04 Nav2 planner 동결 | 누적 실행 3회에서 PASS(독립 실행은 이번 G2 Gate로 따로 하지 않음) |
### 5.3 같은 스택에서의 누적 고장·복구 (`p67`, `p69`; 한 스택에서 매핑→시나리오 24개 전부)
| 실행 | 결과 | 실패 항목 |
|---|---|---|
| run 1(수정 전 하네스) | 21/24 | f05(앞 시나리오 뒤 NAVIGATION 미재진입), d06·d07(Watchdog이 확인하지 않는 모드를 기다리지 않음) — 모두 하네스 선행 조건 |
| run 2 | **24/24 PASS** | |
| run 3 | 22/24 | f05: Gate `FAILSAFE_SENSOR_TIMEOUT`(합성 센서 고정 노드의 기존 간헐 문제), f08: LOAD 직전에 서버 모드가 IDLE로 끝남(원인 미확정) |
**누적 시험은 안정적이지 않다**(24/24 1회, 21·22/24 2회). 실패는 합성 센서 간헐 문제·하네스 선행 조건에 걸쳐 있고, 안전 성질(운동 차단·재사용 거부·자동 재개 없음)을 깨는 실패는 없었다.
### 5.4 기존 모드 E2E 67/67 유지
Gate 이식(G2 Gate 사용)과 Watchdog 변경 위에서 새 스택 2회 연속 **67/67 PASS**(`p67_modes_final_run1/2`).

## 6. 회귀(현재 코드 기준)
| 시험 | 결과 |
|---|---|
| Gate 생성 증거·reset 게이팅 단위(`test_gate_generation_evidence_20261127`) + 기존 Gate 입력 검증(`test_gate_input_source`) | 7 + 24 = 31 OK(사설 ROS 네임스페이스) |
| Watchdog 모드 게이트(idle zero 포함) / R2 `test_motion_watchdog` / 서버 선택기 / 건식 분리·복구 단위 | 26 / 22 / 25 / 9 OK |
| 전체 unittest discover 1196개 | 실패 집합 = 기준 15bb5c59의 것과 동일(+ 스크립트형 `test_speed_limits_server`의 import 시 `exit 0` 1건) |
| 필드 체인 프로브(Arbiter 형태) | 29/29 |
| 속도 clamp 프로브 | 18/18 |
| 웹 속도 제한 E2E | 43 PASS 2회(앞선 2회 시도는 `ros2 lifecycle` CLI 실패 E3, 합성 센서 전부 stale인 Gate `FAILSAFE_SENSOR_TIMEOUT`로 E14 실패 — 기존 간헐 문제) |
| launch 하네스(C4 포함, 전체 42항목) | 10회 중 8회 42/42, 2회 C4(§4) |

## 7. 실차 배포 전 차단 조건 (이 문서 이후에도 남음)
1. **Gate attestation은 비인증**이다. 이식한 `boot_id`·`fault_epoch`는 Gate 프로세스의 자기 보고일 뿐 인증·서명이 아니다. 같은 이름으로 먼저 게시하는 악성 프로세스는 막을 수 없다(DDS 보안 없음).
2. **실차 launch에는 `epoch_state_file`·`reset_gating`·`idle_zero_period_s`가 꺼져 있다.** 켜는 것은 별도 결정이며 켜기 전에는 실차 Gate에 생성 증거가 없다. 켠 뒤의 실차 동작은 검증된 적이 없다.
3. **물리 모터 차단의 별도 검증이 없다.** `locked_motor_proof`는 소프트웨어 그래프 관측이며, E-Stop·STM32 명령 타임아웃·제동거리·구동 전원 분리 확인은 사람이 직접 해야 하고 기록된 값이 없다.
4. 실차 `mode:=real`로 `real_nav_dry_run` Agent가 시작되는 경로, 실차의 saved-map localization worker(map_server/AMCL)는 로봇에서 실행된 적이 없다. 모터가 잠긴 실차에서 Gate의 정지(stall) 감지가 건식 임무에 어떻게 반응하는지 미확인.
5. **C4**: 새 DDS writer 발견 시간의 상한을 보장할 수 없다(§4).
6. f08 독립 시험은 AMCL 수렴 간헐 실패로 반복 안정성이 낮다(복구 규칙 자체는 별도 통과), 누적 시험도 불안정하다(§5.3).
7. 실제 조이스틱의 해제 메시지 패턴 미확인(이전 문서).
8. Gate r13의 나머지(감독 하트비트, nav 증거, cliff, memory costmap)는 이식하지 않았다. r13 `reset_readiness`의 센서 건강·출력 비영 검사는 이식하지 않았다.

## 8. 통합 준비
- **G2 변경 목록(미커밋)**: Gate(`stall_detector_node.py`: boot_id·fault_epoch·fault_latched·reset 게이팅), Watchdog(`idle_zero_period_s`, `sl_diag` 진단, 재무장 정책), Agent(`ros_adapter` 건식 상태 publisher 연속성, `_locked_motion_proof`), 서버(`safety.py` 평문 필드 연속 확인, `ownership.py` 건식 복구 규칙, `service`·`http_api`·`navigation`·`operating_mode`), 격리 스택(`stack.launch.py` 환경 변수, `stack_up_real_shape.sh`), 시험·문서.
- **파일 충돌(읽기 전용 점검)**: Web UI worktree(10 파일)·Agent Harness worktree·Antigravity Navigation worktree(7 파일)와의 교집합은 **`fleet_control/tests/mode_e2e/stack_up.sh` 하나**뿐이다(G2는 `REAL_SHAPE=1`이면 `stack_up_real_shape.sh`로 위임하는 한 줄만 추가; 충돌하면 그 한 줄만 재적용).
- **UI 신규 필드 대응 요구**: 로봇 보기의 `mode_motion{verified,blocker,hard}`, 임무의 `dry_run`·`real_motion_authorized`, 새 endpoint `POST /api/v1/dry-run-missions`, 오류 코드 `mode_unverified`·`dry_run_agent`·`not_dry_run_agent`, Gate 상태의 `boot_id`·`fault_epoch`·`fault_latched`·`evidence_kind`·`reset_readiness`(거부 사유 목록을 운영자에게 보여야 함), `last_reset`.
- **통합 후 회귀**: 모드 E2E 67, 건식 E2E(독립·누적), launch 하네스, Gate 단위(사설 ROS), R1·R2, 속도 clamp·웹 속도 제한 E2E, UI 브라우저 시험, Navigation O6 Nav2 파라미터가 Mapping·Depth·LiDAR 프로파일에 주는 영향.
- **통합 가능 판단**: 변경들은 서로 독립적으로 되돌릴 수 있고 기본값(`epoch_state_file ''`, `reset_gating false`, `idle_zero_period_s 0`, `real_nav_dry_run false`)이 이전 동작이다. 코드 측면은 승인 후 병합 가능하나, **실차 배포 가능이라는 뜻은 아니다**(§7).
