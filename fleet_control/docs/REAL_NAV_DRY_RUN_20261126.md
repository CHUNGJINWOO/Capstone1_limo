# 실차형 Agent의 건식 Fleet Mission(`real_nav_dry_run`), 조이스틱 재무장 정책, C4, AUTO Clamp 비교 (G2, 2026-11-26, 격리 ROS·mock만, 미커밋)

**실제 LIMO 접속·배포·모터 구동 없음. 어떤 결과도 실차 READY가 아니다.** `real_nav_dry_run`은 실제 모터 명령 허용을 뜻하지 않는다.

## 1. 조이스틱 재무장(작업 A) — 침묵만으로는 해제로 인정하지 않는다
### 정책(Watchdog `motion_watchdog.py`)
- **해제 = 명시적 중립(0) 명령 하나.** 0.5 s 이상의 침묵은 해제가 아니다(이전 정책에서 침묵을 해제로 보던 모든 경로 제거: 모드 부여 창, 비인가 episode, 부팅).
- 조이스틱이 눌린 상태(마지막 메시지가 비영)에서 **0.5 s 초과 침묵 후 비영 명령이 돌아오면** 연속으로 보지 않고 래치(`silence_latches`): 0 → 새 비영 순서 필요.
- **manual 채널 endpoint(DDS GID 집합)가 바뀌면**(같은 노드 이름으로 재생성해도) 눌린 상태일 때 래치(`endpoint_latches`).
- Watchdog이 AUTHORIZED를 벗어났다 돌아올 때(Gate 상태 상실·재시작 등)와 새 MANUAL 부여 때도 눌려 있던 입력은 0을 한 번 본 뒤에만 움직인다. 조이스틱이 중립(0)이었고 이후 침묵하다가 새로 누르는 것은 정상 입력이다.
- 서버 쪽(§ 이전 문서 7)의 Watchdog boot·trip 고정과 Agent 재시작·Lease 상실은 이미 모드를 끝내므로 같은 입력으로 자동 재개되지 않는다.
### 고장 주입 결과
| 주입 | 결과 |
|---|---|
| 누른 채 발행자 0.6 s+ 정지 후 비영 재개 | 출력 0 유지(단위 + E2E M15 A), 0 → 새 누름 뒤에만 이동 |
| Gate 재시작, Watchdog LOCKED→AUTHORIZED | 출력 0, 해제 후 새 누름에서 이동(이전 §7 trace 4/4 + E2E) |
| Agent 재시작, Lease 상실·재획득 | 서버 모드 종료, 출력 0, 새 MANUAL 요청 필요(E2E M10·M11) |
| DDS endpoint 제거 후 같은 이름 게시자 재생성 | 출력 0, 0 → 새 누름 뒤에만 이동(단위 + E2E M15 B) |
**판정: 사용자가 해제하지 않았는데 자동으로 움직임이 재개되는 경로는 위 주입에서 발견되지 않았다.** 미검증: 실제 조이스틱 노드가 해제 시 0을 보내는지(보내지 않고 침묵만 하면 이 정책에서는 0 명령이 올 때까지 재개되지 않아 fail-closed지만 운용상 불편할 수 있다 — `teleop_twist_joy`는 해제 시 0을 한 번 보내는 것으로 알려져 있으나 이 로봇에서 확인하지 않았다).

## 2. 건식 Fleet Mission(작업 B)
### 의미와 분리
- **`real_nav_dry_run`(Agent 파라미터, 기본 false)**: 실차형(Arbiter 없음, Watchdog 선택기) Agent가 Fleet 임무 파이프라인(저장 지도 LOAD → 초기 위치 → 임무 승인 → Nav2 goal → 취소/종료)을 **모터 경로가 LOCKED일 때만** 실행한다. `allow_missions`가 아니며 서로 배타적이다(`allow_missions`·Arbiter·웹 bench·수동 매핑과 함께 쓰면 Agent가 시작하지 않는다). Agent 보고: `real_nav_dry_run: true`, **`missions_enabled: false`**.
- **`locked_motor_proof`**: Agent 쪽 `_locked_motion_proof()`와 서버 `safety.py`의 연속 확인 증명. **소프트웨어 운동 경로 차단 증거**일 뿐이다: 이 Agent가 그래프에서 `/cmd_vel`과 드라이버 토픽의 endpoint 없음(모터 경로 LOCKED)을 관측했고 Watchdog 선택기 형태라는 뜻이다. **모터가 물리적으로 비활성임을 증명하지 않으며**, `real_nav_dry_run`이 아닌 Agent에는 적용되지 않는다. 사용 장소: ① Agent의 `GATE_ATTESTATION_UNAVAILABLE` 두 곳(ros_adapter), ② 서버 Safety Context의 Gate STALL_PENDING 연속 확인.
- **API·상태 분리**: 건식 임무는 `POST /api/v1/dry-run-missions`로만 만든다(`POST /missions`는 body의 어떤 값으로도 건식이 되지 않으며 건식 Agent에는 `409 dry_run_agent`; 건식 요청이 건식 Agent가 아니면 `409 not_dry_run_agent`). 임무 보기에 `dry_run: true`, `real_motion_authorized: false`가 붙고, 기록은 meta `mission_dry_run:<id>`로 분리된다. 명령에는 `dry_run: true`가 실리며 Agent는 불일치(건식 명령이 실차 임무 Agent로, 또는 반대)를 `DRY_RUN_MISMATCH`로 거부한다. **건식 결과는 어떤 실제 주행 admission 입력에도 쓰이지 않는다**(Safety Context·`require_mode`는 이를 읽지 않으며, 건식 Agent는 `missions_enabled`가 false라 실제 임무 경로가 열리지 않는다). NAVIGATION 모드 진입은 `missions_enabled` 또는 `real_nav_dry_run`일 때 허용된다.
- **변경하지 않은 것**: Safety Context의 기존 admission 조건 전부(Gate NORMAL·boot·epoch·지도·위치·실행 보고·Watchdog epoch·Lease), 실차에서 `allow_missions` 차단, Gate 계약.
- 서버 `require_mode(motion=False)`: 지도 LOAD/INITIAL_POSE/UNLOAD는 움직임이 없으므로 모드 기록만 확인한다(Watchdog 확인은 운동·임무 admission에만 적용).
### 격리 E2E(실차형 스택: Arbiter 없음, Watchdog mode gate + clamp, Agent selector + `real_nav_dry_run`, Gate r13 후보, O6 Nav2 overlay, 운동학 시뮬레이터)
스택 `tests/mode_e2e/stack_up_real_shape.sh`(`stack_up.sh`에서 `REAL_SHAPE=1`로 위임), 드라이버 `tests/nav_e2e/run_dry_run_missions_e2e_20261126.py`(Phase 3 하네스 파생 + d01~d05). 로그 `p57`~`p59`.
| 시나리오 | 결과 | 비고 |
|---|---|---|
| n01 지도 LOAD → 초기 위치 → localization CONFIRMED | PASS | 실차형 스택에서 웹 매핑→저장→검증→LOAD까지 동작 |
| n03 건식 임무 admission → Nav2 → `SUCCEEDED` | PASS | 운동학 시뮬레이터, 경로점 74개 |
| n04 / d02 취소 → `CANCELED`, Gate 출력 0 | PASS | 취소 결과 확인 뒤 `nav_idle` ok |
| d01 건식/실제 분리(실제 endpoint 거부, 보기 플래그) | PASS | `dry_run_agent`, `real_motion_authorized:false`, `missions_enabled:false` |
| d02 goal 소유권·generation·운동 경로 | PASS | active goal의 mission/command, 명령 `owner_boot_id == Agent boot`, Watchdog `goal_owner=fleet`·`goal_ok`·mode revision 일치, Gate 비영 샘플 ≤ 0.15 m/s, 취소 뒤 0 |
| d03 취소 결과 확인 후 IDLE 복귀 | PASS | IDLE에서 임무 `mode_not_navigation`, 출력 0 |
| d04 Lease 해제 → 임무 종료·모드 IDLE, 재획득 | PASS | 재획득 10 s+ 뒤에도 비영 0, 같은 idempotency key는 종료된 임무를 돌려줄 뿐 새 명령 없음(navigate 명령 1개), 모드 자동 복구 없음 |
| f01·f02·f03 지도 버전/무결성/잘못된 초기 위치 | PASS | |
| f04 Nav2 planner 동결 → FAILED | PASS | |
| f05 AMCL 종료 중 goal → CANCELED, 3 s 내 0 | PASS | f06(재위치)도 PASS(신선한 스택) |
| f07 Watchdog·Gate 입력의 외부 게시자 | PASS | 비영 샘플 0 |
| d05 건식 goal 중 Watchdog 종료 | PASS | 종료 뒤 출력 0, 임무 CANCELED |
| f09 Gate(스택) 종료 | PASS | 종료 뒤 출력 0, 임무 `RECONCILIATION_REQUIRED` |
| **f08 Agent 종료·재시작 뒤 증거 기반 복구와 새 goal** | **미검증(BLOCKED)** | 종료 직후 출력 0·임무 `RECONCILIATION_REQUIRED`·모드 IDLE(agent restarted)·이전 goal 재사용 불가까지는 확인. 그러나 관리자 증거 복구가 `robot_not_ready`(서버의 `reconciled` False)로 60 s 동안 모두 거부되어 새 goal 단계에 도달하지 못했다. 같은 시나리오는 Arbiter 형태 스택에서 통과한다고 Antigravity가 보고했으므로 실차형 형태 고유의 차이일 수 있으나 원인을 찾지 못했다 |
참고: Agent 재시작 뒤 자동 재출발은 관찰되지 않았다(모드 IDLE, 비영 0). 한 번에 전체 순서로 돌리면 앞 시나리오의 상태(Gate 래치 `INPUT_SILENT` 등)가 뒤 시나리오를 오염시켜, 위 결과는 시나리오 묶음별 새 스택에서 얻었다(`p58`~`p59`; 단일 전체 완주는 없음).

### 실차에서 건식 임무로 가기 전의 BLOCKED 조건 (소프트웨어)
1. **설치된 Gate에는 admission에 필요한 attestation(boot_id·fault_epoch 등)이 없다.** 위 E2E는 Gate **r13 후보**(`local_data/raw_results/gate_r13_integ_claude_20261122/candidate_r13`)로 얻었다. r13은 리포에 통합되어 있지 않고 실차에 배포된 적이 없다. 설치된 Gate로는 `GATE_FAULT_LATCHED`·`GATE_BOOT_ID_MISSING`·`GATE_FAULT_EPOCH_INVALID`로 admission이 거부된다(`p57`).
2. **r13 Gate는 `/cmd_vel_raw` 입력이 조용하면 운영자 reset을 `INPUT_SILENT`로 거부한다.** 실차형 형태에서는 Watchdog이 AUTHORIZED이고 입력이 없을 때 아무것도 발행하지 않아(Arbiter가 있던 형태는 0 스트림을 보냈다), Watchdog 재시작·외부 게시자 시험 뒤 Gate 래치가 풀리지 않는다. 필요한 설계 결정: Watchdog의 유휴 0 스트림 또는 reset 조건 변경(Gate 계약 변경이므로 사용자 결정).
3. 실차 모드의 `real_nav_dry_run` 시작 경로(`mode:=real`)와 실차의 saved-map localization worker(map_server/AMCL)는 로봇에서 실행된 적이 없다. 격리 시험은 `mode:=mock simulated:=true`로만 했다.
4. 건식 임무 중 Gate의 정지(stall) 감지: 모터가 잠겨 있으면 로봇이 움직이지 않으므로 Gate가 stall로 판단할 수 있다. 건식 시험의 기대 결과는 "승인·소유권·취소·차단"이지 "목표 도달"이 아니다. 하드웨어에서의 동작은 미확인.
5. 위 `locked_motor_proof`는 그래프 관측이며 물리적 모터 잠금이 아니다. 하드웨어 건식 시험을 하려면 바퀴를 띄우거나 구동 전원을 분리하는 등 물리적 조치를 별도로 사람이 확인해야 한다(E-Stop·STM32 타임아웃 값 미확인 그대로).

## 3. C4 간헐 실패(작업 C) — 원인 미확정, BLOCKED 유지
`real_launch_chain_faults_20261123.py`의 C4에 20 Hz 타임라인을 넣고 반복했다(`p61`~`p62`, 14회: 정상 12회, 실패 2회; 이전 4회 중 1회 포함 전체 약 3/18).
| | 정상 실행 | 실패 실행(run 6, 8) |
|---|---|---|
| 두 번째 `/fleet/speed_limit` 게시자 시작 | +0.0 s | +0.0 s |
| Watchdog이 감지(state `LOCKED`, `SPEED_LIMIT_NOT_SYNCED`, clamp `publisher_ok False`) | +0.35 s | **+2.73 ~ +2.78 s** |
| 마지막 전달 샘플 | +0.20 ~ +0.23 s | **+2.62 ~ +2.63 s** |
| clamp `synced` False | +1.73 ~ +1.77 s | (감지 후) |
- **확인된 것**: 실패는 감지 지연이다. 지연 동안 전달된 출력은 이미 승인된 낮은 한도(0.06)로 clamp되었고 **두 번째 writer가 보낸 상향(0.15)은 수용되지 않았다**(`applied=0.06`, "never takes the raise" 충족). `WATCHDOG_SELF_STALL`은 이 14회 타임라인에서 관찰되지 않았다(첫 실패 때의 `last_fault=WATCHDOG_SELF_STALL`은 그 실행의 다른 episode로 보이나 증거 부족).
- **추정(미확정)**: Watchdog의 판단은 `get_publishers_info_by_topic`(rmw 그래프)에 의존하고, 새 프로세스의 endpoint가 그래프에 나타나는 시점이 DDS discovery에 따라 0.3 s ~ 2.8 s로 흔들리는 것으로 보인다. 이것을 별도 관측자로 직접 측정하지는 않았다.
- **의미**: 그래프 기반(비인증) 감지는 새 writer가 나타난 뒤 최대 약 3 s 늦을 수 있고, 그 사이는 **이미 승인된 낮은 한도 안에서** 움직인다. 시험 기대값(1.5 s 내 정지)은 바꾸지 않았다. 원인 미확정이므로 BLOCKED로 기록한다.

## 4. AUTO 비례 Clamp 비교(작업 D) — 비교만, 실차 안전 Clamp는 변경하지 않음
**제품 clamp(축별)는 그대로 둔다.** 비례 축소는 `compare_auto_clamp_20261126.py` 안에서만 존재한다. 이번에 새로 돌린 것은 오프라인 수치 비교(`p60_auto_clamp_comparison.json`, 격리 ROS·Gazebo 폐루프는 새로 돌리지 않았다)이고, 폐루프(목표 오차·경로 편차·시간)는 Antigravity의 운동학 시뮬레이터 결과(`NAV2_STABILIZATION_AND_PATROL_20261124.md` §4.2, 읽기 전용 재사용)를 쓴다.
| 항목 | 축별 clamp(현재) | 비례 축소(실험) |
|---|---|---|
| 속도 상한 위반(41×41 명령 격자, AUTO 0.15/0.50, 운영자 0.10/0.30, 0.06/0.20) | 0 | 0 |
| 부호 반전 | 0 | 0 |
| STOP(0,0)·비유한 입력(NaN/inf) | (0,0) / (0,0) | (0,0) / (0,0) |
| 제자리 회전 (0,0.8) → | (0, 0.5)(AUTO) | (0, 0.5) — **동일** |
| 곡률 `ω/v` 변화 | 선속만 깎일 때 최대 **2배**(AUTO) ~ **5배**(0.06/0.50)로 증가, 1010~1358개 명령에서 1% 초과 변화 | 0(전 명령에서 곡률 보존) |
| 좁은 회전 (0.10, 0.80) → | (0.10, 0.50): 곡률 8→5(덜 꺾임) | (0.0625, 0.50): 곡률 8 유지, 선속이 더 낮아짐 |
| 곡선 (0.30, 0.30) → | (0.15, 0.30): 곡률 1→2 | (0.15, 0.15): 곡률 1 유지 |
| 폐루프(운동학, Antigravity): 선속만 제한한 `0.06/0.50` | 최대 곡률 5.83~5.88 1/m(약 1.75배), 목표 오차 0.11~0.17 m, 평균 CTE 0.024~0.031 m | (그들의 "Proportional 0.06/0.20": 곡률 3.33 1/m, 소요 57~64 s) |
- **추천(별도 결정으로 남김)**: ① 최종 안전 clamp(Watchdog)는 지금처럼 축별 상한으로 유지한다(출력 상한 증명이 단순하다). ② 속도 제한을 낮추는 UI/운영자 한도(`max_linear_mps`)는 **선속·각속을 함께 낮추는 쌍(예: 0.06/0.20)** 으로만 적용하도록 프리셋을 정의해 곡률 증가를 피한다(clamp 코드 변경 없음). ③ 비례 clamp를 최종 clamp에 넣는 것은 Gazebo 폐루프(좁은 통로·장애물 접근)에서 경로 편차·충돌 여유를 측정한 뒤 별도 승인으로 결정한다. 이번에 못 한 것: 제자리·좁은 공간 회전과 장애물 접근의 폐루프 측정, STOP/FAULT 시 clamp 영향(clamp는 정지 경로에 없으므로 영향 없다는 것은 코드 구조상 판단일 뿐 측정하지 않음).

## 5. 회귀(현재 코드 기준)
| 시험 | 결과 |
|---|---|
| 모드 E2E 67항목(실차형, 조이스틱·Gate·Watchdog·Agent·서버·Lease 고장 포함) | 새 스택 3회 연속 67/67 PASS(`p56`, `p63` ×2) |
| 단위: Watchdog 모드 게이트 24, 서버 선택기·ACTIVE 25, 건식 분리 7, R2 `test_motion_watchdog` 22, stall backlog 23, clamp 15 | PASS |
| 전체 unittest discover 1193개 | 실패 집합 = 기준 15bb5c59의 것과 동일(스크립트형 `test_speed_limits_server`의 import 시 `exit 0` 1건만 차이) |
| clamp 프로브 | 18/18 |
| 웹 속도 제한 E2E | 43 PASS, 종료 0 |
| 필드 체인 프로브(Arbiter 형태) | 3회: 29/29 1회, 28/29 2회(G7 "MANUAL 재바인딩 뒤 구동" 1회 — 재실행 2회에서 재현되지 않음, 이 프로파일은 Watchdog mode gate가 꺼져 있어 이번 Watchdog·서버 변경이 적용되지 않는다; 첫 점검 "DDS 노드 이름 미발견" 1회 — 기존 간헐 문제) |
| NAVIGATION 모드 진입(실차형 mock) | 이전 12/12(변경 전); 이번 변경 뒤 재실행 없음 — 건식 E2E의 n01이 같은 경로를 통과 |
| launch 하네스(전체 42항목 완주 기준) | 12회 중 9회 42/42, 3회 C4 실패(위 §3; p55 4회 중 1회 + p62 8회 중 2회) |

## 6. G2·Web·Navigation 통합 준비
- **G2 미커밋 변경**: Watchdog(재무장·trip 보고), Agent(`real_nav_dry_run`, `_locked_motion_proof`), 서버(`dry-run-missions`, `mode_motion`, ACTIVE 검증, `require_mode(motion)`), 스택 `stack_up_real_shape.sh`(+`stack_up.sh` 한 줄), 시험·문서. 실차 후보 tar 3종은 소스 변경으로 다시 오래된 것이다(커밋 후 재생성).
- **Web(Antigravity UI)**: 새 필드 `mode_motion`(`verified/blocker/hard`)과 임무 `dry_run`/`real_motion_authorized`, 새 endpoint `/dry-run-missions`, 새 오류 코드 `mode_unverified`·`dry_run_agent`·`not_dry_run_agent`를 UI가 표시·처리해야 한다. UI 파일은 건드리지 않았다.
- **Navigation(Antigravity worktree, 읽기 전용 확인, 변경 없음)**: 수정 5파일(Nav2 slow 프로파일: rotate-to-heading·PoseProgressChecker; 스캔 매처 파라미터화; 시험 도구 3개)과 신규 2파일. G2는 이 5개를 바꾸지 않았다(교집합 0). 단 `tests/mode_e2e/stack_up.sh`는 양쪽이 수정했다 — G2는 상단에 한 줄(`REAL_SHAPE=1`이면 `stack_up_real_shape.sh`로 위임)만 추가했으므로 병합 시 충돌하면 그 한 줄만 재적용하면 된다. 건식 E2E는 O6 Nav2 변경을 overlay로 적용해 시험했다(n03 `SUCCEEDED`); 그들의 변경은 Mapping·Depth·LiDAR 프로파일 회귀가 필요하다. 통합 순서(승인 필요): G2 커밋 → UI → Navigation → 병합 → 모드 E2E 67·건식 E2E·launch 하네스 재실행 → 후보 tar 재생성.

## 7. 남은 P0 차단 조건(실차 건식·실제 임무 전)
Gate attestation(r13급) 통합·배포와 그 reset 정책(§2 BLOCKED 1·2), f08 Agent 재시작 뒤 복구(실차형), C4 감지 지연 원인, 실제 조이스틱의 해제 메시지 패턴, E-Stop·STM32 타임아웃·제동거리 실측, 하드웨어 모터 잠금의 물리적 확인, 실차 `mode:=real` 경로 실행 검증.

> **후속(2026-11-27)**: 위 §2의 BLOCKED 1·2와 f08 미검증은 `GATE_EVIDENCE_AND_DRY_RUN_RECOVERY_20261127.md`에서 격리 환경 기준으로 대체·해결되었다(G2 Gate에 생성 증거 이식, reset 게이팅과 idle zero, 건식 복구 규칙, f08·연속 흐름 PASS). §3 C4의 원인 구간도 그 문서 §4에서 DDS 발견 지연으로 좁혀졌다. 실차 차단 조건(§7)은 그대로 남는다.
