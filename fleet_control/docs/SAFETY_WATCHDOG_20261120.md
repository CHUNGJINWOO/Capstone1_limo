# Agent-독립 Motion Watchdog · 회귀시험 복구 · Depth 높이 (2026-11-20)

기준선: 브랜치 `staging/fleet-control-on-progress-c804241`, HEAD `c804241`, 단일 작업트리. `fleet_control/`은 미추적이다. 변경 전 스냅샷은 `local_data/worktree_backups_20261119_c103_web/`(`fleet_control_src_before.tar.gz`, `tests_before_repair_20261120.tar.gz`)에 있다. 다른 에이전트의 미커밋 파일(`stall_detector_node.py`, 실차 launch, Depth 필터, `explore/CMakeLists.txt` 등)은 수정하지 않았다.

## 1. Phase 1 — Motion Watchdog

### 1.1 이전 "STARTUP_NO_HEARTBEAT"의 원인 (로그 근거)

- 실패 실행의 watchdog 상태를 보면 heartbeat는 **수신되고 있었다**(Agent boot ID 기록, heartbeat age 약 0.04 s). `STARTUP_NO_HEARTBEAT`는 heartbeat 수신 후에도 갱신되지 않은 **틀린 사유 문자열**이었다.
- 실제 차단 원인은 재무장 조건 `nav_active == 0`이다. Nav2는 goal 상태가 바뀔 때만 `/navigate_to_pose/_action/status`를 발행하므로, goal이 없으면 상태 메시지가 0건(모니터 기록 0건)이다. 이 조건은 영원히 만족되지 않았다.
- 그 결과 Mapping goal은 watchdog OK를 기다리고 watchdog은 goal 상태를 기다리는 교착이 발생했다.
- Topic, QoS, domain, boot ID는 문제가 아니었다.

### 1.2 수정 (상태기계)

| 상태 | 출력 | 전이 |
|---|---|---|
| `LOCKED` (시작·고장 후) | 0, 입력 폐기 | 다음 세 조건을 모두 만족하면 `MOTION_AUTHORIZED`:<br/>① 연속 heartbeat 1.0 s (프로세스 + 제어 루프 `tasks_age_s` ≤ 0.5 s)<br/>② Gate 상태 데이터 신선(≤ 0.5 s)<br/>③ **살아 있는 Nav2 action server가 cancel-all 탐침에 빈 `goals_canceling`으로 응답**(1.5 s 이내)하고, 수신된 상태 중 활성 goal이 없음 |
| `MOTION_AUTHORIZED` | 입력 그대로 전달(0 명령 포함) | 다음 중 하나면 `FAILSAFE`: heartbeat 상실, unhealthy 신고, 제어 루프 정체, Gate 상태 상실 |
| `FAILSAFE` | 0, 입력 폐기, cancel-all 반복 | 서버가 "취소할 goal 없음"을 응답하면 `LOCKED`(재증명 필요) |

- 상태 미수신은 goal 부재의 증거로 쓰지 않는다. 증거는 cancel-all의 빈 응답뿐이다. 실제 Nav2에서 goal이 없을 때 빈 응답, 활성 goal이 있을 때 비어 있지 않은 응답이 오는 것을 격리 실행으로 확인했다(`cancel_empty`, `cancel_nonempty` 계수).
- 이전 goal은 고장 시 취소된다. 재시작한 Agent·watchdog도 증명 전에는 속도를 내보내지 않으므로 이전 goal이 자동으로 재실행되지 않는다.
- Gate 생존은 **데이터 신선도**로 판단한다. ROS 그래프에는 SIGKILL된 프로세스가 DDS participant lease 만료(관측 약 20 s)까지 남아 있기 때문이다.
- `MOTION_AUTHORIZED`는 Agent 측 `watchdog_ok()`의 유일한 OK 상태다(`READY_TO_SUBMIT`과 같다). 새 goal은 여기에 더해 Mapping 세션·boot·예약 UUID·Lease 검사를 그대로 거친다.
- watchdog이 죽으면 `/cmd_vel_raw`가 끊기고, 기존 Gate가 0.25 s 입력 timeout으로 0을 출력한다(시험으로 확인).
- 한계: heartbeat는 Agent의 ROS executor가 발행한다. executor와 무관한 Agent 스레드만 교착되는 경우는 감지하지 않는다. 그런 경우는 Agent 내부 fence(3 s)가 담당한다.

### 1.3 고장 주입 결과 (격리 ROS 2, domain 221, 모터 잠금, 정지 합성 센서, 실제 Nav2·SLAM·Gate)

각 실행은 새 스택에서 수행했다. 사전조건 8개(Agent 연결, watchdog `MOTION_AUTHORIZED`·boot 일치, goal 전 idle 증명, goal ACCEPTED·UUID, Mapping 세션 소유, Nav2·watchdog·Gate 비영 출력)를 모두 만족해야 유효 실행이다. 하나라도 실패하면 INVALID이며 판정하지 않는다.

시간은 고장 시각 기준(s)이다. 서버 단절·Lease 만료는 Agent 쪽 fence가 처리한다.

| 시나리오 | 감지 | watchdog 출력 0 | Gate 출력 0 | Nav2 goal 종료 | 재시작 후 재실행 | 판정 |
|---|---|---|---|---|---|---|
| Agent SIGKILL | 0.29 | 0.25 | 0.28 | 0.32 | — | PASS 14/14 |
| Agent 무응답 (SIGSTOP 25 s) | 0.29 | 0.26 | 0.28 | 0.30 | 없음 | PASS 17/17 |
| Agent 재시작 | 0.28 | 0.24 | 0.27 | 0.30 | 없음, 재무장은 idle 증명 후 | PASS 17/17 |
| Fleet Server 단절 (SIGSTOP 25 s) | Agent fence | 4.39 | 4.42 | 4.27 | 없음 | PASS 15/15 |
| 제어권 상실 (Lease 10 s 만료) | Agent fence | 3.28 | 3.32 | 3.10 | — | PASS 13/13 |
| Watchdog SIGKILL | — | 즉시 정지 | 0.23 (Gate 입력 timeout) | 1.07 (Agent가 watchdog 상실 감지) | — | PASS 13/13 |
| Gate SIGKILL | 0.44 (Gate 상태 상실) | 0.43 | Gate 자체 정지 | 0.48 | — | PASS 13/13 |

수정 전 같은 Gate SIGKILL 시험에서는 goal이 **20.0 s** 동안 유지되어 FAIL이었다(그래프 기반 감지).

재현성 기록 (판정에서 숨기지 않음):
- 격리 Nav2가 lifecycle `change_state` timeout으로 기동하지 못한 실행이 약 16회 중 2회 있었다. 해당 실행은 INVALID 처리하고 새 스택에서 다시 실행했다. watchdog은 그 경우 올바르게 LOCKED를 유지했다.
- Agent SIGSTOP→SIGCONT 후 ROS executor(heartbeat)가 재개되지 않은 실행이 3회 중 1회 있었다. 안전 결과(정지·취소·LOCKED 유지)는 정상이었고, 복구성 문제로 기록한다.

### 1.4 정지 시간 요구사항 (가정 표시)

| 항목 | 값 | 근거 |
|---|---|---|
| 최대 선속도 | 0.15 m/s (slow, velocity_smoother 상한) / 0.32 m/s (normal) | 설정 파일 |
| Gate 주기 / 입력 timeout | 50 Hz / 0.25 s | `stall_detector_node.py` |
| watchdog heartbeat timeout / 주기 | 0.3 s / 20 Hz | 본 구현 |
| 실차 제동 거리·지연 | **미측정(가정 아님, 실측 필요)** | — |
| 요구안 | Agent·watchdog·Gate 고장 시 Gate 출력 0 ≤ 0.5 s. 0.15 m/s에서 최대 0.075 m + 제동 거리 | 제안. 실측 제동 거리로 확정 |
| 측정 | 0.24–0.43 s | 위 표 |

서버 단절과 Lease 만료(3–4.5 s)는 Agent·Gate·Nav2 충돌 회피가 살아 있는 감독 상실이며 모터 위험과 구분된다. 이 시간을 줄일지는 운영 정책으로 결정해야 한다(미결정).

### 1.5 남은 실차 차단 조건

- **Gate 자체가 죽으면** `/cmd_vel`이 끊긴다. LIMO 베이스/MCU가 마지막 명령을 유지하는지 확인되지 않았다(시리얼 타임아웃 실측 필요).
- watchdog은 실차 launch에 아직 편입되지 않았다(패치 `proposals_20261120/02`, 다른 에이전트 파일).
- 운영자 복구 API는 계약상 실모드에서 항상 거부된다(`terminal_evidence_unavailable`). Agent가 실행 중 미션을 잃으면 **API로 차단을 해제할 경로가 없다**(P1 운영 차단).

## 2. Phase 2 — 회귀시험 복구

| 시험 | 이전 | 현재 | 조치 |
|---|---|---|---|
| `test_phase20_agent_stub` | 1 FAIL | 7/7 | 고정값 `'11'*16` 대신 Agent가 실제로 예약·전송한 UUID가 intent·handle·terminal 기록에서 일치하는지 검사(더 엄격) |
| `test_recovery` | 11 ERROR | 14/14 | 가짜 Agent에 Safety Context v1 입력과 Lease 부여. 기대값 5건을 계약에 맞춤(아래) |
| `test_ownership` | 2개 실행 후 중단 | 45/45 | 계약 v1로 재작성. 중단된 실행이 PASS로 보고되던 하네스 결함도 수정(예외 = FAIL) |
| stub 기반 안전 시험 9종 | 오늘 heartbeat 추가로 **회귀**(stub에 `create_publisher` 없음) | 전부 PASS | stub에 `create_publisher` 추가(발행 기록) |
| 신규 `test_motion_watchdog.py` | — | 7/7 | 상태기계 규칙별 시험 |
| 전체 (의도한 환경별: stub=ROS 없음, namespace=`unshare`, 그 외 ROS domain ≤ 232) | 17 실패 | **75/75 PASS** | 실행기 `scratchpad/regress_final/run.sh` |

완화가 아닌 계약 변경을 반영한 기대값(테스트 주석에 근거 명시):
1. 서버 재시작 또는 오프라인 후 전달된 미션은 RECONCILIATION_REQUIRED로 남고 EXECUTING으로 돌아가지 않는다. 실제 terminal 결과는 수용한다.
2. reconciliation 중에는 새 이동 미션 **생성 자체가 거부**된다(이전: 대기열 보류).
3. Lease 없이, 또는 타인의 idempotency key로 미션을 만들 수 없다.
4. Lease를 반납하면 미전달 미션이 무효화된다.
5. 실모드 운영자 복구는 계약상 거부된다.

`check_creator_allowed`는 호출되지 않는 dead code다.

## 3. Phase 3 — Depth 높이 (변경 없음, 제안)

- 실차 TF는 `base_footprint→base_link` z=0, 바퀴 중심 z=−0.100, URDF 반지름 0.045다. 따라서 `base_footprint`는 바닥 위 약 0.145 m(추정)에 있다.
- 필터 `[0.025, 0.35]`는 바닥 기준 약 0.17–0.50 m에 해당한다.
- 2026-11-10 분석의 "실차 `base_footprint_z`=0이면 바닥" 가정은 기록 데이터와 맞지 않는다.
- 높이 의존 요소:
  - Depth 필터(`height_frame=base_footprint`): 직접 영향
  - Costmap Depth 소스 `[0.02, 0.40]`(odom 기준, `base_footprint` z=0): 직접 영향
  - Gate 가상 범퍼(`base_footprint`)의 z는 Costmap 최소 0.01을 넘어야 한다: 수정 후 재확인 필요
  - 장애물 기억 노드: 평면(z 무시)이라 무관. 단, 필터를 통과한 점만 받는다
- 수정 방안(미적용): 실측값으로 기존 launch 인자 `base_footprint_z:=<값>`을 설정한다. 필터 값 자체는 바꾸지 않는다.
- 경고 키 결함 패치: `proposals_20261120/01`.
- 실측 목록(모터 잠금): ① 바닥→`base_link` 높이(바퀴 반경 포함) ② 카메라 광학중심 높이·pitch ③ LiDAR 빔 높이 ④ 의자 연결봉·책상 하부 프레임·문턱 높이 ⑤ DaBai 수직 FOV·최소 거리 ⑥ 실차 제동 거리(0.15/0.32 m/s) ⑦ LIMO 베이스의 `/cmd_vel` 미수신 시 정지 시간(MCU 타임아웃)

## 4. Phase 5 — 후속 계획 (P0 해소 후, 순서대로)

1. **실차 적용 전 P0 마무리**: 제동 거리·MCU 타임아웃 실측 → 패치 02(watchdog 편입)·03(Gate 요구) 소유 에이전트와 합의·적용 → 잠금 상태 실차 watchdog·heartbeat 확인.
2. **Gazebo 폐루프**:
   - 시뮬 launch에서 자체 SLAM·explore 끄기
   - `SIM_GATED` 경로: Nav2 → watchdog → Gate → gz bridge
   - 단일 goal 제출, 도달·취소, Agent·watchdog·Gate 종료 시나리오를 `motion_faults.py`로 재사용
   - Velocity Arbiter(origin/main)는 수동 입력이 필요할 때만 선별 이식
3. **CONTINUE_MAP**:
   - Worker가 Registry bundle에서 부모 posegraph를 받는다
   - SLAM `map_file_name`과 운영자 초기 pose 또는 `map_start_at_dock`으로 시작
   - localization 확인(스캔 정합 점수, `map→odom` 안정)이 끝난 뒤에만 Frontier 시작
   - SAVE 시 `CONTINUE_MAP` lineage로 새 버전을 등록하고 부모를 보존. 롤백 = 부모 재선택
   - 격리 실험 결과(부모 셀 100% 보존, 잘못된 자세에서 벽 중복)를 회귀 기준으로 사용
4. **웹 관제 실차 단계**:
   1. C103 지도 표시(현장 Registry 등록은 백업·drill 절차대로, 승인 후)
   2. LIMO 연결·센서 상태
   3. 모터 잠금 명령 시험
   4. 안전 차단 확인
   5. 승인 후 저속 단계 주행
5. **지도 품질**: Frontier → VERIFY(벽 RANSAC 직선성·직교성, posegraph 노드 밀도·루프 수, 정적 지도 Frontier BFS 재현) → REVISIT(저관측·이중 벽 후보) → SAVE. 종료 조건은 시간·목표 수·배터리(구현됨)와 이동 거리·Gate 상태(추가). 강의실 내부는 제외.

## 5. 재현

```bash
# 격리 스택(explore_lite 오버레이 필요) + 고장 시나리오 1개
OVERLAY=<overlay>/install fleet_control/tests/web_mapping_isolated/run.sh <new run dir> 28951 221
python3 fleet_control/tests/web_mapping_isolated/motion_faults.py <run dir> 28951 agent_kill
fleet_control/tests/web_mapping_isolated/stop.sh <run dir>
# watchdog 단위
python3 fleet_control/tests/test_motion_watchdog.py
```

증거: [local_data/raw_results/safety_watchdog_20261120/](../../local_data/raw_results/safety_watchdog_20261120/)에 실행별 `vel.jsonl`(속도·goal·watchdog 상태 원시 기록), `motion_faults_<scenario>.json`, 스택·Agent·watchdog·서버 로그, 회귀 요약, `MANIFEST.sha256`이 있다. 토큰·secrets는 복사하지 않았다. INVALID 실행(`mf4_agent_kill`, `mf2_gate_kill`)과 수정 전 FAIL 실행(`mf3_gate_kill`)도 함께 보존했다.

## 6. Safety Gate·실차 Launch 통합 검토 (2026-11-20 오후)

요청: 패치 02·03을 최신 Gate·Launch와 비교하고, 목표 초기화 안전성과 재시작 정책을 검증하며, Gate와 함께 격리 시험한다. 다른 에이전트 파일(`stall_detector_node.py`, 실차 launch)은 **수정하지 않았다**(sha256 시작·종료 동일). 개정 패치는 `proposals_20261120/02b·03b·04`, 목록·사유는 `proposals_20261120/README.md`.

### 6.1 최신 코드와의 비교 결과

| 항목 | 결과 |
|---|---|
| 02·03 적용 가능성 | 둘 다 `git apply --check` 통과(충돌 없음). 그러나 **03은 기능 결함**: `state == 'OK'` 비교 → watchdog 상태값(`MOTION_AUTHORIZED`)과 불일치 → 켜면 항상 0 출력 |
| 02 경로 충돌 | `manual_mapping_probe` 텔레옵이 `/cmd_vel_raw`에 직접 발행(watchdog과 중복 발행자). `start_explore`는 Nav2에 직접 goal 제출(watchdog이 취소). watchdog 없이 모터 허용을 막는 조건 없음 |
| 직접 `/cmd_vel` 발행자 | 실차 launch 안에는 Gate뿐(Nav2 smoother·behavior_server는 remap). launch 밖: `limo_ros2_application`의 `move_limo`·`move_to_pose`·`limo_control`이 `cmd_vel`에 직접 발행(실행 경로 미참조, 수동 실행 시 우회). Agent는 `/cmd_vel` 발행자가 있으면 ARMED로 보고 거부 |
| 이전 시험의 Gate | **§1.3 시험은 설치본 Gate(`capstone_ws/install`, 11-10 빌드)를 사용**했다. 작업트리 Gate는 `max_abs_linear_x` 제한이 추가된 버전이다. 고장 시간 판정에는 영향이 없는 차이지만 기록한다. 이번 시험은 작업트리 Gate + 03b + 04 사본을 직접 실행했다 |
| Gate NaN 처리 | NaN 각속도는 모든 임계 비교를 통과해 출력에 그대로 나간다(선속도는 `max_abs_linear_x>0`일 때만 정리). 패치 04 제안, watchdog도 비유한 입력을 0으로 대체 |
| LIMO 드라이버 | `limo_driver.cpp`에 명령 timeout 없음. Gate가 죽은 뒤 정지는 MCU 펌웨어 동작에 달림(**미측정**) |

### 6.2 목표 초기화 안전성 (요청 C)

실측(rclpy 액션 서버, 2026-11-20): cancel 콜백이 거부하면 **활성 goal이 있는데도 `return_code=0`, 빈 `goals_canceling`**이 반환된다. 따라서 빈 배열만으로는 idle을 증명할 수 없다. Nav2(rclcpp)는 goal이 없을 때 `return_code=0`·빈 배열을 반환했다(격리 시험의 `last_cancel_return_code`).

watchdog idle 증명 조건(모두 필요):
1. NavigateToPose 상태 토픽 발행자 정확히 1개(액션 서버 1개). 0개·2개 이상이면 증명 불가, 승인 중에는 `NAV2_ACTION_SERVER_NOT_UNIQUE`로 FAILSAFE
2. 그 발행자를 본 뒤 0.5 s 이상 지난 시점에 보낸 cancel-all (transient-local 상태가 도착할 시간)
3. `return_code == ERROR_NONE` 그리고 빈 배열, 1.5 s 이내
4. 수신한 상태 배열에 비종료 goal 없음(받은 적 없음은 "이 서버가 goal을 수락한 적 없음"으로만 해석)
5. 응답 유실 대비: 2 s 응답 timeout 후 재시도(`cancel_timeouts`)

동시 제출·소유권: Agent heartbeat에 자기 goal UUID(`goal_uuids`, 제출 직전 예약 UUID 포함)를 싣는다. 승인 중 상태 배열에 선언되지 않은 비종료 goal이 0.5 s 넘게 있으면 `FOREIGN_NAV_GOAL`로 FAILSAFE·cancel-all. idle 증명 직후 경합으로 수락된 goal, explore_lite·RViz·다른 클라이언트 goal을 모두 잡는다.

자기 정지: watchdog 프로세스가 0.25 s 넘게 스케줄되지 않으면(SIGSTOP·기아) 재개 첫 tick에 `WATCHDOG_SELF_STALL`로 FAILSAFE. 큐에 남은 heartbeat 때문에 이전 goal이 다시 움직이는 경우를 막는다.

### 6.3 재시작 정책 (자동 재출발 금지)

- Agent가 `require_motion_watchdog`로 실행되면 상태 보고에 `motion_watchdog{required, state, age_s, epoch=watchdog boot:trips}`를 싣는다.
- 서버 Safety Context: required이면 `MOTION_AUTHORIZED`·2 s 이내·Agent boot 일치가 필요하고, `motion_watchdog_epoch`를 Context에 포함한다. 따라서 **trip 또는 watchdog 재시작 → 이전에 승인된 모든 미션 무효**(대기 → FAILED, 실행 중 → RECONCILIATION_REQUIRED, 실제 종료 결과는 그대로 수용). 재무장 후에도 자동 배차 없음. 새 미션은 운영자가 다시 생성해야 한다.
- Agent도 Fleet goal 제출 전(Stage 1~3) watchdog이 OK가 아니면 `MOTION_WATCHDOG_NOT_OK`로 거부.
- 웹 맵핑: watchdog이 OK가 아니면 세션 UNKNOWN, 새 START는 운영자 검토 전까지 409(기존 계약).

### 6.4 격리 재시험 (Watchdog + 개정 Gate 03b+04 사본, `require_motion_watchdog=true`)

각 실행은 새 스택(포트 18301–18320, domain 151–170, 모터 잠금, `/cmd_vel` 발행·구독 0). 사전조건 9개(기존 8개 + Gate가 watchdog OK·`/cmd_vel_raw` 발행자 1개 보고)를 모두 만족해야 판정한다. 시간은 고장 시각 기준(s). Gate 출력 차단 시간과 goal 종료 시간을 따로 기록했다.

| 시나리오 | 감지 | watchdog 출력 0 | **Gate 출력 0** | **goal 종료** | 판정 |
|---|---|---|---|---|---|
| Agent SIGKILL | 0.29 | 0.25 | 0.27 | 0.34 | PASS 15/15 |
| Agent SIGSTOP 25 s | 0.28 | 0.25 | 0.28 | 0.34 | PASS 18/18 (재개 후 재실행 없음) |
| Agent 재시작(goal 활성) | 0.25 | 0.25 | 0.25 | 0.28 | PASS 18/18 (재실행 없음, 재무장은 idle 증명 후) |
| Watchdog SIGKILL | — | 즉시 | 0.21 (Gate 입력 timeout·watchdog 상태 stale) | 0.91 (Agent) | PASS 14/14 |
| Watchdog SIGSTOP 25 s | — | 즉시 | 0.22 | 0.90 (Agent) | 15/16 — 아래 참고 |
| Gate SIGKILL | 0.50 | 0.50 | Gate 자체 정지 | 0.51 | PASS 14/14 |
| Fleet Server 단절(SIGSTOP 25 s) | Agent fence | 4.45 | 4.50 | 4.31 | PASS 16/16 |
| Lease 만료(TTL 10 s) | Agent fence | 3.22 | 3.31 | 3.04 | PASS 14/14 |
| 외부 goal 직접 제출(신규) | 1.92 (goal 출현 1.34 + 0.5 s 유예) | 1.91 | 1.93 | 1.96 (모든 goal) | PASS 16/16 |
| `/cmd_vel_raw` 우회 발행자(신규) | Gate 0.38 | (영향 없음) | 0.33, 이후 25 s 계속 0 | 해당 없음 | PASS 13/13 |
| **대조**: 우회 발행자, Gate 요구 끔(현행 Gate와 동일) | — | — | **25 s 내내 우회 명령 출력** | — | 예상대로 FAIL 10/12 |

Watchdog SIGSTOP 16번째 검사 FAIL: SIGCONT 후 모니터 프로세스가 watchdog 상태를 다시 받지 못했다(25 s 정지가 DDS participant lease ~20 s보다 길어 모니터 쪽 재발견 실패, 10 s 대기로 늘려도 동일). 같은 실행에서 watchdog 로그는 재개 직후 `WATCHDOG_SELF_STALL → FAILSAFE → LOCKED`, Gate 진단은 재개 후 `LOCKED`를 받고 1.27 s 뒤 `MOTION_AUTHORIZED`(goal은 이미 취소, 세션 UNKNOWN)로 기록했다. 재개 후 `/cmd_vel_raw`·Gate 비영 출력 0. 안전 결과는 정상이나 이 검사는 **관측 불가로 FAIL 유지**(검사를 완화하지 않음).

INVALID: 첫 Watchdog SIGSTOP 실행 1회(격리 Nav2 lifecycle `change_state` timeout → 액션 서버 없음, watchdog은 LOCKED 유지). 첫 스모크 1회(사전조건이 변경 기록 전용 Gate 행을 잘못 걸러냄 — 하네스 수정).

Nav2(rclcpp) cancel-all 응답은 모든 유효 실행에서 `return_code=0`. 상태 발행자는 정상 시 1개.

### 6.5 서버 단절 4.4 s · Lease 만료 3.3 s 검토

- 원인: Mapping Worker의 renew fence 3 s(0.15 s 폴링 + HTTP timeout). Lease 만료는 서버 시각이고 Agent는 마지막 renew 응답 후 3 s를 기다린다.
- 성격: 감독 상실(Gate·watchdog·Nav2 장애물 회피는 동작). 모터 고장 목표(0.5 s)와 다른 범주다. 0.15 m/s에서 최대 약 0.66 m 추가 이동.
- 판단: 운영 허용 기준은 미결정. **제안(미구현)**: (1) 서버 poll 응답에 `lease_remaining_s`를 넣고 fence = min(마지막 renew + 3 s, 남은 Lease) → 만료 즉시 정지, (2) 서버 단절 fence 1.5 s(Wi-Fi 지터 실측 후 확정).

### 6.6 이번 변경 파일 (모두 `fleet_control/`, 미추적)

- `agent/.../motion_watchdog.py`: idle 증명 강화(return_code·상태 발행자 1개·settle·응답 timeout), goal 소유권(`FOREIGN_NAV_GOAL`), 중복 액션 서버, 자기 정지(`WATCHDOG_SELF_STALL`), 비유한 입력 0, 상태 필드(boot_id 등), `healthy`가 true일 때만 정상.
- `agent/.../ros_adapter.py`: heartbeat `goal_uuids`, `watchdog_report()`(상태 보고 `motion_watchdog`), Fleet goal 제출 전 watchdog 요구.
- `server/fleet_server/safety.py`: 필수 watchdog 조건과 `motion_watchdog_epoch` Context.
- 시험: `tests/test_motion_watchdog.py`(15), `tests/test_watchdog_admission.py`(4, 신규), `tests/web_mapping_isolated/`(`stack.launch.py` Gate 선택, `vel_monitor.py` Gate 행, `helper.py` 신규, `motion_faults.py` 시나리오 3개), `tests/real_locked_probe.py`(신규, 실차 잠금 점검).
- 제안: `docs/proposals_20261120/02b·03b·04`, `README.md`.
- 백업: `local_data/worktree_backups_20261119_c103_web/watchdog_before_20261120b.tar.gz`.
- 증거: `local_data/raw_results/safety_watchdog_20261120b/` (Gate 사본 SHA 포함).
