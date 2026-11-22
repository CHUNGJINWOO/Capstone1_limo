# Codex Phase 4·5 독립 검증 결함 수정 (2026-11-21)

입력: `local_data/raw_results/codex_phase45_independent_review_20261121.md`, `codex_phase45_independent_20261121_final_v3.log`,
Codex 경계 시험 `fleet_control/tests/test_phase45_independent_20261121.py` (14), `test_phase45_crosslink_http_20261121.py`.
Codex 시험 파일은 수정하지 않았다 (시작 시 SHA-256과 동일, `protected_before.txt`).
결과 디렉터리: `local_data/raw_results/codex_fix_phase45_claude_20261121/`.

범위: 기존 Supervisor / Fleet Server / Robot Agent / 방문객 API / 웹 UI 재사용, 재설계 없음.
실차 LIMO 연결·운영 18190 서버·실차 서비스 설치·모터 잠금·Mapping START 차단·Gate/Watchdog 코드는 건드리지 않았다.
`capstone_ws/`, `rmf_adapter/`, Frontier C++, Gate 후보, 다른 에이전트의 미커밋 변경은 수정하지 않았다.

## 1. 결함별 원인과 수정

| # | 결함 (Codex) | 원인 (코드) | 수정 | 파일 |
|---|---|---|---|---|
| R1 | 빈 Supervisor 보고·Watchdog 누락이 READY | `readiness()`가 **보고된** 서비스·토픽만 순회. 필수 목록이 없었음 | 서버측 필수 서비스 `stack,watchdog,agent`, 필수 토픽 `/scan,/odom` (CLI `--required-services/--required-topics`). 누락 `SERVICE_MISSING`, 중복 `SERVICE_DUPLICATE`, 미정의 상태 `SERVICE_STATE_INVALID` → FAULT | `readiness.py`, `__main__.py` |
| R2 | Gate/Watchdog `age_s`=30 s가 READY | Gate·Watchdog 보고 나이를 읽지 않았음 | Gate `age_s` 유한·0..1.0 s·`rx_fresh`≠False, Watchdog `age_s` 유한·0..1.5 s. 아니면 `GATE:STALE`/`WATCHDOG:STALE` | `readiness.py` |
| R3 | Watchdog LOCKED가 READY | 허용 목록에 LOCKED 포함 | `MOTION_AUTHORIZED`만 READY. LOCKED는 기동·결함 후 상태이고 heartbeat+idle 증거로 자동 재무장 (`motion_watchdog.py` 문서) | `readiness.py` |
| R4 | STALL_PENDING + `cmd_blocked=True`가 READY | `cmd_blocked` 미확인 | `cmd_blocked is False`만 통과, True→`GATE:CMD_BLOCKED`, 누락/비bool→`GATE:CMD_BLOCKED_UNKNOWN` | `readiness.py` |
| R5 | NaN `/scan` age가 READY | `None` 검사 뒤 `>` 비교 (NaN은 항상 False) | `_num()` 유한 실수만 (bool 제외). `INVALID_AGE`; 비정상 `max_age_s`는 1 s로 대체; 미래 타임스탬프 무효 | `readiness.py` |
| R6 | 플래그 누락 시 Supervisor 미강제 | 기본 비활성, `run_field_server.sh`에 없음 | `run_field_server.sh`에 `--require-supervisor-ready` 추가. 추가로 **Agent가 `mode=real` 또는 `simulated=false`를 보고하면 서버 플래그와 무관하게 강제**. 명시 mock/sim 또는 해당 필드가 없는 Agent만 플래그를 따름 (Codex 관찰 시험과 기존 격리 E2E 유지) | `readiness.py`, `scripts/run_field_server.sh`, `guide.py` |
| R7 | READY와 최종 주행 허가 혼동 | 단일 level | `motion_permitted` = READY AND Agent Gate 판정 `movement_permitted is True`; 차단 사유 `motion_block_reasons`. 화면 표기. 어느 쪽도 주행을 부여하지 않음 (Mode·Lease·Arbiter·Watchdog·Gate가 결정) | `readiness.py`, `system_panel.js` |
| S1 | legacy(모드 기록 없음)에서 active goal 중 restart 허용 | 모드 기록만 검사 | `_service_idle_block`: 신선한 Agent 상태(≤2 s) 필수, `execution.active`, Nav2 `active_count`/goal uuid, Mapping worker, Manual bench, Arbiter source, in-flight mission 모두 확인 | `readiness.py` |
| S2 | FAULT/RECONCILIATION_REQUIRED를 IDLE로 취급 | MANUAL/MAPPING/NAVIGATION만 거부 | 모드 기록이 있으면 `IDLE`만 허용 (FAULT 등 `robot_not_idle`), reconciliation → `reconciliation_required` | `readiness.py` |
| S3 | Lease 소유 미검증 | 관리자 역할만 검사 | 모든 action에 body `epoch`(int) 필수, 그 관리자가 보유한 live Lease epoch과 일치해야 함 (`lease_required`). start/retry는 STOPPED/FAILED/BACKOFF 서비스에만 | `readiness.py`, `system_panel.js`, `tests/phase4/run_phase4.py` |
| V1 | 비소유자가 `enabled:'false'`로 서비스 종료 | `enabled is False`만 소유 검사, `is not True`면 종료 | `type(enabled) is bool` 아니면 400. 종료는 Lease 보유자 또는 admin | `guide.py` |
| V2 | 다른 링크·같은 session으로 조회/취소 | session만 비교 | `_own_request`: `link_id` AND `session` 일치 (상수시간 비교), 아니면 404. active 요청 검사도 (link, session) | `guide.py` |
| V3 | 취소가 요청과 무관한 mission을 취소할 수 있음 | `mission_id`만 사용 | mission `idempotency_key == 'guide-'+request_id`일 때만 cancel 전달 | `guide.py` |
| V4 | 브라우저 저장소 키 전역 공유 | `visitor_session`/`visitor_request` 고정 키 | 토큰 FNV-1a 해시로 키 namespace, 구 전역 키 삭제. 토큰은 sessionStorage만 | `static/visitor.js` |
| Q1 | 신규 요청이 오래된 QUEUED를 추월 | `create_guide_request`가 직접 dispatch, FIFO 검사 없음 | 생성도 `_dispatch_one` 경유, 잠금 내 FIFO head 검사 (`_older_waiting`) | `guide.py` |
| Q2 | housekeeping과 신규 요청이 같은 로봇에 2개 mission | busy 검사와 mission 생성 사이 잠금 없음 | 가용성·busy 힌트는 잠금 밖; **잠금 안에서** 요청 상태 재확인 + FIFO + `_robot_reserved`(in-flight mission 또는 ACCEPTED/IN_PROGRESS/CANCELING guide 요청) + `create_mission` + 요청 갱신. 잠금 구간은 DB만 (ROS/HTTP 호출 없음). 멱등 키로 같은 요청 재dispatch는 같은 mission | `guide.py` |
| Q3 | mission 생성 실패·서버 crash 시 요청 고착 | 일반 예외 미처리, RECEIVED 방치 | ApiError: 일시 코드는 QUEUED, 나머지 FAILED; 기타 예외 FAILED `MISSION_CREATE_ERROR`; 5 s 넘은 RECEIVED는 tick이 QUEUED로 | `guide.py` |
| M1 | quarantine 조회 실패가 "격리 아님" | `except: return False` | `None`(UNKNOWN) → `MAP_STATUS_UNKNOWN`, 경고 이벤트 (rate-limit) | `guide.py` |
| M2 | VERIFIED 지도 파일 변조 후에도 안내 | Registry 메타데이터만 사용 | `MapRegistry.verify_bundle_integrity` (yaml/pgm/posegraph SHA-256·content·치수) 가용성 판단과 **dispatch 시점** 모두. stat 서명(ino,size,mtime,ctime) + 30 s TTL 캐시, 최근 2 s 내 수정 파일·실패는 캐시 안 함 (racy-git 규칙). 변경·실패 시 goal-check raster 캐시 폐기 | `guide.py` |
| M3 | Registry 지도와 Agent 사용 지도 불일치 | `map_version_id`만 비교 | navigation 기록 `pgm_sha256` == Registry `pgm_sha256` (Agent는 다운로드 파일 SHA를 기록과 대조: 기존) | `guide.py` |
| M4 | Real Agent의 Mapping START가 `mapping_real_blocked` 대신 `robot_not_ready`로 거부 (회귀 `test_web_mapping_session.test_real_agent_is_blocked`) | R6로 실차 Agent에 readiness가 강제되면서 `require_ready`가 실차 차단 검사보다 먼저 실행돼 거부 사유를 가림 (거부 자체는 유지) | `_mapping_real_block()`을 `require_ready` 앞에 둠. Mapping START와 MAPPING 모드 요청 모두. 실차가 아니면 순서 변화 없음 | `mapping.py`, `operating_mode.py` |
| E1 | Phase 5 E2E rc=134 | executor daemon thread를 join하지 않고 종료 | 분리 실험(아래) 후 `shutdown_ros()`: `ex.shutdown()` → spin thread join → node destroy → `try_shutdown`. 종료 코드 = 판정 (FAIL 있으면 1) | `tests/phase5/run_phase5.py`, `tests/gazebo_web/run_gazebo_web.py` |

C103 UNVERIFIED 정책은 그대로 (`MAP_NOT_VERIFIED`, 시험 `test_unverified_c103_policy_kept`).

### rc=134 분리 실험 (`rc134_experiment/`, isolated domain 93, 12회씩)

| 종료 순서 | rc |
|---|---|
| shutdown + try_shutdown (원본) | 134 ×12 |
| + thread join | 0 ×12 |
| + node destroy (join 없음) | 134 ×12 |
| + join + destroy | 0 ×12 |

stderr: `terminate called without an active exception` (12/12). 원인은 join 누락이다. node destroy는 영향이 없다.

## 2. Codex 시험 재실행 (같은 환경, 기대값 수정 없음)

| 시험 | 수정 전 | 수정 후 |
|---|---|---|
| `test_phase45_independent_20261121.py` (14) | FAIL 13 / ok 1 (`before_independent.log`) | ok 13 / ERROR 1 (`after_independent.log`) |
| `test_phase45_crosslink_http_20261121.py` | FAIL | OK |

남은 ERROR 1건: `test_different_link_cannot_view_or_cancel_when_session_id_known`. 이 시험은 단언 전에 `print`로
다른 링크의 조회 결과를 출력한다. 그 줄이 이제 요구사항대로 `ApiError 404`를 던진다. Codex의 HTTP 시험도 같은 경우에
조회 404를 기대하고, 그 시험은 OK다. 단언 대상인 취소 거부도 충족한다. 시험 파일은 Codex 소유라 고치지 않았다.
**제안 (Codex):** 출력 줄을 `assertRaises`로 감싸거나 제거.

## 3. 추가 검증 (Mock / unit, `test_phase45_fix_20261121.py` 21 시험)

정상 READY와 `motion_permitted` 분리 · 실차 Agent는 플래그 없어도 Supervisor 강제 · stale·미래·중복·잘못된 형식 보고 ·
Gate·Watchdog 증거 누락 · 정상 IDLE restart 동작 · Lease epoch 누락·불일치·타입 · active goal / Nav2 goal / Mapping / Manual /
stale state 시 stop 거부 · FAULT는 IDLE 아님 · RUNNING 서비스 start 거부, FAILED retry 동작 · 같은 방문객 조회·취소 동작,
다른 링크·같은 링크 다른 session 거부 · `enabled` 엄격 타입, 소유자 종료 동작 · 중복 요청 · 8 동시 요청 + housekeeping →
ACCEPTED 1, mission 1, QUEUED 7 · 서버 재시작 후 FIFO 유지·중복 mission 없음 · crash로 남은 RECEIVED 복구 · mission 생성
실패 3종 · 지도 파일 변경 → 차단·raster 캐시 폐기·원복 시 복구·삭제 시 차단 · 대기 요청이 변조 지도로 dispatch 안 됨 ·
Agent 지도 바이트 불일치 · C103 정책.

## 4. E2E 회귀 (isolated ROS kinematic / Gazebo)

환경: Ubuntu 22.04 / ROS 2 Humble, 개발 PC 1대, 격리 ROS domain (226/227/228/224), 격리 포트 (2896x), 시험마다 새 데이터 디렉터리.
운영 18190 서버와 LIMO는 사용하지 않았다. 실행 스크립트: `chain.sh`(1차), `chain2.sh`(2차), `chain3.sh`(3차). 수치는 각 `*/harness.log`의 SUMMARY 그대로다.

| 수준 | 시험 | 1차 (수정 직후) | 2차 (M4 후) | 3차 | 수정 전 이력 (같은 하네스) |
|---|---|---|---|---|---|
| Mock / unit | 전체 unit runner 121개 | 0 이외 25 | 0 이외 24 | — | 기준선 `refactor35/unit_after_phase45` 0 이외 21 |
| isolated ROS | Phase 1 | 19/0 | — | — | 19/0 |
| isolated ROS | Phase 2 모드 (16 지정) | 17/0/0 | 17/0/0 | — | 17/0, 16/0/1 |
| isolated ROS | Phase 3 | 18/0 | — | — | 18/0 |
| isolated ROS | Gate Reset | 13/0, SNAPSHOT_SAME | — | — | 13/0 |
| isolated ROS | Phase 4 server_first | 6/2 | 2/6, 8/0 | — | 0/4, 4/4, 7/1, 2/4, 8/0, 8/0 |
| isolated ROS | Phase 4 supervisor_first (sim 20 s 지연) | 8/0 | 6/2 | 1/7 (Nav2 lifecycle timeout → stack 4회 재시작, `NAV2_SERVER_MISSING` 지속) | 4/4, 8/0 |
| isolated ROS | Phase 5 방문객 (종료 코드) | 8/0 **rc=0** | 3/5 rc=1 | 8/0 (rc 미기록*) | 0/7, 7/1, 0/8, 8/0, 8/0 (rc=134) |
| Gazebo | gazebo_web 전체 (24) | 13/11 rc=1 | 13/11 rc=1 | — | 전체 하네스 6/10, 10/6, 11/5, 10/5 (16개 시절) |
| 실차 | — | SKIP (금지) | | | |

Unit 차이 해석:
- 기준선 대비 추가된 0 이외 항목은 Codex의 `test_phase45_independent`, `test_phase45_crosslink_http`, `test_phase45_qr_libzbar` 3개뿐이다.
  이 3개는 기준선 시점에 없던 시험이다. 일반 runner에 `CAPSTONE45_SNAPSHOT`·PYTHONPATH가 없어서 import 단계에서 실패한다.
  해당 환경으로 직접 실행하면 §2 결과(ok 13 / ERROR 1, crosslink OK)이고, QR은 9개 decode OK다.
- 1차의 `test_web_mapping_session`은 M4 결함으로 실제 회귀였다. 2차에서 해소됐다.
- 나머지 21개는 수정 전과 같은 목록이다 (ROS graph·Node 경로 등 환경 의존, 이번 변경과 무관).

E2E 실패 분류 (원인을 로그로 확인):
- **Phase 4 s01/s02 및 후속 (server_first 1·2차, supervisor_first 2차):** 기동 직후 Gate가 `FAILSAFE_SENSOR_TIMEOUT`을 latch한다.
  `stall_detector_node` 로그는 `scan:NO_ARRIVAL`, `lidar_odom:NO_ARRIVAL`이다. 즉 kinematic sim의 `/scan`이 실제로 끊겼다.
  새 readiness는 이를 FAULT/DEGRADED로 보고하고 MAPPING/MANUAL을 거부했다. 이는 의도한 fail-closed 동작이다.
  수정 전 코드도 Gate FAULT를 FAULT로 처리했고, 같은 하네스가 수정 전에도 0/4·2/4로 실패한 이력이 있다.
  `kinematic_sim.py`·`stall_detector_node.py`는 이번 작업에서 수정하지 않았다.
- **Phase 5 2차:** 설정 단계에서 Nav2 기동 실패로 하네스가 stack을 재시작했다. 지도 load 2 s 뒤 같은 sensor timeout이 발생했다 (fleet.db 이벤트 17·18).
  - 첫 요청은 `safety_not_ready`로 QUEUED였다 (올바른 차단). 이후 요청은 FIFO head 대기 `ROBOT_BUSY`였다.
  - 하네스는 Gate를 reset하지 않으므로 이후 시험이 모두 연쇄 실패했다.
  - 미확인 READY로 mission을 만든 경우는 없었다.
- **Gazebo g03 → 연쇄:** 탐사 중 로봇 이동 0 m, frontier 후보 1~2이다. 이후 Gate `STALL_LATCHED`, FAULT, `UNKNOWN_AREA`로 g03b·g04·g05·g06·g08·f04~f08이 연쇄 실패했다.
  - 같은 증상(`gz_path_m: 0.0`)이 수정 전 전체 하네스 `gate_stall_gazebo_claude_20261121/gz_full2`, `gz_full4`에도 있었다.
  - Gazebo Agent는 `mode=mock simulated=true`라 새 Supervisor 강제·실차 Mapping 차단 경로를 타지 않는다.
  - 수동 주행·deadman·Gate/Watchdog SIGSTOP/SIGKILL·연결 끊김 시험(g00~g02, g07, f01~f03, d01~d06)은 두 번 모두 PASS다.
  - **판정:** 기존 불안정으로 분류하되, 이번 변경과 무관함을 완전히 증명한 것은 아니다 (탐사 무이동 원인은 별도 조사 필요).

\* `chain3.sh`는 로그 변수 `L` 정의가 빠져 SUMMARY 한 줄 기록과 rc 기록이 누락됐다. 판정은 `p5_r3/harness.log`, `p4_supervisor_first_r3/harness.log`의 SUMMARY다.
실행 중인 스크립트는 수정하지 않았다. Phase 5 rc=0/1 판정 자체는 1·2차에서 확인했다 (FAIL 0 → rc 0, FAIL 있음 → rc 1).

- **Phase 4 supervisor_first 3차 (1/7):** Nav2 lifecycle `change_state` 응답 timeout 이후 Nav2 노드들이 종료됐고, Supervisor가 stack을 4회 재시작했다.
  그동안 `NAV2:NAV2_SERVER_MISSING`이 지속되고 Gate boot reconciliation이 `goal_still_active`로 보류됐다. readiness는 끝까지 FAULT/DEGRADED였다 (차단 유지).
  기동 직후 `cmd_input:NOT_EXCLUSIVE` 결함은 8/0 회차에도 매번 나타나는 정상 기동 과도 상태다.
- **통과율 비교 (완전 통과 회차):** Phase 4 수정 전 5/9, 수정 후 3/6. Phase 5 수정 전 3/6, 수정 후 2/3.
  간헐 실패 원인(kinematic sim `/scan` 중단, Nav2 lifecycle timeout)은 이번 변경 범위 밖이며 미조사다.

정상 기능 유지 근거: Phase 4 8/0 (server_first 3차, supervisor_first 1차), Phase 5 8/0 rc=0 (1차), 8/0 (3차), Phase 1·2·3·Gate Reset 전부 PASS.
이 중 Phase 5 8/0은 방문객 요청 → mission → 도착, 취소 시 정지, FIFO 2명, 불가 사유, 토큰 격리를 포함한다.
즉 정상 자동 기동과 방문객 안내는 센서가 정상인 회차에서 끝까지 동작했다.

## 5. 남은 제한

- 실차 LIMO 미검증: 모든 결과는 Mock / isolated ROS / Gazebo.
- Supervisor 필수 판정은 Agent가 보고하는 `mode`/`simulated`에 의존. 해당 필드가 없는 구버전 Agent는 서버 플래그를 따른다.
- 지도 무결성 캐시: 변경 감지는 stat 서명 + 30 s 재해시. 같은 inode·크기·타임스탬프를 위조하는 로컬 공격은 30 s까지 늦게 잡힐 수 있다.
- 서버 재시작 후에는 NAVIGATION/지도 load 기록을 신뢰하지 않으므로 운영자가 NAVIGATION 진입·지도 load를 다시 해야 대기 요청이 진행된다 (기존 설계, 의도).
- service stop/restart는 Agent가 오프라인이면 거부된다 (증거 없음). 그 경우 start/retry만 가능.
