# 관리자 전용 웹 Gate Reset (2026-11-21, 격리 시험만)

이 작업에서 하지 않은 것:
- LIMO 연결·SSH 없음
- 18190 운영 서버 변경·배포 없음
- 실차 Gate·launch 적용 없음
- 실차 모터 잠금·Mapping START 차단 유지
- Gazebo·실차 시험 보류
- commit·push·PR 없음

Gate는 후보 **r12**(= r11 + Gate가 직접 내리는 Reset 판단)이다.
- field·`capstone_ws` Gate는 미변경이다(다른 담당자의 미커밋 8줄 그대로).
- r11은 동결 사본으로 보존했다.

## 1. 경로와 판단 주체

```
관리자 웹 카드 ─POST /robots/{id}/gate-reset→ Fleet Server ─(state 응답에 1회 전달)→ Robot Agent GateResetWorker ─/stall_detector/reset_latch (기존 Trigger)→ Gate r12
   ▲ GET /robots/{id}/gate-reset (상태·장애 원인·서버/Gate 판단·요청 결과)                                     │
   └──────────── Agent 결과(state.gate_reset) → 서버 기록·감사 로그 ◀──────────────────────────────────┘
```

| 단계 | 검사 | 실패 시 |
|---|---|---|
| **서버** (`gate_reset.py`) | 아래 서버 검사 항목 | 409 `gate_reset_refused` + 감사 기록. 비관리자는 403 + 감사 기록 |
| **Agent** (`gate_reset_worker.py`) | 아래 Agent 검사 항목 | `REFUSED` 보고, Gate 호출 안 함 |
| **Gate** (r12 `cb_reset_latch`) | 아래 Gate 검사 항목. 상태(`reset_readiness`)로 공개하고 서비스 안에서 **다시** 판단 | `REFUSED: …` 응답, 래치 유지 |

**서버 검사**
- **권한·세션:** 관리자 역할(서버가 직접 확인), 호출자의 Lease·epoch, Agent 온라인·재동기화 완료, 프로토콜 v2, Agent의 Gate Reset 지원 여부
- **운용 상태:** 운용 모드 IDLE이며 전환 중이 아님, 진행 중 임무·맵핑 세션·수동 세션 없음, Watchdog MOTION_AUTHORIZED
- **Gate 상태:** Gate 보고가 신선함, Gate가 실제로 장애 상태임, 관리자가 본 Gate boot·fault_epoch와 일치
- **중복·재생 방지:** 처리 중인 요청 없음, 이미 Reset된 (boot, fault_epoch)이 아님

**Agent 검사**
- **요청:** 처음 보는 요청 id(fsync한 영구 journal에 Gate 호출 **전에** 기록), 5 s 이내, 현재 Agent boot·Server boot
- **세션:** 연결됨, 재동기화 불필요, Lease 소유자·epoch, 모드 IDLE
- **Gate:** 보고가 신선함, 같은 Gate boot·fault_epoch, Gate 자신의 readiness가 true
- **Watchdog과 발행자:**
  - Watchdog이 MOTION_AUTHORIZED이고 같은 Agent boot에 대한 것임
  - `/cmd_vel_raw` 발행자가 정확히 1개이고 이름이 `fleet_motion_watchdog`
  - 그 GID가 Watchdog이 자기 출력이라고 보고한 GID와 같음
  - Watchdog 상태 발행자와 같은 DDS participant(GUID prefix)
- **그래프·속도:** 잠금 그래프(`/cmd_vel` 미사용), Arbiter 출력 0, 수동 샘플 없음, 소유 Goal 없음, Nav2 유휴
- **관찰 창:** 위 조건이 **1.5 s 동안 계속** 성립해야 함. Watchdog 상태 갱신 ≥2회, 발행자 GID 불변

**Gate 검사**
- Reset할 장애가 있음, INITIALIZING이 아님
- 입력 발행자가 1개, 허용된 이름, 판독 가능한 GID
- **입력 발행자가 지금 발행 중**(0.3 s): 죽은 발행자의 유령 endpoint는 1개로 보여도 발행하지 않는다
- 입력 발행자가 Watchdog 상태 발행자와 같은 participant
- 수신 명령에 운동 없음, 출력 0이 1 s 유지됨
- 다른 센서·안전 체인 장애 없음

Reset 성공 후에도 기존 동작이 재개되지 않는다:
- 서버는 모드를 바꾸지 않는다(IDLE 유지).
- 서버 수동 세션을 폐기하고, Agent도 남은 수동 샘플을 버린다.
- Gate는 기존대로 INITIALIZING을 거쳐 운동 명령이 없을 때만 무장한다.
- MANUAL·NAVIGATION은 정상 모드 전환·세션 시작·임무 Admission을 새로 거쳐야 한다.

요청 상태 전이:

```
QUEUED → DELIVERED → SUCCEEDED | REFUSED | FAILED
```

전달·결과가 없거나 boot가 바뀌면 `EXPIRED`, `EXPIRED_NO_RESULT`, `ABANDONED_SERVER_RESTART`, `ABANDONED_AGENT_RESTART`가 된다. 자동 재시도는 없다.

감사 이벤트:
- 종류: `gate_reset_requested`, `gate_reset_refused`, `gate_reset_delivered`, `gate_reset_result`, `gate_reset_expired*`, `gate_reset_abandoned_*`
- 내용: 요청자·역할·request_id·사유·실패 검사·Gate 응답
- 거부 기록은 DB 트랜잭션 **밖**에서 남긴다(구현 중 단위 시험이 롤백 결함을 찾아 수정함).

## 2. 보호 범위와 한계

- 모든 출처 검사는 DDS 그래프·GID 기반이며 **인증이 아니다**.
- 다음 경우는 거부된다(격리 시험으로 확인):
  - Watchdog 이름을 쓰는 별도 프로세스: participant 불일치. Watchdog이 죽은 뒤 단독이면 상태 발행자 0개.
  - 유령 endpoint
  - 출처 불명 GID
- **SROS2 없이는 막지 못하는 경우:** Watchdog 프로세스 자체를 대체하거나, 같은 participant 안에서 위조하는 경우.
- 웹 버튼 비활성화는 편의일 뿐이다. 서버는 역할과 무관하게 모든 POST를 직접 검사·기록한다(e02, 단위 시험).

## 3. 변경 파일

모든 원본은 `local_data/raw_results/gate_input_claude_20261120/pre_change_originals/`에 보존했다.

**Gate**
- `experiments/safety_chain_fix_20261113/candidate_gate_r12/nodes/stall_detector_node.py`(새 후보)
  - r11 사본: `candidate_gate_r12/stall_detector_node.r11_frozen.py`
  - 매개변수: `reset_requires_preconditions`(기본 true), `reset_quiet_sec` 1.0, `reset_input_live_sec` 0.3, `reset_input_peer_topic`

**Server**
- `fleet_control/server/fleet_server/gate_reset.py`(새 파일)
- `fleet_control/server/fleet_server/service.py`: Mixin 추가, state 응답에 결과 수집·요청 전달
- `fleet_control/server/fleet_server/http_api.py`: GET·POST `/robots/{id}/gate-reset`. POST는 역할 검사 전에 라우팅해 비관리자 시도도 감사 기록

**Agent**
- `gate_reset_worker.py`(새 파일)
- `core.py`: `gate_reset_request` 수신
- `agent_node.py`: `web_gate_reset` 매개변수(기본 false)
- `ros_adapter.py`: Gate 보고에 `reset_view` 추가(표시용)
- `motion_watchdog.py`: 상태에 `output_gid` 추가

**UI**
- `static/gate_reset_panel.js`(새 파일)
- `index.html`, `app.js`: 운영 탭에 카드 추가

**시험**
- 새 파일:
  - `tests/test_gate_reset_server.py`
  - `tests/test_gate_reset_worker.py`
  - `tests/test_gate_reset_panel.cjs`
  - `tests/gate_reset_e2e/{run_gate_reset_e2e.py, impostor_pub.py, browser_gate_reset.cjs}`
  - `experiments/.../harness/gate_r12_reset/test_gate_r12_reset_unit.py`
- 수정(env 가드, 기본값 불변):
  - `tests/mode_e2e/stack_up.sh`: `AGENT_EXTRA`, `GATE_RESET_PEER`
  - `tests/web_mapping_isolated/stack.launch.py`: `GATE_RESET_PEER`
  - `tests/manual_e2e/run_manual_e2e.py`: s10 진단 기록만 추가, 판정 불변
  - `experiments/.../harness/gate_r11_input/run_gate_input.py`: r12 선택지

## 4. 결과

### 4.1 단위 시험

| 대상 | 결과 |
|---|---|
| Gate r12 Reset 판단 | 14/14 PASS |
| 서버 | 14/14 PASS |
| Agent worker | 11/11 PASS |
| 웹 카드 view | PASS |

**Gate r12:** 정상 수락, INITIALIZING, NOTHING_TO_RESET, sticky 입력 래치, 발행자 2개, GID 불명·이름 불허·없음·조회 오류, 운동 명령, 출력 비영, 장애 지속, 평가 전, 발행자 무발행, peer participant, 비활성 시 r11 동작.

**서버:** 모든 비관리자 역할 403과 감사 기록, Lease·epoch, Gate 정상·변경, 보고 경과·오프라인·재동기화·미지원·Watchdog, 모드·전환·임무·수동 세션, 1회 전달·결과·IDLE 유지, 거부 결과 기록, 결과 재생·타 id 무시, 동시 요청 8개 중 1개만 수락, 서버·Agent 재시작 폐기, 만료 2종, 같은 fault_epoch 재Reset 거부, 역할별 view.

**Agent worker:** 성공(Gate 1회 호출, journal 선기록, 샘플 폐기), 같은 id 재생은 worker 재시작 후에도 거부, boot·세션·Lease·모드, Gate 동일성·readiness·신선도, Watchdog, 유령·GID 불일치·불명·이름·다른 participant·상태 발행자 없음, 그래프·속도 0, Gate 거부·서비스 없음, 오래된 요청, 관찰 창, run loop 1회 처리.

### 4.2 격리 ROS E2E — Gate Reset

환경: `tests/gate_reset_e2e/run_gate_reset_e2e.py`, 도메인 228, kinematic sim, 실제 Nav2·Gate r12·Watchdog·Agent·서버, Playwright.

**집계 정정:** 하네스의 `record(..., result=...)` 인자가 판정 필드를 덮어써서 e06·e11이 요약 집계에서 빠져 있었다. 수정 후 모든 실행을 13개 기록(e00~e11 + `/cmd_vel` 무접점) 기준으로 다시 셌다.

최종 결과: §4.2.1

| 시나리오 | 확인 내용 |
|---|---|
| e01 정상 복구 | rogue로 Gate NOT_EXCLUSIVE FAILSAFE → 관리자 요청 SUCCEEDED → Gate NORMAL. 모드 IDLE 유지, 이후 출력 0, 감사 3종 기록 |
| e02 권한 | operator·viewer 403(감사 2건), 잘못된 토큰 401, operator view는 `ROLE_NOT_ADMIN` |
| e03 장애 지속 | rogue 유지 중 Gate readiness `INPUT_WRITERS:2`, Agent REFUSED(`gate_ready, publisher_identity, graph`), **웹을 거치지 않은 Gate 직접 호출도 `REFUSED: INPUT_WRITERS:2`** |
| e04 Watchdog SIGKILL 후 재시작 | 유령 기간(약 20 s) 요청은 Agent 단계 REFUSED 또는 서버 단계 `WATCHDOG_NOT_AUTHORIZED`로 거부. 유령 해소 후 SUCCEEDED |
| e05 Publisher 교체 | Watchdog 이름의 별도 프로세스와 동시 존재 시 REFUSED. **단독 남았을 때 Gate 자체 readiness `INPUT_PEER_WRITERS:0`, 직접 호출도 REFUSED.** Agent도 REFUSED(Watchdog 상태·GID 교차검증) |
| e06 동시 Reset | 6개 동시 요청 중 1개 QUEUED, 5개 `gate_reset_in_progress`, 결과 SUCCEEDED |
| e07 서버 재시작 | 대기 요청은 `ABANDONED_SERVER_RESTART`, Gate unlatch 횟수 불변(실행 안 됨) |
| e08 Agent 재시작 | `ABANDONED_AGENT_RESTART`, unlatch 불변 |
| e09 이전 요청 재생 | 성공 직후 같은 값 재요청 409(`FAULT_EPOCH_ALREADY_RESET`/`GATE_NOT_FAULTED`). 새 장애 뒤 옛 값은 409 `GATE_CHANGED_SINCE_VIEWED` |
| e10 자동 재개 없음 | MANUAL 주행 중 장애 → IDLE 전환 → Reset 성공. 옛 수동 세션 샘플 409, 출력 0 유지, IDLE에서 임무 409. MANUAL 재전환과 새 세션에서만 다시 주행 |
| e11 웹 카드 | operator 화면은 버튼 비활성·`관리자 권한 필요` 표시. admin 클릭 → SUCCEEDED. 스크린샷 `gate_reset_card_{operator,admin,admin_after}.png` |

**최종 이전 실행은 결과로 세지 않는다.** 모두 보존했다.

- **`run01`(6/13):** 아래 결함을 찾았다.
  - **e04에서 결함 발견:** SIGKILL 직후 죽은 Watchdog의 마지막 상태(1 s 이내)와 유령 endpoint만 보이는 순간 Reset이 수락되었다. 이어 새 Watchdog이 나타나 Gate가 다시 차단했고 운동은 없었다.
  - → 수정 1: Gate `INPUT_SILENT`
  - → 수정 2: Agent 1.5 s 관찰 창(Watchdog 갱신·GID 불변)
  - → 수정 3: 서버 `WATCHDOG_NOT_AUTHORIZED`
  - 나머지는 하네스 대기 조건 문제였다.
- **`run02`(13/13):** e05에서 **Gate 자체는 사칭자 단독 상태를 ready로 판단**했다(Agent만 거부). → Gate에 `reset_input_peer_topic` participant 검사를 추가했다.
- **`run03`(12/13):** e00 대기 조건 문제.
- **`run04`(12/13):** e09에서 **서버가 보고 지연 중 같은 fault_epoch 재요청을 수락**했다(Agent·Gate는 NOTHING_TO_RESET로 거부했을 것). → `FAULT_EPOCH_ALREADY_RESET`을 추가했다.
- **`run05`(13/13).**
- **`final/gate_reset_e2e_1/2`(3/13 두 번):** 알려진 Nav2 수명주기 응답 유실로 Nav2가 활성화되지 않았다. Watchdog이 LOCKED로 남았고 모든 요청이 `WATCHDOG_NOT_AUTHORIZED`로 **올바르게 거부**되었다. → 하네스에 기록되는 Nav2 기동 가드를 추가하고 재실행했다.

- **`final_rerun/gate_reset_e2e_1`(13/13), `_2`(12/13):** 집계 정정 후 기준이다.
  - `_2`의 e06: 동시성 자체는 정상이었다(1 QUEUED, 5 `gate_reset_in_progress`).
  - 그러나 e05에서 재시작된 Watchdog이 1.5 s 관찰 창 동안 계속 승인 상태가 아니었고, Agent가 `AGENT_PRECHECK:watchdog`로 거부했다. 의도된 안전 측 거부다.
  - e06이 Watchdog 3 s 연속 승인을 기다리도록 하네스를 수정했다.

#### 4.2.1 최종 실행 (`final_rerun2`, 스냅샷 동일)

**`gate_reset_e2e_1` 13/13 PASS, `gate_reset_e2e_2` 13/13 PASS.**
- e00~e11과 `/cmd_vel` 무접점 기록을 각각 판정했다.
- Nav2 기동은 두 번 모두 1회에 성공했다.

### 4.3 회귀 (Gate r12, 같은 스냅샷)

| 시험 | 결과 | 비고 |
|---|---|---|
| 단독 입력 출처 10종(r12) | 차단 결과는 r11과 동일. 복구 단계는 r12가 운동 명령 중 Reset을 **거부**(`MOTION_COMMANDED`) | 하네스가 실제 Arbiter처럼 0 송신 후 Reset하면 10종 모두 r11과 동일하게 복구(`standalone_input_r12_run2`) |
| **Phase 1 + 브라우저(r12)** | **19/19 PASS × 2** (`final_rerun/phase1_r12_browser_1,2`) | `final/phase1_r12_browser_*`의 s16 FAIL은 체인 스크립트의 NODE_PATH 전개 오류(Playwright 미발견). 시험 결과 아님 |
| **s10 FollowPath 별도 회귀** | r12 4/4 PASS(0.023, 0.042, 0.038, 0.043 s) | 진단 기록(Goal 전후 출력 샘플·차단 타임라인)을 추가했다. r11의 `None` 2회는 재현되지 않았고 원인은 여전히 미확인이므로 회귀 항목으로 유지 |
| Phase 2 원래 17(r12) | 16/1 (f06 `localization_not_confirmed`) | 같은 변동이 r10·r11에서도 있었음 |
| Phase 3(r12) | 14/4, 14/4, 11/7 — §4.4 | 같은 하네스에서 r11도 18/0 뒤 10/8 |
| 단위·회귀 전체 | 이전과 동일. 새 시험 전부 PASS | 기존 스텁 필요 2개 파일의 오류는 그대로(변경 전부터) |

### 4.4 Phase 3 비교 (같은 하네스·같은 장비, 번갈아 실행)

| Gate | 실행 | 결과 |
|---|---|---|
| r12 | `final/phase3_r12` | 14/4 |
| r12 | `final_rerun/phase3_r12_rerun` | 14/4 |
| r11 | `final_rerun2/phase3_r11_compare` | 18/0 |
| r12 | `final_rerun2/phase3_r12_alt` | 11/7 (Nav2 기동 실패 후 스택+Agent 재시작 1회) |
| r11 | `final_rerun2/phase3_r11_alt` | 10/8 (localization CONFIRMED 실패) |

**실패 분석:**
- r12 실패 대부분은 localization DEGRADED·미확인, Nav2 미션 중단, LiDAR odom 공백 센서 FAILSAFE였다. 이 구간에 Gate Reset 로직은 동작하지 않았다(Gate 로그 확인).
- 마지막 r12 실행의 n03 거부 순간 Agent 진단은 Gate NORMAL·차단 없음·localization CONFIRMED였다. 이 실행은 기록된 스택 재시작 뒤였고, 거부는 서버 safety context에서 나왔다. 재시작 뒤 서버의 Gate boot 재동기화 요구가 원인으로 보이나 확인하지 않았다.
- 같은 하네스에서 r11도 10/8로 실패했으므로 환경 변동이 r12와 r11 모두에 나타난다.

**CPU 비교:** 같은 Phase 3 스택에서 Gate 프로세스 CPU를 20 s 측정했다. r11 14%, r12 14%로 차이가 없다. 상태 발행 때 graph 조회를 추가한 부하 영향은 확인되지 않았다.

**r12 고유 영향(확인됨):** f08 직전 f07 rogue가 운동 명령을 보내고 있었다. 그래서 첫 Reset이 `MOTION_COMMANDED`로 거부되었고, 3 s 뒤 수락되었다. 그 사이 모드 흐름이 달라질 수 있다. 의도된 규칙이지만 운영 절차에 반영이 필요하다(§5.3).

**결론:** Phase 3가 r12에서 r11보다 나쁜지 여부는 **표본이 부족해 판정하지 않는다.** 실행 5회가 모두 환경 변동의 영향을 받았다. 재평가는 Phase 3 하네스 안정화(Nav2 기동·localization·재시작 후 서버 재동기화) 이후로 미룬다.

## 5. 남은 일과 실차 차단 요소

1. **통합 미적용.** r12는 후보이고 field·`capstone_ws` Gate에는 미적용이다.
   - 소유권 확인 후 담당자 검토가 필요하다(제안 05에 Reset 판단을 더하는 패치는 아직 작성하지 않았다).
   - 운영 서버(18190)에는 배포하지 않았다.
2. **인증 없음.** Watchdog 프로세스 자체 대체나 같은 participant 내부 위조는 SROS2로만 막을 수 있다.
3. **Phase 3 f08 상호작용.** Reset이 운동 명령 직후 거부되는 r12 규칙 때문에, 기존 하네스 흐름(f07 직후 f08)에서 모드가 FAULT로 남은 1회가 있었다. 운영 절차에는 'Reset 전 1 s 이상 0 명령'을 명시해야 한다.
4. **Agent heartbeat 여유 부족(SAVE 중)과 s10 원인 미확인**은 기존 항목으로 계속 남는다.
   - Phase 3에서 r12가 r11보다 나쁜지는 미판정이다(§4.4). 하네스 안정화 후 재평가한다.
5. **보류:** Gazebo·실차, STM32 timeout·E-Stop·제동거리 측정.

## 6. Codex 재현

```bash
cd <repo>; source /opt/ros/humble/setup.bash; source capstone_ws/install/setup.bash
export PYTHONPATH=$PWD/fleet_control/agent/limo_robot_agent:$PWD/fleet_control/server
python3 fleet_control/tests/test_gate_reset_server.py
python3 fleet_control/tests/test_gate_reset_worker.py
node fleet_control/tests/test_gate_reset_panel.cjs
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=229 python3 experiments/safety_chain_fix_20261113/harness/gate_r12_reset/test_gate_r12_reset_unit.py
# 격리 E2E (새 디렉터리만 생성; OVERLAY = explore_lite overlay, Playwright NODE_PATH)
cd fleet_control/tests/gate_reset_e2e && OVERLAY=<overlay> GATE_RESET_E2E_NODE_PATH=<node_modules> GATE_RESET_E2E_OUT=<new_dir> python3 run_gate_reset_e2e.py 28957 228 --browser
# 회귀 묶음: local_data/raw_results/gate_reset_claude_20261121/{final_chain.sh,rerun_chain.sh,rerun_phase3.sh}
```

결과 원본: `local_data/raw_results/gate_reset_claude_20261121/`. 단위 요약은 `unit_after/SUMMARY.txt`.

## 7. 검증 구분

- **검증됨(격리 ROS·kinematic sim·mock):**
  - 서버 권한·감사·동시성·재생·재시작
  - Agent 사전검사·관찰 창·journal
  - Gate r12 Reset 판단
  - 웹 카드(Playwright)
  - Phase 1(r12) 19/19 ×2, s10 4/4
- **검증 안 됨:** 실차, Gazebo, field·`capstone_ws` Gate 통합, SROS2
- **가정:** rmw_fastrtps GID의 앞 12바이트가 participant GUID prefix(관찰상 일치, 문서로 확인하지 않음)
