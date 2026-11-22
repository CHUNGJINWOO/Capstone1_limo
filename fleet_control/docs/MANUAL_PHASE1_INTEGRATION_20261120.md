# Phase 1 — 웹 수동 조작 실제 통합 (격리 ROS, 2026-11-20, Claude)

설계 기준: [INTEGRATED_CONTROL_PLAN_20261120.md](INTEGRATED_CONTROL_PLAN_20261120.md). 증거: `local_data/raw_results/manual_phase1_claude_20261120/` (Git 제외).

## 1. 결론

웹 패널(키보드·터치 조이스틱·Gamepad) → Fleet HTTP → Robot Agent → VelocityArbiter → Motion Watchdog → Safety Gate(`/cmd_vel_locked`) 경로가 **격리 ROS에서 0이 아닌 속도로 동작**하고, 위험 상황에서는 0으로 떨어진다. 전체 E2E 18개 시나리오를 3회 연속 실행해 **54/54 PASS** (run08·09·10).

- 검증 범위: 격리 localhost ROS(domain 228), 실제 Fleet 서버·Agent·Watchdog 프로세스, 현장 번들 Gate 코드, 실제 rclpy ActionServer(Nav2 대체), Chrome 실제 대시보드. 센서는 합성 상수 토픽.
- **미검증**: 실제 Nav2 스택, Gazebo, 실차, 물리 Gamepad(합성 `navigator.getGamepads`), 모터 출력. 실차 real 벤치는 여전히 0 출력 전용이며 모터 잠금은 그대로다.

## 2. 발견한 핵심 결함과 수정

**Nav2 idle 판정이 실제 action server와 맞지 않았다.** rclcpp/rclpy action server는 goal 상태가 바뀔 때만 `…/_action/status`를 발행하고, goal이 없던 idle 서버는 **아무것도 발행하지 않는다**(Humble `action_tutorials_cpp`·`_py` 서버로 확인: 3초간 0건, cancel-all rc 0·빈 목록). 기존 Watchdog(`NAV2_STATUS_STALE_OR_UNKNOWN`), Agent Arbiter(`received_status_array`·`age_s≤1`·FollowPath 표본 ≤1 s·한 번이라도 active면 영구 차단), 서버(`age≤2 s`)는 모두 status 메시지의 존재·나이를 요구했으므로 **실제 Nav2 옆에서는 수동 조작이 영원히 허가되지 않는다**. Codex 체인 시험은 80 ms마다 status를 재발행하는 fixture로 이 문제를 가렸다.

수정 원칙(연속성 증거): 상태 토픽 발행자가 **정확히 하나**이고 **같은 GID로 settle 시간 이상 유지**되며, 가장 최근 status에 비종료 goal이 없거나 status가 한 번도 오지 않았으면 idle이다(goal 수락은 항상 status를 발행하므로). 메시지 나이는 쓰지 않는다. Watchdog은 여기에 기존 cancel-all 빈 응답(ERROR_NONE)을 독립 증거로 계속 요구하고, 발행자 GID가 바뀌면 FAILSAFE 후 구독을 다시 만들어 transient-local status를 재수신한다.

## 3. 변경 파일 (원본은 증거 폴더 `superseded_originals/`에 보존)

| 파일 | 변경 |
|---|---|
| `agent/.../action_status_evidence.py` (신규) | 연속성 기반 action idle 증거. 이전 서버의 active status는 현재 서버에 대한 Watchdog 보증(`vouched_gid`)이 있을 때만 무시 |
| `agent/.../ros_adapter.py` | NavigateToPose·FollowPath 추적기, `manual_idle_evidence()`, 상태 보고 `execution.nav_idle` |
| `agent/.../velocity_arbiter.py` | `blocker()`로 첫 실패 사유 보고(`snapshot.blocker`, `source`), 연속성 증거 사용, 영구 sticky 차단 제거, `close()` 종료 경합 방어 |
| `agent/.../motion_watchdog.py` | 나이 검사 → settle/GID 연속성, GID 변경 시 `NAV2_ACTION_SERVER_CHANGED` + 재구독, status JSON에 `nav_status_gid`·`nav_status_received` |
| `server/fleet_server/service.py` | 수동 admission이 `nav_idle` 연속성 증거 허용·차단 사유 포함, **AUTO 임무 생성 시 MANUAL 활성 거부(`manual_active`)**, `robot_view.control_mode`(MAPPING>AUTO>MANUAL>IDLE, UNKNOWN 실패-폐쇄) |
| `static/manual_control_panel.js`, `static/app.js` | 서버 `control_mode`와 Arbiter 차단 사유를 한국어로 표시, MANUAL 모드에서 세션 유지 허용 |
| 시험 | `tests/manual_e2e/{run_manual_e2e.py,fake_nav2_servers.py,browser_manual_inputs.cjs}`, `tests/test_action_status_evidence.py`; 계약 변경에 맞춘 기대값 수정: `test_motion_watchdog.py`(+3 시험), `test_manual_watchdog_predicate_fix_20261120.py`, `test_manual_watchdog_missing_status_20261120.py` |

재사용(무수정): `manual_web_bench.py`, `agent_node.py`, `ownership.py`, `mapping.py`, `http_api.py`, `index.html`, Gate 코드.

## 4. 실제 데이터 흐름·토픽 (격리 구성)

`POST /api/v1/robots/{id}/manual-bench {start|sample|stop}` (80 ms, Lease owner·epoch·세션·단조 seq, 축 [-1,1]) → 서버 세션(TTL 250 ms) → Agent `POST /agent/manual-bench` (20 Hz, 0.18 s timeout) → `ManualWebBench` 상태 → `VelocityArbiter` (선 [0,0.10] m/s, 각 [-0.30,0.30] rad/s) → `/fleet/nav_cmd_vel` → `fleet_motion_watchdog` → `/cmd_vel_raw` → `stall_detector_node` → `/cmd_vel_locked`. 감시 입력: `/fleet/agent_heartbeat`, `/fleet/motion_watchdog/status`, `/stall_detector/status_json`, `/navigate_to_pose/_action/status`·`cancel_goal`, `/follow_path/_action/status`. `/cmd_vel` 발행·구독 0 확인.

## 5. 시험 결과

**E2E (`tests/manual_e2e/run_manual_e2e.py --browser`)** run08·09·10 각 18/18 PASS. 0 정지 시간은 3회 중 최대값.

| 시나리오 | 결과 |
|---|---|
| s01 idle Nav2가 status를 한 번도 발행하지 않는 상태에서 Watchdog 승인, Lease 전 유일 차단 사유 = `LEASE_MISSING_OR_EXPIRED` | PASS |
| s02 정상 주행: 첫 출력 55–81 ms, 최대 0.10 m/s·0.15 rad/s, `control_mode=MANUAL` | PASS |
| s03 Deadman 해제 → 해제 이후 비영 출력 없음 | PASS |
| s04 입력 중단(브라우저 정지·단절) → 0까지 ≤0.219 s, 이전 세션 409 | PASS |
| s05 STOP → ≤0.048 s, 이전 세션 409 | PASS |
| s06 NaN·범위 밖·seq 재사용 400, viewer 403, Lease 비소유자 409 | PASS |
| s07 MANUAL 중 AUTO 임무 409 `manual_active`(주행 유지), Mapping 활성화(합성 주입) → 0, 시작 거부 | PASS |
| s08 미종료 임무(DB 직접 삽입) → `control_mode=AUTO`, 수동 시작 409, 종료 후 정상 | PASS |
| s16 브라우저: 키보드 W+A+Space, 터치 조이스틱, 합성 Gamepad(L1) 모두 주행·해제 시 0, Gamepad timestamp 정지 시 세션 종료, 375 px 폭 가로 스크롤 없음 | PASS |
| s09 외부 NavigateToPose goal → ≤0.023 s 0, Watchdog `FOREIGN_NAV_GOAL`·cancel, 복구 후 **새 세션으로 다시 주행** | PASS |
| s10 외부 FollowPath goal → ≤0.034 s 0 (`FOLLOW_PATH_GOAL_ACTIVE`), 취소 후 복구 | PASS |
| s11 Nav2 서버 재시작 → `NAV2_SERVER_MISSING` 차단·시작 409 → 재기동 후 복구 | PASS |
| s12 Lease 만료·관리자 강제 해제 → ≤0.092 s, 입력 409 | PASS |
| s13 Agent SIGKILL → ≤0.235 s (`AGENT_HEARTBEAT_LOST`), 재기동 후 이전 세션 409, 새 세션 주행 | PASS |
| s14 Watchdog SIGKILL → ≤0.227 s, 재기동 후 복구 | PASS (복구 ~17–20 s, §6) |
| s15 Fleet 서버 중단 → ≤0.016 s, 재시작(새 boot) 후 이전 세션 409, 새 세션 주행 | PASS |
| s17 Gate SIGKILL → `/cmd_vel_locked` 출력 0건, `SAFETY_GATE_STATUS_LOST`, 시작 409 | PASS |

**회귀**: 94개 시험 파일 실행. 단위·서버 시험(`test_motion_watchdog` 18, `test_action_status_evidence` 4, `test_web_mapping_session` 26, `test_ownership`, `test_watchdog_admission`, `test_manual_*` 다수, `test_static_dashboard`, JS 패널·입력 시험) PASS. 변경 전 트리와 같은 환경으로 비교한 결과 **새로 실패한 시험 없음**. 양쪽 모두 실패: `run_checks.py`(mission created 201), `test_manual_bench_version_comparison`(과거 gap 기록), `test_manual_arbiter_freeze_20261120`(공개 기준선 Arbiter API) — 기존 상태. 환경상 SKIP: 사설 network namespace 요구 5개, 후보 패치·공개 트리 경로 요구 2개.

**의도된 계약 변경 (Codex 판단 요청)**: Codex `test_manual_locked_chain_isolated_20261120.py`의 `MANUAL_CHAIN_FAULT=active_goal`은 외부 goal이 status 배열에서 사라진 뒤 선택기가 *영원히* 재무장하지 않기를 요구하며 이번 변경 후 FAIL한다(기본·`agent_restart` 모드는 PASS). 새 규칙은 같은 GID의 유일 서버가 비종료 goal 없음을 보고하고 Watchdog이 cancel-all로 다시 idle을 증명한 뒤, 운영자가 **새 세션과 Deadman을 다시 입력할 때만** 재개를 허용한다. sticky 차단은 AUTO 임무 1회 후 수동 조작을 Agent 재시작 전까지 막으므로 정상 기능 차단으로 판단했다.

## 6. 미해결·한계

1. **SIGKILL된 프로세스의 DDS 유령 참가자**: Watchdog을 SIGKILL하면 그래프에 이전 `/cmd_vel_raw` 발행자가 약 17–20 s 남아 `VELOCITY_GRAPH_NOT_LOCKED`로 막힌다(실패-폐쇄, 정지는 즉시). 복구 시간 단축은 FastDDS liveliness/lease 설정 검토 대상.
2. Watchdog은 FollowPath·NavigateThroughPoses·Spin·BackUp을 감시하지 않는다. FollowPath는 Agent Arbiter만 막는다.
3. 연속성 증거는 그래프 일관성 검사이며 DDS 인증이 아니다. 같은 노드명으로 위장한 발행자는 SROS2 없이는 구분할 수 없다.
4. 실제 Nav2 스택·Gazebo에서는 아직 실행하지 않았다(Phase 3에서 Nav2 smoother 출력을 Arbiter로 옮길 때 함께 검증).
5. Gate 소스는 현장 번들(`experiments/real_auto_mapping_ready_20261113/...`)이며, `capstone_ws/src`의 미커밋 Gate 변경본은 시험하지 않았다.
6. 실차: real 벤치 0 출력 유지, STM32 timeout·E-Stop·제동 거리 미측정 — 실차 주행 근거 아님.

## 7. Codex 독립 검증용 재현 명령

```bash
cd <repo>/fleet_control
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=228 ROS_LOCALHOST_ONLY=1; unset ROS_DISCOVERY_SERVER
export PYTHONPATH=$PWD/agent/limo_robot_agent:$PWD/server:$PYTHONPATH
export MANUAL_E2E_NODE_PATH=<playwright가 있는 node_modules>   # 브라우저 시나리오용
MANUAL_E2E_OUT=$PWD/../local_data/raw_results/<새 run_id> python3 tests/manual_e2e/run_manual_e2e.py --browser
python3 tests/test_motion_watchdog.py && python3 tests/test_action_status_evidence.py
```

`MANUAL_E2E_ONLY=s09_foreign_nav_goal_stops_then_recovers,...`로 일부만 실행할 수 있다. 출력 폴더는 매번 새 이름이어야 한다(기존 폴더면 시작 시 실패). 권장 추가 검증: 실제 `nav2_bt_navigator`·`controller_server` 기동 상태에서 idle 무발행 재확인, GID 변경 경합(Nav2 재시작 직후 retained active status) 재현, Watchdog 미감시 action 경로.
