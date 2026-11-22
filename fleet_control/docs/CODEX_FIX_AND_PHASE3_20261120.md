# Codex 독립 검증 결함 수정 + Phase 3 저장 지도 웹 Navigation (격리 ROS, 2026-11-20, Claude)

입력: `local_data/raw_results/phase12_independent_review_20261120.md`(Codex). 증거: `local_data/raw_results/codex_fix_claude_20261120/`, Phase 3 원본 `local_data/raw_results/nav_phase3_claude_20261120/pre_change_originals/`. 수정 전 해시 `before_fix/sha256.txt`, 시험 코드 해시 `after_fix_sha256_run02.txt`.

## 1. 결과 요약 (같은 코드 스냅샷)

| 시험 | 수정 전 | 수정 후 |
|---|---|---|
| Codex `test_phase12_independent_boundary_20261120.py` (모드 전환·Action 출처, mock 서버/합성 callback) | 2 PASS / 6 FAIL | **8 / 0** |
| Codex `test_phase12_watchdog_gid_independent_20261120.py` (GID 조회 실패, NaN loop age) | 0 / 2 | **2 / 0** |
| Codex `test_phase12_gate_independent_20261120.py` | 3 / 0 | 3 / 0 |
| Phase 1 수동 E2E (격리 ROS, 브라우저) | Codex 재현 18/18 | **18/18** (`phase1_e2e_run02`) |
| Phase 2 모드 E2E (실제 Nav2·SLAM·운동학 sim) | Codex 재현 12 / 4 FAIL(`no_map_loaded`), f06 SKIP | **17/17, SKIP 0** (`mode_e2e_run04`) |
| Phase 3 저장 지도 Navigation E2E (실제 map_server·AMCL·Nav2) | — | **17/17** (`nav_e2e_run02`) |
| 단위·계약 회귀 (Watchdog 22, 모드 11, 증거 7, 예외 guard 50, recovery hint 3, 웹 맵핑 26, Safety Context 계열, 복구 14, 서버–Agent E2E 16, Agent safety 17, adversarial 12+1 SKIP, 패널 Node 3) | — | 전부 PASS (`unit_after/`) |

실행 이력에는 하네스·환경 실패 run(`nav_dev01–17`, `mode_e2e_run01–03`)도 있으며 증거 폴더와 scratch에 보존했다. 최종 수치는 위의 run을 기준으로 한다.

## 2. Codex 결함별 수정

| Codex 결함 | 수정 (파일) | 확인 |
|---|---|---|
| P0 Watchdog: GID 조회 실패에도 승인, 발행자 수 감소로 오인, NaN loop age | `motion_watchdog.py`: 그래프 count와 metadata 수가 같고 모든 GID가 읽혀야 identity OK. 실패나 일부 누락은 `NAV2_SERVER_IDENTITY_UNKNOWN`으로 처리(승인 중이면 FAILSAFE, 관측 폐기). 나쁜 상태에서 회복할 때만 재구독. loop age는 int/float, 유한, 0..timeout. idle 증명에 GID 필수 | Codex 2/2, 신규 단위 3종 |
| P0 모드: CONFIRMING 재검증 없음, Arbiter·Watchdog 누락 시 ACTIVE, boot 변경 시 자동 완료, 미확인 Goal에도 IDLE | `operating_mode.py`: (1) 요청 시 Agent boot·server boot 고정, 변경 시 FAILED→FAULT. (2) 요청 이후 보고만 인정. (3) 능력 계약: locked-arbiter Agent는 Arbiter·Watchdog 보고 필수, Watchdog 없는 Agent는 IDLE로만 전환 가능. (4) CONFIRMING에서 idle·안전 증거 전체를 재검사하고 Arbiter가 목표 모드에서 ready(MANUAL)인지, NO_OWNED_GOAL(MAPPING/NAVIGATION)인지, MODE_IDLE(IDLE)인지 확인. (5) 시간초과·중단 시 FAULT, IDLE은 증명된 경우만 | Codex 8/8, 모드 단위 11 |
| Action 출처: 2개 중 1 GID 누락을 유일로 판단, 이전 writer 지연 메시지 | `action_status_evidence.py`: GID 하나라도 누락되면 IDENTITY_UNKNOWN. 발행자 변경 후 settle 안에 온 메시지는 UNATTRIBUTED. 변경 후에는 Watchdog vouch 또는 창 이후 메시지가 필요(SERVER_CHANGED_UNVOUCHED). 같은 GID 재출현은 같은 writer이므로 연속 유지 | Codex 2/2, 단위 3종. **합성 callback 시험이며 실제 DDS 지연 재현은 하지 않음** |
| f06 하네스가 머신 전체 `pgrep`에 SIGSTOP | `tests/mode_e2e/owned_proc.py`: 하네스가 기록한 프로세스 그룹과 그 자손만, 정확히 1개일 때만 대상. 아니면 신호를 보내지 않음. SIGCONT는 `finally`. Phase 3 하네스도 동일 | f06 PASS (`mode_e2e_run04`) |
| Phase 2 NAVIGATION `no_map_loaded` | 제품 회귀가 아니라 Phase 3 계약(지도 로딩 필수) 추가 때문. 제품 검사는 유지하고 Phase 2 하네스를 계약에 맞춤: n05에서 저장한 지도를 격리 Registry에서만 VERIFIED로 승격 → LOAD → INITIAL_POSE → 실제 AMCL CONFIRMED. 목표는 서버 goal-check가 승인한 점. 기존 fixture(`agent_nav_fixture.py`)는 보존하되 사용하지 않음 | n08/n09/n10/f05/f06 PASS |

## 3. 함께 고친 결함 (시험에서 발견)

- **Phase 2 recovery 미완료**: 로봇 단위 RECONCILIATION_REQUIRED를 웹에서 복구할 수 없었다.
  - 서버(`ownership.py`): 잠금 mock 그래프에서 증거 기반 복구를 추가했다. 필요 조건은 다음과 같다.
    - Agent 저널의 Nav2 action result, 또는 그 goal UUID의 Nav2 status-array terminal
    - Agent 소유 goal 없음, NavigateToPose/FollowPath idle
    - Watchdog 승인, Arbiter 0, Gate NORMAL, Mapping Worker 프로세스 없음
    - admin 승인과 10자 이상 메모, `recovery_refused`/`*_recovered` 감사 이벤트
  - Agent(`core.py`): 서버 hint만으로는 저널을 풀지 않는다. 이 Agent가 직접 본 Nav2 terminal이 같은 UUID에 있을 때만 해제한다.
  - 기존 `recovered_command_ids`가 정확한 문자열만 찾던 결함도 고쳤다.
  - 실차와 unlocked 경로는 계속 거부한다.
- **Watchdog 입력 우회**: Watchdog 입력 `/fleet/nav_cmd_vel`에 rogue 발행자가 있으면 그대로 전달하던 문제. 이제 `VELOCITY_INPUT_NOT_EXCLUSIVE`로 FAILSAFE 처리한다(rogue 출력 0 sample).
- **실제 Nav2와 Safety Context**: idle·실행 중 status가 발행되지 않아 2 s 뒤 임무가 무효화되던 문제. `status_continuous`를 쓰며, 발행자 변경 후에는 Watchdog vouch가 필요하다.
- **Mapping PAUSE/STOP**: 계획·회복 중 취소된 goal에서 FollowPath가 시작되지 않으면 UNKNOWN으로 떨어지던 문제. 현재 FollowPath idle 증거로 보완했다.
- **goal-check가 Fleet lock을 잡아** Agent 상태 게시를 굶기던 문제를 고쳤다.
- **Arbiter**: Gate STALL_PENDING(`cmd_blocked=false`)을 허용하고, 확정 stall은 계속 차단한다.

## 4. Phase 3 — 저장 지도 기반 웹 Navigation (구현)

| 단계 | 구현 |
|---|---|
| 지도 선택·무결성 | `POST /robots/{id}/navigation {action:LOAD, map_version_id, expected_content_sha256?}`(NAVIGATION 모드, Lease). 미등록 404, 버전 불일치 409, quarantine 409, 디스크 무결성 실패 409(`verify_bundle_integrity`). UNVERIFIED는 로딩만 가능하고 임무는 `MAP_UNVERIFIED`로 거부. 자동 승격 없음 |
| Map Server·AMCL | Agent `LocalizationWorker`: 요청된 버전만 `/agent/map-bundle`로 받아 SHA-256 확인 → 소유 프로세스로 `map_server` + `amcl`(프로젝트 `nav2_sim_params.yaml` 값) 실행 → lifecycle bring-up. `/map` 발행자나 SLAM이 있으면 거부. 저장 지도 순찰 중에는 planner `GridBased.allow_unknown=false`, 해제 시 true(맵핑은 true 유지) |
| 초기 위치 | `INITIAL_POSE {x,y,yaw}`: 지도의 FREE 셀만 허용, `/initialpose` 발행, AMCL `request_nomotion_update` |
| Localization 확인 | CONFIRMED 조건: initial pose 이후 AMCL pose, 분산(xy ≤ 0.25 m², yaw ≤ 0.30 rad²), `map→odom`(최신)×`odom→laser`(스캔 시각) 기준 LiDAR 끝점 ≥ 60%가 지도 점유 셀에 연속 5회. 3 s 동안 40% 미만이면 DEGRADED. Safety Context의 map/localization 생산자가 된다(fixture 아님) |
| 목표 검사 | `GET /robots/{id}/goal-check?x&y`와 임무 생성에서 같은 검사: 지도 밖, OCCUPIED, UNKNOWN_AREA, 반경 0.20 m(local `robot_radius` 0.18 + 2 cm) 내 점유·미지. 임무는 해당 요청의 localization CONFIRMED도 요구 |
| 실행·결과 | 기존 `create_mission` → Agent NavigateToPose → Nav2 A\*(`use_astar: true`) → Arbiter(NAVIGATION) → Watchdog → Gate. `/plan` 경로를 상태에 보고. 취소, Nav2 실패(FAILED), 성공 표시 |
| 웹 | `static/navigation_panel.js`: 지도 버전 → 로딩 → 초기 위치(지도 클릭 좌표) → 위치 상태(일치율, AMCL 분산, allow_unknown) → 목표 검사, 단계별 차단 사유. 제출은 기존 Goal Drawer |
| 모드 결합 | NAVIGATION을 벗어나면 지도 요청을 지우고 Agent가 map_server/AMCL을 종료한다. idle 검증에 "localization 프로세스 없음" 포함. 재진입 시 다시 로딩·초기 위치 필요 |

Phase 3 E2E(`nav_e2e_run02`) 확인 항목:
- 웹 맵핑으로 지도 생성(Geofence) → 이번 sim 지도만 격리 Registry에서 VERIFIED(C103은 UNVERIFIED 유지)
- 로딩 전 임무 409 → LOAD → 초기 위치 → CONFIRMED(일치 0.65)
- 지도 밖·벽·반경·미지 영역 거부 → 유효 goal SUCCEEDED(경로 90점) → 취소 CANCELED
- 버전 불일치·미등록 거부, 손상 파일 거부(복원 확인), 잘못된 초기 위치는 CONFIRMED 안 됨 → 재지정
- NAVIGATION→MANUAL→NAVIGATION(재로딩)
- planner SIGSTOP 시 Nav2 FAILED, AMCL SIGKILL 시 위치 FAILED·출력 0·새 임무 거부 → 재로딩
- rogue 우회 시험, Agent SIGKILL 시 출력 0 → 증거 복구 → 재로딩 → 새 goal SUCCEEDED
- 스택 SIGKILL 후 Gate 출력 없음, `/cmd_vel` 0

## 5. 시험 환경과 한계 (검증 수준 구분)

- **격리 ROS, 실제 Nav2/SLAM/AMCL/map_server, 2D 운동학 시뮬레이터**(Gate 출력만 적분, 물리·슬립 없음)다. **Gazebo SKIP**: Arbiter는 mock 잠금 그래프 전용이며 SIM_GATED 연결은 미구현이다. **실차 미실행**.
- 시험 중 기계 부하로 스캔이 0.38 s 넘게 끊기면 Gate가 `FAILSAFE_SENSOR_TIMEOUT`을 래치한다. 하네스는 시나리오 시작 전에 Gate 자체 operator unlatch(`/stall_detector/reset_latch`)를 호출하고 그 사용을 `result.json`의 `gate_unlatch_events`에 기록한다(PASS로 세지 않음). Phase 2 run04 0회, Phase 3 run02 0회.
- 격리 Nav2 lifecycle 기동 실패는 하네스가 스택 재기동으로 처리한다(`nav2_bringup_attempts`, run02에서 1회 시도로 성공).
- 지도 VERIFIED 승격은 HTTP에 노출되지 않는다(정책 미정). 시험은 Registry API로 격리 데이터만 승격했다. 웹에서 지도를 검증할 방법은 없다.

## 6. 미해결 안전 차단 조건 (실차 No-Go 유지)

1. **Gate 입력 독점 없음**: `/cmd_vel_raw`에 rogue 발행자가 있으면 Gate 출력이 나왔다(f07 28 sample). Arbiter는 `VELOCITY_GRAPH_NOT_LOCKED`로 감지하지만 Gate가 막지 않는다. Gate 담당 수정 또는 SROS2가 필요하다.
2. 외부 FollowPath/Spin/BackUp/NavigateThroughPoses goal은 Watchdog이 감시하지 않는다. 발행자 변경 후 FollowPath는 NavigateToPose vouch에 기댄다.
3. Humble rclpy는 메시지별 발행자 GID를 주지 않는다. 출처 결박은 시간 창 기반이며 인증이 아니다.
4. Mock 예외(locked-mock continuation, recovery, movement 허용)는 Agent가 보고하는 문자열과 `motor_path=LOCKED`를 신뢰한다. 하드웨어 attestation은 없다.
5. 실제 지도 검증 정책·권한, 실차 localization 품질 기준(일치율 60%는 sim 값), STM32 timeout·E-Stop·제동 거리, 전원/USB, Watchdog 실차 launch 미편입.

## 7. 다음 단계 선행 조건

- 자동 기동: Agent·Watchdog·Gate·Nav2 기동 순서와 lifecycle 재시도(이번 하네스 패턴), Gate unlatch의 웹 경로·권한 정책, READY 판정에 localization과 identity 포함.
- 방문객 웹: VERIFIED 지도와 목적지(waypoint) 등록 정책, NAVIGATION 모드와 운영자 승인 흐름이 필요하다.

## 8. Codex 독립 재검증용 재현

```bash
cd <repo>; source /opt/ros/humble/setup.bash; source capstone_ws/install/setup.bash
W=$(mktemp -d); colcon --log-base $W/log build --base-paths capstone_ws/src/m-explore-ros2 --packages-select explore_lite --build-base $W/build --install-base $W/install
cd fleet_control; export OVERLAY=$W/install PYTHONPATH=$PWD/agent/limo_robot_agent:$PWD/server:$PYTHONPATH
NAV_E2E_OUT=$PWD/../local_data/raw_results/<새 id> python3 tests/nav_e2e/run_nav_e2e.py <빈 포트> <domain ≤232>
MODE_E2E_OUT=$PWD/../local_data/raw_results/<새 id> python3 tests/mode_e2e/run_mode_e2e.py <빈 포트> <domain>
cd tests; for t in test_phase12_independent_boundary_20261120 test_phase12_watchdog_gid_independent_20261120 test_motion_watchdog test_action_status_evidence test_operating_mode test_phase3_exception_guards test_agent_recovery_hint; do PYTHONPATH=$PWD/../agent/limo_robot_agent:$PWD/../server:$PWD:$PYTHONPATH python3 $t.py; done
```

권장 재검증 항목:
- 실제 DDS 지연·발행자 교체 경합(합성으로만 시험함)
- CONFIRMING과 Agent 재시작 경합
- recovery 증거의 위조 가능성
- Gate 입력 독점
- localization 일치율 기준의 실환경 타당성

## 9. 변경 파일

- 서버: `operating_mode.py`, `navigation.py`(신규), `service.py`, `ownership.py`, `safety.py`, `mapping.py`, `http_api.py`
- Agent: `motion_watchdog.py`, `action_status_evidence.py`, `localization_worker.py`(신규), `ros_adapter.py`, `core.py`, `agent_node.py`, `velocity_arbiter.py`, `mapping_worker.py`, `safety_protocol.py`
- UI: `static/navigation_panel.js`(신규), `app.js`, `index.html`
- 시험(신규): `tests/nav_e2e/run_nav_e2e.py`, `tests/mode_e2e/owned_proc.py`, `tests/test_agent_recovery_hint.py`, `tests/test_phase3_exception_guards.py`, `tests/test_navigation_panel.cjs`
- 시험(수정): `tests/mode_e2e/{run_mode_e2e.py, stack_up.sh, kinematic_sim.py}`, `tests/web_mapping_isolated/stack.launch.py`(env 보호)
- 기대값 변경(원본은 `before_fix/`, `superseded_test_originals/`에 보존):
  - `test_operating_mode.py`: 미증명 idle 시간초과를 IDLE이 아닌 FAULT로
  - `test_motion_watchdog.py`, `test_manual_watchdog_predicate_fix_20261120.py`: GID metadata 반영
  - Codex 가짜 DB: `one()` 추가
- commit/push 없음.
