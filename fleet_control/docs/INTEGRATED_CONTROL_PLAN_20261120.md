# 웹 기반 LIMO 통합 관제 — 목표 아키텍처와 재사용 계획 (2026-11-20, Claude)

> 상태: **설계 기준 문서(제안)**. DECISIONS.md 확정 결정이 아니다. Phase 1 구현·검증 결과는 [MANUAL_PHASE1_INTEGRATION_20261120.md](MANUAL_PHASE1_INTEGRATION_20261120.md).
> 역할: Claude Code = 주 설계·구현, Codex = 독립 리뷰·회귀·안전 검증(사용자 지시 2026-11-20).

## 1. 원칙

1. 기존 구성요소를 연결한다: Fleet Server(`server/fleet_server`), Robot Agent(`agent/limo_robot_agent`), 웹 대시보드(`static/`), Motion Watchdog, Safety Gate(`stall_detector_node`), Mapping Worker·Map Save·Map Registry, Nav2(A\* `GridBased use_astar: true`). 새 서버·새 UI·새 패키지를 만들지 않는다.
2. **단일 속도 경로**: 어떤 모드든 모터 쪽으로 가는 속도는 `VelocityArbiter → /fleet/nav_cmd_vel → Motion Watchdog → /cmd_vel_raw → Safety Gate → /cmd_vel(실차) | /cmd_vel_locked(격리)` 하나만 지난다.
3. **배타적 운용 모드**(MANUAL / MAPPING / AUTO): 서버 admission이 1차, Agent 선택기가 2차, Watchdog·Gate가 3차 방어다. 각 층은 자기 증거로 독립 판정한다.
4. 안전 증거는 *실제 ROS 동작 방식*에 맞춘다. 정상 기능을 영구 차단하는 조건(예: 주기 발행되지 않는 action status의 나이 검사)은 결함으로 취급하고 고친다.
5. 실차 모터 잠금, 실차 Mapping START 차단, 18190 운영 서버, LIMO 접근은 사용자 별도 승인 전 변경하지 않는다.

## 2. 목표 데이터 흐름

```text
운영자 웹(static/index.html + app.js + manual_control_panel.js, mapping_panel.js, map_save_panel.js, goal_check.js)
   │ HTTPS/HTTP (Bearer, 역할: viewer/operator/admin)          방문객 웹(Phase 5, 별도 경로·권한 없음)
   ▼                                                              │ 요청만 생성(운영자 승인 필요)
Fleet Server  ── Lease(owner/epoch/TTL) · control_mode · Mission · Mapping Session · manual-bench 세션(250 ms TTL)
   │  Agent pull(상태 POST 응답으로 명령, /agent/manual-bench 20 Hz poll)
   ▼
Robot Agent (RosRobot + ManualWebBench + VelocityArbiter + MappingWorker/MapSaveWorker)
   │  /fleet/agent_heartbeat (goal_uuids, tasks_age_s)        NavigateToPose ActionClient (AUTO, Agent 소유 goal)
   ▼
VelocityArbiter ──/fleet/nav_cmd_vel──▶ Motion Watchdog ──/cmd_vel_raw──▶ Safety Gate ──▶ /cmd_vel (실차: 현재 잠금)
   ▲ MANUAL 샘플(서버 세션)         ▲ 독립 감시: heartbeat, Gate status, NavigateToPose 서버 유일성·idle 증명, 외부 goal
   └ (Phase 3) Nav2 smoother 출력을 /fleet/cmd_vel_nav로 remap해 AUTO일 때만 통과
```

현재 Phase 1 격리 구성에서는 Nav2 속도 출력이 그래프에 없고(fake action server만 존재) Arbiter가 `/fleet/nav_cmd_vel`의 유일한 발행자다. Phase 3에서 Nav2 smoother 출력을 Arbiter 입력으로 옮겨 AUTO 통과 경로를 추가한다(아래 §4).

## 3. 모드 배타성 계약

| 진입 요청 | 서버가 거부하는 조건 (구현 위치) | Agent/하위 층 독립 방어 |
|---|---|---|
| MANUAL 세션 start/sample | Lease owner·epoch 불일치, Agent offline/미조정/상태 2 s 초과, 미종료 Fleet 임무(`robot_busy`), Mapping 비종료·UNKNOWN, Nav2 idle 증거 없음, 잠금 Arbiter 미준비(+차단 사유) (`service.py::_manual_bench_robot`) | Arbiter: Fleet 연결·Lease·Agent active goal·Nav2/FollowPath idle·Watchdog 승인·Gate NORMAL·odom·속도 그래프. 실패 시 0. 세션 TTL 250 ms |
| AUTO 임무 생성 | **MANUAL 세션 활성(`manual_active`)** (이번 추가), Mapping 비종료, safety context·Nav2 idle·Lease (`service.py::create_mission`) | Agent `allow_missions`, Watchdog 승인, Gate |
| MAPPING START | MANUAL 세션 활성, Fleet 임무 미종료, 외부 goal, SLAM 세션 (`mapping.py`) | Mapping Worker의 Watchdog 요구, 실차 Mapping 차단 유지 |
| 화면 표시 | `robot_view.control_mode` = MAPPING > AUTO > MANUAL > IDLE (UNKNOWN은 실패-폐쇄) | — |

모드 전환 시 이전 출력 중단: MANUAL→다른 모드는 STOP 즉시 세션 삭제 + Agent 샘플 만료(≤250 ms) + Arbiter 0. AUTO→MANUAL은 임무 terminal과 Nav2 idle 증거가 필요하다. MAPPING→MANUAL은 Mapping Session STOPPED가 필요하다.

## 4. Phase별 재사용 계획과 남은 연결

| Phase | 재사용 | 이번에 확인한 빈 곳 / 다음 변경 |
|---|---|---|
| 1 수동 | Codex `manual_web_bench.py`(poll), `velocity_arbiter.py`(잠금 mock 선택기), `motion_watchdog.py`, 서버 manual-bench 세션·Lease 무효화, `manual_control_panel.js`(키보드·터치·Gamepad) | **완료(격리)**: 실제 Nav2 status 의미론 대응, 차단 사유, 모드 배타성, 브라우저 E2E. 남음: 실차 real 벤치는 여전히 0 출력 전용(의도), 실제 장치 Gamepad, Watchdog의 FollowPath 미감시 |
| 2 맵핑 | `mapping_worker.py`, `mapping.py`, `map_save.py`/`map_save_worker.py`, `map_registry.py`, `mapping_panel.js`, `map_save_panel.js`, Gazebo r8 하네스 | Mapping 중 속도 경로를 Arbiter로 통일(현재 Frontier→Nav2 직접), CONTINUE_MAP posegraph 적재, posegraph 3.8 MB 업로드 한계, 실차 START 차단 유지 |
| 3 저장 지도 주행 | Registry 지도 선택, `goal_check.js` 사전 검사, `create_mission`, Agent NavigateToPose, Nav2 A\* params | Nav2 smoother 출력→`/fleet/cmd_vel_nav`→Arbiter AUTO 통과, map_server 로딩·initialpose·AMCL **CONFIRMED 생산자(현재 없음)**, 미지/점유/지도 밖 goal 거부를 서버에서 강제, C103·H107 UNVERIFIED 지도는 주행 금지 유지 |
| 4 자동 기동·진단 | `start_locked_standby.py`, `start_robot_agent_field.sh`, `dds_ready_check.py`, `stop_field_session.sh`, Agent 상태 보고 | systemd user unit(잠금 기본) 설계, 데이터 신선도 기반 READY, DDS participant ID 23 문제는 **미해결로 유지**, 노드 사망·재시작 실패 표시 |
| 5 방문객 웹 | Fleet mission API, Registry 목적지 데이터 | 별도 정적 경로·별도 역할(visitor), 요청→운영자 승인→임무, 카메라·진단·원시 API 비노출, Mock 검증 후 공개 배포는 별도 승인 |

## 5. 실차 차단 조건(변경 없음)

모터 잠금, 실차 Mapping START 차단, STM32 `/cmd_vel` timeout·물리 E-Stop·제동 거리 미측정, 전원/USB 재열거, wheel odom 중단 재발, localization CONFIRMED 생산자 부재, Watchdog 실차 launch 미편입(02b/03b/04 제안 미적용). 이 문서의 어떤 항목도 실차 주행 허가가 아니다.
