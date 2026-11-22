# 미적용 제안 패치 (다른 에이전트 소유 파일)

모두 **적용하지 않았다**. 기준은 2026-11-20 작업트리(다른 에이전트의 미커밋 수정 포함)이며 `git apply --check`로 확인했다. 적용은 Safety Gate·실차 Launch 담당 에이전트가 검토 후 결정한다.

| 패치 | 대상 | 상태 | 요지 |
|---|---|---|---|
| 01 | `limo_depth_obstacles/depth_obstacle_filter_node.py` | 유효 | 경고 키 `marked_points` → `obstacle_points` |
| 02 | 실차 launch | **대체됨 (02b)** | 아래 문제 참조. 보존만 함 |
| 03 | `stall_detector_node.py` | **대체됨 (03b) — 결함** | watchdog 상태를 `'OK'`와 비교한다. watchdog은 `MOTION_AUTHORIZED`를 발행하므로 켜면 **항상 정지**(fail-closed지만 사용 불가) |
| 02b | 실차 launch | 제안 | Nav2 → `/fleet/nav_cmd_vel` → watchdog → `/cmd_vel_raw` → Gate. `use_motion_watchdog`(기본 true), `allow_motor_drive:=true`는 watchdog 필수. watchdog과 함께 쓸 수 없는 직접 goal/속도 소스(`start_explore`, `manual_mapping_probe`)는 `use_motion_watchdog:=false`(잠금 전용)에서만 허용. Gate에 `require_motion_watchdog` 전달 |
| 03b | `stall_detector_node.py` | 제안 | `require_motion_watchdog`(기본 false). 상태가 `MOTION_AUTHORIZED`가 아니거나 0.5 s 넘게 stale이면 0. 비승인 상태 수신 즉시 철회. `/cmd_vel_raw` 발행자 수 ≠ 1이면 0(watchdog 우회 소스 차단). want_motion 계산 **이전**에 적용하므로 센서·TF·Depth·오도메트리·stall 검사와 충돌하지 않음. 진단 JSON에 `motion_watchdog_ok`, `motion_watchdog_state`, `cmd_vel_raw_publishers` 추가 |
| 04 | `stall_detector_node.py` | 제안(별도) | NaN/inf 명령을 0으로. 현재 Gate는 NaN 각속도를 모든 임계 비교를 통과시켜 그대로 출력한다(비교 연산이 False가 되므로) |

02(원본)의 문제:
1. `manual_mapping_probe:=true`일 때 텔레옵이 `/cmd_vel_raw`에 직접 발행하는데 watchdog도 같은 토픽에 0을 20 Hz로 발행 → 중복 발행자.
2. `start_explore:=true`면 explore_lite가 Nav2에 직접 goal을 제출 → watchdog이 LOCKED에서 1 s마다 cancel-all, 개정 watchdog에서는 `FOREIGN_NAV_GOAL`로 정지.
3. watchdog 없이 `allow_motor_drive:=true`를 막는 장치가 없음.
4. `limo_robot_agent` 패키지가 실차 오버레이에 있어야 함(없으면 launch 실패 = fail-closed). 로봇 쪽 빌드 절차는 미정.

적용 순서(권장): 03b+04 → 격리 재시험 → 02b → 잠금 상태 실차 bringup(`allow_motor_drive:=false`) → 그래프 확인 → 실주행은 P0 해소 후.

검증 근거: `fleet_control/docs/SAFETY_WATCHDOG_20261120.md` §6, `local_data/raw_results/safety_watchdog_20261120b/`.

## 적용 조건 (Gate·실차 launch 담당 에이전트용, 2026-11-20)

**02·03(원본)은 적용 금지.** 03은 `'OK'` 비교 결함, 02는 조이스틱 진단·explore_lite와 충돌한다.

적용 전 확인:
1. 대상 파일 sha256이 작성 기준과 같은지(`stall_detector_node.py` `2645bac3…`, 실차 launch `1671d282…`). 다르면 `git apply --check` 후 충돌 부분을 수동 비교한다.
2. 03b와 04는 같은 파일이며 03b → 04 순서로 적용(순차 적용 결과 = 시험한 사본 `local_data/raw_results/safety_watchdog_20261120b/gate_under_test/`).
3. `capstone_ws`를 빌드한 뒤 격리 재시험: `fleet_control/tests/web_mapping_isolated/`에서 `GATE_SCRIPT` 없이(설치본 Gate) `GATE_REQ_WD=1`로 고장 10종을 실행하고 §6.4와 같은 판정이 나오는지 확인한다(`stack.launch.py`는 `GATE_SCRIPT`가 없으면 설치 패키지 Gate를 쓰고 `GATE_REQ_WD`를 그대로 전달한다. 사전조건 9번이 Gate의 `require_motion_watchdog=true` 보고를 확인한다).
4. 02b는 실차 오버레이에 `limo_robot_agent`(fleet_control/agent) 패키지가 있어야 한다. 없으면 launch가 실패한다(fail-closed). 로봇 쪽 빌드·배포는 별도 승인.
5. 02b 적용 후 모터 잠금(`allow_motor_drive:=false`) bringup에서 그래프 확인: `/cmd_vel_raw` 발행자 = `fleet_motion_watchdog` 1개, `/fleet/nav_cmd_vel` 발행자 = velocity_smoother·behavior_server, `/cmd_vel` 발행자 0, Gate 진단 `require_motion_watchdog=true`, Agent 없이 `motion_watchdog_ok=false`.
6. 02b는 `start_explore:=true`, `manual_mapping_probe:=true`를 `use_motion_watchdog:=true`와 함께 거부한다. 기존 잠금 전용 시험은 `use_motion_watchdog:=false`로만 실행된다.

적용 후에도 실차 주행 차단 조건(아래)은 그대로다.

## 실차 주행 차단 조건 (유지)

| 조건 | 상태 |
|---|---|
| STM32 구동 명령 timeout | 미검증 (`limo_driver.cpp`에 timeout 없음) |
| 물리 E-Stop | 미검증 |
| 실제 제동 거리 | 미측정 |
| Depth 기준 높이 (`base_footprint_z`) | 미측정 |
| Wheel odometry 끊김 | 11-18·11-19 반복, 원인 미확정 |
| 실차 Localization 확인 | CONFIRMED 생산자 없음 |
| 운영자 Recovery 경로 | 실모드 없음 |
| Watchdog 동결 시험 | 1개 검사 관측 불가(15/16), 미해결 |
| 서버 단절·Lease 만료 정지 | 11-20 저녁 개선: Lease 만료 0.3–0.4 s, 서버 단절 3.1–3.2 s(격리). 서버 단절 허용 기준(fence 3 s)은 미결정 |

## 2026-11-20 추가: 05 Gate 입력 출처 검증 (미적용)

| 패치 | 대상 | 상태 | 요지 |
|---|---|---|---|
| 05 | `capstone_ws/src/limo_capstone/limo_capstone/stall_detector_node.py` (다른 에이전트의 미커밋 `max_abs_linear_x` 수정 포함 작업트리 기준, `git apply --check` 통과) | 제안 | 후보 Gate r11의 `/cmd_vel_raw` 입력 규칙 이식: 허용 노드 이름 1개·판독 가능한 GID·결속 GID 불변이 아니면 FAILSAFE. 2개 이상·이름 불일치·GID 변경은 operator reset까지 유지. 거부 메시지가 운동을 담으면 0.5 s 동안 무장 금지. NaN/Inf·범위 밖 속도 0. 출력 비유한값 0. 기본(빈 목록)은 현재 동작. 03b의 발행자 수 검사를 대체하는 상세판. 근거·시험: `experiments/safety_chain_fix_20261113/docs/GATE_R11_INPUT_SOURCE_20261120.md` |

05도 인증이 아니다. 같은 노드 이름의 단독 위조 발행자는 수락된다(SROS2 필요).

## 2026-11-21 추가: 06 입력 출처 검증 + Gate가 판단하는 Reset 조건 (미적용)

| 패치 | 대상 | 상태 | 요지 |
|---|---|---|---|
| 06 | `capstone_ws/src/limo_capstone/limo_capstone/stall_detector_node.py` (다른 에이전트의 미커밋 `max_abs_linear_x` 수정 포함 작업트리 기준, `git apply --check` 통과) | 제안 | **05를 포함**하고 후보 r12의 Reset 판단을 더한다: Reset할 장애가 있고, 입력 발행자 1개·허용 이름·판독 가능한 GID·발행 중(0.3 s)·(선택) Watchdog 상태 발행자와 같은 participant, 1 s 동안 운동 명령 없음·출력 0, 센서 장애 없음. 아니면 `REFUSED: …`로 응답하고 래치를 유지한다. 상태 JSON에 `reset_readiness`, `last_reset`, `reset_refusals`를 추가한다. 새 매개변수는 모두 기본 꺼짐(현재 동작 유지): `reset_requires_preconditions=false` |

- 시험(패치 사본 `local_data/raw_results/phase3_stability_claude_20261121/proposal06/`):
  - r12 Reset 판단 단위 14/14 (`GATE_SRC=… test_gate_r12_reset_unit.py`)
  - 입력 출처 단독 10종: 결과는 보고서 참조
- 적용은 Gate 담당자 검토 후에 한다. 적용 순서:
  1. 06 적용
  2. 격리 재시험
  3. 실차 launch에서 `input_exclusive_publishers`, `reset_requires_preconditions`, `reset_input_peer_topic` 설정을 별도 제안
- 06도 인증이 아니다(SROS2 필요). Gazebo·실차·물리 안전 검증은 하지 않았다.
