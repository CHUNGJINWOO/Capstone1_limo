# 수동 조작 부분 구현 및 검증 (2026-11-20)

## 실제 변경

- `server/fleet_server/service.py`: readiness 실패·offline·hello·lease 만료/세대 변경 및 pull 만료 시 벤치 세션 폐기. 부트 변경 후 기존 벤치 세션 폐기. Mapping 진행/UNKNOWN 및 NavigateToPose status 미수신·stale·active·publisher 수·잘못된 수를 거부한다. `age_s` + 로봇 상태 전송 경과 시간으로 2초 상한을 적용한다. 이는 영속도 벤치 보호이며 action terminal 증명이나 움직임 허가가 아니다.
- `server/fleet_server/ownership.py`: 새 lease 획득 또는 정상/강제 해제시 기존 세션을 버린다. 타인의 무단 해제 거부는 보존한다.
- `agent/limo_robot_agent/limo_robot_agent/motion_watchdog.py`: NavigateToPose status가 없는 경우 idle proof 금지, 기본 2초의 상태 신선도와 publisher 수 검사. 상태 미수신/오래됨은 `NAV2_STATUS_STALE_OR_UNKNOWN`으로 차단한다. graph 불명·중복 관측은 이전 상태를 버린다. Nav2의 상태가 변경시에만 발행된다면 정상 유휴도 stale로 잠긴다. 신뢰 가능한 지속 관측 생산자가 준비되기 전 이 제한을 완화하면 안 된다. FollowPath 감시·Goal 결과 인증·외부 제출 fencing은 아직 없다.
- `tests/test_manual_control_bench_contract.py`, `tests/test_manual_session_adversarial_20261120.py`, `tests/test_manual_watchdog_missing_status_20261120.py`: 변경된 벤치 계약에 맞게 fixture와 실패 기대치를 조정했다. 이전 실패 로그는 그대로 보존했다.
- `tests/test_manual_bench_safety_fix_20261120.py`, `tests/test_manual_watchdog_predicate_fix_20261120.py`, `tests/test_manual_inputs_pure_20261120.cjs`: 실제 코드에 대한 독립 함수/모의 DB 검증.

공통 웹 패널과 실차 launch는 변경하지 않았다. 신뢰할 만한 외부 Goal 종료/배타 증거가 없어 UI→Fleet HTTP→Agent→Arbiter→Watchdog→Gate를 연결할 수 없었고, staging에는 공개 기준선의 Arbiter가 없다. ticket·session/양쪽 boot의 종단 검증과 AUTO/MANUAL/MAPPING의 실제 원자적 전이도 미구현이다. LIMO, 운영 18190, 실차 모터를 사용하지 않았다.

## 시험과 로그

- 서버 안전 회귀: [기존 13개 중 12 PASS/1 SKIP](../../local_data/raw_results/manual_session_adversarial_staging_fixed_20261120.log), SKIP은 staging ticket 미구현. 과거 7개 실패의 **벤치 서비스 메서드 경계**는 새 모의 조건에서 통과했지만 외부 ROS·FollowPath·실제 모드 전이는 검증하지 않았다.
- 별도 [벤치 6 PASS](../../local_data/raw_results/manual_bench_safety_fix_v2_20261120.log), [기존 벤치 5 PASS](../../local_data/raw_results/manual_bench_existing_regression_20261120.log), [Watchdog predicate 2 PASS](../../local_data/raw_results/manual_watchdog_predicate_fix_v2_20261120.log), [키보드/가상 조이스틱/Gamepad 변환 함수 PASS](../../local_data/raw_results/manual_inputs_pure_20261120.log). 브라우저 이벤트, 실제 장치 및 입력 전체 경로는 시험하지 않았다.
- [Mapping 기존 시험](../../local_data/raw_results/manual_mapping_regression_20261120.log): 19 PASS/7 SKIP. 별도 기존 Lease 시험 2 PASS. Agent 패키지 [colcon build PASS](../../local_data/raw_results/manual_agent_build_retry_20261120.log) (byte-compiling 비활성 경고).
- loopback TCP·UDP 바인딩 모두 `Operation not permitted`. 실제 Fleet HTTP·Agent·Arbiter·독립 Watchdog·Gate ROS E2E 및 Gazebo는 **SKIP**. Gazebo 실행 파일도 발견하지 못했다. 과거 테스트 산출물을 이번 PASS로 사용하지 않는다.

공개 기준선 FAIL 2는 해당 코드 미수정으로 유지된다. Watchdog의 미수신 결함은 predicate 수준에서 수정했지만 실제 ROS 상태 전이 시험은 SKIP. FollowPath/외부 goal 결과·원자적 제어권·Agent 재생 차단·독립 watchdog process 장애·게이트 데이터 중단·모터 출력 검증은 미완료다. 안정적 목표 미존재를 판별할 신뢰 기준과 실행 중 외부 목표 제출 차단을 먼저 구현·시험해야 정상 수동 구동 허가를 연결할 수 있다. 전체 사용자 목표는 완료되지 않았으며 실차 차단을 유지한다.
