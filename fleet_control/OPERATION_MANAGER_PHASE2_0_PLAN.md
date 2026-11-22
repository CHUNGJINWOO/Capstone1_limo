# Phase 2.0 — 기존 Robot Agent 기반 Goal 경계 최소 통합 계획

**상태: 승인 대기인 계획. 운영 코드·실차 설정은 수정하지 않았다.** 최초 구현·검증 대상은 격리 ROS 2 Humble의 모의 ActionServer이며, 실제 Nav2·Gazebo·LIMO 및 Gate 연동은 각각 별도 승인과 안전 조건이 필요하다. 기존 Fleet Server를 단일 제어 평면으로 유지하고 새 Robot Agent를 만들지 않는다. 실제 Gate Attestation이 없는 동안 실차 이동 허가는 계속 금지한다.

## 1. 현재 코드와 통합 경계

- 기존 [Agent `ros_adapter.py`](agent/limo_robot_agent/limo_robot_agent/ros_adapter.py)는 ActionClient, GoalHandle/UUID별 취소, 제출 직전 Stage 3 검사, 외부 Goal의 관측 기반 식별을 제공한다. [Agent `core.py`](agent/limo_robot_agent/limo_robot_agent/core.py)는 중복 방지 Journal, 부팅 ID 및 미종료 상태의 reconciliation을 제공한다. operation·map·localization 생산자는 현재 기본 UNKNOWN이다.
- 기존 [Fleet Server `service.py`](server/fleet_server/service.py), [`safety.py`](server/fleet_server/safety.py), [`ownership.py`](server/fleet_server/ownership.py), [`recovery.py`](server/fleet_server/recovery.py)의 임무·lease·Safety Context·재조정을 재사용한다. lease는 API 요청자의 제어권이지 동일 SROS 인증서를 가진 두 프로세스의 실행권이 아니다.
- 현재 [`ownership.py`](server/fleet_server/ownership.py)의 `recovery_evidence()`는 신선한 빈 Nav2 status와 Agent active 부재를 운영자 복구 근거로 사용한다. 사용자 요구인 **빈 status만으로 Goal 종료 확정 금지**보다 약하다. 추가 종료·소유권·관측 범위 증거 없이는 미해결로 남도록 강화해야 하며, 기존 복구 API의 409 증가와 호환성 변화가 발생한다. 원래 목표 결과가 불명이면 임의 SUCCEEDED/CANCELED 처리하지 않는다.
- WEB 및 RMF는 Fleet 임무 경로를 재사용한다. Explore는 현재 [explore.cpp](../capstone_ws/src/m-explore-ros2/explore/src/explore.cpp)에서 직접 Goal과 `cancel-all`, 조건부 복귀 Goal을 보낸다. **Explore 운영 소스 변경은 담당 범위 별도 승인 후**다. 그 전에는 Explore를 포함한 단일 경계를 완성됐다고 보고하지 않는다.
- 검토한 실차 통합 [launch](../capstone_ws/src/limo_capstone/launch/limo_integrated_auto_mapping_real.launch.py)는 `start_explore=false` 기본값이며 Nav2 → `/cmd_vel_raw` → Gate → `/cmd_vel` 또는 `/cmd_vel_locked` 경로를 구성한다. 이는 소스 기준이며 실제 배포된 런처·remap·다른 Action 및 직접 속도 발행자는 실차에서 아직 확인하지 않았다. 현재 [motor_policy.py](agent/limo_robot_agent/limo_robot_agent/motor_policy.py)의 실차 조건은 `/cmd_vel` 발행자가 있으면 차단하므로, 안전 정책 조정 없이 정상 Gate 구동 경로가 허용된다고 가정하지 않는다. 실차 해제나 예외 추가는 이번 승인 대상이 아니다.
- Phase 1.3~1.8의 상세 근거는 [1.3](../experiments/phase13_ros_action_20261115_K8m4/RESULTS.md), [1.4](../experiments/phase14_goal_boundary_20261115_q4H7/RESULTS.md), [1.5](../experiments/phase15_sros2_20261115_F8v2/RESULTS.md), [1.6](../experiments/phase16_goal_arbiter_20261115_D2k8/RESULTS.md), [1.7](../experiments/phase17_goal_sros_20261115_M6w3/RESULTS.md), [1.8](../experiments/phase18_fencing_20261115_G7r2/RESULTS.md)과 [1.9 감사](OPERATION_MANAGER_PHASE1_9_AUDIT.md)에 있다.

## 2. 승인 후 변경 후보 — 실제 파일 단위

| 파일·담당 | 최소 변경 후보 | 적용 조건 |
|---|---|---|
| `fleet_control/agent/limo_robot_agent/limo_robot_agent/ros_adapter.py` — Agent | 기존 `self.active`, lock, Stage 3 및 GoalHandle을 확장해 로봇별 하나의 예약/제출 경계. 요청 owner·operation·command를 붙이고 예약→전송 결과 불명→수락→취소 요청/ACK→terminal을 분리. 다른 UUID·이전 실행 세대 callback이 현행 상태를 덮지 못하도록 검사 | 같은 프로세스 lock은 다른 프로세스와 실제 Nav2 접근을 막지 못함. lock 전후 장시간 ROS 대기 금지; 외부 Goal·상태 미관측 시 차단 |
| `fleet_control/agent/limo_robot_agent/limo_robot_agent/core.py` — Agent | 기존 Journal에 소유자·실행 세대·Goal UUID·제출/수락/terminal 증거·불명 상태를 버전 관리하여 보존. 원자적 쓰기 실패·손상·재시작을 fail-closed로 처리; 같은 명령 본문·주체·세대가 다르면 충돌 | 기존 Journal의 비파괴 마이그레이션·rollback 위험 및 디스크 동기화 범위를 별도 시험. 기존 완료 기록 보존 |
| `fleet_control/agent/limo_robot_agent/limo_robot_agent/safety_protocol.py` — Agent | 기존 Safety Context의 Agent/Server boot, lease, Gate·operation·map·localization 세대와 실행 소유권·Attestation 상태를 제출 및 재검증 경계에서 점검 | Gate의 인증된 실제 생산자·서명·challenge 계약 확정 전 정상값 생성 금지. 합성 fixture는 격리 시험 전용 |
| `fleet_control/agent/limo_robot_agent/limo_robot_agent/agent_node.py` — Agent/배포 | 기존 진입점에서 로봇별 고정 실행 저장소, supervisor와 소유권의 획득/상실 및 보안 설정 오류를 fail-closed 처리하도록 연동 | supervisor/OS 사용자 설치·변경은 별도 배포 승인. `allow_missions=false` 기본 유지 |
| `fleet_control/server/fleet_server/ownership.py`, `recovery.py` — Server | 현재 빈 status 기반 복구를 강화하고, 소유자·UUID·terminal/ActionServer 범위·관측 신뢰성이 부족하면 차단 유지. 기존 recovery·audit 경로 재사용 | 상태 근거가 없는 기존 Agent는 복구 409. 과거 정상 동작 기대 테스트와 API 호환성 문서화 |
| `fleet_control/server/fleet_server/safety.py`, `service.py`, 필요 시 `db.py`, `http_api.py` — Server | 기존 lease 및 불변 Safety Context에 새 실행 소유권·operation 정보가 필요할 때만 최소 추가; 생성·배차·전달·재시작에서 검증과 기존 미전송 무효화 유지 | 저장·HTTP 전달과 ROS 실제 Goal 실행은 하나의 트랜잭션이 아님. DB 마이그레이션·구 Agent fail-closed 및 전체 회귀 필수 |
| `fleet_control/rmf_adapter/limo_rmf_adapter/{runner.py,mission_client.py}` — RMF | 기존 Fleet 임무 API와 idempotency 경로 우선 유지. owner/operation 계약상 실제 필요한 필드가 있을 때만 수정 | RMF가 Nav2 직접 client를 새로 만들지 않음; 기존 cancel·recovery 회귀 |
| `capstone_ws/src/m-explore-ros2/explore/src/explore.cpp` 및 연관 interface/launch — Explore 담당 | frontier 후보 제출을 기존 Agent 요청 인터페이스로 전환, stop 후 새 후보 차단, `cancel-all` 제거, UUID 소유권은 Agent가 관리 | **별도 담당·범위 승인 전 변경 금지.** 복귀는 기본 비활성·별도 승인 임무. 인터페이스 위치와 기존 C++ 빌드 영향은 설계 확정 후 결정 |
| 배포 정책·Supervisor — 배포 담당 | 아직 저장소에 운영용 확정 파일 없음. 승인 후 `fleet_control` 기존 배포 구조를 확인하고 서비스 단일 인스턴스, 전용 UID, 고정 상태 디렉터리, SROS2 Enforce와 최소 권한을 설계 | 이 내용은 제안이며 신규 디렉터리·시스템 사용자·서비스 생성은 별도 승인 후 |
| `fleet_control/tests/test_agent_safety_v1.py`, `test_server_agent_safety_e2e.py`, `test_ros2_isolated_action.py`, `test_ros2_late_goal_reconciliation.py`, `test_r3_r4_contract.py`, `test_db_transaction.py` 및 기존 ownership/recovery/dedupe/HTTP 시험 — 시험 담당 | 기존 사례를 보존하고 새 계약 fixture와 격리 ROS·SROS 부정 시험을 추가. 필요한 전용 시험 파일명은 작업 시작 시 충돌 확인 | 기존 Agent 호환성과 안전 정책에 따른 의도적 거절을 분류; 시험 통과를 위한 예외 금지 |

위 표의 모든 항목은 구현 후보이며 이번 변경 사항이 아니다. Gate v4.2, 기존 시뮬레이터와 실차 설정은 변경 대상이 아니다.

## 3. 제안하는 최소 계약

로봇별로 owner_type(WEB/RMF/EXPLORE), 인증된 owner_id, operation_id, command_id, Agent boot, 실행 소유 세대, Gate boot/fault, operation_revision, map/localization epoch를 예약 및 불변 command context와 연결한다. UUID는 실제 Goal 응답 후 저장한다. 전송 호출, 수락, cancel ACK, terminal은 별도 사건이다. 중복 키는 요청자·경로·본문·세대를 대조하고 같은 요청에는 기존 결과만 반환한다.

시작 순서는 UNKNOWN → 실행권 확보 → 서버 lease/context 및 Gate 증거 확인 → 과거 Goal 재조정 → 신규 요청 명시적 승인이다. 실행권 lock, 시간 경과, 빈 status, cancel ACK, Gate reset은 이전 Goal의 terminal이 아니다. 미지의 외부 Goal, Journal 손상, server/Agent/Gate boot 변경, lease·지도·위치추정 세대 변경, stale status, 결과 유실 시 이동을 거절하고 필요하면 RECONCILIATION_REQUIRED를 유지한다. 이미 제출된 목표는 해당 UUID의 handle로만 취소하고 결과가 불명확하면 자동 재개하지 않는다.

단일 Supervisor·전용 UID·0600 상당 키와 상위 디렉터리 권한·SROS2 Enforce는 운영 후보이다. 정책은 Agent enclave만 대상 Action을 허용하고 WEB/RMF/Explore의 직접 Goal·cancel-all·`/cmd_vel` 및 다른 이동 경로를 제한한다. Gate/TF/status 등 필수 통신은 개별 허용하고 회귀한다. 같은 UID의 키 복제 또는 다른 저장소의 비협조 프로세스는 단순 `flock`으로 막지 못한다.

Fencing 세대를 DB에 저장하는 것만으로 실제 Goal 실행이 강제되지 않고, 기존 Nav2는 token을 검증하지 않는다. Agent의 전송 직전 검사 외에 직접 Nav2 접근을 막는 OS/SROS2 강제 경로와 동일 자격정보를 가진 중복 프로세스 대책을 실증해야 한다. DB와 함께 롤백되는 값만으로 이전 상태를 식별할 수 없으므로 독립적인 신뢰 원천이 없으면 복원 후 자동 이동을 금지한다. 실행 경계 proxy의 필요성은 미결정이다.

## 4. 단계별 구현 순서

1. **계약과 호환성**: Agent·Server·Gate 필드와 terminal 증거를 대조하고 기존 회귀·실패 기록을 보존한다. 빈 status 복구의 API 동작 변경을 명시한다.
2. **Agent·Server 격리 구현**: 기존 예약·Journal·재조정과 서버 복구 증거를 최소 확장한다. WEB/RMF API를 회귀하고 Explore 운영 연동은 연결하지 않는다. Goal 허가는 테스트 fixture에만 사용한다.
3. **실행·인증 격리 시험**: 시험 전용 Supervisor와 자격정보로 실제 Agent와 모의 ActionServer를 SROS2 Enforce에 연결한다. 독립 network namespace·공유 메모리·ROS domain·임시 HTTP 포트 격리를 검증할 수 없으면 SKIP한다.
4. **Explore 범위 별도 승인**: 후보 생성·요청·중지·terminal·복귀 계약을 연결하고 C++ 빌드와 격리 시험을 수행한다. Gate 코드와 기존 실차 설정은 수정하지 않는다.
5. **별도 승인 후 단계**: 실제 Gate/Attestation과 Nav2의 비구동 통합, Gazebo, 실차는 각각 별도 안전 조건과 승인이 충족될 때 진행한다.

## 5. 테스트별 합격 기준

| 시험 | PASS 조건 |
|---|---|
| 동시 Goal 예약 | 같은 robot의 WEB·RMF·Explore 경쟁에서 모의 서버가 실제 수신한 Goal은 최대 1개이며 거절된 요청의 Goal은 생성되지 않음 |
| 중복 Agent | 같은 Journal과 다른 Journal, 동일 인증정보의 비협조 client 모두에서 실제 ActionServer에 두 번째 Goal이 전달되지 않음. 협조 `flock`만으로 전체 PASS 금지 |
| SIGKILL·복원 | 예약·전송·늦은 수락·cancel ACK·terminal 전후 별도 프로세스 종료. 증거 불명이면 reconciliation, 자동 재전송 0건 |
| 미인가 Goal·cancel-all | 같은 domain의 미인가 client가 Goal·취소를 전송하지 못하고 기존 Goal이 유지됨. 모든 대상 Action을 열거 |
| DB rollback | Server DB·Agent Journal의 각자 및 동시 snapshot 복원에서 이전 권한이 승인되지 않음. 독립 신뢰 원천이 없으면 이동 BLOCKED |
| Attestation | 부재·만료·서명/발행자 오류·challenge 재사용·Gate/operation/map/localization 세대 불일치에서 Goal 전송 0건. 실 Gate 미구현 시 실제 인증은 SKIP, 실차 이동 차단 |
| 미관측 Goal | 외부 active, status 유실·stale, ActionServer/GID 변경 시 idle을 추정하지 않고 새 Goal 0건 |
| 이전 세대의 늦은 응답 | 이전 Agent boot/실행 세대 또는 다른 UUID callback이 현재 상태를 변경하지 않음. 이전 Goal이 남을 수 있으면 Gate 차단과 reconciliation 유지 |
| 정상 통신·회귀 | 실제 Agent의 정규 Goal·feedback·status·개별 cancel·terminal을 확인하고 R3/R4, Safety Context, lease, recovery, dedupe, HTTP·RMF 회귀를 PASS/FAIL/ERROR/SKIP으로 구분 |

모의 ActionServer 결과는 실제 Nav2 preemption이나 Gate의 물리 정지를 증명하지 않는다.

## 6. Gate 의존성과 승인 사항

Claude 담당 Gate v4.2는 구조화 Attestation, status writer GID 연속성 및 다중 ActionServer 감시가 미완성으로 보고됐다. Gate boot/fault 세대, challenge·서명·키·유효기간·재사용 방지, ActionServer와 UUID별 terminal 증거 및 초기 status 부재 시 idle 증명을 공동으로 확정해야 한다. Gate의 독립 모터 차단과 Agent의 논리적 취소는 별개다. 현재 Attestation 부재로 실차 이동을 허가하지 않는다.

구현 착수 전 승인 범위: (1) 위 Agent·Server 및 테스트 변경과 빈 status 복구 API의 호환성 변경, (2) 직접 Goal 강제 차단 방식과 DB 롤백 후 복구 정책, (3) 시험 전용 Supervisor·SROS2 구성, (4) Explore 코드·인터페이스 담당 범위, (5) Gate 계약 미제공 시 fail-closed. 실제 OS 사용자·서비스 설치와 실제 Gate·Nav2·Gazebo·LIMO 실행은 각각 별도 승인 대상이다.

## 7. 공통 문서 변경 제안 — 미적용

`CODE_INDEX.md`에 이 계획을 색인하고 `CURRENT_STATUS.md`·`HANDOFF.md`에 승인 대기와 실차 이동 차단을, `VALIDATION.md`에 이번 작업의 신규 시험 없음을 기재하는 안이다. 병행 편집 보호를 위해 공통 문서와 `DECISIONS.md`, Gate 작업 문서는 변경하지 않았다.
