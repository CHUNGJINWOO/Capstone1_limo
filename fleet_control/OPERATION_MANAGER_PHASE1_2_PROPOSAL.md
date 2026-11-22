# Explore 목표 제어와 Gate Attestation — Phase 1.2 설계안

읽기 전용 조사와 독립 Mock 결과다. Agent·Explore·Gate·Fleet Server의 통합 구현이나 실제 Nav2/Gazebo/LIMO 시험이 아니다. Fleet Server는 단일 제어 평면으로 유지한다.

## 실제 흐름과 권한 경계

`explore.cpp`의 `makePlan()`은 새 frontier가 나오면 기존 goal의 terminal을 기다리지 않고 NavigateToPose에 직접 `async_send_goal()`을 호출할 수 있다. 수락 callback은 UUID를 `active_goal_id_`에 저장하지만 GoalHandle을 외부에 제공하지 않는다. 결과 callback은 UUID가 현행 ID와 다르면 무시한다. `stop()`은 `async_cancel_all_goals()`를 호출하고, 탐색 종료 시 설정에 따라 별도 초기 위치 복귀 목표까지 전송한다. 복귀 경로의 UUID 소유·중지 인터페이스도 별도 구현돼 있지 않다.

같은 ActionServer를 쓰는 WEB/RMF/Agent 목표도 cancel-all의 영향 범위에 포함될 수 있다. Nav2는 pending/preempt 경로를 제공하므로 다른 새 Goal이 수락될 때 기존 목표에 미치는 영향이 있다. 실제 구성별 선점 정책은 모의 시험만으로 단정하지 않는다. GoalStatusArray의 UUID 관측은 클라이언트 소유권이나 취소 권한의 증거가 아니다.

기존 Agent는 자신의 ActionClient, journal, UUID, GoalHandle 취소, 중복·늦은 수락·불명 결과 처리를 갖는다. 하지만 현재 Explore 요청 경로·소유 프로세스·복귀 목표, 여러 목표 발행자 배제 및 외부 Goal 재조정은 구현하지 않는다. journal 재시작 후 Python GoalHandle이 복원된다고 가정해서는 안 된다.

## 구조 비교

| 기준 | A: explore 자체 개선 | B: 기존 Agent 경계 확장 |
| --- | --- | --- |
| UUID·개별 취소 | explore가 실제 수락 핸들·UUID를 보관하고 외부에 증거 전달 | Agent ActionClient가 실제 핸들·UUID 보유; Explore는 목표 요청자 |
| 소유·중복·선점 | Explore 외 WEB/RMF와 별도 권한 중재가 반드시 필요 | Agent 내부에서 한 번에 하나의 소유권을 기록할 수 있으나 **다른 직접 ActionClient를 차단/탐지하지 않으면 불완전** |
| cancel-all | explore의 해당 경로와 종료 시 복귀 목표까지 제거·교체 필요 | Explore의 모든 직접 send/cancel 경로를 프록시로 전환해야 함 |
| 재시작·늦은 응답 | Explore 별도 durable journal·불명 처리 필요 | Agent의 기존 journal·늦은 수락·UUID 처리 재사용 가능; Explore 소유권 확장 필요 |
| 규모·시험 | 단일 패키지 변경은 작을 수 있으나 전체 다중 소유권 추가 필요 | 인터페이스 변경은 크지만 단일 경계의 실패 주입·감사 시험이 용이 |

권장안은 **B를 목표 설계로 채택하되 단계적으로 A의 관측/안전 제약을 먼저 검증**하는 것이다. 이는 별도 신규 Arbiter 생성이 아니다. Fleet Server는 단일 제어 평면이고 Agent는 로컬 실행 경계다. 기존 Explore의 직접 Goal 제출·복귀·cancel-all과 다른 ActionClient를 실제로 제한할 방법이 확보되지 않으면 B도 선점 방지를 보장하지 못한다. 권한 있는 모든 경로를 제어하지 못하면 외부/미상 목표에서 RECONCILIATION_REQUIRED로 차단한다.

## 목표 증거 계약 (제안)

| 정보 | 생산자 → 소비자 / 영속성 |
| --- | --- |
| operation_id, command_id | Fleet Server → Agent/Explore 요청자; 서버 DB와 Agent journal에 영속·중복 방지 |
| owner_type/id | 서버가 인증된 요청과 lease·operation에서 결정 → Agent/Gate 재조정; 서버·journal에 영속 |
| goal_uuid | 실제 Goal 수락을 관측한 ActionClient → journal·서버; 소유 명령과 결합해 영속. 제출만으로 수락 간주 금지 |
| agent_boot_id / gate_boot_id / fault_epoch | 각각 Agent와 Gate 생산 → 서버·Agent·Gate; 명령 admission 및 journal에 불변으로 영속 |
| operation_revision, map_epoch, localization_epoch | 신뢰 가능한 각 생산자 → 서버·Agent; admission 및 목표 journal에 영속. 현재 생산자는 미완성 |
| submission evidence | Agent가 호출 전 journal에 의도·명령을 저장하고 전송 시도 기록. 서버·Agent 보존; 호출 기록은 수락 증거가 아님 |
| acceptance evidence | 실제 Goal response와 UUID를 Agent가 저장·서버 보고. 지연·유실 시 결과 불명 |
| cancel request / ACK | 해당 핸들에 대한 요청 ID·UUID와 응답을 journal/서버 보존. cancel ACK 자체는 terminal 아님 |
| terminal result | 해당 UUID의 Nav2 terminal 결과와 출처·관측 시각·세대. journal·서버에 영속. `CANCEL_RESPONSE_TERMINATED`를 쓰려면 실제 terminal을 별도로 입증해야 하며 단순 cancel 수락과 동일시하지 않음 |
| process termination | 프로세스 소유자가 ID·종료 결과·세대를 기록해 OM/서버에 전달. 목표 terminal과 별도 |
| reconciliation | 서버가 미확인 명령/UUID·저널·외부 목표·새 세대와 승인자·근거를 기록. 불명확 시 해제 금지 |

충분히 최신의 상태·소유권·terminal 증거가 없으면 idle로 간주하지 않는다. 시간 경과, publisher 생존, 빈 표본 하나, 사용자 버튼이나 `idle=true`만으로 Goal 부재를 증명하지 않는다. Nav2 action 서버 인스턴스와 status 발행자 연속성, 모든 이동 가능 action 범위를 독립 검증할 방법은 미확정이다.

## Gate v4.2 attestation 초안과 차단 조건

Gate 후보의 문서상 초안은 계약 버전·역할·HMAC, Gate boot/fault epoch, 1회용 challenge, Gate 단조 시계 기준 5초 신선도, status writer GID 집합, action 서버 범위, UUID별 terminal evidence, 외부 목표 및 미상 목표 부재 근거를 요구한다. Gate가 본 활성 Goal을 attestation이 덮을 수 없고 Gate가 status 표본을 한 번도 보지 못했으면 Agent 단독 증언은 부족하다. 새 fault·재부팅·발행자 변경·challenge 재사용 시 폐기한다. 역할별 키와 전달·재부팅·재생 방지는 별도 검증해야 한다.

이것은 **제안일 뿐 구조화 attestation 생산·검증 기능은 현재 미구현**이다. Gate의 발행자 GID 연속성 검증 및 여러 action 서버 감시도 미구현이며, 표본 신선도에 관한 Gate 후보 제안과 Agent/서버의 2초 실제 수신 규칙은 상충한다. `STATUS_TERMINAL`의 출처·시점과 `CANCEL_RESPONSE_TERMINATED`의 독립적 종료 증거 범위도 결정 전이다. 물리 fault가 Agent callback보다 먼저 발생하는 경계는 Agent 제출 잠금만으로 막을 수 없으므로 독립 Gate 차단이 필수다.

## Mock 범위와 후속 담당

`tests/goal_boundary_attestation_mock.py`는 프로세스 내 1개 목표 잠금과 합성 attestation 규칙만 모델링한다. MAC은 boolean 테스트 픽스처이고, 진짜 HMAC·DDS·journal·ROS 증거·시계 연속성·프로세스 격리는 구현하지 않는다. 서버·Agent 재시작도 메모리 상태 시뮬레이션으로만 다룬다.

- Explore 담당: `capstone_ws/src/m-explore-ros2/explore/src/explore.cpp`의 직접 Goal/복귀/cancel-all 경로를 소유 계약에 맞추는 후속 설계·시험.
- Agent 담당: `fleet_control/agent/limo_robot_agent/limo_robot_agent/{core,ros_adapter,safety_protocol}.py`의 경계·journal·소유 UUID·terminal·재시작 처리 확장.
- Server 담당: `fleet_control/server/fleet_server/{service,ownership,recovery,safety}.py`의 운용 소유권·lease·불명 결과 영속 계약 및 감사.
- Gate 담당: v4.2 후보의 구조화 attestation, 세대·신선도·GID·범위 검증은 별도 설계와 승인. 후보 파일은 이번에 변경하지 않음.

사용자 결정: A/B 및 직접 ActionClient 통제 수준, 탐색 종료 시 복귀 동작, attestation 권한·키 배포·terminal 증거, 상태 증거 신선도, 중지 시 소유자 외 권한과 복구 승인. 실차 전에는 실제 Nav2·Gate·ROS DDS·재시작과 외부 목표 경쟁 시험, 모터 잠금 및 E-stop 검증이 추가로 필요하다.
