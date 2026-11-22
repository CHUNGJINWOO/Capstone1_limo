# Goal 소유권과 재조정 — Phase 1.1 Mock 제안

운영 서버·Agent·Gate와 연결되지 않는 합성 상태기계다. 기존 Fleet Server를 단일 제어 평면으로 유지하며 새로운 주행 API, 프로세스 감독, ROS Goal 전송 또는 실차 안전 허가가 아니다.

## 소유권

`WEB_NAVIGATION`, `RMF_NAVIGATION`, `AUTO_MAPPING_EXPLORE`, `MANUAL_MAPPING`을 구분한다. 모든 Goal은 operation_id, command_id, owner_type/id, Goal UUID와 agent/gate boot, fault epoch, operation revision, map/localization epoch를 불변 제출 세대로 보존한다. 제출·수락·특정 UUID terminal 증거와 프로세스 종료 증거를 구분한다. 동일 command와 UUID의 재사용 및 동시에 서로 다른 두 소유자의 중복 할당을 금지한다. 외부 Goal의 소유를 추정하지 않는다.

실제 `explore.cpp`는 frontier Goal 수락 callback에서 UUID를 기억하지만 GoalHandle을 저장하는 외부 제어 인터페이스가 없고, `Explore::stop()`은 `async_cancel_all_goals()`를 호출한다. 복귀 Goal 경로도 동일한 UUID 소유 인터페이스가 없다. Agent가 GoalStatusArray에서 UUID를 볼 수 있어도 그것만으로 목표 소유·단일 Goal 취소 권한이 증명되지 않는다. 실제 연동은 소유 GoalHandle/UUID/terminal을 내구적으로 노출하는 explore 측 변경 또는 별도 목표 프록시와 단일 소유권 설계가 필요하다. 이번에는 explore를 바꾸지 않는다.

## 중지와 장애

인증된 operator/admin은 lease 없이 operation_id와 현재 세대를 명시해 중지를 요청할 수 있다. 이때 실제 인증과 권한 검사는 향후 Fleet Server가 수행해야 한다. 소유 목표만 취소하고 요청 중복은 재전송하지 않는다. 상태: STOP_REQUESTED → CANCEL_REQUESTED → CANCEL_ACCEPTED → TERMINAL_CONFIRMED → STOPPED. ACK는 terminal이 아니다. 탐색·수동 맵핑에는 별도의 소유 프로세스 종료 증거가 추가로 필요하다. 핸들·종료·프로세스·status가 불명확하면 RECONCILIATION_REQUIRED다.

Gate boot 또는 fault epoch, 운용·map/localization 세대 변경과 Agent/서버 재시작은 이전 허가를 차단한다. 늦은 수락은 목표가 실제로 수락됐을 수 있으므로 소유 핸들이 있다면 취소하고 reconciliation을 유지한다. UUID 불일치 terminal은 무시하며, 뒤늦은 이전 terminal도 새 허가가 아니다. fresh idle과 소유권·terminal·프로세스 종료 증거 확인 및 관리자 재조정 후 새 요청만 허용한다. 서버와 통신이 끊긴 상태의 실제 로컬 차단은 Gate/Agent에서 독립 보장돼야 하며 이 Mock은 그것을 보장하지 않는다.

## v4.2 연결 시 선행 조건

Gate v4.2 후보는 boot/fault 세대와 status 발행자 검사를 다루지만 구조화 attestation은 NOT_IMPLEMENTED이며 무목표 부팅 시 최초 status 부재로 교착 가능성이 문서화돼 있다. 서로 다른 세대·Agent/OM 재시작, nonce·증거 만료·Goal UUID terminal·소유권 검증을 담당자와 합의해야 한다. 이 Mock의 boolean 증거는 실제 attestation이 아니다. 미관측 물리 fault에서 소프트웨어 제출 차단을 보장하지 않는다.

## 명시적 한계

`tests/operation_goal_mock.py`는 in-memory·단일 스레드 모델이다. 서버/Agent 재시작은 모델 상태를 복사한 시나리오로서 디스크 journal 복원이나 여러 프로세스의 경쟁을 검증하지 않는다. 실제 ROS publisher 출처, GoalHandle 소유·취소 범위, 실제 취소 응답·terminal 신뢰성, 경합·임무 취소 및 안전 게이트는 미검증이다. Boolean 권한·fresh/idle은 테스트 픽스처이며 실제 인증 또는 관측으로 사용해서는 안 된다. 원본 Phase 1 Mock과 독립이다.
