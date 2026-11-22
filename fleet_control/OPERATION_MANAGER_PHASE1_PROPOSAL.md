# Operation Manager Phase 1 — 제안, 미배포

현재 구현과 분리된 Mock 설계다. 서버 API·Agent·Gate·프로세스를 변경하지 않으며 실차 운용 승인이 아니다. Fleet Server를 단일 제어 평면으로 유지하고 기존 lease, mission/command journal, Safety Context, Map Registry 및 Agent 재조정을 재사용한다. 중복 서버·임의 셸·안전 우회는 허용하지 않는다.

## 현재 코드와 경계

- Fleet Server는 소유권 lease, 임무 멱등성·명령 TTL·전달 재검사·불명 결과 reconciliation을 제공한다. 운용 프로파일 실행 관리자는 현재 발견되지 않았다.
- `map_registry.py`는 지도 ID·무결성·등록 상태와 바인딩 모델을 제공하지만 등록만으로 로봇에 지도를 적재하거나 localization을 확인하지 않는다. 운영 승격/철회/격리 API는 이번 범위가 아니다.
- Agent의 `context_fn`은 기본적으로 연결되지 않으며 operation은 UNKNOWN이다. 안전 필드를 가짜 정상값으로 대체하지 않는다.
- 자동 탐색의 explore_lite와 Agent의 NavigateToPose는 별도 클라이언트다. 탐색의 전체 취소 동작은 다른 클라이언트에도 영향을 줄 수 있어 목표 소유권과 모드 격리가 선행돼야 한다.
- `limo_integrated_auto_mapping_sim.launch.py`는 시뮬레이션 전용이다. 실제 런처 및 프로세스 소유 경계는 별도 현장 설계·승인 전까지 연결하지 않는다.

## 상태와 전이

| 출발 → 중간 → 완료 | 요청·승인 | 완료 증거 (모두 필요) |
| --- | --- | --- |
| IDLE → STARTING_MAPPING → MAPPING | operator/admin + 현재 lease | 허용된 프로파일의 소유 프로세스·센서/Gate/TF/SLAM 상태, mapping 모드와 독점 목표 소유권 확인 |
| MAPPING → STOPPING_MAPPING → IDLE | operator/admin; 안전 정지 요청은 별도 정책으로 허용 검토 | 탐색 중단, 소유 Goal UUID terminal, 실제 최신 Nav2 유휴 증거, 저장 처리·종료 결과 |
| IDLE → LOADING_MAP → NAVIGATION | operator/admin + 현재 lease | 승인된 불변 지도·해시, map/localization epoch, AMCL 확인, TF 단일 권한 및 센서 준비, Gate 허가 |
| IDLE/NAVIGATION → PROFILE_SWITCHING → IDLE | operator/admin + 현재 lease, 미종료 Goal·임무가 없어야 함 | 허용 목록 프로파일의 기존 프로세스 종료·새 설정 적용 및 상태 증거; 새 목표 자동 제출 없음 |
| 모든 상태 → RECONCILIATION_REQUIRED | Gate boot/fault 변화, 통신 단절, 미확인 Goal·소유권, 불완전 전환, timeout 또는 재시작 | 자동 복구 금지. admin의 근거 확인 후 명시적 재조정만 IDLE로. 실제 결과 불명은 성공으로 표시 금지 |
| 관측 불가 → UNKNOWN | 초기 부팅 또는 검증 불가 | 신규 동작 불허. UNKNOWN을 임의 IDLE로 바꾸지 않음 |

상태 전이 요청과 중간 상태 진입마다 서버가 단조 `operation_revision`을 갱신·영속화하고 해당 context에 묶인 기존 이동 허가를 무효화한다. 목표 소유권과 cancel terminal이 불명확하면 프로파일 변경과 새 이동 모두 거절한다. 시간 경과·publisher 존재·오래된 GoalStatusArray만으로 idle을 증명하지 않는다. 초기 Gate boot/fault 신뢰 생산자·지도 검증 생산자와 세대 변경 복구는 아직 미구현이다.

## 요청·내구성 계약 제안

각 요청은 인증 principal, 허용된 action·프로파일 ID, robot ID, 유효 lease epoch, 기대 `operation_revision`, agent boot, gate boot/fault epoch, map/localization epoch, 고유 command ID, idempotency key 및 만료를 포함한다. 역할·소유권·최신 Gate/TF/goal 증거를 원자적으로 재검사한다. 같은 principal/경로/key/본문은 동일 결과를 반환하며 다른 본문은 충돌 처리한다. 전이 및 감사를 SQLite 트랜잭션에 기록하되 ROS·네트워크·프로세스 대기는 트랜잭션 밖에서 수행한다. 타임아웃은 취소 성공으로 해석하지 않고 RECONCILIATION_REQUIRED로 고정한다.

Agent 또는 서버 부팅·연결 유실 시 기존 작업을 자동 재개하지 않는다. 미전송 운용 명령은 무효화하고 이미 전달된 명령은 실제 효과를 추정하지 않는다. 중지/취소 요청·응답·terminal 결과를 분리한다. 서버의 명령 성공 응답은 Nav2 성공 근거가 아니다. RBAC의 실제 예외 경로와 lease가 없는 비상 정지 동작은 별도 안전 설계 대상으로 이번 Mock에서 운용 API로 노출하지 않는다.

## Gate v4.2 계약과 선행 조건

Gate boot ID·fault epoch는 실제 Gate 생산자가 보장해야 하고 세대 변경에 따른 차단·재조정을 Agent와 Server 양측에서 확인해야 한다. 외부 fault가 아직 Agent callback에 관측되지 않은 경계는 별도 물리 Gate 차단이 필요하다. Gate 후보 구현과 정책은 Claude 소유이며 이번에 수정·승인하지 않는다. 실제 모터 잠금 기본값과 독립 비상정지를 유지한다.

## Mock 범위·검증

`tests/operation_manager_mock.py`는 in-memory 예시 상태기계로 실제 인증·SQLite·ROS status·Gate·소유 프로세스·시간 검증을 구현하지 않는다. `tests/test_operation_manager_mock.py`는 중복 요청·세대 변경·재시작·외부 목표·stale 및 실패 흐름의 기초만 검사한다. 증거 boolean은 테스트에서만 제공하며 실제 관측으로 간주하면 안 된다. 현재 Mock은 중지 요청에도 idle을 요구하므로 활성 explore 목표를 소유한 채 중단·취소하는 정상 경로를 모델링하지 못한다. UUID 기반 소유권과 terminal 결과를 구현한 후 이 제약을 대체해야 한다. 상세 cancel UUID/lease 경쟁·크래시 원자성·실제 리소스 기동과 통합 회귀는 후속으로 필요하다.

## 공통 문서 변경 제안 (미적용)

- CODE_INDEX: Registry 고아 진단과 별도 Mock/본 제안의 위치 추가.
- CURRENT_STATUS: 132개 동일 목록 및 별도 Registry·JS 회귀를 실제 결과에 맞춰 명시; 고아 복구·운영 부하 미검증 유지.
- VALIDATION/HANDOFF: 실측 로그, 오류 및 Mock 전용 범위를 기록하고 과거 로봇 검증과 분리.
- DECISIONS: Gate/전이 정책은 승인된 결정으로 기록하지 않음.
