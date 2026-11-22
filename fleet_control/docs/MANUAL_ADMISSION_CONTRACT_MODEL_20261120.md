# 수동 승인 계약의 독립 모델 (2026-11-20)

## 범위

이 문서와 `tests/test_manual_admission_contract_model_20261120.py`는 독립 함수/모의 Action Server 시험이다. 새로운 ROS 중재 노드가 아니고, staging 또는 public 공통 코드에 연결되지 않았다. **모델 시험 PASS는 기존 안전 결함 수정이나 실제 로봇의 수동 주행 승인 근거가 아니다.** 실제 소스는 건드리지 않았고 LIMO/18190에 접속하지 않았다.

## 기존 제출 및 소유 경로

- `ros_adapter.py`의 `_do_frontier`는 Mapping 세션의 NavigateToPose를 `self.lock`과 `self.active` 슬롯 아래 예약·제출한다. Fleet 임무의 NavigateToPose 역시 사전 저널 후 같은 잠금 아래 조건을 재검증하고 제출한다. 동일 Agent 안의 두 경로는 출발점으로 재사용한다.
- `mapping_worker.py`는 자신이 보낸 목표 종료와 FollowPath 관측을 기다린다. `follow_path_idle()`은 실제로 활성 관측 뒤 해당 publisher의 비활성 상태를 확인한다. 이 상태만으로 특정 FollowPath UUID의 신뢰할 수 있는 action result를 인증하지는 않는다.
- Nav2 BT navigator의 FollowPath 하위 목표는 Agent가 직접 제출하는 NavigateToPose와 별도 UUID다. `safety_protocol.py`의 일부 상관 규칙은 명시적으로 `SIM_ONLY`이며 그래프 이름 관측은 인증이 아니다.
- 기존 `capstone_ws/src/m-explore-ros2/explore/src/explore.cpp`는 NavigateToPose를 직접 전송하는 경로(탐색과 복귀)를 가진다. RViz 등 외부 Action Client도 있을 수 있다. 단일 Agent의 mutex만으로 이들 제출을 막을 수 없다. 전수 확인은 실제 실행 구성별 추가 검증이 필요하다.
- 현 staging에는 `velocity_arbiter.py`가 없고 `manual_web_bench.py`는 0만 발행한다. 공개 기준선의 Arbiter는 별도 버전이다. `motion_watchdog.py`는 기존 별도 프로세스이며 현 구현의 status 관측은 NavigateToPose에 한정되고 미수신을 0과 같이 취급한다.

## 구현을 위한 안전 계약

1. 실제 실행에서 모든 외부 NavigateToPose 및 FollowPath 제출을 막거나 Agent가 조정하는 단일 권한 경계로 강제해야 한다. Nav2 내부의 FollowPath 제출도 해당 제어 영역에 포함한다. 모든 활성 가능한 추가 Nav2 action도 최종 범위에 넣는다. ROS 이름·publisher 수만으로 인증하지 않는다. 외부 action 실행을 원자적으로 막는 권한 격리 및 충돌 시 차단이 확인되지 않으면 실제 MANUAL 승인은 금지한다.
2. Agent 기존 goal slot에 `UNKNOWN`, `AUTO`, `MAPPING`, `MANUAL` 및 generation을 두고, 목표 제출 예약과 수동 소유권 획득은 동일 잠금·세대 검증 경계에서 직렬화한다. 서버 lease 및 Mapping/Mission 전이도 서버 lock과 transaction에서 검사한다. 분산 계층을 하나의 mutex로 원자화할 수 없으므로 Agent가 실제 최종 fencing을 수행하고 서버가 독립적으로 검증해야 한다. 외부 goal이 잠금 밖에서 들어올 수 있으면 이 계약은 성립하지 않는다.
3. 목표 제출 전 UUID와 서버 세대 및 `MAY_HAVE_SENT` intent를 내구성 있게 기록한다. 수락, 취소 수락과 terminal 결과는 서로 다른 상태다. Agent가 관측·검증할 수 있는 각 NavigateToPose·FollowPath UUID에 대한 action 서버 출처·세대 결박 terminal 결과가 필요하다. 결과 분실, 재시작, 이전 세대, 미확인 외부 제출 또는 모순은 UNKNOWN이다. 단순 empty cancel 응답이나 status 배열로 과거 goal의 종료를 추론하지 않는다.
4. Goal이 한 번도 없던 정상 성공 경로는 **초기 신뢰 기준점**이 필수다. 격리된 단일 제출 경계가 먼저 확립되고, 해당 시점 이후 전달 기록이 완전하며, 모든 해당 action 서버에 대한 현재 세대의 증거와 유효한 상태를 받았다는 전제가 필요하다. 프로세스 재시작 후 비어 있는 로컬 메모리만으로 이 초기 조건을 만들 수 없다.
5. ROS `GoalStatusArray` 자체에는 이 모델의 단조 `seq` 필드나 인증된 server epoch가 없다. 모델은 이것을 제공하는 신뢰 가능한 생산자·전달 계층을 가정한다. 실제 구현은 publisher GID/서버 세대 바인딩, 출처 인증, 지속적 신선도/순서 보증과 늦은 메시지 거부를 별도로 설계·입증해야 한다. 불가하면 UNKNOWN으로 남긴다. 배열 미수신/지연/중복/역순/비일관성은 무장 취소한다. 동일 메시지의 합법적 반복을 구분할 규칙은 실제 생산자 계약에서 정의해야 한다.
6. 서버·Agent boot, 연결 세대, lease epoch·owner·deadline, mode generation, session ID, 단회 ticket 및 단조 sequence에 세션을 결박한다. 연결 상실, 재시작, lease 변경/만료, Gate·Watchdog fault, 증거 이상, Deadman release 또는 입력 만료는 기존 권한을 무효화한다. 복구에는 fresh proof와 새 session 및 명시적 재무장이 필요하다. server-side 단속 외에 Agent 자체가 타임아웃을 독립 강제하고 watchdog/Gate 역시 독립적으로 차단해야 한다.

## 모델 시험

`PYTHONDONTWRITEBYTECODE=1 python3 fleet_control/tests/test_manual_admission_contract_model_20261120.py`

[로그](../../local_data/raw_results/manual_admission_contract_model_v3_20261120.log): **15 PASS**. 함수 기반 mock은 두 action 모두 초기 snapshot을 제공하고, Goal 없는 상태에서 MANUAL과 비영 0.1/0.2 입력을 허용한다. Goal과 그 하위 action의 terminal 결과가 각각 도착하고 그 후 snapshot을 받은 경우에도 승인한다. 활성 Goal, cancel accepted만 있고 결과가 없는 경우, 누락·지연·중복·역순·불일치, 과거 결과 재사용, Mapping, lease/boot 변경, ticket·sequence 재사용, Gate/Watchdog fault를 거부한다. 100회 두 thread 경합은 동일 프로세스 모델의 lock에서 수동 또는 goal 한쪽만 승인됨을 확인한다. 정상 성공은 모델에서만 존재한다.

**모델 가정과 한계:** `MockActionServer`는 해당 프로세스의 객체이지 ROS 2 ActionServer가 아니다. `configure_fence()`는 권한 격리와 신뢰 생산자가 있다고 가정하며 실제 기능이 아니다. terminal 결과의 인증, 권한·IPC, action 서버 재시작과 기존 UUID 저널 복구, FollowPath 자식 소유권, 모드 전환의 분산 동기화는 구현되지 않았다. 검사 단계의 boolean Gate/Watchdog는 독립 프로세스가 아니며, 명령 TTL·Deadman·실시간 fault 출력은 검증하지 않았다. 실제 ROS `GoalStatusArray`는 모델의 sequence를 제공하지 않는다. 모델은 배포하거나 안전 체인에 연결하면 안 된다.

## 기존 FAIL별 상태

| 기존 실패 | 현 코드 | 모델에서 다룬 범위 |
|---|---|---|
| staging Offline 후 이전 세션 | FAIL 유지 | 동일 boot 복구 후 새 session만 허용 |
| staging Mapping 동시 제어 | FAIL 유지 | 합성 상태와 단일 lock에서 차단 |
| staging 활성 Goal | FAIL 유지 | 두 가상 action의 미종결 시 차단 |
| staging 상태 미수신 | FAIL 유지 | 가상 status 미수신 차단 |
| staging 오래된 상태 | FAIL 유지 | 가상 timestamp 만료 차단 |
| staging publisher 중복 | FAIL 유지 | 신뢰 단일 생산자 fence를 가정할 뿐 실제 graph 중복 미시험 |
| staging 잘못된 Goal 수 | FAIL 유지 | 가상 UUID set 불일치 차단; 실제 parser 미시험 |
| public 미수신·지연 | FAIL 2 유지 | public 버전 미변경·미시험 |
| 독립 Watchdog 미수신 | FAIL 유지 | 모델의 유사 조건만 확인. 실제 Watchdog 미수정 |

## 충돌 없는 통합안 및 격리 ROS 시험 계획

`ros_adapter.py`의 기존 `self.lock`/`self.active`와 UUID journal을 확장하고, `mapping_worker.py`의 Mapping 제어 및 결과 기록을 새 generation 계약에 맞춘다. `service.py`/`ownership.py`/`mapping.py`는 기존 Fleet lock과 DB transaction 아래 서버 측 모드·lease·manual 세대를 관리하고, `agent_node.py`와 manual/Arbiter는 부트·세션·ticket·유효시간을 별도 검증한다. `motion_watchdog.py`는 누락/오래된 상태 및 실제 구성의 action 범위를 독립 검증한다. Gate 및 기존 Watchdog을 우회하지 않는다. 이 파일들은 다른 에이전트 작업과 겹치므로 변경 제안만 남긴다. 외부 action fencing/증거 생산의 구현 위치와 신뢰 경계가 확정되기 전에는 실제 성공 경로를 연결하지 않는다.

소켓을 사용할 수 있는 격리 환경에서 먼저 사용 가능한 ROS_DOMAIN_ID와 보호 포트를 `WORKTREES.md` 및 실제 실행 상태로 확인한다. `ROS_LOCALHOST_ONLY=1`, `unset ROS_DISCOVERY_SERVER`, 별도 임시 DB/인증정보, OS 자동 할당 loopback HTTP 포트를 사용하고 운영 18190에는 연결하지 않는다. 이후 실제 Fleet HTTP·Agent·Arbiter·독립 Watchdog·Gate와 별도 mock NavigateToPose/FollowPath Action Server를 배치한다. 모든 속도는 Gate의 잠금 출력으로 remap하고 `/cmd_vel` publisher 0을 시험 시작·도중·종료에 관찰한다. 위 15개 모델 조건과 경쟁·restart·네트워크 단절·TTL·Deadman을 실제 메시지와 subprocess fault로 반복하고 기록한다. 외부 클라이언트가 잠금 밖으로 goal을 내보낼 수 있는 반례도 주입해야 한다. 이 실행은 **SKIP**, 실차 검증은 **SKIP**이다.

공통 문서 `ARCHITECTURE.md`, `CODE_INDEX.md`, `CURRENT_STATUS.md`, `VALIDATION.md`, `HANDOFF.md`는 공유 작업과 충돌을 피하기 위해 변경하지 않았다. 통합 후 실제 구현과 시험 수준을 반영할 것을 제안한다.
