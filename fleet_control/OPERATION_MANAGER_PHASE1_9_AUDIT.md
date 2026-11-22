# Phase 1.9 — Robot Agent 실행 경계와 Fencing 설계 감사

읽기 전용 소스·기존 시험 기록 검토다. 이번 작업에서 ROS·Gazebo·실차 시험이나 운영 코드 변경은 수행하지 않았다. 사용자가 선택한 B안은 목표 설계이며 현재 운영에 구현된 단일 Goal 경계가 아니다. Gate v4.2는 Claude 담당 후보로서 실차 승인 여부를 이 문서에서 판단하지 않는다.

## 1. 현재 확인된 경로와 한계

- `agent_node.py`는 ROS 노드를 실행하고 `allow_missions=false`를 기본으로 사용한다. 실제 entrypoint는 `AgentCore(require_safety_context=True)`를 생성한다. `core.py` 생성자의 기본값만 따로 보면 false이므로 다른 진입 경로는 별도로 검증해야 한다. 토큰 파일의 넓은 권한은 현재 경고만 하며 종료하지 않는다. [entrypoint](agent/limo_robot_agent/limo_robot_agent/agent_node.py).
- [현장 시험용 `limo_run.sh`](scripts/limo_run.sh)는 실행 전 프로세스 문자열을 검사하고 자식 프로세스를 시작·종료한다. 이는 체크와 실행이 원자적이지 않고 우회도 가능하다. 조사한 저장소 경로에서는 운영 Agent에 대한 systemd 단일 인스턴스·OS 사용자·SROS2 enclave 구성이나 강제 lock을 확인하지 못했다. 실차는 접속하지 않았으므로 배포 환경에 존재하지 않는다고 단정하지 않는다. 별도 현장 기록의 ROS 노드 중복 감지 역시 전체 OS 강제 경계의 증거가 아니다.
- [AgentCore](agent/limo_robot_agent/limo_robot_agent/core.py)는 boot ID, 명령 중복 저널, fsync/replace, 이전 비종료 기록의 reconciliation 및 세션 freshness를 제공한다. 손상된 저널은 실패로 둔다. 단일 파일과 `RLock`은 다른 프로세스·서로 다른 state_dir의 실행 소유권을 강제하지 않는다. 파일/DB가 함께 과거로 복원되는 상황의 단조성은 보장되지 않는다.
- [RosRobot](agent/limo_robot_agent/limo_robot_agent/ros_adapter.py)은 Nav2 ActionClient, 다단계 Safety Context 검사와 같은 프로세스 lock 안의 제출, GoalHandle 개별 취소, UUID 및 GoalStatusArray 관측을 제공한다. 외부 Goal은 실제 관측된 status에서 구분하지만 관측되기 전의 다른 ActionClient 제출을 배제하지 못한다. 운영·지도·위치추정 생산자는 `_producer_contexts()` 기본 경로에서 UNKNOWN/미검증이다. fault 발생에서 callback 관측까지의 공백과 실제 Gate의 독립 차단은 별개다.
- [Fleet Server](server/fleet_server/safety.py)는 agent boot·Gate/operation/map/localization 세대, 신선도 및 status를 검사하고 [service](server/fleet_server/service.py)는 생성·배차·전달에서 재검사한다. [lease](server/fleet_server/ownership.py)는 관제 요청자의 제어권이며 호스트 프로세스 실행권이나 이미 전달된 Nav2 Goal의 강제 fencing이 아니다. [reconcile](server/fleet_server/recovery.py)은 소유 불명 결과를 보수적으로 다룬다. HTTP 승인과 실제 ROS 전송은 원자적이지 않다.
- WEB은 기존 Fleet mission API를 호출하고, [RMF adapter](rmf_adapter/limo_rmf_adapter/adapter_node.py)는 Mission Manager 경유다. [Explore](../capstone_ws/src/m-explore-ros2/explore/src/explore.cpp)는 직접 ActionClient를 사용하고 `cancel-all` 및 조건부 복귀 Goal을 가진다. 이는 현재 단일 제출 경계 밖이다.
- 현재 실차 통합 런처 소스의 경로는 Nav2 → `/cmd_vel_raw` → Safety Gate → `/cmd_vel` 또는 모터 잠금 시 `/cmd_vel_locked`이다([launch](../capstone_ws/src/limo_capstone/launch/limo_integrated_auto_mapping_real.launch.py)). 이것은 해당 구성의 설계이며 다른 발행자 배제·물리 E-stop·실제 장치 동작까지 증명하지 않는다.
- SROS2 Enforce의 미인가 엔드포인트 거부는 [Phase 1.5 실험](../experiments/phase15_sros2_20261115_F8v2/RESULTS.md)에서만 검증했다. 운영 enclave, 키 소유권·권한 및 실행 여부는 이번 조사에서 확인되지 않았다. Phase 1.8의 합성 attestation은 Gate 서명이나 challenge가 아니다.

## 2. 위협 모델 및 실제 강제 경계

| 경우 | 현재 위험 | 강제 위치와 필요한 차단 |
|---|---|---|
| A. 동일 호스트 실수로 중복 시작 | 프로세스 목록 사전검사 TOCTOU·직접 실행 | Supervisor의 단일 서비스 + 동일 고정 lock 경로의 커널 lock은 협조적 실수 방지; crash 복구 시 잠금 획득과 기존 Goal 종료를 별개로 판단 |
| B. 다른 Journal의 Agent | 각각 독립된 중복 기록·lock으로 승인 가능 | Agent별 state_dir 선택을 금지하고 로봇 ID→관리자 소유 고정 경로를 구성; 실행 경계가 실제 검증하지 않으면 미보장 |
| C. 동일 enclave 인증정보 복제 | DDS는 같은 자격증명을 쓰는 프로세스를 신뢰; Phase 1.7에서 두 Goal 수락 | 전용 OS 계정·key 접근 분리와 고정 supervisor, 별도 SROS2 principal; 같은 계정에서 같은 키 사용을 완전히 차단한다고 가정하지 않음. 실제 Action 접근 제어 및 중재 경계 필요 |
| D. Agent SIGKILL 후 Goal 잔존 | 새 프로세스가 GoalHandle을 복원할 수 없음 | Gate가 독립 차단, 서버·Agent는 UNKNOWN/RECONCILIATION_REQUIRED. 소유권 lock 해제는 terminal 근거 아님 |
| E. SQLite·Journal 과거 복원 | 세대·중복 이력 되감기. Phase 1.8 실제 FAIL | 복구 외부의 독립 신뢰 가능한 고수위/새 실행 세대 확인; 확인 실패 시 이동 거절. 같은 롤백 영역에 복제한 카운터·서명만으로 확실한 감지 불가 |
| F. Server 재시작/lease 불일치 | 구 명령·outbound 상태가 실제 Action과 비원자적 | 기존 server boot 및 lease epoch 비교, 기존 미전송 무효화, 전달 불명은 reconciliation. 서버 부팅 ID/lease가 현 세대라도 이전 Goal 종료를 증명하지 않음 |
| G. Gate boot/fault 변경 | 구 명령·진행 중 Goal 유효성 상실 | Gate 독립 차단, Agent가 관측 후 취소 요청; terminal·소유권·현재 세대 재조정 전 신규 목표 금지. 물리 고장과 Agent 관측 사이 빈틈 인정 |
| H. 외부/미관측 Action Goal | status 지연·유실 또는 다른 Action 서버의 Goal 누락 | Action 접근 강제 제한 및 전체 이동 경로 감시; 관측 freshness/GID/범위·UUID가 불명확하면 reconciliation. 빈 배열이나 publisher 생존만으로 안전한 idle 확정 금지 |
| I. 자격정보 유출/OS 침해 | 인증 주체 사칭, 프로세스·lock·DB 변조 또는 Gate bypass | 정상 운용 오류와 분리한 보안 위협. 전용 UID·파일 접근·보안 부팅/키 관리 등은 위험을 줄일 뿐, root/커널 장악 상태까지 SROS2·flock으로 보장하지 않음. Gate·물리 정지의 독립성도 별도 검증 |

## 3. 대안과 권고

| 선택지 | 장점·변경 | 고장·롤백·검증 한계 |
|---|---|---|
| 1. Supervisor + 전용 OS 계정 + 전용 SROS2 enclave | 기존 Agent를 유지; 실수 중복과 미인가 일반 프로세스 접근 축소 | 서비스 관리, UID/key/lock 정책 필요. 같은 권한·자격증명 또는 침해된 프로세스에는 불충분; 저장소 롤백과 활성 Goal을 자체 해결하지 못함 |
| 2. 외부 권한 서비스의 단조 실행 세대 | 재시작/복원에 로컬 DB 밖 고수위 사용 가능 | 외부 서비스 자체의 rollback/분할 방어·가용성·인증·lease 만료 필요. 세대만 저장해도 ROS/Nav2가 강제하지 않으면 오래된 프로세스가 Goal을 보낼 수 있음; 장애 중 fail-closed |
| 3. 실제 Action 경계의 권한·세대 검사 프록시 | 모든 허가 요청의 epoch·소유권을 실제 제출 직전에 강제하는 후보 | 직접 Nav2 접근을 SROS2/OS 경계에서 막아야 하고, 프록시 우회·복제·자체 재시작·기존 Goal·in-flight 요청을 검증해야 함. Nav2가 token을 기본 지원한다고 가정 금지. 프록시 추가는 기존 Agent 중재 재사용 우선 원칙과 구현 비용을 비교해 승인 필요 |
| 4. 기존 구조의 단계적 확장 | 기존 Fleet 단일 제어 평면과 Agent journal·제출 fence 재사용, 기존 기능의 변경 범위 최소화 | 먼저 1의 실행 환경과 인증 경계를 검증하고, 안전한 세대 원천과 실제 제출 시점 강제를 검증한 뒤 2/3 필요성을 결정. 각 단계에서 기존 Goal 불명·Attestation 부재는 차단 |

권장안은 **4를 순서로, 1을 최소 운영 기반 후보로 사용**하고, 비협조적 중복 실행과 롤백에 대한 위협 수준을 정한 뒤 2와 3의 추가 여부를 결정하는 것이다. `flock`만으로는 비협조자가 다른 경로를 열거나 SROS credential로 직접 Action 호출하는 것을 막지 못한다. 외부 신뢰 원천이 없는 완전한 동일 상태 복원은 복원 전 상태와 구별할 수 없으므로 세대 재사용을 확실히 탐지할 수 없다.

## 4. 새 실행 주체 및 종료 증거 계약 제안

시작 시 이동 차단 → 로봇별 독점 실행권 획득 → 신뢰 원천과 server/agent/gate/lease 및 map/operation/localization 세대 대조 → 저널·외부 Action 서버 상태와 이전 Goal UUID/소유권 재조정 순서로 진행한다. 소유권 변경·잠금 획득·Gate reset·시간 경과만으로 과거 목표를 재개하지 않는다. 오류 또는 서로 다른 증거는 UNKNOWN/RECONCILIATION_REQUIRED를 유지하고, 새로운 목표는 별도 승인·새 context로만 제출한다.

Cancel 요청과 cancel ACK는 terminal이 아니다. 기존 Goal의 출처와 UUID가 일치하는 신뢰 가능한 실제 terminal result/허용된 독립 증거, 모든 관련 Action 서버의 관측 범위 및 소유 프로세스 상태가 확인되어야 한다. GoalHandle 유실 후 취소 결과나 terminal이 불명확하면 계속 차단한다. 외부 Goal은 소유권이 확인되기 전 임의 cancel-all하지 않는다. 서버의 기존 복구 규칙과 Gate의 증거 기준을 함께 검증해 일치하지 않으면 더 보수적으로 차단한다.

Gate v4.2 후보 보고는 구조화 Attestation·status 발행자 GID 연속성·다중 Action 감시가 미구현이고 최초 status 부재 시 idle 증거 교착 가능성을 기록한다. 현재 Agent의 status 수신 freshness와 Gate 후보의 상태 생존 기준도 차이가 있다. challenge/서명, `(gate_boot_id, fault_epoch)`, ActionServer identity·GID·UUID별 terminal, 만료·재사용 방지, 최초 idle 증명의 책임과 fail-closed 경로는 Gate 담당과 별도로 합의·시험해야 한다. 임시 합성 boolean을 실제 Attestation으로 사용하지 않는다.

## 5. 최소 변경 후보와 검증 계획

실제 수정 전 사용자 승인 및 소유권 조율이 필요하다.

- Agent 담당: `agent_node.py`의 고정 실행 환경·소유권 획득/상실 처리, `core.py`의 저널 세대 및 복원, `ros_adapter.py`의 제출·callback·취소 직전 재검사, `safety_protocol.py`의 확인된 Gate 및 실행 세대 계약. 기존 기능을 교체하지 않는다.
- Fleet 담당: `server/fleet_server/{ownership,recovery,safety,service}.py`에서 기존 lease와 별도의 실행 주체 세대 계약·재조정 및 DB 복원/서버 재시작 검증. 새 제어 서버는 만들지 않는다.
- Explore 담당: `capstone_ws/src/m-explore-ros2/explore/src/explore.cpp`의 직접 Goal/`cancel-all`/복귀 Goal을 Agent 요청으로 전환하는 별도 설계. Gate 담당 파일은 Claude가 유지한다.
- 배포/보안 담당: 전용 OS 계정, 접근 제어된 keystore, Enforce, 고정 서비스 및 Action·`/cmd_vel`·`/cmd_vel_raw`·기타 이동 Action의 최소 권한. 구체 경로는 실제 launch 및 remap을 확인한다.

최소 격리 부정 시험: 다른 Journal 및 lock 경로를 사용하는 중복 Agent; lock을 따르지 않는 비협조 client; 동일 인증정보 복제; 모든 상태 저장소의 DB snapshot rollback; Agent SIGKILL 후 이전 Goal 활성·terminal 유실; Gate Attestation 누락·만료·다른 boot/fault; 외부 Goal 및 상태 유실; 이전 세대 지연 acceptance/terminal; server 재시작·lease 만료·네트워크 분할; 직접 다른 이동 Action과 속도 명령 우회. 정상 경로도 실제 Agent+모의 ActionServer에서 SROS2 인증, 목표 수락·feedback·terminal·개별 취소와 Gate 독립 정지 통신을 검증한다. 이미 Phase 1.8에서 확인된 협조 flock 시험만 반복하지 않는다.

## 6. 승인과 미검증

사용자 결정 필요: 위협 범위(같은 UID의 비협조 프로세스·자격정보 탈취·호스트 침해 포함 여부), 독립 단조성 신뢰 원천과 장애 시 가용성, Agent 내부 경계만으로 충분한지 실제 Action 프록시가 필요한지, 키/운영 계정 관리 책임, Gate Attestation 증거와 복구 승인 권한. 이러한 선택은 별도 승인 전 `DECISIONS.md`에 확정하지 않는다. 이번 조사에 새 실행 시험은 없다.

## 7. 공통 문서 변경 제안 (미적용)

- `CODE_INDEX.md`: 이 감사 문서를 Goal 제출 경계 설계 자료에 추가한다.
- `CURRENT_STATUS.md` / `HANDOFF.md`: 단일 호스트 협조 잠금은 비협조 프로세스 및 저장소 롤백을 막지 못하며, 실제 실행 경계·Gate Attestation 미구현으로 이동은 계속 금지임을 명시한다.
- `VALIDATION.md`: 이번 작업은 기존 자료의 읽기 전용 감사이며 새로운 PASS나 실차 검증을 추가하지 않는다.
- `DECISIONS.md` 및 Claude의 Gate 문서는 변경하지 않는다.
