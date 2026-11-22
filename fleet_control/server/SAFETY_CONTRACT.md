# Fleet Server R3/R4 안전 계약 — 합성 시험 단계

이 파일은 구현·프로토콜 제안과 검증 한계 기록이며 실차 구동 승인이나 Gate 정책 변경이 아닙니다. 기존 Agent 및 Safety Gate에는 이번에 변경을 가하지 않았습니다.

## 서버에 필요한 Agent 상태

이동 승인에는 `/agent/hello`의 프로토콜 2, `safety_contract_version: 1`, 고유한 `boot_id`가 필요하며 `/agent/reconcile`은 동일 boot의 journal/execution을 사용합니다. **hello 자체는 구형 Agent 등록을 허용하지만 이는 이동 승인이 아닙니다.** 다음은 `/agent/state`의 필수 필드입니다. 현재 Agent는 이 계약 전체를 발행하지 않으며 구버전은 신규 이동을 `409 safety_not_ready`로 거절합니다. 필드를 고정값으로 채우거나 SIM_GATED를 실차에 사용하는 전환은 금지합니다.

```json
{
  "safety_gate": {"boot_id":"gate-process-id", "status":"NORMAL", "age_s":0.1,
    "fault_epoch":3, "fault_latched":false, "cmd_blocked":false,
    "movement_permitted":true},
  "operation_mode":"NAVIGATION", "operation_revision":4,
  "map_epoch":7, "map_version_id":"verified-map-v1", "map_verified":true,
  "localization_epoch":9, "localization_map_version_id":"verified-map-v1",
  "localization_status":"CONFIRMED",
  "execution":{"active":null,"nav2_goals":{"known":true,"active_count":0,
    "received_status_array":true,"age_s":0.1}}
}
```

나이(age_s)는 Agent의 각 실제 관측 시점 기준 초이고 서버 상태 수신 이후 경과 시간과 합산합니다. `execution.nav2_goals.age_s`는 마지막 status 배열 수신 이후 시간이며 `last_change_age_s`(마지막 상태 변경 이후 시간)와 다릅니다. 후자는 신선도 증거가 아닙니다. 서버는 상태·Gate·목표 관측에 2초 상한(offline_after가 짧으면 그 값)을 적용합니다. 값은 음수/NaN/무한/불명이면 거절하며 이 임계값은 실차 검증된 파라미터가 아닙니다. 각 epoch는 boot 내에서 감소하지 않는 정수이며 감소하면 해당 boot는 차단됩니다. 지도 검증과 localization CONFIRMED는 실제 지도 해시·동일 지도·TF·covariance·scan 검증으로 산출해야 하고 이 값만으로 물리적 안전이 증명되는 것은 아닙니다. READY는 동일 의미로 추정하지 않습니다.

`movement_permitted`는 Gate 정상만의 동의어가 아닙니다. 생산자는 임무 허용 설정, 검증된 모터 경로 및 신선한 안전 상태 등 별도 조건을 모두 확인해야 합니다. `allow_missions=false`이면 false여야 하지만 **현재 서버는 해당 설정을 독립 확인하지 않고 Agent 보고를 신뢰**합니다. 실차 의미와 생산 책임, watchdog·localization 검증 방법은 미확정입니다. 이 값은 **전원·모터의 물리 잠금 해제 명령이 아닙니다**. 낡은 Gate 상태·누락 필드·변경된 맵/위치추정·운용 모드 불명은 fail-closed입니다. 유효한 operator lease와 정확한 epoch는 `/missions` 요청의 `lease_epoch`로 전달해야 합니다. 기존 웹/RMF 호출은 변경 전까지 이동 요청이 거부됩니다.

### 현재 서버가 주고받는 JSON

인증된 Agent의 hello 본문은 `{"robot_id":"r","agent":{"protocol_version":2,"safety_contract_version":1,"boot_id":"agent-id"}}`입니다. 상태 본문은 `{"robot_id":"r","agent_boot_id":"agent-id","seq":1,"state":{...위 상태 객체...}}`이며 seq는 동일 세션에서 증가합니다. 상태 응답의 `lease`는 null 또는 `robot_id`, `owner`, `epoch`, `acquired_at`, `expires_at`, `ttl_s`, `note`, `remaining_s`를 갖습니다. 응답에는 `server_time`, `server_boot_id`, `safety_contract_version`, `commands`, `recovered_commands`가 있으며 상황에 따라 `needs_reconcile`, `reconciliation_required`가 포함됩니다. lease를 응답에서 받았다는 것만으로 장래 실행이 허가되지는 않습니다.

이동 명령의 정확한 예시 (시각은 Unix 초, 값은 예시):

```json
{"command_id":"c_example","robot_id":"r","mission_id":"m_example",
 "type":"navigate_to_pose","seq":1,"attempt":1,"owner_boot_id":"agent-id",
 "issued_at":1000.0,"expires_at":1010.0,
 "payload":{"goal":{"frame_id":"map","x":1.0,"y":2.0,"yaw_rad":0.0},
  "safety_context":{"version":1,"server_boot_id":"server-id","agent_boot_id":"agent-id",
   "gate_boot_id":"gate-id","fault_epoch":3,"operation_mode":"NAVIGATION","operation_revision":4,
   "map_version_id":"map-v1","map_epoch":7,"localization_epoch":9,"localization_status":"CONFIRMED",
   "lease_epoch":5,"lease_owner":"operator","issued_at":1000.0,"expires_at":1010.0}}}
```

`safety_context` 15개 필드는 생성 시 `commands.payload_json`에 기록하고 재전달해도 갱신하지 않습니다. 서버의 현재 state 승인 context에는 Gate boot/fault와 operation/map/localization 세대가 포함됩니다. Gate boot가 바뀌면 이전 목표를 무효화하지만, 같은 Agent boot에서 fault epoch의 감소는 Gate boot가 바뀌어도 high-water 검사가 차단합니다. 따라서 새 Gate boot의 카운터가 0으로 돌아갈 때 자동 복구된다고 가정하지 않습니다.

### Agent의 필수 검증 순서 (아직 미구현)

1. 로컬 임무 허용, journal 정상, 미해결 reconciliation 없음, 명령 robot/Agent boot·sequence·중복 여부를 확인한다.
2. 서버 연결·server boot, 명령 구조와 계약 버전, clock 신뢰성 및 양쪽 만료 시각을 확인한다. 불명이면 전송하지 않는다.
3. 실제 수신된 최신 Nav2 status 배열의 `age_s`와 비활성 상태를 확인한다. publisher 수나 단순 graph 생존으로 배열의 수신 시각을 갱신하지 않는다. 마지막 수신이 오래되었다면 데이터가 바뀌지 않았더라도 차단한다. 기존 Agent는 `last_change_age_s`만 보고하며 필수 `age_s`는 미구현이다.
4. 실제 Gate 관측과 boot/fault, 모터 경로 및 `movement_permitted`를 확인하고 운용·지도·위치추정·최신 lease의 owner/epoch/만료를 명령 context와 비교한다.
5. Nav2 서버 대기 후 goal 전송 직전에 2~4를 다시 확인한다. 제출 결과나 취소 결과가 불명이면 재전송하지 않고 reconciliation으로 남긴다.

Phase A의 publisher 생존만으로 idle 기간의 `age_s`를 계속 갱신하는 제안은 위 실제 배열 수신 기반 계약과 충돌하므로 채택하지 않습니다. 현재 status 토픽이 변화 시에만 발행된다면 긴 idle 중에도 의도적으로 차단됩니다. 더 강한 독립 관측 방식은 별도 설계·검증이 필요합니다.

기존 Agent의 `last_change_age_s`라는 이름은 실제로 callback 수신마다 갱신되는 시각에서 계산하며, 상태 내용이 바뀐 시점과 같다고 보장하지 않습니다. 또한 transient-local로 받은 과거 배열이 현재 Nav2의 최신 상태라는 보장은 별도 검증되지 않았습니다.

서버는 계약 버전과 epoch에 불리언을 허용하지 않고 ID·TTL 및 저장된 명령 context의 타입과 범위를 검사합니다. Agent도 독립적으로 동일한 검증을 수행해야 합니다. 현재 시각이 만료 시각 이상이면 전달을 건너뜁니다. 시간 동기화 오차 정책은 미확정입니다.

## 처리 정책

- 동일 idempotency_key, 같은 principal/목표는 이전 결과를 반환합니다. 다른 목표·principal은 409입니다. 기존 결과 반환이 재실행을 의미하지 않습니다.
- 신규 임무 생성 시 Gate, nav status, 모드, 지도 및 위치추정, Agent session, lease와 epoch를 검사합니다. 동일 admission을 DB에 기록하고 배차와 전달 시 재검사합니다.
- 고장·오프라인·Agent hello/reconnect·boot 변경·서버 재시작·안전 context 변경 또는 미검증 상태가 관측되면, 아직 전송하지 않은 대기 임무/이동 명령은 FAILED/REJECTED로 영구 무효화합니다.
- 전송된 이동 명령의 결과를 추정하지 않고 RECONCILIATION_REQUIRED로 둡니다. 이미 종료가 확인된 결과는 실제 terminal 상태만 보존합니다. cancel 결과가 불명확하면 완료로 보고하지 않습니다.
- recover_mission/recover_robot_block은 과거 대기 이동을 무효화하며 `dispatch_next()`를 직접 호출하지 않습니다. reset/recovery만으로 이전 임무를 재개하지 않습니다. 새 임무에는 새로운 요청과 유효한 context·lease가 필요합니다.
- Agent status에 fault epoch가 증가해도 이전 admission과 일치하지 않습니다. fault 중 관측된 epoch의 high-water mark도 저장하므로 같은 boot에서 낮은 epoch로 reset할 수 없습니다. 새로운 boot는 이전 boot를 retire합니다.

## SQLite 경계와 한계

락 순서는 `Fleet.lock → DB.lock → BEGIN IMMEDIATE`입니다. 생성+admission 기록+배차, 상태 수신+고장 무효화+전달 표시, 복구+무효화는 transaction으로 묶이며 중첩 호출은 savepoint를 사용합니다. DB helper 밖의 임의 코드가 DB lock을 잡은 뒤 Fleet lock을 잡으면 안 됩니다. 트랜잭션 안에서 외부 네트워크를 기다리지 않습니다. 실패 시 DB rollback과 메모리 복원을 수행하며 복구 실패 시 reconciliation을 해제하지 않습니다. 기존 이벤트 기록 방식은 rollback에 포함됩니다.

이 설계는 하나의 Fleet Server 프로세스가 해당 DB의 단일 제어 평면이라는 전제입니다. 별도의 두 서버 인스턴스가 같은 DB에 접근할 때의 메모리 상태 동기화는 검증·구현하지 않았습니다. 서버가 HTTP 응답을 전달한 순간과 Agent가 ROS goal을 실제 전송하는 순간은 하나의 트랜잭션이 아닙니다. 서버가 차단한 뒤 이미 전송된 goal을 자체적으로 정지시킬 수 없습니다.

## 다음 Agent 구현에 필요한 조건

Agent는 실제 Gate source의 fault_epoch·latch·이동 허가 및 관측 나이, 승인된 NAVIGATION 모드, 정식 지도 ID/해시·map epoch, localization 검증 결과/epoch, 실제 Nav2 status 수신 여부·관측 나이를 보고해야 합니다. 각 epoch의 증가·재부팅·rollover 정책을 계약화하고, Agent는 서버 command의 session/epoch/lease·만료를 **Nav2 goal 전송 직전 로컬에서 다시 확인**해야 합니다. 연결 단절·Gate fault·map change 시 활성 목표 취소를 요청하고 terminal 결과가 불명확하면 RECONCILIATION_REQUIRED로 남겨야 합니다. 원래 안전 Gate·독립 E-Stop을 대체하지 않습니다.

명령 payload에 불변 `safety_context`를 저장하고 `/agent/state` 응답에 현재 lease를 제공합니다. 유효기간은 명령 TTL과 생성 당시 lease 만료 중 빠른 시각이며 lease 갱신이 과거 명령의 유효기간을 연장하지 않습니다. 서버 재시작은 기존 목표를 무효화합니다. payload 값은 실행 허가 증거가 아니며 Agent의 실행 직전 재검사는 미구현입니다. 장애 발생 시 이미 전달된 목표의 자동 취소와 실차 위치추정 검증도 미구현입니다. 원격 이동 및 기존 운영 서버 배포는 금지 상태입니다.

fixed2 Gate 코드에서 fault_epoch는 프로세스 시작 시 0으로 초기화되고 fault에 증가합니다. Gate boot ID는 해당 코드에서 확인되지 않았습니다. 따라서 Agent가 신뢰할 수 있는 Gate 프로세스 식별을 확보하지 못하면 계약을 충족하지 못합니다. 임의 고정 ID나 정상 상태를 생성해서는 안 됩니다. Gate 부팅 ID 변경 또는 revision 변경은 이전 admission을 무효화하며 같은 Agent boot 내 fault epoch 역행은 보수적으로 차단됩니다.

## Agent 구현용 제안 계약: 생성과 검증 책임

아래는 서버가 요구하는 계약과 다음 Agent 구현의 책임입니다. Agent에서의 보고와 검증은 아직 구현되지 않았습니다. 운영 모드·지도·위치추정의 권한 있는 생산자와 영속화 방식은 통합 시 확정해야 합니다.

| 필드 | 생성·증가와 재시작 | 검사 |
| --- | --- | --- |
| agent_boot_id | Agent 프로세스 시작마다 새로운 고유 ID. 재시작 시 과거 ID 재사용 금지 | 서버 hello/state/reconcile 및 명령 소유자 검사; Agent도 실행 직전 검사 |
| fault_epoch | 신뢰 가능한 Gate 상태 생산자가 장애/잠금 진입과 안전 의미 변화마다 증가. 재시작 시 지속 카운터 또는 새 boot와 함께 울타리 적용 | 서버 생성·배차·전달, Agent goal 전송 직전. 리셋만으로 과거 허가 복원 금지 |
| operation_mode | 허용된 운용 관리자만 NAVIGATION 등 결정; 전환 시 이전 허가 무효화 | 서버와 Agent 모두 실행 직전 최신 값 확인 |
| map_version_id / map_epoch | 지도 등록자의 검증된 불변 ID와 지도 변경 세대. 변경·재적재 때 epoch 증가; 재시작 시 지속 또는 boot로 분리 | 양쪽에서 허가 시점과 실행 시점 일치 확인 |
| localization state / localization_epoch | 위치추정 관리자가 검증 결과와 세대 발행; 초기화·위치 상실·지도 변경 시 세대 증가 | 양쪽에서 CONFIRMED, 지도 일치 및 epoch 검사 |
| lease_epoch | Fleet Server가 lease 신규 획득 때 증가, 갱신은 동일 값. 서버 재시작 후 DB 지속 | 서버와 Agent 실행 직전 검사; Agent가 서버와 끊기면 최신 유효성을 확인할 수 없어 거절 |
| 명령 유효기간 | 서버가 발행 및 만료 시각 지정. clock 불일치·역행·만료 불명 시 차단 | Agent 수신 및 실행 직전 검증; 신뢰 가능한 시간 동기화와 오차 한계는 미확정 |
| Nav2 목표·종료 증거 | Agent가 실제 action status 수신·관측 시각·goal ID와 terminal 결과를 보고하고 내구성 있게 보존 | 서버 복구는 최신 실제 수신 증거 확인; Agent는 취소 후 terminal 결과를 기다림 |

서버의 현재 recovery 증거 검사는 상태 수신 2초 이내(offline 제한이 더 짧으면 그 제한), 실제 status 배열 수신, 활성 목표 수와 해당 관측 나이를 요구한다. Agent에서 합성된 빈 배열을 관측으로 보고해서는 안 된다. 서버의 미션 생성/배차/전달 검사와 Agent의 실제 goal 직전 검사는 서로 대체할 수 없다.

## 공통 문서 변경 제안 (미적용)

- CURRENT_STATUS / HANDOFF: 서버의 합성 R3/R4 단위·HTTP 결과, 레거시 Agent 호환 차단, 미검증 사항을 기록.
- VALIDATION: 각 합성 테스트 명령/결과와 기존 회귀의 정책 충돌을 구분해 추가.
- CODE_INDEX: `safety.py`, 신규 서버 테스트, 이 계약 파일을 추가.
- DECISIONS: 사용자 승인 전 C-01 확정 또는 안전 정책 확정으로 변경하지 않음.

## Agent의 서버 프로토콜 확인과 Mapping STOP 예외 (R1.4.1, 2026-11-23)
- Agent는 hello 응답의 `protocol_version`을 가공 없이 보관하며 그 값은 응답을 준 server boot에만 유효합니다. 재연결(hello 재시작), server boot 변경, `needs_reconcile`에서 폐기하고, 상태 응답에 같은 필드가 오면 그 값으로 교체한 뒤 다시 판정합니다. 이동 허가(`compute_movement_permitted`)와 진행 중인 goal 감시(`check_active_goal_safety`, 사유 `SERVER_CONTRACT_INVALID`)는 `protocol_version`이 정확한 정수 2이고 `safety_contract_version`이 정확한 정수 1일 때만 통과합니다(누락·null·bool·float·문자열·다른 값은 모두 거부). 읽기, 취소, STOP 경로는 이 검사를 쓰지 않습니다.
- Fleet Server는 비호환 Agent(`agent_contract_ok` 거짓)의 Mapping Worker에게 START/RESUME/PAUSE/SAVE, `reserve_goal`, `renew`를 주지 않습니다. 예외: 그 Agent 자신의 boot·세션에 이미 속한 STOP 하나만 `poll`로 전달하고 `accept`/`result`를 받으며, 결과는 최초 1회만 기록하고 서버 이벤트로 남깁니다. 이 예외는 STOP이 전달·기록됨을 뜻할 뿐 물리 정지를 검증하지 않습니다.

### R1.4.2 정정 (2026-11-23)
- `safety_contract_version`은 서버의 모든 상태 응답에 들어 있으므로 Agent는 **매 응답에서** 이 값을 읽습니다. 키가 없거나 정확한 정수 1이 아니면 직전의 정상 값을 유지하지 않고 즉시 무효가 되어 새 이동 명령이 거부되고 진행 중 goal은 감시 주기(0.1 s)에서 `SERVER_CONTRACT_INVALID`로 취소 요청됩니다. 이후 유효한 응답이 오면 계약은 다시 유효해지지만, 무효 동안 거부된 명령은 재실행하지 않습니다. 취소 요청의 성공은 로봇의 실제 정지를 뜻하지 않습니다.
- `protocol_version`의 승인 지점은 hello뿐입니다. 상태 응답에는 보통 없으며(정상), 있고 정확한 정수 2가 아니면 승인을 철회합니다. 유효한 2가 와도 무효 hello를 복구하지 않으며 새 hello(재연결, `needs_reconcile`, server boot 변경)만 다시 승인할 수 있습니다.
- 비호환 Mapping Worker의 STOP `poll`은 boot와 `session_id`가 모두 그 STOP이 속한 세션과 일치할 때만 STOP을 돌려줍니다(없거나 다른 `session_id`면 `command: null`, `renew: false`).

### R1.4.3 계약 세대 (2026-11-23)
- Agent는 서버 계약이 유효에서 무효가 되는 모든 순간(무효 응답, 키 누락·오류값, hello 재시작, boot 변경, `needs_reconcile`)마다 `contract_generation`을 1 올립니다. 복구해도 내려가지 않습니다.
- `navigate_to_pose`는 **수신 시점**에 계약이 유효할 때만 받아들이며(무효면 종결 REJECTED, 큐에 넣지 않고 재전달돼도 재실행하지 않음), 받아들인 명령은 그 시점의 세대를 가집니다. 실행 단계(Stage 1·2·3)마다 현재 세대와 같아야 하며 다르면 `CONTRACT_GENERATION_CHANGED`로 거부됩니다.
- 실행 중 goal은 제출 시점의 세대를 가지며, 세대가 달라지면 이후 정상 응답이 와도 모니터는 계속 `SERVER_CONTRACT_INVALID`로 취소를 요청합니다(의무는 goal 종료 시에만 사라지고 새 goal에는 승계되지 않음). 취소 요청 성공은 Nav2 취소 완료나 로봇 정지를 뜻하지 않으며, 0.1 s 모니터 주기는 정지시간 보장이 아닙니다.
- Cancel/STOP 경로와 Watchdog·Gate의 독립 정지는 이 계약에 종속시키지 않습니다.

### R1.4.4 제출 원자성과 재조정 철회 (2026-11-23)
- Agent의 실행 권한 = 유효한 서버 계약 **그리고** `reconciliation_required`가 정확히 False. 상태 응답에서 True가 오면 계약 무효와 똑같이 세대가 증가하고, 이후 False가 와도 이전 세대의 명령·goal 승인은 되살아나지 않습니다.
- 권한 필드의 변경(Agent 실행 루프)과 Nav2 제출 직전의 최종 검사 및 로컬 `send_goal_async` 호출 시작(RosRobot Stage 3)은 같은 `contract_fence`로 직렬화됩니다(락 순서 `contract_fence -> RosRobot.lock -> AgentCore.lock`, Future 대기는 락 밖). 무효화가 먼저면 제출 0건, 제출이 먼저면 그 goal에 취소 의무가 남습니다. 이미 떠난 요청을 Nav2가 실행하는 구간은 취소 의무로만 제한되며 정지시간 보장이 아닙니다.

### R1.4.5 권한 철회 경로 통일과 재조정 플래그 (2026-11-23)
- Agent의 실행 권한 = 유효한 계약 + `reconciliation_required`가 정확한 Boolean `False` + 서버 세션 연결(`connected`). 셋 중 하나라도 잃으면(HTTP 오류·통신 예외 포함) `contract_fence` 아래에서 세대가 증가하고, 대기 명령은 거부되며 실행 중 goal은 즉시 취소가 요청됩니다(기존 3 s 단절 유예는 이 경로에서 더 이상 적용되지 않음).
- Fleet Server의 정상 상태 응답은 이제 `reconciliation_required: false`를 명시합니다(차단 응답과 reconcile 응답은 원래 명시). Agent는 정확한 `false`만 인정하며 누락·null·숫자·문자열·배열·객체는 모두 재조정 필요로 취급합니다. v4 Agent는 영향 없음, v5 Agent는 이 서버 변경 이후에만 운동이 허용되므로 서버를 먼저 배포합니다. 계약 버전 변경은 하지 않습니다.

### R1.4.6 Lease는 매 상태 응답에서 다시 확인 (2026-11-23)
- 상태 응답의 `lease` 키는 필수입니다(서버는 매번 Lease view 또는 null을 보냄). Agent는 매 응답에서 다시 읽으며, 누락·null·구조 오류(owner 빈 문자열, epoch가 int가 아님, 만료 시각 오류)·만료된 Lease는 모두 "Lease 없음"으로 처리해 실행 권한을 철회합니다(세대 증가). 이전 응답의 Lease를 재사용하지 않습니다.
- Lease 소유자·epoch가 바뀌면 유효→유효 전이라도 세대가 증가하며, 명령의 safety context에 적힌 Lease가 현재 응답의 Lease와 다르면 수신 시점에 거부합니다.
- 운동 승인 필수 필드: hello `server_boot_id`·`protocol_version`(정수 2), 상태 응답 `server_boot_id`(일치)·`safety_contract_version`(정수 1)·`reconciliation_required`(정확한 false)·`lease`(유효), 그리고 연결 상태. `protocol_version`은 상태 응답에서 선택이며 철회만 가능합니다.
