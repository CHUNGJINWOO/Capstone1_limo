# Fleet HTTP API v1 / Agent recovery protocol 2

HTTP JSON API의 기준 구현은 `server/fleet_server/http_api.py`, 임무 로직은 `service.py`, 재동기화는 `recovery.py`입니다. 경로 접두사는 `/api/v1`입니다. 인증 오류를 제외한 정상 JSON 응답에는 별도 envelope가 없는 경로도 있습니다.

## 인증과 접근

`Authorization: Bearer <token>` 헤더를 사용합니다. 토큰을 URL/query/body에 넣지 않습니다. 실제 토큰 예시는 문서에 포함하지 않습니다. 정적 `/`, `/index.html`, `/app.js`, `/style.css`와 `GET /api/v1/health`만 무인증입니다. `/health` 예시는 `{"status":"ok","api":"v1"}`입니다.

viewer/operator/admin은 조회할 수 있고 operator/admin만 임무 생성·취소를 할 수 있습니다. agent 토큰은 지정된 robot_id의 Agent POST만 허용됩니다. agent는 operator 조회 API를 읽을 수 없습니다. admin 전용 수동 복구는 아래 "제어권 lease와 운영자 복구" 절을 따릅니다. 인증 누적 실패는 429, 누락/잘못된 토큰은 401, 역할/로봇 권한 위반은 403입니다.

토큰 해시는 data-dir의 `auth.json`에서 요청 시 핫리로드합니다. 파일 식별자와 ns 시각으로 원자 교체를 감지합니다. 토큰 파일 내용·Authorization 헤더를 로그에 남기지 않습니다.

## 경로

아래 경로는 모두 `/api/v1` 기준입니다.

| 메서드 | 경로 | 권한 | 응답/용도 |
|---|---|---|---|
| GET/HEAD | `/health` | 없음 | 상태/API 버전 |
| GET | `/me` | 조회 역할 | name, role, robot_id |
| GET | `/dashboard` | 조회 역할 | server, robots, missions(최근40), events(최근80) |
| GET | `/robots` | 조회 역할 | `{robots:[...]}` |
| GET | `/robots/{robot_id}` | 조회 역할 | robot 상태 객체 |
| GET | `/robots/{robot_id}/map.png` | 조회 역할 | image/png, `X-Map-Version` |
| GET | `/robots/{robot_id}/map.json` | 조회 역할 | `{version,meta}` |
| GET | `/missions` | 조회 역할 | `{missions:[...]}` |
| GET | `/missions/{mission_id}` | 조회 역할 | mission 객체 |
| GET | `/events` | 조회 역할 | `{events:[...]}` |
| POST | `/missions` | operator/admin | 생성201, 같은 중복키200 |
| POST | `/missions/{mission_id}/cancel` | operator/admin | 202, 최신 mission 객체 |
| POST | `/agent/hello` | 해당 robot agent | 세션 등록, server boot/주기 |
| POST | `/agent/reconcile` | 해당 robot agent | 목표 소유권·저널·핸들 대조 |
| POST | `/agent/state` | 해당 robot agent | 상태 수신 및 최초 명령/동일 ID 전달 |
| POST | `/agent/map` | 해당 robot agent | 지도 저장 |
| POST | `/agent/commands/{command_id}/ack` | 해당 robot agent | 상태/진행 ACK |

`/missions` 쿼리: `robot_id`, `status`, `limit`(기본50, 최대500). `/events`: `since_id`(기본0), `limit`(기본100, 최대500). 이벤트는 id 내림차순이며 since_id보다 큰 항목을 반환합니다. 큰 이벤트 스트림의 무손실 페이지네이션/웹소켓은 구현하지 않았습니다. PUT/DELETE는 405입니다.

robot_id는 `[A-Za-z0-9_.-]{1,64}`, mission_id는 `m_`+12자리 소문자 hex, command_id는 `c_`+12자리 hex입니다. 시각은 별도 설명이 없으면 Unix epoch seconds입니다. 서버/Agent의 시각 동기화가 필요합니다. 일반 POST body 상한은 1MiB, 지도 body는 6MiB입니다.

## 임무 생성과 조회

```json
{
  "robot_id": "mock1",
  "type": "navigate_to_pose",
  "goal": {"frame_id":"map", "x":6.25, "y":2.125, "yaw_rad":0.35},
  "idempotency_key":"caller-stable-task-key",
  "note":"mock verification"
}
```

현재는 navigate_to_pose만 지원합니다. x/y는 필수 유한 수치이고 절댓값 1000m 이하, yaw_rad는 선택(기본0)입니다. frame_id는 map만 허용합니다. idempotency_key는 선택이며 로봇별 1–128자 문자열입니다. 같은 키는 같은 임무를 반환하며 goal이 달라도 기존 임무가 반환되므로 호출자는 동일 요청에만 키를 재사용해야 합니다. 생성 응답에는 `created: true/false`가 추가됩니다.

mission 객체는 다음 필드를 포함합니다.

- mission_id, robot_id, type, goal, status, detail, progress
- idempotency_key, created_by, note, created_at, updated_at
- cancel_requested, terminal
- commands: command_id, type, status, attempts, seq, created_at, sent_at, acked_at, owner_boot_id

progress는 Agent가 제공하며 현재 ROS adapter는 distance_remaining_m, recoveries, navigation_time_s를 보고합니다. ETA나 RMF 스케줄 결과는 아닙니다.

일반 전이:

```text
QUEUED -> DISPATCHED -> ACCEPTED -> EXECUTING -> SUCCEEDED / FAILED
                               \-> REJECTED / CANCELED
QUEUED -> CANCELED (전달 전 취소)
실행/전달 중 -> CANCELING -> CANCELED (실제 Nav2 canceled 결과 확인)
미전달 명령의 TTL 만료 -> EXPIRED
전달/실행/취소/소유권 불명확 -> RECONCILIATION_REQUIRED
```

중간 상태 ACK는 최신 진행 상태로 병합될 수 있어 ACCEPTED가 생략되고 EXECUTING으로 바로 보일 수 있습니다.

SUCCEEDED/FAILED/CANCELED/REJECTED/EXPIRED는 terminal이며 나중의 상충 ACK로 바뀌지 않습니다. RECONCILIATION_REQUIRED는 terminal이 아니고 다음 임무 dispatch를 막습니다. 같은 프로세스의 유효 핸들 대조 또는 영속화된 terminal 결과로만 자동 해소합니다. 증거 없이 강제 해제하는 경로는 없습니다. admin의 증거 기반 복구는 아래 제어권 lease와 운영자 복구 절을 따릅니다.

POST cancel body는 `{}`입니다. 반복 취소는 멱등이며 queued 취소는 Agent 전달 없이 처리합니다. 실행 중 취소의 HTTP 202는 취소 **요청 접수**일 뿐 모터 정지/취소 완료를 뜻하지 않습니다. 취소 실패·만료·목표 핸들 상실은 CANCELED로 가장하지 않고 RECONCILIATION_REQUIRED로 남깁니다. 목표가 먼저 SUCCEEDED/FAILED로 끝나면 해당 결과를 보존합니다.

## 상태 계약

robot 응답에는 robot_id, online, first_seen/last_seen, last_seen_age_s, agent, state, state_age_s, map_version, map_meta, reconciled, reconciliation_required가 있습니다. 서버 재시작 시 저장된 로봇은 OFFLINE·미동기화 상태에서 시작합니다.

현재 state schema_version=1의 주요 키는 pose, map, nav2, battery, lidar, depth_camera, safety_gate, mission, execution, errors, agent_seq, communication입니다.

- pose: 상태/프레임/x/y/yaw_rad/출처/나이. TF map→base_footprint를 우선하며 없으면 /odom을 표시하고 map 위치라고 가장하지 않습니다.
- battery: 실제 LimoStatus 전압만 VOLTAGE_ONLY. 백분율은 null.
- lidar/depth/safety: 기존 토픽의 상태 및 수신 나이. 데이터가 없으면 NO_DATA/UNKNOWN.
- safety_gate.motor_path: 기본/실차는 /cmd_vel publisher가 있으면 ARMED, /cmd_vel 없이 /cmd_vel_locked publisher가 있으면 LOCKED, 아니면 UNKNOWN. 명시적 sim 조건을 모두 만족한 단일 Safety Gate endpoint만 SIM_GATED입니다(아래 절).
- execution.active: 실제 현 프로세스가 유지하는 mission_id, command_id, goal_handle_available. 실행 중 핸들 소유권 판정에 쓰며 이전 임무 표시용 mission과 구분합니다.
- communication: connected, last_success, failures, reconnects, retry_delay_s. 단절 중 서버의 마지막 state는 과거 표본이므로 online/수신 나이도 같이 확인합니다.

## Agent 연결·복구 순서

### 1. hello

```json
{"robot_id":"mock1", "agent":{"protocol_version":2,"boot_id":"unique-process-boot","mode":"mock","simulated":true,"missions_enabled":true}}
```

Agent boot_id는 프로세스마다 생성하며 실행 중 재연결에는 동일 값을 유지합니다. 응답은 server_time, server_boot_id, heartbeat_s, offline_after_s, protocol_version입니다. 새 세션은 이전 세션을 퇴역시키며 이전 boot_id의 재등록을 409로 거부합니다. 오래된 Agent에 새 프로토콜을 위장해 부여하면 안 됩니다. protocol_version 2가 없는 Agent도 상태 등록 자체는 가능하지만 reconcile 전 명령은 전달하지 않습니다.

### 2. reconcile

```json
{
  "robot_id":"mock1", "agent_boot_id":"unique-process-boot", "journal_ok":true,
  "journal": {
    "c_example": {"mission_id":"m_example", "type":"navigate_to_pose", "seq":12,
                  "status":"EXECUTING", "detail":"navigating", "t":0}
  },
  "execution": {
    "active":{"mission_id":"m_example","command_id":"c_example","goal_handle_available":true},
    "pending_cancel_ids":[]
  }
}
```

위 ID는 형식 설명용 자리표시자입니다. 실제 ID는 서버가 생성합니다. journal은 Agent 로컬의 영속 명령 기록이며 token을 포함하지 않습니다. 응답은 ok, server_boot_id, commands(command_id/status 목록), reconciliation_required입니다. 이 응답은 새로운 목표 지시가 아닙니다.

- 같은 boot의 소유 command/mission과 실제 goal handle이 일치하면 기존 실행 상태를 복원합니다.
- Agent 재시작 후 잃은 핸들을 재생성하기 위해 NavigateToPose를 다시 보내지 않습니다.
- 영속화한 terminal 결과는 ACK 유실/재시작 뒤에 동기화할 수 있습니다.
- 전달을 시도했으나 수신/실행을 입증할 수 없는 명령, 소유권 불일치, 손상 저널은 보수적으로 차단합니다.
- PENDING이면서 attempts=0인 명령만 재동기화 이후 TTL 내에서 최초 전달할 수 있습니다.

### 3. state 및 명령

```json
{"robot_id":"mock1", "agent_boot_id":"unique-process-boot", "seq":1,
 "state":{"schema_version":1,"execution":{"active":null,"pending_cancel_ids":[]}}}
```

seq는 같은 boot에서 단조 증가합니다. 응답은 ok, server_time, server_boot_id, commands이며 필요한 경우 needs_reconcile=true입니다. server boot 변경/needs_reconcile을 받으면 Agent는 해당 응답의 명령을 실행하지 않고 hello/reconcile부터 다시 시작합니다.

명령은 command_id, robot_id, mission_id, type, seq, attempt, payload, issued_at, expires_at, owner_boot_id입니다. navigate_to_pose payload는 `{goal:{frame_id,x,y,yaw_rad}}`, cancel_mission payload는 `{mission_id}`입니다. 서버의 미확인 정상 전달 재시도는 같은 command_id를 사용합니다. 단절 후 미확인 목표를 새로운 ID로 재전송하지 않습니다.

Agent는 소유 robot/boot, TTL·발급 경과·영속 seq, 기존 command_id를 검사하고 수신 기록을 저장한 뒤에만 핸들러를 부릅니다. Nav2 제출 직전에도 연결·기한 및 기존 모터 잠금 인터록을 확인합니다. 잘못된 소유권은 로컬에서 실행하지 않습니다.

### 4. ACK

```json
{"robot_id":"mock1", "agent_boot_id":"unique-process-boot",
 "status":"EXECUTING", "detail":"navigating", "progress":{"distance_remaining_m":1.2}}
```

status는 ACCEPTED, EXECUTING, SUCCEEDED, FAILED, CANCELED, REJECTED, EXPIRED, RECONCILIATION_REQUIRED입니다. 응답은 `{ok,duplicate,command_status}`입니다. 회귀 ACK/이미 확정된 상충 terminal ACK는 무시합니다. RECONCILIATION_REQUIRED 상태를 단순 ACCEPTED/EXECUTING ACK로 덮어쓸 수 없고, 핸들 대조를 먼저 통과해야 합니다. ACK의 robot/boot와 명령의 owner_boot_id가 다르면 409입니다.

취소 명령의 SUCCEEDED는 같은 목표의 실제 terminal STATUS_CANCELED를 확인했다는 의미입니다. cancel 서비스의 return_code=0과 goals_canceling UUID 일치만으로는 SUCCEEDED를 보내지 않습니다.

### 5. map

body는 robot_id, agent_boot_id, version(integer), meta(object), png_base64입니다. meta에는 width/height/resolution/origin_x/origin_y/origin_yaw/frame_id 및 셀 통계가 포함됩니다. PNG는 free=255, occupied=0, unknown=128, 첫 행은 위쪽(+y)입니다. 서버는 저장/전달하며 지도를 계산하거나 검증된 SLAM이라고 선언하지 않습니다. 현재 UI의 지도 좌표 표시에는 회전된 map origin 보정이 없으므로 nonzero origin_yaw는 추가 검증이 필요합니다.

## 오류·제한

오류 형식: `{"error":{"code":"...","message":"..."}}`. 대표 상태는 400 입력 오류, 401 인증, 403 역할, 404 미등록 robot/mission/command/지도, 405 메서드, 409 세션·소유권·오래된 상태, 413 body 상한, 429 인증 실패 제한입니다. 일반 HTTP 상태 외 내부 예외는 500이며, 이 구현은 모든 잘못된 JSON 타입의 정규화를 보장하지 않습니다.

SQLite는 ID·임무·명령·이벤트를 보존합니다. ACK와 mission 갱신 사이 중단은 재동기화 시 저장된 terminal 명령 결과로 보완합니다. 이는 분산 exactly-once/물리적 정지 보장 또는 장시간 장애 내구성 인증이 아닙니다. 서버 DB와 Agent 저널을 함께 보존해야 하며 저널을 지워 목표를 재전송하는 방식은 지원하지 않습니다.

Open-RMF Fleet Adapter(`rmf_adapter/`)가 이 HTTP 계약의 호출자로 모의 Nav2 및 짧은 Gazebo 프로파일에서 검증되었습니다(`rmf_adapter/docs/RMF.md`). Safety Gate를 우회하는 API나 속도 명령 API는 없습니다.

## 제어권 lease와 운영자 복구 (additive)

lease가 없으면 기존 동작과 같습니다. 모든 판단은 서버의 단일 락 안에서 원자적으로 수행됩니다(소유권 확인과 임무 생성 사이에 틈이 없음).

| 메서드 | 경로 | 권한 | 용도 |
|---|---|---|---|
| GET | `/robots/{id}/lock` | 조회 | `{lease: {owner, epoch, expires_at, remaining_s, ...} \| null}` |
| POST | `/robots/{id}/lock` | operator/admin | body `{action: acquire\|renew\|release\|force_release, ttl_s(5–300, 기본 20), note}` |
| POST | `/missions/{id}/recover` | **admin** | `{note}` (10자 이상). RECONCILIATION_REQUIRED 임무 복구 |
| POST | `/robots/{id}/recover` | **admin** | `{note}`. 로봇 수준 reconciliation block 해제 |

- 소유자는 인증된 토큰 **이름**입니다. 유효한 lease가 있으면 소유자가 아닌 주체의 `POST /missions`는 큐에 넣지 않고 **HTTP 409 `ownership_conflict`**. 같은 `idempotency_key`의 기존 임무 재조회(200)는 호환을 위해 유지, 취소는 모든 제어 역할에 허용(정지는 안전한 방향).
- `acquire` 409: `lock_held`(다른 소유자의 유효 lease), `robot_busy`(다른 주체의 QUEUED/진행 임무 존재), `reconciliation_required`. `renew` 409 `lock_not_held`(만료·해제됨: 다시 acquire). lease는 갱신이 없으면 만료되며 진행 중인 임무에는 영향이 없습니다. `force_release`는 admin 전용, 이벤트 기록.
- 복구는 **Agent가 측정한 증거**가 있을 때만: 최근 5 s 이내 state의 `execution.nav2_goals`(`navigate_to_pose/_action/status` 기반 `known:true`, `active_count:0`)와 `execution.active:null`, 로봇 online·reconciled. 실패 시 409 `goal_still_active` / `no_evidence` / `stale_state` / `robot_not_ready`. 증거가 없으면 새 이동 임무는 계속 차단됩니다.
- 복구된 임무는 SUCCEEDED/CANCELED가 **아니라** `FAILED` + detail `RECOVERED_UNVERIFIED_OUTCOME`이며 복구 기록(누가·언제·메모·증거)이 이벤트에 남습니다. 서버는 로봇에 아무 명령도 보내지 않습니다. 이후 대기 중인 임무는 정상 순서로 dispatch됩니다.
- `POST /agent/state` 응답의 `recovered_commands`(복구된 command_id, 최대 20개): Agent는 **자체 측정**(목표 핸들 없음, `active_count==0`)이 일치할 때만 저널의 해당 명령을 FAILED(미확인)로 닫습니다. 일치하지 않으면 계속 차단합니다. Agent가 이전 미확정 명령 때문에 새 목표를 거부할 때는 `REJECTED`(`BLOCKED_BY_UNRESOLVED_COMMAND`, 목표를 보내지 않았음이 확실)로 응답합니다.


## Gazebo 전용 Agent 인터록 (HTTP 계약 변경 없음)

`allow_sim_safety_gate`는 ROS 파라미터이며 HTTP로 변경할 수 없습니다. 기본 false입니다. `mode=sim` + `simulated=true` + 명시적 opt-in + 실제 Agent의 `use_sim_time=true`이고 `/cmd_vel` endpoint가 정확히 하나의 `/stall_detector_node`일 때만 SIM_GATED 예외입니다. namespace가 다르거나 중복/다른 publisher가 있거나 조회가 실패하면 거부합니다. 실차/기본 거부와 `allow_missions=false`는 유지합니다. 노드 이름 검사는 보안 인증이 아니며 실차 허용 용도로 사용하지 않습니다.

별도 속도/직접 Nav2 HTTP API는 추가하지 않았습니다. 모든 Gazebo 목표도 인증·lease·멱등성·reconcile을 거친 기존 `/missions` 계약을 사용합니다. [실행 구성](../gazebo/README.md), [검사 결과](../gazebo/RESULTS.md)를 참고하세요.

## 웹 Mapping 세션 · Map Registry 영역/계보 (2026-11-19, additive)

- `GET /robots/{id}/mapping`: 세션 상태(`IDLE|STARTING|MAPPING|PAUSING|PAUSED|SAVING|STOPPING|STOPPED|UNKNOWN`), 마지막 명령, Worker 증거, `registered`, `plan`.
- `POST /robots/{id}/mapping` (operator·admin, 202=접수만 의미): `{action: START|PAUSE|RESUME|SAVE|STOP, epoch, idempotency_key, session_id?}`.
  - START 옵션: `mode` (`NEW_MAP` | `CONTINUE_MAP`+`parent_map_version_id`), `geofence`, `limits`.
  - SAVE 옵션: `floor_id`, `area_id`.
  - 오류 코드: `lease_required`, `operation_state_conflict`, `session_conflict`, `idempotency_conflict`, `nav_terminal_unproven`, `foreign_goal_present`, `destination_pending`, `manual_active`, `slam_session_active`, `worker_unavailable`, `mapping_real_blocked`, `bad_mode`, `bad_limits`, `bad_geofence`, `parent_required`, `parent_not_found`, `parent_without_posegraph`, `continue_unsupported`, `localization_unconfirmed`, `invalid_floor`, `invalid_area`.
- `POST /missions`, SIM `slam/start`: 세션이 IDLE/STOPPED가 아니면 `409 mapping_active`(Safety Context 검사가 먼저 실패하면 `safety_not_ready`).
- Agent: `POST /agent/operation` (`poll|accept|result|reserve_goal`), `POST /agent/mapping-map` (수락된 SAVE bundle 등록, ≤48 MiB).
- Map Registry 응답의 map version에 `area`, `lineage` 필드 추가(`/maps`, `/maps/{id}`, `/maps/{id}/meta`). `/facilities`의 층에 `areas[]`.
- 상세: [WEB_MAPPING_20261119.md](WEB_MAPPING_20261119.md).

## Map Registry 저작 API 추가 (2026-11-20)

- `POST /api/v1/map-registry/areas` — operator/admin. 본문 `{area_id, floor_id, name, coverage: PARTIAL_FLOOR|FULL_FLOOR, description?}`. 201 `{ok, area}`. 같은 속성 재요청은 멱등, 다른 속성은 409, 없는 층 404, 잘못된 값 400, viewer 403. `Idempotency-Key` 지원.
- `POST /api/v1/map-registry/maps` 추가 선택 필드: `area_id`(같은 층의 영역), `lineage{mapping_mode: NEW_MAP|CONTINUE_MAP|IMPORTED, parent_map_version_id?, session_id?}`, `note`(≤500), `threshold_policy: reject(기본)|derive`. 영역·계보 검증은 저장 전에 하며 실패하면 아무것도 저장하지 않는다. 응답에 `area_id`, `lineage`, `yaml_adjustment`.
- 임계값 안전 검사: trinary PGM(0/205/254)에서 `free_thresh`/`occupied_thresh`가 unknown 픽셀을 free/occupied로 읽게 하면 422 `policy_violation`(`detail.code=unknown_pixels_misclassified`, `fixable_by_free_thresh`, `derived_free_thresh`). `derive`는 `negate: 0`이고 free 쪽 문제일 때만 가능하며 원본은 보존, 보정본 YAML만 등록하고 증거에 원본·보정본 sha256과 사유를 기록한다. 클라이언트 `expected_checksums`는 원본 기준으로 먼저 검증한다.
- 검증·활성화·복원 라우트는 여전히 없다(404).


## Map save (web "save current SLAM map and register", 2026-11-20)
Details: `WEB_MAP_SAVE_20261120.md`.
- `GET /api/v1/robots/{id}/map-saves` (any role): `{capability:{available,reason}, saves:[...]}`; each save has `status`, `stage`, `error_code`, `detail`, `manifest` (per-file size/sha256/save window), `consistency`, `result.map_version_id`.
- `POST /api/v1/robots/{id}/map-save` (operator, Lease owner+epoch): `{epoch, idempotency_key, floor_id, area_id?, label, note?, threshold_policy?: "reject"|"derive"}` -> 202 QUEUED. Errors: `lease_required`, `map_save_unavailable` (Agent reason), `save_in_progress`, `policy_violation`, validation errors. Same `idempotency_key` returns the same save (`duplicate: true`).
- `POST /api/v1/agent/map-save` (Agent token only): actions `poll|accept|stage|fail|bundle`. The server re-verifies per-file size+SHA-256 (`transfer_corrupt` 422 is retryable) and registers the map as UNVERIFIED (`source_mode=SLAM_SAVED_BUNDLE`).

## 추가 Agent 경로 (R1.3, 2026-11-22)

아래 3개 경로는 모두 `POST`, `Authorization: Bearer <agent-token>`(본문 `robot_id`와 일치하는 Agent 토큰, 불일치 시 403) 및 활성 세션 검증(`agent_boot_id` 일치, 불일치 시 409 `agent_session_mismatch`)을 요구합니다.

- `POST /api/v1/agent/map-bundle` (body 상한 1 MiB, 구현: `http_api.py` → `navigation.py::agent_map_bundle`):
  - 요청: `{"robot_id", "agent_boot_id", "map_version_id", "file": "map.yaml"|"map.pgm"}`.
  - 동작/응답(200): 현재 `NAVIGATION` 모드(또는 `NAVIGATION` 종료 중인 `TRANSITIONING`)에서 서버가 요청 중인 `navigation_request.map_version_id`와 일치할 때만 Map Registry 체크섬 검증 후 파일 반환: `{"map_version_id", "file", "data_b64"}`.
  - 주요 오류: 400 `bad_robot_id`, 403 `forbidden`, 405 `method_not_allowed`, 409 `agent_session_mismatch` / `not_requested`.
- `POST /api/v1/agent/slam-map` (body 상한 6 MiB, 동시 업로드 세마포어 2, 구현: `http_api.py::_agent_slam_map`):
  - 요청: `{"robot_id", "agent_boot_id", "command_id", "session_id", "floor_id", "yaml_b64", "pgm_b64", "posegraph_b64"?, "posegraph_data_b64"?}`.
  - 동작/응답(200): 해당 `agent_boot_id` 소유의 활성 `slam_save`(또는 `manual_stop: true`인 `slam_stop`, 상태 `SENT|ACCEPTED|EXECUTING`) 명령과 `session_id`/`floor_id`가 일치할 때 Map Registry에 `source_mode="SLAM_LIVE_SNAPSHOT"`으로 등록하고 무결성 검증 후 반환: `{"map_version", "created", "integrity"}`.
  - 주요 오류: 400 `bad_json` / `bad_robot_id` / `invalid_map` / `map_registry_rejected`, 403 `forbidden` / `invalid_save_command`, 404 `registry_unavailable`, 405 `method_not_allowed`, 409 `agent_session_mismatch` / `session_mismatch`, 413 `too_large`, 500 `registry_integrity_failed`, 503 `busy`.
- `POST /api/v1/agent/camera` (G2 2026-11-23, origin/main 기능; 구현: `http_api.py` → `service.py::ingest_camera`):
  - 요청: `{"robot_id", "agent_boot_id", "kind": "rgb"|"depth", "jpeg_base64", "source_age_s"}` (base64 ≤ 350,000자, 0 ≤ `source_age_s` < 2).
  - 동작/응답(200 `{"ok": true}`): 현재 세션(`check_session`)이고 hello에서 `camera_preview: true`를 알린 Agent의 JPEG만 메모리에 보관합니다. 운동 권한과 무관한 조회 전용 경로입니다.
  - 주요 오류: 400 `bad_camera_frame`, 403 `forbidden`, 409 `agent_session_mismatch` / `camera_disabled`.
  - 조회: `GET /api/v1/robots/{id}/camera/{rgb|depth}.jpg` (404 `camera_unavailable`: 프레임 없음·로봇 offline), `GET /api/v1/robots/{id}/manual-bench` (`manual_preview_status`: bench 활성 여부).
- `POST /api/v1/robots/{id}/manual-bench` (운영자 Bench, v5):
  - `start`: `{"action":"start","epoch":7}` → `{"session_id":"..."}`.
  - `sample`: 현재 `epoch`·`session_id`, `input_format:"normalized_axes_v1"`, 세션 안에서 증가하는 정수 `seq`, Boolean `deadman`, 유한한 `linear`·`turn` 축 값(각 −1~1)을 보냅니다. 응답은 `{"received":seq}`입니다. 이전 main의 ticket 및 m/s 입력 형식은 `400 bad_joy_format`으로 거부합니다.
  - `stop`: 현재 `epoch`·`session_id`와 함께 보내면 활성 세션을 폐기하고 `{"stopped":true}`를 반환합니다. 샘플은 250 ms 내 만료됩니다. Deadman 해제·권한 변경은 이전 입력을 무효화합니다.
  - Agent의 수동 속도 상한은 전진 0.10 m/s 및 회전 ±0.30 rad/s입니다. Mock은 잠금 Arbiter와 Watchdog을 사용하고 실차 Bench는 관측만 합니다.
- `POST /api/v1/agent/manual-bench` (body 상한 1 MiB, 구현: `http_api.py` → `service.py::pull_manual_bench`):
  - 요청: `{"robot_id", "agent_boot_id"}`.
  - 동작/응답(200): `_manual_bench_robot` 조건(`manual_web_bench: true`, `real`+`missions_enabled: false` 또는 `locked_mock`, online/reconciled/신선 상태, 임무·매핑 비활성, Nav2 idle 증거)과 현재 Lease·`MANUAL` 모드·TTL을 만족하면 최신 조이스틱 샘플을 반환하고, 활성 샘플이 없거나 모드가 `MANUAL`이 아니면 `null`을 반환: `{"sample": {"linear", "turn", "deadman", "session_id", "seq", "ttl_s"} | null}`.
  - 주요 오류: 400 `bad_robot_id`, 403 `forbidden`, 405 `method_not_allowed`, 409 `agent_session_mismatch` / `manual_bench_unavailable` / `robot_busy` / `mapping_unknown` / `mapping_active_or_unknown` / `nav_status_unknown_or_busy` / `mock_arbiter_not_ready`.

