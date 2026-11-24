# 속도 제한: Arbiter Clamp와 Server–Agent API (G2, 2026-11-23, 구현·격리 검증, 미커밋)

속도 제한은 **모터 명령이 아니다.** 웹이 요청한 값은 Agent가 "검증된 설정 상한 이하의 더 낮은 값"으로만 받아들이고, Arbiter가 최종 속도에서 자른다. Agent가 스스로 값을 올리지 않는다.

## 1. 기존 속도 경로와 상한 (조사 결과, 재사용)
| 경로 | 상한(설정값) | 위치 |
|---|---|---|
| MANUAL | 선속도 0…0.10 m/s(전진만), 각속도 ±0.30 rad/s | `velocity_arbiter.select_locked` |
| AUTO(NAV2: Mapping·Navigation) | 전진 0.15 m/s, **후진 −0.10 m/s(별도)**, 각속도 ±0.50 rad/s | `velocity_arbiter.NAV_LINEAR`, `NAV_TURN`; Nav2 `integrated_nav2_lidar_only_slow_params.yaml` |
| Gate 입력 검증(안전망) | 0.20 m/s, 0.60 rad/s | `limo_fleet_bringup/safety.py`, 실차 launch |
이 값은 설정값이며 **실차에서 검증된 값이 아니다.** 기본 launch는 `speed_profile:=normal`이라 위 slow 프로파일 파일을 쓰지 않는다(격리 bringup `navigation.py`는 0.10/0.30).

**가감속**: Nav2 `velocity_smoother`의 `max_accel/max_decel`(Humble 기본 2.5 m/s²·3.2 rad/s²; slow 프로파일은 이 값을 따로 정하지 않음)이 유일하다. 0.15 m/s 대비 사실상 즉시 변화다. MANUAL 경로에는 가감속 제한이 없다(Deadman TTL 250 ms로만 감쇠). Gate·Watchdog에도 slew 제한이 없다. **이번 변경은 가감속을 추가하지 않았다.** 후속 과제.

## 2. Arbiter Clamp (구현)
- 위치: `VelocityArbiter.update()`에서 **소스를 선택한 뒤 `/fleet/nav_cmd_vel`(Watchdog 입력)로 publish하기 직전**. 선택 조건(Lease, Watchdog, Gate, 그래프 등)이 하나라도 실패하면 기존대로 0이다.
- 규칙: 축별 포화(선속도와 각속도를 따로 자름, 비례 축소 아님). 0은 그대로 0(STOP 보존). 비유한·bool·문자열은 0. `min(ceiling, applied)`이므로 ceiling을 넘지 않는다. 자동 Nav2 후진은 `min(0.10, 적용된 전진 한도)`. MANUAL은 전진만 유지.
- 범위(scope): `manual`과 `auto` 별도. 모드 전환은 값을 바꾸지 않는다(각 scope의 값이 유지되고, 전환 시 세대(generation)·표본은 기존대로 초기화).
- clamp 코드가 예외를 내면 0을 보낸다.
- **적용 지점**: mock/simulated Agent는 Arbiter(`enforced_by: "ARBITER"`). 실차(real) Agent에는 Arbiter가 없으므로(`VelocityArbiter`는 mock·simulated에서만 생성) **Motion Watchdog의 clamp**가 적용 지점이다(`enforced_by: "WATCHDOG"`, `WATCHDOG_SPEED_CLAMP_20261123.md`). Agent는 Watchdog이 자기 상태로 "clamp 켜짐·동기화됨·이 Agent boot와 일치·게시자 1개·결함 없음"을 보고하는 동안에만 `WATCHDOG`을 표시하고, 아니면 `null`이다(서버는 `409 agent_cannot_enforce`).
- **Watchdog 경로의 적용 확인**: Agent는 요청을 검증한 뒤 `PENDING_ENFORCEMENT`로 두고 Watchdog에 후보를 보낸다. **Watchdog 상태가 같은 generation·request_id·값을 보고한 뒤에만** `APPLIED`(서버 `approved=true`)가 된다. 10 s 안에 확인이 없으면 `REJECTED/WATCHDOG_NO_CONFIRMATION`.

## 3. API 계약
### `POST /api/v1/robots/{id}/speed-limits` → `202`
요청:
```json
{"lease_epoch": 5, "agent_boot_id": "…", "protocol_version": 2, "safety_contract_version": 1,
 "scope": "auto", "max_linear_mps": 0.12, "max_angular_rps": 0.35,
 "client_request_id": "optional ≤64", "expected_generation": 3}
```
`scope` 생략 시 `auto`. `client_request_id`와 `expected_generation`은 선택.
거부(모두 감사 이벤트 `speed_limit_refused`, 큐에 넣지 않음):
| 코드 | HTTP | 조건 |
|---|---|---|
| `not_lease_owner` | 403 | 현재 Lease 소유자가 아님(Lease 없음 포함) |
| `lease_epoch_mismatch` | 409 | `lease_epoch` 없음·다름 |
| `agent_protocol` | 409 | Agent 오프라인/계약 무효, `protocol_version≠2`, `safety_contract_version≠1` |
| `agent_boot_mismatch` | 409 | `agent_boot_id`가 연결된 Agent의 boot와 다름 |
| `agent_cannot_enforce` | 409 | Agent가 enforcement point(`ARBITER` 또는 동기화된 `WATCHDOG`)를 보고하지 않음 |
| `agent_ceiling_unknown` | 409 | Agent가 ceiling을 보고하지 않음 |
| `bad_scope`, `bad_value`, `bad_client_request_id` | 400 | 형식(비숫자·bool·비유한·0 이하) |
| `speed_exceeds_ceiling` | 422 | Agent ceiling 또는 서버 상한(manual 0.10/0.30, auto 0.15/0.50) 초과 |
| `speed_below_minimum` | 422 | 0.01 m/s 또는 0.02 rad/s 미만(정지는 STOP) |
| `stale_generation` | 409 | `expected_generation`이 현재 generation과 다름 |
| `idempotency_conflict` | 409 | 같은 `client_request_id`에 다른 값 |

### 응답(POST·GET 공통) — UI 계약
| 필드 | 의미 |
|---|---|
| `generation` | 서버가 부여한 단조 증가 번호(서버 재시작 후에도 유지, 로봇별) |
| `request_id`, `status` | `PENDING` / `APPLIED` / `REJECTED` / `EXPIRED` / `SUPERSEDED` / `NONE` |
| `approved` | **Agent가 같은 generation을 적용했다고 보고했고 stale이 아닐 때만 true.** POST 직후는 항상 false |
| `reason` | `PENDING_AGENT_APPLY`, `APPLIED`, Agent 거부 사유(`EXCEEDS_CEILING`, `STALE_GENERATION`, `LEASE_OR_SERVER_BOOT_MISMATCH` …), `BINDING_CHANGED`, `AGENT_NO_CONFIRMATION`, `NO_ENFORCEMENT_POINT`, `NEWER_REQUEST`, `NO_REQUEST` |
| `scope`, `requested` | 요청한 scope와 `{max_linear_mps, max_angular_rps}` |
| `applied` | Agent가 보고한 적용값 `{linear_mps, angular_rps, generation, lease_epoch, age_s}` 또는 `null` |
| `ceiling` | Agent가 보고한 scope별 상한(`manual`, `auto` 각각 linear/angular, auto는 `reverse_mps`) |
| `effective` | 지금 실제로 자르는 값(`stale`이어도 낮은 값 유지) |
| `stale`, `stale_reason` | 적용값이 Lease·서버 boot·계약 세대 변경 또는 Agent 재시작으로 더는 유효한 설정이 아님(자동 복원 없음) |
| `source` | 값의 출처 설명(Agent Arbiter clamp, 설정값, 실차 미검증) |
| `requested_at`, `resolved_at`, `agent_report_age_s` | 시각(epoch s)과 Agent 보고 경과 시간 |
관측 속도는 `robot.state.velocity`(`/odom` twist 기반 추정, 독립 측정 아님, `status` OK/STALE/UNAVAILABLE)이며 이 API의 일부가 아니다.

### 흐름과 의미
1. POST → `202`, `PENDING`, `approved:false`. 서버가 요청을 받았다는 뜻일 뿐이다.
2. 서버가 같은 요청을 상태 응답의 `speed_limit_request`로 Agent에 재전송(세대별 멱등)한다.
3. Agent(Arbiter)가 자기 세션(Lease 에포크·서버 boot·Agent boot·계약)으로 다시 검증해 적용하고, 다음 상태 보고에서 `speed_limits.last_result`로 결과를 보고한다.
4. 서버가 같은 `request_id`/generation의 보고를 보면 `APPLIED`, 거부면 `REJECTED`. 20 s 안에 확인이 없거나 Lease·서버 boot·Agent boot가 바뀌면 `EXPIRED`. 새 요청이 오면 이전 PENDING은 `SUPERSEDED`.
5. `GET`은 항상 최신 상태를 계산해 돌려준다. **UI는 POST 응답이 아니라 GET(또는 `state.speed_limits`)을 폴링해 `approved`를 확인해야 한다.**

### 오래된 설정 처리(자동 복원 없음)
- Lease 상실·에포크 변경·서버 boot·계약 세대 변경·연결 끊김: Agent가 설정을 `stale`로 표시하되 **낮은 값은 계속 적용**한다. 새 Lease에서 새 요청이 와야만 값이 바뀐다(올릴 수도 있음, ceiling까지).
- Agent 재시작: 저장된 낮은 값을 `stale`로 불러와 계속 적용한다(파일 `speed_limits.json`, 상태 디렉터리). 파일이 손상·범위 밖이면 더 낮은 SAFE_FALLBACK(0.05 m/s, 0.15 rad/s) 적용. 저장된 값이 없으면 기존 ceiling.
- generation 비교는 같은 (서버 boot, Lease 에포크, 계약 세대) 안에서만 한다. 같은 묶음에서 작거나 같은 generation은 `STALE_GENERATION`으로 거부. 같은 delivery를 다시 받으면 한 번만 처리(`request_id`).

## 4. Nav2 속도 제한 검토
- **`velocity_smoother`의 `max_velocity`를 런타임에 바꾸는 방식은 효과가 없다**(Humble, 격리 ROS 실측: 설정은 "성공", 출력 최댓값 0.150 유지; `p27_nav2_smoother_dynamic_param.log`). 이 방식으로 제한 완료를 주장하지 않는다.
- **`nav2_msgs/SpeedLimit`**: 인터페이스는 존재(`header, bool percentage, float64 speed_limit`). 실제 실차 설정의 `controller_server`(FollowPath = RegulatedPurePursuit)는 격리 ROS에서 configure한 뒤 `/speed_limit`(`speed_limit_topic` 기본값)을 **구독**하는 것을 확인했다(`p30_nav2_speedlimit_check.log`). 그러나 **효과(출력 속도가 실제로 줄어드는지)는 측정하지 않았다.** RPP의 `setSpeedLimit`은 선속도에 적용되는 것으로 알려져 있고 각속도는 아닐 수 있으며, 소스 없이 확인하지 못했다.
- 따라서 Nav2 SpeedLimit은 구현하지 않았고, **Arbiter clamp를 유일한 강제 지점으로 둔다.** 추후 SpeedLimit을 보조로 추가하더라도 clamp는 유지한다.
- 격리 스택 Mapping 중 Nav2/Frontier 출력(248개 비영 표본)이 auto 한도 0.06/0.20에서 정확히 포화됐다(Gate 출력 최댓값 0.060/0.200). 한도 없을 때의 Nav2 출력 기준선은 같은 실행에서 측정하지 않았다(한도 요청 후 Mapping을 시작).

## 5. 검증 수준
격리 ROS·mock Agent만(실차 없음, 모터 구동 없음). 결과와 로그는 `VALIDATION.md`(38) 이후 항목과 `local_data/raw_results/real_limo_integ_claude_20261122/field_ready_01/p29*`, `p30*`, `p31*`, `p32*`.

## 6. Phase C 개정 (2026-11-24): 응답 필드와 Watchdog 증거
- 응답에 추가: `agent_boot_id`(요청 기록의 Agent boot), `watchdog_boot_id`, `watchdog_evidence`(WATCHDOG 경로), `applied.scope`·`applied.request_id`.
- `watchdog_evidence`: Agent가 Watchdog 상태를 **데이터로 복사**한 값(`watchdog_boot_id`, `agent_boot_id`, `output_gid`, `synced`, `publisher_ok`, `fault`, `enabled`, `evidence_age_s`, `applied{scope: generation, request_id, linear_mps, angular_rps, stale}`, `agent_verified`)에 **서버가 계산한** `server_verified`(작동하는 enforcement point인가: Agent boot 일치, boot id·GID 존재, 켜짐·동기화·게시자 정상·결함 없음, 나이 ≤ 2 s)와 `applied_match`(현재 기록이 Watchdog 증거와 일치하는가), `server_checks`를 붙인 것. 콘솔 상태 `state.speed_limits.watchdog_evidence`에도 같은 서버 판정이 들어간다.
- `approved`(WATCHDOG): Agent `APPLIED` 보고 + `stale=false` + enforcement point 보고 + `server_verified` + `applied_match`. 하나라도 아니면 false(`PENDING`에 머물다 20 s 후 `EXPIRED/WATCHDOG_EVIDENCE_MISSING_OR_MISMATCH`).
- DDS writer GID와 노드 이름은 그래프 식별자이며 인증이 아니다(SROS2 없음).
- 시간: Agent→Watchdog 확인 10 s, 서버 만료 20 s, UI 폴링 25 s. 서버 장애로 상태 수신이 실패해도 속도 제한 처리는 상태 수신을 깨뜨리지 않는다(오류 이벤트만 남기고 요청은 만료).
