> **대체됨 (2026-11-23 후반)**: 이 제안은 구현되어 `SPEED_LIMIT_API_20261123.md`(계약)와 `SPEED_LIMIT_UI_INTEGRATION_20261123.md`(UI 연결)가 우선한다. 경로는 `speed-limits`, 응답 필드는 그쪽을 따른다. 아래 본문은 경위로 남긴다.

# 제안: 속도 설정 서버·Agent 계약 (Antigravity UI 연결용, 미구현)

상태: **제안만**. 서버·Agent 코드와 웹 파일(`fleet_control/server/static/*`)은 이 문서로 바뀌지 않았다. 웹 지도·속도 UI는 Antigravity가 담당하고, 이 문서는 UI가 붙을 계약을 기존 안전 구조 기준으로 정의한다. 병합은 사용자 승인 후 각 담당이 한다.

## 1. 현재 사실 (코드 확인)
- 수동(MANUAL): Arbiter가 웹 bench 표본을 0..0.10 m/s, ±0.30 rad/s로 제한한다(`velocity_arbiter.py` 머리 주석).
- 자율: `navigation.py`의 `velocity_smoother` `max_velocity [.10, 0, .30]`.
- Gate 입력 검증 한계: 0.20 m/s, 0.60 rad/s(`safety.py`, 실차 launch). 이는 상한 위의 안전망이다.
- 웹/서버에는 속도 설정 API가 없다. 웹은 `/cmd_vel`을 발행하지 않으며 앞으로도 그래서는 안 된다.

## 2. 원칙
1. 속도는 **낮추는 방향으로만** 웹에서 바꾼다. 상한(ceiling)은 Agent 설정이며 서버·웹이 올릴 수 없다. 현재 실차 상한(수동 0.10/0.30, 자율 0.10/0.30)은 별도 실차 검증 전까지 그대로다.
2. 적용 경로: 웹 → 서버(Lease 보유자 검사) → Agent(상태 응답으로 전달) → Arbiter의 clamp(수동)·Nav2 파라미터(자율). Arbiter→Watchdog→Gate 체인은 변경하지 않는다.
3. Agent가 최종 `applied` 값을 계산하고 상태로 되돌린다. 웹은 `requested`가 아니라 `applied`를 표시한다.
4. 값이 오래되었거나(`age_s > 3`) Agent 오프라인이면 STALE/UNAVAILABLE로 표시한다.

## 3. 서버 API 제안
`PUT /api/v1/robots/{id}/speed_profile`
```json
{"lease_epoch": 7, "manual": {"max_linear": 0.08, "max_angular": 0.25}, "navigation": {"max_linear": 0.08, "max_angular": 0.25}}
```
- Lease 보유자만 허용. 계약 세대·세션과 같은 Submission Fence 규칙을 적용한다.
- 거부: 비유한 값, 음수, ceiling 초과(HTTP 422 `speed_exceeds_ceiling`), 계약 세대 불일치, Lease 없음.
- 응답: `{"requested": {...}, "ceiling": {...}, "revision": n}`. `GET .../speed_profile`로 읽기.
- 감사 이벤트 `speed_profile_changed`(주체, 이전/이후 값).

## 4. Agent 계약 제안
- 서버가 상태 응답에 `speed_profile: {revision, manual, navigation}`을 실어 보낸다(기존 `operating_mode`와 같은 방식).
- Arbiter: 수동 표본을 `min(요청, 입력 ceiling, min(applied.manual, ceiling))`으로 clamp. 기존 0.10/0.30 하드 제한은 유지한다.
- Nav2: `velocity_smoother`·`controller_server`의 `max_velocity`를 `ros2 param set`으로 낮출 때 실패하면 `applied`에 반영하지 않고 `speed_profile.apply_error`를 보고한다.
- 상태 보고: `speed_profile: {requested, applied, ceiling, revision, applied_at, apply_error}`.
- 무효·미수신 시 기본값은 ceiling이 아니라 **더 낮은 안전 기본값**(제안: 수동 0.05/0.20, 자율 0.05/0.20)으로 시작하고, 서버가 한 번 응답한 뒤에만 올라간다. Agent 재시작 후에도 같은 규칙이다.

## 5. UI 연결 사항 (Antigravity)
- 슬라이더 최대값은 `ceiling`, 표시값은 `applied`, 변경 중에는 `requested`를 구분해 보인다.
- Lease가 없거나 상태가 STALE이면 슬라이더를 비활성화한다.
- 변경은 `PUT speed_profile`만 사용한다. WASD·조이스틱·게임패드는 기존 bench 경로를 쓰고 속도 값을 직접 보내지 않는다(정규화 축만 전송).
- 지도 위 위치·방향·계획 경로·궤적은 각 항목의 `age_s`를 받아 5초 이상이면 STALE, 없으면 UNAVAILABLE로 표시한다.

## 6. 검증 계획 (구현 후)
단위: 거부 규칙(비유한, ceiling 초과, Lease 없음, 세대 불일치). 격리 ROS: 수동 표본이 `applied`를 넘지 않음, 재시작 후 안전 기본값, Nav2 파라미터 적용 실패 보고. 실차 상한 상향은 별도 실차 검증 이후에만.

## 7. 미결 사항
Nav2 파라미터를 실행 중 바꾸는 방식이 Nav2 Humble 컨트롤러에서 안전한지(컨트롤러 재초기화 필요 여부) 미확인. 서버 권한 모델(Lease 외 운영자 역할)은 기존 정책을 따른다고 가정했으나 확인하지 않았다.

---
# 개정 (2026-11-23, Antigravity 제안 `WEB_MAP_SPEED_INTEGRATION_PROPOSAL_20261123.md` 검토 후)
Antigravity 작업트리(`Capstone1_limo_web_ui_20261123`, 브랜치 `feat/web-map-speed-ui-20261123`)의 문서·`speed_control_panel.js`를 **읽기만** 했다. 그 트리의 파일은 수정하지 않았다. 해당 브랜치는 아직 커밋이 없고(기준 `15bb5c59`, 변경은 작업트리에만 있음), 아래는 통합 전 정합성 검토다.

## R1. 관측 속도 텔레메트리 — Agent에 구현함(G2 작업트리, 미커밋)
- 위치: `ros_adapter.py` `_velocity_telemetry()`, 상태 키 `state.velocity`. UI가 기대하는 형태와 같다: `{status, linear_x_mps, angular_z_rps, source, age_s}`.
- **출처**: `/odom`의 `twist.twist`(드라이버가 보고한 바퀴 오도메트리). 독립 측정이 아니고, 명령값·실제 모터 움직임의 증거도 아니다. 모터 잠금 상태에서는 0으로 읽힌다. `source` 문자열에 이를 적었다.
- **유효성**: 최신(≤1.0 s)이고 유한하면 `OK`, 오래되면 `STALE`(값은 유지, `age_s` 표시), 수신 전·비유한이면 `UNAVAILABLE`(값 `null`).
- 시각: `age_s`는 Agent가 `/odom`을 받은 후 경과 시간(Agent 단조 시계). 센서 헤더 시각이 아니다.
- 시험: `test_agent_motor_path_driver_topic_20261123.py`(3개). 서버는 `state`를 그대로 저장하므로 키 추가에 서버 변경은 필요 없다(`ingest_state`). 실차·실제 odom 토픽 시험은 하지 않았다.

## R2. 자율 속도 제한 API — 서버 미구현(제안만), UI와의 불일치 3가지
UI 요청 계약(`POST /api/v1/robots/{id}/speed-limits`, `lease_epoch`, `agent_boot_id`, `protocol_version`, `safety_contract_version`)은 수용한다. 이전 제안의 `PUT .../speed_profile`은 이 경로로 통일한다. 다만:
1. **상한 상수 0.15 m/s / 0.50 rad/s를 검증된 값으로 쓰면 안 된다.** 이 값은 `integrated_nav2_lidar_only_slow_params.yaml`의 설정값(`desired_linear_vel 0.15`, `velocity_smoother.max_velocity [0.15,0,0.5]`)일 뿐 실차에서 검증된 적이 없다. 기본 launch는 `speed_profile:=normal`이라 이 파일이 아니고, 격리 bringup(`navigation.py`)은 0.10/0.30이다. → UI는 상한을 상수로 두지 말고 Agent가 보고하는 `state.speed_limits.ceiling`을 쓰고, 값이 없으면 입력을 비활성화한다. 서버는 Agent가 보고한 ceiling과 동적으로 비교해 거부한다(`422 speed_exceeds_ceiling`).
2. **Nav2 `velocity_smoother`의 파라미터 변경은 Humble에서 효과가 없다(실측).** 격리 ROS에서 활성 smoother에 `max_velocity`를 0.15→0.05로 `ros2 param set`하면 "Set parameter successful"이지만 출력 최댓값은 0.150 그대로였다(`p27_nav2_smoother_dynamic_param.log`). 따라서 "파라미터로 전달"은 적용 수단이 아니다. 최종 상한 강제는 **Agent Arbiter의 출력 clamp**(Nav2·수동 모두 통과하는 유일한 지점)에서 하고, Nav2 `SpeedLimit` 토픽은 부드러운 감속 보조로만 검토한다(미시험).
3. 응답의 `approved: true / effective_*`는 서버가 요청을 받아들였다는 뜻이 아니라 **Agent가 `applied`를 보고한 뒤에만** 참이어야 한다. Agent 보고 전에는 `pending`, 실패하면 `APPLY_FAILED`, Agent 재시작·계약 세대 변경·Lease 상실 시 저속 기본값으로 되돌린다.

불변식(서버·Agent 공통): Lease 보유자·`lease_epoch`·`agent_boot_id`·계약 버전 검증 실패는 `409`; 낮추는 방향 이외(ceiling 초과)는 `422`; 웹은 `/cmd_vel`을 발행하지 않는다; 기본·재시작 값은 ceiling이 아니라 더 낮은 값.

## R3. 구현 순서 제안 (이번 작업에서는 R1만 구현)
Agent: `state.speed_limits{ceiling,requested,applied,revision,apply_error}`와 Arbiter clamp → 서버: 라우트·검증·이벤트 → UI: `autoSpeedApiSupported` 전환. 각 단계에 격리 ROS 시험(clamp 효과, 재시작 후 기본값, 거부 규칙)을 붙인다. 실차 상한 상향은 별도 실차 검증 이후.

## R4. Antigravity에 전달할 변경 요청
- `speed_control_panel.js`: `AUTO_LINEAR_MAX/AUTO_ANGULAR_MAX` 상수 → `state.speed_limits.ceiling`(없으면 비활성).
- `state.velocity.status`가 `STALE`이면 값을 흐리게, `UNAVAILABLE`이면 숫자를 표시하지 않는다. 설명 툴팁에 "바퀴 오도메트리, 독립 측정 아님"을 넣는다.
- 서버 API 연결은 서버 구현 후에만 `autoSpeedApiSupported`를 켠다.
