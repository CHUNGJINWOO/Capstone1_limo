# Web Map Visualization & Speed Control UI — Server/Agent 통합 제안 (2026-11-23)

- **작성 작업트리 / 브랜치**: `Capstone1_limo_web_ui_20261123` / `feat/web-map-speed-ui-20261123` (G2 `15bb5c59` 기준)
- **범위**: 웹 지도 시각화(위치·방향·Nav2 경로·궤적·경유지·HUD) 및 속도 설정 UI (`speed_control_panel.js`)
- **원칙**: Claude Code 담당 안전 제어·Robot Agent·Fleet Server·배포 코드는 직접 수정하지 않고 본 문서로 통합 계약만 제안합니다.

---

## 1. 미구현 Server · Agent 계약 (Claude Code 통합 요청)

### 1.1 관측 속도 텔레메트리 (`/agent/state` 및 `/robots/{id}/sensors`)
현재 웹 UI(`state_helpers.js::extractObservedVelocity`, `map_view.js`, `speed_control_panel.js`, `app.js`)는 아래 필드가 보고되면 실제 관측 속도를 표시하고, 미제공 시 `UNAVAILABLE`로 표시합니다.
```json
"velocity": {
  "status": "OK",
  "linear_x_mps": 0.10,
  "angular_z_rps": -0.15,
  "source": "odom",
  "age_s": 0.08
}
```
- **Agent 요청**: `/odom` (또는 Safety Gate 출력 관측치)에서 최신 `Twist`를 읽어 `state.velocity`로 포함. `odom`이 stale이면 `status: "STALE"` 보고.

### 1.2 자율주행 속도 제한 API (`POST /api/v1/robots/{id}/speed-limits`)
현재 백엔드에 해당 엔드포인트가 없으므로 웹 UI(`speed_control_panel.js`)는 `autoSpeedApiSupported: false`로 설정하여 버튼을 **명시적으로 비활성화(`서버·Agent API 미구현`)**하고 있습니다.
- **요청 계약**:
  ```json
  {
    "lease_epoch": 5,
    "agent_boot_id": "<boot_id>",
    "protocol_version": 2,
    "safety_contract_version": 1,
    "max_linear_mps": 0.12,
    "max_angular_rps": 0.35
  }
  ```
- **서버·Agent 필수 안전 불변식**:
  1. `agent_contract_valid` (`protocol_version=2`, `safety_contract_version=1`, 유효한 `boot_id`) 및 소유자 `lease_epoch` 검증 실패 시 `409 Conflict` 거부.
  2. 실차 검증 저속 상한(`max_linear_mps <= 0.15`, `max_angular_rps <= 0.50`) 초과 요청은 서버에서 무조건 거부(`400/422`).
  3. `/cmd_vel`을 직접 발행하거나 Arbiter·Watchdog·Safety Gate를 우회하지 않고, Nav2 `SpeedLimit` 또는 `velocity_smoother` 파라미터/토픽으로만 전달.
- **응답 계약**:
  ```json
  {
    "approved": true,
    "effective_linear_mps": 0.12,
    "effective_angular_rps": 0.35,
    "reason": "APPLIED"
  }
  ```
  - 서버 구현 완료 후 `app.js`에서 `autoSpeedApiSupported: true` 및 `applyAutoSpeedLimit` fetch 핸들러만 연결하면 즉시 활성화됩니다.

### 1.3 경유지(`waypoints`) 필드 확장
`state_helpers.js::extractWaypoints`는 활성 임무의 `mission.waypoints` 또는 `robot.state.waypoints` (`[{ "frame_id": "map", "x": 1.5, "y": 0.8, "yaw_rad": 0.0, "label": "W1" }]`)를 자동 렌더링합니다.

---

## 2. 공통 문서 갱신 제안 (직접 미수정)

- **`WORKTREES.md`**: `Capstone1_limo_web_ui_20261123` (`feat/web-map-speed-ui-20261123`, base `15bb5c59`, Antigravity 웹 지도·속도 UI 독립 작업트리) 행 추가.
- **`CODE_INDEX.md`**: `fleet_control/server/fleet_server/static/speed_control_panel.js` 및 테스트 2종(`test_web_map_and_speed_ui_20261123.cjs`, `test_web_map_speed_chrome_e2e_20261123.cjs`) 항목 추가.
- **`CURRENT_STATUS.md` / `VALIDATION.md` / `HANDOFF.md`**:
  > 웹 지도 시각화(좌표 변환 검증, `OK`/`STALE`/`UNAVAILABLE` 위치 구분, Nav2 경로·경유지·궤적 초기화/상한·HUD) 및 속도 설정 UI(`speed_control_panel.js`, 수동 상한 `<= 0.10 m/s · 0.30 rad/s` 연동, 자율 속도 제한 API 미구현 명시)를 독립 worktree(`feat/web-map-speed-ui-20261123`)에서 구현하고 Node 단위 시험 + 격리 Chrome E2E + 기존 Mapping/Gamepad 회귀 36/36 PASS 확인. 실차 연동은 Server `velocity` 텔레메트리 및 `POST /api/v1/robots/{id}/speed-limits` 구현 후 통합 예정.

---

## 3. Phase C 웹 속도 UI 통합 완료 현황 (2026-11-23)

Claude Code의 `SPEED_LIMIT_API_20261123.md` 및 `SPEED_LIMIT_UI_INTEGRATION_20261123.md` 계약을 `feat/web-map-speed-ui-20261123` 작업트리의 `speed_control_panel.js`, `app.js`, `state_helpers.js`, `style.css`에 반영 완료했습니다.

- **POST(202 PENDING) → 1초 간격 GET 폴링(최대 20초)**: `approved:true && stale !== true`일 때만 `APPLIED`로 표시하며 `PENDING` / `APPLIED` / `REJECTED` / `EXPIRED` / `SUPERSEDED` / `STALE` 상태를 구분 표시합니다.
- **동적 Ceiling 및 상수 상한 제거**: 하드코딩된 `0.15 / 0.50` 상수를 제거하고 Agent가 보고한 `ceiling[scope]`만 사용합니다(미수신 시 입력 차단, "설정값이며 실차 검증값이 아님" 툴팁 표시).
- **강제 지점(`enforced_by`) 게이팅**:
  - `ARBITER`: Mock/시뮬 UI 활성화.
  - `null` (실차 Agent): 모든 요청 차단(`agent_cannot_enforce`).
  - 향후 `WATCHDOG`: `enforced_by === 'WATCHDOG'` 단독으로는 활성화하지 않으며, 서버·Agent가 명시적 검증 증거(`watchdog_clamp_verified: true` 또는 `enforcement_verified: true`)를 보고할 때만 활성화되도록 대비했습니다.
- **`/odom` 추정 속도 텔레메트리**: `null`/`UNAVAILABLE`가 `0.00 m/s`로 오표시되지 않도록 `extractObservedVelocity`를 보강하고, `STALE` 시 `.vel-stale`로 흐리게, `UNAVAILABLE` 시 숫자 없이 표시합니다.
- **통합 방법**: `feat/web-map-speed-ui-20261123`(웹 `static/*`, UI 테스트)과 G2 작업트리(서버 `speed_limits.py`/`service.py`/`http_api.py`, Agent/Watchdog)는 수정 파일이 겹치지 않으므로 각각 검증·승인 후 별도 통합 커밋으로 병합하고, 통합 후 `test_web_map_speed_chrome_e2e_20261123.cjs` 및 `test_speed_limits_server_20261123.py`를 함께 재실행합니다.
