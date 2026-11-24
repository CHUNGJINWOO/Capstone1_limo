# Antigravity UI 연결에 필요한 백엔드 변경 사항 (속도 제한, 2026-11-23)

대상: 브랜치 `feat/web-map-speed-ui-20261123`(웹 파일은 이 작업에서 수정하지 않음). 서버·Agent 구현은 G2 작업트리(미커밋)에 있고, 양쪽은 검증 후 별도로 통합한다. 상세 계약은 `SPEED_LIMIT_API_20261123.md`.

## 1. UI가 바꿔야 하는 것 (현재 `speed_control_panel.js` 기준 읽기 전용 검토)
1. `AUTO_LINEAR_MAX 0.15`, `AUTO_ANGULAR_MAX 0.50` 상수 제거. 슬라이더 상한은 `GET /api/v1/robots/{id}/speed-limits`의 `ceiling[scope]`(또는 `robot.state.speed_limits.ceiling`)을 쓴다. 값이 없으면 입력을 비활성화한다. 이 상한은 **설정값이며 실차 검증값이 아님**을 툴팁에 쓴다.
2. 전송: `POST /api/v1/robots/{id}/speed-limits` 본문에 `lease_epoch`(현재 Lease), `agent_boot_id`(`robot.agent.boot_id`), `protocol_version: 2`, `safety_contract_version: 1`, `scope`(`auto` 또는 `manual`), `max_linear_mps`, `max_angular_rps`, 가능하면 `client_request_id`(재시도 멱등)와 `expected_generation`.
3. **`approved`는 POST 응답이 아니다.** POST는 항상 `202`, `status: PENDING`, `approved:false`. UI는 `GET .../speed-limits`를 1 s 간격으로 폴링해 `approved:true`일 때만 "적용됨"으로 표시한다. `REJECTED`/`EXPIRED`이면 `reason`을 표시한다. 폴링 한도는 25 s(서버가 20 s 후 `EXPIRED`). Watchdog 경로는 확인에 0.5~2 s 더 걸릴 수 있다.
4. 표시 값: 사용자가 입력한 `requested`, Agent가 확인한 `applied`, 실제로 자르고 있는 `effective`, `ceiling`, `generation`, `status`/`reason`, `source`, 나이(`agent_report_age_s`, `applied.age_s`). `stale:true`이면 "Lease/재시작으로 설정이 만료됨, 낮은 값이 계속 적용 중 — 다시 설정하세요"를 표시하고 `approved`를 신뢰하지 않는다.
5. 오류 코드별 문구: `not_lease_owner`(Lease를 먼저 획득), `lease_epoch_mismatch`, `agent_boot_mismatch`(Agent 재시작: 상태 새로고침), `agent_cannot_enforce`(이 로봇은 서버 속도 제한을 강제할 수 없음 — 실차 Agent 포함), `speed_exceeds_ceiling`, `speed_below_minimum`, `stale_generation`, `idempotency_conflict`.
6. `robot.state.velocity`는 `/odom` twist의 **추정 표시값**이다(독립 측정 아님, 모터 정지 보장 아님). `status`가 `STALE`이면 흐리게, `UNAVAILABLE`이면 숫자 없이 표시.
7. 수동(WASD·게임패드)의 `manual` scope도 같은 API를 쓰되 상한은 0.10 m/s, 0.30 rad/s이고 전진만 있다.
8. 웹은 `/cmd_vel`을 발행하지 않는다(변경 없음).

## 2. 백엔드 쪽 현황과 UI가 의존하면 안 되는 것
- 구현됨(G2 미커밋): 서버 `POST/GET .../speed-limits`, 상태 응답의 `speed_limit_request`, Agent `state.speed_limits`, Arbiter clamp, `state.velocity`.
- 실차 Agent는 Watchdog clamp가 동기화됐을 때만 `enforced_by:'WATCHDOG'`을 보고한다(Watchdog이 없거나 clamp가 꺼져 있거나 동기화가 안 되면 `null`이고 요청은 `409 agent_cannot_enforce`). UI는 `state.speed_limits.enforced_by`가 `'ARBITER'` 또는 `'WATCHDOG'`일 때만 활성화하고 값에 따라 "Arbiter에서 강제" / "Watchdog에서 강제"를 표시한다. 격리 시험 결과이며 **실차 검증은 없다**.
- Nav2 `SpeedLimit`은 사용하지 않는다(Arbiter clamp가 유일한 강제 지점).
- 서버·Agent 응답 필드 이름은 이 문서와 같고, 바뀌면 이 문서와 `test_speed_limits_server_20261123.py`가 먼저 바뀐다.

## 3. 통합 방법 제안
두 작업트리를 자동 병합하지 않는다. 각각 검증·승인 후 `git diff`로 파일 충돌이 없음을 확인하고(서버 쪽은 `service.py`, `http_api.py`, 새 `speed_limits.py`; UI 쪽은 `static/*`) 별도 통합 커밋으로 합친다. 통합 후 UI E2E(Antigravity 7/7 Headless Chrome 시험 + 이 문서의 계약 시험)를 다시 실행한다.

## 4. Phase C 개정 (2026-11-24)
- 실서버 대조 결과: UI의 요청 본문·202/PENDING·GET 폴링·`verifyAppliedMatch`·증거 평가는 서버와 일치했다. 브라우저에서 APPLIED, 콘솔 버튼으로 낸 요청의 `PENDING → APPLIED`, Lease 변경 후 `STALE`(유효 한도 유지, 새 요청 버튼 활성)을 확인했다.
- 변경 요청 1건: 폴링 한도 20 s → 25 s(`UI_INTEGRATION_PATCH_20261123.diff`). 서버 `PENDING` 만료가 20 s라서 UI가 `EXPIRED`를 보기 전에 포기하지 않게 하려는 것이다.
- 서버가 이제 주는 필드: `agent_boot_id`, `watchdog_boot_id`, `watchdog_evidence{server_verified, applied_match, ...}`. UI는 `server_verified`가 **서버 판정**임을 알고 쓰되, 표시 전에 `applied`의 generation·request_id·값을 요청과 다시 대조(`verifyAppliedMatch`)하는 현재 동작을 유지한다.
- 표시 문구 제안: 첫 화면(UI에서 요청한 적 없는 상태)의 결과줄이 `[STALE] 적용 상태 미확인 — 통신 단절…`로 보이지만 상태 배지는 APPLIED인 경우가 있었다. 결과줄은 "이 화면에서 요청한 이력이 없음"으로 구분하는 것이 혼동이 적다.
