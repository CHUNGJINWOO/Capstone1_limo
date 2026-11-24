# Phase C 통합 기록 (G2 + Antigravity 웹 UI, 2026-11-23/24, 격리 ROS·mock 검증, 미커밋)

**실차 검증 없음.** 모든 결과는 mock Agent·격리 ROS·합성 센서이며 실제 LIMO(vendor `limo_base`, STM32, 물리 E-Stop, 모터)는 포함하지 않는다.

## 1. 통합에 사용한 기준
- G2 worktree `Capstone1_limo_g2_20261123`, HEAD `15bb5c59b806f58d95e19fb6308effd02c15f0de`. 스테이징 32개 + 이후 수정(미스테이징 7개 이상). Antigravity worktree `Capstone1_limo_web_ui_20261123`(브랜치 `feat/web-map-speed-ui-20261123`, HEAD도 `15bb5c59`, **커밋 없음**): 수정 6개(`static/app.js`, `index.html`, `manual_control_panel.js`, `map_view.js`, `state_helpers.js`, `style.css`) + 신규 4개(`speed_control_panel.js`, 문서 1, `.cjs` 시험 2).
- 브랜치 ref만 보면 UI 변경이 통째로 빠진다(두 브랜치 ref가 같다). 그래서 **작업트리 파일 내용**을 기준으로 삼았다.
- 두 작업트리의 변경 파일은 겹치지 않는다(G2 스테이징 목록과 UI 변경 목록의 교집합 0). 충돌 해결이 필요하지 않았다.
- 통합 공간: 새 git worktree는 만들지 않았다(AGENTS.md 승인 규칙). 스크래치패드의 **일반 복사본**(`integ/`)에 G2 추적 파일 + UI 변경 파일을 복사해 시험했다. 복사 후 SHA-256으로 UI 10개 파일과 G2 스테이징 32개가 원본과 동일함을 확인했고, 두 작업트리는 읽기만 했다(`git status` 변화 없음).
- 통합 복사본에서만 바꾼 것: `speed_control_panel.js`의 `DEFAULT_POLL_TIMEOUT_MS` 20000 → 25000. 변경안은 `UI_INTEGRATION_PATCH_20261123.diff`(Antigravity가 자기 작업트리에 적용). 나머지 UI 파일은 원본 그대로다.
- 통합 커밋(한 브랜치로 합치기)은 하지 않았다. 필요한 순서: ① 사용자 승인 ② G2 변경 커밋 ③ UI 변경 커밋(Antigravity) ④ 둘을 한 곳에서 병합하고 이 문서의 시험을 다시 실행.

## 2. API 계약 정합성 (웹 ↔ 서버 ↔ Agent ↔ Watchdog)
UI(`speed_control_panel.js`)는 SPEED_LIMIT_API 문서대로 이미 구현되어 있었고(요청 본문, 202/PENDING, GET 폴링, `verifyAppliedMatch`, 증거 평가), 실제 서버와 맞는지 실물로 확인했다. 확인 중 서버·Agent 쪽에서 아래를 고쳤다.

| 필드 | 출처 | 비고 |
|---|---|---|
| `robot_id`, `agent_boot_id` | 서버 | 응답에 포함(요청 기록의 Agent boot) |
| `watchdog_boot_id` | Watchdog 상태 → Agent → 서버 | **새로 추가**(응답과 `state.speed_limits.watchdog_evidence`) |
| `scope`, `request_id`, `generation`, `requested` | 서버 | 서버가 부여, 로봇별 단조 증가 |
| `applied` | Agent 보고 | `scope`, `request_id`, `generation`, 값, `age_s` 포함 |
| `ceiling`, `effective` | Agent 보고 | 상수가 아니라 Agent가 보고한 값 |
| `approved` | 서버 | 아래 조건을 모두 만족할 때만 true |
| `stale`, `stale_reason` | Agent 보고 | 연결 단절·Lease·서버 boot·계약 세대 변경·Agent 재시작 시 true |
| `enforced_by` | Agent 보고 | `WATCHDOG`은 Watchdog이 상태로 입증할 때만, 아니면 `null` |
| `watchdog_evidence` | Agent가 Watchdog 상태를 **데이터로 복사** + 서버가 계산한 `server_verified`, `applied_match`, `server_checks` | UI가 받은 boolean 하나만으로 인정하지 않는다 |
| `source`, `agent_report_age_s`, `requested_at`, `resolved_at` | 서버 | `source`는 enforced_by에 따라 Arbiter/Watchdog 문구 |

`approved=true` 조건(WATCHDOG 경로): Agent가 같은 generation을 `APPLIED`로 보고 **AND** `stale=false` **AND** enforcement point 보고 **AND** 서버가 증거에서 직접 확인: ① 증거의 Agent boot가 연결된 Agent boot와 같음 ② `watchdog_boot_id`·`output_gid` 존재 ③ clamp 켜짐·동기화·게시자 정상·결함 없음 ④ 증거 나이 ≤ 2 s이고 상태 나이 ≤ 3 s ⑤ **Watchdog이 보고한 `applied[scope]`의 generation·request_id·값이 서버 기록과 Agent 적용값과 일치하고 `stale=false`**. 하나라도 어긋나면 `PENDING`에 머물고 20 s 후 `EXPIRED/WATCHDOG_EVIDENCE_MISSING_OR_MISMATCH`.
- `server_verified`(증거가 "작동하는 enforcement point"임)와 `applied_match`(특정 요청이 적용됨)를 분리했다. Lease가 바뀐 뒤에도 `server_verified`는 true를 유지해야 UI가 **새 요청**을 허용할 수 있다. 처음 구현은 둘을 합쳐서, Lease 변경 후 UI가 새 요청을 막는 결함이 있었고 브라우저에서 발견해 고쳤다.
- DDS 게시자 이름·writer GID는 **그래프 식별자이지 인증이 아니다**. 문서·응답 어디에서도 보안 보장으로 표현하지 않는다(`source` 문자열에도 명시). SROS2는 도입하지 않았다.

**시간 관계**(UI 폴링·Agent 확인·서버 만료): Agent→Watchdog 확인 대기 10 s, 서버 `PENDING` 만료 20 s, UI 폴링 한도를 **25 s**로 늘렸다(UI가 서버의 `EXPIRED`를 보기 전에 포기하지 않도록). UI는 `status==APPLIED && approved===true && !stale`일 때만 적용으로 표시하고, `EXPIRED`/`REJECTED`/`SUPERSEDED`는 별도 문구로 표시한다(실제 브라우저에서 확인). 실측 승인 시간: POST 후 1.4~1.9 s.

## 3. 발견·수정한 결함 (이번 통합에서)
1. **서버 HTTP 500**: Watchdog 증거가 있는데 `enforced_by`가 아직 `WATCHDOG`이 아닌 순간(Agent 재시작 직후)에 `_sl_annotate`가 `KeyError`로 상태 수신 전체를 500으로 만들었고, 재시작한 Agent가 서버에 다시 붙지 못했다. 수정: 키 누락 보완, 그리고 속도 제한 기록 처리가 상태 수신을 절대 깨뜨리지 못하도록 `try/except`(오류는 이벤트로 기록, 요청은 `PENDING`으로 남아 만료). 회귀 시험 추가.
2. UI가 Lease 변경 후 새 요청을 막음(위 `server_verified`/`applied_match` 분리).
3. `approved`가 enforcement point 없이 true일 수 있었음(Agent가 `enforced_by:null`을 보고하는 순간): 이제 false.
4. 기록 문구: 응답 `source`가 Watchdog 경로에서도 "Arbiter"라고 표시됨 → 수정.

## 4. 전체 E2E: Web → Server → Agent → Watchdog → Gate (격리 ROS, mock Agent)
스택: `scripts/run_isolated_fleet.sh`에 `FLEET_FLAGS='speed_clamp:=true manual:=false velocity:=false'`(실차 launch의 모양: Arbiter 없음, Watchdog 속도 clamp 켬)로 **실제 프로세스** Fleet Server, Robot Agent(mock/simulated, `WatchdogSpeedLink`), Motion Watchdog, Safety Gate(`stall_detector_node`, 출력은 `/cmd_vel_locked`), Nav2 velocity_smoother, 합성 센서. 이 프로파일에는 모터 드라이버가 없어 측정 지점은 Gate 출력이다(드라이버 리매핑과 Mock 드라이버는 `real_launch_chain_faults_20261123.py` 42/42가 담당). 시험 `e2e_web_watchdog_speed_limit_20261123.py` **43/43 PASS**(로그 `p37_e2e_web_watchdog_g.log`, 결과 `…_g_result.txt`).

| 단계 | 결과(측정) |
|---|---|
| 연결·Lease | online·reconciled, `enforced_by WATCHDOG`, 증거를 서버가 검증, Watchdog AUTHORIZED, Gate NORMAL, 모터 경로 LOCKED, Lease epoch |
| 양성 대조 | 컨트롤러 stand-in이 0.30 m/s를 제공 → smoother·Watchdog·Gate를 거쳐 Gate 출력 최대 **0.100 m/s**(이 프로파일의 smoother 상한 0.10/0.30), 302 샘플 |
| POST 0.06 m/s / 0.20 rad/s | HTTP **202**, `PENDING`, `approved=false` |
| 확인 | `(PENDING,false)` → `(APPLIED,true)` 순서, **1.4~1.9 s**, `generation`·`request_id`·`scope`·값·`ceiling`·`agent_boot_id`·`watchdog_boot_id`·`enforced_by` 일치, Watchdog 증거의 generation·request_id·값·`stale=false`가 서버 기록과 일치 |
| 실제 Gate 출력 | 한도 0.06 아래에서 컨트롤러가 0.30을 계속 제공하는 동안 최대 전진 **0.060**, 후진 −0.060, 각속도 ≤ 0.20, 301 샘플(흐름 유지) |
| 초과 요청 | 0.20 m/s → 422 `speed_exceeds_ceiling`, 각속도 0.60 → 422, manual 0.12 → 422, 잘못된 `lease_epoch` → 409, 잘못된 `agent_boot_id` → 409; 거부는 아무것도 바꾸지 않음 |
| Lease 상실·재획득 | 이전 설정 `stale=true`, `approved=false`, 유효 한도는 **0.06 유지**(자동 상향 없음), 새 Lease 후 측정 최대 0.060 |
| Watchdog 재시작 | 새 `watchdog_boot_id`, 복원된 한도 0.06(stale·미확인), **재시작한 Watchdog은 새 writer라 Gate가 fail-closed**(출력 0, 컨트롤러 0.30 제공 중에도), 운영자가 죽은 writer의 DDS endpoint가 사라질 때까지 기다린 뒤(관측 약 20 s) Gate를 reset해야 재개, 재개 후 출력 ≤ 0.06 |
| 명시적 상향 | 새 Lease 아래 0.12 요청 → 새 Watchdog boot 증거로 승인, 측정 출력이 0.06을 넘음(이 프로파일 smoother 상한 0.10에서 포화) |
| Agent 재시작 | Agent kill → Watchdog heartbeat 상실 → Gate 출력 0; 새 boot로 재연결, 이전 기록은 `approved=false`, Agent 한도는 stale이지만 **0.12 유지**(0.15 아님), 옛 `agent_boot_id`는 409; 출력 ≤ 0.12 |
| STOP | 컨트롤러가 명령을 멈추면 Gate 출력 0 |
| FAULT | Agent를 SIGSTOP(heartbeat·제어 루프 정지)하면 컨트롤러가 0.30을 계속 제공해도 Gate 출력 0, 복구 후 한도 불변(상향 없음) |
| UI 정합 | `state.velocity`는 `/odom` 기반 추정(독립 측정 아님)으로 표기, `state.speed_limits.effective`와 `GET`의 값 일치 |

**이 E2E의 한계(중요)**
- 비영 명령은 **Nav2 컨트롤러 stand-in**(smoother 입력 `/fleet/cmd_vel_nav`에 게시)이다. 이 프로파일에서는 운영 모드(MAPPING/NAVIGATION/MANUAL)에 들어갈 수 없다: 서버의 모드 확인은 Agent의 velocity selector(Arbiter)를 요구하고(`agent has no velocity selector to confirm MAPPING`), Arbiter는 mock Agent에만 있다. 즉 **실차와 같은 모양(Arbiter 없음)의 Agent로는 웹에서 Mapping/Navigation 목표를 시작할 수 없다**. 이것은 속도 제한과 별개의 기존 설계 한계이며 실차에서는 Mapping·미션이 어차피 차단되어 있다.
- 따라서 Lease 해제가 모션을 끊는 것(목표 취소)과 Nav2의 경로 추종은 이 E2E에서 측정하지 않았다. 합성 로봇은 움직이지 않는다.
- **경로 곡률 영향**: 입력 ω/v = 1.00 1/m(0.30 m/s, 0.30 rad/s)이 이 프로파일에서 smoother로 3.00 1/m(v 0.10, ω 0.30), 0.06 한도에서 3.33 1/m(v 0.06, ω 0.20)이 되었다. 한 축만 포화되면 곡률이 커진다. 이것이 Nav2의 경로 추종에 미치는 영향은 **측정하지 못했다**(후속 차단 조건, Gazebo 같은 움직이는 시뮬레이터가 필요).
- 가감속: clamp와 이 경로 어디에도 가감속 제한이 추가되지 않았다(0 → 0.30 입력에 출력이 계단).
- 브라우저(실제 콘솔, 내장 브라우저): APPLIED(WATCHDOG 증거 확인됨) 표시, 콘솔의 "자율 제한 요청" 버튼으로 0.05/0.15를 보내 `PENDING → APPLIED(gen 2)` 도달, Lease 변경 후 `STALE`·실효 한도 0.06 유지·새 요청 버튼 활성 확인. 화면 캡처는 저장하지 않았고 DOM 값을 읽어 확인했다.

보조 시험: 서버 정지(SIGSTOP 25 s)·재개 `e2e_server_outage_speed_limit_20261123.py` 6/6(연결 단절은 STALE, 한도 0.06 유지, 새 요청으로만 재승인). 서버 **재시작**은 `run_isolated_fleet.sh`가 서버 종료 시 스택 전체를 내리는 구조라 이 하네스에서 시험할 수 없다(기존 `test_phase20_http_recovery` 등이 다룸).

## 5. 기존 기능 회귀
- Python(R1·R2·Gate·Arbiter·bench·cancel·계약·Agent·서버·Phase4·ownership 등) 77개 항목 전부 OK(`p39_regression_phaseC.log`): R2 66(22+4+23+12+3+2), 서버 속도 API 61/61, `test_ownership` 45/45, R3 11(`unshare` 밖에서 실행).
- 격리 launch 추출 하네스 42/42(`p41`), Arbiter 프로파일 clamp 프로브 18/18(Mapping Nav2 출력이 한도에서 포화), 체인 프로브 29/29(첫 실행은 `ros2 topic info -v`가 시작 직후 노드 이름을 `_NODE_NAME_UNKNOWN_`으로 돌려준 DDS 발견 시점 플레이크 1회로 28/29, 재실행 29/29).
- 웹(Node): `test_web_map_and_speed_ui` 통과, Headless Chrome E2E(`test_web_map_speed_chrome_e2e`) 통과, 네비게이션·운영 모드·시스템·Gate reset·지도 저장·등록 업로드·goal-check 6/6·static dashboard·gamepad payload·수동 입력 순수 함수 시험 통과. `test_manual_control_panel.cjs`(playwright 없음)와 `test_mapping_panel.mjs`(assertion)는 **G2 기준 커밋에서도 똑같이 실패**한다(사전 존재, 이번 변경과 무관).
- Mapping START/PAUSE/RESUME/SAVE/STOP, 수동 입력·Gamepad, 지도 위치·방향·경로·궤적 UI는 Arbiter 프로파일의 기존 시험과 Antigravity의 UI 시험이 담당한다(이번에 Mapping은 Arbiter 프로파일 프로브에서 START/STOP, 지도 UI는 Node·Chrome 시험으로 확인). 웹 지도 화면의 시각 검증(스크린샷)은 하지 않았다.

## 6. 남은 안전 차단 조건 (실차 의존, 이번 작업으로 해소되지 않음)
- `/limo_driver/cmd_vel` **직접 발행 우회**: Gate는 감지만, Agent는 ARMED 래치로 보고만 한다. 물리적 차단이 아니다.
- vendor `limo_base`의 입력 리매핑 실확인, STM32 명령 타임아웃, 물리 E-Stop, 제동거리, 전원·USB/UART.
- 실차 하드웨어에서 Watchdog·Agent·Nav2의 동기화 순서와 `normal` 프로파일의 0.15 clamp 영향.
- 실차 Agent는 운영 모드를 확인할 수 없어 웹의 Mapping/Navigation이 불가(설계 한계), Nav2 경로 추종·곡률 영향 미측정, 가감속 제한 없음.
- DDS 인증(SROS2) 없음: 그래프 식별만.
- 기본 launch가 `normal` 프로파일에서 Nav2 목표 속도가 0.15보다 높을 수 있다(이제 Watchdog이 자른다).
- 배포 후보는 미커밋 작업트리 기준이라 커밋 후 SHA가 바뀐다.

## 7. 후속(2026-11-24): 실차형 Agent의 모드 진입 차단 해소
위 "한계" 중 *실차와 같은 모양(Arbiter 없음)의 Agent는 웹에서 Mapping/Navigation 모드에 들어갈 수 없다*는 항목은 `REAL_AGENT_MODE_SELECTION_20261124.md`에서 해소했다(격리 ROS E2E 45/45와 12/12). 이 문서의 E2E 결과(43/43)는 그대로 유효하고, 그 뒤의 변경(Watchdog 모드 게이트 등)을 얹은 상태에서도 다시 43/43을 확인했다(`p49_e2e_web_watchdog_after_modes.log`). Nav2 경로 추종·곡률 영향 미측정, 가감속 제한 없음은 그대로 남아 있다.
