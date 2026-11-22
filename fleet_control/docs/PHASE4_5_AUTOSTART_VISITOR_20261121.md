# Phase 4 SSH 없는 자동 기동 · Phase 5 방문객 안내 웹 (2026-11-21, Claude)

## 범위와 검증 수준

검증 수준은 다음과 같다.
- **단위**: ROS 없음.
- **격리 ROS**: kinematic sim, Nav2, Gate r12, Motion Watchdog, Agent를 실제 프로세스로 띄웠다. ROS_LOCALHOST_ONLY=1, 사설 도메인 226/227을 썼다.
- **실제 브라우저**: headless Chrome. 방문객 화면은 390×844 모바일 뷰로 시험했다.
- **Gazebo**: 이번 단계에서는 실행하지 않았다(SKIP). 지도 클릭 경로는 격리 ROS에서 시험했다.
- **실차**: 실행하지 않았다.

지키거나 하지 않은 것:
- 운영 18190, LIMO SSH, 실차 자동 기동 등록, 실차 모터 잠금, 실차 Mapping START 차단은 건드리지 않았다.
- 다음 소유 파일은 수정하지 않았다: `rmf_adapter/`(오늘 다른 작업자 수정 있음), Frontier C++ 노드, Gate 후보.
- commit/push는 하지 않았다. `fleet_control/`은 미추적이다. 변경 전 원본은 `local_data/raw_results/refactor35_claude_20261121/pre_change_originals/*.before_phase4`에 있다.

원자료: `local_data/raw_results/{phase4_autostart_claude_20261121, phase5_visitor_claude_20261121}`

## 1. 구현한 기능

### Phase 4 — SSH 없는 자동 기동

**로봇 측 감독 프로세스 `limo_robot_agent.supervisor`**
- 로봇의 로컬 설정 파일에 있는 서비스만 실행한다(stack: 센서·Nav2·Gate / watchdog / agent).
- 순서: `after`(의존 서비스가 READY여야 시작).
- 준비 판정: 프로세스 생존 + ROS 토픽 신선도.
- 재시작 정책: 윈도우 안 최대 횟수를 넘으면 FAILED로 두고 운영자 재시도를 기다린다.
- `restart_with`: Gate stack이 재시작되면 Watchdog과 Agent도 재시작한다. 기존 설계상 새 Gate boot에는 새 Agent가 필요하다. 단, Watchdog 재시작은 Agent를 재시작하지 않는다.
- 서버에 1 s마다 보고하고, 응답으로 관리자가 넣은 작업을 받는다.
- 웹에서 보낼 수 있는 것은 **서비스 이름 + start/stop/restart/retry**뿐이다. 명령어·인자·경로·환경은 웹에서 줄 수 없다. 감독도 로컬 허용 목록을 다시 확인한다.
- ROS 메시지를 발행하지 않고 `/cmd_vel*`을 구독하지 않는다. Mapping/Navigation을 시작하지 않으며 Gate 래치를 해제하지 않는다.
- `--check`로 설정 파일을 정적 검증한다.

**서버 `readiness.py`**
- READY / DEGRADED / FAULT와 모든 사유를 계산한다.
  - FAULT: 감독 보고 없음/끊김(필수 설정 시), 서비스 FAILED/STOPPED, Agent 오프라인, 재조정 필요, Gate boot 재조정, Gate 장애 래치.
  - DEGRADED: 서비스 시작 중, Agent 재동기화, Gate 미준비, Watchdog 상태, 토픽 끊김, **Nav2 서버 없음**.
- `--require-supervisor-ready` 서버 옵션이 켜지면 MAPPING/NAVIGATION 모드 전환, mapping START, 임무 생성은 READY일 때만 허용한다. 옵션이 꺼져 있으면 기존 동작이다.
- 서비스 작업 권한: 관리자만 가능하다.
  - stop/restart는 MANUAL/MAPPING/NAVIGATION, 전환 중, 임무 중에는 거부한다.
  - start/retry(정지·실패 서비스 기동)는 항상 허용한다. 기동 자체가 모터 구동 허가를 주지 않기 때문이다.
- 모든 요청·결과·서비스 상태 변화·감독 재시작을 이벤트로 기록한다.

**운영자 웹 "로봇 준비 상태 · 서비스" 카드(`system_panel.js`)**: 수준, 사유, 서비스 표, 관리자 버튼, 마지막 작업 결과를 보여 준다.

**배포 템플릿(설치하지 않음)**: `fleet_control/scripts/autostart/limo-supervisor.service`(systemd user unit, 감독만 기동), `supervisor.limo.example.json`(`limo_run.sh`의 실차 명령을 옮긴 예시이며 **실차 미검증**).

### Phase 5 — 방문객 안내 웹

**서버 `guide.py`**
- 방문객 링크: 관리자가 발급하고 만료·폐기할 수 있다. SHA-256 해시로 저장한다. 헤더는 `Authorization: Visitor <token>`이다.
  - Bearer 인증과 완전히 분리되어 있다. 방문객 토큰은 `/api/v1/visitor/*` 외에는 쓸 수 없고, Bearer 토큰은 방문객 경로에서 거부된다.
- 목적지: `(map_version_id, content_sha256)`과 지도 좌표에 묶인다. 등록 시 **임무와 같은 서버 목표 검사**(`check_point`, 0.20 m)로 그 버전에서 검증한다.
- 안내 가능 조건(모두 충족해야 함):
  - 지도가 VERIFIED이고 격리되지 않았으며 내용 해시가 같다. **C103 등 UNVERIFIED 지도는 안내하지 않는다.**
  - 로봇이 안내 서비스를 운영 중이고(운영자가 Lease를 보유한 채 시작), READY(옵션 시)이며 NAVIGATION 모드이다.
  - 같은 지도 버전에서 CONFIRMED이고, 로딩된 지도에서 목표 검사를 통과한다.
- 요청 상태: RECEIVED → QUEUED(다른 안내 중) → ACCEPTED → IN_PROGRESS → COMPLETED / CANCELED / FAILED. 생성 시 거부되면 UNAVAILABLE이며 사유를 함께 준다.
  - 임무는 서비스를 연 **운영자 이름과 Lease epoch**로 생성된다(note `visitor guide <id>`).
  - 세션당 진행 중 요청은 1건이다. 링크당 분당 10건으로 제한한다.
  - 대기열은 FIFO이고 housekeeping에서 배차한다.
- 운영자 API: 목적지 관리, 서비스 열기/닫기, 요청 목록. 관리자 API: 방문객 링크 발급/폐기/목록.

**방문객 화면 `visitor.html/js/css`**(모바일 우선, 외부 라이브러리 없음)
- 기능: 건물·층 선택, 구역 칩, 검색, 안내 가능 여부와 사유, 요청, 단계 표시(접수 → 배정 → 안내 중 → 도착), 대기 순번, 취소, 새로고침 후 복원.
- 토큰은 주소창에서 지운다. 운영 기능, 좌표, 진단 정보는 없다.

**운영자 카드(`guide_panel.js`)**
- 서비스 시작/중지, 목적지 목록, 지도 클릭 좌표로 목적지를 추가한다(로딩된 지도 버전에 묶임).
- 관리자는 링크를 발급해 QR(`qr_encode.js`)과 주소를 함께 표시한다.

### 목표 유효성 기준 통일(웹 ↔ 서버)
- 대시보드 지도 클릭 사전 검사는 이제 **서버 `goal-check`(임무 생성과 같은 규칙)**로 판정한다.
- 기존 화면 검사(0.32 m)는 참고 문구로만 남겼다. 서버가 0.20 m로 허용하는 지점을 화면이 거부하던 불일치(이전 보고 6절)를 없앴다.

### 수정하지 않은 것
- Gate 재개 직후 오래된 명령 1회 출력: Gate 소유 코드라 별도 차단 항목으로 유지한다.

## 2. 변경 파일

| 파일 | 성격 |
|---|---|
| `server/fleet_server/readiness.py`(신규), `guide.py`(신규) | 서버 |
| `server/fleet_server/service.py` | mixin, 임무의 READY 검사, robot_view `readiness`, housekeeping `guide_tick` |
| `server/fleet_server/operating_mode.py`, `mapping.py` | MAPPING/NAVIGATION/mapping START의 READY 검사(옵션) |
| `server/fleet_server/http_api.py` | `/agent/supervisor`, `/robots/{id}/readiness`, `/robots/{id}/services`, `/guide/*`, `/visitor-links*`, `/visitor/*`(별도 인증) |
| `server/fleet_server/__main__.py` | `--require-supervisor-ready` |
| `server/fleet_server/static/{system_panel.js, guide_panel.js, qr_encode.js, visitor.html, visitor.js, visitor.css}`(신규) | 웹 |
| `server/fleet_server/static/{index.html, app.js}` | 카드 장착, 서버 기준 목표 검사 |
| `agent/limo_robot_agent/limo_robot_agent/supervisor.py`(신규) | 로봇 측 |
| `scripts/autostart/{limo-supervisor.service, supervisor.limo.example.json}`(신규) | 템플릿(미설치) |
| `tests/mode_e2e/stack_up.sh` | `NO_START`, `SERVER_EXTRA`(기본값 불변) |
| `tests/{test_phase4_readiness.py, test_phase4_supervisor.py, test_phase5_guide.py, test_qr_encode.py, test_system_panel.cjs}`(신규) | 단위 |
| `tests/phase4/{run_phase4.py, browser_system_panel.cjs}`, `tests/phase5/{run_phase5.py, browser_visitor.cjs}`(신규), `tests/gazebo_web/browser_map_click.cjs`(임무 표 행 추적) | E2E |

## 3. 자동 시작과 준비 상태 흐름

```text
전원 → systemd(템플릿) → supervisor(로컬 config)
  stack(sim/센서·ICP·Nav2·Gate) ─READY(/scan·/odom·Gate 상태)→ watchdog ─READY(Watchdog 상태)→ agent ─(Agent hello)→ 서버
서버 readiness: UNKNOWN_ROBOT → DEGRADED(SERVICE_RUNNING:agent, Gate/Nav2 대기) → READY
READY 전: MAPPING/NAVIGATION/임무 409 robot_not_ready. READY 후에도 모터 구동은 운영자의 모드 전환·Lease·Arbiter·Watchdog·Gate가 결정
```

격리 실측:

| 순서 | READY까지 | 경과 |
|---|---|---|
| 서버 먼저 | 17.9 s | Agent hello 14.9 s, DEGRADED 3 s |
| 감독 먼저 + 센서 20 s 지연 | 서버 기동 후 12 s(감독 기동 후 37.1 s) | 서버 없이 25 s 대기, DEGRADED 중 MAPPING 거부(`robot_not_ready`, Nav2 FollowPath 서버 없음 사유 포함) |

## 4. 운영자 웹에서 가능한 것

| 기능 | 권한 |
|---|---|
| 준비 상태와 사유, 서비스 상태 확인 | viewer 이상 |
| 서비스 start / stop / restart / retry(로봇 설정이 허용한 것만) | admin |
| 안내 서비스 시작/중지 | Lease 보유자(operator/admin) |
| 목적지 추가(지도 클릭 좌표, 로딩된 지도 버전) | operator 이상 |
| 방문객 링크 발급·폐기, QR 표시 | admin |

기존 기능(수동 조작, 맵핑, 지도 로딩·초기 위치, 지도 클릭 임무, 모드 전환, Gate Reset)은 그대로다(8절 회귀).

## 5. 방문객 웹 — 화면·API·권한

| API(방문객 토큰) | 내용 |
|---|---|
| `GET /visitor/catalog` | 목적지가 있는 시설·건물·층·구역, 서비스 운영 여부 |
| `GET /visitor/destinations?floor_id&area_id&q` | 목적지(이름·설명·가능 여부·사유 문구·대기 수). 좌표·지도 ID 없음 |
| `POST /visitor/requests` | 안내 요청(본인 세션 1건) |
| `GET /visitor/requests/{id}`, `POST …/cancel` | 본인 요청 상태/취소 |

- 방문객 토큰으로 막히는 것: `/robots`, `/dashboard`, `/missions`, manual-bench, gate-reset, mapping, mode, navigation LOAD, missions 생성, services, guide 관리, visitor-links. 모두 **401**(격리 E2E 11개 경로, 단위 12개 경로).
- 폐기된 링크는 401이고, 화면은 "QR을 다시 스캔" 안내로 바뀐다.

## 6. 결과(PASS / FAIL / SKIP)

### 단위
- `test_phase4_readiness` 10, `test_phase4_supervisor` 7, `test_phase5_guide` 7, `test_system_panel.cjs`: 모두 PASS.
- `test_qr_encode`: **SKIP**. 이 환경의 OpenCV는 QR 디코더(QUIRC) 없이 빌드되었고, 다른 디코더는 없다. **QR 출력은 디코더로 검증하지 않았다.**
- `test_manual_control_panel.cjs`(Playwright 경로 지정 시): PASS.
- 전체 단위 96 / 21. 21 FAIL은 변경 전과 같은 환경 의존 목록이다.

### 격리 ROS Phase 4(`final_server_first`, `final_supervisor_first_delay`): 각 8/8 PASS

| 시나리오 | 결과 |
|---|---|
| s01 기동 → READY, READY 전 MAPPING 거부 | 17.9 s / 37.1 s, `robot_not_ready` |
| s02 READY 후 MANUAL, 모터 잠금 | `/cmd_vel` 토픽 없음, `/cmd_vel_locked` writer = Gate 1개 |
| s08 운영자 웹 카드 + 새로고침(재접속) | READY · 서비스 4 · 동일 |
| s03 Watchdog SIGKILL | 감독이 재시작했다. Gate `NOT_EXCLUSIVE` 래치(알려진 DDS 고스트) → 관리자 웹 Gate Reset → 23.2 s 후 READY |
| s04 Agent SIGKILL(MANUAL 세션 중) | 10 s 안에 새 Agent boot. 이전 세션 샘플 409, 모드 IDLE |
| s05 서버 SIGKILL·재기동 | 감독 유지, READY 복귀. 모드 IDLE, 이전 세션 샘플 409, Arbiter source NONE. Lease는 소유권으로 유지(설계) |
| s06 서비스 작업 권한 | operator·viewer 403, 셸 형태 400, 미등록 서비스 404, MANUAL 중 restart 409, IDLE에서 관리자 restart DONE → READY |
| s07 Gate stack 연속 종료 | 4회 후 FAILED → FAULT, NAVIGATION 409. 관리자 retry → stack, Watchdog, Agent 재시작 → READY |

### 격리 ROS Phase 5(`phase5_visitor_claude_20261121/final`): 8/8 PASS

| 시나리오 | 결과 |
|---|---|
| v00 목적지·링크 | 장애물 목적지 409, C103 목적지는 `MAP_NOT_VERIFIED`, operator 링크 발급 403 |
| v01 서비스 닫힘 → 열림(모바일) | 닫힘: 전부 "운영 중이 아닙니다". 열림: 안내데스크·세미나실 가능, C103 "검증되지 않은 지도". 가로 스크롤 없음, 운영 기능 없음, 주소창 토큰 제거 |
| v02 요청 → 도착 | ACCEPTED → IN_PROGRESS → COMPLETED, 목표 오차 0.079 m. 새로고침 후 COMPLETED 유지. 임무 생성자 = operator |
| v03 안내 중 취소 | … → CANCELING → CANCELED, 1.5 s 뒤 Gate 출력 0 |
| v04 방문객 2명 | 두 번째는 QUEUED → … → COMPLETED(FIFO) |
| v05 이용 불가 사유 | MANUAL → `ROBOT_NOT_IN_NAVIGATION`(요청 시 UNAVAILABLE), 지도 없음/C103 로딩 → `ROBOT_ON_OTHER_MAP`, 복귀 → 가능, Lease 변경·재획득 → `OPERATOR_LEASE_CHANGED`, 서비스 재시작 → 가능 |
| v07 운영자 지도 클릭(서버 기준 검사) | 지도 밖·장애물은 "서버 검사: …"로 차단(미관측 지점은 이 지도 반경 안에 없어 미시험). 서버가 허용한 지점을 화면이 거부한 경우 0건. 클릭 임무 SUCCEEDED |
| v06 토큰 격리·폐기 | 운영 경로 11개 401, 폐기 후 401, 화면은 링크 안내 |

### Gazebo / 실차
- **SKIP.** Phase 4/5 기능은 로봇 모델과 무관하게 격리 ROS에서 실제 체인으로 시험했다. Gazebo 재실행은 하지 않았다.

### 실행 중 발견·수정
- Nav2 lifecycle 기동이 실패했는데도 READY로 보고되던 문제 → `NAV2:*` 사유를 추가했다.
- 최초 cascade가 Watchdog 재시작 때 Agent까지 재시작하던 문제 → `restart_with`로 명시하게 바꿨다.
- 하네스 결함: TTL 상한, Gate reset 대기, 전환 대기, 임무 ID 추적.
- 실패한 중간 실행도 원자료로 보존했다(`server_first_run1~4`, `supervisor_first_delay_run1`, phase5 `run1~4`).

## 7. 기존 기능 회귀

최종 코드로 실행했다. 스냅샷은 시작과 끝이 동일하다. 위치는 `local_data/raw_results/refactor35_claude_20261121/e2e_after_phase45`이다.

| 묶음 | 결과 | 포함 기능 |
|---|---|---|
| Phase 1 manual(브라우저) | **19 / 0** | 키보드·포인터·게임패드, Deadman, s10 FollowPath |
| Phase 2 mode(17) | **16 / 0 / SKIP 1** | AUTO/MANUAL/MAPPING 전환, Mapping START/PAUSE/RESUME/SAVE/STOP |
| Phase 3 nav(고정 지도) | **18 / 0** | 지도 선택·초기 위치, 목표 검사, Safety Context 차단 |
| Gate Reset E2E | **13 / 0** | 관리자 Gate Reset·복구 |
| 지도 클릭 Navigation | Phase 5 v07 PASS | 실제 대시보드 클릭, 서버 기준 검사 |

- Phase 2 SKIP 1건: `f06_cancel_unanswered_bt_navigator_frozen`. 사유는 "robot still RECONCILIATION_REQUIRED after f05 (admin recovery refused)"이다. f05(Watchdog kill)의 관리자 복구가 `idle_not_proven`으로 거부되면 하네스가 f06을 건너뛴다.
- 같은 SKIP은 이번 변경 전에도 있었다: `gate_input_claude_20261120/e2e_run03`, `e2e_run04`, `e2e_run05`, `e2e_run08`, `mode_phase2_claude_20261120/e2e_run02`. 이번 변경과 무관한 간헐 현상이며, 원인(Nav2 idle 증명 도착 시점)은 미분석이다.

## 8. 실차 적용을 막는 미해결 조건

1. **기존 안전 미해결 항목은 그대로다**: Gate E 원인, STM32 명령 timeout, E-Stop, 제동거리, Gate 재개 직후 오래된 명령 1회 출력.
2. 감독 설정·systemd 템플릿은 **실차 미검증**이다. 실차 명령(`limo_run.sh` 기준), ROS 도메인 77, 토큰 경로, ydlidar·limo_base 준비 판정 토픽은 실차에서 확인해야 한다.
3. Watchdog 재시작 후 Gate `NOT_EXCLUSIVE` 래치는 관리자 Gate Reset이 필요하다(약 20~40 s). 자동 해제는 하지 않는다(안전 정책).
4. Nav2 lifecycle 간헐 기동 실패: 자동 복구는 없다. readiness가 DEGRADED로 알리고, 관리자가 stack을 재시작한다.
5. QR 코드: 디코더 검증을 하지 않았다. 휴대전화 카메라로 확인하거나 디코더 설치를 승인받아야 한다. 주소 문자열은 항상 함께 표시한다.
6. 방문객 세션 식별은 브라우저 난수로 하며 인증이 아니다. 공개 인터넷 배포 전에 TLS, 네트워크 경계, 링크 배포 정책이 필요하다(공개 배포는 하지 않았다).
7. Open-RMF 연동은 하지 않았다. 방문객 요청은 Fleet 임무로 만든다. RMF 어댑터와의 계약은 다른 작업자와 조율해야 한다.

## 9. Codex 독립 검증 명령

```bash
cd fleet_control/tests && for t in test_phase4_readiness.py test_phase4_supervisor.py test_phase5_guide.py test_qr_encode.py; do python3 $t || echo FAIL $t; done; node test_system_panel.cjs
```

```bash
cd fleet_control && PYTHONPATH=agent/limo_robot_agent python3 -m limo_robot_agent.supervisor --config scripts/autostart/supervisor.limo.example.json --check
```

```bash
local_data/raw_results/phase4_autostart_claude_20261121/final_chain.sh
```

```bash
local_data/raw_results/refactor35_claude_20261121/regress.sh e2e_codex_phase45
```

- `final_chain.sh`는 Phase 4 두 순서와 Phase 5를 순서대로 실행한다.
- `OVERLAY`, Playwright `node_modules` 경로는 이 기계 기준이다. 다른 기계에서는 바꾼다.
- 도메인 226/227/228, 포트 28957/28963/28964는 시험 전용이다.


## 10. 공통 문서

- 갱신: `HANDOFF.md`, `CURRENT_STATUS.md`, `VALIDATION.md`, `CODE_INDEX.md`(각 끝에 이 작업 절 추가).
- **`ARCHITECTURE.md` 변경 제안(직접 수정하지 않음)**: 구성도에 다음 두 경로를 추가하자.
  - `로봇 전원 → systemd → limo_robot_agent.supervisor → {stack(센서·Nav2·Gate), motion_watchdog, robot_agent}`. 감독 보고는 `/api/v1/agent/supervisor`, 준비 상태는 `/robots/{id}/readiness`로 본다.
  - `방문객 휴대전화(QR 링크) → /api/v1/visitor/*(Visitor 토큰) → guide.py → Fleet 임무(운영자 Lease) → 기존 Agent·Arbiter·Watchdog·Gate`.
  - 경계도 함께 적자: 방문객 토큰은 운영 API에 쓸 수 없다. 자동 기동은 모터 구동 허가가 아니다.
