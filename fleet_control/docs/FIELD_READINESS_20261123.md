# 현장 실행 준비 (G2 기준, 2026-11-23)

범위: 노트북의 Fleet Server·웹 관제와 LIMO의 Robot Agent·ROS 스택을 현장에서 연결하고 시험할 수 있는 준비 상태. 실제 LIMO 접속, 설치, 운영 서버 18190 변경, 모터 구동은 하지 않았다(모든 검증은 Mock·격리 ROS·브라우저). 검증 수준을 항상 구분한다: **Mock**(합성 입력) / **격리 ROS**(실제 Nav2·SLAM·Gate·Watchdog·Agent, 합성 센서, `ROS_LOCALHOST_ONLY=1`, /cmd_vel 없음) / **Gazebo** / **실차**.
기록: `local_data/raw_results/real_limo_integ_claude_20261122/field_ready_01/`. 기존 Go/No-Go 문서 [FIELD_LOCKED_READINESS_20261120.md](FIELD_LOCKED_READINESS_20261120.md)와 이 문서가 함께 기준이다.

## 1. 최종 시스템 구성

```
브라우저(운영자) ──HTTP──▶ [노트북] Fleet Server (웹 관제 UI + API, SQLite, Map Registry, 토큰 인증)
                                   ▲ HTTP(1 Hz 상태·명령·결과, 토큰, Tailscale)
[LIMO] Robot Agent (robot_agent) ──┘  ◀── 모든 운동 승인: Lease · 계약 세대 · 제출 Fence · Safety Context
   │ ROS 2 (도메인 77, localhost 전용)
   ├ Nav2 / Mapping Worker(Frontier) / 웹 수동 입력 ──▶ Velocity Arbiter(/fleet/nav_cmd_vel)
   │         ──▶ Motion Watchdog(독립 프로세스, 하트비트·Gate·goal 증거) ──▶ /cmd_vel_raw
   │         ──▶ Safety Gate(stall_detector_node) ──▶ /cmd_vel_locked (모터 잠금) ──▶ (/cmd_vel 은 잠금 해제 승인 전까지 연결 없음)
   └ 센서: LiDAR, 휠 Odom, IMU, Depth/RGB · rosbag(Bag Worker/Supervisor, 격리 구성)
```
- ROS DDS는 LIMO 안에서만 쓴다(localhost). 노트북과 LIMO 사이는 HTTP만이다. 따라서 노트북에는 ROS가 필요 없다.
- 운동 권한은 서버가 아니라 **Agent가 로컬에서** 결정한다(Lease, 계약 세대, Fence, Watchdog, Gate). 서버·네트워크가 끊기면 Agent가 활성 goal을 취소하고 Watchdog이 0을 낸다.

## 2. 설치 대상

| 대상 | 무엇 | 출처(G2 worktree) | 비고 |
|---|---|---|---|
| **노트북** | Fleet Server + 웹 UI | `fleet_control/server/` (정적 파일 포함) | `scripts/field_laptop_up_20261123.sh`로 기동. Python 표준 라이브러리 + numpy/PIL 정도(Map Registry) |
| 노트북 | 점검·수집 도구 | `scripts/field_preflight_20261123.py`, `verify_bag_copy.py`, `register_saved_map.py` | 읽기 전용 HTTP |
| 노트북 | 배포 도구 | `scripts/limo_package_deploy_20261122.sh`(R3), `scripts/atomic_swap_20261123.py` | LIMO로 스트리밍·복사해 실행 |
| **LIMO** | `limo_robot_agent` 패키지(36 파일) | `fleet_control/agent/limo_robot_agent/` → 후보 `deploy_candidate_agent_g2/` | `robot_ws`에 빌드(`colcon build --packages-select limo_robot_agent`). 진입점 `robot_agent`, `motion_watchdog`, `mock_robot` |
| LIMO | 팀 ROS 워크스페이스(변경 없음) | `capstone_ws`의 `limo_capstone`(Gate `stall_detector_node`, 실차 launch), `limo_base` 등 | 현장 설치본 유지. G2에서 main이 바꾼 launch·파라미터는 **LIMO에 올리지 않는다**(별도 승인·검증 전) |
| LIMO | 자동 기동 | `scripts/autostart/limo-supervisor.service` + `supervisor.limo.example.json` | 1회 설치 후 SSH 없이 기동(§7). **현재 미설치** |
| LIMO에 불필요 | `limo_fleet_bringup`(격리 시험용), `fleet_control/tests`, `server/` | | |

**기존 실차 설치본과 G2의 차이**(LIMO의 `limo_robot_agent`):
- 현재 LIMO: v3(11-22 Stage A에서 v4 준비, v4 배포 여부는 보고 기준 미확인 — 현장에서 `FILES.sha256`로 확인).
- G2 후보 `limo_robot_agent_candidate_g2_20261123.tar.gz`(36 파일, SHA-256 `0cb0488891d4ff1fd835b1860946cf62404bed979d5c0141830e12428901b43a`, 두 번 생성 바이트 동일, 소스는 미커밋 G2 작업트리):
  - v5b 대비 추가 4개: `bag_worker.py`, `bag_supervisor.py`, `owned_bag.py`, `camera_preview.py`. 변경 5개: `agent_node.py`, `ros_adapter.py`, `safety_protocol.py`, `manual_web_bench.py`, `package.xml`. 나머지 27개 동일.
  - v4 대비: R1 계약(필수 Lease, 세대, Fence, 철회 시 취소 의무), R2 Watchdog 종료 안전, Frontier goal의 R1 권한, 수동 bench v5 프로토콜.
- **서버가 먼저**: v5 Agent는 R1.4.5 이후 서버(명시 `reconciliation_required:false`, lease 키)가 있어야 운동 승인을 받는다. 구서버에서는 fail-closed로 모두 거부된다.

## 3. 웹 관제에서 가능한 동작 (검증 수준 포함)

| 기능 | Mock/단위 | 격리 ROS | Gazebo | 실차(현재) |
|---|---|---|---|---|
| 로봇 Online/Offline, 센서·Gate·모터 경로·Watchdog·배터리 표시 | PASS | PASS(UI 브라우저 확인, 프로브 §5) | 이전 기록 | **사용 가능(읽기)** — 11-19 잠금 실차 관측 |
| Lease 획득·갱신·반납 | PASS | PASS(UI 버튼, API) | — | 사용 가능(명령 효과 없음) |
| Mapping START/PAUSE/RESUME/SAVE/STOP | PASS | PASS(UI의 START·PAUSE·SAVE, API 전체) | 이전 기록 | **차단**: 서버 `mapping_real_blocked` + Agent `REAL_LOCKED` 정책 |
| 수동 제어(웹 게임패드, WASD, 조이스틱) | PASS | PASS(축 입력 → Arbiter MANUAL, Deadman 만료 시 0) | — | 영(0) 출력 bench만 허용. 실제 출력은 차단 |
| 지도 업로드·등록·조회 | PASS | PASS(저장 → 레지스트리 번들) | — | Registry 등록은 승인 필요 |
| 합성 지도에서 Nav2 목적지 지정 | PASS | 부분 PASS(§8): 모드 E2E에서 NAVIGATION 목표 성공·취소 PASS, 저장 지도 E2E에서 로드·초기 위치·검증·취소 PASS, 유효 목표 성공은 FAIL(Nav2 abort) | — | 차단(missions 비활성, 모터 잠금) |
| 안내·순찰 임무 생성·취소·완료 | PASS(서버 로직) | **미실행**(Playwright 없음) — 임무 생성·취소·완료는 위 Nav2 E2E와 같은 경로 | — | 차단 |
| 오류 원인·차단 표시 | PASS | PASS(차단 사유가 상태에 표시: `VELOCITY_GRAPH_NOT_LOCKED`, `WATCHDOG_NOT_AUTHORIZED` …) | — | |
| rosbag 시작/종료(Field Ops 탭) | PASS | PASS(API: START→STOP, 검증된 metadata) | — | 격리 구성만(실차 recording 차단) |
| 카메라 RGB/Depth 미리보기 | PASS | PASS(UI에 표시) | — | 격리만 |

**G2에서 UI에 추가한 것**: 콘솔에 `Field Ops` 탭(main의 Operations·게임패드·카메라 패널). 브라우저로 확인해 패널이 쓰는 전역 함수 3개(`fillSelect`, `drawMap`, `uuid`)가 없어서 패널이 죽는 결함을 찾아 `legacy_panels_shim.js`로 고쳤다.

## 4. 실차에서 막혀 있는 것과 이유 (의도된 코드 수준 차단)
이 항목들은 설정으로 풀리지 않는다. 풀려면 **물리 증거 + 사용자 승인 + 해당 코드 변경과 재검증**이 필요하다. 이번 작업에서 풀지 않았다.
1. `real_locked_policy.py`: REAL_LOCKED는 "독립 검증된 물리적 모터 격리 증거 생산자가 없다"는 이유로 무조건 차단.
2. 서버 `_mapping_real_block` / `mapping_real_blocked`: 실차 Agent의 Mapping·MAPPING 모드 거부. NAVIGATION 모드는 Agent `missions_enabled:=true` 필요.
3. 실차 Agent의 `allow_missions:=false`, 수동은 영 출력 bench, 실차 recording 차단.
4. 모터: `allow_motor_drive:=false`(launch 기본)로 `/cmd_vel` 연결이 없다.

**잠금 해제 전 선행 조건(체크리스트, 모두 미완료)**: 물리 E-Stop과 실제 제동거리 측정, 전원·USB/UART·STM32 watchdog 원인 분리(11-19: 5분 관측에서 /odom·/imu 0 Hz), `/cmd_vel` 우회 경로 차단 검증(드라이버 쪽), Depth 연동, 실차 Mapping Worker 허용 정책과 서버 차단 해제의 코드 변경, 서버 선배포와 Agent 실차 배포 승인, 모터 잠금 스택에서 Mapping STOP·Arbiter·Watchdog·Gate·DDS 현장 검증.

## 5. ROS 안전 체인 검증 (격리 ROS, G2 소스, `probe_field_chain_20261123.py`)
체인: Nav2 / Frontier / Manual → Arbiter → **Watchdog** → Gate → `/cmd_vel_locked`. 20/20 PASS(`p8_chain_probe_3.log`):
- 그래프: `/cmd_vel_raw`는 Watchdog만 발행하고 Gate만 구독, `/cmd_vel`은 발행자·구독자 없음, `/cmd_vel_locked`는 Gate만 발행하고 구독자 없음.
- MAPPING 모드: Arbiter가 Nav2(Frontier) 소스를 선택(Watchdog·Lease·Gate 조건 충족 시), STOP은 goal terminal 증거와 함께 완료.
- MANUAL 모드: 게임패드 축 입력 → 양의 값이되 0.10 m/s·0.30 rad/s 이내, 입력이 끊기면 0으로 감쇠(Deadman TTL).
- 고장 주입: 외부 `/cmd_vel` 발행자 → 모터 경로 ARMED, Arbiter `VELOCITY_GRAPH_NOT_LOCKED`로 정지, Mapping 세션 철회. Watchdog 프로세스 정지(SIGSTOP) → 상태가 1초 넘게 낡고 Arbiter `WATCHDOG_NOT_AUTHORIZED`. Lease 반납 → Arbiter 준비 해제.
- main의 bringup은 Nav2 출력을 Gate 입력에 직결해 Watchdog을 우회했다 → G2에서 Watchdog·Arbiter를 거치도록 고침(정적 시험 3/3, 위 그래프 시험으로 확인).
- 코드 시험(오프라인): R1 필수 필드·Lease·세대·Fence, Frontier R1 권한, UNKNOWN 세션, R2 Watchdog 66/66, Agent 재시작·재조정(Recovery), 루프백 E2E 36/36 등은 G2_INTEGRATION 문서 §4, §6.
- **미검증**: 실제 DDS 지연·유실, 통신 단절 중 실제 정지 거리, Gate 독립 sensor 이상의 실차 재현, 물리 정지.

## 6. 배포·롤백
**순서(별도 승인 항목)**: ① 노트북 서버를 새 세션 디렉터리로 기동(`field_laptop_up_20261123.sh`), 서버 응답 계약 확인 → ② LIMO에서 `preflight` → `backup`(R3) → 백업 3종을 노트북으로 복사·해시 대조 → ③ 후보 tar·`TAR.sha256`·`FILES.sha256` 복사·대조 → ④ `stage` → **`atomic_swap_20261123.py replace`** → `build` → `verify` → ⑤ Agent를 모터 잠금 구성으로 기동, `field_preflight --real` 통과 확인.
**R3 이중 실패 해결(`atomic_swap_20261123.py`, 오프라인 검증)**: R3는 `mv SRC OLD; mv STAGE SRC` 두 번의 이동이라 중간에 SRC가 없고, 둘째 이동과 보상이 모두 실패하면 패키지가 사라졌다. 새 도구는 `renameat2(RENAME_EXCHANGE)` 한 번으로 SRC와 stage를 맞바꾼다 — SRC가 항상 존재하고(시험: 300회 교환 동안 감시 스레드가 부재를 한 번도 보지 못함), 교환이 불가능하면 아무것도 바뀌지 않으며, 교환 후 검증이 실패하면 같은 호출로 되돌리고, 되돌림마저 실패해도 SRC는 존재하며 이전 트리는 stage 경로에 보존된다. 심볼릭 링크, 다른 파일시스템, 로봇 프로세스 실행 중, FILES.sha256 불일치는 변경 전에 거부한다. 시험 10/10. 롤백도 같은 도구(`rollback`: 백업을 추출한 트리와 교환).
**리허설(스크래치 워크스페이스)**: v5b → G2 교환, `colcon build`, 진입점 3개·신규 모듈 import, 다시 v5b로 교환, 두 방향 모두 `FILES.sha256` 일치(`p10_swap_rehearsal.log`). R3 도구 자체는 바꾸지 않았다(기존 흐름의 `backup`/`stage`/`build`/`verify`와 함께 사용).
**남은 위험**: 도구를 실제 LIMO 파일시스템에서 실행해 보지 않았다(ext4 가정, 같은 파일시스템 stage 필요), `colcon build` 실패 시 되돌림은 `rollback` 후 재빌드로 수행한다.

## 7. 현장 실행·종료 절차 (SSH 최소화)
**1회(승인 후) 설치**: LIMO에 후보 배포(§6) → `limo-supervisor.service` 사용자 유닛 설치와 `loginctl enable-linger` → `~/.config/limo_fleet/`에 Agent 토큰(0600)과 `supervisor.json`(`supervisor.limo.example.json`에서 서버 주소·토큰 경로만 채움). 이후 전원 투입만으로 supervisor가 스택(`limo_integrated_auto_mapping_real.launch.py` 모터 잠금 기본값)·Watchdog·Agent를 순서대로 올린다. 이 흐름은 격리에서 검증했다(Phase 4, §8 결과). 자동 기동은 **운동 권한을 주지 않는다**.
**현장 당일(노트북에서 3단계)**:
1. `scripts/field_laptop_up_20261123.sh SESSION_DIR PORT pcu` → 콘솔 주소 확인, `operator.token`을 콘솔에 입력(화면에 출력하지 않음).
2. LIMO 전원 투입 → 1~2분 뒤 콘솔에서 `pcu` Online. 터미널에서 `python3 scripts/field_preflight_20261123.py --server http://127.0.0.1:PORT --token-file SESSION_DIR/secrets/operator.token --robot pcu --real` → FAIL 0이어야 다음 단계(Gate NORMAL, 모터 경로 LOCKED, Watchdog 승인·신선, 센서, Agent 계약). 시험 5/5.
3. 콘솔에서 Lease 획득 → 잠금 상태에서 가능한 항목만 확인(§3의 실차 열). 맵·로그는 Agent가 서버로 올리는 지도 번들이 Registry에 등록되고, rosbag·로그는 LIMO에 남으므로 종료 후 `verify_bag_copy.py`로 노트북에 사본을 만들고 해시를 대조(기존 메모리 지침: 실차 원본은 노트북에도 보관).
**정상 종료**: 콘솔에서 Lease 반납 → LIMO 전원 종료(supervisor가 SIGINT로 역순 정지) → 노트북 서버 Ctrl+C. 어느 단계에서든 서버가 먼저 꺼져도 Agent·Watchdog은 로컬에서 정지·0 출력한다.
**통신 장애**: 서버/네트워크 단절 → Agent가 활성 goal 취소(3 s 이내 기준 R1), 새 운동 승인 불가, 복구 후 `reconciliation`을 거쳐야 재개. Agent 재시작 → 새 boot·세대, 이전 명령 폐기.
**복구**: UNKNOWN 상태는 운영자 RECONCILE(콘솔), 배포 실패는 §6 롤백.

## 8. 격리 E2E 결과 (G2 소스·overlay, `tests/mode_e2e`, `nav_e2e`, `phase4`; 기록 `p9*_*.log`)
하네스는 HTTP(웹 UI와 같은 API)로만 조작하고 ROS는 읽기 전용으로 관찰한다. 필요 환경: ROS Humble overlay(G2의 limo_robot_agent·limo_capstone·explore_lite·limo_fleet_bringup), `limo_msgs`(이 노트북의 기존 빌드를 source), 후보 Gate 스크립트 `experiments/safety_chain_fix_20261113/candidate_gate_r10`(staging 저장소에만 있는 미추적 파일, 읽기 전용 사용), Map Registry의 시설·층·영역 항목. C103 실지도 아티팩트(`limo_runs/20261119_auto_map_depth`)는 이 노트북에 없어서 같은 ID의 **합성 시설·층·영역 항목**만 하네스 밖에서 만들었다(지도 내용은 하네스가 웹 Mapping으로 직접 만든 것).
| 하네스 | 결과 | 비고 |
|---|---|---|
| Phase 2 모드 전환·Mapping(`run_mode_e2e`) | **17 PASS / 3 FAIL** | PASS: 시작·잠금 확인, IDLE→MANUAL 주행·정지, MANUAL→IDLE 0.5 s 내 0, IDLE→MAPPING·탐색, PAUSE·SAVE·RESUME, 모드 직접 전환, Lease 상실·서버 재시작·Lease 만료 후 RECONCILE·Agent 재시작, **NAVIGATION 목표 성공·취소·MANUAL 전환**, Watchdog 강제 종료 중 정지, bt_navigator 정지 중 취소 무응답, `/cmd_vel` 끝점 없음. FAIL 3개는 아래 Gate 입력 항목 |
| Phase 3 저장 지도 → Nav2(`run_nav_e2e`) | **14 PASS / 4 FAIL** | PASS: 웹 Mapping으로 지도 생성·저장·검증, LOAD, INITIAL_POSE 확인, 목표 검증, 취소, 버전 불일치·손상 지도 거부, 틀린 초기 위치 미확정, planner 정지·AMCL 종료·Gate 종료 시 정지, 재위치. FAIL 4: n03 유효 목표가 SUCCEEDED가 아니라 FAILED(Nav2가 goal을 abort, 109개 경로 점 확인), n05 두 번째 목표 FAILED, f08 Agent 강제 종료 후 복구 확인, f07b Gate 입력 우회. n03·n05는 시뮬레이터 Nav2 동작이며 원인 미확정, v5b 기준선과 비교하지 못했다(기준선 실행은 p01에서 다른 사유로 실패) |
| Phase 4 supervisor SSH-free 기동(`run_phase4`) | **7 PASS / 0 FAIL / 1 SKIP(브라우저)** | server-first 기동·Gate 전 준비 거부, Watchdog 강제 종료 후 supervisor 재기동(7.6 s), Agent 강제 종료 후 이전 세션 재사용 거부, 서버 재시작, 역할별 서비스 조작(admin만), 반복 크래시 → FAULT 후 admin 재시도 |
| Phase 5 안내·순찰(`run_phase5`) | **미실행** | Playwright(브라우저 자동화)가 이 노트북에 없다. 안내·순찰 임무의 서버 로직 시험(`test_phase5_guide`)은 PASS이지만 UI·Nav2 결합 E2E는 G2에서 **미검증** |

**이 E2E가 G2에서 잡은 결함(수정함)**: ① UNKNOWN Mapping 세션의 활성 goal을 처음에 `reconciliation_required`로 막았더니 운영자가 Lease를 못 잡아 RECONCILE 불가(f03 교착) → 추적만 하도록 수정, ② main에서 가져온 "bench Agent는 목적지 임무 금지" 검사가 v5의 NAVIGATION(잠금 Arbiter + missions)까지 막음 → 임무가 꺼진 bench Agent에만 적용, ③ 웹 Field Ops 패널의 전역 함수 누락(브라우저에서 발견).

**남은 실패와 그 의미**
- **Gate 입력 우회(g01, g02, g03, f07b)** — `/cmd_vel_raw`(Gate 입력)에 외부 발행자가 쓰면 Gate가 그 값을 통과시킨다(Gate 상태 NORMAL 유지, 0.3 s 뒤에도 비영 샘플). Watchdog 입력(`/fleet/nav_cmd_vel`)의 외부 발행자는 `VELOCITY_INPUT_NOT_EXCLUSIVE`로 잡힌다(f07 PASS). 하네스가 쓰는 후보 Gate r10과 저장소의 Gate(`stall_detector_node`)에는 입력 출처 검증(`GATE_INPUT_EXCL`, Gate r11+)이 없다. **[갱신 36] G2 Gate와 bringup에는 입력 출처 검증이 적용되어 격리 시험에서 g01/g02/f07b가 통과한다(`GATE_INPUT_SOURCE_G2_20261123.md`). 실차 launch에는 Watchdog이 없어 아직 꺼져 있고 g03은 미검증이다.** 모터 잠금이 풀리기 전에 반드시 해결해야 하는 차단 항목이었다(제안 패치는 `docs/proposals_20261120/05_gate_input_source_validation.patch`).
- n03·n05·f08: 위 표 참고(시뮬레이터 Nav2 abort와 Agent 강제 종료 후 복구 확인). 실차 안전과 직접 관련은 없지만 해결 전에는 "합성 지도에서 목적지 이동 성공"을 완전히 PASS로 쓰지 않는다.
