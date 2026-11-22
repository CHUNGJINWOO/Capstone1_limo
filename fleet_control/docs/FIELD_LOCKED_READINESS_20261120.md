# 실차 LIMO 웹 관제 — 모터 잠금 준비와 Go/No-Go (2026-11-20)

목표: 노트북 18190 관제에서 실제 LIMO의 상태를 보고, 지도·맵핑·목표 명령이 **안전하게 거부·처리되는지** 모터 잠금으로 확인한다. 실제 주행은 이 문서 범위 밖이며 별도 사용자 승인이 필요하다.

## 1. 기준선 (2026-11-20 오전 확인)

| 항목 | 상태 |
|---|---|
| 저장소 | `staging/fleet-control-on-progress-c804241` @ `c804241`, 단일 worktree. `fleet_control/` 미추적. 다른 에이전트 미커밋 파일(Gate, 실차 launch, Depth 필터, explore CMake 등) 미수정 |
| LIMO (`pcu`) | tailnet **offline**. 실차 프로세스·배포본 조사는 연결 후 수행(아래 §4 단계 1) |
| 18190 | **미실행**. 현장 런처(`experiments/real_limo_field_20261113/laptop/lf_t1_server.sh`)의 `FLEET_SRC`(`~/.config/limo_field_20261113/kit.env`)가 **존재하지 않는 경로**(`…/Capstone1_limo_video_preverify/fleet_control`)를 가리킨다. 그대로는 기동 실패 |
| 현장 data | `~/.local/share/robot-sim-projects/limo_field_20261113/data`. 11-19 B0 백업과 바이트 동일. Registry 비어 있음(시설·지도 0). 스키마는 신규 area/lineage 테이블 이전 버전 |
| 백업 B1 | `local_data/field_registry_backups_20261120/data_B1` (0700, `auth.json` 포함 — 공개 금지). 원본과 SHA-256 7/7 일치, 사본에서 `integrity_check` ok(두 DB) |
| 실차 배포본 | Agent: `~/limo_field_agent_ws`(11-18). ROS: `/home/pcu/Capstone1_limo/robot_ws`(11-19, 9개 패키지). **오늘의 watchdog·Agent 변경은 미배포**. 실차 launch에는 watchdog 없음(패치 02b 미적용) |

## 2. 웹 기능 상태 (구현 / 검증 / 실차 차단)

| 기능 | 구현 | 검증 | 실차(오늘) |
|---|---|---|---|
| 상태 표시(Online, LiDAR, Depth, TF/pose, 지도, Gate, 모터 경로, Nav2) | 있음 | 격리·잠금 실차(11-18/19) | **사용 가능**(읽기) |
| C103 지도 표시·선택 | 있음(Registry, area PARTIAL_FLOOR) | B1 사본 리허설 PASS | 현장 Registry 등록 **승인 필요** |
| Lease 획득·갱신·해제 | 있음 | 격리 PASS | 사용 가능(명령 효과 없음) |
| Mapping START/PAUSE/RESUME/SAVE/STOP, NEW_MAP | 있음 | 격리 22/22, 고장 시험 | **차단**: 실모드 Agent는 Worker 없음 → START 409(`worker_unavailable`), 서버도 `mapping_real_blocked` |
| CONTINUE_MAP | 서버 거부 | 기계적 경로만 | 차단(미구현) |
| 지도 클릭 → 목표 좌표 변환·사전 검사 | 있음(브라우저) | Node 11/11 | 좌표 표시만. 제출은 아래에서 차단 |
| Fleet Mission 생성 → Nav2 goal 제출·취소 | 있음 | 격리·스텁 | **차단**: Safety Context 생산자 없음(Gate attestation·localization CONFIRMED·verified map) → 생성 409, Agent도 `GATE_ATTESTATION_UNAVAILABLE` |
| Agent 재시작 | 있음(재실행 없음) | 격리 PASS | 사용 가능(정지 상태 확인용) |
| 운영자 Recovery | 실모드 항상 거부(`terminal_evidence_unavailable`) | 스텁 | **경로 없음(P1)**: 실행 중 Agent를 잃으면 API로 해제 불가 |

## 3. 안전 적용 판정 (실차)

| 조건 | 상태 |
|---|---|
| Watchdog → Gate → Base 속도 경로 | 격리 PASS(개정 Gate 03b+04 사본, §SAFETY_WATCHDOG 6). **실차 launch 미적용** → FAIL(미배포) |
| 직접 `/cmd_vel` 우회 없음 | 실차 launch 내 PASS(코드 검토). launch 밖 벤더 데모 3개가 `cmd_vel` 직접 발행 가능 — 실행 금지·실차에서 그래프 확인 필요 |
| STM32 명령 timeout / Gate 종료 시 정지 | **미측정** (`limo_driver.cpp`에 timeout 없음) |
| 물리 E-Stop | **미확인** |
| 제동 시간·거리 | **미측정** |
| Depth 기준 높이·낮은 장애물 | **미측정**(`base_footprint_z` 기본 0.0, 추정 0.145 m 차이) |
| 초기 위치 추정 / 재시작 후 goal 소유권 | localization CONFIRMED 생산자 없음. 소유권은 격리 PASS |
| 서버 단절 4.4 s·Lease 만료 3.3 s | 감독 상실 시간(Gate·watchdog·Nav2 회피는 동작). 0.15 m/s에서 최대 약 0.66 m. **허용 기준 미결정** — 제안: Lease 만료 시각에 맞춘 fence(만료 즉시 정지), 서버 단절 fence 1.5 s |
| 바퀴 odom 중단(`wheel_odom_stale`) | 11-18·11-19 반복 관측, 물리 원인 미확정 — **FAIL** |
| 비정상 종료 후 운영자 복구 | 실모드 경로 없음 — **FAIL(P1)** |

**판정: 실차 자율주행 No-Go.** 오늘은 모터 잠금 시험만 진행한다.

## 4. 오늘 모터 잠금 시험 절차 (사용자 연결 후)

승인 지점은 ★로 표시한다. ★ 없이 운영 데이터·현장 서비스·실차 배포를 바꾸지 않는다.

1. **실차 조사(읽기 전용)**: SSH로 실행 중 ROS 노드·launch·`/cmd_vel` 발행자/구독자, 배포본 경로·해시, 디스크 여유. 다른 프로세스 종료 없음.
2. ★ **18190 기동 방식 결정**: (a) 이 저장소 `fleet_control/`로 현장 data를 그대로 사용 — 신규 스키마(area/lineage 테이블)가 현장 Registry에 **추가 마이그레이션**된다(B1로 복원 가능), (b) B1 사본 data로 기동(현장 data 무변경, 토큰은 현장 토큰 사본).
3. ★ **C103 현장 등록**: `python3 fleet_control/scripts/register_c103_map.py --data-dir <선택한 data>` (멱등, UNVERIFIED, floor active map 아님). 리허설 결과 동일 예상.
4. 실차 bring-up(모터 잠금): LIMO에서 기존 `robot_ws` 런처, `allow_motor_drive:=false`. rosbag2 기록을 노트북에서 먼저 시작.
5. Agent 연결(기존 실차 Agent 또는 ★ 오늘 버전 재배포 — 재배포 시 기존 `~/limo_field_agent_ws`를 tar 보존 후).
6. 확인: `python3 fleet_control/tests/real_locked_probe.py --url http://127.0.0.1:18190 --robot <id> --token-file <viewer> [--operator-token-file <operator> --refusal] --out <run>/probe.json`
   - Online, LiDAR, Depth, pose(map), 지도, Gate 신선도, 모터 경로 LOCKED, movement_permitted false, Nav2 ready, C103 표시, START 거부, Lease 해제.
7. 브라우저 육안: 지도·C103 영역·Gate/Watchdog 카드·START 거부 메시지.
8. 종료: launch SIGINT → 노드 부재 확인 → rosbag 정지·`ros2 bag info` → 로봇 로그/지도/bag을 노트북 `local_data/raw_results/limo_runs/<run_id>/`로 복사·SHA 확인(로봇 원본 유지) → 18190 Ctrl+C.

## 5. 저속 주행 Go/No-Go 전제 (별도 승인)

모두 PASS여야 Go 후보: 패치 02b·03b·04 적용·실차 잠금 확인, STM32 timeout 실측(Gate SIGKILL 시 바퀴 정지 시간), 물리 E-Stop 동작 시험, 제동 거리(0.15 m/s), `base_footprint_z` 실측 반영, 바퀴 odom 연속성(30분 무중단), localization 확인 절차, 서버 단절·Lease 허용 기준 결정, 운영자 복구 절차.

## 6. 오늘 운영 절차 — (a) 백업 사본 사용 (2026-11-20 오후, 실행 중)

### 6.1 18190 경로 부재 원인

현장 런처 `experiments/real_limo_field_20261113/laptop/lf_t1_server.sh`는 `~/.config/limo_field_20261113/kit.env`(11-13 작성)의 `FLEET_SRC`를 쓴다. 그 값은 `Capstone1_limo_video_preverify/fleet_control`(브랜치 `exp/video-ai-preverify` worktree)인데, 이 worktree는 코드가 `fleet_control/`로 통합된 뒤 11-14에 `git worktree remove`로 해제됐다(WORKTREES.md). `kit.env`와 런처는 수정하지 않았다. 대신 `fleet_control/scripts/run_field_server.sh`(신규)를 쓴다.

### 6.2 현재 상태

| 항목 | 값 |
|---|---|
| 서버 | 이 저장소 `fleet_control/server`, 포트 **18190**, 바인드 127.0.0.1 + 노트북 Tailscale IPv4 (0.0.0.0 아님) |
| 데이터 | `local_data/field_run_20261120_b1/data` = B1 사본 + C103 등록. 현장 data·B1 원본은 미변경(SHA 확인) |
| 인증 | 현장 `auth.json` 그대로(viewer·operator·admin·agent-pcu·rmf-adapter). 토큰 없음/틀림 → 401 확인 |
| C103 | 배재대학교 / C관 / 1F / C103 인근 라운지(PARTIAL_FLOOR), `c103_lounge_20261119_depthmem#959a701c0974`, UNVERIFIED, floor active 아님. 센서 출처: SLAM 래스터 LiDAR(정규화 스캔)만, Depth는 SLAM 입력 아님. 맵핑 중 회피 Depth·기억 노드 사용(기억 노드 소스 미보존 명시) |
| 로봇 | `pcu` 등록, **offline**(tailnet offline, `tailscale ping` 무응답) |
| 웹 화면 | 브라우저 로그인은 토큰 입력이 필요해 Claude가 하지 않았다. 등록·표시 데이터는 API로 확인 |

### 6.3 노트북 명령 (저장소 루트에서)

```bash
fleet_control/scripts/run_field_server.sh local_data/field_run_20261120_b1/data 18190
```

- 주소: `http://127.0.0.1:18190/` (로봇은 자기 `~/.config/limo_field_20261113/field.env`의 `FIELD_SERVER` = 노트북 Tailscale 주소:18190)
- 로그인 토큰 파일: `~/.local/share/robot-sim-projects/limo_video_ai_preverify/real_limo_kit/secrets/fleet_viewer.token`(보기), `fleet_operator.token`(Lease·명령)
- 종료: 포그라운드면 Ctrl+C. 백그라운드로 띄운 경우 `ss -ltnp | grep :18190`의 pid에 `kill -INT <pid>`.
- 점검(LIMO 접속 후):

```bash
python3 fleet_control/tests/real_locked_probe.py --url http://127.0.0.1:18190 --robot pcu --token-file ~/.local/share/robot-sim-projects/limo_video_ai_preverify/real_limo_kit/secrets/fleet_viewer.token --operator-token-file ~/.local/share/robot-sim-projects/limo_video_ai_preverify/real_limo_kit/secrets/fleet_operator.token --refusal --out local_data/field_run_20261120_b1/probe_locked.json
```

### 6.4 LIMO 연결 조건과 명령 (모터 잠금)

연결 조건: LIMO 전원·Tailscale `pcu` online, 노트북 18190 실행 중, 물리 정지 준비, 바퀴가 바닥에서 떠 있거나 공간 확보. **`allow_motor_drive:=true`, `frontier`, `--drive` 금지.**

LIMO 터미널 A(스택, 기존 11-13 번들 기준 — 11-19 `robot_ws` 런처를 쓸 경우에도 `allow_motor_drive:=false`):

```bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
cd ~/real_auto_mapping_ready_20261113
./start_real_auto_mapping.sh --depth false     # 프롬프트: START HARDWARE LOCKED, READY 후 status
```

LIMO 터미널 B(Agent, 상태 전송만·임무 거부):

```bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
source ~/real_auto_mapping_ready_20261113/env.sh
source ~/.config/limo_field_20261113/field.env
ros2 run limo_robot_agent robot_agent --ros-args -p server_url:="$FIELD_SERVER" \
  -p token_file:="$HOME/.config/limo_field_20261113/agent-pcu.token" \
  -p robot_id:=pcu -p mode:=real -p simulated:=false -p allow_missions:=false \
  -p state_dir:="$HOME/.local/state/limo_robot_agent_field"
```

로봇 Agent는 11-18 배포본이며 오늘의 watchdog 보고·`goal_uuids`는 없다(재배포는 별도 승인). 로봇 launch에는 watchdog이 없다(02b 미적용).

종료 순서: Agent Ctrl+C → 스택 `stop` → 로봇에서 잔류 프로세스 확인(`ps -eo pid,etime,args | grep -E "limo|ydlidar|astra|slam|nav2|stall" | grep -v grep`) → 로그·bag을 노트북 `local_data/raw_results/limo_runs/<run_id>/`로 복사·SHA 비교(로봇 원본 유지) → 18190 종료.

### 6.5 복구

- 서버 이상: 18190 종료 후 같은 명령으로 재기동. 데이터가 의심되면 B1에서 **새 이름**의 사본(`local_data/field_run_20261120_b1_r2/data`)을 만들고 C103을 다시 등록해 그 경로로 기동한다(기존 사본은 증거로 보존, 삭제 안 함).
- 현장 data는 이 절차에서 쓰이지 않는다(`run_field_server.sh`가 `ALLOW_FIELD_DATA=1` 없이는 거부).

### 6.6 (b) 현장 Registry 반영 — 승인 후에만

범위: 현장 `map_registry.db`에 신규 테이블(map_areas, map_version_areas, map_version_lineage) 생성 + C103 시설/건물/층/영역/지도 버전 1개(UNVERIFIED) + `map_store/` 번들 1개. `fleet.db`·`auth.json` 변경 없음(서버 기동 시 정상 기록은 발생).
절차: 18190 종료 → 현장 data 새 백업 B2(복사·SHA·integrity) → `python3 fleet_control/scripts/register_c103_map.py --data-dir ~/.local/share/robot-sim-projects/limo_field_20261113/data` → `ALLOW_FIELD_DATA=1 fleet_control/scripts/run_field_server.sh ~/.local/share/robot-sim-projects/limo_field_20261113/data 18190`.
복원: 서버 종료 → 현재 현장 data를 보존 이름으로 이동 → B2 사본을 원래 경로로 복사 → SHA 비교. (현장 파일 덮어쓰기이므로 복원도 승인 후 수행.)

## 7. 실차에서 Mapping·지도 클릭 Goal을 쓰기 위해 남은 조건 (코드 기준, 2026-11-20 저녁)

### 7.1 Mapping START/PAUSE/RESUME/SAVE/STOP

현재 실차에서 막는 지점(모두 의도된 차단, 유지):

| # | 차단 지점 | 위치 | 해제에 필요한 것 |
|---|---|---|---|
| M1 | 웹 버튼 비활성 "Mapping Worker 없음" | `static/mapping_panel.js` | M2 해소 시 자동 |
| M2 | Agent가 `web_mapping_enabled`를 SIM/mock·localhost 격리에서만 허용 | `agent_node.py` | 실차용 Worker 실행 조건 설계(아래 전제 충족 후) |
| M3 | 서버 `mapping_real_blocked` | `server/fleet_server/mapping.py` | 실차 안전 조건 충족 + 사용자 결정 |
| M4 | Worker `locked()`: 모터 경로 LOCKED(또는 SIM_GATED)일 때만 goal 제출 | `mapping_worker.py` | 실제 주행 맵핑은 "ARMED 경로 허용" 규칙이 필요: watchdog(02b)·Gate 요구(03b) 적용 + Gate attestation(아래 G1) |

M1–M3만 풀면 모터 잠금 상태에서 실차 SLAM·Frontier·웹 수명주기를 검증하는 **"잠금 리허설"**은 기술적으로 가능하다(로봇은 움직이지 않고 지도도 자라지 않음). 이것도 차단 해제이므로 **사용자 승인 전에는 하지 않는다.**

실제 이동 맵핑 전제: 02b·03b·04 적용·실차 확인, STM32 timeout·E-Stop·제동 거리 실측, 바퀴 odom 연속성, Depth 기준 높이, `limo_scan_normalizer`·기억 노드의 정식 패키지 편입(현재 `experiments/`에만 있음 → SLAM 스캔 품질 재현 불가).

### 7.2 지도 클릭 Goal (Fleet Mission → Nav2)

| # | 조건 | 현재 |
|---|---|---|
| G1 | Gate attestation(Gate `boot_id`, `fault_epoch`, `movement_permitted`) | 실차 Gate(`stall_detector_node.py`)가 발행하지 않음. Agent는 실모드에서 `GATE_ATTESTATION_UNAVAILABLE`로 항상 거부 |
| G2 | Operation mode producer (`NAVIGATION`, revision) | **없음**(`context_fn` 미연결 → UNKNOWN) |
| G3 | Map producer: 로봇에 적재된 지도 = Registry 버전, `map_verified` | 없음. C103은 UNVERIFIED, verify 경로는 서버에서 미라우팅(정책 미정) |
| G4 | Localization producer: 그 지도 기준 `CONFIRMED`(초기 위치 절차 포함) | 없음 |
| G5 | `allow_missions:=true` Agent | 현장 Agent는 false(의도) |
| G6 | 모터 경로 규칙: 실주행 시 `/cmd_vel` 발행자(Gate)가 있으면 ARMED → 거부 | 실차 ARMED 허용 규칙 없음(G1·02b·03b와 함께 설계 필요) |
| G7 | Watchdog 필수(`require_motion_watchdog`) | 실차 launch에 없음(02b) |

지도 클릭 → 좌표 변환·여유 검사(브라우저)는 동작한다. 제출은 서버(`safety_not_ready`)와 Agent에서 막힌다.

### 7.3 오늘 처리한 안전 통합 (fleet_control, 격리 검증)

- **Lease 만료 정지 단축**: 서버 poll 응답에 `renew_valid_s`(= min(3 s, Lease 남은 시간)), Worker fence를 요청 **송신 시각** 기준으로 설정 → Lease 만료 후 Gate 출력 0: **3.31 s → 0.28 / 0.31 / 0.41 s**(3회), goal 종료 0.10–0.19 s.
- **서버 단절 정지 단축**: poll HTTP timeout을 fence 남은 시간 이하로 제한(최소 0.2 s) → Gate 출력 0: **4.50 s → 3.18 / 3.06 s**(2회 유효, 1회 Nav2 기동 실패 INVALID). fence 길이 3 s 자체는 운영 정책으로 남김(Wi-Fi 지터 실측 후 결정).
- 단위: `test_web_mapping_session.py` 26/26(Lease 상한 시험 추가).

## 8. 실차 연결 점검 결과 (2026-11-20 LIMO 전원 ON, 읽기 전용)

| # | 항목 | 판정 | 근거 |
|---|---|---|---|
| 1 | `pcu` Tailscale | PASS | online, ping 90 ms |
| 1 | SSH | PASS (조건부) | 이름 `pcu`로는 호스트 키 검증 실패(known_hosts에 IP로만 등록). 이미 신뢰된 Tailscale IP로 접속 성공. 호스트 키 검증은 우회하지 않음 |
| 2 | 전원·배터리 | SKIP | sysfs에 전원 장치 없음. 배터리는 `/limo_status`(베이스 드라이버 구동 필요)로만 읽음 → 스택 미기동이라 미확인 |
| 2 | USB | PASS | `/dev/ttylimo`→ttyUSB1, `/dev/ydlidar`→ttyUSB0, CP210x 2개, Orbbec Depth·카메라 인식. 부팅 후 USB 오류 로그 0건(부팅 5분) |
| 2 | 기존 실행 프로세스 | PASS | LIMO·ROS·Agent·bag 관련 프로세스 0개 → 중복 실행 위험 없음 |
| 3 | 로봇→서버 도달 | PASS | 로봇에서 `http://<노트북>:18190/api/v1/health` 200, `field.env`의 주소 = 현재 노트북 주소 |
| 3 | Robot Agent | **FAIL (미설치)** | 로봇에 `limo_robot_agent` 패키지·`~/limo_field_agent_ws` 없음(상태 폴더만 남음). `robot_ws`(11-19)는 Agent 없이 구성됨. 11-19에는 읽기 전용 센서 브리지(`local-only/limo_robot_ws/fleet18190_sensor_bridge.py`)로 18190에 표시했음 |
| 3 | 인증·버전 호환 | SKIP | Agent 없음. agent-pcu 토큰은 로봇 `~/.config/limo_field_20261113/`(0600)에 있고 서버 auth.json에 등록됨 |
| 4 | LiDAR/Depth/Odom/TF/Nav2 | SKIP | ROS 스택 미기동(프로세스 0). 기동은 실행 launch이므로 별도 승인 대기 |
| 5 | Safety Gate·모터 잠금 | SKIP | 위와 동일. 로봇 launch의 `allow_motor_drive` 기본값은 false |
| 6 | 웹 Online·센서 갱신 | FAIL | 서버가 `pcu`를 offline·미조정으로 표시(Agent 없음), 센서 필드 없음 |
| 7 | Lease 획득·해제 | PASS | operator 토큰, 30 s Lease 획득 200 → 해제 200 (B1 사본 서버). 갱신·명령 ACK는 Agent 없어 SKIP |
| 8 | Mapping START 거부 | PASS | 409 `worker_unavailable`. Goal 제출은 로봇 미연결이라 SKIP(서버 `safety_not_ready` 예상) |

추가 발견:
- **로봇의 실차 launch는 작업트리와 다름**: `limo_integrated_auto_mapping_real.launch.py` sha256 `b87e16d3…`(로봇) vs `1671d282…`(노트북 작업트리, 다른 에이전트 미커밋 수정 포함). 로봇 launch에는 Gate가 제공하는 `max_abs_linear_x` 등 최신 변경이 없을 수 있음. 동일 여부는 diff로 확인 필요.
- **ROS 통신 방식 불일치**: 로봇 `env.sh`는 FastDDS Discovery Server(노트북 `:11811`, `ROS_LOCALHOST_ONLY=0`)를 쓰지만 노트북의 11811은 닫혀 있음. 11-13 현장 절차는 도메인 77·localhost 전용. 어느 쪽으로 기동할지 정해야 함(Agent는 로봇 안에서 localhost 그래프로 Nav2·Gate를 읽는 구성이 안전).
- 프로브 보정: 오프라인 로봇의 낡은 서버 값을 PASS로 세지 않도록 pose 검사 수정.
- 증거: `local_data/field_run_20261120_b1/probe_pcu_before_agent.json`.

## 9. Agent 설치와 연결 (2026-11-20 밤, 사용자 승인: 모터 잠금 스택 + 읽기 전용 Agent)

실행 결과 (모터 구동 명령 없음, 베이스 드라이버·ROS 스택은 **미기동**):

| 단계 | 판정 | 근거 |
|---|---|---|
| 로봇 launch vs 작업트리 비교 | PASS | 로봇 Gate = `HEAD`와 동일. 로봇 launch = `HEAD` + 11-12 scan normalizer. 모터 경로는 `HEAD`와 동일(Nav2→`/cmd_vel_raw`→Gate→`/cmd_vel_locked`). 작업트리(다른 에이전트 수정 포함)와는 172줄 차이이며 **덮어쓰지 않음** |
| `allow_motor_drive:=false` 차단 검증 | PASS(코드·launch 평가) / SKIP(실행 중 그래프) | 로봇 launch를 평가: 기본·`false`·`True`·`1`·빈 값 → Gate 출력 `/cmd_vel_locked`, 정확히 `true`만 `/cmd_vel`. 로봇 드라이버는 `/cmd_vel`만 구독하고 `robot_ws/src`에 다른 `/cmd_vel` 발행자 없음. 실행 중 그래프(`/cmd_vel` 발행자 0)는 스택 기동 후 확인 |
| 백업 | PASS | 로봇 `~/Capstone1_limo/predeploy_agent_20261120_085514.tar.gz`(+`.sha256`,`.files.sha256`, 0600) = robot_ws src·install·env.sh, `~/.config/limo_field_20261113`, Agent 상태 폴더. 노트북 `local_data/robot_backups_20261120/`(0700)에 복사, 해시 일치. **토큰 포함 — 공개 금지** |
| Agent 설치 | PASS | `robot_ws/src/limo_robot_agent` 신규(노트북 소스와 파일 해시 23/23 일치), `colcon build --packages-select limo_robot_agent`. 기존 robot_ws 564개 파일 해시 변화 0, 삭제 0, 신규만 66개 |
| ROS 환경 | PASS | 신규 `robot_ws/env_local77.sh`: 도메인 77, `ROS_LOCALHOST_ONLY=1`, Discovery Server 미사용. 기존 `env.sh`·11811 미변경·미기동. 빈 그래프 확인 |
| Agent 연결·인증·버전 | PASS | `mode=real simulated=False missions_enabled=False`, 서버에 `connected and reconciled`, 프로토콜 2·계약 1 |
| 웹 Online | PASS | `online=true reconciled=true` |
| Lease | PASS | 획득(epoch 5)·갱신(epoch 유지, 남은 20 s)·해제 |
| Mapping START 거부 | PASS | 409 `worker_unavailable` |
| 지도 클릭 Goal 거부 | PASS | 409 `safety_not_ready`, 미션·명령 생성 없음 |
| 명령 ACK | SKIP | 안전하게 보낼 수 있는 명령이 없음(미션은 서버가 생성 단계에서 거부, 취소할 미션 없음) |
| 센서·TF·Nav2·Gate | 미확인 | 스택 미기동 → 웹에 NO_DATA/UNAVAILABLE 표시(정상) |
| 카메라·YOLO | **미준비** | 노트북 AI host 미실행(18191/18192 비어 있음), 로봇에 영상 송신 키트(`~/limo_video_kit`) 없음, 현재 서버는 `--video-origin` 없이 기동. 서버 런처에 `VIDEO_ORIGIN` 옵션만 추가(재기동은 하지 않음) |

로봇에서 Agent는 실행 중이다(상태 폴더 `~/.local/state/limo_robot_agent_field_20261120/`, 로그 `agent.out`). 중지: 로봇에서 `pkill -INT -f "robot_agent --ros-args"`(이 Agent만, 다른 프로세스 없음).

복구(롤백): Agent 중지 → `robot_ws/install/limo_robot_agent`, `build/limo_robot_agent`, `src/limo_robot_agent`, `env_local77.sh`만 제거하거나 보존 이름으로 이동 → 필요 시 백업 tar를 별도 위치에 풀어 해시 비교 후 복원. 기존 파일은 수정하지 않았으므로 되돌릴 대상은 신규 파일뿐이다.

### 다음 단계(대기): 모터 잠금 ROS 스택 기동 — 베이스 드라이버가 `/dev/ttylimo`를 연다
승인 조건 (사용자 확인 필요): 로봇 주변 확보, 물리 정지 수단(전원 스위치/E-Stop 접근) 확인.

## 10. 모터 잠금 스택 기동과 실차 점검 결과 (2026-11-20 09:00~, 사용자 확인 후)

기동: 로봇에서 `. ~/Capstone1_limo/robot_ws/env_local77.sh` 후 `ros2 launch limo_capstone limo_integrated_auto_mapping_real.launch.py allow_motor_drive:=false use_depth_camera:=<true|false> scan_normalizer:=true`(로봇 자체 launch, 기존 launch 중복 0 확인). Agent는 별도 프로세스(읽기 전용). 로그·프로브 결과·USB 커널 이벤트: `local_data/raw_results/limo_runs/20261120_locked_agent_01/`(MANIFEST 있음).

| 항목 | 판정 | 근거 |
|---|---|---|
| `/cmd_vel` 발행자 0 (기동 6/12/18 s, 이후 매 측정) | PASS | 구독자 1(베이스), Gate는 `/cmd_vel_locked` 발행, `[SAFETY LOCK ACTIVE]` 로그 |
| 배터리 | 전압만 | 12.3 V(Depth 끔) → 12.0 V(Depth 켬). 퍼센트·임계 기준 없음(드라이버는 전압만 제공) |
| USB | PASS (조건부) | 첫 기동 중 09:00:28–30에 베이스 UART 허브(3-7)가 재열거 반복(`clear tt 1 error -71`, `ttyUSB1`→`ttyUSB2`) → 베이스 데이터 끊김·Gate `FAILSAFE_SENSOR_TIMEOUT`. 스택 재기동 2회(Depth 끔 2분, Depth 켬 5분+)는 USB 이벤트 0건. **원인 미확정, 재발 가능** |
| LiDAR | PASS | 10.03 Hz |
| Wheel odom / IMU | PASS (재기동 후) | 49.9 Hz / 99.8 Hz. 첫 기동은 위 USB 사건으로 FAIL |
| Depth / RGB | PASS | 포인트 11.2 Hz, RGB 13.6 Hz, 웹 Depth `OK DEPTH_READY` |
| TF (map→base_footprint, odom→base_footprint) | PASS | 웹 pose `OK`, 나이 0.12 s |
| Nav2 | PASS | controller·planner·bt_navigator active, 웹 `READY` |
| Safety Gate | PASS | NORMAL, 웹 Gate 나이 0.04–0.08 s (Depth 끔 구간 24/24 NORMAL) |
| 웹 Online·센서·지도 | PASS | LiDAR OK, 지도 수신(157×173), Gate·모터 경로 LOCKED(`stall_detector_node`) |
| Localization | 미확정(차단 유지) | `UNKNOWN`/`LIVE_SLAM`, C103 지도와 정합 절차 없음 |
| Lease | PASS | 획득·갱신·해제 |
| Mapping START | PASS(거부) | 409 `worker_unavailable` |
| 지도 클릭 Goal | PASS(거부) | 409 `safety_not_ready`, 미션 생성 없음 |
| 명령 ACK | SKIP | 안전하게 보낼 명령 없음 |
| 카메라 웹 표시·YOLO | **미준비** | 로봇에 영상 송신 키트 없음, 노트북 AI host 미실행, 서버에 `--video-origin` 없음. RGB 토픽 자체는 13.6 Hz 수신 |

현재 상태: 로봇에서 스택(Depth 켬)과 Agent가 모터 잠금으로 실행 중. 종료(로봇): `kill -INT -- -$(ps -o pgid= -p $(pgrep -f "ros2 launch limo_capstone" | head -1) | tr -d ' ')` 후 `pkill -INT -f "robot_agent --ros-args"`. 종료 후 `ps`로 잔류 확인.

안전 플래그: `allow_motor_drive:=true`는 사용하지 않았다. 베이스 UART 허브 재열거는 11-18과 같은 양상이다. 전원/USB 허브/케이블 물리 점검이 필요하며, 이것이 해결되기 전 실주행 No-Go 유지.

## 11. 영상 경로(RGB → AI host YOLO → 대시보드) (2026-11-20 09:15, 사용자 승인)

| 단계 | 판정 | 근거 |
|---|---|---|
| 노트북 AI host(CUDA YOLO11n) 기동 | PASS | 링크 TS주소:18191, HTTP TS주소:18192(와일드카드·루프백 바인드 없음). 기존 field `run/` 로그를 덮어쓰지 않도록 `RUN_DIR`을 `local_data/field_run_20261120_b1/run`로 지정 |
| 로봇에 영상 키트 배포 | PASS | 신규 폴더 `~/limo_video_kit`만 생성(기존 파일 변경 0), `sender.token` 0600, 키트 `deploy_to_limo.sh` 사용(print-only 목록 확인 후) |
| 송신기 기동 | PASS | 로봇에서 `IMAGE_TOPIC=/camera/color/image_raw ./21_limo_start_sender.sh`. 읽기 전용 구독자. 시작 전 `/cmd_vel` 발행자 0 확인(ARMED면 스스로 거부) |
| RGB → YOLO 왕복 | PASS | 8.6 fps 수신, 추론 평균 10.5 ms(p95 13.1), 서버 총 15.9 ms, 오류 0, 드롭 0, 재연결 0 |
| 18190 `--video-origin` | PASS | 서버 재기동(내 서버만, 현장 data 미변경), CSP `connect-src`에 AI host 허용 확인. Agent 자동 재접속·재조정 PASS |
| 웹 프로브 | PASS | 17/0, START·Goal 거부, Lease 유지 |
| 브라우저 영상 패널 표시 | **미확인** | 대시보드 로그인에 토큰 입력이 필요해 Claude가 하지 않음. 주소: `http://127.0.0.1:18190/?video=http://<노트북 TS주소>:18192` (`tailscale ip -4`) |

스택 점검(09:17): `/cmd_vel` 발행자 0, odom 49.9 Hz, 배터리 12.0 V, 09:03 이후 USB 이벤트 0건. 송신기 종료(로봇): `pkill -INT -f sender_node.py`(또는 키트 `50_stop_all.sh`). AI host 종료(노트북): `kill -INT $(cat local_data/field_run_20261120_b1/run/ai_host.pid)`.
증거: `local_data/raw_results/limo_runs/20261120_locked_agent_01/`(`sender.log`, `ai_host.log`, 프로브 JSON).

## 12. 현재 위치 H107 (사용자 보고, 2026-11-20 09:2x)

- 사용자 확인: 브라우저에서 영상과 지도가 표시됨. 로봇의 현재 위치는 **H107**이며 등록된 C103(C관 1F 라운지 일부)과 다른 장소다.
- 의미: 오늘 스택은 저장 지도를 적재하지 않고 새 SLAM으로 기동했으므로(웹 `localization_status=LIVE_SLAM`, 지도 157×173) 화면의 라이브 지도는 H107 기준이다. C103 지도는 이 장소에서 **Localization·Goal·CONTINUE 기준으로 쓸 수 없다**(쓰면 잘못된 지도 위의 위치가 된다).
- Registry에는 H107이 없다. 시설·건물·층·영역은 운영자가 선언하는 메타데이터이며 로봇이 자동으로 알 수 없다. H107의 건물·층·영역 이름을 받기 전에는 등록하지 않았다.

## 13. H107 지도 저장·등록 (2026-11-20 09:2x, 사용자 승인: "저장 후 등록")

- 위치 선언(운영자): 배재대학교 / H관 / 1F / H107 강의실, `PARTIAL_FLOOR`.
- 저장(로봇, 새 폴더 `~/maps_h107_20261120`, 0700): `map_saver_cli`(map.pgm·map.yaml) + `slam_toolbox/serialize_map`(posegraph.data·.posegraph, result 0). 로봇 정지 상태, 속도 명령 없음, `/cmd_vel` 발행자 0. 로봇 원본 보존. 노트북 `local_data/raw_results/limo_runs/20261120_h107_map/map/`로 복사, 4개 파일 sha256 로봇=노트북 일치.
- 지도 품질: 157×173, 확인 셀 866개(자유 851, 점유 15) ≈ **2.17 m²**. 한 자리에서 본 범위만 담은 지도이며 강의실 전체 지도가 아니다. 정답 지도 없음.
- 등록(B1 사본 Registry만, 현장 Registry 미변경): 신규 스크립트 `fleet_control/scripts/register_saved_map.py`(C103 스크립트는 그대로). 지도 버전 `h107_classroom_20261120_stationary#91e118e9e5ed`, UNVERIFIED, 층 활성 지도 아님, 계보 NEW_MAP(부모 없음), 번들 무결성 OK. C103 항목 변화 없음.
- **YAML 보정(기록)**: 로봇 `map_saver_cli`의 `free_thresh: 0.25`는 ROS map_server에서 unknown 픽셀(p=0.196)을 free로 해석하게 만든다. 등록기의 안전 검사가 이를 거부했고, 검사를 완화하지 않았다. 대신 `--fix-free-thresh 0.196`으로 **파생 YAML**을 등록했다(C103과 같은 값). 원본 `map.yaml`(sha256 `ae2c2499…`)은 그대로이며 원본·파생 해시와 사유를 증거에 기록했다. 앞으로 로봇에서 저장할 때는 `--free 0.196`을 주는 편이 낫다.
- 사전 백업: `local_data/field_run_20261120_b1/data_before_h107`(+`.sha256`). 되돌리기: 서버 종료 → 현재 `data`를 보존 이름으로 이동 → 이 백업을 `data`로 복사.
- 점검: 웹 프로브 17/0, 서버는 영상 origin 포함해 재기동, Agent 자동 재접속.
- 사용 제한: H107 지도로도 Localization·Goal은 쓰지 않는다(미검증, 한 자리 지도, 실차 차단 유지).

## 14. 표시 확인·보존 확인·종료 준비 (2026-11-20 11:00~)

- **로봇 재부팅 발견**: 로봇은 09:52에 종료되고 11:02에 다시 켜졌다(`last -x`). ROS 스택·Agent·영상 송신기는 모두 종료 상태이며 웹에는 `pcu` OFFLINE(마지막 보고 약 72분 전)으로 표시된다. 재기동은 사용자의 주변 확보·물리 정지 수단 재확인 후에만 한다. 노트북의 AI host와 18190 서버는 계속 실행 중(`SENDER_DISCONNECTED`).
- **지도 표시 PASS(API·미리보기 PNG)**: H107(157×173, 확인 셀 866개)과 C103(344×472)이 레지스트리 미리보기로 렌더링되고, 건물·층·영역·버전·UNVERIFIED·계보·센서 출처가 메타에 있다. 브라우저 화면은 사용자가 확인함("보임").
- **오래된 값이 실시간으로 보이는지 — 발견(부분 결함)**: 서버는 오프라인 로봇의 마지막 상태를 **보고 시점의 `age_s` 그대로** 돌려준다(pose `OK 0.15`, LiDAR `OK 0.05`, Gate `NORMAL 0.024`, 72분 뒤에도). UI는 헤더에 "OFFLINE (아래는 마지막 수신값)", "마지막 보고 72분"을 빨강으로 보이고 Gate 행에 `[OFFLINE 과거 표본]`을 붙이지만, **LiDAR·Depth·위치·배터리·Nav2 행에는 접두 표시가 없고 `OK … age 0.05s`가 정상 톤으로 남는다.** 서버 쪽 안전 판단(Safety Context, 미션 생성)은 `state_ts`로 걸러져 영향 없음. 제안: 오프라인일 때 모든 행에 과거 표본 표시·톤을 회색으로, 표시용 age에 `state_age_s`를 더함(`static/app.js`). 구현은 하지 않았다.
- **프로브 수정**: 같은 문제로 `real_locked_probe.py`가 오프라인 로봇의 낡은 값을 PASS로 셀 수 있어 `online`과 `state_age_s < 3 s`일 때만 라이브 판정하도록 고쳤다(오프라인이면 모든 라이브 검사 FAIL·"state is history" 표시).
- **보존 확인 PASS**: H107 원본 4개 파일이 로봇 해시와 일치, 파생 YAML 증거(원본 `ae2c2499…` → 파생 `52910ce7…`, 0.25→0.196, 사유)가 레지스트리에 있음, C103 증거 해시 4개 불변, 레지스트리 `integrity_check` ok, 현장 data·B1 백업 불변, `/cmd_vel` 발행자 0은 로봇이 꺼져 있어 해당 없음.
- **종료 준비**: `fleet_control/scripts/stop_field_session.sh`(기본 dry-run). `--execute`일 때만 로봇의 `sender_node`·`robot_agent`·launch 그룹(SIGINT)과 노트북의 AI host·18190을 종료하고 잔류를 확인하며 로그를 노트북에 보존한다. 지금은 dry-run만 실행했다.
- **웹 건물·층·영역 생성 + 저장 후 등록 재사용 조사**: [WEB_REGISTRY_AUTHORING_INVESTIGATION_20261120.md](WEB_REGISTRY_AUTHORING_INVESTIGATION_20261120.md). 시설·건물·층 생성 UI와 지도 업로드 UI는 이미 있고, 영역 생성 라우트·UI, 업로드 시 영역·계보 연결, YAML 임계값 검사, 실차 저장 전용 작업이 없다. 구현하지 않았다.

## 15. 12:40 로봇 LiDAR 스캔 끊김 (읽기 전용 조사, 조치 없음)

11:09 기동한 모터 잠금 스택이 11:45:38 이후 LiDAR 스캔을 내지 못한다(YDLidar `Timeout count: 3`, 11:25부터 오류 91줄). 웹 LiDAR·pose STALE, Gate `FAILSAFE_SENSOR_TIMEOUT`(출력 차단, 모터 경로 LOCKED 유지). `/odom`·`/imu` 수신 미확인, ROS 노드 4개만 관측, FastDDS 공유메모리 오류와 `/dev/shm` fastrtps 파일 120개. USB 커널 이벤트 없음. 오전의 USB 허브 재열거와 별개이거나 같은 계열인지 미확정. 스택 재기동은 승인 대기.

## 16. 12:45 모터 잠금 스택 재기동 (사용자 승인)

- 종료: launch SIGINT 후 `lidar_scan_matcher_odom_node`(pid 4690)가 종료되지 않아 **SIGTERM 무반응 → SIGKILL**(그 노드만). `limo_base`는 종료 시 `terminate called without an active exception`(exit -6)으로 비정상 종료(종료 시점 크래시, 오전에도 관찰된 유형). 이후 launch 프로세스도 SIGTERM으로 정리. Agent·송신기는 계속 실행.
- 기동: 같은 구성(`allow_motor_drive:=false`, Depth 켬, scan normalizer). `/cmd_vel` 발행자 0(6/12/18 s), `[SAFETY LOCK ACTIVE]` 로그.
- 상태: ROS 노드 34개, Gate NORMAL, LiDAR 10.0 Hz, odom 49.9 Hz, IMU 100 Hz, Depth 21 Hz, RGB 13.9 Hz, Nav2 active, USB 이벤트 0, LiDAR 오류 0. 웹 프로브 17/0(START·Goal 거부, 모터 경로 LOCKED). AI host 수신 7 fps.
- **배터리**: 12.3 V(09:06) → 12.0 V(09:17) → **11.0–11.1 V**(12:50). 드라이버는 전압만 주며 퍼센트·임계 기준이 없다. 11:25–11:45의 LiDAR 타임아웃과의 관련은 **미확정**(전압 강하 가능성은 가설일 뿐 검증 안 됨). 충전 또는 교체를 권장한다.
- 원인 미확정 항목: LiDAR 끊김(11:45), 종료 시 `lidar_scan_matcher_odom_node` 무응답과 `limo_base` 크래시.
- 증거: `local_data/raw_results/limo_runs/20261120_locked_agent_02/`.

## 17. 정지 상태 측정 — Depth 카메라 높이·기울기 (충전 중, 읽기 전용, 2026-11-20 13:0x)

방법: 로봇 정지·모터 잠금, 구독만 하는 스크립트(`local_data/raw_results/limo_runs/20261120_stationary_measure_01/stationary_measure.py`, 결과 `measure_1.json`). Depth 점군 8프레임에서 바닥 평면을 RANSAC+SVD로 맞췄다. 속도 명령 없음, `/cmd_vel` 발행자 0 유지.

| 측정 | 값 |
|---|---|
| 바닥 평면 적합 | 후보 약 48,000점 중 36,700–37,600점이 inlier, 잔차 RMS 2.8 mm, 8프레임 모두 일치, 사용 거리 0.44–1.78 m |
| **카메라 높이(바닥→카메라 광학 원점)** | **0.1988 m** (0.1981–0.2001) |
| **카메라 아래 기울기(pitch)** | **5.4°** (5.33–5.53, 광학 프레임 기준). 현재 TF 기준 바닥 법선 기울기는 6.1° |
| 카메라 roll | 0.4° (현재 TF 기준 0.9°) |
| 현재 TF의 카메라 | base_link→camera_link (0.084, 0, 0.030) m, **회전 없음(공칭)**. LiDAR는 base_link (0.103, 0, −0.034) |

전제(확인 필요): 바닥이 평평하고 수평이며, 바퀴가 바닥에 닿아 있고(충전대·받침 위가 아님), 측정 중 로봇이 정지. 충전 도크나 받침 위라면 기울기·높이는 마운트가 아니라 설치 상태를 반영한다.

**의미(전제가 맞다면)**
- 현재 launch는 `camera_pitch=0`이고 Depth 필터는 `base_footprint` 프레임의 `0.025–0.35 m` 대역을 쓴다. 실제 기울기 약 6°가 TF에 없어서, 정면 거리 d의 점은 높이가 약 0.107·d m 틀어진다(1 m 10.7 cm, 2 m 21 cm, 3 m 32 cm). 바닥이 현재 TF에서는 앞쪽으로 올라가는 경사로 보여 약 2.3 m 부근부터 바닥이 낮은 장애물 대역에 들어올 수 있다.
- 또 `base_footprint_z`가 0.0이라 `base_footprint`가 바닥이 아니라 `base_link`와 같다. 그래서 대역이 바닥 기준이 아니다(앞서 문서의 "약 0.17–0.50 m" 추정과 같은 방향).
- 보정값(계산): `camera_pitch:=0.1038` rad(5.95°, 공칭 TF 위에 더하는 값), `camera_roll:=0.0140` rad, 그리고 `base_footprint_z = 0.1988 − camera_z`(공칭 camera_z 0.0303이면 **+0.1685 m**, base_link가 바닥 위 0.1685 m). 이 보정을 적용하면 바닥 법선 기울기가 0.00°가 된다(계산값). URDF 추정(바퀴 중심 −0.100, 반지름 0.045 → 0.145 m)과 2.35 cm 차이가 나는데, 공칭 camera_z(0.03)가 실제와 다를 때 생기는 차이이며 이 값은 줄자로 확인해야 한다.
- 이 값들은 launch 인자(`camera_pitch`, `camera_roll`, `base_footprint_z`)로 줄 수 있고 launch 파일을 고치지 않아도 된다. **아직 적용·재측정하지 않았다**(스택 재기동 필요).

**무효 처리된 측정**: Depth와 LiDAR 정면 거리 비교(오래된 "Depth 스케일 0.91–0.93배" 확인)는 장면이 달라 **무효**다. Depth는 0.12–0.6 m 높이 대역에서 정면 약 1.0 m의 물체를 봤고 LiDAR 정면 중앙값은 3.08 m였다. 평평한 판을 둘 다 보이는 위치(LiDAR 높이와 Depth 대역을 모두 지나는 판, 정면 약 1 m)에 세워야 의미가 있다.

**배터리**: 충전 중 8초 샘플 27개가 11.0–11.3 V로 흔들렸고(±0.15 V 잡음) 상승 추세는 이 구간에서 보이지 않았다. 충전 여부는 드라이버 메시지에 충전 플래그가 없어 판단하지 못한다.

**줄자로 확인할 것**: (1) 바닥에서 DaBai 카메라 렌즈 중심까지 높이(예상 약 0.199 m), (2) 바닥에서 base_link(섀시 기준점)까지 또는 카메라 렌즈~base_link 수직 거리, (3) 로봇이 평평한 바닥에 바퀴로 서 있는지.

## 18. 카메라 TF 보정과 재측정 (정지·충전 중, 사용자 승인, 13:0x–13:10)

사용자 확인: 바퀴는 바닥에 닿아 있음. 휴대폰 자로 잰 바닥→카메라 렌즈 높이 17–22 cm(점군 측정 19.9 cm와 모순 없음, 자 오차 큼).

**측정이 흔들린 점(정직하게 기록)**: 같은 스크립트를 시간 순서로 10회 돌렸을 때 카메라 높이는 0.197–0.200 m로 일정하지만 **기울기는 변했다.** 첫 측정(13:0x)은 광학 프레임 기준 5.4°, 사용자가 렌즈 높이를 잰 직후부터는 2.2–2.4°로 낮아져 7분 이상 일정했다(측정 3–7). 측정 2(사용자 작업 중)는 plane 적합이 나빠(inlier 36 %) 이상치로 제외했다. LiDAR 정면 거리는 3.07–3.08 m로 같아 로봇은 움직이지 않았다. 따라서 **카메라가 만져지면서 실제 기울기가 바뀐 것으로 보이며(미확정), 마운트가 느슨하거나 조절 가능하다는 뜻일 수 있다.** 기울기는 한 번 재서 코드나 설정에 박아 둘 값이 아니다.

**적용한 보정(이번 스택 기동 인자만, 파일 변경 없음)**: `camera_pitch:=0.0502 camera_roll:=0.0140 base_footprint_z:=0.1683` (평탄화 계산 중앙값 pitch 2.87°, roll 0.8°, 카메라 높이 중앙값 0.1986 m − 공칭 camera_z 0.0303). 모터 잠금 유지, `/cmd_vel` 발행자 0, Gate NORMAL, LiDAR 10 Hz, odom 50 Hz, IMU 100 Hz, 웹 프로브 17/0.

| `base_footprint` 프레임에서 본 바닥 (정지, 앞 약 0.4–1.8 m) | 보정 전(측정 3–7 중앙값) | 보정 후(측정 8–10) |
|---|---|---|
| 로봇 위치의 바닥 높이 z | **−0.172 m** | **−0.001 m** |
| 바닥 경사 앞뒤 / 좌우 | +3.03° / −0.86° | **+0.29° / −0.05°** |
| 바닥 평면 잔차 | 3.4 mm | 3.3 mm |
| 카메라 높이 | 0.197–0.199 | 0.1995–0.2004 |

의미: 보정 전에는 Depth 필터 대역 `0.025–0.35 m`(기준 프레임 `base_footprint`)가 바닥 위 약 **0.20–0.52 m**를 보고 있어서 **높이 약 20 cm 미만의 낮은 장애물은 아예 보이지 않았다**(오래된 "낮은 장애물 FAIL"의 유력한 원인). 보정 후에는 대역이 바닥 위 2.5–35 cm이다. `depth_status`는 `DEPTH_READY`(유효 23,258점 중 장애물 13,515점).
주의: "대역 안 점의 비율"(47 % → 59 %)은 누출 지표가 **아니다**. 보정 후 대역에는 벽·가구의 아랫부분이 실제로 들어온다. 이 값으로 개선을 주장하지 않는다.

**아직 확인하지 못한 것**: 낮은 물체(예: 높이 10 cm 상자)를 정면에 둘 때 실제로 Depth 필터와 costmap에 표시되는지(실물 시험 필요), 카메라가 다시 움직이면 보정이 틀어진다는 점, 바닥이 평평하다는 가정, 먼 거리(2 m 이상)에서의 바닥 잡음. 보정 인자는 **이 스택 인스턴스에만** 적용돼 있고 다음 기동 때 기본값(0)으로 돌아간다.

**제안(미구현)**: 정지 상태에서 시작할 때마다 바닥 평면을 맞춰 카메라 pitch·roll·높이를 구해 TF에 반영하는 보정 단계(마운트가 바뀌어도 맞음). 그 전까지는 이 인자를 시작 명령에 넣어야 하며, 카메라를 만진 뒤에는 `stationary_measure.py`로 다시 확인한다.

증거: `local_data/raw_results/limo_runs/20261120_stationary_measure_01/`(측정 1–10, 스크립트, 프로브 JSON, MANIFEST).

## 19. 정면 물체 시험 1차 (정지, 보정 후 TF, 읽기 전용, 13:2x)

스크립트 `low_obstacle_check.py`, 결과 `low_obstacle_1.json`, `low_obstacle_2.json`(같은 폴더).

놓인 물체는 **낮은 물체가 아니었다**: 정면 약 0.49 m(`base_footprint` 기준), 폭 약 0.5 m(y −0.17~+0.34), 윗면이 최소 0.33 m(카메라 시야가 이 거리에서 위를 자른다). LiDAR 정면 거리가 3.08 m에서 0.47 m로 바뀌었으므로 LiDAR(바닥 위 약 0.134 m)도 본다.

| 경로 | 결과 |
|---|---|
| 원본 Depth 점군(`base_footprint`, 대역 0.025–0.35 m) | x 0.45–0.5 m에 약 44,000 + 16,000점, z 0.026–0.338 |
| Depth 필터 출력 (`camera_color_optical_frame` 좌표, 변환 후) | x 0.45–0.5 m에 약 11,000 + 3,900점, 그 뒤 0.65–0.9 m에 가구로 보이는 군집 |
| 로컬 costmap(odom, lethal=100만) | x 0.45–0.5 m 정면에 lethal 셀 7–9개/5 cm, 뒤쪽 0.7–1.0 m에도 있음. 총 lethal 177 |

판정: 보정 후 TF에서 **큰 장애물은 Depth 필터와 costmap에 모두 나타난다.** 그러나 이 물체는 LiDAR도 보기 때문에 이번 시험은 **낮은 장애물 검출(원래 문제)을 입증하지 못한다.** 그리고 보정 전에는 같은 물체가 어떻게 보였는지 측정하지 않았으므로 보정 효과의 비교도 아니다. 낮은 물체 시험은 높이 약 8–10 cm(LiDAR 높이 아래, 대역 하한 2.5 cm 위), 정면 0.6–0.9 m(카메라는 약 0.44 m 안쪽 바닥을 보지 못함)에 놓고 LiDAR 정면 거리가 변하지 않는지, 필터 출력과 costmap에만 나타나는지로 판정해야 한다.

## 20. 웹 지도 저장·등록 (2026-11-20)
실차 모터 잠금 저장·등록 결과와 실차에서 발견한 DDS 수신 문제는 `WEB_MAP_SAVE_20261120.md` 참조.
