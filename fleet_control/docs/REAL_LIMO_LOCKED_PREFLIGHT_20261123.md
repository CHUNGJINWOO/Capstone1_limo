# 실제 LIMO 모터 잠금 상태 읽기 전용 Preflight 점검표 (2026-11-24 준비본)

**이 문서는 절차이며 실행하지 않았다.** LIMO 접속·설치·모터 조작은 사용자의 별도 승인 전까지 하지 않는다. 승인 후에도 이 점검표는 **읽기 전용**이다: 서비스 호출, 토픽 발행, 파라미터 변경, 파일 설치를 하지 않는다. 하나라도 FAIL이면 중단하고 사용자에게 보고한다. 모터 잠금(`allow_motor_drive` 기본 false)은 이 점검 동안 절대 풀지 않는다.

## 0. 준비물과 후보 (승인 전에 노트북에서 확인)
| 후보 | 파일 | SHA-256 (tar) | 파일 수 | 비고 |
|---|---|---|---|---|
| Agent | `limo_robot_agent_candidate_g2phaseC.tar.gz` | `2c46ca16b43a6a28d8af3f4d5c4054fbfa383a2a14ca24cf9cffba994687348f` | 39 | R3 `artifact` 도구, 두 번 생성해 바이트 동일 |
| launch·Gate(`limo_capstone`) | `limo_capstone_candidate_g2phaseC.tar.gz` | `742d4f362db250c1a2525c0348def406a70efe3ea7e7de7b0b6a58c85e729b04` | 91 | R3 allowlist 밖이라 수동 manifest |
| Fleet Server(노트북, 웹 UI 포함 전 단계) | `fleet_server_candidate_g2phaseC.tar.gz` | `eb1468ab0ecc6ee989665e728ab5e213d615188eb25ea71b617048731a1b056f` | 48 | UI 파일 미포함(Antigravity 미커밋 상태이므로 통합 복사본 기준 별도 생성 필요) |
위치: `local_data/raw_results/real_limo_integ_claude_20261122/field_ready_01/p43_candidates_phaseC/`(tar, `TAR.sha256`, `FILES.sha256`, `ARTIFACT.manifest.json`). **모두 미커밋 작업트리 기준**이며 `deployment_approved=false`다. 이전 후보(`148eb74e…`/`b3603242…`, `e6488cee…`)는 폐기한다: 속도 제한·증거·preflight 변경 이전 소스다. 교체·롤백 리허설(임시 workspace, `atomic_swap`)은 두 패키지 모두 통과했다(`p43_phase5_candidates.log`).
> Fleet Server 후보에는 Antigravity의 UI 변경이 들어 있지 않다(그쪽이 미커밋). 서버에 UI까지 배포하려면 두 작업트리를 커밋·병합한 뒤 서버 후보를 다시 만든다.

## 1. 노트북에서 (접속 전, 승인 불필요)
```bash
python3 fleet_control/scripts/field_preflight_20261123.py --server http://127.0.0.1:PORT --token-file TOKEN --robot ROBOT --real
```
GET만 한다. 새로 추가된 F 항목(아래)은 Agent가 보고한 상태만 읽는다. 실행 가능한 시점: Agent가 서버에 붙어 있을 때(승인 후).

| 항목 | 의미 | 기대 | FAIL이면 |
|---|---|---|---|
| A1~A5 | Agent online·신선·reconciled·프로토콜 2/계약 1 | PASS | 중단 |
| B1~B5 | Gate NORMAL, **모터 경로 LOCKED**, Watchdog 승인·신선, Arbiter 출력 없음 | PASS | 중단 |
| C1~C5 | LiDAR·pose·map·battery·depth | PASS/WARN | 센서 확인 |
| E1~E3 | 실차 구성(mode real, simulated false, missions 꺼짐, 웹 Mapping Worker 없음) | PASS | 중단 |
| **F1** | 속도 제한 enforcement point 보고(`WATCHDOG`) | PASS | 속도 제한 불가 → 중단 |
| **F2** | 실차에서 `WATCHDOG`(Arbiter 없음) | PASS | 중단 |
| **F3** | Watchdog 증거를 **서버와 Agent가 검증**, 나이 ≤ 2 s | PASS | 중단 |
| **F4** | `watchdog_boot_id`·`agent_boot_id`·`output_gid` 존재(GID는 그래프 식별자이지 인증이 아님) | PASS | 중단 |
| **F5** | stale 속도 설정 없음 | PASS/WARN | WARN은 새 요청 필요(낮은 값은 유지) |
| **F6** | `/cmd_vel`·`/limo_driver/cmd_vel`에 Gate가 아닌 writer를 본 적 없음(래치) | PASS | **중단**: 직접 발행이 있었을 수 있음, Agent 재시작 전까지 래치 |
| **F7** | LOCKED면 두 토픽 writer 없음 | PASS | 중단 |
| F8 | `/odom` 속도 텔레메트리(추정, 독립 측정 아님) | PASS/WARN | 정보 |

## 2. LIMO에서 읽기 전용으로만 확인 (사용자가 SSH 접속을 승인한 뒤)
(어느 명령도 토픽을 발행하거나 서비스를 호출하지 않는다. `ros2 topic echo`는 `--once` 또는 `timeout`을 쓴다.)
1. **실제 vendor 드라이버가 구독하는 토픽**: `ros2 topic info -v /limo_driver/cmd_vel`과 `ros2 topic info -v /cmd_vel`. 기대: `/limo_driver/cmd_vel`의 Subscription에 vendor `limo_base`(노드 이름 기록), `/cmd_vel`의 Subscription 0개. **이 확인이 이번 변경의 핵심 전제(launch 리매핑이 vendor 드라이버에 실제로 먹힘)이며, 노트북에서는 확인할 수 없었다.** 어긋나면 중단.
2. **Publisher**: 두 토픽 모두 잠금 상태에서 Publisher 0개(`/cmd_vel_locked`만 Gate가 발행).
3. **센서·TF·오도메트리**: `/scan`, `/odom`, `/imu`(Hz와 마지막 stamp), `ros2 run tf2_tools view_frames` 대신 `ros2 topic echo --once /tf_static`. 이전 관측(2026-11-19: `/odom`·`/imu` 0 Hz)이 재현되는지 기록.
4. **Agent·Watchdog 동기화**: `ros2 topic echo --once /fleet/motion_watchdog/status` → `speed_clamp.enabled/synced/publisher_ok/fault`, `state`, `agent_boot_id`가 Agent의 boot와 같은지. `ros2 topic info -v /fleet/speed_limit` → Publisher 1개(`fleet_robot_agent`), Subscription 1개(`fleet_motion_watchdog`).
5. **Gate·Watchdog 오류**: `ros2 topic echo --once /stall_detector/status_json`(state NORMAL, `cmd_input.state BOUND`, `output_topic_faults []`, `sticky null`), Watchdog `last_fault`.
6. **직접 발행자 우회**: `ros2 topic info -v /cmd_vel_raw`(Publisher는 `fleet_motion_watchdog` 1개), `/fleet/nav_cmd_vel`(허용 노드 1개 이하). 위 F6과 같은 결과여야 한다.
7. **모터 잠금**: launch 인자 `allow_motor_drive`가 `false`(또는 미지정)임을 프로세스 명령줄에서 확인. Gate 출력이 `/cmd_vel_locked`인지.
8. **서버 연결·로그·rosbag**: Agent 로그의 `connected and reconciled`, 서버 시각 차이, 디스크 여유, rosbag 기록·복사·해시 검증은 기존 `VALIDATION.md` §2.5 절차를 따르되 **기록 시작은 사용자가 승인한 경우에만**.

## 3. 사람이 직접 확인해야 하는 항목 (코드로 확인할 수 없음, 값은 사용자가 기록)
- **물리 E-Stop**: 동작 방식(전원 차단인지 소프트인지), 바퀴를 띄운 상태의 확인, 복구 절차.
- **STM32 명령 타임아웃**: 입력이 끊겼을 때 구동부가 마지막 속도를 유지하는지(관련 이전 관측: 11-19 STM32 watchdog 원인 미분리). **Gate·Watchdog·Agent가 모두 죽어도 모터가 멈추는지는 이 값에 달려 있다.**
- **제동거리**(저속 0.05~0.10 m/s에서), 배터리 전압·전원·USB/UART 접촉.
- 이 값들이 없으면 운동 활성화 요청 자체를 하지 않는다.

## 4. 중단 조건 (하나라도 해당하면 더 진행하지 않고 보고)
vendor 드라이버가 `/cmd_vel`을 구독 / `/limo_driver/cmd_vel`을 구독하지 않음, 잠금 상태에서 두 토픽에 writer 존재, F1~F4·F6·F7 FAIL, Watchdog 동기화 미성립, Gate NORMAL 아님, 센서 Hz 0, 물리 E-Stop 미확인.

## 5. 검증 수준 표시 (오해 방지)
이 점검표의 F 항목과 Agent·Watchdog·서버 동작은 **격리 ROS·mock Agent에서만 시험**됐다. 실차 하드웨어에서 실행된 적이 없다.

## 6. 추가 항목 (2026-11-24): 모드 선택기와 Watchdog 입력 채널
`field_preflight_20261123.py --real`에 읽기 전용 G 항목이 추가되었다: **G1** Agent가 `velocity_selector: WATCHDOG_SELECTOR`를 선언하고 보고함(Arbiter 없음), **G2** Watchdog의 nav·manual 입력 채널이 모두 exclusive, **G3** 최근 소스 충돌 없음, **G4** 유휴 중 선택기 출력 0(Watchdog의 마지막 출력). LIMO에서 읽기 전용으로 추가 확인: `ros2 topic info -v /fleet/manual_cmd_vel`(게시자는 `manual_mapping_teleop` 하나 이하, 구독은 `fleet_motion_watchdog`), `/fleet/nav_cmd_vel`(게시자 `velocity_smoother`, 구독 Watchdog). **후보 3종은 이번 변경으로 다시 오래된 것이 되었다**: Agent·launch·서버 소스가 바뀌었으므로 커밋 후 재생성하고 SHA를 새로 기록해야 한다.
