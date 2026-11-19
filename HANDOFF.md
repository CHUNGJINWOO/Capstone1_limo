# HANDOFF.md — 에이전트 작업 인수인계 기록

> **최종 갱신**: 2026-11-14  
> 작업을 마친 에이전트는 기존 검증 기록을 삭제하지 말고, **완료 작업 / 미해결 문제 / 다음 작업 / 주의 사항**을 간결하게 갱신하세요.

---

## 1. 최근 완료 작업 (2026-11-12 ~ 2026-11-14)

1. **공통 AI 컨텍스트 문서 8종 구축·정합성 보정 및 폴더·워크트리 감사 (2026-11-14, Gemini)**
   - 기존 파일·폴더·워크트리·프로세스 무수정 상태로 전체 디렉터리 및 외부 실행 환경(`$HOME/rmf_isolated`, `$HOME/rmf_demos_humble`, `$HOME/.local/share/robot-sim-projects/*`) 인벤토리 조사 완료.
   - 루트 공통 문서 8종([START_HERE.md](START_HERE.md), [ARCHITECTURE.md](ARCHITECTURE.md), [CURRENT_STATUS.md](CURRENT_STATUS.md), [CODE_INDEX.md](CODE_INDEX.md), [WORKTREES.md](WORKTREES.md), [DECISIONS.md](DECISIONS.md), [VALIDATION.md](VALIDATION.md), [HANDOFF.md](HANDOFF.md)) 및 [AGENTS.md](AGENTS.md) §18 선택적 열람 규칙 반영 완료.
2. **온보드 안전 체인 결함 수정 후보 및 `fixed2` 검증 (2026-11-13~14, Claude Code)**
   - 경로: 로컬 미추적 `experiments/safety_chain_fix_20261113/` (노트북 전용 실험 후보, 실차 미배포)
   - `obstacle_memory_node.py`(Depth 3D TF 변환, 반복 stamp 무시, `/collision_monitor/memory_status`, 차체 프레임 발행, `/collision_monitor/clear_memory_near`), `stall_detector_node.py`(stamp 정체 감시, 고장 잠금 `latch_safety_faults`, 고장 시점 목표 자동 재개 차단 `block_resume_of_fault_goals`), `lidar_scan_matcher_odom_node.py`(진단 v2), `navigate_to_pose_diff_no_backup.xml` 구현.
   - 검증(**합성 하네스 + Office Gazebo**): `run04`(`fixed`) 및 `run05`(`fixed2`) 28/28 PASS (FAIL 0, 기존 Gate 86/86 PASS). Office Gazebo(`office03_20261114`)에서 Costmap `memory_layer` 및 전용 approach footprint 적용으로 초기 27초 교착을 해소하고 `explore_rep2` 16.22 m, `explore_rep3` 18.48 m, `odom_tf_loss_recover` 20.83 m 주행 확인 (단, `explore_rep1` 1.55 m 조기 종료 및 `low_box_rep1` `sensor_normalizer_node` exit `-2` 실패 기록 보존).
3. **실차 현장 시험 및 Depth·관제·대시보드 연동 (2026-11-13, Claude Code / Codex)**
   - 경로: 로컬 미추적 `experiments/real_limo_field_20261113/`, `experiments/real_depth_integration_20261113/`, `fleet_control/dashboard` (PR #9 병합)
   - 검증(**실차**):
     - LiDAR-only 저속(0.15 m/s) 자동 맵핑(`run_010`, `real_auto_mapping_ready_20261113` 배포본): 약 110초 실제 구동, 지도 1,080→30,430셀 생성 및 `map_001`/`map_002` 저장.
     - 모터 잠금 상태에서 신규 Robot Agent 배포(`~/limo_field_agent_ws`), `409 agent_session_mismatch` 해결 및 지도 API 업로드(`map.png` HTTP 200) 확인.
     - 모터 잠금 상태에서 DaBai RGB → Tailscale → 노트북 CUDA YOLO11n 왕복(10 fps) 및 신규 시각화 대시보드(`dashboard`) 15초 제한 실차 `/map`·Costmap·TF·Depth(SSH `bridge_node.py`) 및 별도 15초 RGB(`readonly_rgb.py`, 5 fps) 표시 확인.
4. **대시보드 스트리밍 연구소(`dashboard_lab`), v2 설계, `rmf-web` 조사 및 통합 시뮬레이션 (2026-11-12~13, Codex / Claude Code)**
   - `fleet_control/{dashboard,dashboard_lab,dashboard_v2_design,rmf_web_research}` (스냅샷 `bc3df5e` → PR #9 병합): `dashboard_lab` 이전 121 PASS 기록(`test_lab.py`는 `aiohttp` 미설치로 미검증), `rmf_web_research` Storybook(18410) 확인.
   - `fleet_control/{agent,rmf_adapter,gazebo}` (스냅샷 `0dc7ef9` → PR #9 병합): 이전 Gazebo × Open-RMF E2E 69/69 PASS(PR #9 격리 검증 미재실행).
   - `fleet_control/integrated_sim` (스냅샷 `29818e8` → PR #9 병합): 이전 단일 통합 실행(`final4`) 12/13 PASS(통합 후 풀스택 재실행 미검증).
5. **대용량 실험 데이터 Option D 비공개 백업·검증 진행 (2026-11-14, [VALIDATION.md](VALIDATION.md) §2.5 기준)**:
   - `run03~06_20261114` (합성 하네스): GPG AES-256 봉인, Google Drive 비공개 업로드 및 원격 복호화·복구 검증(`remote_full_restore_verified: true`) 완료 (단, `run03~05` 과거 중간 편집 소스 4건은 `not_found_in_searched_sources` 한계 유지).
   - `office04_20261115` (25회 실행, 281개 파일): 로컬 전체 복원(393/393, SQLite 25/25, `ros2 bag info` 25/25) 및 Google Drive 8/8 업로드·원격 다운로드 해시(7/7) 검증 완료; 원격 전체 복호화는 사용자 판단으로 생략(`skipped_by_user` / `remote_backup_integrity_verified`), 과거 `tools/run_office2.py`(`e96874a8bec59c35`) 5회분 미확보 유지.
   - `office03_20261114` (12회 실행, 원본 113개): 로컬 전체 복원(225/225, SQLite 12/12, `ros2 bag info` 12/12) 및 Google Drive 8개 파일 업로드·원격 해시 7/7 검증 완료; 원격 전체 복호화는 `skipped_by_user`로 마감(`remote_backup_integrity_verified`, 실행 당시 코드 해시 연결 기록 `meta.json` 미확보).

---

## 2. 미해결 문제 및 차단 요인 (Open Issues & Blockers)

1. **[실차 차단 1] 전원 돌발 차단(4회) 및 베이스 USB 재연결 원인 미규명**:
   - 2026-11-13 시험 중 4회 전원 꺼짐 발생, 충전기 분리·이동 직후 `usb 3-7.4`(`ttyUSB1→ttyUSB2`) 재연결로 `wheel_odom_stale` FAILSAFE(`run_008`) 발생. 원인 규명 전 실차 `--drive` 금지.
2. **[실차 차단 2] DaBai Depth 스케일·지터·소크 정지 문제**:
   - Depth 거리가 LiDAR 대비 약 `0.91~0.93×`로 짧고, 필터 출력 최대 간격(680 ms)이 타임아웃(0.75 s)에 근접하며, 소크 중 IR 투사 정지 현상이 관찰됨.
3. **[시뮬레이션/알고리즘] Obstacle Memory 및 안전 체인 `fixed2`의 미해결 한계 5건 (로컬 미추적 `experiments/safety_chain_fix_20261113/docs/MEMORY_POLICY_V2.md`, `FAILSAFE_CONTRACT.md`)**:
   - **(a) 사각 내 확정 셀(`BLIND_HELD`) 자가 해제 불가**: 로봇 footprint 내부나 바로 앞에 놓인 확정 기억 셀은 LiDAR/Depth 광선이 통과하지 못해 스스로 해제할 수 없으며, 운영자 서비스(`/collision_monitor/clear_memory_near`) 호출이 필요함.
   - **(b) 코너 근접 시 제자리 회전 차단**: 차체 사각형 footprint 대각선 반경(`~0.236 m`) 내에 벽·모서리가 들어오면 `FootprintApproach`가 제자리 회전까지 차단해 좁은 코너에서 정지할 수 있음 (`explore_rep3` 후반부).
   - **(c) 장거리 주행 시 F4 ICP 오도메트리 드리프트 미보정**: `base_shift_correction: false` 적용으로 정지 중 발산은 막았으나, 약 7 m 이상 주행 후 오도메트리 드리프트가 누적될 때 보정하지 못함 (`explore_rep2`).
   - **(d) Costmap 반경 확대 실패 및 미이식**: `robot_radius: 0.22 / inflation_radius: 0.40` 설정은 `explore_rep1`(26.5초·1.55 m 정지)에서 좁은 통로 frontier 고갈을 유발해 기각되었으며, `fixed2` 코드는 아직 `capstone_ws/src/limo_capstone` 및 실차 번들에 이식되지 않음.
   - **(e) 미확정 안전 계약 3건 및 상위 관제 미연동**: `FAILSAFE_ICP_DEGRADED` 래치 잠금 여부, 시뮬/실차 `require_operator_unlatch` 단일화 여부, 실차 Depth 지터 해결 전 `latch_safety_faults` 적용 여부가 미확정이며, Mission Manager·Agent 측 `SAFETY_LATCHED` 연동은 미구현 상태임.
4. **[관제/RMF] Adapter 재시작 시 RMF 고아 작업 및 E2E 타이밍 경합**:
   - RMF Adapter 재시작 시 이전 RMF 작업이 RMF 스케줄러에서 `underway`로 남는 한계 미해결.
   - 통합 워크트리에서 실행한 `run_rmf_checks.py`(47/51) 및 `run_gazebo_checks.py`(66/68)에서 Agent 재시작 구역 타이밍 경합 실패가 발생해 원인 분리 필요.
5. **[저장소 구조] `fleet_control` 통합·기능 워크트리 정리 완료, 남은 후속 정리 항목**:
   - `fleet_control/` 단일 기준(`Capstone1_limo/fleet_control/`, `feat/fleet-control-unified` `ed53d81`) 통합 및 `cleanup/01~04`·기능 워크트리 4개(`bc3df5e`, `0dc7ef9`, `e4dd598`, `29818e8`) 스냅샷 커밋 후 해제 완료.
   - **PR #9(`fleet_control/` 173개) 및 PR #10(`run_recorder.py` 경로 개선) `main`(`a8132c7`) 병합 완료.**
   - 남은 정리 대상: (a) 공통 문서 PR-A 반영, (b) 외부 실험 스크립트 3개 및 형제 스크립트 PR-D 검토, (c) 메인 폴더 `c804241` 미추적 실험/노트 정리.

---

## 3. 다음 권장 작업 (Next Steps)

1. **[실차 비구동 진단]** LIMO 완충 후 충전기를 분리한 상태에서 모터 잠금 모드로 `power_logger.py` 소크 시험을 수행해 전원 차단 및 USB 재연결 재발 여부 확인.
2. **[정책 승인 및 코드 통합]** 사용자와 [DECISIONS.md](DECISIONS.md) `C-01`의 미확정 정책 3건을 확정한 뒤, `experiments/safety_chain_fix_20261113/fixed2`의 검증된 노드를 실차 준비 번들 및 `capstone_ws/src/limo_capstone`으로 이식하는 계획 수립.
3. **[폴더/Git 후속 마무리]** 사용자 승인 하에 (a) `feat/fleet-control-unified`(`ed53d81`) 및 공통 문서 증분(`a021186` + 이번 `WORKTREES.md`/`CODE_INDEX.md` 등) `origin/main` PR 반영, (b) `integrate_worktree` scratch 및 홈 디렉터리 잔여 후보(`~/limo_runs`, `~/slam_toolbox_source`, `webots_limo_pro`, `coppeliasim_limo_pro`) 정리 여부 확정.

---

## 4. 작업 시 주의 사항 (Cautions)

- **실차 구동 금지**: 사용자 현장 입회, 배터리 완충 확인, 물리적 정지 준비, 명시적 승인 없이는 `--drive` 또는 `allow_motor_drive:=true`를 절대 실행하지 마세요.
- **통합 경로 및 실험 스크립트 보호**: `fleet_control/` 및 `experiments/office_limo_sim_20261113` 경로는 실험 스크립트(`lf_env.sh`, `lf_setup.sh`, `offline_registration_test.py`, `office_limo_*.launch.py`)가 참조하므로 임의 이동·삭제를 금지합니다.
- **민감정보 보호**: `Capstone1_limo/fleet_control/{data,secrets}`, `$HOME/.config/limo_field_20261113/`, `$HOME/.local/share/robot-sim-projects/limo_video_ai_preverify/real_limo_kit/secrets/` 내 토큰과 `.env`, DB는 절대 Git에 커밋하거나 Google Drive에 업로드하지 마세요.

## 다음 현장 개발 우선순위

1. 전원·USB 및 물리 정지 수단을 확인하고, 별도 승인 후 기존 실차 Frontier 맵핑을 시험한다.
2. 지도·posegraph·rosbag을 확보하고 재로드한다.
3. 현장 건물·층 정보를 확인해 Map Registry에 등록한다.
4. 저장 지도 기반 Nav2 주행을 별도 승인 후 시험한다.
5. 웹 지도·RGB·Depth 동시 모니터링을 검증한다.
6. 웹 AUTO와 조이스틱 캘리브레이션·버튼 설정을 개발한다.

기존 실차 실행 진입점은 현장 번들의 `start_real_auto_mapping.sh`다. 실제 주행은 현장 조건 확인과 별도 승인이 필요하다.

Frontier 후보 및 Coordinator는 독립 계산·상태 추적 단계다. Agent 연결, 라이브 SLAM Mapping Goal 계약과 웹 AUTO는 미완료다. 기존 실차 자동 맵핑 진입점은 현장 배포본의 `start_real_auto_mapping.sh`이며, 모터 잠금 기동 후 Frontier 명령은 현장 안전 조건과 별도 구동 승인에 따른다.


## 2026-11-18 선택 통합 후 남은 작업 소유 위치

- 실차 현장 자동 맵핑은 기존 배포본의 `~/real_auto_mapping_ready_20261113/start_real_auto_mapping.sh`를 사용한다. 배터리, USB–UART, Gate와 물리 정지 수단을 확인하고 주행 전 별도 승인을 받는다. 지도·posegraph·rosbag 확보 후 건물·층을 확인해 Registry에 등록한다.
- 공개 `fleet_control/`은 기존 Fleet 구현이며, 로컬 staging의 `fleet_control/`에 있는 웹 SLAM START/SAVE/STOP 및 rosbag 제어는 Server·Agent의 Lease/Journal 및 상태 처리 구조가 달라 일괄 교체하지 않았다. 소유권과 안전 차단을 유지하는 설계·통합 및 회귀 검증이 필요하다. 웹 AUTO와 실차 MANUAL은 미완성이다.
- 공개 `capstone_ws/src/limo_capstone/`의 고정 지도 저장은 격리 합성 지도로 검증됐다. 현장 자동 맵핑 배포본 `experiments/real_auto_mapping_ready_20261113/`의 저장 경로와는 별도이며 실차에 이 수정본을 배포하거나 재시험하지 않았다. posegraph와 OccupancyGrid는 서로 다른 시각에 저장될 수 있다.
- 후속 개발은 현장 지도 수집과 병행해 웹 Fleet 지도·RGB·Depth 모니터링을 다룬다. 기존 미추적 실험 및 데이터는 로컬 보관하고 공개하지 않는다.

### 11월 19일 현장 Frontier Runbook

1. 현장 LIMO 로컬 터미널에서 배터리, USB–UART, 휠 odom 연속성, 물리 정지 수단을 확인한다. 기존 SLAM·Nav2·Explorer·bag 프로세스는 소유자를 확인한다. 별도 bringup을 겹쳐 실행하지 않는다. Fleet Agent는 `allow_missions:=false`로 확인하고, 같은 로봇의 RMF runner와 웹 수동 제어는 비활성 상태를 확인한다.
2. `cd ~/real_auto_mapping_ready_20261113`에서 기존 현장 ROS 환경으로 `./start_real_auto_mapping.sh`를 먼저 잠금 상태에서 기동한다. 안내된 현장 확인 후 잠금 프롬프트를 사용한다. `READY`와 `status`에서 Gate NORMAL, 단일 Nav2, 센서·TF·출력 상태를 확인한다. Depth와 slow 조합은 보존된 배포본에서 지원되지 않는다.
3. 실제 주행은 현장 승인 후 기존 번들의 drive 절차로 별도 세션을 열고, `READY` 확인 후 운영자가 `frontier`를 입력한다. 영상은 별도 읽기 전용 시각화 콘솔에서 확인한다. 카메라나 관제 장애는 탐색 실행 조건으로 사용하지 않는다.
4. 이상 시 현장 정지 수단을 사용하고 `pause`로 Goal terminal 및 정지를 확인한다. `save` 후 raster와 posegraph 결과를 따로 확인한다. `stop`으로 자기 bag과 launch를 종료한다. 실행 디렉터리의 `summary.json`, YAML/PGM, posegraph/data, bag과 로그를 확인하고 `ros2 bag info <bag>`를 실행한다. OccupancyGrid와 posegraph는 별도 시점에 저장될 수 있다.
5. 보존된 현장 supervisor는 자기 Nav2가 독점이라는 전제에서 pause 후 광역 취소를 보낸다. 다른 Goal 제출자가 존재하거나 결과가 불명확하면 Frontier를 시작하거나 재개하지 않는다. 이 브랜치의 Explorer UUID별 취소 수정은 현장 설치본에서 검증·배포되지 않았다.

LOCAL_FRONTIER는 Explorer 로그의 UUID와 현장 run 디렉터리로 추적한다. FLEET_NAVIGATION은 Fleet Mission ID와 Agent UUID 로그를 연결한다. RMF_TASK는 기존 RMF principal 및 rmf idempotency key를 통해 같은 Fleet 경로의 임무에 연결한다. WEB_MANUAL은 별도 속도 제어다. ROS Action 상태 검사는 관측된 Goal을 차단하며 동시 최초 제출의 원자적 잠금은 제공하지 않으므로 실행 모드별 제출자 분리가 필요하다.

### 웹 조이스틱 격리 실행

Fleet 테스트 서버와 Mock Nav2 Action Server를 별도 환경에 두고 Agent를 `mode:=mock simulated:=true allow_missions:=false manual_web_bench:=true`로 실행한다. Chrome에서 localhost 또는 HTTPS 관제 페이지를 열고 게임패드를 조작하여 장치·축·버튼 표시를 노출한다. 장치 확인 → 실제 축·Deadman 번호 및 방향 설정 → Fleet 제어권 획득 → Deadman을 놓은 상태에서 모의 입력 시작. Deadman을 놓거나 연결이 끊기면 세션이 종료되어 다음 입력 전에 명시적으로 다시 시작한다. 같은 탭에서 Lease를 갱신할 수 있다.

현장 자동 맵핑은 위의 기존 Runbook만 사용한다. 이번 모의 수동 제어는 ROS 속도를 발행하지 않는다. 후속 잠금 검증의 조건은 Nav2 Goal terminal 확인, 지정 모드의 단일 제어권, Smoother/Collision Monitor/Gate remap 확인, Gamepad 실물 매핑과 해제·단절 검증이다. 배터리·USB–UART·Gate·정지 수단·별도 구동 승인도 현장에서 확인한다.

11월 19일 기존 현장 번들의 로컬 실행 순서와 프로파일을 유지한다. 새 Fleet Mapping API는 모의 플래그를 등록한 worker 전용이다. UNKNOWN 세션에는 자동 재개나 Goal 제출이 없다. 후속 연결은 실제 Goal UUID 쌍의 종료 증거, SLAM 상태, 래스터와 posegraph의 개별 저장 결과, session이 시작한 rosbag만 종료하는 절차를 실제 worker로 연결한 뒤 격리 ROS에서 재시험한다. Depth는 기존 읽기 전용 콘솔에서 계속 볼 수 있다.

4차 통합 이후 기능별 ROS 2 구조 정리의 기준은 공개 `main` `1ca1393d229a1a3e1727079d61f8a1210a9520ac`와 4차 선택 변경이다. 복구 위치는 `WORKTREES.md`에 기록했다. 실제 Mapping·Bag Worker부터 격리 환경에 연결한다. Goal terminal과 소유 프로세스 종료를 관측한 뒤 완료 상태를 보고하고, 게이트 속도 경로는 별도 검증한다.

Phase 3A의 Bag Worker 웹 기록·재생은 격리 ROS에서 확인됐다. 중복 STOP, 실제 Agent/Server 재시작, 통신 단절, 녹화 중 비정상 종료·강제 종료 및 다른 recorder 보호 시험이 남았다. 연결 유실 중 recorder는 종료되지 않을 수 있으며 재시작 시 소유권 복구 없이 UNKNOWN으로 남는다. 이 상태에서 현장 적용을 진행하지 않는다.

Phase 3A 격리 Worker는 실행 중 최대 300초, 파일 합계 256 MiB, 남은 디스크 1 GiB, Fleet 단절 5초를 검사해 recorder 종료를 요청한다. 프로세스 PID·start ticks·PGID·session·경로를 저널에 남기며 재시작 시 인수하지 않는다. Agent가 SIGKILL되면 검사 스레드도 함께 중지되므로 살아남은 recorder의 무제한 기록을 아직 방지하지 못한다. 이 한계와 잔여 장애 E2E를 해결하기 전 Phase 3B를 시작하지 않는다.

이전 단계에서 `isolated_ros_bag`를 독립 Supervisor 부재로 fail-closed 처리했다. 기존 Bag 코드는 보존돼 있으며 활성화 전에 세션 단위 cgroup 및 독립 자원 감시를 연결해야 한다. 이후 격리 ROS Worker와 Depth 화면 연결을 추가했다.

### 전체 통합 검증 인계

현재 격리 경로에서 독립 Bag Supervisor와 실제 Mapping Worker를 사용할 수 있다. 새 하네스는 로컬 Fleet, 두 로봇, 합성 센서, 실제 SLAM·Nav2·rosbag2를 새 런 디렉터리에서 시작한다. 명령 결과의 UUID 및 세션 저널을 [격리 통합 매트릭스](fleet_control/docs/ISOLATED_E2E.md)에 따라 대조한다. UNKNOWN/FAILED의 원본 저널과 Bag·지도 파일을 보존한다. 실차 시나리오는 별도 승인 이후 기존 Runbook으로 진행한다.

### 잠금 캠페인 후속 확인

A–E 기준선과 캠페인 수정분은 별도 해시로 보존한다. 이전 기본 임계값으로 생성된 PGM/YAML은 Map Server가 미관측 셀을 자유 공간으로 해석할 수 있다. 캠페인은 원본을 보존하고 같은 Gazebo Bag으로 수정본을 별도 생성해 분류를 확인했다. 다음 검증 범위는 Gazebo에서의 폐루프 이동, 실물 Gamepad, 별도 승인 후 실차다. 격리 Launch의 모터 잠금과 기존 실차 진입점을 유지한다.

## 2026-11-20 공개 기준선과 후속 작업 감사

실차 웹 관제는 구동 차단 상태다. 최신 실차 Launch에 Watchdog이 편입되지 않았으며 STM32 명령 timeout, 물리 E-Stop, 제동 거리, Depth 높이 실측, wheel odometry 안정화, localization 확인, 실차 Operator Recovery가 남았다. LIMO 연결과 모터 잠금 웹 시험은 미확인 상태다.

현장 인계 기록상 원래 18190 런처가 해제된 워크트리의 소스를 참조한다. 별도 백업 Registry 사본에서 18190과 C103 표시를 확인한 기록은 현장 Registry 전환과 구분한다. Gate·실차 Launch 담당은 02b·03b·04를 미적용 상태로 관리한다. Fleet/Agent 담당은 CONTINUE_MAP Worker와 실차 복구를 맡는다.
