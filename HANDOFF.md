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
   - 경로: 로컬 미추적 `experiments/real_limo_field_20261113/`, `experiments/real_depth_integration_20261113/`, `<WORKSPACE_PARENT>/Capstone1_limo_dashboard/fleet_control/dashboard`
   - 검증(**실차**):
     - LiDAR-only 저속(0.15 m/s) 자동 맵핑(`run_010`, `real_auto_mapping_ready_20261113` 배포본): 약 110초 실제 구동, 지도 1,080→30,430셀 생성 및 `map_001`/`map_002` 저장.
     - 모터 잠금 상태에서 신규 Robot Agent 배포(`~/limo_field_agent_ws`), `409 agent_session_mismatch` 해결 및 지도 API 업로드(`map.png` HTTP 200) 확인.
     - 모터 잠금 상태에서 DaBai RGB → Tailscale → 노트북 CUDA YOLO11n 왕복(10 fps) 및 신규 시각화 대시보드(`dashboard`) 15초 제한 실차 `/map`·Costmap·TF·Depth(SSH `bridge_node.py`) 및 별도 15초 RGB(`readonly_rgb.py`, 5 fps) 표시 확인.
4. **대시보드 스트리밍 연구소(`dashboard_lab`), v2 설계, `rmf-web` 조사 및 통합 시뮬레이션 (2026-11-12~13, Codex / Claude Code)**
   - 로컬 `<WORKSPACE_PARENT>/Capstone1_limo_dashboard`: `dashboard_lab` WebSocket JPEG 11.90 fps / Depth 7.99 fps 오프라인 검증(121 PASS), `dashboard_v2_design` 계약 문서화, `rmf_web_research` Storybook(18410) 확인.
   - 로컬 `<WORKSPACE_PARENT>/Capstone1_limo_fleet_rmf`: Gazebo × Open-RMF E2E 69/69 PASS(필수 15/15 PASS).
   - 로컬 `<WORKSPACE_PARENT>/Capstone1_limo_integrated_sim`: RMF × Gazebo × Nav2 × Video/YOLO 단일 통합 실행(`final4`) 12/13 PASS 및 장애 주입 복구 확인.
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
   - 남은 정리 대상: (a) 메인 폴더 `c804241` 미추적 실험/노트 정리, (b) *(완료)* `integrate_worktree` 정식 해제(브랜치 `integrate/privacy-preserved-main` `b1d14de`·고유 커밋 11개 보존) 및 `~/limo_runs` 스모크 원본 정리(복사본 `local_data/raw_results/limo_runs/`), (c) 홈의 독립 학습·과거 기록 폴더(`~/slam_toolbox_source`, `webots_limo_pro`, `coppeliasim_limo_pro`, `notebook/`)는 삭제하지 않고 현 위치 유지, (d) `ed53d81` 및 `a021186`의 `origin/main` PR 반영.

---

## 3. 다음 권장 작업 (Next Steps)

1. **[실차 비구동 진단]** LIMO 완충 후 충전기를 분리한 상태에서 모터 잠금 모드로 `power_logger.py` 소크 시험을 수행해 전원 차단 및 USB 재연결 재발 여부 확인.
2. **[정책 승인 및 코드 통합]** 사용자와 [DECISIONS.md](DECISIONS.md) `C-01`의 미확정 정책 3건을 확정한 뒤, `experiments/safety_chain_fix_20261113/fixed2`의 검증된 노드를 실차 준비 번들 및 `capstone_ws/src/limo_capstone`으로 이식하는 계획 수립.
3. **[폴더/Git 후속 마무리]** 사용자 승인 하에 (a) `feat/fleet-control-unified`(`ed53d81`) 및 공통 문서 증분(`a021186` + 이번 `WORKTREES.md`/`CODE_INDEX.md` 등) `origin/main` PR 반영, (b) `run_recorder.py`·`AUTONOMOUS_DRIVING_DESIGN.md` 출력 경로 수정(현재 로컬 미커밋)의 커밋 여부 결정. 홈의 독립 학습·과거 기록 폴더는 유지.

---

## 4. 작업 시 주의 사항 (Cautions)

- **실차 구동 금지**: 사용자 현장 입회, 배터리 완충 확인, 물리적 정지 준비, 명시적 승인 없이는 `--drive` 또는 `allow_motor_drive:=true`를 절대 실행하지 마세요.
- **타 에이전트 작업 및 경로 보호**: `Capstone1_limo_fleet_rmf`, `Capstone1_limo_video_preverify`, `experiments/office_limo_sim_20261113` 경로는 다른 실험 스크립트(`lf_env.sh`, `lf_setup.sh`, `offline_registration_test.py`, `office_limo_*.launch.py`)가 하드코딩으로 참조하므로 임의 이동·삭제를 금지합니다.
- **민감정보 보호**: `Capstone1_limo/fleet_control/{data,secrets}`, `$HOME/.config/limo_field_20261113/`, `$HOME/.local/share/robot-sim-projects/limo_video_ai_preverify/real_limo_kit/secrets/` 내 토큰과 `.env`, DB는 절대 Git에 커밋하거나 Google Drive에 업로드하지 마세요.


## 2026-11-15 공통 문서 조사 인수인계

Registry·Operation Manager·Gate의 근거와 한계를 [CURRENT_STATUS.md](CURRENT_STATUS.md), [CODE_INDEX.md](CODE_INDEX.md), [VALIDATION.md](VALIDATION.md)에 추가했다. 기존 제안과 Gate 작업 파일은 보존했다. Phase 1.3 ROS 시험의 저장된 실행 로그와 v4.2 결과 2~4절을 확인한 뒤 결과를 갱신해야 한다. 복구 실행, Goal 제출 경계 B안, attestation 및 증거 신선도·epoch 계약은 미확정/미구현 항목으로 남는다. 기존 [Phase 1](fleet_control/OPERATION_MANAGER_PHASE1_PROPOSAL.md), [Phase 1.1](fleet_control/OPERATION_MANAGER_PHASE1_1_PROPOSAL.md), [Phase 1.2](fleet_control/OPERATION_MANAGER_PHASE1_2_PROPOSAL.md), [고아 복구 제안](fleet_control/server/ORPHAN_RECOVERY_PROPOSAL.md)과 Claude Gate 보고를 삭제하거나 확정 결정으로 전환하지 않는다.

### 담당별 미적용 제안 및 충돌

- **Claude / Gate**: `gatev42r2_live_20261115` 실행 산출물이 조사 중 계속 생성되는 것을 확인했다. 최종 보고가 나오기 전 v4.2 전체 PASS·fixed4 승격을 공통 문서에 기록하지 않는다. 표본 최초 부재·GID 연속성·다중 action 감시 및 epoch 복구 계약을 담당자 결과와 대조한다.
- **Codex / Operation Manager·Agent**: [Phase 1.2 제안](fleet_control/OPERATION_MANAGER_PHASE1_2_PROPOSAL.md)의 B안과 terminal·소유권·외부 목표 차단 조건은 승인 및 구현 전이다. 격리 ROS 시험 실행 로그를 보존·대조하고 직접 ActionClient 통제 방식을 설계한다.
- **Antigravity / 공통 문서**: 이전 공통 문서의 워크트리 개수와 일부 fleet 구현·검증 서술은 현재 확인과 차이가 있다. 현황 추가 항목을 기준으로 기존 이력의 본문 통합을 검토하되 기존 제안을 삭제하지 않는다.
- **Registry 담당**: [고아 복구 제안](fleet_control/server/ORPHAN_RECOVERY_PROPOSAL.md)의 운영 복구·부하 대응은 별도 설계·검증 전까지 보류한다. 진단은 운영 DB나 쓰기 중 번들에 적용하지 않는다.

- **Operation Manager 후속 근거**: 사용자 선택 B안은 운영 구현이 아닌 설계 기준이다. [Phase 1.3~1.6 색인](CODE_INDEX.md)의 격리 시험에서 중재 프로토타입과 SROS2를 각각 검증했으나 함께 연결하지 않았다. 외부 Goal 자동 감지, 실제 Agent 및 Gate 통합, 프로세스 재시작·키 권한 강화가 남았다. Gate 담당 기록은 변경하지 않았다.


## 2026-11-16 Fleet UI 개편 인수인계 (Codex)

- 완료: 실제 Vanilla 대시보드 3열 관제·하단 제어/이벤트·고정 상태바, 기존 상세 기능 분리·보존, 미리보기 Goal 차단, 오래된 상태의 미확인 표시. [보고/실행법/스크린샷](fleet_control/docs/DASHBOARD_UI_20261116.md).
- 검증: Chromium 23 checks, Python 회귀 44 tests, JS 2 suites PASS. 큰 화면 3해상도 overflow 없음. 메뉴 확대는 SKIP(대응 viewport만 PASS), 실제 센서/실차·승인 이미지 일치도 SKIP.
- 주의: 전체 `fleet_control/`은 시작부터 미추적이었다. Git diff만으로 변경분을 판정하지 않는다. 서버·Agent·Gate·RMF 및 타 담당자 본문 미수정, commit/push/PR 없음. 없는 API를 가짜 정상/허가/영상으로 채우지 않는다.


## 2026-11-16 Fleet UI 2차 개편

기능별 탭과 공유 지도, 키보드/뒤로 가기를 추가했다. 실제 코드, 격리 Chromium 캡처·시험 및 기능 매핑은 [보고](fleet_control/docs/DASHBOARD_UI_PHASE2.md) 참조. 실차/Gazebo 및 승인 이미지 직접 비교는 SKIP. 서버·Agent·Gate·RMF 로직 미수정.


## 2026-11-16 Spatial Monitor 3차 UI

조회 전용 공간 탭·공유 지도/RGB, Depth 및 경로 미제공 표시 추가. 합성 loopback Chromium과 기존 안전/Registry 회귀. 실제 Depth/계획 연동과 점유 영역 맞춤은 미구현. [보고](fleet_control/docs/SPATIAL_MONITOR_PHASE3.md), [인터페이스 요청](fleet_control/docs/SPATIAL_MONITOR_INTERFACE_REQUEST.md). Gate/Agent/Server/RMF 미수정; 실차/Gazebo 미실행.


## 2026-11-16 Spatial 3가지 보기

Spatial Monitor에 Building/Floor/Driving 모드를 추가했다. 층별 Registry 미리보기와 조회 전용 지도, 영상 확대, Fault/Offline/미확정 표시를 확인했다. 건물 소속·Depth·Nav2 계획 API는 미제공. [보고](fleet_control/docs/SPATIAL_VIEWS.md).


## 2026-11-16 Fleet UI 전체 리디자인

Fleet UI 사이드바·공통 스타일과 주요 화면 재배치, 임무 상태 단계 및 Building 지도 확대. [화면·검증·매핑](fleet_control/docs/FLEET_UI_REDESIGN.md). 실차/Gazebo 및 단계별 중간 캡처 SKIP; Gate/Agent/Server/RMF 변경 없음.


## 2026-11-16 지도 집중 모드

지도 설정 접힘 시 실제 캔버스 확장, Overview/Spatial 읽기 전용 및 안전 상세 접기 구현. [측정·검증](fleet_control/docs/MAP_FOCUS_DENSITY.md). Gate/Agent/Server/RMF 미수정.


## 2026-11-17 Fleet UI 테마

System/Light/Dark 및 세 우선 화면 리디자인. [캡처·레퍼런스 비교·검증](fleet_control/docs/UI_THEME_REDESIGN_20261117.md). 실제 스트림/실차/Gazebo와 네이티브 대화상자 테마는 SKIP. Gate/Agent/Server/RMF 미수정.


## 2026-11-17 승인 Workspace 운영 UI 적용

사용자 승인 Workspace를 기존 static UI에 적용했다. Overview/Operation 로봇·임무 목록 + 공유 지도 + Inspector, 접근성 Goal Drawer와 좌표 확인, 34px 상태바, 카메라 접기/너비. 실제 loopback API로 임무 생성·조회·취소 PASS. [보고/캡처/재현](fleet_control/docs/WORKSPACE_RUNTIME_20261117.md). Gate·Agent·Server·RMF 미수정. 실차/Gazebo 및 실제 센서 SKIP.


## 2026-11-17 `/map` 타입 분리 및 Gazebo rosbag 검증 인수인계

`/map`=OccupancyGrid, BuildingMap=`/rmf/building_map`(Gazebo/RMF 쪽 `LD_PRELOAD` shim, 임시 호환 계층). 지도 두 종류의 bag 기록·재생을 Gazebo 2회 확인. [보고/재현](fleet_control/docs/RMF_MAP_TYPE_SEPARATION_20261117.md). 실차 미검증, DECISIONS 미변경.

- **다음 Fleet 통합 작업**: Gazebo에서 RMF/Fleet 스택을 띄울 때는 sim launch와 `building_map_server`를 shim 아래에서 시작하고, 새 BuildingMap 소비자는 `/rmf/building_map`을 쓸 것. 근본 해결은 `rmf_robot_sim_common` 재빌드(토픽 설정화)이며 shim의 장기 채택 여부는 사용자 설계 결정 대기.
- **Posegraph 등록(미해결)**: Agent `upload_slam_map` 3.8 MB 임계값과 서버 `MAX_MAP` 6 MiB(base64 ×4/3) 때문에 Office 지도 posegraph(약 5 MB)는 등록에서 빠진다(`posegraph_registered:false`). 분할/스트림 업로드 설계 필요, 지도 커질수록 항상 발생.
- **센서 기록 잔여**: `/scan_slam`은 sim에서 발행되지 않아 기록 FAIL 유지 — 실차 현장 graph에서 발행·기록 확인. CameraInfo·원본 영상은 기록 프로파일 밖(포함 여부는 별도 결정).
- **주의**: `run_recorder.py`에 다른 에이전트의 미커밋 변경이 있어 수정하지 않음. `&`로 기동한 하네스 자식은 SIGINT를 무시하므로 시험 소유 프로세스만 SIGTERM으로 정리(`ROS_DOMAIN_ID`로 구분). Gate r9/r10 Cliff, Fleet Goal 소유권, Gateway 접근 제어의 기존 미해결 기록은 변경하지 않았다.


## 2026-11-18 Fleet 단일 임무 Gazebo 인수인계

수정: Agent `safety_protocol.py`·`ros_adapter.py`, Server `safety.py`(SIM_ONLY 제한 신뢰 모델). 시험·증거: `experiments/fleet_followpath_trust_20261118_N3p5/`, [보고](fleet_control/docs/FLEET_FOLLOWPATH_OWNERSHIP_20261118.md).
- **다음 작업(우선순위)**: (1) 외부 NavigateToPose goal 차단 설계(Gateway/ACL/인증, 새 구조 필요 — 이번 범위 밖), (2) Agent 재시작 후 Gate 재무장·Server `recover` `no_evidence`, (3) Gate `MEMORY_NOT_APPLIED_SHADOW`·기동 래치는 Gate 담당에게 전달(실행 14회 중 7회 환경성 무효), (4) Goal 결과 지연·중복 실주입 방법, (5) 반복·장시간 시험.
- **주의**: 이 신뢰 모델을 실차에 쓰지 말 것. 공통 문서가 이 브랜치(`c804241`)에서 미추적인 이유는 `origin/main`이 `d9c7c42`(PR #9/#10)에서 문서를 커밋했고 이 브랜치가 그 이전(`6129d9f`)에서 갈라졌기 때문(병합 안 함). 기존 `test_recovery.py` 11 FAIL은 원본에서도 동일. 하네스 자식 프로세스 정리는 `ROS_DOMAIN_ID`로 구분해 시험 소유분만.

## 2026-11-18 MANUAL SLAM Gazebo 인계

r8 런치 `manual_mapping:=true`는 SLAM·explore 자동 기동을 제외하고 첫 `/map` 후 Nav2를 기동한다. Agent는 SIM_ONLY·`allow_missions=false`에서 웹 SLAM을 감독한다. Gazebo 증거와 초기 실패, Gate 래치 및 ROS graph 관측 한계는 [VALIDATION.md](VALIDATION.md)의 2026-11-18 항목 참조. Gate·Cliff 로직과 실차는 건드리지 않았다.

## 2026-11-18 MANUAL MAPPING Joy 인계

SIM_ONLY Gazebo에서 Joy → Agent → Collision Monitor → Gate → GT 전진·회전·watchdog 정지 및 지도 변화를 검증했다. Agent 옵션 `manual_mapping_enabled:=true`, `sim_r8_supervisor:=true`, `allow_sim_safety_gate:=true`, `allow_missions:=false`와 유효 Fleet 임대가 필요하다. Gate 기존 readiness/reset 절차를 유지한다. 로그와 시작 시 래치 원인은 [VALIDATION.md](VALIDATION.md) 참조. 웹 STOP·지도 저장·재로드는 이번 시험 범위가 아니다.

## 2026-11-18 MANUAL STOP 인계

MANUAL 웹 STOP은 입력 차단 후 Gate 출력·Gazebo GT 정지 확인, 지도 저장·원본 셀/정밀 메타데이터 확인, Registry 무결성 확인, SLAM 종료 순서로 동작한다. 실패 시 `ERROR`, 입력 비활성 및 파일 보존, STOP 재시도 가능. 실제 SIM_ONLY 연속 실행과 Registry 번들 재로드 근거는 [VALIDATION.md](VALIDATION.md) 참조. 재로드는 시험에서 Nav2 map_server를 직접 실행했으며 별도 웹 reload 제어 구현이나 실차 검증을 뜻하지 않는다.

추가 연속 실행 `manual_finish_332030`은 원점 x/y/yaw·해상도·전체 셀을 Registry 재로드 결과와 직접 비교해 일치했다. STOP 중 Joy가 계속 들어오는 조건에서도 정지 확인 후 저장했다. [상세](VALIDATION.md).

## 2026-11-18 AUTO MAPPING 미완료 인계

후보 전용 `fleet_frontier_candidates` 및 미연결 `FrontierFleetCoordinator`가 추가됐다. `auto_mapping:=true`는 격리 SIM 후보 평가용이며 AUTO 임무를 시작하지 않는다. Gazebo 실제 시도에서 웹 AUTO 404, 기존 Fleet 임무 409(`safety_not_ready`), Gate `supervisor_alive` 미충족으로 운전하지 않았다. `slam_sim_enabled`는 `allow_missions=false`를 요구하며 Fleet은 `NAVIGATION`·검증 지도·확정 localization을 요구하므로 라이브 맵핑 전용의 안전 상태 계약과 STOP 소유 Goal 종료 절차를 실제로 설계·구현하고 증거를 검증해야 한다. 상태 fixture로 verified map/localization을 위조하거나 소유권 검사를 끄지 말 것. [실험과 재현](VALIDATION.md).

AUTO Supervisor 후속 `344513`: 기존 Agent supervisor를 AUTO SIM 전용·미션 비활성 opt-in에 연결하여 실제 heartbeat·Gate NORMAL을 확인했다. 기존 RMF map shim으로 `/map` rosbag 기록도 확인했다. 웹 AUTO 404 및 live-SLAM Mapping Goal 계약 미구현으로 자동 이동은 시작되지 않았다. 상세 실패·rosbag은 [VALIDATION.md](VALIDATION.md) 참조. 사용자는 실차 준비를 Gazebo 성공 이후에 요청했다.

## 2026-11-18 실제 LIMO Agent 비구동 인계

사용자 지시에 따라 `pcu`의 기존 `~/limo_field_agent_ws` Agent만 백업 후 최신 로컬 패키지로 빌드했다. 이전 workspace 아카이브 및 두 차례 임시 Fleet 인증·재접속 결과는 [VALIDATION.md](VALIDATION.md)에 기록했다. 실차 bundle overlay를 먼저 source해야 `limo_msgs`가 인식됐다. 시험 Agent와 임시 서버는 종료했고 실차 센서 스택·모터는 시작하지 않았다. 실제 물리 정지 수단·전원/USB·통신 단절 시 모터 정지 및 SIGSEGV는 미해결이다. Web 실차 SLAM/녹화 제한과 AUTO 미완성은 유지한다.

## 2026-11-18 실차 Depth·지도 저장 후속 인계

run_003 모터 잠금 기동, Depth 실측 개선, SLAM/Fleet 웹 회귀 및 실차 스냅샷 격리 저장·재로드 완료. run_002 중복 검사는 기존 Agent가 종료돼야 한다. 다음 재시작도 현장 런처로 READY를 확인한 뒤 읽기 전용 Agent를 연결한다. [상세 증거와 롤백](local_data/raw_results/real_limo_depth_map_deploy_20261118_01/).

**우선순위**는 베이스 `/odom`·`/imu`·`/limo_status` 동시 중단의 원인 확인과 재현이다. 물리 전원·USB/MCU·시리얼 수신 증거를 확보하고 잠금 모드에서 세 스트림 복구 및 Gate 안정성을 확인한다. 다음으로 잔여 Depth TF 7회 만료 구간과 카메라 TF 중복 부모를 조사하고, 현장 세션의 정지 확인→정확한 스냅샷 저장→posegraph 경로를 재시험한다. 저속 Depth 설정과 물리 정지·통신 단절 정지를 검증한 뒤 별도 승인으로만 주행한다. Agent SIGSEGV는 정상 종료 한 번으로 해결 확정하지 않는다.

## 2026-11-18 베이스 스트림 진단 인계

실차 run_003은 최신 base telemetry를 받지 못해 계속 잠금·FAILSAFE다. 우선 현장 전원·물리 정지 조건을 확인한 뒤, 승인된 잠금 시험에서 kernel·udev·제어기 진단과 세 ROS 스트림을 같은 시간축으로 기록해 손실 시점의 데이터 경로를 분리한다. 현재 열린 포트에 두 번째 리더를 붙이지 않는다. 재현 원인이 확인되고 지속 수신/Gate 안정성이 확보된 뒤 실제 조이스틱의 장치·축·deadman을 입력 전용으로 확인한다. 실차 Joy 중재와 잠금 출력을 검증한 뒤 SLAM·bag·지도/posegraph·Fleet을 점검하고 주행은 별도 승인으로 진행한다. [재현 스크립트와 로그](local_data/raw_results/real_limo_base_stream_20261118_01/README.md)

## 2026-11-18 베이스 복구 후 인계

CP2102 포트 초기화 EIO에서 단독 드라이버 교체 실패, 사용자 재부팅 후 잠금 run_004에서 2분간 연속 수신 및 Gate NORMAL 회복. 원인은 USB–UART 계층까지 좁혔고 전원·케이블·장치 중 물리 원인은 미확정. run_004와 읽기 전용 Fleet 관제 유지. 이후 장기 잠금 시험에서는 같은 시간축으로 커널/USB·세 스트림을 기록한다. 실차 수동 프로파일 배포 전 장치 축·deadman·재연결과 독점 명령 경로를 잠금 상태에서 확인하고, SLAM·bag·정지 후 지도/posegraph 및 재로드를 이어서 검증한다. 주행은 별도 승인. [증거](local_data/raw_results/real_limo_base_recovery_20261118_01/README.md)

## 2026-11-18 조이스틱 연결 후 현장 상태

베이스 교체 PID 25511은 run_004 supervisor 밖에서 시작했으며 속도 입력 격리 상태다. 다음 종료·기동 시 소유권을 확인해야 한다. 공유 허브 재발이 관측됐으므로 웹 운용용 조이스틱은 노트북 쪽에서 검증한다. 다음은 브라우저→Fleet→실차 Agent의 권한·deadman·단절 및 속도 발행 소유권을 잠금 상태에서 확인하는 작업이다. 물리 정지 수단·전원·USB·시험 공간 확인과 별도 주행 승인까지 구동 금지. [증거](local_data/raw_results/real_limo_joystick_locked_20261118_01/README.md)

## 2026-11-18 웹 수동 벤치 인계

실차 Agent 선택 배포·롤백 경로와 격리/현장 증거는 [시험 기록](local_data/raw_results/web_manual_bench_20261118_01/README.md). 진행 순서는 수동 우선. 12:34:48 KST 이후 CP2102 별도 베이스가 더 이상 세 스트림을 제공하지 않아 원인·연속 수신을 잠금 상태에서 재확인한다. 기존 run_004 정상 마감 및 분리 베이스 프로세스 소유권 확인 후, 현장 잠금 프로파일 후보에서 Nav2 raw 출력 격리와 실물 노트북 게임패드를 확인한다. 실제 구동은 현장 물리 정지·공간·전원·연속 베이스 및 별도 사용자 승인 이후에만 진행한다. 기존 영상은 별도 송출 경로로 현재 Fleet 인스턴스에 연결되지 않았다.

## 2026-11-18 수동 최종 연결 전원 차단 인계

현장 배터리 부족 보고 당시 run_004의 베이스 단독 복구가 실패했다. 충전 후 잠금으로 다시 기동해 연속 odom/imu/status와 Gate 정상 상태를 먼저 확인한다. 기존 SLAM·계측을 중복 실행하지 않고, 원본 rosbag 대신 로봇의 20261118_run_004_bag_recovered_copy를 읽는다. 검증된 Nav2 격리 후보에서 운용자 실물 Gamepad 입력과 단절을 확인한 뒤 주행은 별도 현장 승인. [로그](local_data/raw_results/web_manual_field_final_20261118_01/README.md)

## 다음 현장 작업 우선순위

1. 전원·USB 통신과 물리 정지 수단을 확인하고, 별도 주행 승인 후 기존 실차 Frontier 경로를 시험한다.
2. 각 실행의 지도·posegraph·rosbag을 확보하고 재로드와 원본 보존을 확인한다.
3. 현장 건물·층 식별 정보를 확인해 Map Registry에 미검증 지도를 등록한다.
4. 저장 지도 기반 Nav2 위치 추정과 짧은 목표를 별도 승인 후 시험한다.
5. 웹에서 지도·RGB·Depth 동시 모니터링을 연결하고 검증한다.
6. 웹 AUTO와 조이스틱 캘리브레이션·버튼 설정을 후속 개발한다.

기존 실차 실행 진입점은 현장 LIMO의 `~/real_auto_mapping_ready_20261113/start_real_auto_mapping.sh`이며, 초기 실행은 모터 잠금이다. Frontier 및 주행은 현장 조건 확인과 별도 승인이 필요하다.

## 2026-11-19 LIMO 전용 단일 워크스페이스(`robot_ws`) 구성 및 비주행 검증 인계

- **완료**: LIMO `/home/pcu/Capstone1_limo/robot_ws`에 최소 9개 패키지만 선택 배포하고 `/opt/ros/humble` 단독 기준 독립 빌드 완료(기존 `/home/pcu/capstone_ws` 미수정·미참조). CP2102 중복 시리얼(`0001`) 배제 후 `ID_PATH` 기반 `/dev/ttylimo`(베이스) 및 `/dev/ydlidar`(LiDAR) 실통신 검증. FastDDS Discovery Server(`<TAILNET_DISCOVERY_IP>:11811`, `ROS_DOMAIN_ID=77`, `ROS_LOCALHOST_ONLY=0`)로 모터 잠금 전체 LiDAR-only 느린 주행 스택(`allow_motor_drive:=false`) 검증, 노트북 로컬 원본 rosbag2 기록(`local_data/raw_results/limo_runs/20261119_locked_verify/rosbag`) 및 지도 저장·이송·재로드 완료. LIMO 내 rosbag·잔류 프로세스 0.
- **차단/주의**: 15:03 잠금 실행에서 `wheel_odom_stale: true` → Gate `FAILSAFE_SENSOR_TIMEOUT` 1회 관측(15:06 기록 구간에서는 113/113 NORMAL). 물리적 정지 수단, USB/전원 유지, 현장 공간 확인 및 사용자 명시적 주행 승인 전 `allow_motor_drive:=true` 금지.
- **다음 작업**: 현장 운영자 승인 후 `robot_ws/env.sh` + `fastdds discovery` + 노트북 `local-only/limo_robot_ws/env_laptop.sh` 기반 rosbag2 기록을 먼저 켠 상태에서 `allow_motor_drive:=true` LiDAR-only 저속 자동 맵핑 시험 진행. [상세 검증](VALIDATION.md)


## 2026-11-19 자동 맵핑 중단 재시험

잠금 감시 시험에서 지도·스캔·Gate·프론티어 수신을 확인한 뒤 사용자 승인 하에 저속 실행했다. 감시 출력 정체 및 녹화 조기 종료로 SIGINT 중단했고 맵 완성은 확인하지 못했다. 마지막 bag 지도 306×251, known 23,239; 복원 지도·부분 영상·로그·무결성 확인된 bag은 [결과 폴더](local_data/raw_results/limo_runs/20261119_auto_map_full/)에 있다. 감시 출력 정체 원인과 녹화 조기 종료 원인은 미확정이며 재주행 전 해결이 필요하다.

## 2026-11-19 자동 맵핑 재실행 02 안전 중단

- 사용자가 현장 확인·저속 주행을 승인해 새 실행으로 기록을 시작했다. 약 45초에 Gate `FAILSAFE_SENSOR_TIMEOUT`, `wheel_odom_stale=true`, `cmd_blocked=true` 관측. 감시기 3.1초 지속 감지로 종료 코드 2, launch SIGINT; 이후 이번 LIMO 프로세스 부재 확인. 이전과 같은 odom stale 현상이 재발했으나 물리 원인은 미확정이며 재주행하지 않았다.
- 노트북 bag 14,603건, SQLite `ok` 및 `ros2 bag info` PASS. 마지막 `/map` 272×219, known 12,487; bag에서 복구한 PGM/YAML은 격리 도메인 219 map_server 메타데이터 재로드 PASS. 완성 지도 아님. 로봇 map_saver 및 SHA 비교 SKIP. 관제 녹화 342프레임을 MP4로 저장. [결과](local_data/raw_results/limo_runs/20261119_auto_map_full_02/).

## 2026-11-19 웹 Mapping·C103 지도 등록 인계

- **구현·격리 검증**: C103 영역·지도 등록 스크립트, 웹 START/PAUSE/RESUME/SAVE/STOP, Geofence·종료 한계, SAVE→UNVERIFIED 새 버전(area·lineage), 지도 클릭 사전 검사, 맵핑 중 이동 임무 차단.
- **미구현**: CONTINUE_MAP posegraph 적재(서버가 `continue_unsupported`로 거부), 재방문·품질 평가, Registry 영역별 Geofence 저장, 버전 복원 HTTP.
- **현장 적용 전 승인 필요**: 현장 Registry 등록(`register_c103_map.py --data-dir <현장 data>`), 18190 서버 교체, Agent 배포. 실차 Mapping과 주행은 서버가 차단하며 안전 차단 조건이 남아 있다.
- [상세](fleet_control/docs/WEB_MAPPING_20261119.md)

## 2026-11-19 후속 인계 (실차 전환 판정: 불가)

- **P0**: Agent가 죽으면 Agent가 제출한 Nav2 goal과 속도가 지속된다(격리 관측 약 14.6 s 이상). Gate heartbeat 요구 또는 독립 goal 취소 감시 노드가 필요하다. Gate 파일은 다른 에이전트의 미커밋 변경이 있어 수정하지 않았다.
- **P0 실측**: 바닥→`base_link` 높이, 카메라 높이·pitch, LiDAR 높이, 낮은 장애물 높이. 실측 후 `base_footprint_z` 적용과 잠금 상태 정적 시험을 진행한다.
- **P1**: CONTINUE_MAP Worker 통합(localization 확인 필수), ownership·recovery 시험 픽스처 갱신, Gazebo SIM_GATED 실행 구성.
- 현장 Registry 백업 B0: `local_data/field_registry_backups_20261119/` (`auth.json` 포함, 공개 금지).

## 2026-11-20 인계

- **완료(격리)**: Motion Watchdog 상태기계와 고장 7종 검증, 회귀 75/75, 구조 감사 문서.
- **실차 차단(P0)**:
  - 제동 거리 미측정, MCU `/cmd_vel` 타임아웃 미확인
  - 패치 02(실차 launch watchdog 편입)·03(Gate 요구) 미적용(다른 에이전트 파일)
  - `base_footprint_z` 실측
- **P1**:
  - 실모드 운영자 복구 경로 없음(`terminal_evidence_unavailable`)
  - SIGCONT 후 executor 미재개(간헐)
  - 격리 Nav2 기동 간헐 실패
  - `limo_scan_normalizer`·장애물 기억 노드가 `capstone_ws/src`에 없음
- 다음: Gazebo 폐루프 → CONTINUE_MAP Worker → 웹 실차 단계(SAFETY_WATCHDOG §4).

## 2026-11-20 오후 인계 (Watchdog·Gate 통합 검토, 실차 잠금 준비)

- **Gate·실차 launch 담당 에이전트에게**: `fleet_control/docs/proposals_20261120/README.md`. 원본 03은 결함(`'OK'` 비교)이므로 적용하지 말 것. 권장 순서 03b+04 → 격리 재시험 → 02b → 잠금 bringup.
- **실차 차단(P0)**: watchdog·03b 미배포, STM32 timeout·E-Stop·제동 거리 미측정, `base_footprint_z` 미실측, 바퀴 odom 중단 반복(11-18·11-19), localization CONFIRMED 생산자 없음.
- **P1**: 실모드 운영자 복구 경로 없음, 서버 단절·Lease 만료 허용 기준 미결정(fence 제안 §6.5), 격리 Nav2 기동 간헐 실패, `limo_ros2_application` 데모의 직접 `cmd_vel` 발행.
- **현장 적용 승인 대기**: 18190 기동 방식(현장 data 마이그레이션 vs B1 사본), C103 현장 등록, 실차 Agent 재배포. 현장 launcher `FLEET_SRC` 경로가 존재하지 않음.
- 오늘 잠금 시험 절차: `fleet_control/docs/FIELD_LOCKED_READINESS_20261120.md` §4.

## 2026-11-20 저녁 인계 (18190 백업 사본 운영)

- 18190 실행 중: 로컬 `fleet_control/` + B1 사본(C103 등록). 현장 Registry 미변경. 절차·명령·복구: `fleet_control/docs/FIELD_LOCKED_READINESS_20261120.md` §6.
- LIMO `pcu` offline → 실차 잠금 점검 미실행. 접속 후 `real_locked_probe.py`.
- (b) 현장 Registry 반영과 실차 주행은 사용자 승인 대기.

## 2026-11-20 공개 기준선과 후속 작업 감사

실차 웹 관제는 구동 차단 상태다. 최신 실차 Launch에 Watchdog이 편입되지 않았으며 STM32 명령 timeout, 물리 E-Stop, 제동 거리, Depth 높이 실측, wheel odometry 안정화, localization 확인, 실차 Operator Recovery가 남았다. LIMO 연결과 모터 잠금 웹 시험은 미확인 상태다.

현장 인계 기록상 원래 18190 런처가 해제된 워크트리의 소스를 참조한다. 별도 백업 Registry 사본에서 18190과 C103 표시를 확인한 기록은 현장 Registry 전환과 구분한다. Gate·실차 Launch 담당은 02b·03b·04를 미적용 상태로 관리한다. Fleet/Agent 담당은 CONTINUE_MAP Worker와 실차 복구를 맡는다.

2026-11-20 인계상 B1 사본으로 실행한 18190은 이번 감사에서 재시작하거나 현장 Registry·배포본과 교체하지 않았다. 현재 연결은 이 감사에서 재확인하지 못했다. Git 공개 반영은 공개 기준선에서 검토한 문서 패치로 준비했고, 신규 반영 브랜치 승인이 필요하다.

## 2026-11-20 밤 LIMO 연결 점검 (읽기 전용)

- pcu online·SSH 가능, 프로세스 0, USB 정상. **로봇에 Robot Agent 없음** → 웹 Online·센서 표시 불가. 모터 잠금 스택·Agent 설치는 승인 대기. 상세 `fleet_control/docs/FIELD_LOCKED_READINESS_20261120.md` §8.
- 같은 밤: 로봇에 `limo_robot_agent`(읽기 전용, 임무 거부) 설치·연결, 웹 Online·Lease·START/Goal 거부 확인. 스택·카메라는 대기. 백업 `local_data/robot_backups_20261120/`. 상세 `FIELD_LOCKED_READINESS_20261120.md` §9.
- 같은 밤(사용자 확인 후): 로봇 모터 잠금 스택+Agent 실행 중, 웹 Online·센서·Gate·LOCKED 표시 PASS. 첫 기동 중 베이스 UART 허브 재열거 1회(원인 미확정). 카메라 웹 표시는 영상 키트·AI host·`VIDEO_ORIGIN` 필요(승인 대기). 종료 절차 `FIELD_LOCKED_READINESS_20261120.md` §10.
- 같은 밤(승인 후): 영상 경로 구성 — 노트북 AI host(YOLO CUDA), 로봇 `~/limo_video_kit` 송신기, 18190 `VIDEO_ORIGIN`. 왕복 8.6 fps·오류 0. 브라우저 표시는 사용자 로그인 필요. 종료 절차 `FIELD_LOCKED_READINESS_20261120.md` §11.
- 현재 로봇 위치는 H107(C103 아님). C103 지도는 이 위치의 Localization·Goal 기준으로 사용 금지. H107 Registry 등록은 건물·층·영역 정보 확인 후.
- H107(H관 1F 강의실) 지도 저장·등록 완료(B1 사본 Registry, UNVERIFIED, 확인 셀 866개). 파생 YAML(free_thresh 0.25→0.196) 사유 `FIELD_LOCKED_READINESS_20261120.md` §13. 현장 Registry 미변경.
- 11:00 로봇이 09:52 종료·11:02 재부팅되어 스택·Agent·송신기 중지(재기동은 주변·정지 수단 재확인 후). 오프라인 로봇의 마지막 상태가 `OK age 0.05s`로 남는 UI 표시 결함 발견(제안만). 종료 스크립트 `fleet_control/scripts/stop_field_session.sh`(dry-run 기본). 웹 등록 기능 조사 `WEB_REGISTRY_AUTHORING_INVESTIGATION_20261120.md`.
- 12:40 18190 재기동(새 웹 UI·Registry API 반영). 같은 시각 로봇 LiDAR 스캔이 11:45부터 끊김(Gate FAILSAFE, 모터 잠금 유지). 스택 재기동 승인 대기. `FIELD_LOCKED_READINESS_20261120.md` §15.
- 12:45 로봇 모터 잠금 스택 재기동(LiDAR 복구, 웹 프로브 17/0). 종료 시 한 노드 SIGKILL·`limo_base` 크래시 발생. **배터리 11.0 V**(오전 12.3 V)로 하락, 충전 권장. `FIELD_LOCKED_READINESS_20261120.md` §16.
- 13:0x 정지 측정(충전 중): Depth 카메라 바닥 위 0.199 m, 아래로 약 5.4–6°, 현재 TF에는 기울기 없음 → 높이 필터 오차 약 0.107·d m. 보정 제안 `camera_pitch:=0.1038 camera_roll:=0.0140`, `base_footprint_z=0.1988−camera_z`. 전제(평평한 바닥·바퀴 접지)와 줄자 확인 필요, 미적용. `FIELD_LOCKED_READINESS_20261120.md` §17.
- 13:10 카메라 TF 보정 적용(현재 스택 기동 인자: `camera_pitch:=0.0502 camera_roll:=0.0140 base_footprint_z:=0.1683`). 바닥이 z −0.172→−0.001 m, 경사 3.0°→0.3°. 카메라 기울기는 만지면 변함(5.4°→2.3°) → 값을 박아 두지 말고 시작 시 보정 필요. 다음 기동은 기본값으로 돌아감. 낮은 물체 실물 시험은 대기. §18.

## 2026-11-20 저녁 Phase 1 웹 수동 조작 통합 (Claude, 주 구현 담당)

- **역할**: 사용자 지시로 Claude = 주 설계·구현, Codex = 독립 리뷰·회귀·안전 검증. 설계 기준 `fleet_control/docs/INTEGRATED_CONTROL_PLAN_20261120.md`(제안, DECISIONS 미반영).
- **완료(격리)**: 웹 패널 → Fleet HTTP → Agent → VelocityArbiter → Motion Watchdog → Gate(`/cmd_vel_locked`) 비영 속도 동작, 장애 시 0. E2E 18 시나리오 × 3회 54/54 PASS. 핵심 수정: 실제 action server는 idle 시 status를 발행하지 않아 기존 Watchdog·Arbiter·서버가 실제 Nav2 옆에서 수동을 영구 차단하던 결함 → 발행자 GID 연속성 증거로 교체. AUTO 임무 생성 시 MANUAL 거부, `control_mode`, 차단 사유 UI. [보고](fleet_control/docs/MANUAL_PHASE1_INTEGRATION_20261120.md).
- **Codex 검토 요청**: (1) Codex `test_manual_locked_chain_isolated_20261120.py` `active_goal` 모드의 "영구 재무장 금지" 기대와 새 규칙(Watchdog cancel-all 재증명 + 새 세션만 재개) 충돌, (2) 연속성 증거의 GID 경합, (3) Watchdog의 FollowPath 등 미감시.
- **미변경**: 실차 모터 잠금, real 벤치 0 출력, 실차 Mapping START 차단, 18190, LIMO, Gate 코드, 실차 launch. commit/push 없음.
- **다음**: Phase 2(웹 Mapping 속도 경로를 Arbiter로 통일, Gazebo), Phase 3(Nav2 smoother 출력 → Arbiter AUTO 통과, 실제 Nav2 스택으로 연속성 증거 재검증).

## 2026-11-20 밤 Phase 2 운용 모드 전환·웹 맵핑 통합 (Claude)

- **완료(격리 ROS + 2D 운동학 시뮬레이터)**: 서버 운용 모드(IDLE/MANUAL/MAPPING/NAVIGATION; 이전 작업 종료 → idle 검증 → Agent 확인 → ACTIVE, 실패 시 IDLE/FAULT, 자동 재시도 없음). Arbiter를 모드별 단일 속도 출처로 확장(Nav2 smoother → Arbiter). 웹 맵핑 RECONCILE(UNKNOWN 증거 기반 해제), 운용 모드 카드. E2E run01·02 각 16 PASS / 1 SKIP, f06 subset 8/8. [보고](fleet_control/docs/MODE_PHASE2_INTEGRATION_20261120.md).
- **함께 고친 결함**: 실제 Nav2 idle 무발행 때문에 서버 Safety Context·Agent movement 판정이 실행 중 임무를 2 s 뒤 무효화하던 문제(`status_continuous`). Gate STALL_PENDING에서 수동·임무가 끊기던 문제. 전환 중 취소 임무가 reconciliation으로 가던 문제.
- **미해결**: 로봇 단위 RECONCILIATION_REQUIRED를 admin recover가 `no_evidence`로 거부해 웹 복구가 불가하다. localization 생산자가 없어 NAVIGATION은 FIXTURE로만 검증했다. Gazebo + Arbiter는 미구현. Watchdog의 외부 action 감시 범위가 제한적이다.
- **Codex 검토 요청**: VERIFYING_IDLE 충분성, CONFIRMING 경합, STOPPING 중 NAVIGATION 유지 규칙, 잠금 mock continuation 예외(`safety.py`), RECONCILE 증거.
- **미변경**: 실차 모터 잠금·Mapping START 차단, 18190, LIMO, Gate 코드, 실차 launch. commit/push 없음.

## 2026-11-20 심야 Codex 결함 수정 + Phase 3 저장 지도 Navigation (Claude)

- **수정(같은 스냅샷 재시험)**: Codex boundary 2/8→8/8, Watchdog GID 0/2→2/2. Phase 1 18/18, Phase 2 17/17(SKIP 0, Phase 3 지도 계약에 맞춘 하네스), Phase 3 17/17. [보고](fleet_control/docs/CODEX_FIX_AND_PHASE3_20261120.md).
- **Phase 3 구현(격리)**: Registry 지도 선택·무결성 → Agent LocalizationWorker(map_server + AMCL, allow_unknown=false) → 웹 초기 위치 → CONFIRMED(AMCL + 스캔/지도 일치) → 서버 goal 검사(밖/벽/미지/반경) → Nav2. 웹 카드 `navigation_panel.js`. Phase 2 로봇 단위 복구를 Nav2 terminal 증거 기반으로 구현했다(실차는 거부 유지).
- **실차 No-Go 추가 항목**: Gate 입력 `/cmd_vel_raw` 독점 없음(rogue 발행 시 Gate 출력 발생, Gate 담당), 외부 FollowPath 등 미감시, 지도 VERIFIED 웹 정책 없음, Gazebo + Arbiter 미연결.
- **Codex 재검증 요청**: 실제 DDS 지연·발행자 교체 경합, CONFIRMING–재시작 경합, recovery 증거 위조 가능성, localization 기준.

## 2026-11-20 Safety Gate 입력 독점 (Claude, 격리 시험만)

- **원인**: Gate가 `/cmd_vel_raw`의 모든 메시지를 발행자 확인 없이 사용했다. Humble rclpy에는 메시지별 송신 GID가 없다(MessageInfo는 시각만 담음, 실측). NaN은 `abs(NaN) > x`가 False이므로 그대로 출력되었다.
- **수정(후보)**: `experiments/safety_chain_fix_20261113/candidate_gate_r11/` = r10 + 입력 출처 검증. 단일 허용 노드·판독 가능한 GID·결속 GID 불변이 아니면 FAILSAFE이며, NOT_EXCLUSIVE·UNEXPECTED·CHANGED는 operator reset까지 유지된다. 거부된 운동 명령이 있으면 무장하지 않는다. NaN/Inf·범위 밖 값은 0이다. r10은 그대로 두었다. field·`capstone_ws` Gate는 **미변경**이며, 제안 패치 05만 있다(미적용, `git apply --check` 통과, 패치 사본으로 단독 10종 시험 통과).
- **결과**: 우회 프로브(MANUAL s18/g01, MAPPING g02, NAVIGATION g03/f07b)에서 field/r10은 매 실행 rogue 값이 출력되었고, r11은 매 실행 0이었다. 원래 Phase 2 17/17, Phase 3 18/18, Phase 1(브라우저 없음) 18/18. [보고](fleet_control/docs/GATE_INPUT_EXCLUSIVITY_20261120.md), [Gate 보고](experiments/safety_chain_fix_20261113/docs/GATE_R11_INPUT_SOURCE_20261120.md)
- **미해결**: 인증 아님(단독 위조 발행자는 수락됨, SROS2 필요). Gate reset 웹 경로 없음(Watchdog 재시작 때마다 필요). SAVE 중 Agent heartbeat 여유 부족 1회. Phase 1 s10 latency None(r11, 전체 브라우저 순서 2회, 원인 미상). g-프로브를 넣은 Phase 2는 순서 의존 변동(양쪽 Gate). Gazebo SKIP(연결 지점은 보고서 §6).
- **미변경**: LIMO, 18190, 실차 모터 잠금·Mapping START 차단, 실차 launch, 서버·Agent 코드. commit/push 없음.

## 2026-11-21 관리자 전용 웹 Gate Reset (Claude, 격리 시험만)

- **구현**:
  - 서버 `gate_reset.py`(관리자 강제·감사·단일 요청·boot 바인딩·1회 전달·재생 거부)
  - Agent `gate_reset_worker.py`(사전검사·1.5 s 관찰·영구 journal)
  - 기존 `/stall_detector/reset_latch`
  - **Gate 후보 r12**: Gate가 직접 Reset을 판단. 입력 발행자 1개·발행 중·Watchdog과 같은 participant, 운동 0, 장애 해소
  - 웹 카드(운영 탭)
  - Reset 후 IDLE 유지, 자동 재개 없음
- **결과**:
  - 격리 E2E 13/13 ×2: 정상 복구, 장애 지속, Watchdog 재시작 유령, Publisher 교체, 동시 Reset, 서버·Agent 재시작, 재생, 자동 재개 없음, 브라우저
  - 단위 14+14+11+panel
  - Phase 1(r12, 브라우저) 19/19 ×2, s10 4/4
  - [보고](fleet_control/docs/GATE_RESET_WEB_20261121.md)
- **시험 중 찾아 고친 결함**:
  - 거부 감사 행이 트랜잭션 롤백으로 사라짐
  - SIGKILL 직후 유령 endpoint로 Reset 수락
  - Gate가 사칭 단독 발행자를 ready로 판단
  - 보고 지연 중 같은 fault_epoch 재요청 수락
  - 하네스 판정 필드 덮어쓰기
- **미해결**:
  - field·`capstone_ws` Gate 미적용(소유권 확인 필요), 인증 아님(SROS2)
  - Phase 3 r12/r11 비교 미판정(환경 변동, §4.4)
  - Reset 전 1 s 0 명령 운영 절차 필요
  - Gazebo·실차 보류
- **미변경**: LIMO, 18190, 실차 모터 잠금·Mapping START 차단, 실차 launch. commit/push 없음.

## 2026-11-21 Phase 3 불안정성 분석·Gate r12 통합 준비 (Claude, 격리만)

- **원인 분리**(timeline 기록기 + 서버 최초 실패 사유):
  - A·B: 매 실행 다른 지도·시작 자세 → 고정 지도 모드
  - C: 재LOAD 시 AMCL이 초기 위치를 놓침 → Localization Worker 수정
  - D: Gate boot 변경 플래그가 영구히 지워지지 않아 임무 영구 차단 → `/recover` 증거 기반 재조정
  - 고정 지도 기동 교착(planner 대기 ↔ LOAD 실패) → CONFIRMED에만 `allow_unknown=false` 요구
  - E: Gate 프로세스 일시 정지 → 센서 FAILSAFE 래치. r10·r11·r12 공통, 원인 콜백 미확인
- **r12 회귀 판정**: 증거 없음. 고정 지도·같은 스냅샷 교대 8회에서 r11 17/1·18/0·18/0·18/0, r12 18/0 ×4. r11의 1건은 E.
- **회귀**: Phase 1 19/19 ×2 (r11·r12 각각), Phase 2 원래 17 PASS (r11·r12), Gate Reset 13/13, r01 Gate boot 재조정 PASS (r11·r12), 단위 전부. s10은 하네스 측정 결함(첫 샘플 전 정지)으로 확인·수정.
- **통합**: 제안 06(05 + Reset 판단, 기본 꺼짐, `git apply --check` 통과) 담당자 검토 대기. 정식 Gate·실차·Gazebo 미적용. [보고](fleet_control/docs/PHASE3_STABILITY_AND_GATE_R12_20261121.md)
- **미변경**: LIMO, 18190, 실차 모터 잠금·Mapping START 차단, 다른 에이전트 Gate diff(바이트 동일 확인). commit/push 없음.

## 2026-11-21 Gate 정지(E) 분석 · Gazebo 웹 제어 통합 · Phase 3.5 리팩토링 (Claude, 격리만)

- **E**:
  - 원인 미확인. 계측 없는 Phase 2에서는 1회 재현했다(MAPPING 시작 2 s 후).
  - 추적 계측(콜백·발행·OS 지연 + 외부 scan 증인) 7회에서는 0회였다. 프로브를 끈 2회에서도 0회였다.
  - 이전 증거는 그대로다: 외부 기록기는 scan을 받았고 Gate만 멈췄다. 따라서 Gate 프로세스 내부 문제로 본다.
- **정지 중 안전 영향**(주입 24 case + Gazebo):
  - Gate가 멈추면 0 명령도 나가지 않는다. 그동안의 이동은 하부(hold/timeout)가 결정한다.
  - 0.38 s 미만 정지는 흔적 없이 지나간다.
  - Gazebo(DiffDrive hold)에서는 0.35 s 정지 중 0.042 rad 회전했다.
  - STM32 timeout은 미측정이다.
- **Gazebo**:
  - `stack_up.sh SIM_BACKEND=gazebo`(기본값 불변). Office 월드 + `limo_eval_diff`.
  - Web→Server→Agent(Arbiter)→Watchdog→Gate r12→`/cmd_vel_locked`→bridge. ROS `/cmd_vel` 엔드포인트 0개.
  - 웹 MANUAL·keyboard/gamepad 정지, 웹 MAPPING(2/6), LOAD+CONFIRMED, Nav2 목표 SUCCEEDED(1회), 장애 정지 지연을 측정했다.
  - Gazebo 휠 odom이 회전을 +64 % 과대 보고하므로 relay는 ICP odom을 쓴다.
  - 발견: Frontier 교착, pointer 입력 간헐 무반응, LiDAR에 안 보이는 물체에 의한 물리 정지(Gate STALL 래치는 참 양성).
- **Phase 3.5**:
  - 토픽 상수·GID 헬퍼 통일(동작 불변).
  - 잠금 그래프 판정 특성화 시험.
  - Reset Worker fallback을 fail-closed로 변경(의도된 동작 변경).
  - `tests/e2e_common.py` 추가.
- **회귀**: 단위 90/21(실패 목록 불변), Phase 1 18/1 → s16 창 수정 후 19/0, Phase 2 16/1(E), Phase 3 18/0, Gate Reset 13/0.
- **보고**: [E·Gazebo](fleet_control/docs/GATE_STALL_GAZEBO_20261121.md), [리팩토링](fleet_control/docs/REFACTOR_PHASE35_20261121.md).
- **미변경**: LIMO, 18190, 실차 모터 잠금·Mapping START 차단, r11/r12/패치 06 원본, 다른 담당자 `capstone_ws` Gate diff. commit/push 없음.

## 2026-11-21 Gazebo 웹 관제 안정화 · Gate 독립 하위 정지 (Claude, 격리만)

- **하위 구동 보호**:
  - Ignition DiffDrive가 마지막 명령을 무기한 유지함을 확인했다(침묵 3 s에 0.30 m).
  - Gate와 독립인 하위 timeout 프로세스를 추가했다: `tests/gazebo_web/drive_timeout_relay.py`, `GZ_DRIVE_TIMEOUT_S`(기본 0.2).
  - Gate SIGSTOP 0.35/0.5/2 s, SIGKILL, Gate+Watchdog SIGSTOP에서 약 0.2 s에 하위 0, 0.22~0.26 s에 정지, 이동 0.020~0.024 m(2회 반복). timeout이 없으면 0.04~0.20 m를 움직이고, SIGKILL이면 멈추지 않는다.
  - 실차 STM32 timeout은 미측정이다.
- **Gate r12 발견**(수정 안 함): 긴 정지에서 재개된 첫 tick에 오래된 명령을 1회 출력한 뒤 래치된다. 복도에서는 ICP 축퇴로 STALL 거짓 양성과 ICP_DEGRADED가 난다.
- **포인터**:
  - 손실 지점은 브라우저 패널 `tick()`이다. 증거 나이 > 1000 ms인데 대시보드 갱신이 1 s라 생긴다. HTTP·Agent·Arbiter에서는 손실이 없었다.
  - 수정: 세션 중 300 ms 갱신, 시작 전 갱신. 한계는 그대로다. 45/45 주기 확인.
- **Frontier**:
  - `frontier_policy.py`: 근접 후보 건너뛰기, 방문·실패 처리, REPOSITION, EXHAUSTED/GOALS_FAILING 보고.
  - 수정 후 탐색 8/8(수정 전 4/6 정체). 근접 후보 건너뛰기가 기록으로 확인됐다.
- **지도 클릭 E2E**(실제 대시보드):
  - 좌표 7/7.
  - 지도 밖·장애물·미관측 7/7 차단, Localization 미확인 4/4 서버 거부.
  - 정상 목표 완료 1회(오차 0.055 m). 나머지는 Agent 안전 취소(Gate 래치 등).
- **회귀**: Phase 1 19/0, Phase 2 17/0, Phase 3 18/0, Gate Reset 13/0, 단위 91/21(실패 목록 불변). E 외부 관찰에서 미재현.
- **보고**: [안정화](fleet_control/docs/GAZEBO_STABILIZATION_20261121.md) §11에 자동 기동·방문객 웹 인수인계가 있다.
- **미변경**: LIMO, 18190, 실차 모터 잠금·Mapping START 차단, Gate 후보, Frontier C++, `rmf_adapter/`. commit/push 없음.

## 2026-11-21 Phase 4 자동 기동 · Phase 5 방문객 안내 웹 (Claude, 격리만)

- **Phase 4**:
  - 로봇 측 `limo_robot_agent.supervisor`: 로컬 config의 서비스만 순서대로 기동하고 READY를 판정한다. 재시작 상한을 넘으면 FAILED가 되며, Gate stack 재시작 시 Watchdog과 Agent도 재시작한다(`restart_with`).
  - 서버 `readiness.py`: READY/DEGRADED/FAULT와 사유를 계산한다. `--require-supervisor-ready`이면 MAPPING/NAVIGATION/임무는 READY일 때만 허용한다.
  - 관리자 웹 서비스 작업은 이름 + start/stop/restart/retry만 가능하다(셸 없음).
  - 웹 카드를 추가했다. systemd·config 템플릿은 미설치이고 실차 미검증이다.
- **Phase 5**:
  - `guide.py`: 방문객 링크(별도 `Visitor` 인증), 지도 버전·해시에 묶인 목적지, VERIFIED 지도만 안내, 요청 → 운영자 Lease로 Fleet 임무 생성, 대기열·취소.
  - 모바일 `visitor.html`, 운영자 안내 카드, QR(디코더 미검증)을 추가했다.
- **목표 검사 통일**: 대시보드 지도 클릭은 서버 `goal-check`(0.20 m)가 결정한다. 화면 0.32 m 검사는 참고로만 남겼다.
- **결과**:
  - 격리 ROS: Phase 4 8/8 ×2(서버 먼저 / 감독 먼저+센서 지연), Phase 5 8/8.
  - 단위 96/21(실패 목록 불변).
  - 회귀: Phase 1 19/0, Phase 2 16/0/SKIP 1(기존 간헐), Phase 3 18/0, Gate Reset 13/0. Gazebo·실차는 미실행.
- **발견**: Nav2 기동 실패를 READY로 오인하던 문제 → `NAV2:*` 사유. Watchdog 재시작 후 Gate 래치는 관리자 Reset으로 복구.
- **보고**: [Phase 4·5](fleet_control/docs/PHASE4_5_AUTOSTART_VISITOR_20261121.md)
- **미변경**: LIMO, 18190, 실차 자동 기동·모터 잠금·Mapping START 차단, Gate 후보, Frontier C++, `rmf_adapter/`. commit/push 없음.

## 2026-11-21 Codex Phase 4·5 독립 검증 결함 수정 (Claude, 격리만)

- **수정 파일**: `fleet_server/{readiness,guide,mapping,operating_mode,__main__}.py`, `static/{system_panel,visitor}.js`, `scripts/run_field_server.sh`(`--require-supervisor-ready`), 시험 하네스 `tests/{phase4,phase5,gazebo_web}/run_*.py`, `tests/test_phase4_readiness.py`, `tests/test_system_panel.cjs`, 신규 `tests/test_phase45_fix_20261121.py`.
- **API 변경 (운영자 웹 외 호출자 주의)**: `POST /robots/{id}/services`는 body에 관리자 보유 Lease `epoch`(int) 필수 (없으면 400 `bad_epoch`, 불일치 409 `lease_required`). `POST /guide/service` `enabled`는 bool만 (아니면 400). readiness 응답에 `motion_permitted`, `motion_block_reasons`, `supervisor_required`, `required_services`, `required_topics` 추가.
- **운영 서버**: 실행 중 18190은 변경·재시작하지 않았다. `run_field_server.sh`의 새 플래그는 다음 기동부터 적용된다.
- **Codex에 남긴 제안**: `test_different_link_cannot_view_or_cancel_when_session_id_known`의 print 줄을 `assertRaises`로 감싸기 (현재 요구사항대로 404를 받아 ERROR).
- **후속**: kinematic sim `/scan` 간헐 중단(Phase 4·5 간헐 실패 원인), Gazebo g03 탐사 무이동, Gate "재개 직후 오래된 명령"(별도 항목 유지).
- **보고**: [CODEX_FIX_PHASE45](fleet_control/docs/CODEX_FIX_PHASE45_20261121.md). commit/push 없음.

## 2026-11-22 Gazebo 통합 실패 수정 · Phase 4·5 안정화 (Claude, 격리만)
- **제품 변경 (Agent)**: `ros_adapter.py` heartbeat/task 타이머를 STEADY clock으로, `velocity_arbiter.py` `nav_blocker`가 `active.cancel_requested`이면 `CANCEL_REQUESTED`(Nav2 출력 0). 원본 `local_data/raw_results/gazebo_integ_claude_20261121/pre_change_originals/`.
- **시험 하네스**: `tests/gazebo_web/run_gazebo_web.py` 복귀 주행(`home`, `open_heading`, `HOME_BEFORE`에 g04 포함), 진실값→지도 좌표 변환(`T2M`), d06 마지막. 신규 `timeline_recorder.py`/`timeline_analyse.py`, `tests/gate_resume_stale_cmd_isolated.py`, `tests/mode_e2e/dds_restart_probe.py`.
- **Gate 담당자에게**: `gate_candidate_r12_resume/resume_hold.patch` 제안 (r12 재개 직후 1–2 tick 이전 명령 재출력 해소). 운영 Gate·capstone_ws·r12 미적용.
- **Nav2 설정 담당자에게 (제안만)**: `integrated_nav2_lidar_only_slow_params.yaml` RPP `use_collision_detection: false` — Gazebo g08에서 LiDAR로 보이는 장애물로 직진(가설, costmap 미수집).
- **후속**: f03 Watchdog SIGKILL 후 DDS 유령 writer, ICP 회전 과소 추정에 따른 지도 왜곡·AMCL 추적 상실, LiDAR 평면 아래 장애물. commit/push 없음. [보고](fleet_control/docs/GAZEBO_INTEG_FIX_20261122.md)

## 2026-11-22 (2) 속도 재출력 안전 · Gazebo 잔여 실패 (Claude, 격리만)
- **제품 변경 (Agent)**: `velocity_arbiter.py` 제어 세대·`NAV_GEN_SETTLE_S`·`MANUAL_DEADMAN_NOT_REARMED`, `motion_watchdog.py` 자기 정지 후 입력 폐기. 원본 `local_data/raw_results/gazebo_p0_claude_20261122/pre_change/`.
- **하네스**: `run_gazebo_web.py` f03이 새 writer 재결합까지 대기, `timeline_recorder.py` amcl/icp, `stack.launch.py` env 게이트 `GATE_WRITER_STATUS`·`NAV2_PARAMS_OVERLAY`, `gate_resume_stale_cmd_isolated.py` swap.
- **Gate 담당자에게 (제안)**: `local_data/raw_results/gazebo_p0_claude_20261122/gate_candidate_r13/r13_vs_r12.patch` (resume_hold + writer 상태 교차 확인). 운영 Gate·capstone_ws·r12 미적용.
- **ICP/위치 추정 담당자에게 (제안)**: `lidar_scan_matcher_odom_node` 회전 약 7 % 과소 → g05 AMCL DEGRADED. 측정 `local_data/raw_results/gazebo_p0_claude_20261122/amcl_vs_truth.py`.
- **Nav2/costmap 담당자에게 (제안)**: LiDAR 평면 밖 장애물(의자·책상)에 depth 층 또는 keep-out 필요. `nav2_overlay_rpp_collision.yaml`은 미시험.
- **Codex에게**: `test_public_manual_arbiter_locked_ros`의 `ManualWebBench` 고정 hash가 기준선부터 오래됨 (수정 안 함).
- commit/push/worktree 없음. [보고](fleet_control/docs/GAZEBO_INTEG_FIX_20261122.md) Part 2

## 2026-11-22 (3) Gate r13 통합 준비 · 속도 세대 계약 (Claude, 격리만)
- **제품 변경 (Agent, mock/sim Arbiter)**: `velocity_arbiter.py` producer liveness (`NAV_RAW_MAX_AGE_S` 0.25, `NAV_PRODUCER_NOT_LIVE`), `topics.py` `NAV_RAW`. 원본 `local_data/raw_results/gate_r13_integ_claude_20261122/pre_change/`. Watchdog 변경 없음.
- **시험 도구**: `fleet_control/tests/run_guard.py` (실행별 새 디렉터리·hash manifest), `nav2_smoother_tail_probe_20261122.py`; `GATE_SCRIPT` env 선택: gate_reset_e2e·phase4·phase5, `GATE_WRITER_STATUS`: manual_e2e (기본 r12 그대로).
- **Gate 담당자에게**: r13 후보 `candidate_r13/` (sha f0756915…) 통합 검토 요청. 정식 Gate(capstone_ws, 담당자 미커밋 8줄) 미수정·미인계.
- **Nav2 담당자에게**: D3 원인 — RPP `use_rotate_to_heading: false`로 뒤쪽 goal에 전진 호 주행, 장애물 0.16–0.20 m 정지 → 계획 실패·Collision Ahead. 후보·시험 기준은 [결함 인계](fleet_control/docs/GAZEBO_OPEN_DEFECTS_20261122.md) (미시험).
- **Phase 2 하네스 담당자에게**: `gate_guard`가 시험 사이에만 동작해 시험 도중 sim scan 끊김이 f04 실패로 기록됨 (수정 안 함).
- commit/push/worktree 없음. [보고](fleet_control/docs/GAZEBO_INTEG_FIX_20261122.md) Part 3

## 2026-11-22 (4) 실차 통합 조사 · Gazebo LiDAR+Depth 기준 정정 (Claude, 격리만)
- **Depth 프로파일 담당자에게**: `integrated_nav2_depth_slow_params.yaml`의 `depth_obstacles`를 `obstacle_layer`에서 별도 `depth_layer`(ObstacleLayer, inflation 앞)로 분리 제안. 공유 시 LiDAR raytrace가 낮은 장애물 표시를 지움 (Gazebo FAIL 3건 → 분리 후 0건). 시험 overlay `local_data/raw_results/gazebo_depth_claude_20261122/c_depth_sep/run/nav2_overlay.yaml`. 미적용. 이동 장애물 잔상은 실차 확인 필요.
- **Gazebo 하네스 담당자에게**: `tests/gazebo_web`·`web_mapping_isolated` 스택에는 depth 경로가 없음. 재사용 가능한 launch `fleet_control/tests/gazebo_depth/depth_sensors.launch.py`와 Gate depth_mode wrapper `gate_depth_mode.py` (공유 launch 미수정).
- **Gate 담당자에게**: LiDAR 단독 직선 통로에서 r13 STALL 오정지 3회 (`lidar_speed` 0.001–0.002, 실제 0.236 m 이동). 원인 가정(ICP 축 퇴화, D1 계열), 미확정.
- **실차 다음 단계 (승인 필요)**: pcu online + 현장 안전 확인 후 읽기 전용 조사 → `env_local77.sh`·격리 포트·`allow_motor_drive:=false` 잠금 시험 → 정지 Depth 의자 받침 시험. [조사 보고](fleet_control/docs/REAL_LIMO_INTEGRATION_SURVEY_20261122.md) §5·§6·§9.
- commit/push/worktree 없음. [Depth 보고](fleet_control/docs/GAZEBO_DEPTH_20261122.md)

## 2026-11-22 (5) 실차 통합 전환 준비 (Claude)
- 로봇 online 후 승인 순서: (1) 현장 안전(L4) (2) 격리 서버 포트·데이터 사본 (3) 로봇 패키지 tar 백업 후 Agent 배포 후보 설치 (4) Depth 드라이버 정지 시험. 절차·판정: [조사 보고](fleet_control/docs/REAL_LIMO_INTEGRATION_SURVEY_20261122.md) §10.4–10.5.
- **Codex 검증 추가 대상**: real 모드 `manual_web_bench`가 Gate HEAD에 두 번째 `/cmd_vel_raw` writer가 되는 문제, `test_agent_safety_v1` 단독 실행 fixture 오류.


## 2026-11-22 (6) Nav2 D3 · D1/D2 (Claude 보조 세션, 격리만)
- **Nav2 담당자에게**: O6 overlay `local_data/raw_results/nav2_d3_claude_20261122/overlays/nav2_overlay_d3_o6_rotate_poseprogress.yaml` 검토 요청(공유 프로파일 미수정). 미해결: 좁은 복도 U턴, robot_radius 0.18 vs 실차 약 0.195, lifecycle change_state 응답 유실(부하 시).
- **ICP 담당자에게**: keyframe 임계값·trim 백분위를 파라미터로; 후보 keyframe 0.30 + trim 없음. O6는 제자리 회전을 늘리므로 D1과 함께 적용 권장.
- **Codex에게**: Gate r13 격리 결과·정식 적용 판단(Part 3 Q4) 독립 검증. g07 수동 포인터 `gate_nonzero_after_up` 2(부하 ~9) 확인. Arbiter는 mock/sim 전용.
- 하네스: `run_gazebo_web.py` gate_guard 지속 결함 재시도 상한 + failure_classes. commit/push/worktree 없음. [보고](fleet_control/docs/GAZEBO_INTEG_FIX_20261122.md) Part 4

## 2026-11-22 (7) 실차 Fleet 통합안 (Claude)
- [계획](fleet_control/docs/REAL_LIMO_FLEET_INTEGRATION_PLAN_20261122.md): 재사용/신규/차단 표, Agent 후보 변경 범위·호환성, Stage A(잠금 웹 통합)·Stage B(Arbiter·Watchdog 실차 편입) 최소안, L1–L5 잠금 검증, 격리 서버 시험 순서, 변경 대상 파일.
- **Gate·launch·Depth 프로파일·ICP 담당자**: 제안만, 파일 미수정. **Codex**: plan §8 (real 모드 기동 가드, 계약 호환, L1–L5 충분성).
- **승인 필요 순서**: 현장 안전(L4) → 격리 서버 포트·토큰 → Agent 후보 설치(백업 후) → Depth 드라이버 정지 시험.

## 2026-11-22 (8) Agent 배포 전 검증 (Claude)
- **Codex**: real 모드 `web_gate_reset` 명시 가드 부재(`agent_node.py`) 검토·판단, `manual_web_bench` 둘째 `/cmd_vel_raw` 발행자(Gate 그래프 포함 시험), Agent 종료 시 heartbeat RCLError 정리.
- **승인 필요(실제 배포)**: plan §11.2 (현장 안전 L4, 로봇 백업 생성, Agent 패키지 교체, Tailscale 바인드와 토큰 복사, 잠금 스택 기동). 롤백: `isolated_server_prep_01/ROLLBACK_CHECKLIST.md`. commit/push 없음.

## 2026-11-22 (9) 실차 Agent 배포 직전 (Claude)
- 승인 순서와 롤백: [plan §13](fleet_control/docs/REAL_LIMO_FLEET_INTEGRATION_PLAN_20261122.md) (현장 안전 L4 → 노트북 패킷 필터(root) 또는 tailnet ACL → 로봇 백업 → v2 교체 → 토큰 복사 → 서버 Tailscale 바인드 → 잠금 스택).
- **Codex**: §12.3 호출 경로 표, real `allow_missions:=true` 노드 수락 여부, Gate HEAD 필드 부재 시 fail-closed 확인. commit/push 없음.

## 2026-11-22 (10) Agent v3 배포 직전 (Claude)
- 배포 후보는 **v3**(`deploy_candidate_agent_v3/`). 승인 순서: plan §14.8 (현장 안전 → ufw 규칙/tailnet ACL → 로봇 백업 → v3 교체 → 토큰 → 서버 Tailscale 바인드 + Agent 단독 기동; 전체 ROS 스택은 별도 승인). 규칙안: `isolated_server_prep_01/NETWORK_SCOPE_18290_PROPOSED.md`.
- **Codex**: §14.1 우회 경로 B1–B6, fixture 기반 3개 시험의 harness 정비, real 서버측 MANUAL 모드 거부(올바른 mode 이름으로) 확인. commit/push 없음.

## 2026-11-22 (11) Stage A 진행 상태 (Claude)
- 로봇 Agent는 v3로 교체·빌드됨(미기동). 원복: plan §15 + `ROLLBACK_CHECKLIST.md` (백업 `~/backups/limo_robot_agent_*_pre_20261122_200059*`, 노트북 `local_data/robot_backups_20261122/`).
- 다음: 네트워크 단계 선택(A/B/C, plan §15.1) → 토큰 설치 → 서버 기동 → Agent 단독 기동 → ONLINE/Lease/ACK/재연결/거부 확인. 전체 ROS 스택은 별도 승인.

## 2026-11-22 (12) Stage A 후속 (Claude)
- **Agent 소유(우리)**: 실차 Agent 종료 루프 결함(SIGINT/SIGTERM 무시, `RCLError` 재시도) 수정 + 시험. 11-20 Agent에서도 재현되는지는 미확인.
- **사용자 결정**: `~/limo_isolated_18290_tools` 표준 위치 통합, 로봇의 백업·`~/stage_a_v3` 보존 기간. 다음 승인 대상: 잠금 ROS 스택 + 같은 터널(plan §16.4). commit/push 없음.

## 2026-11-22 (13) Agent v4 후보 · LIMO 표준 구조 제안 (Claude)
- **승인 대기**: ① v4 실차 배포(백업 후, §16 절차와 같음) ② LIMO 문서 등록(CODE_INDEX/WORKTREES에 Stage A 경로 3개 등록, `~/limo_field_agent_ws` 오기 정정) ③ 이후 선택적 통합(`~/Capstone1_limo/{backups,tools,staging}`).
- **사용자/소유자 확인**: 11-13 실차 run(~4.1 GB)이 로봇에만 있을 수 있음 → 해시 비교 후 노트북 복사 전까지 삭제 금지. `.bashrc`의 `stop` alias는 `/cmd_vel` 발행.
- **Codex**: 종료 수정(`agent_node.py`)과 matrix 독립 검토, LIMO 작업 규칙 초안(audit §7) 검토. commit/push 없음.

## 2026-11-22 (14) 리팩토링 설계 승인 대기 항목 (Claude)
- **사용자 결정**: ① `fleet_control/` 버전관리 브랜치(설계 §3.4; 소유자 미커밋 파일은 포함 금지) ② 11-13·11-18 로봇 run을 해시 검증하며 노트북에 복사(로봇 원본 유지) ③ Stage A 경로 3개 등록·`~/limo_field_agent_ws` 오기 정정 ④ `.bashrc` alias(`stop`=/cmd_vel 발행 등) 정리 ⑤ v4 배포.
- **소유자**: Gate/launch/Nav2/Depth/ICP는 R4(설계 §4) 결정 대기; 공통 문서(START_HERE/AGENTS/CODE_INDEX/WORKTREES/ARCHITECTURE) 변경 제안은 설계 §7에만 보존(AGENTS.md는 다른 변경이 진행 중이라 미수정). commit/push 없음.

## 2026-11-22 (15) R0 승인 대기 (Claude)
- **사용자 승인 필요**: ① 로봇 원본 보호 복사(`local_data/robot_originals_20261122/`, 해시·bag 검증, 약 7.9 GB, 로봇은 읽기만) ② git G1 스냅샷(신규 브랜치, noreply 신원) 또는 파일 스냅샷 ③ 검토 플래그(시작 스크립트의 tailnet IP 기본값, 스크린샷 77개, 절대경로 문서 16개) ④ G2용 새 워크트리.
- 금지 유지: `git clean`/`reset --hard`/체크아웃/브랜치 전환, 소유자 미커밋 파일(Gate, launch, depth filter, package.xml, run_recorder, AGENTS.md) 수정. commit/push 없음.

## 2026-11-22 (16) R0 다음 승인 (Claude)
- **사용자**: ① 두 번째 보관 위치(외장 디스크 또는 VALIDATION §2.5 암호화 Drive, 암호는 사용자 입력) ② G1 전 결정: `live-api.jpg` 보류(제외/블러), tailnet IP 문서 4개+복사 도구 치환 여부, `start_robot_agent_field.sh` 기본 URL을 필수 인자로 변경 ③ noreply 신원 사용 승인 후 G1 ④ `20261119_run_002` bag 살리기(스크래치 복사본에서만) 여부 ⑤ `WORKTREES.md`에 `robot_originals_*`·`fleet_control_snapshot_*` "삭제 금지" 등록.
- 금지 유지: `git clean`/`reset --hard`/체크아웃/stash/브랜치 이동/commit/push/PR, 로봇 원본 이동·삭제·수정. 이 단계의 스냅샷 이후 문서 수정분은 `..._02` 스냅샷으로 따로 보존 예정.

## 2026-11-22 (17) G1 승인 순서 (Claude)
- ① 패치 P1–P5를 실제 파일에 적용(우리 파일, 8개 해시 변경; 적용 후 `g1_rehearse.py` 재실행으로 tree id 재확정) ② 부모 HEAD·브랜치 `r0/fleet-control-snapshot-20261122`·noreply 신원으로 `g1_execute.sh` 실행 승인 ③ 선택: G1b(공통 문서 8개, `AGENTS.md` 제외), `STATE_DIR` 필수화, 로봇의 도구 사본 교체(로봇 변경, 별도 승인) ④ 두 번째 보관 위치. push/PR은 계획에 없음.
- 공통 문서(WORKTREES/CODE_INDEX/AGENTS/START_HERE) 제안은 `COMMON_DOCS_PROPOSAL_R0_20261122.md` 그대로; `AGENTS.md`는 다른 쪽 미커밋 21줄이 남아 있어 계속 제안만. 다른 세션 3개 모두 idle, 충돌 없음.

## 2026-11-22 (18) G1 이후 (Claude)
- **G2 차단 조건(반드시 확인)**: unified(`ed53d81b`, `419e7987`는 원격에 없음)를 새 브랜치나 main에 그대로 병합·cherry-pick·rebase 하지 말 것 — 추적 제외한 `live-api.jpg`가 이력에 들어온다. 병합 전 `git merge-base --is-ancestor ed53d81b <후보>`가 "no"이고 `git rev-list --objects <후보> | grep live-api.jpg`가 비어야 한다. 허용 경로: `origin/main`만 병합, unified에서 경로 단위로 가져오기(이미지 제외), 또는 로컬 커밋 2개 이력 재작성(명시 승인·bundle 백업 후).
- **사용 변경(P3·P2)**: `start_robot_agent_field.sh`는 이제 `SERVER_URL`과 `TOKEN_FILE`을 반드시 인자로 받는다(11-20 현장 절차 문서는 인자를 직접 쓰도록 읽을 것; 로봇의 사본은 옛 버전 그대로). `robot_originals_copy_20261122.py`는 `--host` 필수.
- **미수행**: push·PR, G2 worktree, G1b(공통 문서 8개), `STATE_DIR` 필수화, 로봇 도구 사본 교체, 두 번째 보관 위치, `WORKTREES.md`에 새 브랜치 등록(제안만). 스냅샷·임시 폴더(scratchpad의 `g1clone.*`, `g1test.*` 등)는 삭제하지 않음.

## 2026-11-22 (19) Agent v4 배포 승인 항목 (Claude)
- ① 배포 창·현장 안전 ② 로봇에 `~/stage_a_v4`와 v3 새 백업 생성 + 노트북 복사·해시 대조 ③ `limo_robot_agent` v4 교체·재빌드 ④ 격리 실행(루프백 서버, SSH 역터널, 토큰 0600, Agent SIGINT 종료 실측) ⑤ 선택: 로봇의 `start_robot_agent_field.sh` 사본 교체. 원복: 같은 stamp로 `rollback`.
- 이후 R1(공용 계약)·R3(manifest 기반 배포)로 진행; R2는 Watchdog·mock·supervisor에 같은 종료 matrix 확장 필요. G1 브랜치·HEAD·실차 원본·다른 에이전트 파일 불변, commit/push 없음.

## 2026-11-22 (20) v4 실차 검증 이후 (Claude)
- v4는 로봇에 설치·검증 완료(미기동 상태). 남은 선택: 로봇의 시작 스크립트 사본 교체(인자 필수 버전), Stage A SIGKILL 잔재 shm 2개 정리 여부, 잠금 ROS 스택과 함께 Agent v4 검증(별도 승인), R1~R3 리팩토링 착수. commit/push/PR 없음, 스냅샷·임시 폴더 삭제 없음.

## 2026-11-22 (21) 다음 승인 대기 (Claude)
1. "30 s" 조건 정의(T_agent/T_launch) 확정. 2. P6 저장소 적용 여부 + 로봇 시작 스크립트 교체(별도 승인, 절차는 문서 §2). 3. R1.1 계약 테스트 추가(신규 테스트만), R1.2 상수화는 Codex 소유 파일(`service.py` 등) 확인 후. 4. R2: watchdog `ExternalShutdownException` 처리 + 종료 matrix `--target`. 5. R3: deploy 도구에 artifact·단일파일 모드.

## 2026-11-22 (22) 다음 승인 (Claude)
R1.2 적용(Codex 수정 파일 소유권 확인 + 패치 `r1_1_01/patches/R1_2_constants.patch`), 로봇 시작 스크립트 교체(별도 승인), `docs/API.md` 라우트 3개 추가(R1.3), R2 Watchdog 종료 수정(신호 플래그 조건부). 세부: [제안 문서](fleet_control/docs/R1_1_P6_AND_R1_2_PROPOSAL_20261122.md).

## 2026-11-23 (23) R1.4.1 다음 (Claude)
Codex 독립 재감사(재현 명령: [R1_4_1 문서 §10](fleet_control/docs/R1_4_1_P1_FIXES_20261123.md)) → 통과 시 Agent v5 패키지·격리 실차 검증 승인 요청. 남은 항목: Arbiter/Bench/Gate-reset/localization worker에 server_contract_valid 공통 적용 제안, R1.2 상수 패치 재생성(문맥 변경), R2 Watchdog 종료 코드.

## 2026-11-23 (24) R1.4.2 다음 (Claude)
Codex 재감사(명령: [R1_4_2 문서 §7](fleet_control/docs/R1_4_2_CONTRACT_FOLLOWUP_20261123.md)) → 통과 시 Agent v5 패키지·격리 실차 검증 승인 요청. R2/R3는 오프라인 진행 가능.

## 2026-11-23 (25) R1.4.3 다음 (Claude)
Codex 재감사(명령: [R1_4_3 문서 §7](fleet_control/docs/R1_4_3_CONTRACT_GENERATION_20261123.md)) → 통과 시 Agent v5 패키지·격리 실차 검증 승인 요청. 남은 제안: 응답 세대/시각 비교(순서 역전 방어)는 실증 전까지 보류.

## 2026-11-23 (26) R1.4.4 다음 (Claude)
Codex 재감사(명령: [R1_4_4 문서 §9](fleet_control/docs/R1_4_4_SUBMISSION_FENCE_20261123.md)) → 통과 시 v5 패키지·격리 실차 검증 승인 요청. Codex fence.py는 Stage 3 내부 무효화를 기다리므로 수정 후 경계에 도달 불가(설계상 기대); 동등 시험은 test_r1_4_4 B.

## 2026-11-23 (27) R1.4.5 다음 (Claude)
Codex 재검증(명령: [R1_4_5 문서 §9](fleet_control/docs/R1_4_5_WITHDRAWAL_PATHS_20261123.md)) → 통과 시 R1 종료 제안. 배포 순서: 서버(명시 False) 먼저, 그다음 Agent v5(별도 승인). 별도 제안: 보조 실행 주체(Arbiter, bench, gate reset, localization) 세대 결합, 응답 순서 검사.

## 2026-11-23 (28) R1.4.6 다음 (Claude)
Codex가 R1.4.6 불변식만 독립 재검증(명령: [R1_4_6 문서 §6](fleet_control/docs/R1_4_6_REQUIRED_FIELDS_20261123.md)) → 통과 시 R1 종료. Neon R2/R3 파일은 건드리지 않음.

## 2026-11-23 (29) R1·R2·R3 오프라인 종료 후 인계 (Claude, 문서 통합)
**완료(오프라인만):** R1 안전 계약 R1.4.1~R1.4.6 (Claude 구현, Codex PASS `codex_r146_close_20261123_6ZGjQf/`), R2 Watchdog 종료 안전 (Neon 구현, Codex PASS 66/66), R3 배포 도구 (Neon 구현, Codex PASS 11/11; R2·R3 기록 `codex_neon_r2_r3_close_20261123_6v3X5W/`). 모든 변경은 작업트리에 미커밋. 실차·18190·모터 미사용.
**미해결:** 전체 discover 2 FAIL·33 ERROR(환경); R3 이중 실패 시 SRC 부재(수동 복구); 보조 실행 주체의 권한 세대 결합; 같은 boot 내 응답 순서; P2 스텁 캐시; Watchdog admission 타이밍 flake; 실제 Nav2/DDS·물리 정지·제동거리·E-Stop; `/cmd_vel` 우회; Depth; 전원·USB/UART·STM32 watchdog.
**다음(우선순위, 각각 사용자 승인 필요):**
1. Agent v5 배포 후보 준비(오프라인): R1+R2 소스의 R3 `artifact` 생성·manifest·해시 고정, 커밋 범위 결정(G1 이후), 서버 먼저 배포하는 순서 확정(서버의 명시 `reconciliation_required: false`·lease 필수 전송이 v5의 전제). 담당 제안: Claude(R1 서버·Agent), Neon(R3 도구·R2 Watchdog 패키징).
2. R3 이중 실패의 현장 안전 복구 절차(또는 원자 교체) — 현장 배포 전 차단 조건.
3. 실제 LIMO 모터 잠금 상태 격리 검증(R4): Fleet 서버 → Agent v5 → 잠금 스택의 Gate·Watchdog·Arbiter·Mapping STOP·DDS.
4. 후속 백로그: 보조 실행 주체 세대 결합, 응답 순서 검사.
R4·실차 배포·모터 잠금 해제는 이번 종료로 승인되지 않음.

## 2026-11-23 (30) Agent v5 candidate 다음 (Claude)
후보 준비 완료(오프라인), 배포 BLOCKED. 승인 필요 순서: (1) 소스 commit 범위 결정 후 candidate 재생성, (2) package.xml 의존성 보완 여부, (3) R3 이중 실패 복구 절차, (4) 서버 선배포 승인, (5) 실차 격리 검증(R4). 상세 [AGENT_V5_CANDIDATE](fleet_control/docs/AGENT_V5_CANDIDATE_20261123.md).

## 2026-11-23 (31) v5b 다음 (Claude)
v5b(`c71b166f…`)가 현재 후보. 남은 승인 순서는 (30)과 같음: commit 범위(목록은 AGENT_V5 문서 §8) → 후보 재생성 → R3 이중 실패 복구 → 서버 선배포 → 실차 격리 검증.
