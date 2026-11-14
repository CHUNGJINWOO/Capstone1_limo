# DECISIONS.md — 확정된 아키텍처 · 안전 · 운용 설계 결정

> **기준일**: 2026-11-14  
> 이 문서는 **(1) 이미 확정된 아키텍처·안전·운용 결정(`D-01`~`D-08`)**과 **(2) 합성·Gazebo 검증을 마쳤으나 정식 병합·실차 이식 및 세부 정책 확정을 위해 사용자 승인이 필요한 후보(`C-01: fixed2`)**를 엄격히 구분합니다.  
> 사용자 승인 없이 새로운 설계 결정을 확정 항목으로 승격하지 마세요.

---

## 1. 확정된 온보드 자율주행 및 안전 게이트 결정

### D-01. 단일 속도 명령 게이트(`stall_detector_node`) 및 후진 차단
- **결정**: 모든 자율주행 속도 명령은 `Nav2 -> (/cmd_vel_pre_cm -> Collision Monitor ->) /cmd_vel_raw -> stall_detector_node -> /cmd_vel` 단일 경로만 통과해야 하며, 어떠한 노드도 `/cmd_vel`을 직접 발행해 게이트를 우회할 수 없다. 후진 명령(`linear.x < 0`)은 LiDAR 후방 사각(약 140°) 때문에 게이트에서 항상 `0.0`으로 클램핑한다.
- **근거**: 로컬 개발 노트 `docs/dev_notes/2026_11_10_cmd_vel_안전게이트_경로_점검.md`

### D-02. 실차 기본 모터 잠금(`allow_motor_drive:=false`) 및 구동 승인 조건
- **결정**:
  1. 모든 실차 launch 및 현장 시험 스크립트는 기본값을 `allow_motor_drive:=false`(출력 토픽 `/cmd_vel_locked`, `/cmd_vel` 발행자 0개)로 둔다.
  2. 2026-11-13 실차 시험에서 발생한 **전원 돌발 차단(4회) 및 베이스 USB(`ttyUSB`) 재연결 원인이 규명되고 완충 소크 시험을 통과하기 전까지 실차 자율 구동(`--drive`)을 금지**한다.
  3. 구동 시험 시에도 반드시 사용자가 전원 스위치 옆에 입회하고 단계별 승인을 얻어야 하며, 주행 중 케이블 강제 분리 시험은 엄격히 금지한다.
- **근거**: [docs/REAL_ROBOT_STATUS.md](docs/REAL_ROBOT_STATUS.md) (`main` `2010e44` / `419e798` 브랜치; 로컬 `<WORKSPACE_PARENT>/Capstone1_limo_dashboard/docs/REAL_ROBOT_STATUS.md`), 로컬 미추적 `experiments/real_limo_field_20261113/FIELD_RESULTS_20261113.md` 및 `experiments/real_depth_lidar_mapping_20261113/STATUS.md`

### D-03. 시뮬레이션 기준선과 실차 설정 파일의 명시적 분리
- **결정**: 시뮬레이션 기준선 YAML(`integrated_nav2_params.yaml`, `integrated_explore_params.yaml`)을 실차 튜닝값으로 덮어쓰지 않고, 실차 전용 설정(`integrated_nav2_real_depth_params.yaml`, `integrated_explore_real_params.yaml`, `integrated_nav2_lidar_only_params.yaml`)과 차동 시뮬레이션 설정(`integrated_nav2_diff_params.yaml`)으로 파일을 분리한다.
- **근거**: [docs/CONFIG_SIM_VS_REAL.md](docs/CONFIG_SIM_VS_REAL.md) (`main` `2010e44` / `419e798` 브랜치; 로컬 `<WORKSPACE_PARENT>/Capstone1_limo_dashboard/docs/CONFIG_SIM_VS_REAL.md`)

### D-04. SLAM Toolbox 입력 스캔 정규화 (`limo_scan_normalizer`)
- **결정**: YDLiDAR 기동 직후 스캔 점 개수 변동(233~245개)으로 인해 `slam_toolbox`(Karto)가 스캔을 거부하고 지도가 고정되는 문제를 막기 위해, `/scan`을 244빔 고정 `/scan_slam`으로 정규화해 SLAM에만 공급하고 안전 게이트와 장애물 감지는 원본 `/scan`을 그대로 사용한다.
- **근거**: [experiments/safety_gate_verification_20261110/slam_scan_fix_20261111/REPORT.md](experiments/safety_gate_verification_20261110/slam_scan_fix_20261111/REPORT.md), `docs/scan-normalizer-locked-test-20261112`(`09d742c`) 브랜치 및 로컬 `docs/dev_notes/2026_11_12_실차_스캔정규화_배포와_모터잠금_검증.md`

---

## 2. 확정된 중앙 관제 · Open-RMF · 영상 AI · 데이터 관리 결정

### D-05. Mission Manager 단일 제어 평면 유지 및 제어권 Lease
- **결정**: Open-RMF Fleet Adapter나 웹 대시보드가 로봇의 Nav2 또는 `/cmd_vel`에 직접 명령하는 것(`rmf_demos`의 기본 `full_control` 방식)을 금지하고, 반드시 **Mission Manager(`fleet_server`)**를 단일 제어 평면으로 경유한다. 웹 운영자와 Open-RMF 간의 충돌은 서버 측 제어권 Lease(`ownership.py`)와 증거 기반 복구(`recovery.py`)로 통제한다.
- **근거**: `feat/fleet-control-rmf-integration`(`419e798`) 브랜치의 `fleet_control/docs/API.md` 및 `fleet_control/rmf_adapter/docs/RMF.md`, 로컬 미추적 `experiments/real_rmf_20261113/RMF_DEMOS_RESEARCH.md`

### D-06. Open-RMF 격리 설치 (`$HOME/rmf_isolated`)
- **결정**: 시스템 ROS 2 Humble 패키지를 오염시키거나 루트 권한을 요구하지 않도록, Open-RMF 2.1.8 `.deb` 패키지를 `$HOME/rmf_isolated`에 압축 해제하고 `PYTHONNOUSERSITE=1`(NumPy 2.x 충돌 방지)을 포함한 `env.sh`로 격리 실행한다.
- **근거**: `feat/fleet-control-rmf-integration`(`419e798`) 브랜치의 `fleet_control/rmf_adapter/scripts/install_rmf_isolated.sh`

### D-07. 실시간 영상 AI(YOLO)와 항법·안전 제어의 분리
- **결정**: 노트북 CUDA YOLO11n 검출 결과는 관제 대시보드 표시 및 상위 상황 인지 보조에만 사용하며, 로컬 Nav2나 Safety Gate의 정지·회피 루프 입력으로 연결하지 않는다. 영상 링크 지연·단절이 발생해도 온보드 LiDAR/Depth 기반 안전 루프는 독립적으로 동작해야 한다.
- **근거**: `fleet_control/integrated_sim/RESULTS.md` §3, §5 (`exp/integrated-sim-20261113` 워크트리 내 미커밋 문서; 로컬 `<WORKSPACE_PARENT>/Capstone1_limo_integrated_sim/fleet_control/integrated_sim/RESULTS.md`)

### D-08. 저장소 공개 범위 및 대용량 데이터 분리 (`rosbag` 우선 원칙)
- **결정**:
  1. GitHub 저장소에는 소스코드, 설정, 정제된 요약 지표(JSON/CSV), 기술 보고서만 커밋한다.
  2. 학교 제출 문서(진도보고서·계획서·발표자료 원본), 개인정보(학번·연락처), 인증 토큰·런타임 DB는 절대 커밋하거나 외부에 공개하지 않는다([docs/PUBLICATION_POLICY.md](docs/PUBLICATION_POLICY.md)).
  3. 실험 재현성의 1차 증거는 영상이 아니라 **`rosbag2` + 실행 메타데이터(`result.json`, `events.jsonl`, 파라미터 스냅샷, 커밋/파일 SHA-256)**이며, 대용량 `rosbag2`와 영상은 로컬 `local_data/` 및 Google Drive 아카이브에서 `run_id`로 연결해 관리한다([VALIDATION.md](VALIDATION.md)).

---

## 3. 합성·Gazebo 검증 후보 및 미확정 정책 (사용자 승인 대기 — 확정 금지)

### C-01. 안전 체인 및 장애물 기억·해제 정책 v2 (`experiments/safety_chain_fix_20261113/fixed2`)
> **상태**: 합성 하네스(`run05`, 28/28 PASS) 및 Office Gazebo(`office03_20261114`)에서 검증된 **실험 후보(Candidate)**이며, 아직 `capstone_ws/src/limo_capstone` 및 실차 번들에 정식 병합되지 않았습니다. 아래의 **검증된 후보 기능**, **기각된 실험 설정**, **미확정 정책**을 구분합니다.

1. **합성 하네스 + Office Gazebo 검증 완료 후보 기능**:
   - **사각 장애물 Fail-closed 유지 및 증거 기반 해제**: 사각(LiDAR 최소거리 안, Depth 0.55 m 안)에 들어간 확정 장애물 셀은 시간 경과만으로 자동 삭제하지 않고(`TTL` 제거), 동일 센서의 연속 free 관측(LiDAR 5회, Depth 3회), 미확정 단발 노이즈 만료(1초), 또는 운영자의 명시적 해제 서비스(`/collision_monitor/clear_memory_near`)로만 해제 (로컬 미추적 `experiments/safety_chain_fix_20261113/docs/MEMORY_POLICY_V2.md`).
   - **전용 Approach Footprint 토픽 및 Costmap `memory_layer`**: Humble `nav2_collision_monitor`가 YAML의 `FootprintApproach.points`를 무시하는 문제를 우회하기 위해 전용 토픽(`/collision_monitor/approach_footprint`)으로 차체 사각형 footprint를 공급하고, Local/Global Costmap에 `/collision_monitor/memory_points`를 구독하는 `memory_layer`를 추가해 플래너와 모니터의 장애물 인식을 일치시킴 (`explore_rep2` 16.22 m, `explore_rep3` 18.48 m 주행 확인).
   - **센서 타임스탬프 정체 감시 및 고장 시점 목표 자동 재개 차단**: `/collision_monitor/memory_status`와 `/auto_mapping/depth_status`의 `STAMP_NOT_ADVANCING` 감시 및 `block_resume_of_fault_goals:=true`(고장 시점의 이전 Nav2 목표가 종료된 후 운영자 reset을 거쳐야 새 목표 주행 허용) 동작을 합성·Gazebo에서 확인.
2. **시험 중 기각(Rejected) 또는 트레이드오프가 확인된 설정**:
   - **Costmap `robot_radius: 0.22 / inflation_radius: 0.40` 확대 기각**: `office03` `explore_rep1`에서 해당 값 적용 시 26.5초(1.55 m) 만에 좁은 통로 frontier가 모두 막혀 탐색이 조기 종료됨에 따라 기각하고 `0.18 / 0.30`으로 복귀함.
   - **F4 ICP `base_shift_correction: false` 트레이드오프**: 정지 상태에서 각도 오차가 거짓 선속도로 피드백되는 현상은 막았으나, 약 7 m 주행 후 오도메트리 드리프트가 누적될 때 보정하지 못하는 한계가 남아 있음.
3. **미확정·미구현 정책 (사용자 의사결정 필요 — 로컬 미추적 `experiments/safety_chain_fix_20261113/docs/FAILSAFE_CONTRACT.md` §4~§5)**:
   - `FAILSAFE_ICP_DEGRADED`를 일시적 정지(복구 시 해제)로 둘지, 래치 잠금(`SAFETY_LATCHED`) 대상으로 포함할지 미확정.
   - 시뮬레이션(`require_operator_unlatch=false`)과 실차(`require_operator_unlatch=true`) 간 잠금 해제 정책을 단일화할지 분리 유지할지 미확정.
   - 실차 DaBai Depth 지터(최대 680 ms vs 타임아웃 0.75 s) 해결 전에 `FAILSAFE_DEPTH_STALE`에 `latch_safety_faults=true`를 적용하면 반복 잠금이 발생할 수 있어 적용 시점 미확정.
   - Mission Manager(`fleet_server`) 및 Robot Agent(`limo_robot_agent`)에서 `SAFETY_LATCHED` 수신 시 임무 자동 취소 및 `/safety_gate/reset` 연동 계약은 아직 **제안 단계(미구현)**임.
