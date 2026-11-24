# CODE_INDEX.md — 소스코드 단일 기준점(SSOT) 및 실험 변형 색인

> **기준일**: 2026-11-14  
> 여러 워크트리와 `experiments/` 하위 폴더에 패키지 사본이 존재합니다. 코드를 수정하거나 읽을 때는 반드시 아래 표의 **(1) `main` 브랜치 공개 GitHub 경로(클릭 가능한 마크다운 링크)**, **(2) 타 브랜치(`419e798`, `c804241`) 전용 경로**, **(3) 실차(LIMO) 실제 배포·실행 번들(로컬 미추적)**, **(4) Gazebo/합성 시험본 및 `fixed2` 실험 후보(로컬 미추적)**를 구분하세요.  
> **링크 및 경로 표기 규칙**: 공개 GitHub `main` 브랜치에서 실제로 열리는 경로만 마크다운 상대 링크로 연결하고, 타 브랜치 전용 파일이나 로컬 워크트리의 미추적·미커밋 파일은 백틱(`` `경로` ``)과 `<WORKSPACE_PARENT>` 기준 로컬 경로로 구분해 표기합니다.

---

## 1. ROS 2 온보드 패키지 기준점 (`main` 브랜치 기준 · 실차 배포본 · 시뮬/실험 후보 구분)

| 패키지 / 모듈 | `main`(`2010e44`) 공개 GitHub 정식 경로 (SSOT) | 실차(LIMO) 실제 배포·실행 번들 (2026-11-12~13, 로컬 미추적) | Gazebo·합성 시험본 및 `fixed2` 실험 후보 (노트북 전용 미추적, 실차 미배포) | 보관 / 레거시 사본 (수정 금지) |
|---|---|---|---|---|
| `limo_capstone` (`stall_detector_node.py`, `lidar_scan_matcher_odom_node.py`, launch, Nav2/Explore YAML) | [capstone_ws/src/limo_capstone](capstone_ws/src/limo_capstone)<br/>- 실차 분리 YAML 2종(`integrated_explore_real_params.yaml`, `integrated_nav2_real_depth_params.yaml`)은 `main`(`2010e44`) 및 `419e798` 브랜치의 `capstone_ws/src/limo_capstone/config/`에 포함됨 (로컬 `c804241` 체크아웃에서는 `<WORKSPACE_PARENT>/Capstone1_limo_dashboard/capstone_ws/src/limo_capstone` 참조) | 1. **실차 LiDAR-only 배포본 (`run_001~011`, `run_010` 구동 맵핑)**: `experiments/real_auto_mapping_ready_20261113/src/limo_capstone` → 실차 `~/limo_auto_mapping_ws`<br/>2. **실차 Depth+Slow 배포본 (`run_012~015` 모터 잠금 시험)**: `experiments/real_depth_slow_20261113/src/limo_capstone` → 실차 `~/limo_depth_slow_ws` | 1. **안전 체인 수정 후보 v2 (`fixed2`, 합성 `run05` + Office Gazebo `office03` 검증)**: `experiments/safety_chain_fix_20261113/fixed2` (`stall_detector_node.py`, `obstacle_memory_node.py`, `lidar_scan_matcher_odom_node.py`, `navigate_to_pose_diff_no_backup.xml`)<br/>2. **실차 준비용 미배포 번들**: `experiments/real_depth_slow_fp_20261113/src/limo_capstone` (Footprint), `experiments/real_depth_slow_cm_20261113/src/limo_capstone` (Collision Monitor + 초기 기억 노드) | - `experiments/` 내 2026-11-05~08 폴더의 `ws/src` 및 `nodes/`<br/>- `safety_chain_fix_20261113/baseline` 및 `fixed`<br/>- `notebook/2026_11_03~07/` 내 `LIMO_COPY`<br/>- `local_data/old_workspaces/capstone_ws_10_21` |
| `limo_depth_obstacles` (`depth_obstacle_filter_node.py`) | [capstone_ws/src/limo_depth_obstacles](capstone_ws/src/limo_depth_obstacles) (`main` `2010e44` 및 `c804241` 동일) | **실차 Depth 배포본 (`run_012~015`)**: `experiments/real_depth_slow_20261113/src/limo_depth_obstacles` → 실차 `~/limo_depth_slow_ws` | - `experiments/real_depth_slow_fp_20261113/src/limo_depth_obstacles`<br/>- `experiments/real_depth_slow_cm_20261113/src/limo_depth_obstacles` (실차 미배포) | `experiments/depth_mapping_20261107/ws/src/limo_depth_obstacles` (`c804241` 브랜치 전용) |
| `limo_scan_normalizer` (`scan_normalizer_node.py`) | [experiments/safety_gate_verification_20261110/variants/scan_normalizer_ws/src/limo_scan_normalizer](experiments/safety_gate_verification_20261110/variants/scan_normalizer_ws/src/limo_scan_normalizer) | - 2026-11-12 실차 배포: `~/capstone_io_ws`<br/>- 2026-11-13 실차 번들 포함본(로컬 미추적): `experiments/real_auto_mapping_ready_20261113/src/limo_scan_normalizer`, `experiments/real_depth_slow_20261113/src/limo_scan_normalizer` | `experiments/real_depth_slow_cm_20261113/src/limo_scan_normalizer` (로컬 미추적) | 없음 |
| `obstacle_memory_node.py` (근거리 사각 장애물 기억 노드) | 아직 `capstone_ws/src/limo_capstone` 정식 브랜치에 미병합 | **실차 미배포** (실차에서 실행된 적 없음) | **`experiments/safety_chain_fix_20261113/fixed2/nodes/obstacle_memory_node.py`** (기억·해제 정책 v2 후보, 합성 `run05` + Office Gazebo `office03` 검증, 로컬 미추적) | - `safety_chain_fix_20261113/baseline/` 및 `fixed/`<br/>- `real_depth_slow_cm_20261113/src/limo_capstone/`<br/>- `office_limo_sim_20261113/ws/src/limo_collision_guard/` |
| `limo_ros2` (`limo_base`, `limo_car`, `limo_description`, `limo_msgs`) | `capstone_ws/src/limo_ros2` (`.git/info/exclude`에 의해 로컬 전용 보존; `main` 공개 트리에서는 제외됨) | 실차 하드웨어 드라이버는 로봇 온보드 기존 워크스페이스(`~/agilex_ws` 등) 사용 | 시뮬레이션 URDF/메시지는 각 실험 워크스페이스에서 참조 | - `archive/legacy_agilex_src/limo_ros2` (`.git/info/exclude` 로컬 보관본)<br/>- `Capstone1_limo/src/limo_ros2` (미추적 100% 중복본) |
| `m-explore-ros2` (`explore_lite`) | [capstone_ws/src/m-explore-ros2](capstone_ws/src/m-explore-ros2) (`main` `2010e44` 및 `c804241` 동일) | 실차 번들(`real_auto_mapping_ready_20261113`, `real_depth_slow_20261113`)에 동일 포함 | [experiments/safety_gate_verification_20261110/variants/explore_variant_ws/src/explore_relaxed](experiments/safety_gate_verification_20261110/variants/explore_variant_ws/src/explore_relaxed) (시뮬 평가 전용 변형) | 없음 |

---

## 2. 중앙 관제 · Open-RMF · 대시보드 · 시뮬레이션 기준점 (Git 브랜치 기준 vs 로컬 워크트리)

`fleet_control/`은 Git 브랜치 `feat/fleet-control-unified`(`ed53d81`, `origin/main` `51175ef` + `419e798` 기반)에 통합본(203개 비런타임 파일)이 커밋되었으며, 로컬 단일 기준 위치는 `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/`입니다. (기능 워크트리 4개는 각 브랜치 스냅샷 커밋 후 해제 완료)

| 하위 시스템 | Git 브랜치 기준 버전 | 현재 로컬 단일 기준 경로 (`<WORKSPACE_PARENT>` 기준) | 포함 기능 및 검증 구분 | 참조 스크립트 및 주의 사항 |
|---|---|---|---|---|
| **중앙 관제 서버 (`fleet_server`) 기본본** | `feat/fleet-control-unified` (`ed53d81`) `fleet_control/server` | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/server` | `fleet_server/{__main__.py, http_api.py, service.py, db.py, auth.py, ownership.py, recovery.py, static/}` (`run_checks` 46/46, `test_ownership` 37/37, `test_recovery` 14/14 PASS) | `Capstone1_limo/fleet_control/`은 **런타임 `data/{auth.json,fleet.db}`와 `secrets/*.token`을 보유**하므로 삭제 금지 (Git 비추적 유지) |
| **관제 서버 + 실시간 영상 패널 (`video_panel.js`)** | `feat/fleet-control-unified` (`ed53d81`) `fleet_control/server` (스냅샷: `exp/video-ai-preverify` `e4dd598`) | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/server` | `--video-origin` 지원, `static/video_panel.js`, `docs/DASHBOARD_GUIDE.md`, `docs/REAL_LIMO_VIDEO_TEST.md` | [lf_setup.sh](experiments/real_limo_field_20261113/laptop/lf_setup.sh)가 `$REPO_ROOT/fleet_control` 상대경로를 참조하도록 정정됨 |
| **Robot Agent (`limo_robot_agent`) + Open-RMF Adapter (`limo_rmf_adapter`) + Gazebo RMF** | `feat/fleet-control-unified` (`ed53d81`) (스냅샷: `codex/rmf-gazebo-integration` `0dc7ef9`) | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/{agent,rmf_adapter,gazebo}` (실차 온보드: `~/limo_field_agent_ws`) | `motor_policy.py`(`SIM_GATED`), `adapter_node.py`, `config/fleet_gazebo.json`, `gazebo/localized_sim.launch.py`, `tests/run_gazebo_checks.py` (Gazebo RMF 69/69 PASS) | [lf_env.sh](experiments/real_limo_field_20261113/laptop/lf_env.sh) 및 [offline_registration_test.py](experiments/real_rmf_20261113/tools/offline_registration_test.py)가 `Capstone1_limo/fleet_control` 상대경로 참조 |
| **통합 시뮬레이션 (`integrated_sim`)** | `feat/fleet-control-unified` (`ed53d81`) (스냅샷: `exp/integrated-sim-20261113` `29818e8`) | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/integrated_sim` | `integrated_env.py`, `integrated_sim.launch.py`, `run_integrated.py`, `fleet_integrated.json`, `RESULTS.md` (**Gazebo + CUDA YOLO 통합 시뮬 전용**, 풀스택 재실행 미검증) | RMF + Gazebo + Nav2 + Video/YOLO 통합 실행기 |
| **시각화 대시보드 (`dashboard` / `dashboard_lab` / `dashboard_v2_design` / `rmf_web_research`)** | `feat/fleet-control-unified` (`ed53d81`) (스냅샷: `codex/dashboard-visualization` `bc3df5e`) | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/{dashboard,dashboard_lab,dashboard_v2_design,rmf_web_research}` | - `dashboard/`: `test_visualization` 31/31, JS 18/18 PASS<br/>- `dashboard_lab/`: `test_lab.py`는 `aiohttp` 미설치로 미검증<br/>- `dashboard_v2_design/`, `rmf_web_research/` | `dashboard*/README.md` 등 문서 4곳 예시 경로도 `Capstone1_limo` 기준으로 정정됨 |
| **실차 `map_002`용 Open-RMF 설정** | 로컬 미추적 폴더: `experiments/real_rmf_20261113` | `<WORKSPACE_PARENT>/Capstone1_limo/experiments/real_rmf_20261113` | `limo_rmf_real/`, `config/`, `localization/`, `tools/offline_registration_test.py` (**오프라인 등록·합성 검증만 완료**, 실차 RMF 주행 미실시) | 현장 시험은 전원 차단 이슈로 보류 상태 |
| **Office Fortress 월드 + LIMO 시뮬레이션** | 로컬 미추적 폴더: `experiments/office_limo_sim_20261113` 및 `experiments/safety_chain_fix_20261113/office` | `<WORKSPACE_PARENT>/Capstone1_limo/experiments/office_limo_sim_20261113` | `office_limo.world`, `make_world.py`, `make_launch.py`, `office_limo_fixed2.launch.py` (**Gazebo 시뮬레이션 전용**) | `safety_chain_fix_20261113`의 launch 파일이 `office_limo_sim_20261113/office_limo.world` 경로를 직접 참조함 |

---

## 3. 완전 중복 확인된 경로 (신규 수정 금지)

아래 경로들은 정식 경로와 `diff -rq` 대조 결과 100% 중복이 확인된 경로이므로 절대 수정하지 마세요.
- `Capstone1_limo/src/limo_ros2` → 로컬 정식 보관본은 `archive/legacy_agilex_src/limo_ros2`
- 루트의 구 개발 노트 및 산출물 중복 폴더 → 정식 경로는 로컬 `docs/dev_notes/` 및 [docs/deliverables](docs/deliverables) (비공개 학교 행정 문서는 [docs/PUBLICATION_POLICY.md](docs/PUBLICATION_POLICY.md)에 따라 로컬 전용 보존 및 커밋 제외)


## 2026-11-15 Registry · 운용 경계 · Gate 후보 색인

- Registry 구현: [map_registry.py](fleet_control/server/fleet_server/map_registry.py), [http_api.py](fleet_control/server/fleet_server/http_api.py); 고아 읽기 전용 진단: [orphan_diagnostics.py](fleet_control/server/fleet_server/orphan_diagnostics.py)와 [복구 제안](fleet_control/server/ORPHAN_RECOVERY_PROPOSAL.md). 진단은 쓰기가 멈춘 번들 사본과 체크포인트된 DB 스냅샷에 한정된다.
- 독립 Mock 및 설계: [Phase 1](fleet_control/OPERATION_MANAGER_PHASE1_PROPOSAL.md), [Phase 1.1](fleet_control/OPERATION_MANAGER_PHASE1_1_PROPOSAL.md), [Phase 1.2](fleet_control/OPERATION_MANAGER_PHASE1_2_PROPOSAL.md), [격리 ROS Action 시험 코드](fleet_control/tests/test_ros2_goal_boundary_isolated.py). 기존 Explore 구현은 [explore.cpp](capstone_ws/src/m-explore-ros2/explore/src/explore.cpp).
- Gate `fixed2`/`fixed3` 실험본과 별도로 [v4/v4.1 보고](experiments/safety_chain_fix_20261113/docs/GATE_V4_RESIDUAL_RISK_DESIGN_20261115.md), [v4.2 후보 보고](experiments/safety_chain_fix_20261113/docs/GATE_V4_2_SAFETY_REVALIDATION_20261115.md) 및 해당 diff가 있다. 후보 diff는 배포된 fixed 버전이 아니다.
- Gate r7 독립 후보(2026-11-16): `experiments/safety_chain_fix_20261113/candidate_gate_r7/nodes/stall_detector_node.py`와 [r6→r7 diff](experiments/safety_chain_fix_20261113/docs/CANDIDATE_gate_v4_2r6_to_v4_2r7_20261116.diff)가 있다. 통합 도구는 `tools/gate_r7_integration.py`(격리), `tools/r7_agent_proc.py`(Codex Agent를 수정 없이 import하는 SIM_ONLY 래퍼), `tools/run_office_r7.{py,sh}` + `office/office_limo_r7.launch.py`(Gazebo; fixed3 런치에서 Gate만 교체)다. 배포본·fixed4가 아니다.
- Gate r8 독립 후보(2026-11-16, Agent 생존 감시): `experiments/safety_chain_fix_20261113/candidate_gate_r8/nodes/stall_detector_node.py`, [r7→r8 diff](experiments/safety_chain_fix_20261113/docs/CANDIDATE_gate_v4_2r7_to_v4_2r8_20261116.diff). 도구는 `tools/gate_r8_integration.py`(Codex B4n8 드라이버에서 파생), `tools/r8_agent_proc.py`(SIM_ONLY heartbeat 응답자 래퍼), `tools/run_office_r8.{py,sh}` + `office/office_limo_r8.launch.py`, `tools/r8_heartbeat_latency.py`다. 배포본·fixed4가 아니다.
- Gate r9 독립 후보(2026-11-17, Cliff 입력): `experiments/safety_chain_fix_20261113/candidate_gate_r9/nodes/stall_detector_node.py`, [r8→r9 diff](experiments/safety_chain_fix_20261113/docs/CANDIDATE_gate_v4_2r8_to_v4_2r9_20261117.diff). 도구는 `tools/run_cliff_r9.py` + `run_cliff_r9_isolated.sh`, `tools/analyze_cliff_bag.py`, `tools/gate_r9_cliff_unit_test.py`, `tools/make_office_launch_r9.py`, `office/office_limo_r9{,_keepout}.launch.py`, `office/cliff_keepout_r9_*`다. 관측기는 `experiments/cliff_gazebo_20261117_P4d8/`(다른 에이전트, 읽기 전용 사용).
- Gate r10 독립 후보(2026-11-17): `experiments/safety_chain_fix_20261113/candidate_gate_r10/nodes/stall_detector_node.py` = r9 + GetState 재요청, [diff](experiments/safety_chain_fix_20261113/docs/CANDIDATE_gate_v4_2r9_to_v4_2r10_20261117.diff). 도구는 `tools/run_cliff_r10.py`(+`_isolated.sh`), `tools/analyze_cliff_bag_r10.py`, `tools/make_worlds_r10.py` → `office/worlds_r10/`, `tools/gate_r10_getstate_unit_test.py`다.

- Goal 제출 경계 시험 원본: [Phase 1.3 재현](experiments/phase13_ros_action_20261115_K8m4/RESULTS.md), [1.4](experiments/phase14_goal_boundary_20261115_q4H7/RESULTS.md), [1.5 SROS2](experiments/phase15_sros2_20261115_F8v2/RESULTS.md), [1.6 독립 프로토타입](experiments/phase16_goal_arbiter_20261115_D2k8/RESULTS.md). 운영 구현이 아니다.


## 2026-11-16 Fleet UI 검증 경로 (Codex)

- 실제 UI: `fleet_control/server/fleet_server/static/{index.html,style.css,app.js,map_view.js,video_panel.js}`. 기존 서버·Agent·Gate·RMF API/로직은 변경 없음.
- 격리 실행: [dashboard_preview.py](fleet_control/tests/dashboard_preview.py), Chromium E2E: [dashboard_browser.cjs](fleet_control/tests/dashboard_browser.cjs), [보고·캡처](fleet_control/docs/DASHBOARD_UI_20261116.md). 임시 DB·무작위 토큰·loopback 자동할당 포트만 사용하며 ROS/실차 미실행.


## 2026-11-16 Spatial Monitor UI

공간 관제 UI는 `fleet_control/server/fleet_server/static/`, 테스트는 `fleet_control/tests/dashboard_spatial_browser.cjs`. 계약 변경 제안은 `fleet_control/docs/SPATIAL_MONITOR_INTERFACE_REQUEST.md`. [상세](fleet_control/docs/SPATIAL_MONITOR_PHASE3.md).


## 2026-11-16 Spatial 3가지 보기

3모드 UI는 `fleet_control/server/fleet_server/static/{index.html,style.css,app.js}`, 격리 브라우저 검증은 `fleet_control/tests/dashboard_spatial_modes.cjs`. [보고](fleet_control/docs/SPATIAL_VIEWS.md).


## 2026-11-16 Fleet UI 전체 리디자인

Fleet UI 전체 리디자인은 `fleet_control/server/fleet_server/static/{index.html,style.css,app.js}`. 새 Chromium 테스트 `fleet_control/tests/dashboard_redesign_browser.cjs`. [화면·매핑](fleet_control/docs/FLEET_UI_REDESIGN.md).


## 2026-11-16 지도 집중 모드

지도 집중 UI는 `fleet_control/server/fleet_server/static/{index.html,style.css,app.js}`; 테스트는 `fleet_control/tests/dashboard_map_focus.cjs`, `dashboard_focus_regression.cjs`. [보고](fleet_control/docs/MAP_FOCUS_DENSITY.md).


## 2026-11-17 Fleet UI 테마

UI 테마는 `fleet_control/server/fleet_server/static/theme.js` 및 CSS tokens, 캔버스 테마는 `map_view.js`. 테스트는 `fleet_control/tests/dashboard_theme_browser.cjs`. [보고](fleet_control/docs/UI_THEME_REDESIGN_20261117.md).


## 2026-11-17 승인 Workspace 운영 UI 적용

승인 Workspace runtime: `fleet_control/server/fleet_server/static/{index.html,style.css,app.js,map_view.js}`. 새 검증 `fleet_control/tests/dashboard_workspace_runtime.cjs`, `dashboard_workspace_camera.cjs`. [캡처/검증](fleet_control/docs/WORKSPACE_RUNTIME_20261117.md).


## 2026-11-17 `/map` 타입 분리 (Gazebo/RMF 임시 호환 계층)

- `fleet_control/rmf_adapter/rmf_map_shim/` — `rmf_map_remap.c`, `build.sh`, `README.md`. `LD_PRELOAD` shim: `rmf_building_map_msgs/BuildingMap`의 `/map`·`map` publisher/subscription만 `/rmf/building_map`으로 이동(OccupancyGrid `/map`은 그대로). **Gazebo/RMF launch 트리(ign gazebo·`building_map_server`)에만 적용**, Agent·SLAM·rosbag에는 불필요. 빌드 산출물 `build/librmf_map_remap.so`는 생성물(Git 제외). **임시 호환 계층**이며 근본 해결(`rmf_robot_sim_common` 플러그인 소스의 토픽 설정화)과 구분한다. 장기 채택은 미확정(DECISIONS 미반영).
- 실행 시 적용 경로: `experiments/rmf_map_bag_gazebo_20261117_R4m2/{run.sh,run_e2e.py,verify_bag.py,verify.sh}` (`LD_PRELOAD`를 sim launch와 `building_map_server` 환경에만 설정, `--no-shim` 대조군).
- 기록 프로파일: `fleet_control/agent/limo_robot_agent/limo_robot_agent/recording_session.py`의 고정 `--extra`가 `/scan_slam`과 `/rmf/building_map`(`RMF_BUILDING_MAP_TOPIC`)을 전달. `/map`(OccupancyGrid)은 `capstone_ws/.../run_recorder.py` CORE_TOPICS. 운영자 입력 토픽 없음.
- 시험: `fleet_control/tests/test_rmf_map_shim.py`, `test_recording_profile_topics.py` (기존 `test_recording_session_p0.py`·`test_slam_control.py` 유지).
- 보고: [RMF_MAP_TYPE_SEPARATION_20261117.md](fleet_control/docs/RMF_MAP_TYPE_SEPARATION_20261117.md).


## 2026-11-18 Fleet FollowPath 하위 Goal 귀속 (SIM_ONLY 제한 신뢰 모델)

- `fleet_control/agent/limo_robot_agent/limo_robot_agent/safety_protocol.py`(`sim_classify_nav2_goals`, `sim_r7_nav_evidence`), `ros_adapter.py`(`_sim_followpath_child_ok`, `_sim_followpath_overlap_ok`), `fleet_control/server/fleet_server/safety.py`(`sim_subordinate_valid`). SIM_ONLY 전용(`UNAUTHENTICATED_SIM_ONLY` 증거 경로), 실차 경로·Gate·Nav2 무변경.
- 시험: `fleet_control/tests/test_nav2_subordinate_goals.py`. 실제 Gazebo: `experiments/fleet_followpath_trust_20261118_N3p5/`(`run_isolated.sh`, `web_office.py`, `office.py`, `faults_office.py`, `analyze.py`, `fault_metrics.py`, diff `CHANGES_agent_server_followpath.diff`, 원본 `original_before/`).
- 보고: [FLEET_FOLLOWPATH_OWNERSHIP_20261118.md](fleet_control/docs/FLEET_FOLLOWPATH_OWNERSHIP_20261118.md).

## 2026-11-18 Gazebo MANUAL SLAM 프로파일

- 기존 r8 런치 [office_limo_r8.launch.py](experiments/safety_chain_fix_20261113/office/office_limo_r8.launch.py): `manual_mapping:=true`는 자동 SLAM·explore를 끄고 첫 `/map` 이후 기존 Nav2 navigation 런치를 시작한다. AMCL 런치는 포함하지 않는다. 기본값은 기존 동작이다.
- Agent [slam_control.py](fleet_control/agent/limo_robot_agent/limo_robot_agent/slam_control.py): SLAM·AMCL·지도·기존 map→odom 및 Goal 충돌 검사, Nav2 존재 시 Gate 유휴 관측 증거 검사. 시험 [test_slam_control.py](fleet_control/tests/test_slam_control.py). ROS graph 관측은 인증된 소유권이나 동시 실행 경합 방지가 아니다.

- 2026-11-18 MANUAL 주행: [manual_mapping_input.py](fleet_control/agent/limo_robot_agent/limo_robot_agent/manual_mapping_input.py)의 SIM·임대·Gate·Nav2 증거 기반 Joy watchdog, [agent_node.py](fleet_control/agent/limo_robot_agent/limo_robot_agent/agent_node.py)의 opt-in r8 supervisor, r8 MANUAL 런치의 Nav2 속도 격리. 검증 [test_manual_mapping_guard.py](fleet_control/tests/test_manual_mapping_guard.py), `test_manual_mapping_ros.py`; 실차 미검증.

- 2026-11-18 MANUAL STOP: `slam_control.py`의 `enable_manual_stop`, `_confirm_manual_stop`, `_finish_manual` 및 원본 grid/YAML 보존 검사; `fleet_server/service.py`, `http_api.py`, `static/app.js`의 기존 SLAM STOP 명령·Registry·상태 표시 확장. SIM_ONLY Gazebo 근거는 [VALIDATION.md](VALIDATION.md).

## 2026-11-18 AUTO Frontier 연결 부분 구현 (미완성)

- [fleet_frontier_candidates.cpp](capstone_ws/src/m-explore-ros2/explore/src/fleet_frontier_candidates.cpp): 기존 `FrontierSearch`·`Costmap2DClient`로 순위 후보만 발행. Nav2 Action 클라이언트·취소 기능은 없다. `explore/CMakeLists.txt`에 실행 대상 추가.
- [frontier_fleet_coordinator.py](fleet_control/agent/limo_robot_agent/limo_robot_agent/frontier_fleet_coordinator.py): Fleet 제출/개별 취소 호출을 주입받는 미연결 조정 구성요소. 이전 Mission의 Agent journal terminal·동일 UUID·Nav2 idle 관측 전 다음 후보를 허용하지 않는다. 동시 STOP·취소 ACK·불확실한 응답 단위시험은 `test_frontier_fleet_coordinator.py`.
- r8 `auto_mapping:=true`는 localhost 전용 SIM 후보 관측 프로파일이며 SLAM·기존 explore 자동 실행을 끈다. AUTO 웹 제어·Fleet 연결·종합 STOP은 **미구현**이고 Goal 제출을 시작하지 않는다. [실행 실패 근거](VALIDATION.md).

- AUTO Supervisor 진입점: `fleet_control/agent/limo_robot_agent/limo_robot_agent/agent_node.py`의 `auto_mapping_enabled`는 격리 Gazebo SIM, SLAM 활성, 미션 비활성에서만 기존 `sim_r8_supervisor`를 허용한다. AUTO 임무 제출 구현은 아니다. Gazebo·rosbag 결과는 [VALIDATION.md](VALIDATION.md).

- 2026-11-18 실차 `~/limo_field_agent_ws` Agent 소스는 로컬 `fleet_control/agent/limo_robot_agent` 19개 파일과 SHA-256 일치하도록 선택 배포·빌드했다. 실행은 비구동 상태 보고·재연결만 확인했으며 기존 실차 맵핑 bundle 및 Safety Gate는 교체하지 않았다. 백업·제약은 [VALIDATION.md](VALIDATION.md).

## 2026-11-18 실차 Depth TF 및 지도 저장 수정

- [Depth 필터](capstone_ws/src/limo_depth_obstacles/limo_depth_obstacles/depth_obstacle_filter_node.py)는 `/camera/depth/points`를 받아 `base_footprint` 기준 촬영 시각 TF를 조회한다. 미래 시각 TF를 최대 약 150 ms 비동기 대기하고, 기존 신선도·고장 보고를 유지한다. `/auto_mapping/depth_status`의 `tf_timing`에 수신·시도·미래시각·해소·만료 카운터와 10 ms 대기 히스토그램을 발행한다. 실차 bundle의 설치 Depth 모듈과 기준 소스 SHA-256 일치 검증.
- [실차 세션 진입점](experiments/real_auto_mapping_ready_20261113/start_real_auto_mapping.sh)과 [session.py](experiments/real_auto_mapping_ready_20261113/session.py)의 저장 경로는 `/real_mapping_session/map_snapshot`에 고정한 `OccupancyGrid`를 전달한다. Nav2 trinary PGM `--free 0.196` 적용, YAML 원점·해상도 보존, `diagnostics/save_input_NNN.json` 기록. [회귀 스크립트](experiments/real_auto_mapping_ready_20261113/tests/check_map_roundtrip.py)는 격리 map_server 응답과 원본 셀·원점을 대조한다.

## 2026-11-18 베이스 수신 및 Joy 준비 경로

- [LIMO 베이스 드라이버](capstone_ws/src/limo_ros2/limo_base/src/limo_driver.cpp)의 readData → processRxData → parseFrame 경로가 /odom, /imu, /limo_status를 발행한다. [SerialPort](capstone_ws/src/limo_ros2/limo_base/src/serial_port.cpp)는 최소 1바이트 블로킹 읽기를 설정한다. 실차 배포 원본과 두 소스 해시 일치. 이 작업에서 드라이버·Gate 변경 없음.
- 실차 [기준 런치](experiments/real_auto_mapping_ready_20261113/reference/real_mapping_20261112.launch.py)에서 Gate는 /cmd_vel_raw 입력과 잠금 출력을 사용한다. 기존 Nav2 발행자와 공존하는 조이스틱은 별도 명령 중재가 선행돼야 한다. [Joy 격리 설정과 시험](local_data/raw_results/real_limo_base_stream_20261118_01/)은 검증용이다. Agent의 [Joy 경로](fleet_control/agent/limo_robot_agent/limo_robot_agent/manual_mapping_input.py)는 SIM_ONLY 제한을 유지한다.

## 2026-11-18 수동 맵핑 잠금 진단 프로파일

[기준 실차 런치](capstone_ws/src/limo_capstone/launch/limo_integrated_auto_mapping_real.launch.py)의 manual_mapping_probe 기본값은 false. true이면 잠금·차동·Frontier 비활성·조이스틱 이름/두 축/deadman을 요구한다. Nav2 출력은 /nav2_cmd_vel_quarantined, Joy→teleop 출력은 기존 /cmd_vel_raw, 최종 출력은 /cmd_vel_locked. [실차 bundle 이식 후보](local_data/raw_results/real_limo_base_recovery_20261118_01/manual_probe_field.patch)는 스캔 정규화를 보존하며 dry-run과 런치 계획만 검증했다. 운용 배포는 미실시.

실차 Joy는 입력 전용 /manual_mapping/probe_joy로 관측했다. 기존 manual_mapping_probe는 온보드 Joy용 잠금 디버그 경로이며, 웹에서 노트북 입력을 보내는 운용 경로와 별개다. 현장 적용 후보 patch와 기존 파일 해시는 [시험 자료](local_data/raw_results/real_limo_joystick_locked_20261118_01/)에 보관. 현장 번들은 교체하지 않음. Fleet Agent의 기존 SIM_ONLY 제한 유지.

### 웹 수동 맵핑 영속도 벤치

Fleet의 /robots/{id}/manual-bench 및 Agent의 /agent/manual-bench는 기존 인증·제어권을 쓰며 시퀀스와 250 ms 신호 만료를 적용한다. Dashboard Operation의 Gamepad 입력은 실차 Agent manual_web_bench opt-in으로 전달된다. Agent는 계산한 요청을 상태로 보고하고, Gate 그래프와 wheel odom이 유효한 단독 입력 환경에서만 영속도를 보낸다. real_locked_policy 및 기존 SIM_ONLY 경로는 유지. 로컬 실차 런치 후보 manual_mapping_input:=web은 Nav2 속도를 격리하며 로봇의 USB Joy를 기동하지 않는다. [검증 근거](local_data/raw_results/web_manual_bench_20261118_01/README.md)

웹 수동 잠금 프로파일의 Nav2 러닝 그래프는 ROS 203에서 확인했다. 기존 실제 런치의 Nav2 그룹만 재사용해 behavior_server 3개와 velocity_smoother 1개를 /nav2_cmd_vel_quarantined에 격리한다. 실차 현장 런치에 적용하지 않았다. [실측과 재현기](local_data/raw_results/web_manual_field_final_20261118_01/)

## 2026-11-19 LIMO 전용 단일 워크스페이스(`robot_ws`) 배포 색인

- **LIMO 경로**: `/home/pcu/Capstone1_limo/robot_ws` (`env.sh` + 최소 9개 패키지 `src/`: `limo_msgs`, `limo_base`, `limo_description`, `ydlidar_ros2_driver`, `explore_lite_msgs`, `explore_lite` ← `capstone_ws/src/`; `limo_scan_normalizer`, `limo_capstone` ← `experiments/real_auto_mapping_ready_20261113/src/`; `limo_depth_obstacles` ← `capstone_ws/src/`).
- **LIMO 전용 최소 패치**: `/home/pcu/Capstone1_limo/robot_ws/src/limo_capstone/launch/limo_integrated_auto_mapping_real.launch.py`에서 `astra_camera`를 선택 조회(`PackageNotFoundError` 시 `use_depth_camera:=true` 요청을 fail-closed `Shutdown`)로 변경(노트북 원본 무수정).
- **노트북 환경/증거**: `local-only/limo_robot_ws/env_laptop.sh` (Git 제외, FastDDS Discovery Server `<TAILNET_DISCOVERY_IP>:11811` 클라이언트), 검증 rosbag2 및 지도 `local_data/raw_results/limo_runs/20261119_locked_verify/`.


## 2026-11-19 실차 재시험 결과

[자동 맵핑 중단 실행](local_data/raw_results/limo_runs/20261119_auto_map_full/)에 노트북 rosbag, bag에서 복원한 부분 지도, 관제 부분 영상 및 실행 로그가 있다. 완성 지도가 아니다.

## 2026-11-19 자동 맵핑 재실행 02

[안전 중단된 결과](local_data/raw_results/limo_runs/20261119_auto_map_full_02/)에 노트북 bag·부분 지도·관제 녹화·로그 보존. `wheel_odom_stale=true`, 감시기 종료 코드 2; 완성 지도 아님.

## 2026-11-19 STANDBY 통합 기동 1차

- 기존 `limo_capstone` 안의 [잠금 기동 관리자](capstone_ws/src/limo_capstone/scripts/start_locked_standby.py)가 [통합 실차 런치](capstone_ws/src/limo_capstone/launch/limo_integrated_auto_mapping_real.launch.py)와 [Depth 저속 프로파일](capstone_ws/src/limo_capstone/config/integrated_nav2_depth_slow_params.yaml)을 사용한다. 모드 STANDBY, Depth required, diff, Frontier off, 모터 잠금을 고정한다. 설치본은 ROS 환경을 source한 뒤 `python3 "$(ros2 pkg prefix --share limo_capstone)/scripts/start_locked_standby.py"`로 실행한다. `--probe-seconds`는 시간 한정 확인, `--fleet-url`·`--fleet-token-file`·`--fleet-state-dir`는 읽기 전용 실차 Agent를 함께 소유할 때 사용한다. 부팅 자동 등록은 아직 하지 않았다.
- Agent의 [상태 어댑터](fleet_control/agent/limo_robot_agent/limo_robot_agent/ros_adapter.py)가 `/limo/standby_status`를 받아 Fleet로 중계하고, [18190 화면](fleet_control/server/fleet_server/static/app.js)은 최신 STANDBY 준비 실패를 읽기 전용으로 표시한다. 노트북 Fleet/YOLO는 기존 서비스를 별도 운영한다. 브라우저 페이지 접근으로 프로세스를 시작하는 경로는 추가하지 않았다.
- GitHub main `ea2d9632`에는 driver-free `limo_fleet_bringup/launch/isolated_locked.launch.py`와 Agent `velocity_arbiter.py`가 있다. 로컬 staging `c804241a`, 로컬 main `2010e444`, 기존 원격 추적 ref `fb37e742`, LIMO `robot_ws` 설치본은 이 버전과 구분한다. 기존 Arbiter는 잠금 벤치용으로 Phase 1에 연결하지 않았다.

- STANDBY 후속: Supervisor는 외부 PID를 보고하고 기존 스택을 인수하지 않는다. 자기 자식만 정리한다. 스트림별 발행자·수신 갱신, Depth 상태·장애물 점·Costmap 구독, Nav2 lifecycle 및 파라미터, 잠금 증거를 읽기 전용으로 중계한다. 잠금 Depth slow 전용 Gate 최종 선속도 제한 0.15 m/s를 추가했다.
- Fleet Agent는 이전 상태 POST의 HTTP 왕복 시간을 보고하며 18190 UI는 상태 보고 신선도와 별도로 표시한다. RGB 프레임과 YOLO 결과 신선도를 따로 검증한다. 아직 실행 중인 실차/18190 서비스에는 배포하지 않았다.

## 2026-11-19 웹 Mapping 선별 이식 (로컬 `fleet_control/`)

- 서버: `server/fleet_server/mapping.py`(신규, origin/main `ea2d963` 계약 이식), `map_registry.py`(영역·계보 테이블).
- Agent: `agent/limo_robot_agent/limo_robot_agent/mapping_worker.py`, `local_process.py`(신규), `agent_node.py`의 `web_mapping_enabled`(기본 false, SIM/격리 전용).
- UI: `static/mapping_panel.js`, `static/goal_check.js`(신규).
- 스크립트: `scripts/register_c103_map.py`.
- 시험: `tests/test_web_mapping_session.py`, `tests/test_map_areas.py`, `tests/test_goal_check.mjs`, `tests/web_mapping_isolated/`.
- 변경 전 스냅샷: `local_data/worktree_backups_20261119_c103_web/`.
- [문서](fleet_control/docs/WEB_MAPPING_20261119.md)

## 2026-11-20 안전·구조 문서

- `fleet_control/agent/limo_robot_agent/limo_robot_agent/motion_watchdog.py`: Agent-독립 속도·goal 차단(console `motion_watchdog`).
- 시험: `tests/test_motion_watchdog.py`, `tests/web_mapping_isolated/motion_faults.py`, `vel_monitor.py`.
- 문서: `fleet_control/docs/SAFETY_WATCHDOG_20261120.md`, `fleet_control/docs/ARCHITECTURE_AUDIT_20261120.md`(패키지·노드·launch 감사), `fleet_control/docs/proposals_20261120/`(다른 에이전트 파일 대상 패치 제안 3건).

## 2026-11-20 오후 추가

- 시험: `fleet_control/tests/test_watchdog_admission.py`(재시작 정책), `fleet_control/tests/real_locked_probe.py`(실차 모터 잠금 웹 점검, 읽기 전용 기본), `tests/web_mapping_isolated/helper.py`(외부 goal·우회 발행자).
- 제안 패치: `fleet_control/docs/proposals_20261120/02b·03b·04`, 목록 `README.md`.
- 문서: `fleet_control/docs/FIELD_LOCKED_READINESS_20261120.md`.

## 2026-11-20 공개 기준선과 후속 작업 감사

공개 기준 `ea2d9632`의 잠금 격리 런치는 `isolated_locked.launch.py`에 있다. [Mapping Worker](fleet_control/agent/limo_robot_agent/limo_robot_agent/mapping_worker.py), [Fleet Mapping 상태 관리](fleet_control/server/fleet_server/mapping.py), `velocity_arbiter.py`는 이 기준선에 포함된다.

후속 로컬 구현에는 별도 프로세스 `motion_watchdog`, 격리 검증 런치, C103 등록 도구와 후속 회귀시험이 있다. Watchdog은 `/fleet/nav_cmd_vel`, `/fleet/agent_heartbeat`, Nav2 Action 상태를 받아 `/cmd_vel_raw`와 `/fleet/motion_watchdog/status`를 발행한다. Worker는 후보 토픽 `/fleet/frontier_candidates`와 SLAM 저장 서비스 `/slam_toolbox/serialize_map`을 사용한다. Goal 제출 액션은 `navigate_to_pose`다.

로컬 상세 경로는 [구조 감사](fleet_control/docs/ARCHITECTURE_AUDIT_20261120.md), [Watchdog 보고](fleet_control/docs/SAFETY_WATCHDOG_20261120.md), [웹 Mapping 보고](fleet_control/docs/WEB_MAPPING_20261119.md)를 참조한다. 공개 기준선 전용 런치와 Velocity Arbiter는 로컬 staging에 없으므로 `git show origin/main:<경로>`로 확인할 수 있다.

## 2026-11-20 Phase 1 웹 수동 조작 통합 (Claude)

- 연속성 기반 action idle 증거: `fleet_control/agent/limo_robot_agent/limo_robot_agent/action_status_evidence.py` (사용: `ros_adapter.py::manual_idle_evidence`, `velocity_arbiter.py::blocker`). Watchdog 대응: `motion_watchdog.py` (`nav_status_gid`, `NAV2_ACTION_SERVER_CHANGED`).
- 서버 모드 배타성: `fleet_control/server/fleet_server/service.py` (`control_mode`, `create_mission`의 `manual_active`, `_manual_bench_robot`의 `nav_idle`). UI: `static/manual_control_panel.js`(차단 사유 표), `static/app.js`(getContext).
- 격리 E2E: `fleet_control/tests/manual_e2e/{run_manual_e2e.py,fake_nav2_servers.py,browser_manual_inputs.cjs}`, 단위 `tests/test_action_status_evidence.py`. 설계 `fleet_control/docs/INTEGRATED_CONTROL_PLAN_20261120.md`, 보고 `fleet_control/docs/MANUAL_PHASE1_INTEGRATION_20261120.md`.

## 2026-11-20 Phase 2 운용 모드 (Claude)

- 서버: `fleet_control/server/fleet_server/operating_mode.py`(신규 `OperatingModeMixin`), `service.py`(모드 게이트, `operating_mode` 전달), `mapping.py`(RECONCILE), `safety.py`(`status_continuous`, 잠금 mock continuation), `http_api.py`(`GET/POST /robots/{id}/mode`), `__main__.py`(`--require-operating-mode`).
- Agent: `velocity_arbiter.py`(모드별 출처 선택, 입력 `/fleet/cmd_vel_nav_smoothed`), `ros_adapter.py`(서버 모드 = operation 생산자, `status_continuous`), `safety_protocol.py`, `mapping_worker.py`(RECONCILE), `core.py`, `agent_node.py`.
- UI: `static/operating_mode_panel.js`(신규), `mapping_panel.js`, `manual_control_panel.js`, `app.js`, `index.html`.
- 시험: `fleet_control/tests/mode_e2e/`(`stack_up.sh`, `stack_proc.sh`, `kinematic_sim.py`, `agent_nav_fixture.py`[map/loc FIXTURE], `run_mode_e2e.py`, `browser_mode_card.cjs`), `tests/test_operating_mode.py`, `tests/test_operating_mode_panel.cjs`. `tests/web_mapping_isolated/stack.launch.py`에 env 보호된 `ARBITER`·`LIFECYCLE_DELAY_S` 추가(기본 동작 불변).

## 2026-11-20 Phase 3 저장 지도 Navigation·Codex 수정 (Claude)

- 서버 `fleet_control/server/fleet_server/navigation.py`(LOAD/INITIAL_POSE/goal-check, `/agent/map-bundle`), 복구 증거 `ownership.py::_locked_mock_recovery`. Agent `localization_worker.py`. UI `static/navigation_panel.js`.
- 시험: `fleet_control/tests/nav_e2e/run_nav_e2e.py`, `tests/mode_e2e/owned_proc.py`(소유 프로세스만 신호), `tests/test_agent_recovery_hint.py`, `tests/test_phase3_exception_guards.py`, `tests/test_navigation_panel.cjs`.

## 2026-11-20 Gate r11 입력 출처 검증
- 후보: `experiments/safety_chain_fix_20261113/candidate_gate_r11/nodes/stall_detector_node.py` (r10 원본 사본 `candidate_gate_r11/stall_detector_node.r10_frozen.py`), diff `docs/CANDIDATE_gate_v4_2r10_to_v4_2r11_20261120.diff`
- 시험: `experiments/safety_chain_fix_20261113/harness/gate_r11_input/{run_gate_input.py,test_gate_r11_input_unit.py,fake_sensors.py,twist_pub.py}`, 결과 `runs/gate_r11_input_20261120/`
- E2E 프로브: `fleet_control/tests/manual_e2e/run_manual_e2e.py` s18, `tests/mode_e2e/run_mode_e2e.py` g01–g03, `tests/nav_e2e/run_nav_e2e.py` f07b. 환경변수 `GATE_SCRIPT`·`GATE_INPUT_EXCL`·`GATE_INPUT_MAX_VX/WZ`(기본은 기존 Gate)
- 제안 패치: `fleet_control/docs/proposals_20261120/05_gate_input_source_validation.patch` (미적용)

## 2026-11-21 웹 Gate Reset
- 서버 `fleet_control/server/fleet_server/gate_reset.py` (GET·POST `/robots/{id}/gate-reset`), Agent `limo_robot_agent/gate_reset_worker.py` (`web_gate_reset:=true`), UI `static/gate_reset_panel.js`
- Gate 후보 `experiments/safety_chain_fix_20261113/candidate_gate_r12/` (r11 동결 사본 포함)
- 시험 `fleet_control/tests/{test_gate_reset_server.py,test_gate_reset_worker.py,test_gate_reset_panel.cjs,gate_reset_e2e/}`

## 2026-11-21 Phase 3 계측·안정화
- 기록·분석 `fleet_control/tests/nav_e2e/{timeline_recorder.py,analyze_timeline.py}`, 고정 지도 `NAV_E2E_FIXED_MAP`, 재현 `r01_gate_boot_change_then_reconcile_and_mission`(NAV_E2E_ONLY)
- 서버 `ownership.py` `gate_boot_evidence`/`_reconcile_gate_boot`, `safety.py` `_sc_fail`(최초 실패 사유), Agent `localization_worker.py`(초기 위치 전달·planner 지연)
- 진단 `experiments/safety_chain_fix_20261113/harness/gate_r12_reset/gate_callback_timing.py`, 제안 `fleet_control/docs/proposals_20261120/06_gate_input_and_reset_preconditions.patch`

## 2026-11-21 Gazebo 웹 제어 · Gate 정지 진단 · Phase 3.5
- Gazebo(시뮬 전용): `fleet_control/tests/gazebo_web/{gazebo_world.launch.py,gz_truth_adapter.py,run_gazebo_web.py}`, 기동 `tests/mode_e2e/stack_up.sh` `SIM_BACKEND=gazebo`(+`IGN_PARTITION`, `ODOM_ACTIVE_TOPIC`), `tests/web_mapping_isolated/stack.launch.py` `USE_SIM_TIME`
- Gate 정지 진단: `experiments/safety_chain_fix_20261113/harness/gate_r12_reset/{gate_trace.py,scan_witness.py,repro_gate_stall.py,analyse_gate_stall.py,stall_injection.py}`
- Phase 3.5: Agent `limo_robot_agent/{topics.py,ros_graph.py}`, 시험 `fleet_control/tests/{test_locked_graph_contract_characterization.py,test_ros_graph_helpers_characterization.py,e2e_common.py,test_e2e_common.py}`

## 2026-11-21 Gazebo 안정화
- 하위 timeout(시뮬): `fleet_control/tests/gazebo_web/drive_timeout_relay.py`(`GZ_DRIVE_TIMEOUT_S`), 지도 클릭 `browser_map_click.cjs`, 포인터 진단 `tests/manual_e2e/browser_pointer_diag.cjs`
- 웹: `static/manual_control_panel.js`(`active()`, `refreshEvidence`), `static/app.js`(세션 중 300 ms 증거 갱신)
- Agent: `limo_robot_agent/frontier_policy.py`, `mapping_worker.py`(`recent_decisions`, `stop_reason` EXHAUSTED/GOALS_FAILING), 시험 `tests/test_frontier_policy.py`
- 진단: `experiments/safety_chain_fix_20261113/harness/gate_r12_reset/gate_proc_watch.py`

## 2026-11-21 Phase 4 자동 기동 · Phase 5 방문객 웹
- 로봇 측 감독: `fleet_control/agent/limo_robot_agent/limo_robot_agent/supervisor.py`(`--check`), 템플릿 `fleet_control/scripts/autostart/{limo-supervisor.service,supervisor.limo.example.json}`(미설치)
- 서버: `fleet_server/readiness.py`(`/agent/supervisor`, `/robots/{id}/readiness`, `/robots/{id}/services`, `--require-supervisor-ready`), `fleet_server/guide.py`(`/guide/*`, `/visitor-links*`, `/visitor/*`)
- 웹: `static/{system_panel.js,guide_panel.js,qr_encode.js,visitor.html,visitor.js,visitor.css}`, `app.js` 서버 기준 목표 검사
- 시험: `fleet_control/tests/{test_phase4_readiness.py,test_phase4_supervisor.py,test_phase5_guide.py,test_qr_encode.py,test_system_panel.cjs}`, `tests/phase4/`, `tests/phase5/`

## 2026-11-21 Codex Phase 4·5 결함 수정
- 보고: `fleet_control/docs/CODEX_FIX_PHASE45_20261121.md`, 결과 `local_data/raw_results/codex_fix_phase45_claude_20261121/`
- 시험: `fleet_control/tests/test_phase45_fix_20261121.py` (Codex 소유: `test_phase45_{independent,crosslink_http,qr_libzbar}_20261121.py`, 수정 안 함)

## 2026-11-22 Gazebo 통합 실패 수정
- 보고: `fleet_control/docs/GAZEBO_INTEG_FIX_20261122.md`, 결과 `local_data/raw_results/gazebo_integ_claude_20261121/`
- 시간축 기록·분석: `fleet_control/tests/gazebo_web/{timeline_recorder.py,timeline_analyse.py}` (`GZWEB_TIMELINE=1`), 복귀 주행 `run_gazebo_web.py` `home()`/`open_heading()` (`GZWEB_HOME=0`으로 끔)
- Gate 재개 격리 시험: `fleet_control/tests/gate_resume_stale_cmd_isolated.py`; DDS 재시작 프로브: `fleet_control/tests/mode_e2e/dds_restart_probe.py`
- Agent: `velocity_arbiter.py` `CANCEL_REQUESTED`, `ros_adapter.py` steady 타이머

## 2026-11-22 (2) 속도 재출력 안전
- 보고: `fleet_control/docs/GAZEBO_INTEG_FIX_20261122.md` Part 2, 결과 `local_data/raw_results/gazebo_p0_claude_20261122/`
- Agent: `velocity_arbiter.py` 제어 세대(`generation_key`, `update_generation`), `motion_watchdog.py` self-stall 입력 폐기
- 시험: `fleet_control/tests/{test_arbiter_generation_20261122.py,test_watchdog_stall_backlog_20261122.py,gate_writer_swap_graph_20261122.py}`
- Gate r13 후보: `local_data/raw_results/gazebo_p0_claude_20261122/gate_candidate_r13/` (미적용)

## 2026-11-22 (3) Gate r13 통합 준비
- 보고: `fleet_control/docs/GAZEBO_INTEG_FIX_20261122.md` Part 3, 결함 인계 `fleet_control/docs/GAZEBO_OPEN_DEFECTS_20261122.md`, 결과 `local_data/raw_results/gate_r13_integ_claude_20261122/`
- Agent: `velocity_arbiter.py` producer liveness (`on_nav_raw`, `NAV_RAW_MAX_AGE_S`), `topics.NAV_RAW`
- 시험: `fleet_control/tests/{run_guard.py,nav2_smoother_tail_probe_20261122.py,test_arbiter_generation_20261122.py}`
- 캠페인 실행기: `local_data/raw_results/gate_r13_integ_claude_20261122/{run.sh,campaign.sh}`; Gate r13 후보 사본 `candidate_r13/` (미적용)

## 2026-11-22 (4) 실차 조사 · Gazebo LiDAR+Depth
- 보고: `fleet_control/docs/REAL_LIMO_INTEGRATION_SURVEY_20261122.md`, `fleet_control/docs/GAZEBO_DEPTH_20261122.md`; 결과 `local_data/raw_results/{real_limo_integ_claude_20261122,gazebo_depth_claude_20261122}/`
- 시험: `fleet_control/tests/gazebo_depth/{run_depth_targets.py,depth_sensors.launch.py,gate_depth_mode.py,aggregate.py}` (기존 bridge·`depth_to_pointcloud_fixed_node`·`limo_depth_obstacles` 재사용, 공유 launch 미수정)


## 2026-11-22 (6) Nav2 D3 · D1/D2
- 시험: `fleet_control/tests/gazebo_web/{nav2_d3_targeted.py,d1_rotation_probe.py}`, 하네스 변경 `run_gazebo_web.py` (gate_guard, failure_classes)
- 결과·overlay 후보: `local_data/raw_results/nav2_d3_claude_20261122/` (`overlays/`, `d3_run.sh`, `targeted_summary.md`)
- 보고: `fleet_control/docs/GAZEBO_INTEG_FIX_20261122.md` Part 4, `GAZEBO_OPEN_DEFECTS_20261122.md` D1/D2/D3

## 2026-11-23 R1·R2·R3 진입점 (미커밋 작업트리; 오프라인 검증만)
- R1 안전 계약(Agent): `fleet_control/agent/limo_robot_agent/limo_robot_agent/core.py` — `_contract_ok_locked`, `_lease_live_locked`, `_contract_note_locked`(`contract_generation`), `_on_command` 수신 시점 승인, `contract_fence`; `safety_protocol.py` — `lease_wellformed`, `check_active_goal_safety`; `ros_adapter.py`(RosRobot) `_do_nav` Stage 1~3·`_submission_fence`. 서버: `fleet_control/server/fleet_server/service.py` `_deliver_atomic`(명시 `reconciliation_required`), `fleet_server/mapping.py` STOP 전용 분기. 계약 문서 `fleet_control/server/SAFETY_CONTRACT.md`, 단계별 문서 `fleet_control/docs/R1_4_{1..6}_*_20261123.md`.
- R1 시험: `fleet_control/tests/test_r1_4_safety_contract_hardening_20261123.py`, `test_r1_4_1_agent_protocol_20261123.py`, `test_r1_4_1_mapping_stop_20261123.py`, `test_r1_4_2_contract_followup_20261123.py`, `test_r1_4_3_contract_generation_20261123.py`, `test_r1_4_4_submission_fence_20261123.py`, `test_r1_4_5_withdrawal_paths_20261123.py`, `test_r1_4_6_required_fields_20261123.py`(`STATE_MATRIX`).
- R2 Watchdog 종료(Neon): `motion_watchdog.py` — `mark_stop_requested`, `request_shutdown`, `_publish_zero`, `_publish_authorized`(`_publish_lock`), `main()`의 `_on_stop_signal`·`_finalize_signal_stop`; `mock_robot_node.py` 종료 경로. 시험 `fleet_control/tests/test_r2_watchdog_shutdown_20261123.py`(ROS 필요), 종료 matrix `fleet_control/tests/agent_shutdown_matrix_20261122.py --target {agent,motion_watchdog,mock_robot,supervisor}`.
- R3 배포 도구(Neon): `fleet_control/scripts/limo_package_deploy_20261122.sh` — `preflight`/`backup`/`stage`/`replace`/`build`/`verify`/`rollback`/`artifact`/`file-replace`/`file-verify`/`file-rollback`, 전역 `--dry-run`. 시험 `fleet_control/tests/test_r3_deploy_tool_20261123.py`.
- Codex 독립 검증 기록: `local_data/raw_results/codex_r146_close_20261123_6ZGjQf/`, `local_data/raw_results/codex_neon_r2_r3_close_20261123_6v3X5W/`.

- Agent v5 candidate: `fleet_control/docs/AGENT_V5_CANDIDATE_20261123.md`, 산출물 `local_data/raw_results/real_limo_integ_claude_20261122/deploy_candidate_agent_v5/`, 서버 계약 프로브 `.../v5_candidate_01/contract_probe.py`.
- v5b candidate: `local_data/raw_results/real_limo_integ_claude_20261122/deploy_candidate_agent_v5b/`, 빌드 기록 `v5b_build_01/`.

## G2 병합 (2026-11-23): origin/main `ea2d9632` 판의 이 문서에만 있던 줄
> 통합 브랜치(v5b) 판을 기준으로 두고, origin/main 판에만 있던 줄을 원래 순서대로 아래에 보존한다. 날짜가 이른 기록(11-14~11-18)이며 위 본문의 최신 상태와 다르면 위 본문이 우선한다. 원문: `git show ea2d9632:CODE_INDEX.md`.

`fleet_control/`은 **PR #9(`b350efa`)로 `main`(`a8132c7`)에 병합 완료**된 단일 정본입니다(공개 비런타임 **173개** 파일; 로컬 통합 브랜치 `feat/fleet-control-unified` `ed53d81` 및 기능 스냅샷 4개 보존). 외부 실험 스크립트 3개(`lf_setup.sh`, `lf_env.sh`, `offline_registration_test.py`)는 후속 PR-D 대상이며, `run_recorder.py`는 **PR #10(`a8132c7`)**으로 저장소 내 `local_data/raw_results/limo_runs/` 기본 경로(신규 clone 포함, 저장소 밖은 `~/limo_runs/`)가 반영되었습니다.
| **중앙 관제 서버 (`fleet_server`) 기본본** | `main` (PR #9, 로컬 통합본 `ed53d81`) `fleet_control/server` | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/server` | `fleet_server/{__main__.py, http_api.py, service.py, db.py, auth.py, ownership.py, recovery.py, static/}` (`run_checks` 46/46, `test_ownership` 37/37, `test_recovery` 14/14 PASS) | `Capstone1_limo/fleet_control/`은 **런타임 `data/{auth.json,fleet.db}`와 `secrets/*.token`을 보유**하므로 삭제 금지 (Git 비추적 유지) |
| **관제 서버 + 실시간 영상 패널 (`video_panel.js`)** | `main` (PR #9, 로컬 통합본 `ed53d81`) `fleet_control/server` (스냅샷: `exp/video-ai-preverify` `e4dd598`) | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/server` | `--video-origin` 지원, `static/video_panel.js`, `docs/DASHBOARD_GUIDE.md`, `docs/REAL_LIMO_VIDEO_TEST.md` | `experiments/real_limo_field_20261113/laptop/lf_setup.sh` (로컬 전용·PR-D 보류)가 `$REPO_ROOT/fleet_control` 상대경로를 참조하도록 정정됨 |
| **Robot Agent (`limo_robot_agent`) + Open-RMF Adapter (`limo_rmf_adapter`) + Gazebo RMF** | `main` (PR #9, 로컬 통합본 `ed53d81`) (스냅샷: `codex/rmf-gazebo-integration` `0dc7ef9`) | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/{agent,rmf_adapter,gazebo}` (실차 온보드: `~/limo_field_agent_ws`) | `motor_policy.py`(`SIM_GATED`), `adapter_node.py`, `config/fleet_gazebo.json`, `gazebo/localized_sim.launch.py`, `tests/run_gazebo_checks.py` (Gazebo RMF 69/69 PASS) | `experiments/real_limo_field_20261113/laptop/lf_env.sh` 및 `experiments/real_rmf_20261113/tools/offline_registration_test.py` (로컬 전용·PR-D 보류)가 `Capstone1_limo/fleet_control` 상대경로 참조 |
| **통합 시뮬레이션 (`integrated_sim`)** | `main` (PR #9, 로컬 통합본 `ed53d81`) (스냅샷: `exp/integrated-sim-20261113` `29818e8`) | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/integrated_sim` | `integrated_env.py`, `integrated_sim.launch.py`, `run_integrated.py`, `fleet_integrated.json`, `RESULTS.md` (**Gazebo + CUDA YOLO 통합 시뮬 전용**, 풀스택 재실행 미검증) | RMF + Gazebo + Nav2 + Video/YOLO 통합 실행기 |
| **시각화 대시보드 (`dashboard` / `dashboard_lab` / `dashboard_v2_design` / `rmf_web_research`)** | `main` (PR #9, 로컬 통합본 `ed53d81`) (스냅샷: `codex/dashboard-visualization` `bc3df5e`) | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/{dashboard,dashboard_lab,dashboard_v2_design,rmf_web_research}` | - `dashboard/`: `test_visualization` 31/31, JS 18/18 PASS<br/>- `dashboard_lab/`: `test_lab.py`는 `aiohttp` 미설치로 미검증<br/>- `dashboard_v2_design/`, `rmf_web_research/` | `dashboard*/README.md` 등 문서 4곳 예시 경로도 `Capstone1_limo` 기준으로 정정됨 |
## 2026-11-18 Depth TF와 수동 잠금 프로파일
`limo_depth_obstacles/depth_obstacle_filter_node.py`는 촬영 시각의 변환을 짧게 재시도하며 상태에 대기 통계를 포함한다. `limo_integrated_auto_mapping_real.launch.py`의 `manual_mapping_probe`는 기본 비활성이고, 활성 시 모터 잠금과 Frontier 비활성을 요구한다. 이 경로는 비주행 진단용이다.
### Frontier 후보 및 Fleet Coordinator
`explore_lite`의 `fleet_frontier_candidates`는 기존 Frontier 검색 결과를 `/fleet/frontier_candidates`에 발행하며 Nav2 Goal을 직접 제출하지 않는다. `frontier_fleet_coordinator.py`는 Fleet 임무 및 Agent 종료 증거를 연결하는 후보 코드다. 현재 Agent 실행 경로와 웹 AUTO에는 연결되지 않았으며 실차 주행은 미검증이다.
### 고정 지도 저장
`capstone_ws/src/limo_capstone/scripts/auto_mapping_experiment_manager.py`의 `save-map`은 `/map`의 한 스냅샷으로 PGM/YAML과 식별 메타데이터를 저장한다. `tests/map_roundtrip_ros.py`는 격리 ROS의 Nav2 저장·재로드 회귀 시험이다. posegraph 직렬화는 별도 요청이다.
11-18 개발에서 `explore/src/explore.cpp`의 LOCAL_FRONTIER 취소는 수락된 Goal 핸들을 사용한다. `ros_adapter.py`는 다른 NavigateToPose 활성 Goal 또는 Action 서버 수 이상에서 Fleet Goal을 거부한다. 지도·RGB·Depth 동시 모니터링 진입점은 `fleet_control/dashboard/README.md`다. 영상 호스트의 `/api/v1/status`, `/api/v1/frame.jpg`와 Fleet 상태 API는 별개의 서비스다.
웹 모의 수동 입력 진입점은 Fleet 화면의 `static/gamepad_panel.js`, 서버 `/robots/{id}/manual-bench`와 `/agent/manual-bench`, Agent `manual_web_bench.py`다. `ARCHITECTURE.md`의 11-18 항목에 ROS 노드 목록과 현재·제안 구조도를 정리했다.
### 웹 운용 통합 (모의)
- `fleet_control/server/fleet_server/mapping.py`와 `db.py`는 별도 Mapping/Bag 상태·요청 저널을 관리한다. Operator는 `/robots/{id}/mapping`, `/robots/{id}/bag`를 사용하고, 명시적 모의 worker는 `/agent/operation`을 사용한다.
- `static/operations_panel.js`는 선택 로봇·Lease·Mapping·Bag을 표시한다. `static/gamepad_panel.js`는 장치별 보정과 버튼을 브라우저 로컬 프로필로 관리한다. 지도와 RGB는 Fleet 화면의 기존 패널, Depth는 기존 읽기 전용 시각화 콘솔을 사용한다.
- `fleet_control/tests/test_mapping_session.py`는 독립 Bag 상태, 요청 재전송, 결과 유실, 외부 Goal 및 real Agent 차단을 시험한다.
`limo_robot_agent/bag_worker.py`는 Fleet Bag 요청을 기록하고 `owned_bag.py`를 통해 자신이 시작한 rosbag2 프로세스만 관리한다. `isolated_ros_bag`은 mock·simulated·로컬 ROS에 한해 명시적으로 켠다.
`owned_bag.py`는 소유한 PID·시작 tick·PGID를 확인해 신호를 보낸다. `bag_worker.py`의 자원 한도는 독립 Bag Supervisor가 감시한다.
`agent_node.py`의 `isolated_ros_bag`는 기본 비활성이며 격리 mock 모드에서 Supervisor를 사용한다.
### 격리 실제 ROS 작업자 및 통합 Launch
- `fleet_control/agent/limo_robot_agent/limo_robot_agent/`의 `mapping_worker.py`는 SLAM·Frontier 및 지도 저장, `bag_supervisor.py`는 개별 기록 세션, `velocity_arbiter.py`는 Gate 앞 속도 선정, `camera_preview.py`는 이미지 수집을 담당한다.
- `fleet_control/server/fleet_server/`의 `mapping.py`는 Mapping Goal 예약과 작업 상태, `static/camera_panel.js`와 `static/operations_panel.js`는 운영 화면 연동을 담당한다.
- `capstone_ws/src/limo_fleet_bringup`은 기존 ROS 패키지를 재사용하는 잠금 전용 Launch의 진입점이다. 토픽·서비스·Action 매핑은 [ISOLATED_E2E.md](fleet_control/docs/ISOLATED_E2E.md)를 참조한다.

## 2026-11-23 G2 통합 진입점 (G2 worktree, 미커밋)
- Frontier R1 권한: `safety_protocol.frontier_authority_fault`, `ros_adapter.start_frontier`/`_do_frontier`(fence)/`_monitor_active_goal`; 시험 `fleet_control/tests/test_g2_frontier_authority_20261123.py`.
- Bag: `fleet_control/server/fleet_server/bag_operation.py`(`bag_status`, `request_bag`, `bag_worker_operation`), Agent `bag_worker.py`·`bag_supervisor.py`·`owned_bag.py`; 시험 `test_g2_bag_operation_20261123.py`, main `test_bag_worker.py`·`test_bag_supervisor.py`.
- 카메라: Agent `camera_preview.py`, 서버 `service.py::ingest_camera`·`camera_frame`, 웹 `static/camera_panel.js`; Field Ops 탭 `static/{operations_panel,gamepad_panel,legacy_panels_shim}.js`.
- Bringup: `capstone_ws/src/limo_fleet_bringup/launch/isolated_locked.launch.py`, `safety.motion_watchdog`; 시험 `test_g2_bringup_chain_20261123.py`.
- 보고: `fleet_control/docs/G2_INTEGRATION_20261123.md`.

## 2026-11-23 현장 준비 진입점 (G2 worktree, 미커밋)
- 문서 `fleet_control/docs/FIELD_READINESS_20261123.md`; 도구 `scripts/{field_laptop_up_20261123.sh,field_preflight_20261123.py,atomic_swap_20261123.py}`; 프로브 `tests/probe_field_chain_20261123.py`; 시험 `tests/{test_atomic_swap_20261123.py,test_field_preflight_20261123.py,test_gamepad_bench_payload.cjs}`; 웹 `server/fleet_server/static/legacy_panels_shim.js`.

- Gate 입력 출처 검증(G2, 미커밋): `capstone_ws/src/limo_capstone/limo_capstone/stall_detector_node.py`, `limo_fleet_bringup/safety.py`, 시험 `fleet_control/tests/test_gate_input_source_20261123.py`, 문서 `fleet_control/docs/GATE_INPUT_SOURCE_G2_20261123.md`.

- 속도 제한(G2, 미커밋): Agent `fleet_control/agent/limo_robot_agent/limo_robot_agent/speed_limits.py`(+`velocity_arbiter.py` clamp), 서버 `fleet_control/server/fleet_server/speed_limits.py`, 계약 `fleet_control/docs/SPEED_LIMIT_API_20261123.md`, UI 연결 `SPEED_LIMIT_UI_INTEGRATION_20261123.md`, 시험 `tests/test_speed_limits_{agent,server}_20261123.py`, `tests/probe_speed_clamp_20261123.py`.

- Watchdog 속도 clamp(G2, 미커밋): `fleet_control/agent/limo_robot_agent/limo_robot_agent/{speed_clamp,watchdog_speed_link}.py`, `motion_watchdog.py`(연결부), 문서 `fleet_control/docs/WATCHDOG_SPEED_CLAMP_20261123.md`, 시험 `tests/test_watchdog_speed_{clamp,link}_20261123.py`, `tests/real_launch_chain_faults_20261123.py`(42).

- Phase C 통합(G2 미커밋): 문서 `fleet_control/docs/PHASE_C_INTEGRATION_20261123.md`, `REAL_LIMO_LOCKED_PREFLIGHT_20261123.md`, `UI_INTEGRATION_PATCH_20261123.diff`; 서버 `fleet_server/speed_limits.py`(Watchdog 증거 검증), 시험 `tests/e2e_web_watchdog_speed_limit_20261123.py`, `tests/e2e_server_outage_speed_limit_20261123.py`; `scripts/field_preflight_20261123.py` F 항목; bringup `speed_clamp` 인자(`FLEET_FLAGS`).

- 실차형 모드 선택(G2 미커밋): Agent `mode_policy.py`, `mode_selector.py`, `motion_watchdog.py`(모드 게이트), `watchdog_speed_link.py`; 서버 `operating_mode.py`; 문서 `fleet_control/docs/REAL_AGENT_MODE_SELECTION_20261124.md`; 시험 `tests/test_{mode_selector_server,real_mode_selector,watchdog_mode_gate}_20261124.py`, `tests/e2e_real_agent_{modes,navigation}_20261124.py`.
- Gate 재시작 추적·서버 ACTIVE 검증(G2 미커밋): `fleet_control/tests/probe_gate_restart_20261125.py`; 서버 `fleet_server/operating_mode.py`(`mode_motion_status`, `_mode_active_blocker`), `service.py`(`mode_motion`); 시험 `tests/test_mode_selector_server_20261124.py`(ACTIVE 검증), `tests/test_watchdog_mode_gate_20261124.py`(해제 후 재개); 문서 `REAL_AGENT_MODE_SELECTION_20261124.md` §7.
- 건식 임무·조이스틱 재무장·clamp 비교(G2 미커밋): Agent `agent_node.py`(`real_nav_dry_run`)·`core.py`(DRY_RUN_MISMATCH)·`ros_adapter.py`(`_locked_motion_proof`)·`motion_watchdog.py`(재무장); 서버 `service.py`·`http_api.py`(`/dry-run-missions`)·`safety.py`(`locked_motor_proof`)·`operating_mode.py`·`navigation.py`; 스택 `tests/mode_e2e/stack_up_real_shape.sh`; E2E `tests/nav_e2e/run_dry_run_missions_e2e_20261126.py`; 단위 `tests/test_dry_run_missions_20261126.py`; 비교 `tests/compare_auto_clamp_20261126.py`; 문서 `fleet_control/docs/REAL_NAV_DRY_RUN_20261126.md`.
- Gate 생성 증거·reset 게이팅·건식 복구(G2 미커밋): Gate `capstone_ws/src/limo_capstone/limo_capstone/stall_detector_node.py`(boot_id·epoch·reset_readiness), Watchdog `motion_watchdog.py`(`idle_zero_period_s`, `sl_diag`), 서버 `safety.py`·`ownership.py`, Agent `ros_adapter.py`; 단위 `tests/test_gate_generation_evidence_20261127.py`; E2E `tests/nav_e2e/run_dry_run_missions_e2e_20261126.py`(d06 연속 흐름, d07 reset 정책); 문서 `fleet_control/docs/GATE_EVIDENCE_AND_DRY_RUN_RECOVERY_20261127.md`.

## 2026-11-24 최종 통합 기준선 추가 색인 (`integration/capstone1-field-release-20261124`)
- **Web UI 지도·속도·통합 필드 (Antigravity + q10 패치)**: `fleet_control/server/fleet_server/static/{app.js,index.html,manual_control_panel.js,map_view.js,operating_mode_panel.js,speed_control_panel.js,state_helpers.js,style.css}` (`state_helpers.js` 헬퍼: `classifyModeMotion`, `classifyDryRun`, `classifyGateEvidence`, `explainApiError`, `missionEndpoint`), 시험 `fleet_control/tests/{test_web_map_and_speed_ui_20261123.cjs,test_web_map_speed_chrome_e2e_20261123.cjs,test_integration_ui_fields_20261128.cjs}`, 문서 `fleet_control/docs/WEB_MAP_SPEED_INTEGRATION_PROPOSAL_20261123.md`.
- **Navigation O6 안정화 및 순찰 검증 (Antigravity)**: `capstone_ws/src/limo_capstone/config/integrated_nav2_lidar_only_slow_params.yaml` (`use_rotate_to_heading: true`, `PoseProgressChecker`), `capstone_ws/src/limo_capstone/limo_capstone/lidar_scan_matcher_odom_node.py` (`keyframe_threshold`/`trim_percentile` 파라미터 노출), `fleet_control/tests/mode_e2e/stack_up.sh` (G2+Nav 병합본), `fleet_control/tests/nav_e2e/{run_nav_e2e.py,run_nav_patrol_7scenarios_20261124.py,run_dry_run_missions_e2e_20261126.py}`, `fleet_control/tests/gazebo_web/nav2_d3_targeted.py`, 문서 `fleet_control/docs/NAV2_STABILIZATION_AND_PATROL_20261124.md`.
- **Agent Development Harness v0 / M0 (Antigravity)**: `scripts/agent_harness/{preflight.py,scopes.json,REPORT_TEMPLATE.md,README.md,M1_M2_ROADMAP.md,test_preflight.py}` (Python 표준 라이브러리 전용 읽기 전용 개발 Preflight CLI 및 단위 시험 18종).
