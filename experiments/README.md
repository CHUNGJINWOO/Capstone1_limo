# 실험 인덱스

기준일: 2026-11-11. 이 폴더의 기록이 어디까지 검증되었는지를 먼저 구분한다. 파일이 있다고 해서 해당 기능이 실차에서 동작한다는 뜻은 아니다(`docs/REAL_ROBOT_STATUS.md`).

## 1. 이 브랜치에 들어 있는 것과 보존 브랜치에만 있는 것

정리 과정에서 실험 자료를 삭제하지 않았다. 아래 표의 "이 브랜치" 열은 `main`에 병합될 때 저장소에 들어가는 파일이고, 나머지 전체 자료(시계열, 지도 이미지, 원본 probe JSON, `ws/src` 스냅샷, 코드 사본)는 보존 브랜치 `docs/limo-real-robot-progress-20261111`(커밋 `c804241ac9af`)에 그대로 있다. 디렉터리별 파일 수와 위치는 `PRESERVED_SNAPSHOTS.tsv`에 있다.

| 디렉터리 | 내용 | 검증 수준 | 이 브랜치 |
|---|---|---|---|
| `limo_integrated_auto_mapping_20261108/` | 통합 자동 맵핑 회귀 감사, 장기 맵핑 보고서 | 시뮬레이션 | 이미 `main`에 있음 |
| `simulation_benchmarks_20261024/` | MC관 지도·이동 장애물 시뮬레이션 벤치마크 | 시뮬레이션 | 이미 `main`에 있음 |
| `safety_gate_verification_20261110/` | 안전 게이트 검증, 자동 맵핑 시뮬레이션 평가, SLAM 스캔 문제 조사, 배포 계획·감독 실차 시험 기록 | 시뮬레이션 + 실차(소수 시행) + 오프라인 | 보고서·스크립트·변형·제안 diff 선별 포함 |
| `limo_nav2_closed_loop_verification_20261107/` | 상태 추정기 폐루프 검증(Nav2) | 시뮬레이션 | 보존 브랜치에만 |
| `limo_state_estimation_benchmark_20261107/` | 상태 추정 알고리즘(B0~F6) 벤치마크와 녹화 | 시뮬레이션 | 보존 브랜치에만 |
| `limo_final_deployment_prep_20261108/` | F4/F5 재평가, 배포 준비 자료 | 시뮬레이션 | 보존 브랜치에만 |
| `lounge_failure_investigation_20261107/` | 라운지 환경 주행 실패 재현·원인 분석 프레임워크 | 시뮬레이션 | 보존 브랜치에만 |
| `failure_measurement_audit_20261105/`, `failure_limits_verification/` | 실패 계측 감사, 한계 검증 | 시뮬레이션(디렉터리 이름 기준, 이번 정리에서 내용 재검토 안 함) | 보존 브랜치에만 |
| `gazebo_obstacle_mapping/`, `depth_mapping_20261107/`, `inscribed_start_verification/`, `auto_resume_verification/`, `auto_mapping_verification/`, `mapping_goal_journal_20261105/` | 장애물 맵핑, 깊이 맵핑, 시작 위치 검증, 재개, 자동 맵핑 검증, 목표 기록 | 시뮬레이션(디렉터리 이름 기준, 이번 정리에서 내용 재검토 안 함) | 보존 브랜치에만 |
| `auto_mapping_action/`, `auto_mapping_python/` | 자동 맵핑 Action 시험본(C++/Python). 각 README에 "노트북에서만 수행, 실제 LIMO에는 복사·설치·실행하지 않았다"고 적힌 시험본 | 노트북 시뮬레이션 | 보존 브랜치에만 |
| `limo_onboard_trials/` | LIMO용 추가 시험본(middle / RPP / 복구 BT) 소스 안내 | 상태는 해당 README 참조(이번 정리에서 재검토 안 함) | 보존 브랜치에만 |

## 2. `safety_gate_verification_20261110/` 안내

| 하위 경로 | 내용 | 검증 수준 |
|---|---|---|
| `README.md`, `ANALYSIS_phase2_failure_causes.md`, `FOLLOWUP_proposals.md` | 구성·기준선, 실패 원인 분석, 후속 제안 | 시뮬레이션 / 제안 |
| `scripts/` | gate 단독 시험, 전체 sim 시험, 모니터·분석 스크립트 | 보고서 재현용 |
| `slam_scan_fix_20261111/` | SLAM 지도 갱신 불가 원인(LiDAR 스캔 점 수 불일치)과 정규화 해결안, 재생·비교 스크립트와 요약 JSON | 원인: 실차 기록+오프라인 / 정규화: **오프라인 검증, 실차 미적용** |
| `variants/scan_normalizer_ws/` | `limo_scan_normalizer` 패키지(/scan → /scan_slam, 244 beam) | 오프라인 검증 |
| `variants/explore_variant_ws/` | `explore_relaxed` | **실험용**, 기본 탐색기 아님(원본 `m-explore-ros2` 라이선스 파일 동봉) |
| `variants/*.launch.py`, `*.yaml` | 단일 변경 검증용 sim 변형 | 시뮬레이션 변형 |
| `estimator_study_20261111/` | 추정기 비교(F4 점대선 ICP 개선안 포함) | 시뮬레이션, 개선안은 **제안** |
| `explore_relaxed_eval/`, `post_analysis_20261111/`, `sim_settings_20261111/` | explore_relaxed 평가, 사후 분석, 시뮬레이션 설정 점검 | 시뮬레이션 |
| `deploy_plan_20261111/` | 배포 계획, 벤치 MCU 워치독 절차·결과, 감독 구동 시험 기록, `proposed/*.diff`(실차 launch·YAML 제안 diff) | 계획 + 실차(소수 시행). diff는 **제안**이며 일부만 적용되었음 |

## 3. 읽을 때 주의
- 기록은 작성 당시의 설정·환경에서 얻은 값이다. 정리 이후의 파일 이름·값(`docs/CONFIG_SIM_VS_REAL.md`)으로 다시 측정한 것이 아니다.
- 공개 정책에 따라 개인 노트북의 절대 경로가 들어 있던 diff 머리글과 트레이스백의 저장소 경로는 `<repo>`로 바꿨다. 측정값과 로직은 바꾸지 않았다. 과거 기록 일부에는 작성 당시 경로가 남아 있을 수 있으며 현재 환경의 실행 경로로 보지 않는다.
- `deploy_plan_20261111/proposed/`의 `real_launch_*.diff`가 가리키는 전체 launch 스냅샷(`real_launch_*.py`)과 `limo_vs_notebook.diff`, 배포 해시 목록은 보존 브랜치에만 있다.
- 원본 rosbag, 영상, 원본 로그·캡처는 `docs/PUBLICATION_POLICY.md`에 따라 저장소에 올리지 않는다.
- 문서가 가리키는 `ws/src`, `nodes/` 사본은 보존 브랜치에 있다. 이 브랜치의 실제 코드는 `capstone_ws/src/limo_capstone/`다.
