# CURRENT_STATUS.md — 영역별 구현 및 검증 수준 현황

> **기준일**: 2026-11-14  
> `AGENTS.md` §5.2에 따라 **구현 완료**와 **검증 수준(실차 / Gazebo / 합성·오프라인 / 미검증 / 제안)**을 엄격히 구분합니다.  
> 시뮬레이션(Gazebo) 또는 합성 하네스 통과 결과를 실차 검증으로 해석하지 마세요.

---

## 1. 검증 수준 범례

- **실차 검증 (Real Robot Verified)**: 물리 LIMO 로봇에서 실행해 로그·측정 기록을 남긴 항목 (모터 잠금 시험과 실제 구동 시험을 명시적으로 구분)
- **Gazebo 검증 (Gazebo Verified)**: Ignition Fortress 물리 시뮬레이션에서 실제 노드·센서·제어 루프를 기동해 확인한 항목
- **합성·오프라인 검증 (Synthetic / Offline Verified)**: 모의(Mock) 액션 서버, 합성 센서 주입 하네스, 또는 저장된 실차 rosbag 재생으로 확인한 항목
- **미검증 (Unverified)**: 코드·설정은 존재하나 해당 환경에서 아직 실행·확인하지 않은 항목
- **제안 (Proposed)**: 설계 문서나 diff 패치만 존재하고 정식 코드·운용에 반영되지 않은 항목

---

## 2. 영역별 구현 및 검증 상태 총괄표

### 2.1 ROS 2 온보드 정식 코어 (`capstone_ws/src/`) 및 실차 배포 번들 (`real_auto_mapping_ready_20261113`, `real_depth_slow_20261113`)

| 기능 / 구성요소 | 기준 경로 (실차 배포본 구분) | 구현 상태 | 실차 검증 | Gazebo 검증 | 합성·오프라인 검증 | 실패 기록 · 한계 및 미검증 사항 |
|---|---|---|---|---|---|---|
| Safety Gate 단일 명령 경로 (`stall_detector_node`) | [capstone_ws/src/limo_capstone](capstone_ws/src/limo_capstone) (실차: 로컬 미추적 `real_auto_mapping_ready_20261113`, `real_depth_slow_20261113`) | 구현 완료 | **모터 잠금 검증** (`/cmd_vel` 발행자 0, `/cmd_vel_locked` 1개 확인) | **검증 완료** (자동 맵핑 및 RMF 시뮬) | **검증 완료** (기존 단위 하네스 86/86 PASS) | 물리 E-Stop·RC 비상정지 채널 미검증 ([docs/REAL_ROBOT_STATUS.md](docs/REAL_ROBOT_STATUS.md) — `main` 브랜치; 로컬 `<WORKSPACE_PARENT>/Capstone1_limo_dashboard/docs/REAL_ROBOT_STATUS.md`) |
| `/cmd_vel` 중단 시 하위 MCU 정지 시간 | `limo_base` + STM32 | 펌웨어 기본 | **무부하 벤치 제한 검증** (0.10 m/s 직진 중단 후 정지까지 0.37~0.38 s, 3회) | 해당 없음 | 해당 없음 | 지상 주행·회전·고속 정지 거리 미측정, 주행 중 케이블 강제 분리 시험 금지 |
| LiDAR 스캔 정규화 (`limo_scan_normalizer`, 244빔) | [experiments/safety_gate_verification_20261110/variants/scan_normalizer_ws/src/limo_scan_normalizer](experiments/safety_gate_verification_20261110/variants/scan_normalizer_ws/src/limo_scan_normalizer) 및 실차 번들 | 구현 완료 | **모터 잠금 수동 이동 검증** (1.06 m 수동 이동 시 지도 701→7,531셀 성장, Karto 경고 0건) | **검증 완료** | **오프라인 bag 재생 검증** | 정규화 OFF 시 0.92 m 이동 동안 685셀 고정(경고 5건) 실패 확인 (`docs/scan-normalizer-locked-test-20261112` `09d742c` 브랜치 및 로컬 `docs/dev_notes/2026_11_12_실차_스캔정규화_배포와_모터잠금_검증.md`) |
| LiDAR-only Frontier 자동 맵핑 | 실차 배포본(로컬 미추적): `experiments/real_auto_mapping_ready_20261113` | 구현 완료 | **부분 구동 검증 (`run_010`)**: 0.15 m/s로 약 110초 주행, 지도 1,080→30,430셀 성장 후 `FAILSAFE_ICP_DEGRADED`로 안전 정지·지도(`map_001`,`map_002`) 저장 | **검증 완료** (`auto_mapping_final_20261112`: 6회 중 3회 98~99% 완주, 2회 82~85% 조기 종료, 1회 일시적 Depth stale 정지) | 해당 없음 | - 실차 `run_008`에서 충전기 분리 직후 USB 재연결(`wheel_odom_stale`)로 구동 전 차단<br/>- 실차에서 Frontier 탐색 자연 완주(목표 소진)는 미검증; `map_002` 약 13.8° 회전 (로컬 미추적 `experiments/real_limo_field_20261113/FIELD_RESULTS_20261113.md`) |
| 3D Depth 장애물 필터 (`limo_depth_obstacles`) + Costmap 반영 | 정식: [capstone_ws/src/limo_depth_obstacles](capstone_ws/src/limo_depth_obstacles)<br/>실차 배포본(로컬 미추적): `experiments/real_depth_slow_20261113` | 구현 완료 | **모터 잠금 정지 검증 (`run_012~014`)**: 화이트보드 하단 프레임(LiDAR 미검출)이 local costmap에 333개 lethal/inscribed 셀로 마킹됨 | **검증 완료** | **검증 완료** | **실차 Depth 포함 자율 주행(`--drive --depth true`)은 미검증**; 실차 Depth 스케일(`~0.91~0.93×LiDAR`), 지터(최대 680 ms vs 타임아웃 0.75 s), 소크 중 센서 정지 현상 미해결 (로컬 미추적 `experiments/real_depth_lidar_mapping_20261113/STATUS.md`) |

---

### 2.2 안전 체인 실험 수정 후보 (`experiments/safety_chain_fix_20261113/` — 노트북 전용 미추적, 실차 미배포)

| 버전 / 기능 | 로컬 미추적 경로 | 구현 상태 | 실차 검증 | Gazebo 검증 | 합성·오프라인 검증 | 실패 사례 · 핵심 결과 및 남은 한계 |
|---|---|---|---|---|---|---|
| `baseline` (`real_depth_slow_cm_20261113` 사본) | `experiments/safety_chain_fix_20261113/baseline` | 기준선 보존 (실차 미배포) | **미검증** (주행 안 함) | `office01/02`: 탐색 약 27초(1.4~4.0 m)에 TinyRobot1 인근 교착 | `run03`: PASS 15, **FAIL 12**, 측정 10 (Gate 86/86 PASS) | Depth 3D TF 축 불일치, 반복 stamp 미감지, 고장 후 자동 재출발 등 12건 결함 확인 |
| `fixed` (1차 안전 체인 수정본) | `experiments/safety_chain_fix_20261113/fixed` | 실험 폴더 내 구현 (실차 미배포) | **미검증** | `office02/03`: 기억 노드 종료 시 Gate 즉시 차단 확인; **단, 일반 탐색 시 2.18~4.03 m 주행 후 기억 셀로 인한 교착 지속** | `run04`: **PASS 28, FAIL 0**, 측정 9 (Gate 86/86 PASS) | 로컬 미추적 `experiments/safety_chain_fix_20261113/README.md` 및 `docs/DEADLOCK_ANALYSIS.md` 참조 |
| `fixed2` (장애물 기억·해제 정책 v2 + Costmap `memory_layer` 후보) | `experiments/safety_chain_fix_20261113/fixed2` | 실험 폴더 내 구현 (실차 미배포, 정식 미병합) | **미검증** | `office03`:<br/>- 성공: `explore_rep2` **16.22 m**, `explore_rep3` **18.48 m**, `odom_tf_loss_recover` **20.83 m**, `memory_hang_recover_rep1` 8.85 m, `low_box_rep2` 3.33 m<br/>- **실패/제한 기록**: `explore_rep1`(반경 `0.22/0.40` 시험 중 26.5초·1.55 m에서 frontier 소진 정지 → 기각 후 `0.18/0.30`으로 복귀), `low_box_rep1`(`sensor_normalizer_node` exit `-2` 및 57.2초 `normal_Stop` 정지) | `run05`: **PASS 28, FAIL 0**, 측정 9 (`odom_tf_lag_0.28`에서도 모니터 출력 p99 0.054 s) | - 사각 내 확정 셀이 footprint 안에 놓이면 운영자 해제(`/collision_monitor/clear_memory_near`) 필요<br/>- 코너 `0.236 m` 접근 시 제자리 회전 차단, 7 m 주행 후 F4 ICP 드리프트 미보정<br/>- 아직 `capstone_ws/src` 및 실차 번들에 미이식 (로컬 미추적 `experiments/safety_chain_fix_20261113/docs/MEMORY_POLICY_V2.md`) |

> **실험 데이터 아카이브(Option D) 백업 현황 ([VALIDATION.md](VALIDATION.md) §2.5, §2.7)**: `run03~06`은 Google Drive 원격 복구 검증 완료(`run03~05` 과거 중간 소스 4건 미확보 한계 유지). `Office04`(`office04_20261115`, 25회)는 로컬 전체 복원 및 원격 다운로드 무결성 검증 완료·원격 전체 복호화 `skipped_by_user`(`tools/run_office2.py` 5회 미확보 유지). `Office03`(`office03_20261114`, 12회, 원본 113개)은 로컬 복원(225/225, SQLite·`ros2 bag info` 12/12)·Google Drive 8개 파일 업로드·원격 해시 7/7 검증 완료, 원격 전체 복호화 `skipped_by_user`로 `remote_backup_integrity_verified` 마감(실행 당시 코드 해시 연결 기록 미확보).

---

### 2.3 중앙 관제 · Robot Agent · Open-RMF (`fleet_control/`, PR #9 `main` 병합 완료)

| 기능 / 구성요소 | Git 브랜치 기준 / 로컬 워크트리 경로 | 구현 상태 | 실차 검증 | Gazebo 검증 | 합성·오프라인 검증 | 실패 기록 · 한계 및 미검증 사항 |
|---|---|---|---|---|---|---|
| 중앙 관제 서버 (`fleet_server`) + 제어권 Lease + 복구 | 브랜치: `main` (`a8132c7`, PR #9 `b350efa` 병합 · 통합본 `ed53d81` 보존)<br/>로컬: `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/server` | 구현·통합 완료 | **모터 잠금 연동 검증** (`18190` 서버에 실차 Agent 접속, 상태·지도 API 업로드 및 `map.png` HTTP 200 응답 확인) | **검증 완료** (이전 기준선 `run_gazebo_checks.py` **69/69 PASS**; PR #9 격리 검증에서는 외부 `$HOME/rmf_isolated`/Gazebo 런타임 미비로 미재실행) | **검증 완료** (`run_checks.py` 46/46, `test_ownership.py` 37/37, `test_recovery.py` 14/14, `test_dedupe.py` 14/14, `test_agent_recovery.py` 2/2, `test_cancel_results.py` 14/14) | - 과거 `run_checks.py` 1회 원인 미상 42/46 실패 기록 있음 (`419e798` 브랜치 `fleet_control/docs/VALIDATION.md`)<br/>- 실차 시험 시 브라우저 UI 화면 육안 확인은 아직 사용자 미회신 (`experiments/real_limo_field_20261113/FIELD_RESULTS_20261113.md`) |
| LIMO Robot Agent (`limo_robot_agent`) | 브랜치: `main` (`a8132c7`, PR #9 `b350efa` 병합 · 통합본 `ed53d81` 보존)<br/>로컬: `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/agent` (실차: `~/limo_field_agent_ws`) | 구현·통합 완료 | **모터 잠금 상태·지도 보고 검증** (신규 Agent를 실차 `~/limo_field_agent_ws`에 오버레이 빌드 후 `409 agent_session_mismatch` 해결 확인) | **검증 완료** (`mode=sim`, `SIM_GATED` 주행·취소·복구) | **검증 완료** (`test_sim_motor_policy.py` 12/12 등) | **실차에서 관제/RMF를 통한 실제 이동 임무 실행은 미검증** (`allow_missions=false` 유지) |
| Open-RMF Fleet Adapter (`limo_rmf_adapter`) | 브랜치: `main` (`a8132c7`, PR #9 `b350efa` 병합 · 통합본 `ed53d81` 보존)<br/>로컬: `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/rmf_adapter` | 구현·통합 완료 | **미검증** (`lf_t2_rmf.sh`는 Gazebo 그래프 기준이라 실차 좌표 `(20,10)`에서 등록 거부됨; `experiments/real_rmf_20261113` 단독 오프라인 등록 시험 40/41 — §4 unconfirmed 타이밍 1건 미해결) | **검증 완료** (이전 기준선 `run_gazebo_checks.py` **69/69 PASS**; PR #9 격리 검증 미재실행) | **부분 검증** (`run_rmf_checks.py` 47/51 — §7 mock 타이밍 레이스, `test_runner_policy.py` 33/33) | - `run_rmf_checks.py` §7(47/51)은 원본·통합본 공통으로 `wp_e` 근거리 재지정에 따른 mock 조기 완료 레이스 확인<br/>- Adapter 재시작 시 이전 RMF 작업이 `underway` 고아 상태로 남는 한계 미해결 |

---

### 2.4 대시보드 · 실시간 영상 AI (`fleet_control/`, PR #9 `main` 병합 완료 · 공개 173개 파일)

| 기능 / 구성요소 | 로컬 단일 기준 경로 (`feat/fleet-control-unified` `ed53d81`) | 구현 상태 | 실차 검증 | Gazebo 검증 | 합성·오프라인 검증 | 실패 기록 · 한계 및 미검증 사항 |
|---|---|---|---|---|---|---|
| 관제 대시보드 영상/AI 패널 (`video_panel.js`) + CUDA YOLO11n | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/{server,integrated_sim}` | 구현·통합 완료 | **모터 잠금 검증** (DaBai RGB → Tailscale TCP `18191` → 노트북 RTX 5060 YOLO11n → 대시보드 HTTP `18192`, 10 fps) | 과거 `final4` 12/13 검증; 통합 후 `integrated_sim` 풀스택은 **미검증** (CLI/문법 PASS) | **검증 완료** (`run_checks.py` 46/46) | 실차 주행 중 영상 스트리밍 동시 부하 미검증; 통합 후 `integrated_sim` 풀 GPU/Gazebo 실행은 미검증 |
| 읽기 전용 시각화 대시보드 (`fleet_control/dashboard`) | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/dashboard` | 구현·통합 완료 | **15초 제한 읽기 전용 분리 검증** (`bridge_node.py` + `readonly_rgb.py` 5 fps) | 해당 없음 (저장 Gazebo 프레임 재생만 사용) | **검증 완료** (통합 후 `test_visualization.py` 31/31, JS 12/12 PASS) | 실차 정지 상태 시험으로 `/plan`, `/local_plan`, Frontier 선택 목표는 `UNAVAILABLE`; Depth+RGB 단일 통합 루프 미검증 |
| 고성능 스트리밍 연구소 (`dashboard_lab`) & v2 설계 (`dashboard_v2_design`) | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/{dashboard_lab,dashboard_v2_design}` | 구현·통합 완료 (Lab) / 설계 완료 (v2) | **미검증** (실차 미연결, 이전 실차 기록 REPLAY만 확인) | 미실행 (저장 Gazebo 자산만 사용) | `frame_policy.test.mjs` **6/6 PASS**; `test_lab.py`는 `aiohttp` 부재로 통합 후 **미검증** | 과거 Lab 벤치 121 PASS(11.90 fps) 기록 유지; 통합 환경에서 `test_lab.py` 실행에는 `requirements-lab.txt` 격리 venv 필요 |
| 공식 `rmf-web` 독립 연구 (`rmf_web_research`) | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/rmf_web_research` | 격리 하네스 통합 | **미검증** | **미검증** | **Storybook 오프라인 fixture 확인** (포트 18410, TypeScript 빌드 PASS) | 원본 Storybook 애드온 충돌로 커스텀 설정 사용; `/tmp/capstone-rmf-web-official-20261113` 소스는 재부팅 시 소멸되므로 `prepare_frontend.sh` 재실행 필요 |

---

### 2.5 기록기 (`run_recorder.py`, PR #10 `main` 병합 완료)

- **PR #10(`a8132c7`)** 반영으로 `--root` > `LIMO_RUNS_DIR` > 저장소(`capstone_ws/src/limo_capstone/`) 내 `local_data/raw_results/limo_runs/`(신규 clone 포함) > 저장소 밖 `~/limo_runs/` 우선순위가 확정되었습니다.
- 격리 합성 시험 **16/16 PASS** (실제 `rosbag2` 기록 및 실차 온보드 실행은 **미검증**).

---

## 3. 실차 구동 재개 전 필수 차단 요인 (Strict Blockers)

1. **전원 돌발 차단(4회) 원인 미규명**: 2026-11-13 실차 시험 중 전원이 4회 갑자기 차단되었으며, 충전기 분리·이동 시 베이스 USB(`usb 3-7.4`, `ttyUSB1→ttyUSB2`) 재연결로 `wheel_odom_stale` FAILSAFE(`run_008`)가 발생했습니다. 원인 규명 및 완충 상태 소크 시험(`power_logger.py`) 통과 전까지 `--drive` 자율 주행을 금지합니다.
2. **Depth 센서 거리 스케일 및 지터**: 실차 Depth 거리가 LiDAR 대비 약 `0.91~0.93배`로 짧게 측정되고, 필터 출력 주기 지터(최대 간격 680 ms vs 타임아웃 0.75 s) 및 소크 중 IR 투사 정지 현상이 미해결 상태입니다.
3. **물리적 비상 정지(E-Stop) 미검증**: 모든 실차 구동 시험은 사용자가 전원 스위치 옆에 입회하고 단계별 승인을 얻은 감독 시험으로만 제한됩니다.
