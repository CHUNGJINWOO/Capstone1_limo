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

### 2.3 중앙 관제 · Robot Agent · Open-RMF (`fleet_control/`)

| 기능 / 구성요소 | Git 브랜치 기준 / 로컬 워크트리 경로 | 구현 상태 | 실차 검증 | Gazebo 검증 | 합성·오프라인 검증 | 실패 기록 · 한계 및 미검증 사항 |
|---|---|---|---|---|---|---|
| 중앙 관제 서버 (`fleet_server`) + 제어권 Lease + 복구 | 브랜치: `feat/fleet-control-unified` (`ed53d81`)<br/>로컬: `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/server` | 구현·통합 완료 | **모터 잠금 연동 검증** (`18190` 서버에 실차 Agent 접속, 상태·지도 API 업로드 및 `map.png` HTTP 200 응답 확인) | **검증 완료** (`run_gazebo_checks.py` 69/69) | **검증 완료** (`run_checks.py` 46/46, `test_ownership.py` 37/37, `test_recovery.py` 14/14, `test_dedupe.py` 14/14, `test_agent_recovery.py` 14/14) | - 과거 `run_checks.py` 1회 원인 미상 42/46 실패 기록 있음 (`419e798` 브랜치 `fleet_control/docs/VALIDATION.md`)<br/>- 실차 시험 시 브라우저 UI 화면 육안 확인은 아직 사용자 미회신 (`experiments/real_limo_field_20261113/FIELD_RESULTS_20261113.md`) |
| LIMO Robot Agent (`limo_robot_agent`) | 브랜치: `feat/fleet-control-unified` (`ed53d81`)<br/>로컬: `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/agent` (실차: `~/limo_field_agent_ws`) | 구현·통합 완료 | **모터 잠금 상태·지도 보고 검증** (신규 Agent를 실차 `~/limo_field_agent_ws`에 오버레이 빌드 후 `409 agent_session_mismatch` 해결 확인) | **검증 완료** (`mode=sim`, `SIM_GATED` 주행·취소·복구) | **검증 완료** (`test_sim_motor_policy.py` 12/12 등) | **실차에서 관제/RMF를 통한 실제 이동 임무 실행은 미검증** (`allow_missions=false` 유지) |
| Open-RMF Fleet Adapter (`limo_rmf_adapter`) | 브랜치: `feat/fleet-control-unified` (`ed53d81`)<br/>로컬: `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/rmf_adapter` | 구현·통합 완료 | **미검증** (`lf_t2_rmf.sh`는 Gazebo 그래프 기준이라 실차 좌표 `(20,10)`에서 등록 거부됨; `experiments/real_rmf_20261113` 단독 오프라인 등록 시험 40/41 — §4 unconfirmed 타이밍 1건 미해결) | **검증 완료** (통합 후 `run_gazebo_checks.py` **69/69, 필수 15/15 PASS**) | **부분 검증** (`run_rmf_checks.py` 47/51 — §7 mock 타이밍 레이스, `test_runner_policy.py` 33/33) | - `run_rmf_checks.py` §7(47/51)은 원본·통합본 공통으로 `wp_e` 근거리 재지정에 따른 mock 조기 완료 레이스 확인<br/>- Adapter 재시작 시 이전 RMF 작업이 `underway` 고아 상태로 남는 한계 미해결 |

---

### 2.4 대시보드 · 실시간 영상 AI (`Capstone1_limo/fleet_control/` 로컬 통합 완료 · GitHub 미반영)

| 기능 / 구성요소 | 로컬 단일 기준 경로 (`feat/fleet-control-unified` `ed53d81`) | 구현 상태 | 실차 검증 | Gazebo 검증 | 합성·오프라인 검증 | 실패 기록 · 한계 및 미검증 사항 |
|---|---|---|---|---|---|---|
| 관제 대시보드 영상/AI 패널 (`video_panel.js`) + CUDA YOLO11n | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/{server,integrated_sim}` | 구현·통합 완료 | **모터 잠금 검증** (DaBai RGB → Tailscale TCP `18191` → 노트북 RTX 5060 YOLO11n → 대시보드 HTTP `18192`, 10 fps) | 과거 `final4` 12/13 검증; 통합 후 `integrated_sim` 풀스택은 **미검증** (CLI/문법 PASS) | **검증 완료** (`run_checks.py` 46/46) | 실차 주행 중 영상 스트리밍 동시 부하 미검증; 통합 후 `integrated_sim` 풀 GPU/Gazebo 실행은 미검증 |
| 읽기 전용 시각화 대시보드 (`fleet_control/dashboard`) | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/dashboard` | 구현·통합 완료 | **15초 제한 읽기 전용 분리 검증** (`bridge_node.py` + `readonly_rgb.py` 5 fps) | 해당 없음 (저장 Gazebo 프레임 재생만 사용) | **검증 완료** (통합 후 `test_visualization.py` 31/31, JS 12/12 PASS) | 실차 정지 상태 시험으로 `/plan`, `/local_plan`, Frontier 선택 목표는 `UNAVAILABLE`; Depth+RGB 단일 통합 루프 미검증 |
| 고성능 스트리밍 연구소 (`dashboard_lab`) & v2 설계 (`dashboard_v2_design`) | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/{dashboard_lab,dashboard_v2_design}` | 구현·통합 완료 (Lab) / 설계 완료 (v2) | **미검증** (실차 미연결, 이전 실차 기록 REPLAY만 확인) | 미실행 (저장 Gazebo 자산만 사용) | `frame_policy.test.mjs` **6/6 PASS**; `test_lab.py`는 `aiohttp` 부재로 통합 후 **미검증** | 과거 Lab 벤치 121 PASS(11.90 fps) 기록 유지; 통합 환경에서 `test_lab.py` 실행에는 `requirements-lab.txt` 격리 venv 필요 |
| 공식 `rmf-web` 독립 연구 (`rmf_web_research`) | `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/rmf_web_research` | 격리 하네스 통합 | **미검증** | **미검증** | **Storybook 오프라인 fixture 확인** (포트 18410, TypeScript 빌드 PASS) | 원본 Storybook 애드온 충돌로 커스텀 설정 사용; `/tmp/capstone-rmf-web-official-20261113` 소스는 재부팅 시 소멸되므로 `prepare_frontend.sh` 재실행 필요 |

---

## 3. 실차 구동 재개 전 필수 차단 요인 (Strict Blockers)

1. **전원 돌발 차단(4회) 원인 미규명**: 2026-11-13 실차 시험 중 전원이 4회 갑자기 차단되었으며, 충전기 분리·이동 시 베이스 USB(`usb 3-7.4`, `ttyUSB1→ttyUSB2`) 재연결로 `wheel_odom_stale` FAILSAFE(`run_008`)가 발생했습니다. 원인 규명 및 완충 상태 소크 시험(`power_logger.py`) 통과 전까지 `--drive` 자율 주행을 금지합니다.
2. **Depth 센서 거리 스케일 및 지터**: 실차 Depth 거리가 LiDAR 대비 약 `0.91~0.93배`로 짧게 측정되고, 필터 출력 주기 지터(최대 간격 680 ms vs 타임아웃 0.75 s) 및 소크 중 IR 투사 정지 현상이 미해결 상태입니다.
3. **물리적 비상 정지(E-Stop) 미검증**: 모든 실차 구동 시험은 사용자가 전원 스위치 옆에 입회하고 단계별 승인을 얻은 감독 시험으로만 제한됩니다.


## 2026-11-15 추가 현황: Registry · Operation Manager · Gate

- **Map Registry — 로컬 구현**: 지도 조회와 제한된 등록/업로드 경로가 있으며 등록 지도는 UNVERIFIED다. 멱등성·DB/파일 원자성 처리와 충돌·강제 종료 시험 코드가 있다. 등록은 지도 활성화·위치추정 검증이 아니다. 고아 진단은 오프라인 스냅샷 전용 읽기 진단이며 복구·자동 채택은 미구현이다. 대형 요청의 메모리 부하와 복구·운영 부하 검증이 남았다. [구현](fleet_control/server/fleet_server/map_registry.py), [복구 제안](fleet_control/server/ORPHAN_RECOVERY_PROPOSAL.md).
- **Operation Manager — 제안 및 독립 Mock**: Phase 1~1.2는 실제 서버·Agent·Gate와 연결되지 않는다. Phase 1.3 격리 ROS Action 시험은 별도의 모의 서버 대상이며 [시험 코드](fleet_control/tests/test_ros2_goal_boundary_isolated.py)는 직접 클라이언트의 수락과 소유 Goal UUID만 취소하는 결과를 검사한다. 이번 조사에서는 그 시험의 저장된 실행 로그를 찾지 못해 PASS를 독립 확인하지 못했다. 실제 Explore의 cancel-all은 다른 목표에도 영향을 줄 수 있고 종료 시 복귀 Goal도 별도 소유권 문제가 있다. [Phase 1.2](fleet_control/OPERATION_MANAGER_PHASE1_2_PROPOSAL.md)의 Agent 단일 제출 경계 B안은 권고이지 확정·구현이 아니다. 모든 직접 제출·복귀·취소 경로와 외부 클라이언트를 통제/탐지하지 못하면 선점 방지를 보장하지 않는다.
- **Safety Gate — 후보**: `fixed2`/`fixed3`과 v4.1/v4.2 diff 후보를 구분한다. fixed4는 생성되지 않았다. v4.1은 보정된 모의 ROS 그래프 결과와 무장 불가 문제가 보고됐고, 최신 v4.2 보고는 결과 2~4절이 미기입이다. 구조화 Gate Attestation, 발행자 GID 연속성 및 다중 action 서버 감시는 미구현이다. 최초 status 부재, 2초 표본 신선도, Gate boot 변경 후 epoch 역행 처리의 충돌은 재조정·운용을 차단할 수 있다. [최신 보고](experiments/safety_chain_fix_20261113/docs/GATE_V4_2_SAFETY_REVALIDATION_20261115.md). 실제 Nav2·Gazebo·실차에 대한 v4.2 검증으로 확대하지 않는다.
- **Safety Gate r7 후보 (2026-11-16, SIM_ONLY)**: [r7 보고](experiments/safety_chain_fix_20261113/docs/GATE_R7_PRODUCER_INTEGRATION_20261116.md). r6에 상태 계약 필드(`status_seq`·`safety_state`·`nav_evidence` 등), 다중 Nav2 action 서버 관측(E-L/E-S), `/cmd_vel`·상태·action writer 중복 차단, 비인증 challenge echo를 추가했다. 실제 Gate r7 + 실제 Agent·Server를 격리 ROS 2(모의 Nav2·합성 플랜트)에서 시험해 5개 케이스 65/65 PASS를 얻었다. Gazebo office(실제 Nav2)에서 Gate는 E-L로 무장했고, **시험 클라이언트 직접 goal** 주행 중 F4 정지 주입 시 Gate 출력 0까지 0.374 s, GT 정지까지 0.52 s가 걸렸다. **Fleet→Agent→Nav2 경로는 `NAV2_STATUS_STALE_OR_UNKNOWN`으로 BLOCKED**다(Codex 변경 요청 CR-1). Attestation·발행자 인증은 미구현(`trust=UNAUTHENTICATED_SIM_ONLY`)이고, Agent 사망 중 goal 계속 주행이 관측됐다. 실차 미실행·실차 허가 근거 아님. fixed4 미생성.
- **Safety Gate r8 후보 — Agent 생존 감시 (2026-11-16, SIM_ONLY)**: [r8 보고](experiments/safety_chain_fix_20261113/docs/GATE_R8_AGENT_LIVENESS_20261116.md). Gate가 0.1 s마다 nonce를 발행하고, Agent heartbeat가 같은 Gate boot와 nonce를 0.5 s 이내에 되돌리지 않으면 안전 체인 fault(래치)로 처리한다. 재시작·재개·재생만으로는 재무장하지 않는다. 기본 비활성이며 응답자는 Claude SIM_ONLY 래퍼에 있다(Codex 운영 구현 요청 CR-R8-1).
  - 격리 결과: Goal 중 Agent SIGKILL 시 출력 0 고정까지 0.405 s, 이후 비영 0(감지 창 비영 20건). Codex R0 기준(2 s 비영 0건)은 감지 창 24건으로 **FAIL** 유지. SIGSTOP·생존 메시지 중단·이전 세대 재생·재시작 PASS.
  - Gazebo(실제 Nav2) 결과: Fleet 경로 goal 주행 중 SIGKILL 시 0.455 s에 출력 0, GT 정지 0.597 s / 0.151 m.
  - **Fleet 정상 임무는 실제 Nav2 `follow_path` 하위 UUID를 foreign으로 판정해 `RECONCILIATION_REQUIRED`가 되고, Agent는 취소하지 않는다(CR-R8-2).** 인증 없음, 실차 미실행, fixed4 미생성.
- **Safety Gate r9 후보 — Cliff 관측 연동 (2026-11-17, SIM_ONLY)**: [계약](experiments/safety_chain_fix_20261113/docs/GATE_R9_CLIFF_CONTRACT_20261117.md), [Gazebo 결과](experiments/safety_chain_fix_20261113/docs/GATE_R9_CLIFF_GAZEBO_20261117.md). P4d8 관측기(수정 없음)의 출력을 Gate가 소비한다. 전진에는 신선한 GROUND_OBSERVED가 필요하고, CLIFF_CANDIDATE·UNKNOWN·stale이면 fault와 래치가 걸린다. 실제 Gazebo + 실제 Nav2(직접 goal) 결과는 다음과 같다.
  - 8 cm 단차·계단·30 cm 가장자리·난간 경계: 정지 시 바퀴–경계 0.79/0.78/0.80/0.61 m, 낙하 없음. 무보호 기준선은 실제로 낙하했다.
  - 정지 근거는 위험 직접 검출이 아니라 원거리 바닥 미관측(UNKNOWN)이다. Gazebo 제동은 비현실적으로 빠르다.
  - 평지 근접 장애물로 인한 주행 불능은 미시험. 실차 미실행, fixed4 미생성.
- **Gate r9/r10 Cliff 잔여 위험 (2026-11-17, SIM_ONLY)**: [보고](experiments/safety_chain_fix_20261113/docs/GATE_R10_CLIFF_RESIDUAL_RISK_20261117.md). 결과를 세 항목으로 나눴다.
  - UNKNOWN 기반 정지: 원거리 경계와 회전 직후 전진에서 PASS.
  - CLIFF_CANDIDATE 기반 정지: 정지 상태에서만 PASS(주행 중 미발생).
  - **실제 바퀴 지지 영역: FAIL.** 카메라 사각지대(0.25–0.48 m)나 경로 위의 폭 0.15 m 틈에서 GROUND_OBSERVED가 전진을 허가했고, 바퀴가 틈에 빠졌다. 무보호와 결과가 같다. 근거리 cliff 센서나 카메라 배치 변경이 필요하며, 관측기 변경 요청은 다른 에이전트 앞으로 남겼다.
  - 그 밖: 벽 근접 불필요 래치 1/2회.
  - r10 = r9 + Nav2 lifecycle GetState 재요청이다. 기동 30 s 래치의 원인(응답 유실)을 해결했다: r10 17회 중 0회, r9는 15회 중 2회. 실차 미실행.

- **2026-11-15 후속 Operation Manager 결과**: 위의 Phase 1.3 로그 미확보 문장은 당시 조사 상태다. 이후 [Phase 1.3 재현](experiments/phase13_ros_action_20261115_K8m4/RESULTS.md), [1.4](experiments/phase14_goal_boundary_20261115_q4H7/RESULTS.md), [1.5](experiments/phase15_sros2_20261115_F8v2/RESULTS.md)의 격리 로그가 보존됐다. 사용자가 B안을 후속 설계 기준으로 선택했지만 운영 경계는 미구현이다. [1.6 시험 프로토타입](experiments/phase16_goal_arbiter_20261115_D2k8/RESULTS.md)은 운영 Agent가 아니며 SROS2와 중재기를 함께 검증하지 않았다.

- **Phase 1.7 후속 시험**: [결합 시험](experiments/phase17_goal_sros_20261115_M6w3/RESULTS.md)은 SROS2 Enforce와 시험 중재기·모의 ActionServer를 실제 ROS로 연결해 단일 프로세스 경합과 SIGKILL 후 미완료 Journal 차단을 확인했다. 동일 Agent 자격증명의 별도 중재기 두 개가 각각 Goal을 제출해 **프로세스 간 fencing FAIL**이다. Attestation 검사도 미구현이므로 운용 이동 허가는 금지 상태다. 실제 Agent·Nav2·Gate·실차 통합은 SKIP.

- **Phase 1.8 시험 모델**: [격리 결과](experiments/phase18_fencing_20261115_G7r2/RESULTS.md)는 같은 단일 호스트 저장소를 협조적으로 쓰는 두 프로세스의 flock 경합과 미완료 Goal 복구 차단을 확인했다. DB snapshot 롤백 시 epoch 재사용은 FAIL이고, 합성 Attestation은 인증된 Gate 증거가 아니다. Nav2 자체는 fencing 세대를 확인하지 않으며 운영 통합·실차 이동 허가는 금지 상태다.


## 2026-11-16 Fleet UI 개편 (Codex · 합성/격리 HTTP 검증)

중앙 지도·로봇 상태·현재 임무/센서·운영 제어·최근 이벤트를 메인 화면에 배치하고 기존 Lease/Registry/안전 원문/RMF/로그는 상세 탭에 보존했다. 1920×1080·1600×900·1366×768 문서 overflow 없음, Chromium 23 checks와 기존 Python 44 tests 및 JS 2 suites PASS. 실제 센서·실차/운영 서버 및 승인 이미지 일치 검증은 SKIP. Gate 보고값을 이동 승인으로 확정하지 않는다. [상세·캡처·재현](fleet_control/docs/DASHBOARD_UI_20261116.md).


## 2026-11-16 Spatial Monitor UI

격리 Fleet 공간 관제 UI 추가. 지도·RGB 패널과 Depth 미제공 표시, 조회 전용 클릭, 위치 미확정 차단을 구현했다. 실제 Depth/계획·실차·Gazebo는 SKIP. [상세](fleet_control/docs/SPATIAL_MONITOR_PHASE3.md).


## 2026-11-16 Spatial 3가지 보기

공간 관제 3모드 UI 추가. Registry 저장 지도 썸네일과 공유 지도/RGB, 증거 없는 소속 UNKNOWN 구현. 실제 경로·Depth 영상·실차/Gazebo 미검증. [보고](fleet_control/docs/SPATIAL_VIEWS.md).


## 2026-11-16 Fleet UI 전체 리디자인

Fleet UI 전체 리디자인: 단일 열 접이식 사이드바와 상태바, 기능별 화면 배치. Chromium 10화면×3 해상도 가로 넘침 없음, 기존 임무·안전 회귀 PASS. 실제 스트림·실차/Gazebo SKIP. [보고](fleet_control/docs/FLEET_UI_REDESIGN.md).


## 2026-11-16 지도 집중 모드

지도 집중 UI: 1920×1080 실제 캔버스 면적 Overview +17.5%, Spatial +14.5%, Maps +18.7%. 격리 Chromium·안전/임무 회귀 PASS, 실차/Gazebo SKIP. [결과](fleet_control/docs/MAP_FOCUS_DENSITY.md).


## 2026-11-17 Fleet UI 테마

테마 선택과 Overview Fleet 요약 추가. 테마 전환·상태 보존 및 Chromium 안전/임무 회귀 PASS, 실제 센서·실차/Gazebo SKIP. [보고](fleet_control/docs/UI_THEME_REDESIGN_20261117.md).


## 2026-11-17 승인 Workspace 운영 UI 적용

승인 Workspace 실제 적용. Chromium 10개 runtime 시나리오 그룹, Light/Dark 3화면×3해상도·34px 상태바, 실제 임무 REST 및 권한·Fault/Offline/미확정 위치 PASS. Python 44 tests·JS 2 suite PASS. 추가 legacy recovery/ownership 실행은 sandbox 환경 오류 SKIP으로 구분. [보고](fleet_control/docs/WORKSPACE_RUNTIME_20261117.md).


## 2026-11-17 `/map` 타입 충돌 해결 (Gazebo/RMF, SIM_ONLY)

- **해결됨(Gazebo 2회)**: `/map`=`nav_msgs/OccupancyGrid`(SLAM·Nav2·Agent), BuildingMap은 `/rmf/building_map`. SLAM 지도와 BuildingMap이 rosbag에 실제 기록되고 재생 시 복원됨. 웹 SLAM→저장→Registry→재로드→중지 회귀 PASS. [보고](fleet_control/docs/RMF_MAP_TYPE_SEPARATION_20261117.md).
- **적용 범위**: 해결은 `LD_PRELOAD` shim(`fleet_control/rmf_adapter/rmf_map_shim/`)을 **Gazebo/RMF 프로세스에 적용했을 때만** 성립한다. 임시 호환 계층이며 RMF 플러그인 소스 수정(근본 해결)은 미수행. 실차 그래프에는 해당 플러그인이 없어 shim 불필요. shim 없이 BuildingMap을 `/map`에서 기대하는 RMF 소비자는 미호환·미검증.
- **미해결**: `/scan_slam` 기록(sim 미발행, 실차 현장 검증 필요), CameraInfo·원본 영상 기록(sim 미발행·프로파일 설계 외), Posegraph Registry 등록(4.97 MB가 Agent 3.8 MB 임계값과 서버 6 MiB 본문 한도에 걸림, 진단만). 실차·장시간 SKIP.
- **변경 없음**: Gate r9/r10 Cliff 잔여 위험, Fleet Goal 소유권·Gateway 접근 제어 등 다른 작업의 FAIL/미해결 기록과 Fleet 주행 기존 FAIL은 이번 결과와 무관하며 그대로 유효하다.


## 2026-11-18 Fleet 단일 임무 Gazebo 통합 (SIM_ONLY)

- **Fleet Goal 소유권(정상 FollowPath 오판)**: 수정·검증됨(SIM_ONLY 제한 신뢰 모델, 웹 단일 임무 PASS 1회, 반복성 미입증). 정상 FollowPath 자식·교체 겹침·증거 지연은 foreign으로 취급하지 않음. **실차용 아님**: 부모-자식 provenance는 비인증 graph 증거(`bt_navigator*` 노드명 위장 불가능 아님). [보고](fleet_control/docs/FLEET_FOLLOWPATH_OWNERSHIP_20261118.md).
- **여전히 FAIL/미해결**: 외부 NavigateToPose goal 차단 없음(Agent는 감지, Gate/Agent가 정지시키지 않음), 기존 Gateway 접근 제어 FAIL, 재시작 후 Gate 재무장·Server recover, Goal 결과 지연·중복 실주입, Gate `MEMORY_NOT_APPLIED_SHADOW`·기동 래치(Gate 담당), 실차 미검증.
- 변경 없음: Gate r9/r10 Cliff 잔여 위험 기록, 이전 Fleet 소유권 FAIL 원본 기록(해당 시점 코드의 결과로 유효).

## 2026-11-18 MANUAL SLAM Gazebo 상태

SIM_ONLY r8 MANUAL 프로파일에서 웹 SLAM 시작·RUNNING·`/map`과 `map→odom` 단일 발행자(동적/정적 TF 15초 관측)를 확인했다. 별도 실행에서 지도 이후 Gate의 Nav2 5개 액션 유휴를 확인했다. Gate는 안전 래치 상태였고 주행·실차는 검증하지 않았다. 상세 및 초기 실패는 [VALIDATION.md](VALIDATION.md)의 같은 날짜 항목 참조.

## 2026-11-18 MANUAL MAPPING Joy Gazebo 상태

SIM_ONLY에서 Fleet 임대와 Gate 준비 상태를 확인한 뒤 기존 reset으로 NORMAL에 재무장하고, 합성 Joy 전진·회전과 중립/deadman/timeout 정지 및 `/map` 변화까지 관측했다. 회전 중 GT 병진 편차 약 0.056 m. 실차·지도 저장/재로드는 미검증. 상세·초기 래치·로그는 [VALIDATION.md](VALIDATION.md) 참고.

## 2026-11-18 MANUAL STOP · 지도 보존

SIM_ONLY 연속 Gazebo 실행에서 웹 STOP 이후 입력 차단·최종 출력과 GT 정지·지도 저장·Registry 무결성·SLAM 종료·Registry 번들 재로드를 확인했다. 재로드 셀 0개 차이, 해상도와 x/y 원점 일치; 원점 yaw 독립 비교 및 웹 브라우저 시각 검증은 해당 실행에서 미수행. Registry 상태는 `UNVERIFIED`; 실차 미검증. 상세 실패·제약은 [VALIDATION.md](VALIDATION.md) 참조.

추가 실행 `manual_finish_332030`에서 Registry 번들 재로드의 원점 yaw까지 SLAM 원본과 일치를 확인했다. 웹 브라우저 시각 검증 및 웹 reload 버튼은 미검증·미구현이다. [상세](VALIDATION.md).

## 2026-11-18 AUTO MAPPING Frontier 실험: 미완성

기존 FrontierSearch를 재사용하는 후보 전용 노드와 Fleet Mission/Agent journal 기반 조정 구성요소를 추가했으나 서로 및 웹에 연결하지 않았다. 실제 격리 Gazebo에서 후보 15개와 SLAM RUNNING을 관측했지만 웹 AUTO 시작은 404, 선택 후보의 기존 Fleet 임무 제출은 `safety_not_ready` 409, Gate는 `supervisor_alive` 미충족 래치였다. 자동 주행·반복 탐색·AUTO STOP/저장/재로드는 미검증. 기존 Fleet의 외부 NavigateToPose 미차단 FAIL도 미해결이며 검사를 해제하지 않았다. [상세](VALIDATION.md).

AUTO Supervisor 후속: SIM 전용 Agent opt-in 연결 후 실제 heartbeat와 Gate NORMAL을 관측했다(`344513`, 기존 RMF map shim 사용). 웹 AUTO 404, Fleet Mapping Goal 미구현·409이므로 자동 주행 E2E는 여전히 미완료이며 실차 시험 준비는 후속 단계다. [VALIDATION.md](VALIDATION.md) 참조.

## 2026-11-18 실제 LIMO Agent 비구동 배포

실차 `pcu`의 기존 Agent workspace를 백업하고 최신 로컬 Agent 패키지만 빌드·해시 확인했다. 미션·SLAM·녹화 비활성 상태에서 임시 Fleet Server 인증·상태 보고·서버 중단 후 재접속, 두 번 정상 종료를 확인했다. 실제 센서 스택은 미기동으로 지도·배터리·LiDAR·Depth·Nav2·Gate 데이터 및 실차 주행은 미검증. SIGSEGV 원인과 전원·USB·물리 E-Stop 차단은 유지한다. [VALIDATION.md](VALIDATION.md) 참조.

## 2026-11-18 실차 Depth TF·지도 저장 수정 배포

- **실차 모터 잠금, run_003**: 센서·SLAM 재기동과 읽기 전용 Fleet 재인증, 새 라이브 지도 업로드·화면 표시 PASS. 300초 동안 Depth 상태 오류 38/5,976(0.64%), 수정 전 bag 214/270(79.26%). 촬영 시각 TF를 기다린 시도 866회 중 859회 해소, 7회 만료.
- **Gate 안정성 FAIL**: 바퀴 `/odom`, `/imu`, `/limo_status` 배포 중단. 관측 기준 NORMAL 48/2,989, FAILSAFE_SENSOR_TIMEOUT 2,941/2,989. 처음 전환은 Depth READY 상태에서 wheel odom stale. 로봇 베이스 시리얼 장치 FD는 열려 있었으며 데이터 중단 원인은 미확정. 물리 정지 수단·전원/USB·통신 단절 정지 및 Depth 저속 설정까지 확인하기 전 실차 자동 맵핑 주행은 차단.
- **실차 입력 + 격리 Nav2 PASS**: run_003 실시간 `/map` 스냅샷으로 새 저장 코드 실행, 41×120 / 0.05 m / 원점·회전·4,920셀 일치. 현장 세션 내부 최종 저장·신규 posegraph는 최신 바퀴 odom 요구 조건 때문에 미검증. 기존 원본은 보존. [측정·절차](VALIDATION.md#2026-11-18-실차-depth-tf지도-저장-수정-배포와-300초-관측)

## 2026-11-18 베이스 스트림 중단 추적 및 수동 맵핑 준비

run_003의 최초 Gate 오류는 마지막 wheel odom 약 387 ms 이후이며 Depth는 READY였다. 기존 ttyUSB1과 열린 FD는 같은 장치였고 실차 드라이버 수신 카운터는 8초간 증가하지 않았다. 현장 커널 조회는 해당 구간 출력이 없었다. 11/13처럼 USB 재열거가 있었다는 증거는 없으며 물리 원인은 미확정. 기존 실차 20초 재확인 세 베이스 스트림 0건, Gate 201/201 FAILSAFE로 주행 차단 유지. 격리 PTY 재현 및 합성 Joy → Humble teleop → 기존 Gate 잠금 경로는 PASS. 실제 조이스틱 연결·실차 명령 중재·주행은 미검증. [증거](local_data/raw_results/real_limo_base_stream_20261118_01/README.md)

## 2026-11-18 베이스 단독 재시작 실패·재부팅 복구

run_003 베이스 단독 재시작은 포트 초기화 실패. 읽기 전용 open도 EIO였으며 CP2102 커널 로그에 -71/EPROTO와 UART 활성화 실패가 남았다. 11:36 초기 장애의 물리 원인은 미확정. 사용자 승인 재부팅 뒤 run_004 모터 잠금에서 연속 120초 Odom/IMU/상태 약 49.95/99.90/2.00 Hz, Gate NORMAL 1,191/1,191, FAILSAFE 0. Fleet 읽기 전용 재연결·지도 업로드 PASS. 조이스틱 잠금 진단 프로파일은 로컬 준비이며 실차 배포·주행은 미검증. [증거](local_data/raw_results/real_limo_base_recovery_20261118_01/README.md)

## 2026-11-18 실차 Joy 입력·공유 USB 허브 장애

실차 직접 입력 확인 PASS(전진 1, 조향 0, 버튼 4, 중립·빼면 무수신). 장착·분리 과정에서 허브 3-7과 CP2102가 같이 재열거되어 ttyUSB1→ttyUSB2, Gate wheel_odom_stale. 승인된 베이스 단독 재연결 뒤 120초 Gate NORMAL 1,200/1,200, 49.95/99.90/2.00 Hz 및 잠금 속도 0. 기존 run_004·SLAM·Fleet·기록기는 유지. 현장 저장과 raster/posegraph 개별 격리 재로드에서 5,760셀 차이 0. 온보드 수동 진단 프로파일은 배포하지 않았으며 최종 노트북 웹 입력은 기존 SIM_ONLY Agent Joy와 구분해 신규 검증 필요. [증거](local_data/raw_results/real_limo_joystick_locked_20261118_01/README.md)

## 2026-11-18 웹 수동 맵핑 입력 벤치

노트북 Fleet에서 실차 Agent까지 합성 웹 축 프레임 전달 PASS(전진·양방향 회전·중립·deadman 해제 및 만료). Agent는 잠금 벤치에서 영속도만 발행할 수 있다. 현장 run_004는 기존 Nav2의 /cmd_vel_raw 발행과 약 12:34 기준 wheel odom/IMU/상태 중단 때문에 Gate까지 시험이 차단됐다. 실제 노트북 Gamepad 브라우저 입력 및 실차 수동 구동 미검증. 격리 ROS 실제 Gate까지는 PASS. 수동 주행·새 맵핑/등록 및 후속 AUTO/Nav2는 미실행. [증거](local_data/raw_results/web_manual_bench_20261118_01/README.md)

## 2026-11-18 수동 맵핑 최종 연결 시도와 전원 차단

run_004 베이스 단독 재시작 FAIL, ttyUSB2 CP2102 -71/UART 활성화 실패. 12:34경 실제 스트림 중단과 동시에 새 재열거가 있었다는 증거가 없음. 현장 배터리 부족 보고로 추가 실차 기동과 주행 보류. 기존 웹 수동 프로파일은 격리 실제 Nav2 그래프에서 raw 발행자 4→0 및 격리 토픽 4 PASS. 실차 적용, 실제 브라우저 입력·단절, 수동 맵핑 및 등록 SKIP. 기존 run_004 bag 복사본의 해시·무결성·재생 확인 PASS. [시험 기록](local_data/raw_results/web_manual_field_final_20261118_01/README.md)

## 2026-11-19 웹 관제 Mapping·C103 지도 (상태)

| 기능 | 구현 | 검증 |
|---|---|---|
| C103 라운지 지도 Registry 등록·영역 표시 | 구현 | 격리 Registry·웹 UI PASS. 현장 Registry는 미등록 |
| 웹 START/PAUSE/RESUME/SAVE/STOP | 구현 (SIM/격리 Agent 전용) | Mock·격리 ROS 2·웹 UI PASS. Gazebo SKIP, 실차 BLOCKED |
| NEW_MAP / CONTINUE_MAP / 계보 | NEW·계보 구현, CONTINUE는 서버 게이트만 | CONTINUE 실제 적재는 미구현 |
| Geofence·종료 한계(시간·목표 수·배터리) | 구현 | Mock·격리 PASS(실제 경계 효과는 정지 센서라 제한적) |
| 지도 클릭 목표 사전 검사·맵핑 중 차단 | 구현 | Node·웹 PASS. Nav2 도달은 Safety Context 생산자 부재로 미검증 |

## 2026-11-20 안전 기능 상태

| 기능 | 구현 | 검증 |
|---|---|---|
| Agent-독립 Motion Watchdog (heartbeat·Gate 신선도·idle 증명·cancel-all) | 구현 (`limo_robot_agent/motion_watchdog.py`) | 단위 7/7, 격리 ROS 2 고장 7종 PASS. 실차 launch 미편입(패치 제안 02) |
| 회귀시험 | 75/75 PASS (의도 환경) | ownership·recovery·phase20 계약 v1 복구 |
| Depth 높이 기준 | 변경 없음 | 실측 대기 (`base_footprint_z`) |

## 2026-11-20 오후 Watchdog·Gate 통합 상태

| 항목 | 구현 | 검증 |
|---|---|---|
| Watchdog idle 증명 강화·goal 소유권·자기 정지 | 구현 | 단위 15/15, 격리 재시험(개정 Gate 사본) 9/10 PASS, 1건 관측 불가 |
| Gate watchdog 요구(03b)·NaN 정리(04)·실차 launch 편입(02b) | **제안만** (다른 에이전트 파일) | 사본으로 격리 검증, 원본 미적용 |
| watchdog trip 후 자동 재출발 금지(Safety Context epoch) | 구현 | `test_watchdog_admission.py` 4/4 |
| 실차 웹 관제 | 상태 표시·Lease만 사용 가능. Mapping·Mission은 실차에서 차단 | LIMO offline, 오늘 미시험. **실차 주행 No-Go** |

## 2026-11-20 저녁 웹 관제 운영 상태

| 항목 | 상태 |
|---|---|
| 18190 관제 서버 | 실행 중(로컬 `fleet_control/`, B1 사본). 인증 401/200 확인 |
| C103 지도(사본) | 등록·API 표시 확인(건물·층·영역·버전·센서 출처). 브라우저 로그인 화면 확인은 사용자 몫 |
| 실차 연결 | 모터 잠금 스택+읽기 전용 Agent+영상 경로 연결(웹 프로브 17/0). 주행 No-Go |

## 2026-11-20 공개 기준선과 후속 작업 감사

- **Mapping, 로컬 격리 검증** — START/PAUSE/RESUME/SAVE/STOP, C103 등록과 지도 표시, 지도 클릭 좌표 변환과 Goal 제출 경로를 시험했다. SAVE는 UNVERIFIED 버전과 계보를 기록한다. NEW_MAP이 구현되어 있다. 부모 Pose Graph를 다시 읽는 독립 CONTINUE_MAP 시험은 성공했으나 Worker의 재로드·localization 확인·후속 버전 흐름은 미완성이다.
- **Goal, 격리 ROS 2** — Frontier/Fleet 공유 슬롯, 소유 UUID에 대한 취소와 terminal 대조, 외부 Goal 감지, 재시작 시 이전 승인 무효화를 점검했다. 실차 Goal과 운영자 복구는 미검증·미구현 상태다.
- **Watchdog, 격리 ROS 2와 단위시험** — 로컬에 구현되어 있다. 실제 주행 준비는 차단 상태다. 02b·03b·04는 적용 전 제안이며 03b·04는 Gate 사본에서 시험했다. 기존 Gazebo·실차 이력은 별도의 소스와 실행 조건에 해당한다.

## 2026-11-20 수동 세션 부분 통합

Fleet `manual-bench`는 오프라인·재연결, lease 세대 변경 및 관측된 만료 시 이전 벤치 세션을 폐기하고, Mapping 및 NavigateToPose 상태 미수신·오래됨·활성·중복 publisher·잘못된 수치에서 벤치 입력을 거부한다. 잠금 `ManualWebBench`는 여전히 영속도만 발행하므로 이는 수동 주행 허가가 아니다. 독립 `motion_watchdog`은 NavigateToPose 상태 미수신·신선도 만료 시 차단한다. Nav2 상태는 변경 시 발행될 수 있으므로 2초 기본 만료가 정상 유휴 상태에서도 차단할 수 있다. 지속적인 신뢰 상태 생산자, FollowPath 및 외부 Goal fencing은 미구현이다.

기존 staging 메서드 단위 안전 회귀는 12 PASS/1 SKIP(티켓 미구현), 별도 벤치 6 PASS, Watchdog predicate 2 PASS. Mapping 회귀는 19 PASS/7 SKIP. Agent 패키지 빌드 PASS. 실행 환경의 loopback TCP/UDP 차단으로 실제 HTTP·ROS 전체 경로, Gate 잠금 출력, Gazebo 및 실차 시험은 SKIP이다. 수동 패널은 공통 화면 미연결이며 staging Arbiter와 종단 체인은 미구현이다. 공개 기준선 FAIL 2 및 실제 Watchdog ROS 검증은 미해결이다. 실차 수동 주행·Mapping START 차단 해제를 금지한다. [변경 및 시험 보고](fleet_control/docs/MANUAL_IMPLEMENTATION_PARTIAL_20261120.md).

## 2026-11-20 저녁 Phase 1 웹 수동 조작 (Claude)

| 항목 | 상태 |
|---|---|
| 웹 → Fleet → Agent → Arbiter → Watchdog → Gate 비영 수동 속도 | **격리 ROS 검증**(54/54 PASS), Gazebo·실차 미검증 |
| 실제 Nav2 idle(status 무발행) 대응 | 구현·격리 검증(rclpy ActionServer). 실제 Nav2 스택 미검증 |
| MANUAL/AUTO/MAPPING 서버 배타성·`control_mode` 표시 | 구현·격리 검증(Mapping·AUTO 상태는 합성 주입) |
| 실차 수동 주행 | **차단 유지**(real 벤치 0 출력, 모터 잠금) |

## 2026-11-20 밤 Phase 2 운용 모드 (Claude)

| 항목 | 상태 |
|---|---|
| 웹 운용 모드 전환 IDLE/MANUAL/MAPPING/NAVIGATION | **격리 ROS + 운동학 시뮬 검증**, Gazebo·실차 미검증 |
| 웹 자동 맵핑 START/PAUSE/RESUME/SAVE/STOP/RECONCILE | 격리 검증(실제 SLAM Toolbox·Frontier·Nav2). 실차 차단 유지 |
| NAVIGATION 임무 | 격리 검증. 지도·localization은 FIXTURE(실제 생산자 없음) |
| 로봇 단위 reconciliation 웹 복구 | **미해결**(admin recover `no_evidence`) |

## 2026-11-20 심야 Phase 3 (Claude)

| 항목 | 상태 |
|---|---|
| 저장 지도 웹 Navigation(Registry → map_server/AMCL → 초기 위치 → goal 검사 → Nav2) | **격리 ROS + 운동학 sim 검증**(17/17). Gazebo·실차 미검증 |
| 로봇 단위 reconciliation 웹 복구 | 격리 mock에서 Nav2 terminal 증거 기반으로 구현·검증. 실차 거부 유지 |
| Codex P0/P1 결함(Watchdog GID, 모드 CONFIRMING, Action 출처, f06 하네스) | 수정, Codex 시험 PASS. 실제 DDS 경합은 미검증 |
| Gate 입력 독점 | **미해결**(Gate 담당) |

## 2026-11-20 Safety Gate r11 후보 — `/cmd_vel_raw` 입력 독점 (SIM_ONLY)
- **구현(후보)·격리 검증**: 그래프 수준 입력 출처 검증과 NaN/범위 차단. 격리 시험에서 field/r10 우회를 차단했다. **인증 아님**(SROS2 필요). field·`capstone_ws` Gate에는 미적용(제안 05). 실차 미검증, Gazebo 미검증.
- **실차 No-Go 유지**: Gate reset 웹 경로 없음, 인증 없음, STM32 timeout·E-Stop·제동거리 미측정. [보고](fleet_control/docs/GATE_INPUT_EXCLUSIVITY_20261120.md)

## 2026-11-21 관리자 웹 Gate Reset (SIM_ONLY)
- **구현·격리 검증**: 서버(관리자 강제)→Agent(사전검사)→Gate r12(최종 판단). SSH 없이 Gate 장애 확인·복구. 인증 아님. field·`capstone_ws` Gate 미적용, 실차·Gazebo 미검증. [보고](fleet_control/docs/GATE_RESET_WEB_20261121.md)

## 2026-11-21 Phase 3 안정화 (SIM_ONLY)
- **구현·격리 검증**: Gate boot 변경 후 서버 재조정, 재LOAD 초기 위치 전달, 저장 지도 직접 주행 교착 해소, Reset 카드 재시도 표시. 같은 조건 교대 시험에서 r12 회귀 근거 없음.
- **미해결**: Gate 프로세스 일시 정지 원인(r10~r12 공통). 정식 Gate 통합(제안 06) 담당자 검토 전. Gazebo·실차 미검증. [보고](fleet_control/docs/PHASE3_STABILITY_AND_GATE_R12_20261121.md)

## 2026-11-21 Gazebo 웹 제어 통합 · Gate 정지 분석 · Phase 3.5 (SIM_ONLY)
- **구현·Gazebo 검증(합성)**:
  - 웹 체인이 Gazebo LIMO를 움직이고 멈춘다(MANUAL, Deadman, 웹 끊김, Agent/Watchdog kill, LiDAR 정지, AMCL kill, 외부 publisher 주입).
  - 저장 지도 LOAD·CONFIRMED와 Nav2 목표 SUCCEEDED는 1회 확인했다.
  - 웹 MAPPING은 간헐적으로 교착한다.
- **미해결**: Gate 정지(E) 원인, STM32 timeout, Frontier 교착, pointer 입력. 실차 미검증. [보고](fleet_control/docs/GATE_STALL_GAZEBO_20261121.md)

## 2026-11-21 Gazebo 안정화 (SIM_ONLY)
- **구현·Gazebo 검증(합성)**:
  - Gate와 독립인 하위 명령 timeout(Gate 정지·종료 시 약 0.02 m 후 정지)
  - 웹 포인터 입력 손실 수정
  - Frontier 정체 수정
  - 실제 브라우저 지도 클릭 E2E(거부 경로 전부, 완료 1회)
- **미해결**: Gate E 원인, STM32 timeout, Gate 재개 직후 오래된 명령, 복도 ICP 거짓 양성, 서버/웹 목표 검사 기준 불일치. 실차 미검증. [보고](fleet_control/docs/GAZEBO_STABILIZATION_20261121.md)

## 2026-11-21 Phase 4·5 (SIM_ONLY)
- **구현·격리 검증**:
  - 감독 프로세스 기반 SSH 없는 기동, 준비 상태(READY/DEGRADED/FAULT)와 READY 전 MAPPING/NAVIGATION 거부, 관리자 웹 서비스 작업, 재시작 후 이전 세션·모드 재사용 금지
  - 방문객 안내 웹(별도 인증, 검증 지도만, 요청 → 임무 → 완료/취소/대기열)
  - 웹 목표 검사의 서버 기준 통일
- **미검증·미해결**: 실차 감독·systemd, QR 디코딩, 기존 안전 항목(Gate E, STM32 timeout, E-Stop, 제동거리). [보고](fleet_control/docs/PHASE4_5_AUTOSTART_VISITOR_20261121.md)

## 2026-11-21 Codex Phase 4·5 결함 수정 (SIM_ONLY)
- **수정·격리 검증**: 미확인 READY(Supervisor 누락·stale·NaN·Watchdog LOCKED·Gate cmd_blocked) 차단, READY와 `motion_permitted` 분리, 실차 Agent는 Supervisor 강제, 서비스 stop/restart는 신선한 IDLE 증거 + 관리자 Lease epoch, 방문객 요청 link·session 소유 확인, 같은 로봇 ACCEPTED 1개·FIFO, 지도 quarantine 조회 실패·파일 SHA 불일치·Agent 지도 불일치 차단, Phase 5 종료 rc=134 해소.
- **정상 기능**: Phase 4 8/0, Phase 5 8/0 rc=0 회차에서 자동 기동·방문객 안내 동작 확인 (격리 ROS). 센서 중단 회차는 간헐 실패(기존 이력).
- **미검증**: 실차 전부, Gazebo 탐사 g03 무이동 원인, kinematic sim `/scan` 간헐 중단 원인. [보고](fleet_control/docs/CODEX_FIX_PHASE45_20261121.md)

## 2026-11-22 Gazebo 통합 실패 수정 · Phase 4·5 안정화 (SIM_ONLY)
- **수정·Gazebo 검증**: 전체 24개 시험 13/11 → **18/5 (SKIP 1) ×2**(r3, r4). g03 탐사 무이동 원인 = 시험 순서상 물리적 끼임(시험 하네스 복귀 주행으로 해결), Mapping 세션 소실 원인 = `/clock` 정지 시 Agent sim-time 타이머 정지(steady 타이머로 해결).
- **제품 결함 수정**: 웹 취소 후 bt_navigator 무응답 시 Arbiter가 최대 `cancel_result_timeout_s`(5 s)까지 Nav2 출력을 전달하여 4 s+/0.5 m 계속 주행 → 취소 요청 즉시 0 출력(`CANCEL_REQUESTED`). Gazebo f07 0.17 s/0.016 m 정지 확인. Phase 1/2/3·Gate Reset 회귀 통과.
- **Phase 4·5**: 6회 기록 중 `/scan` 중단 재현 없음(원인 미확인). 방문객 FIFO·link 격리 통과.
- **미해결**: Watchdog SIGKILL 후 DDS 유령 writer(f04 연쇄), 지도/ICP 회전 품질에 따른 AMCL 추적 상실(가설), Nav2 RPP 충돌 감지 꺼짐(가설), LiDAR 평면 아래 낮은 장애물(실차에도 해당), Gate 재개 직후 1–2 tick 이전 명령 재출력(후보 패치 제안, 미적용). 실차 전부 미검증. [보고](fleet_control/docs/GAZEBO_INTEG_FIX_20261122.md)

## 2026-11-22 (2) 속도 재출력 안전 · Gazebo 잔여 실패 (Claude, SIM_ONLY)
- **제품 수정·격리 검증**: Agent VelocityArbiter 제어 세대(모드·서버 boot·lease epoch·goal UUID·cancel·공통 조건). 세대 변경 시 Nav2 표본 폐기, 세대 시작 0.3 s 이후 수신 표본만 전달, MANUAL은 deadman 재입력 필요. Motion Watchdog은 자기 정지 후 밀린 입력 폐기. 단위 7/7 + 1 (수정 전 원본에서 FAIL 재현).
- **Gazebo (r12 Gate)**: 24개 시험 18/5/1 → **21/2/1, 20/3/1**. f04 PASS 3/3 (f03이 새 Watchdog GID로 재결합될 때까지 대기, 유령 writer 16–20 s 동안 FAILSAFE 유지). Phase 1/2/3·Gate Reset 19/0·17/0·18/0·13/0, Phase 4 8/0 ×2, Phase 5 6/2·8/0·8/0 (r1 Nav2 계획 실패, 간헐).
- **Gate r13 후보 (미적용, Gate 담당자 제안)**: Watchdog 상태가 보고한 GID에만 결합, writer 재시작 시 sticky 0. Gazebo d01/d03/d04 재개 직후 이전 명령 재출력 r12 1/0/1·1/1/1 → r13 0/0/0.
- **미해결 (측정된 원인)**: g05 = ICP 회전 과소 추정(약 7 %) → AMCL yaw 오차 −0.20 rad → DEGRADED → 서버가 임무 무효화(안전 반응은 정상). 충돌 = LiDAR 평면 밖 의자(OfficeChairGrey_8 0.65 m)에 끼임(Gazebo depth 꺼짐, 실차에도 해당). 실차 전부 미검증. [보고](fleet_control/docs/GAZEBO_INTEG_FIX_20261122.md) Part 2

## 2026-11-22 (3) Gate r13 통합 준비 · 속도 세대 계약 최종 검증 (Claude, SIM_ONLY)
- **0.3 s settle 부족 (측정)**: 실제 nav2_velocity_smoother는 controller가 0 없이 멈추면 이전 명령 0.10 m/s를 0.91–0.95 s 재출력 (3/3). Arbiter에 producer liveness 추가: `/fleet/cmd_vel_nav` (controller·behavior 출력)가 세대 시작 이후 0.25 s 이내 수신된 경우에만 smoothed 표본 전달 (`NAV_PRODUCER_NOT_LIVE`). 단위 14/14, 이전 Arbiter는 새 2건 FAIL. 정상 새 goal은 주행 확인 (Phase 3 18/0, Phase 5 v02·v03).
- **Gate r13 vs r12 (격리 rig)**: zero/silent 재개 후 이전 명령 재사용 r12 20건(유효 35) → r13 0건(유효 36) (15:10 정정: 이전 23건은 집계 오류), live 동일, writer 교체 r13 미검증 writer로 이동 없음·reset 후 복구. NOT_ARMED 각 1건 = rig 기동 실패 (SKIP).
- **Gazebo/Phase (r13)**: gz 5/18/1 (RPP가 보이는 칸막이 벽으로 호를 그리며 접근해 붙음 → ICP DEGRADED, Gate가 reset 거부, 연쇄), 20/3/1 (평면 밖 의자 끼임 D2); r12 대조 17/6/1 (지도 UNKNOWN_AREA). d01–d04 재개 후 비영 출력 r12 2/1/1/1 → r13 0/0/0. Phase 1 19/0, Phase 2 16/1 (시험 도중 sim scan 끊김, 하네스 guard는 시험 사이에만 해제), Phase 3 18/0, Gate Reset 13/0, Phase 4 8/0 ×2, Phase 5 5/3·8/0·8/0.
- **판정**: r13은 Gate 담당자 검토용 통합 후보로 준비됨. 정식 Gate·실차 적용은 미인계·미측정으로 불가. 남은 차단: D1 ICP, D2 평면 밖 가구, D3 RPP 호 주행 (원인 확인), 지도 coverage. [보고](fleet_control/docs/GAZEBO_INTEG_FIX_20261122.md) Part 3, [결함 인계](fleet_control/docs/GAZEBO_OPEN_DEFECTS_20261122.md)

## 2026-11-22 (4) 실차 LIMO 통합 조사 · Gazebo LiDAR+Depth 시험 기준 정정 (Claude)
- **실차 (조사만, 주행·잠금 해제·SSH 없음)**: pcu OFFLINE으로 접근 불가 → 11-20 백업 기준 코드 조사. 실차 경로 = Nav2 → `/cmd_vel_raw` → Gate HEAD(`d103f445`) → limo_base, **Arbiter·Watchdog 없음**, host 측 timeout 없음. 실차 Gate HEAD를 격리 rig에서 시험: 재개 후 이전 명령 재사용 zero 9/18·silent 6/17, 미검증 writer 수용 17/17 (약 0.48 m 이동). `env.sh`는 tailnet 전체가 domain 77에 참여 가능 (우회 위험). 최소 변경·잠금 시험 계획은 [조사 보고](fleet_control/docs/REAL_LIMO_INTEGRATION_SURVEY_20261122.md). 실차 전부 SKIP.
- **Depth가 꺼져 있던 원인 (소스 확인)**: Gazebo 웹 하네스에 depth bridge·변환·카메라 TF·필터·costmap 소스가 없고 Gate가 `depth_mode: disabled`로 고정. 센서 모델은 존재. **이전 Gazebo 21/2/1·20/3/1·r13 결과는 모두 LiDAR 단독** (Depth 통합 검증 아님).
- **Gazebo LiDAR+Depth (시험용 방, truth 위치, 고정물 visual=collision)**: 별도 `depth_layer`에서 의자 받침·의자 가로대·책상 하부 프레임·낮은 상자·유리 모두 PASS_AVOIDED 2/2 (최소 0.19 m). 기존 방식(공유 `obstacle_layer`)은 LiDAR raytrace가 FOV를 벗어난 depth 표시를 지워 책상 2/2·상자 1/2 접촉 FAIL. LiDAR 단독은 낮은 상자 0점·의자 받침 접촉 FAIL. Depth 정지 고장 시 Gate가 0.53–0.55 s 내 정지 (2/2).
- **미해결·미검증**: 유리는 Gazebo에서 검증 불가 (센서가 투명도를 무시), LiDAR 단독 3회는 Gate 오정지(ICP 속도 ≈0, 가정: D1 계열)로 INCONCLUSIVE, `depth_layer` 변경은 Depth 프로파일 담당자 제안 (미적용), 실차 정지 Depth 의자 받침 시험 SKIP (pcu offline). [보고](fleet_control/docs/GAZEBO_DEPTH_20261122.md)

## 2026-11-22 (5) 실차 LIMO Pro 통합 전환 준비 (Claude, 실차 SKIP)
- **접근**: 18:25 pcu OFFLINE (2일 전 마지막), 18190 미실행 → 실차 시험 전부 SKIP, 로봇에 전송한 것 없음.
- **실차 체인 (코드 확인)**: 웹 → 서버 → Agent(real: 상태·Lease·거부만, Arbiter는 real 모드에서 기동 불가) → [Arbiter 없음] → [Watchdog 없음] → Gate HEAD → `/cmd_vel_locked`. `manual_web_bench` 사용 시 Agent가 `/cmd_vel_raw`에 0만 발행하는 두 번째 writer가 됨 (Gate HEAD는 writer 결합 없음). 실차 Depth 프로파일은 공유 `obstacle_layer` (Gazebo에서 낮은 장애물 지워짐이 확인된 배치).
- **준비**: Agent 배포 후보 (`local_data/raw_results/real_limo_integ_claude_20261122/deploy_candidate_agent/`, 32파일, 미배포), 독립 모터 잠금 조건 L1–L5와 잠금 통합 시험표. Gate r13·Depth 별도 layer·ICP 개선은 담당자 미검토로 미적용. [조사 보고](fleet_control/docs/REAL_LIMO_INTEGRATION_SURVEY_20261122.md) §10


## 2026-11-22 (6) Nav2 D3 개선 · D1/D2 분리 (Claude, SIM_ONLY)
- **D3 (Gazebo 표적 시험, 동일 조건)**: 기준 프로파일은 뒤쪽 목적지에서 벽으로 전진 호 주행 → 0/5(접촉, Gate STALL_LATCHED). 후보 O6(`use_rotate_to_heading` + `PoseProgressChecker`)은 3/3, 이동 중 여유 0.34 m, 벽 접근·인접 출발·취소 동등 이상. RPP 충돌 검사(O2)는 벽 근처 오중단으로 기각, 비용 감속(O7)은 효과 없음. 0.66 m 복도 U턴은 모든 프로파일 실패(미해결). 공유 Nav2 프로파일은 수정하지 않음(overlay 제안만).
- **D1**: 순수 제자리 회전에서 ICP가 4–7 %(사무실)·12–13 %(개방 지점) 과소. 노드 자체 코드 오프라인 재생이 동일 → 알고리즘·파라미터 원인(TF·시간 동기 아님). 재생 후보(keyframe 0.30 + trim 없음) −1.1~−2.9 %. ICP 노드 미수정.
- **D2**: Gazebo 의자·AdjTable 충돌 메시가 시각 메시보다 훨씬 큼(LiDAR·Depth는 시각만 봄). Depth 검출은 부모 세션(`GAZEBO_DEPTH_20261122.md`).
- **전체 회귀(조기 종료)**: Gazebo base 20/3/1·21/2/1, O6 18/5/1(실패 주원인 D1·지도 coverage), Phase 2/3/Gate Reset 17/0·18/0·13/0, Phase 1 18/1(타이밍), Phase 5 O6 미실행. Gate 리셋 1,376회 거부 = 시험 하네스 재시도(제품 아님) → 하네스에 지속 결함 재시도 상한 적용. 실차 미검증. [보고](fleet_control/docs/GAZEBO_INTEG_FIX_20261122.md) Part 4

## 2026-11-22 (7) 실차 기존 검증 재사용 · 최신 Fleet 통합안 (Claude, 실차 변경 없음)
- **재사용 근거**: 18:31 읽기 전용 조사로 로봇의 launch(`b87e16d3`)·Gate HEAD(`d103f445`)·Nav2 프로파일·Depth 필터·Agent가 11-20 기록과 동일 → LiDAR·Odom·IMU·TF·Depth·SLAM Toolbox·지도 저장·웹 ONLINE/Lease/거부 프로브(17/0, 13/0) 기존 실차 증거 재사용. 센서 단독 시험 반복 안 함.
- **신규 검증 대상**: 최신 Agent 후보(7개 변경·9개 신규, 신규 기능은 real에서 기본 off 또는 기동 거부)의 real 동작, STOP·세션 만료·재시작·링크 단절 ACK, 낮은 구조물(의자 받침 등) Depth 검출(필터+costmap), Depth 정지 시 Gate FAILSAFE, 격리 Fleet Server(18190 아님), 독립 모터 잠금 L1–L5.
- **미해결·차단**: 실차 Gate HEAD의 이전 명령·writer 계약 실패, 실차에 Arbiter·Watchdog 없음(real Arbiter 모드 미설계), STM32·E-Stop·제동거리 미측정, 베이스 USB 단절 원인 미확정. 구동·잠금 해제 금지. Gate r13·Depth 별도 layer·ICP는 담당자 소유 후보로만 관리. [계획](fleet_control/docs/REAL_LIMO_FLEET_INTEGRATION_PLAN_20261122.md)

## 2026-11-22 (8) 격리 서버·Agent 배포 전 검증 (Claude, 노트북 전용, 로봇 변경 없음)
- **검증됨**: 포트 18290 사용 가능, 별도 데이터·토큰으로 격리 서버(루프백) 기동·종료. Agent 후보 real 모드가 최신 서버에 연결(프로토콜 2·계약 1), 센서 없는 그래프에서 fail-closed(movement 불허), Lease·START 거부(409)·지도 클릭 목표 거부(409), Agent의 `/cmd_vel*` 토픽 없음. real에서 `manual_locked_arbiter`·`navigation_localization`·`map_save+missions` 기동 거부. 회귀 단위시험 9개 모듈 전부 통과(지도 저장 17+20).
- **발견**: `web_gate_reset`은 real 모드에서 거부되지 않음(명시 가드 없음; 간접 방어는 Gate `reset_readiness` 필요 — 코드 읽기만). `manual_web_bench` 둘째 발행자 문제는 Gate 그래프 없이는 미시험이라 차단 조건 유지.
- **미검증**: STOP ACK·Lease 만료·서버/Agent 재시작·링크 단절(실제 그래프 필요), 실차 상태 보고. 로봇에 파일 교체·ROS 기동·명령 주입 없음. L1–L5는 관측 기준이며 독립 차단 충분조건 아님. [검증](fleet_control/docs/REAL_LIMO_FLEET_INTEGRATION_PLAN_20261122.md) §11

## 2026-11-22 (9) 실차 Agent 배포 직전 안전 검토 (Claude, Agent 소유 파일만, 로봇 변경 없음)
- **수정 (Agent 소유)**: `agent_node.py` — `web_gate_reset`은 mock/sim·격리 도메인이 아니면 기동 거부(real 명시 차단). `manual_web_bench.py` — real 모드에서 `/cmd_vel_raw` 발행자를 만들지 않음(관측 전용). 신규 시험 `tests/test_real_mode_agent_blocks_20261122.py` 5/5 (수정 전 Agent에서는 3 FAIL + 1 ERROR), 기존 회귀 15개 모듈 전부 통과.
- **호환성**: Gate HEAD는 `boot_id`·`fault_epoch`·`fault_latched`·`safety_state`·`reset_readiness`를 보고하지 않아 후보 Agent의 movement permission은 항상 거부(잠금 단계에 맞는 fail-closed). 프로토콜 2/계약 1 동일. 잔여: 노드 자체는 real에서 `allow_missions:=true`를 받아들임(스크립트만 false 강제) → 제안만.
- **배포 후보 v2** (`deploy_candidate_agent_v2/`, 변경 2파일), 18290 접근 제한(패킷 필터·토큰 역할·전달)·롤백·승인 순서 준비. 실제 배포·ROS 기동·명령 주입 없음. [검토](fleet_control/docs/REAL_LIMO_FLEET_INTEGRATION_PLAN_20261122.md) §12–13

## 2026-11-22 (10) Agent v3 · 설정 오류 우회 차단 · 18290 루프백 검증 (Claude, 노트북 전용)
- **수정 (Agent 소유)**: `agent_node.py` — real+`allow_missions` 거부, 모드명 정확 일치(Real/foo 등 거부), real+`simulated` 거부, sim/mock이 도메인 77·Discovery Server에서 기동되면 거부. 시험 13/13 (v2 Agent 대조 5 FAIL), 기존 회귀 15개 모듈 통과. fixture 기반 3개 시험(`agent_safety_v1` 등)은 수정 전 Agent에서도 동일 오류로 미판정.
- **루프백 18290 + real Agent**: online·reconciled·movement 불허, Lease ACK·만료(10.1 s), Mapping START 409, Goal 409, Manual bench 409, Gate Reset 409/403, Agent 재시작 시 새 boot id, 서버 재시작 시 재연결 — 전부 PASS. 실차 연결은 SKIP(미배포).
- **배포 후보 v3** (`deploy_candidate_agent_v3/`, tar `3bf1e88d…`, v2와 1파일 차이). 토큰 비노출 확인. 18290 ufw 규칙은 root 필요라 제안서만 작성, 미적용. [상세](fleet_control/docs/REAL_LIMO_FLEET_INTEGRATION_PLAN_20261122.md) §14

## 2026-11-22 (11) Stage A 실행 1부: 실차 Agent v3 배포 완료, 18290 연결은 방화벽 단계에서 중단 (Claude)
- **완료(PASS)**: 로봇 재확인(프로세스 0, 모터 포트 미사용), 로봇 `~/backups` 백업(src/install tar) + 노트북 복사·SHA256 일치, v3 검증·스테이징·교체·`colcon build --packages-select limo_robot_agent` rc 0, 로봇 src == v3 FILES.sha256 (32파일, `agent_node.py` `94ade245…`). 프로세스는 시작하지 않음.
- **중단(FAIL/SKIP)**: ufw는 켜져 있으나 sudo 비밀번호가 필요해 규칙 미적용 → 토큰 설치·서버 기동·Agent 연결·ONLINE/Lease/ACK/재연결 시험은 SKIP. 선택지 A(사용자가 ufw 규칙 실행)/B(SSH 역터널, 루트 불필요·서버 루프백 전용)/C(tailnet ACL)는 plan §15.1. [상세](fleet_control/docs/REAL_LIMO_FLEET_INTEGRATION_PLAN_20261122.md) §15

## 2026-11-22 (12) Stage A 완료: 실차 Agent v3 ↔ 격리 서버 18290 (SSH 역터널, Agent 단독) (Claude)
- **PASS (실차)**: 서버 127.0.0.1:18290 전용, 로봇 측 listener도 loopback 전용(`ExitOnForwardFailure`). ONLINE·robot_id pcu·boot id·heartbeat(0.3–0.4 s), Lease 획득·반납·만료(10.1 s), 명령 ACK, Agent 재시작(새 boot id)·서버 재시작·터널 단절 후 재연결, Mapping START/Goal/Manual/Gate Reset 거부(409/403), `limo_base` 미실행·모터 포트 미사용·`/cmd_vel*` 토픽 없음. 토큰은 0600으로 설치 후 삭제.
- **FAIL(발견)**: 실차 Agent v3가 SIGINT/SIGTERM에 종료되지 않고(rclpy 종료 후 `RCLError` 재시도 루프) SIGKILL로만 종료됨 → 11-20 정지 절차 부적합, 수정 필요(Agent 소유).
- 로봇에 남긴 것(삭제·이동 안 함): `~/backups`(이전 Agent 백업), `~/stage_a_v3`, `~/limo_isolated_18290_tools`(임시 도구, 이후 표준 위치로 통합 예정), 격리 상태 디렉터리. 18190·Gate·Nav2·launch 미변경. [상세](fleet_control/docs/REAL_LIMO_FLEET_INTEGRATION_PLAN_20261122.md) §16

## 2026-11-22 (13) Agent 정상 종료 수정(v4 후보) · LIMO 디렉터리 감사 (Claude, 노트북 + 로봇 읽기 전용)
- **원인/수정**: rclpy가 SIGINT/SIGTERM 핸들러로 ROS 컨텍스트만 종료 → Agent 루프가 `RCLError`를 재시도로 처리해 무한 대기(SIGINT가 `SIG_IGN`으로 상속되면 KeyboardInterrupt도 없음, 일반 SIGINT는 `announce_shutdown` 예외로 정리 중단). `agent_node.py`에 신호 핸들러·ROS 컨텍스트 감시·단계별 정리 추가. 수정 전 0/12 → **수정 후 12/12 PASS**(종료 ≈0.2 s, traceback·유령 노드·shm 없음), 재시작 새 boot id, 기존 회귀 17개 모듈 OK. **후보 v4**(`deploy_candidate_agent_v4/`)는 미배포 — 실차 Agent는 v3 유지(SIGKILL 절차).
- **LIMO 감사**: 작업 워크스페이스는 `~/Capstone1_limo/robot_ws` 하나, `~/capstone_ws` 등은 해시가 다른 중복본, `.bashrc` alias(`stop`=/cmd_vel 발행)·문서가 말하는 `~/limo_field_agent_ws` 부재, 백업 3곳 분산, 11-13 실차 run(약 4.1 GB) 노트북 사본 미확인. 표준 구조·LIMO 작업 규칙 초안은 [감사 보고](fleet_control/docs/LIMO_DIRECTORY_AUDIT_20261122.md), 상세는 plan §17. 이동·삭제·신규 폴더 없음.

## 2026-11-22 (14) LIMO 구조 표준화·리팩토링 설계 (Claude, 조사·설계만)
- **핵심 발견**: ① `fleet_control/`(646개 파일: Server·Agent v3/v4·시험 229개·문서)는 현재 브랜치와 `main`에서 **추적되지 않음**(`feat/fleet-control-unified`는 203개짜리 옛 스냅샷) → 버전관리 밖 ② 로봇 `robot_ws` 342개 파일을 노트북과 해시 비교: 소유자 미커밋 3파일(launch·Gate·package.xml)과 Agent `agent_node.py`(v3 vs v4)만 다르고 `limo_scan_normalizer`는 `capstone_ws/src`에 없이 experiments 4곳에만 존재 ③ 11-13 실차 run 21개(5.3 GB)는 로봇에만 있는 것으로 보임(노트북 사본 이름 없음).
- **v4 독립 재검토**: 서버가 응답하지 않을 때 SIGTERM/무시된 SIGINT 2.5 s 종료, 연속 2회 신호는 즉시 종료(rc −15), supervisor SIGINT→SIGTERM→SIGKILL 단계와 호환, 결함 없음. 남은 일은 Watchdog·mock·supervisor에 같은 matrix 확장.
- **설계**: 유지·병합·폐기 후보 분류(사용 증거 기준), 노트북=개발 원본·LIMO=승인 배포 환경 구조, 신규 폴더·백업·배포·원복 규칙, `.bashrc` 위험 alias 금지 규칙, 건물·층·지도·RMF·3D가 공유하는 공간 모델(층별 `T_building_from_map` 하나만 저장, RMF 변환은 파생), 단계 R0~R8과 Mock/격리 ROS/Gazebo/실차 검증 기준. 실행·이동·삭제·커밋 없음. [설계](fleet_control/docs/LIMO_REFACTORING_DESIGN_20261122.md)

## 2026-11-22 (15) R0 기준선 검토 (Claude, 읽기 전용; 복사·git 작업은 승인 대기)
- **원본 보존 상태**: `fleet_control/`(647 파일)는 git 객체도 스냅샷도 없음(옛 스냅샷: `feat/fleet-control-unified` 203, `origin/main` 197은 **별도 계보**로 작업트리에 없는 17개 파일·다른 35개 파일 보유). 11-13 실차 run 21개 5.63 GB는 노트북 사본 없음(13 MB만 이름·크기 일치), 11-18 `run_001/002/003`은 사본 있음, `run_004`와 복구본(1.77 GB)은 미확인. 로봇 원본 총 약 8.3 GB 중 0.88 GB만 노트북에 있음.
- **Git 후보/제외**: 후보 607 파일 14.5 MB(토큰 값 0건, 이메일은 example.com뿐), 제외 188(secrets 5, data, pycache, evidence). 검토 플래그: 절대경로 16, tailnet IP 5, 이미지 77. **git user.email이 개인 gmail**이라 커밋은 noreply 명시 필요(AGENTS §17).
- **전략**: 체크아웃 없이 plumbing으로 unified 위에 스냅샷 커밋→신규 브랜치(G1), `origin/main` 병합은 별도 워크트리(G2), 대안은 파일 스냅샷. 상세·복사 절차: [R0 보고](fleet_control/docs/R0_BASELINE_20261122.md), 문서 변경 제안: `COMMON_DOCS_PROPOSAL_R0_20261122.md`.

## 2026-11-22 (16) R0 실행: 실차 원본 보호 복사·Fleet 파일 스냅샷 완료, G1 미실행 (Claude)
- **실차 원본**: `local_data/robot_originals_20261122/` 7.7 GB, 36개 항목 8,233,753,902 B를 로봇 매니페스트와 파일별 SHA256·크기·개수로 검증하고 복사 후 노트북 단독 재검증 36/36 PASS. 9개 항목은 노트북에 이미 동일 SHA256 사본이 있어 재사용(11-18 run_001/002 등). 로봇 원본 935개 파일 모두 그대로(누락 0, 크기 불일치 0). rosbag 23개 중 22 PASS, **`20261119_run_002`는 원본이 손상**(SQLite 헤더 83,047쪽 vs 실제 81,455쪽, 끝 6.5 MB 없음, 복사본은 바이트 동일). 비밀정보(`~/.config/limo_*`, video_kit/secrets)는 복사하지 않음.
- **Fleet 스냅샷**: `local_data/fleet_control_snapshot_20261122/` 11 MB (소스 610 + 증거 33 + 공통문서·소유자 파일 20 + 소유자 미커밋 패치; 토큰·data는 이름만), 해시 목록·복원 시험 PASS. 647/607/188 집계 설명은 [R0 실행 보고](fleet_control/docs/R0_EXECUTION_20261122.md).
- **Git**: 최종 후보 609개(603 그대로 + 6 수정 후; 보류 1: 실제 카메라 프레임 `live-api.jpg`); 편집 필요: tailnet IP 문서 4 + 복사 도구, 시작 스크립트 기본 URL. 이미지 77개 전수 확인(사람·사생활 이미지 없음, 1개 보류). commit/워크트리/브랜치 이동 없음.

## 2026-11-22 (17) G1 준비 완료, 커밋 미실행 (Claude)
- **최종 후보**: 612개(작업트리 612 + G1 준비 문서 1 − `live-api.jpg`; 매니페스트 `g1_prep_01/g1_final_manifest.tsv`). `live-api.jpg`는 제외·원본 보존(스냅샷 3개 모두 포함). 발견: `feat/fleet-control-unified`가 이미 이 이미지를 추적하고(`ed53d81b`, 원격 미푸시) 로컬 전용 커밋 2개가 있어 unified를 부모로 하면 이력에 남음 → **부모는 HEAD `c804241a`**(이미 공개된 이력).
- **민감정보**: 토큰 값 0, 이메일 `example.com`뿐. 수정안 P1(문서 4개 tailnet IP→자리표시자), P2(복사 도구 `--host` 필수), P3(`start_robot_agent_field.sh` 서버 URL·토큰 경로 필수 인자, `bash -n`·rc 2 시험), P4(noreply 조각), P5(`live-api.jpg` ignore) — 적용 전 patch만(dry-run 통과).
- **G1 리허설**: 임시 객체 저장소에서 tree `740fd17b…`(612 추가, HEAD 대비 변경 0, 금지 경로 0, 토큰 0), 임시 shared clone에서 실행 스크립트 end-to-end(noreply 작성자·커미터, 부모 HEAD, 롤백 확인). 실제 저장소는 변경 없음(브랜치 0, HEAD·상태 동일). [준비 문서](fleet_control/docs/G1_PREPARATION_20261122.md)

## 2026-11-22 (18) G1 스냅샷 커밋 생성 완료 (Claude, 로컬 브랜치만)
- **커밋** `0ca09dc41a516b8a6953073b214ec2347536c987`, tree `740fd17b2215658583ac311ab5f166a13deaf6cb`, 부모 `c804241a`, 브랜치 `refs/heads/r0/fleet-control-snapshot-20261122`(로컬 전용, push·PR 없음). 작성자·커미터 모두 GitHub noreply(개인 gmail 0). `fleet_control/` 612개 파일 전부 추가(A), HEAD 대비 수정·삭제 0.
- **보존 확인**: 실행 전후 HEAD·현재 브랜치·실제 Index(cached diff)·작업트리 상태·tracked diff·stash·다른 ref 모두 동일(ref는 새 브랜치 1개만 추가: 43→44). 승인된 패치 P1–P5는 파일 8개에만 적용(해시가 리허설과 일치), 다른 에이전트 소유 파일 변경 없음.
- **검증**: 커밋의 612개 blob = 매니페스트와 SHA256·blob id 일치, 스냅샷 03과 비교해 604 동일·8 다름(= 패치한 8개), 스냅샷에만 있는 파일은 `live-api.jpg` 1개. 금지 경로 0, 토큰 값 0, 노트북·로봇 tailnet 주소 0, noreply 조각·gmail 0. `live-api.jpg`는 새 브랜치에서 도달 불가(원본은 디스크·스냅샷에 보존, `.gitignore` 규칙 적용).
- **G2 차단 조건**: `feat/fleet-control-unified`(`ed53d81b`, 로컬 전용 커밋)는 `live-api.jpg`를 이력에 갖고 있어 **단순 병합·cherry-pick·push 금지**. 상세 `local_data/raw_results/real_limo_integ_claude_20261122/g1_exec_01/G2_BLOCKERS.md`.

## 2026-11-22 (19) Agent v4 실차 배포 준비 (Claude, 로봇 변경 없음)
- **로봇(읽기 전용)**: v3 설치 그대로(32파일 해시 일치), ROS/limo_base/Agent 프로세스 0, 서버 포트 listener 0, 모터 포트 사용자 0. 기존 `~/backups`는 pre-v3 백업이라 v4 배포 전에 **v3 새 백업**이 필요.
- **v3→v4**: 32파일 중 `agent_node.py` 1개만 다름(`94ade245`→`6a8252da`). v4 tar = 작업트리 = G1 커밋 `0ca09dc4`(바이트 동일).
- **v4 아티팩트 검증(노트북)**: 종료 matrix 12/12, 실제 모드 생명주기(online·Lease·ACK·거부 4종·SIGTERM·offline·재시작 새 boot id·SIGINT) 통과, 회귀 17개 모듈 OK. 배포·원복 도구 `limo_package_deploy_20261122.sh`(삭제 없음, 해시 검증, 단일 패키지 빌드)를 스크래치 워크스페이스에서 배포→원복 리허설, 중단 조건 시험(프로세스·변조 tar·백업 없음) 통과. 로봇에는 읽기 전용 `preflight`만 실행.
- 중단 조건·변경 범위·승인 순서: [배포 준비](fleet_control/docs/AGENT_V4_DEPLOY_PREP_20261122.md). 실제 교체·기동은 별도 승인 대기.

## 2026-11-22 (20) Agent v4 실차 격리 실행 검증 PASS (Claude, 승인·현장 안전 확인 후)
- 21:59–22:05 KST, 격리 서버 127.0.0.1:18290 + SSH 역터널(loopback 전용), 토큰 0600, Agent v4 단독(mode=real, allow_missions=false), 운영 18190·모터 포트·limo_base·Gate·Nav2 등 미사용. **ONLINE·heartbeat·Lease(획득·반납·만료 10.1 s)·ACK·거부 4종(12/12 ×2)·서버 재시작·터널 단절 재연결·재시작(새 boot id) 전부 PASS**, `movement_permitted` 항상 false.
- **실차 정상 종료: SIGINT 0.164 s, SIGTERM 0.162 s, 종료 코드 0** (v3는 SIGKILL 필요했음). 잔여 Agent/ROS 프로세스 0, `/cmd_vel*` 토픽 0, 모터 포트 사용자 0. shm은 Stage A의 v3 SIGKILL 잔재 2개만 남음(v4 유래 없음). 원복 불필요, v4 유지. 상세 [배포 문서 §8](fleet_control/docs/AGENT_V4_DEPLOY_PREP_20261122.md).

## 2026-11-22 (21) v4 검증 재판정·R1 준비 (Claude, 읽기 전용)
- 실차 추가 실행 없음. v4 설치·백업 불변. 16/16 항목 PASS; "30 s" 조건은 기점 모호로 미결정(정의 정정 필요); 실행 스택 동시 시험은 SKIP. shm 잔재 귀속 오류 정정(데몬 추정).
- 로봇 시작 스크립트 정리 제안(P6 STATE_DIR 필수) 및 R1/R2/R3 최소안은 [V4_VERDICT_AND_R1_PREP](fleet_control/docs/V4_VERDICT_AND_R1_PREP_20261122.md). 로봇 스크립트·저장소 소스 변경, commit/push/PR 없음.

## 2026-11-22 (22) R1.1·P6 (Claude)
- P6 노트북 스크립트 적용(미커밋, 로봇 사본 미변경), R1.1 계약 테스트 추가(11/11 PASS). T_agent 30 s / T_launch 90 s 잠정 기준 정의. R1.2 패치는 스크래치 시험만 완료, 미적용. Watchdog 소스 미변경(R2 항목 보존).

## 2026-11-23 (23) R1.4.1 (Claude)
- Codex R1.4 재감사의 P1 2건 수정 완료(작업트리, 미커밋): Agent가 서버 protocol_version을 boot에 결합해 보관·엄격 검사, 비호환 Mapping Worker의 기존 세션 STOP만 전달. 회귀시험 21개 추가(수정 전 FAIL → 후 PASS), P2 스텁 지원 보완. 실차·배포 없음. R4 모터 잠금 통합은 Codex 재감사 및 별도 승인 전까지 보류.

## 2026-11-23 (24) R1.4.2 (Claude)
- Codex R1.4.1 감사의 신규 결함 2건 수정(작업트리, 미커밋): 상태 응답 safety_contract_version 매번 검증, STOP poll에 session_id 결합. 신규 시험 40개. 실차·배포 없음, R4 자동 승인 아님.

## 2026-11-23 (25) R1.4.3 (Claude)
- Codex R1.4.2 경합 결함 수정(작업트리, 미커밋): Agent 계약 세대, 수신 시점 승인 고정, 활성 goal 취소 의무 래치. 결정적 경합 시험 15개. 실차·배포 없음, R4 자동 승인 아님.

## 2026-11-23 (26) R1.4.4 (Claude)
- Codex R1.4.3 P1 2건 수정(작업트리, 미커밋): Stage 3 검사·제출 시작을 권한 변경과 같은 fence로 직렬화, reconciliation_required를 권한 철회로 처리. 실차·배포 없음, R4 자동 승인 아님.

## 2026-11-23 (27) R1.4.5 (Claude)
- Codex R1.4.4 P1 2건 수정(작업트리, 미커밋): HTTP 오류·예외 경로를 fence·세대 규칙에 통합, reconciliation_required는 정확한 False만 인정 + 서버 정상 상태 응답에 명시 False. 권한 필드 전수 조사표 작성. R1 종료는 Codex 재검증 대기, R4 자동 승인 아님.

## 2026-11-23 (28) R1.4.6 (Claude)
- Lease를 매 상태 응답에서 다시 읽고 권한에 포함(누락·null·구조 오류·만료 = 철회, owner/epoch 변경 = 새 세대). 필수 승인 필드 매트릭스 시험 추가. R1 종료는 Codex 재검증 대기. 작업트리, 미커밋.

## 2026-11-23 (29) R1·R2·R3 오프라인 종료, R4 차단 유지 (Claude, 문서 통합만)
- **R1 안전 계약(R1.4.1~R1.4.6): 오프라인 종료.** Codex R1.4.6 독립 검증 PASS(`local_data/raw_results/codex_r146_close_20261123_6ZGjQf/`). 전체 discover는 PASS가 아님(2 FAIL·33 ERROR, 환경 원인, (29) VALIDATION 참조).
- **R2 Watchdog 종료 안전: 오프라인 종료.** Codex 독립 검증 PASS(격리 ROS 2 Humble 66/66, `codex_neon_r2_r3_close_20261123_6v3X5W/`). 실제 로봇의 물리 정지·제동거리 미검증.
- **R3 배포 도구: 오프라인 종료(진단·복구 범위).** Codex 독립 검증 PASS(11/11). 패키지 교체의 두 디렉터리 이동은 원자적이지 않고, 이중 실패 시 `WS/src/PKG`가 없을 수 있어 수동 복구 필요 — 현장 배포 차단 조건으로 유지.
- 모두 격리·Mock·격리 ROS 결과이며 **실제 LIMO 검증 아님**. 제품 코드는 작업트리에만 있고 미커밋(HEAD `c804241a`, G1 `0ca09dc4`).
- **R4 차단(미완료):** Fleet 서버 선배포·Agent v5 실차 배포 승인, R3 이중 실패의 현장 안전 복구, 실제 모터 잠금 스택 통합, Mapping STOP·Arbiter·Watchdog·Gate·DDS 현장 검증, `/cmd_vel` 우회 차단, Depth 연동, 전원·USB/UART·STM32 watchdog, 물리 E-Stop·제동거리, 보조 실행 주체(Arbiter·bench·Gate reset·localization worker)의 권한 세대 결합, 같은 boot 내 응답 순서 검사.

## 2026-11-23 (30) Agent v5 오프라인 candidate (Claude)
- R1(core·safety_protocol·ros_adapter)+R2(motion_watchdog·mock_robot_node) 소스로 `limo_robot_agent` candidate 생성: tar SHA-256 `33a9e55d…`, 32 파일, 반복 생성 바이트 동일, v4 대비 변경 5 파일. 문서 [AGENT_V5_CANDIDATE](fleet_control/docs/AGENT_V5_CANDIDATE_20261123.md). `deployment_approved=false`, 소스 미커밋.
- 실차 배포 BLOCKED 유지(서버 선배포 승인, R3 이중 실패, 모터 잠금 스택 검증 등 R4 조건 전부 미완료). R1.4.6 문서의 매트릭스 수 41→39 정정.

## 2026-11-23 (31) Agent v5b candidate: 의존성 선언·격리 빌드 (Claude)
- `package.xml`에 geometry_msgs·std_srvs·rcl_interfaces·python3-yaml 추가(직접 import 확인, 코드 무변경). 격리 colcon build PASS, 진입점 3개 설치·모듈 import 실패 0. candidate v5b tar `c71b166f…`(v5 대비 변경은 `package.xml` 1개). [문서 §8](fleet_control/docs/AGENT_V5_CANDIDATE_20261123.md). 소스 미커밋, 실차 배포 BLOCKED 유지.

## G2 병합 (2026-11-23): origin/main `ea2d9632` 판의 이 문서에만 있던 줄
> 통합 브랜치(v5b) 판을 기준으로 두고, origin/main 판에만 있던 줄을 원래 순서대로 아래에 보존한다. 날짜가 이른 기록(11-14~11-18)이며 위 본문의 최신 상태와 다르면 위 본문이 우선한다. 원문: `git show ea2d9632:CURRENT_STATUS.md`.

### 2.3 중앙 관제 · Robot Agent · Open-RMF (`fleet_control/`, PR #9 `main` 병합 완료)
| 중앙 관제 서버 (`fleet_server`) + 제어권 Lease + 복구 | 브랜치: `main` (`a8132c7`, PR #9 `b350efa` 병합 · 통합본 `ed53d81` 보존)<br/>로컬: `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/server` | 구현·통합 완료 | **모터 잠금 연동 검증** (`18190` 서버에 실차 Agent 접속, 상태·지도 API 업로드 및 `map.png` HTTP 200 응답 확인) | **검증 완료** (이전 기준선 `run_gazebo_checks.py` **69/69 PASS**; PR #9 격리 검증에서는 외부 `$HOME/rmf_isolated`/Gazebo 런타임 미비로 미재실행) | **검증 완료** (`run_checks.py` 46/46, `test_ownership.py` 37/37, `test_recovery.py` 14/14, `test_dedupe.py` 14/14, `test_agent_recovery.py` 2/2, `test_cancel_results.py` 14/14) | - 과거 `run_checks.py` 1회 원인 미상 42/46 실패 기록 있음 (`419e798` 브랜치 `fleet_control/docs/VALIDATION.md`)<br/>- 실차 시험 시 브라우저 UI 화면 육안 확인은 아직 사용자 미회신 (`experiments/real_limo_field_20261113/FIELD_RESULTS_20261113.md`) |
| LIMO Robot Agent (`limo_robot_agent`) | 브랜치: `main` (`a8132c7`, PR #9 `b350efa` 병합 · 통합본 `ed53d81` 보존)<br/>로컬: `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/agent` (실차: `~/limo_field_agent_ws`) | 구현·통합 완료 | **모터 잠금 상태·지도 보고 검증** (신규 Agent를 실차 `~/limo_field_agent_ws`에 오버레이 빌드 후 `409 agent_session_mismatch` 해결 확인) | **검증 완료** (`mode=sim`, `SIM_GATED` 주행·취소·복구) | **검증 완료** (`test_sim_motor_policy.py` 12/12 등) | **실차에서 관제/RMF를 통한 실제 이동 임무 실행은 미검증** (`allow_missions=false` 유지) |
| Open-RMF Fleet Adapter (`limo_rmf_adapter`) | 브랜치: `main` (`a8132c7`, PR #9 `b350efa` 병합 · 통합본 `ed53d81` 보존)<br/>로컬: `<WORKSPACE_PARENT>/Capstone1_limo/fleet_control/rmf_adapter` | 구현·통합 완료 | **미검증** (`lf_t2_rmf.sh`는 Gazebo 그래프 기준이라 실차 좌표 `(20,10)`에서 등록 거부됨; `experiments/real_rmf_20261113` 단독 오프라인 등록 시험 40/41 — §4 unconfirmed 타이밍 1건 미해결) | **검증 완료** (이전 기준선 `run_gazebo_checks.py` **69/69 PASS**; PR #9 격리 검증 미재실행) | **부분 검증** (`run_rmf_checks.py` 47/51 — §7 mock 타이밍 레이스, `test_runner_policy.py` 33/33) | - `run_rmf_checks.py` §7(47/51)은 원본·통합본 공통으로 `wp_e` 근거리 재지정에 따른 mock 조기 완료 레이스 확인<br/>- Adapter 재시작 시 이전 RMF 작업이 `underway` 고아 상태로 남는 한계 미해결 |
### 2.4 대시보드 · 실시간 영상 AI (`fleet_control/`, PR #9 `main` 병합 완료 · 공개 173개 파일)
### 2.5 기록기 (`run_recorder.py`, PR #10 `main` 병합 완료)
- **PR #10(`a8132c7`)** 반영으로 `--root` > `LIMO_RUNS_DIR` > 저장소(`capstone_ws/src/limo_capstone/`) 내 `local_data/raw_results/limo_runs/`(신규 clone 포함) > 저장소 밖 `~/limo_runs/` 우선순위가 확정되었습니다.
- 격리 합성 시험 **16/16 PASS** (실제 `rosbag2` 기록 및 실차 온보드 실행은 **미검증**).
## 2026-11-18 실차 Depth와 수동 맵핑 현황
실차 비주행 시험에서 촬영 시각 TF 대기 수정 후 Depth 오류 상태 표본은 214/270에서 38/5,976으로 감소했다. 이후에도 USB–UART 통신 중단과 Gate FAILSAFE가 재발해 원인은 미확정이다. 실차 주행 전 전원·USB와 물리 정지 수단 검증이 필요하다.
웹 수동 입력은 모터 잠금 시험 단계다. 웹 AUTO START의 HTTP 404와 일반 Fleet 임무 경로의 Mapping Goal 409는 미해결이며, 웹 자동 맵핑 실차 주행은 검증되지 않았다.
2026-11-18 선택 통합에서 Frontier 후보 발행기와 Fleet Coordinator를 추가했다. 이 코드는 Agent 실행 경로에 연결되지 않았고 웹 AUTO START 404 및 일반 Fleet 임무를 통한 Mapping Goal 409는 미해결이다. 실차 USB–UART 불안정과 웹 MANUAL 제어 미완료 상태도 유지한다.
2026-11-18 추가 통합에서 실험 관리자의 지도 저장은 고정 스냅샷·셀 검증·원점 정밀도·식별 해시를 사용한다. 격리 합성 지도 Nav2 재로드만 검증했으며 실차 posegraph와 동일 시각의 지도라는 보장은 없다.
2026-11-18 비실차 개발에서 Explorer UUID별 Goal 취소와 Fleet의 외부 Goal 관측 차단을 검증했다. 동일 Action 서버에 동시에 처음 제출하는 경우의 원자적 중재는 없으므로 현장 LOCAL_AUTO_MAPPING은 Explorer만 활성화한다. 합성 ROS 및 모의 영상 호스트에서 지도·RGB·Depth 동시 표시를 검증했으며, 현장 배포본 재설치와 실차 시험은 미실행이다.
웹 Gamepad를 통해 Fleet Lease를 획득한 운영자의 입력을 모의 Agent에서 속도 미리보기로 받는다. 축·Deadman 번호는 브라우저에서 확인한 값을 지정한다. 실물 Gamepad, Gate까지의 ROS 속도 연결, 실제 주행은 UNVERIFIED다.
웹 Mapping/Bag의 명령 계약과 UI는 Mock 검증 단계다. 요청은 QUEUED/ACCEPTED와 실제 완료 보고를 구분한다. Goal terminal 누락·저장 중 worker 연결 유실은 UNKNOWN으로 차단하고 YAML/PGM과 posegraph 결과를 별도로 표시한다. 실제 Frontier·SLAM 저장·rosbag 프로세스 연결, Gate 속도 연결 및 실차 적용은 미구현 또는 미검증이다.
격리 전용 Bag Worker는 실차 기본 비활성이다. 웹 START/STOP과 rosbag2 기록·재생을 격리 시험했으며, 장애 주입 회귀가 완료되지 않아 Phase 3A 전체 완료로 간주하지 않는다.
Bag Worker의 독립 recorder 보호와 비정상 종료 판정은 격리 ROS에서 부분 통과했다. Agent 강제 종료 후 살아남은 recorder에 대한 독립 자원 watchdog 및 재시작·연결 단절 전체 시험은 미완료다.
전체 기능 구현 이전에 Agent 강제 종료 후 Bag recorder가 무제한으로 남는 결함을 확인해 `isolated_ros_bag` 기동을 차단했다. 이후 독립 Supervisor, 실제 Mapping Worker 및 Gate 연결 Velocity Arbiter를 격리 경로에 추가했다.
### 후속 전체 기능 구현 결과
격리 ROS opt-in으로 독립 Bag Supervisor, 실제 Mapping Worker, 공유 Goal 중재, 모터 잠금 Velocity Arbiter를 연결했다. 한 관제 화면에 모드·Lease·Gate·명령 상태와 로봇별 RGB·Depth를 표시한다. 새 `limo_fleet_bringup`의 모듈형 launch는 기존 ROS 실행 파일을 사용한다. 실제 SLAM·Nav2·rosbag2를 합성 센서 환경에서 짧게 실행해 Mapping 수명주기, 지도·posegraph 저장과 Bag 정상 종료를 확인했다. 세부 범위와 잔여 검증은 [VALIDATION.md](VALIDATION.md)와 [ISOLATED_E2E.md](fleet_control/docs/ISOLATED_E2E.md)를 참조한다.
### A–E 잠금 통합 검증 결과
실제 Chrome·ROS 2·Gazebo Fortress에서 관제 명령, Goal 중재 및 수동 전환, Bag 장애 종료, Gazebo 센서와 Nav2 장애물 우회 계획을 확인했다. 세션 저널의 Supervisor 강제 종료 판정, FollowPath 종결 조건, 저장 지도 미관측 셀 분류, 하네스 프로세스 정리를 수정했다. Gazebo 폐루프 주행과 실차·실물 Gamepad 시험은 대기다. [VALIDATION.md](VALIDATION.md)

## 2026-11-23 (32) 로컬 통합 브랜치 생성 (Claude, push 없음)
- 브랜치 `integration/fleet-agent-v5b-g1-20261123`(부모 G1 `0ca09dc4`)에 로컬 커밋 4개: R1 `b6d2c651`(43 파일), R2 `a61a9e8c`(4), R3 `126972e0`(5), 공통 문서 `40b666ec`(8, Tailnet 주소를 `<TAILNET_DISCOVERY_IP>`로 치환). staging·G1 ref·작업트리 불변, push/PR 없음. v5b 소스 32개는 커밋 트리와 `FILES.sha256` 일치.

## 2026-11-23 (33) G2 origin/main 통합 (Claude, G2 worktree, 미커밋)
- worktree `<WORKSPACE_PARENT>/Capstone1_limo_g2_20261123`, 브랜치 `integration/fleet-g2-main-v5b-20261123`에서 `origin/main`(`ea2d9632`) 병합 진행(충돌 77개 해결, merge commit 미생성). 상세 [G2_INTEGRATION](fleet_control/docs/G2_INTEGRATION_20261123.md).
- 보존한 main 기능: Bag 세션(BagWorker/Supervisor), 카메라 미리보기, 게임패드·Operations UI, `limo_fleet_bringup`, capstone_ws 실차 launch의 manual probe 옵션 등. R1·R2는 v5b 기준 유지, Frontier goal에 R1 권한 추가, bringup에 Watchdog 연결.
- 오프라인 검증: R2 66/66, R3 11/11, 루프백 36/36, 격리 ROS 3/3, colcon 3 패키지 PASS. **미해결 FAIL**: main `test_mapping_session` 2건(설계 차이), main velocity/bench·`test_cancel_results` 일부(API 대체). 실차·R4 BLOCKED 유지.

## 2026-11-23 (34) G2 UNKNOWN·Frontier·Cancel 조정 (Claude, G2 worktree, 미커밋)
- UNKNOWN Mapping 세션: goal 소유(승인)와 추적 분리, 활성 goal은 서버가 추적(식별 증거·경고)만 하고 Agent가 취소; 늦은 result는 409. Nav2 상태 publisher 유일성 규칙 추가(v5b 회귀 수정). main Cancel 17/17, Mapping 5/5. **BLOCKED 잔여**: main bench/Arbiter 프로토콜 차이(시험 2개) — 결정 필요. 상세 [G2_INTEGRATION §6](fleet_control/docs/G2_INTEGRATION_20261123.md).

## 2026-11-23 (35) 현장 실행 준비 (Claude, G2 worktree, 미커밋; 실차·18190 없음)
- 문서 [FIELD_READINESS_20261123](fleet_control/docs/FIELD_READINESS_20261123.md): 구성, 설치 대상(노트북/LIMO), 웹 기능별 검증 수준, 안전 체인, 배포·롤백, 현장 절차.
- 새 도구: `field_laptop_up_20261123.sh`(서버 1줄 기동), `field_preflight_20261123.py`(읽기 전용 점검, 시험 5), `atomic_swap_20261123.py`(R3 이중 실패 해소: `renameat2(RENAME_EXCHANGE)`, 시험 10), `probe_field_chain_20261123.py`(안전 체인 20/20). G2 agent 후보 `deploy_candidate_agent_g2`(36 파일, tar `0cb04888…`).
- 격리 E2E(G2 소스): 모드·Mapping 17/20, 저장 지도→Nav2 14/18, supervisor 기동 7/7(+1 SKIP), 안내·순찰 UI E2E 미실행. 안전 체인(그래프·Arbiter·Watchdog·고장 주입) 20/20, 브라우저에서 Field Ops 패널 결함 3개 발견·수정.
- **차단 확인**: Gate가 `/cmd_vel_raw` 외부 발행자를 막지 않는다(Gate 입력 출처 검증 r11+ 미적용). 실차 모터·Mapping은 코드 수준으로 차단된 상태 그대로(REAL_LOCKED, `mapping_real_blocked`). 실차 판정 **BLOCKED**, 잠금 연결·관측·준비는 READY.

- (36) Gate 입력 출처 검증을 G2 Gate·bringup에 이식(`fleet_control/docs/GATE_INPUT_SOURCE_G2_20261123.md`). 격리 시험: Gate 단위 24, 체인 프로브 29/29, g01/g02/f07b FAIL→PASS. 실차 launch에는 Watchdog이 없어(제안 02b 미적용) 검증이 꺼져 있고 벤더 노드의 `/cmd_vel` 직접 발행은 막지 못한다. g03 미검증, n03/n05/f08은 Gate와 무관한 후속 과제. 실차 READY 아님.

- (37) Phase A 일부: 실차 launch가 벤더 드라이버 입력을 `/limo_driver/cmd_vel`로 격리하고(Gate만 해제 시 기록), Nav2·조이스틱을 Watchdog 입력으로 연결, Watchdog 추가, Gate 입력 검증 켬. 정적·격리 ROS 시험만, 실차 launch 미실행. 속도 설정 계약은 제안만(`PROPOSAL_SPEED_PROFILE_CONTRACT_20261123.md`); 웹 지도·속도 UI는 Antigravity 담당. 미완: 배포 후보 재생성, 실차 잠금 절차, 웹 E2E, n03/n05/f08.

- (38) Phase A 고장 주입(실제 launch의 Gate·Watchdog·driver 리매핑 추출 + Mock 드라이버, 격리 ROS) 22/22 PASS, 실차·vendor limo_base·STM32 미검증. 후보 `limo_robot_agent`(SHA `b3603242e82e3150e65753cc51c0fadf0755af4ffa1becb8d6028fe288542eb1`, 텔레메트리 포함 최종)와 `limo_capstone`(SHA `e6488cee…`)을 생성, 격리 swap·rollback 리허설 통과. `state.velocity` 텔레메트리를 Agent에 추가, 속도 제한 API는 제안(Humble smoother 파라미터 변경은 효과 없음 실측). 실차 READY 아님.

- (39) 속도 제한 구현(G2 미커밋, 격리 ROS·mock Agent만): Agent `speed_limits.py` + Arbiter 최종 clamp(manual 0.10/0.30, auto 0.15/−0.10/0.50 상한 이하로만 낮춤, Lease·재시작 후 stale이지만 낮은 값 유지), 서버 `POST/GET /api/v1/robots/{id}/speed-limits`(PENDING→Agent 보고 후에만 approved=true). **실차 Agent에는 Arbiter가 없어 강제하지 못함(`agent_cannot_enforce`)**; 실차 강제는 Watchdog clamp 후속. Nav2 smoother 파라미터 방식은 효과 없음, `SpeedLimit` 구독만 확인(효과 미측정). `/limo_driver/cmd_vel` 외부 발행자는 ARMED로 래치(감지만, 차단 아님). 실차 READY 아님.

- (40) 실차 Watchdog 속도 clamp 구현(G2 미커밋, 격리 ROS·mock만): `speed_clamp.py` + `watchdog_speed_link.py`, 기본 꺼짐·실차 launch에서만 켬. 정적 ceiling과 낮추기만 하는 승인 한도 분리, Agent 동기화 필수(`SPEED_LIMIT_NOT_SYNCED`), 재시작 시 낮은 값 stale 복원, 높임은 메시지 2회+게시자 1개, Agent는 Watchdog이 같은 generation을 보고한 뒤에만 `APPLIED`. 가감속 제한은 추가되지 않음(출력이 계단), 한 축 포화 시 곡률 변화(측정 1.0→2.0 1/m). 실차 READY 아님.

- (41) Phase C 통합(G2 + Antigravity UI, 격리 ROS·mock, 미커밋): 통합 복사본(스크래치패드, 새 worktree 없음)에서 두 작업트리 변경이 충돌 없이 합쳐졌고, 실제 Fleet Server·Agent·Watchdog·Gate·내장 브라우저 E2E 43/43. 서버가 Watchdog 증거(generation·request_id·값·boot·GID·나이)를 직접 검증해야만 `approved=true`(`server_verified`/`applied_match` 분리). 통합 중 서버 HTTP 500 결함 등 4건 수정. 실차 Agent는 운영 모드를 확인할 수 없어(Arbiter 없음) 웹 Mapping/Navigation은 불가하고, Nav2 경로 추종·곡률 영향은 미측정. 새 배포 후보(Agent `2c46ca16…`, limo_capstone `742d4f36…`, server `eb1468ab…`)와 읽기 전용 실차 preflight 점검표를 준비했으나 설치·접속은 하지 않음. 실차 READY 아님.

- (42) 실차형 Agent(Arbiter 없음)의 운영 모드 진입 구현(G2 미커밋, 격리 ROS·mock만): 원인은 서버가 Arbiter 증거(`locked_velocity_arbiter`)를 요구하고 `VelocityArbiter`가 mock에서만 생성되는 것. Agent `RealModeSelector`(Arbiter 상속·속도 미발행) + Watchdog `mode_gate`(모드별 입력 채널 하나만 전달, 소유 goal·cancel·deadman 재무장·소스 충돌) + 서버의 증거 재검사. E2E 45/45(MAPPING 시작·PAUSE·RESUME·SAVE·STOP, MANUAL, MAPPING↔MANUAL, 충돌·중복 게시자, Lease 상실, Watchdog·Agent 재시작, 서버 정지)와 NAVIGATION 진입 12/12. Fleet 미션 goal·취소는 실차형 Agent에서 미검증. 실차 READY 아님.
- (43) 실차형 Agent 모드 E2E 확장(격리 ROS, 미커밋): 소스 침묵·Watchdog 정지(SIGSTOP)·Agent 정지·Gate 종료/재시작·중복 게시자를 추가. 전체 57항목 중 두 번의 완주가 각각 56/57(실패: ① 중복 게시자 확인의 시험 타이밍 → 수정 후 통과, ② 재시작한 Gate가 /cmd_vel_raw에 0.25 s 초과 공백이 생기면 명령 중에도 NORMAL로 무장하는 설계 동작 때문에 "명령 중 무장 안 함" 단언이 1회 실패). ②는 Gate 정책(공백=무명령)과 시험 단언의 불일치이며 해결/수정하지 않음. 실차 READY 아님.
- (44) Gate 재시작 56/57 해결 + 서버 ACTIVE 모드 지속 검증(격리 ROS·mock만, G2 미커밋): 원인은 Gate가 아니라 Watchdog — 비인가(LOCKED) 구간 동안 눌려 있던 조이스틱이 Gate 복귀 즉시 해제 없이 재개(수정 전 6회 중 3회 재현). Watchdog에 "AUTHORIZED를 벗어나면 해제(0 또는 0.5 s 침묵) 관측 후에만 MANUAL 재전달"을 추가. 서버는 ACTIVE 모드를 Watchdog 증거(모드·revision·boot·채널·상태·trip 횟수)로 계속 재검사: 불일치면 `mode_unverified`로 신규 운동·임무 거부, 모순 3 s 지속 또는 trip이면 모드 종료(자동 복구 없음), 응답에 `mode_motion` 분리 표시. 모드 E2E 64항목 깨끗한 새 스택 3회 연속 PASS. 열린 항목: launch 하네스 C4 간헐 1/4(원인 미격리), Fleet 임무 승인·취소(실차형 Agent)는 BLOCKED(안전 계약 결정 필요, 제안은 `REAL_AGENT_MODE_SELECTION_20261124.md` §7.4). 실차 READY 아님.
- (45) 조이스틱 재무장·건식 Fleet Mission·C4·AUTO clamp 비교(격리 ROS·mock만, G2 미커밋): ① Watchdog은 이제 **침묵을 해제로 인정하지 않는다**(해제=명시적 0 명령; 눌린 채 0.5 s+ 침묵 후 재개·manual endpoint 재생성·비인가 episode·새 MANUAL 부여는 0 → 새 누름 필요). 모드 E2E 67/67 새 스택 3회. ② `real_nav_dry_run`: 실차형 Agent의 건식 임무(별도 `POST /dry-run-missions`, `dry_run`/`real_motion_authorized:false`, `missions_enabled:false` 유지, 모터 경로 LOCKED일 때만, `locked_motor_proof`=소프트웨어 그래프 증거·물리 잠금 아님). 격리 E2E: 승인·소유권/generation·취소→IDLE·Lease 상실·Watchdog/Gate 종료·AMCL 종료 통과, **f08(Agent 재시작 후 증거 복구)은 미검증**. 실차 전 BLOCKED: 설치된 Gate에 attestation 없음(r13 후보 필요)·r13 reset의 `INPUT_SILENT`. ③ C4 간헐 실패: 감지 지연(+0.35 s 정상 vs +2.7 s 실패, 12회 중 3회), 상향 미수용, 원인 미확정 BLOCKED. ④ AUTO 비례 clamp: 오프라인 비교만(곡률 보존 vs 축별), 최종 clamp 변경은 별도 결정. 실차 READY 아님.
- (46) Gate 생성 증거 이식·reset 정책·f08 복구·C4 조사(격리 ROS·mock만, G2 미커밋; 문서 `fleet_control/docs/GATE_EVIDENCE_AND_DRY_RUN_RECOVERY_20261127.md`): ① r13 후보의 "attestation"은 sim 전용 비인증 계약이라 계약은 이식하지 않고, G2 Gate에 `boot_id`·영속 `fault_epoch`(영속 후 게시)·`fault_latched`를 선별 이식(`evidence_kind: GATE_SELF_REPORT_UNAUTHENTICATED`); r13 파일 없이 건식 admission·취소·소유권 통과. ② `INPUT_SILENT`는 제거하지 않고, Watchdog의 유휴 명시적 0(`idle_zero_period_s`)과 Gate `reset_gating`(단일 writer·최신 0 입력·정지 대기)으로 만족시킴; 0 입력이 없으면 reset 거부(E2E d07), reset 성공은 운동 재개가 아님. ③ f08: 실차 경로는 설계상 `terminal_evidence_unavailable`; 건식 Agent에 locked-path 복구 규칙(명령별 Nav2 terminal 증거 등 전부 유지)을 적용하자 복구·새 goal 통과, 연속 흐름 d06 PASS. ④ C4: 지연의 원인 구간은 Watchdog 프로세스의 새 DDS participant 발견 지연(0.2~2.7 s)으로 확정(CPU·tick 지연 아님), 상향 미수용; DDS 내부 원인·감지 시간 상한은 미확정 BLOCKED. 모드 E2E 67/67 ×2. 실차 launch 기본값은 모두 꺼짐. 실차 READY 아님.

- (47) **최종 4개 작업 통합 (`integration/capstone1-field-release-20261124`, 2026-11-24)**: Claude G2(Real Agent·Watchdog·Gate·Fleet Server·건식 임무) + Antigravity Web UI(`UI_INTEGRATION_PATCH_20261128b.diff` 및 `operating_mode_panel.js` 포함) + Antigravity Navigation O6 안정화(`stack_up.sh` 병합 포함) + Antigravity Agent Harness M0를 하나의 Git 기준선으로 통합.
  - **구현 및 격리 검증 완료**: 실차형 모드 E2E 67/67, Agent Harness M0 18/18, 통합 소스 단위·서버·UI 시험 235/235 PASS, q10 LOCKED 배포 후보 3종(Agent 41 / LIMO 91 / Server 49 = 181/181 파일)과 Git 소스 SHA-256 100% 일치.
  - **부분 검증 (`PARTIAL`)**: 실제 브라우저 E2E(`Mapping -> Save -> Load -> Dry-run Goal -> Cancel`) 핵심 10단계 통과(`WEB_E2E_VERIFIED: PARTIAL`; 다건물 UI 1건물·1층·1구역만 확인, 지도 `VERIFIED` 승격은 UI 밖 예외, Manual UI playwright SKIP). Navigation 운동학(72/72, 7시나리오 24/24) 및 개별 Gazebo(`nav2_d3_targeted.py` 8/8) 통과했으나 **통합본 Gazebo 시험은 미실행(SKIP)**.
  - **실차 미검증 및 차단 (`UNVERIFIED` / `BLOCKED` — `REAL_MOTION_READY: BLOCKED`)**: 실제 LIMO 주행은 **NOT READY**이며, LOCKED 배포 후보가 준비된 상태와 실제 LIMO 설치 완료는 별개(미설치, `deployment_approved: false`)입니다. 아래 10개 항목은 모두 **UNVERIFIED 또는 BLOCKED**로 유지합니다:
    1. 실차 vendor `limo_base` 명령 토픽 리매핑(`/limo_driver/cmd_vel` 단독 구독 여부) — `UNVERIFIED/BLOCKED`
    2. `/cmd_vel` 직접 발행 우회 차단(`limo_ros2_application` 노드 등) — `UNVERIFIED/BLOCKED`
    3. 실제 물리 E-Stop 동작 및 복구 — `UNVERIFIED/BLOCKED`
    4. STM32 명령 타임아웃(상위 노드 전멸 시 모터 정지 여부) — `UNVERIFIED/BLOCKED`
    5. 물리 모터 차단(구동 전원 분리 또는 바퀴 부양 확인) — `UNVERIFIED/BLOCKED`
    6. 실제 제동거리(0.05~0.10 m/s 실측) — `UNVERIFIED/BLOCKED`
    7. DDS 중복 writer 발견 지연(C4, 0.2~2.7 s 관측)의 안전 상한 — `BLOCKED`
    8. 실제 조이스틱 해제 메시지 패턴(0 발행 vs 침묵) — `UNVERIFIED/BLOCKED`
    9. 실차 Nav2 O6 프로파일 거동(시뮬레이터에서만 검증, 복원본 별도 보관) — `UNVERIFIED`
    10. 실차 운동 Mission admission(`real_motion_authorized`) — `BLOCKED` (건식 임무 성공은 실제 자율주행 완료가 아님)

- (48) **실차 LOCKED 1·2차 정지 상태 검증 및 USB Device Resolver 구현 (2026-11-24, Git 반영·실차 주행 없음)**:
  - **실차 LOCKED 배포 및 정지 검증 (q10 설치본 유지)**: 바퀴 부양 상태에서 `limo_robot_agent`(q10 `4327ff37…d1be`) + `limo_capstone`(q10 `af18d9bd…93b9`) 실차 배포·빌드 완료, `allow_motor_drive:=false`, `allow_missions:=false`, `real_nav_dry_run:=false`로 기동하여 격리 노트북 서버(`127.0.0.1:29810`, `18190` 무변경) 및 웹 관제 연결 확인 (`WEB_REAL_ROBOT_CONNECTED: PASS`, `AGENT_DEPLOYED: PASS`, `LIMO_PACKAGE_DEPLOYED: PASS`, `BACKUP_VERIFIED: PASS`).
  - **Watchdog·Agent 동기화 해결 (`field_preflight --real` 25/32 → 29/32)**: 1차의 `B3`/`F1~F3` FAIL은 구 `robot_agent` 잔존 프로세스로 인한 중복 발행자 차단이었으며, 단일 프로세스 그룹 기동으로 `agent_boot_matches=true`, `enforced_by=WATCHDOG`, `server_verified=true`, `agent_verified=true`, `motor_path=LOCKED`, `movement_permitted=false`를 실차에서 확인(**29 PASS / 2 WARN / 1 FAIL**).
  - **미해결 실차 USB 허브 문제 (`BASE_HUB_NOT_ENUMERATED`, 물리적 근본 원인 미확정)**: 재부팅 후 및 최종 읽기 전용 재확인까지도 LIMO 베이스 보드용 USB2.0 허브(`1a40:0101`, 과거 `usb 3-7` → `usb 3-7.4` `ttyUSB1` `/dev/ttylimo`)가 커널 USB 버스 전체에 미열거 상태라는 **관측 사실**이 확인되었으나(YDLIDAR `usb 3-3` `10c4:ea60` `ttyUSB0`만 존재), 전원/케이블/허브 하드웨어 등 **물리적 근본 원인은 미확정**입니다. 이로 인해 `limo_base`가 `/dev/ttylimo` 미열림으로 종료(exit 255)되어 `B1`(Gate `FAILSAFE_SENSOR_TIMEOUT`) FAIL 및 `C4`(배터리 `UNKNOWN`)·`F8`(`/odom` `UNAVAILABLE`) WARN이 잔존합니다. 기존 `/etc/udev/rules.d/99-limo-devices.rules`는 정상이며 OS/udev/펌웨어는 미변경입니다.
  - **USB Device Resolver 구현 (합성·Mock 검증 완료, 실차 미설치)**: `limo_capstone/usb_device_resolver.py` + `limo_integrated_auto_mapping_real.launch.py` 연동 구현(합성 sysfs `14/14`, ROS 2 Humble LaunchService Mock 드라이버 `5/5`, 관련 회귀 포함 `56/56 PASS`, 실차 읽기 전용 프로브 `BASE_HUB_NOT_ENUMERATED` fail-closed 확인; `use_usb_resolver=false`는 안전 검사 우회 경로이므로 정상 운용에서는 `true` 유지).
  - **q12 후보 (`q12_limo_capstone_usb_resolver_20261124`) 배포 보류**: 94/94 파일 소스 일치 및 `stage` 검증은 통과했으나, `limo_package_deploy_20261122.sh artifact` allowlist에 해당하지 않는 파일 32개(`.gitignore`, `2026_10_15`, `maps/*`, `models/*`, `worlds/*` 등; 자격증명 유사 이름 없음)로 인해 정책 도구 기준 미충족이며 수동 생성(`.gitignore`만 tar 제외, 나머지 93개 포함)되었고, 정책 적합성은 미확인이며 `replace`/`build`/`verify`/`rollback` 리허설이 미실행 상태이므로 **배포 보류**로 기록합니다(기존 q10 후보 및 실차 q10 설치본 유지).
  - **`REAL_MOTION_READY`: BLOCKED 유지**.
