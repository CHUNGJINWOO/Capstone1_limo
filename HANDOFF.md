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
