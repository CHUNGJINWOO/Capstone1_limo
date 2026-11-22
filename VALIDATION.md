# VALIDATION.md — 통합 검증 색인 및 대용량 실험 자료(Google Drive · YouTube · rosbag) 관리 규격

> **기준일**: 2026-11-14  
> 이 문서는 **(1) 영역별 검증 결과 및 원본 보고서 색인**과 **(2) 대용량 `rosbag2`·시뮬레이션 영상·실차 주행 영상의 로컬 / GitHub / Google Drive / YouTube 통합 관리 설계**를 정의합니다.

---

## 1. 영역별 핵심 검증 기록 색인

모든 시험 결과는 **실차(Real Robot)**, **Gazebo(물리 시뮬레이션)**, **합성/오프라인(Synthetic/Offline)**으로 구분하며, 상세 수치와 한계는 각 원본 보고서를 따릅니다.

| 검증 영역 | 검증 환경 | 대표 실행 ID (`run_id`) | 핵심 결과 요약 (성공 및 실패·한계 병기) | 상세 근거 문서 (브랜치 / 로컬 미추적 구분) |
|---|---|---|---|---|
| **온보드 안전 체인 결함 수정 후보 (`baseline` vs `fixed` vs `fixed2`)** | 합성 하네스 + Office Gazebo (실차 미검증) | `run03_20261114` (baseline)<br/>`run04_20261114` (fixed)<br/>`run05_20261114` (fixed2)<br/>`office01~03_20261114`<br/>`gate86_20261114` | - 하네스: `baseline` PASS 15 / **FAIL 12** → `fixed`·`fixed2` **PASS 28 / FAIL 0** (기존 Gate 회귀 86/86 PASS)<br/>- Office Gazebo: `fixed`는 일반 탐색 2.18~4.03 m 정지 지속; `fixed2`에서 `explore_rep2` **16.22 m**, `explore_rep3` **18.48 m**, `odom_tf_loss_recover` **20.83 m** 주행 확인 (단, 반경 `0.22/0.40` 시험인 `explore_rep1` 1.55 m 조기 종료 및 `low_box_rep1` `sensor_normalizer_node` exit `-2` 실패 기록 보존) | - 로컬 미추적 보고서:<br/>  - `experiments/safety_chain_fix_20261113/README.md`<br/>  - `experiments/safety_chain_fix_20261113/docs/RESULTS_harness_baseline-run03_fixed-run04.md`<br/>  - `experiments/safety_chain_fix_20261113/docs/DEADLOCK_ANALYSIS.md`<br/>  - `experiments/safety_chain_fix_20261113/docs/MEMORY_POLICY_V2.md`<br/>- 원본 bag/로그(7.0 GB): `experiments/safety_chain_fix_20261113/runs/` |
| **실차 현장 시험 (관제·영상 AI·LiDAR 자동 맵핑·Depth 정지)** | **실차 (LIMO)** (모터 잠금 + `run_010`만 저속 구동) | `limo_run2~4` (영상 AI)<br/>`run_008` (USB 재연결 차단)<br/>`run_010` (LiDAR 구동 맵핑)<br/>`run_012~014` (Depth+RGB 잠금) | - `run_008`: 충전기 분리 직후 베이스 USB(`ttyUSB1→ttyUSB2`) 재연결로 `wheel_odom_stale` 구동 전 차단<br/>- `run_010` (`real_auto_mapping_ready_20261113`): 0.15 m/s로 약 110초 주행, 지도 1,080→30,430셀 성장 후 `FAILSAFE_ICP_DEGRADED` 정지·지도(`map_001`,`map_002`) 저장<br/>- `run_012` (`real_depth_slow_20261113`): 정지 상태 화이트보드 하단 프레임 Depth→local costmap 마킹 확인<br/>- 전원 4회 돌발 차단 발생으로 추가 구동 보류 | - 로컬 미추적 보고서:<br/>  - `experiments/real_limo_field_20261113/FIELD_RESULTS_20261113.md`<br/>  - `experiments/real_depth_integration_20261113/README_DEPTH_INTEGRATION.md`<br/>  - `experiments/real_depth_lidar_mapping_20261113/STATUS.md` |
| **실차 스캔 정규화 (`limo_scan_normalizer`)** | **실차 (모터 잠금 수동 이동)** + 오프라인 bag | `20261112_scan_norm` | 정규화 OFF 시 0.92 m 이동 동안 지도 685셀 고정(경고 5건) 실패 → ON 시 1.06 m 이동 동안 **701→7,531셀 성장(경고 0건)** | - 원인 분석(`main` 공개): [experiments/safety_gate_verification_20261110/slam_scan_fix_20261111/REPORT.md](experiments/safety_gate_verification_20261110/slam_scan_fix_20261111/REPORT.md)<br/>- 실차 기록(`09d742c` 브랜치 및 로컬): `docs/dev_notes/2026_11_12_실차_스캔정규화_배포와_모터잠금_검증.md` |
| **Gazebo 차동구동 자동 맵핑 + 자동 지도 저장** | Gazebo | `auto_mapping_final_20261112` (`run1~3`, `v1~3`) | 6회 실행 중 3회 실내 커버리지 98~99% 완주, **2회 82~85% 조기 종료, 1회 일시적 Depth stale FAILSAFE 정지** | - 보고서(`cc58183` 브랜치): `experiments/auto_mapping_final_20261112/REPORT.md`<br/>- 로컬 데이터(252 MB): `experiments/auto_mapping_final_20261112/runs/` |
| **Office Fortress 월드 + LIMO + Collision Monitor 통합** | Gazebo | `office_limo_02` | 190초 Frontier 탐색 주행 21.1 m, 알려진 셀 10,643→34,802, 최소 LiDAR 거리 0.299 m | - 요약(로컬 미추적): `experiments/office_limo_sim_20261113/README.md`<br/>- 원본 bag: `$HOME/rmf_demos_humble/runs/office_limo_02/bag_explore` |
| **중앙 관제 서버 · Agent · Open-RMF 모의 E2E** | **합성/모의 환경 전용** (Open-RMF 실차 시험 없음) | `20261112_fleet_rmf_mock` | `run_checks.py` 46/46(연속 6회, 과거 1회 원인 미상 42/46 기록), `test_ownership.py` 37/37, `test_runner_policy.py` 33/33, `run_rmf_checks.py` 59/59(연속 3회; 통합 세션 병행 시 47/51 간헐 실패) | - 요약(`419e798` 브랜치): `fleet_control/docs/VALIDATION.md` |
| **Gazebo × Open-RMF E2E** | Gazebo | `20261112_gazebo_rmf` | **69/69 PASS (필수 15/15 PASS)**, RMF patrol → Mission Manager → Gazebo 실제 이동 → RMF `completed`, 취소·소유권·단절·복구 검증 (통합 세션 병행 시 66/68 간헐 실패 기록) | - 보고서(로컬 미커밋): `<WORKSPACE_PARENT>/Capstone1_limo_fleet_rmf/fleet_control/gazebo/RESULTS.md`<br/>- 결과 JSON: `<WORKSPACE_PARENT>/Capstone1_limo_fleet_rmf/fleet_control/gazebo/results.json` |
| **통합 시뮬레이션 (RMF × Gazebo × Nav2 × Video/YOLO)** | Gazebo + CUDA YOLO11n | `final3`, `final4` | 단일 실행 **12/13 PASS** (박스 최소 간격 0.151 m로 보수적 반경 0.155 m 기준 1건 미달 기록), 영상/서버 장애 주입 중 RMF 5/5 완료 및 Gate NORMAL 유지 | - 보고서(로컬 미커밋): `<WORKSPACE_PARENT>/Capstone1_limo_integrated_sim/fleet_control/integrated_sim/RESULTS.md`<br/>- 런타임 로그: `$HOME/.local/share/robot-sim-projects/limo_integrated_sim/runs/` |
| **시각화 대시보드 (`dashboard` / `dashboard_lab` / `rmf_web_research`)** | 합성 + **실차 15초 읽기 전용(분리 창)** + 오프라인 벤치 | `20261113_dashboard_v1`<br/>`20261113_dashboard_lab`<br/>`20261113_rmf_web` | - `dashboard`: 자동 검사 37+83 PASS, 실차 15초 지도·Costmap·TF·Depth 및 별도 15초 RGB/YOLO(5 fps) 표시 확인 (정지 상태라 `/plan`, `/local_plan`은 `UNAVAILABLE`)<br/>- `dashboard_lab`: 오프라인/합성 121 PASS, WS JPEG 11.90 fps / Depth 7.99 fps 달성 (실차 미연결)<br/>- `rmf_web_research`: Storybook 오프라인 fixture 확인 | - 로컬 워크트리(`<WORKSPACE_PARENT>/Capstone1_limo_dashboard/fleet_control/`):<br/>  - `dashboard/docs/VALIDATION.md`<br/>  - `dashboard/docs/REAL_LIMO.md`<br/>  - `dashboard_lab/docs/RESULTS.md`<br/>  - `rmf_web_research/README.md` |
| **`run_recorder.py` 기본 출력 경로 (`--root` > `$LIMO_RUNS_DIR` > 저장소 `local_data/raw_results/limo_runs/` > `~/limo_runs`)** | **합성/오프라인** (`--dry-run`만 사용, 실차·rosbag 기록 미실행) | `20261114_run_recorder_root` | 12/12 PASS: 저장소 내부 실행, `LIMO_RUNS_DIR`, `--root` 우선순위, 저장소 외부·탐색 실패·잘못된 `.git`(검증 실패)·`local_data` 없음 시 `~/limo_runs` 유지, 공백 포함 경로, dry-run 무부작용. **한계**: 실제 `ros2 bag record`, 실차 온보드 환경 실행은 미검증. 이관한 스모크 rosbag `20261110_003944_smoke`는 zstd·SQLite 무결성 정상이나 `messages_count=0`인 **기록 체계 스모크**이며 센서 기록·주행 검증이 아님 | - `local_data/worktree_backups_20261114/limo_runs_cleanup_20261114/` (`test_run_recorder_root.py`, `test_run_recorder_root.result.txt`, `LIMO_RUNS_SMOKE_VERIFICATION.json`, 변경 전 파일·SHA-256)<br/>- 보존 복사본: `local_data/raw_results/limo_runs/20261110_003944_smoke` |

---

## 2. 대용량 실험 자료(rosbag · 영상 · 지도 · 실행 스냅샷) 4계층 스토리지 설계

### 2.1 로컬 · GitHub · Google Drive · YouTube의 역할 분리

| 저장소 | 보관 대상 | 보관 금지 대상 | 보존 목적 |
|---|---|---|---|
| **1. 로컬 워크스페이스** (`local_data/`, `experiments/*/runs/`, `$HOME/.local/share/robot-sim-projects/`) | - 최근 작업 중인 원본 `rosbag2`(`*.db3`, `*.mcap`), 원시 로그(`*.log`, `*.jsonl`), 미편집 영상<br/>- 런타임 DB(`fleet.db`), 인증 토큰(`*.token`, `kit.env`), 학교 행정 제출 원본 | 없음 (단, 민감 파일은 권한 `0600`/`0700` 유지 및 Git 추적 제외) | 즉시 재현·디버깅 및 업로드 전 해시 검증 원본 |
| **2. GitHub 저장소** (`CHUNGJINWOO/Capstone1_limo`) | - ROS 2 / Fleet / Dashboard 소스코드 및 설정 파일<br/>- 실험 요약 JSON/CSV, 정제된 지도(`*.pgm`, `*.yaml`, `*.png`), 마크다운 보고서<br/>- 대용량 자료의 **메타데이터 매니페스트(`manifest.json`, SHA-256, Drive/YouTube 링크)** | - 대용량 `rosbag2`, 영상(`*.mp4`, `*.webm`), 빌드 산출물(`build/`, `install/`)<br/>- **토큰, 비밀번호, IP/SSH 접속 정보, 학번, 개인 연락처, 학교 제출 보고서/발표자료 원본** ([docs/PUBLICATION_POLICY.md](docs/PUBLICATION_POLICY.md)) | 버전 관리 및 코드·검증 근거의 단일 기준점 |
| **3. Google Drive** (팀/프로젝트 전용 아카이브) | - 검증 완료된 주요 실험의 압축 아카이브(`rosbag_<run_id>.tar.zst`, `logs_<run_id>.tar.zst`, **비밀정보 제거 검사를 통과한 `code_and_config_<run_id>.tar.zst`**, `manifest.json`)<br/>- 실차 주행 영상, Gazebo 시뮬레이션 녹화 영상, 시연 영상 보관본 | - **인증 토큰(`*.token`), `*.env`, `kit.env`, `auth.json`, `real_connection.json`, `fleet.db`/`*.sqlite*`, 무검토 원본 `git diff`, 무검토 환경변수 덤프(`env`)**<br/>- `before-privacy-reset` 등 개인정보가 포함된 구 Git 번들 및 학교 행정 제출 원본 | 장기 보존, 미커밋 실험 코드·설정의 정확한 재현, 팀원 간 대용량 데이터 공유 |
| **4. YouTube** (외부 시연 채널) | - 개인정보·민감 화면이 제거(마스킹/블러)된 주요 시연 영상 (Gazebo 통합 시뮬레이션, 실차 주행 시연, 대시보드 구동 화면) | - 원시 미검토 촬영본, 학번·실명·얼굴·개인 화면·내부 IP가 노출된 영상 | 보고서·README·발표 임베딩 및 외부 검증 시연 |

> **재현성 우선순위 원칙 (`rosbag` > 로그/코드 스냅샷 > 영상)**:  
> 디스크 용량이나 업로드 대역폭이 제한될 때는 **1순위: `manifest.json` + `code_and_config_<run_id>.tar.zst`(허용 목록 통과분) + `events.jsonl`/`result.json`**, **2순위: `rosbag2`(`*.db3`/`*.mcap`) 및 저장 지도**, **3순위: 녹화 영상(`*.mp4`/`*.webm`)** 순으로 보존합니다. 영상만 있고 `rosbag2`/로그가 없는 결과는 정량 검증 근거로 인정하지 않습니다.

---

### 2.2 Google Drive 권장 폴더 구조

```text
Capstone1_LIMO_Archive/
├── 00_INDEX/
│   ├── ARCHIVE_CATALOG.tsv                 # 전체 업로드 항목(run_id, sha256, drive_path, youtube_url) 마스터 표
│   └── RESTORE_GUIDE.md                    # 다운로드 및 SHA-256 무결성 검증 절차
├── 01_REAL_ROBOT/                          # 실차(Physical LIMO) 실험 아카이브
│   └── <YYYYMMDD>_<experiment_slug>/       # 예: 20261113_real_limo_field/
│       └── <run_id>/                       # 예: run_010_lidar_drive_slow/
│           ├── manifest.json               # 실행 메타데이터 및 파일별 SHA-256
│           ├── code_and_config_<run_id>.tar.zst # [필수 검열 통과분] 실행 코드·YAML·launch 스냅샷
│           ├── rosbag_<run_id>.tar.zst     # rosbag2 (metadata.yaml + *.db3)
│           ├── logs_<run_id>.tar.zst       # monitor.jsonl, stack.log, result.json
│           ├── maps/                       # map_001.pgm, map_001.yaml 등
│           └── videos/                     # 실차 외관 촬영 및 대시보드/RViz 녹화본 (*.mp4)
├── 02_GAZEBO_SIM/                          # Gazebo 물리 시뮬레이션 아카이브
│   └── <YYYYMMDD>_<experiment_slug>/       # 예: 20261114_safety_chain_fix/, 20261113_integrated_sim/
│       └── <run_id>/                       # 예: office03_20261114_fixed2_explore_rep3/, final4/
│           ├── manifest.json
│           ├── code_and_config_<run_id>.tar.zst
│           ├── rosbag_<run_id>.tar.zst
│           ├── logs_<run_id>.tar.zst
│           └── videos/
├── 03_SYNTHETIC_BENCHMARKS/                # 합성 하네스·오프라인 재생·Dashboard Lab 벤치마크
│   └── <YYYYMMDD>_<experiment_slug>/
│       └── <run_id>/
└── 04_PRESENTATION_AND_DEMOS/              # 개인정보 검토를 마친 시연·발표 영상 (학교 행정 제출 원본 제외)
    └── <YYYYMMDD>_<title>/
        ├── manifest.json
        └── <video_name>_sanitized.mp4
```

---

### 2.3 `code_and_config_<run_id>.tar.zst` 비밀정보 제외 목록(Denylist) 및 허용 파일 목록(Allowlist) 사전 검사 규격

현재 많은 실험(`safety_chain_fix_20261113/fixed2`, `real_limo_field_20261113`, 4개 기능 워크트리)이 Git 미추적(`??`) 또는 미커밋(`M`) 상태에서 수행되므로, `git_commit` 해시만으로는 실행 코드를 재현할 수 없습니다.  
단, **원본 `git diff`, 환경변수 전체 덤프(`env`), 토큰, 런타임 DB를 무검토로 압축·업로드하면 비밀정보와 개인정보가 유출**되므로 반드시 아래 **2단계 필터(Denylist 차단 → Allowlist 선별)**를 통과한 파일만 `code_and_config_<run_id>.tar.zst`에 포함합니다.

1. **절대 포함·업로드 금지 목록 (Strict Denylist)**:
   - 인증 토큰·키·비밀번호 파일: `*.token`, `*.key`, `*.pem`, `*secret*`, `auth.json`, `real_connection.json`, `$HOME/.config/limo_field_20261113/*`, `real_limo_kit/secrets/*`
   - 환경변수 원본 파일 및 무검토 덤프: `*.env`, `kit.env`, `.env.*`, 무검토 `printenv`/`env` 출력 전체
   - 런타임 데이터베이스 및 대용량 산출물: `fleet.db`, `*.db`, `*.sqlite*`, `*.db3`, `*.mcap`, `build/`, `install/`, `log/`
   - **무검토 원본 `git diff` / `git status`**: 로컬 사용자 홈 경로, Tailscale IP(`100.x.y.z`), SSH 계정명, 임시 토큰이 포함될 수 있는 원시(raw) `git diff` 출력
   - 학교 행정 제출 문서 및 개인정보 파일: 학기 보고서·계획서·발표 슬라이드 원본 문서 및 학번·성명·연락처가 포함된 문서 ([docs/PUBLICATION_POLICY.md](docs/PUBLICATION_POLICY.md))
2. **검토 후 포함 허용 목록 (Explicit Allowlist)**:
   - 실행에 실제 사용된 소스·설정 파일: 명시적으로 지정한 `nodes/*.py`, `fleet_server/*.py`, `limo_robot_agent/*.py`, `limo_rmf_adapter/*.py`, `launch/*.launch.py`, `config/*.yaml`, `bt/*.xml`, `world/*.world`
   - 파일별 무결성 해시표: `SOURCE_SHA256SUMS` (허용 목록에 포함된 소스·설정 파일들의 `sha256sum` 목록)
   - **마스킹·검열을 마친 버전 기록**:
     - `git_head_info.txt`: 브랜치명 및 `HEAD` 커밋 해시만 기록
     - `git_diff_sanitized.patch` (선택): 포함 전 반드시 정규식 검사(`grep -E -i "(token|secret|password|bearer|100\.[0-9]+\.[0-9]+\.[0-9]+|sshpass)"`) 결과가 **0건**임을 확인하고, 호스트 경로/IP를 `<REDACTED>`로 치환한 패치만 허용
     - `ros_launch_args_sanitized.yaml`: 토큰·IP·경로를 제거한 순수 알고리즘/ROS 파라미터(`allow_motor_drive`, `robot_radius`, `inflation_radius`, `ROS_LOCALHOST_ONLY` 등) 스냅샷

---

### 2.4 `run_id`와 소스 버전으로 연결되는 실험 메타데이터 (`manifest.json`) 규격

Google Drive에 업로드하는 모든 `<run_id>/` 폴더와 로컬 실험 폴더(`experiments/<exp>/runs/<run_id>/`)에는 아래 표준 `manifest.json`을 생성해 **어떤 코드 버전과 설정 스냅샷에서 얻은 데이터인지** 영구적으로 연결합니다.

```json
{
  "schema_version": "1.1",
  "run_id": "office03_20261114_fixed2_explore_rep3",
  "experiment_slug": "safety_chain_fix_20261113",
  "environment": "gazebo",
  "created_at": "2026-11-14T11:57:45+09:00",
  "source_version": {
    "git_branch": "staging/fleet-control-on-progress-c804241",
    "git_commit": "c804241ac9afb702616c7e9cd59478c8ea077a8f",
    "worktree_dirty": true,
    "variant_path": "experiments/safety_chain_fix_20261113/fixed2",
    "code_and_config_archive": "code_and_config_office03_fixed2_explore_rep3.tar.zst",
    "source_sha256sums_file": "SOURCE_SHA256SUMS"
  },
  "command": "tools/run_office.sh fixed2 office03_20261114 explore",
  "verdict": "PASS (18.48 m travelled, normal_stops=0)",
  "privacy_and_denylist_checked": true,
  "files": [
    {
      "name": "code_and_config_office03_fixed2_explore_rep3.tar.zst",
      "role": "sanitized_code_and_config",
      "size_bytes": 38420,
      "sha256": "<64-char hex sha256>"
    },
    {
      "name": "rosbag_office03_fixed2_explore_rep3.tar.zst",
      "role": "rosbag2",
      "size_bytes": 412345678,
      "sha256": "<64-char hex sha256>"
    },
    {
      "name": "logs_office03_fixed2_explore_rep3.tar.zst",
      "role": "logs_and_params",
      "size_bytes": 485120,
      "sha256": "<64-char hex sha256>"
    }
  ],
  "external_links": {
    "gdrive_path": "Capstone1_LIMO_Archive/02_GAZEBO_SIM/20261114_safety_chain_fix/office03_20261114_fixed2_explore_rep3/",
    "gdrive_file_id": "<업로드 후 기록>",
    "youtube_url": null,
    "youtube_visibility": null
  }
}
```

---

### 2.5 위험 기반 백업·암호화·무결성 검증 및 복구 정책 (SSOT)

민감정보(로컬 절대경로·`/rosout` 등)가 포함될 수 있는 원본 `rosbag2`와 바이트 단위 원본 코드·설정은 **원본을 변조하지 않고 GPG AES-256으로 암호화해 비공개 Google Drive에 보관(Option D)**하며, 아래 위험 기반 검증 기준을 따릅니다.

1. **필수 검증 (모든 중요 백업 공통)**:
   1. **원본·아카이브 해시 검증**: 원본 파일 및 아카이브의 `SHA-256` 생성 및 대조 (`FILE_HASH_MAPPING.tsv`, `SHA256SUMS.txt`).
   2. **로컬 격리 복원 검증**: `/tmp` 등 격리 디렉터리에 복원(암호화 아카이브는 로컬 복호화 포함) 후 전수 파일 `SHA-256` 일치 확인.
   3. **`rosbag2` 무결성 검증**: 복원된 모든 `.db3`의 `SQLite PRAGMA integrity_check` 및 `ros2 bag info <bag_dir>` 정상 파싱 확인.
   4. **민감 원본 암호화**: 개인정보가 포함될 수 있는 원본 `rosbag2` 및 바이트 원본 코드·설정은 GPG AES-256 대칭 암호화(`.gpg`, 사용자 대화형 암호 입력; 명령행·로그·환경변수 저장 금지).
   5. **비공개 Google Drive 업로드**: `Shared: false` 및 소유자 전용 권한 확인 후 검증된 파일만 업로드(평문 민감 아카이브 업로드 금지, 기존 파일 임의 덮어쓰기 금지).
   6. **원격 다운로드 해시 검증**: 원격에서 재다운로드(또는 이에 준하는 신뢰 가능한 원격 해시 대조) 후 `sha256sum -c SHA256SUMS.txt` 및 `.gpg` 바이트 일치(`cmp -s`) 확인(실패 시 즉시 비정상 종료).
   7. **검증 결과·한계 기록**: `manifest.json` 및 별도 검증 기록에 확인 사실과 재현성 한계(과거 실행 코드 미확보 등)를 기록.
2. **선택적 검증 (원격 전체 복호화·복원 시험)**:
   - 모든 백업에 대해 매번 원격 다운로드본 전체 복호화·복원 시험을 반복하지 않습니다.
   - 동일한 검증 절차에서 대표 백업(`run03~06` 등)의 원격 복구 시험이 성공했고, 개별 백업의 `local_restore_verified`와 `remote_download_integrity_verified`가 통과했다면 전체 원격 복호화는 생략할 수 있습니다.
   - 단, 다음 상황에서는 추가 원격 복원 시험 여부를 반드시 평가합니다:
     - 암호화 방식 또는 복구 스크립트(`seal_*`, `verify_remote_restore_*`) 변경
     - 기존 백업 형식·매니페스트 스키마와 비호환되는 변경
     - 대규모 데이터 이동 또는 로컬 원본 삭제 전
     - 중요한 장기 보관 자료의 정기 복구 점검
3. **검증 상태 구분 (매니페스트 스키마 호환성 유지)**:
   - `local_restore_verified` (기존 `local_decrypt_restore_verified`와 호환): 로컬 격리 복원 및 해시·SQLite·`ros2 bag info` 검증 성공
   - `remote_download_integrity_verified`: Google Drive 업로드 후 원격 재다운로드 `SHA256SUMS.txt` 및 암호문 바이트 일치 확인
   - `remote_full_restore_verified` (기존 `gdrive_remote_restore_verified`와 호환): 원격 다운로드본을 실제 복호화·복원해 전수 검증 완료 (`true`)
   - `skipped_by_user`: 원격 전체 복원 시험을 사용자 판단으로 생략 (사유 병기; **생략한 검증은 절대 `true`/`PASS`로 기록 금지**)
4. **원본 보존 및 보안 원칙**:
   - 원본 `rosbag2`·실험 결과(`result.json`, `meta.json`)·코드·설정은 임의로 수정하지 않습니다.
   - 검열본(`code_and_config_*.tar.zst`, `logs_*.tar.zst`)은 `private_backup`으로 취급하며 자동으로 공개 가능한 자료로 간주하지 않습니다([docs/PUBLICATION_POLICY.md](docs/PUBLICATION_POLICY.md)).
   - 로컬 원본 삭제(수 GB 반복 Gazebo `rosbag2` 등)는 별도 사용자 명시적 승인 및 복구 위험 평가 전에는 절대 수행하지 않습니다. 실차 실험 원본 `rosbag2`와 요약 지표(`result.json` 등)는 로컬 영구 보존합니다.
   - 과거 실행 코드 미확보(`not_found_in_searched_sources`) 등 재현성 한계는 현재 코드로 대체하지 않고 매니페스트에 유지합니다.

---

### 2.6 영상 개인정보 보호 및 YouTube 공개 등급 정책

[docs/PUBLICATION_POLICY.md](docs/PUBLICATION_POLICY.md)를 영상 자료에도 동일하게 적용합니다.

1. **업로드 전 필수 개인정보·보안 점검 체크리스트**:
   - **실차 촬영 영상**: 실험실/복도에 있는 **사람의 얼굴**, 화이트보드/모니터에 적힌 **이름·학번·전화번호·이메일·비밀번호**, 개인 노트북 바탕화면/알림창이 노출되지 않았는지 전 프레임 확인 (노출 시 블러 처리 또는 해당 구간 크롭).
   - **화면 녹화(RViz / Dashboard / 터미널)**: 터미널 프롬프트나 브라우저 주소창·로그에 **Tailscale 사설 IP(`100.x.y.z`), 공인 IP, SSH 호스트명, Bearer 토큰, 학생 학번**이 노출되지 않았는지 확인.
   - **학교 제출 자료**: 학기 보고서, 계획서, 성적/기여도 평가표가 포함된 발표 영상은 외부 공개 금지.
2. **YouTube 공개 등급 구분**:
   - **전체 공개 (Public)**: 개인정보·IP·학번이 전혀 없는 순수 Gazebo/RViz 시뮬레이션 주행 영상 또는 블러 처리가 완료되고 팀/사용자 승인을 거친 최종 대표 시연 영상.
   - **일부 공개 (Unlisted, 기본 권장)**: GitHub `README.md`나 `VALIDATION.md`, 기술 보고서에서 링크로 연결해 심사자·팀원이 볼 수 있도록 하는 실험 증빙 영상.
   - **비공개 (Private) 또는 Google Drive 전용**: 디버깅용 실패 재현 영상, 편집 전 실차 원본 영상, 내부 화면이 포함된 테스트 녹화본.

---

### 2.7 `VALIDATION.md`에서 실험 결과와 영상·rosbag을 연결하는 표준 표 형식

새로운 실험을 마치고 영상이나 `rosbag2`를 백업한 경우, 아래 표에 행을 추가해 코드·보고서·대용량 자료를 1:1로 연결합니다. (실패/기준선 실행도 삭제하지 않고 함께 기록합니다.)

| `run_id` | 날짜 | 환경 | 소스 커밋 / 변형 경로 | 로컬 요약·보고서 | `rosbag2` 아카이브 (`SHA-256` 앞 12자리) | Google Drive 경로 (백업 상태) | YouTube 시연 링크 (등급) |
|---|---|---|---|---|---|---|---|
| `run03_20261114` | 2026-11-14 | 합성 하네스 | `c804241` + `safety_chain_fix_20261113/baseline,fixed` | 로컬 `RESULTS_harness_baseline-run03_fixed-run04.md` (과거 중간 소스 2건 미확보) | 암호화봉인 `9752c5ba625f` | `03_SYNTHETIC_BENCHMARKS/20261114_safety_chain_fix/run03_20261114/` (**원격 복구 검증 완료**) | 해당 없음 |
| `run04_20261114` | 2026-11-14 | 합성 하네스 | `c804241` + `safety_chain_fix_20261113/fixed` | 로컬 `RESULTS_harness_baseline-run03_fixed-run04.md` (과거 중간 소스 1건 미확보) | 암호화봉인 `925af08fb0d9` | `03_SYNTHETIC_BENCHMARKS/20261114_safety_chain_fix/run04_20261114/` (**원격 복구 검증 완료**) | 해당 없음 |
| `run05_20261114` | 2026-11-14 | 합성 하네스 | `c804241` + `safety_chain_fix_20261113/fixed2` | 로컬 `runs/run05_fixed2.console.log` (과거 중간 소스 1건 미확보) | 암호화봉인 `3e7b025c628b` | `03_SYNTHETIC_BENCHMARKS/20261114_safety_chain_fix/run05_20261114/` (**원격 복구 검증 완료**) | 해당 없음 |
| `run06_policy_20261114` | 2026-11-14 | 합성 하네스 | `c804241` + `safety_chain_fix_20261113/fixed2` | 로컬 `runs/run06_policy_20261114/` | 암호화봉인 `c3da47794fea` | `03_SYNTHETIC_BENCHMARKS/20261114_safety_chain_fix/run06_policy_20261114/` (**원격 복구 검증 완료**) | 해당 없음 |
| `office04_20261115` | 2026-11-14 | Gazebo (Office) | `c804241` + `office_fixed,office_fixed2` | 로컬 25회 실행 (과거 `run_office2.py` 5회 미확보) | 암호화봉인 `6841d19843c9` | `02_GAZEBO_OFFICE_BENCHMARKS/20261114_safety_chain_fix/office04_20261115/` (**원격 해시 검증 완료 / 원격 복호화 `skipped_by_user`**) | 미녹화 (헤드리스) |
| `office03_20261114` | 2026-11-14 | Gazebo (Office) | `c804241` + `safety_chain_fix_20261113` (실행 당시 코드 해시 연결 기록 미확보) | 로컬 12회 실행 (`MEMORY_POLICY_V2.md`, 원본 113개, 로컬 복원 225/225, SQLite·`ros2 bag info` 12/12) | 암호화봉인 `96e415ee6f79` | `02_GAZEBO_OFFICE_BENCHMARKS/20261114_safety_chain_fix/office03_20261114/` (**8개 업로드·원격 해시 7/7 완료 / 원격 복호화 `skipped_by_user` / `remote_backup_integrity_verified`**) | 미녹화 (헤드리스) |
| `run_010` | 2026-11-13 | **실차 (LIMO)** | `real_auto_mapping_ready_20261113` | 로컬 `experiments/real_limo_field_20261113/FIELD_RESULTS_20261113.md` | 온보드/로컬 보존 (`map_001`, `map_002`) | 업로드 예정 (`01_REAL_ROBOT/...`) | 미등록 |
| `final4` | 2026-11-13 | Gazebo + YOLO | `419e798` + 로컬 `Capstone1_limo_integrated_sim` | 로컬 `<WORKSPACE_PARENT>/Capstone1_limo_integrated_sim/fleet_control/integrated_sim/RESULTS.md` (12/13 PASS) | `$HOME/.local/share/robot-sim-projects/limo_integrated_sim/runs/` | 업로드 예정 (`02_GAZEBO_SIM/...`) | 미등록 |


## 2026-11-15 추가 검증 근거와 한계

- Registry의 [원자성 시험](fleet_control/tests/test_map_registry_atomicity.py)에는 중복 요청과 저장 직후 강제 종료 사례가 있다. [고아 진단·부하 보고](fleet_control/server/ORPHAN_RECOVERY_PROPOSAL.md)는 두 48 MiB 근접 요청에서 RSS 27.3→322.1 MiB를 관측했다고 기록한다. 이는 운영 부하 한계나 복구 성공의 검증이 아니다. 이번 문서 조사에서는 운영 데이터에 접근하거나 시험을 재실행하지 않았다.
- Phase 1~1.2는 프로세스 내 독립 Mock이다. [격리 ROS Action 시험](fleet_control/tests/test_ros2_goal_boundary_isolated.py)은 실제 ROS 2 Action과 모의 서버를 사용하며 실제 Nav2·Gazebo·실차를 사용하지 않는다. 소유 UUID 취소 PASS 및 직접 제출 우회 차단 FAIL이라는 전달된 결과는 저장된 실행 로그를 이번 조사에서 찾지 못해 독립 확인 보류다. 테스트 코드는 직접 제출 수락을 예상한다.
- [v4.1 보고](experiments/safety_chain_fix_20261113/docs/GATE_V4_RESIDUAL_RISK_DESIGN_20261115.md)는 장치 보정 후 G/GR/GF 28 PASS / 2 MEASURE를 보고하고, 이전 INVALID와 무장 불가 문제도 보존한다. [v4.2 보고](experiments/safety_chain_fix_20261113/docs/GATE_V4_2_SAFETY_REVALIDATION_20261115.md)는 2~4절이 미기입이며 참조 계약 사례 19/19는 구현 검증이 아니다. 최신 전체 행렬 PASS 또는 실차 승인으로 기록하지 않는다.
- **Gate r7 (2026-11-16, SIM_ONLY)**: 기록은 `experiments/safety_chain_fix_20261113/runs/gate_r7_integration_20261116/final2_{normal,fault,restart,exclusivity,gate_restart}`와 `runs/gate_r7_gazebo_20261116/run{1,2}`이며, 실패·보정 전 실행(`try1`–`try9`, Gazebo `run1`)도 보존했다. 결과는 다음과 같다. 격리 65/65 PASS, 기존 70개 live 시나리오 70/70 PASS(`runs/gatev42r7_live_20261116`, r6 대비 판정 차이 0), r6 단위 64/64 PASS(호스트 네임스페이스; unshare 내부 61/3은 root 권한 때문의 환경 FAIL로 보존). Gazebo run2는 PASS 7 / BLOCKED 1(Fleet 경로)이며, 주행·정지 결과는 **직접 goal** 기준이다. 영상은 녹화 노드가 빈 파일만 남겨 미확보다. 원격 백업·암호화는 미실시(`not_started`). [보고](experiments/safety_chain_fix_20261113/docs/GATE_R7_PRODUCER_INTEGRATION_20261116.md).
- **Gate r8 (2026-11-16, SIM_ONLY)**: 격리 기록 `experiments/safety_chain_fix_20261113/runs/gate_r8_liveness_20261116/`과 Gazebo 기록 `runs/gate_r8_gazebo_20261116/run{1,2}`. 결과는 다음과 같다.
  - 격리: kill 16/16, sigstop 13/13, mute 12/12, replay 10/10, fault_hb 13/13, normal 14/14, fault 19/19, lost_terminal 16/16, exclusivity 11/11. gate_restart 8/1(하네스 seq≤5 가정 FAIL), Codex restart 14/1(R0 비영 24건 FAIL). r6 단위 64/64. 기존 70개 live 시나리오 70/70(`runs/gatev42r8_live_20261116`, r6 대비 판정 차이 0).
  - Gazebo run1: Fleet 정상 임무 FAIL. run2: SIGKILL 차단 9 PASS / 2 OBSERVED.
  - 영상 미확보(녹화 노드 빈 파일). 원격 백업 `not_started`.
- **Gate r9 Cliff (2026-11-17, SIM_ONLY)**: 기록 `experiments/safety_chain_fix_20261113/runs/gate_r9_cliff_20261117/`. 판정은 rosbag sim time 분석(`analysis_simtime.json`) 기준이다.
  - 합성 Cliff 단위: 17/17(1회차 FAIL 1건은 결함으로 수정, 기록 보존). r6 단위 64/64.
  - Gazebo PASS: A 평지, B 단차, C 계단(2회차), D 가장자리, E 난간(3회차), F 관측기 중단, G odom TF 중단, H 재출발 방지.
  - Gazebo SKIP: 카메라 TF 지연, 평지 근접 장애물 주행 불능. 무효·FAIL 기록(C1, E1, E2 기동 실패, I1 2건) 보존.
  - 무보호 기준선 낙하. 영상 미기록, 원격 백업 `not_started`.
- **Gate r10 잔여 위험 (2026-11-17, SIM_ONLY)**: 기록 `experiments/safety_chain_fix_20261113/runs/gate_r10_residual_20261117/`.
  - FAIL: 사각지대 틈 S1, 원거리 틈 재접근 S3.
  - PASS: 회전 S2a·S2b, 정지 CANDIDATE S4, 관측기 중단 S7, H1/H2 17건, 단위 Cliff 17 / GetState 6 / r6 64.
  - 주행 중 CANDIDATE: SKIP(미발생). 근접 장애물 S5: 불필요 래치 1/2.
  - r10 1회차의 bag 녹화 지연(이동 구간 누락)은 `_try2`로 재실행해 대체했다. r9 bag도 출발 직후 일부를 누락했으나 여유 측정에는 영향 없음.

- **후속 증거**: Phase 1.3 재현은 [로그](experiments/phase13_ros_action_20261115_K8m4/RESULTS.md)를 보존했다. [1.4](experiments/phase14_goal_boundary_20261115_q4H7/RESULTS.md)는 같은 도메인 직접 우회 및 cancel-all 보호 FAIL, [1.5](experiments/phase15_sros2_20261115_F8v2/RESULTS.md)는 격리 SROS2 미인가 endpoint 차단 PASS와 인가 주체 동시 제출 FAIL이다. [1.6](experiments/phase16_goal_arbiter_20261115_D2k8/RESULTS.md)은 독립 중재기·mock Action과 별도 SROS2 프로브만 시험했으며 실제 Agent·Nav2·Gate·실차는 SKIP이다.

- **Phase 1.7**: [SROS2 + 시험 중재기 결합 및 SIGKILL 재시작](experiments/phase17_goal_sros_20261115_M6w3/RESULTS.md)에서 단일 프로세스 경합 및 미완료 journal 복원 차단 PASS, 동일 인증정보의 두 프로세스 fencing FAIL, 구조화 Attestation 부재 차단 FAIL을 기록했다. 실제 Agent·Nav2·Gate·실차 시험은 SKIP이다.

- **Phase 1.8**: [시험용 flock·SQLite 모델](experiments/phase18_fencing_20261115_G7r2/RESULTS.md)에서 동일 저장소의 협조적 프로세스 경합과 SIGKILL 후 미완료 상태 차단 PASS, 합성 Attestation 구조 검사 PASS, DB snapshot 롤백 시 세대 재사용 FAIL을 확인했다. Nav2의 fencing token 검증, 실제 Gate Attestation·Agent·실차는 SKIP이다.


## 2026-11-16 Fleet UI 개편 검증 (Codex)

격리 loopback Fleet Server + 임시 DB/토큰 + 합성 telemetry에서 Chromium 23 checks PASS. 기존 static HTTP 2, Registry HTTP 4, safety regression 24, 취소 callback 14 = Python 44 tests PASS; JS helper/Registry upload 2 suites PASS. JS 구문 및 기존 DOM ID 보존 PASS. Fault·Offline·Reconciliation·미검증 지도·Attestation 부재·Lease 만료·viewer·서버 단절 캡처 포함. 실차·운영 서버·실제 센서·승인 이미지 일치·Chrome 메뉴 확대는 SKIP. [근거/초기 실패와 수정/최종 결과](fleet_control/docs/DASHBOARD_UI_20261116.md).


## 2026-11-16 Spatial Monitor UI

공간 탭 Chromium·2차 Chromium 23 checks 및 JS 2 suite, Python 30 tests PASS. 실제 RGB/Depth/계획 스트림, 점유영역 맞춤, 실차/Gazebo는 SKIP. [상세](fleet_control/docs/SPATIAL_MONITOR_PHASE3.md).


## 2026-11-16 Spatial 3가지 보기

공간 3모드·Registry 썸네일·상태 유지·Fault/Offline/Localization UNKNOWN 격리 Chromium 검증 PASS, JS 두 suite·Python 30 tests PASS. 실제 스트림·실차/Gazebo SKIP. [보고](fleet_control/docs/SPATIAL_VIEWS.md).


## 2026-11-16 Fleet UI 전체 리디자인

전체 UI 10화면×3 해상도 가로 넘침 없음, 기존 Chromium 23 checks·공간 모드·JS 2 suite·Python 30 tests PASS. 실차/Gazebo·각 단계별 중간 캡처는 SKIP. [결과](fleet_control/docs/FLEET_UI_REDESIGN.md).


## 2026-11-16 지도 집중 모드

지도 3화면·3해상도 접힘/펼침과 선택 유지·좌표·권한·Fault/Offline/Reconciliation 격리 Chromium 검증 PASS, JS 2 suites·Python 30 tests PASS. 실차/Gazebo SKIP. [근거](fleet_control/docs/MAP_FOCUS_DENSITY.md).


## 2026-11-17 Fleet UI 테마

테마 전환·저장·선택 유지 Chromium, 임무/안전 23 checks, 지도 집중, JS 2 suites·Python 30 tests PASS. 실제 센서/실차/Gazebo·네이티브 대화상자 테마 SKIP. [상세](fleet_control/docs/UI_THEME_REDESIGN_20261117.md).


## 2026-11-17 승인 Workspace 운영 UI 적용

Workspace runtime 검증: 실제 loopback Lease→Drawer 확인→POST missions 201→dashboard 조회→cancel 202; 10 Chromium 그룹, System/Light/Dark 및 카메라 검증 PASS. 기존 Python 44·JS 2 suites PASS. legacy recovery/ownership sandbox 소켓 오류는 PASS 제외. 실차/Gazebo·실제 스트림 SKIP. [상세](fleet_control/docs/WORKSPACE_RUNTIME_20261117.md).


## 2026-11-17 `/map` 타입 충돌 해결 및 Gazebo rosbag 통합 검증

실제 격리 Gazebo(Office+LIMO+Open-RMF slotcar)·실제 Fleet Server/Agent·SLAM Toolbox·rosbag2·Chromium. **실차 아님, 모터 구동 없음.** 상세·재현: [보고](fleet_control/docs/RMF_MAP_TYPE_SEPARATION_20261117.md).

| 항목 | 결과 | 근거 |
|---|---|---|
| shim 적용 Gazebo 2회: `/map` OccupancyGrid(2·3개, 265×202, known 53530)와 `/rmf/building_map` BuildingMap(1개, L1) 기록, 타입 단일, metadata=SQLite 일치 | **PASS** | `local_data/raw_results/rmf_map_bag_gazebo_20261117/{shim_01,shim_02}/verify.json` |
| 다른 domain에서 `ros2 bag play`: 두 지도 메시지 복원, `/map` 데이터 해시 일치 | **PASS** | 위 `verify.log` |
| 웹 SLAM 시작→저장→Registry 등록(UNVERIFIED)→중지→`map_server` 재로드→웹 녹화 VERIFIED | **PASS** (2회) | 위 `browser.log` |
| shim 없는 대조군: `/map`에 2타입 공존, rosbag "more than one type" 591회, 웹 SLAM 시작 거부 | **FAIL (원인 재현용, 예상된 실패)** | `.../noshim_01/` (`recorder.log`, `slam/current.json`, `graph-*.log`) |
| pytest 76 (recording_session_p0, slam_control, 신규 profile·shim 시험, slam_sessions/upload_auth, map_registry 3종, bag_copy 2종, agent_safety_v1) | **PASS** | 명령은 보고서 §7 |
| `/scan_slam` 기록 | **FAIL 유지** | sim 그래프에 발행자 없음(실차 normalizer 출력). 현장 graph 검증 필요 |
| CameraInfo·원본 영상 기록 | **미기록** | sim에 발행자 없음 + 고정 프로파일 설계 외 |
| Posegraph Registry 등록 | **미해결(진단만)** | posegraph 4.97 MB, Agent 3.8 MB 임계값으로 제외, 서버 JSON 한도 6 MiB(base64 약 6.7 MB) 초과 |
| 실차·장시간·shim 없는 RMF 소비자 호환 | **SKIP/미검증** | |

시험 증거 폴더는 Git 제외이며 시험 전용 토큰 파일을 포함하므로 외부 공유 금지. 다른 작업(Gate r9/r10 Cliff 등)의 기존 FAIL·SKIP 기록은 이 결과로 바뀌지 않는다.


## 2026-11-18 Fleet → Agent → Nav2 → Gate → Gazebo 단일 임무 (Phase 2, SIM_ONLY)

[보고](fleet_control/docs/FLEET_FOLLOWPATH_OWNERSHIP_20261118.md). 실제 격리 Gazebo + 실제 Nav2 + Gate r8 후보(무수정) + 실제 Fleet Server/Agent. **실차 아님, 반복성 미입증(단일 표본).**
- **원인 재현 FAIL 보존** `before01`: 소유 UUID가 Agent 직접 표본에만 있고 Gate 증거는 idle인 짧은 구간을 "모순"으로 판정 → Server 재조정 → AUTHORITY_LOST → CANCELED. 별도로 정상 FollowPath 자식 UUID와 교체 겹침이 foreign으로 분류되던 결함(`fault01` FAIL 보존).
- **PASS(제한 신뢰 모델, SIM_ONLY)**: 웹 단일 임무 `web03`(Journal/Nav2/Server SUCCEEDED 일치, Gate 출력 0, GT 정지 3.0 s 유지), Agent SIGKILL `fault06`(출력 0 0.43 s, GT 정지 0.57 s, 추가 0.12 m), 통신 중단 `comm01`(0.33 s/0.46 s), 운영자 Cancel `cancel01`(ACK≠Terminal), 외부 FollowPath 감지·정지 `foreign_fp02`.
- **FAIL 유지**: 외부 **NavigateToPose** goal은 차단되지 않음(`foreign_nav02`: Agent는 foreign 표시, Gate NORMAL·로봇 계속 주행) — 기존 Gateway 접근 제어 FAIL과 같은 계열. Gate 재시작 후 재무장·Server recover(`no_evidence`) 미해결.
- **SKIP**: Goal 결과 지연·중복 실주입, 실차, 통신 복구.
- **무효(환경)**: Gate 기동 래치 3회, Gate `MEMORY_NOT_APPLIED_SHADOW` 4회(Gate 담당 기존 문제). 원본 보존.
- 회귀: 신규 16 + Agent/Server 안전·r8·terminal 계열 276 PASS. `test_recovery.py` 11 FAIL은 수정 전 코드에서도 동일(레거시, 변경 무관).
다른 작업의 기존 FAIL(Gate r9/r10 Cliff, Fleet Goal 소유권 기록의 원본 FAIL, Gateway 접근 제어 등)은 삭제·변경하지 않았다.

## 2026-11-18 Gazebo MANUAL SLAM (SIM_ONLY)

- 실제 Office Gazebo, 로컬 전용 ROS 도메인 219, 임시 Fleet Server·Agent(`allow_missions=false`), r8 `manual_mapping:=true`. 웹 SLAM 시작 명령 `SUCCEEDED`, Agent `RUNNING`, `/map` frame `map`, 265×202, 알려진 셀 1,429개. 저장된 시험 증거: [local_data/raw_results/manual_slam_gazebo_20261118_310573](local_data/raw_results/manual_slam_gazebo_20261118_310573/).
- 동적·정적 TF를 함께 관측한 15초 시험에서 `map→odom` 149개 메시지의 발행자 GID는 SLAM Toolbox 1개였다(`static_check_tf_owners.txt`). 다른 실행에서는 지도 이후 Nav2 5개 액션을 Gate가 모두 `IDLE_PROVEN_EL`, 활성 UUID 0개로 관측했다(`after_probe.json`). 후속 정적 TF 시험의 관측 종료 시점에는 일부 Nav2 액션이 아직 `UNOBSERVED`여서 그 실행 자체로 전체 액션 유휴를 증명하지 않는다. 자동 임무 명령은 제출하지 않았고 Agent의 미션 실행은 비활성화했다.
- Gate는 실행 중 `FAILSAFE_SENSOR_TIMEOUT` 래치였다. 지도 발행 결과는 Gate 정상·주행 가능성이나 실차 검증을 뜻하지 않는다. 정지 명령도 `SUCCEEDED`. ROS 그래프 관찰은 비인증이며 관측 기간 밖 경쟁 발행을 배제하지 못한다.
- 최초 샌드박스 시험은 HTTP 소켓 `PermissionError: [Errno 1] Operation not permitted`로 시작 전 실패; 격상 후 초기 실행은 SLAM 전 Nav2의 비활성 액션이 `UNOBSERVED:no_event_no_sample`이어서 시작이 거부됐다. 첫 맵 이후 Nav2 기동으로 순서 문제를 해소했다. 두 TF Python 관측 시도는 Humble의 메시지 정보 제한으로 실패했고 C++ `MessageInfo` 관측기로 재시험했다.
- `limo_robot_agent` 격리 colcon 빌드 성공, `test_slam_control.py` 6개 통과, Python 구문 검사 통과. 실차·수동 주행·지도 저장은 미검증.

## 2026-11-18 MANUAL MAPPING Joy → Gate → Gazebo

- SIM_ONLY, localhost ROS_DOMAIN_ID 218, 임시 Fleet Server/Agent 및 r8 `manual_mapping:=true`, `allow_missions=false`. 기존 웹 SLAM 시작·정지 명령은 모두 `SUCCEEDED`, 시작 후 `RUNNING`과 `/map` 발행을 확인. 시험 중 Nav2 목표는 제출하지 않았다. 결과와 이벤트 원본: [local_data/raw_results/manual_joy_gazebo_20261118_315207](local_data/raw_results/manual_joy_gazebo_20261118_315207/), 요약 [summary.json](local_data/raw_results/manual_joy_gazebo_20261118_315207/summary.json).
- 최초 기준선 Gate 래치는 08:46:34.094 KST(시뮬레이션 시간 30.01초), `startup:NOT_READY supervisor_alive,nav_action_idle_proven,nav_status_evidence`. SLAM 후 Nav2 5개 유휴에도 supervisor 발행자가 없어서 `supervisor_alive`가 미충족이었고 reset과 Joy를 실행하지 않았다. 또한 `/cmd_vel_pre_cm`에 Agent·behavior_server·velocity_smoother가 공존했다.
- SIM 전용으로 Agent의 기존 r8 supervisor를 명시적으로 활성화하고, 임대·안전·유휴 증거를 수동 출력 조건에 추가했다. MANUAL에서 Nav2 속도 출력은 별도 비제어 토픽으로 분리했다. 재시험 최초 래치도 같은 사유(08:49:29.423 KST, sim 30.01초); 이후 센서·supervisor·Nav2 5개 유휴·fault goal 부재를 확인하고 기존 `/stall_detector/reset_latch`로 `INITIALIZING → NORMAL`(0.51초) 재무장했다. Gate 또는 Cliff 코드 변경·우회는 없다.
- 제어권 확인: `/cmd_vel_pre_cm` Agent 단독, `/cmd_vel_raw` Collision Monitor 단독, `/cmd_vel` Gate 단독. 실제 합성 Joy 전진 0.09 m/s, 회전 0.24 rad/s 명령이 세 출력 토픽을 통과했다. GT 전진 약 0.359 m; 제자리 회전 명령 중 yaw 약 0.569 rad, 병진 편차 약 0.056 m. 중립·deadman 해제·Joy 중단 후 최종 출력과 GT 속도는 0으로 돌아왔다. 정지까지 지연과 잔여 이동이 있으며 즉시 정지로 해석하지 않는다.
- 주행 전후 `/map` known cells 1,429 → 4,587, 데이터 SHA-256 변경. 실제 지도 셀 데이터의 변경 증거이며 저장·재로드 검증은 아니다. `limo_robot_agent` colcon 빌드, 수동 입력 ROS 전송/watchdog 시험, 수동 안전 가드 시험, SLAM 제어 시험 6개 통과. 실차는 미검증. 정상 재무장에 실패하는 경우 우회하지 않는다. 시험 종료 후 테스트 소유 프로세스 종료.

## 2026-11-18 MANUAL STOP · 저장 · Registry 재로드

- SIM_ONLY 연속 실행 `manual_finish_330007`: 웹 SLAM start/stop HTTP 202, 양 명령 최종 `SUCCEEDED`. Gate 정상 재무장 후 합성 Joy 전진 0.357 m, 회전 0.576 rad, `/map` known cells 1,429 → 4,636. 움직임 명령을 계속 발행하는 상태에서 STOP을 요청했고, `STOPPING → SAVING → STOPPING → COMPLETED`를 관찰했다. 정지 확인 시점 Gate `/cmd_vel` 0, GT 선속도·각속도는 측정 정밀도상 0. Gate·Cliff 코드는 변경하지 않았다.
- 저장 [map.yaml](local_data/raw_results/manual_finish_e2e_20261118_330007/slam/manual_joy/manual_final/map.yaml), [map.pgm](local_data/raw_results/manual_finish_e2e_20261118_330007/slam/manual_joy/manual_final/map.pgm); Registry `manual_joy#87232c2ca1eb`는 `UNVERIFIED`이며 저장 파일과 Registry YAML·PGM SHA-256, Registry 자체 무결성 검사 일치. 등록은 실차 항법용 검증 또는 활성 지도를 뜻하지 않는다.
- SLAM 종료 후 Registry 번들을 Nav2 map_server로 재로드: 265×202, 해상도 0.05000000074505806, 원점 x/y -3.9239569585007845/-4.971566629027297; 원본 SLAM 셀과 0셀 차이, 저장 이미지와 0셀 차이. 이번 기록의 비교 스크립트는 원점 yaw를 독립적으로 비교하지 않았다. [재로드 결과](local_data/raw_results/manual_finish_e2e_20261118_330007/reload.json), [이벤트](local_data/raw_results/manual_finish_e2e_20261118_330007/events.jsonl), [웹 명령 결과](local_data/raw_results/manual_finish_e2e_20261118_330007/web_commands.json).
- 초기 등록 실패는 파일명 규약(`map.pgm`) 불일치; 별도 [실패 기록](local_data/raw_results/manual_finish_failure_20261118_319960/) 보존. 이후 재로드 시 기본 `free_thresh=0.25`가 PGM 미지 셀 205를 자유공간으로 해석해 데이터 보존 실패했고, 다음 실행은 원점 반올림으로 정밀 메타데이터 비교에 실패했다. [미지 셀 실패](local_data/raw_results/manual_finish_reload_failure_20261118_321287/), [원점 실패](local_data/raw_results/manual_finish_origin_failure_20261118_324271/). 수정 후 원본 셀 검증, `free_thresh=0.1`, 원본 정밀 메타데이터 보존을 적용했다. 여러 Gate 유휴 증거 불충족 실행은 운전·reset을 생략하고 별도로 보존했다.
- 변경 후 `limo_robot_agent` 빌드, SLAM 제어 10개, 세션 4개, 등록 권한 1개 및 JS 구문 검사를 통과했다. 별도 브라우저에서 STOP 버튼의 시각적 상태 갱신은 미검증이며, 이 시험은 실제 웹 API와 Agent/ROS 상태를 관측했다. 실차·Web reload 제어 기능은 검증하지 않았다.

### 원점 회전 포함 추가 연속 시험

`manual_finish_332030`에서 같은 SIM_ONLY 웹 API → Joy → STOP → Registry → map_server 경로를 재실행했다. HTTP start/stop 202 후 명령 모두 `SUCCEEDED`; GT 전진 0.359 m, yaw 변화 0.571 rad; STOP 중에도 Joy 35건을 보냈으나 저장 단계 진입 시 Gate 출력과 GT 속도는 0이었다. 지도 known cells 1,429 → 4,577. Registry `manual_joy#cbf404932ccd` (`UNVERIFIED`)의 YAML·PGM은 로컬 저장본과 SHA-256 일치, Registry 무결성 검사 통과. Registry 번들 재로드 결과 SLAM 원본과 셀 차이 0, 크기 265×202, 해상도 0.05000000074505806, 원점 x/y/yaw `[-3.9239569585007845, -4.971566629027297, 0.0]` 일치. [요약](local_data/raw_results/manual_finish_e2e_20261118_332030/summary.json), [Registry](local_data/raw_results/manual_finish_e2e_20261118_332030/registry.json), [재로드 결과](local_data/raw_results/manual_finish_e2e_20261118_332030/reload.json), [이벤트](local_data/raw_results/manual_finish_e2e_20261118_332030/events.jsonl). HTTP API·상태는 실제 시험했지만 브라우저 화면 렌더링 및 웹 reload 버튼은 미검증·미구현이다.

## 2026-11-18 AUTO MAPPING 부분 구현 · 격리 Gazebo 시도 (미완료)

- 후보 전용 노드에서 기존 `explore_lite::FrontierSearch` 및 `Costmap2DClient`를 사용한다. Nav2 Goal 또는 cancel을 호출하지 않는다. Fleet 조정 구성요소는 단위시험만 수행했고 웹·Agent 실행 경로에 연결되지 않았다. r8 AUTO 옵션은 격리 SIM 후보 평가 프로파일이다.
- 실제 Gazebo 실행: ROS_DOMAIN_ID 215, localhost-only, `auto_mapping:=true`, Fleet 임대 후 웹 AUTO 시작 요청 **404 FAIL**(endpoint 미구현). 진단 목적으로 기존 웹 SLAM 시작은 HTTP 202 후 `RUNNING`·`/map` **PASS**. Frontier 후보 **15개 PASS**, 첫 후보 `(0.2010, 1.7534)` map 좌표. 기존 Fleet `/missions` 제출은 **409 FAIL**, `safety_not_ready`: 검증된 safety context와 idle Nav2가 필요하다. Gate는 `FAILSAFE_SENSOR_TIMEOUT`, `supervisor_alive` 미충족. Agent 미션은 비활성화돼 있었으며 외부 Goal을 제출하거나 Gate를 우회하지 않았다.
- Fleet 임무 ID·Agent UUID·Nav2 Goal/FollowPath·자동 GT 이동·지도 확장·다음 Frontier·AUTO STOP·저장·재로드는 **SKIP**. 진단 SLAM stop은 HTTP 202 및 `ACCEPTED`까지만 관찰했고 완료로 판정하지 않는다. 과거 외부 NavigateToPose 미차단 FAIL은 이번에 해결하거나 재검증하지 않았다.
- [실행 요약](local_data/raw_results/auto_mapping_attempt_20261118_334911/summary.json), [후보·Gate·GT 이벤트](local_data/raw_results/auto_mapping_attempt_20261118_334911/events.jsonl), [실행기](local_data/raw_results/auto_mapping_attempt_20261118_334911/harness.py), `gazebo.log`, `frontier.log`, `fleet.db` 보존. 기존 실험과 rosbag 원본은 건드리지 않았다.
- 격리 빌드: `source /opt/ros/humble/setup.bash; source capstone_ws/install/local_setup.bash; colcon --log-base /tmp/capstone_frontier_logs2 build --base-paths capstone_ws/src/m-explore-ros2/explore --build-base /tmp/capstone_frontier_build --install-base /tmp/capstone_frontier_install --packages-select explore_lite --allow-overriding explore_lite`. 후보/조정기 4 tests, SLAM 10 tests, FollowPath 귀속 16 tests 통과; Agent·explore_lite 빌드 통과. 실행은 같은 ROS 환경과 `$HOME/rmf_demos_humble/env.sh`를 source한 뒤 보존된 실행기를 사용한다. 위 실행기는 AUTO 미완성 진단용으로 기존 SLAM API를 별도 호출하며 자동 탐색을 주장하지 않는다.
- 차단 조건: 실제 Agent는 SLAM과 `allow_missions=true`의 동시 기동을 명시적으로 금지하고, Fleet/Agent 정상 임무 안전 계약은 verified 고정 지도와 CONFIRMED localization 및 NAVIGATION 상태를 요구한다. 라이브 SLAM을 이를 만족한다고 위조할 수 없다. AUTO 전용의 검증 가능한 SIM_ONLY 안전 계약, 정상 Fleet submission 연결, journal 기반 목표 terminal 피드백 및 STOP cancellation 동기화가 필요하다.

## 2026-11-18 AUTO Supervisor 연결 재시험 (AUTO E2E 미완료)

- 원인: `agent_node.py`의 `sim_r8_supervisor`가 MANUAL만 허용하여 기존 AUTO 진단에서 heartbeat publisher가 없었다. AUTO 명시 opt-in을 localhost 전용·비기본 도메인·Gazebo SIM·SLAM 활성·미션 비활성·MANUAL 배타 조건으로 제한하고 기존 Agent challenge/lease 검증과 Supervisor 구현을 재사용했다. Gate 코드는 변경하지 않았다. AUTO Goal을 활성화하는 계약은 아니다.
- 첫 재시험 `340581`: Supervisor 응답 710건 및 `alive=true`를 확인했으나 `/controller_server/change_state` 응답 timeout, Nav2 유휴 증거 부재로 Gate 재무장을 하지 않았다. [로그](local_data/raw_results/auto_supervisor_attempt_20261118_340581/).
- `342668`: Supervisor `alive=true`; Nav2 유휴 증거 부재로 Gate 래치 유지. rosbag 기록 중 `/map`에 두 ROS 타입이 공존해 기록 거부되었다. [로그·부분 rosbag](local_data/raw_results/auto_supervisor_attempt_20261118_342668/).
- `344513`: 기존 RMF map 타입 분리 shim을 Gazebo 프로세스에만 적용. Supervisor alive, Nav2 5개 idle 증거와 준비 조건을 확인한 뒤 기존 reset으로 Gate NORMAL; Frontier 후보 15개. 실제 Fleet `/missions`는 `safety_not_ready` 409로 거절했고 Goal을 직접 보내지 않았다. rosbag 4,909건(`/map` 2, 후보 5, heartbeat 125, `/cmd_vel` 627 등)을 보존했다. 웹 AUTO 시작은 여전히 404. 진단 SLAM 중지는 하네스 종료 시점 Fleet `RECONCILIATION_REQUIRED`이므로 종료 성공으로 판정하지 않는다. [요약](local_data/raw_results/auto_supervisor_shim_20261118_344513/summary.json), [rosbag](local_data/raw_results/auto_supervisor_shim_20261118_344513/rosbag/), [로그](local_data/raw_results/auto_supervisor_shim_20261118_344513/).
- Agent 빌드, Frontier 조정기 4·FollowPath 16·SLAM 10 단위시험 통과. AUTO API, Mapping Goal 안전 계약과 Agent Coordinator 연결, Goal terminal/GT 이동·반복 탐색·AUTO STOP/저장/재로드 및 실패 주입은 **미구현 또는 SKIP**. 실차 연결·시험 준비는 Gazebo 성공 후 수행하기로 한 사용자 지시대로 시작하지 않았다.

## 2026-11-18 실제 LIMO Agent 비구동 점검·선택 배포

- 사용자 요청으로 기존 SSH/Tailscale 접속. LIMO `pcu` Ubuntu 22.04.5 LTS x86_64, ROS 2 Humble, slam_toolbox 2.6.10, Nav2 1.1.20. 도메인 77 localhost-only ROS graph는 Agent 시험 전후 노드 없음·`/parameter_events`, `/rosout`만 관측. 센서·TF·지도·Nav2·Gate와 실제 모터 정지 수단은 **미검증**이다. 홈 저장장치 80 GB 여유. `~/rosbags`, `~/limo_capstone/maps`, `~/capstone_ws/mapping_trials` 보존.
- 기존 `~/limo_field_agent_ws`의 Agent 0.1.0 소스와 최신 로컬 Agent는 해시 불일치. 사용자 승인 범위의 선택적 Agent 패키지 19개 파일만 해당 기존 workspace에 반영하고 `colcon build --packages-select limo_robot_agent` 성공, 소스 19개 SHA-256 전체 일치 및 설치된 주요 3개 모듈 해시 일치. 버전 문자열은 양쪽 0.1.0이므로 버전만으로 동일성을 판단할 수 없다. 기존 안전 체인·자동 맵핑 workspace·맵/rosbag·토큰/설정은 수정하지 않았다.
- 이전 Agent `src`, `install`, `build`, `log` 및 기존 field 실행 설정을 LIMO `~/limo_field_agent_ws/predeploy_20261118_103822_6758.tar.gz`에 보존(0600, SHA-256 `0269374820b9354ad84c9948e514c722100c67c2633246c008ef773aedffdb63`). tar 목록과 이전 주요 3개 소스 해시를 백업에서 재검증. 실제 복원은 실행하지 않았다. 롤백은 실행 프로세스 부재·소유권 확인 후 현재 각 디렉터리를 유일한 보존 이름으로 이동하고, 백업을 별도 스테이징 위치에 풀어 내용 비교 후 이전 `src/install/build/log`를 복원한다. 기존 field 설정은 비교 없이 덮어쓰지 않는다.
- 임시 노트북 Fleet Server(최신 로컬 소스 0.1.0, 격리 DB, 기존 field Agent 인증 항목 사용) 및 실제 LIMO Agent(`mode=real`, `allow_missions=false`, SLAM·녹화·Joy 비활성)를 두 차례 실행. Agent 인증·상태 보고, HTML과 로봇 상태 API HTTP 200, 시험 소유 서버만 일시 중단 후 재접속을 각각 확인. 종료 코드 두 번 0, 종료 후 ROS graph 재확인. 기존 SIGSEGV의 원인은 미해결이며 두 번 정상 종료로 해소 판정하지 않는다. 대시보드 브라우저 화면과 실제 지도 업로드는 미검증(`/map` 없음, map API 404). 두 시험 서버와 Agent는 종료했다.
- 첫 overlay는 `limo_msgs` 미인식, 기존 실차 bundle overlay 재시험에서 `limo_msgs` 인식 및 배터리 source `/limo_status` 확인. 실제 상태 메시지가 없어 전압·센서값은 UNKNOWN/NO_DATA, Nav2 UNAVAILABLE, Gate NO_DATA다. bundle의 `explore_lite`, SLAM·Nav2 및 실차 launch 경로 존재는 확인했으나 실행하지 않았다. Web SLAM은 SIM_ONLY, 실차 웹 녹화/REAL_LOCKED는 차단 유지, 웹 AUTO 404 및 Mapping Goal 409·Gazebo AUTO E2E 미완료는 변함없다.
- [비공개 로컬 결과](local_data/raw_results/real_limo_nodrive_20261118_agent_refresh/summary.json), [overlay 시험 결과](local_data/raw_results/real_limo_nodrive_20261118_agent_refresh/mapping_overlay_result.json), [Agent 로그](local_data/raw_results/real_limo_nodrive_20261118_agent_refresh/mapping_overlay_redacted_agent.log). 임시 인증 파일·토큰은 기록에 복사하지 않았다. 현장 물리 정지 수단, 전원 돌발 차단·USB 재연결, 실차 통신 단절 시 모터 정지와 속도/공간 확인 전 주행은 차단한다.

## 2026-11-18 실차 Depth TF·지도 저장 수정 배포와 300초 관측

- 런치: LIMO 현장 콘솔에서 `ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 ./start_real_auto_mapping.sh --depth true`와 `START HARDWARE LOCKED`. run_001 정상 종료와 지도·posegraph 저장. bag 591,719 메시지, 복원 파일 49개 SHA-256 일치, SQLite·`ros2 bag info` PASS. run_002는 기존 읽기 전용 Agent가 중복 노드 검사를 막아 FAIL. 해당 Agent만 정상 종료(SSH 경로 exit 0, SIGSEGV 미재현) 후 run_003 READY, Agent 재인증·업로드 PASS.
- 실차 설치 Depth SHA-256 `ba01c61258dce4f908502303b6b77744ee04a1f29f1d1568e277431a5aae6945`, 세션 소스 `634bc38cb19495f7fa794637b1fcbf21bf3f949990b5af71f95e6ae72c6982b0`. 임계값·메타데이터와 Depth TF 대기 및 계측만 배포. Gate 코드와 구동 제한은 그대로 사용.
- Depth 300초: 오류 상태 38/5,976(0.64%) 대 기존 bag 214/270(79.26%). 시도 988개 중 미래 시각 866, 해소 859, 만료 7. 수신 4,711(15.77 Hz), 카메라 optical TF 2,984(9.98 Hz), 유효 출력 981(3.28 Hz), 촬영~출력 수신 중앙값 59.5 ms/p95 108.3 ms. 해소까지 0–100 ms 히스토그램과 두 color TF 부모는 [실차 계측](local_data/raw_results/real_limo_depth_map_deploy_20261118_01/field_result.log). 7회 만료는 약 1.8초 동안 집중됐고 원인 미확정.
- Gate 48 NORMAL, 2,941 FAILSAFE_SENSOR_TIMEOUT. 관측 6.06초에 wheel odom stale로 전환. FAILSAFE 중 2,922개는 Depth READY, 19개는 Depth 오류. `/odom`, `/imu`, `/limo_status` 동시 재수신 단절과 베이스 시리얼 FD 유지 확인. 시험은 모터 잠금, `/cmd_vel` 발행자 없음. 4,272 메시지 별도 진단 bag의 `/cmd_vel_locked` 1,137개 모두 0, `/odom` 0건. 원본과 로컬 복사본 해시·SQLite·bag info 검증.
- 지도: run_003 실제 `/map` CDR 한 장을 캡처하고 도메인 208에서 수정 CLI·메타데이터 함수와 Nav2 map_server로 비교. UNKNOWN 4,304 / FREE 557 / OCCUPIED 59, 41×120, 해상도 0.05000000074505806, 원점 x/y −0.9941498206599038/−0.5361028258968245, 회전 항등, 불일치 0. 최신 `/odom`이 필요한 현장 세션 `save` 및 신규 posegraph는 SKIP. 웹 ONLINE·라이브 지도·Gate FAILSAFE 현장 화면 확인. Fleet 임무 및 웹 녹화 비활성.
- [비공개 로컬 증거](local_data/raw_results/real_limo_depth_map_deploy_20261118_01/)에는 설치 및 롤백, 기존 bag 백업, 실패 run_002, 연속 계측, 새 map YAML/PGM과 별도 bag이 있다. Google Drive 암호화 보관과 원격 복원 검증은 미실시.
- run_003의 소유 기록기에만 SIGINT를 보내 350.87초, 73,392 메시지 bag을 마감했다. `Recording stopped`, metadata, 로컬 복원 해시, SQLite, `ros2 bag info` 확인. 기록기는 부모가 회수하기 전 종료 상태로 보였고, 잠금 ROS 스택·Gate는 유지했다. [bag 검증](local_data/raw_results/real_limo_depth_map_deploy_20261118_01/run003_main_bag_local_verify.log)

## 2026-11-18 베이스 스트림 중단 시각·격리 재현

- run_003 DDS bag 최종 수신 시각(KST) /odom 11:36:21.195645, /imu 11:36:21.204487, /limo_status는 보조 monitor age 추정 11:36:21.125경. 수신 중 약 49.95 / 99.91 / 1.99 Hz였고 최종 중단 이후 복구 없음. /odom·/imu는 최종 중단 전 기록에서 0.38초 초과 공백 0회. 최초 Gate 오류 11:36:21.583033, 마지막 wheel odom 기록 +0.3874초. [시각 대조](local_data/raw_results/real_limo_base_stream_20261118_01/timeline.log)
- 실차 그대로 20초 재관측 시 /odom·/imu·/limo_status 모두 0 Hz, Gate NORMAL 0/201, FAILSAFE 201/201. 5분 직전 기록 48 NORMAL / 2,941 FAILSAFE. FD와 ttyUSB1 inode 일치, 프로세스 read 카운터 8초간 변화 없음. 관리자 커널 조회에 출력 없음(현장 사용자 보고). USB/UART/MCU/전원 원인 분리 전 드라이버 수정·재배포 및 실차 수정 후 시험 SKIP.
- 격리 PTY에서 합성 패킷을 중단하면 프로세스·FD 유지 중 3개 ROS 토픽 모두 멈추고 입력 재개 후 복구 PASS. 별도 격리 ROS 207에서 합성 Joy → 실제 Humble teleop → 기존 Gate → /cmd_vel_locked의 전진·제자리 회전, 후진 차단, neutral·deadman 해제·Joy 중단 및 wheel odom 상실 시 0 확인 PASS. 실물 조이스틱과 물리 정지는 SKIP. [재현과 한계](local_data/raw_results/real_limo_base_stream_20261118_01/README.md)

## 2026-11-18 CP2102 재시작 실패·실차 재부팅 회복

- run_003 수신 0 Hz 지속. 같은 ttyUSB1 FD·USB 장치가 활성 상태였고 USB 요청만 증가했다. 승인된 단일 베이스 프로세스 재시작은 포트 open 실패. 읽기 전용 open EIO, 커널 CP2102 -71/EPROTO·Unable to enable UART. 해당 증거는 재시작 시점의 USB–UART 실패를 확정하며 초기 장애의 물리 원인까지 특정하지 않는다.
- 현장 사용자 승인 재부팅(12:04:01 KST) 후 기존 Depth 활성·모터 잠금 프로파일로 run_004 READY. 60초 두 구간 기준 Odom 약 49.95 Hz, IMU 약 99.90 Hz, 상태 약 2.00 Hz. Odom/IMU 0.38초와 상태 1초 한계 초과 공백 0회. Gate 591/591 및 600/600 NORMAL, FAILSAFE 0. 잠금 속도 5,952개 모두 0, /cmd_vel 발행자 0. 장기 내구 시험은 미실시.
- 단독 24.2초 진단 bag 5,123개 메시지, 로컬 복원 해시·SQLite·ros2 bag info PASS. run_003과 run_004의 기존 기록 보존. Fleet API ONLINE·지도 업로드 확인. 임무와 웹 녹화 비활성. [실행 증거](local_data/raw_results/real_limo_base_recovery_20261118_01/)
- 수동 진단 후보: 패키지 빌드, 기본 경로 보존/격리 경로/위험 인수 거부 런치 계획, 격리 ROS Joy→Gate 잠금 출력 PASS. 실차에 이 프로파일을 적용하지 않았다.
- 노트북 실물 USB 패드의 격리 입력 시험 PASS. 축 1 전진 +1, 축 0 조향 양방향 ±1, 버튼 4 press/release, 약 12.266초 무수신 및 장치 재오픈 확인. 입력 도메인에서 속도 노드·속도 발행자 없음. 실차 OS에서 장치 열거와 최종 실제 매핑은 미검증. [장치 입력 로그](local_data/raw_results/real_limo_base_recovery_20261118_01/joy_device_result.log)

## 2026-11-18 실차 Joy 입력과 USB 허브 재열거

- 입력 전용 Joy 536개, 전체 평균 15.10 Hz·간격 중앙값 50.7 ms. 축 1 전진, 축 0 좌우, 버튼 4 누름·해제, 중립 및 USB 제거 후 무수신 74.22초. 실차 재연결을 통한 Joy 복원은 미관측. 현장 /cmd_vel 없음.
- 12:18:26 허브 3-7/CP2102 재열거, 마지막 odom 12:18:26.156462, Gate wheel_odom_stale 12:18:26.549806, 기존 드라이버는 삭제된 ttyUSB1 FD 유지. 분리 과정에서도 허브 재열거. 승인받아 베이스만 속도 입력 격리 후 새 포트에 회복. 이후 120초 Gate NORMAL 1,200/1,200, Odom/IMU/상태 약 49.95/99.90/2.00 Hz, 잠금 출력 5,999개 모두 0. 복구 bag 5,098 메시지, 복사 해시·SQLite·bag info PASS.
- 정지 확인 및 run_004의 현장 save PASS. 48×120, 0.05 m, UNKNOWN 5,167 / FREE 537 / OCCUPIED 56. 복사본 SHA-256 일치, 동일 저장 스냅샷 대비 Nav2 raster 및 별도 SLAM Toolbox posegraph 재구성 모두 셀 차이 0과 원점 일치. [증거](local_data/raw_results/real_limo_joystick_locked_20261118_01/)

## 2026-11-18 Fleet 웹 Joy 잠금 벤치

- 격리 ROS 204, 합성 센서, 실제 Fleet HTTP/Agent/Gate. viewer·다른 Lease 소유자·중복 세션 거절; 전진/회전 요청·후진 0 제한·deadman·입력 만료·wheel odom loss·발행자 충돌 확인. 관측 /cmd_vel_raw 75개와 /cmd_vel_locked 371개 모두 0, 모터 발행자 없음. [로그](local_data/raw_results/web_manual_bench_20261118_01/bench_http_ros.log)
- 실제 pcu, ROS 77, 기존 세션 유지. 브라우저 대신 합성 HTTP로는 실제 실차 Agent에 ±회전/전진/중립/해제 및 만료 검증. 현재 Gate 입력에 기존 Nav2 발행자 4개로 Agent 발행 차단, /cmd_vel 발행자 0. 12:34:48 경 wheel odom FAILSAFE 재발. 실물 브라우저 Joy 및 실차 Gate 경로 PASS로 보지 않는다. [실차 전달](local_data/raw_results/web_manual_bench_20261118_01/live_transport_repeat.log) · [Gate 실패](local_data/raw_results/web_manual_bench_20261118_01/field_fault.log)
- 실차 구동·새 수동 맵핑·등록·자동 맵핑·자율 주행은 SKIP. 기존 저장 지도와 원본 bag은 보존했다.

## 2026-11-18 웹 수동 맵핑 최종 연결 시도

- base PID 재시작 전 잠금 출력 53개 0, /cmd_vel 발행자 없음. 동일 CP2102 포트가 열리지 않아 FAIL, ttyUSB2 kernel -71. 마지막 수신 12:34:47.965 KST, 해당 시간창 kernel 재열거 기록 없음. 사용자 배터리 부족 보고로 추가 시동 및 구동 SKIP. [현장 로그](local_data/raw_results/web_manual_field_final_20261118_01/recovery_failure.log)
- ROS domain 203 localhost에서 기존 launch의 Nav2 그룹만 실행. 기본 /cmd_vel_raw behavior_server×3 + velocity_smoother×1. 웹 수동 프로파일에서 /nav2_cmd_vel_quarantined로 모두 이동, raw 0, /cmd_vel 0. 실차 런치 변경은 미실시. [격리 실측](local_data/raw_results/web_manual_field_final_20261118_01/nav2_graph_test.log)
- 노트북 실물 USB 인식, 100초 브라우저 관측에서는 입력 프레임 없음. 실차 E2E, 해제·단절 정지, 실차 주행 및 지도 등록 SKIP.
- 기존 run_004 기록기는 SQLite lock 오류로 종료. 별도 복사본 재색인·해시·SQLite 검사(267,577건)와 도메인 211의 지도/odom 재생(6/267) PASS. 원본 미수정. [복구·재생](local_data/raw_results/web_manual_field_final_20261118_01/README.md)

## 2026-11-19 LIMO 전용 단일 워크스페이스(`robot_ws`) 구성 및 비주행 검증

- **대상 경로**: LIMO `/home/pcu/Capstone1_limo/robot_ws` (기존 `/home/pcu/capstone_ws` 무수정·미빌드·미참조). 최소 9개 패키지(`limo_msgs`, `limo_base`, `limo_description`, `ydlidar_ros2_driver`, `explore_lite_msgs`, `explore_lite`, `limo_scan_normalizer`, `limo_depth_obstacles`, `limo_capstone`) 선택 배포 및 `/opt/ros/humble` 단독 기준 `colcon build --symlink-install` PASS (9/9, 17.3초, `install/` 내 `capstone_ws` 참조 0건). LiDAR-only 독립 실행을 위해 LIMO 배포본 `limo_integrated_auto_mapping_real.launch.py`만 `astra_camera` 선택 조회(미설치 시 `use_depth_camera:=true` fail-closed `Shutdown`) 적용.
- **USB–UART 고정**: CP2102 2개 모두 `10c4:ea60`, `ID_SERIAL_SHORT=0001` 중복으로 `by-id` 배제. `ID_PATH` 기반 udev 규칙(`99-limo-devices.rules`)과 실통신 검증으로 `pci-0000:00:14.0-usb-0:7.4:1.0`(`/dev/ttylimo` → `ttyUSB1`) = 베이스(`/odom` 49.95 Hz, `/imu` 99.91 Hz, `/limo_status` 12.7~13.0 V, `error_code=0`, `motion_mode=0`), `pci-0000:00:14.0-usb-0:3:1.0`(`/dev/ydlidar` → `ttyUSB0`) = YDLidar Tmini Plus(`/scan` 10 Hz, `/scan_slam` 244빔) 확정.
- **모터 잠금 통합 스택 및 노트북 원본 rosbag2**: `allow_motor_drive:=false`, `use_depth_camera:=false`, `speed_profile:=slow`, `scan_normalizer:=true`, `estimator_type:=f4`, `drive_mode:=diff`, `start_explore:=true` 기동. `/cmd_vel` 발행자 0, FastDDS Discovery Server(`<TAILNET_DISCOVERY_IP>:11811`, `ROS_DOMAIN_ID=77`, `ROS_LOCALHOST_ONLY=0`)로 노트북에서 직접 원본 CDR rosbag2 기록 PASS (`local_data/raw_results/limo_runs/20261119_locked_verify/rosbag/rosbag_0.db3`, 13.62초, 15개 토픽 2,629건, 1.4 MiB, SQLite `ok`; LIMO 내 rosbag 0건). 기록 구간 Safety Gate `NORMAL` 113/113, `wheel_odom_stale=false`, `max_abs_wz=0.5`, TF 5개 링크(`map→odom→base_footprint→base_link→laser_link,imu_link`) 모두 OK.
- **주의 관측**: 직전 15:03 잠금 실행에서는 일시적 `wheel_odom_stale: true` → Gate `FAILSAFE_SENSOR_TIMEOUT`(`cmd_blocked: true`) 1회 관측(Gate 차단 동작은 정상, 베이스 USB/전원 간헐 중단 리스크 잔존).
- **지도 저장·이송·재로드**: LIMO `/tmp` 저장(`map_saver_cli --free 0.196`) 후 노트북 `local_data/raw_results/limo_runs/20261119_locked_verify/map/` `scp` 이송·SHA-256 일치 검증 및 LIMO `/tmp` 삭제 완료. 노트북 격리 도메인 219 `nav2_map_server` 재로드 PASS (`84×88`, 0.05 m, origin `[-1.29, -0.268, 0]`, `unk=7037, free=334, occ=21`).
- **SKIP**: 실차 모터 구동(`allow_motor_drive:=true`), 물리 E-Stop/정지 시험, 장시간 소크 시험 (현장 운영자 별도 승인 대기).


## 2026-11-19 자동 맵핑 중단 재시험

- 모터 잠금 감시 시험: `/map`, `/scan`, Gate NORMAL, 프론티어 3개 수신; 30초 상한 종료 코드 4. 모터 출력 주파수 및 60초 연속 조건은 별도 검증하지 않았다.
- 주행 중 감시 출력 정체와 헤드리스 녹화 조기 종료로 런치 SIGINT; 감시기 종료 코드는 확인하지 못했고 맵 완성은 FAIL/미확인. 종료 후 이번 런치·베이스·감시기·브리지·카메라 프로세스 부재 확인.
- 노트북 rosbag 13,297건, `ros2 bag info` 및 SQLite integrity `ok`; 마지막 `/map` 306×251, known 23,239. 마지막 CDR에서 복원한 PGM/YAML은 격리 도메인 219 map_server 활성 및 메타데이터 확인. 로봇 map_saver 및 이송 해시 비교는 SKIP.
- RGB/Depth 화면을 포함한 헤드리스 녹화 421프레임, 약 105초를 MP4로 조립; 전체 주행 녹화는 FAIL. `/plan` rosbag 20건, 화면 내 예상 경로 표시는 미검증. [결과](local_data/raw_results/limo_runs/20261119_auto_map_full/).

## 2026-11-19 자동 맵핑 재실행 02 안전 중단

- 사용자가 현장 확인·저속 주행을 승인해 새 실행으로 기록을 시작했다. 약 45초에 Gate `FAILSAFE_SENSOR_TIMEOUT`, `wheel_odom_stale=true`, `cmd_blocked=true` 관측. 감시기 3.1초 지속 감지로 종료 코드 2, launch SIGINT; 이후 이번 LIMO 프로세스 부재 확인. 이전과 같은 odom stale 현상이 재발했으나 물리 원인은 미확정이며 재주행하지 않았다.
- 노트북 bag 14,603건, SQLite `ok` 및 `ros2 bag info` PASS. 마지막 `/map` 272×219, known 12,487; bag에서 복구한 PGM/YAML은 격리 도메인 219 map_server 메타데이터 재로드 PASS. 완성 지도 아님. 로봇 map_saver 및 SHA 비교 SKIP. 관제 녹화 342프레임을 MP4로 저장. [결과](local_data/raw_results/limo_runs/20261119_auto_map_full_02/).

## 2026-11-19 STANDBY 통합 기동 1차

- Mock/설정 PASS. 준비 실패 항목과 세 모드 BLOCKED, 0.15 m/s Nav2 설정, 두 costmap의 Depth 소스, 필터 높이 0.025–0.35 m에서 낮은 모형 점 채택 및 바닥/상부 모형 점 제외를 확인했다.
- 격리 ROS 2 PASS. localhost 도메인 217, 합성 TF/지도/센서로 실제 Depth 필터와 Nav2 controller 및 두 costmap을 실행했다. Depth 모형 장애물 위치에 lethal 셀이 나타났고, Gate NORMAL 이후 Depth 입력을 끊자 `DEPTH_STALE` / `FAILSAFE_SENSOR_TIMEOUT`으로 전환해 가상 전진 요청 중 잠금 속도 출력이 0이 되었다. MCU 정지 및 실제 모터 동작 증거가 아니다. Nav2 목표 제출·회피 궤적은 시험하지 않았다.
- 실차 중복 방어 PASS, 실차 센서 통합 검증 SKIP. LIMO `/tmp` 후보 실행이 기존 베이스·LiDAR·카메라·SLAM·Nav2·Frontier 프로세스를 감지해 새 스택 시작 전 코드 2로 종료했다. 기존 프로세스 및 설치본 보존. 실차 의자/책상 마킹, 18190 상태 연동·영상, Gazebo, 통신 단절 차체 정지 및 MCU watchdog은 미검증이며 구동 차단 유지.

### 2026-11-19 후속 — STANDBY 가구·속도 제한 검증

- Mock 회귀와 `limo_capstone` 격리 빌드 PASS. 격리 ROS 217과 Gazebo Fortress Depth(ROS 218)에서 실제 필터, 로컬/글로벌 Costmap, 활성 Nav2 컨트롤러, 잠금 Gate를 시험했다. 의자 연결봉 z=0.035 m 및 책상 프레임 z=0.210 m 위치에 두 Costmap 모두 lethal 셀, 상부 z=0.42 m 표본 위치에는 lethal 셀 없음. Gazebo 센서 수신 20개. 0.25 m/s 가상 요청이 잠금 출력 0.15 m/s로 제한되고 Depth 중단 시 DEPTH_STALE/FAILSAFE, 잠금 출력 0 확인. 고정 카메라 픽스처이며 실제 차체 주행·회피 시험이 아니다.
- 실차 외부 세션 PID 4259, PPID 1, SID 4256, 사용자 pcu, 18:58:10 KST 시작. 자식은 18:58:11 시작. 베이스 4335, LiDAR 4337, 카메라 컨테이너 4339, Depth 필터 4349, SLAM 4408, Nav2 4411/4413/4416/4420/4425/4428/4430/4433, Frontier 4435, Gate 4437. ROS 77, LIMO robot_ws overlay 및 Humble 기본 Nav2. Frontier·Depth required·모터 활성 옵션이 지정되어 있고 생성 Nav2 설정의 목표 선속도 0.32 m/s. Gate에서 /cmd_vel 357건, /cmd_vel_locked 0건 수신. 기존 세션 시작 주체 개인은 확인 불가. Agent 프로세스와 ROS 노드는 관측되지 않음.
- 실차 읽기 전용 수신 PASS: LiDAR 72, RGB 102, Depth 영상 208, 입력 점군 73, 필터 점군 19, 지도 7, TF 510. 관측 창과 QoS 기준으로 /odom·/imu·/limo_status·두 Costmap 수신 0. Gate FAILSAFE_SENSOR_TIMEOUT / wheel_odom_stale=true. 포트 상태 및 차체 물리 이동은 확인하지 않았다. 후보 실차 잠금 기동·가구 마킹·18190 실기 연동 SKIP. 외부 프로세스에 명령·시그널을 보내지 않았다.
- 크기 검토: AgileX 명목 높이 0.251 m, 지상고 0.024 m. 필터 0.025–0.35 m는 명목 차체 높이를 포함하지만 장착물·경사·센서 시야/가림과 지면 가까운 턱을 별도 평가해야 한다. 실측 최고점 및 여유를 확보하기 전에는 0.35 m 상부 제외를 실차 안전 근거로 사용할 수 없다.

## 2026-11-19 웹 Mapping·C103 지도 등록·지도 클릭 목표 (격리 검증)

- Mock PASS: Mapping 세션 20, 영역 5, 목표 사전 검사 Node 11, 기존 Registry 45.
- 전체 `fleet_control/tests` 회귀: 54 PASS / 17 FAIL. 실패 17개는 변경 전 백업에서도 같은 스크립트·같은 오류 수로 실패하는 기존 실패다.
- 격리 ROS 2 (localhost, domain 221, 모터 잠금, 정지 합성 센서): 실제 SLAM Toolbox·`fleet_frontier_candidates`·Nav2·잠금 Gate로 START/PAUSE/SAVE/RESUME/STOP 22/22 PASS, Agent 강제 종료 장애 6/6 PASS. 재시작 후 자동 재개 없음.
- 웹 UI(내장 Chromium) E2E PASS. 지도 클릭 목표는 서버 `safety_not_ready`에서 정지하므로 Nav2 도달은 미검증이다.
- Gazebo SKIP, 실차 BLOCKED.
- C103 지도 `c103_lounge_20261119_depthmem#959a701c0974`(UNVERIFIED)는 격리 Registry에만 등록했고, 현장 Registry에는 미등록이다.
- [상세](fleet_control/docs/WEB_MAPPING_20261119.md)

## 2026-11-19 후속: 실차 전환 전 조사

- Depth 높이: 실차 기록 TF상 `base_footprint`(=`base_link`)는 바닥 위 약 0.145 m에 있다. 필터는 바닥 기준 약 0.17–0.50 m만 본다. 실측 필요.
- CONTINUE_MAP 격리: 부모 셀 100% 보존, 새 셀 +3,215, 부모 해시 불변. 잘못된 시작 자세에서는 벽 중복이 발생한다. Worker 통합은 미구현.
- 웹 제어 상실 시나리오:
  - Lease 만료·네트워크 단절·서버 재시작: PASS (Nav2 idle 3.1–4.3 s)
  - **Agent SIGKILL: FAIL** (goal과 속도가 계속됨)
- 현장 Registry: 사본 drill PASS(백업·등록·map_server 재로드·복원). 원본 해시 불변, 운영 DB 미변경.
- 기존 실패 17개: 신규 0. 14개는 의도한 환경에서 PASS. 3개는 오래된 기대값(ownership, recovery, phase20_agent_stub 1건)으로 백업에서도 동일하게 실패.
- [상세 §9–10](fleet_control/docs/WEB_MAPPING_20261119.md)

## 2026-11-20 Motion Watchdog·회귀시험 복구

- 원인: 이전 `STARTUP_NO_HEARTBEAT`는 틀린 사유 문자열이었고, heartbeat는 수신 중이었다. 실제 원인은 Nav2가 goal이 없을 때 status를 발행하지 않아 재무장 조건(`nav_active==0`)이 영원히 불충족된 교착이다.
- 수정: `LOCKED/MOTION_AUTHORIZED/FAILSAFE`, cancel-all 빈 응답을 idle 증명으로 사용, 제어 루프·Gate 상태 데이터 신선도 감시.
- 격리 ROS 2 고장 7종 모두 유효 실행에서 PASS.
  - Gate 출력 0까지: Agent·watchdog·Gate 고장 0.23–0.32 s, 서버 단절·Lease 만료 3.3–4.4 s.
  - Gate SIGKILL 시 goal 유지 시간: 20 s(수정 전) → 0.48 s.
  - INVALID 실행 2건(Nav2 기동 실패)과 SIGCONT 후 executor 미재개 1건은 기록했다.
- 회귀: 의도한 환경별 75/75 PASS. ownership·recovery·phase20을 계약 v1로 복구했다. 오늘 추가한 heartbeat로 깨진 stub 시험 9종은 stub 보완으로 복구했다.
- 실차 미검증: 제동 거리, MCU `/cmd_vel` 타임아웃, watchdog 실차 launch 편입.
- [상세](fleet_control/docs/SAFETY_WATCHDOG_20261120.md), [증거](local_data/raw_results/safety_watchdog_20261120/)

## 2026-11-20 오후 Watchdog·Gate 통합 검토와 실차 잠금 준비

- 정정: 오전 고장 7종 시험은 작업트리 Gate가 아니라 **설치본 Gate(`capstone_ws/install`, 11-10 빌드)**로 실행됐다. 이번 재시험은 작업트리 Gate + 제안 03b·04 사본으로 실행했다(원본 미수정).
- 패치 03(원본) 결함: watchdog 상태를 `'OK'`와 비교 → 켜면 항상 0 출력. 02(원본)는 조이스틱 진단·explore_lite 직접 goal과 충돌. 개정안 02b·03b·04는 `git apply --check` 통과, **미적용**.
- 실측: rclpy 액션 서버가 cancel을 거부하면 활성 goal이 있어도 `return_code 0`·빈 `goals_canceling`. watchdog idle 증명에 return_code·상태 발행자 1개·settle·상태 배열 조건 추가. Nav2는 유효 실행 전부 `return_code 0`.
- 격리 재시험 10종(신규: 외부 goal, `/cmd_vel_raw` 우회 발행자, watchdog SIGSTOP): 9종 PASS, watchdog SIGSTOP 15/16(재개 후 모니터 재발견 실패로 1개 검사 관측 불가, Gate·watchdog 로그상 정상). 대조(Gate 요구 끔)는 우회 명령이 25 s 출력되어 예상대로 FAIL.
- 재시작 정책: watchdog trip·재시작 시 이전 승인 미션 전부 무효(`test_watchdog_admission.py` 4/4, 반복 12/12; 계측 실행 1회 원인 미확인 오류, 6회 재현 안 됨).
- 회귀: 의도 환경별 76/76 PASS (namespace 5, no-ROS 11, ROS 57, Node 3; 신규 `test_watchdog_admission` 포함). [요약](local_data/raw_results/safety_watchdog_20261120b/regression_summary.txt)
- 현장 Registry: 변경 없음. B1 백업(`local_data/field_registry_backups_20261120/`, 원본과 7/7 일치, integrity ok). C103 등록·표시 리허설은 B1 사본에서 PASS(area PARTIAL_FLOOR, UNVERIFIED, floor active map 아님). 원본 C103 증거 SHA 불변.
- 실차: LIMO offline으로 미시험. 18190 미실행, 현장 런처 `FLEET_SRC` 경로 부재.
- [상세](fleet_control/docs/SAFETY_WATCHDOG_20261120.md) §6, [실차 준비](fleet_control/docs/FIELD_LOCKED_READINESS_20261120.md), [증거](local_data/raw_results/safety_watchdog_20261120b/)

## 2026-11-20 공개 기준선과 후속 작업 감사

| 근거 | 환경·조건 | 관측 결과 |
|---|---|---|
| 11-20 초기 Watchdog 시험 | 격리 ROS 2, 이전 설치본 Gate | 서버 단절 Gate 출력 차단 4.42 s, lease 만료 3.32 s. 전제 미충족 INVALID와 이전 FAIL을 보존했다. |
| 11-20 후속 회귀 | 스크립트별 의도 환경 | 76/76 PASS(격리 5, ROS 없이 11, ROS 57, Node 3). Watchdog 단위 15/15 PASS 포함. |
| 11-20 후속 고장 주입 | 격리 ROS 2, 모터 잠금, 작업트리 Gate에 03b·04를 적용한 시험 사본. 구동 토픽 연결 없음. | 표준 실행의 Agent SIGKILL 0.27 s, SIGSTOP 0.28 s, 재시작 0.25 s. 서버 SIGSTOP 4.50 s, lease TTL 10 s 만료 3.31 s. 고장 시각부터 Gate 출력 0까지이며, Goal terminal 시점은 별도로 계측했다. |
| 후속 Watchdog SIGSTOP 25 s | 같은 시험 사본 | 유효 재시험 두 회 각각 15/16, Gate 출력 0은 0.24 s·0.22 s. SIGCONT 후 재발견 관측 검사 FAIL 유지. Nav2 기동 전제 미충족 실행과 초기 스모크는 INVALID. Gate 요구를 끈 대조 시험은 기대대로 FAIL(10/12). |
| CONTINUE_MAP | 독립 격리 Pose Graph 재로드 | 부모 셀 100% 보존, +3,215셀, 부모 해시 불변. 초기 자세가 틀리면 벽 중복. Worker 연결과 실차 시험은 미실행. |

원본은 로컬 `local_data/raw_results/safety_watchdog_20261120*/`의 실행별 출력과 회귀 요약에서 확인했다. 후속 매니페스트의 `summary.txt` 해시가 불일치하여 해당 집계는 개별 실행 출력과 교차 확인했다. 이번 감사에서 기존 ROS 2, Gazebo, 실차 시험을 재실행하지 않았다.

최신 시험 근거는 로컬 `local_data/raw_results/safety_watchdog_20261120b/`의 `r1_agent_kill.out`, `r1_agent_freeze.out`, `r1_agent_restart.out`, `r1_server_partition.out`, `r1_lease_loss.out`, `r{2,3}_watchdog_freeze.out`, `regression_summary.txt`다. 봉인 원본과 보고서는 보존했다.

이번 감사에서 로컬 회귀를 선별 재실행했다. Mapping 세션·계보·Worker 정책 26/26 PASS(ROS 2 Humble Python 패키지 사용, Mock). Watchdog admission 4/4 PASS(임시 loopback 서버). 초기 실행의 Mapping 7건은 ROS Python 환경 미로딩으로 SKIP됐고, Watchdog admission은 샌드박스의 소켓 제한으로 setup에서 ERROR였다. 위 최종 결과는 각 환경을 갖춘 후의 재실행이다.

## 2026-11-20 저녁 18190 복구·정지 시간 단축

- 18190: 로컬 `fleet_control/` + B1 사본(C103 등록)으로 실행. 인증 401/200, C103 시설·건물·층·영역·버전·센서 출처 API 확인. 현장 data 미변경. 브라우저 화면은 토큰 로그인 필요로 미확인. LIMO 전원 꺼짐 → 실차 점검 미실행.
- Lease 만료 → Gate 출력 0: 3.31 s → 0.28/0.31/0.41 s. 서버 단절: 4.50 s → 3.18/3.06 s(INVALID 1회, Nav2 기동 실패). 격리 ROS 2, 개정 Gate 사본.
- 회귀 76/76 PASS(`test_web_mapping_session` 26/26 포함). [요약](local_data/raw_results/safety_watchdog_20261120b/regression_summary_fence.txt)
- 실차 Mapping·지도 클릭 Goal 남은 조건: `fleet_control/docs/FIELD_LOCKED_READINESS_20261120.md` §7.

## 2026-11-20 실차 Agent 설치·모터 잠금 스택·웹 연결

- 로봇 백업(tar·해시, 노트북 사본 일치) 후 `limo_robot_agent` 신규 설치(기존 파일 변화 0), 로컬 도메인 77 환경 `env_local77.sh`. Agent 연결·인증 PASS(프로토콜 2·계약 1).
- 모터 잠금 스택 기동: `/cmd_vel` 발행자 0, Gate NORMAL, LiDAR 10 Hz, odom 49.9 Hz, IMU 99.8 Hz, Depth 11 Hz, RGB 13.6 Hz, Nav2 active, 웹 Online·센서·지도·Gate·LOCKED 표시, Lease PASS, START·Goal 거부 PASS. 웹 프로브 17/0(Depth 끔), 13/0(Depth 켬).
- FAIL/주의: 첫 기동 중 베이스 UART 허브 재열거(원인 미확정, 재기동 2회 재현 안 됨). 카메라 웹 표시·YOLO 미준비.
- [상세](fleet_control/docs/FIELD_LOCKED_READINESS_20261120.md) §9–10, [로그](local_data/raw_results/limo_runs/20261120_locked_agent_01/)

## 2026-11-20 Phase 1 웹 수동 조작 격리 E2E (Claude)

- 범위: 격리 localhost ROS(domain 228), 실제 Fleet 서버·Agent(mock, locked arbiter)·Motion Watchdog 프로세스, 현장 번들 Gate 코드(`/cmd_vel:=/cmd_vel_locked`, 드라이버 없음), rclpy NavigateToPose/FollowPath ActionServer, Chrome 실제 대시보드. 센서 합성, Gamepad 합성.
- 결과: `e2e_run08`·`09`·`10` 각 18/18 PASS — 정상 주행(≤0.10 m/s, 0.15 rad/s), Deadman·입력 중단·STOP·Lease 만료/강제 해제·Mapping 활성·외부 NavigateToPose/FollowPath goal·Nav2/Agent/Watchdog/Fleet 서버/Gate 중단 시 0(최대 0.235 s), 복구 후 새 세션 재주행, 키보드·터치·Gamepad 브라우저 입력, 375 px 레이아웃. 개발 중 run01–07의 하네스 결함 실패도 보존.
- 회귀: 변경 전 트리와 같은 환경 비교에서 신규 실패 없음. 기존 실패 유지: `run_checks.py` mission 201, `test_manual_bench_version_comparison`, `test_manual_arbiter_freeze_20261120`. SKIP(환경): network namespace 요구 5개, 후보/공개 트리 경로 2개. 의도된 기대값 변경: Codex chain `active_goal` 모드 FAIL(검토 요청).
- 미검증: 실제 Nav2 스택, Gazebo, 실차, 물리 Gamepad, 모터 출력. 증거 `local_data/raw_results/manual_phase1_claude_20261120/`, 보고 `fleet_control/docs/MANUAL_PHASE1_INTEGRATION_20261120.md`.

## 2026-11-20 Phase 2 운용 모드 전환·웹 맵핑 격리 E2E (Claude)

- 범위: 격리 localhost ROS(domain 228). 실제 Fleet 서버(`--require-operating-mode`)·Agent·Watchdog·Nav2 6개 서버·SLAM Toolbox·`fleet_frontier_candidates`, r10 후보 Gate(`/cmd_vel_locked`), Gate 출력만으로 움직이는 2D 운동학 시뮬레이터. NAVIGATION의 지도·localization은 FIXTURE(저장 SLAM 지도 + 정적 map→odom)다.
- 결과: `e2e_run01`, `e2e_run02` 각 16 PASS / 0 FAIL / 1 SKIP(f06은 f05 뒤 reconciliation 잔존으로 건너뜀), `e2e_run03_f06` 8/8.
  - 확인한 전환 9종 + FAULT 복구.
  - 맵핑: START·PAUSE·SAVE(Registry UNVERIFIED)·RESUME·STOP, SAVE 중 전환 거부.
  - 장애: Lease 손실, 서버·Agent·Watchdog SIGKILL, Lease 만료 + RECONCILE, bt_navigator 정지(취소 무응답) → 모두 0 출력, FAULT/IDLE, 자동 재출발 없음.
- 회귀: Phase 1 E2E 18/18, 서버·Agent 단위 및 계약 시험 PASS. 기존 실패(`run_checks` 2건 등)는 변화 없음.
- 미검증·한계: Gazebo(Arbiter 미연결), 실차, 실제 localization, 로봇 단위 reconciliation의 웹 복구(admin recover `no_evidence`). 증거: `local_data/raw_results/mode_phase2_claude_20261120/`.

## 2026-11-20 심야 Codex 결함 수정·Phase 3 격리 E2E (Claude)

- Codex 재현 시험: boundary 8/8, Watchdog GID 2/2, Gate 3/3(수정 전 2/8, 0/2, 3/3). 단위·계약 회귀 전부 PASS.
- E2E(domain 228, 실제 Nav2·SLAM Toolbox·AMCL·map_server, 2D 운동학 sim, r10 후보 Gate 잠금 출력): `phase1_e2e_run02` 18/18, `mode_e2e_run04` 17/17, `nav_e2e_run02` 17/17. Gate 환경 unlatch와 Nav2 재기동 사용은 각 `result.json`에 기록.
- 미검증: Gazebo, 실차, 실제 DDS 메시지 출처 경합(합성 시험만). 증거 `local_data/raw_results/codex_fix_claude_20261120/`.

## 2026-11-20 Gate 입력 독점 검증 (격리, SIM_ONLY)

| 항목 | 상태 | 근거 |
|---|---|---|
| r10/field Gate `/cmd_vel_raw` 우회 재현 | PASS(재현됨) | `experiments/safety_chain_fix_20261113/runs/gate_r11_input_20261120/before_r10_run2`, E2E s18/g01–g03/f07b |
| r11 단독 10종(정상·외부·외부단독·교체·원본퇴장·Twist·만료·Watchdog/Gate 재시작) | PASS (위조 단독 발행자 수락 = 알려진 한계) | `after_r11_run2`, `r11_run3_fixed_cases`, `r11_run4_watchdog_restart` |
| r11 단위 13종(GID 조회 실패 포함) | PASS | `harness/gate_r11_input/test_gate_r11_input_unit.py` |
| Phase 1/2(원래 17)/3, r11 | 18/0/1(s16 미실행), 17/0/0, 18/0/0 | `local_data/raw_results/gate_input_claude_20261120/` |
| 제안 05 패치 사본 단독 10종 | PASS | `runs/.../p05_capstone_ws_patched_copy` |
| Gazebo | SKIP | 보고서 §6 |
| 실차 | 미시험 | — |

## 2026-11-21 웹 Gate Reset 검증 (격리, SIM_ONLY)

| 항목 | 상태 | 근거 |
|---|---|---|
| 서버 권한·감사·동시성·재생·재시작 단위 14 | PASS | `fleet_control/tests/test_gate_reset_server.py` |
| Agent 사전검사·관찰 창·journal 단위 11 | PASS | `fleet_control/tests/test_gate_reset_worker.py` |
| Gate r12 Reset 판단 단위 14 | PASS | `experiments/safety_chain_fix_20261113/harness/gate_r12_reset/` |
| 격리 E2E 13종 × 2 | PASS | `local_data/raw_results/gate_reset_claude_20261121/final_rerun2/` |
| Phase 1(r12, 브라우저) 19 × 2, s10 | PASS | `final_rerun/phase1_r12_browser_{1,2}` |
| Phase 2 원래 17(r12) | 16/1 (f06 localization) | `final/phase2_original17_r12` |
| Phase 3(r12) | 미판정 (r11도 변동) | 보고서 §4.4 |
| Gazebo·실차 | 보류 | — |

## 2026-11-21 Phase 3 안정화·r11/r12 비교 (격리)

| 항목 | 결과 | 근거 |
|---|---|---|
| Phase 3 교대 8회(고정 지도, 스냅샷 동일) | r11 17/1, 18/0, 18/0, 18/0. r12 18/0 ×4 | `local_data/raw_results/phase3_stability_claude_20261121/final_series` |
| Gate boot 재조정 재현 r01 | PASS (r11·r12) | `.../closing/r01_*` |
| Phase 1 + 브라우저 | 19/19 ×2 (r11·r12 각각) | `.../closing_s10` |
| Phase 2 원래 17, Gate Reset E2E | 17/17 (r11·r12), 13/13 (r12) | `.../closing` |
| Gate 프로세스 정지(E) 원인 | 미판정 | 보고서 §9 |
| Gazebo·실차 | 미검증 | — |

## 2026-11-21 Gazebo 웹 제어 · Gate 정지 · Phase 3.5 회귀 (격리)

| 항목 | 결과 | 근거 |
|---|---|---|
| 단위(리팩토링 후) | 90 / 21, 실패 목록이 변경 전과 동일 | `local_data/raw_results/refactor35_claude_20261121/unit_after_r3_gz` |
| Phase 1 / 2 / 3 / Gate Reset | 18/1 (s16 하네스 창) → 수정 후 19/0, 16/1 (E), 18/0, 13/0 | `.../refactor35_claude_20261121/{e2e_after_r1r2r3,phase1_after_s16fix}` |
| Gate 정지(E) 추적 재현 | 계측 7회 0건, 계측 없는 Phase 2 1건 | `local_data/raw_results/gate_stall_gazebo_claude_20261121/{repro_e*,phase2_traced*}` |
| 정지 주입(0.2/0.35/0.5 s × freeze/delay × Gate/both × hold/timeout) | 24 case 전부 실행. 정지 중 출력 0. 0.5 s는 FAILSAFE | `.../stall_injection_1` |
| Gazebo 웹 시나리오 | 실행 6회, 시나리오별 표 | `.../gazebo_runs_matrix.txt`, 보고서 §6~7 |
| STM32 timeout, 실차 | 미측정 / 미검증 | — |

## 2026-11-21 Gazebo 안정화 · 하위 정지 (격리)

| 항목 | 결과 | 근거 |
|---|---|---|
| 하위 timeout 0.2 s: Gate SIGSTOP/SIGKILL, +Watchdog | 하위 0 0.19~0.22 s, 정지 0.22~0.26 s, 이동 0.020~0.024 m(2회). hold는 0.04~0.20 m, SIGKILL이면 미정지 | `local_data/raw_results/gazebo_drive_timeout_claude_20261121` |
| 포인터 입력(15주기 + 끌기·STOP·cancel) | 수정 전 12/15 도달(3/15만 끝까지 유지), 수정 후 45/45 | `.../gazebo_pointer_claude_20261121` |
| Frontier 맵핑 반복 | 수정 후 8/8 탐색(0.85~17.08 m), 수정 전 4/6 정체 | `.../gazebo_mapping_click_claude_20261121` |
| 지도 클릭 E2E | 거부 경로 전부 PASS, 정상 완료 1회 | 같은 곳 |
| 회귀 Phase 1/2/3/Gate Reset, 단위 | 19/0, 17/0, 18/0, 13/0, 91/21(불변) | `.../refactor35_claude_20261121/{e2e,unit}_after_stabilization` |
| Gate E, STM32 timeout, 실차 | 미재현 / 미측정 / 미검증 | — |

## 2026-11-21 Phase 4 자동 기동 · Phase 5 방문객 웹 (격리)

| 항목 | 결과 | 근거 |
|---|---|---|
| 단위(readiness 10, supervisor 7, guide 7, panel/QR 구조) | PASS. QR 디코딩 SKIP(디코더 없음) | `fleet_control/tests/test_phase4_*.py`, `test_phase5_guide.py`, `test_qr_encode.py`, `test_system_panel.cjs` |
| Phase 4 격리 ROS(기동 순서 2종, 센서 20 s 지연, Watchdog/Agent/서버 kill, 연속 종료, 권한, 웹 카드) | 8/8 × 2 | `local_data/raw_results/phase4_autostart_claude_20261121/final_*` |
| Phase 5 격리 ROS + 모바일 브라우저(요청·도착·취소·대기열·사유·토큰 격리·지도 클릭) | 8/8 | `.../phase5_visitor_claude_20261121/final` |
| 회귀 Phase 1/2/3/Gate Reset, 단위 | 19/0, 16/0/S1(기존 간헐), 18/0, 13/0, 96/21(불변) | `.../refactor35_claude_20261121/{e2e,unit}_after_phase45` |
| Gazebo, 실차 감독·systemd | 미실행 / 미검증 | — |

## 2026-11-21 Codex Phase 4·5 독립 검증 결함 수정 (Mock / 격리 ROS / Gazebo, 실차 SKIP)

| 항목 | 결과 | 근거 |
|---|---|---|
| Codex `test_phase45_independent` 14 (기대값 미수정) | 수정 전 FAIL 13 / ok 1 → 수정 후 ok 13 / ERROR 1 (시험 내 print 줄이 요구사항대로 404를 받음. Codex에 수정 제안) | `local_data/raw_results/codex_fix_phase45_claude_20261121/{before,after}_independent.log` |
| Codex crosslink HTTP | FAIL → OK | `.../{before,after}_crosslink.log` |
| 결함 수정 unit `test_phase45_fix_20261121.py` 21 | OK | `.../after_fix_tests.log` |
| 전체 unit runner | 0 이외 24 = 기준선 21 + Codex 신규 3(일반 runner에 snapshot env 없음, env 지정 시 통과) | `.../unit_after_fix2/` |
| Phase 1 / 2 / 3 / Gate Reset (격리 ROS) | 19/0, 17/0/0 ×2, 18/0, 13/0 | `.../e2e_regress/`, `.../phase2_r2/` |
| Phase 4 (격리 ROS) | server_first 6/2, 2/6, 8/0; supervisor_first 8/0, 6/2, 1/7 (Nav2 기동 실패) | `.../p4_*` |
| Phase 5 (격리 ROS, 종료 코드) | 8/0 rc=0, 3/5 rc=1, 8/0 (rc=134 해소) | `.../p5*` |
| Gazebo 전체 24 | 13/11 rc=1 ×2 (g03 탐사 무이동 연쇄, 수정 전 전체 하네스에도 동일 증상) | `.../gazebo_web*/` |
| 실차 | SKIP (금지) | — |

- Phase 4·5 실패 회차는 모두 kinematic sim `/scan` 도착 중단 → Gate `FAILSAFE_SENSOR_TIMEOUT` latch(`scan:NO_ARRIVAL`)에서 시작했고, 새 readiness는 이를 차단했다. 같은 하네스가 수정 전에도 0/4·2/4·0/8 등 간헐 실패 이력이 있다. 센서 중단 원인은 미조사.
- Gazebo g03 무이동은 이번 변경과 무관함을 완전히 증명하지 못했다(Agent가 mock이라 새 강제 경로 미사용).
- 상세: [fleet_control/docs/CODEX_FIX_PHASE45_20261121.md](fleet_control/docs/CODEX_FIX_PHASE45_20261121.md)

## 2026-11-22 Gazebo 통합 실패 수정 · Phase 4·5 (Gazebo / 격리 ROS / Mock, 실차 SKIP)

| 항목 | 결과 | 근거 (`local_data/raw_results/gazebo_integ_claude_20261121/`) |
|---|---|---|
| Gazebo 전체 24 (동일 domain·spawn·Gate r12) | 수정 전 13/11 ×2 → r3 18/5/SKIP1, r4 18/5/SKIP1 | `after_full_r{1..4}/` (r2 3/21은 하네스 방향 선택 부작용, 수정 후 r3) |
| g03 Mapping 탐사 | 수정 전 전체 실행 1/7 → r3·r4 PASS | `after_g03seq_r1/`, `after_full_r3/`, `after_full_r4/` |
| f07 취소+bt_navigator 정지 | 수정 전 4.0 s+/0.50 m 계속 주행 → 0.17 s/0.016 m | `after_full_r3/`, `after_cancelfix_nav_r2/` (7/0) |
| Phase 1 / 2 / 3 / Gate Reset (취소 수정 후) | 19/0, 17/0, 18/0, 13/0, snapshot 동일 | `regress_after_cancelfix/` |
| Phase 4 ×4 / Phase 5 ×2 (timeline 부착) | 7/1(CLI 그래프 조회 흔들림), 8/0 ×3; 8/0 ×2; `/scan` 최대 간격 0.13 s | `p4_*`, `p5_*` |
| DDS 재시작 프로브 (default/UDP) | 14 cycle 중단 0 → SHM 가설 기각 | `dds_probe_*` |
| Gate 재개 r12 / 후보 | r12 STALE_REUSE(0.2 s 정지 5/5 등) / 후보 zero·silent OK, live ≤0.12 s | `gate_resume_{r12,r12_live,candidate}/` |
| unit runner | 0 이외 24 = 기준선 21 + Codex 신규 3(snapshot env 필요; 지정 시 2 OK, 1 ERROR는 의도된 404) | `unit_after_steady_timers/` |
| 실차 | SKIP (금지) | — |

- 남은 Gazebo 실패: f04(f03 DDS 유령 writer 연쇄), g05→g06·f06·f07 전제(지도/위치 추정 품질, 가설), r3 g08 장애물 직진(Nav2 설정 가설). 상세: [GAZEBO_INTEG_FIX_20261122](fleet_control/docs/GAZEBO_INTEG_FIX_20261122.md)

## 2026-11-22 (2) 속도 재출력 안전 · Gazebo 잔여 실패 (SIM_ONLY, `local_data/raw_results/gazebo_p0_claude_20261122/`)
| 항목 | 결과 | 근거 |
|---|---|---|
| Arbiter 세대 단위 시험 7건 | 수정 전 7 FAIL → 7/7 OK | `unit/` |
| Watchdog 자기 정지 후 밀린 입력 | 수정 전 FAIL → OK (Watchdog 23건 포함) | `unit/` |
| unit runner | 0 이외 22, 기준선 부분집합 (신규 실패 없음) | `unit_suite_after/` |
| Gate 재개 격리 (r13 + 상태 writer) | zero/silent OK 전 조건, live 0.105–0.118 s, NOT_ARMED 2 (rig 기동) | `resume_r13/` |
| writer 교체 그래프 (r4 상태) | r12 ACCEPTS_UNVERIFIED_NEW_WRITER / r13 ZERO_UNTIL_RESET | `gate_writer_swap_graph.json` |
| Gazebo 전체 r12 ×2 | 21/2/1, 20/3/1 (이전 r3·r4 18/5/1) | `full_r12_{a,b}/` |
| Gazebo 전체 r13 ×1 | 19/4/1 — g03 후 의자 끼임이 첫 실패 원인 (Gate 무관) | `full_r13_a/` |
| Gazebo d01/d03/d04 재개 후 비영 출력 | r12 1/0/1, 1/1/1 → r13 0/0/0 | 위 동일 |
| Phase 1/2/3/Gate Reset, Phase 4 ×2 | 19/0, 17/0, 18/0, 13/0, 8/0, 8/0 | `regress_after/` (콘솔 .out 덮어씀, `INCIDENT_20261122.txt`), `p4_*` |
| Phase 5 ×3 | 6/2, 8/0, 8/0 (r1만 v04 `failed to create plan`, v07 Nav2 abort, 간헐; v07는 11-21에도 1/3 실패) | `p5_r{1,2,3}/` |
| 실차 | SKIP (금지) | — |

## 2026-11-22 (3) Gate r13 통합 준비 · 속도 세대 계약 (SIM_ONLY, `local_data/raw_results/gate_r13_integ_claude_20261122/`)
| 항목 | 결과 | 근거 |
|---|---|---|
| smoother 꼬리 (실제 Humble velocity_smoother) | 무음 정지 0.91–0.95 s 재출력 (3/3), 0 정지·0 연속 0 s | `smoother_tail.json` |
| Arbiter 세대 단위 14건 | 14/14 OK; 이전 Arbiter는 smoother 재출력 2건 FAIL | `unit/` |
| unit runner | 0 이외 22/123, 이전 기준과 동일 집합 | `unit_suite/` |
| 격리 rig zero/silent/live/swap ×3 | r12 STALE_REUSE 20/35 / r13 0/36 (15:10 정정, 이전 23은 집계 오류); live 18/18 둘 다; swap r12 18 RECOVERED, r13 17 RECOVERED + 1 NOT_ARMED (SKIP) | `rig_r1{2,3}_resume_swap/` |
| Gazebo r13 ×2, r12 ×1 | 5/18/1, 20/3/1; r12 17/6/1 (실패 원인 모두 Gate 외: D3·D2·UNKNOWN_AREA) | `gz_r13_{a,b}/`, `gz_r12_c/` |
| d01–d04 재개 후 비영 출력 | r12 2/1/1/1 → r13 0/0/0 | `gz_r12_c/`, `gz_r13_b/` |
| Phase 1/2/3/Gate Reset (r13) | 19/0, 16/1 (f04 시험 중 sim scan 끊김), 18/0, 13/0 | `phase{1,2,3}_r13/`, `gate_reset_r13/` |
| Phase 4 ×2, Phase 5 ×3 (r13) | 8/0, 8/0; 5/3, 8/0, 8/0 (v04 RPP 호 주행으로 장애물 0.16 m, D3) | `p4_*`, `p5_r13_*` |
| 기록 보호 | 실행마다 run_guard MANIFEST/FINISH, 실행 중 소스 변경 0 (14/14) | 각 실행 디렉터리 |
| 정식 Gate·실차 | SKIP (미인계·금지) | — |

## 2026-11-22 (4) 실차 조사 · Gazebo LiDAR+Depth (SIM_ONLY, `local_data/raw_results/{real_limo_integ_claude_20261122,gazebo_depth_claude_20261122}/`)
| 항목 | 결과 | 근거 |
|---|---|---|
| 실차 접근 | SKIP (pcu OFFLINE) | survey §1 |
| 실차 Gate HEAD `d103f445` 격리 rig | zero STALE_REUSE 9/18, silent 6/17 (+1 NOT_ARMED), live OK 18/18, swap 미검증 writer 수용 17/17 (+1 NOT_ARMED) | `rig_robot_gate_HEAD/` |
| Depth 시작 조건 (status·freshness·TF·costmap 구독·Gate required) | 측정 회차 전부 충족; 미충족 4회 = Nav2 lifecycle 응답 timeout (NOT_READY, 제외) | `c_*/run/*/run.json` |
| 4단계 검출 (2.0 m 정지) | raw/filter/LiDAR/local LETHAL: chair_base 321/179/2/88, chair_bars 345/265/4/84, desk_frame 972/500/4/110, low_box 101/142/0/50, glass 2025/1932/40/60, none 0/0/0/0 | `c_depth_sep/` |
| 회피, Depth 별도 layer | 5 대상 × 2 PASS_AVOIDED 10/10, 최소 0.190 m; 대조 1/1 | `c_depth_sep/`, `c_control_sep/` |
| 회피, Depth 공유 layer | chair_base 2/2 (0.079 m), chair_bars 2/2, desk_frame FAIL 2/2, low_box 1 PASS/1 FAIL, glass NOT_READY 2 | `c_depth_shared/` |
| LiDAR 단독 (BASELINE) | chair_bars 2/2, glass 2/2 (실차 무효), chair_base FAIL 1, desk_frame·low_box INCONCLUSIVE 3 (Gate STALL 오정지), NOT_READY 2 | `c_lidar/` |
| Depth 고장 (SIGSTOP) | Gate FAILSAFE_SENSOR_TIMEOUT 정지 0.53 s·0.549 s, 접촉 없음 2/2 | `c_depth_fault_sep/` |
| Collision Monitor | N/A (이 스택·실차 launch 모두 미배치) | — |
| 실차 Depth 정지 의자 받침 | SKIP (pcu offline), 절차 준비 | survey §9 |
| 기록 보호 | run_guard MANIFEST/FINISH, 기존 run.json 재작성 없음 | 각 디렉터리 |

## 2026-11-22 (5) 실차 통합 전환 준비 (실차 SKIP)
| 항목 | 결과 | 근거 |
|---|---|---|
| pcu 접근 (18:25) | SKIP: Tailscale OFFLINE, ping 응답 없음 | survey §10.1 |
| Agent 배포 후보 real 정책 단위 | `test_real_locked_policy` 2/2 OK, `test_phase4_supervisor` 7/7 OK; `test_agent_safety_v1` 단독 실행 19 errors (fixture의 rclpy.init 누락, 판정 안 함) | 노트북 실행 |
| 실차 잠금 통합·Depth 정지 시험·FAILSAFE | SKIP (로봇 offline) | survey §10.5 |


## 2026-11-22 (6) Nav2 D3 · D1/D2 (SIM_ONLY, `local_data/raw_results/nav2_d3_claude_20261122/`)
| 항목 | 결과 | 근거 |
|---|---|---|
| D3 표적 behind_goal | base 0/5 → O1 5/5, O6 3/3 (여유 0.34 m) | `tg_*`, `targeted_summary.md` |
| corridor_uturn 0.66 m | 모든 프로파일 0/3~0/5 (미해결) | 동일 |
| RPP 충돌 검사 O2 | wall_oblique 0/3 (오중단) → 기각 | `tg_o2_a` |
| 취소 후 마지막 비영 출력 | 모든 프로파일 0.06–0.17 s | 동일 |
| D1 ICP 순수 회전 | −4.3~−7.0 % / −12.4~−12.9 %, 재생 일치 | `d1_rot_*` |
| D2 충돌·시각 메시 | 의자 30, AdjTable 10 은폐 | `d2_collision_vs_visual.json` |
| Gazebo 전체 | base 20/3/1, 21/2/1; O6 18/5/1 (계획 3+3 중 2+1) | `gz_*` |
| Phase 1/2/3/Gate Reset (O6) | 18/1, 17/0, 18/0, 13/0 | `phase*_o6`, `gate_reset_o6` |
| Phase 4 server_first (O6) | 1회 무효(Nav2 lifecycle 응답 유실, loadavg ~10), 재시험 8/0; supervisor_first 미실행 | `p4_*` |
| Phase 5 (O6) | SKIP (조기 종료) | — |
| 실차 | SKIP | — |

## 2026-11-22 (7) 실차 기존 검증 재사용 판정 (실차 읽기 전용, 시험 실행 없음)
| 항목 | 판정 | 근거 |
|---|---|---|
| 로봇 소프트웨어가 11-20 기록과 동일 | Verified (해시) | survey §10.7, plan §1 |
| LiDAR·Odom·IMU·TF·Depth·SLAM·지도 저장·웹 프로브 | 재사용 (재시험 안 함) | plan §2.1 |
| Agent 후보 real 기동·STOP/만료/재시작/링크 단절 ACK, 낮은 구조물 Depth, Depth 정지 FAILSAFE, 격리 서버 | 신규, 미실행 (현장 안전 확인 대기) | plan §2.2, §6 |
| 독립 모터 잠금 L4(물리) | 사용자 확인 필요 | plan §5 |

## 2026-11-22 (8) 격리 서버·Agent 후보 배포 전 (노트북, `local_data/raw_results/real_limo_integ_claude_20261122/isolated_server_prep_01/`)
| 항목 | 결과 |
|---|---|
| 격리 서버 18290 (루프백) + Agent real 연결 | PASS (protocol 2, contract 1) |
| 프로브(센서 그래프 없음) viewer / operator --refusal | 5/6/2 / 9/6/2 (FAIL = NO_DATA 예상, movement 불허 PASS, START·goal 409 PASS) |
| real 모드 Arbiter / navigation_localization / map_save+missions | 거부 rc 1 ×3 |
| real 모드 web_gate_reset / manual_web_bench | 거부 안 됨 (후자는 설계상 허용). 둘째 발행자 시험 SKIP |
| 회귀 단위시험 | 지도 저장 17/17·20/20, readiness 10/10, status 7/7, 그래프 계약 1/1, bench 계약 5/5, Lease 2/2, r3r4 2/2, real policy 2/2, phase4 supervisor 7/7 |
| 롤백 체크리스트 | 작성만, 미실행 |

## 2026-11-22 (9) Agent real 모드 차단 수정·회귀 (노트북, `local_data/raw_results/real_limo_integ_claude_20261122/agent_fix_regress_01/`)
| 항목 | 결과 |
|---|---|
| 신규 `test_real_mode_agent_blocks_20261122` | 5/5 OK; 수정 전 Agent 대조 3 FAIL + 1 ERROR |
| 회귀 | manual bench 4개 모듈 13, 지도 저장 17+20, readiness 10, status 7, 그래프 특성 1, Lease 2, r3r4 2, real policy 2, Phase 4 supervisor 7, Arbiter 세대 14, 잠금 Arbiter 3 전부 OK |
| 배포 후보 v2 | 변경 2파일, tar sha `2fc8305e…` |
| 로봇 교체·ROS 기동·명령 주입 | 미실행 |

## 2026-11-22 (10) Agent v3 설정 오류 가드·루프백 검증 (노트북, `local_data/raw_results/real_limo_integ_claude_20261122/agent_v3_*`)
| 항목 | 결과 |
|---|---|
| `test_real_mode_agent_blocks_20261122` | 13/13 OK; v2 Agent 대조 5 FAIL |
| 회귀 15개 모듈 | 전부 OK |
| fixture 기반 `test_agent_safety_v1`·`test_terminal_agent_order`·`test_r8_reconcile_cancel` | 오류 (수정 전 v1 Agent와 동일 개수) → 미판정 |
| 루프백 real Agent ↔ 18290 | PASS: online/Lease ACK·만료/START·Goal·Manual·Gate Reset 거부/재시작 새 boot/서버 재시작 재연결 |
| 실차 ↔ 18290 | SKIP (미배포, 승인 대기) |
| ufw 18290 규칙 | 미적용 (root·승인 필요), 타 기기 차단 확인 SKIP |

## 2026-11-22 (11) Stage A 실차 배포 (`local_data/raw_results/real_limo_integ_claude_20261122/stage_a_deploy_01/`)
| 항목 | 결과 |
|---|---|
| 로봇 재확인 | PASS (ROS/limo_base/Agent 프로세스 0, `/dev/ttylimo` 미사용) |
| 로봇 백업 + 노트북 사본 SHA256 | PASS (src `3e1f2811…`, install `09c19bbb…`, 양쪽 일치) |
| v3 교체·빌드·해시 | PASS (32파일 일치, build rc 0) |
| ufw 규칙 | FAIL/중단 (sudo 비밀번호 필요) |
| 토큰 설치, 서버 기동, Agent 연결, ONLINE/Lease/ACK/재연결/거부 (실차) | SKIP |

## 2026-11-22 (12) Stage A 실차 연결 (`local_data/raw_results/real_limo_integ_claude_20261122/stage_a_connect_01/`)
| 항목 | 결과 |
|---|---|
| 서버·터널 바인드(노트북·로봇 모두 loopback만) | PASS |
| ONLINE / robot_id / boot_id / heartbeat | PASS |
| Lease 획득·반납·만료(10.1 s) / 명령 ACK | PASS |
| Agent 재시작(새 boot id) / 서버 재시작 / 터널 단절 재연결 | PASS ×3 |
| Mapping START · Goal · Manual · Gate Reset 거부 | PASS (409/403) |
| `limo_base`·모터 포트·`/cmd_vel*` | 없음/미사용 (PASS) |
| Agent 정상 종료(SIGINT/SIGTERM) | FAIL (SIGKILL 필요) |
| 토큰 삭제·터널·서버 종료 | PASS |

## 2026-11-22 (13) Agent 정상 종료 수정 검증 (노트북, `local_data/raw_results/real_limo_integ_claude_20261122/agent_shutdown_fix_01/`)
| 항목 | 결과 |
|---|---|
| 종료 matrix 수정 전 (서버 정상·불가·중 단절·재연결 × SIGINT/SIGINT 무시/SIGTERM) | 0 PASS / 12 FAIL (traceback 4, 무한 대기 8) |
| 같은 matrix 수정 후 (live tree) | 12/12 PASS |
| Lifecycle (heartbeat·Lease probe 12/12·SIGTERM·offline·재시작 새 boot id·SIGINT) | PASS |
| 신규 `test_agent_shutdown_20261122` | PASS |
| 회귀 17개 모듈 | OK (fixture 기반 3개는 기존과 동일 오류, 미판정) |
| 실차 Agent 교체 | 미실시 (v4 후보만 준비) |
| LIMO 디렉터리 감사 (읽기 전용) | 완료, 변경 없음 |

## 2026-11-22 (14) 리팩토링 설계 근거 (읽기 전용, `local_data/raw_results/real_limo_integ_claude_20261122/refactor_design_01/`)
| 항목 | 결과 |
|---|---|
| 로봇 `robot_ws/src` 342파일 vs 노트북 소스 해시 | 3(소유자 미커밋)+1(v3/v4) 외 동일; normalizer 4개 experiments 사본과 8/8 동일 |
| 노트북 `fleet_control` 추적 상태 | 646파일 중 현재 브랜치 추적 0 (옛 스냅샷 branch 203) |
| 로봇 11-13 run 인벤토리 | 21개 디렉터리, 5.3 GB, bag 19개; 노트북 사본 이름 없음 |
| v4 추가 적대 시험 (응답 없는 서버) | SIGTERM 2.5 s·무시된 SIGINT 2.5 s 종료, 2회 신호 즉시 종료 |
| 리팩토링 단계별 회귀시험 | 설계 문서 §4에 정의만 (미실행) |

## 2026-11-22 (16) R0 실행 검증 (`local_data/raw_results/real_limo_integ_claude_20261122/r0_copy_01/`)
| 항목 | 결과 |
|---|---|
| 로봇 원본 복사 36개 항목 (871 파일, 8,233,753,902 B): 파일별 SHA256·크기·개수 | PASS 36/36; 노트북 단독 재검증 36/36 |
| 기존 사본 재사용 (전 파일 SHA256 일치) | 9개 항목, 64 파일 |
| 로봇 원본 변화 | 935 파일, 누락 0, 크기 불일치 0 |
| rosbag 23개: SQLite integrity·메시지 수, `ros2 bag info`(metadata 있는 15개) | 22 PASS / 1 BAG_INTEGRITY_FAIL (`20261119_run_002` 원본 손상: 6,520,832 B 누락, 복사본은 원본과 바이트 동일) |
| Fleet 스냅샷 (`fleet_control_snapshot_20261122`, `_02`) 복원·해시·스모크 시험 | PASS (610 / 612 파일) |
| 비밀정보 | 복사 안 함(토큰, `auth.json`, `fleet.db`는 이름·크기·권한만 기록) |
| G1 / G2 / 브랜치 이동 / 커밋 | 미실행 |

## 2026-11-22 (17) G1 리허설 검증 (`local_data/raw_results/real_limo_integ_claude_20261122/g1_prep_01/`)
| 항목 | 결과 |
|---|---|
| 후보 매니페스트(612) 파일별 SHA256·git blob | 생성, 패치 적용 후 기준 |
| 임시 객체 저장소 write-tree | tree `740fd17b…`: HEAD 대비 612 A / 0 M / 0 D, `fleet_control/` 밖은 HEAD와 동일 |
| 금지 경로·토큰 값 9개 | 0 / 0 |
| 패치 P1–P5 dry-run, 패치본 스크래치 시험(IP 0, `bash -n`, 인자 부족 rc 2) | PASS |
| `g1_execute.sh` 스크래치 clone end-to-end + 롤백 | PASS (tree 일치, noreply 작성자·커미터, 부모 HEAD) |
| 실제 저장소 | 브랜치 0개 추가, HEAD·상태·stash 불변 |

## 2026-11-22 (18) G1 실행 검증 (`local_data/raw_results/real_limo_integ_claude_20261122/g1_exec_01/`)
| 항목 | 결과 |
|---|---|
| 커밋 `0ca09dc4…`, tree `740fd17b…` (리허설 tree와 동일), 부모 `c804241a`, 612 A / 0 M / 0 D | PASS |
| 실행 전후 HEAD·브랜치·Index·작업트리 상태·diff·stash·다른 ref | 동일 (ref +1) |
| 작성자·커미터 | noreply 둘 다, HEAD 작성자와 동일, gmail 0 |
| 커밋 blob vs 매니페스트 / 스냅샷 03 | 612/612 일치 / 604 동일 + 8 패치 파일, `live-api.jpg`만 스냅샷에 있음 |
| 금지 경로·토큰 값·tailnet 주소·noreply 조각·기타 이메일 | 0 / 0 / 0 / 0 / 0 |
| 강화한 실행 스크립트 시험 (스크래치 clone): 금지 목록 위반 시 중단, 정상 실행 후 롤백 | PASS |

## 2026-11-22 (19) Agent v4 배포 전 검증 (`local_data/raw_results/real_limo_integ_claude_20261122/v4_deploy_prep_01/`)
| 항목 | 결과 |
|---|---|
| 로봇 v3 상태(읽기 전용), 프로세스·listener·모터 포트 | 정상, 변경 없음 |
| v3 vs v4 | `agent_node.py` 1개 차이; v4 tar = 작업트리 = G1 커밋 |
| v4 tar 풀어 종료 matrix | 12/12 PASS |
| 아티팩트 생명주기 (연결·Lease·ACK·거부·종료·재시작) | PASS ×2 |
| 회귀 17개 모듈 | OK |
| 배포·원복 리허설(스크래치) + 중단 시험 | PASS |
| 로봇 교체·기동 | 미실시 (승인 대기) |

## 2026-11-22 (20) Agent v4 실차 격리 검증 (`local_data/raw_results/real_limo_integ_claude_20261122/v4_isolated_01/`)
| 항목 | 결과 |
|---|---|
| ONLINE/robot_id/boot_id/heartbeat, Lease 획득·반납·만료, ACK | PASS |
| Mapping START·Goal·Manual·Gate Reset 거부 (run1, run2) | PASS 12/12 ×2 |
| 서버 재시작, 터널 단절, Agent 재시작(새 boot id) | PASS ×3 |
| 로봇에서 SIGINT 종료 / SIGTERM 종료 | 0.164 s rc 0 / 0.162 s rc 0 |
| 잔여 프로세스, `/cmd_vel*` 토픽, 모터 포트 사용자 | 0 / 0 / 0 |
| 중단 조건 | 발생 없음, 원복 불필요 |

## 2026-11-22 (21) v4 실차 검증 재판정 (읽기 전용; `r1_prep_01/`, 문서 [V4_VERDICT_AND_R1_PREP_20261122.md](fleet_control/docs/V4_VERDICT_AND_R1_PREP_20261122.md))
| 항목 | 판정 |
|---|---|
| 실제 검사 항목 16개 (이전 "PASS 12개"는 한 probe 안의 거부 확인 수였음) | 16/16 PASS, FAIL 없음 |
| SIGINT 0.164 s / SIGTERM 0.162 s, rc 0, ROS 노드·토픽·모터 포트 잔여 0 | PASS |
| 시작~ONLINE: 스크립트 기동 기준 ≈47 s (DDS 슬롯 사전검사 ≈43 s + 대기 4 s), Agent 프로세스 시작 기준 ≈1 s | Agent 기준 PASS; 사전 정의된 "30 s" 조건은 **미결정**(기점 미지정, 타이머 미적용). 정정안: T_agent ≤ 30 s, T_launch ≤ 90 s(사전검사 포함) |
| shm 잔재 2개 귀속 | 정정: v3 SIGKILL 아님, 20:10:46 봉인 내 `ros2` 데몬 시작 시각과 일치(추정, 소유자 미확인). 데몬이 DDS 슬롯 2 점유 → 찮까끈한 슬롯 지도가 아님 |
| 실행 스택과 함께, 오래된 shm 조각이 있는 상태에서의 v4 동작 | **SKIP(미검증)** |
| "warm-up 31 s 대기" 표현 | `pgrep -f` 자기매칭 아티팩트; 유효 근거는 22:04:35 앵커 확인(0개) |
| 추가 산출물 (레포지토리, 미커밋) | 문서 1개, `AGENT_V4_DEPLOY_PREP` §8 정정 1줄 |
| 레거시 부수 발견: `motion_watchdog` SIGTERM/무시된 SIGINT에서 종료 0.16 s이지만 rc 1 + ExternalShutdownException (노트북 격리 도메인 시험) | R2 대상 (3줄 수정 제안, 미적용) |

## 2026-11-22 (22) R1.1 계약 테스트·P6 노트북 적용 (로봇 무변경; 문서 [R1_1_P6_AND_R1_2_PROPOSAL](fleet_control/docs/R1_1_P6_AND_R1_2_PROPOSAL_20261122.md), 기록 `r1_1_01/`)
| 항목 | 결과 |
|---|---|
| 시작 스크립트 P6(STATE_DIR 필수·절대경로): `bash -n`, 인자 없음/1개/잘못된 URL/STATE_DIR 없음·빈값·상대경로 (Agent 미기동) | 전부 rc 2 PASS; 정상 경로는 미실행(SKIP) |
| `tests/test_agent_server_contract_20261123.py` | 11/11 PASS (R1.2 적용 전·후 스크래치 둘 다) |
| 기존 계약·안전 범위 테스트 | PASS (계약 5종, safety-context 4종); `test_locked_graph_contract_characterization` = SKIP(노트북에 ROS 없음) |
| `tests/` 전체 discover 1회 | 784 실행, 4 실패, 65 오류(ROS import 등), 17 skip; `test_recovery` 는 단독 PASS이나 discover에서 실패 → 원인 미조사, 기준선 비교 없음(기존 실패로 단정 불가) |
| 계약 드리프트 | 버전 불일치 없음; 리터럴 중복, `docs/API.md`에 라우트 3개(`map-bundle`, `slam-map`, `manual-bench`) 누락, gate-reset은 safety_contract_version 미검사 |
| R1.2 패치 | 스크래치 시험 PASS, **미적용** (승인 대기) |

## 2026-11-23 (23) R1.4.1 P1 수정 2건 + P2 (격리/Mock/격리 ROS, 실차 없음; [R1_4_1_P1_FIXES](fleet_control/docs/R1_4_1_P1_FIXES_20261123.md), 기록 `r1_4_1_01/`)
| 항목 | 결과 |
|---|---|
| P1-1 Agent: hello protocol_version 저장·boot 결합, 누락/null/bool/float/문자열/구버전 거부, 활성 goal 취소 | `test_r1_4_1_agent_protocol` 11/11 PASS (수정 전 16 FAIL + 5 ERROR); Codex probe 입력 7/7 차단 |
| P1-2 Mapping STOP: 비호환 Agent의 자기 세션 STOP만 전달·수락·결과 | `test_r1_4_1_mapping_stop` 10/10 PASS (수정 전 7 ERROR + 1 FAIL); 물리 정지는 **미검증** |
| P2 `test_agent_safety_v1` 스텁 격리 | 17/17 (ROS 없음·ROS 소스 둘 다); 잠재 스텁 잔존 문제는 ROS 프로세스에서만 정리; 한 프로세스 내 타 스텁 공유 스위트와의 동시 실행은 ROS 없는 셸에서만 (부분 해결) |
| 정상 Agent 탐색 유지 (격리 ROS: Agent+mock Nav2 action+Fleet) | `test_ros2_isolated_action`, `test_ros2_late_goal_reconciliation` OK (private netns, domain 187) |
| 전체 discover (ROS 없음) | 869 실행 668 OK, 2 FAIL, 34 ERROR, 14 SKIP; 실제 회귀 0; 오류 분류는 문서 §8 (ROS 부재, netns 가드, 환경 변수, 타이밍 flake) |
| 한계 | R4 실차 통합 자동 승인 아님; Agent v5 배포·실차 검증 미실시 |

## 2026-11-23 (24) R1.4.2 신규 계약 결함 2건 ([R1_4_2 문서](fleet_control/docs/R1_4_2_CONTRACT_FOLLOWUP_20261123.md), 기록 `r1_4_2_01/`; 격리/Mock/격리 ROS, 실차 없음)
| 항목 | 결과 |
|---|---|
| A: 후속 상태 응답의 safety_contract_version 누락이 이전 승인 유지 | 수정 후 신규 goal 0, 실행 goal 취소 요청 1(스텁 핸들, 실제 정지 미검증) |
| A의 부수: 상태 응답의 protocol_version이 무효 hello를 복구 | 차단(상태응답은 철회만 가능) |
| B: 비호환 STOP poll의 session_id 미검사 | 수정 후 다른/없는 session이면 command null; 정상 round trip 유지 |
| `test_r1_4_2_contract_followup` | 40/40 PASS (수정 전 6 FAIL) |
| 회귀(R1.4/R1.4.1/계약/E2E 16/Recovery 28/Watchdog admission/Mapping/격리 ROS 2+safety_v1) | PASS |
| 전체 discover(ROS 없음) | 909 실행, 2 FAIL, 33 ERROR, 17 SKIP; 실패 집합 불변, 회귀 0 |
| 미해결 | P2 stub 캐시, Watchdog 타이밍 flake, Agent v5 배포·실차 검증, R4 차단 조건 |

## 2026-11-23 (25) R1.4.3 계약 세대·수신 시점 승인·취소 의무 ([R1_4_3 문서](fleet_control/docs/R1_4_3_CONTRACT_GENERATION_20261123.md), 기록 `r1_4_3_01/`; 격리/Mock/격리 ROS, 실차 없음)
| 항목 | 결과 |
|---|---|
| Codex interleave 8개 순서 | 비활성 4개 제출 0, 활성 4개 취소 1 (수정 전 제출 1 / 취소 0 포함) |
| `test_r1_4_3_contract_generation` (결정적 경합 시나리오 1~7 + 단위) | 15/15 PASS (수정 전 7 FAIL + 2 ERROR) |
| 회귀: R1.4/R1.4.1/R1.4.2/계약/E2E 16/Recovery/Watchdog admission/Mapping/Gate Reset/dedupe/격리 ROS 2 | PASS |
| 전체 discover(ROS 없음) | 924 실행, 2 FAIL, 33 ERROR, 17 SKIP — 실패 집합 R1.4.2와 동일. 중간에 픽스처 유래 회귀 3건 발견·수정(dedupe, phase20_fail_closed, phase20_agent_sigkill: 계약 없는 가짜 서버) |
| 미검증 | 실제 Nav2 취소 완료·물리 정지, 실제 ROS/DDS, 응답 순서 역전, Agent v5 배포, R4 차단 조건 |

## 2026-11-23 (26) R1.4.4 제출 원자성·재조정 철회 ([R1_4_4 문서](fleet_control/docs/R1_4_4_SUBMISSION_FENCE_20261123.md), 기록 `r1_4_4_01/`; 격리/Mock/격리 ROS, 실차 없음)
| 항목 | 결과 |
|---|---|
| Stage 3 검사–send_goal_async 사이 무효화 | 불가(실행 루프가 fence에서 대기함을 관측) → 제출 후 선형화, 취소 의무 유지 |
| reconciliation_required False→True→False | 세대 증가, 대기 명령 제출 0, 활성 goal 취소 1 (수정 전 제출 1/취소 0) |
| `test_r1_4_4_submission_fence` A~J | 12/12 PASS ×3 (수정 전 4 FAIL + 8 ERROR); J는 교착 없음(타임아웃 감지기) |
| 회귀(R1.4~R1.4.3, 계약, E2E, Recovery, Watchdog admission, SIGKILL, fail-closed, dedupe, 격리 ROS 2) | PASS |
| 전체 discover(ROS 없음) | 936 실행, 2 FAIL, 33 ERROR, 17 SKIP — 실패 집합 동일, 신규 회귀 0 |
| 미검증 | 실제 Nav2 취소·물리 정지, 실제 DDS 타이밍, 실제 Fleet 재조정 순서, 보조 실행 주체의 세대 결합, v5 배포, R4 차단 조건 |

## 2026-11-23 (27) R1.4.5 권한 철회 경로 통일·재조정 플래그 ([R1_4_5 문서](fleet_control/docs/R1_4_5_WITHDRAWAL_PATHS_20261123.md), 기록 `r1_4_5_01/`; 격리/Mock/격리 ROS, 실차 없음)
| 항목 | 결과 |
|---|---|
| HTTP 503·통신 예외의 권한 철회 | fence 통과, 세대 +1; Stage 3 중 오류는 제출 뒤로 직렬화(전송 시점 connected·lease 유지 관측), 취소 의무 유지 |
| reconciliation_required 잘못된 값 7종+0.0/'' | 신규 goal 0 (수정 전 각 1); 서버 정상 상태 응답에 명시 False 추가 |
| `test_r1_4_5_withdrawal_paths` A~J | 13/13 PASS ×3 (수정 전 23 실패, 하위 포함) |
| 회귀(R1.4~R1.4.4, Mapping STOP, E2E, Recovery, Watchdog admission, SIGKILL, dedupe, 격리 ROS 3개 포함 shutdown) | PASS |
| 전체 discover(ROS 없음) | 949 실행, 2 FAIL, 33 ERROR, 17 SKIP — 실패 집합 동일, 신규 회귀 0; shutdown SETUP_FAIL은 ROS netns에서 PASS(환경) |
| 동작 변화 | HTTP 오류 시 활성 goal 즉시 취소(기존 3 s 유예 미적용) |
| 미검증 | 실제 Nav2/DDS·물리 정지, 보조 실행 주체 세대 결합, 응답 순서 역전, 서버→v5 배포, R4 차단 조건 |

## 2026-11-23 (28) R1.4.6 Lease 누락 차단·필수 승인 필드 매트릭스 ([R1_4_6 문서](fleet_control/docs/R1_4_6_REQUIRED_FIELDS_20261123.md), 기록 `r1_4_6_01/`; 격리/Mock/격리 ROS, 실차 없음)
| 항목 | 결과 |
|---|---|
| 상태 응답 lease 누락 | 대기 goal 제출 0 (수정 전 1), 활성 goal 취소 요청 1 (수정 전 0) |
| 필수 필드 5종 41개 값 매트릭스 | 제출 0·취소 유지·수신 거부; 수정 전 실패 57건은 모두 lease (다른 필드는 이미 통과) |
| `test_r1_4_6_required_fields` | 14/14 PASS ×3 |
| 회귀(R1.4~R1.4.5, E2E, Recovery, Watchdog admission, SIGKILL, dedupe, 격리 ROS 3, 실제 HTTP wire) | PASS |
| 전체 discover(ROS 없음) | 974 실행, 2 FAIL, 33 ERROR, 22 SKIP(Neon R2 시험 5개는 ROS 필요로 skip) — 실패 집합 동일, 신규 회귀 0 |
| 미검증 | 실제 Nav2/DDS·물리 정지, 보조 실행 주체, 응답 순서, v5 배포, R4 차단 조건 |

## 2026-11-23 (29) R1.4.6·R2·R3 Codex 독립 검증 결과 (오프라인; 실차·18190·모터 미사용; Claude 문서 통합)
검증 환경: 노트북, 저장소 작업트리(HEAD `c804241a` + 미추적 `fleet_control/`)의 스냅샷을 임시 디렉터리에 풀어 실행. HTTP는 임시 루프백 포트, ROS는 Humble을 source한 `unshare -Urn`(lo만) 안에서 실행. Codex는 감사 전후 작업트리 상태·해시가 같음을 기록(`git-before` = `git-after`).

**R1.4.6 / R1 종료** — 기록 `local_data/raw_results/codex_r146_close_20261123_6ZGjQf/` (`REPRO.txt`, `run.sh`, `SHA256SUMS`, 297 파일 스냅샷)
| 항목 | 결과 |
|---|---|
| Lease 누락(Codex `independent.py`, 수정 전 → 후) | 대기 goal 제출 1 → 0, 활성 goal 취소 요청 0 → 1; 명시적 null은 변화 없음 |
| 필수 필드 매트릭스 | 실제 `STATE_MATRIX` 39개 값(lease 18, safety version 6, reconcile 6, protocol 5, server boot 4); 독립 프로브 대기·활성 78 경로 + owner/epoch 일치 정상 2건 |
| `test_r1_4_6_required_fields` | 14 PASS (수정 전 소스: 하위 시험 57 실패) |
| 계약·Mapping STOP 회귀 | 162 PASS, 7 SKIP |
| Dedupe | 14 PASS |
| Fleet·Recovery·Admission·SIGKILL(루프백) | 36 PASS |
| 격리 ROS(실제 rclpy: mock action, late goal, shutdown) | 3 PASS (ROS_DOMAIN_ID 218) |
| Gate Reset worker (ROS source) | 11 PASS |
| 실제 HTTP `wire.py current` | PASS |
| **전체 discover** | **PASS 아님.** 이번 감사에서 재실행하지 않고 기존 원본 `real_limo_integ_claude_20261122/r1_4_6_01/discover_current_plain.txt`만 재집계: 974 중 920 PASS / 2 FAIL / 33 ERROR / 19 SKIP (하위 SKIP 포함 요약 22). ERROR 중 28은 ROS 없는 셸의 import 실패. QR 스냅샷 환경 실패는 재실행 안 함 |
| 판정 | R1.4.6 오프라인 PASS, R1 오프라인 안전 계약 종료 가능 |

**R2 Watchdog 종료** — 기록 `local_data/raw_results/codex_neon_r2_r3_close_20261123_6v3X5W/` (이전 감사 `codex_neon_r2_r3_audit_20261123_PBSflg/`, `codex_neon_r2_r3_reaudit_20261123_iysb8l/` 보존)
| 항목 | 결과 |
|---|---|
| 격리 ROS 2 Humble 스위트(`test_r2_watchdog_shutdown_20261123` + 기존 watchdog 시험 5종) | 66/66 PASS, 0 FAIL/ERROR/SKIP (`r2-suite.log`) |
| 종료 후 비영속도 재발행 반례 | 해결: 발행은 `_publish_lock` 아래의 `_publish_zero`·`request_shutdown`·`_publish_authorized`에만 있음, 신호 handler는 정지 요청만 기록, 진행 중 발행은 종료 확정 전에 끝나고 marker 이후 입력은 차단 (`r2-edge.log`, `r2-lock-check.log`) |
| 정상·중복·비정상 종료 구분 | 유지 (중복 SIGTERM에서 -15 종료 없음, SIGINT 정상/비정상 구분) |
| 미검증 | 응답 지연, DDS 전달 이후 물리 정지, 시스템 전체 데드락 부재, 실제 로봇 정지·제동거리 |

**R3 배포 도구** — 같은 기록 디렉터리
| 항목 | 결과 |
|---|---|
| `test_r3_deploy_tool_20261123` | 11/11 PASS, 0 FAIL/ERROR/SKIP (`r3-suite.log`; 정상 rollback은 실제 Agent 패키지·colcon rebuild 사용) |
| 단일 파일 원복·Dry-run 무변경·체크섬·경로(symlink) 안전 | PASS |
| 패키지 교체·원복 실패 보고 | 정확: 이중 실패 rc 3, SRC 없음, FAILED/OLD 디렉터리와 backup tar 보존, 성공 선언 없음; backup tar 추출 해시 = baseline (`r3-extended.log`, `r3-recovery-check.py`) |
| 한계(현장 차단 조건) | 두 번의 디렉터리 이동은 원자적이지 않음; 이중 실패 시 `WS/src/PKG` 부재 가능, 수동 복구 필요 |
| 판정 | R3 오프라인 PASS(진단·복구 범위); 실차 배포 SKIP(별도 승인) |

정정: (28)의 "필수 필드 5종 41개 값"은 실제 `STATE_MATRIX` 기준 **39개**이다(수정 전 57 실패·14 PASS 결과는 같음).

## 2026-11-23 (30) Agent v5 candidate 생성·검증 (오프라인; 실차·18190 없음; [문서](fleet_control/docs/AGENT_V5_CANDIDATE_20261123.md), 기록 `real_limo_integ_claude_20261122/{deploy_candidate_agent_v5,v5_candidate_01}/`)
| 항목 | 결과 |
|---|---|
| artifact 생성(R3 도구, `--status candidate`) | 32 파일, tar `33a9e55d…`, FILES `be62c38a…` |
| 재현성(두 번 생성) | tar·FILES.sha256 바이트 동일 |
| TAR.sha256 / FILES.sha256 / 추출본=소스 / py_compile | 모두 OK |
| v4 대비 | 5 파일만 다름(R1 3 + R2 2) |
| 서버 응답 계약 프로브(메모리, 소켓 없음) | hello·상태 응답 필수 필드 모두 존재, `reconciliation_required` 명시 false, lease 키 항상 존재 |
| 미실시 | colcon 빌드·설치, 로봇 배포, R3·Agent 시험 재실행(소스 무변경), package.xml 의존성 선언 보완(`geometry_msgs`·`std_srvs`·`rcl_interfaces`·yaml 누락, v4부터 있던 것) |

## 2026-11-23 (31) Agent v5b 격리 빌드 (오프라인, ROS Humble 임시 워크스페이스; 실차 없음; 기록 `real_limo_integ_claude_20261122/{deploy_candidate_agent_v5b,v5b_build_01}/`)
| 항목 | 결과 |
|---|---|
| 파일 수 일치(tar / FILES.sha256 / 소스) | 32 / 32 / 32 |
| 재현성(두 번 생성) | tar·FILES 바이트 동일, tar `c71b166f…` |
| 추출 후 FILES.sha256 | 32 OK |
| `colcon build --packages-select limo_robot_agent` | PASS (경고: byte-compiling 비활성만) |
| 설치 진입점 / 모듈 import | `robot_agent`·`mock_robot`·`motion_watchdog` / 실패 0 |
| 미실시 | rosdep install(미초기화), 로봇 ROS 환경 확인, Agent 실행·시험 재실행, 배포 |
