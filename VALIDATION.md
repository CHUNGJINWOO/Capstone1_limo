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
| `final4` | 2026-11-13 | Gazebo + YOLO | `419e798` + 로컬 `Capstone1_limo_integrated_sim` | `fleet_control/integrated_sim/RESULTS.md` (`29818e8` / `ed53d81` / PR #9 `b350efa`, 12/13 PASS; 풀스택 재실행 미검증) | `$HOME/.local/share/robot-sim-projects/limo_integrated_sim/runs/` | 업로드 예정 (`02_GAZEBO_SIM/...`) | 미등록 |

---

## 4. PR #9 · PR #10 병합 기준 검증 및 미검증 구분 (`origin/main` `a8132c7`)

| 구분 | 대상 | 결과 | 실행 환경 / 한계 |
|---|---|---|---|
| **PASS** | `fleet_control/tests/run_checks.py` | **46/46 PASS** | PR #9(`dab07d1`) `git archive` 격리 + 호스트 ROS 2 Humble mock Nav2 |
| **PASS** | `fleet_control` 단위·JS 시험 | `test_ownership` 37/37, `test_recovery` 14/14, `test_dedupe` 14/14, `test_agent_recovery` 2/2, `test_sim_motor_policy` 12/12, `test_auth_reload` 2/2, `test_cancel_results` 14/14, `test_runner_policy` 33/33, `test_visualization` 31/31, JS 3종 PASS | `git archive` 격리 검증 (`test_cancel_results`는 호스트 `rclpy`) |
| **PASS** | `run_recorder.py` 경로 정책 (PR #10) | 합성 시험 **16/16 PASS** | `f63688e` 격리 검증 (신규 clone `local_data/` 미존재, `--dry-run`, 저장소 밖 fallback 포함) |
| **이전 기준선** | `run_gazebo_checks.py` | 이전 **69/69 PASS** | PR #9 격리 검증에서는 Ignition Gazebo / `$HOME/rmf_isolated` 미비로 미재실행 |
| **부분 검증** | `run_rmf_checks.py` | **47/51** | §7 타이밍 레이스 기존 한계, PR #9 격리 검증 미재실행 |
| **미검증** | `dashboard_lab/tests/test_lab.py` | **미검증** | `aiohttp` 의존성 부재 |
| **미검증** | `integrated_sim` 풀스택, GPU Video AI 실환경, LIMO 실차 안전·주행 전체, 실제 `rosbag2` 기록 | **미검증** | 실환경/실차 감독 시험 및 별도 GPU 런타임 필요 |
