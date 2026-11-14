# WORKTREES.md — Git 워크트리 · 외부 격리 실행 환경 · 포트/도메인 할당표

> **기준일**: 2026-11-14  
> 모든 Git 워크트리의 `.git` 파일과 `Capstone1_limo/.git/worktrees/*/gitdir`는 **절대 경로**를 사용합니다.  
> 일반 `mv` 명령으로 폴더를 옮기면 Git 연결과 CMake 빌드 경로가 즉시 깨지므로 임의 이동·삭제를 금지합니다.

---

## 1. Git 워크트리 및 주요 브랜치 현황 (`git worktree list --porcelain`)

> **경로 표기 규칙**: 공개 저장소 정책([docs/PUBLICATION_POLICY.md](docs/PUBLICATION_POLICY.md))에 따라 로컬 사용자 홈은 `$HOME`, 워크트리 상위 디렉터리는 `<WORKSPACE_PARENT>`로 표기합니다.

| 구분 | 로컬 워크트리 경로 (`<WORKSPACE_PARENT>` 기준) | 브랜치 | HEAD | 소유/역할 | 미커밋·미추적 상태 | 정리 방침 |
|---|---|---|---|---|---|---|
| **메인 (보존 1)** | `<WORKSPACE_PARENT>/Capstone1_limo` | `staging/fleet-control-on-progress-c804241` | `c804241` | 공통 / Claude Code: 메인 저장소, `capstone_ws`, **통합 `fleet_control/`(단일 기준)**, `experiments/*20261113*`, `local_data/` | `M AGENTS.md`, `M README.md`, 다수 미추적(`fleet_control/`, `experiments/*`, `notebook/`, 공통 문서 8종) | **보존 필수 (절대 이동·초기화 금지)** |
| **기능 1~4 (해제 완료)** | *(워크트리 해제됨)* | `codex/dashboard-visualization` (`bc3df5e`), `codex/rmf-gazebo-integration` (`0dc7ef9`), `exp/video-ai-preverify` (`e4dd598`), `exp/integrated-sim-20261113` (`29818e8`) | 각 스냅샷 커밋 | `fleet_control` 통합 완료 후 각 브랜치에 스냅샷 커밋 보존 및 `git worktree remove` 완료 | Clean (스냅샷 diff/status는 `local_data/worktree_backups_20261114/`에도 보존) | **해제 완료** (기준 소스는 `Capstone1_limo/fleet_control/` 및 `feat/fleet-control-unified` `ed53d81`) |
| **정리 1~4 (해제 완료)** | *(워크트리 해제됨)* | `cleanup/01~04` | PR #1~#4 | 2026-11-11 PR #1~#4 분할 병합용 워크트리 (`main` 100% 병합 완료) | Clean | **해제 완료** |
| **유실 1 (해제 완료)** | *(워크트리 해제 완료)* | `integrate/privacy-preserved-main` | `b1d14de` | 과거 임시 워크트리 정리 완료 | Clean | **해제 완료** |

### 워크트리가 연결되지 않은 주요 브랜치
- `main` / `origin/main` (`a8132c7`): PR #1~#4, PR #8(공통 문서), **PR #9(`b350efa`, `fleet_control/` 173개)**, **PR #10(`a8132c7`, `run_recorder.py` 경로 개선)**이 병합된 정식 공개 기준선.
- `feat/fleet-control-pr-b` (`e0bc7be`): PR #9 병합 브랜치 (`fleet_control/` 173개 + 증거 해시 안내 보완).
- `feat/run-recorder-pr-c` (`f63688e`): PR #10 병합 브랜치 (`run_recorder.py` 신규 clone 출력 경로 보완).
- `feat/fleet-control-unified` (`ed53d81`): `51175ef` + `419e798` 기반 로컬 통합 보존본(203개 비런타임 파일 + 비필수 증빙 30개).
- `docs/backup-policy-and-status-20261114` (`a021186`): `origin/main`(`51175ef`) 기반 공통 문서 증분 5종 분리 커밋.
- `feat/fleet-control-rmf-integration` (`origin/feat/fleet-control-rmf-integration`, `419e798`): `main` 위에서 `fleet_control/`을 추가한 커밋 (기능 브랜치 4종의 공통 베이스).
- `docs/gazebo-auto-mapping-final-20261112` (`cc58183`): Gazebo 차동구동 자동 맵핑 6회 결과 및 자동 지도 저장기(`experiments/auto_mapping_final_20261112/`).
- `docs/scan-normalizer-locked-test-20261112` (`09d742c`): 실차 스캔 정규화 배포 및 모터 잠금 검증 노트.
- `docs/limo-real-robot-progress-20261111` (`c804241`): 2026-11-11 정리 이전 전체 실험 스냅샷 보존 브랜치 (`PRESERVED_SNAPSHOTS.tsv` 참조).

---

## 2. 워크스페이스 외부(`$HOME` / `/tmp`) 격리 실행 환경 및 런타임 데이터

아래 경로들은 프로젝트 스크립트와 `env.sh`가 절대 경로로 참조하는 외부 실행 환경입니다. **물리적 이동이나 임의 삭제를 금지합니다.**

| 경로 | 용량 | 역할 | 참조하는 스크립트 / 환경변수 |
|---|---|---|---|
| `$HOME/rmf_isolated` | 74 MB | Open-RMF 2.1.8 `.deb` 추출 격리 환경 (`env.sh`, `prefix/`) | `RMF_ISOLATED_ENV` / `RMF_ISOLATED_DIR`; `run_rmf_stack.sh`, `rmf_env.py`, `integrated_env.py`, `lf_t2_rmf.sh` |
| `$HOME/rmf_demos_humble` | 604 MB | `rmf_demos` Fortress Office 월드·모델·플러그인(`env.sh`, `ws/`, `prefix/`, `runs/`) | `experiments/office_limo_sim_20261113/office_limo.launch.py`, `safety_chain_fix_20261113/tools/run_office.sh` |
| `$HOME/.local/share/robot-sim-projects/yolo_gazebo_limo` | 6.0 GB | CUDA YOLO11n 가상환경(`venv/`), 가중치(`weights/yolo11n.pt`), Chair/Person 메시 | `YOLO_HOME` / `YOLO_EXP`; `integrated_env.py`, `lf_setup.sh` |
| `$HOME/.local/share/robot-sim-projects/limo_video_ai_preverify` | 26 MB | 실차 영상·AI 연동 키트(`real_limo_kit`) 및 토큰(`secrets/*.token`) | `VIDEO_AI_HOME` / `LIMO_VIDEO_KIT`; `integrated_env.py`, `lf_env.sh`, `lf_setup.sh` |
| `$HOME/.local/share/robot-sim-projects/limo_integrated_sim` | 46 MB | 통합 시뮬레이션 실행 디렉터리(`runs/<timestamp>/logs,results,procs.json`) | `INT_RUN_ROOT`; `integrated_env.py` |
| `$HOME/.local/share/robot-sim-projects/limo_field_20261113` | 44 MB | 실차 현장 시험 런타임 DB(`data/`), PID/로그(`run/`), 결과(`results/`) | `LIMO_FIELD_STATE`; `experiments/real_limo_field_20261113/laptop/lf_env.sh` |
| `$HOME/.local/state/capstone-dashboard-visualization` | 47 MB | `dashboard` 런타임 상태, `real_connection.json`(0600), `real_trials/` | `Capstone1_limo_dashboard/fleet_control/dashboard/real_trial.py` |
| `$HOME/.local/state/capstone-dashboard-lab` | 21 MB | `dashboard_lab` 런타임 상태 및 세션 기록 | `Capstone1_limo_dashboard/fleet_control/dashboard_lab/server.py` |
| `$HOME/.config/limo_field_20261113` | 16 KB | 실차 현장 시험 로컬 설정(`kit.env`, 0600) 및 `secrets/rmf_adapter.token` | `LIMO_FIELD_CONF`; `lf_env.sh`, `lf_setup.sh` (**절대 Git 커밋/업로드 금지**) |
| `/tmp/capstone-rmf-web-official-20261113` | 휘발성 | 공식 `open-rmf/rmf-web` 프론트엔드 Storybook 연구 소스 | `RMF_WEB_RESEARCH_ROOT`; 재부팅 시 소멸되면 `prepare_frontend.sh`로 재생성 |

---

## 3. 포트 및 `ROS_DOMAIN_ID` 할당표 (충돌 및 강제 종료 방지)

다른 에이전트나 사용자의 실행 환경을 보호하기 위해 아래 포트와 ROS 도메인 규칙을 준수합니다.

| 환경 / 용도 | HTTP / WS / TCP 포트 | `ROS_DOMAIN_ID` | 보호 규칙 |
|---|---|---|---|
| 운영 중앙 관제 서버 (Production Reserve) | `8080` | - | **보호 포트**: 시험 스크립트가 `8080`에 바인딩하거나 종료하는 것을 엄격히 금지 (`FORBIDDEN_PORTS`) |
| 실차 현장 시험 (`real_limo_field_20261113`) | `18190` (Dashboard/MM), `18191` (AI TCP Link), `18192` (AI HTTP) | `77` (LIMO 로컬), `151` (노트북 RMF 로컬; `kit.env` 기본값은 `77`이나 양측 모두 `ROS_LOCALHOST_ONLY=1`로 DDS 비공유) | **보호 포트**: 현장 시험 서버 및 AI 호스트 포트 |
| 기본 관제 단위/통합 시험 (`run_checks.py`, `test_ownership.py`) | `18080`, `18083` | `141` | 테스트 전용 임시 포트 |
| 모의 RMF E2E (`run_rmf_checks.py`) | `18090` (기본) / `18140~18141` (회귀) | `151` (RMF), `152` (Robot) | `ROS_LOCALHOST_ONLY=1` 필수 |
| Gazebo × RMF E2E (`run_gazebo_checks.py`) | `18120` (HTTP), `18121` (WS) | `162` (RMF), `163` (Robot) | `IGN_PARTITION=capstone_rmf_<pid>` 격리 |
| Office Gazebo 단독/안전 체인 (`office_limo_sim_20261113`, `safety_chain_fix`) | - | `162` | `ROS_LOCALHOST_ONLY=1` |
| 통합 시뮬레이션 (`integrated_sim`) | `18200` (MM), `18201` (WS), `18210` (AI TCP), `18211` (AI HTTP), `18212` (Proxy) | `181` (Robot), `182` (RMF) | `18210/18211`은 `dashboard`의 live/test gateway와도 겹칠 수 있으므로 실행 전 리스너 확인 필수 |
| Dashboard Lab (`dashboard_lab`) | `18310` (개선 UI), `18311` (Baseline 비교), `18313` (Native 계측) | - | `8080, 18190, 18210, 18211` 바인딩 코드 수준 거부 |
| RMF Web Storybook (`rmf_web_research`) | `18410` | - | 프론트엔드 오프라인 스토리북 전용 |
