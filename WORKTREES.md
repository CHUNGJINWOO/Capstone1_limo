# WORKTREES.md — Git 워크트리 · 외부 격리 실행 환경 · 포트/도메인 할당표

> **기준일**: 2026-11-14  
> 모든 Git 워크트리의 `.git` 파일과 `Capstone1_limo/.git/worktrees/*/gitdir`는 **절대 경로**를 사용합니다.  
> 일반 `mv` 명령으로 폴더를 옮기면 Git 연결과 CMake 빌드 경로가 즉시 깨지므로 임의 이동·삭제를 금지합니다.

---

## 1. Git 워크트리 및 주요 브랜치 현황 (`git worktree list --porcelain`)

> **경로 표기 규칙**: 공개 저장소 정책([docs/PUBLICATION_POLICY.md](docs/PUBLICATION_POLICY.md))에 따라 로컬 사용자 홈은 `$HOME`, 워크트리 상위 디렉터리는 `<WORKSPACE_PARENT>`로 표기합니다.

| 구분 | 로컬 워크트리 경로 (`<WORKSPACE_PARENT>` 기준) | 브랜치 | HEAD | 소유/역할 | 미커밋·미추적 상태 | 정리 방침 |
|---|---|---|---|---|---|---|
| **메인 (보존 1)** | `<WORKSPACE_PARENT>/Capstone1_limo` | `staging/fleet-control-on-progress-c804241` | `c804241` | 공통 / Claude Code: 메인 저장소, `capstone_ws`, **통합 `fleet_control/`(단일 기준)**, `experiments/*20261113*`, `local_data/` | `M AGENTS.md`, `M README.md`, `M capstone_ws/src/limo_capstone/scripts/run_recorder.py`, `M docs/design/AUTONOMOUS_DRIVING_DESIGN.md`(출력 경로 통일, 미커밋), 다수 미추적(`fleet_control/`, `experiments/*`, `notebook/`, 공통 문서 8종) | **보존 필수 (절대 이동·초기화 금지)** |
| **기능 1~4 (해제 완료)** | *(워크트리 해제됨)* | `codex/dashboard-visualization` (`bc3df5e`), `codex/rmf-gazebo-integration` (`0dc7ef9`), `exp/video-ai-preverify` (`e4dd598`), `exp/integrated-sim-20261113` (`29818e8`) | 각 스냅샷 커밋 | `fleet_control` 통합 완료 후 각 브랜치에 스냅샷 커밋 보존 및 `git worktree remove` 완료 | Clean (스냅샷 diff/status는 `local_data/worktree_backups_20261114/`에도 보존) | **해제 완료** (기준 소스는 `Capstone1_limo/fleet_control/` 및 `feat/fleet-control-unified` `ed53d81`) |
| **정리 1~4 (해제 완료)** | *(워크트리 해제됨)* | `cleanup/01~04` | PR #1~#4 | 2026-11-11 PR #1~#4 분할 병합용 워크트리 (`main` 100% 병합 완료) | Clean | **해제 완료** |
| **유실 1 (해제 완료)** | *(워크트리 해제됨; 구 `$HOME/.gemini/antigravity/brain/<session_id>/scratch/integrate_worktree`)* | `integrate/privacy-preserved-main` | `b1d14de` | 과거 임시 워크트리. 정식 `git worktree remove`(`--force`·`prune` 미사용)로 해제 | 해제 전 `git status`: 추적 변경 0, 미추적 2개(`docs/EXTERNAL_REFERENCES.md`, `docs/third_party.repos`), 무시 44개(`capstone_ws/{build,install,log}`, `__pycache__`) | **해제 완료**. 브랜치 ref(`b1d14de`)와 `origin/main` 이후 고유 커밋 11개는 로컬에 보존. 해제 전 상태·해시·`EXTERNAL_REFERENCES.md` 차이는 `local_data/worktree_backups_20261114/integrate_worktree_release_20261114/`에 기록 (`third_party.repos`는 `origin/main`과 동일, `EXTERNAL_REFERENCES.md`는 `c804241` 작업트리 사본과 동일하며 `origin/main` 최신본(§7 라이선스 재확인 반영)과는 다름) |

### 워크트리가 연결되지 않은 주요 브랜치
- `main` (`2010e44`) / `origin/main` (`51175ef`): PR #1~#4 및 PR #8(공통 문서)이 병합된 정식 공개 기준선.
- `feat/fleet-control-unified` (`ed53d81`): `origin/main`(`51175ef`) + `419e798` 기반 `fleet_control/` 단일 통합 기준선(203개 비런타임 파일 + 경로 상대화 스크립트 3개).
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

---

## 4. 자료 구분 및 실행 결과 기본 출력 위치

| 구분 | 경로 | 취급 원칙 |
|---|---|---|
| **기준 소스** | `Capstone1_limo/capstone_ws/`, `Capstone1_limo/fleet_control/` (`feat/fleet-control-unified` `ed53d81`) | 새 최상위 폴더·기능별 복제 저장소·추가 워크트리를 만들지 않고 이 두 위치에서 개발 |
| **실험 원본·대용량 보존** | `Capstone1_limo/experiments/`, `Capstone1_limo/local_data/` (`raw_results/`, `rosbags/`, `worktree_backups_20261114/` 등) | Git 제외 보존. 원본 삭제·이동 금지 |
| **외부 런타임** | §2의 `$HOME/rmf_isolated`, `$HOME/rmf_demos_humble`, `$HOME/.local/share/robot-sim-projects/{yolo_gazebo_limo,limo_video_ai_preverify,limo_integrated_sim,limo_field_20261113}` 등 | 스크립트가 절대 경로로 참조하므로 현 위치 유지 |
| **독립 학습·과거 개발 기록** | `$HOME/slam_toolbox_source`(독립 ROS 2 학습, Capstone1 정리 범위 제외), `$HOME/.local/share/robot-sim-projects/{webots_limo_pro,coppeliasim_limo_pro}`(고유 시뮬레이터 실험 기록), `Capstone1_limo/notebook/`(고유 개발 노트, `COLCON_IGNORE`) | 삭제 금지, 현 위치 유지 |

**`run_recorder.py` 기본 출력 위치**: `--root` > `$LIMO_RUNS_DIR` > `<저장소>/local_data/raw_results/limo_runs/`(저장소의 `.git`과 `local_data/`, `capstone_ws/src/limo_capstone/`가 모두 확인될 때만) > `~/limo_runs/`(저장소를 찾지 못한 실차 온보드 단독 워크스페이스 등). 초기 스모크 기록 `20261110_003944_smoke`는 `local_data/raw_results/limo_runs/`로 이관·검증을 마쳤고 홈의 원본 `~/limo_runs`는 제거했습니다. 이 스모크 rosbag은 `messages_count=0`인 기록 체계 시험이며 센서 기록이나 주행 검증이 아닙니다.


## 2026-11-15 연결 상태 재확인

`git worktree list --porcelain`은 현재 `Capstone1_limo` 한 개, 브랜치 `staging/fleet-control-on-progress-c804241`, HEAD `c804241`을 표시한다. 위 표의 해제 기록은 과거 이력이다. 이 Git 목록만으로 다른 프로세스나 에이전트의 파일 소유·작업 종료를 증명할 수 없으므로 기존 미커밋·미추적 자료와 Gate 작업 경로를 계속 보호한다.


## 2026-11-16 Fleet UI 격리 미리보기 (Codex)

새 worktree/프로젝트 최상위 폴더 없이 기존 `fleet_control/`에서 작업했다. `tests/dashboard_preview.py`는 `/tmp/limo-dashboard-ui-*`에 새 DB/토큰/Registry를 만들고 loopback의 OS 자동할당 포트만 사용한다. 보호 포트/운영 데이터/ROS는 사용하지 않는다. 사용자 확인용 실행 정보는 `/tmp/limo-ui-preview-ready.json`, 이번 실행 URL은 `http://127.0.0.1:46069`이다(프로세스 종료 시 유효하지 않음). 앞선 이 작업 소유 시험 서버는 종료 요청했고 운영/타 세션 프로세스는 건드리지 않았다. [재실행 방법](fleet_control/docs/DASHBOARD_UI_20261116.md).


## 2026-11-17 승인 Workspace 운영 UI 적용

새 worktree 없이 기존 static UI에 적용. 이번 작업 소유의 runtime-test/final-test fixture 서버만 종료하고 사용자 확인용 loopback 미리보기 `http://127.0.0.1:44995`를 남겼다. 실행 정보 `/tmp/limo-workspace-approved-ready.json`, 임시 DB/토큰 `/tmp/limo-dashboard-ui-5jxdndbt`. 운영 포트/DB와 ROS 미접근. [재현](fleet_control/docs/WORKSPACE_RUNTIME_20261117.md).

## 2026-11-18 실제 LIMO Agent 선택 배포

실차의 기존 `~/limo_field_agent_ws`를 재사용하고 Agent 패키지만 갱신했다. 배포 전 source/install/build/log 및 field 실행 설정을 같은 workspace의 `predeploy_20261118_103822_6758.tar.gz`에 보존하고 내용·SHA-256을 확인했다. 워크트리 추가 없음, 맵·rosbag·토큰 변경 없음. 임시 Fleet Server 및 Agent 비구동 접속은 시험 후 종료했다. [검증·한계](VALIDATION.md).

## 2026-11-19 LIMO 전용 단일 워크스페이스(`robot_ws`) 구성

- LIMO 확정 경로: `/home/pcu/Capstone1_limo/robot_ws` (기존 `/home/pcu/capstone_ws` 미수정·미빌드·언더레이 배제, `/opt/ros/humble` 단독 기준 9개 패키지 빌드).
- 노트북 수집 경로: `local_data/raw_results/limo_runs/`(원본 rosbag2·이송 지도 보관, LIMO에 rosbag 미기록), `local-only/limo_robot_ws/env_laptop.sh`(Git 제외).
- DDS 연결: `ROS_DOMAIN_ID=77`, `ROS_LOCALHOST_ONLY=0`, FastDDS Discovery Server `<TAILNET_DISCOVERY_IP>:11811` (Tailscale UDP, 보호 포트 미사용). 워크트리 추가 없음.


## 2026-11-19 실차 재시험 산출물

`local_data/raw_results/limo_runs/20261119_auto_map_full/`은 이번 중단 실행의 결과 경로다. 기존 실험 기록과 별도로 보존하며 부분 지도·부분 관제 영상·노트북 rosbag을 포함한다.

## 2026-11-19 자동 맵핑 재실행 02

[안전 중단된 결과](local_data/raw_results/limo_runs/20261119_auto_map_full_02/)에 노트북 bag·부분 지도·관제 녹화·로그 보존. `wheel_odom_stale=true`, 감시기 종료 코드 2; 완성 지도 아님.

## 2026-11-20 18190 임시 운영 (백업 사본)

- 18190: 이 저장소 `fleet_control/scripts/run_field_server.sh` + `local_data/field_run_20261120_b1/data`(B1 사본 + C103). 현장 data(`limo_field_20261113/data`)와 `kit.env`는 미변경. 원래 런처의 `FLEET_SRC`(`Capstone1_limo_video_preverify`)는 11-14 worktree 해제로 부재. 새 worktree·최상위 폴더 없음.

## 2026-11-20 공개 기준선과 후속 작업 감사

감사 시 연결된 Git 작업트리는 staging 한 개(`c804241a`)였고, 조회한 공개 main은 `ea2d9632`였다. 양쪽 고유 커밋 수는 staging 3개, 공개 main 33개다. 로컬 `fleet_control/`은 staging에 미추적이고 공개 main에는 별도 추적 버전이 있다. 실차 Launch·Gate·Depth·Frontier의 기존 변경은 담당자 검토 대상으로 보존한다.

## G2 병합 (2026-11-23): origin/main `ea2d9632` 판의 이 문서에만 있던 줄
> 통합 브랜치(v5b) 판을 기준으로 두고, origin/main 판에만 있던 줄을 원래 순서대로 아래에 보존한다. 날짜가 이른 기록(11-14~11-18)이며 위 본문의 최신 상태와 다르면 위 본문이 우선한다. 원문: `git show ea2d9632:WORKTREES.md`.

| **메인 (보존 1)** | `<WORKSPACE_PARENT>/Capstone1_limo` | `staging/fleet-control-on-progress-c804241` | `c804241` | 공통 / Claude Code: 메인 저장소, `capstone_ws`, **통합 `fleet_control/`(단일 기준)**, `experiments/*20261113*`, `local_data/` | `M AGENTS.md`, `M README.md`, 다수 미추적(`fleet_control/`, `experiments/*`, `notebook/`, 공통 문서 8종) | **보존 필수 (절대 이동·초기화 금지)** |
| **유실 1 (해제 완료)** | *(워크트리 해제 완료)* | `integrate/privacy-preserved-main` | `b1d14de` | 과거 임시 워크트리 정리 완료 | Clean | **해제 완료** |
- `main` / `origin/main` (`a8132c7`): PR #1~#4, PR #8(공통 문서), **PR #9(`b350efa`, `fleet_control/` 173개)**, **PR #10(`a8132c7`, `run_recorder.py` 경로 개선)**이 병합된 정식 공개 기준선.
- `feat/fleet-control-pr-b` (`e0bc7be`): PR #9 병합 브랜치 (`fleet_control/` 173개 + 증거 해시 안내 보완).
- `feat/run-recorder-pr-c` (`f63688e`): PR #10 병합 브랜치 (`run_recorder.py` 신규 clone 출력 경로 보완).
- `feat/fleet-control-unified` (`ed53d81`): `51175ef` + `419e798` 기반 로컬 통합 보존본(비런타임 파일 총 203개 = PR #9 공개 173개 + 제외된 비필수 로그·스크린샷 30개; 외부 실험 스크립트 3개는 PR-D 보류).
## 2026-11-18 4차 선택 통합
- 공개 `main` 기준은 `1ca1393d229a1a3e1727079d61f8a1210a9520ac`이다. 기존 통합 복사본 `/tmp/capstone-public-integration-20261118`의 `integration/selected-20261118`에서 Goal, 영상, 모의 Mapping·Bag·Gamepad, 공통 문서 후보 27개를 확인했다. 이 항목은 별도 작업트리 기록이다.
- 원본 브랜치 `staging/fleet-control-on-progress-c804241`의 소스와 실험 데이터는 원래 위치에 보존한다. 기존 실차 자동 맵핑 번들도 그 위치를 유지한다.
- 로컬 보존본은 원본 작업트리의 `local_data/raw_results/capstone18_git4.2VR6Cu/`다. `BASE_COMMIT`, `base-files.tar.gz`, `changes.patch`, 묶음별 패치, `files.tsv`, `SHA256SUMS`가 있다. 임시 통합 복사본이 사라지면 기준 커밋과 복구 패치를 사용한다.
- 제6 문서는 영상 그룹의 `fleet_control/dashboard/README.md`다. 공통 문서 그룹 5개에 대한 추적은 감사 목록을 따른다. 모의 Mapping·Bag 결과는 실제 ROS 실행 및 실차 검증과 구분한다.
최종 재검증 기록과 이 문서를 포함한 복구본은 `local_data/raw_results/capstone18_git4_execution.0R5xNb/`에 보관한다.
### 전체 기능 구현 우선 개발
기존 `/tmp/capstone-public-integration-20261118`의 `integration/selected-20261118`에서 작업했다. 시작 시 GitHub main과 통합 HEAD는 `f1a87cb44916db4769cef575e6de9a19a1e555a8`였다. 작업 중 별도 Gazebo·LiDAR 증거 커밋으로 로컬 HEAD가 `8b03e0f`까지 전진했다. 종료 점검에서 GitHub main과 로컬 HEAD가 `8b03e0f`로 일치했다. 변경 경로가 겹치지 않아 두 커밋을 그대로 보존했다. 이번 검증의 HTTP 28943–28947 및 로컬 DDS 도메인 193, 195–202은 임시 사용했다. 다음 실행은 새로운 디렉터리와 비어 있는 포트·도메인을 지정한다.

## 2026-11-23 G2 통합 워크트리 (Claude, 사용자 승인)
| 경로 | 브랜치 | 출발 | 담당 | 상태 |
|---|---|---|---|---|
| `<WORKSPACE_PARENT>/Capstone1_limo_g2_20261123` | `integration/fleet-g2-main-v5b-20261123` | `40b666ec` (v5b 통합) | Claude Code | `origin/main` 병합 진행 중(no-commit), 충돌 해결 완료, 미커밋 |

## 2026-11-24 최종 통합 기준선 및 5개 원본 Worktree 보존 현황

| 로컬 Worktree (`<WORKSPACE_PARENT>` 기준) | 브랜치 | HEAD | 역할 / 담당 | 보존 상태 |
|---|---|---|---|---|
| `Capstone1_limo` | `staging/fleet-control-on-progress-c804241` | `c804241a` | 기존 staging · `local_data/` · `experiments/` 원본 보존 | 무변경 보존 |
| `Capstone1_limo_g2_20261123` | `integration/fleet-g2-main-v5b-20261123` | `15bb5c59` | Claude Code: Real Agent · Watchdog · Gate · Fleet Server · 건식 임무 (미커밋 67건) | 무변경 보존 |
| `Capstone1_limo_web_ui_20261123` | `feat/web-map-speed-ui-20261123` | `15bb5c59` | Antigravity: Web UI 지도·속도 제어 패널 (미커밋 10건) | 무변경 보존 |
| `Capstone1_limo_nav_validation_20261124` | `feat/nav2-stabilization-patrol-20261124` | `15bb5c59` | Antigravity: Nav2 O6 안정화 · 7시나리오 순찰 검증 (미커밋 7건) | 무변경 보존 |
| `Capstone1_limo_agent_harness_20261124` | `feat/agent-harness-m0-20261124` | `15bb5c59` | Antigravity: Agent Development Harness v0 / M0 (미커밋 6건) | 무변경 보존 |

- **새 통합 기준선 브랜치**: `integration/capstone1-field-release-20261124` (기준 `15bb5c59` 위에 위 4개 작업 + `stack_up.sh.MERGED` + `UI_INTEGRATION_PATCH_20261128b.diff` + `E2E_KEEP_STACK` + 공통 문서 최신화를 통합).
- **통합 작업 공간 정책**: 신규 영구 worktree를 추가하지 않고 임시 스크래치 Git 클론에서 통합·검증·커밋·푸시를 수행하여 위 5개 원본 worktree와 미커밋 파일을 모두 그대로 보존했습니다.

## 2026-11-24 실제 LIMO 프로젝트 디렉터리 구조 (1·2단계 정리 후)

실차(`pcu@LIMO`)의 프로젝트 루트는 `/home/pcu/Capstone1_limo`이며 이 문서에만 머신 고유 경로를 기록한다. 정리는 이동만 했고(총 13건) 삭제하지 않았다.

```text
/home/pcu/Capstone1_limo/
├── robot_ws/                          유지 (ROS 2 워크스페이스, 이동·수정 금지, q10 설치본)
├── deploy_candidates_20261124/        유지 (agent1, cap1 = q10 후보, restore_nav2_profile)
├── deploy_backups_20261124/           유지 (11-24 교체 전 백업, 롤백 기준)
├── archive/
│   ├── 20261120/  predeploy_agent_20261120_085514.tar.gz(+.files.sha256, .sha256), predeploy_mapsave_agent_20261120_134125.tar.gz(+.sha256)
│   └── 20261122/  backups/, stage_a_v3/, stage_a_v4/
├── tools/limo_isolated_18290_tools/   start_robot_agent_field.sh, dds_slot_precheck.py
└── records/
    ├── mapping/limo-real-auto-mapping/   33개 run (8.2 GB)
    ├── validation/limo-depth-observe/
    ├── video/limo_field_cam/
    └── maps/maps_h107_20261120/
```

- **이동하지 않은 항목(보류)**: `~/.local/state/limo_locked_stack`(노트북 `stop_field_session.sh`가 이 경로를 수집, `latest` 파일에 절대 경로), `~/.local/state/limo_robot_agent_field_20261120`(현재 Agent state_dir), `~/.local/state/limo_robot_agent_isolated_18290`, `limo_robot_agent`, `limo_robot_agent_field`, `~/limo_video_kit`(`stop_field_session.sh`가 참조, 실행 상태·비밀 포함), 홈 직하 개별 스크립트 5개(소유 불확실·상호 절대 경로 의존), `~/.config/limo_*`(인증), `capstone_ws`, `limo_capstone`, 학교·이전 기수 환경, 휴지통.
- **노트북 기록(git 제외 `local_data/`)**: `raw_results/limo_cleanup_20261124/` — `pre_move_backup/`(1단계 대상의 독립 사본), `phase2_backup/`(영상 키트 비밀 제외 사본 등), `unbacked_copy_from_limo/`, `restore_verify/`(하드링크 복원본), 이동 목록과 해시(`MOVES*_forward.tsv`, `LIMO*_BEFORE/AFTER_*`), 역방향 이동 `move_reverse.sh`·`move2_reverse.sh`, `verify_moved_checksums.sh`, `DOC_CHANGE_PROPOSALS.md`, `robot_originals_copy_relocation.diff`(저장소에 적용된 패치의 원본 diff).
- 이번 정리에서 새 worktree·새 ROS 2 워크스페이스는 만들지 않았다.
