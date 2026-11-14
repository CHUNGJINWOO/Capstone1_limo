# START_HERE.md — Capstone1 프로젝트 진입점 (공통 AI 컨텍스트)

> **기준일**: 2026-11-14  
> **프로젝트명**: LIMO를 이용한 실내 자율 순찰 및 안내 (AgileX LIMO Pro · ROS 2 Humble)  
> **원격 저장소**: `https://github.com/CHUNGJINWOO/Capstone1_limo.git`

이 문서는 **Claude Code, Codex, Gemini 및 ChatGPT**가 작업을 시작할 때 가장 먼저 읽는 **단일 진입점**입니다. 매 작업마다 모든 공통 문서를 무조건 읽지 말고, 아래 표에 따라 **현재 작업 유형에 필요한 문서만 선택적으로 확인**하세요.

---

## 1. 작업 유형별 선택적 읽기 가이드 (Selective Reading)

| 작업 유형 | 필수 확인 문서 | 필요 시 추가 참조 문서 |
|---|---|---|
| **모든 작업 공통 (최초 진입)** | [START_HERE.md](START_HERE.md), [AGENTS.md](AGENTS.md), [HANDOFF.md](HANDOFF.md) | [CURRENT_STATUS.md](CURRENT_STATUS.md) (담당 영역 행만 확인) |
| **온보드 ROS 2 / 안전 체인(`fixed2`) / SLAM·Nav2 코드 수정** | [CODE_INDEX.md](CODE_INDEX.md) (§1), [ARCHITECTURE.md](ARCHITECTURE.md) (§2), [CURRENT_STATUS.md](CURRENT_STATUS.md) (§2.1~2.2) | [DECISIONS.md](DECISIONS.md) (§1), [VALIDATION.md](VALIDATION.md) (§1) |
| **중앙 관제 / Robot Agent / Open-RMF / 대시보드·영상 AI 작업** | [CODE_INDEX.md](CODE_INDEX.md) (§2), [WORKTREES.md](WORKTREES.md) (§1~§3), [ARCHITECTURE.md](ARCHITECTURE.md) (§3) | [CURRENT_STATUS.md](CURRENT_STATUS.md) (§2.3~2.4), [DECISIONS.md](DECISIONS.md) (§2) |
| **실차(LIMO) 점검·시험 또는 시뮬레이션 실행·평가** | [CURRENT_STATUS.md](CURRENT_STATUS.md) (§3 차단 요인), [WORKTREES.md](WORKTREES.md) (§3 포트·도메인), [VALIDATION.md](VALIDATION.md) | [DECISIONS.md](DECISIONS.md) (D-02, D-05) |
| **폴더·워크트리 정리 또는 Google Drive / YouTube 백업** | [WORKTREES.md](WORKTREES.md), [CODE_INDEX.md](CODE_INDEX.md) (§3), [VALIDATION.md](VALIDATION.md) (§2), [docs/PUBLICATION_POLICY.md](docs/PUBLICATION_POLICY.md) | [DECISIONS.md](DECISIONS.md) (D-09) |

### 공통 문서 8종 역할 요약
- **[AGENTS.md](AGENTS.md)** — 개발·검증 원칙, 공개 정책([docs/PUBLICATION_POLICY.md](docs/PUBLICATION_POLICY.md)), 멀티에이전트 보호 규칙
- **[CURRENT_STATUS.md](CURRENT_STATUS.md)** — 영역별 구현 완료 vs 검증 수준(**실차 / Gazebo / 합성·오프라인 / 미검증 / 제안**) 및 실패·한계 요약
- **[HANDOFF.md](HANDOFF.md)** — 최근 완료 작업, 실차·시뮬레이션 차단 요인(Blocker), 다음 작업, 주의 사항
- **[WORKTREES.md](WORKTREES.md)** — 10개 Git 워크트리(디스크 실존 9개 + `prunable` 1개), `$HOME` 하위 격리 실행 환경, 포트·`ROS_DOMAIN_ID` 할당표
- **[CODE_INDEX.md](CODE_INDEX.md)** — `main` 기준 소스, 실차 배포본, 시뮬레이션 시험본, `fixed2` 실험 수정본의 위치 구분
- **[ARCHITECTURE.md](ARCHITECTURE.md)** — 확인된 실행 모드 기준 온보드 안전 체인 및 중앙 관제·RMF·대시보드·영상 AI 통신 구조
- **[DECISIONS.md](DECISIONS.md)** — 확정된 설계 결정(`D-01`~`D-08`)과 합성·Gazebo 검증 후보·미확정 정책(`C-01: fixed2`) 구분
- **[VALIDATION.md](VALIDATION.md)** — 시험 결과(PASS 및 실패·한계 포함) 색인 및 Google Drive / YouTube / `rosbag2`·설정 스냅샷 보관 규격

---

## 2. 현재 저장소·워크트리 핵심 주의사항 (필독)

1. **메인 폴더(`Capstone1_limo`)의 체크아웃 브랜치 상태**:
   - 정식 통합 브랜치는 `main`(`2010e44`)이고 중앙 관제 초기 통합 커밋은 `feat/fleet-control-rmf-integration`(`419e798`)입니다.
   - 그러나 현재 로컬 메인 워크트리(`<WORKSPACE_PARENT>/Capstone1_limo`)는 미추적 실험 폴더와 로컬 데이터를 보호하기 위해 정리 이전 브랜치인 **`staging/fleet-control-on-progress-c804241` (`c804241`)**에 머물러 있습니다.
   - 따라서 **`main`(`2010e44`) 또는 `419e798` 기준 파일([docs/REAL_ROBOT_STATUS.md](docs/REAL_ROBOT_STATUS.md), [docs/CONFIG_SIM_VS_REAL.md](docs/CONFIG_SIM_VS_REAL.md), [experiments/README.md](experiments/README.md) 등)**을 로컬 `c804241` 워크트리에서 확인할 때는 `git show main:<경로>`를 쓰거나 `419e798` 기반 로컬 워크트리([WORKTREES.md](WORKTREES.md))를 참조하세요.
   - **승인 없이 `git checkout`, `git reset`, `git clean`, `git stash`, `git worktree move/remove`를 실행하지 마세요.**
2. **구현 완료와 검증 완료의 엄격한 구분**:
   - 코드가 존재하거나 합성 하네스·Gazebo에서 통과했다는 사실을 **실차(Physical LIMO) 검증**으로 기술하면 안 됩니다.
   - 특히 실차 자율 주행(`--drive`)은 **전원 돌발 차단(4회) 원인 미규명**, **Depth 스케일·지터 한계**, **독립 물리 E-Stop 미검증**으로 인해 사용자 현장 입회·단계별 승인 없이는 엄격히 차단(**BLOCKED**)되어 있습니다.
3. **보호 포트 및 외부 프로세스 중단 금지**:
   - 포트 `8080`(운영 서버), `18190~18192`(실차 현장 키트), `18120~18121`(Gazebo RMF), `18200~18212`(통합 시뮬레이션), `18310~18313`(Dashboard Lab), `18410`(RMF Web Storybook)에서 실행 중인 타 세션 프로세스를 임의로 종료하지 마세요.
4. **절대 경로 의존 스크립트 보호**:
   - `.git/worktrees/*` 포인터, `capstone_ws/{build,install}`, `experiments/office_limo_sim_20261113`, `experiments/safety_chain_fix_20261113/office/*.launch.py`, `experiments/real_limo_field_20261113/laptop/lf_*.sh`가 현재 폴더 절대 경로 및 `$HOME/rmf_isolated`, `$HOME/rmf_demos_humble`, `$HOME/.local/share/robot-sim-projects/*`를 직접 참조합니다. 폴더를 임의로 이동하거나 이름을 바꾸지 마세요.

---

## 3. 작업 종료 시 문서 갱신 규칙

작업이 끝나면 전체 문서를 다시 쓰지 말고 **실제 변경 사항에 해당하는 문서만 최소한으로 갱신**합니다.

| 작업 유형 | 갱신 대상 문서 |
|---|---|
| 코드·패키지·실험 변형 경로 추가/변경 | [CODE_INDEX.md](CODE_INDEX.md), [HANDOFF.md](HANDOFF.md) |
| 시뮬레이션·하네스·실차 시험 실행 및 결과 추가 (실패·한계 포함) | [VALIDATION.md](VALIDATION.md), [CURRENT_STATUS.md](CURRENT_STATUS.md), [HANDOFF.md](HANDOFF.md) |
| 사용자 승인을 거쳐 새로운 설계 결정을 최종 확정한 경우 | [DECISIONS.md](DECISIONS.md), (구조 변경 시) [ARCHITECTURE.md](ARCHITECTURE.md) |
| 워크트리·브랜치·포트·외부 실행 환경 변경 | [WORKTREES.md](WORKTREES.md), [HANDOFF.md](HANDOFF.md) |
| 일반 작업 인수인계 (다음 에이전트 전달 사항) | [HANDOFF.md](HANDOFF.md) |
