# 작업 완료 보고서 템플릿 (Agent Development Harness v0 / M0)

> **주의**: 기존 [VALIDATION.md](../../VALIDATION.md), [CURRENT_STATUS.md](../../CURRENT_STATUS.md), [HANDOFF.md](../../HANDOFF.md), [WORKTREES.md](../../WORKTREES.md)의 본문을 중복 복제하지 말고, 해당 문서를 참조·연결하거나 변경 제안으로 남기세요.
> **안전 고지**: 개발 Preflight `PASS`나 시뮬레이션 통과를 **실차(LIMO Pro) 운동 안전 승인**으로 해석하지 마세요.

---

## 1. 작업 ID 및 담당 에이전트
- **작업 ID**: `<예: harness-m0-20261124>`
- **담당 에이전트**: `<예: Claude Code | Antigravity | Codex>`
- **선언된 작업 범위 (`--scope`)**: `<예: agent_harness | web_ui | navigation_patrol | real_agent_safety>`
- **완료 시각**: `<YYYY-MM-DD HH:MM KST>`

## 2. 기준 커밋과 작업 브랜치
- **작업 Worktree 경로**: `<WORKSPACE_PARENT>/<worktree_name>`
- **기준 커밋 (Base Commit)**: `<예: 15bb5c59b806f58d95e19fb6308effd02c15f0de>`
- **작업 브랜치 / 현재 HEAD**: `<branch_name> @ <short_sha>`
- **실행 전 Preflight 결과**: `<PASS | WARN | BLOCKED | UNKNOWN>` (`<로그/JSON 경로>`)

## 3. 수정된 파일
| 상태 (`A`/`M`/`D`) | 상대 경로 | 변경 요약 |
|---|---|---|
| `A` | `<path/to/new_file>` | `<요약>` |
| `M` | `<path/to/edited_file>` | `<요약>` |

## 4. 구현한 기능
- `<핵심 구현 항목 1>`
- `<핵심 구현 항목 2>`
- *(범위 외 / 미구현 항목은 9절에 명시)*

## 5. 실제 수행한 시험
- `<시험 1: 대상, 조건, 반복 횟수>`
- `<시험 2: 대상, 조건, 반복 횟수>`

## 6. 시험 판정 요약 (`PASS / FAIL / SKIP / BLOCKED`)
| 시험 항목 | 결과 (`PASS`/`FAIL`/`SKIP`/`BLOCKED`) | 비고 / 실패·건너뜀 사유 |
|---|---|---|
| `<항목 1>` | `PASS` | `<요약>` |
| `<항목 2>` | `SKIP` | `<사유>` |

## 7. 검증 수준 (해당 항목만 체크 — 복수 선택 가능)
- [ ] **설계 / 정적 검토** (문서·코드 리뷰, 정적 분석)
- [ ] **Mock** (모의 API / 합성 Git fixture / 외부 의존성 대체)
- [ ] **단위 시험** (`unittest` / `pytest` / `node` 등 자동 단위 테스트)
- [ ] **격리 ROS 2** (`ROS_LOCALHOST_ONLY=1`, 격리 도메인/포트, Kinematic 시뮬레이터 포함)
- [ ] **Gazebo** (`Ignition Fortress` 물리 시뮬레이터 주행)
- [ ] **실차** (AgileX LIMO Pro 실기 검증 — 현장 입회·단계별 승인 기록 필수)

> 전체 프로젝트 누적 검증 색인은 [VALIDATION.md](../../VALIDATION.md) 및 [CURRENT_STATUS.md](../../CURRENT_STATUS.md)를 참조합니다.

## 8. 로그 및 재현 명령
- **로그 / 아티팩트 경로**: `<예: /tmp/... 또는 로컬 결과 경로 (민감 정보·토큰 제외)>`
- **재현 명령**:
  ```bash
  # 복사해서 그대로 재현 가능한 명령 (비밀값·토큰 절대 기재 금지)
  ```

## 9. 미해결 문제 및 한계
- `<미해결 문제, 미검증 영역, 후속 작업 필요 사항>`

## 10. 필요한 공통 문서 변경 (직접 수정 대신 제안 기재)
- [WORKTREES.md](../../WORKTREES.md): `<신규/변경 워크트리·포트·도메인 반영 제안 또는 없음>`
- [CODE_INDEX.md](../../CODE_INDEX.md): `<신규 도구·경로 색인 제안 또는 없음>`
- [VALIDATION.md](../../VALIDATION.md) / [CURRENT_STATUS.md](../../CURRENT_STATUS.md) / [HANDOFF.md](../../HANDOFF.md): `<갱신 제안 또는 없음>`

## 11. 다른 에이전트와의 충돌 가능성
- **타 Worktree 상태 불변 확인**: `<확인한 worktree 목록 및 실행 전후 git status 일치 여부>`
- **타 에이전트 담당 영역 접촉 여부**: `<없음 | 있음 (패치 제안 경로 명시)>`

## 12. commit · push · 배포 여부
- **Git commit**: `<미수행 | 수행 (SHA)>`
- **Git push / PR**: `<미수행 | 수행 (링크)>`
- **실차(LIMO) / 운영 서버(`18190`) 배포·변경**: `<미수행 | 수행 (승인 근거)>`
