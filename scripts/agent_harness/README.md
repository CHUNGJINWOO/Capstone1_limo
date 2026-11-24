# Agent Development Harness v0 (M0)

`Capstone1_limo` 저장소에서 여러 AI 코딩 에이전트(Claude Code, Codex, Antigravity 등)가 병렬 개발할 때, **작업 시작 전 읽기 전용 Preflight**와 **작업 종료 후 보고 표준화**를 지원하는 경량 개발 도구입니다.

- **ROS 2 주행 테스트 하네스가 아닙니다.** (실시간 로봇/ROS 런타임 점검은 기존 `fleet_control/scripts/field_preflight_20261123.py`, `dds_slot_precheck.py`를 재사용합니다.)
- **이 도구의 `PASS`는 로봇 운동 안전 승인이 아닙니다.**
- **Python 3 표준 라이브러리만 사용**하며 외부 패키지 의존성이 없습니다.

---

## 1. 파일 구성

| 파일 | 역할 |
|---|---|
| `scripts/agent_harness/preflight.py` | 읽기 전용 개발 Preflight CLI (터미널 및 JSON 출력) |
| `scripts/agent_harness/scopes.json` | 에이전트 담당 영역, 공통 문서, 보호 포트/ROS 도메인, 위험 작업 카탈로그 |
| `scripts/agent_harness/REPORT_TEMPLATE.md` | 12개 항목 + 6단계 검증 수준 작업 완료 보고 템플릿 |
| `scripts/agent_harness/test_preflight.py` | 합성 Git fixture 및 모의 환경 기반 단위 시험 |
| `scripts/agent_harness/M1_M2_ROADMAP.md` | M0 범위 제한 및 M1/M2 확장 제안서 |

---

## 2. 읽기 전용 보장 (Hard Guarantees)

`preflight.py`는 다음 작업을 **절대 자동 실행하지 않습니다**:
- Git 변경 명령: `git reset`, `git clean`, `git stash`, `git commit`, `git push`, `git merge`, `git checkout`, `git worktree remove`
- 프로세스 시그널 전송(`kill`), 포트 바인딩/점유, DDS 설정 변경, ROS 2 노드 실행
- 인증 토큰, 비밀번호, 비밀 환경변수 출력 또는 JSON 기록 (허용된 ROS 환경변수 5종만 읽고, `ps` 인자의 토큰·비밀번호 패턴은 `<REDACTED>`로 마스킹)

---

## 3. 에이전트별 작업 시작 명령 예시 (복사해 바로 사용)

> **중요 안전 고지**: Preflight `PASS`는 읽기 전용 개발 위생 점검 결과일 뿐 **작업 안전이나 로봇 운동 안전을 보장하지 않으며**, 실제 LIMO 접속·모터 제어·실차 배포·운영 서버(`18190`) 변경·Git `commit`/`push`/`PR`에 대한 **사용자 승인을 절대 대체하지 않습니다.**
> **추적 한계 안내**: M0는 작업 시작 전 `--files`로 선언한 경로와 현재 `git status`만 검사합니다. 작업 중 `--files`에 명시하지 않고 추가로 수정한 파일은 자동 추적되지 않으므로, 범위가 바뀌면 Preflight를 다시 실행하세요.

### Claude Code (Real Agent / Watchdog / Gate / Fleet Server — G2 Worktree)
```bash
python3 /home/cjinw/school/2026-2/Capstone1_limo_agent_harness_20261124/scripts/agent_harness/preflight.py \
  --repo /home/cjinw/school/2026-2/Capstone1_limo_g2_20261123 \
  --agent "Claude Code" \
  --task-id "g2-task-YYYYMMDD" \
  --scope real_agent_safety \
  --files fleet_control/agent/limo_robot_agent/limo_robot_agent/core.py
```

### Codex (독립 안전 감사 / 읽기 전용 검증)
```bash
python3 /home/cjinw/school/2026-2/Capstone1_limo_agent_harness_20261124/scripts/agent_harness/preflight.py \
  --repo /home/cjinw/school/2026-2/Capstone1_limo_g2_20261123 \
  --agent "Codex" \
  --task-id "codex-audit-YYYYMMDD" \
  --skip-ros \
  --json
```

### Antigravity (Web UI Worktree)
```bash
python3 /home/cjinw/school/2026-2/Capstone1_limo_agent_harness_20261124/scripts/agent_harness/preflight.py \
  --repo /home/cjinw/school/2026-2/Capstone1_limo_web_ui_20261123 \
  --agent "Antigravity" \
  --task-id "webui-task-YYYYMMDD" \
  --scope web_ui \
  --files fleet_control/server/fleet_server/static/speed_control_panel.js
```

### Antigravity (Navigation / 순찰 검증 Worktree — 격리 ROS 도메인·포트 점검 포함)
```bash
python3 /home/cjinw/school/2026-2/Capstone1_limo_agent_harness_20261124/scripts/agent_harness/preflight.py \
  --repo /home/cjinw/school/2026-2/Capstone1_limo_nav_validation_20261124 \
  --agent "Antigravity" \
  --task-id "nav-task-YYYYMMDD" \
  --scope navigation_patrol \
  --files fleet_control/tests/nav_e2e/run_nav_e2e.py \
  --domain 164 \
  --ports 29711
```

### 위험 작업 사전 점검 예시 (승인 없는 경우 `BLOCKED` 반환)
```bash
python3 scripts/agent_harness/preflight.py \
  --repo . \
  --agent "Antigravity" \
  --task-id "deploy-check" \
  --actions real_robot_deploy,git_write_or_pr
```

---

## 4. 주요 옵션

| 옵션 | 설명 |
|---|---|
| `--repo PATH` | 검사 대상 저장소/워크트리 경로 (기본: `.`) |
| `--agent NAME` | 에이전트명 (`Claude Code`, `Antigravity`, `Codex` 등, 필수) |
| `--task-id ID` | 작업 식별자 (필수) |
| `--scope KEY_OR_LABEL` | `scopes.json`의 담당 영역 키 또는 라벨 (`real_agent_safety`, `web_ui`, `navigation_patrol`, `agent_harness`) |
| `--files P1,P2` | 이번 작업에서 수정 예정인 상대 경로 (반복 지정 또는 쉼표 구분) |
| `--base-commit SHA` | 기준 커밋 (기본값: `scopes.json`의 `baseline_commit`) |
| `--expected-branch NAME` | 기대 브랜치명 |
| `--domain INT` | 사용 예정 `ROS_DOMAIN_ID` |
| `--ports P1,P2` | 사용 예정 TCP 시험 포트 |
| `--actions A1,A2` | 수행 예정 위험 작업 키 (`scopes.json`의 `dangerous_actions`) |
| `--approved-actions A1` | 사람이 명시적으로 승인한 위험 작업 키 |
| `--no-siblings` | 타 워크트리(`git worktree list`)의 dirty 파일 교차 검사 생략 |
| `--skip-ros` | ROS 2 프로세스/포트 검사 건너뛰기 (`SKIP`) |
| `--json` / `--json-out FILE` | JSON 표준 출력 / 파일 저장 |

---

## 5. 상태 코드 및 종료 코드

- **항목별 상태**: `PASS`, `WARN`, `BLOCKED`, `UNKNOWN`, `SKIP`
- **종료 코드**:
  - `0`: `PASS`, `WARN`, `UNKNOWN`, `SKIP` (읽기 전용 경고 포함, 차단 없음)
  - `2`: 하나 이상의 검사에서 `BLOCKED` 발생 (타 에이전트 담당 파일 충돌, 타 워크트리 미커밋 충돌, 활성 ROS 도메인/보호 포트 충돌, 미승인 위험 작업, 잘못된 저장소 경로)
  - `3`: CLI 인자 오류 또는 설정 파일(`scopes.json`) 누락

---

## 6. 단위 시험 실행

```bash
python3 -m unittest discover -s scripts/agent_harness -p "test_*.py" -v
```
