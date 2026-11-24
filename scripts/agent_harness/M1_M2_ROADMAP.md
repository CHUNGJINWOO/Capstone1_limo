# Agent Development Harness — M1 · M2 확장 제안서

본 문서는 **M0 (읽기 전용 Preflight + 완료 보고 템플릿)** 이후 단계에서 검토할 수 있는 M1·M2 확장 후보를 정리합니다.
M0에서는 의도적으로 자동 실행·자동 병합·자동 배포 기능을 제외했습니다.

---

## 1. M0에서 의도적으로 제외한 기능 (비목표)

- AI 에이전트 자동 실행 및 AI 모델 자동 선택
- 작업 자동 분배 / 중앙 멀티에이전트 오케스트레이터
- Git 자동 병합(`git merge`), 코드 자동 수정, 자동 커밋/푸시
- 실제 ROS 2 / Gazebo 테스트 자동 실행
- 실차(LIMO Pro) 자동 배포 및 운영 서버(`18190`) 재시작
- 실차 운동 안전 승인 자동화

---

## 2. M1 확장 권고 (저위험 · 개발 생산성 강화)

1. **Post-flight 보고서 초안 자동 생성기 (`postflight.py`, 읽기 전용)**
   - 작업 종료 시 현재 worktree의 `git status`/`git diff --stat`, 실행한 preflight JSON, 지정한 테스트 결과 JSON(`result.json` 등)을 읽어 `REPORT_TEMPLATE.md` 형식의 마크다운 초안을 자동 채움.
   - 기존 worktree와 `VALIDATION.md`는 직접 수정하지 않고, `VALIDATION.md`/`HANDOFF.md`용 append 제안 스니펫만 생성.
2. **Pre-existing Dirty 스냅샷 기준선 (`--baseline-snapshot`)**
   - 작업 시작 시 `preflight --json-out /tmp/pre.json`으로 기존 dirty 파일 목록과 해시를 저장하고, 종료 시 `postflight --pre /tmp/pre.json`과 비교해 **기존 미커밋 변경이 1바이트도 훼손되지 않았음**을 자동 증명.
3. **ROS 도메인 / 포트 빈 슬롯 추천 (읽기 전용)**
   - `WORKTREES.md`의 예약 대역과 현재 `ps`/`ss` 결과를 조합해 충돌 없는 격리 `(ROS_DOMAIN_ID, TCP_PORT)` 후보를 제안 (단, 바인딩이나 ROS 노드 실행 없이 조회만 수행).
4. **공통 문서 패치 제안 검증기**
   - `WORKTREES.md`, `CODE_INDEX.md`, `VALIDATION.md`에 대해 생성된 패치 제안이 개인정보/토큰/절대경로 공개 정책(`docs/PUBLICATION_POLICY.md`)을 위반하지 않는지 정적 검사.

---

## 3. M2 확장 권고 (승인 기반 반자동 워크플로)

1. **사람 승인 게이트(Human-in-the-loop) 기반 통합 체크리스트**
   - 여러 worktree(G2, Web UI, Navigation, Harness)의 패치 제안을 한 번에 비교해 파일 교차 충돌 매트릭스를 생성하고, 사람이 승인한 순서대로만 수동 병합 절차를 안내.
2. **기존 `field_preflight_20261123.py` / `dds_slot_precheck.py` 연동 브리지 (선택 호출)**
   - 개발자가 명시적 플래그로 요청할 때만 기존 ROS/실차 읽기 전용 점검 도구의 JSON 결과를 보고서 첨부 자료로 수집 (단, 실차 구동·모터 제어·배포 승인은 여전히 사람이 현장 입회 하에 별도 수행).
3. **CI / 로컬 훅 연동 (비파괴 검사 전용)**
   - `scopes.json` 스키마 검증, 비밀정보 패턴 누출 검사, 보고서 템플릿 필수 항목 누락 검사를 읽기 전용 체크로 연결.
