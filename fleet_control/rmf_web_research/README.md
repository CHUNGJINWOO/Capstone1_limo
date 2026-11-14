# 공식 RMF Web 독립 UI 연구 — 2026-11-13

## 실행 결과와 범위

공식 `open-rmf/rmf-web`의 framework package0.3.0, main commit **c1b13dc90c0f6983970df87f58004e011a322d2f**(2026-11-12)를 격리 복제했습니다. Node22.23.5/pnpm11.25.0, Storybook10.5.8/Vite7.3.6 및 공식 pnpm-lock 고정 의존성을 사용했습니다. 별도 pnpm store와 node_modules만 설치했고 ROS/Python/system install scripts는 실행하지 않았습니다.

- Storybook: http://127.0.0.1:18410/
- 지도·로봇·임무 연구 스토리: http://127.0.0.1:18410/iframe.html?id=capstone-offline-office-research--map-robots-tasks&viewMode=story
- 로봇 요약: http://127.0.0.1:18410/?path=/story/capstone-offline-office-research--populated-robot-summary
- 실제 실행 소스: `/tmp/capstone-rmf-web-official-20261113` (tmp 정리 시 재준비 필요)
- 이 폴더의 harness/는 실험용 설정과 스토리만 보관합니다. 기존 dashboard/dashboard_lab 구현을 변경하지 않았습니다.

**실행한 것은 frontend Storybook + mock provider입니다. RMF API Server, ROS middleware, Docker container, Office World, 실제 LIMO는 실행/연결하지 않았습니다.** Mission Manager/Fleet Adapter/Collision Monitor/기존 서버를 교체하거나 재시작하지 않았습니다.

## 원본과 실험 변경

원래 Storybook 설정은10.x 본체와8.x essentials/interactions 애드온 export 충돌(SB_CORE-SERVER_0004)로 실패했습니다. 공식 lockfile/원본소스는 그대로 두고 `.storybook-capstone/`에서 이 두 애드온만 제외했습니다. 구버전 test helper 등 호환성 경고는 남아 있습니다. stock 설정의 완전 성공으로 보고하지 않습니다.

공식 API client/rmf-models TypeScript 빌드는 성공했습니다. rmf-models는 원래 build script와 동일하게 dist/package.json 복사를 완료했습니다. `--ignore-scripts`로 pipenv-install/ROS/Python 의존성 설치를 막았으며 pnpm11의 자동 의존성 재설치를 피하도록 빌드는 local tsc 바이너리를 직접 사용했습니다.

추가 스토리는 공식 Map, RobotDataGridTable, RobotSummary, TaskTable, TaskTimeline 및 MockRmfApi를 조합합니다. 공식 test-data/assets/office.png(3047×1717)와 officeMap, 공식 FakeTrajectoryManager를 사용합니다. 로봇2대·임무·배터리·시간은 합성 fixture입니다. L2_OFFLINE_COPY는 L1 복사본이며 실제 Office의 두 번째 층이라고 주장하지 않습니다. 모든 화면에 OFFLINE FIXTURE 표시가 있습니다. 원래 RobotSummary story는 fleet observable을 채우지 않아 제목/값이 비어 있었고, 추가 스토리는 해당 observable에 합성 데이터를 공급했습니다.

## 재실행·복구

현재18410이 실행 중이면 추가 시작하지 않습니다. 기존18310/18311 및 운영8080/18190/18210/18211은 소유가 다릅니다. 포트 충돌 시 다른 프로세스를 종료하지 않습니다.

```bash
# 이 연구 폴더에서, 현재 source/node_modules가 남아 있는 경우:
bash start_storybook.sh
# 종료는 이 명령을 실행한 터미널의 Ctrl-C
```

재부팅/tmp 정리로 source가 사라졌다면:

```bash
# 공식 GitHub/npm 네트워크 필요, 시스템 패키지/ROS 설치 없음
bash prepare_frontend.sh
bash start_storybook.sh
```

prepare는 기록된 commit과 다르면 기존 폴더를 덮어쓰지 않고 실패합니다. 기본 연구 소스 이외의 경로는 RMF_WEB_RESEARCH_ROOT로 지정합니다. 실행 스크립트 둘은 bash -n 구문 검사를 통과했습니다. 실행 환경을 만든 각 명령은 실제 성공했지만, 새 기기에서 prepare 전체를 처음부터 재실행한 재현 시험은 하지 않았습니다. 패키지 lock은 고정했고 Node/pnpm 버전 변경 시 별도 검증이 필요합니다. 전역 npm/pnpm/ROS 설정은 변경하지 않습니다.

## 설치 전 확인과 자원

Ubuntu22.04, RAM23Gi 중 available약11Gi, swap2Gi 미사용, 디스크 여유약720Gi를 확인한 후 설치했습니다. source+빌드+node_modules 약746MiB, 별도 store32MiB(하드링크 중복이 있어 단순 합계가 실제 점유량과 다를 수 있음). Storybook Node 순간 RSS556.34MiB,2초 CPU표본0%였습니다. 이는 화면 준비 후 유휴 표본이며 peak/build/브라우저/전체 RMF 부하나 최소 요구량이 아닙니다.

Docker client29.9.3은 있지만 현재 사용자는 docker.sock 접근 권한이 없습니다. sudo, docker 그룹 추가, rootless daemon, system package 설치, 이미지 pull은 하지 않았습니다. 관리자 변경이 필요한 full stack 실행은 별도 승인 후에만 진행합니다. frontend로 가능한 UI 검증은 완료했으므로 이번 작업을 위해 권한 변경을 요청하거나 시스템 설치를 강행하지 않았습니다.

main README는 Lyrical/Ubuntu26.04를 지원 대상으로 명시하며 nightly workflow matrix는 jazzy/lyrical/rolling입니다. 공식 humble branch54c9630abdce1eede0d319dc87f3a740ca1faed4는 Ubuntu22.04/Humble/RMF22.09, legacy packages/dashboard와 Python3.10을 기준으로 합니다. 현재 설치 RMF2.1.8과 완전 호환된다는 뜻은 아닙니다.

GHCR 메타데이터만 확인: api-server:humble-nightly는404, jazzy-nightly의 API/demo-dashboard digest는 evidence/image-digests.json에 기록했습니다. 현재 manifest는linux/amd64를 제공합니다. 이미지는 내려받지 않았으며 nightly tag 대신 digest 고정이 필요합니다. main 소스와 Jazzy 이미지가 같은 버전이라고 보장하지 않습니다. 사용하지 않은 image digest는 실행 성공 버전이 아닙니다.

## 검증 상태

|항목|결과|
|---|---|
|공식 소스 commit/lock 고정·frontend install|PASS|
|공식 API client·ROS 모델의 TypeScript 빌드|PASS|
|원본 Storybook 설정|FAIL: addon export mismatch|
|별도 설정 Storybook 렌더링|PASS, 호환성/DOM warning 남음|
|공식 임무 표3개·타임라인3단계|브라우저 확인, sample의 unknown 상태는 원래 값 유지|
|추가 Office fixture 지도·합성2대·임무|브라우저 확인, live 아님|
|지도 확대·층 선택|브라우저 확인, L2는 copy fixture|
|추가 로봇 요약|72% 배터리·40% 예상 진행률 표시 확인|
|인증/RBAC 실제 backend 보장|소스 분석만, end-to-end 미검증|
|full API/Docker/ROS/Office 연결|미실행|
|실차·실제 임무 할당/취소/모터|미실행|
|공식 전체 테스트·성능 회귀|미실행|

별도 연구 harness의 DOM 구조 경고 및 UI 폭이 좁을 때 표 가로 스크롤이 관찰됐습니다. 공식 UI의 반응형/정확한 time stale 처리·지도 yaw·장시간 WebGL 성능이 모두 검증됐다고 확대 해석하지 않습니다. dashboard_lab의 기존31+90 PASS/FPS 결과는 이전 기준선이며 이번 RMF Web 테스트 결과가 아닙니다.

[기능 비교·도입 우선순위](COMPARISON.md) · [API·Office 연동 설계](API_AND_OFFICE.md) · [버전·자원 근거](evidence/versions.json) · [공식 라이선스](harness/UPSTREAM-LICENSE.md)
