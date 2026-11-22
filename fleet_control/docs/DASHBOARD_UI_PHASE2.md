# Fleet Dashboard 2차 UI 개편

기존 Vanilla UI를 기능별 화면으로 재배치했다. 승인 이미지 원본은 대화에 제공되지 않아 1차 캡처와 텍스트 요구사항으로 비교했으며 이미지 일치 검증은 SKIP이다. 캡처의 데이터는 격리 fixture 합성 데이터다.

| 기존 기능 | 이동 화면 |
|---|---|
| 지도·선택·줌·라이브/미리보기 | Overview와 Maps (같은 캔버스와 선택 공유) |
| 임무 생성·현재 Goal·Lease·취소 | Operation, Overview 빠른 취소 |
| Map Registry 조회·등록·이력, 맵핑 미지원 | Maps |
| Gate/Safety Context 상세·Fault·재조정 복구 | Safety |
| Fleet 로봇 선택·RMF 현황·임무 이력 | Fleet/RMF |
| RGB/AI 연결과 검출·LiDAR/Depth 상태·텔레메트리 | Sensors |
| 이벤트·감사 로그·안전 원문·Journal 미제공 표기 | History |
| Pre-flight·지원되지 않는 전원/프로파일 조작 | Settings |
| 운영 매뉴얼 | Documentation |

상단 로봇/모드/Gate/이동/임무/재조정 상태는 모든 탭에서 유지된다. Overview의 긴 센서 목록과 영상은 Sensors로 이동했으며 최근 이벤트는 3개다. Goal UUID와 Command ID는 Operation에 유지된다. 브라우저 뒤로 가기와 키보드 Enter 탐색을 지원한다. Gate NORMAL 보고는 이동 승인과 구분하고 Attestation 미제공과 독립 정지 미지원 표시를 유지한다. 저장 지도 선택 시 Goal 생성 차단을 유지한다. 서버·Agent·Gate·RMF 코드는 변경하지 않았다.

## 검증

- PASS: JS helper 및 Registry upload; Safety Context Python 24, static HTTP 2, Map Registry HTTP 4.
- PASS: Chromium 기존 회귀 23항목(임무 생성/취소, Lease 만료, viewer, 미리보기 차단, Fault·Offline·Reconciliation·Attestation 누락, 서버 단절 포함). 탭 위치에 맞춰 기존 스크립트 사본을 수정했다.
- PASS: 각 9개 화면 키보드 Enter, 브라우저 뒤로 가기, 로봇·지도 선택 유지, 미리보기 차단, pageerror 없음.
- PASS: Overview 1920×1080·1600×900·1366×768 문서 양방향 overflow 없음, 모바일 390×844 가로 overflow 없음. 상세 화면은 내용에 따라 페이지 스크롤이 발생한다.
- SKIP: 승인 이미지와 직접 비교, 실제 센서 스트림, 실차 및 Gazebo. API가 제공하지 않는 진단이나 영상은 UNAVAILABLE로 남는다. 그래프는 근거 데이터가 없어 합성하지 않았다.

## 확인과 재현

```bash
python3 fleet_control/tests/dashboard_preview.py --info-file /tmp/unique-limo-ui.json
# 출력되는 loopback 주소를 열고 출력된 operator.token 파일의 내용을 입력
NODE_PATH=/home/cjinw/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules node fleet_control/tests/dashboard_browser_phase2.cjs /tmp/unique-limo-ui.json /tmp
NODE_PATH=/home/cjinw/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules node fleet_control/tests/dashboard_tabs_phase2.cjs /tmp/unique-limo-ui.json /tmp
```

브라우저 시험은 임시 fixture의 scenario를 변경하고 임무를 생성하므로 독립 fixture에서만 실행한다. 경로는 이 환경의 Playwright 설치 경로이며 다른 환경에서는 설치 위치를 사용한다.

1차 [1920×1080](ui-20261116-1920x1080.png) → 2차 [Overview](ui-phase2-overview.png), [1366×768](ui-phase2-overview-1366.png). 탭별: [Operation](ui-phase2-control.png), [Maps](ui-phase2-maps.png), [Safety](ui-phase2-safety.png), [Fleet/RMF](ui-phase2-rmf.png), [Sensors](ui-phase2-media.png), [History](ui-phase2-history.png), [Settings](ui-phase2-settings.png), [Documentation](ui-phase2-docs.png). 전체 페이지 캡처이며 일부 상세 화면은 1080px보다 길다.
