# 승인 Workspace 실제 static UI 적용

2026-11-17. 사용자 승인된 Overview·Spatial·Operation Workspace를 기존 Fleet Control static UI에 적용했다. 신규 프런트엔드 프로젝트나 프레임워크는 만들지 않았다. 기존 REST, Map Registry, 테마, 영상 패널, 지도 렌더러와 폼 ID를 재사용했다.

## 실제 기능 연결

| 화면/기능 | 구현 및 연결 |
|---|---|
| Overview | 왼쪽 로봇 목록, 중앙 공유 지도·최근 이벤트, 오른쪽 선택 로봇·안전·현재 임무 Inspector. 로봇 행/전역 selector 변경이 지도·폼 대상·Inspector·활동을 갱신. |
| Operation | 왼쪽 선택 로봇의 실제 임무 목록·로봇 대상, 중앙 지도와 명령 Drawer 진입, 오른쪽 선택 임무 상태·명령 출처·Lease·현재 Goal 취소. 과거 임무 Inspector 선택과 실제 취소 대상 ID를 구분해 표시. |
| Goal Drawer | 기존 NavigateToPose 폼을 이동. 좌표/로봇 요약과 사용자 확인 체크 후 최종 제출. 좌표/로봇이 바뀌면 확인 해제. 실제 권한·미리보기·서버 단절 차단을 유지. 최종 Safety Context는 서버가 판단한다. Escape·닫기·초점 순환·배경 inert 적용. |
| 지도 클릭 | Overview/Spatial 조회 전용. Operation/Maps는 기존 좌표 변환으로 목표 지정 후 Drawer에서 검토·제출. 초기 위치 미지원 미리보기는 Maps에 유지. |
| 공유 선택 | 동일 map_view 인스턴스와 기존 건물·층·버전 selector를 모든 지도 Workspace가 재사용한다. 저장 지도 선택은 조회이며 로봇 소속이나 map_server 전환으로 해석하지 않는다. |
| 지도 위치/궤적 | fresh map pose·검증 지도·Localization CONFIRMED·버전 일치가 없으면 위치 마커를 숨김. 궤적은 신선하고 정합된 표본만 추가하고 지도 버전 변경 시 초기화. 실제 소속 건물/층은 UNKNOWN. |
| 전역 상태바 | desktop 실제 높이 34px. 연결·Gate 보고·이동 차단/미확인·Reconciliation 항상 표시. 운용 모드·현재 임무는 상태 상세에 보존. Fleet Fault 요약도 헤더에 유지. Gate NORMAL을 허가로 표시하지 않음. |
| Spatial | Building/Floor/Driving 유지. 보기 도구를 접이식 메뉴에 통합. Driving 카메라 접기/펼치기, 24~45% 너비 슬라이더, 기존 영상 확대 유지. RGB 실제 별도 토큰/신선도 경로 재사용, Depth/Plan 미제공 유지. |
| 기존 전용 화면 | Maps Registry 등록·검증 이력, Fleet/RMF 이력·취소·복구, Safety의 관리자 복구, Data/History의 원문 진단, Settings/Documentation 보존. |

## 승인 시안과 구현 차이

시안은 정적 snapshot이었으나 구현은 기존 polling API로 갱신한다. 시안의 비활성 Lease/Goal 동작은 기존 실제 REST로 연결했다. Operation 새 목표는 접근성 있는 modal Drawer로 구현했으며 좌표 확인 체크를 추가했다. 기존 ID·진단·강제 Lease 해제·맵 초기 위치 미리보기 등은 보존을 위해 메뉴와 상세에 남겼다. Overview의 빠른 취소는 Operation의 현재 Goal 영역으로 이동했다. 작은 화면에서는 Overview의 보조 운영 제어 카드를 숨기지만 동일 기능은 Operation/Maps/Safety에서 접근한다. 자유로운 패널 드래그·사용자 Workspace 저장은 구현하지 않았다.

## 실제 Chromium 캡처

1920×1080, 합성 loopback 데이터이며 실차 검증이 아니다.

| 화면 | Light | Dark |
|---|---|---|
| Overview | [Light](workspace-live-overview-light-20261117.png) | [Dark](workspace-live-overview-dark-20261117.png) |
| Spatial / Floor | [Light](workspace-live-spatial-light-20261117.png) | [Dark](workspace-live-spatial-dark-20261117.png) |
| Operation | [Light](workspace-live-operation-light-20261117.png) | [Dark](workspace-live-operation-dark-20261117.png) |

[실제 목표 Drawer](workspace-live-goal-drawer-20261117.png), [격리 API 임무 생성 후 조회](workspace-live-api-mission-20261117.png), [Fault](workspace-live-fault-20261117.png), [Offline](workspace-live-offline-20261117.png), [Reconciliation](workspace-live-reconciliation-20261117.png), [Driving 1366](workspace-live-driving-1366-20261117.png).

## 검증 결과

- PASS: 새 runtime Chromium 10개 시나리오 그룹. 실제 API로 Lease 획득, Drawer 확인 전 submit 비활성, 확인 후 `POST /missions` 201, dashboard/Inspector 조회, 실제 `POST /cancel` 202. Nav2 실행/취소 완료를 검증했다는 뜻은 아니다.
- PASS: Light/Dark × Overview/Spatial/Operation × 1920×1080·1600×900·1366×768에서 문서 가로/세로 overflow 없음. status bar 실측 34px. Inspector/목록/Drawer는 내용이 길면 내부 스크롤.
- PASS: 로봇·폼 대상·지도·Inspector 공유 선택, 임무 목록 선택, 건물/층/버전 유지와 미리보기 Goal 차단, 지도 클릭 좌표/zoom/pan/Fit, Overview/Spatial 읽기 전용.
- PASS: Localization UNKNOWN 주입 시 위치 미확정 문구 및 canvas 로봇 마커 픽셀 없음. Fault/Offline/Reconciliation 상태와 경고가 1366 및 1920에서 넘침 없이 유지.
- PASS: viewer 쓰기 버튼 차단, operator 실제 명령 흐름, admin 기존 복구 버튼 접근 유지. 실제 관리자 복구 REST 경계는 기존 Safety Context 회귀에서 확인.
- PASS: 카메라 접기/크기/영상 확대 3해상도, System/Light/Dark 선택·저장·OS 변경·새로고침 복원 및 기존 탭 렌더링.
- PASS: JS helper/Registry upload 두 suite. 기존 Python Safety Context 24, static HTTP 2, Registry HTTP 4, ROS adapter 취소 callback 14 = 44 tests. callback 시험은 ROS 노드나 모터를 시작하지 않는다. 기존 Safety Context suite에 lease epoch/만료/복구 거부·rollback·재실행 차단 검사가 포함된다.
- 초기 발견 후 수정: Drawer DOM이 스크립트보다 늦게 파싱되어 초기화가 실패한 문제, 숨긴 Operation 패널의 display 우선순위로 다른 화면 밑에 노출된 문제. 수정 후 runtime/테마/좌표 회귀를 통과했다.
- SKIP/환경 오류: 추가 실행을 시도한 legacy `test_recovery.py`와 `test_ownership.py`는 sandbox 소켓 권한 오류로 유효한 assertions를 실행하지 못했다. Ownership 파일은 독립 실행 스크립트라 unittest import에도 맞지 않는다. 이 실행을 PASS로 집계하지 않았다. 최신 Safety Context 및 loopback runtime 검증 결과와 구분한다.
- SKIP: 실제 RGB/Depth 프레임 수신, Nav2 Plan, 건물·층 소속 증거, 실차/Gazebo. 없는 상태·경로·영상을 생성하지 않았다. 완전한 접근성/모든 데이터 길이에 대한 대비 검사는 미실행.

[브라우저 결과](workspace-runtime-results-20261117.json), [현재 static 해시](workspace-source-hashes-20261117.json).

## 실행 및 재현

저장소 루트에서:

```bash
python3 fleet_control/tests/dashboard_preview.py --info-file /tmp/workspace-unique.json
# 출력 URL 접속 → 출력된 operator.token 파일 내용으로 로그인
NODE_PATH=/home/cjinw/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules node fleet_control/tests/dashboard_workspace_runtime.cjs /tmp/workspace-unique.json /tmp
# camera 및 theme 검증은 같은 fixture의 읽기 전용 UI로 실행 가능
NODE_PATH=/home/cjinw/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules node fleet_control/tests/dashboard_workspace_camera.cjs /tmp/workspace-unique.json
NODE_PATH=/home/cjinw/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules node fleet_control/tests/dashboard_theme_browser.cjs /tmp/workspace-unique.json /tmp
node fleet_control/tests/test_static_dashboard.mjs
node fleet_control/tests/test_registry_upload.mjs
python3 -m unittest discover -s fleet_control/tests -p 'test_safety_context_v1_regression.py' -q
python3 -m unittest discover -s fleet_control/tests -p 'test_static_dashboard.py' -q
python3 -m unittest discover -s fleet_control/tests -p 'test_map_registry_http_e2e.py' -q
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s fleet_control/tests -p 'test_cancel_results.py' -q
```

Node 경로는 이 환경의 Playwright 위치다. loopback 소켓·Chrome 실행 권한이 필요하다. runtime 테스트는 임무 생성과 장애 시나리오를 주입하므로 **각 실행은 새 fixture DB**를 사용한다. 기존 legacy 브라우저 스크립트의 직접 폼 입력 시나리오는 Drawer 구조로 대체했고 기존 파일은 덮어쓰지 않았다.

## 변경 파일 및 보호

- 실제 UI: `static/index.html`, `style.css`, `app.js`, `map_view.js`.
- 새 검증: `tests/dashboard_workspace_runtime.cjs`, `dashboard_workspace_camera.cjs`.
- 보고·실제 캡처·결과·해시.
- System/Light/Dark 테마 시스템, 영상 소스 및 서버 REST 계약 보존. `server` 안전 코드, Agent UUID/Journal, Gate r8, Open-RMF Adapter, 인증/RBAC/CSP 변경 없음. 실차/Gazebo 미구동. Commit/push/PR 없음.
- 공통 문서는 해당 UI 결과만 append. 기존 Gate/다른 담당자의 본문은 수정하지 않았다.
