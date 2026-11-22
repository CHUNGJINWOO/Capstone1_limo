# LIMO Fleet Control 전체 UI 리디자인

## 레퍼런스 조사와 적용 범위

- [Open-RMF Web 소개](https://osrf.github.io/ros2multirobotbook/rmf-web.html), [공식 저장소](https://github.com/open-rmf/rmf-web): 지도, 시스템 개요, 작업 화면의 분리와 Fleet·로봇별 상태 접근을 참고했다. 이 프로젝트 Fleet Server는 RMF Core 직접 feed와 graph API가 없으므로 이를 생성하거나 모방하지 않는다.
- [InOrbit 문서](https://www.inorbit.ai/docs): Locations, Fleet, Robot, Navigation을 각 목적의 대시보드로 구분하는 방식을 참고했다. 이 프로젝트는 Registry 목록 및 실제 수신 상태만 사용하며 로봇의 건물·층 소속을 임의 추정하지 않는다.
- [Foxglove 시각화 문서](https://docs.foxglove.dev/docs/visualization): 좌우 사이드바와 데이터 소스·패널 설정 분리, 지도/영상 중심 화면을 참고했다. 프로젝트에 없는 데이터 소스나 동기화 기능을 만들지 않았다.

기존 소스와 레퍼런스 텍스트/공식 예시 화면을 검토했다. 외부 서비스 로그인 화면이나 보호 자산을 복사하지 않았다. 기존 공통 자산·API와 Vanilla HTML/CSS/JS를 재사용했다.

## 기능 매핑

| 화면 | 보존 및 변경 기능 |
|---|---|
| Overview | 선택 로봇 연결·Gate 보고·현재 임무·지도·빠른 임무/취소·최근 이벤트, 상세 이동. `SIMULATED`는 Agent 보고값일 때만 표기 |
| Spatial Monitor | Building/Floor/Driving, 건물·층·Registry 저장 지도 미리보기, 라이브 지도·위치·목표·수신 궤적, RGB·Depth 미수신, Fit/Zoom/레이어/전체화면. 공간 지도 클릭은 조회 전용 |
| Operation | 현재 Goal·실제 상태 단계, 명령 출처·UUID·Lease·임무 생성 및 사전 점검. 빠른 취소도 Overview에 보존 |
| Maps & Mapping | 기존 라이브/저장 지도, Registry 등록·검증 이력, 미구현 맵핑 버튼 비활성 |
| Fleet / RMF | Fleet 로봇 선택, 비교 테이블, RMF 상태 및 임무 이력·취소/복구 |
| Safety | Gate 상세·차단 원인·Reconciliation 및 관리자 복구. 원문 진단은 History의 접힌 상세 |
| Data & History / Sensors & AI | 기존 RGB/AI 연결·별도 토큰, 센서 및 진단 상태, Depth/LiDAR 미제공 표시 |
| Data & History / History & Diagnostics | 이벤트·명령 감사·Safety Context/Journal 원문(서버 미제공값은 UNAVAILABLE) |
| Settings / Documentation | 기존 Pre-flight·미지원 설정/전원 제어와 운영 매뉴얼 |

Settings와 Documentation은 사이드바 하단에 구분했다. 중요 로봇 연결·Gate·이동·임무·재조정은 전역 상단 상태바에 남는다. Gate NORMAL 보고와 이동 승인을 구분하고, 독립 정지 API 미구현은 물리 E-Stop으로 명명하지 않는다. 서버·Agent·Gate·RMF 로직, API, CSP와 기존 권한 정책은 변경하지 않았다.

## 디자인 시스템

짙은 남색 배경과 동일한 표면·테두리·간격, 접이식 단일 열 사이드바, 활성 파랑, 정상 녹색, 경고 노랑, Fault 빨강, 미확인 회색을 사용한다. 상태는 텍스트와 색상을 함께 사용한다. Status/Robot 선택은 고정 상태바, Mission은 상태 단계와 요약, Map은 접힌 레이어 메뉴, Timeline은 실제 이벤트, 상세는 기존 details, 미수신은 UNAVAILABLE을 사용한다. 단계는 **현재 보고 상태만** 강조하며 완료되지 않은 단계를 추정하지 않는다.

Building View는 층별 이미지 영역을 확대했다. 층이 하나라도 큰 지도 미리보기를 보여주며, 층이 여럿이면 그리드로 배치한다. 가짜 KPI·건물 모델·카메라 영상·경로를 생성하지 않았다.

## 캡처

모두 임시 fixture 합성 데이터. 아래는 최종 1920×1080 화면이며 긴 상세 페이지는 전체 페이지 캡처다. 이전 실제 캡처: [Overview 변경 전](ui-phase2-overview.png), [Building 변경 전](spatial-view-building-1920.png).

[Overview](redesign-overview.png) · [Spatial/Floor](redesign-spatial.png) · [Building](redesign-building.png) · [Driving](redesign-driving.png) · [Operation](redesign-control.png) · [Maps](redesign-maps.png) · [Fleet/RMF](redesign-rmf.png) · [Safety](redesign-safety.png) · [Sensors](redesign-media.png) · [History](redesign-history.png) · [Settings](redesign-settings.png) · [Documentation](redesign-docs.png).

각 단계(A~E)를 독립 체크포인트로 구현·캡처하지 않았으므로 단계별 중간 캡처는 SKIP이다. 기존 캡처와 최종 전체 화면 비교만 제공한다.

## 검증과 한계

- PASS: Chromium 주요 10개 화면 × 1920×1080·1600×900·1366×768 가로 넘침 없음, 단일 열 메뉴, 뒤로 가기·사이드바 접기·지도/로봇 선택 유지, 미리보기 Goal 차단, admin 복구 버튼 접근 및 JS pageerror 없음. 전 화면에 대해 세로 스크롤이 없다는 뜻은 아니며 긴 상세 화면은 세로 스크롤한다.
- PASS: 기존 Chromium 23 checks: viewer 제어 차단, operator Lease 만료·획득 및 임무 생성/취소, Fault·Gate 통신 중단·Attestation 없음·미검증 지도·Offline·Reconciliation·서버 단절. 390px 모바일 overview 가로 넘침 없음.
- PASS: 공간 세 모드, Registry 썸네일, 실제 상태 미확정 및 조회 전용 클릭, Fault/Offline/미확정, 두 해상도 화면 넘침 없음.
- PASS: JS helper·Registry 2 suites; Python Safety Context 24, Fleet HTTP 2, Registry HTTP 4 tests.
- 초기 1366px overview 6px, 모바일 헤더 가로 32px, 저장 지도 경고가 열린 공간 화면 2px 넘침을 발견해 수정 후 재검증했다.
- SKIP: 단계별 중간 스크린샷/각 단계 별 회귀, 실제 RGB/Depth/계획 경로, 실차 및 Gazebo. 실제 Depth와 계획 경로 계약은 [후속 요청](SPATIAL_MONITOR_INTERFACE_REQUEST.md)에 남아 있다. 가짜 정상 상태나 데이터를 대신하지 않았다.

## 재현

```bash
python3 fleet_control/tests/dashboard_preview.py --info-file /tmp/fleet-ui-unique.json
# 출력된 loopback 주소와 operator.token 사용
NODE_PATH=/home/cjinw/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules node fleet_control/tests/dashboard_redesign_browser.cjs /tmp/fleet-ui-unique.json /tmp
```

Node Playwright 경로는 이 환경의 예시다. 브라우저 테스트는 임시 fixture DB와 토큰만 사용해야 한다. 추가 검증: `dashboard_browser_phase2.cjs`, `dashboard_spatial_modes.cjs`, `test_static_dashboard.mjs`, `test_registry_upload.mjs` 및 Python 테스트. 실차/운영 서버를 실행하지 않았다.

변경: `static/index.html`, `style.css`, `app.js`, 신규 `tests/dashboard_redesign_browser.cjs` 및 이 보고와 캡처. 전체 `fleet_control/`은 시작 전부터 미추적이므로 Git diff만으로 변경 파일을 판정하지 않는다. 다른 에이전트의 Gate 작업을 변경하지 않았다.
