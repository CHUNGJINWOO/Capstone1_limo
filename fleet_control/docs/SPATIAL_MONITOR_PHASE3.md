# Spatial Monitor 3차 UI — 2026-11-16

기존 Fleet HTML/CSS/JS에 공간 관제 탭을 추가했다. 지도와 RGB 패널은 기존 인스턴스를 이동해 로봇·건물·층·버전 및 영상 연결 상태를 유지한다. 전역 Gate·이동·임무·재조정 상태는 유지한다. Overview 개념도는 기본적으로 숨겼다. 공간 탭은 지도 클릭으로 목표를 설정하거나 제출하지 않는다. Operation, Maps, Safety, Fleet/RMF, Sensors, History와 상세 기능은 보존한다.

- 대형 지도: Registry 층과 버전 조회, 라이브/저장 미리보기 구분, 기존 pose·방향·목표·브라우저 세션의 실제 수신 궤적. 선택 층과 로봇 실제 소속은 다르므로 실제 건물/층은 UNKNOWN. 위치는 online·2초 이내·map pose OK·지도 검증·동일 localization 버전·CONFIRMED일 때만 요약하고 그렇지 않으면 미확정.
- Fit to Floor는 occupancy 이미지 전체 경계와 해상도/origin/yaw 사용. 종횡비 유지. 실제 점유 영역만의 bounding box는 지원하지 않는다. Fit to Robot은 신선한 map pose가 있을 때만 이동하며 미리보기에서는 동작하지 않는다. Fit to Route는 실제 계획 API 부재로 비활성. Reset, 줌, Fullscreen, 기본 접힌 레이어 제공. 건물 전체 통합 및 3D는 데이터 부재로 미구현.
- RGB는 기존 별도 영상 서버, viewer 토큰, stale overlay 사용. 영상 시각/나이와 연결 상태 제공. Depth는 Fleet 영상 API 부재로 UNAVAILABLE: 단위/표시 범위/컬러맵 미확인. 플랜·Depth를 별도 게이트웨이에서 가져오는 보안·데이터 계약은 [후속 요청](SPATIAL_MONITOR_INTERFACE_REQUEST.md).

## 검증

- PASS: Chromium 격리 fixture에서 1920×1080·1366×768 지도/RGB/Depth 동시 표시와 문서 overflow 없음; 지도 캔버스 실제 픽셀, 레이어, 확대/축소, Floor/Robot/Reset, Fullscreen, 지도 클릭 조회 전용, 미리보기 제출 차단, 로봇·지도 선택 유지, Depth 미수신, JS pageerror 없음.
- PASS: 기존 JS helper/Registry 두 suite, Python Safety Context 24, Fleet HTTP 2 및 Registry HTTP 4 tests. 2차 Chromium 회귀 23 checks(임무·Lease·RBAC·Fault·Offline·Reconciliation 포함) PASS.
- SKIP: 실제 RGB 송신·Depth 송신 시나리오 및 실제 계획 수신·프레임 불일치 시나리오(Fleet 인터페이스 부재), 점유영역만 맞춤, 건물 전체 합성, 실차·Gazebo. 이 항목은 완료로 주장하지 않는다.
- 브라우저 캡처 중 빈 지도 회귀와 3px overflow를 발견해 수정하고 캔버스 픽셀 검사를 추가했다.

[1920×1080](spatial-monitor-1920.png) · [1366×768](spatial-monitor-1366.png). 합성 fixture 화면이다.

실행:

```bash
python3 fleet_control/tests/dashboard_preview.py --info-file /tmp/spatial-unique.json
# 출력되는 주소와 operator.token 사용
NODE_PATH=/home/cjinw/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules node fleet_control/tests/dashboard_spatial_browser.cjs /tmp/spatial-unique.json /tmp
```

변경: `static/index.html`, `style.css`, `app.js`, `map_view.js`, `video_panel.js`, 신규 `tests/dashboard_spatial_browser.cjs`와 이 보고·인터페이스 요청·캡처. Fleet API·Agent·Gate·RMF·인증/CSP 코드는 변경하지 않았다.
