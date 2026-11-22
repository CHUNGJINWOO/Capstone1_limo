# Spatial Monitor — Building / Floor / Driving

기존 지도와 Registry 및 RGB 패널을 재사용하는 세 가지 조회 전용 모드를 구현했다. 최초 진입은 Floor다. 모드를 전환해도 로봇·건물·층·버전은 유지된다. 전역 Safety/이동/재조정 상태는 모든 모드에 유지된다.

- Building: Registry 시설·건물·층 목록과 저장 지도 미리보기(인증된 API로 수신), 검증 상태를 표시한다. 층 선택 시 저장 지도 미리보기를 열고 Floor로 이동한다. 로봇의 실제 건물/층, 구역 이름 및 건물 간 경로는 API 부재로 UNKNOWN/미제공이다. 지도 썸네일은 가짜 3D가 아니다.
- Floor: 지도 전체, 실제 Fleet pose·목표·세션 수신 궤적을 기존 렌더러로 표시한다. 지도/Localization 증거 부족 시 위치는 미확정이며 공간 지도 위치 마커를 숨긴다. 저장 지도 미리보기와 라이브 보고는 분리한다. 회전과 레이어는 기본 접힌 메뉴. Nav2 계획은 Fleet API가 없어 표시하지 않는다.
- Driving: 지도와 RGB/Depth 영역을 65:35 정도로 배치하고 영상/지도 확대 모드를 제공한다. RGB는 기존 독립 토큰 영상 서버를 재사용한다. Depth API는 연결되지 않아 UNAVAILABLE, 단위·컬러맵·표시 범위 미확인이다. 동기화나 보정도 주장하지 않는다.

[Building](spatial-view-building-1920.png), [Floor](spatial-view-floor-1920.png), [Driving 1920](spatial-view-driving-1920.png), [Driving 1366](spatial-view-driving-1366.png), [Fault](spatial-view-fault.png), [Offline](spatial-view-offline.png), [Localization 미확정](spatial-view-localization-unknown.png). 모두 합성 fixture. Driving 캡처는 저장 지도 UNVERIFIED 미리보기이며 실제 지도 전환이나 Goal 제출을 의미하지 않는다.

## 참고 자료

화면 구성 원칙은 [InOrbit 문서](https://www.inorbit.ai/docs), [RMF Web](https://osrf.github.io/ros2multirobotbook/rmf-web.html), [RMF Web 저장소](https://github.com/open-rmf/rmf-web), [Traffic Editor](https://osrf.github.io/ros2multirobotbook/traffic-editor.html), [Foxglove AMR](https://foxglove.dev/solutions/autonomous-mobile-robot), [Foxglove 3D 패널](https://docs.foxglove.dev/docs/visualization/panels/3d)을 참고했다. 제품 자산이나 UI를 복제하지 않았다.

## 검증

- PASS: 격리 Chromium에서 기본 Floor, Building 실제 Registry 썸네일/검증 상태, 층 선택·미리보기, 모드 전환·선택 유지, 읽기 전용 클릭, 영상 확대, Fault·Offline·Localization UNKNOWN, pageerror 없음. 1920×1080 및 1366×768 페이지 가로/세로 overflow 없음.
- PASS: 기존 JS helper/Registry 2 suites, Python 안전 24, Fleet HTTP 2, Map Registry HTTP 4 tests.
- PASS: 새 임시 DB에서 기존 UI Chromium 회귀 23 checks(임무 생성·취소, Lease, RBAC, 안전 포함). 먼저 Fault/Offline 주입 이력이 남은 DB에서 임무 생성이 409로 거부된 실패가 있었고, 깨끗한 별도 DB에서 재실행해 통과했다.
- SKIP: 실제 RGB·Depth·Nav2 계획 수신, 실제 로봇 위치·건물 소속, 실차/Gazebo. Fleet API에 없는 값이며 인터페이스 후속 요청은 [SPATIAL_MONITOR_INTERFACE_REQUEST.md](SPATIAL_MONITOR_INTERFACE_REQUEST.md)에 있다.

## 실행

```bash
python3 fleet_control/tests/dashboard_preview.py --info-file /tmp/spatial-views-new.json
# 출력 주소 및 operator.token 파일 사용
NODE_PATH=/home/cjinw/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules node fleet_control/tests/dashboard_spatial_modes.cjs /tmp/spatial-views-new.json /tmp
```

수정: `server/fleet_server/static/{index.html,style.css,app.js}`, 신규 `tests/dashboard_spatial_modes.cjs`, 보고·캡처. 서버/Agent/Gate/RMF, 인증과 CSP는 변경하지 않았다. 공통 문서에는 이 결과만 추가했다.
