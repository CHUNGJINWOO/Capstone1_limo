# Fleet Control UI 개편 — 2026-11-16

Vanilla HTML/CSS/JS 실제 대시보드 개편. 승인 이미지 파일은 이번 대화/입력에서 제공되지 않아 **요청된 텍스트 구조 기준**으로 구현했다. 이미지와의 픽셀 단위 일치 검증은 SKIP이다. 모든 캡처는 `UI-FIXTURE-*` 모의 로봇과 합성 지도이며 실차 운용 증거가 아니다.

## 구현 및 유지

- 고정 헤더·상태바: 로봇 선택, 모드, Gate 보고, 이동 미확인/차단, 현재 임무, 재조정. Attestation 생산자 부재 때문에 UI가 이동 허가를 확정하지 않는다.
- 왼쪽 로봇 개념도·전압·센서·Gate 요약. 실제 API 미제공인 속도·CPU·메모리·온도·IMU는 미수신/UNAVAILABLE. 전압을 배터리 %로 변환하지 않는다. 영상 송신기의 CPU/RAM은 기존 영상 설정 상세에 그대로 남는다.
- 중앙 2D 지도: 기존 좌표 변환, 지도 이미지, 위치·방향, 목표, 궤적, 회전·확대·축소·맞춤, 건물·층·버전 선택 보존. ResizeObserver로 캔버스 크기 일치, 숨겨진 탭에서 크기 초기화 방지, 맞춤 모드의 자동 재맞춤 적용.
- 오른쪽 현재 임무 ID·Goal UUID·Command ID·목표·생성 후 경과·보고된 남은 거리. UUID는 `execution.active`와 같은 Mission일 때만 사용하며 없으면 UNAVAILABLE. 경과는 주행 시간이 아닌 **생성 후** 시간이다. ETA/진행률은 생성하지 않는다.
- 센서 탭: 기존 RGB/YOLO 영상 요소와 연결·독립 토큰·신선도 표시 보존. LiDAR/Depth 영상은 Fleet API 미제공으로 UNAVAILABLE. 별도 `dashboard/` 게이트웨이의 API를 임의로 관제 API로 합치지 않았다.
- 하단: 임무 입력 화면 이동, 실제 Goal 취소 요청, 지도 관리·복구 화면 이동, 최근 이벤트 최대 5건(선택 로봇/공통 이벤트, 날짜·시각·종류·아이콘). 전체 로그와 RMF 이력 유지.
- 상세 탭: 임무/Lease, 지도 등록·이력, 운영/Pre-flight/관리자 복구, History/RMF, 영상 설정/감사 로그, 시스템 상세/수신 원문/운영 매뉴얼. 모든 기존 DOM ID 보존 및 중복 없음 확인.
- 독립 정지 API 없음: **정지 요청 · 미지원**으로 비활성. 물리 E-Stop으로 표시하지 않는다. 맵핑 제어·3D·초기 위치 전송·프로파일 제어도 미지원 상태 보존.
- 서버 API, 인증/RBAC/CSP, Agent UUID·Journal, Gate 후보, RMF Adapter는 변경하지 않았다. 실제 로봇·운영 서버에 접속/구동하지 않았다. Commit/push/PR/폴더 이동·삭제 없음.

## 직접 확인

저장소 루트에서:

```bash
python3 fleet_control/tests/dashboard_preview.py
```

출력되는 `http://127.0.0.1:<자동할당 포트>`를 브라우저에서 연다. 출력된 임시 `.../tokens/operator.token` 파일 내용을 로그인 입력창에 넣는다. viewer/admin 토큰도 같은 폴더에 생성된다. 토큰 값은 로그나 문서에 기록하지 않는다.

이 스크립트는 새 임시 DB·Map Registry·무작위 토큰을 만들고 **loopback/자동할당 포트만** 사용한다. ROS 노드·Agent·실차를 시작하지 않는다. 합성 상태 생산자와 실제 Fleet HTTP API로 화면·제어를 시험한다. 종료는 실행 터미널에서 Ctrl+C. 기본 info 파일이 이미 있으면 덮어쓰지 않고 종료하며 `--info-file /tmp/새이름.json`을 사용한다.

시나리오는 출력 info JSON의 `scenario` 파일에 `normal`, `fault`, `gate-stale`, `no-attestation`, `unverified`, `offline`, `reconciliation`을 기록해 변경한다. Reconciliation은 의도적으로 잠금이 유지되므로 깨끗한 초기 상태는 새 fixture 실행으로 확인한다. 실제 API/로봇 주소로 스크립트를 바꾸지 않는다.

## 재현 명령

```bash
node fleet_control/tests/test_static_dashboard.mjs
node fleet_control/tests/test_registry_upload.mjs
python3 -m unittest discover -s fleet_control/tests -p 'test_static_dashboard.py' -v
python3 -m unittest discover -s fleet_control/tests -p 'test_map_registry_http_e2e.py' -v
python3 -m unittest discover -s fleet_control/tests -p 'test_safety_context_v1_regression.py' -v
# 콜백만 시험하며 ROS 노드나 모터를 시작하지 않음
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s fleet_control/tests -p 'test_cancel_results.py' -v
# Playwright가 설치된 Node 환경; CHROME_BIN으로 Chrome 실행 파일 지정 가능
node fleet_control/tests/dashboard_browser.cjs /tmp/limo-dashboard-preview.json /tmp
```

이 환경의 브라우저 실행은 번들 Playwright 모듈 디렉터리를 `NODE_PATH`에 지정했다. 별도 프레임워크/앱 의존성은 추가하지 않았다. HTTP 시험에는 루프백 소켓 실행 권한이 필요하다.

## 검증 결과

| 항목 | 결과 | 범위 |
|---|---|---|
| 모든 JS 구문·기존 DOM ID 보존·ID 중복 | PASS | static JS 전체, 기존 ID 전체 |
| 기존 JS helper/Registry upload | PASS | 2개 suite |
| 실제 Fleet HTTP 대시보드 회귀 | PASS | 2 tests |
| Map Registry HTTP 조회·인증·격리·무결성 | PASS | 4 tests |
| Safety Context v1 회귀 | PASS | 24 tests |
| 실제 Agent 취소 콜백의 합성 Future 시험 | PASS | 14 tests; 최초 ROS 환경 미로딩 오류 후 Humble 환경 로딩하여 통과 |
| Chromium E2E | PASS | 23 checks; [결과](ui-20261116-browser-results.json) |
| 1920×1080 / 1600×900 / 1366×768 | PASS | 메인 문서 세로·가로 overflow 없음; 작은 패널은 내부 스크롤 |
| 390×844 | PASS | 세로 배치·탭; 문서 가로 overflow 없음 |
| 1536×864 / 1280×720 / 1024×768 | PASS | 확대 대응 유효 viewport 및 작은 창 가로 overflow 없음; 1024에서는 세로 배치/스크롤 허용 |
| 브라우저 실제 메뉴 확대 125%/150% | SKIP | 임시 Chrome 프로필 설정이 적용되지 않아 실제 확대 검증으로 인정하지 않음; 위 유효 viewport 시험만 인정 |
| Fault·Gate stale·Attestation 없음·미검증 지도·Offline·Reconciliation | PASS | 합성 상태의 차단/미확인 표시, 캡처 |
| Lease 만료·viewer 권한·서버 단절 | PASS | Lease 5s 실제 만료, 쓰기 버튼 차단, 네트워크 실패 후 UNKNOWN/OFFLINE |
| 임무 생성·취소 | PASS | 실제 테스트 API 201/202; 물리 정지나 Nav2 완료를 검증한 것은 아님 |
| 지도 확대·축소·맞춤·로봇 선택·탭 | PASS | 실제 Chromium 입력 |
| 콘솔/예상 밖 API 오류 | PASS | 예상 밖 오류 없음; 의도적 dashboard 네트워크 차단의 ERR_FAILED는 결과에 명시 |
| 실제 센서 스트림·Nav2·실차·운영 서버 | SKIP | 요청 범위에 따라 접속/구동하지 않음 |
| 승인 이미지 시각적 일치 | SKIP | 이미지 미제공 |

초기 검증에서 favicon 404와 1920 화면 12px overflow가 발견되어 수정했다. 캡처 검토 중 지도 맞춤 회귀도 수정 후 E2E를 다시 통과했다. 최종 미해결 FAIL은 없다. SKIP은 PASS에 포함하지 않는다.

## 스크린샷

- [1920×1080](ui-20261116-1920x1080.png), [1600×900](ui-20261116-1600x900.png), [1366×768](ui-20261116-1366x768.png), [모바일](ui-20261116-390x844.png)
- [임무 생성](ui-20261116-active-mission.png), [취소 요청](ui-20261116-cancel-requested.png)
- [Fault](ui-20261116-fault.png), [Gate 단절](ui-20261116-gate-stale.png), [Attestation 없음](ui-20261116-no-attestation.png)
- [미검증 지도](ui-20261116-unverified.png), [Offline](ui-20261116-offline.png), [Reconciliation](ui-20261116-reconciliation.png), [서버 단절](ui-20261116-server-offline.png)

## 변경 파일

- `server/fleet_server/static/index.html`, `style.css`, `app.js`, `map_view.js`, `video_panel.js`
- `tests/dashboard_preview.py`, `tests/dashboard_browser.cjs` 신규. 기존 테스트는 수정하지 않음.
- 이 보고서, 캡처 13개, 브라우저 결과 JSON 및 [검증 소스 해시](ui-20261116-source-hashes.json).
- `state_helpers.js`, `registry_upload.js` 및 모든 서버/Agent/Gate/RMF 구현은 보존.
- 공통 문서는 이번 UI 결과만 추가하며 기존 Gate/타 담당자 본문을 수정하지 않는다.

## 남은 API 연동 범위

속도/IMU/직접 하드웨어 상태, Nav2 Plan·Costmap·안전구역, LiDAR/Depth 영상, 인증된 Attestation/Journal 조회, 독립 정지 및 맵핑 제어는 이 Fleet API에 없거나 별도 게이트웨이에만 있다. UI에 임의 상태·임의 허가·가짜 영상을 넣지 않았다. 이들을 메인 UI에 연결하려면 별도 API/소스 선택 설계가 필요하며 이번 작업에서 서버 범위를 확장하지 않았다.
