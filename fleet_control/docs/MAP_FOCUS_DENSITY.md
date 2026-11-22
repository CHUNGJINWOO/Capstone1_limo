# 지도 집중 모드 및 정보 밀도 개선

2026-11-16. 기존 공유 지도 캔버스와 Registry·선택 상태를 유지하면서 설정을 기본 접힘으로 변경했다. 설정을 펼치면 건물·층·지도 버전·새로고침·회전·레이어를 사용할 수 있고, 접으면 선택 요약·설정·Fit·레이어·전체화면만 지도 위에 남는다. Spatial 고유 Fit/로봇 중심/줌 등도 보존된다. 설정 접힘 상태는 탭 전환 시 유지되며 로봇·지도 선택과 독립적이다. 본문 지도 패널은 자체 스크롤 대신 사용 가능한 영역에 캔버스를 맞춘다.

Overview와 Spatial에서 지도 클릭은 읽기 전용이다. Maps에서는 기존 목표 좌표 지정·Operation 전환을 유지하며 최종 임무 생성은 기존 Operation/RBAC/서버 안전 검사를 따른다. Registry 저장 지도 미리보기는 기존 Goal 제출 차단을 유지한다.

Operation의 안전 점검 목록은 요약 배지와 접이식 상세로 구성했다. Safety는 차단·재조정·미확인 상태를 최상단 요약에 표시하고 기존 상세와 관리자 복구를 유지한다. Fleet/RMF 설명, Sensors 영상 연결 안내, Settings 설명을 접어서 실제 목록·영상·설정의 우선순위를 높였다. History 원문은 기존 접이식 상세를 유지한다. 새 안전 허가 판정, 가상 그래프나 데이터는 추가하지 않았다.

## 실제 Chromium 캔버스 크기 (1920×1080, CSS 픽셀)

| 화면 | 설정 펼침 | 설정 접힘 | 면적 증가 |
|---|---:|---:|---:|
| Overview | 992 × 521.61 | 992 × 613.00 | 약 17.5% |
| Spatial / Floor | 1662 × 629.73 | 1662 × 721.13 | 약 14.5% |
| Maps | 1646 × 655.22 | 1646 × 778.00 | 약 18.7% |

너비와 높이는 canvas `getBoundingClientRect` 기준이다. [측정 JSON](map-focus-measurements.json) · [설정 펼침](map-focus-expanded.png) · [설정 접힘](map-focus-collapsed.png). 두 이미지는 동일 viewport와 합성 테스트 지도다.

## 검증

- PASS: Chromium 세 지도 화면에서 실제 접힘 전후 높이 증가, 1920×1080·1600×900·1366×768에서 지도 패널 넘침/내부 스크롤 없음. 1366에서 미검증 지도 경고와 설정을 동시에 펼친 경우도 확인.
- PASS: 설정 접힘과 건물·층·버전 유지, 미검증 저장 지도 미리보기의 임무 제출 차단, Overview/Spatial 클릭 조회 전용.
- PASS: 줌·팬·Fit·레이어·전체화면 및 Maps 클릭 좌표와 기존 커서 변환 일치, canvas 픽셀 렌더링 및 JS pageerror 없음. 기존 좌표 역변환 helper 테스트 PASS.
- PASS: 설정을 펼치도록 조정한 기존 Chromium 회귀 23 checks: Lease 만료/획득, 임무 생성·취소, viewer, Fault, Gate stale, Attestation 없음, Offline, Reconciliation, 서버 단절 등.
- PASS: 기존 JS helper/Registry upload 2 suites, Python Safety Context 24·Fleet HTTP 2·Registry HTTP 4 tests.
- SKIP: 실차·Gazebo·실제 센서와 Nav2 주행. 테스트는 임시 루프백 Fleet Server·합성 데이터로만 실행.

과거 Chromium 스크립트 일부는 지도 설정이 항상 열려 있다고 가정한다. 기존 시나리오를 보존하면서 `dashboard_focus_regression.cjs`에 설정 열기 동작을 추가해 실행했다. 기존 파일은 덮어쓰지 않았다.

## 재현

```bash
python3 fleet_control/tests/dashboard_preview.py --info-file /tmp/limo-focus-unique.json
# 출력된 loopback URL과 operator.token 사용
NODE_PATH=/home/cjinw/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules node fleet_control/tests/dashboard_map_focus.cjs /tmp/limo-focus-unique.json /tmp
# 회귀는 장애 이력이 없는 별도 새 fixture로 실행
NODE_PATH=/home/cjinw/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules node fleet_control/tests/dashboard_focus_regression.cjs /tmp/limo-focus-unique.json /tmp
```

변경 파일: `static/index.html`, `style.css`, `app.js`; 신규 `tests/dashboard_map_focus.cjs`, `dashboard_focus_regression.cjs`, 보고·캡처·측정 JSON. `map_view.js` 좌표 변환, 서버/Agent/Gate/RMF, 인증/RBAC/CSP는 변경하지 않았다.
