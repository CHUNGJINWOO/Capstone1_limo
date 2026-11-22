# 공통 문서 갱신안 — 웹 UI·Registry 저작 (2026-11-20, 적용 전 제안)

공통 문서(CURRENT_STATUS, CODE_INDEX, VALIDATION, HANDOFF)는 이번 작업에서 **수정하지 않았다**. 승인 시 아래 항목을 해당 문서 끝에 추가한다. 근거·상세는 `fleet_control/docs/WEB_OPERATIONS_UI_20261120.md`.

## CURRENT_STATUS.md (표 추가)

| 항목 | 구현 | 검증 |
|---|---|---|
| 웹 맵핑 차단 사유 표시·명령 상태 7종·실차 차단 3곳 표시 | 구현(`mapping_panel.js`) | Node, 격리 ROS 2(22/22 + 버튼 E2E), 브라우저. 실차 맵핑은 차단 유지 |
| Registry 영역 생성 API·UI, 업로드 영역·계보, YAML 임계값 안전 검사(보정본 등록) | 구현 | Python 12 + 기존 49, Node, 브라우저(격리) |
| 오프라인 로봇 센서 행 "과거 표본" 표시 | 구현 | Node, 브라우저 |
| 휴대폰 레이아웃(375 px 가로 스크롤 제거) | 구현 | 에뮬레이션만, 실제 기기 미검증 |
| 18190 서버 | **이전 코드로 실행 중**(재기동 필요) | — |

## CODE_INDEX.md

- 서버: `server/fleet_server/map_registry.py`(`_prepare_web_upload`, `unknown_threshold_issue`, `derive_yaml_free_thresh`, `areas` 라우트), `http_api.py`(라우트 집합).
- UI: `static/mapping_panel.js`, `registry_upload.js`, `state_helpers.js`(`staleInfo`), `app.js`, `index.html`, `style.css`.
- 시험: `tests/test_registry_authoring_api.py`(신규), `tests/test_mapping_panel.mjs`(신규), `tests/test_registry_upload.mjs`(확장), `tests/test_map_registry_phase2b.py`(고정 입력 `free_thresh` 0.25→0.196).
- 문서: `docs/WEB_OPERATIONS_UI_20261120.md`, `docs/WEB_REGISTRY_AUTHORING_INVESTIGATION_20261120.md`, `docs/API.md`.

## VALIDATION.md

2026-11-20 저녁: 웹 UI 확장 검증. 서버 신규 12 + 기존 Registry 49 PASS, Node(mapping_panel·registry_upload·static_dashboard·goal_check) PASS, 격리 ROS 2 `probe.py` 22/22, 웹 버튼 START→PAUSE→SAVE→RESUME→STOP E2E, 브라우저에서 영역 생성·임계값 보정 등록·저장 결과·실차 차단 3곳·과거 표본·휴대폰 375 px. 미검증: 실패/타임아웃/거부 상태의 브라우저 실표시, 실제 휴대폰, 실차. 증거 `local_data/raw_results/web_ui_e2e_20261120/`.

## HANDOFF.md

- 새 UI·API는 `fleet_control/`에 있고 18190은 아직 재기동 전. 재기동: `fleet_control/scripts/run_field_server.sh local_data/field_run_20261120_b1/data 18190`(Agent 자동 재접속).
- 기본 임계값 정책이 `reject`로 바뀌어 `map_saver_cli` 기본 YAML(`free_thresh: 0.25`)은 웹 업로드 시 "보정본 등록"을 골라야 한다. 로봇에서 저장할 때 `--free 0.196`을 주면 보정이 필요 없다.
- 미구현: 실차에서 웹이 SLAM 저장을 시키는 작업(Agent 변경, 별도 승인), 지도 검증·활성화·복원.
- 이번 작업은 다른 에이전트 소유 파일을 수정하지 않았다.
