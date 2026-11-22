# 18190 웹 관제 확장: 맵핑 명령 상태·차단 사유·Registry 관리·오래된 값 표시 (2026-11-20)

범위는 `fleet_control/` 웹 UI·Registry API뿐이다. 실차 모터, Mapping 실차 차단 3곳, Gate·Watchdog, 지도 활성화, 운영 Registry, 다른 에이전트 소유 파일(`capstone_ws/` launch·Gate 등)은 건드리지 않았다. 변경 전 백업: `local_data/worktree_backups_20261119_c103_web/web_ui_before_20261120d.tar.gz`.

## 1. 구현한 것

| # | 기능 | 파일 |
|---|---|---|
| 1 | 맵핑 버튼이 **왜 비활성인지** 화면에 글로 표시(같은 사유는 묶어서). 툴팁에만 있던 사유를 본문으로 | `static/mapping_panel.js`, `index.html`(`#webMapBlocks`) |
| 2 | 실차 Agent(`mode=real` 또는 `simulated=false`)에는 **실차 맵핑 차단 3곳**(웹 버튼·Agent·서버 `mapping_real_blocked`)을 사실 그대로 표시. UI는 차단을 풀지 않는다 | 〃 |
| 3 | 명령 상태 구분: 전송 중 / 접수됨(Worker 수락 대기) / 실행 중 / 성공 / 실패 / 타임아웃·결과 미확인 / 서버가 거부함. 타임아웃까지 남은 시간 표시 | `mapping_panel.js`, `#webMapPhase` |
| 4 | 건물·층·영역·지도 버전 **선택 요약 한 줄**(검증 상태, 층 활성 여부, "선택은 미리보기이며 로봇 지도는 바뀌지 않음") | `app.js`, `#mapSelSummary` |
| 5 | **영역 생성**: `POST /api/v1/map-registry/areas`(operator/admin, 멱등) + UI 폼 | `map_registry.py`, `http_api.py`, `registry_upload.js`, `index.html` |
| 6 | **지도 업로드에 영역·계보·메모·임계값 정책** 추가(`area_id`, `lineage{mapping_mode,parent_map_version_id,session_id}`, `note`, `threshold_policy`). 영역은 같은 층만, 계보 규칙(CONTINUE는 부모 필요)은 저장 **전에** 검증하며 거부되면 아무것도 저장하지 않는다 | `map_registry.py`(`_prepare_web_upload`) |
| 7 | **YAML 임계값 안전 검사**(`register_saved_map.py`와 같은 규칙): trinary 지도에서 `free_thresh`가 unknown 픽셀(p=0.196)을 free로 읽게 하면 기본 **거부**(422 `unknown_pixels_misclassified`). `threshold_policy=derive`면 원본은 보존하고 `free_thresh`만 고친 **보정본**을 등록하며 원본·보정본 해시와 사유를 증거에 기록. 클라이언트가 보낸 원본 체크섬은 보정 전에 먼저 검증 | `map_registry.py`, `registry_upload.js`(`thresholdCheck`) |
| 8 | **저장 결과 확인**: 등록 후 서버가 저장한 증거(위치 선언, 영역, 계보, 확인된 칸·면적, 보정 여부와 해시, 업로더·메모, 체크섬 일치, UNVERIFIED·층 활성 아님) 표시 | `registry_upload.js`(`evidenceRows`) |
| 9 | **오래된 값 표시**: 서버는 오프라인 로봇의 마지막 상태를 보고 시점의 `age_s` 그대로 돌려준다. 센서 행(배터리·LiDAR·Depth·위치·Localization·지도·Nav2)마다 `[과거 표본 · N초/분 전]`을 붙이고 회색 처리, 표시 age에 경과 시간을 더함 | `state_helpers.js`(`staleInfo`·`shownAge`), `app.js` |
| 10 | **휴대폰 레이아웃**: 375 px에서 가로 스크롤 제거(문서 폭 692→375 px), 상태 줄 겹침·잘림 해소, 입력·버튼 세로 배치와 44 px 터치 높이, 새 폼 반응형 | `style.css` |

## 2. 하지 않은 것 (의도적)

- 실차 Mapping 차단 3곳 해제, Mapping Worker 실차 허용, 지도 클릭 Goal 경로 변경(권한·안전 경계 그대로).
- 지도 검증·활성화·복원 라우트(정책 미결정 → 계속 404).
- 실차에서 SLAM 지도를 웹이 **저장시키는** 작업(Agent 변경 필요, 별도 승인). 오늘처럼 저장 후 웹 업로드(또는 스크립트)로 등록한다.
- 운영 Registry 변경, 18190 서버 재기동(아래 §5).

## 3. API 변경

`POST /api/v1/map-registry/areas` — 본문 `{area_id, floor_id, name, coverage: PARTIAL_FLOOR|FULL_FLOOR, description?}` → 201 `{ok, area}`. 같은 id·같은 속성은 멱등, 다른 속성은 409, 없는 층은 404, 잘못된 값은 400, viewer는 403.

`POST /api/v1/map-registry/maps` 추가 선택 필드: `area_id`, `lineage`, `note`(≤500자), `threshold_policy`(`reject` 기본 | `derive`). 응답에 `area_id`, `lineage`, `yaml_adjustment` 추가. `verification_evidence`가 이제 `method=web_upload`로 기록된다(위치 선언·`grid_counts_pgm`·`artifacts_sha256`·`yaml_adjustment`·`client_checksums_provided`·`uploader`·`note`).

**동작 변경(주의)**: 기본 `reject` 때문에 `free_thresh`가 0.196보다 큰 trinary 지도(`map_saver_cli` 기본 0.25)는 이전엔 등록됐지만 이제 422다. 기존 시험 `test_map_registry_phase2b.py`의 고정 입력이 `free_thresh: 0.25`였으므로 그 값만 0.196으로 바꿨다(시험 대상이 임계값이 아니며, 다른 시험은 이미 0.196). 임계값 거부·보정은 새 시험이 직접 검증한다. 영역 연결·계보 기록은 각각 멱등이라 업로드가 중간에 실패해도 같은 요청 재시도로 완료된다. 저장된 번들 디렉터리까지 롤백하는 단일 트랜잭션은 아니다.

## 4. 검증

| 시험 | 결과 |
|---|---|
| 신규 `tests/test_registry_authoring_api.py` (영역 권한·멱등·충돌, 영역/계보 검증과 미저장, 임계값 거부·보정·해시·negate·비 trinary, 활성 지도 불변) | 12/12 PASS |
| 기존 Registry 시험(`test_map_registry`·`atomicity`·`http_e2e`·`http_upload`·`phase2b`·`test_map_areas`) | 49/49 PASS (phase2b는 위 입력 1줄 변경) |
| 신규 `tests/test_mapping_panel.mjs`(사유·묶음·실차 3곳·명령 상태 7종·HTML 이스케이프·stale 도우미) | PASS |
| 확장 `tests/test_registry_upload.mjs`(임계값 검사·검증·오류 문구·저장 결과 행) | PASS |
| `test_static_dashboard`(Py·Node), `test_goal_check.mjs` 11/11, `test_web_mapping_session.py` 26/26 | PASS |
| **격리 ROS 2 E2E** (실제 Nav2·SLAM·Frontier·잠금 Gate·watchdog, 모터 없음, 포트 18371): `probe.py` | 22/22 PASS |
| 같은 스택을 **웹 버튼으로**: START→PAUSE→SAVE→RESUME→STOP 전부 COMPLETED, 상태 표시가 전송→접수→실행 중→성공으로 전환, SAVE가 영역 연결·계보 포함 UNVERIFIED 버전 등록 | PASS |
| 브라우저(격리 서버 18370, 시험용 토큰): 영역 생성 → 건물·층 선택 → 안전하지 않은 YAML 선택 시 경고와 업로드 버튼 비활성 → "보정본" 선택 → 등록 → 저장 결과 표시 | PASS |
| 브라우저: 실차 Agent(mock, `mode=real`)에서 비활성 사유와 차단 3곳 표시 | PASS |
| 브라우저: Agent를 끈 뒤 센서 7행에 `[과거 표본 · 22초 전]`, LiDAR age 0.05→22.25 s | PASS |
| 휴대폰 375 px: 가로 스크롤 0, 맵핑 카드·등록 패널 세로 배치 | PASS |
| 전체 회귀(의도 환경별 78/78 스크립트, 이번 신규 2개 포함, 이전 76개에서 증가) | PASS |

미검증: 실패·타임아웃·거부 상태의 **브라우저 실표시**(Node 시험으로만 검증), 실제 휴대폰 기기(에뮬레이션만), 실차에서의 동작(실차 맵핑은 차단 유지).

## 5. 배포 상태

현재 18190 서버(B1 사본, 노트북)는 **변경 전 코드로 실행 중**이다. 새 UI·API를 쓰려면 18190을 재기동해야 한다(`fleet_control/scripts/run_field_server.sh`, 데이터 `local_data/field_run_20261120_b1/data`; Agent는 자동 재접속). 재기동은 사용자 확인 후에 한다.

## 6. 증거

`local_data/raw_results/web_ui_e2e_20261120/`(`probe.out`, 격리 스택 로그, 시험용 토큰은 0600/0700·공개 금지), 격리 서버와 시험용 토큰은 모두 종료했다.

## 7. 18190 재기동 (사용자 승인 후, 2026-11-20 12:39)

`fleet_control/scripts/run_field_server.sh`로 같은 데이터(`local_data/field_run_20261120_b1/data`, 현장 Registry 아님)와 `VIDEO_ORIGIN`으로 재기동. 확인: 인증 401/200, viewer의 `POST /areas` 403, 잘못된 본문 400(생성 없음), C103·H107 등록 유지, `pcu` 자동 재접속(online·reconciled), AI host 수신 9 fps, 현장 data 불변. 웹 프로브: START·지도 클릭 Goal 거부 PASS, 모터 경로 LOCKED PASS, Mapping 차단 유지. 단 **프로브 2건 FAIL(LiDAR·pose STALE)과 Gate `FAILSAFE_SENSOR_TIMEOUT`은 서버 재기동과 무관한 로봇 쪽 문제**다(아래).

### 로봇 쪽 발견 (재기동 확인 중, 읽기 전용 조사)
- 로봇의 YDLidar 드라이버가 11:25부터 오류를 기록했고 11:45:38 `Timeout count: 3`을 끝으로 스캔이 끊겼다(웹: LiDAR STALE 54분). 이후 `/odom`·`/imu` 수신도 확인되지 않고 ROS 노드가 4개만 보인다. 프로세스(launch, Agent, 송신기)는 살아 있고 부팅 후 USB 커널 이벤트는 없다.
- Gate는 `FAILSAFE_SENSOR_TIMEOUT`으로 출력을 막고 있다. 모터 경로는 LOCKED이며 속도 명령은 나가지 않는다.
- 같은 관찰자(ssh `ros2` CLI)에서 FastDDS 공유메모리 오류가 보이고 `/dev/shm`에 `fastrtps` 파일 120개가 쌓여 있다. 이것이 CLI 관측만 흐리는지 원인인지는 미확정이다.
- 조치는 하지 않았다. 스택 재기동은 사용자 확인이 필요하다.
