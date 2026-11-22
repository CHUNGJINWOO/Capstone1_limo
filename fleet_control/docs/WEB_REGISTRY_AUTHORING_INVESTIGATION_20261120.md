# 웹에서 건물·층·영역 생성 + SLAM 지도 저장 → 등록: 기존 구현 재사용 조사 (2026-11-20, 조사만, 미구현)

구현·수정은 하지 않았다. 아래는 현재 `fleet_control/` 코드를 읽고 정리한 사실과 제안이다.

## 1. 이미 있는 것 (재사용 가능)

| 기능 | 위치 | 상태 |
|---|---|---|
| 시설·건물·층 **생성** UI와 API | `static/registry_upload.js`(`regFacBtn`·`regBldBtn`·`regFlrBtn` → `hierarchyPost`), `POST /map-registry/facilities·buildings·floors`, `http_api.py` `MAP_REGISTRY_POST_ROUTES` | 있음. operator/admin만(`can_control`). viewer는 UI 안내 문구 |
| 지도 **파일 업로드** 등록 UI와 API | `registry_upload.js`(`regUp*`: YAML·PGM·posegraph·posegraph.data 선택, SHA-256 계산, `Idempotency-Key`), `POST /map-registry/maps` | 있음. 항상 `UNVERIFIED`로 등록, 번들 무결성 검사, 업로드 동시 2개 제한 |
| 건물·층·영역·지도 버전 **선택·미리보기** | `index.html`(`mapBuilding`·`mapLevel`·`mapArea`·`mapVersionSelect`), `app.js`, `GET /map-registry/facilities·maps·maps/{id}/meta·preview.png` | 있음. 오늘 H107·C103 모두 표시 확인(API·PNG) |
| 영역 테이블·생성·지도 연결·계보 | `map_registry.py`(`create_area`, `link_map_version_area`, `record_lineage`) | **라이브러리에만** 있음(스크립트에서 사용) |
| 웹 Mapping SAVE → 등록(영역 연결·계보 포함) | `mapping.py`(`_save_target`: 등록된 층·영역 검증), `http_api.py`(`/agent/mapping-map` → UNVERIFIED 등록 + area link + lineage) | 있음. **SIM/mock Worker 전용**(실차는 `mapping_real_blocked`) |
| 구 SLAM 저장 명령 `slam_start/save/stop` (+`floor_id`) → 등록 | `service.slam_command`, `/agent/slam-map`(`_agent_slam_map`), `ros_adapter`·`slam_control` | 있음. Agent가 `state.slam.enabled`일 때만, SIM 전용 |

## 2. 없는 것 (오늘 H107 등록을 스크립트로 해야 했던 이유)

| # | 부족한 점 | 영향 |
|---|---|---|
| R1 | **영역 생성 라우트·UI 없음.** `POST /map-registry/areas` 미라우팅, 폼 없음 | 웹에서 "H107 강의실" 같은 영역을 만들 수 없음 |
| R2 | **지도 업로드에 영역·계보 필드 없음.** `POST /maps` 본문이 `floor_id`·파일만 받음. 영역 연결과 계보(NEW_MAP/CONTINUE_MAP/IMPORTED, 부모, 세션)는 별도 메서드 | 직접 업로드한 지도는 영역에 속하지 않고 계보가 없음 |
| R3 | **YAML 임계값 검사 없음.** 레지스트리는 `0 ≤ free_thresh < occupied_thresh ≤ 1`만 확인. 로봇 `map_saver_cli` 기본 `free_thresh: 0.25`는 unknown 픽셀(p=0.196)을 free로 해석시킴. 등록 스크립트(`register_saved_map.py`)에만 이 검사가 있음 | 웹 업로드로 오늘의 H107 원본 YAML을 올리면 **그대로 등록**됨 |
| R4 | **실차에서 SLAM 지도 저장을 웹이 시킬 수 없음.** 저장 경로 둘 다(Mapping SAVE, `slam_command`) SIM 전용. 실차 Agent에는 저장 실행기가 없음. 오늘은 SSH로 `map_saver_cli`+`serialize_map`을 직접 호출 | "웹에서 SLAM 지도 저장 → 등록" 불가 |
| R5 | 지도 파일을 로봇→노트북으로 옮기는 경로 없음(Agent는 `/agent/mapping-map`·`/agent/slam-map`으로 올릴 수 있으나 위 이유로 실차 미사용) | 수동 복사·해시 확인 필요 |
| R6 | 영역 목록은 `GET /facilities` 트리에 포함되나 영역 수정·삭제·층 전체/일부 변경 경로 없음 | 오타 수정은 불가(레지스트리 쓰기 전용 트리거) |

## 3. 재사용 설계안 (구현 전 제안, 승인 필요)

1. **R1** `POST /map-registry/areas`를 기존 4개 라우트와 같은 방식으로 추가(operator, 멱등, 입력 검증은 `create_area` 재사용). 같은 `hierarchyPost` 패턴으로 UI 폼(`regAreaFloor`·`regAreaId`·`regAreaName`·`regAreaCoverage`) 추가. 변경 파일: `http_api.py`(라우트 집합), `map_registry.py`(`dispatch_write` 분기), `index.html`, `registry_upload.js`.
2. **R2** `POST /maps` 본문에 선택 필드 `area_id`와 `lineage{mapping_mode, parent_map_version_id, session_id}`를 추가하고 **같은 트랜잭션**에서 `link_map_version_area`·`record_lineage` 호출(지금은 별도 호출이라 중간 실패 시 연결 누락 가능). 영역은 같은 층에 속해야 함(`_save_target`의 검사 재사용). UI에 영역 선택·계보 선택 추가.
3. **R3** 업로드 검증에 "unknown 픽셀이 free/occupied로 해석되는 임계값" 검사를 넣고, 거부 대신 선택지를 준다: (a) 거부하고 안내, (b) 서버가 파생 YAML(`free_thresh`를 unknown 확률 이하로)을 만들고 원본·파생 해시와 사유를 증거에 기록(오늘 스크립트가 한 방식). 원본은 변경하지 않는다.
4. **R4** 실차용 **저장 전용(SAVE-only) 작업**을 Agent에 추가: Lease 보유 operator 요청 → Agent가 `map_saver_cli`와 `serialize_map`만 호출 → `/agent/mapping-map`(또는 `/agent/slam-map`)으로 업로드 → UNVERIFIED 등록(영역·계보 포함, 계보는 NEW_MAP). 이동·Goal·START를 열지 않고 `mapping_real_blocked`와 Gate 모터 잠금은 그대로 둔다. 조건: 로봇이 정지 상태(속도 0 확인), Gate 신선, 모터 경로 LOCKED 또는 속도 명령 없음, 저장 파일 크기·무결성 확인 후 업로드. 이것은 **실차 Agent 변경**이므로 별도 승인과 로봇 백업·롤백이 필요하다.
5. **시험**: Mock(영역 라우트·계보 원자성·임계값 검사), 격리 ROS 2(저장 전용 작업, 정지 조건 위반 시 거부), 브라우저(폼 → 트리 갱신), 회귀 76/76 유지.

## 4. 안전·정책 메모

- 건물·층·영역은 운영자 선언 정보이며 로봇이 자동 검출하지 않는다는 표기를 유지한다(`auto_detected:false`).
- 등록된 지도는 항상 UNVERIFIED. 층 활성 지도 지정·검증은 정책 결정 전까지 라우팅하지 않는다(현재도 404).
- 오늘 H107처럼 한 자리에서 저장한 지도는 품질이 낮다. 등록 시 확인 셀 수·면적을 증거에 기록해 UI에서 보이게 하는 것을 권장한다.
- 다른 에이전트 소유 파일(Gate, 실차 launch)은 위 어느 변경에도 필요하지 않다.

## 5. 구현 상태 갱신 (2026-11-20 저녁)

R1(영역 생성 라우트·UI), R2(업로드의 영역·계보, 저장 전 검증), R3(임계값 안전 검사 + 보정본 등록)은 구현·시험했다. R4(실차 저장 전용 작업), R5, R6은 미구현이다. 내용과 검증은 [WEB_OPERATIONS_UI_20261120.md](WEB_OPERATIONS_UI_20261120.md).
