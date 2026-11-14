# dashboard_lab 후속 도입 설계 v0.1

상태: **설계 완료 / 계약 예시 작성 / UI 구현 미착수 / 실제 RMF 검증 미착수**.

`dashboard-lab-offline-20261113-v1`과 `rmf_web_research`는 읽기 전용 참조입니다. 이 폴더는 차기 구현을 위한 별도 실험 공간입니다. 현재 서버·Office·ROS·LIMO에 접속하거나 새 bridge를 실행하지 않습니다. 기존 `dashboard_lab/docs/OFFICE_INTEGRATION.md`를 Office 연동의 상위 기준으로 유지하며, 여기서는 UI 읽기 모델을 구체화합니다. 충돌이 생기면 실제 Office 메시지/상위 계약을 확인한 뒤 이 설계를 개정합니다.

## 1. 아키텍처와 구현 경계

```text
기존 MM/Agent + 확정된 Office 관측 export (향후)
  → 독립 ReadOnlyNormalizer
  → versioned ViewSnapshot + scoped resource index
  → SelectionStore + derived selectors
  → RobotSummary / SpatialView / MediaView / TaskTimeline / WarningInbox
```

기존 JS/Canvas 구조를 유지하고 `state/selection.mjs`, `state/selectors.mjs`, `contracts/`, `panels/`, `layers/`, `tests/`를 차기 실험 공간에만 추가하는 안입니다. React/MUI/Three.js 도입을 전제로 하지 않습니다. 기존 RGB-D encoder·인증·FrameGate·Costmap 좌표 검증을 교체하지 않습니다. 읽기 모델은 현재 `/api/state`와 호환되는 adapter로 연결하되, 새 계약 전체를 현재 API가 구현했다고 주장하지 않습니다. 제어 POST/action/service client는 만들지 않습니다.

## 2. 도입할 8개 기능과 완료 조건

|순서|기능|자체 설계|의존 데이터|합성 검증 → 실제 검증|
|---|---|---|---|---|
|1|로봇 요약 카드|fleet/이름/층/배터리/임무/수신시각, robot OFFLINE과 media STALE 독립|robot state, task binding, safety 관측값|동명 다른 fleet·offline → Office2대 상태 원본 대조|
|2|임무 타임라인|phase/event 트리, 상태 변화·시각·원본 ID, 불완전 이력 표시|task state/log, 명시적 MM↔RMF correlation|누락·중복·지연 → Office 원본 event와 비교|
|3|선택 상태 연동|한 SelectionStore에서 지도·카메라·Costmap·임무 selectors 생성|resource owner/scope와 변환 registry|빠른 전환·늦은 응답 → 실제 로봇별 매핑 대조|
|4|층별 지도|BuildingMap level 배경과 SLAM 세션을 독립 레이어로 관리|building/level, affine image, ROS grid, 등록 변환|다른층·원점/yaw → Office landmark·TF 검증|
|5|예상 완료 정보|출처가 명시된 reported/derived progress·ETA, 없으면unknown|task estimate/시간·clock mapping|누락·pause·재계산 → 원본 RMF estimate와 비교|
|6|경고 Inbox|심각도·로봇·task·출처 필터, 연결된 타임라인, 로컬 읽음 상태|alert/event log, sequence/revision|중복·retract·역순 → 실제 alert/log 대조|
|7|패널 배치 저장|layout_schema_version, 사용자별 view preference, reset|브라우저 로컬 설정만|좁은 화면·잘못된 설정 복구 → 표시 성능 회귀|
|8|교통 trajectory|RMF schedule·reported route·Nav2 plan·관측 trail 독립 legend|별도 trajectory feed, participant/run/clock 매핑|공식 mock → 승인된 격리 Office schedule 대조|

레이아웃 저장에는 토큰·원시 센서·임무 내용·비공개 URL을 넣지 않습니다. 상대 패널 크기와 표시 설정만 저장하며 서버나 로봇 상태를 복원하는 기능으로 사용하지 않습니다. 기존 페이지 스키마와 맞지 않으면 기본 배치로 복구합니다.

## 3. 로봇 요약과 선택 상태

현재 기준선의 `selected`는 지도/일부 경로를 바꾸지만 RGB·Depth는 단일 독립 스트림입니다. 따라서 카메라가 선택 로봇을 따른다고 가정하지 않습니다.

새 SelectionState는 environment/run/clock_epoch, robot_key, map_context_key, mission_key, media_bindings, selection_revision을 갖습니다. 선택은 브라우저 view 의도일 뿐 원격 명령이 아닙니다. revision은 로컬 증가 정수이며 데이터 source sequence와 다릅니다.

|사용자 동작/상황|정해진 동작|
|---|---|
|로봇 카드 선택|robot_key 갱신, 해당 로봇에 속하지 않는 임무 선택 해제, 올바른 owner의 media/Costmap/path만 선택|
|임무 선택|검증된 assignee가 있으면 로봇 선택. 미배정·불명확하면 임무 상세만 열고 다른 로봇을 자동 선택하지 않음|
|지도 로봇 마커 선택|카드 선택과 같은 action. 같은 robot_key를 모든 패널에 전달|
|지도 층 수동 변경|층을 바꾸고 선택 로봇이 다른 층이면 OFF_LEVEL 표시. 로봇을 다른 층에 그리지 않음|
|선택 로봇이 OFFLINE/목록에서 사라짐|마지막 선택을 경고와 함께 유지. 첫 번째 다른 로봇으로 카메라를 자동 교체하지 않음|
|로봇에 카메라/Costmap 없음|해당 패널 UNAVAILABLE, 소유자가 다른 최신 데이터로 대체 금지|
|빠른 A→B 전환 후 A 응답 도착|응답 scope/owner/selection_revision 재검사 후 폐기. bitmap/depth/overlay pending slot 해제|
|camera publisher 재시작|해당 stream boot cache만 폐기. 동일 세계의 map/mission까지 임의 초기화하지 않음|
|run/세계 clock epoch 변경|모든 종속 cache·궤적·task binding·media 매칭 폐기. 새 scope에서 명시적으로 선택|
|전역 RMF 장애 경고 선택|전역 상세만 열고 임의 로봇/지도 초점 이동 없음|

카메라 바인딩에는 robot_key, camera_id, stream_id, producer_id, source_frame_id가 필요합니다. RGB/Depth가 같은 robot이라도 registration_verified=false이면 픽셀 거리나 YOLO의 3D 위치를 서로 대응시키지 않습니다. 사용자가 미디어를 수동 고정한다면 PINNED OTHER ROBOT 표시를 필수로 하고 거리·overlay 결합을 금지합니다. 기본은 owner가 같은 스트림만 선택합니다.

상태 카드는 연결/자료 신선도/안전 관측/모터 잠금 관측을 별도로 표시합니다. 관측 없는 Safety Gate는 UNAVAILABLE이며 UI가 NORMAL·정지·비상정지를 추론하지 않습니다. Inbox의 로컬 읽음, 임무 취소, 물리 비상정지를 구분합니다.

## 4. 경로 계열과 legend

|layer_kind|의미·자료|표시안|금지되는 해석|
|---|---|---|---|
|RMF_SCHEDULE_TRAJECTORY|시간이 붙은 예약/예측 trajectory, participant ID·conflicts|보라 곡선, 시간 범위·예측 시각 표시|Nav2 명령·실제 이동 이력 아님|
|RMF_REPORTED_ROUTE|RobotState.path 등 보고 waypoint|보라 점선, 보고 경로라는 이름|확정 goal·schedule reservation이라고 부르지 않음|
|NAV2_GLOBAL_PLAN|검증된 nav_msgs/Path|파란 점선, 생성 stamp|성공적으로 따라갔다고 해석 금지|
|NAV2_LOCAL_PLAN|실제 controller path/trajectory|노란 짧은 선, 짧은 TTL|global goal이나 실제 trail과 혼합 금지|
|OBSERVED_TRAIL|관측 pose 연속 표본|민트 실선, 추정 출처·gap 표시|Ground Truth/휠 odometry로 자동 명명 금지|

색상만으로 구분하지 않고 선 종류·텍스트·출처·시각을 제공합니다. 경로마다 robot_key, frame_context, source stamp, clock domain/epoch, transform evidence를 요구합니다. missing TF/다른 level/epoch는 renderable=false. offline gap·텔레포트·SLAM reset·층 변경 시 trail을 segment로 끊습니다. RMF 예측 곡선과 실제 trail을 겹쳐 성능 비교하려면 동일 시간/좌표 등록이 검증돼야 합니다. conflict는 일정상 충돌이며 실물 충돌 검출이라고 표시하지 않습니다.

## 5. BuildingMap·SLAM·Costmap 좌표계

- BuildingMap은 건물/층의 미터 좌표와 AffineImage입니다. SLAM은 별도 map session의 OccupancyGrid이며 서로 같은 map 이름만으로 일치하지 않습니다.
- SLAM grid 셀 중심은 `(i+0.5, j+0.5)*resolution`에 grid origin의 yaw·translation을 적용해 ROS map 좌표가 됩니다. bottom-row ROS와 top-row canvas의 변환을 한 번만 수행합니다.
- BuildingMap 이미지 픽셀→level 좌표는 해당 이미지 scale/offset/yaw와 원본 축 관례를 보존하는 등록 변환으로 관리합니다. generic PNG 원점과 ROS OccupancyGrid origin을 같은 것으로 취급하지 않습니다.
- 각 변환은 source/target frame context, map revision, 검증 방법, 유효 시간 또는 static evidence를 포함합니다. `T_level_from_slam`이 없으면 두 지도를 별도 view로 제공하고 중첩하지 않습니다. UI 회전은 view transform만 바꿉니다.
- 같은 층에서 로봇별 독립 SLAM이 여러 개 생기면 map_session을 로봇별로 분리합니다. 공유 지도 등록이 확인돼야 같은 view에 겹칩니다.
- Global/Local Costmap은 owner robot의 실제 frame/origin/resolution/stamp를 사용합니다. rolling origin을 매 갱신 반영하고 update patch는 같은 grid revision/shape/frame에서만 적용합니다. revision이 바뀌면 full grid를 다시 받아야 합니다.
- 공식 UI의 image yaw 적용을 그대로 믿고 복사하지 않습니다. 0이 아닌 yaw, 음수 origin, 다른 resolution, TF 누락/시간 오차를 검증합니다. 등록 근거가 없는 identity transform 자동 생성은 금지합니다.

## 6. 진행률·ETA 정책

우선순위: 원본 provider가 의미와 범위를 명시한 progress → 검증된 RMF estimate 기반 예상값 → UNAVAILABLE. phase 완료 개수/직선거리/경과시간만으로 실제 완료율을 만들지 않습니다.

reported progress는 provider·방법·0..1 범위·source stamp를 보존합니다. estimate 기반 후보는 `1 - remaining_ms / (planned_finish_ms - start_ms)`이며 유효한 동일 clock/epoch, 양수 분모, 음수가 아닌 remaining을 만족할 때만 사용합니다. 범위 밖은 invalid로 표시하고 0/100으로 조용히 잘라내지 않습니다. label은 '예상 진행률'이고 actual progress와 구분합니다. 이 식은 업무별로 유효성 검토가 필요하며 일반 정답으로 확정하지 않습니다.

ETA는 reported planned_finish 또는 동일 clock의 source_now+remaining으로 구분합니다. ROS_SIM에는 sim 시각/잔여 초를 표시하고 wall clock 달력시각으로 임의 변환하지 않습니다. /clock pause는 SIM_PAUSED, 역행은 새 epoch, 불명확 시 TIME_UNVERIFIED입니다. 추정 변경으로 진행률이 내려갈 수 있으며 이전값을 강제로 유지하지 않습니다.

데이터 없음은 '— / 예상 정보 없음', 오래된 값은 마지막 값+STALE+원본 시각, invalid는 오류 이유를 표시합니다. task가 reported completed면 완료 상태를 보여주지만 missing progress를 측정된100%로 기록하지 않습니다. 취소/실패/지연은 원래 상태를 유지하고 진행률로 성공을 대신 판단하지 않습니다.

## 7. 구현 단계와 Office 착수 조건

A. 별도 실험 공간에서 계약/selection reducer/패널1·2·3·6을 합성 입력으로 구현. B. BuildingMap/진행률/배치, 기존 RGB-D 성능·좌표 회귀 확인. C. Claude가 Office 실행환경/토픽·QoS·schema·clock을 확정한 후 기존 OFFICE_INTEGRATION을 기준으로 읽기 연결. D. 별도 trajectory feed 검증. 이번에는 A의 UI/서비스 구현도 시작하지 않았습니다.

Office 착수 전 인수 항목: repo commit/launch, ROS 배포판·RMF/msg/schema 버전, domain/RMW, topic/type/endpoint QoS, building/level·fleet/robot 명칭, map/TF/clock 정의, 읽기 export/API와 인증, stream 소유자, 보호 프로세스 목록. 최신 RMF Web main API를 Humble 도메인에 직접 연결하지 않습니다.

합성 사례를 통과해도 실제 Office PASS가 아닙니다. 향후 기존31+90 검사와 RGB11.90/Depth7.99FPS 기준은 같은 조건에서 재측정하고, 다른 입력의 수치를 단순 비교하지 않습니다. 실제 LIMO 검증은 별도 승인된 현장 단계입니다.
