# rmf_demos Office 읽기 전용 연동 계약 v0.1 — 설계, 미구현

이번 작업은 기존 기준선을 보존하는 문서 정리입니다. Office/Gazebo/ROS/Adapter를 시작하지 않았고 현재 실행 graph·QoS·토픽 존재 여부를 조회하지 않았습니다. 아래 토픽은 공식/설치 소스 기반 **후속 조사 후보**이며 활성화됐다는 뜻이 아닙니다. Collision Monitor C·주행·안전 정책은 이 계약 밖입니다.

## 근거와 버전

로컬 설치 파일 확인: rmf_fleet_msgs3.0.4, rmf_task_msgs3.0.4, rmf_building_map_msgs1.2.1, rmf_fleet_adapter2.1.8. 설치된 msg와 `share/schemas/{fleet_state_update,fleet_state,robot_state,task_state_update,task_state}.json`을 대조했습니다. 기존 프로젝트 tests/ws_sink.py는 type=task_state_update/data.booking.id/data.status를 수신하며, RMF.md는 이 버전에서 레거시 task_summaries 대신 해당 JSON을 검증했다고 기록합니다. 이것은 Office 런타임 성공 증거가 아닙니다.

공식 참고:

- [rmf_demos](https://github.com/open-rmf/rmf_demos): Office와 server_uri 구성. main의 현재 예시는 최신 배포판이므로 Humble에 명령/이미지 태그를 그대로 적용하지 않습니다.
- [RMF state reporting](https://github.com/open-rmf/rmf_docs/blob/main/docs/source/interfacing/index.rst): FleetState와 제어 요청의 역할 구분.
- [RobotState](https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_fleet_msgs/msg/RobotState.msg), [Location](https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_fleet_msgs/msg/Location.msg): 로봇명·task_id·seq·level 위치·path.

다음 작업에서는 Claude가 확정한 Office 실행 저장소 commit/ROS 배포판/launch/level·fleet·robot 이름을 인수받아 계약에 고정합니다. 최신 main과 로컬 Humble의 차이는 로컬 설치/실행 버전을 우선합니다.

## 입력 인터페이스와 의미

|대상|읽기 입력 후보|정규화/제약|
|---|---|---|
|건물 지도|building_map, rmf_building_map_msgs/msg/BuildingMap|name,levels[].name/elevation/images/nav_graphs. OccupancyGrid가 아님|
|배경 이미지|Level.images[], AffineImage|x_offset/y_offset/yaw/scale/encoding/data 보존. 실제 축 방향·픽셀 좌표와RMF 미터 좌표 대응 검증 필요|
|로봇|fleet_states, rmf_fleet_msgs/msg/FleetState|name=fleet, robots[].name,location,seq,mode,battery_percent. 관측 heartbeat만 의미|
|위치|RobotState.location, Location|x/y는level 기준m,yaw는rad,t는원본ROS시간,level_name 보존. std_msgs/Header.frame_id가 없음|
|경로|RobotState.path[], Location[]|RMF 보고 waypoint 경로. Nav2 Global Plan·Local trajectory와 구분. 비어 있으면 없는 것|
|실제 궤적|동일 robot의 location 연속 관측|추정/보고 위치의 이력. 물리 Ground Truth·휠 odom이라고 부르지 않음|
|Nav2 경로/Costmap|해당 Office 로봇이 실제 발행하는 nav_msgs/Path,OccupancyGrid 및 update|namespaced 토픽·TF·stamp·QoS 확인 후에만 활성. 기본 demo가 Nav2를 사용한다고 가정하지 않음|
|임무|기존 관측 endpoint 또는 RMF task_state_update JSON|booking.id,status,assigned_to.group/name,phase 등. 실제 query/subscribe 경로는 후속 확인|
|fleet JSON|fleet_state_update JSON|data.name,data.robots dictionary. ROS FleetState와 중복되면 우선 소스 하나 선택|
|배정 상태|dispatch_states, rmf_task_msgs/msg/DispatchStates|active/finished는 배정 처리 상태. finished를 로봇 임무 완료로 해석하지 않음|
|Frontier|확인된 탐색 marker/목표/status 토픽만|Office에 Frontier가 없으면 UNAVAILABLE. RMF 목적지로 대체 금지|
|시계|clock, rosgraph_msgs/msg/Clock|Gazebo sim time. wall/Unix 시각과 구분|
|RMF 연결|각 observer의 수신 시각·API/Adapter 관측|transport 연결, fleet telemetry fresh, task feed fresh, 등록 확인을 별도로 표시|

QoS는 토픽명으로 추정하지 않습니다. 후속 Office 실행 담당자가 endpoint별 reliability/durability/history/depth를 기록하고 subscriber를 맞춥니다. 정적 건물 지도의 late join 수신 여부, volatile 경로의 재수신, 실시간 fleet 상태 주기를 확인합니다. TF가 없는 RMF level 좌표는 등록된 level 변환으로 처리하고 이름이 map이라는 이유로 identity TF를 생성하지 않습니다.

`RobotState.task_id`는 최근 command의 식별자일 수 있으므로 RMF booking.id·Mission Manager mission_id와 같다고 가정하지 않습니다. 명시적 correlation이 없으면 각각의 ID와 출처를 표시합니다. 계획된 path 마지막 점을 실제 current_goal이라고 확정하지 않습니다. 필요하면 derived_route_endpoint로 구분하고 unknown goal은 null로 둡니다.

RMF Web `_internal`은 Adapter 상태 송신 목적지이며 범용 브라우저 구독 API가 아닙니다. 기존 server_uri를 다른 sink로 바꾸면 기존 소비자가 데이터를 잃을 수 있습니다. 우선 기존 observer의 읽기 export 또는 실행 중 API server의 인증된 query/subscribe를 사용합니다. 정확한 API 버전·경로를 확인하기 전 임의 endpoint를 완료했다고 문서화하지 않습니다. 단독 기존 시험 WsSink는 기록 검사용이며 인증/운영 fanout 서버로 재사용하지 않습니다.

## ID·좌표·시간 규칙

새 계약은 아래 provenance를 모든 robot/map/path/task/media에 요구합니다. **현재 lab이 이 전체 필드를 강제한다는 뜻은 아닙니다.** 기존 보호는 kind/profile,session/sequence와 media boot/frame key 범위입니다.

```json
{
  "schema_version":"office-view/1",
  "source_mode":"SIMULATION", "source_kind":"RMF_DEMOS_OFFICE",
  "run_id":"office-run-unique", "source_boot_id":"observer-boot-unique",
  "clock_domain":"ROS_SIM", "clock_epoch":1,
  "building_id":"configured-building", "level_name":"configured-level",
  "map_session":"map-revision-or-hash",
  "fleet_name":"configured-fleet", "robot_name":"configured-robot",
  "robot_id":"stable-opaque-key",
  "source_frame_id":null, "render_frame":"office/configured-building/configured-level",
  "stamp_ns":"123456789000", "received_at_unix_ms":1794572400000,
  "sequence":"42", "age_s":0.1, "timestamp_status":"VERIFIED_SIM_CLOCK"
}
```

- robot_id는 (fleet_name, robot_name)을 canonical JSON 배열로 직렬화한 UTF-8의 SHA256 등 충돌 방지 키를 사용합니다. 표시명은 원래 이름입니다. cache key에는 source_mode/run_id/boot도 포함합니다. 이름·topic namespace·배열 index 하나만으로 로봇을 동일시하지 않습니다.
- 원본 ROS frame_id와 이미지 frame 식별자는 그대로 보존합니다. RMF Location에는 frame_id가 없으므로 source_frame_id=null, level에 등록한 render_frame을 별도 지정합니다. prefix를 TF에 실제 발행하거나 ROS 원본을 변경하지 않습니다.
- 같은 level·map revision·변환이 검증된 로봇만 같이 그립니다. 층 변경/원점 교체 시 궤적 분리. 서로 다른 로봇의 odom/base_link 이름 충돌은 구성 namespace/TF 매핑으로 해결하고 단순 문자열 동일성을 근거로 합치지 않습니다.
- ns와uint64 seq는 JSON 문자열. source stamp, 수신 wall 시각, monotonic freshness, 원본 capture/재생 생성 시각을 분리합니다. Location.t와JSON unix_millis_time이 같거나 sim time이라고 가정하지 않습니다. 타입/출처를 기록하고 모르면 TIME_UNVERIFIED.
- /clock 역행·reset은 clock_epoch 증가 및 새 session으로 처리하고 경로·목표·영상 매칭 cache를 비웁니다. pause는 SIM_PAUSED로 표시해 정지한sim clock만으로OFFLINE을 만들지 않습니다. 수신 watchdog과센서 clock watchdog은 독립적입니다.
- 기존 media result_key(boot,stream,seq,frame_id,stamp_ns)에 새run/robot/clock scope를 추가하는 것은 후속 변경입니다. 다른 robot/frame 결과 혼합·ns Number 반올림을 금지합니다.
- mission_id는 origin(MISSION_MANAGER/RMF),source instance/run,원본 ID를 함께 갖습니다. RMF status는 원문을 보존하고 표시는 대문자 토큰으로 구분합니다. underway를 COMPLETED나command ACK를 SUCCEEDED로 바꾸지 않습니다. 상태 충돌은 source별로 남깁니다.

## 최소 API·구현 변경안 — 아직 없는 기능

경로명이 있는 것은 후속 설계이며 현재 서버에 배포하지 않았습니다. 현재 API 계약은 INTERFACES.md입니다.

```text
Office ROS / 기존 RMF 관측 export
    → 별도 구독 전용 office_observer
    → 원자적으로 교체하는 private snapshot 파일
    → dashboard_lab OfficeReadModel (명시적 office 입력 선택)
    → 기존 인증 GET /api/state + 지도·경로·작업 패널
```

1. observer는 publisher/action client/service command 없이 구독·읽기만 합니다. launch/Adapter/server_uri/Collision Monitor는 건드리지 않습니다. 가능한 기존 export/API를 읽어 ROS 구독 중복을 줄입니다. snapshot.tmp→rename, 크기·주기 제한, boot/sequence 포함, 비밀 없음. ROS·이미지 디코딩은 worker에서하고 UI poll에 따라 재처리하지 않습니다.
2. lab-local `OfficeReadModel` 및 `--office-snapshot`(제안)을 추가합니다. 현재 SIMULATION fixture generator와 명시적으로 상호 배타적으로 선택합니다. 새 local source validator는 RMF_DEMOS_OFFICE를 허용하고 Office를 SYNTHETIC_ROS_TEST/GAZEBO_REPLAY/ROS_LIVE로 위장하지 않습니다. 현재 공유 Store의허용kind를변경하지않고 필요한 Grid/좌표 helper를 사용해 lab-local read model을만듭니다.
3. 인증된 기존 GET `/api/state`에 mission.robots/missions/events,visualization.layers/paths/robot_trails,rmf를 정규화합니다. provenance와 source raw status를 추가하되 기존 schema를 유지합니다. 경로 배열에는 robot_id와level을 명시하고 선택로봇에 해당하는 경로만 current view에 노출합니다. 기존 viewer/lab권한 유지, Office가 연결된 SIMULATION에서는 합성 fixture/fault/media가 섞이지 않도록 로컬 suite를 비활성화합니다.
4. BuildingMap은 별도 `background_raster`/vector type으로 표시합니다. 제안 GET `/api/office/maps/{map_id}.png`는이미지출처·revision과짝지어인증된읽기만제공합니다. 기존 OccupancyGrid PNG API에는 실제 grid만 넣습니다. floorplan 색을 센서 occupancy로 변환하지 않습니다. 바닥 이미지 축 반전/미터 변환은 실제 landmark3개 이상 대조후 확정합니다.
5. RMF reported path는 `rmf_reported_route` 레이어로 추가하고 Nav2 global/local plan과 별도 legend를 사용합니다. actual trail은 위치관측으로 갱신. 임무 이력은같은source/run/ID 이벤트만 연결. 로봇별plan·trail cache와상한길이·freshness를분리합니다.
6. 기존 Adapter snapshot {observed_at,registered,fleet,robots}는 그대로 둡니다. 단일fleet legacy view와 별개로 제안 rmf.fleets[]에 fleet별 telemetry/task-feed/registration evidence를 표시합니다. registered=true는등록확인근거가있을때만; fleet_states수신만으로승격하지않습니다. heartbeat expiry는기존5초규칙을기본으로시뮬pause/수신상태를분리해확정합니다.
7. 영상은없는한 UNAVAILABLE입니다. 기존 Gazebo JPEG/합성 Depth를 Office 로봇 영상으로자동선택하지않습니다. Office 카메라가있다면 명시적robot/stream/frame/clock매핑후연결하며 기존인코더·preview/raw분리구조를재사용합니다.

새 ingest POST나 command API는필요없습니다. 백엔드는local snapshot만읽고 브라우저는기존session/Bearer API로조회합니다. 새파일·토큰권한은기존정책을유지하고 외부주소·STUN/TURN·공개포트를추가하지않습니다. 구현시험은별도프로세스/새미사용포트에서실시하고 현재18310/18311및운영프로세스를무단재기동하지않습니다.

## 합격 체크리스트와 후속 작업 순서

- [ ] Claude 인수인계: Office commit/launch/ROS배포판, building/level·fleet·robot명,topic/type/QoS,clock,기존export 소유자 확정.
- [ ] 한 번 저장한 실제 Office 메시지와 schema hash로 로컬fixture 생성. 원본과 변환물 분리; 예제합성결과와구분.
- [ ] 서로다른fleet의동일robot명,같은level명/다른building,namespace동일frame명을 구분.
- [ ] LIVE→Office SIMULATION→REPLAY 전환 시 session·cache 초기화; source 다르면 map/TF/path/media 조합 거부.
- [ ] map session/revision 변경,layer origin/yaw/resolution,층 변경,TF부재,지연/역순seq,boot재시작 처리.
- [ ] /clock reset/pause/배속과wallclock 차이: stale/offline/latency 오판 없음.
- [ ] RMF보고경로/Nav2계획/actual trail 구별,원본goal 없으면null,dispatch finished≠task completed.
- [ ] unknown status 유지; Mission Manager와RMF task ID 충돌 방지; 등록 미확인UNAVAILABLE.
- [ ] Office RGB없으면저장Gazebo RGB를대체하지않음; 다른robot/boot/frame/time YOLO 폐기; Depth단위미확인표시.
- [ ] 인증401/403,command route 부재,observer publisher/action-client 없음,subscriber하나당공유처리 확인.
- [ ] 기준선Python25+JS6 및 기존회귀90의실행환경확인후재실행. 새Office정규화·좌표·clock·분리검사 추가.
- [ ] Claude가실행한Office를읽기만하여2대이상 위치/임무/경로를브라우저검증. 별도run에수신주기·변환CPU/RAM·대역폭·displayFPS/P95/P99 기록. 기존11.90/7.99FPS 결과와입력조건차이표시.

현재까지 PASS인 것은 설치msg/schema와기존코드의정적대조 및 문서·기준선해시검사입니다. 위Office 런타임 체크리스트는 전부향후작업으로남습니다. 실제LIMO검증과는별개입니다.
