# 시각화 데이터 계약

실제 실차에서 확인한 publisher QoS·frame·인코딩과15초 검증 결과는 [REAL_LIMO.md](REAL_LIMO.md)에 추가 기록했습니다. 아래 기본 후보 표의 최초 미확인 상태와 구분하세요.

## 기존 서비스와 경계

노트북 gateway만 기존 Mission Manager `/api/v1/dashboard`, `/api/v1/robots/{id}/map.png` 및 기존 AI `/status`, `/frame.jpg`를 **GET**합니다. 실제 URL·토큰은 개인 설정 파일에만 있습니다. 브라우저는 같은 origin의 인증 API만 사용합니다. 기존 서버·Agent·영상 프로토콜·ROS 제어 경로는 수정하지 않습니다. 기존 영상 서버의 JPEG와 결과 tuple `(server_boot, seq, frame_id, stamp_ns)`가 일치할 때만 새 결과를 표시합니다. 추론 지연/기존 로봇 RTT와 장치 간 단방향 지연은 별개입니다. 시계 동기화가 미검증이면 단방향 지연을 수치로 만들지 않습니다.

## ROS 입력 (기본 후보, 실차 활성 publisher 미확인)

|키/기본 토픽|타입|브리지 구독 QoS|의미|
|---|---|---|---|
|map `/map`|nav_msgs/OccupancyGrid|Reliable, TransientLocal, depth1|원본 map frame, origin quaternion yaw|
|global/local_costmap `/global_costmap/costmap`, `/local_costmap/costmap`|nav_msgs/OccupancyGrid|Reliable, TransientLocal, depth1|signed -1~100; free/inflation/lethal/unknown|
|각 costmap의 `_updates`|map_msgs/OccupancyGridUpdate|Reliable, Volatile, depth10|x/y/width/height 부분 갱신, 0으로 clearing|
|global_plan `/plan`, local_plan `/local_plan`|nav_msgs/Path|Reliable, Volatile, depth10|각 pose/header frame 및 stamp로 map 변환|
|frontiers `/explore/frontiers`|visualization_msgs/MarkerArray|Reliable, Volatile, depth10|marker.pose+points의 후보 위치, 선택 목표로 간주하지 않음|
|selected_frontier (기본 없음)|geometry_msgs/PoseStamped|Reliable, Volatile, depth10|실제 선택 목표 토픽을 명시해야 표시|
|depth `/camera/depth/image_raw`|sensor_msgs/Image|BestEffort, Volatile, depth1|16UC1 또는 32FC1만 허용|
|camera_info `/camera/depth/camera_info`|sensor_msgs/CameraInfo|SensorData QoS|K,D,distortion,크기,frame,stamp 보존|
|explore `/explore/status`|explore_lite_msgs/ExploreStatus|Reliable, Volatile, depth10|실제 패키지 underlay 필요; 시험만 std_msgs/String JSON 사용|
|`/tf`, `/tf_static`|tf2_msgs/TFMessage|tf2 기본 listener QoS|map→sensor/odom/base_footprint|

`--topics` JSON으로 키별 토픽을 바꿀 수 있습니다. Nav2 raw `nav2_msgs/Costmap`과 다른 custom trajectory/Frontier 타입은 현재 직접 지원하지 않습니다. OccupancyGrid가 없는 경우 UNAVAILABLE로 남기고 임의 변환값을 만들지 않습니다. 구독 QoS는 **구현값**이며 실차 publisher에서 확인한 값이 아닙니다.

## 좌표와 시각

- Grid 셀→ROS frame: `origin.position + R(origin.yaw) × resolution × [u,v]`. 셀 중심은 `[u+.5,v+.5]`입니다. map 변환을 추가 합성합니다.
- PNG는 ROS row0(bottom)을 top-down으로 한 번 뒤집습니다. Canvas world y-up/화면 y-down 변환을 적용합니다. 별도 보기 회전·pan·zoom은 ROS 데이터/원본 지도에 영향을 주지 않습니다.
- rolling local costmap은 **각 full message의 origin**과 해당 stamp의 map_from_frame을 사용합니다. 부분 갱신은 가장 최근 full grid에 경계·frame 검사를 거쳐 반영합니다. frame/TF가 없거나 비평면 변환이면 레이어를 표시하지 않습니다.
- Path/Marker/PoseStamped는 **메시지 timestamp의 TF**를 사용합니다. stamp0만 tf2 latest 규칙을 따릅니다. 후보와 선택 목표를 분리합니다. 이동 궤적은 실제 TF 표본 이력이며 계획 경로가 아닙니다.
- `stamp_ns`, `ros_time_ns`, `tf_stamp_ns`는 브라우저 응답에서 **10진 문자열**입니다. JS Number로 변환해 frame 정합 비교를 하지 않습니다. 브리지 입력 Python 정수도 받습니다.
- ROS source age와 gateway monotonic 수신 age를 분리합니다. 3초 초과 STALE; TF 불가 UNAVAILABLE; 원본 시각/clock 불명확 TIME_UNVERIFIED. 기존 지도 API는 원본 ROS stamp가 없어 Agent 수신 나이만 표시합니다.
- 과거 static map/path도 3초 기준으로 STALE 표기할 수 있습니다. 이는 오래된 데이터를 현재 측정값으로 오인하지 않기 위한 보수적 표기이며 map이 삭제되었다는 뜻은 아닙니다.

## Depth 수치

[REP-118](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0118.rst)의 32FC1 m / OpenNI 16UC1 mm 관례를 기본으로 삼습니다. `--depth-scale-m` 기본0.001은 **실제 DaBai 보정 확인값이 아닙니다**. 장치 확인 후 설정합니다. endian/step/padding을 반영하고 0·NaN·Inf·음수는 무효로 처리합니다. float32LE 미터 배열을 zlib+base64로 보냅니다. 최대 너비320, nearest sampling stride를 기록하며 프레임의 CameraInfo는 원본 해상도 기준이므로 다운샘플 픽셀로 직접 역투영하지 않습니다. 시각화만 수행하고 Nav2/Gate 입력은 변경하지 않습니다. RGB registration은 false/미검증이며 3D 추정·객체 대응에 사용하지 않습니다.

## 새 HTTP API

|경로|권한|내용|
|---|---|---|
|GET `/`, 정적 CSS/ES modules|공개 shell|센서/토큰 데이터 포함하지 않음|
|POST `/api/login`|viewer 토큰 JSON|8시간 HttpOnly, SameSiteStrict cookie; same-origin 검사|
|POST `/api/logout`|현재 세션|세션 폐기|
|GET `/api/state`|viewer cookie 또는 Bearer|mission/video/visualization 상태·counter·권한|
|GET `/api/map/{robot}.png`|viewer|기존 지도 캐시|
|GET `/api/grid/{layer}.png`|viewer|RGBA grid/costmap|
|GET `/api/video.jpg`|viewer|정합된 JPEG; frame key 헤더|
|GET `/api/depth`|viewer|실제 또는 격리 시험 metric depth 배열+metadata|
|POST `/api/bridge`|**별도 bridge Bearer**|read-only ROS telemetry 수신, 최대12MiB|
|그 외 제어 POST, PUT, DELETE|항상 거부|405, upstream 전달 안 함|

토큰은0600 파일에서 읽고 응답·로그에 출력하지 않습니다. viewer와 bridge 토큰은 달라야 합니다. 기존 서비스 viewer 토큰은 gateway에만 남습니다. 새 토큰을 브라우저 localStorage에 저장하지 않습니다. loopback HTTP라 cookie Secure는 사용하지 않습니다. 외부 서비스화에는 TLS/접근 제어를 별도 설계해야 하며 현재 외부 노출하지 않습니다. 토큰 변경 시 이 독립 gateway만 재시작해야 합니다.

브리지 envelope:

```json
{
  "kind":"ROS_LIVE",
  "session":"bridge-boot-uuid",
  "sequence":1,
  "robot_id":"pcu",
  "ros_time_ns":"1234567890000000000",
  "clock_mode":"ros",
  "grids":{}, "paths":{},
  "depth":null, "camera_info":null,
  "metrics":{}, "unavailable":{}, "robot_pose":null
}
```

각 grid는 width/height/resolution/origin{x,y,yaw}/frame_id/stamp_ns/source_age_s/map_from_frame/cells_zlib를 포함합니다. `cells_zlib`는 row-major int8 배열입니다. HTTP `/api/state`에는 배열 대신 image_id와 PNG 경로·metadata가 제공됩니다. path는 map 좌표 points[[x,y],…], 원본 frame/stamp, renderable을 포함합니다. depth는 width/height/source_width/source_height/sample_stride/encoding/scale_m/unit/values_f32le_zlib/frame_id/stamp_ns/valid_fraction/rgb_registered를 포함합니다.

live는 ROS_LIVE만, test는 SYNTHETIC_ROS_TEST/GAZEBO_REPLAY만 허용합니다. 세션 교체와 단조 sequence로 늦은 패킷/과거 boot를 거부합니다. bridge counters의 received는 **HTTP 수신 패킷**, callbacks는 **ROS 구독 메시지**이며 렌더 프레임 수가 아닙니다.

## 처리량과 재연결

ROS 구독 depth1 최신 프레임, 변환 기본1Hz(최대3), 영상 기존 host5Hz, 상태 UI2Hz입니다. 지도 PNG는 셀이 변하지 않으면 재사용하고 이미지 id가 바뀔 때만 브라우저가 재수신합니다. map/Costmap 전송은 현재1Hz full snapshot이며 변경분 uplink 최적화는 남아 있습니다. 부분 ROS 갱신은 bridge에서 합칩니다. 여러 브라우저도 LIMO 인코딩을 추가하지 않습니다. gateway upstream poll worker는 브라우저 수와 무관한 고정 주기이며 브라우저별 다운로드 트래픽은 증가합니다.

브리지는 HTTP 실패 timeout2초, 지수 백오프0.5→8초, 큐 없이 최신 snapshot을 다시 보냅니다. depth overwrite/old-drop, ROS callbacks, upload bytes/errors, conversion time, gateway request/bytes/pair mismatch, 브라우저 render p95를 기록합니다. 네트워크 byte counters는 애플리케이션 집계이며 NIC 전체 wire bandwidth가 아닙니다.

## 향후 확장 (미구현)

동적 객체 계약 후보: `object_id`, `position{x,y,z}`, `frame_id`, `observation_stamp_ns`, `confidence`, `tracking_state`(tentative/confirmed/lost), `expires_at_ns`, `source_frame_key`, `covariance`. RGB-D 등록·보정/카메라 3D 위치·TF·추적 ID 지속성·TTL 만료를 검증한 뒤에만 map layer를 활성화합니다. 현재 UNIMPLEMENTED 상태입니다.

원격 정지는 별도 인증/권한/명령 확인·결과 확인·통신 장애 정책·물리 E-stop 관계를 먼저 설계해야 합니다. 이 대시보드에는 구현하지 않았고 기존 Safety Gate도 바꾸지 않았습니다. RMF는 현재 UNAVAILABLE이며 Mission Manager의 mission/robot/event 조회 계층만 시각화합니다. Fleet Adapter나 RMF task control을 구현했다고 보지 않습니다.
