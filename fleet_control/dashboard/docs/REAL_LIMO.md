# 실제 LIMO 읽기 전용 연동 — 2026-11-13

기존 합성 시험과 **다른 실제 실차 데이터 시험**입니다. Claude의 실차 작업 폴더와 실행 스택은 수정하지 않았습니다. 기존 Mission Manager/Agent를 대체하지 않습니다.

## 결과와 현재 상태

|항목|실차 결과|
|---|---|
|`/map`|실제 SLAM 지도 수신·웹 표시 성공, 브리지 구간13회 callback|
|Global/Local Costmap|실제 OccupancyGrid 수신·반투명 표시 성공, 각각11/21회 callback|
|로봇 위치·방향|실제 map→base_footprint TF 표시 성공; 기존 Agent OFFLINE과 별도로 표시|
|DaBai Depth|실제16UC1 640×400 수신, 320×200 샘플링 컬러맵 표시 성공|
|Nav2 Global Plan|`/plan` publisher는 있지만 관측 구간 메시지0: **UNAVAILABLE**|
|Local Plan|활성 publisher 없음: **UNAVAILABLE**|
|Frontier 후보/선택 목표/탐색 상태|활성 publisher 또는 명시적 선택 목표 토픽 없음: **UNAVAILABLE**|
|RGB·YOLO|실제 RGB → 기존 GPU YOLO →18210 표시·원본 프레임 정합 성공. 이 장면 검출0개|

두 번의 **각15초 제한 시험**은 종료했고 신규 시험 프로세스만 자동 종료했습니다. 따라서 현재18210의 실제 센서 데이터는 시간이 지나면 **STALE**, RGB 송신은 **SENDER_DISCONNECTED**가 정상입니다. 상시 스트리밍을 켜놓았다고 주장하지 않습니다. Depth/지도와 RGB는 서로 다른 시간 구간에서 시험했으며 동시 장시간 부하는 미검증입니다.18211은 기존 합성 ROS·저장 Gazebo RGB 시험 화면으로 유지합니다.

## 승인·변경 범위

사용자가 승인한 후에만 실차 홈의 **새 폴더** `~/limo_dashboard_readonly_20261113/`를 생성했습니다.

- `dashboard/{__init__.py,core.py,bridge_node.py}`, `topics.json`, `MANIFEST.json`: 기존 독립 브리지 소스/설정, SHA256 검증. 토큰 미포함.
- 추가 승인 후 `rgb_readonly.py`, `RGB_MANIFEST.json`: 기존 RGB sender 통신 코드를 재사용하되 ROS 애플리케이션 발행을 제거한 독립 어댑터.
- 기존 `~/limo_video_kit`은 vlink 모듈·송신 토큰·설정을 **읽기만** 했습니다. 원본 sender, 기존 설정/토큰 파일을 덮어쓰지 않았습니다.
- 두 어댑터는 센서·주행·모터 명령 및 검출 토픽을 발행하지 않고 Action/제어 Service를 호출하지 않습니다. 소스 AST에서 명시적 `create_publisher/create_client/create_service/send_goal_async/publish` 호출 부재를 검사했습니다. rclpy 자체의 내부 parameter event 같은 ROS 관리 동작까지 네트워크상 완전 무발행이라고 주장하지 않습니다.
- 카메라·Agent·Nav2·SLAM·Safety Gate·자동 맵핑 런처·팀 workspace·Tailscale은 변경/재시작하지 않았습니다. 원격 임무 제출/취소·Gate reset·모터 명령 없음. 패키지 설치, commit/push/PR 없음.
-18210/18211/18190은 기존 PID를 유지했습니다.8080은 조사 시작 시 listener가 없었고 새로 실행하지 않았습니다.

## 실제 인터페이스와 좌표

|데이터|타입|확인한 publisher QoS|frame|
|---|---|---|---|
|`/map`|nav_msgs/OccupancyGrid|Reliable/TransientLocal|map|
|`/global_costmap/costmap`|nav_msgs/OccupancyGrid|Reliable/TransientLocal|map|
|`/local_costmap/costmap`|nav_msgs/OccupancyGrid|Reliable/TransientLocal|odom|
|두 costmap `_updates`|map_msgs/OccupancyGridUpdate|Reliable/TransientLocal|해당 grid frame|
|`/plan`|nav_msgs/Path|Reliable/Volatile|메시지 미수신으로 frame/timestamp 미검증|
|`/camera/color/image_raw`|sensor_msgs/Image|BestEffort/Volatile|camera_color_optical_frame|
|`/camera/depth/image_raw`|sensor_msgs/Image|BestEffort/Volatile|camera_color_optical_frame|
|`/camera/depth/camera_info`|sensor_msgs/CameraInfo|BestEffort/Volatile|camera_color_optical_frame|

실제 graph 조사에서 endpoint depth가0으로 보고돼 KEEP_LAST queue depth를 확정하지 않았습니다. QoS reliability/durability만 위처럼 확인했습니다. Frontier를 생성하거나 Nav2 목표를 보내서 없는 데이터를 만들지 않았습니다.

지도와 Global Costmap은 둘 다197×233, resolution 약0.050000000745m, origin 약(-1.175546,-1.858329), yaw0으로 일치했습니다. Local은80×80(4×4m), origin 약(-2.35,-1.85), yaw0의 odom grid였습니다. **Local 메시지 stamp의 map←odom TF** 약(x0.34084,y0.08094,yaw0)을 구해 합성했습니다. 실제 robot TF는 약(-0.009,0.238,yaw1.329rad)였고 관측 시 TF age 약171ms였습니다. ROS grid origin/yaw, TF, PNG row 뒤집기, 화면 y축 변환을 유지합니다. 화면 회전은 보기만 바꿉니다.

정지 시험이라 rolling window가 큰 거리로 이동하는 경우와 실제 clearing 장애물 변화는 재현하지 않았습니다. Grid 부분 갱신/clearing은 기존 자동 검사로 검증한 범위와 구분합니다. `/plan` 메시지가 없어 경로 frame의 실제 정합은 미검증입니다.

실차 Agent가 OFFLINE이므로 Agent의 과거 좌표와 새 SLAM 지도를 혼합하지 않도록 UI를 보완했습니다. fresh ROS TF를 별도 출처로 표시하고 bridge 종료 후에는 **그 ROS 좌표를 STALE로 유지**합니다. TF가 없거나 비정상 값이면 과거 Agent 좌표를 새 지도 위에 대입하지 않습니다. Agent 연결·Gate 상태를 새 브리지 연결 성공만으로 ONLINE/NORMAL로 바꾸지 않습니다.

## Depth 인코딩·거리 제한

실제 depth는16UC1, little endian, width640/height400/step1280입니다. CameraInfo도640×400이며 K는fx=fy454.8289, cx330.5996, cy242.0512였습니다. RGB·Depth header frame 이름이 같다는 것만으로 픽셀 등록·보정이 검증된 것은 아닙니다.

센서의 **장치 단위·배율을 독립 검증하지 못했습니다**. 0.001m 기본 배율은 ROS/OpenNI 관례를 적용한 가정이며 웹에 “가정 배율·거리 정확성 미검증”을 표시합니다. 줄자·장치 보정과 대조하지 않았으므로 m 표시에 대한 정확도 주장은 하지 않습니다. 원본 raw depth 유효 비율은 구간별 약36~40%였고 0은 무효로 구분했습니다. RGB-D 정렬, 3D 객체 위치, 동적 객체 지도 추적은 미검증/미구현입니다.

## 측정값

서로 다른 구간과 측정 위치를 구분합니다. CPU100%는 한 코어이며 RSS는 표본값입니다.

###12초 읽기 전용 ROS 조사

- RGB raw: 약26.1fps, payload 약20.0MB/s, 같은 실차 시계 기준 header→callback age 중앙1.47ms/p95 2.29ms.
- Depth raw: 약29.0fps, payload 약14.8MB/s, age 중앙1.03ms/p95 1.71ms.
- 위 raw payload는 **실차 내부 ROS 구독 바이트**이며 Tailscale 인터넷 전송량이 아닙니다.
- `/map` 약1.02Hz(연속 간격 기준), Global 약0.82Hz, Local 약1.69Hz.
- 실차 RAM 총7514MiB/가용4902MiB, 시스템 CPU14.44%(전체 CPU 기준, 조사 중 표본).
- 기존 카메라 CPU68.94%(1core), RSS157.1MiB. 기존 맵핑 감독 프로세스 CPU23.56%, RSS89.7MiB. 이 값은 신규 브리지의 추가 부하를 뜻하지 않습니다.
- `/cmd_vel_locked` 약50Hz, 관측된 최종 선속도·각속도0, `/cmd_vel` publisher0, 실행 중 Gate의 locked remap과 맵핑 `--drive` 부재 확인. 소프트웨어 경로 관찰이며 물리 비상정지 성능 시험은 아닙니다.

###15초 지도·Depth 브리지

- 실제15개 packet 수신, 전송 실패0. 원격 timeout 종료124는15초 제한의 정상 결과이며 로컬 전체 실행 약16.2초(SSH/정리 포함).
- 브리지 Python CPU약21.01%(1core), 최대 관측 RSS78.17MiB. 별도 timeout wrapper PID는CPU0%/RSS1.88MiB로 분리합니다.
- 마지막 변환3.43ms, 원본 Depth callback408개 중 오래된 큐를 누적하지 않고 최신 프레임 우선. pending frame 교체394회, stale depth drop0, upload error0.
- JSON 앱 payload 약38.34kB/s.18초 NIC 표본의 Tailscale TX 약35.24kB/s, 물리 무선 TX 약39.20kB/s(다른 트래픽 포함, 서로 중복 계층이라 더하지 않음).
- 웹 Depth 표시 약0.8~1.2fps, canvas 약2.6fps/render p95 0.90ms(실제 브라우저 관측).
- 로컬 sink ACK 시간은 SSH RTT/센서→브라우저 지연이 아닙니다. 장치 간 단방향 지연 정확도는 검증하지 않았습니다.

###15초 RGB·기존 YOLO

- 설정 상한5fps/scale0.5/JPEG quality70. 실제 웹 수신은 최대5.0fps, 관측 화면3.4fps;320×200 표시.
- 종료 직후 최근5초 window159.6kbps(약19.95kB/s). 이는 해당 window 표본이며15초 전체 평균이 아닙니다.
- 같은 송신기 telemetry: 결과56개 성공/원본 stamp 일치56회, stale 결과 차단/송신 전 drop0, source slot 교체52회. gateway polling 관측26회에서 fresh+paired였습니다(영상 프레임 수가 아님).
- 송신→결과 수신 RTT 평균30.2ms, 원본 capture stamp→결과 수신 평균85.8ms(실차 동일 시계의 왕복 처리 경로). YOLO host의 표시 평균9.65ms는 기존 실행 이력을 포함한 rolling 평균입니다.
- 실차 RGB 어댑터 CPU4.0%(1core), RSS100.6MiB, 시스템 CPU11.7%, RAM 사용2633/7514MiB 표본. Depth 브리지와 **동시에** 측정한 값이 아닙니다.
- clock offset 추정-9.6ms/불확실성±2.7ms/6표본은 동기화 완료의 증거가 아니며 정밀 단방향 지연값으로 쓰지 않습니다.
- 실제 장면 검출0개이며 통신 실패가 아닙니다. 원본 frame_id/timestamp 정합과 fresh 영상 확인. 임의 bbox/지도 장애물 생성 없음.

## 다시 실행하는 명령

아래는 사용자가 준비 상태·기존 송신자 중복·모터 잠금 확인 후 실행할 **15초 제한 명령**입니다. 자동 시작이나 서비스 등록은 하지 않았습니다.

###노트북: 실차 지도·Depth → 기존18210

```bash
cd /path/to/Capstone1_limo
python3 fleet_control/dashboard/real_trial.py \
  --connection-config "$HOME/.local/state/capstone-dashboard-visualization/real_connection.json" \
  --seconds 15
```

개인 connection config는0600이고 `ssh_target`, `setup_files` 목록을 담습니다. 실제 주소·토큰은 Git/문서에 넣지 않습니다. 이번 시험과 동일한 전송 로직에 CLI/개인 출력 폴더를 추가한 도구이며, CLI/구문 검사는 통과했지만 **이 정리된 CLI 자체로 추가 실차 재실행은 하지 않았습니다**. 실차에는 배포된 bridge를 사용하며 새로운 파일 배포나 Agent 재시작이 필요 없습니다. 실차 ROS domain77/localhost-only, 기존 slow stack underlay를 source합니다. 로그·마지막 패킷·summary는 private `~/.local/state/capstone-dashboard-visualization/real_trials/<date_run>/`에 생성합니다.

###실차 터미널: 기존 RGB → 기존 YOLO

```bash
source /opt/ros/humble/setup.bash
set -a
source "$HOME/limo_video_kit/real_limo_kit/.env"
set +a
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$HOME/limo_video_kit${PYTHONPATH:+:$PYTHONPATH}"
timeout --signal=INT --kill-after=3 15s python3 \
  "$HOME/limo_dashboard_readonly_20261113/rgb_readonly.py" --ros-args \
  -r __node:=dashboard_rgb_readonly \
  -p robot_id:="$ROBOT_ID" -p stream_id:=dashboard_readonly_rgb \
  -p image_topic:=/camera/color/image_raw \
  -p server_host:="$AI_HOST_ADDR" -p server_port:="$AI_LINK_PORT" \
  -p token_file:="$HOME/limo_video_kit/real_limo_kit/secrets/sender.token" \
  -p fps:=5.0 -p jpeg_quality:=70 -p scale:=0.5
```

기존 `.env`의 IMAGE_TOPIC은 현재 실차 graph와 달랐습니다. 첫 시도는 **파일 배포·실행 전에** 중단됐고 기존 설정을 수정하지 않은 채 새 실행 인자만 위 실제 토픽으로 지정해 성공했습니다. 기존 sender가 활성 상태면 중복 실행하지 마세요. 이번 실차 시험은 동일 인자를 Python launch wrapper로 전달했습니다.

###화면

[실차18210](http://127.0.0.1:18210/) / [합성18211](http://127.0.0.1:18211/). 기존 viewer token 파일로 로그인합니다. 새 브리지 중단 후 stale가 되는 것을 실패로 감추지 않습니다.

## 회귀·증거와 남은 작업

새 live fusion 검사6개(Agent OFFLINE 보존, source 분리, bridge 종료 후 ROS pose STALE, TF 부재/비정상 좌표 숨김), 기존 geometry6개와 visualization31개를 재실행해43/43 PASS했습니다. 상세 결과는 `docs/evidence/real-limo/`에 보관합니다. 애플리케이션 발행·제어 호출 부재 AST 검사를 두 어댑터에 적용합니다.

실차 화면: `screenshots/actual-map-depth.jpg`, `screenshots/actual-rgb-yolo.jpg`. 기존 합성 스크린샷은 별도로 보존합니다.

남은 항목: 실제 계획 경로·Frontier 메시지 발생 시 frame/시각/TF 검증, DaBai 배율·거리 실측·등록 검증, 움직이는 rolling origin/clearing 관찰, RGB+Depth 동시 지속 부하·재연결, 새 브리지 lifecycle 운영 정책. 임무나 Frontier를 인위적으로 시작하지 않았습니다. 실제 Robot Agent가 중단돼 있으므로 중앙 관제 상태·Gate 상태 동기화는 이 구독 브리지의 성공과 별개이며 기존 Agent를 임의 재시작하지 않았습니다.
