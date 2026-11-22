# 실제 LIMO 영상 → 노트북 YOLO → 결과 반환 시험 (준비 문서)

**상태: 모든 실차 항목 UNVERIFIED.** 아래 절차의 스크립트는 모의 환경(모의 로봇 + 재생 카메라 소스)에서 실행해 동작을 확인했지만, 실제 LIMO에서는 한 번도 실행하지 않았습니다. 이 문서에 있는 토픽 이름·도메인·Python 환경은 **내일 확인한 뒤** `.env`에 적습니다. 확인하지 못한 값은 추측하지 마세요.

## 1. 목적

실제 DaBai RGB 영상을 Tailscale로 노트북에 보내 YOLO11n(RTX 5060)으로 검출하고, 결과가 LIMO로 되돌아오는지 확인하며, 다음을 **실제로** 측정합니다: 영상 FPS, 프레임 손실률, 영상 대역폭, 추론 시간, 송신→결과 왕복 시간, LIMO CPU·RAM, 연결 중단 감지·재연결 시간, 오래된 결과 폐기 수.

범위 밖: 거리(Depth) 추정, 검출 결과의 Nav2 costmap·Safety Gate 연동(연결하지 않음), 모터 구동, 임무 실행.

## 2. 안전 전제 (먼저 읽기)

- 첫 시험은 **모터 잠금 상태**(`allow_motor_drive`는 기본값 false)로만 진행합니다. 송신 스크립트는 `/cmd_vel`에 발행자가 있으면(모터 경로 ARMED) 시작을 거부합니다.
- 송신·수신 노드는 이미지 토픽을 읽고 결과를 JSON/표준 메시지로 내보낼 뿐입니다. 속도 명령을 발행하지 않고 Nav2·Safety Gate·Robot Agent를 수정하지 않습니다. 검출 결과를 소비하는 자동 제어는 없습니다.
- 기존 LIMO 설정·팀 워크스페이스를 수정하지 않습니다. LIMO에는 새 폴더(`~/limo_video_kit`)만 만듭니다(이미 있으면 중단).
- 사람이 로봇 곁에서 E-Stop을 쥐고 진행하는 것을 권장합니다.

## 3. 준비물과 전제

- 노트북(이 저장소와 영상·AI 실험 폴더): Tailscale 실행 중, GPU 드라이버, `yolo_gazebo_limo/`(가중치 `weights/yolo11n.pt`, venv), `real_limo_kit/`.
- LIMO(`pcu`): Tailscale 연결과 SSH 접속(과거 검증 완료, 오늘 재확인 안 함), ROS 2 Humble, DaBai 카메라가 구동 중이거나 구동할 수 있어야 함. LIMO에서 필요한 Python 모듈: `rclpy`, `numpy`(필수), `cv2`·`vision_msgs`(선택). `20_limo_preflight.sh`가 확인합니다.
- `.env`에 채울 값(자리표시자 `<...>`): `AI_BIND_HOST`(노트북의 Tailscale 주소, 공인/와일드카드 금지), `AI_HOST_ADDR`(LIMO가 접속할 같은 주소), `LIMO_SSH`, `IMAGE_TOPIC`, `IMAGE_COMPRESSED`, 필요하면 `LIMO_ROS_DOMAIN_ID`. 실제 주소는 이 문서나 Git에 쓰지 않습니다.

## 4. 실행 순서

### A. 노트북 (오늘 이미 가능, 내일 `AI_BIND_HOST` 수정 후 다시)
```bash
cd real_limo_kit
./00_gen_env.sh                       # .env/토큰 생성(이미 있으면 건너뜀)
$EDITOR .env                          # AI_BIND_HOST, AI_HOST_ADDR, LIMO_SSH 를 채움 (아직 IMAGE_TOPIC 은 비워둠)
./10_start_ai_host.sh                 # AI 호스트: 0.0.0.0 으로는 시작하지 않음
./11_start_dashboard.sh && ./13_info.sh   # 대시보드 주소 확인, 브라우저로 열기 (DASHBOARD_GUIDE.md)
./20_limo_preflight.sh laptop         # Tailscale 상태, GPU, 모델, 시계 확인
```
`TRUST_SENDER_CLOCK`는 기본 0입니다(도착 시각 기준으로만 오래된 프레임 판단). 두 장치 시계가 NTP로 맞는 것을 확인한 뒤에만 `TRUST_SENDER_CLOCK=1`을 고려하세요.

### B. LIMO로 파일 배포 (내일, 실제 LIMO 필요)
```bash
./deploy_to_limo.sh --print-only      # 무엇이 복사되는지 먼저 확인
./deploy_to_limo.sh                   # ~/limo_video_kit 새 폴더에만 복사. 송신 토큰은 0600
```

### C. LIMO 사전 점검 (LIMO에서, 읽기 전용)
```bash
ssh <LIMO_SSH>
cd ~/limo_video_kit/real_limo_kit
./20_limo_preflight.sh limo
```
점검표(결과는 `results/preflight_limo_*.txt`):

| 항목 | 기대 | 결과 기록 |
|---|---|---|
| ROS 환경(`ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`) | 카메라/Nav2가 쓰는 값과 같을 것 | UNVERIFIED |
| Python 모듈 | `rclpy`, `numpy` OK | UNVERIFIED |
| 이미지 토픽 후보(Image / CompressedImage) | 목록에서 **본인이** 선택 | UNVERIFIED |
| RGB 압축 토픽(`.../compressed`) 존재 여부 | 있으면 우선 사용 | UNVERIFIED |
| Depth 토픽, camera_info | 이번 시험에서는 목록 확인만 | UNVERIFIED |
| 선택 토픽의 발행 속도(`ros2 topic hz`)와 QoS | 카메라 자체 fps 기록 | UNVERIFIED |
| TF 프레임(`/tf_static`) | 카메라 프레임 존재 | UNVERIFIED |
| Nav2/Safety Gate 상태, `/cmd_vel` 발행자 수 | 모터 경로 LOCKED(발행자 0) | UNVERIFIED |
| CPU·RAM·온도 | 시험 전 기준값 기록 | UNVERIFIED |
| 시간 동기화(`NTPSynchronized`, chrony) | yes | UNVERIFIED |
| 노트북 AI 호스트 TCP 접근 | OK | UNVERIFIED |

**토픽 선택:** 카메라 드라이버가 JPEG 압축 토픽(`sensor_msgs/CompressedImage`)을 제공하면 그것을 쓰세요(`IMAGE_COMPRESSED=true`): 재인코딩이 없어 LIMO CPU가 줄고, 모의 시험에서 원본 영상(약 0.9 MB/프레임)을 ROS로 전달하는 구간에서 20~40 % 프레임 유실이 있었기 때문에(압축 토픽에서는 사라짐) 압축 토픽이 유리할 것으로 예상하지만 **실제 LIMO에서는 미검증**입니다. 압축 토픽이 없으면 원본 토픽을 쓰고(`IMAGE_COMPRESSED=false`, `JPEG_QUALITY`, `SCALE`로 조절) 압축 토픽을 만들 수 있는지(`image_transport republish`)는 별도로 검토합니다. `format`에 `jpeg`가 없는 압축 토픽(예: PNG, depth 압축)은 지원하지 않습니다.

### D. 송신·수신 시작 (LIMO에서)
```bash
# ~/limo_video_kit/real_limo_kit/.env 의 IMAGE_TOPIC / IMAGE_COMPRESSED 를 preflight 결과로 채운 뒤
./21_limo_start_sender.sh             # /cmd_vel 발행자가 있으면(ARMED) 거부. 토픽에 발행자가 없으면 거부
./22_limo_start_consumer.sh           # 읽기 전용 결과 소비 노드 (/ai/agent_summary)
```

### E. 노트북에서 확인
```bash
./30_laptop_check_roundtrip.sh        # 프레임 수신 / YOLO 결과 / 결과가 로봇에 되돌아옴 / 타임스탬프 일치를 PASS/FAIL
```
브라우저(대시보드, [DASHBOARD_GUIDE.md](DASHBOARD_GUIDE.md))의 AI 패널에서 영상과 박스를 눈으로 확인합니다. LIMO에서 결과 확인: `ros2 topic echo /ai/agent_summary`(FRESH/STALE/NO_DATA, `fresh_detections`, `last_frame_id`, `last_stamp`).

### F. 측정 (노트북)
```bash
./40_record_metrics.sh limo_run1 180  # 180 s 기록 -> results/limo_run1/summary.md
```
`summary.md`에 노트북 쪽(FPS, 대역폭, 추론, 호스트 처리, CPU·GPU)과 LIMO 쪽(송신 노드가 보내는 텔레메트리: 프레임 손실, 폐기, 오래된 결과 차단, 왕복, CPU·RAM)이 함께 기록됩니다. 설정을 바꿔 비교하려면(FPS 10/15, `JPEG_QUALITY`, `SCALE`) LIMO에서 `.env`를 바꾸고 송신기를 재시작합니다.

### G. 종료와 복구
```bash
# LIMO: ./50_stop_all.sh        노트북: ./50_stop_all.sh      (각자 자기 스크립트가 시작한 프로세스만 종료, 로그는 results/archive_*)
```
LIMO의 기존 프로세스(카메라, Nav2, Safety Gate)는 건드리지 않았으므로 되돌릴 것이 없습니다. 복사한 폴더를 지우려면 사용자가 직접 `~/limo_video_kit`을 삭제하세요(스크립트는 삭제하지 않음).

## 5. 측정값의 의미와 주의

- **왕복 시간(로봇 측정):** 로봇이 프레임을 보낸 시각부터 결과를 받은 시각까지, **로봇의 시계 하나**로 잰 값입니다. 시계 동기가 필요 없습니다.
- **카메라 콜백→결과 수신:** 송신 노드가 이미지를 받은 시각부터 결과를 받을 때까지(한 시계). 센서 노출·드라이버 지연은 포함되지 않아 촬영부터의 종단 지연이 **아닙니다**(별도 측정 수단 필요, 미측정).
- **단방향 지연:** 두 장치 시계가 맞지 않으면 주장하지 않습니다. 요약의 "시계 오차 추정"은 ping/pong에서 계산한 추정치(±RTT/2)이며 동기화가 아닙니다.
- **프레임 손실:** (a) 송신 노드가 카메라에서 받은 뒤 보내기 전에 버린 수, (b) 송신한 수 대비 노트북이 받은 수(링크 손실). **카메라 드라이버에서 송신 노드까지 ROS 안에서 잃은 프레임은 보이지 않습니다**: 송신 노드의 `src_fps`를 카메라 자체 발행 속도(`ros2 topic hz`)와 비교하세요.
- LIMO CPU·RAM은 송신 노드가 `/proc`에서 읽어 보냅니다(전체 CPU %, 메모리, 송신 프로세스). 보조로 `tools/sysmon.py`(CSV)가 있습니다.

## 6. 실패 대응

| 증상 | 원인 후보와 조치 |
|---|---|
| `21`이 "ABORT: /cmd_vel has N publisher" | 모터 경로 ARMED. 시험 중단, 모터 잠금 상태로 되돌린 뒤 재시도(우회하지 말 것) |
| "ABORT: ... has no publisher in domain" | 도메인(`LIMO_ROS_DOMAIN_ID`)·토픽 이름 확인, 카메라 구동 여부, `20_limo_preflight.sh` 결과 |
| 송신 노드가 바로 종료 | `run/sender.log`: `rclpy` import 실패면 ROS 환경 소싱 문제, 파라미터 타입 오류 등 |
| 노트북에 프레임이 안 옴 | `AI_BIND_HOST`가 루프백이면 LIMO가 접속 못 함. Tailscale 연결(`tailscale status` 읽기 전용), 포트 `AI_LINK_PORT` 방화벽, 토큰 파일 |
| 연결은 되는데 raw 이미지가 안 보내짐 | 카운터 `no_cv2_for_raw_image`: LIMO에 `cv2`가 없음 → 압축 토픽 사용(`IMAGE_COMPRESSED=true`) |
| 프레임이 오는데 `unsupported_image` | 인코딩이 `rgb8/bgr8/mono8`이 아니거나 압축 포맷이 JPEG가 아님 |
| LIMO CPU가 높음 | FPS·`SCALE` 낮추기, `JPEG_QUALITY` 낮추기, 압축 토픽 사용 |
| 결과가 `STALE`/`NO_RESULT` | 링크 지연/끊김 또는 AI 호스트 과부하. 노트북 `run/ai_host.log`, GPU 사용률 확인. 자동으로 재접속하며(1→5 s 백오프) 오래된 결과는 폐기됩니다 |
| 이상 징후 시 | `./50_stop_all.sh` 후 원인 기록. LIMO의 기존 시스템에는 영향이 없도록 설계됨 |

## 7. 시험 결과 기록 양식 (내일 채움; 측정하지 않은 칸은 UNVERIFIED로 둡니다)

| 항목 | 값 | 비고 |
|---|---|---|
| 날짜 / 시험 번호 | | |
| 선택한 토픽, 타입, 압축 여부 | | |
| 카메라 발행 속도(`ros2 topic hz`) | UNVERIFIED | |
| 설정(fps 상한, 해상도, JPEG 품질) | | |
| 노트북 수신 FPS | UNVERIFIED | `summary.md` |
| 송신 노드 src_fps / 전송 FPS | UNVERIFIED | |
| 로봇 측 폐기 / 링크 손실률 | UNVERIFIED | |
| 영상 대역폭(kbit/s) | UNVERIFIED | |
| YOLO 추론(평균/p95, ms) | UNVERIFIED | |
| 왕복(로봇 측정, 평균/p95, ms) | UNVERIFIED | |
| 노트북 CPU % / GPU % / GPU 메모리 | UNVERIFIED | |
| LIMO CPU % / RAM / 송신 프로세스 CPU % | UNVERIFIED | |
| 두 장치 시계(NTP 동기 여부, 오차 추정) | UNVERIFIED | 단방향 지연은 주장하지 않음 |
| 연결 중단 시험: 감지 시간 / 재연결 시간 | UNVERIFIED | 송신 프로세스/네트워크 끊어서 측정(Wi-Fi·Tailscale 설정 변경 없이 노트북 AI 호스트 재시작 등) |
| 오래된 결과 차단 수 | UNVERIFIED | |
| 검출 결과(예: person/chair 신뢰도), 화면 스크린샷 | UNVERIFIED | |
| 모터 상태(잠금 유지 확인) | | 시험 내내 `/cmd_vel` 발행자 0 |
| 문제와 조치 | | |
