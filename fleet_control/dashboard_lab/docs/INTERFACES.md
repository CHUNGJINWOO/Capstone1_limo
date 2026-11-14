# Lab API·데이터·좌표 계약

모든 서버는 loopback입니다. 실차 SSH/ROS 조회/명령 인터페이스가 없으며, `GET`로 명시 설정한 로컬 Mission Manager를 읽는 선택 기능만 제공합니다. 기본 실행은 외부/운영 upstream 없이 SIMULATION입니다.

## 인증과 권한

0600의 `viewer.token`과 `lab.token`은 다릅니다. POST `/api/login`은 같은 origin에서만 토큰을 받아 HttpOnly/SameSiteStrict cookie를 발급합니다. 세션은0700 private state의0600 `sessions.json`에 저장해 서버 재시작 후에도 유효합니다. 로그아웃은 해당 세션을 폐기합니다. 토큰을 URL·브라우저 localStorage·응답에 넣지 않습니다. 직접 API 클라이언트는 Bearer를 사용할 수 있습니다.

- viewer: 센서/상태/임무 읽기, WebSocket/WebRTC 수신, 로컬 성능 표본 제출.
- lab: 위 권한과 **SIMULATION/REPLAY 조건·품질 프로파일 변경만** 가능.
- LIVE에서 장애 주입 거부. 모든 역할에서 임무 제출/취소, motor/Gate/reset API는405/미제공.
- loopback HTTP 실험입니다. 외부 공개/TLS·STUN·TURN 운영 구성은 하지 않았습니다.

## HTTP

|경로|내용|
|---|---|
|GET `/health`|read_only, 현재 media boot|
|POST `/api/login`, `/api/logout`|인증/세션 폐기|
|GET `/api/state`|source_mode, mission, visualization, rmf, media, role, capabilities|
|GET `/api/clock`|server_ms; 브라우저 최소RTT 기준 offset/불확실성 추정|
|GET `/api/metrics`|단계별 원시 통계·counter·브라우저 표시 표본|
|POST `/api/telemetry`|로컬 브라우저 계측 기록; 로봇 제어 아님|
|GET `/api/grid/{map,global_costmap,local_costmap}.png`|원래 grid origin/resolution/yaw·map_from_frame 기반 이미지|
|GET `/api/map/{robot}.png`|선택적으로 설정한 loopback Mission Manager 지도 GET proxy|
|GET `/api/frame-meta?seq=`|최근128개 RGB frame metadata; WebRTC 표시 프레임과 대조|
|GET `/api/depth/raw?key=`|같은 depth_key의 zlib 무손실 배열, 만료 시410|
|POST `/api/offer`|로컬 H.264/WebRTC SDP 협상; LIVE에서 거부|
|POST `/api/lab/control`|lab 전용 mode/fault/profile/adaptive 설정|

baseline18311은 원래 gateway/UI를 별도 실행하며 `/api/benchmark` 계측만 추가했습니다. 최적화 서버의 `/api/v1/dashboard`, `/api/v1/status`, `/api/v1/frame.jpg`는 baseline을 위한 인증된 동일 소스 alias입니다. 이 경로가 실차/운영 서버라는 뜻은 아닙니다.

## 원자적 RGB/Depth 프레임

`/ws/rgb`, `/ws/depth`, `/ws/h264`의 binary message:

```text
uint32 big-endian JSON_header_length | UTF-8 JSON header | encoded payload
```

native TCP18313도 같은 envelope를 uint32 길이로 한 번 더 감싸며 첫 JSON hello에 viewer 토큰/codec을 보냅니다. 비교용 로컬 프로토콜이고 기존 실차 vlink를 교체하지 않았습니다.

헤더의 핵심 필드:

```json
{
  "boot":"source-boot-uuid", "stream":"rgb", "sequence":123,
  "frame_id":"stored_frame.jpg", "stamp_ns":"1794572440657407536",
  "capture_wall_ms":1794572440657.4075,
  "encoded_wall_ms":1794572440660.0, "send_wall_ms":1794572440661.0,
  "codec":"jpeg", "width":480, "height":270,
  "source":"RECORDED_GAZEBO_RGB", "mode":"SIMULATION",
  "original_sha256":"...", "measurement_overlay":true,
  "result_key":["source-boot-uuid","rgb",123,"stored_frame.jpg","1794572440657407536"],
  "inference_mode":"PRECOMPUTED_CPU_REPLAY", "detections":[]
}
```

`stamp_ns`는 문자열이며 비교 전에 JS Number로 바꾸지 않습니다. source boot/stream/seq/frame_id/stamp_ns가 모두 일치할 때만 결과를 겹칩니다. TTL700ms, 중복·역순·이전 boot·불명확한 미래 시각을 차단합니다. viewer는 decode ACK를 반환하고 sender는 다음 최신 슬롯을 선택합니다. compositor에도 bitmap 한 장만 대기시키고 교체 시 이전 bitmap을 close합니다. 표시 스케줄이 늦어져도 처음 기다린 오래된 장면을 보여주지 않습니다.

## H.264 / WebRTC

공유 encoder는 baseline/zerolatency/no B-frame, GOP15, 목표600kbps입니다. TCP/WebCodecs는 Annex-B packet과 정확한 decode timestamp를 받습니다. 새 수신자는 keyframe까지 기다립니다. WebRTC도 동일 encoded packet을 재사용하고 RTP 패킷화/암호화만 연결별로 수행합니다. 실제 codec 협상은 H.264로 제한했습니다.

로컬 계측 영상의 복사본 상단에는 sequence32bit+capture epoch ms 하위32bit 바코드를 넣습니다. 저장 원본을 덮어쓰지 않습니다. WebRTC는 디코딩된 영상의 **두 값 모두**를 metadata의 원래 nanosecond 문자열과 대조한 뒤 검출을 표시합니다. 손실 등으로 대응을 확인할 수 없으면 결과를 폐기합니다. 이 바코드는 실차 영상에 배포한 기능이 아닙니다.

## Depth

관제 JPEG와 무손실 거리 데이터는 별도입니다. `/api/depth/raw`의 `X-Depth-Meta`에 raw_dtype/encoding/scale_m/frame_id/stamp_ns/source_stamp_ns/depth_key/unit_verified/rgb_registered가 있습니다.

- SIMULATION: little-endian uint16,16UC1, scale0.001m,0은 invalid.
- REPLAY: 저장된 little-endian float32 metric sample,32FC1 운반 형식, NaN/0은 invalid. **실제 카메라가32FC1을 발행한다는 뜻이 아닙니다.** 이전 실차는16UC1이었지만 이번에 저장된 파생 float sample을 보존한 것입니다.
- REPLAY `unit_verified=false`: 이전 변환의 장치 배율/거리 정확도는 미검증. 원본 실차640×400 전체16bit 값을 재구성하지 않았습니다.
- `rgb_registered=false`: 같은 pixel 좌표로 RGB 객체 거리를 추정하지 않습니다.
- hover는 같은 depth_key의 정지 snapshot에서만 값을 읽고 키가 만료되면 UNAVAILABLE입니다. JPEG 색으로 거리값을 만들지 않습니다.

## 지도·다중 로봇·임무

기존 `dashboard.core`의 OccupancyGrid PNG row 변환·origin yaw·frame 변환 검사를 재사용합니다. 계획은 파란 점선, 실제 TF/표본 궤적은 민트 실선, 목적지는 원형, 자세는 화살표입니다. SIMULATION은 두 로봇이 같은 세계 프레임임을 명시합니다. 실제로 공통 map/TF가 확인되지 않은 여러 로봇을 임의로 같은 지도에 투영하지 않습니다. 선택 로봇과 다른 로봇의 계획은 표시하지 않습니다. REPLAY 실차 지도에 SIMULATION 목적지·로봇을 투영하지 않습니다.

선택적 LIVE Mission Manager 읽기 옵션은 다음입니다(이번 실차 미접속 작업에서는 사용하지 않음).

```text
--mode LIVE --mission-url http://127.0.0.1:PORT --mission-token-file /private/viewer.token
```

사용 경로는 dashboard,missions?limit=500,events?limit=500,robots/{id}/map.png의 GET뿐입니다. 실제 `events.level`을 INFO/WARNING/ERROR에 매핑하고 알 수 없는 값은 UNCLASSIFIED입니다. 임무 status는 API 원본이며 이력은 mission_id가 같은 관측 이벤트입니다. 최근500개 범위이므로 무제한 전체 DB 이력을 보장하지 않습니다. 기록이 없으면 이력 UNAVAILABLE입니다.

Adapter는 `--adapter-snapshot /private/adapter.json`으로 읽습니다.

```json
{"observed_at":1794572400.0,"registered":true,"fleet":"observed_fleet","robots":["robot_id"]}
```

5초 초과는 STALE이며 등록을 확정하지 않습니다. 미설정/불명확하면 UNAVAILABLE입니다. 실제 observer/heartbeat를 기존 Adapter에 넣거나 Adapter를 실행하지 않았습니다. SIMULATION의 등록 YES는 명시적 fixture일 뿐 실제 RMF 성공의 근거가 아닙니다.

## 장애 모델과 계측 해석

- RTT400은 순방향200ms 지연 주입이며 실제 ping400ms 측정이 아닙니다. bandwidth150은 스트림·클라이언트별150kbps 서비스 제한으로 전체 NIC 공유 대역폭 제한이 아닙니다.
- TCP: 프로그램 수준 지연/지터/대역폭/재전송 HOL 대기. 실제 커널 IP packet loss를 재현했다고 주장하지 않습니다.
- WebRTC: 실제 RTP datagram 송신 전 확률 손실. 손실 후 영상 복구를 관측했습니다. RTCP/NACK/keyframe 중 어느 기제가 복구에 기여했는지는 별도로 측정하지 않았습니다.
- disconnect:3초 media 무전송; 별도 서버 크래시 시험은 실제 TCP/WS/peer 연결을 끊었습니다.
- stalequeue: 지연+지터+손실+낮은 대역폭으로 압력을 가하지만 서버/브라우저 큐는 최신 한 장으로 유지합니다.
- 서비스시간·ACK EMA 피드백은 로컬 WebSocket 지연 시험으로 검증했습니다. 현장 네트워크의 안정성/품질 진동/재연결 정책은 아직 튜닝되지 않았습니다.

브라우저 지연은 저장 프레임 재생 생성→rAF/RVFC 표시 기회 추정입니다. 시계 오프셋 추정과 RTT/2 오차를 기록하며 실제 카메라 노출·디스플레이 광학 지연은 미측정입니다. GPU 추론8~10ms라는 과거 실차 결과를 이번 CPU 사전계산 스트리밍의 측정값으로 사용하지 않습니다.
