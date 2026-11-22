# 노트북 오프라인 검증 결과 — 2026-11-13

## 범위와 출처

실제 LIMO SSH·ROS graph·센서·모터·Agent에 접속하지 않았습니다. 기존 dashboard 원본은 보존하고 dashboard_lab만 추가했습니다. 운영8080/18190·기존18210/18211을 수정·종료하지 않았습니다. 새 서버는18310, 비교 서버18311, native 계측18313입니다.

SIMULATION은 합성 ROS bridge 스키마의 지도·Costmap·TF·임무·Adapter 상태와 저장 Gazebo RGB를 사용합니다. 이번에 Gazebo나 ROS middleware를 새로 실행하지 않았습니다. REPLAY는 이전 실차 지도·Costmap·파생 Depth 기록을 재생하며 동시 RGB-D가 아닙니다. LIVE는 연결 없음/UNAVAILABLE로 검증했습니다.

YOLO는 저장 JPEG6장의 CPU 사전계산 결과입니다. SHA와 정확한 frame key로 대응하지만 실시간 GPU 추론 비용을 포함하지 않습니다. 아래 capture는 **재생 프레임 생성**이며 실제 카메라 노출→화면 광학 지연이 아닙니다. 시계 오프셋 추정 기반 rAF/RVFC 표시 기회까지 측정했습니다. 각 단계 초반1초 제외, 짧은 표본의 P99는 장시간 안정성 보장이 아닙니다.

## UI 구현 및 확인

- 임무 목록·상태 필터·관측 이벤트 기반 이력, 이벤트 심각도/종류/로봇 필터 및 시간 정렬.
- 다중 로봇 카드, OFFLINE과 영상 STALE 독립 표시, 계획 파란 점선·실제 궤적 민트 실선·목적지/방향 마커.
- 공통 map/TF가 확인된 좌표만 투영. REPLAY 실차 지도에 합성 로봇 좌표를 겹치지 않음.
- 기존 지도·Global/Local Costmap·RGB·Depth·YOLO 패널, 레이어/투명도·확대·이동, 테마·접기·반응형 유지.
- Mission Manager는 선택적으로 loopback GET 읽기, Adapter는 timestamp 있는 읽기 전용 snapshot 계약. 실제 연결은 미검증, 합성 등록은 SIMULATION으로 표시.
- viewer/lab 역할 분리, 임무 제출·취소·모터 제어 API 없음. LIVE 장애 주입 금지.

실제 브라우저에서 두 로봇/오프라인 선택, SUCCEEDED 필터, 임무 이력, 경고·링크 필터와 시간 정렬, 영상·Depth 동일 프레임 거리 조회, REPLAY와 LIVE 미연결, LIVE→SIM 재연결을 확인했습니다. 모바일390px viewport에서 clientWidth/scrollWidth375px로 가로 넘침이 없었습니다. Depth 범위는0.2–4.0m 고정이며 입력 비활성입니다. 선택 로봇별 카메라 매핑·실제 Adapter heartbeat 생산자는 미구현입니다.

## 병목과 변경

기존 gateway5Hz polling, 웹2Hz polling, Depth 원래1Hz 제한이 지배적이었습니다. YOLO 추론8–10ms 기록만으로 화면 FPS를 설명할 수 없습니다. 원래 화면의 upstream FPS와 실제 표시 FPS를 분리했습니다.

새 경로는30Hz 재생 capture→최신 한 슬롯→공유 encoder→원자적 metadata+payload→decode ACK→최신 bitmap 한 슬롯→rAF입니다. 오래된 bitmap/AVFrame을 닫고 TTL700ms·boot·seq·frame_id·nanosecond 문자열로 오래된 영상/검출 결과를 거부합니다. 인코딩은 접속자 수와 무관하게 공유하며 전송/암호화 비용은 접속자 수에 비례합니다.

초기 다중/비활성 탭 계측에서 약1FPS가 관찰됐습니다. decode와 compositor를 분리하고 rAF 실행 시 최신 bitmap을 선택하도록 고쳤습니다. 이후 단일 전경 탭에서 아래 수치를 얻었습니다. 여러 탭·비활성 탭의 브라우저 스케줄 제한까지 해결됐다고 주장하지 않으며, 최종 스크린샷 순간 FPS도 정식 계측 구간 수치와 다를 수 있습니다.

## 브라우저 비교 — 단일 전경 표시 탭

동일 저장 입력·사전계산 YOLO 조건. baseline은 원래 UI/gateway의 별도 복사본이며 계측만 추가했습니다. 실제 실차 전송 전체의 전후 비교는 아닙니다.

|경로|RGB 표시 FPS|Depth 표시 FPS|RGB 평균 ms|P95 ms|P99 ms|
|---|---:|---:|---:|---:|---:|
|기존 polling,25초|1.96|1.00|214.42|373.30|436.04|
|JPEG / TCP WebSocket,11초 n131|11.90|7.99|22.68|37.71|39.98|
|H.264 / TCP WebCodecs,11초 n131|11.90|7.90|24.21|38.47|41.19|
|H.264 / WebRTC,11초 n132|11.82|7.97|60.02|72.38|77.00|

Depth 기존 평균387.70/P95588.30/P99595.11ms → JPEG 최적화 평균7.50/P9511.21/P9918.21ms(n88). RGB10–15FPS·Depth5–10FPS 목표는 이 로컬 전경 조건에서 달성했습니다. WebRTC가 가장 빠르지 않았으므로 기본은 JPEG입니다.

## 네이티브 전송 비교 — 브라우저와 별도 표본

각7초, payload kB는1000bytes이며 프로토콜/IP 총량이 아닙니다. 화질 동등성/PSNR 검사는 하지 않았습니다.

|프로파일/코덱|FPS|payload kB/s|생성→native decode P95 ms|
|---|---:|---:|---:|
|640px q75 15FPS JPEG|14.89|337.68|40.04|
|480px q65 12FPS JPEG|12.00|172.91|38.64|
|320px q45 8FPS JPEG|8.14|59.50|35.16|
|320px q35 5FPS JPEG|5.16|34.33|34.51|
|480px H.264 target600kbps|11.61|66.55|37.69|

balanced JPEG는 high 대비 약49%, H.264는 balanced JPEG 대비 약62% 전송량 감소. 네이티브 WebRTC 수신기는 별도8초 표본에서11.68FPS/P95128.75/P99136.42ms였으며 브라우저 수치와 혼합하지 않습니다.

## 단계별 시간 및 자원

|단계|평균 ms|P95 ms|P99 ms|
|---|---:|---:|---:|
|재생 capture 준비|0.404|0.766|1.048|
|JPEG resize/encode|2.275|3.519|4.159|
|Depth 색상/encode|2.867|4.148|6.108|
|H.264 encode|2.392|3.358|6.219|
|JPEG 브라우저 decode,정상|1.89|2.65|3.87|
|JPEG draw,정상|0.27|0.40|0.50|
|delivery service,장애 포함|28.225|204.336|489.699|

서버 계측110초: 최적화 프로세스 평균19.04%/최대21.8%(한 CPU core100% 기준), 최대RSS225.36MiB. baseline 평균1.77%/최대2.5%, RSS37.18MiB. 최적화 서버는 JPEG·Depth·H.264 및 fixture를 함께 생성하므로 동등 기능 CPU 비교가 아닙니다. 화면 프로세스 전체 CPU/RAM·LIMO 부하는 미측정입니다. GPU 순간 표본0%,674MiB는 시스템 전체 사용량이며 lab 귀속값이 아닙니다. CUDA 실시간 추론 미실행.

다중 접속 시험에서 encoder는 설정8FPS일 때7.89FPS로 유지되어 접속 수만큼 인코딩하지 않음을 확인했습니다. 총 NIC 트래픽은 미측정이며 위 payload와 서버 송신 counter를 제공합니다.

## 프로그램 수준 장애 주입

각 약8초, 첫1초 제외. 지연 수치는 **수신된 프레임만**의 지연이고 끊김 기간을 숨기지 않도록 FPS/결측을 함께 봅니다. TCP 손실은 transfer unit 재전송/HOL 모델이며 커널 IP loss가 아닙니다. RTT400은 순방향200ms 주입,150kbps는 스트림·클라이언트별 제한으로 전체NIC cap이 아닙니다.

|조건|RGB/Depth FPS|RGB P95/P99 ms|관찰|
|---|---:|---:|---|
|RTT400 모델|4.99 / 4.85|338 / 346|품질 하향, 최신 슬롯 유지|
|지터|7.85 / 7.99|163 / 172|지연 표시·적응|
|TCP loss10 모델|10.28 / 7.56|249 / 253|재전송 지연, 프레임 정합 유지|
|150kbps 모델|2.85 / 4.85|470 / 482|목표 FPS 미달, 품질 적응|
|3초 미디어 단절|8.14 / 5.57|40 / 47|결측 후 자동 수신 복구|
|누적 압력 stalequeue|0.43 / 1.14|620 / 622|700ms 초과 폐기, 목표 FPS 미달|
|정상 복귀|11.89 / 7.89|39 / 42|동일 브라우저 회복|

별도 WebRTC 시험에서 실제 RTP datagram153개를 송신 직전 폐기했고 loss 구간에 정합 확인된35프레임을 수신했습니다. 수신 age 약196–232ms. NACK/keyframe 중 복구 기여도를 구분하지 않았습니다.

독립 lab 서버만 실제 종료→재실행한 시험: 종료→HTTP ready11.38초(수동 도구 재실행 시간 포함), ready→첫 새 boot 표시4.33초, 합계15.71초. 브라우저 자동 재연결/기존 세션 유지 확인. 서버 자동 재시작 daemon을 구현했다는 뜻은 아닙니다.

## 자동 검사

- 새 Python 검사25/25 PASS, JS frame 정책6/6 PASS.
- 기존 회귀90 PASS: visualization31, geometry6, live_fusion6, dedupe14, RMF runner policy33.
- 합계121 PASS. 전체 실제 LIMO/ROS/SLAM/Nav2 회귀가 아니라 지정한 로컬 코드 검사 범위입니다.
- 기존 dashboard49개 원본 파일 SHA256 비교는 최종 보존 검사로 확인합니다. 브랜치 변경/commit/push/PR 없음.

## 남은 항목

실차 camera→웹 E2E, 실제 RGB-D 단위/정합, 매 프레임 YOLO GPU 비용, LIMO CPU/RAM/USB·안전 노드 자원 영향, 실제 Mission Manager/Adapter 연결과 live ROS Costmap/TF/Frontier는 이번 미검증입니다. 현장 검증은 [재연결 절차](REAL_LIMO_NEXT.md)를 따릅니다. H.264/WebRTC는 로컬 후보 구현이며 기존 실차 송신기를 교체하지 않았습니다. 거리 범위 조절, 로봇별 카메라 선택, 장시간 적응 진동/다중 탭 품질, 전체 NIC 및 브라우저 프로세스 자원 측정은 남아 있습니다.
