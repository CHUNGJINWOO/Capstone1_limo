# Capstone 대시보드 / RGB-D 전송 Lab

기존 `dashboard/`와 운영 서비스를 보존하는 **독립 노트북 실험 환경**입니다. 실제 LIMO 연결, SSH, ROS graph 조회, 장치 설정, 모터 명령, 임무 제출/취소를 하지 않습니다. 주행·센서·Gate 입력을 변경하는 코드가 없습니다.

## 화면과 실행

- 개선 UI: <http://127.0.0.1:18310/>
- 기존 동작 비교 UI: <http://127.0.0.1:18311/>
- native TCP 코덱 계측: loopback18313
- 개인 런타임: `~/.local/state/capstone-dashboard-lab/` (0700)
- 로그인: `viewer.token`은 읽기 전용, `lab.token`은 **SIMULATION/REPLAY 시험 조건만** 변경 가능. 두 파일은0600이며 값을 출력하지 않습니다.

이번 작업에서 transport 라이브러리는 `/tmp/capstone-dashboard-lab-venv`에만 설치했습니다. 시스템 패키지와 기존 ROS/YOLO 환경은 변경하지 않았습니다. 현재 시험 환경은 기존 시스템 NumPy1.21/OpenCV4.5/Pillow를 사용하며 `PYTHONNOUSERSITE=1`로 사용자 NumPy2와 OpenCV ABI 충돌을 피합니다. 별도 컴퓨터에서는 해당 의존성이 있는지 먼저 확인해야 합니다. 자동 apt 설치를 하지 않습니다.

```bash
cd /path/to/Capstone1_limo
export PYTHONPATH="$PWD/fleet_control"
export PYTHONNOUSERSITE=1
LAB_PY=/tmp/capstone-dashboard-lab-venv/bin/python
"$LAB_PY" -m dashboard_lab.run_local start --stay --python "$LAB_PY"
# 다른 터미널
"$LAB_PY" -m dashboard_lab.run_local status
"$LAB_PY" -m dashboard_lab.run_local stop
```

필요하면 새 venv를 사용자 경로에 만들고 `requirements-lab.txt`를 설치합니다. 이는 시스템 설치가 아닙니다. `--assets`로 저장 JPEG 폴더, `--recorded`로 저장 bridge packet을 지정할 수 있습니다. 실제 camera device는 열지 않습니다. 이번에 사용한 map/Depth packet은 원본을 보존하고 private state의 `recorded_map_depth.json`에 복사했습니다. 이 파일이 없으면 REPLAY 전환을 거부합니다.

`--stay`는 lab 프로세스만 감독합니다. 서버 크래시 후 프로세스 자체를 무조건 재기동하는 서비스는 만들지 않았습니다. 브라우저는 재연결하며 서버 세션은 private JSON에 보존됩니다. stop은 PID와 시작 시각이 일치하는 등록 프로세스만 종료합니다.8080/18190/18210/18211은 바인딩을 거부합니다.

## UI 기능

임무 목록·상태 필터·선택한 임무의 이력, 로봇/심각도/이벤트 분류 필터와 시간 정렬, 다중 로봇 카드, 목적지·계획 경로·측정 궤적 구분, 기존 grid/costmap/영상 패널, 다크/라이트·확대·접기·반응형 표시를 제공합니다. 임무 선택은 지도 초점만 바꾸고 주행 명령을 보내지 않습니다.

OFFLINE은 Robot Agent 상태이고 영상 STALE과 독립적입니다. 서로 다른 SLAM 좌표의 로봇을 같은 지도에 무조건 투영하지 않습니다. 다중 마커는 공통 세계가 확정된 SIMULATION 예제에서 검증했고, 선택 로봇에 없는 계획 경로는 숨깁니다. 현재 RGB/Depth는 각각 하나의 독립 스트림이며 선택 로봇별 카메라 자동 매핑은 미구현입니다.

## 데이터 출처

- **SIMULATION:** 합성 OccupancyGrid·Costmap·TF 위치/궤적·임무·Adapter 상태와 저장 Gazebo RGB 재생입니다. ROS bridge envelope 스키마를 사용하며 이번 lab은 ROS middleware를 실행하지 않았습니다.
- **REPLAY:** 이전 실차 map/Costmap/metric Depth packet 및 저장 Gazebo RGB입니다. 동시 촬영된 RGB-D 쌍이 아닙니다. 기록 원본 timestamp를 별도로 보존합니다. 임무/로봇 목록은 SIMULATION 예제임을 표시하고 예제 좌표를 실차 기록 지도에 투영하지 않습니다.
- **LIVE:** 기본값은 실제 연결 없음/UNAVAILABLE입니다. 실제 LIMO에 자동 접속하지 않습니다. 선택적으로 명시한 loopback Mission Manager의 **GET API**와 읽기 전용 Adapter snapshot 파일만 표시할 수 있습니다. 실차 적용 검증은 하지 않았습니다.

YOLO는 저장 JPEG6장에 기존 모델로 **CPU 사전계산한 결과**를 재생합니다. 원본 SHA와 프레임 키로 대응하며 이번 전송 성능 표본에 매 프레임 실시간 추론 비용은 포함하지 않습니다. 기존 환경에서 CUDA가 감지되지 않아 이전 실차 GPU8~10ms 기록을 이번 결과로 재사용하지 않았습니다.

## 영상 구조

RGB30Hz 재생 capture → 최신 한 슬롯 → 공유 JPEG encoder(기본480px/q65/12FPS) → 인증 WebSocket의 원자적 metadata+JPEG envelope → 디코딩 ACK → 브라우저 최신 bitmap 한 슬롯 → rAF 표시입니다. H.264는 libx264/zerolatency/baseline, 같은 encoded packet을 TCP/WebCodecs 또는 WebRTC/RTP에 재사용합니다. 접속 수만큼 encoder를 새로 만들지 않습니다. RTP 패킷화·암호화·네트워크 비용은 클라이언트 수에 따라 증가합니다.

Depth는320px/q60/8FPS의 **관제 JPEG 컬러 영상**과 같은 depth_key의 **무손실 거리 배열**을 분리합니다. 거리 배열은 최근32개만 보존하고 만료 시410을 반환합니다. hover 시 화면을 정지해 같은 프레임의 값을 조회합니다. JPEG 색을 역변환해 거리를 추정하지 않습니다. JPEG 미리보기 범위는 현재0.2~4.0m 고정이며 범위 입력은 비활성화돼 있습니다. 임의 범위 재인코딩 UI는 남은 작업입니다.

SIMULATION은 알려진16UC1/mm→m 변환을 검사했습니다. REPLAY는 저장된 float32 metric sample을 비트 단위로 보존하며, 해당 기록의 실차 배율 정확성은 계속 미검증입니다. 원래640×400 네이티브16bit 배열을 기록에 없는 픽셀까지 복원하지 않습니다. RGB-D registration은 false입니다.

## 계측·장애 시험

화면의 `비교 시험 시작`은 **lab 권한**에서 JPEG/TCP·H.264/TCP·WebRTC와 링크 장애 프리셋을 순서대로 실행합니다. viewer는 제어할 수 없고 LIVE에서는 fault injection을 거부합니다. 기본 모드도 임무 제출/취소/모터 API가 없습니다.

전송 서비스시간·ACK EMA로 품질을 낮추거나 복구하며 프리셋의 알려진 링크 조건도 초기값에 사용합니다. 이 적응 정책은 실차에서 튜닝한 네트워크 추정기가 아닙니다. TCP 모델은 지연/지터/재전송 HOL 대기/대역폭 제한을 프로그램 수준에서 주입합니다. WebRTC는 실제 RTP datagram 송신 직전 손실을 주입합니다. OS netem, Wi-Fi, Tailscale을 변경하지 않습니다.

```bash
"$LAB_PY" -m unittest discover -s fleet_control/dashboard_lab/tests
node fleet_control/dashboard_lab/tests/frame_policy.test.mjs
"$LAB_PY" -m dashboard_lab.bench_tcp --seconds 7 --out /tmp/native-tcp.json
"$LAB_PY" -m dashboard_lab.bench_webrtc --seconds 8 --out /tmp/native-webrtc.json
```

capture/resize+encode/queue·link/decode/draw/presentation을 분리합니다. native decode FPS와 브라우저 rAF/RVFC 표시 FPS는 같지 않습니다. Timestamp는 replay frame 생성 시각이며 실제 카메라 노출부터의 지연 측정이 아닙니다. 브라우저 clock offset과 최소 RTT/2 불확실성을 기록하고 물리 모니터의 광학적 지연을 측정했다고 주장하지 않습니다. 단일 표시 탭과 다중/비활성 탭 결과를 분리해야 합니다.

[측정 결과](docs/RESULTS.md) · [API·소스 계약](docs/INTERFACES.md) · [실차 재연결 시 절차](docs/REAL_LIMO_NEXT.md)

참고한 공식 API: [aiortc](https://aiortc.readthedocs.io/en/stable/api.html), [PyAV codec](https://pyav.org/docs/stable/api/codec.html), [aiortc H.264/WebRTC 예제](https://github.com/aiortc/aiortc/blob/main/examples/webcam/webcam.py). RMF 코드를 복사하지 않고 기존 프로젝트 UI·좌표 렌더러를 별도 lab에 재사용했습니다.
