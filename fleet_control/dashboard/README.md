# LIMO 시각화 콘솔

기존 Mission Manager·Robot Agent를 교체하지 않는 독립 읽기 전용 웹 UI입니다. 지도 중심 배치, 로봇 상태, 접을 수 있는 작업/이벤트, RGB·Depth 패널을 제공합니다. 기존 관제의 임무 제출·취소 UI는 그대로 보존하며, 새 화면에는 제어 API를 만들지 않았습니다. 임무 취소와 물리 비상정지는 서로 다릅니다.

## 현재 검증 범위

**추가 실차 검증:** 승인된 각15초 시험에서 실제 지도·Global/Local Costmap·TF·Depth와 실제 RGB→기존 YOLO 표시를 확인했습니다. 두 시험은 종료했으며 지금은 STALE가 정상입니다. 실제 경로/Frontier와 Depth 거리 정확도는 미검증입니다. [실차 결과·측정값·재실행 명령](docs/REAL_LIMO.md)을 우선 참고하세요. 아래 초기 조사 내용은 추가 시험 이전 시점과 구분합니다.

- **실제 기존 API:** Mission Manager 연결, 기존 실차 저장 지도·마지막 로봇 상태 표시. 검사 시 실차 Agent는 이후 OFFLINE, RGB 송신은 연결되지 않았습니다. 저장 지도를 실시간 SLAM 검증 결과로 간주하지 않습니다.
- **격리 시험:** ROS 2 합성 OccupancyGrid/갱신/TF/경로/Frontier/Depth → 읽기 전용 브리지 → 웹 표시. 저장된 Gazebo JPEG → 기존 실제 CUDA YOLO 프로세스 → 동일 프레임 영상·검출 표시.
- **남은 실차 미검증:** 실제 경로/Frontier 메시지·TF, DaBai 단위/거리 정확도·RGB-D 정렬, 동시 지속 부하. 데이터가 없으면 UNAVAILABLE로 표시합니다. 배포·활성 지도/Costmap/Depth/RGB 연결은 추가15초 시험에서 확인했습니다.
- **미구현:** 지도 프레임 동적 객체 추적, 원격 정지, RMF 실시간 작업 상태 연결. YOLO 2D 상자만으로 지도 객체 위치를 만들지 않습니다.

## 독립 실행

새 worktree 루트에서 실행합니다. Ubuntu의 기존 Python 3, NumPy, Pillow, ROS 2 Humble을 재사용하며 npm 빌드나 새 시스템 패키지는 필요하지 않습니다. 성능 측정 도구만 기존 psutil을 사용합니다. ROS 브리지에는 `rclpy`, `tf2_ros`, `nav_msgs`, `map_msgs`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`가 필요하고, 실제 Frontier 상태는 기존 `explore_lite_msgs` 환경을 로드해야 합니다.

```bash
cd /path/to/Capstone1_limo
export PYTHONPATH="$PWD/fleet_control${PYTHONPATH:+:$PYTHONPATH}"
python3 -m dashboard.run_local start --source-config /path/to/private/upstreams.json
python3 -m dashboard.run_local status
# 이 런처가 등록한 PID와 시작 시각이 일치하는 프로세스만 종료
python3 -m dashboard.run_local stop
```

`upstreams.json`은 Git 밖의 0600 파일입니다. 다음 키를 환경에 맞게 지정합니다. 토큰 값 대신 기존 **viewer 토큰 파일 경로**만 사용합니다.

```json
{
  "mission_url": "http://127.0.0.1:18190",
  "mission_token_file": "/private/mission-viewer.token",
  "ai_url": "http://127.0.0.1:VIDEO_HTTP_PORT",
  "ai_token_file": "/private/video-viewer.token"
}
```

- [실차 API 읽기 화면](http://127.0.0.1:18210/)
- [합성 ROS·저장 Gazebo RGB 시험 화면](http://127.0.0.1:18211/)
- 로그인: 파일 선택 버튼에서 `~/.local/state/capstone-dashboard-visualization/viewer.token` 선택. 토큰 내용을 출력하거나 공유하지 마세요.
- 런타임 설정·로그·PID 등록부: `~/.local/state/capstone-dashboard-visualization/` (디렉터리0700, 설정·토큰0600). `DASHBOARD_STATE`로 변경할 수 있습니다.
- 런처는 추가 시험용 YOLO TCP18220/HTTP18221을 사용합니다. 기존 사전 검증 프로젝트와 가중치·JPEG가 있을 때만 실행합니다. `--video-project`, `--yolo-project`로 경로를 지정할 수 있습니다. 이는 실차 카메라가 아니라 저장 영상 시험입니다.
- 기본 서버는 loopback 전용이고 8080/18190 바인딩을 거부합니다. 외부 공개/방화벽/Tailscale 변경은 하지 않습니다.

현재 실행 상태를 다시 만들 때 필요한 개인 설정은 런타임 폴더 `live.json`에 보관되어 있습니다. `run_local start --source-config ~/.local/state/capstone-dashboard-visualization/live.json`으로 재사용할 수 있습니다. 이미 실행 중이면 먼저 `status`를 확인합니다. 기존 서버는 이 런처로 종료하지 않습니다.

## 화면 사용

지도 드래그/휠 또는 포커스 후 방향키·+/−로 이동·확대합니다. 지도 맞춤, 보기 회전, 레이어 토글, Costmap 투명도를 제공합니다. 보기 회전은 화면에만 적용됩니다. 오른쪽 경계 드래그/방향키로 영상 패널 너비를 조절하고 각 패널 확대 버튼/Esc로 크게 보기/복원합니다. 작업·이벤트 패널은 접을 수 있습니다. 다크·라이트 테마와 작은 화면 세로 배치를 지원합니다.

Depth는 카메라 Z 거리(m)이며 최소/최대 색상 범위와 픽셀 거리 조회를 제공합니다. 무효 값은 회색입니다. RGB 픽셀과 Depth 픽셀을 임의로 대응하지 않습니다. 오래된 영상·지도는 STALE/회색 덮개로 표시하며 게이트웨이 연결이 끊기면 마지막 화면임을 알립니다. 지도 저장 여부는 실제 API가 제공하지 않으면 미제공으로 남깁니다.

## 실제 브리지 합류 절차 — 초기 준비안, 추가 시험은 REAL_LIMO.md 참고

1. Claude 작업 완료 후 실제 publisher의 토픽·타입·QoS·frame·인코딩·스케일을 읽기 전용으로 다시 조사합니다. 현재 소스의 기본값은 장치 확인 결과가 아닙니다.
2. ROS 토픽이 보이는 승인된 별도 환경에서 아래 브리지만 실행합니다. 기존 Agent/SLAM/Nav2/Gate/카메라 노드를 재시작하지 않습니다. 원격 실행·터널 구성은 이번 작업에서 하지 않았습니다.
3. 기존 현장 절차로 **모터 잠금을 확인한 상태**에서 먼저 `--stdout` 또는 새 live 수신 경로로 검증합니다. `/cmd_vel` publisher 부재는 물리 모터 잠금의 증명이 아닙니다. `--lock-check required`는 추가적인 소프트웨어 확인일 뿐 Safety Gate를 대체하지 않습니다.
4. 센서·TF·timestamp·배율 및 기존 노드 CPU 부하를 확인한 뒤 별도로 합류 여부를 결정합니다. 자동 주행을 시작하거나 임무를 제출하지 않습니다.

```bash
source /opt/ros/humble/setup.bash
source /path/to/existing/field/install/setup.bash
export PYTHONPATH=/path/to/Capstone1_limo/fleet_control
# 승인된 ROS domain/localhost 설정을 그대로 사용
python3 -m dashboard.bridge_node --stdout --lock-check required \
  --topics /private/actual-topics.json
# stdout 대신 로컬 live gateway로 전송하는 경우
python3 -m dashboard.bridge_node --endpoint http://127.0.0.1:18210/api/bridge \
  --token-file ~/.local/state/capstone-dashboard-visualization/bridge.token \
  --topics /private/actual-topics.json --fps 1 --lock-check required
```

실제 장치에서 위 예시를 그대로 실행했다고 보고하지 않습니다. 원격 브리지→노트북 loopback 경로는 별도 승인된 전송 구조가 필요합니다. 토큰을 URL/명령행 값으로 넣지 않습니다. 실제 QoS가 다르면 어댑터 수정·재검증이 필요합니다.

## 시험

```bash
python3 fleet_control/dashboard/tests/test_visualization.py
node fleet_control/dashboard/tests/geometry.test.mjs
# 격리 서버가 실행된 상태에서 현재 성능·연결 확인
python3 fleet_control/dashboard/tests/probe_current.py --seconds 10
```

[인터페이스 계약](docs/INTERFACES.md) · [검증 결과와 제한](docs/VALIDATION.md)

## 디자인·라이선스

[공식 RMF Web](https://github.com/open-rmf/rmf-web)과 [Dashboard Framework](https://open-rmf.github.io/rmf-web/packages/rmf-dashboard-framework/)의 지도·로봇·작업·세부 패널 계층을 참고했습니다. RMF 소스는 복사하지 않았고 React/RMF 의존성을 추가하지 않았습니다. RMF Web의 [Apache-2.0 라이선스](https://github.com/open-rmf/rmf-web/blob/main/LICENSE.md)를 확인했습니다. 기존 외부 YOLO 프로세스는 그대로 재사용하며 그 프로젝트의 Ultralytics 라이선스 의무는 별도로 유지됩니다.
