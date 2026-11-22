# 웹 SLAM 세션: 격리 ROS E2E (2026-11-17)

## 범위 및 중요한 제한

**SIM_ONLY 구현이다. 실차 모터 잠금 환경에서도 아직 사용할 수 없다.** Agent의 `slam_sim_enabled`는 기본 false이며 true로 설정하려면 `mode=sim`, `simulated=true`, `allow_missions=false`, `ROS_LOCALHOST_ONLY=1`, 별도 `slam_root`가 필요하다. 이 값은 물리 모터 차단기나 DDS 보안 증명이 아니다. 테스트는 격리 ROS domain, 합성 scan/TF, Nav2·Explore·로봇 드라이버 없이 실행했다. 실차의 전원·USB·E-Stop·watchdog 및 Cliff/Fleet Terminal/Gateway FAIL은 그대로다.

Maps & Mapping의 시작·지도 저장·중지 버튼은 기존 인증된 Fleet Server 명령 큐 및 Agent command journal을 사용한다. 동작은 `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false`만 기동하며 Nav2, Explore, 모터 드라이버 및 Gate 설정은 시작하거나 변경하지 않는다. 시작 전 기존 SLAM/Map Server/AMCL/Nav2 노드, 주요 Nav2 action 서버 및 `/map` 발행자를 확인한다. 이 ROS graph 확인은 관측 시점의 검사이며 동시 기동 경쟁이나 악의적 DDS 참가자, 보이지 않는 다른 domain의 하드웨어가 없다는 증명이 아니다. 격리 환경 외 사용 금지.

`RUNNING`은 소유 launch 프로세스 생존, 단일 `slam_toolbox` 지도 발행자, 시작 후 map 메시지 및 유효한 지도 크기·알려진 셀, 신선한 scan·map/odom/base TF의 Agent 관측을 뜻한다. DDS 발행자 인증, 지도 정확도, 이동 허가 또는 실제 소속 건물·층의 증명이 아니다. 상태 손실은 DEGRADED, 프로세스 소유권 미확정은 UNKNOWN이다. 중지는 해당 시험 세션의 launch process group에 SIGINT한 뒤 프로세스 종료와 ROS graph에서 노드/지도 발행자 소멸을 기다린다. 불명확하거나 timeout이면 UNKNOWN이다. Agent가 죽으면 그 자식 SLAM/녹화 프로세스가 남을 수 있으며 재시작 시 소유권을 추정하지 않고 UNKNOWN으로 새 시작을 차단한다. 자동 정리/UNKNOWN 복구는 미구현이다.

지도 저장은 선택한 **등록된 층**에 지도 파일을 생성하고, 가능할 때 SLAM Toolbox serialize 서비스를 통해 `.posegraph`·`.data`를 생성한다. Agent 전용 토큰, 현재 Agent boot, 해당 로봇에 전달된 `slam_save` command ID, 동일 session/floor를 확인한 뒤 서버에 전송한다. Registry는 `SLAM_LIVE_SNAPSHOT`·`UNVERIFIED`, label=Session ID, floor_id와 불변 map_version으로 등록한다. 건물·시설은 floor의 기존 관계로 연결되며 로봇 위치 확정이나 활성화가 아니다. 저장/등록 실패 시 성공으로 표시하지 않는다. 기존 파일은 덮어쓰지 않는다. 현재 실패한 저장 command의 동일 세션 재시도 및 UNKNOWN 복구 UI는 제공되지 않는다. Agent 지도 전송은 제한된 크기만 가능하며 raster가 약 3.8 MB 이상이면 거부한다. posegraph 포함 전체가 이 한도를 넘으면 raster만 등록하고 posegraph는 로컬 보관한다. 업로드에는 총 6 MiB HTTP 제한 및 Registry 업로드 동시성·시간 제한이 적용된다.

녹화가 먼저 시작되면 SLAM은 같은 Session ID를 사용하고, SLAM 실행 중 녹화를 시작하면 기본적으로 해당 ID를 사용한다. 명시적으로 다른 ID이거나 다른 ID의 시작 명령이 대기 중이면 서버가 거부한다. 두 상태와 종료는 독립이다. SLAM 중지로 bag을 중지하지 않는다.

## 시험 결과

최종 격리 자료: `/tmp/limo-slam-web-e2e-20261117-o/` (임시; 토큰이 포함되어 폴더 전체를 외부에 공유하지 말 것).

- **PASS / 격리 ROS + Chromium**: [브라우저 HTTP 및 동작 로그](/tmp/limo-slam-web-e2e-20261117-o/browser.log), [Agent 로그](/tmp/limo-slam-web-e2e-20261117-o/agent.log), [SLAM 실행 화면](/tmp/limo-slam-web-e2e-20261117-o/slam-running.png), [저장 화면](/tmp/limo-slam-web-e2e-20261117-o/slam-saved.png). 버튼 → 실제 Server → 실제 Agent → 실제 SLAM Toolbox, 합성 LiDAR/TF 및 지도 메시지, 지도 저장, 등록, 중지 확인.
- **PASS / 격리 ROS**: Session `rec_44bdf7f087b54185b8517827fb18fc20`의 `map.yaml`, `map.pgm`, `map.posegraph`, `map.data` 생성. 등록 버전 `rec_44bdf7f087b54185b8517827fb18fc20#637121977fa7`, `test-floor`, UNVERIFIED. posegraph 파일과 Registry checksum 존재. 같은 ID의 bag은 별도로 중지·구조 검증됐다.
- **PASS / 격리 ROS**: [재로드 로그](/tmp/limo-slam-web-e2e-20261117-o/reload.log)에 실제 `map_server`가 저장 지도 87×91, 0.05 m/cell을 읽고 lifecycle configure/activate한 기록이 있다.
- **PASS / 격리 API**: 중복 start는 기존 명령 반환, 다른 세션 동시 start는 409, viewer는 403. 서버 loopback 리스너를 같은 Fleet 인스턴스로 중지·재기동한 뒤 Agent 상태 재연결 확인. 이는 새 server boot ID를 가진 별도 프로세스 전체 재시작 시험은 아니다.
- **PASS / 격리 ROS**: 새 SLAM 진행 중 Agent 종료/재시작 후 UNKNOWN, 새 start 409, 이전 지도의 version/floor/posegraph 상태를 새 세션에 표시하지 않음. 테스트 소유 launch process group만 PID·생성 시각·group을 확인 후 SIGINT하여 종료.
- **PASS / 모의**: `test_slam_control.py` 4개 (기존 SLAM/Nav2/map graph 차단, 저장 실패, 재시작 UNKNOWN, 이전 지도 정보 미상속), `test_slam_sessions.py` 3개 (상충/대기 session), `test_slam_upload_auth.py` 1개 (viewer·다른 로봇 토큰·잘못된 boot 거부). 저장 실패는 실제 디스크 고장이 아닌 Mock이다.
- **PASS / 기존 회귀**: 독립 [녹화 E2E](/tmp/limo-slam-record-regression-20261117-final/browser.log), 녹화 재시작 UNKNOWN 및 중복 거부; recording session 5, Agent safety 17, Safety Context regression 24, Registry HTTP 4, Safety HTTP 1, Dashboard/Registry JS 2 suites.
- **SKIP**: Gazebo, 실차 모터 잠금/실차 구동, 실제 센서, 네트워크 파일 전송, 실제 디스크 부족 저장 실패, 새 server boot 전체 재시작, 장시간 안정성 및 시뮬레이션 외 승인.

이전 녹화 종료 메시지 조사: 변경 전 `...baseline-20261117-b/agent.log`에서 두 번 재현됐고 같은 이전 코드의 `...baseline-20261117-c`에서는 재현되지 않았다. `agent_node.py`에서 executor 종료 뒤 spin thread join을 추가한 `...join-20261117-d` 및 최종 녹화 회귀에는 나타나지 않았고 두 종료 코드 모두 0이었다. 스레드 종료 경합은 가능한 설명이지만 원인은 **미확정**이다. 기록기는 Agent 종료 후 남는 것을 확인했으며 UNKNOWN을 정상으로 강제하지 않았다.

실패/중간 기록도 보존: 첫 sandbox loopback은 socket PermissionError. SLAM `b,c`는 graph 소멸을 8초 내 확인하지 못해 UNKNOWN; `f,g`는 자기 Agent action client를 Nav2 서버로 잘못 취급해 시작 거부; `i`는 viewer 403 후 읽지 않은 POST 본문 때문에 다음 요청 501; `j,k` map_server 시험 정리 문제를 시험 소유 프로세스만 확인해 정리했다. graph 대기·서버 식별·거부 시 연결 닫기·시험 정리를 보완했다. 이 실패를 실차 결과로 간주하지 않는다.

## 재현

```bash
cd /home/cjinw/school/2026-2/Capstone1_limo
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=212
export NODE_PATH=/home/cjinw/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules
# 반드시 빈 격리 ROS graph와 아직 존재하지 않는 출력 경로를 사용한다.
python3 fleet_control/tests/slam_web_ros_e2e.py --output /tmp/limo-slam-web-new
python3 -m unittest discover -s fleet_control/tests -p test_slam_control.py -v
python3 -m unittest discover -s fleet_control/tests -p test_slam_sessions.py -v
```

격리 ROS domain 번호와 `ROS_LOCALHOST_ONLY`만으로 네트워크/물리 격리가 보장되지는 않는다. `slam_synthetic_publisher.py`는 합성 `/scan`·TF를 발행하므로 실제 로봇 graph에서 실행하지 않는다. loopback socket 접근이 제한된 sandbox에서는 테스트가 실행되지 않을 수 있다.

## 변경 파일과 향후 작업

Agent `agent_node.py`, `core.py`, `slam_control.py`; Server `service.py`, `http_api.py`; 대시보드 `static/index.html`, `static/app.js`; 격리/단위 시험 `slam_web_ros_e2e.py`, `.cjs`, `slam_synthetic_publisher.py`, `test_slam_control.py`, `test_slam_sessions.py`, `test_slam_upload_auth.py`, 및 녹화 재시작 시험 보강 `recording_web_ros_e2e.py`. 기존 `run_recorder.py`, Gate, Cliff, Ground Memory, RMF, Nav2 설정은 수정하지 않았다. 공통 문서 대신 충돌을 피한 독립 보고다.

실차를 위한 후속 작업은 독립적인 모터 잠금 증명과 현장 차단·정지 절차, 신뢰할 수 있는 실행/프로세스 소유권 및 ROS graph 충돌 통제, 실차 센서/TF/SLAM 검증, Agent 사망 후 안전한 복구, 저장 실패 재시도 및 대용량 업로드 정책, 별도 모터 잠금 실차 E2E다. 이번 결과로 내일 웹 SLAM을 실제 LIMO에서 사용하도록 안내하지 않는다.
