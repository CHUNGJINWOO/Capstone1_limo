# 웹 rosbag 녹화 E2E (2026-11-17)

## 범위

History & Diagnostics의 녹화 시작/중지·상태·경로를 Fleet Server 기존 bearer 인증 및 명령 큐, AgentCore의 기존 command ID·journal·boot 확인을 통해 Robot Agent의 `RecordingControl`→`RecordingSession`→`run_recorder.py`에 연결했다. 별도 셸 실행 API나 입력 토픽/명령 지정은 없다. 기록 기능은 Agent에서 **기본 비활성**, 시작 시 명시적으로 세 경로를 지정한 경우에만 사용 가능하다. GET `/api/v1/robots/{id}/recording`이 상태 조회에 해당한다. POST `.../recording/{start|stop}`은 operator/admin만 가능하다. Agent 상태가 오래되거나 미수신이면 UNKNOWN이다.

`STARTING`은 실행 요청이고 `RECORDING`은 소유 recorder 프로세스 생존 및 생성된 bag 디렉터리 관측이다. 실제 토픽 수신의 증거가 아니다. `STOP_REQUESTED`와 `VERIFYING`은 완료가 아니다. `VERIFIED`는 recorder 정상 종료 증거와 `verify_bag_copy.py`의 `ros2 bag info`, SQLite 무결성·zstd 복원 검사 통과를 의미한다. 토픽 존재·수량·기록 품질, 네트워크 복사·백업 완료, 실제 로봇 주행 허가를 뜻하지 않는다. 저장 경로는 **Agent 호스트**의 경로다.

Agent 설정의 `recording_root`는 로봇별 전용 디렉터리여야 하며, 디렉터리 flock과 영속 current-session pointer로 하나의 기록 세션만 소유한다. 동일 command ID는 기존 journal이 재실행하지 않고 같은 세션/action 재요청은 서버가 기존 명령을 반환한다. 다른 세션의 시작은 기존 활성/불명 세션 또는 미완료 명령에서 거부한다. Agent 재시작 시 진행 중인 세션은 UNKNOWN으로 남기고 PID를 재소유하거나 임의 종료하지 않는다. UNKNOWN의 운영 복구 UI는 아직 없다.

## 실제 격리 결과

`/tmp/limo-record-web-e2e-20261117-e/` (임시 보관): [브라우저 및 HTTP 로그](/tmp/limo-record-web-e2e-20261117-e/browser.log), [Agent 로그](/tmp/limo-record-web-e2e-20261117-e/agent.log), [bag 정보](/tmp/limo-record-web-e2e-20261117-e/bag-info.log), [녹화 중 화면](/tmp/limo-record-web-e2e-20261117-e/recording.png), [검증 완료 화면](/tmp/limo-record-web-e2e-20261117-e/verified.png). 이 디렉터리에는 테스트 토큰이 있으므로 외부에 공유하지 않는다.

- Chromium 버튼 → POST start 202 → 실제 Fleet Server → 실제 Robot Agent → `run_recorder.py` → rosbag 시작: **PASS**.
- 격리 ROS 합성 `/scan` (`sensor_msgs/msg/LaserScan`) 1건을 실제 zstd bag에서 `ros2 bag info`로 확인: **PASS**.
- Chromium 중지 → POST stop 202 → Agent SIGINT → 자식 종료 → bag 검사 → 웹 VERIFIED·Agent 저장 경로 표시: **PASS**.
- 같은 세션 start 재요청 반환, 다른 세션 중복 시작 409, viewer stop 403, 페이지 reload 후 VERIFIED 상태 복원: **PASS**.
- 별도 시험 세션 녹화 중 실제 Agent SIGINT 종료·재시작 후 `UNKNOWN`: **PASS**. 시험이 소유한 원래 recorder 프로세스는 PID·시작 시각·명령을 확인한 후 SIGINT했다. 종료 시 Agent 로그의 `terminate called without an active exception` 문구가 관측됐다. restart UNKNOWN은 확인했지만 이 종료 메시지의 원인은 미분석이다.
- `ros2` 실행 파일을 PATH에서 제외한 실제 `run_recorder.py` 오류는 종료 코드 2·FAILED를 확인했다 (`/tmp/limo-rec-error-zbvp4we1/`). 예기치 않은 종료와 stop timeout은 **모의 프로세스** 단위 시험 PASS. timeout에서 강제 kill하지 않고 UNKNOWN. 실차 네트워크 단절·장시간 기록·디스크 부족을 통한 실제 프로세스 장애는 미시험.
- `test_recording_session_p0.py` 5, `test_verify_bag_copy.py` 8, `test_bag_copy_faults_p0.py` 2, `test_agent_safety_v1.py` 17, `test_safety_context_v1_regression.py` 24, `test_safety_context_v1.py` 5, Dashboard/Registry JS 2 suites PASS.
- 첫 시도 `...-a`는 sandbox loopback socket PermissionError, `...-b`는 NODE_PATH 누락으로 실패했다. `...-c`는 본 E2E, `...-d`는 재시작 추가 시험, `...-e`는 HTTP 로그 포함 최종 시험이다. 실패 기록을 보존한다. 테스트의 서버는 127.0.0.1 OS 할당 포트, 임시 DB·토큰만 사용했다.

## 재현

```bash
cd /home/cjinw/school/2026-2/Capstone1_limo
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=214
export NODE_PATH=/home/cjinw/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules
# 충돌하지 않는 ROS domain과 아직 존재하지 않는 출력 경로를 고른다.
python3 fleet_control/tests/recording_web_ros_e2e.py --output /tmp/limo-record-web-new
python3 -m unittest discover -s fleet_control/tests -p test_recording_session_p0.py -v
```

시험은 ROS 합성 `/scan`을 발행하므로 실제 ROS 그래프와 분리해야 한다. loopback socket이 제한된 sandbox에서는 서버가 시작되지 않을 수 있다. 실제 Agent를 별도 배치할 때는 기존 Agent 실행 인자에 `recording_enabled:=true`, 로봇 전용 `recording_root`, 신뢰된 배포 경로의 `recorder_script`와 `verifier_script`를 명시해야 하며, 해당 호스트에 ROS·`zstd`·충분한 저장 공간이 필요하다. 배포 및 LIMO 실행은 이번에 하지 않았다.

## 남은 범위

자동 맵핑, SLAM, 지도 저장, Map Registry 연결, rosbag 네트워크 전송, 프로필 선택, UNKNOWN 복구, 녹화 중 예상 용량/실제 메시지 수의 웹 표시, 장시간·실차 검증은 미구현 또는 미검증이다. 기존 Gate·Cliff·Ground Memory·RMF·Nav2 실행 조건은 변경하지 않았다. 기존 FAIL 기록을 보존하며 공통 문서는 충돌 회피를 위해 본 독립 보고만 추가했다.
