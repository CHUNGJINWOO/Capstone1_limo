# REAL_LOCKED 비구동 후보 — 준비 및 차단 기록 (2026-11-17)

## 결론

**차단(BLOCKED): 실차 연결·운영 가능 상태가 아니다.** 이번에는 실차에 접속하지 않았고 모터·Gazebo를 구동하지 않았다. 현재 트리에서 `mode=real_locked` 또는 `real_locked_requested=true`는 로봇 노드/기록 자식 기동 전에 항상 거부된다. 또한 기존 `mode=real`에서 웹 녹화 활성화(`recording_enabled=true`)도 같은 이유로 거부한다. **현재 Agent는 설정을 읽기 위해 거부 전에 ROS context와 임시 파라미터 노드를 생성한다.** 따라서 이 거부 장치는 DDS 비접속 보증이 아니며 실차에서 실행하면 안 된다. 기본 실차 Agent의 `recording_enabled=false` 읽기 전용 경로와 격리 SIM_ONLY를 의도적으로 유지한다. 이미 실행 중인 기존 Agent나 다른 프로그램을 원격으로 중지하거나 변경하지 않는다. SIM으로 설정을 속이는 것은 물리적 격리가 아니며 실제 LIMO에서 사용 금지다.

[real_locked_policy.py](../agent/limo_robot_agent/limo_robot_agent/real_locked_policy.py)는 현 단계의 **무조건 거부**이다. 사람의 체크박스, 웹 토큰, 단일 JSON 파일, 임의의 승인 플래그, `/cmd_vel` 발행자 0개, `/cmd_vel_locked`, Gate NORMAL, 정지된 odometry, E-Stop 화면값, 통신 단절은 단독으로 물리 모터 에너지 차단 증거가 아니다. 제안된 증거가 마련되더라도 현재 코드는 그것을 검증하거나 수용하지 않는다.

## 물리 잠금 증거 및 현장 절차 — 제안, 미구현

1. 실차 접속·현장 점검은 별도 사용자 승인 후 진행한다. 제조사와 현장 안전 책임자가 승인한 에너지 차단 수단·유지 방법·복구 절차와 독립적인 확인 방법을 먼저 정한다. 센서/컴퓨팅 전원을 유지하면서 모터 에너지만 차단할 수 있는지 확인되지 않았다. 임의 배선 분리나 원격 명령으로 검증하지 않는다.
2. 해당 개체의 구동 에너지 경로가 실제로 차단된 상태임을 적격한 현장 담당자가 승인된 장비와 절차로 독립 확인하고, 로봇 식별·차단 수단·시각·담당자·현장 확인 결과를 기록한다. 실제 측정 방법과 허용 기준은 제조사/안전 책임자가 정한다. 관찰 기록은 책임 추적이며 자동 기동 승인 증거로 변환하지 않는다.
3. 향후 자동 검증이 필요하다면 차단 상태를 독립적으로 측정하고 연속 감시하는 신뢰 가능한 증거 생산자, 로봇/장치 식별, 인증·무결성·재생 방지, boot/세대·만료, 신호 단절·차단 해제 시 즉시 무효화 및 독립 현장 재확인 계약을 설계·시험해야 한다. 현재 이 생산자·키·검증기·재무장 정책은 없다. 구현 전에는 언제나 거부한다.
4. 동력 차단이 확인되지 않거나 센서 전원과 차단 방식이 양립하는지 불명확하면 시험을 시작하지 않는다. 물리 E-Stop의 동작과 watchdog은 별도 검증 사항이다. 과거 전원 돌발 차단 4회와 USB 재연결 원인·완충 소크 미해결, Cliff 지지 영역 FAIL, Fleet Terminal/Gateway 접근 제어 FAIL도 유지한다.

## 기존 실행본·소유권 조사

- 기존 `real_auto_mapping_ready_20261113`는 `allow_motor_drive=false`에서 소프트웨어 출력을 `/cmd_vel_locked`로 돌리지만 base·LiDAR·SLAM·Nav2까지 포함하는 기존 bringup이다. Frontier는 별도 명령 전 기본 중지이나 이는 이 요청의 SLAM·센서 전용 프로필과 같지 않다. 기존 launch 및 물리 E-Stop/watchdog을 바꾸거나 현장에서 이를 새 웹 REAL_LOCKED로 대신 기동하지 않는다.
- 기존 웹 `SlamControl`은 명시적 SIM_ONLY, `allow_missions=false`와 로컬 ROS 설정에서만 동작한다. 기동 전 알려진 SLAM/Nav2 노드·Action 서버·지도 publisher를 확인하지만 ROS graph의 관측/동시 실행 경합, 다른 namespace/domain, 비협조 프로세스의 부재를 보장하지 않는다. 이 graph는 인증된 소유권 증거가 아니다.
- 웹 recording은 선택한 **동일 root**를 공유하는 협조 Agent 간 `flock`, 세션 포인터 및 journal을 이용한다. 다른 root, 별도 `ros2 bag record`, 다른 사용자/namespace, PID 재사용 또는 비협조 프로세스까지 배제하지 않는다. 실제 환경에서 정확한 소유자·실행 세대·자식 관계와 외부 recorder 부재를 검증하는 방법이 아직 없다. 검출 실패/불명은 시작 차단이다.
- Agent 재시작 후 현재 세션이 미완료이면 기존 자식 프로세스를 재인수하지 않고 UNKNOWN을 영속화한다. SIGINT 및 SIGKILL 격리 시험에서 원래 recorder가 남고 새 Agent는 UNKNOWN 및 새 녹화 409를 보고했다. 시험 소유 프로세스만 PID/시작 시각/명령을 대조해 SIGINT했다. 현장에서는 임의 프로세스를 kill/재인수하지 않고 담당자 확인과 별도 승인된 복구 절차가 필요하다. 현재 UNKNOWN 복구 UI는 없다.
- `terminate called without an active exception`은 기존 비결정적 현상이다. Agent executor join을 적용한 뒤에도 추가 격리 종료 6회 중 1회 종료 코드 -11이 발생했다. `PYTHONFAULTHANDLER=1` 8회 중 1회도 -11이고 [로그](/tmp/limo-executor-faulthandler-20261117-a/run1.log)에 executor thread의 `rclpy.signals` guard condition `destroy`/`__del__` 스택이 보인다. 두 실험 모두 sandbox DDS `Operation not permitted` 경고가 있었다. 이후 loopback을 허용한 비교 시험 8회는 종료 코드 0이고 같은 interface 권한 오류가 없었다 ([결과](/tmp/limo-executor-allowed-20261117-a/result.json)). 작은 표본의 상관관계이며 원인·실차 발생률은 미확정이다. 메시지가 사라졌다는 이유로 수정 완료라고 판정하지 않는다. 무권한 sandbox 환경 시험 실패를 보존하고 해당 담당자에게 ROS signal/executor lifecycle 별도 분석을 요청한다.

## 실차 저장·센서 확인 준비 (아직 미실행)

실차 연결을 승인받고 현장 담당자가 물리적 차단을 확인한 후에만, 먼저 읽기 중심으로 실제 설치·설정을 점검한다. 아래 명령은 **후속 현장 점검 예시**이지 실행 승인이나 기동 절차가 아니다. 주소·토큰·개인정보 및 실제 bag/영상은 로그나 공개 저장소에 넣지 않는다.

1. 실제 Agent 패키지와 배포본의 버전·파일 SHA-256, launch/설정 경로 및 `ROS_DOMAIN_ID`·DDS 분리 범위를 현장 소유자와 비교한다. 저장소 사본과 LIMO 배포본의 일치를 이번에 확인하지 않았다. 기존 배포 `env.sh` 기본 domain 77, `ROS_LOCALHOST_ONLY=1`은 과거 설정일 뿐 현행 배포 상태나 네트워크 격리의 증거가 아니다.
2. 실제 Agent가 사용할 로봇 전용 `recording_root` 및 SLAM root가 존재하는 저장 장치, 쓰기 권한, 0700/0600, 배터리/전원과 개인정보 보관 책임을 확인한다. 웹 기록 경로는 `<recording_root>/<session>/<날짜>_<label>/bag`; 지도 후보는 `<slam_root>/<session>/map.yaml`, `.pgm`, `.posegraph`, `.data`다. 이는 코드상 형식이며 **LIMO의 실제 절대 경로·사용 가능 용량은 미확인**이다. 기존 로컬 기록 정책의 우선 경로는 `--root` > `LIMO_RUNS_DIR` > 저장소 `local_data/raw_results/limo_runs` > `~/limo_runs`. 기존 실차 supervisor의 기본은 `~/.local/state/limo-real-auto-mapping/`으로 별도다. 웹 기록은 명시 root를 사용한다. 백업 완료 전 원본 삭제 금지.
3. 담당자가 승인한 마운트에 대해 `df -h -- <저장_디렉터리>`와 `df -i -- <저장_디렉터리>`로 바이트·inode 여유와 권한을 확인한다. 단일 1 GiB는 과거 준비 스크립트의 최소값일 뿐, 장시간 LiDAR/TF·영상의 충분한 용량이라는 근거가 아니다. 목표 수집 시간, 실제 압축 전후 발생률, 분할, 임시 복원 공간, 시스템 예약 공간, 실패 여유를 현장에서 산정한다. 실제 측정 전 시작 금지.
4. 충돌 진단은 현장 승인과 그래프 범위를 확인한 뒤 `ros2 node list`, `ros2 topic list -t`, 필요한 `ros2 topic info --verbose <topic>`와 현장 프로세스 관리자의 소유권 목록을 대조한다. 조회는 권한/namespace/DDS 경계 밖 프로세스 부재를 보장하지 않으며 ROS CLI daemon이 시작될 수 있다. 다른 사람이 실행한 노드·bag을 종료하지 않는다. ROS graph·프로세스 출처/생성시각/세대가 일치하지 않거나 시작 경쟁을 막지 못하면 차단한다.
5. 센서 토픽·타입·프레임·QoS와 신선도를 현장 graph에서 확인한다. 과거 실차 준비본은 scan normalizer를 켜면 SLAM 입력을 `/scan_slam`으로 변경하고, F4 odometry 및 `map→odom→base_footprint` TF를 사용한다. 현재 웹 `run_recorder.py`의 고정 CORE 목록은 `/scan`, `/odom`, `/imu`, `/tf`, `/tf_static`, `/map`, Gate 등은 포함하지만 **`/scan_slam`은 포함하지 않는다.** 웹 recorder는 현재 `--extra /scan_slam` 또는 QoS 프로필 선택을 연결하지 않았다. 이를 해결·시험하기 전 정규화된 SLAM 입력을 bag이 보존한다고 말할 수 없다. 정적 TF의 실제 기록과 `ros2 bag info` 메시지 수·타입·내용, 이미지 미수신과 시간/QoS도 현장에서 확인해야 한다.
6. SIM에서 쓴 기본 Slam Toolbox YAML과 과거 실차 bundle의 `integrated_slam_params.yaml` 및 런치의 `/scan_slam` rewrite는 다르다. 실차 지도/posegraph 저장 경로, 프레임, 센서 토픽, 로컬 디스크, 종료·재로드, Registry `UNVERIFIED`와 수집 Session ID의 연결을 별도로 시험해야 한다. 현재 웹 SlamControl은 기본 SIM YAML을 사용하며 REAL_LOCKED를 지원하지 않는다. posegraph는 용량 제한 시 로컬만 저장될 수 있다. rosbag의 PC 자동 전송은 미구현이다.

## 이번 시험

- **PASS / 격리 실제 ROS+Chromium**: `/tmp/limo-real-locked-final-slam-20261117-a/browser.log` — 최종 실차 녹화 차단 보강 후 웹 SLAM 시작, 합성 scan/TF 지도, 저장·posegraph 및 Registry, map_server 재로드, 동일 세션 bag 종료; 서버 리스너 재연결 및 Agent 재시작 UNKNOWN. 사용된 domain은 분리된 테스트다.
- **PASS / 격리 실제 ROS+Chromium**: `/tmp/limo-real-locked-final-record-20261117-a/browser.log` — 웹 녹화 시작/중지, bag 합성 scan 1건, viewer 거부, Agent SIGKILL 후 recorder 잔존·UNKNOWN·409. SIM mode, real motion disabled. 이 시험 폴더 안에 임시 토큰이 있어 전체 공개 금지.
- **PASS / 단위+실제 Agent 거부**: `test_real_locked_policy.py` 2개. `mode=real_locked` 및 `mode=real, recording_enabled=true`는 토큰 검사/로봇 노드 생성/자식 기동 전 거부했다. sandbox ROS DDS interface/socket PermissionError 로그가 같이 발생했다. 별도 최초 실행은 PYTHONPATH 설정 실수로 rclpy import 실패했고 이후 수정해 재실행했다.
- **FAIL / 격리 프로세스 종료**: 6회 중 1회 -11, 독립 faulthandler 8회 중 1회 -11; DDS 권한 제한이 동반된 범위로서 원인 미확정. 이전 `terminate called...` 문구가 없다는 것이 PASS는 아니다.
- **PASS / 격리 프로세스 비교**: loopback 허용 환경의 동일한 비구동 종료 절차 8회 모두 exit 0·해당 문자열 미발생. sandbox 실패의 원인 해명이나 실차 검증이 아니다.
- **SKIP**: 실차 접속, 실제 물리 잠금·센서·경로·용량·TF·bag, 실차 디스크 및 장애, Gazebo, 실차 주행/Frontier/원격 조종. 자동 파일 전송·웹 복구 미구현.

## 재현 명령 (격리 전용)

```bash
cd /home/cjinw/school/2026-2/Capstone1_limo
python3 -m unittest discover -s fleet_control/tests -p test_real_locked_policy.py -v
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=204
export NODE_PATH=/home/cjinw/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules
# 반드시 격리 graph 및 기존에 없는 결과 경로를 사용한다.
python3 fleet_control/tests/slam_web_ros_e2e.py --output /tmp/limo-real-locked-new-slam
python3 fleet_control/tests/recording_web_ros_e2e.py --output /tmp/limo-real-locked-new-record --restart-signal SIGKILL
```

`ROS_LOCALHOST_ONLY`나 domain은 물리 안전 경계가 아니다. 실행 가능한 REAL_LOCKED launch 명령은 제공하지 않는다.

## 변경 및 문서 소유권

`agent_node.py`의 후보 사전 거부, 새 `real_locked_policy.py`, `test_real_locked_policy.py`, SIGKILL 진단 옵션의 `recording_web_ros_e2e.py`, 본 독립 보고서를 추가/변경했다. 기존 `SIM_ONLY` 실행 명령과 Gate·Cliff·Ground Memory·Fleet 안전 판정·RMF·기존 `run_recorder.py`는 변경하지 않았다. 공통 문서에는 충돌 가능성 때문에 직접 쓰지 않았다. 후보 변경 제안: `CURRENT_STATUS.md`에 REAL_LOCKED BLOCKED 및 `mode=real` 녹화 비활성, `VALIDATION.md`에 SIGKILL 결과와 SIGSEGV FAIL, `HANDOFF.md`에 `/scan_slam`·QoS·독립 물리 증거/소유권 계약 미완료를 추가한다. 기존 FAIL 및 기록은 보존한다.
