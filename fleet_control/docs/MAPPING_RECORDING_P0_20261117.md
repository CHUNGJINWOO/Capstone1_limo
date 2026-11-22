# 맵핑·기록 P0 부분 구현 및 격리 시험 (2026-11-17)

**전체 P0 미완료.** 웹/서버/Robot Agent 명령 연결, 웹 상태, 맵핑 세션 PREPARE/START/STOP/SAVE, rosbag 전송 및 Map Registry 세션 연결은 미구현이다. 본 작업의 Agent 패키지 신규 모듈은 기존 AgentCore나 서버에서 호출하지 않는 독립 구성요소다. 운영에 배포하거나 실차 준비 완료로 간주하지 않는다.

## 구현과 범위

- `fleet_control/agent/limo_robot_agent/limo_robot_agent/recording_session.py`: 기존 `run_recorder.py`를 고정 인자로 실행하는 실험적 기록 수명주기 구성요소. 세션별 디스크 상태·잠금, STARTING/RECORDING/STOP_REQUESTED/VERIFYING/VERIFIED/FAILED/UNKNOWN, 동일 인스턴스 중복 start/stop, 소유한 자식의 SIGINT, 기존 `verify_bag_copy.py`의 종료·SQLite·zstd·`ros2 bag info` 검사 재사용. 발행된 ROS 토픽을 구독하여 기록만 하며 어떤 주행 명령도 발행하지 않는다. 기록 프로필 선택 없음.
- `RECORDING`은 bag 디렉터리 관측만 뜻한다. 센서 수신·기록 유효성을 보장하지 않는다. `VERIFIED`도 종료 코드와 bag의 구조 검증만 뜻하며 데이터 완전성이나 메시지 수를 보장하지 않는다. 별도 ROS 시험에서 합성 `/scan` 1건을 확인했다.
- supervisor 재시작 후 이전 자식의 PID/소유권을 재구성하지 않고 활성/불명 세션은 UNKNOWN 처리한다. 타 세션 간 로봇 단위 상호배제, 프로세스 고아 정리, 크래시/디스크 내구성 검증, 이벤트 API, RBAC, Agent 상태 연결과 운영 환경 배포는 미완료다. 다른 세션 ID를 사용한 중복 녹화까지 차단한다고 주장하지 않는다.
- `tests/test_recording_session_p0.py`는 모의 기록 프로세스 4개 시험. `tests/recording_session_ros_p0.py`는 실제 rosbag과 합성 ROS 메시지 시험. `tests/test_bag_copy_faults_p0.py`는 기존 복사기의 ENOSPC·해시 불일치 주입 시험이다.
- 서버·Agent 기존 core 및 Goal journal·Gate·Collision Monitor·RMF·Map Registry와 실차 배포본은 수정하지 않았다. 자동 탐색, 광역 Goal 취소, 실차·Gazebo 구동은 수행하지 않았다.

## 확인된 E2E와 근거

| 경로 / 환경 | 결과 | 근거 |
| --- | --- | --- |
| 웹 → Fleet Server → Robot Agent → SLAM/녹화 | **SKIP** | 명령 전달·API·UI 미연결 |
| 실제 `run_recorder.py` → ROS2 bag → SIGINT → bag 검증 | **PASS**, 격리 ROS, 합성 스캔 | `/tmp/limo-p0-repeat-20261117/result.json`, `bag-info.log`, `synthetic_scan/recording-state.json`, `recorder.log`; `sensor_msgs/msg/LaserScan` `/scan` 1개, zstd SQLite |
| start 중복·stop 중복·동시 잠금·재시작 UNKNOWN·예상 밖 종료 | **PASS**, 모의 프로세스 | `test_recording_session_p0.py` 4 tests |
| 정상 종료 bag → 노트북 역할 디렉터리 로컬 복사·SHA-256 | **PASS**, 로컬 복사만 | `/tmp/limo-p0-rec-qnn04duq/laptop-role/synthetic_p0/verified-manifest.json`. 원본과 복사본 `bag_0.db3.zstd`: `da8f1aea694b97316c02364bc6c19886e5d075047d561f482f32fbca8ae2717a` |
| ENOSPC·복사 후 SHA 불일치 | **PASS**, 장애 주입 Mock | `test_bag_copy_faults_p0.py` 2 tests, 성공 receipt 없음. 실제 디스크를 채우지 않음 |
| 기존 prefix 재개·충돌·소스 손상 | **PASS**, Mock | `test_verify_bag_copy.py` 8 tests. 네트워크 중단은 아님 |
| 실제 SLAM Toolbox + 합성 스캔/TF → map_saver → Registry | **PASS**, 수동 격리 ROS 절차 | `/tmp/limo-p0-slam-hncpgw7k/{publish.py,slam.log,map.yaml,map.pgm,registry.db}`. `map_saver_cli` exit 0, 87×91 @ 0.05 m; 직접 API 호출로 `synthetic_p0#637121977fa7`, IMPORTED_STATIC·UNVERIFIED·비활성 등록. 웹·Agent 제어 아님; bag과 자동 연결 아님. |
| 실제 로봇→노트북 네트워크 전송·재시도 | **SKIP** | 구현·실차 연결 없음 |
| posegraph 저장·map reload·실차·Gazebo | **SKIP** | 실시하지 않음 |

격리 ROS에서 Fast DDS 인터페이스 `Operation not permitted` 경고가 출력되었으나 공유 메모리에서 합성 메시지 수신 및 맵 저장이 확인됐다. 이 결과는 실제 네트워크 품질을 입증하지 않는다. `/tmp` 증거는 임시 보관이며 삭제·재부팅 후 사라질 수 있다.

## 재현

```bash
cd /home/cjinw/school/2026-2/Capstone1_limo
python3 -m unittest discover -s fleet_control/tests -p test_recording_session_p0.py -v
python3 -m unittest discover -s fleet_control/tests -p test_bag_copy_faults_p0.py -v
python3 -m unittest discover -s fleet_control/tests -p test_verify_bag_copy.py -v
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=216
# 충돌 없는 domain과 아직 존재하지 않는 시험 디렉터리를 선택한다.
python3 fleet_control/tests/recording_session_ros_p0.py --output /tmp/limo-p0-new-test
```

Map Registry 등록은 격리 SQLite/파일 스토어에 `register_map_bundle_from_dir(..., source_mode='IMPORTED_STATIC')`를 직접 호출했다. 기존 제품의 인증된 HTTP 등록 및 세션 ID 스키마가 이 시험에서 확장된 것은 아니다.

## 남은 실제 연결 작업

1. 기존 Server 명령 DB와 AgentCore에 안전한 비주행 세션 타입을 추가하고 인증·RBAC·idempotency·감사·boot 세대·불명 결과를 검증한다. 기존 Agent의 다른 명령과 소유권 처리를 보존해야 한다.
2. Robot Agent에서 사전에 고정된 SLAM 전용 launch만 실행하도록 구성하고, 기존 ROS graph/TF/SLAM 및 실제 로봇 상태 충돌을 fail-closed 점검한다. START 수락·프로세스 생존·SLAM 상태·맵 수신과 STOP 요청·실제 종료를 구분한다. 모터 경로 및 Explore를 시작하거나 변경하지 않는다.
3. 지도·posegraph 저장과 실제 생성·실패 증거를 묶고 Registry의 미검증 등록에 session ID를 확실하게 연결한다. 건물/층 메타데이터가 자동으로 실제 위치를 증명하지 않는다.
4. 녹화 supervisor를 Agent lifecycle에 연결하고 세션/로봇 단위 중복·크래시·소유권·디스크 부족·토픽 확인과 상태 노출을 완성한다.
5. 인증된 LIMO→노트북 전송을 별도 구성하고 보안·연결 재시도·해시·충돌·quota·영수증·개인정보 통제를 실제 두 환경에서 검증한다. 원본 삭제 없음.
6. 웹 버튼과 실제 REST 경로를 연결하고 격리 실제 Server+Agent+ROS 브라우저 E2E, 권한/재시작/단절 및 기존 안전 회귀를 완료한다.

Cliff, Fleet Terminal, Gateway 접근 제어 FAIL 및 전원·USB·물리 정지 조건은 그대로다. 위 시험은 실차 이동 승인이나 내일 현장 웹 기능 제공의 근거가 아니다. 공통 문서는 다른 작업과 충돌 가능하여 변경하지 않고 본 독립 기록을 추가했다.
