# LIMO 실차 배포 계획 (2026-11-11, 계획만 — 배포하지 않음)

상태: 노트북 쪽 조사는 끝났다(읽기 전용). **LIMO 쪽은 아직 조사하지 못했다**: 접속 정보(호스트/계정)가 저장소와 SSH 설정에 없고, 추측하지 않았다. LIMO에 아무것도 접속·변경·전송하지 않았다. 아래 "미확인" 항목은 LIMO 인벤토리(`limo_readonly_inventory.sh`)를 읽은 뒤 확정한다.

## 1. 실차 구동에 필요한 패키지 (real launch 추적 결과)
`limo_integrated_auto_mapping_real.launch.py`가 직접 참조하는 패키지와 용도:

| 패키지 | 출처 | 용도 | git 상태 |
|---|---|---|---|
| `limo_capstone` | 노트북 src | 이 프로젝트의 노드·launch·설정 | 일부 **미커밋/미추적** (4절) |
| `limo_depth_obstacles` | src | 깊이 장애물 필터 (`use_depth_camera:=true`일 때만) | 추적됨 |
| `limo_base` + `limo_msgs` | src/limo_ros2 (AgileX) | 시리얼 드라이버: `/dev/ttylimo`, `/cmd_vel` 구독, `/odom` `/imu` 발행 | **git 제외됨**(`.git/info/exclude`) |
| `limo_description` | src/limo_ros2 | URDF(`limo_four_diff.xacro`) + robot_state_publisher | git 제외됨 |
| `ydlidar_ros2_driver` | src | `/dev/ydlidar` → `/scan`(frame `laser_link`) | 추적됨, **시스템에 ydlidar_sdk 필요** |
| `astra_camera`, `astra_camera_msgs` | src/ros2_astra_camera | DaBai 깊이 카메라 (`use_depth_camera:=true`일 때만) | **git 제외됨** |
| `explore_lite`, `explore_lite_msgs` | src/m-explore-ros2 (원본 그대로) | frontier 탐색 (`start_explore:=true`일 때만) | 추적됨, 변경 없음 |
| `robot_localization` | src (소스 복사본) | F1b / F5 EKF (`estimator_type`가 f1b·f5일 때만) | 추적됨. 노트북 apt에는 없음 |
| apt: `nav2_bringup`, `slam_toolbox`, `xacro`, `joint_state_publisher`, `robot_state_publisher`, `tf2_ros` | ROS Humble | Nav2, SLAM, URDF | 노트북에는 설치됨 |

필요 없는 것(실차 launch가 참조하지 않음): `wego*`, `limo_ros2_application`, `libuvc`/`magic_enum`(깊이 카메라 빌드에만), `map_merge`, `limo_ros2/catmux`, `limo_car`(저장소 구조 설명에는 있으나 이 src에는 없음).

기본 실행(`use_depth_camera:=false`, `start_explore:=false`)이 실제로 쓰는 것: limo_base, ydlidar, robot_state_publisher, F4(+ F5/F1b/F3b 선택 시 해당 노드), odom_tf_relay, slam_toolbox, Nav2(`integrated_nav2_lidar_only_params.yaml`), stall_detector.

## 2. real launch가 참조하는 파일 (배포 후보)
`manifest`: `deploy_manifest.tsv` (등급, git 상태, sha256 앞 16자, 경로). 기준은 노트북 작업 트리.

**T1 — 필수 (`limo_capstone`, 17개)**
- 패키지 메타: `package.xml`, `setup.py`(수정됨), `setup.cfg`, `resource/limo_capstone`
- Python: `limo_capstone/__init__.py`, `stall_detector_node.py`(수정됨), `lidar_scan_matcher_odom_node.py`(수정됨), `odom_tf_relay_node.py`, `complementary_filter_node.py`(**미추적**)
- launch: `launch/limo_integrated_auto_mapping_real.launch.py`(수정됨) — 이것만. sim launch는 보내지 않는다.
- config: `integrated_slam_params.yaml`, `integrated_nav2_params.yaml`(수정됨), `integrated_nav2_lidar_only_params.yaml`(**미추적, 기본 실행에 필수**), `integrated_explore_params.yaml`(수정됨), `fusion_f5_improved.yaml`, `ekf_tuned.yaml`(**미추적**), `navigate_to_pose_ackermann_no_reverse.xml`(**미추적, 항상 적용됨**)

**T2 — 선택 (실험 보조)**: `rviz/limo_real_auto_mapping.rviz`, `scripts/{auto_mapping_experiment_manager.py, manage_auto_mapping.sh, audit_estimator_dryrun.py, run_unpowered_trial.py, run_recorder.py}` — 모두 미추적. 일부는 `/home/pcu/capstone_experiment_ws`와 `/home/pcu/load_experiment_env.sh`를 하드코딩한다(이 저장소 AGENTS.md의 환경 독립 원칙과 충돌하지만 LIMO 전용 스크립트로 보인다. 보내기 전에 내용을 검토할 것).

**T3 — LIMO에 없을 때만 (인벤토리 후 결정)**: `limo_depth_obstacles`(7), `explore_lite`+msgs(20), `ydlidar_ros2_driver`(20), `robot_localization`(102, bag 제외). 드라이버(`limo_ros2`, `ros2_astra_camera`)는 git 제외이고 팀 원본에 이미 있을 가능성이 높아 **기본적으로 보내지 않는다**. 노트북에서 보내야 한다면 팀 원본과 해시를 비교한 뒤 별도로 결정한다.

**보내지 않는 것**: `worlds/`, `models/`, `urdf/limo_diff_fortress.urdf`, `launch/*sim*.launch.py`, `config/integrated_nav2_diff_params.yaml`, `config/nav2_sim*.yaml`, `ground_truth_publisher_node`/`sensor_normalizer_node`/`depth_to_pointcloud_fixed_node`/`integrated_recorder_node`/`nav2_action_monitor_node`(sim 보조; setup.py의 entry point에는 남아 있어 빌드는 되지만 실차에서 쓰이지 않음), `experiments/`, `docs/`, `build|install|log`, `__pycache__`, `*_Zone.Identifier`, `*.bag`.

## 3. 노트북과 LIMO의 차이 / 6. LIMO 실험 워크스페이스 — **미확인**
`limo_readonly_inventory.sh`가 읽는 것: 두 워크스페이스(`/home/pcu/capstone_ws`, `/home/pcu/capstone_experiment_ws`)의 패키지 목록·git 상태·T1 파일 해시, OS/ROS/numpy/scipy, apt 패키지, `/dev/ttylimo`·`/dev/ydlidar`, udev 규칙, 실행 중인 ROS 프로세스. 쓰기 없음, ROS 노드 실행 없음, 모터 접근 없음. 결과로 다음을 확정한다: LIMO 고유 수정 파일(해시가 노트북과 다르고 노트북 이력에 없는 것), 이미 있는 의존성, 팀 원본과의 차이.

## 4. git 미커밋·미추적·무시 상태 (확인함)
- 배포 필수 17개 중 **10개가 커밋되지 않았다**(수정 6, 미추적 4). HEAD에서 체크아웃하거나 `git archive`로 만들면 `integrated_nav2_lidar_only_params.yaml`, `navigate_to_pose_ackermann_no_reverse.xml`, `complementary_filter_node.py`, `ekf_tuned.yaml`이 빠지고, real launch의 gate 개선(INITIALIZING, odom 감시)도 빠진다. **작업 트리에서 직접 만든 선택 목록으로 배포해야 한다.**
- `limo_ros2`(65개)와 `ros2_astra_camera`(159개)는 `.git/info/exclude`로 제외되어 있다. 이 저장소를 새로 clone하면 없다(보존 위험).
- 이번 배포에서 커밋은 하지 않는다. 배포 후에도 미커밋 상태가 그대로이므로 노트북 쪽 작업 트리가 유일한 원본이다. 실제 배포 시 `deploy_manifest.tsv`와 같은 내용의 해시 목록을 LIMO 쪽에도 기록해 두는 것을 권한다.

## 5. 시뮬레이션 전용 설정이 실차에 섞일 위험
| 위험 | 상태 | 대응 |
|---|---|---|
| 공유 YAML(`slam`, `nav2`, `fusion_f5`)에 `use_sim_time: true` | 실차 launch는 `use_sim_time=false`를 인자로 넘기며 slam_toolbox와 nav2_bringup(Humble)이 YAML 값을 덮어쓴다. EKF는 launch 파라미터 딕셔너리가 뒤에서 덮어쓴다 (코드로 확인, **실행하여 확인하지는 않음**) | 배포 후 `ros2 param get`으로 각 노드의 `use_sim_time` 확인 |
| sim 전용 깊이/레일 설정(`min_height -0.02`, `min_obstacle_height -0.03`, `processing_hz 20`) | sim launch 안에만 있다. 공유 YAML과 real launch는 원래 값(0.02 등, real은 `processing_hz 4`) | sim launch를 보내지 않는다 |
| `integrated_nav2_params.yaml`은 sim과 실차가 공유 | 현재 값이 실차용이다. sim 쪽 변경은 launch의 RewrittenYaml로만 한다 | 앞으로도 공유 YAML에 sim 값을 직접 넣지 않는다 |
| **Ackermann BT(후진/Spin 제거)가 실차에 항상 적용** | real launch가 무조건 `navigate_to_pose_ackermann_no_reverse.xml`을 쓴다. 반면 URDF는 `limo_four_diff.xacro`, 컨트롤러는 RPP. LIMO Pro의 실제 주행 모드(4WD 차동/Ackermann) 확인 필요 | **사용자 확인 필요**. 배포 전 실제 모드와 일치하는지 결정 |
| gate가 실차에서는 `require_operator_unlatch: False` (sim은 True) | 의도된 차이일 수 있으나 STALL 후 자동 복귀 가능 | gate는 이번에 변경하지 않음. 모터 잠금 상태에서는 무관 |
| F4가 벽과 평행 이동에서 이동량을 놓침 (이번 세션 조사) | 실차에도 같은 코드. 기본 `estimator_type=f4`, `run_icp_observer=true`이므로 어떤 추정기든 gate가 F4에 의존 | 모터 잠금 단계에서는 영향 없음. 구동 단계 전에 판단 필요 |
| 스크립트의 하드코딩 경로 | T2 일부 `/home/pcu/...` | 필요한 것만 검토 후 보냄 |
| `*_Zone.Identifier`, `__pycache__` 등 | 노트북 작업 트리에 있음 | rsync 제외 |
| `setup.py`가 `models/limo_eval_diff` 등을 data_files로 glob | 디렉터리가 없으면 빈 목록이라 빌드는 됨 (**LIMO에서 확인 필요**) | 보내는 `setup.py` 그대로 빌드해 확인 |

## 6. 설치가 필요한 의존성 (LIMO 인벤토리 후 확정; 우선 후보)
- apt (Humble): `ros-humble-nav2-bringup`, `ros-humble-slam-toolbox`, `ros-humble-xacro`, `ros-humble-joint-state-publisher`, `ros-humble-robot-state-publisher`, `ros-humble-tf2-ros`, `ros-humble-robot-localization`(소스 복사본을 쓰지 않을 때), `python3-numpy`, `python3-scipy`(package.xml에 있으나 실차 경로 노드는 import하지 않음)
- 깊이 카메라를 쓸 때만: `ros-humble-cv-bridge`, `image-transport`, `image-publisher`, `camera-info-manager`, `libgoogle-glog-dev`, `libopenni2-dev`, `libeigen3-dev`, udev 규칙, `libuvc`·`magic_enum` 소스
- 시스템: **YDLidar SDK(`ydlidar_sdk`)** — `ydlidar_ros2_driver`가 `find_package(ydlidar_sdk REQUIRED)`.
- 주의: 노트북의 numpy는 2.2.6이다. LIMO의 numpy 버전은 미확인이며, 실차 경로 노드는 numpy만 쓴다.
- **apt 설치는 LIMO 팀 환경을 바꾸므로 설치 전에 반드시 목록을 승인받는다.**

## 7. 동기화 방법 (실제 배포 시; 지금은 실행하지 않음)
원칙: 팀 원본 `/home/pcu/capstone_ws`는 쓰기 금지(읽기·해시 비교만). 대상은 `/home/pcu/capstone_experiment_ws/src`뿐. `--delete` 사용 금지.
1. 읽기 전용 인벤토리 실행 → 결과 검토(3절 차이 확정).
2. 노트북에서 스테이징 디렉터리 생성: 2절의 T1(+선택 T2)만 복사. 해시를 `deploy_manifest.tsv`와 대조.
3. **변경 목록 제시**: LIMO의 실험 워크스페이스 해당 파일을 읽기 전용으로 내려받아(`rsync`로 노트북 임시 폴더로) `diff -ruN`을 만들고 사용자에게 보여 준다. 또한 `rsync -avnc --itemize-changes`(dry-run)로 전송 대상 목록을 보여 준다.
4. LIMO 고유 수정(해시가 노트북과 다르고 이력에 없는 파일)은 자동 덮어쓰지 않고 파일별로 병합 방식을 사용자와 정한다.
5. 승인 후: 대상 파일을 타임스탬프 백업(`capstone_experiment_ws/` 밖, 예 `~/deploy_backup_<날짜>/`)에 복사한 뒤 `rsync -av --backup --backup-dir=<백업>`로 T1 파일만 전송. 전송 후 원격에서 `sha256sum -c`로 확인.
6. 전송 경로는 `/home/pcu/capstone_experiment_ws/src/limo_capstone/` 하위로만 제한한다.

## 8. LIMO에서의 빌드 방법
```bash
source /opt/ros/humble/setup.bash
source /home/pcu/capstone_ws/install/setup.bash        # 팀 원본 install을 underlay로 읽기만 함 (팀 워크스페이스는 빌드하지 않음)
cd /home/pcu/capstone_experiment_ws
colcon build --packages-select limo_capstone limo_depth_obstacles   # 필요한 T3 패키지가 있으면 추가
source install/setup.bash
```
- `--symlink-install`은 쓰지 않는 것을 권한다(전송 후 파일 변경이 install에 그대로 반영되어 해시 확인이 어려워짐; 팀 환경 관례가 있으면 따른다).
- 빌드 결과(경고/오류)를 그대로 기록한다. 빌드 전에는 `colcon list`로 팀 원본과 이름이 겹치는 패키지가 없는지 확인한다.
- 각 파일 `use_sim_time` 확인: 빌드 후 launch는 하지 않고 `python3 -c "import ast..."`/`ros2 launch ... -s`(인자 표시)까지만 한다.

## 9. 모터 잠금 검증 절차 (단계별; 모든 단계에서 `allow_motor_drive:=true`는 사용하지 않음)
**단계 0 — 실행 없이 정적 확인 (로봇 전원/연결 불필요)**
1. real launch의 `allow_motor_drive` 기본값이 `false`인지, gate 출력 토픽이 `'true'` 문자열일 때만 `/cmd_vel`인지 확인(`grep`).
2. 설치본에서 `/cmd_vel`을 발행하는 노드가 gate 하나뿐인지 확인: launch 파일 안의 `SetRemap`과 Nav2 노드 remap을 읽는다(노트북에서 확인한 구조: controller → smoother → `/cmd_vel_raw`, behavior_server → `/cmd_vel_raw`).
3. `limo_base`가 `/cmd_vel`을 구독한다는 것을 확인(소스 `limo_driver.cpp`에서 확인함).

**단계 1 — 로봇 연결, 바퀴 들어 올림/ E-Stop 사람이 손에 닿는 위치 (소유자 승인 필요)**
`limo_base`는 시리얼을 열기 때문에 노드만 떠도 로봇과 통신한다. 따라서 이 단계는 소유자가 현장에서 바퀴를 띄우거나 전원 차단을 준비한 뒤에만 한다.
1. 기본 인자로만 실행: `ros2 launch limo_capstone limo_integrated_auto_mapping_real.launch.py` (인자 없음). 로그에서 `[SAFETY LOCK ACTIVE]` 확인.
2. 다른 터미널에서:
   - `ros2 topic info /cmd_vel -v` → 발행자 0, 구독자 `limo_base`
   - `ros2 topic info /cmd_vel_locked -v` → 발행자 `stall_detector_node`
   - `ros2 node info /stall_detector_node` → `/cmd_vel_locked`만 발행, `/cmd_vel` 없음
   - `timeout 5 ros2 topic echo --once /cmd_vel` → 메시지 없음
   - `ros2 param get /stall_detector_node use_sim_time` 및 Nav2/SLAM 노드 → false
   - `ros2 topic echo --once /stall_detector/status_json` → INITIALIZING → NORMAL, `depth_code`, `wheel_odom_stale=false`
3. 센서 확인(구동 없음): `/scan`, `/odom`, `/imu` 수신 주기, TF `odom→base_footprint`, `map→odom`.
**단계 2 — 라우팅 확인 (단계 1 통과 후, 선택)**: `/cmd_vel_raw`에 0 속도만 발행해 gate가 `/cmd_vel_locked`로 전달하는지 본다. 0이 아닌 값은 사용자가 별도 승인한 경우에만. 이때도 `/cmd_vel`은 계속 비어 있어야 한다.
- 실패 기준: `/cmd_vel`에 발행자가 생기거나 메시지가 나오면 즉시 중단(Ctrl-C).
- 권장: `allow_motor_drive:=true`가 들어간 명령을 거부하는 실행 래퍼 스크립트를 따로 만들어 실수를 막는다(제안만).

## 10. 미확인 / 사용자 결정이 필요한 것
- LIMO 접속 정보(호스트, 계정, 접속 방법)와 읽기 전용 인벤토리 실행 허가.
- 2·3·6절의 LIMO 쪽 사실 전부.
- LIMO Pro의 실제 주행 모드(4WD 차동 / Ackermann)와 launch가 적용하는 Ackermann BT의 일치.
- 팀 원본이 이미 가진 드라이버·의존성, apt 설치 허가 범위.
- F4의 벽 평행 문제를 구동 단계 전에 해결할지.
