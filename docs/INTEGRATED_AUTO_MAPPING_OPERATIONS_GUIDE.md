# LIMO Pro 통합 자동 맵핑 정식 운영 가이드 (Operations Guide)

**문서 번호:** GUIDE-CAPSTONE-WS-AUTO-MAPPING-20261108
**최종 갱신일:** 2026년 11월 8일
**적용 대상:** AgileX LIMO Pro (Ackermann Steering, 2D LiDAR, DaBai Depth Camera)
**지원 환경:** ROS 2 Humble Hawksbill (Ubuntu 22.04.5 LTS), Gazebo Fortress
**대상 워크스페이스:** `capstone_ws/` (`limo_capstone`, `limo_depth_obstacles`)

---

## 1. 개요 및 워크스페이스 구조

본 문서는 `capstone_ws`에 정식 이식된 **LIMO Pro 통합 자동 맵핑 시스템**의 빌드, 시뮬레이션 구동, 모니터링, 안전 게이트 운용, rosbag2 기록·재생 및 실물 적용 전 안전 수칙을 기술한 정식 운영 가이드입니다.

### 1.1 패키지 및 저장소 역할 분담
- **`capstone_ws` (메인 워크스페이스):** 정식 개발·빌드 및 런타임 실행 환경. `limo_capstone`, `limo_depth_obstacles` 패키지가 위치하며, 외부 참조 없이 자립 실행 가능합니다.
- **`experiments/limo_integrated_auto_mapping_20261108/` (실험 아카이브):** 검증 회귀 스크립트, 장시간 맵핑 데이터, 고장 주입 벤치마크 결과 JSON, 원본 rosbag2 기록 및 단계별 검증 보고서가 보존된 연구 아카이브입니다.

```text
capstone_ws/src/
├── limo_depth_obstacles/                    # [신규] 3D Depth 카메라 장애물 필터 패키지
│   ├── limo_depth_obstacles/
│   │   ├── depth_obstacle_filter_node.py   # 포인트클라우드 필터 및 상태 진단 노드 (0.38s 워치독)
│   │   └── filtering.py                    # 높이 윈도잉 [0.025, 0.35]m 및 셀프 반경 필터
│   ├── package.xml
│   └── setup.py
└── limo_capstone/                           # [확장] 캡스톤 내비게이션 통합 패키지
    ├── config/                              # 통합 파라미터 YAML 5종
    │   ├── fusion_f5_improved.yaml          # EKF 융합 설정 (F4 ICP + IMU wz)
    │   ├── integrated_explore_params.yaml   # explore_lite 프론티어 탐색 파라미터
    │   ├── integrated_nav2_params.yaml      # Nav2 Costmap (2D+3D) & RPP 제어 파라미터
    │   ├── integrated_slam_params.yaml      # SLAM Toolbox 온라인 비동기 맵핑 파라미터
    │   └── rosbag2_qos_overrides.yaml       # rosbag2 센서 토픽 QoS 프로파일 설정
    ├── launch/                              # 통합 Launch 2종
    │   ├── limo_integrated_auto_mapping_sim.launch.py   # 시뮬레이션 브링업 런치
    │   └── limo_integrated_auto_mapping_real.launch.py  # 실물 로봇 브링업 런치 (기본 모터 잠금)
    ├── limo_capstone/                       # 신규 노드 8종
    │   ├── depth_to_pointcloud_fixed_node.py
    │   ├── ground_truth_publisher_node.py
    │   ├── integrated_recorder_node.py
    │   ├── lidar_scan_matcher_odom_node.py  # F4: 2D LiDAR ICP 오도메트리 노드 (10 Hz)
    │   ├── nav2_action_monitor_node.py
    │   ├── odom_tf_relay_node.py            # odom -> base_footprint 단일 TF 중재 노드
    │   ├── sensor_normalizer_node.py
    │   └── stall_detector_node.py           # 안전 명령 게이트 (/cmd_vel_raw -> /cmd_vel)
    ├── models/                              # Lounge SDF 평가 모델 4종
    ├── urdf/                                # LIMO Ackermann 시뮬레이션 URDF
    └── worlds/                              # Lounge 평가 월드 SDF
```

---

## 2. 환경 준비 및 빌드 가이드

### 2.1 사전 요구사항
- Ubuntu 22.04 LTS, ROS 2 Humble Hawksbill 데스크톱 풀 설치
- Nav2, SLAM Toolbox, explore_lite, robot_localization 설치:
  ```bash
  sudo apt update
  sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox ros-humble-robot-localization
  ```

### 2.2 워크스페이스 빌드
```bash
# 1. 캡스톤 메인 워크스페이스 이동
cd ~/school/2026-2/Capstone1_limo/capstone_ws

# 2. ROS 2 기본 환경 로드
source /opt/ros/humble/setup.bash

# 3. 통합 패키지 선별 빌드 (심볼릭 링크 설치)
colcon build --symlink-install --packages-select limo_depth_obstacles limo_capstone

# 4. 빌드 결과물 환경 오버레이 및 라이브러리 경로 설정
source install/setup.bash
export LD_LIBRARY_PATH=$PWD/install/robot_localization/lib:/opt/ros/humble/lib:$LD_LIBRARY_PATH
```

---

## 3. 시뮬레이션 자동 맵핑 실행

시뮬레이션은 Gazebo Fortress의 실내 라운지 월드(`lounge_evaluation_world.sdf`)에서 LIMO Pro 아커만 조향 모델을 사용하여 실행됩니다.

### 3.1 F4 모드 실행 (2D LiDAR ICP 기반 탐색)
2D LiDAR 포인트 클라우드 간 ICP 정합 오도메트리를 기반으로 주행합니다.
```bash
ros2 launch limo_capstone limo_integrated_auto_mapping_sim.launch.py \
  use_sim_time:=true \
  estimator_type:=f4 \
  start_explore:=true \
  depth_mode:=required \
  sensor_timeout_sec:=0.38
```

### 3.2 F5_improved 모드 실행 (LiDAR ICP + IMU EKF 융합 기반 탐색)
F4 ICP 오도메트리와 IMU 요 각속도를 EKF로 융합하여 50Hz 고주파 오도메트리를 생성합니다.
```bash
ros2 launch limo_capstone limo_integrated_auto_mapping_sim.launch.py \
  use_sim_time:=true \
  estimator_type:=f5_improved \
  start_explore:=true \
  depth_mode:=required \
  sensor_timeout_sec:=0.38
```

### 3.3 주요 실행 인자(Launch Arguments)
| 인자명 | 기본값 | 선택 가능값 | 설명 |
| :--- | :---: | :---: | :--- |
| `use_sim_time` | `true` | `true`, `false` | Gazebo 시뮬레이션 클록(`/clock`) 사용 여부 |
| `estimator_type` | `f4` | `f4`, `f5_improved` | 상태 추정기 선택 |
| `start_explore` | `true` | `true`, `false` | explore_lite 프론티어 자동 탐색 활성화 여부 |
| `depth_mode` | `required` | `required`, `optional`, `disabled` | Depth 카메라 페일세이프 모드 |
| `sensor_timeout_sec`| `0.38` | 양의 실수 | 센서 패킷 미수신 허용 타임아웃 (초) |
| `record_video` | `false` | `true`, `false` | 오버헤드 주행 MP4 비디오 녹화 여부 |

---

## 4. 모니터링 및 운영 인터페이스

### 4.1 RViz2 시각화 설정
별도 터미널에서 RViz2를 실행하여 지도를 확인합니다:
```bash
source /opt/ros/humble/setup.bash
rviz2
```
- **Global Options:** `Fixed Frame` $\to$ `map` 설정
- **Map 디스플레이 추가:** Topic `/map`, Durability: `Transient Local`, Reliability: `Reliable`
- **LiDAR 디스플레이 추가:** Topic `/scan`, Reliability: `Best Effort`
- **3D Depth 장애물 추가:** Topic `/depth_obstacles/points`, Reliability: `Reliable`
- **TF 트리 확인:** `map -> odom -> base_footprint -> base_link` 체인 활성화

### 4.2 안전 래치(Safety Latch) 상태 확인 및 수동 해제
센서 단절이나 휠 슬립으로 인해 로봇이 정지(`FAILSAFE_SENSOR_TIMEOUT` 또는 `STALL_LATCHED`)하면 안전 게이트가 동작하여 `/cmd_vel` 출력을 `0.0 m/s`로 클램핑합니다.
- **안전 게이트 텔레메트리 모니터링:**
  ```bash
  ros2 topic echo /stall_detector/status_json
  ```
- **수동 안전 언래치(Reset Latch) 호출:**
  센서가 정상 회복된 후 로봇을 다시 주행시키려면 다음 서비스로 래치를 해제합니다.
  ```bash
  ros2 service call /stall_detector/reset_latch std_srvs/srv/Trigger
  ```

### 4.3 Nav2 Action 이벤트 모니터링
explore_lite가 발행하는 NavigateToPose 액션 목표 상태는 다음 토픽으로 모니터링할 수 있습니다:
```bash
# 액션 목표 상태 확인 (SUCCEEDED, EXECUTING, CANCELED 등)
ros2 topic echo /navigate_to_pose/_action/status

# 액션 피드백 확인 (현재 이동 속도, 남은 거리)
ros2 topic echo /navigate_to_pose/_action/feedback
```
> [!NOTE]
> Action Goal, Feedback, Status 토픽은 rosbag2 및 토픽 모니터링으로 수집 가능합니다. 단, 공식 Action Result 서비스 응답은 Nav2 클라이언트-서버 RPC 특성상 rosbag 기본 토픽으로 남지 않으므로 본 시스템에서는 **미기록(Not Captured)** 상태로 유지됩니다.

---

## 5. rosbag2 기록 및 재생 프로파일

### 5.1 F4 기본 주행 기록 프로파일 (실측 환산 ~7.0 MB/min)
기본 2D LiDAR ICP 오도메트리, 지도 확장, 제어 명령 모니터링용 프로파일:
```bash
ros2 bag record -o /tmp/limo_bag_f4 \
  /scan \
  /odom \
  /odometry/lidar_odom \
  /odom_nav2 \
  /tf \
  /tf_static \
  /map \
  /depth_obstacles/points \
  /cmd_vel_raw \
  /cmd_vel \
  /stall_detector/status_json
```

### 5.2 F5_improved 상세 진단 기록 프로파일 (실측 환산 ~8.7 ~ 9.2 MB/min)
F4 토픽 전량에 IMU 고주파(50Hz) 토픽 및 F5 EKF 융합 오도메트리를 추가한 전체 진단 프로파일:
```bash
ros2 bag record -o /tmp/limo_bag_f5_diag \
  /scan \
  /odom \
  /odometry/lidar_odom \
  /odometry/f5_improved \
  /odom_nav2 \
  /imu \
  /tf \
  /tf_static \
  /map \
  /depth_obstacles/points \
  /cmd_vel_raw \
  /cmd_vel \
  /stall_detector/status_json
```
*(※ 과거 문서의 0.8 MB/min은 고주기 TF를 제외한 미검증 이론치이며, 21.5초 실측 회귀 시험 데이터 기준 F4는 약 7.0 MB/min, F5는 약 8.7~9.2 MB/min이 기록되었습니다.)*

### 5.3 bag 재생 검증
현재 실행 중인 ROS 노드와의 간섭을 피하기 위해 독립된 `ROS_DOMAIN_ID`에서 재생합니다:
```bash
ROS_DOMAIN_ID=42 ros2 bag play /tmp/limo_bag_f5_diag --rate 1.0
```

---

## 6. 정상 종료 및 비상 정지 구분

### 6.1 소프트웨어 정상 종료 (Graceful Shutdown)
1. Launch 파일이 실행 중인 터미널 창에서 **`Ctrl + C`**를 1회 입력합니다.
2. ROS 2 Launch 시스템이 SIGINT를 전파하여 `ign gazebo`, `nav2`, `slam_toolbox`, `explore_node`, `stall_detector_node` 등 모든 자식 프로세스를 약 3~5초 내에 정상 회수합니다.
3. 모든 노드가 `process has finished cleanly` 메시지를 출력하고 쉘 프롬프트로 복귀하는지 확인합니다.

> [!WARNING]
> **비상 정지(E-Stop) 아님:**
> `Ctrl + C`는 소프트웨어 인터럽트 기반의 정상 종료 절차일 뿐, 하드웨어 비상 정지 장치가 아닙니다. 온보드 PC의 커널 패닉, 통신 두절, ROS 2 데드락 발생 시에는 `Ctrl + C`가 전달되지 않으므로 비상 정지 수단으로 사용할 수 없습니다.

> [!IMPORTANT]
> 정식 운영 가이드에서는 시스템 전역 프로세스를 무차별 강제 종료하는 `pkill -9` 사용을 엄격히 배제합니다. 시스템 전체 SIGKILL은 다른 ROS 작업이나 백그라운드 서비스를 파괴할 위험이 있습니다.

### 6.2 프로세스 고립 시 안전 회수
터미널 비정상 종료 등으로 Gazebo 데몬이 백그라운드에 고립된 경우에 한하여, 해당 세션 프로세스만 선별 종료합니다:
```bash
kill -SIGINT $(pgrep -f "limo_capstone|limo_depth_obstacles|ign gazebo") 2>/dev/null
```

---

## 7. 기술 정합성 및 검증 측정 기준 (Specification)

### 7.1 F5 EKF 융합 입력의 실제 구성 (`fusion_f5_improved.yaml`)
- **`odom0` (`/odometry/lidar_odom` - F4 ICP):** $x, y, \text{yaw}, v_x, v_y$ (2D 평면 위치 및 선속도)
- **`imu0` (`/imu` - LIMO IMU):** $\omega_z$ ONLY (요 각속도만 융합, 가속도 및 롤/피치는 제외)
- **원시 휠 오도메트리 (`/odom`):** 슬립 오차 전파 방지를 위해 EKF 융합에서 배제하고 독립 모니터링용으로 분리.
- **TF 브로드캐스팅:** EKF의 `publish_tf: false`로 설정하여 [`odom_tf_relay_node`](../capstone_ws/src/limo_capstone/limo_capstone/odom_tf_relay_node.py)가 `odom -> base_footprint` TF 권한을 단일 소유.

### 7.2 Depth 센서 단절 페일세이프 시간 및 변위 분석
실제 센서 프로세스 강제 종료 시뮬레이션(Test B) 결과:
- **마지막 정상 메시지부터 명령 차단까지:** **0.421초** ($t_{\text{cutoff}} - t_{\text{last\_depth}} = 6.521 - 6.100\text{s}$)
  (워치독 타임아웃 0.380s + 필터/제어 루프 주기 0.041s)
- **프로세스 종료(Kill)부터 명령 차단까지:** **0.263초** ($t_{\text{cutoff}} - t_{\text{kill}} = 6.521 - 6.258\text{s}$)
- **구동 명령 차단 시점:** $t = 6.521\text{s}$ (`/cmd_vel` 0.0 m/s 강제 클램핑)
- **차체 물리 완전 정지 시점:** $t = 6.703\text{s}$ (관성 감속 완료, $v < 0.005\text{ m/s}$)
- **0.165m 변위 해석 (UNVERIFIED 분류):**
  - 고장 주입($6.258\text{s}$)부터 정지($6.703\text{s}$)까지의 경과 시간은 **$0.445\text{초}$**이며, $0.15\text{ m/s}$ 등속 주행 시 이론상 최대 이동 거리는 $0.15 \times 0.445 = \mathbf{0.0668\text{m}}$(약 6.7cm)입니다.
  - $0.165\text{m}$는 이론상 최대치($0.0668\text{m}$)의 약 2.5배이며, 차단 시점($6.521\text{s}$) 이후 $0.182\text{초}$ 구간 평균 속도로 환산 시 **$0.907\text{ m/s}$**가 되어 순항 속도($0.15\text{ m/s}$)와 물리적으로 불일치합니다.
  - 평가 스크립트(`execute_full_regression_audit.py`)는 $p_0$와 $p_1$의 원본 좌표 및 GT 시계열을 별도 저장하지 않고 스칼라 결과만 남겼으므로 사후 원천 재현이 불가능합니다.
  - 따라서 임의 수치 수정 없이 해당 제동 거리를 공식 **UNVERIFIED (미검증)**로 분류합니다.

### 7.3 검증 범위 구분
- **22초 브링업 회귀 감사:** 단일 TF 트리 무결성, F4/F5 주기 충족(10.0Hz / 50.0Hz), 초기 지도 격자 확장($1.63 \to 1.70\text{ m}^2$, $1.64 \to 1.72\text{ m}^2$), rosbag2 수집 및 페일세이프 차단 등 "패키지 인터페이스 및 기동 무결성"을 검증.
- **80초 장시간 자율 탐색 벤치마크 (`EXTENDED_INTEGRATION_VERIFICATION_REPORT.md`):**
  - **F4:** 총 주행 거리 3.005 m, 총 지도 확장 64.77 m² (자유 영역 62.42 m², 937 cells), 벽면 두께 표준편차 $\sigma = 0.0860\text{ m}$, 최종 상태 NORMAL 무중단 완주.
  - **F5_improved:** 총 주행 거리 2.351 m, 총 지도 확장 63.00 m² (자유 영역 60.81 m², 878 cells), 벽면 두께 표준편차 $\sigma = 0.0904\text{ m}$, 최종 상태 STALL_LATCHED.

---

## 8. 실물 로봇 적용 전 안전 가이드 및 체크리스트

> [!CAUTION]
> **실물 로봇 자율 주행 엄격 금지 (STRICTLY BLOCKED):**
> 독립적인 물리적 비상 정지(E-Stop) 장치 및 하위 STM32 MCU 워치독이 무부하 벤치 시험대에서 실측 입증되기 전까지는 실물 로봇의 자율 주행을 승인하지 않습니다.

### 8.1 소프트웨어 잠금(`allow_motor_drive:=false`)의 의미와 한계
- Launch 파일의 `allow_motor_drive:=false` 설정은 Nav2 제어 명령을 모터 드라이버 토픽(`/cmd_vel`) 대신 모의 토픽([`/cmd_vel_locked`](../capstone_ws/src/limo_capstone/limo_capstone/stall_detector_node.py#L380-L390))으로 우회시킵니다.
- **한계:** 이는 온보드 PC의 ROS 2 스택이 정상 작동할 때만 유효한 **순수 소프트웨어적 차단**입니다. 운영체제 정지, 전원 차단, 커널 패닉 시에는 안전을 담보하지 못합니다.

### 8.2 하드웨어 안전의 미검증 상태 (UNVERIFIED)
- **STM32 MCU 통신 워치독:** 제조사 설명상 PC와 시리얼 통신 두절 시 200ms 후 모터가 정지한다고 알려져 있으나, 실제 펌웨어 동작 및 타임아웃 값은 **실측 미검증(UNVERIFIED)** 상태의 참고치입니다.
- **물리 비상 정지(E-Stop):** LIMO Pro 기구부에는 즉각적인 전원 차단용 물리 E-Stop 버튼이 없습니다 (**UNVERIFIED**).
- **RC 조종기 비상 채널:** 원격 RC 조종기의 비상 스위치 기능 및 수신기 맵핑은 **실측 미검증(UNVERIFIED)** 상태입니다.

> [!WARNING]
> **위험 시험 금지:** 제조사의 공식 안전 문서 및 펌웨어 스펙을 정식으로 확인하기 전에는, 주행 중 USB 통신 케이블을 강제로 분리하는 시험을 **수행하거나 권장하지 않습니다**. (기구부 손상 및 급발진 위험)

### 8.3 실물 적용 전 사전 점검 체크리스트
```markdown
[ ] 1. 하드웨어 거치대 준비 (바퀴 4개가 공중에 완전히 뜬 무부하 거치 상태 확인)
[ ] 2. LIMO 배터리 전압 점검 (브라운아웃 방지 확인, 단 12.0V/11.5V는 미검증 참고치 [UNVERIFIED])
[ ] 3. 실제 센서 모델 확인 (2D LiDAR YDLiDAR T-mini Plus / DaBai Depth Camera)
[ ] 4. 센서 장착 정적 TF 실측 (LiDAR laser_link x=+0.103m, Depth depth_camera_link x=+0.12m)
[ ] 5. limo_base 드라이버의 자체 odom TF 브로드캐스트 비활성화 (odom_tf_relay 충돌 방지)
[ ] 6. allow_motor_drive:=false 상태에서 실물 센서 토픽 수신율 확인 (/scan, /depth_obstacles/points, /imu)
[ ] 7. 온보드 물리 전원 스위치 즉시 차단 대기조 배치 (Ctrl+C는 E-Stop 아님, RC 비상 채널은 미검증 [UNVERIFIED])
[ ] 8. 0.1 m/s 저속 수동 주행을 통한 휠 인코더 거리 실측 및 슬립 보정
[ ] 9. 무부하 벤치에서 통신 단절 시 모터 정지 실측 (200ms는 미검증 참고치, 케이블 강제 탈착 금지)
[ ] 10. 독립 물리 E-Stop 및 MCU 워치독 벤치 입증 전까지 실물 자율 주행 엄격 금지 (STRICTLY BLOCKED)
```
