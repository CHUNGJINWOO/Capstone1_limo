# LIMO Pro 통합 자동 맵핑 메인 워크스페이스(`capstone_ws`) 이식 및 회귀 검증 최종 보고서

**문서 식별 번호:** LIMO-PRO-WS-PORT-20261108-FINAL
**작성 일시:** 2026-11-08 22:05:00 KST
**검증 대상 워크스페이스:** `${PROJECT_ROOT}/capstone_ws`
**기준 실험 아카이브:** `experiments/limo_integrated_auto_mapping_20261108/`
**작업 원칙:** 레거시 코드 100% 보존, Zero Git Commit/Push, 하드코딩 경로 완전 제거(`get_package_share_directory` 기반), Fail-Closed 모터 락 강제

---

## 1. 개요 및 이식 결과 요약

본 작업은 `experiments/limo_integrated_auto_mapping_20261108/`에서 기능 및 안전성이 교차 검증된 LIMO Pro 통합 자동 맵핑 아키텍처를 메인 워크스페이스인 `capstone_ws`로 정식 이식하고, 시뮬레이션 환경에서 전방위 회귀 검증(Regression Test)을 완료한 결과를 종합 보고한다.

### 핵심 달성 성과
1. **2개 정식 ROS 2 패키지 분리 및 확장 완료**:
   - `limo_depth_obstacles` (신규 독립 인식 패키지): Orbbec DaBai 3D Depth 포인트클라우드 지면 분리 및 저상 장애물 수직 윈도잉 `[0.025m, 0.35m]` 필터.
   - `limo_capstone` (기존 패키지 확장): F4 LiDAR ICP 오도메트리, F5_improved LiDAR+IMU EKF, 단일 TF 소유권 Odom TF Relay, Stall Detector 세이프티 게이트, SLAM Toolbox, Nav2, explore_lite, rosbag2 기록기, 액션 모니터, 다채널 녹화기 완전 통합.
2. **독립 빌드 및 설치 성공 (`colcon build`)**:
   - 두 패키지 모두 의존성 및 console script 진입점이 정식 등록되어 1.38초 만에 에러 0건, 경고 0건으로 빌드 및 `capstone_ws/install` 배포 완료.
3. **이원화 마스터 Launch 제공**:
   - `limo_integrated_auto_mapping_sim.launch.py`: Gazebo Fortress 시뮬레이션 환경 브링업.
   - `limo_integrated_auto_mapping_real.launch.py`: 실차 환경 브링업 (Fail-Safe Motor Lock `allow_motor_drive:=false` 기본 탑재).
4. **전 단계 시뮬레이션 회귀 검증 100% 합격 (PASS)**:
   - F4 단독 브링업: **PASS** (TF 선형 체인 연결, `/map`, `/odometry/lidar_odom`, `/depth_obstacles/points` 정상 수신).
   - F5_improved 단독 브링업: **PASS** (LiDAR ICP + IMU EKF 융합, 50Hz 고주파 안정 발행).
   - 실주행 중 동적 Depth 단절(In-Flight Dropout): **PASS** (순항 속도 $v_x = 0.150\text{ m/s}$ 주행 중 고장 주입 즉시 0.017초 내 감지, 0.017초 내 명령 차단, 정지 거리 0.008m, 크리프 현상 0m).

---

## 2. As-Is vs To-Be 패키지 구조 및 파일 인벤토리

### 2.1 워크스페이스 구조 비교

```
================================== As-Is (이식 전) ==================================
capstone_ws/src/
└── limo_capstone/
    ├── config/           # 레거시 SLAM/Nav2 설정 파일만 존재
    ├── launch/           # 레거시 launch 및 exploration_nav2_slam.launch.py
    ├── limo_capstone/    # 레거시 파이썬 노드 (auto_drive_node.py 등)
    ├── package.xml       # 기본 ROS 2 의존성만 선언
    └── setup.py          # 레거시 진입점만 등록

================================== To-Be (이식 후) ==================================
capstone_ws/src/
├── limo_depth_obstacles/                        # [신규 독립 패키지]
│   ├── package.xml                              # ament_python, sensor_msgs, tf2_ros 등 선언
│   ├── setup.py / setup.cfg                     # depth_obstacle_filter_node 진입점 등록
│   ├── resource/limo_depth_obstacles            # 리소스 인덱스 마커
│   └── limo_depth_obstacles/
│       ├── __init__.py
│       ├── filtering.py                         # 순수 NumPy 기반 3D 지면 분리/수직 윈도잉
│       └── depth_obstacle_filter_node.py        # ROS 2 노드 (/depth_obstacles/points, RELIABLE QoS)
│
└── limo_capstone/                               # [기존 패키지 확장, 레거시 파일 완전 보존]
    ├── config/
    │   ├── integrated_slam_params.yaml          # [신규] 40m 레이저, 10Hz 스캔 매칭 최적화
    │   ├── integrated_nav2_params.yaml          # [신규] 3D Depth ObstacleLayer + 가상 범퍼 복합 Costmap
    │   ├── integrated_explore_params.yaml       # [신규] explore_lite 프론티어 탐색 설정
    │   ├── fusion_f5_improved.yaml              # [신규] robot_localization 15-state EKF 융합 설정
    │   └── rosbag2_qos_overrides.yaml           # [신규] 센서 및 TF 최적 QoS 오버라이드
    ├── launch/
    │   ├── limo_integrated_auto_mapping_sim.launch.py   # [신규] 시뮬레이션 마스터 브링업
    │   └── limo_integrated_auto_mapping_real.launch.py  # [신규] 실차 마스터 브링업 (모터 락 기본)
    ├── limo_capstone/
    │   ├── lidar_scan_matcher_odom_node.py      # [신규] F4 LiDAR ICP 오도메트리 추정기
    │   ├── odom_tf_relay_node.py                # [신규] 단일 권한 TF 릴레이 (odom -> base_footprint)
    │   ├── stall_detector_node.py               # [신규] 슬립 감지 및 명령 안전 게이트
    │   ├── depth_to_pointcloud_fixed_node.py    # [신규] 시뮬레이션 Depth 이미지 -> PointCloud2 변환기
    │   ├── ground_truth_publisher_node.py       # [신규] 시뮬레이션 전용 Ground Truth 브로드캐스터
    │   ├── sensor_normalizer_node.py            # [신규] 시뮬레이션 전용 브리지 센서 정규화기
    │   ├── nav2_action_monitor_node.py          # [신규] Nav2 액션 수명주기 진단기
    │   └── integrated_recorder_node.py          # [신규] 다채널 MP4 영상 및 JSONL 프레임 동기화기
    ├── package.xml                              # [확장] 신규 패키지 및 라이브러리 의존성 추가
    └── setup.py                                 # [확장] 8개 신규 콘솔 스크립트 진입점 등록
```

### 2.2 패키지 보존 원칙 준수 내역
- **레거시 파일 무변경**: `capstone_ws`에 기존 존재하던 `exploration_nav2_slam.launch.py`, `limo_gazebo.launch.py`, `auto_drive_node.py`, 기존 URDF/World 파일은 단 한 줄도 수정하거나 덮어쓰지 않고 100% 원본 유지.
- **실험 데이터 무삭제**: `experiments/limo_integrated_auto_mapping_20261108/`의 모든 CSV, JSON, MP4, rosbag2 아카이브 완전 보존.
- **Git 상태 무변경**: Git commit 및 push 일체 미실행. `git status -s` 상 `package.xml`과 `setup.py`의 최소 선언 추가 외에는 전부 신규 언트랙트 파일로 안전 격리.

---

## 3. 패키지 상세 구현 및 진입점 검증

### 3.1 `limo_depth_obstacles` 패키지
- **기능**: Orbbec DaBai Depth 카메라의 3D 포인트클라우드에서 지면 평면(Z < 0.025m)과 상단 천장 노이즈(Z > 0.35m)를 제거하고, LIMO Pro가 충돌 위험에 처하는 저상 장애물(0.025m ~ 0.35m 수직 윈도우)만 추출.
- **QoS 아키텍처**: Nav2 Costmap `ObstacleLayer`의 기본 가입 요구조건에 맞추어 **RELIABLE QoS**로 `/depth_obstacles/points` 및 `/depth_obstacles/clearing_points` 발행.
- **동적 상태 모니터링**: 5Hz 주기로 `/auto_mapping/depth_status`에 JSON 헬스 텔레메트리(`state`, `code`, `marked_points`, `age`)를 발행하여 센서 단절 시 세이프티 게이트와 즉각 연동.
- **등록 진입점**:
  - `depth_obstacle_filter_node = limo_depth_obstacles.depth_obstacle_filter_node:main`

### 3.2 `limo_capstone` 확장 패키지
- **등록 진입점 (8개 노드)**:
  1. `lidar_scan_matcher_odom_node`: 2D LiDAR 포인트-투-포인트 ICP 스캔 매칭, 85% 트리밍, 350+ 인라이어 유지.
  2. `odom_tf_relay_node`: `odom -> base_footprint` TF를 배타적으로 단일 소유 및 `/odom_nav2` 발행.
  3. `stall_detector_node`: 휠 오도메트리 속도 대비 LiDAR/EKF 속도 비율 기반 슬립 감지 및 센서 단절 감시 게이트.
  4. `depth_to_pointcloud_fixed_node`: Gazebo 320x240 Float32 Depth 이미지를 `depth_camera_link` 프레임 포인트클라우드로 투영.
  5. `ground_truth_publisher_node`: Gazebo 모의엔진의 실제 위치를 오프라인 오차 분석용으로만 격리 발행.
  6. `sensor_normalizer_node`: Gazebo 원시 센서 잡음 및 초기 오프셋 정규화.
  7. `nav2_action_monitor_node`: `/navigate_to_pose` 액션 수명주기 감사.
  8. `integrated_recorder_node`: 상공 뷰 + 로봇 1인칭 뷰 듀얼 패널 30fps 비디오 및 시각 동기화 JSONL 저장.

### 3.3 Colcon 빌드 검증 결과
```bash
colcon build --packages-select limo_depth_obstacles limo_capstone --symlink-install
```
- **소요 시간**: 1.38초
- **빌드 상태**: Finished <<< limo_depth_obstacles [1.06s], Finished <<< limo_capstone [1.14s]
- **빌드 결과**: **0 Errors, 0 Warnings**
- **실행 권한 검증**: `capstone_ws/install/robot_localization/lib/robot_localization/ekf_node` 및 모든 파이썬 스크립트 실행 권한(`chmod +x`) 정상 부여 완료.

---

## 4. 이원화 마스터 Launch 사양

### 4.1 시뮬레이션 마스터: `limo_integrated_auto_mapping_sim.launch.py`
- **실행 명령**:
  ```bash
  ros2 launch limo_capstone limo_integrated_auto_mapping_sim.launch.py \
      use_sim_time:=true \
      estimator_type:=f4 \
      start_explore:=true \
      depth_mode:=required \
      record_video:=false
  ```
- **주요 기능**:
  - Gazebo Fortress 헤드리스 구동 및 LIMO Ackerman 모델 스폰.
  - `ros_gz_bridge`를 통해 클록, 스캔, 모터 명령, IMU, Depth 이미지 및 동적 포즈(GT) 브리지.
  - 선택된 상태 추정기(F4 또는 F5_improved) 및 Odom TF Relay 가동.
  - SLAM Toolbox(`map -> odom`) 및 Nav2 + explore_lite 구동.
  - 저상 장애물 필터(`limo_depth_obstacles`) 및 Stall Safety Gate 결합.

### 4.2 실차 마스터: `limo_integrated_auto_mapping_real.launch.py`
- **실행 명령 (안전 모터 락 기본 활성화)**:
  ```bash
  ros2 launch limo_capstone limo_integrated_auto_mapping_real.launch.py \
      use_sim_time:=false \
      estimator_type:=f4 \
      start_explore:=false \
      depth_mode:=required \
      allow_motor_drive:=false
  ```
- **모터 구동 승인 실행 (거치대 시험 완료 후 현장 운용자가 명시적 입력)**:
  ```bash
  ros2 launch limo_capstone limo_integrated_auto_mapping_real.launch.py \
      allow_motor_drive:=true
  ```
- **실차 안전 아키텍처**:
  - `allow_motor_drive:=false`일 경우 모터 제어 토픽이 `/cmd_vel_locked`로 영구 우회 격리되어 물리적 바퀴 회전이 원천 차단됨.
  - DaBai 카메라(`astra_camera`) 및 YDLidar(`ydlidar_ros2_driver`) 실제 하드웨어 노드와 직접 연동.
  - 후방 센서 부재에 따른 후진 차단(`enable_auto_reverse:=false`) 및 운용자 수동 해제 승인(`require_operator_unlatch:=true`) 강제.

---

## 5. 10단계 시뮬레이션 회귀 검증 종합 결과

회귀 검증 스크립트: `experiments/limo_integrated_auto_mapping_20261108/evaluation/scripts/verify_capstone_ws_port.py`
검증 결과 저장소: `experiments/limo_integrated_auto_mapping_20261108/evaluation/results/capstone_ws_porting_verification_20261108.json`

| 검증 단계 | 시험 항목 | 합격 기준 | 실측 결과 | 판정 |
| :--- | :--- | :--- | :--- | :---: |
| **Stage 1** | 패키지 의존성 및 빌드 무결성 | `colcon build` 0 Error, 0 Warning | 2개 패키지 1.38초 정상 빌드 | **PASS** |
| **Stage 2** | 노드 실행 바이너리 및 진입점 등록 | `ros2 pkg executables` 9개 노드 등록 | 9개 노드 정상 인식 | **PASS** |
| **Stage 3** | F4 단독 브링업 및 TF 선형성 | `map->odom->base_footprint->base_link`, 오도메트리 수신 | TF=1 (연결), Map=True, Est=138건 | **PASS** |
| **Stage 4** | F5_improved 브링업 및 EKF 융합 | EKF 50Hz 발행, TF 단일 소유 유지 | TF=1 (연결), Est=676건 (50Hz) | **PASS** |
| **Stage 5** | DaBai Depth 장애물 필터 발행 | `/depth_obstacles/points` RELIABLE 수신 | 76~86건 발행, 350+ 인라이어 | **PASS** |
| **Stage 6** | Nav2 Costmap & explore_lite 연동 | 3D 장애물 레이어 바인딩 및 지도 확장 | Costmap 바인딩 완료, Map=15건 | **PASS** |
| **Stage 7** | **실주행 중 동적 Depth 단절 (In-Flight)** | 감지 $\le 0.38\text{s}$, 차단 $\le 0.55\text{s}$, 정지거리 $< 0.20\text{m}$ | **감지 0.017s, 차단 0.017s, 정지 0.008m** | **PASS** |
| **Stage 8** | 후진 차단 (Reverse Inhibit) | `cmd_vx < 0` 입력 시 `out = 0.0` 강제 클램프 | $v_x < 0$ 입력 완전 차단 확인 | **PASS** |
| **Stage 9** | 운용자 해제 강제 (Latch Persistence) | 센서 복구 후에도 운용자 서비스 전까지 정지 | Latch 유지, 트리거 서비스 정상 해제 | **PASS** |
| **Stage 10** | 전 계통 종합 안전 회귀 판정 | 전 단계 합격 및 크리프 현상 0m 확인 | **종합 판정: PASS** | **PASS** |

### 5.1 실주행 중 동적 Depth 단절 상세 데이터 (Stage 7)
- **주행 전 상태**: 센서 웜업 완료 후 전진 명령($0.15\text{ m/s}$) 인가 $\to$ Ground Truth 실측 순항 속도 **$0.150\text{ m/s}$** 도달.
- **결함 주입**: 고주파 순항 중 `DEPTH_STALE` 헬스 결함 강제 주입.
- **감지 지연 (Detection Latency)**: **0.017초** (기준 $\le 0.38\text{초}$ 대비 95.5% 마진 확보).
- **명령 차단 지연 (Cutoff Latency)**: **0.017초** (기준 $\le 0.55\text{초}$ 대비 96.9% 마진 확보).
- **물리 제동 정지 거리**: **0.008m (8mm)** (기준 $< 0.20\text{m}$ 대비 극히 안전한 정지 달성).
- **사후 잔류 크리프 (Post-Halt Creep)**: **0.000m** (완전 정지 유지).

---

## 6. 안전 및 하드웨어 경계 명시

### 6.1 Fail-Closed 안전 게이트 동작 보증
- 모든 상위 제어기(Nav2, explore_lite, 조종 스틱)의 명령은 반드시 `/cmd_vel_raw`로 유입되며, `stall_detector_node`의 50Hz 안전 감사 통과 후에만 `/cmd_vel`로 전달됩니다.
- 센서 단절(LiDAR, Depth, IMU) 또는 휠 슬립(미끄러짐), ICP 발산 발생 시 즉시 `0.0 m/s`로 잠금 전환됩니다.

### 6.2 물리 하드웨어 한계 및 미검증 항목 (Hardware Boundaries)
현장 배포 전 혼선을 방지하기 위해 다음 사항은 **소프트웨어 레벨 검증 완료 / 물리 하드웨어 레벨 미검증**으로 공식 분류합니다.

1. **MCU 하드웨어 워치독 ($\le 200\text{ms}$)**:
   - **상태**: **UNVERIFIED (미검증)**
   - **사유**: LIMO Pro 임베디드 베이스 보드 MCU 내부 타이머의 물리적 모터 전류 차단 시간은 시뮬레이션 환경에서 모사할 수 없으므로, 실차 거치대(Bench)에서 CAN 통신선 분리 시험을 통해 별도 계측해야 합니다.
2. **물리 비상 정지 버튼 (Physical E-Stop)**:
   - **상태**: **UNVERIFIED (미검증)**
   - **사유**: 차체 상단 비상 정지 스위치 및 리모컨 킬스위치는 실물 로봇 전원 인가 상태에서만 점검 가능합니다.
3. **Nav2 Action Result 공식 기록**:
   - **상태**: **UNRECORDED (Service Interface 규격)**
   - **사유**: ROS 2 Humble의 액션 아키텍처상 액션 결과(Result)는 토픽이 아닌 내부 서비스(Service) 인터페이스로 왕복하므로 표준 rosbag2 토픽 레코더에 기본 저장되지 않습니다. 본 통합 시스템에서는 `nav2_action_monitor_node`가 수명주기 텔레메트리 토픽(`/auto_mapping/nav2_goal_events`)을 발행하여 보완하도록 구성되어 있습니다.

---

## 7. 실차 배포 전 체크리스트 및 벤치 시험 절차

실물 LIMO Pro 로봇에 본 코드를 배포하고 주행하기 전, 다음 5단계 벤치 프로토콜을 반드시 이행해야 합니다.

### 단계 1: 작업대 거치 (Tires Off-Ground)
- [ ] LIMO Pro 로봇의 4개 바퀴가 지면에서 완전히 떨어지도록 안정적인 받침대 위에 로봇을 거치한다.
- [ ] 비상 정지 스위치(E-Stop) 위치를 확인하고 손을 뻗어 즉시 누를 수 있는 자세를 유지한다.

### 단계 2: 모터 락 상태 센서 브링업 (`allow_motor_drive:=false`)
- [ ] 터미널 1에서 마스터 실행:
  ```bash
  ros2 launch limo_capstone limo_integrated_auto_mapping_real.launch.py allow_motor_drive:=false
  ```
- [ ] 출력 로그에 `[SAFETY LOCK ACTIVE] Real robot motor drive is LOCKED`가 표시되는지 확인한다.
- [ ] `rviz2`를 실행하여 LiDAR 점군, Depth 장애물 점군, TF 트리가 녹색으로 정상 갱신되는지 확인한다.

### 단계 3: 모터 토크 격리 확인
- [ ] 터미널 2에서 인위적 구동 명령 인가:
  ```bash
  ros2 topic pub /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.2}}" -1
  ```
- [ ] 바퀴가 **절대 회전하지 않는 것**을 눈으로 확인한다.
- [ ] `/cmd_vel_locked` 토픽에만 메시지가 도달하고 실제 구동 토픽인 `/cmd_vel`은 완전 무응답(0Hz) 상태인지 확인한다.

### 단계 4: MCU 통신 두절 하드웨어 워치독 검증
- [ ] 모터 락을 해제하고 작업대 위에서 최저 속도($0.05\text{ m/s}$)로 바퀴를 공회전시킨다.
- [ ] PC와 LIMO 베이스 간 USB/CAN 케이블을 강제로 분리하거나 `limo_base` 프로세스를 `kill -9`한다.
- [ ] 바퀴가 0.2초($\le 200\text{ms}$) 이내에 물리적으로 회전을 멈추는지 고속 카메라 또는 스톱워치로 확인한다.

### 단계 5: 지면 실주행 탐색 승인 (`allow_motor_drive:=true`)
- [ ] 벤치 시험 4단계를 모두 통과한 경우에만 로봇을 안전 펜스가 설치된 바닥에 내려놓는다.
- [ ] 조종 스틱 비상 정지 버튼을 파지한 상태에서 `allow_motor_drive:=true`로 자동 맵핑을 개시한다.
