# LIMO Pro 자동 맵핑 통합 아키텍처 4~6단계 및 마스터 Smoke Test 최종 검증 보고서

**작성 일시:** 2026-11-08
**실험 환경:** `${PROJECT_ROOT}/experiments/limo_integrated_auto_mapping_20261108/`
**대상 로봇 플랫폼:** AgileX LIMO Pro (Ackermann Steering, Orbbec DaBai Depth Camera, 2D LiDAR, IMU, Wheel Encoders)
**기반 시뮬레이터:** Gazebo Fortress (`lounge_evaluation_world.sdf`, `lounge_world`)
**ROS 2 배포판:** ROS 2 Humble

---

## 1. 개요 및 종합 판정 (Executive Summary)

LIMO Pro 자동 맵핑 시스템의 실물 적용을 위한 단계별 기능 통합(Stage 4 ~ Stage 6) 및 단일 통합 Launch 기반 마스터 주행 검증(Master Smoke Test)을 완료하였다.

모든 단계는 선행 단계의 실제 계측 데이터 검증을 거친 후 순차적으로 진행되었으며, **전 단계 100% 합격(PASS)** 을 기록하였다.

### 통합 검증 요약 매트릭스

| 검증 단계 | 검증 대상 및 항목 | 주요 계측치 | 판정 기준 | 최종 판정 |
| :--- | :--- | :--- | :--- | :---: |
| **Stage 4A** | 정적 목표 지점 Nav2 경로 계획 및 추종 | 오차 0.236m, 주행거리 0.813m, 소요시간 5.92s | 오차 $\le$ 0.25m, `SUCCEEDED` | **PASS** |
| **Stage 4B** | `explore_lite` 동적 Frontier 자율 탐색 | 계획 20회, 명령 300회, 1.000m 주행, 334셀 탐색 | 주행거리 $\ge$ 0.50m, 맵 생성 | **PASS** |
| **Stage 5A** | 2D LiDAR 단독 (8cm 바닥 장애물 사각지대) | 장애물 거리 0.197m (충돌 발생), Depth 프레임 0개 | 취약점 확인 (`CONFIRMED_VULNERABLE`) | **PASS** |
| **Stage 5B** | 2D LiDAR + DaBai Depth 복합 감지 | 장애물 거리 0.302m (충돌 방지), 89,938점 장애물 등록 | 제동 거리 $\ge$ 0.25m, 무충돌 | **PASS** |
| **Stage 6A** | 투명 유리벽 충돌 시 Stall Detector 래칭 | 지연시간 3.343s, 명령 0.0 m/s 강제 고정, 후진 0.000m | `STALL_LATCHED`, 후진 억제 | **PASS** |
| **Stage 6B** | 운용자 수동 래치 해제 서비스 (`/reset_latch`) | 서비스 응답 `success=True`, 상태 `NORMAL` 복귀 | 운용자 해제 정상 동작 | **PASS** |
| **Stage 6C** | 센서 단절 / 지연 Failsafe 타임아웃 | 단절 감지 0.202s ($\le$ 0.45s), 명령 0.0 m/s 차단 | `FAILSAFE_SENSOR_TIMEOUT` | **PASS** |
| **Master** | 단일 통합 Launch 마스터 자율 맵핑 (35s) | 1,138셀 매핑, 5.500m 주행, 204회 Depth 연동 | 10개 서브시스템 동시 가동 | **PASS** |
| **Video** | 1920x960 듀얼패널 시뮬레이션 MP4 녹화 | 555 프레임 (37.0초), 15.0 FPS, 용량 10.29 MB | 재생 및 프레임 무결성 검증 | **PASS** |

---

## 2. 4단계: Nav2 및 Explore 통합 검증 (Stage 4)

### 2.1 아키텍처 및 구성
- **경로 추종 제어기:** `nav2_regulated_pure_pursuit_controller` (최대 선속도 0.30 m/s, 곡률 감속 0.10 m/s, 전방 주시거리 0.40m ~ 0.80m).
- **전역 경로 계획기:** Nav2 `nav2_navfn_planner` (A* 알고리즘).
- **자율 탐색기:** `explore_lite` (글로벌 코스트맵 `/global_costmap/costmap`의 미탐색 영역 경계선(Frontier)을 실시간 클러스터링하여 동적 목표 발행).
- **TF 및 오도메트리 단일 소유권:**
  - `map → odom`: SLAM Toolbox Online Asynchronous 노드 독점 발행.
  - `odom → base_footprint`: `odom_tf_relay_node` 단독 발행 (F4 Scan Matcher 또는 F5 EKF 중 선택된 단일 추정기 반영).
  - Nav2는 단일 TF 체인과 `/odom_nav2`를 구독하여 TF 역방향 루프 및 다중 오너 충돌 0건 달성.

### 2.2 서브테스트 계측 결과
- **Subtest 4A (정적 목표 지점 추종):**
  - 출발 좌표: $(x=-1.10\text{m}, y=-0.60\text{m})$
  - 목표 좌표: $(x=-0.287\text{m}, y=-0.60\text{m})$ (전방 1.00m)
  - 최종 도달 시간: **5.92초**
  - 최종 오차: **0.236m** (허용공차 0.25m 이내 충족)
  - Action Server 상태: `SUCCEEDED`
  - 판정: **PASS**
- **Subtest 4B (자율 Frontier 탐색):**
  - 가동 시간: 25.0초
  - 생성된 탐색 경로 수: 20회
  - 발행된 구동 명령 수: 300회
  - 자율 주행 누적 거리: **1.000m**
  - 생성된 점유 지도 셀 수: **334 셀**
  - 판정: **PASS**

---

## 3. 5단계: DaBai Depth 3D 장애물 회피 검증 (Stage 5)

### 3.1 2D LiDAR 사각지대 실측 및 Depth VoxelLayer 연동
2D LiDAR는 바닥에서 높이 $z = 0.14\text{m}$에 수평으로 장착되어 있어, 라운지 복도 바닥에 놓인 $8\text{cm}$ ($z = 0.08\text{m}$) 높이의 턱이나 슬레드 체어 프레임을 전혀 감지하지 못하는 근본적 사각지대를 갖는다.

이를 극복하기 위해 Orbbec DaBai Depth 카메라의 3D 포인트클라우드를 Nav2 코스트맵의 `ObstacleLayer`에 직접 융합하였다.

### 3.2 DDS QoS 불일치 해결
- **문제점:** 업스트림 패키지가 포인트클라우드를 `BEST_EFFORT` QoS로 발행하였으나, Nav2 `ObstacleLayer`의 기본 구독자는 `RELIABLE`을 요구하여 DDS 계층에서 토픽이 무시되는 현상 확인.
- **해결책:** `nodes/depth_obstacle_filter_node.py`를 신규 구현하여 `QoSProfile(reliability=ReliabilityPolicy.RELIABLE)`로 코스트맵 호환성을 완벽히 확보함.
- **수직 필터링 윈도우:** `base_footprint` 기준 높이 $0.025\text{m} \sim 0.35\text{m}$ (바닥 반사 노이즈 2.5cm 이하는 배제하고 8cm 턱과 35cm 의자 다리만 정확히 추출).

### 3.3 실험 비교 결과
- **Subtest 5A (2D LiDAR 단독, Depth 비활성화):**
  - Depth 포인트클라우드 수신: **0 프레임 (0점)**
  - 2D LiDAR는 8cm 턱 위를 수평으로 통과하여 전방을 장애물 없는 빈 공간(Free-space)으로 인식.
  - 로봇이 전방 0.558m를 직진하여 허들 전면부에 정면 충돌 ($d = 0.197\text{m} \le 0.22\text{m}$).
  - 결과: 사각지대 취약성 실증 완료 (`CONFIRMED_VULNERABLE`, **PASS**)
- **Subtest 5B (2D LiDAR + DaBai Depth 카메라 복합 감지):**
  - 수신된 Depth 클라우드: **104 프레임**
  - 필터링된 유효 3D 장애물 점: **89,938 점**
  - Nav2 로컬 및 글로벌 코스트맵에 허들이 치명적 장애물(Lethal Obstacle, Cost=254)로 즉각 반영됨.
  - 로봇이 장애물 전방 $0.302\text{m}$ 지점에서 안전하게 감속 후 정지 (충돌 0건, 거리 마진 $\ge 0.25\text{m}$ 확보).
  - 결과: 3D 장애물 회피 및 비충돌 안전 정지 입증 (**PASS**)

---

## 4. 6단계: Stall Detector 및 최종 Safety Command Gate 검증 (Stage 6)

### 4.1 제어 경로 단일화 및 우회 차단 (Single Command Gate)
- Nav2 `controller_server` 및 `behavior_server`의 속도 출력 토픽을 `/cmd_vel_raw`로 완전 격리.
- `stall_detector_node`만이 `/cmd_vel_raw`를 수신하여 센서 정합성을 실시간(50Hz) 검증한 후, 안전이 보장된 경우에만 최종 액추에이터 토픽 `/cmd_vel`로 명령을 전달.
- 시스템 전체에서 `/cmd_vel`을 발행하는 노드가 오직 `stall_detector_node` 하나뿐임을 검증하여 우회 경로(Bypass) 0건 확인.

### 4.2 실물 안전 규칙 강제 적용
1. `enable_auto_reverse = False` (강제 비활성화):
   LIMO Pro 후방에는 초음파나 라이다 센서가 없으므로, 충돌 시 후진 자동 복구는 의자, 책상 등 후방 장애물과의 2차 충돌 위험을 초래함. 따라서 후진 복구를 원천 차단함.
2. `enable_auto_rotate_recovery = False` (강제 비활성화):
   벽면에 끼인 상태에서 무리한 제자리 회전 시 모터 과부하 및 기구 파손 방지.
3. `require_operator_unlatch = True` (운용자 수동 해제 강제):
   래치 발생 시 `/cmd_vel`을 영구히 $0.0\text{ m/s}$로 잠그며, 운용자의 명시적인 `/stall_detector/reset_latch` 서비스 호출 없이는 자율 재출발을 엄격히 차단.

### 4.3 계측 결과
- **Subtest 6A (유리벽 정면 충돌 및 Stall 래칭):**
  - 투명 유리벽($y = -1.50\text{m}$)으로 전진(0.20 m/s) 주행. 라이다 광선은 유리를 통과하여 VIP룸을 관측하므로 휠 속도는 0.260 m/s로 유지되나, F4 라이다 오도메트리 차체 속도는 0.023 m/s로 급락.
  - 속도 불일치 검출 후 **0.220초** 만에 `STALL_LATCHED` 상태로 진입.
  - 출력 `/cmd_vel` 즉시 선속도 $0.000\text{ m/s}$, 각속도 $0.000\text{ rad/s}$로 완전 차단.
  - 충돌 후 후진 이동 거리: **0.0000m** (후진 억제 성공).
  - 래치 유지 시간: 1.5초 이상 안정적으로 잠금 유지 확인 (**PASS**).
- **Subtest 6B (운용자 수동 래치 해제):**
  - 구동 명령 영점화 확인 후 `/stall_detector/reset_latch` 트리거 서비스 호출.
  - 응답: `success=True`, 상태가 `NORMAL`로 안전 복귀 완료 (**PASS**).
- **Subtest 6C (센서 단절 타임아웃 Fail-Safe):**
  - 정상 주행 중 F4 라이다 오도메트리 노드를 인위적으로 강제 종료.
  - 센서 단절 발생 후 **0.202초** ($\le 0.45\text{s}$) 만에 `FAILSAFE_SENSOR_TIMEOUT` 상태로 진입.
  - `/cmd_vel` 출력을 즉시 $0.0\text{ m/s}$로 완전 차단하여 폭주 방지 입증 (**PASS**).

### 4.4 소프트웨어 명령 차단 vs 물리 비상정지(E-Stop)의 명확한 구분
> [!IMPORTANT]
> **소프트웨어적 명령 차단 (Software Command Zeroing):**
> 본 시스템의 Stall Detector는 유저스페이스 ROS 2 노드로서, 토픽 `/cmd_vel`을 0으로 발행하여 모터 제어 보드에 "속도 0"을 지시하는 감독형 제어 가드(Supervisory Guard)이다. 이는 Linux OS 커널, DDS 미들웨어, MCU 펌웨어가 정상 동작할 때만 유효하다.
>
> **물리적 비상정지 (Hardware Emergency Stop):**
> 실제 실물 LIMO Pro 운용 시에는 모터 전원 공급 라인에 물리적 비상정지 스위치(E-Stop Button) 또는 하드웨어 릴레이를 연결하여, 소프트웨어 멈춤이나 OS 커널 크래시 발생 시에도 액추에이터 전원을 물리적으로 완전 차단할 수 있는 독립된 2중 안전 회로가 반드시 병행되어야 한다.

---

## 5. 전체 마스터 통합 실행 검증 (Smoke Test 종합)

### 5.1 단일 통합 Launch 구성
단일 실행 명령으로 시스템 전체가 동시 구동되는 통합 론치를 확립하였다:
- 파일 경로: `launch/limo_integrated_auto_mapping_master.launch.py`
- 동시 실행 노드 (총 10개 서브시스템):
  1. Gazebo Fortress (`lounge_evaluation_world.sdf`) + LIMO Pro 베이스
  2. ROS-GZ Bridge (클럭, 라이다, IMU, 오도메트리, Depth 영상, 오버헤드 영상)
  3. 센서 정규화 노드 (`trial_sensor_normalizer_node.py`)
  4. Ground Truth 발행 노드 (`ground_truth_publisher_node.py`)
  5. F4 LiDAR Scan Matching Odometry (`lidar_scan_matcher_odom_node.py`)
  6. 단일 권한 Odom TF Relay (`odom_tf_relay_node.py`)
  7. SLAM Toolbox 온라인 비동기 맵핑 (`async_slam_toolbox_node`)
  8. Orbbec DaBai Depth 3D 장애물 필터 (`depth_obstacle_filter_node.py`)
  9. Nav2 내비게이션 스택 (Planner, Controller, Behavior, BT Navigator, Lifecycle)
  10. 자율 탐색기 (`explore_lite`) + 안전 게이트 (`stall_detector_node.py`) + 듀얼패널 녹화기 (`integrated_auto_mapping_recorder_node.py`)

### 5.2 35초 연속 자율 주행 실측 데이터
- **테스트 일시:** 2026-11-08 17:32:34
- **주행 모드:** `explore_lite` 완전 자율 탐색 모드
- **실행 시간:** 35.0초 (영상 기록 37.0초)
- **점유 지도 생성 셀 수:** **1,138 셀** (기준 150셀 대비 758% 달성)
- **자율 주행 누적 이동 거리:** **5.500 m** (기준 0.70m 대비 785% 달성)
- **동적 경로 계획 발행 수:** **33 회**
- **DaBai Depth 클라우드 처리 수:** **204 회** (총 **464,043 개** 3D 포인트 필터링)
- **안전 게이트 통과 명령 수:** 원시 명령 525회 수신 $\to$ 안전 명령 1,458회 인가
- **비정상 정체 / 오경보 횟수:** **0 회** (자유 주행 영역에서 오작동 0건)
- **최종 안전 상태:** `NORMAL` 유지

---

## 6. 녹화 영상 (MP4) 무결성 및 HUD 검증

- **비디오 파일 경로:** `limo_integrated_auto_mapping_master.mp4` *(로컬 생성 비디오 원본 - 대용량 미추적)*
- **인코딩 포맷:** MPEG-4 Video (mp4v), OpenCV 호환
- **해상도:** **1920 x 960** (Dual-Panel 동시 렌더링)
- **프레임 레이트:** **15.0 FPS**
- **총 프레임 수:** **555 프레임** (총 37.0초 분량)
- **파일 크기:** **10.29 MB** (10,294,209 bytes)
- **패널 레이아웃:**
  - **좌측 패널 (1280x960):** Gazebo Fortress 실제 오버헤드 카메라 영상, Ground Truth 궤적(초록색), F4 추정 궤적(하늘색), Nav2 실시간 계획 경로(노란색), 로봇 위치 십자선, 실시간 SimTime 및 상태 배너.
  - **우측 상단 패널 (640x460):** 2D Top-Down Metric Trajectory Canvas (월드 좌표계 격자 및 누적 이동 궤적).
  - **우측 하단 패널 (640x500):** 실시간 정량 계측 HUD (활성 추정기, SLAM 매핑 셀 수, 자율 탐색 상태, DaBai Depth 감지 점수, Stall Detector 안전 게이트 상태, 원시 명령 대 출력 명령 비교, ATE RMSE 오차).

---

## 7. 결론 및 실물 LIMO Pro 적용 가이드

1. **기능 통합 완료:**
   상태 추정(F4/F5), SLAM 매핑, Nav2 자율 주행, DaBai 3D 장애물 회피, Stall Detector 안전 게이트가 단일 Launch 및 단일 TF 체계 하에서 상호 간섭이나 데드락 없이 안정적으로 연동됨을 완벽히 입증하였다.
2. **실물 하드웨어 이식 시 주의사항:**
   - 실물 로봇 배포 시 `depth_obstacle_filter_node.py`의 토픽명을 Orbbec DaBai 드라이버의 실제 출력 토픽인 `/camera/depth/points`로 일치시킨다.
   - 휠 오도메트리와 LiDAR 스캔 매칭 간의 캘리브레이션 오차를 줄이기 위해 로봇 베이스 파라미터(`wheel_track`, `wheel_radius`)를 최종 점검한다.
   - Stall Detector의 안전 파라미터(`enable_auto_reverse = False`, `require_operator_unlatch = True`)를 반드시 유지하여 후방 사각지대 충돌을 방지한다.
   - 물리적 비상정지 버튼(E-Stop)의 하드웨어 차단 동작을 소프트웨어 구동 전에 최우선 점검한다.
