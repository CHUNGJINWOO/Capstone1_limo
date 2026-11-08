# LIMO Pro 자동 맵핑 통합 아키텍처 1~3단계 구현 및 Smoke Test 결과 보고서

## 1. 개요 및 목적

본 보고서는 사용자 피드백을 반영하여 수립된 **LIMO Pro 자동 맵핑 통합 아키텍처**의 1~3단계를 구현하고, Gazebo Fortress 시뮬레이션 상에서 단일 통합 Smoke Test를 수행하여 검증한 결과입니다.

### 1.1 핵심 보완 원칙 준수 내역
1. **기존 시스템 및 실험 보존**:
   - `capstone_ws` 및 이전 실험 디렉터리의 원본 파일은 일체 수정하지 않고 보존.
   - 신규 통합 코드는 독립 경로([`experiments/limo_integrated_auto_mapping_20261108/`](..))에서 작성 및 실행.
   - Git 커밋 및 푸시 일체 배제.
2. **TF 및 오도메트리 단일 권한 체계 확립**:
   - `odom -> base_footprint` 변환은 선택된 추정기(F4 또는 F5_improved)의 중계 노드만 유일하게 발행.
   - 기존 `limo_base`의 중복 TF 및 launch 파일 내 역방향 `static_transform_publisher`(`base_link -> base_footprint`) 완전 제거.
   - `map -> odom`은 SLAM Toolbox가 50Hz로 독점 소유.
   - 원시 휠 오도메트리 토픽(`/odom`)을 훼손 없이 독립 보존.
3. **F4 / F5_improved 선택형 상태 추정**:
   - `estimator_type` 런치 인자(`'f4'` 또는 `'f5_improved'`)를 통해 런타임에 동적으로 전환 가능하도록 모듈화.
   - 라이다 신뢰성 상실 시 IMU 단독 병진 추정 가정 배제 및 페일세이프 정책 수립.

---

## 2. 신규 통합 구조의 파일 배치 및 변경 영향 범위

### 2.1 파일 배치 구조
```
experiments/limo_integrated_auto_mapping_20261108/
├── config/
│   ├── integrated_slam_params.yaml      # SLAM Toolbox 50Hz PGO 및 base_footprint 기준 설정
│   └── fusion_f5_improved.yaml          # F5_improved EKF (LiDAR x,y,yaw,vx,vy + IMU wz) 설정
├── launch/
│   ├── base_gazebo_for_integrated.launch.py # Fortress 시뮬레이터 + URDF + 센서 브리지
│   └── integrated_stage1_to_3.launch.py     # 1~3단계 통합 메인 런치 (F4/F5 선택형)
├── nodes/
│   ├── lidar_scan_matcher_odom_node.py  # F4 2D LiDAR Trimmed-ICP 오도메트리 노드
│   ├── odom_tf_relay_node.py            # odom -> base_footprint 단일 TF 권한 중계 노드
│   ├── ground_truth_publisher_node.py   # Ground Truth 검증 노드
│   ├── trial_sensor_normalizer_node.py  # 센서 토픽 규격화 노드
│   ├── stall_detector_node.py           # 사전 배포 Stall Detector 안전 노드 (4단계 준비)
│   └── depth_to_pointcloud_fixed_node.py# Depth PointCloud 변환 노드 (4단계 준비)
├── bin/
│   └── ekf_node                         # robot_localization C++ EKF 실행 바이너리
├── results/
│   └── stage1_to_3_smoke_test_summary.json # Smoke Test 실측 데이터 요약
└── docs/
    └── STAGE1_TO_3_SMOKE_TEST_REPORT.md   # 본 보고서
```

---

## 3. 1~3단계 통합 구현 상세

### 3.1 1단계: TF 트리 및 센서 입력 연결
- **단일 소유권 TF 체계**:
  $$\text{map} \xrightarrow[\text{slam\_toolbox}]{\text{50Hz}} \text{odom} \xrightarrow[\text{odom\_tf\_relay}]{\text{50Hz}} \text{base\_footprint} \xrightarrow[\text{robot\_state\_publisher}]{\text{Static}} \text{base\_link} \xrightarrow[\text{URDF}]{\text{Static}} \text{laser\_link}$$
  - `odom_tf_relay_node.py`가 선택된 오도메트리 토픽을 받아 유일하게 `odom -> base_footprint` TF를 브로드캐스트합니다.
  - TF 중복, 역방향 연결, 순환 루프(Cycle)가 0건임을 런타임 상에서 검증했습니다.
- **원시 센서 보존**:
  - 원시 휠 오도메트리는 `/odom` 토픽으로 유지되어 다른 진단 노드가 접근 가능합니다.

### 3.2 2단계: F4/F5 선택형 상태 추정 연동
- [`integrated_stage1_to_3.launch.py`](../launch/integrated_stage1_to_3.launch.py) 런치 인자 `estimator_type` 구현:
  - `estimator_type:=f4`:
    - `lidar_scan_matcher_odom_node`가 `/odometry/lidar_odom` (10Hz) 발행.
    - `odom_tf_relay_node`가 해당 토픽을 바인딩하여 TF 발행.
  - `estimator_type:=f5_improved`:
    - `lidar_scan_matcher_odom_node`가 베이스 포즈/속도를 제공하고,
    - `ekf_node`가 IMU 고주파 각속도($\omega_z$)를 융합하여 `/odometry/f5_improved` (50Hz) 발행.
    - `odom_tf_relay_node`가 50Hz EKF 토픽을 바인딩하여 TF 발행.
  - **라이다 단절 시 정책**: F5는 바퀴 슬립 오염 방지를 위해 선속도 입력으로 라이다 속도($v_x, v_y$)만을 사용하므로, 라이다가 단절되면 선형 위치 추정이 불가능합니다. 따라서 라이다 단절 시 맹목적 IMU 적분을 금지하고 0.45초 내 안전 정지(`FAILSAFE_SENSOR_TIMEOUT`)하도록 설계되었습니다.

### 3.3 3단계: SLAM Toolbox 자동 맵핑 연동
- `slam_toolbox/async_slam_toolbox_node`를 신규 파라미터([`integrated_slam_params.yaml`](../config/integrated_slam_params.yaml))로 연결:
  - `base_frame: base_footprint`, `odom_frame: odom`, `map_frame: map`
  - Ceres Solver 기반으로 스캔 매칭을 수행하며 50Hz 주기로 `map -> odom` TF를 갱신.
  - 로봇 주행에 따라 실시간으로 `/map` OccupancyGrid를 확장 생성.

---

## 4. Smoke Test 실측 검증 결과 ([`stage1_to_3_smoke_test_summary.json`](../results/stage1_to_3_smoke_test_summary.json))

동일한 Gazebo Fortress 시뮬레이션 환경에서 F4 모드와 F5_improved 모드를 순차 실행하여 1~3단계 통합 동작을 실측했습니다.

| 검증 항목 (Metric) | F4 모드 실측치 (`estimator_type:=f4`) | F5_improved 모드 실측치 (`estimator_type:=f5_improved`) | 판정 기준 및 결과 |
| :--- | :---: | :---: | :---: |
| **라이다 스캔 수신 (`/scan`)** | **75 샘플** 수신 (정상) | **73 샘플** 수신 (정상) | $\ge 10$ 샘플 $\to$ **PASS** |
| **IMU 수신 (`/imu`)** | **726 샘플** 수신 (정상) | **716 샘플** 수신 (정상) | $\ge 20$ 샘플 $\to$ **PASS** |
| **원시 휠 오도메트리 (`/odom`)** | **363 샘플** 수신 (정상 보존) | **정상 보존** | 원본 토픽 분리 $\to$ **PASS** |
| **추정기 출력 수신** | **74 샘플** (`/odometry/lidar_odom`, 10Hz) | **312 샘플** (`/odometry/f5_improved`, 50Hz) | F5가 50Hz 고주파 출력 $\to$ **PASS** |
| **`map -> odom` TF** | **정상 100% 연결** (SLAM Toolbox 소유) | **정상 100% 연결** (SLAM Toolbox 소유) | TF 단절/사이클 없음 $\to$ **PASS** |
| **`odom -> base_footprint` TF** | **정상 100% 연결** (F4 Relay 단일 소유) | **정상 100% 연결** (F5 Relay 단일 소유) | 단일 발행자 보장 $\to$ **PASS** |
| **`base_footprint -> laser_link` TF** | **정상 100% 연결** (URDF 트리) | **정상 100% 연결** (URDF 트리) | 전체 TF 체인 완성 $\to$ **PASS** |
| **주행 구동 명령 (`/cmd_vel`)** | $v_x = 0.22\text{ m/s}$ (3.5초 정상 주행) | $v_x = 0.22\text{ m/s}$ (3.5초 정상 주행) | 주행 모터 구동 $\to$ **PASS** |
| **SLAM 점유 셀 생성 (`/map`)** | **53 셀** 점유 생성 (맵 크기 $177 \times 237$) | **54 셀** 점유 생성 (맵 정상 확장) | 점유 셀 $> 25$ $\to$ **PASS** |
| **최종 판정 (Verdict)** | **`PASS`** | **`PASS`** | **1~3단계 100% 정상 통과** |

---

## 5. 단계별 판정 요약 및 후속 단계 계획

- **1단계 (TF 및 센서 연결)**: **`VERIFIED`** (중복/역방향 TF 제거, 단일 권한 체계 확립, 원시 휠 오도메트리 보존)
- **2단계 (F4/F5 선택형 추정기)**: **`VERIFIED`** (`estimator_type` 런치 전환 확인, F5 50Hz EKF 출력 확인)
- **3단계 (SLAM Toolbox 연동)**: **`VERIFIED`** (Ceres 스캔 매칭 및 `/map` 점유 격자 실시간 생성 확인)
- **4단계 ~ 6단계 (Depth Camera, Nav2, Stall Detector)**: **`READY FOR IMPLEMENTATION`** (다음 단계에서 확장 구현)
