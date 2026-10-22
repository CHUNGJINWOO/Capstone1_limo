# LIMO를 이용한 실내 자율 순찰 및 안내 (LIMO Indoor Autonomous Patrol & Guidance)

> **2026학년도 2학기 캡스톤디자인 1 (4조)**<br>
> **개인 개발 저장소 및 엔지니어링 포트폴리오**<br>
> **주요 담당 분야:** 자율주행(Navigation2), TF 좌표계, Costmap, 위치추정(AMCL) 및 동적 장애물 회피

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble_Hawksbill-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04.5_LTS-E95420?logo=ubuntu)](https://ubuntu.com/)
[![Platform](https://img.shields.io/badge/Platform-AgileX_LIMO_Pro-orange)]()
[![Navigation](https://img.shields.io/badge/Stack-Navigation2_(Nav2)-007ACC)]()

**AgileX LIMO Pro**를 활용하여 실내 환경에서 자율주행, 순찰 및 목적지 안내가 가능한 ROS 2 기반 로봇 시스템을 개발합니다.

---

## 1. 프로젝트 소개

### 프로젝트명
**LIMO를 이용한 실내 자율 순찰 및 안내**

### 프로젝트 목적
본 프로젝트는 AgileX LIMO Pro를 기반으로 실내 환경에서 안정적인 자율주행 시스템을 구축하고, 이를 활용하여 **자율 순찰 및 목적지 안내 서비스**를 구현하는 것을 목표로 합니다.

이를 위해 LiDAR를 이용한 실내 지도 작성과 위치 추정, Navigation2 기반 경로 계획 및 주행, 동적 장애물 대응 등의 기능을 단계적으로 구현하고 실제 주행 환경에서 검증합니다.

최종적으로는 복도 및 로비와 같은 실내 환경에서 LIMO가 지정된 경로를 따라 자율적으로 순찰하고, 사용자의 호출에 따라 목적지까지 안내할 수 있는 시스템을 구축하는 것을 목표로 합니다.

---

## 2. 주요 개발 범위

본 프로젝트의 주요 개발 범위는 다음과 같습니다.

- LiDAR 기반 실내 환경 인식
- SLAM 기반 실내 지도 작성
- AMCL 기반 로봇 위치 추정
- Navigation2(Nav2) 기반 자율주행
- Global / Local Path Planning
- Costmap 구성 및 활용
- 경로 추종
- 정적 및 동적 장애물 대응
- 동적 장애물 회피
- Waypoint 기반 자율 순찰
- ROS 2 Action 기반 순찰 로직
- 목적지 안내 기능
- 실제 주행 환경에서의 파라미터 튜닝
- 주행 성능 및 기능 검증

---

## 3. 시스템 구성

전체 시스템은 다음과 같은 흐름으로 구성합니다.

```text
                    AgileX LIMO Pro
                           │
             ┌─────────────┼─────────────┐
             │             │             │
           LiDAR          IMU         Odometry
             │             │             │
             └─────────────┼─────────────┘
                           │
                           ▼
                    센서 데이터 처리
                           │
              ┌────────────┴────────────┐
              │                         │
              ▼                         ▼
          SLAM Mapping              Localization
              │                       (AMCL)
              │                         │
              └────────────┬────────────┘
                           ▼
                    Navigation2
                           │
          ┌────────────────┼────────────────┐
          │                │                │
          ▼                ▼                ▼
    Global Planner    Local Planner      Costmap
          │                │                │
          └────────────────┼────────────────┘
                           ▼
                      경로 추종
                           │
                           ▼
                    장애물 대응
                           │
                           ▼
                         LIMO
                           │
              ┌────────────┴────────────┐
              ▼                         ▼
          자율 순찰                    목적지 안내
```

위 구성은 프로젝트의 최종 목표 시스템이며, 각 기능은 실제 구현 및 검증을 거쳐 단계적으로 통합합니다.

---

## 4. 저장소 구조

저장소의 주요 디렉터리 구성은 다음과 같습니다.

```text
├── docs/                                  # 프로젝트 기술 문서 및 개발 기록
│   ├── weekly-progress.md                 # 1~8주차 주차별 개발 기록 (개인 기여 중심)
│   ├── troubleshooting.md                 # 핵심 트러블슈팅 7선 가이드
│   ├── navigation.md                      # Nav2 아키텍처 및 주행 파이프라인 분석
│   ├── roadmap.md                         # 구현 상태 매트릭스 및 향후 로드맵
│   ├── DEVELOPMENT_LOG.md                 # 시스템 점검 이력 및 일자별 개발 기록
│   ├── PROJECT_PLAN.md                    # 프로젝트 전체 개발 계획 및 단계별 목표
│   ├── NAV2_TROUBLESHOOTING.md            # Nav2 자율주행 검증 및 문제 분석
│   ├── SLAM_MAPPING.md                    # SLAM 및 지도 작성 검증
│   └── PHASE12_SIMULATION_VALIDATION.md   # 시뮬레이션 환경 검증 기록
├── capstone_ws/                           # 캡스톤 메인 ROS 2 워크스페이스
│   └── src/
│       ├── limo_capstone/                 # 캡스톤 통합 패키지 (Launch, Config, Maps)
│       │   ├── config/                    # Nav2, SLAM, 탐색 파라미터 YAML
│       │   ├── launch/                    # limo_nav2, limo_auto_mapping 등 런치
│       │   ├── maps/                      # 점유격자지도 (demo_map, MC_2F, MC_3F 등)
│       │   ├── scripts/                   # 2D 맵-Gazebo 월드 변환 스크립트
│       │   └── setup.py                   # 파일 필터링 배포 정의 (Commit b46aba9)
│       ├── limo_ros2/                     # LIMO 기본 구동 드라이버 및 모델
│       ├── m-explore-ros2/                # 프론티어 탐색 소스 (middle 패치 적용)
│       └── ydlidar_ros2_driver/           # 2D 라이다 드라이버 (std::round 패치)
├── scripts/
│   └── remote_teleop_node.py              # 원격 조향 제어 퍼블리셔 노드
└── src/
    └── limo_ros2/
        ├── limo_base/                     # LIMO 통신 및 기본 구동 드라이버
        ├── limo_car/                      # 로봇 모델 및 시뮬레이션 패키지
        ├── limo_description/              # URDF / Xacro 및 시각화 리소스
        └── limo_msgs/                     # 커스텀 ROS 2 메시지 정의
```

### `docs`
- `weekly-progress.md`: 1~8주차까지의 활동 목표, 개인 수행 작업, 문제 해결 및 검증 결과를 정리한 주차별 개발 기록입니다.
- `troubleshooting.md`: 하드웨어 전원 브라운아웃, QoS 불일치, `setup.py` 빌드 에러, TF 누락, 탐색 무한 루프, Lifecycle 지도 미수신 등 핵심 오류 7선에 대한 분석과 해결책을 표준 형식으로 수록합니다.
- `navigation.md`: Nav2 아키텍처, 10개 TF 프레임 무결성 실측치, AMCL 및 Costmap 파라미터, 실행 가이드를 상세히 분석합니다.
- `roadmap.md`: 4단계 구현 상태 매트릭스(확인됨, 검증 부족, 수정 중, 계획)와 동적 장애물 회피, 순찰, 안내 기능의 아키텍처 설계를 수록합니다.
- `DEVELOPMENT_LOG.md`: LIMO 온보드 환경에서의 하드웨어 연결, 센서 검증, SLAM 매핑, Nav2 연동 및 문제 해결 과정을 일자별로 상세히 기록합니다.
- `PROJECT_PLAN.md`: 캡스톤 디자인 목표, 개발 단계(Phase 1~9), 팀원별 역할 분담 및 최종 평가 항목을 정의합니다.

### `scripts`
- `remote_teleop_node.py`: ROS 2 `geometry_msgs/msg/Twist` 표준 인터페이스를 사용하여 LIMO의 속도 명령(`/cmd_vel`)을 발행하는 조향 제어 스크립트입니다.

### `src/limo_ros2` 및 `capstone_ws`
LIMO Pro 구동을 위한 핵심 ROS 2 패키지입니다.
- **`limo_base`**: 하부 MCU와의 시리얼 통신, 기본 주행 제어 노드, 센서 및 TF 브로드캐스팅, LiDAR 실행 런치 파일을 포함합니다.
- **`limo_car`**: LIMO 모델(Ackermann 등), Gazebo 시뮬레이션 환경, 센서 모델, 시뮬레이션 런치 파일을 포함합니다.
- **`limo_description`**: 로봇 형상 URDF/Xacro, 3D 메쉬, Gazebo 설명 파일 및 RViz 시각화 설정을 포함합니다.
- **`limo_msgs`**: LIMO 시스템 전용 커스텀 ROS 2 메시지 정의를 포함합니다.
- **`limo_capstone`**: 캡스톤 프로젝트 메인 패키지로 맵핑, Nav2 주행, 파라미터, 지도 파일을 통합 관리합니다.

---

## 5. 구현 과정 및 빌드 가이드

본 프로젝트에서는 기능을 한 번에 통합하지 않고, **기본 소스 확인 → 센서 및 TF 확인 → 지도 작성 → 위치 추정 → Navigation2 → 장애물 대응 → 순찰 및 안내**의 순서로 단계적으로 개발합니다.

---

### 5.1 LIMO ROS 2 기본 소스 및 구현 상태

프로젝트는 AgileX에서 제공하는 LIMO ROS 2 소스를 기반으로 구성하며, 현재 저장소의 구현 및 준비 상태는 다음과 같습니다.

| 구분 | 대상 | 현재 상태 | 상세 설명 |
|---|---|:---:|---|
| **기본 패키지** | `limo_msgs`, `limo_description`, `limo_base`, `limo_car` | **완료** | `colcon build --symlink-install` 전체 빌드 성공 |
| **제어 스크립트** | `scripts/remote_teleop_node.py` | **완료** | ROS 2 표준 `geometry_msgs/Twist` 기반 조향 퍼블리셔 |
| **로봇 시각화** | URDF, Xacro, RViz2 런치 | **완료** | 차동 구동 및 Ackermann 조향 모델 시각화 확인 |
| **실내 SLAM (수동 맵핑)** | SLAM Toolbox 2D 점유격자지도 생성 | **완료** | `demo_map`, `domi_map`, `MC_2F_10_07`, `MC_3F_10_15` 지도 취득 및 저장 확인 |
| **패키지 빌드 자동화** | `setup.py` 파일 필터링 배포 | **완료** | `os.path.isfile(f)` 패치로 지도 13종 자동 배포 정상화 (Commit `b46aba9`) |
| **좌표계 (TF)** | `base_link → base_footprint` 연동 | **완료** | 정적 TF 브로드캐스터 추가로 Costmap TF 타임아웃 해소 (Commit `7439841`) |
| **TF 계측** | 10개 프레임 단일 트리 연결 | **완료** | `tf2_tools view_frames` 계측 완료 (`frames_2026-10-13_19.38.47.gv`) |
| **지도 수신 및 Lifecycle** | Map Server 수명주기 및 QoS 일치 | **완료** | Lifecycle 노드 활성화 및 `Transient Local` 설정으로 지도 수신 정상화 |
| **자율주행 (Nav2)** | 정적 지도 기반 AMCL & 목표점 주행 | **완료 (실차)** | 연구실 및 미래관 복도/세미나실 실차 경로 계획 및 주행 완주 확인 |
| **자율 탐색 (Explore Lite)** | `frontier->middle` 목표 선정 수정 | **완료 (시뮬레이션)** | 동일 좌표 무한 성공 루프 소멸 확인 (Commit `7439841`). 실물 복도 평가는 진행 중 |
| **동적 장애물 회피** | 전방 감지 노드 및 능동 회피 제어기 | **계획** | 아키텍처 설계 완료, 노드 패키징 및 실차 검증 예정 ([docs/roadmap.md](docs/roadmap.md)) |
| **순찰 / 안내** | Action 기반 Waypoint 순찰 및 안내 | **계획** | 서비스 시나리오 및 상태 머신 설계 완료, 노드 구현 예정 ([docs/roadmap.md](docs/roadmap.md)) |

---

### 5.2 빌드 환경 및 방법

#### 개발 환경
- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **빌드 도구**: `colcon`

#### 빌드 명령어
저장소 루트 디렉터리 또는 `capstone_ws`에서 아래 명령어를 순차적으로 실행하여 패키지를 빌드하고 환경을 설정합니다.

```bash
# 1. ROS 2 Humble 기본 환경 로드
source /opt/ros/humble/setup.bash

# 2. 전체 패키지 빌드 (저장소 루트 기준)
colcon build --symlink-install

# 또는 캡스톤 메인 패키지만 선별 빌드
cd capstone_ws
colcon build --symlink-install --packages-select limo_capstone

# 3. 빌드 결과물 환경 오버레이
source install/setup.bash
```

> [!NOTE]
> `src/limo_ros2/limo_car/CMakeLists.txt`에서 저장소에 존재하지 않는 설치 디렉터리(`log`, `src`, `worlds`) 참조로 인해 발생하던 `ament_cmake_symlink_install_directory()` 오류를 수정한 후(`commit: 8c3d4f3`), 패키지 빌드가 성공적으로 수행됩니다. 빌드 상세 기록은 [docs/DEVELOPMENT_LOG.md](docs/DEVELOPMENT_LOG.md)에서 확인할 수 있습니다.

---

### 5.3 현재 실행 가능한 기능 및 명령

현재 저장소 상태에서 즉시 실행 가능한 주요 노드 및 런치 명령입니다.

#### 1) LIMO Base 하드웨어 구동 (온보드 환경)
LIMO onboard PC에서 하부 MCU와 시리얼 통신을 연결하고 오도메트리(`odom`) 및 좌표계(`base_link`)를 브로드캐스팅합니다.
```bash
ros2 launch limo_base limo_base.launch.py port_name:=ttyUSB1
```
*(장치 연결 상태에 따라 `/dev/ttyUSB0` 또는 `/dev/ttyUSB1` 포트 지정)*

#### 2) YDLiDAR T-mini Plus 라이다 구동
```bash
ros2 launch ydlidar_ros2_driver ydlidar.launch.py
```

#### 3) 원격 조향 제어 스크립트 실행
`geometry_msgs/msg/Twist` 표준 메시지 인터페이스를 통해 LIMO에 주행 속도 명령(`/cmd_vel`)을 발행합니다.
```bash
python3 scripts/remote_teleop_node.py
```

#### 4) 로봇 모델 RViz2 시각화
LIMO의 URDF/Xacro 형상 및 센서 좌표계를 3D로 시각화합니다.
```bash
# 차동 구동 모델 시각화
ros2 launch limo_description display_models_diff.launch.py

# 또는 Ackermann 조향 모델 시각화
ros2 launch limo_car display_ackermann.launch.py
```

#### 5) 정적 지도 기반 Navigation2 자율주행 실행
```bash
ros2 launch limo_capstone limo_nav2.launch.py \
  map:=$(ros2 pkg prefix limo_capstone)/share/limo_capstone/maps/demo_map.yaml \
  use_sim_time:=false
```

---

## 6. 자율주행 구현

### 6.1 LiDAR 데이터 확인
실제 LIMO에서 LiDAR 데이터를 정상적으로 수신할 수 있는지 확인합니다.

확인 항목:
- LiDAR Node (`ydlidar_ros2_driver_node`)
- LaserScan 데이터 수신 상태
- `/scan` Topic 발행 여부
- RViz2에서 LaserScan 표시: QoS Reliability Policy를 **`BEST_EFFORT`**로 설정하여 360도 스캔 포인트 정상 투영 확인
- 센서 좌표계 및 TF: `base_link → laser_link` 고정 TF 연결 확인

실제 하드웨어 연결 후 데이터를 확인하고 결과를 기록합니다.

---

### 6.2 TF 및 Odometry 확인
자율주행을 위해서는 센서와 로봇의 좌표계 관계가 정상적으로 구성되어야 합니다.

```text
map
 └── odom
      └── base_link
           ├── base_footprint (정적 TF 추가 패치)
           ├── laser_link
           ├── imu_link
           └── wheel_links (4개 휠)
```

확인 항목:
- TF Tree 무결성
- Odometry 발행 주기 (50 Hz)
- `base_link` 중심 좌표
- LiDAR Frame (`laser_link`)
- 좌표계 방향 및 회전 부호
- 주행 중 실시간 TF 변화

문제가 발생할 경우 TF Tree와 Topic 데이터를 기반으로 원인을 분석합니다.
본 프로젝트에서는 `base_link → base_footprint` 정적 TF를 추가하여 Nav2 Costmap 요구사항을 충족하였으며, `tf2_tools view_frames`를 실행하여 10개 프레임이 단일 트리로 정상 연결됨을 확인하였습니다 (`frames_2026-10-13_19.38.47.gv`). 상세 계측 데이터는 [docs/navigation.md](docs/navigation.md)를 참조합니다.

---

### 6.3 SLAM Mapping
LiDAR 센서 데이터를 이용하여 실제 실내 환경의 지도를 작성합니다.

#### 목표
- 복도
- 로비
- 주요 이동 구역

등의 환경을 지도에 반영합니다.

#### 구현 과정
```text
LiDAR
  ↓
LaserScan (/scan)
  ↓
SLAM Toolbox (online_async_launch.py)
  ↓
실시간 점유격자 지도 작성 (/map)
  ↓
Map 저장 (demo_map, MC_2F, MC_3F)
```

#### 검증 항목
- 지도 생성 상태
- 벽 및 복도 형태의 표현
- 지도 왜곡 여부
- 주행 중 위치 변화
- Loop Closure 상태

검증 완료된 지도 파일은 `demo_map.pgm`/`.yaml`, `domi_map.pgm`/`.yaml`, `MC_2F_10_07.pgm`/`.yaml`, `MC_3F_10_15.pgm`/`.yaml`이며, 상세 매핑 절차와 설정은 [docs/SLAM_MAPPING.md](docs/SLAM_MAPPING.md) 및 [docs/weekly-progress.md](docs/weekly-progress.md)에서 확인할 수 있습니다.

---

## 7. Localization 구현

### 7.1 AMCL
SLAM을 통해 작성한 정적 지도를 이용하여 로봇의 현재 위치를 추정합니다.

#### 구현 목표
- Map 기반 위치 추정
- 초기 위치 설정
- 주행 중 위치 추정
- 위치 추정 안정성 확인

#### 주요 확인 항목
- AMCL Pose
- Particle 분포 및 수렴도
- Map ↔ Odom 관계
- 초기 위치 설정: RViz2 상단 툴바의 **`2D Pose Estimate`** 도구를 통해 `/initialpose` 주입
- 주행 중 위치 오차

---

### 7.2 AMCL 파라미터 튜닝
실제 환경에서는 센서 특성 및 주행 환경에 따라 Localization 성능이 달라질 수 있으므로 파라미터를 단계적으로 조정합니다.

#### 주요 검토 항목
- Sensor Model (`laser_model_type: "likelihood_field"`)
- Motion Model (`robot_model_type: "nav2_amcl::DifferentialMotionModel"`)
- Particle 수 (`min_particles: 500`, `max_particles: 2000`)
- Update Threshold (`update_min_d: 0.25`, `update_min_a: 0.20`)
- Laser 관련 파라미터 (`laser_min_range: -1.0`, `laser_max_range: 100.0`)

파라미터 변경 전후의 위치 추정 상태를 비교하여 실제 환경에 적합한 설정을 찾습니다. 저장소의 실제 설정값 및 세부 튜닝 분석은 [docs/navigation.md](docs/navigation.md)에서 다룹니다.

---

## 8. Navigation2 구현

Navigation2를 이용하여 목표 위치까지의 경로 계획과 주행을 구현합니다.

전체 흐름은 다음과 같습니다.

```text
현재 위치 (AMCL)
   ↓
목표 위치 (2D Goal Pose)
   ↓
Global Planner (Navfn)
   ↓
Global Path
   ↓
Local Planner / Controller (DWB)
   ↓
Local Path & /cmd_vel
   ↓
LIMO 주행
```

---

### 8.1 Global Planning
Global Planner를 이용하여 지도상에서 현재 위치부터 목표 위치까지의 전체 경로를 생성합니다.

확인 항목:
- 경로 생성
- 장애물 반영
- 목표 위치 변경
- 경로 재계획

---

### 8.2 Local Planning 및 경로 추종
Global Path를 기반으로 실제 로봇이 주행할 Local Path 및 제어 명령을 생성합니다.

확인 항목:
- 경로 추종 안정성
- 회전 구간 조향 제어
- 복도 주행 속도 제어 (`max_vel_x: 0.26 m/s`)
- 목표 위치 접근 및 도달 판정 (`xy_goal_tolerance: 0.08 m`)
- 좁은 복도 통과를 위한 슬림한 Local Costmap 인플레이션 반경(`inflation_radius: 0.30 m`, `cost_scaling_factor: 7.0`) 적용

실제 LIMO의 주행 특성에 맞게 관련 파라미터를 조정합니다.

---

## 9. 동적 장애물 회피

본 프로젝트의 주요 개발 항목 중 하나입니다.
실내 환경에서는 사람과 같은 이동 물체가 경로에 진입할 수 있으므로 정적인 지도만 이용하는 자율주행으로는 충분하지 않습니다.

### 목표
- 동적 장애물 인식
- Costmap 반영
- 주행 중 경로 변경
- 충돌 방지
- 안전한 회피 주행

### 기본 처리 흐름
```text
LiDAR / Sensor
      ↓
장애물 정보 (전방 ROI 필터링)
      ↓
Local Costmap 갱신 / 긴급 제동
      ↓
경로 재계획
      ↓
회피 주행
```

실제 복도 및 로비 환경에서 장애물을 배치하고 주행 테스트를 수행하여 회피 성능을 확인합니다. 전방 감지 노드(`obstacle_detector.py`)와 중재 제어 노드(`obstacle_controller.py`)의 상세 아키텍처 설계는 [docs/roadmap.md](docs/roadmap.md)에 기술되어 있으며, 향후 패키지 구현 및 실차 검증을 진행할 예정입니다.

---

## 10. 자율 순찰

정해진 Waypoint를 순서대로 방문하는 자율 순찰 기능을 구현합니다.

```text
Waypoint 1
    ↓
Waypoint 2
    ↓
Waypoint 3
    ↓
Waypoint 4
    ↓
Waypoint 1
    ↺
```

### 개발 항목
- 순찰 지점 설정
- Waypoint 순서 관리
- ROS 2 Action 기반 목표 전달 (`nav2_msgs/action/NavigateToPose`)
- 목표 도착 확인
- 다음 Waypoint 전환
- 순찰 반복

실제 순찰 경로를 구성한 후 전체 경로의 주행 성공 여부를 검증합니다. 순찰 상태 머신 및 복구 로직 상세 설계는 [docs/roadmap.md](docs/roadmap.md)를 참조합니다.

---

## 11. 목적지 안내

사용자가 지정한 목적지까지 LIMO가 이동하는 안내 기능을 구현합니다.

### 기본 흐름
```text
사용자 호출 (/guide_request)
    ↓
목적지 선택
    ↓
Navigation Goal 전송
    ↓
경로 계획 및 동행 모니터링 (RGB-D 카메라)
    ↓
장애물 대응
    ↓
목적지 도착 알림 및 홈 대기 위치 복귀
```

안내 기능은 자율주행 시스템과 통합하여 실제 실내 환경에서 검증합니다. 세부 시나리오 및 서비스 설계는 [docs/roadmap.md](docs/roadmap.md)에 기술되어 있습니다.

---

## 12. 문제 해결 및 Troubleshooting

개발 과정에서 발생한 문제는 단순히 수정하는 데 그치지 않고, **문제 → 원인 분석 → 해결 → 검증**의 체계로 분석하여 기록합니다.

```text
문제 발생
   ↓
증상 확인 및 로그 수집
   ↓
관련 Topic / Node / TF 상태 확인
   ↓
원인 분석 및 가설 수립
   ↓
코드 또는 파라미터 최소 수정
   ↓
재실행 및 비교 검증
   ↓
개발 로그 기록 및 미해결 과제 분류
```

실제 하드웨어 구동, 통신, 빌드, TF, 자율 탐색 및 Nav2 구동 중 발생한 핵심 문제 7선(USB 시리얼 포트 브라운아웃, RViz `/scan` QoS 비호환, `setup.py` 디렉터리 복사 오류, `base_footprint` TF 누락, Explore Lite 목표 가상 무게중심 조기 정지 무한 루프, Map Server Lifecycle/QoS 지도 수신 실패, 시뮬레이션-실차 정합 차이)의 분석과 해결책은 **[docs/troubleshooting.md](docs/troubleshooting.md)** 및 **[docs/DEVELOPMENT_LOG.md](docs/DEVELOPMENT_LOG.md)**에서 확인할 수 있습니다.

---

## 13. 실험 및 검증

구현한 기능은 실제 주행 환경에서 단계적으로 검증합니다.

### 평가 항목

#### Localization
- 위치 추정 안정성
- 초기 위치 설정
- 주행 중 위치 오차

#### Navigation
- 목표 위치 도착 여부
- 경로 추종 성능
- 주행 시간
- 경로 이탈 여부

#### 장애물 회피
- 장애물 인식 여부
- 회피 성공 여부
- 충돌 여부
- 경로 재계획 여부

#### Patrol
- Waypoint 도착 성공률
- 전체 순찰 성공 여부
- 반복 순찰 가능 여부

#### Guide
- 목적지 도착 여부
- 안내 과정의 안정성
- 장애물 발생 시 대응 여부

실험 결과는 개발이 진행됨에 따라 측정값과 테스트 조건을 지속적으로 갱신합니다.

---

## 14. 개발 기록

프로젝트의 실제 개발 과정과 문제 해결 과정은 별도의 상세 문서에서 관리합니다.

- **[docs/weekly-progress.md](docs/weekly-progress.md)**: 1~8주차 주차별 개발 목표, 개인 수행 작업, 문제 해결, 실차 검증 결과 종합 기록
- **[docs/troubleshooting.md](docs/troubleshooting.md)**: 7단계 표준 형식으로 작성된 핵심 트러블슈팅 7선 가이드
- **[docs/navigation.md](docs/navigation.md)**: Nav2 아키텍처, 10개 TF 프레임 무결성 계측, AMCL 파라미터, Costmap 분석 및 실행 가이드
- **[docs/roadmap.md](docs/roadmap.md)**: 4단계 구현 상태 매트릭스 및 동적 장애물 회피, 순찰, 안내 기능 상세 설계
- **[docs/DEVELOPMENT_LOG.md](docs/DEVELOPMENT_LOG.md)**: 실제 하드웨어 센서 검증, SLAM 매핑, Nav2 연동, 오류 분석 및 현장 검증 기록
- **[docs/PROJECT_PLAN.md](docs/PROJECT_PLAN.md)**: 전체 개발 단계(Phase 1~9), 팀원별 역할 분담 및 평가 기준 정의
- **[docs/NAV2_TROUBLESHOOTING.md](docs/NAV2_TROUBLESHOOTING.md)**: Nav2 자율주행 심층 트러블슈팅 및 가이드
- **[docs/SLAM_MAPPING.md](docs/SLAM_MAPPING.md)**: 실내 SLAM 매핑 절차 및 지도 작성 검증 기록
- **[docs/PHASE12_SIMULATION_VALIDATION.md](docs/PHASE12_SIMULATION_VALIDATION.md)**: Gazebo 시뮬레이션 환경 검증 기록

---

## 15. 팀 역할

| 담당 | 주요 업무 |
|---|---|
| 정진우 | 자율주행 및 동적 장애물 회피 총괄 (팀장) |
| 배민혁 | 실내 SLAM 맵핑 |
| 한동훈 | 시스템 구축 및 센서 융합 |
| 김정민 | 순찰·안내 애플리케이션 및 검증 |

### 정진우 — 자율주행 및 동적 장애물 회피 (팀장)
- Nav2 기반 AMCL 위치 추정 및 파라미터 튜닝
- 복도 및 로비 주행을 위한 Global / Local Path Planning
- TF 좌표계 무결성 분석 및 `base_footprint` 정적 TF 브로드캐스터 구현
- Explore Lite 프론티어 선정 알고리즘 수정 (`frontier->middle`)
- 동적 장애물 인식 및 회피 알고리즘 구조 설계
- 실제 주행 환경에서 주행 파라미터 최적화

### 배민혁 — 실내 SLAM 맵핑
- 실내 2D/3D 점유격자지도 작성 (`demo_map`, `MC_2F_10_07`, `MC_3F_10_15`)
- 센서 노이즈 및 맵핑 오차 보정
- 센서 데이터 정합성 검증

### 한동훈 — 시스템 구축 및 센서 융합
- ROS 2 개발환경 구축 및 관리 (Ubuntu 22.04 LTS / ROS 2 Humble)
- LIMO Pro 드라이버 및 하드웨어 제어 (`port_name:=ttyUSB1`)
- LiDAR / Depth Camera / IMU 데이터 수집
- 센서 좌표계 및 데이터 정합성 확인
- 시스템 통합

### 김정민 — 순찰·안내 애플리케이션 및 검증
- ROS 2 Action 기반 Waypoint 순찰 로직 설계
- 사용자 호출 기반 안내 기능 시나리오 수립
- 서비스 시나리오 필드 테스트
- 테스트 결과 데이터 수집 및 기능 검증

위 역할은 캡스톤디자인 팀별 계획서의 역할 분담을 기준으로 작성했습니다.

---

## 16. 향후 개발 방향

프로젝트는 다음 순서로 통합합니다.

```text
LIMO 기본 동작 (완료)
      ↓
센서 및 TF 확인 (완료)
      ↓
SLAM Mapping (완료)
      ↓
AMCL Localization (완료)
      ↓
Navigation2 (완료)
      ↓
경로 추종 (완료)
      ↓
자율 탐색 실차 완주 (진행 중)
      ↓
동적 장애물 회피 (계획)
      ↓
자율 순찰 (계획)
      ↓
목적지 안내 (계획)
      ↓
통합 테스트 및 1시간 연속 주행 평가 (계획)
      ↓
파라미터 최적화 (계획)
      ↓
최종 시연 및 평가 (계획)
```

최종적으로는 실제 실내 환경에서 LIMO가 안정적으로 자율주행하면서 순찰 및 안내 서비스를 수행할 수 있도록 시스템을 통합하고 검증합니다.

---

## 17. 참고자료 및 출처

본 프로젝트의 개발 및 구현 과정에서는 다음 자료를 우선적으로 참고합니다.

1. **WeGo, 「Limo_ROS2 교재」**
2. **AgileX Robotics, LIMO 공식 자료**
3. **ROS 2 공식 문서**
4. **Navigation2 공식 문서**
5. **SLAM Toolbox 공식 문서**
6. **2026-2 캡스톤디자인 팀별계획서 및 진도보고서 (4조)**
7. 프로젝트 개발 과정에서 확인한 관련 기술 문서 및 커뮤니티 자료

---

## 18. 프로젝트 산출물

최종적으로 다음 결과물을 구축하고 정리합니다.

- ROS 2 기반 LIMO 자율주행 소스 코드
- 커스텀 Node 및 Launch 파일
- Navigation2 설정 및 Parameter 파일
- 실내 지도 데이터 (`demo_map`, `MC_2F_10_07`, `MC_3F_10_15`)
- 자율 순찰 기능 (Action 노드)
- 목적지 안내 기능 (Service 노드)
- 동적 장애물 대응 기능
- 주행 테스트 및 성능 평가 결과
- 캡스톤디자인 최종 결과 보고서
- 시연 영상 및 발표 자료
