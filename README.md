# LIMO를 이용한 실내 자율 순찰 및 안내 (LIMO Indoor Autonomous Patrol & Guidance)

> **2026학년도 2학기 캡스톤디자인 1 (4조)**<br>
> **개인 개발 저장소 및 엔지니어링 포트폴리오**<br>
> **주요 개발 범위:** 자율주행(Navigation2), TF 좌표계, Costmap, 위치추정(AMCL) 및 동적 장애물 회피

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble_Hawksbill-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04.5_LTS-E95420?logo=ubuntu)](https://ubuntu.com/)
[![Platform](https://img.shields.io/badge/Platform-AgileX_LIMO_Pro-orange)]()
[![Navigation](https://img.shields.io/badge/Stack-Navigation2_%28Nav2%29-007ACC)](https://navigation.ros.org/)

**AgileX LIMO Pro**를 활용하여 실내 환경에서 자율주행, 순찰 및 목적지 안내가 가능한 ROS 2 기반 로봇 시스템을 개발합니다.

---

## 최근 작업과 통합 기준선 (2026-11-08 확정: LIMO Pro 통합 자동 맵핑 시뮬레이션 1차 완료)

2026년 11월 8일, `capstone_ws`를 메인 자립형 워크스페이스로 확정하고 시뮬레이션 통합 자동 맵핑 파이프라인의 1차 기능 동등성 및 안전 회귀 검증을 완료하였습니다.

- **워크스페이스 역할 분담:**
  - **`capstone_ws` (메인 워크스페이스):** 정식 ROS 2 패키지(`limo_capstone`, `limo_depth_obstacles`), 정식 Launch 2종(`sim`, `real`), 설정 YAML, SDF/URDF 모델을 포함합니다. 단, **외부 의존성이 필요합니다**: 시뮬레이션·실차 launch 모두 AgileX `limo_ros2`의 `limo_description`을, 실차는 추가로 `limo_base`, `ydlidar_ros2_driver`(시스템 라이브러리 `ydlidar_sdk` 필요), `astra_camera`(깊이 카메라 사용 시)를 요구하며 이들은 `capstone_ws/src`에 포함되어 있지 않습니다(출처: `docs/third_party.repos`). 이 의존성이 없는 환경에서는 launch가 `PackageNotFoundError`로 실패합니다.
  - **`experiments/limo_integrated_auto_mapping_20261108/` (실험 아카이브):** 자동 회귀 검증 스크립트, 장시간 탐색 벤치마크 로그, 고장 주입 시험 JSON, 공식 rosbag2 아카이브 및 기술 보고서 5종을 보존합니다.
- **핵심 통합 서브시스템:**
  - **F4/F5_improved 선택형 상태 추정:** 2D LiDAR ICP 오도메트리(F4, 10.0Hz) 및 ICP + IMU 요 각속도 EKF 융합 오도메트리(F5_improved, 50.0Hz).
  - **단일 소유권 TF 아키텍처:** SLAM Toolbox(`map -> odom`) 및 `odom_tf_relay_node`(`odom -> base_footprint`)의 단일 브로드캐스터 체인 확립.
  - **자율 탐색 및 3D Costmap:** explore_lite 프론티어 탐색, Nav2 RPP 제어기, DaBai Depth 높이 윈도잉([0.025, 0.35]m) 3D 장애물 레이어 연동.
  - **Stall Detector 안전 명령 게이트:** 후진 금지(`enable_auto_reverse:=false`), 0.38초 센서 단절 페일세이프, 모터 구동 잠금(`allow_motor_drive:=false`).
  - **rosbag2 기록 프로파일:** 12개 핵심 토픽 기록 및 배속 재생 검증 (경량 프로파일 및 상세 진단 프로파일 분리).

> [!IMPORTANT]
> **검증 수준 구분 (2026-11-11).** 위 "1차 완료"는 **시뮬레이션** 기준이다. 실차(LIMO)에서는 모터 잠금 launch, 벤치 MCU 정지 시간 측정, 느린 감독 주행 몇 건만 확인했고 **실차 자동 맵핑의 정상 동작은 미검증**이다. SLAM 지도 갱신 문제의 해결안(스캔 정규화)은 저장된 실차 bag으로 **오프라인 검증**했을 뿐 실차 launch에 적용하지 않았다. F4 점대선 ICP는 실험 제안이고, `explore_relaxed`는 시뮬레이션 평가용 실험 변형이며 기본 탐색기가 아니다. 항목별 근거와 한계는 [실차·시뮬레이션 검증 현황](docs/REAL_ROBOT_STATUS.md), 시뮬레이션/실차 설정 차이는 [설정 분리](docs/CONFIG_SIM_VS_REAL.md)를 본다.

| 확인할 핵심 자료 | 위치 |
|---|---|
| **정식 통합 운영 가이드** | [운영 가이드](docs/INTEGRATED_AUTO_MAPPING_OPERATIONS_GUIDE.md) |
| **최종 회귀 검증 보고서** | [최종 회귀 보고서](experiments/limo_integrated_auto_mapping_20261108/docs/LIMO_PRO_WORKSPACE_FINAL_REGRESSION_REPORT.md) |
| **최종 회귀 결과 데이터 (JSON)** | [회귀 결과 JSON](experiments/limo_integrated_auto_mapping_20261108/evaluation/results/capstone_ws_final_regression_audit_20261108.json) |
| **장기 맵핑 및 우회 벤치마크** | [확장 검증 보고서](experiments/limo_integrated_auto_mapping_20261108/docs/EXTENDED_INTEGRATION_VERIFICATION_REPORT.md) |
| **실차·시뮬레이션 검증 현황** | [검증 현황](docs/REAL_ROBOT_STATUS.md) |
| **시뮬레이션/실차 설정 차이** | [설정 분리](docs/CONFIG_SIM_VS_REAL.md) |
| **기존 개발 상태 및 로드맵** | [개발 상태](docs/roadmap.md) |

> [!NOTE]
> **이전 이력 보존:** 2026-10-25 정리된 MC관 수동/자동 지도 및 복합 이동 장애물 시뮬레이션 기록은 [docs/SIMULATION_VERIFICATION_REPORT_2026-10-24.md](docs/SIMULATION_VERIFICATION_REPORT_2026-10-24.md) 및 [docs/experiments/2026-10-24/README.md](docs/experiments/2026-10-24/README.md)에 온전히 보존되어 있습니다.

---

## 중앙 관제와 Open-RMF 연동 (`fleet_control/`)

노트북의 중앙 관제 서버(웹 대시보드, REST API, SQLite), LIMO의 Robot Agent, Open-RMF Fleet Adapter를 Tailscale 사설망으로 연결하는 구성입니다. 자세한 내용은 [fleet_control/README.md](fleet_control/README.md), [API](fleet_control/docs/API.md), [검증 요약](fleet_control/docs/VALIDATION.md), [Open-RMF 설계](fleet_control/rmf_adapter/docs/RMF.md)를 보세요.

| 구분 | 현재 상태 |
|---|---|
| 구현 완료 | 대시보드, 서버, Robot Agent, 임무 생성·취소·상태, 인증·역할, 제어권 lease, 운영자 복구, Open-RMF Fleet Adapter |
| 모의 환경 검증 | RMF → Mission Manager → 모의 로봇 임무 수행·취소, 웹/RMF 소유권 충돌, 통신 장애·재연결, 복구 |
| 실차 검증 | 모터 잠금 상태에서 Tailscale 경유 **상태 수신**만(임무·이동 없음) |
| 미검증 | Gazebo·Open-RMF 통합, Open-RMF 실차 이동, 다중 로봇, 장시간 운용, RMF 고아 작업 복구, 최신 서버·Agent의 운영 배포 |

시뮬레이션 결과를 실차 검증으로 읽지 마세요. `tests/run_checks.py`는 한 번 원인 미상의 실패(42/46)가 있었습니다. 한계는 위 문서에 적혀 있습니다.

`fleet_control/`에는 현재 별도의 오픈소스 라이선스가 부여되지 않았습니다([안내](fleet_control/README.md#라이선스)).

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

전체 시스템은 다음과 같은 통합 흐름으로 구성됩니다.

```text
                                [AgileX LIMO Pro / Gazebo Simulation]
                                      │                    │
                   ┌──────────────────┘                    └──────────────────┐
                   ▼                                                          ▼
             [/scan, 10Hz]                                             [/camera/depth/points]
                   │                                                          │
                   ▼                                                          ▼
    ┌──────────────────────────────┐                          ┌──────────────────────────────┐
    │  lidar_scan_matcher_odom     │                          │  depth_obstacle_filter_node  │
    │  - F4: ICP 오도메트리 (10Hz) │                          │  - Z:[0.025, 0.35]m 윈도잉  │
    └──────────────┬───────────────┘                          │  - 워치독 타임아웃 0.38초     │
                   │                                          └──────────────┬───────────────┘
           ┌───────┴───────┐                                                 │
           ▼               ▼ (F5_improved 선택 시)                            │
     [/odometry/lidar]   [/imu, 50Hz]                                        │
           │               │                                                 │
           │               ▼                                                 ▼
           │      ┌──────────────────────┐                          [/depth_obstacles/points]
           │      │    ekf_node (F5)     │                                   │
           │      │ - EKF 융합 오도 (50Hz)│                                   │
           │      └──────────┬───────────┘                                   │
           ▼                 ▼                                               │
    ┌──────────────────────────────┐                                         │
    │      odom_tf_relay_node      │                                         │
    │  - odom -> base_footprint TF │                                         │
    │  - /odom_nav2 브로드캐스트    │                                         │
    └──────────────┬───────────────┘                                         │
                   │                                                         │
                   ├───────────────────────────────────┐                     │
                   ▼                                   ▼                     ▼
    ┌──────────────────────────────┐    ┌──────────────────────────────────────────────┐
    │     async_slam_toolbox       │    │                 Nav2 Stack                   │
    │  - map -> odom TF 단일 소유  │    │  - Global Planner (Navfn)                    │
    │  - 실시간 점유 격자 지도 /map │    │  - Local Controller (RPP)                    │
    └──────────────┬───────────────┘    │  - Costmap: 2D LiDAR + 3D Depth Obstacle     │
                   │                    └──────────────────────┬───────────────────────┘
                   ▼                                           │
    ┌──────────────────────────────┐                           ▼
    │         explore_lite         │                     [/cmd_vel_raw]
    │  - 프론티어 자동 탐색 목표 생성│                           │
    └──────────────────────────────┘                           ▼
                                        ┌──────────────────────────────────────────────┐
                                        │             stall_detector_node              │
                                        │  - 안전 명령 게이트 (/cmd_vel_raw -> /cmd_vel)│
                                        │  - 후진 금지 (enable_auto_reverse:=false)    │
                                        │  - 센서 타임아웃 0.38초 / 슬립 정체 감지     │
                                        │  - 모터 잠금 (allow_motor_drive:=false)      │
                                        └──────────────────────┬───────────────────────┘
                                                               │
                                         ┌─────────────────────┴─────────────────────┐
                                         ▼ (allow_motor_drive=true)                  ▼ (false)
                                   [/cmd_vel] (모터 드라이버)                   [/cmd_vel_locked] (안전 모의)
```

---

## 4. 저장소 구조

저장소의 주요 디렉터리 구성은 다음과 같습니다.

```text
├── docs/                                  # 프로젝트 기술 문서 및 개발 기록
│   ├── INTEGRATED_AUTO_MAPPING_OPERATIONS_GUIDE.md # [신규] 통합 자동 맵핑 정식 운영 가이드
│   ├── troubleshooting.md                 # 핵심 트러블슈팅 7선 가이드
│   ├── navigation.md                      # Nav2 아키텍처 및 주행 파이프라인 분석
│   ├── roadmap.md                         # 구현 상태 매트릭스 및 향후 로드맵
│   ├── DEVELOPMENT_LOG.md                 # 시스템 점검 이력 및 일자별 개발 기록
│   ├── NAV2_TROUBLESHOOTING.md            # Nav2 자율주행 검증 및 문제 분석
│   ├── SLAM_MAPPING.md                    # SLAM 및 지도 작성 검증
│   └── PHASE12_SIMULATION_VALIDATION.md   # 시뮬레이션 환경 검증 기록
├── capstone_ws/                           # 캡스톤 메인 ROS 2 워크스페이스 (정식 빌드 기준선)
│   ├── README.md                          # capstone_ws 빠른 시작 안내서
│   └── src/
│       ├── limo_depth_obstacles/          # [신규] 3D DaBai Depth 카메라 장애물 필터 패키지
│       │   ├── limo_depth_obstacles/      # depth_obstacle_filter_node, filtering.py
│       │   ├── package.xml
│       │   └── setup.py
│       ├── limo_capstone/                 # 캡스톤 내비게이션 통합 패키지 (Launch, Config, Models)
│       │   ├── config/                    # EKF, explore, nav2, slam, rosbag 파라미터 YAML 5종
│       │   ├── launch/                    # integrated sim / real 런치 및 레거시 런치 6종
│       │   ├── limo_capstone/             # F4 오도, TF 중재, 안전 게이트, 모니터 노드 8종
│       │   ├── maps/                      # 점유격자지도 (demo_map, MC_2F, MC_3F 등)
│       │   ├── models/                    # SDF 평가 메쉬 모델 (lounge_chair_sled, table 등)
│       │   ├── urdf/                      # limo_ackermann_fortress.urdf
│       │   ├── worlds/                    # lounge_evaluation_world.sdf
│       │   └── setup.py                   # 정식 배포 엔트리포인트 8종 정의
│       ├── (m-explore-ros2, robot_localization, ydlidar_ros2_driver, wego*, libuvc, magic_enum  # 외부 소스: 출처는 docs/EXTERNAL_REFERENCES.md)
├── experiments/                           # 실험·회귀 검증 기록 (목록과 검증 수준: experiments/README.md)
│   └── limo_integrated_auto_mapping_20261108/
│       ├── docs/                          # 회귀 감사 보고서 및 장기 맵핑 보고서 5종
│       │   └── LIMO_PRO_WORKSPACE_FINAL_REGRESSION_REPORT.md
│       ├── evaluation/
│       │   ├── scripts/                   # 전 부문 자동 회귀 감사 스크립트 (execute_full_regression_audit.py)
│       │   └── results/                   # 감사 결과 JSON 및 공식 검증 rosbag2 아카이브
│       └── videos/                        # 시뮬레이션 자율 탐색 및 우회 녹화 MP4
├── scripts/
│   └── remote_teleop_node.py              # 원격 조향 제어 퍼블리셔 노드
└── (src/limo_ros2: AgileX 원본은 저장소에 포함하지 않음 — docs/third_party.repos 참고)
```

### `docs`
- `INTEGRATED_AUTO_MAPPING_OPERATIONS_GUIDE.md`: F4/F5 빌드 및 실행, RViz2 설정, rosbag2 프로파일, 안전 게이트 언래치, 정상 종료 및 실물 사전 체크리스트 수록.
- `troubleshooting.md`: 하드웨어 전원 브라운아웃, QoS 불일치, `setup.py` 빌드 에러, TF 누락, 탐색 무한 루프, Lifecycle 지도 미수신 등 핵심 오류 7선에 대한 분석과 해결책을 표준 형식으로 수록.
- `navigation.md`: Nav2 아키텍처, 10개 TF 프레임 무결성 실측치, AMCL 및 Costmap 파라미터, 실행 가이드를 상세히 분석.

### `capstone_ws` 및 `experiments`의 역할 구분
- **`capstone_ws`:** 런타임 구동과 정식 코드베이스 배포를 담당하며, `experiments/`에 대한 의존성 없이 단독 빌드 및 실행됩니다.
- **`experiments/`:** 회귀 감사 스크립트, 장시간 벤치마크 로그, 공식 rosbag2 아카이브, 고장 주입 시험 데이터를 보존하여 검증 재현성을 보장합니다.

---

## 5. 구현 과정 및 빌드 가이드

본 프로젝트에서는 기능을 한 번에 통합하지 않고, **기본 소스 확인 → 센서 및 TF 확인 → 지도 작성 → 위치 추정 → Navigation2 → 장애물 대응 → 순찰 및 안내**의 순서로 단계적으로 개발합니다.

---

### 5.1 LIMO ROS 2 기본 소스 및 구현 상태

프로젝트는 AgileX에서 제공하는 LIMO ROS 2 소스를 기반으로 구성하며, 현재 저장소의 구현 및 준비 상태는 다음과 같습니다.

| 구분 | 대상 | 현재 상태 | 상세 설명 |
|---|---|:---:|---|
| **기본 패키지** | LIMO 드라이버·모델·메시지 | 소스 보유 | 원제공 소스와 프로젝트 수정 사항은 Git 이력으로 구분 |
| **상태 추정 (F4)** | 2D LiDAR ICP 오도메트리 | **통합 완료** | 시뮬레이션 10.0 Hz 실측 검증. 고유 TF는 `odom_tf_relay_node`를 통해 중재 |
| **상태 추정 (F5)** | ICP + IMU EKF 융합 | **통합 완료** | 시뮬레이션 50.0 Hz 실측 검증. 슬립 방지를 위해 휠 오도는 EKF에서 배제 |
| **SLAM 지도 작성** | SLAM Toolbox 온라인 비동기 | 시뮬레이션 통합 완료 / **실차 미해결** | 시뮬레이션: `map -> odom` TF 단일 소유, 점유 격자 확장 검증. 실차: LiDAR 스캔 점 수 불일치로 지도가 갱신되지 않는 문제 확인, 정규화 해결안은 오프라인 검증만 됐고 실차 미적용 (`docs/REAL_ROBOT_STATUS.md`) |
| **자율 탐색 주행** | Nav2 RPP + explore_lite | 시뮬레이션 통합 완료 / **실차 맵핑 미검증** | 시뮬레이션: 프론티어 자동 감지 및 NavigateToPose 연동 탐색 완주. 실차: 감독 하의 짧은 시행만 있고 정상 맵핑은 확인되지 않음 |
| **3D 장애물 필터** | DaBai Depth 높이 윈도잉 | **통합 완료** | `limo_depth_obstacles` 패키지. 지상 [0.025, 0.35]m 윈도잉, 0.38초 워치독 |
| **안전 명령 게이트** | Stall Detector 노드 | **통합 완료** | 후진 차단(`enable_auto_reverse:=false`), 0.38초 센서 단절 제동, 휠 헛돎 감지 |
| **모터 잠금 격리** | 실물 Launch 안전 격리 | **통합 완료** | `allow_motor_drive:=false` 시 `/cmd_vel` 0건 누출, `/cmd_vel_locked` 전량 라우팅 |
| **실험 데이터 기록** | rosbag2 프로파일 | **통합 완료** | 12개 토픽 기록 및 재생성 검증 (경량 프로파일 및 상세 진단 프로파일 분리) |
| **순찰·안내 응용** | Waypoint 순찰 및 안내 | 계획 | 통합 맵핑 및 안전 기준선 확정 후 후속 단계로 진행 예정 |

---

### 5.2 빌드 환경 및 방법

#### 개발 환경
- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **빌드 도구**: `colcon`

#### 워크스페이스 구조 및 빌드 안내
캡스톤 프로젝트의 통합 기능(`limo_capstone`, `limo_depth_obstacles`)은 `capstone_ws/`에 정식 이식되어 있으며, 외부 의존성 없이 자립적으로 빌드됩니다.

```bash
# 1. 캡스톤 메인 워크스페이스로 이동
cd ~/school/2026-2/Capstone1_limo/capstone_ws

# 2. ROS 2 Humble 기본 환경 로드
source /opt/ros/humble/setup.bash

# 3. 통합 패키지 선별 빌드 (심볼릭 링크 설치)
colcon build --symlink-install --packages-select limo_depth_obstacles limo_capstone

# 4. 빌드 결과물 환경 오버레이 및 EKF 라이브러리 경로 설정
source install/setup.bash
export LD_LIBRARY_PATH=$PWD/install/robot_localization/lib:/opt/ros/humble/lib:$LD_LIBRARY_PATH
```

---

### 5.3 주요 실행 명령과 검증 수준

시뮬레이션 명령은 시뮬레이션에서 실행·재현을 확인했다. 실차 명령은 모터 잠금 상태의 토픽 흐름 점검이 목적이며, 실차 자동 맵핑이 정상 동작한다는 뜻이 아니다(`docs/REAL_ROBOT_STATUS.md`).

#### 1) 시뮬레이션 통합 자동 맵핑 실행 (Gazebo Fortress)
실내 라운지 평가 월드에서 F4 또는 F5_improved 기반으로 explore_lite 자율 탐색 및 맵핑을 구동합니다.
```bash
# A. F4 모드 (2D LiDAR ICP 기반 탐색)
ros2 launch limo_capstone limo_integrated_auto_mapping_sim.launch.py \
  use_sim_time:=true estimator_type:=f4 start_explore:=true depth_mode:=required

# B. F5_improved 모드 (LiDAR ICP + IMU EKF 융합 기반 탐색)
ros2 launch limo_capstone limo_integrated_auto_mapping_sim.launch.py \
  use_sim_time:=true estimator_type:=f5_improved start_explore:=true depth_mode:=required
```

#### 2) 실물 로봇 모터 잠금 실행
실물 센서와 토픽 흐름을 점검하되 `/cmd_vel`을 소프트웨어로 막습니다. 모든 Nav2 명령은 `/cmd_vel_raw`로 나가 안전 게이트 하나를 거치며, `allow_motor_drive`가 문자열 `true`일 때만 `/cmd_vel`로 나갑니다. 실차에서는 이 launch로 구동 시험도 했으나(사용자 입회, 느린 프로파일) 범위는 `docs/REAL_ROBOT_STATUS.md`에 한정되어 있습니다.
```bash
ros2 launch limo_capstone limo_integrated_auto_mapping_real.launch.py \
  allow_motor_drive:=false estimator_type:=f4
```
*(※ 잠금 모드에서 `/cmd_vel` 발행자는 0개이며 명령은 `/cmd_vel_locked`로 격리됩니다. 실차에서는 LiDAR가 시작 직후와 정상 상태에서 점 수가 달라 slam_toolbox가 스캔을 거부해 지도가 갱신되지 않는 문제가 확인되었고, 정규화 해결안은 아직 실차 launch에 적용되지 않았습니다.)*

> [!CAUTION]
> **레거시 launch 사용 금지:** `limo_nav2.launch.py`, `limo_auto_mapping.launch.py`, `limo_nav2_sim.launch.py`, `limo_auto_mapping_sim.launch.py`, `limo_mc3f_sim.launch.py` 등 (Nav2를 띄우는 비통합 launch) `limo_integrated_auto_mapping_*`가 아닌 launch에는 현재 안전 게이트(`/cmd_vel_raw` → `stall_detector_node`)가 적용되어 있지 않습니다. Nav2 출력이 `/cmd_vel`로 직접 나갈 수 있으므로 **실차 자율 주행에 사용하지 마십시오.** (게이트 적용은 구현하지 않았고 경고만 추가한 상태입니다.)

#### 3) RViz2 시각화 설정
별도 터미널에서 RViz2를 실행하여 점유 격자 지도 및 센서를 확인합니다.
```bash
source /opt/ros/humble/setup.bash && rviz2
```
- **Fixed Frame:** `map` 설정
- **Map:** Topic `/map` (Durability: `Transient Local`, Reliability: `Reliable`)
- **LiDAR:** Topic `/scan` (Reliability: `Best Effort`)
- **3D Depth:** Topic `/depth_obstacles/points` (Reliability: `Reliable`)

#### 4) 안전 래치 확인 및 수동 리셋
정체(STALL) 래치는 아래 서비스로 운영자가 해제해야 합니다. **실차 launch(`require_operator_unlatch=False`)에서 센서 단절 FAILSAFE는 필요한 입력이 모두 다시 정상이 되고 Nav2 명령이 0 근처(선·각속도 0.02 미만)일 때 운영자 확인 없이 자동으로 NORMAL로 복귀합니다**(코드상 동작이며 실차에서 안전성이 검증되지 않았음). 시뮬레이션 launch는 `require_operator_unlatch=True`라 센서 FAILSAFE도 서비스로 해제합니다.
```bash
# 안전 게이트 텔레메트리 모니터링
ros2 topic echo /stall_detector/status_json

# 안전 래치 수동 해제 (Operator Unlatch)
ros2 service call /stall_detector/reset_latch std_srvs/srv/Trigger
```

#### 5) rosbag2 기록 프로파일 및 실측 용량
- **F4 기본 기록 프로파일 (실측 기준 환산 ~7.0 MB/min):**
  ```bash
  ros2 bag record -o /tmp/limo_bag_f4 \
    /scan /odom_nav2 /tf /tf_static /map /cmd_vel /cmd_vel_raw /stall_detector/status_json
  ```
- **F5_improved 상세 진단 프로파일 (실측 기준 환산 ~8.7~9.2 MB/min, IMU 50Hz 및 EKF 포함):**
  ```bash
  ros2 bag record -o /tmp/limo_bag_f5_diag \
    /scan /odom /odometry/lidar_odom /odometry/f5_improved /odom_nav2 /imu \
    /tf /tf_static /map /depth_obstacles/points /cmd_vel_raw /cmd_vel \
    /stall_detector/status_json
  ```
  *(※ 과거 언급된 0.8 MB/min은 고주기 TF를 배제한 미검증 이론 추정치이며, 실제 21.5초 실측 데이터 기준 F4는 약 7.0 MB/min, F5는 약 8.7~9.2 MB/min입니다.)*
- **독립 재생 검증:**
  ```bash
  ROS_DOMAIN_ID=42 ros2 bag play /tmp/limo_bag_f5_diag --rate 1.0
  ```

#### 6) 정상 종료 시퀀스 및 비상 정지 구분
- **소프트웨어 정상 종료(Graceful Shutdown):** Launch가 실행 중인 터미널에서 **`Ctrl + C`**를 입력하면 Launch 시스템이 SIGINT를 모든 자식 노드(`ign gazebo`, `nav2`, `slam_toolbox`, `explore_node` 등)에 전파하여 3~5초 내에 정상 종료합니다.
- **비상 정지(E-Stop) 아님:** `Ctrl + C`는 소프트웨어 인터럽트 종료 방법일 뿐, 온보드 PC의 락업이나 통신 단절 시 모터를 즉시 멈추는 하드웨어 비상 정지 장치가 아닙니다.
- **안전 수칙:** 정식 운영 가이드에서는 시스템 전역 프로세스를 무차별 파괴할 수 있는 `pkill -9` 사용을 배제합니다.

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


저장 지도에는 MC관 2층 테라스 수동 지도와 3층 자동 지도가 포함됩니다. 지도 파일과 좌표 설정은 `capstone_ws/src/limo_capstone/maps/`, 작성 절차는 [SLAM 문서](docs/SLAM_MAPPING.md)에서 확인할 수 있습니다.

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
- 목표 위치 접근 및 도달 판정 (`xy_goal_tolerance: 0.25 m`, 현재 로봇용 YAML 설정값)
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

구현한 기능은 가상 시뮬레이션 환경 및 실제 주행 환경에서 단계적으로 검증합니다.

### 13.1 메인 워크스페이스(`capstone_ws`) 통합 자동 맵핑 최종 회귀 검증 (2026-11-08)

실험실(`experiments/limo_integrated_auto_mapping_20261108/`)에서 사전 검증된 통합 자동 맵핑 기능이 메인 워크스페이스(`capstone_ws`)의 정식 패키지(`limo_capstone`, `limo_depth_obstacles`)로 결함 없이 이식되었는지 전 부문 회귀 검사를 수행하였으며, 7개 감사 부문 모두 **100% PASS**로 판정되었습니다.

#### 1) 전 부문 감사 판정 매트릭스

| 감사 영역 | 검증 항목 | 합격 기준 | 실제 측정값 | 최종 판정 |
| :--- | :--- | :--- | :--- | :---: |
| **Audit 1: 패키지 이식 정합성** | 엔트리포인트 9종, share 디렉터리, experiments 참조 0건, 레거시 파일 보존 | 결함/오류 0건, 레거시 보존 | 엔트리포인트 9종 정상 등록, share 접근 정상, experiments 참조 0건, 원본 6종 100% 보존 | **PASS** |
| **Audit 2: 실물 구동 안전 게이트** | `allow_motor_drive:=false` 모터 잠금, Nav2 바이패스 차단 | `/cmd_vel` 0건, `/cmd_vel_locked` 라우팅 | `/cmd_vel` 0건 (누출 제로), `/cmd_vel_locked` 160건 전량 라우팅 | **PASS** (※ 아래 주의) |
| **Audit 3: F4 통합 자동 탐색** | 단일 TF 트리, 오도메트리 주기, 지도 확장, rosbag2 기록/재생 | TF 100%, Freq $\ge 8\text{Hz}$, Map 확장, 재생 가능 | TF 단일 소유, Freq 10.0 Hz, Map 격자 확장, 2.509 MB 재생 성공 | **PASS** |
| **Audit 4: F5 통합 자동 탐색** | 단일 TF 트리, EKF 오도메트리 주기, 지도 확장, rosbag2 기록/재생 | TF 100%, Freq $\ge 40\text{Hz}$, Map 확장, 재생 가능 | TF 단일 소유, Freq 50.0 Hz, Map 격자 확장, 3.267 MB 재생 성공 | **PASS** |
| **Audit 5A: Depth 차단 (상태 반응성)** | 인공 `DEPTH_STALE` JSON 상태 주입 (Test A) | 반응 $\le 0.05\text{s}$, 정지거리 $< 0.20\text{m}$ | 감지 0.004s, 차단 0.003s, 총변위 0.100m (고장주입~정지), 완전정지 6.455s | **PASS** |
| **Audit 5B: Depth 차단 (물리 단절)** | 실제 PointCloud2 발행 프로세스 강제 종료 (Test B) | 감지 $0.30\sim0.55\text{s}$, 정지거리 $< 0.20\text{m}$ | 감지 0.421s, 차단 0.421s (PASS) / 정지변위 0.165m (UNVERIFIED) | **PASS (지연시간)** / **UNVERIFIED (제동거리)** |
| **Audit 6: LiDAR/ICP 고장 차단** | 주행 중 F4 스캔매처 노드 강제 종료 | 감지 $\le 0.55\text{s}$, 차단 $\le 0.58\text{s}$, 정지거리 $< 0.20\text{m}$ | 감지 0.406s, 차단 0.403s, 총변위 0.174m (고장주입~정지), 즉시 제동 | **PASS** |
| **종합 평가** | 전 7개 세부 감사 기준 충족 | 전 부문 합격 | **기능 동등성 및 지연시간 충족 / 실측 정지거리 UNVERIFIED** | **PASS (기능)** |

> [!WARNING]
> **11월 8일 감사(위 표)의 한계:** Audit 2 등 안전 게이트 PASS는 당시 launch 구조에서 얻은 값입니다. 그 구조에서는 전역 `cmd_vel` remap 때문에 `velocity_smoother`의 출력이 안전 게이트를 거치지 않고 `/cmd_vel`로 나갈 수 있었고, 이 감사는 주행 없이 유휴 상태에서 `/cmd_vel` 0건을 본 것이어서 그 우회를 검증하지 못했습니다. 11월 10일에 이 배선을 고쳤습니다(`cmd_vel_smoothed`, `behavior_server:cmd_vel` → `/cmd_vel_raw`, 게이트 1개; 기록: `docs/dev_notes/2026_11_10_cmd_vel_안전게이트_경로_점검.md`). 수정 후에는 게이트 단독 시험 86/86과 launch 그래프 확인(`/cmd_vel` 발행자가 게이트 하나)을 했지만, 위 표의 수치(0.421 s 등)를 수정된 배선으로 다시 측정한 것은 아닙니다.

#### 2) Depth 센서 단절 지연 및 제동 거리 정밀 분석 (0.421초 지연 및 0.165m 변위 검증)

이전 초기 시험에서 언급된 수치들의 물리적 정합성을 확인하기 위해, 시간-속도-변위 관계를 정밀 재검토하였습니다.

```text
[Test B: 실제 PointCloud2 발행 프로세스 강제 종료 타임라인]
t_last = 6.100s (마지막 유효 패킷 타임스탬프)
  │
  ├─> t_kill = 6.258s (프로세스 SIGKILL 강제 종료 시점, 평가 스크립트 p0 샘플링)
  │     │
  │     └─> 경과 시간 = 0.263s (프로세스 종료 시점부터 명령 차단까지의 지연)
  │
  ├─> 0.380s 워치독 만료 + 0.041s 루프 틱 = 총 0.421s
  │
  └─> t_cutoff = 6.521s (Stall Detector가 /cmd_vel을 0.0 m/s로 강제 클램핑)
        │
        └─> 감속 경과 후 완전 정지 (t_halt = 6.703s, 속도 < 0.005 m/s, 평가 스크립트 p1 샘플링)
              [경과 시간: t_halt - t_cutoff = 0.182s, 총 고장 경과 시간: t_halt - t_kill = 0.445s]
```

- **물리 감지 지연 (0.421초 - PASS):** 센서의 마지막 정상 수신 시점($t_{\text{last}} = 6.100\text{s}$)부터 안전 게이트의 차단 명령 시점($t_{\text{cutoff}} = 6.521\text{s}$)까지의 경과 시간으로, 설정된 워치독 타임아웃($0.380\text{s}$)에 타이머 루프 틱($0.041\text{s}$)이 더해진 실제 물리 지연 시간입니다.
- **프로세스 종료 지연 (0.263초):** 노드가 강제 종료된 시점($t_{\text{kill}} = 6.258\text{s}$)부터 차단($6.521\text{s}$)까지의 경과 시간입니다.
- **0.165m 수치에 대한 속도-변위 검증 및 UNVERIFIED 분류:**
  - 순항 속도가 약 $0.15\text{ m/s}$일 때, 고장 주입($t_{\text{kill}} = 6.258\text{s}$)부터 완전 정지($t_{\text{halt}} = 6.703\text{s}$)까지의 총 경과 시간은 **$0.445\text{초}$**입니다.
  - $0.15\text{ m/s}$ 속도로 $0.445\text{초}$ 동안 감속 없이 등속 주행하더라도 이론상 최대 이동 거리는 $0.15 \times 0.445 = \mathbf{0.0668\text{m}}$ (약 6.7cm)이며, 감속을 고려한 이론적 예상 변위는 약 $0.0532\text{m}$입니다.
  - 따라서 보고된 $0.165\text{m}$는 이론상 최대 이동 거리($0.0668\text{m}$)의 약 2.5배에 달해 단순 주행 모델로는 설명되지 않습니다.
  - 평가 스크립트(`execute_full_regression_audit.py`) 확인 결과, $p_0$와 $p_1$의 원본 위치 좌표 및 Ground Truth 시계열 데이터가 디스크에 별도 저장되지 않고 최종 스칼라 연산 결과(`round(stop_dist, 3)`)만 JSON에 기록되었습니다.
  - 원본 Ground Truth 데이터가 미보존되어 $0.165\text{m}$의 구체적 계산 과정을 사후 재현할 수 없으므로, **새로운 수치를 임의 추정하지 않고 해당 제동 거리를 공식 UNVERIFIED (미검증)로 판정**합니다. (향후 필요 시 원본 GT 시계열 로깅을 포함한 전용 제동 시험을 별도로 수행해야 함)

#### 3) 검증 범위의 구분: 22초 브링업 회귀 시험 vs 80초 장시간 탐색 시험

실험실(`experiments/limo_integrated_auto_mapping_20261108/`)의 서로 다른 실험 결과가 혼용되지 않도록 시험 목적과 정량 결과를 명확히 분리합니다.

- **22초 브링업 회귀 시험 (Regression Audit, 2026-11-08):**
  - **목적:** 메인 워크스페이스 빌드 후 9개 노드의 정상 연동, TF 단일 브로드캐스터 권한, F4/F5 주기 충족, 초기 지도 확장 및 안전 차단 페일세이프 검증.
  - **F4 (LiDAR ICP 단독):** Freq 10.0 Hz, TF 단일 소유, 초기 지도 확장 ($1.63\text{ m}^2 \to 1.70\text{ m}^2$), rosbag 2.509 MB 수집 및 재생 확인.
  - **F5_improved (ICP + IMU EKF):** Freq 50.0 Hz, TF 단일 소유, 초기 지도 확장 ($1.64\text{ m}^2 \to 1.72\text{ m}^2$), rosbag 3.267 MB 수집 및 재생 확인.
- **80초 장시간 자율 탐색 맵핑 시험 (`EXTENDED_INTEGRATION_VERIFICATION_REPORT.md`):**
  - **목적:** Lounge World 전역에서 explore_lite 연속 주행, 프론티어 누적, SLAM 루프 클로저 및 벽면 선명도 정량 평가.
  - **F4 (80초 완수):** 총 주행 거리 **3.005 m**, 총 지도 확장 **64.77 m²** (자유 탐색 면적 62.42 m², 점유 셀 수 937 cells), 벽면 두께 표준편차 $\sigma = 0.0860\text{ m}$, 최종 안전 상태 **NORMAL (무중단 완주)**.
  - **F5_improved (80초 완수):** 총 주행 거리 **2.351 m**, 총 지도 확장 **63.00 m²** (자유 탐색 면적 60.81 m², 점유 셀 수 878 cells), 벽면 두께 표준편차 $\sigma = 0.0904\text{ m}$, 최종 안전 상태 **STALL_LATCHED (유리벽 경계 정체)**.
  *(※ 과거 일부 문서 초안에 기재되었던 6.84m² 및 7.12m²는 녹화 비디오 파일 용량[6.84 MB, 7.12 MB in `video_decoding_audit.json`]의 단순 오기였으므로 전면 삭제함.)*

#### 4) 소프트웨어 안전 격리와 실물 하드웨어 페일세이프 한계 (UNVERIFIED 항목 명시)

> [!CAUTION]
> **하드웨어 안전 한계 및 실물 자동 주행 엄격 금지 안내:**
> 1. **소프트웨어 차단의 한계:** `allow_motor_drive:=false` 설정은 **ROS 2 소프트웨어 레벨에서 `/cmd_vel`을 차단하고 `/cmd_vel_locked`로 우회 격리**하는 기능입니다. 온보드 PC의 커널 패닉, OS 정지, 전원 차단 시의 하드웨어 페일세이프를 보장하지 못합니다.
> 2. **종료 수단과 비상 정지 구분:** `Ctrl + C`는 소프트웨어 정상 종료(Graceful Shutdown) 절차이며 비상 정지 장치가 아닙니다.
> 3. **RC 비상 채널 미검증:** 원격 무선 조종기(RC)의 비상 정지 채널 맵핑 및 실제 하드웨어 수신기 기능은 실측되지 않았으므로 **UNVERIFIED (미검증)** 상태로 유지합니다.
> 4. **MCU 워치독 및 전압 기준:** 바퀴를 띄운 무부하 벤치(0.10 m/s 직진)에서 `/cmd_vel` 발행이 끊긴 뒤 속도가 0이 되기까지 약 0.37~0.38 s(3회, `/odom` 기준)를 측정했고, `limo_base`를 SIGINT/SIGKILL로 종료해도 육안 1초 이내에 멈췄습니다(영상·타임스탬프 없음). 그러나 제조사 규격서·펌웨어로 확인한 값이 아니며 지상 주행·고속·회전 조건은 측정하지 않았습니다. 배터리 12.0V 기준도 현장 참고치입니다(**제한적 실측, 규격 UNVERIFIED**).
> 5. **실물 자동 주행 승인 조건:** 독립적인 물리적 비상 정지(E-Stop) 검증은 아직 없습니다. 실차 구동은 사용자가 입회하고 전원 스위치에 손을 둔 상태에서 단계별 승인을 받은 감독 시험에 한정했으며, 무인·장시간 자율 주행은 승인되지 않았습니다. 주행 중 임의 케이블 강제 분리 시험은 엄격히 금지합니다.

---

### 13.2 기존 개별 단위 평가 항목 (레거시 검증 기준선)

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

---

## 14. 개발 기록 및 관련 문서

프로젝트의 상세 운영 매뉴얼, 설계서, 회귀 검증 보고서 및 트러블슈팅 이력은 아래 문서들을 통해 확인할 수 있습니다.

### 정식 운영 및 워크스페이스 문서
- **[docs/INTEGRATED_AUTO_MAPPING_OPERATIONS_GUIDE.md](docs/INTEGRATED_AUTO_MAPPING_OPERATIONS_GUIDE.md)**: **[공식]** LIMO Pro 통합 자동 맵핑 정식 운영 가이드 (빌드, F4/F5 실행, RViz2 설정, rosbag2 프로파일, 비상 제동 언래치, 정상 종료 및 실차 점검 10단계 수록)
- **[capstone_ws/README.md](capstone_ws/README.md)**: **[공식]** 캡스톤 메인 ROS 2 워크스페이스 빠른 시작 안내서 (패키지 구조, 의존성, 빌드/실행 요약)

### 실험실 아카이브 및 검증 보고서
- **[experiments/limo_integrated_auto_mapping_20261108/docs/LIMO_PRO_WORKSPACE_FINAL_REGRESSION_REPORT.md](experiments/limo_integrated_auto_mapping_20261108/docs/LIMO_PRO_WORKSPACE_FINAL_REGRESSION_REPORT.md)**: 메인 워크스페이스 패키지 이식 최종 회귀 검증 보고서 (7개 부문 실측 데이터 및 판정 수록)
- **[experiments/limo_integrated_auto_mapping_20261108/docs/LIMO_PRO_WORKSPACE_PORTING_AND_INTEGRATION_REPORT.md](experiments/limo_integrated_auto_mapping_20261108/docs/LIMO_PRO_WORKSPACE_PORTING_AND_INTEGRATION_REPORT.md)**: 메인 워크스페이스 패키지 이식 및 인터페이스 구현 보고서
- **[experiments/limo_integrated_auto_mapping_20261108/docs/LIMO_PRO_WORKSPACE_PORTING_AND_INTEGRATION_PLAN.md](experiments/limo_integrated_auto_mapping_20261108/docs/LIMO_PRO_WORKSPACE_PORTING_AND_INTEGRATION_PLAN.md)**: 메인 워크스페이스 패키지 이식 및 인터페이스 설계서

### 검증 현황 · 설정 · 설계
- **[docs/REAL_ROBOT_STATUS.md](docs/REAL_ROBOT_STATUS.md)**: 실차 검증 / 시뮬레이션 검증 / 오프라인 검증 / 제안의 구분
- **[docs/CONFIG_SIM_VS_REAL.md](docs/CONFIG_SIM_VS_REAL.md)**: 시뮬레이션 기준선과 실차 설정의 파일·값 차이
- **[docs/design/AUTONOMOUS_DRIVING_DESIGN.md](docs/design/AUTONOMOUS_DRIVING_DESIGN.md)**: 자율주행 설계 합의 기록(대부분 제안 단계)
- **[docs/dev_notes/2026_11_10_cmd_vel_안전게이트_경로_점검.md](docs/dev_notes/2026_11_10_cmd_vel_안전게이트_경로_점검.md)**: cmd_vel 안전 게이트 경로 점검

### 기존 핵심 기술 및 문제 해결 기록
- **[docs/troubleshooting.md](docs/troubleshooting.md)**: 7단계 표준 형식으로 작성된 핵심 트러블슈팅 7선 가이드
- **[docs/navigation.md](docs/navigation.md)**: Nav2 아키텍처, 10개 TF 프레임 무결성 계측, AMCL 파라미터, Costmap 분석 및 실행 가이드
- **[docs/roadmap.md](docs/roadmap.md)**: 4단계 구현 상태 매트릭스 및 동적 장애물 회피, 순찰, 안내 기능 상세 설계
- **[docs/DEVELOPMENT_LOG.md](docs/DEVELOPMENT_LOG.md)**: 실제 하드웨어 센서 검증, SLAM 매핑, Nav2 연동, 오류 분석 및 현장 검증 기록
- **[docs/NAV2_TROUBLESHOOTING.md](docs/NAV2_TROUBLESHOOTING.md)**: Nav2 자율주행 심층 트러블슈팅 및 가이드
- **[docs/SLAM_MAPPING.md](docs/SLAM_MAPPING.md)**: 실내 SLAM 매핑 절차 및 지도 작성 검증 기록
- **[docs/PHASE12_SIMULATION_VALIDATION.md](docs/PHASE12_SIMULATION_VALIDATION.md)**: Gazebo 시뮬레이션 환경 검증 기록

---

## 15. 향후 개발 방향

프로젝트는 다음 순서로 단계를 밟아 실물 하드웨어 주행 및 서비스 애플리케이션으로 확장합니다.

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
통합 자동 맵핑 및 안전 게이트 시뮬레이션 검증 (완료, 2026-11-08)
      ↓
실물 LIMO 거치대 벤치 무부하 안전 점검 (예정)
      ↓
실물 주행 환경 통합 자동 맵핑 현장 실차 검증 (예정)
      ↓
동적 장애물 회피 실차 튜닝 (계획)
      ↓
자율 순찰 및 목적지 안내 응용 확장 (계획)
      ↓
최종 시연 및 평가 (계획)
```

---

## 16. 참고자료 및 출처

본 프로젝트의 개발 및 구현 과정에서는 다음 자료를 우선적으로 참고합니다.

1. **WeGo, 「Limo_ROS2 교재」**
2. **AgileX Robotics, LIMO 공식 자료**
3. **ROS 2 공식 문서**
4. **Navigation2 공식 문서**
5. **SLAM Toolbox 공식 문서**
6. 프로젝트 개발 과정에서 확인한 관련 기술 문서 및 커뮤니티 자료

---

## 17. 프로젝트 산출물

- **ROS 2 통합 패키지 (`capstone_ws/src/`):**
  - `limo_depth_obstacles`: 3D DaBai Depth 카메라 장애물 필터링 및 진단 패키지
  - `limo_capstone`: 통합 자동 맵핑 Launch 2종, 파라미터 YAML 5종, F4/F5 상태 추정, TF 중재, 안전 명령 게이트 노드 8종, 시뮬레이션 월드 및 모델
- **정식 문서 (`docs/` 및 `capstone_ws/`):**
  - [docs/INTEGRATED_AUTO_MAPPING_OPERATIONS_GUIDE.md](docs/INTEGRATED_AUTO_MAPPING_OPERATIONS_GUIDE.md): 통합 자동 맵핑 정식 운영 가이드
  - [capstone_ws/README.md](capstone_ws/README.md): 메인 워크스페이스 빠른 시작 안내서
- **실험 및 회귀 검증 아카이브 (`experiments/limo_integrated_auto_mapping_20261108/`):**
  - 전 부문 자동 회귀 감사 스크립트: `evaluation/scripts/execute_full_regression_audit.py`
  - 회귀 감사 결과 JSON: `evaluation/results/capstone_ws_final_regression_audit_20261108.json`
  - 공식 rosbag2 검증 데이터: F4 (2.509 MB), F5_improved (3.267 MB)
  - 검증 완료 보고서: `docs/LIMO_PRO_WORKSPACE_FINAL_REGRESSION_REPORT.md`
- **실내 점유격자 지도 데이터 (`capstone_ws/src/limo_capstone/maps/`):**
  - `demo_map`, `MC_2F_10_07`, `MC_3F_10_15`
- **후속 개발 응용 (계획):**
  - 자율 순찰 기능 (Action 노드)
  - 목적지 안내 기능 (Service 노드)

---

## 18. 실물 LIMO 이식 전 하드웨어 안전 체크리스트 (Hardware Deployment Gateways)

실물 LIMO Pro 로봇에 본 소프트웨어를 적용하기 전, 안전사고 방지를 위해 아래 10단계 체크리스트를 순차적으로 확인하고 서명해야 합니다.

| 단계 | 점검 항목 | 확인 기준 및 절차 | 판정 |
| :---: | :--- | :--- | :---: |
| **1** | **배터리 전압** | 주행용 배터리 전압 $\ge 12.0\text{V}$ 점검 (브라운아웃 방지용 점검치, 제조사 공인 스펙 미검증 [UNVERIFIED]) | [ ] 확인 |
| **2** | **벤치 무부하 거치** | 바퀴 4개가 지면에서 완전히 뜨도록 전용 거치대에 로봇 거치 | [ ] 확인 |
| **3** | **소프트웨어 모터 잠금** | `allow_motor_drive:=false` 설정 후 Launch 기동 | [ ] 확인 |
| **4** | **명령 격리 실측** | `ros2 topic echo /cmd_vel` 메시지 수신 0건, `/cmd_vel_locked` 라우팅 확인 | [ ] 확인 |
| **5** | **비상 차단 체계 대기** | 온보드 물리 전원 스위치 즉시 차단 담당자 배치 (`Ctrl+C`는 E-Stop 아님, RC 비상 채널은 미검증 [UNVERIFIED]) | [ ] 확인 |
| **6** | **센서 및 TF 단일 소유** | `/scan` (10Hz), `/imu` (50Hz), `map->odom->base_footprint->base_link` 점검 | [ ] 확인 |
| **7** | **Depth 장애물 필터** | DaBai Depth 카메라 구동 후 `/depth_obstacles/points` 토픽 확인 | [ ] 확인 |
| **8** | **안전 게이트 텔레메트리** | `/stall_detector/status_json`에서 `state: "NORMAL"`, `allow_motor_drive: false` 확인 | [ ] 확인 |
| **9** | **STM32 워치독 벤치 점검** | 무부하 상태에서 명령 중단 시 모터 정지 확인 (2026-11-11 벤치 측정값 약 0.4 s는 펌웨어 미검증, **케이블 강제 분리 금지**) | [ ] 확인 |
| **10** | **실차 주행 승인 조건** | **물리 E-Stop 검증 전에는 감독 입회 없는 실물 자율 주행 금지. 구동은 단계별 사용자 승인 후에만** | [ ] BLOCKED |
