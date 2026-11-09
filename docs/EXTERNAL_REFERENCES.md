# External References, Upstream Attribution & Dependency Guide

본 문서는 **Capstone1_limo** 프로젝트에서 참고·인용한 외부 오픈소스 프로젝트의 공식 출처, 업스트림 라이선스 현황, 자체 개발 기능과 외부 의존성의 경계, 그리고 외부 자산의 재배포 제외 정책을 명시합니다.

---

## 1. 공개 정책 및 외부 자산 재배포 원칙

본 프로젝트의 GitHub 공개 정책은 다음과 같습니다:

1. **자체 연구·개발 성과물의 투명한 공개**:
   - 캡스톤 디자인 연구를 통해 독자 개발한 ROS 2 알고리즘 노드, Ackermann 특화 Behavior Tree XML, 가상 평가 환경(SDF/World), 26개 실험 결과 JSON 및 분석 문서는 완전하게 공개합니다.
2. **라이선스 미확인 자산의 재배포 배제**:
   - 상위 업스트림 저장소에서 오픈소스 라이선스가 명시되지 않았거나(`TODO`), 상업적/비상업적 제3자 재배포 권한이 확인되지 않은 외부 원본 코드, 독점 바이너리, 3D CAD 메시 파일은 공개 저장소 Git 이력에서 배제합니다.
3. **공식 업스트림 명시를 통한 의존성 분리**:
   - 실차 로봇 하드웨어 구동에 필요한 제조사 드라이버 및 오픈소스 패키지는 저장소에 복사·포함(Vendoring)하지 않고, 공식 업스트림 저장소 출처와 표준 설치 절차를 문서로 안내합니다.

---

## 2. 외부 프로젝트 및 업스트림 공식 출처

본 프로젝트가 기구학 규격, 센서 인터페이스, 통신 프로토콜을 참고하거나 의존하는 외부 프로젝트 목록입니다.

| 프로젝트 / 패키지 명 | 원저작자 / 공식 출처 | 공식 업스트림 링크 | 선언 라이선스 | 본 프로젝트 내 상태 및 권한 |
| :--- | :--- | :--- | :---: | :--- |
| **`limo_ros2`**<br>(`limo_base`, `limo_msgs`) | AgileX Robotics<br>(松灵机器人) | [agilexrobotics/limo_ros2](https://github.com/agilexrobotics/limo_ros2) | `limo_base`: Apache-2.0<br>`limo_msgs`: BSD | 하드웨어 섀시 시리얼 통신 프로토콜 참고. 공개 저장소에서는 제외하고 공식 upstream 설치 권장. |
| **`limo_description`**<br>(3D 메시 및 URDF) | AgileX Robotics | [agilexrobotics/limo_ros2](https://github.com/agilexrobotics/limo_ros2) | `TODO: License declaration` | **라이선스 미확인 / 재배포 불가**. 3D 메시(`limo_wheel.dae`/`stl`)는 공개 이력에서 배제. |
| **`limo_car`**<br>(Classic Gazebo) | AgileX Robotics &<br>TU Braunschweig | [agilexrobotics/limo_ros2](https://github.com/agilexrobotics/limo_ros2) | Apache-2.0 | 과거 Classic Gazebo 모델. Fortress 시뮬레이션에서는 자체 모델로 대체되었으므로 공개 제외. |
| **`ros2_astra_camera`** | Orbbec 3D Technology | [orbbec/ros2_astra_camera](https://github.com/orbbec/ros2_astra_camera) | `astra_camera`: TODO<br>`astra_camera_msgs`: all copyrights reserved | **독점 저작권 및 바이너리 포함**. 비공개 드라이버 라이브러리(`liborbbec.so`) 포함으로 재배포 배제. |
| **`ydlidar_ros2_driver`** | EAI YDLIDAR | [YDLIDAR/ydlidar_ros2_driver](https://github.com/YDLIDAR/ydlidar_ros2_driver) | **MIT** | 실차 2D LiDAR 센서 공식 드라이버. 라이선스 확인됨. 공식 upstream 설치 권장. |
| **`wego` / `wego_2d_nav`**<br>`limo_ros2_application` | WeGO Robotics | [wego-robotics](https://github.com/wego-robotics) | **Apache-2.0** | 국내 유통사 교육·실습용 런치 및 기본 노드. 본 프로젝트 핵심 알고리즘과 무관하여 제외. |
| **`robot_localization`** | Tom Moore / CRA | [cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization) | **Apache-2.0** | 표준 EKF 센서 융합 패키지. ROS 2 apt 바이너리 또는 upstream 소스 설치 권장. |
| **`m-explore-ros2`**<br>(`explore_lite`) | Carlos Alvarez / Robo Friends | [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) | **BSD** | 프론티어 기반 자동 탐색 알고리즘. ROS 2 공식 커뮤니티 패키지. |
| **`magic_enum`** | Neargye | [Neargye/magic_enum](https://github.com/Neargye/magic_enum) | **MIT** | C++17 Header-only Enum 리플렉션 라이브러리. |
| **`libuvc`** | Ken Tossell / libuvc | [libuvc/libuvc](https://github.com/libuvc/libuvc) | **BSD** | USB Video Class 기기 크로스 플랫폼 라이브러리. |

---

## 3. 자체 개발 기능 vs 외부 의존성 명확 구분표 (Attribution Matrix)

본 연구의 공학적 기여도와 외부 기반 기술의 경계를 명확히 구분합니다.

| 모듈 / 컴포넌트 | 본 프로젝트 자체 개발 내용 (Self-Developed) | 외부 의존 및 참고 내용 (External Basis) | 구분 |
| :--- | :--- | :--- | :---: |
| **3D Depth 장애물 회피**<br>(`limo_depth_obstacles`) | - 지면 분리 및 관심 영역 높이 윈도잉 [0.025m, 0.35m]<br>- 3D 점군의 2D 레이저 스캔 투영 알고리즘<br>- 사각지대 돌발 장애물 감지 및 동적 세이프티 게이트<br>- 0.38초 센서 워치독 페일세이프 | Orbbec 카메라가 발행하는 표준 ROS 2 `sensor_msgs/msg/Image` 및 `PointCloud2` 인터페이스 | **100% 자체 개발** |
| **Ackermann 내비게이션 BT**<br>(`limo_capstone`) | - 조향각 한계(0.488 rad) 고려 제자리 회전(`Spin`) 배제<br>- 후진 및 지연 대기(`BackUp`, `Wait`) 기반 복구 시퀀스 설계<br>- 협소 통로 끼임 방지 허용 오차 튜닝 | ROS 2 Nav2 BT Navigator 프레임워크 및 액션 인터페이스 | **자체 알고리즘 설계**<br>(Nav2 구조 활용) |
| **보조 오도메트리 추정**<br>(`limo_capstone`) | - 2D LiDAR 점군 매칭(`lidar_scan_matcher_odom_node`)<br>- 슬립 감지 및 EKF 융합 가중치 동적 전환 (`fusion_f5`) | ROS 2 Humble 센서 메시지 규격 | **100% 자체 개발** |
| **Gazebo Fortress 시뮬레이션**<br>(`limo_capstone`) | - Fortress 전용 단일 모델(`models/limo_eval/model.sdf`)<br>- Ackermann 기구학 및 센서 플러그인 독자 구성<br>- 실측 MC관 3층 복도 및 라운지 장애물 SDF 월드 구축 | AgileX LIMO 하드웨어 규격 (휠베이스 0.2m, 윤거 0.14m, 바퀴 반경 0.045m) | **자체 가상환경 구축**<br>(치수 규격 인용) |
| **차체 구동 및 시리얼 통신** | - 통신 상태 모니터링 및 실물 안전 게이트 런치 제어 | AgileX STM32 CAN/시리얼 프로토콜 (`limo_base`) | **외부 제조사 드라이버** |
| **센서 하드웨어 구동** | - 센서 좌표계(TF) 캘리브레이션 및 데이터 정규화 | YDLIDAR 2D 라이다 드라이버, Orbbec 카메라 드라이버 | **외부 제조사 드라이버** |
| **프론티어 탐색** | - 탐색 완료 판정 및 조기 종료 감지 게이트 설계 | `m-explore-ros2` (`explore_lite`) | **외부 오픈소스 패키지** |

---

## 4. 라이선스 미확인 외부 자산의 재배포 제외 사유

1. **AgileX `limo_description` 및 3D 메시 파일 (`limo_wheel.dae`, `limo_wheel.stl`)**:
   - 업스트림 공식 `package.xml`에 `<license>TODO: License declaration</license>`으로 작성되어 있으며, 별도의 라이선스 허용 문서가 없습니다.
   - 바퀴 3D CAD 메시는 크기가 파일당 11.4MB/7.0MB에 달하며 저장소 내 3곳에 중복 존재(총 54MB)하여 저장소를 불필요하게 비대화시킵니다.
   - 본 프로젝트의 물리 시뮬레이션은 충돌 프리미티브(Cylinder)를 기반으로 하므로, 3D CAD 메시를 저장소에 직접 배포하지 않고 공식 업스트림을 참조하도록 제외합니다.
2. **Orbbec `ros2_astra_camera`**:
   - `astra_camera_msgs` 패키지에 `all copyrights reserved` 조항이 명시되어 있어 오픈소스 재배포가 법적으로 제한됩니다.
   - `openni2_redist/` 내부에 제조사 독점 컴파일 바이너리(`liborbbec.so`)가 포함되어 있어 공개 Git 저장소에 부적합합니다.

---

## 5. 외부 의존성 관리 및 재현 가이드

외부 코드를 저장소에 포함하지 않고 독립적으로 관리하기 위한 표준 ROS 2 워크플로우입니다.

### 5.1 검증된 환경 vs 미검증 환경 구분

- **[검증 완료] Gazebo Fortress 시뮬레이션 환경**:
  - 본 저장소의 자체 개발 패키지(`limo_capstone`, `limo_depth_obstacles`)와 ROS 2 Humble 기본 바이너리만으로 **물리 시뮬레이션, Nav2 자율주행, 3D 장애물 회피, SLAM 자동 맵핑이 100% 구동**됨을 검증했습니다.
  - 가상 환경의 센서 데이터는 Gazebo Fortress의 `ros_gz_bridge`를 통해 생성되므로, YDLidar나 Astra 카메라 드라이버가 전혀 필요하지 않습니다.
- **[실물 환경 설치 필요] 실차 로봇 하드웨어 배포 환경**:
  - 실제 LIMO Pro 로봇 하드웨어에 배포할 때는 STM32 모터 제어 및 실제 USB 센서 통신을 위해 공식 드라이버를 워크스페이스에 설치해야 합니다.

### 5.2 권장 외부 의존성 명세 (`third_party.repos`)

ROS 2 표준 `vcstool`을 사용하여 외부 업스트림을 독립적으로 관리할 수 있습니다:

```yaml
repositories:
  # 1. AgileX LIMO 공식 드라이버 (실차 모터 제어 및 시각화용 메시 필요 시)
  limo_ros2:
    type: git
    url: https://github.com/agilexrobotics/limo_ros2.git
    version: humble

  # 2. YDLidar 2D 라이다 드라이버 (실차 라이다 센서 구동 시)
  ydlidar_ros2_driver:
    type: git
    url: https://github.com/YDLIDAR/ydlidar_ros2_driver.git
    version: master

  # 3. Orbbec DaBai 카메라 드라이버 (실차 깊이 카메라 센서 구동 시)
  ros2_astra_camera:
    type: git
    url: https://github.com/orbbec/ros2_astra_camera.git
    version: humble

  # 4. 프론티어 탐색 알고리즘
  m-explore-ros2:
    type: git
    url: https://github.com/robo-friends/m-explore-ros2.git
    version: main

  # 5. 로봇 로컬라이제이션 EKF (apt 바이너리 미사용 시 소스 빌드)
  robot_localization:
    type: git
    url: https://github.com/cra-ros-pkg/robot_localization.git
    version: humble-devel
```

### 5.3 단계별 설치 명령어

```bash
# 1. 워크스페이스 src 디렉터리로 이동
cd ~/school/2026-2/Capstone1_limo/capstone_ws/src

# 2. 실차 구동에 필요한 외부 의존성 일괄 내려받기 (필요 시)
vcs import . < ../../docs/third_party.repos

# 3. 시스템 바이너리 의존성 설치 (rosdep)
cd ~/school/2026-2/Capstone1_limo/capstone_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. 워크스페이스 빌드 (자체 개발 패키지 우선 빌드)
colcon build --symlink-install --packages-select limo_depth_obstacles limo_capstone
```

---

## 6. 라이선스 고지 의무 및 권리 표기

- 본 프로젝트의 자체 개발 코드는 특별한 명시가 없는 한 프로젝트 라이선스(Apache-2.0 권장)를 따릅니다.
- 본 프로젝트에서 참고한 AgileX Robotics, Orbbec, YDLidar, WeGO Robotics 및 ROS 2 오픈소스 커뮤니티의 모든 상표, 상호 및 원저작권은 각 원저작자에게 귀속됩니다.
- 본 고지는 오픈소스 저작권 의무를 준수하고, 라이선스 미확인 자산의 무단 재배포를 방지하며, 연구 개발 기여도를 투명하게 밝히기 위해 작성되었습니다.
