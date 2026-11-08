# LIMO Pro 캡스톤 메인 ROS 2 워크스페이스 (`capstone_ws`)

본 디렉터리는 LIMO Pro 실내 자율 순찰 및 안내 프로젝트의 메인 ROS 2 Humble 워크스페이스입니다.

---

## 1. 패키지 구성

| 패키지명 | 유형 | 주요 역할 |
| :--- | :---: | :--- |
| **`limo_capstone`** | ament_python | 내비게이션 통합 메인 패키지. F4 ICP 오도메트리, EKF 융합, TF 중재, Stall Detector 안전 게이트, Nav2 & explore_lite 설정, Gazebo 시뮬레이션 Launch 및 실물 Launch |
| **`limo_depth_obstacles`** | ament_python | Orbbec DaBai Depth 카메라 3D 포인트클라우드 장애물 필터 패키지. 높이 윈도잉 [0.025m, 0.35m], 셀프 필터링, 0.38초 센서 워치독 |
| **`limo_ros2`** | ament_cmake / python | AgileX 공식 LIMO 하드웨어 드라이버 및 모델 (`limo_base`, `limo_car`, `limo_description`, `limo_msgs`) |
| **`m-explore-ros2`** | ament_cmake | explore_lite 프론티어 탐색 알고리즘 |
| **`ydlidar_ros2_driver`** | ament_cmake | 2D LiDAR 센서 드라이버 |

---

## 2. 빠른 빌드 및 실행

### 2.1 빌드
```bash
cd ~/school/2026-2/Capstone1_limo/capstone_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select limo_depth_obstacles limo_capstone
source install/setup.bash
export LD_LIBRARY_PATH=$PWD/install/robot_localization/lib:/opt/ros/humble/lib:$LD_LIBRARY_PATH
```

### 2.2 시뮬레이션 자동 맵핑 실행
```bash
# F4 모드 (2D LiDAR ICP 기반 탐색)
ros2 launch limo_capstone limo_integrated_auto_mapping_sim.launch.py estimator_type:=f4

# F5_improved 모드 (LiDAR ICP + IMU EKF 융합 기반 탐색)
ros2 launch limo_capstone limo_integrated_auto_mapping_sim.launch.py estimator_type:=f5_improved
```

### 2.3 실물 로봇 안전 모의 실행 (모터 잠금 상태)
```bash
ros2 launch limo_capstone limo_integrated_auto_mapping_real.launch.py allow_motor_drive:=false
```

---

## 3. 상세 운영 매뉴얼
전체 파라미터 조작, RViz2 설정, rosbag2 프로파일, 센서 타임아웃 페일세이프 분석 및 실물 로봇 사전 체크리스트는 **[docs/INTEGRATED_AUTO_MAPPING_OPERATIONS_GUIDE.md](../docs/INTEGRATED_AUTO_MAPPING_OPERATIONS_GUIDE.md)**를 참조하십시오.
