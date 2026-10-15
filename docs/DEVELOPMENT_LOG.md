# 개발 기록

이 문서는 날짜별 명령어 나열 대신, 프로젝트에서 확인한 기술적 결과와 아직 해결되지 않은 문제를 추적한다. 상세한 원인 분석과 재현 절차는 주제별 문서에 기록한다.

## 2026-09-30 — LIMO Base 및 LiDAR 데이터 확인

### 목적
실기기에서 LIMO 구동부와 YDLiDAR T-mini Plus가 ROS 2 토픽을 제공하는지 확인한다.

### 확인한 사실
- 환경: LIMO onboard PC, Ubuntu 22.04.5, ROS 2 Humble, workspace `~/agilex_ws`.
- `/dev/ttyUSB1`로 `limo_base_node`가 실행되는 것을 확인했다.
- `/dev/ttyUSB0`에서 YDLiDAR T-mini Plus가 연결되고 스캔을 시작하는 로그를 확인했다.
- `/scan`에서 `sensor_msgs/msg/LaserScan` 데이터와 `laser_frame` 프레임을 확인했다.
- `/wheel/odom`의 메시지 타입은 `nav_msgs/msg/Odometry`이며 Publisher는 `limo_base_node`였다.
- `/imu`에서 `sensor_msgs/msg/Imu` 데이터가 출력되는 것을 확인했다.
- `odom → base_link` TF 변환을 확인했다.

### 문제 및 후속 작업
- workspace를 source할 때 `limo_car/local_setup.bash`를 찾지 못한다는 경고가 있었다. 당시 원인과 영향은 확정하지 않았고 수정도 하지 않았다.
- 이후 SLAM을 검증하기 전에 센서 토픽, TF, QoS를 다시 확인할 수 있도록 기준 상태를 기록했다.

### 검증 범위
센서와 기본 ROS 2 인터페이스의 일부를 확인했다. 이것만으로 SLAM, localization 또는 Nav2 자율주행이 검증된 것은 아니다.

---

## 2026-10-01 — SLAM Toolbox 초기 연결 점검

### 목적
실기기 LiDAR 데이터가 SLAM Toolbox에 전달되는지, 지도 생성까지 확인되는지 점검한다.

### 확인한 사실
- SLAM Toolbox 노드 `/slam_toolbox`가 실행되는 것을 확인했다.
- `/scan`의 Publisher는 `ydlidar_ros2_driver_node`, Reliability는 `BEST_EFFORT`였다.
- SLAM Toolbox의 `/scan` Subscriber도 `BEST_EFFORT`로 확인되어 QoS 호환성을 확인했다.
- SLAM Toolbox 노드에 `/scan` 구독과 `/map`, `/map_metadata`, `/tf` 등 Publisher가 등록된 것을 확인했다.

### 문제
실제 `/map` 지도 데이터가 정상적으로 생성되는 것까지 확인하지 못했다. 노드가 실행되고 토픽 인터페이스가 등록된 사실만으로 SLAM 성공을 판단할 수 없었다.

### 결론
당시 검증 결과는 **SLAM Toolbox 실행 및 입력 연결 확인**, 지도 생성은 **미확인**이다. 지도 생성 여부를 확인하려면 `/map` 메시지와 RViz 시각화, 저장된 지도 파일을 각각 확인해야 한다.

---

## 2026-10-15 — 지도 저장(Map Saver) 확인

### 목적
현재 `/map` 토픽의 점유격자 지도를 파일로 저장할 수 있는지 확인한다.

### 실행 및 결과
사용자가 `save_map` 명령을 실행했고, Map Saver가 다음 결과를 출력했다고 보고했다.

```text
Received a 764 X 809 map @ 0.05 m/pix
Writing map occupancy data to /home/pcu/map_limo.pgm
Writing map metadata to /home/pcu/map_limo.yaml
Map saved
Map saved successfully
```

- 저장 요청에서 받은 지도 크기: 764 × 809 cells
- 해상도: 0.05 m/cell
- 저장 형식: PGM 이미지와 YAML 메타데이터
- 사용자는 파일 이름을 `MC_3F_10_15`로 변경했다고 보고했다.

### 검증 범위 및 한계
위 내용은 사용자가 제공한 실행 결과와 보고를 근거로 기록했다. 현재 저장 파일의 실제 존재 여부, YAML의 image 경로, 지도 품질과 실내 구조의 정합성은 이 기록만으로 확인할 수 없다. 지도 저장 성공은 AMCL localization이나 Nav2 자율주행 성공을 의미하지 않는다.

---

## 다음 검증 항목

- [ ] `MC_3F_10_15.yaml`의 `image` 경로와 PGM 파일이 일치하는지 확인
- [ ] RViz에서 저장 지도를 열어 복도·벽면·장애물의 형상과 왜곡을 점검
- [ ] 지도 기반 localization(AMCL)의 초기 위치 설정 및 위치 추정 확인
- [ ] Nav2에서 목표점 경로 계획, 경로 추종, 장애물 반응을 각각 분리해 검증
- [ ] 실제 주행 결과와 실패 로그를 수집한 뒤 문제·원인·조치·재검증 결과를 주제별 문서에 기록
