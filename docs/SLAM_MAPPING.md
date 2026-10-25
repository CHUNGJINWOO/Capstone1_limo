# SLAM 및 지도 작성

## 목적과 범위

이 문서는 LIMO Pro의 LiDAR와 ROS 2를 이용해 2D 지도를 작성할 때 필요한 데이터 흐름, 확인된 상태, 검증 절차를 정리한다. 설정 파일에 노드가 정의되어 있거나 노드가 실행된다는 사실만으로 SLAM이 성공했다고 판단하지 않는다.

## 데이터 흐름

```text
YDLiDAR T-mini Plus
        ↓
ydlidar_ros2_driver
        ↓  /scan (sensor_msgs/msg/LaserScan)
      TF 변환
        ↓
   SLAM Toolbox
        ↓
 /map, /map_metadata, map → odom
        ↓
  Map Saver (PGM + YAML)
```

위 그림은 시스템 구성 개념이다. 모든 단계가 현재 정상 동작한다고 의미하지 않는다.

## 확인된 내용

### LiDAR 입력
2026-09-30 실기기 확인에서 YDLiDAR T-mini Plus가 연결되고 `/scan`에 LaserScan 데이터가 출력되는 것을 확인했다. 당시 확인된 메시지 프레임은 `laser_frame`이었다.

### SLAM Toolbox 연결
2026-10-01 점검에서 `/slam_toolbox` 실행, `/scan` 구독, `/map` 및 `/map_metadata` Publisher 등록을 확인했다. LiDAR와 SLAM Toolbox 사이의 QoS Reliability는 모두 `BEST_EFFORT`였다.

단, 그 실험에서는 실제 `/map` 데이터가 정상 생성되는 것까지 확인하지 못했다. 따라서 해당 시점의 결과는 지도 작성 성공이 아니라 입력 연결 및 노드 상태 확인이다.

### 지도 저장
사용자가 2026-10-15 제공한 Map Saver 출력에는 `764 × 809` cells, `0.05 m/pix`, `/home/pcu/map_limo.pgm` 및 `/home/pcu/map_limo.yaml` 저장 경로와 `Map saved successfully`가 기록되어 있다. 사용자는 이후 파일명을 `MC_3F_10_15.pgm`과 `MC_3F_10_15.yaml`로 변경했으며 두 파일이 LIMO 온보드 PC에 있다고 보고했다. 이 기록은 출력과 사용자 보고를 구분해 반영한 것이며, 현재 실제 파일 경로와 YAML의 `image` 참조, 지도 품질은 별도 확인이 필요하다.

## 문제를 진단하는 순서

지도가 생성되지 않거나 RViz에 표시되지 않으면 모든 구성요소를 한 번에 변경하지 말고 다음 순서로 점검한다.

1. **LiDAR 입력:** `ros2 topic hz /scan`, `ros2 topic echo /scan --once`로 메시지 발행과 프레임을 확인한다.
2. **QoS:** `ros2 topic info /scan --verbose`에서 Publisher와 Subscriber의 Reliability 등 QoS를 비교한다.
3. **TF:** `ros2 run tf2_ros tf2_echo odom base_link` 및 실제 센서 프레임 변환을 확인한다. 프레임 이름은 실행 중인 URDF와 드라이버 설정을 기준으로 확인한다.
4. **SLAM 입력:** `ros2 node info /slam_toolbox`에서 `/scan` 구독과 관련 Publisher를 확인한다.
5. **실제 지도 메시지:** `ros2 topic echo /map --once` 또는 `ros2 topic echo /map_metadata --once`로 메시지가 발행되는지 확인한다.
6. **시각화:** RViz Fixed Frame과 Map display 설정을 확인한다.
7. **저장 결과:** Map Saver의 종료 메시지, YAML의 `image` 경로, PGM 파일 존재를 확인한다.

이 단계들은 진단 절차이며, 실행 결과가 없는 항목은 아직 검증되지 않은 상태로 기록한다.

## 지도 저장

기본 Map Saver CLI는 다음 형태로 실행한다.

```bash
ros2 run nav2_map_server map_saver_cli -f <저장_경로>
```

실행 전 현재 ROS 2 환경과 `/map` 토픽을 확인한다. 저장 후에는 YAML 파일의 이미지 경로가 실제 PGM 파일을 가리키는지 확인한다. 파일이 저장되었다는 사실만으로 지도 품질이나 localization 적합성이 보장되지는 않는다.

## 현재 한계와 다음 작업

- 저장 지도 `MC_3F_10_15`의 실제 파일 경로와 YAML 참조는 별도 확인이 필요하다.
- 지도 품질(벽면 직선성, 폐루프 정합, 노이즈)은 정량적으로 평가하지 않았다.
- AMCL과 Nav2는 지도 저장과 별개로 테스트하고, 각 결과를 실기기·시뮬레이션 여부와 함께 기록한다.

## 실험 기록 원칙

각 실험은 아래 형식으로 기록한다.

- **문제:** 관찰된 현상과 재현 조건
- **원인 가설:** 근거가 있는 가설만 기재하고 확정 원인과 구분
- **조치:** 실제 변경한 설정·코드·명령
- **검증:** 변경 전후 출력, 토픽, 로그 또는 주행 결과
- **한계:** 아직 확인하지 못한 내용
