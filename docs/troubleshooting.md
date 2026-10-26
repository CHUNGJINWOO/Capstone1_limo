# 트러블슈팅 및 문제 해결 기록 (Troubleshooting)

본 문서는 LIMO Pro를 활용한 자율주행, SLAM, TF 좌표계, 그리고 내비게이션 파이프라인 구축 과정에서 마주친 핵심 기술적 문제들과 그 해결 과정을 기록합니다.
모든 사례는 **실제 터미널 로그, 커널 메시지, 소스 코드 diff, 그리고 실차/시뮬레이션 계측 데이터**를 바탕으로 작성되었으며, 추정 사항과 확인된 사실을 명확히 구분하였습니다.

---

## 목차
- [사례 1: 배터리 저전압 돌입전류로 인한 USB 브라운아웃 및 시리얼 포트 변동](#사례-1-배터리-저전압-돌입전류로-인한-usb-브라운아웃-및-시리얼-포트-변동)
- [사례 2: YDLiDAR `/scan` 토픽의 DDS QoS 불일치 및 RViz 포인트클라우드 미표시](#사례-2-ydlidar-scan-토픽의-dds-qos-불일치-및-rviz-포인트클라우드-미표시)
- [사례 3: `setup.py`의 디렉터리 복사 시도로 인한 패키지 빌드 및 지도 배포 실패](#사례-3-setuppy의-디렉터리-복사-시도로-인한-패키지-빌드-및-지도-배포-실패)
- [사례 4: Nav2 Costmap 필수 좌표계 `base_footprint` TF 누락 및 변환 타임아웃](#사례-4-nav2-costmap-필수-좌표계-base_footprint-tf-누락-및-변환-타임아웃)
- [사례 5: Explore Lite 목표 좌표와 조기 종료](#사례-5-explore-lite-목표-좌표-기준과-반복-목표조기-종료-조사)
- [사례 6: Nav2 Lifecycle 노드 비활성화 및 Durability 불일치로 인한 지도 수신 실패](#사례-6-nav2-lifecycle-노드-비활성화-및-durability-불일치로-인한-지도-수신-실패)
- [사례 7: Gazebo 시뮬레이션 환경과 실차 지도(`MC_3F_10_15`) 간 LiDAR 기하학적 정합 불일치](#사례-7-gazebo-시뮬레이션-환경과-실차-지도mc_3f_10_15-간-lidar-기하학적-정합-불일치)

---

## 사례 1: 배터리 저전압 돌입전류로 인한 USB 브라운아웃 및 시리얼 포트 변동

### 1. 문제 현상
- LIMO 실차 주행 맵핑 중 갑자기 로봇 제어가 중단되고 `odom → base_link` TF 발행이 정지됨.
- 프로세스를 재시작하려 했으나 기존 `/dev/ttyUSB1` 포트가 열리지 않고 디바이스를 찾을 수 없다는 에러 발생.
- `ls /dev/ttyUSB*` 확인 시 포트 번호가 기존 1번에서 2번(`/dev/ttyUSB2`)으로 변경되어 나타남.

### 2. 환경
- **로봇:** AgileX LIMO Pro (온보드 PC: Ubuntu 22.04.5 LTS, ROS 2 Humble)
- **노드 및 장치:** `/limo_base_node`, Silicon Labs CP2102 USB-to-UART 브릿지
- **관련 토픽 및 TF:** `/cmd_vel`, `/wheel/odom`, `odom → base_link`
- **배터리 상태:** LIMO 배터리 인디케이터 2칸 (저전압 상태)

### 3. 원인 분석
- **커널 로그(`dmesg`) 확인 사실:**
  ```text
  [14006.132714] usb 3-7: USB disconnect, device number 7
  [14006.132718] usb 3-7.4: USB disconnect, device number 9
  [14006.132995] cp210x ttyUSB1: failed set request 0x7 status: -19
  [14006.133001] cp210x ttyUSB1: failed set request 0x12 status: -19
  [14006.133476] cp210x ttyUSB1: cp210x converter now disconnected from ttyUSB1
  [14007.080604] usb 3-7.4: cp210x converter now attached to ttyUSB2
  ```
- **원인 후보:** 낮은 배터리 잔량에서 모터 기동과 함께 USB 단절이 기록돼 전원 강하를 의심했다. 전압 파형·USB 전원 상태를 직접 계측하지 않았으므로 Brownout은 확정 원인이 아니다. `ENODEV`는 장치 단절을 뜻하며 단독으로 전원 원인을 입증하지 않는다. 포트 번호 변경 후 드라이버가 이전 핸들을 참조하는 문제도 함께 점검한다.

### 4. 해결 방법
1. 기존 장치를 점유하고 있는 좀비 프로세스 강제 정리:
   ```bash
   sudo fuser -k /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2
   ```
2. 주행 시험 전 배터리 잔량 관리 기준 수립: **배터리 3칸 이상에서만 주행 시험을 수행하고 2칸 이하 시 즉시 충전.**
3. 물리 경로 기반(`by-path`) 또는 고정 포트 런치 인자 적용.

### 5. 검증 방법
- 충전 완료(배터리 4칸) 상태에서 동일 맵핑 경로 주행 시험 수행.
- `dmesg -w` 모니터링을 통해 USB disconnect 이벤트 발생 여부 확인.

### 6. 결과
- **확인 범위:** 충전·포트 관리 후 주행 복구 기록이 있다. 전원 강하는 가설이며 전압 파형 측정·장시간 반복 시험 없이 원인 확정이나 영구 재발 방지를 주장하지 않는다.
- **남아 있는 한계:** 하드웨어 전원부의 물리적 한계이므로, 장시간 연속 주행 시 배터리 모니터링 노드를 통한 소프트웨어적 안전 복귀(Return to Base) 로직 추가 필요.

### 7. 관련 파일
- [`src/limo_ros2/limo_base/src/limo_driver.cpp`](../src/limo_ros2/limo_base/src/limo_driver.cpp)
- [`src/limo_ros2/limo_base/launch/limo_base.launch.py`](../src/limo_ros2/limo_base/launch/limo_base.launch.py)

---

## 사례 2: YDLiDAR `/scan` 토픽의 DDS QoS 불일치 및 RViz 포인트클라우드 미표시

### 1. 문제 현상
- YDLiDAR T-mini Plus 드라이버를 정상 구동하고 `ros2 topic list`에 `/scan`이 확인되나, RViz2에서 LaserScan 디스플레이를 추가했을 때 360도 스캔 점군이 화면에 전혀 렌더링되지 않음.

### 2. 환경
- **패키지 및 노드:** `ydlidar_ros2_driver` (`ydlidar_ros2_driver_node`), RViz2 (`rviz2`)
- **토픽:** `/scan` (`sensor_msgs/msg/LaserScan`)
- **실행 조건:** 로봇 온보드 PC에서 드라이버 구동 후 원격 RViz2로 시각화

### 3. 원인 분석
- **터미널 경고 로그 확인:**
  ```text
  [WARN] [rviz2]: New publisher discovered on topic '/scan', offering incompatible QoS.
  Quality of service risk: BEST_EFFORT vs RELIABLE
  Last incompatible policy: RELIABILITY_QOS_POLICY
  ```
- **원인 규명:**
  - `ydlidar_ros2_driver_node`는 무선 네트워크 대역폭 및 대용량 센서 스트리밍 최적화를 위해 퍼블리셔의 신뢰성 정책을 `BEST_EFFORT`로 설정하여 발행함.
  - 반면 RViz2의 LaserScan 디스플레이 구독자 기본 정책은 `RELIABLE`로 설정되어 있었음.
  - ROS 2의 DDS QoS 호환성 계약 원칙에 따라 Publisher(`BEST_EFFORT`) < Subscriber(`RELIABLE`)인 경우 데이터 전달이 차단됨.

### 4. 해결 방법
- RViz2의 디스플레이 옵션 패널에서 `/scan` 토픽의 `Topic` $\rightarrow$ `Reliability Policy` 설정을 `RELIABLE`에서 **`BEST_EFFORT`**로 변경.
- 또는 RViz 설정 파일(`default.rviz` 등)에 QoS 프로파일 사전 저장.

### 5. 검증 방법
- RViz2 설정 변경 직후 화면 확인 및 `ros2 topic echo /scan --qos-reliability best_effort` 수신 여부 확인.

### 6. 결과
- **확인 범위:** 보고서에 RViz 스캔 표시 복구 기록이 있다. 유효 탐지거리·수신 지연을 이번 정리에서 별도 측정하지 않았다.
- **남아 있는 한계:** 없음. 모든 런치 파일 및 시각화 도구에서 라이다 토픽 구독 시 `BEST_EFFORT` 정책을 명시하도록 표준화함.

### 7. 관련 파일
- [`capstone_ws/src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp`](../capstone_ws/src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp)
- [`capstone_ws/src/limo_capstone/launch/limo_auto_mapping.launch.py`](../capstone_ws/src/limo_capstone/launch/limo_auto_mapping.launch.py)

---

## 사례 3: `setup.py`의 디렉터리 복사 시도로 인한 패키지 빌드 및 지도 배포 실패

### 1. 문제 현상
- `colcon build --packages-select limo_capstone` 실행 시 빌드가 도중에 중단되며 에러 발생.
- 빌드 실패로 인해 새로 취득한 실내 점유격자지도(`MC_3F_10_15.yaml`/`pgm`)가 패키지 설치 디렉터리(`install/share/limo_capstone/maps/`)로 복사되지 않아 내비게이션 런치 실행 시 지도 파일을 찾을 수 없음.

### 2. 환경
- **패키지:** `limo_capstone` (Python 기반 ament_python 패키지)
- **빌드 도구:** `colcon build`, Python `setuptools`

### 3. 원인 분석
- **빌드 에러 로그:**
  ```text
  Starting >>> limo_capstone
  --- stderr: limo_capstone
  error: can't copy 'maps/install': doesn't exist or not a regular file
  ---
  Failed   <<< limo_capstone [0.45s, run with --event-handlers console_direct+ --debug]
  ```
- **원인 규명:**
  - `setup.py` 18행에 `(os.path.join('share', package_name, 'maps'), glob('maps/*'))`로 작성되어 있었음.
  - 이전 작업 과정에서 `maps/` 내에 `install`, `build`, `log` 등의 하위 디렉터리가 생성되었는데, 표준 `glob('maps/*')`는 파일과 디렉터리를 가리지 않고 모두 리스트에 포함함.
  - Python `setuptools`의 `data_files` 복사 루틴은 일반 파일(`regular file`)만 복사할 수 있으므로, 디렉터리 경로(`maps/install`)를 복사하려다 예외를 발생시키며 전체 빌드가 중단됨.

### 4. 해결 방법
- [`capstone_ws/src/limo_capstone/setup.py`](../capstone_ws/src/limo_capstone/setup.py#L16-L22)의 `data_files` 정의에 `os.path.isfile(f)` 리스트 컴프리헨션을 적용하여 정규 파일만 선별 복사하도록 방어적 코드 작성 (Commit `b46aba9`):
  ```python
  # 수정 전:
  (os.path.join('share', package_name, 'maps'), glob('maps/*')),

  # 수정 후:
  (os.path.join('share', package_name, 'maps'), [f for f in glob('maps/*') if os.path.isfile(f)]),
  (os.path.join('share', package_name, 'worlds'), [f for f in glob('worlds/*') if os.path.isfile(f)]),
  (os.path.join('share', package_name, 'scripts'), [f for f in glob('scripts/*') if os.path.isfile(f)]),
  ```

### 5. 검증 방법
- `colcon build --packages-select limo_capstone` 빌드 재실행.
- `ls -l install/share/limo_capstone/maps/` 디렉터리 내 파일 복사 여부 확인.

### 6. 결과
- **해결 여부:** 해결 완료. 0.68초 만에 에러 없이 빌드가 완료되었으며, `MC_3F_10_15.yaml`/`pgm`, `demo_map`, `domi_map` 등 13개의 지도 파일이 정상 설치됨.
- **남아 있는 한계:** `maps/` 하위에 서브폴더 구조(예: `maps/floor3/`)를 생성할 경우 단일 계층 `os.path.isfile`로는 복사되지 않으므로 계층 구조 확장 시 `os.walk` 기반 처리가 필요함.

### 7. 관련 파일
- [`capstone_ws/src/limo_capstone/setup.py`](../capstone_ws/src/limo_capstone/setup.py)

---

## 사례 4: Nav2 Costmap 필수 좌표계 `base_footprint` TF 누락 및 변환 타임아웃

### 1. 문제 현상
- `limo_nav2.launch.py` 또는 `limo_auto_mapping.launch.py` 실행 시 터미널에 지속적으로 TF 타임아웃 경고가 출력됨.
- Costmap이 센서 데이터를 로봇 좌표계에 투영하지 못하고 드롭하여 로봇의 위치가 지도상에서 어긋나거나 경로 계획이 실패함.

### 2. 환경
- **패키지 및 노드:** `nav2_costmap_2d`, `nav2_amcl`, `limo_base`
- **좌표계 체인:** `map` $\rightarrow$ `odom` $\rightarrow$ `base_link` $\rightarrow$ `base_footprint`
- **출력 로그:**
  ```text
  [amcl]: Timed out waiting for transform from base_link to odom to become available...
  [costmap_2d]: Message Filter dropping message: frame 'base_footprint' would require transform to 'map' which is not available
  ```

### 3. 원인 분석
- **원인 규명:**
  1. LIMO의 공식 하드웨어 드라이버인 `limo_driver.cpp`는 `odom → base_link` 변환까지만 브로드캐스트하도록 구현되어 있음.
  2. 그러나 ROS 2 Navigation2(Nav2) 스택과 Costmap2D는 2D 주행 평면에서의 충돌 반경 및 회전 중심을 계산하기 위해 로봇의 지면 투영 좌표계인 `base_footprint` 프레임을 기본 참조 좌표계(`robot_base_frame`)로 요구함.
  3. `base_link → base_footprint` 변환이 시스템상에 존재하지 않아 TF 트리가 단절되었고, 메시지 필터가 센서 데이터를 폐기함.
  4. 아울러 통합 런치 파일 내에서 `explore_node`가 TF 브로드캐스터 노드보다 먼저 기동되어 초기화 단계에서 좌표 조회를 실패하는 순서 결함도 함께 발견됨.

### 4. 해결 방법
1. [`capstone_ws/src/limo_capstone/launch/limo_nav2.launch.py`](../capstone_ws/src/limo_capstone/launch/limo_nav2.launch.py#L35-L42)에 `base_link → base_footprint` (0, 0, 0, 0, 0, 0) 항등 변환을 제공하는 정적 TF 브로드캐스터 노드 추가 (Commit `7439841`):
   ```python
   static_tf = Node(
       package='tf2_ros',
       executable='static_transform_publisher',
       name='static_tf_pub',
       arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
   )
   ```
2. `limo_auto_mapping.launch.py`에서 `LaunchDescription` 노드 등록 순서를 `slam_launch`, `nav2_navigation_launch`, `static_tf`, `explore_node` 순서로 재배치하여 `static_tf`가 탐색 노드보다 앞서 실행되도록 보장.

### 5. 검증 방법
- `ros2 run tf2_ros tf2_echo base_link base_footprint` 실행을 통한 항등 변환(Identity Transform) 확인.
- Nav2 Bringup 콘솔에서 `frame 'base_footprint' would require transform to 'map'` 경고 소멸 여부 확인.

### 6. 결과
- **확인 범위:** 정적 `base_link → base_footprint` 추가와 지도·주행 복구 기록이 있다. 도구의 10000 표시를 실제 10kHz 발행률이나 메시지 드롭 완전 제거 근거로 사용하지 않는다.
- **남아 있는 한계:** 평지 주행(2D) 환경에서는 0 오프셋 항등 변환이 유효하나, 경사로 주행 시에는 피치(Pitch) 각을 반영하는 동적 투영 고려 필요.

### 7. 관련 파일
- [`capstone_ws/src/limo_capstone/launch/limo_nav2.launch.py`](../capstone_ws/src/limo_capstone/launch/limo_nav2.launch.py)
- [`capstone_ws/src/limo_capstone/launch/limo_auto_mapping.launch.py`](../capstone_ws/src/limo_capstone/launch/limo_auto_mapping.launch.py)

---

## 사례 5: Explore Lite 목표 좌표 기준과 반복 목표·조기 종료 조사

### 1. 문제 현상
- 밀폐된 방이나 복도 환경에서 `limo_auto_mapping` 실행 시 로봇이 전진하지 못함.
- 기존 기록의 동일 목표 좌표 `(0.27, 0.03)`에 대해 콘솔에 초당 40회 속도로 `Reached the goal!` 및 `Goal succeeded`가 수천 번 반복 출력되며 탐색률 약 16.8%에서 조기 종료됨 (`docs/DEVELOPMENT_LOG.md` L1268-1308).

### 2. 환경
- **패키지 및 노드:** `m-explore-ros2` (`explore_node`), Nav2 Action Server (`navigate_to_pose`)
- **실행 환경:** Gazebo Fortress 시뮬레이션 및 실물 로봇 환경

### 3. 원인 분석
- **알고리즘 및 수치 분석:**
  1. 360도 라이다가 실내 공간을 스캔하면 방 안쪽은 자유 영역(`FREE`), 벽면 너머는 미지의 영역(`NO_INFORMATION`)으로 인식되어 벽면을 따라 고리(Ring) 형태의 프론티어 셀 군집이 형성됨.
  2. `frontier_search.cpp`의 `buildNewFrontier` 함수는 해당 군집의 모든 셀 좌표를 산술 평균하여 `centroid`를 계산함.
  3. 방 둘레 벽면 셀들의 평균값은 기하학적으로 방 한가운데, 즉 **로봇 바로 발치 앞 빈 공간 `(0.27, 0.03)`**에 위치하게 됨.
  4. `explore.cpp`의 268행이 `target_position = frontier->centroid;`로 목표를 전송함.
  5. 목표가 로봇에 가까워 도착 판정이 반복됐을 가능성을 조사함. 당시 공차의 변경 이력은 자료 간 차이가 있어 0.08m로 확정하지 않으며, 현재 로봇용 YAML의 공차는 0.25m임.
  6. 그러나 실제 미탐색 영역은 2~3m 밖 벽면에 있으므로 프론티어가 전혀 해소되지 않음.
  7. `explore.cpp`의 `reachedGoal()`은 성공 판정 시 목표를 블랙리스트에 넣지 않고 20ms 만에 즉시 다음 계획(`makePlan()`)을 재호출하여, 서 있는 자리에서 무한 성공 루프에 빠짐.

### 4. 해결 방법
- [`capstone_ws/src/m-explore-ros2/explore/src/explore.cpp` 268행](../capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L268)의 목표 좌표 할당 로직을 `frontier->centroid`에서 **`frontier->middle`**로 수정 (Commit `7439841`):
  ```diff
  - geometry_msgs::msg::Point target_position = frontier->centroid;
  + geometry_msgs::msg::Point target_position = frontier->middle;
  ```
- **원리:** `frontier->middle`은 로봇의 현재 위치에서 가장 가까운 실제 프론티어 경계 셀(Closest Boundary Cell)이므로, 로봇을 실제 미탐색 벽면 방향으로 전진시켜 라이다로 미탐색 영역을 확실하게 개척하도록 유도함.
- 아울러 블랙리스트 조회 함수(`goalOnBlacklist`) 호출 시에도 `frontier.middle`을 사용하도록 기준을 통일함.

### 5. 검증 방법
- `colcon build --packages-select explore_lite` 재빌드 후 210초 연속 주행 시험 수행.
- 목표 전송 로그, 로봇 이동 변위, 점유격자지도 셀 통계 수집.

### 6. 결과와 남은 확인
- 목표와 블랙리스트에 `middle`을 사용하는 수정은 소스에서 확인된다. `middle`은 로봇에 가까운 경계 셀이고 평균점이 아니다.
- 과거 개발 기록에 목표 반복 완화와 열린 공간 탐색 개선이 보고돼 있다. 이 저장소 정리에서 해당 시험을 다시 실행한 것은 아니다.
- 목표 기준 수정만으로 모든 조기 종료 원인을 확정하지 않는다. 대기 시간·통로 접근성·잔여 미지 영역을 함께 확인해야 하며 복도 완주와 반복 안정성은 추가 시험이다.

### 7. 관련 파일
- [`capstone_ws/src/m-explore-ros2/explore/src/explore.cpp`](../capstone_ws/src/m-explore-ros2/explore/src/explore.cpp)
- [`capstone_ws/src/m-explore-ros2/explore/src/frontier_search.cpp`](../capstone_ws/src/m-explore-ros2/explore/src/frontier_search.cpp)

---

## 사례 6: Nav2 Lifecycle 비활성화와 과거 지도 샘플 수신 점검

### 1. 문제 현상
- `limo_nav2.launch.py`를 기동하고 RViz2를 열었으나 지도 화면이 비어 있고 `No map received` 상태가 무한 지속됨.
- `ros2 topic echo /map`을 실행해도 아무런 메시지가 출력되지 않음.

### 2. 환경
- **패키지 및 노드:** `nav2_map_server` (`map_server`), `nav2_amcl` (`amcl`), RViz2
- **토픽:** `/map` (`nav_msgs/msg/OccupancyGrid`)

### 3. 원인 분석
- **ROS 2 노드 및 수명주기(Lifecycle) 분석:**
  1. `ros2 lifecycle get /map_server` 및 `ros2 lifecycle get /amcl` 실행 결과, 두 노드가 모두 **`unconfigured [1]`** 상태에 머물러 있었음.
  2. ROS 2 Nav2의 주요 노드들은 `LifecycleNode` 인터페이스를 따르므로, `configure`와 `activate` 전이 단계를 거치지 않으면 내부 퍼블리셔가 활성화되지 않고 대기 상태에 머뭄.
  3. 또한 `nav2_map_server`는 `/map` 토픽을 `Transient Local` (Latched) 내구성으로 발행하는데, RViz2의 Map 디스플레이 기본 설정이 `Volatile`로 되어 있어 신규 구독자가 이전 지도를 수신하지 못함.

### 4. 해결 방법
1. 터미널에서 Lifecycle 전환 명령을 명시적으로 호출하여 노드를 강제 활성화:
   ```bash
   ros2 lifecycle set /map_server configure
   ros2 lifecycle set /map_server activate
   ros2 lifecycle set /amcl configure
   ros2 lifecycle set /amcl activate
   ```
2. RViz2의 Map 디스플레이 설정 내 `Topic` $\rightarrow$ `Durability Policy`를 `Volatile`에서 **`Transient Local`**로 변경.
3. 런치 파일에서 `nav2_lifecycle_manager`의 관리 노드 목록(`node_names: ['map_server', 'amcl', ...]`)과 `autostart: true` 파라미터 검증.

### 5. 검증 방법
- `ros2 lifecycle get /map_server` 결과 `active [3]` 확인.
- RViz2 화면에서 점유격자지도가 즉각 렌더링되는지 확인.

### 6. 결과
- **해결 여부:** 해결 완료. 764×809 해상도의 복도 및 세미나실 지도가 즉시 화면에 표출되고 AMCL 초기 위치 주입이 가능해짐.
- **남아 있는 한계:** 전체 스택 기동 시 AMCL 초기 위치(`/initialpose`)가 주입되기 전까지 Planner Server가 TF 대기로 인해 일시 지연될 수 있으므로 기동 시퀀스 준수 필요.

### 7. 관련 파일
- [`capstone_ws/src/limo_capstone/launch/limo_nav2.launch.py`](../capstone_ws/src/limo_capstone/launch/limo_nav2.launch.py)
- [`capstone_ws/src/limo_capstone/config/nav2_explore_params.yaml`](../capstone_ws/src/limo_capstone/config/nav2_explore_params.yaml)

---

## 사례 7: Gazebo 시뮬레이션 환경과 실차 지도(`MC_3F_10_15`) 간 LiDAR 기하학적 정합 불일치

### 1. 문제 현상
- Gazebo Fortress 시뮬레이터에서 실차 맵핑 지도인 `MC_3F_10_15`를 불러온 후 AMCL과 라이다를 구동했을 때, 시뮬레이션 로봇의 LiDAR 스캔 점군(빨간 점)이 지도의 벽체를 뚫고 나가거나 크게 어긋남.
- AMCL 파티클이 수렴하지 않고 로봇 주변에 넓게 흩어져 위치 추정이 불가능함.

### 2. 환경
- **시뮬레이터:** Gazebo Fortress 6.18.0 (`enclosed_room_fortress.sdf`)
- **실차 지도:** MC관 3층 복도 및 연구실 지도 (`MC_3F_10_15.yaml`/`pgm`, 38.2m × 40.45m)
- **노드:** `amcl`, `ros_gz_bridge`

### 3. 원인 분석 (정량적 수치 근거)
- **공간 형상의 근본적 불일치 분석:**
  1. Gazebo 시뮬레이션 월드는 **5.0m × 5.0m 크기의 단순 사각 밀폐 방**임. 원점 `(0, 0)`의 로봇 라이다는 사방 벽면을 **2.32m ~ 3.47m** 거리에서 관측함.
  2. 반면 `MC_3F_10_15` 지도는 **실제 건물 복도(38.2m × 40.45m)**로, 지도 좌표계 `(0, 0)` 위치는 복도 구석 모서리(가장 가까운 벽면 0.05m, 0.20m, 0.30m)에 위치함.
  3. 5m 사각 방에서 측정한 360개 LiDAR 스캔과 `MC_3F_10_15` 지도 내 장애물 셀 간의 최단 유클리드 거리를 전수 측정한 결과:
     - **평균 오차 (Mean error):** `0.7047 m (70.5 cm)`
     - **최대 오차 (Max error):** `2.4096 m (241.0 cm)`
     - **오차 1.0m 초과 스캔 비율:** `26.67% (96 / 360개)`
     - **10cm 이내 정합 비율:** `17.22%`에 불과함.
  4. AMCL의 Likelihood Field 모델은 센서 관측과 지도의 일치 확률이 극히 낮아 파티클 가중치를 갱신하지 못하고 분산이 발산함.

### 4. 해결 방법
1. **시뮬레이션과 실차 환경의 엄격한 분리:** 5m 사각 방 시뮬레이션에서는 해당 월드를 맵핑한 전용 지도(`phase12s_map.pgm`)를 로드하도록 구성.
2. 실차 복도 지도(`MC_3F_10_15`)를 시뮬레이션에서 검증하기 위해, 2D 점유격자지도를 Gazebo Fortress 3D SDF 월드로 자동 변환하는 스크립트([`map_to_world.py`](../capstone_ws/src/limo_capstone/scripts/map_to_world.py))를 제작하여 `mc3f_10_15.sdf` 월드 모델을 구축함 (Commit `b46aba9`).

### 5. 검증 방법
- `map_to_world.py`로 생성된 `mc3f_10_15.sdf` 월드에서 라이다 스캔과 지도 간의 거리 오차 재측정.

### 6. 결과
- **해결 여부:** 원인 규명 및 해결 완료.
  - 전용 SDF 월드(`mc3f_10_15.sdf`) 로딩 시 라이다 스캔과 지도의 정합 오차가 10cm 이내로 수렴함.
- **남아 있는 한계:** 자동 생성된 SDF 월드는 2D 평면 벽체만을 반영하므로, 실제 건물 복도의 유리창 반사나 천장 구조물 등의 비정형 노이즈는 실물 로봇에서 직접 검증해야 함.

### 7. 관련 파일
- [`capstone_ws/src/limo_capstone/scripts/map_to_world.py`](../capstone_ws/src/limo_capstone/scripts/map_to_world.py)
- [`capstone_ws/src/limo_capstone/worlds/mc3f_10_15.sdf`](../capstone_ws/src/limo_capstone/worlds/mc3f_10_15.sdf)
- [`docs/TROUBLESHOOTING.md`](TROUBLESHOOTING.md)
