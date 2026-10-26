# Capstone1_limo 트러블슈팅 가이드 및 사례 기록

이 문서는 Capstone1_limo 프로젝트의 내비게이션, SLAM, 자율 탐색 및 하드웨어 연동 과정에서 발생한 문제와 해결 과정을 기록합니다. 상세 검증 기준은 [NAV2_TROUBLESHOOTING.md](NAV2_TROUBLESHOOTING.md)를 참조합니다.

---

## 사례 1: 자동 매핑 반복 목표 및 조기 정지 (문제 C)

### 1. 문제 현상
- **발생 조건:** Fortress 시뮬레이션 환경(5m × 5m 방)에서 `limo_auto_mapping_sim.launch.py` 실행 시 발생.
- **현상:** 로봇이 물리적으로 거의 이동하지 않고 동일한 목표 좌표 `(0.27, 0.03)`에 대해 초당 40회 속도로 `Reached the goal!` 및 `Goal succeeded`를 1,000회 이상 무한 반복 출력하며 탐색이 조기 정지됨.

### 2. 원인 정밀 분석
- **측정 및 코드 근거:**
  1. 5m × 5m 방 안에서 360° LiDAR가 방 안쪽 전체를 Free Space로 인식함에 따라, 미지의 영역(`NO_INFORMATION`) 경계 셀들이 방 둘레 4개 벽면에 고리 형태로 형성됨.
  2. `frontier_search.cpp`의 `buildNewFrontier`가 계산하는 `centroid`는 모든 셀 좌표의 산술 평균이므로, 실제 벽면이 아닌 방 한가운데(로봇 발치 앞 `(0.27, 0.03)`)에 위치함.
  3. `explore.cpp` 268행이 `target_position = frontier->centroid;`를 목표로 전송함.
  4. 로봇이 20cm 전진하여 `(0.20, 0.00)`에 도달하면 잔여 거리가 $0.076\,\text{m} < \text{xy\_goal\_tolerance}(0.08\,\text{m})$로 목표 도달 판정이 내려짐.
  5. 그러나 실제 미탐색 셀은 2.5m 밖 벽면에 있으므로 프론티어가 해소되지 않음.
  6. `explore.cpp`의 `reachedGoal()`은 `SUCCEEDED` 시 목표를 블랙리스트에 넣지 않고 20ms 만에 즉시 `makePlan()`을 동기 재호출하므로, 로봇이 서 있는 동일 가상 무게중심에 대해 초고속 무한 성공 루프가 발생함.

### 3. 해결 조치
- **코드 수정:** [`capstone_ws/src/m-explore-ros2/explore/src/explore.cpp` 268행](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L268) 단 1줄 수정:
  ```diff
  - geometry_msgs::msg::Point target_position = frontier->centroid;
  + geometry_msgs::msg::Point target_position = frontier->middle;
  ```
- **원리:** `frontier->middle`은 로봇에서 가장 가까운 실제 프론티어 격자 셀(Closest point on frontier, 벽면 방향 약 2~2.5m)이므로, 로봇을 실제 미탐색 영역으로 이동시켜 센서로 프론티어를 실질적으로 해소시킴.
- **빌드 명령:** `colcon build --packages-select explore_lite` (9.48초 소요)

### 4. 210초 장시간 검증 결과
- **검증 환경:** Fortress 시뮬레이션, 210초(3분 30초) 연속 주행.
- **측정 사실:**
  1. 목표 반복 횟수: **0회** (초고속 무한 루프 100% 소멸).
  2. 목표 좌표 순차 전이: `(-0.41, -0.46)` $\rightarrow$ `(0.78, -0.61)` $\rightarrow$ `(-1.17, 1.31)`.
  3. 로봇 이동: 초기 `(0.00, 0.00)`에서 최종 `(0.222, -0.995)`로 **1m 이상 횡단 주행**.
  4. 지도 셀 통계 변화:
     - 미탐색(UNKNOWN) 셀: 초기 **7,832개 (83.2%)** $\rightarrow$ 최종 **1,755개 (15.0%)** (68.2%p 대폭 감소).
     - 자유(FREE) 공간 셀: 초기 **1,569개 (16.7%)** $\rightarrow$ 최종 **8,823개 (75.6%)** (4.5배 증가).
     - 장애물(OCCUPIED) 셀: 초기 **8개** $\rightarrow$ 최종 **1,086개** (4개 외벽 매핑 완료).
  5. 탐색 완료 감지: 모든 프론티어가 해소된 후 `No frontiers found, stopping.` 로그와 함께 자동 정상 종료.
  6. Progress Checker 및 Recovery: 1차 목표 선회 중 `Failed to make progress` 6회 발생했으나, `backup` 복구 동작이 1회 성공 완료된 후 정상 주행 속행됨.

### 5. 한계 및 향후 과제
- **실물 로봇 (미검증):** 실물 LIMO Pro 온보드 환경에서의 모터 슬립 및 센서 특성 검증 필요.
- **원점 복귀 (미검증):** `explore_lite`의 `return_to_init` 기능은 구현되어 있으나 기본값(`false`) 상태로 이번 실험에서는 미동작함.
- **Progress Checker 튜닝:** 제자리 회전 시의 경고 방지를 위해 `movement_time_allowance`를 10초에서 15~20초로 완화하는 방안 검토 필요.

---

## 사례 2: MC_3F_10_15 지도 로딩 실패 원인 분석 및 단계별 검증

### 1. 문제 현상
- **발생 조건:** 시뮬레이션 및 Nav2 실행 환경에서 미래관 3층 실차 매핑 지도(`MC_3F_10_15.yaml`, `MC_3F_10_15.pgm`)를 불러올 때 발생.
- **현상:** 지도가 정상적으로 화면이나 Nav2 스택에 로드되지 않거나, 실행 초기에 라이프사이클 노드들이 멈추며 지도가 표시되지 않음.

### 2. 단계별 검증 및 원인 분석
- **1단계 (파일 유효성 및 포맷):**
  - `MC_3F_10_15.pgm`: 764 × 809 cells, P5 매직 넘버, 618,076 픽셀 데이터 정상 확인 (점유 1.66%, 미지 84.86%, 자유 13.47%).
  - `MC_3F_10_15.yaml`: `image: MC_3F_10_15.pgm`, 해상도 0.05m, 원점 `[-13.7, -30.8, 0]`, trinary 모드 정상 확인. 소스(`src/`)와 설치(`install/`) 디렉터리 파일의 MD5 체크섬 100% 일치 (`ed490592...`).
- **2단계 (빌드 및 패키지 설치 실패):**
  - **최초 실패 지점:** `capstone_ws/src/limo_capstone/setup.py` 18행의 `(os.path.join('share', package_name, 'maps'), glob('maps/*'))`.
  - `maps/` 내부에 임시 빌드 디렉터리(`maps/build`, `maps/install`, `maps/log`)가 존재하여 `colcon build` 시 `error: can't copy 'maps/install': doesn't exist or not a regular file` 오류로 패키지 전체 설치가 중단됨.
  - 이로 인해 `install/limo_capstone/share/limo_capstone/maps/` 디렉터리에 지도가 배포되지 않아 launch 파일이 지도를 찾지 못함.
- **3단계 (Map Server 라이프사이클 전이 및 QoS):**
  - `nav2_map_server`는 LifecycleNode이므로 단순 프로세스 실행만으로는 지도를 로드하지 않으며, `configure` 및 `activate` 전이를 거쳐야만 지도를 메모리에 로드함.
  - 또한 `/map` 토픽은 `TRANSIENT_LOCAL` (Latched) 내구성으로 발행되므로, 구독자가 `VOLATILE` QoS를 사용할 경우 기동 후 발행된 지도를 수신하지 못함.
- **4단계 (Nav2 통합 기동 시 라이프사이클 교착):**
  - Nav2 Bringup 실행 시 `lifecycle_manager_navigation`이 `planner_server`를 활성화할 때 `global_costmap`이 `base_link -> map` TF를 확인하는 과정에서 대기함.
  - AMCL은 초기 포즈(`/initialpose`)를 수신하기 전까지 `map -> odom` TF를 브로드캐스트하지 않으므로, `/initialpose`가 선행 주입되지 않으면 전체 네비게이션 스택이 활성화 도중 멈춤.

### 3. 해결 조치
1. `setup.py` 18행 수정: `[f for f in glob('maps/*') if os.path.isfile(f)]` 필터를 적용하여 일반 파일만 배포하도록 수정.
2. `limo_nav2_sim.launch.py` 및 검증 스크립트에서 AMCL 활성화 직후 `/initialpose`를 선행 공급하여 `global_costmap`의 TF 대기를 해소.

### 4. 검증 결과
- `nav2_map_server` 단독 로딩 검증: `unconfigured` -> `inactive` -> `active [3]` 전이 성공, `/map` 토픽 (width: 764, height: 809, origin: [-13.7, -30.8]) 수신 성공.
- 시뮬레이션 Nav2 전체 스택 로딩 검증: 6개 노드(`map_server`, `amcl`, `controller_server`, `planner_server`, `bt_navigator`, `velocity_smoother`) 전원 `ACTIVE` 진입 성공.

---

## 사례 3: 시뮬레이션 환경과 실차 지도(MC_3F_10_15) 간 LiDAR 정합 불일치 원인 규명

### 1. 문제 현상
- **발생 조건:** Gazebo Fortress 시뮬레이션 환경에서 `MC_3F_10_15` 지도를 로드하고 AMCL 및 LiDAR 스캔을 구동할 때 발생.
- **현상:** RViz 상에서 로봇의 LiDAR 스캔 점군(빨간 점)이 `MC_3F_10_15` 지도의 벽체와 일치하지 않고 벽을 뚫고 나가거나 크게 어긋나며, AMCL 파티클이 수렴하지 않고 높은 위치 분산을 유지함.

### 2. 원인 정밀 분석 (수치 및 기하학적 근거)
1. **물리적 환경의 근본적 불일치 (가장 핵심적인 원인):**
   - **Gazebo 시뮬레이션 월드:** `enclosed_room_fortress.sdf`는 **5.0m × 5.0m 크기의 사각 밀폐 방**임. 원점 `(0, 0)`에 배치된 로봇의 LiDAR는 상하좌우 4개 벽면을 **약 2.32m ~ 3.47m 거리**에서 관측함.
   - **MC_3F_10_15 지도:** 실제 LIMO 실차로 측정한 **미래관 3층 복도 및 연구실 구역(38.2m × 40.45m)**임. 지도 좌표계 `(0, 0)` 위치는 복도 구석 모서리 부근으로, 가장 가까운 벽면이 동쪽 **0.05m**, 남쪽 **0.20m**, 서쪽 **0.30m**에 위치함.
   - **불일치 실측 결과:**
     - 5m × 5m Gazebo 방에서 취득된 360개 LiDAR 스캔 점군과 `MC_3F_10_15` 지도 내 장애물 셀 간의 최단 유클리드 거리를 전수 조사한 결과:
       - **평균 오차 (Mean error):** `0.7047 m (70.5 cm)`
       - **중앙값 오차 (Median error):** `0.5409 m (54.1 cm)`
       - **최대 오차 (Max error):** `2.4096 m (241.0 cm)`
       - **오차 1.0m 초과 스캔 비율:** `26.67% (96 / 360개)`
       - **10cm 이내 정합 비율:** `17.22%`에 불과.
     - 반면, 동일 5m × 5m 방을 매핑한 `phase12s_map.pgm`과의 정합 오차는 평균 `0.1505 m (15.1 cm)`(외벽 SDF 두께 0.15m 반영치)로 완벽히 정합됨.
2. **AMCL 파티클 필터 갱신 실패:**
   - AMCL의 `Likelihood Field Model`은 센서 스캔 빔이 지도 상의 장애물 셀과 일치할 확률을 계산하여 가중치를 부여함.
   - Gazebo LiDAR가 관측하는 5m × 5m 박스 형태는 `MC_3F_10_15` 지도 어디에도 물리적으로 존재하지 않으므로, 모든 파티클의 관측 우도(Likelihood)가 0에 가깝게 계산됨.
   - 이로 인해 AMCL 공분산이 `cov_xx = 0.2391, cov_yy = 0.2355`에서 전혀 줄어들지 못하고 수렴이 차단됨 (대조군: 5m 방 맵에서는 공분산이 70.7% 급감하며 1.25cm로 정밀 수렴함).
3. **TF 및 센서 프레임 설정 검증 (가설 배제):**
   - 시뮬레이션 TF 체인: `map -> odom -> base_footprint -> base_link -> laser_link`가 모두 유효하며 단절 없음.
   - LiDAR 장착 오프셋: `base_link` 기준 `x = +0.103m, z = -0.034m`로 URDF 및 Gazebo SDF 센서 위치와 일치함.
   - 따라서 LiDAR 정합 불일치는 TF 단절이나 드라이버 버그가 아닌, **시뮬레이션 월드(5m 방)와 지도 환경(미래관 3층)의 물리적 불일치**에서 비롯된 것임.

### 3. 해결 방향 및 한계 명시
- 시뮬레이션 환경에서 `MC_3F_10_15` 지도와의 정합을 검증하려면, Gazebo 내에 미래관 3층의 실내 벽체 구조를 모델링한 전용 SDF 월드를 구성해야 함.
- 현재의 5m × 5m 가상 방 환경(`enclosed_room_fortress.sdf`)에서는 해당 방을 직접 매핑한 전용 정적 지도(`phase12s_map.yaml` 또는 `limo_fortress_map.yaml`)를 사용하는 것이 정상적인 검증 절차임.
- `MC_3F_10_15` 지도의 실질적인 정합 검증은 실제 미래관 3층 현장에서 LIMO Pro 실차를 구동하여 수행해야 함.

---

## 사례 4: MC_3F_10_15 전용 Gazebo SDF 월드 자동 생성 및 LiDAR 정합 검증

### 1. 해결 목적 및 배경
- 5m × 5m 사각 방 시뮬레이터와 38m 미래관 3층 지도 간의 물리적 공간 불일치(사례 3)를 극복하기 위해, 지도 점유 격자(`MC_3F_10_15.yaml/.pgm`)로부터 1:1 좌표계가 완벽히 일치하는 Gazebo Fortress 전용 월드(`mc3f_10_15.sdf`)를 자동 생성하고 시뮬레이션 정합을 재검증함.

### 2. 구현 및 조치 사항
1. **월드 자동 생성기 구현 ([`map_to_world.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/scripts/map_to_world.py)):**
   - 2D 점유 픽셀(10,272개)을 수평 우선 탐욕 병합(greedy merging) 알고리즘으로 3,732개의 충돌 박스로 압축 생성.
   - 지도 좌표계의 원점 `[-13.7, -30.8]` 및 해상도 0.05m를 그대로 반영하여 Gazebo world 좌표계와 ROS 2 `map` 프레임이 1:1로 정확히 일치하도록 구성.
   - 동일 입력에 대해 100% 비트 단위 동일한 재현성 확인.
2. **전용 런치 파일 구축 ([`limo_mc3f_sim.launch.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/launch/limo_mc3f_sim.launch.py)):**
   - 복도 내 안전 스폰 위치 `(-1.07, -0.68)`, 복도 방향 헤딩 `yaw = 2.23 rad` (128°) 지정.
   - 로봇 모델 및 브리지 파라미터 연동.
3. **`free_thresh` 임계치 영향 규명 및 시뮬레이션 사본 적용:**
   - 원본 YAML의 `free_thresh = 0.25`는 미지 셀(205)의 확률(0.196)을 자유 공간(0)으로 오인식하여 524,520개 미지 셀이 모두 0으로 변환됨을 규명.
   - 실차 원본은 보존하고 시뮬레이션 사본(`MC_3F_10_15_sim.yaml`)에 `free_thresh = 0.15`를 적용하여 자유 셀 83,284개(13.47%), 미지 셀 524,520개(84.86%)가 정상 분리되도록 조치.

### 3. 정량적 검증 결과
1. **LiDAR 정합 오차 획기적 개선 (347개 유효 빔 전수 측정):**
   - **평균 오차 (Mean error):** 기존 `0.7047 m (70.5 cm)` $\rightarrow$ **`0.0296 m (2.96 cm)` (95.8% 개선)**
   - **중앙값 오차 (Median error):** 기존 `0.5409 m (54.1 cm)` $\rightarrow$ **`0.0000 m (0.00 cm)`**
   - **최대 오차 (Max error):** 기존 `2.4096 m (241.0 cm)` $\rightarrow$ **`0.1414 m (14.14 cm)`**
   - **10cm 이내 정상 정합 비율:** 기존 `17.22%` $\rightarrow$ **`99.14%` (잠정 기준 80.0% 대폭 초과 달성)**
   - **20cm 이내 정상 정합 비율:** **`100.00%`**
   - **50cm 초과 이상치 비율:** 기존 `26.67%` $\rightarrow$ **`0.00%` (완전 소멸)**
2. **Nav2 Costmap 및 TF 직접 확인:**
   - Global Costmap: 764×809 크기, 장애물(100) 10,373셀, 인플레이션(1-99) 47,795셀 정상 갱신 확인.
   - Local Costmap: 60×60 크기, 장애물 144셀, 인플레이션 1,363셀 정상 갱신 확인.
   - TF `map -> laser_link`: `(-1.120, -0.556, 0.116)` 정상 해석.
3. **복도 내 주행 테스트:**
   - Goal 1 (단거리 1.0m NW): 로봇 복도 따라 순 변위 `0.832 m` 실제 이동, 최종 AMCL 오차 `0.235 m`.
   - Goal 2 (중거리 2.0m NW): 추가 순 변위 `0.977 m` 이동 (누적 `1.809 m` 주행), 최종 AMCL 오차 `0.477 m`.

### 4. 한계 및 실차 검증 필요성
- 본 검증은 2D 점유 지도 격자에서 생성된 world와의 기하학적 정합을 검증한 것이며, **실제 미래관 3층 건물의 물리적 벽체(벽 높이, 재질, 유리창, 실제 가구)와의 일치도를 증명하는 것은 아님.**
- 실제 건물과의 최종 정합은 미래관 3층 현장에서 LIMO Pro 실차로 검증해야 함.

---

## 사례 5: Nav2 시험 하네스 오류 분석 및 단거리 목표(Goal 1) 재검증 (Phase 15-B/C)

### 1. 문제 현상
- **발생 조건:** `mc3f_10_15.sdf` 월드에서 Nav2 복도 주행 시험 실행 시 발생.
- **현상:** 두 시험 목표 모두 Nav2 액션 상태 `6` (`STATUS_ABORTED`)으로 종료되었고, Odom 기준 오차가 각각 2.51m 및 4.16m로 크게 보고됨.

### 2. 원인 정밀 분석
1. **시험 하네스의 액션 상태 강제 대입 오류 (Phase 15-B):**
   - 검증 스크립트(`phase15_full_sim_verification.py` 422행)에서 호스트 시간 기준 35초 경과 시 `gh.cancel_goal_async()`를 호출하면서 `status = GoalStatus.STATUS_ABORTED (6)`를 직접 대입하는 하드코딩 오류가 있었음.
   - Nav2 내부 로그(`phase15_nav2.log`) 확인 결과, Nav2 자체 알고리즘이 실패(Abort)한 것이 아니라 외부 시험 스크립트의 취소 요청에 의해 주행이 취소(`STATUS_CANCELED`)된 것이었음.
2. **Odom-Map 좌표계 직접 감산에 의한 수학적 허위 오차 (Phase 15-B):**
   - 스크립트 444행에서 `math.hypot(end_odom['x'] - g['x'], end_odom['y'] - g['y'])` 수식을 사용하여, 스폰 시 `(0, 0)`으로 초기화되는 `odom` 프레임 좌표와 `map` 프레임 목표 좌표를 TF 변환 없이 직접 뺄셈함.
   - 이로 인해 2.51m, 4.16m라는 비정상적인 허위 오차가 계산되었음이 밝혀짐.

### 3. 해결 조치 (하네스 수정)
1. **타임아웃 안전 기준 및 상태 처리 정상화:**
   - 호스트 안전 제한 시간을 65초로 현실화하고, 임의의 상태 대입을 제거하여 Nav2 액션 서버가 반환하는 실제 Result 상태를 그대로 수신하도록 수정함.
2. **오차 계산 및 물리 주행량 분리 측정:**
   - Odom은 차체 휠 회전에 따른 순수 물리 주행 거리(`net_motion`) 계산용으로만 사용하고, 목표 오차는 Map 프레임 기준 TF(`map -> base_link`) 및 AMCL 토픽으로 분리 산출하도록 수정함.

### 4. 재검증 결과 (Goal 1 단독, Phase 15-C)
1. **Nav2 액션 정상 완료:**
   - Nav2 액션 서버가 **`STATUS_SUCCEEDED` (4)**를 반환하며 자연스럽게 주행 완료 (`Reached the goal!`).
   - 외부 타임아웃 발생 없음 (`timeout_triggered: False`).
   - 소요 시간: 호스트 시간 52.90초, 시뮬레이션 시간(`/clock`) 6.74초 (평균 RTF 0.127x).
2. **정량적 측정 지표:**
   - **순 물리 주행량 (Odom):** **`0.9970 m`** (목표 직선 거리 `0.9980 m` 대비 **99.9%** 이동).
   - **TF 최종 위치 오차 (`map -> base_link`, stamp: 7.480s):** **`0.1401 m (14.01 cm)`**, Yaw 오차 **`6.81°`** (허용 각도 14.3° 이내).
   - **AMCL 최종 위치 오차 (stamp: 5.800s):** **`0.2784 m (27.84 cm)`**, Yaw 오차 **`2.80°`** (허용 각도 14.3° 이내).
   - **Nav2 Controller 잔여 거리 피드백:** `0.117 m`.

### 5. 성공 판정 및 한계 명시
- **위치 허용 오차 구분:**
  - Nav2 내부 Controller의 goal checker는 목표 도달을 판정(`STATUS_SUCCEEDED`)했으나, TF 기준 최종 오차(`14.01 cm`)는 설정된 `xy_goal_tolerance` (`8.0 cm`)보다 약 6cm 컸음.
  - 따라서 Nav2 액션 성공과 독립적인 8cm 위치 정확도 충족을 동일시하지 않고 구별하여 기록함.
- **AMCL vs TF 측정 시각 불일치 (가설 표기):**
  - AMCL 측정 시각(`5.800 s`)과 TF 측정 시각(`7.480 s`)이 서로 다르므로 두 오차를 동일 시각 측정치로 직접 비교할 수 없음.
  - AMCL 파티클 갱신이 종료 약 1.6초 전에 멈춘 현상은 `update_min_d: 0.25` 설정으로 인해 마지막 극저속 접근 구간의 이동량이 25cm에 미달하여 갱신이 발생하지 않았을 가능성이 가설로 제기됨 (추후 추가 입증 필요).
- **Goal 2 미검증:**
  - Phase 15-C에서는 Goal 1만 재검증하였으며, **Goal 2는 아직 성공 여부가 검증되지 않은 미검증 상태**임.
- **물리적 환경 검증 한계:**
  - 본 월드는 2D 점유 지도 격자에서 생성된 기하학적 시뮬레이션 모델이며, 실제 건물의 재질, 유리창 투과, 실제 문 개폐 상태와의 일치성을 증명하지는 않음. 실물 LIMO Pro를 이용한 현장 검증이 필수적임.
