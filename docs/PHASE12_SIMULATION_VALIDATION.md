# Phase 12: LIMO 자율 탐색 및 Nav2 시뮬레이션 검증 보고서 (Phase 12-T ~ 12-V)

## 1. 개요 및 목적

본 문서는 AgileX LIMO 로봇의 ROS 2 Humble 기반 실내 자율 주행 및 자율 매핑(Autonomous Exploration) 프로젝트에서, 시뮬레이션 환경(Gazebo Fortress)을 활용해 수행한 **Phase 12-T(지도 품질 확인), Phase 12-U(지도 재로딩 검증), Phase 12-V(주행 명령 원인 분석)**의 통합 검증 결과와 분석 내역을 기록한다.

본 검증의 목적은 다음과 같다:
1. **지도 품질 확인 (Phase 12-T):** Phase 12-S에서 저장된 `phase12s_map.pgm` 및 `.yaml`이 실내 환경의 4면 벽체와 자유 공간을 폐루프 형태로 충실히 표현하고 있는지 기하학적·정량적으로 검증.
2. **지도 재로딩 검증 (Phase 12-U):** 생성된 지도가 Nav2 `map_server` 단독 실행 환경에서 정적 지도로 오류 없이 로드 및 활성화(Lifecycle Activate)되는지 검증.
3. **주행 명령 원인 분석 (Phase 12-V):** 이전 실험(Phase 12-O/P)에서 `/cmd_vel_nav`가 생성되었음에도 `/cmd_vel`이 0이었던 원인과, Phase 12-R/S에서 정상 주행이 이루어진 원인을 설정 및 제어 파이프라인 관점에서 명확히 규명.

---

## 2. 실험 환경 및 격리 조건

- **저장소 위치:** `/home/cjinw/school/2026-2/Capstone1_limo`
- **기준 커밋:** `54699d05a7dcb918cc2b5644dc4bf647df0dcd18`
- **OS / ROS 2:** Ubuntu 22.04.5 LTS / ROS 2 Humble
- **시뮬레이터:** Ignition Gazebo Fortress 6.16.0 (`enclosed_room_fortress.sdf`, 5m × 5m 방)
- **차량 모델:** AgileX LIMO Ackermann 조향 모델 (`libignition-gazebo-ackermann-steering-system.so`)
- **DDS 통신 격리:**
  - `ROS_DOMAIN_ID=44` (Phase 12-S 자율 매핑), `ROS_DOMAIN_ID=47` (Phase 12-U 맵 로딩)
  - `ROS_LOCALHOST_ONLY=1`
  - `FASTRTPS_DEFAULT_PROFILES_FILE`에 `fastdds_udp_only.xml` 강제 적용 (UDPv4 단독 루프백, 공유 메모리 충돌 차단)
- **산출물 보관 경로:**
  `/home/cjinw/.gemini/antigravity/brain/0e39003d-5210-4560-840d-5c8e6dd6055f/scratch/`

---

## 3. Phase 12-T: 지도 품질 정량 검증 결과

Phase 12-S에서 저장된 `phase12s_map.pgm` (100×100 점유 격자, 해상도 0.05m/pixel)을 이미지 픽셀 레벨에서 전수 조사한 결과입니다.

### 1) 지도 메타데이터 정합성
- **이미지 크기:** 100 × 100 픽셀 (가로 5.00 m × 세로 5.00 m)
- **해상도:** `0.050000 m/cell`
- **원점 (Origin):** `[-2.500, -2.510, 0.000]`
- **물리 공간 일치성:** 원점 $(-2.50, -2.51)$에서 $100 \times 0.05\text{m}$를 전개하면 $X \in [-2.50, +2.50]$, $Y \in [-2.51, +2.49]$가 되어, 시뮬레이터 월드(`enclosed_room_fortress.sdf`)의 내부 5m × 5m 공간과 정확히 일치함을 확인.

### 2) 래스터 픽셀 분포 및 기하학적 형상
```text
총 격자 수: 10,000 셀 (100 x 100)
- 자유 공간 (Pixel 254, Free):      9,052 픽셀 (90.52%)
- 점유 벽체 (Pixel   0, Occupied):    645 픽셀 ( 6.45%)
- 미지 영역 (Pixel 205, Unknown):     303 픽셀 ( 3.03%)
```

- **4면 외벽 연속성 및 밀폐도 (Wall Continuity & Enclosure):**
  - Gazebo SDF 벽체 두께(0.15m = 3격자) 및 미세 원점 오프셋으로 인해 외벽은 2~3격자 두께의 밴드(Band) 형태로 형성됨:
    - **상단 외벽 밴드 (Row 0~2)**: 열 2 ~ 99 전체(98개 열)에 걸쳐 점유 격자 누락 없이 100% 연속 연결됨 (`top_cols: [2..99]`, 결손 열 0개).
    - **하단 외벽 밴드 (Row 96~99)**: 열 1 ~ 99 전체(99개 열)에 걸쳐 점유 격자 누락 없이 100% 연속 연결됨 (`bot_cols: [1..99]`, 결손 열 0개, Row 97 중심).
    - **좌측 외벽 밴드 (Col 0~3)**: 행 0 ~ 97 전체(98개 행)에 걸쳐 점유 격자 누락 없이 100% 연속 연결됨 (`left_rows: [0..97]`, 결손 행 0개, Col 2 중심).
    - **우측 외벽 밴드 (Col 97~99)**: 행 0 ~ 98 전체(99개 행)에 걸쳐 점유 격자 누락 없이 100% 연속 연결됨 (`right_rows: [0..98]`, 결손 행 0개, Col 99 중심).
    - 네 외벽 밴드는 모서리 교차 영역(Row 0~2/Col 2~3, Row 0~2/Col 97~99, Row 96~98/Col 1~3, Row 96~98/Col 97~99)에서 상호 중첩되어 틈새가 없음.
  - **플러드 필(Flood Fill) 누출 검증**:
    - 실내 정중앙 셀인 `(Row 50, Col 50)`에서 점유 격자(0)를 통과하지 않는 4방향 탐색 수행 결과, 실내 자유 공간이 이미지 외곽 경계(Row 0, 99 / Col 0, 99)로 누출되는 셀 수: **0개 (Complete Enclosure 입증)**.
- **실내 공간 완전성 및 셀 좌표 분포:**
  - 외벽 안쪽 내부 영역(벽체 경계 3픽셀 제외, Rows 3..96 / Cols 3..96, 총 8,836셀)에서 자유 공간 비율은 **99.55%** (8,796셀).
  - 실내 한가운데 부유하는 가상 점유 노이즈는 **0건**.
  - 벽체 안쪽 경계에 나타난 27개 점유 셀은 Gazebo SDF 벽체 두께(0.15m = 3픽셀)를 정확히 반영한 두께 격자임.
  - 내부 영역에 위치한 미지 격자 13셀(0.15%)의 실제 좌표 및 벽체와의 거리 실측:
    - 상단-우측 대각 선상 6셀: `(Row 3, Col 96)` ~ `(Row 8, Col 91)` (최단 외벽 거리 0.05m ~ 0.30m)
    - 좌측 경계 인근 1셀: `(Row 19, Col 5)` (최단 외벽 거리 0.14m)
    - 하단-좌측 대각 선상 5셀: `(Row 80~81, Col 16~15)`, `(Row 86~88, Col 9~7)` (최단 외벽 거리 0.25m ~ 0.70m)
    - 하단-좌측 벽면 인근 1셀: `(Row 91, Col 3)` (최단 외벽 거리 0.05m)
    - 전체 13셀 중 10셀은 외벽과 0.30m 이내(0.05~0.30m)로 인접하며, 나머지 3셀은 0.30m 초과 0.71m 이내에 위치함. 실내 중심부 개활 공간에는 미지 격자가 전혀 존재하지 않음을 확인.

---

## 4. Phase 12-U: Map Server 단독 재로딩 검증 결과

> [!NOTE]
> 본 Phase 12-U 검증은 `nav2_map_server` 단독 노드의 지도 파일(`.yaml`/`.pgm`) 파싱, Lifecycle 상태 전이 및 `/map` 토픽 브로드캐스트의 무결성을 검증한 것입니다. 저장된 정적 지도를 기반으로 AMCL 위치추정 및 Nav2 전체 스택(Global/Local Costmap, Planner, Controller)을 기동하는 통합 자율 주행 시험은 별도의 차기 검증 단계로 명확히 구분합니다.

`phase12s_map.yaml`과 `phase12s_map.pgm`을 `nav2_map_server` 단독 노드로 로드하여 정적 맵 서비스 인터페이스를 검증함.

### 1) 실행 명령 및 Lifecycle 전이
```bash
export ROS_DOMAIN_ID=47
export ROS_LOCALHOST_ONLY=1
ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:=<경로>/phase12s_map.yaml \
    -p use_sim_time:=false
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```
- **Lifecycle 전이 결과:** `unconfigured` $\rightarrow$ `inactive` $\rightarrow$ `active` (성공, 오류 0건)
- **상대 경로 참조:** YAML 내 `image: phase12s_map.pgm`이 Map Server에 의해 동일 디렉터리 내 PGM 파일로 정상 해석됨.

### 2) `/map` 토픽 및 GetMap 서비스 검증
- **QoS:** `Reliability: RELIABLE`, `Durability: TRANSIENT_LOCAL`
- **토픽 메타데이터 (`OccupancyGrid.info`):**
  - `width: 100`, `height: 100`, `resolution: 0.05`
  - `origin: (-2.500, -2.510, 0.000)`
- **점유 격자 데이터 통계 (`OccupancyGrid.data`):**
  - 점유 셀 (`data == 100`): **645개 (6.45%)** (PGM의 0 픽셀과 100% 일치)
  - 자유 셀 (`data == 0`): **9,355개 (93.55%)**
  - 미지 셀 (`data == -1`): **0개 (0.00%)**
- **Trinary 모드 임계값 해석 규명:**
  - `phase12s_map.yaml`의 `free_thresh`는 `0.25`로 설정됨.
  - PGM의 Unknown 색상(205)은 점유율 계산 공식 $p = (255 - 205)/255 = 0.1961$을 따름.
  - $0.1961 \le 0.25$이므로, Map Server의 Trinary 모드에서 $p \le free\_thresh$ 조건이 성립하여 303개의 205 픽셀이 자유 공간(0)으로 분류됨 ($9,052 + 303 = 9,355$).
  - 이는 저장소 내 기존 표준 맵인 `MC_3F_10_15`에서도 동일하게 관측된 정상 동작임.
- **GetMap 서비스:** `/map_server/map` 서비스 호출 결과 100×100 격자 정상 응답 확인.

---

## 5. Phase 12-V: 이전 주행 명령 문제 원인 분석

### 1) 관찰 현상 및 비교 데이터
| 시험 단계 | 목표 특성 | `/cmd_vel_nav` (Nav2 출력) | `velocity_smoother` 파라미터 | 최종 `/cmd_vel` | 누적 이동 거리 | 탐색 종료 사유 |
| :--- | :--- | :---: | :---: | :---: | :---: | :--- |
| **Phase 10** | DWB (제자리 회전) | $v_x = 0, \omega_z \ne 0$ | 정상 (0.26 m/s) | $v_x = 0, \omega_z \ne 0$ | **0.0000 m** | `Failed to make progress` (정체) |
| **Phase 12-O** | 단일 프론티어 타임아웃 | $v_x = 0.19, \omega_z = -0.58$ | **`max_vel: [0.0, 0.0, 0.0]`** | **전 구간 0.000000** | **0.0000 m** | `All frontiers traversed...` (블랙리스트) |
| **Phase 12-P2**| 다중 목표 A $\rightarrow$ B 전환 | $v_x = 0.17, \omega_z = \pm 0.58$ | **`max_vel: [0.0, 0.0, 0.0]`** | **전 구간 0.000000** | **0.0000 m** | `All frontiers traversed...` (블랙리스트) |
| **Phase 12-R** | 자율 탐색 완주 진단 | $v_x \le 0.26, \omega_z = \pm 0.58$ | **`max_vel: [0.26, 0.0, 1.0]`** | **최대 0.26 m/s 실발행** | **2.8373 m** | `No frontiers found...` (탐색 완주) |
| **Phase 12-S** | 완주 및 지도 저장 | $v_x \le 0.26, \omega_z = \pm 0.58$ | **`max_vel: [0.26, 0.0, 1.0]`** | **최대 0.26 m/s 실발행** | **2.1499 m** | `No frontiers found...` (탐색 완주) |

### 2) 확정된 근본 원인 (Root Cause Analysis)
1. **Phase 10 정체의 원인 (기구학 불일치):**
   - DWB 컨트롤러가 Ackermann 조향 모델에 대해 차동 구동 방식의 순수 제자리 회전 명령($v_x = 0, \omega_z \ne 0$)을 발행하여 Gazebo 상에서 토크가 전달되지 않아 발생함. 이는 RPP(`use_rotate_to_heading: false`) 도입으로 해결됨.
2. **Phase 12-O/P에서 `/cmd_vel = 0`이었던 원인 (의도된 스트레스 파라미터 적용):**
   - 당시 시험 목적은 "로봇이 움직이지 못할 때 `progress_timeout`이 발생하여 목표를 블랙리스트에 넣고 차순위 목표로 전환하는가"를 검증하는 것이었음.
   - 이를 위해 시험 스크립트(`phase12o_verify.sh`, `phase12p_transition_verify.sh`)에 전용 설정 파일(`phase12o_nav2_sim_params.yaml`, `phase12p_nav2_sim_params.yaml`)을 인자로 넘겼으며, 해당 파일의 `velocity_smoother`에 **`max_velocity: [0.0, 0.0, 0.0]`**, **`min_velocity: [0.0, 0.0, 0.0]`**이 명시적으로 주입되어 있었음.
   - 따라서 RPP가 `/cmd_vel_nav`로 $0.17 \sim 0.19\text{m/s}$를 출력했음에도 불구하고, 후단의 `velocity_smoother`가 모든 속도를 0으로 제한하여 최종 `/cmd_vel`이 0으로 출력된 것임 (버그나 통신 단절이 아닌 설계된 시험 조건).
3. **Phase 12-R/S에서 정상 주행이 이루어진 원인 (표준 설정 복원):**
   - Phase 12-R/S에서는 파라미터 오버라이드 없이 프로젝트 표준 설정인 `capstone_ws/src/limo_capstone/config/nav2_sim_params.yaml`(`max_velocity: [0.26, 0.0, 1.0]`)을 온전히 적용함에 따라 `velocity_smoother`가 제어 명령을 그대로 통과시켜 Gazebo 시뮬레이션 상의 로봇 이동(2.15~2.84m) 및 맵 확장이 정상 수행됨.

---

## 6. 확인된 사실, 추정, 미검증 항목

### [확인된 사실]
1. `phase12s_map`은 5m × 5m 실내 폐공간을 100×100 격자, 90.52% 자유 공간, 4면 연속 벽체로 충실하게 표현함.
2. `nav2_map_server` 단독 실행 시 상대 경로를 통해 PGM을 오류 없이 로드하고 Lifecycle 활성화 및 `/map` 발행을 완료함.
3. Phase 12-O/P의 `/cmd_vel = 0` 현상은 `velocity_smoother`의 `[0, 0, 0]` 임시 시험 설정 때문이었으며, 표준 설정(`0.26 m/s`) 하에서는 RPP-Smoother-Gazebo 구동 체인이 정상 작동함.

### [미검증 항목 및 실물 LIMO 이식 시 주의사항]
1. **실물 LIMO Pro 하드웨어 구동 (미검증):**
   - Gazebo Fortress는 완벽한 평면과 이상적인 모터 토크를 가정하므로, 실차의 바퀴 슬립, 배터리 전압 강하, 노면 마찰력에 따른 거동 오차는 실기기에서 검증해야 함.
2. **LiDAR 센서 노이즈 (미검증):**
   - 시뮬레이션 환경의 완벽한 360° 광선 반사와 달리 실차 YDLiDAR T-mini Plus는 유리창, 검은색 흡수체, 먼지 등에 의한 결측/노이즈가 발생할 수 있음.
3. **복도 및 대형 환경 매핑 (미검증):**
   - 5m × 5m 단일 방을 벗어나 긴 복도나 문턱을 지나는 다중 룸 환경에서의 장거리 SLAM 루프 클로징 성능은 추가 검증 필요.
4. **정적 지도 기반 Nav2 전체 내비게이션 주행 (미검증):**
   - Phase 12-U에서는 `nav2_map_server` 단독 노드의 맵 발행 무결성까지만 검증함.
   - 저장된 정적 지도(`phase12s_map.yaml`)를 불러와 AMCL 위치추정을 수행하고 목표 지점(Goal Pose)으로 실제 이동하는 Nav2 전체 스택 주행 검증은 시뮬레이션 및 실기기에서 추후 수행 필요.

---

## 7. 실제 LIMO 온보드 테스트 체크리스트 (차기 실기기 시험 대비)

1. **사전 하드웨어 통신 점검:**
   - [ ] LIMO Base 시리얼 포트 연결 확인 (`/dev/ttyTHS1` 또는 USB 포트).
   - [ ] YDLiDAR 드라이버 기동 및 `/scan` 토픽 주기 확인 (10 Hz 이상).
   - [ ] 하드웨어 비상 정지(E-Stop) 스위치 및 원격 조종기(RC) 수동 개입 준비.
2. **소프트웨어 파라미터 점검:**
   - [ ] `nav2_params.yaml`의 `velocity_smoother` 속도 제한이 실차 안전 속도(`max_velocity: [0.20, 0.0, 0.8]`)로 설정되었는지 확인.
   - [ ] `slam_params.yaml`의 `use_sim_time: false` 확인.
3. **자율 매핑 및 저장 점검:**
   - [ ] `ros2 launch limo_capstone limo_auto_mapping.launch.py` 기동.
   - [ ] 탐색 완료 시 `ros2 run nav2_map_server map_saver_cli -f <저장경로>`를 통한 실물 지도 파일 생성 여부 확인.
