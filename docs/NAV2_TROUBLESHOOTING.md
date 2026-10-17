# Nav2 자율주행 검증

## 목적

Nav2가 지도 위에서 목표점으로 이동하는 과정을 센서 입력, TF, localization, costmap, planner, controller, 속도 명령으로 나누어 검증한다. 현재 문서는 검증 계획과 확인 기준을 정리한 것이며, 아래 항목을 모두 구현 또는 실기기 검증했다는 뜻은 아니다.

## 데이터 및 제어 흐름

```text
/scan + TF + map + localization
              ↓
         Nav2 Costmaps
              ↓
        Global Planner
              ↓
        Local Controller
              ↓
           /cmd_vel
              ↓
           LIMO Base
```

## 문제를 분리하는 기준

| 관찰 현상 | 먼저 확인할 항목 | 확인 방법 |
|---|---|---|
| 목표를 보내도 경로가 만들어지지 않음 | map, localization, TF, costmap | RViz에서 지도·로봇 pose·costmap을 확인하고 Nav2 로그를 수집 |
| 경로는 있지만 로봇이 움직이지 않음 | controller 상태, velocity command, LIMO Base | Nav2 action 상태와 `/cmd_vel`, Base 노드 상태를 각각 확인 |
| 장애물 앞에서 경로가 끊김 | 장애물 관측, sensor TF, costmap layer/inflation | `/scan`과 costmap에 장애물이 반영되는지 비교 |
| 로봇 위치가 지도와 어긋남 | map–odom–base TF, localization | TF와 localization pose를 확인하고 초기 pose를 다시 검증 |
| 목표를 반복해서 취소하거나 재전송함 | action 상태, planner/controller 결과, recovery, goal tolerance | 시간순 로그와 action 상태를 수집해 최초 실패 지점을 찾음 |

표는 진단 방향을 제시하며 특정 원인이 현재 문제의 원인이라고 단정하지 않는다.

## 검증 순서

1. **입력:** `/scan`, odometry, TF가 지속적으로 발행되는지 확인한다.
2. **지도 및 localization:** 지도 파일을 로드하고 로봇 pose가 지도와 일치하는지 확인한다.
3. **Costmap:** RViz에서 global/local costmap이 올바른 위치에 장애물을 표시하는지 확인한다.
4. **Global planning:** 정지 상태에서 안전한 목표점을 지정하고 경로 생성 여부를 확인한다.
5. **Controller:** 생성된 경로를 따라가는 동안 controller 상태와 `/cmd_vel`을 확인한다.
6. **장애물 반응:** 정적 장애물부터 시험하고, 동적 장애물은 안전한 통제 환경에서 별도로 검증한다.
7. **성능 기록:** 목표 도착 여부, 실패 유형, 소요 시간, 재시도 횟수와 환경을 기록한다.

## 현재 상태

- SLAM과 지도 저장은 [SLAM 및 지도 작성](SLAM_MAPPING.md)에 기록된 범위까지만 확인되었다.
- Nav2의 목표점 도착, 안정적인 경로 추종, 장애물 회피 성공률은 이 문서를 작성하는 시점에 증거가 확보되지 않았으므로 **미검증**으로 둔다.
- 실기기 테스트 전에는 속도 제한과 비상 정지 방법을 확인하고, 로봇 주변을 통제한다.

## 실험 결과 기록 템플릿

### 문제
무슨 현상이 어떤 조건에서 발생했는가?

### 원인 분석
로그·토픽·TF·costmap 등으로 확인된 사실은 무엇이며, 원인 가설은 무엇인가?

### 해결 조치
변경한 파라미터나 코드, 실행 명령은 무엇인가?

### 검증
동일 조건에서 문제가 재현되는가? 변경 후 무엇이 달라졌는가?

### 한계 및 다음 작업
아직 확인하지 못한 조건과 다음 실험을 기록한다.

---

## 사례 1: 자동 매핑 반복 목표 및 조기 정지 (문제 C)

### 문제
- **발생 조건:** Fortress 시뮬레이션 환경(5m × 5m 방)에서 `limo_auto_mapping_sim.launch.py` 실행 시 발생.
- **현상:** 로봇이 물리적으로 거의 이동하지 않고 동일한 목표 좌표 `(0.27, 0.03)`에 대해 초당 40회 속도로 `Reached the goal!` 및 `Goal succeeded`를 1,000회 이상 무한 반복 출력하며 탐색이 멈춤.

### 원인 분석
- **측정 및 코드 근거:**
  1. 5m × 5m 방 안에서 360° LiDAR가 방 안쪽 전체를 Free Space로 인식함에 따라, 미지의 영역(`NO_INFORMATION`) 경계 셀들이 방 둘레 4개 벽면에 고리 형태로 형성됨.
  2. `frontier_search.cpp`의 `buildNewFrontier`가 계산하는 `centroid`는 모든 셀 좌표의 산술 평균이므로, 실제 벽면이 아닌 방 한가운데(로봇 발치 앞 `(0.27, 0.03)`)에 위치함.
  3. `explore.cpp` 268행이 `target_position = frontier->centroid;`를 목표로 전송함.
  4. 로봇이 20cm 전진하여 `(0.20, 0.00)`에 도달하면 잔여 거리가 $0.076\,\text{m} < \text{xy\_goal\_tolerance}(0.08\,\text{m})$로 목표 도달 판정이 내려짐.
  5. 그러나 실제 미탐색 셀은 2.5m 밖 벽면에 있으므로 프론티어가 해소되지 않음.
  6. `explore.cpp`의 `reachedGoal()`은 `SUCCEEDED` 시 목표를 블랙리스트에 넣지 않고 20ms 만에 즉시 `makePlan()`을 동기 재호출하므로, 로봇이 서 있는 동일 가상 무게중심에 대해 초고속 무한 성공 루프가 발생함.

### 해결 조치
- **코드 수정:** [`capstone_ws/src/m-explore-ros2/explore/src/explore.cpp` 268행](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L268) 단 1줄 수정:
  ```diff
  - geometry_msgs::msg::Point target_position = frontier->centroid;
  + geometry_msgs::msg::Point target_position = frontier->middle;
  ```
- **원리:** `frontier->middle`은 로봇에서 가장 가까운 실제 프론티어 격자 셀(Closest point on frontier, 벽면 방향 약 2~2.5m)이므로, 로봇을 실제 미탐색 영역으로 이동시켜 센서로 프론티어를 실질적으로 해소시킴.
- **빌드 명령:** `colcon build --packages-select explore_lite` (9.48초 소요)

### 검증
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

### 한계 및 다음 작업
- **실물 로봇 (미검증):** 실물 LIMO Pro 온보드 환경에서의 모터 슬립 및 센서 특성 검증 필요.
- **원점 복귀 (미검증):** `explore_lite`의 `return_to_init` 기능은 구현되어 있으나 기본값(`false`) 상태로 이번 실험에서는 미동작함.
- **Progress Checker 튜닝:** 제자리 회전 시의 경고 방지를 위해 `movement_time_allowance`를 10초에서 15~20초로 완화하는 방안 검토 필요.

---

## 사례 2: Ackermann 시뮬레이션 정체와 Explore 블랙리스트 검증 (Phase 12)

### 문제
- Fortress Ackermann 시뮬레이터에서 기존 DWB(`dwb_core::DWBLocalPlanner`) 사용 시 제자리 회전 명령(`linear.x = 0.0, angular.z != 0.0`)이 발행되어 최소 회전 반경 제약이 있는 Ackermann 조향 모델이 전진하지 못하고 `Failed to make progress` 및 `BackUp`/`Spin` 복구 루프에 빠지는 현상이 있었음.
- 또한 [`explore.cpp`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp)에서 목표 전송(`target_position = frontier->middle`, [`explore.cpp:268`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L268)) 및 블랙리스트 저장([`explore.cpp:284`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L284), [`explore.cpp:392`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L392))은 `frontier.middle`을 사용하는 반면, 블랙리스트 조회(`visualizeFrontiers()` [`explore.cpp:196`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L196), `makePlan()` [`explore.cpp:258`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L258))는 `frontier.centroid`를 참조하고 있어 타임아웃 시 블랙리스트에 등록된 프론티어가 필터링되지 않고 재선택될 위험이 있었음.

### 원인 분석
1. **조향 기구학 불일치:** DWB 설정(`min_speed_xy: 0.0`)이 제자리 회전을 허용하여 Ackermann 조향 기구학과 충돌함.
2. **블랙리스트 좌표 기준 불일치:** 큰 프론티어(`n=823` 셀 등)에서는 가장 가까운 점인 `frontier.middle`(예: `(-0.409, -0.457)`)과 전체 평균점인 `frontier.centroid`(예: `(0.272, 0.028)`) 사이의 좌표 차이가 `goalOnBlacklist()`의 허용 오차(`tolerance = 5 * costmap_resolution = 0.25 m`)를 초과($|\Delta x| = 0.681\,\text{m}$, $|\Delta y| = 0.485\,\text{m}$)하므로, `middle`을 블랙리스트에 저장하고 `centroid`로 조회하면 동일 프론티어를 블랙리스트로 인식하지 못함.

### 해결 조치
1. **시뮬레이션 전용 RPP 설정 분리:** 실차용 설정(`nav2_params.yaml`, `nav2_explore_params.yaml`)을 보존한 채 시뮬레이션 전용 [`nav2_sim_params.yaml`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/config/nav2_sim_params.yaml), [`limo_nav2_sim.launch.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/launch/limo_nav2_sim.launch.py), [`limo_auto_mapping_sim.launch.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/launch/limo_auto_mapping_sim.launch.py)를 구성하고 `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`(`use_rotate_to_heading: false`, `allow_reversing: false`)를 적용함.
2. **블랙리스트 조회 좌표 통일 (Phase 12-L):** [`explore.cpp`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp)의 [`visualizeFrontiers()` (196행)](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L196)와 [`makePlan()` (258행)](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L258)에서 `goalOnBlacklist` 인자를 `frontier.middle` / `f.middle`로 통일하고 `explore_lite`를 재빌드함.

### 검증
1. **정상 주행 검증 (Phase 12-N):**
   - RPP + `frontier.middle` 조합으로 네 개 목표(`(-0.41, -0.46)` $\rightarrow$ `(1.03, 0.89)` $\rightarrow$ `(-1.24, -1.69)` $\rightarrow$ `(1.66, 1.56)`)가 순차 전송되었고, 새 목표에 의한 선점 전이(`Received goal preemption request`) 중 누적 이동 거리 **`2.1797 m`** (순 변위 **`2.0594 m`**)의 탐색 주행이 계속됨.
   - 네 번째 목표 활성화 상태에서 모든 프론티어가 해소되어 목표 취소(`Goal was canceled`)와 함께 `No frontiers found, stopping.`으로 정상 종료됨.
   - 주행 중 순수 제자리 회전 **0회**, `Failed to make progress` **0회**, `BackUp`/`Spin` 복구 **0회**를 기록함.
2. **블랙리스트 타임아웃 분기 검증 (Phase 12-O):**
   - 임시 설정(`progress_timeout: 3.0`, `velocity_smoother` 속도 `0.0` 고정)으로 무이동 정체를 유도한 결과, 첫 목표 수락(`sim_t = 9.167 s`) 후 `sim_t = 12.988 s`(경과 `3.821 s`)에 `Adding current goal to black list` 분기가 실행됨을 확인함.
   - 직후 `/explore/frontiers` 마커 색상이 `BLUE`(`seq=5`)에서 **`RED`(`seq=6`, `sim_t = 13.013 s`)**로 전환되고, 해당 프론티어(`middle = (-0.409, -0.457)`, `centroid = (0.272, 0.028)`)가 `goalOnBlacklist(f.middle)`에 의해 필터링되어 재선택 없이 `All frontiers traversed/tried out, stopping.`으로 정상 종료됨.
3. **다중 프론티어 환경에서 블랙리스트 제외 및 차순위 목표 전환 검증 (Phase 12-P1/P2):**
   - **초기 다중 프론티어 분할 검출 (Phase 12-P1):** 임시 SDF 월드(`phase12p_multi_frontier.sdf`)에서 시작 시점에 북쪽 프론티어 A(`332 cells`, `middle = (-0.409, 0.543)`, `BLUE`)와 남쪽 프론티어 B(`322 cells`, `middle = (-0.359, -0.507)`, `BLUE`) 2개가 독립 검출되고 두 `middle` 간 y좌표 차이가 `1.050 m`($> 0.25\,\text{m}$)임을 확인함.
   - **A 타임아웃 후 B 전환 및 후보 소진 종료 (Phase 12-P2):** 목표 A 수락(`sim_t = 9.443 s`) 후 `3.009 s` 경과(`sim_t = 12.452 s`) 시점에 A가 블랙리스트에 등록되자, 마커 A만 **`RED`**로 전환되고 B는 **`BLUE`**를 유지한 채 즉시 차순위 목표 B(`(-0.36, -0.51)`)가 전송·수락(`sim_t = 12.465 s`)됨. 이후 A는 재선택되지 않았고, `last_progress_` 갱신에 의해 B가 약 `3.980 s` 유지된 뒤(`sim_t = 16.445 s`) B도 블랙리스트에 등록되면서 A·B 모두 **`RED`**(`seq=10`, `sim_t = 16.468 s`)가 되어 `All frontiers traversed/tried out, stopping.` $\rightarrow$ `Exploration stopped.`로 종료됨.
   - **정지 및 종료 진단 시 주의사항:**
     - `/cmd_vel_nav`(컨트롤러 출력: A 구간 `vx = 0.1677 m/s, wz = +0.5778 rad/s`, B 구간 `vx = 0.1719 m/s, wz = -0.5778 rad/s`)와 최종 로봇 구동 입력인 `/cmd_vel`을 반드시 구분해야 함. Phase 12-O/P1/P2에서는 `velocity_smoother` 속도 제한(`0.0`)으로 인해 `/cmd_vel`은 전 구간 `0.000000`, `/odom` 이동 거리는 `0.000000 m`였음.
     - `All frontiers traversed/tried out, stopping.`(모든 후보 프론티어가 블랙리스트 등으로 소진되어 중단됨)과 `No frontiers found, stopping.`(맵 내 프론티어가 실제로 모두 해소되어 탐색 완료됨)은 서로 다른 종료 원인이므로 로그 분석 시 구분해야 함.

### 한계 및 다음 작업
- **개별 목표 도달 성공 (미검증):** 로그에서 개별 목표의 `SUCCEEDED`는 확인되지 않았고 선점된 목표들의 최종 액션 `ResultCode`도 기록되지 않았음. 따라서 Phase 12-N은 탐색 중 주행과 정상적인 탐색 종료를 검증한 것이며 개별 목표 도달 성공을 검증한 것은 아님.
- **`makePlan()` 타임아웃 분기 및 A $\rightarrow$ B 전환 알고리즘만 검증됨 (실제 주행 전환은 미검증):** Phase 12-O와 Phase 12-P1/P2를 통해 `makePlan()` 내부의 `progress_timeout` 블랙리스트 등록, `goalOnBlacklist(f.middle)` 필터링, 그리고 다중 프론티어 환경에서 차순위 프론티어(A $\rightarrow$ B)로 목표를 전환하는 알고리즘 흐름은 검증되었으나, 속도 제한을 해제한 상태에서 목표 B로 실제 이동 및 도달(`SUCCEEDED`)하는 통합 주행은 아직 검증되지 않았음.
- **Nav2 `ABORTED` 콜백 블랙리스트 분기 (미검증):** `reachedGoal()` 콜백 내 `ResultCode::ABORTED` 수신에 의한 블랙리스트 등록 경로([`explore.cpp:392`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L392))는 이번 런타임 테스트에서 발생하지 않아 미검증 상태임.
- **실물 LIMO 로봇 (미검증):** 이번 검증은 모두 Fortress 시뮬레이터에서 수행되었으며 실물 LIMO 하드웨어에서는 검증되지 않았음.
- **탐색 종료 시 `bt_navigator` halt 로그 관측 (원인 별도 미검증):** Phase 12-N, 12-O, 12-P2에서 탐색 종료(`stop(true)`)로 활성 목표를 취소할 때 `bt_navigator`에서 `[ERROR] [bt_navigator]: Failed to get result for follow_path in node halt!`가 1회 출력된 직후 `[INFO] [bt_navigator]: Goal canceled`로 종료되는 현상이 관측됨. 종료 절차 자체는 정상 완료되었으나, 해당 에러 로그의 내부 원인은 별도로 검증하지 않았으므로 무조건 무해하다고 단정하지 않고 관측 사실로 기록함.
- **후속 우선순위 과제:** (1) 맵 파일 로딩/저장 경로 및 포맷 검증, (2) 정지·종료 원인 진단 로그 및 확인 절차 정리, (3) 빌드/런타임 반영 여부 사전 점검 절차 정리, (4) 속도 제한 해제 상태에서 A $\rightarrow$ B 전환 후 실제 주행 테스트, (5) 진도 보고서 및 중간 발표 PPT용 결과·근거 표 정리.
