# Nav2 안정화 및 7개 시나리오 순찰 E2E 검증 보고서 (2026-11-24)

## 1. 작업 개요 및 격리 경계

- **기준 커밋**: `15bb5c59b806f58d95e19fb6308effd02c15f0de` (`15bb5c59`)
- **작업 Worktree / 브랜치**: `/home/cjinw/school/2026-2/Capstone1_limo_nav_validation_20261124` (`feat/nav2-stabilization-patrol-20261124`)
- **격리 원칙 준수**:
  - 실차 LIMO 접속·배포·주행 없음 (전원 OFF 상태).
  - 운영 서버 `18190` / `8080` 무접촉.
  - 타 worktree(`Capstone1_limo`, `Capstone1_limo_g2_20261123`, `Capstone1_limo_web_ui_20261123` 미커밋 10개 파일) 무수정 보존.
  - Claude Code 담당 파일(`fleet_control/server/*`, `fleet_control/agent/*`) 무수정, 통합이 필요한 사항은 6절 패치 제안으로 분리.
  - ROS 2 DDS 격리: `ROS_LOCALHOST_ONLY=1`, Kinematic 테스트는 도메인 `161..166`(포트 `29701..29713`), Gazebo 테스트는 미사용 도메인 `205` + 전용 `IGN_PARTITION` 사용.

---

## 2. 기존 `n03`, `n05`, `f08` 간헐 실패 근본 원인 분석

원본 로그(`r8_nav/result.json`, `r8_nav/stack/stack.0.log`, `r8_nav/timeline.jsonl`, `p9h_nav_e2e.log`, 기존 `14 PASS / 4 FAIL`)를 분석한 결과, 3가지 연쇄 원인이 확인되었다.

### 원인 A (주원인 — `n03`, `n05`, `f08` 공통): `use_rotate_to_heading: false`로 인한 벽면 근접 정지 및 시작 footprint 치명 비용(Lethal Cost) 진입 (Defect D3)
- 기존 `integrated_nav2_lidar_only_slow_params.yaml`은 `FollowPath.use_rotate_to_heading: false`로 설정되어 있었다.
- `p01` 웹 매핑 마지막 프론티어 목표(`(-1.58, 1.76) -> (-1.03, 0.52)`) 수행 중, 목표가 로봇 후방에 위치할 때 RPP가 제자리 회전 대신 `regulated_linear_scaling_min_speed = 0.10 m/s`로 전진 호를 그리며 내부 장애물 박스(`x ∈ [-1.6, -0.9], y ∈ [0.8, 1.5]`) 쪽으로 접근했고, `PAUSE` 취소 시점에 `truth = (-1.253, 1.631)`, `amcl = (-1.228, 1.590)`에 정지했다.
- 이 위치는 박스 북쪽 벽면(`row 19, y = 1.53`)으로부터 불과 **0.05 m (1셀)** 거리였다.
- 이후 `INITIAL_POSE`는 중심 셀 `(35, 18)`만 확인(`clearance = 0.0`)하므로 통과했지만, 실제 `robot_radius = 0.18 m` (`inflation_radius = 0.28 m`) footprint가 벽면 점유 셀과 겹쳐 `global_costmap`과 `local_costmap`에서 시작 위치가 치명 비용 상태가 되었다.
- 결과적으로 `n03`, `n05`, `f08`에서 `NavfnPlanner`가 `GridBased: failed to create plan with tolerance 0.30`을 14회 연속 실패하고, `behavior_server`의 `Spin`·`BackUp` 복구도 `Collision Ahead`로 중단되어 로봇이 전혀 이동하지 못한 채 `ABORTED (FAILED)`로 종료되었다.
  - 참고: `n03` 로그에 보인 `109`개 경로 포인트는 `n03`의 새 경로가 아니라 `p01` 취소 직전 `LocalizationWorker.path`에 캐시되어 남아 있던 잔여 `/plan`이었다.

### 원인 B (`f08` 복구 지연): `f04` `SIGSTOP`/`SIGCONT` 이후 Nav2 `lifecycle_manager` Bond Timeout 연쇄 리셋
- `f04`에서 `planner_server`를 4초 이상 `SIGSTOP`하면 `lifecycle_manager_navigation`이 bond heartbeat timeout으로 Nav2 전체 노드를 비활성화·재구성한다.
- `r8_nav`에서는 `f04` 종료 후 Nav2 재활성화 완료를 기다리지 않고 곧바로 후속 시나리오로 진입했고, `f08` 시점(`1795428893`–`1795428920`)에도 `controller_server` bond timeout이 겹쳐 23초간 `/navigate_to_pose/_action/status` 발행자가 사라지며 `_locked_mock_recovery`가 5회 연속 `409 idle_not_proven`을 반환했다. 이후 복구 자체는 성공했으나 최종 검증 미션이 원인 A의 치명 시작 위치 `(-1.37, 1.58)` 때문에 실패했다.

### 원인 C (`f07` → `f08` 경합): 비인가 `/cmd_vel_raw` 퍼블리셔 해제 직후 DDS 전파 지연
- `f07_bypass_attempts` 종료 직전 테스트 노드가 비인가 `/cmd_vel_raw` 퍼블리셔를 `destroy_publisher`한 직후 대기 없이 `f08`의 `gate_guard`가 실행되면서, DDS 그래프에서 퍼블리셔가 제거되기 전에 Gate가 `cmd_input:NOT_EXCLUSIVE`를 재래치하여 `MotionWatchdog`가 `LOCKED`에 머무르는 경합이 발생했다.

---

## 3. 수정 파일 목록 (본 Worktree 내부)

| 파일 | 변경 내용 | 목적 |
|---|---|---|
| `capstone_ws/src/limo_capstone/config/integrated_nav2_lidar_only_slow_params.yaml` | `FollowPath.use_rotate_to_heading: true`, `rotate_to_heading_min_angle: 0.785` 적용; `progress_checker`를 `SimpleProgressChecker`에서 `PoseProgressChecker`(`required_movement_radius: 0.12`, `required_movement_angle: 0.5`, `movement_time_allowance: 5.0`)로 변경 | 후방 목표 시 벽면 돌진(D3) 제거 및 5초 이상 제자리 회전 시 오탐 abort 방지 |
| `capstone_ws/src/limo_capstone/limo_capstone/lidar_scan_matcher_odom_node.py` | `keyframe_threshold`(기본 `0.12`), `trim_percentile`(기본 `85.0`)을 ROS 파라미터로 노출 | 기존 Kinematic 기준선 동작을 보존하면서 Gazebo D1 튜닝(`0.30`/`100.0`) 주입 가능화 |
| `fleet_control/tests/mode_e2e/stack_up.sh` | Git worktree 환경에서 `limo_msgs` fallback 경로 추가, 기본 `GATE_SCRIPT`를 `candidate_r13` 및 watchdog 연동 옵션으로 설정, C103 아티팩트 부재 시 합성 facility/floor/area 자동 시드 | 독립 worktree에서 Phase 3 및 7시나리오 하네스 재현성 확보 |
| `fleet_control/tests/nav_e2e/run_nav_e2e.py` | `f04` 이후 `nav_idle` 및 `safety_context_reason is None` 대기 추가; `f07` 퍼블리셔 해제 후 DDS 정착 대기(`0.5s`/`1.5s`) 및 `f08` 미션 전 blocker 해제 확인 추가 | 원인 B(bond timeout 리셋 중 진입) 및 원인 C(DDS 해제 경합) 제거 |
| `fleet_control/tests/nav_e2e/run_nav_patrol_7scenarios_20261124.py` (신규) | 7개 요구 시나리오 + `no_cmd_vel_endpoints` 자동 검증 및 정량 지표(오차, CTE, 곡률, 취소 정지 거리, 속도 제한 비교) 수집 하네스 | 반복 검증 및 정량 근거 자동 기록 |
| `fleet_control/tests/gazebo_web/nav2_d3_targeted.py` | Git worktree에서 `office_limo.world`, `candidate_r13`, truth localization fixture의 본 저장소 fallback 경로 추가 | 독립 worktree에서 Gazebo 실물리 주행 검증 실행 |

---

## 4. 검증 결과 요약 (합성 Kinematic vs 실제 Gazebo 물리 주행 분리)

### 4.1 합성(Kinematic 시뮬레이터) — Phase 3 `run_nav_e2e.py` 반복 검증
| 실행 ID | 맵 모드 | 도메인 / 포트 | 결과 | `n03` | `n05` | `f08` | 로그 경로 |
|---|---|---|---|---|---|---|---|
| `phase3_o6_rep1` | 고정맵 (`phase3_fixed_series#792b410f9cdc`) | 195 / 29601 | **18/18 PASS** | PASS | PASS | PASS | `/tmp/nav_validation_20261124/phase3_o6_rep1/result.json` |
| `phase3_o6_rep2` | 고정맵 (`phase3_fixed_series#792b410f9cdc`) | 162 / 29702 | **18/18 PASS** | PASS | PASS | PASS | `/tmp/nav_validation_20261124/phase3_o6_rep2/result.json` |
| `phase3_o6_rep3` | 고정맵 (`phase3_fixed_series#792b410f9cdc`) | 163 / 29703 | **18/18 PASS** | PASS | PASS | PASS | `/tmp/nav_validation_20261124/phase3_o6_rep3/result.json` |
| `phase3_live_map3` | **실시간 웹 SLAM 매핑 (`p01` 포함)** (`web_135a08f605f0_179e9b#5f8407ebfa47`) | 161 / 29701 | **18/18 PASS** | PASS | PASS | PASS | `/tmp/nav_validation_20261124/phase3_live_map3/result.json` |
| **합계** | **4회 반복** | — | **72/72 PASS (0 FAIL)** | 4/4 | 4/4 | 4/4 | — |

- `phase3_live_map3`에서 `p01` 종료 좌표는 `(-0.612, 1.671)`로 벽면 안전 이격을 유지했으며, 이어진 `n03`, `n05`, `f08` 모두 1차 시도에서 `SUCCEEDED`를 달성했다.

### 4.2 합성(Kinematic 시뮬레이터) — 7개 시나리오 순찰 및 속도 제한 E2E (3회 반복)
- 실행 경로: `/tmp/nav_validation_20261124/patrol_7scen_rep1` (도메인 164, 포트 29711), `patrol_7scen_rep2` (도메인 165, 포트 29712), `patrol_7scen_rep3` (도메인 166, 포트 29713)
- **총 결과**: **24/24 PASS (0 FAIL, 0 SKIP)** (`8/8 PASS × 3회`), 모든 실행에서 `/cmd_vel` 엔드포인트 0개(`no_cmd_vel_endpoints: PASS`), 비정상 취소·재시작 0건.

| 시나리오 | 반복 | 통과 | Nav2 결과 | 대표 정량 지표 (`rep1 / rep2 / rep3`) |
|---|---|---|---|---|
| **1. 단일 목적지 이동** (`s01`: `(0,0) -> (1.5,-1.0)`) | 3 | 3/3 | `SUCCEEDED` | 소요 `13.39 / 13.14 / 13.13 s`, 이동거리 `1.746 / 1.742 / 1.736 m`, 목표 오차 `dxy = 0.0791 / 0.0940 / 0.0984 m`, 평균 CTE `0.0399 / 0.0396 / 0.0377 m` (최대 `0.0736 / 0.0710 / 0.0695 m`) |
| **2. 연속 목적지 이동** (`s02`: 2연속 + 모드전환 취소 + 재로컬라이즈 + 3번째 목표) | 3 | 3/3 | `SUCCEEDED` ×3, 전환 `CANCELED` | `goal1 dxy = 0.1534 / 0.1793 / 0.1434 m`, `goal2 dxy = 0.0601 / 0.0646 / 0.0420 m`, 모드전환 취소 `CANCELED`, 재로컬라이즈 `CONFIRMED`, `goal3 dxy = 0.1177 / 0.1079 / 0.1033 m` |
| **3. 복수 경유지 순찰** (`s03`: 4개 웨이포인트 순환) | 3 | 3/3 | 4/4 `SUCCEEDED` ×3 | 총 거리 `7.267 / 7.130 / 7.270 m`, 총 시간 `67.34 / 73.07 / 67.37 s`, 구간 최대 위치 오차 `0.1702 / 0.1565 / 0.1362 m` |
| **4. 중간 취소와 정지** (`s04`) | 3 | 3/3 | `CANCELED` → 후속 `SUCCEEDED` | 0.5초 내 0속도 달성 `True` (3/3), 취소 후 제동 이동거리 `0.000 m` (3/3), Arbiter `NO_OWNED_GOAL`, 후속 목적지 `SUCCEEDED` |
| **5. 장애물 경로 실패** (`s05`) | 3 | 3/3 | 정적 거부 `409`, Nav2 차단 `FAILED` | 박스 목표 `FOOTPRINT_OCCUPIED`(HTTP 409 거부), 벽 목표 `OCCUPIED`(HTTP 409 거부), 동적/Nav2 차단 목표 `FAILED`, 실패 후 0속도 유지 `True` (3/3) |
| **6. Agent 재시작 이후 임무 재조정** (`s06`) | 3 | 3/3 | `RECONCILIATION_REQUIRED` → `CANCELED` → 복구 후 `SUCCEEDED` | `boot_id` 변경 확인, 재시작 0.5초 내 0속도 `True`, 구 세션 재사용 HTTP 409 거부, 복구 후 목적지 `SUCCEEDED` (`dxy = 0.1054 / 0.1093 / 0.1195 m`, `17.77 / 18.03 / 17.76 s`) |
| **7. 속도 제한 vs 경로 추종 안정성 비교** (`s07`: `(-0.5,-1.0) -> (1.5,1.0)` 대각 곡선) | 3 | 3/3 | 3개 프로파일 모두 `SUCCEEDED` | 아래 4.2.1 상세표 참조 |

#### 4.2.1 시나리오 7 속도 제한 프로파일별 정량 비교 (`rep1 / rep2 / rep3`)
| 프로파일 | 속도 상한 (`vx / wz`) | 소요 시간 (s) | 실제 이동거리 (m) | 목표 오차 `dxy` (m) | 평균 / 최대 CTE (m) | 최대 명령 `vx / wz` | 최대 곡률 `\|wz/vx\|` (`1/m`) |
|---|---|---|---|---|---|---|---|
| **Baseline** | `0.15 m/s / 0.50 rad/s` | `25.40 / 25.41 / 25.13` | `2.794 / 2.795 / 2.803` | `0.1236 / 0.0669 / 0.0992` | `0.0645 / 0.1084` (`rep1`)<br>`0.0643 / 0.1231` (`rep2`)<br>`0.0739 / 0.1035` (`rep3`) | `0.15 / 0.352` (`rep1`)<br>`0.15 / 0.300` (`rep2`)<br>`0.15 / 0.346` (`rep3`) | `3.5192 / 2.9822 / 3.4615` |
| **Proportional Clamp** | `0.06 m/s / 0.20 rad/s` | `57.15 / 64.22 / 58.47` | `2.739 / 2.790 / 2.772` | `0.1219 / 0.0872 / 0.0916` | `0.0575 / 0.1193` (`rep1`)<br>`0.0368 / 0.0883` (`rep2`)<br>`0.0479 / 0.0748` (`rep3`) | `0.06 / 0.200` (3회 동일) | `3.3333 / 3.3333 / 3.3333` |
| **Linear-Only Clamp** (축 독립 포화) | `0.06 m/s / 0.50 rad/s` | `52.38 / 54.44` (`rep1/2`)<br>*(주1)* | `2.670 / 2.742` (`rep1/2`) | `0.1700 / 0.1106 / 0.1094` | `0.0243 / 0.0544` (`rep1`)<br>`0.0312 / 0.0648` (`rep2`) | `0.06 / 0.350` (`rep1`)<br>`0.06 / 0.353` (`rep2`) | **`5.8338 / 5.8770`** (약 **1.75배** 증가) |

- *(주1)* `patrol_7scen_rep3`의 `linear_only_clamp` 구간은 `_t0_first` 합산 보강 전 실행되어 중간 재시도 구간(`(0.92, 0.43) -> (1.5, 1.0)`, `12.5s / 0.705m`)만 별도 집계되었으며, 최종 `rep1`·`rep2`에서 전체 구간이 일관되게 재현되었다.
- **핵심 해석**: `vx`만 `0.06 m/s`로 제한하고 `wz`를 `0.50 rad/s`로 유지하는 축 독립 포화(Linear-Only Clamp)에서는 선속이 포화되는 동안 각속이 `0.35 rad/s`까지 유지되어 최대 곡률 `|wz/vx|`가 `5.83~5.88 1/m`로 Baseline/Proportional(`3.0~3.5 1/m`) 대비 약 1.75배 급증한다. 반면 비례 제한(Proportional Clamp)은 최대 곡률을 `3.33 1/m`로 안정적으로 유지한다.

### 4.3 실제 Gazebo 물리 시뮬레이터 주행 검증 (`Ignition Fortress` Office World)
- 실행 스크립트: `fleet_control/tests/gazebo_web/nav2_d3_targeted.py` (도메인 `205`, `O6_integrated` 프로파일, Gate r13 포함)
- 로그 경로: `/tmp/nav_validation_20261124/gazebo_d3_o6_verify/SUMMARY.json`
- **총 결과**: **8/8 PASS (0 FAIL)** (`4개 케이스 × 2회 반복`), **접촉/정체 시간(`blocked_s`) 전 구간 `0.0 s`**, Nav2 경고 이벤트(`planner_fail`, `collision_ahead`, `rpp_collision`, `progress_fail`, `spin`, `backup`, `controller_abort`) **전 항목 `0`**.

| 케이스 | 반복 | Nav2 결과 | 최소 footprint 이격 (`min_clear_m`) | 접촉/정체 (`blocked_s`) | 목표 오차 (`final_err_m`) | 소요 시간 (`duration_s`) | Nav2 이상 이벤트 |
|---|---|---|---|---|---|---|---|
| `behind_goal` (벽 대면 후방 목표) | r0 / r1 | `SUCCEEDED` / `SUCCEEDED` | `0.166 m` / `0.165 m` | `0.0` / `0.0` | `0.114 m` / `0.116 m` | `24.6 s` / `24.6 s` | 전부 `0` |
| `wall_oblique` (벽면 사선 접근) | r0 / r1 | `SUCCEEDED` / `SUCCEEDED` | `0.342 m` / `0.342 m` | `0.0` / `0.0` | `0.116 m` / `0.116 m` | `11.5 s` / `11.5 s` | 전부 `0` |
| `adjacent_start` (벽 0.11m 측면 출발) | r0 / r1 | `SUCCEEDED` / `SUCCEEDED` | `0.104 m` / `0.104 m` | `0.0` / `0.0` | `0.113 m` / `0.111 m` | `20.9 s` / `20.9 s` | 전부 `0` |
| `cancel_mid` (직선 주행 2.5초 후 취소) | r0 / r1 | `CANCELED` / `CANCELED` | `0.588 m` / `0.589 m` | `0.0` / `0.0` | `2.202 m` / `2.202 m` | `7.0 s` / `7.0 s` | 전부 `0` |

### 4.4 미검증 / SKIP 항목 (투명 구분)
- **실차(LIMO Pro) 주행 검증**: **SKIP** (실차 전원 OFF 및 야간 병렬 작업 금지 범위).
- **Gazebo + AMCL + Fleet Server 전체 7시나리오 통합 순찰**: **SKIP** (본 검증에서는 Gazebo 물리 주행은 Truth Localization fixture 기반 `nav2_d3_targeted.py` 8회로 검증했고, AMCL+Fleet Server+Agent 재시작+속도 제한 통합은 Kinematic 시뮬레이터 3회 반복으로 검증함).

---

## 5. Claude Code 통합 패치 제안 (충돌 파일 비수정 원칙 준수)

아래 3건은 G2 통합 브랜치(`Capstone1_limo_g2_20261123`) 반영을 권장하는 패치 제안이다.

1. **Nav2 Slow 프로파일(`integrated_nav2_lidar_only_slow_params.yaml`) O6 반영**
   - `FollowPath.use_rotate_to_heading: true`, `rotate_to_heading_min_angle: 0.785`
   - `progress_checker.plugin: "nav2_controller::PoseProgressChecker"` (`required_movement_radius: 0.12`, `required_movement_angle: 0.5`, `movement_time_allowance: 5.0`)
   - 근거: Kinematic Phase 3 72/72 PASS + Gazebo 실물리 8/8 PASS (`blocked_s=0.0`, `planner_fail=0`).
2. **서버 `INITIAL_POSE` footprint clearance 사전 점검 제안 (`fleet_control/server/fleet_server/navigation.py`)**
   - 현재 `set_initial_pose`는 중심점 `clearance=0.0`만 검사하므로 벽면 `0.05 m` 인접 위치도 `CONFIRMED`가 된 뒤 Nav2 `NavfnPlanner`에서 즉시 치명 비용 실패가 발생한다.
   - 제안: `check_point(..., clearance=robot_radius)`가 `FOOTPRINT_OCCUPIED`인 경우 운영자 UI에 경고를 노출하거나(또는 strict 모드에서 사전 거부) 안전 이격 부족을 즉시 진단할 수 있도록 보강.
3. **속도 제한(Watchdog / Arbiter Speed Clamp) 비례 스케일링 권고**
   - 시나리오 7 측정 결과, `vx`만 낮추는 축 독립 클램프는 곡률 `|wz/vx|`를 최대 1.75배 증가시켜 좁은 통로나 실차 타이어 슬립 환경에서 급선회를 유발할 수 있다.
   - 제안: 실차 Watchdog Speed Clamp 또는 웹 UI 프리셋에서 `max_linear_mps`를 낮출 때 `max_angular_rps`도 동일 비율로 하향하는 비례 제한 프로파일을 기본값으로 권장(또는 `wz_eff <= min(wz_cap, wz_cmd * (vx_eff / vx_cmd))` 곡률 보존 클램프 옵션 검토).
