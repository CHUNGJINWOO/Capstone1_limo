# 설정 분리: 시뮬레이션 기준선과 실차 설정의 차이

기준일: 2026-11-11. 시뮬레이션 기준선 YAML은 수정하지 않았다. 실차에서 쓰던 조정값은 별도 파일로 두고 launch가 경로를 명시적으로 가리킨다.

## 1. 파일 대응

| 용도 | 파일 | 사용하는 launch |
|---|---|---|
| 시뮬레이션 기준선 Nav2 | `integrated_nav2_params.yaml` (변경 없음) | `limo_integrated_auto_mapping_sim.launch.py` |
| 시뮬레이션 기준선 explore | `integrated_explore_params.yaml` (변경 없음) | sim, sim_diff |
| 실차 Nav2(깊이 사용) | `integrated_nav2_real_depth_params.yaml` | `limo_integrated_auto_mapping_real.launch.py` |
| 실차 explore | `integrated_explore_real_params.yaml` | real |
| 실차 LiDAR 전용 Nav2 | `integrated_nav2_lidar_only_params.yaml`, `..._slow_params.yaml` | real (인자로 선택) |
| 차동 구동 시뮬레이션 변형 | `integrated_nav2_diff_params.yaml` | `limo_integrated_auto_mapping_sim_diff.launch.py` |

## 2. 기준선과 실차 설정의 차이(두 파일을 평탄화해 키 단위로 비교한 결과)

`integrated_nav2_params.yaml` → `integrated_nav2_real_depth_params.yaml`

| 키 (`controller_server.ros__parameters.FollowPath.` 생략) | 기준선(sim) | 실차 |
|---|---|---|
| `desired_linear_vel` | 0.26 | 0.32 |
| `lookahead_dist` / `min_` / `max_` / `lookahead_time` | 0.50 / 0.30 / 0.80 / 1.5 | 0.40 / 0.20 / 0.70 / 1.2 |
| `min_approach_linear_velocity` | 0.10 | 0.15 |
| `approach_velocity_scaling_dist` | 0.4 | 0.3 |
| `regulated_linear_scaling_min_radius` / `_min_speed` | 0.8 / 0.15 | 0.6 / 0.18 |
| `general_goal_checker.xy_goal_tolerance` | 0.25 | 0.12 |
| `bt_navigator…default_nav_to_pose_bt_xml` | (키 없음) | Nav2 기본 트리 경로(자리 키) |

`integrated_explore_params.yaml` → `integrated_explore_real_params.yaml`

| 키 (`explore_node.ros__parameters.` 생략) | 기준선(sim) | 실차 |
|---|---|---|
| `min_frontier_size` | 0.2 | 0.35 |
| `planner_frequency` | 0.33 | 0.5 |
| `progress_timeout` | 25.0 | 30.0 |
| `use_sim_time` | True | False |

시뮬레이션 차동 변형(`integrated_nav2_diff_params.yaml`)은 위 실차 컨트롤러 값에 `use_rotate_to_heading: true`를 더한 값이다. 즉 기준선과의 차이 27줄 중 대부분이 실차 조정값에서 왔다. 시뮬레이션 Ackermann 기준선은 계속 위 "기준선" 값을 쓴다.

## 3. launch가 덮어쓰는 값
- real launch: Ackermann 복구 BT(`navigate_to_pose_ackermann_no_reverse.xml`)를 `RewrittenYaml`로 적용한다(Spin/BackUp 제거, 게이트가 후진을 막기 때문).
- sim 기준선 launch: Nav2 기본 BT를 그대로 쓴다. 따라서 **시뮬레이션 Ackermann에서는 Spin 복구가 남아 있다**(원 실험에서 yaw 변화 0°로 시간 초과했던 동작). 이 정리에서는 기준선을 보존했으므로 고치지 않았다.
- sim_diff launch만의 시뮬레이션 전용 값: 깊이 필터 `processing_hz: 20`, `min_height: -0.02`, 비용 지도 `min_obstacle_height: -0.03`(시뮬레이션 모델의 `base_footprint`가 바닥에서 0.044 m 떠 있어 보정). 실차에는 적용하지 않는다.

## 4. 해석상 주의
- 과거 실험 결과(`experiments/`)는 당시 설정으로 얻었다. 위 분리 이후의 파일 이름·값으로 다시 얻은 결과가 아니다.
- 실차 YAML 두 개는 실차 launch 파일을 올려 본 적이 없다(문법·인자 평가까지만 확인, 3절은 `REAL_ROBOT_STATUS.md` 참고).
