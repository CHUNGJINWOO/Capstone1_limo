# 패키지·노드·Launch 구조 감사 (2026-11-20, 읽기 전용)

현재 작업트리 기준이다(브랜치 `staging/fleet-control-on-progress-c804241`, HEAD `c804241`, `fleet_control/` 미추적). 파일 이동·삭제·패키지 이름 변경은 하지 않았다. 아래 "제안"은 승인 전 설계안이다.

## 1. ROS 2 패키지 (실제 `package.xml` 기준)

| 패키지 | 위치 | 빌드 | 역할 | 판단 |
|---|---|---|---|---|
| `limo_base`, `limo_msgs`, `limo_description` | `capstone_ws/src/limo_ros2/` | cmake | 베이스 드라이버(시리얼)·메시지·URDF | 유지 (벤더) |
| `ydlidar_ros2_driver` | `capstone_ws/src/` | cmake | LiDAR 드라이버 | 유지 (벤더) |
| `astra_camera`, `astra_camera_msgs` | `capstone_ws/src/ros2_astra_camera/` | cmake | DaBai Depth/RGB | 유지 (벤더) |
| `robot_localization` | `capstone_ws/src/` | cmake | EKF (F1b/F5 추정기) | 유지 (벤더). 실차 F4 경로는 미사용 |
| `explore_lite`(+`_msgs`), `multirobot_map_merge` | `capstone_ws/src/m-explore-ros2/` | cmake | Frontier 탐색. 미커밋 `fleet_frontier_candidates` 실행 파일 추가(다른 작업) | `explore`(직접 goal 제출)와 `fleet_frontier_candidates`(후보만 발행)를 동시에 실행하면 안 됨. `map_merge`는 제거 후보(미사용) |
| `limo_capstone` | `capstone_ws/src/` | python | 추정기(F4 ICP, EKF 보조), Safety Gate(`stall_detector_node`), odom TF relay, 기록·분석 노드, launch·config | 핵심. 기록·분석 노드는 실행 구조와 분리 권장 |
| `limo_depth_obstacles` | `capstone_ws/src/` | python | Depth 장애물 필터 | 유지 |
| `limo_ros2_application` | `capstone_ws/src/` | python | 벤더 데모(`move_limo`, `detect_line` 등) | 제거 후보(실행 경로 미참조) |
| `wego`, `wego_msgs`, `wego_2d_nav`, `magic_enum` | `capstone_ws/src/` | cmake | 이전 교재/벤더 자료 | 제거 후보(참조 확인 후) |
| `limo_robot_agent` | `fleet_control/agent/` | python | Fleet Agent, Mapping Worker, **motion_watchdog**, mock | 유지. Runtime Supervisor 역할 |
| `limo_fleet_bringup` | **origin/main에만 존재** | python | 기능별 bringup 모듈 | 로컬 미존재. 격리 시험은 `tests/web_mapping_isolated/stack.launch.py`로 대체 |
| `limo_scan_normalizer` | **`experiments/real_depth_slow_*` 번들에만 존재** | python | `/scan`→`/scan_slam` 고정 빔 | 실차 launch가 의존하지만 `capstone_ws/src`에 없음 → **재현성 결함** |
| (비패키지) `obstacle_memory_node.py` | `experiments/safety_chain_fix_20261113/fixed2/`, 실차 /tmp | - | 장애물 기억 노드 | 패키지 미편입·실차 소스 유실 → **재현성 결함** |
| (비패키지) `limo_rmf_adapter` | `fleet_control/rmf_adapter/` | - | Open-RMF Fleet Adapter | ROS 패키지 아님(모듈 실행) |

## 2. 실행 노드와 인터페이스 (코드에서 추출)

| 노드 | 주요 PUB | 주요 SUB / 서비스 / 액션 | TF |
|---|---|---|---|
| `stall_detector_node` (Safety Gate) | `/cmd_vel`(실차) 또는 `/cmd_vel_locked`(잠금), `/stall_detector/status_json`, `virtual_bumper_cloud` | `/cmd_vel_raw`, `/odom`, `/odometry/lidar_odom`, `/scan`, Depth 상태·점군, SRV `/stall_detector/reset_latch` | - |
| `lidar_scan_matcher_odom_node` (F4) | `/odometry/lidar_odom`, `/diagnostics/f4_latency` | `/scan` | - |
| `odom_tf_relay_node` | `/odom_nav2` | 선택 추정기 odom | `odom→base_footprint` |
| `depth_obstacle_filter_node` | `/depth_obstacles/points`, `clearing_points`, `/auto_mapping/depth_status` | `/camera/depth/points` | - |
| `motion_watchdog` (신규, 별도 프로세스) | `/cmd_vel_raw`, `/fleet/motion_watchdog/status` | `/fleet/nav_cmd_vel`, `/fleet/agent_heartbeat`, NavigateToPose status, SRV-CLI `cancel_goal` | - |
| `fleet_robot_agent` (ros_adapter) | `/fleet/agent_heartbeat` | ACT-CLI `navigate_to_pose`, FollowPath status, Gate·Depth·map·odom·scan, watchdog 상태 | - |
| Mapping Worker (Agent 내부) | (소유 프로세스 기동) | `/fleet/frontier_candidates`, SRV-CLI `/slam_toolbox/serialize_map` | - |
| `slam_control` (SIM 전용) | - | **`/cmd_vel` 구독** | - |
| `integrated_recorder_node` | - | **`/cmd_vel` 구독**, 다수 | - |
| `nav2_action_monitor_node` | - | NavigateToPose status·feedback | - |
| SLAM Toolbox / Nav2 / explore | 표준 | 표준 | `map→odom`(SLAM) |

주의: `/cmd_vel`을 구독하는 노드(`slam_control`, `integrated_recorder_node`, 시험 모니터)가 있으면 Mapping Worker의 잠금 검사가 실패해 맵핑이 거부된다(fail-closed). 웹 Mapping과 같은 그래프에서 실행하지 않는다.

## 3. Launch 파일

| Launch | 모드 | 비고 |
|---|---|---|
| `limo_integrated_auto_mapping_real.launch.py` | 실차 통합(드라이버·추정기·SLAM·Nav2·Gate·선택 Depth) | 다른 에이전트 미커밋 수정 중. `base_footprint_z` 기본 0.0(문제 §Depth), 속도 경로에 watchdog 없음(패치 02 제안) |
| `limo_integrated_auto_mapping_sim(_diff).launch.py` | Gazebo 통합 | 시뮬 URDF는 `base_footprint`가 바닥 |
| `limo_auto_mapping(_sim).launch.py`, `limo_nav2(_sim).launch.py`, `limo_mc3f_sim.launch.py`, `limo_light.launch.py` | 단계별·과거 시험 | 통합 launch와 기능 중복 |
| `fleet_control/tests/web_mapping_isolated/stack.launch.py` | 격리 잠금 시험 | Nav2→`/fleet/nav_cmd_vel`→watchdog→`/cmd_vel_raw`→Gate(`/cmd_vel_locked`) |

## 4. 책임 분리 현황과 목표

| 목표 책임 | 현재 구현 | 차이 / 제안 |
|---|---|---|
| Hardware / Sensors | `limo_base`, `ydlidar_ros2_driver`, `astra_camera` | 유지 |
| Perception | `limo_depth_obstacles`, `limo_scan_normalizer`(experiments), 기억 노드(experiments) | normalizer·기억 노드를 `capstone_ws/src`의 패키지로 편입(새 패키지 1개 또는 `limo_depth_obstacles`에 통합) |
| Localization / TF | `limo_capstone`(F4, relay, EKF 보조), `robot_localization` | 유지. TF 높이 기준(`base_footprint_z`) 실측 후 확정 |
| Mapping / Exploration | SLAM Toolbox, `explore_lite`(`explore`, `fleet_frontier_candidates`), Mapping Worker, SIM `slam_control` | Worker를 단일 소유자로. `explore` 직접 제출 경로와 SIM `slam_control`은 운영 경로에서 제외 |
| Navigation / Goal | Nav2, Agent goal slot(Fleet 미션 + Frontier) | 유지(단일 goal slot) |
| Safety / Motion Watchdog | `stall_detector_node`(Gate), `motion_watchdog`(신규) | watchdog을 실차 launch에 편입(패치 02), Gate 측 선택 요구(패치 03) |
| Fleet / RMF | `fleet_server`, `limo_rmf_adapter` | 유지 |
| Robot Agent / Runtime Supervisor | `limo_robot_agent` | 유지 |
| Web / Map Registry | `fleet_server/static`, `map_registry` | `dashboard/`, `dashboard_lab/`, `dashboard_v2_design/`는 별도 시험·설계 자료 → 운영 UI는 `fleet_server/static` 하나로 명시 |
| Bringup / Simulation | `limo_capstone/launch/*`, `integrated_sim/`, `gazebo/`, origin/main `limo_fleet_bringup` | 실차·Gazebo 공통 진입점 1개(인자로 하드웨어/시뮬 선택) 제안. origin/main `limo_fleet_bringup`의 모듈 구성을 참고하되 병합은 승인 후 |

## 5. 중복·미사용 (제거 후보, 미실행)

- 기록 경로 3종: `integrated_recorder_node`, `scripts/run_recorder.py`, Agent `recording_session`
- SLAM 제어 2종: `slam_control`(SIM), `mapping_worker`(웹)
- Frontier 2종: `explore`(직접 제출), `fleet_frontier_candidates`(후보)
- 대시보드 3종 + 설계 자료
- 단계별 launch 다수(통합 launch와 중복)
- `check_creator_allowed`(ownership.py, 호출되지 않는 dead code)
- 벤더 데모·교재 패키지(`limo_ros2_application`, `wego*`, `magic_enum`, `multirobot_map_merge`)

## 6. 프로세스 실행·종료 책임

| 대상 | 기동 | 종료 | 비고 |
|---|---|---|---|
| 드라이버·추정기·Nav2·Gate | 실차 launch | launch 종료 | Gate 사망 시 `/cmd_vel` 소실 → 모터 드라이버/MCU 타임아웃에 의존(미검증) |
| SLAM·Frontier(웹 맵핑) | Mapping Worker (`spawn_owned`, PDEATHSIG) | Worker STOP / Agent 사망 시 커널 | 시험으로 확인 |
| motion_watchdog | (제안) 실차 launch | launch | Agent와 독립 |
| Agent | systemd/수동 | - | 현장 자동 기동은 미구성 |
