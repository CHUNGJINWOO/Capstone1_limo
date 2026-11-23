# ARCHITECTURE.md — 시스템 아키텍처 및 데이터·명령 흐름

> **기준일**: 2026-11-14  
> 상세 설정 차이는 [docs/CONFIG_SIM_VS_REAL.md](docs/CONFIG_SIM_VS_REAL.md) (`main` `2010e44` / `419e798` 브랜치; 로컬 `<WORKSPACE_PARENT>/Capstone1_limo_dashboard/docs/CONFIG_SIM_VS_REAL.md`), 자율주행 설계 논의는 [docs/design/AUTONOMOUS_DRIVING_DESIGN.md](docs/design/AUTONOMOUS_DRIVING_DESIGN.md), 관제·RMF API는 `feat/fleet-control-rmf-integration`(`419e798`) 브랜치의 `fleet_control/docs/API.md` 및 `fleet_control/rmf_adapter/docs/RMF.md`를 참조하세요.

---

## 1. 전체 시스템 계층 구조 및 확인된 실행 모드별 통신 경로

시스템은 크게 **(A) 로봇 온보드(또는 Gazebo) 자율주행·안전 스택**과 **(B) 노트북 중앙 관제·Open-RMF·AI·대시보드 스택**으로 나뉩니다.  
아래 다이어그램과 프로토콜 명세는 **실제 코드와 실행 로그로 확인된 실행 모드에만 적용되는 사실**이며, 아직 실행·검증하지 않은 운용 모드(예: 호스트 간 직접 DDS 공유, 실차 RMF 주행, 상시 프로덕션 서버 운용)로 일반화하지 않습니다.

```mermaid
flowchart TB
    subgraph Laptop["노트북 (중앙 관제 · Open-RMF · AI · 대시보드)"]
        UI["웹 대시보드 / Dashboard Lab<br/>(HTTP / WebSocket)"]
        RMF["Open-RMF 코어<br/>(rmf_traffic_schedule / rmf_task_dispatcher)"]
        ADAPTER["LIMO RMF Fleet Adapter<br/>(limo_rmf_adapter)"]
        MM["Mission Manager (fleet_server)<br/>HTTP/1.1 REST + SQLite + 제어권 Lease"]
        YOLO["AI 호스트 (CUDA YOLO11n)<br/>커스텀 TCP 수신 + 로컬 HTTP 제공"]
        RO_GW["읽기 전용 시각화 Gateway<br/>(dashboard / dashboard_lab)"]
    end

    subgraph Robot["LIMO 온보드 / Gazebo 시뮬레이션 (로컬 ROS 2 DDS)"]
        AGENT["LIMO Robot Agent<br/>(limo_robot_agent)"]
        EXPLORE["Frontier 탐색기<br/>(explore_lite)"]
        NAV2["Nav2 스택<br/>(Planner / Controller / BT Navigator)"]
        CM["Collision Monitor + 장애물 기억<br/>(nav2_collision_monitor + obstacle_memory_node)"]
        GATE["Safety Gate<br/>(stall_detector_node)"]
        BASE["LIMO 구동부 / Gazebo Diff-Drive<br/>(limo_base / /cmd_vel)"]
        SENSORS["센서 · 오도메트리 · SLAM<br/>(/scan -> /scan_slam, Depth, F4 ICP, Wheel Odom, slam_toolbox / AMCL)"]
        VSEND["RGB 영상 송신 / 읽기 전용 브리지<br/>(sender / bridge_node.py)"]
    end

    UI -->|HTTP/1.1 REST 임무 생성·취소 / Lease| MM
    RMF <-->|로컬 DDS 토픽 + WS 브로드캐스트<br/>(--rmf-api-uri, 시뮬 검증)| ADAPTER
    ADAPTER -->|HTTP/1.1 REST Lease · 임무 변환| MM
    AGENT -->|아웃바운드 HTTP/1.1 폴링 · 상태/지도/ACK 보고| MM
    AGENT -->|NavigateToPose Action| NAV2
    EXPLORE -->|NavigateToPose Action| NAV2
    SENSORS -->|TF / Map / Costmap 입력| NAV2
    SENSORS -->|Scan / Depth 3D 점군| CM
    SENSORS -->|신선도 · 속도 · 상태 감시| GATE
    NAV2 -->|/cmd_vel_raw 또는 /cmd_vel_pre_cm| CM
    CM -->|/cmd_vel_raw| GATE
    GATE -->|allow_motor_drive=true: /cmd_vel<br/>false: /cmd_vel_locked| BASE
    SENSORS -->|로컬 ROS 2 구독| VSEND
    VSEND <-->|커스텀 길이 프리픽스 TCP<br/>(현장 18191 / 통합시뮬 18210)| YOLO
    YOLO -->|HTTP 18192/18211 (/mjpeg, /api/video/*)| UI
    YOLO -->|HTTP 18192/18211 프록시 조회| RO_GW
    VSEND -.->|실차 15초 시험 한정: SSH stdout 파이프<br/>→ 로컬 POST /api/bridge (18085)| RO_GW
```

### 1.1 확인된 실행 모드별 통신 및 격리 사실 (`ROS_LOCALHOST_ONLY`, HTTP, TCP, SSH, RMF)

| 통신 구간 | 확인된 실제 실행 모드 및 스크립트 | 실제 사용 프로토콜 · 포트 · 환경변수 | 미검증 모드 (일반화 금지) |
|---|---|---|---|
| **온보드 / 시뮬 내부 ROS 2 통신** | - 현장 시험 노트북(로컬 미커밋): `experiments/real_limo_field_20261113/laptop/lf_env.sh` (`ROS_DOMAIN_ID=77`), `lf_t2_rmf.sh` (`ROS_DOMAIN_ID=151`)<br/>- 격리 시뮬 시험: `run_rmf_checks.py`, `run_gazebo_checks.py`, `integrated_env.py` | **로컬 ROS 2 DDS (`ROS_LOCALHOST_ONLY=1`)**.<br/>**Zenoh 브리지(`zenoh-bridge-ros2dds`)는 현재 어떤 코드·스크립트에서도 구현·실행되지 않음** ([docs/design/AUTONOMOUS_DRIVING_DESIGN.md](docs/design/AUTONOMOUS_DRIVING_DESIGN.md)의 후보 논의일 뿐). | 로봇↔노트북 간 멀티호스트 DDS 직접 공유 또는 Zenoh 브리지 운용은 **미검증** |
| **Robot Agent $\rightarrow$ Mission Manager** | - 실차 모터 잠금 시험(로컬 미커밋): `experiments/real_limo_field_20261113/laptop/lf_t1_server.sh` (Tailscale `18190`)<br/>- 통합 시뮬: `integrated_env.py` (로컬 `18208`) | `limo_robot_agent`가 **단방향 아웃바운드 HTTP/1.1 REST 폴링·POST**(`POST /api/v1/agent/state`, `/api/v1/agent/map`, `/api/v1/agent/commands/{id}/ack`) 수행. 서버가 로봇으로 직접 연결을 열지 않음. | `fleet_local.json` 기본 포트(`8080`) 상시 운용 및 실차 `allow_missions=true` 구동 명령 전달은 **미검증** |
| **RMF Adapter $\leftrightarrow$ RMF Core & Mission Manager** | - 합성/Gazebo/통합 시뮬: `run_rmf_checks.py`, `run_gazebo_checks.py`, `integrated_env.py`<br/>- 현장 노트북 기동 시험: `experiments/real_limo_field_20261113/laptop/lf_t2_rmf.sh` | - Adapter $\leftrightarrow$ RMF Core: 동일 머신 내 로컬 ROS 2 DDS 토픽 + WebSocket 상태 수신(`--rmf-api-uri ws://127.0.0.1:<port>`)<br/>- Adapter $\rightarrow$ Mission Manager: HTTP/1.1 REST API (`mission_client.py`) | 실차 `map_002` 기반 실제 RMF 배차·주행은 **미검증** (`lf_t2_rmf.sh`는 Gazebo 그래프 기준이라 실차 좌표 `(20,10)`에서 등록 거부됨) |
| **RGB 송신기(`sender`) $\leftrightarrow$ AI 호스트(`ai_host`) $\rightarrow$ 대시보드** | - 실차 모터 잠금 시험(로컬 미커밋): `experiments/real_limo_field_20261113/laptop/lf_t3_ai.sh` (TCP `18191`, HTTP `18192`)<br/>- 통합 시뮬: `integrated_env.py` (TCP `18210`, HTTP `18211`) | - `sender` $\leftrightarrow$ `ai_host`: **4바이트 길이 프리픽스 커스텀 TCP 소켓**으로 JPEG 프레임 전송 및 원본 stamp 보존 검출 JSON 수신<br/>- `ai_host` $\rightarrow$ 브라우저/Gateway: **로컬 HTTP 서버**(`/mjpeg`, `/api/video/status`, `/api/video/detections`) | 실차 자율 주행(`--drive`) 중 동시 영상/AI 스트리밍 부하는 **미검증** |
| **읽기 전용 시각화 브리지 (`bridge_node.py` $\rightarrow$ `dashboard`)** | - 실차 15초 정지 시험: `Capstone1_limo_dashboard`의 `real_trial.py` | LIMO 온보드 `bridge_node.py`가 로컬 ROS 2 토픽(`/map`, Costmap, TF, Depth)을 구독해 **SSH stdout 파이프(JSON 라인)**로 출력하면, 노트북 `real_trial.py`가 받아 로컬 Gateway(`POST http://127.0.0.1:18085/api/bridge`)로 전달. RGB는 별도 15초 창에서 `readonly_rgb.py`(5 fps)로 시험. | 장시간 연속 스트리밍, 주행 중 `/plan`·`/local_plan` 실시간 표시, Depth+RGB 동시 단일 루프 실차 검증은 **미검증** |

---

## 2. 온보드 자율주행 및 안전 체인 (Robot Stack)

### 2.1 센서 및 위치추정 파이프라인
- **LiDAR 스캔 정규화**: YDLiDAR `/scan`은 기동 직후 빔 개수 변동(233~245개)으로 인해 `slam_toolbox`(Karto)에서 스캔 거부 경고를 유발할 수 있어, `limo_scan_normalizer`가 고정 244빔 `/scan_slam`으로 정규화합니다 (`docs/scan-normalizer-locked-test-20261112` `09d742c` 브랜치 및 로컬 미커밋 `docs/dev_notes/2026_11_12_실차_스캔정규화_배포와_모터잠금_검증.md`).
- **3D Depth 장애물**: Orbbec DaBai 깊이 영상에서 생성한 `/camera/depth/points`를 `depth_obstacle_filter_node`가 받습니다. 촬영 시각의 `base_footprint` 변환을 사용해 높이·거리 필터를 적용하고, `/depth_obstacles/points`와 `/auto_mapping/depth_status`를 발행합니다. 2026-11-18 실차 배포본은 미래 시각 TF를 비동기로 짧게 기다리며, 만료 시 Depth 오류를 보고합니다.
- **실차 현장 지도 저장**: 세션이 받은 SLAM `/map` 스냅샷을 `/real_mapping_session/map_snapshot`으로 Nav2 map saver에 전달합니다. 같은 스냅샷에서 해상도·원점을 보존하며 trinary PGM은 미탐색 셀 보존용 임계값으로 저장합니다. 현장 세션은 기존 정지 확인이 끝난 뒤 저장 경로에 진입합니다.
- **오도메트리 및 TF**:
  - `base_footprint -> base_link / laser_link / depth_camera_link`: 정적 TF (`limo_description`).
  - `odom -> base_footprint`: 시뮬레이션/설정에 따라 휠 오도메트리 또는 `lidar_scan_matcher_odom_node`(F4 ICP)가 담당하며, Safety Gate 비교용으로 `/odometry/lidar_odom`이 항상 발행됩니다.
  - `map -> odom`: 자동 맵핑 시 `slam_toolbox`, 저장 지도 주행 시 `amcl`이 발행합니다.

### 2.2 속도 명령(Command Velocity) 안전 체인
모든 자율주행 명령은 **단일 안전 게이트(`stall_detector_node`)**를 거치며, 우회 경로는 금지됩니다 (로컬 개발 노트 `docs/dev_notes/2026_11_10_cmd_vel_안전게이트_경로_점검.md`).

1. **기본 경로 ([capstone_ws/src/limo_capstone](capstone_ws/src/limo_capstone) 및 실차 배포된 `real_auto_mapping_ready_20261113` / `real_depth_slow_20261113`)**:
   - `Nav2 controller_server` → `/cmd_vel_raw` → `stall_detector_node` (Safety Gate) → `/cmd_vel` (잠금 모드 시 `/cmd_vel_locked`, `/cmd_vel` 발행자 0개).
2. **Collision Monitor 확장 경로 (노트북 준비 번들 `real_depth_slow_cm_20261113` 및 시뮬 실험 수정본 `safety_chain_fix_20261113/fixed2`)**:
   - `Nav2` → `/cmd_vel_pre_cm` → `nav2_collision_monitor` (입력: `/scan`, `/depth_obstacles/points`, `/collision_monitor/memory_points`) → `/cmd_vel_raw` → `stall_detector_node` → `/cmd_vel`.
   - `obstacle_memory_node`가 근거리 사각(LiDAR 최소거리 안쪽, Depth 0.55 m 안쪽)으로 들어간 장애물 셀을 유지하며, `fixed2`(합성·Office Gazebo 검증 후보)에서는 `/collision_monitor/memory_status`와 `memory_layer`(Costmap 공유)를 통해 플래너와 모니터의 장애물 인식을 일치시킵니다 (로컬 미커밋 문서 `experiments/safety_chain_fix_20261113/docs/MEMORY_POLICY_V2.md`). (단, `real_depth_slow_cm_20261113`과 `fixed2`는 아직 실차 LIMO에 배포·실행되지 않음)
3. **Safety Gate (`stall_detector_node`) 보호 규칙**:
   - 후진 속도(`linear.x < 0`) 강제 0 클램핑.
   - 센서 타임아웃(`/scan`, `/odom`, `/odometry/lidar_odom`, `/auto_mapping/depth_status`), 스톨(명령 대비 실측 이동 0), ICP 인라이어 저하 시 즉시 0 출력 (`FAILSAFE_*` / `STALL_LATCHED`).
   - `fixed`/`fixed2` 실험 후보에서는 타임스탬프 정체(`STAMP_NOT_ADVANCING`), TF 단절, 고장 시점 이전 목표의 자동 재개 차단(`block_resume_of_fault_goals`) 동작이 합성·Gazebo에서 검증되었습니다 (로컬 미커밋 문서 `experiments/safety_chain_fix_20261113/docs/FAILSAFE_CONTRACT.md`).

---

## 3. 중앙 관제 · Open-RMF · 영상 AI · 대시보드 (Fleet & Laptop Stack)

1. **Mission Manager (`fleet_control/server/fleet_server` — `feat/fleet-control-rmf-integration` `419e798` 브랜치)**:
   - **유일한 상위 제어 평면(Single Control Plane)**입니다. 웹 운영자와 Open-RMF Adapter 모두 로봇에 직접 명령하지 않고 Mission Manager의 제어권 Lease(`ownership.py`)와 명령 큐(`service.py`)를 통해서만 임무를 생성·취소합니다.
   - 인증 토큰(`viewer`, `operator`, `admin`, `agent-*`, `rmf-adapter`)과 SQLite 저널(`db.py`), 증거 기반 운영자 복구(`recovery.py`)를 수행합니다.
2. **Robot Agent (`fleet_control/agent/limo_robot_agent` — `419e798` 브랜치 및 `Capstone1_limo_fleet_rmf` 수정본)**:
   - 로봇(또는 시뮬레이션)의 로컬 ROS 2 상태를 폴링해 Mission Manager에 HTTP/1.1 REST로 보고하고, 수락된 임무를 Nav2 `NavigateToPose` 액션으로 전달합니다.
   - 기존 SLAM/Nav2/Safety Gate 파라미터를 변경하거나 `/cmd_vel`을 직접 발행하지 않습니다.
   - 실차 기본값은 `allow_missions:=false`이며, 시뮬레이션에서만 `motor_policy.py`(`allow_sim_safety_gate:=true`, `SIM_GATED`)를 허용합니다.
3. **Open-RMF Fleet Adapter (`fleet_control/rmf_adapter/limo_rmf_adapter` — `419e798` 브랜치 및 `Capstone1_limo_fleet_rmf` 수정본)**:
   - `$HOME/rmf_isolated`의 Open-RMF 2.1.8 격리 환경에서 동작하며, RMF 그래프 좌표와 Nav2 지도 좌표를 양방향 변환(`coords.py`)하고 Mission Manager HTTP API를 호출(`mission_client.py`)합니다.
4. **RGB 영상 · CUDA YOLO 파이프라인**:
   - 확인된 실행 모드(`lf_t3_ai.sh`, `integrated_env.py`)에서 로봇/시뮬 카메라(`/camera/color/image_raw/compressed`) 프레임을 최대 10 fps로 커스텀 TCP 소켓을 통해 노트북 AI 호스트(`yolo_gazebo_limo` venv, YOLO11n CUDA)에 전송하고, 원본 카메라 타임스탬프가 보존된 검출 결과를 대시보드 및 로봇 consumer로 반환합니다.
   - **YOLO 검출 결과는 시각화·관제 보조용이며 Nav2나 Safety Gate의 제어 입력으로 사용되지 않습니다.**
5. **대시보드 계층 (`codex/dashboard-visualization` 워크트리 미추적 작업본 포함)**:
   - `fleet_control/server/fleet_server/static/`: 기본 중앙 관제 웹 UI (`exp/video-ai-preverify` / `exp/integrated-sim-20261113` 워크트리의 미커밋 작업본에서 실시간 영상 패널 `video_panel.js` 통합; 로컬 경로 `<WORKSPACE_PARENT>/Capstone1_limo_video_preverify/fleet_control/server/fleet_server/static/`).
   - `fleet_control/dashboard/`: SLAM `/map`, Global/Local Costmap, TF, DaBai Depth, RGB/YOLO 읽기 전용 시각화 게이트웨이 및 브리지 (`codex/dashboard-visualization` 워크트리 내 미커밋 경로: `<WORKSPACE_PARENT>/Capstone1_limo_dashboard/fleet_control/dashboard/README.md`).
   - `fleet_control/dashboard_lab/`: 30 Hz 최신 슬롯·JPEG/H.264/WebRTC 비교 및 네트워크 장애 주입 오프라인/합성 연구 환경 (`codex/dashboard-visualization` 워크트리 내 미커밋 경로: `<WORKSPACE_PARENT>/Capstone1_limo_dashboard/fleet_control/dashboard_lab/README.md`).

## 2026-11-20 현행 구조 감사·제안 (승인 전)

현행 패키지·노드·인터페이스와 목표 책임 분리안, 중복·제거 후보는 [fleet_control/docs/ARCHITECTURE_AUDIT_20261120.md](fleet_control/docs/ARCHITECTURE_AUDIT_20261120.md)에 있다. 제안 사항(패키지 편입·통합·제거)은 승인 전이며 적용하지 않았다. 현행 안전 속도 경로(격리 시험 구성): Nav2 → `/fleet/nav_cmd_vel` → motion_watchdog → `/cmd_vel_raw` → Safety Gate. 실차 launch는 아직 Nav2 → `/cmd_vel_raw` → Gate다.

## 2026-11-20 공개 기준선과 후속 작업 감사

공개 기준선의 Velocity Arbiter는 잠금 격리 구성에서 Nav2·수동 입력을 선택한다. 후속 로컬 격리 시험은 별도 Motion Watchdog을 사용했다. Nav2 → `/fleet/nav_cmd_vel` → Watchdog → `/cmd_vel_raw` → Safety Gate → 잠금 출력 경로다. Watchdog은 heartbeat·승인·입력 시간 초과와 비유한 속도를 검사하고 외부 Goal에서 정지·취소를 요청한다. Gate는 센서와 속도 경로를 감시한다. 최신 실차 Launch는 Watchdog 미통합 상태다.

Fleet Server가 lease·Safety Context와 임무 상태를 관리한다. Agent가 로컬 journal과 Frontier/Fleet 공유 Goal 슬롯을 관리하고 UUID를 Nav2 결과와 대조한다. Mapping Worker는 자신이 기동한 SLAM·Frontier 프로세스와 세션을 관리한다. Frontier 후보 노드와 직접 제출형 Explore는 별개 경로다. `map→odom`은 SLAM, `odom→base_footprint`는 선택 추정기와 relay가 담당하는 경로가 있다. Scan normalizer·장애물 기억의 패키지 편입, 실차와 Gazebo 진입점 통합, 중복 구현 정리는 제안 상태다.

## G2 병합 (2026-11-23): origin/main `ea2d9632` 판의 이 문서에만 있던 줄
> 통합 브랜치(v5b) 판을 기준으로 두고, origin/main 판에만 있던 줄을 원래 순서대로 아래에 보존한다. 날짜가 이른 기록(11-14~11-18)이며 위 본문의 최신 상태와 다르면 위 본문이 우선한다. 원문: `git show ea2d9632:ARCHITECTURE.md`.

- **3D Depth 장애물**: Orbbec DaBai `/camera/depth/image_raw`(`16UC1`)를 `limo_depth_obstacles`(`depth_obstacle_filter_node`)가 바닥 제거·높이 필터링 후 `/depth_obstacles/points` 및 `/auto_mapping/depth_status`로 발행합니다.
## 11-18 기능별 노드와 제어 경계
기준 소스는 `limo_integrated_auto_mapping_real.launch.py`, `limo_integrated_auto_mapping_sim_diff.launch.py`, `fleet_control/`이다. 현장 번들, 공개 launch, Gazebo r8 시험은 서로 다른 구성이다.
| 기능 | 실제 패키지와 진입점 | 토픽·Action 및 의존성 | 검증 범위 |
| 하드웨어·센서 | `limo_base` 드라이버, `ydlidar_ros2_driver/ydlidar.launch.py`, `astra_camera` DaBai launch | USB–UART, /cmd_vel, /odom, /imu, /limo_status, /scan, RGB·Depth·CameraInfo | 기존 실차 잠금 수신. USB–UART 불안정. |
| Gazebo | `ros_gz_sim/create`, `ros_gz_bridge/parameter_bridge`, `limo_capstone/ground_truth_publisher_node` | 모델, ROS 센서·속도 브리지, /clock, GT | 기존 Gazebo 시험. |
| 센서 전처리 | `limo_scan_normalizer/scan_normalizer_node`(현장 번들), `limo_depth_obstacles/depth_obstacle_filter_node`, `limo_capstone/sensor_normalizer_node`·`depth_to_pointcloud_fixed_node`(모의) | /scan → /scan_slam, 촬영 시각 TF와 포인트클라우드 → /depth_obstacles/points | 기존 실차·Gazebo 설정별 관측. |
| 위치·TF | `limo_capstone/complementary_filter_node`, `lidar_scan_matcher_odom_node`, `odom_tf_relay_node`, `robot_localization/ekf_node`, `tf2_ros/static_transform_publisher`, 모의 `robot_state_publisher` | 센서·휠 odom → 추정기 선택 → odom→base. URDF·정적 변환. | 기존 실차·Gazebo 추정기별 기록. |
| SLAM·탐색 | `slam_toolbox/async_slam_toolbox_node`, `explore_lite/explore`, 독립 후보 `fleet_frontier_candidates` | /scan_slam 및 TF → /map, map→odom, serialize_map. Costmap → Frontier → NavigateToPose. | 기존 실차 Frontier 경로. UUID 취소 수정은 격리 Mock Nav2. |
| Nav2 | `nav2_bt_navigator/bt_navigator`, `nav2_planner/planner_server`, `nav2_controller/controller_server`, `nav2_behaviors/behavior_server`, `nav2_velocity_smoother/velocity_smoother`, `nav2_lifecycle_manager/lifecycle_manager` | NavigateToPose → 경로·FollowPath. 현장 launch controller → cmd_vel_nav → smoother → /cmd_vel_raw. Behavior 출력도 Gate 입력. Collision Monitor는 해당 프로파일에 위치. | 기존 실차·Gazebo 프로파일별 기록. |
| Safety Gate | `limo_capstone/stall_detector_node` | 깊이·LiDAR·odom·TF와 /cmd_vel_raw → /cmd_vel_locked 또는 구동 승인 구성의 /cmd_vel. /stall_detector/status_json. | 기존 실차 잠금 및 Gazebo. |
| Goal 관리·Fleet | `limo_robot_agent/agent_node`, `limo_rmf_adapter`, HTTP `fleet_server` | NavigateToPose와 _action/status, Lease·Journal, RMF principal·idempotency key | 격리 Mock 및 기존 Gazebo SIM. 웹 수동은 발행 없는 모의 입력. |
| 영상·기록 | `dashboard.bridge_node`, `dashboard.readonly_rgb`, 별도 RGB/AI 호스트, `nav2_map_server/map_saver_cli`, `ros2 bag record` | 읽기 전용 지도·Depth·TF, RGB 인증 전달, YAML/PGM·posegraph·bag | 기존 실차 읽기 전용, 이번 Chrome 합성 ROS·모의 RGB. |
### 현재 실행 구조
flowchart LR
  IO["LIMO 센서 / Gazebo 브리지"] --> Prep["Scan·Depth 전처리"]
  IO --> TF["Odometry·TF"]
  Prep --> SLAM["SLAM Toolbox"]
  TF --> SLAM
  SLAM --> Nav["공용 Nav2"]
  Prep --> Nav
  TF --> Nav
  Explore["LOCAL_FRONTIER · explore_lite"] -->|NavigateToPose| Nav
  Agent["Fleet / RMF → Lease·Journal → Agent"] -->|NavigateToPose| Nav
  Nav --> Smooth["Smoother / 프로파일별 Collision Monitor"]
  Smooth --> Gate["Safety Gate /cmd_vel_raw"]
  Nav -->|현장 behavior remap| Gate
  Gate --> Output["잠금 출력 또는 승인된 베이스 경로"]
  SLAM --> View["읽기 전용 지도·RGB·Depth"]
  Prep --> View
  IO --> RGB["별도 인증 RGB/AI 호스트"] --> View
  SLAM --> Store["YAML/PGM · posegraph · rosbag"]
### 제안하는 분리 구조
flowchart LR
  Goals["단일 모드의 Goal 제출자"] --> Nav["공용 Nav2"]
  Nav --> N["Nav 전용 입력 · 기존 Smoother"]
  Pad["Gamepad"] --> Agent["Fleet Lease · Agent · deadman/만료"]
  Agent --> M["Manual 전용 입력"]
  N --> Arb["Velocity Arbiter · 단일 모드 선택"]
  M --> Arb
  Nav -. "Goal terminal 확인" .-> Arb
  Arb --> CM["해당 프로파일의 Collision Monitor"]
  CM --> Gate["기존 Safety Gate"]
  Arb -->|Collision Monitor 없는 프로파일| Gate
  Gate --> Lock["격리 검증에서는 잠금 경로"]
제안도의 ROS 속도 라우팅은 미구현이다. 이번 모의 제어는 Agent에서 선택된 값을 상태로 보고한다. LOCAL_FRONTIER는 UUID와 현장 run 디렉터리, FLEET_NAVIGATION은 Mission ID와 Agent UUID, RMF_TASK는 RMF principal과 idempotency key로 추적한다. WEB_MANUAL은 Action Goal을 생성하지 않는다. 전용 모드별로 Goal 제출자가 하나라면 별도 Goal Manager를 도입할 필요가 없다. 같은 Nav2에 여러 독립 제출자를 허용하려면 관측 검사에 더해 원자적 소유권 중재가 필요하다.
### Mapping Session 및 관제 경계
Perception·Localization·Safety·Bringup의 기존 launch와 패키지 배치를 유지했다. Mapping 명령의 중앙 계약은 `fleet_control/server/fleet_server/mapping.py`에 있고, Navigation의 기존 Mission·Lease·Recovery와 SQLite를 공유한다. 명령과 결과는 별도 Mapping/Bag 저널에 기록한다. 웹 Gamepad는 Teleoperation에서 소유 Lease 동안 모의 입력을 제공한다. Mapping이 STOPPED로 확인되기 전에는 Fleet 목적지 dispatch와 수동 입력을 차단한다. 모드 간 ROS 속도 및 Gate 연결은 기존 분리 제안 범위에 남아 있다.
브라우저는 목적지 Goal, 영상, Mapping과 Bag을 서로 다른 계약으로 다룬다. 현재 Mapping/Bag 실행 증거는 격리 시험의 mock worker가 보낸다. 설치된 SLAM, Frontier, 저장기 또는 기록기와 연동하려면 실제 worker에서 상태·소유 프로세스·Goal UUID를 관측하고 별도 시험해야 한다. 동시 최초 Goal 제출을 위한 원자적 ROS 중재는 아직 없다.
### 전체 기능 구현 우선 격리 경로
`limo_fleet_bringup`은 기존 Perception·Localization·Navigation·Safety 노드 및 Fleet Bridge를 기능별 launch 모듈로 조합한다. Mapping은 Agent가 SLAM Toolbox와 Frontier 후보 프로세스를 소유한다. Fleet Server는 Mapping 명령 저널에 Frontier 슬롯 예약을 같은 Agent boot와 session으로 기록한다. Agent는 Fleet·Frontier 제출을 단일 ROS 작업 큐와 공유 Goal 슬롯으로 중재한다. PAUSE/STOP은 소유 Action 핸들이 반환한 terminal 결과를 기다린다.
격리 속도 체인은 Nav2의 `/fleet/cmd_vel_nav`와 Lease가 검증된 웹 입력에서 샘플 하나를 골라 Smoother → `/cmd_vel_raw` → 기존 Safety Gate → `/cmd_vel_locked`로 전달한다. Bag recorder는 Agent 밖의 세션 Supervisor가 소유하며 익명 pipe로 Fleet 연결 상태를 갱신한다. 로봇별 RGB·Depth는 Agent에서 읽기 전용으로 수집해 Fleet 관제 화면에 표시한다. 인터페이스와 하네스는 [ISOLATED_E2E.md](fleet_control/docs/ISOLATED_E2E.md)에 정리했다.
