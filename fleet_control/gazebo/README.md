# Open-RMF × Gazebo Fortress 통합 프로파일

구조는 `Open-RMF → Fleet Adapter → Mission Manager → Robot Agent → NavigateToPose → Nav2 → /cmd_vel_raw → 기존 stall_detector_node → /cmd_vel → Gazebo`입니다. Adapter는 Nav2나 속도 토픽을 직접 제어하지 않습니다. 실차 프로파일이 아닙니다.

## 참조 구성 조사

기존 `limo_integrated_auto_mapping_sim_diff.launch.py`는 Gazebo 차동 구동 모델·센서/bridge·F4·TF relay·SLAM·Nav2·Safety Gate를 함께 시작합니다. `start_explore:=false`로 explore는 끌 수 있지만 SLAM은 별도 선택 없이 실행됩니다. 기존 자동 맵핑 실행 스크립트는 explore를 켜고 새 지도를 만드는 용도이므로 이 시험에서 실행하지 않습니다.

이번 `localized_sim.launch.py`는 설치된 기존 launch를 읽어 액션을 재사용하며 다음만 별도로 구성합니다.

- SLAM 액션을 제외하고 저장 지도 map_server + AMCL을 사용합니다. 자동 탐색은 false입니다.
- 기존 F4는 **Gate가 요구하는 실제 LiDAR odometry/진단 입력으로 유지**합니다. 기존 F4/SLAM/Nav2/Gate 소스는 수정하지 않았습니다.
- Nav2 TF/odometry에는 `/odom_gz_raw → 기존 sensor_normalizer → /odom → 기존 odom_tf_relay_node → /odom_nav2 및 odom→base_footprint`를 선택합니다. 합성 잡음/슬립 주입은 기본값(없음)입니다. Ground Truth는 Nav2에 입력하지 않습니다.
- 초기 센서 준비를 기다릴 수 있도록 Gate **시작만 20초 지연**합니다. enabled·required depth·0.38초 freshness·operator latch 등 Gate의 코드와 설정은 유지합니다. 시험은 Gate NORMAL 및 Nav2 lifecycle 활성화 전에는 목표를 보내지 않습니다. reset/unlatch는 호출하지 않습니다.
- 로봇 모델, world 물리/장애물, 센서, depth 처리, Nav2 파라미터 및 속도 remap은 원본 액션을 유지합니다. Gazebo는 원본과 같은 server-only `-s` 방식이며 센서 렌더링은 필요합니다.

## 좌표계

저장된 `fleet_control/maps/limo_sim_map.yaml/.pgm`을 그대로 사용합니다. 따라서 RMF waypoint는 변하는 SLAM 지도 대신 고정된 Nav2 map 픽셀 좌표에 연결됩니다. AMCL이 실제 scan과 선택된 wheel odometry로 map→odom을 발행하며 GT로 TF를 덮어쓰지 않습니다.

재현 프로파일의 Gazebo spawn은 world `(4.4,-3.0,yaw=π/2)`, AMCL 초기 추정은 map `(-2.4,-5.5,yaw=0)`입니다. 저장 지도의 벽 방향·최초 SLAM 시작 자세에서 얻은 초기 **추정**이며 정밀 world↔map 캘리브레이션이 아닙니다. AMCL 보정 결과를 사용합니다. GT publisher의 `/ground_truth/world_odom` 값은 world 좌표이며, 기존 메시지의 frame_id 표기가 map이라고 해서 Nav2 map과 같은 좌표로 비교하면 안 됩니다. 시험은 GT 변위와 Nav2 위치를 분리합니다.

Gazebo 전용 `rmf_adapter/config/fleet_gazebo.json`은 기존 모의용 `fleet.json`을 보존하고, 짧은 검증에 적합한 free 구간을 선택합니다.

| waypoint | Nav2 map x,y (m) |
|---|---|
| wp_a | -2.400, -5.500 |
| wp_b | -2.880, -4.300 |
| wp_c | -3.054, -3.865 |
| wp_d | -3.228, -3.430 |
| wp_e | -3.400, -3.000 |

4개 양방향 lane a–b–c–d–e입니다. 정적 시험은 모든 waypoint가 free 셀이고 lane 표본이 occupied/unknown 셀에서 0.2m 이상 떨어져 있는지 검사합니다. 이는 2D 지도 여유 검사이지 모든 깊이 장애물/실차 안전 여유를 인증하는 검사가 아닙니다. `p_rmf=R(0.3)·p_nav2+(20,10)`, yaw offset=0.3rad이며 항등 변환을 쓰지 않습니다. wp_a의 charger 플래그는 RMF 등록용일 뿐 실제 충전 시설이 아닙니다.

## Agent 인터록

기존 실차/기본 동작은 유지됩니다. `/cmd_vel` 발행자가 있으면 기본적으로 목표를 거부합니다. 다음 조건이 **전부** 맞는 시뮬레이션에서만 예외를 허용합니다.

1. `mode:=sim`
2. `simulated:=true`
3. `allow_sim_safety_gate:=true` (새 파라미터, 기본 false)
4. `use_sim_time:=true`
5. `/cmd_vel` publisher endpoint가 정확히 1개이며 `/stall_detector_node`임

다른 발행자, 같은 이름의 중복 endpoint, 다른 namespace, 그래프 조회 실패는 거부합니다. `mode=real`은 다른 플래그를 켜도 예외가 적용되지 않습니다. `allow_missions`는 별도로 필요하며 기본 false입니다. 상태의 `SIM_GATED`는 시뮬레이션 Gate 경로를 확인했다는 뜻입니다. 노드 이름은 인증 수단이 아니므로 이 옵션을 실차에서 사용하면 안 됩니다.

## 실행

시스템 패키지 설치/업그레이드는 필요하지 않습니다. 기존 ROS 2 Humble/Gazebo/빌드된 참조 워크스페이스와 사용자 격리 RMF 2.1.8을 사용합니다. 저장소 루트에서:

```bash
export CAPSTONE_REFERENCE_WS="<READ_ONLY_WS>"
source /opt/ros/humble/setup.bash
source "$CAPSTONE_REFERENCE_WS/install/setup.bash"
source "$HOME/rmf_isolated/env.sh"
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/rmf-gazebo-roslogs
python3 -u fleet_control/rmf_adapter/tests/run_gazebo_checks.py
```

시험 기본값은 Mission Manager 18120, WebSocket sink 18121, RMF 도메인162, 로봇 도메인163, 프로세스별 Ignition partition입니다. `GAZEBO_TEST_PORT`, `GAZEBO_TEST_WS_PORT`, `GAZEBO_RMF_DOMAIN`, `GAZEBO_ROBOT_DOMAIN`으로 지정할 수 있으며 운영8080이나 다른 실행 중 시험과 겹치면 안 됩니다. 전체 watchdog은 `RMF_E2E_TIMEOUT_S`(기본1800초)입니다.

스크립트는 자체 임시 토큰/DB/저널/로그를 생성하고, 소유한 process group과 PID 생성 시각을 추적해 종료합니다. 이름 기반 pkill을 하지 않습니다. 서버/Agent 중단은 이 시험 인스턴스에만 주입하며 Wi-Fi/Tailscale 설정은 바꾸지 않습니다. raw runtime 로그는 로컬 임시 경로에만 남기며 공개 저장소에 넣지 않습니다.

## 검증과 한계

최종 수치·시간은 [통합 결과](RESULTS.md)와 상위 [VALIDATION](../docs/VALIDATION.md)에 기록합니다. 15개 필수 항목, 실행/취소/lease/서버·Agent·Adapter 재시작, native Nav2 UUID와 Agent 수락 응답/서버 명령 대조를 사용합니다. 가까운 waypoint 병합 시에도 `Start.location`에 실제 변환 좌표를 보존하도록 수정하고 좌표 오차 <0.1m를 검사합니다. 거절된 RMF 작업과 이전 작업의 후속 위치 보정 임무도 구분합니다.

초기 장거리 F4-TF 프로파일에서는 큰 map/GT 불일치와 경로 실패가 관측됐습니다. F4 단독 결함으로 원인을 확정한 것은 아닙니다. 이번 결과는 wheel-TF + AMCL + 짧은 free 구간에 한정하며, 장거리/다양한 장애물/전구간 회귀나 정밀 물리 보정으로 확대 해석하지 않습니다. F4 입력을 완전히 제거하면 Gate의 필수 입력이 없어 INITIALIZING/FAILSAFE가 되는 실패도 확인했고, 이를 끄거나 우회하지 않고 입력을 복원했습니다.

실제 LIMO 접속·구동·배포는 하지 않았습니다. 운영8080, 읽기 전용 참조 소스, 팀 워크스페이스 및 기존 모의 fleet.json은 변경하지 않았습니다. RMF 2.1.8 Adapter 재시작 후 이전 작업이 underway로 남는 기존 한계도 그대로입니다. 새 라이선스를 추가하지 않았습니다.
