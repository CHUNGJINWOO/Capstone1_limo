# 자동 맵핑 Action 시험용 복사본 — 2026-11-03

기존 노트북 소스와 USB 파일을 수정하지 않고 만든 별도 ROS 2 Humble 워크스페이스다. **이번 작업은 노트북에서만 수행했다. 실제 LIMO에는 복사·설치·실행하지 않았다.** USB의 `capstone1/capstone_ws`를 LIMO 구성의 비교 기준으로 사용했으며, 현재 로봇 내부와 동일한지는 직접 접속해 확인하지 않았다.

## 결과와 검증 범위

| 구분 | 확인 내용 |
|---|---|
| 구현 | `/auto_mapping` Action, 단일 세션 수락, 진행 피드백, 취소, 최대 실행시간 |
| 구현 | 기존 frontier 탐색 → 제한적 inflation 축소 판단 → 최대 1회 새 탐색 프로세스로 재시도 → 원래 값 복원 |
| 구현 | 지도 이미지/YAML 및 SLAM pose graph 쌍 저장, pose graph와 시작 자세를 지정한 이어 맵핑 launch |
| 노트북에서 검증 | Humble에서 4개 패키지 빌드, Python 24개 검사, 기존 C++ 검사 1개, launch 인자 로딩 |
| 노트북에서 검증 | 격리된 로컬 ROS domain에서 실제 Action 통신을 사용한 수락·중복 거절·취소·성공 결과 전달. 맵핑 백엔드는 모의 구현 |
| 미검증 | 실제 탐색 프로세스와 Nav2의 통합 주행, 동적 inflation 변경/복구, SLAM 저장·재개 실동작, LIMO 주행·충돌 여유·지도 품질 |
| 미수행 | 이번 구현으로 Gazebo 주행, 실물 LIMO 시험, USB 쓰기, 기존 코드 변경, Git 커밋·푸시 |

빌드의 `CATKIN_INSTALL_INTO_PREFIX_ROOT` 미사용 CMake 경고 1건은 있었으며 빌드는 성공했다. 처음 Python 검사 실행은 이 노트북의 외부 pytest 플러그인 버전 충돌로 실패했다. 해당 플러그인 자동 로딩을 끈 후 아래 검사 명령으로 통과했다. 이 설정은 테스트 실행에만 사용한다.

## USB와 노트북 기존 파일 차이

`capstone_ws/src` 전체를 파일 해시로 비교했다. build/install/log, `.git`, Python 캐시는 제외했다. **노트북에만 14개, USB에만 9개, 양쪽에서 내용이 다른 파일 6개**다. 전체 목록은 `source_comparison.json`, 비교 당시 양쪽 원본 해시는 `baseline_hashes.json`에 있다. 파일의 차이가 실물에서의 실행·검증 이력을 뜻하지는 않는다.

| 파일/묶음 (`capstone_ws/src` 기준) | USB 기준과 노트북 차이 | 이번 실기기 시험에 전달 |
|---|---|---|
| `limo_capstone/config/explore.yaml` | 주석만 다름. 파라미터 값 동일 | 기존 파일 덮어쓰지 않음 |
| `limo_capstone/config/nav2_explore_params.yaml` | 주석만 다름. 파라미터 값 동일 | USB 버전을 새 패키지에 복사 |
| `m-explore-ros2/explore/src/explore.cpp` | USB는 centroid, 노트북은 middle을 목표점·blacklist 판단에 사용 | 새 이름의 복사 패키지에 middle 변경 반영 |
| `limo_capstone/launch/limo_nav2.launch.py` | 노트북은 기본 지도 MC_3F_10_15, params_file 인자, 정적 base_footprint TF 추가. USB는 map_limo, 이 추가 구성 없음 | 전달 불필요 |
| `limo_capstone/setup.py` | 노트북에 worlds/scripts/ablation/XML 설치 항목 추가 | 전달 불필요 |
| `limo_capstone/maps/MC_2F_10_07.png` | 이미지 바이트 다름. 지도 품질 차이는 미평가 | 전달 불필요 |
| 노트북 전용 14개 | 시뮬레이션 YAML 2개, Ackermann BT XML 2개, ablation YAML 4개, simulation launch 3개, map_to_world.py, SDF world 2개 | 전부 전달 불필요 |
| USB 전용 9개 | 2026_10_21 기록, MC3F_nav2 안 이미지/TF 기록 7개, rosgraph.png | 전달 불필요 |

LIMO 드라이버·URDF·LiDAR·SLAM 설정 파일은 이 비교에서 별도의 바이트 차이가 발견되지 않았다. 저장소 최상위 `src/limo_ros2`는 USB workspace와의 직접 비교 대상이 아니며 수정하지 않았다.

## LIMO에 옮길 것: 새 소스 4개 폴더만

`ws/src/` 아래 다음 **새 패키지 전체**를 별도 시험 workspace의 `src/`로 복사한다. `limo_auto_mapping_trial_source.tar.gz`에도 이 네 패키지와 이 문서만 담았다.

| 폴더 | 필요한 이유 |
|---|---|
| `limo_mapping_interfaces/` | `action/AutoMapping.action` 및 인터페이스 빌드 |
| `limo_auto_mapping/` | Action 서버, 경계 판별, launch, USB에서 복사한 설정 3개 |
| `limo_mapping_explore/` | USB explore_lite의 독립 복사본. 패키지 이름 변경, middle 목표점, 탐색 소진 상태 구분 |
| `limo_mapping_explore_msgs/` | 복사한 탐색 노드가 사용하는 상태 메시지 |

원래 `limo_capstone`, `explore_lite`, `explore_lite_msgs`는 덮어쓰지 않는다. 패키지 이름을 달리해서 기존 설치와 구분했다. 노트북의 `build/`, `install/`, `log/`, 시뮬레이션 world·지도·추가 BT 파일은 옮기지 않는다. ARM LIMO에서는 소스를 다시 빌드해야 한다. `COLCON_IGNORE`는 상위 프로젝트에서 실험 폴더가 자동 수집되지 않도록 둔 것으로, 별도 `ws` 안에서 빌드하면 된다.

이어 맵핑 시험에는 추가로 **동일 세션에서 저장된 `<prefix>.posegraph`와 `<prefix>.data` 두 파일**이 필요하다. PGM/YAML은 시각화·일반 localization용으로도 보관한다.

## 동작과 안전마진 재시도 조건

1. Launch는 SLAM Toolbox mapping 모드, Nav2 navigation 스택, 대기 상태의 Action 서버를 실행한다. Action goal 전에는 탐색 노드를 실행하지 않는다. 드라이버·URDF·LiDAR는 기존 LIMO 구성을 별도로 실행한다.
2. `/scan`, `/map`, 전체 global costmap의 수신·타임스탬프, `map→base_link`, scan 시각의 센서 TF를 확인한다. 정상 수신이 끊기면 탐색을 중단한다.
3. 복사한 오픈소스 frontier 탐색을 실행한다. 탐색이 멈추면 프로세스 종료와 Nav2 goal 취소를 확인한다.
4. `allow_margin_retry: true`인 경우에만 원본 map의 free 영역과 연결된 미탐색 경계를 조사한다. 해당 free cell에 global costmap 비용이 `1..98`로 나타나는 경우를 **inflation에 가려졌을 가능성**으로 본다. unknown(-1), inscribed(99), lethal(100)은 축소 근거로 삼지 않는다.
5. global/local의 원래 `inflation_radius`, `robot_radius`, footprint, inflation 활성화를 읽는다. polygon footprint는 자동 축소를 거절한다. 기본값 `0.30 → 0.24 m`를 적용하되 각 costmap의 `robot_radius + minimum_clearance(0.05 m)` 이상인지 확인한다. `robot_radius`나 footprint, 장애물 관측 레이어는 줄이거나 끄지 않는다.
6. 신선한 전체 costmap 갱신을 기다린 후 기존 후보가 실제 free(0)가 되었는지 확인한다. Nav2 `ComputePathToPose`로 경로가 생성되고 끝점이 후보 근처인지, 경로 점들이 known 공간에 있는지 검사한다. 실패하면 원복하고 재시도하지 않는다. 통과한 경우 새 탐색 프로세스를 한 번 실행해 blacklist도 초기화한다.
7. 종료·취소·오류 시 양쪽 inflation 값을 복원하고 readback으로 확인한다. 복구 실패 시 다음 Action 수락을 막는다. 일반 SIGINT/SIGTERM은 ROS 통신을 유지하며 정리를 시도한다. 전원 차단·SIGKILL·통신 단절에는 복원을 보장할 수 없다.
8. 정상 탐색 종료 시 이미지와 pose graph를 저장한다. 미탐색 경계나 소진된 frontier가 남으면 **부분 지도 저장 + Action ABORTED**를 반환한다. 오류·취소·시간초과에서는 자동 저장을 보장하지 않으며 중간 파일이 남을 수 있다. 기존 출력 파일은 덮어쓰지 않는다.

**inflation_radius는 물리적인 최소 이격거리와 동일하지 않다.** 원인 판정은 비용 비교에 의한 가설이며 동적 장애물 변화와 완전히 분리된 증명은 아니다. 경로 생성도 실제 주행 가능성의 증명이 아니다. 작은 구역의 경계만 보거나 센서 사각지대가 남는 경우도 있어 완전한 건물 커버리지를 보장하지 않는다. 비용 비교나 경로 검사가 모호하면 축소 탐색을 하지 않는다. 실측 차체 크기·회전 궤적·주행 모드에 대한 확인은 LIMO 시험에서 필요하다. 복사된 USB 설정은 DWB 기반이며 Ackermann 모드용 경로 추종 설정으로 검증된 것이 아니다.

## LIMO 빌드와 실행

아래 `~/auto_mapping_trial_ws`는 새 시험 폴더의 예시다. 기존 workspace 위치가 다르면 해당 설치 경로로 바꾼다. 모든 서버와 SLAM은 **LIMO 온보드 PC에서 실행**한다. 저장 경로도 LIMO의 경로다.

압축 파일을 LIMO의 임의 폴더에 옮긴 후:

```bash
mkdir -p ~/auto_mapping_trial_ws
tar -xzf limo_auto_mapping_trial_source.tar.gz -C ~/auto_mapping_trial_ws --strip-components=1
cd ~/auto_mapping_trial_ws/ws
source /opt/ros/humble/setup.bash
source ~/capstone_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

기존 workspace의 드라이버·URDF·LiDAR 구성(예: 기존 `limo_light.launch.py`)을 한 번만 실행한다. 이미 실행 중이면 추가 실행하지 않는다. 기존 자동 맵핑·explore_lite·AMCL·다른 Nav2 launch는 종료한 뒤 새 launch를 사용한다. 이 Action은 **Nav2를 독점 사용**하며 정리 과정에서 해당 Nav2의 모든 NavigateToPose goal을 취소한다. RViz goal·순찰 등 다른 goal을 동시에 보내지 않는다.

터미널 1, 새 지도:

```bash
source /opt/ros/humble/setup.bash
source ~/capstone_ws/install/setup.bash
source ~/auto_mapping_trial_ws/ws/install/setup.bash
ros2 launch limo_auto_mapping auto_mapping.launch.py
```

터미널 2, 위 환경을 동일하게 source한 후 먼저 센서와 TF를 확인한다:

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /global_costmap/costmap --once
ros2 action list -t
```

첫 시험은 마진 축소 없이 실행한다. 출력 prefix는 매 시험 새 이름을 사용한다:

```bash
mkdir -p ~/mapping_trials
ros2 action send_goal /auto_mapping limo_mapping_interfaces/action/AutoMapping \
  "{output_prefix: '$HOME/mapping_trials/trial01', allow_margin_retry: false, max_duration_sec: 600.0}" --feedback
```

다음 시험에서 `allow_margin_retry: true`와 새 prefix를 사용하면 조건부 마진 재시도를 켠다. 초기 실기기 시험은 작은 통제 구역에서 비상정지를 사용할 수 있게 하고 진행한다. 피드백은 `waiting_for_inputs`, `exploring`, `exploring_reduced`, `saving`을 구분한다.

Action 취소는 별도 터미널에서 다음과 같이 한다. goal을 보낸 CLI의 Ctrl+C만으로 서버 goal이 취소되었다고 가정하지 않는다:

```bash
ros2 service call /auto_mapping/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
```

실물 정지와 원래 값(USB 기본 `0.30`) 복원을 확인한다. 강제 종료 등으로 복구되지 않았다면 로봇을 정지하고 양쪽을 수동 복원한 후 정상 스택을 재시작한다:

```bash
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.30
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.30
```

시험 전 값을 바꿨다면 `0.30` 대신 실제 원래 값을 사용한다.

## 기존 지도를 이어 맵핑하기

SLAM Toolbox 공식 serialization 방식으로 저장된 `.posegraph`와 `.data`가 있어야 한다. 이번 USB workspace에서는 해당 쌍을 찾지 못했고, 기존 설정의 `map_file_name`도 주석 상태였다. 이는 저장·재개 구성 부재의 근거이며 기존 실물 실패의 유일한 원인으로 확정한 것은 아니다.

새 Action으로 첫 세션을 종료하면 이미지와 pose graph가 같은 prefix로 저장된다. 기존 SLAM 세션이 아직 살아 있다면 먼저 수동으로 다음과 같이 pose graph를 보존할 수도 있다:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/mapping_trials/seed01'}"
```

SLAM과 Nav2 기존 launch를 종료한 후, 로봇의 현재 위치를 **저장 지도 좌표계의 x/y/yaw(rad)**로 확인해서 새 launch에 전달한다. 아래 좌표는 형식 예시이므로 실제 자세로 바꾼다:

```bash
ros2 launch limo_auto_mapping auto_mapping.launch.py \
  posegraph:="$HOME/mapping_trials/seed01" start_pose:="[1.2, 0.8, 0.0]"
```

이 launch는 파일 쌍과 유한한 3개 자세 값을 검사하고, SLAM Toolbox에 `mode: mapping`, `map_file_name`, `map_start_pose`를 전달한다. AMCL이나 static map_server로 PGM을 로드해서 이어 맵핑하는 방식은 아니다. **RViz에서 이전 지도와 scan 정합, map→odom을 확인한 뒤 Action을 보낸다.** 파일 존재 검사는 SLAM의 성공적인 역직렬화·정합까지 보장하지 않는다. 재개 검증에는 이전 구역 보존과 새 구역 추가를 모두 확인해야 한다. 출력은 seed와 다른 새 prefix로 지정한다.

PGM/YAML밖에 없다면 pose graph를 원본에서 추가로 확보하거나 새로 맵핑해야 한다. PGM을 억지로 pose graph로 변환하는 기능은 구현하지 않았다.

## 실물에서 기록할 시험

| 순서 | 조건/검증 | 현재 상태 |
|---|---|---|
| 1 | 드라이버 한 개, scan 정상, TF 연결, map와 costmap 갱신 | 미수행 |
| 2 | 축소 꺼짐: 기본 자동 탐색, 성공/부분 완료 구분, 지도 저장 | 미수행 |
| 3 | 통로 진입 전후 costmap 비교. 0.30/0.24 파라미터와 footprint 기록 | 미수행 |
| 4 | 마진 후보 없음·물리적 막힘·경로 실패에서 재시도하지 않는지 | 미수행 |
| 5 | 축소 탐색 중 취소, 센서 중단, 시간초과 시 정지·복구 | 미수행 |
| 6 | 두 저장 파일 존재, 재시작 후 이전 지도 복원·scan 정합 | 미수행 |
| 7 | 새 구역을 추가 탐색하고 이전 지도 보존 여부 확인 | 미수행 |

## 노트북에서 검사 재현

ROS 환경과 이 workspace의 `install/setup.bash`를 source한 후 `ws`에서 실행한다. 아래 domain 값은 검사 전용 예시이며 실제 로봇과 공유하지 않는다:

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=176 MAPPING_TRANSPORT_TEST=1 \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="src/limo_auto_mapping:$PYTHONPATH" \
python3 -m pytest -q src/limo_auto_mapping/test
./build/limo_mapping_explore/test_explore
ros2 launch limo_auto_mapping auto_mapping.launch.py --show-args
```

상위 프로젝트의 ROS build에 실험 폴더를 합치지 않도록 분리했다. 원본 패키지 전체를 복제하거나 기존 네비게이션 구성을 교체할 필요가 없다.

## 근거와 오픈소스

- [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2): USB에 있던 ROS 2 frontier 탐색 소스를 복사하여 사용. BSD 저작권·라이선스 보존. centroid→middle은 노트북에 이미 있던 차이를 반영했다.
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox): pose graph serialization 및 mapping 모드의 map_file_name/map_start_pose에 의한 재개 방식.
- [Nav2 inflation layer](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/): inflation 비용과 footprint는 구분해야 한다.
- 실제 서비스 정의는 노트북 설치 Humble의 `SerializePoseGraph`, `SaveMap`, `ComputePathToPose`를 확인했다. LIMO의 설치 버전과 인터페이스는 시험 전에 다시 확인한다.
