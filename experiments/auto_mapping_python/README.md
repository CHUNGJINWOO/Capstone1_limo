# LIMO 자동 맵핑 Python 시험본

기존 노트북 소스·USB·이전 C++ 시험본은 수정하지 않고, 별도 복사본에서 탐색과 Action 로직을 Python으로 작성했다. 사용자가 설명한 **실물 수동 맵핑·3층 자동 맵핑·저장 지도 기반 Nav2 목표 주행 기록**과 이번 새 기능의 검증 범위는 별개다. 이번 Python Action·마진 변경·이어 맵핑은 LIMO나 Gazebo에서 시험하지 않았다.

## 옮길 것: 두 폴더 전체

이 문서가 있는 폴더를 기준으로 다음 두 폴더만 LIMO의 **새 시험 workspace `src/`** 안으로 복사한다. 기존 패키지는 덮어쓰지 않는다.

```text
ws/src/limo_auto_mapping/         → <새 시험 workspace>/src/limo_auto_mapping/
ws/src/limo_mapping_interfaces/   → <새 시험 workspace>/src/limo_mapping_interfaces/
```

이전 C++ 시험본의 `limo_mapping_explore`, `limo_mapping_explore_msgs`는 이번 Python 버전에 필요하지 않다. 노트북의 `build`, `install`, `log`도 옮기지 않는다. RPP 시험 파일·Gazebo world·시뮬레이션 설정·별도 BT 파일도 이 시험에는 필요하지 않다. 기존 드라이버·LiDAR·TF·Nav2·SLAM Toolbox 설치가 LIMO에 있어야 한다.

이어 맵핑을 할 때만 같은 세션에서 저장된 `<prefix>.posegraph`와 `<prefix>.data` 쌍을 추가로 옮긴다. PGM/YAML만으로 SLAM 탐색 이력을 복원하는 기능은 없다.

## 직접 읽고 수정할 Python 파일

모두 `limo_auto_mapping/` 패키지 안에 있다. 주요 단계와 안전 판단의 이유에 한국어 주석을 붙였다.

| 파일 | 역할 |
|---|---|
| `limo_auto_mapping/server.py` | `/auto_mapping` Action, 취소·시간제한, 센서·TF 상태 확인, 조건부 마진 재시도, 원복, 지도·pose graph 저장 |
| `limo_auto_mapping/explorer.py` | Nav2에 목표를 보내고 진행·실패·취소를 관리하는 Python 탐색 노드 |
| `limo_auto_mapping/frontier.py` | costmap의 free/unknown 경계 그룹 탐색, free 셀 대표점 선택, 후보 점수·방문 제외 |
| `limo_auto_mapping/policy.py` | 지도 좌표 변환, 원본 지도 미탐색 경계 확인, inflation 비용 비교, 마진 하한 검사 |
| `launch/auto_mapping.launch.py` | SLAM·Nav2·대기 상태 Action 서버 실행, pose graph를 지정한 이어 맵핑 설정 |
| `config/explore.yaml` | Python 탐색 주기·진행 타임아웃·후보 크기·제외 반경. 각 값 옆에 설명 |
| `config/nav2_explore_params.yaml` | USB 기준으로 복사한 기존 DWB/Nav2 설정 |
| `config/slam_params.yaml` | USB 기준으로 복사한 SLAM 설정 |

`limo_mapping_interfaces/action/AutoMapping.action`은 ROS 2 Action 메시지 정의다. 그 패키지의 `CMakeLists.txt`는 인터페이스 생성에 필요하며, 직접 작성한 C++ 탐색 소스는 없다. 기존 Nav2·SLAM Toolbox 내부 구현 언어는 바꾸지 않는다.

## 동작과 기존 C++ 시험본의 차이

- Action 시작 전에는 탐색 노드를 실행하지 않는다. Action 시작 후 Python 탐색 노드가 전체 global costmap에서 free 영역을 따라 미탐색 경계를 찾는다.
- 가까운 경계와 큰 경계를 가중치로 평가한다. 그룹 평균 좌표 자체가 벽에 들어갈 수 있어, 실제 free 셀 중 그룹 중심에 가까운 점을 목표로 고른다.
- 한 번 시도한 목표 주변은 이번 회차에서 제외한다. 성공 응답 후에도 같은 목표를 무한 반복하지 않게 했다. 성공·실패 이후 costmap 갱신을 기다리고 다시 후보를 찾는다. 이 정책은 C++ explore_lite와 완전히 같지 않으며 새로 주행 검증해야 한다.
- 진행이 없으면 현재 Nav2 goal을 취소하고, 결과를 받은 뒤 다음 후보로 넘어간다. 취소 결과가 불명확하면 탐색 오류로 종료한다.
- `allow_margin_retry: true`일 때 원본 지도에 경계가 남고 inflation 비용에 가린 정황이 있으면 global/local inflation을 기본 **0.30 → 0.24m**로 줄인다. 로봇 크기·footprint는 줄이지 않는다. `robot_radius + 0.05m` 하한을 검사하며, polygon footprint는 자동 축소를 거절한다.
- 갱신된 costmap에 새 free 후보가 생기고 Nav2 경로 검사도 통과할 때만 새 탐색 회차를 한 번 시작한다. 그때 방문 제외 목록도 초기화된다. 모호한 경우 원복하고 재시도하지 않는다.
- 종료·취소·오류 때 원래 inflation을 복원하고 값을 다시 읽어 확인한다. 복구 실패 시 다음 goal을 거절한다. 전원 차단·SIGKILL·통신 단절에는 복원을 보장할 수 없다.
- 정상 탐색 종료 시 PGM/YAML과 pose graph 쌍을 저장한다. 경계가 남았거나 후보를 모두 시도했으면 부분 지도 저장 후 Action은 ABORTED다. 취소·시간초과·오류에서는 자동 저장을 보장하지 않는다.

inflation 반경은 실물 최소 이격거리와 같지 않다. 비용 비교는 마진이 원인일 가능성을 판별하는 조건이며, 경로 생성은 실제 주행 가능성의 증명이 아니다. Python BFS의 큰 지도 처리시간도 실물에서 측정하지 않았다. 기본 DWB 설정을 RPP 시뮬레이션 성능과 동일하게 설명하지 않는다.

## LIMO에서 빌드·실행

새 workspace에 두 폴더를 복사한 뒤, LIMO의 기존 ROS·드라이버 workspace 환경을 source하고 그 새 workspace에서 빌드한다. 아래 경로는 사용자가 정할 새 시험 폴더의 예시다.

```bash
source /opt/ros/humble/setup.bash
source ~/capstone_ws/install/setup.bash
cd ~/auto_mapping_python_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
ros2 launch limo_auto_mapping auto_mapping.launch.py
```

드라이버·URDF·LiDAR는 기존 LIMO 구성으로 한 번만 실행한다. 기존 explore_lite·자동 맵핑·AMCL·다른 Nav2 launch는 종료하고 새 launch를 사용한다. 이 시험은 Nav2를 독점 사용하며 정리할 때 모든 NavigateToPose goal을 취소하므로, RViz goal이나 순찰을 동시에 실행하지 않는다.

다른 터미널에서도 같은 환경을 source하고 scan/TF/map/costmap 상태를 확인한 뒤 goal을 보낸다. 첫 시험은 축소 없이 진행한다. 출력 prefix는 LIMO의 새 경로다.

```bash
mkdir -p ~/mapping_trials
ros2 action send_goal /auto_mapping limo_mapping_interfaces/action/AutoMapping \
  "{output_prefix: '$HOME/mapping_trials/python01', allow_margin_retry: false, max_duration_sec: 600.0}" --feedback
```

취소와 복구 확인:

```bash
ros2 service call /auto_mapping/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
```

CLI Ctrl+C만으로 Action 취소가 보장된다고 가정하지 않는다. 실물 정지와 원래 반경 복원을 확인한다. 강제 종료 시 수동 복구가 필요할 수 있다. 기본 원래 값은 양쪽 `0.30`이며, 시험 전에 값을 바꿨다면 실제 원래 값으로 복구한다.

이어 맵핑은 이전 세션 launch를 종료하고 pose graph의 두 파일과 **지도 좌표계의 실제 현재 x/y/yaw(rad)**를 지정한다. 아래 자세는 형식 예시다.

```bash
ros2 launch limo_auto_mapping auto_mapping.launch.py \
  posegraph:="$HOME/mapping_trials/python01" start_pose:="[1.2, 0.8, 0.0]"
```

RViz에서 기존 지도·scan 정합과 TF를 확인한 뒤 새 prefix로 Action을 보낸다. 파일 존재·자세 형식 검사는 SLAM 역직렬화와 정합 성공까지 보장하지 않는다.

## 노트북 검사와 실물 미검증 범위

- ROS 2 Humble에서 새 두 패키지 빌드.
- Python 검사 35개: 기존 Action/마진 정책 검사, Python 후보 선택 검사, 이어 맵핑 launch 인자 검사 포함.
- 별도 로컬 ROS domain에서 Action 수락·중복 거절·취소·결과 전달 확인. 맵핑·지도 저장 백엔드는 모의 구현.
- Python 탐색 노드가 실제 ROS Action 통신으로 모의 Nav2에 free 목표점을 보내고, 모의 지도 갱신 후 탐색 완료를 알리는 것 확인. 실제 이동은 없음.
- 실제 센서·SLAM·Nav2 통합 주행, 파일 저장·재개, 동적 inflation 변경과 복구, 실물 간격·성공률·오차·처리시간은 미검증. Gazebo 주행도 하지 않음.

검사는 이 새 workspace의 `install/setup.bash`를 source한 후 `ws`에서 재현할 수 있다. 실제 로봇과 분리한 domain을 사용한다.

```bash
ROS_LOG_DIR=/tmp/limo-python-test-logs ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=177 \
MAPPING_TRANSPORT_TEST=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q src/limo_auto_mapping/test
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD`는 노트북의 외부 pytest 플러그인 충돌을 피하기 위한 검사 설정이다. 빌드에서 인터페이스 패키지의 `CATKIN_INSTALL_INTO_PREFIX_ROOT` 미사용 CMake 경고가 있었으며 빌드는 성공했다.

참고: [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2)의 frontier 개념, [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)의 pose graph 저장·재개 방식. 기존 C++ 원본과 앞서 만든 4개 패키지 시험본은 보존되어 있다. 노트북/USB 원본 차이 전체는 이전 `auto_mapping_action/source_comparison.json`에 그대로 남아 있다.
