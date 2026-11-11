# LIMO 추가 시험본: middle / RPP / 복구 BT

사용자의 USB `capstone1/capstone_ws`를 기존 로봇 구성으로 삼았다. 기존 노트북 소스·USB·앞서 만든 Python 자동 맵핑 시험본을 수정하지 않고 새 복사본을 준비했다. 사용자 기록의 실물 수동 맵핑·3층 자동 맵핑·저장 지도 목표 주행과 이번 새 시험본의 검증 범위는 구분한다.

## 옮길 파일

`LIMO_trial_sources.zip`의 `src/` 안에 필요한 **5개 패키지 전체**를 모았다. 기존 `capstone_ws`에 추가하거나 별도 시험 workspace를 사용한다. USB의 기존 패키지와 이름이 겹치지 않는다. 노트북 `build/`, `install/`, `log/`, Gazebo world, 기존 지도는 포함하지 않았다.

| 패키지 | 내용 | 이번 요청에서 |
|---|---|---|
| `limo_auto_mapping` | 앞서 만든 Python 자동 맵핑 Action·탐색기. 한국어 주석 포함 | 이전 2개 중 하나, 내용 유지 |
| `limo_mapping_interfaces` | Python Action 메시지 정의 | 이전 2개 중 하나, 내용 유지 |
| `limo_onboard_trials` | Python 시험 launch와 설정·BT 복사본 | 추가 |
| `limo_mapping_explore` | 기존 노트북의 C++ middle 수정본을 별도 이름으로 복사 | 추가 |
| `limo_mapping_explore_msgs` | middle 복사본이 사용하는 상태 메시지 | 추가 |

앞의 두 패키지를 이미 옮겼다면 뒤의 세 개만 추가한다. 이전 최초 C++ Action 묶음을 옮겼던 경우에는 `limo_mapping_explore*` 이름이 이미 있을 수 있으므로 같은 이름의 패키지를 두 경로에 두지 말고 이 시험본은 별도 workspace에서 빌드한다. 현재 USB 기존 구성에는 이 이름이 없다.

### Python 선호와 middle 코드의 관계

새 시험 실행·설정 구성 코드는 Python으로 작성하고 한국어 주석을 붙였다. **이전에 노트북에서 시험한 `middle` 변경을 그대로 비교하려면 기존 C++ 탐색 소스를 그대로 사용해야 한다.** 그래서 탐색기는 원본을 고치지 않고 복사했다. 새로 만든 Python 탐색기의 대표점 선택으로 대체하면 원래 middle 변경과 다른 시험이 된다.

`middle` 복사본의 `explore.cpp`는 노트북 기존 파일과 상태 메시지 패키지 이름을 바꾼 것 외에는 동일하다. 목표 선택·blacklist 판정·시각화의 세 지점이 모두 `middle`을 사용한다. 기존 USB 탐색기 `explore_lite`는 centroid 기준이며 그대로 둔다. 참고로 이 소스에서 middle은 이름과 달리 프론티어 그룹의 단순 중심점이 아니라 로봇과 가까운 프론티어 셀을 저장하는 필드다. Python 탐색기는 free 셀 대표점을 선택하는 별도 구현이다.

## 시험 조건을 분리한 이유

모든 변경을 동시에 켜면 어느 변경이 주행 결과에 영향을 줬는지 구분하기 어렵다. 다음 profile을 순서대로 비교할 수 있다.

| profile | Controller | NavigateToPose BT | 유지하는 구성 |
|---|---|---|---|
| `usb` | USB 자동 맵핑 YAML의 DWB | Nav2 기본 BT | USB의 nav2_explore_params.yaml 값 유지 |
| `bt` | 기존 DWB | Spin 없는 Wait·Clear·BackUp BT | 센서·AMCL·costmap·속도·도착 공차는 USB 기준 |
| `rpp_bt` | RPP | 위와 같은 BT | FollowPath와 BT 선택 외에는 USB 기준 |

- `nav2_usb.yaml`, `slam_params.yaml`, `explore.yaml`은 USB 파일의 바이트 그대로다.
- USB의 기존 저장 지도 주행 launch는 params_file을 명시하지 않는다. 따라서 `usb` profile이 과거 목표 주행 때의 실제 Nav2 기본값과 동일하다고 단정하지 않는다. 이번 비교에서는 USB 자동 맵핑에 쓰던 nav2_explore_params.yaml을 공통 기준으로 명시해 조건을 고정했다.
- `nav2_bt.yaml`은 USB 기준에서 BT 경로 키만 추가했다.
- `nav2_rpp.yaml`은 USB 기준에서 FollowPath 블록을 기존 실물 RPP 시험 준비 파일의 RPP 블록으로 바꾸고 BT 경로 키를 추가했다. 충돌 검사 활성화, 예측 시간 0.8초 등 준비 파일의 RPP 값은 유지한다.
- 기본 BT의 후진은 0.15m, 속도 0.05m/s다. `backup:=30`으로 기존 0.30m 후진 변형도 별도 선택할 수 있다. 기본은 15cm다.
- 기존 시뮬레이션의 예측 시간 축소를 별도로 비교하려면 `profile:=rpp_bt collision_time:=0.6`을 명시한다. 예측 시간 단축은 안전 강화와 같지 않으며 자동으로 켜지 않는다.
- 시뮬레이션의 느슨한 도착 공차, AMCL 변경, 긴 후진 등을 기본 조건에 한꺼번에 적용하지 않았다. USB의 도착 공차를 유지하므로 기존 시뮬레이션 전체 설정과 동일한 재현은 아니다.
- RPP는 기존 시뮬레이션/시험 준비 설정에서 가져왔다. 실제 LIMO 모드에서의 회전·목표 자세 도달·좁은 통로 성능은 새로 확인해야 한다. 드라이버의 조향 모드를 바꾸는 기능은 추가하지 않았다.

## LIMO에 복사하고 빌드

압축 파일은 새 파일 이름의 패키지 5개만 포함한다. 이미 같은 이름의 시험본을 설치했다면 기존 사본을 보존하고 **별도 시험 workspace**를 쓰는 것이 좋다. 다음은 경로 예시다. 기존 workspace 경로가 다르면 실제 경로를 사용한다.

```bash
mkdir -p ~/limo_trial_ws
unzip LIMO_trial_sources.zip -d ~/limo_trial_ws
cd ~/limo_trial_ws
source /opt/ros/humble/setup.bash
source ~/capstone_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

RPP 패키지가 로봇에 없다면 위 의존성 설치에 포함된다. 설치 여부는 다음으로 확인할 수 있다.

```bash
ros2 pkg prefix nav2_regulated_pure_pursuit_controller
```

모든 시험 터미널에서 ROS, 기존 LIMO workspace, 새 시험 workspace 순서로 source한다. 드라이버·URDF·LiDAR는 기존 구성으로 한 번만 실행한다. 기존 SLAM·AMCL·Nav2·explore_lite·Python 자동 맵핑 Action 등 같은 역할의 노드는 종료하고 선택한 시험 스택만 실행한다. 이번 시험에는 기존 Python 자동 맵핑 Action을 함께 실행하지 않는다.

## 1. 저장 지도에서 BT / RPP 비교

기존 LIMO 지도 `limo_capstone/maps/MC_3F_10_15.yaml`을 기본으로 읽는다. 지도 YAML과 이미지가 실제 설치 위치에 있어야 한다. 다른 지도는 `map:=<지도 YAML 절대경로>`로 지정한다.

기준 시험:

```bash
ros2 launch limo_onboard_trials navigation_trial.launch.py profile:=usb
```

로봇을 정지하고 launch를 종료한 뒤 BT만 바꿔 같은 경로를 시험한다:

```bash
ros2 launch limo_onboard_trials navigation_trial.launch.py profile:=bt
```

같은 방식으로 RPP+BT를 비교한다:

```bash
ros2 launch limo_onboard_trials navigation_trial.launch.py profile:=rpp_bt
```

각 실행에서 RViz 초기 자세·scan 정합·TF를 확인한 뒤 동일 목표를 지정한다. AMCL 초기 자세를 설정하지 않고 주행 결과를 비교하지 않는다. 이 launch 자체는 주행 goal을 보내지 않는다. profile 간 비교는 같은 시작 자세·목표·지도·주행 모드·장애물 조건을 사용한다.

## 2. centroid / middle 자동 탐색 비교

먼저 맵핑 스택만 실행한다. 첫 비교는 컨트롤러·BT를 바꾸지 않는다:

```bash
ros2 launch limo_onboard_trials navigation_trial.launch.py mode:=mapping profile:=usb
```

별도 터미널에서 `/scan`, `map→base_link`, `/map`, `/global_costmap/costmap`을 확인한 뒤 기존 탐색기를 실행한다. **이 명령부터는 탐색 목표를 보내므로 실제 로봇이 움직일 수 있다.**

```bash
ros2 launch limo_onboard_trials exploration_trial.launch.py target:=centroid
```

비교를 위해 완전히 새 SLAM 세션에서 같은 시작점·같은 조건으로 반복하고, 두 번째 세션에서는 다음을 실행한다. 기존 지도 위에서 계속 실행하는 것과 새 SLAM 세션 비교는 다른 시험이다.

```bash
ros2 launch limo_onboard_trials exploration_trial.launch.py target:=middle
```

탐색기 두 개를 동시에 실행하지 않는다. 완료 상태는 탐색기가 더 이상 시도할 frontier를 찾지 못했다는 뜻이며 건물 전체 맵핑 성공을 보장하지 않는다.

필요하면 첫 비교가 끝난 뒤 `mode:=mapping profile:=rpp_bt`와 `target:=middle`을 조합해 통합 시험할 수 있다. 이 조합에는 Python Action의 자동 마진 축소·저장·이어 맵핑 기능이 연결되어 있지 않다. 기존 개선과 새 Action을 처음부터 한 시험으로 섞지 않기 위해 분리했다.

이 middle 비교 세션은 자동 저장 Action이 아니므로 필요하면 별도로 저장한다:

```bash
ros2 run nav2_map_server map_saver_cli -f <새_지도_prefix>
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '<새_posegraph_prefix_절대경로>'}"
```

탐색 중단은 `/explore/resume`에 false를 보내고, Nav2 goal도 취소한 후 실제 정지를 확인한다. launch/CLI 종료만으로 정지가 확인되었다고 가정하지 않는다.

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}'
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
```

## 검증 상태와 기록할 항목

**확인됨:** 새 시험 패키지 3개 Humble 빌드, 설정·실행 조건 Python 검사 17개, 기존 C++ 검사 1개. 테스트는 BT 경로가 실제 생성 YAML에 들어가는지, RPP 외의 USB 설정이 유지되는지, map/image 경로 확인, mapping/navigation 모드 분리를 포함한다. C++의 기존 검사는 same_goal 보조 함수 검사이며 탐색·주행 검증은 아니다.

**미검증:** 이번 묶음의 Gazebo 주행·실물 LIMO 실행, middle의 실제 frontier/blacklist 동작, RPP 플러그인 실시간 제어·BT 복구 주행, 장애물 대응, 성공률·오차·최소 간격. 이전 시뮬레이션 결과가 이번 실물 설정의 성능을 증명하지 않는다.

초기 검사 중 Humble `RewrittenYaml`이 존재하지 않는 BT 키를 추가하지 않는 것을 확인했다. 원본 USB YAML에는 그 키가 주석뿐이어서, 시험용 BT/RPP 템플릿에 경로 키를 명시하고 실행 시 설치 경로로 치환하도록 수정했다. 노트북의 고정 경로는 전달하지 않는다.

실물 기록에는 profile, target, 시작 자세, 목표, 지도, 주행 모드, 반복 횟수, 성공/실패, 도착 오차, 장애물 최소 간격, 정지·재출발 여부를 남긴다. 측정하지 않은 항목은 미계측으로 표시한다. 기존 실패 조건인 정면 접근·복합 장애물 조건을 성공한 기능으로 설명하지 않는다.

새 시험 workspace에서 검사 재현(테스트를 포함해 빌드한 경우):

```bash
ROS_LOG_DIR=/tmp/limo-trial-tests PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q src/limo_onboard_trials/test
./build/limo_mapping_explore/test_explore
ros2 launch limo_onboard_trials navigation_trial.launch.py --show-args
```

참고 오픈소스: [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2), [Nav2](https://github.com/ros-navigation/navigation2). 복사한 C++ 탐색 소스의 BSD 저작권·라이선스를 보존했다.
