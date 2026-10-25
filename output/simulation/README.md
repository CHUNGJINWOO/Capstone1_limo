# LIMO 가상 주행 실험 코드

이 디렉터리는 기존 실행 경로를 유지하기 위해 `output/simulation`에 둔다. `.gitignore`에서 **검토한 코드·소형 입력만 허용**하고 실행별 로그·영상·캡처·빌드는 제외한다. 새 기능으로 재작성한 코드가 아니라 2026-10-24 시험에 사용한 로컬 소스의 보존과 경로 정리다. 실행별 원본 사본과 요약은 변경하지 않았다.

## 코드 구성

| 경로 | 역할 |
|---|---|
| `evaluation_lib.py` | 평면 형상 간격, 단계별 정지 시간, Costmap 소거 및 독립 판정 |
| `2026-10-24_rigorous/test_*.py` | 공유 평가 함수의 기존 단위 시험 |
| `complex_indoor_env/generate_complex_env.py` | 벽체·지도·고정·이동 장애물 조건 생성 |
| `interactive_sim_manager.py`, `sim_ctl.py` | Gazebo·RViz 실행, 센서·경로 기록, 주행 시작·정지 제어 |
| `run_oscillating_trials.py` | 상하·정면 접근 왕복 장애물 시험 |
| `continuous_motion/` | Gazebo 물리 스텝 기반 연속 이동 플러그인과 다수 장애물 시험 |
| `run_crossing_experiments.py`, `run_video_scenarios.py` | 협로 횡단 조사와 영상용 시험 실행 |
| `2026-10-24_verification/`, `2026-10-24_rigorous/` | 초기 미로·단계 분리·비교 시험 코드와 소형 입력 |
| `2026-10-24/inputs/` | 초기 방 환경·접촉 보정 모델 및 영상 기록 보조 코드 |
| `render_*`, `continuous_motion/render_trial.py` | 저장 로그·카메라 프레임의 보조 영상 생성 |

결과와 판정 기준: [실험 요약과 한계](../../docs/experiments/2026-10-24/README.md).

## 실행 전 확인

Ubuntu 22.04·ROS 2 Humble·Gazebo Fortress(ignition-gazebo6), Nav2, `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, NumPy·Pillow·PyYAML을 사용했다. GUI 시험은 X11 데스크톱, RViz2, `xwininfo`와 GStreamer 녹화 도구가 필요하다. 연속 이동 플러그인은 CMake·C++ 컴파일러·ignition-gazebo6·ignition-plugin1 개발 패키지를 사용한다.

**새 복제본에서 전체 실행이 즉시 되는 상태로 검증하지 않았다.** 기존 `limo_mc3f_sim.launch.py`에는 저장소 밖 `simulator-research/limo_sim_ws`의 URDF·RViz·메시 자산 경로가 남아 있다. 초기 모델의 메시 URI와 일부 시험 YAML의 기본 BT 경로에도 당시 머신 경로가 들어 있다. 이 자료는 실행 입력의 보존이며 경로가 다른 머신에서는 해당 자산을 확보하고 현재 머신 경로에 맞춘 **시험 사본**을 만들어야 한다. 이를 하드웨어 설정에 덮어쓰지 않는다. 설치된 설정은 로봇에 옮겼다는 근거가 아니다.

`interactive_sim_manager.py`와 횡단 러너는 `NavigateToPose.Goal.behavior_tree`에 선택한 BT를 직접 전달한다. YAML 기본 BT와 실제 Goal에 적용된 BT를 구분하고 각 실행의 아카이브·SHA-256을 확인한다. 시험 중 여러 설정을 동시에 바꿔 원인을 숨기지 않는다.

## 이번 저장소 정리에서 수행한 검사

저장소 루트에서 실행한다. 아래 검사는 평가 계산·환경 생성·빌드를 확인하는 것이며 새 Nav2 주행 성공을 입증하지 않는다.

```bash
python3 output/simulation/2026-10-24_rigorous/test_obb_sat_clearance.py
python3 output/simulation/2026-10-24_rigorous/test_evaluator_phases.py

source /opt/ros/humble/setup.bash
colcon build --base-paths capstone_ws/src/limo_capstone --packages-select limo_capstone

cmake -S output/simulation/continuous_motion -B output/simulation/continuous_motion/build
cmake --build output/simulation/continuous_motion/build -j2
```

## 환경 준비와 주행 순서

경로·외부 자산을 위 조건에 맞춘 뒤 진행한다. 기존 입력·로그를 보존하려면 별도 복제본에서 환경을 생성한다. 생성기는 동일한 환경 파일을 다시 쓰므로 과거 실행 결과의 입력은 실행별 아카이브를 기준으로 한다.

```bash
# 환경 생성만 수행
python3 output/simulation/complex_indoor_env/generate_complex_env.py
python3 output/simulation/run_oscillating_trials.py --prepare-only
python3 output/simulation/continuous_motion/run_continuous_trial.py --prepare-only

# ROS와 프로젝트 환경 로드 후 GUI 시험을 준비 상태로 실행
source /opt/ros/humble/setup.bash
source capstone_ws/install/setup.bash
python3 output/simulation/sim_ctl.py select C
# 별도 터미널에서 준비 상태 확인 후 목표 전달
python3 output/simulation/sim_ctl.py status
python3 output/simulation/sim_ctl.py start
python3 output/simulation/sim_ctl.py stop
```

왕복 시험은 `run_oscillating_trials.py --stage G` 또는 `--stage H`, 연속 이동 시험은 `continuous_motion/run_continuous_trial.py`로 준비한다. 공통 `sim_ctl.py start`가 목표를 전달한다. GUI 제어기는 자신의 세션 프로세스 그룹을 추적하며 기존 세션을 종료할 수 있으므로 한 번에 하나의 시험을 운용한다. 다른 ROS 프로그램을 일괄 종료하는 명령으로 대체하지 않는다.

새 결과는 시각이 붙은 실행 폴더에 저장된다. 물리적으로 접촉하는 자동차 모델이 아니라 지정 궤적의 운동학적 장애물을 사용하며, 로봇의 정지·재개·우회·도착을 따로 평가한다. 녹화 프레임과 로그 보간 영상도 구분한다.

MC관 카메라 파생 월드 `worlds/mc3f_with_camera.sdf`는 기존 `mc3f_10_15.sdf`에 카메라를 추가한 로컬 생성물이다. 큰 중복 사본 대신 `2026-10-24/inputs/mc3f_demo_camera.fragment.sdf`에 추가 모델 설정을 보존한다. 이 파일은 전체 월드가 아니라 `<world>` 안에 삽입하는 조각이다.
