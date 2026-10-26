# Nav2 가상환경 주행 시연 기록 (2026-10-24)

## 목적과 범위

발표에 넣을 시뮬레이션 주행 영상을 실제 Gazebo 실행에서 확보했다. 실제 LIMO 주행 결과가 아니며, MC관을 재현한 환경도 아니다. 기존 사각 방 시험 환경과 지도, 간략한 LIMO Ackermann 모델을 사용했다. 사람·이동 장애물 회피 성공이나 위치 정확도를 입증하는 시험으로 설명하지 않는다.

## 구성

- Gazebo Fortress와 ROS 2 Humble, Nav2 NavFn·RPP 구성.
- 기존 `limo_mc3f_sim.launch.py`에 시험용 월드를 인자로 전달했다. 월드 이름은 기존 런치 호환을 위해 `mc3f_world`지만, 실제 형상은 기존 사각 방이다.
- `/scan`, `/odom`, `/clock`, TF, `/plan`, `/global_costmap/costmap`, `/cmd_vel`, 가상 카메라를 기록했다.
- 최초 기준 시험 Domain 84, 최초 상자 시험 Domain 85. 모델 접촉 보정 후 채택한 기본 주행 Domain 86, 상자 추가 시험 Domain 87. 모두 localhost 제한 및 별도 Ignition 파티션을 사용했다. 실제 로봇 드라이버를 실행하지 않았다.
- 원본 ROS 패키지·소스·설정 및 외부 시뮬레이터 모델 파일은 수정하지 않았다. 실행 시 시험 사본을 전달했다. 따라서 패키지 재빌드 대상 변경은 없다. 입력 사본·실행 보조 코드·해시는 `output/simulation/2026-10-24/inputs` 및 `input_hashes.json`에 보관했다. 이 자료의 일부 경로는 이번 실행 환경 재현용이며 다른 PC의 표준 경로로 가정하지 않는다.

## 발견한 문제와 조치

처음에는 `/odom`의 이동량과 Nav2 상태 4가 나타났지만 카메라상 차체는 출발 위치에 남아 있었다. 이 결과는 이동 검증 자료에서 제외했다. 제외한 결과는 `baseline_initial_model_stationary`와 `obstacle_initial_model_stationary`에 보존했다.

현재 모델에서 차체 collision의 바닥은 차축 기준 -0.050m, 바퀴 반경은 0.045m여서 차체가 바퀴보다 먼저 바닥에 닿을 수 있었다. 원본을 보존한 모델 사본에서 차체 collision 및 대응 visual 중심의 z를 0.020m로 올렸다. 다른 센서·구동 파라미터는 이 문제 때문에 변경하지 않았다. 보정 뒤 카메라에서 차체의 실제 이동을 확인했다. 기존 검증 문서의 모든 과거 실행에 같은 문제가 있었다고 소급하여 단정하지 않는다.

첫 기준 시험의 Nav2 컨테이너가 종료 시 남아 다음 시험의 준비 과정과 충돌했다. 해당 실행 로그로 소유 PID를 확인한 뒤 그 프로세스만 종료했다. 이후 실행에서는 별도 Domain과 실행 자식 PID 추적 정리를 사용했다. 임의의 ROS 프로세스를 일괄 종료하지 않았다.

## 채택한 기본 주행

- 초기 map pose (-1.5, 0, 0), 목표 (1.5, 0, 0).
- Nav2 액션의 실제 반환 상태: 4 (`SUCCEEDED`).
- Gazebo 카메라에서 차체가 출발 지점에서 녹색 목표 표시까지 이동하는 것을 확인.
- `/odom` 종료 값: x=2.9975m, y=-0.0350m. 이는 odom 기준 이동 정보이며 독립적인 세계 좌표 오차 검증값이 아니다.
- 원본 카메라 프레임 130장, 영상에 대응하는 시뮬레이션 시각 6.9~26.7초.
- `baseline/summary.json`, `baseline/telemetry.jsonl`, 원본 프레임·실행 로그를 근거로 사용.
- 별도의 실제 위치 오차 측정, 반복 성공률 또는 이동 장애물 대응 검증은 수행하지 않았다.

## 상자 추가 시험

저장 지도는 같게 두고 시뮬레이터에만 0.55×0.55×0.6m의 고정 상자를 추가했다. 물체가 옮겨진 상황을 시험한 것으로, 움직이는 사람이나 동적 물체 추적 시험이 아니다.

카메라와 Costmap에서 상자를 확인했고 `/plan`이 우회 방향으로 바뀌는 반응도 나타났다. 하지만 채택 모델의 시험은 50초 시뮬레이션 제한 안에 완주하지 못해 취소를 요청했다. 최종 액션 종료 결과는 별도로 수집하지 않았으므로 이를 `ABORTED` 상태라고 기록하지 않는다. `obstacle/summary.json`의 action_status는 null이다.

이 시험은 회피 성공 영상으로 발표에 넣지 않았다. 기본 주행 완료와 장애물 회피 완료를 구분한다. 다음에는 센서·TF·Costmap 정합, 경로의 회전 가능성, RPP 충돌 검사 및 Ackermann 모델에서 제자리 회전을 요구하는 복구 동작을 각각 확인해야 한다. 원인 확정 없이 전역 플래너를 교체하지 않았다.

## 영상 편집과 발표

`Nav2_시뮬레이션_목표점주행.mp4` 및 Impress용 WebM은 1920×1080, 20fps, 19.8초다. 왼쪽은 실제 Gazebo 가상 카메라를 지도와 같은 방향으로 회전·잘라 표시했다. 오른쪽은 기록된 Costmap·경로·TF의 시각화다. RViz 화면 녹화로 설명하지 않는다. 움직임을 임의로 생성하지 않았다.

시뮬레이션 타임스탬프 기준 1배속으로 구성했고 다음 실제 관측이 올 때까지 영상 프레임을 유지했다. 실제 장비의 처리 속도를 입증하는 영상이 아니다. 상자 추가 시험의 실패 장면을 잘라 성공으로 제시한 것이 아니라, 다른 독립 실행인 기본 주행만 채택했다.

발표 자료는 기존 LIMO 영상 다음인 새 9장에 추가했고, 전체 16장과 마지막 목표 아키텍처 구성을 유지했다. 기존 영상은 보존했다. Windows용 PPTX에는 MP4, Impress용 ODP에는 WebM을 내장했다.

## 지도와 경로에 대한 설명 근거

저장 지도는 벽·복도 구조와 위치추정의 기준을 제공한다. Nav2 경로는 목표를 지정할 때 현재 위치에서 계산하며 관측 변화에 따라 재계획할 수 있다. 전역 Costmap도 static layer 외에 obstacle layer를 사용하므로 전역 계획이 반드시 정적 지도만 참조하는 것은 아니다. 현재 시험 설정에도 두 계층이 포함돼 있다. 지도에 없는 물체를 센서로 반영하는 것과 그 물체를 안정적으로 피하며 주행을 완료하는 것은 별도 검증 항목이다.

공식 근거:
- https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/sensors/mapping_localization/
- https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/
- https://docs.nav2.org/jazzy/getting_started/navigation_concepts/navigation_servers/

---

## 확장된 복잡 실내 환경(`16.0m × 12.0m`) 및 다수 고정·이동 장애물 실시간 GUI 시연 기록 (Stages A ~ F)

### 1. 개요 및 실행 방식
- 기존 소형 사각 방을 넘어 교차로, 순환 루프, 막다른 길(Dead-end), 통과 폭이 다른 3개 동서 통로(`2.10m`, `1.40m`, `2.20m`), 동측 광장, 지도 반영 고정 장애물 4개가 포함된 **`16.0m × 12.0m` 복잡 실내 환경**을 구성하고, 사용자의 데스크톱(`DISPLAY=:0`)에 **Gazebo GUI(상단 고정 시점)**와 **RViz2(TopDownOrtho 시점 + 실시간 S/G 마커, 파란 점선 최초 경로, 주황색 현재 경로, 초록색 실제 이동 궤적, 빨간색/마젠타 장애물 마커)** 창을 동시에 띄워 실시간으로 실행·검증했다.
- 실제 LIMO 하드웨어 및 USB의 `capstone_ws`, 그리고 발표 자료(`*.pptx`, `*.odp`, `*.pdf`)는 일체 수정하지 않았다.
- 다른 ROS·Gazebo 프로세스를 강제 종료하는 광범위한 `pkill`을 사용하지 않고, 실행 세션이 직접 생성한 프로세스 그룹(`active_session_pids.json`)만 안전하게 정리했다.

### 2. 실시간 GUI 단계별 조작 명령어 (`sim_ctl.py`)

언제든 데스크톱 화면에서 원하는 단계(Stage A ~ F)의 Gazebo GUI + RViz2 창을 띄우고 대기 상태에서 주행 시작·일시정지·재개·초기화·종료를 직접 제어할 수 있다:

```bash
# 1) 원하는 단계(A, B, C, D, E, F)의 Gazebo GUI + RViz2 창을 띄우고 출발점 S에서 대기 상태로 준비
python3 output/simulation/sim_ctl.py select B

# 2) 준비 상태 및 로봇 위치 확인
python3 output/simulation/sim_ctl.py status

# 3) 목적지 G(+5.80, +1.40)로 자율주행 시작 (동시에 실시간 GUI 녹화 및 원본 로그 자동 기록)
python3 output/simulation/sim_ctl.py start

# 4) 주행 중 일시정지 / 재개 / 출발점 S로 리셋 / 안전 종료
python3 output/simulation/sim_ctl.py pause
python3 output/simulation/sim_ctl.py resume
python3 output/simulation/sim_ctl.py reset
python3 output/simulation/sim_ctl.py stop
```

### 3. 단계별(Stages A ~ F) 실행 요약 및 영상 파일 경로

| 단계 | 시나리오 구성 | 실행 결과 및 핵심 지표 | 실시간 GUI 데스크톱 녹화본 (`output/video/`) | 로그 기반 좌우 비교 영상 (`1.0x` / `2.0x`) |
|:---:|:---|:---|:---|:---|
| **Stage A** | **사전 지도 기반 기본 복잡 주행**<br>(고정 장애물 4개 지도 반영) | `STATUS_SUCCEEDED` (`52.94s`)<br>GT 오차 `0.223 m`, 최소 간격 `25.88 cm` | `live_gui_stageA_recording.mp4` (`output/video/live_gui_stageA_recording.mp4`, 로컬 보관) | `stageA_log_reconstructed_1.0x.mp4` (`output/video/stageA_log_reconstructed_1.0x.mp4`, 로컬 보관)<br>`stageA_log_reconstructed_2.0x.mp4` (`output/video/stageA_log_reconstructed_2.0x.mp4`, 로컬 보관) |
| **Stage B** | **미등록 고정 장애물 3개 순차 발견 및 3단계 경로 재계획**<br>(남측 통로 차단 → 중앙 복도 차단 → 북측 통로 부분 장애물 회피) | `STATUS_SUCCEEDED` (`72.97s`)<br>GT 오차 `0.203 m`, 최소 간격 `15.99 cm` (추가 장애물 `25.79 cm`) | `live_gui_stageB_recording.mp4` (`output/video/live_gui_stageB_recording.mp4`, 로컬 보관) | `stageB_log_reconstructed_1.0x.mp4` (`output/video/stageB_log_reconstructed_1.0x.mp4`, 로컬 보관)<br>`stageB_log_reconstructed_2.0x.mp4` (`output/video/stageB_log_reconstructed_2.0x.mp4`, 로컬 보관) |
| **Stage C** | **단일 이동 장애물 완전 횡단 후 주행 재개**<br>(`x=-3.0m` 통로 완전 횡단 후 이탈) | `STATUS_SUCCEEDED` (`54.63s`, **`SUCCESS_STRICT`**)<br>GT 오차 **`0.140 m`**, 최소 간격 `19.33 cm` | `live_gui_stageC_recording.mp4` (`output/video/live_gui_stageC_recording.mp4`, 로컬 보관) | `stageC_log_reconstructed_1.0x.mp4` (`output/video/stageC_log_reconstructed_1.0x.mp4`, 로컬 보관)<br>`stageC_log_reconstructed_2.0x.mp4` (`output/video/stageC_log_reconstructed_2.0x.mp4`, 로컬 보관) |
| **Stage D** | **다수(3개) 이동 장애물 순차 횡단 대응**<br>(서측 교차로·중앙 출구·동측 광장 3개 이동 장애물) | `STATUS_SUCCEEDED` (`59.03s`)<br>GT 오차 `0.212 m`, 최소 간격 `12.82 cm` (감속·대기 후 `67.90s` 주행 재개) | `live_gui_stageD_recording.mp4` (`output/video/live_gui_stageD_recording.mp4`, 로컬 보관) | `stageD_log_reconstructed_1.0x.mp4` (`output/video/stageD_log_reconstructed_1.0x.mp4`, 로컬 보관)<br>`stageD_log_reconstructed_2.0x.mp4` (`output/video/stageD_log_reconstructed_2.0x.mp4`, 로컬 보관) |
| **Stage E** | **미등록 고정 장애물 우회 + 이동 장애물 2개 복합 환경**<br>(남측 통로 차단 우회 + 우회로 출구·목적지 전면 이동 장애물 횡단) | `STATUS_SUCCEEDED` (`59.79s`)<br>GT 오차 `0.200 m`, 최소 간격 `13.51 cm` (이동 장애물 `27.15 cm`) | `live_gui_stageE_recording.mp4` (`output/video/live_gui_stageE_recording.mp4`, 로컬 보관) | `stageE_log_reconstructed_1.0x.mp4` (`output/video/stageE_log_reconstructed_1.0x.mp4`, 로컬 보관)<br>`stageE_log_reconstructed_2.0x.mp4` (`output/video/stageE_log_reconstructed_2.0x.mp4`, 로컬 보관) |
| **Stage F** | **장애물 통로 영구 잔류 및 완전 차단 안전 정지**<br>(모든 통로 차단 상태에서 안전 정지·대기 유지) | **`SAFE_STOP_PASSED`** (`40.02s`)<br>차단 장애물 최소 간격 **`11.18 cm`** (`>= 10cm`), **`22.3초` 연속 정지** | `live_gui_stageF_recording.mp4` (`output/video/live_gui_stageF_recording.mp4`, 로컬 보관) | `stageF_log_reconstructed_1.0x.mp4` (`output/video/stageF_log_reconstructed_1.0x.mp4`, 로컬 보관)<br>`stageF_log_reconstructed_2.0x.mp4` (`output/video/stageF_log_reconstructed_2.0x.mp4`, 로컬 보관) |
