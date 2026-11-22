# 통합 자율주행 시뮬레이션 (Open-RMF × Gazebo × Nav2 × YOLO)

**시뮬레이션 전용**입니다. 실제 LIMO·모터·Tailscale·운영 8080 서버를 사용하지 않고, Safety Gate(`stall_detector_node`)를 수정·우회하지 않으며 `allow_motor_drive`를 켜지 않습니다. 수치와 PASS/FAIL은 [RESULTS.md](RESULTS.md)에 있고, 거기에 적지 않은 것은 **미검증**입니다.

## 구조

```
명령 경로:  Open-RMF → Fleet Adapter → Mission Manager → Robot Agent → Nav2(+AMCL) → /cmd_vel_raw → Safety Gate → /cmd_vel → Gazebo LIMO
영상/AI:    Gazebo RGB 카메라(움직이는 로봇) → JPEG → sender → [chaos proxy] → AI 호스트(CUDA YOLO11n) → 결과 → 로봇 쪽 consumer
화면:       Mission Manager 대시보드(지도·위치·임무·영상 패널·검출) http://127.0.0.1:<포트>/?video=http://127.0.0.1:<AI HTTP 포트>
```

- 명령 경로와 영상 경로는 **별도 TCP 연결**입니다. YOLO 결과는 Nav2/Safety Gate에 연결되어 있지 않습니다(consumer는 읽기 전용, 상태만 발행).
- 모의 직선 이동 노드(`mock_robot_node`)는 쓰지 않습니다. 실제 Gazebo 물리, Ignition Fortress, 저장 지도 + AMCL, LiDAR·Depth·RGB, 기존 Safety Gate 경로입니다.
- 원본 Gazebo 모델·월드·SLAM·Nav2·Safety Gate 소스는 수정하지 않았습니다. `models/limo_eval_diff_rgb/`는 `limo_eval_diff`의 **복사본 + RGB 카메라**(깊이 카메라와 같은 위치, 640×480, 15 Hz)이고, `integrated_sim.launch.py`가 기존 launch를 불러와 (1) SLAM 제거·저장 지도 AMCL, (2) 스폰 모델 교체, (3) RGB 이미지 브리지 추가만 합니다(`fleet_control/gazebo/localized_sim.launch.py`에서 파생).
- 시나리오 월드는 기존 lounge 월드 그대로이고, 장애물·사람·의자는 **실행 중에** `ign service`로 생성합니다(`tools/scenario_ctl.py`). 사람/의자 메시는 저장소에 넣지 않으며 YOLO 실험 폴더의 것을 가리킵니다.

## 파일

| 파일 | 역할 |
|---|---|
| `run_integrated.sh` / `run_integrated.py` | 통합 실행기(`up` / `down` / `status` / `all`) |
| `integrated_env.py` | 포트·ROS 도메인 충돌 검사, 프로세스 등록·종료(등록된 프로세스 그룹만), 환경 |
| `integrated_sim.launch.py` | Gazebo + AMCL + Nav2 + Safety Gate + RGB 브리지 |
| `fleet_integrated.json` | RMF 그래프(남쪽 홀의 열린 공간, wp_a~wp_d), `fleet_gazebo.json`과 같은 RMF↔Nav2 변환 |
| `tools/scenario_ctl.py` | 실행 중 장애물 박스·걷는 사람·의자 생성/제거 |
| `tools/sim_metrics.py` | 로봇 쪽 측정(GT 거리·충돌·최소 간격·costmap 증거·계획 경로·Gate 상태) |
| `models/` | RGB 카메라 모델 복사본, 시나리오 SDF |

## 실행

필요 조건: ROS 2 Humble, 빌드된 참조 워크스페이스(읽기 전용, `CAPSTONE_REFERENCE_WS`), 격리된 Open-RMF 2.1.8(`~/rmf_isolated/env.sh`), 영상/AI 호스트 코드와 YOLO 환경(저장소 밖, AGPL이므로 커밋·배포하지 않음: `VIDEO_AI_HOME`, `YOLO_HOME`), NVIDIA GPU.

```bash
export CAPSTONE_REFERENCE_WS="<READ_ONLY_WS>"
cd fleet_control/integrated_sim
./run_integrated.sh up            # 단계별: 전체 기동 후 대기, 대시보드 주소 출력
./run_integrated.sh down          # 이 실행이 등록한 프로세스만 종료 + 잔여 검사
./run_integrated.sh all           # 한 번에: 기동 → 시나리오 → 종료 → 요약 (결과: <run_dir>/results)
./run_integrated.sh all --scenarios baseline,obstacle --repeats 1
```

환경 변수(기본값): `INT_SERVER_PORT=18200`, `INT_WS_PORT=18201`, `INT_AI_LINK_PORT=18210`, `INT_AI_HTTP_PORT=18211`, `INT_PROXY_PORT=18212`, `INT_ROBOT_DOMAIN=181`, `INT_RMF_DOMAIN=182`, `INT_RUN_ROOT`(실행 기록 위치, 기본 `~/.local/share/robot-sim-projects/limo_integrated_sim/runs`), `INT_GPU_RENDER=1`(NVIDIA PRIME 렌더링).
시작 전 검사: 8080 금지, 포트 사용 중 여부, ROS 도메인에 기존 노드가 있는지(있으면 중단). 종료 후: 등록 프로세스·포트 잔여 검사.

## 대시보드

`up` 후 출력되는 주소(`/?video=...`)를 이 노트북의 브라우저로 엽니다(127.0.0.1 전용). 로그인에는 `<run_dir>/secrets/viewer.token`(보기) 또는 `operator.token`(임무 생성·취소)을 **본인 터미널에서** 열어 복사하고, 영상 패널에는 `video_viewer.token`을 넣습니다. 토큰 값은 출력·문서·Git에 넣지 않습니다. 화면 사용법은 [DASHBOARD_GUIDE.md](../docs/DASHBOARD_GUIDE.md). 임무는 대시보드에서 직접 만들 수도 있지만, RMF가 제어권 lease를 가지면 웹 생성은 409입니다.

## RMF Web 검토

[RMF_WEB_REVIEW.md](RMF_WEB_REVIEW.md) 참고(설치하지 않음, 가능성·한계만 기록).
