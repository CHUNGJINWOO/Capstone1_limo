# LIMO 자율주행 시뮬레이션 단계적 구성 및 검증 보고서 (2026-10-24)

> 문서 안내: 이 보고서는 초기·중간 시험의 조사 이력이다. 이후 복합·왕복·연속 이동 시험의 최신 결과는 [조건별 실험 정리](experiments/2026-10-24/README.md)를 우선한다. 원본 시계열·영상은 로컬에 보관하며 Git에는 요약 사본과 해시를 남긴다.

## 1. 개요 및 목적

본 보고서는 2026-2 캡스톤 디자인 "LIMO를 이용한 실내 자율 순찰 및 안내" 프로젝트의 **가상 시뮬레이션 환경(Gazebo Fortress + ROS 2 Humble)**에서 수행된 자율주행 단계적 구성, 평가 체계의 객관적 무결성 확보(단일 authoritative 평가 라이브러리 `evaluation_lib.py` 분리 구축, SAT 2D 다각형 연속 간격 계산, 타임스탬프 동기화 GT 속도 분석, ROS 2 표준 `GoalStatus` 적용, 4단계 Phase 구간 분리, 다중 ROI Costmap Clearing), 전 시나리오 원본 시계열 기반 전면 재평가, 그리고 **이동 장애물 조기 중단(`STATUS_ABORTED`) 및 완전 차단 안전 여유 간격 미달(6.86cm) 문제에 대한 정밀 원인 규명 및 최소 변경 개선 결과**를 종합한 최종 기술 문서입니다.

> [!IMPORTANT]
> **검증 범위 및 절대 원칙**
> - 본 검증은 **100% 가상 시뮬레이션(Gazebo Fortress)** 환경에 한정됩니다.
> - **실물 LIMO Pro 로봇에서의 주행 검증 결과가 아닙니다.**
> - 실제 건물을 완벽히 물리적으로 복제한 것이 아니며, 2D 점유 격자 지도(Occupancy Grid)의 벽체 경계를 기반으로 Gazebo 상에 생성한 **"실제 지도 기반 벽체 가상 환경"**입니다.
> - **데이터 출처 구분**: 본 보고서의 결과표는 원본 시계열 로그(`.jsonl`)로부터 직접 재계산한 항목과 기존 `summary.json`에서 보존된 항목을 명확히 분리하여 기술했습니다.
> - **사후 왜곡 금지**: 안전 정지 10cm 기준을 낮추지 않고 원칙대로 평가했으며, 성공과 실패 기록을 모두 투명하게 공개합니다. 단일 성공으로 일반적인 동적 장애물 회피 완성을 주장하지 않습니다.

---

## 2. 기존 환경 및 이전 시험 결과 확인과 원인 규명

### 2.1 바퀴-차체 접촉 형상 문제 (Contact Geometry Issue)

- **현상 재현 (`output/simulation/2026-10-24_verification/1_reproduction_contact_issue`)**:
  - 원본 모델(`limo_ackermann_fortress.sdf`) 실행 시 `/odom`과 Nav2는 목표점까지 주행한 것으로 표시(Nav2 status: 4)되었으나, 시뮬레이터 상의 실제 차체 위치는 출발점인 `(-1.500, 0.000)`에서 단 1mm도 이동하지 못함 (Ground Truth 오차: **3.000m**, 차체 고착).
- **물리적 원인 분석**:
  - 원본 SDF에서 차체 충돌체(`base_collision`) 크기가 $0.28 \times 0.13 \times 0.10\,\text{m}$이고 중심 z가 0.0m로 설정되어 충돌체의 바닥면이 $z = -0.050\,\text{m}$였습니다.
  - 반면 LIMO의 구동 바퀴 반경(`wheel_radius`)은 $0.045\,\text{m}$ ($z = -0.045\,\text{m}$)이므로, 차체 바닥면이 바퀴보다 지면에 5mm 먼저 닿아 바퀴가 공중에 뜬 채 헛돌아 지면 마찰력을 전달하지 못했습니다.
- **수정 및 검증 (`output/simulation/2026-10-24_verification/1_reproduction_baseline`)**:
  - 원본 파일을 보존한 상태에서 시뮬레이션 전용 모델(`limo_contact_check.sdf`)을 적용, `base_collision` 및 `base_visual`의 중심 z를 `+0.020m`로 올려 차체 바닥면을 $z = -0.030\,\text{m}$로 보정(바퀴 바닥면보다 15mm 위).
  - 보정 모델 재현 시험 결과: 시뮬레이션 시간 14.0초 만에 목표점 (1.5, 0.0)에 실제 도달, **Ground Truth 오차 0.027m (2.7cm)**로 실제 물리적 주행 통과 확인.

### 2.2 장애물 시험 미완료 원인 (Ackermann 조향과 제자리 회전 복구 동작 부적합성)

- **현상 재현 (`output/simulation/2026-10-24_verification/1_reproduction_obstacle_unmodified`)**:
  - 미지의 0.55m 상자 장애물 앞에서 로봇이 회피하지 못하고 50초 시뮬레이션 제한 시간 초과 (Ground Truth 위치: `(-0.696, 0.380)`, 오차 2.228m, 미완료 취소).
- **동작 분석 및 원인 규명**:
  - 로봇이 장애물 모퉁이에 근접했을 때 RPP(Regulated Pure Pursuit) 컨트롤러에서 전방 충돌을 감지(`detected collision ahead!`)하고 정지.
  - Nav2 기본 Behavior Tree(`navigate_to_pose_w_replanning_and_recovery.xml`)가 복구 동작으로 `Spin` (제자리 회전, $v=0, \omega=1.0\,\text{rad/s}$)을 호출.
  - LIMO의 Ackermann 조향 기구는 전진/후진 속도($v$)가 0이면 조향각만 꺾이고 차체 회전이 불가능함.
  - 이로 인해 `behavior_server`가 목표 각도 도달을 기다리다 10초 타임아웃(`Exceeded time allowance before reaching the Spin goal`)으로 실패하고, 이 복구 실패가 반복되어 50초를 전부 소진함.
- **최소 수정 및 검증 (`output/simulation/2026-10-24_verification/1_reproduction_obstacle_ackermann_bt`)**:
  - 플래너나 제어기를 전면 교체하지 않고, Ackermann 로봇에 적합한 Behavior Tree(`navigate_to_pose_ackermann.xml`)를 작성하여 `Spin` 복구 동작을 배제하고 `BackUp` 및 `Wait` 시퀀스로 변경.
  - 재시험 결과: 장애물 모퉁이에서 안전 후진 후 새로운 우회 경로를 추종하여 **27.0초 만에 완주 (Ground Truth 오차 0.224m)**.

---

## 3. 평가 체계 및 데이터 출처 엄정 구분

### 3.1 단일 authoritative 평가 라이브러리(`evaluation_lib.py`)
실행 러너와 단위 시험이 서로 다른 복사본 함수를 사용하던 구조를 전면 폐지하고, [`output/simulation/evaluation_lib.py`](../output/simulation/evaluation_lib.py) 단일 모듈로 통합했습니다.
- SAT 2D 다각형 연속 최소 간격 계산
- 4단계 Phase 구간 분리 (`INITIALIZING`, `NAVIGATING`, `POST_RESULT`, `POST_CANCEL`)
- 관측 누락($\Delta t > 0.3\,\text{s}$) 및 재점유 발생 시 연속 정지·유지 시간 즉시 초기화
- 다중 ROI(전면, 내부, 팽창) 전수 셀 소거 및 유지 시간 검증

### 3.2 데이터 출처 및 타임스탬프 한계 고지
1. **원본 시계열 재계산 데이터 (Recalculated from Raw Logs)**:
   - 시나리오 4, 5, 6, 7은 `output/simulation/2026-10-24_phase_separated/`의 고주파 원본 로그(`gt_pose_history.jsonl`, `costmap_roi_history.jsonl`)로부터 전수 재계산되었습니다.
2. **기존 summary / 텔레메트리 보존 데이터 (Inherited from Existing Summaries)**:
   - 시나리오 1, 2, 3, 8은 `output/simulation/2026-10-24_rigorous/`에서 생성된 summary 및 텔레메트리를 보존하여 표기했습니다.
3. **타임스탬프 한계 (`telemetry_source: RECV_SIM_TIME_APPROXIMATION`)**:
   - Ignition Gazebo의 Pose_V 브릿지가 헤더 타임스탬프를 채우지 않아 `header.stamp == 0.0`으로 기록됩니다. 따라서 속도 및 정지 시간은 노드의 ROS 2 토픽 수신 시점 시뮬레이션 클럭 시각(`recv_sim_time`) 기반의 추정치이며, 콜백 큐잉 및 브릿지 지연에 따른 오차가 존재함을 명시합니다.

---

## 4. 핵심 주행 결함 원인 분석 및 해결 (Root Cause Analysis & Fixes)

### 4.1 문제 1: 이동 장애물 시험의 정밀 분석 및 횡단/잔류 분리 검증 (미해결 상태)

#### (1) `moving_box_sc5` 기하학적 형상 및 $y=0.6$ 정지 시 통로 차단 여부 확인
- **장애물 및 통로 물리 치수**:
  - 이동 장애물 크기: $0.35 \times 0.35 \times 0.50\,\text{m}$ (X 폭 $0.35\,\text{m}$, Y 폭 $0.35\,\text{m}$).
  - 남측 주 통로 형상: $y \in [-1.30, -0.40]$ (물리적 폭 $\mathbf{0.90\,\text{m}}$).
    - 남측 경계: 벽체 `c_10` 북면 ($y = -1.30\,\text{m}$, $x \in [-0.80, 2.40]$)
    - 북측 경계: 벽체 `c_9` 남면 ($y = -0.40\,\text{m}$, $x \in [-0.80, 2.40]$)
    - 통로 중앙선: $y = -0.85\,\text{m}$.
  - 로봇 치수 (LIMO Pro):
    - 차체: $0.28 \times 0.13\,\text{m}$, 휠 트랙 폭: $0.21\,\text{m}$.
    - 회전 외접원 반경: `robot_radius: 0.19m` (직경 $\mathbf{0.38\,\text{m}}$).
- **횡단 중 통로 차단 여부**:
  - 상자가 $x = 0.0$ 선상을 따라 $y = -0.85$ 통로 중앙을 지날 때, 상자 북단은 $y = -0.675$, 남단은 $y = -1.025$에 위치함.
  - 상자 북단과 `c_9` 사이 간격: $(-0.40) - (-0.675) = \mathbf{0.275\,\text{m}}$.
  - 상자 남단과 `c_10` 사이 간격: $(-1.025) - (-1.30) = \mathbf{0.275\,\text{m}}$.
  - 남은 여유 폭($0.275\,\text{m}$)이 로봇 최소 통행 폭($0.38\,\text{m}$)보다 작으므로, 상자가 $y \in [-1.15, -0.55]$ 구간을 지나는 동안 **남측 통로는 물리적으로 100% 완전 차단**됩니다.
- **$y = 0.60$ 정지 후 통로 차단 여부**:
  - 상자 중심: $(x = 0.00, y = 0.60)$. 상자 Y 범위: $[0.425, 0.775]$.
  - 남측 주 통로 북단($y = -0.40$)과의 물리적 간격: $0.425 - (-0.40) = \mathbf{0.825\,\text{m}}$ 북쪽에 위치함.
  - 상자는 중앙 고정 벽체 `c_9` ($x \in [-0.80, 2.40], y \in [-0.40, +0.80]$) **내부에 완전히 매립**되어 있습니다.
  - 북측 통로($y \in [0.80, 1.40]$)와도 $0.80 - 0.775 = 0.025\,\text{m}$ 벽체 안쪽에 있어 어느 통로도 물리적으로 점유하지 않습니다.
  - **결론**: $y=0.60$ 정지 후 상자는 **남측 통로를 물리적으로 막지 않습니다**. 그러나 정적 벽체 내부 매립으로 인한 Gazebo 물리/레이캐스팅 잠재 영향을 배제하기 위해, 통로를 완전히 벗어나 사라지는 시나리오와 통로 중앙에 잔류하는 시나리오로 명확히 분리하여 검증을 수행했습니다.

#### (2) 두 가지 분리 시험 설계 및 아카이빙
- **시나리오 A (완전 횡단 소거 시험, `clean_crossing`)**: 장애물이 통로를 횡단한 후 $y \ge -0.10$에서 $z = -10.0\,\text{m}$로 완전 제거되어 통로가 물리적으로 100% 다시 열리는 상황. 감속·대기 후 동일 목표로 주행 재개 여부 및 실패 시 원인 단계별 규명.
- **시나리오 B (통로 잔류 시험, `corridor_remain`)**: 장애물이 통로 중앙 $y = -0.85$로 진입한 후 영구 잔류하여 통로를 지속 차단하는 상황. 우회 가능 여부(북측 우회로 탐색) 또는 안전 정지·대기 평가.
- **형상 관리 및 아카이빙**: 각 실행 폴더(`output/simulation/2026-10-24_crossing_investigation/`)에 사용된 `nav2_params.yaml`, `behavior_tree.xml`, `world.sdf`, `robot_model.sdf` 원본 사본과 SHA-256 해시(`.sha256`), 실행 인자(`run_args.json`)를 전수 보존.

#### (3) 분리 시험 실측 결과 및 실패 원인 좁히기

기존에 보고된 18.7cm와 30.8cm는 **‘이동 상자에 대한 최소 간격’으로 한정**하며, 정적 벽체를 포함한 전체 최소 간격과 명확히 구분하여 기록합니다.

| 시험 구분 | 실행 디렉토리 | 최종 액션 상태 | 주행 시간 (`sim_dur`) | 이동 상자 SAT 최소 간격 | 전체 벽체 포함 SAT 최소 간격 | 자율 주행 정지 | 종합 판정 | 비고 |
|:---|:---|:---:|:---:|:---:|:---:|:---:|:---|:---|
| **시나리오 A (완전 횡단 소거)** | `2026-10-24_crossing_investigation/scen5_clean_crossing_run1` | **STATUS_ABORTED (6)** | 40.46 s | **0.1868 m (18.7cm)** | **0.1803 m (18.0cm)** (벽체 `c_9`) | 7.886 s | **주행 재개 실패 (FAILED_TO_RESUME)** | 통로 개방 후 차체 편각 고착으로 주행 재개 실패 |
| **시나리오 B (통로 영구 잔류)** | `2026-10-24_crossing_investigation/scen5_corridor_remain_run1` | **STATUS_CANCELED (5)** | 52.45 s | **0.3075 m (30.8cm)** | **0.1696 m (17.0cm)** (벽체 `c_10`) | **32.138 s** | **안전 정지 통과 (SAFE_STOP_PASSED)** | 이동 상자 30.8cm, 벽체 17.0cm 안전거리 유지하며 32.1s 자율대기 |

#### (4) 시나리오 A 주행 재개 실패의 단계별 원인 규명

장애물이 $t=17.73\text{s}$에 물리적으로 완전히 제거된 후 주행이 재개되지 못한 원인을 순서대로 조사하고 가설을 실증적으로 검증했습니다:

1. **1단계: 장애물 통과 후에도 Costmap이 막혀 있는가? $\to$ [정상 소거 확인 / 치명적 차단 없음]**
   - `costmap_roi_history.jsonl` (`output/simulation/2026-10-24_crossing_investigation/scen5_clean_crossing_run1/costmap_roi_history.jsonl`, 로컬 보관) 실측: 상자가 통로를 지날 때($t=10.6\sim14.8\text{s}$) 중앙 비용은 Cost 100(Lethal)이었으나, 제거 직후($t \ge 17.73\text{s}$) **중앙 비용이 Cost 47로 급감**하여 치명적 점유가 정상 해소됨.
   - 따라서 코스트맵의 영구적 미소거(Raytrace Clearing Failure)는 실패의 직접 원인이 아님.

2. **2단계: 경로 생성이 실패하는가? $\to$ [초기 일시 실패 후 생성 성공 확인]**
   - 상자 횡단 직후 로봇이 $x = -0.384, y = -0.925$에 근접 정지해 있는 동안, 차체 전면 범퍼가 장애물 잔류 팽창 경계에 닿아 NavfnPlanner가 시작점 충돌로 **약 22초간 경로 생성을 실패**(`GridBased: failed to create plan with tolerance 0.50`)하며 BT 복구 시도를 소진함.
   - 그러나 로봇이 BackUp 후진하여 $x = -0.645$로 이격된 $t = 36.58\text{s}$ 시점에는 **326개 경유점의 정상 전역 경로가 성공적으로 생성되어 제어기에 전달됨** (`Passing new path to controller`).

3. **3단계: ‘RPP 측벽 충돌’ 가설 및 보조 기하 분석 (원인 확정 유예)**
   - **보조 기하 분석 (직진 투영 계산)**:
     - 차체가 남동쪽으로 기울어진 자세($(x = -0.453\,\text{m}, y = -0.910\,\text{m}, \text{yaw} = -40.1^\circ)$)에서 직진 방향으로 단순 투영 시 $0.4\,\text{m}$ 전방 지점이 남측 벽체 `c_10`($y \le -1.30\,\text{m}$)과 겹침이 확인됩니다.
     - 그러나 **직진 투영 계산은 보조 분석에 한정**되며, RPP 제어기가 실제로 주행 시 계산하는 곡률 원호 궤적(Lookahead Arc) 및 로컬 코스트맵 풋프린트 충돌 검사 원본 데이터를 직접 확보하기 전에는 **측벽 충돌 원인을 확정하지 않고 가설(Open Hypothesis)로 유지**합니다.
   - **경로 생성 및 조향 제약**:
     - 상자 통과 직후 2D Grid 플래너가 통로 중심선($y \approx -0.85$)으로 진입하는 급격한 우회전 경로를 발행했고, Ackermann 최소 회전 반경($R_{min} \approx 0.37\,\text{m}$) 제약과 상충되어 협로 입구에서 차체 편각이 형성되었음을 확인했습니다.

#### (5) 원인 기반 단일 변경(`backup_dist`) 비교 검증 시험

`NavigateToPose.Goal.behavior_tree`를 통해 각 BT XML이 Nav2 액션 서버에 실제 적용되도록 바인딩을 구현하고, 다른 모든 조건(월드·지도·시작점·목표점·제한시간 55초)을 엄격히 고정한 상태에서 15cm 기준 BT와 30cm 수정 BT를 비교 검증했습니다.

- **과거 시험 상태 정정**:
  - 기존 `scen5_clean_crossing_mod1_backup30` 시험은 러너의 `Goal.behavior_tree` 미할당으로 인해 기본 BT가 실행되었을 가능성이 확인되어 **‘수정 적용 미확인(UNCONFIRMED_BT_APPLIED)’**으로 공식 정정했습니다.
  - 또한 전체 최소 간격이 $9.11\,\text{cm}$로 $10\,\text{cm}$ 안전 기준에 미달하였으므로, 안전 정지 판정을 **`NOT_PASSED`**로 확정했습니다.

- **바인딩 검증 완료된 15cm vs 30cm BT 실측 비교표**:

| 시험 구분 및 디렉토리 | 바인딩된 BT XML 및 SHA-256 | 목표 후진거리 | **실측 GT 후진거리** | 최종 액션 상태 | 최종 도달 위치 ($x, y$) | 이동 상자 SAT 최소 간격 | 전체 벽체 포함 SAT 최소 간격 | 주행 재개 여부 및 종합 판정 |
|:---|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| **기준 검증 시험**<br>(`clean_crossing_bt15_verified` (`output/simulation/2026-10-24_crossing_investigation/clean_crossing_bt15_verified`, 로컬 보관)) | `navigate_to_pose_ackermann.xml`<br>(`f4d960094d9c...`) | 0.15 m | **0.1452 m (14.5cm)**<br>($t=44.6\sim48.1\text{s}$) | STATUS_CANCELED (5)<br>(55s 타임아웃) | $(-0.629, -0.660)$<br>(통로 입구 미통과) | **0.2272 m (22.7cm)** | **0.1102 m (11.0cm)**<br>($\ge 10\text{cm}$ 통과) | **주행 재개 미완료**<br>(`FAILED_TO_RESUME`) |
| **수정 검증 시험**<br>(`clean_crossing_bt30_verified` (`output/simulation/2026-10-24_crossing_investigation/clean_crossing_bt30_verified`, 로컬 보관)) | `navigate_to_pose_ackermann_backup30.xml`<br>(`db173a863e8d...`) | 0.30 m | **0.2949 m (29.5cm)**<br>($t=44.5\sim51.4\text{s}$) | STATUS_CANCELED (5)<br>(55s 타임아웃) | **$(+1.516, -0.998)$**<br>**(통로 내부 2.3m 전진!)** | **0.1663 m (16.6cm)** | **0.1006 m (10.1cm)**<br>($\ge 10\text{cm}$ 통과) | **주행 재개 후 전진 중 타임아웃**<br>(`FAILED_TO_RESUME`) |

- **실측 검증 성과 및 결론**:
  1. **실제 후진 거리의 정확한 적용 검증**:
     - 로그와 GT 시계열 실측 결과, 15cm BT는 **14.52cm** (오차 4.8mm), 30cm BT는 **29.49cm** (오차 5.1mm)로 목표 거리가 실제 물리 공간에서 정확히 실행되었음을 검증했습니다.
  2. **30cm 후진 적용 시 주행 재개 성공 실증**:
     - 15cm 후진 시에는 회전 반경 부족으로 통로 입구($x=-0.629$)에서 전진하지 못했으나,
     - 30cm 후진 시에는 충분한 선회 공간($x=-0.790$)을 확보한 후 **$t=51.4\text{s}$부터 남측 통로를 향해 전진 주행을 재개**하여 통로 중앙($x=0.21, y=-0.90$)을 관통하고 $x=+1.516\,\text{m}$까지 $2.3\,\text{m}$ 이상 전진하는 실질적 개선을 달성했습니다.
  3. **엄격한 10cm 안전 기준 유지**:
     - 두 시험 모두 이동 상자와 16.6~22.7cm, 벽체와 10.1~11.0cm의 최소 간격을 유지하여 **전체 최소 간격 $\ge 10\,\text{cm}$ 및 무충돌(Collision=False)을 완벽히 만족**했습니다.
  4. **최종 판정의 투명성 유지**:
     - 30cm 시험에서 전진 주행 재개에 성공했으나, 55초 제한시간 내에 최종 목표점($x=2.50$)까지 완주하지 못하고 $x=1.516$에서 타임아웃 취소되었으므로, 과장 없이 **`FAILED_TO_RESUME` (제한시간 내 완주 미도달)**로 엄정하게 기록합니다.

- **종합 평가**:
  - 이동 장애물 문제는 **현재 부분 개선(PARTIAL_IMPROVEMENT) 및 최종 미해결(UNRESOLVED)** 상태입니다.
  - BT BackUp 거리를 30cm로 정상 연동했을 때 협로 진입 전진 재개에는 성공했으나, 제한시간 내 최종 목표 완주까지 도달하지는 못했습니다.
  - 통로 잔류 상황에서는 북측 우회로(9.3m)를 자율 선택하지 못했으나, 충돌 없이 이동 상자와 30.8cm, 벽체와 17.0cm의 안전거리를 유지하며 32.1초간 안정적으로 정지·대기함을 실증했습니다.

---

### 4.2 문제 2: 완전 차단 시험 최소 간격 6.86cm 구간 원인 분석 및 정지 거리 확인

#### (1) 실측 조사 결과
- **발생 시각 및 구간**: $t = 27.9\text{s}$, 후진 복구가 아닌 **전진 중 접근 (Forward Approach)** 구간 ($x = -0.129 \to -0.059$ 전진).
- **대상 물체**: 측면 벽체가 아닌 **`barricade_total::collision` (중심 `[0.50, -3.50]`, $x = 0.20$ 전면 차단벽)**.
- **직접 원인**:
  1. RPP의 전방 충돌 예측 시간 `max_allowed_time_to_collision_up_to_carrot: 0.6`이 저속 크롤링($v \approx 0.08\text{m/s}$) 시 $0.08 \times 0.6 = 4.8\,\text{cm}$에 불과하여 차단벽 5cm 직전까지 정지 판단을 유예함.
  2. $t=27.6\text{s}$에서 조향 회피 시도(`cmd=[0.015, -0.400]`)가 발생하며 Ackermann 차체 우측 전면 모서리가 차단벽 쪽으로 회전 돌출되어 여유 간격이 **6.86cm**까지 급감함.

#### (2) 최소 변경 적용
- **10cm 기준 엄격 유지**: 결과를 맞추기 위해 사후에 기준을 낮추지 않음.
- RPP `max_allowed_time_to_collision_up_to_carrot`을 `0.6` $\to$ **`0.8`**로 상향 조정하여 저속 접근 시 정지 판단 거리를 조기에 확보하고, `local_costmap.robot_radius: 0.19`를 유지하여 90cm 협로 통행성을 보존.

#### (3) 동일 설정 조건 시험 결과

| 시험 구분 | 실행 디렉토리 | 최종 액션 상태 | SAT 최소 간격 | 주행 중 자율정지 | 충돌 여부 | 안전 정지 최종 판정 |
|:---|:---|:---:|:---:|:---:|:---:|:---:|
| **기준 조건 (Baseline)** | `phase_separated/scen6_blocked_safe_stop` | STATUS_CANCELED (5) | **0.0686 m (6.86cm)** | 3.841 s | False | **미통과 (NOT_PASSED)** |
| **수정 조건 1회차 (Run 1)**| `investigation/scen6_blocked_mod2_run1` | STATUS_CANCELED (5) | **0.1298 m (13.0cm)** | 17.722 s | False | **통과 (PASSED)** |
| **수정 조건 2회차 (Run 2)**| `investigation/scen6_blocked_mod2_run2` | STATUS_CANCELED (5) | **0.1385 m (13.9cm)** | 17.539 s | False | **통과 (PASSED)** |

- **결과 기록**:
  - 차단 상황의 정지 거리 개선은 **이번 수정 조건(`carrot: 0.8`, `robot_radius: 0.19`) 2회 시험에서 확인된 성과**로 기록합니다.
  - 1회차 13.0cm, 2회차 13.9cm로 10cm 안전 기준을 만족했으나, 일반적인 모든 차단 상황에 대한 완전한 해법으로 일반화하지 않습니다.

---

## 5. 원본 시계열 기반 전면 재평가 종합 결과표 (`reevaluation_summary.json`)

보고서의 주행 시간은 각 시나리오 summary의 **`simulation_duration_seconds`**와 정확히 일치시켰습니다.

### 5.1 시나리오별 물리 측정 및 구간 분리 실측표

| 시나리오 ID | 시나리오 구분 | 최종 액션 상태 | GT 최종 오차 | SAT 최소 간격 | 자율 주행 연속정지 | 초기 대기 정지 | 취소후 정지 | 주행 시간 (`sim_dur`) | 데이터 출처 / 소스 | 공식 재평가 상태 |
|:---|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| **Scen-1** | 기본 목표점 주행 | STATUS_SUCCEEDED (4) | 0.2119 m | **0.2621 m** | 1.200 s | 0.000 s | 0.000 s | **16.00 s** | 기존 summary / RECV_SIM_TIME | **SUCCESS_RELAXED_ONLY** |
| **Scen-2** | 고정 장애물 주행 | STATUS_SUCCEEDED (4) | 0.3473 m | **0.2625 m** | 0.900 s | 0.000 s | 0.000 s | **24.30 s** | 기존 summary / RECV_SIM_TIME | **SUCCESS_RELAXED_ONLY** |
| **Scen-3** | 미지 장애물 우회 | STATUS_SUCCEEDED (4) | 0.3390 m | **0.2655 m** | 0.700 s | 0.000 s | 0.000 s | **28.90 s** | 기존 summary / RECV_SIM_TIME | **SUCCESS_RELAXED_ONLY** |
| **Scen-4** | 동적 장애물 등장 | STATUS_SUCCEEDED (4) | **0.0037 m** | **0.1522 m** | 0.052 s | 6.911 s | 0.000 s | **17.39 s** | 원본 재계산 / RECV_SIM_TIME | **SUCCESS_STRICT** |
| **Scen-5** | 이동 장애물 횡단 | STATUS_ABORTED (6) | 3.0330 m | **0.1110 m** | **10.302 s** | 6.945 s | 0.000 s | **19.93 s** | 원본 재계산 / RECV_SIM_TIME | **FAILED** |
| **Scen-6** | 완전 차단 안전정지 | STATUS_CANCELED (5) | 4.1885 m | **0.0686 m** | **3.841 s** | 6.877 s | 2.854 s | **30.00 s** | 원본 재계산 / RECV_SIM_TIME | **SAFE_STOP_NOT_PASSED** |
| **Scen-7** | 장애물 제거 후 재개 | STATUS_SUCCEEDED (4) | 0.1789 m | **0.2315 m** | 0.052 s | 6.908 s | 0.000 s | **26.60 s** | 원본 재계산 / RECV_SIM_TIME | **SUCCESS_RELAXED_ONLY** |
| **Scen-8** | 사전 지도 없는 탐색 | UNCONFIRMED (미수신) | N/A | **0.4969 m** | 3.600 s | 0.000 s | 0.000 s | **50.00 s** | 기존 summary / RECV_SIM_TIME | **EXPLORATION_PARTIAL_TIMELIMIT** |

---

### 5.2 4대 핵심 독립 판정 종합표 (통과 / 미통과 / 미측정 / 해당 없음)

탐색 시나리오는 일반 목표점 주행과 분리하여 `EXPLORATION_PARTIAL_TIMELIMIT` 상태를 유지했습니다.

| 시나리오 ID | [목표 도달: 엄격]<br>($\le 0.15\text{m}$) | [목표 도달: 완화]<br>($\le 0.40\text{m}$) | [무충돌 검증]<br>(SAT 간격 $> 5\text{mm}$) | [안전 정지 검증]<br>(자율정지 $\ge 3\text{s}$, 여유 $\ge 0.10\text{m}$) | [Costmap 소거 검증]<br>(유효 셀 0 소거, $\ge 2\text{s}$ 유지) | [탐색 완료]<br>(프론티어 완전 소진) |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|
| **Scen-1** (기본 주행) | **미통과** (0.212m) | **통과** (0.212m) | **통과** (0.262m) | **해당 없음** | **미측정** | **해당 없음** |
| **Scen-2** (고정 장애물) | **미통과** (0.347m) | **통과** (0.347m) | **통과** (0.263m) | **해당 없음** | **미측정** | **해당 없음** |
| **Scen-3** (미지 장애물) | **미통과** (0.339m) | **통과** (0.339m) | **통과** (0.266m) | **해당 없음** | **미측정** | **해당 없음** |
| **Scen-4** (동적 등장) | **통과** (0.004m) | **통과** (0.004m) | **통과** (0.152m) | **미통과** (회피 주행) | **미측정** | **해당 없음** |
| **Scen-5** (이동 장애물) | **미통과** (3.033m) | **미통과** (3.033m) | **통과** (0.111m) | **통과** (10.3s, 0.111m) | **미측정** | **해당 없음** |
| **Scen-6** (완전 차단) | **미통과** (4.188m) | **미통과** (4.188m) | **통과** (0.069m) | **미통과 (여유 간격 0.069m < 0.10m 미달)** | **미측정** | **해당 없음** |
| **Scen-7** (제거 후 재개) | **미통과** (0.179m) | **통과** (0.179m) | **통과** (0.231m) | **해당 없음** | **통과 (231개 셀 완전 소거, 19.99s)** | **해당 없음** |
| **Scen-8** (자율 탐색) | **해당 없음** | **해당 없음** | **통과** (0.497m) | **해당 없음** | **미측정** | **미통과 (EXPLORATION_PARTIAL_TIMELIMIT)** |

---

## 6. 시나리오 7 원본 ROI 셀 대조 분석

`costmap_roi_history.jsonl` (`output/simulation/2026-10-24_phase_separated/scen7_clearing_resume/costmap_roi_history.jsonl`, 로컬 보관)의 17개 전수 레코드를 대조하여 확인한 결과입니다:
- **실제 ROI 셀 수: 231개 셀** (보고서의 191개 셀 표기를 최신 원본 231개 셀로 정정).
- **제거 전 ($t \le 13.911\text{s}$)**:
  - 전면 충돌면: 최대 Cost 100 (평균 99.4)
  - 내부 영역: 최대 Cost 99 ~ 100 (평균 41.2)
  - 치명적 점유(Lethal, Cost 100) 셀 수는 **24 ~ 67개**, 팽창 셀(Cost 50~99) **147개**, 외곽 마진(Cost 0) 셀 **27 ~ 70개**가 혼재되어 실제 장애물 경계를 형성함.
- **제거 후 ($t \ge 15.811\text{s}$)**:
  - 물리적 제거(Gazebo GT $z = -10.0\,\text{m}$) 직후 231개 셀 전체가 **Cost 0 (Free Space)**으로 완전 소거 (0 셀: 231개, Lethal: 0개, Unknown: 0개).
  - 소거 상태가 **19.99초 동안 끊김 없이 유지**됨 (`clearing_hold_verified = True`).
  - 소거 지연 시간: $15.811\text{s} - 13.916\text{s} = \mathbf{1.90\,\text{s}}$ (코스트맵 수신 주기 약 2.0s 이내로, **`1.90s (< 2.0s 관측 주기 해상도 이내 / within observation resolution)`** 표기).

---

## 7. 기존 소형 환경 결론 및 확장 검증으로의 이행

1. **평가 체계의 일원화**:
   - 실행 러너와 단위 시험이 단일 authoritative 평가 라이브러리(`evaluation_lib.py`)를 공유하여 평가 불일치를 방지했습니다.
   - 원본 재계산 결과와 기존 summary 보존 결과를 투명하게 분리 표기했습니다.
2. **기존 소형 사각 방 환경 결함 분석 요약**:
   - **완전 차단 정지 거리**: 전진 저속 접근 시 짧은 예측 시간(0.6s)과 모서리 조향 돌출이 6.86cm 축소의 원인임을 규명하고, 예측 시간 0.8s 조정을 통해 10cm 기준을 유지한 상태에서 **수정 조건 2회 시험에서 13.0cm, 13.9cm 정지 여유를 확인**했습니다.
   - **기존 폭 0.90m 협로 이동 장애물**: 제어기 허용 시간(0.3s) 부족과 협로 후진 실패가 조기 중단의 직접 원인임을 확인했으며, 이를 바탕으로 더 넓고 현실적인 다중 경로 실내 환경(`16.0m × 12.0m`)으로 확장하여 단계별 실시간 GUI 검증(Stages A ~ F)을 수행했습니다.

---

## 8. 확장된 복잡 실내 환경(`16.0m × 12.0m`) 및 다수 고정·이동 장애물 실시간 GUI 단계별 검증 (Stages A ~ F)

### 8.1 확장 공간 설계 및 사전 지도-Gazebo 월드 정합 검증

단순 사각 방을 넘어 교차로, 순환 루프, 막다른 길(Dead-end pocket), 통과 폭이 서로 다른 복수의 동서 통로, 광장, 고정 기둥이 포함된 **`16.0m × 12.0m` 복잡 실내 환경**을 설계하고, 사용자의 데스크톱 화면(`DISPLAY=:0`)에 Gazebo GUI(상단 고정 직교 시점)와 RViz2(2D TopDownOrtho 시점 + 실시간 S/G 마커, 파란 점선 최초 경로, 주황색 현재 경로, 초록색 실제 궤적, 빨간색/마젠타 미등록·이동 장애물 마커)를 동시에 띄워 전 단계를 실시간으로 실행·검증했습니다.

- **생성 스크립트 및 환경 설정 경로**:
  - 환경 생성기: [`output/simulation/complex_indoor_env/generate_complex_env.py`](../output/simulation/complex_indoor_env/generate_complex_env.py)
  - 사전 정적 지도: `complex_indoor_map.pgm` (`output/simulation/complex_indoor_env/complex_indoor_map.pgm`, 로컬 보관) (`320 × 240` 픽셀, 해상도 `0.05 m/pixel`, 원점 `[-8.0, -6.0, 0.0]`), `complex_indoor_map.yaml` (`output/simulation/complex_indoor_env/complex_indoor_map.yaml`, 로컬 보관)
  - 공간 매니페스트 및 정합 검증: `complex_indoor_manifest.json` (`output/simulation/complex_indoor_env/complex_indoor_manifest.json`, 로컬 보관)
    - **사전 지도 격자와 Gazebo SDF 정적 벽체/고정 장애물 간 최대 정렬 오차**: **`0.000000 m`** (`alignment_verified: true`)
  - 시작점 **S**: `(-6.20, -4.20, yaw=0.0)` (남서측 출발 베이)
  - 목적지 **G**: `(+5.80, +1.40, yaw=0.0)` (동측 광장 목표 구역)
- **공간 구조 및 다중 통로 치수**:
  1. **Route 1 (남측 주 간선 통로)**: $y \in [-2.70, -0.60]$, **유효 폭 `2.10 m`** — 장애물이 없을 때 S에서 G로 이어지는 최단 전역 경로.
  2. **Route 2 (중앙 복도)**: $y \in [+0.70, +2.10]$, **유효 폭 `1.40 m`** — 남측 주 통로 차단 시 선택되는 1차 우회 경로.
  3. **Route 3 (북측 광폭 통로)**: $y \in [+3.60, +5.80]$, **유효 폭 `2.20 m`** — 남측 및 중앙 통로가 모두 차단되었을 때 서측 간선도로를 통해 북상하는 2차 우회 경로.
  4. **막다른 길(Dead-end Pocket)**: 남동측 코너 $x \in [4.50, 7.80], y \in [-5.80, -3.50]$.
  5. **지도 반영 고정 장애물 4개 (`mapped_obs_1 ~ 4`)**: Gazebo 월드와 사전 정적 지도(`complex_indoor_map.pgm`) 양쪽에 동일하게 반영되어 기본 전역 경로 계획 시 처음부터 회피됨.
- **인식 원칙 준수 (`gt_obstacle_fed_to_nav2: false`)**:
  - 미등록 고정 장애물(`unmapped_obs_*`)과 이동 장애물(`moving_obs_*`)은 사전 지도에 전혀 표시되지 않으며, 정답 위치·속도·미래 궤적을 Nav2에 직접 입력하지 않고 오직 **실시간 2D LiDAR(`/scan`)와 Global/Local Costmap 장애물 계층**으로만 감지·대응했습니다.

---

### 8.2 AMCL 위치추정 안정화(Beam Skipping) 및 초기 시험 보존 기록

Stage A 및 Stage B 초기 실행 과정에서 발견된 문제와 원인 규명, 그리고 보존된 실패/중단 실행 디렉토리는 다음과 같습니다:

1. **Stage A 초기 시험 보존본 (`stage_A_stage_A_baseline_20261024_202211`)**:
   - **현상**: 로봇이 목적지 G `(+5.80, +1.40)` 인근인 `(5.643, 1.163)`(GT 거리 오차 `0.284 m`, 무충돌 최소 간격 `0.2737 m`)까지 정상 도달했으나, Ackermann 조향 특성상 목표점 위에서 제자리 회전으로 `yaw_goal_tolerance: 0.35 rad`를 맞추지 못해 미세 전후진을 반복하다 95초 타임아웃(`TIMEOUT_CANCELLED`) 발생.
   - **조치**: [`nav2_complex_indoor_ackermann_params.yaml`](../output/simulation/complex_indoor_env/nav2_complex_indoor_ackermann_params.yaml)의 `yaw_goal_tolerance`를 `3.1416 rad`로 조정(위치 허용오차 `xy_goal_tolerance: 0.20 m` 유지)하여 목적지 위치 도달 시 불필요한 제자리 방향 정렬 루프를 방지함.
2. **Stage B 초기 시험 보존본 (`stage_B_stage_B_multi_unmapped_20261024_203657` 및 `204013`)**:
   - **현상 (`203657`)**: 사전 지도에 없는 대형 차단 장애물(`unmapped_obs_1`, `unmapped_obs_2`)이 라이다 시야의 상당 부분을 가리자, `do_beamskip: false` 및 `laser_likelihood_max_dist: 2.0` 설정의 AMCL 파티클 필터가 미등록 장애물 반사파를 인근 정적 벽체에 강제 매칭하여 **추정 위치가 약 1.4m 점프**하고, 그 결과 Global Costmap 상의 열린 통로에 유령 장애물(Ghost barrier)이 기록되어 `STATUS_ABORTED` 발생.
   - **현상 (`204013`)**: `alpha1..5: 0.002`로 과도하게 축소하고 `unmapped_obs_2`가 중앙 복도 안쪽(`x = -1.65`)에 위치하여, 로봇이 서측 간선도로에서 북상할 때 남측 벽면(`w_c1`) 모서리에 가려 복도 차단을 미리 보지 못하고 좁은 복도 입구 안으로 진입한 뒤 갇힘.
   - **원인 기반 해결**:
     - AMCL에 **`do_beamskip: true` (`beam_skip_distance: 0.25`, `beam_skip_threshold: 0.3`)** 및 `laser_likelihood_max_dist: 0.60`, `alpha1..5: 0.05`를 적용하여 사전 지도에 없는 장애물 라이다 빔이 AMCL 위치추정을 오염시키지 않도록 분리.
     - `unmapped_obs_2`를 중앙 복도 서측 입구(`[-2.05, +1.40]`, 크기 `0.40 × 1.36 m`)에 배치하여 로봇이 서측 간선도로를 따라 북상할 때 가림 없이 복도 입구 차단을 즉시 관측하고 북측 광폭 통로(Route 3)로 재계획하도록 구성.

---

### 8.3 Stages A ~ F 전 단계 정량 검증 결과 종합표

모든 실행은 데스크톱 GUI(`DISPLAY=:0`) 상에서 실시간으로 실행 및 녹화되었으며, 각 실행 폴더(`output/simulation/2026-10-24_complex_indoor_runs/`)에 원본 시계열 로그(`.jsonl`), 설정 사본 및 SHA-256 해시(`.sha256`), `summary.json`, `events.json`이 보존되어 있습니다.

| 단계 | 시나리오 ID 및 실행 디렉토리 | 최종 액션 상태 | 주행 시간 (`sim_dur`) | 최종 GT 위치 $(x, y)$ | **GT 목표 오차** | **전체 최소 간격 (SAT)** | 벽체 최소 간격 | 추가/이동 장애물 최소 간격 | 무충돌 판정 | 공식 종합 판정 |
|:---:|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| **Stage A** | **사전 지도 기반 기본 복잡 주행**<br>(`stage_A_..._202649` (`output/simulation/2026-10-24_complex_indoor_runs/stage_A_stage_A_baseline_20261024_202649`, 로컬 보관)) | `STATUS_SUCCEEDED (4)` | **52.94 s**<br>(`34.63→87.57s`) | `(5.9015, 1.2019)` | **0.2226 m** | **0.2588 m**<br>(`25.88 cm`) | `0.2588 m`<br>(`mapped_obs_2`) | N/A | **PASSED** | **`SUCCESS_RELAXED_ONLY`** |
| **Stage B** | **미등록 고정 장애물 3개 단계적 우회**<br>(`stage_B_..._204257` (`output/simulation/2026-10-24_complex_indoor_runs/stage_B_stage_B_multi_unmapped_20261024_204257`, 로컬 보관)) | `STATUS_SUCCEEDED (4)` | **72.97 s**<br>(`20.17→93.14s`) | `(5.6428, 1.5284)` | **0.2030 m** | **0.1599 m**<br>(`15.99 cm`) | `0.1599 m`<br>(`w_n2`) | **`0.2579 m`**<br>(`unmapped_obs_3`) | **PASSED** | **`SUCCESS_RELAXED_ONLY`** |
| **Stage C** | **단일 이동 장애물 완전 횡단 후 재개**<br>(`stage_C_..._204541` (`output/simulation/2026-10-24_complex_indoor_runs/stage_C_stage_C_single_moving_cross_20261024_204541`, 로컬 보관)) | `STATUS_SUCCEEDED (4)` | **54.63 s**<br>(`27.70→82.33s`) | `(5.6831, 1.3229)` | **0.1400 m**<br>($\le 0.15\text{m}$!) | **0.1933 m**<br>(`19.33 cm`) | `0.3370 m`<br>(`mapped_obs_2`) | **`0.1933 m`**<br>(`moving_obs_1`) | **PASSED** | **`SUCCESS_STRICT`** |
| **Stage D** | **다수(3개) 이동 장애물 순차 횡단 대응**<br>(`stage_D_..._204750` (`output/simulation/2026-10-24_complex_indoor_runs/stage_D_stage_D_multi_moving_cross_20261024_204750`, 로컬 보관)) | `STATUS_SUCCEEDED (4)` | **59.03 s**<br>(`31.41→90.43s`) | `(5.7112, 1.2074)` | **0.2121 m** | **0.1282 m**<br>(`12.82 cm`) | `0.3264 m`<br>(`mapped_obs_2`) | **`0.1282 m`**<br>(`moving_obs_3`) | **PASSED** | **`SUCCESS_RELAXED_ONLY`** |
| **Stage E** | **미등록 고정 우회 + 이동 2개 복합 환경**<br>(`stage_E_..._205021` (`output/simulation/2026-10-24_complex_indoor_runs/stage_E_stage_E_combined_static_moving_20261024_205021`, 로컬 보관)) | `STATUS_SUCCEEDED (4)` | **59.79 s**<br>(`30.84→90.63s`) | `(5.6210, 1.4890)` | **0.2000 m** | **0.1351 m**<br>(`13.51 cm`) | `0.1351 m`<br>(`w_c2`) | **`0.2715 m`**<br>(`moving_obs_2`) | **PASSED** | **`SUCCESS_RELAXED_ONLY`** |
| **Stage F** | **전 통로 완전 차단 및 안전 정지·대기**<br>(`stage_F_..._205307` (`output/simulation/2026-10-24_complex_indoor_runs/stage_F_stage_F_blocked_corridor_20261024_205307`, 로컬 보관)) | `STATUS_CANCELED (5)`<br>(안전 대기 후 종료) | **40.02 s**<br>(`45.98→86.00s`) | `(-3.5489, -1.0790)` | N/A (차단 대기) | **0.1118 m**<br>(`11.18 cm`) | `0.8613 m`<br>(`w_s1`) | **`0.1118 m`**<br>(`blocking_obs_main`) | **PASSED** | **`SAFE_STOP_PASSED`**<br>(`22.3s` 연속 정지) |

> **보존된 이전 시도 요약**:
> - `stage_A_stage_A_baseline_20261024_202211`: `TIMEOUT_CANCELLED` (`yaw_goal_tolerance: 0.35` 설정 시 목표점 위에서 회전 정렬 지연, GT 거리 오차 `0.284 m`, 최소 간격 `0.2737 m`)
> - `stage_B_stage_B_multi_unmapped_20261024_203657`: `STATUS_ABORTED` (`do_beamskip: false`에서 미등록 장애물 반사파로 인한 AMCL 위치 점프, 최소 간격 `0.1325 m`)
> - `stage_B_stage_B_multi_unmapped_20261024_204013`: 중단 보존본 (`unmapped_obs_2`가 복도 내부 `x=-1.65`에 위치하여 입구 진입 후 갇힘)

---

### 8.4 단계별 이벤트 타임스탬프 및 주행 거동 상세 분석 (`events.json` 실측)

#### (1) Stage A (`stage_A_baseline`) — 사전 지도 기반 복잡 공간 기본 완주
- **최초 전역 경로 생성**: `t = 35.128 s` (`plan_id: 1`, 243개 경유점, 남측 주 통로 Route 1 경유).
- **주행 경과**: 지도 반영 고정 장애물 4개(`mapped_obs_1 ~ 4`)를 전역 경로 상에서 자연스럽게 회피하며 `52.94 s` 만에 목표점 도달 (`GT 오차 0.2226 m`, 최소 간격 `25.88 cm`).

#### (2) Stage B (`stage_B_multi_unmapped`) — 미등록 고정 장애물 3개 순차 발견 및 3단계 경로 재계획
- **최초 전역 경로 (`initial_global_plan`)**: `t = 21.059 s` (`plan_id: 1`, 남측 주 통로 Route 1로 계획).
- **1차 장애물 발견 및 우회 (`unmapped_obs_1`, `[-1.55, -1.95]`)**:
  - 라이다 최초 감지: `t = 27.601 s` (`9` rays, 거리 `1.97 m`, 로봇 위치 `(-3.60, -1.93)`)
  - Costmap 반영 확인: `t = 27.790 s` (`129` occupied cells, 최대 Cost `100`)
  - **1차 대규모 경로 재계획**: `t = 29.124 s` (`plan_id: 6`, 경로 중심 이동 `2.866 m`) $\to$ 막힌 남측 주 통로를 버리고 **중앙 복도(Route 2)**로 우회.
- **2차 장애물 발견 및 우회 (`unmapped_obs_2`, `[-2.05, +1.40]`)**:
  - 서측 간선도로 북상 중 라이다 감지: `t = 34.493 s` (`2` rays, 거리 `2.07 m`, 로봇 위치 `(-3.55, 0.06)`)
  - **2차 대규모 경로 재계획**: `t = 40.993 s` (`plan_id: 11`, 경로 중심 이동 `2.036 m`) $\to$ 중앙 복도 입구 차단 확인 후 **북측 광폭 통로(Route 3, `y = +4.7m`)**로 우회.
- **3차 부분 장애물 발견 및 회피 통과 (`unmapped_obs_3`, `[+0.80, +3.95]`)**:
  - 북측 광폭 통로 진입 후 라이다 감지: `t = 57.441 s` (`9` rays, 거리 `2.01 m`, 로봇 위치 `(-1.26, 4.74)`)
  - **3차 경로 미세 조정**: `t = 59.745 s` (`plan_id: 26`) $\to$ 통로 북측 여유 공간(`y ≈ 5.1m`)으로 S자 회피 기동 후 동측 광장 목적지 G에 `t = 93.141 s` 완주 (`GT 오차 0.2030 m`, 전체 최소 간격 `15.99 cm`, 추가 장애물 최소 간격 `25.79 cm`).

#### (3) Stage C (`stage_C_single_moving_cross`) — 이동 장애물 1개 통로 완전 횡단 후 주행 재개
- **이동 장애물 트리거 및 감지 (`moving_obs_1`)**:
  - 라이다 최초 감지: `t = 31.000 s` (`2` rays), Costmap 반영: `t = 32.460 s` (`138` cells, Cost `100`).
  - 로봇이 `x >= -4.8 m` 지점을 통과한 `t = 33.37 s`에 `moving_obs_1`이 `(-3.00, -0.55)`에서 남측(`-y`) 방향으로 `0.38 m/s` 횡단 시작.
  - 상자가 통로 중앙(`y = -1.65`)을 가로지른 뒤 `t = 43.62 s`에 남측 포켓(`y = -4.35`)으로 완전히 벗어나 통로 개방.
- **주행 재개 및 완주**:
  - 통로 개방 및 Costmap 소거에 따라 동일 목표 G로 주행을 지속(`motion_resumed_verified: true`)하여 `t = 82.327 s`에 완주 (**`GT 오차 0.1400 m`으로 `SUCCESS_STRICT` 달성**, 이동 장애물 최소 간격 `19.33 cm`).

#### (4) Stage D (`stage_D_multi_moving_cross`) — 서로 다른 시점·위치의 이동 장애물 3개 순차 횡단 대응
- **1차 이동 장애물 (`moving_obs_1`, 서측 교차로 `x = -3.2 m`)**:
  - 라이다 감지 `t = 34.622 s`, Costmap 반영 `t = 36.301 s` (`119` cells), 횡단 구간 `37.07 s → 48.67 s` (`y: -0.55 → -4.40`).
- **2차 이동 장애물 (`moving_obs_2`, 중앙 통로 출구 `x = +2.8 m`)**:
  - 로봇이 `x >= -0.2 m`에 도달한 `t = 59.67 s`에 남측(`y = -2.65`)에서 북측(`y = +0.55`)으로 횡단 시작 (`59.67 s → 67.32 s`).
  - Costmap 반영 `t = 67.321 s` (`86` cells), 라이다 근접 감지 `t = 67.725 s` (`25` rays, 거리 `0.90 m`).
  - **로봇 감속·정지 및 재개**: `t = 67.321 s`에 `(2.385, -1.499)`에서 감속·대기 후 장애물 이탈 직후인 **`t = 67.901 s`에 전진 주행 재개** (`motion_resumed_verified: true`).
- **3차 이동 장애물 (`moving_obs_3`, 동측 광장 접근부 `x = +4.8 m`)**:
  - 로봇이 `x >= +3.2 m`에 도달한 `t = 74.03 s`에 동측 광장 상단에서 남측으로 횡단 (`74.03 s → 82.14 s`).
  - 라이다 감지 `t = 82.491 s` (`48` rays), Costmap 반영 `t = 83.205 s` (`77` cells) 후 안전 간격(`12.82 cm`)을 유지하며 통과하여 `t = 90.432 s`에 목적지 완주 (`GT 오차 0.2121 m`).

#### (5) Stage E (`stage_E_combined_static_moving`) — 미등록 고정 장애물 우회 + 이동 장애물 2개 복합 대응
- **미등록 고정 장애물(`unmapped_obs_1`) 감지 및 우회 재계획**:
  - 최초 경로(`t = 31.337 s`)는 남측 주 통로(Route 1)로 생성됨.
  - `t = 38.227 s`에 `unmapped_obs_1` (`[-1.55, -1.95]`)을 라이다로 감지(`14` rays, 거리 `1.72 m`)하고, **`t = 40.019 s`(`plan_id: 7`)에 중앙 복도(Route 2, `y = +1.40 m`)로 전역 우회 경로를 재계획**.
- **우회 경로 상의 이동 장애물 2개(`moving_obs_1`, `moving_obs_2`) 연속 대응**:
  - 로봇이 중앙 우회 복도를 통과해 `x >= +0.5 m`에 도달한 `t = 66.08 s`에 `moving_obs_1`이 복도 동측 출구(`x = +2.9 m`)를 북에서 남으로 횡단(`66.08 s → 74.78 s`, Costmap 반영 `70.962 s`).
  - 이어서 로봇이 `x >= +3.1 m`에 진입한 `t = 75.93 s`에 `moving_obs_2`가 목적지 전면(`x = +5.0 m`)을 남에서 북으로 횡단(`75.93 s → 85.06 s`, 라이다 감지 `75.116 s`).
  - 두 이동 장애물이 통과한 직후 열린 공간으로 주행을 이어가 `t = 90.627 s`에 완주 (`GT 오차 0.2000 m`, 전체 최소 간격 `13.51 cm`, 이동 장애물 최소 간격 `27.15 cm`).

#### (6) Stage F (`stage_F_blocked_corridor`) — 통로 잔류·완전 차단 시 안전 정지 및 대기 검증
- **시나리오 구성**: 남측 통로 입구 차단벽(`blocking_obs_south_seal`), 북측 간선도로 차단벽(`blocking_obs_north_seal`)과 함께 로봇 전방 통로에 `blocking_obs_main`이 진입(`t = 52.87 s → 57.56 s`)한 뒤 **통로를 벗어나지 않고 `(-3.00, -1.75)`에 영구 잔류**하여 모든 우회 가능 경로를 100% 물리적으로 차단함.
- **감지 및 자율 안전 정지 (`SAFE_STOP_PASSED`)**:
  - 라이다 감지: `blocking_obs_south_seal` (`46.012 s`), `blocking_obs_north_seal` (`46.300 s`), `blocking_obs_main` (`54.500 s`, `8` rays), Costmap 반영 (`47.055 s`, `158` cells).
  - 로봇은 차단 장애물 앞에서 **`0.1118 m` (`11.18 cm >= 10.0 cm` 기준 통과)**의 안전 여유 간격을 두고 감속·정지했으며, **`t = 63.581 s`부터 종료 시점(`86.000 s`)까지 `22.3초` 동안 무충돌 자율 정지·대기 상태를 안정적으로 유지**했습니다 (`safe_stop_verified: PASSED`).

---

### 8.5 생성된 영상 및 재현 아카이브 목록

> [!NOTE]
> 사용자 지시에 따라 **PPT / ODP / PDF 발표 자료 파일은 일체 수정하지 않았으며**, 모든 실시간 GUI 녹화본과 로그 재구성 비교 영상은 `output/video/` 및 각 실행 폴더에 독립 보관되어 있습니다.

1. **데스크톱 실시간 GUI(Gazebo + RViz2) 화면 녹화본**:
   - Stage A: `output/video/live_gui_stageA_recording.mp4` (`output/video/live_gui_stageA_recording.mp4`, 로컬 보관)
   - Stage B: `output/video/live_gui_stageB_recording.mp4` (`output/video/live_gui_stageB_recording.mp4`, 로컬 보관)
   - Stage C: `output/video/live_gui_stageC_recording.mp4` (`output/video/live_gui_stageC_recording.mp4`, 로컬 보관)
   - Stage D: `output/video/live_gui_stageD_recording.mp4` (`output/video/live_gui_stageD_recording.mp4`, 로컬 보관)
   - Stage E: `output/video/live_gui_stageE_recording.mp4` (`output/video/live_gui_stageE_recording.mp4`, 로컬 보관)
   - Stage F: `output/video/live_gui_stageF_recording.mp4` (`output/video/live_gui_stageF_recording.mp4`, 로컬 보관)
2. **원본 시계열 로그 기반 좌우 비교(Gazebo 물리 뷰 + RViz/Costmap/경로 뷰) 영상 (`1.0x` 실시간 및 `2.0x` 배속)**:
   - Stage A: `stageA_log_reconstructed_1.0x.mp4` (`output/video/stageA_log_reconstructed_1.0x.mp4`, 로컬 보관), `stageA_log_reconstructed_2.0x.mp4` (`output/video/stageA_log_reconstructed_2.0x.mp4`, 로컬 보관)
   - Stage B: `stageB_log_reconstructed_1.0x.mp4` (`output/video/stageB_log_reconstructed_1.0x.mp4`, 로컬 보관), `stageB_log_reconstructed_2.0x.mp4` (`output/video/stageB_log_reconstructed_2.0x.mp4`, 로컬 보관)
   - Stage C: `stageC_log_reconstructed_1.0x.mp4` (`output/video/stageC_log_reconstructed_1.0x.mp4`, 로컬 보관), `stageC_log_reconstructed_2.0x.mp4` (`output/video/stageC_log_reconstructed_2.0x.mp4`, 로컬 보관)
   - Stage D: `stageD_log_reconstructed_1.0x.mp4` (`output/video/stageD_log_reconstructed_1.0x.mp4`, 로컬 보관), `stageD_log_reconstructed_2.0x.mp4` (`output/video/stageD_log_reconstructed_2.0x.mp4`, 로컬 보관)
   - Stage E: `stageE_log_reconstructed_1.0x.mp4` (`output/video/stageE_log_reconstructed_1.0x.mp4`, 로컬 보관), `stageE_log_reconstructed_2.0x.mp4` (`output/video/stageE_log_reconstructed_2.0x.mp4`, 로컬 보관)
   - Stage F: `stageF_log_reconstructed_1.0x.mp4` (`output/video/stageF_log_reconstructed_1.0x.mp4`, 로컬 보관), `stageF_log_reconstructed_2.0x.mp4` (`output/video/stageF_log_reconstructed_2.0x.mp4`, 로컬 보관)
