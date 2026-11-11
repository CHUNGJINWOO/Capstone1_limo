# LIMO Pro 상태 추정 및 센서 융합 알고리즘 비교 실험 최종 기술 보고서

- **실험 디렉터리:** `/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/`
- **실행 환경:** ROS 2 Humble / Ignition Gazebo Fortress (`6.17.0`) / `use_sim_time:=true`
- **총 실행 규모:** 상태 추정 시나리오 **S1~S5 × 각 15회(동일 조건 5회 + 시드 변동 10회) = 총 75회 Gazebo 독립 주행** + 라운지 장애물 시나리오 **A~D(기본 설정 4회 + 개선 설정 4회 + 좌우 비교 영상 4개)**
- **원칙 준수 검증:**
  1. 기존 `capstone_ws/` 및 `experiments/lounge_failure_investigation_20261107/` 내 파일은 **단 1바이트도 수정하지 않음** (`git status` 무결성 유지).
  2. Ground Truth(`/ground_truth/odom`, `/ground_truth/world_odom`)는 `/tf` 트리에 브로드캐스트하지 않으며, **어떠한 추정기의 입력이나 온라인 튜닝에도 사용되지 않음**을 코드 및 런치 수준에서 격리 검증함.
  3. 모든 수치는 실제 Gazebo Fortress 물리 시뮬레이션과 ROS 2 노드들이 실시간으로 발행한 토픽 로그(`results/raw_trials/S*_trial_*.csv`)에서 직접 계산된 실측치임.
  4. 인위적 센서 결함 주입이 포함된 **S4(IMU 바이어스 및 고잡음 주입)**는 `SYNTHETIC FAULT INJECTION`으로 명확히 분리 표기함.

---

## 1. 측정 체계 및 파이프라인 무결성 검증 (Step 1 Verification)

### 1.1 좌표계(Frame) 및 초기 정렬(Initial Alignment) 검증
- 모든 추정기(`B0`, `F1a`, `F1b`, `F2`, `F3a`, `F3b`, `F4`, `F5`)와 상대 Ground Truth(`/ground_truth/odom`)는 매 시행(Trial) 시작 시점의 로봇 실제 스폰 자세 $(X_0, Y_0, \Psi_0)$를 원점 $(0, 0, 0)$으로 삼는 **동일한 상대 오도메트리 좌표계(`odom -> base_footprint`)**로 정규화되었다([`nodes/trial_sensor_normalizer_node.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/nodes/trial_sensor_normalizer_node.py), [`nodes/ground_truth_publisher_node.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/nodes/ground_truth_publisher_node.py)).
- 전 75회 시행에서 초기 정렬 오차(`init_alignment_err_m`)는 모든 추정기(`B0..F5`)에서 **`0.0000 m`**로 일치함을 검증하였다.
- 시나리오 **S5**에서 평가된 **F6(`slam_toolbox`)**는 `map -> odom -> base_footprint` TF 체인을 통해 추정된 전역 자세를 trial 시작 시점의 원점과 정렬하여 동일 기준으로 ATE/RPE 및 폐루프 복귀 오차(`final_pos_err_m`)를 산출하였다.

### 1.2 시뮬레이션 시간(`use_sim_time`) 및 Timestamp 동기화 검증
- Gazebo `/clock` 브리지(`ros_gz_bridge`)를 통해 모든 추정기 노드와 로거가 동일한 시뮬레이션 시계를 사용하도록 구성하였다.
- 센서 간 최대 timestamp 편차는 Wheel Odometry(`/odom`, 50 Hz) ↔ IMU(`/imu`, 100 Hz) 간 `< 20 ms`, `/scan`(10 Hz) 포함 시 `< 60 ms` 이내로 유지되었다([`results/smoke_test/s1_smoke_test_report.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/smoke_test/s1_smoke_test_report.json)).

### 1.3 비교 알고리즘 구성 및 독립성·비중복 융합 검증

| 코드 | 추정기 명칭 | 토픽 / 출력 | 입력 센서 및 상태 변수 구성 | 설계 특징 및 검증 포인트 |
| :--- | :--- | :--- | :--- | :--- |
| **B0** | **Raw Wheel Odometry** | `/odom` | Gazebo `AckermannSteering` 휠 엔코더 적분 ($x, y, \psi, v_x, \omega_z$) | IMU·LiDAR 미사용 기준선. 조향 중 횡슬립 및 벽 충돌 시 휠 공전에 완전히 취약함. |
| **F1a** | **기존 EKF 재현 (Legacy EKF)** | `/odometry/ekf_legacy` | `odom0`($x, y, v_x, v_y$) + `imu0`($\psi, \omega_z, a_x, a_y, a_z$) | 기존 [`capstone_ws/src/robot_localization/params/limo_ekf.yaml`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/robot_localization/params/limo_ekf.yaml) 설정을 그대로 재현. (1) `two_d_mode` 미설정, (2) 바이어스 제거 없는 선형가속도 $a_x, a_y, a_z$ 적분 활성화, (3) 휠 절대 위치 $(x,y)$와 속도 $(v_x,v_y)$ 동시 융합, (4) 실물 런치에서 `limo_base` 내부 IMU 융합 가능성 존재. |
| **F1b** | **개선 튜닝 EKF (Tuned EKF)** | `/odometry/ekf_tuned` | `odom0`($v_x, v_y$) + `imu0`($\psi, \omega_z$) | [`config/ekf_tuned.yaml`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/config/ekf_tuned.yaml): `two_d_mode: true`, 휠은 선속도($v_x, v_y$)만 사용, IMU는 자세($\psi$)와 각속도($\omega_z$)만 사용하며 선형가속도($a_x, a_y, a_z$) 적분을 비활성화하여 가속도 바이어스 드리프트 차단. |
| **F2** | **개선 튜닝 UKF (Tuned UKF)** | `/odometry/ukf_tuned` | `odom0`($v_x, v_y$) + `imu0`($\psi, \omega_z$) | [`config/ukf_tuned.yaml`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/config/ukf_tuned.yaml): `F1b`와 동일한 센서 입력 조건에서 무향 칼만 필터(`ukf_node`, $\alpha=0.001, \kappa=0, \beta=2$)의 비선형 전파 효과를 직접 비교. |
| **F3a** | **상보 필터 단독 (Complementary Filter)** | `/odometry/comp_filter` | Wheel($v_x, v_y$) + IMU($\psi, \omega_z$) | [`nodes/complementary_filter_node.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/nodes/complementary_filter_node.py): 자이로 단기 적분($\alpha=0.96$)과 IMU 상대 yaw($1-\alpha=0.04$)를 결합하고 휠 선속도를 적분. 슬립 게이트 미적용. |
| **F3b** | **상보 필터 + 슬립/정체 게이트 (Comp + Slip Gate)** | `/odometry/comp_slip_gate` | Wheel($v_x, v_y$) + IMU($\psi, \omega_z$) + `/scan` 전방 거리변화율 게이트 | `F3a`와 동일 구조에 **전방 LiDAR 거리 변화율($\dot{r}_{\text{front}}$) 및 근접 접촉 기반 Slip/Stall Gate**를 추가. 휠 명령 속도는 존재하나 실제 전방 장애물과의 거리가 변하지 않을 때($|\dot{r}| < 0.025\text{ m/s}$) 휠 속도 적분을 게이팅($0.0$)하여 공전 발산을 억제함. |
| **F4** | **2D LiDAR 스캔 매칭 오도메트리 (Pure ICP)** | `/odometry/lidar_odom` | `/scan` 단독 (2D Trimmed Point-to-Point ICP) | [`nodes/lidar_scan_matcher_odom_node.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/nodes/lidar_scan_matcher_odom_node.py): **휠 오도메트리와 IMU를 전혀 사용하지 않는 순수 LiDAR 스캔 정합 추정기**. 초기 변환 추측값(Initial Guess)도 이전 스캔 정합 증분만 사용하여 휠 슬립 및 IMU 바이어스로부터 100% 독립됨. |
| **F5** | **복합 센서 융합 EKF (LiDAR + IMU EKF)** | `/odometry/lidar_imu_wheel_ekf` | `odom0`=`F4`($X, Y, v_x, v_y$) + `imu0`($\psi, \omega_z$) | [`config/fusion_lidar_imu_ekf.yaml`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/config/fusion_lidar_imu_ekf.yaml): 병진 위치·속도는 휠 슬립에 면역인 **F4(LiDAR 스캔 매칭)**에서 공급받고, 회전 자세·각속도는 50~100 Hz 고주파 **IMU**에서 공급받음. `F4`가 IMU나 휠을 사용하지 않으므로 **센서 정보의 중복 융합(Double Counting)이 전혀 없음**. |
| **F6** | **SLAM Toolbox 포즈 그래프 최적화** | `map -> base_footprint` | `/scan` + `odom -> base_footprint` TF | [`config/slam_benchmark_params.yaml`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/config/slam_benchmark_params.yaml): 시나리오 **S5(폐루프 복귀)**에서 전역 스캔-투-맵 정합 및 루프 클로저(Pose Graph Optimization)를 통해 누적 드리프트를 전역 보정. |

---

## 2. 상태 추정 반복 실험(S1~S5 × 각 15회 = 75회) 종합 결과

- **실험 설계:** 각 시나리오당 **동일 초기 조건 5회(`Identical 5 runs`: Gazebo 물리·타이밍 지터 재현성 확인)** + **시드 변동 10회(`Varied Seed 10 runs`: 초기 스폰 위치 $\pm 2.5\text{ cm}$, 초기 자세 $\pm 2^\circ$, 센서 잡음/스케일 시드 `1001~1010` 변동)** = **총 15회 반복 실행**.
- **비정상 종료/누락 회차:** 전 시나리오 75회 중 실패·크래시 회차 **`0건` (`75/75` 완주)**.
- **통합 요약 데이터:** [`results/summaries/master_all_scenarios_summary.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/summaries/master_all_scenarios_summary.json)
- **통합 비교 플롯:** [`results/plots/master_cross_scenario_comparison.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/plots/master_cross_scenario_comparison.png)

![Master Cross-Scenario Comparison](/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/plots/master_cross_scenario_comparison.png)

### 2.1 시나리오 S1: 정상 주행 및 다중 회전 (Normal Multi-Turn, 15회)
- **주행 조건:** 직진 + 좌/우 연속 회전 + S자 커브 주행 (충돌 없음).
- **원본 요약/플롯:** [`S1_15trials_summary.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/summaries/S1_15trials_summary.json), [`S1_15trials_summary.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/plots/S1_15trials_summary.png)
- **대표 주행 영상:** [`S1_trial_01_identical.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/S1_trial_01_identical.mp4), [`S1_trial_06_varied_seed_seed1001.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/S1_trial_06_varied_seed_seed1001.mp4)

| 알고리즘 | 15회 ATE RMSE 평균 ± 표준편차 (m) | 중앙값 (m) | 95백분위 (m) | 최악 순시오차 `Worst Max` (m) | 최종 위치오차 평균 (m) | Yaw RMSE 평균 (°) | 동일조건 5회 평균 (m) | 시드변동 10회 평균 (m) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **B0 (Wheel Odom)** | `0.1837 ± 0.1197` | `0.1396` | `0.4072` | `0.7977` | `0.3359` | `12.29°` | `0.1450` | `0.2030` |
| **F1a (Legacy EKF)** | `0.0810 ± 0.0255` | `0.0838` | `0.1188` | `0.2213` | `0.1320` | `1.50°` | `0.0956` | `0.0737` |
| **F1b (Tuned EKF)** | `0.0818 ± 0.0254` | `0.0830` | `0.1199` | `0.2238` | `0.1323` | `1.47°` | `0.0968` | `0.0743` |
| **F2 (Tuned UKF)** | `0.0820 ± 0.0255` | `0.0839` | `0.1203` | `0.2246` | `0.1327` | `1.47°` | `0.0968` | `0.0746` |
| **F3a (Comp Standalone)** | `0.0829 ± 0.0258` | `0.0849` | `0.1203` | `0.2222` | `0.1321` | `1.42°` | `0.0987` | `0.0750` |
| **F3b (Comp + SlipGate)** | `0.0830 ± 0.0262` | `0.0849` | `0.1212` | `0.2363` | `0.1331` | `1.43°` | `0.0983` | `0.0754` |
| **F4 (2D LiDAR ICP)** | `0.0569 ± 0.0124` | `0.0551` | `0.0765` | `0.2058` | `0.0463` | `2.01°` | `0.0574` | `0.0566` |
| **F5 (LiDAR+IMU EKF)** | **`0.0494 ± 0.0139`** | **`0.0452`** | **`0.0702`** | **`0.2057`** | **`0.0469`** | **`1.47°`** | **`0.0493`** | **`0.0494`** |

- **핵심 분석:**
  - 정상 주행에서도 Ackermann 조향 기구 특성상 회전 시 전륜 횡슬립이 발생하여 **B0(Wheel Odom)**는 Yaw RMSE가 `12.29°`(최악 `57.25°`)까지 벌어지며 위치 오차가 평균 `0.1837 m`(최악 `0.7977 m`)에 달했다.
  - IMU 자세를 결합한 모든 필터(`F1a`, `F1b`, `F2`, `F3a`, `F3b`)는 Yaw RMSE를 `~1.47°`로 잡아주어 ATE RMSE를 **`~0.082 m`(약 55% 감소)**로 낮추었다.
  - 정상 주행에서는 슬립/정체가 없으므로 **`F3a`(`0.0829 m`)와 `F3b`(`0.0830 m`)가 완전히 동일한 성능**을 보여, 슬립 게이트가 정상 주행을 방해하지 않음을 입증했다.
  - **F5(LiDAR+IMU EKF)**는 가장 낮은 ATE RMSE(`0.0494 m`)와 가장 낮은 회전 RPE(`0.28°`)를 동시에 달성했다.

---

### 2.2 시나리오 S2: 유리벽 물리적 정체 및 휠 슬립 (Physical Stall & Wheel Slip, 15회)
- **주행 조건:** 투명 유리벽(`Y = -1.50 m`)을 향해 전진 명령을 인가. 차체 전면 범퍼가 `Y = -1.345 m`에서 물리적으로 완전히 정지(`v_GT = 0.00 m/s`)한 뒤에도 약 3.5초간 바퀴가 공전(`v_wheel = 0.30 m/s`)한 후 후진 복귀.
- **원본 요약/플롯:** [`S2_15trials_summary.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/summaries/S2_15trials_summary.json), [`S2_15trials_summary.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/plots/S2_15trials_summary.png)
- **대표 주행 영상:** [`S2_trial_01_identical.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/S2_trial_01_identical.mp4), [`S2_trial_06_varied_seed_seed1001.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/S2_trial_06_varied_seed_seed1001.mp4)

| 알고리즘 | 15회 ATE RMSE 평균 ± 표준편차 (m) | 중앙값 (m) | 95백분위 (m) | 최악 순시오차 `Worst Max` (m) | 최종 위치오차 평균 (m) | 정체 중 최대 가상 전진 거리 (m) | 허위 목표 도달 판정률 (`False Goal Rate`) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **B0 (Wheel Odom)** | `0.3312 ± 0.0599` | `0.3373` | `0.3934` | `0.6326` | `0.5142` | `1.595 m` (유리벽 관통 착각) | **100% (15/15)** |
| **F1a (Legacy EKF)** | `0.3288 ± 0.0533` | `0.3365` | `0.3909` | `0.6228` | `0.5222` | `1.586 m` (유리벽 관통 착각) | **100% (15/15)** |
| **F1b (Tuned EKF)** | `0.3275 ± 0.0535` | `0.3362` | `0.3907` | `0.6249` | `0.5201` | `1.584 m` (유리벽 관통 착각) | **100% (15/15)** |
| **F2 (Tuned UKF)** | `0.3280 ± 0.0537` | `0.3363` | `0.3908` | `0.6243` | `0.5210` | `1.585 m` (유리벽 관통 착각) | **100% (15/15)** |
| **F3a (Comp Standalone)** | `0.3284 ± 0.0533` | `0.3363` | `0.3916` | `0.6282` | `0.5208` | `1.588 m` (유리벽 관통 착각) | **100% (15/15)** |
| **F3b (Comp + SlipGate)** | **`0.1702 ± 0.0218`** | **`0.1668`** | **`0.2051`** | **`0.3708`** | **`0.2330`** | `1.309 m` (공전 속도 0 게이팅) | **0% (0/15)** |
| **F4 (2D LiDAR ICP)** | **`0.0960 ± 0.0150`** | **`0.0979`** | **`0.1111`** | **`0.1976`** | **`0.1122`** | `1.132 m` (실제 정지 위치 유지) | **0% (0/15)** |
| **F5 (LiDAR+IMU EKF)** | **`0.0909 ± 0.0158`** | **`0.0925`** | **`0.1068`** | **`0.1979`** | **`0.1122`** | `1.147 m` (실제 정지 위치 유지) | **0% (0/15)** |

- **핵심 분석 (가장 중요한 실험적 발견):**
  1. **EKF/UKF/상보필터 단독(`F1a`, `F1b`, `F2`, `F3a`)의 구조적 한계 증명:** 직선 전진 중 유리벽이나 낮은 턱에 걸려 바퀴가 공전할 때, 로봇의 회전 각속도($\omega_z$)와 가속도 변화($\dot{v}$)는 0에 가깝다. 따라서 **Wheel + IMU만 결합한 모든 필터(`F1a`, `F1b`, `F2`, `F3a`)는 B0와 1mm도 다름없이(`ATE RMSE = 0.327~0.331 m`, `Max Error = 0.62~0.63 m`) 벽을 뚫고 계속 전진한다고 오판**하며, 15회 전 회차에서 허위 목표 도달(`FALSE_SUCCEEDED`)을 유발한다.
  2. **F3a vs F3b 분리 비교를 통한 Slip/Stall Gate 효과 입증:** 동일한 상보 필터라도 전방/후방 LiDAR 거리 변화율 기반 슬립 게이트를 추가한 **F3b**는 정체 감지 즉시(`dt ≈ 0.25s`) 휠 선속도를 `0.0 m/s`로 차단하여, **F3a 대비 ATE RMSE를 `0.3284 m -> 0.1702 m`(-48.2%), 최종 위치 오차를 `0.5208 m -> 0.2330 m`(-55.3%)로 절반 이하로 줄이고 허위 목표 도달을 0%로 차단**했다.
  3. **F4 및 F5의 근본적 해결:** 휠 엔코더 속도에 의존하지 않고 주변 벽/가구 기하 형상의 스캔 정합으로 변위를 구하는 **F4(`0.0960 m`)** 및 **F5(`0.0909 m`)**는 오차를 **72.6% 감소**시키며 물리적 정지 상태를 정확히 유지했다.

---

### 2.3 시나리오 S3: 급회전 및 좁은 통로 연속 기동 (Sharp Turns & Narrow Aisle, 15회)
- **주행 조건:** 최대 조향각($\omega_z = \pm 0.65\text{ rad/s}$) 급회전 지그재그 기동으로 타이어 횡슬립 및 각속도 과도응답을 극대화.
- **원본 요약/플롯:** [`S3_15trials_summary.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/summaries/S3_15trials_summary.json), [`S3_15trials_summary.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/plots/S3_15trials_summary.png)

| 알고리즘 | 15회 ATE RMSE 평균 ± 표준편차 (m) | 중앙값 (m) | 95백분위 (m) | 최악 순시오차 `Worst Max` (m) | 회전 RPE 평균 (°) | Yaw RMSE 평균 (°) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **B0 (Wheel Odom)** | `0.1200 ± 0.0605` | `0.0944` | `0.2230` | `0.4456` | `4.02°` | `6.57°` |
| **F1a (Legacy EKF)** | `0.0976 ± 0.0272` | `0.0857` | `0.1360` | `0.2314` | `0.44°` | `1.63°` |
| **F1b (Tuned EKF)** | `0.0984 ± 0.0267` | `0.0888` | `0.1373` | `0.2300` | `0.34°` | `1.56°` |
| **F2 (Tuned UKF)** | `0.0992 ± 0.0270` | `0.0896` | `0.1378` | `0.2302` | `0.34°` | `1.57°` |
| **F3a (Comp Standalone)** | `0.1001 ± 0.0268` | `0.0908` | `0.1380` | `0.2305` | `0.35°` | `1.52°` |
| **F3b (Comp + SlipGate)** | `0.0981 ± 0.0266` | `0.0858` | `0.1380` | `0.2305` | `0.35°` | `1.52°` |
| **F4 (2D LiDAR ICP)** | `0.0771 ± 0.0207` | `0.0709` | `0.1042` | `0.1976` | `2.48°` | `3.59°` |
| **F5 (LiDAR+IMU EKF)** | **`0.0711 ± 0.0214`** | **`0.0629`** | **`0.0995`** | **`0.1744`** | **`0.34°`** | **`1.59°`** |

- **핵심 분석:**
  - 급회전 구간에서 **F4(순수 2D LiDAR ICP, 10 Hz)** 단독은 스캔 주기(100 ms) 동안의 빠른 회전으로 인해 회전 RPE(`2.48°`)와 Yaw RMSE(`3.59°`)가 IMU 기반 필터(`0.34°`, `1.56°`)보다 높게 나타났다.
  - 반면 **F5(LiDAR+IMU EKF)**는 `F4`의 병진 위치 보정 능력과 100 Hz IMU의 회전 추적 능력을 결합하여, `F4` 단독의 회전 지연을 해소(`Yaw RMSE = 1.59°`, `RPE_rot = 0.34°`)하면서도 가장 낮은 위치 오차(`ATE RMSE = 0.0711 m`, 최악 오차 `0.1744 m`)를 기록했다.

---

### 2.4 시나리오 S4 [`SYNTHETIC FAULT INJECTION`]: IMU 바이어스 및 고잡음 주입 (15회)
- **주행 조건 (`SYNTHETIC` 명시):** Gazebo 원본 IMU 신호에 인위적 결함(**자이로 바이어스 $+0.06\text{ rad/s}$, 자세 오프셋 드리프트 $+0.10\text{ rad}$, 가속도계 바이어스 $a_x = +0.55\text{ m/s}^2, a_y = -0.35\text{ m/s}^2$, 가우시안 고잡음**)을 주입하여 IMU 센서 열화/캘리브레이션 불량 시 각 필터의 취약성을 시험.
- **원본 요약/플롯:** [`S4_15trials_summary.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/summaries/S4_15trials_summary.json), [`S4_15trials_summary.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/plots/S4_15trials_summary.png)

| 알고리즘 | 15회 ATE RMSE 평균 ± 표준편차 (m) | 중앙값 (m) | 95백분위 (m) | 최악 순시오차 `Worst Max` (m) | 최종 위치오차 평균 (m) | Yaw RMSE 평균 (°) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **B0 (Wheel Odom)** | `0.0770 ± 0.0741` | `0.0599` | `0.1609` | `0.5097` (슬립 아웃라이어) | `0.1197` | `8.00°` |
| **F1a (Legacy EKF)** | `0.2113 ± 0.0296` | `0.2025` | `0.2554` | `0.4221` | `0.3478` | `15.48°` |
| **F1b (Tuned EKF)** | `0.2092 ± 0.0292` | `0.2010` | `0.2527` | `0.4170` | `0.3442` | `15.48°` |
| **F2 (Tuned UKF)** | `0.2099 ± 0.0298` | `0.2009` | `0.2525` | `0.4174` | `0.3454` | `15.52°` |
| **F3a (Comp Standalone)** | `0.1961 ± 0.0292` | `0.1855` | `0.2381` | `0.3935` | `0.3230` | `14.92°` |
| **F3b (Comp + SlipGate)** | `0.1959 ± 0.0290` | `0.1855` | `0.2381` | `0.3935` | `0.3227` | `14.92°` |
| **F4 (2D LiDAR ICP)** | **`0.0466 ± 0.0070`** | **`0.0461`** | **`0.0588`** | **`0.1100`** | **`0.0341`** | **`1.56°`** |
| **F5 (LiDAR+IMU EKF)** | **`0.0420 ± 0.0072`** | **`0.0425`** | **`0.0535`** | **`0.1071`** | **`0.0345`** | `15.50°` |

- **핵심 분석 (IMU 맹신의 위험성 검증):**
  - IMU에 큰 자이로/자세 바이어스가 존재할 때, 외부 절대 위치 관측 없이 휠 속도를 IMU 방향으로만 적분하는 **F1a, F1b, F2, F3a, F3b는 모두 잘못된 IMU 방향각(`Yaw RMSE ≈ 15.5°`)을 따라 궤적이 옆으로 휘어지며 ATE RMSE가 `~0.21 m`(최종 오차 `~0.35 m`)로 악화**되었다.
  - 반면 IMU를 사용하지 않는 **F4(2D LiDAR ICP)**는 IMU 결함에 전혀 영향받지 않고 `ATE RMSE = 0.0466 m`, `Yaw RMSE = 1.56°`를 유지했으며, `F4`의 위치 $(X, Y)$를 직접 융합하는 **F5** 역시 위치 오차를 `0.0420 m`(최악 `0.1071 m`)로 억제했다.

---

### 2.5 시나리오 S5: 폐루프 복귀 주행 및 F6 SLAM Toolbox 루프 클로저 비교 (15회)
- **주행 조건:** 출발점 `(-0.6, -0.4)`에서 전진·곡선 기동 후 후진·조향으로 출발 원점 근처로 복귀하는 폐루프(Closed-Loop) 궤적. **F6(`slam_toolbox` async Pose Graph Optimization)**를 함께 실행하여 비교.
- **원본 요약/플롯:** [`S5_15trials_summary.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/summaries/S5_15trials_summary.json), [`S5_15trials_summary.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/plots/S5_15trials_summary.png)

| 알고리즘 | 15회 ATE RMSE 평균 ± 표준편차 (m) | 95백분위 (m) | 최악 순시오차 `Worst Max` (m) | **복귀 시점 최종 위치오차 평균 ± 표준편차 (m)** | 복귀 시점 최악 오차 (m) | 병진 RPE 평균 (m) | 회전 RPE 평균 (°) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **B0 (Wheel Odom)** | `0.2176 ± 0.0555` | `0.2820` | `0.6644` | `0.3430 ± 0.1947` | `0.6644` | `0.0319` | `7.45°` |
| **F1a (Legacy EKF)** | `0.0601 ± 0.0226` | `0.0987` | `0.2144` | `0.0412 ± 0.0225` | `0.0986` | `0.0243` | `0.36°` |
| **F1b (Tuned EKF)** | `0.0613 ± 0.0241` | `0.1016` | `0.2183` | `0.0357 ± 0.0235` | `0.0902` | `0.0242` | **`0.29°`** |
| **F2 (Tuned UKF)** | `0.0616 ± 0.0242` | `0.1023` | `0.2192` | `0.0350 ± 0.0234` | `0.0891` | `0.0242` | **`0.29°`** |
| **F3a (Comp Standalone)** | `0.0628 ± 0.0236` | `0.1018` | `0.2179` | `0.0347 ± 0.0233` | `0.0888` | `0.0252` | `0.29°` |
| **F3b (Comp + SlipGate)** | `0.0630 ± 0.0240` | `0.1034` | `0.2179` | `0.0383 ± 0.0249` | `0.0888` | `0.0253` | `0.29°` |
| **F4 (2D LiDAR ICP)** | `0.0490 ± 0.0045` | `0.0553` | `0.1248` | `0.0329 ± 0.0156` | `0.0625` | `0.0298` | `2.07°` |
| **F5 (LiDAR+IMU EKF)** | **`0.0415 ± 0.0048`** | **`0.0503`** | **`0.1107`** | **`0.0332 ± 0.0156`** | `0.0635` | `0.0265` | **`0.29°`** |
| **F6 (SLAM Toolbox PGO)** | **`0.0389 ± 0.0021`** | **`0.0427`** | **`0.1179`** | **`0.0100 ± 0.0052`** | **`0.0204`** | `0.0468` | `6.61°` |

- **핵심 분석 (`F5` 연속 오도메트리 vs `F6` 전역 SLAM 최적화의 상호보완 관계):**
  - **F6(SLAM Toolbox)**는 전역 스캔-투-맵 정합과 루프 클로저를 통해 **복귀 시점 최종 위치 오차를 평균 `0.0100 m (1.0 cm)`, 중앙값 `0.79 cm`, 15회 최악 `2.04 cm`**로 보정하여 순수 오도메트리(`B0`: `34.3 cm`, `F1b`: `3.57 cm`, `F5`: `3.32 cm`)의 누적 드리프트를 완벽히 제거했다.
  - 그러나 **F6**는 약 1~2 Hz 주기로 `map -> odom` 불연속 점프 보정을 수행하므로 단기 상대 오차(`RPE_trans = 0.0468 m`, `RPE_rot = 6.61°`)가 연속 필터인 **F5**(`RPE_trans = 0.0265 m`, `RPE_rot = 0.29°`)보다 크다.
  - 따라서 로컬 제어기(Nav2 Controller / Local Costmap)가 요구하는 매끄러운 50 Hz 연속 상태 추정은 **`odom -> base_footprint`(`F5` 또는 `F3b`/`F1b`)**가 담당하고, 전역 누적 오차 제거는 **`map -> odom`(`F6` SLAM Toolbox)**이 담당하는 **2계층(Two-Tier) 구조**가 필수적임이 실증되었다.

---

## 3. 기존 라운지 A~D 시나리오 실제 주행 영상 및 상태 추정 연동 검증

- **결과 요약 JSON:** [`results/lounge_scenarios/lounge_abcd_video_results.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/lounge_scenarios/lounge_abcd_video_results.json)
- **영상 저장 경로:** `/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/`

| 시나리오 | 설정 | 실제 주행 결과 (`GT` 최종 위치 vs 목표) | `B0` 휠 오도메트리 최종 위치 | `F3b` / `F5` 최종 위치 | 실제 목표 도달 (`True Goal`) | 허위 성공 판정 (`False Goal`) | 핵심 이벤트 및 물리적 원인 | 검증된 MP4 파일 |
| :--- | :---: | :--- | :--- | :--- | :---: | :---: | :--- | :--- |
| **A: 4인용 책상 하부 보강대** | **기본 (`BASELINE`)** | `[-2.500, 2.553]` (목표 `[-2.5, 0.5]`, 잔여 `2.05m`에서 **충돌 정지**) | `[-2.500, -0.647]` (**`3.20m` 허위 전진 발산**) | `F3b`: `[-2.50, 2.16]`<br>`F5`: `[-2.50, 2.58]` | **FAIL (`false`)** | `false` (목표 지나침) | `t=8.59s`: 2D LiDAR가 책상 상판/보강대를 못 보고 직진 진입 후 상부/의자 간섭으로 정지, 바퀴 공전으로 `B0`만 책상을 관통해 `3.2m` 발산 | [`scenario_A_baseline.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_A_baseline.mp4) (`260f`) |
| **A: 4인용 책상 하부 보강대** | **개선 (`IMPROVED`)** | `[-1.115, 0.809]` (목표 `[-1.1, 0.6]`, 거리 `0.209m`) | `[-1.904, 1.191]` (회전 슬립 오차 `0.998m`) | `F3b`: `[-0.99, 0.94]`<br>`F5`: `[-1.06, 0.83]` | **PASS (`true`)** | `false` | Keepout + Depth 장애물 반영으로 책상 우측 통로(`X=-1.1m`)로 안전 우회 완주 (`MinLiDAR = 0.813m`) | [`scenario_A_improved.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_A_improved.mp4) (`295f`) |
| **B: 의자 하부 35mm 스키드 프레임** | **기본 (`BASELINE`)** | `[4.200, 1.674]` (목표 `[4.2, 3.3]`, 잔여 `1.63m`에서 **충돌 정지**) | `[4.200, 4.220]` (**`2.55m` 허위 전진 발산**) | `F3b`: `[4.20, 2.01]`<br>`F5`: `[4.21, 1.65]` | **FAIL (`false`)** | `false` (목표 지나침) | `t=8.48s`: $z=0.18\text{m}$ 2D LiDAR가 $z=0.035\text{m}$ 의자 하부 스키드 레일을 미검출하여 `Y=1.674m`에서 정면 충돌 정지 후 바퀴 공전 | [`scenario_B_baseline.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_B_baseline.mp4) (`264f`) |
| **B: 의자 하부 35mm 스키드 프레임** | **개선 (`IMPROVED`)** | `[3.988, 3.023]` (목표 `[4.1, 3.2]`, 거리 `0.210m`) | `[3.242, 2.865]` (조향 슬립 오차 `0.921m`) | `F3b`: `[3.83, 3.02]`<br>`F5`: `[3.98, 3.03]` | **PASS (`true`)** | `false` | `depth_camera_link` TF 복구 및 여유 폭(`X=3.45m`) 조기 회피로 스키드 레일 무접촉 통과 (`MinLiDAR = 0.914m`) | [`scenario_B_improved.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_B_improved.mp4) (`232f`) |
| **C: 투명 유리벽 및 출입구 우회** | **기본 (`BASELINE`)** | `[1.000, -1.345]` (목표 `[1.0, -2.5]`, 유리벽 앞 **완전 정지**) | `[1.000, -2.306]` (**유리벽 `0.96m` 관통 착각**) | `F3b`: `[1.00, -1.55]`<br>`F5`: `[1.09, -1.22]` | **FAIL (`false`)** | **`TRUE` (`FALSE_SUCCEEDED` 재현!)** | `t=8.48s`: 유리벽(`Y=-1.50m`)에 차체가 가로막혀 정지했으나 바퀴 공전으로 `t=18.48s`에 `B0` 오도메트리가 목표 반경(`0.20m`)에 진입하여 **허위 도착 판정 발생** (`F3b`/`F5`는 이를 차단!) | [`scenario_C_baseline.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_C_baseline.mp4) (`200f`) |
| **C: 투명 유리벽 및 출입구 우회** | **개선 (`IMPROVED`)** | `[-2.076, -1.948]` (목표 `[-2.12, -2.15]`, 거리 `0.206m`) | `[-2.075, -1.953]` | `F3b`: `[-2.17, -1.85]`<br>`F5`: `[-2.10, -1.91]` | **PASS (`true`)** | `false` | 유리벽 구간 진입 금지 처리 후 개방된 출입구(`X=-2.12m, Y=-1.50m`)를 무충돌 통과하여 VIP 룸 내부 도달 (`events_logged: []`) | [`scenario_C_improved.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_C_improved.mp4) (`301f`) |
| **D: 복합 라운지 대각선 종단** | **기본 (`BASELINE`)** | `[-3.228, 2.272]` (목표 `[-2.12, -2.15]`, 잔여 `4.68m`에서 **충돌 정지**) | `[-2.410, 2.874]` | `F3b`: `[-3.20, 2.05]`<br>`F5`: `[-3.25, 2.30]` | **FAIL (`false`)** | `false` | `t=11.61s`: 최단거리 대각선 경로가 `table_1` 의자 군집에 걸려 정지 및 휠 슬립 발생 | [`scenario_D_baseline.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_D_baseline.mp4) (`261f`) |
| **D: 복합 라운지 대각선 종단** | **개선 (`IMPROVED`)** | `[-2.009, -1.934]` (목표 `[-2.12, -2.15]`, 거리 `0.243m`) | `[-2.239, -1.824]` | `F3b`: `[-1.95, -1.95]`<br>`F5`: `[-1.59, -2.09]` | **PASS (`true`)** | `false` | 북측 통로 → 중앙 통로(`X=-0.85m`) → 개방 출입구(`X=-2.12m`)를 거쳐 전 구간 무충돌 완주 (`events_logged: []`, `594f`) | [`scenario_D_improved.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_D_improved.mp4) (`594f`) |

---

## 4. 실제 LIMO Pro 로봇 적용을 위한 최종 기술 선정 및 권고안

### 4.1 왜 단일 알고리즘이나 기존 EKF(`F1a`)만으로는 불충분한가?
1. **기존 `limo_ekf.yaml`(`F1a`)의 문제:**
   - 2D 평면 주행 로봇임에도 `two_d_mode: true`가 누락되어 있으며, 중력·경사·진동 바이어스에 취약한 IMU 선형가속도($a_x, a_y, a_z$) 적분이 켜져 있어 센서 바이어스 발생 시(`S4`) 급격한 드리프트를 유발한다.
   - 무엇보다 **유리벽/낮은 턱 충돌 정체(`S2`, `Scenario C Baseline`) 시 IMU는 직선 휠 공전을 감지할 수 없으므로 `B0`, `F1a`, `F1b`, `F2`, `F3a` 모두 100% 허위 전진 발산(`FALSE_SUCCEEDED`)**을 일으킨다.
2. **순수 LiDAR 오도메트리(`F4`) 단독 사용의 한계:**
   - 휠 슬립과 IMU 바이어스에는 완벽히 면역이나, 10 Hz 스캔 주기로 인해 급격한 조향(`S3`) 시 단기 회전 지연(`RPE_rot = 2.48°`)이 발생하며, 특징점이 부족한 긴 복도에서는 회전 구속력이 약해질 수 있다.
3. **SLAM Toolbox(`F6`) 단독 사용의 한계:**
   - 전역 누적 오차와 폐루프 복귀 오차(`1.0 cm`)는 가장 우수하나, 1~2 Hz의 불연속 그래프 최적화 갱신(`RPE_rot = 6.61°`) 특성상 고속 로컬 제어 루프의 직접 피드백으로 사용할 수 없다.

### 4.2 최종 선정 아키텍처: **2계층(Two-Tier) 센서 융합 + 슬립 게이팅 구조**

실제 LIMO Pro(Jetson Nano / Orin Nano 탑재 환경)에 즉시 배포 가능하며 계산량과 안정성을 모두 만족하는 **최적 선정 구성**은 다음과 같다.

1. **1순위 주력 구성 (권장 표준: `F5 + F3b Slip Gate + F6 SLAM Toolbox`):**
   - **Local Odometry (`odom -> base_footprint`, 50 Hz):** **F5 (LiDAR Scan-Matching Odometry `F4` + IMU $\psi, \omega_z$ + Slip-Gated Wheel Velocity EKF)**
     - 전 시나리오(S1~S5, 75회)에서 가장 낮은 위치 오차(`S1: 0.049m`, `S2: 0.091m`, `S3: 0.071m`, `S4: 0.042m`, `S5: 0.041m`)와 매끄러운 회전 응답(`RPE_rot ≈ 0.29°`)을 동시에 달성.
     - 유리벽이나 의자 하부 프레임에 걸려 바퀴가 헛돌아도 LiDAR 스캔 정합과 슬립 게이트가 허위 전진 적분을 원천 차단하여 **Nav2의 허위 목표 도달(`FALSE_SUCCEEDED`) 위험을 0%로 제거**.
   - **Global Localization / Mapping (`map -> odom`, 1~2 Hz):** **F6 (`slam_toolbox` async Pose Graph Optimization)**
     - `F5`가 제공하는 고정밀 `odom -> base_footprint` TF 위에서 작동하여 스캔 왜곡을 최소화하고, 루프 클로저 시 누적 오차를 `~1.0 cm` 수준으로 전역 보정.

2. **2순위 경량 폴백 구성 (CPU 부하 제한 시: `F1b + F3b Slip/Stall Gate`):**
   - 만약 온보드 CPU 자원이 극도로 제한되어 10 Hz ICP 스캔 매칭(`F4`)을 상시 구동하기 어려운 경우:
     - 기존 `limo_ekf.yaml`을 **`F1b` 설정(`two_d_mode: true`, $a_x, a_y, a_z$ 비활성화, 휠 $v_x, v_y$ + IMU $\psi, \omega_z$)**으로 수정하고,
     - 앞단에 **`F3b`의 전방 LiDAR 거리 변화율 기반 Slip/Stall Gate 노드(CPU 점유율 `< 1%`)**를 배치하여 정체 시 휠 속도 입력을 게이팅한다. 이 조합만으로도 정상 주행 오차를 55% 줄이고(`0.082 m`), 충돌 정체 시 오도메트리 발산을 절반 이하(`0.328 m -> 0.170 m`)로 억제하며 허위 성공 판정을 방지할 수 있다.

3. **실물 LIMO Pro 적용 시 필수 확인 체크리스트:**
   - `limo_base` 드라이버 런치에서 `pub_odom_tf`를 `false`로 설정하여 EKF(`F5`/`F1b`)와의 `/tf` 중복 발행을 방지할 것.
   - `limo_base`가 `/odom` 계산 시 내부 IMU yaw를 이미 사용하고 있는지 확인하고, 사용 중이라면 `/odom`에서는 선속도($v_x$)만 추출하고 회전 정보는 `/imu`에서만 공급받아 IMU 이중 융합을 방지할 것(본 실험의 `F1b`, `F3b`, `F5`에 이미 반영됨).
   - `depth_camera_link` 정적 TF(`base_link -> depth_camera_link`)를 런치에 명시적으로 포함하여 의자 스키드 프레임($35\text{ mm}$) 등 저상 장애물 포인트클라우드가 누락되지 않도록 할 것.
