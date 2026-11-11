# LIMO Pro 주행 영상(MP4) 및 75회 반복 실험 로그 카탈로그

본 문서는 `/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/` 내에 생성·검증된 **전체 MP4 주행 영상(총 23개)** 및 **상태 추정 75회 반복 실험 원본 로그·요약 파일**의 전체 목록과 재생 검증 결과를 정리한 카탈로그이다.

---

## 1. 라운지 장애물 시나리오 A~D 주행 영상 (기본 설정 vs 개선 설정 + 좌우 비교 영상, 총 12개 MP4)

- **디렉터리:** `/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/`
- **메타데이터 원본:** [`results/lounge_scenarios/lounge_abcd_video_results.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/lounge_scenarios/lounge_abcd_video_results.json)
- **화면 구성 (1920×960, 15 FPS, H.264 MP4):**
  - 좌측(`1280×960`): Gazebo Fortress 실시간 상단 카메라(`overhead_camera`) 화면 위에 **Ground Truth(초록색)**, **B0 Wheel Odom(빨간색)**, **F1b Tuned EKF(하늘색)**, **F3b Slip-Gated Comp(노란색)**, **F5 LiDAR+IMU EKF(자홍색)**의 실시간 주행 궤적을 투영하고, 물리적 정체 및 슬립 감지 시 상단에 붉은색 `WHEEL SLIP / STALL GATED!` 배너를 표시.
  - 우측 상단(`640×460`): 오도메트리 좌표계 실시간 궤적 비교 플롯 (`0.5 m` 격자).
  - 우측 하단(`640×500`): 전체 추정기(`B0, F1a, F1b, F2, F3a, F3b, F4, F5`)의 실시간 $(X, Y, \text{Yaw})$, 순시 위치 오차(`PosErr`), 누적 `RMSE`, `YawErr`, 실제 속도(`v_GT`) 대 휠 속도(`v_Wheel`) 대 슬립 게이팅 속도(`v_SlipGated`) 수치 패널.

### 1.1 개별 주행 영상 (8개 MP4)

| 시나리오 | 설정 | MP4 파일 경로 | 해상도 / FPS | 검증된 프레임 수 | 시뮬레이션 시간 구간 | 파일 크기 | 주요 타임스탬프 및 관측 내용 |
| :--- | :---: | :--- | :---: | :---: | :---: | :---: | :--- |
| **Scenario A**<br>(4인용 책상 하부 보강대) | **기본 (`BASELINE`)** | [`scenario_A_baseline.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_A_baseline.mp4) | `1920x960` / 15.0 | `260 / 260` (`playable: true`) | `6.69s ~ 24.42s` (`17.73s`) | `3.81 MB` | `SimTime 8.59s`: 책상/의자 하부 진입 중 물리적 접촉 정지. `v_GT=0.00m/s`이나 바퀴 공전(`v_Wheel=0.26m/s`)으로 `B0..F3a` 주황/적색 궤적이 책상을 뚫고 `3.2m` 아래로 발산하는 반면 `F3b`는 게이팅되고 `F4`/`F5`는 실제 정지 위치에 고정됨. |
| **Scenario A**<br>(4인용 책상 하부 보강대) | **개선 (`IMPROVED`)** | [`scenario_A_improved.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_A_improved.mp4) | `1920x960` / 15.0 | `295 / 295` (`playable: true`) | `6.60s ~ 26.28s` (`19.68s`) | `6.33 MB` | 책상 하부 진입 없이 우측 복도(`X=-1.1m`)로 안전 우회하여 목표(`[-1.1, 0.6]`) 도달 (`MinLiDAR = 0.813m`). 우회 회전 시 `B0`는 조향 슬립으로 궤적이 꺾이나 `F1b~F5`는 `GT`와 일치. |
| **Scenario B**<br>(의자 35mm 스키드 프레임) | **기본 (`BASELINE`)** | [`scenario_B_baseline.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_B_baseline.mp4) | `1920x960` / 15.0 | `264 / 264` (`playable: true`) | `6.61s ~ 24.17s` (`17.56s`) | `3.98 MB` | `SimTime 8.48s`: 2D LiDAR($z=0.18\text{m}$)가 바닥의 $35\text{mm}$ 의자 스키드 프레임을 보지 못해 `Y=1.674m`에서 충돌 정지. `B0..F3a`는 의자를 관통해 `Y=4.22m`까지 허위 진행하나 `F3b`/`F4`/`F5`는 정지 상태를 포착. |
| **Scenario B**<br>(의자 35mm 스키드 프레임) | **개선 (`IMPROVED`)** | [`scenario_B_improved.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_B_improved.mp4) | `1920x960` / 15.0 | `232 / 232` (`playable: true`) | `6.57s ~ 21.99s` (`15.42s`) | `5.37 MB` | `depth_camera_link` 정적 TF 복구 및 여유 폭(`X=3.45m`) 조기 우회 경로로 의자 스키드 레일을 안전하게 회피하여 목표(`[4.1, 3.2]`) 도달 (`MinLiDAR = 0.914m`). |
| **Scenario C**<br>(투명 유리벽 및 출입구) | **기본 (`BASELINE`)** | [`scenario_C_baseline.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_C_baseline.mp4) | `1920x960` / 15.0 | `200 / 200` (`playable: true`) | `6.67s ~ 19.93s` (`13.26s`) | `3.25 MB` | `SimTime 8.48s`: 유리벽(`Y=-1.50m`) 앞에서 차체가 `Y=-1.345m`에 정지. 그러나 바퀴 공전으로 `B0` 오도메트리가 유리벽 너머로 계속 전진하여 `SimTime 18.48s`에 목표(`Y=-2.50m`) 반경 내 진입(`Y=-2.304m`), **허위 목표 도달(`FALSE_SUCCEEDED`)이 발생하는 장면과 `F3b`/`F5`가 이를 방어하는 장면을 동시에 시각화**. |
| **Scenario C**<br>(투명 유리벽 및 출입구) | **개선 (`IMPROVED`)** | [`scenario_C_improved.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_C_improved.mp4) | `1920x960` / 15.0 | `301 / 301` (`playable: true`) | `6.68s ~ 26.66s` (`19.98s`) | `7.27 MB` | 유리벽 구간을 우회하여 좌측 개방 출입구(`X=-2.12m, Y=-1.50m`)를 무충돌 통과, VIP 룸 내부 목표(`[-2.12, -2.15]`)에 정확히 도달 (`events_logged: []`). |
| **Scenario D**<br>(복합 라운지 대각선 종단) | **기본 (`BASELINE`)** | [`scenario_D_baseline.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_D_baseline.mp4) | `1920x960` / 15.0 | `261 / 261` (`playable: true`) | `6.60s ~ 24.05s` (`17.45s`) | `4.92 MB` | `SimTime 11.61s`: 가구 군집을 가로지르는 대각선 단축 경로 진입 중 `table_1` 의자에 걸려 정지 및 휠 슬립 발산 발생. |
| **Scenario D**<br>(복합 라운지 대각선 종단) | **개선 (`IMPROVED`)** | [`scenario_D_improved.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_D_improved.mp4) | `1920x960` / 15.0 | `594 / 594` (`playable: true`) | `6.66s ~ 46.33s` (`39.67s`) | `13.84 MB` | 북측 통로 → 중앙 복도(`X=-0.85m`) → 개방 출입구(`X=-2.12m`)를 거쳐 VIP 룸 내부(`[-2.009, -1.934]`)까지 전 구간 무충돌 완주 (`events_logged: []`). |

### 1.2 기본 설정(좌) vs 개선 설정(우) 동시 비교 영상 (4개 MP4)

| 비교 시나리오 | MP4 파일 경로 | 해상도 / FPS | 검증된 프레임 수 | 파일 크기 |
| :--- | :--- | :---: | :---: | :---: |
| **Scenario A 좌우 비교** (Baseline vs Improved) | [`scenario_A_comparison.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_A_comparison.mp4) | `1920x960` / 15.0 | `295` | `6.40 MB` |
| **Scenario B 좌우 비교** (Baseline vs Improved) | [`scenario_B_comparison.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_B_comparison.mp4) | `1920x960` / 15.0 | `264` | `5.71 MB` |
| **Scenario C 좌우 비교** (Baseline vs Improved) | [`scenario_C_comparison.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_C_comparison.mp4) | `1920x960` / 15.0 | `301` | `6.58 MB` |
| **Scenario D 좌우 비교** (Baseline vs Improved) | [`scenario_D_comparison.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/lounge_scenarios/scenario_D_comparison.mp4) | `1920x960` / 15.0 | `594` | `12.93 MB` |

---

## 2. 상태 추정 시나리오 S1~S5 대표 주행 영상 (총 11개 MP4)

- **디렉터리:** `/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/` 및 `videos/smoke_test/`

| 시나리오 | 구분 | MP4 파일 경로 | 해상도 / FPS | 재생 검증 |
| :--- | :---: | :--- | :---: | :---: |
| **Smoke Test (S1)** | 기반 파이프라인 검증 | [`videos/smoke_test/s1_normal_multiturn_smoke_test.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/smoke_test/s1_normal_multiturn_smoke_test.mp4) | `1920x960` / 15.0 | `222 frames` (`playable: true`) |
| **S1 (정상 다중 회전)** | 동일 조건 대표 (`Trial #01`) | [`videos/state_estimation/S1_trial_01_identical.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/S1_trial_01_identical.mp4) | `1920x960` / 15.0 | `playable: true` |
| **S1 (정상 다중 회전)** | 시드 변동 대표 (`Trial #06`) | [`videos/state_estimation/S1_trial_06_varied_seed_seed1001.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/S1_trial_06_varied_seed_seed1001.mp4) | `1920x960` / 15.0 | `playable: true` |
| **S2 (유리벽 정체/슬립)** | 동일 조건 대표 (`Trial #01`) | [`videos/state_estimation/S2_trial_01_identical.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/S2_trial_01_identical.mp4) | `1920x960` / 15.0 | `playable: true` |
| **S2 (유리벽 정체/슬립)** | 시드 변동 대표 (`Trial #06`) | [`videos/state_estimation/S2_trial_06_varied_seed_seed1001.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/S2_trial_06_varied_seed_seed1001.mp4) | `1920x960` / 15.0 | `playable: true` |
| **S3 (급회전/좁은 통로)** | 동일 조건 대표 (`Trial #01`) | [`videos/state_estimation/S3_trial_01_identical.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/S3_trial_01_identical.mp4) | `1920x960` / 15.0 | `playable: true` |
| **S3 (급회전/좁은 통로)** | 시드 변동 대표 (`Trial #06`) | [`videos/state_estimation/S3_trial_06_varied_seed_seed1001.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/S3_trial_06_varied_seed_seed1001.mp4) | `1920x960` / 15.0 | `playable: true` |
| **S4 (IMU 바이어스 주입)** | 동일 조건 대표 (`Trial #01`) | [`videos/state_estimation/S4_trial_01_identical.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/S4_trial_01_identical.mp4) | `1920x960` / 15.0 | `playable: true` |
| **S4 (IMU 바이어스 주입)** | 시드 변동 대표 (`Trial #06`) | [`videos/state_estimation/S4_trial_06_varied_seed_seed1001.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/S4_trial_06_varied_seed_seed1001.mp4) | `1920x960` / 15.0 | `playable: true` |
| **S5 (폐루프 복귀/SLAM)** | 동일 조건 대표 (`Trial #01`) | [`videos/state_estimation/S5_trial_01_identical.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/S5_trial_01_identical.mp4) | `1920x960` / 15.0 | `playable: true` |
| **S5 (폐루프 복귀/SLAM)** | 시드 변동 대표 (`Trial #06`) | [`videos/state_estimation/S5_trial_06_varied_seed_seed1001.mp4`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos/state_estimation/S5_trial_06_varied_seed_seed1001.mp4) | `1920x960` / 15.0 | `playable: true` |

---

## 3. 75회 반복 실험 원본 시계열 CSV 및 통계 요약 JSON 목록

1. **통합 및 시나리오별 요약 JSON (`results/summaries/`):**
   - 전 시나리오 통합: [`results/summaries/master_all_scenarios_summary.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/summaries/master_all_scenarios_summary.json)
   - S1 (15회): [`results/summaries/S1_15trials_summary.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/summaries/S1_15trials_summary.json)
   - S2 (15회): [`results/summaries/S2_15trials_summary.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/summaries/S2_15trials_summary.json)
   - S3 (15회): [`results/summaries/S3_15trials_summary.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/summaries/S3_15trials_summary.json)
   - S4 (15회): [`results/summaries/S4_15trials_summary.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/summaries/S4_15trials_summary.json)
   - S5 (15회): [`results/summaries/S5_15trials_summary.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/summaries/S5_15trials_summary.json)

2. **시각화 플롯 PNG (`results/plots/`):**
   - 통합 4-패널 비교 플롯: [`results/plots/master_cross_scenario_comparison.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/plots/master_cross_scenario_comparison.png)
   - 시나리오별 분포 및 궤적 플롯:
     - [`results/plots/S1_15trials_summary.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/plots/S1_15trials_summary.png)
     - [`results/plots/S2_15trials_summary.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/plots/S2_15trials_summary.png)
     - [`results/plots/S3_15trials_summary.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/plots/S3_15trials_summary.png)
     - [`results/plots/S4_15trials_summary.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/plots/S4_15trials_summary.png)
     - [`results/plots/S5_15trials_summary.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/results/plots/S5_15trials_summary.png)

3. **개별 회차 원본 시계열 CSV (`results/raw_trials/`, 총 75개 파일):**
   - `S1_trial_01_identical.csv` ~ `S1_trial_15_varied_seed_seed1010.csv` (15개)
   - `S2_trial_01_identical.csv` ~ `S2_trial_15_varied_seed_seed1010.csv` (15개)
   - `S3_trial_01_identical.csv` ~ `S3_trial_15_varied_seed_seed1010.csv` (15개)
   - `S4_trial_01_identical.csv` ~ `S4_trial_15_varied_seed_seed1010.csv` (15개)
   - `S5_trial_01_identical.csv` ~ `S5_trial_15_varied_seed_seed1010.csv` (15개)
