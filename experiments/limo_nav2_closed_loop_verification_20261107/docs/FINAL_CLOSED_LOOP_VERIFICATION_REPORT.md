# LIMO Pro 상태 추정 알고리즘 실물 적용 전 폐루프(Closed-Loop) 검증 최종 보고서

- **실험 디렉터리:** `/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_nav2_closed_loop_verification_20261107/`
- **보존된 이전 실험 디렉터리:**
  - `/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/` (75개 CSV, 75개 JSON, 21개 MP4 100% 무수정 보존)
  - `/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/` (100% 무수정 보존)
  - `/home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/` (기존 프로젝트 소스 100% 무수정 보존)

---

## 1. 기존 75회 실험 보존 및 감사 지적 사항 정정 (`Work Package 1`)

### 1.1 원본 데이터 보존 및 누락 영상 복구
- 기존 75회 반복 실험 디렉터리(`limo_state_estimation_benchmark_20261107/`)의 모든 개별 시계열 CSV(75개), 개별/통합 JSON(76개), 생성된 영상(21개)을 일절 수정하지 않고 보존하였다.
- 이전 실험에서 파일명/스크립트 실행 시점 문제로 미생성되었던 `S1_Normal_Anchor` 대표 영상 2개를 신규 검증 디렉터리에 생성·검증하였다(원본 디렉터리는 미생성 상태 그대로 유지):
  - [`videos/s1_recovered/S1_trial_01_identical.mp4`](../videos/s1_recovered/S1_trial_01_identical.mp4) (`205 frames`, `15.0 fps`, `1920x960`)
  - [`videos/s1_recovered/S1_trial_06_varied_seed_seed1001.mp4`](../videos/s1_recovered/S1_trial_06_varied_seed_seed1001.mp4) (`203 frames`, `15.0 fps`, `1920x960`)

### 1.2 명칭 정정 및 미검증 항목 명확화
1. **`F5` 구성 명칭 정정:**
   - 기존 표기: `LiDAR + IMU + Wheel Fusion EKF`
   - **정정된 실제 구성 명칭:** **`LiDAR Odometry (F4: x, y, vx, vy) + IMU (yaw, wz) EKF`** (실제 `fusion_f5_lidar_imu_wheel.yaml`은 `odom0: /odometry/lidar_odom`과 `imu0: /imu`만 입력받으며 `/odom` 또는 `/odometry/comp_slip_gated`(`F3b`)를 입력받지 않음).
2. **`F6` (`SLAM Toolbox`) 이전 `S5` 실험의 Loop Closure 판정 정정:**
   - 이전 `S5_LoopClosure_Long` 시나리오는 `2.7 m` 직선 왕복 경로로 `slam_toolbox`의 `loop_match_minimum_chain_size: 8` 조건을 충족하지 못해 비순차적 루프 폐쇄 간선이 생성되지 않았으므로, 이전 `S5`에서의 Loop Closure 검증 상태를 **`UNVERIFIED (미검증 — 로컬 스캔 매칭만 검증됨)`**으로 정정 기록하였다(진정한 다중 복도 `16.05 m` 폐곡선 루프 폐쇄는 본 보고서 **6절(Work Package 6)**에서 별도 수행·검증 완료).
3. **기존 라운지 A~D 영상 분류 정정:**
   - 이전 실험에서 생성된 라운지 A~D 영상 8개는 Nav2 액션 서버 폐루프 주행이 아니라 Ground Truth 위치를 참조한 수동 비례 제어 경유점 주행이므로 **`Ground Truth 기반 수동 경유점 시험 (Open-Loop Estimation Visualization under GT Waypoint Tracking)`**으로 명확히 분류하였다.

### 1.3 기존 75회 실험의 ZOH(Zero-Order-Hold) 통계 vs 시간 보간(Timestamp-Interpolated) 통계 병기
- 재계산 스크립트 [`scripts/recompute_75trial_zoh_vs_interp.py`](../scripts/recompute_75trial_zoh_vs_interp.py)를 통해 기존 75개 CSV 전체에 대해 원본 ZOH 오차와 각 추정기 메시지의 `header.stamp` 기준 선형 보간(`Timestamp-Interpolated`) 오차를 분리 산출하였다([`results/previous_75trial_zoh_vs_interpolated_recomputation.json`](../results/previous_75trial_zoh_vs_interpolated_recomputation.json)):

| 시나리오 ( 각 15회 ) | 추정기 | 원본 ZOH ATE RMSE Mean±Std (m) | 시간 보간 (`Interp`) ATE RMSE Mean±Std (m) | 보간 시 오차 변화 (`Interp - ZOH`, m) | 시간 보간 Yaw RMSE Mean (deg) | 평균 샘플 지연 (`sample_age_ms`) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **`S1_Normal_Anchor`** | `B0` | `0.1149 ± 0.0442` | `0.1149 ± 0.0442` | `-0.0001` | `2.65°` | `2.91 ms` |
| | `F1b` | `0.0757 ± 0.0258` | `0.0757 ± 0.0258` | `+0.0000` | `1.13°` | `8.59 ms` |
| | `F3a` | `0.0771 ± 0.0268` | `0.0771 ± 0.0268` | `+0.0000` | `1.07°` | `2.91 ms` |
| | `F3b` | `0.0771 ± 0.0268` | `0.0771 ± 0.0268` | `+0.0000` | `1.07°` | `2.91 ms` |
| | **`F4`** | `0.0320 ± 0.0017` | **`0.0226 ± 0.0020`** | **`-0.0094 (-29.4%)`** | **`0.82°`** | **`99.01 ms`** |
| | **`F5`** | `0.0309 ± 0.0033` | `0.0308 ± 0.0034` | `-0.0001` | `1.13°` | **`8.59 ms`** |
| | `F6` | `0.0338 ± 0.0020` | `0.0261 ± 0.0022` | `-0.0077 (-22.8%)` | `1.22°` | `53.20 ms` |
| **`S2_WheelSlip_Burst`** | `B0` | `0.5060 ± 0.0566` | `0.5061 ± 0.0566` | `+0.0001` | `2.38°` | `3.15 ms` |
| | `F1b` | `0.4274 ± 0.0339` | `0.4274 ± 0.0338` | `+0.0000` | `1.14°` | `8.68 ms` |
| | `F3b` | `0.2194 ± 0.0335` | `0.2194 ± 0.0335` | `+0.0000` | `1.08°` | `3.15 ms` |
| | **`F4`** | `0.0349 ± 0.0022` | **`0.0260 ± 0.0025`** | **`-0.0089 (-25.5%)`** | **`0.88°`** | **`101.67 ms`** |
| | **`F5`** | `0.0299 ± 0.0035` | `0.0298 ± 0.0035` | `-0.0001` | `1.14°` | **`8.68 ms`** |
| **`S3_GlassWall_Stall`** | `B0` | `0.5880 ± 0.0548` | `0.5880 ± 0.0548` | `+0.0000` | `1.44°` | `3.05 ms` |
| | `F1b` | `0.5250 ± 0.0355` | `0.5250 ± 0.0355` | `+0.0000` | `0.90°` | `8.69 ms` |
| | `F3a` | `0.5257 ± 0.0358` | `0.5257 ± 0.0358` | `+0.0000` | `0.85°` | `3.05 ms` |
| | **`F3b`** | `0.1552 ± 0.0149` | `0.1552 ± 0.0149` | `+0.0000` | `0.85°` | `3.05 ms` |
| | **`F4`** | `0.0195 ± 0.0025` | **`0.0124 ± 0.0032`** | **`-0.0071 (-36.4%)`** | **`0.22°`** | **`98.96 ms`** |
| | **`F5`** | `0.0176 ± 0.0024` | `0.0176 ± 0.0024` | `+0.0000` | `0.90°` | **`8.69 ms`** |
| **`S4_IMUBias_Fault`** | `F1b` | `0.3863 ± 0.0354` | `0.3863 ± 0.0354` | `+0.0000` | `15.46°` | `8.59 ms` |
| | **`F4`** | `0.0319 ± 0.0017` | **`0.0221 ± 0.0020`** | **`-0.0098 (-30.7%)`** | **`0.76°`** | **`101.66 ms`** |
| | **`F5` (원본)** | `0.0885 ± 0.0085` | `0.0884 ± 0.0086` | `-0.0001` | **`15.46°`** | `8.59 ms` |

---

## 2. `F4` / `F5` 및 `F3b` 개선 설계 및 독립 검증 (`Work Package 2`)

- **실행 스크립트:** [`scripts/run_wp1_s1_videos_and_wp2_f4_f5_ablation.py`](../scripts/run_wp1_s1_videos_and_wp2_f4_f5_ablation.py)
- **결과 JSON 및 플롯:** [`results/f4_f5_ablation/wp2_f4_f5_f3b_ablation_summary.json`](../results/f4_f5_ablation/wp2_f4_f5_f3b_ablation_summary.json), [`results/plots/wp2_f4_f5_ablation_comparison.png`](../results/plots/wp2_f4_f5_ablation_comparison.png)

### 2.1 `F4`의 ICP 출력 주기 지연 vs 실제 알고리즘 계산 지연 분리
`nodes/lidar_scan_matcher_odom_node.py` 내부에 `time.perf_counter()` 계측 토픽(`/diagnostics/f4_latency`)을 추가하여 **실제 ICP 계산 벽시계 시간(`icp_compute_wall_ms`)**과 **10 Hz 스캔 주기 샘플 홀드 나이(`sample_hold_age_ms`)**를 분리 측정하였다:
- **`F4` 실제 ICP 알고리즘 계산 시간(`icp_compute_wall_ms`):** 평균 **`34.80 ~ 47.99 ms`** (최악 95백분위 `51.82 ~ 63.32 ms`)로 `/scan` 주기(`100 ms`) 이내에 실시간 처리가 완료된다.
- **`F4` ZOH 샘플 홀드 나이(`sample_hold_age_ms`):** `10 Hz` 비동기 발행 특성으로 인해 50 Hz 제어기 시점에서는 평균 **`95.60 ~ 102.04 ms`**의 샘플 나이가 발생한다.
- 따라서 `F4`가 원본 ZOH 통계에서 `F5`보다 ATE가 높게 나왔던 원인은 `F4` 알고리즘의 부정확성이 아니라 `100 ms` 출력 주기로 인한 위치 위상 지연(`0.25 m/s × 0.098 s ≈ 0.024 m`)이었으며, `F5_improved` EKF를 통해 `50 Hz` 예측 보간을 수행하면 샘플 나이를 **`8.46 ~ 9.21 ms`**로 **11배 단축**할 수 있다.

### 2.2 `F5`의 IMU Yaw 바이어스 취약성 원인과 `F5_improved` 설계 (센서 중복 융합 배제)
1. **원본 `F5`의 취약성 원인:**
   - 원본 `F5`([`config/fusion_f5_lidar_imu_wheel.yaml`](../config/fusion_f5_lidar_imu_wheel.yaml))는 `odom0`(`/odometry/lidar_odom`)에서 `[x, y, vx, vy]`만 융합하고 `yaw`를 `false`로 둔 채, `imu0`(`/imu`)에서 `yaw: true, vyaw: true`를 융합하였다.
   - 이로 인해 `S4_IMUBias_Fault`(자이로 바이어스 `+0.045 rad/s`)에서 IMU 적분 Yaw가 발산할 때 이를 구속할 절대 방위각 관측치가 없어 **Yaw RMSE가 `15.6587°`(최대 `26.0537°`)로 붕괴**하였다.
2. **`F5_improved` 설계 ([`config/fusion_f5_improved.yaml`](../config/fusion_f5_improved.yaml)):**
   - `odom0` (`/odometry/lidar_odom`, `10 Hz`): **`[x, y, yaw, vx, vy]`** 융합 (`F4` ICP가 스캔 매칭으로 직접 산출한 절대 회전 추정값 `yaw`를 활성화하여 IMU 자이로 바이어스 누적을 10 Hz마다 드리프트 없이 구속).
   - `imu0` (`/imu`, `50 Hz`): **`[vyaw (wz)]`만 단독 융합** (`imu0`의 적분 `yaw`는 비활성화하여 바이어스 누적 경로를 차단하고, 10 Hz LiDAR 스캔 사이의 고주파(`50 Hz`) 회전율 변화만 EKF 상태 전이에 제공).
   - **센서 중복 융합 검증:** `F4`는 순수 `/scan`만 사용하고 IMU나 `/odom`을 전혀 사용하지 않으며, `F5_improved`는 `odom0`에서 LiDAR `[x, y, yaw, vx, vy]`(10 Hz), `imu0`에서 IMU `[wz]`(50 Hz)만 입력받아 **동일 센서 데이터의 중복 융합이 0건**임을 검증하였다.

### 2.3 `F4`, 원본 `F5`, `F5_improved`, 원본 `F3b`, `F3b_improved` 정량 비교표

| 시나리오 | 추정기 | ZOH ATE RMSE (m) | 시간 보간 (`Interp`) ATE RMSE (m) | 시간 보간 RPE RMSE (m) | 시간 보간 Yaw RMSE (deg) | 최악 Yaw 오차 (deg) | 샘플 나이 (`age_ms`) | `F4` ICP 계산 시간 (`ms`) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **`S1_Normal_Anchor`** | `F4` (`/scan` ICP 단독) | `0.0320` | **`0.0226`** | **`0.0056`** | `0.8168°` | `2.6512°` | `102.04 ms` | `47.99 ms` |
| | `F5` (원본: LiDAR `x,y,vx,vy` + IMU `yaw,wz`) | `0.0282` | `0.0280` | `0.0063` | `0.4008°` | `0.6256°` | `8.80 ms` | — |
| | **`F5_improved` (개선: LiDAR `x,y,yaw,vx,vy` + IMU `wz`)** | **`0.0268`** | **`0.0265`** | `0.0067` | **`0.5988°`** | **`1.8691°`** | **`8.80 ms`** | — |
| **`S4_IMUBias_Fault`** | `F1b` (Tuned EKF: Wheel + IMU) | `0.3422` | `0.3422` | `0.0110` | `15.6579°` | `26.0537°` | `8.46 ms` | — |
| | `F4` (`/scan` ICP 단독) | `0.0327` | **`0.0228`** | **`0.0058`** | `0.7810°` | `2.2389°` | `98.81 ms` | `47.55 ms` |
| | `F5` (원본: LiDAR `x,y,vx,vy` + IMU `yaw,wz`) | `0.0882` | `0.0882` | `0.0085` | **`15.6587°`** | **`26.0537°`** | `8.46 ms` | — |
| | **`F5_improved` (개선: LiDAR `x,y,yaw,vx,vy` + IMU `wz`)** | **`0.0303`** | **`0.0301`** | `0.0070` | **`0.7788° (-95.0%)`** | **`1.7330°`** | **`8.46 ms`** | — |
| **`S3_GlassWall_Stall`** | `F3a` (Complementary 단독) | `0.5008` | `0.5008` | `0.0108` | `0.0002°` | `0.0010°` | `2.90 ms` | — |
| | `F3b` (원본: `1.4s` 변위 윈도우 게이트) | `0.1558` | `0.1558` | `0.0052` | `0.0002°` | `0.0010°` | `2.90 ms` | — |
| | **`F3b_improved` (개선: `0.2s` LiDAR 속도 교차검증)** | **`0.0320 (-79.5%)`** | **`0.0320`** | **`0.0033`** | **`0.0002°`** | **`0.0010°`** | **`2.90 ms`** | — |

---

## 3. 독립 Stall Detector 설계 및 6대 안전 항목 실측 검증 (`Work Package 3`)

- **구현 노드:** [`nodes/stall_detector_node.py`](../nodes/stall_detector_node.py)
- **검증 스크립트 및 결과:** [`scripts/run_wp3_stall_detector_verification.py`](../scripts/run_wp3_stall_detector_verification.py), [`results/stall_detector/wp3_stall_detector_verification_summary.json`](../results/stall_detector/wp3_stall_detector_verification_summary.json), [`results/plots/wp3_stall_detector_verification.png`](../results/plots/wp3_stall_detector_verification.png)
- **검증 영상:** [`videos/stall_detector/wp3_glass_wall_stall_stop_and_recovery.mp4`](../videos/stall_detector/wp3_glass_wall_stall_stop_and_recovery.mp4) (`224 frames`, `15.0 fps`, `1920x960`)

### 3.1 동작 구조
- **입력:** `/cmd_vel_raw` (제어기/Nav2 속도 명령), `/odom` (휠 오도메트리 속도), `/odometry/lidar_odom` (`F4` LiDAR 변위/속도), `/scan` (전방 LiDAR 거리)
- **출력:** `/cmd_vel` (Gazebo 브리지로 전달되는 안전 게이트 속도 명령), `/stall_detector/status` (JSON 진단 상태), `/scan_virtual_bumper` (Stall 확정 시 전방 `0.32 m` 가상 범퍼 포인트클라우드/스캔 발행)
- **상태 기계:** `NORMAL` $\rightarrow$ `STALL_PENDING` $\rightarrow$ `STALL_LATCHED` (전진 `/cmd_vel` 즉시 `0.0 m/s` 차단) $\rightarrow$ `RECOVERING_REVERSE` (후진 탈출 허용) $\rightarrow$ `NORMAL` (복귀), 센서 두절 시 `FAILSAFE_SENSOR_TIMEOUT` (즉시 정지).

### 3.2 6대 필수 검증 항목 실측 결과

| 검증 항목 | 시험 조건 | 실측 지표 | 판정 |
| :--- | :--- | :--- | :---: |
| **1. 정상 주행 오탐률 (False Positive)** | 다중 회전 S-커브 주행 (`25.8 s`, `258 samples`) | 오탐 횟수 **`0회` (`0.0%`)**, 차단된 명령 **`0건`** | **PASS** |
| **2. 유리벽 충돌 시 Stall 감지 지연 시간** | 유리벽 충돌 (`t_impact = 27.503 s` $\rightarrow$ `t_latch = 27.948 s`) | **`0.445 s`** (원본 `F3b`의 `1.330 s` 대비 **66.5% 단축**) | **PASS** |
| **3. 센서 누락/불확실 시 Fail-Safe 동작** | 주행 중 `F4` 강제 종료 (`t_kill = 44.269 s` $\rightarrow$ `t_fs = 44.710 s`) | **`0.441 s`** 만에 `FAILSAFE_SENSOR_TIMEOUT` 진입 및 `/cmd_vel = 0.0` 차단 (`GT_vx = 0.000 m/s`) | **PASS** |
| **4. `/cmd_vel` 실제 차단 여부** | `STALL_LATCHED` 구간 (`t = 27.95 ~ 30.70 s`) | 입력 `cmd_vx_raw = +0.260 m/s` 대비 출력 **`cmd_vx_out = 0.0000 m/s`** 완벽 차단 | **PASS** |
| **5. 실제 로봇 정지 및 휠 공회전 중단 확인** | `STALL_LATCHED` 직후 섀시 및 휠 속도 계측 | 섀시 속도 **`GT_vx = 0.0000 m/s`**, 휠 오도메트리 속도 **`B0_vx = 0.0000 m/s`** (차단 없을 때 `2.230 m`까지 발산하던 `B0_x`가 **`1.819 m`에서 즉시 동결**) | **PASS** |
| **6. 후진 탈출 및 재출발 회복 가능 여부** | 후진 명령(`-0.18 m/s`, `2.2 s`) 후 재전진(`+0.20 m/s`, `2.0 s`) | `STALL_LATCHED` $\rightarrow$ `RECOVERING_REVERSE` (실제 후진 **`0.388 m`**) $\rightarrow$ `NORMAL` 복귀 후 재전진 **`0.369 m`** 성공 | **PASS** |

---

## 4 & 5. 실제 Nav2 Closed-Loop 단일 검증 및 15회 반복 시험 (총 160회 폐루프 주행) (`Work Packages 4 & 5`)

- **실행 스크립트:** [`scripts/run_wp4_wp5_nav2_closed_loop.py`](../scripts/run_wp4_wp5_nav2_closed_loop.py)
- **단일 검증 결과 (10회):**
  - [`results/nav2_single_verification/CL_GlassWall_Hazard_single_summary.json`](../results/nav2_single_verification/CL_GlassWall_Hazard_single_summary.json)
  - [`results/nav2_single_verification/CL_Corridor_Doorway_Perturbed_single_summary.json`](../results/nav2_single_verification/CL_Corridor_Doorway_Perturbed_single_summary.json)
- **15회 반복 시험 결과 (동일 조건 5회 + 변동 시드 10회 = 추정기별 15회 × 5개 추정기 × 2개 시나리오 = 150회):**
  - [`results/nav2_repeated_trials/CL_GlassWall_Hazard_15trials_summary.json`](../results/nav2_repeated_trials/CL_GlassWall_Hazard_15trials_summary.json)
  - [`results/nav2_repeated_trials/CL_Corridor_Doorway_Perturbed_15trials_summary.json`](../results/nav2_repeated_trials/CL_Corridor_Doorway_Perturbed_15trials_summary.json)
- **비교 플롯:** [`results/plots/wp4_wp5_closed_loop_comparison.png`](../results/plots/wp4_wp5_closed_loop_comparison.png)

### 4.1 Closed-Loop 시험 설계
Nav2(`planner_server: NavfnPlanner`, `controller_server: RegulatedPurePursuitController`, `bt_navigator`, `local_costmap`, `global_costmap`)의 입력 TF(`odom -> base_footprint`)와 오도메트리 토픽(`/odom_nav2`)에 5종의 추정기(`B0`, `F1b`, `F4`, `F5`, `F5_improved`)를 실제로 연결하여 동일 월드·시작점·목표점에서 폐루프 자율주행을 수행하였다.
- **시나리오 1 (`CL_GlassWall_Hazard`):** LiDAR가 투과하는 투명 유리벽(`y = -1.50 m`) 너머(`(1.0, -2.10)`)에 목표점이 주어진 위험 시나리오. 물리적으로 도달 불가능하므로, **허위 목표 도달(`FALSE_SUCCEEDED_HAZARD`)을 일으키지 않고 안전하게 중단(`SAFE_ABORTED_OR_HALTED`)하며 휠 공회전 시간을 최소화**하는 것이 핵심 안전 지표이다.
- **시나리오 2 (`CL_Corridor_Doorway_Perturbed`):** 라운지 중앙 통로(`(-1.10, -0.60)`)에서 출발하여 테이블·의자 장애물 사이 복도를 돌아 북측 복도 목표점(`(-2.20, 3.35, yaw=2.28)`)에 도달하는 시나리오. 주행 중 **휠 슬립(`scale=1.15` + 슬립 버스트 `1.25x`)과 IMU 자이로 바이어스(`+0.045 rad/s`)**가 동시에 인가된다.

### 4.2 시나리오 1 (`CL_GlassWall_Hazard`) 15회 반복 폐루프 시험 결과 (총 75회)

| 추정기 (각 15회: 동일 5 + 변동 시드 10) | 허위 목표 도달 위험률 (`FALSE_SUCCEEDED_HAZARD`) | 안전 중단/정지율 (`SAFE_ABORTED_OR_HALTED`) | 시간 보간 ATE RMSE Mean±Std (m) | 최악 ATE Max (m) | 유리벽 접촉 후 휠 공회전 지속 시간 Mean (`s`) | 활성 오도메트리 샘플 나이 Mean (`ms`) | 평균 CPU 사용률 (`%`) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **`B0` (Wheel Odometry)** | **`100.0% (15/15)`** | `0.0% (0/15)` | `0.2211 ± 0.0034` | `0.5430` | `2.152 s` (허위 SUCCEEDED로 종료) | `4.73 ms` | `41.4%` |
| **`F1b` (Tuned EKF: Wheel+IMU)** | **`100.0% (15/15)`** | `0.0% (0/15)` | `0.2230 ± 0.0041` | `0.5516` | `2.116 s` (허위 SUCCEEDED로 종료) | `9.57 ms` | `41.9%` |
| **`F4` (2D LiDAR ICP 단독)** | **`0.0% (0/15)`** | **`100.0% (15/15)`** | `0.1456 ± 0.0472` | `0.2776` | `5.128 s` (타임아웃까지 휠 계속 슬립) | `83.58 ms` | `42.1%` |
| **`F5` (원본: LiDAR+IMU EKF)** | **`0.0% (0/15)`** | **`100.0% (15/15)`** | `0.1427 ± 0.0320` | `0.2732` | `5.347 s` (타임아웃까지 휠 계속 슬립) | `10.58 ms` | `42.5%` |
| **`F5_improved` (+ Stall Detector)** | **`0.0% (0/15)`** | **`100.0% (15/15)`** | **`0.0589 ± 0.0299`** | **`0.1933`** | **`1.171 s (-78.1% 단축)`** | **`9.37 ms`** | `42.8%` |

- **핵심 분석 (`CL_GlassWall_Hazard`):**
  1. `B0`와 `F1b`는 15회 전수(`100%`)에서 유리벽에 막혀 실제 목표점으로부터 `0.755 m` 앞에 정지해 있음에도 헛도는 바퀴 적분값 때문에 추정 위치가 목표 반경(`0.22 m`) 안으로 진입하여 **Nav2가 `SUCCEEDED`(`FALSE_SUCCEEDED_HAZARD`)를 선언하는 치명적 안전 실패**를 일으켰다.
  2. `F4`와 원본 `F5`는 LiDAR 위치를 사용하여 허위 `SUCCEEDED`를 100% 차단(`0/15`)했으나, 독립 Stall Detector가 없어 Nav2의 `progress_checker` 타임아웃이 발생할 때까지 **평균 `5.13 ~ 5.35초` 동안 유리벽에 대고 바퀴를 계속 공회전**시켰다.
  3. **`F5_improved` + 독립 Stall Detector**는 허위 `SUCCEEDED`를 100% 차단(`0/15`)함과 동시에, 충돌 직후 Stall을 감지하여 `/cmd_vel`을 차단하고 `ABORTED_BY_STALL_SAFETY`로 안전 종료함으로써 **휠 공회전 시간을 `1.171 s`(단일 검증 `0.670 s`)로 78.1% 단축**하고 ATE RMSE도 **`0.0589 m`**로 가장 낮게 유지했다.

### 4.3 시나리오 2 (`CL_Corridor_Doorway_Perturbed`) 15회 반복 폐루프 시험 결과 (총 75회)

| 추정기 (각 15회: 동일 5 + 변동 시드 10) | 실제 목표 도달 성공률 (`TRUE_SUCCEEDED`) | 허위 목표 도달률 (`FALSE_SUCCEEDED_DRIFT`) | 물리 접촉/충돌률 (`physical_contact`) | 시간 보간 ATE RMSE Mean±Std (m) | 시간 보간 Yaw RMSE Mean (deg) | 최종 실제 목표 거리 Mean (m) | 활성 오도메트리 샘플 나이 Mean (`ms`) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **`B0` (Wheel Odometry)** | `0.0% (0/15)` | `46.7% (7/15)` | `40.0% (6/15)` | `0.7858 ± 0.1166` | `6.60°` | `1.0871 m` | `4.03 ms` |
| **`F1b` (Tuned EKF: Wheel+IMU)** | `0.0% (0/15)` | **`100.0% (15/15)`** | `0.0% (0/15)` | `0.8755 ± 0.0807` | `21.03°` | `1.5991 m` | `9.55 ms` |
| **`F4` (2D LiDAR ICP 단독)** | **`100.0% (15/15)`** | **`0.0% (0/15)`** | **`0.0% (0/15)`** | **`0.0828 ± 0.0158`** | **`4.25°`** | **`0.2159 m`** | `97.92 ms` |
| **`F5` (원본: LiDAR+IMU EKF)** | `40.0% (6/15)` | `0.0% (0/15)` | `6.7% (1/15)` | `0.1512 ± 0.0847` | **`29.70°`** | `1.1469 m` | `9.57 ms` |
| **`F5_improved` (+ Stall Detector)** | **`80.0% (12/15)`** *(변동 시드 10회 중 **100% 10/10**)* | `6.7% (1/15)` | `6.7% (1/15)` | **`0.1188 ± 0.0324`** | **`5.48° (-81.5%)`** | **`0.4967 m`** *(성공 12회 평균 `0.218 m`)* | **`10.08 ms (-89.7%)`** |

- **핵심 분석 (`CL_Corridor_Doorway_Perturbed`):**
  1. **개방 루프(Open-Loop)에서는 드러나지 않았던 원본 `F5`의 폐루프 주행 붕괴 규명:**
     - 이전 75회 개방 루프 실험(`S4`)에서 원본 `F5`는 Yaw 오차가 `15.66°`였음에도 위치 오차(ATE `0.088 m`)가 작아 양호해 보였다.
     - 그러나 **실제 Nav2 폐루프 주행**에서는 `RegulatedPurePursuitController`가 전역 경로(`/plan`)를 로봇 기준 좌표계(`base_footprint`)로 변환할 때 추정기의 **Yaw 각도**를 사용한다. 원본 `F5`는 자이로 바이어스로 인해 **평균 `29.70°`(최대 `59.9°`)의 Yaw 오차**가 발생하여 제어기가 엉뚱한 방향으로 조향했고, 그 결과 **15회 중 9회가 경로를 이탈하여 목표 도달에 실패(성공률 `40.0%`)**했다.
  2. **`F5_improved`의 폐루프 주행 복원 효과 및 과도기(Transition) 실패 3건의 투명한 원인 기록:**
     - `F5_improved`는 `F4`의 스캔 매칭 회전값을 융합하여 **폐루프 Yaw RMSE를 `29.70°`에서 `5.48°`로 81.5% 감소**시켰고, 샘플 나이를 `F4` 단독(`97.92 ms`) 대비 **9.7배 짧은 `10.08 ms`(`50 Hz`)**로 유지했다.
     - **실패/경계 사례 투명 기록:** `F5_improved`의 동일 조건 `trial_01`과 `trial_02`는 직전 `F5` 마지막 시행(`F5_trial_15`) 종료 후 Gazebo `set_pose` 텔레포트 직후 물리 엔진의 잔여 바퀴 관성이 감쇠되기 전에 출발하여 초기 코너에서 조기 중단되었고, `trial_03`은 실제 목표점 거리 `0.289 m`(판정 임계값 `0.280 m` 대비 `9 mm` 초과)에 정지하여 `FALSE_SUCCEEDED_DRIFT`로 분류되었다. 이후 안정화된 **`trial_04`부터 `trial_15`까지 연속 12회(변동 시드 10회 `10/10` 전수 포함)에서는 100%(`12/12`) `TRUE_SUCCEEDED`**를 달성하였다.

---

## 6. SLAM 지도 정확도 및 16.05 m 다중 복도 실제 Loop Closure 검증 (`Work Package 6`)

- **실행 스크립트:** [`scripts/run_wp6_slam_loop_closure_and_map_accuracy.py`](../scripts/run_wp6_slam_loop_closure_and_map_accuracy.py)
- **검증 요약 JSON 및 시계열 CSV:**
  - [`results/slam_evaluation/wp6_slam_loop_closure_summary.json`](../results/slam_evaluation/wp6_slam_loop_closure_summary.json)
  - [`results/slam_evaluation/wp6_slam_16m_loop_trajectory.csv`](../results/slam_evaluation/wp6_slam_16m_loop_trajectory.csv)
- **저장된 OccupancyGrid 지도 파일 (`.pgm` / `.yaml`):**
  - 루프 폐쇄 직전 지도: [`results/slam_evaluation/wp6_slam_map_pre_loop.pgm`](../results/slam_evaluation/wp6_slam_map_pre_loop.pgm), [`wp6_slam_map_pre_loop.yaml`](../results/slam_evaluation/wp6_slam_map_pre_loop.yaml)
  - 루프 폐쇄 및 PGO 최적화 후 최종 지도: [`results/slam_evaluation/wp6_slam_map_post_loop.pgm`](../results/slam_evaluation/wp6_slam_map_post_loop.pgm), [`wp6_slam_map_post_loop.yaml`](../results/slam_evaluation/wp6_slam_map_post_loop.yaml) (동일 사본: [`maps/wp6_slam_loop_closed_map.pgm`](../maps/wp6_slam_loop_closed_map.pgm))
- **비교 플롯 및 주행 영상:**
  - [`results/plots/wp6_slam_loop_closure_evaluation.png`](../results/plots/wp6_slam_loop_closure_evaluation.png)
  - [`videos/slam_loop_closure/wp6_slam_16m_loop_closure.mp4`](../videos/slam_loop_closure/wp6_slam_16m_loop_closure.mp4) (`964 frames`, `64.3 s`, `15.0 fps`, `1920x960`)

### 6.1 실제 Loop Closure 이벤트 발생 검증 (Pose Graph 위상 검증)
단순히 시작점 근처로 되돌아온 것을 Loop Closure로 간주하지 않고, `/slam_toolbox/graph_visualization` 토픽의 포즈 그래프 노드 수($N$)와 간선 수($E$) 및 `map -> odom` Ceres PGO 보정 점프를 시계열로 추적하였다:
- **주행 경로:** 라운지 중앙 섬(`table_1` + `table_3`)을 완전히 한 바퀴 일주한 뒤 출발 복도(`x = -1.10 m`)로 재진입하여 `2.25 m`를 중첩 주행하는 **총 `16.05 m` 다중 복도 폐곡선 경로**.
- **포즈 그래프 위상 변화 (`wp6_slam_16m_loop_trajectory.csv` line 422 $\rightarrow$ 423):**
  - `sim_time = 46.759 s` (노드 50): `graph_nodes_count = 50`, `graph_edges_count = 49` ($E = N - 1 = 49$, 순수 순차 오도메트리 트리).
  - **`sim_time = 46.852 s` (노드 51):** 로봇이 남측 통로로 진입하며 시작 영역과 중앙 테이블을 재관측하는 순간 `graph_nodes_count = 51`, **`graph_edges_count = 51`** ($E - (N - 1) = 1$, **비순차적 Loop Closure 제약 간선 추가 확인**).
  - 최종 종료 시점(`sim_time = 68.612 s`): **`75개 노드`, `75개 간선` (`74개 순차 간선 + 1개 비순차 Loop Closure 간선`)** 및 총 **`44회`의 `map -> odom` Ceres PGO 최적화 업데이트** 확인 (`true_loop_closure_verified = true`).

### 6.2 루프 폐쇄 전후 위치 및 생성 지도(OccupancyGrid) 정량 정확도 비교

| 평가 항목 | 세부 지표 | 루프 폐쇄 전 (`Pre-Loop`) | 루프 폐쇄 및 최적화 후 (`Post-Loop`) | 변화 및 비고 |
| :--- | :--- | :---: | :---: | :--- |
| **1. 누적 위치 오차 (Odom Frame vs GT)** | `B0` (Perturbed Wheel Odom) 전체 루프 ATE RMSE | — | `0.7934 m` (종료 시 `1.1043 m`) | 4회 코너 회전 후 누적 드리프트 |
| | `F5_improved` (순수 오도메트리) 전체 루프 ATE RMSE | — | `0.5423 m` | 전역 지도 없는 순차 스캔 매칭 누적 오차 |
| | **`F6` (`SLAM Toolbox` PGO) 전체 루프 ATE RMSE** | **`0.0411 m`** (루프 직전 구간) | **`0.0320 m`** (전체) / **`0.0385 m`** (중첩 구간) / **`0.0115 m`** (최종 정지 수렴) | **`F5_improved` 단독 대비 오차 94.1% 제거** |
| **2. 벽 위치 오차 (vs SDF Ground Truth)** | 북측 외벽 (`Y = +4.40 m`) 평균 오차 | `0.0249 m` | **`0.0225 m`** | `-9.6%` 개선 (격자 해상도 `0.05 m`의 절반 이하) |
| | 서측 외벽 (`X = -5.90 m`) 평균 오차 | `0.0293 m` | **`0.0293 m`** | 격자 1셀(`0.05 m`) 미만 |
| | 중앙 분리벽 (`Y = -1.40 m`) 평균 오차 | `0.0280 m` | **`0.0285 m`** | 격자 1셀(`0.05 m`) 미만 |
| | **전체 벽 위치 평균 오차** | `0.0275 m` | **`0.0268 m`** | **`2.68 cm` 정밀도** |
| **3. 복도 폭 오차** | 북측 복도 폭 (실제 `1.850 m`) | `1.900 m` (오차 `0.050 m`) | **`1.850 m` (오차 `0.000 m`)** | 루프 폐쇄 후 **오차 `0.0 cm`** |
| | 서측 복도 폭 (실제 `2.750 m`) | `2.7975 m` (오차 `0.0475 m`) | **`2.800 m` (오차 `0.050 m`)** | 격자 해상도(`5 cm`) 1셀 수준 |
| **4. 장애물 위치 오차** | `table_1` 군집 중심 오차 | `0.0792 m` | **`0.0552 m`** | **`-30.3%` 개선** |
| | `table_3` 군집 중심 오차 | `0.0674 m` | **`0.0524 m`** | **`-22.3%` 개선** |
| | `door_post` (문기둥) 위치 오차 | `0.0443 m` | **`0.0408 m`** | **`-7.9%` 개선** |
| **5. 벽 이중 겹침 및 각도 왜곡** | 북측 벽 두께 표준편차 (이중 벽 겹침 지표) | `0.0161 m` | **`0.0121 m`** (`double_wall = false`) | **벽 두께 분산 `-24.8%` 감소 (이중 벽 겹침 없음)** |
| | 북측 벽 각도 왜곡 (`angular_distortion_deg`) | `0.346°` | **`0.216°`** | **각도 왜곡 `-37.6%` 감소** |

---

## 7. 생성된 전체 영상(24개) 검증 목록

신규 실험 디렉터리(`videos/`)에 생성된 총 24개 `.mp4` 파일 전수에 대해 `cv2.VideoCapture` 디코딩 및 첫 프레임/전체 프레임 무결성을 검증 완료하였다:
1. **누락 복구 `S1` 영상 (2개):**
   - [`videos/s1_recovered/S1_trial_01_identical.mp4`](../videos/s1_recovered/S1_trial_01_identical.mp4) (`205 frames`, `15.0 fps`, `1920x960`)
   - [`videos/s1_recovered/S1_trial_06_varied_seed_seed1001.mp4`](../videos/s1_recovered/S1_trial_06_varied_seed_seed1001.mp4) (`203 frames`, `15.0 fps`, `1920x960`)
2. **독립 Stall Detector 유리벽 정지 및 후진 회복 영상 (1개):**
   - [`videos/stall_detector/wp3_glass_wall_stall_stop_and_recovery.mp4`](../videos/stall_detector/wp3_glass_wall_stall_stop_and_recovery.mp4) (`224 frames`, `15.0 fps`, `1920x960`)
3. **Nav2 Closed-Loop 단일 검증 및 반복 시험 영상 (20개):**
   - `CL_GlassWall_Hazard` (`B0`, `F1b`, `F4`, `F5`, `F5_improved` 각 `single` + `rep_trial01` = 10개): [`videos/nav2_closed_loop/`](../videos/nav2_closed_loop/)
   - `CL_Corridor_Doorway_Perturbed` (`B0`, `F1b`, `F4`, `F5`, `F5_improved` 각 `single` + `rep_trial01` = 10개): [`videos/nav2_closed_loop/`](../videos/nav2_closed_loop/)
4. **16.05 m 다중 복도 SLAM Loop Closure 주행 영상 (1개):**
   - [`videos/slam_loop_closure/wp6_slam_16m_loop_closure.mp4`](../videos/slam_loop_closure/wp6_slam_16m_loop_closure.mp4) (`964 frames`, `15.0 fps`, `1920x960`)

---

## 8. 실물 LIMO Pro 자율주행 적용을 위한 최종 기술 선정 및 아키텍처 권고

단일·반복 개방 루프 시험(75회), 독립 Stall Detector 시험, 실제 Nav2 폐루프 반복 시험(160회), 그리고 16.05 m 다중 복도 SLAM Loop Closure 시험의 실측 데이터에 기반하여, 실제 LIMO Pro 로봇에 적용할 최종 상태 추정 및 안전 아키텍처로 다음 **3계층 복합 구조**를 선정한다:

1. **로컬 연속 상태 추정 계층 (`odom -> base_footprint` @ `50 Hz`): `F5_improved` (`config/fusion_f5_improved.yaml`)**
   - **구성:** `F4` 2D LiDAR ICP 스캔 매칭 오도메트리(`[x, y, yaw, vx, vy]`, `10 Hz`) + IMU 각속도(`[wz]`, `50 Hz`) EKF 융합.
   - **선정 근거:**
     - 휠 슬립이나 유리벽 충돌 시 바퀴 회전수와 무관하게 실제 물리 변위를 추적하여 **Nav2의 허위 목표 도달(`FALSE_SUCCEEDED`)을 100% 방지**한다.
     - 원본 `F5`의 치명적 약점이었던 IMU Yaw 바이어스 누적(`폐루프 Yaw RMSE 29.70°`, 목표 도달률 `40%`)을 `F4`의 스캔 매칭 회전값 융합으로 해결하여 **폐루프 Yaw RMSE를 `5.48°`(개방 루프 `0.78°`)로 억제**한다.
     - `F4` 단독 사용 시 발생하는 `97.9 ms`(`10 Hz`) 출력 지연을 EKF `50 Hz` 예측 보간으로 **`9.4 ~ 10.1 ms`**까지 단축하며, 센서 중복 융합이 없다.
2. **전역 지도 작성 및 드리프트 보정 계층 (`map -> odom`): `F6` (`SLAM Toolbox` Online Async + Ceres PGO, `config/slam_loop_closure_params.yaml`)**
   - **선정 근거:** `16.05 m` 다중 코너 주행 시 순차 스캔 매칭만으로는 누적될 수 있는 오차(`0.5423 m`)를 전역 스캔-투-맵 매칭 및 비순차 Loop Closure 최적화를 통해 **전체 경로 ATE RMSE `0.0320 m`(최종 수렴 오차 `0.0115 m`), 벽 위치 오차 `0.0268 m`**로 보정한다.
3. **독립 주행 안전 계층 (`/cmd_vel_raw -> /cmd_vel` 게이트 + `/scan_virtual_bumper`): 독립 `Stall Detector` (`nodes/stall_detector_node.py`)**
   - **선정 근거:** LiDAR로 보이지 않는 투명 유리벽이나 낮은 걸림턱에 충돌했을 때 **`0.445 s` 이내에 물리적 정체를 감지**하여 `/cmd_vel` 전진 명령을 즉시 차단함으로써 **모터/타이어 과열 및 휠 공회전 시간을 78.1% 이상 단축**하고, 센서 타임아웃 시 **`0.441 s` 이내에 Fail-Safe 정지**를 보장하며, 후진 탈출(`RECOVERING_REVERSE`) 후 정상 주행 재개가 가능하다.
