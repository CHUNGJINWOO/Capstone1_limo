# LIMO Pro 대표 영상 카탈로그 및 100% 프레임 디코딩 무결성 감사 보고서

## 1. 개요 및 검증 원칙

자율주행 및 상태 추정 시뮬레이션에서 생성되는 영상(MP4)은 단순한 시각화 자료가 아니라, 시뮬레이터와 ROS 2 노드의 실시간 상호작용 및 물리적 거동을 입증하는 **실측 멀티미디어 증거(Empirical Evidence)**이다.
본 감사는 다음 두 가지 핵심 사항을 검증하고 기록한다:
1. **100% 전수 프레임 디코딩 검증 (Full Frame Decoding Integrity)**:
   - 첫 번째 프레임만 확인하는 불완전한 헤더 검사가 아니라, [verify_mp4_complete_decoding.py](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_final_deployment_prep_20261108/scripts/verify_mp4_complete_decoding.py)를 통해 전체 프레임($0 \sim N-1$)을 순회 디코딩하여 손상(Corrupt) 및 누락(Dropped) 프레임이 0건임을 전수 입증.
2. **이전 75회 S1 F4 보간 RMSE 수치 불일치 정정**:
   - 기존 Table 1에 기재되었던 `0.0226 m`와 재계산 결과인 `0.0414 m` 사이의 불일치 원인을 역추적하여 정정 기록.

---

## 2. 이전 75회 시험 S1 F4 RMSE 수치 불일치 정정 (Correction)

### 2.1 수치 정정 내역
- **기존 문서 오기**: Table 1에 S1 F4 보간 RMSE가 `0.0226 m`로 기재됨.
- **실제 원본 15회 데이터 재계산 결과**: **`0.0414 ± 0.0153 m`** ([previous_75trial_zoh_vs_interpolated_recomputation.json](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_final_deployment_prep_20261108/results/previous_75trial_zoh_vs_interpolated_recomputation.json) 참조).

### 2.2 오기 원인 분석 (Root Cause)
- 보존된 원본 데이터 파일 `experiments/limo_state_estimation_benchmark_20261107/results/state_estimation/S1_trial_01~15.csv`를 전수 재계산한 결과:
  - F4 ZOH RMSE: `0.0438 ± 0.0151 m`
  - F4 Linear Interpolated RMSE: `0.0414 ± 0.0153 m`
- `0.0226 m`라는 수치는 WP2 소거 시험 요약 파일(`wp2_f4_f5_f3b_ablation_summary.json`)의 F4 직선 단순 주행 구간의 평균 오차($0.0226\text{ m}$)가 보고서 작성 과정에서 Table 1의 S1 복합 회전 주행 칸으로 잘못 복사-붙여넣기(Copy-paste artifact)된 전사 오류임이 명확히 규명되었다.
- 본 보고서를 통해 공식 수치를 **`0.0414 ± 0.0153 m`**로 최종 정정한다.

---

## 3. 전수 48개 MP4 영상 100% 디코딩 무결성 감사 결과

본 감사는 OpenCV `cv2.VideoCapture` 엔진을 통해 시스템 내 생성된 48개 영상 전수(총 용량 약 420MB, 총 프레임 12,800+ 프레임)를 1프레임씩 끝까지 디코딩하여 수행되었으며, 결과는 [video_decoding_audit.json](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_final_deployment_prep_20261108/results/video_decoding_audit.json)에 기록되었다.

### 3.1 디코딩 무결성 총괄 통계
- **총 감사 영상 수**: **`48 편`**
- **100% 전수 디코딩 성공 영상 수**: **`48 편` (`100.0%`)**
- **손상/드롭 프레임 발생 영상 수**: **`0 편` (`0.0%`)**
- **모든 영상 무결성 검증 판정**: **`ALL_VIDEOS_FULLY_DECODED = TRUE`**

---

## 4. 핵심 대표 영상 카탈로그 (Representative Video Catalog)

### 4.1 최종 사전 배포 검증 스위트 (`limo_final_deployment_prep_20261108`)

| 영상 파일명 | 해상도 / FPS | 총 프레임 / 재생시간 | 코덱 | 영상 오버레이 정보 및 주요 입증 내용 |
| :--- | :---: | :---: | :---: | :--- |
| **`REEVAL_F4_identical_trial01.mp4`** | 1280x720 / 15 | 256 프레임 / 17.07s | FMP4 | F4 1회차 동일 조건 주행. Nav2 액션 상태 `SUCCEEDED`, GT 목표 거리 $0.251\text{m}$, 실시간 ICP 매칭 및 궤적 오버레이. |
| **`REEVAL_F4_varied_seed2001.mp4`** | 1280x720 / 15 | 256 프레임 / 17.07s | FMP4 | F4 변동 시드(seed=2001) 조건. 스폰 편차 하에서 안착 후 문틀을 통과하여 $0.206\text{m}$ 목표 정밀 도달. |
| **`wp3_glass_wall_stall_safety_locked.mp4`** | 1280x720 / 15 | 199 프레임 / 13.27s | FMP4 | 유리벽 정면 충돌 시험. $0.351\text{s}$ 만에 `STALL_LATCHED` 진입, 모터 명령 $0.0\text{m/s}$ 차단, 후진 명령 $0.0\text{m/s}$ 원천 차단 및 운영자 수동 언래치 복귀 시연. |

### 4.2 Nav2 폐루프 및 SLAM 루프 폐합 스위트 (`limo_nav2_closed_loop_verification_20261107`)

| 영상 파일명 | 해상도 / FPS | 총 프레임 / 재생시간 | 코덱 | 주요 내용 및 검증 포인트 |
| :--- | :---: | :---: | :---: | :--- |
| **`wp6_slam_16m_loop_closure.mp4`** | 1280x720 / 15 | 964 프레임 / 64.27s | FMP4 | 16.05m 대형 폐루프 주행. $t=46.852\text{s}$ 루프 폐합 제약 에지 삽입 및 Ceres PGO에 의한 지도 왜곡 즉각 보정 실시간 표출. |
| **`CL_Corridor_Doorway_Perturbed_F4_rep_trial01.mp4`** | 1280x720 / 15 | 245 프레임 / 16.33s | FMP4 | F4 폐루프 문틀 통과 시험. 슬립 버스트($1.25\times$) 하에서도 스캔 매칭 안정성 유지. |
| **`CL_Corridor_Doorway_Perturbed_F5_improved_rep_trial01.mp4`** | 1280x720 / 15 | 116 프레임 / 7.73s | FMP4 | F5 개선형(LiDAR+IMU) 시험. 빠른 회전 및 복도 안정 주행. |
| **`CL_GlassWall_Hazard_F5_improved_rep_trial01.mp4`** | 1280x720 / 15 | 140 프레임 / 9.33s | FMP4 | 유리벽 위험 구역에서 Stall Detector 가상 범퍼 반응 및 차체 보호 정지. |
| **`CL_Corridor_Doorway_Perturbed_B0_rep_trial01.mp4`** | 1280x720 / 15 | 244 프레임 / 16.27s | FMP4 | 원시 휠 오도메트리 실패 대조군. 슬립 구간에서 위치가 크게 이탈하여 벽면 간섭 유발. |
| **`CL_Corridor_Doorway_Perturbed_F1b_rep_trial01.mp4`** | 1280x720 / 15 | 178 프레임 / 11.87s | FMP4 | EKF 오도메트리 허위 성공 재현. 차체는 정지했으나 EKF가 가상 목표 도달을 판정하는 현상 입증. |
| **`S1_trial_01_identical.mp4`** | 1280x720 / 15 | 205 프레임 / 13.67s | FMP4 | 복구된 S1 동일 조건 기준 궤적 영상. |
| **`S1_trial_06_varied_seed_seed1001.mp4`** | 1280x720 / 15 | 203 프레임 / 13.53s | FMP4 | 복구된 S1 변동 조건 기준 궤적 영상. |

### 4.3 라운지 A~D 비교 스위트 (`limo_state_estimation_benchmark_20261107`)

- **시나리오 A (유리벽 충돌 및 휠 슬립)**:
  - `scenario_A_baseline.mp4` (260 프레임 / 17.33s): 유리벽 1.43m 관통 및 오도메트리 무한 발산 장면.
  - `scenario_A_improved.mp4` (295 프레임 / 19.67s): 가상 범퍼 및 Stall 감지에 의한 물리 정지.
  - `scenario_A_comparison.mp4` (295 프레임 / 19.67s): 기본형과 개선형 분할 화면 비교.
- **시나리오 B (의자 다리 회피)**: `scenario_B_baseline.mp4`, `scenario_B_improved.mp4`, `scenario_B_comparison.mp4` (232~264 프레임 전수 무결).
- **시나리오 C (테이블 밑 통과)**: `scenario_C_baseline.mp4`, `scenario_C_improved.mp4`, `scenario_C_comparison.mp4` (200~301 프레임 전수 무결).
- **시나리오 D (협소 통로 회전 주행)**: `scenario_D_baseline.mp4`, `scenario_D_improved.mp4`, `scenario_D_comparison.mp4` (261~594 프레임 전수 무결).

---

## 5. 비디오 오버레이 텔레메트리 규격

모든 대표 주행 영상은 상단 및 좌하단에 다음 7가지 핵심 실측 데이터를 실시간 프레임 동기화 오버레이로 렌더링하여 재현성을 보장한다:
1. **Scenario & Estimator Name**: 활성 상태 추정기 명칭 (F4, F5_imp, B0 등)
2. **Nav2 Action State**: `NAVIGATING` $\to$ `SUCCEEDED` / `ABORTED` / `SAFE_ABORTED`
3. **Ground Truth Distance to Goal**: 실시간 실제 목표 도달 거리 ($m$)
4. **Estimated Distance to Goal**: 알고리즘 추정 목표 도달 거리 ($m$)
5. **Stall Detector State**: `NORMAL` $\to$ `STALL_PENDING` $\to$ `STALL_LATCHED`
6. **Command Blocking Status**: `CmdBlocked: True/False`
7. **Contact Status**: 차체 물리 충돌 여부 (`Contact: True/False`)
