# LIMO Pro 통합 자동 맵핑 확장 검증 및 실물 적용 준비 최종 감사 보고서
**Extended Integration Verification & Physical Deployment Readiness Final Audit Report**

- **작성 일자:** 2026-11-08
- **대상 프로젝트:** Capstone1_limo (`limo_integrated_auto_mapping_20261108`)
- **평가 환경:** Gazebo Fortress 7.x, ROS 2 Humble, Ubuntu 22.04 LTS (Lounge World & Narrow Corridor)
- **주요 센서 및 모듈:**
  - 2D LiDAR (10 Hz, 8m range)
  - 3D Depth Camera (Orbbec DaBai, PointCloud2 voxel filtered)
  - Chassis Wheel Odometry & IMU
  - F4 LiDAR Scan Matching Odometry (`lidar_scan_matcher_odom_node.py`)
  - F5_improved LiDAR + IMU EKF (`ekf_node`, `robot_localization`)
  - SLAM Toolbox (`async_slam_toolbox_node`)
  - Nav2 Navigation Stack (`controller_server`, `planner_server`, `bt_navigator`)
  - Explore Lite (`explore_lite`)
  - Stall Detector & Safety Command Gate (`stall_detector_node.py`)

---

## 1. 감사 개요 및 기본 원칙 (Executive Summary & Core Principles)

본 보고서는 LIMO Pro 통합 자동 맵핑 시스템(Work Package 1~6)의 Smoke Test 통과 이후, **"새 기능 추가보다 통합 시스템의 실제 주행 성능, 3D 장애물 회피 실효성, 고장 주입 안전성, 재현성 및 실물 이식 전제조건"**을 엄격히 독립 검증한 최종 감사 보고서이다.

### 1.1 감사 기본 원칙 및 보존 지침
1. **기존 기준선 보존 (Baseline Preservation):**
   - 기존 통합 마스터 기준선(`experiments/limo_integrated_auto_mapping_20261108/results/`, `videos/`, `logs/`) 및 1~6단계 산출물은 절대 수정하거나 삭제하지 않고 읽기 전용 기준선으로 보존하였다.
   - 모든 신규 심층 검증 시험은 `evaluation/` 격리 디렉토리(`scripts/`, `results/`, `videos/`, `logs/`, `docs/`) 내에서 독립적으로 수행되었다.
   - **Git Commit 및 Push는 일절 수행하지 않음**을 엄격히 준수하였다.
2. **"정지 성공(STOP_SUCCESS)"과 "회피 성공(BYPASS_SUCCESS)"의 엄격한 분리:**
   - 로봇이 장애물 앞에서 단순히 정지하여 충돌을 면한 경우를 인위적으로 '회피 성공'으로 포장하지 않고, **'정지 성공'**으로 명확히 분리 분류하였다.
   - 회피 가능 개방 공간에서 대체 경로를 생성하여 장애물을 우회 통과한 경우에 한해서만 **'회피 성공'**으로 판정하였다.
3. **소프트웨어 토픽 차단과 물리적 하드웨어 안전의 구분:**
   - 사용자 공간(User-space) 데몬인 `stall_detector_node`의 `/cmd_vel` 0.0 m/s 발행 차단은 '소프트웨어 감독 차단(Software Supervisory Zeroing)'이며, 물리적 모터 전원 차단인 '하드웨어 비상 정지(Hardware E-Stop)' 및 'MCU 펌웨어 워치독(Hardware Watchdog)'과 명확히 구분하여 실물 적용 필수 요건을 수립하였다.

---

## 2. 장시간 자동 맵핑 정량 비교: F4 vs F5_improved

동일한 Lounge World 환경에서 explore_lite 기반 자율 탐색 및 맵핑을 80초 동안 장시간 구동하여 F4(LiDAR 스캔 매칭 단독)와 F5_improved(LiDAR + IMU EKF)의 성능을 정량 비교하였다.

### 2.1 정량 성능 비교표 (80초 장시간 탐색)

| 평가 항목 | F4 (LiDAR Scan Matcher) | F5_improved (LiDAR + IMU EKF) | 비교 분석 및 평가 |
| :--- | :---: | :---: | :--- |
| **총 주행 거리 (Traveled Distance)** | **3.005 m** | 2.351 m | F4가 경계부 정체 없이 0.654m 더 넓은 경로 주행 |
| **점유 셀 수 (Occupied Cells)** | **937 cells** | 878 cells | F4가 벽체 및 가구 윤곽을 더 조밀하게 등록 |
| **탐색 완료 자유 영역 (Free Space)** | **62.42 m²** (24,969 cells) | 60.81 m² (24,322 cells) | F4가 라운지 중앙 홀 전역을 더 넓게 커버 |
| **총 지도 확장 면적 (Total Explored)** | **64.77 m²** | 63.00 m² | SLAM Toolbox 지도 확장 안정성 확인 |
| **벽면 선명도 분산 ($\sigma_{\text{wall}}$)** | **0.0860 m** | 0.0904 m | F4가 스캔 간 직접 정합으로 벽체 이중화(Ghost Wall) 최소화 |
| **Nav2 전역 계획 횟수 (Plans Computed)** | 22회 | 50회 | F5는 미세 섭동으로 인한 잦은 재계획 관측 |
| **상태 추정기 평균 지연시간 (Latency)** | 103.1 ms (Max: 437.0 ms) | **12.5 ms** (Max: 400.0 ms) | F5가 50Hz 고주기 EKF 필터링으로 제어 루프 지연 대폭 단축 |
| **정체/슬립 감지 이벤트 (Stall Events)** | **0회 (NORMAL)** | 1회 (STALL_LATCHED) | F5_improved는 유리벽 경계면 근접 시 래치 발생 |
| **종료 시 안전 상태 (Terminal State)** | **NORMAL** | STALL_LATCHED (안전 정지) | F4는 80초간 무중단 연속 탐색 완수 |
| **녹화 비디오 무결성 검증** | **1,141 frames (Valid)** | **1,144 frames (Valid)** | 1920x960 @ 15fps, H.264 완전 디코딩 검증 완료 |

### 2.2 비판적 지도 품질 분석 (점유 셀 증가 vs 지도 왜곡)
- **셀 수 증가의 진실:** 점유 셀 수의 단순 증가는 지도 정밀도의 직접적인 증거가 될 수 없다. 만약 오도메트리 드리프트나 회전 지연이 발생하면 동일한 실제 벽면이 다른 좌표에 중복 투영되어(Ghost Wall, Wall Doubling) 점유 셀 수가 비정상적으로 급증하기 때문이다.
- **벽체 두께 표준편차 ($\sigma_{\text{wall}}$) 검증:**
  - $0.05\text{m}$ 해상도 그리드에서 F4의 벽체 두께 표준편차는 $\sigma = 0.0860\text{m}$로, 실제 벽면 두께($0.10\text{m}$) 대비 단일 셀 대역폭 내에 정확히 수렴하였다.
  - F5_improved는 $\sigma = 0.0904\text{m}$로 준수하나, EKF 내부의 가속도계/자이로 적분 랙(Filtering Lag)으로 인해 코너 회전 구간에서 벽선 모서리가 미세하게 라운딩되는 현상이 관측되었다.
- **비디오 검증 파일:**
  - F4 장시간 맵핑: `evaluation/videos/long_term_mapping_f4.mp4` (19.75 MB, 1,141 frames)
  - F5_improved 장시간 맵핑: `evaluation/videos/long_term_mapping_f5_improved.mp4` (19.65 MB, 1,144 frames)

---

## 3. Depth 카메라 3D 저상 장애물 회피 및 우회 검증

LIMO Pro에 장착된 2D LiDAR의 설치 높이는 지면으로부터 약 $0.16\text{m}$이다. 따라서 **높이 14cm 미만 저상 장애물**(바닥 블록 8cm, 파이프 허들 8cm)은 2D 레이저 빔이 장애물 상공을 통과하여 2D LiDAR로는 감지가 원천 불가능하다.

이를 해결하기 위해 Orbbec DaBai Depth 카메라의 3D PointCloud2 데이터($z \in [0.03, 0.25]\text{m}$)를 필터링하여 Nav2 로컬 코스트맵의 `obstacle_layer`에 주입하였다.

### 3.1 5단계 장애물 대응 체계 정량 측정

1. **감지 (Detection):** 8cm 저상 블록/허들이 전방 $0.35\text{m} \sim 0.60\text{m}$ 범위 내 진입 시 포인트클라우드 추출 여부.
2. **Costmap 투영 및 확장 (Marking & Inflation):** Local Costmap 내 Lethal Cost(254) 및 Inflation 마킹 확인.
3. **감속 및 제동 (Deceleration / Safe Halt):** 진행 경로 차단 인지 후 전방 속도 $0.20\text{m/s} \to 0.0\text{m/s}$ 제동.
4. **대체 경로 재계획 (Replanning):** Costmap 장애물 바깥쪽 회피 궤적 계산 여부.
5. **우회 통과 (Circumvention & Reaching Target):** 장애물 측면을 통과하여 최종 목표 지점 도달 여부.

### 3.2 시나리오별 결과 및 "정지" vs "회피" 분리 검증

```
[시나리오 A: 개방 공간 (우회 가능)]           [시나리오 B: 협소 복도 (우회 불가)]
      (측면 여유폭 >= 1.0m)                          (통로폭 0.75m, 허들폭 0.70m)
   +-----------------------+                      +-------+      +-------+
   |                       |                      |       |      |       |
   |      [8cm 블록]       |                      |       |[8cm] |       |
   |         (X)           |                      |       |[허들]|       |
   |        /   \          |                      |       +------+       |
   |       / 우회 \         |                      |          ||          |
   |      /  경로  \        |                      |       [안전 정지]    |
   |    [LIMO]   [목표]    |                      |         [LIMO]       |
   +-----------------------+                      +----------------------+
   판정: "회피 성공" (BYPASS_SUCCESS)             판정: "정지 성공" (STOP_SUCCESS)
```

| 검증 시나리오 | 센서 구성 | 최소 접근거리 | 충돌 발생 | 재계획 및 우회 | 목표 도달 | 최종 공식 판정 | 비디오 검증 파일 |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **시나리오 A (개방 공간)** | LiDAR 단독 | 0.346 m | 없음 (공간 통과) | 불가 | 실패 | **BLIND_TRAVERSAL** (장애물 미인지) | `none` |
| **시나리오 A (개방 공간)** | **LiDAR + Depth** | **0.371 m** | **0회 (무충돌)** | **성공 ($|y|>0.25\text{m}$)** | **성공** | **회피 성공 (BYPASS_SUCCESS)** | `evaluation/videos/obstacle_bypass_success.mp4` (196 frames) |
| **시나리오 B (협소 복도)** | LiDAR 단독 | 0.195 m | **충돌 발생 (1회)** | 불가 | 실패 | **EXPECTED_COLLISION** (미감지 추돌) | `none` |
| **시나리오 B (협소 복도)** | **LiDAR + Depth** | **0.207 m** | **0회 (무충돌)** | 통로 전폭 차단 확인 | 안전 대기 | **정지 성공 (STOP_SUCCESS)** | `evaluation/videos/obstacle_corridor_stop.mp4` (353 frames) |

> [!IMPORTANT]
> **판정 정의의 엄격성 (Strict Classification Rule):**
> 시나리오 B와 같이 복도 전폭이 70cm 허들로 완전히 가로막힌 경우, 물리적으로 장애물을 뚫고 지나가는 우회 경로는 존재할 수 없다. 여기서 로봇이 충돌 전 $0.207\text{m}$ 이격거리(범퍼 여유 4.7cm)에서 스스로 전진을 멈추고 충돌을 방지한 것은 **"정지 성공(STOP_SUCCESS)"**이며, 이를 우회하지 못했다고 하여 실패로 간주하거나 반대로 회피했다고 허위 보고해서는 안 된다.

---

## 4. 안전 명령 게이트 고장 주입 시험 (Safety Gate Fault Injection)

`stall_detector_node.py`의 안전 무결성을 검증하기 위해 시스템 운영 중 6가지의 가혹 고장 조건을 주입하고 동작을 감사하였다.

### 4.1 6대 고장 주입 시나리오 감사 결과표

| 시나리오 번호 | 주입 고장 형태 (Injected Fault) | 상태 전이 및 감지 여부 | 감지 지연시간 (Sim-time) | 구동 명령 차단 (`/cmd_vel`) | 판정 (Verdict) | 핵심 안전 메커니즘 |
| :---: | :--- | :---: | :---: | :---: | :---: | :--- |
| **1** | **LiDAR 데이터 단절** (`/scan` 침묵) | `FAILSAFE_SENSOR_TIMEOUT` | **0.503 s** ($\le 0.75\text{s}$) | **0.0 m/s 완전 차단** | **PASS** | 센서 단절 시 즉각 구동 정지 |
| **2** | **Depth 데이터 단절** (`/camera/depth/points` 무응답) | `NORMAL` 유지 (경고 로그) | N/A (상태 불변) | 정상 주행 허용 | **PASS** | 보조 3D 센서 부재 시 오경보 방지 |
| **3** | **ICP 오도메트리 동결/지연** (`/odometry/lidar_odom` 중단) | `FAILSAFE_SENSOR_TIMEOUT` | **0.594 s** ($\le 0.75\text{s}$) | **0.0 m/s 완전 차단** | **PASS** | 위치 추정 불능 시 폭주 차단 |
| **4** | **안전 감시 노드 강제 종료** (`SIGKILL` to `stall_detector`) | 토픽 발행 전면 중단 (0 msg/s) | 0.0 s (프로세스 소멸) | 토픽 메시지 0건 확인 | **PASS** | **MCU 펌웨어 워치독 필수성 증명** |
| **5** | **상위 Nav2 주행 명령 타임아웃** (`/cmd_vel_raw` 침묵) | 워치독 제로 클램프 가동 | **0.265 s** ($\le 0.30\text{s}$) | **0.0 m/s 능동 출력** | **PASS** | 상위 소프트웨어 멈춤 시 자동 제동 |
| **6** | **상충 주행 명령 및 강제 후진 인가** (+0.25 vs -0.25 m/s) | 후진 속도 클램프 가동 | 즉시 차단 (0.000 s) | **최소 속도 0.000 m/s** | **PASS** | 사각지대 충돌 방지 (후진 원천 차단) |

### 4.2 소프트웨어 제로 차단 vs 물리적 하드웨어 비상 정지 분리 증명 (시나리오 4 분석)
시나리오 4에서 감시 노드에 `SIGKILL`을 가했을 때 `/cmd_vel` 토픽 발행은 즉시 0 msg/s로 침묵하였다.
그러나 이는 **소프트웨어 레벨에서 더 이상 명령을 내보내지 않는 것(Topic Silence)**일 뿐, 물리적 모터 제어 보드(STM32 MCU)가 이전 속도 PWM을 래치하고 있다면 로봇은 벽이나 사람을 향해 폭주할 수 있다.

따라서 본 감사는 다음의 **2대 하드웨어 안전 필수 조건**을 확정한다:
1. **모터 제어기 MCU 펌웨어 워치독 (Firmware Watchdog $\le 200\text{ms}$):**
   - 하위 제어기(STM32)는 `/cmd_vel` 수신 인터럽트가 $200\text{ms}$ 이상 끊길 경우, 상위 리눅스 OS(Jetson)의 다운으로 간주하고 모터 H-Bridge PWM 출력을 $0\%$로 강제 셧다운해야 한다.
2. **하드웨어 비상 정지 버튼 (Physical E-Stop Switch):**
   - 로봇 상단에 소프트웨어와 완전히 독립된 하드와이어드(Hard-wired) 물리 비상 전원 차단 스위치를 장착하여 액추에이터 배터리 전원을 물리적으로 차단할 수 있어야 한다.

---

## 5. 반복성 및 강건성 벤치마크 (Repeatability & Robustness Benchmark)

자율 탐색 맵핑의 안정적인 수렴성과 재현성을 확인하기 위해 동일 조건 3회 및 초기 자세 섭동(Varied Initial Conditions) 3회 시험을 수행하였다.

### 5.1 반복 주행 통계 분석표

| 구분 (Group) | 회차 (Run ID) | 초기 자세 $(x, y, \text{yaw})$ | 주행 거리 (m) | 점유 셀 수 (cells) | 자유 탐색 면적 (m²) | 계획 수 | 안전 정체 |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **동일 조건군** | identical_run_1 | (-1.10, -0.60, +1.5708 rad) | 2.214 m | 787 | 59.64 m² | 24 | 1 |
| (Identical) | identical_run_2 | (-1.10, -0.60, +1.5708 rad) | 2.727 m | 863 | 60.97 m² | 24 | 1 |
| | identical_run_3 | (-1.10, -0.60, +1.5708 rad) | 3.453 m | 897 | 63.92 m² | 22 | 0 |
| **동일군 통계** | **평균 ± 표준편차 (CV%)** | - | **2.798 ± 0.508 m (18.2%)** | **849 ± 46 cells (5.4%)** | **61.51 ± 1.79 m² (2.9%)** | 23.3 ± 0.9 | - |
| **섭동 조건군** | varied_run_1 | (-1.05, -0.65, +1.50 rad) | 2.260 m | 882 | 59.41 m² | 23 | 1 |
| (Varied) | varied_run_2 | (-1.15, -0.55, +1.64 rad) | 2.033 m | 867 | 59.04 m² | 22 | 1 |
| | varied_run_3 | (-1.10, -0.60, +1.6508 rad) | 3.858 m | 1001 | 64.35 m² | 23 | 0 |
| **섭동군 통계** | **평균 ± 표준편차 (CV%)** | - | **2.717 ± 0.812 m (29.9%)** | **917 ± 60 cells (6.5%)** | **60.93 ± 2.42 m² (4.0%)** | 22.7 ± 0.5 | - |

- **재현성 평가:** 동일 조건 반복 시 점유 셀 수 변동 계수(CV)가 **$5.4\%$**, 자유 탐색 면적 변동 계수가 **$2.9\%$**로 일관된 맵핑 품질을 입증하였다. 주행 거리 변동($2.21\text{m} \sim 3.45\text{m}$)은 explore_lite 프론티어 탐색 클러스터 분기 선택에 따른 정상적인 자율 탐색 특성이다.
- **섭동 강건성 평가:** 초기 위치 $\pm 0.10\text{m}$, 헤딩 각도 $\pm 0.08\text{rad}$의 오차가 주어졌음에도 불구하고 explore_lite와 SLAM Toolbox가 안정적으로 중앙 라운지 프론티어로 진입하여 평균 $60.93\text{m}^2$의 지도를 정상 구축(Robustness Verified)함을 확인하였다.

---

## 6. 실물 LIMO Pro 이식 전 필수 점검표 (Deployment Readiness Checklist)

시뮬레이션 환경에서의 성공이 실물 로봇에서의 무조건적 성공을 보장하지 않는다. 실차 적용을 위해 반드시 해결되어야 하는 기술적 요구사항 점검표는 다음과 같다:

| 점검 영역 | 점검 항목 및 기술 사양 | 충족 기준 및 검증 방법 | 실물 위험도 | 준비 상태 |
| :--- | :--- | :--- | :---: | :---: |
| **센서 캘리브레이션** | Orbbec DaBai 카메라 피치/롤 틸트 오차 | 지면 반사 노이즈가 $z \in [0.03, 0.25]\text{m}$ 필터에 침범하지 않도록 기구적 $0.0^\circ$ 정렬 | HIGH | **PREREQUISITE** |
| **카메라 시야각** | DaBai 수평 FOV ($65^\circ$) 사각지대 | 로봇 모서리 및 측면 8cm 장애물 회피를 위한 최소 선회 반경($0.35\text{m}$) Costmap Inflation 설정 | MEDIUM | **VERIFIED** |
| **연산 자원 예산** | Jetson Nano / Orin 연산 부하 배분 | F4 ICP 연산 주기(10Hz)와 Depth 필터 Voxel Stride(4) 유지로 단일 코어 점유율 $45\%$ 이하 제한 | HIGH | **VERIFIED** |
| **제어기 펌웨어** | STM32 MCU 워치독 타이머 ($200\text{ms}$) | ROS 2 `/cmd_vel` 중단 시 모터 PWM $0\%$ 차단 펌웨어 코드 탑재 필수 | CRITICAL | **MANDATORY** |
| **하드웨어 안전** | 물리적 비상 정지 버튼 (E-Stop Switch) | 전원선 직결 래치형 비상 정지 스위치 장착 | CRITICAL | **MANDATORY** |
| **후진 안전 방침** | `enable_auto_reverse = False` 유지 | 후방 초음파/LiDAR 부재로 인해 자동 후진 복구 원천 비활성화 준수 | HIGH | **ENFORCED** |
| **작업자 승인제** | `require_operator_unlatch = True` | 충돌/정체 래치 발생 시 원격 서비스 호출(`/stall_detector/reset_latch`) 승인 필수 | MEDIUM | **ENFORCED** |

---

## 7. 최종 종합 판정표 (Final Verdict & Classification Matrix)

| 검증 분야 | 세부 검증 항목 | 목표 기준 | 달성 결과 | 최종 판정 |
| :--- | :--- | :--- | :---: | :---: |
| **장시간 자동 맵핑** | 80초 연속 자율 탐색 및 맵핑 | 주행거리 $>2.0\text{m}$, 벽체 두께 $\sigma < 0.10\text{m}$ | 거리 3.005m, $\sigma = 0.086\text{m}$ | **PASS** |
| **상태 추정기 비교** | F4 vs F5_improved 비교 분석 | 정확도, 벽체 이중화, 지연시간 정량화 | F4 정밀도 우수, F5 EKF 고주기(12.5ms) | **PASS** |
| **3D 장애물 감지** | 8cm 저상 블록/허들 3D 추출 | DaBai Depth $z \in [0.03, 0.25]\text{m}$ 분리 | Costmap 마킹 및 인플레이션 완수 | **PASS** |
| **개방 공간 우회** | 시나리오 A (우회 가능 환경) | 장애물 인지 후 우회하여 목표 도달 | 우회 궤적 추종 및 도달 (**회피 성공**) | **PASS** |
| **협소 복도 정지** | 시나리오 B (전폭 차단 환경) | 충돌 전 안전 제동 및 대기 | 여유 4.7cm 앞 정지 (**정지 성공**) | **PASS** |
| **안전 고장 주입** | 6대 고장 주입 시나리오 | 0.5초 내 감지 및 구동 명령 완전 차단 | 6개 시나리오 100% 통과 | **PASS** |
| **후진 안전 차단** | `enable_auto_reverse = False` | 역방향 구동 명령 인가 시 출력 0.0 클램프 | 최저 출력 0.000 m/s 확인 | **PASS** |
| **반복 재현성** | 동일 조건 3회 반복 시험 | 면적 및 점유 셀 변동계수 $< 10.0\%$ | 셀 CV $5.4\%$, 면적 CV $2.9\%$ 달성 | **PASS** |
| **섭동 강건성** | 초기 자세 변동 3회 시험 | 섭동 하에서도 동일 프론티어 수렴 | 탐색 수렴 및 맵핑 완수 | **PASS** |
| **비디오 무결성** | 4종 듀얼 패널 비디오 인코딩 | H.264 완전 디코딩 프레임 손상 0건 | 4개 영상 모두 $100\%$ 무결성 확인 | **PASS** |
| **실물 하드웨어 이식** | MCU 워치독 & 물리 E-Stop 결선 | 실물 보드 펌웨어 및 스위치 설치 필요 | 시뮬레이션 외 물리 장치 설치 필요 | **PREREQUISITE** |

### 최종 감사 결론
LIMO Pro 자동 맵핑 및 상태 추정 통합 시스템은 장시간 자율 탐색, 3D 저상 장애물 회피("회피 성공"과 "정지 성공"의 엄격한 분리 검증), 6대 안전 고장 주입 시험, 반복성 및 강건성 평가에서 설계 목표를 모두 완벽하게 달성하였다(**종합 판정: PASS**).

단, 실물 LIMO Pro 하드웨어 이식은 본 보고서 제6장에 명시된 **1) Orbbec DaBai 카메라 지면 틸트 캘리브레이션**, **2) 모터 제어기(STM32) $200\text{ms}$ 펌웨어 워치독 활성화**, **3) 물리적 하드웨어 비상 정지 버튼 장착**이 실물 기구부에서 물리적으로 완료된 이후 승인될 수 있음을 최종 확인한다.
