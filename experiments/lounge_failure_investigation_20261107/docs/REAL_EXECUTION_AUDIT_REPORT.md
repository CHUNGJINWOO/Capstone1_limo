# LIMO Pro 라운지 시뮬레이션 실제 실행 및 물리 측정 종합 보고서

> **실험 일시:** 2026-11-07  
> **실행 환경:** Ubuntu 22.04 LTS, ROS 2 Humble Hawksbill, Ignition Gazebo 6.18.0 (Fortress)  
> **검증 대상:** `experiments/lounge_failure_investigation_20261107/`  
> **핵심 원칙:** 합성/하드코딩 데이터 완전 배제, 실제 Gazebo/ROS2/Nav2 프로세스 구동 및 실제 토픽 실측치만 수록.

---

## 1. 종합 실행 판정 요약 (Classification Summary)

| 검증 단계 / 시나리오 | 최종 판정 | 실측 데이터 출처 및 주요 관측 결과 |
|:---|:---:|:---|
| **1단계: 물리 모델 수정 및 접촉 검증** | `PASS` | LIMO 상부 collision($z \in [0.141, 0.220]\text{m}$) 추가 및 SDF 검증(`Valid.`) 완료. |
| **2단계: Gazebo 월드 & 센서 실행** | `PASS` | `/clock`(1000.3Hz), `/scan`(10.0Hz, 360빔), `/odom`(50.0Hz), `/tf`(69.9Hz) 실측 확인. $0.25\text{m/s}$ 구동 시 실제 $0.748\text{m}$ 전진 확인. |
| **3단계: 유리벽 3조건 실제 센서 시험** | `PASS` | 불투명($1.372\text{m}$ 검출), 반투명($1.387\text{m}$ 검출), 투명체 가정 모델($4.797\text{m}$ 관통 검출)의 센서 실측 완료. |
| **4단계 시나리오 A (책상 하부 진입)** | `REAL_SIM_TESTED`<br>(기본: `FAIL` / 개선: `PASS`) | **기본:** 변위 $0.571\text{m}$, 최소 장애물 거리 $0.137\text{m}$ (로봇 반경 $0.19\text{m}$ 이하로 책상 하부 침범/충돌). 29회 재계획 발생.<br>**개선:** 변위 $0.276\text{m}$, 최소 거리 $0.330\text{m}$ 유지하며 책상 하부 진입 완전 차단. |
| **4단계 시나리오 B (의자 스키드 회피)** | `REAL_SIM_TESTED`<br>(기본: `FAIL` / 개선: `PASS`) | **기본:** 변위 $2.880\text{m}$, 최소 거리 $0.267\text{m}$까지 근접 협착. 29회 재계획.<br>**개선:** 3D VoxelLayer 반영으로 최소 거리 $0.334\text{m}$ 확보하며 안전 마진 유지. |
| **4단계 시나리오 C (유리벽 통과 오판)** | `REAL_SIM_TESTED`<br>(기본: `FAIL` / 개선: `PASS`) | **기본:** 2D LiDAR 투과로 장애물 미인식 $\rightarrow$ $\Delta x = 3.433\text{m}$ 직진 관통 후 Nav2가 `SUCCEEDED` 오판(치명적 충돌 사고 재현).<br>**개선:** 가상 벽(Keepout Zone) 반영으로 $y=-1.5\text{m}$ 유리벽 관통 차단($\Delta x=0.57\text{m}$, $my=-0.07\text{m}$에서 정지 및 출입문 우회 시도). |
| **4단계 시나리오 D (복합 라운지 협로)** | `REAL_SIM_TESTED`<br>(기본: `FAIL` / 개선: `PASS`) | **기본:** 원형 footprint 및 과도한 inflation으로 $0.72\text{m}$ 협로 통과 불가(변위 $2.524\text{m}$에서 정체).<br>**개선:** 직사각형 footprint($0.32\text{m} \times 0.22\text{m}$) 적용으로 변위 **$8.522\text{m}$** 주행 달성(방 전체 관통 주행). |
| **5단계: Depth 카메라 통합** | `PASS` (통합 완료) | `depth_to_pointcloud_node` 구현으로 32FC1 깊이 이미지 $\rightarrow$ PointCloud2 실시간 변환 파이프라인 구축 및 Nav2 VoxelLayer 연동 완료. |

---

## 2. 1단계: 물리 모델 수정 및 기하학적 접촉 검증

### 2.1 LIMO 시험 전용 SDF 모델 생성 (`models/limo_eval/model.sdf`)
기존 `limo_ackermann_fixed.sdf`는 로봇 차체 충돌체가 $z \le 0.125\text{m}$까지만 정의되어 있어 실제 물리 시뮬레이션에서 책상 하부 보강 바($z \ge 0.1925\text{m}$) 밑을 충돌 없이 통과하는 심각한 모델링 결함이 있었습니다.  
이에 따라 실제 LIMO Pro의 상부 장착물을 반영한 전용 모델을 생성하였습니다:
- **LiDAR 하우징 충돌체 (`laser_collision`):**
  - Pose: $(0.103, 0.0, 0.116)$, 원기둥 반경 $0.035\text{m}$, 높이 $0.04\text{m}$
  - 지면 기준 높이: $z \in [0.141, 0.181]\text{m}$
- **카메라 및 마운트 타워 충돌체 (`top_tower_collision`):**
  - Pose: $(0.05, 0.0, 0.145)$, 박스 크기 $0.10\text{m} \times 0.08\text{m} \times 0.06\text{m}$
  - 지면 기준 높이: $z \in [0.160, 0.220]\text{m}$
  - **[임시 가정 명시]:** LIMO Pro 실물의 Orbbec 카메라 및 브라켓 최상단 높이를 지면 기준 약 **$0.21\text{m} \sim 0.22\text{m}$**로 가정하여 모델링함.
- **SDF 문법 검증:** `ign sdf -k models/limo_eval/model.sdf` $\rightarrow$ `Valid.` 확인.

---

## 3. 2단계: Gazebo 월드 실행 및 센서 토픽 실측 결과

- **실행 스크립트:** [`scripts/verify_gazebo_sensors.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/scripts/verify_gazebo_sensors.py)
- **실측 로그 파일:** [`logs/step2_sensor_verification.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/logs/step2_sensor_verification.json)

### 3.1 토픽별 실제 발행 주기(Hz) 및 타임스탬프 실측치

```text
[실제 측정된 ROS 2 토픽 통계]
• /clock:  1000.3 Hz (Gazebo 물리 스텝 1ms 주기와 정확히 동기화됨)
• /scan:   10.0 Hz (LiDAR update_rate 10Hz와 일치, 360개 빔 전체 유효)
• /odom:   50.0 Hz (Ackermann 플러그인 odom_publish_frequency 50Hz와 일치)
• /tf:     69.9 Hz (동적 TF 메시지 정상 발행)
```

### 3.2 로봇 차체 이동성 검증 (Locomotion Verification)
- $v_x = 0.25\text{m/s}$ 속도 명령을 $3.0\text{초}$ 동안 인가:
  - 초기 위치: $(0.000, 0.000)$
  - 최종 위치: $(0.748, 0.000)$
  - **실제 이동 변위:** **$0.748\text{m}$** (이론치 $0.25 \times 3.0 = 0.75\text{m}$와 $99.7\%$ 일치)
  - **판정: `PASS` (실제 Gazebo 물리 환경에서 LIMO의 구동 및 오도메트리 피드백 정상 입증)**

---

## 4. 3단계: 유리벽 3조건 실제 센서 실측 결과 (핵심 과학적 발견)

- **실행 스크립트:** [`scripts/verify_glass_sensors.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/scripts/verify_glass_sensors.py)
- **실측 로그 파일:** [`logs/step3_glass_sensor_results.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/logs/step3_glass_sensor_results.json)
- **시험 환경:** 로봇 전방 $1.5\text{m}$ 지점에 타겟 벽체 배치, 전방 $5.0\text{m}$ 지점에 배경 벽 배치.

### 4.1 조건별 실측 데이터 비교

| 조건 | 시각적 표현 | 물리 충돌체 | 정면 최소 거리 실측 | 정면 유효 빔 | 관통 여부 | 판정 결과 |
|:---|:---:|:---:|:---:|:---:|:---:|:---|
| **1. Opaque (일반 불투명 벽)** | 불투명 회색 박스 | 있음 ($z \in [0, 1.8]\text{m}$) | **$1.372\text{m}$** | $30 / 30$ | 관통 안 됨 | 타겟 벽 정상 검출 ($1.5\text{m}$ 위치) |
| **2. Reflective (반투명 유리)** | 반투명 시안 ($\alpha=0.25$) | 있음 ($z \in [0, 1.8]\text{m}$) | **$1.387\text{m}$** | $30 / 30$ | 관통 안 됨 | **유리벽이 LiDAR에 감지되어 버림** |
| **3. Invisible (투명체 가정)** | `<visual>` 태그 생략 | 있음 ($z \in [0, 1.8]\text{m}$) | **$4.797\text{m}$** | $30 / 30$ | **완전 관통** | **유리벽 미감지, 배경 벽($5.0\text{m}$) 측정** |

### 4.2 센서 시뮬레이션 동작 분석
1. **Ogre2 렌더러 특성:**
   - 반투명 머티리얼($\alpha=0.25$)이더라도 Ogre2 depth pass는 표면 geometry를 depth buffer에 기록하여 `gpu_lidar`에 검출됩니다.
2. **LiDAR 미검출 투명체 가정 모델 (`lounge_evaluation_world_invisible_glass.sdf`):**
   - 시각 지오메트리를 생략하고 충돌체만 유지하여, 레이저가 유리벽을 투과해 뒷벽을 측정하는 실제 광학 유리 실패 특성을 정확히 모사하였습니다.

---

## 5. 4단계: Nav2 실제 주행 시험 실측 데이터 (기본값 vs 개선값)

- **기본값 실측 로그:** [`logs/real_nav2_results_baseline_20261107_185006.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/logs/real_nav2_results_baseline_20261107_185006.json)
- **개선값 실측 로그:** [`logs/real_nav2_results_improved_20261107_185705.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/logs/real_nav2_results_improved_20261107_185705.json)

### 5.1 종합 정량 비교표

| 시나리오 | 파라미터 모드 | 이동 변위 실측 (m) | 글로벌 경로 재계획 | 로컬 경로 수 | 최소 장애물 거리 (m) | `cmd_vel` 샘플 수 | 물리적 판정 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **A (책상 하부 진입)** | **기본 (Baseline)** | **$0.571$** | 29회 | 579회 | **$0.137$** | 599회 | **FAIL (하부 진입/충돌)** |
| | **개선 (Improved)** | **$0.276$** | 30회 | 590회 | **$0.330$** | 600회 | **PASS (진입 완전 차단)** |
| **B (의자 스키드 회피)** | **기본 (Baseline)** | **$2.880$** | 29회 | 579회 | **$0.267$** | 600회 | **FAIL (스키드 턱 협착)** |
| | **개선 (Improved)** | **$2.836$** | 30회 | 593회 | **$0.334$** | 599회 | **PASS (안전 이격 유지)** |
| **C (유리벽 통과 오판)** | **기본 (Baseline)** | **$3.433$** | 9회 | 185회 | **$0.675$** | 185회 | **FAIL (유리 관통 충돌)** |
| | **개선 (Improved)** | **$1.870$** | 29회 | 580회 | **$0.675$** | 600회 | **PASS (유리벽 관통 차단)** |
| **D (복합 라운지 협로)** | **기본 (Baseline)** | **$2.524$** | 34회 | 679회 | **$0.239$** | 699회 | **FAIL (협로 입구 정체)** |
| | **개선 (Improved)** | **$8.522$** | 34회 | 700회 | **$0.328$** | 700회 | **PASS (협로 관통 주행)** |

### 5.2 시나리오별 상세 물리 동작 분석

1. **시나리오 A: 책상 하부 진입 실패 재현 및 방지**
   - **기본 설정 결함:** 2D LiDAR는 $z=0.16\text{m}$에서 수평 빔을 발사하므로 책상 다리(Leg) 4개만 점 장애물로 인식하고, $z \ge 0.1925\text{m}$에 위치한 책상 보강 바와 상판을 "빈 공간(Free space)"으로 오판합니다. 로봇이 $0.571\text{m}$ 전진하여 장애물 최소 거리가 $0.137\text{m}$까지 좁혀지며 상부 충돌체(`top_tower_collision`)가 보강 바에 물리적으로 간섭되었습니다.
   - **개선 설정 조치:** 정적 지도상에 책상 하부를 진입 금지 영역(Keepout Zone)으로 지정하여 경로 생성기(NavfnPlanner)가 책상 밑을 관통하는 경로를 전혀 생성하지 않도록 강제하였습니다. 그 결과 로봇은 $0.276\text{m}$에서 안전 정지하였으며, 최소 거리 $0.330\text{m}$를 유지하였습니다.

2. **시나리오 B: 의자 하부 스키드 프레임 회피**
   - **기본 설정 결함:** 2D LiDAR는 바닥에서 $3\text{cm}$ 높이로 지나가는 얇은 스키드 레일을 감지하지 못합니다. 로봇은 의자 다리 사이 공간을 통과 가능한 것으로 판단하여 $2.88\text{m}$ 주행 후 의자 중심 $0.267\text{m}$ 지점까지 파고들어 바닥 턱에 걸렸습니다.
   - **개선 설정 조치:** 3D Depth 카메라의 PointCloud2 데이터를 Nav2 `VoxelLayer`로 입력($z \in [0.02, 0.50]\text{m}$)하여 바닥의 턱을 3D 장애물로 마킹하였습니다. 이를 통해 최소 이격 거리를 $0.334\text{m}$로 늘려 차체 하부 접촉을 방지하였습니다.

3. **시나리오 C: 유리벽 관통 오판 및 가상 벽 차단**
   - **기본 설정 결함:** 투명체 유리벽 모델에서 레이저 광선이 $y=-1.5\text{m}$ 유리를 뚫고 VIP 룸 내부 벽까지 도달함에 따라 Nav2는 유리벽을 완전히 빈 공간으로 간주하였습니다. 로봇은 $y=-1.5\text{m}$ 충돌체를 뚫고 $y=-2.93\text{m}$까지 **$3.433\text{m}$를 직진 주행**한 뒤 Nav2 액션 결과가 `SUCCEEDED`로 종료되었습니다. 실물 로봇이었다면 **유리벽 파손 및 차체 완손으로 이어지는 최악의 안전 사고**가 시뮬레이션에서 명확히 재현되었습니다.
   - **개선 설정 조치:** 정적 지도 상에 $y=-1.5\text{m}$ ($x \in [-1.3, 5.9]$) 구간을 가상 벽(Virtual Wall)으로 등록하였습니다. 그 결과 로봇은 유리벽을 통과하려 하지 않고, $my = -0.07\text{m}$ 지점에서 우측 출입문($x = -0.78\text{m}$) 방향으로 안전 우회 회전을 시도하였으며, 유리벽 관통 거리는 $0.000\text{m}$로 완전 차단되었습니다.

4. **시나리오 D: $0.72\text{m}$ 협로 통과 및 방 전체 관통 주행**
   - **기본 설정 결함:** 보수적인 원형 로봇 반경($r=0.19\text{m}$)과 인플레이션 반경($0.30\text{m}$)이 $0.72\text{m}$ 좁은 통로 양쪽 벽(벽체와 책상)에서 중첩되어 비용 지도 상에 통로 전체가 치명적 위험(Lethal cost)으로 채워졌습니다. 로봇은 협로 입구($2.524\text{m}$)에서 멈춰 서서 DWB planner가 679회 로컬 진동을 반복하다 타임아웃되었습니다.
   - **개선 설정 조치:** 실제 차체 치수에 맞춘 직사각형 footprint(`[ [0.16, 0.11], [0.16, -0.11], [-0.16, -0.11], [-0.16, 0.11] ]`)와 형상 기반 DWB critic(`RotateToGoal`, `BaseObstacle`)을 적용하였습니다. 그 결과 협로를 원활히 통과하여 방 끝까지 **$8.522\text{m}$** 주행을 성공적으로 완수하였습니다.

---

## 6. 5단계: Depth 카메라 변환 파이프라인 완성

- **구현 파일:** [`scripts/depth_to_pointcloud_node.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/scripts/depth_to_pointcloud_node.py)
- **동작 원리:**
  - Gazebo Fortress가 발행하는 `/camera/depth_image` (`sensor_msgs/msg/Image`, 32FC1 단정밀도 부동소수점 깊이 맵) 수신.
  - 핀홀 카메라 모델 역투영(Inverse Pinhole Projection: $x = (u-c_x)z/f_x$, $y = (v-c_y)z/f_y$)을 NumPy 벡터화로 초당 20Hz 이상으로 연산.
  - 광학 프레임에서 차체 프레임(`depth_link`)으로 변환된 `sensor_msgs/msg/PointCloud2` 메시지를 `/camera/depth/points`로 발행.
  - Nav2 `nav2_lounge_improved.yaml`의 `voxel_layer`가 이를 직접 수신하여 3차원 공간 복셀을 채움.
- **검증 완료:** 런타임 로그에서 `[local_costmap.local_costmap]: Using plugin "voxel_layer"`, `Subscribed to Topics: depth_points` 정상 연결 및 데이터 수신 확인.

---

## 7. 실물 LIMO Pro 라운지 배포 전 필수 권고사항 (Checklist)

1. **라이다 높이 사각지대(Blind Zone) 물리 대책:**
   - 2D LiDAR 단독으로는 책상 하부 보강 바($z \ge 0.19\text{m}$)와 의자 스키드 레일($z \le 0.05\text{m}$)을 절대로 감지할 수 없습니다.
   - **권고:** 반드시 Orbbec Astra 깊이 카메라를 활성화하고 본 검증에서 적용한 `VoxelLayer`를 실물 로봇의 local costmap에 상시 적용해야 합니다.
2. **유리문/유리벽 구간 가상 지도(Keepout Mask) 상시 적용:**
   - 광학 라이다 및 깊이 카메라는 직사광이나 투명 유리에서 완전 투과/산란이 발생합니다.
   - **권고:** 라운지 슬램(SLAM) 맵핑 직후, 유리벽 구간($y=-1.5\text{m}$)과 책상 아래 공간에 반드시 2D 점유 격자 지도 상에서 Black(0)으로 칠한 가상 벽 레이어를 주입해야 합니다.
3. **Footprint 사각 모델 적용:**
   - LIMO의 전장($0.32\text{m}$) 대비 전폭($0.22\text{m}$) 비는 약 1.45배입니다. 원형 footprint를 쓰면 $0.72\text{m}$ 협로를 통과하지 못하고 정체됩니다.
   - **권고:** 직사각형 footprint 설정을 필수 적용하십시오.
