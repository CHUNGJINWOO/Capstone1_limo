# LIMO Pro 라운지 시뮬레이션 2차 정밀 검증 및 결함 규명 보고서

> **검증 일시:** 2026-11-07  
> **검증 대상:** `experiments/lounge_failure_investigation_20261107/` 전반  
> **핵심 원칙:** 성공 판정 유지 배제, 물리 엔진 실측 Ground Truth와 ROS 토픽 간의 괴리 추적, "실패한 원인"과 "성공했다고 착각한 부분"의 전면 규명.

---

## 1. 종합 재검토 매트릭스 (4대 독립 평가 축)

기존의 단순 이진 판정(PASS/FAIL)을 폐기하고, **안전성(Safety), 장애물 회피(Avoidance), 경로 완주(Completion), Nav2 액션 상태(Action Status)**의 4개 독립 축으로 전면 재평가합니다.

| 시나리오 | 시험 조건 | 안전성 (Safety) | 장애물 회피 (Avoidance) | 경로 완주 (Completion) | Nav2 액션 상태 (Action) | 종합 재평가 및 실측 근거 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| **A (책상)** | **기본 (Baseline)** | ❌ **FAIL** | ❌ **FAIL** | ❌ **FAIL** | ❌ **RUNNING_TIMEOUT** | 이동 변위 $0.571\text{m}$ ($my=2.653\text{m}$). 책상 상판($Y=2.2\text{m}$)에 닿기 전 북측 의자($Y=2.35\text{m}$) 앞에서 정체. 최소거리 $0.137\text{m}$. |
| | **개선 (Improved)** | ✅ **PASS** | ✅ **PASS** | ❌ **FAIL** | ❌ **RUNNING_TIMEOUT** | 이동 변위 $0.276\text{m}$. 책상 하부 진입은 차단되었으나 우회로를 주행하지 못하고 스폰 직후 타임아웃 정지. |
| **B (의자)** | **기본 (Baseline)** | ⚠️ **CONDITIONAL** | ✅ **PASS** | ⚠️ **NEAR-COMPLETE** | ❌ **RUNNING_TIMEOUT** | 이동 변위 $2.880\text{m}$ ($my \approx 3.35\text{m}$). **의자($Y=2.0\text{m}$) 턱에 걸리지 않고 우회 통과함**. 목표($3.5\text{m}$) $15\text{cm}$ 앞 감속 중 30초 종료. |
| | **개선 (Improved)** | ✅ **PASS** | ✅ **PASS** | ⚠️ **NEAR-COMPLETE** | ❌ **RUNNING_TIMEOUT** | 이동 변위 $2.836\text{m}$. 기본 설정과 거의 동일한 궤적으로 통과 후 30초 종료. |
| **C (유리벽)** | **기본 (Baseline)** | ❌ **CRITICAL FAIL** | ❌ **FAIL** | ❌ **FALSE-PASS** | ⚠️ **SUCCEEDED (치명적 허위성공)** | **물리적 관통 없음**. $Y=-1.345\text{m}$에서 유리벽 충돌체에 저지됨. 바퀴 헛돌림(Wheel Slip)으로 오도메트리만 $3.43\text{m}$ 누적되어 Nav2 성공 오판. |
| | **개선 (Improved)** | ✅ **PASS** | ✅ **PASS** | ❌ **FAIL** | ❌ **RUNNING_TIMEOUT** | 이동 변위 $1.870\text{m}$ ($my=-0.07\text{m}, mx=-0.78\text{m}$). 유리벽 충돌은 막았으나 **출입문($X=-1.4\text{m}$)을 통과하지 못하고 타임아웃 종료**. |
| **D (협로)** | **기본 (Baseline)** | ✅ **PASS** | ⚠️ **OVER-AVOID** | ❌ **FAIL** | ❌ **RUNNING_TIMEOUT** | 이동 변위 $2.524\text{m}$. 원형 footprint($r=0.19\text{m}$) 과다 인플레이션으로 $0.72\text{m}$ 협로 입구 통과 불가. |
| | **개선 (Improved)** | ✅ **PASS** | ✅ **PASS** | ⚠️ **NEAR-COMPLETE** | ❌ **RUNNING_TIMEOUT** | 이동 변위 **$8.522\text{m}$** 달성. 직사각형 footprint로 협로 완벽 통과. 목표 $0.5\text{m}$ 전방에서 35초 시간 초과. |

---

## 2. 핵심 쟁점 1: 유리벽 "1.43m 관통"의 진실 규명

### 2.1 실제 물리 엔진 Ground-Truth 검증 결과
- **시험 스크립트:** [`scripts/investigate_scenario_c_ground_truth.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/scripts/investigate_scenario_c_ground_truth.py)
- **실측 로그:** [`logs/glass_penetration_ground_truth_audit.json`](file:///home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/logs/glass_penetration_ground_truth_audit.json)
- **시험 방법:** $X=1.0, Y=0.5$ (Yaw = $-1.5708$)에서 $Y=-1.50\text{m}$ 유리벽을 향해 $0.28\text{m/s}$로 14초간 직진 명령 인가. Ignition Gazebo의 Ground Truth 모델 포즈(`ign model -m limo -p`)와 ROS 2 `/odom`을 동시 기록.

```text
[실제 측정된 Ground Truth vs Odometry 시계열 데이터]
• t =  0.0s | Odom X = 0.000m (추정 my =  0.50m) | Gazebo GT Y = +0.500m (정상 전진)
• t =  4.5s | Odom X = 1.058m (추정 my = -0.56m) | Gazebo GT Y = -0.560m (정상 전진)
• t =  7.1s | Odom X = 1.495m (추정 my = -0.99m) | Gazebo GT Y = -1.000m (정상 전진)
• t =  8.0s | Odom X = 1.920m (추정 my = -1.42m) | Gazebo GT Y = -1.345m (유리벽 접촉 발생!)
• t =  8.9s | Odom X = 1.920m (추정 my = -1.42m) | Gazebo GT Y = -1.345m (물리적 정지 고정)
• t = 10.7s | Odom X = 2.357m (추정 my = -1.86m) | Gazebo GT Y = -1.345m (휠 슬립 발생)
• t = 12.4s | Odom X = 2.794m (추정 my = -2.29m) | Gazebo GT Y = -1.345m (휠 슬립 누적)
• t = 13.3s | Odom X = 3.225m (추정 my = -2.73m) | Gazebo GT Y = -1.345m (휠 슬립 누적)
• 최종 상태 | Odom X = 3.449m (추정 my = -2.95m) | Gazebo GT Y = -1.345m (물리 관통 0.000m!)
```

### 2.2 입증된 사실과 결론
1. **유리벽 충돌체는 물리 엔진에 완벽히 등록되어 작동함:**
   - 유리벽은 $Y=-1.50\text{m}$에 위치하며, 두께는 $0.02\text{m}$입니다. LIMO의 전방 범퍼($X_{center} + 0.15\text{m}$)가 $Y=-1.345 - 0.15 = -1.495\text{m}$에서 유리벽 충돌체와 맞닿아 **완전히 정지**했습니다.
   - **물리적 관통 깊이는 $0.000\text{m}$입니다.**
2. **"1.43m 관통"은 휠 슬립(Wheel Slip)에 의한 오도메트리 허상임:**
   - 2D LiDAR가 유리를 보지 못해 Nav2가 장애물로 등록하지 않았고, DWB는 전진 속도 명령(`cmd_vel`)을 계속 인가했습니다.
   - 모터가 회전하면서 타이어가 바닥에서 공회전했고, 구동륜 회전수를 적분하는 Ackermann Odometry 플러그인이 로봇이 전진하고 있는 것으로 착각하여 $X_{odom} = 3.449\text{m}$를 발행했습니다.
   - 이전 평가 스크립트(`run_real_lounge_navigation.py`)는 실제 Gazebo 좌표가 아닌 `/odom` 기반 추정치(`0.5 - odom_x = -2.95m`)를 계산하여 물리적 관통이 일어난 것으로 오판 보고했습니다.
3. **가장 위험한 실패는 물리적 관통이 아닌 "자율주행 스택의 허위 성공":**
   - 로봇이 실제로는 유리벽에 처박혀 5초 이상 헛바퀴를 돌고 있었음에도, Nav2는 오도메트리가 목표 공차에 들어왔다는 이유로 **`SUCCEEDED`를 선언**했습니다.
   - 이는 실물 로봇에서 모터 과부하 소손, 타이어 마모, 유리벽 파손으로 이어지는 치명적인 결함입니다.
4. **개선 설정의 "출입문 우회 성공" 착각:**
   - 개선 설정에서 로봇은 유리벽에 부딪히지 않고 멈췄으나, 최종 도달 좌표는 $mx=-0.78\text{m}, my=-0.07\text{m}$였습니다. 출입문 개구부($X \in [-2.5, -1.4]\text{m}$)까지 도달하지도 못한 채 30초 타임아웃으로 정지했습니다.
   - 따라서 **"출입문 안전 우회 통과"는 허위 판정이며, 실제로는 "목표 도달 실패(RUNNING_TIMEOUT)"**입니다.

---

## 3. 핵심 쟁점 2: 의자 스키드 레일(턱) 걸림의 진실 규명

### 3.1 실측치 재검토
- **의자 배치:** $X = 4.2\text{m}, Y = 2.0\text{m}$
- **로봇 시작/목표:** 시작 $(4.2, 0.5)$, 목표 $(4.2, 3.5)$ (북쪽으로 $+3.0\text{m}$ 직진 과제)
- **실측 이동 변위:** Baseline **$2.880\text{m}$**, Improved **$2.836\text{m}$**
- **최종 로봇 위치:** $my = 0.5 + 2.85\text{m} = \mathbf{3.35\text{m}}$!

### 3.2 입증된 사실과 결론
1. **기본 설정 로봇은 의자 턱에 걸리지 않았음:**
   - 만약 바닥 $35\text{mm}$ 턱에 차체가 걸려 휠 스핀이 일어났다면, 로봇의 실제 위치는 의자가 있는 $Y=2.0\text{m}$ (변위 $1.5\text{m}$)에 멈춰 있어야 합니다.
   - 그러나 최종 라이다 최소 거리는 **$0.933\text{m}$**였으며, 로봇은 의자를 $1.35\text{m}$나 지나쳐 목표 $15\text{cm}$ 앞($Y=3.35\text{m}$)에 도달해 있었습니다.
   - 로봇의 2D LiDAR는 바닥 턱($35\text{mm}$)은 보지 못했지만, **의자 다리 4개($\phi 24\text{mm}$, 높이 $0.37\text{m}$)를 명확히 감지**했습니다.
   - 다리 사이 간격($0.34\text{m}$)이 Nav2의 원형 치명 반경($0.38\text{m}$)보다 좁았기 때문에, 플래너는 의자 사이로 들어가지 않고 **우측으로 $0.4\text{m}$ 조향하여 의자를 안전하게 우회 통과**했습니다.
2. **타임아웃의 진짜 원인:**
   - 턱 걸림 정체가 아니라, 목표점 부근($0.15\text{m}$ 전방)에서 정밀 정지 및 목표 방향 정렬을 수행하던 중 30초 타임아웃 제한에 도달한 것입니다.
   - 따라서 **"기본 설정에서 의자 턱에 걸려 정체되었다"는 이전 보고서의 판정은 완전한 오판**이었습니다.

---

## 4. 핵심 쟁점 3: 3D Depth 카메라 및 VoxelLayer의 치명적 결함

### 4.1 TF 단절(Disconnection) 입증
- `depth_to_pointcloud_node.py`는 변환된 점군 메시지의 프레임을 다음과 같이 지정했습니다:
  ```python
  cloud_msg.header.frame_id = 'depth_link'
  ```
- 그러나 시스템 점검 결과:
  1. `limo_ackermann_fortress.urdf`: `depth_link` 없음.
  2. `models/limo_eval/model.sdf`: `depth_link` 없음 (`depth_camera_link`만 센서 태그 내부에 존재).
  3. `launch/*.launch.py`: `static_transform_publisher` 없음.
  4. TF 변환 확인: `buf.can_transform('base_link', 'depth_link') == False` (변환 불가!).

### 4.2 입증된 사실과 결론
- Nav2의 `voxel_layer`는 수신한 모든 `/camera/depth/points` 메시지를 **TF 변환 실패로 100% 드롭(Drop)**시켰습니다.
- 즉, **3D VoxelLayer는 단 한 번도 비용지도에 장애물 포인트를 등록한 적이 없습니다.**
- 이전 보고서에서 "3D VoxelLayer가 의자 턱과 책상 보강 바를 감지하여 개선되었다"고 주장한 것은 **소프트웨어 연결 검증 부재로 인한 심각한 착각**이었습니다.

---

## 5. 핵심 쟁점 4: 개선 설정의 공정성 및 혼재 변수 분석

개선 설정에서 충돌이 방지된 진짜 원인은 센서나 플래너의 개선이 아니라 **정적 지도 조작**이었습니다.

### 5.1 7대 설정 차이 분석

| 비교 항목 | 기본 설정 (Baseline) | 개선 설정 (Improved) | 공정성 평가 및 영향 분석 |
|:---|:---|:---|:---|
| **1. 정적 지도 (Map)** | `lounge_map.pgm` (개방형) | `lounge_map_improved.pgm` (벽 추가) | ❌ **불공정 (핵심 원인):** 책상 밑과 유리벽 전체를 사람이 수동으로 **검은색 점유 벽(0)**으로 칠해둠. 플래너는 센서가 아니라 정적 지도 벽을 보고 멈춘 것임. |
| **2. Footprint** | 원형 `robot_radius: 0.19m` | 직사각형 `[[0.16, 0.11], ...]` | ✅ **정당한 개선:** 협로($0.72\text{m}$) 통과를 가능하게 한 유일한 진짜 원인. |
| **3. 센서 파이프라인** | 2D LiDAR 단독 | LiDAR + Depth PointCloud | ❌ **무효:** `depth_link` TF 미등록으로 점군이 전량 드롭되어 영향력 제로. |
| **4. Inflation 설정** | $r=0.30\text{m}$, Scale $7.0$ | $r=0.38\text{m}$, Scale $5.0$ | ⚠️ **혼재 변수:** 반경을 키우고 감쇠를 완화하여 협로 비용에 복합 영향. |
| **5. DWB 파라미터** | $v_{max}=0.4$, Accel $1.5$ | $v_{max}=0.35$, Accel $1.2$ | ⚠️ **혼재 변수:** 감속 설정으로 인해 30초 내 목표 미도달 원인 제공. |
| **6. 월드/로봇 모델** | 동일 (`limo_eval`) | 동일 (`limo_eval`) | ✅ 공정하게 동일 조건 유지됨. |
| **7. 시작점/목표점/시간** | 동일 (30s / 35s) | 동일 (30s / 35s) | ✅ 공정하게 동일 조건 유지됨. |

---

## 6. 최종 결론 및 실물 LIMO 적용 전 필수 권고사항

### 6.1 실패와 착각의 최종 정리
1. **실제로 실패한 것:**
   - 투명 유리에 대한 2D LiDAR의 완전한 사각지대 (Raytrace Clearing으로 지도 소거).
   - 유리벽 충돌 중 휠 슬립 발생 시 Nav2가 성공(`SUCCEEDED`)으로 오판하는 안전 행동 결함.
   - 직사각형 로봇을 원형 Footprint로 근사하여 협로에서 진동 및 정체 발생.
   - 30초 타임아웃 제한으로 인한 모든 시나리오에서의 목표 도달 미완주.
2. **성공했다고 착각한 것:**
   - "유리벽을 물리적으로 1.43m 관통했다": **거짓** (물리 충돌체는 정상 저지함, 휠 슬립 오도메트리 허상임).
   - "개선 설정에서 출입문 우회 주행에 성공했다": **거짓** (문 앞에 가보지도 못하고 타임아웃 정지).
   - "기본 설정에서 의자 턱에 걸려 휠 스핀했다": **거짓** (의자 다리를 보고 우회하여 이미 $1.35\text{m}$ 지나침).
   - "3D VoxelLayer가 의자 턱과 보강 바를 감지했다": **거짓** (`depth_link` TF 단절로 점군 전량 폐기).
   - "개선 설정 주행이 센서 통합으로 개선되었다": **거짓** (사람이 지도에 가상 벽을 그려 넣은 효과였음).

### 6.2 실물 LIMO Pro 현장 배포 전 긴급 조치 요구사항
1. **[소프트웨어 긴급 수정] `depth_link` TF 브로드캐스터 구현:**
   - 실물 로봇 배포 전, `base_link` $\rightarrow$ `depth_link` 간의 Static Transform(`0.12, 0.0, 0.13`)을 반드시 발행해야 VoxelLayer가 작동합니다.
2. **[안전 제어] 휠 슬립 / 스톨 감지 비상 정지(E-STOP) 구현:**
   - 바퀴 엔코더는 회전하지만 IMU 가속도 또는 LiDAR Scan Matching 변위가 0일 경우, 즉시 모터를 차단하는 Stall Monitor가 필수적입니다 (유리벽 충돌 시 모터 소손 방지).
3. **[공정성 입증] 단일 변수 분리 시험(Ablation Test) 필수 수행:**
   - 가상 지도 수정 없이 오직 직사각형 Footprint와 실제 작동하는 VoxelLayer만으로 주행하는 독립 시험을 수행해야 합니다.
