# 라운지 주행 시험 실행 매뉴얼 (Test Manual)

이 문서는 라운지 시뮬레이션 환경에서 AgileX LIMO Pro의 4대 핵심 검증 시나리오를 실행하고, 센서/오도메트리/충돌 실측 로그를 수집 및 분석하는 상세 절차를 설명합니다.

---

## 1. 사전 환경 준비

Ubuntu 22.04 LTS 및 ROS 2 Humble 환경에서 아래 명령으로 환경 변수를 로드합니다:

```bash
# 기본 ROS 2 Humble 및 LIMO 패키지 환경 로드
source /opt/ros/humble/setup.bash
source /home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash

# 시험 디렉터리로 이동
cd /home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/scripts
```

---

## 2. 시험 시나리오 사양

| 시나리오 ID | 시나리오 명칭 | 시작 위치 $(x, y, \text{yaw})$ | 목표 위치 $(x, y, \text{yaw})$ | 적용 월드 | 검증 핵심 목적 |
|:---:|:---|:---:|:---:|:---|:---|
| **A** | 책상 반대편 목표 | $(-2.5, 3.2, -1.57)$ | $(-2.5, 0.4, -1.57)$ | `lounge_evaluation_world.sdf` | 책상 하부 진입 및 상부 보강대($z=0.21\text{m}$) 간섭 확인 |
| **B** | 의자 근접 통과 | $(4.2, 0.5, 1.57)$ | $(4.2, 3.5, 1.57)$ | `lounge_evaluation_world.sdf` | 바닥 근접 35mm 스키드 레일 턱 걸림 및 휠 슬립 확인 |
| **C** | 유리벽 너머 목표 | $(1.0, 0.5, -1.57)$ | $(1.0, -3.0, -1.57)$ | `lounge_evaluation_world_invisible_glass.sdf` | 투명 유리벽 관통 경로 생성 및 Nav2 허위 성공 오판 확인 |
| **D** | 복합 라운지 주행 | $(-4.5, 3.0, 0.0)$ | $(4.5, 3.0, 0.0)$ | `lounge_evaluation_world.sdf` | $0.72\text{m}$ 협로 통과 진동(Oscillation) 및 통과 여부 확인 |

---

## 3. 실제 물리 엔진 및 센서 검증 실행 (Live Physical Mode)

실제 Gazebo Fortress 및 Nav2 주행 스택을 직접 구동하여 실시간 ROS 2 토픽 실측치를 수집합니다.

### 3.1 센서 및 물리 모델 검증 (1~3단계)
```bash
# 1단계: 물리 모델 상부 충돌체(z=0.14~0.22m) 및 의자 턱 접촉 기하 검증
python3 verify_physical_contacts.py

# 2단계: Gazebo 백그라운드 기동, 센서 주기(/clock, /scan, /odom, /tf) 및 모터 구동 실측
python3 verify_gazebo_sensors.py

# 3단계: 유리벽 3조건(불투명/반투명/투명체 가정) 레이저 실측 비교
python3 verify_glass_sensors.py
```

### 3.2 Nav2 실제 주행 시험 (4단계)
```bash
# 기본 설정(Baseline) 전체 시나리오 실행 (원형 footprint, 2D 센서만 사용)
python3 run_real_lounge_navigation.py --scenario ALL

# 개선 설정(Improved) 전체 시나리오 실행 (직사각형 footprint, 3D VoxelLayer, Keepout Zone)
python3 run_real_lounge_navigation.py --scenario ALL --improved

# 특정 시나리오만 단독 실행할 경우 (예: 시나리오 C)
python3 run_real_lounge_navigation.py --scenario C
```

---

## 4. 고속 간이 검증 (Dry-run Mode)

물리 엔진 기동 오버헤드 없이 기하학적 OBB SAT 충돌 판정 및 파이프라인 무결성을 신속 점검할 때 사용합니다:

```bash
# 기본값 검증
python3 run_lounge_scenarios.py --dry-run

# 개선값 검증
python3 run_lounge_scenarios.py --dry-run --improved
```

---

## 5. 결과 로그 확인 및 보고서 자동 갱신

모든 실행 결과는 `logs/` 디렉터리에 타임스탬프와 함께 정량 JSON으로 저장됩니다:
- `logs/real_nav2_results_baseline_*.json` : 기본 설정 주행 실측 데이터
- `logs/real_nav2_results_improved_*.json` : 개선 설정 주행 실측 데이터
- `logs/step2_sensor_verification.json` : 센서 주기 및 주행 변위 실측치
- `logs/step3_glass_sensor_results.json` : 유리벽 3조건 센서 측정치

아래 명령어로 종합 분석 보고서를 최신 실측 데이터로 즉시 재생성할 수 있습니다:
```bash
python3 generate_failure_report.py
cat ../docs/FAILURE_ANALYSIS_REPORT.md
```
