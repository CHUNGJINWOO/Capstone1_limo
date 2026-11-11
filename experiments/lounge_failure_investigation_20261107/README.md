# LIMO Pro 라운지 환경 주행 실패 재현 및 원인 분석 프레임워크

이 저장소는 AgileX LIMO Pro의 자율주행 및 자동 맵핑 스택 사전 검증을 위해, 실제 라운지 환경의 잠재적 위험 요소를 모델링하고 **"억지 성공(Success-forcing)"이 아닌 "실패 재현과 원인 분석"**을 체계적으로 수행하는 실측 기반 검증 패키지입니다.

> **2차 정밀 감사 원칙 준수:**
> - 단순 이진(PASS/FAIL) 판정을 지양하고, **안전성(Safety), 장애물 회피(Avoidance), 경로 완주(Completion), Nav2 액션 성공(Action Status)**의 4대 축으로 독립 평가.
> - 물리 엔진 실측 Ground Truth와 ROS 2 오도메트리 토픽 간의 괴리를 추적하여 **"실제로 실패한 원인"과 "성공했다고 착각한 부분"**을 전면 규명.
> - 기존 `Capstone1_limo` 기본 소스의 무결성 보존 및 시험 전용 파일 내 격리. Git 커밋/푸시 금지.

---

## 1. 디렉터리 구성

```text
experiments/lounge_failure_investigation_20261107/
├── README.md                      # 패키지 총괄 개요 및 실측 결과 안내
├── models/                        # 라운지 전용 Collision 및 로봇 평가 모델
│   ├── limo_eval/                 # LIMO 상부 충돌체(LiDAR/카메라 타워 z=0.22m) 및 Depth 카메라 센서 SDF
│   ├── lounge_table/              # 상판, 다리 4개, 하부 보강 바(z=0.21m) 포함
│   ├── lounge_chair_sled/         # 바닥 근접 스키드 레일(z=0.035m, 35mm 턱) 포함
│   └── glass_partition/          # 물리 충돌 차단 + 광학 투과형 유리벽/문
├── worlds/                        # 시뮬레이션 월드 및 필터 마스크
│   ├── lounge_evaluation_world.sdf                # 라운지 기본 평가 월드 (12m x 9m)
│   ├── lounge_evaluation_world_invisible_glass.sdf# 광학 투과(LiDAR 미검출) 유리벽 재현 월드
│   ├── lounge_keepout_mask.pgm                    # 책상 하부 진입 금지 Keepout 마스크 이미지
│   └── lounge_keepout_mask.yaml                   # Nav2 Keepout Filter 설정 파일
├── maps/                          # Nav2 주행용 정적 지도
│   ├── lounge_map.pgm / .yaml                     # 기본 원본 지도 (개방형)
│   └── lounge_map_improved.pgm / .yaml            # 가상 벽(Virtual Wall) 및 책상 금지구역 주입 지도
├── config/                        # Nav2 파라미터 구성
│   ├── nav2_lounge_baseline.yaml  # 실패 재현용 기본 파라미터 (원형 반경, 2D 센서 단독)
│   └── nav2_lounge_improved.yaml  # 대조군용 개선 파라미터 (직사각형 Footprint, 3D VoxelLayer)
├── scripts/                       # 실행 및 분석 스크립트
│   ├── verify_physical_contacts.py# [1단계] 물리 충돌체 기하학적 접촉/간섭 검증기
│   ├── verify_gazebo_sensors.py   # [2단계] Gazebo 토픽 주기(Hz) 및 로봇 이동성 실측기
│   ├── verify_glass_sensors.py    # [3단계] 유리벽 3조건(불투명/반투명/투명) 레이저 실측기
│   ├── run_real_lounge_navigation.py # [4단계] Nav2 실제 주행 실행 및 텔레메트리 수집기
│   ├── depth_to_pointcloud_node.py# [5단계] 32FC1 깊이 이미지 -> PointCloud2 변환 노드
│   ├── investigate_scenario_c_ground_truth.py # [2차 감사] 유리벽 Ground Truth vs Odom 실측기
│   ├── generate_failure_report.py # [6단계] 실측 JSON 로그 기반 종합 분석 보고서 생성기
│   ├── run_lounge_scenarios.py    # 기하학적 Dry-run 간이 검증기
│   └── generate_lounge_world.py   # 파라미터화 라운지 월드/모델 SDF 자동 생성기
├── logs/                          # 실제 물리 실행 실측 JSON 로그
│   ├── step2_sensor_verification.json             # 센서 주기(1000Hz clock, 10Hz scan) 및 변위(0.748m)
│   ├── step3_glass_sensor_results.json            # 유리 투과 거리(1.372m, 1.387m, 4.797m) 실측
│   ├── glass_penetration_ground_truth_audit.json  # [2차 감사] 유리벽 GT Y=-1.345m vs Odom 3.449m 실측
│   ├── real_nav2_results_baseline_20261107_185006.json # 기본 설정 4개 시나리오 실제 주행 로그
│   └── real_nav2_results_improved_20261107_185705.json # 개선 설정 4개 시나리오 실제 주행 로그
└── docs/                          # 상세 기술 문서
    ├── SECOND_STAGE_VERIFICATION_REPORT.md # [2차 감사] 4대 축 재평가 및 결함 규명 최종 보고서
    ├── REAL_EXECUTION_AUDIT_REPORT.md      # 실제 Gazebo/Nav2 실행 및 물리 측정 종합 보고서
    ├── FAILURE_ANALYSIS_REPORT.md          # 실패 지점 및 원인 분석 종합 보고서 (실측치 기반)
    ├── ENVIRONMENT_GUIDE.md                # 환경 구성 및 치수 사양서
    └── TEST_MANUAL.md                      # 시험 실행 및 디버깅 매뉴얼
```

---

## 2. 4대 핵심 검증 시나리오 및 실측 결과 종합 (4대 축 재평가)

| 시나리오 | 조건 | 안전성 | 장애물 회피 | 경로 완주 | Nav2 액션 | 이동 변위 | 최소거리 | 실측 사실 및 판정 근거 요약 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| **A (책상)** | **기본** | ❌ FAIL | ❌ FAIL | ❌ FAIL | `TIMEOUT` | `0.571m` | `0.137m` | 책상 하부 진입 시도, 북측 의자($Y=2.35\text{m}$) 앞에서 정체. |
| | **개선** | ✅ PASS | ✅ PASS | ❌ FAIL | `TIMEOUT` | `0.276m` | `0.330m` | 지도 가상벽으로 진입 차단 성공, 단 우회 주행 미완주. |
| **B (의자)** | **기본** | ⚠️ COND | ✅ PASS | ⚠️ NEAR | `TIMEOUT` | `2.880m` | `0.267m` | **의자($Y=2.0\text{m}$) 턱 걸림 없음**. 의자 다리 우회하여 $Y=3.35\text{m}$ 통과. |
| | **개선** | ✅ PASS | ✅ PASS | ⚠️ NEAR | `TIMEOUT` | `2.836m` | `0.334m` | 의자 측면 우회 통과, 목표 $0.16\text{m}$ 전방 감속 중 30초 종료. |
| **C (유리벽)** | **기본** | ❌ CRIT | ❌ FAIL | ❌ FALSE | `SUCCEEDED` | `3.433m` | `0.675m` | **물리 관통 0m**($Y=-1.345\text{m}$ 저지). 휠슬립 누적 오도로 **허위 성공**. |
| | **개선** | ✅ PASS | ✅ PASS | ❌ FAIL | `TIMEOUT` | `1.870m` | `0.675m` | 가상벽으로 충돌 차단, 출입문 도달 전 타임아웃 종료. |
| **D (협로)** | **기본** | ✅ PASS | ⚠️ OVER | ❌ FAIL | `TIMEOUT` | `2.524m` | `0.239m` | 원형 footprint 과다 인플레이션으로 $0.72\text{m}$ 협로 입구 정체. |
| | **개선** | ✅ PASS | ✅ PASS | ⚠️ NEAR | `TIMEOUT` | **`8.522m`** | `0.328m` | 직사각형 footprint로 협로 완벽 통과, 목표 $0.5\text{m}$ 전방 타임아웃. |

---

## 3. 2차 검증을 통해 밝혀진 핵심 사실 및 성공 착각(Illusion) 규명

1. **유리벽 "1.43m 관통"은 물리 관통이 아닌 휠 슬립(Wheel Slip) 오도메트리 허상:**
   - Ground Truth 실측 결과, 로봇은 $Y = -1.345\text{m}$에서 유리벽 충돌체에 의해 물리적으로 완전히 멈춰 섰습니다 (**물리적 관통 $0.000\text{m}$**).
   - 2D LiDAR의 투과로 벽체가 지워진 상태에서 DWB가 계속 전진 명령을 인가하여 바퀴가 바닥에서 헛돌았고, 헛바퀴 회전수가 누적되어 오도메트리가 $3.43\text{m}$까지 전진했습니다.
   - Nav2는 이 헛바퀴 오도메트리만 보고 목표에 도달했다고 오판하여 **치명적인 허위 성공(`SUCCEEDED`)**을 발생시켰습니다.
2. **개선 설정의 "출입문 우회 성공" 착각:**
   - 개선 설정에서 로봇은 유리벽에 부딪히지는 않았으나, 출입문($X \in [-2.5, -1.4]\text{m}$)에 가보지도 못하고 $X=-0.78\text{m}, Y=-0.07\text{m}$에서 30초 타임아웃으로 정지했습니다.
3. **의자 하부 턱 걸림 가설 기각:**
   - 기본 설정 로봇은 의자($Y=2.0\text{m}$)에 걸린 적이 없으며, 2D LiDAR로 의자 다리를 감지하고 우측으로 $0.4\text{m}$ 조향하여 $Y=3.35\text{m}$까지 정상 통과했습니다.
4. **3D VoxelLayer 소프트웨어 결함 (TF 미등록으로 점군 100% 드롭):**
   - `depth_to_pointcloud_node.py`가 발행하는 `'depth_link'`가 URDF/SDF/TF 트리에 존재하지 않아, Nav2 `VoxelLayer`는 수신한 모든 점군을 TF Lookup 실패로 100% 폐기하고 있었습니다.
   - 즉, 개선 설정의 회피 동작은 3D VoxelLayer가 아니라 **정적 지도(`lounge_map_improved.pgm`)에 수동으로 그려 넣은 검은색 가상 벽**에 의한 것이었습니다.
5. **Footprint 직사각형화의 실제 효용 입증:**
   - $0.72\text{m}$ 협로를 통과시켜 $8.522\text{m}$ 완주를 이끈 유일한 실제 알고리즘 개선은 직사각형 Footprint 모델링이었습니다.

---

## 4. 실물 LIMO Pro 현장 배포 전 긴급 조치 요구사항

1. **`depth_link` TF 브로드캐스터 구현:** `base_link` $\rightarrow$ `depth_link` Static Transform 필수 발행.
2. **휠 슬립 / 스톨 감지 비상 정지(E-STOP) 구현:** 바퀴 회전 대비 위치 변위 없을 시 3초 이내 모터 차단.
3. **단일 변수 분리 시험(Ablation Test) 수행:** 가상 지도 조작 없이 순수 직사각형 Footprint와 작동하는 VoxelLayer만으로 주행성 입증.
