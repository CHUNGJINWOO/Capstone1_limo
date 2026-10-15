# LIMO를 이용한 실내 자율 순찰 및 안내

2026-2 캡스톤디자인 프로젝트

**AgileX LIMO Pro**를 활용하여 실내 환경에서 자율주행, 순찰 및 목적지 안내가 가능한 ROS 2 기반 로봇 시스템을 개발합니다.

---

## 1. 프로젝트 소개

### 프로젝트명

**LIMO를 이용한 실내 자율 순찰 및 안내**

### 프로젝트 목적

본 프로젝트는 AgileX LIMO Pro를 기반으로 실내 환경에서 안정적인 자율주행 시스템을 구축하고, 이를 활용하여 **자율 순찰 및 목적지 안내 서비스**를 구현하는 것을 목표로 합니다.

이를 위해 LiDAR를 이용한 실내 지도 작성과 위치 추정, Navigation2 기반 경로 계획 및 주행, 동적 장애물 대응 등의 기능을 단계적으로 구현하고 실제 주행 환경에서 검증합니다.

최종적으로는 복도 및 로비와 같은 실내 환경에서 LIMO가 지정된 경로를 따라 자율적으로 순찰하고, 사용자의 호출에 따라 목적지까지 안내할 수 있는 시스템을 구축하는 것을 목표로 합니다.

---

## 2. 주요 개발 범위

본 프로젝트의 주요 개발 범위는 다음과 같습니다.

- LiDAR 기반 실내 환경 인식
- SLAM 기반 실내 지도 작성
- AMCL 기반 로봇 위치 추정
- Navigation2(Nav2) 기반 자율주행
- Global / Local Path Planning
- Costmap 구성 및 활용
- 경로 추종
- 정적 및 동적 장애물 대응
- 동적 장애물 회피
- Waypoint 기반 자율 순찰
- ROS 2 Action 기반 순찰 로직
- 목적지 안내 기능
- 실제 주행 환경에서의 파라미터 튜닝
- 주행 성능 및 기능 검증

---

## 3. 시스템 구성

전체 시스템은 다음과 같은 흐름으로 구성합니다.

```text
                    AgileX LIMO Pro
                           │
             ┌─────────────┼─────────────┐
             │             │             │
           LiDAR          IMU         Odometry
             │             │             │
             └─────────────┼─────────────┘
                           │
                           ▼
                    센서 데이터 처리
                           │
              ┌────────────┴────────────┐
              │                         │
              ▼                         ▼
          SLAM Mapping              Localization
              │                       (AMCL)
              │                         │
              └────────────┬────────────┘
                           ▼
                    Navigation2
                           │
          ┌────────────────┼────────────────┐
          │                │                │
          ▼                ▼                ▼
    Global Planner    Local Planner      Costmap
          │                │                │
          └────────────────┼────────────────┘
                           ▼
                      경로 추종
                           │
                           ▼
                    장애물 대응
                           │
                           ▼
                         LIMO
                           │
              ┌────────────┴────────────┐
              ▼                         ▼
          자율 순찰                    목적지 안내
```

위 구성은 프로젝트의 최종 목표 시스템이며, 각 기능은 실제 구현 및 검증을 거쳐 단계적으로 통합합니다.

---

## 4. 저장소 구조

저장소의 주요 디렉터리 구성은 다음과 같습니다.

```text
├── docs/
│   ├── DEVELOPMENT_LOG.md     # 시스템 점검 이력 및 일자별 개발 기록
│   └── PROJECT_PLAN.md        # 프로젝트 전체 개발 계획 및 단계별 목표
├── scripts/
│   └── remote_teleop_node.py  # 원격 조향 제어 퍼블리셔 노드
└── src/
    └── limo_ros2/
        ├── limo_base/         # LIMO 통신 및 기본 구동 드라이버
        ├── limo_car/          # 로봇 모델 및 시뮬레이션 패키지
        ├── limo_description/  # URDF / Xacro 및 시각화 리소스
        └── limo_msgs/         # 커스텀 ROS 2 메시지 정의
```

### `docs`

- `DEVELOPMENT_LOG.md`: LIMO 온보드 환경에서의 하드웨어 연결, 센서 검증, SLAM 매핑, Nav2 연동 및 문제 해결 과정을 일자별로 상세히 기록합니다.
- `PROJECT_PLAN.md`: 캡스톤 디자인 목표, 개발 단계(Phase 1~9), 팀원별 역할 분담 및 최종 평가 항목을 정의합니다.

### `scripts`

- `remote_teleop_node.py`: ROS 2 `geometry_msgs/msg/Twist` 표준 인터페이스를 사용하여 LIMO의 속도 명령(`/cmd_vel`)을 발행하는 조향 제어 스크립트입니다.

### `src/limo_ros2`

LIMO Pro 구동을 위한 핵심 ROS 2 패키지입니다.

- **`limo_base`**: 하부 MCU와의 시리얼 통신, 기본 주행 제어 노드, 센서 및 TF 브로드캐스팅, LiDAR 실행 런치 파일을 포함합니다.
- **`limo_car`**: LIMO 모델(Ackermann 등), Gazebo 시뮬레이션 환경, 센서 모델, 시뮬레이션 런치 파일을 포함합니다.
- **`limo_description`**: 로봇 형상 URDF/Xacro, 3D 메쉬, Gazebo 설명 파일 및 RViz 시각화 설정을 포함합니다.
- **`limo_msgs`**: LIMO 시스템 전용 커스텀 ROS 2 메시지 정의를 포함합니다.

---

# 5. 구현 과정 및 빌드 가이드

본 프로젝트에서는 기능을 한 번에 통합하지 않고, **기본 소스 확인 → 센서 및 TF 확인 → 지도 작성 → 위치 추정 → Navigation2 → 장애물 대응 → 순찰 및 안내**의 순서로 단계적으로 개발합니다.

---

## 5.1 LIMO ROS 2 기본 소스 및 구현 상태

프로젝트는 AgileX에서 제공하는 LIMO ROS 2 소스를 기반으로 구성하며, 현재 저장소의 구현 및 준비 상태는 다음과 같습니다.

| 구분 | 대상 | 현재 상태 | 상세 설명 |
|---|---|:---:|---|
| **기본 패키지** | `limo_msgs`, `limo_description`, `limo_base`, `limo_car` | **완료** | `colcon build --symlink-install` 전체 빌드 성공 |
| **제어 스크립트** | `scripts/remote_teleop_node.py` | **완료** | ROS 2 표준 `geometry_msgs/Twist` 기반 조향 퍼블리셔 |
| **로봇 시각화** | URDF, Xacro, RViz2 런치 | **완료** | 차동 구동 및 Ackermann 조향 모델 시각화 확인 |
| **실내 SLAM** | SLAM Toolbox / 2D 지도 작성 | **진행 중** | LIMO 온보드 현장 매핑 검증 진행 중, 저장소 패키지 통합 예정 |
| **위치 추정 (AMCL)** | Nav2 AMCL 파라미터 튜닝 | **계획** | 실내 지도 구축 완료 후 단계적 적용 |
| **자율주행 (Nav2)** | Global / Local Path Planning, Costmap | **계획** | 실내 복도 및 로비 주행 파이프라인 구성 예정 |
| **동적 장애물 회피** | Costmap 기반 동적 장애물 회피 | **계획** | 보행자 및 장애물 회피 실험 및 파라미터 최적화 예정 |
| **순찰 / 안내** | Action 기반 Waypoint 순찰 및 안내 | **계획** | 사용자 호출 및 목적지 안내 애플리케이션 개발 예정 |

---

## 5.2 빌드 환경 및 방법

### 개발 환경

- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **빌드 도구**: `colcon`

### 빌드 명령어

저장소 루트 디렉터리에서 아래 명령어를 순차적으로 실행하여 패키지를 빌드하고 환경을 설정합니다.

```bash
# 1. ROS 2 Humble 기본 환경 로드
source /opt/ros/humble/setup.bash

# 2. 전체 패키지 빌드 (심볼릭 링크 설치)
colcon build --symlink-install

# 3. 빌드 결과물 환경 오버레이
source install/setup.bash
```

> [!NOTE]
> `src/limo_ros2/limo_car/CMakeLists.txt`에서 저장소에 존재하지 않는 설치 디렉터리(`log`, `src`, `worlds`) 참조로 인해 발생하던 `ament_cmake_symlink_install_directory()` 오류를 수정한 후(`commit: 8c3d4f3`), 현재 4개 패키지 모두 성공적으로 빌드됩니다. 빌드 상세 기록은 [docs/DEVELOPMENT_LOG.md](docs/DEVELOPMENT_LOG.md)에서 확인할 수 있습니다.

---

## 5.3 현재 실행 가능한 기능 및 명령

현재 저장소 상태에서 즉시 실행 가능한 주요 노드 및 런치 명령입니다.

### 1) LIMO Base 하드웨어 구동 (온보드 환경)

LIMO onboard PC에서 하부 MCU와 시리얼 통신을 연결하고 오도메트리(`odom`) 및 좌표계(`base_link`)를 브로드캐스팅합니다.

```bash
ros2 launch limo_base limo_base.launch.py port_name:=ttyUSB1
```
*(장치 연결 상태에 따라 `/dev/ttyUSB0` 또는 `/dev/ttyUSB1` 포트 지정)*

### 2) 원격 조향 제어 스크립트 실행

`geometry_msgs/msg/Twist` 표준 메시지 인터페이스를 통해 LIMO에 주행 속도 명령(`/cmd_vel`)을 발행합니다.

```bash
python3 scripts/remote_teleop_node.py
```

### 3) 로봇 모델 RViz2 시각화

LIMO의 URDF/Xacro 형상 및 센서 좌표계를 3D로 시각화합니다.

```bash
# 차동 구동 모델 시각화
ros2 launch limo_description display_models_diff.launch.py

# 또는 Ackermann 조향 모델 시각화
ros2 launch limo_car display_ackermann.launch.py
```

> [!NOTE]
> 실내 지도 작성(SLAM) 노드 및 Nav2 자율주행 런치 파일은 온보드 현장 주행 검증을 거쳐 향후 단계별로 저장소에 통합될 예정입니다.

---

# 6. 자율주행 구현

## 6.1 LiDAR 데이터 확인

실제 LIMO에서 LiDAR 데이터를 정상적으로 수신할 수 있는지 확인합니다.

확인 항목:

- LiDAR Node
- LaserScan 데이터
- `/scan` Topic
- RViz2에서 LaserScan 표시
- 센서 좌표계 및 TF

실제 하드웨어 연결 후 데이터를 확인하고 결과를 기록합니다.

---

## 6.2 TF 및 Odometry 확인

자율주행을 위해서는 센서와 로봇의 좌표계 관계가 정상적으로 구성되어야 합니다.

주요 확인 항목:

```text
map
 └── odom
      └── base_link
           └── laser
```

확인 항목:

- TF Tree
- Odometry
- `base_link`
- LiDAR Frame
- 좌표계 방향
- 주행 중 TF 변화

문제가 발생할 경우 TF Tree와 Topic 데이터를 기반으로 원인을 분석합니다.

---

## 6.3 SLAM Mapping

LiDAR 센서 데이터를 이용하여 실제 실내 환경의 지도를 작성합니다.

### 목표

- 복도
- 로비
- 주요 이동 구역

등의 환경을 지도에 반영합니다.

### 구현 과정

```text
LiDAR
  ↓
LaserScan
  ↓
SLAM Toolbox
  ↓
지도 작성
  ↓
Map 저장
```

### 검증

- 지도 생성 상태
- 벽 및 복도 형태의 표현
- 지도 왜곡 여부
- 주행 중 위치 변화
- Loop Closure 상태

등을 확인합니다.

---

# 7. Localization 구현

## 7.1 AMCL

SLAM을 통해 작성한 정적 지도를 이용하여 로봇의 현재 위치를 추정합니다.

### 구현 목표

- Map 기반 위치 추정
- 초기 위치 설정
- 주행 중 위치 추정
- 위치 추정 안정성 확인

### 주요 확인 항목

- AMCL Pose
- Particle 분포
- Map ↔ Odom 관계
- 초기 위치 설정
- 주행 중 위치 오차

---

## 7.2 AMCL 파라미터 튜닝

실제 환경에서는 센서 특성 및 주행 환경에 따라 Localization 성능이 달라질 수 있으므로 파라미터를 단계적으로 조정합니다.

주요 검토 항목:

- Sensor Model
- Motion Model
- Particle 수
- Update Threshold
- Laser 관련 파라미터

파라미터 변경 전후의 위치 추정 상태를 비교하여 실제 환경에 적합한 설정을 찾습니다.

---

# 8. Navigation2 구현

Navigation2를 이용하여 목표 위치까지의 경로 계획과 주행을 구현합니다.

전체 흐름은 다음과 같습니다.

```text
현재 위치
   ↓
목표 위치
   ↓
Global Planner
   ↓
Global Path
   ↓
Local Planner / Controller
   ↓
Local Path
   ↓
LIMO 주행
```

---

## 8.1 Global Planning

Global Planner를 이용하여 지도상에서 현재 위치부터 목표 위치까지의 전체 경로를 생성합니다.

확인 항목:

- 경로 생성
- 장애물 반영
- 목표 위치 변경
- 경로 재계획

---

## 8.2 Local Planning 및 경로 추종

Global Path를 기반으로 실제 로봇이 주행할 Local Path 및 제어 명령을 생성합니다.

확인 항목:

- 경로 추종
- 회전 구간
- 복도 주행
- 속도 제어
- 목표 위치 접근

실제 LIMO의 주행 특성에 맞게 관련 파라미터를 조정합니다.

---

# 9. 동적 장애물 회피

본 프로젝트의 주요 개발 항목 중 하나입니다.

실내 환경에서는 사람과 같은 이동 물체가 경로에 진입할 수 있으므로 정적인 지도만 이용하는 자율주행으로는 충분하지 않습니다.

### 목표

- 동적 장애물 인식
- Costmap 반영
- 주행 중 경로 변경
- 충돌 방지
- 안전한 회피 주행

### 기본 처리 흐름

```text
LiDAR / Sensor
      ↓
장애물 정보
      ↓
Local Costmap
      ↓
경로 재계획
      ↓
회피 주행
```

실제 복도 및 로비 환경에서 장애물을 배치하고 주행 테스트를 수행하여 회피 성능을 확인합니다.

---

# 10. 자율 순찰

정해진 Waypoint를 순서대로 방문하는 자율 순찰 기능을 구현합니다.

```text
Waypoint 1
    ↓
Waypoint 2
    ↓
Waypoint 3
    ↓
Waypoint 4
    ↓
Waypoint 1
    ↺
```

### 개발 항목

- 순찰 지점 설정
- Waypoint 순서 관리
- ROS 2 Action 기반 목표 전달
- 목표 도착 확인
- 다음 Waypoint 전환
- 순찰 반복

실제 순찰 경로를 구성한 후 전체 경로의 주행 성공 여부를 검증합니다.

---

# 11. 목적지 안내

사용자가 지정한 목적지까지 LIMO가 이동하는 안내 기능을 구현합니다.

### 기본 흐름

```text
사용자 호출
    ↓
목적지 선택
    ↓
Navigation Goal
    ↓
경로 계획
    ↓
장애물 대응
    ↓
목적지 도착
```

안내 기능은 자율주행 시스템과 통합하여 실제 실내 환경에서 검증합니다.

---

# 12. 문제 해결 및 Troubleshooting

개발 과정에서 발생한 문제는 단순히 수정하는 데 그치지 않고, **문제 → 원인 분석 → 해결 → 검증**의 체계로 분석하여 기록합니다.

```text
문제 발생
   ↓
증상 확인 및 로그 수집
   ↓
관련 Topic / Node / TF 상태 확인
   ↓
원인 분석 및 가설 수립
   ↓
코드 또는 파라미터 최소 수정
   ↓
재실행 및 비교 검증
   ↓
개발 로그 기록 및 미해결 과제 분류
```

실제 하드웨어 구동 및 자율 매핑 과정에서 발생한 주요 트러블슈팅(USB 시리얼 포트 충돌, RViz `/scan` QoS 비호환, 런치 파일 static TF 실행 순서 지연, 자율 탐색 목표 진동 등)의 상세 내역은 [docs/DEVELOPMENT_LOG.md](docs/DEVELOPMENT_LOG.md)에서 확인할 수 있습니다.

---

# 13. 실험 및 검증

구현한 기능은 실제 주행 환경에서 단계적으로 검증합니다.

## 평가 항목

### Localization

- 위치 추정 안정성
- 초기 위치 설정
- 주행 중 위치 오차

### Navigation

- 목표 위치 도착 여부
- 경로 추종 성능
- 주행 시간
- 경로 이탈 여부

### 장애물 회피

- 장애물 인식 여부
- 회피 성공 여부
- 충돌 여부
- 경로 재계획 여부

### Patrol

- Waypoint 도착 성공률
- 전체 순찰 성공 여부
- 반복 순찰 가능 여부

### Guide

- 목적지 도착 여부
- 안내 과정의 안정성
- 장애물 발생 시 대응 여부

실험 결과는 개발이 진행됨에 따라 측정값과 테스트 조건을 지속적으로 갱신합니다.

---

# 14. 개발 기록

프로젝트의 실제 개발 과정과 문제 해결 과정은 별도의 상세 문서에서 관리합니다.

- **[docs/DEVELOPMENT_LOG.md](docs/DEVELOPMENT_LOG.md)**: 실제 하드웨어 센서 검증, SLAM 매핑, Nav2 연동, 오류 분석 및 현장 검증 기록
- **[docs/PROJECT_PLAN.md](docs/PROJECT_PLAN.md)**: 전체 개발 단계(Phase 1~9), 팀원별 역할 분담 및 평가 기준 정의

---

# 15. 팀 역할

| 담당 | 주요 업무 |
|---|---|
| 정진우 | 자율주행 및 동적 장애물 회피 |
| 배민혁 | 실내 SLAM 맵핑 |
| 한동훈 | 시스템 구축 및 센서 융합 |
| 김정민 | 순찰·안내 애플리케이션 및 검증 |

### 정진우 — 자율주행 및 동적 장애물 회피

- Nav2 기반 AMCL 위치 추정 및 파라미터 튜닝
- 복도 및 로비 주행을 위한 Global / Local Path Planning
- 동적 장애물 인식 및 회피 알고리즘 구현
- 실제 주행 환경에서 주행 파라미터 최적화

### 배민혁 — 실내 SLAM 맵핑

- 실내 2D/3D 점유격자지도 작성
- 센서 노이즈 및 맵핑 오차 보정
- 센서 데이터 정합성 검증

### 한동훈 — 시스템 구축 및 센서 융합

- ROS 2 개발환경 구축 및 관리
- LIMO Pro 드라이버 및 하드웨어 제어
- LiDAR / Depth Camera / IMU 데이터 수집
- 센서 좌표계 및 데이터 정합성 확인
- 시스템 통합

### 김정민 — 순찰·안내 애플리케이션 및 검증

- ROS 2 Action 기반 Waypoint 순찰 로직 개발
- 사용자 호출 기반 안내 기능 구현
- 서비스 시나리오 필드 테스트
- 테스트 결과 데이터 수집 및 기능 검증

위 역할은 캡스톤디자인 팀별 계획서의 역할 분담을 기준으로 작성했습니다.

---

# 16. 향후 개발 방향

프로젝트는 다음 순서로 통합합니다.

```text
LIMO 기본 동작
      ↓
센서 및 TF 확인
      ↓
SLAM Mapping
      ↓
AMCL Localization
      ↓
Navigation2
      ↓
경로 추종
      ↓
동적 장애물 회피
      ↓
자율 순찰
      ↓
목적지 안내
      ↓
통합 테스트
      ↓
파라미터 최적화
      ↓
최종 시연 및 평가
```

최종적으로는 실제 실내 환경에서 LIMO가 안정적으로 자율주행하면서 순찰 및 안내 서비스를 수행할 수 있도록 시스템을 통합하고 검증합니다.

---

# 17. 참고자료 및 출처

본 프로젝트의 개발 및 구현 과정에서는 다음 자료를 우선적으로 참고합니다.

1. **WeGo, 「Limo_ROS2 교재」**
2. **AgileX Robotics, LIMO 공식 자료**
3. **ROS 2 공식 문서**
4. **Navigation2 공식 문서**
5. **SLAM Toolbox 공식 문서**
6. **2026-2 캡스톤디자인 팀별계획서**
7. 프로젝트 개발 과정에서 확인한 관련 기술 문서 및 커뮤니티 자료

---

## 18. 프로젝트 산출물

최종적으로 다음 결과물을 구축하고 정리합니다.

- ROS 2 기반 LIMO 자율주행 소스 코드
- 커스텀 Node 및 Launch 파일
- Navigation2 설정 및 Parameter 파일
- 실내 지도 데이터
- 자율 순찰 기능
- 목적지 안내 기능
- 동적 장애물 대응 기능
- 주행 테스트 및 성능 평가 결과
- 캡스톤디자인 최종 결과 보고서
- 시연 영상
- 발표 자료
