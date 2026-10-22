# 1~8주차 주차별 개발 기록 (Weekly Progress)

본 문서는 2026학년도 2학기 캡스톤디자인 프로젝트 **"LIMO를 이용한 실내 자율순찰 및 안내"**의 1주차부터 8주차까지의 개발 진행 기록입니다.
주요 담당 분야인 **자율주행, Navigation2(Nav2), TF 좌표계, Costmap, 위치추정(AMCL), 그리고 동적 장애물 회피**를 중심으로 실제 개발 과정에서 발생한 기술적 문제, 원인 분석, 소스 코드 수정, 그리고 실차/시뮬레이션 검증 결과를 체계적으로 정리하였습니다.

---

## 목차
- [1주차: 과제 선정 및 역할 분담](#1주차-과제-선정-및-역할-분담)
- [2주차: 개발 범위 및 기술 검토](#2주차-개발-범위-및-기술-검토)
- [3주차: LIMO 구동 및 통신 환경 구축](#3주차-limo-구동-및-통신-환경-구축)
- [4주차: LiDAR 연동 및 수동 SLAM 맵핑](#4주차-lidar-연동-및-수동-slam-맵핑)
- [5주차: 맵핑 확장 및 하드웨어·주행 문제 분석](#5주차-맵핑-확장-및-하드웨어주행-문제-분석)
- [6주차: 자율 탐색 맵핑 시도 및 3대 핵심 결함 도출](#6주차-자율-탐색-맵핑-시도-및-3대-핵심-결함-도출)
- [7주차: 패키지 빌드 오류 및 TF·탐색 알고리즘 수정](#7주차-패키지-빌드-오류-및-tf탐색-알고리즘-수정)
- [8주차: Nav2 지도 수신 해결, TF 트리 계측 및 실차 주행 검증](#8주차-nav2-지도-수신-해결-tf-트리-계측-및-실차-주행-검증)
- [8주차 이후 개발 계획 요약](#8주차-이후-개발-계획-요약)

---

## 1주차: 과제 선정 및 역할 분담

### 1. 활동 목표
- 2026학년도 2학기 캡스톤디자인 과제 주제 선정 및 개발 목표 설정
- 팀 구성 및 팀원별 세부 전문 분야 분담
- 자율주행 및 동적 장애물 회피를 위한 시스템 요구사항 도출

### 2. 수행 작업 (개인 담당 중심)
- **과제 선정:** 학교 실내 환경(미래관 복도 및 세미나실)에서 활용 가능한 모바일 로봇 기반 "실내 자율순찰 및 안내 시스템" 선정.
- **플랫폼 선정:** 실내 주행 및 ROS 2 개발에 적합한 AgileX LIMO Pro 모바일 로봇 채택.
- **개인 담당 분야 확정:** **자율주행 및 동적 장애물 회피**
  - Navigation2(Nav2) 기반 위치추정(AMCL) 및 전역/지역 경로 계획
  - Costmap 레이어 구성 및 장애물 회피 제어기 튜닝
  - 좌표계(TF) 정합 및 센서 데이터-주행 스택 파이프라인 연계
- **협력 분야 분담:**
  - 실내 SLAM 맵핑 및 점유격자지도 생성
  - 온보드 PC 환경 구축 및 하드웨어 센서 드라이버 구동
  - 순찰/안내 상위 애플리케이션 시나리오 설계

### 3. 결과 및 다음 과제
- 과제 목표 및 담당 역할 분담 완료.
- 다음 단계로 하드웨어 및 ROS 2 Humble 기반 기술 검토 및 세부 일정 수립 필요.

---

## 2주차: 개발 범위 및 기술 검토

### 2. 활동 목표
- 실내 자율순찰 및 목적지 안내 시스템의 개발 범위 및 단계별 추진 계획 수립
- AgileX LIMO Pro 및 ROS 2 Humble, Nav2 스택 기술 검토
- 버전 관리(Git) 및 협업 개발 환경 초기화

### 2. 수행 작업 (개인 담당 중심)
- **개발 범위 정의:**
  - 1단계: 온보드 환경 점검, 센서 연동 및 2D SLAM 맵핑
  - 2단계: Nav2 스택 포팅, TF 트리 정합 및 정적 지도 기반 자율주행 검증
  - 3단계: 동적 장애물 감지 및 회피 알고리즘 개발
  - 4단계: 다중 웨이포인트 자율 순찰 및 사용자 호출 기반 목적지 안내 구현
- **기술 스택 검토:**
  - OS: Ubuntu 22.04.5 LTS
  - Middleware: ROS 2 Humble Hawksbill
  - SLAM: SLAM Toolbox (online_async)
  - Navigation: Navigation2 (Nav2 - Navfn Planner, DWB Local Planner / Regulated Pure Pursuit)
  - Sensor: YDLiDAR T-mini Plus (2D LiDAR), Orbbec Daabai (RGB-D Camera), 온보드 IMU 및 휠 엔코더
- **형상 관리 초기화:**
  - GitHub 협업 저장소 개설 및 기본 LIMO ROS 2 패키지 커밋.

### 3. 결과 및 다음 과제
- 프로젝트 개발 로드맵 수립 및 형상 관리 체계 확립.
- LIMO 온보드 PC 접속 및 하드웨어 인터페이스 실측 준비.

---

## 3주차: LIMO 구동 및 통신 환경 구축

### 1. 활동 목표
- LIMO 온보드 PC(Ubuntu 22.04.5 LTS, ROS 2 Humble) 환경 확인 및 원격 접속 환경 구축
- LIMO Base 드라이버와 온보드 MCU 간의 시리얼 통신 인터페이스 검증
- 핵심 오도메트리 토픽 및 `odom → base_link` TF 브로드캐스팅 확인

### 2. 수행 작업 및 발생 문제 해결

#### [문제 1] NoMachine 10 버전 라이선스 제약으로 인한 원격 접속 차단
- **현상:** 호스트 개발 PC에서 LIMO 온보드 PC로 GUI 원격 연결 시 라이선스 제약 팝업 발생 및 연결 거부.
- **원인:** NoMachine 10 버전의 무료 라이선스 정책 변경으로 단일 사용자/세션 제한이 엄격 적용됨.
- **해결 조치:** NoMachine 10을 제거하고 무료 다중 세션이 안정적으로 지원되는 NoMachine 9 버전으로 다운그레이드 설치.
- **결과:** 온보드 PC의 데스크톱 화면 및 RViz2 환경이 원격으로 정상 스트리밍됨을 확인.

#### [문제 2] LIMO Base 드라이버 실행 시 `/dev/ttylimo` 심볼릭 링크 부재
- **현상:** `ros2 launch limo_base limo_base.launch.py` 실행 시 시리얼 포트를 열지 못해 노드 비정상 종료 (`exit code 255`).
  ```text
  [limo_base-1]: Failed to open: '/dev/ttylimo'
  [ERROR] [limo_base-1]: process has died [pid ..., exit code 255]
  ```
- **원인:** 기본 OS에 LIMO 전용 udev 규칙(`99-limo.rules`)이 등록되어 있지 않아 `/dev/ttylimo` 링크가 생성되지 않음. 실제 MCU는 리눅스 기본 장치명인 `/dev/ttyUSB1`에 할당되어 있었으나 런치 파일 기본값이 `ttylimo`로 하드코딩되어 있었음.
- **해결 조치:** 실제 시리얼 포트를 런치 인자로 명시하여 실행:
  ```bash
  ros2 launch limo_base limo_base.launch.py port_name:=ttyUSB1
  ```
- **검증 결과:**
  - `Open the serial port:'/dev/ttyUSB1'` 출력과 함께 `/limo_base_node` 정상 기동.
  - `/wheel/odom`, `/imu`, `/limo_status`, `/cmd_vel` 토픽 발행 확인.
  - `tf2_echo odom base_link` 조회 결과 `odom → base_link` 변환이 약 50.14 Hz로 안정적으로 브로드캐스팅됨을 확인.

### 3. 결과 및 다음 과제
- LIMO 하부 모터 제어 및 기초 오도메트리/TF 파이프라인 검증 완료.
- 다음 단계로 2D LiDAR 센서 연동 및 SLAM 맵핑 진행 필요.

---

## 4주차: LiDAR 연동 및 수동 SLAM 맵핑

### 1. 활동 목표
- YDLiDAR T-mini Plus 2D 라이다 드라이버 구동 및 센서 데이터 수신
- SLAM Toolbox(`online_async_launch.py`)를 활용한 실내 수동 맵핑 파이프라인 점검
- 취득한 정적 지도를 기반으로 Nav2 전역 경로 계획 및 목표점 주행 기초 확인

### 2. 수행 작업 및 발생 문제 해결

#### [문제 1] RViz2 LaserScan QoS 불일치 및 포인트 미표시
- **현상:** 라이다 드라이버 구동 후 RViz2에서 `/scan` 토픽을 등록했으나 스캔 포인트가 전혀 표시되지 않음.
- **원인 로그:**
  ```text
  [WARN] [rviz2]: New publisher discovered on topic '/scan', offering incompatible QoS.
  Quality of service risk: BEST_EFFORT vs RELIABLE
  ```
  `ydlidar_ros2_driver_node`는 네트워크 대역폭 절감을 위해 `Reliability: BEST_EFFORT`로 퍼블리시하고 있었으나, RViz2 기본 설정이 `RELIABLE`이어서 DDS 계약 불일치로 데이터가 드롭됨.
- **해결 조치:** RViz2 LaserScan 디스플레이의 `Reliability Policy`를 `BEST_EFFORT`로 변경.
- **결과:** 360도 2D 라이다 스캔 포인트클라우드가 RViz 상에 정상 투영됨.

#### [문제 2] SLAM 백그라운드 중복 프로세스로 인한 TF 충돌
- **현상:** 수동 조종 매핑 중 RViz의 지도가 실시간 갱신되지 않고 멈춤 현상 발생.
- **원인:** 이전 세션에서 종료되지 않은 `/slam_toolbox` 프로세스가 백그라운드에 남아 중복으로 `map → odom` TF를 브로드캐스트하여 좌표 충돌 발생.
- **해결 조치:** `ps -ef | grep '[s]lam_toolbox'`로 잔류 PID 확인 후 강제 종료(`kill -9`).
- **결과:** 실시간 점유격자 갱신이 정상화됨.

#### [수행 결과 및 지도 저장]
- 수동 주행을 통해 연구실 실내 점유격자지도 생성 및 저장 완료:
  ```bash
  ros2 run nav2_map_server map_saver_cli -f ~/capstone_ws/src/limo_capstone/maps/demo_map
  ```
  - 산출물: `demo_map.pgm`, `demo_map.yaml`
- 정적 지도를 로드한 상태에서 Nav2를 구동하고, RViz2의 "2D Goal Pose" 도구로 목표점을 주어 Global Planner가 유효한 전역 경로를 생성하고 로봇이 해당 목표점으로 이동함을 1차 확인.

---

## 5주차: 맵핑 확장 및 하드웨어·주행 문제 분석

### 1. 활동 목표
- 미래관(MC관) 2층 실내 복도 환경으로 맵핑 영역 확장
- 정적 지도 기반 Navigation2 위치추정(AMCL) 및 목표점 주행 신뢰성 점검
- 맵핑 도중 발생한 통신 단절 및 TF 중단 원인 분석

### 2. 수행 작업 및 발생 문제 해결

#### [문제 1] 배터리 저전압 돌입전류로 인한 USB 브라운아웃 및 포트 변동
- **현상:** 주행 맵핑 도중 갑자기 TF가 중단되고 로봇 통신이 먹통이 됨. 이후 포트를 조회하니 `/dev/ttyUSB1`이 사라지고 `/dev/ttyUSB2`로 재할당됨.
- **커널 로그(`dmesg`) 정밀 분석:**
  ```text
  [14006.132714] usb 3-7: USB disconnect, device number 7
  [14006.132995] cp210x ttyUSB1: failed set request 0x7 status: -19
  [14006.133476] cp210x ttyUSB1: cp210x converter now disconnected from ttyUSB1
  [14007.080604] usb 3-7.4: cp210x converter now attached to ttyUSB2
  ```
- **원인 규명:**
  1. 배터리 2칸 저전압 상태에서 모터 구동 시 발생하는 **돌입 전류(Inrush Current)**로 인해 5V USB 버스에 순간 전압 강하(Brownout) 발생.
  2. USB-UART 브릿지인 CP210x 칩셋에 순간 정전이 일어나 커널에서 `status: -19 (ENODEV)` 에러와 함께 연결 해제.
  3. 1초 뒤 장치가 재인식되었으나, 기존 `ttyUSB1` 디스크립터를 점유하던 노드로 인해 커널이 `ttyUSB2`로 새 번호를 부여함.
  4. LIMO Base 노드는 끊어진 포트를 바라보다 정지하여 `odom → base_link` TF가 영구 단절됨.
- **해결 및 대응 조치:**
  - 좀비 프로세스 정리: `sudo fuser -k /dev/ttyUSB*`
  - **안전 운용 수칙 수립:** 배터리 3칸 이상에서만 주행 시험을 수행하고 2칸 이하 시 즉시 충전.

#### [문제 2] Nav2 초기 위치 미설정 시 글로벌 경로 계획 실패
- **현상:** 정적 지도 내비게이션 기동 후 목표를 주어도 로봇이 움직이지 못하고 TF 룩업 타임아웃 경고 반복:
  ```text
  [amcl]: Timed out waiting for transform from base_link to map to become available...
  [global_costmap]: Message Filter dropping message: frame 'base_link' would require transform to 'map' which is not available
  ```
- **원인 분석:** AMCL은 초기 위치 추정값(`/initialpose`)을 수신하여 파티클을 수렴시켜야 `map → odom` TF를 활성화함. 런치 직후 초기 포즈가 주입되지 않아 전역 코스트맵과 플래너가 좌표계를 확정할 수 없었음.
- **해결 조치:** RViz 상단 툴바의 `2D Pose Estimate` 도구를 통해 실제 로봇 위치와 방향을 주입함. 파티클이 수렴하며 `map → odom` TF가 활성화되어 정상적인 전역 경로가 생성됨을 확인.
- **산출물:** 미래관 2층 복도 점유격자지도 `MC_2F_10_07.pgm` 및 `.yaml` 취득 완료.

---

## 6주차: 자율 탐색 맵핑 시도 및 3대 핵심 결함 도출

### 1. 활동 목표
- SLAM Toolbox, Nav2, Explore Lite를 결합한 통합 자율 탐색 런치(`limo_auto_mapping.launch.py`)를 작성하여 MC관 3층 복도에서 자동 맵핑 시도
- 자율 탐색 파이프라인의 런타임 결함 및 병목 현상 조사

### 2. 수행 작업 및 도출된 3대 결함

MC관 3층 복도 현장에서 자동 맵핑을 수행하여 점유격자지도 `MC_3F_10_15.pgm`/`.yaml`(764×809 cells, 38.2m × 40.45m)을 생성하였으나, 다음 3가지 핵심 결함이 도출됨.

#### [결함 1] `setup.py` 패키지 빌드 오류로 인한 맵 파일 미배포
- `colcon build --packages-select limo_capstone` 실행 시 `maps/` 내에 잔류하던 임시 빌드 디렉터리까지 복사하려다 빌드가 실패함:
  ```text
  error: can't copy 'maps/install': doesn't exist or not a regular file
  ```
- 결과: `install/share/limo_capstone/maps/` 디렉터리에 신규 지도 파일이 배포되지 않아 내비게이션 런치에서 맵을 찾지 못함.

#### [결함 2] Nav2 Costmap 필수 좌표계 `base_footprint` TF 누락
- LIMO 하드웨어 드라이버(`limo_base`)는 `odom → base_link` 변환만 발행함.
- 반면 Nav2 Costmap2D는 로봇의 2D 바닥면 투영 좌표계인 `base_footprint`를 필수로 요구하므로, `base_link → base_footprint` 변환이 없어 Costmap이 센서 데이터를 투영하지 못하고 드롭하는 문제 발생.

#### [결함 3] Explore Lite 자율 탐색의 조기 정지 및 무한 성공 루프
- 360도 라이다가 실내 벽면을 관측할 때 고리 형태로 형성된 프론티어 셀들의 무게중심(`centroid`)이 로봇 바로 앞 빈 공간에 계산됨.
- 로봇이 목표에 도달하면 즉시 성공 판정이 내려지지만 실제 미탐색 영역은 벽면에 있으므로 프론티어가 지워지지 않고, 동일 좌표에 대해 초당 40회씩 무한 성공 판정이 반복되며 탐색이 멈춤.

---

## 7주차: 패키지 빌드 오류 및 TF·탐색 알고리즘 수정

### 1. 활동 목표
- 6주차에 도출된 3대 핵심 결함을 소스 코드 및 설정 파일 수정을 통해 근본적으로 해결
- 패키지 클린 빌드 성공, 맵 파일 자동 배포 및 TF 무결성 확보
- 자율 탐색 목표점 선정 로직 개선 및 단위 검증

### 2. 세부 수정 사항 및 코드 변경 내역

#### [수정 1] `setup.py` 일반 파일 필터링 적용 (Commit: `b46aba9`)
- **수정 파일:** [`capstone_ws/src/limo_capstone/setup.py`](../capstone_ws/src/limo_capstone/setup.py#L16-L22)
- **변경 내용:** `glob('maps/*')` 대신 `os.path.isfile(f)` 리스트 컴프리헨션을 적용하여 임시 디렉터리가 생성되어도 정규 파일만 안전하게 선별 복사하도록 방어적 빌드 설정 구현.
  ```python
  (os.path.join('share', package_name, 'maps'), [f for f in glob('maps/*') if os.path.isfile(f)]),
  (os.path.join('share', package_name, 'worlds'), [f for f in glob('worlds/*') if os.path.isfile(f)]),
  (os.path.join('share', package_name, 'scripts'), [f for f in glob('scripts/*') if os.path.isfile(f)]),
  ```
- **검증 결과:** `colcon build --packages-select limo_capstone`이 0.68초 만에 성공하고, install 디렉터리에 13개 맵 파일이 정상 배포됨을 확인.

#### [수정 2] `base_link → base_footprint` 정적 TF 브로드캐스터 추가 (Commit: `7439841`)
- **수정 파일:** [`capstone_ws/src/limo_capstone/launch/limo_nav2.launch.py`](../capstone_ws/src/limo_capstone/launch/limo_nav2.launch.py#L35-L54) 및 `limo_auto_mapping.launch.py`
- **변경 내용:**
  1. `tf2_ros`의 `static_transform_publisher` 노드를 추가하여 `base_link → base_footprint` (0, 0, 0, 0, 0, 0) 항등 변환을 100 Hz로 브로드캐스트.
  2. `limo_auto_mapping.launch.py`에서 `static_tf` 노드가 `explore_node`보다 먼저 기동되도록 런치 선언 순서 조정.
- **검증 결과:** `tf2_echo base_link base_footprint` 항등 변환 확인 및 Nav2 Costmap의 TF 타임아웃 경고 완전 소멸.

#### [수정 3] Explore Lite 프론티어 목표 선정 기준 변경 (Commit: `7439841`)
- **수정 파일:** [`capstone_ws/src/m-explore-ros2/explore/src/explore.cpp`](../capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L268)
- **변경 내용:** 가상 무게중심(`frontier->centroid`) 대신 로봇에서 가장 가까운 실제 미탐색 경계 셀(`frontier->middle`)을 목표 좌표로 설정하도록 수정:
  ```cpp
  - geometry_msgs::msg::Point target_position = frontier->centroid;
  + geometry_msgs::msg::Point target_position = frontier->middle;
  ```
- **검증 결과:** 시뮬레이션 환경에서 동일 좌표 무한 성공 루프가 100% 소멸하고, 로봇이 실제 미탐색 영역으로 전진하며 탐색 면적이 대폭 증가함(미탐색 셀 83.2% $\rightarrow$ 15.0% 감소).

#### [수정 4] YDLiDAR 스캔 배열 크기 정수 계산 패치
- **수정 파일:** `ydlidar_ros2_driver_node.cpp`
- **변경 내용:** 각도 분해능 나눗셈 결과의 부동소수점 오차로 인한 배열 크기 1개 절사 현상을 `std::round`로 보정.

---

## 8주차: Nav2 지도 수신 해결, TF 트리 계측 및 실차 주행 검증

### 1. 활동 목표
- Nav2 Lifecycle Manager 및 RViz2 QoS 불일치로 인한 `No map received` 문제 해결
- `tf2_tools view_frames`를 통한 로봇 전체 10개 TF 좌표계 연결 상태 및 발행 주기 정밀 계측
- 미래관 3층 복도 및 세미나실 실내 환경에서 정적 지도 기반 실차 자율주행 검증

### 2. 수행 작업 및 발생 문제 해결

#### [문제 1] Nav2 실행 후 지도가 방출되지 않는 현상 (`No map received`)
- **현상:** `limo_nav2.launch.py` 기동 후 RViz2를 켰으나 화면에 지도가 표시되지 않고 `No map received` 경고 출력.
- **원인 분석:**
  1. `/map_server`와 `/amcl` 노드가 ROS 2 Lifecycle 관리자에 의해 활성화되지 못하고 `unconfigured [1]` 상태에 머물러 있어 지도 퍼블리셔가 대기 상태에 갇힘.
  2. Map Server는 지도를 `Transient Local` (Latched) QoS로 발행하는데, RViz2의 Map 디스플레이 기본 Durability가 `Volatile`로 설정되어 신규 구독 시 기존 맵 데이터를 수신 거부함.
- **해결 조치:**
  1. ROS 2 Lifecycle 서비스 명령을 통해 노드를 명시적으로 활성화:
     ```bash
     ros2 lifecycle set /map_server configure
     ros2 lifecycle set /map_server activate
     ros2 lifecycle set /amcl configure
     ros2 lifecycle set /amcl activate
     ```
  2. RViz2 Map Display의 `Durability Policy`를 `Transient Local`로 변경.
- **결과:** `/map` 토픽 데이터가 즉시 수신되어 화면에 고해상도 점유격자지도가 정상 렌더링됨.

#### [TF 트리 무결성 정밀 계측 실험]
- `ros2 run tf2_tools view_frames`를 실행하여 실시간 좌표계 변환 트리를 추출하고 무결성을 확인 (`frames_2026-10-13_19.38.47.gv` 및 10월 21일 보고서 실측 기록).
- **계측 결과 (10개 프레임 전원 정상 연결):**
  - 10월 13일 온보드 실측(`frames_2026-10-13_19.38.47.gv`): `odom → base_footprint` (50.20 Hz), `base_footprint → base_link` (정적 TF, 10,000 Hz), 센서 링크(`laser_link`, `imu_link`, `depth_camera_link` 등 10,000 Hz), 4륜 휠 링크(약 60.2 Hz).
  - 8주차 Nav2 구동 실측(10월 21일 보고서 기록): AMCL 노드가 활성화되어 상위에 `map → odom` (약 10.36 Hz) 변환이 결합되고, 전체 10개 프레임이 단절이나 지연 없이 단일 트리로 견고하게 결합됨을 확인 완료함.

#### [실차 자율주행 시험 및 결과]
- 미래관 3층 세미나실 정밀 지도(`MC3F_room_map.pgm`, 10월 21일 작업 폴더 및 발표 이미지에 보관)를 로드. (참고: Git 메인 트랙 워크스페이스에는 연구실 `demo_map`, 복도 `MC_2F_10_07`, `MC_3F_10_15` 등이 관리됨).
- AMCL 초기 위치를 설정한 후, RViz 상에서 2D Goal Pose를 지정하여 전역 경로 계획 및 DWB Local Planner 기반의 궤적 추종 수행.
- 복도 직진 및 세미나실 출입문 통과 주행 시험을 성공적으로 수행함.

---

## 8주차 이후 개발 계획 요약

> [!NOTE]
> 1~8주차까지의 작업은 실제 코드, 로그, 계측 파일 및 실차 시험 결과로 확인된 내용입니다.
> 9주차 이후의 작업은 프로젝트 계획서 및 추진체계에 근거한 **향후 개발 계획**이며, 구현 및 검증이 완료되는 주차별로 추가 갱신될 예정입니다.

| 주차 | 계획 과제명 | 세부 목표 및 구현 대상 | 상태 |
|:---:|---|---|:---:|
| **9주차** | 자율 탐색 파라미터 최적화 | `progress_timeout: 25.0` 완화 적용 및 실차 복도 탐색 완주 시험 | 계획 |
| **10주차** | Nav2 로컬 제어기 파라미터 최적화 | LIMO 차체 치수(반경 0.19m) 및 복도 폭을 반영한 DWB / Costmap Inflation 튜닝 | 계획 |
| **11주차** | 동적 장애물 감지 및 회피 노드 개발 | 전방 LiDAR 필터링 노드(`obstacle_detector.py`) 및 회피 제어(`obstacle_controller.py`) 구현 | 계획 |
| **12주차** | ROS 2 Action 기반 자율 순찰 노드 개발 | `NavigateToPose` 액션 기반 다중 웨이포인트 순환 주행 노드(`patrol_limo.py`) 구현 | 계획 |
| **13주차** | 목적지 에스코트 안내 노드 개발 | 사용자 호출 수신 시 목표 지점 이동 및 대기 위치 복귀 노드(`guide_limo.py`) 구현 | 계획 |
| **14~15주차**| 시스템 종합 통합 시험 및 최종 평가 | 순찰/안내 모드 통합, 1시간 연속 장시간 주행 신뢰성 및 비상 정지 안전 검증 | 계획 |
