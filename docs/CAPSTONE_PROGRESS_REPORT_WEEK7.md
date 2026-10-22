# 캡스톤디자인 1~7주차 진도보고서 (오류 분석 및 문제 해결 과정 중심)

- **과제명:** LIMO를 이용한 실내 자율 순찰 및 안내
- **소속:** 배재대학교 드론로봇공학과
- **학년도 / 학기:** 2026학년도 2학기
- **교과목명:** 캡스톤디자인 1
- **팀명:** LIMO의 모험 (4조)
- **지도교수:** 이창훈 교수님
- **참여 학생:**
  - **팀장:** 정진우 (학번: 2248050, 3학년)
  - **팀원:** 김정민 (학번: 2248075, 3학년)
  - **팀원:** 배민혁 (학번: 2248082, 3학년)
  - **팀원:** 한동훈 (학번: 2048089, 4학년)
- **보고 기간:** 1주차 ~ 7주차 (중간 점검)
- **작성 기준일:** 2026-10-20 (일자는 초안이며 사용자가 학사 포털 입력 시 직접 조정 가능)

---

## 1. 보고서 개요 및 작성 목적

본 진도보고서는 프로젝트의 자율주행이나 자율 탐색 기능의 구현 완료를 홍보하거나 과장하기 위한 성과 보고서가 아닙니다. LIMO Pro 모바일 로봇과 ROS 2를 기반으로 시스템을 구축해 나가는 과정에서 **실제로 마주친 오류와 문제를 발견하고, 원인을 추적·분석하며, 관련 소스 코드와 설정을 수정한 디버깅 및 문제 해결 과정을 있는 그대로 기록**하는 것을 최우선 목적으로 합니다.

특히 7주차는 주행 성공률이나 자율 순찰의 완성도를 강조하지 않고, **6주차에서 도출된 세 가지 핵심 결함(지도 파일 빌드 배포 오류, Nav2 TF 프레임 연결 누락, Explore Lite 목표점 반복 생성 및 조기 정지 버그)**을 해결하기 위해 수행한 기술적 조사와 코드 수정 과정, 그리고 수정 효과를 확인하기 위해 수행한 검증 결과를 정리하였습니다.

아울러 본 보고서에서는 **소스 코드 및 로그로 확인된 객관적 사실**과 **추정/가설**을 엄격히 분리하였으며, **가상 시뮬레이션 환경에서 확인된 동작**과 **실물 LIMO Pro 로봇에서 아직 확인되지 않은 한계(미검증 조건)**를 명확히 구분하여 작성하였습니다.

---

## 2. 1~7주차 주차별 문제 해결 과정 기록

### [1주차] 팀 편성 및 주제 선정
- **진행 목표:** 캡스톤디자인 1 팀을 편성하고, 2학기 동안 수행할 프로젝트 과제 주제 및 팀원별 세부 역할을 확정한다.
- **수행 내용:** 드론로봇공학과 학생 4명으로 팀(팀명: "LIMO의 모험", 4조)을 구성하고, 학교 소유 모바일 로봇인 AgileX LIMO Pro와 ROS 2 플랫폼을 기반으로 미래관 실내 복도 및 로비에서 활용 가능한 "실내 자율 순찰 및 안내"를 과제 주제로 선정하였다 `[사용자 제공 사실]`.
- **진행 중 발생한 문제:** 팀 구성 및 주제 협의 단계로, 실제 하드웨어 구동이나 소프트웨어 개발이 착수되지 않아 발생한 시스템 오류나 기술적 문제는 없다 `[확인된 문제 없음]`. (확인되지 않은 오류를 임의로 기재하지 않음)
- **해결을 위해 수행한 조치:** 팀장(정진우) 및 분야별 담당자(자율주행/알고리즘: 정진우, SLAM/맵핑: 배민혁, 시스템 구축/센서/TF: 한동훈, 순찰/안내 액션 애플리케이션: 김정민)의 역할 분담을 협의 완료하였다.
- **조치 후 확인한 결과:** 4인 팀 구성 완료 및 과제 기본 추진 방향 수립.
- **남은 과제:** LIMO 하드웨어 분석 및 주차별 구체적 개발 마일스톤 수립 필요.

---

### [2주차] 계획서 작성 및 개발 방향 수립
- **진행 목표:** 학기 동안의 개발 범위와 추진 일정을 체계화한 캡스톤디자인 팀별 계획서를 작성하고 형상 관리 환경을 초기화한다.
- **수행 내용:** `4조. 캡스톤디자인 팀별계획서_수정.hwp`를 작성하여 제출하였다. 4단계 개발 일정(1단계: 환경 구축 및 맵핑 $\rightarrow$ 2단계: 자율주행 알고리즘 구현 $\rightarrow$ 3단계: 필드 테스트 $\rightarrow$ 4단계: 최적화 및 순찰/안내 완성)을 수립하고, GitHub 저장소(`Capstone1_limo`)를 생성하여 기본 LIMO 패키지를 초기 커밋(`commit 06b0bca`, 2026.09.14)하였다.
- **진행 중 발생한 문제:** 계획서에 기술된 최종 완성 목표(다중 경유지 자율 순찰, 사용자 목적지 안내 등)와 초기 개발 단계(기본 드라이버 검증, 맵핑) 간의 개발 범위 혼선 가능성이 식별됨.
- **문제를 확인한 방법:** 팀별 계획서 작성 검토 과정에서 최종 목표의 정량 지표와 당장 1단계에서 확인 가능한 단위 기능의 격차 분석.
- **원인 분석:** 계획서의 목표는 학기 전체의 최종 산출물을 목표로 하므로, 주차별 보고서에서 이를 이미 구현된 것으로 기술할 경우 실제 개발 진세가 왜곡될 위험이 있었음.
- **해결을 위해 수행한 조치:** 계획서의 최종 목표는 장기 지향점으로 유지하되, 주차별 진도보고서에는 각 단계에서 소스 코드, 설정 파일, 터미널 로그로 실제 검증된 단위 기능과 오류 해결 내용만 기록한다는 관리 원칙을 정립함.
- **조치 후 확인한 결과:** 개발 범위 혼선 방지 및 GitHub 저장소 기본 구조 확립.
- **남은 과제:** LIMO 하드웨어 통신 확인 및 온보드 ROS 2 환경 점검 필요.

---

### [3주차] 개발환경 구축 및 LIMO 베이스 분석 중 발생한 통신·접속 문제
- **진행 목표:** LIMO 온보드 PC(Ubuntu 22.04 LTS, ROS 2 Humble) 환경을 점검하고, 공식 패키지 구조 분석 및 LIMO Base 드라이버와 YDLiDAR T-mini Plus의 시리얼 통신 상태를 확인한다.

#### 문제 1: LIMO 베이스 드라이버 실행 시 `/dev/ttylimo` 심볼릭 링크 부재
- **진행 중 발생한 문제:** 공식 실행 명령인 `ros2 launch limo_base limo_base.launch.py` 실행 시 장치 파일을 열지 못하여 베이스 노드가 비정상 종료됨.
- **문제를 확인한 방법:** 터미널 에러 로그 확인 (`docs/DEVELOPMENT_LOG.md` L88-96):
  ```text
  [limo_base-1]: Failed to open: '/dev/ttylimo'
  [ERROR] [limo_base-1]: process has died [pid ..., exit code 255]
  ```
- **원인 분석:** 온보드 OS에 LIMO 하드웨어용 udev 규칙(`99-limo.rules` 등)이 미설정되어 `/dev/ttylimo` 심볼릭 링크가 생성되지 않았음. 실제 MCU 시리얼 포트는 리눅스 기본 장치명인 `/dev/ttyUSB1`에 할당되어 있었으나, 런치 파일의 기본값이 `ttylimo`로 하드코딩되어 있어 접근에 실패함.
- **해결을 위해 수행한 조치:** `ls -l /dev/ttyUSB*` 조사를 통해 LIMO MCU가 연결된 포트가 `/dev/ttyUSB1`임을 확인하고, 런치 인자로 실제 장치명을 명시하여 실행함:
  ```bash
  ros2 launch limo_base limo_base.launch.py port_name:=ttyUSB1
  ```
- **조치 후 확인한 결과:** `Open the serial port:'/dev/ttyUSB1'` 로그와 함께 `/limo_base_node`가 정상 기동되고, `/wheel/odom`, `/imu`, `/limo_status` 토픽 발행 및 `odom -> base_link` TF 브로드캐스팅을 확인함 (`docs/DEVELOPMENT_LOG.md` L104-120).

#### 문제 2: 원격 데스크톱 연결 시 NoMachine 10 세션 라이선스 제약
- **진행 중 발생한 문제:** 호스트 PC에서 로봇 온보드 PC로 GUI 원격 연결 시 NoMachine 세션이 차단되거나 다중 접속이 제한됨 `[사용자 제공 사실]`.
- **문제를 확인한 방법:** NoMachine 10 연결 시 라이선스 제약 팝업 발생 및 연결 거부 관찰.
- **원인 분석:** NoMachine 10 버전에서 무료 라이선스 정책이 변경되어 단일 사용자/세션 제한이 엄격하게 적용됨에 따라, 개발 PC와 온보드 환경 간의 동시 디버깅 세션 유지가 불가능했음.
- **해결을 위해 수행한 조치:** NoMachine 10 버전을 제거하고, 무료 다중 접속이 지원되는 이전 안정 버전인 NoMachine 9 버전으로 다운그레이드 설치함 `[사용자 제공 사실]`.
- **조치 후 확인한 결과:** 온보드 PC의 데스크톱 화면 및 RViz GUI가 원격 환경에서 정상 스트리밍됨을 확인.
- **남은 과제:** 재부팅 시 USB 포트 번호가 변경될 가능성이 있어 물리 경로 확인 체계 필요.

---

### [4주차] 수동 매핑 시험 및 센서·TF 동기화 오류
- **진행 목표:** LIMO를 수동 조작하여 SLAM Toolbox 기반의 2D 점유격자 매핑을 시험하고, 라이다 센서 데이터와 TF 변환의 실시간성을 점검한다.

#### 문제 1: RViz LaserScan QoS 비호환 및 포인트클라우드 미표시
- **진행 중 발생한 문제:** 라이다 드라이버 구동 후 RViz2에서 `/scan` 토픽을 LaserScan 디스플레이로 등록했으나, 화면에 포인트클라우드가 전혀 표시되지 않음.
- **문제를 확인한 방법:** 터미널에 다음 QoS 정책 충돌 경고가 출력됨을 확인 (`docs/DEVELOPMENT_LOG.md` L622-630):
  ```text
  New publisher discovered on topic '/scan', offering incompatible QoS.
  No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
  ```
- **원인 분석:** `ydlidar_ros2_driver_node`는 네트워크 대역폭 절감을 위해 `Reliability: BEST_EFFORT` 정책으로 퍼블리시하고 있었으나, RViz2의 LaserScan Subscriber 기본 정책은 `Reliability: RELIABLE`로 설정되어 있어 DDS QoS 호환성 계약이 성립되지 않았음.
- **해결을 위해 수행한 조치:** RViz2의 LaserScan 디스플레이 설정 내 `Reliability Policy`를 `RELIABLE`에서 `BEST_EFFORT`로 변경함 (`docs/DEVELOPMENT_LOG.md` L657-677).
- **조치 후 확인한 결과:** RViz 상에서 360도 2D 라이다 스캔 포인트가 실시간으로 정상 투영됨을 확인.

#### 문제 2: 온보드 USB 시리얼 포트 번호 변동 혼선
- **진행 중 발생한 문제:** 실차 온보드 PC를 재부팅하거나 USB 케이블을 재연결했을 때 기존 1번(`ttyUSB1`)으로 잡히던 베이스 포트가 인식되지 않고 2번(`ttyUSB2`) 등으로 표시되는 현상 발생 `[사용자 제공 사실]`.
- **문제를 확인한 방법:** `port_name:=ttyUSB1` 지정 시 디바이스 오픈 에러 재발생 및 `ls /dev/ttyUSB*` 확인 시 포트 인덱스 변경 관찰.
- **원인 분석:** 리눅스 커널의 USB enumeration 순서 변경 또는 직전 프로세스의 시리얼 디바이스 점유 해제 지연으로 판단되나, 하드웨어 회로 수준의 근본 원인은 명확히 규명되지 않음 $\rightarrow$ **'포트 인식 문제로 추정'**으로 분류.
  *(※ 후속 분석 참고: 후속 트러블슈팅 과정에서는 배터리 잔량이 낮은 상태(2칸 이하)에서 모터를 구동할 때 CP210x 관련 커널 오류(`status: -19`)와 USB 시리얼 장치 재할당이 관찰되었다. 전원 강하에 따른 USB Brownout 가능성이 제기되었으나, USB 전압을 직접 측정한 자료는 없어 근본 원인이 확정된 것은 아니다. 이에 따라 주행 전 배터리 잔량을 충분히 확보(3칸 이상)하는 예방 수칙을 마련하였다.)*
- **해결을 위해 수행한 조치:** 물리적 PCI 허브 경로(`ls -l /dev/serial/by-path/`)를 대조하여 LIMO Base와 라이다의 물리 연결 경로를 확인하고 포트 파라미터를 임시 재지정함.
- **조치 후 확인한 결과:** 변경된 포트 번호로 드라이버 통신 일시 복구 확인.

#### 문제 3: 수동 주행 중 `/map` 토픽 갱신 지연 및 TF 충돌
- **진행 중 발생한 문제:** SLAM Toolbox 노드가 실행 중임에도 로봇 주행 시 RViz의 지도가 실시간 확장되지 않고 멈춤 현상 발생 (`docs/DEVELOPMENT_LOG.md` L853-875).
- **문제를 확인한 방법:** `ros2 topic hz /map` 측정 결과 퍼블리시 주기가 극히 불규칙하고 TF 변환 룩업 타임아웃 경고 관찰.
- **원인 분석:** 이전 테스트에서 비정상 종료된 `/slam_toolbox` 백그라운드 프로세스가 백그라운드에 남아 중복으로 TF 변환을 브로드캐스팅하여 좌표 충돌이 발생함.
- **해결을 위해 수행한 조치:** 백그라운드 잔류 SLAM 프로세스를 확인 후 `kill -9`로 강제 정리하고, 단일 인스턴스로 재실행함.
- **조치 후 확인한 결과:** 실시간 점유격자 갱신이 정상화되어 도미토리 라운지 점유격자지도(`domi_map.pgm`/`yaml`, 2026.10.05)를 수동 저장 완료함.
- **남은 과제:** 수동 조종 방식의 한계를 극복하기 위해 자율 탐색 및 정적 지도 기반 내비게이션 파이프라인 구축 필요.

---

### [5주차] 정적 지도 로딩 및 Nav2 목표점 주행 분석
- **진행 목표:** 기 취득된 2D 점유격자지도(`demo_map`, `MC_2F_10_07`)를 불러오고, Navigation2(Nav2) 스택(AMCL, Costmap, Planner, Controller)을 연동하여 정적 지도 기반 내비게이션 파이프라인을 점검한다.

#### 문제 1: 초기 위치 미설정 시 글로벌 경로 계획 실패
- **진행 중 발생한 문제:** `limo_nav2.launch.py` 구동 후 목표 위치를 전달해도 로봇이 주행을 시작하지 못하고 정체됨.
- **문제를 확인한 방법:** Nav2 콘솔에 다음 경고 및 플래너 실패 로그 확인:
  ```text
  [amcl]: Timed out waiting for transform from base_link to map to become available...
  [global_costmap]: Message Filter dropping message: frame 'base_link' would require transform to 'map' which is not available
  ```
- **원인 분석:** 정적 지도 내비게이션에서는 AMCL이 초기 위치 추정값(`/initialpose`)을 수신하여 파티클을 수렴시켜야 `map -> odom` TF가 활성화되는데, 런치 직후 초기 위치가 주입되지 않아 전역 코스트맵과 플래너가 좌표계를 확정할 수 없었음.
- **해결을 위해 수행한 조치:** 로봇의 실제 실내 위치와 지도의 기준 특징점을 대조하여 RViz 상단 툴바의 "2D Pose Estimate" 도구로 초기 포즈를 주입함.
- **조치 후 확인한 결과:** AMCL 파티클이 수렴하며 `map -> odom` TF가 활성화되고, 글로벌 코스트맵 상에 정상 전역 경로가 생성되어 `/cmd_vel` 출력이 발생함을 확인.

#### 사실 확인 및 기능 구분 (목표 지정 방식):
- 본 주차에서 수행한 목표점 지정 및 주행 명령 전달은 본 팀이 직접 작성한 Python/C++ 노드가 아니라, **RViz2 표준 플러그인(`nav2_rviz_plugins`)의 내장 GUI 도구인 "2D Goal Pose"**를 사용한 것임을 확인하고 자체 개발 기능과 명확히 구분함 `[저장소 확인]`.
- **남은 과제:** 사용자가 매번 RViz에서 목표를 수동으로 찍지 않고 로봇이 스스로 미탐색 영역을 찾아 지도를 완성하는 완전 자동 매핑 파이프라인 구축 필요.

---

### [6주차] 자동 매핑 파이프라인 구축 중 발생한 3대 핵심 결함 도출
- **진행 목표:** SLAM Toolbox(온라인 비동기 맵핑), Nav2 Navigation, Explore Lite(프론티어 자동 탐색)를 결합한 완전 자율 탐색 매핑 파이프라인(`limo_auto_mapping.launch.py`)을 구축하고 실내 매핑을 수행한다.
- **수행 내용:** `limo_auto_mapping.launch.py` 작성 및 미래관 3층 실차 주행을 통해 실내 점유격자지도 `MC_3F_10_15.yaml`/`pgm` (764×809 cells, 38.2m × 40.45m) 생성 및 저장 (`2026.10.15`).
- **도출된 3대 핵심 결함:**

#### 결함 1: `setup.py` 패키지 빌드 오류로 인한 지도 파일 배포 실패
- **관찰 증상:** `colcon build --packages-select limo_capstone` 수행 시 빌드가 실패하여 `install/share/limo_capstone/maps/` 디렉터리에 지도 파일이 설치되지 않음.
- **발견 로그 (`docs/DEVELOPMENT_LOG.md` L1211-1224):**
  ```text
  error: can't copy 'maps/install': doesn't exist or not a regular file
  ```
- **원인 분석:** `setup.py`의 `glob('maps/*')`가 빌드 임시 디렉터리까지 포함하여 setuptools 파일 복사 루틴이 디렉터리를 복사하려다 예외를 발생시킴.

#### 결함 2: Nav2 Costmap 필수 좌표계 `base_footprint` TF 누락
- **관찰 증상:** 내비게이션 구동 시 로봇의 위치가 맵상에서 왜곡되거나 TF 룩업 타임아웃 경고 반복 출력.
- **원인 분석:** LIMO 공식 하드웨어 드라이버(`limo_base`)는 `odom -> base_link`까지만 브로드캐스트하여, Nav2 및 Costmap2D가 필수로 요구하는 지면 투영 좌표계 `base_footprint` 프레임이 누락됨. 아울러 런치 파일 내에서 `explore_node`가 `static_tf` 노드보다 먼저 실행되어 시작 시점에 변환 조회를 실패함.

#### 결함 3: Explore Lite 자율 탐색 조기 정지 및 무한 성공 루프 (문제 C)
- **관찰 증상:** 5m × 5m 밀폐 공간 시뮬레이션에서 로봇이 전진하지 못하고 동일 좌표 `(0.27, 0.03)`에 대해 초당 40회씩 `Reached the goal!`을 무한 반복하며 16.8%에서 탐색이 멈춤 (`docs/DEVELOPMENT_LOG.md` L1268-1308).
- **원인 분석:** 360도 라이다가 실내 벽면 안쪽을 스캔할 때 고리 형태로 형성된 프론티어 셀들의 무게중심(`centroid`)이 로봇 바로 앞 빈 공간에 계산되어 즉시 도착 판정이 발생하고, 프론티어가 지워지지 않아 무한 루프에 빠짐.
- **결론:** 6주차에 도출된 이 3가지 핵심 오류의 원인 분석 및 코드 수정을 7주차의 집중 과제로 이관함.

---

### [7주차] 오류 원인 분석 및 코드·설정 수정 (중심 주차)

**7주차는 자율주행의 성공이나 주행 성능을 과시하는 주차가 아닙니다. 6주차에 도출된 3대 결함을 소스 코드와 설정 파일 수정을 통해 하나씩 분석하고 해결해 나간 디버깅 과정입니다.**

```
[6주차 발견 3대 결함]
├── 1. setup.py 디렉터리 복사 오류 ──────> os.path.isfile(f) 필터링 (commit b46aba9)
├── 2. Nav2 base_footprint TF 누락 ─────> static_transform_publisher 추가 (commit 7439841)
└── 3. explore.cpp centroid 조기 정지 ──> frontier->middle 전략 수정 (commit 7439841)
```

---

#### 1. `setup.py`에서 리소스 파일 배포 시 디렉터리가 포함되어 빌드 오류가 발생한 문제

- **발생한 증상:**
  - `colcon build --packages-select limo_capstone` 실행 시 패키지 빌드가 비정상 중단됨.
  - 빌드 실패로 인해 `install/share/limo_capstone/maps/` 디렉터리에 신규 지도 파일(`MC_3F_10_15.yaml`/`pgm` 등)이 설치(배포)되지 않아, 내비게이션 실행 시 지도 파일을 찾을 수 없는 오류 발생.
  - 아울러 본 수정 시 지도(`maps/`) 파일뿐 아니라 시뮬레이션 월드(`worlds/`) 및 유틸리티 스크립트(`scripts/`) 파일 배포 목록에도 동일한 정규 파일 필터링 로직이 함께 적용되어 패키지 리소스 배포 구조가 안정화됨.
- **문제를 발견한 로그 또는 관찰 내용:**
  - `colcon build` 터미널 출력 에러 로그 (`docs/DEVELOPMENT_LOG.md` L1211-1224):
    ```text
    Starting >>> limo_capstone
    --- stderr: limo_capstone
    error: can't copy 'maps/install': doesn't exist or not a regular file
    ---
    Failed   <<< limo_capstone [0.45s, run with --event-handlers console_direct+ --debug]
    ```
- **원인 분석과 근거:**
  - [`capstone_ws/src/limo_capstone/setup.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/setup.py#L18) 18행의 `data_files` 항목이 다음과 같이 작성되어 있었음:
    ```python
    (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ```
  - 이전 작업 과정에서 `maps/` 디렉터리 내에 `install`, `build` 등의 하위 디렉터리가 생성되었거나 심볼릭 링크가 존재할 경우, 파이썬 표준 라이브러리 `glob('maps/*')`는 디렉터리 경로까지 구분 없이 리스트에 포함함.
  - 파이썬 `setuptools`의 `data_files` 복사 루틴은 일반 파일(`regular file`)만 복사할 수 있으므로, 디렉터리 경로(`maps/install`)를 단일 파일처럼 복사하려다 `copy_file` 함수에서 시스템 예외를 발생시키며 전체 빌드가 중단된 것임.
- **수정한 파일 및 핵심 코드:**
  - 수정한 파일: [`capstone_ws/src/limo_capstone/setup.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/setup.py#L16-L22) (16~22행, `commit b46aba9`):
    ```python
    # 수정 전:
    (os.path.join('share', package_name, 'maps'), glob('maps/*')),

    # 수정 후: os.path.isfile(f) 조건문 필터링 적용
    (os.path.join('share', package_name, 'maps'), [f for f in glob('maps/*') if os.path.isfile(f)]),
    (os.path.join('share', package_name, 'worlds'), [f for f in glob('worlds/*') if os.path.isfile(f)]),
    (os.path.join('share', package_name, 'scripts'), [f for f in glob('scripts/*') if os.path.isfile(f)]),
    ```
- **수정 이유:**
  - `os.path.isfile(f)` 리스트 컴프리헨션을 적용하여 폴더 내에 예기치 않은 하위 디렉터리나 임시 폴더가 생성되더라도 순수 정규 파일만 안전하게 선별 복사하도록 방어적 빌드 설정을 구성하기 위함.
- **수정 후 확인한 결과:**
  - `colcon build --packages-select limo_capstone` 재실행 시 0.68초 만에 에러 없이 빌드가 완료됨.
  - `install/share/limo_capstone/maps/` 디렉터리에 `MC_3F_10_15.yaml`, `MC_3F_10_15.png`, `demo_map.yaml`, `domi_map.yaml` 등 13개의 지도 관련 파일이 정상 복사된 것을 `ls` 명령으로 직접 확인함 (`docs/DEVELOPMENT_LOG.md` L1237-1255).
- **아직 검증되지 않은 조건:**
  - `maps/` 내에 서브디렉터리 구조(예: `maps/floor3/map.yaml`)로 계층화하여 관리할 경우, 현재의 단일 계층 `os.path.isfile` 필터로는 서브디렉터리 내부 파일이 복사되지 않으므로 추가 디렉터리 순회 로직이 필요할 수 있음 (현재는 단일 계층 파일 구조만 검증됨).

---

#### 2. Nav2에서 필요한 TF 프레임 연결이 누락되어 위치 및 Costmap 관련 문제가 발생한 문제

- **발생한 증상:**
  - `limo_nav2.launch.py` 또는 `limo_auto_mapping.launch.py` 실행 시 Nav2 Costmap이 로봇 바닥 투영 위치를 확정하지 못하고, 터미널에 TF 룩업 타임아웃 경고가 지속적으로 출력되며 로봇이 지도상에서 올바른 위치에 매핑되지 않음.
- **문제를 발견한 로그 또는 관찰 내용:**
  - Nav2 bringup 실행 콘솔에서 다음 TF 변환 지연 및 드롭 경고 확인 (`docs/DEVELOPMENT_LOG.md` L1228-1234):
    ```text
    [amcl]: Timed out waiting for transform from base_link to odom to become available...
    [costmap_2d]: Message Filter dropping message: frame 'base_footprint' would require transform to 'map' which is not available
    ```
  - `2026_10_15` 디버깅 기록: `limo_auto_mapping.launch.py` 내에서 `explore_node`가 `static_tf` 노드보다 먼저 실행되어 시작 시점에 변환 조회를 실패하는 노드 기동 순서 문제 관찰.
- **원인 분석과 근거:**
  - LIMO 공식 하드웨어 드라이버인 `limo_driver.cpp`(598~627행)는 오도메트리 연산 시 `odom -> base_link` 변환만 브로드캐스트함.
  - 반면 Nav2 표준 스택 및 Costmap2D는 로봇의 2D 바닥면 투영 좌표계인 `base_footprint` 프레임을 기본 참조 프레임(`robot_base_frame`)으로 요구함.
  - 드라이버 레벨에서 `base_link -> base_footprint` 간의 변환을 제공하지 않았기 때문에 `map -> odom -> base_link`까지 연결되더라도 마지막 `base_footprint` 체인이 단절되어 Costmap이 센서 데이터를 로봇 바닥 중심에 투영하지 못함.
  - 또한 `limo_nav2.launch.py`에서 `bringup_launch.py`를 호출할 때 `params_file` 인자가 누락되어 커스텀 내비게이션 설정이 아닌 기본 파라미터로 구동되던 결함도 함께 확인됨.
- **수정한 파일 및 핵심 코드:**
  - 수정한 파일: [`capstone_ws/src/limo_capstone/launch/limo_nav2.launch.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/launch/limo_nav2.launch.py#L35-L54) 및 `limo_auto_mapping.launch.py` (`commit 7439841`):
    ```python
    # 1. base_link -> base_footprint 정적 TF 브로드캐스터 노드 추가
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    # 2. bringup_launch.py에 params_file 및 use_sim_time 인자 명시적 전달
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
        }.items()
    )
    ```
  - 아울러 `limo_auto_mapping.launch.py` 내 `LaunchDescription` 노드 등록 순서를 `slam_launch`, `nav2_navigation_launch`, `static_tf`, `explore_node` 순서로 정렬하여 `static_tf`가 `explore_node`보다 앞서 기동되도록 구성함.
- **수정 이유:**
  - 하드웨어 C++ 드라이버 소스코드를 임의로 수정하지 않고, 상위 런치 레벨에서 `tf2_ros`의 `static_transform_publisher`를 실행하여 `base_link`와 지면 기준점(`base_footprint`) 간의 항등 변환($x=0, y=0, z=0, \text{yaw}=0$)을 공급함으로써 표준 좌표계 체인(`map -> odom -> base_link -> base_footprint`)을 완성하기 위함.
- **수정 후 확인한 결과:**
  - `ros2 run tf2_ros tf2_echo base_link base_footprint` 조회 시 항등 변환이 실시간 정상 출력됨.
  - Costmap 및 AMCL에서 `base_footprint` 관련 TF 타임아웃 경고가 소멸되고, RViz2 상에서 로봇 다각형이 지도 좌표계에 실시간으로 정상 정합됨을 확인.
- **아직 검증되지 않은 조건:**
  - 로봇이 경사로나 턱을 주행하여 지면과 본체 간의 피치/롤 각도 변화가 발생하는 환경에서는 6축 정적 항등 변환($z=0, \text{rpy}=0$)이 실제 기구학적 오차를 유발할 수 있으나, 평탄한 실내 복도 외의 경사 지형에서는 검증되지 않음.

---

#### 3. `explore.cpp`에서 목표점이 반복 생성되어 탐색이 조기에 멈추던 문제

- **발생한 증상:**
  - 5m × 5m 밀폐 공간 시뮬레이션(`limo_auto_mapping_sim.launch.py`) 실행 시 로봇이 탐색을 시작한 직후 벽면으로 이동하지 않고 제자리에서 미세하게 전진/정지하다가, 탐색률이 16.8%에 불과한 초기 상태에서 탐색이 조기 종료됨.
- **문제를 발견한 로그 또는 관찰 내용:**
  - 터미널 콘솔 로그 관찰 (`docs/DEVELOPMENT_LOG.md` L1268-1308):
    ```text
    [explore-3]: Navigation succeeded
    [controller_server]: Reached the goal!
    [explore-3]: Navigation succeeded
    [controller_server]: Reached the goal!
    ... (동일 좌표 (0.27, 0.03)에 대해 초당 40회 이상 1,000회 연속 출력)
    [explore-3]: Exploration stopped.
    ```
- **원인 분석과 근거:**
  - 원인 분석 1 (가상 중심점 산출): 360도 전방향 라이다가 실내 4개 벽면 안쪽을 한 번에 스캔하면서, 아직 센서가 닿지 않은 벽면 외곽의 미탐색 셀 경계(Frontier)가 방 둘레를 따라 고리(Ring) 형태로 형성됨.
  - 원인 분석 2: `m-explore-ros2`의 원본 알고리즘인 `explore.cpp` 268행은 프론티어의 모든 셀 좌표를 산술 평균한 무게중심(`centroid`)을 목표점으로 선정함:
    ```cpp
    geometry_msgs::msg::Point target_position = frontier->centroid;
    ```
    고리 형태의 경계선 좌표를 산술 평균하면, 실제 미개척 경계선이 있는 벽면($x \approx \pm 2.5\,\text{m}$)이 아니라 이미 라이다로 스캔이 완료된 방 한가운데(로봇 위치 기준 전방 20cm 지점, `(0.27, 0.03)`)에 가상 목표점이 계산됨.
  - 원인 분석 3 (즉시 도달 판정 및 무한 반복): 로봇이 수 센티미터만 움직여도 Nav2의 도달 판정 반경(`xy_goal_tolerance = 0.08m`) 이내로 진입하여 즉시 `Reached the goal!`이 발생함.
  - 원인 분석 4: 목표에 성공적으로 도착했으나 실제 벽면 외곽의 프론티어 셀은 전혀 해소되지 않았고, `explore.cpp`는 성공한 목표를 블랙리스트에 추가하지 않으므로 동일한 가상 무게중심 좌표를 초당 40회씩 재발행하며 무한 루프에 빠짐.
- **수정한 파일 및 핵심 코드:**
  - 수정한 파일: [`capstone_ws/src/m-explore-ros2/explore/src/explore.cpp`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L268) (268행, `commit 7439841`):
    ```cpp
    // 수정 전 (Line 268): 가상 무게중심 지정으로 실내 고리형 프론티어에서 중심점 정체 유발
    // geometry_msgs::msg::Point target_position = frontier->centroid;

    // 수정 후 (Line 268): 로봇과 가장 가까운 실제 프론티어 경계 셀(middle)을 목표점으로 지정
    geometry_msgs::msg::Point target_position = frontier->middle;
    ```
  - 연관 수정: `visualizeFrontiers()`(196행)와 `makePlan()`(258행)의 블랙리스트 조회 좌표 기준도 `centroid`에서 `middle`로 통일하여 좌표 비교 일관성을 유지함.
- **수정 이유:**
  - 기하학적 평균값(`centroid`)은 닫힌 실내 공간에서 이미 탐색된 빈 공간 한가운데에 찍히는 치명적 결함이 있으므로, 프론티어 클러스터 상에 실제로 존재하는 물리적 경계 격자 셀(`frontier->middle`)을 직접 목표로 지정하여 로봇이 실제 미개척 벽면 경계선으로 이동하도록 유도하기 위함.
- **수정 후 확인한 결과:**
  - Gazebo Fortress 시뮬레이션 환경(5m × 5m 방, 210초 실행)에서 단위 검증을 수행함 (`docs/DEVELOPMENT_LOG.md` L1388-1411).
  - 기존 1,000회 이상 발생하던 `Reached the goal!` 무한 반복 횟수가 **0회**로 소멸됨.
  - 로봇이 실제 벽면 방향으로 1.0m 이상 횡단 주행을 수행하여 사각 외벽 4면의 격자를 모두 검출하고, 더 이상 탐색할 프론티어가 없을 때 `No frontiers found, stopping.` 로그를 출력하며 탐색이 정상 종료됨을 확인함.
- **아직 검증되지 않은 조건:**
  - 본 검증은 5m × 5m 정방형 시뮬레이션 방 환경에서만 확인된 결과임.
  - **실물 LIMO Pro 실차에 해당 바이너리를 탑재하여 미래관 실내 복도 및 실제 연구실에서 자율 탐색을 완주하는 실차 테스트는 아직 수행되지 않은 미검증 상태임.**
  - 복도 끝이 개방되어 있거나 동적 장애물(보행자)이 이동하는 복잡 환경에서 `middle` 선정 알고리즘이 로컬 미니마에 빠지는지 여부는 추가 검증이 필요함.

---

#### 4. (보조 디버깅) Nav2 시험 하네스(Phase 15-B) 타임아웃 및 좌표계 감산 오류 규명

- **발생한 증상:**
  - Phase 15 초기 내비게이션 자동 검증 시험에서 목표 주행이 Nav2 상태 `6` (`STATUS_ABORTED`)으로 종료되고, Odom 오차가 2.51m, 4.16m로 과도하게 보고되는 현상 발생 (`docs/DEVELOPMENT_LOG.md` L1640-1655).
- **원인 규명 및 분석:**
  1. 외부 시험 스크립트(`phase15_full_sim_verification.py` 419행)가 35초 타임아웃 도달 시 취소를 요청하면서 변수에 `GoalStatus.STATUS_ABORTED (6)`를 직접 대입하는 하드코딩 오류가 있었음 (Nav2 플래너 실패가 아닌 시험 도구의 취소 판정 오류).
  2. 스크립트 444행에서 `math.hypot(end_odom['x'] - g['x'], end_odom['y'] - g['y'])` 수식을 사용하여, 스폰 위치 기준의 **`odom` 프레임** 좌표와 전역 지도 기준의 **`map` 프레임** 좌표를 TF 변환 없이 직접 감산하여 발생한 수학적 허위 오차였음을 규명함.
- **수정 및 확인 결과:**
  - 시험 하네스 수식을 바로잡아 호스트 안전 타임아웃을 65초로 연장하고, TF 및 AMCL 기준 잔여 거리를 분리 측정하도록 보정한 후 단거리 주행($0.998\,\text{m}$) 시험을 재수행함.
  - 재수행 결과 Nav2 액션 서버가 **`STATUS_SUCCEEDED` (4)**를 정상 반환하며 도달 판정이 완료됨을 확인함 (`docs/DEVELOPMENT_LOG.md` L1668-1670).

---

## 3. 오류 해결에 사용한 주요 코드 및 수정 근거

본 프로젝트에서 발생한 빌드 에러, 좌표계 단절, 자율 탐색 무한 루프 등의 오류를 해결하기 위해 실제로 수정한 핵심 코드와 근거입니다.

### [코드 1] `setup.py` 리소스 파일 배포 시 디렉터리 제외 필터링 (`commit b46aba9`)
- **파일 경로:** [`capstone_ws/src/limo_capstone/setup.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/setup.py#L16-L22) (16~22행)
- **수정 전 문제:** `glob('maps/*')`가 빌드 임시 디렉터리(`maps/install` 등)를 포함하여 setuptools 파일 복사 에러(`error: can't copy 'maps/install'`)로 빌드가 실패하고 지도 파일이 배포되지 않음.
- **변경 내용 및 코드:**
  ```python
  data_files=[
      ('share/ament_index/resource_index/packages',
          ['resource/' + package_name]),
      ('share/' + package_name, ['package.xml']),
      (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
      (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
      # 수정: os.path.isfile() 조건문으로 순수 정규 파일만 선별 복사
      (os.path.join('share', package_name, 'maps'), [f for f in glob('maps/*') if os.path.isfile(f)]),
      (os.path.join('share', package_name, 'worlds'), [f for f in glob('worlds/*') if os.path.isfile(f)]),
      (os.path.join('share', package_name, 'scripts'), [f for f in glob('scripts/*') if os.path.isfile(f)]),
  ],
  ```
- **변경 이유:** 하위 디렉터리 복사 예외를 차단하여 `colcon build`를 정상화하고, `install/share/limo_capstone/maps/` 디렉터리에 13개 지도 파일이 정상 설치되도록 함.

---

### [코드 2] `limo_nav2.launch.py` 내 `base_footprint` TF 추가 및 파라미터 전달 (`commit 7439841`)
- **파일 경로:** [`capstone_ws/src/limo_capstone/launch/limo_nav2.launch.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/launch/limo_nav2.launch.py#L35-L54) (35~54행)
- **수정 전 문제:** LIMO 베이스 드라이버가 `odom -> base_link`만 발행하여 Nav2 및 Costmap에 필수적인 `base_footprint` TF가 누락되어 좌표계 단절 및 타임아웃이 발생하고, `params_file` 인자가 전달되지 않음.
- **변경 내용 및 코드:**
  ```python
  # base_link -> base_footprint 정적 TF 브로드캐스터 (AMCL 및 Costmap 필수 프레임 제공)
  static_tf = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='static_tf_pub',
      arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
  )

  # Nav2 bringup 런치 포함 (params_file 명시적 전달)
  nav2_bringup_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
      ),
      launch_arguments={
          'map': LaunchConfiguration('map'),
          'params_file': LaunchConfiguration('params_file'),
          'use_sim_time': LaunchConfiguration('use_sim_time'),
          'autostart': 'true',
      }.items()
  )
  ```
- **변경 이유:** 상위 AMCL 및 Costmap이 참조할 수 있는 표준 2D 지면 투영 좌표계를 런치 레벨에서 제공하여 좌표계 체인을 완성하고, 커스텀 파라미터가 누락 없이 적용되도록 함.

---

### [코드 3] `explore.cpp` 프론티어 목표 선정 기준 변경 (문제 C 해결, `commit 7439841`)
- **파일 경로:** [`capstone_ws/src/m-explore-ros2/explore/src/explore.cpp`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L267-L271) (267~271행)
- **수정 전 문제:** 실내 밀폐 공간에서 둘레 벽면 프론티어의 기하학적 무게중심(`centroid`)이 로봇 바로 앞 빈 공간에 형성되어 로봇이 벽면으로 전진하지 못하고 초당 40회씩 `Reached the goal!`을 출력하는 무한 루프 발생.
- **변경 내용 및 코드:**
  ```cpp
  // 수정 전 (Line 268): 가상 무게중심을 목표로 선정하여 제자리 정체 유발
  // geometry_msgs::msg::Point target_position = frontier->centroid;

  // 수정 후 (Line 268): 로봇과 가장 가까운 실제 프론티어 경계 셀(middle)을 목표점으로 지정
  geometry_msgs::msg::Point target_position = frontier->middle;
  ```
- **변경 이유:** 가상 중심점이 아닌 프론티어 클러스터 상의 실제 물리적 격자점(`middle`)을 목표로 지정하여 로봇이 실제 미개척 벽면 방향으로 전진 주행하도록 수정함.

---

### [코드 4] `explore.cpp` 블랙리스트 조회 좌표 일원화 (`commit 7439841`)
- **파일 경로:** [`capstone_ws/src/m-explore-ros2/explore/src/explore.cpp`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L196) (196행, 258행)
- **수정 전 문제:** 목표 발행 좌표는 `middle`로 변경되었으나, 시각화(`visualizeFrontiers`) 및 계획 수립(`makePlan`) 단계의 블랙리스트 조회 좌표가 여전히 `centroid`로 남아 있을 경우, 실패한 목표점을 블랙리스트에서 정상 조회하지 못해 무한 재시도가 발생할 위험이 있음.
- **변경 내용 및 코드:**
  ```cpp
  // 196행: 시각화 단계 블랙리스트 검사 좌표를 middle로 통일
  if (isPointBlacklisted(frontier.middle)) {
      continue;
  }

  // 258행: 플랜 생성 단계 블랙리스트 검사 좌표를 middle로 통일
  if (isPointBlacklisted(frontier.middle)) {
      continue;
  }
  ```
- **변경 이유:** 목표 지정 좌표와 실패 시 블랙리스트 등록 및 조회 좌표를 동일한 물리 좌표계(`middle`)로 일치시켜 탐색 실패 목표 재방문 방지 로직의 신뢰성을 확보함.

---

### [코드 5] `limo_base.launch.py` 시리얼 포트 기본값 및 인자 전달 구조
- **파일 경로:** [`src/limo_ros2/limo_base/launch/limo_base.launch.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/src/limo_ros2/limo_base/launch/limo_base.launch.py#L13-L26) (13~26행)
- **수정 전 문제:** 온보드 리눅스에 `/dev/ttylimo` 심볼릭 링크가 존재하지 않아 기본값 실행 시 드라이버가 종료됨.
- **변경 내용 및 코드:**
  ```python
  port_name_arg = DeclareLaunchArgument('port_name', default_value='ttyUSB1',
                                       description='usb bus name, e.g. ttyUSB1')
  port_name_arg = LaunchConfiguration('port_name', default='ttylimo')
  ...
  parameters=[{
      'port_name': port_name_arg,
      'odom_frame': odom_frame_arg,
      'base_frame': base_link_frame_arg,
      'pub_odom_tf': pub_odom_tf_arg_,
      'use_mcnamu': False
  }]
  ```
- **변경 이유:** 하드웨어 포트 변동 시 드라이버 소스코드 수정 없이 외부에서 `port_name:=ttyUSB1` 인자를 넘겨 유연하게 시리얼 장치에 바인딩할 수 있도록 함.

---

### [코드 6] `limo_auto_mapping.launch.py` 내 노드 선언 순서 보완
- **파일 경로:** [`capstone_ws/src/limo_capstone/launch/limo_auto_mapping.launch.py`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/launch/limo_auto_mapping.launch.py#L54-L66) (54~66행)
- **수정 전 문제:** `explore_node`가 `static_tf` 노드보다 먼저 구동될 경우 초기 구동 시점에 `base_footprint` TF 조회를 실패하여 노드가 대기 또는 에러 상태로 진입함.
- **변경 내용 및 코드:**
  ```python
  # static_tf를 explore_node보다 앞서 선언 및 배치
  return LaunchDescription([
      slam_launch,
      nav2_navigation_launch,
      static_tf,
      explore_node
  ])
  ```
- **변경 이유:** TF 브로드캐스터가 먼저 퍼블리시를 개시하여 탐색 노드가 시작 즉시 좌표계 변환을 안정적으로 조회할 수 있도록 기동 순서를 보장함.

---

## 4. 증빙 시각 자료 및 문제 해결 연계 분석

| 그림 번호 | 캡션 | 파일 경로 | 문제 해결 연계 분석 내용 |
|---|---|---|---|
| **그림 1** | 미래관 3층 실차 매핑 취득 지도 (`MC_3F_10_15`) | [`capstone_ws/src/limo_capstone/maps/MC_3F_10_15.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/maps/MC_3F_10_15.png) | **실제 매핑 결과물:** 10월 15일 LIMO Pro 실차 주행을 통해 취득된 $764 \times 809$ ($38.2\,\text{m} \times 40.45\,\text{m}$) 점유격자지도. 6주차 수동/실차 매핑으로 취득된 원본 데이터. |
| **그림 2** | RViz2 기반 미래관 3층 지도 로딩 및 레이저 스캔 정합 화면 | [`pictures/limo map_MC3F_1.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/pictures/limo%20map_MC3F_1.png) | **오류 해결 확인:** `setup.py` 파일 필터링 수정 후 `MC_3F_10_15` 지도가 정상 설치되어 RViz2 Map Display에 로드되고 실시간 `/scan`이 정합된 화면 (2026-10-16 13:07). |
| **그림 3** | 미래관 3층 복도 내비게이션 경로 계획 화면 | [`pictures/limo map MC3F_2.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/pictures/limo%20map%20MC3F_2.png) | **기능 동작 확인:** 정적 지도 상에서 Nav2 글로벌 플래너가 계산한 복도 경로 라인을 따라 로봇이 이동 궤적을 형성하고 있는 RViz2 화면 (2026-10-16 13:14). |
| **그림 4** | 자동 매핑 시스템의 실제 노드·토픽 연결 그래프 | [`pictures/limo graph_1.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/pictures/limo%20graph_1.png) | **파이프라인 점검:** `limo_auto_mapping.launch.py` 구동 중 `rqt_graph`로 캡처한 런타임 토폴로지. 드라이버, SLAM, Explore, Nav2 노드의 실시간 연결 확인. |
| **그림 5** | 4주차 작성 실내 점유격자지도 (`domi_map`) | [`capstone_ws/src/limo_capstone/maps/domi_map.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/maps/domi_map.png) | **4주차 산출물:** 4주차(10/05) QoS 문제 해결 후 수동 조작 주행으로 취득된 도미토리 룸 점유격자지도. |
| **그림 6** | 5주차 사각 방 점유격자지도 (`demo_map`) | [`capstone_ws/src/limo_capstone/maps/demo_map.png`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/maps/demo_map.png) | **5주차 시험용 지도:** 정적 내비게이션 초기 연동 시험에 사용된 실내 격자 데이터. |

---

## 5. 오류 수정 효과 확인을 위한 보조 검증 데이터 및 미검증 한계

본 절의 수치는 자율주행의 완성도를 입증하는 성과 지표가 아니며, **7주차에 적용한 소스 코드 수정(`explore.cpp` 목표점 변경, `setup.py` 파일 필터링 등)이 실제 결함을 해소했는지를 확인하기 위해 측정한 보조 검증 데이터**입니다.

### 5.1 자율 탐색 목표점 수정 전후 비교 데이터 (문제 C 오류 소멸 확인용)
Gazebo Fortress 시뮬레이션 환경(5m × 5m 방, 210초 실행)에서 `explore.cpp`의 목표점을 `middle`로 수정한 전후의 실측 비교 데이터입니다.

| 평가 항목 | 수정 전 (`centroid`) | 수정 후 (`middle`) | 오류 수정 효과 확인 내용 |
|---|:---:|:---:|:---|
| **목표 반복 횟수 (무한 루프)** | 1,000회 이상 (초당 40회) | **0회** | **`Reached the goal!` 무한 루프 완전 소멸 확인** |
| **로봇 실제 횡단 주행** | 0.20m 미만 (제자리 정체) | **1.0m 이상** 횡단 주행 | 가상 중심점 정체에서 벗어나 실제 벽면 이동 확인 |
| **미탐색 격자 셀 (UNKNOWN, -1)** | 7,832개 (83.2%) | 1,755개 (15.0%) | 외벽 방향으로 이동하여 미개척 셀 축소 확인 |
| **자유 공간 격자 셀 (FREE, 0)** | 1,569개 (16.7%) | 8,823개 (75.6%) | 라이다 스캔 확장을 통한 지도 작성 진행 확인 |
| **장애물 벽면 셀 (OCCUPIED, 100)**| 8개 (매핑 조기 실패) | 1,086개 (외벽 4면 검출) | 실내 사각 외벽 4면 정상 매핑 확인 |
| **탐색 종료 감지** | 조기 비정상 정지 | 정상 자동 종료 | `No frontiers found, stopping.` 로그 확인 |

### 5.2 Nav2 시험 하네스 수식 보정 후 단거리 주행 확인 데이터 (Phase 15-C)
미래관 3층 실차 지도 기반 가상 월드(`mc3f_10_15.sdf`)에서 시험 하네스의 프레임 직접 감산 및 타임아웃 오류를 수정한 후 수행한 Goal 1 단거리($0.998\,\text{m}$) 주행 실측 데이터입니다.

| 검증 지표 | 목표 기준치 | 실측 데이터 | 확인 결과 |
|---|:---:|:---:|:---|
| **Nav2 액션 완료 상태** | `STATUS_SUCCEEDED` (4) | **`STATUS_SUCCEEDED` (4)** | 정상 완료 (하네스 타임아웃 오판정 소멸) |
| **물리 주행 거리 (Odom)** | 0.9980m | 0.9970m | 단거리 직선 주행 수행 확인 |
| **헤딩 각도 오차** | $< 14.3^\circ$ | $6.81^\circ$ (0.1189 rad) | 허용 각도 오차 범위 내 주행 확인 |
| **TF 기준 유클리드 위치 오차** | $< 0.08\,\text{m}$ (8.0cm) | $0.1401\,\text{m}$ (14.01cm) | 조건부 도달 (+6cm 잔여 오차) |
| **AMCL 파티클 위치 오차** | 수렴 확인 | $0.2784\,\text{m}$ (27.84cm) | 파티클 분포 수렴 확인 |

### 5.3 시뮬레이션과 실차 환경의 엄격한 분리 및 미검증 한계 목록
- **시뮬레이션 상에서 확인된 사실:**
  - `setup.py` 파일 필터링 수정을 통한 패키지 정상 빌드 및 13개 지도 파일 배포 확인.
  - 가상 환경(5m × 5m 방)에서 `explore.cpp`의 목표점 수정 후 `Reached the goal!` 무한 루프 0회 소멸 확인.
  - 가상 환경(미래관 3층 SDF)에서 Nav2 단거리 목표에 대해 액션 완료(`STATUS_SUCCEEDED`) 반환 확인.
- **실물 LIMO Pro 및 미검증 한계 목록 (솔직한 한계 명시):**
  - **실물 LIMO Pro 실차 주행 검증: 미검증.**  
    7주차의 코드 패치(`middle` 전략 및 TF 보정)는 Gazebo 시뮬레이션에서 단위 검증되었으며, **실물 LIMO Pro 로봇에 패키지를 올려 실제 복도에서 주행시킨 실차 검증은 아직 수행되지 않은 '미검증' 상태임.**
  - **원점 복귀 기능 (`return_to_init`): 미검증.**  
    `explore_lite` 파라미터에 원점 복귀 기능이 존재하나, 현재 기본값(`false`)으로 비활성화되어 동작 여부가 검증되지 않음.
  - **중거리 목표 (Goal 2): 미검증.**  
    Phase 15-C에서는 단거리 Goal 1만 재검증하였으며, 긴 복도 횡단을 포함하는 Goal 2는 아직 검증되지 않은 상태로 잔류함.

---

## 6. 학교 포털 주차별 입력 초안 (문제 해결 중심)

학교 학사 포털 시스템에 직접 입력할 때 복사하여 활용할 수 있도록 정리한 텍스트입니다.

```text
================================================================================
[1주차]
- 상태: 완료
- 일자: 2026.09.05 (사용자 수정 가능)
- 활동내용: 캡스톤디자인 1 팀 편성(팀명: LIMO의 모험, 4조), 실내 자율 순찰 및 안내 프로젝트 주제 선정, 팀원별 세부 역할 분담(자율주행, SLAM, 시스템 구축, 애플리케이션) 완료.
- 기타사항: 기획 및 준비 단계로 확인된 기술적 오류 없음.
- 주차별 교수 지도결과: [확인 필요]
--------------------------------------------------------------------------------
[2주차]
- 상태: 완료
- 일자: 2026.09.12 (사용자 수정 가능)
- 활동내용: 캡스톤디자인 팀별 계획서 작성 및 제출, 4단계 개발 일정 수립, GitHub 형상 관리 저장소 초기화.
- 기타사항: 계획서의 최종 목표와 실제 주차별 진행 상황을 구분하여, 검증된 사실만 보고한다는 개발 관리 원칙 정립.
- 주차별 교수 지도결과: [확인 필요]
--------------------------------------------------------------------------------
[3주차]
- 상태: 완료
- 일자: 2026.09.30 (사용자 수정 가능)
- 활동내용: LIMO 온보드 PC(Ubuntu 22.04, ROS 2 Humble) 개발환경 구축, LIMO 공식 패키지 구조 분석, LIMO Base(ttyUSB1) 및 YDLiDAR(ttyUSB0) 시리얼 드라이버 구동 확인.
- 기타사항: NoMachine 10 무료 라이선스 제약으로 버전 9 다운그레이드 사용, ttylimo 심볼릭 링크 부재로 런치 파라미터 port_name:=ttyUSB1 직접 지정하여 연결.
- 주차별 교수 지도결과: [확인 필요]
--------------------------------------------------------------------------------
[4주차]
- 상태: 완료
- 일자: 2026.10.05 (사용자 수정 가능)
- 활동내용: LIMO 수동 조작 기반 2D SLAM 매핑 시험, domi_map 작성, /scan BEST_EFFORT QoS와 RViz 간 비호환 문제 해결, 온보드 시리얼 포트 물리 경로 조사.
- 기타사항: 포트 1번이 2번으로 표시되는 혼선 발생(포트 인식 문제로 추정), 수동 매핑 중 백그라운드 중복 프로세스 종료 후 맵 저장 완료.
- 주차별 교수 지도결과: [확인 필요]
--------------------------------------------------------------------------------
[5주차]
- 상태: 완료
- 일자: 2026.10.12 (사용자 수정 가능)
- 활동내용: 기 작성된 점유격자지도(demo_map, MC_2F_10_07) 로드, limo_nav2.launch.py 작성 및 Nav2 Bringup 연동, RViz2 2D Goal Pose 기반 주행 시험.
- 기타사항: 목표점 전송은 자체 개발 노드가 아닌 RViz2 표준 GUI 플러그인을 활용함. 초기 위치 미부여 시 AMCL 파티클 미수렴으로 경로 계획이 실패하는 현상 확인 및 해결.
- 주차별 교수 지도결과: [확인 필요]
--------------------------------------------------------------------------------
[6주차]
- 상태: 완료
- 일자: 2026.10.15 (사용자 수정 가능)
- 활동내용: SLAM Toolbox + Nav2 + Explore Lite 결합 자동 매핑 파이프라인(limo_auto_mapping.launch.py) 구축, 미래관 3층 실차 매핑(MC_3F_10_15) 수행.
- 기타사항: setup.py 디렉터리 복사 오류로 인한 지도 로딩 실패, base_footprint TF 누락으로 인한 위치 불일치, 프론티어 무게중심(centroid)으로 인한 조기 정지(문제 C) 원인 규명.
- 주차별 교수 지도결과: [확인 필요]
--------------------------------------------------------------------------------
[7주차]
- 상태: 완료
- 일자: 2026.10.17 (사용자 수정 가능)
- 활동내용: 6주차 도출 3대 결함 소스 코드 수정(setup.py 파일 필터링 추가, limo_nav2 정적 TF 추가, explore.cpp 프론티어 middle 전략 적용) 및 단위 검증 수행.
- 기타사항: 시뮬레이션 환경에서 Reached the goal! 초당 40회 무한 루프 0회 소멸 확인 및 외벽 4면 정상 매핑 확인. 실물 LIMO 온보드 주행 및 원점 복귀는 향후 과제로 분류(미검증).
- 주차별 교수 지도결과: [확인 필요]
================================================================================
```

---

## 7. 자료 출처 및 미확인 항목 목록

1. **자료 출처 구분:**
   - `[저장소 확인]`: `src/limo_ros2/`, `capstone_ws/src/`, `pictures/` 디렉터리 내의 실제 코드, 런치 파일, 맵 이미지 및 실행 화면.
   - `[개발 기록 확인]`: `docs/DEVELOPMENT_LOG.md`, `docs/NAV2_TROUBLESHOOTING.md`의 Phase 12~15 실험 및 디버깅 로그.
   - `[팀원 저장소 참고]`: `https://github.com/mavic89k/limo_capstone` (초기 맵핑 및 설정 이력).
   - `[사용자 제공 사실]`: NoMachine 10 버전 라이선스 제약 및 9 버전 다운그레이드 사용 사실.
   - `[포트 인식 문제로 추정]`: 온보드 시리얼 포트 1번 $\rightarrow$ 2번 인식 변동 원인.
2. **미확인 항목 (솔직한 한계 명시):**
   - **주차별 교수 지도결과:** 이창훈 지도교수님의 공식 멘토링 평가 및 피드백 내용은 포털 입력 시 확인 후 직접 기입 필요 (`[확인 필요]`로 유지).
   - **실물 LIMO Pro 실차 자율 탐색 및 주행:** 7주차의 코드 수정은 시뮬레이션에서 검증되었으며, 실물 로봇을 통한 최종 실차 검증은 미검증 상태임.
   - **원점 복귀 (`return_to_init`):** 알고리즘 파라미터는 존재하나 비활성화되어 미검증 상태임.
