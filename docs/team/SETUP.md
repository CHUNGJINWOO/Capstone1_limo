# 설치 및 재현 준비

## 범위

Ubuntu 22.04와 ROS 2 Humble 환경을 대상으로 한다. 이 절차는 저장소의 일부 패키지를 빌드해 보는 개발 환경 준비이며, 실차 운영본의 재현이나 센서 기동을 보증하지 않는다. 현장 배포본의 누락은 [기준선](STATUS.md)을 확인한다.

## ROS 2와 소스

ROS 2 Humble의 공식 설치를 마치고 `rosdep`, `colcon`, Git을 준비한다. 팀에서 승인한 저장소 URL로 clone한 뒤 저장소 루트에서 다음을 실행한다. 이미 clone했다면 첫 줄은 생략한다.

```bash
git clone <REPOSITORY_URL> capstone-limo
cd capstone-limo
source /opt/ros/humble/setup.bash
colcon list --base-paths capstone_ws/src
```

성공 조건은 `limo_capstone`, `limo_depth_obstacles` 등이 목록에 나타나는 것이다. `colcon` 또는 ROS 명령이 없으면 설치와 셸 source 상태를 확인한다. 패키지 목록 출력은 빌드 성공이 아니다.

## 의존성과 빌드

[외부 소스 목록](../third_party.repos)은 브랜치 이름을 포함하므로 commit 고정 재현 자료가 아니다. AgileX `limo_description`과 실차의 `limo_base`, Astra 드라이버, YDLidar SDK 등은 별도 의존성이다. 중복 패키지를 가져오지 말고 각 프로젝트 라이선스와 현장 검증 버전을 확인한다. GitHub 코드만으로 실제 장비용 조합을 재현할 수 없다.

패키지 메타데이터의 시스템 의존성을 검토한 뒤 인터넷과 관리 권한이 허용된 개발 머신에서만 실행한다.

```bash
rosdep install --from-paths capstone_ws/src --ignore-src -r -y
cd capstone_ws
colcon build --symlink-install --packages-up-to limo_depth_obstacles limo_capstone
source install/setup.bash
ros2 pkg prefix limo_capstone
ros2 pkg prefix limo_depth_obstacles
```

`rosdep`에서 해결되지 않는 SDK·외부 소스는 오류를 기록하고 중단한다. 별도의 깨끗한 환경에서 `--packages-select`로 두 패키지만 빌드하면 작업공간 내 의존 패키지의 설치 파일이 없어 실패했다. `--packages-up-to`도 SDK 등 필수 의존성이 준비되어야 한다. `colcon`은 패키지별 성공을 확인하고 두 `ros2 pkg prefix`가 이 워크스페이스 install을 가리켜야 한다. 기존 드라이버 overlay가 필요한 경우 `/opt/ros/humble` → 검증된 벤더 overlay → 현재 `install/setup.bash` 순서로 source하고 `ros2 pkg prefix`와 라이브러리 경로를 확인한다. 순서나 중복 패키지에 따라 선택되는 코드가 달라진다.

## 네트워크·장치

Tailscale은 팀 관리자가 승인한 계정과 기기 연결 절차를 사용한다. 인증키, 내부 IP, 토큰은 저장소에 기록하지 않는다. ROS_DOMAIN_ID, discovery, Fast DDS XML은 현장 승인된 프로파일을 프로세스 단위로 맞춘다. 기존 16 MiB 실험을 전역 설정으로 복사하거나 무조건 적용하지 않는다.

장치 확인은 현장 담당자의 승인 후 해당 컴퓨터에서 `lsusb` 및 `ls -l /dev/serial/by-id/`로 시작한다. 경로는 연결 순서에 따라 바뀔 수 있어 특정 tty를 단정하지 않는다. 모터 차단은 물리적으로 확인한다. 실제 ROS 센서·실차 launch는 별도 승인 없이는 시작하지 않는다.

설치 후 실행 파일만 확인할 때는 `ros2 pkg executables limo_capstone`과 `ros2 pkg executables limo_depth_obstacles`를 사용한다. 결과는 실행 가능 여부를 확인한 것이며 하드웨어 동작 검증이 아니다.
