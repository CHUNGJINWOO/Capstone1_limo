# Development Log

## 2026-09-30 — Repository 및 프로젝트 문서 정리

LIMO ROS 2 패키지는 다음과 같이 구성되어 있다.
- limo_base
- limo_car
- limo_description
- limo_msgs
LIMO ROS 2 패키지 확인
LIMO ROS 2 패키지의 주요 구성요소를 확인하였다.
- LIMO base driver
- Serial communication
- TF 관련 코드
- LIMO message
- Ackermann model
- Gazebo simulation resources
- RViz configuration
- LiDAR 관련 launch 구성
프로젝트 문서 정리
기존 프로젝트 문서를 검토하고
계획과 실제 개발 기록을 분리하기 위해 문서 구조를 재정리하였다.
현재 주요 문서:
- docs/PROJECT_PLAN.md
- docs/DEVELOPMENT_LOG.md
PROJECT_PLAN.md는 프로젝트의 목표와 개발 단계를 기록한다.
DEVELOPMENT_LOG.md는 실제로 수행한 개발과 검증 결과를 기록한다.
Ackermann 모델 수정
limo_car/gazebo/ackermann_with_sensor.xacro를 수정하였다.
기존 파일에는 Ackermann 모델을 실제로 생성하는
xacro:limo_ackermann 인스턴스가 없었다.
다음 내용을 추가하였다.
<xacro:limo_ackermann/>

이를 통해 센서가 포함된 Ackermann 모델 구성에서
Ackermann 모델 인스턴스가 생성되도록 수정하였다.
개발 과정에서 실제로 확인하지 않은 기능은
완료된 것으로 기록하지 않는다.
개발 기록 원칙
- 실제 실행한 명령만 기록한다.
- 실제 확인한 결과만 완료로 기록한다.
- 계획과 구현 결과를 구분한다.
- 오류가 발생하면 원인과 해결 과정을 기록한다.
- 코드 변경과 테스트 결과를 함께 기록한다.
- 실제 주행 및 센서 검증 결과는 해당 환경에서 직접 확인한 후 기록한다.
