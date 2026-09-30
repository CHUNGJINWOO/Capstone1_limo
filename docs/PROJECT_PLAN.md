# 프로젝트 계획

## 1. 프로젝트 개요

- 학년도: 2026학년도 2학기
- 학과: 드론로봇공학과
- 팀명: LIMO의 모험
- 과제명: LIMO를 이용한 실내 자율 순찰 및 안내
- 지도교수: 이창훈
- 로봇: AgileX LIMO Pro
- 기반 플랫폼: ROS 2

## 2. 프로젝트 목표

ROS 2 기반 LIMO Pro를 이용하여 실내 환경에서
자율 순찰 및 목적지 안내가 가능한 시스템을 구현한다.

주요 목표:

1. ROS 2 기반 LIMO 시스템 구축
2. 실내 SLAM 및 지도 작성
3. 지도 기반 Localization
4. Nav2 기반 자율주행
5. 전역/지역 경로 계획
6. 동적 장애물 회피
7. Waypoint 기반 자율 순찰
8. 목적지 안내
9. 실제 환경 필드 테스트 및 검증

## 3. 팀 역할

### 정진우 — 자율주행 및 동적 장애물 회피

- Nav2 기반 AMCL 파라미터 조정
- Global / Local Path Planning
- 경로 추종
- 동적 장애물 회피
- 주행 파라미터 최적화
- 실제 환경 주행 테스트

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

- ROS 2 Action 기반 Waypoint 순찰
- 사용자 호출 기반 안내 기능
- 필드 테스트
- 테스트 데이터 취합 및 검증

## 4. 전체 시스템 구성

```text
LiDAR / Depth Camera / IMU
          ↓
     Sensor Data
          ↓
   SLAM / Localization
          ↓
     Navigation2
          ↓
    Path Planning
          ↓
  Obstacle Avoidance
          ↓
     Patrol / Guide

각 구성요소는 담당 영역에 따라 개발한 후 단계적으로 통합한다.
5. 개발 단계
Phase 1 — LIMO / ROS 2 기본 기능 확인
- LIMO ROS 2 패키지 확인
- 기본 구동 확인
- Topic / TF 구조 확인
- 기본 제어 확인
Phase 2 — LiDAR
- /scan 확인
- sensor_msgs/msg/LaserScan 데이터 확인
- LaserScan 데이터 분석
- 기본 장애물 검출 실험
Phase 3 — SLAM
- SLAM Toolbox 적용
- 실내 지도 작성
- TF 구조 검증
- 지도 저장
Phase 4 — Localization
- 저장된 지도 사용
- AMCL 적용
- 위치 추정 확인
- AMCL 파라미터 조정
Phase 5 — Navigation2
- Nav2 구성
- Global Planner 확인
- Local Planner / Controller 확인
- Costmap 구성
- 경로 추종 확인
Phase 6 — 동적 장애물 회피
- 정적 장애물 대응
- 동적 장애물 실험
- 회피 및 재계획
- 주행 파라미터 조정
Phase 7 — Autonomous Patrol
- Waypoint 구성
- 다중 목적지 순차 이동
- 반복 순찰
Phase 8 — Guide
- 목적지 입력
- Navigation Goal 생성
- 목적지 이동
- 취소 및 예외 처리
Phase 9 — 통합 테스트
- 실제 실내 환경 테스트
- 주행 성공률 측정
- 장애물 회피 실험
- 문제점 분석
- 파라미터 최적화
6. 개발 환경
실제 ROS 2 및 LIMO 기능 개발과 검증은
학교 컴퓨터에서 우선 진행한다.
필요한 경우 LIMO 자체 컴퓨터에서
실제 로봇과 센서를 이용하여 개발 및 검증한다.
7. 개발 원칙
각 기능은 독립적으로 검증한 후 단계적으로 통합한다.
계획서에 정의된 기능은 구현 완료를 의미하지 않는다.
실제로 확인하지 않은 기능은 완료된 기능으로 기록하지 않는다.
오류가 발생한 경우 원인과 해결 과정을 개발 기록에 남긴다.
주요 실험 결과는 DEVELOPMENT_LOG.md 및
향후 작성할 실험 기록에 남긴다.
8. 최종 평가 항목
- 목적지 도착 성공률
- 평균 이동 시간
- 평균 주행 거리
- 장애물 회피 성공 여부
- 반복 주행 안정성
- Patrol 성공 여부
- Guide 성공 여부
