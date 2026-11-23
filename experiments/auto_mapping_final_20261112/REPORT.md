# Gazebo 차동구동 자동 맵핑 시험 (2026-11-12)

ROS 2 Humble과 Ignition Fortress의 lounge 월드에서 기존 `limo_integrated_auto_mapping_sim_diff.launch.py`의 SLAM·Nav2·explore_lite 및 Safety Gate를 사용했다. 측정값은 당시 Gazebo 시험 기록이며 실차 주행 결과가 아니다.

| 실행 | 종료와 조건 | 시뮬레이션 시간 | 경로 길이 | 방 내부 알려진 영역 | Gate FAILSAFE |
|---|---|---:|---:|---|---:|
| run1 | 기존 launch, explore 종료 | 211 s | 16.1 m | 시작 자세 정렬 94.1%, 벽 맞춤 정렬 99.0% | 0 |
| run3 | 기존 launch, Gate FAILSAFE 유지로 중단 | 31 s | 기록 간 차이, 아래 설명 | 시작 자세 정렬 58.4%, 벽 맞춤 정렬 59.0% | 1 |

run1의 known 영역은 약 104.36 m², 지도 갱신 231회로 기록됐다. Goal 상태는 SUCCEEDED 0, ABORTED 29, CANCELED 1이었으며 당시 반복 Goal 교체가 ABORTED에 포함됐다. 이동 및 지도 확장만으로 개별 Fleet 임무 성공을 의미하지 않는다.

run3에서는 기동 약 17.42초에 `FAILSAFE_SENSOR_TIMEOUT`으로 전환됐고, 로그에 Depth PointCloud age 0.400초가 0.38초 한도를 초과한 사건이 있었다. 상태 로그에서 다음 메시지의 Depth는 정상으로 나타났지만 Gate는 래치된 채 중단됐다. 요약의 `gt_path_m`은 0.01 m, 별도 보고 표의 이동은 0.05 m다. 집계 차이의 원자료 검증이 이 선별 기록에 없어 둘을 임의로 통일하지 않았다.

커버리지 분모는 가구를 제외하지 않은 벽 내부 103.84 m²이다. 시작 자세 정렬은 초기 GT/SLAM 자세를 사용하고 벽 맞춤 정렬은 벽에 추가 정합하므로 값이 다르다. 한 월드의 소수 시험으로 일반 성공률을 추정할 수 없다. 좁은 공간, 회전 오차, 실차 센서·모터 구동의 안전성은 검증 범위 밖이다.

자동 복구 정책을 사용한 변형 시험도 별도로 존재했으나 그 실행에서 FAILSAFE가 발생하지 않아 자동 복구 동작은 검증되지 않았다. 변형 launch 및 Gate 설정은 이 선별 반영에 포함하지 않았다.

## 기준선 재현

ROS 2 Humble, Ignition Fortress 및 빌드된 `capstone_ws/install`이 필요하다. 기존 패키지·월드와 [기존 수동 관측 monitor](../safety_gate_verification_20261110/scripts/auto_mapping_monitor.py)를 재사용한다. 새로운 실행 태그를 사용하며 기존 결과 폴더가 있으면 스크립트가 거부한다.

```bash
bash experiments/auto_mapping_final_20261112/scripts/run_final.sh <새_태그> 1200
```

스크립트는 기본 ROS domain 121과 localhost를 사용한다. 기존 런치와 당시 기록 시점 코드의 차이 및 운영 환경을 확인해야 하며, 현재 재현 결과가 과거 수치와 같다고 가정하지 않는다. 실행 중에는 자신이 시작한 launch 및 bag 프로세스 그룹에 종료 신호를 보낸다. 실행 원본 bag·지도·이미지·로그는 이 선별 반영에 포함하지 않는다. 이전 run1·run3의 공개 요약은 [결과 요약](RESULTS.md)에 보존했다.
