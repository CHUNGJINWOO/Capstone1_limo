# 대표 실행 요약

원본 기록의 주요 필드를 전사했다. 시간과 위치 관련 수치는 당시 Gazebo 시험 기준이다.

| 항목 | run1 | run3 |
|---|---|---|
| stop_reason | exploration_stopped | gate_failsafe_held |
| wall_s / sim_s | 230.7 / 211.0 | 18.3 / 31.0 |
| map known m² | 104.355 | 61.753 |
| map updates | 231 | 18 |
| goal succeeded / aborted / canceled | 0 / 29 / 1 | 0 / 9 / 0 |
| FAILSAFE events | 0 | 1 |
| 마지막 Gate 상태 | NORMAL | FAILSAFE_SENSOR_TIMEOUT |
| 보고서상 이동 | 16.1 m | 0.05 m |
| summary의 gt_path_m | 16.07 m | 0.01 m |

run3의 이동 수치 차이는 확인되지 않았다. 서로 다른 측정·집계일 가능성을 단정하지 않으며 원자료는 로컬에 보존한다. `ABORTED`에는 당시 목표 교체가 포함될 수 있다.
