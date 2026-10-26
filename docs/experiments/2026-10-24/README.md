# 복합 장애물 시뮬레이션 기록 (2026-10-24)

2026-10-25에 기존 실행 자료를 정리했다. 새 주행을 수행한 결과가 아니다. 아래 12개 실행의 `summary.json`과 보조 메타데이터는 원본 바이트 그대로 복사했다. 최초 실패와 후속 시험을 함께 보관하며, 이를 동일 조건의 반복 성공률로 합산하지 않는다.

## 조건별 기록

단위는 시간 s, 거리 m이다. 표의 간격은 요약 파일의 `minimum_obstacle_clearance_meters`이며, 이동체만의 간격과 구분한다.

| 원본 실행 ID·요약 | 시험 상황 | Nav2 종료 상태 | 시간 | 위치 오차 | 전체 최소 간격 | 저장된 종합 판정 |
|---|---|---|---:|---:|---:|---|
| [stage_A_stage_A_baseline_20261024_202211](./stage_A_stage_A_baseline_20261024_202211/summary.json) | 추가 장애물 없는 기준 주행 | STATUS_CANCELED | 75.0000 | 0.2841 | 0.2737 | TIMEOUT_CANCELLED |
| [stage_A_stage_A_baseline_20261024_202649](./stage_A_stage_A_baseline_20261024_202649/summary.json) | 추가 장애물 없는 기준 주행 | STATUS_SUCCEEDED | 52.9400 | 0.2226 | 0.2588 | SUCCESS_RELAXED_ONLY |
| [stage_B_stage_B_multi_unmapped_20261024_203657](./stage_B_stage_B_multi_unmapped_20261024_203657/summary.json) | 지도에 없는 고정 장애물 | STATUS_ABORTED | 66.4700 | 9.4570 | 0.4334 | FAILED |
| [stage_B_stage_B_multi_unmapped_20261024_204257](./stage_B_stage_B_multi_unmapped_20261024_204257/summary.json) | 지도에 없는 고정 장애물 | STATUS_SUCCEEDED | 72.9700 | 0.2030 | 0.1599 | SUCCESS_RELAXED_ONLY |
| [stage_C_stage_C_single_moving_cross_20261024_204541](./stage_C_stage_C_single_moving_cross_20261024_204541/summary.json) | 이동 장애물 1개 횡단 | STATUS_SUCCEEDED | 54.6300 | 0.1400 | 0.1933 | SUCCESS_STRICT |
| [stage_D_stage_D_multi_moving_cross_20261024_204750](./stage_D_stage_D_multi_moving_cross_20261024_204750/summary.json) | 이동 장애물 3개 순차 횡단 | STATUS_SUCCEEDED | 59.0300 | 0.2121 | 0.1282 | SUCCESS_RELAXED_ONLY |
| [stage_E_stage_E_combined_static_moving_20261024_205021](./stage_E_stage_E_combined_static_moving_20261024_205021/summary.json) | 고정·이동 장애물 복합 | STATUS_SUCCEEDED | 59.7900 | 0.1999 | 0.1351 | SUCCESS_RELAXED_ONLY |
| [stage_E_stage_E_combined_static_moving_20261024_212333](./stage_E_stage_E_combined_static_moving_20261024_212333/summary.json) | 고정·이동 장애물 복합 | STATUS_SUCCEEDED | 62.0800 | 0.1526 | 0.1450 | SUCCESS_RELAXED_ONLY |
| [stage_F_stage_F_blocked_corridor_20261024_205307](./stage_F_stage_F_blocked_corridor_20261024_205307/summary.json) | 통로 완전 차단 | STATUS_CANCELED | 40.0200 | 5.1235 | 0.1118 | SAFE_STOP_PASSED |
| [stage_G_fixed_plus_vertical_oscillation_20261024_212851](./stage_G_fixed_plus_vertical_oscillation_20261024_212851/summary.json) | 고정 장애물·상하 왕복 | STATUS_SUCCEEDED | 60.5300 | 0.1604 | 0.1402 | SUCCESS_RELAXED_ONLY |
| [stage_H_fixed_plus_headon_oscillation_20261024_213113](./stage_H_fixed_plus_headon_oscillation_20261024_213113/summary.json) | 고정 장애물·정면 접근 왕복 | STATUS_SUCCEEDED | 57.3800 | 0.1797 | 0.0027 | FAILED |
| [stage_J_dense_unmapped_continuous_loops_20261024_213809](./stage_J_dense_unmapped_continuous_loops_20261024_213809/summary.json) | 지도에 없는 고정 7개·연속 이동 3개 | STATUS_SUCCEEDED | 56.8400 | 0.2691 | 0.0000 | FAILED |

`SUCCESS_RELAXED_ONLY`는 별도 완화 도착 공차(40cm)를 만족한 기록이며 15cm 엄격 도착 기준 통과가 아니다. `SAFE_STOP_PASSED`는 도착 성공이 아니라 차단 조건의 정지 기준 통과다. `FAILED`는 Nav2가 성공을 반환한 경우에도 독립 평가를 통과하지 못할 수 있다. 실행 ID의 A~J는 원본 코드·기록과 대조하기 위한 식별자다.

## 발표에 사용한 세 조건의 해석

- **이동 장애물 1개 횡단:** 위치 오차 14.0cm, 전체 최소 간격 19.3cm. 엄격 도착·2D 비충돌 기준을 통과한 한 번의 기록이다. 정지 후 재출발 성공을 입증한 결과는 아니다.
- **통로 완전 차단:** 전체 최소 간격 11.2cm, NAVIGATING 구간의 연속 정지 약 22.3초. 정지 기준은 통과했지만 목표에는 도착하지 않았다. 차단 해제 후 재개는 별도 시험이다.
- **지도에 없는 고정 7개·연속 이동체 3개:** Nav2 성공 반환 56.84초, Gazebo 위치 오차 26.91cm, 이동체와의 평면 형상 겹침 기록. 15cm 도착 평가와 비충돌 기준을 통과하지 못했다. 이동 생성과 안전한 회피 완성은 구분한다.

## 판정 기준과 측정 한계

- 엄격 도착: Nav2 성공, Gazebo 위치 오차 ≤15cm, 평면 형상 간격 >5mm. 5mm는 비접촉·겹침 판정의 임계값이며 충분한 안전 여유를 뜻하지 않는다.
- 차단 시 안전 정지: 전체 최소 간격 ≥10cm, 선속도 <0.015m/s 및 각속도 <0.03rad/s인 상태가 연속 ≥3초. 초기 준비·결과 후 대기·취소 후 대기를 NAVIGATING 정지 시간과 구분한다.
- 위치와 간격은 수신 시뮬레이션 시각 기반의 샘플링한 2D 형상 평가다. 타임스탬프·수신 지연·데이터 갭 한계가 있고 3D 접촉 센서 검증과 다르다.
- 복합 연속 이동 시험의 Nav2 위치 허용치는 20cm, 독립 평가 공차는 15cm다. Nav2의 map→base_link TF와 Gazebo 위치를 성공 판정 시각에 동기 기록하지 못해 27cm 오차와 성공 반환의 차이를 단일 원인으로 확정하지 않는다. AMCL 마지막 샘플만으로 원인을 설명하지 않는다.
- 시뮬레이션은 RPP·Ackermann, 로봇용 기본 설정은 DWB다. 시뮬레이션 결과를 LIMO의 장애물 안전 성능으로 일반화하지 않는다.
- 연속 이동체는 Gazebo PreUpdate에서 지정 타원 궤적으로 위치를 갱신하는 운동학적 물체다. 자동차 동역학·보행자 예측 모델이 아니다. 이동체 궤적을 Nav2에 제공하지 않았고 센서 관측에 따른 반응을 시험했다.

## 자료 보관과 재현

- `evidence_manifest.json`: 복사한 요약·메타데이터 및 로컬 원본 JSONL의 SHA-256. JSONL을 Git에 포함했다는 뜻은 아니며, 해시만으로 재계산할 수는 없다.
- 원본 시계열·영상·캡처는 각 `output/simulation/*_runs/`와 `output/video/`에 로컬 보관한다. 새로 복제한 저장소에는 없다.
- 메타데이터의 절대 경로는 당시 실행 환경의 기록이다. 해당 경로를 새 머신의 표준 경로로 가정하지 않는다.
- [실행 코드·환경 준비](../../../output/simulation/README.md), [초기 및 협로 시험 조사](../../SIMULATION_VERIFICATION_REPORT_2026-10-24.md), [현재 개발 상태](../../roadmap.md).
