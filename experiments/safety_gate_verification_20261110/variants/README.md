# variants (검증용 변형, 원본에는 적용하지 않음)

> **상태:** 모두 실험 변형이다. 기본 설정이나 기본 탐색기가 아니다. `explore_variant_ws/.../explore_relaxed`는 시뮬레이션 평가용 실험 변형이며 `m-explore-ros2`(BSD, `LICENSE_m-explore-ros2` 동봉)에서 파생되었다. `scan_normalizer_ws/.../limo_scan_normalizer`는 오프라인 검증된 해결안이며 실차 launch에는 적용하지 않았다. 변형 launch의 이름·값은 정리 이전 파일 기준이다.

| 파일 | 변경(원본 대비) |
|---|---|
| sim_diff_depth_hz20.launch.py | Differential sim launch + depth 필터 `processing_hz: 20.0` (그 하나만) |
| sim_ack_depth_minheight_m0p02.launch.py | Ackermann sim launch + depth 필터 `min_height: -0.02` (그 하나만) |
| sim_ack_depth_minheight_and_minobs.launch.py + integrated_nav2_params_depth_minobs_m0p03.yaml | 위 + costmap depth `min_obstacle_height 0.02 → -0.03` |
| sim_ack_smac_dubin.launch.py + integrated_nav2_params_smac_dubin.yaml | Ackermann sim, 전역 플래너만 SmacPlannerHybrid(DUBIN, 반경 0.35) |

원본과의 diff는 `diff <원본> <변형>`으로 확인. 변형 launch는 절대 경로 yaml을 참조한다.
