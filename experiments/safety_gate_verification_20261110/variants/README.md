# variants (검증용 변형, 원본에는 적용하지 않음)

> **상태:** 모두 실험 변형이다. 기본 설정이나 기본 탐색기가 아니다. `explore_variant_ws/.../explore_relaxed`는 시뮬레이션 평가용 실험 변형이며 `m-explore-ros2`(BSD, `LICENSE_m-explore-ros2` 동봉)에서 파생되었다. `scan_normalizer_ws/.../limo_scan_normalizer`는 오프라인 검증된 해결안이며 실차 launch에는 적용하지 않았다. 변형 launch의 이름·값은 정리 이전 파일 기준이다.

| 파일 | 변경(원본 대비) |
|---|---|
| sim_diff_depth_hz20.launch.py | Differential sim launch + depth 필터 `processing_hz: 20.0` (그 하나만) |
| sim_ack_depth_minheight_m0p02.launch.py | Ackermann sim launch + depth 필터 `min_height: -0.02` (그 하나만) |
| sim_ack_depth_minheight_and_minobs.launch.py + integrated_nav2_params_depth_minobs_m0p03.yaml | 위 + costmap depth `min_obstacle_height 0.02 → -0.03` |
| sim_ack_smac_dubin.launch.py + integrated_nav2_params_smac_dubin.yaml | Ackermann sim, 전역 플래너만 SmacPlannerHybrid(DUBIN, 반경 0.35) |

원본과의 diff는 `diff <원본> <변형>`으로 확인. 변형 launch는 절대 경로 yaml을 참조한다.

## 빌드와 라이선스 메모
- 이 폴더에는 `COLCON_IGNORE`가 있어 저장소 루트에서 `colcon list/build`를 해도 `explore_relaxed`, `limo_scan_normalizer`가 기본 워크스페이스에 섞이지 않는다. 변형을 직접 빌드하려면 `explore_variant_ws` 또는 `scan_normalizer_ws` 안에서 colcon을 실행한다.
- `limo_scan_normalizer`의 라이선스는 프로젝트 라이선스가 정해질 때까지 `TODO: License declaration`으로 둔다(이전 `Apache-2.0` 표기는 결정된 것이 아니었다). `explore_relaxed`는 `m-explore-ros2`에서 파생되어 원본 BSD 라이선스 파일(`LICENSE_m-explore-ros2`)과 저작권 표기를 유지한다.
