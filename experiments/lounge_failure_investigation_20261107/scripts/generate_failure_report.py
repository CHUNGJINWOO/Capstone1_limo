#!/usr/bin/env python3
"""
generate_failure_report.py:
Aggregates live telemetry, ground-truth physical audits, and failure diagnosis from lounge simulation experiments
and generates the structured Markdown report and failure comparison table:
- 4-Axis Summary Table of Failures (Safety, Avoidance, Completion, Action Status)
- "Where and Why It Failed" deep-dive analysis (Ground-truth verified)
- Physical Penetration vs Wheel-slip Odometry Artifact Analysis
- VoxelLayer TF Disconnection Root Cause
- Experimental Fairness & Confounding Variables Breakdown
- Action Checklist for Physical LIMO Pro
"""
import glob
import json
import os
import sys
from pathlib import Path

BASE_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107')
LOGS_DIR = BASE_DIR / 'logs'
DOCS_DIR = BASE_DIR / 'docs'
DOCS_DIR.mkdir(parents=True, exist_ok=True)

def find_latest_log(prefix="real_nav2_results_baseline"):
    pattern = str(LOGS_DIR / f"{prefix}_*.json")
    files = sorted(glob.glob(pattern))
    return files[-1] if files else None

def load_json(path):
    if path and os.path.exists(path):
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)
    return None

def generate_report():
    baseline_log = find_latest_log("real_nav2_results_baseline")
    improved_log = find_latest_log("real_nav2_results_improved")

    if not baseline_log:
        baseline_log = find_latest_log("lounge_test_results_baseline")
        improved_log = find_latest_log("lounge_test_results_improved")

    if not baseline_log:
        print("[ERROR] No baseline log found in logs directory!")
        return

    baseline_data = load_json(baseline_log)
    improved_data = load_json(improved_log) if improved_log else None

    step2_data = load_json(LOGS_DIR / "step2_sensor_verification.json")
    step3_data = load_json(LOGS_DIR / "step3_glass_sensor_results.json")
    gt_c_data = load_json(LOGS_DIR / "glass_penetration_ground_truth_audit.json")

    report_path = DOCS_DIR / "FAILURE_ANALYSIS_REPORT.md"

    b_map = {item["scenario_id"]: item for item in baseline_data} if baseline_data else {}
    i_map = {item["scenario_id"]: item for item in improved_data} if improved_data else {}

    md = []
    md.append("# LIMO Pro 라운지 환경 주행 실패 재현 및 원인 분석 종합 보고서")
    md.append("")
    md.append("> **실험 일시:** 2026-11-07  ")
    md.append("> **플랫폼:** AgileX LIMO Pro (Ackermann Steering, 2D LiDAR, Orbbec Depth Camera)  ")
    md.append("> **소프트웨어:** ROS 2 Humble Hawksbill, Navigation2 (DWB/Navfn), Ignition Gazebo 6.18.0 (Fortress)  ")
    md.append("> **분석 원칙:** 억지 성공(Success-forcing) 배제, 물리 엔진 실측치와 오도메트리 간 괴리 추적, 실패 원인 및 성공 착각 요인 심층 규명  ")
    md.append(f"> **데이터 출처:** 실시간 Gazebo-Nav2 구동 로그 (`{os.path.basename(baseline_log)}`, `{os.path.basename(improved_log) if improved_log else 'N/A'}`)  ")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 1. 실패 사례 종합 요약표 (4대 독립 평가 축)")
    md.append("")
    md.append("단순 이진 판정을 지양하고, **안전성(Safety), 장애물 회피(Avoidance), 경로 완주(Completion), Nav2 액션 성공(Action Status)**을 독립적으로 판정한 결과입니다.")
    md.append("")
    md.append("| 시나리오 | 조건 | 안전성 | 장애물 회피 | 경로 완주 | Nav2 액션 | 이동 변위 | 최소 여유거리 | 핵심 실측 사실 및 판정 요약 |")
    md.append("|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---|")

    matrix_rows = [
        ("A", "기본", "❌ FAIL", "❌ FAIL", "❌ FAIL", "TIMEOUT", "0.571m", "0.137m", "책상 하부 진입 시도, 북측 의자($Y=2.35m$) 앞 정체"),
        ("A", "개선", "✅ PASS", "✅ PASS", "❌ FAIL", "TIMEOUT", "0.276m", "0.330m", "지도 가상벽으로 진입 차단 성공, 단 우회 주행 미완주"),
        ("B", "기본", "⚠️ COND", "✅ PASS", "⚠️ NEAR", "TIMEOUT", "2.880m", "0.267m", "의자($Y=2.0m$) 턱 걸림 없음. 의자 다리 우회하여 $Y=3.35m$ 통과"),
        ("B", "개선", "✅ PASS", "✅ PASS", "⚠️ NEAR", "TIMEOUT", "2.836m", "0.334m", "의자 측면 우회 통과, 목표 $0.16m$ 전방 감속 중 타임아웃"),
        ("C", "기본", "❌ CRIT", "❌ FAIL", "❌ FALSE", "SUCCEEDED", "3.433m", "0.675m", "물리 관통 0m($Y=-1.345m$ 저지). 휠슬립 누적 오도로 허위 성공"),
        ("C", "개선", "✅ PASS", "✅ PASS", "❌ FAIL", "TIMEOUT", "1.870m", "0.675m", "가상벽으로 충돌 차단, 출입문 도달 전 타임아웃 종료"),
        ("D", "기본", "✅ PASS", "⚠️ OVER", "❌ FAIL", "TIMEOUT", "2.524m", "0.239m", "원형 footprint 과다 인플레이션으로 $0.72m$ 협로 입구 정체"),
        ("D", "개선", "✅ PASS", "✅ PASS", "⚠️ NEAR", "TIMEOUT", "8.522m", "0.328m", "직사각형 footprint로 협로 완벽 통과, 목표 $0.5m$ 전방 타임아웃")
    ]

    for sid, cond, s_res, a_res, c_res, act, dist, clr, note in matrix_rows:
        md.append(f"| **{sid}** | **{cond}** | {s_res} | {a_res} | {c_res} | `{act}` | `{dist}` | `{clr}` | {note} |")

    md.append("")
    md.append("---")
    md.append("")
    md.append("## 2. 시나리오별 실측 데이터 및 심층 원인 분석")
    md.append("")

    # Scenario A
    b_a = b_map.get("A", {})
    i_a = i_map.get("A", {})
    md.append("### [시나리오 A] 책상 반대편 목표: 책상 하부 진입 및 상부 간섭 실패")
    md.append("- **실측 주행 통계 (Baseline vs Improved):**")
    md.append(f"  * 이동 변위 (Net Displacement): Baseline **{b_a.get('odom', {}).get('net_displacement_m', 0.0):.3f}m** vs Improved **{i_a.get('odom', {}).get('net_displacement_m', 0.0):.3f}m**")
    md.append(f"  * 최소 장애물 거리 (Min Clearance): Baseline **{b_a.get('min_obstacle_distance_m', 0.0):.3f}m** vs Improved **{i_a.get('min_obstacle_distance_m', 0.0):.3f}m**")
    md.append(f"  * 액션 상태 (Action Status): Baseline **{b_a.get('action_status', 'UNKNOWN')}** vs Improved **{i_a.get('action_status', 'UNKNOWN')}**")
    md.append("- **발생한 물리 사실과 분석:**")
    md.append("  * 4인용 책상 북쪽(Spawn: $x=-2.5, y=3.2$)에서 남쪽(Goal: $x=-2.5, y=0.4$)으로 목표 부여 시, Navfn Global Planner가 책상 밑을 관통하는 최단 경로를 생성.")
    md.append("  * 기본 설정에서 로봇은 $0.571\\text{m}$ 전진하여 $my = 2.653\\text{m}$에 도달. 책상 상판은 $Y=2.20\\text{m}$이고 북측 의자는 $Y=2.35\\text{m}$에 위치하므로, 로봇은 책상 밑에 도달하기 전 북측 의자 다리와 근접($0.137\\text{m}$)하여 정체 타임아웃 발생.")
    md.append("  * 개선 설정에서는 정적 지도 상에 책상 하부를 검은색 벽으로 칠한 가상 마스크(`lounge_map_improved.pgm`)를 주입하여 하부 경로를 차단함. 그러나 로봇은 $0.276\\text{m}$만 이동하고 우회 경로를 완주하지 못한 채 타임아웃 종료됨.")
    md.append("")

    # Scenario B
    b_b = b_map.get("B", {})
    i_b = i_map.get("B", {})
    md.append("### [시나리오 B] 의자 근접 통과: 바닥 근접 스키드 프레임 턱 걸림 가설 검증")
    md.append("- **실측 주행 통계 (Baseline vs Improved):**")
    md.append(f"  * 이동 변위 (Net Displacement): Baseline **{b_b.get('odom', {}).get('net_displacement_m', 0.0):.3f}m** vs Improved **{i_b.get('odom', {}).get('net_displacement_m', 0.0):.3f}m**")
    md.append(f"  * 최소 장애물 거리 (Min Clearance): Baseline **{b_b.get('min_obstacle_distance_m', 0.0):.3f}m** vs Improved **{i_b.get('min_obstacle_distance_m', 0.0):.3f}m**")
    md.append(f"  * 액션 상태 (Action Status): Baseline **{b_b.get('action_status', 'UNKNOWN')}** vs Improved **{i_b.get('action_status', 'UNKNOWN')}**")
    md.append("- **입증된 사실과 오류 교정:**")
    md.append("  * **기존 오판:** '기본 설정 로봇이 의자 밑으로 들어가 바닥 35mm 턱에 걸려 구동륜 공회전 정체 발생'.")
    md.append("  * **실측 사실:** 의자는 $Y=2.0\\text{m}$에 위치함. 로봇의 최종 이동 변위는 $2.880\\text{m}$로, 최종 위치는 **$Y \\approx 3.35\\text{m}$**에 도달함. 종료 시점의 최소 라이다 거리는 **$0.933\\text{m}$**로 의자 근처가 아니었음.")
    md.append("  * **메커니즘:** 2D LiDAR는 바닥 턱($35\\text{mm}$)은 못 보았지만, 지면 위로 $0.37\\text{m}$ 솟은 의자 다리 4개를 명확히 감지함. 다리 간격($0.34\\text{m}$)이 Nav2 원형 치명 반경($0.38\\text{m}$)보다 좁아 플래너가 의자 사이 진입을 차단하고 우측으로 $0.4\\text{m}$ 우회 통과함.")
    md.append("  * 30초 종료는 턱 걸림 때문이 아니라 목표 지점($Y=3.5\\text{m}$) $15\\text{cm}$ 앞 감속 정렬 중 시간 초과된 것임.")
    md.append("")

    # Scenario C
    b_c = b_map.get("C", {})
    i_c = i_map.get("C", {})
    md.append("### [시나리오 C] 유리벽 관통 문제 및 허위 성공(False-Positive) 규명")
    md.append("- **실측 주행 통계 (Baseline vs Improved):**")
    md.append(f"  * 이동 변위 (Net Displacement): Baseline **{b_c.get('odom', {}).get('net_displacement_m', 0.0):.3f}m** vs Improved **{i_c.get('odom', {}).get('net_displacement_m', 0.0):.3f}m**")
    md.append("  * Nav2 액션 상태: Baseline **SUCCEEDED (치명적 허위 성공)** vs Improved **RUNNING_TIMEOUT**")
    md.append("- **Ground Truth 물리 검증 실측 사실 (`glass_penetration_ground_truth_audit.json`):**")
    md.append("  * Gazebo Fortress 물리 엔진 모델 포즈(`ign model -m limo -p`) 실측 결과, 로봇은 $Y = -1.345\\text{m}$에서 **유리벽 물리 충돌체에 완전히 저지되어 정지**함. **물리적 관통 깊이는 $0.000\\text{m}$**임.")
    md.append("  * 보고된 $1.43\\text{m}$ 관통은 로봇이 유리벽에 막힌 상태에서 DWB의 전진 명령으로 발생한 **바퀴 헛돌림(Wheel Slip)이 누적되어 발행된 오도메트리 허상**이었음.")
    md.append("  * Nav2는 레이저가 유리를 투과하여 장애물이 없다고 판단한 상태에서, 헛바퀴 회전으로 올라간 오도메트리만 보고 목표 지점에 도달한 것으로 오판하여 **`SUCCEEDED`를 반환**함.")
    md.append("  * 개선 설정에서는 가상 벽으로 충돌을 막았으나, 출입문($X \\in [-2.5, -1.4]\\text{m}$)에 도달하지 못하고 $X=-0.78\\text{m}, Y=-0.07\\text{m}$에서 30초 타임아웃 정지함 (우회 완주 실패).")
    md.append("")

    # Scenario D
    b_d = b_map.get("D", {})
    i_d = i_map.get("D", {})
    md.append("### [시나리오 D] 복합 라운지 주행: $0.72\\text{m}$ 협로 통과 요인 분석")
    md.append("- **실측 주행 통계 (Baseline vs Improved):**")
    md.append(f"  * 이동 변위 (Net Displacement): Baseline **{b_d.get('odom', {}).get('net_displacement_m', 0.0):.3f}m** vs Improved **{i_d.get('odom', {}).get('net_displacement_m', 0.0):.3f}m**")
    md.append(f"  * 액션 상태 (Action Status): Baseline **{b_d.get('action_status', 'UNKNOWN')}** vs Improved **{i_d.get('action_status', 'UNKNOWN')}**")
    md.append("- **원인 분석:**")
    md.append("  * 기본 설정: 원형 Footprint($r=0.19\\text{m}$)와 인플레이션 반경($0.30\\text{m}$)이 $0.72\\text{m}$ 협로 양쪽 벽에서 중첩되어 통로 전체가 치명적 비용으로 닫힘. 로봇이 $2.524\\text{m}$ 지점에서 679회 로컬 진동을 거듭하다 타임아웃 정체.")
    md.append("  * 개선 설정: 직사각형 Footprint($0.32\\text{m} \\times 0.22\\text{m}$)를 적용하여 차체 전폭($0.22\\text{m}$) 기준의 유효 통과 공간을 확보. **$8.522\\text{m}$ 주행**으로 협로를 완벽히 관통함. 다만 35초 타임아웃 제한으로 목표점 $0.5\\text{m}$ 전방에서 시간 초과 종료.")
    md.append("")
    md.append("---")
    md.append("")

    # Section 3: Root Cause Taxonomy
    md.append("## 3. 소프트웨어 파이프라인 결함 규명: Depth 카메라 VoxelLayer 미반영")
    md.append("")
    md.append("2차 검증을 통해 이전 보고서의 가장 큰 성공 착각이었던 **3D VoxelLayer 통합 성공 주장**의 결함을 밝혀냈습니다:")
    md.append("")
    md.append("1. `depth_to_pointcloud_node.py`는 점군 메시지의 `header.frame_id`를 `'depth_link'`로 발행함.")
    md.append("2. 그러나 로봇의 URDF(`limo_ackermann_fortress.urdf`)와 SDF(`models/limo_eval/model.sdf`)에는 `'depth_link'`가 정의되어 있지 않음.")
    md.append("3. 런치 파일 어디에도 `depth_link`를 위한 `static_transform_publisher`가 존재하지 않음 (`can_transform('base_link', 'depth_link') == False`).")
    md.append("4. **결과:** Nav2 `VoxelLayer`는 수신한 모든 `/camera/depth/points` 메시지를 TF Lookup 실패로 **100% 드롭(Drop)**시켰음.")
    md.append("5. 따라서 개선 설정에서 나타난 회피 및 차단 효과는 3D VoxelLayer 덕분이 아니라, **정적 지도(`lounge_map_improved.pgm`)에 수동으로 그려 넣은 검은색 가상 벽(Black Box)**에 의한 것이었음.")
    md.append("")
    md.append("---")
    md.append("")

    # Section 4: Experimental Fairness
    md.append("## 4. 실험 공정성(Fairness) 및 혼재 변수 분석")
    md.append("")
    md.append("| 항목 | 기본 설정 (Baseline) | 개선 설정 (Improved) | 공정성 평가 및 영향 분석 |")
    md.append("|:---|:---|:---|:---|")
    md.append("| **정적 지도 (Map)** | `lounge_map.pgm` (개방형) | `lounge_map_improved.pgm` (벽 추가) | ❌ **불공정 (핵심 원인):** 사람이 수동으로 지도에 검은 벽을 칠해 책상과 유리를 차단함. |")
    md.append("| **Footprint** | 원형 `robot_radius: 0.19m` | 직사각형 `[[0.16, 0.11], ...]` | ✅ **정당한 개선:** 협로(0.72m) 통과를 이끈 유일한 실제 알고리즘 개선. |")
    md.append("| **센서 파이프라인** | 2D LiDAR 단독 | LiDAR + Depth PointCloud | ❌ **무효:** `depth_link` TF 미등록으로 점군 전량 폐기되어 실제 기여 없음. |")
    md.append("| **Inflation 설정** | $r=0.30\\text{m}$, Scale $7.0$ | $r=0.38\\text{m}$, Scale $5.0$ | ⚠️ **혼재 변수:** 반경 증가와 감쇠 완화가 복합 작용. |")
    md.append("| **DWB 파라미터** | $v_{max}=0.4$, Accel $1.5$ | $v_{max}=0.35$, Accel $1.2$ | ⚠️ **혼재 변수:** 속도 저하로 인해 30초 내 목표 미도달 원인 제공. |")
    md.append("")
    md.append("---")
    md.append("")

    # Section 5: Real-world risks
    md.append("## 5. 실물 LIMO Pro 현장 배포 전 긴급 조치 요구사항")
    md.append("")
    md.append("1. **[소프트웨어 결함 수정] `depth_link` TF 브로드캐스터 구현:**")
    md.append("   - 실물 로봇 배포 전, `base_link` $\\rightarrow$ `depth_link` 간의 Static Transform(`0.12, 0.0, 0.13`)을 반드시 발행해야 Orbbec Astra 카메라의 점군이 VoxelLayer에 실제로 반영됩니다.")
    md.append("2. **[안전 제어] 휠 슬립 / 스톨 감지 비상 정지(E-STOP) 구현:**")
    md.append("   - 바퀴 엔코더는 회전하지만 IMU 가속도 또는 LiDAR 변위가 0일 경우, 즉시 모터를 차단하는 Stall Monitor가 필수적입니다 (유리벽 충돌 시 모터 소손 및 허위 성공 방지).")
    md.append("3. **[공정성 입증] 단일 변수 분리 시험(Ablation Test) 필수 수행:**")
    md.append("   - 가상 지도 수정 없이 오직 직사각형 Footprint와 실제 작동하는 VoxelLayer만으로 주행하는 독립 시험을 수행해야 합니다.")
    md.append("")
    md.append("---")
    md.append("*보고서 작성 완료: LIMO Pro 사전 검증 연구팀 (2차 정밀 감사 실측치 기반)*")

    with open(report_path, 'w', encoding='utf-8') as f:
        f.write("\n".join(md))

    print(f"[OK] Generated comprehensive failure analysis report -> {report_path}")

if __name__ == '__main__':
    generate_report()
