#!/usr/bin/env python3
"""
sim_ctl.py:
Simple CLI and Interactive Menu controller for the LIMO Complex Indoor GUI Simulation.
Usage:
  python3 output/simulation/sim_ctl.py                 # Opens interactive selection menu
  python3 output/simulation/sim_ctl.py status          # Shows current GUI window & simulation status
  python3 output/simulation/sim_ctl.py start           # Starts navigation (dispatches goal from standby)
  python3 output/simulation/sim_ctl.py pause           # Pauses simulation physics
  python3 output/simulation/sim_ctl.py resume          # Resumes simulation physics
  python3 output/simulation/sim_ctl.py reset           # Resets robot & obstacles to initial start S
  python3 output/simulation/sim_ctl.py stop            # Cleanly terminates tracked session processes
  python3 output/simulation/sim_ctl.py select <A..F>   # Launches Stage A, B, C, D, E, or F in GUI standby mode
"""

import os
import sys
import time
import json
import subprocess
import pathlib

BASE_DIR = pathlib.Path(__file__).resolve().parents[2]
SIM_DIR = BASE_DIR / 'output/simulation'
CMD_FILE = SIM_DIR / 'sim_command.json'
STATUS_FILE = SIM_DIR / 'sim_status.json'
PID_TRACK_FILE = SIM_DIR / 'active_session_pids.json'
MANAGER_SCRIPT = SIM_DIR / 'interactive_sim_manager.py'

STAGE_DESCRIPTIONS = {
    'A': 'Stage A: 추가 장애물 없는 기본 목표점 주행 (Baseline)',
    'B': 'Stage B: 여러 미등록 고정 장애물 발견과 단계적 경로 재계획',
    'C': 'Stage C: 이동 장애물 1개가 통로를 완전히 횡단한 뒤 이탈',
    'D': 'Stage D: 서로 다른 시점에 이동 장애물 3개가 순차 횡단',
    'E': 'Stage E: 고정 장애물과 이동 장애물이 함께 있는 복합 환경',
    'F': 'Stage F: 장애물이 통로에 남는 완전 차단 및 안전 정지 상황',
}


def send_command(cmd_name: str, extra: dict = None):
    payload = {'command': cmd_name, 'timestamp': time.time()}
    if extra:
        payload.update(extra)
    CMD_FILE.write_text(json.dumps(payload, indent=2))
    print(f"[sim_ctl] 명령 전송 완료: '{cmd_name}'")


def print_status():
    if not STATUS_FILE.exists():
        print("[sim_ctl] 현재 실행 중인 시뮬레이션 상태 파일이 없습니다.")
        return
    try:
        st = json.loads(STATUS_FILE.read_text())
    except Exception as e:
        print(f"[sim_ctl] 상태 파일 읽기 오류: {e}")
        return

    win = st.get('windows', {})
    print("=" * 78)
    print(f" [LIMO 복잡 실내 시뮬레이션 상태]  {st.get('stage_title')}")
    print("=" * 78)
    print(f" - 현재 단계(Stage) : {st.get('stage_id')} ({st.get('stage_name')})")
    print(f" - 진행 상태(Phase) : {st.get('phase')}  (일시정지 여부: {st.get('is_paused')})")
    print(f" - 시뮬레이션 시각  : {st.get('sim_time')} s")
    print(f" - 시작점 S -> G    : S={st.get('start_pose')} -> G={st.get('goal_pose')}")
    print(f" - 로봇 현재 위치   : GT={st.get('robot_truth_pose')} | AMCL={st.get('robot_amcl_pose')}")
    print(f" - 경로 경유점 수   : 최초 경로(파란 점선)={st.get('initial_plan_waypoints')}개 | 현재 경로(주황 실선)={st.get('current_plan_waypoints')}개")
    print(f" - 최소 안전 간격   : 전체={st.get('min_all_clearance_m')}m | 벽체={st.get('min_wall_clearance_m')}m | 장애물={st.get('min_moving_clearance_m')}m")
    print(f" - 데스크톱 창 표시 : Gazebo GUI={win.get('gazebo_gui_visible')} (ID: {win.get('gazebo_window_id')}, {win.get('gazebo_geometry')})")
    print(f"                      RViz2 GUI ={win.get('rviz_gui_visible')} (ID: {win.get('rviz_window_id')}, {win.get('rviz_geometry')})")
    print(f" - 결과 저장 폴더   : {st.get('run_dir')}")
    if st.get('final_summary'):
        print(f" - 최종 평가 결과   : {st.get('final_summary')}")
    print("=" * 78)


def get_ros2_env() -> dict:
    env = os.environ.copy()
    if 'limo_capstone' not in env.get('AMENT_PREFIX_PATH', ''):
        ws_setup = str(BASE_DIR / 'capstone_ws/install/setup.bash')
        cmd = f'source /opt/ros/humble/setup.bash && source "{ws_setup}" && env -0'
        raw_env = subprocess.check_output(['/bin/bash', '-c', cmd])
        for entry in raw_env.split(b'\0'):
            if b'=' in entry:
                k, v = entry.split(b'=', 1)
                env[k.decode('utf-8', 'ignore')] = v.decode('utf-8', 'ignore')
    return env


def launch_stage_background(stage_id: str):
    stage_key = stage_id.upper()
    if stage_key not in STAGE_DESCRIPTIONS:
        print(f"[sim_ctl] 잘못된 단계입니다: {stage_id} (A~F 중 선택)", flush=True)
        return
    if STATUS_FILE.exists():
        try:
            st = json.loads(STATUS_FILE.read_text())
            if time.time() - st.get('updated_at_epoch', 0.0) < 5.0:
                print("[sim_ctl] 실행 중인 기존 시뮬레이션 세션을 먼저 정리합니다...", flush=True)
                send_command('stop')
                time.sleep(2.0)
        except Exception:
            pass
    for f in (PID_TRACK_FILE, CMD_FILE, STATUS_FILE):
        if f.exists():
            try:
                f.unlink()
            except Exception:
                pass
    print(f"[sim_ctl] {STAGE_DESCRIPTIONS[stage_key]} 기동을 시작합니다...", flush=True)
    ros_env = get_ros2_env()
    p = subprocess.Popen(
        [sys.executable, '-u', str(MANAGER_SCRIPT), '--stage', stage_key],
        env=ros_env
    )
    reported_ready = False
    reported_finished = False
    try:
        while p.poll() is None:
            time.sleep(1.0)
            if STATUS_FILE.exists():
                try:
                    st = json.loads(STATUS_FILE.read_text())
                    if not reported_ready and st.get('stage_id') == stage_key and st.get('phase') == 'WAITING_FOR_USER_START':
                        print_status()
                        print("\n[준비 완료] Gazebo GUI와 RViz2 창이 화면에 표시되었으며 목표 전송 전 대기(WAITING_FOR_USER_START) 상태입니다.", flush=True)
                        print(" -> 주행을 시작하려면: python3 output/simulation/sim_ctl.py start", flush=True)
                        reported_ready = True
                    elif not reported_finished and st.get('stage_id') == stage_key and st.get('phase') == 'FINISHED_WAITING_USER_CHOICE':
                        print_status()
                        print("\n[주행 종료] 현재 단계 시험이 완료되어 대기(FINISHED_WAITING_USER_CHOICE) 중입니다.", flush=True)
                        reported_finished = True
                except Exception:
                    pass
        print(f"[sim_ctl] 시뮬레이션 매니저 프로세스 종료됨 (returncode={p.returncode})", flush=True)
    except KeyboardInterrupt:
        send_command('stop')
        p.wait(timeout=5.0)


def interactive_menu():
    while True:
        print("\n" + "=" * 68)
        print("   LIMO 복잡 실내 자율주행 시뮬레이션 제어 메뉴 (Gazebo + RViz GUI)")
        print("=" * 68)
        print("  [1] start      : 목표점 전송 및 자율주행 시작 (대기 상태에서 출발)")
        print("  [2] pause      : 시뮬레이션 일시정지")
        print("  [3] resume     : 시뮬레이션 재개")
        print("  [4] reset      : 현재 단계 초기화 (시작점 S 복귀 및 대기)")
        print("  [5] status     : 현재 상태 및 창 표시 확인")
        print("  [A~F] 단계 선택: A(기본), B(다중 미등록), C(단일 이동), D(다중 이동), E(복합), F(차단)")
        print("  [0] stop       : 시뮬레이션 종료 (현재 세션 프로세스만 안전 정리)")
        print("  [q] 메뉴 나가기 (시뮬레이션 창 유지)")
        print("-" * 68)
        choice = input("명령 번호 또는 알파벳을 입력하세요: ").strip().lower()
        if choice in ('1', 'start'):
            send_command('start')
            time.sleep(1.0)
            print_status()
        elif choice in ('2', 'pause'):
            send_command('pause')
        elif choice in ('3', 'resume'):
            send_command('resume')
        elif choice in ('4', 'reset'):
            send_command('reset')
        elif choice in ('5', 'status'):
            print_status()
        elif choice.upper() in STAGE_DESCRIPTIONS:
            launch_stage_background(choice.upper())
        elif choice in ('0', 'stop'):
            send_command('stop')
            break
        elif choice in ('q', 'quit', 'exit'):
            break
        else:
            print("알 수 없는 입력입니다.")


def main():
    os.chmod(__file__, 0o755)
    if len(sys.argv) < 2:
        interactive_menu()
        return

    cmd = sys.argv[1].lower()
    if cmd == 'status':
        print_status()
    elif cmd in ('start', 'pause', 'resume', 'reset', 'stop'):
        send_command(cmd)
        if cmd != 'stop':
            time.sleep(0.8)
            print_status()
    elif cmd == 'select' and len(sys.argv) >= 3:
        launch_stage_background(sys.argv[2].upper())
    elif cmd.upper() in STAGE_DESCRIPTIONS:
        launch_stage_background(cmd.upper())
    else:
        print(__doc__)


if __name__ == '__main__':
    main()
