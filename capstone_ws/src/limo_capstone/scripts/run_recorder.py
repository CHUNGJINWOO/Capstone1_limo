#!/usr/bin/env python3
"""run_recorder.py - 실행(run)별 기록 디렉터리와 rosbag2 기록을 만든다.

주행 동작에는 관여하지 않는다. 기존 launch/노드를 수정하지 않고, 별도 터미널에서
실행해 다음을 한 폴더에 남긴다.

  <runs_root>/<YYYYmmdd_HHMMSS>_<label>/
      meta.json     시작 시각, git commit/수정 여부, ROS 환경, 사용자가 적은 launch 인자·메모
      params/       (--snapshot-params) 실행 중인 노드별 `ros2 param dump` 결과
      bag/          rosbag2 (분할 + 압축)
      exit.json     기록기가 끝난 시각·종료 코드·종료 사유·bag 크기

한계: exit.json은 "기록기 프로세스"의 종료 사유다. launch나 개별 노드가 죽은 이유는
ROS logger 로그(meta.json의 ros_log_dir_*)나 이후 단계에서 추가할 launch 종료
이벤트 핸들러로 확인한다.

사용 예:
  python3 run_recorder.py --label corridor_f1b --snapshot-params
  python3 run_recorder.py --label cable_test --extra /camera/color/image_raw
  python3 run_recorder.py --dry-run
"""
import argparse
import datetime
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

# 프로젝트 코드·launch에서 확인한 토픽 이름이다. 목록에 있어도 발행되지 않는 토픽은
# 기록되지 않을 뿐 오류가 아니다. Nav2 표준 costmap 토픽은 이 프로젝트 파일에서 직접
# 확인한 이름이 아니므로 실행 환경에서 `ros2 topic list`로 확인한다.
CORE_TOPICS = [
    '/scan', '/odom', '/imu', '/tf', '/tf_static',
    '/odom_nav2', '/odometry/lidar_odom', '/odometry/filtered',
    '/odometry/f5_improved',
    '/cmd_vel_raw', '/cmd_vel_locked', '/cmd_vel',
    '/map', '/plan', '/goal_pose',
    '/navigate_to_pose/_action/status', '/navigate_to_pose/_action/feedback',
    '/stall_detector/status_json', '/stall_detector/virtual_bumper_cloud',
    '/auto_mapping/depth_status', '/depth_obstacles/points',
    '/depth_obstacles/clearing_points', '/diagnostics/f4_latency',
    '/local_costmap/costmap', '/global_costmap/costmap',
]

DEFAULT_MAX_BAG_MB = 512


def now_utc():
    return datetime.datetime.now(datetime.timezone.utc)


def run_cmd(cmd, timeout=10, cwd=None):
    """명령을 실행해 (returncode, stdout)을 돌려준다. 실패해도 예외를 던지지 않는다."""
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout, cwd=cwd, check=False)
        return proc.returncode, proc.stdout
    except (OSError, subprocess.TimeoutExpired):
        return -1, ''


def find_repo_root(start):
    for parent in [start] + list(start.parents):
        if (parent / '.git').exists():
            return parent
    return None


def default_runs_root(start=None, environ=None):
    """run 디렉터리의 기본 위치를 정한다. (--root 인자는 호출부에서 이 값보다 우선한다.)

    우선순위: $LIMO_RUNS_DIR > <저장소>/local_data/raw_results/limo_runs > ~/limo_runs
    저장소는 find_repo_root()로 찾되, 다른 Git 저장소를 잘못 잡지 않도록
    capstone_ws/src/limo_capstone/ 가 있을 때만 Capstone1 저장소로 인정한다.
    신규 clone 직후라 local_data/ 가 아직 없어도 경로를 선택하며,
    실제 디렉터리 생성은 기록 시작 시점(mkdir(parents=True))에만 수행한다.
    """
    env = os.environ if environ is None else environ
    explicit = env.get('LIMO_RUNS_DIR')
    if explicit:
        return explicit
    begin = Path(__file__).resolve().parent if start is None else Path(start)
    repo = find_repo_root(begin)
    if repo is not None and (repo / 'capstone_ws' / 'src' / 'limo_capstone').is_dir():
        return str(repo / 'local_data' / 'raw_results' / 'limo_runs')
    return str(Path.home() / 'limo_runs')


def git_info(repo):
    if repo is None:
        return {'available': False}
    rc, rev = run_cmd(['git', 'rev-parse', 'HEAD'], cwd=repo)
    if rc != 0:
        return {'available': False}
    _, status = run_cmd(['git', 'status', '--porcelain'], cwd=repo)
    return {
        'available': True,
        'commit': rev.strip(),
        'dirty': bool(status.strip()),
        'changed_files': len([ln for ln in status.splitlines() if ln.strip()]),
    }


def ros_log_base():
    explicit = os.environ.get('ROS_LOG_DIR')
    if explicit:
        return Path(explicit)
    ros_home = os.environ.get('ROS_HOME')
    base = Path(ros_home) if ros_home else Path.home() / '.ros'
    return base / 'log'


def newest_ros_log_dir():
    base = ros_log_base()
    if not base.is_dir():
        return None
    dirs = [p for p in base.iterdir() if p.is_dir() and not p.is_symlink()]
    if not dirs:
        return None
    return max(dirs, key=lambda p: p.stat().st_mtime).name


def dir_size_bytes(path):
    total = 0
    for root, _, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n',
                    encoding='utf-8')


def snapshot_params(run_dir):
    """실행 중인 노드의 파라미터를 params/ 아래에 저장한다. 저장한 노드 수를 돌려준다."""
    rc, out = run_cmd(['ros2', 'node', 'list'], timeout=15)
    if rc != 0:
        print('[run_recorder] ros2 node list 실패: 파라미터 스냅샷을 건너뜀')
        return 0
    params_dir = run_dir / 'params'
    params_dir.mkdir(exist_ok=True)
    saved = 0
    for node in [ln.strip() for ln in out.splitlines() if ln.strip()]:
        rc, text = run_cmd(['ros2', 'param', 'dump', node], timeout=20)
        if rc == 0 and text.strip():
            fname = node.strip('/').replace('/', '__') + '.yaml'
            (params_dir / fname).write_text(text, encoding='utf-8')
            saved += 1
    return saved


def build_bag_cmd(args, run_dir, topics):
    cmd = ['ros2', 'bag', 'record', '-o', str(run_dir / 'bag'),
           '--max-bag-size', str(int(args.max_bag_mb * 1_000_000))]
    if not args.no_compression:
        cmd += ['--compression-mode', 'file', '--compression-format', 'zstd']
    return cmd + topics


def sanitize_label(label):
    return re.sub(r'[^A-Za-z0-9_-]', '_', label)[:40]


def parse_args():
    ap = argparse.ArgumentParser(
        description='run 디렉터리 생성 + rosbag2 기록 (주행 동작 변경 없음)')
    ap.add_argument('--root', default=default_runs_root(),
        help='run 디렉터리를 만들 위치 (기본: $LIMO_RUNS_DIR, 없으면 저장소의 '
             'local_data/raw_results/limo_runs, 저장소를 못 찾으면 ~/limo_runs)')
    ap.add_argument('--label', default='run', help='run 이름에 붙일 짧은 설명')
    ap.add_argument('--note', default='', help='메모 (meta.json에 저장)')
    ap.add_argument('--launch-args', default='',
                    help='이번 실행에 쓴 launch 인자를 그대로 적어 둠 (meta.json에 저장)')
    ap.add_argument('--extra', action='append', default=[], metavar='TOPIC',
                    help='기록할 토픽 추가 (반복 가능). 이미지·깊이 영상은 용량이 커서 '
                         '기본에 없으니 필요할 때 직접 지정')
    ap.add_argument('--snapshot-params', action='store_true',
                    help='시작 시점에 노드별 파라미터를 params/에 저장')
    ap.add_argument('--max-bag-mb', type=float, default=DEFAULT_MAX_BAG_MB,
                    help='bag 파일 분할 크기(MB), 기본 %(default)s')
    ap.add_argument('--no-compression', action='store_true', help='압축 끄기')
    ap.add_argument('--dry-run', action='store_true',
                    help='아무것도 만들거나 실행하지 않고 계획만 출력')
    return ap.parse_args()


def main():
    args = parse_args()
    topics = list(dict.fromkeys(CORE_TOPICS + args.extra))
    started = now_utc()
    run_name = '{}_{}'.format(started.astimezone().strftime('%Y%m%d_%H%M%S'),
                              sanitize_label(args.label))
    run_dir = Path(args.root).expanduser() / run_name
    bag_cmd = build_bag_cmd(args, run_dir, topics)

    if args.dry_run:
        print('[dry-run] run 디렉터리:', run_dir)
        print('[dry-run] 기록 토픽 {}개'.format(len(topics)))
        print('[dry-run] 명령:', ' '.join(bag_cmd))
        return 0

    if subprocess.run(['which', bag_cmd[0]], capture_output=True,
                      check=False).returncode != 0:
        print('[run_recorder] ros2 명령을 찾을 수 없음: '
              'source /opt/ros/humble/setup.bash 를 먼저 실행하세요', file=sys.stderr)
        return 2

    run_dir.mkdir(parents=True, exist_ok=False)
    meta = {
        'run_name': run_name,
        'started_utc': started.isoformat(),
        'label': args.label,
        'note': args.note,
        'launch_args': args.launch_args,
        'git': git_info(find_repo_root(Path(__file__).resolve().parent)),
        'ros': {key: os.environ.get(key) for key in
                ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION')},
        'ros_log_dir_at_start': newest_ros_log_dir(),
        'topics': topics,
        'bag_command': bag_cmd,
    }
    if args.snapshot_params:
        meta['params_saved_nodes'] = snapshot_params(run_dir)
    write_json(run_dir / 'meta.json', meta)
    print('[run_recorder] run 디렉터리:', run_dir)
    print('[run_recorder] 기록 시작 (종료: Ctrl+C)')

    proc = subprocess.Popen(bag_cmd)
    state = {'signal': None}

    def forward(signum, _frame):
        # rosbag2는 SIGINT를 받으면 파일을 정상적으로 닫는다. 바로 죽이지 않는다.
        state['signal'] = signal.Signals(signum).name
        try:
            proc.send_signal(signal.SIGINT)
        except OSError:
            pass

    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)

    t0 = time.monotonic()
    returncode = proc.wait()
    duration = time.monotonic() - t0

    if state['signal']:
        reason = 'operator_signal_' + state['signal']
    elif returncode == 0:
        reason = 'recorder_exited_normally'
    else:
        reason = 'recorder_failed'
    ended = now_utc()
    write_json(run_dir / 'exit.json', {
        'ended_utc': ended.isoformat(),
        'duration_sec': round(duration, 1),
        'recorder_returncode': returncode,
        'reason': reason,
        'bag_size_bytes': dir_size_bytes(run_dir / 'bag'),
        'ros_log_dir_at_end': newest_ros_log_dir(),
    })
    print('[run_recorder] 종료:', reason, '(returncode={})'.format(returncode))
    # SIGINT로 정상 종료한 rosbag2의 반환 코드는 환경에 따라 다를 수 있어 0으로 취급한다.
    return 0 if state['signal'] else returncode


if __name__ == '__main__':
    sys.exit(main())
