"""Isolated non-motion ROS recording smoke. Requires sourced ROS and a fresh directory.

Creates only a synthetic /scan publisher. Never starts Nav2, Gazebo or drivers.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

BASE = Path(__file__).resolve().parents[2]
SCRIPT = BASE / 'fleet_control/agent/limo_robot_agent/limo_robot_agent/recording_session.py'
spec = importlib.util.spec_from_file_location('recording_session_p0', SCRIPT)
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='new test output directory')
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise SystemExit('ROS_LOCALHOST_ONLY=1 and an isolated ROS_DOMAIN_ID are required')
    root = Path(args.output)
    root.mkdir(mode=0o700)  # refuses existing evidence
    recorder = BASE / 'capstone_ws/src/limo_capstone/scripts/run_recorder.py'
    verifier = BASE / 'fleet_control/scripts/verify_bag_copy.py'
    session = m.RecordingSession(root, 'synthetic_scan', recorder, verifier)
    result = {'environment': 'isolated ROS synthetic; no motor or Gazebo'}
    try:
        result['start'] = session.start()
        for _ in range(100):
            time.sleep(.1)
            state = session.status()
            if state['status'] != 'STARTING':
                break
        result['observed'] = state
        if state['status'] != 'RECORDING':
            raise RuntimeError('recording did not start: ' + repr(state))
        time.sleep(3)
        pub = subprocess.run(['ros2', 'topic', 'pub', '--once', '/scan', 'sensor_msgs/msg/LaserScan',
                              "{header: {frame_id: 'laser'}, angle_min: 0.0, angle_max: 1.0, angle_increment: 1.0, range_min: 0.1, range_max: 10.0, ranges: [1.0, 2.0]}"],
                             capture_output=True, text=True, timeout=20)
        result['publisher_returncode'] = pub.returncode
        (root / 'publisher.log').write_text(pub.stdout + pub.stderr)
        if pub.returncode:
            raise RuntimeError('synthetic publisher failed')
        time.sleep(2)
        result['stop'] = session.stop(timeout=40)
        if result['stop']['status'] != 'VERIFIED':
            raise RuntimeError('recording not verified: ' + repr(result['stop']))
        info = subprocess.run(['ros2', 'bag', 'info', str(Path(result['stop']['run_dir']) / 'bag')], capture_output=True, text=True, timeout=30)
        (root / 'bag-info.log').write_text(info.stdout + info.stderr)
        if info.returncode or 'Topic: /scan' not in info.stdout or 'Count: 1' not in info.stdout:
            raise RuntimeError('synthetic scan receipt not confirmed')
        result['result'] = 'PASS'
        return 0
    except Exception as exc:
        result.update(result='FAIL', error=type(exc).__name__ + ': ' + str(exc))
        if session.process is not None and session.process.poll() is None and session.state['status'] in ('STARTING', 'RECORDING'):
            try:
                result['cleanup'] = session.stop(timeout=40)
            except Exception as cleanup:
                result['cleanup_error'] = type(cleanup).__name__
        return 1
    finally:
        (root / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
        session.close()

if __name__ == '__main__':
    sys.exit(main())
