#!/usr/bin/env python3
"""
auto_mapping_experiment_manager.py
Unified Experiment Lifecycle, Mapping, and Archival Manager for LIMO Pro:
- Session Management: start, status, save-map, stop
- Filter Selection: B0, F1b, F3b, F4, F5_improved
- Depth Mode: ON/OFF
- Real-time RViz2 Network Coordination (ROS_DOMAIN_ID=0)
- Automated Map Saving: nav2 map_saver_cli (PGM/YAML) + SLAM Toolbox serialized posegraph
- Automated Rosbag2 recording and system resource metrics
- Archival Directory: /home/pcu/experiments/auto_mapping/<EXP_ID>
"""
import argparse
import datetime
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import yaml

ACTIVE_SESSION_FILE = "/home/pcu/experiments/auto_mapping/active_session.json"
BASE_EXPERIMENT_DIR = "/home/pcu/experiments/auto_mapping"


def cleanup_residual_nodes():
    my_pid = os.getpid()
    parent_pid = os.getppid()
    targets = [
        'limo_base', 'ydlidar_ros2_driver', 'component_container', 'astra_camera_container',
        'static_transform_publisher', 'robot_state_publisher', 'joint_state_publisher',
        'depth_obstacle_filter_node', 'lidar_scan_matcher_odom_node',
        'odom_tf_relay_node', 'stall_detector_node', 'complementary_filter_node',
        'ekf_node', 'async_slam_toolbox_node', 'controller_server',
        'smoother_server', 'planner_server', 'behavior_server',
        'bt_navigator', 'waypoint_follower', 'velocity_smoother',
        'lifecycle_manager', 'lib/explore_lite/explore', 'ros2 bag record'
    ]
    for tgt in targets:
        try:
            res = subprocess.run(['pgrep', '-f', tgt], capture_output=True, text=True)
            if res.returncode == 0:
                for p in res.stdout.strip().split():
                    try:
                        pid = int(p)
                        if pid not in (my_pid, parent_pid):
                            os.kill(pid, signal.SIGTERM)
                    except (ValueError, ProcessLookupError, PermissionError):
                        pass
        except Exception:
            pass
    time.sleep(1.5)


def get_system_resources():
    try:
        with open('/proc/meminfo', 'r') as f:
            lines = f.readlines()
        mem_total = 0
        mem_avail = 0
        for l in lines:
            if l.startswith('MemTotal:'):
                mem_total = int(l.split()[1])
            elif l.startswith('MemAvailable:'):
                mem_avail = int(l.split()[1])
        mem_used_mb = round((mem_total - mem_avail) / 1024.0, 1)
        load1, load5, load15 = os.getloadavg()
        return {
            'mem_used_mb': mem_used_mb,
            'mem_total_mb': round(mem_total / 1024.0, 1),
            'load_1min': load1,
            'load_5min': load5,
        }
    except Exception as e:
        return {'error': str(e)}


def cmd_start(args):
    os.makedirs(BASE_EXPERIMENT_DIR, exist_ok=True)
    if os.path.exists(ACTIVE_SESSION_FILE):
        with open(ACTIVE_SESSION_FILE, 'r') as f:
            sess = json.load(f)
        print(f"[ERROR] An active session is already running: {sess.get('experiment_id')}")
        print("Please stop it first using: auto_mapping_experiment_manager.py stop")
        sys.exit(1)

    est = args.estimator.lower()
    depth_bool = (str(args.depth).lower() == 'true')
    allow_motor_bool = (str(args.allow_motor).lower() == 'true')
    explore_bool = (str(args.start_explore).lower() == 'true')

    stall_bool = (str(getattr(args, 'enable_stall_detector', 'false')).lower() == 'true')

    now_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    depth_tag = "DEPTH_ON" if depth_bool else "DEPTH_OFF"
    exp_id = f"EXP_{now_str}_{est.upper()}_{depth_tag}_MAPPING"
    exp_dir = os.path.join(BASE_EXPERIMENT_DIR, exp_id)
    os.makedirs(exp_dir, exist_ok=True)
    os.makedirs(os.path.join(exp_dir, "map"), exist_ok=True)
    os.makedirs(os.path.join(exp_dir, "config_backup"), exist_ok=True)

    print("==================================================================")
    print(f" [STARTING AUTO-MAPPING SESSION] {exp_id}")
    print(f" Estimator      : {est}")
    print(f" Depth Camera   : {'ON' if depth_bool else 'OFF'}")
    print(f" Motor Drive    : {'ENABLED (LIVE)' if allow_motor_bool else 'LOCKED (SAFETY ACTIVE)'}")
    print(f" Stall Detector : {'ENABLED' if stall_bool else 'BYPASSED (DIRECT NAV2 CONTROL)'}")
    print(f" Auto-Explore   : {'ENABLED' if explore_bool else 'DISABLED (MANUAL/IDLE)'}")
    print(f" Experiment Dir : {exp_dir}")
    print("==================================================================")

    # 1. Backup Configs
    print("[1/5] Backing up configuration files...")
    cfg_src_dir = "/home/pcu/capstone_experiment_ws/src/limo_capstone/config"
    launch_src = "/home/pcu/capstone_experiment_ws/src/limo_capstone/launch/limo_integrated_auto_mapping_real.launch.py"
    backup_dir = os.path.join(exp_dir, "config_backup")
    if os.path.exists(cfg_src_dir):
        for f in os.listdir(cfg_src_dir):
            sp = os.path.join(cfg_src_dir, f)
            if os.path.isfile(sp):
                shutil.copy2(sp, backup_dir)
    if os.path.exists(launch_src):
        shutil.copy2(launch_src, backup_dir)

    # 2. Cleanup old processes
    print("[2/5] Cleaning up any residual nodes...")
    cleanup_residual_nodes()

    # 3. Launch Process
    print("[3/5] Starting real robot auto-mapping launch tree...")
    launch_cmd = [
        'ros2', 'launch', 'limo_capstone', 'limo_integrated_auto_mapping_real.launch.py',
        f'estimator_type:={est}',
        f'use_depth_camera:={"true" if depth_bool else "false"}',
        f'allow_motor_drive:={"true" if allow_motor_bool else "false"}',
        f'start_explore:={"true" if explore_bool else "false"}',
        f'enable_stall_detector:={"true" if stall_bool else "false"}',
        'use_sim_time:=false'
    ]

    log_file = os.path.join(exp_dir, "launch.log")
    log_fp = open(log_file, 'w')
    launch_proc = subprocess.Popen(
        launch_cmd,
        stdout=log_fp,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid
    )

    print("[4/5] Waiting 14s for hardware drivers and SLAM Toolbox to initialize...")
    time.sleep(14.0)

    if launch_proc.poll() is not None:
        print(f"[FATAL] Launch process exited prematurely with code {launch_proc.returncode}!")
        log_fp.close()
        sys.exit(1)

    # 4. Start Rosbag Recording
    bag_dir = os.path.join(exp_dir, "bag")
    print(f"[5/5] Starting continuous Rosbag2 recorder to {bag_dir}...")
    topics_to_record = [
        '/map', '/map_metadata', '/odom', '/imu', '/scan',
        '/odom_nav2', '/cmd_vel_raw', '/cmd_vel_locked', '/cmd_vel',
        '/tf', '/tf_static', '/plan'
    ]
    if depth_bool:
        topics_to_record.append('/depth_obstacles/points')
    if est == 'f3b':
        topics_to_record.append('/diagnostics/slip_gate')

    bag_cmd = ['ros2', 'bag', 'record', '-o', bag_dir] + topics_to_record
    bag_proc = subprocess.Popen(
        bag_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )

    # Save active session descriptor
    session_data = {
        'experiment_id': exp_id,
        'exp_dir': exp_dir,
        'estimator_type': est,
        'depth_camera': depth_bool,
        'allow_motor_drive': allow_motor_bool,
        'start_explore': explore_bool,
        'launch_pid': launch_proc.pid,
        'bag_pid': bag_proc.pid,
        'start_time_iso': datetime.datetime.now().isoformat(),
        'start_time_epoch': time.time(),
        'map_saved': False
    }
    with open(ACTIVE_SESSION_FILE, 'w') as f:
        json.dump(session_data, f, indent=2)

    print("\n>>> AUTO-MAPPING SESSION ACTIVE & LOGGING! <<<")
    print(f"Session ID: {exp_id}")
    print("------------------------------------------------------------------")
    print("[LAPTOP RVIZ2 INSTRUCTIONS]")
    print("On your laptop terminal, run:")
    print("  source /opt/ros/humble/setup.bash")
    print("  export ROS_DOMAIN_ID=0")
    print("  rviz2 -d <repo>/capstone_ws/src/limo_capstone/rviz/limo_real_auto_mapping.rviz")
    print("------------------------------------------------------------------")
    print("[COMMANDS]")
    print("  Save Map    : python3 /home/pcu/auto_mapping_experiment_manager.py save-map")
    print("  Check Status: python3 /home/pcu/auto_mapping_experiment_manager.py status")
    print("  Stop Session: python3 /home/pcu/auto_mapping_experiment_manager.py stop")
    print("==================================================================")


def cmd_save_map(args=None):
    if not os.path.exists(ACTIVE_SESSION_FILE):
        print("[ERROR] No active auto-mapping session found!")
        sys.exit(1)

    with open(ACTIVE_SESSION_FILE, 'r') as f:
        sess = json.load(f)

    exp_dir = sess['exp_dir']
    map_dir = os.path.join(exp_dir, "map")
    os.makedirs(map_dir, exist_ok=True)
    map_base_path = os.path.join(map_dir, "map")

    print("\n=======================================================")
    print(f" SAVING MAP FOR SESSION: {sess['experiment_id']}")
    print(f" Target Directory: {map_dir}")
    print("=======================================================")

    # 1. Standard Map Saver CLI (PGM + YAML)
    print("[1/3] Querying nav2_map_server map_saver_cli...")
    cmd_map_saver = [
        'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
        '-f', map_base_path,
        '--ros-args', '-p', 'save_map_timeout:=8.0'
    ]
    res_saver = subprocess.run(cmd_map_saver, capture_output=True, text=True, timeout=12.0)
    print(res_saver.stdout)
    if res_saver.returncode != 0:
        print(f"[WARN] map_saver_cli stderr: {res_saver.stderr}")

    # 2. SLAM Toolbox Map Serialization (.posegraph and .data)
    posegraph_base = os.path.join(map_dir, "slam_toolbox_posegraph")
    print("[2/3] Calling slam_toolbox SerializePoseGraph service...")
    cmd_serialize = [
        'ros2', 'service', 'call',
        '/slam_toolbox/serialize_map',
        'slam_toolbox/srv/SerializePoseGraph',
        f"{{filename: {posegraph_base}}}"
    ]
    try:
        res_ser = subprocess.run(cmd_serialize, capture_output=True, text=True, timeout=8.0)
        print(res_ser.stdout)
    except Exception as e:
        print(f"[WARN] SerializePoseGraph service call failed: {e}")

    # 3. Verification of generated map files
    print("[3/3] Validating saved map files and metadata...")
    pgm_path = f"{map_base_path}.pgm"
    yaml_path = f"{map_base_path}.yaml"

    map_valid = False
    map_metadata = {}

    if os.path.exists(yaml_path) and os.path.exists(pgm_path):
        pgm_size = os.path.getsize(pgm_path)
        yaml_size = os.path.getsize(yaml_path)
        if pgm_size > 100 and yaml_size > 20:
            map_valid = True
            with open(yaml_path, 'r') as yf:
                yaml_data = yaml.safe_load(yf)
            map_metadata = {
                'pgm_file': pgm_path,
                'pgm_bytes': pgm_size,
                'yaml_file': yaml_path,
                'yaml_bytes': yaml_size,
                'resolution': yaml_data.get('resolution'),
                'origin': yaml_data.get('origin'),
                'negate': yaml_data.get('negate'),
                'occupied_thresh': yaml_data.get('occupied_thresh'),
                'free_thresh': yaml_data.get('free_thresh'),
                'saved_at_iso': datetime.datetime.now().isoformat()
            }
            with open(os.path.join(map_dir, "map_metadata.json"), 'w') as mf:
                json.dump(map_metadata, mf, indent=2)
            print("[SUCCESS] Map PGM and YAML saved and verified!")
            print(f"  PGM Size   : {pgm_size} bytes")
            print(f"  Resolution : {yaml_data.get('resolution')} m/cell")
            print(f"  Origin     : {yaml_data.get('origin')}")
        else:
            print(f"[ERROR] Map files exist but are suspiciously small: PGM={pgm_size}B, YAML={yaml_size}B")
    else:
        print(f"[ERROR] Map files were not generated at {map_base_path}!")

    sess['map_saved'] = map_valid
    with open(ACTIVE_SESSION_FILE, 'w') as f:
        json.dump(sess, f, indent=2)

    return map_valid, map_metadata


def cmd_stop(args=None):
    if not os.path.exists(ACTIVE_SESSION_FILE):
        print("[ERROR] No active auto-mapping session found!")
        sys.exit(1)

    with open(ACTIVE_SESSION_FILE, 'r') as f:
        sess = json.load(f)

    exp_id = sess['experiment_id']
    exp_dir = sess['exp_dir']
    launch_pid = sess.get('launch_pid')
    bag_pid = sess.get('bag_pid')
    start_t = sess.get('start_time_epoch', time.time())

    print("\n=======================================================")
    print(f" STOPPING AUTO-MAPPING SESSION: {exp_id}")
    print("=======================================================")

    # 1. Save Map if not already saved
    if not sess.get('map_saved', False):
        print("[1/4] Saving final map before teardown...")
        try:
            cmd_save_map()
        except Exception as e:
            print(f"[WARN] Map saving during stop encountered: {e}")

    # 2. Graceful stop of Rosbag
    print("[2/4] Stopping Rosbag recorder and flushing storage...")
    if bag_pid:
        try:
            os.killpg(os.getpgid(bag_pid), signal.SIGINT)
            time.sleep(3.0)
        except Exception:
            pass

    # 3. Graceful stop of Launch Tree
    print("[3/4] Stopping launch tree and robot drivers...")
    if launch_pid:
        try:
            os.killpg(os.getpgid(launch_pid), signal.SIGINT)
            time.sleep(4.0)
        except Exception:
            pass
    cleanup_residual_nodes()

    # 4. Generate Final Experiment Report
    print("[4/4] Compiling session metrics and bag information...")
    end_t = time.time()
    duration_sec = round(end_t - start_t, 2)

    bag_dir = os.path.join(exp_dir, "bag")
    bag_summary = {}
    try:
        res_bag = subprocess.run(['ros2', 'bag', 'info', bag_dir], capture_output=True, text=True, timeout=5.0)
        bag_summary = {'stdout': res_bag.stdout, 'returncode': res_bag.returncode}
    except Exception as e:
        bag_summary = {'error': str(e)}

    sys_res = get_system_resources()

    final_report = {
        'experiment_id': exp_id,
        'estimator_type': sess.get('estimator_type'),
        'depth_camera': sess.get('depth_camera'),
        'allow_motor_drive': sess.get('allow_motor_drive'),
        'start_explore': sess.get('start_explore'),
        'start_time_iso': sess.get('start_time_iso'),
        'end_time_iso': datetime.datetime.now().isoformat(),
        'duration_seconds': duration_sec,
        'map_saved': sess.get('map_saved', False),
        'system_resources': sys_res,
        'bag_info': bag_summary
    }

    report_path = os.path.join(exp_dir, "experiment_report.json")
    with open(report_path, 'w') as rf:
        json.dump(final_report, rf, indent=2)

    # Clean active session file
    if os.path.exists(ACTIVE_SESSION_FILE):
        os.remove(ACTIVE_SESSION_FILE)

    print("\n=======================================================")
    print(" [SESSION TERMINATED CLEANLY]")
    print(f" Report Saved : {report_path}")
    print(f" Map Saved    : {sess.get('map_saved')}")
    print(f" Duration     : {duration_sec} s")
    print(f" Memory Used  : {sys_res.get('mem_used_mb')} MB")
    print("=======================================================")


def cmd_status(args=None):
    if not os.path.exists(ACTIVE_SESSION_FILE):
        print("\n[STATUS] No active auto-mapping session.")
        return

    with open(ACTIVE_SESSION_FILE, 'r') as f:
        sess = json.load(f)

    elapsed = round(time.time() - sess.get('start_time_epoch', time.time()), 1)
    print("\n=======================================================")
    print(f" [ACTIVE AUTO-MAPPING SESSION] {sess['experiment_id']}")
    print(f" Estimator      : {sess.get('estimator_type')}")
    print(f" Depth Camera   : {'ON' if sess.get('depth_camera') else 'OFF'}")
    print(f" Motor Drive    : {'LIVE' if sess.get('allow_motor_drive') else 'LOCKED'}")
    print(f" Auto-Explore   : {'RUNNING' if sess.get('start_explore') else 'IDLE'}")
    print(f" Elapsed Time   : {elapsed} s")
    print(f" Experiment Dir : {sess.get('exp_dir')}")
    print("=======================================================")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="LIMO Pro Auto-Mapping Experiment Manager")
    subparsers = parser.add_subparsers(dest='command', required=True)

    # Start command
    p_start = subparsers.add_parser('start', help="Start a new auto-mapping session")
    p_start.add_argument('--estimator', default='f4', choices=['b0', 'f1b', 'f3b', 'f4', 'f5_improved'])
    p_start.add_argument('--depth', default='false', help="Enable Orbbec DaBai Depth Camera (true/false)")
    p_start.add_argument('--allow-motor', default='false', help="Safety lock override: true allows motor driving")
    p_start.add_argument('--start-explore', default='false', help="Start explore_lite frontier exploration")
    p_start.add_argument('--enable-stall-detector', default='false', help="Enable stall detector gate (default: false for direct driving)")

    # Save-map command
    subparsers.add_parser('save-map', help="Save current 2D occupancy grid and serialized posegraph")

    # Stop command
    subparsers.add_parser('stop', help="Stop the active session and archive report")

    # Status command
    subparsers.add_parser('status', help="Check active session status")

    args = parser.parse_args()

    if args.command == 'start':
        cmd_start(args)
    elif args.command == 'save-map':
        cmd_save_map(args)
    elif args.command == 'stop':
        cmd_stop(args)
    elif args.command == 'status':
        cmd_status(args)
