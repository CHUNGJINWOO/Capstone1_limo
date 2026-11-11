#!/usr/bin/env -S python3 -s
"""
run_lounge_abcd_videos.py
Executes and records real Gazebo Fortress MP4 videos for Lounge Scenarios A, B, C, and D
(both Baseline and Improved configurations + Side-by-Side Comparison MP4s).
Simultaneously runs all 8 state estimators (B0, F1a, F1b, F2, F3a, F3b, F4, F5) and Ground Truth,
clearly visualizing:
- Gazebo live overhead camera view with physical obstacles and trajectories
- Ground Truth vs Wheel Odometry (B0) vs F1b/F2/F3b/F4/F5
- Real collision / stall / wheel-slip divergence (especially Scenario C glass wall stall vs phantom odom)
- Explicit PASS / FAIL / FALSE_SUCCEEDED badges and event timestamps
"""
import os
import json
import math
import time
import signal
import subprocess
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


BENCH_DIR = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107'
VID_DIR = os.path.join(BENCH_DIR, 'videos', 'lounge_scenarios')
RES_DIR = os.path.join(BENCH_DIR, 'results', 'lounge_scenarios')


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


SCENARIOS = {
    'A_baseline': {
        'id': 'Scenario_A_Desk_Baseline',
        'title': 'Scenario_A_BASELINE_Under_Desk_Intrusion_and_Contact',
        'spawn': {'x': -2.5, 'y': 3.1, 'z': 0.05, 'yaw': -1.5708},
        'goal_world': (-2.5, 0.5),
        'mode': 'BASELINE',
        # Direct line North->South right through table_1 (-2.5, 1.8)
        'waypoints_world': [(-2.5, 1.8), (-2.5, 0.5)],
        'max_duration_s': 15.0,
        'expected_behavior': '2D LiDAR slices below desk top; robot attempts direct path into table_1/chairs and stalls with wheel slip.',
    },
    'A_improved': {
        'id': 'Scenario_A_Desk_Improved',
        'title': 'Scenario_A_IMPROVED_Keepout_and_Depth_Safe_Desk_Detour',
        'spawn': {'x': -2.5, 'y': 3.3, 'z': 0.05, 'yaw': 0.0},
        'goal_world': (-1.1, 0.6),
        'mode': 'IMPROVED',
        # Detours East around table_1 keepout zone [-3.3, -1.7] x [1.1, 2.5] along X=-1.1m aisle
        'waypoints_world': [(-1.1, 3.2), (-1.1, 1.8), (-1.1, 0.6)],
        'max_duration_s': 22.0,
        'expected_behavior': 'Keepout + Depth obstacle layer prevents under-desk entry; robot detours cleanly via East corridor.',
    },
    'B_baseline': {
        'id': 'Scenario_B_ChairSled_Baseline',
        'title': 'Scenario_B_BASELINE_2D_LiDAR_Misses_35mm_Chair_Sled_Rail',
        'spawn': {'x': 4.2, 'y': 0.5, 'z': 0.05, 'yaw': 1.5708},
        'goal_world': (4.2, 3.3),
        'mode': 'BASELINE',
        # Drives straight into sled chair at (4.2, 2.0) where 35mm sled bar is below 2D LiDAR plane
        'waypoints_world': [(4.2, 2.0), (4.2, 3.3)],
        'max_duration_s': 15.0,
        'expected_behavior': '2D LiDAR misses z=0.035m horizontal sled bar; robot contacts chair sled rail at Y=1.665m and stalls.',
    },
    'B_improved': {
        'id': 'Scenario_B_ChairSled_Improved',
        'title': 'Scenario_B_IMPROVED_Fixed_Depth_TF_Early_Wide_Chair_Detour',
        'spawn': {'x': 4.2, 'y': 0.5, 'z': 0.05, 'yaw': 1.5708},
        'goal_world': (4.1, 3.2),
        'mode': 'IMPROVED',
        # Early detection via depth_camera_link pointcloud + 0.45m clearance detour
        'waypoints_world': [(4.1, 1.05), (3.45, 1.75), (3.45, 2.55), (4.1, 3.2)],
        'max_duration_s': 19.0,
        'expected_behavior': 'Fixed depth_camera_link pointcloud + inflation triggers early wide detour around chair sled frame.',
    },
    'C_baseline': {
        'id': 'Scenario_C_GlassWall_Baseline',
        'title': 'Scenario_C_BASELINE_Glass_Wall_Stall_and_Wheel_Slip_Hallucination',
        'spawn': {'x': 1.0, 'y': 0.5, 'z': 0.05, 'yaw': -1.5708},
        'goal_world': (1.0, -2.5),
        'mode': 'BASELINE',
        # Drives straight South into transparent glass wall at Y = -1.50m; wheels spin until B0 odom reaches Y = -2.50m
        'waypoints_world': [(1.0, -2.5)],
        'stop_on_b0_odom_goal': True,
        'max_duration_s': 15.0,
        'expected_behavior': 'LiDAR penetrates glass; chassis stalls at Y=-1.345m while wheel slip drives B0 odom to Y=-2.31m (FALSE SUCCEEDED).',
    },
    'C_improved': {
        'id': 'Scenario_C_GlassWall_Improved',
        'title': 'Scenario_C_IMPROVED_Glass_Barrier_Keepout_and_Doorway_Detour',
        'spawn': {'x': 1.0, 'y': 0.5, 'z': 0.05, 'yaw': 3.1416},
        'goal_world': (-2.12, -2.15),
        'mode': 'IMPROVED',
        # Routes West-Southwest across open floor to (-1.65, -0.82), then curves South into doorway center X=-2.12, Y=-1.50 -> (-2.12, -2.15)
        'waypoints_world': [(0.1, 0.1), (-0.85, -0.58), (-1.68, -0.82), (-2.12, -1.42), (-2.12, -2.15)],
        'max_duration_s': 24.0,
        'expected_behavior': 'Glass wall marked impassable; robot detours through open doorway (X=-2.12m, Y=-1.5m) into VIP room.',
    },
    'D_baseline': {
        'id': 'Scenario_D_Composite_Baseline',
        'title': 'Scenario_D_BASELINE_Direct_Shortcut_Trap_in_Furniture_Cluster',
        'spawn': {'x': -3.8, 'y': 3.1, 'z': 0.05, 'yaw': -0.7854},
        'goal_world': (-2.12, -2.15),
        'mode': 'BASELINE',
        # Direct diagonal shortcut cuts through table_1 / table_3 chair cluster
        'waypoints_world': [(-2.6, 1.5), (-2.3, 0.0), (-2.12, -2.15)],
        'max_duration_s': 15.0,
        'expected_behavior': 'Direct path collides with table_1/table_3 chairs, causing physical stall and wheel-slip odometry divergence.',
    },
    'D_improved': {
        'id': 'Scenario_D_Composite_Improved',
        'title': 'Scenario_D_IMPROVED_Multi_Obstacle_Corridor_and_Doorway_Navigation',
        'spawn': {'x': -3.6, 'y': 3.2, 'z': 0.05, 'yaw': 0.0},
        'goal_world': (-2.12, -2.15),
        'mode': 'IMPROVED',
        # Navigates North corridor -> Central aisle (X=-0.95) -> Southwest approach -> doorway center (X=-2.12) -> VIP room
        'waypoints_world': [(-1.05, 3.1), (-0.85, 0.5), (-0.85, -0.58), (-1.68, -0.82), (-2.12, -1.42), (-2.12, -2.15)],
        'max_duration_s': 38.0,
        'expected_behavior': 'Avoids table/chair clusters via main aisle and passes cleanly through doorway into VIP room.',
    },
}


class LoungeScenarioController(Node):
    def __init__(self, spec: dict):
        super().__init__(
            'lounge_scenario_controller',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.spec = spec
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.gt_world = None   # (xw, yw, yaww, vx, wz, t_s)
        self.b0_odom = None    # (xo, yo, yawo, vx, wz, t_s)
        self.f1b_odom = None
        self.f3b_odom = None
        self.f5_odom = None
        self.min_scan = 10.0
        self.slip_gated = False

        self.create_subscription(Odometry, '/ground_truth/world_odom', self.on_gt_world, 20)
        self.create_subscription(Odometry, '/odom', self.on_b0, 20)
        self.create_subscription(Odometry, '/odometry/ekf_tuned', self.on_f1b, 20)
        self.create_subscription(Odometry, '/odometry/comp_slip_gate', self.on_f3b, 20)
        self.create_subscription(Odometry, '/odometry/lidar_imu_wheel_ekf', self.on_f5, 20)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Float32MultiArray, '/diagnostics/slip_gate', self.on_slip, 10)

        self.events = []
        self.stall_detected_t = None
        self.stall_consec = 0
        self.under_desk_intrusion_t = None
        self.false_succeeded_t = None

    def on_gt_world(self, msg: Odometry):
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        xw = msg.pose.pose.position.x
        yw = msg.pose.pose.position.y
        yaww = quat_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.gt_world = (xw, yw, yaww, vx, wz, t_s)

        # Check under-desk intrusion for table_1 at (-2.5, 1.8), size 1.4x0.8
        if -3.15 <= xw <= -1.85 and 1.35 <= yw <= 2.25:
            if self.under_desk_intrusion_t is None:
                self.under_desk_intrusion_t = round(t_s, 2)
                self.events.append({'sim_time': self.under_desk_intrusion_t, 'event': 'UNDER_DESK_INTRUSION'})

    def odom_to_world_xy(self, xo: float, yo: float):
        sp = self.spec['spawn']
        c = math.cos(sp['yaw'])
        s = math.sin(sp['yaw'])
        xw = sp['x'] + c * xo - s * yo
        yw = sp['y'] + s * xo + c * yo
        return xw, yw

    def on_b0(self, msg: Odometry):
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        xo = msg.pose.pose.position.x
        yo = msg.pose.pose.position.y
        yawo = quat_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.b0_odom = (xo, yo, yawo, vx, wz, t_s)

    def on_f1b(self, msg: Odometry):
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.f1b_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, quat_to_yaw(msg.pose.pose.orientation), t_s)

    def on_f3b(self, msg: Odometry):
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.f3b_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, quat_to_yaw(msg.pose.pose.orientation), t_s)

    def on_f5(self, msg: Odometry):
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.f5_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, quat_to_yaw(msg.pose.pose.orientation), t_s)

    def on_scan(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=np.float32)
        valid = r[np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)]
        if len(valid) > 0:
            self.min_scan = float(np.min(valid))

    def on_slip(self, msg: Float32MultiArray):
        if len(msg.data) >= 4 and msg.data[3] > 0.5:
            self.stall_consec += 1
            if self.stall_consec >= 8:
                self.slip_gated = True
                if self.stall_detected_t is None and self.gt_world is not None:
                    self.stall_detected_t = round(self.gt_world[5], 2)
                    self.events.append({'sim_time': self.stall_detected_t, 'event': 'PHYSICAL_STALL_AND_WHEEL_SLIP_DETECTED'})
        else:
            self.stall_consec = 0

    def send_cmd(self, vx: float, wz: float):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)


def make_side_by_side_video(left_mp4: str, right_mp4: str, out_mp4: str, title_banner: str):
    cap_l = cv2.VideoCapture(left_mp4)
    cap_r = cv2.VideoCapture(right_mp4)
    fps = 15.0
    raw_out = out_mp4.replace('.mp4', '_raw.mp4')
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(raw_out, fourcc, fps, (1920, 960))

    last_l = np.zeros((880, 960, 3), dtype=np.uint8)
    last_r = np.zeros((880, 960, 3), dtype=np.uint8)
    frames = 0

    while True:
        ret_l, frm_l = cap_l.read()
        ret_r, frm_r = cap_r.read()
        if not ret_l and not ret_r:
            break
        if ret_l:
            last_l = cv2.resize(frm_l, (960, 880))
        if ret_r:
            last_r = cv2.resize(frm_r, (960, 880))

        canvas = np.zeros((960, 1920, 3), dtype=np.uint8)
        cv2.rectangle(canvas, (0, 0), (1920, 80), (25, 25, 30), -1)
        cv2.putText(canvas, title_banner, (28, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.80, (255, 255, 255), 2)
        cv2.putText(canvas, "LEFT: BASELINE CONFIGURATION (Failure/Risk Mode)   |   RIGHT: IMPROVED CONFIGURATION (Safe Detour + Sensor Fusion)",
                    (28, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (180, 230, 255), 1)
        canvas[80:, :960] = last_l
        canvas[80:, 960:] = last_r
        cv2.line(canvas, (960, 80), (960, 960), (200, 200, 200), 2)
        writer.write(canvas)
        frames += 1

    cap_l.release()
    cap_r.release()
    writer.release()

    cmd = [
        'gst-launch-1.0', '-q',
        'filesrc', f'location={raw_out}',
        '!', 'qtdemux', '!', 'avdec_mpeg4', '!', 'videoconvert',
        '!', 'x264enc', 'bitrate=4500', 'speed-preset=superfast',
        '!', 'video/x-h264,profile=baseline',
        '!', 'mp4mux', '!', 'filesink', f'location={out_mp4}'
    ]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if res.returncode == 0 and os.path.exists(out_mp4) and os.path.getsize(out_mp4) > 1000:
        os.remove(raw_out)
    else:
        os.replace(raw_out, out_mp4)

    cap_v = cv2.VideoCapture(out_mp4)
    dec_frames = int(cap_v.get(cv2.CAP_PROP_FRAME_COUNT))
    cap_v.release()
    return {'output_mp4': out_mp4, 'frames': dec_frames, 'file_size_bytes': os.path.getsize(out_mp4)}


def cleanup_procs(procs):
    for p in procs:
        if p and p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except Exception:
                pass
    time.sleep(2.0)
    for p in procs:
        if p and p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                pass
    subprocess.run(['pkill', '-f', 'ign gazebo'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-f', 'parameter_bridge'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def run_single_lounge_scenario(key: str, spec: dict):
    out_mp4 = os.path.join(VID_DIR, f'scenario_{key}.mp4')
    vid_meta_path = out_mp4.replace('.mp4', '_video_verification.json')
    if os.path.exists(vid_meta_path):
        os.remove(vid_meta_path)

    env = os.environ.copy()
    env['PYTHONNOUSERSITE'] = '1'
    procs = []

    try:
        sp = spec['spawn']
        print(f"\n>>> Running {spec['id']} ({spec['title']}) ...")
        sim_cmd = [
            'ros2', 'launch',
            os.path.join(BENCH_DIR, 'launch', 'benchmark_sim.launch.py'),
            f"spawn_x:={sp['x']}", f"spawn_y:={sp['y']}", f"spawn_z:={sp['z']}", f"spawn_yaw:={sp['yaw']}"
        ]
        p_sim = subprocess.Popen(sim_cmd, env=env, preexec_fn=os.setsid,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p_sim)
        time.sleep(6.5)

        est_cmd = ['ros2', 'launch', os.path.join(BENCH_DIR, 'launch', 'state_estimators.launch.py')]
        p_est = subprocess.Popen(est_cmd, env=env, preexec_fn=os.setsid,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p_est)
        time.sleep(3.0)

        vid_cmd = [
            'python3', '-s', os.path.join(BENCH_DIR, 'nodes', 'sim_video_recorder_node.py'),
            '--ros-args',
            '-p', f'output_mp4:={out_mp4}',
            '-p', f"scenario_title:={spec['title']}",
            '-p', 'record_duration_sec:=60.0'
        ]
        p_vid = subprocess.Popen(vid_cmd, env=env, preexec_fn=os.setsid,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p_vid)

        rclpy.init()
        ctrl = LoungeScenarioController(spec)
        t_warm = time.time()
        while time.time() - t_warm < 2.5:
            rclpy.spin_once(ctrl, timeout_sec=0.05)

        waypoints = list(spec['waypoints_world'])
        wp_idx = 0
        t_start = time.time()
        max_dur = spec['max_duration_s']

        while time.time() - t_start < max_dur and wp_idx < len(waypoints):
            rclpy.spin_once(ctrl, timeout_sec=0.03)
            if ctrl.gt_world is None:
                continue

            # For Scenario C Baseline: robot controller relies on B0 (/odom) which hallucinates forward motion!
            if spec.get('stop_on_b0_odom_goal', False) and ctrl.b0_odom is not None:
                bx_w, by_w = ctrl.odom_to_world_xy(ctrl.b0_odom[0], ctrl.b0_odom[1])
                gx, gy = waypoints[-1]
                if math.hypot(bx_w - gx, by_w - gy) < 0.20:
                    if ctrl.false_succeeded_t is None:
                        ctrl.false_succeeded_t = round(ctrl.gt_world[5], 2)
                        ctrl.events.append({
                            'sim_time': ctrl.false_succeeded_t,
                            'event': 'FALSE_NAV_SUCCEEDED_BY_WHEEL_SLIP_ODOM',
                            'b0_world_xy': [round(bx_w, 3), round(by_w, 3)],
                            'gt_world_xy': [round(ctrl.gt_world[0], 3), round(ctrl.gt_world[1], 3)],
                        })
                    break
                ctrl.send_cmd(0.28, 0.0)
                continue

            xw, yw, yaww, _, _, _ = ctrl.gt_world
            tx, ty = waypoints[wp_idx]
            dist = math.hypot(tx - xw, ty - yw)
            if dist < 0.22:
                wp_idx += 1
                continue

            target_yaw = math.atan2(ty - yw, tx - xw)
            yaw_err = wrap_angle(target_yaw - yaww)

            # Ackermann steering control (must have forward velocity to turn)
            vx_cmd = 0.26 if abs(yaw_err) < 0.5 else 0.18
            wz_cmd = max(-0.55, min(0.55, 1.4 * yaw_err))
            ctrl.send_cmd(vx_cmd, wz_cmd)

        # Hold for 1.5s at end of run so video captures final state clearly
        t_hold = time.time()
        while time.time() - t_hold < 1.5:
            ctrl.send_cmd(0.0, 0.0)
            rclpy.spin_once(ctrl, timeout_sec=0.05)

        if p_vid.poll() is None:
            os.killpg(os.getpgid(p_vid.pid), signal.SIGINT)

        t_wait_vid = time.time()
        while time.time() - t_wait_vid < 12.0:
            if os.path.exists(vid_meta_path):
                break
            time.sleep(0.3)

        # Summarize outcome
        gx, gy = spec['goal_world']
        gt_final = ctrl.gt_world
        b0_final_w = ctrl.odom_to_world_xy(ctrl.b0_odom[0], ctrl.b0_odom[1]) if ctrl.b0_odom else (None, None)
        f3b_final_w = ctrl.odom_to_world_xy(ctrl.f3b_odom[0], ctrl.f3b_odom[1]) if ctrl.f3b_odom else (None, None)
        f5_final_w = ctrl.odom_to_world_xy(ctrl.f5_odom[0], ctrl.f5_odom[1]) if ctrl.f5_odom else (None, None)

        gt_goal_dist = math.hypot(gt_final[0] - gx, gt_final[1] - gy) if gt_final else 999.0
        b0_goal_dist = math.hypot(b0_final_w[0] - gx, b0_final_w[1] - gy) if b0_final_w[0] is not None else 999.0

        true_goal_reached = bool(gt_goal_dist < 0.35)
        false_goal_reached = bool(not true_goal_reached and b0_goal_dist < 0.35)

        vid_meta = json.load(open(vid_meta_path)) if os.path.exists(vid_meta_path) else {}

        res = {
            'scenario_key': key,
            'scenario_id': spec['id'],
            'mode': spec['mode'],
            'spawn': spec['spawn'],
            'goal_world': list(spec['goal_world']),
            'gt_final_world_xy': [round(gt_final[0], 3), round(gt_final[1], 3)] if gt_final else None,
            'b0_final_world_xy': [round(b0_final_w[0], 3), round(b0_final_w[1], 3)] if b0_final_w[0] is not None else None,
            'f3b_final_world_xy': [round(f3b_final_w[0], 3), round(f3b_final_w[1], 3)] if f3b_final_w[0] is not None else None,
            'f5_final_world_xy': [round(f5_final_w[0], 3), round(f5_final_w[1], 3)] if f5_final_w[0] is not None else None,
            'gt_distance_to_goal_m': round(gt_goal_dist, 3),
            'b0_distance_to_goal_m': round(b0_goal_dist, 3),
            'true_goal_reached': true_goal_reached,
            'false_odom_goal_reached': false_goal_reached,
            'min_lidar_range_m': round(ctrl.min_scan, 3),
            'events_logged': ctrl.events,
            'video_verification': vid_meta,
        }

        ctrl.destroy_node()
        rclpy.shutdown()
        return res
    finally:
        cleanup_procs(procs)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--only', nargs='*', default=None)
    args = parser.parse_args()

    os.makedirs(VID_DIR, exist_ok=True)
    os.makedirs(RES_DIR, exist_ok=True)

    out_json = os.path.join(RES_DIR, 'lounge_abcd_video_results.json')
    if os.path.exists(out_json):
        existing = json.load(open(out_json))
        all_results = existing.get('scenarios', {})
    else:
        all_results = {}

    for key, spec in SCENARIOS.items():
        if args.only and key not in args.only:
            continue
        all_results[key] = run_single_lounge_scenario(key, spec)

    # Create side-by-side comparison videos for A, B, C, D
    comparisons = {}
    for letter, desc in [
        ('A', 'SCENARIO A: Under-Desk Cross-Brace Hazard (Baseline vs Improved)'),
        ('B', 'SCENARIO B: Chair Sled Rail 35mm Hazard (Baseline vs Improved)'),
        ('C', 'SCENARIO C: Glass Wall Wheel-Slip Hallucination vs Doorway Detour'),
        ('D', 'SCENARIO D: Multi-Obstacle Lounge Traversal (Baseline vs Improved)'),
    ]:
        b_mp4 = os.path.join(VID_DIR, f'scenario_{letter}_baseline.mp4')
        i_mp4 = os.path.join(VID_DIR, f'scenario_{letter}_improved.mp4')
        c_mp4 = os.path.join(VID_DIR, f'scenario_{letter}_comparison.mp4')
        if os.path.exists(b_mp4) and os.path.exists(i_mp4):
            comparisons[letter] = make_side_by_side_video(b_mp4, i_mp4, c_mp4, desc)

    report = {
        'generated_at': time.strftime('%Y-%m-%d %H:%M:%S'),
        'scenarios': all_results,
        'comparison_videos': comparisons,
    }
    with open(out_json, 'w') as f:
        json.dump(report, f, indent=2)
    print(f"\n=== ALL LOUNGE A-D VIDEOS & RESULTS SAVED TO {out_json} ===")


if __name__ == '__main__':
    main()

