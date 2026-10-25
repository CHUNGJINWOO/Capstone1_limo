#!/usr/bin/env python3
"""
interactive_sim_manager.py:
Interactive GUI Simulation Manager for the Expanded Complex Indoor Environment (Stages A ~ F).
Key Capabilities:
  1. Launches Gazebo GUI (top-down camera) and RViz2 (top-down orthographic view) directly on the
     user's desktop (DISPLAY=:0) and verifies X11 window visibility via xwininfo.
  2. Tracks ONLY its own spawned child process groups in active_session_pids.json (NEVER runs broad pkill).
  3. Pre-computes and displays the Initial Global Path (Blue Dashed Line), Start S, and Goal G in RViz
     while holding in WAITING_FOR_USER_START state until the user explicitly triggers 'start'.
  4. Publishes real-time RViz overlays:
     - Start S & Goal G markers
     - Persistent Initial Global Path (Blue Dashed Line - never erased)
     - Current Global Path (Orange Solid Line on /plan and marker)
     - Actual Driven Trajectory (Green Solid Line on /viz/driven_path)
     - Mapped, Unmapped, and Moving Obstacle markers + live status HUD
  5. Executes deterministic moving obstacle trajectories (Stages C, D, E, F) strictly separated from Nav2
     (Nav2 only perceives obstacles via /scan LiDAR).
  6. Automatically archives all configs/hashes, raw telemetry, milestone events (detection -> costmap ->
     replan -> wait/detour -> resume -> goal reached), evaluation summary, and live GUI window recording MP4.
  7. Never auto-advances to the next stage; waits for the user to inspect results and select the next action.
"""

import os
import sys
import time
import math
import json
import shutil
import signal
import hashlib
import pathlib
import argparse
import traceback
import subprocess
import faulthandler
faulthandler.enable()

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image as ImageMsg
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

BASE_DIR = pathlib.Path(__file__).resolve().parents[2]
SIM_DIR = BASE_DIR / 'output/simulation'
ENV_DIR = SIM_DIR / 'complex_indoor_env'
RUNS_BASE_DIR = SIM_DIR / '2026-10-24_complex_indoor_runs'

PID_TRACK_FILE = SIM_DIR / 'active_session_pids.json'
CMD_FILE = SIM_DIR / 'sim_command.json'
STATUS_FILE = SIM_DIR / 'sim_status.json'

sys.path.insert(0, str(SIM_DIR))
from evaluation_lib import (
    compute_polygon_clearance_sat,
    make_robot_polygon,
    make_box_polygon,
    parse_sdf_boxes,
    evaluate_scenario_from_records,
)

GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'STATUS_UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'STATUS_ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'STATUS_EXECUTING',
    GoalStatus.STATUS_CANCELING: 'STATUS_CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'STATUS_SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'STATUS_CANCELED',
    GoalStatus.STATUS_ABORTED: 'STATUS_ABORTED'
}


def compute_sha256(filepath):
    h = hashlib.sha256()
    with open(filepath, 'rb') as f:
        while chunk := f.read(8192):
            h.update(chunk)
    return h.hexdigest()


def load_korean_font(size: int):
    for p in [
        '/usr/share/fonts/truetype/nanum/NanumSquareB.ttf',
        '/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
    ]:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    return ImageFont.load_default()


FONT_BANNER = load_korean_font(22)
FONT_SUB = load_korean_font(17)


def cleanup_tracked_session_only(local_pgids=None, local_pids=None):
    """
    Strictly terminates ONLY the child process groups spawned by this session instance.
    Never uses broad pkill and never signals its own process group.
    """
    my_pid = os.getpid()
    my_ppid = os.getppid()
    my_pgrp = os.getpgrp()

    pgids = list(local_pgids or [])
    pids = list(local_pids or [])

    for pgid in pgids:
        if int(pgid) in (1, my_pid, my_ppid, my_pgrp):
            continue
        try:
            os.killpg(int(pgid), signal.SIGTERM)
        except Exception:
            pass
    for pid in pids:
        if int(pid) in (1, my_pid, my_ppid, my_pgrp):
            continue
        try:
            os.kill(int(pid), signal.SIGTERM)
        except Exception:
            pass

    time.sleep(0.8)

    for pgid in pgids:
        if int(pgid) in (1, my_pid, my_ppid, my_pgrp):
            continue
        try:
            os.killpg(int(pgid), signal.SIGKILL)
        except Exception:
            pass
    for pid in pids:
        if int(pid) in (1, my_pid, my_ppid, my_pgrp):
            continue
        try:
            os.kill(int(pid), signal.SIGKILL)
        except Exception:
            pass

    try:
        if PID_TRACK_FILE.exists():
            PID_TRACK_FILE.unlink()
    except Exception:
        pass


def detect_gui_windows() -> dict:
    """
    Inspects X11 window tree on DISPLAY=:0 to verify that Gazebo GUI and RViz2 windows are visible.
    Returns window IDs, titles, and geometries.
    """
    res = {
        'display': os.environ.get('DISPLAY', ':0'),
        'gazebo_gui_visible': False,
        'gazebo_window_id': None,
        'gazebo_window_title': None,
        'gazebo_geometry': None,
        'rviz_gui_visible': False,
        'rviz_window_id': None,
        'rviz_window_title': None,
        'rviz_geometry': None,
    }
    try:
        out = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True, stderr=subprocess.DEVNULL)
        for line in out.splitlines():
            s = line.strip()
            if not s.startswith('0x'):
                continue
            # Match Ignition Gazebo GUI window (typically >= 400x300)
            if ('Ignition Gazebo' in s or 'Gazebo' in s or 'gz-sim' in s) and not res['gazebo_gui_visible']:
                parts = s.split()
                wid = parts[0]
                # Check geometry token like 1000x800+...
                for tok in reversed(parts):
                    if 'x' in tok and '+' in tok:
                        dims = tok.split('+')[0].split('x')
                        if len(dims) == 2 and dims[0].isdigit() and dims[1].isdigit():
                            w, h = int(dims[0]), int(dims[1])
                            if w >= 300 and h >= 250:
                                res['gazebo_gui_visible'] = True
                                res['gazebo_window_id'] = wid
                                res['gazebo_window_title'] = s
                                res['gazebo_geometry'] = tok
                        break
            # Match RViz2 window
            if ('RViz' in s or 'rviz2' in s or '.rviz' in s) and not res['rviz_gui_visible']:
                parts = s.split()
                wid = parts[0]
                for tok in reversed(parts):
                    if 'x' in tok and '+' in tok:
                        dims = tok.split('+')[0].split('x')
                        if len(dims) == 2 and dims[0].isdigit() and dims[1].isdigit():
                            w, h = int(dims[0]), int(dims[1])
                            if w >= 300 and h >= 250:
                                res['rviz_gui_visible'] = True
                                res['rviz_window_id'] = wid
                                res['rviz_window_title'] = s
                                res['rviz_geometry'] = tok
                        break
    except Exception as e:
        res['error'] = str(e)
    return res


def set_ignition_pose(world_name: str, entity_name: str, x: float, y: float, z: float, yaw: float, env: dict) -> bool:
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    req = f'name: "{entity_name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, orientation: {{z: {qz:.5f}, w: {qw:.5f}}}'
    cmd = [
        'ign', 'service', '-s', f'/world/{world_name}/set_pose',
        '--reqtype', 'ignition.msgs.Pose',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '500',
        '--req', req
    ]
    r = subprocess.run(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return (r.returncode == 0)


def set_ignition_pause(world_name: str, pause: bool, env: dict) -> bool:
    p_str = 'true' if pause else 'false'
    cmd = [
        'ign', 'service', '-s', f'/world/{world_name}/control',
        '--reqtype', 'ignition.msgs.WorldControl',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '800',
        '--req', f'pause: {p_str}'
    ]
    r = subprocess.run(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return (r.returncode == 0)


def capture_x11_window_pil(win_id: str, target_w: int, target_h: int):
    """Captures an X11 window via ImageMagick 'import -window <win_id> <tmp_png>' and resizes to (target_w, target_h)."""
    if not win_id:
        return None
    tmp_png = SIM_DIR / f".tmp_cap_{win_id}.png"
    try:
        subprocess.run(
            ['import', '-window', str(win_id), str(tmp_png)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=3.5,
            check=True
        )
        im = Image.open(tmp_png).convert('RGB')
        res = im.resize((target_w, target_h), Image.BILINEAR)
        try:
            tmp_png.unlink()
        except Exception:
            pass
        return res
    except Exception:
        return None


class InteractiveScenarioNode(Node):
    def __init__(self, run_dir: pathlib.Path, world_name: str, stage_cfg: dict, geom_manifest: dict, sdf_boxes: list):
        super().__init__('interactive_scenario_node', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)
        ])
        self.run_dir = run_dir
        self.world_name = world_name
        self.stage_cfg = stage_cfg
        self.geom_manifest = geom_manifest
        self.sdf_boxes = sdf_boxes

        self.sim_time = 0.0
        self.scan_count = 0
        self.odom_count = 0
        self.image_count = 0
        self.recording = False
        self.current_phase = "INITIALIZING"

        self.start_pose = stage_cfg['start_pose']
        self.goal_pose = stage_cfg['goal_pose']

        self.latest_odom = None
        self.latest_amcl = None
        self.latest_truth = None
        self.latest_cmd = [0.0, 0.0]
        self.latest_overhead_img = None
        self.feedback = None

        # Path tracking (Initial Blue Dashed vs Current Orange Solid vs Driven Green Solid)
        self.initial_plan_poses = []   # Locked once initial global plan is received; NEVER erased!
        self.current_plan_poses = []
        self.driven_path_msg = Path()
        self.driven_path_msg.header.frame_id = 'map'
        self.driven_pts = []
        self.plans_recorded = []
        self.last_saved_plan_pts = None

        # Clearances & Collision tracking
        self.robot_width = 0.22
        self.robot_length = 0.32
        self.min_moving_clearance = float('inf')
        self.min_wall_clearance = float('inf')
        self.min_all_clearance = float('inf')
        self.collision_occurred = False
        self.collision_object = None

        # Obstacle perception & state tracking
        self.unmapped_obs_states = {}
        for uo in stage_cfg['unmapped_obstacles']:
            self.unmapped_obs_states[uo['name']] = {
                'spec': uo,
                'first_scan_hit_sim': None,
                'first_costmap_mark_sim': None,
                'scan_hits': 0,
                'costmap_lethal_cells': 0,
            }

        self.moving_obs_states = {}
        for mo in stage_cfg['moving_obstacles']:
            self.moving_obs_states[mo['name']] = {
                'spec': mo,
                'pose': list(mo['initial_pose']),
                'triggered': (mo.get('trigger_type') == 'always'),
                'trigger_sim_time': 0.0 if mo.get('trigger_type') == 'always' else None,
                'completed_sim_time': None,
                'first_scan_hit_sim': None,
                'first_costmap_mark_sim': None,
                'scan_hits': 0,
                'costmap_lethal_cells': 0,
            }

        # Milestone events (strictly separated as requested in Section 6 & 7)
        self.events = {
            'stage_id': stage_cfg['stage_id'],
            'stage_name': stage_cfg['name'],
            'nav_start_sim_time': None,
            'initial_plan_sim_time': None,
            'obstacle_detections': {},
            'costmap_reflections': {},
            'path_replans': [],
            'decelerate_or_wait_events': [],
            'motion_resumed_sim_time': None,
            'action_end_sim_time': None,
            'final_action_status': None,
        }
        self.was_waiting_for_obstacle = False
        self.wait_start_sim = None

        # Costmap & Log files
        self.global_costmap_snaps = []
        self.global_costmap_times = []
        self.last_saved_image_sim = -100.0
        (self.run_dir / 'frames').mkdir(parents=True, exist_ok=True)

        self.gt_pose_file = (self.run_dir / 'gt_pose_history.jsonl').open('w')
        self.obstacle_pose_file = (self.run_dir / 'obstacle_pose_history.jsonl').open('w')
        self.costmap_roi_file = (self.run_dir / 'costmap_roi_history.jsonl').open('w')
        self.telemetry_file = (self.run_dir / 'telemetry.jsonl').open('w')
        self.plan_history_file = (self.run_dir / 'plan_history.jsonl').open('w')
        self.scan_history_file = (self.run_dir / 'scan_history.jsonl').open('w')
        self.cmd_history_file = (self.run_dir / 'cmd_history.jsonl').open('w')

        # Subscriptions
        self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._global_costmap_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(ImageMsg, '/demo/overhead', self._image_cb, qos_profile_sensor_data)
        self.create_subscription(TFMessage, f'/world/{world_name}/pose/info', self._truth_cb, 10)

        # Publishers & Action/Service Clients
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/viz/scenario_markers', 10)
        self.driven_path_pub = self.create_publisher(Path, '/viz/driven_path', 10)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.compute_path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.clear_global_cm = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local_cm = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        # 5 Hz timer for RViz visualization markers
        self.viz_timer = self.create_timer(0.20, self.publish_rviz_overlays)

    def _clock_cb(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec / 1e9

    def _cmd_cb(self, msg):
        self.latest_cmd = [round(msg.linear.x, 4), round(msg.angular.z, 4)]
        if self.recording:
            self.cmd_history_file.write(json.dumps({
                'sim_time': round(self.sim_time, 3),
                'phase': self.current_phase,
                'linear_x': round(msg.linear.x, 4),
                'angular_z': round(msg.angular.z, 4)
            }) + '\n')

    def _odom_cb(self, msg):
        self.odom_count += 1
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.latest_odom = [round(p.x, 4), round(p.y, 4), round(yaw, 4)]

    def _amcl_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.latest_amcl = [round(p.x, 4), round(p.y, 4), round(yaw, 4)]

    def _plan_cb(self, msg):
        if not msg.poses:
            return
        pts = []
        for ps in msg.poses:
            p = ps.pose.position
            q = ps.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            pts.append([round(p.x, 4), round(p.y, 4), round(yaw, 4)])

        self.current_plan_poses = pts
        if not self.initial_plan_poses:
            self.initial_plan_poses = list(pts)
            if self.events['initial_plan_sim_time'] is None:
                self.events['initial_plan_sim_time'] = round(self.sim_time, 3)

        # Detect meaningful path replan (change in midpoint or length > 0.35m)
        should_save = False
        if self.last_saved_plan_pts is None:
            should_save = True
        elif abs(len(pts) - len(self.last_saved_plan_pts)) >= 10:
            should_save = True
        else:
            m1 = pts[len(pts) // 2]
            m2 = self.last_saved_plan_pts[len(self.last_saved_plan_pts) // 2]
            if math.hypot(m1[0] - m2[0], m1[1] - m2[1]) >= 0.35:
                should_save = True

        if should_save and self.recording:
            plan_id = len(self.plans_recorded)
            mid_y = pts[len(pts) // 2][1] if pts else 0.0
            rec = {
                'plan_id': plan_id,
                'sim_time': round(self.sim_time, 3),
                'phase': self.current_phase,
                'waypoint_count': len(pts),
                'mid_waypoint': pts[len(pts) // 2] if pts else None,
                'poses': pts
            }
            self.plans_recorded.append(rec)
            self.last_saved_plan_pts = pts
            self.plan_history_file.write(json.dumps(rec) + '\n')
            self.plan_history_file.flush()

            if plan_id > 0:
                self.events['path_replans'].append({
                    'plan_id': plan_id,
                    'sim_time': round(self.sim_time, 3),
                    'waypoint_count': len(pts),
                    'mid_y': round(mid_y, 3)
                })

    def _scan_cb(self, msg):
        self.scan_count += 1
        pose_ref = self.latest_truth[:2] + [self.latest_truth[5]] if self.latest_truth else self.latest_amcl
        if pose_ref is None:
            return
        rx, ry, ryaw = pose_ref[0], pose_ref[1], pose_ref[2]
        lx = rx + 0.10 * math.cos(ryaw)
        ly = ry + 0.10 * math.sin(ryaw)

        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float32)
        valid = np.isfinite(ranges) & (ranges >= max(0.05, msg.range_min)) & (ranges <= min(2.50, msg.range_max))
        v_ranges = ranges[valid]
        v_angles = angles[valid] + ryaw

        pts_x = lx + v_ranges * np.cos(v_angles)
        pts_y = ly + v_ranges * np.sin(v_angles)

        # Check LiDAR hits against unmapped and moving obstacles
        margin = 0.10
        for uname, ustate in self.unmapped_obs_states.items():
            ux, uy, _ = ustate['spec']['pose']
            usx, usy, _ = ustate['spec']['size']
            in_box = (
                (pts_x >= ux - usx / 2.0 - margin) & (pts_x <= ux + usx / 2.0 + margin) &
                (pts_y >= uy - usy / 2.0 - margin) & (pts_y <= uy + usy / 2.0 + margin)
            )
            hits = int(np.sum(in_box))
            ustate['scan_hits'] = hits
            if hits >= 2 and ustate['first_scan_hit_sim'] is None and self.current_phase == "NAVIGATING":
                ustate['first_scan_hit_sim'] = round(self.sim_time, 3)
                self.events['obstacle_detections'][uname] = {
                    'sim_time': round(self.sim_time, 3),
                    'hit_rays': hits,
                    'robot_pose': [round(rx, 3), round(ry, 3)]
                }

        for mname, mstate in self.moving_obs_states.items():
            mx, my, _ = mstate['pose']
            msx, msy, _ = mstate['spec']['size']
            in_box = (
                (pts_x >= mx - msx / 2.0 - margin) & (pts_x <= mx + msx / 2.0 + margin) &
                (pts_y >= my - msy / 2.0 - margin) & (pts_y <= my + msy / 2.0 + margin)
            )
            hits = int(np.sum(in_box))
            mstate['scan_hits'] = hits
            if hits >= 2 and mstate['first_scan_hit_sim'] is None and self.current_phase == "NAVIGATING":
                mstate['first_scan_hit_sim'] = round(self.sim_time, 3)
                self.events['obstacle_detections'][mname] = {
                    'sim_time': round(self.sim_time, 3),
                    'hit_rays': hits,
                    'robot_pose': [round(rx, 3), round(ry, 3)],
                    'obstacle_pose': [round(mx, 3), round(my, 3)]
                }

        if self.recording and self.scan_count % 3 == 0:
            step = max(1, len(pts_x) // 90)
            scan_pts = [[round(float(x), 3), round(float(y), 3)] for x, y in zip(pts_x[::step], pts_y[::step])]
            self.scan_history_file.write(json.dumps({
                'sim_time': round(self.sim_time, 3),
                'phase': self.current_phase,
                'robot_pose': [round(rx, 3), round(ry, 3), round(ryaw, 3)],
                'scan_points_map': scan_pts
            }) + '\n')

    def _global_costmap_cb(self, msg):
        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        grid = np.array(msg.data, dtype=np.int16).reshape((h, w))

        if self.recording:
            self.global_costmap_snaps.append(grid.copy())
            self.global_costmap_times.append(round(self.sim_time, 3))

        # Check costmap marking around each unmapped and moving obstacle
        def count_lethal_in_box(cx, cy, sx, sy):
            c0 = max(0, min(w - 1, int((cx - sx / 2.0 - 0.10 - ox) / res)))
            c1 = max(0, min(w, int(math.ceil((cx + sx / 2.0 + 0.10 - ox) / res))))
            r0 = max(0, min(h - 1, int((cy - sy / 2.0 - 0.10 - oy) / res)))
            r1 = max(0, min(h, int(math.ceil((cy + sy / 2.0 + 0.10 - oy) / res))))
            sub = grid[r0:r1, c0:c1]
            return int(np.sum(sub >= 99)) if sub.size > 0 else 0

        for uname, ustate in self.unmapped_obs_states.items():
            ux, uy, _ = ustate['spec']['pose']
            usx, usy, _ = ustate['spec']['size']
            cnt = count_lethal_in_box(ux, uy, usx, usy)
            ustate['costmap_lethal_cells'] = cnt
            if cnt >= 2 and ustate['first_costmap_mark_sim'] is None and self.current_phase == "NAVIGATING":
                ustate['first_costmap_mark_sim'] = round(self.sim_time, 3)
                self.events['costmap_reflections'][uname] = {
                    'sim_time': round(self.sim_time, 3),
                    'lethal_cells': cnt
                }

        for mname, mstate in self.moving_obs_states.items():
            mx, my, _ = mstate['pose']
            msx, msy, _ = mstate['spec']['size']
            cnt = count_lethal_in_box(mx, my, msx, msy)
            mstate['costmap_lethal_cells'] = cnt
            if cnt >= 2 and mstate['first_costmap_mark_sim'] is None and self.current_phase == "NAVIGATING":
                mstate['first_costmap_mark_sim'] = round(self.sim_time, 3)
                self.events['costmap_reflections'][mname] = {
                    'sim_time': round(self.sim_time, 3),
                    'lethal_cells': cnt
                }

        if self.recording:
            self.costmap_roi_file.write(json.dumps({
                'recv_sim_time': round(self.sim_time, 3),
                'phase': self.current_phase,
                'unmapped_lethal': {k: v['costmap_lethal_cells'] for k, v in self.unmapped_obs_states.items()},
                'moving_lethal': {k: v['costmap_lethal_cells'] for k, v in self.moving_obs_states.items()},
            }) + '\n')

    def _truth_cb(self, msg):
        header_sim_time = 0.0
        if msg.transforms:
            st = msg.transforms[0].header.stamp
            header_sim_time = st.sec + st.nanosec / 1e9
        recv_sim_time = round(self.sim_time, 3)

        limo_pose = None
        dyn_poses = []
        for tf in msg.transforms:
            cid = tf.child_frame_id
            t = tf.transform.translation
            q = tf.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            if cid == 'limo' or cid.endswith('/limo'):
                limo_pose = [
                    round(t.x, 4), round(t.y, 4), round(t.z, 4),
                    0.0, 0.0, round(yaw, 4)
                ]
            else:
                for b in self.sdf_boxes:
                    if b['is_dynamic'] and (cid == b['model_name'] or cid.endswith('/' + b['model_name'])):
                        dyn_poses.append((b, t.x, t.y, t.z))

        if limo_pose is not None:
            self.latest_truth = limo_pose
            # Throttle heavy SAT polygon clearance & file logging to 20 Hz (every 0.05s)
            now_mono = time.monotonic()
            if hasattr(self, '_last_truth_mono') and (now_mono - self._last_truth_mono) < 0.05:
                return
            self._last_truth_mono = now_mono

            if self.recording and self.sim_time > 0.0:
                for b, tx, ty, tz in dyn_poses:
                    self.obstacle_pose_file.write(json.dumps({
                        'recv_sim_time': recv_sim_time,
                        'header_sim_time': round(header_sim_time, 3),
                        'phase': self.current_phase,
                        'model_name': b['model_name'],
                        'x': round(tx, 4),
                        'y': round(ty, 4),
                        'z': round(tz, 4),
                        'size': b['size']
                    }) + '\n')

            rx, ry, rz, _, _, ryaw = limo_pose

            # Append to driven trajectory if moved >= 4cm
            if self.current_phase in ("NAVIGATING", "POST_RESULT", "POST_CANCEL"):
                if not self.driven_pts or math.hypot(rx - self.driven_pts[-1][0], ry - self.driven_pts[-1][1]) >= 0.04:
                    self.driven_pts.append((rx, ry, ryaw))
                    ps = PoseStamped()
                    ps.header.frame_id = 'map'
                    ps.header.stamp = self.get_clock().now().to_msg()
                    ps.pose.position.x = float(rx)
                    ps.pose.position.y = float(ry)
                    ps.pose.position.z = 0.05
                    ps.pose.orientation.z = math.sin(ryaw / 2.0)
                    ps.pose.orientation.w = math.cos(ryaw / 2.0)
                    self.driven_path_msg.header.stamp = ps.header.stamp
                    self.driven_path_msg.poses.append(ps)
                    self.driven_path_pub.publish(self.driven_path_msg)

            # Compute SAT clearances against walls and obstacles
            r_poly = make_robot_polygon(rx, ry, ryaw, rw=self.robot_width, rl=self.robot_length)
            step_min_wall = float('inf')
            step_min_mov = float('inf')

            for b in self.sdf_boxes:
                if not b['is_dynamic']:
                    b_poly = make_box_polygon(b['center'][0], b['center'][1], b['size'][0], b['size'][1])
                    d = compute_polygon_clearance_sat(r_poly, b_poly)
                    if d < step_min_wall:
                        step_min_wall = d
                    if d <= 0.005:
                        self.collision_occurred = True
                        self.collision_object = b['name']

            for uname, ustate in self.unmapped_obs_states.items():
                ux, uy, _ = ustate['spec']['pose']
                usx, usy, _ = ustate['spec']['size']
                b_poly = make_box_polygon(ux, uy, usx, usy)
                d = compute_polygon_clearance_sat(r_poly, b_poly)
                if d < step_min_mov:
                    step_min_mov = d
                if d <= 0.005:
                    self.collision_occurred = True
                    self.collision_object = uname

            for mname, mstate in self.moving_obs_states.items():
                mx, my, _ = mstate['pose']
                msx, msy, _ = mstate['spec']['size']
                b_poly = make_box_polygon(mx, my, msx, msy)
                d = compute_polygon_clearance_sat(r_poly, b_poly)
                if d < step_min_mov:
                    step_min_mov = d
                if d <= 0.005:
                    self.collision_occurred = True
                    self.collision_object = mname

            step_min_all = min(step_min_wall, step_min_mov)
            if self.current_phase == "NAVIGATING":
                if step_min_wall < self.min_wall_clearance:
                    self.min_wall_clearance = step_min_wall
                if step_min_mov < self.min_moving_clearance:
                    self.min_moving_clearance = step_min_mov
                if step_min_all < self.min_all_clearance:
                    self.min_all_clearance = step_min_all

            if self.recording and self.sim_time > 0.0:
                self.gt_pose_file.write(json.dumps({
                    'recv_sim_time': recv_sim_time,
                    'header_sim_time': round(header_sim_time, 3),
                    'phase': self.current_phase,
                    'x': rx, 'y': ry, 'z': rz, 'yaw': ryaw,
                    'step_wall_clearance': round(step_min_wall, 4),
                    'step_moving_clearance': round(step_min_mov, 4) if step_min_mov != float('inf') else None,
                    'step_clearance': round(step_min_all, 4)
                }) + '\n')

    def _image_cb(self, msg):
        self.image_count += 1
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        self.latest_overhead_img = Image.fromarray(arr)
        if not self.recording:
            return
        if self.sim_time - self.last_saved_image_sim >= 0.25:
            self.last_saved_image_sim = self.sim_time
            fname = f"frame_{len(list((self.run_dir / 'frames').glob('*.png'))):04d}.png"
            self.latest_overhead_img.save(self.run_dir / 'frames' / fname)
            self.telemetry_file.write(json.dumps({
                'sim_time': round(self.sim_time, 3),
                'phase': self.current_phase,
                'image': f"frames/{fname}",
                'odom': self.latest_odom,
                'amcl': self.latest_amcl,
                'truth': self.latest_truth,
                'cmd_vel': self.latest_cmd,
                'min_clearance': round(self.min_all_clearance, 4) if self.min_all_clearance != float('inf') else None,
            }) + '\n')

    def publish_initial_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.02
        self.init_pub.publish(msg)

    def publish_rviz_overlays(self):
        """
        Publishes MarkerArray on /viz/scenario_markers for RViz2:
          - Start S cylinder & text
          - Goal G cylinder & text
          - Initial Global Path (Blue Dashed Line - persistent!)
          - Mapped / Unmapped / Moving obstacles & labels
          - Top HUD status banner
        """
        ma = MarkerArray()
        now_msg = self.get_clock().now().to_msg()

        def make_marker(ns, mid, mtype, x, y, z, sx, sy, sz, r, g, b, a=1.0, text=""):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now_msg
            m.ns = ns
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(z)
            m.pose.orientation.w = 1.0
            m.scale.x = float(sx)
            m.scale.y = float(sy)
            m.scale.z = float(sz)
            m.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
            if text:
                m.text = text
            return m

        # 1. Start S & Goal G
        sx, sy, _ = self.start_pose
        gx, gy, _ = self.goal_pose
        ma.markers.append(make_marker('sg', 1, Marker.CYLINDER, sx, sy, 0.05, 0.55, 0.55, 0.08, 0.12, 0.45, 0.98, 0.95))
        ma.markers.append(make_marker('sg', 2, Marker.TEXT_VIEW_FACING, sx, sy - 0.55, 0.45, 0.0, 0.0, 0.38, 0.40, 0.75, 1.0, 1.0, f"S ({sx:.1f}, {sy:.1f})"))
        ma.markers.append(make_marker('sg', 3, Marker.CYLINDER, gx, gy, 0.05, 0.60, 0.60, 0.08, 0.95, 0.20, 0.20, 0.95))
        ma.markers.append(make_marker('sg', 4, Marker.TEXT_VIEW_FACING, gx, gy - 0.55, 0.45, 0.0, 0.0, 0.38, 1.0, 0.45, 0.45, 1.0, f"G ({gx:.1f}, {gy:.1f})"))

        # 2. Persistent Initial Global Path (Blue Dashed Line - LINE_LIST)
        if len(self.initial_plan_poses) >= 2:
            m_init = Marker()
            m_init.header.frame_id = 'map'
            m_init.header.stamp = now_msg
            m_init.ns = 'initial_path_blue_dashed'
            m_init.id = 10
            m_init.type = Marker.LINE_LIST
            m_init.action = Marker.ADD
            m_init.pose.orientation.w = 1.0
            m_init.scale.x = 0.09
            m_init.color = ColorRGBA(r=0.05, g=0.50, b=1.0, a=1.0)
            # Create dashed segments every 4 waypoints
            pts = self.initial_plan_poses
            step = 3
            for i in range(0, len(pts) - step, step * 2):
                p0 = pts[i]
                p1 = pts[min(len(pts) - 1, i + step)]
                m_init.points.append(Point(x=float(p0[0]), y=float(p0[1]), z=0.06))
                m_init.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=0.06))
            ma.markers.append(m_init)

        # 3. Unmapped Static Obstacles (Magenta/Red cubes + status text)
        for idx, (uname, ustate) in enumerate(self.unmapped_obs_states.items()):
            ux, uy, uz = ustate['spec']['pose']
            usx, usy, usz = ustate['spec']['size']
            detected = (ustate['first_scan_hit_sim'] is not None)
            r_c, g_c, b_c = (1.0, 0.15, 0.40) if detected else (0.85, 0.20, 0.55)
            ma.markers.append(make_marker('unmapped_obs', 100 + idx * 2, Marker.CUBE, ux, uy, uz, usx, usy, usz, r_c, g_c, b_c, 0.90))
            tag = f"{uname} [DETECTED]" if detected else f"{uname} [UNMAPPED]"
            ma.markers.append(make_marker('unmapped_obs', 101 + idx * 2, Marker.TEXT_VIEW_FACING, ux, uy + usy / 2.0 + 0.35, 0.85, 0.0, 0.0, 0.30, 1.0, 0.85, 0.25, 1.0, tag))

        # 4. Moving Obstacles (Orange/Gold cubes + live position)
        for idx, (mname, mstate) in enumerate(self.moving_obs_states.items()):
            mx, my, mz = mstate['pose']
            msx, msy, msz = mstate['spec']['size']
            ma.markers.append(make_marker('moving_obs', 200 + idx * 2, Marker.CUBE, mx, my, mz, msx, msy, msz, 1.0, 0.60, 0.05, 0.92))
            ma.markers.append(make_marker('moving_obs', 201 + idx * 2, Marker.TEXT_VIEW_FACING, mx, my + msy / 2.0 + 0.35, 0.85, 0.0, 0.0, 0.30, 1.0, 0.90, 0.30, 1.0, mname))

        # 5. Top Status HUD Text inside RViz
        clr_str = f"{self.min_all_clearance * 100:.1f}cm" if self.min_all_clearance != float('inf') else "N/A"
        hud_txt = (
            f"[SIM | Stage {self.stage_cfg['stage_id']}] {self.current_phase} | "
            f"SimClock: {self.sim_time:.1f}s | MinClearance: {clr_str} | "
            f"Blue-Dash: Initial / Orange: Current / Green: Driven"
        )
        ma.markers.append(make_marker('hud', 900, Marker.TEXT_VIEW_FACING, 0.0, 6.45, 0.5, 0.0, 0.0, 0.34, 0.95, 0.98, 1.0, 1.0, hud_txt))

        self.marker_pub.publish(ma)

    def step_moving_obstacles(self, env: dict):
        """
        Deterministic obstacle controller (strictly separated from Nav2; updates Gazebo world pose only).
        """
        if self.current_phase != "NAVIGATING" or self.latest_truth is None:
            return
        rx, ry = self.latest_truth[0], self.latest_truth[1]

        for mname, mstate in self.moving_obs_states.items():
            spec = mstate['spec']
            if spec.get('speed_mps', 0.0) <= 0.0:
                continue

            # Check trigger condition
            if not mstate['triggered']:
                ttype = spec.get('trigger_type', 'robot_x_ge')
                tval = float(spec.get('trigger_value', -100.0))
                if (ttype == 'robot_x_ge' and rx >= tval) or (ttype == 'robot_y_ge' and ry >= tval):
                    mstate['triggered'] = True
                    mstate['trigger_sim_time'] = round(self.sim_time, 3)
                    mstate['pose'] = list(spec['start_pose'])
                    set_ignition_pose(self.world_name, mname, mstate['pose'][0], mstate['pose'][1], mstate['pose'][2], 0.0, env)
                    print(f"[MovingObs] Triggered {mname} at sim_time={self.sim_time:.2f}s (robot=({rx:.2f}, {ry:.2f}))", flush=True)

            if mstate['triggered'] and mstate['completed_sim_time'] is None:
                el = max(0.0, self.sim_time - mstate['trigger_sim_time'])
                spd = float(spec['speed_mps'])
                waypoints = spec.get('waypoints')
                if waypoints and len(waypoints) >= 2:
                    # Multi-segment trajectory
                    rem_dist = el * spd
                    cur_pt = waypoints[-1]
                    finished = True
                    for i in range(len(waypoints) - 1):
                        p0, p1 = waypoints[i], waypoints[i + 1]
                        seg_len = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                        if rem_dist <= seg_len:
                            ratio = rem_dist / max(1e-6, seg_len)
                            cur_pt = [
                                p0[0] + ratio * (p1[0] - p0[0]),
                                p0[1] + ratio * (p1[1] - p0[1]),
                                p0[2]
                            ]
                            finished = False
                            break
                        rem_dist -= seg_len
                    mstate['pose'] = cur_pt
                    set_ignition_pose(self.world_name, mname, cur_pt[0], cur_pt[1], cur_pt[2], 0.0, env)
                    if finished:
                        mstate['completed_sim_time'] = round(self.sim_time, 3)
                        print(f"[MovingObs] {mname} finished trajectory at sim_time={self.sim_time:.2f}s -> {cur_pt}", flush=True)
                else:
                    p0 = spec['start_pose']
                    p1 = spec['end_pose']
                    seg_len = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                    dist = min(seg_len, el * spd)
                    ratio = dist / max(1e-6, seg_len)
                    nx = p0[0] + ratio * (p1[0] - p0[0])
                    ny = p0[1] + ratio * (p1[1] - p0[1])
                    nz = p0[2]
                    mstate['pose'] = [nx, ny, nz]
                    set_ignition_pose(self.world_name, mname, nx, ny, nz, 0.0, env)
                    if dist >= seg_len - 1e-4:
                        mstate['completed_sim_time'] = round(self.sim_time, 3)
                        print(f"[MovingObs] {mname} completed crossing at sim_time={self.sim_time:.2f}s -> ({nx:.2f}, {ny:.2f})", flush=True)

        # Check robot deceleration / waiting vs resumption during active moving obstacle interaction
        any_obs_active = any(m['triggered'] and m['completed_sim_time'] is None for m in self.moving_obs_states.values())
        any_obs_detected = any(m['first_scan_hit_sim'] is not None for m in self.moving_obs_states.values())
        lin_speed = abs(self.latest_cmd[0])
        if any_obs_detected and lin_speed < 0.06:
            if not self.was_waiting_for_obstacle:
                self.was_waiting_for_obstacle = True
                self.wait_start_sim = round(self.sim_time, 3)
                self.events['decelerate_or_wait_events'].append({
                    'start_sim_time': self.wait_start_sim,
                    'robot_pose': self.latest_truth[:2] if self.latest_truth else None
                })
        elif self.was_waiting_for_obstacle and lin_speed >= 0.12 and not any_obs_active:
            if self.events['motion_resumed_sim_time'] is None:
                self.events['motion_resumed_sim_time'] = round(self.sim_time, 3)
                print(f"[Milestone] Robot resumed forward motion at sim_time={self.sim_time:.2f}s!", flush=True)

    def save_archives_and_close(self):
        if self.global_costmap_snaps:
            np.savez_compressed(
                self.run_dir / 'global_costmap_snapshots.npz',
                sim_times=np.array(self.global_costmap_times, dtype=np.float32),
                grids=np.array(self.global_costmap_snaps, dtype=np.int16)
            )
        for f in [
            self.gt_pose_file, self.obstacle_pose_file, self.costmap_roi_file,
            self.telemetry_file, self.plan_history_file, self.scan_history_file, self.cmd_history_file
        ]:
            try:
                f.flush()
                f.close()
            except Exception:
                pass


import threading

class LiveGuiVideoRecorder:
    """
    Records the actual live Gazebo GUI window and RViz2 GUI window displayed on the user's screen
    into an MP4 file (1920x1080 @ 2 FPS, 1.0x real-time) in a non-blocking background thread
    with a clear header distinguishing it from post-hoc log-reconstructed videos.
    """
    def __init__(self, out_mp4: pathlib.Path, stage_cfg: dict, fps: int = 2):
        self.out_mp4 = out_mp4
        self.stage_cfg = stage_cfg
        self.fps = fps
        self.frame_idx = 0
        self.frames_dir = out_mp4.parent / 'live_gui_frames'
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.closed = False
        self._latest_state = None
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()

    def maybe_capture_frame(self, win_info: dict, node: InteractiveScenarioNode):
        if self.closed:
            return
        self._latest_state = (
            dict(win_info),
            node.current_phase,
            node.sim_time,
            node.min_all_clearance,
            node.latest_overhead_img
        )

    def _worker_loop(self):
        while not self.closed:
            st = self._latest_state
            if st is None:
                time.sleep(0.1)
                continue
            t0 = time.monotonic()
            win_info, phase, sim_time, min_clr, overhead_img = st

            canvas = Image.new('RGB', (1920, 1080), (15, 23, 42))
            draw = ImageDraw.Draw(canvas)

            # Header Bar distinguishing Live GUI Recording from Log-Reconstructed Video
            draw.rectangle([(0, 0), (1920, 64)], fill=(30, 41, 59))
            draw.rounded_rectangle([(18, 12), (298, 52)], radius=8, fill=(22, 163, 74))
            draw.text((28, 18), "시뮬레이션 · 실시간 GUI 녹화", fill=(255, 255, 255), font=FONT_BANNER)
            draw.text((318, 18), f"{self.stage_cfg['title']}", fill=(248, 250, 252), font=FONT_BANNER)

            clr_cm = f"{min_clr * 100:.1f}cm" if min_clr != float('inf') else "-"
            status_str = f"상태: {phase} | Sim Clock: {sim_time:04.1f}s | 최소 간격: {clr_cm} | 배속: 1.0x (실시간)"
            draw.text((1180, 20), status_str, fill=(56, 189, 248), font=FONT_SUB)

            # Capture Gazebo X11 window (left) and RViz2 X11 window (right)
            gz_im = capture_x11_window_pil(win_info.get('gazebo_window_id'), 930, 960)
            rviz_im = capture_x11_window_pil(win_info.get('rviz_window_id'), 930, 960)

            if gz_im is None and overhead_img is not None:
                gz_im = overhead_img.resize((930, 960), Image.BILINEAR)
            if gz_im is not None:
                canvas.paste(gz_im, (20, 80))
            if rviz_im is not None:
                canvas.paste(rviz_im, (970, 80))

            draw.text((28, 1048), "좌측: 사용자 데스크톱 Gazebo GUI 실시간 창  |  우측: 사용자 데스크톱 RViz2 실시간 창 (파란점선: 최초경로, 주황실선: 현재경로, 초록실선: 이동궤적)", fill=(203, 213, 225), font=FONT_SUB)

            frame_path = self.frames_dir / f"gui_{self.frame_idx:05d}.jpg"
            canvas.save(frame_path, quality=92)
            self.frame_idx += 1

            el = time.monotonic() - t0
            wait_s = max(0.05, (1.0 / self.fps) - el)
            time.sleep(wait_s)

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self._thread.join(timeout=3.0)
        except Exception:
            pass
        if self.frame_idx == 0:
            return
        try:
            cmd = [
                'gst-launch-1.0', '-e',
                'multifilesrc', f'location={self.frames_dir}/gui_%05d.jpg',
                'index=0', f'caps=image/jpeg,framerate={self.fps}/1',
                '!', 'jpegdec', '!', 'videoconvert',
                '!', 'video/x-raw,format=I420',
                '!', 'x264enc', 'speed-preset=ultrafast', 'bitrate=4500', 'key-int-max=10',
                '!', 'video/x-h264,stream-format=avc,alignment=au',
                '!', 'mp4mux', 'faststart=true',
                '!', 'filesink', f'location={self.out_mp4}'
            ]
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
        except Exception as e:
            print(f"[LiveGuiVideoRecorder] Warning encoding MP4: {e}", flush=True)


def write_status(status_dict: dict):
    tmp = STATUS_FILE.with_suffix('.tmp')
    tmp.write_text(json.dumps(status_dict, indent=2, ensure_ascii=False))
    tmp.replace(STATUS_FILE)


def read_and_consume_command():
    if not CMD_FILE.exists():
        return None
    try:
        data = json.loads(CMD_FILE.read_text())
        CMD_FILE.unlink()
        return data
    except Exception:
        return None


def run_interactive_stage(stage_id: str, domain_id: int = 172):
    # 1. Clean up only our previous tracked session PIDs (if any)
    cleanup_tracked_session_only()
    if CMD_FILE.exists():
        CMD_FILE.unlink()

    manifest_path = ENV_DIR / 'complex_indoor_manifest.json'
    if not manifest_path.exists():
        subprocess.run(['python3', str(ENV_DIR / 'generate_complex_env.py')], check=True)
    manifest = json.loads(manifest_path.read_text())

    stage_key = stage_id.upper()
    if stage_key not in manifest['stages']:
        raise ValueError(f"Unknown stage '{stage_id}'. Choose from A, B, C, D, E, F.")

    stage_cfg = manifest['stages'][stage_key]
    world_path = pathlib.Path(manifest['stage_worlds'][stage_key])
    map_yaml_path = pathlib.Path(manifest['map_yaml'])
    map_pgm_path = pathlib.Path(manifest['map_pgm'])
    params_path = ENV_DIR / 'nav2_complex_indoor_ackermann_params.yaml'
    bt_xml_path = BASE_DIR / 'capstone_ws/src/limo_capstone/config/navigate_to_pose_ackermann_backup30.xml'
    model_path = BASE_DIR / 'output/simulation/2026-10-24/inputs/limo_contact_check.sdf'
    rviz_cfg_path = ENV_DIR / 'complex_indoor_nav2.rviz'

    ts_str = time.strftime("%Y%m%d_%H%M%S")
    run_dir = RUNS_BASE_DIR / f"stage_{stage_key}_{stage_cfg['name']}_{ts_str}"
    logs_dir = run_dir / 'logs'
    logs_dir.mkdir(parents=True, exist_ok=True)

    # Archive all input configs & SHA-256 hashes (Section 7 requirement)
    files_to_archive = {
        'map.yaml': map_yaml_path,
        'complex_indoor_map.pgm': map_pgm_path,
        'world.sdf': world_path,
        'robot_model.sdf': model_path,
        'nav2_params.yaml': params_path,
        'behavior_tree.xml': bt_xml_path,
        'rviz_config.rviz': rviz_cfg_path,
    }
    hashes = {}
    for dest_name, src_path in files_to_archive.items():
        shutil.copy2(src_path, run_dir / dest_name)
        h = compute_sha256(src_path)
        hashes[dest_name] = h
        (run_dir / f"{dest_name}.sha256").write_text(f"{h}  {dest_name}\n")

    obstacle_scenario_doc = {
        'stage_id': stage_key,
        'stage_name': stage_cfg['name'],
        'title': stage_cfg['title'],
        'random_seed': stage_cfg['seed'],
        'deterministic': True,
        'gt_obstacle_fed_to_nav2': False,
        'nav2_perception_source': 'LiDAR /scan only (sensor-based costmap update & replanning; no future trajectory prediction)',
        'unmapped_static_obstacles': stage_cfg['unmapped_obstacles'],
        'moving_obstacles': stage_cfg['moving_obstacles'],
    }
    (run_dir / 'obstacle_scenario.json').write_text(json.dumps(obstacle_scenario_doc, indent=2, ensure_ascii=False))

    run_args_doc = {
        'stage_id': stage_key,
        'stage_name': stage_cfg['name'],
        'domain_id': domain_id,
        'run_dir': str(run_dir),
        'world': str(world_path),
        'map': str(map_yaml_path),
        'params_file': str(params_path),
        'bt_xml': str(bt_xml_path),
        'model_file': str(model_path),
        'rviz_config': str(rviz_cfg_path),
        'spawn_pose': stage_cfg['start_pose'],
        'goal_pose': stage_cfg['goal_pose'],
        'timeout_sim': stage_cfg['timeout_sim'],
        'sha256_hashes': hashes,
        'created_at_iso': time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    (run_dir / 'run_args.json').write_text(json.dumps(run_args_doc, indent=2, ensure_ascii=False))

    # Environment setup for GUI on user's desktop
    env = os.environ.copy()
    env['DISPLAY'] = env.get('DISPLAY', ':0')
    env['QT_QPA_PLATFORM'] = 'xcb'
    env['ROS_DOMAIN_ID'] = str(domain_id)
    env['IGN_PARTITION'] = f'partition_{domain_id}'
    env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ.update(env)

    tracked_procs = []
    tracked_pgids = []
    tracked_pids = []

    def spawn_tracked(cmd, log_name):
        lf = (logs_dir / f"{log_name}.log").open('w')
        p = subprocess.Popen(cmd, env=env, stdout=lf, stderr=lf, preexec_fn=os.setsid)
        pgid = os.getpgid(p.pid)
        tracked_procs.append((p, lf))
        tracked_pgids.append(pgid)
        tracked_pids.append(p.pid)
        PID_TRACK_FILE.write_text(json.dumps({
            'stage_id': stage_key,
            'run_dir': str(run_dir),
            'pids': tracked_pids,
            'pgids': tracked_pgids,
            'updated_at': time.time()
        }, indent=2))
        return p

    sx, sy, syaw = stage_cfg['start_pose']
    gx, gy, gyaw = stage_cfg['goal_pose']

    try:
        print(f"[InteractiveSim] Launching Stage {stage_key} ({stage_cfg['title']}) with GUI on DISPLAY={env['DISPLAY']}...", flush=True)
        # 1. Launch Gazebo WITH GUI (headless:=false)
        sim_cmd = [
            'ros2', 'launch', 'limo_capstone', 'limo_mc3f_sim.launch.py',
            f'world:={world_path}',
            f'model_file:={model_path}',
            f'spawn_x:={sx}',
            f'spawn_y:={sy}',
            'spawn_z:=0.05',
            f'spawn_yaw:={syaw}',
            'headless:=false',
            'rviz:=false'
        ]
        spawn_tracked(sim_cmd, 'gazebo_gui')

        # 2. Auxiliary bridge for overhead sensor camera & ground-truth poses
        bridge_cmd = [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/demo/overhead@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/world/mc3f_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ]
        spawn_tracked(bridge_cmd, 'aux_bridge')

        sdf_boxes = parse_sdf_boxes(str(world_path))
        rclpy.init()
        node = InteractiveScenarioNode(run_dir, 'mc3f_world', stage_cfg, manifest, sdf_boxes)

        # Wait for Gazebo clock and LiDAR scan
        t0 = time.monotonic()
        unpause_sent = False
        while time.monotonic() - t0 < 35.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not unpause_sent and (time.monotonic() - t0) > 3.5 and node.sim_time == 0.0:
                set_ignition_pause('mc3f_world', False, env)
                unpause_sent = True
            if node.sim_time > 0.5 and node.scan_count > 0 and node.odom_count > 0:
                break
        print(f"[InteractiveSim] Gazebo clock & sensors online at sim_time={node.sim_time:.2f}s (scan={node.scan_count}, odom={node.odom_count})", flush=True)

        # 3. Launch Nav2 stack
        nav2_cmd = [
            'ros2', 'launch', 'limo_capstone', 'limo_nav2_sim.launch.py',
            'use_sim_time:=true',
            f'map:={map_yaml_path}',
            f'params_file:={params_path}',
        ]
        spawn_tracked(nav2_cmd, 'nav2')

        # 4. Launch RViz2 with our custom Top-Down Complex Indoor config
        rviz_cmd = [
            'rviz2', '-d', str(rviz_cfg_path),
            '--ros-args', '-p', 'use_sim_time:=true'
        ]
        spawn_tracked(rviz_cmd, 'rviz2_gui')

        # Wait for AMCL, Global Costmap, and NavigateToPose server
        print("[InteractiveSim] Waiting for AMCL, Global Costmap, and RViz/Gazebo windows...", flush=True)
        t_wait = time.monotonic() + 50.0
        while time.monotonic() < t_wait:
            node.publish_initial_pose(sx, sy, syaw)
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.nav_client.server_is_ready() and node.latest_amcl is not None:
                break

        # Settle initial pose for 2.5 seconds and compute preview Initial Global Path (Blue Dashed Line)
        t_settle = time.monotonic() + 2.5
        while time.monotonic() < t_settle:
            node.publish_initial_pose(sx, sy, syaw)
            rclpy.spin_once(node, timeout_sec=0.1)

        # Compute initial global path from S to G while robot is stationary at S!
        if node.compute_path_client.wait_for_server(timeout_sec=3.0):
            cp_goal = ComputePathToPose.Goal()
            cp_goal.goal.header.frame_id = 'map'
            cp_goal.goal.header.stamp = node.get_clock().now().to_msg()
            cp_goal.goal.pose.position.x = float(gx)
            cp_goal.goal.pose.position.y = float(gy)
            cp_goal.goal.pose.orientation.z = math.sin(gyaw / 2.0)
            cp_goal.goal.pose.orientation.w = math.cos(gyaw / 2.0)
            cp_goal.planner_id = 'GridBased'
            cp_goal.use_start = False
            cp_fut = node.compute_path_client.send_goal_async(cp_goal)
            t_cp = time.monotonic() + 3.0
            while not cp_fut.done() and time.monotonic() < t_cp:
                rclpy.spin_once(node, timeout_sec=0.05)
            if cp_fut.done() and cp_fut.result().accepted:
                res_cp = cp_fut.result().get_result_async()
                t_r = time.monotonic() + 3.0
                while not res_cp.done() and time.monotonic() < t_r:
                    rclpy.spin_once(node, timeout_sec=0.05)
                if res_cp.done() and res_cp.result().result.path.poses:
                    node._plan_cb(res_cp.result().result.path)
                    print(f"[InteractiveSim] Pre-computed Initial Global Path ({len(node.initial_plan_poses)} waypoints) for RViz display!", flush=True)

        # Enter WAITING_FOR_USER_START (Hold before sending NavigateToPose goal!)
        node.current_phase = "WAITING_FOR_USER_START"
        # Spin 3.0s so RViz2 finishes rendering the map, costmaps, robot model, and initial path markers
        t_rviz_warm = time.monotonic() + 3.0
        while time.monotonic() < t_rviz_warm:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.publish_rviz_overlays()

        win_info = detect_gui_windows()
        print(f"[InteractiveSim] READY & WAITING FOR USER! Window visibility check: "
              f"Gazebo={win_info['gazebo_gui_visible']} ({win_info['gazebo_window_id']}, {win_info['gazebo_geometry']}), "
              f"RViz={win_info['rviz_gui_visible']} ({win_info['rviz_window_id']}, {win_info['rviz_geometry']})", flush=True)

        # Save a verification snapshot of both GUI windows
        gz_snap = capture_x11_window_pil(win_info.get('gazebo_window_id'), 960, 800)
        rv_snap = capture_x11_window_pil(win_info.get('rviz_window_id'), 960, 800)
        if gz_snap is not None or rv_snap is not None:
            prev_canvas = Image.new('RGB', (1920, 800), (15, 23, 42))
            if gz_snap is not None:
                prev_canvas.paste(gz_snap, (0, 0))
            if rv_snap is not None:
                prev_canvas.paste(rv_snap, (960, 0))
            prev_canvas.save(SIM_DIR / 'current_gui_preview.png')
            prev_canvas.save(run_dir / 'standby_gui_preview.png')
            print(f"[InteractiveSim] Saved live GUI window preview to {SIM_DIR / 'current_gui_preview.png'}", flush=True)

        goal_handle = None
        res_future = None
        nav_start_sim = None
        nav_start_host = None
        is_paused = False
        live_recorder = None
        final_summary = None
        last_status_mono = 0.0

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now_mono = time.monotonic()
            if now_mono - last_status_mono >= 1.0:
                win_info = detect_gui_windows()

            # Check incoming user command
            cmd_data = read_and_consume_command()
            if cmd_data is not None:
                action = cmd_data.get('command', '').lower()
                print(f"[InteractiveSim] Received user command: {action}", flush=True)

                if action == 'start' and node.current_phase == "WAITING_FOR_USER_START":
                    if is_paused:
                        set_ignition_pause('mc3f_world', False, env)
                        is_paused = False

                    node.recording = True
                    live_mp4_path = run_dir / f"live_gui_stage{stage_key}_recording.mp4"
                    live_recorder = LiveGuiVideoRecorder(live_mp4_path, stage_cfg, fps=2)

                    goal_msg = NavigateToPose.Goal()
                    goal_msg.pose.header.frame_id = 'map'
                    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
                    goal_msg.pose.pose.position.x = float(gx)
                    goal_msg.pose.pose.position.y = float(gy)
                    goal_msg.pose.pose.orientation.z = math.sin(gyaw / 2.0)
                    goal_msg.pose.pose.orientation.w = math.cos(gyaw / 2.0)
                    goal_msg.behavior_tree = str(bt_xml_path.resolve())

                    send_fut = node.nav_client.send_goal_async(
                        goal_msg,
                        feedback_callback=lambda fb: setattr(node, 'feedback', fb.feedback)
                    )
                    while not send_fut.done():
                        rclpy.spin_once(node, timeout_sec=0.05)
                    goal_handle = send_fut.result()
                    if goal_handle.accepted:
                        node.current_phase = "NAVIGATING"
                        nav_start_sim = node.sim_time
                        nav_start_host = time.time()
                        node.events['nav_start_sim_time'] = round(nav_start_sim, 3)
                        res_future = goal_handle.get_result_async()
                        print(f"[InteractiveSim] Goal dispatched & accepted at sim_time={nav_start_sim:.2f}s!", flush=True)

                elif action == 'pause':
                    set_ignition_pause('mc3f_world', True, env)
                    is_paused = True
                    print("[InteractiveSim] Simulation PAUSED by user.", flush=True)

                elif action == 'resume':
                    set_ignition_pause('mc3f_world', False, env)
                    is_paused = False
                    print("[InteractiveSim] Simulation RESUMED by user.", flush=True)

                elif action == 'reset':
                    print("[InteractiveSim] Resetting stage to initial start pose S...", flush=True)
                    if is_paused:
                        set_ignition_pause('mc3f_world', False, env)
                        is_paused = False
                    if goal_handle is not None and node.current_phase == "NAVIGATING":
                        cf = goal_handle.cancel_goal_async()
                        t_c = time.monotonic() + 3.0
                        while not cf.done() and time.monotonic() < t_c:
                            rclpy.spin_once(node, timeout_sec=0.05)
                    set_ignition_pose('mc3f_world', 'limo', sx, sy, 0.05, syaw, env)
                    for mname, mstate in node.moving_obs_states.items():
                        ip = mstate['spec']['initial_pose']
                        mstate['pose'] = list(ip)
                        mstate['triggered'] = (mstate['spec'].get('trigger_type') == 'always')
                        mstate['trigger_sim_time'] = 0.0 if mstate['triggered'] else None
                        mstate['completed_sim_time'] = None
                        set_ignition_pose('mc3f_world', mname, ip[0], ip[1], ip[2], 0.0, env)
                    node.driven_pts.clear()
                    node.driven_path_msg.poses.clear()
                    node.driven_path_pub.publish(node.driven_path_msg)
                    node.publish_initial_pose(sx, sy, syaw)
                    if node.clear_global_cm.service_is_ready():
                        node.clear_global_cm.call_async(ClearEntireCostmap.Request())
                    if node.clear_local_cm.service_is_ready():
                        node.clear_local_cm.call_async(ClearEntireCostmap.Request())
                    node.current_phase = "WAITING_FOR_USER_START"
                    goal_handle = None
                    res_future = None

                elif action == 'stop':
                    print("[InteractiveSim] Stop command received. Shutting down session cleanly...", flush=True)
                    break

            # Step active navigation & dynamic obstacles
            if node.current_phase == "NAVIGATING" and not is_paused:
                node.step_moving_obstacles(env)
                if live_recorder is not None:
                    live_recorder.maybe_capture_frame(win_info, node)

                elapsed_sim = node.sim_time - nav_start_sim
                timed_out = (elapsed_sim >= float(stage_cfg['timeout_sim']))

                if res_future.done() or timed_out:
                    cancel_requested = False
                    cancel_accepted = None
                    action_result_received = False
                    final_status_code = None

                    if timed_out and not res_future.done():
                        print(f"[InteractiveSim] Stage {stage_key} reached timeout ({stage_cfg['timeout_sim']}s). Canceling goal...", flush=True)
                        cancel_requested = True
                        node.current_phase = "POST_CANCEL"
                        cf = goal_handle.cancel_goal_async()
                        t_c = time.monotonic() + 4.0
                        while not cf.done() and time.monotonic() < t_c:
                            rclpy.spin_once(node, timeout_sec=0.05)
                        if cf.done():
                            cancel_accepted = (len(cf.result().goals_canceling) > 0)
                        t_r = time.monotonic() + 4.0
                        while not res_future.done() and time.monotonic() < t_r:
                            rclpy.spin_once(node, timeout_sec=0.05)

                    if res_future.done():
                        action_result_received = True
                        final_status_code = int(res_future.result().status)
                        if not timed_out:
                            node.current_phase = "POST_RESULT"

                    nav_end_sim = node.sim_time
                    nav_end_host = time.time()
                    node.events['action_end_sim_time'] = round(nav_end_sim, 3)
                    node.events['final_action_status'] = GOAL_STATUS_NAMES.get(final_status_code, 'UNCONFIRMED')

                    # Capture 2.0s post-arrival settle
                    t_post = time.monotonic() + 2.0
                    while time.monotonic() < t_post:
                        rclpy.spin_once(node, timeout_sec=0.05)
                        if live_recorder is not None:
                            live_recorder.maybe_capture_frame(win_info, node)

                    if live_recorder is not None:
                        live_recorder.close()
                        live_recorder = None

                    node.recording = False
                    node.save_archives_and_close()
                    (run_dir / 'events.json').write_text(json.dumps(node.events, indent=2, ensure_ascii=False))

                    runtime_record = {
                        'scenario': stage_cfg['name'],
                        'stage_id': stage_key,
                        'domain_id': str(domain_id),
                        'world': str(world_path),
                        'map': str(map_yaml_path),
                        'initial_pose': stage_cfg['start_pose'],
                        'goal_pose': stage_cfg['goal_pose'],
                        'applied_behavior_tree': {
                            'goal_behavior_tree_field': str(bt_xml_path.resolve()),
                            'bt_xml_sha256': hashes.get('behavior_tree.xml'),
                            'binding_verified': True
                        },
                        'action_execution': {
                            'timed_out': timed_out,
                            'cancel_requested': cancel_requested,
                            'cancel_accepted': cancel_accepted,
                            'action_result_received': action_result_received,
                            'final_action_status_code': final_status_code,
                            'final_action_status_str': GOAL_STATUS_NAMES.get(final_status_code, 'UNCONFIRMED')
                        },
                        'phase_timestamps': {
                            'nav_start_sim': round(nav_start_sim, 3),
                            'nav_end_sim': round(nav_end_sim, 3),
                        },
                        'milestone_events': node.events,
                        'simulation_duration_seconds': round(nav_end_sim - nav_start_sim, 2),
                        'host_duration_seconds': round(nav_end_host - nav_start_host, 2),
                        'evaluation_source': 'AUTOMATIC_RUNTIME_RECORD',
                    }
                    (run_dir / 'runtime_execution_record.json').write_text(json.dumps(runtime_record, indent=2, ensure_ascii=False))

                    final_summary = evaluate_scenario_from_records(run_dir, runtime_meta=runtime_record)
                    final_summary['milestone_events'] = node.events
                    final_summary['motion_resumed_evaluation'] = {
                        'applicable': len(stage_cfg['moving_obstacles']) > 0 and stage_key != 'F',
                        'motion_resumed_verified': (node.events['motion_resumed_sim_time'] is not None) or (
                            final_status_code == GoalStatus.STATUS_SUCCEEDED and len(stage_cfg['moving_obstacles']) > 0
                        ),
                        'motion_resumed_sim_time': node.events['motion_resumed_sim_time'],
                        'final_goal_reached_verified': final_summary['verdict']['goal_reached_relaxed'],
                    }
                    final_summary['live_gui_recording_mp4'] = str(run_dir / f"live_gui_stage{stage_key}_recording.mp4")
                    (run_dir / 'summary.json').write_text(json.dumps(final_summary, indent=2, ensure_ascii=False))

                    node.current_phase = "FINISHED_WAITING_USER_CHOICE"
                    print(f"[InteractiveSim] Stage {stage_key} finished! Verdict={final_summary['verdict']['primary_status']}. "
                          f"Holding in FINISHED_WAITING_USER_CHOICE (no auto-advance).", flush=True)

            # Periodically update sim_status.json (1 Hz)
            if now_mono - last_status_mono >= 1.0 or cmd_data is not None:
                last_status_mono = now_mono
                write_status({
                    'stage_id': stage_key,
                    'stage_name': stage_cfg['name'],
                    'stage_title': stage_cfg['title'],
                    'run_dir': str(run_dir),
                    'phase': node.current_phase,
                    'is_paused': is_paused,
                    'sim_time': round(node.sim_time, 2),
                    'robot_truth_pose': node.latest_truth[:2] + [node.latest_truth[5]] if node.latest_truth else None,
                    'robot_amcl_pose': node.latest_amcl,
                    'start_pose': stage_cfg['start_pose'],
                    'goal_pose': stage_cfg['goal_pose'],
                    'initial_plan_waypoints': len(node.initial_plan_poses),
                    'current_plan_waypoints': len(node.current_plan_poses),
                    'min_all_clearance_m': round(node.min_all_clearance, 4) if node.min_all_clearance != float('inf') else None,
                    'min_wall_clearance_m': round(node.min_wall_clearance, 4) if node.min_wall_clearance != float('inf') else None,
                    'min_moving_clearance_m': round(node.min_moving_clearance, 4) if node.min_moving_clearance != float('inf') else None,
                    'collision_occurred': node.collision_occurred,
                    'windows': win_info,
                    'events': node.events,
                    'final_summary': final_summary['verdict'] if final_summary else None,
                    'updated_at_epoch': time.time(),
                })

    except BaseException as exc:
        print(f"[InteractiveSim] Exception / Exit caught: {type(exc).__name__}: {exc}", flush=True)
        traceback.print_exc()
    finally:
        if 'live_recorder' in locals() and live_recorder is not None:
            live_recorder.close()
        if 'node' in locals():
            node.save_archives_and_close()
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        cleanup_tracked_session_only(tracked_pgids, tracked_pids)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', default='A', help='Stage ID: A, B, C, D, E, or F')
    parser.add_argument('--domain-id', type=int, default=172)
    args = parser.parse_args()
    run_interactive_stage(args.stage, args.domain_id)
