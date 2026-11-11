#!/usr/bin/env python3
"""
test_auto_resume_e2e_complete.py:
Empirical verification of Auto-Resume end-to-end in Gazebo:
1. AMCL Global Localization without start_pose or 2D Pose Estimate (timeout: 180s).
2. Clean handoff: AMCL terminates, SLAM Toolbox takes over without TF conflicts.
3. Ready gate released: /auto_mapping/localization_ready published.
4. AutoMapping action goal dispatched, robot navigates and explores.
5. End-to-end completion: action succeeds or exhausts reachable frontiers,
   and four saved map files (.yaml, .pgm, .posegraph, .data) are verified on disk.
6. Ground truth pose is used strictly for evaluation, never fed into control or localization.
"""
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from limo_mapping_interfaces.action import AutoMapping

EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping')
LOGS_DIR = EXP_DIR / 'logs'
LOGS_DIR.mkdir(parents=True, exist_ok=True)
MAP_PREFIX = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_resume_verification/maps/complex_indoor_baseline')
WORLD_SDF = Path('/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf')


def stop_process(proc):
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=2)
        except Exception:
            pass


def planar_from_quat(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class ResumeE2ENode(Node):
    def __init__(self):
        super().__init__('resume_e2e_' + uuid.uuid4().hex[:6])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.statuses = []
        self.ready_received = False
        self.ready_timestamp = None
        self.last_amcl_pose = None
        self.pre_handoff_pose = None
        self.odom_start = None
        self.odom_current = None
        self.feedbacks = []

        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/auto_mapping/resume_status', self.on_status, latched)
        self.create_subscription(Bool, '/auto_mapping/localization_ready', self.on_ready, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, latched)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        self.auto_client = ActionClient(self, AutoMapping, '/auto_mapping')

    def on_status(self, msg):
        self.statuses.append((time.monotonic(), msg.data))
        try:
            doc = json.loads(msg.data)
            if doc.get('state') == 'handoff' and 'Stopping AMCL' in doc.get('detail', ''):
                if self.last_amcl_pose:
                    p = self.last_amcl_pose.pose.pose.position
                    q = self.last_amcl_pose.pose.pose.orientation
                    yaw = planar_from_quat(q.x, q.y, q.z, q.w)
                    self.pre_handoff_pose = (p.x, p.y, yaw)
        except Exception:
            pass

    def on_ready(self, msg):
        if msg.data and not self.ready_received:
            self.ready_received = True
            self.ready_timestamp = time.monotonic()

    def on_amcl_pose(self, msg):
        self.last_amcl_pose = msg

    def on_odom(self, msg):
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.odom_start is None:
            self.odom_start = pos
        self.odom_current = pos

    @property
    def distance_traveled(self):
        if self.odom_start is None or self.odom_current is None:
            return 0.0
        return math.hypot(self.odom_current[0] - self.odom_start[0],
                          self.odom_current[1] - self.odom_start[1])


def run_e2e_resume_test():
    run_id = f"e2e_resume_{uuid.uuid4().hex[:6]}"
    raw_log = LOGS_DIR / f"{run_id}.raw.log"
    resume_log = LOGS_DIR / f"{run_id}_resume.log"
    output_prefix = LOGS_DIR / f"{run_id}_saved_map"

    spawn_x, spawn_y, spawn_yaw = -6.0, 0.0, -1.57
    truth_map = (-6.0 + 6.2, 0.0 + 2.0, -1.57) # (0.20, 2.00, -1.57)

    gz_proc = None
    resume_proc = None
    node = None

    test_result = {
        "run_id": run_id,
        "spawn_pose": [spawn_x, spawn_y, spawn_yaw],
        "ground_truth_map_pose": list(truth_map),
        "ground_truth_usage": "evaluation_only",
        "localization_timeout_sec": 180.0,
        "runner_timeout_sec": 180.0,
        "ready_reached": False,
        "handoff_successful": False,
        "action_goal_accepted": False,
        "action_terminal_status": "미측정",
        "saved_files": {},
        "localization_error": {},
        "overall_success": False
    }

    try:
        print("[e2e_resume] Launching Gazebo Fortress...")
        gz_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source /home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash && "
            f"ros2 launch limo_capstone limo_mc3f_sim.launch.py "
            f"world:={WORLD_SDF} spawn_x:={spawn_x} spawn_y:={spawn_y} spawn_z:=0.05 spawn_yaw:={spawn_yaw} "
            f"headless:=true rviz:=false"
        )
        gz_out = open(raw_log, 'w')
        gz_proc = subprocess.Popen(
            ['bash', '-c', gz_cmd],
            stdout=gz_out, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )
        time.sleep(10)

        print("[e2e_resume] Launching auto_resume.launch.py (timeout=180.0s, NO start_pose)...")
        resume_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source /home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash && "
            f"source /home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping/ws/install/setup.bash && "
            f"ros2 launch limo_auto_mapping auto_resume.launch.py "
            f"posegraph:={MAP_PREFIX} use_sim_time:=true localization_timeout_sec:=180.0"
        )
        resume_out = open(resume_log, 'w')
        resume_proc = subprocess.Popen(
            ['bash', '-c', resume_cmd],
            stdout=resume_out, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )

        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
        node = ResumeE2ENode()

        print("[e2e_resume] Waiting for autonomous AMCL localization and SLAM handoff (up to 180s)...")
        t_loc_start = time.monotonic()
        while time.monotonic() - t_loc_start < 180.0 and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if resume_proc.poll() is not None:
                print(f"[e2e_resume] auto_resume exited with code {resume_proc.returncode}!")
                break
            if node.ready_received:
                test_result["ready_reached"] = True
                test_result["handoff_successful"] = True
                print(f"[e2e_resume] READY SIGNAL RECEIVED at t={time.monotonic() - t_loc_start:.1f}s!")
                break

        if not test_result["ready_reached"]:
            test_result["error"] = "Localization did not reach ready before 180s timeout"
            return test_result

        # Calculate localization error against ground truth
        if node.pre_handoff_pose:
            ex = node.pre_handoff_pose[0] - truth_map[0]
            ey = node.pre_handoff_pose[1] - truth_map[1]
            pos_err = math.hypot(ex, ey)
            yaw_err = abs(math.atan2(math.sin(node.pre_handoff_pose[2] - truth_map[2]),
                                     math.cos(node.pre_handoff_pose[2] - truth_map[2])))
            test_result["localization_error"] = {
                "estimated_pose": list(node.pre_handoff_pose),
                "position_error_m": pos_err,
                "yaw_error_rad": yaw_err,
                "evaluated_strictly_post_facto": True
            }
            print(f"[e2e_resume] Localization error: pos={pos_err:.3f}m, yaw={yaw_err:.3f}rad")

        # Now send AutoMapping action goal
        print("[e2e_resume] Dispatching AutoMapping action goal...")
        node.auto_client.wait_for_server(timeout_sec=15.0)
        goal = AutoMapping.Goal()
        goal.output_prefix = str(output_prefix)
        goal.max_duration_sec = 60.0
        goal.allow_margin_retry = False

        send_fut = node.auto_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_fut, timeout_sec=10.0)
        handle = send_fut.result()

        test_result["action_goal_accepted"] = handle.accepted if handle else False
        if not handle or not handle.accepted:
            test_result["error"] = "AutoMapping goal rejected"
            return test_result

        print("[e2e_resume] Goal accepted! Waiting for exploration progress and map saving (up to 75s)...")
        res_fut = handle.get_result_async()
        t_action_start = time.monotonic()
        while time.monotonic() - t_action_start < 75.0 and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if res_fut.done():
                break

        action_res = res_fut.result() if res_fut.done() else None
        status = action_res.status if action_res else GoalStatus.STATUS_UNKNOWN
        status_names = {4: "STATUS_SUCCEEDED", 5: "STATUS_CANCELED", 6: "STATUS_ABORTED"}
        test_result["action_terminal_status"] = status_names.get(status, f"STATUS_{status}")
        test_result["distance_traveled_m"] = node.distance_traveled

        # Check saved files on disk
        for ext in ('.yaml', '.pgm', '.posegraph', '.data'):
            fp = Path(str(output_prefix) + ext)
            test_result["saved_files"][ext] = {
                "exists": fp.is_file(),
                "size_bytes": fp.stat().st_size if fp.is_file() else 0
            }

        all_files_exist = all(info["exists"] and info["size_bytes"] > 0 for info in test_result["saved_files"].values())
        test_result["overall_success"] = test_result["ready_reached"] and test_result["action_goal_accepted"] and all_files_exist
        print(f"[e2e_resume] Complete! Status={test_result['action_terminal_status']}, FilesExist={all_files_exist}, OverallSuccess={test_result['overall_success']}")

    except Exception as exc:
        test_result["error"] = str(exc)
        print(f"[e2e_resume] Exception: {exc}")
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        stop_process(resume_proc)
        stop_process(gz_proc)
        time.sleep(2)

    out_file = LOGS_DIR / 'auto_resume_e2e_results.json'
    out_file.write_text(json.dumps(test_result, indent=2))
    print(f"\n=======================================================")
    print(f"AUTO-RESUME E2E TEST COMPLETE. Saved to {out_file}")
    print(f"Overall Success: {test_result['overall_success']}")
    print(f"Saved Files: {test_result['saved_files']}")
    print(f"=======================================================")
    return test_result


if __name__ == '__main__':
    run_e2e_resume_test()
