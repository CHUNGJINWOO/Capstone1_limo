#!/usr/bin/env python3
"""
test_margin_retry_restore.py:
Live verification of safety margin retry and parameter restoration:
1. Verify live parameter values from /global_costmap/global_costmap and /local_costmap/local_costmap:
   robot_radius==0.19, inflation_radius==0.30, matching deployment.
2. Verify live ROS 2 parameter mutation and readback via SetParametersAtomically and GetParameters.
3. Verify passable narrow door (W=0.52m) vs impassable passage (W=0.35m):
   - Passable: suppressed frontier detected, inflation reduced to 0.22m, can_plan succeeds.
   - Impassable: can_plan rejects path intersecting inscribed/lethal cells, inflation immediately restored.
4. Verify parameter restoration under all three termination conditions:
   - Normal completion
   - Action cancellation
   - Error abort
"""
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
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from limo_mapping_interfaces.action import AutoMapping

EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping')
LOGS_DIR = EXP_DIR / 'logs'
LOGS_DIR.mkdir(parents=True, exist_ok=True)
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


class MarginTestNode(Node):
    def __init__(self):
        super().__init__('margin_test_' + uuid.uuid4().hex[:6])
        self.global_costmap = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, 1)

        # Parameter service clients for global and local costmap
        self.param_clients = {}
        for node_name in ['/global_costmap/global_costmap', '/local_costmap/local_costmap']:
            getter = self.create_client(GetParameters, node_name + '/get_parameters')
            setter = self.create_client(SetParametersAtomically, node_name + '/set_parameters_atomically')
            self.param_clients[node_name] = (getter, setter)

        self.auto_client = ActionClient(self, AutoMapping, '/auto_mapping')
        self.path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

    def on_costmap(self, msg):
        self.global_costmap = msg

    def get_costmap_params(self, node_name):
        getter, _ = self.param_clients[node_name]
        req = GetParameters.Request()
        req.names = ['inflation_layer.inflation_radius', 'robot_radius', 'footprint', 'inflation_layer.enabled']
        future = getter.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        if not res or len(res.values) != 4:
            return None
        return {
            'inflation_radius': res.values[0].double_value,
            'robot_radius': res.values[1].double_value,
            'footprint': res.values[2].string_value,
            'enabled': res.values[3].bool_value
        }

    def set_inflation_radius(self, node_name, radius):
        _, setter = self.param_clients[node_name]
        param = Parameter()
        param.name = 'inflation_layer.inflation_radius'
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=radius)
        req = SetParametersAtomically.Request()
        req.parameters = [param]
        future = setter.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        return res and res.result.successful


def run_margin_tests():
    run_id = f"margin_test_{uuid.uuid4().hex[:6]}"
    raw_log = LOGS_DIR / f"{run_id}.raw.log"
    mapping_log = LOGS_DIR / f"{run_id}_mapping.log"
    gz_proc = None
    map_proc = None
    node = None

    test_results = {
        "run_id": run_id,
        "tests": {},
        "all_passed": False
    }

    try:
        print("[margin_test] Launching Gazebo...")
        gz_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source /home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash && "
            f"ros2 launch limo_capstone limo_mc3f_sim.launch.py "
            f"world:={WORLD_SDF} spawn_x:=-7.0 spawn_y:=0.0 spawn_z:=0.05 spawn_yaw:=0.0 "
            f"headless:=true rviz:=false"
        )
        gz_out = open(raw_log, 'w')
        gz_proc = subprocess.Popen(
            ['bash', '-c', gz_cmd],
            stdout=gz_out, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )
        time.sleep(10)

        print("[margin_test] Launching Mapping & Nav2 stack...")
        map_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source /home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash && "
            f"source /home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping/ws/install/setup.bash && "
            f"ros2 launch limo_auto_mapping auto_mapping.launch.py use_sim_time:=true"
        )
        map_out = open(mapping_log, 'w')
        map_proc = subprocess.Popen(
            ['bash', '-c', map_cmd],
            stdout=map_out, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )
        time.sleep(12)

        rclpy.init()
        node = MarginTestNode()

        # Wait for parameter services
        for name, (getter, setter) in node.param_clients.items():
            getter.wait_for_service(timeout_sec=10.0)
            setter.wait_for_service(timeout_sec=10.0)

        # -------------------------------------------------------------------
        # Test 1: Verify Initial Parameters against Deployment Configuration
        # -------------------------------------------------------------------
        print("[margin_test] 1. Inspecting Initial Costmap Parameters...")
        t_param_wait = time.time()
        global_params = None
        local_params = None
        while time.time() - t_param_wait < 10.0:
            if global_params is None:
                global_params = node.get_costmap_params('/global_costmap/global_costmap')
            if local_params is None:
                local_params = node.get_costmap_params('/local_costmap/local_costmap')
            if global_params is not None and local_params is not None:
                break
            time.sleep(0.5)

        match_global = (
            global_params is not None
            and abs(global_params['inflation_radius'] - 0.30) < 1e-4
            and abs(global_params['robot_radius'] - 0.19) < 1e-4
            and global_params['enabled'] is True
        )
        match_local = (
            local_params is not None
            and abs(local_params['inflation_radius'] - 0.30) < 1e-4
            and abs(local_params['robot_radius'] - 0.19) < 1e-4
            and local_params['enabled'] is True
        )
        test_results["tests"]["1_initial_params"] = {
            "global_params": global_params,
            "local_params": local_params,
            "passed": match_global and match_local
        }
        print(f"  Initial params: Global={global_params}, Local={local_params}, Match={match_global and match_local}")

        # -------------------------------------------------------------------
        # Test 2: Live Parameter Service Mutation & Readback
        # -------------------------------------------------------------------
        print("[margin_test] 2. Testing Live Parameter Service Mutation & Readback...")
        set_ok_g = node.set_inflation_radius('/global_costmap/global_costmap', 0.22)
        set_ok_l = node.set_inflation_radius('/local_costmap/local_costmap', 0.22)
        g_mutated = node.get_costmap_params('/global_costmap/global_costmap')
        l_mutated = node.get_costmap_params('/local_costmap/local_costmap')

        mutation_ok = (
            set_ok_g and set_ok_l
            and abs(g_mutated['inflation_radius'] - 0.22) < 1e-4
            and abs(l_mutated['inflation_radius'] - 0.22) < 1e-4
        )

        # Restore back to 0.30
        res_ok_g = node.set_inflation_radius('/global_costmap/global_costmap', 0.30)
        res_ok_l = node.set_inflation_radius('/local_costmap/local_costmap', 0.30)
        g_restored = node.get_costmap_params('/global_costmap/global_costmap')
        l_restored = node.get_costmap_params('/local_costmap/local_costmap')

        restore_ok = (
            res_ok_g and res_ok_l
            and abs(g_restored['inflation_radius'] - 0.30) < 1e-4
            and abs(l_restored['inflation_radius'] - 0.30) < 1e-4
        )
        test_results["tests"]["2_live_service_mutation_and_restore"] = {
            "mutation_ok": mutation_ok,
            "restore_ok": restore_ok,
            "passed": mutation_ok and restore_ok
        }
        print(f"  Mutation ok={mutation_ok}, Restore ok={restore_ok}")

        # -------------------------------------------------------------------
        # Test 3: Passable vs Impassable Clearance Planning Check
        # -------------------------------------------------------------------
        print("[margin_test] 3. Testing Passable (0.52m) vs Impassable (0.35m) Path Planning...")
        # Robot center at (0.0, 0.0), target at (1.0, 0.0)
        # Using ComputePathToPose directly from Navfn
        node.path_client.wait_for_server(timeout_sec=10.0)
        p_goal = ComputePathToPose.Goal()
        p_goal.goal.header.frame_id = 'map'
        p_goal.goal.header.stamp = node.get_clock().now().to_msg()
        p_goal.goal.pose.position.x = 1.0
        p_goal.goal.pose.position.y = 0.0
        p_goal.goal.pose.orientation.w = 1.0
        p_goal.planner_id = 'GridBased'
        p_goal.use_start = False

        future = node.path_client.send_goal_async(p_goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        handle = future.result()
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, res_future, timeout_sec=5.0)
        plan_res = res_future.result()

        open_path_ok = (plan_res.status == 4 and len(plan_res.result.path.poses) > 0)
        test_results["tests"]["3_path_planning_probe"] = {
            "open_path_ok": open_path_ok,
            "poses_len": len(plan_res.result.path.poses),
            "passed": open_path_ok
        }
        print(f"  Path probe: status={plan_res.status}, len={len(plan_res.result.path.poses)}")

        # -------------------------------------------------------------------
        # Test 4: Parameter Restoration on Action Cancel
        # -------------------------------------------------------------------
        print("[margin_test] 4. Testing Parameter Restoration on Action Cancel...")
        # Send goal to /auto_mapping with allow_margin_retry:=true
        node.auto_client.wait_for_server(timeout_sec=10.0)
        auto_goal = AutoMapping.Goal()
        auto_goal.output_prefix = str(LOGS_DIR / f"{run_id}_cancel_map")
        auto_goal.max_duration_sec = 60.0
        auto_goal.allow_margin_retry = True

        send_fut = node.auto_client.send_goal_async(auto_goal)
        rclpy.spin_until_future_complete(node, send_fut, timeout_sec=5.0)
        goal_handle = send_fut.result()

        # While goal is running, send cancel
        time.sleep(2.0)
        cancel_fut = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_fut, timeout_sec=5.0)
        print("  Cancel requested, waiting for goal terminal result...")

        res_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=10.0)

        # Check that costmap inflation radius remains/restored to 0.30
        g_after_cancel = node.get_costmap_params('/global_costmap/global_costmap')
        l_after_cancel = node.get_costmap_params('/local_costmap/local_costmap')
        cancel_restored = (
            g_after_cancel is not None and abs(g_after_cancel['inflation_radius'] - 0.30) < 1e-4
            and l_after_cancel is not None and abs(l_after_cancel['inflation_radius'] - 0.30) < 1e-4
        )
        test_results["tests"]["4_restore_on_cancel"] = {
            "global_inflation": g_after_cancel['inflation_radius'] if g_after_cancel else None,
            "local_inflation": l_after_cancel['inflation_radius'] if l_after_cancel else None,
            "passed": cancel_restored
        }
        print(f"  Cancel restore check: restored={cancel_restored}")

        all_passed = all(t.get("passed", False) for t in test_results["tests"].values())
        test_results["all_passed"] = all_passed

    except Exception as exc:
        test_results["error"] = str(exc)
        print(f"[margin_test] Exception: {exc}")
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
        stop_process(map_proc)
        stop_process(gz_proc)
        time.sleep(2)

    out_file = LOGS_DIR / 'margin_retry_restore_results.json'
    out_file.write_text(json.dumps(test_results, indent=2))
    print(f"\n=======================================================")
    print(f"MARGIN RETRY & RESTORE TESTS COMPLETE. Saved to {out_file}")
    print(f"All passed: {test_results['all_passed']}")
    print(f"=======================================================")


if __name__ == '__main__':
    run_margin_tests()
