import os
import sys
import time
import json
import subprocess
import signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from rcl_interfaces.srv import GetParameters

def clean_processes():
    subprocess.run(['killall', '-9', 'ign', 'ruby', 'gzserver', 'gzclient', 'ros2', 
                    'mapping_server', 'frontier_explorer', 'async_slam_toolbox_node', 
                    'controller_server', 'planner_server', 'bt_navigator', 'behavior_server'], 
                   stderr=subprocess.DEVNULL)
    time.sleep(2)

class SystemComparator(Node):
    def __init__(self):
        super().__init__('system_comparator')
        self.scan_msgs = []
        self.odom_msgs = []
        self.imu_msgs = []
        self.clock_msgs = []
        self.gcost_msgs = []
        self.lcost_msgs = []
        self.tf_msgs = []

        self.create_subscription(LaserScan, '/scan', lambda m: self.scan_msgs.append((time.time(), m)), 50)
        self.create_subscription(Odometry, '/odom', lambda m: self.odom_msgs.append((time.time(), m)), 50)
        self.create_subscription(Imu, '/imu', lambda m: self.imu_msgs.append((time.time(), m)), 50)
        self.create_subscription(Clock, '/clock', lambda m: self.clock_msgs.append((time.time(), m.clock.sec + m.clock.nanosec*1e-9)), 100)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: self.gcost_msgs.append((time.time(), m)), 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: self.lcost_msgs.append((time.time(), m)), 10)
        self.create_subscription(TFMessage, '/tf', lambda m: self.tf_msgs.append((time.time(), m)), 100)

    def get_params_from(self, node_name, param_names):
        cli = self.create_client(GetParameters, f'{node_name}/get_parameters')
        if not cli.wait_for_service(timeout_sec=5.0):
            return {p: 'SERVICE_UNAVAILABLE' for p in param_names}
        req = GetParameters.Request()
        req.names = param_names
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if not res:
            return {p: 'CALL_FAILED' for p in param_names}
        out = {}
        for name, pval in zip(param_names, res.values):
            # pval is ParameterValue
            if pval.type == 1: # bool
                out[name] = pval.bool_value
            elif pval.type == 2: # int
                out[name] = pval.integer_value
            elif pval.type == 3: # double
                out[name] = pval.double_value
            elif pval.type == 4: # string
                out[name] = pval.string_value
            elif pval.type == 7: # double_array
                out[name] = list(pval.double_array_value)
            elif pval.type == 8: # string_array
                out[name] = list(pval.string_array_value)
            else:
                out[name] = str(pval)
        return out

def main():
    clean_processes()
    ws_root = "/home/cjinw/school/2026-2/Capstone1_limo"
    exp_ws = f"{ws_root}/experiments/failure_limits_verification/ws"
    world_sdf = f"{ws_root}/experiments/gazebo_obstacle_mapping/worlds/c103_obstacle_exploration.sdf"
    model_sdf = f"{ws_root}/experiments/failure_limits_verification/models/limo_ackermann_fixed.sdf"
    out_json = f"{ws_root}/notebook/2026_11_05/05_실물시험대비_실패조건_한계검증/records/system_comparison_sim_measured.json"

    print("[1] Launching Gazebo...")
    gz_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {ws_root}/capstone_ws/install/setup.bash && "
        f"ros2 launch limo_capstone limo_mc3f_sim.launch.py "
        f"world:={world_sdf} model_file:={model_sdf} spawn_x:=-3.0 spawn_y:=-1.1 spawn_z:=0.05 spawn_yaw:=0.0 "
        f"headless:=true rviz:=false"
    )
    gz_proc = subprocess.Popen(["bash", "-c", gz_cmd], preexec_fn=os.setsid)
    time.sleep(10)

    print("[2] Launching Auto Mapping (slow profile)...")
    map_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {ws_root}/capstone_ws/install/setup.bash && "
        f"source {exp_ws}/install/setup.bash && "
        f"ros2 launch limo_auto_mapping auto_mapping.launch.py nav2_profile:=slow use_sim_time:=true"
    )
    map_proc = subprocess.Popen(["bash", "-c", map_cmd], preexec_fn=os.setsid)
    time.sleep(12)

    rclpy.init()
    measurer = SystemComparator()

    print("[3] Collecting sensor and topic streams for 10 seconds...")
    t0 = time.time()
    while time.time() - t0 < 10.0:
        rclpy.spin_once(measurer, timeout_sec=0.1)

    print("[4] Querying Nav2 / Costmap / Controller parameters...")
    ctrl_params = measurer.get_params_from('/controller_server', [
        'FollowPath.max_vel_x', 'FollowPath.max_vel_theta', 'FollowPath.max_speed_xy',
        'FollowPath.trans_stopped_velocity', 'general_goal_checker.xy_goal_tolerance',
        'general_goal_checker.yaw_goal_tolerance', 'progress_checker.required_movement_radius',
        'progress_checker.movement_time_allowance'
    ])
    beh_params = measurer.get_params_from('/behavior_server', [
        'max_rotational_vel', 'min_rotational_vel', 'cycle_frequency', 'simulate_ahead_time'
    ])
    smoother_params = measurer.get_params_from('/velocity_smoother', [
        'max_velocity', 'min_velocity', 'smoothing_frequency'
    ])
    gcost_params = measurer.get_params_from('/global_costmap/global_costmap', [
        'robot_radius', 'inflation_layer.inflation_radius', 'inflation_layer.cost_scaling_factor',
        'update_frequency', 'publish_frequency', 'resolution'
    ])
    lcost_params = measurer.get_params_from('/local_costmap/local_costmap', [
        'robot_radius', 'inflation_layer.inflation_radius', 'inflation_layer.cost_scaling_factor',
        'update_frequency', 'publish_frequency', 'resolution'
    ])

    # Compute frequencies
    def calc_freq(msg_list):
        if len(msg_list) < 2:
            return 0.0
        dt = msg_list[-1][0] - msg_list[0][0]
        return (len(msg_list) - 1) / dt if dt > 0 else 0.0

    # Compute RTF: sim_time_elapsed / wall_time_elapsed
    rtf = 0.0
    if len(measurer.clock_msgs) >= 2:
        wall_dt = measurer.clock_msgs[-1][0] - measurer.clock_msgs[0][0]
        sim_dt = measurer.clock_msgs[-1][1] - measurer.clock_msgs[0][1]
        rtf = sim_dt / wall_dt if wall_dt > 0 else 0.0

    # Extract scan specs
    scan_specs = {}
    if measurer.scan_msgs:
        s = measurer.scan_msgs[-1][1]
        scan_specs = {
            'angle_min_rad': s.angle_min,
            'angle_max_rad': s.angle_max,
            'angle_min_deg': round(s.angle_min * 180.0 / 3.1415926535, 2),
            'angle_max_deg': round(s.angle_max * 180.0 / 3.1415926535, 2),
            'range_min': s.range_min,
            'range_max': s.range_max,
            'angle_increment': s.angle_increment,
            'scan_time': s.scan_time,
            'num_beams': len(s.ranges),
            'header_frame_id': s.header.frame_id
        }

    results = {
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'measured_frequencies_hz': {
            'scan': round(calc_freq(measurer.scan_msgs), 2),
            'odom': round(calc_freq(measurer.odom_msgs), 2),
            'imu': round(calc_freq(measurer.imu_msgs), 2),
            'tf': round(calc_freq(measurer.tf_msgs), 2),
            'global_costmap': round(calc_freq(measurer.gcost_msgs), 2),
            'local_costmap': round(calc_freq(measurer.lcost_msgs), 2)
        },
        'sim_performance': {
            'measured_rtf': round(rtf, 3),
            'total_clock_samples': len(measurer.clock_msgs)
        },
        'lidar_scan_properties': scan_specs,
        'controller_server_parameters': ctrl_params,
        'behavior_server_parameters': beh_params,
        'velocity_smoother_parameters': smoother_params,
        'global_costmap_parameters': gcost_params,
        'local_costmap_parameters': lcost_params
    }

    os.makedirs(os.path.dirname(out_json), exist_ok=True)
    with open(out_json, 'w') as f:
        json.dump(results, f, indent=2)

    print("\n=== MEASUREMENT COMPLETE ===")
    print(json.dumps(results, indent=2))

    measurer.destroy_node()
    rclpy.shutdown()
    os.killpg(os.getpgid(map_proc.pid), signal.SIGINT)
    os.killpg(os.getpgid(gz_proc.pid), signal.SIGINT)
    time.sleep(2)
    clean_processes()

if __name__ == '__main__':
    main()
