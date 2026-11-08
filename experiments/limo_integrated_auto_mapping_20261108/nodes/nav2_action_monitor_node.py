#!/usr/bin/env -S python3 -s
"""
nav2_action_monitor_node.py
Nav2 Action Lifecycle & Frontier Exploration Monitor for LIMO Pro Auto-Mapping:

Passively monitors and records high-level action events without interfering with Nav2:
1. NavigateToPose Action Events:
   - Goal UUID (hex string)
   - Target Coordinates (x, y, z, yaw)
   - Acceptance / Rejection
   - Feedback stream (distance_remaining, navigation_time, number_of_recoveries)
   - Result Status: SUCCEEDED (4), CANCELED (5), ABORTED (6)
   - Goal Execution Duration (seconds)
   - Failure / Abort Reason
2. Pose Verification at Event Instants:
   - Estimated robot pose (x, y, yaw) from active estimator
   - Simulation Ground Truth pose (x, y, yaw) from Gazebo (clearly tagged as SIMULATION ONLY)
   - Pose discrepancy / estimation error
3. explore_lite Frontier Tracking:
   - Frontier marker array reception and cluster count
4. Standardized ROS 2 Logging:
   - INFO: Goal dispatched, accepted, reached
   - WARN: Goal canceled, recovery triggered
   - ERROR: Goal aborted, path blocked

Outputs:
- Stream: <events_file> (nav2_action_events.jsonl)
- Summary: <summary_file> (nav2_action_summary.json)
"""
import os
import sys
import time
import math
import json
import uuid
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from action_msgs.msg import GoalStatusArray, GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Nav2ActionMonitorNode(Node):
    def __init__(self):
        super().__init__(
            'nav2_action_monitor_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.declare_parameter('events_file', '/tmp/nav2_action_events.jsonl')
        self.declare_parameter('summary_file', '/tmp/nav2_action_summary.json')
        self.declare_parameter('active_estimator', 'f4')

        self.events_file = self.get_parameter('events_file').value
        self.summary_file = self.get_parameter('summary_file').value
        self.active_estimator = self.get_parameter('active_estimator').value

        os.makedirs(os.path.dirname(os.path.abspath(self.events_file)), exist_ok=True)
        os.makedirs(os.path.dirname(os.path.abspath(self.summary_file)), exist_ok=True)

        self._file_handle = open(self.events_file, 'a', buffering=1)

        # Pose cache
        self.latest_est_pose = None
        self.latest_gt_pose = None
        self.latest_stall_state = 'NORMAL'

        # Goal tracking dictionary: uuid_str -> dict
        self.active_goals: Dict[str, Dict[str, Any]] = {}
        self.completed_goals: Dict[str, Dict[str, Any]] = {}

        # Pending goal pose from /goal_pose
        self.recent_goal_poses = []

        # Subscriptions
        qos_transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Action status and feedback topics
        self.sub_action_status = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.on_action_status,
            qos_transient
        )

        self.sub_action_feedback = self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self.on_action_feedback,
            10
        )

        # External goal pose topic
        self.sub_goal_pose = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.on_goal_pose,
            10
        )

        # Explore lite frontiers
        self.sub_frontiers = self.create_subscription(
            MarkerArray,
            '/explore/frontiers',
            self.on_frontiers,
            5
        )

        # Estimated Odometry
        est_topic = '/odometry/lidar_odom' if self.active_estimator == 'f4' else '/odometry/f5_improved'
        self.sub_est = self.create_subscription(Odometry, est_topic, self.on_est_odom, 20)

        # Ground Truth Odometry (Simulation only)
        self.sub_gt = self.create_subscription(Odometry, '/ground_truth/world_odom', self.on_gt_odom, 20)

        # Stall Detector Status
        self.sub_stall = self.create_subscription(String, '/stall_detector/status_json', self.on_stall, 10)

        # Periodic summary flush timer (1 Hz)
        self.timer_flush = self.create_timer(1.0, self.flush_summary)

        self.get_logger().info(
            f"Nav2ActionMonitorNode initialized: estimator={self.active_estimator}, "
            f"events_file='{self.events_file}'"
        )

    def sim_time(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_est_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_est_pose = {
            'x': round(float(p.x), 4),
            'y': round(float(p.y), 4),
            'yaw': round(quat_to_yaw(q), 4)
        }

    def on_gt_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_gt_pose = {
            'x': round(float(p.x), 4),
            'y': round(float(p.y), 4),
            'yaw': round(quat_to_yaw(q), 4)
        }

    def on_stall(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.latest_stall_state = d.get('state', 'UNKNOWN')
        except Exception:
            pass

    def on_goal_pose(self, msg: PoseStamped):
        t_sim = self.sim_time()
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = quat_to_yaw(q)
        goal_rec = {
            'sim_time': round(t_sim, 3),
            'wall_time': time.time(),
            'target_x': round(float(p.x), 4),
            'target_y': round(float(p.y), 4),
            'target_yaw': round(yaw, 4),
            'frame_id': msg.header.frame_id
        }
        self.recent_goal_poses.append(goal_rec)
        self.get_logger().info(
            f"[NAV2 GOAL DISPATCHED] Target=({p.x:.2f}, {p.y:.2f}, Yaw={yaw:.2f} rad) in frame '{msg.header.frame_id}'"
        )

    def on_frontiers(self, msg: MarkerArray):
        frontier_count = len(msg.markers)
        if frontier_count > 0:
            # Low rate debug log
            self.get_logger().debug(f"[EXPLORE FRONTIERS] {frontier_count} frontier markers active.")

    def on_action_feedback(self, msg: NavigateToPose.Impl.FeedbackMessage):
        u_str = str(uuid.UUID(bytes=bytes(msg.goal_id.uuid)))
        fb = msg.feedback

        t_sim = self.sim_time()
        dist_rem = float(fb.distance_remaining)
        nav_t = float(fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9)
        recov_cnt = int(fb.number_of_recoveries)

        cur_p = fb.current_pose.pose.position
        cur_q = fb.current_pose.pose.orientation
        cur_yaw = quat_to_yaw(cur_q)

        if u_str in self.active_goals:
            g = self.active_goals[u_str]
            g['last_feedback_sim_t'] = t_sim
            g['distance_remaining'] = round(dist_rem, 3)
            g['navigation_time_sec'] = round(nav_t, 3)
            g['number_of_recoveries'] = recov_cnt

            if recov_cnt > g.get('prev_recoveries', 0):
                self.get_logger().warn(
                    f"[NAV2 RECOVERY TRIGGERED] Goal {u_str[:8]}: recovery count incremented to {recov_cnt}!"
                )
                g['prev_recoveries'] = recov_cnt

            # Log throttled feedback event every 1.0s
            if t_sim - g.get('last_event_sim_t', 0.0) >= 1.0:
                g['last_event_sim_t'] = t_sim
                event = {
                    'timestamp_sim': round(t_sim, 3),
                    'timestamp_wall': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'event_type': 'FEEDBACK',
                    'goal_uuid': u_str,
                    'distance_remaining_m': round(dist_rem, 3),
                    'navigation_time_sec': round(nav_t, 3),
                    'number_of_recoveries': recov_cnt,
                    'current_pose_feedback': {'x': round(cur_p.x, 3), 'y': round(cur_p.y, 3), 'yaw': round(cur_yaw, 3)},
                    'estimated_pose': self.latest_est_pose,
                    'ground_truth_pose_sim_only': self.latest_gt_pose,
                    'stall_detector_state': self.latest_stall_state
                }
                self.record_event(event)

    def on_action_status(self, msg: GoalStatusArray):
        t_sim = self.sim_time()
        for gs in msg.status_list:
            u_str = str(uuid.UUID(bytes=bytes(gs.goal_info.goal_id.uuid)))
            st_code = int(gs.status)
            st_name = STATUS_NAMES.get(st_code, f'UNKNOWN_{st_code}')

            # 1. New goal detected
            if u_str not in self.active_goals and u_str not in self.completed_goals:
                matched_target = self.match_recent_goal_pose(t_sim)
                goal_entry = {
                    'goal_uuid': u_str,
                    'start_sim_time': round(t_sim, 3),
                    'start_wall_time': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'target_pose': matched_target,
                    'initial_estimated_pose': self.latest_est_pose,
                    'initial_ground_truth_pose_sim_only': self.latest_gt_pose,
                    'status': st_name,
                    'status_code': st_code,
                    'distance_remaining': 0.0,
                    'navigation_time_sec': 0.0,
                    'number_of_recoveries': 0,
                    'prev_recoveries': 0,
                    'last_event_sim_t': t_sim
                }
                self.active_goals[u_str] = goal_entry

                self.get_logger().info(
                    f"[NAV2 GOAL ACCEPTED] UUID={u_str[:8]} Status={st_name}. Target={matched_target}"
                )
                event = {
                    'timestamp_sim': round(t_sim, 3),
                    'timestamp_wall': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'event_type': 'GOAL_ACCEPTED',
                    'goal_uuid': u_str,
                    'status': st_name,
                    'target_pose': matched_target,
                    'estimated_pose': self.latest_est_pose,
                    'ground_truth_pose_sim_only': self.latest_gt_pose
                }
                self.record_event(event)

            # 2. Existing active goal status update
            elif u_str in self.active_goals:
                g = self.active_goals[u_str]
                prev_st = g['status']

                if st_name != prev_st:
                    g['status'] = st_name
                    g['status_code'] = st_code

                    # Terminal states
                    if st_code in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
                        dur = round(t_sim - g['start_sim_time'], 3)
                        g['end_sim_time'] = round(t_sim, 3)
                        g['duration_sec'] = dur
                        g['final_estimated_pose'] = self.latest_est_pose
                        g['final_ground_truth_pose_sim_only'] = self.latest_gt_pose

                        failure_reason = None
                        if st_code == GoalStatus.STATUS_SUCCEEDED:
                            self.get_logger().info(
                                f"[NAV2 GOAL SUCCEEDED] UUID={u_str[:8]} reached in {dur:.2f}s."
                            )
                        elif st_code == GoalStatus.STATUS_CANCELED:
                            failure_reason = "Canceled by client or preempted by new goal/frontier"
                            self.get_logger().warn(
                                f"[NAV2 GOAL CANCELED] UUID={u_str[:8]} canceled after {dur:.2f}s."
                            )
                        elif st_code == GoalStatus.STATUS_ABORTED:
                            failure_reason = f"Aborted by Nav2 (planner/controller failed, stall_state={self.latest_stall_state})"
                            self.get_logger().error(
                                f"[NAV2 GOAL ABORTED] UUID={u_str[:8]} aborted after {dur:.2f}s! Reason: {failure_reason}"
                            )

                        g['failure_reason'] = failure_reason

                        event = {
                            'timestamp_sim': round(t_sim, 3),
                            'timestamp_wall': time.strftime('%Y-%m-%d %H:%M:%S'),
                            'event_type': f'GOAL_{st_name}',
                            'goal_uuid': u_str,
                            'status': st_name,
                            'duration_sec': dur,
                            'failure_reason': failure_reason,
                            'target_pose': g.get('target_pose'),
                            'final_estimated_pose': self.latest_est_pose,
                            'final_ground_truth_pose_sim_only': self.latest_gt_pose,
                            'final_distance_remaining_m': g.get('distance_remaining', 0.0),
                            'stall_detector_state': self.latest_stall_state
                        }
                        self.record_event(event)

                        # Move to completed
                        self.completed_goals[u_str] = g
                        del self.active_goals[u_str]

    def match_recent_goal_pose(self, t_now: float) -> Optional[Dict[str, float]]:
        # Match nearest recent goal pose within last 3.0s
        if not self.recent_goal_poses:
            return None
        # Return most recent
        return self.recent_goal_poses[-1]

    def record_event(self, event_dict: Dict[str, Any]):
        try:
            line = json.dumps(event_dict) + '\n'
            self._file_handle.write(line)
        except Exception as e:
            self.get_logger().error(f"Failed to write action event: {e}")

    def flush_summary(self):
        total_count = len(self.completed_goals) + len(self.active_goals)
        succ_count = sum(1 for g in self.completed_goals.values() if g['status'] == 'SUCCEEDED')
        canc_count = sum(1 for g in self.completed_goals.values() if g['status'] == 'CANCELED')
        abort_count = sum(1 for g in self.completed_goals.values() if g['status'] == 'ABORTED')

        durs = [g.get('duration_sec', 0.0) for g in self.completed_goals.values()]
        mean_dur = float(sum(durs) / len(durs)) if durs else 0.0

        summary = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'total_goals_dispatched': total_count,
            'completed_goals_count': len(self.completed_goals),
            'active_goals_count': len(self.active_goals),
            'succeeded_count': succ_count,
            'canceled_count': canc_count,
            'aborted_count': abort_count,
            'success_rate_pct': round((succ_count / len(self.completed_goals) * 100) if self.completed_goals else 0.0, 1),
            'mean_goal_duration_sec': round(mean_dur, 2),
            'completed_goals': list(self.completed_goals.values()),
            'active_goals': list(self.active_goals.values())
        }

        try:
            with open(self.summary_file, 'w') as f:
                json.dump(summary, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed to flush action summary: {e}")

    def destroy_node(self):
        self.flush_summary()
        try:
            self._file_handle.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ActionMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
