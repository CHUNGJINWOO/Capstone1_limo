/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <explore_relaxed/costmap_client.h>
#include <explore_relaxed/frontier_search.h>
#include <std_msgs/msg/string.hpp>
#include <map>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <chrono>
#include <cmath>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;
#ifdef ELOQUENT
#define ACTION_NAME "NavigateToPose"
#elif DASHING
#define ACTION_NAME "NavigateToPose"
#else
#define ACTION_NAME "navigate_to_pose"
#endif
namespace explore_relaxed
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class Explore : public rclcpp::Node
{
public:
  Explore();
  ~Explore();

  void start();
  void stop(bool finished_exploring = false);
  void resume();

  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

private:
  /**
   * @brief  Make a global plan
   */
  void makePlan();

  // /**
  //  * @brief  Publish a frontiers as markers
  //  */
  void visualizeFrontiers(
      const std::vector<frontier_exploration_relaxed::Frontier>& frontiers);

  bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);

  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  // void
  // goal_response_callback(std::shared_future<NavigationGoalHandle::SharedPtr>
  // future);
  void reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                   const geometry_msgs::msg::Point& frontier_goal);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_publisher_;

  /**
    * @brief Publisher for exploration status updates (see ExploreStatus.msg for status values)
    */
  rclcpp::Publisher<explore_lite_msgs::msg::ExploreStatus>::SharedPtr status_pub_;

  rclcpp::Logger logger_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      move_base_client_;
  frontier_exploration_relaxed::FrontierSearch search_;
  rclcpp::TimerBase::SharedPtr exploring_timer_;
  // rclcpp::TimerBase::SharedPtr oneshot_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
  void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg);

  std::vector<geometry_msgs::msg::Point> frontier_blacklist_;
  geometry_msgs::msg::Point prev_goal_;
  double prev_distance_;
  rclcpp::Time last_progress_;
  size_t last_markers_count_;

  geometry_msgs::msg::Pose initial_pose_;
  void returnToInitialPose(void);

  // parameters
  double planner_frequency_;
  double potential_scale_, orientation_scale_, gain_scale_;
  double progress_timeout_;
  bool visualize_;
  bool return_to_init_;
  std::string robot_base_frame_;
  bool resuming_ = false;
  bool goal_active_{false};
  rclcpp_action::GoalUUID active_goal_id_;

  // ---- explore_relaxed additions (defaults reproduce explore_lite) ----
  int bfs_max_traverse_cost_{0};          // OccupancyGrid units 0..98; 0 = original BFS rule
  bool check_reachability_{false};        // separate drivability check on the discovered candidates
  int reachability_max_candidates_{10};   // how many best-cost candidates to test per cycle
  int unreachable_blacklist_after_{3};    // consecutive unreachable observations before a candidate is blacklisted
  int no_reachable_patience_{5};          // cycles without any reachable candidate before stopping (like "tried out")
  bool dry_run_{false};                   // discover/select/publish only; never send a goal, never stop
  int no_reachable_cycles_{0};
  // Drivability according to Nav2 itself (read-only ComputePathToPose; never sends a navigation goal)
  std::string reachability_mode_{"costmap"};   // costmap | nav2 | both
  std::string nav2_planner_id_{"GridBased"};
  int nav2_check_timeout_ms_{1500};
  int nav2_check_max_per_cycle_{3};
  rclcpp::CallbackGroup::SharedPtr nav2_check_group_;
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr compute_client_;
  bool nav2Reachable(const geometry_msgs::msg::Point& goal);
  bool isReachable(const geometry_msgs::msg::Point& robot, const geometry_msgs::msg::Point& goal);
  std::map<std::pair<int, int>, int> unreachable_ledger_;   // 0.25 m grid key -> consecutive unreachable count
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidates_pub_;
  void publishCandidates(size_t found, size_t blacklisted, size_t checked, size_t reachable,
                         const geometry_msgs::msg::Point* chosen,
                         const std::vector<frontier_exploration_relaxed::Frontier>* list = nullptr,
                         const geometry_msgs::msg::Point* robot = nullptr);
};
}  // namespace explore_relaxed

#endif
