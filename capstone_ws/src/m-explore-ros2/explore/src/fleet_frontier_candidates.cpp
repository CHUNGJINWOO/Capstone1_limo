// SIM evaluation adapter: reuse explore_lite frontier scoring without owning
// a Nav2 action client. Publishing candidates never authorizes robot motion.
#include <chrono>
#include <memory>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "explore/costmap_client.h"
#include "explore/frontier_search.h"

class FleetFrontierCandidates : public rclcpp::Node {
 public:
  FleetFrontierCandidates() : Node("fleet_frontier_candidates"), tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_), costmap_(*this, &tf_buffer_) {
    const auto potential = declare_parameter<double>("potential_scale", 3.0);
    const auto gain = declare_parameter<double>("gain_scale", 1.0);
    const auto min_size = declare_parameter<double>("min_frontier_size", 0.35);
    search_ = std::make_unique<frontier_exploration::FrontierSearch>(
        costmap_.getCostmap(), potential, gain, min_size, get_logger());
    output_ = create_publisher<geometry_msgs::msg::PoseArray>("/fleet/frontier_candidates", 10);
    timer_ = create_wall_timer(std::chrono::seconds(2), [this] { publish_candidates(); });
  }
 private:
  void publish_candidates() {
    if (!tf_buffer_.canTransform(costmap_.getGlobalFrameID(), costmap_.getBaseFrameID(), tf2::TimePointZero)) {
      RCLCPP_WARN(get_logger(), "frontier candidates withheld: robot transform unavailable");
      return;
    }
    const auto pose = costmap_.getRobotPose();
    const auto frontiers = search_->searchFrom(pose.position);
    geometry_msgs::msg::PoseArray out;
    out.header.frame_id = costmap_.getGlobalFrameID(); out.header.stamp = now();
    for (const auto &f : frontiers) {
      geometry_msgs::msg::Pose candidate; candidate.position = f.middle;
      candidate.orientation.w = 1.0; out.poses.push_back(candidate);
    }
    output_->publish(out);
    RCLCPP_INFO(get_logger(), "frontier candidates=%zu", out.poses.size());
  }
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  explore::Costmap2DClient costmap_;
  std::unique_ptr<frontier_exploration::FrontierSearch> search_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr output_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FleetFrontierCandidates>());
  rclcpp::shutdown();
}
