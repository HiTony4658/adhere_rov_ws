// adhere_apriltag_detector_node: Sim-to-Real-identical wall-tag station detector (CLAUDE.md
// Phase 5.0 Stage 6). The simulated version here is a proximity-triggered point publisher, NOT
// a vision pipeline: it holds the same adhere_wall_tags_y/adhere_wall_tags_depth_m station table
// AdhereController's /rov/adhere_target_cmd TAG_PRESET resolves against, subscribes to the
// ROV's ESTIMATED pose on /rov/controller_state (never ground truth, to stay Sim-to-Real pure),
// and when within detection_range_m of a station's (Y, depth) centre point, publishes that
// station's TagDetection with a small configurable Gaussian offset + confidence -- exercising
// the "preset != actual sticker" case a real apriltag_ros node would also produce. The real
// robot's apriltag_ros node (tag36h11, 2x2 bundle per station -- see
// simplified_v1_description/README.md for the physical bundle layout) publishes the identical
// TagDetection message on the identical /rov/tag_detection topic, so AdhereController's consumer
// logic (applyResolvedAdhereTarget()) is unchanged between sim and real.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "adhere_control_msgs/msg/controller_state.hpp"
#include "adhere_control_msgs/msg/tag_detection.hpp"

namespace
{

struct Station
{
  double y;
  double depth_m;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("adhere_apriltag_detector_node");

  // Defaults mirror robot_control.yaml's adhere_wall_tags_y/adhere_wall_tags_depth_m (CLAUDE.md
  // Sec.9) -- this node reads its own copy of these parameters (rather than including
  // adhere_controller) so this package stays zero-dependency on MuJoCo/CUDA/ros2_control/
  // adhere_controller; a deployment that changes the station table must pass the same values to
  // both nodes (documented in this package's README).
  const auto wall_tags_y =
    node->declare_parameter<std::vector<double>>("wall_tags_y", {0.00, 0.80, -0.80, 1.20});
  const auto wall_tags_depth_m =
    node->declare_parameter<std::vector<double>>("wall_tags_depth_m", {1.00, 1.00, 1.50, 2.00});
  const double detection_range_m = node->declare_parameter<double>("detection_range_m", 0.5);
  const double position_noise_std_m =
    node->declare_parameter<double>("position_noise_std_m", 0.02);
  const double publish_rate_hz = node->declare_parameter<double>("publish_rate_hz", 5.0);

  std::vector<Station> stations;
  const size_t n = std::min(wall_tags_y.size(), wall_tags_depth_m.size());
  stations.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    stations.push_back({wall_tags_y[i], wall_tags_depth_m[i]});
  }

  std::mt19937 rng{std::random_device{}()};
  std::normal_distribution<double> noise(0.0, position_noise_std_m);

  std::mutex pose_mutex;
  bool have_pose = false;
  double latest_y = 0.0;
  double latest_depth_m = 0.0;

  auto controller_state_sub = node->create_subscription<adhere_control_msgs::msg::ControllerState>(
    "/rov/controller_state", 10,
    [&](const adhere_control_msgs::msg::ControllerState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(pose_mutex);
      latest_y = msg->estimated_pose.position.y;
      latest_depth_m = -msg->estimated_pose.position.z;
      have_pose = true;
    });

  auto tag_detection_pub =
    node->create_publisher<adhere_control_msgs::msg::TagDetection>("/rov/tag_detection", 10);

  const auto period =
    std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_hz));
  auto timer = node->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [&]() {
      bool pose_ready;
      double y, depth_m;
      {
        std::lock_guard<std::mutex> lock(pose_mutex);
        pose_ready = have_pose;
        y = latest_y;
        depth_m = latest_depth_m;
      }

      adhere_control_msgs::msg::TagDetection msg;
      msg.header.stamp = node->now();
      msg.valid = false;

      if (pose_ready && !stations.empty()) {
        size_t best_idx = 0;
        double best_dist = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < stations.size(); ++i) {
          const double dy = y - stations[i].y;
          const double dd = depth_m - stations[i].depth_m;
          const double dist = std::sqrt(dy * dy + dd * dd);
          if (dist < best_dist) {
            best_dist = dist;
            best_idx = i;
          }
        }
        if (best_dist <= detection_range_m) {
          msg.valid = true;
          msg.station_id = static_cast<uint8_t>(best_idx);
          msg.target_y = stations[best_idx].y + noise(rng);
          msg.target_depth_m = stations[best_idx].depth_m + noise(rng);
          msg.confidence =
            std::clamp(1.0 - best_dist / detection_range_m, 0.0, 1.0);
        }
      }

      tag_detection_pub->publish(msg);
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
