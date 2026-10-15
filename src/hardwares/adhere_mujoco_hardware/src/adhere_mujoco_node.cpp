// adhere_mujoco_node: single-process host for AdhereMujocoHardware (500Hz) + the standard
// ros2_control ResourceManager/ControllerManager plumbing, plus a 100Hz TF/Odometry/
// JointState/MarkerArray publisher so RViz2 mirrors the embedded MuJoCo viewer in real time.
// With no controller spawned, ControllerManager::update() is a no-op every tick and
// hw_commands_ stays at zero -- this is exactly the "u=0 passive drift" verification harness.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"

#include "adhere_mujoco_hardware/adhere_mujoco_hardware.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
constexpr double kPhysicsUpdateRateHz = 500.0;
constexpr double kPublishRateHz = 100.0;
constexpr std::array<const char *, 4> kPadStateKeys = {
  "pad_contact/pad_TL/force", "pad_contact/pad_TR/force", "pad_contact/pad_BL/force",
  "pad_contact/pad_BR/force"};
constexpr std::array<std::array<double, 3>, 4> kPadPosBody = {
  std::array<double, 3>{-0.1, 0.25, 0.15}, std::array<double, 3>{-0.1, -0.25, 0.15},
  std::array<double, 3>{-0.1, 0.25, -0.15}, std::array<double, 3>{-0.1, -0.25, -0.15}};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("adhere_mujoco_node");
  node->declare_parameter<std::string>("robot_description", "");
  const std::string urdf = node->get_parameter("robot_description").as_string();
  if (urdf.empty()) {
    RCLCPP_FATAL(node->get_logger(), "robot_description parameter is empty, cannot start");
    rclcpp::shutdown();
    return 1;
  }

  auto resource_manager = std::make_unique<hardware_interface::ResourceManager>(urdf);
  resource_manager->start_components();

  // Claim the state interfaces this node republishes to RViz2 before transferring ownership
  // to the ControllerManager -- the underlying StateInterface objects outlive this move since
  // they live in heap-allocated resource storage, not in the ResourceManager object itself.
  auto pos_x = resource_manager->claim_state_interface("rov_odom/pos.x");
  auto pos_y = resource_manager->claim_state_interface("rov_odom/pos.y");
  auto pos_z = resource_manager->claim_state_interface("rov_odom/pos.z");
  auto quat_w = resource_manager->claim_state_interface("rov_odom/quat.w");
  auto quat_x = resource_manager->claim_state_interface("rov_odom/quat.x");
  auto quat_y = resource_manager->claim_state_interface("rov_odom/quat.y");
  auto quat_z = resource_manager->claim_state_interface("rov_odom/quat.z");
  auto vel_x = resource_manager->claim_state_interface("rov_odom/vel_body.x");
  auto vel_y = resource_manager->claim_state_interface("rov_odom/vel_body.y");
  auto vel_z = resource_manager->claim_state_interface("rov_odom/vel_body.z");
  auto gyro_x = resource_manager->claim_state_interface("imu_sensor/angular_velocity.x");
  auto gyro_y = resource_manager->claim_state_interface("imu_sensor/angular_velocity.y");
  auto gyro_z = resource_manager->claim_state_interface("imu_sensor/angular_velocity.z");
  std::vector<hardware_interface::LoanedStateInterface> pad_force;
  for (const auto * key : kPadStateKeys) {
    pad_force.push_back(resource_manager->claim_state_interface(key));
  }

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto controller_manager = std::make_shared<controller_manager::ControllerManager>(
    std::move(resource_manager), executor, "controller_manager");
  executor->add_node(controller_manager);
  executor->add_node(node);

  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/rov/odom", 10);
  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/rov/environment_markers", 10);

  // Runtime wave-maker switch (CLAUDE.md Phase 2.5 Stage 2.3): wave_console_node publishes
  // 1 (calm) / 2 (gust) here; forwarded straight to the one live AdhereMujocoHardware instance
  // via its single-instance back-pointer (see adhere_mujoco_hardware.cpp).
  auto wave_mode_sub = node->create_subscription<std_msgs::msg::Int32>(
    "/rov/wave_mode_cmd", 10,
    [](const std_msgs::msg::Int32::SharedPtr msg) {
      auto * hw = adhere_mujoco_hardware::AdhereMujocoHardware::activeInstance();
      if (hw) {
        hw->setWaveMode(msg->data);
      }
    });

  auto publish_timer = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / kPublishRateHz),
    [&]() {
      const rclcpp::Time now = node->now();

      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = now;
      tf_msg.header.frame_id = "world";
      tf_msg.child_frame_id = "base_link";
      tf_msg.transform.translation.x = pos_x.get_value();
      tf_msg.transform.translation.y = pos_y.get_value();
      tf_msg.transform.translation.z = pos_z.get_value();
      tf_msg.transform.rotation.w = quat_w.get_value();
      tf_msg.transform.rotation.x = quat_x.get_value();
      tf_msg.transform.rotation.y = quat_y.get_value();
      tf_msg.transform.rotation.z = quat_z.get_value();
      tf_broadcaster->sendTransform(tf_msg);

      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = now;
      odom_msg.header.frame_id = "world";
      odom_msg.child_frame_id = "base_link";
      odom_msg.pose.pose.position.x = pos_x.get_value();
      odom_msg.pose.pose.position.y = pos_y.get_value();
      odom_msg.pose.pose.position.z = pos_z.get_value();
      odom_msg.pose.pose.orientation.w = quat_w.get_value();
      odom_msg.pose.pose.orientation.x = quat_x.get_value();
      odom_msg.pose.pose.orientation.y = quat_y.get_value();
      odom_msg.pose.pose.orientation.z = quat_z.get_value();
      odom_msg.twist.twist.linear.x = vel_x.get_value();
      odom_msg.twist.twist.linear.y = vel_y.get_value();
      odom_msg.twist.twist.linear.z = vel_z.get_value();
      odom_msg.twist.twist.angular.x = gyro_x.get_value();
      odom_msg.twist.twist.angular.y = gyro_y.get_value();
      odom_msg.twist.twist.angular.z = gyro_z.get_value();
      odom_pub->publish(odom_msg);

      // No articulated URDF joints exist on this model (see simplified_v1_description's
      // README), so the published JointState is intentionally empty.
      sensor_msgs::msg::JointState joint_state_msg;
      joint_state_msg.header.stamp = now;
      joint_state_pub->publish(joint_state_msg);

      visualization_msgs::msg::MarkerArray markers;

      visualization_msgs::msg::Marker wall;
      wall.header.stamp = now;
      wall.header.frame_id = "world";
      wall.ns = "environment";
      wall.id = 0;
      wall.type = visualization_msgs::msg::Marker::CUBE;
      wall.action = visualization_msgs::msg::Marker::ADD;
      wall.pose.position.x = -0.01;
      wall.scale.x = 0.02;
      wall.scale.y = 4.0;
      wall.scale.z = 4.0;
      wall.color.r = 0.25f;
      wall.color.g = 0.25f;
      wall.color.b = 0.3f;
      wall.color.a = 0.6f;
      markers.markers.push_back(wall);

      for (size_t i = 0; i < pad_force.size(); ++i) {
        visualization_msgs::msg::Marker bar;
        bar.header.stamp = now;
        bar.header.frame_id = "base_link";
        bar.ns = "pad_force";
        bar.id = static_cast<int>(i) + 1;
        bar.type = visualization_msgs::msg::Marker::ARROW;
        bar.action = visualization_msgs::msg::Marker::ADD;
        const double force = std::max(0.0, pad_force[i].get_value());
        const double length = 0.01 * force;  // 0.01 m per N
        geometry_msgs::msg::Point p0;
        p0.x = kPadPosBody[i][0];
        p0.y = kPadPosBody[i][1];
        p0.z = kPadPosBody[i][2];
        geometry_msgs::msg::Point p1 = p0;
        p1.x += length;
        bar.points.push_back(p0);
        bar.points.push_back(p1);
        bar.scale.x = 0.015;
        bar.scale.y = 0.03;
        bar.color.r = 0.1f;
        bar.color.g = 1.0f;
        bar.color.b = 0.1f;
        bar.color.a = 1.0f;
        markers.markers.push_back(bar);
      }

      marker_pub->publish(markers);
    });

  std::atomic<bool> physics_running{true};
  std::thread physics_thread([&]() {
    rclcpp::Rate rate(kPhysicsUpdateRateHz);
    while (rclcpp::ok() && physics_running.load()) {
      controller_manager->read();
      controller_manager->update();
      controller_manager->write();
      rate.sleep();
    }
  });

  executor->spin();

  physics_running = false;
  physics_thread.join();
  rclcpp::shutdown();
  return 0;
}
