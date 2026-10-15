// adhere_mujoco_node: single-process host for AdhereMujocoHardware (500Hz) + the standard
// ros2_control ResourceManager/ControllerManager plumbing, plus a 100Hz TF/Odometry/
// JointState/MarkerArray publisher so RViz2 mirrors the embedded MuJoCo viewer in real time.
// With no controller spawned, ControllerManager::update() is a no-op every tick and
// hw_commands_ stays at zero -- this is exactly the "u=0 passive drift" verification harness.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
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
constexpr std::array<std::array<double, 3>, 4> kPadPosBody = {
  std::array<double, 3>{-0.1, 0.25, 0.15}, std::array<double, 3>{-0.1, -0.25, 0.15},
  std::array<double, 3>{-0.1, 0.25, -0.15}, std::array<double, 3>{-0.1, -0.25, -0.15}};

// Snapshot of the state this node needs for TF/Odom/Marker publishing, read from
// AdhereMujocoHardware::visBridge() under its mutex (CLAUDE.md Phase 2.6 Stage 2.1 bug #8: no
// claim_state_interface() calls here -- that API is not meant to be raced against the
// ResourceManager-owning controller_manager from a second thread).
struct TelemetrySnapshot
{
  double pos[3] = {0, 0, 0};
  double quat[4] = {1, 0, 0, 0};  // w, x, y, z
  double vel_body[3] = {0, 0, 0};
  double gyro_body[3] = {0, 0, 0};
  std::array<double, 4> pad_force{};
  std::array<double, 3> flow_vel_world{};
  bool valid = false;
};

TelemetrySnapshot readTelemetry(adhere_mujoco_hardware::AdhereMujocoHardware * hw)
{
  TelemetrySnapshot snap;
  auto bridge = hw->visBridge();
  if (!bridge) {
    return snap;
  }
  std::lock_guard<std::mutex> lock(bridge->mtx);
  if (!bridge->valid || bridge->qpos.size() < 7 || bridge->qvel.size() < 6) {
    return snap;
  }
  snap.pos[0] = bridge->qpos[0];
  snap.pos[1] = bridge->qpos[1];
  snap.pos[2] = bridge->qpos[2];
  snap.quat[0] = bridge->qpos[3];
  snap.quat[1] = bridge->qpos[4];
  snap.quat[2] = bridge->qpos[5];
  snap.quat[3] = bridge->qpos[6];

  // MuJoCo freejoint qvel convention: first 3 = world-frame linear velocity, last 3 =
  // body-frame angular velocity. Rotate linear velocity into the body frame with R^T.
  const double w = snap.quat[0], x = snap.quat[1], y = snap.quat[2], z = snap.quat[3];
  double R[9] = {
    1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)};
  const double vx = bridge->qvel[0], vy = bridge->qvel[1], vz = bridge->qvel[2];
  snap.vel_body[0] = R[0] * vx + R[3] * vy + R[6] * vz;
  snap.vel_body[1] = R[1] * vx + R[4] * vy + R[7] * vz;
  snap.vel_body[2] = R[2] * vx + R[5] * vy + R[8] * vz;
  snap.gyro_body[0] = bridge->qvel[3];
  snap.gyro_body[1] = bridge->qvel[4];
  snap.gyro_body[2] = bridge->qvel[5];
  snap.pad_force = bridge->pad_force;
  snap.flow_vel_world = bridge->flow_vel_world;
  snap.valid = true;
  return snap;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("adhere_mujoco_node");
  node->declare_parameter<std::string>("robot_description", "");
  node->declare_parameter<bool>("headless", false);
  const std::string urdf = node->get_parameter("robot_description").as_string();
  const bool headless = node->get_parameter("headless").as_bool();
  if (urdf.empty()) {
    RCLCPP_FATAL(node->get_logger(), "robot_description parameter is empty, cannot start");
    rclcpp::shutdown();
    return 1;
  }

  // Constructing ResourceManager parses the URDF and runs AdhereMujocoHardware::configure(),
  // which registers itself via activeInstance() -- but defers MujocoViewer construction to
  // start() (triggered below by start_components()), since `headless` is no longer a
  // <ros2_control> hardware_parameters entry (CLAUDE.md Phase 2.6 pure-xacro decoupling).
  auto resource_manager = std::make_unique<hardware_interface::ResourceManager>(urdf);

  auto * hw = adhere_mujoco_hardware::AdhereMujocoHardware::activeInstance();
  if (!hw) {
    RCLCPP_FATAL(node->get_logger(), "AdhereMujocoHardware failed to configure");
    rclcpp::shutdown();
    return 1;
  }
  hw->setHeadless(headless);

  resource_manager->start_components();

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
      auto * active_hw = adhere_mujoco_hardware::AdhereMujocoHardware::activeInstance();
      if (active_hw) {
        active_hw->setWaveMode(msg->data);
      }
    });

  auto publish_timer = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / kPublishRateHz),
    [&, hw]() {
      const TelemetrySnapshot snap = readTelemetry(hw);
      if (!snap.valid) {
        return;
      }
      const rclcpp::Time now = node->now();

      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = now;
      tf_msg.header.frame_id = "world";
      tf_msg.child_frame_id = "base_link";
      tf_msg.transform.translation.x = snap.pos[0];
      tf_msg.transform.translation.y = snap.pos[1];
      tf_msg.transform.translation.z = snap.pos[2];
      tf_msg.transform.rotation.w = snap.quat[0];
      tf_msg.transform.rotation.x = snap.quat[1];
      tf_msg.transform.rotation.y = snap.quat[2];
      tf_msg.transform.rotation.z = snap.quat[3];
      tf_broadcaster->sendTransform(tf_msg);

      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = now;
      odom_msg.header.frame_id = "world";
      odom_msg.child_frame_id = "base_link";
      odom_msg.pose.pose.position.x = snap.pos[0];
      odom_msg.pose.pose.position.y = snap.pos[1];
      odom_msg.pose.pose.position.z = snap.pos[2];
      odom_msg.pose.pose.orientation.w = snap.quat[0];
      odom_msg.pose.pose.orientation.x = snap.quat[1];
      odom_msg.pose.pose.orientation.y = snap.quat[2];
      odom_msg.pose.pose.orientation.z = snap.quat[3];
      odom_msg.twist.twist.linear.x = snap.vel_body[0];
      odom_msg.twist.twist.linear.y = snap.vel_body[1];
      odom_msg.twist.twist.linear.z = snap.vel_body[2];
      odom_msg.twist.twist.angular.x = snap.gyro_body[0];
      odom_msg.twist.twist.angular.y = snap.gyro_body[1];
      odom_msg.twist.twist.angular.z = snap.gyro_body[2];
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

      for (size_t i = 0; i < snap.pad_force.size(); ++i) {
        visualization_msgs::msg::Marker bar;
        bar.header.stamp = now;
        bar.header.frame_id = "base_link";
        bar.ns = "pad_force";
        bar.id = static_cast<int>(i) + 1;
        bar.type = visualization_msgs::msg::Marker::ARROW;
        bar.action = visualization_msgs::msg::Marker::ADD;
        const double force = std::max(0.0, snap.pad_force[i]);
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

      // Ambient flow-velocity arrow (CLAUDE.md Phase 2.6 Stage 2.1 bug #7), world frame, from
      // the ROV's current position -- mirrors the arrow MujocoViewer draws from its own
      // d_vis_, so RViz2 and the embedded MuJoCo window always agree.
      visualization_msgs::msg::Marker flow_arrow;
      flow_arrow.header.stamp = now;
      flow_arrow.header.frame_id = "world";
      flow_arrow.ns = "flow_velocity";
      flow_arrow.id = 0;
      flow_arrow.type = visualization_msgs::msg::Marker::ARROW;
      flow_arrow.action = visualization_msgs::msg::Marker::ADD;
      geometry_msgs::msg::Point fp0;
      fp0.x = snap.pos[0];
      fp0.y = snap.pos[1];
      fp0.z = snap.pos[2];
      geometry_msgs::msg::Point fp1;
      fp1.x = snap.pos[0] + snap.flow_vel_world[0];
      fp1.y = snap.pos[1] + snap.flow_vel_world[1];
      fp1.z = snap.pos[2] + snap.flow_vel_world[2];
      flow_arrow.points.push_back(fp0);
      flow_arrow.points.push_back(fp1);
      flow_arrow.scale.x = 0.02;
      flow_arrow.scale.y = 0.04;
      flow_arrow.color.r = 0.2f;
      flow_arrow.color.g = 0.6f;
      flow_arrow.color.b = 1.0f;
      flow_arrow.color.a = 1.0f;
      markers.markers.push_back(flow_arrow);

      marker_pub->publish(markers);
    });

  // Deterministic 500Hz clock (CLAUDE.md Phase 2.6 Stage 3.2): steady_clock::sleep_until
  // instead of rclcpp::Rate, so neither MujocoViewer (60Hz GL) nor RViz2 (100Hz topics) can
  // skew the physics tick period under load. actual_hz_ is an exponential moving average
  // written into the vis bridge every tick so the GUI can prove 500.0 Hz execution.
  std::atomic<bool> physics_running{true};
  std::thread physics_thread([&, hw]() {
    using Clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / kPhysicsUpdateRateHz));
    auto next_tick = Clock::now() + period;
    auto last_tick = Clock::now();
    double actual_hz_ema = kPhysicsUpdateRateHz;
    auto bridge = hw->visBridge();

    while (rclcpp::ok() && physics_running.load()) {
      controller_manager->read();
      controller_manager->update();
      controller_manager->write();

      const auto now = Clock::now();
      const double dt = std::chrono::duration<double>(now - last_tick).count();
      last_tick = now;
      if (dt > 0.0) {
        const double instant_hz = 1.0 / dt;
        actual_hz_ema = 0.98 * actual_hz_ema + 0.02 * instant_hz;
        if (bridge) {
          bridge->actual_physics_hz.store(actual_hz_ema);
        }
      }

      std::this_thread::sleep_until(next_tick);
      next_tick += period;
      if (next_tick < Clock::now()) {
        // Fell behind by more than one period (e.g. a debugger breakpoint): resync instead of
        // free-running catch-up ticks.
        next_tick = Clock::now() + period;
      }
    }
  });

  executor->spin();

  physics_running = false;
  physics_thread.join();
  rclcpp::shutdown();
  return 0;
}
