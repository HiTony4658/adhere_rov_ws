// adhere_mujoco_node: single-process host for AdhereMujocoHardware (500Hz) + the standard
// ros2_control ResourceManager/ControllerManager plumbing, plus a 100Hz TF/Odometry/
// JointState/MarkerArray publisher so RViz2 mirrors the embedded MuJoCo viewer in real time.
// With no controller spawned (the default), ControllerManager::update() is a no-op every tick
// and hw_commands_ stays at zero -- this is exactly the "u=0 passive drift" verification
// harness. This node has zero compile-time or runtime knowledge of any specific controller
// (CLAUDE.md Phase 3.1.1 Stage 2.2): the optional `autostart_controllers` string-array
// parameter (default empty) lets a separate closed-loop launch file bring up its own
// controller in-process, entirely from launch-time parameters, with no package-level
// coupling in either direction.
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
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "hardware_interface/resource_manager.hpp"

#include "adhere_control_msgs/msg/controller_state.hpp"
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
  // [TL,TR,BL,BR] body-local (x,y,z), read unlocked from bridge->pad_site_pos_body (CLAUDE.md
  // Phase 2.10 Section A3: single source of truth, resolved once in
  // AdhereMujocoHardware::configure() -- replaces this file's former hardcoded kPadPosBody).
  std::array<std::array<double, 3>, 4> pad_site_pos_body{};
  // CLAUDE.md Phase 5.0 Stage 4: environment marker geometry, write-once unlocked (same pattern
  // as pad_site_pos_body above), resolved once by AdhereMujocoHardware::configure() from
  // mjcf/rov_wall.xml's own wall_geom/floor_geom/water_surface geoms.
  std::array<double, 3> wall_pos_world{};
  std::array<double, 3> wall_size_full{};
  double floor_z = 0.0;
  std::array<double, 2> floor_extent_xy{};
  double water_surface_z = 0.0;
  std::array<double, 2> water_surface_extent_xy{};
  bool valid = false;
};

TelemetrySnapshot readTelemetry(adhere_mujoco_hardware::AdhereMujocoHardware * hw)
{
  TelemetrySnapshot snap;
  auto bridge = hw->visBridge();
  if (!bridge) {
    return snap;
  }
  {
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
  }
  // pad_site_pos_body is write-once unlocked geometry (CLAUDE.md Phase 2.10 Section A3): safe
  // to read outside bridge->mtx once snap.valid has been observed true.
  snap.pad_site_pos_body = bridge->pad_site_pos_body;
  snap.wall_pos_world = bridge->wall_pos_world;
  snap.wall_size_full = bridge->wall_size_full;
  snap.floor_z = bridge->floor_z;
  snap.floor_extent_xy = bridge->floor_extent_xy;
  snap.water_surface_z = bridge->water_surface_z;
  snap.water_surface_extent_xy = bridge->water_surface_extent_xy;
  return snap;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("adhere_mujoco_node");
  node->declare_parameter<std::string>("robot_description", "");
  node->declare_parameter<bool>("headless", false);
  // Zero-recompile model switching (CLAUDE.md Phase 2.12 Section 1): defaults reproduce the
  // exact pre-Phase-2.12 hardcoded path, so every existing launch invocation that passes
  // neither parameter is unaffected.
  node->declare_parameter<std::string>("pkg_description", "simplified_v1_description");
  node->declare_parameter<std::string>("mjcf_file", "mjcf/rov_wall.xml");
  // Generic, controller-agnostic autostart list (CLAUDE.md Phase 3.1.1 Stage 2.2): each entry is
  // a controller *name* only -- its plugin *type* must already be present in this process'
  // "controller_manager" parameter namespace (e.g. robot_control.yaml's
  // `controller_manager.ros__parameters.<name>.type`), which the single-arg
  // load_controller(name) overload below reads. Empty by default, so mujoco_sim.launch.py's
  // plain hardware-only launch starts exactly 0 controllers.
  node->declare_parameter<std::vector<std::string>>("autostart_controllers", {});
  const std::string urdf = node->get_parameter("robot_description").as_string();
  const bool headless = node->get_parameter("headless").as_bool();
  const std::string pkg_description = node->get_parameter("pkg_description").as_string();
  const std::string mjcf_file = node->get_parameter("mjcf_file").as_string();
  const std::vector<std::string> autostart_controllers =
    node->get_parameter("autostart_controllers").as_string_array();
  if (urdf.empty()) {
    RCLCPP_FATAL(node->get_logger(), "robot_description parameter is empty, cannot start");
    rclcpp::shutdown();
    return 1;
  }

  // Must run BEFORE the ResourceManager below is constructed: that constructor is what
  // pluginlib-instantiates AdhereMujocoHardware and synchronously calls its configure(), which
  // is where this override is consulted (CLAUDE.md Phase 2.12 Section 1 -- there is no
  // activeInstance() to reach yet at this point).
  adhere_mujoco_hardware::AdhereMujocoHardware::setModelSourceOverride(pkg_description, mjcf_file);

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

  // Generic programmatic load/configure (CLAUDE.md Phase 3.1.1 Stage 2.2): the single-arg
  // load_controller(name) overload reads each controller's plugin type from this process'
  // "controller_manager" parameter namespace (populated by whatever --params-file the launch
  // file passed, e.g. robot_control.yaml), so this node never names a specific controller type.
  // Each plugin library is discovered via pluginlib's ament-index plugin manifest at runtime,
  // not by linking against it here. switch_controller() itself is deferred until AFTER the
  // physics thread starts (below): Foxy's ControllerManager::switch_controller() busy-waits for
  // the pending switch to be APPLIED, which only happens inside a subsequent
  // ControllerManager::update() call -- calling it here, before any thread is calling update(),
  // would deadlock forever on the infinite default timeout.
  for (const auto & controller_name : autostart_controllers) {
    if (!controller_manager->load_controller(controller_name)) {
      RCLCPP_FATAL(node->get_logger(), "Failed to load controller '%s'", controller_name.c_str());
      rclcpp::shutdown();
      return 1;
    }
    if (controller_manager->configure_controller(controller_name) !=
        controller_interface::return_type::OK) {
      RCLCPP_FATAL(
        node->get_logger(), "Failed to configure controller '%s'", controller_name.c_str());
      rclcpp::shutdown();
      return 1;
    }
  }

  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  // Phase 3.6 Sim-to-Real purity: renamed from /rov/odom -- this is noise-free ground truth
  // (straight from AdhereMujocoHardware's gt_* state interfaces via readTelemetry()), which does
  // not exist on a real Jetson deployment. adhere_controller's /rov/controller_state never
  // carries this; it is offline-evaluation-only.
  auto ground_truth_odom_pub =
    node->create_publisher<nav_msgs::msg::Odometry>("/rov/ground_truth_odom", 10);
  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/rov/environment_markers", 10);

  // Runtime flow-mode switch (CLAUDE.md Phase 2.5 Stage 2.3, renamed from wave_mode in Phase
  // 3.6 for naming consistency with FlowEnvironmentPlugin): a scripted/remote-control entry
  // point alongside MujocoViewer's own in-window 1/2 keys (the primary interactive control,
  // CLAUDE.md Phase 2.6). Forwarded straight to the one live AdhereMujocoHardware instance via
  // its single-instance back-pointer (see adhere_mujoco_hardware.cpp).
  auto flow_mode_sub = node->create_subscription<std_msgs::msg::Int32>(
    "/rov/flow_mode_cmd", 10,
    [](const std_msgs::msg::Int32::SharedPtr msg) {
      auto * active_hw = adhere_mujoco_hardware::AdhereMujocoHardware::activeInstance();
      if (active_hw) {
        active_hw->setFlowMode(msg->data);
      }
    });

  // Phase 4.2 (generalized Phase 4.3): mirrors AdhereController's live target pose/FSM
  // state/adhere stage into MujocoVisBridge (so the embedded MuJoCo viewer's overlay + green
  // target sphere update) and into these node-local atomics (read by the
  // /rov/environment_markers RViz2 marker above) -- a message-only dependency on
  // adhere_control_msgs, zero MuJoCo/CUDA/ros2_control coupling in the other direction
  // (CLAUDE.md Sec.3.3 governs the reverse link, not this one). Phase 4.3: the sphere/overlay
  // used to be gated on adhere_stage being non-empty (ADHERE_* only); now gated on the
  // ControllerState::target_valid field, which is also true for HOLD_PID_6DOF.
  // ADHERE_* states (FsmCommand IDs 4/5/6, compacted Phase 5.0) still force the sphere's X onto
  // the wall plane (world X=0) regardless of the live standoff/approach X, matching the
  // existing visual design; every other target_valid state uses target_pose.x directly.
  static std::atomic<bool> latest_target_valid{false};
  static std::atomic<double> latest_target_x{0.0};
  static std::atomic<double> latest_target_y{0.0};
  static std::atomic<double> latest_target_z{0.0};
  auto controller_state_sub = node->create_subscription<adhere_control_msgs::msg::ControllerState>(
    "/rov/controller_state", 10,
    [](const adhere_control_msgs::msg::ControllerState::SharedPtr msg) {
      // This phase: 3-stage rename (DEPTH_ALIGN/WALL_ADHERE -> STANDOFF_HOMING/SOFT_DOCKING/
      // WALL_ADHERED); SOFT_DOCKING is PID/SMC-only (MPPI only ever reports STANDOFF_HOMING/
      // WALL_ADHERED), hence the 4-way (0..3) code instead of the former 3-way (0..2).
      const int stage_code = msg->adhere_stage == "STANDOFF_HOMING"
        ? 1
        : (msg->adhere_stage == "SOFT_DOCKING" ? 2 : (msg->adhere_stage == "WALL_ADHERED" ? 3 : 0));
      const bool is_adhere_state = msg->current_state >= 4 && msg->current_state <= 6;
      const double sphere_x = is_adhere_state ? 0.0 : msg->target_pose.position.x;
      latest_target_valid.store(msg->target_valid);
      latest_target_x.store(sphere_x);
      latest_target_y.store(msg->target_pose.position.y);
      latest_target_z.store(msg->target_pose.position.z);

      // Quaternion -> yaw (Z-axis Euler angle), Hamilton (w,x,y,z) convention, matching
      // adhere_controller's own mathTools.h::rotMatToRPY() derivation.
      const double qw = msg->target_pose.orientation.w, qx = msg->target_pose.orientation.x,
                   qy = msg->target_pose.orientation.y, qz = msg->target_pose.orientation.z;
      const double yaw_rad =
        std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

      auto * active_hw = adhere_mujoco_hardware::AdhereMujocoHardware::activeInstance();
      if (!active_hw) {
        return;
      }
      auto bridge = active_hw->visBridge();
      if (!bridge) {
        return;
      }
      bridge->fsm_current_state_code.store(msg->current_state);
      bridge->adhere_stage_code.store(stage_code);
      bridge->target_valid.store(msg->target_valid);
      bridge->target_x.store(sphere_x);
      bridge->target_y.store(msg->target_pose.position.y);
      bridge->target_z.store(msg->target_pose.position.z);
      bridge->target_yaw_deg.store(yaw_rad * 180.0 / M_PI);
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
      ground_truth_odom_pub->publish(odom_msg);

      // No articulated URDF joints exist on this model (see simplified_v1_description's
      // README), so the published JointState is intentionally empty.
      sensor_msgs::msg::JointState joint_state_msg;
      joint_state_msg.header.stamp = now;
      joint_state_pub->publish(joint_state_msg);

      visualization_msgs::msg::MarkerArray markers;

      // Wall marker (CLAUDE.md Phase 5.0 Stage 4: pose/scale read from mjcf/rov_wall.xml's own
      // wall_geom via AdhereMujocoHardware -- single source of truth, never hand-synced again).
      visualization_msgs::msg::Marker wall;
      wall.header.stamp = now;
      wall.header.frame_id = "world";
      wall.ns = "environment";
      wall.id = 0;
      wall.type = visualization_msgs::msg::Marker::CUBE;
      wall.action = visualization_msgs::msg::Marker::ADD;
      wall.pose.position.x = snap.wall_pos_world[0];
      wall.pose.position.y = snap.wall_pos_world[1];
      wall.pose.position.z = snap.wall_pos_world[2];
      wall.scale.x = snap.wall_size_full[0];
      wall.scale.y = snap.wall_size_full[1];
      wall.scale.z = snap.wall_size_full[2];
      wall.color.r = 0.25f;
      wall.color.g = 0.25f;
      wall.color.b = 0.3f;
      wall.color.a = 0.6f;
      markers.markers.push_back(wall);

      // Phase 4.2 (generalized Phase 4.3): green target sphere, mirrors MujocoViewer's own
      // scene-geom version -- published whenever the active FSM state reports a meaningful
      // target_pose (HOLD_PID_6DOF/ADHERE_*), not just the three ADHERE_* states.
      if (latest_target_valid.load()) {
        visualization_msgs::msg::Marker target_sphere;
        target_sphere.header.stamp = now;
        target_sphere.header.frame_id = "world";
        target_sphere.ns = "adhere_target";
        target_sphere.id = 0;
        target_sphere.type = visualization_msgs::msg::Marker::SPHERE;
        target_sphere.action = visualization_msgs::msg::Marker::ADD;
        target_sphere.pose.position.x = latest_target_x.load();
        target_sphere.pose.position.y = latest_target_y.load();
        target_sphere.pose.position.z = latest_target_z.load();
        target_sphere.scale.x = 0.07;
        target_sphere.scale.y = 0.07;
        target_sphere.scale.z = 0.07;
        target_sphere.color.r = 0.1f;
        target_sphere.color.g = 1.0f;
        target_sphere.color.b = 0.2f;
        target_sphere.color.a = 0.9f;
        markers.markers.push_back(target_sphere);
      }

      // Water surface marker (CLAUDE.md Phase 5.0 Stage 4: pose/scale read from
      // mjcf/rov_wall.xml's own water_surface geom), semi-transparent cyan.
      visualization_msgs::msg::Marker water_surface;
      water_surface.header.stamp = now;
      water_surface.header.frame_id = "world";
      water_surface.ns = "environment";
      water_surface.id = 1;
      water_surface.type = visualization_msgs::msg::Marker::CUBE;
      water_surface.action = visualization_msgs::msg::Marker::ADD;
      water_surface.pose.position.z = snap.water_surface_z;
      water_surface.scale.x = snap.water_surface_extent_xy[0];
      water_surface.scale.y = snap.water_surface_extent_xy[1];
      water_surface.scale.z = 0.01;
      water_surface.color.r = 0.2f;
      water_surface.color.g = 0.8f;
      water_surface.color.b = 1.0f;
      water_surface.color.a = 0.25f;
      markers.markers.push_back(water_surface);

      // Tank floor marker (CLAUDE.md Phase 5.0 Stage 4: pose/scale read from
      // mjcf/rov_wall.xml's own floor_geom).
      visualization_msgs::msg::Marker floor;
      floor.header.stamp = now;
      floor.header.frame_id = "world";
      floor.ns = "environment";
      floor.id = 2;
      floor.type = visualization_msgs::msg::Marker::CUBE;
      floor.action = visualization_msgs::msg::Marker::ADD;
      floor.pose.position.z = snap.floor_z;
      floor.scale.x = snap.floor_extent_xy[0];
      floor.scale.y = snap.floor_extent_xy[1];
      floor.scale.z = 0.05;
      floor.color.r = 0.4f;
      floor.color.g = 0.4f;
      floor.color.b = 0.4f;
      floor.color.a = 0.8f;
      markers.markers.push_back(floor);

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
        p0.x = snap.pad_site_pos_body[i][0];
        p0.y = snap.pad_site_pos_body[i][1];
        p0.z = snap.pad_site_pos_body[i][2];
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
    constexpr auto kBasePeriod =
      std::chrono::duration<double>(1.0 / kPhysicsUpdateRateHz);
    auto next_tick = Clock::now() + std::chrono::duration_cast<Clock::duration>(kBasePeriod);
    auto last_tick = Clock::now();
    double actual_hz_ema = kPhysicsUpdateRateHz;
    double step_work_ms_ema = 0.0;
    auto bridge = hw->visBridge();

    while (rclcpp::ok() && physics_running.load()) {
      const auto work_start = Clock::now();
      controller_manager->read();
      controller_manager->update();
      controller_manager->write();
      const double work_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - work_start).count();
      step_work_ms_ema = 0.98 * step_work_ms_ema + 0.02 * work_ms;
      if (bridge) {
        bridge->step_work_ms.store(step_work_ms_ema);
      }

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

      // Slow-motion wall-clock scale (CLAUDE.md Phase 2.10 Section C4): factor<1 stretches
      // the wall-clock period between ticks (factor=0.1 -> 10x longer period -> 10x slower
      // wall-clock stepping), while mjModel::opt.timestep -- and therefore every numerical-
      // parity contract -- stays exactly 2.0ms, unaffected. Clamped to (0, 1] so the GUI can
      // never speed the simulation up past real-time through this control.
      const double speed_factor =
        bridge ? std::clamp(bridge->sim_speed_factor.load(), 0.01, 1.0) : 1.0;
      const auto period =
        std::chrono::duration_cast<Clock::duration>(kBasePeriod / speed_factor);

      std::this_thread::sleep_until(next_tick);
      next_tick += period;
      if (next_tick < Clock::now()) {
        // Fell behind by more than one period (e.g. a debugger breakpoint): resync instead of
        // free-running catch-up ticks.
        next_tick = Clock::now() + period;
      }
    }
  });

  // Now that physics_thread is concurrently calling controller_manager->update() every tick,
  // switch_controller()'s busy-wait for the pending switch to be applied resolves quickly
  // (CLAUDE.md Phase 3.1.1 Stage 2.2). Skipped entirely when autostart_controllers is empty --
  // mujoco_sim.launch.py's default plain hardware launch stays at exactly 0 active controllers.
  if (!autostart_controllers.empty() &&
      controller_manager->switch_controller(
        autostart_controllers, {}, controller_manager_msgs::srv::SwitchController::Request::STRICT,
        true) != controller_interface::return_type::OK) {
    RCLCPP_FATAL(node->get_logger(), "Failed to activate autostart_controllers");
    physics_running = false;
    physics_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  executor->spin();

  physics_running = false;
  physics_thread.join();
  rclcpp::shutdown();
  return 0;
}
