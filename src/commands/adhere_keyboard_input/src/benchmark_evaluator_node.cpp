// benchmark_evaluator_node: standalone offline-evaluation node (CLAUDE.md Phase 3.8 Stage 2.3)
// that turns a rolling window of /rov/controller_state + /rov/ground_truth_odom +
// /rov/runtime_diagnostics telemetry into a publication-ready comparison table (tracking
// accuracy vs. ground truth, control energy/effort, smoothness/chattering, wall-adhesion
// safety). This is the one node in adhere_keyboard_input explicitly allowed to subscribe to
// /rov/ground_truth_odom (CLAUDE.md: that topic is offline-evaluation-only, never consumed by
// adhere_controller) -- every other node in this package stays Sim-to-Real pure. Zero
// dependency on MuJoCo/CUDA/ros2_control: nav_msgs/Odometry is a core ROS 2 message type, so
// this still deploys standalone on a remote GCS laptop.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "adhere_control_msgs/msg/controller_state.hpp"
#include "adhere_control_msgs/msg/runtime_diagnostics.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"

namespace
{

// One sample per /rov/controller_state message, paired with the most recent ground-truth
// odometry known at that time (independent topics/rates, so this is a "latest known" pairing,
// not a timestamp-synchronized one -- acceptable for a 12s-window offline evaluation tool).
struct Sample
{
  rclcpp::Time stamp;
  double est_y = 0.0, est_z = 0.0;
  double gt_y = 0.0, gt_z = 0.0;
  // Minimal-rotation angle (deg) between estimated and ground-truth orientation, computed from
  // the relative quaternion's scalar part (2*acos(|w_rel|)) -- robust to hemisphere sign, no
  // dependency on adhere_controller's mathTools.h (this package stays zero-dependency on it).
  double attitude_error_deg = 0.0;
  double cumulative_energy_l2 = 0.0;
  double cumulative_impulse_l1 = 0.0;
  double thrust_power_proxy = 0.0;
  double thrust_slew_rate_l2 = 0.0;
  bool any_saturated = false;
  double kern_margin = 0.0;
  bool full_contact = false;
};

std::mutex g_mutex;
std::deque<Sample> g_window;
double g_window_sec = 12.0;

bool g_have_gt = false;
double g_gt_y = 0.0, g_gt_z = 0.0;
double g_gt_qw = 1.0, g_gt_qx = 0.0, g_gt_qy = 0.0, g_gt_qz = 0.0;

double attitudeErrorDeg(
  double qw_a, double qx_a, double qy_a, double qz_a, double qw_b, double qx_b, double qy_b,
  double qz_b)
{
  // Relative quaternion scalar part |q_a . q_b| (dot product of the two unit quaternions) is
  // cos(theta/2) of the minimal rotation between them, hemisphere-independent via the abs().
  const double dot = qw_a * qw_b + qx_a * qx_b + qy_a * qy_b + qz_a * qz_b;
  const double clamped = std::clamp(std::abs(dot), 0.0, 1.0);
  return 2.0 * std::acos(clamped) * 180.0 / M_PI;
}

void pruneWindow(const rclcpp::Time & now)
{
  while (!g_window.empty() && (now - g_window.front().stamp).seconds() > g_window_sec) {
    g_window.pop_front();
  }
}

template<typename Fn>
double rmsOf(Fn fn)
{
  double sum_sq = 0.0;
  size_t n = 0;
  for (const auto & s : g_window) {
    const double v = fn(s);
    sum_sq += v * v;
    ++n;
  }
  return n > 0 ? std::sqrt(sum_sq / n) : 0.0;
}

template<typename Fn>
double maxOf(Fn fn)
{
  double m = 0.0;
  for (const auto & s : g_window) {
    m = std::max(m, fn(s));
  }
  return m;
}

void printReport()
{
  std::lock_guard<std::mutex> lock(g_mutex);
  if (g_window.size() < 2) {
    std::printf("benchmark_evaluator_node: waiting for samples (%zu so far)...\n", g_window.size());
    std::fflush(stdout);
    return;
  }

  const double yz_rmse =
    rmsOf([](const Sample & s) { return std::hypot(s.est_y - s.gt_y, s.est_z - s.gt_z); });
  const double yz_max =
    maxOf([](const Sample & s) { return std::hypot(s.est_y - s.gt_y, s.est_z - s.gt_z); });
  const double att_rmse = rmsOf([](const Sample & s) { return s.attitude_error_deg; });
  const double att_max = maxOf([](const Sample & s) { return s.attitude_error_deg; });

  const double e_u = g_window.back().cumulative_energy_l2 - g_window.front().cumulative_energy_l2;
  const double i_u =
    g_window.back().cumulative_impulse_l1 - g_window.front().cumulative_impulse_l1;
  double mean_power = 0.0;
  for (const auto & s : g_window) {
    mean_power += s.thrust_power_proxy;
  }
  mean_power /= static_cast<double>(g_window.size());

  const double slew_rms = rmsOf([](const Sample & s) { return s.thrust_slew_rate_l2; });
  const double slew_peak = maxOf([](const Sample & s) { return s.thrust_slew_rate_l2; });
  size_t saturated_count = 0;
  size_t kern_violation_count = 0;
  size_t full_contact_count = 0;
  double min_kern = g_window.front().kern_margin;
  for (const auto & s : g_window) {
    if (s.any_saturated) {++saturated_count;}
    if (s.kern_margin < 0.0) {++kern_violation_count;}
    if (s.full_contact) {++full_contact_count;}
    min_kern = std::min(min_kern, s.kern_margin);
  }
  const double n = static_cast<double>(g_window.size());

  std::printf(
    "\n"
    "================================================================\n"
    "  Benchmark Evaluation (trailing %.1fs window, %zu samples)\n"
    "================================================================\n"
    "| Metric                                   | Value          |\n"
    "|-------------------------------------------|----------------|\n"
    "| Tangential (Y,Z) RMSE vs GT (m)            | %14.4f |\n"
    "| Max Tangential Slip (m)                    | %14.4f |\n"
    "| Attitude RMSE vs GT (deg)                  | %14.4f |\n"
    "| Max Attitude Error (deg)                   | %14.4f |\n"
    "| Quadratic Energy E_u (N^2 s)                | %14.4f |\n"
    "| L1 Impulse I_u (N s)                        | %14.4f |\n"
    "| Mean Power Proxy (N^1.5)                    | %14.4f |\n"
    "| Thrust Slew Rate RMS (N/s)                  | %14.4f |\n"
    "| Peak Slew Rate (N/s)                        | %14.4f |\n"
    "| Thruster Saturation Ratio (%%)               | %14.2f |\n"
    "| Min Rhombic Kern Margin                     | %14.4f |\n"
    "| Kern Violation Ratio (%%)                    | %14.2f |\n"
    "| Full 4-Pad Contact Retention (%%)            | %14.2f |\n"
    "================================================================\n",
    g_window_sec, g_window.size(), yz_rmse, yz_max, att_rmse, att_max, e_u, i_u, mean_power,
    slew_rms, slew_peak, 100.0 * saturated_count / n, min_kern,
    100.0 * kern_violation_count / n, 100.0 * full_contact_count / n);
  std::fflush(stdout);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("benchmark_evaluator_node");

  g_window_sec = node->declare_parameter<double>("window_sec", 12.0);
  const bool trigger_flow_gust = node->declare_parameter<bool>("trigger_flow_gust", false);

  auto flow_mode_pub = node->create_publisher<std_msgs::msg::Int32>("/rov/flow_mode_cmd", 10);

  auto gt_odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "/rov/ground_truth_odom", 10, [](const nav_msgs::msg::Odometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_mutex);
      g_gt_y = msg->pose.pose.position.y;
      g_gt_z = msg->pose.pose.position.z;
      g_gt_qw = msg->pose.pose.orientation.w;
      g_gt_qx = msg->pose.pose.orientation.x;
      g_gt_qy = msg->pose.pose.orientation.y;
      g_gt_qz = msg->pose.pose.orientation.z;
      g_have_gt = true;
    });

  auto runtime_diagnostics_sub =
    node->create_subscription<adhere_control_msgs::msg::RuntimeDiagnostics>(
      "/rov/runtime_diagnostics", 10,
      [](const adhere_control_msgs::msg::RuntimeDiagnostics::SharedPtr) {
        // Timing-health is reported in the live rt_monitor_node table, not duplicated here --
        // this subscription only exists so benchmark_evaluator_node can be extended to gate the
        // report on timing_healthy in the future without a topic-wiring change.
      });

  auto controller_state_sub =
    node->create_subscription<adhere_control_msgs::msg::ControllerState>(
      "/rov/controller_state", 10,
      [node](const adhere_control_msgs::msg::ControllerState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (!g_have_gt) {
          return;
        }
        Sample s;
        s.stamp = msg->header.stamp;
        s.est_y = msg->estimated_pose.position.y;
        s.est_z = msg->estimated_pose.position.z;
        s.gt_y = g_gt_y;
        s.gt_z = g_gt_z;
        s.attitude_error_deg = attitudeErrorDeg(
          msg->estimated_pose.orientation.w, msg->estimated_pose.orientation.x,
          msg->estimated_pose.orientation.y, msg->estimated_pose.orientation.z, g_gt_qw, g_gt_qx,
          g_gt_qy, g_gt_qz);
        s.cumulative_energy_l2 = msg->cumulative_energy_l2;
        s.cumulative_impulse_l1 = msg->cumulative_impulse_l1;
        s.thrust_power_proxy = msg->thrust_power_proxy;
        s.thrust_slew_rate_l2 = msg->thrust_slew_rate_l2;
        s.any_saturated = msg->saturated_thruster_count > 0;
        s.kern_margin = msg->kern_margin;
        s.full_contact = msg->active_contact_pad_count == 4;

        g_window.push_back(s);
        pruneWindow(s.stamp);
      });

  if (trigger_flow_gust) {
    // Single edge-triggered publish at startup (not a timer) to kick off the 12.0s wave-gust
    // benchmark scenario (CLAUDE.md Sec.3's FlowEnvironmentPlugin mode 2).
    std_msgs::msg::Int32 flow_mode_msg;
    flow_mode_msg.data = 2;
    flow_mode_pub->publish(flow_mode_msg);
  }

  auto report_timer = node->create_wall_timer(std::chrono::seconds(2), printReport);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
