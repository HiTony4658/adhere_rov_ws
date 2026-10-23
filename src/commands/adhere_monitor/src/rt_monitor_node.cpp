// rt_monitor_node: live CLI dashboard for /rov/runtime_diagnostics (CLAUDE.md Phase 3.6 Stage
// 3.2) and /rov/benchmark_metrics' Phase 3.8 efficiency/safety telemetry (repointed off
// /rov/controller_state in Phase 5.0 Stage 7). Subscribes to both (never raw hardware topics)
// and redraws a full-screen ANSI table on every message from either topic, highlighting any overrun, off-target frequency, saturation, or non-positive Rhombic
// Kern margin in red. Zero dependency on MuJoCo/CUDA/ros2_control, so it runs standalone on a
// remote Upper-Computer (GCS) laptop like adhere_teleop_gcs's/this package's other nodes.
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "adhere_control_msgs/msg/benchmark_metrics.hpp"
#include "adhere_control_msgs/msg/runtime_diagnostics.hpp"

namespace
{

constexpr const char * kRed = "\033[31m";
constexpr const char * kGreen = "\033[32m";
constexpr const char * kReset = "\033[0m";
constexpr const char * kClearScreen = "\033[2J\033[H";

// Latest Phase 3.8 efficiency/safety telemetry (updated by the /rov/benchmark_metrics
// subscription, read by the /rov/runtime_diagnostics subscription's redraw so both tables stay
// on screen together regardless of which topic's message triggered this particular redraw).
std::mutex g_metrics_mutex;
bool g_have_metrics = false;
adhere_control_msgs::msg::BenchmarkMetrics g_latest_benchmark_metrics;

std::string colorize(double measured, double target, const std::string & text)
{
  const bool ok = target <= 0.0 || std::abs(measured - target) <= 0.10 * target;
  return (ok ? kGreen : kRed) + text + kReset;
}

std::string colorizeOverrun(uint64_t count)
{
  const std::string text = std::to_string(count);
  return (count == 0 ? kGreen : kRed) + text + kReset;
}

void printRow(const char * label, double hz, double target, double exec_ms, double budget_ms)
{
  char line[256];
  if (budget_ms > 0.0) {
    std::snprintf(
      line, sizeof(line), "%-16s %8.1f Hz (target %6.1f)   exec %6.2f ms (budget %5.2f ms)",
      label, hz, target, exec_ms, budget_ms);
  } else {
    std::snprintf(
      line, sizeof(line), "%-16s %8.1f Hz (target %6.1f)   exec %6.2f ms", label, hz, target,
      exec_ms);
  }
  std::printf("%s\n", colorize(hz, target, std::string(line)).c_str());
}

void printDiagnosticsTable(const adhere_control_msgs::msg::RuntimeDiagnostics & msg)
{
  std::printf(
    "============================================================\n"
    "  ADHERE ROV Real-Time Diagnostics (/rov/runtime_diagnostics)\n"
    "============================================================\n");
  printRow("HW Loop", msg.hw_loop_hz, 500.0, msg.hw_step_exec_ms, 0.0);
  printRow("Sensor IMU", msg.sensor_imu_hz, 500.0, 0.0, 0.0);
  printRow("Sensor LoadCell", msg.sensor_loadcell_hz, 500.0, 0.0, 0.0);
  printRow("Sensor Depth", msg.sensor_depth_hz, 50.0, 0.0, 0.0);
  printRow("Sensor DVL", msg.sensor_dvl_hz, 20.0, 0.0, 0.0);
  printRow("StateEstimator", msg.estimator_hz, 0.0, msg.estimator_exec_ms, 0.0);
  printRow("Inner Loop", msg.inner_loop_hz, 250.0, msg.inner_loop_exec_ms, msg.inner_loop_budget_ms);
  printRow("Outer Loop", msg.outer_loop_hz, 25.0, msg.outer_loop_exec_ms, msg.outer_loop_budget_ms);
  printRow("Thruster Cmd", msg.thruster_cmd_hz, 250.0, 0.0, 0.0);
  // Phase 4.1: 0.0 whenever no GPU outer-loop planner (ADHERE_MPPI_BASELINE/ADHERE_MPPI_KAMR) is active -- not
  // colorized against a target Hz like the rows above, just the raw worker-thread solve latency.
  std::printf(
    "%-16s                                 solve %6.2f ms\n", "GPU Planner",
    msg.outer_planner_solve_ms);
  std::printf("------------------------------------------------------------\n");
  const std::string status =
    (msg.timing_healthy ? std::string(kGreen) + "HEALTHY" : std::string(kRed) + "UNHEALTHY") +
    kReset;
  std::printf(
    "Inner overruns: %s   Outer overruns: %s   Status: %s\n",
    colorizeOverrun(msg.inner_loop_overruns).c_str(),
    colorizeOverrun(msg.outer_loop_overruns).c_str(), status.c_str());
}

// Phase 3.8 efficiency/chattering/safety table, built from the latest /rov/benchmark_metrics
// message (if any has arrived yet).
void printMetricsTable()
{
  adhere_control_msgs::msg::BenchmarkMetrics m;
  bool have = false;
  {
    std::lock_guard<std::mutex> lock(g_metrics_mutex);
    have = g_have_metrics;
    m = g_latest_benchmark_metrics;
  }
  std::printf(
    "============================================================\n"
    "  Phase 3.8 Efficiency/Safety Telemetry (/rov/benchmark_metrics)\n"
    "============================================================\n");
  if (!have) {
    std::printf("  (no /rov/benchmark_metrics message received yet)\n");
    return;
  }
  char line[256];
  std::snprintf(
    line, sizeof(line),
    "||u||=%7.2fN  Power=%7.2f  Slew=%8.2fN/s  E=%10.2fN^2s  I=%9.2fNs", m.thrust_l2_norm,
    m.thrust_power_proxy, m.thrust_slew_rate_l2, m.cumulative_energy_l2, m.cumulative_impulse_l1);
  std::printf("%s\n", line);
  const std::string sat_text =
    std::to_string(static_cast<int>(m.saturated_thruster_count)) + "/8";
  const std::string sat_colored =
    (m.saturated_thruster_count == 0 ? std::string(kGreen) : std::string(kRed)) + sat_text +
    kReset;
  const std::string pads_text =
    std::to_string(static_cast<int>(m.active_contact_pad_count)) + "/4";
  const std::string pads_colored =
    (m.active_contact_pad_count == 4 ? std::string(kGreen) : std::string(kRed)) + pads_text +
    kReset;
  char kern_line[64];
  std::snprintf(kern_line, sizeof(kern_line), "%.3f", m.kern_margin);
  const std::string kern_colored =
    (m.kern_margin > 0.0 ? std::string(kGreen) : std::string(kRed)) + kern_line + kReset;
  std::printf(
    "Saturated thrusters: %s   Active contact pads: %s   Kern margin: %s\n", sat_colored.c_str(),
    pads_colored.c_str(), kern_colored.c_str());
  std::printf("------------------------------------------------------------\n");
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rt_monitor_node");

  auto benchmark_metrics_sub =
    node->create_subscription<adhere_control_msgs::msg::BenchmarkMetrics>(
      "/rov/benchmark_metrics", 10,
      [](const adhere_control_msgs::msg::BenchmarkMetrics::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(g_metrics_mutex);
        g_latest_benchmark_metrics = *msg;
        g_have_metrics = true;
      });

  auto sub = node->create_subscription<adhere_control_msgs::msg::RuntimeDiagnostics>(
    "/rov/runtime_diagnostics", 10,
    [](const adhere_control_msgs::msg::RuntimeDiagnostics::SharedPtr msg) {
      std::printf("%s", kClearScreen);
      printDiagnosticsTable(*msg);
      printMetricsTable();
      std::fflush(stdout);
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
