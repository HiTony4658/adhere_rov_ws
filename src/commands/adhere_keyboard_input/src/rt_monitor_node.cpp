// rt_monitor_node: live CLI dashboard for /rov/runtime_diagnostics (CLAUDE.md Phase 3.6 Stage
// 3.2). Subscribes only to RuntimeDiagnostics (never raw hardware topics) and redraws a
// full-screen ANSI table of all 9 frequency/latency channels on every message, highlighting any
// overrun or off-target frequency in red. Zero dependency on MuJoCo/CUDA/ros2_control, so it
// runs standalone on a remote Upper-Computer (GCS) laptop like the other adhere_keyboard_input
// nodes.
#include <cmath>
#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "adhere_control_msgs/msg/runtime_diagnostics.hpp"

namespace
{

constexpr const char * kRed = "\033[31m";
constexpr const char * kGreen = "\033[32m";
constexpr const char * kReset = "\033[0m";
constexpr const char * kClearScreen = "\033[2J\033[H";

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

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rt_monitor_node");

  auto sub = node->create_subscription<adhere_control_msgs::msg::RuntimeDiagnostics>(
    "/rov/runtime_diagnostics", 10,
    [](const adhere_control_msgs::msg::RuntimeDiagnostics::SharedPtr msg) {
      std::printf("%s", kClearScreen);
      std::printf(
        "============================================================\n"
        "  ADHERE ROV Real-Time Diagnostics (/rov/runtime_diagnostics)\n"
        "============================================================\n");
      printRow("HW Loop", msg->hw_loop_hz, 500.0, msg->hw_step_exec_ms, 0.0);
      printRow("Sensor IMU", msg->sensor_imu_hz, 500.0, 0.0, 0.0);
      printRow("Sensor LoadCell", msg->sensor_loadcell_hz, 500.0, 0.0, 0.0);
      printRow("Sensor Depth", msg->sensor_depth_hz, 50.0, 0.0, 0.0);
      printRow("Sensor DVL", msg->sensor_dvl_hz, 20.0, 0.0, 0.0);
      printRow("StateEstimator", msg->estimator_hz, 0.0, msg->estimator_exec_ms, 0.0);
      printRow(
        "Inner Loop", msg->inner_loop_hz, 250.0, msg->inner_loop_exec_ms,
        msg->inner_loop_budget_ms);
      printRow(
        "Outer Loop", msg->outer_loop_hz, 25.0, msg->outer_loop_exec_ms,
        msg->outer_loop_budget_ms);
      printRow("Thruster Cmd", msg->thruster_cmd_hz, 250.0, 0.0, 0.0);
      std::printf("------------------------------------------------------------\n");
      const std::string status =
        (msg->timing_healthy ? std::string(kGreen) + "HEALTHY" : std::string(kRed) + "UNHEALTHY") +
        kReset;
      std::printf(
        "Inner overruns: %s   Outer overruns: %s   Status: %s\n",
        colorizeOverrun(msg->inner_loop_overruns).c_str(),
        colorizeOverrun(msg->outer_loop_overruns).c_str(), status.c_str());
      std::fflush(stdout);
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
