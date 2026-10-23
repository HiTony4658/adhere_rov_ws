// fsm_console_node: master FSM state-switcher terminal (CLAUDE.md Phase 3.2 Stage 2.2, Phase
// 5.0 simplified). Subscribes ONLY to /rov/controller_state (never raw hardware/odom topics) and
// publishes ONLY FsmCommand on /rov/fsm_command, plus a compact status readout from
// /rov/runtime_diagnostics. Phase 5.0 deleted the per-state setpoint-terminal auto-spawn
// machinery (state_cmd_console_node itself is also deleted): the joystick is the primary
// interface, and the controller's own pose is only an estimate, so manual numeric setpoints are
// entered ad hoc via `ros2 topic pub /rov/setpoint_cmd ...` instead of a dedicated console.
#include <array>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <csignal>

#include "rclcpp/rclcpp.hpp"

#include "adhere_control_msgs/msg/benchmark_metrics.hpp"
#include "adhere_control_msgs/msg/controller_state.hpp"
#include "adhere_control_msgs/msg/fsm_command.hpp"
#include "adhere_control_msgs/msg/runtime_diagnostics.hpp"

namespace
{

std::atomic<int32_t> g_current_state{adhere_control_msgs::msg::FsmCommand::IDLE_PASSIVE_NONE};
std::string g_state_name = "state_idle_passive_none";
std::mutex g_state_name_mutex;

// Set by handleSignal() on SIGINT/SIGTERM/SIGHUP; the watchdog thread below polls this (and
// rclcpp::ok()) every 100ms so a blocking std::getline(std::cin, ...) (e.g. stdin closed by the
// terminal emulator on window-close) never hangs shutdown.
std::atomic<bool> g_shutdown_requested{false};

void handleSignal(int /*sig*/)
{
  g_shutdown_requested.store(true);
}

// Compact 1-line timing summary (CLAUDE.md Phase 3.6 Stage 3.2), updated from
// /rov/runtime_diagnostics -- full detail lives in rt_monitor_node, this is just a glance.
std::mutex g_timing_mutex;
std::string g_timing_line = "Inner: --Hz  Outer: --Hz  Status: --";

// Compact 1-line efficiency/safety summary (CLAUDE.md Phase 3.8), updated from
// /rov/controller_state -- full detail also lives in rt_monitor_node.
std::mutex g_metrics_mutex;
std::string g_metrics_line = "||u||: --N  Power: --  Slew: --N/s  E: --N^2s  Sat: --/8  Pads: --/4  Kern: --";

const char * stateLabel(int32_t state)
{
  switch (state) {
    case adhere_control_msgs::msg::FsmCommand::IDLE_PASSIVE_NONE:
      return "[1] state_idle_passive_none";
    case adhere_control_msgs::msg::FsmCommand::TELEOP_OPENLOOP_6DOF:
      return "[2] state_teleop_openloop_6dof";
    case adhere_control_msgs::msg::FsmCommand::HOLD_PID_6DOF:
      return "[3] state_hold_pid_6dof";
    case adhere_control_msgs::msg::FsmCommand::ADHERE_PID_WBC:
      return "[4] state_adhere_pid_wbc";
    case adhere_control_msgs::msg::FsmCommand::ADHERE_SMC_WBC:
      return "[5] state_adhere_smc_wbc";
    case adhere_control_msgs::msg::FsmCommand::ADHERE_MPPI_BASELINE:
      return "[6] state_adhere_mppi_baseline";
    case adhere_control_msgs::msg::FsmCommand::ADHERE_MPPI_KAMR:
      return "[7] state_adhere_mppi_kamr";
    default:
      return "[?] unknown";
  }
}

void printMenu()
{
  std::string state_name;
  {
    std::lock_guard<std::mutex> lock(g_state_name_mutex);
    state_name = g_state_name;
  }
  std::cout << "============================================================\n"
            << "  Underwater ROV FSM Control Console (有限状态机终端控制台)\n"
            << "============================================================\n"
            << "  [1] state_idle_passive_none      (推进器全零输出 u = 0 N，自由漂流)\n"
            << "  [2] state_teleop_openloop_6dof   (Xbox 开环推力映射，B 伪逆分配)\n"
            << "  [3] state_hold_pid_6dof          (6DOF 位置+航向保持，手动设定点见下)\n"
            << "  [4] state_adhere_pid_wbc         (贴壁 PID 基线：三阶段靠壁+法向力 PI 保压)\n"
            << "  [5] state_adhere_smc_wbc         (贴壁 SMC 基线：6DOF 滑模控制+法向预压)\n"
            << "  [6] state_adhere_mppi_baseline   (纯 CUDA 外环 MPPI，250Hz 零阶保持)\n"
            << "  [q] Quit Console     (退出控制台)\n"
            << "------------------------------------------------------------\n"
            << "手动数值设定点（深度/位姿/贴壁目标）请用 ros2 topic pub 直接发布到\n"
            << "/rov/setpoint_cmd 或 /rov/adhere_target_cmd（不再自动弹出专属终端）。\n"
            << "------------------------------------------------------------\n"
            << "Current State: " << stateLabel(g_current_state.load()) << " (" << state_name
            << ")\n";
  {
    std::lock_guard<std::mutex> lock(g_timing_mutex);
    std::cout << g_timing_line << "\n";
  }
  {
    std::lock_guard<std::mutex> lock(g_metrics_mutex);
    std::cout << g_metrics_line << "\n";
  }
  std::cout << "Select state (1/2/3/4/5/6/q): " << std::flush;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fsm_console_node");

  auto fsm_command_pub =
    node->create_publisher<adhere_control_msgs::msg::FsmCommand>("/rov/fsm_command", 10);

  auto controller_state_sub = node->create_subscription<adhere_control_msgs::msg::ControllerState>(
    "/rov/controller_state", 10, [](const adhere_control_msgs::msg::ControllerState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_state_name_mutex);
      g_state_name = msg->state_name;
      g_current_state.store(msg->current_state);
    });

  // CLAUDE.md Phase 5.0 Stage 7: efficiency/safety telemetry moved off ControllerState onto its
  // own BenchmarkMetrics message (/rov/benchmark_metrics, 10Hz diag rate).
  auto benchmark_metrics_sub =
    node->create_subscription<adhere_control_msgs::msg::BenchmarkMetrics>(
      "/rov/benchmark_metrics", 10,
      [](const adhere_control_msgs::msg::BenchmarkMetrics::SharedPtr msg) {
        std::ostringstream oss;
        oss << "||u||: " << static_cast<int>(msg->thrust_l2_norm + 0.5) << "N  Power: "
            << static_cast<int>(msg->thrust_power_proxy + 0.5) << "  Slew: "
            << static_cast<int>(msg->thrust_slew_rate_l2 + 0.5) << "N/s  E: "
            << static_cast<int>(msg->cumulative_energy_l2 + 0.5) << "N^2s  Sat: "
            << static_cast<int>(msg->saturated_thruster_count) << "/8  Pads: "
            << static_cast<int>(msg->active_contact_pad_count) << "/4  Kern: " << std::fixed
            << std::setprecision(2) << msg->kern_margin;
        std::lock_guard<std::mutex> lock(g_metrics_mutex);
        g_metrics_line = oss.str();
      });

  auto runtime_diagnostics_sub =
    node->create_subscription<adhere_control_msgs::msg::RuntimeDiagnostics>(
      "/rov/runtime_diagnostics", 10,
      [](const adhere_control_msgs::msg::RuntimeDiagnostics::SharedPtr msg) {
        std::ostringstream oss;
        oss << "Timing: Inner " << static_cast<int>(msg->inner_loop_hz + 0.5) << "Hz  Outer "
            << static_cast<int>(msg->outer_loop_hz + 0.5) << "Hz  Status: "
            << (msg->timing_healthy ? "HEALTHY" : "UNHEALTHY")
            << " (full detail: ros2 run adhere_monitor rt_monitor_node)";
        std::lock_guard<std::mutex> lock(g_timing_mutex);
        g_timing_line = oss.str();
      });

  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);
  std::signal(SIGHUP, handleSignal);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spin_thread([executor]() { executor->spin(); });

  // Watchdog: forces the process to exit even if std::getline(std::cin, ...) below is stuck
  // blocking on a terminal whose window/stdin already closed.
  std::thread watchdog_thread([]() {
    while (rclcpp::ok() && !g_shutdown_requested.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::exit(0);
  });
  watchdog_thread.detach();

  while (rclcpp::ok()) {
    printMenu();

    std::string line;
    if (!std::getline(std::cin, line)) {
      break;
    }
    if (line.empty()) {
      continue;
    }
    if (line == "q" || line == "Q") {
      break;
    }

    int command = 0;
    if (line == "1") {
      command = adhere_control_msgs::msg::FsmCommand::IDLE_PASSIVE_NONE;
    } else if (line == "2") {
      command = adhere_control_msgs::msg::FsmCommand::TELEOP_OPENLOOP_6DOF;
    } else if (line == "3") {
      command = adhere_control_msgs::msg::FsmCommand::HOLD_PID_6DOF;
    } else if (line == "4") {
      command = adhere_control_msgs::msg::FsmCommand::ADHERE_PID_WBC;
    } else if (line == "5") {
      command = adhere_control_msgs::msg::FsmCommand::ADHERE_SMC_WBC;
    } else if (line == "6") {
      command = adhere_control_msgs::msg::FsmCommand::ADHERE_MPPI_BASELINE;
    } else {
      std::cout << "Unrecognized selection \"" << line << "\", expected 1, 2, 3, 4, 5, 6, or q.\n\n";
      continue;
    }

    adhere_control_msgs::msg::FsmCommand cmd;
    cmd.header.stamp = node->now();
    cmd.command = command;
    fsm_command_pub->publish(cmd);
    std::cout << "-> requested " << stateLabel(command) << "\n\n";
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
