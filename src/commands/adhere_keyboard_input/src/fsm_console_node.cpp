// fsm_console_node: master FSM state-switcher terminal (CLAUDE.md Phase 3.2 Stage 2.2).
// Subscribes ONLY to /rov/controller_state (never raw hardware/odom topics) and publishes ONLY
// FsmCommand on /rov/fsm_command -- all state-specific numeric setpoints live on
// state_cmd_console_node instead. Dynamically spawns/kills that per-state console as a
// dedicated gnome-terminal/xterm window whenever the ACTIVE state changes, regardless of
// whether the switch came from this console's own keyboard input or an Xbox gamepad button
// (the /rov/controller_state subscription is the only source of truth for "what state is
// active").
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <csignal>
#include <signal.h>
#include <sys/prctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "adhere_control_msgs/msg/controller_state.hpp"
#include "adhere_control_msgs/msg/fsm_command.hpp"
#include "adhere_control_msgs/msg/runtime_diagnostics.hpp"

namespace
{

std::atomic<int32_t> g_current_state{adhere_control_msgs::msg::FsmCommand::IDLE_PASSIVE_NONE};
std::string g_state_name = "state_idle_passive_none";
std::mutex g_state_name_mutex;

// setsid() in spawnStateConsole() below puts the forked child in its own session, so its pid
// IS its process group id -- killpg() on that pid tears down gnome-terminal/xterm and the
// inner ros2 run process together.
std::atomic<pid_t> g_child_pgid{-1};

// Set by handleSignal() on SIGINT/SIGTERM/SIGHUP; the watchdog thread below polls this (and
// rclcpp::ok()) every 100ms so a blocking std::getline(std::cin, ...) (e.g. stdin closed by the
// terminal emulator on window-close) never hangs shutdown.
std::atomic<bool> g_shutdown_requested{false};

void killChildConsole();

void handleSignal(int /*sig*/)
{
  g_shutdown_requested.store(true);
  // killpg() is a thin syscall wrapper, safe enough to call directly from a signal handler for
  // this interactive CLI tool (not a hard real-time context).
  killChildConsole();
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
    case adhere_control_msgs::msg::FsmCommand::HOLD_PID_DEPTH:
      return "[2] state_hold_pid_depth";
    case adhere_control_msgs::msg::FsmCommand::TELEOP_OPENLOOP_6DOF:
      return "[3] state_teleop_openloop_6dof";
    case adhere_control_msgs::msg::FsmCommand::HOLD_PID_6DOF:
      return "[4] state_hold_pid_6dof";
    case adhere_control_msgs::msg::FsmCommand::ADHERE_PID_WBC:
      return "[5] state_adhere_pid_wbc";
    case adhere_control_msgs::msg::FsmCommand::ADHERE_SMC_WBC:
      return "[6] state_adhere_smc_wbc";
    case adhere_control_msgs::msg::FsmCommand::ADHERE_MPPI_BASELINE:
      return "[7] state_adhere_mppi_baseline";
    case adhere_control_msgs::msg::FsmCommand::ADHERE_MPPI_KAMR:
      return "[8] state_adhere_mppi_kamr";
    default:
      return "[?] unknown";
  }
}

void killChildConsole()
{
  const pid_t pgid = g_child_pgid.exchange(-1);
  if (pgid > 0) {
    killpg(pgid, SIGTERM);
  }
}

// Forks a child that execs a shell snippet trying gnome-terminal first (with --wait, so the
// forked pid's lifetime tracks the inner command and killChildConsole() above actually works --
// a bare gnome-terminal client without --wait exits immediately after asking the persistent
// gnome-terminal-server to open a window, which would make the forked pid meaningless to kill),
// falling back to xterm if gnome-terminal is not installed. Mirrors the idiom already used in
// adhere_controller/launch/mujoco.launch.py's fsm_console ExecuteProcess block.
void spawnStateConsole(int mode)
{
  killChildConsole();

  std::string title;
  switch (mode) {
    case adhere_control_msgs::msg::FsmCommand::HOLD_PID_DEPTH:
      title = "ROV HOLD_PID_DEPTH Setpoint Console";
      break;
    case adhere_control_msgs::msg::FsmCommand::ADHERE_PID_WBC:
      title = "ROV ADHERE_PID_WBC Setpoint Console";
      break;
    case adhere_control_msgs::msg::FsmCommand::ADHERE_SMC_WBC:
      title = "ROV ADHERE_SMC_WBC Setpoint Console";
      break;
    case adhere_control_msgs::msg::FsmCommand::ADHERE_MPPI_BASELINE:
      title = "ROV ADHERE_MPPI_BASELINE Setpoint Console";
      break;
    default:
      title = "ROV HOLD_PID_6DOF Setpoint Console";
      break;
  }
  const std::string inner_cmd =
    "ros2 run adhere_keyboard_input state_cmd_console_node --ros-args -p mode:=" +
    std::to_string(mode);
  const std::string shell_cmd =
    "if command -v gnome-terminal >/dev/null 2>&1; then "
    "exec gnome-terminal --wait --title='" + title + "' -- " + inner_cmd + "; "
    "elif command -v xterm >/dev/null 2>&1; then "
    "exec xterm -T '" + title + "' -e " + inner_cmd + "; "
    "else echo 'fsm_console: neither gnome-terminal nor xterm found, run \"" + inner_cmd +
    "\" manually' >&2; fi";

  const pid_t pid = fork();
  if (pid == 0) {
    setsid();
    // Second line of defense alongside setsid()+killpg() above: if this process is SIGKILL'd
    // (killChildConsole() can't run), the kernel delivers SIGTERM to the child directly once its
    // parent dies, so the spawned terminal window still closes.
    prctl(PR_SET_PDEATHSIG, SIGTERM);
    execl("/bin/bash", "bash", "-c", shell_cmd.c_str(), nullptr);
    _exit(127);  // execl() only returns on failure
  } else if (pid > 0) {
    g_child_pgid.store(pid);
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
            << "  [2] state_hold_pid_depth         (PID 定深控制，自动弹出深度设定终端)\n"
            << "  [3] state_teleop_openloop_6dof   (Xbox 开环推力映射，B 伪逆分配)\n"
            << "  [4] state_hold_pid_6dof          (6DOF 位置+航向保持，自动弹出设定终端)\n"
            << "  [5] state_adhere_pid_wbc         (贴壁 PID 基线：两阶段靠壁+法向力 PI 保压)\n"
            << "  [6] state_adhere_smc_wbc         (贴壁 SMC 基线：6DOF 滑模控制+法向预压)\n"
            << "  [7] state_adhere_mppi_baseline   (纯 CUDA 外环 MPPI，250Hz 零阶保持)\n"
            << "  [q] Quit Console     (退出控制台)\n"
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
  std::cout << "Select state (1/2/3/4/5/6/7/q): " << std::flush;
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
      {
        std::lock_guard<std::mutex> lock(g_state_name_mutex);
        g_state_name = msg->state_name;
      }
      {
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
      }
      if (g_current_state.exchange(msg->current_state) != msg->current_state) {
        if (msg->current_state == adhere_control_msgs::msg::FsmCommand::HOLD_PID_DEPTH ||
            msg->current_state == adhere_control_msgs::msg::FsmCommand::HOLD_PID_6DOF ||
            msg->current_state == adhere_control_msgs::msg::FsmCommand::ADHERE_PID_WBC ||
            msg->current_state == adhere_control_msgs::msg::FsmCommand::ADHERE_SMC_WBC ||
            msg->current_state == adhere_control_msgs::msg::FsmCommand::ADHERE_MPPI_BASELINE) {
          spawnStateConsole(msg->current_state);
        } else {
          killChildConsole();
        }
      }
    });

  auto runtime_diagnostics_sub =
    node->create_subscription<adhere_control_msgs::msg::RuntimeDiagnostics>(
      "/rov/runtime_diagnostics", 10,
      [](const adhere_control_msgs::msg::RuntimeDiagnostics::SharedPtr msg) {
        std::ostringstream oss;
        oss << "Timing: Inner " << static_cast<int>(msg->inner_loop_hz + 0.5) << "Hz  Outer "
            << static_cast<int>(msg->outer_loop_hz + 0.5) << "Hz  Status: "
            << (msg->timing_healthy ? "HEALTHY" : "UNHEALTHY")
            << " (full detail: ros2 run adhere_keyboard_input rt_monitor_node)";
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
    killChildConsole();
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
      command = adhere_control_msgs::msg::FsmCommand::HOLD_PID_DEPTH;
    } else if (line == "3") {
      command = adhere_control_msgs::msg::FsmCommand::TELEOP_OPENLOOP_6DOF;
    } else if (line == "4") {
      command = adhere_control_msgs::msg::FsmCommand::HOLD_PID_6DOF;
    } else if (line == "5") {
      command = adhere_control_msgs::msg::FsmCommand::ADHERE_PID_WBC;
    } else if (line == "6") {
      command = adhere_control_msgs::msg::FsmCommand::ADHERE_SMC_WBC;
    } else if (line == "7") {
      command = adhere_control_msgs::msg::FsmCommand::ADHERE_MPPI_BASELINE;
    } else {
      std::cout << "Unrecognized selection \"" << line
                << "\", expected 1, 2, 3, 4, 5, 6, 7, or q.\n\n";
      continue;
    }

    adhere_control_msgs::msg::FsmCommand cmd;
    cmd.header.stamp = node->now();
    cmd.command = command;
    fsm_command_pub->publish(cmd);
    std::cout << "-> requested " << stateLabel(command) << "\n\n";
  }

  killChildConsole();
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
