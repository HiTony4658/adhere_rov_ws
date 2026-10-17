// fsm_console_node: master FSM state-switcher terminal (CLAUDE.md Phase 3.2 Stage 2.2).
// Subscribes ONLY to /rov/fsm_state (never raw hardware/odom topics) and publishes ONLY
// FsmCommand on /rov/fsm_command -- all state-specific numeric setpoints live on
// state_cmd_console_node instead. Dynamically spawns/kills that per-state console as a
// dedicated gnome-terminal/xterm window whenever the ACTIVE state changes, regardless of
// whether the switch came from this console's own keyboard input or an Xbox gamepad button
// (the /rov/fsm_state subscription is the only source of truth for "what state is active").
#include <array>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <signal.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "adhere_control_msgs/msg/fsm_command.hpp"
#include "adhere_control_msgs/msg/fsm_state.hpp"

namespace
{

std::atomic<int32_t> g_current_state{adhere_control_msgs::msg::FsmCommand::PASSIVE};
std::string g_state_name = "state_passive";
std::mutex g_state_name_mutex;

// setsid() in spawnStateConsole() below puts the forked child in its own session, so its pid
// IS its process group id -- killpg() on that pid tears down gnome-terminal/xterm and the
// inner ros2 run process together.
std::atomic<pid_t> g_child_pgid{-1};

const char * stateLabel(int32_t state)
{
  switch (state) {
    case adhere_control_msgs::msg::FsmCommand::PASSIVE:
      return "[1] state_passive";
    case adhere_control_msgs::msg::FsmCommand::DEPTH_HOLD:
      return "[2] state_depth_hold";
    case adhere_control_msgs::msg::FsmCommand::OPEN_LOOP:
      return "[3] state_open_loop";
    case adhere_control_msgs::msg::FsmCommand::PID_HOLD:
      return "[4] state_pid_hold";
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

  const std::string title =
    (mode == adhere_control_msgs::msg::FsmCommand::DEPTH_HOLD) ? "ROV DEPTH_HOLD Setpoint Console"
                                                                : "ROV PID_HOLD Setpoint Console";
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
            << "  [1] state_passive    (推进器全零输出 u = 0 N，自由漂流)\n"
            << "  [2] state_depth_hold (PID 定深控制，自动弹出深度设定终端)\n"
            << "  [3] state_open_loop  (Xbox 开环推力映射，B 伪逆分配)\n"
            << "  [4] state_pid_hold   (6DOF 位置+航向保持，自动弹出设定终端)\n"
            << "  [q] Quit Console     (退出控制台)\n"
            << "------------------------------------------------------------\n"
            << "Current State: " << stateLabel(g_current_state.load()) << " (" << state_name
            << ")\n"
            << "Select state (1/2/3/4/q): " << std::flush;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fsm_console_node");

  auto fsm_command_pub =
    node->create_publisher<adhere_control_msgs::msg::FsmCommand>("/rov/fsm_command", 10);

  auto fsm_state_sub = node->create_subscription<adhere_control_msgs::msg::FsmState>(
    "/rov/fsm_state", 10, [](const adhere_control_msgs::msg::FsmState::SharedPtr msg) {
      {
        std::lock_guard<std::mutex> lock(g_state_name_mutex);
        g_state_name = msg->state_name;
      }
      if (g_current_state.exchange(msg->current_state) != msg->current_state) {
        if (msg->current_state == adhere_control_msgs::msg::FsmCommand::DEPTH_HOLD ||
            msg->current_state == adhere_control_msgs::msg::FsmCommand::PID_HOLD) {
          spawnStateConsole(msg->current_state);
        } else {
          killChildConsole();
        }
      }
    });

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spin_thread([executor]() { executor->spin(); });

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
      command = adhere_control_msgs::msg::FsmCommand::PASSIVE;
    } else if (line == "2") {
      command = adhere_control_msgs::msg::FsmCommand::DEPTH_HOLD;
    } else if (line == "3") {
      command = adhere_control_msgs::msg::FsmCommand::OPEN_LOOP;
    } else if (line == "4") {
      command = adhere_control_msgs::msg::FsmCommand::PID_HOLD;
    } else {
      std::cout << "Unrecognized selection \"" << line << "\", expected 1, 2, 3, 4, or q.\n\n";
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
