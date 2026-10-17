// fsm_console_node: interactive terminal FSM console (CLAUDE.md Phase 3.1 Stage 2.4). Spins
// rclcpp on a background thread (so /rov/odom keeps updating the displayed Z) while the main
// thread blocks on std::cin for the menu, publishing adhere_control_msgs::msg::FsmCommand to
// /rov/fsm_command on each valid selection.
#include <atomic>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "adhere_control_msgs/msg/fsm_command.hpp"

namespace
{
std::atomic<double> g_current_z{0.0};

void printMenu(const std::string & current_state_label)
{
  std::cout << "============================================================\n"
            << "  Underwater ROV FSM Control Console (有限状态机终端控制台)\n"
            << "============================================================\n"
            << "  [1] state_passive    (推进器全零输出 u = 0 N，自由漂流)\n"
            << "  [2] state_depth_hold (PID 定深控制，输入目标深度 Z 坐标)\n"
            << "  [q] Quit Console     (退出控制台)\n"
            << "------------------------------------------------------------\n"
            << "Current State: " << current_state_label
            << " | Current Z: " << g_current_z.load() << " m\n"
            << "Select state (1/2/q): " << std::flush;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fsm_console_node");

  auto fsm_command_pub =
    node->create_publisher<adhere_control_msgs::msg::FsmCommand>("/rov/fsm_command", 10);
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "/rov/odom", 10, [](const nav_msgs::msg::Odometry::SharedPtr msg) {
      g_current_z.store(msg->pose.pose.position.z);
    });

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spin_thread([executor]() { executor->spin(); });

  std::string current_state_label = "[1] state_passive";

  while (rclcpp::ok()) {
    printMenu(current_state_label);

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

    if (line == "1") {
      adhere_control_msgs::msg::FsmCommand cmd;
      cmd.header.stamp = node->now();
      cmd.command = adhere_control_msgs::msg::FsmCommand::PASSIVE;
      fsm_command_pub->publish(cmd);
      current_state_label = "[1] state_passive";
      std::cout << "-> state_passive: all 8 thrusters zeroed (u_i = 0.0 N).\n\n";
      continue;
    }

    if (line == "2") {
      std::cout << "Enter target depth Z in meters (float, e.g. 0.50; water surface is 1.00): "
                << std::flush;
      std::string depth_line;
      if (!std::getline(std::cin, depth_line)) {
        break;
      }
      try {
        const double target_z = std::stod(depth_line);
        adhere_control_msgs::msg::FsmCommand cmd;
        cmd.header.stamp = node->now();
        cmd.command = adhere_control_msgs::msg::FsmCommand::DEPTH_HOLD;
        cmd.target_z = target_z;
        fsm_command_pub->publish(cmd);
        current_state_label = "[2] state_depth_hold (target_z=" + depth_line + "m)";
        std::cout << "-> state_depth_hold: target_z = " << target_z << " m\n\n";
      } catch (const std::exception &) {
        std::cout << "Invalid depth value \"" << depth_line << "\", ignored.\n\n";
      }
      continue;
    }

    std::cout << "Unrecognized selection \"" << line << "\", expected 1, 2, or q.\n\n";
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
