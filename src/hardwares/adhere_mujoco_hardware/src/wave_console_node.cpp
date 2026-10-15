// wave_console_node: interactive terminal "wave-maker" console. Publishes 1 (Calm Water) /
// 2 (Wave Gust) selections on /rov/wave_mode_cmd for adhere_mujoco_node to forward to the
// live AdhereMujocoHardware instance (CLAUDE.md Phase 2.5 Stage 2.3).
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace
{
const char * modeLabel(int mode) { return mode == 2 ? "[2] Wave Gust" : "[1] Calm Water"; }
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("wave_console_node");
  auto pub = node->create_publisher<std_msgs::msg::Int32>("/rov/wave_mode_cmd", 10);

  std::cout
    << "========================================================\n"
    << "  Underwater ROV Wave Environment Console (造浪机控制台)\n"
    << "========================================================\n"
    << "  [1] Calm Water Environment (静水环境 - 默认/停止造浪)\n"
    << "  [2] Wave Gust Scenario     (开启造浪: 0.3m/s横流 + 双阵风冲击)\n"
    << "  [q] Quit Console           (退出控制台)\n"
    << "--------------------------------------------------------\n";

  int current_mode = 1;
  while (rclcpp::ok()) {
    std::cout << "Current Mode: " << modeLabel(current_mode)
               << " | Enter selection (1/2/q): " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) {
      break;
    }
    if (line == "1" || line == "2") {
      current_mode = std::stoi(line);
      std_msgs::msg::Int32 msg;
      msg.data = current_mode;
      pub->publish(msg);
      std::cout << "-> Wave mode set to " << modeLabel(current_mode) << "\n";
    } else if (line == "q" || line == "Q") {
      break;
    } else if (!line.empty()) {
      std::cout << "Invalid selection '" << line << "'. Please enter 1, 2, or q.\n";
    }
  }

  rclcpp::shutdown();
  return 0;
}
