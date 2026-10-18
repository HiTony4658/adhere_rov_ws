// state_cmd_console_node: dedicated DEPTH_HOLD/PID_HOLD setpoint terminal (CLAUDE.md Phase 3.2
// Stage 2.2). Auto-spawned by fsm_console_node when the active state becomes DEPTH_HOLD/
// PID_HOLD, or run manually. Subscribes /rov/fsm_state for live actual/target display and
// publishes ONLY SetpointCommand on /rov/setpoint_cmd -- never FsmCommand (setpoint terminals
// are orthogonal to state switching).
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "adhere_control_msgs/msg/fsm_state.hpp"
#include "adhere_control_msgs/msg/setpoint_command.hpp"

namespace
{

std::mutex g_pose_mutex;
// [x, y, z, roll(rad), pitch(rad), yaw(rad)] (Phase 3.5: extended from 4 to 6 DOF).
std::array<double, 6> g_target_pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// "Actual" display uses FsmState::estimated_pose -- what the controller itself tracks (ground
// truth in "ground_truth" mode, RovStateEstimator's fused output in "sensor_fusion" mode).
std::array<double, 6> g_actual_pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;

// CLAUDE.md Phase 3.3 tank geometry: water surface at world Z=0.0m, floor at Z=-10.0m.
// Mirrors adhere_controller's robot_control.yaml workspace_z_min/workspace_z_max.
constexpr double kWorkspaceZMin = -9.80;
constexpr double kWorkspaceZMax = -0.15;

// Accepts either a positive depth (2.50 -> 2.50m below the surface) or the raw negative world
// Z (-2.50) -- both normalize to the same target_z, clamped into the safe workspace band.
double depthInputToTargetZ(double input_val)
{
  return std::clamp(-std::abs(input_val), kWorkspaceZMin, kWorkspaceZMax);
}

void printDepthHoldPrompt()
{
  std::array<double, 6> target, actual;
  {
    std::lock_guard<std::mutex> lock(g_pose_mutex);
    target = g_target_pose;
    actual = g_actual_pose;
  }
  // CLAUDE.md Phase 3.3 tank geometry: water surface at world Z=0.0m, floor at Z=-10.0m, so
  // depth below surface = -Z. Displayed alongside raw Z for unambiguous cross-reference.
  std::cout << "============================================================\n"
            << "  [DEPTH_HOLD] Setpoint Console (定深设定终端)\n"
            << "============================================================\n"
            << "  Current Depth: " << -actual[2] << " m (Z = " << actual[2] << " m)   |   "
            << "Target Depth: " << -target[2] << " m (Z = " << target[2] << " m)\n"
            << "------------------------------------------------------------\n"
            << "[DEPTH_HOLD] Enter target depth in meters, positive or negative (float, e.g. "
               "2.50 or -2.50 -- both mean 2.50m below the surface), or 'h' to hold current "
               "depth: " << std::flush;
}

void printPidHoldPrompt()
{
  std::array<double, 6> target, actual;
  {
    std::lock_guard<std::mutex> lock(g_pose_mutex);
    target = g_target_pose;
    actual = g_actual_pose;
  }
  // CLAUDE.md Phase 3.3 tank geometry: water surface at world Z=0.0m, so depth = -Z.
  std::cout << "============================================================\n"
            << "  [PID_HOLD] Setpoint Console (6DOF 位置+姿态保持设定终端)\n"
            << "============================================================\n"
            << "  Actual [X, Y, Z, Roll, Pitch, Yaw]: [" << actual[0] << ", " << actual[1] << ", "
            << actual[2] << ", " << actual[3] * kRadToDeg << ", " << actual[4] * kRadToDeg << ", "
            << actual[5] * kRadToDeg << " deg]  (Depth: " << -actual[2] << " m)\n"
            << "  Target [X, Y, Z, Roll, Pitch, Yaw]: [" << target[0] << ", " << target[1] << ", "
            << target[2] << ", " << target[3] * kRadToDeg << ", " << target[4] * kRadToDeg << ", "
            << target[5] * kRadToDeg << " deg]  (Depth: " << -target[2] << " m)\n"
            << "------------------------------------------------------------\n"
            << "[PID_HOLD] Enter 'z <val>' (depth), 'y <val>' (lateral), 'x <val>' (wall dist), "
               "'roll <deg>', 'pitch <deg>', 'yaw <deg>', 'all <x> <y> <z> <yaw_deg>', "
               "'reset' (roll/pitch -> 0), or 'h' (lock current pose): "
            << std::flush;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("state_cmd_console_node");

  const int mode = node->declare_parameter<int>("mode", 2);
  if (mode != 2 && mode != 4) {
    std::cerr << "state_cmd_console_node: unsupported mode " << mode << " (expected 2 or 4)\n";
    rclcpp::shutdown();
    return 1;
  }

  auto setpoint_pub = node->create_publisher<adhere_control_msgs::msg::SetpointCommand>(
    "/rov/setpoint_cmd", 10);
  auto fsm_state_sub = node->create_subscription<adhere_control_msgs::msg::FsmState>(
    "/rov/fsm_state", 10, [](const adhere_control_msgs::msg::FsmState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_pose_mutex);
      std::copy(msg->target_pose.begin(), msg->target_pose.end(), g_target_pose.begin());
      std::copy(msg->estimated_pose.begin(), msg->estimated_pose.end(), g_actual_pose.begin());
    });

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spin_thread([executor]() { executor->spin(); });

  while (rclcpp::ok()) {
    if (mode == 2) {
      printDepthHoldPrompt();
    } else {
      printPidHoldPrompt();
    }

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

    adhere_control_msgs::msg::SetpointCommand cmd;
    cmd.header.stamp = node->now();
    cmd.mode = static_cast<uint8_t>(mode);

    bool valid = true;
    if (mode == 2) {
      if (line == "h" || line == "H") {
        cmd.lock_current_pose = true;
      } else {
        try {
          cmd.target_z = depthInputToTargetZ(std::stod(line));
        } catch (const std::exception &) {
          valid = false;
        }
      }
    } else {
      // mode == 4: parse 'z <val>' / 'y <val>' / 'x <val>' / 'roll <deg>' / 'pitch <deg>' /
      // 'yaw <deg>' / 'all <x> <y> <z> <yaw_deg>' / 'reset' / 'h'. Partial updates carry the
      // CURRENTLY known target_pose for the other axes forward, so e.g. 'z 0.4' never silently
      // resets x/y/roll/pitch/yaw to 0.
      std::array<double, 6> target;
      {
        std::lock_guard<std::mutex> lock(g_pose_mutex);
        target = g_target_pose;
      }
      cmd.target_x = target[0];
      cmd.target_y = target[1];
      cmd.target_z = target[2];
      cmd.target_roll = target[3];
      cmd.target_pitch = target[4];
      cmd.target_yaw = target[5];

      if (line == "h" || line == "H") {
        cmd.lock_current_pose = true;
      } else if (line == "reset" || line == "RESET") {
        // Phase 3.5: reset roll/pitch to 0 while carrying x/y/z/yaw forward unchanged.
        cmd.target_roll = 0.0;
        cmd.target_pitch = 0.0;
      } else {
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        try {
          if (token == "z") {
            double v;
            iss >> v;
            cmd.target_z = depthInputToTargetZ(v);
          } else if (token == "y") {
            double v;
            iss >> v;
            cmd.target_y = v;
          } else if (token == "x") {
            double v;
            iss >> v;
            cmd.target_x = v;
          } else if (token == "roll") {
            double v_deg;
            iss >> v_deg;
            cmd.target_roll = v_deg * kDegToRad;
          } else if (token == "pitch") {
            double v_deg;
            iss >> v_deg;
            cmd.target_pitch = v_deg * kDegToRad;
          } else if (token == "yaw") {
            double v_deg;
            iss >> v_deg;
            cmd.target_yaw = v_deg * kDegToRad;
          } else if (token == "all") {
            double x, y, z, yaw_deg;
            iss >> x >> y >> z >> yaw_deg;
            cmd.target_x = x;
            cmd.target_y = y;
            cmd.target_z = depthInputToTargetZ(z);
            cmd.target_yaw = yaw_deg * kDegToRad;
          } else {
            valid = false;
          }
          if (iss.fail()) {
            valid = false;
          }
        } catch (const std::exception &) {
          valid = false;
        }
      }
    }

    if (!valid) {
      std::cout << "Invalid input \"" << line << "\", ignored.\n\n";
      continue;
    }

    setpoint_pub->publish(cmd);
    std::cout << "-> setpoint published.\n\n";
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
