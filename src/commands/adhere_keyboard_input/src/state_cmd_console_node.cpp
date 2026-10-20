// state_cmd_console_node: dedicated HOLD_PID_DEPTH/HOLD_PID_6DOF setpoint terminal (CLAUDE.md
// Phase 3.2 Stage 2.2, quaternion-ized in Phase 3.6). Auto-spawned by fsm_console_node when the
// active state becomes HOLD_PID_DEPTH/HOLD_PID_6DOF, or run manually. Subscribes
// /rov/controller_state for live
// actual/target display and publishes ONLY SetpointCommand on /rov/setpoint_cmd -- never
// FsmCommand (setpoint terminals are orthogonal to state switching). The wire format
// (SetpointCommand::target_pose) is a geometry_msgs/Pose quaternion, but the human prompt stays
// RPY-friendly: this file owns its own small RPY<->quaternion helpers (no Eigen dependency, same
// extrinsic-ZYX/Hamilton(w,x,y,z) convention as adhere_controller's mathTools.h) purely for
// console input parsing and display.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <sys/prctl.h>

#include "rclcpp/rclcpp.hpp"

#include "adhere_control_msgs/msg/controller_state.hpp"
#include "adhere_control_msgs/msg/setpoint_command.hpp"

namespace
{

// Set by handleSignal() on SIGINT/SIGTERM/SIGHUP; the watchdog thread below polls this (and
// rclcpp::ok()) every 100ms so a blocking std::getline(std::cin, ...) (e.g. stdin closed by the
// terminal emulator on window-close, or the parent fsm_console_node's killpg()) never hangs
// shutdown.
std::atomic<bool> g_shutdown_requested{false};

void handleSignal(int /*sig*/)
{
  g_shutdown_requested.store(true);
}

struct Quat
{
  double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
};

// Builds a unit quaternion from extrinsic-ZYX (roll, pitch, yaw) in radians: R = Rz(yaw)*Ry(pitch)*Rx(roll).
Quat quatFromRpy(double roll, double pitch, double yaw)
{
  const double hr = 0.5 * roll, hp = 0.5 * pitch, hy = 0.5 * yaw;
  const double cr = std::cos(hr), sr = std::sin(hr);
  const double cp = std::cos(hp), sp = std::sin(hp);
  const double cy = std::cos(hy), sy = std::sin(hy);
  Quat q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

// Inverse of quatFromRpy() above: extracts (roll, pitch, yaw) in radians from a unit quaternion.
std::array<double, 3> rpyFromQuat(const Quat & q)
{
  const double r00 = 1 - 2 * (q.y * q.y + q.z * q.z);
  const double r10 = 2 * (q.x * q.y + q.w * q.z);
  const double r20 = -2 * (q.x * q.z - q.w * q.y);
  const double r21 = 2 * (q.y * q.z + q.w * q.x);
  const double r22 = 1 - 2 * (q.x * q.x + q.y * q.y);
  return {std::atan2(r21, r22), std::atan2(r20, std::sqrt(r00 * r00 + r10 * r10)), std::atan2(r10, r00)};
}

std::mutex g_pose_mutex;
// Position [x, y, z] (m) + orientation quaternion [w, x, y, z] (Phase 3.6: replaces the old
// Euler float64[6]).
struct PoseDisplay
{
  std::array<double, 3> pos{0.0, 0.0, 0.0};
  Quat quat{};
};
PoseDisplay g_target_pose;
// "Actual" display uses ControllerState::estimated_pose -- what the controller itself tracks
// (ground truth in "ground_truth" mode, RovStateEstimator's fused output in "sensor_fusion" mode).
PoseDisplay g_actual_pose;

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
  PoseDisplay target, actual;
  {
    std::lock_guard<std::mutex> lock(g_pose_mutex);
    target = g_target_pose;
    actual = g_actual_pose;
  }
  // CLAUDE.md Phase 3.3 tank geometry: water surface at world Z=0.0m, floor at Z=-10.0m, so
  // depth below surface = -Z. Displayed alongside raw Z for unambiguous cross-reference.
  std::cout << "============================================================\n"
            << "  [HOLD_PID_DEPTH] Setpoint Console (定深设定终端)\n"
            << "============================================================\n"
            << "  Current Depth: " << -actual.pos[2] << " m (Z = " << actual.pos[2] << " m)   |   "
            << "Target Depth: " << -target.pos[2] << " m (Z = " << target.pos[2] << " m)\n"
            << "------------------------------------------------------------\n"
            << "[HOLD_PID_DEPTH] Enter target depth in meters, positive or negative (float, e.g. "
               "2.50 or -2.50 -- both mean 2.50m below the surface), or 'h' to hold current "
               "depth: " << std::flush;
}

// mode 4=HOLD_PID_6DOF, 5=ADHERE_PID_WBC, 6=ADHERE_SMC_WBC, 7=ADHERE_MPPI_BASELINE -- all four
// share the exact same command grammar below (y/z/roll/pitch/yaw/all/reset/h), since sliding
// along the wall in Y/Z/attitude while ADHERE_PID_WBC/ADHERE_SMC_WBC maintain wall-normal preload
// (or ADHERE_MPPI_BASELINE tracks a free 6DOF reference via its CUDA rollout) is identical in
// shape to HOLD_PID_6DOF's free 6-DOF setpoint move -- the FSM states themselves simply ignore
// the X component of target_pose.position once docked (CLAUDE.md Phase 3.8/4.1).
void printPidHoldPrompt(int mode)
{
  PoseDisplay target, actual;
  {
    std::lock_guard<std::mutex> lock(g_pose_mutex);
    target = g_target_pose;
    actual = g_actual_pose;
  }
  const auto actual_rpy = rpyFromQuat(actual.quat);
  const auto target_rpy = rpyFromQuat(target.quat);
  const char * title = mode == 5 ? "[ADHERE_PID_WBC] Setpoint Console (贴壁 PID 基线设定终端)"
    : mode == 6 ? "[ADHERE_SMC_WBC] Setpoint Console (贴壁 SMC 基线设定终端)"
    : mode == 7 ? "[ADHERE_MPPI_BASELINE] Setpoint Console (纯 CUDA 外环 MPPI 设定终端)"
    : "[HOLD_PID_6DOF] Setpoint Console (6DOF 位置+姿态保持设定终端)";
  // CLAUDE.md Phase 3.3 tank geometry: water surface at world Z=0.0m, so depth = -Z. Phase 3.6:
  // displays both the wire-format quaternion and the human-friendly RPY(deg) derived from it.
  std::cout << "============================================================\n"
            << "  " << title << "\n"
            << "============================================================\n"
            << "  Actual  [X, Y, Z] = [" << actual.pos[0] << ", " << actual.pos[1] << ", "
            << actual.pos[2] << "] m  (Depth: " << -actual.pos[2] << " m)\n"
            << "  Actual  Quat [qw,qx,qy,qz] = [" << actual.quat.w << ", " << actual.quat.x << ", "
            << actual.quat.y << ", " << actual.quat.z << "]  RPY(deg) = [" << actual_rpy[0] * kRadToDeg
            << ", " << actual_rpy[1] * kRadToDeg << ", " << actual_rpy[2] * kRadToDeg << "]\n"
            << "  Target  [X, Y, Z] = [" << target.pos[0] << ", " << target.pos[1] << ", "
            << target.pos[2] << "] m  (Depth: " << -target.pos[2] << " m)\n"
            << "  Target  Quat [qw,qx,qy,qz] = [" << target.quat.w << ", " << target.quat.x << ", "
            << target.quat.y << ", " << target.quat.z << "]  RPY(deg) = [" << target_rpy[0] * kRadToDeg
            << ", " << target_rpy[1] * kRadToDeg << ", " << target_rpy[2] * kRadToDeg << "]\n"
            << "------------------------------------------------------------\n"
            << "Enter 'z <val>' (depth), 'y <val>' (lateral), 'x <val>' (wall dist, ignored "
               "once ADHERE_PID_WBC/ADHERE_SMC_WBC have docked), 'roll <deg>', 'pitch <deg>', "
               "'yaw <deg>', 'all <x> <y> <z> <yaw_deg>', 'reset' (roll/pitch -> 0), or 'h' "
               "(lock current pose): "
            << std::flush;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Self-terminate if the parent fsm_console_node dies without delivering a signal (e.g.
  // SIGKILL'd, bypassing its killpg() cleanup path).
  prctl(PR_SET_PDEATHSIG, SIGTERM);
  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);
  std::signal(SIGHUP, handleSignal);

  auto node = std::make_shared<rclcpp::Node>("state_cmd_console_node");

  const int mode = node->declare_parameter<int>("mode", 2);
  if (mode != 2 && mode != 4 && mode != 5 && mode != 6 && mode != 7) {
    std::cerr << "state_cmd_console_node: unsupported mode " << mode
               << " (expected 2, 4, 5, 6, or 7)\n";
    rclcpp::shutdown();
    return 1;
  }

  auto setpoint_pub = node->create_publisher<adhere_control_msgs::msg::SetpointCommand>(
    "/rov/setpoint_cmd", 10);
  auto controller_state_sub = node->create_subscription<adhere_control_msgs::msg::ControllerState>(
    "/rov/controller_state", 10,
    [](const adhere_control_msgs::msg::ControllerState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_pose_mutex);
      g_target_pose.pos = {
        msg->target_pose.position.x, msg->target_pose.position.y, msg->target_pose.position.z};
      g_target_pose.quat = {
        msg->target_pose.orientation.w, msg->target_pose.orientation.x,
        msg->target_pose.orientation.y, msg->target_pose.orientation.z};
      g_actual_pose.pos = {
        msg->estimated_pose.position.x, msg->estimated_pose.position.y,
        msg->estimated_pose.position.z};
      g_actual_pose.quat = {
        msg->estimated_pose.orientation.w, msg->estimated_pose.orientation.x,
        msg->estimated_pose.orientation.y, msg->estimated_pose.orientation.z};
    });

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
    if (mode == 2) {
      printDepthHoldPrompt();
    } else {
      printPidHoldPrompt(mode);
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
    cmd.target_pose.orientation.w = 1.0;  // identity default, overwritten below where relevant

    bool valid = true;
    if (mode == 2) {
      if (line == "h" || line == "H") {
        cmd.command_type = adhere_control_msgs::msg::SetpointCommand::MODE_LOCK_CURRENT;
      } else {
        try {
          cmd.command_type = adhere_control_msgs::msg::SetpointCommand::MODE_DEPTH_ONLY;
          cmd.target_pose.position.z = depthInputToTargetZ(std::stod(line));
        } catch (const std::exception &) {
          valid = false;
        }
      }
    } else {
      // mode == 4/5/6/7 (HOLD_PID_6DOF/ADHERE_PID_WBC/ADHERE_SMC_WBC/ADHERE_MPPI_BASELINE): parse 'z <val>' / 'y <val>' /
      // 'x <val>' / 'roll <deg>' / 'pitch <deg>' / 'yaw <deg>' /
      // 'all <x> <y> <z> <yaw_deg>' / 'reset' / 'h'. Partial updates carry the
      // CURRENTLY known target pose forward (decomposed to x/y/z/roll/pitch/yaw), so e.g.
      // 'z 0.4' never silently resets x/y/roll/pitch/yaw to 0.
      PoseDisplay target;
      {
        std::lock_guard<std::mutex> lock(g_pose_mutex);
        target = g_target_pose;
      }
      double x = target.pos[0], y = target.pos[1], z = target.pos[2];
      auto rpy = rpyFromQuat(target.quat);
      double roll = rpy[0], pitch = rpy[1], yaw = rpy[2];

      if (line == "h" || line == "H") {
        cmd.command_type = adhere_control_msgs::msg::SetpointCommand::MODE_LOCK_CURRENT;
      } else if (line == "reset" || line == "RESET") {
        // Server-side re-derives this from its own live target, keeping yaw & position --
        // no payload fields are consulted for this command_type.
        cmd.command_type = adhere_control_msgs::msg::SetpointCommand::MODE_LEVEL_ATTITUDE;
      } else {
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        try {
          if (token == "z") {
            iss >> z;
            z = depthInputToTargetZ(z);
          } else if (token == "y") {
            iss >> y;
          } else if (token == "x") {
            iss >> x;
          } else if (token == "roll") {
            double v_deg;
            iss >> v_deg;
            roll = v_deg * kDegToRad;
          } else if (token == "pitch") {
            double v_deg;
            iss >> v_deg;
            pitch = v_deg * kDegToRad;
          } else if (token == "yaw") {
            double v_deg;
            iss >> v_deg;
            yaw = v_deg * kDegToRad;
          } else if (token == "all") {
            double yaw_deg;
            iss >> x >> y >> z >> yaw_deg;
            z = depthInputToTargetZ(z);
            yaw = yaw_deg * kDegToRad;
          } else {
            valid = false;
          }
          if (iss.fail()) {
            valid = false;
          }
        } catch (const std::exception &) {
          valid = false;
        }

        if (valid) {
          cmd.command_type = adhere_control_msgs::msg::SetpointCommand::MODE_FULL_POSE;
          cmd.target_pose.position.x = x;
          cmd.target_pose.position.y = y;
          cmd.target_pose.position.z = z;
          const Quat q = quatFromRpy(roll, pitch, yaw);
          cmd.target_pose.orientation.w = q.w;
          cmd.target_pose.orientation.x = q.x;
          cmd.target_pose.orientation.y = q.y;
          cmd.target_pose.orientation.z = q.z;
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
