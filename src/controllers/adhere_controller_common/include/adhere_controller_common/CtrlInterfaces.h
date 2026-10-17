#ifndef ADHERE_CONTROLLER_COMMON__CTRLINTERFACES_H_
#define ADHERE_CONTROLLER_COMMON__CTRLINTERFACES_H_

#include <atomic>
#include <cstdint>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace adhere_controller_common
{

// Live FSM inputs set by AdhereKamrController's /rov/fsm_command, /rov/setpoint_cmd, and
// /rov/teleop_cmd subscription callbacks (which run on the executor thread) and read every tick
// by the active FSMState::run() (which runs on the 500Hz physics thread). Plain std::atomic<T>
// members instead of a mutex: every field here is a single scalar, so lock-free
// read-modify-write is sufficient and avoids blocking the 500Hz hot loop on the (much slower)
// ROS executor thread (CLAUDE.md Phase 3.2 Stage 2.1's 4-topic decoupled contract).
struct ControlInputs
{
  // /rov/fsm_command (FsmCommand.msg): ONLY switches FSM states.
  std::atomic<int> command{0};  // 0=NO_CHANGE, 1=PASSIVE, 2=DEPTH_HOLD, 3=OPEN_LOOP, 4=PID_HOLD

  // /rov/setpoint_cmd (SetpointCommand.msg): discrete numeric setpoints, orthogonal to state
  // switching. setpoint_seq increments once per received message so StateDepthHold/StatePidHold
  // can edge-detect "a new setpoint just arrived" instead of re-applying the same message every
  // tick (which would fight the Xbox left-stick continuous nudge/move integration).
  std::atomic<uint64_t> setpoint_seq{0};
  std::atomic<uint8_t> setpoint_mode{0};         // 2 = DEPTH_HOLD, 4 = PID_HOLD
  std::atomic<bool> setpoint_lock_current_pose{false};
  std::atomic<double> setpoint_target_x{0.0};    // m, world frame
  std::atomic<double> setpoint_target_y{0.0};    // m, world frame
  std::atomic<double> setpoint_target_z{0.5};    // m, world frame
  std::atomic<double> setpoint_target_yaw{0.0};  // rad

  // /rov/teleop_cmd (TeleopCommand.msg): continuous Mode-2 (美国手) joystick inputs in
  // [-1.0, 1.0]. last_rx_time_sec is wall-clock seconds at last receipt, stamped by
  // AdhereKamrController's subscription callback and read by its own 0.25s watchdog (which
  // forces teleop_active/the 4 axes to false/0.0 if the joystick has gone stale -- so FSMState
  // implementations never need to know about watchdog timeouts themselves).
  std::atomic<double> teleop_surge_x{0.0};
  std::atomic<double> teleop_sway_y{0.0};
  std::atomic<double> teleop_heave_z{0.0};
  std::atomic<double> teleop_yaw_rate{0.0};
  std::atomic<bool> teleop_active{false};
  std::atomic<double> teleop_last_rx_time_sec{0.0};
};

// Zero-copy FSM/hardware bridge, modeled after quadruped_ros2_control's CtrlInterfaces: holds
// LoanedCommandInterface/LoanedStateInterface references bound by AdhereKamrController's
// on_activate(), read/written directly by FSMState::run() implementations with no DDS
// round-trip. Slot order within each vector matches adhere_mujoco_hardware's documented layout
// (CLAUDE.md Sec.3/6): thrusters as thr_N1..thr_N4, thr_T5..thr_T8; imu as orientation.w/x/y/z,
// angular_velocity.x/y/z, linear_acceleration.x/y/z; odom as pos.x/y/z, quat.w/x/y/z,
// vel_body.x/y/z; pad_contact as force TL/TR/BL/BR then v_sep TL/TR/BL/BR.
struct CtrlInterfaces
{
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    thruster_command_interface_;  // size 8

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    thruster_state_interface_;  // size 8
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    imu_state_interface_;  // size 10
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    odom_state_interface_;  // size 10
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    pad_contact_state_interface_;  // size 8

  ControlInputs control_inputs_;
  int frequency_ = 500;

  CtrlInterfaces() = default;

  void clear()
  {
    thruster_command_interface_.clear();
    thruster_state_interface_.clear();
    imu_state_interface_.clear();
    odom_state_interface_.clear();
    pad_contact_state_interface_.clear();
  }
};

}  // namespace adhere_controller_common

#endif  // ADHERE_CONTROLLER_COMMON__CTRLINTERFACES_H_
