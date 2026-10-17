#ifndef ADHERE_CONTROLLER_COMMON__CTRLINTERFACES_H_
#define ADHERE_CONTROLLER_COMMON__CTRLINTERFACES_H_

#include <atomic>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace adhere_controller_common
{

// Live FSM target set by /rov/fsm_command (AdhereKamrController's subscription callback, which
// runs on the executor thread) and read every tick by the active FSMState::run() (which runs on
// the 500Hz physics thread). Plain std::atomic<T> members instead of a mutex: command/target_z/
// target_y/target_yaw are each a single scalar, so lock-free read-modify-write is sufficient and
// avoids blocking the 500Hz hot loop on the (much slower) ROS executor thread.
struct ControlInputs
{
  std::atomic<int> command{0};          // 0 = NO_CHANGE, 1 = PASSIVE, 2 = DEPTH_HOLD
  std::atomic<double> target_z{0.5};    // m, world frame
  std::atomic<double> target_y{0.0};    // m, world frame (reserved)
  std::atomic<double> target_yaw{0.0};  // rad (reserved)
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
