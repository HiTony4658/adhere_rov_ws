#ifndef ADHERE_CONTROLLER__CTRLINTERFACES_H_
#define ADHERE_CONTROLLER__CTRLINTERFACES_H_

#include <atomic>
#include <cstdint>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace adhere_controller
{

// Live FSM inputs set by AdhereController's /rov/fsm_command, /rov/setpoint_cmd, and
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
  // tick (which would fight the Xbox left-stick continuous nudge/move integration). Phase 3.6:
  // target pose is now carried as position + unit quaternion (matching SetpointCommand.msg's
  // geometry_msgs/Pose target_pose), replacing the old discrete x/y/z/roll/pitch/yaw scalars.
  std::atomic<uint64_t> setpoint_seq{0};
  // 1=MODE_DEPTH_ONLY, 2=MODE_FULL_POSE, 3=MODE_LOCK_CURRENT, 4=MODE_LEVEL_ATTITUDE
  std::atomic<uint8_t> setpoint_command_type{0};
  std::atomic<double> setpoint_target_pos_x{0.0};   // m, world frame
  std::atomic<double> setpoint_target_pos_y{0.0};   // m, world frame
  std::atomic<double> setpoint_target_pos_z{0.5};   // m, world frame
  std::atomic<double> setpoint_target_quat_w{1.0};  // world-frame orientation, Hamilton (w,x,y,z)
  std::atomic<double> setpoint_target_quat_x{0.0};
  std::atomic<double> setpoint_target_quat_y{0.0};
  std::atomic<double> setpoint_target_quat_z{0.0};

  // /rov/teleop_cmd (TeleopCommand.msg): continuous Mode-2 (美国手) joystick inputs in
  // [-1.0, 1.0]. Phase 3.6: the message's fields are now geometry_msgs/Vector3 linear/angular
  // instead of 6 bare scalars, but the atomics here keep their original semantic names (fed
  // from msg->linear.{x,y,z}/msg->angular.{x,y,z} in the subscription callback) so FSMState
  // code reading e.g. teleop_surge_x is unaffected. last_rx_time_sec is wall-clock seconds at
  // last receipt, stamped by AdhereController's subscription callback and read by its own
  // 0.25s watchdog (which forces teleop_active/the 6 axes to false/0.0 if the joystick has gone
  // stale -- so FSMState implementations never need to know about watchdog timeouts themselves).
  std::atomic<double> teleop_surge_x{0.0};
  std::atomic<double> teleop_sway_y{0.0};
  std::atomic<double> teleop_heave_z{0.0};
  std::atomic<double> teleop_yaw_rate{0.0};
  std::atomic<double> teleop_pitch_rate{0.0};  // Phase 3.5: Xbox LT/LB
  std::atomic<double> teleop_roll_rate{0.0};   // Phase 3.5: Xbox RT/RB
  std::atomic<bool> teleop_active{false};
  std::atomic<double> teleop_last_rx_time_sec{0.0};
};

// Phase 3.5 strict ground-truth isolation (CLAUDE.md Stage 2.3): the ONLY pose/velocity/
// contact data any FSMState may read. Populated once per 500Hz tick by AdhereController,
// either copied directly from ground-truth hardware state interfaces ("ground_truth" mode) or
// from RovStateEstimator's fused output ("sensor_fusion" mode) -- FSMState has no way to tell
// which, and no access to the raw ground-truth hardware interfaces at all.
struct ActiveState
{
  double pos_w[3] = {0.0, 0.0, 0.0};        // world-frame CoM position [x, y, z]
  double quat_w[4] = {1.0, 0.0, 0.0, 0.0};  // world-frame orientation, Hamilton (w, x, y, z) (Phase 3.6)
  double rpy[3] = {0.0, 0.0, 0.0};          // [roll, pitch, yaw], rad -- derived from quat_w, kept
                                             // for StateDepthHold's small-angle Euler PD damping
  double vel_body[3] = {0.0, 0.0, 0.0};     // body-frame linear velocity
  double omega_body[3] = {0.0, 0.0, 0.0};   // body-frame angular velocity
  double pad_force[4] = {0.0, 0.0, 0.0, 0.0};  // TL, TR, BL, BR normal contact force
  double v_sep[4] = {0.0, 0.0, 0.0, 0.0};      // TL, TR, BL, BR separation velocity
};

// Zero-copy FSM/hardware bridge, modeled after quadruped_ros2_control's CtrlInterfaces: holds
// LoanedCommandInterface references bound by AdhereController's on_activate(), written
// directly by FSMState::run() implementations with no DDS round-trip. Slot order within
// thruster_command_interface_ matches adhere_mujoco_hardware's documented layout (CLAUDE.md
// Sec.3/6): thr_N1..thr_N4, thr_T5..thr_T8. Ground-truth hardware state interfaces
// (imu_state_interface_/odom_state_interface_/pad_contact_state_interface_) are deliberately
// NOT members of this struct -- they are bound as private AdhereController members instead
// (see adhere_controller.hpp) and never handed to any FSMState; active_state below is the sole
// read path.
struct CtrlInterfaces
{
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    thruster_command_interface_;  // size 8

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    thruster_state_interface_;  // size 8 (effort readback only, not pose/velocity -- harmless)

  ActiveState active_state;

  ControlInputs control_inputs_;
  int frequency_ = 500;

  CtrlInterfaces() = default;

  void clear()
  {
    thruster_command_interface_.clear();
    thruster_state_interface_.clear();
  }
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__CTRLINTERFACES_H_
