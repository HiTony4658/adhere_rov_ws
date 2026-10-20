#ifndef ADHERE_CONTROLLER__FSM__STATEADHEREPID_H_
#define ADHERE_CONTROLLER__FSM__STATEADHEREPID_H_

#include <cstdint>

#include "adhere_controller/algorithms/pid/AdherePidController.h"
#include "adhere_controller/algorithms/pid/SixDofPidController.h"
#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/dynamics/FossenDynamics.h"
#include "adhere_controller/FSM/FSMState.h"

namespace adhere_controller
{

// Wall-Adhesion PID baseline (CLAUDE.md Phase 3.8 Stage 2.2.1): a classical two-stage
// force/pose hybrid controller, the first of two fair wall-adhesion baselines for the Phase 4
// MPPI comparison (free-swimming StatePidHold cannot serve this role -- it never regulates
// wall-normal preload).
//
// APPROACH phase: while not yet docked (sum of the 4 estimated pad normal forces below
// adhere_pid.dock_force_thresh, or still further than adhere_pid.x_dock_eps from the wall),
// p_d_world_.x() ramps toward adhere_pid.x_wall at adhere_pid.approach_speed and the full 6-DOF
// wrench comes straight from the shared SixDofPidController (ordinary position/attitude hold,
// same law StatePidHold uses).
//
// ADHERED phase (sticky: once docked, this state never un-docks itself -- only an explicit
// /rov/fsm_command state switch leaves it): p_d_world_.x() is re-latched to the current estimate
// every tick (keeps SixDofPidController's X contribution ~0 so it never fights the force loop),
// SixDofPidController still drives Y/Z/attitude, and the dedicated AdherePidController
// overwrites the wrench's Fx component with a wall-normal force PI law (preload + force-error
// P/I - receding-velocity damping, see AdherePidController.h for the exact sign convention).
//
// Setpoint (/rov/setpoint_cmd) and teleop (/rov/teleop_cmd) support mirrors StatePidHold
// (MODE_FULL_POSE/MODE_LOCK_CURRENT/MODE_LEVEL_ATTITUDE, stick-driven Y/Z/attitude advancement);
// the X component of both is simply never meaningful once docked, since p_d_world_.x() is
// re-latched every tick regardless of what was requested.
class StateAdherePid final : public FSMState
{
public:
  StateAdherePid(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params);

  void enter() override;

  void run(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void exit() override;

  FSMStateName checkChange() override;

  // Reports the live hold setpoint: pos=p_d_world_, quat=q_d_.
  PoseTarget targetPose() const override;

  // Exposed for closed-loop gtests (CLAUDE.md Phase 3.8 Stage 3.1): whether the state has
  // transitioned from APPROACH to ADHERED.
  bool isDocked() const { return docked_; }

  // Phase 4.2: whether Stage 1 (DEPTH_ALIGN) has completed and Stage 2 (APPROACH/ADHERED, i.e.
  // WALL_ADHERE) has begun.
  bool isDepthAligned() const { return depth_aligned_; }

  const char * adhereStageName() const override
  {
    return depth_aligned_ ? "WALL_ADHERE" : "DEPTH_ALIGN";
  }

private:
  Mat86 b_pinv_;

  FossenDynamics dynamics_;
  SixDofPidController pid_;
  AdherePidController force_pid_;

  Vec3 p_d_world_ = Vec3::Zero();
  Quat q_d_ = Quat(1.0, 0.0, 0.0, 0.0);

  // Phase 4.2 two-stage adhere sequence: Stage 1 DEPTH_ALIGN (hold standoff_x/target_y/
  // target_depth_m) must complete before Stage 2 WALL_ADHERE (the existing APPROACH/ADHERED
  // ramp-then-dock logic below) begins -- docked_ can only become true once depth_aligned_ is.
  bool depth_aligned_ = false;
  bool docked_ = false;
  uint64_t last_seen_setpoint_seq_ = 0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__FSM__STATEADHEREPID_H_
