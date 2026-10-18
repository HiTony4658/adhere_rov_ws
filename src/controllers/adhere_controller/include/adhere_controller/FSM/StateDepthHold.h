#ifndef ADHERE_CONTROLLER__FSM__STATEDEPTHHOLD_H_
#define ADHERE_CONTROLLER__FSM__STATEDEPTHHOLD_H_

#include <array>
#include <cstdint>

#include "adhere_controller/algorithms/pid/SixDofPidController.h"
#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/dynamics/FossenDynamics.h"
#include "adhere_controller/FSM/FSMState.h"

namespace adhere_controller
{

// Closed-loop PID depth (world Z) controller (CLAUDE.md Phase 3.1/3.2 Stage 2.2). Tracks a
// local target_z_, retargeted either discretely (a new SetpointCommand with mode==2, edge-
// detected via setpoint_seq so the same message is never re-applied every tick) or continuously
// (the Xbox left-stick heave_z nudge, integrated every tick and left latched when the stick
// returns to center -- CLAUDE.md Phase 3.2 Stage 2.2.2). The integrator is only ever cleared in
// enter(), which runs exactly once per transition INTO this state.
//
// Phase 3.7: the error->wrench law (gentle PD attitude damping included) is delegated to the
// shared algorithms::pid::SixDofPidController, driven in a reduced "depth-only" mode: the
// lateral (X/Y) position setpoint is re-latched to the current estimate every tick (zero
// lateral authority, preserving the pre-3.7 contract exactly), the attitude setpoint is the
// identity quaternion (equivalent to the old `-kp_att*rpy` damping law at the small attitudes
// this state operates at -- see StateDepthHold.cpp's constructor comment), and this also adds
// hydrostatic buoyancy feedforward via FossenDynamics, which the pre-3.7 law never had (an
// intentional improvement, not a regression -- see PROGRESS.md Phase 3.7).
class StateDepthHold final : public FSMState
{
public:
  StateDepthHold(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params);

  void enter() override;

  void run(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void exit() override;

  FSMStateName checkChange() override;

  // Reports pos=[0, 0, target_z_], quat=identity -- only Z is a meaningful reference here.
  PoseTarget targetPose() const override;

private:
  // Damped Moore-Penrose pseudoinverse of the 6x8 thruster allocation matrix B (shared
  // allocation::ThrusterAllocator.h geometry), computed once at construction (rov_wall.xml's
  // thruster geometry is static for the lifetime of the process).
  Mat86 b_pinv_;

  FossenDynamics dynamics_;
  SixDofPidController pid_;

  double target_z_ = 0.5;

  // Last setpoint_seq value this state has already applied (CLAUDE.md Phase 3.2 Stage 2.1's
  // edge-detection pattern); starts at 0 and only ever moves forward.
  uint64_t last_seen_setpoint_seq_ = 0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__FSM__STATEDEPTHHOLD_H_
