#ifndef ADHERE_CONTROLLER_COMMON__FSM__STATEDEPTHHOLD_H_
#define ADHERE_CONTROLLER_COMMON__FSM__STATEDEPTHHOLD_H_

#include <array>
#include <cstdint>

#include "adhere_controller_common/common/mathTypes.h"
#include "adhere_controller_common/FSM/FSMState.h"

namespace adhere_controller_common
{

// Closed-loop PID depth (world Z) controller (CLAUDE.md Phase 3.1/3.2 Stage 2.2). Tracks a
// local target_z_, retargeted either discretely (a new SetpointCommand with mode==2, edge-
// detected via setpoint_seq so the same message is never re-applied every tick) or continuously
// (the Xbox left-stick heave_z nudge, integrated every tick and left latched when the stick
// returns to center -- CLAUDE.md Phase 3.2 Stage 2.2.2). The integrator is only ever cleared in
// enter(), which runs exactly once per transition INTO this state. Also applies gentle PD
// attitude damping on roll/pitch/yaw so the hull does not tumble while thrusting, and maps the
// resulting 6-DOF body-frame wrench onto the 8 thruster commands through the shared
// ThrusterAllocator's damped pseudoinverse of rov_wall.xml's exact thruster allocation matrix B.
class StateDepthHold final : public FSMState
{
public:
  explicit StateDepthHold(CtrlInterfaces & ctrl_interfaces);

  void enter() override;

  void run(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void exit() override;

  FSMStateName checkChange() override;

  // Reports [0, 0, target_z_, 0] -- only Z is a meaningful reference for this state.
  std::array<double, 4> targetPose() const override;

private:
  // Damped Moore-Penrose pseudoinverse of the 6x8 thruster allocation matrix B (shared
  // ThrusterAllocator.h geometry), computed once at construction (rov_wall.xml's thruster
  // geometry is static for the lifetime of the process).
  Mat86 b_pinv_;

  double target_z_ = 0.5;
  double integral_z_ = 0.0;

  // Last setpoint_seq value this state has already applied (CLAUDE.md Phase 3.2 Stage 2.1's
  // edge-detection pattern); starts at 0 and only ever moves forward.
  uint64_t last_seen_setpoint_seq_ = 0;
};

}  // namespace adhere_controller_common

#endif  // ADHERE_CONTROLLER_COMMON__FSM__STATEDEPTHHOLD_H_
