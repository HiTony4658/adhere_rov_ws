#ifndef ADHERE_CONTROLLER_COMMON__FSM__STATEDEPTHHOLD_H_
#define ADHERE_CONTROLLER_COMMON__FSM__STATEDEPTHHOLD_H_

#include "adhere_controller_common/common/mathTypes.h"
#include "adhere_controller_common/FSM/FSMState.h"

namespace adhere_controller_common
{

// Closed-loop PID depth (world Z) controller (CLAUDE.md Phase 3.1 Stage 2.2). Tracks
// ctrl_interfaces_.control_inputs_.target_z, read live every tick so the console can retarget
// without a state transition (and therefore without resetting the integrator -- the integrator
// is only ever cleared in enter(), which runs exactly once per PASSIVE -> DEPTH_HOLD
// transition). Also applies gentle PD attitude damping on roll/pitch/yaw so the hull does not
// tumble while thrusting, and maps the resulting 6-DOF body-frame wrench onto the 8 thruster
// commands through the Moore-Penrose pseudoinverse of rov_wall.xml's exact thruster allocation
// matrix B (see StateDepthHold.cpp).
class StateDepthHold final : public FSMState
{
public:
  explicit StateDepthHold(CtrlInterfaces & ctrl_interfaces);

  void enter() override;

  void run(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void exit() override;

  FSMStateName checkChange() override;

private:
  // Moore-Penrose pseudoinverse of the 6x8 thruster allocation matrix B, computed once at
  // construction (rov_wall.xml's thruster geometry is static for the lifetime of the process).
  Mat86 b_pinv_;

  double integral_z_ = 0.0;
};

}  // namespace adhere_controller_common

#endif  // ADHERE_CONTROLLER_COMMON__FSM__STATEDEPTHHOLD_H_
