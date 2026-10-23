#ifndef ADHERE_CONTROLLER__FSM__STATEPIDHOLD_H_
#define ADHERE_CONTROLLER__FSM__STATEPIDHOLD_H_

#include <array>
#include <cstdint>

#include "adhere_controller/algorithms/pid/SixDofPidController.h"
#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/common/PoseSetpointIntegrator.h"
#include "adhere_controller/dynamics/FossenDynamics.h"
#include "adhere_controller/FSM/FSMState.h"

namespace adhere_controller
{

// Closed-loop 6-DOF position + heading hold (CLAUDE.md Phase 3.2 Stage 2.2.4). On enter(),
// latches the ROV's current (x, y, z, yaw) as the hold setpoint. While /rov/teleop_cmd is
// active, advances that setpoint at commanded body-aligned velocities; the instant the sticks
// return to center (including via AdhereController's 0.25s watchdog forcing teleop_active
// false on joystick disconnect), the setpoint freezes exactly where it was released.
//
// Phase 3.7: the setpoint-tracking error->wrench law (PID + hydrostatic/drag feedforward) is
// delegated to the shared algorithms::pid::SixDofPidController; this class keeps only the
// setpoint integration (stick-driven advancement, discrete MODE_* handling) in the FSM layer,
// allocated onto the 8 thrusters through the shared allocation::ThrusterAllocator's damped
// pseudoinverse.
class StatePidHold final : public FSMState
{
public:
  StatePidHold(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params);

  void enter() override;

  void run(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void exit() override;

  // Reports the live hold setpoint: pos=p_d_world_, quat=q_d_.
  PoseTarget targetPose() const override;

  bool targetValid() const override { return true; }

private:
  Mat86 b_pinv_;

  FossenDynamics dynamics_;
  SixDofPidController pid_;

  // Setpoint/teleop integration (CLAUDE.md Phase 5.0 Stage 2.2): owns p_d_world_/q_d_ (see
  // mathTools.h's quatError()/quatFromRPY()/smallAngleQuat() for the SO(3) math it drives).
  PoseSetpointIntegrator setpoint_;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__FSM__STATEPIDHOLD_H_
