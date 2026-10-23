#ifndef ADHERE_CONTROLLER__FSM__STATEOPENLOOP_H_
#define ADHERE_CONTROLLER__FSM__STATEOPENLOOP_H_

#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/FSM/FSMState.h"

namespace adhere_controller
{

// Open-loop Mode-2 (美国手) joystick wrench control (CLAUDE.md Phase 3.2 Stage 2.2.3): maps
// /rov/teleop_cmd directly to a body-frame wrench every tick (no integrators, no memory between
// ticks) and allocates it onto the 8 thrusters through the shared ThrusterAllocator's damped
// pseudoinverse. Centered sticks (teleop inputs all 0.0, including the AdhereController
// watchdog's forced-zero on joystick disconnect) produce u = 0 exactly. Used to verify the
// B/B^dagger allocation (zero parasitic pitch/yaw coupling for pure sway/heave commands) before
// trusting closed-loop states built on top of the same allocator.
class StateOpenLoop final : public FSMState
{
public:
  StateOpenLoop(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params);

  void enter() override;

  void run(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void exit() override;

private:
  Mat86 b_pinv_;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__FSM__STATEOPENLOOP_H_
