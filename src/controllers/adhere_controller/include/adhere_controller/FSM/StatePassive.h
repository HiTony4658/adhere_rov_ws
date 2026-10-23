#ifndef ADHERE_CONTROLLER__FSM__STATEPASSIVE_H_
#define ADHERE_CONTROLLER__FSM__STATEPASSIVE_H_

#include "adhere_controller/FSM/FSMState.h"

namespace adhere_controller
{

// Zero-output safe/idle state (CLAUDE.md Phase 3.1 Stage 3.1): all 8 thruster commands are
// identically 0.0 N on enter() and every run() tick, so the ROV resumes exactly the passive
// free-drift behavior verified in Phase 2 (net buoyancy bias lifts it, hydrodynamic drag is the
// only other force acting on the hull).
class StatePassive final : public FSMState
{
public:
  StatePassive(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params);

  void enter() override;

  void run(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void exit() override;

};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__FSM__STATEPASSIVE_H_
