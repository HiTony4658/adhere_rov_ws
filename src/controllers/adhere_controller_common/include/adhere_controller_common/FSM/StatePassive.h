#ifndef ADHERE_CONTROLLER_COMMON__FSM__STATEPASSIVE_H_
#define ADHERE_CONTROLLER_COMMON__FSM__STATEPASSIVE_H_

#include "adhere_controller_common/FSM/FSMState.h"

namespace adhere_controller_common
{

// Zero-output safe/idle state (CLAUDE.md Phase 3.1 Stage 3.1): all 8 thruster commands are
// identically 0.0 N on enter() and every run() tick, so the ROV resumes exactly the passive
// free-drift behavior verified in Phase 2 (net buoyancy bias lifts it, hydrodynamic drag is the
// only other force acting on the hull).
class StatePassive final : public FSMState
{
public:
  explicit StatePassive(CtrlInterfaces & ctrl_interfaces);

  void enter() override;

  void run(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void exit() override;

  FSMStateName checkChange() override;
};

}  // namespace adhere_controller_common

#endif  // ADHERE_CONTROLLER_COMMON__FSM__STATEPASSIVE_H_
