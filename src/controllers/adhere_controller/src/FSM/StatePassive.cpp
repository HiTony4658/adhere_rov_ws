#include "adhere_controller/FSM/StatePassive.h"

namespace adhere_controller
{

StatePassive::StatePassive(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params)
: FSMState(FSMStateName::PASSIVE, "state_passive", ctrl_interfaces, model_params)
{
}

void StatePassive::enter()
{
  for (auto & cmd : ctrl_interfaces_.thruster_command_interface_) {
    cmd.get().set_value(0.0);
  }
}

void StatePassive::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Zero-output contract (CLAUDE.md Phase 3.1 Stage 3.1): re-asserted every tick, not just on
  // enter(), so this state is immune to any stray non-zero value left behind by a future state
  // that forgets to zero a thruster on exit().
  for (auto & cmd : ctrl_interfaces_.thruster_command_interface_) {
    cmd.get().set_value(0.0);
  }
}

void StatePassive::exit()
{
}

FSMStateName StatePassive::checkChange()
{
  return checkFsmCommandChange();
}

}  // namespace adhere_controller
