#include "adhere_controller/FSM/StatePassive.h"

namespace adhere_controller
{

StatePassive::StatePassive(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params)
: FSMState(FSMStateName::IDLE_PASSIVE_NONE, "state_idle_passive_none", ctrl_interfaces, model_params)
{
}

void StatePassive::enter()
{
  ctrl_interfaces_.pending_thruster_cmd.fill(0.0);
  for (auto & cmd : ctrl_interfaces_.thruster_command_interface_) {
    cmd.get().set_value(0.0);
  }
}

void StatePassive::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Zero-output contract (CLAUDE.md Phase 3.1 Stage 3.1): re-asserted every tick, not just on
  // enter(), so this state is immune to any stray non-zero value left behind by a future state
  // that forgets to zero a thruster on exit().
  ctrl_interfaces_.pending_thruster_cmd.fill(0.0);
}

void StatePassive::exit()
{
}

}  // namespace adhere_controller
