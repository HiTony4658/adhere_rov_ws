#include "adhere_controller/FSM/StateOpenLoop.h"

#include "adhere_controller/allocation/ThrusterAllocator.h"
#include "adhere_controller/common/mathTools.h"

namespace adhere_controller
{

StateOpenLoop::StateOpenLoop(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params)
: FSMState(FSMStateName::TELEOP_OPENLOOP_6DOF, "state_teleop_openloop_6dof", ctrl_interfaces, model_params)
{
  b_pinv_ = buildThrusterAllocationPseudoInverse(
    buildThrusterAllocationMatrix(model_params_.thruster_pos, model_params_.thruster_dir));
}

void StateOpenLoop::enter()
{
  ctrl_interfaces_.pending_thruster_cmd.fill(0.0);
  for (auto & cmd : ctrl_interfaces_.thruster_command_interface_) {
    cmd.get().set_value(0.0);
  }
}

void StateOpenLoop::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto & ci = ctrl_interfaces_.control_inputs_;

  // surge_x > 0 => move toward the wall along body -X (CLAUDE.md Phase 3.2 Stage 2.2.3).
  const auto & max_wrench = model_params_.open_loop.max_wrench;
  const double f_x = -max_wrench[0] * ci.teleop_surge_x.load();
  const double f_y = -max_wrench[1] * ci.teleop_sway_y.load();
  const double f_z = max_wrench[2] * ci.teleop_heave_z.load();
  // Phase 3.5: Xbox LT/LB -> pitch torque Ty, RT/RB -> roll torque Tx.
  const double t_x = max_wrench[3] * ci.teleop_roll_rate.load();
  const double t_y = max_wrench[4] * ci.teleop_pitch_rate.load();
  const double t_z = max_wrench[5] * ci.teleop_yaw_rate.load();

  Vec6 wrench_des;
  wrench_des << f_x, f_y, f_z, t_x, t_y, t_z;

  const Vec8 u = b_pinv_ * wrench_des;
  for (int i = 0; i < 8; ++i) {
    ctrl_interfaces_.pending_thruster_cmd[i] = u(i);
  }
}

void StateOpenLoop::exit()
{
  ctrl_interfaces_.pending_thruster_cmd.fill(0.0);
  for (auto & cmd : ctrl_interfaces_.thruster_command_interface_) {
    cmd.get().set_value(0.0);
  }
}

FSMStateName StateOpenLoop::checkChange()
{
  return checkFsmCommandChange();
}

}  // namespace adhere_controller
