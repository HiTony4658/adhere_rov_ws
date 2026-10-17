#include "adhere_controller_common/FSM/StateOpenLoop.h"

#include "adhere_controller_common/common/mathTools.h"
#include "adhere_controller_common/common/ThrusterAllocator.h"

namespace adhere_controller_common
{

namespace
{

// Mode-2 (美国手) open-loop wrench scaling (CLAUDE.md Phase 3.2 Stage 2.2.3).
constexpr double kFxMax = 60.0;  // N
constexpr double kFyMax = 50.0;  // N
constexpr double kFzMax = 50.0;  // N
constexpr double kTzMax = 8.0;   // N*m

constexpr double kThrusterMin = -40.0;
constexpr double kThrusterMax = 40.0;

}  // namespace

StateOpenLoop::StateOpenLoop(CtrlInterfaces & ctrl_interfaces)
: FSMState(FSMStateName::OPEN_LOOP, "state_open_loop", ctrl_interfaces)
{
  b_pinv_ = buildThrusterAllocationPseudoInverse(buildThrusterAllocationMatrix());
}

void StateOpenLoop::enter()
{
  for (auto & cmd : ctrl_interfaces_.thruster_command_interface_) {
    cmd.get().set_value(0.0);
  }
}

void StateOpenLoop::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto & ci = ctrl_interfaces_.control_inputs_;

  // surge_x > 0 => move toward the wall along body -X (CLAUDE.md Phase 3.2 Stage 2.2.3).
  const double f_x = -kFxMax * ci.teleop_surge_x.load();
  const double f_y = -kFyMax * ci.teleop_sway_y.load();
  const double f_z = kFzMax * ci.teleop_heave_z.load();
  const double t_z = kTzMax * ci.teleop_yaw_rate.load();

  Vec6 wrench_des;
  wrench_des << f_x, f_y, f_z, 0.0, 0.0, t_z;

  const Vec8 u = b_pinv_ * wrench_des;
  for (int i = 0; i < 8; ++i) {
    ctrl_interfaces_.thruster_command_interface_[i].get().set_value(
      saturation(u(i), kThrusterMin, kThrusterMax));
  }
}

void StateOpenLoop::exit()
{
  for (auto & cmd : ctrl_interfaces_.thruster_command_interface_) {
    cmd.get().set_value(0.0);
  }
}

FSMStateName StateOpenLoop::checkChange()
{
  return checkFsmCommandChange();
}

}  // namespace adhere_controller_common
