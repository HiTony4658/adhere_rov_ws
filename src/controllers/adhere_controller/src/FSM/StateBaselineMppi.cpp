#include "adhere_controller/FSM/StateBaselineMppi.h"

namespace adhere_controller
{

StateBaselineMppi::StateBaselineMppi(
  CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params,
  baseline_mppi::BaselineMppiController & planner)
: StateAdhereBase(
    FSMStateName::ADHERE_MPPI_BASELINE, "state_adhere_mppi_baseline", ctrl_interfaces,
    model_params),
  planner_(planner)
{
}

void StateBaselineMppi::enter()
{
  StateAdhereBase::enter();

  // Resets the async worker's persistent nominal sequence to the hydrostatic hover wrench
  // (CLAUDE.md Phase 4.1 Sec.3.1) and invalidates any stale ZOH output from a previous activation
  // so run() never writes a leftover command from before this enter().
  planner_.warmStart(setpoint_.p_d_world_, setpoint_.q_d_);
  ctrl_interfaces_.outer_loop_cmd_.valid.store(false);

  // Safe-default contract (CLAUDE.md Phase 3.1 Stage 3.1, same as StatePassive::enter()): zeros
  // out any stale command left behind by the previous FSM state so the thrusters command 0 for
  // the handful of ticks between entering this state and the worker's first valid solve above.
  ctrl_interfaces_.pending_thruster_cmd.fill(0.0);
}

bool StateBaselineMppi::computeAdhesionWrench(
  const Vec3 & /*pos_world*/, const Quat & /*q_est*/, const Vec3 & /*vel_body*/,
  const Vec3 & /*omega_body*/, double /*sum_f*/, double /*dt*/, Vec6 & /*wrench_out*/)
{
  // Strict Zero-Order Hold (CLAUDE.md Phase 4.1 Sec.3.3): writes the latest async 25Hz CUDA MPPI
  // output verbatim into pending_thruster_cmd. If the worker hasn't produced a first valid solve
  // yet (e.g. immediately after enter()), pending_thruster_cmd is simply left untouched -- it was
  // zeroed at enter()'s preceding FSM transition / defaults to the all-zero safe state.
  if (ctrl_interfaces_.outer_loop_cmd_.valid.load()) {
    for (int i = 0; i < 8; ++i) {
      ctrl_interfaces_.pending_thruster_cmd[i] = ctrl_interfaces_.outer_loop_cmd_.u_star[i].load();
    }
  }
  return false;
}

}  // namespace adhere_controller
