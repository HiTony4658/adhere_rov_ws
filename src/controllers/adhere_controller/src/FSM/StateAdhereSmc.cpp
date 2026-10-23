#include "adhere_controller/FSM/StateAdhereSmc.h"

#include "adhere_controller/allocation/ThrusterAllocator.h"

namespace adhere_controller
{

StateAdhereSmc::StateAdhereSmc(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params)
: StateAdhereBase(
    FSMStateName::ADHERE_SMC_WBC, "state_adhere_smc_wbc", ctrl_interfaces, model_params),
  dynamics_(model_params),
  // model_params.adhere_smc inherits SixDofSmcController::Gains directly (CLAUDE.md Phase 5.0
  // Stage 2.4) -- no copy function needed.
  smc_(model_params.adhere_smc, dynamics_)
{
  b_pinv_ = buildThrusterAllocationPseudoInverse(
    buildThrusterAllocationMatrix(model_params_.thruster_pos, model_params_.thruster_dir));
}

void StateAdhereSmc::enter()
{
  StateAdhereBase::enter();
  smc_.resetIntegrator();
}

bool StateAdhereSmc::computeAdhesionWrench(
  const Vec3 & pos_world, const Quat & q_est, const Vec3 & vel_body, const Vec3 & omega_body,
  double /*sum_f*/, double dt, Vec6 & wrench_out)
{
  SixDofSmcController::Inputs in;
  in.p_d_world = setpoint_.p_d_world_;
  in.q_d = setpoint_.q_d_;
  in.p_est_world = pos_world;
  in.q_est = q_est;
  in.v_d_body = teleop_.v_d_body;
  in.omega_cmd_body = teleop_.omega_cmd_body;
  in.v_est_body = vel_body;
  in.omega_est_body = omega_body;

  wrench_out = smc_.computeWrench(in, docked_, dt);
  return true;
}

}  // namespace adhere_controller
