#include "adhere_controller/FSM/StateAdherePid.h"

#include "adhere_controller/allocation/ThrusterAllocator.h"

namespace adhere_controller
{

namespace
{

// Anti-windup clamps for the Y/Z/attitude SixDofPidController half of this state (not part of
// robot_control.yaml's adhere_pid.* schema, kept as fixed design constants).
constexpr double kIntegralPosMax = 180.0;  // N
constexpr double kIntegralAttMax = 10.0;   // N*m

}  // namespace

StateAdherePid::StateAdherePid(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params)
: StateAdhereBase(
    FSMStateName::ADHERE_PID_WBC, "state_adhere_pid_wbc", ctrl_interfaces, model_params),
  dynamics_(model_params),
  // model_params.adhere_pid inherits BOTH SixDofPidController::Gains and
  // AdherePidController::Gains directly (CLAUDE.md Phase 5.0 Stage 2.4) -- no copy functions
  // needed.
  pid_(model_params.adhere_pid, dynamics_, kIntegralPosMax, kIntegralAttMax),
  force_pid_(model_params.adhere_pid)
{
  b_pinv_ = buildThrusterAllocationPseudoInverse(
    buildThrusterAllocationMatrix(model_params_.thruster_pos, model_params_.thruster_dir));
}

void StateAdherePid::enter()
{
  StateAdhereBase::enter();
  pid_.resetIntegrators();
  force_pid_.resetIntegrator();
}

bool StateAdherePid::computeAdhesionWrench(
  const Vec3 & pos_world, const Quat & q_est, const Vec3 & vel_body, const Vec3 & omega_body,
  double sum_f, double dt, Vec6 & wrench_out)
{
  SixDofPidController::Inputs in;
  in.p_d_world = setpoint_.p_d_world_;
  in.q_d = setpoint_.q_d_;
  in.p_est_world = pos_world;
  in.q_est = q_est;
  in.v_d_body = teleop_.v_d_body;
  in.omega_cmd_body = teleop_.omega_cmd_body;
  in.v_est_body = vel_body;
  in.omega_est_body = omega_body;

  wrench_out = pid_.computeWrench(in, dt);
  if (docked_) {
    wrench_out(0) =
      force_pid_.computeNormalForceWrench(model_params_.adhere_pid.f_des, sum_f, vel_body.x(), dt);
  }
  return true;
}

}  // namespace adhere_controller
