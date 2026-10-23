#include "adhere_controller/algorithms/pid/SixDofPidController.h"

#include "adhere_controller/common/mathTools.h"
#include "adhere_controller/dynamics/FossenDynamics.h"

namespace adhere_controller
{

namespace
{

// Conditional anti-windup (CLAUDE.md Phase 3.7 Stage 1.3): integrates normally unless this
// axis's integral term is already at its +-clamp AND the new error would push it further past
// that clamp, in which case the axis is simply frozen for this tick instead of hard-clamping
// after the fact (hard-clamping alone still lets the raw accumulator wind up arbitrarily far
// past the clamp before saturating the *output*; freezing stops the accumulator itself).
double integrateConditional(double integral, double error, double dt, double clamp)
{
  const double next = integral + error * dt;
  const bool already_saturated = std::abs(integral) >= clamp;
  const bool pushing_further = (next > integral) == (integral > 0.0);
  if (already_saturated && pushing_further) {
    return integral;
  }
  return saturation(next, -clamp, clamp);
}

}  // namespace

SixDofPidController::SixDofPidController(
  const Gains & gains, const FossenDynamics & dynamics, double integral_pos_max,
  double integral_att_max)
: gains_(gains),
  dynamics_(dynamics),
  integral_pos_max_(integral_pos_max),
  integral_att_max_(integral_att_max)
{
}

void SixDofPidController::resetIntegrators()
{
  integral_pos_body_.setZero();
  integral_att_body_.setZero();
}

Vec6 SixDofPidController::computeWrench(const Inputs & in, double dt)
{
  const RotMat r = quatToRotMat(in.q_est);
  const Quat q_d_aligned = alignQuaternionHemisphere(in.q_est, in.q_d);

  const Vec3 e_p_body = r.transpose() * (in.p_d_world - in.p_est_world);
  const Vec3 e_v_body = in.v_d_body - in.v_est_body;
  const Vec3 e_r = quatError(in.q_est, q_d_aligned);
  const Vec3 e_omega_body = in.omega_cmd_body - in.omega_est_body;

  for (int i = 0; i < 3; ++i) {
    integral_pos_body_(i) = integrateConditional(
      integral_pos_body_(i), e_p_body(i), dt, integral_pos_max_ / gains_.ki_pos(i));
    integral_att_body_(i) = integrateConditional(
      integral_att_body_(i), e_r(i), dt, integral_att_max_ / gains_.ki_att(i));
  }

  // Hydrostatic buoyancy/gravity feedforward: the control law cancels g(q_est) by adding its
  // negative (CLAUDE.md Phase 3.7 Stage 1.2's Fossen EOM sign convention: M*nu_dot + C*nu +
  // D*nu + g(q) = tau).
  const Vec6 g = dynamics_.computeRestoringWrench(in.q_est);

  // Drag feedforward at the desired linear velocity only (matches the pre-extraction inline law
  // in StatePidHold.cpp, which never fed an angular-velocity feedforward term either).
  Vec6 nu_d = Vec6::Zero();
  nu_d.head<3>() = in.v_d_body;
  const Vec6 d_ff = dynamics_.dampingWrench(nu_d);

  const Vec3 w_lin = gains_.kp_pos.cwiseProduct(e_p_body) +
    gains_.ki_pos.cwiseProduct(integral_pos_body_) + gains_.kd_vel.cwiseProduct(e_v_body) +
    d_ff.head<3>() - g.head<3>();
  const Vec3 w_ang = gains_.kp_att.cwiseProduct(e_r) +
    gains_.ki_att.cwiseProduct(integral_att_body_) + gains_.kd_att.cwiseProduct(e_omega_body) -
    g.tail<3>();

  Vec6 wrench;
  wrench << w_lin, w_ang;
  return wrench;
}

}  // namespace adhere_controller
