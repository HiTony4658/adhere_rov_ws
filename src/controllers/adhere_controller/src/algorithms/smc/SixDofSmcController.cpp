#include "adhere_controller/algorithms/smc/SixDofSmcController.h"

#include "adhere_controller/common/mathTools.h"

namespace adhere_controller
{

Vec6 SixDofSmcController::computeWrench(const Inputs & in, bool apply_wall_preload) const
{
  const RotMat r = quatToRotMat(in.q_est);
  const Quat q_est_aligned = alignQuaternionHemisphere(in.q_d, in.q_est);

  const Vec3 e_p_body = r.transpose() * (in.p_est_world - in.p_d_world);
  // Swapped argument order vs. SixDofPidController's quatError(q_est, q_d): gives e_R_body the
  // same "estimate minus desired" sign as e_p_body (see class header comment).
  const Vec3 e_r_body = quatError(in.q_d, q_est_aligned);
  const Vec3 e_eta_lin = e_p_body;
  const Vec3 e_eta_ang = e_r_body;

  Vec6 nu_est;
  nu_est << in.v_est_body, in.omega_est_body;
  Vec6 nu_d;
  nu_d << in.v_d_body, in.omega_cmd_body;
  const Vec6 e_nu = nu_est - nu_d;

  Vec6 e_eta;
  e_eta << e_eta_lin, e_eta_ang;

  const Vec6 s = e_nu + gains_.lambda.cwiseProduct(e_eta);
  const Vec6 s_over_phi = s.cwiseQuotient(gains_.phi);
  const Vec6 sat_s = s_over_phi.cwiseMax(-1.0).cwiseMin(1.0);

  const Vec6 coriolis = dynamics_.coriolisWrench(nu_est);
  const Vec6 damping = dynamics_.dampingWrench(nu_est);
  const Vec6 restoring = dynamics_.computeRestoringWrench(in.q_est);
  const Vec6 lambda_e_nu = gains_.lambda.cwiseProduct(e_nu);

  Vec6 wall_preload = Vec6::Zero();
  if (apply_wall_preload) {
    wall_preload(0) = -gains_.wall_preload_fx;
  }

  const Vec6 tau = coriolis + damping + restoring - dynamics_.massMatrix() * lambda_e_nu +
    wall_preload - gains_.kd.cwiseProduct(s) - gains_.k_switch.cwiseProduct(sat_s);
  return tau;
}

}  // namespace adhere_controller
