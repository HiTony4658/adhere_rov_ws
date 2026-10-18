#ifndef ADHERE_CONTROLLER__ALGORITHMS__SMC__SIXDOFSMCCONTROLLER_H_
#define ADHERE_CONTROLLER__ALGORITHMS__SMC__SIXDOFSMCCONTROLLER_H_

#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/dynamics/FossenDynamics.h"

namespace adhere_controller
{

// Wall-Adhesion 6-DOF Sliding Mode Control baseline (CLAUDE.md Phase 3.8 Stage 2.2.2): the
// second classical wall-adhesion baseline, intended to show stronger disturbance rejection than
// AdherePidController/SixDofPidController at the cost of higher chattering/energy.
//
// Error/sign convention (a deliberate, documented choice -- the task's prose formula does not
// fully pin this down, see the Phase 3.8 plan's "sign/engineering decisions" section): this
// class uses an "estimate minus desired" convention for BOTH position and attitude error, unlike
// SixDofPidController's "desired minus estimate" convention for position:
//   e_p_body  = R(q_est)^T * (p_est_world - p_d_world)
//   e_R_body  = quatError(q_d, q_est_aligned)   (mathTools.h; note the swapped argument order
//                                                relative to SixDofPidController's quatError(
//                                                q_est, q_d) -- this flips the sign so e_R_body
//                                                shares e_p_body's "estimate minus desired" sign)
//   e_eta     = [e_p_body; e_R_body]                         (6D combined tracking error)
//   e_nu      = nu_est - nu_d, nu = [v_body; omega_body]     (6D velocity error)
//   s         = e_nu + Lambda . e_eta                        (sliding surface)
// Control law (CLAUDE.md Sec.2.2.2's SMC law, nominal-dynamics-cancellation + linear reaching +
// boundary-layer switching + wall preload):
//   tau = C(nu_est)*nu_est + D(nu_est)*nu_est + g(q_est) - M*(Lambda . e_nu)
//         + [-wall_preload_fx, 0,0,0,0,0] (only when apply_wall_preload) - Kd . s
//         - Ksw . sat(s / phi)
// This reduces to a stable form near s=0 (nu_dot ~= -M^-1*Kd*s, since the nominal feedforward
// terms cancel the real dynamics and -M*Lambda*e_nu cancels the Lambda*e_eta contribution to
// s_dot): verified empirically by the closed-loop gtest rather than a symbolic Lyapunov proof,
// matching the validation approach already used for FossenDynamics/SixDofPidController.
class SixDofSmcController
{
public:
  struct Gains
  {
    Vec6 lambda = Vec6::Zero();
    Vec6 kd = Vec6::Zero();
    Vec6 k_switch = Vec6::Zero();
    Vec6 phi = Vec6::Ones();
    double wall_preload_fx = 0.0;  // N magnitude; see class comment for sign
  };

  struct Inputs
  {
    Vec3 p_d_world = Vec3::Zero();
    Quat q_d = Quat(1.0, 0.0, 0.0, 0.0);
    Vec3 p_est_world = Vec3::Zero();
    Quat q_est = Quat(1.0, 0.0, 0.0, 0.0);
    Vec3 v_d_body = Vec3::Zero();
    Vec3 omega_cmd_body = Vec3::Zero();
    Vec3 v_est_body = Vec3::Zero();
    Vec3 omega_est_body = Vec3::Zero();
  };

  SixDofSmcController(const Gains & gains, const FossenDynamics & dynamics)
  : gains_(gains), dynamics_(dynamics)
  {
  }

  // apply_wall_preload gates the constant pressing bias (StateAdhereSmc passes true only once
  // docked) -- no integrator state in this controller, so no resetIntegrators() is needed.
  Vec6 computeWrench(const Inputs & in, bool apply_wall_preload) const;

private:
  Gains gains_;
  const FossenDynamics & dynamics_;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__ALGORITHMS__SMC__SIXDOFSMCCONTROLLER_H_
