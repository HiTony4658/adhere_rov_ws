#ifndef ADHERE_CONTROLLER__ALGORITHMS__SMC__SIXDOFSMCCONTROLLER_H_
#define ADHERE_CONTROLLER__ALGORITHMS__SMC__SIXDOFSMCCONTROLLER_H_

#include "adhere_controller/common/mathTypes.h"

namespace adhere_controller
{

// Forward-declared (not included): see SixDofPidController.h's identical comment -- avoids a
// header cycle now that ModelParams.h includes this header for ModelParams::AdhereSmcGains'
// inheritance from Gains (Phase 5.0 Stage 2.4).
class FossenDynamics;

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
//
// This phase: bounded integral-sliding-mode augmentation, s = e_nu + Lambda.e_eta + Ki.integral_
// e_eta_ (integral_e_eta_ accumulates e_eta every tick, clamped per-axis to kIntegralEtaClamp).
// Motivation -- a pure P (Lambda) + switching (Ksw) surface has nothing to "ratchet up" against
// static Coulomb pad friction once wall preload is pressing: a statically-stuck ROV has zero
// velocity error (e_nu~0) and the switching term saturates at a FIXED magnitude once |s|>phi, so
// neither term grows over time the way an integral does. gains_.ki defaults to all-zero (no
// behavior change for any future SMC consumer); only adhere_smc.ki in robot_control.yaml sets
// the Y/Z components nonzero.
class SixDofSmcController
{
public:
  struct Gains
  {
    Vec6 lambda = Vec6::Zero();
    Vec6 kd = Vec6::Zero();
    Vec6 k_switch = Vec6::Zero();
    Vec6 phi = Vec6::Ones();
    Vec6 ki = Vec6::Zero();  // This phase: integral-sliding-mode gain, see class comment
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
  // docked). This phase: no longer const -- accumulates integral_e_eta_ every call (see
  // resetIntegrator() below, which StateAdhereSmc::enter() must now call, same pattern as
  // SixDofPidController/AdherePidController's own integrator resets).
  Vec6 computeWrench(const Inputs & in, bool apply_wall_preload, double dt);

  void resetIntegrator() { integral_e_eta_ = Vec6::Zero(); }

private:
  Gains gains_;
  const FossenDynamics & dynamics_;
  Vec6 integral_e_eta_ = Vec6::Zero();
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__ALGORITHMS__SMC__SIXDOFSMCCONTROLLER_H_
