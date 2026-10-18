#ifndef ADHERE_CONTROLLER__DYNAMICS__FOSSENDYNAMICS_H_
#define ADHERE_CONTROLLER__DYNAMICS__FOSSENDYNAMICS_H_

#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/dynamics/ModelParams.h"

namespace adhere_controller
{

// Nominal Fossen 6-DOF rigid-body + added-mass model (CLAUDE.md Phase 3.7), built once from
// ModelParams and reused by both algorithms::pid::SixDofPidController's feedforward and future
// Phase 4 MPPI rollouts. Diagonal-mass assumption (CoM at body origin, principal-axis inertia --
// matches the legacy ~/Research/mujoco_mppi_ws/src/env/hydrodynamics.py reference's own
// no-off-diagonal-coupling treatment): M = diag(m,m,m,Ixx,Iyy,Izz) + diag(added_mass_lin,
// added_mass_ang). For this diagonal M, the rigid-body+added-mass Coriolis-centripetal matrix
// C(nu) has the standard closed form (Fossen, "Handbook of Marine Craft Hydrodynamics and Motion
// Control", eq. 6.43) that is skew-symmetric by construction, so nu^T*C(nu)*nu == 0 exactly.
class FossenDynamics
{
public:
  explicit FossenDynamics(const ModelParams & params);

  const Mat6 & massMatrix() const { return m_; }
  const Mat6 & massMatrixInverse() const { return m_inv_; }

  // C(nu)*nu, skew-symmetric C(nu) built from the diagonal mass terms above.
  Vec6 coriolisWrench(const Vec6 & nu) const;

  // D(nu)*nu using ModelParams' existing D_lin/D_quad/D_ang_lin/D_ang_quad (linear + quadratic
  // drag per DOF, already tuned/verified elsewhere -- no new parameters needed here).
  Vec6 dampingWrench(const Vec6 & nu) const;

  // g(q): hydrostatic buoyancy/gravity restoring wrench in the body frame, same Fossen EOM sign
  // convention as the rest of this wrench (M*nu_dot + C(nu)*nu + D(nu)*nu + g(q) = tau) -- a
  // feedforward controller cancels it by adding -g(q) to its commanded wrench (this is exactly
  // what StatePidHold.cpp's buoyancy feedforward already computed inline before this extraction).
  Vec6 computeRestoringWrench(const Quat & q) const;

  // Forward dynamics: nu_dot = M^-1 * (tau - C(nu)*nu - D(nu)*nu - g(q)).
  Vec6 forwardDynamics(const Vec6 & nu, const Quat & q, const Vec6 & tau) const;

private:
  ModelParams params_;
  Mat6 m_;
  Mat6 m_inv_;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__DYNAMICS__FOSSENDYNAMICS_H_
