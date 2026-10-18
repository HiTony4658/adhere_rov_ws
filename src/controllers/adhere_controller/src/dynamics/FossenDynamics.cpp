#include "adhere_controller/dynamics/FossenDynamics.h"

#include "adhere_controller/common/mathTools.h"

namespace adhere_controller
{

FossenDynamics::FossenDynamics(const ModelParams & params)
: params_(params)
{
  const double m = params_.vehicle_mass;
  Vec6 diag;
  diag.head<3>() = Vec3::Constant(m) + params_.dynamics.added_mass_lin;
  diag.tail<3>() = params_.dynamics.inertia_diag + params_.dynamics.added_mass_ang;
  m_ = diag.asDiagonal();
  m_inv_ = m_.inverse();
}

Vec6 FossenDynamics::coriolisWrench(const Vec6 & nu) const
{
  const double m1 = m_(0, 0);
  const double m2 = m_(1, 1);
  const double m3 = m_(2, 2);
  const double m4 = m_(3, 3);
  const double m5 = m_(4, 4);
  const double m6 = m_(5, 5);
  const double u = nu(0);
  const double v = nu(1);
  const double w = nu(2);
  const double p = nu(3);
  const double q = nu(4);
  const double r = nu(5);

  // Closed-form skew-symmetric rigid-body+added-mass Coriolis-centripetal matrix for a diagonal
  // mass matrix (Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control", eq. 6.43).
  // Skew-symmetric by construction, so nu^T*C(nu)*nu == 0 exactly for any nu.
  Mat6 c = Mat6::Zero();
  c(0, 4) = m3 * w;
  c(0, 5) = -m2 * v;
  c(1, 3) = -m3 * w;
  c(1, 5) = m1 * u;
  c(2, 3) = m2 * v;
  c(2, 4) = -m1 * u;
  c(3, 1) = m3 * w;
  c(3, 2) = -m2 * v;
  c(3, 4) = m6 * r;
  c(3, 5) = -m5 * q;
  c(4, 0) = -m3 * w;
  c(4, 2) = m1 * u;
  c(4, 3) = -m6 * r;
  c(4, 5) = m4 * p;
  c(5, 0) = m2 * v;
  c(5, 1) = -m1 * u;
  c(5, 3) = m5 * q;
  c(5, 4) = -m4 * p;

  return c * nu;
}

Vec6 FossenDynamics::dampingWrench(const Vec6 & nu) const
{
  const Vec3 v = nu.head<3>();
  const Vec3 omega = nu.tail<3>();

  Vec6 d;
  d.head<3>() = params_.D_lin.cwiseProduct(v) +
    params_.D_quad.cwiseProduct(v.cwiseProduct(v.cwiseAbs()));
  d.tail<3>() = params_.D_ang_lin.cwiseProduct(omega) +
    params_.D_ang_quad.cwiseProduct(omega.cwiseProduct(omega.cwiseAbs()));
  return d;
}

Vec6 FossenDynamics::computeRestoringWrench(const Quat & q) const
{
  const RotMat r = quatToRotMat(q);
  const double buoyancy_full = params_.rho_water * params_.gravity * params_.displaced_volume;
  const Vec3 f_buoy_net_world(0.0, 0.0, buoyancy_full - params_.vehicle_mass * params_.gravity);
  const Vec3 f_buoy_full_world(0.0, 0.0, buoyancy_full);

  Vec6 g;
  g.head<3>() = r.transpose() * f_buoy_net_world;
  g.tail<3>() = params_.cob_offset_body.cross(r.transpose() * f_buoy_full_world);
  return g;
}

Vec6 FossenDynamics::forwardDynamics(const Vec6 & nu, const Quat & q, const Vec6 & tau) const
{
  return m_inv_ * (tau - coriolisWrench(nu) - dampingWrench(nu) - computeRestoringWrench(q));
}

}  // namespace adhere_controller
