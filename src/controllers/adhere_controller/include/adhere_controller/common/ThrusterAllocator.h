#ifndef ADHERE_CONTROLLER__COMMON__THRUSTERALLOCATOR_H_
#define ADHERE_CONTROLLER__COMMON__THRUSTERALLOCATOR_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "adhere_controller/common/mathTypes.h"

namespace adhere_controller
{

// Dynamic 6x8 body-frame wrench allocation matrix B, built generically from per-thruster
// position/direction vectors loaded at runtime via ModelParams (CLAUDE.md Phase 3.3 Sec.3: zero
// hardcoded robot geometry in adhere_controller C++). Column i is the rigid-body wrench a unit
// force along dir[i] applied at pos[i] produces: [dir_i; pos_i x dir_i]^T in R^6, row order
// [Fx, Fy, Fz, Tx, Ty, Tz]. Algebraically identical to the Phase 3.2 hardcoded corner/zaxis
// derivation for rov_wall.xml's actual geometry (verified by test_thruster_allocator.cpp), but
// generalizes to any thruster layout supplied by robot_control.yaml.
inline Mat68 buildThrusterAllocationMatrix(
  const std::array<Vec3, 8> & pos, const std::array<Vec3, 8> & dir)
{
  Mat68 b = Mat68::Zero();
  for (int i = 0; i < 8; ++i) {
    b.block<3, 1>(0, i) = dir[i];
    b.block<3, 1>(3, i) = pos[i].cross(dir[i]);
  }
  return b;
}

// Damped Moore-Penrose pseudoinverse B^dagger = B^T (B B^T + lambda_reg*I_6)^-1 in R^{8x6}
// (CLAUDE.md Phase 3.2 Stage 2.2). rank(B) == 6 by construction, so lambda_reg's effect on the
// result is negligible at the default 1e-6 -- it exists purely as a numerical-conditioning
// safety margin, not to regularize an actual rank deficiency.
inline Mat86 buildThrusterAllocationPseudoInverse(const Mat68 & b, double lambda_reg = 1e-6)
{
  const Mat6 gram = b * b.transpose() + lambda_reg * Mat6::Identity();
  return b.transpose() * gram.inverse();
}

// Pre-MPPI contact-force distribution & Rhombic Kern constraint matrices (CLAUDE.md Phase 3.6
// Stage 2.2 / Sec.5): pre-computed once at on_activate() so the KAMR-MPPI's 12D OSQP (Phase 4)
// has A_contact/C_kern ready to consume, and so AdhereController can already report an
// estimated kern_margin on /rov/controller_state ahead of that.

// G in R^{3x4}: maps the 4 pad normal forces F_N,i >= 0 (along body +X) to the body wrench
// components [Fx, Ty, Tz]^T. Column i = [+1, -z_i, y_i]^T, built from pad_pos (body frame,
// relative to CoM) -- CLAUDE.md Sec.4.4/Phase 3.6 Sec.2.2. Note: the first component is +1, not
// the -1 the prose formula literally states -- F_N,i is explicitly defined "along +X", and the
// documented worked example (equal pressing thrusts u[0:4]=25N, i.e. Fx_thrusters=-100N via the
// -X-pointing N1..N4 dir vectors, must yield A_contact*u=[25,25,25,25]^T N) only holds
// numerically with +1 here; verified in test_thruster_allocator.cpp.
inline Mat34 buildPadNormalMap(const std::array<Vec3, 4> & pad_pos)
{
  Mat34 g = Mat34::Zero();
  for (int i = 0; i < 4; ++i) {
    g(0, i) = 1.0;
    g(1, i) = -pad_pos[i].z();
    g(2, i) = pad_pos[i].y();
  }
  return g;
}

// Damped Moore-Penrose pseudoinverse G^dagger = G^T(GG^T + lambda_reg*I_3)^-1 in R^{4x3}. G is
// full row-rank (3) by construction for any non-degenerate 4-pad rhombic layout, so lambda_reg
// is purely a numerical-conditioning safety margin, exactly like buildThrusterAllocationPseudoInverse.
inline Mat43 buildPadNormalMapPseudoInverse(const Mat34 & g, double lambda_reg = 1e-6)
{
  const Mat3 gram = g * g.transpose() + lambda_reg * Mat3::Identity();
  return g.transpose() * gram.inverse();
}

// Quasi-static compliant contact map A_contact = -G^dagger * B_[Fx,Ty,Tz] in R^{4x8}
// (CLAUDE.md Sec.3.3/Sec.5): F_N_hat ~= A_contact * u estimates the 4 pad normal forces
// produced by a thruster command u, without needing the real contact-force sensors. B's row
// order is [Fx, Fy, Fz, Tx, Ty, Tz] (buildThrusterAllocationMatrix), so rows {0, 4, 5} select
// [Fx, Ty, Tz].
inline Mat48 buildContactMap(const Mat43 & g_pinv, const Mat68 & b)
{
  Mat38 b_sub;
  b_sub.row(0) = b.row(0);
  b_sub.row(1) = b.row(4);
  b_sub.row(2) = b.row(5);
  return -g_pinv * b_sub;
}

// Half-spans (Ly, Lz) of the Rhombic Kern (CLAUDE.md Sec.4.4), derived directly from pad_pos so
// nothing here is hardcoded independently of the model's actual pad geometry.
inline std::pair<double, double> padHalfSpans(const std::array<Vec3, 4> & pad_pos)
{
  double l_y = 0.0;
  double l_z = 0.0;
  for (const auto & p : pad_pos) {
    l_y = std::max(l_y, std::abs(p.y()));
    l_z = std::max(l_z, std::abs(p.z()));
  }
  return {l_y, l_z};
}

// Rhombic Kern 4-edge constraint matrix C_kern in R^{4x8} (CLAUDE.md Sec.5): for each of the 4
// sign combinations (s_y, s_z) in {+1,-1}x{+1,-1}, row m enforces
// ((s_y/Ly)*b_Tz^T - (s_z/Lz)*b_Ty^T + (1-eps_margin)*b_Fx^T) * u <= 0, i.e. C_kern*u <= 0 keeps
// the center of pressure strictly inside the (1-eps_margin)-shrunk rhombic kern.
inline Mat48 buildKernConstraintMatrix(
  const Mat68 & b, double l_y, double l_z, double eps_margin)
{
  const Eigen::Matrix<double, 1, 8> b_fx = b.row(0);
  const Eigen::Matrix<double, 1, 8> b_ty = b.row(4);
  const Eigen::Matrix<double, 1, 8> b_tz = b.row(5);
  constexpr std::array<std::pair<double, double>, 4> kSigns = {
    {{1.0, 1.0}, {1.0, -1.0}, {-1.0, 1.0}, {-1.0, -1.0}}};

  Mat48 c = Mat48::Zero();
  for (size_t m = 0; m < kSigns.size(); ++m) {
    const double s_y = kSigns[m].first;
    const double s_z = kSigns[m].second;
    c.row(m) = (s_y / l_y) * b_tz - (s_z / l_z) * b_ty + (1.0 - eps_margin) * b_fx;
  }
  return c;
}

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__THRUSTERALLOCATOR_H_
