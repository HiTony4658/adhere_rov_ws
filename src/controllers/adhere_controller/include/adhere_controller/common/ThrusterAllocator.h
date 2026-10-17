#ifndef ADHERE_CONTROLLER__COMMON__THRUSTERALLOCATOR_H_
#define ADHERE_CONTROLLER__COMMON__THRUSTERALLOCATOR_H_

#include <array>

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

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__THRUSTERALLOCATOR_H_
