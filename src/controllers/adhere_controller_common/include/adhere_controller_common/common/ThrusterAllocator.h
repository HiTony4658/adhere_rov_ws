#ifndef ADHERE_CONTROLLER_COMMON__COMMON__THRUSTERALLOCATOR_H_
#define ADHERE_CONTROLLER_COMMON__COMMON__THRUSTERALLOCATOR_H_

#include <cmath>

#include "adhere_controller_common/common/mathTypes.h"

namespace adhere_controller_common
{

// Exact 6x8 body-frame wrench allocation matrix B for rov_wall.xml's 8 thrusters (CLAUDE.md
// Phase 3.2 Stage 2.2), shared by every FSMState that allocates a body wrench onto the 8
// thrusters (StateDepthHold, StateOpenLoop, StatePidHold). Derived directly from the real MJCF
// geometry, NOT from CLAUDE.md Sec.4.3's illustrative X=-0.10m lever-arm placeholder constants,
// which do not match rov_wall.xml (see phase2_status_and_decisions memory / PROGRESS.md): all 8
// thruster sites sit at body X=0.0, the SAME X as the CoM, so there is zero lever arm and
// therefore zero parasitic pitch/yaw coupling from the tangential thrusters.
//
// Row order: [Fx, Fy, Fz, Tx, Ty, Tz]^T (body frame). Column order: thr_N1..thr_N4, thr_T5..
// thr_T8 (matches adhere_mujoco_hardware::kThrusterNames).
//
// Normal thrusters N1..N4: site at body (0, y_i, z_i), gear "-1 0 0 0 0 0" -> force -X_B * u_i.
// Cross product r x F with r=(0,y_i,z_i), F=(-u,0,0) gives (0, -z_i*u, y_i*u), i.e. Ty=-z_i*u,
// Tz=y_i*u.
//
// Tangential thrusters T5..T8: site at the same 4 corners, local +Z axis rotated to
// (0, -+cos30, +-sin30) per their MJCF <site zaxis="...">, gear "0 0 1 0 0 0" -> force
// u*(0, dy_i, dz_i). Cross product r x F with r=(0,y_i,z_i) gives
// (u*(y_i*dz_i - z_i*dy_i), 0, 0), i.e. Tx=u*(y_i*dz_i - z_i*dy_i), Ty=Tz=0 (r_x=0 kills the
// other two rows).
inline Mat68 buildThrusterAllocationMatrix()
{
  const double c30 = std::sqrt(3.0) / 2.0;  // cos(30 deg)
  const double s30 = 0.5;                   // sin(30 deg)

  // Corner order fixed to [TL, TR, BL, BR] = [N1/T5, N2/T6, N3/T7, N4/T8].
  const double y_corner[4] = {0.25, -0.25, 0.25, -0.25};
  const double z_corner[4] = {0.15, 0.15, -0.15, -0.15};
  // Tangential thrust direction (dy, dz) per corner, from rov_wall.xml's thr_T5..T8 zaxis.
  const double dy_corner[4] = {-c30, c30, -c30, c30};
  const double dz_corner[4] = {s30, s30, -s30, -s30};

  Mat68 b = Mat68::Zero();
  for (int i = 0; i < 4; ++i) {
    // Normal thruster column i.
    b(0, i) = -1.0;
    b(4, i) = -z_corner[i];
    b(5, i) = y_corner[i];

    // Tangential thruster column 4+i.
    const int col = 4 + i;
    b(1, col) = dy_corner[i];
    b(2, col) = dz_corner[i];
    b(3, col) = y_corner[i] * dz_corner[i] - z_corner[i] * dy_corner[i];
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

}  // namespace adhere_controller_common

#endif  // ADHERE_CONTROLLER_COMMON__COMMON__THRUSTERALLOCATOR_H_
