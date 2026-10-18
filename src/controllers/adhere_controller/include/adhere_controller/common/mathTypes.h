#ifndef ADHERE_CONTROLLER__COMMON__MATHTYPES_H_
#define ADHERE_CONTROLLER__COMMON__MATHTYPES_H_

#include <Eigen/Dense>

namespace adhere_controller
{

using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec4 = Eigen::Matrix<double, 4, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec8 = Eigen::Matrix<double, 8, 1>;
using Quat = Eigen::Matrix<double, 4, 1>;  // w, x, y, z

using Mat3 = Eigen::Matrix<double, 3, 3>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat68 = Eigen::Matrix<double, 6, 8>;
using Mat86 = Eigen::Matrix<double, 8, 6>;
using RotMat = Eigen::Matrix<double, 3, 3>;

// Pre-MPPI contact/Kern matrices (CLAUDE.md Phase 3.6 Stage 2.2): G (pad-normal-force ->
// [Fx,Ty,Tz] map), its pseudoinverse, the resulting A_contact, and the Rhombic Kern 4-edge
// constraint matrix C_kern.
using Mat34 = Eigen::Matrix<double, 3, 4>;
using Mat43 = Eigen::Matrix<double, 4, 3>;
using Mat38 = Eigen::Matrix<double, 3, 8>;
using Mat48 = Eigen::Matrix<double, 4, 8>;

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__MATHTYPES_H_
