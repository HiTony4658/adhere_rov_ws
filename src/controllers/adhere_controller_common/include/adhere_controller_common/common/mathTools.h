#ifndef ADHERE_CONTROLLER_COMMON__COMMON__MATHTOOLS_H_
#define ADHERE_CONTROLLER_COMMON__COMMON__MATHTOOLS_H_

#include <algorithm>
#include <cmath>

#include "adhere_controller_common/common/mathTypes.h"

namespace adhere_controller_common
{

// Hamilton quaternion (w, x, y, z), body -> world, same convention MuJoCo's qpos[3:7] and
// adhere_mujoco_node's TF/Odom publishing already use.
inline RotMat quatToRotMat(double w, double x, double y, double z)
{
  RotMat r;
  r << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y);
  return r;
}

inline RotMat quatToRotMat(const Quat & q) { return quatToRotMat(q(0), q(1), q(2), q(3)); }

// Extrinsic ZYX (yaw-pitch-roll) Euler angle extraction, returned as (roll, pitch, yaw) about
// body (X, Y, Z) respectively. Only used here for small-angle attitude damping/stabilization,
// so the pitch = +-90 deg gimbal-lock singularity is not specially handled.
inline Vec3 rotMatToRPY(const RotMat & r)
{
  Vec3 rpy;
  rpy(0) = std::atan2(r(2, 1), r(2, 2));
  rpy(1) = std::atan2(-r(2, 0), std::sqrt(r(2, 1) * r(2, 1) + r(2, 2) * r(2, 2)));
  rpy(2) = std::atan2(r(1, 0), r(0, 0));
  return rpy;
}

template <typename T>
inline T saturation(T value, T min, T max)
{
  return std::clamp(value, min, max);
}

}  // namespace adhere_controller_common

#endif  // ADHERE_CONTROLLER_COMMON__COMMON__MATHTOOLS_H_
