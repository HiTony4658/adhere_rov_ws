#ifndef ADHERE_CONTROLLER__COMMON__MATHTOOLS_H_
#define ADHERE_CONTROLLER__COMMON__MATHTOOLS_H_

#include <algorithm>
#include <cmath>

#include "adhere_controller/common/mathTypes.h"

namespace adhere_controller
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

// Inverse of rotMatToRPY() above: builds R = Rz(yaw)*Ry(pitch)*Rx(roll) from the same
// extrinsic-ZYX (roll, pitch, yaw) convention. Used wherever a state is driven from
// CtrlInterfaces::ActiveState's rpy[3] (Phase 3.5) instead of a raw quaternion.
inline RotMat rpyToRotMat(const Vec3 & rpy)
{
  const double sr = std::sin(rpy(0));
  const double cr = std::cos(rpy(0));
  const double sp = std::sin(rpy(1));
  const double cp = std::cos(rpy(1));
  const double sy = std::sin(rpy(2));
  const double cy = std::cos(rpy(2));
  RotMat r;
  r << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr;
  return r;
}

template <typename T>
inline T saturation(T value, T min, T max)
{
  return std::clamp(value, min, max);
}

// Hamilton quaternion product a (x) b (w, x, y, z convention, matches quatToRotMat above).
inline Quat quatMul(const Quat & a, const Quat & b)
{
  Quat out;
  out(0) = a(0) * b(0) - a(1) * b(1) - a(2) * b(2) - a(3) * b(3);
  out(1) = a(0) * b(1) + a(1) * b(0) + a(2) * b(3) - a(3) * b(2);
  out(2) = a(0) * b(2) - a(1) * b(3) + a(2) * b(0) + a(3) * b(1);
  out(3) = a(0) * b(3) + a(1) * b(2) - a(2) * b(1) + a(3) * b(0);
  return out;
}

inline Quat normalizeQuat(const Quat & q)
{
  const double n = q.norm();
  return n > 1e-12 ? (q / n).eval() : Quat(1.0, 0.0, 0.0, 0.0);
}

// Conjugate/inverse of a unit quaternion (w, -x, -y, -z).
inline Quat quatInverse(const Quat & q) { return Quat(q(0), -q(1), -q(2), -q(3)); }

// Small-angle exponential-map increment dq ~ [1, 0.5*theta] for a rotation vector theta = omega*dt,
// renormalized. Used both by RovStateEstimator's gyro-integration prediction step and by
// StatePidHold's teleop-driven quaternion setpoint integration (Phase 3.6).
inline Quat smallAngleQuat(const Vec3 & theta)
{
  Quat dq;
  dq << 1.0, 0.5 * theta.x(), 0.5 * theta.y(), 0.5 * theta.z();
  return normalizeQuat(dq);
}

// Builds a unit quaternion from the same extrinsic-ZYX (roll, pitch, yaw) convention as
// rpyToRotMat() above (R = Rz(yaw)*Ry(pitch)*Rx(roll)).
inline Quat quatFromRPY(const Vec3 & rpy)
{
  const double hr = 0.5 * rpy(0);
  const double hp = 0.5 * rpy(1);
  const double hy = 0.5 * rpy(2);
  const double cr = std::cos(hr);
  const double sr = std::sin(hr);
  const double cp = std::cos(hp);
  const double sp = std::sin(hp);
  const double cy = std::cos(hy);
  const double sy = std::sin(hy);
  Quat q;
  q(0) = cr * cp * cy + sr * sp * sy;
  q(1) = sr * cp * cy - cr * sp * sy;
  q(2) = cr * sp * cy + sr * cp * sy;
  q(3) = cr * cp * sy - sr * sp * cy;
  return normalizeQuat(q);
}

// Minimal-rotation SO(3) attitude error (CLAUDE.md Phase 3.6 Sec.2.1): q_e = q_est^-1 (x) q_d,
// e_R = 2*sgn(q_e.w)*[q_e.x, q_e.y, q_e.z]. Singularity-free alternative to an Euler-angle
// difference, used by StatePidHold's quaternion attitude PD.
inline Vec3 quatError(const Quat & q_est, const Quat & q_d)
{
  const Quat q_e = quatMul(quatInverse(q_est), q_d);
  const double sign = q_e(0) < 0.0 ? -1.0 : 1.0;
  return Vec3(2.0 * sign * q_e(1), 2.0 * sign * q_e(2), 2.0 * sign * q_e(3));
}

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__MATHTOOLS_H_
