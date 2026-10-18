#include "adhere_controller/algorithms/pid/AdherePidController.h"

#include <algorithm>
#include <cmath>

#include "adhere_controller/common/mathTools.h"

namespace adhere_controller
{

double AdherePidController::computeNormalForceWrench(
  double f_des, double sum_f_meas, double vx_body, double dt)
{
  const double error = f_des - sum_f_meas;

  // Conditional anti-windup (same pattern as SixDofPidController::integrateConditional): freeze
  // the accumulator once it is already at its clamp and the new error would push it further
  // past that clamp, instead of letting it wind up arbitrarily far before clamping the output.
  const double next = integral_force_ + error * dt;
  const bool already_saturated = std::abs(integral_force_) >= gains_.integral_force_max;
  const bool pushing_further = (next > integral_force_) == (integral_force_ > 0.0);
  if (!(already_saturated && pushing_further)) {
    integral_force_ = saturation(next, -gains_.integral_force_max, gains_.integral_force_max);
  }

  const double a = gains_.f_preload + gains_.kp_force * error + gains_.ki_force * integral_force_;
  return -a - gains_.kd_x * std::max(0.0, vx_body);
}

}  // namespace adhere_controller
