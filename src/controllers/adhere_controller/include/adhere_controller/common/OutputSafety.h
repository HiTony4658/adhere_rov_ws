#ifndef ADHERE_CONTROLLER__COMMON__OUTPUTSAFETY_H_
#define ADHERE_CONTROLLER__COMMON__OUTPUTSAFETY_H_

#include <cmath>
#include <cstdint>

#include "adhere_controller/common/mathTools.h"
#include "adhere_controller/CtrlInterfaces.h"
#include "adhere_controller/dynamics/ModelParams.h"

namespace adhere_controller
{

// Centralized output safety gate (CLAUDE.md Phase 3.7): the single place
// AdhereController::update() writes ci.pending_thruster_cmd[0..7] out to hardware. If any of
// the 8 commands is NaN/Inf, ALL 8 thrusters are zeroed (never a partial/mixed write) and
// `violation_count` is incremented so the caller can log/report it; otherwise each command is
// clamped to [thruster_min[i], thruster_max[i]] before being written. Header-only and
// ROS-logger-free (no RCLCPP_ERROR here) so it is trivially reusable from gtests -- the call
// site owns throttled error logging.
inline void sanitizeAndWriteThrusterCommands(
  CtrlInterfaces & ci, const ModelParams & params, uint64_t & violation_count)
{
  bool all_finite = true;
  for (int i = 0; i < 8; ++i) {
    if (!std::isfinite(ci.pending_thruster_cmd[i])) {
      all_finite = false;
      break;
    }
  }

  if (!all_finite) {
    ++violation_count;
    for (int i = 0; i < 8; ++i) {
      ci.thruster_command_interface_[i].get().set_value(0.0);
    }
    return;
  }

  for (int i = 0; i < 8; ++i) {
    ci.thruster_command_interface_[i].get().set_value(
      saturation(ci.pending_thruster_cmd[i], params.thruster_min[i], params.thruster_max[i]));
  }
}

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__OUTPUTSAFETY_H_
