#ifndef ADHERE_CONTROLLER__COMMON__CONTROLLERMETRICS_H_
#define ADHERE_CONTROLLER__COMMON__CONTROLLERMETRICS_H_

#include <array>
#include <cmath>
#include <cstdint>

#include "adhere_controller/dynamics/ModelParams.h"

namespace adhere_controller
{

// Centralized efficiency/chattering/safety telemetry (CLAUDE.md Phase 3.8), computed once per
// 250Hz inner-loop tick by AdhereController::update() from the actually-sent (sanitized +
// saturated) thruster commands, then published at the outer-loop rate on
// ControllerState.msg. Header-only, ROS-free (same style as OutputSafety.h/LoopProfiler.h), so
// it is independently gtest-able without a live ControllerInterface node.
struct ControllerMetricsSample
{
  double thrust_l2_norm = 0.0;
  double thrust_power_proxy = 0.0;
  double thrust_slew_rate_l2 = 0.0;
  double cumulative_energy_l2 = 0.0;
  double cumulative_impulse_l1 = 0.0;
  uint8_t saturated_thruster_count = 0;
  uint8_t active_contact_pad_count = 0;
};

class ControllerMetricsAccumulator
{
public:
  // Called on every FSM state transition (CLAUDE.md Phase 3.8): zeroes the cumulative
  // integrators AND clears the "have a previous sample" flag, so the very first post-reset
  // sample reports thrust_slew_rate_l2=0 instead of a spurious slew artifact computed against
  // the previous state's last commanded thruster vector.
  void reset()
  {
    has_prev_ = false;
    prev_u_.fill(0.0);
    cumulative_energy_l2_ = 0.0;
    cumulative_impulse_l1_ = 0.0;
  }

  // u: the 8 actually-sent (post sanitize+saturate) thruster commands (N); contact_forces: the 4
  // estimated pad normal forces (N, via A_contact*u); dt: inner-loop period (s).
  ControllerMetricsSample update(
    const std::array<double, 8> & u, const std::array<double, 4> & contact_forces,
    const ModelParams & params, double dt)
  {
    ControllerMetricsSample out;

    double sum_sq = 0.0;
    double power_proxy = 0.0;
    double l1 = 0.0;
    uint8_t saturated_count = 0;
    for (int i = 0; i < 8; ++i) {
      sum_sq += u[i] * u[i];
      power_proxy += std::pow(std::abs(u[i]), 1.5);
      l1 += std::abs(u[i]);
      if (u[i] >= 0.98 * params.thruster_max[i] || u[i] <= 0.98 * params.thruster_min[i]) {
        ++saturated_count;
      }
    }
    out.thrust_l2_norm = std::sqrt(sum_sq);
    out.thrust_power_proxy = power_proxy;
    out.saturated_thruster_count = saturated_count;

    if (has_prev_) {
      double slew_sum_sq = 0.0;
      for (int i = 0; i < 8; ++i) {
        const double d = (u[i] - prev_u_[i]) / dt;
        slew_sum_sq += d * d;
      }
      out.thrust_slew_rate_l2 = std::sqrt(slew_sum_sq);
    } else {
      out.thrust_slew_rate_l2 = 0.0;
    }

    cumulative_energy_l2_ += sum_sq * dt;
    cumulative_impulse_l1_ += l1 * dt;
    out.cumulative_energy_l2 = cumulative_energy_l2_;
    out.cumulative_impulse_l1 = cumulative_impulse_l1_;

    uint8_t active_pads = 0;
    for (int i = 0; i < 4; ++i) {
      if (contact_forces[i] > params.contact_force_thresh) {
        ++active_pads;
      }
    }
    out.active_contact_pad_count = active_pads;

    prev_u_ = u;
    has_prev_ = true;
    return out;
  }

private:
  bool has_prev_ = false;
  std::array<double, 8> prev_u_ = {};
  double cumulative_energy_l2_ = 0.0;
  double cumulative_impulse_l1_ = 0.0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__CONTROLLERMETRICS_H_
