#ifndef ADHERE_CONTROLLER__COMMON__MULTIRATEDECIMATOR_H_
#define ADHERE_CONTROLLER__COMMON__MULTIRATEDECIMATOR_H_

#include <cstdint>

namespace adhere_controller
{

// Uniform integer-ratio multi-rate tick decimator (CLAUDE.md Phase 3.1.1): replaces the former
// non-uniform [3,3,3,3,3,2,2,2,2,2] alternating substep schedule with exact modulo decimation
// of the 500Hz controller_manager/AdhereMujocoHardware master clock, so every inner-loop period
// is identically cm_update_rate/inner_decimation seconds with zero jitter -- important for
// discrete integrators/derivatives/low-pass filters such as the Pad-Force DOB and the PID
// integral term in StateDepthHold.
class MultiRateDecimator
{
public:
  MultiRateDecimator(int cm_update_rate, int inner_decimation, int outer_decimation)
  : cm_update_rate_(cm_update_rate),
    inner_decimation_(inner_decimation),
    outer_decimation_(outer_decimation)
  {
  }

  struct TickResult
  {
    bool inner_due;
    bool outer_due;
  };

  // Advances the tick counter and reports which loop(s) are due on this physics tick.
  // outer_decimation_ is always an integer multiple of inner_decimation_ by construction
  // (CLAUDE.md Sec.3), so outer_due == true implies inner_due == true.
  TickResult tick()
  {
    const uint64_t t = tick_count_++;
    return TickResult{
      t % static_cast<uint64_t>(inner_decimation_) == 0,
      t % static_cast<uint64_t>(outer_decimation_) == 0};
  }

  double innerPeriodSeconds() const
  {
    return static_cast<double>(inner_decimation_) / static_cast<double>(cm_update_rate_);
  }

  double outerPeriodSeconds() const
  {
    return static_cast<double>(outer_decimation_) / static_cast<double>(cm_update_rate_);
  }

  // Zeroes the tick counter so a deactivate/reactivate cycle always begins its inner/outer
  // schedule aligned on tick 0 (both due simultaneously), instead of carrying over a phase
  // offset from the previous activation.
  void reset() { tick_count_ = 0; }

private:
  int cm_update_rate_;
  int inner_decimation_;
  int outer_decimation_;
  uint64_t tick_count_ = 0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__MULTIRATEDECIMATOR_H_
