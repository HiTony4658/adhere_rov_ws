#ifndef ADHERE_CONTROLLER__COMMON__MULTIRATEDECIMATOR_H_
#define ADHERE_CONTROLLER__COMMON__MULTIRATEDECIMATOR_H_

#include <cstdint>
#include <string>
#include <vector>

namespace adhere_controller
{

// Named-channel integer-ratio multi-rate decimator (CLAUDE.md Phase 3.1.1, generalized to a
// registry in Phase 5.0 Stage 3): every rate derived from the 500Hz controller_manager/
// AdhereMujocoHardware master clock -- the 250Hz inner loop, the 25Hz outer loop, the 10Hz
// diagnostics publish, the (nominally every-tick) estimator cadence -- registers one named
// channel here instead of each owning its own ad-hoc modulo counter. tick() reports, in
// registration order, which channels are due on this physics tick with zero jitter: every
// channel's period is exactly decimation/master_rate_hz seconds, so discrete integrators/
// derivatives/low-pass filters (the Pad-Force DOB, PID integral terms, LoopProfiler EMAs) never
// see non-uniform substep alternation.
class MultiRateDecimator
{
public:
  explicit MultiRateDecimator(int master_rate_hz)
  : master_rate_hz_(master_rate_hz)
  {
  }

  // Registers a new channel firing once every `decimation` master ticks and returns its stable
  // index (used to read tick()'s result and query periodSeconds()). Call only during
  // configuration (before the 500Hz hot loop starts calling tick()) -- not thread-safe against
  // concurrent tick() calls.
  int addChannel(const std::string & name, int decimation)
  {
    channels_.push_back(Channel{name, decimation});
    return static_cast<int>(channels_.size()) - 1;
  }

  // Advances the tick counter and reports, in registration order, whether each channel is due
  // on this tick. A channel with decimation N is due on master ticks 0, N, 2N, ... so every
  // channel is simultaneously due on tick 0 (and after reset()).
  std::vector<bool> tick()
  {
    const uint64_t t = tick_count_++;
    std::vector<bool> due(channels_.size());
    for (size_t i = 0; i < channels_.size(); ++i) {
      due[i] = (t % static_cast<uint64_t>(channels_[i].decimation)) == 0;
    }
    return due;
  }

  double periodSeconds(int channel_id) const
  {
    return static_cast<double>(channels_[channel_id].decimation) /
           static_cast<double>(master_rate_hz_);
  }

  int decimation(int channel_id) const { return channels_[channel_id].decimation; }

  const std::string & name(int channel_id) const { return channels_[channel_id].name; }

  size_t channelCount() const { return channels_.size(); }

  // Zeroes the tick counter so a deactivate/reactivate cycle always begins every channel's
  // schedule aligned on tick 0 (all simultaneously due), instead of carrying over a phase offset
  // from the previous activation.
  void reset() { tick_count_ = 0; }

private:
  struct Channel
  {
    std::string name;
    int decimation;
  };

  int master_rate_hz_;
  std::vector<Channel> channels_;
  uint64_t tick_count_ = 0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__MULTIRATEDECIMATOR_H_
