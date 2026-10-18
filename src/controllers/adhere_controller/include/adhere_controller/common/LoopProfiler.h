#ifndef ADHERE_CONTROLLER__COMMON__LOOPPROFILER_H_
#define ADHERE_CONTROLLER__COMMON__LOOPPROFILER_H_

#include <algorithm>
#include <chrono>
#include <cstdint>

namespace adhere_controller
{

// Lightweight real-time frequency/latency instrumentation (CLAUDE.md Phase 3.6 Stage 1.2):
// header-only, no ROS/MuJoCo dependency (same style as MultiRateDecimator.h/
// RovStateEstimator.h), so it is usable from both AdhereController (inner/outer loop, estimator,
// edge-detected sensor sample rates) and AdhereMujocoHardware (raw physics-step cadence) without
// either package depending on the other. Two independent concerns, tracked separately:
//   - tickMarker(): call once per logical "trigger" (a new sensor sample, a loop iteration) to
//     measure the achieved trigger frequency via an EMA of the inter-call period.
//   - beginExec()/endExec(): bracket the actual work done on that trigger to measure latest/
//     EMA-mean/max execution time and count overruns against an optional budget_ms.
class LoopProfiler
{
public:
  // budget_ms <= 0.0 disables overrun counting (e.g. for sensor-only profilers that only ever
  // call tickMarker(), never beginExec()/endExec()).
  explicit LoopProfiler(double budget_ms = 0.0) : budget_ms_(budget_ms) {}

  void tickMarker()
  {
    const auto now = std::chrono::steady_clock::now();
    if (have_last_tick_) {
      const double dt = std::chrono::duration<double>(now - last_tick_).count();
      if (dt > 0.0) {
        const double hz_inst = 1.0 / dt;
        hz_ema_ = have_hz_ ? (1.0 - kEmaAlpha) * hz_ema_ + kEmaAlpha * hz_inst : hz_inst;
        have_hz_ = true;
      }
    }
    last_tick_ = now;
    have_last_tick_ = true;
  }

  void beginExec()
  {
    exec_start_ = std::chrono::steady_clock::now();
    have_exec_start_ = true;
  }

  void endExec()
  {
    if (!have_exec_start_) {
      return;
    }
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - exec_start_).count();
    latest_ms_ = ms;
    mean_ms_ = have_mean_ ? (1.0 - kEmaAlpha) * mean_ms_ + kEmaAlpha * ms : ms;
    have_mean_ = true;
    max_ms_ = std::max(max_ms_, ms);
    if (budget_ms_ > 0.0 && ms > budget_ms_) {
      ++overrun_count_;
    }
    have_exec_start_ = false;
  }

  double hz() const { return hz_ema_; }
  double latestExecMs() const { return latest_ms_; }
  double meanExecMs() const { return mean_ms_; }
  double maxExecMs() const { return max_ms_; }
  double budgetMs() const { return budget_ms_; }
  uint64_t overruns() const { return overrun_count_; }

  void reset()
  {
    have_last_tick_ = false;
    hz_ema_ = 0.0;
    have_hz_ = false;
    have_exec_start_ = false;
    latest_ms_ = 0.0;
    mean_ms_ = 0.0;
    have_mean_ = false;
    max_ms_ = 0.0;
    overrun_count_ = 0;
  }

private:
  // ~20-sample EMA time constant: smooths tick-to-tick jitter without lagging too far behind a
  // genuine frequency/latency shift (e.g. an estimator correction suddenly getting expensive).
  static constexpr double kEmaAlpha = 0.05;

  double budget_ms_;

  std::chrono::steady_clock::time_point last_tick_{};
  bool have_last_tick_ = false;
  double hz_ema_ = 0.0;
  bool have_hz_ = false;

  std::chrono::steady_clock::time_point exec_start_{};
  bool have_exec_start_ = false;
  double latest_ms_ = 0.0;
  double mean_ms_ = 0.0;
  bool have_mean_ = false;
  double max_ms_ = 0.0;
  uint64_t overrun_count_ = 0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__LOOPPROFILER_H_
