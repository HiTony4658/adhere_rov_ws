#include "adhere_controller/common/LoopProfiler.h"

#include <chrono>
#include <thread>

#include <gtest/gtest.h>

using adhere_controller::LoopProfiler;

// CLAUDE.md Phase 3.6 Stage 3.2: tickMarker() measures the achieved trigger frequency via an
// EMA of the real inter-call period. Sleeping ~4ms between ticks should converge the EMA toward
// ~250Hz (not exact, since std::this_thread::sleep_for has scheduler jitter, hence the loose
// tolerance).
TEST(LoopProfiler, TickMarkerConvergesTowardConfiguredRate)
{
  LoopProfiler profiler;
  for (int i = 0; i < 60; ++i) {
    profiler.tickMarker();
    std::this_thread::sleep_for(std::chrono::microseconds(4000));
  }
  EXPECT_NEAR(profiler.hz(), 250.0, 40.0);
}

TEST(LoopProfiler, FirstTickMarkerProducesNoFrequencyYet)
{
  LoopProfiler profiler;
  profiler.tickMarker();
  EXPECT_DOUBLE_EQ(profiler.hz(), 0.0);
}

// beginExec()/endExec() latency measurement and overrun counting against budget_ms.
TEST(LoopProfiler, ExecTimingTracksLatestMeanMaxAndOverruns)
{
  LoopProfiler profiler(/*budget_ms=*/1.0);

  profiler.beginExec();
  std::this_thread::sleep_for(std::chrono::microseconds(500));
  profiler.endExec();
  EXPECT_GT(profiler.latestExecMs(), 0.0);
  EXPECT_LT(profiler.latestExecMs(), 1.0);
  EXPECT_EQ(profiler.overruns(), 0u);

  profiler.beginExec();
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  profiler.endExec();
  EXPECT_GT(profiler.latestExecMs(), 1.0);
  EXPECT_EQ(profiler.overruns(), 1u);
  EXPECT_GE(profiler.maxExecMs(), profiler.latestExecMs() - 1e-6);
}

TEST(LoopProfiler, ResetClearsAllAccumulatedState)
{
  LoopProfiler profiler(/*budget_ms=*/0.1);
  profiler.tickMarker();
  profiler.tickMarker();
  profiler.beginExec();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  profiler.endExec();
  EXPECT_GT(profiler.overruns(), 0u);

  profiler.reset();
  EXPECT_DOUBLE_EQ(profiler.hz(), 0.0);
  EXPECT_DOUBLE_EQ(profiler.latestExecMs(), 0.0);
  EXPECT_DOUBLE_EQ(profiler.maxExecMs(), 0.0);
  EXPECT_EQ(profiler.overruns(), 0u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
